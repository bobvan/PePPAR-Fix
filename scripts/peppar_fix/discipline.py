"""M7 adaptive discipline interval scheduler.

Decides when to apply a servo correction based on two physical signals
plus a hard phase-error budget:

1. **σ_obs** (input side): noise floor of the error signal — slope of a
   linear fit through the rolling error_ns history, residual std after
   removing that slope.  Tells us the per-sample observation noise.
2. **D_physical** (output side, *long*-window slope of adjfine_ppb):
   captures the open-loop DO frequency drift.  Math: adjfine = −drift
   when the loop is locked, so the long-window slope of adjfine is
   approximately the magnitude of the physical drift rate.  Distinct
   from the old EMA-of-|Δadjfine|, which mixed loop noise with drift.
3. **T_budget**: hard cap on tolerable phase error before forced
   correction.

The scheduling rule:

    τ = √(2 · T_budget / D_physical)        # drift-budget upper bound
    τ ← clamp(τ, min_interval, max_interval)

Plus the hard floor: should_correct() returns True whenever the latest
buffered |error_ns| exceeds T_budget, regardless of τ.  This catches
transient excursions without waiting for the next scheduled flush.

Why both signals:

- Input-side alone (the previous design) saw only the post-correction
  residual.  When the loop was tight, residual drift looked quiet → τ
  grew → physical DO drift surfaced → chA TDEV(1s) degraded 40×.
- Output-side EMA-of-|Δadjfine| (the original design) conflated
  correction noise with drift → kept τ pinned at 1 even on calm DOs.
- Long-window slope of adjfine captures the *signed* trend, which on
  timescales > 1 minute reflects only physical aging/thermal drift —
  high-frequency control content averages away.

Replaced 2026-05-22 (schedulerCombinedDriftEstimator) after the
input-side-only design (schedulerInputSideDriftRate) was found
insufficient via PiFace lab validation.
"""

import logging
import math
from collections import deque

log = logging.getLogger("peppar_fix.discipline")

# Minimum samples in the rolling error window before we trust the
# slope estimator.  Below this the linear fit's covariance is poor;
# fall back to base_interval.
_MIN_FIT_SAMPLES = 30


class DisciplineScheduler:
    """Accumulates error samples and decides when to apply a correction.

    M7: instead of correcting every epoch, buffer N samples and apply one
    averaged correction. This reduces correction jitter while preserving
    tracking bandwidth.
    """

    def __init__(self, base_interval=1, adaptive=False,
                 min_interval=1, max_interval=120,
                 converge_threshold_ns=100.0, settle_window=10,
                 unconverge_factor=5.0,
                 error_history_len=300,
                 adjfine_history_len=300,
                 phase_error_budget_ns=1.0):
        self.base_interval = base_interval
        self.adaptive = adaptive
        self.min_interval = min_interval
        self.max_interval = max_interval
        self.interval = base_interval
        self.phase_error_budget_ns = float(phase_error_budget_ns)

        self._errors = []
        self._confidences = []
        self._sources = []

        # Input-side drift+σ estimators.  drift_rate is the linear slope
        # (ns/s) of the running error window; σ_obs is the residual
        # standard deviation.  Both are computed in
        # _update_drift_rate_from_input() called by compute_adaptive_interval().
        self._drift_rate_ns_per_s = 0.0  # slope of error history
        self._sigma_obs_ns = 0.0         # residual std
        self._error_history = deque(maxlen=error_history_len)

        # Output-side physical-drift estimator.  Long-window slope of
        # adjfine over the rolling history → |D_physical| in ppb/s.
        # Caller updates via record_actuation() after each servo write.
        self._d_physical_ppb_per_s = 0.0
        self._adjfine_history = deque(maxlen=adjfine_history_len)

        # Retained for back-compat with callers that still invoke
        # update_drift_rate(timestamp, adjfine_ppb).  No longer used in
        # adaptive interval computation.
        self._legacy_adjfine_seen = False

        self._converge_threshold = converge_threshold_ns
        self._settled_count = 0
        self._settle_window = settle_window
        self._unconverge_factor = unconverge_factor
        self._converging = True

    @property
    def n_accumulated(self):
        return len(self._errors)

    @property
    def drift_rate_ns_per_s(self):
        """Most recently estimated *residual* drift rate from error history
        (ns/s).  Diagnostic only — no longer used to set τ.  0 before
        bootstrap."""
        return self._drift_rate_ns_per_s

    @property
    def sigma_obs_ns(self):
        """Most recently estimated input observation σ (ns).  Diagnostic.
        0 before bootstrap."""
        return self._sigma_obs_ns

    @property
    def d_physical_ppb_per_s(self):
        """Most recently estimated open-loop physical drift rate of the
        DO (ppb/s), from the long-window slope of adjfine.  This IS
        used to set τ via τ = √(2·T_budget/D_physical).  0 before
        record_actuation() has been called enough times."""
        return self._d_physical_ppb_per_s

    def accumulate(self, error_ns, confidence_ns, source_name,
                   t_monotonic=None):
        """Buffer one error sample.

        ``t_monotonic`` is optional but required for adaptive scheduling
        (we need timestamps to compute drift slope).  When None we fall
        back to importing time.monotonic() here — convenient for unit
        tests and callers that don't have a timestamp at hand.
        """
        self._errors.append(error_ns)
        self._confidences.append(confidence_ns)
        self._sources.append(source_name)
        if t_monotonic is None:
            import time
            t_monotonic = time.monotonic()
        self._error_history.append((float(t_monotonic), float(error_ns)))

    def should_correct(self):
        """True when it's time to flush the buffer and correct."""
        n = len(self._errors)
        if n == 0:
            return False

        # Hard budget cap: if the most recently buffered error already
        # exceeds the tolerated phase budget, fire immediately
        # regardless of interval.  Prevents τ-induced excursions from
        # blowing the budget while waiting for the scheduled flush.
        if abs(self._errors[-1]) > self.phase_error_budget_ns:
            return True

        effective_interval = 1 if self._converging else self.interval

        if n >= effective_interval:
            return True

        if n > 1 and self._sources[-1] != self._sources[0]:
            return True

        return False

    def flush(self):
        """Return averaged error, confidence, and sample count; reset buffer.

        Returns:
            (avg_error_ns, avg_confidence_ns, n_samples)
        """
        if not self._errors:
            return (0.0, 0.0, 0)

        n = len(self._errors)
        avg_error = sum(self._errors) / n
        avg_confidence = sum(self._confidences) / n
        self._errors.clear()
        self._confidences.clear()
        self._sources.clear()

        if self._converging:
            if abs(avg_error) < self._converge_threshold:
                self._settled_count += 1
                if self._settled_count >= self._settle_window:
                    self._converging = False
                    log.info(f"  M7: settled after {self._settled_count} corrections, "
                             f"interval -> {self.base_interval}")
            else:
                self._settled_count = 0
        else:
            if abs(avg_error) > self._converge_threshold * self._unconverge_factor:
                self._converging = True
                self._settled_count = 0
                log.info(f"  M7: error {avg_error:+.0f}ns, back to convergence mode")

        return (avg_error, avg_confidence, n)

    def update_drift_rate(self, timestamp, adjfine_ppb):
        """DEPRECATED no-op.

        Earlier versions used the servo output (``adjfine_ppb``) to
        estimate drift_rate, which produced a circular dependency
        through the control loop.  Drift rate is now computed inside
        compute_adaptive_interval() from the error_ns history (input
        side, actuator-independent).  See schedulerInputSideDriftRate
        in dayplan.

        Kept callable for back-compat with any external code that
        still invokes it; logs once at INFO so the call surfaces.
        """
        if not self._legacy_adjfine_seen:
            log.info("DisciplineScheduler.update_drift_rate(adjfine_ppb=…) "
                     "is deprecated and no longer affects adaptive "
                     "interval — drift estimated from error history "
                     "(schedulerInputSideDriftRate).")
            self._legacy_adjfine_seen = True

    def record_actuation(self, t_monotonic, adjfine_ppb):
        """Append a (t, adjfine_ppb) sample to the actuation history.

        Caller invokes this after each successful actuator.adjust /
        DAC write / FCW write.  The long-window slope of this history
        is the open-loop DO drift rate (because adjfine = -drift while
        the loop is locked).
        """
        self._adjfine_history.append((float(t_monotonic), float(adjfine_ppb)))

    def _update_d_physical_from_adjfine(self):
        """Linear-fit slope of adjfine over the history window.

        Sets self._d_physical_ppb_per_s = |slope|.  Falls back to 0 if
        the window has fewer than _MIN_FIT_SAMPLES entries.

        Why this is *not* the same defect as the old EMA-of-|Δadjfine|:
        a signed long-window slope picks up the secular trend
        (aging/thermal drift) and averages out high-frequency control
        loop content over the same window.  The old EMA computed the
        magnitude of *per-sample* deltas, which is dominated by loop
        noise when τ is small.
        """
        n = len(self._adjfine_history)
        if n < _MIN_FIT_SAMPLES:
            self._d_physical_ppb_per_s = 0.0
            return
        t0 = self._adjfine_history[0][0]
        sx = sy = sxx = sxy = 0.0
        for t, a in self._adjfine_history:
            x = t - t0
            sx += x
            sy += a
            sxx += x * x
            sxy += x * a
        denom = n * sxx - sx * sx
        if denom <= 0:
            self._d_physical_ppb_per_s = 0.0
            return
        slope = (n * sxy - sx * sy) / denom
        self._d_physical_ppb_per_s = abs(slope)

    def _update_drift_rate_from_input(self):
        """Recompute drift_rate and σ_obs from the error history.

        Linear-fits ``error_ns(t)`` over the rolling window:

            error_ns ≈ a + b · t       (slope b = drift_rate ns/s)
            σ_obs = stdev(error_ns - (a + b·t))   (per-sample residual)

        Falls back to (0, 0) when the window has fewer than
        ``_MIN_FIT_SAMPLES`` entries.
        """
        n = len(self._error_history)
        if n < _MIN_FIT_SAMPLES:
            self._drift_rate_ns_per_s = 0.0
            self._sigma_obs_ns = 0.0
            return

        # Use the first timestamp as the origin to keep the fit
        # numerically conditioned (monotonic time can be large).
        t0 = self._error_history[0][0]
        sx = 0.0
        sy = 0.0
        sxx = 0.0
        sxy = 0.0
        for t, e in self._error_history:
            x = t - t0
            sx += x
            sy += e
            sxx += x * x
            sxy += x * e
        denom = n * sxx - sx * sx
        if denom <= 0:
            self._drift_rate_ns_per_s = 0.0
            self._sigma_obs_ns = 0.0
            return
        slope = (n * sxy - sx * sy) / denom
        intercept = (sy - slope * sx) / n

        # Residual std (population stdev — n divisor, not n-1; the
        # bias is tiny for n ≥ 30 and we use the value for a scaling
        # heuristic, not statistical inference).
        ss_res = 0.0
        for t, e in self._error_history:
            r = e - (intercept + slope * (t - t0))
            ss_res += r * r
        sigma = math.sqrt(ss_res / n) if n > 0 else 0.0

        self._drift_rate_ns_per_s = abs(slope)
        self._sigma_obs_ns = sigma

    def compute_adaptive_interval(self, measurement_sigma_ns=None):
        """Compute optimal discipline interval τ from physical drift
        budget.

        τ = √(2 · T_budget_ns / D_physical_ppb_per_s) clamped to
        [min_interval, max_interval].

        Derivation: a DO drifting at D ppb/s has its frequency
        deviation grow as D·t between corrections, accumulating
        phase deviation ∫(D·t)dt = D·t²/2 (in ppb·s = ns).
        Solving D·τ²/2 ≤ T_budget for τ gives τ_max = √(2·T_budget/D).

        Input-side σ_obs / residual-drift are still computed and
        exported as diagnostics; ``measurement_sigma_ns`` arg is
        retained for API back-compat but ignored.
        """
        # Keep diagnostics fresh regardless of adaptive on/off.
        self._update_drift_rate_from_input()
        self._update_d_physical_from_adjfine()

        if not self.adaptive:
            return self.base_interval

        # Bootstrap: D_physical not yet measured (not enough actuations
        # recorded).  Use base_interval until we have data.
        if len(self._adjfine_history) < _MIN_FIT_SAMPLES:
            return self.base_interval

        # Quiet DO (drift below resolution) → τ = max_interval.
        # _MIN_PHYSICAL_DRIFT_PPB_PER_S = 1e-9 is well below any real
        # OCXO drift (~1e-5 ppb/s for aging) and protects against the
        # √(T/0) blowup.
        if self._d_physical_ppb_per_s < 1e-9:
            self.interval = self.max_interval
            return self.max_interval

        # τ from physical drift budget.  Units: T_budget in ns,
        # D in ppb/s = ns/s² (1 ppb = 1 ns/s).  So τ² has units of s².
        tau_sec = math.sqrt(2.0 * self.phase_error_budget_ns
                             / self._d_physical_ppb_per_s)
        tau = max(self.min_interval,
                   min(self.max_interval, int(round(tau_sec))))
        self.interval = tau
        return tau


# ---------------------------------------------------------------------------
# longTauGnssCoupling — coast-cap + graded-taper policy primitives
#
# The deterministic drift budget above (τ = √(2·T/D)) bounds only the
# *signed* aging/thermal ramp.  A quiet DO (D≈0) gets τ → max_interval,
# but its *stochastic* phase wander (random-walk / white-FM) keeps
# accumulating over a long coast — the tau~64-256 s TDEV hump observed
# in the 2026-05-29 overnight runs (PiFace 6.4 ns @ 64 s, MadHat 3.6 ns
# @ 256 s), precisely where the GNSS reference is at its best.
#
# These are pure functions: the building blocks the engine/sim compose
# to (a) cap the coast by the DO's *stochastic* phase growth and (b)
# replace the binary converging/tracking latch with a continuous taper.
# They have no effect on DisciplineScheduler until wired — integration
# is validated in closedLoopServoSim (charlie) before hardware.  The
# sqrt(P22) path additionally requires the honest small Q from freerun
# characterization (qFromCharPerActuator); with an inflated Q the cap
# is spuriously tight.
# ---------------------------------------------------------------------------

def coast_cap_from_tdev(t_budget_ns, tdev_ref_ns, tdev_slope,
                        tau_ref_s=1.0, k_sigma=1.0):
    """Largest coast interval τ (s) whose characterized DO phase wander
    stays within budget, from a power-law TDEV model.

    Models the DO's *freerun* phase wander as

        TDEV(τ) = tdev_ref_ns · (τ / tau_ref_s) ** tdev_slope

    Common slopes: +0.5 white-FM, +1.0 flicker-FM, +1.5 random-walk-FM.
    A slope ≤ 0 (white/flicker-PM, or a flat floor) means the wander
    does not grow with τ → no coast constraint → returns +inf.

    Solves  k_sigma · TDEV(τ) ≤ t_budget_ns  for the largest τ:

        τ_cap = tau_ref_s · (t_budget_ns / (k_sigma·tdev_ref_ns))
                          ** (1 / tdev_slope)

    Returns float seconds (caller clamps to [min,max] and rounds).  Use
    the DO's *characterized freerun* TDEV here, not the disciplined
    output TDEV — the cap answers "how long can this oscillator coast
    open-loop before its own noise floor exceeds budget", which is a
    DO-class property (long for an OCXO, short for a TCXO).
    """
    if tdev_slope <= 0.0 or tdev_ref_ns <= 0.0 or k_sigma <= 0.0:
        return float("inf")
    ratio = t_budget_ns / (k_sigma * tdev_ref_ns)
    if ratio <= 0.0:
        return 0.0
    return tau_ref_s * (ratio ** (1.0 / tdev_slope))


def coast_cap_from_p22(t_budget_ns, p22_at_tau, k_sigma=1.0,
                       min_tau_s=1, max_tau_s=120):
    """Largest integer coast interval τ ∈ [min,max] whose projected
    filter phase-state uncertainty stays within budget.

    ``p22_at_tau(tau_s)`` → projected P[2,2] (ns²) of the DO phase state
    after coasting ``tau_s`` seconds with no measurement update.  The
    engine passes a closure over DOFreqEst's predict-only covariance
    growth; this layer only needs the callable.

    Returns the largest τ with  k_sigma · √(p22_at_tau(τ)) ≤ budget,
    or ``min_tau_s`` if even the shortest coast is already over budget.

    Assumes P22 grows monotonically with coast length (true under
    predict-only EKF propagation): scans ascending and returns the τ
    just before the first violation.  sqrt(P22) is the filter's own
    honest accumulated-uncertainty estimate — but only in the
    characterized small-Q regime (qFromCharPerActuator); an inflated Q
    makes this cap spuriously tight.
    """
    if k_sigma <= 0.0 or t_budget_ns <= 0.0:
        return min_tau_s
    # Compare in variance space; a relative epsilon keeps the ≤ test
    # honest at an exact boundary (e.g. 0.2²·25 == 1.0 up to float
    # rounding) without admitting anything physically over budget.
    budget_var = (t_budget_ns / k_sigma) ** 2 * (1.0 + 1e-9)
    best = min_tau_s
    for tau in range(int(min_tau_s), int(max_tau_s) + 1):
        if p22_at_tau(tau) <= budget_var:
            best = tau
        else:
            break
    return best


def normalized_distance_to_lock(metric_ns, converged_ns, far_ns):
    """Map a raw convergence signal (√(P22) or σ_total, in ns) to a
    [0,1] *distance-to-lock* — the single derived signal every
    discipline policy reads (disciplineModeFsm reframe: continuous, not
    a latched state).

    Named for what the return value means: 0.0 = locked/converged,
    1.0 = far from lock (NOT "fully converged" — naming-honesty, per
    charlie's PR #80 review; docs/misnomers.md).

    Linear ramp: ≤ ``converged_ns`` → 0.0 (locked, trust the DO);
    ≥ ``far_ns`` → 1.0 (far, correct aggressively); linear between.
    Continuous and monotone, so policies keyed on it cannot chatter on
    enter/exit/re-enter the way a thresholded state can.

    Requires far_ns > converged_ns ≥ 0.
    """
    if far_ns <= converged_ns:
        raise ValueError("far_ns must exceed converged_ns")
    frac = (metric_ns - converged_ns) / (far_ns - converged_ns)
    return min(1.0, max(0.0, frac))


def graded_interval(target_interval, distance_to_lock, min_interval=1):
    """Continuously taper the coast interval by the derived
    distance-to-lock — replaces the binary ``1 if converging else
    interval`` latch (and the 1→120 s actuation cliff it creates).

    ``distance_to_lock`` m ∈ [0,1] (see normalized_distance_to_lock):
    0 = locked → coast the full ``target_interval`` (low loop
    bandwidth, trust the DO); 1 = far → ``min_interval`` (high
    bandwidth, correct every epoch).  Geometric taper:

        τ_eff = round(target · (min/target) ** m)

    Continuous and monotone in m, so — unlike the latch — it cannot
    chatter on enter/exit/re-enter (Bob's emergent-not-event
    preference).  The taper *is* the converging→tracking transition,
    done continuously.
    """
    m = min(1.0, max(0.0, distance_to_lock))
    target = max(int(min_interval), int(target_interval))
    if target <= min_interval:
        return int(min_interval)
    tau = target * (float(min_interval) / target) ** m
    return max(int(min_interval), min(target, int(round(tau))))

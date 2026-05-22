"""M7 adaptive discipline interval scheduler.

Decides when to apply a servo correction based on **input-side** drift
and noise estimates — the error_ns sequence going into accumulate().
Earlier versions watched the servo's own output (adjfine_ppb) to
estimate drift_rate, which created a circular dependency: fast updates
→ big per-epoch output changes → high drift_rate estimate → fast
updates.  Replaced 2026-05-22 (schedulerInputSideDriftRate).

drift_rate now comes from the slope of a linear fit through the last
N samples of (t_monotonic, error_ns); σ_obs from the residual after
removing the slope.  Both are independent of the actuator (PHC
adjfine, DAC code, ClockMatrix FCW, etc.) so the scheduling decision
no longer self-references through the loop.
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
                 error_history_len=300):
        self.base_interval = base_interval
        self.adaptive = adaptive
        self.min_interval = min_interval
        self.max_interval = max_interval
        self.interval = base_interval

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
        """Most recently estimated drift rate (ns/s).  0 before bootstrap."""
        return self._drift_rate_ns_per_s

    @property
    def sigma_obs_ns(self):
        """Most recently estimated input observation σ (ns).  0 before bootstrap."""
        return self._sigma_obs_ns

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
        """Compute optimal discipline interval from input drift + noise.

        ``measurement_sigma_ns`` is retained for API back-compat but
        ignored — σ is now estimated from the error-history residual
        instead of being passed in from a single-correction window.
        """
        if not self.adaptive:
            return self.base_interval

        self._update_drift_rate_from_input()

        # Bootstrap: not enough history yet to estimate drift.
        if len(self._error_history) < _MIN_FIT_SAMPLES:
            return self.base_interval

        # When σ_obs ≈ 0 and drift ≈ 0 the formula is degenerate.
        # Treat as "very quiet system" → max interval.
        if self._drift_rate_ns_per_s < 1e-6:
            self.interval = self.max_interval
            return self.max_interval

        # Drift is unresolvable when the linear trend over the window
        # is smaller than the typical per-sample noise.  Slope SE under
        # white noise of σ over n samples at uniform cadence is
        # σ / √(n · var(t)).  Equivalently: drift × window_span must
        # exceed σ to be detectable above the fit's own noise.  If not,
        # treat drift as zero and return max_interval — actuating when
        # drift is below the noise floor only injects noise.
        window_span_s = (self._error_history[-1][0]
                          - self._error_history[0][0])
        if (self._drift_rate_ns_per_s * window_span_s
                < self._sigma_obs_ns):
            self.interval = self.max_interval
            return self.max_interval

        # Tau optimum (same shape as the legacy formula):
        #   τ = (2 σ_obs / drift_rate) ^ 0.4
        # Both σ and drift_rate are now input-side, so τ truly
        # reflects "how often should we correct given how noisy the
        # input is and how fast the underlying system drifts".
        # σ_obs floor avoids divide-by-near-zero blowups in pristine
        # synthetic test sequences.
        sigma = max(self._sigma_obs_ns, 1e-3)
        tau = (2.0 * sigma / self._drift_rate_ns_per_s) ** 0.4
        tau = max(self.min_interval, min(self.max_interval, int(round(tau))))
        self.interval = tau
        return tau

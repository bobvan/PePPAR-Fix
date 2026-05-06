"""M6 competitive error source selection."""

import math


class ErrorSource:
    """One candidate error estimate with its confidence."""
    __slots__ = ('name', 'error_ns', 'confidence_ns')

    def __init__(self, name, error_ns, confidence_ns):
        self.name = name
        self.error_ns = error_ns
        self.confidence_ns = confidence_ns

    def __repr__(self):
        return f"{self.name}({self.error_ns:+.1f}ns ±{self.confidence_ns:.1f})"


def ppp_qerr(dt_rx_ns, tick_ns=8.0, cal_offset_ns=0.0):
    """Compute PPP-derived PPS quantization error from the 125 MHz tick model.

    The receiver fires PPS at the nearest clock tick to the GNSS integer
    second.  Given the PPP filter's precise dt_rx (receiver clock offset
    from GNSS time), the PPS timing error is the signed distance from
    dt_rx to the nearest tick boundary.

    Args:
        dt_rx_ns: receiver clock offset from GNSS time (ns), from PPP filter.
        tick_ns: receiver clock tick period (8.0 ns for 125 MHz F9T).
        cal_offset_ns: calibration offset (ns) to align PPP's dt_rx
            with the receiver's internal clock estimate.  Determined at
            startup by comparing against TIM-TP qErr.

    Returns:
        PPS quantization error in nanoseconds (same sign convention as
        TIM-TP qErr: positive = PPS fires late).
    """
    D_ns = (dt_rx_ns - cal_offset_ns) % 1_000_000_000
    nearest_tick = round(D_ns / tick_ns) * tick_ns
    return nearest_tick - D_ns


class PPPCalibration:
    """Calibrate PPP dt_rx offset against TIM-TP qErr at startup.

    Accumulates (qerr_ppp - qerr_timtp) samples and computes a circular
    mean (period = tick_ns) to determine the constant offset between PPP's
    dt_rx and the receiver's internal clock estimate.

    Requires dt_rx to be stable (consecutive values within a few µs)
    before accepting samples, to avoid calibrating during filter convergence.
    """

    def __init__(self, tick_ns=8.0, min_samples=10, max_dt_rx_jump_ns=1000.0):
        self.tick_ns = tick_ns
        self.min_samples = min_samples
        self.max_dt_rx_jump_ns = max_dt_rx_jump_ns
        self._sin_sum = 0.0
        self._cos_sum = 0.0
        self._n = 0
        self._prev_dt_rx = None
        self._stable_count = 0
        self._stable_threshold = 3  # need 3 consecutive stable epochs
        self.offset_ns = 0.0
        self.calibrated = False

    def add_sample(self, dt_rx_ns, qerr_timtp_ns):
        """Add one comparison sample.  Returns True when calibration is done."""
        if self.calibrated:
            return True

        # Require dt_rx stability before accepting calibration samples.
        if self._prev_dt_rx is not None:
            jump = abs(dt_rx_ns - self._prev_dt_rx)
            if jump < self.max_dt_rx_jump_ns:
                self._stable_count += 1
            else:
                self._stable_count = 0
        self._prev_dt_rx = dt_rx_ns

        if self._stable_count < self._stable_threshold:
            return False

        raw_qerr = ppp_qerr(dt_rx_ns, self.tick_ns, cal_offset_ns=0.0)
        delta = raw_qerr - qerr_timtp_ns
        angle = 2.0 * math.pi * delta / self.tick_ns
        self._sin_sum += math.sin(angle)
        self._cos_sum += math.cos(angle)
        self._n += 1
        if self._n >= self.min_samples:
            mean_angle = math.atan2(self._sin_sum / self._n,
                                    self._cos_sum / self._n)
            self.offset_ns = mean_angle * self.tick_ns / (2.0 * math.pi)
            self.calibrated = True
            return True
        return False


class CarrierPhaseTracker:
    """DO phase error from PPP + measured inter-oscillator drift rate.

    On most hardware, the PPP receiver (F9T) and the DO use different
    oscillators.  dt_rx from PPP measures the rx TCXO drift from GPS;
    the DO drifts at a different rate.  We measure both rates directly
    and use their difference (D) to correct the Carrier formula:

        carrier_error = (dt_rx - dt_rx_ref) + cumulative_adjfine + D * t

    Where:
    - dt_rx - dt_rx_ref: F9T TCXO phase change from GPS (PPP, ~0.1 ns)
    - cumulative_adjfine: integral of servo corrections applied to PHC
    - D: inter-oscillator differential drift rate (PHC_rate - TCXO_rate)
    - t: elapsed epochs since initialization

    D is estimated directly from two independent measurements:
    - R_tcxo = d(dt_rx)/dt  (from PPP, ~0.1 ppb/epoch precision)
    - R_phc = d(pps_error)/dt - adjfine  (from PPS, ~3.3 ppb/epoch,
      averages to ~0.1 ppb after ~1000 epochs)
    - D = R_phc - R_tcxo

    D improves with runtime: after N epochs, σ_D ≈ 3.3/sqrt(N) ppb.
    Phase error from D uncertainty is bounded by σ_D per epoch (not
    accumulated) because D is re-estimated every epoch from all history.

    No filter lag.  No steady-state phase offset.  The Carrier error
    tracks PPS truth with zero bias while using PPP precision for
    short-term stability.

    Sign convention: positive = PHC ahead of GPS (matches pps_error_ns).
    """

    def __init__(self, stable_threshold=5, max_jump_ns=5000.0):
        self.dt_rx_ref_ns = None
        self.cumulative_adjfine_ns = 0.0
        self.initialized = False
        self._prev_dt_rx_ns = None
        self._stable_count = 0
        self._stable_threshold = stable_threshold
        self._max_jump_ns = max_jump_ns
        # Inter-oscillator drift estimation
        self._epoch_count = 0
        self._sum_d = 0.0       # running sum of per-epoch D samples
        self._sum_d_sq = 0.0    # for variance estimation
        self._n_d = 0           # number of D samples
        self._prev_dt_rx_for_rate = None
        self._prev_pps_error = None
        self._prev_adjfine = None
        self.drift_rate_ppb = 0.0  # current best estimate of D
        # Phase anchor: ensures Carrier zero-point matches PPS truth
        self.phase_anchor_ns = 0.0
        self._anchored = False
        # Anchor residual: running mean of (pps_error - carrier_error).
        # Diagnostic only — shows how much a re-anchor would shift things.
        self._residual_sum = 0.0
        self._residual_n = 0
        self.anchor_residual_ns = 0.0

    def initialize(self, dt_rx_ns):
        """Set the reference dt_rx (called when PHC is aligned to GPS)."""
        self.dt_rx_ref_ns = dt_rx_ns
        self.cumulative_adjfine_ns = 0.0
        self._epoch_count = 0
        self._sum_d = 0.0
        self._sum_d_sq = 0.0
        self._n_d = 0
        self._prev_dt_rx_for_rate = None
        self._prev_pps_error = None
        self._prev_adjfine = None
        self.drift_rate_ppb = 0.0
        self.phase_anchor_ns = 0.0
        self._anchored = False
        self._residual_sum = 0.0
        self._residual_n = 0
        self.anchor_residual_ns = 0.0
        self.initialized = True
        self._stable_count = 0

    def try_auto_init(self, dt_rx_ns):
        """Auto-initialize after dt_rx stabilizes (PPP filter convergence).

        Returns True when initialization is complete.
        """
        if self.initialized:
            return True
        if self._prev_dt_rx_ns is not None:
            if abs(dt_rx_ns - self._prev_dt_rx_ns) < self._max_jump_ns:
                self._stable_count += 1
            else:
                self._stable_count = 0
        self._prev_dt_rx_ns = dt_rx_ns
        if self._stable_count >= self._stable_threshold:
            self.initialize(dt_rx_ns)
            return True
        return False

    def accumulate_adjfine(self, adjfine_ppb, dt_s=1.0):
        """Accumulate one epoch of adjfine. Call every discipline epoch."""
        self.cumulative_adjfine_ns += adjfine_ppb * dt_s
        self._epoch_count += 1

    def update_drift_estimate(self, dt_rx_ns, pps_error_ns, adjfine_ppb):
        """Update the inter-oscillator drift rate estimate.

        Call every epoch with current dt_rx, pps_error, and adjfine.
        Computes per-epoch rates for both oscillators and accumulates
        their difference for a running mean estimate of D.

        Also updates anchor_residual_ns: the running mean of
        (pps_error - carrier_error).  This is the bias we'd correct
        if we re-anchored to PPS right now.  Diagnostic only.
        """
        if not self.initialized:
            return
        # Update anchor residual: running mean of (pps - current carrier)
        carrier = self.compute_error(dt_rx_ns)
        if carrier is not None:
            residual = pps_error_ns - carrier
            self._residual_sum += residual
            self._residual_n += 1
            self.anchor_residual_ns = self._residual_sum / self._residual_n

        if (self._prev_dt_rx_for_rate is not None
                and self._prev_pps_error is not None):
            # R_tcxo: F9T TCXO drift rate (ppb) from PPP
            r_tcxo = dt_rx_ns - self._prev_dt_rx_for_rate

            # R_phc: PHC oscillator drift rate (ppb) from PPS,
            # removing the adjfine contribution we applied
            r_phc = (pps_error_ns - self._prev_pps_error) - self._prev_adjfine

            # D = R_phc - R_tcxo (inter-oscillator differential)
            d_sample = r_phc - r_tcxo
            self._sum_d += d_sample
            self._sum_d_sq += d_sample * d_sample
            self._n_d += 1
            self.drift_rate_ppb = self._sum_d / self._n_d

        self._prev_dt_rx_for_rate = dt_rx_ns
        self._prev_pps_error = pps_error_ns
        self._prev_adjfine = adjfine_ppb

    def drift_rate_sigma(self):
        """Standard error of the drift rate estimate (ppb)."""
        if self._n_d < 2:
            return float('inf')
        mean = self._sum_d / self._n_d
        var = (self._sum_d_sq / self._n_d) - mean * mean
        if var < 0:
            var = 0.0
        import math
        return math.sqrt(var / self._n_d)

    def anchor_to_pps(self, pps_error_ns, dt_rx_ns):
        """One-time anchor of Carrier zero-point to PPS truth.

        Called once when D is reasonably trusted (e.g., right after
        initialization with a seeded D from drift file).  Computes the
        offset such that compute_error(dt_rx_ns) == pps_error_ns at
        this moment.  After anchoring, the Carrier source tracks PPS
        truth with PPP precision instead of carrying a hidden bias.
        """
        if not self.initialized or self._anchored:
            return
        raw = (dt_rx_ns - self.dt_rx_ref_ns) + self.cumulative_adjfine_ns
        correction = self.drift_rate_ppb * self._epoch_count
        unanchored = raw + correction
        self.phase_anchor_ns = pps_error_ns - unanchored
        self._anchored = True

    def compute_error(self, dt_rx_ns):
        """Compute DO phase error with inter-oscillator correction.

        Returns None if not initialized.
        """
        if not self.initialized:
            return None
        raw = (dt_rx_ns - self.dt_rx_ref_ns) + self.cumulative_adjfine_ns
        correction = self.drift_rate_ppb * self._epoch_count
        return raw + correction + self.phase_anchor_ns

    def reset(self, dt_rx_ns):
        """Reset after a PHC restep (phase was re-aligned to GPS)."""
        self.initialize(dt_rx_ns)


def compute_error_sources(pps_error_ns, qerr_ns, dt_rx_ns, dt_rx_sigma_ns,
                          pps_confidence=20.0, qerr_confidence=3.0,
                          carrier_max_sigma=50.0,
                          ticc_error_ns=None, ticc_confidence=None,
                          ppp_cal=None, tick_ns=8.0,
                          carrier_tracker=None,
                          corr_age_s=None,
                          corr_staleness_ns_per_s=0.1):
    """Compute all available error sources and return sorted by confidence.

    Args:
        pps_error_ns: fractional-second PHC error from PPS timestamp
        qerr_ns: quantization error from TIM-TP (None if unavailable)
        dt_rx_ns: receiver clock offset from carrier-phase filter
        dt_rx_sigma_ns: filter's confidence in dt_rx (None if unavailable)
        pps_confidence: assumed PPS-only confidence (ns)
        qerr_confidence: assumed PPS+qErr confidence (ns)
        carrier_max_sigma: max sigma to accept carrier-phase (ns)
        ppp_cal: PPPCalibration instance (None disables PPS+PPP)
        tick_ns: receiver clock tick period (ns)
        carrier_tracker: CarrierPhaseTracker instance (None disables Carrier)
        corr_age_s: age of the freshest applicable NTRIP correction stream
            (seconds, host-monotonic).  When > 0, Carrier and PPS+PPP
            confidences are inflated to reflect that their dt_rx input is
            being computed against increasingly stale corrections.  None
            disables inflation (back-compat).
        corr_staleness_ns_per_s: linear inflation rate (ns of σ per second
            of correction age).  Default 0.1 ns/s puts the GNSS-derived
            sources past PPS+qErr's ~3 ns floor at ~30 s of staleness and
            past PPS-only's ~20 ns floor at ~200 s, giving graceful
            degradation through the source competition.

    Returns:
        List of ErrorSource, sorted by confidence (best first).

    Note: when NTRIP corrections go stale, this function does NOT remove
    Carrier/PPS+PPP from the running — it inflates their σ so source
    competition naturally hands off to PPS+qErr / PPS / TICC.  When
    corrections return, the age drops, the inflation drops, and the GNSS
    sources reclaim the top slot automatically.  No tear-down/restart.
    """
    sources = []

    # Compute the staleness inflation once.  Squared so it can be
    # quadrature-summed with each source's native sigma.
    if corr_age_s is not None and corr_age_s > 0.0:
        inflation_var = (corr_staleness_ns_per_s * corr_age_s) ** 2
    else:
        inflation_var = 0.0

    # ── no-gnss-pps experiment ──────────────────────────────────── #
    # Branch hypothesis: F9T PPS edge timing is not needed for clock
    # discipline.  Carrier-phase-derived clock (PPP dt_rx) is the
    # only steering source.  All sources that consume the F9T PPS
    # edge — PPS, PPS+qErr, TICC (which is qErr-corrected via the
    # caller), PPS+PPP — get a hardwired huge confidence so they
    # cannot win the source competition.  They stay in the source
    # list to preserve existing engine fallback paths (logging,
    # holdover handoff) but never become `best`.
    #
    # Why this shape vs. removing the sources entirely: we want to
    # be able to A/B against the prior behavior with minimal code
    # change, and we want the engine's downstream consumers (source-
    # change logging, scheduler accumulation, status emit) to still
    # see the source list non-empty when carrier is unavailable.
    #
    # Branch fate: this lockout is the experiment.  If the lab test
    # (cold-boot with F9T PPS physically disconnected from the
    # TICC, DO PPS agreement vs otcBob1 within ~100 ns) succeeds,
    # the lockout is justified and merges to main — at which point
    # the lockout becomes a removal of these branches entirely.
    # If it fails, the branch is abandoned.  See
    # docs/no-gnss-pps-experiment.md.
    _GNSS_PPS_SUPPRESS_NS = 1.0e6

    sources.append(ErrorSource('PPS', pps_error_ns,
                               _GNSS_PPS_SUPPRESS_NS))

    if qerr_ns is not None:
        sources.append(ErrorSource('PPS+qErr',
                                   pps_error_ns + qerr_ns,
                                   _GNSS_PPS_SUPPRESS_NS))

    if (carrier_tracker is not None and carrier_tracker.initialized
            and dt_rx_sigma_ns is not None
            and dt_rx_sigma_ns < carrier_max_sigma):
        carrier_error = carrier_tracker.compute_error(dt_rx_ns)
        if carrier_error is not None:
            carrier_sigma = math.sqrt(dt_rx_sigma_ns ** 2 + inflation_var)
            sources.append(ErrorSource('Carrier', carrier_error,
                                       carrier_sigma))

    if (dt_rx_sigma_ns is not None and dt_rx_sigma_ns < carrier_max_sigma
            and ppp_cal is not None and ppp_cal.calibrated):
        qerr_ppp_ns = ppp_qerr(dt_rx_ns, tick_ns, ppp_cal.offset_ns)
        # Sanity: qerr_ppp must be within ±tick/2 (by construction it is,
        # but guard against numerical edge cases).
        if abs(qerr_ppp_ns) <= tick_ns / 2 + 0.1:
            sources.append(ErrorSource('PPS+PPP',
                                       pps_error_ns + qerr_ppp_ns,
                                       _GNSS_PPS_SUPPRESS_NS))

    if ticc_error_ns is not None and ticc_confidence is not None:
        sources.append(ErrorSource('TICC',
                                   ticc_error_ns,
                                   _GNSS_PPS_SUPPRESS_NS))

    sources.sort(key=lambda s: s.confidence_ns)
    return sources



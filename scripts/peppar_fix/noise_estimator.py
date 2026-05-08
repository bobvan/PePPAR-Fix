"""In-band DO noise estimation from discipline gaps.

When the adaptive discipline scheduler extends its interval, epochs
between corrections are genuinely free-running: the DO drifts with no
adjfine change applied.  After removing the expected linear drift
(constant adjfine × dt), the residual is pure DO phase noise.

Two channels:
- Gap channel: epochs where no correction was applied.  Pure DO noise.
- Correction channel: residuals after removing expected frequency step.
  If corrections are truly noise-free (as measured), both channels
  should agree.  Divergence indicates DO stress or write latency.

Computes running TDEV (time deviation, in ns) at tau = 1, 2, 4, ...
seconds from the residual phase samples.

State is persisted to state/dos/<do_uid>_noise.json on clean shutdown
and loaded on the next run, so TDEV at long taus can accumulate across
runs.
"""

import json
import logging
import math
import os
import time
from datetime import datetime, timezone

log = logging.getLogger("peppar_fix.noise_estimator")


class InBandNoiseEstimator:
    """Estimates DO noise floor from discipline gap measurements.

    Feed every epoch's phase error and whether a correction was applied.
    The estimator detrends phase using the current adjfine and computes
    ADEV from the residuals.

    Args:
        min_gap_epochs: minimum gap epochs before computing statistics
        max_history: maximum phase samples to retain per channel
    """

    def __init__(self, min_gap_epochs=10, max_history=7200):
        self._min_gap_epochs = min_gap_epochs
        self._max_history = max_history

        # Gap channel: phase residuals during no-correction epochs
        self._gap_phases = []       # detrended phase (ns)
        self._gap_times = []        # monotonic timestamps

        # Correction channel: phase residuals including correction epochs
        self._corr_phases = []
        self._corr_times = []

        # Running state
        self._last_correction_mono = None
        self._last_adjfine_ppb = None
        self._gap_tdev = {}         # tau -> tdev_ns
        self._corr_tdev = {}
        self._gap_count = 0
        self._corr_count = 0

        # Detrending: accumulated phase from known frequency
        self._phase_acc_ns = 0.0
        self._prev_mono = None

        # Last-feed exposure for external CSV / instrumentation paths
        # (I-162848 Step 1).  Populated on every feed() call so the
        # engine can record (mono, residual_ns, channel) without
        # duplicating detrending math.  Channel is "gap" when this
        # epoch landed in BOTH gap_phases and corr_phases (no
        # correction applied), "correction" when this epoch landed in
        # corr_phases only (a correction was applied — the linear
        # drift accumulator was reset).
        self.last_mono = None
        self.last_residual_ns = None
        self.last_channel = None

    def feed(self, phase_error_ns, adjfine_ppb, corrected_this_epoch,
             mono=None):
        """Feed one epoch's measurement.

        Args:
            phase_error_ns: measured phase error (from best source)
            adjfine_ppb: current adjfine setting
            corrected_this_epoch: True if a frequency correction was
                applied this epoch
            mono: CLOCK_MONOTONIC timestamp (default: now)
        """
        if mono is None:
            mono = time.monotonic()

        # Detrend: remove expected phase accumulation from adjfine drift.
        # If adjfine is A ppb, phase drifts A ns/s.  Between epochs,
        # expected_phase_change = adjfine_ppb * dt_s.
        if self._prev_mono is not None and self._last_adjfine_ppb is not None:
            dt_s = mono - self._prev_mono
            if 0 < dt_s < 30:
                expected_drift_ns = self._last_adjfine_ppb * dt_s
                self._phase_acc_ns += expected_drift_ns

        residual_ns = phase_error_ns - self._phase_acc_ns
        self._prev_mono = mono
        self._last_adjfine_ppb = adjfine_ppb

        # Correction channel: all epochs
        self._corr_phases.append(residual_ns)
        self._corr_times.append(mono)
        self._corr_count += 1
        if len(self._corr_phases) > self._max_history:
            self._corr_phases.pop(0)
            self._corr_times.pop(0)

        # Gap channel: only non-correction epochs
        if not corrected_this_epoch:
            self._gap_phases.append(residual_ns)
            self._gap_times.append(mono)
            self._gap_count += 1
            if len(self._gap_phases) > self._max_history:
                self._gap_phases.pop(0)
                self._gap_times.pop(0)
            channel = "gap"
        else:
            # Reset detrending accumulator on correction — the correction
            # changes the frequency, so phase accumulation restarts.
            self._phase_acc_ns = 0.0
            self._last_correction_mono = mono
            channel = "correction"

        # Expose last-feed values for external CSV instrumentation.
        self.last_mono = mono
        self.last_residual_ns = residual_ns
        self.last_channel = channel

        # Periodically recompute ADEV (every 60 samples)
        if self._corr_count % 60 == 0:
            self._recompute_tdev()

    def _recompute_tdev(self):
        """Recompute overlapping ADEV for both channels."""
        self._gap_tdev = _compute_tdev(self._gap_phases)
        self._corr_tdev = _compute_tdev(self._corr_phases)

    @property
    def gap_tdev(self):
        """Gap channel TDEV: {tau_s: tdev_ns}."""
        return dict(self._gap_tdev)

    @property
    def correction_tdev(self):
        """Correction channel TDEV: {tau_s: tdev_ns}."""
        return dict(self._corr_tdev)

    @property
    def gap_samples(self):
        return len(self._gap_phases)

    @property
    def total_samples(self):
        return len(self._corr_phases)

    def summary(self):
        """One-line summary for logging."""
        parts = [f"gap={self.gap_samples} corr={self.total_samples}"]
        if self._gap_tdev:
            tau1 = self._gap_tdev.get(1)
            if tau1 is not None:
                parts.append(f"gap_TDEV(1s)={tau1:.2f}ns")
        if self._corr_tdev:
            tau1 = self._corr_tdev.get(1)
            if tau1 is not None:
                parts.append(f"corr_TDEV(1s)={tau1:.2f}ns")
        return " ".join(parts)

    # ── Persistence ───────────────────────────────────────────────── #

    def to_dict(self):
        """Serialize state for JSON persistence.

        Saves the phase buffers (for warm-start TDEV at long taus),
        the computed TDEV curves, and sample counts.  Detrending state
        is NOT saved — it's reset on load since the adjfine context
        changes between runs.
        """
        # Only save the tail of the phase buffers (max_history already caps)
        return {
            "gap_phases": self._gap_phases[-self._max_history:],
            "corr_phases": self._corr_phases[-self._max_history:],
            "gap_tdev": {str(k): v for k, v in self._gap_tdev.items()},
            "corr_tdev": {str(k): v for k, v in self._corr_tdev.items()},
            "gap_count": self._gap_count,
            "corr_count": self._corr_count,
            "updated": datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
        }

    @classmethod
    def from_dict(cls, data, max_history=7200):
        """Restore from a previously saved dict.

        Phase buffers are loaded so TDEV at long taus can build on
        prior data.  Detrending state starts fresh (phase_acc=0,
        prev_mono=None) since the servo context has changed.
        """
        est = cls(max_history=max_history)
        est._gap_phases = data.get("gap_phases", [])[-max_history:]
        est._corr_phases = data.get("corr_phases", [])[-max_history:]
        est._gap_tdev = {int(k): v for k, v in data.get("gap_tdev", {}).items()}
        est._corr_tdev = {int(k): v for k, v in data.get("corr_tdev", {}).items()}
        est._gap_count = data.get("gap_count", len(est._gap_phases))
        est._corr_count = data.get("corr_count", len(est._corr_phases))
        # Timestamps not restored — detrending restarts fresh
        est._gap_times = []
        est._corr_times = []
        return est


def noise_state_path(do_uid, state_dir=None):
    """Path for a DO's noise estimator state file."""
    if state_dir is None:
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        state_dir = os.path.join(repo_root, "state", "dos")
    safe_id = str(do_uid).replace(":", "-").replace("/", "_")
    return os.path.join(state_dir, f"{safe_id}_noise.json")


def save_noise_state(do_uid, estimator, state_dir=None):
    """Save noise estimator state to disk.  Atomic write."""
    if estimator is None or estimator.total_samples < 10:
        return  # nothing worth saving
    path = noise_state_path(do_uid, state_dir)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    data = estimator.to_dict()
    data["do_uid"] = str(do_uid)
    tmp = path + ".tmp"
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=2)
        f.write('\n')
    os.replace(tmp, path)
    log.info("Saved noise state to %s (%d gap, %d corr samples)",
             path, estimator.gap_samples, estimator.total_samples)


def load_noise_state(do_uid, state_dir=None):
    """Load a saved noise estimator.  Returns InBandNoiseEstimator or None."""
    path = noise_state_path(do_uid, state_dir)
    if not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            data = json.load(f)
        est = InBandNoiseEstimator.from_dict(data)
        log.info("Loaded noise state from %s (%d gap, %d corr samples)",
                 path, est.gap_samples, est.total_samples)
        return est
    except (json.JSONDecodeError, OSError, KeyError) as e:
        log.warning("Failed to load noise state from %s: %s", path, e)
        return None


def _compute_tdev(phases, taus=None):
    """Compute time deviation (TDEV) from phase samples.

    TDEV has units of time (ns when phase is in ns), unlike ADEV which
    is dimensionless.  Uses the overlapping TDEV estimator:

        TDEV²(nτ₀) = τ₀²/(6n²(N-3n+1)) Σ_{j=0}^{N-3n} [Σ_{i=j}^{j+n-1} (x[i+2n] - 2x[i+n] + x[i])]²

    This is equivalent to TDEV(τ) = τ/√3 × MDEV(τ), where MDEV is the
    modified Allan deviation computed with nested averaging.

    Args:
        phases: list of phase values (ns), equally spaced at tau0=1s
        taus: list of averaging factors n to compute (default: 1, 2, 4, ..., N/4)

    Returns:
        dict {tau_s: tdev_ns}
    """
    N = len(phases)
    if N < 4:
        return {}

    if taus is None:
        taus = []
        n = 1
        while 3 * n < N:
            taus.append(n)
            n *= 2

    result = {}
    for n in taus:
        if 3 * n >= N:
            break
        # Outer sum: j = 0 .. N-3n
        outer_count = N - 3 * n + 1
        if outer_count < 1:
            break
        total = 0.0
        for j in range(outer_count):
            # Inner sum: average n consecutive second-differences
            inner = 0.0
            for i in range(j, j + n):
                inner += phases[i + 2 * n] - 2 * phases[i + n] + phases[i]
            total += inner * inner
        # τ₀ = 1s, so τ₀² = 1
        tdev_sq = total / (6.0 * n * n * outer_count)
        result[n] = math.sqrt(tdev_sq)

    return result

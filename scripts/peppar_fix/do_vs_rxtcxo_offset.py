"""Captures the rolling DO-vs-rx_TCXO phase offset during TRACKING.

During TRACKING, both x[0] (rx_TCXO phase from GPS) and x[2] (DO phase
from GPS) are anchored by hardware arms.  The offset x[2] - x[0] is
the calibration that gets frozen at HOLDOVER entry and used by the
TdcpPhaseIntegrator to propagate x[2]_pseudo.

The EMA smooths epoch-to-epoch noise.  τ controls the trade-off:
too short → noisy offset re-noises HOLDOVER predictions; too long →
stale offset blind to real drift during TRACKING.  Default 120 s.
"""

import math


class DoVsRxTcxoOffset:
    """Rolling EMA of (x[2] - x[0]) during TRACKING."""

    def __init__(self, tau_s: float = 120.0, measured_bias_ppb: float = 0.0):
        """
        Args:
            tau_s: EMA time constant in seconds (60-300 recommended).
            measured_bias_ppb: per-host TDCP bias from the prerequisite
                measurement.  If nonzero, contributes a bias × T term
                to the holdover σ calculation.
        """
        if tau_s <= 0:
            raise ValueError(f"tau_s must be positive, got {tau_s}")
        self._tau_s = tau_s
        self._measured_bias_ppb = measured_bias_ppb
        self._offset_ns: float | None = None
        self._variance_ns2: float = 0.0
        self._last_capture_mono: float | None = None
        self._n_samples: int = 0

    def update(self, x0_phi_rx_ns: float, x2_phi_do_ns: float,
               p00: float, p22: float, mono_s: float) -> None:
        """Feed a new (x[0], x[2]) pair from the EKF during TRACKING.

        Args:
            x0_phi_rx_ns: current x[0] (rx_TCXO phase from GPS, ns).
            x2_phi_do_ns: current x[2] (DO phase from GPS, ns).
            p00: P[0,0] (rx_TCXO phase variance, ns²).
            p22: P[2,2] (DO phase variance, ns²).
            mono_s: CLOCK_MONOTONIC timestamp of this epoch (seconds).
        """
        raw_offset = x2_phi_do_ns - x0_phi_rx_ns
        raw_var = p00 + p22

        if self._offset_ns is None:
            self._offset_ns = raw_offset
            self._variance_ns2 = raw_var
            self._last_capture_mono = mono_s
            self._n_samples = 1
            return

        dt = mono_s - self._last_capture_mono if self._last_capture_mono is not None else 1.0
        if dt <= 0:
            dt = 1.0
        alpha = 1.0 - math.exp(-dt / self._tau_s)

        self._offset_ns += alpha * (raw_offset - self._offset_ns)
        self._variance_ns2 += alpha * (raw_var - self._variance_ns2)
        self._last_capture_mono = mono_s
        self._n_samples += 1

    @property
    def last_offset_ns(self) -> float | None:
        """The smoothed DO-vs-rx_TCXO offset, or None if never updated."""
        return self._offset_ns

    @property
    def last_offset_sigma_ns(self) -> float:
        """1-σ uncertainty on the smoothed offset (ns)."""
        return math.sqrt(max(0.0, self._variance_ns2))

    @property
    def last_capture_mono(self) -> float | None:
        """CLOCK_MONOTONIC of the last update, or None."""
        return self._last_capture_mono

    @property
    def n_samples(self) -> int:
        return self._n_samples

    @property
    def measured_bias_ppb(self) -> float:
        return self._measured_bias_ppb

    @property
    def tau_s(self) -> float:
        return self._tau_s

    def holdover_sigma_ns(self, holdover_duration_s: float,
                          tdcp_sigma_y_ppb: float = 0.026) -> float:
        """Predicted σ(x[2]_pseudo) after holdover_duration_s seconds.

        tdcp_sigma_y_ppb default is PiFace F9T-20B's value.  Per-host:
        TimeHat F9T-10 ~0.051, clkPoC3 F9T-20B ~0.057, MadHat F10T ~0.065.

        Combines:
          S1: TDCP integrated phase noise = σ_y × √T
          S2: frozen offset initial uncertainty = last_offset_sigma_ns
          S3: differential drift (estimated from measured_bias_ppb × T)

        Returns σ in nanoseconds.  Used by the engine to report holdover
        quality and drive PTP clockClass transitions.
        """
        T = holdover_duration_s
        s1_ns = tdcp_sigma_y_ppb * math.sqrt(max(0.0, T))
        s2_ns = self.last_offset_sigma_ns
        s3_ns = abs(self._measured_bias_ppb) * T
        return math.sqrt(s1_ns ** 2 + s2_ns ** 2 + s3_ns ** 2)

    @property
    def ready(self) -> bool:
        """True once the EMA has had enough samples to be meaningful."""
        return self._n_samples >= 10

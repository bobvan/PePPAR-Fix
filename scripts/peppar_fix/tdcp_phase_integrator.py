"""Integrates rx_TCXO phase from TDCP df_f deltas during HOLDOVER.

Seeded at HOLDOVER entry from the filter's current x[0].  Each epoch
accumulates phase via `phase += df_f * dt`.  Combined with the frozen
DoVsRxTcxoOffset, produces x[2]_pseudo for the holdover pseudo-obs.

Gap protection: drops any epoch where dt > max_dt_s and marks the
integrator as needing a re-seed at the next hardware re-acquisition.
"""

import math


class TdcpPhaseIntegrator:
    """Maintains integrated rx_TCXO phase from TDCP frequency deltas."""

    def __init__(self, max_dt_s: float = 1.5):
        if max_dt_s <= 0:
            raise ValueError(f"max_dt_s must be positive, got {max_dt_s}")
        self._max_dt_s = max_dt_s
        self._phase_ns: float | None = None
        self._last_mono_s: float | None = None
        self._holdover_start_mono: float | None = None
        self._n_integrated: int = 0
        self._gap_detected: bool = False

    def seed(self, x0_phi_rx_ns: float, mono_s: float) -> None:
        """Seed the integrator at HOLDOVER entry.

        Args:
            x0_phi_rx_ns: current x[0] from the EKF (rx_TCXO phase, ns).
            mono_s: CLOCK_MONOTONIC of this epoch.
        """
        self._phase_ns = x0_phi_rx_ns
        self._last_mono_s = mono_s
        self._holdover_start_mono = mono_s
        self._n_integrated = 0
        self._gap_detected = False

    def integrate(self, df_f: float, mono_s: float) -> bool:
        """Accumulate one TDCP epoch's fractional frequency into phase.

        Args:
            df_f: TDCP ensemble fractional-frequency estimate (dimensionless,
                e.g., -1.6e-8 for -16 ppb).
            mono_s: CLOCK_MONOTONIC of this epoch.

        Returns:
            True if the epoch was integrated; False if dropped (gap or
            not seeded).
        """
        if self._phase_ns is None or self._last_mono_s is None:
            return False

        dt = mono_s - self._last_mono_s
        if dt <= 0:
            return False

        if dt > self._max_dt_s:
            self._gap_detected = True
            return False

        self._phase_ns += df_f * dt * 1e9
        self._last_mono_s = mono_s
        self._n_integrated += 1
        return True

    def pseudo_obs(self, offset_ns: float) -> float | None:
        """Compute x[2]_pseudo = integrated_phase + frozen_offset.

        Args:
            offset_ns: the DoVsRxTcxoOffset.last_offset_ns frozen at
                HOLDOVER entry.

        Returns:
            x[2]_pseudo in nanoseconds, or None if not seeded or gap.
        """
        if self._phase_ns is None or self._gap_detected:
            return None
        return self._phase_ns + offset_ns

    @property
    def phase_ns(self) -> float | None:
        """Current integrated rx_TCXO phase (ns), or None if not seeded."""
        return self._phase_ns

    @property
    def holdover_duration_s(self) -> float:
        """Seconds since HOLDOVER entry (seed)."""
        if self._holdover_start_mono is None or self._last_mono_s is None:
            return 0.0
        return max(0.0, self._last_mono_s - self._holdover_start_mono)

    @property
    def n_integrated(self) -> int:
        return self._n_integrated

    @property
    def gap_detected(self) -> bool:
        return self._gap_detected

    @property
    def seeded(self) -> bool:
        return self._phase_ns is not None

    def reset(self) -> None:
        """Clear state (e.g., on REACQUIRED → TRACKING transition)."""
        self._phase_ns = None
        self._last_mono_s = None
        self._holdover_start_mono = None
        self._n_integrated = 0
        self._gap_detected = False

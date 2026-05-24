"""Per-epoch ensemble-innovation gate for TDCP observations.

Layer L2 of the TDCP slip-protection stack (Phase B), per
`docs/tdcp-servo-integration.md`.  Catches multi-SV co-slips that
defeat per-SV detectors (L1) when several SVs slip together —
e.g., the day0419h sunrise event where 115 slips landed in 2 hours
across rising SVs sharing the same azimuth/elevation regime.

Maintains a sliding window of recent ensemble `tdcp_freq_ppb`
observations.  After warmup, rejects any new observation whose
absolute deviation from the running median exceeds `n_sigma × MAD-σ`.

Robust statistics (median + MAD) are used so that an outlier from
a single slip epoch doesn't move the gate's center.  Always
buffers (including rejected observations) so that genuine drift
in the rx-TCXO is tracked over window-many epochs — the gate
adapts to real signal, only filters transients.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass


@dataclass
class GateResult:
    accepted: bool
    median: float       # running median over buffer (ppb)
    sigma: float        # MAD-based σ (ppb)
    deviation: float    # |obs − median| in σ-units; NaN if no stats yet


class TdcpInnovGate:
    """Sliding-window median-anomaly gate on TDCP frequency observations.

    Default window 30 s, n_sigma 5 — generous enough that intrinsic
    TDCP noise (σ ~0.026 ppb on a clean F9T) never trips the gate
    spuriously, tight enough that a single co-slip on >=1 SV
    (≥ 0.6 ppb frequency punch) is rejected.
    """

    def __init__(self, window: int = 30, n_sigma: float = 5.0,
                 sigma_floor_ppb: float = 0.01):
        """
        sigma_floor_ppb: lower bound on MAD-derived σ.  Prevents the
            gate from becoming hypersensitive when the buffer is too
            clustered (MAD ≈ 0).  Default 0.01 ppb sits ~half the
            intrinsic TDCP TDEV-derived σ_y (~0.026 ppb on clean F9T),
            so a real co-slip punch (≥ 0.1 ppb) is still tens of σ
            but tiny epoch-to-epoch drift doesn't trip the gate.
        """
        if window < 3:
            raise ValueError("window must be ≥ 3 for stable MAD")
        if n_sigma <= 0:
            raise ValueError("n_sigma must be positive")
        if sigma_floor_ppb <= 0:
            raise ValueError("sigma_floor_ppb must be positive")
        self.window = window
        self.n_sigma = n_sigma
        self.sigma_floor_ppb = sigma_floor_ppb
        self._buf: deque[float] = deque(maxlen=window)

    def evaluate(self, freq_ppb: float) -> GateResult:
        """Decide whether `freq_ppb` is an outlier.

        Returns a GateResult.  During warmup (buffer not yet full),
        always accepts and returns a NaN-decoration result.  Buffers
        every observation (accepted or rejected) so the running
        statistic tracks real rx-TCXO drift.
        """
        if len(self._buf) < self.window:
            self._buf.append(freq_ppb)
            return GateResult(
                accepted=True, median=0.0, sigma=0.0, deviation=float("nan")
            )

        arr = sorted(self._buf)
        n = len(arr)
        if n % 2 == 1:
            median = arr[n // 2]
        else:
            median = 0.5 * (arr[n // 2 - 1] + arr[n // 2])

        abs_dev = sorted(abs(x - median) for x in arr)
        if n % 2 == 1:
            mad = abs_dev[n // 2]
        else:
            mad = 0.5 * (abs_dev[n // 2 - 1] + abs_dev[n // 2])

        # 1.4826: MAD-to-σ scale factor for Gaussian noise.
        # sigma_floor_ppb prevents a degenerate (all-identical-value)
        # buffer from collapsing σ → 0 and rejecting every subsequent
        # observation.  Default 0.01 ppb ≈ half the intrinsic TDCP
        # σ_y on a clean F9T.
        sigma = max(1.4826 * mad, self.sigma_floor_ppb)

        deviation = abs(freq_ppb - median) / sigma
        accepted = deviation <= self.n_sigma

        # Always buffer — adaptive to genuine drift.  A single
        # rejected outlier moves the median negligibly; a sustained
        # shift will see the window flip from "old" to "new" and
        # the gate re-anchor to the new baseline.
        self._buf.append(freq_ppb)

        return GateResult(
            accepted=accepted, median=median, sigma=sigma, deviation=deviation
        )

    def reset(self) -> None:
        """Drop all buffered observations.  Use after slip recovery
        or filter reset to avoid mixing pre- and post-event regimes."""
        self._buf.clear()

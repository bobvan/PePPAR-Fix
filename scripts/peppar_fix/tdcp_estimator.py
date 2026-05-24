"""Time-differenced carrier-phase (TDCP) clock-frequency estimator.

Phase A of the TDCP servo integration (`docs/tdcp-servo-integration.md`).

Read-only.  Consumes the per-epoch observation dicts that
`realtime_ppp.serial_reader` puts onto `obs_queue` plus a
`BroadcastEphemeris` instance, and emits an ensemble TDCP
estimate of the receiver-clock frequency error every epoch.

Math + lab validation: `prototypes/tdcp_proto.py` and
`prototypes/tdcp-findings-2026-05-22.md`.

Does NOT steer the servo.  Phase B wires the output into
`do_freq_est.py` as Arm 5; Phase C adds the L2/L3 slip-protection
layers.  Phase A's slip layer is L1 (per-SV) via the engine's
existing `cycle_slip.CycleSlipMonitor` when supplied, plus an
internal lock-duration / LLI fallback when not.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Optional

import numpy as np

# ── Physical constants ──────────────────────────────────────────── #

C_LIGHT = 299_792_458.0
OMEGA_E = 7.2921151467e-5     # Earth rotation rate (rad/s)
GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)


# ── Per-SV state across epochs ──────────────────────────────────── #

@dataclass
class _SvPrev:
    """Per-SV state remembered from the previous epoch."""

    t_gps_s: float       # continuous GPS seconds since GPS_EPOCH
    phi_m: float         # carrier phase in metres (cycles × λ)
    rho_geom_m: float    # geometric range to satellite at prev epoch
    sat_clk_m: float     # c × satellite-clock offset at prev epoch
    lock_ms: float = 0.0  # UBX lock_duration_ms at prev epoch


# ── Estimator output ────────────────────────────────────────────── #

@dataclass
class TdcpResult:
    """One epoch's TDCP estimator output.

    `df_f` is the dimensionless fractional-frequency offset:
    `Δdt_rx / Δt` where Δdt_rx is the per-epoch advance of the
    receiver clock relative to GPS time.  Positive ⇒ rx clock ran
    fast over the interval.
    """

    t_dt: datetime
    n_total: int
    n_used: int
    c_dt_rx_m: float    # ensemble median c·Δdt_rx (m); NaN if no solve
    df_f: float         # dimensionless fractional frequency; NaN if no solve
    median_r_m: float   # median per-SV TDCP residual (m); NaN if empty
    mad_r_m: float      # MAD of per-SV residuals (m); NaN if empty
    dt_s: float         # median epoch interval used (s)
    kept_svs: list[str] = field(default_factory=list)


# ── The estimator ───────────────────────────────────────────────── #

class TdcpEstimator:
    """Ensemble time-differenced carrier-phase clock-frequency estimator.

    Per-epoch flow:

      1. For each SV with continuous lock, compute the TDCP residual:
              r(sv) = Δφ_meas − (Δρ_geom − Δ(c·dt_sat))
         All in metres.  Right-hand side is the broadcast-ephemeris
         prediction of the per-epoch range + sat-clock change.  r(sv)
         then approximates c·Δdt_rx plus measurement noise.

      2. Robust ensemble: median across SVs, with N-MAD outlier
         rejection (default N = 3).

      3. Output `c·Δdt_rx` (m) and `df/f = Δdt_rx / Δt`.

    The estimator drops an SV's residual for one epoch if:

      - lock_duration_ms dropped between epochs (UBX cap-aware), OR
      - the supplied slip monitor flagged it this epoch, OR
      - LLI bit 0 set on this epoch's carrier phase (RINEX path), OR
      - epoch gap exceeds `max_dt_s`, OR
      - the residual is > `mad_k` × MAD of the current epoch's ensemble.

    Per-SV state is still updated even when the diff is suppressed,
    so the next clean epoch can diff against this one.
    """

    # UBX caps locktime around 64 s — drops below that band are real.
    _UBX_LOCKTIME_CAP_MS = 64_000.0
    _LOCKTIME_DROP_MS = 500.0

    def __init__(
        self,
        eph,                          # BroadcastEphemeris (or duck-equiv)
        rx_ecef,                      # iterable of 3 floats; ~10 m OK
        slip_monitor=None,            # optional CycleSlipMonitor
        max_dt_s: float = 1.5,        # epoch-pair max gap
        mad_k: float = 3.0,           # ensemble outlier multiplier
    ):
        self.eph = eph
        self.rx_ecef = np.asarray(rx_ecef, dtype=float)
        self.slip_monitor = slip_monitor
        self.max_dt_s = float(max_dt_s)
        self.mad_k = float(mad_k)
        self._prev: dict[str, _SvPrev] = {}
        self._epoch_count: int = 0

    # ── public API ────────────────────────────────────────────── #

    def update(
        self,
        observations: list[dict[str, Any]],
        t_dt: datetime,
        t_mono_s: Optional[float] = None,
    ) -> TdcpResult:
        """Process one epoch of observations and return the result.

        `observations` follows the same schema the engine's
        `cycle_slip.CycleSlipMonitor.check` consumes: each dict has
        `sv`, `phi1_cyc`, `wl_f1`, `cno`, `lock_duration_ms`, and
        optionally `lli` and dual-freq fields (unused here).

        `t_dt` must be a timezone-aware UTC datetime corresponding
        to GPS-time of the epoch.  `t_mono_s` is CLOCK_MONOTONIC
        seconds and is passed through to the slip monitor; if None,
        we substitute continuous GPS seconds (slip monitor only uses
        it for inter-epoch deltas, which is the same either way).
        """
        self._epoch_count += 1
        t_gps_s = (t_dt - GPS_EPOCH).total_seconds()
        if t_mono_s is None:
            t_mono_s = t_gps_s

        # External slip-monitor pass (optional).  Flags SVs that
        # slipped this epoch; we'll still update their prev state but
        # skip the diff so the next clean epoch can resume.
        external_slips: set[str] = set()
        if self.slip_monitor is not None:
            try:
                events = self.slip_monitor.check(
                    observations, t_mono_s, epoch=self._epoch_count
                )
                external_slips = {e.sv for e in events}
            except Exception:
                # A buggy slip monitor must not break the estimator.
                external_slips = set()

        residuals: list[tuple[str, float]] = []
        new_prev: dict[str, _SvPrev] = {}
        dts: list[float] = []
        n_total = len(observations)

        for obs in observations:
            sv = obs.get("sv")
            phi_cyc = obs.get("phi1_cyc")
            wl = obs.get("wl_f1")
            if sv is None or phi_cyc is None or wl is None or phi_cyc == 0:
                continue
            phi_m = float(phi_cyc) * float(wl)

            pos, clk = self.eph.sat_position(sv, t_dt)
            if pos is None:
                continue

            rho_geom_m = self._rotated_range(pos)
            sat_clk_m = float(clk or 0.0) * C_LIGHT

            lock_ms = float(obs.get("lock_duration_ms") or 0.0)
            lli = int(obs.get("lli") or 0)

            cur = _SvPrev(
                t_gps_s=t_gps_s, phi_m=phi_m,
                rho_geom_m=rho_geom_m, sat_clk_m=sat_clk_m,
                lock_ms=lock_ms,
            )

            # Slip determination.  When an SV slips, we keep its
            # current epoch's data in new_prev (so the *next* epoch
            # can diff against post-slip phase) but skip this epoch's
            # residual (which would cross the slip).
            slipped = sv in external_slips or self._locktime_slipped(sv, lock_ms) \
                or (lli & 1) != 0
            new_prev[sv] = cur

            if slipped:
                continue

            prev = self._prev.get(sv)
            if prev is None:
                continue
            dt = t_gps_s - prev.t_gps_s
            if dt <= 0 or dt > self.max_dt_s:
                continue
            dts.append(dt)

            d_phi = phi_m - prev.phi_m
            d_rho = rho_geom_m - prev.rho_geom_m
            d_sat = sat_clk_m - prev.sat_clk_m
            r = d_phi - (d_rho - d_sat)
            residuals.append((sv, r))

        self._prev = new_prev
        dt_used = float(np.median(dts)) if dts else 1.0

        if not residuals:
            return TdcpResult(
                t_dt=t_dt, n_total=n_total, n_used=0,
                c_dt_rx_m=float("nan"), df_f=float("nan"),
                median_r_m=float("nan"), mad_r_m=float("nan"),
                dt_s=dt_used,
            )

        rs = np.array([r[1] for r in residuals])
        med = float(np.median(rs))
        mad = float(np.median(np.abs(rs - med))) * 1.4826
        if mad < 1e-3:                # floor: 1 mm
            mad = 1e-3
        keep = np.abs(rs - med) <= self.mad_k * mad
        kept_pairs = [residuals[i] for i in range(len(residuals)) if keep[i]]

        if not kept_pairs:
            return TdcpResult(
                t_dt=t_dt, n_total=n_total, n_used=0,
                c_dt_rx_m=med, df_f=float("nan"),
                median_r_m=med, mad_r_m=mad, dt_s=dt_used,
            )

        c_dt_rx_m = float(np.median([k[1] for k in kept_pairs]))
        df_f = (c_dt_rx_m / C_LIGHT) / max(dt_used, 1e-3)

        return TdcpResult(
            t_dt=t_dt, n_total=n_total, n_used=len(kept_pairs),
            c_dt_rx_m=c_dt_rx_m, df_f=df_f,
            median_r_m=med, mad_r_m=mad,
            dt_s=dt_used, kept_svs=[k[0] for k in kept_pairs],
        )

    # ── internals ─────────────────────────────────────────────── #

    def _rotated_range(self, sat_pos_ecef) -> float:
        """Apply the Sagnac (Earth-rotation) correction to the
        sat→rx geometric range.  One-pass iteration suffices for
        m-class rx position uncertainty.
        """
        d0 = sat_pos_ecef - self.rx_ecef
        rho = float(np.linalg.norm(d0))
        theta = OMEGA_E * rho / C_LIGHT
        ct, st = math.cos(theta), math.sin(theta)
        rotated = np.array([
            ct * sat_pos_ecef[0] + st * sat_pos_ecef[1],
            -st * sat_pos_ecef[0] + ct * sat_pos_ecef[1],
            sat_pos_ecef[2],
        ])
        return float(np.linalg.norm(rotated - self.rx_ecef))

    def _locktime_slipped(self, sv: str, lock_ms: float) -> bool:
        """UBX-locktime drop check.  Returns True iff the previous
        epoch's locktime was below the cap and this epoch's locktime
        dropped by more than `_LOCKTIME_DROP_MS`.
        """
        prev = self._prev.get(sv)
        if prev is None:
            return False
        if prev.lock_ms >= self._UBX_LOCKTIME_CAP_MS:
            return False  # u-blox locktime is capped; we can't tell
        if (prev.lock_ms - lock_ms) > self._LOCKTIME_DROP_MS:
            return True
        return False

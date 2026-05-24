"""Unit tests for the TDCP estimator (`tdcp_estimator.py`).

Synthetic obs + mock ephemeris.  Exercises the math, slip handling,
and ensemble robustness end-to-end without depending on a real UBX
fixture.  See `prototypes/tdcp_validation.py` for the live-data
TDEV regression run against the prototype fixture.
"""

from __future__ import annotations

import math
import unittest
from datetime import datetime, timedelta, timezone

import numpy as np

from peppar_fix.tdcp_estimator import (
    C_LIGHT,
    GPS_EPOCH,
    TdcpEstimator,
    TdcpResult,
)


# ── Mock ephemeris that returns a deterministic sat position+clock ── #

class _MockEph:
    """Mock BroadcastEphemeris.

    `sat_position(sv, t)` returns `(np.array, clock)` for a fixed table
    of SVs.  Position can be a constant or a callable of `(sv, t)`
    returning ECEF (so tests can model a moving satellite).
    """

    def __init__(self, table):
        # table: {sv_str: (pos_or_callable, clock_or_callable)}
        self._table = table

    def sat_position(self, sv, t):
        entry = self._table.get(sv)
        if entry is None:
            return None, None
        pos_spec, clk_spec = entry
        pos = pos_spec(sv, t) if callable(pos_spec) else np.asarray(pos_spec, dtype=float)
        clk = clk_spec(sv, t) if callable(clk_spec) else float(clk_spec)
        return pos, clk


# A representative lab ECEF (CHOKE1, ~Chicago lab; doesn't have to be exact)
_LAB_ECEF = np.array([157470.0, -4756189.5, 4232768.0])


def _stationary_sat(offset_xyz):
    """Helper: build a stationary-sat position around an offset."""
    pos = _LAB_ECEF + np.asarray(offset_xyz, dtype=float)
    return (pos, 0.0)


def _moving_sat(offset_xyz, vel_xyz):
    """Helper: build a sat that moves linearly with time.

    Position at epoch t = offset + vel × (t − t0).  t0 = arbitrary
    reference (we just need the *change* to be predictable).
    """
    t0_ref = datetime(2026, 5, 23, 12, 0, 0, tzinfo=timezone.utc)
    offset = np.asarray(offset_xyz, dtype=float)
    vel = np.asarray(vel_xyz, dtype=float)

    def _pos(_sv, t):
        dt_s = (t - t0_ref).total_seconds()
        return _LAB_ECEF + offset + vel * dt_s

    return (_pos, 0.0)


def _epoch_obs(svs, phi_offset_m=0.0, wl=0.190293673, lock_ms=10_000, cno=45.0,
               lli=0):
    """Build a list of obs dicts for the given SVs.

    `phi_offset_m` adds a constant phase (metres) to every SV — used
    to inject a synthetic c·Δdt_rx into the residual.  Returns the
    dicts; caller sets gps_time outside.

    A fixed non-zero baseline (1000 cycles ≈ 190 m) sits underneath
    `phi_offset_m` so cpMes is non-zero (estimator rejects literal
    zero as "no valid measurement").  The estimator only ever uses
    epoch-to-epoch *deltas*, so the baseline cancels.
    """
    _BASELINE_CYC = 1000.0
    obs = []
    for sv in svs:
        obs.append({
            "sv": sv,
            "phi1_cyc": _BASELINE_CYC + phi_offset_m / wl,
            "wl_f1": wl,
            "cno": cno,
            "lock_duration_ms": lock_ms,
            "lli": lli,
        })
    return obs


def _ts(secs_from_anchor):
    """Convenience: return a tz-aware datetime offset from a fixed anchor."""
    anchor = datetime(2026, 5, 23, 12, 0, 0, tzinfo=timezone.utc)
    return anchor + timedelta(seconds=secs_from_anchor)


# ── Test cases ────────────────────────────────────────────────────── #

class TdcpEstimatorBasics(unittest.TestCase):
    """Core math + ensemble behaviour."""

    def test_first_epoch_returns_nan_no_diff(self):
        """No previous epoch ⇒ no residual; result has NaN df_f."""
        eph = _MockEph({"G01": _stationary_sat([20e6, 0, 0])})
        est = TdcpEstimator(eph, _LAB_ECEF)
        r = est.update(_epoch_obs(["G01"]), _ts(0))
        self.assertEqual(r.n_used, 0)
        self.assertTrue(math.isnan(r.df_f))
        self.assertEqual(r.n_total, 1)

    def test_zero_clock_motion_stationary_sat(self):
        """Sat + rx clock both stationary → residual ≈ 0.

        Per-SV residual is ~mm-level from float arithmetic; ensemble
        median should be well under 1 cm.
        """
        eph = _MockEph({
            "G01": _stationary_sat([20e6, 0, 0]),
            "G02": _stationary_sat([0, 20e6, 0]),
            "G03": _stationary_sat([0, 0, 20e6]),
        })
        est = TdcpEstimator(eph, _LAB_ECEF)
        _ = est.update(_epoch_obs(["G01", "G02", "G03"]), _ts(0))
        r = est.update(_epoch_obs(["G01", "G02", "G03"]), _ts(1))
        self.assertEqual(r.n_used, 3)
        self.assertAlmostEqual(r.c_dt_rx_m, 0.0, places=4)
        self.assertAlmostEqual(r.df_f, 0.0, places=12)

    def test_injected_clock_step_recovered(self):
        """Add a synthetic c·Δdt_rx of 1 metre between epochs.

        Adding the same constant phase to every SV is mathematically
        identical to advancing the rx clock by 1/c seconds.  The
        ensemble median residual should equal 1 m.
        """
        eph = _MockEph({
            "G01": _stationary_sat([20e6, 0, 0]),
            "G02": _stationary_sat([0, 20e6, 0]),
            "G03": _stationary_sat([0, 0, 20e6]),
            "G04": _stationary_sat([15e6, 10e6, 0]),
        })
        est = TdcpEstimator(eph, _LAB_ECEF)
        _ = est.update(_epoch_obs(["G01", "G02", "G03", "G04"], phi_offset_m=0.0), _ts(0))
        r = est.update(_epoch_obs(["G01", "G02", "G03", "G04"], phi_offset_m=1.0), _ts(1))
        self.assertEqual(r.n_used, 4)
        self.assertAlmostEqual(r.c_dt_rx_m, 1.0, places=4)
        # df_f over 1 s of c·Δdt_rx = 1 m: 1/c = 3.336e-9
        self.assertAlmostEqual(r.df_f, 1.0 / C_LIGHT, places=14)

    def test_moving_sat_geometric_change_cancels(self):
        """A moving satellite produces a per-SV Δρ_geom; the estimator
        must subtract it via the ephemeris prediction so r(sv) ≈ 0.
        """
        # Sat moving at 3 km/s along x — about half typical orbital speed
        eph = _MockEph({
            "G01": _moving_sat([20e6, 0, 0], [3000.0, 0.0, 0.0]),
            "G02": _moving_sat([0, 20e6, 0], [0.0, 3000.0, 0.0]),
            "G03": _moving_sat([0, 0, 20e6], [0.0, 0.0, 3000.0]),
        })
        est = TdcpEstimator(eph, _LAB_ECEF)
        # cpMes for a stationary rx with a moving sat changes
        # by Δρ_geom / λ per epoch.  Compute and inject so we mimic
        # what the receiver would actually report when its own clock
        # is stationary.
        wl = 0.190293673
        # Build obs with phi adjusted to match Δρ_geom (perfect
        # carrier-phase tracking, no clock motion).
        def obs_for(t, prev_pos):
            """phi(sv,t) such that phi(sv, t) - phi(sv, t-1) =
            Δρ_geom(sv) / wl exactly.  Cumulative phase = ρ_geom / wl."""
            obs = []
            for sv in ["G01", "G02", "G03"]:
                pos, _clk = eph.sat_position(sv, t)
                # Use the same Sagnac-rotated range the estimator uses.
                rho = est._rotated_range(pos)
                obs.append({
                    "sv": sv,
                    "phi1_cyc": rho / wl,
                    "wl_f1": wl,
                    "cno": 45.0,
                    "lock_duration_ms": 10_000,
                    "lli": 0,
                })
            return obs

        _ = est.update(obs_for(_ts(0), None), _ts(0))
        r = est.update(obs_for(_ts(1), None), _ts(1))
        self.assertEqual(r.n_used, 3)
        # Pure geometric change should cancel.  Per-SV residual is
        # arithmetic-noise-level; ensemble well under 1 mm.
        self.assertAlmostEqual(r.c_dt_rx_m, 0.0, places=3)

    def test_three_mad_rejects_an_outlier_sv(self):
        """One SV produces a 100-metre residual relative to the
        ensemble.  3-MAD must drop it from `kept_svs`."""
        eph = _MockEph({
            "G01": _stationary_sat([20e6, 0, 0]),
            "G02": _stationary_sat([0, 20e6, 0]),
            "G03": _stationary_sat([0, 0, 20e6]),
            "G04": _stationary_sat([15e6, 10e6, 0]),
            "G05": _stationary_sat([10e6, 10e6, 10e6]),
        })
        est = TdcpEstimator(eph, _LAB_ECEF)
        _ = est.update(_epoch_obs(["G01", "G02", "G03", "G04", "G05"]), _ts(0))
        # Inject a single-SV 100 m phase punch on G05 only.
        obs2 = _epoch_obs(["G01", "G02", "G03", "G04"], phi_offset_m=0.0)
        # G05 gets +100 m phase relative to its prev — looks like
        # a 100 m clock jump on that SV alone (which is impossible)
        outlier = _epoch_obs(["G05"], phi_offset_m=100.0)
        r = est.update(obs2 + outlier, _ts(1))
        # 4 inliers + 1 outlier; 3-MAD should keep the 4 inliers.
        self.assertEqual(r.n_used, 4)
        self.assertNotIn("G05", r.kept_svs)
        # Ensemble should hover near zero (the inliers' residual).
        self.assertAlmostEqual(r.c_dt_rx_m, 0.0, places=3)


class TdcpEstimatorSlips(unittest.TestCase):
    """L1 slip-handling: locktime drops, LLI bit, external monitor."""

    def _three_stationary_eph(self):
        return _MockEph({
            "G01": _stationary_sat([20e6, 0, 0]),
            "G02": _stationary_sat([0, 20e6, 0]),
            "G03": _stationary_sat([0, 0, 20e6]),
        })

    def test_locktime_drop_suppresses_sv_diff_this_epoch(self):
        """SV's lock_duration_ms drops by > 500 ms ⇒ diff suppressed
        this epoch.  Other SVs still contribute.
        """
        est = TdcpEstimator(self._three_stationary_eph(), _LAB_ECEF)
        _ = est.update(_epoch_obs(["G01", "G02", "G03"], lock_ms=10_000), _ts(0))
        # G01 locktime drops 10s → 5s (5000 ms drop, way over 500)
        obs2 = _epoch_obs(["G02", "G03"], lock_ms=11_000)
        obs2 += _epoch_obs(["G01"], lock_ms=5_000)
        r = est.update(obs2, _ts(1))
        # G01 dropped; G02 + G03 contribute.
        self.assertEqual(r.n_used, 2)
        self.assertIn("G02", r.kept_svs)
        self.assertIn("G03", r.kept_svs)
        self.assertNotIn("G01", r.kept_svs)

    def test_lli_bit_zero_suppresses_sv_diff_this_epoch(self):
        """LLI bit 0 = loss-of-lock ⇒ diff suppressed."""
        est = TdcpEstimator(self._three_stationary_eph(), _LAB_ECEF)
        _ = est.update(_epoch_obs(["G01", "G02", "G03"]), _ts(0))
        obs2 = _epoch_obs(["G01", "G02"]) + _epoch_obs(["G03"], lli=1)
        r = est.update(obs2, _ts(1))
        self.assertEqual(r.n_used, 2)
        self.assertNotIn("G03", r.kept_svs)

    def test_slip_clears_after_one_clean_epoch(self):
        """After a slip epoch, the next clean epoch resumes diffing
        against the post-slip phase — the slip taints exactly one
        epoch of differential, not two.
        """
        est = TdcpEstimator(self._three_stationary_eph(), _LAB_ECEF)
        _ = est.update(_epoch_obs(["G01", "G02", "G03"]), _ts(0))
        # Slip on G01
        obs2 = _epoch_obs(["G02", "G03"]) + _epoch_obs(["G01"], lli=1)
        _ = est.update(obs2, _ts(1))
        # Clean epoch — G01 should now diff against epoch-1's data
        # (post-slip phase), not epoch-0.
        r = est.update(_epoch_obs(["G01", "G02", "G03"]), _ts(2))
        self.assertEqual(r.n_used, 3)
        self.assertIn("G01", r.kept_svs)

    def test_external_slip_monitor_drops_flagged_sv(self):
        """If a slip_monitor flags an SV, it's excluded this epoch."""

        class _Stub:
            def check(self, observations, t_mono, epoch, **_):
                # Flag G02 every epoch
                class _Ev:
                    def __init__(self, sv):
                        self.sv = sv
                return [_Ev("G02")]

        est = TdcpEstimator(self._three_stationary_eph(), _LAB_ECEF,
                            slip_monitor=_Stub())
        _ = est.update(_epoch_obs(["G01", "G02", "G03"]), _ts(0))
        r = est.update(_epoch_obs(["G01", "G02", "G03"]), _ts(1))
        self.assertEqual(r.n_used, 2)
        self.assertNotIn("G02", r.kept_svs)


class TdcpEstimatorEdgeCases(unittest.TestCase):

    def test_sat_with_missing_ephemeris_is_skipped(self):
        eph = _MockEph({
            "G01": _stationary_sat([20e6, 0, 0]),
            # G02 deliberately absent
        })
        est = TdcpEstimator(eph, _LAB_ECEF)
        _ = est.update(_epoch_obs(["G01", "G02"]), _ts(0))
        r = est.update(_epoch_obs(["G01", "G02"]), _ts(1))
        self.assertEqual(r.n_used, 1)
        self.assertIn("G01", r.kept_svs)

    def test_zero_phase_is_skipped(self):
        """Receiver sometimes reports cpMes=0 (no valid measurement).
        Estimator must skip those rather than producing garbage.
        """
        eph = _MockEph({"G01": _stationary_sat([20e6, 0, 0])})
        est = TdcpEstimator(eph, _LAB_ECEF)
        obs = [{
            "sv": "G01", "phi1_cyc": 0.0, "wl_f1": 0.19,
            "cno": 0.0, "lock_duration_ms": 0, "lli": 0,
        }]
        r = est.update(obs, _ts(0))
        self.assertEqual(r.n_used, 0)
        self.assertEqual(r.n_total, 1)

    def test_epoch_gap_over_max_dt_skips_diff(self):
        """Gap of 5 s with max_dt_s=1.5 ⇒ no diff."""
        eph = _MockEph({
            "G01": _stationary_sat([20e6, 0, 0]),
            "G02": _stationary_sat([0, 20e6, 0]),
        })
        est = TdcpEstimator(eph, _LAB_ECEF, max_dt_s=1.5)
        _ = est.update(_epoch_obs(["G01", "G02"]), _ts(0))
        r = est.update(_epoch_obs(["G01", "G02"]), _ts(5))
        self.assertEqual(r.n_used, 0)
        self.assertTrue(math.isnan(r.df_f))

    def test_no_observations_returns_empty_result(self):
        eph = _MockEph({})
        est = TdcpEstimator(eph, _LAB_ECEF)
        r = est.update([], _ts(0))
        self.assertEqual(r.n_total, 0)
        self.assertEqual(r.n_used, 0)

    def test_phi1_raw_cyc_preferred_over_phi1_cyc(self):
        """SSR phase-bias steps drop integer cycles into `phi1_cyc`
        without changing the underlying carrier phase — a 1-cycle L1
        step = 0.19 m / c / 1 s = 0.635 ppb single-epoch frequency
        punch = 4.9σ on default --tdcp-sigma-ppb (63σ on L2 gate
        σ_floor).  The estimator must read
        `phi1_raw_cyc` (the receiver's uncorrected carrier phase) when
        available so the SSR PB stream doesn't propagate into Arm 5.

        Mirrors `cycle_slip.py:286` fallback pattern.
        """
        eph = _MockEph({"G01": _stationary_sat([20e6, 0, 0])})
        est = TdcpEstimator(eph, _LAB_ECEF)
        wl = 0.190293673
        baseline = 1000.0  # cycles

        # Epoch 0: phi1_cyc=phi1_raw_cyc=baseline (no PB yet).
        obs0 = [{
            "sv": "G01",
            "phi1_cyc": baseline, "phi1_raw_cyc": baseline,
            "wl_f1": wl, "cno": 45.0, "lock_duration_ms": 10_000, "lli": 0,
        }]
        _ = est.update(obs0, _ts(0))

        # Epoch 1: phi1_cyc has a 1-cycle SSR PB step.  phi1_raw_cyc
        # tracks the actual physics (no step — same as epoch 0 +
        # whatever small motion).  TDCP must follow phi1_raw_cyc.
        obs1 = [{
            "sv": "G01",
            "phi1_cyc": baseline + 1.0,     # PB step (1 full cycle)
            "phi1_raw_cyc": baseline,        # truth, unchanged
            "wl_f1": wl, "cno": 45.0, "lock_duration_ms": 10_000, "lli": 0,
        }]
        r = est.update(obs1, _ts(1))
        # If the fix is in place: TDCP residual ≈ 0 (no real motion).
        # If we had been reading phi1_cyc directly: residual = 1 cycle =
        # 0.190 m / c / 1 s → df_f ≈ 6.3e-10 (0.635 ppb).
        self.assertEqual(r.n_used, 1)
        self.assertLess(
            abs(r.df_f), 1e-11,
            "TDCP must not pick up SSR PB step (phi1_raw_cyc fallback)",
        )

    def test_phi1_cyc_used_when_phi1_raw_cyc_absent(self):
        """Backward compatibility: if the receiver-reader doesn't emit
        `phi1_raw_cyc` (older code path), TdcpEstimator must still work
        with `phi1_cyc` alone — same as before this fallback landed.
        """
        eph = _MockEph({"G01": _stationary_sat([20e6, 0, 0])})
        est = TdcpEstimator(eph, _LAB_ECEF)
        # _epoch_obs only emits phi1_cyc; no phi1_raw_cyc key.
        _ = est.update(_epoch_obs(["G01"]), _ts(0))
        r = est.update(_epoch_obs(["G01"]), _ts(1))
        self.assertEqual(r.n_used, 1)
        self.assertTrue(math.isfinite(r.df_f))


if __name__ == "__main__":
    unittest.main()

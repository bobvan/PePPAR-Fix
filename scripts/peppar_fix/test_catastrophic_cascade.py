"""Tests for the engine-integration catastrophic-reject cascade
(I-202649 Note A + B follow-up).

The FixedPosFilter exposes _consecutive_catastrophic_rejects.  When
the engine sees that counter reach CATASTROPHIC_REJECT_LIMIT (30 by
default), it logs an error, transitions DOFreqEst → HOLDOVER,
flips servo_ctx['phc_diverged'] = True, and returns exit code 5
for wrapper-driven re-bootstrap.

Test scope: the FILTER's counter behaviour at the limit boundary.
The engine-side wiring (skip_stats['catastrophic'] increment, exit
code 5, dfe_sm transition, servo_ctx flip) lives in
peppar_fix_engine.run_steady_state — not directly testable on the
dev box without pyserial.  Lab smoke validates the engine path.
"""
from __future__ import annotations

import math
import os
import sys
import unittest
from datetime import datetime, timedelta, timezone
from unittest.mock import MagicMock

import numpy as np

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from solve_ppp import FixedPosFilter

_BASE_ECEF = np.array([157470.222, -4756189.544, 4232767.952])


def _sat_pos_above_site(elev_deg, az_deg=0.0, range_m=22e6):
    e = math.radians(elev_deg)
    a = math.radians(az_deg)
    up = _BASE_ECEF / np.linalg.norm(_BASE_ECEF)
    east = np.cross(np.array([0., 0., 1.]), up)
    east /= np.linalg.norm(east)
    north = np.cross(up, east)
    site_frame = (math.cos(e) * (math.sin(a) * east + math.cos(a) * north)
                  + math.sin(e) * up)
    return _BASE_ECEF + range_m * site_frame


def _make_obs(sv, sys_name, pr_if_m, phi_if_m=None, cno_db=45.0):
    return {
        'sv': sv,
        'sys': sys_name,
        'pr_if': float(pr_if_m),
        'phi_if_m': None if phi_if_m is None else float(phi_if_m),
        'cno': float(cno_db),
    }


def _build_filter_with_history(n_warmup_epochs=8, pr_offset_m=0.5):
    """Initialized FixedPosFilter past warmup + clock seeded.  Same
    helper shape as test_catastrophic_gate.py uses."""
    f = FixedPosFilter(_BASE_ECEF)
    f.x[FixedPosFilter.IDX_CLK] = 0.0
    f.P[FixedPosFilter.IDX_CLK, FixedPosFilter.IDX_CLK] = 100.0 ** 2
    f.initialized = True
    f.prev_clock = 0.0
    sat_positions = {
        "G01": _sat_pos_above_site(60.0, az_deg=0.0),
        "G02": _sat_pos_above_site(45.0, az_deg=90.0),
        "G03": _sat_pos_above_site(30.0, az_deg=180.0),
        "G04": _sat_pos_above_site(70.0, az_deg=270.0),
    }
    sp3 = MagicMock()
    sp3.sat_position = MagicMock(
        side_effect=lambda sv, t_tx: (
            np.asarray(sat_positions[sv], dtype=float), 0.0))
    ranges = {sv: float(np.linalg.norm(sat_positions[sv] - _BASE_ECEF))
              for sv in sat_positions}
    t = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
    for k in range(n_warmup_epochs):
        obs = [_make_obs(sv, "gps",
                         ranges[sv] + pr_offset_m,
                         phi_if_m=ranges[sv] + 0.001)
               for sv in sat_positions]
        f.update(obs, sp3, t + timedelta(seconds=k), clk_file=None)
    return f, sp3, ranges, t + timedelta(seconds=n_warmup_epochs - 1)


class CounterAccumulationTest(unittest.TestCase):
    """Pin the counter behaviour the engine integrates against."""

    def test_class_attribute_default(self):
        """CATASTROPHIC_REJECT_LIMIT default must match the engine's
        cascade trigger.  30 = analog of 30-outlier servo cascade."""
        self.assertEqual(FixedPosFilter.CATASTROPHIC_REJECT_LIMIT, 30)

    def test_counter_starts_zero(self):
        f = FixedPosFilter(_BASE_ECEF)
        self.assertEqual(f._consecutive_catastrophic_rejects, 0)

    def test_burst_increments_counter_each_epoch(self):
        """Sustained burst → counter increments by 1 per rejected
        epoch.  Engine reads this after each filt.update() call to
        decide cascade exit."""
        f, sp3, ranges, t = _build_filter_with_history()
        # Burst of 5 consecutive rejected epochs.
        for k in range(5):
            obs = [_make_obs(sv, "gps",
                             ranges[sv] + 339.0,
                             phi_if_m=ranges[sv] + 339.0)
                   for sv in ranges]
            n_pr, _, _ = f.update(obs, sp3,
                                   t + timedelta(seconds=k + 1),
                                   clk_file=None)
            self.assertEqual(n_pr, 0,
                             f"epoch {k}: expected reject, got n_pr={n_pr}")
            self.assertEqual(
                f._consecutive_catastrophic_rejects, k + 1,
                f"epoch {k}: counter should be {k+1}",
            )

    def test_counter_resets_to_zero_on_clean_accept(self):
        """A clean-residual epoch after a burst resets the counter.
        Engine relies on this for the limit-resets-on-recovery
        semantic — sustained-burst-then-recovery should NOT keep
        counting past the limit if recovery happens earlier."""
        f, sp3, ranges, t = _build_filter_with_history()
        # 3 rejected epochs.
        for k in range(3):
            obs = [_make_obs(sv, "gps",
                             ranges[sv] + 339.0,
                             phi_if_m=ranges[sv] + 339.0)
                   for sv in ranges]
            f.update(obs, sp3, t + timedelta(seconds=k + 1),
                     clk_file=None)
        self.assertEqual(f._consecutive_catastrophic_rejects, 3)
        # Clean epoch.
        obs = [_make_obs(sv, "gps",
                         ranges[sv] + 0.5,
                         phi_if_m=ranges[sv] + 0.001)
               for sv in ranges]
        f.update(obs, sp3, t + timedelta(seconds=4), clk_file=None)
        self.assertEqual(f._consecutive_catastrophic_rejects, 0)

    def test_counter_reaches_limit_under_sustained_burst(self):
        """Counter increments past CATASTROPHIC_REJECT_LIMIT under
        sustained burst.  The engine's cascade trigger fires when
        counter >= limit; this test pins the limit-crossing for the
        engine wiring to detect."""
        f, sp3, ranges, t = _build_filter_with_history()
        # Burst long enough to hit the limit + 5.
        n_burst = FixedPosFilter.CATASTROPHIC_REJECT_LIMIT + 5
        for k in range(n_burst):
            obs = [_make_obs(sv, "gps",
                             ranges[sv] + 339.0,
                             phi_if_m=ranges[sv] + 339.0)
                   for sv in ranges]
            f.update(obs, sp3, t + timedelta(seconds=k + 1),
                     clk_file=None)
        self.assertEqual(f._consecutive_catastrophic_rejects, n_burst)
        # The limit-crossing is what the engine cascade detects.
        self.assertGreaterEqual(
            f._consecutive_catastrophic_rejects,
            FixedPosFilter.CATASTROPHIC_REJECT_LIMIT,
        )

    def test_n_pr_zero_only_when_counter_active(self):
        """Engine integration relies on distinguishing the catastrophic
        path from the n_pr<1 / no-geometry path.  Both return n_used=0
        but only the catastrophic path advances the counter.  This test
        pins that semantic — engine code reads
        (n_used == 0 AND counter > 0) as 'this epoch was gate-rejected'."""
        f, sp3, ranges, t = _build_filter_with_history()
        # Catastrophic reject — counter > 0.
        obs = [_make_obs(sv, "gps",
                         ranges[sv] + 339.0,
                         phi_if_m=ranges[sv] + 339.0)
               for sv in ranges]
        n_pr, _, _ = f.update(obs, sp3, t + timedelta(seconds=1),
                               clk_file=None)
        self.assertEqual(n_pr, 0)
        self.assertGreater(f._consecutive_catastrophic_rejects, 0)
        # Now an empty observation list → n_pr<1 path.  Counter should
        # NOT advance — this isn't the gate firing.
        n_pr2, _, _ = f.update([], sp3, t + timedelta(seconds=2),
                                clk_file=None)
        self.assertEqual(n_pr2, 0)
        # Counter remains at the value from the prior reject.  Important:
        # the engine's "was this catastrophic?" check is "n_used==0 AND
        # counter advanced THIS call" — verified by tracking before/after,
        # OR equivalently by checking counter > 0 on the immediate call.
        # Empty-obs path doesn't reset the counter to zero (no accept
        # path ran), but it also doesn't increment it.
        self.assertEqual(f._consecutive_catastrophic_rejects, 1,
                         "empty-obs n_pr<1 path must NOT advance counter")


class GapRecoveryTest(unittest.TestCase):
    """gap_recovery absorbs a post-gap common-mode clock drift instead of
    cascading into the catastrophic gate (clkpoc3GapAbsorb).

    Reproduces the clkpoc3F9tInputReboot mechanism: a long obs gap leaves
    the receiver clock drifted ~clkD·dt un-tracked, so every post-gap PR
    residual carries the same large common-mode offset.  Without the flag
    that trips the catastrophic gate 30× → exit-5.  With it, the offset is
    absorbed into dt_rx and the epoch is accepted.
    """

    # ~8 µs of clock drift over a 32 s gap at clkD≈260 ns/s — the real
    # clkPoC3 value (2415 m measured on day0528-xhost).
    _GAP_DRIFT_M = 2415.0

    def _gap_obs(self, ranges):
        return [_make_obs(sv, "gps",
                          ranges[sv] + self._GAP_DRIFT_M,
                          phi_if_m=ranges[sv] + self._GAP_DRIFT_M)
                for sv in ranges]

    def test_gap_offset_without_flag_cascades(self):
        """Control: the same common-mode offset WITHOUT gap_recovery is
        catastrophic-rejected — the gate still protects normal operation."""
        f, sp3, ranges, t = _build_filter_with_history()
        n_pr, _, _ = f.update(self._gap_obs(ranges),
                              sp3, t + timedelta(seconds=1), clk_file=None)
        self.assertEqual(n_pr, 0, "offset without flag should be rejected")
        self.assertEqual(f._consecutive_catastrophic_rejects, 1)

    def test_gap_recovery_absorbs_and_accepts(self):
        """With gap_recovery=True the offset is absorbed into dt_rx, the
        epoch is accepted (n_used>0), and the catastrophic counter stays 0."""
        f, sp3, ranges, t = _build_filter_with_history()
        clk_before = f.x[FixedPosFilter.IDX_CLK]
        n_pr, _, _ = f.update(self._gap_obs(ranges),
                              sp3, t + timedelta(seconds=1),
                              clk_file=None, gap_recovery=True)
        self.assertGreater(n_pr, 0, "gap-recovery epoch must be accepted")
        self.assertEqual(f._consecutive_catastrophic_rejects, 0)
        # dt_rx re-anchored by ~the drift (within the post-update Kalman
        # correction; the absorb shifts the bulk, the update trims it).
        clk_moved = f.x[FixedPosFilter.IDX_CLK] - clk_before
        self.assertAlmostEqual(clk_moved, self._GAP_DRIFT_M, delta=50.0)

    def test_gap_recovery_clears_prior_reject_counter(self):
        """A gap-recovery epoch after a few near-miss rejects resets the
        counter — a transient pre-gap burst must not push the post-gap
        re-anchor over the exit-5 limit."""
        f, sp3, ranges, t = _build_filter_with_history()
        for k in range(3):
            f.update(self._gap_obs(ranges), sp3,
                     t + timedelta(seconds=k + 1), clk_file=None)
        self.assertEqual(f._consecutive_catastrophic_rejects, 3)
        f.update(self._gap_obs(ranges), sp3,
                 t + timedelta(seconds=4), clk_file=None, gap_recovery=True)
        self.assertEqual(f._consecutive_catastrophic_rejects, 0)

    def test_gap_recovery_drops_stale_tdcp_baseline(self):
        """gap_recovery drops the pre-gap TD-CP baseline (a >30 s baseline
        may straddle undetected slips); the epoch re-anchors on PR alone
        (n_td == 0) and rebuilds a clean baseline for the next epoch."""
        f, sp3, ranges, t = _build_filter_with_history()
        self.assertTrue(f.prev_geo, "warmup should leave a TD-CP baseline")
        _, _, n_td = f.update(self._gap_obs(ranges), sp3,
                              t + timedelta(seconds=1),
                              clk_file=None, gap_recovery=True)
        self.assertEqual(n_td, 0, "no TD-CP rows on the gap-recovery epoch")
        self.assertTrue(f.prev_geo, "baseline rebuilt for the next epoch")


if __name__ == "__main__":
    unittest.main()

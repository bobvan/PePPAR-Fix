"""Tests for FixedPosFilter F9T clkReset-driven receiver-clock realignment.

Per ZED-F9P Interface Description (UBX-18010854 §5.15.3 page 190):

    "clkReset: Clock reset applied. Typically the receiver clock is
     changed in increments of integer milliseconds."

The F9T's local clock is approximately GPS-aligned but bounded; when
drift accumulates it steps in integer-ms increments, applying the same
shift to all PR and CP measurements that epoch.  Before chipSlipHandling
landed, the catastrophic-reject gate treated the resulting ~6.3 Mm PR
residuals as an anomaly (median |PR| >> 20× baseline) and cascaded the
engine to exit-5 after 30 consecutive epochs.

The fix: when clk_reset=True is passed to FixedPosFilter.update(),
the filter recognizes the documented event, measures the integer-ms
shift from the signed median PR residual, absorbs the offset into
dt_rx, resets the TD-CP baselines + PR median history + catastrophic
counter, and proceeds with the normal Kalman update.

Verified empirically: clkPoC3 2026-05-25 19:00:30 — exactly one
clkReset=1 event in 56,751 RXM-RAWX messages from a 22h capture, at
the precise moment of the catastrophic-reject cascade, with iTOW
jumping from 172848011 → 172848990 (a 21.021 ms shift) matching the
PR residual signature reported as 21.000 ms.
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
_MS_C_M = 299_792.458  # 1 ms × c


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
    """Initialized FixedPosFilter with warmup history."""
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
               for sv in ranges]
        f.update(obs, sp3, t + timedelta(seconds=k), clk_file=None)
    return f, sp3, ranges, t + timedelta(seconds=n_warmup_epochs - 1)


class ClkRealignTest(unittest.TestCase):

    def test_clk_reset_absorbs_21ms_shift_into_dt_rx(self):
        """The clkPoC3 2026-05-25 scenario: all 4 SVs show +21 ms × c
        ≈ +6,295,640 m of PR residual.  With clk_reset=True, the gate
        is bypassed, dt_rx is shifted by +21 × c, residuals are
        rebased, the normal Kalman update proceeds with small post-
        realignment residuals, and the catastrophic counter is reset."""
        f, sp3, ranges, t_last = _build_filter_with_history()
        clk_before = float(f.x[FixedPosFilter.IDX_CLK])
        # Inject a +21 ms shift on all SVs — F9T-style clock realignment.
        shift_m = 21 * _MS_C_M
        obs = [_make_obs(sv, "gps",
                         ranges[sv] + shift_m,
                         phi_if_m=ranges[sv] + shift_m)
               for sv in ranges]
        n_pr, z, n_td = f.update(
            obs, sp3, t_last + timedelta(seconds=1),
            clk_file=None, clk_reset=True)
        clk_after = float(f.x[FixedPosFilter.IDX_CLK])
        # dt_rx absorbed the +21 ms (within a few meters of the
        # geometric noise floor).
        delta = clk_after - clk_before - shift_m
        self.assertLess(abs(delta), 50.0,
                        f"dt_rx absorbed offset off by {delta:.1f} m")
        # Catastrophic counter was NOT incremented.
        self.assertEqual(f._consecutive_catastrophic_rejects, 0)
        # PR median history was cleared by the realignment branch and
        # then the accept path appends the fresh post-shift sample —
        # so length is exactly 1, and the value is small (post-shift
        # residual, not the pre-shift 6 Mm).
        self.assertEqual(len(f._pr_median_history), 1)
        self.assertLess(f._pr_median_history[-1], 50.0,
                        "post-shift sample should be at baseline noise")
        # prev_geo / prev_clock are re-set by the normal accept path to
        # the post-realignment values — so they're non-None at end of
        # update, and ready to serve as TD-CP baseline for next epoch.
        self.assertIsNotNone(f.prev_geo)
        self.assertIsNotNone(f.prev_clock)
        # Update produced a non-empty result (gate did not reject).
        self.assertGreater(n_pr, 0)

    def test_clk_reset_negative_shift(self):
        """F9T can realign in either direction; verify -21 ms also
        absorbed correctly (the clkPoC3 event was actually backward)."""
        f, sp3, ranges, t_last = _build_filter_with_history()
        clk_before = float(f.x[FixedPosFilter.IDX_CLK])
        shift_m = -21 * _MS_C_M
        obs = [_make_obs(sv, "gps",
                         ranges[sv] + shift_m,
                         phi_if_m=ranges[sv] + shift_m)
               for sv in ranges]
        f.update(obs, sp3, t_last + timedelta(seconds=1),
                 clk_file=None, clk_reset=True)
        clk_after = float(f.x[FixedPosFilter.IDX_CLK])
        delta = clk_after - clk_before - shift_m
        self.assertLess(abs(delta), 50.0)

    def test_clk_reset_false_keeps_existing_gate_behavior(self):
        """Without the clk_reset flag, a 21-ms PR injection should
        still trip the catastrophic gate exactly as before — the new
        code path is opt-in via the flag."""
        f, sp3, ranges, t_last = _build_filter_with_history()
        shift_m = 21 * _MS_C_M
        obs = [_make_obs(sv, "gps",
                         ranges[sv] + shift_m,
                         phi_if_m=ranges[sv] + shift_m)
               for sv in ranges]
        # Default clk_reset=False — gate fires as before.
        n_pr, z, n_td = f.update(
            obs, sp3, t_last + timedelta(seconds=1), clk_file=None)
        self.assertEqual(n_pr, 0)
        self.assertEqual(len(z), 0)
        self.assertEqual(f._consecutive_catastrophic_rejects, 1)

    def test_clk_reset_with_tiny_offset_is_noop(self):
        """clk_reset=True but residuals don't round to any integer-ms
        shift: filter should not modify dt_rx and should proceed with
        a normal update (the F9T occasionally sets clkReset without a
        large measurable shift — defensive behavior)."""
        f, sp3, ranges, t_last = _build_filter_with_history()
        clk_before = float(f.x[FixedPosFilter.IDX_CLK])
        # Sub-meter residuals: median PR / (1 ms × c) rounds to 0
        obs = [_make_obs(sv, "gps",
                         ranges[sv] + 0.5,
                         phi_if_m=ranges[sv] + 0.001)
               for sv in ranges]
        f.update(obs, sp3, t_last + timedelta(seconds=1),
                 clk_file=None, clk_reset=True)
        clk_after = float(f.x[FixedPosFilter.IDX_CLK])
        # dt_rx unchanged by clk_reset path (the Kalman update may move
        # it slightly via the normal innovation, but the realignment
        # branch itself contributed nothing).
        self.assertLess(abs(clk_after - clk_before), 100.0)

    def test_clk_reset_resets_catastrophic_counter(self):
        """If a near-miss catastrophic event left the counter at, say,
        5, a clean clk_reset event should reset it (the realignment is
        a known good event that interrupts any unrelated near-miss
        sequence)."""
        f, sp3, ranges, t_last = _build_filter_with_history()
        # Set up a stale counter from a prior epoch.
        f._consecutive_catastrophic_rejects = 7
        shift_m = 1 * _MS_C_M  # 1 ms realignment
        obs = [_make_obs(sv, "gps",
                         ranges[sv] + shift_m,
                         phi_if_m=ranges[sv] + shift_m)
               for sv in ranges]
        f.update(obs, sp3, t_last + timedelta(seconds=1),
                 clk_file=None, clk_reset=True)
        self.assertEqual(f._consecutive_catastrophic_rejects, 0)


if __name__ == "__main__":
    unittest.main()

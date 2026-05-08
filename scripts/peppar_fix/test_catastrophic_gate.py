"""Tests for FixedPosFilter catastrophic residual gate (I-202649).

The gate rejects an entire epoch when the input observations show
catastrophic PR residuals (e.g. F9T serial-burst corruption that
injects coordinated +339m residuals on multiple SVs).  MAD outlier
rejection above the gate catches isolated outliers; this gate
catches coordinated catastrophes that MAD-vs-median sees as
agreeing with each other.

Motivated by MadHat's death 2026-05-08 14:49 UTC.  See
project_madhat_death_serial_burst_20260508.md.
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


def _build_filter_with_initialized_state():
    """Initialized FixedPosFilter with a clean clock state and 4 SVs in prev_geo."""
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
    obs1 = [_make_obs(sv, "gps", ranges[sv], phi_if_m=ranges[sv])
            for sv in sat_positions]
    t = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
    f.update(obs1, sp3, t, clk_file=None)
    return f, sp3, ranges, t


class CatastrophicGateTest(unittest.TestCase):

    def test_clean_observations_pass_through(self):
        """A normal epoch with sub-meter residuals should NOT trip the gate.

        The gate must not interfere with healthy operation — that's
        the whole point of choosing a 100m median threshold."""
        f, sp3, ranges, t = _build_filter_with_initialized_state()
        x_before = f.x.copy()
        # Second epoch with realistic m-scale offsets (well below 50m).
        obs2 = [_make_obs(sv, "gps",
                          ranges[sv] + 0.5,
                          phi_if_m=ranges[sv] + 0.001)
                for sv in ranges]
        t2 = t + timedelta(seconds=1)
        f.update(obs2, sp3, t2, clk_file=None)
        # Update happened — state should have moved, prev_clock updated.
        self.assertTrue(np.any(f.x != x_before),
                        "clean update should move state")
        # 4 PR residuals exposed for downstream visibility.
        self.assertEqual(f.last_n_pr, 4)

    def test_coordinated_339m_burst_rejects(self):
        """The MadHat 2026-05-08 14:49 failure mode: all 3 SVs
        simultaneously show PR residuals at +339m.  The gate
        must reject the epoch and leave state untouched."""
        f, sp3, ranges, t = _build_filter_with_initialized_state()
        x_before = f.x.copy()
        prev_clock_before = f.prev_clock
        # All 4 SVs corrupted at +339m.  MAD-vs-median doesn't fire
        # because the residuals all agree with each other.
        obs2 = [_make_obs(sv, "gps",
                          ranges[sv] + 339.0,
                          phi_if_m=ranges[sv] + 339.0)
                for sv in ranges]
        t2 = t + timedelta(seconds=1)
        n_pr, z, n_td = f.update(obs2, sp3, t2, clk_file=None)
        # Gate rejected — return value is the n_pr=0 / empty-z signal.
        self.assertEqual(n_pr, 0)
        self.assertEqual(len(z), 0)
        # State must NOT have moved.
        np.testing.assert_array_equal(f.x, x_before)
        # prev_clock must NOT have updated (would poison next epoch's
        # time-differenced phase if it did).
        self.assertEqual(f.prev_clock, prev_clock_before)
        # Pre-fit residuals exposed for diagnostic visibility.
        self.assertEqual(f.last_n_pr, 4)
        self.assertEqual(len(f.last_resid_pr), 4)
        # All four PR residuals should be ~339m magnitude.
        self.assertTrue(np.all(np.abs(f.last_resid_pr) > 100.0))

    def test_one_bad_sv_among_clean_passes_through(self):
        """A single bad SV among 4 clean ones should NOT trip the gate.

        n_huge=1 of 4 (25%) is below the 50% count-fraction threshold,
        and median |PR| is dominated by the 3 clean ones.  Existing
        MAD outlier rejection (when enabled by caller) would catch
        the lone outlier on its own."""
        f, sp3, ranges, t = _build_filter_with_initialized_state()
        x_before = f.x.copy()
        # 3 clean + 1 corrupted at +339m.
        obs2 = []
        for i, sv in enumerate(ranges):
            offset = 339.0 if i == 0 else 0.5
            obs2.append(_make_obs(sv, "gps",
                                  ranges[sv] + offset,
                                  phi_if_m=ranges[sv] + 0.001))
        t2 = t + timedelta(seconds=1)
        n_pr, z, n_td = f.update(obs2, sp3, t2, clk_file=None)
        # Gate did NOT fire — update happened.
        self.assertGreater(n_pr, 0)
        self.assertTrue(np.any(f.x != x_before),
                        "1-of-4 outlier should not trip the gate; update "
                        "should still happen (MAD is the right defence)")

    def test_three_of_four_at_60m_trips_count(self):
        """3 of 4 PR residuals at 60m (above 50m threshold) is 75% —
        exceeds 50% count fraction.  Gate fires on count, not median.

        Median |PR| of [60, 60, 60, 0.5] = 60m, which IS above 100m?
        No — median is 60m, well below the 100m median threshold.
        So this test exercises the count-trip path independently."""
        f, sp3, ranges, t = _build_filter_with_initialized_state()
        x_before = f.x.copy()
        obs2 = []
        for i, sv in enumerate(ranges):
            offset = 60.0 if i < 3 else 0.5
            obs2.append(_make_obs(sv, "gps",
                                  ranges[sv] + offset,
                                  phi_if_m=ranges[sv] + 0.001))
        t2 = t + timedelta(seconds=1)
        n_pr, z, n_td = f.update(obs2, sp3, t2, clk_file=None)
        # Gate fired on count.
        self.assertEqual(n_pr, 0)
        np.testing.assert_array_equal(f.x, x_before)


if __name__ == "__main__":
    unittest.main()

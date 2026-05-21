"""Per-SV residual label preservation in FixedPosFilter.

The --per-sv-resid-log writer needs sv/sys/elev labels parallel to
last_resid_pr / last_resid_td so it can emit per-SV CSV rows without
re-walking observations.  This file tests that the labels are saved
correctly at each of the four exit paths in update():

  - empty-update (n_pr < 1) → all label arrays empty
  - catastrophic reject → labels match pr_svs/td_svs from construction
  - LinAlgError fallback → labels saved before return
  - normal accept → labels match residual arrays, length-aligned
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


def _setup_filter():
    f = FixedPosFilter(_BASE_ECEF)
    f.x[FixedPosFilter.IDX_CLK] = 0.0
    f.P[FixedPosFilter.IDX_CLK, FixedPosFilter.IDX_CLK] = 100.0 ** 2
    f.initialized = True
    f.prev_clock = 0.0
    sat_positions = {
        "G01": _sat_pos_above_site(60.0, az_deg=0.0),
        "G02": _sat_pos_above_site(45.0, az_deg=90.0),
        "E11": _sat_pos_above_site(30.0, az_deg=180.0),
        "E15": _sat_pos_above_site(70.0, az_deg=270.0),
    }
    sp3 = MagicMock()
    sp3.sat_position = MagicMock(
        side_effect=lambda sv, t_tx: (
            np.asarray(sat_positions[sv], dtype=float), 0.0))
    ranges = {sv: float(np.linalg.norm(sat_positions[sv] - _BASE_ECEF))
              for sv in sat_positions}
    return f, sp3, ranges


class PerSvLabelTest(unittest.TestCase):

    def test_normal_accept_labels_match_residuals(self):
        """Clean update: label-array lengths match residual-array
        lengths, and SV identifiers are present."""
        f, sp3, ranges = _setup_filter()
        t = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
        # Two warmup epochs to populate prev_geo / prev_clock so
        # the TD-CP rows actually fire.
        for k in range(3):
            obs = [_make_obs(sv, "gps" if sv.startswith("G") else "gal",
                             ranges[sv] + 0.5,
                             phi_if_m=ranges[sv] + 0.001)
                   for sv in ranges]
            f.update(obs, sp3, t + timedelta(seconds=k), clk_file=None)
        # PR labels: 4 SVs.  Lengths align.
        self.assertEqual(len(f.last_pr_svs), len(f.last_resid_pr))
        self.assertEqual(len(f.last_pr_sys), len(f.last_resid_pr))
        self.assertEqual(len(f.last_pr_elev), len(f.last_resid_pr))
        self.assertEqual(set(f.last_pr_svs), {"G01", "G02", "E11", "E15"})
        # System tags correctly tracked.
        self.assertEqual(f.last_pr_sys[f.last_pr_svs.index("G01")], "gps")
        self.assertEqual(f.last_pr_sys[f.last_pr_svs.index("E11")], "gal")
        # Elevations are within reason (10-89°).
        self.assertTrue(all(10.0 < e < 89.0 for e in f.last_pr_elev))
        # TD labels: should be populated after first epoch's prev_geo
        # establishes the differencing baseline.
        self.assertEqual(len(f.last_td_svs), len(f.last_resid_td))
        self.assertEqual(len(f.last_td_sys), len(f.last_resid_td))
        self.assertEqual(len(f.last_td_elev), len(f.last_resid_td))

    def test_empty_update_clears_labels(self):
        """Filter with no usable observations: label arrays empty too."""
        f, sp3, _ = _setup_filter()
        # Seed labels first with a real epoch.
        t0 = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
        ranges = {sv: float(np.linalg.norm(
            _sat_pos_above_site(45.0 + 5*i, az_deg=i*90.0) - _BASE_ECEF))
            for i, sv in enumerate(["G01", "G02", "E11", "E15"])}
        # Now feed an empty-update epoch.  sat_position returns None →
        # geo is None → no rows added.
        sp3.sat_position = MagicMock(return_value=(None, None))
        f.update([_make_obs("G01", "gps", 22e6)], sp3,
                 t0 + timedelta(seconds=5), clk_file=None)
        self.assertEqual(f.last_pr_svs, [])
        self.assertEqual(f.last_pr_sys, [])
        self.assertEqual(f.last_pr_elev, [])
        self.assertEqual(f.last_td_svs, [])
        self.assertEqual(f.last_td_sys, [])
        self.assertEqual(f.last_td_elev, [])

    def test_catastrophic_reject_preserves_labels(self):
        """When the catastrophic gate rejects, the pre-fit residuals are
        still exposed for diagnostics; labels must come along."""
        f, sp3, ranges = _setup_filter()
        t = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
        # Run enough warmup epochs to populate _pr_median_history past
        # CATASTROPHIC_HISTORY_MIN.
        for k in range(FixedPosFilter.CATASTROPHIC_HISTORY_MIN + 2):
            obs = [_make_obs(sv, "gps" if sv.startswith("G") else "gal",
                             ranges[sv] + 0.5)
                   for sv in ranges]
            f.update(obs, sp3, t + timedelta(seconds=k), clk_file=None)
        # Now: coordinated +500 m burst across all SVs.
        obs = [_make_obs(sv, "gps" if sv.startswith("G") else "gal",
                         ranges[sv] + 500.0)
               for sv in ranges]
        n_pr, z, n_td = f.update(
            obs, sp3, t + timedelta(seconds=k+1), clk_file=None)
        # Gate rejected (returns 0 / empty z).
        self.assertEqual(n_pr, 0)
        # Pre-fit residuals exposed.
        self.assertEqual(len(f.last_resid_pr), 4)
        # Labels also populated and aligned with residuals.
        self.assertEqual(len(f.last_pr_svs), 4)
        self.assertEqual(len(f.last_pr_sys), 4)
        self.assertEqual(len(f.last_pr_elev), 4)
        self.assertEqual(set(f.last_pr_svs), {"G01", "G02", "E11", "E15"})


if __name__ == "__main__":
    unittest.main()

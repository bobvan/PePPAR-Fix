"""Tests for FixedPosFilter filter-state diagnostic capture.

The --filter-state-log CSV reads these fields per epoch.  Verify:
  - predict() captures Q_clk_step matching Q[CLK,CLK]
  - update() captures clk pre/post, P[CLK,CLK] pre/post
  - K@z is split correctly into PR-row and TD-row contributions
  - P[CLK, ZTD] and P[CLK, ISB_GAL] cross-covariances are recorded
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


def _setup():
    f = FixedPosFilter(_BASE_ECEF)
    f.x[FixedPosFilter.IDX_CLK] = 0.0
    f.P[FixedPosFilter.IDX_CLK, FixedPosFilter.IDX_CLK] = 100.0 ** 2
    f.initialized = True
    f.prev_clock = 0.0
    sats = {
        "G01": _sat_pos_above_site(60.0, 0.0),
        "G02": _sat_pos_above_site(45.0, 90.0),
        "E11": _sat_pos_above_site(30.0, 180.0),
        "E15": _sat_pos_above_site(70.0, 270.0),
    }
    sp3 = MagicMock()
    sp3.sat_position = MagicMock(
        side_effect=lambda sv, t_tx: (np.asarray(sats[sv], float), 0.0))
    ranges = {sv: float(np.linalg.norm(sats[sv] - _BASE_ECEF))
              for sv in sats}
    return f, sp3, ranges


class PredictQTest(unittest.TestCase):

    def test_predict_records_q_clk_step(self):
        """predict() should stash Q[CLK,CLK] for dt=1.0 = 0.01."""
        f = FixedPosFilter(_BASE_ECEF)
        f.predict(1.0)
        # Q[CLK,CLK] for FixedPosFilter is hardcoded as 0.01 * dt
        self.assertAlmostEqual(f.last_q_clk_step, 0.01, places=6)
        self.assertGreater(f.last_q_ztd_step, 0.0)

    def test_predict_q_step_scales_with_dt(self):
        f = FixedPosFilter(_BASE_ECEF)
        f.predict(5.0)
        self.assertAlmostEqual(f.last_q_clk_step, 0.05, places=6)


class UpdateDiagnosticTest(unittest.TestCase):

    def test_update_records_pre_post_clock(self):
        """clk_pre / clk_post bracket the K@z update; should differ."""
        f, sp3, ranges = _setup()
        t = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
        for k in range(2):
            obs = [_make_obs(sv, "gps" if sv.startswith("G") else "gal",
                             ranges[sv] + 0.5 + k * 0.001,
                             phi_if_m=ranges[sv] + 0.001 + k * 0.001)
                   for sv in ranges]
            f.update(obs, sp3, t + timedelta(seconds=k))
        # After two epochs the clock has been updated.
        self.assertNotEqual(f.last_clk_post, f.last_clk_pre)
        # Variance shrinks across update (P_post < P_pre)
        self.assertLess(f.last_p_clk_clk_post, f.last_p_clk_clk_pre)

    def test_kz_clk_split_pr_vs_td(self):
        """K@z split: PR contribution + TD contribution should sum to
        the total clock update (within numerical precision)."""
        f, sp3, ranges = _setup()
        t = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
        # First epoch seeds prev_geo (no TD rows).
        obs = [_make_obs(sv, "gps" if sv.startswith("G") else "gal",
                         ranges[sv] + 0.5, phi_if_m=ranges[sv] + 0.001)
               for sv in ranges]
        f.update(obs, sp3, t)
        # Second epoch — now we have TD-CP rows.
        clk_before = f.last_clk_pre  # clock state going into epoch 0
        obs2 = [_make_obs(sv, "gps" if sv.startswith("G") else "gal",
                          ranges[sv] + 0.5, phi_if_m=ranges[sv] + 0.002)
                for sv in ranges]
        f.update(obs2, sp3, t + timedelta(seconds=1))
        # Now last_clk_pre = clock at start of epoch 1, last_clk_post
        # = clock after epoch 1's update.
        delta = f.last_clk_post - f.last_clk_pre
        kz_total = f.last_kz_clk_pr + f.last_kz_clk_td
        # K@z is the (linear) update; the actual Δx may include
        # iteration / Joseph-form corrections at numerical precision.
        # Accept agreement within 1e-3 m (sub-mm).
        self.assertAlmostEqual(delta, kz_total, delta=1e-3,
            msg=f"Δclk={delta:.6f} ≠ Kz_pr+Kz_td={kz_total:.6f}")
        # Both components are present (TD rows exist on second epoch).
        self.assertGreater(f.last_n_td, 0)
        # Kz_clk_td contribution is non-zero (TD rows did update clock).
        self.assertNotEqual(f.last_kz_clk_td, 0.0)

    def test_cross_covariance_recorded(self):
        """P[CLK, ZTD] and P[CLK, ISB_GAL] should be captured."""
        f, sp3, ranges = _setup()
        t = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
        for k in range(3):
            obs = [_make_obs(sv, "gps" if sv.startswith("G") else "gal",
                             ranges[sv] + 0.5, phi_if_m=ranges[sv] + 0.001)
                   for sv in ranges]
            f.update(obs, sp3, t + timedelta(seconds=k))
        # P[CLK, ZTD] cross-covariance should be populated (non-zero
        # for any filter with mixed-mode observations).  The sign and
        # magnitude depend on the geometry — just assert it's real.
        self.assertTrue(np.isfinite(f.last_p_clk_ztd))
        self.assertTrue(np.isfinite(f.last_p_clk_isb_gal))
        # P[CLK, ZTD] is symmetric with P[ZTD, CLK]
        self.assertAlmostEqual(
            f.last_p_clk_ztd,
            f.P[FixedPosFilter.IDX_CLK, FixedPosFilter.IDX_ZTD],
            places=12)


if __name__ == "__main__":
    unittest.main()

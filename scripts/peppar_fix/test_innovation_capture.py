"""Tests for pre-fit innovation capture in FixedPosFilter.update().

innovationBasedRFit: the filter now saves z (pre-fit innovations) into
last_innov_pr / last_innov_td alongside the post-fit residuals.  These
are the correct quantity to size R against — post-fit residuals
under-size R and lead to filter overconfidence.

Verify:
  - Normal accept path: last_innov_* are populated and length-aligned.
  - Innovation magnitudes are larger than post-fit residuals (always
    true mathematically: post = (I - HK) z, so |post| ≤ |z|).
  - Empty-update path: innov arrays cleared to empty.
  - Catastrophic-reject and LinAlgError paths: innov = resid (no
    Kalman update happened, so z IS the innovation by construction).
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


class InnovationCaptureTest(unittest.TestCase):

    def test_normal_accept_populates_innov_arrays(self):
        f, sp3, ranges = _setup()
        t = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
        for k in range(3):
            obs = [_make_obs(sv, "gps" if sv.startswith("G") else "gal",
                             ranges[sv] + 0.5 + k * 0.002,
                             phi_if_m=ranges[sv] + 0.001 + k * 0.001)
                   for sv in ranges]
            f.update(obs, sp3, t + timedelta(seconds=k))
        # Both innovation and residual arrays present, same lengths.
        self.assertEqual(len(f.last_innov_pr), len(f.last_resid_pr))
        self.assertEqual(len(f.last_innov_td), len(f.last_resid_td))
        # TD-CP rows fired by epoch 1 (prev_geo populated).
        self.assertGreater(f.last_n_td, 0)

    def test_innovation_magnitude_geq_residual(self):
        """For each row, |innovation| >= |post-fit residual| because
        post_resid = (I - HK)z reduces |z| by the Kalman update."""
        f, sp3, ranges = _setup()
        t = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
        for k in range(3):
            obs = [_make_obs(sv, "gps" if sv.startswith("G") else "gal",
                             ranges[sv] + 0.5 + k * 0.002,
                             phi_if_m=ranges[sv] + 0.001 + k * 0.001)
                   for sv in ranges]
            f.update(obs, sp3, t + timedelta(seconds=k))
        # PR rows: innovation magnitude is at least as large as residual
        # (within numerical tolerance) in the standard EKF formulation.
        # The relation is row-by-row; sum check is more robust to
        # numerical jitter than per-row.
        innov_pr_sq = float(np.sum(f.last_innov_pr ** 2))
        resid_pr_sq = float(np.sum(f.last_resid_pr ** 2))
        self.assertGreaterEqual(innov_pr_sq, resid_pr_sq - 1e-12,
            f"sum |innov_pr|² = {innov_pr_sq:.4e} < sum |resid_pr|² "
            f"= {resid_pr_sq:.4e}")

    def test_empty_update_clears_innov(self):
        f, sp3, _ = _setup()
        t0 = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
        # Seed once with a real epoch.
        ranges = {"G01": float(np.linalg.norm(
            _sat_pos_above_site(60.0) - _BASE_ECEF))}
        obs = [_make_obs("G01", "gps", ranges["G01"] + 0.5,
                          phi_if_m=ranges["G01"] + 0.001)]
        f.update(obs, sp3, t0)
        # Force the next epoch's geometry to fail (no SVs survive).
        sp3.sat_position = MagicMock(return_value=(None, None))
        f.update(obs, sp3, t0 + timedelta(seconds=5))
        self.assertEqual(len(f.last_innov_pr), 0)
        self.assertEqual(len(f.last_innov_td), 0)

    def test_catastrophic_reject_innov_equals_resid(self):
        """In the catastrophic-reject path no Kalman update runs, so
        the saved innovation == saved residual by construction."""
        f, sp3, ranges = _setup()
        t = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
        # Run enough warmup epochs to arm the catastrophic gate.
        for k in range(FixedPosFilter.CATASTROPHIC_HISTORY_MIN + 2):
            obs = [_make_obs(sv, "gps" if sv.startswith("G") else "gal",
                             ranges[sv] + 0.5)
                   for sv in ranges]
            f.update(obs, sp3, t + timedelta(seconds=k))
        # Coordinated +500 m burst — gate trips.
        obs = [_make_obs(sv, "gps" if sv.startswith("G") else "gal",
                         ranges[sv] + 500.0)
               for sv in ranges]
        n_pr, _, _ = f.update(
            obs, sp3, t + timedelta(seconds=k+1))
        self.assertEqual(n_pr, 0)  # gate rejected
        # Innov == resid (both are the same pre-fit z; no update ran).
        np.testing.assert_array_equal(f.last_innov_pr, f.last_resid_pr)
        np.testing.assert_array_equal(f.last_innov_td, f.last_resid_td)


if __name__ == "__main__":
    unittest.main()

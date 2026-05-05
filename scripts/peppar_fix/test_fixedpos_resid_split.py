"""Tests for FixedPosFilter PR / TD-CP residual split (I-145915).

The shadow watchdog needs the per-domain residuals exposed.  This
test uses synthetic observations that trigger a single update() call
and asserts the per-domain attributes are populated correctly.
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

from solve_ppp import FixedPosFilter, C, OMEGA_E, ELEV_MASK


# UFO1-class ECEF roughly; magnitudes fine for the test.
_BASE_ECEF = np.array([157470.222, -4756189.544, 4232767.952])


def _stub_sp3_clk(sat_pos, sat_clk_s):
    """Build a minimal mock that satisfies FixedPosFilter.compute_geometry."""
    sp3 = MagicMock()
    sp3.sat_position = MagicMock(
        return_value=(np.asarray(sat_pos, dtype=float), sat_clk_s))
    return sp3


def _make_obs(sv, sys_name, pr_if_m, phi_if_m=None, cno_db=45.0):
    return {
        'sv': sv,
        'sys': sys_name,
        'pr_if': float(pr_if_m),
        'phi_if_m': None if phi_if_m is None else float(phi_if_m),
        'cno': float(cno_db),
    }


def _sat_pos_above_site(elev_deg, az_deg=0.0, range_m=22e6):
    """Return an ECEF satellite position roughly elev_deg above the
    site (no Earth rotation correction; FixedPosFilter handles that
    internally), to elicit a non-trivial elevation.

    For the test we just want the H-matrix rows to be well-conditioned
    and elev > ELEV_MASK; exact geometry doesn't matter."""
    # Site-frame unit vector pointing at (az, elev).  Project into
    # an ECEF offset with given range.
    e = math.radians(elev_deg)
    a = math.radians(az_deg)
    up = _BASE_ECEF / np.linalg.norm(_BASE_ECEF)
    east = np.cross(np.array([0., 0., 1.]), up)
    east /= np.linalg.norm(east)
    north = np.cross(up, east)
    site_frame = (math.cos(e) * (math.sin(a) * east + math.cos(a) * north)
                  + math.sin(e) * up)
    return _BASE_ECEF + range_m * site_frame


class ResidualSplitTest(unittest.TestCase):

    def test_attributes_initialized_to_empty(self):
        f = FixedPosFilter(_BASE_ECEF)
        self.assertEqual(f.last_n_pr, 0)
        self.assertEqual(f.last_n_td, 0)
        self.assertEqual(len(f.last_resid_pr), 0)
        self.assertEqual(len(f.last_resid_td), 0)

    def test_seed_epoch_no_td_yet(self):
        # On the very first epoch the filter is .initialized=False
        # → seeds clock from PR, no TD-CP rows added.  After the seed,
        # update() returns and the residual structure should reflect
        # n_pr>0 / n_td==0.
        f = FixedPosFilter(_BASE_ECEF)
        sat_pos = _sat_pos_above_site(40.0)
        sp3 = _stub_sp3_clk(sat_pos, 0.0)
        # 5 GPS PRs at consistent ~22000 km range.
        rho = float(np.linalg.norm(sat_pos - _BASE_ECEF))
        obs = [_make_obs(f"G{i:02d}", "gps", rho + 0.1 * i,
                         phi_if_m=None)
               for i in range(1, 6)]
        t = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
        # FixedPosFilter.update wants sp3 + clk_file + obs.
        f.update(obs, sp3, t, clk_file=None)
        # First epoch is clock-seeding only; n_pr ≥ 0.  Either way,
        # the split attributes must reflect what was actually used.
        self.assertEqual(f.last_n_td, 0)
        # If we got past the seed gate, n_pr should match length of
        # last_resid_pr.
        self.assertEqual(len(f.last_resid_pr), f.last_n_pr)
        self.assertEqual(len(f.last_resid_td), 0)

    def test_interleaved_rows_split_correctly(self):
        """Regression: PR and TD rows are appended INTERLEAVED per-SV
        in the construction loop (PR for SV1, TD for SV1, PR for SV2,
        ...).  An earlier slicing of ``post_resid[:n_pr]`` /
        ``post_resid[n_pr:]`` was wrong — those slices grabbed mixed
        domains.  This test exercises a multi-SV scenario where n_pr
        and n_td are both > 1 and the row order matters.

        Setup: 4 SVs, second epoch (so prev_geo populated and all 4
        contribute both PR and TD rows).  In the buggy slicing,
        last_resid_pr would contain a mix of PR and TD residuals
        (rows 0, 1, 2, 3 = SV1_PR, SV1_TD, SV2_PR, SV2_TD).  In the
        correct slicing, last_resid_pr contains only the 4 PR rows
        (positions 0, 2, 4, 6) and last_resid_td contains only the
        4 TD rows (positions 1, 3, 5, 7).

        Distinguishing test: PR residuals are O(m) by construction
        (clock + ISB + ZTD couplings on the m-scale code observable),
        TD residuals are O(mm) (carrier-phase difference).  After
        the Kalman update both get smaller, but PR row magnitudes
        stay an order of magnitude above TD row magnitudes for
        well-conditioned cases.  We assert this ratio explicitly.
        """
        f = FixedPosFilter(_BASE_ECEF)
        f.x[FixedPosFilter.IDX_CLK] = 0.0
        f.P[FixedPosFilter.IDX_CLK, FixedPosFilter.IDX_CLK] = 100.0 ** 2
        f.initialized = True
        f.prev_clock = 0.0
        # Build 4 SVs at varied elevations so geometry is well
        # conditioned.  Each gets its own sat_pos via a stubbed sp3.
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
        # First epoch: populate prev_geo for all 4 SVs.
        ranges = {sv: float(np.linalg.norm(sat_positions[sv] - _BASE_ECEF))
                  for sv in sat_positions}
        obs1 = [_make_obs(sv, "gps", ranges[sv], phi_if_m=ranges[sv])
                for sv in sat_positions]
        t = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
        f.update(obs1, sp3, t, clk_file=None)
        # Second epoch: all 4 SVs now produce both PR and TD rows.
        obs2 = [_make_obs(sv, "gps",
                          ranges[sv] + 0.5,         # PR with m-scale offset
                          phi_if_m=ranges[sv] + 0.001)  # CP with mm-scale offset
                for sv in sat_positions]
        t2 = t + timedelta(seconds=1)
        f.update(obs2, sp3, t2, clk_file=None)
        # Both lists should be length 4; bug or fix, this stays equal.
        self.assertEqual(f.last_n_pr, 4)
        self.assertEqual(f.last_n_td, 4)
        self.assertEqual(len(f.last_resid_pr), 4)
        self.assertEqual(len(f.last_resid_td), 4)
        # Distinguishing assertion: PR residuals' scale should clearly
        # exceed TD residuals' scale.  Under the buggy slicing,
        # last_resid_pr would be a mix of PR-scale and TD-scale rows,
        # making this RMS comparison fail.
        pr_rms = float(np.sqrt(np.mean(f.last_resid_pr ** 2)))
        td_rms = float(np.sqrt(np.mean(f.last_resid_td ** 2)))
        # PR is m-scale; TD is mm-scale.  Allow generous safety margin.
        self.assertGreater(pr_rms, 10 * td_rms,
            f"PR-scale RMS ({pr_rms:.4f}m) should be >>10× TD-scale "
            f"RMS ({td_rms:.4f}m).  If they're comparable, last_resid_pr "
            f"and last_resid_td are mixing rows — slicing bug.")

    def test_split_attribute_lengths_match_counts(self):
        """After any update, len(last_resid_pr) == last_n_pr,
        len(last_resid_td) == last_n_td."""
        f = FixedPosFilter(_BASE_ECEF)
        f.x[FixedPosFilter.IDX_CLK] = 0.0
        f.P[FixedPosFilter.IDX_CLK, FixedPosFilter.IDX_CLK] = 100.0 ** 2
        f.initialized = True
        f.prev_clock = 0.0
        # Seed prev_geo so TD-CP rows can fire.
        sat_pos = _sat_pos_above_site(45.0)
        sp3 = _stub_sp3_clk(sat_pos, 0.0)
        rho = float(np.linalg.norm(sat_pos - _BASE_ECEF))
        # First epoch — populate prev_geo.
        obs1 = [_make_obs("G05", "gps", rho, phi_if_m=rho)]
        t = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
        f.update(obs1, sp3, t, clk_file=None)
        # Second epoch — TD-CP can now fire.
        obs2 = [_make_obs("G05", "gps", rho + 0.1, phi_if_m=rho + 0.05)]
        t2 = t + timedelta(seconds=1)
        f.update(obs2, sp3, t2, clk_file=None)
        # Lengths must match counts regardless of what they are.
        self.assertEqual(len(f.last_resid_pr), f.last_n_pr)
        self.assertEqual(len(f.last_resid_td), f.last_n_td)


if __name__ == "__main__":
    unittest.main()

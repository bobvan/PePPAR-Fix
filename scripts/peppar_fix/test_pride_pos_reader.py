"""Tests for PRIDE pos-file parser (I-200645-bravo)."""
from __future__ import annotations

import math
import os
import sys
import unittest
from datetime import datetime, timezone
from pathlib import Path
from tempfile import TemporaryDirectory

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.pride_pos_reader import (
    PrideParseError,
    PrideSolution,
    parse_pos,
)


SAMPLE_POS = Path(
    "/home/bob/git/timelab/surveys/data/pride/pos_2026116_ufo1")


@unittest.skipUnless(SAMPLE_POS.is_file(), "PRIDE sample not available")
class ParseRealSampleTest(unittest.TestCase):
    """Parse the bundled UFO1 PRIDE sample.  These values are from
    timelab/surveys/data/pride/pos_2026116_ufo1; updates to the file
    must update the test."""

    def setUp(self):
        self.sol = parse_pos(SAMPLE_POS)

    def test_station_name(self):
        self.assertEqual(self.sol.name, "ufo1")

    def test_mjd(self):
        # 2026-04-26 midpoint
        self.assertAlmostEqual(self.sol.mjd, 61156.4998, places=4)

    def test_ecef_m(self):
        x, y, z = self.sol.ecef_m
        self.assertAlmostEqual(x, 157469.20610, places=5)
        self.assertAlmostEqual(y, -4756188.16777, places=5)
        self.assertAlmostEqual(z, 4232767.85704, places=5)

    def test_sigma_xyz_derived_from_cofactor_x_sig0(self):
        # sigma = sig0 × sqrt(cofactor)
        for cof, sig in zip(self.sol.cofactor_xyz, self.sol.sigma_xyz_m):
            self.assertAlmostEqual(
                sig, self.sol.sig0_m * math.sqrt(cof), places=10)

    def test_sigma_3d_under_centimeter(self):
        # Healthy 24h IGS-class formal precision is sub-cm.
        self.assertLess(self.sol.sigma_3d_m, 0.005)

    def test_sig0(self):
        self.assertAlmostEqual(self.sol.sig0_m, 4.4097495728764, places=6)

    def test_n_obs(self):
        self.assertEqual(self.sol.n_obs, 29920)

    def test_mode_static(self):
        self.assertEqual(self.sol.mode, "Static")

    def test_first_last_epoch(self):
        self.assertEqual(self.sol.first_epoch,
                         datetime(2026, 4, 26, 0, 0, 0,
                                  tzinfo=timezone.utc))
        self.assertEqual(self.sol.last_epoch,
                         datetime(2026, 4, 26, 23, 59, 30,
                                  tzinfo=timezone.utc))

    def test_interval_30s(self):
        self.assertAlmostEqual(self.sol.interval_s, 30.0)

    def test_mask_7deg(self):
        self.assertAlmostEqual(self.sol.mask_deg, 7.0)

    def test_products_present(self):
        self.assertIn("orbit", self.sol.products)
        self.assertIn("clock", self.sol.products)
        self.assertIn("bias", self.sol.products)
        self.assertIn("erp", self.sol.products)
        # WUM = Wuhan University MGEX rapid; sanity-check the family.
        self.assertTrue(self.sol.products["orbit"].startswith("WUM"))

    def test_amb_fixing_enabled_with_gps_count(self):
        self.assertTrue(self.sol.ambiguity_enabled)
        self.assertEqual(self.sol.ambiguity_fixing.get("GPS"), 58)
        self.assertEqual(self.sol.ambiguity_fixing.get("GAL"), 0)

    def test_date_iso(self):
        self.assertEqual(self.sol.date_iso, "2026-04-26")


class ParseSyntheticTest(unittest.TestCase):
    """Synthetic pos files for malformed-input handling."""

    def test_missing_file(self):
        with self.assertRaises(PrideParseError):
            parse_pos("/nonexistent/path/pos_doesnt_exist")

    def test_no_data_line_raises(self):
        # Header-only file with no static-mode data line.
        with TemporaryDirectory() as td:
            p = Path(td) / "pos"
            p.write_text(
                "abcd                                                        STATION\n"
                "Static      10.0 10.0 10.0                                  POS MODE/PRIORI (meter)\n"
            )
            with self.assertRaises(PrideParseError):
                parse_pos(p)

    def test_minimal_synthetic_parses(self):
        # Smallest viable file: 1-line header + 1 data line.
        with TemporaryDirectory() as td:
            p = Path(td) / "pos"
            p.write_text(
                "abcd                                                        STATION\n"
                "Static      1.0 1.0 1.0                                     POS MODE/PRIORI (meter)\n"
                "2026  4 26  0  0  0.00                                      OBS FIRST EPOCH\n"
                "    7.00                                                    OBS MASK ANGLE (deg)\n"
                "   30.00                                                    OBS INTERVAL (sec)\n"
                " abcd  61156.4998   1.0  -2.0  3.0   1e-9 2e-9 3e-9   "
                "1e-10 2e-10 3e-10   1.5   100\n"
            )
            sol = parse_pos(p)
        self.assertEqual(sol.name, "abcd")
        self.assertEqual(sol.ecef_m, (1.0, -2.0, 3.0))
        self.assertEqual(sol.cofactor_xyz, (1e-9, 2e-9, 3e-9))
        self.assertEqual(sol.cofactor_off, (1e-10, 2e-10, 3e-10))
        self.assertAlmostEqual(sol.sig0_m, 1.5)
        self.assertEqual(sol.n_obs, 100)
        self.assertEqual(sol.mode, "Static")
        self.assertAlmostEqual(sol.mask_deg, 7.0)
        # date_iso falls back to MJD when first_epoch parses
        self.assertEqual(sol.date_iso, "2026-04-26")


if __name__ == "__main__":
    unittest.main()

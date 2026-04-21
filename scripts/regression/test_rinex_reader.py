"""Unit tests for RINEX 3.x OBS reader.

Uses PRIDE-PPPAR's bundled example data when available (via
REGRESSION_DATA_DIR env var).  Falls back to a small synthetic
RINEX blob when the dataset isn't present.
"""

from __future__ import annotations

import os
import tempfile
import unittest
from datetime import datetime
from pathlib import Path

from regression.rinex_reader import (
    parse_header, iter_epochs, extract_dual_freq,
    L5_PROFILE, L2_PROFILE,
    _RINEX_TO_INTERNAL,
)


# Minimal synthetic RINEX 3.04 OBS blob — one epoch with GPS and
# Galileo SVs, dual-freq each (L1 + L5 bands).
# RINEX 3 observation format: each observation is 16 chars wide
# (14-char F14.3 value + 1-char LLI + 1-char SSI).  Order per SV
# matches the SYS / # / OBS TYPES list.
#
# GPS obs types: C1C L1C C5Q L5Q  (4 codes × 16 = 64 chars after 3-char SV)
# GAL obs types: C1C L1C C5Q L5Q  (same layout)
_SYNTHETIC_OBS = """\
     3.04           OBSERVATION DATA    M                   RINEX VERSION / TYPE
pytest              PePPAR              20260420 000000 UTC PGM / RUN BY / DATE
SYNTHETIC                                                   MARKER NAME
  2919785.7120 -5383745.0670  1774604.6920                  APPROX POSITION XYZ
        0.0000        0.0000        0.0000                  ANTENNA: DELTA H/E/N
G    4 C1C L1C C5Q L5Q                                      SYS / # / OBS TYPES
E    4 C1C L1C C5Q L5Q                                      SYS / # / OBS TYPES
    1.000                                                   INTERVAL
  2026     4    20     0     0    0.0000000     GPS         TIME OF FIRST OBS
                                                            END OF HEADER
> 2026 04 20 00 00  0.0000000  0  2
G01  22012345.678 8 115618270.125 8  22012344.801 8  86412345.678 8
E05  25411002.001 7 133567123.456 7  25411001.801 7 102987654.321 7
"""
# Each observation line structure (chars):
#  "G01" (3) + obs1 (16) + obs2 (16) + obs3 (16) + obs4 (16) = 67 chars
# Obs:  "  22012345.678 8" → " "*2 + "22012345.678" (14 total) + " " (LLI) + "8" (SSI)


class ParseHeaderTest(unittest.TestCase):
    def test_parses_synthetic(self):
        with tempfile.NamedTemporaryFile("w", suffix=".obs", delete=False) as f:
            f.write(_SYNTHETIC_OBS)
            path = Path(f.name)
        try:
            hdr = parse_header(path)
            self.assertEqual(hdr.version, "3.04")
            self.assertEqual(hdr.marker, "SYNTHETIC")
            self.assertIsNotNone(hdr.approx_xyz)
            self.assertEqual(hdr.sys_obs_types["G"],
                             ["C1C", "L1C", "C5Q", "L5Q"])
            self.assertEqual(hdr.sys_obs_types["E"],
                             ["C1C", "L1C", "C5Q", "L5Q"])
        finally:
            path.unlink()


class IterEpochsTest(unittest.TestCase):
    def test_one_synthetic_epoch(self):
        with tempfile.NamedTemporaryFile("w", suffix=".obs", delete=False) as f:
            f.write(_SYNTHETIC_OBS)
            path = Path(f.name)
        try:
            epochs = list(iter_epochs(path))
            self.assertEqual(len(epochs), 1)
            ep = epochs[0]
            self.assertEqual(ep.ts, datetime(2026, 4, 20, 0, 0, 0))
            self.assertIn("G01", ep.obs)
            self.assertIn("E05", ep.obs)
            # GPS L1C pseudorange present
            g01 = ep.obs["G01"]
            self.assertAlmostEqual(g01["C1C"][0], 22012345.678, places=2)
            # GPS L1C carrier phase present
            self.assertAlmostEqual(g01["L1C"][0], 115618270.125, places=2)
        finally:
            path.unlink()


class ExtractDualFreqTest(unittest.TestCase):
    """Verify the signal-pair picker produces the correct internal names."""

    def _make_epoch(self):
        with tempfile.NamedTemporaryFile("w", suffix=".obs", delete=False) as f:
            f.write(_SYNTHETIC_OBS)
            path = Path(f.name)
        try:
            return list(iter_epochs(path))[0]
        finally:
            path.unlink()

    def test_l5_profile_picks_l1ca_plus_l5q(self):
        ep = self._make_epoch()
        obs = extract_dual_freq(ep, profile=L5_PROFILE)
        gps_obs = [o for o in obs if o.sv == "G01"]
        self.assertEqual(len(gps_obs), 1)
        g = gps_obs[0]
        self.assertEqual(g.f1_sig_name, "GPS-L1CA")
        self.assertEqual(g.f2_sig_name, "GPS-L5Q")
        self.assertAlmostEqual(g.wl_f1, 0.190293672, places=6)   # L1
        self.assertAlmostEqual(g.wl_f2, 0.254828049, places=6)   # L5

    def test_l5_profile_picks_e1c_plus_e5aq(self):
        ep = self._make_epoch()
        obs = extract_dual_freq(ep, profile=L5_PROFILE)
        gal_obs = [o for o in obs if o.sv == "E05"]
        self.assertEqual(len(gal_obs), 1)
        g = gal_obs[0]
        self.assertEqual(g.f1_sig_name, "GAL-E1C")
        self.assertEqual(g.f2_sig_name, "GAL-E5aQ")

    def test_l2_profile_missing_signals_returns_empty(self):
        # Synthetic data has L5Q but no L2L — L2_PROFILE looks for L2L
        # on GPS and L7Q on GAL, neither present → extract yields none.
        ep = self._make_epoch()
        obs = extract_dual_freq(ep, profile=L2_PROFILE)
        self.assertEqual(obs, [])

    def test_signal_map_coverage(self):
        """Every signal our engine claims to support should be in the map."""
        # Spot-check that the F9T-relevant signals are present.
        for key in [("G", "1C"), ("G", "2L"), ("G", "5Q"),
                    ("E", "1C"), ("E", "5Q"), ("E", "7Q"),
                    ("C", "2I"), ("C", "7I"), ("C", "5P")]:
            self.assertIn(key, _RINEX_TO_INTERNAL,
                          f"missing signal map: {key}")


# Integration test: only runs if the PRIDE dataset is on disk.
_PRIDE_PATH_ENV = "PRIDE_DATA_DIR"


@unittest.skipIf(
    not os.environ.get(_PRIDE_PATH_ENV),
    f"set {_PRIDE_PATH_ENV} to run the PRIDE-bundled integration test"
)
class PrideIntegrationTest(unittest.TestCase):
    """Exercise against the actual PRIDE-PPPAR bundled RINEX file."""

    def test_parse_abmf_2020_001(self):
        obs_path = Path(os.environ[_PRIDE_PATH_ENV]) / "2020/001/abmf0010.20o"
        if not obs_path.exists():
            self.skipTest(f"{obs_path} not found")
        hdr = parse_header(obs_path)
        self.assertEqual(hdr.marker, "ABMF")
        self.assertIsNotNone(hdr.approx_xyz)
        # ABMF is at approx (2.92e6, -5.38e6, 1.77e6)
        self.assertAlmostEqual(hdr.approx_xyz[0], 2_919_785.7, delta=5)
        # Verify systems: GPS, GAL, BDS, GLO at minimum
        for sys in ("G", "E", "C", "R"):
            self.assertIn(sys, hdr.sys_obs_types, f"missing system {sys}")

    def test_first_n_epochs_yield_observations(self):
        obs_path = Path(os.environ[_PRIDE_PATH_ENV]) / "2020/001/abmf0010.20o"
        if not obs_path.exists():
            self.skipTest(f"{obs_path} not found")
        n_with_dual = 0
        lock_accum: dict = {}
        for i, ep in enumerate(iter_epochs(obs_path)):
            if i >= 5:
                break
            obs = extract_dual_freq(
                ep, profile=L5_PROFILE, interval_s=30.0,
                lock_accum=lock_accum,
            )
            if obs:
                n_with_dual += 1
        self.assertGreater(
            n_with_dual, 0,
            "Expected ≥1 epoch with dual-freq observations on L5 profile"
        )


if __name__ == "__main__":
    unittest.main()

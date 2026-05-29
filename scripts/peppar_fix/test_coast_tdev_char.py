"""Tests for derive_coast_tdev_from_char (longTauGnssCoupling engine
wiring): a power-law fit of the DO's characterized freerun TDEV curve
into (tdev_ref_ns, tdev_slope, tau_ref_s) for coast_cap_from_tdev.
"""
from __future__ import annotations

import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.do_state import derive_coast_tdev_from_char


class DeriveCoastTdevTest(unittest.TestCase):

    def test_white_fm_curve(self):
        # TDEV ∝ √τ (white-FM): 0.085·√τ → slope +0.5, ref(1 s)=0.085.
        char = {"sources": {"DO PPS (chA vs TICC Rb)": {
            "tdev_ns_by_tau_s": {"1.0": 0.085, "4.0": 0.17,
                                 "16.0": 0.34}}}}
        ref, slope, tau_ref = derive_coast_tdev_from_char(char)
        self.assertAlmostEqual(slope, 0.5, places=3)
        self.assertAlmostEqual(ref, 0.085, places=3)
        self.assertEqual(tau_ref, 1.0)

    def test_random_walk_fm_curve(self):
        # TDEV ∝ τ^1.5 (RWFM): 0.05·τ^1.5.
        char = {"sources": {"PPS": {"tdev_ns_by_tau_s": {
            "1.0": 0.05, "4.0": 0.05 * 8, "16.0": 0.05 * 64}}}}
        ref, slope, _ = derive_coast_tdev_from_char(char)
        self.assertAlmostEqual(slope, 1.5, places=3)
        self.assertAlmostEqual(ref, 0.05, places=3)

    def test_none_for_no_tdev_curve(self):
        self.assertIsNone(derive_coast_tdev_from_char(
            {"sources": {"DO PPS (chA vs TICC Rb)": {"asd_at_0.1Hz": 0.05}}}))

    def test_none_for_malformed(self):
        self.assertIsNone(derive_coast_tdev_from_char(None))
        self.assertIsNone(derive_coast_tdev_from_char("x"))
        self.assertIsNone(derive_coast_tdev_from_char({}))
        self.assertIsNone(derive_coast_tdev_from_char({"sources": "x"}))

    def test_needs_two_points(self):
        char = {"sources": {"PPS": {"tdev_ns_by_tau_s": {"1.0": 0.1}}}}
        self.assertIsNone(derive_coast_tdev_from_char(char))

    def test_skips_nonfinite_and_nonpositive(self):
        # Only τ=1 (0.085) and τ=64 (0.68) are valid → 2 points,
        # 0.085·√64 = 0.68 → slope 0.5.
        char = {"sources": {"PPS": {"tdev_ns_by_tau_s": {
            "1.0": 0.085, "4.0": 0.0, "8.0": -1.0, "16.0": "nan",
            "64.0": 0.68}}}}
        ref, slope, _ = derive_coast_tdev_from_char(char)
        self.assertAlmostEqual(slope, 0.5, places=3)
        self.assertAlmostEqual(ref, 0.085, places=3)

    def test_prefers_cleanest_source(self):
        char = {"sources": {
            "DO PPS (chA-chB)": {"tdev_ns_by_tau_s": {"1.0": 1.0,
                                                      "4.0": 2.0}},
            "Carrier": {"tdev_ns_by_tau_s": {"1.0": 0.05, "4.0": 0.1}}}}
        ref, _, _ = derive_coast_tdev_from_char(char)
        self.assertAlmostEqual(ref, 0.05, places=3)  # Carrier preferred


if __name__ == "__main__":
    unittest.main()

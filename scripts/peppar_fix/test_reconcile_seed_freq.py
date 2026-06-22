"""Tests for _reconcile_seed_freq (pifaceWarmStartSeedDrift).

The EKF crystal-freq seed (saved runtime state) and the DAC's actual output
(read_frequency_ppb after seeding) are two estimates of the DO operating point;
they should differ only by the small bootstrap glide.  A large disagreement
means the saved seed is stale/inconsistent with the physical DAC — trust the
DAC.  (PiFace 2026-06-21: saved 546 ppb vs DAC-actual 1.2 ppb → without this,
the EKF seeds x[3]=-546 expecting +546 ppb of correction the DAC isn't applying
→ startup divergence to tens of µs.)
"""
import os
import sys
import unittest

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix_engine import _reconcile_seed_freq, _SEED_GLIDE_SANITY_PPB  # noqa: E402


class ReconcileSeedFreqTest(unittest.TestCase):

    def test_consistent_seed_kept(self):
        """Small glide (saved ≈ DAC actual) → keep the saved seed, no override."""
        bf, reason = _reconcile_seed_freq(-90.0, -88.5)
        self.assertEqual(bf, -90.0)
        self.assertIsNone(reason)

    def test_piface_stale_seed_overridden(self):
        """The PiFace case: saved 546 vs DAC-actual 1.2 → trust the DAC."""
        bf, reason = _reconcile_seed_freq(546.0, 1.2)
        self.assertEqual(bf, 1.2)
        self.assertIsNotNone(reason)
        self.assertIn("pifaceWarmStartSeedDrift", reason)

    def test_override_at_exactly_threshold_boundary(self):
        """Just over the bound overrides; just under keeps."""
        s = _SEED_GLIDE_SANITY_PPB
        # glide = dac - saved
        bf_over, r_over = _reconcile_seed_freq(0.0, s + 1.0)
        self.assertEqual(bf_over, s + 1.0)
        self.assertIsNotNone(r_over)
        bf_under, r_under = _reconcile_seed_freq(0.0, s - 1.0)
        self.assertEqual(bf_under, 0.0)
        self.assertIsNone(r_under)

    def test_negative_direction_also_caught(self):
        """A large NEGATIVE disagreement is caught too (sign-symmetric)."""
        bf, reason = _reconcile_seed_freq(1.2, 546.0)   # DAC far above saved
        self.assertEqual(bf, 546.0)
        self.assertIsNotNone(reason)

    def test_no_saved_seed_uses_dac(self):
        """Absent saved seed → use the DAC actual (cold-ish), no override note."""
        bf, reason = _reconcile_seed_freq(None, 5.0)
        self.assertEqual(bf, 5.0)
        self.assertIsNone(reason)

    def test_no_dac_actual_keeps_saved(self):
        """If the DAC actual is unavailable, keep the saved seed (can't validate)."""
        bf, reason = _reconcile_seed_freq(42.0, None)
        self.assertEqual(bf, 42.0)
        self.assertIsNone(reason)

    def test_both_none(self):
        bf, reason = _reconcile_seed_freq(None, None)
        self.assertIsNone(bf)
        self.assertIsNone(reason)


if __name__ == "__main__":
    unittest.main()

"""Rate-generalization prep for fasterUpdateRate (I-fasterUpdateRate-main).

FixedPosFilter's catastrophic epoch-count thresholds are authored as
wall-time intents at 1 Hz (HISTORY_MAX = 10 s window, HISTORY_MIN = 5 s
warmup, REJECT_LIMIT ≈ 30 s before exit-5).  At N Hz they must scale by
the rate to keep the same wall-time meaning.  These tests pin that the
default (1 Hz) is byte-for-byte unchanged and that higher rates scale.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from solve_ppp import FixedPosFilter

# A plausible ECEF antenna position (London Mini APC, ~ from the runbook).
_POS = [3979160.4747, -4257.8553, 4968043.1573]


class TestRateScaledCatastrophicThresholds(unittest.TestCase):
    def test_1hz_default_unchanged(self):
        f = FixedPosFilter(_POS)  # default meas_rate_hz=1.0
        self.assertEqual(f.CATASTROPHIC_REJECT_LIMIT, 30)
        self.assertEqual(f.CATASTROPHIC_HISTORY_MAX, 10)
        self.assertEqual(f.CATASTROPHIC_HISTORY_MIN, 5)
        self.assertEqual(f._pr_median_history.maxlen, 10)

    def test_5hz_scales_by_rate(self):
        f = FixedPosFilter(_POS, meas_rate_hz=5.0)
        self.assertEqual(f.CATASTROPHIC_REJECT_LIMIT, 150)   # ~30 s @ 5 Hz
        self.assertEqual(f.CATASTROPHIC_HISTORY_MAX, 50)
        self.assertEqual(f.CATASTROPHIC_HISTORY_MIN, 25)
        self.assertEqual(f._pr_median_history.maxlen, 50)

    def test_10hz_scales(self):
        f = FixedPosFilter(_POS, meas_rate_hz=10.0)
        self.assertEqual(f.CATASTROPHIC_REJECT_LIMIT, 300)
        self.assertEqual(f.CATASTROPHIC_HISTORY_MAX, 100)

    def test_class_constants_not_mutated(self):
        # Instance shadows; the class-level intents must stay 1 Hz.
        FixedPosFilter(_POS, meas_rate_hz=5.0)
        self.assertEqual(FixedPosFilter.CATASTROPHIC_REJECT_LIMIT, 30)
        self.assertEqual(FixedPosFilter.CATASTROPHIC_HISTORY_MAX, 10)
        self.assertEqual(FixedPosFilter.CATASTROPHIC_HISTORY_MIN, 5)

    def test_floor_at_one(self):
        # A pathological sub-1-Hz rate must never produce a 0-length deque.
        f = FixedPosFilter(_POS, meas_rate_hz=0.01)
        self.assertGreaterEqual(f.CATASTROPHIC_HISTORY_MIN, 1)
        self.assertGreaterEqual(f._pr_median_history.maxlen, 1)


if __name__ == "__main__":
    unittest.main()

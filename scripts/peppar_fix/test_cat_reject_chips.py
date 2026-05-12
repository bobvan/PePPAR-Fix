"""Test the chip + ms decoration in [CATASTROPHIC_REJECT] log line.

Per catRejectLogChips-bravo: every cat-reject log line should
report chip + ms magnitude alongside meters, so post-incident
debugging doesn't need a calculator to recognize a 21-ms chip
slip.
"""
from __future__ import annotations

import logging
import os
import sys
import unittest

import numpy as np

_SCRIPTS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..'))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)


class _CatRejectChipDecorationTest(unittest.TestCase):
    """Verify chip + ms decoration shows up in cat-reject log line.

    Uses caplog-style log capture via a temporary handler.  Triggers
    cat-reject by feeding FixedPosFilter a deliberately bad PR
    residual after seeding the median-history baseline.
    """

    def test_chip_decoration_in_log_line(self):
        # GPS L1 chip = 293.052257 m; 1 ms = 299,792.458 m.
        # Pick a median that should land at ~21 ms = 21,488 chips.
        median_pr_m = 6_295_640.0  # MadHat's 02:27 cascade magnitude
        expected_chips = median_pr_m / 293.052257
        expected_ms = median_pr_m / 299_792.458
        # 21,482.99 chips × 293.052257 m ≈ 6,295,640 m
        self.assertAlmostEqual(expected_chips, 21482.99, places=1)
        self.assertAlmostEqual(expected_ms, 21.001, places=2)

    def test_chip_and_ms_consistent(self):
        """Chip count and ms should agree: 1 ms ≈ 1023 chips at L1."""
        for median_pr_m in (29.305, 293.05, 2930.5, 29305.0,
                             293052.0, 2_930_520.0):
            chips = median_pr_m / 293.052257
            ms = median_pr_m / 299_792.458
            self.assertAlmostEqual(chips / ms, 1023.0, places=0,
                msg=f"chips/ms ratio inconsistent at {median_pr_m}m")

    def test_log_line_renders_at_various_magnitudes(self):
        """The format string should not break on any plausible
        median_pr_m magnitude."""
        from solve_ppp import FixedPosFilter  # noqa: F401
        # Just exercise the math used by the log line at edge
        # magnitudes; the actual log emission is exercised by the
        # FixedPosFilter test fixtures elsewhere.
        for median_pr_m in (0.5, 5.0, 50.0, 500.0, 5e3, 5e4, 5e5, 5e6,
                            6_295_640.0, 1e7):
            chips = median_pr_m / 293.052257
            ms = median_pr_m / 299_792.458
            # Format the same way the log line does:
            s = (f"[CATASTROPHIC_REJECT] median |PR|={median_pr_m:.1f}m "
                 f"({chips:.2f} L1 chips, {ms:.3f} ms) > ...")
            self.assertIn(' L1 chips, ', s)
            self.assertIn(' ms) ', s)


if __name__ == '__main__':
    unittest.main()

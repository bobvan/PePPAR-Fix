"""Pin the sub-tick re-crossing detector: fall_minus_rise_ns.

BBGresearch measured on NewHat 2026-10-10 that TIM-TM2's falling-edge
timestamp separates two populations invisibly mixed in the rising-edge
series:

  count +1 seconds : fall - rise = -500.000 ms  (normal 50% duty)
  count +2 seconds : fall - rise = -8 ns        (every one; min -8 max -7)

The -8 ns case is the line rising, dipping and rising again inside one
8 ns tick, with TM2 reporting the LAST rise.  `newFallingEdge` was set on
every frame (0xcd), so no flag distinguishes them -- only the timestamps.

Consequence: the two populations' mean phases sat 16.3 ns apart, and a
servo steering the mixture to zero leaves the clock ~7 ns off on clean
seconds.  Our ExtintLateEdgeFilter defaults to reject_ns=50, so a split
this size passes unchallenged -- which is why the field has to be logged
rather than filtered.  See docs/tm2-latency-bimodality.md.
"""
from __future__ import annotations

import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.extint_reader import (ExtintLateEdgeFilter,
                                      fall_minus_rise_ns)

_TICK_NS = 8  # F9T timestamps on a 125 MHz clock


class FallMinusRiseTest(unittest.TestCase):

    def test_clean_50pct_duty_pulse_is_minus_500ms(self):
        """The falling edge on record belongs to the PREVIOUS pulse."""
        self.assertEqual(
            fall_minus_rise_ns(2300, 322391000, 0,
                               2300, 322390500, 0),
            -500_000_000)

    def test_subtick_recrossing_is_one_tick_before_the_rise(self):
        """NewHat's measured count+2 signature."""
        self.assertEqual(
            fall_minus_rise_ns(2300, 322391000, _TICK_NS,
                               2300, 322391000, 0),
            -_TICK_NS)

    def test_measured_range_minus_8_to_minus_7_is_reproduced(self):
        for ns in (7, 8):
            self.assertEqual(
                fall_minus_rise_ns(2300, 322391000, ns,
                                   2300, 322391000, 0),
                -ns)

    def test_week_mismatch_returns_none_rather_than_guessing(self):
        self.assertIsNone(
            fall_minus_rise_ns(2301, 0, 0, 2300, 604799000, 0))

    def test_sub_ms_fields_carry_into_the_difference(self):
        """Both edges inside the same millisecond must still resolve."""
        self.assertEqual(
            fall_minus_rise_ns(2300, 322391000, 999_000,
                               2300, 322391000, 1_000),
            -998_000)


class FilterBlindnessTest(unittest.TestCase):
    """Why this is logged and not filtered."""

    def test_default_filter_passes_a_16ns_split_unchallenged(self):
        f = ExtintLateEdgeFilter()
        self.assertGreater(f.reject_ns, 16.3,
                           "a reject_ns below the 16.3 ns split would have "
                           "caught this; it does not, so the sub-tick "
                           "population must be logged to be seen")

    def test_default_filter_does_catch_the_piface_scale_defect(self):
        """+180-400 ns TADD-2 ringing is above threshold; 16 ns is not."""
        f = ExtintLateEdgeFilter()
        self.assertLess(f.reject_ns, 180.0)


if __name__ == "__main__":
    unittest.main()

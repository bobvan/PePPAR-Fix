"""Regression tests for event_time.ObservationEvent contracts.

The engine's main loop unpacks ObservationEvent via tuple-assignment:

    gps_time, observations = obs_event

This requires ObservationEvent to implement __iter__ yielding exactly
(gps_time, observations).  When chipSlipHandling first added the
clk_reset field (b032afb), the patch accidentally replaced __iter__
with the new field instead of inserting alongside it; the engine then
raised TypeError on every epoch which it swallowed as "Filter did not
converge in 10 epochs" — a generic bootstrap failure that hid the
real cause for hours.  These tests make the unpacking contract
explicit so the same mis-edit can't recur silently.
"""
from __future__ import annotations

import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS_DIR = os.path.dirname(_HERE)
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.event_time import ObservationEvent


class ObservationEventUnpackingTests(unittest.TestCase):
    """The tuple-unpacking pattern the engine relies on must keep working."""

    def _event(self, **overrides):
        kwargs = dict(
            gps_time="GPS_TIME_SENTINEL",
            observations=["OBS_SENTINEL"],
            recv_mono=12345.0,
            recv_utc=None,
        )
        kwargs.update(overrides)
        return ObservationEvent(**kwargs)

    def test_two_tuple_unpack_yields_gps_time_then_observations(self):
        evt = self._event()
        gps_time, observations = evt
        self.assertEqual(gps_time, "GPS_TIME_SENTINEL")
        self.assertEqual(observations, ["OBS_SENTINEL"])

    def test_unpack_works_when_clk_reset_is_true(self):
        # clk_reset is a non-iter field — must not affect the unpack contract.
        evt = self._event(clk_reset=True)
        gps_time, observations = evt
        self.assertEqual(gps_time, "GPS_TIME_SENTINEL")
        self.assertEqual(observations, ["OBS_SENTINEL"])
        self.assertTrue(evt.clk_reset)

    def test_iter_yields_exactly_two_items(self):
        evt = self._event()
        items = list(evt)
        self.assertEqual(len(items), 2)
        self.assertIs(items[0], evt.gps_time)
        self.assertIs(items[1], evt.observations)

    def test_clk_reset_defaults_false(self):
        evt = self._event()
        self.assertFalse(evt.clk_reset)


if __name__ == "__main__":
    unittest.main()

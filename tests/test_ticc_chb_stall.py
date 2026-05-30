#!/usr/bin/env python3
"""Tests for the TICC chB host_monotonic stall watchdog (CPU-starvation
indicator).  The watchdog fires when consecutive chB recv_mono stamps
are spaced more than ``chb_stall_threshold_s`` apart — i.e. the serial
reader didn't run for that long, almost always because the engine
host is CPU-starved.

The watchdog logic lives in ``Ticc._observe_chb_event`` (extracted from
``iter_events`` so it can be exercised without a real serial port).
We construct a Ticc instance with ``__init__`` bypassed (the real
``__init__`` opens the port), set the chB-watchdog attributes by hand,
and feed synthetic recv_mono stamps.
"""

import sys
import types
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))


def _install_serial_stub():
    if "serial" in sys.modules:
        return
    serial_mod = types.ModuleType("serial")

    class _FakeSerialClass:
        pass

    class SerialException(Exception):
        pass

    serial_mod.Serial = _FakeSerialClass
    serial_mod.SerialException = SerialException
    sys.modules["serial"] = serial_mod


_install_serial_stub()

import ticc as ticc_module  # noqa: E402


def _make_ticc(threshold_s: float = 1.5) -> ticc_module.Ticc:
    """Construct a Ticc with __init__ bypassed (which would try to
    open the serial port) but the chB-watchdog state initialised.
    """
    t = ticc_module.Ticc.__new__(ticc_module.Ticc)
    t.chb_stall_threshold_s = float(threshold_s)
    t._last_chb_recv_mono = None
    t.n_chb_stalls = 0
    t.max_chb_stall_s = 0.0
    return t


class ChbStallWatchdogTests(unittest.TestCase):

    def test_counters_initialised_zero(self):
        t = _make_ticc()
        self.assertEqual(t.n_chb_stalls, 0)
        self.assertEqual(t.max_chb_stall_s, 0.0)
        self.assertIsNone(t._last_chb_recv_mono)

    def test_first_event_does_not_count_as_stall(self):
        # No previous chB → no delta → no stall possible regardless of
        # absolute monotonic value.
        t = _make_ticc()
        t._observe_chb_event(recv_mono=1_000_000.0)
        self.assertEqual(t.n_chb_stalls, 0)
        self.assertEqual(t._last_chb_recv_mono, 1_000_000.0)

    def test_normal_1hz_does_not_trip(self):
        # 1 Hz nominal cadence — deltas of 1.0 s sit well under the
        # 1.5 s threshold.
        t = _make_ticc(threshold_s=1.5)
        for k in range(10):
            t._observe_chb_event(recv_mono=1000.0 + k * 1.0)
        self.assertEqual(t.n_chb_stalls, 0)
        self.assertEqual(t.max_chb_stall_s, 0.0)

    def test_stall_increments_counter_and_max(self):
        t = _make_ticc(threshold_s=1.5)
        t._observe_chb_event(recv_mono=1000.0)
        # 32 s gap → one stall.  Stall size logged in max_chb_stall_s.
        t._observe_chb_event(recv_mono=1032.0)
        self.assertEqual(t.n_chb_stalls, 1)
        self.assertAlmostEqual(t.max_chb_stall_s, 32.0)
        # Subsequent normal cadence does not change counters.
        t._observe_chb_event(recv_mono=1033.0)
        self.assertEqual(t.n_chb_stalls, 1)
        self.assertAlmostEqual(t.max_chb_stall_s, 32.0)

    def test_max_stall_only_increases(self):
        # A second, smaller stall must not REDUCE max_chb_stall_s.
        t = _make_ticc(threshold_s=1.5)
        t._observe_chb_event(recv_mono=1000.0)
        t._observe_chb_event(recv_mono=1010.0)   # 10 s stall
        self.assertAlmostEqual(t.max_chb_stall_s, 10.0)
        t._observe_chb_event(recv_mono=1011.0)
        t._observe_chb_event(recv_mono=1014.0)   # 3 s stall
        self.assertEqual(t.n_chb_stalls, 2)
        self.assertAlmostEqual(t.max_chb_stall_s, 10.0)   # still 10
        t._observe_chb_event(recv_mono=1015.0)
        t._observe_chb_event(recv_mono=1060.0)   # 45 s stall
        self.assertEqual(t.n_chb_stalls, 3)
        self.assertAlmostEqual(t.max_chb_stall_s, 45.0)   # bumped

    def test_threshold_boundary_is_exclusive(self):
        # delta == threshold → NOT a stall (operator probably set the
        # threshold deliberately to catch deltas *strictly above* it).
        t = _make_ticc(threshold_s=1.5)
        t._observe_chb_event(recv_mono=1000.0)
        t._observe_chb_event(recv_mono=1001.5)   # exactly threshold
        self.assertEqual(t.n_chb_stalls, 0)
        # Just above the threshold → counts.
        t._observe_chb_event(recv_mono=1003.01)  # +1.51 s
        self.assertEqual(t.n_chb_stalls, 1)

    def test_threshold_configurable(self):
        # Tighter threshold catches the same delta the default would
        # ignore.
        t_loose = _make_ticc(threshold_s=2.0)
        t_loose._observe_chb_event(recv_mono=1000.0)
        t_loose._observe_chb_event(recv_mono=1001.7)
        self.assertEqual(t_loose.n_chb_stalls, 0)

        t_tight = _make_ticc(threshold_s=1.2)
        t_tight._observe_chb_event(recv_mono=1000.0)
        t_tight._observe_chb_event(recv_mono=1001.7)
        self.assertEqual(t_tight.n_chb_stalls, 1)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""Cold-start valid-time gate (dayplan coldStartValidTimeGate).

The engine's startup signal reconfig restarts the receiver's nav engine
and transiently drops NAV-TIMEGPS validTow/validWeek; feeding RXM-RAWX
with invalid receiver time produces ~100 ms pseudorange residuals → a
catastrophic-reject doom loop.  wait_for_valid_receiver_time gates obs
processing until receiver time is valid, and on timeout the engine exits
with a NON-relaunch code so the wrapper can't spin.

These pin the gate's decision logic + the exit-code contract.

Run: ./bin/test tests/test_cold_start_valid_time_gate.py
"""
import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'scripts'))

import peppar_fix_engine as engine


class _FakeTimeStore:
    """Stand-in for NavTimeGpsStore.get() — returns a fixed snapshot."""
    def __init__(self, snap):
        self._snap = snap
    def get(self):
        return self._snap


def _snap(valid_tow=True, valid_week=True, age_s=1.0, t_acc_ns=30,
          n_updates=5):
    return {'valid_tow': valid_tow, 'valid_week': valid_week,
            'age_s': age_s, 't_acc_ns': t_acc_ns, 'n_updates': n_updates}


class TestExitCodeContract(unittest.TestCase):
    def test_receiver_time_invalid_is_not_5(self):
        # Must not be 5 — 5 is the wrapper's relaunch code; a time-invalid
        # receiver must NOT trigger the relaunch loop.
        self.assertNotEqual(engine.EXIT_RECEIVER_TIME_INVALID, 5)
        self.assertEqual(engine.EXIT_RECEIVER_TIME_INVALID, 6)


class TestWaitForValidTime(unittest.TestCase):
    def test_valid_time_returns_true_fast(self):
        store = _FakeTimeStore(_snap(valid_tow=True, valid_week=True))
        t0 = time.time()
        ok = engine.wait_for_valid_receiver_time(
            store, threading.Event(), timeout_s=10.0)
        self.assertTrue(ok)
        self.assertLess(time.time() - t0, 2.0)

    def test_invalid_time_with_data_times_out(self):
        # Receiver emitting NAV-TIMEGPS but towValid=0 → wait, then False.
        store = _FakeTimeStore(_snap(valid_tow=False, n_updates=9))
        ok = engine.wait_for_valid_receiver_time(
            store, threading.Event(), timeout_s=2.0, no_data_grace_s=30.0)
        self.assertFalse(ok)

    def test_no_navtimegps_fast_giveup_returns_true(self):
        # No NAV-TIMEGPS at all → can't gate on it → proceed (True).
        store = _FakeTimeStore(None)
        t0 = time.time()
        ok = engine.wait_for_valid_receiver_time(
            store, threading.Event(), timeout_s=30.0, no_data_grace_s=0.5)
        self.assertTrue(ok)
        self.assertLess(time.time() - t0, 5.0)

    def test_stale_valid_fix_not_accepted(self):
        # valid flags set but the fix is stale (age > max_age_s) → not
        # accepted; times out.
        store = _FakeTimeStore(_snap(valid_tow=True, age_s=999.0, n_updates=9))
        ok = engine.wait_for_valid_receiver_time(
            store, threading.Event(), timeout_s=2.0, max_age_s=5.0,
            no_data_grace_s=30.0)
        self.assertFalse(ok)

    def test_none_store_proceeds(self):
        # Legacy/mock with no store → can't gate → proceed.
        self.assertTrue(engine.wait_for_valid_receiver_time(
            None, threading.Event(), timeout_s=5.0))

    def test_stop_event_aborts(self):
        store = _FakeTimeStore(_snap(valid_tow=False, n_updates=9))
        ev = threading.Event(); ev.set()
        self.assertFalse(engine.wait_for_valid_receiver_time(
            store, ev, timeout_s=30.0, no_data_grace_s=30.0))


if __name__ == "__main__":
    unittest.main()

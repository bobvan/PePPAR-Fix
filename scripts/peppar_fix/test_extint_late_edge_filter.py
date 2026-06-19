"""Tests for ExtintLateEdgeFilter — ringing late-edge rejection.

Mechanism (PiFace F9P, 2026-06-19): the EXTINT input network rings and
produces a spurious SECOND edge one ringing-period later (~180-400 ns),
which trumps the real edge and mis-steers the DO.  The spurious edges are
strictly LATE (never early) and jump discretely above the running trend by
>> the nominal-delay jitter (~±10 ns).
"""
from __future__ import annotations

import os
import sys
import types
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.extint_reader import (  # noqa: E402
    ExtintLateEdgeFilter, TimTm2Store, _wrap_ns)


class WrapTest(unittest.TestCase):
    def test_wrap_small(self):
        self.assertEqual(_wrap_ns(5), 5)
        self.assertEqual(_wrap_ns(-5), -5)

    def test_wrap_near_second(self):
        # +999_999_999 ns is really -1 ns from the next second
        self.assertEqual(_wrap_ns(999_999_999), -1)


class LateEdgeFilterTest(unittest.TestCase):
    def setUp(self):
        self.f = ExtintLateEdgeFilter(reject_ns=50.0, warmup=4)

    def _run(self, phases):
        return [self.f.accept(p) for p in phases]

    def test_steady_baseline_all_accepted(self):
        """Normal edges with small jitter around a fixed delay all pass."""
        phases = [425, 427, 423, 426, 424, 428, 422, 425, 427]
        self.assertTrue(all(self._run(phases)))
        self.assertEqual(self.f.n_rejected, 0)

    def test_isolated_late_edge_rejected(self):
        """A single ringing edge (+250 ns) is dropped; the next normal edge
        (back at baseline) is kept because the trend never moved."""
        self._run([425, 426, 424, 425, 427])      # warm up + trend
        self.assertFalse(self.f.accept(425 + 250))  # ringing edge -> reject
        self.assertTrue(self.f.accept(426))         # real edge -> accept
        self.assertEqual(self.f.n_rejected, 1)

    def test_early_edge_always_accepted(self):
        """One-sided: an EARLY edge is never ringing and must pass (a real
        DO speed-up must not be filtered)."""
        self._run([425, 426, 424, 425, 427])
        self.assertTrue(self.f.accept(425 - 300))   # very early -> accept
        self.assertEqual(self.f.n_rejected, 0)

    def test_fast_genuine_drift_not_rejected(self):
        """A fast but smooth DO drift (+30 ns/edge) is tracked, not filtered —
        the predictor follows the trend."""
        phases = [425 + 30 * k for k in range(12)]
        self.assertTrue(all(self._run(phases)))
        self.assertEqual(self.f.n_rejected, 0)

    def test_late_jump_on_top_of_drift_rejected(self):
        """During a +30 ns/edge drift, an edge +250 ns above the predicted
        next value is still caught."""
        self._run([425 + 30 * k for k in range(6)])   # establish drift
        nxt = 425 + 30 * 6
        self.assertFalse(self.f.accept(nxt + 250))     # ringing on top -> reject
        self.assertTrue(self.f.accept(425 + 30 * 7))   # back on trend -> accept

    def test_persistent_offset_rebaselines(self):
        """A SUSTAINED late offset is a real DO step, not ringing — after
        max_consecutive_rejects the filter force-accepts and re-baselines so
        EXTINT is never blinded forever (load-bearing on --no-ticc hosts)."""
        f = ExtintLateEdgeFilter(reject_ns=50.0, warmup=4,
                                 max_consecutive_rejects=5)
        for p in [400, 402, 401, 403, 402]:
            f.accept(p)
        # a real +500 ns step: every edge stays at the new level
        decisions = [f.accept(902 + (k % 3)) for k in range(8)]
        self.assertEqual(decisions[:5], [False] * 5)   # rejected as "late"
        self.assertTrue(decisions[5])                  # forced accept + rebase
        self.assertEqual(f.n_rebaseline, 1)
        # now locked at the new level — subsequent edges there are accepted
        self.assertTrue(f.accept(903))

    def test_reset_clears_trend(self):
        self._run([425, 426, 424, 425, 427])
        self.f.reset()
        # after reset it re-warms up, so a big jump is accepted during warmup
        self.assertTrue(self.f.accept(425 + 250))

    def test_reject_ns_must_be_positive(self):
        with self.assertRaises(ValueError):
            ExtintLateEdgeFilter(reject_ns=0)


class _Parsed:
    def __init__(self, tow_ms, sub_ns, count, acc=30):
        self.wnR = 2423
        self.towMsR = tow_ms
        self.towSubMsR = sub_ns
        self.accEst = acc
        self.count = count
        self.flags = 0


class TimTm2StoreFilterTest(unittest.TestCase):
    def test_store_drops_late_edge_from_ekf_but_keeps_others(self):
        store = TimTm2Store()  # default filter ON
        # baseline edges at +425 ns into each second
        for k in range(6):
            store.update(_Parsed(100000 + k * 1000, 425, 200 + k))
        s = store.consume_latest()
        self.assertIsNotNone(s)
        self.assertAlmostEqual(s[0], 425, delta=1)
        # ringing edge: +300 ns later in the SAME-style second -> dropped
        store.update(_Parsed(106000, 725, 206))
        self.assertIsNone(store.consume_latest())   # nothing new for the EKF
        self.assertEqual(store.n_dropped_late_edge, 1)
        # next real edge passes through
        store.update(_Parsed(107000, 426, 207))
        self.assertIsNotNone(store.consume_latest())

    def test_filter_can_be_disabled(self):
        store = TimTm2Store(late_edge_filter=None)
        for k in range(6):
            store.update(_Parsed(100000 + k * 1000, 425, 200 + k))
            store.consume_latest()
        store.update(_Parsed(106000, 725, 206))     # late edge
        s = store.consume_latest()
        self.assertIsNotNone(s)                      # NOT filtered
        self.assertAlmostEqual(s[0], 725, delta=1)
        self.assertEqual(store.n_dropped_late_edge, 0)


if __name__ == "__main__":
    unittest.main()

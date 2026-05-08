"""Tests for the TIM-TM2 ingest path (DOFreqEst Arm 3 producer).

Pin the contract that the engine relies on:
  - phase_residual_ns wraps to (-500ms, +500ms]
  - update() is thread-safe
  - consume_latest() is one-shot per sample
  - max_acc_est_ns gates degraded F9T nav-engine readings
"""
from __future__ import annotations

import os
import sys
import threading
import time
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.extint_reader import TimTm2Store, phase_residual_ns


class _FakeMsg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class PhaseResidualWrapTest(unittest.TestCase):

    def test_at_top_of_second_returns_zero(self):
        self.assertEqual(phase_residual_ns(322391000, 0), 0)
        self.assertEqual(phase_residual_ns(0, 0), 0)

    def test_small_positive_offset(self):
        # 1 µs after top-of-second
        self.assertEqual(phase_residual_ns(322391000, 1000), 1000)
        # 1 ms after top-of-second
        self.assertEqual(phase_residual_ns(322391001, 0), 1_000_000)

    def test_small_negative_via_wrap(self):
        # 1 ms before next second tick → -1 ms
        self.assertEqual(phase_residual_ns(322391999, 0), -1_000_000)
        # 1 µs before next second tick → -1 µs
        self.assertEqual(phase_residual_ns(322391999, 999_000), -1_000)

    def test_smoke_test_observed_value(self):
        # Reproduces the actual numbers we saw in the smoke test:
        # towMs=322391001, towSubMs=479832 → +1.479832 ms
        self.assertEqual(phase_residual_ns(322391001, 479832), 1_479_832)


class TimTm2StoreTest(unittest.TestCase):

    def test_consume_returns_pushed_sample(self):
        store = TimTm2Store()
        msg = _FakeMsg(wnR=1393, towMsR=322391001, towSubMsR=479832,
                       accEst=11, count=5997, flags=0)
        store.update(msg)
        result = store.consume_latest()
        self.assertIsNotNone(result)
        phase, acc = result
        self.assertEqual(phase, 1_479_832)
        self.assertEqual(acc, 11)

    def test_consume_is_one_shot(self):
        store = TimTm2Store()
        store.update(_FakeMsg(wnR=1, towMsR=1000, towSubMsR=0,
                              accEst=5, count=1, flags=0))
        first = store.consume_latest()
        second = store.consume_latest()
        self.assertIsNotNone(first)
        self.assertIsNone(second,
                          "second consume must return None until next update")

    def test_update_overwrites_unconsumed_sample(self):
        store = TimTm2Store()
        store.update(_FakeMsg(wnR=1, towMsR=1000, towSubMsR=0,
                              accEst=5, count=1, flags=0))
        store.update(_FakeMsg(wnR=1, towMsR=2000, towSubMsR=12345,
                              accEst=7, count=2, flags=0))
        # Only the freshest sample should come through.
        phase, acc = store.consume_latest()
        self.assertEqual(phase, 12345)
        self.assertEqual(acc, 7)

    def test_acc_est_gate(self):
        # accEst above max is recorded but not exposed to the engine.
        store = TimTm2Store(max_acc_est_ns=50)
        store.update(_FakeMsg(wnR=1, towMsR=1000, towSubMsR=0,
                              accEst=200, count=1, flags=0))
        self.assertIsNone(store.consume_latest(),
                          "accEst above gate must hide the sample")
        self.assertEqual(store.n_received, 1)
        self.assertEqual(store.n_dropped_acc_est, 1)
        # A subsequent good sample comes through.
        store.update(_FakeMsg(wnR=1, towMsR=2000, towSubMsR=500,
                              accEst=8, count=2, flags=0))
        result = store.consume_latest()
        self.assertIsNotNone(result)
        self.assertEqual(result[1], 8)

    def test_stale_sample_is_skipped(self):
        # A sample older than max_age_s must not be returned.
        store = TimTm2Store()
        store.update(_FakeMsg(wnR=1, towMsR=1000, towSubMsR=0,
                              accEst=5, count=1, flags=0))
        time.sleep(0.05)
        # max_age_s very small so we always trip the staleness check.
        self.assertIsNone(store.consume_latest(max_age_s=0.01))

    def test_thread_safe_update_and_consume(self):
        # Hammer update() from a producer thread while a consumer
        # repeatedly reads.  No exceptions, sane stats.
        store = TimTm2Store()
        stop = threading.Event()

        def producer():
            i = 0
            while not stop.is_set():
                store.update(_FakeMsg(
                    wnR=1, towMsR=1000 + i, towSubMsR=0,
                    accEst=5, count=i, flags=0))
                i += 1

        consumed = []
        def consumer():
            while not stop.is_set():
                r = store.consume_latest()
                if r is not None:
                    consumed.append(r)

        threads = [threading.Thread(target=producer),
                   threading.Thread(target=consumer)]
        for t in threads:
            t.start()
        time.sleep(0.05)
        stop.set()
        for t in threads:
            t.join()
        self.assertGreater(store.n_received, 0)
        # Consumer should have got at least one sample.
        self.assertGreater(len(consumed), 0)


if __name__ == "__main__":
    unittest.main()

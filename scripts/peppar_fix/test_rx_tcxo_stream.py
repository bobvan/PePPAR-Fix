"""Tests for RxTcxoTracker raw-stream feed (I-151540 Item 7).

The tracker's update() now accepts an absolute t_s timestamp.  When
present, the slope LS regression uses real time differences instead
of integer indices — variable-cadence raw qErr streams from
QErrStore are handled correctly without per-PPS-edge matching.

The new update_from_qerr_store() method drains samples newer than
the tracker's last consumed timestamp and feeds each into update()
with its host_time.  Replaces the legacy per-edge feed via
QErrStore.match_pps_mono.
"""
from __future__ import annotations

import math
import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix_engine import RxTcxoTracker
from realtime_ppp import QErrStore


class _FakeQErrStore:
    """Lightweight stand-in matching QErrStore.drain_since semantics."""
    def __init__(self, samples):
        self._samples = list(samples)  # [(host_time, qerr_ns), ...]
    def drain_since(self, host_time_floor):
        return [(t, q) for (t, q) in self._samples if t > host_time_floor]


class BackwardCompatTest(unittest.TestCase):
    """Legacy callers that omit t_s should keep their existing semantics:
    integer-index slope LS with implicit dt=1.0."""

    def test_legacy_update_no_t_s(self):
        rx = RxTcxoTracker(freq_window=10)
        # Feed 10 samples at fixed 1 Hz with constant slope (drift)
        # baked in via predicted_drift to break ticks correctly.
        # 1 ns/s drift → unwrapped phase grows by 1 ns/sample.
        for i in range(10):
            qerr_ns = (i * 1.0) % 8.0  # wraps at 8-ns tick
            if qerr_ns >= 4.0:
                qerr_ns -= 8.0
            rx.update(qerr_ns, predicted_drift_ns_per_s=1.0)
        slope, sigma, n = rx.freq_ppb_with_sigma()
        self.assertEqual(n, 10)
        self.assertIsNotNone(slope)
        self.assertAlmostEqual(slope, 1.0, delta=0.001)


class VariableDtSlopeTest(unittest.TestCase):
    """When t_s is supplied, slope reflects real time differences.
    A 2× sample density gives the same slope (per second), not the
    same slope (per index)."""

    def test_uniform_t_s_matches_legacy(self):
        rx_legacy = RxTcxoTracker(freq_window=10)
        rx_ts = RxTcxoTracker(freq_window=10)
        for i in range(10):
            q = (i * 1.0) % 8.0
            if q >= 4.0:
                q -= 8.0
            rx_legacy.update(q, predicted_drift_ns_per_s=1.0)
            rx_ts.update(q, t_s=float(i),
                          predicted_drift_ns_per_s=1.0)
        s_legacy, _, _ = rx_legacy.freq_ppb_with_sigma()
        s_ts, _, _ = rx_ts.freq_ppb_with_sigma()
        self.assertAlmostEqual(s_legacy, s_ts, places=10,
                                msg="uniform t_s == legacy index path")

    def test_double_density_same_slope_per_second(self):
        # 1 ppb drift sampled at 0.5 s vs 1.0 s should give the same
        # slope (per-second).  The integer-index path would give 2×
        # slope at higher density — wrong.
        rx_1hz = RxTcxoTracker(freq_window=10)
        rx_2hz = RxTcxoTracker(freq_window=10)
        for i in range(10):
            t = i * 1.0
            q = (t * 1.0) % 8.0
            if q >= 4.0:
                q -= 8.0
            rx_1hz.update(q, t_s=t, predicted_drift_ns_per_s=1.0)
        for i in range(10):
            t = i * 0.5
            q = (t * 1.0) % 8.0
            if q >= 4.0:
                q -= 8.0
            rx_2hz.update(q, t_s=t, predicted_drift_ns_per_s=1.0)
        s_1, _, _ = rx_1hz.freq_ppb_with_sigma()
        s_2, _, _ = rx_2hz.freq_ppb_with_sigma()
        self.assertAlmostEqual(s_1, 1.0, delta=0.01)
        self.assertAlmostEqual(s_2, 1.0, delta=0.01,
                                msg="2x density must yield same slope per second")


class UpdateFromQErrStoreTest(unittest.TestCase):

    def test_drain_and_feed_consumes_samples(self):
        rx = RxTcxoTracker(freq_window=20)
        # 12 samples at 1 Hz with 1 ppb drift.
        samples = []
        for i in range(12):
            t = float(i)
            q = (t * 1.0) % 8.0
            if q >= 4.0:
                q -= 8.0
            samples.append((t, q))
        store = _FakeQErrStore(samples)
        n_drained, last_phase = rx.update_from_qerr_store(
            store, predicted_drift_ns_per_s=1.0)
        self.assertEqual(n_drained, 12)
        self.assertEqual(rx.n_samples, 12)
        slope, _, _ = rx.freq_ppb_with_sigma()
        self.assertAlmostEqual(slope, 1.0, delta=0.05)

    def test_second_drain_only_returns_new_samples(self):
        # Cursor semantics: after the first drain, a subsequent call
        # with the same store should return zero unless new samples
        # arrived.  The tracker's _prev_t_s is its cursor.
        rx = RxTcxoTracker(freq_window=20)
        samples = [(float(i), 0.1 * i) for i in range(5)]
        store = _FakeQErrStore(samples)
        n1, _ = rx.update_from_qerr_store(store)
        self.assertEqual(n1, 5)
        n2, _ = rx.update_from_qerr_store(store)
        self.assertEqual(n2, 0,
                         "second drain with no new samples must return 0")

    def test_real_qerr_store_integration(self):
        # End-to-end: feed real QErrStore, drain through tracker.
        store = QErrStore(maxlen=64)
        # update() takes qerr_ps (not ns) and records host_time
        # internally.  Sequence three TIM-TP messages.
        import time as _time
        store.update(qerr_ps=1000, tow_ms=1000)        # 1 ns
        _time.sleep(0.005)
        store.update(qerr_ps=2000, tow_ms=2000)        # 2 ns
        _time.sleep(0.005)
        store.update(qerr_ps=-1500, tow_ms=3000)       # -1.5 ns
        rx = RxTcxoTracker(freq_window=10)
        n, _ = rx.update_from_qerr_store(store)
        self.assertEqual(n, 3)
        self.assertEqual(rx.n_samples, 3)


class QErrStoreDrainSinceTest(unittest.TestCase):

    def test_floor_zero_returns_all(self):
        store = QErrStore(maxlen=64)
        store.update(qerr_ps=1000, tow_ms=1000)
        store.update(qerr_ps=2000, tow_ms=2000)
        out = store.drain_since(0.0)
        self.assertEqual(len(out), 2)

    def test_floor_excludes_seen(self):
        store = QErrStore(maxlen=64)
        store.update(qerr_ps=1000, tow_ms=1000)
        first_t, _ = store.drain_since(0.0)[0]
        # Floor at the first sample's host_time should exclude it.
        out = store.drain_since(first_t)
        self.assertEqual(len(out), 0)
        # Add a new sample; should now be included.
        store.update(qerr_ps=3000, tow_ms=3000)
        out = store.drain_since(first_t)
        self.assertEqual(len(out), 1)

    def test_invalid_samples_not_in_drain(self):
        # qerr_invalid=True skips the in-memory deque, so drain_since
        # never returns them.
        store = QErrStore(maxlen=64)
        store.update(qerr_ps=1000, tow_ms=1000)
        store.update(qerr_ps=2000, tow_ms=2000, qerr_invalid=True)
        out = store.drain_since(0.0)
        self.assertEqual(len(out), 1)


if __name__ == "__main__":
    unittest.main()

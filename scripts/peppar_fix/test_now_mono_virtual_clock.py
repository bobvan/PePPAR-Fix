"""now_mono virtual-clock parameterization of the freshness getters.

The gate-purity refactor (pos_replay milestone 0,
docs/pos-replay-capture-manifest.md §6): freshness/age decisions accept an
optional ``now_mono`` so they can be driven by a captured/virtual
``recv_mono`` instead of live ``time.monotonic()`` — the precondition for
bit-identical deterministic replay.  Default (None) = live, byte-identical.
"""
import os
import sys
import time
import unittest

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from realtime_ppp import QErrStore, NavClockStore, NavTimeGpsStore  # noqa: E402


class TestQErrStoreNowMono(unittest.TestCase):
    def _store_with_sample(self):
        s = QErrStore()
        s.update(qerr_ps=1000.0, tow_ms=12345)
        return s, s._samples[-1]["host_time"]

    def test_get_decision_is_pure_in_now_mono(self):
        s, ht = self._store_with_sample()
        # Fresh / stale is decided ONLY by now_mono vs the stored stamp —
        # independent of wall-clock.
        self.assertIsNotNone(s.get(max_age_s=2.0, now_mono=ht + 1.0)[0])
        self.assertIsNone(s.get(max_age_s=2.0, now_mono=ht + 10.0)[0])

    def test_get_default_uses_live_clock(self):
        s, _ = self._store_with_sample()
        # No now_mono → live monotonic → a just-added sample is fresh.
        self.assertIsNotNone(s.get(max_age_s=2.0)[0])

    def test_snapshot_age_is_pure_in_now_mono(self):
        s, ht = self._store_with_sample()
        q, _tow, age = s.snapshot(max_age_s=5.0, now_mono=ht + 3.0)
        self.assertIsNotNone(q)
        self.assertAlmostEqual(age, 3.0, places=9)
        self.assertIsNone(s.snapshot(max_age_s=5.0, now_mono=ht + 6.0)[0])


class TestNavStoreAgeNowMono(unittest.TestCase):
    def test_navclock_age_uses_now_mono(self):
        s = NavClockStore()
        s._host_mono = 100.0
        s._update_count = 1
        out = s.get(now_mono=104.0)
        self.assertIsNotNone(out)
        self.assertAlmostEqual(out["age_s"], 4.0, places=9)

    def test_navtimegps_age_uses_now_mono(self):
        s = NavTimeGpsStore()
        s._host_mono = 200.0
        s._update_count = 1
        out = s.get(now_mono=207.5)
        self.assertIsNotNone(out)
        self.assertAlmostEqual(out["age_s"], 7.5, places=9)

    def test_navclock_default_uses_live_clock(self):
        s = NavClockStore()
        s._host_mono = time.monotonic()
        s._update_count = 1
        out = s.get()                 # live → tiny age
        self.assertIsNotNone(out)
        self.assertLess(out["age_s"], 1.0)


if __name__ == "__main__":
    unittest.main()

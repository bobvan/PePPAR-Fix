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
import types
import unittest

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from realtime_ppp import QErrStore, NavClockStore, NavTimeGpsStore  # noqa: E402


class TestEndToEndPurity(unittest.TestCase):
    """The decisive property: with the ingest stamp from a captured
    ``recv_mono`` AND the read from a virtual ``now_mono``, the freshness
    age is a pure function of captured time — no live clock anywhere.  This
    is what makes the getter deterministic *end-to-end* under replay."""

    def test_qerr_age_is_purely_captured_time(self):
        s = QErrStore()
        s.update(qerr_ps=1000.0, tow_ms=12345, recv_mono=5000.0)  # captured stamp
        # snapshot age is exactly now_mono - recv_mono, regardless of wall clock
        _q, _tow, age = s.snapshot(max_age_s=10.0, now_mono=5003.5)
        self.assertAlmostEqual(age, 3.5, places=9)
        self.assertIsNotNone(s.get(max_age_s=2.0, now_mono=5001.0)[0])
        self.assertIsNone(s.get(max_age_s=2.0, now_mono=5005.0)[0])

    def test_match_pps_mono_pure_end_to_end(self):
        """match_pps_mono was already pure in pps_recv_mono; with the ingest
        stamp also captured, the qErr↔PPS match is deterministic end-to-end."""
        s = QErrStore()
        s.update(qerr_ps=2000.0, tow_ms=1000, recv_mono=8000.0)
        # PPS edge ~0.9 s after the TIM-TP (the expected offset) → matches
        qerr_ns, offset = s.match_pps_mono(pps_recv_mono=8000.9)
        self.assertIsNotNone(qerr_ns)
        self.assertAlmostEqual(offset, 0.9, places=6)
        # an edge far from the expected offset → no match
        self.assertEqual(s.match_pps_mono(pps_recv_mono=8005.0)[0], None)

    def test_navclock_ingest_and_read_pure(self):
        s = NavClockStore()
        msg = types.SimpleNamespace(clkB=1.0, clkD=2.0, tAcc=3.0, fAcc=4.0,
                                    iTOW=5)
        s.update(msg, recv_mono=100.0)
        self.assertAlmostEqual(s.get(now_mono=106.0)["age_s"], 6.0, places=9)


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


class TestTimTm2StoreNowMono(unittest.TestCase):
    """milestone-0b completes the EXTTS store (TimTm2Store) that milestone-0
    missed: its ingest stamp takes recv_mono and consume_latest's freshness
    takes now_mono, so capture + live converge on one captured clock."""

    def _store_with_sample(self, recv_mono):
        from peppar_fix.extint_reader import TimTm2Store
        s = TimTm2Store(late_edge_filter=None)   # no late-edge rejection
        parsed = types.SimpleNamespace(wnR=2300, towMsR=1000, towSubMsR=0,
                                       accEst=10, count=1, flags=0)
        s.update(parsed, recv_mono=recv_mono)
        return s

    def test_age_is_pure_in_captured_recv_mono_and_now_mono(self):
        # ingest stamp from recv_mono + read from now_mono → freshness is a
        # pure function of captured time, no live clock anywhere.
        self.assertIsNotNone(
            self._store_with_sample(200.0).consume_latest(
                max_age_s=10.0, now_mono=209.0))      # age 9 < 10 → fresh
        self.assertIsNone(
            self._store_with_sample(200.0).consume_latest(
                max_age_s=10.0, now_mono=211.0))      # age 11 > 10 → stale

    def test_default_uses_live_clock(self):
        s = self._store_with_sample(time.monotonic())
        self.assertIsNotNone(s.consume_latest(max_age_s=5.0))   # live → fresh


if __name__ == "__main__":
    unittest.main()

"""Tests for PvtClockStore + the reader's PVTGeodetic capture.

The store carries the mosaic-T's OWN clock solution (PVTGeodetic RxClkBias /
RxClkDrift) so the servo can log our carrier-phase steering vs theirs.
"""
import os
import queue
import sys
import unittest
from types import SimpleNamespace

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix.sbf_obs_source import PvtClockStore, sbf_obs_reader


def _pvt(bias_ms, drift_ppm=0.5, mode=1, tow=100):
    return SimpleNamespace(identity="PVTGeodetic", RxClkBias=bias_ms,
                           RxClkDrift=drift_ppm, Mode=mode, TOW=tow)


class PvtClockStoreTest(unittest.TestCase):

    def test_empty(self):
        self.assertIsNone(PvtClockStore().latest())

    def test_update_and_read(self):
        s = PvtClockStore()
        s.update_from_sbf(_pvt(0.001234, drift_ppm=-0.5, mode=4, tow=42))
        tow, bias, drift, mode = s.latest()
        self.assertEqual(tow, 42)
        self.assertAlmostEqual(bias, 0.001234)
        self.assertAlmostEqual(drift, -0.5)
        self.assertEqual(mode, 4)

    def test_do_not_use_sentinel_rejected(self):
        s = PvtClockStore()
        s.update_from_sbf(_pvt(0.005))
        s.update_from_sbf(_pvt(-2.0e10))   # SBF Do-Not-Use
        # the DNU update must not clobber the last good value
        self.assertAlmostEqual(s.latest()[1], 0.005)

    def test_reader_routes_pvt_to_store_skips_non_meas(self):
        # PVTGeodetic → store; other non-MeasEpoch blocks are skipped and no
        # obs are queued (no MeasEpoch present, so the decode path isn't hit).
        store = PvtClockStore()
        obs_q = queue.Queue()
        messages = [(b"", _pvt(0.00042, mode=2, tow=7)),
                    (b"", SimpleNamespace(identity="ReceiverTime")),
                    (b"", None)]
        n = sbf_obs_reader(iter(messages), obs_q, None, sig_lookup={},
                           pvt_store=store, systems={"gps"})
        self.assertEqual(n, 0, "no MeasEpoch → nothing queued")
        self.assertEqual(obs_q.qsize(), 0)
        self.assertIsNotNone(store.latest(), "PVTGeodetic captured to store")
        self.assertAlmostEqual(store.latest()[1], 0.00042)
        self.assertEqual(store.latest()[3], 2)


if __name__ == "__main__":
    unittest.main()

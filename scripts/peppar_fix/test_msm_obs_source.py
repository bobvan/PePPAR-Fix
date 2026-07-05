"""Tests for the MSM engine obs-source seam (I-030423 step 3).

Epoch assembly, GPS-week timestamping (rollover-safe), and the reader loop's
routing — all with injected message iterators + wall clock, no network.
"""
import os
import queue
import sys
import unittest
from datetime import timedelta
from types import SimpleNamespace

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix import msm_obs_source as s  # noqa: E402
from peppar_fix.rtcm_msm_obs import default_gps_sig_lookup  # noqa: E402

_C = 299792458.0
_FIXED_NOW = s._GPS_EPOCH + timedelta(seconds=2400 * s._WEEK_S + 100000.0 / 1000
                                      - s._GPS_UTC_LEAP_S)


def _msm(tow_ms, sats, sigs, rough, cells, msg_num=1077, cellmask=None):
    nsat, nsig = len(sats), len(sigs)
    attrs = dict(
        DF002=msg_num, DF004=tow_ms,
        DF394=sum(1 << (64 - p) for p in sats),
        DF395=sum(1 << (32 - x) for x in sigs),
        DF396=(cellmask if cellmask is not None else (1 << (nsat * nsig)) - 1))
    for i, rg in enumerate(rough):
        attrs[f"DF397_{i+1:02d}"] = int(rg)
        attrs[f"DF398_{i+1:02d}"] = rg - int(rg)
    for ci, c in enumerate(cells, start=1):
        attrs[f"DF405_{ci:02d}"] = c["pr_fine"]
        attrs[f"DF406_{ci:02d}"] = c.get("ph_fine")
        attrs[f"DF408_{ci:02d}"] = c.get("cno", 45.0)
        attrs[f"DF407_{ci:02d}"] = c.get("lock", 9)
        attrs[f"DF420_{ci:02d}"] = c.get("half", 0)
    return SimpleNamespace(identity=str(msg_num), **attrs)


def _gps_epoch_msg(tow_ms, *, sv=5):
    """One GPS L1CA+L2CL dual-freq epoch for one SV at the given tow."""
    return _msm(tow_ms, [sv], [2, 16], [73.0],
                [{"pr_fine": 0.0004, "ph_fine": 0.00042},
                 {"pr_fine": 0.0005, "ph_fine": 0.00051}])


class TowToGpsDatetimeTest(unittest.TestCase):
    def test_basic_within_week(self):
        dt = s.tow_to_gps_datetime(100000, _FIXED_NOW)
        gps_now = _FIXED_NOW + timedelta(seconds=s._GPS_UTC_LEAP_S)
        self.assertLess(abs((dt - gps_now).total_seconds()), 1.0)

    def test_rollover_nudges_week_down(self):
        # wall clock just after a week boundary (tow_now≈30s), message tow near
        # the END of the week → it belongs to the previous week.
        gps_now = s._GPS_EPOCH + timedelta(seconds=2400 * s._WEEK_S + 30)
        now_utc = gps_now - timedelta(seconds=s._GPS_UTC_LEAP_S)
        dt = s.tow_to_gps_datetime(604700 * 1000, now_utc)
        self.assertLess(dt, gps_now)                       # placed before now
        self.assertAlmostEqual((gps_now - dt).total_seconds(), 130, delta=5)


class AssemblerTest(unittest.TestCase):
    def setUp(self):
        self.look = default_gps_sig_lookup("GPS-L2CL")

    def _asm(self):
        return s.MsmEpochAssembler(self.look, {"gps"}, None)

    def test_single_epoch_released_on_flush(self):
        a = self._asm()
        self.assertIsNone(a.add(_gps_epoch_msg(100000)))   # no boundary yet
        tow, obs = a.flush()
        self.assertEqual(tow, 100000)
        self.assertEqual(len(obs), 1)
        self.assertEqual(obs[0]["sv"], "G05")

    def test_boundary_emits_previous_epoch(self):
        a = self._asm()
        a.add(_gps_epoch_msg(100000))
        completed = a.add(_gps_epoch_msg(101000))          # new tow → emit prev
        self.assertIsNotNone(completed)
        self.assertEqual(completed[0], 100000)
        self.assertEqual(a.flush()[0], 101000)             # the new one still pending

    def test_multi_message_same_epoch_merges(self):
        # two GPS messages at the same tow (different SVs) → one epoch, 2 SVs
        a = self._asm()
        self.assertIsNone(a.add(_gps_epoch_msg(100000, sv=5)))
        self.assertIsNone(a.add(_gps_epoch_msg(100000, sv=10)))
        tow, obs = a.flush()
        self.assertEqual({o["sv"] for o in obs}, {"G05", "G10"})

    def test_unsupported_constellation_ignored(self):
        a = self._asm()
        self.assertIsNone(a.add(_msm(100000, [5], [2], [73.0],
                                     [{"pr_fine": 0.0004}], msg_num=1097)))
        self.assertIsNone(a.flush())                       # nothing accumulated


class ReaderLoopTest(unittest.TestCase):
    def setUp(self):
        self.look = default_gps_sig_lookup("GPS-L2CL")
        self.q = queue.Queue()

    def _read(self, msgs, **kw):
        stop = SimpleNamespace(is_set=lambda: False)
        return s.msm_ntrip_reader(iter(msgs), self.q, stop, self.look,
                                  systems={"gps"}, now_fn=lambda: _FIXED_NOW,
                                  **kw)

    def test_queues_epoch_tuples(self):
        n = self._read([_gps_epoch_msg(100000), _gps_epoch_msg(101000)])
        self.assertEqual(n, 2)
        gps_time, obs = self.q.get_nowait()
        self.assertTrue(hasattr(gps_time, "year"))         # a datetime
        self.assertEqual(obs[0]["sv"], "G05")

    def test_routes_eph_to_beph(self):
        seen = []
        beph = SimpleNamespace(update_from_rtcm=lambda m: seen.append(m.identity))
        eph_msg = SimpleNamespace(identity="1019", DF002=1019)
        n = self._read([eph_msg, _gps_epoch_msg(100000)], beph=beph)
        self.assertEqual(seen, ["1019"])
        self.assertEqual(n, 1)                             # eph not counted as epoch

    def test_station_arp_callback(self):
        got = []
        arp = SimpleNamespace(identity="1005", DF002=1005,
                              DF025=1.0, DF026=2.0, DF027=3.0)
        self._read([arp, _gps_epoch_msg(100000)], station_cb=got.append)
        self.assertEqual(got, [(1.0, 2.0, 3.0)])

    def test_stop_event_halts(self):
        stop = SimpleNamespace(_n=0)

        def is_set():
            stop._n += 1
            return stop._n > 1                             # stop after first msg
        n = s.msm_ntrip_reader(
            iter([_gps_epoch_msg(100000), _gps_epoch_msg(101000),
                  _gps_epoch_msg(102000)]),
            self.q, SimpleNamespace(is_set=is_set), self.look,
            systems={"gps"}, now_fn=lambda: _FIXED_NOW)
        self.assertLessEqual(n, 1)

    def test_single_freq_epoch_not_queued(self):
        only_l1 = _msm(100000, [5], [2], [73.0],
                       [{"pr_fine": 0.0004, "ph_fine": 0.0004}])
        n = self._read([only_l1])
        self.assertEqual(n, 0)
        self.assertTrue(self.q.empty())


if __name__ == "__main__":
    unittest.main()

"""pos_replay runner: drive the real AntPosEstThread._process_epoch from a
replayed bundle (the loop closer)."""
import logging
import os
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import realtime_ppp as r                                    # noqa: E402
from peppar_fix import pos_replay_driver as drv             # noqa: E402
from peppar_fix import pos_replay_filter as prf             # noqa: E402
from peppar_fix.raw_capture import RawCaptureBundle         # noqa: E402
from peppar_fix.receiver import get_driver                  # noqa: E402

_TRUTH = (-2_730_000.0, -4_440_000.0, 3_975_000.0)


def _synthetic_rawx():
    d = get_driver("f9t")
    rev = {name: (g, s) for (g, s), name in d.signal_names.items()}
    _, f1, f2, _ = (getattr(d, "if_pairs", None) or r.IF_PAIRS)[0]
    (g1, s1), (g2, s2) = rev[f1], rev[f2]
    return SimpleNamespace(
        rcvTow=100.0, week=2300, leapS=18, numMeas=2,
        gnssId=[g1, g2], sigId=[s1, s2], svId=[1, 1],
        prMes=[22_000_000.0, 22_000_000.0],
        cpMes=[115_000_000.0, 90_000_000.0],
        cno=[45, 44], locktime=[5000.0, 5000.0],
        prValid=[True, True], cpValid=[True, True],
        halfCyc=[True, True], clk_reset=False)


def _bundle(d, n_rawx=1):
    b = RawCaptureBundle(d)
    b.write_manifest(host="h", started_iso="2026-01-01T00:00:00+00:00",
                     conventions={"receiver": "f9t", "systems": ["gps"]})
    for i in range(n_rawx):
        b.record("ubx", b"RAWX-%d" % i, recv_mono=100.0 + i)
    b.close()


class TestEpochSink(unittest.TestCase):
    def test_sink_called_inline_per_epoch(self):
        rawx = _synthetic_rawx()
        seen = []
        with tempfile.TemporaryDirectory() as d:
            _bundle(d, n_rawx=2)
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                rd = drv.ReplayDriver(
                    d, epoch_sink=lambda gt, obs, c: seen.append((gt, len(obs))))
                rd.run()
        self.assertEqual(len(seen), 2)             # one call per RAWX epoch
        self.assertEqual(seen[0][1], 1)            # G01 dual-freq observation
        # epoch_sink implies decode_obs
        self.assertTrue(rd.decode_obs)

    def test_no_sink_no_calls(self):
        with tempfile.TemporaryDirectory() as d:
            RawCaptureBundle(d).close()
            rd = drv.ReplayDriver(d)
            self.assertIsNone(rd.epoch_sink)


class TestPppStateCapture(unittest.TestCase):
    def test_captures_ppp_state_lines_only(self):
        log = logging.getLogger("peppar-fix")
        with prf.PppStateCapture() as cap:
            log.info("[PPP_STATE] gps=2026-01-01T00:00:00+00:00 epoch=1 n=8 "
                     "ecef=1.0,2.0,3.0 sigma_pos=0.1m ztd=+0.0m sigma_ztd=0.0m")
            log.info("[STATUS] unrelated line")
        self.assertEqual(len(cap.lines), 1)
        self.assertTrue(cap.lines[0].startswith("[PPP_STATE]"))

    def test_detaches_on_exit(self):
        log = logging.getLogger("peppar-fix")
        with prf.PppStateCapture() as cap:
            pass
        log.info("[PPP_STATE] should-not-be-captured")
        self.assertEqual(cap.lines, [])


class TestRunPosReplayDrivesFilter(unittest.TestCase):
    def test_drives_real_process_epoch(self):
        # the loop closer: a real AntPosEstThread is built and its
        # _process_epoch is driven over the replayed epoch.  With no broadcast
        # eph in the bundle the filter returns early (no sat positions → no
        # [PPP_STATE]), but the DRIVE is proven: _prev_t advances to the epoch.
        rawx = _synthetic_rawx()
        with tempfile.TemporaryDirectory() as d:
            _bundle(d, n_rawx=1)
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                res = prf.run_pos_replay(d, _TRUTH)
        self.assertEqual(res["n_epochs_decoded"], 1)
        # _process_epoch ran → _prev_t set to the epoch's gps_time (year ~2024)
        self.assertIsNotNone(res["thread"]._prev_t)
        self.assertGreaterEqual(res["thread"]._prev_t.year, 2023)

    def test_spy_confirms_process_epoch_called(self):
        rawx = _synthetic_rawx()
        with tempfile.TemporaryDirectory() as d:
            _bundle(d, n_rawx=3)
            # build driver + thread by hand to spy on _process_epoch
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                rd = drv.ReplayDriver(d, decode_obs=True)
                thread = prf.build_filter_thread(rd, _TRUTH, systems=["gps"])
                calls = []
                real = thread._process_epoch
                def _spy(gt, obs, c):
                    calls.append(gt)
                    return real(gt, obs, c)
                rd.epoch_sink = _spy
                rd.run()
        self.assertEqual(len(calls), 3)            # driven once per RAWX epoch


class TestProductSwapDriver(unittest.TestCase):
    def test_captured_ssr_not_applied_when_swapped(self):
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.write_manifest(host="h", started_iso="2026-01-01T00:00:00+00:00",
                             conventions={"receiver": "f9t", "systems": ["gps"]})
            b.record("ssr", b"\xd3\x00\x05dummy", recv_mono=100.0)
            b.close()
            # product-swap: captured SSR is traced but NOT routed
            with mock.patch("realtime_ppp.route_rtcm_message") as route:
                rd = drv.ReplayDriver(d, apply_captured_corrections=False)
                rd.run()
            route.assert_not_called()
            self.assertIn("ssr:swapped-out", [i for _t, _s, i in rd.trace])
            self.assertEqual(rd.counts["ssr"], 1)        # still counted

    def test_captured_ssr_applied_by_default(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.write_manifest(host="h", started_iso="2026-01-01T00:00:00+00:00",
                             conventions={"receiver": "f9t"})
            b.record("ssr", b"\xd3\x00\x05dummy", recv_mono=100.0)
            b.close()
            # mock the parser (dummy bytes don't parse) so dispatch reaches the
            # router; assert the captured SSR IS routed by default (vs swapped)
            with mock.patch("pyrtcm.RTCMReader.parse",
                            return_value=SimpleNamespace(identity="1060")), \
                 mock.patch("realtime_ppp.route_rtcm_message",
                            return_value=("ssr", None)) as route:
                rd = drv.ReplayDriver(d)                 # default: apply
                rd.run()
            route.assert_called_once()                   # captured SSR applied


class TestProductSwapRunner(unittest.TestCase):
    def test_loader_called_and_captured_swapped_out(self):
        rawx = _synthetic_rawx()
        seen = {}

        def _loader(stores):
            seen["called"] = True
            seen["has_ssr"] = "ssr" in stores and "beph" in stores

        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.write_manifest(host="h", started_iso="2026-01-01T00:00:00+00:00",
                             conventions={"receiver": "f9t", "systems": ["gps"]})
            b.record("ssr", b"\xd3\x00\x05dummy", recv_mono=99.0)
            b.record("ubx", b"RAWX-0", recv_mono=100.0)
            b.close()
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx), \
                 mock.patch("realtime_ppp.route_rtcm_message") as route:
                res = prf.run_pos_replay(d, _TRUTH, corrections_loader=_loader)
            self.assertTrue(res["product_swapped"])
            self.assertTrue(seen.get("called"))          # loader ran ...
            self.assertTrue(seen.get("has_ssr"))         # ... with the stores
            route.assert_not_called()                    # captured SSR swapped out
            self.assertIn("ssr:swapped-out",
                          [i for _t, _s, i in res["driver"].trace])

    def test_no_loader_means_no_swap(self):
        rawx = _synthetic_rawx()
        with tempfile.TemporaryDirectory() as d:
            _bundle(d, n_rawx=1)
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                res = prf.run_pos_replay(d, _TRUTH)
            self.assertFalse(res["product_swapped"])


class TestPerStreamSwap(unittest.TestCase):
    def test_swap_ssr_keep_eph(self):
        # ssr-source swap: swap {ssr} but KEEP eph (orbits) — eph still routed,
        # ssr traced swapped-out
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.write_manifest(host="h", started_iso="2026-01-01T00:00:00+00:00",
                             conventions={"receiver": "f9t"})
            b.record("ssr", b"\xd3\x00\x05ssr", recv_mono=99.0)
            b.record("eph", b"\xd3\x00\x05eph", recv_mono=99.5)
            b.close()
            with mock.patch("pyrtcm.RTCMReader.parse",
                            return_value=SimpleNamespace(identity="1019")), \
                 mock.patch("realtime_ppp.route_rtcm_message",
                            return_value=("eph", 5)) as route:
                rd = drv.ReplayDriver(d, swap_streams={"ssr", "ssr_bias"})
                rd.run()
            idents = [i for _t, _s, i in rd.trace]
            self.assertIn("ssr:swapped-out", idents)     # ssr swapped
            self.assertIn("eph:1019", idents)            # eph KEPT (routed)
            route.assert_called_once()                   # only eph routed


class TestSsrRecordsLoader(unittest.TestCase):
    def test_loads_records_into_ssrstate(self):
        import json
        from ssr_corrections import SSRState
        recs = {"epoch_s": 100.0, "records": [
            {"prn": "G01", "iode": 7, "orbit": [0.1, 0.2, 0.3],
             "clock": 0.05, "code_bias": {"C1C": -1.2, "C5Q": -1.5}}]}
        with tempfile.NamedTemporaryFile("w", suffix=".json",
                                         delete=False) as f:
            json.dump(recs, f)
            path = f.name
        try:
            loader = prf.make_ssr_records_loader(path)
            ssr = SSRState()
            loader({"ssr": ssr})
            self.assertAlmostEqual(ssr.get_code_bias("G01", "C1C"), -1.2)
            self.assertAlmostEqual(ssr.get_code_bias("G01", "C5Q"), -1.5)
            self.assertEqual(ssr.n_orbit, 1)
            self.assertEqual(ssr.n_clock, 1)
        finally:
            os.unlink(path)

    def test_empty_records_raises_coverage_check(self):
        import json
        from ssr_corrections import SSRState
        with tempfile.NamedTemporaryFile("w", suffix=".json",
                                         delete=False) as f:
            json.dump({"epoch_s": 0.0, "records": []}, f)
            path = f.name
        try:
            with self.assertRaises(ValueError):
                prf.make_ssr_records_loader(path)({"ssr": SSRState()})
        finally:
            os.unlink(path)


if __name__ == "__main__":
    unittest.main()

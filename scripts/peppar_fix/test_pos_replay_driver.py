"""pos_replay_driver — the deterministic stage-1 re-feed.

The decisive properties: two replays of one bundle are bit-identical, the
result is independent of wall-clock time, and the re-feed lands in the real
engine stores via their recv_mono ingest path.
"""
import os
import sys
import tempfile
import time
import unittest

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix import pos_replay_driver as drv  # noqa: E402
from peppar_fix.raw_capture import RawCaptureBundle  # noqa: E402


def _ubx_nav_clock(itow, clkb):
    from pyubx2 import UBXMessage, GET
    return UBXMessage("NAV", "NAV-CLOCK", GET, iTOW=itow, clkB=clkb,
                      clkD=1, tAcc=2, fAcc=3).serialize()


def _ubx_tim_tp(tow_ms, qerr, qerr_invalid=0):
    from pyubx2 import UBXMessage, GET
    return UBXMessage("TIM", "TIM-TP", GET, towMS=tow_ms, qErr=qerr,
                      qErrInvalid=qerr_invalid).serialize()


def _make_bundle(d):
    """A small mixed bundle: UBX (TIM-TP, NAV-CLOCK) + TICC + a deferred SSR."""
    b = RawCaptureBundle(d)
    b.record("ubx", _ubx_tim_tp(900, 1234), recv_mono=100.0)
    b.record("ubx", _ubx_nav_clock(1000, 10), recv_mono=100.2)
    b.record_line("ticc", "  0.123456789012 chA", recv_mono=100.5)
    b.record_line("ticc", "  1.234567890123 chB", recv_mono=101.5)
    b.record("ubx", _ubx_nav_clock(2000, 20), recv_mono=101.7)
    b.record("ssr", b"\xd3\x00\x05dummy", recv_mono=101.8)   # deferred → stage 2
    b.close()
    return d


class TestReplayDeterminism(unittest.TestCase):
    def test_two_runs_bit_identical(self):
        with tempfile.TemporaryDirectory() as d:
            _make_bundle(d)
            a, b = drv.ReplayDriver(d), drv.ReplayDriver(d)
            a.run(); b.run()
            self.assertEqual(a.trace, b.trace)                  # same trace
            self.assertEqual(a.freshness_snapshot(),
                             b.freshness_snapshot())            # same store state

    def test_independent_of_wall_clock(self):
        # Sleeping between runs must not change a thing — replay reads the
        # virtual clock, never time.monotonic().
        with tempfile.TemporaryDirectory() as d:
            _make_bundle(d)
            a = drv.ReplayDriver(d); a.run()
            time.sleep(0.05)
            b = drv.ReplayDriver(d); b.run()
            self.assertEqual(a.freshness_snapshot(), b.freshness_snapshot())

    def test_records_in_recv_mono_order(self):
        with tempfile.TemporaryDirectory() as d:
            _make_bundle(d)
            r = drv.ReplayDriver(d); r.run()
            times = [t for t, _s, _i in r.trace]
            self.assertEqual(times, sorted(times))              # monotone replay
            self.assertAlmostEqual(r.clock.now_mono, 101.8)     # last record


class TestReplayDispatch(unittest.TestCase):
    def test_ubx_lands_in_stores(self):
        with tempfile.TemporaryDirectory() as d:
            _make_bundle(d)
            r = drv.ReplayDriver(d); r.run()
            # qErr ingested with the captured recv_mono → fresh at that time
            q = r.stores["qerr"].get(max_age_s=5.0, now_mono=100.5)
            self.assertIsNotNone(q[0])
            # NAV-CLOCK: latest is iTOW=2000/clkB=20 at recv_mono 101.7
            nc = r.stores["nav_clock"].get(now_mono=101.7)
            self.assertIsNotNone(nc)

    def test_trace_identities(self):
        with tempfile.TemporaryDirectory() as d:
            _make_bundle(d)
            r = drv.ReplayDriver(d); r.run()
            idents = [i for _t, _s, i in r.trace]
            self.assertIn("TIM-TP", idents)
            self.assertIn("NAV-CLOCK", idents)
            self.assertIn("ticc:chA", idents)
            self.assertIn("ticc:chB", idents)
            # the dummy (invalid) SSR frame routes through RTCMReader and tags
            # gracefully — stage 2a applies SSR/eph (no longer deferred)
            self.assertIn("rtcm?", idents)

    def test_ticc_events_accumulated(self):
        with tempfile.TemporaryDirectory() as d:
            _make_bundle(d)
            r = drv.ReplayDriver(d); r.run()
            evs = r.stores["ticc_events"]
            self.assertEqual([e.channel for e in evs], ["chA", "chB"])
            self.assertEqual(evs[0].ref_sec, 0)
            self.assertEqual(evs[0].recv_mono, 100.5)   # captured stamp, not live

    def test_qerr_invalid_filtered_like_live(self):
        # Charlie #236 F1: a captured qErrInvalid=1 sample must be filtered out
        # of the deque get() reads, EXACTLY as serial_reader→update() does live
        # — not appended just because replay omitted the flag.
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.record("ubx", _ubx_tim_tp(900, 1234, qerr_invalid=1),
                     recv_mono=100.0)
            b.close()
            r = drv.ReplayDriver(d); r.run()
            # invalid sample present in the trace (it was dispatched) ...
            self.assertIn("TIM-TP", [i for _t, _s, i in r.trace])
            # ... but NOT in the deque get() reads (filtered, like live)
            self.assertIsNone(
                r.stores["qerr"].get(max_age_s=5.0, now_mono=100.5)[0])

    def test_qerr_valid_admitted(self):
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.record("ubx", _ubx_tim_tp(900, 1234, qerr_invalid=0),
                     recv_mono=100.0)
            b.close()
            r = drv.ReplayDriver(d); r.run()
            self.assertIsNotNone(
                r.stores["qerr"].get(max_age_s=5.0, now_mono=100.5)[0])

    def test_counts(self):
        with tempfile.TemporaryDirectory() as d:
            _make_bundle(d)
            r = drv.ReplayDriver(d); r.run()
            self.assertEqual(r.counts["ubx"], 3)
            self.assertEqual(r.counts["ticc"], 2)
            self.assertEqual(r.counts["ssr"], 1)

    def test_ssr_store_exists_and_frame_routed_gracefully(self):
        # stage 2a: SSR is applied via the shared router; an invalid frame is
        # tagged "rtcm?" without raising, and the store set carries ssr/beph.
        with tempfile.TemporaryDirectory() as d:
            _make_bundle(d)
            r = drv.ReplayDriver(d); r.run()
            self.assertEqual(r.counts["ssr"], 1)
            self.assertIn("ssr", r.stores)
            self.assertIn("beph", r.stores)


class TestRtcmApplication(unittest.TestCase):
    def test_eph_frame_routes_to_beph(self):
        # real RTCM bytes are impractical to synthesize, so mock the parser and
        # confirm the driver wires a parsed eph identity through the SHARED
        # router into the beph store (1019 = GPS broadcast eph).
        import types
        from unittest import mock
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.record("eph", b"\xd3rawframe", recv_mono=200.0)
            b.close()
            fake_msg = types.SimpleNamespace(identity="1019")
            captured = {}

            def _fake_route(identity, msg_view, beph, ssr, label, **kw):
                captured["identity"] = identity
                captured["label"] = label
                captured["bias_only"] = kw.get("bias_only")
                return "eph", 5

            with mock.patch("pyrtcm.RTCMReader.parse", return_value=fake_msg), \
                 mock.patch("realtime_ppp.route_rtcm_message", _fake_route):
                r = drv.ReplayDriver(d); r.run()
            self.assertEqual(captured["identity"], "1019")
            self.assertEqual(captured["label"], "eph")
            self.assertFalse(captured["bias_only"])
            self.assertIn("eph:1019", [i for _t, _s, i in r.trace])

    def test_ssr_bias_routes_bias_only(self):
        import types
        from unittest import mock
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.record("ssr_bias", b"\xd3rawframe", recv_mono=200.0)
            b.close()
            fake_msg = types.SimpleNamespace(identity="1265")
            captured = {}

            def _fake_route(identity, msg_view, beph, ssr, label, **kw):
                captured["bias_only"] = kw.get("bias_only")
                return "bias_gapfill", None

            with mock.patch("pyrtcm.RTCMReader.parse", return_value=fake_msg), \
                 mock.patch("realtime_ppp.route_rtcm_message", _fake_route):
                r = drv.ReplayDriver(d); r.run()
            self.assertTrue(captured["bias_only"])   # secondary mount = bias-only


class TestRunConfig(unittest.TestCase):
    def test_reads_bias_skip_from_manifest(self):
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.write_manifest(host="h", started_iso="2026-01-01T00:00:00+00:00",
                             conventions={"skip_phase_biases": True,
                                          "skip_code_biases": False})
            b.close()
            cfg = drv.read_run_config(d)
            self.assertTrue(cfg["skip_phase_biases"])
            self.assertFalse(cfg["skip_code_biases"])
            self.assertFalse(cfg["skip_biases"])        # absent → default False

    def test_missing_manifest_defaults_all_false(self):
        with tempfile.TemporaryDirectory() as d:
            RawCaptureBundle(d).close()                 # no manifest written
            cfg = drv.read_run_config(d)
            self.assertEqual(set(cfg.values()), {False})

    def test_driver_passes_captured_bias_skip_to_router(self):
        # #237: a capture made with --no-ssr-phase-bias dropped phase biases
        # LIVE, so replay must drop them too — the flag flows manifest → router.
        import types
        from unittest import mock
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.write_manifest(host="h", started_iso="2026-01-01T00:00:00+00:00",
                             conventions={"skip_phase_biases": True})
            b.record("ssr", b"\xd3rawframe", recv_mono=200.0)
            b.close()
            fake_msg = types.SimpleNamespace(identity="1265")
            seen = {}

            def _fake_route(identity, msg_view, beph, ssr, label, **kw):
                seen.update(kw)
                return "skip_phase_bias", None

            with mock.patch("pyrtcm.RTCMReader.parse", return_value=fake_msg), \
                 mock.patch("realtime_ppp.route_rtcm_message", _fake_route):
                drv.ReplayDriver(d).run()
            self.assertTrue(seen.get("skip_phase_biases"))
            self.assertFalse(seen.get("skip_code_biases"))

    def test_run_config_override_wins(self):
        with tempfile.TemporaryDirectory() as d:
            RawCaptureBundle(d).close()
            r = drv.ReplayDriver(d, run_config={"skip_biases": True,
                                                "skip_code_biases": False,
                                                "skip_phase_biases": False})
            self.assertTrue(r.run_config["skip_biases"])


class TestEmptyBundle(unittest.TestCase):
    def test_empty_bundle_is_clean(self):
        with tempfile.TemporaryDirectory() as d:
            RawCaptureBundle(d).close()
            r = drv.ReplayDriver(d)
            self.assertEqual(r.run(), [])
            self.assertIsNone(r.clock.now_mono)


if __name__ == "__main__":
    unittest.main()

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


class TestFilterConfigFaithfulness(unittest.TestCase):
    """I-191033: build_filter_thread must construct the AntPosEstThread with the
    SAME null-constraining config + NAV2 store the live engine ran, else the
    replay wanders the (pos,ZTD,clk) null and diverges ~8 m from captured-live."""

    def tearDown(self):
        # Q_POS_CONVERGED is a PPPFilter CLASS attribute build_filter_thread
        # mutates; restore the engine default so a Q-tuned bundle in one test
        # can't leak into another (the very leak the always-set guard prevents).
        from solve_ppp import PPPFilter
        PPPFilter.Q_POS_CONVERGED = 1e-4

    def _thread_from_filter_config(self, filter_config):
        """build_filter_thread over a minimal bundle carrying filter_config."""
        rawx = _synthetic_rawx()
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.write_manifest(host="h", started_iso="t",
                             conventions={"receiver": "f9t", "systems": ["gps"]},
                             filter_config=filter_config)
            b.record("ubx", b"R", recv_mono=1.0)
            b.close()
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                rd = drv.ReplayDriver(d, decode_obs=True)
                return prf.build_filter_thread(rd, _TRUTH, systems=["gps"])

    def test_captured_q_pos_converged_lands_on_class_attr(self):
        """A bundle whose [filter_config] tuned --q-pos-converged replays with the
        same converged-position stiffness on PPPFilter.Q_POS_CONVERGED (the class
        attr the engine sets globally; Main review #258)."""
        from solve_ppp import PPPFilter
        self._thread_from_filter_config({"q_pos_converged": 1e-9})
        self.assertEqual(PPPFilter.Q_POS_CONVERGED, 1e-9)

    def test_absent_q_pos_converged_resets_to_default_no_leak(self):
        """A bundle with no q_pos_converged resets the class attr to 1e-4 — so a
        prior Q-tuned replay's override can't leak into this one in a shared
        process (the always-set guard)."""
        from solve_ppp import PPPFilter
        PPPFilter.Q_POS_CONVERGED = 1e-9              # simulate a prior replay's leak
        self._thread_from_filter_config({"clock_model": "wno"})  # no q_pos_converged
        self.assertEqual(PPPFilter.Q_POS_CONVERGED, 1e-4)

    def test_build_thread_wires_nav2_store_anchor_and_virtual_clock(self):
        rawx = _synthetic_rawx()
        with tempfile.TemporaryDirectory() as d:
            _bundle(d)
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                rd = drv.ReplayDriver(d, decode_obs=True)
                thread = prf.build_filter_thread(rd, _TRUTH, systems=["gps"])
        # the driver's NAV2 store is wired into the thread (anchor can fire)
        self.assertIs(thread._nav2_store, rd.stores["nav2"])
        self.assertTrue(thread._nav2_anchor_enabled)          # spec default True
        # NAV2 freshness ages against the virtual clock, not wall time
        rd.clock.now_mono = 1234.5
        self.assertEqual(thread._now_mono(), 1234.5)

    def test_captured_filter_config_overrides_defaults(self):
        # a bundle whose [filter_config] turns the anchor OFF replays with it off
        rawx = _synthetic_rawx()
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.write_manifest(host="h", started_iso="2026-01-01T00:00:00+00:00",
                             conventions={"receiver": "f9t", "systems": ["gps"]},
                             filter_config={"nav2_soft_anchor": False,
                                            "clock_model": "wno"})
            b.record("ubx", b"R", recv_mono=1.0)
            b.close()
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                rd = drv.ReplayDriver(d, decode_obs=True)
                thread = prf.build_filter_thread(rd, _TRUTH, systems=["gps"])
        self.assertFalse(thread._nav2_anchor_enabled)         # captured override
        self.assertEqual(thread._filt.clock_model, "wno")

    def test_pcv_parser_wired_so_pcv_is_not_silently_inert(self):
        # Charlie #253 finding 1: passing receiver_antenna WITHOUT antex_parser
        # left _pcv_enabled False (silent inert).  build_filter_thread must build
        # the parser so the replay applies the SAME PCV the live filter did.
        rawx = _synthetic_rawx()
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.write_manifest(host="h", started_iso="t",
                             conventions={"receiver": "f9t", "systems": ["gps"]},
                             filter_config={"pcv": True,
                                            "receiver_antenna": "SFESPK6618H     NONE"})
            b.record("ubx", b"R", recv_mono=1.0)
            b.close()
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                rd = drv.ReplayDriver(d, decode_obs=True)
                thread = prf.build_filter_thread(rd, _TRUTH, systems=["gps"])
        self.assertIsNotNone(thread._antex)            # parser wired
        self.assertTrue(thread._pcv_enabled)           # PCV actually on (not inert)

    def test_no_pcv_capture_leaves_pcv_off(self):
        rawx = _synthetic_rawx()
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.write_manifest(host="h", started_iso="t",
                             conventions={"receiver": "f9t", "systems": ["gps"]},
                             filter_config={"pcv": False,
                                            "receiver_antenna": "SFESPK6618H     NONE"})
            b.record("ubx", b"R", recv_mono=1.0)
            b.close()
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                rd = drv.ReplayDriver(d, decode_obs=True)
                thread = prf.build_filter_thread(rd, _TRUTH, systems=["gps"])
        self.assertFalse(thread._pcv_enabled)          # --no-pcv run → off

    def test_filter_config_roundtrip_drops_none_restores_default(self):
        from types import SimpleNamespace
        args = SimpleNamespace(solid_tide=False, clock_model="wno",
                               ztd_tie_sigma=None, ar_elev_mask=30.0)
        fc = drv.filter_config_from_args(args)
        self.assertNotIn("ztd_tie_sigma", fc)     # None dropped (no TOML null)
        self.assertEqual(fc["solid_tide"], False)
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.write_manifest(host="h", started_iso="t", filter_config=fc)
            b.close()
            read = drv.read_filter_config(d)
        self.assertEqual(read["solid_tide"], False)
        self.assertEqual(read["clock_model"], "wno")
        self.assertEqual(read["ar_elev_mask"], 30.0)
        self.assertIsNone(read["ztd_tie_sigma"])  # restored to spec default None
        self.assertTrue(read["nav2_soft_anchor"])  # unset key → spec default


class TestSeedStateInheritance(unittest.TestCase):
    """I-204115: the live AntPos inherits the Phase-1 bootstrap filter, so a
    faithful replay must restore that captured initial state instead of cold-
    starting — else it settles at a different (pos,ZTD,clk) null coordinate
    (~1.6 m stable residual)."""

    def _converged_filter(self):
        import numpy as np
        from solve_ppp import PPPFilter
        f = PPPFilter(clock_model="random_walk")
        f.initialize(_TRUTH, -680000.0, systems={"gps"})
        f.sv_to_idx = {"G01": 7}
        f.x = np.append(f.x, [1.5])        # one float ambiguity
        f.P = np.pad(f.P, ((0, 1), (0, 1)))
        return f

    def test_dump_restore_roundtrip_through_json(self):
        import json, numpy as np
        from solve_ppp import PPPFilter
        f = self._converged_filter()
        st = json.loads(json.dumps(prf.dump_ppp_filter_state(f)))
        g = PPPFilter(clock_model="random_walk")
        g.initialize((0, 0, 0), 0.0)
        prf.restore_ppp_filter_state(g, st)
        self.assertTrue(np.allclose(g.x, f.x))
        self.assertEqual(g.x.shape, f.x.shape)     # ambiguity-extended length
        self.assertTrue(np.allclose(g.P, f.P))
        self.assertEqual(g.sv_to_idx, {"G01": 7})
        self.assertTrue(g.initialized)             # clock in x → no self-seed

    def test_nondefault_ztd_q_coef_survives_dump_restore(self):
        """--q-ztd-antpos reaches AntPos via bootstrap inheritance, not the fresh
        _shim, so the seed snapshot must carry _ztd_q_coef — else a Q-tuned bundle
        reverts to the 1.29e-3 default ZTD stiffness on replay (I-215452, Main
        review #258).  The default roundtrip test above uses the default coef, so
        it can't catch a regression — this pins the NON-default path."""
        import json
        from solve_ppp import PPPFilter
        f = self._converged_filter()
        f._ztd_q_coef = 1e-4                        # RTKLIB-recipe stiffness
        st = json.loads(json.dumps(prf.dump_ppp_filter_state(f)))
        self.assertEqual(st["ztd_q_coef"], 1e-4)   # captured, not defaulted
        g = PPPFilter(clock_model="random_walk")
        g.initialize((0, 0, 0), 0.0)               # fresh filter has 1.29e-3
        prf.restore_ppp_filter_state(g, st)
        self.assertEqual(g._ztd_q_coef, 1e-4)      # restored, not reverted

    def test_old_snapshot_without_ztd_q_coef_restores_default(self):
        """A pre-field snapshot (no ztd_q_coef key) restores to the engine's
        1.29e-3 default, so old bundles replay exactly as before."""
        from solve_ppp import PPPFilter
        st = prf.dump_ppp_filter_state(self._converged_filter())
        del st["ztd_q_coef"]                        # simulate an old snapshot
        g = PPPFilter(clock_model="random_walk")
        g.initialize((0, 0, 0), 0.0)
        g._ztd_q_coef = 42.0                        # prove restore overwrites it
        prf.restore_ppp_filter_state(g, st)
        self.assertAlmostEqual(g._ztd_q_coef, 1.29e-3)

    def test_build_thread_restores_captured_seed_state(self):
        import numpy as np
        rawx = _synthetic_rawx()
        seed = prf.dump_ppp_filter_state(self._converged_filter())
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.write_manifest(host="h", started_iso="t",
                             conventions={"receiver": "f9t", "systems": ["gps"]})
            b.write_engine_json("ape_init_state.json", seed)
            b.record("ubx", b"R", recv_mono=1.0)
            b.close()
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                rd = drv.ReplayDriver(d, decode_obs=True)
                thread = prf.build_filter_thread(rd, _TRUTH, systems=["gps"])
        # filter starts from the captured converged state, NOT a cold seed
        self.assertEqual(len(thread._filt.x), len(seed["x"]))
        self.assertTrue(np.allclose(thread._filt.x, seed["x"]))
        self.assertEqual(thread._filt.sv_to_idx, {"G01": 7})
        self.assertTrue(thread._filt.initialized)

    def test_no_seed_state_falls_back_to_cold_self_seed(self):
        rawx = _synthetic_rawx()
        with tempfile.TemporaryDirectory() as d:
            _bundle(d)                     # no ape_init_state.json
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                rd = drv.ReplayDriver(d, decode_obs=True)
                thread = prf.build_filter_thread(rd, _TRUTH, systems=["gps"])
        # falls back to the I-175208 clock self-seed (initialized=False)
        self.assertFalse(thread._filt.initialized)


class TestNav2AnchorCaptureReplay(unittest.TestCase):
    """I-215452: the live anchor's per-epoch firing decision is captured to
    Group-B and applied VERBATIM in replay (deterministic), instead of
    re-deriving via get_opinion (the freshness/timing mismatch that injected
    ~600mm in dynamic windows)."""

    def _thread(self, anchor_decisions, nav2_store, raw_bundle):
        import peppar_fix_engine as eng
        import time as _t
        t = eng.AntPosEstThread.__new__(eng.AntPosEstThread)
        t._nav2_anchor_enabled = True
        t._nav2_anchor_max_hacc_m = 3.0
        t._nav2_floor_enabled = False   # floor off: _apply_nav2_anchor bails early
        t._pin_position = False
        t._nav2_store = nav2_store
        t._anchor_decisions = anchor_decisions
        t._raw_bundle = raw_bundle
        t._now_mono = _t.monotonic
        return t

    def _filt(self):
        calls = []
        f = SimpleNamespace(apply_nav2_anchor=lambda e, h, v: calls.append((tuple(e), h, v)))
        return f, calls

    def _gt(self):
        from datetime import datetime, timezone
        return datetime(2026, 6, 28, 3, 55, 22, 994000, tzinfo=timezone.utc)

    def test_replay_applies_captured_decision_verbatim(self):
        gt = self._gt()
        dec = {gt.isoformat(): ((1.0, 2.0, 3.0), 0.5, 1.1)}
        t = self._thread(dec, nav2_store=object(), raw_bundle=None)
        f, calls = self._filt()
        t._apply_nav2_anchor(gt, f)
        self.assertEqual(calls, [((1.0, 2.0, 3.0), 0.5, 1.1)])   # verbatim, no get_opinion

    def test_replay_no_decision_for_epoch_means_no_anchor(self):
        gt = self._gt()
        t = self._thread({}, nav2_store=object(), raw_bundle=None)  # empty map
        f, calls = self._filt()
        t._apply_nav2_anchor(gt, f)
        self.assertEqual(calls, [])                  # epoch absent → did not fire live

    def test_live_derives_and_captures_decision(self):
        gt = self._gt()
        store = SimpleNamespace(get_opinion=lambda **k: {
            "fix_type": 3, "h_acc_m": 1.2, "v_acc_m": 2.0,
            "ecef": [10.0, 20.0, 30.0]})
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            t = self._thread(None, nav2_store=store, raw_bundle=b)  # live mode
            f, calls = self._filt()
            t._apply_nav2_anchor(gt, f)
            b.close()
            self.assertEqual(len(calls), 1)
            self.assertEqual(calls[0][0], (10.0, 20.0, 30.0))
            with open(os.path.join(d, "engine", "anchor_decisions.log")) as fh:
                line = fh.read().strip()
        self.assertTrue(line.startswith("[NAV2_ANCHOR]"))
        self.assertIn("h_acc=1.2000", line)

    def test_build_thread_wires_anchor_decisions_when_present(self):
        from peppar_fix.ppp_state_line import format_anchor_line
        rawx = _synthetic_rawx()
        gt = self._gt()
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.write_manifest(host="h", started_iso="t",
                             conventions={"receiver": "f9t", "systems": ["gps"]})
            b.engine_log("anchor_decisions.log",
                         format_anchor_line(gt, (1.0, 2.0, 3.0), 0.5, None))
            b.record("ubx", b"R", recv_mono=1.0)
            b.close()
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                rd = drv.ReplayDriver(d, decode_obs=True)
                thread = prf.build_filter_thread(rd, _TRUTH, systems=["gps"])
        self.assertIsNotNone(thread._anchor_decisions)
        self.assertIn(gt.isoformat(), thread._anchor_decisions)

    def test_build_thread_none_when_no_anchor_log(self):
        rawx = _synthetic_rawx()
        with tempfile.TemporaryDirectory() as d:
            _bundle(d)                       # no anchor_decisions.log
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                rd = drv.ReplayDriver(d, decode_obs=True)
                thread = prf.build_filter_thread(rd, _TRUTH, systems=["gps"])
        self.assertIsNone(thread._anchor_decisions)   # old bundle → get_opinion path


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


class TestEmitPppStateToBundle(unittest.TestCase):
    """AntPosEstThread._emit_ppp_state Group-B routing: when a raw-capture
    bundle is attached, the per-epoch [PPP_STATE] (+ one-shot [ZTD_APRIORI])
    lands in the bundle's engine/ dir — independent of the noisy --ppp-state-log
    main-log gate (a capture must be self-sufficient).  Driven on a bare thread
    instance (no heavy filter/eph setup needed to exercise the sink routing)."""

    def _bare_thread(self, bundle, ppp_state_log):
        import numpy as np
        import peppar_fix_engine as eng
        t = eng.AntPosEstThread.__new__(eng.AntPosEstThread)
        t._args = SimpleNamespace(ppp_state_log=ppp_state_log)
        t._raw_bundle = bundle
        t._n_epochs = 7
        t._ztd_apriori_logged = False
        filt = SimpleNamespace(
            x=np.zeros(eng.N_BASE), P=np.eye(eng.N_BASE) * 0.04)
        return t, filt

    def _gps_time(self):
        from datetime import datetime, timezone
        return datetime(2026, 1, 1, tzinfo=timezone.utc)

    def test_bundle_gets_ppp_state_even_without_main_log(self):
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            t, filt = self._bare_thread(b, ppp_state_log=False)
            t._emit_ppp_state(self._gps_time(), 8, filt)
            t._n_epochs = 8
            t._emit_ppp_state(self._gps_time(), 9, filt)
            b.close()
            with open(os.path.join(d, "engine", "ppp_state.log")) as f:
                lines = f.read().splitlines()
        self.assertEqual(lines[0], "[ZTD_APRIORI] m=2.3000")  # one-shot, first
        self.assertEqual(sum(1 for ln in lines if ln.startswith("[ZTD_APRIORI]")), 1)
        ps = [ln for ln in lines if ln.startswith("[PPP_STATE]")]
        self.assertEqual(len(ps), 2)                          # one per epoch
        self.assertIn("n=8", ps[0])

    def test_no_bundle_no_main_log_is_noop(self):
        t, filt = self._bare_thread(None, ppp_state_log=False)
        # must not raise and must not mark the apriori as logged (nothing emitted)
        t._emit_ppp_state(self._gps_time(), 8, filt)
        self.assertFalse(t._ztd_apriori_logged)

    def test_bundle_write_failure_does_not_raise(self):
        # a closed/failing sink must never take down the filter thread
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            t, filt = self._bare_thread(b, ppp_state_log=False)
            b.close()                       # engine handles now closed
            # force a write attempt to a closed handle
            b._engine_files["ppp_state.log"] = open(os.devnull, "ab")
            b._engine_files["ppp_state.log"].close()
            t._emit_ppp_state(self._gps_time(), 8, filt)   # swallowed, no raise


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


class TestSsrSourceSwapSeam(unittest.TestCase):
    """The seam the per-stream knob exists to make safe (Charlie #247): the
    ssr-source loader + swap_streams + run_pos_replay together — loader supplies
    the SSR, captured SSR is swapped out, captured eph is KEPT (else no orbits)."""

    def test_loader_supplies_ssr_and_eph_is_kept(self):
        import json
        from types import SimpleNamespace
        rawx = _synthetic_rawx()
        recs = {"epoch_s": 100.0,
                "records": [{"prn": "G01", "code_bias": {"C1C": -1.0}}]}
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.write_manifest(host="h", started_iso="2026-01-01T00:00:00+00:00",
                             conventions={"receiver": "f9t", "systems": ["gps"]})
            b.record("ssr", b"\xd3\x00\x05ssr", recv_mono=98.0)
            b.record("eph", b"\xd3\x00\x05eph", recv_mono=99.0)
            b.record("ubx", b"RAWX-0", recv_mono=100.0)
            b.close()
            rec_path = os.path.join(d, "records.json")
            with open(rec_path, "w") as f:
                json.dump(recs, f)
            loader = prf.make_ssr_records_loader(rec_path)
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx), \
                 mock.patch("pyrtcm.RTCMReader.parse",
                            return_value=SimpleNamespace(identity="1019")), \
                 mock.patch("realtime_ppp.route_rtcm_message",
                            return_value=("eph", 5)) as route:
                res = prf.run_pos_replay(
                    d, _TRUTH, corrections_loader=loader,
                    swap_streams={"ssr", "ssr_bias"})
            self.assertTrue(res["product_swapped"])
            ssr = res["driver"].stores["ssr"]
            # the loader's SSR is in place (captured SSR was swapped out, so this
            # bias can only come from the loader)
            self.assertAlmostEqual(ssr.get_code_bias("G01", "C1C"), -1.0)
            idents = [i for _t, _s, i in res["driver"].trace]
            self.assertIn("ssr:swapped-out", idents)     # captured SSR swapped
            self.assertIn("eph:1019", idents)            # captured eph KEPT
            route.assert_called_once()                   # only eph routed


import numpy as _np                            # noqa: E402

_DEFAULT_SP3_POS = _np.array([1.0, 2.0, 3.0])


class _StubSP3:
    # The REAL SP3.sat_position contract (solve_pseudorange.py:133): returns the
    # (pos_array, clk) 2-tuple — or (None, None) outside its window — NOT (x,y,z).
    # Deriving the stub from the real return is the #239 lesson (Charlie #248).
    def __init__(self, pos=_DEFAULT_SP3_POS, clk=2.0e-7):
        self.pos = pos
        self.clk = clk

    def sat_position(self, sv, t):
        return self.pos, self.clk


class _StubClk:
    def __init__(self, clk=1.5e-6):
        self.clk = clk

    def sat_clock(self, prn, t):
        return self.clk                        # seconds, or None


class TestPreciseCorrections(unittest.TestCase):
    """The precise-orbit corrections OBJECT (Charlie #245-1/#248): SP3 orbits +
    CLK clocks via the engine's sat_position→(pos,clk) interface — SP3 already
    returns that 2-tuple, the precise CLK substitutes the clock."""

    def test_bridges_sp3_position_and_clk_clock(self):
        pc = prf.PreciseCorrections(_StubSP3(), _StubClk(1.5e-6))
        pos, clk = pc.sat_position("G01", None)
        self.assertTrue(_np.allclose(pos, [1.0, 2.0, 3.0]))   # SP3 position
        self.assertAlmostEqual(clk, 1.5e-6)    # CLKFile clock substituted for SP3's
        self.assertAlmostEqual(pc.sat_clock("G01", None), 1.5e-6)

    def test_outside_sp3_window_returns_none(self):
        pc = prf.PreciseCorrections(_StubSP3(pos=None, clk=None), _StubClk())
        self.assertEqual(pc.sat_position("G01", None), (None, None))

    def test_clkfile_none_clock_skips_sv(self):
        # CLKFile.sat_clock returns None (PRN absent / out of range) → must
        # return (None,None), NOT (pos,None) which the engine's pos-only guard
        # would let propagate a None clock (Charlie #248 bug 2).
        pc = prf.PreciseCorrections(_StubSP3(), _StubClk(clk=None))
        self.assertEqual(pc.sat_position("G01", None), (None, None))

    def test_no_clk_file_uses_sp3_clock(self):
        # no CLK file → SP3's OWN clock, not 0.0 (0.0 = ~100s-km error, #248 bug 3)
        pc = prf.PreciseCorrections(_StubSP3(clk=7.0e-7), clk_file=None)
        _pos, clk = pc.sat_position("G01", None)
        self.assertAlmostEqual(clk, 7.0e-7)
        self.assertAlmostEqual(pc.sat_clock("G01", None), 7.0e-7)  # not 0.0


class TestCorrectionsOverride(unittest.TestCase):
    def test_build_filter_thread_leaves_clock_unseeded(self):
        # runner bootstrap gap (I-175208): the fresh filter must be left
        # initialized=False so PPPFilter.update self-seeds the clock from the
        # first epoch — else clock=0 → every IF residual is an outlier →
        # n_used=0, never converges.
        with tempfile.TemporaryDirectory() as d:
            _bundle(d, n_rawx=0)
            rd = drv.ReplayDriver(d, decode_obs=True)
            thread = prf.build_filter_thread(rd, _TRUTH, systems=["gps"])
            self.assertFalse(thread._filt.initialized)

    def test_build_filter_thread_uses_override(self):
        rawx = _synthetic_rawx()
        sentinel = prf.PreciseCorrections(_StubSP3(), _StubClk())
        with tempfile.TemporaryDirectory() as d:
            _bundle(d, n_rawx=0)
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                rd = drv.ReplayDriver(d, decode_obs=True)
                thread = prf.build_filter_thread(rd, _TRUTH, systems=["gps"],
                                                 corrections=sentinel)
        self.assertIs(thread._corrections, sentinel)

    def test_run_pos_replay_precise_override_swaps_captured(self):
        rawx = _synthetic_rawx()
        override = prf.PreciseCorrections(_StubSP3(), _StubClk())
        with tempfile.TemporaryDirectory() as d:
            b = RawCaptureBundle(d)
            b.write_manifest(host="h", started_iso="2026-01-01T00:00:00+00:00",
                             conventions={"receiver": "f9t", "systems": ["gps"]})
            b.record("eph", b"\xd3\x00\x05eph", recv_mono=99.0)
            b.record("ubx", b"RAWX-0", recv_mono=100.0)
            b.close()
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                res = prf.run_pos_replay(d, _TRUTH,
                                         corrections_override=override)
        self.assertTrue(res["product_swapped"])
        self.assertIs(res["thread"]._corrections, override)
        # captured eph swapped out by default (override supplies orbits)
        self.assertIn("eph:swapped-out", [i for _t, _s, i in res["driver"].trace])


if __name__ == "__main__":
    unittest.main()

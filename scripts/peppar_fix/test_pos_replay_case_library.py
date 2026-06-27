"""pos_replay case-library batch runner: iterate cases through the real filter,
with a per-bundle guard so one bad case doesn't sink the sweep (Charlie #244-2)."""
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
from peppar_fix import pos_replay_case_library as cl        # noqa: E402
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


def _good_bundle(d, receiver="f9t"):
    b = RawCaptureBundle(d)
    conv = {"systems": ["gps"]}
    if receiver is not None:
        conv["receiver"] = receiver
    b.write_manifest(host="h", started_iso="2026-01-01T00:00:00+00:00",
                     conventions=conv)
    b.record("ubx", b"RAWX-0", recv_mono=100.0)
    b.close()


class TestCaseLibrary(unittest.TestCase):
    def test_runs_all_cases(self):
        rawx = _synthetic_rawx()
        with tempfile.TemporaryDirectory() as a, tempfile.TemporaryDirectory() as b:
            _good_bundle(a)
            _good_bundle(b)
            cases = [{"name": "calm", "bundle_dir": a, "known_ecef": _TRUTH},
                     {"name": "active", "bundle_dir": b, "known_ecef": _TRUTH}]
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                summary = cl.run_case_library(cases)
        self.assertEqual(summary["n_ok"], 2)
        self.assertEqual(summary["n_failed"], 0)
        self.assertEqual([r["name"] for r in summary["results"]],
                         ["calm", "active"])

    def test_one_bad_case_does_not_sink_the_sweep(self):
        # the bad case names no receiver → replay_sig_config raises inside
        # run_pos_replay; the guard records it and the good case still runs.
        rawx = _synthetic_rawx()
        with tempfile.TemporaryDirectory() as good, \
             tempfile.TemporaryDirectory() as bad:
            _good_bundle(good)
            _good_bundle(bad, receiver=None)        # no receiver → raises
            cases = [{"name": "bad", "bundle_dir": bad, "known_ecef": _TRUTH},
                     {"name": "good", "bundle_dir": good, "known_ecef": _TRUTH}]
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                summary = cl.run_case_library(cases)
        self.assertEqual(summary["n_ok"], 1)
        self.assertEqual(summary["n_failed"], 1)
        bad_r = next(r for r in summary["results"] if r["name"] == "bad")
        self.assertEqual(bad_r["status"], "failed")
        self.assertIn("ValueError", bad_r["error"])
        good_r = next(r for r in summary["results"] if r["name"] == "good")
        self.assertEqual(good_r["status"], "ok")    # sweep continued

    def test_epoch_error_is_caught_at_batch_layer(self):
        # an exception raised from inside the filter drive (epoch_sink) must be
        # caught by the batch guard, not abort the sweep — _process_epoch stays
        # unwrapped; the guard is at the orchestration layer.
        rawx = _synthetic_rawx()
        with tempfile.TemporaryDirectory() as d:
            _good_bundle(d)
            cases = [{"name": "boom", "bundle_dir": d, "known_ecef": _TRUTH}]
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx), \
                 mock.patch("peppar_fix.pos_replay_filter.build_filter_thread",
                            side_effect=RuntimeError("filter boom")):
                summary = cl.run_case_library(cases)
        self.assertEqual(summary["n_failed"], 1)
        self.assertIn("RuntimeError", summary["results"][0]["error"])

    def test_truth_bearing_case_inconclusive(self):
        # Charlie #246: exercise the SCORING path (truth → verdict).  Synthetic
        # RAWX with no eph → n_used<4 → 0 [PPP_STATE] → n=0 < window →
        # inconclusive.  Pins that compare_position keeps returning n/window
        # (a future return-dict change can't silently break the library).
        from peppar_fix.pos_replay_compare import StaticTruth
        rawx = _synthetic_rawx()
        with tempfile.TemporaryDirectory() as d:
            _good_bundle(d)
            cases = [{"name": "calm", "bundle_dir": d, "known_ecef": _TRUTH,
                      "truth": StaticTruth(_TRUTH, sigma_m=0.012)}]
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                summary = cl.run_case_library(cases)
        r0 = summary["results"][0]
        self.assertEqual(r0["status"], "ok")
        self.assertIn("position_diverged", r0)          # scored
        self.assertFalse(r0["position_diverged"])
        self.assertTrue(r0["position_inconclusive"])    # n=0 < window
        self.assertEqual(summary["n_diverged"], 0)
        self.assertIn("inconclusive", cl.format_summary(summary))

    def test_diverged_case_counted_and_summarized(self):
        # mock run_pos_replay to return a FIRED verdict → exercises
        # position_diverged, n_diverged, and the DIVERGED summary line.
        fired = {
            "ppp_state_lines": ["[PPP_STATE] ..."] * 200,
            "n_epochs_decoded": 200,
            "product_swapped": True,
            "position": {"verdict": {"fired": True}, "n": 200, "window": 120},
        }
        with mock.patch("peppar_fix.pos_replay_case_library.run_pos_replay",
                        return_value=fired):
            summary = cl.run_case_library(
                [{"name": "active", "bundle_dir": "x", "known_ecef": _TRUTH,
                  "truth": object()}])
        r0 = summary["results"][0]
        self.assertTrue(r0["position_diverged"])
        self.assertFalse(r0["position_inconclusive"])   # n=200 >= window
        self.assertEqual(summary["n_diverged"], 1)
        out = cl.format_summary(summary)
        self.assertIn("DIVERGED", out)
        self.assertIn("[swapped]", out)                 # product_swapped True

    def test_swap_streams_plumbed_through_run_case(self):
        # Charlie #247: an ssr-source loader is unusable through the orchestrator
        # unless swap_streams reaches run_pos_replay (else all-three swap drops
        # eph → empty).  Assert run_case forwards the case's swap_streams.
        with mock.patch("peppar_fix.pos_replay_case_library.run_pos_replay",
                        return_value={"ppp_state_lines": [], "n_epochs_decoded": 0,
                                      "product_swapped": True}) as rp:
            cl.run_case({"name": "ssr-src", "bundle_dir": "b",
                         "known_ecef": _TRUTH,
                         "corrections_loader": lambda s: None,
                         "swap_streams": {"ssr", "ssr_bias"}})
        self.assertEqual(rp.call_args.kwargs["swap_streams"], {"ssr", "ssr_bias"})
        self.assertIsNotNone(rp.call_args.kwargs["corrections_loader"])

    def test_corrections_override_plumbed_through_run_case(self):
        # precise-orbit case must be reachable through the orchestrator too
        # (same lesson as #247) — assert the override reaches run_pos_replay.
        sentinel = object()
        with mock.patch("peppar_fix.pos_replay_case_library.run_pos_replay",
                        return_value={"ppp_state_lines": [], "n_epochs_decoded": 0,
                                      "product_swapped": True}) as rp:
            cl.run_case({"name": "precise", "bundle_dir": "b",
                         "known_ecef": _TRUTH,
                         "corrections_override": sentinel})
        self.assertIs(rp.call_args.kwargs["corrections_override"], sentinel)

    def test_format_summary(self):
        summary = {"n_ok": 1, "n_failed": 1, "n_diverged": 0, "results": [
            {"name": "a", "status": "ok", "n_epochs": 5, "n_ppp_state": 0,
             "product_swapped": False},
            {"name": "b", "status": "failed", "error": "ValueError: nope"}]}
        out = cl.format_summary(summary)
        self.assertIn("1 ok, 1 failed", out)
        self.assertIn("✓ a", out)
        self.assertIn("✗ b: ValueError: nope", out)


if __name__ == "__main__":
    unittest.main()

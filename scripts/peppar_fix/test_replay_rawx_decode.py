"""pos_replay RAWX decode (runner): ReplayDriver(decode_obs=True) decodes the
captured RAWX into a (gps_time, observations, obs_counts) timeline via the
engine's own rawx_decode + rawx_to_observations with the manifest sig config."""
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
from peppar_fix.raw_capture import RawCaptureBundle         # noqa: E402
from peppar_fix.receiver import get_driver                  # noqa: E402


def _synthetic_rawx_for_f9t():
    # build a one-SV dual-freq RAWX consistent with the REAL f9t signal map
    driver = get_driver("f9t")
    rev = {name: (g, s) for (g, s), name in driver.signal_names.items()}
    _, f1, f2, _ = (getattr(driver, "if_pairs", None) or r.IF_PAIRS)[0]
    (g1, s1), (g2, s2) = rev[f1], rev[f2]
    return SimpleNamespace(
        rcvTow=100.0, week=2300, leapS=18, numMeas=2,
        gnssId=[g1, g2], sigId=[s1, s2], svId=[1, 1],
        prMes=[22_000_000.0, 22_000_000.0],
        cpMes=[115_000_000.0, 90_000_000.0],
        cno=[45, 44], locktime=[5000.0, 5000.0],
        prValid=[True, True], cpValid=[True, True],
        halfCyc=[True, True], clk_reset=False)


def _bundle(d, receiver="f9t", systems=("gps",)):
    b = RawCaptureBundle(d)
    conv = {"systems": list(systems)}
    if receiver is not None:
        conv["receiver"] = receiver
    b.write_manifest(host="h", started_iso="2026-01-01T00:00:00+00:00",
                     conventions=conv)
    b.record("ubx", b"RAWX-MARKER-BYTES", recv_mono=100.0)
    b.close()


class TestRawxDecodeBuildsEpochs(unittest.TestCase):
    def test_decode_obs_true_builds_timeline(self):
        rawx = _synthetic_rawx_for_f9t()
        with tempfile.TemporaryDirectory() as d:
            _bundle(d)
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                rd = drv.ReplayDriver(d, decode_obs=True)
                rd.run()
            self.assertEqual(len(rd.epochs), 1)
            gps_time, obs, counts = rd.epochs[0]
            self.assertEqual(len(obs), 1)
            self.assertEqual(obs[0]["sv"], "G01")
            self.assertEqual(obs[0]["sys"], "gps")
            self.assertGreaterEqual(gps_time.year, 2023)      # week 2300 → ~2024
            self.assertIsNotNone(gps_time.tzinfo)             # tz-aware UTC
            self.assertEqual(counts["n_raw"], 1)
            self.assertIn("RXM-RAWX", [i for _t, _s, i in rd.trace])

    def test_decode_obs_false_leaves_epochs_empty(self):
        # default: RAWX falls through to UBXReader.parse (no obs reconstruction)
        with tempfile.TemporaryDirectory() as d:
            _bundle(d)
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True):
                rd = drv.ReplayDriver(d)        # decode_obs default False
                rd.run()
            self.assertEqual(rd.epochs, [])

    def test_float_ppp_obs_survive_when_no_phase_biases(self):
        # Integration (I-175208 / Charlie #251): a phase-bias-free bundle
        # reconstructs obs that pass obs_for_position — pure-float pass-through
        # is in rawx_to_observations itself (shared live/replay), so the replay
        # obs carry ar_phase_bias_ok=True with no replay-side override.
        rawx = _synthetic_rawx_for_f9t()
        with tempfile.TemporaryDirectory() as d:
            _bundle(d)                      # no ssr stream → no phase biases
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx", return_value=rawx):
                rd = drv.ReplayDriver(d, decode_obs=True); rd.run()
            _gt, obs, _c = rd.epochs[0]
            self.assertTrue(obs)
            self.assertTrue(all(o["ar_phase_bias_ok"] for o in obs))

    def test_decode_obs_requires_receiver(self):
        with tempfile.TemporaryDirectory() as d:
            _bundle(d, receiver=None)           # no receiver in manifest
            with self.assertRaises(ValueError):
                drv.ReplayDriver(d, decode_obs=True)

    def test_decode_skip_on_garbled_rawx(self):
        with tempfile.TemporaryDirectory() as d:
            _bundle(d)
            with mock.patch("peppar_fix.rawx_decode.is_rawx", return_value=True), \
                 mock.patch("peppar_fix.rawx_decode.decode_rawx",
                            side_effect=ValueError("bad rawx")):
                rd = drv.ReplayDriver(d, decode_obs=True)
                rd.run()
            self.assertEqual(rd.epochs, [])     # decode-skip, no crash
            self.assertIn("RXM-RAWX?", [i for _t, _s, i in rd.trace])


if __name__ == "__main__":
    unittest.main()

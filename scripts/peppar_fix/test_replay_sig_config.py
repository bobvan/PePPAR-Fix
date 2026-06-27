"""Tests for the per-receiver signal config reaching pos_replay (stage 2b
runner prerequisite): the engine records `receiver` in the manifest, and the
replay reconstructs signal_names/sig_lookup/bds_l1_ref_cycles via the SAME
get_driver + build_sig_lookup the live engine uses (no drift)."""
import os
import sys
import tempfile
import unittest

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import realtime_ppp as r                                    # noqa: E402
from peppar_fix import pos_replay_driver as drv             # noqa: E402
from peppar_fix.raw_capture import RawCaptureBundle         # noqa: E402
from peppar_fix.receiver import get_driver                  # noqa: E402


class TestBuildSigLookup(unittest.TestCase):
    def test_matches_driver_if_pairs(self):
        d = get_driver("f9t")
        sl = r.build_sig_lookup(d)
        self.assertTrue(sl)
        # every entry is (gnss_id, prefix, role, a1, a2, sig_name)
        for sig, tup in sl.items():
            self.assertEqual(len(tup), 6)
            self.assertIn(tup[2], ("f1", "f2"))
            self.assertEqual(tup[5], sig)
        # both signals of each IF pair are present with matching coeffs
        for gnss_id, sig_f1, sig_f2, prefix in (
                getattr(d, "if_pairs", None) or r.IF_PAIRS):
            self.assertIn(sig_f1, sl)
            self.assertIn(sig_f2, sl)
            self.assertEqual(sl[sig_f1][3], sl[sig_f2][3])   # a1
            self.assertEqual(sl[sig_f1][4], sl[sig_f2][4])   # a2

    def test_unsupported_pair_raises(self):
        class _D:
            name = "bogus"
            if_pairs = [(0, "GPS-NOPE1", "GPS-NOPE2", "G")]
        with self.assertRaises(ValueError):
            r.build_sig_lookup(_D())


class TestReplaySigConfig(unittest.TestCase):
    def _bundle(self, d, receiver):
        b = RawCaptureBundle(d)
        conv = {"systems": ["gps", "gal"]}
        if receiver is not None:
            conv["receiver"] = receiver
        b.write_manifest(host="h", started_iso="2026-01-01T00:00:00+00:00",
                         conventions=conv)
        b.close()

    def test_reconstructs_sig_config_from_manifest(self):
        with tempfile.TemporaryDirectory() as d:
            self._bundle(d, "f9t")
            names, sig_lookup, bds = drv.replay_sig_config(d)
            # identical to building it directly from the driver
            driver = get_driver("f9t")
            self.assertEqual(names, driver.signal_names)
            self.assertEqual(sig_lookup, r.build_sig_lookup(driver))
            self.assertEqual(bds, driver.bds_l1_ref_cycles)

    def test_missing_receiver_raises(self):
        with tempfile.TemporaryDirectory() as d:
            self._bundle(d, None)                 # no receiver in manifest
            with self.assertRaises(ValueError):
                drv.replay_sig_config(d)


class TestEngineRecordsReceiver(unittest.TestCase):
    def test_manifest_carries_receiver(self):
        import types, logging, tomllib
        import peppar_fix_engine as eng
        with tempfile.TemporaryDirectory() as d:
            args = types.SimpleNamespace(raw_capture_dir=d, receiver="f9t-l5")
            eng.make_raw_capture_bundle(args, ["gps"],
                                        logging.getLogger("t")).close()
            with open(os.path.join(d, "manifest.toml"), "rb") as f:
                conv = tomllib.load(f)["conventions"]
            self.assertEqual(conv["receiver"], "f9t-l5")


if __name__ == "__main__":
    unittest.main()

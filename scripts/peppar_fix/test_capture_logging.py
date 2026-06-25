"""Capture-logging additions for a pos_replay reference capture
(docs/pos-replay-capture-manifest.md): the periodic [METAR] surface-pressure /
ZHD line (Group C ZTD-truth anchor).  The [PPP_STATE] position+σ line (Group B)
runs deep in AntPosEstThread and is exercised by the antpos integration tests;
here we pin the [METAR] line, which is cleanly callable.
"""
import logging
import math
import os
import sys
import types
import unittest
from unittest import mock

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import numpy as np  # noqa: E402
import peppar_fix_engine as eng  # noqa: E402


class _StubFilt:
    """Minimal filter exposing what _periodic_ztd_tie touches."""
    def __init__(self):
        self.x = np.zeros(7)
        self.P = np.eye(7) * 0.04   # σ_ztd = 0.2 m
        self.tied = None

    def apply_ztd_tie(self, sigma_m, target_m=0.0):
        self.tied = (sigma_m, target_m)


class TestMetarLine(unittest.TestCase):
    def test_periodic_ztd_tie_emits_metar_line(self):
        args = types.SimpleNamespace(ztd_tie_sigma_mm=100.0,
                                     init_ztd_station="KORD")
        rec = {"temp_C": 15.0, "dewp_C": 5.0, "altim_hPa": 1013.2,
               "station": "KORD"}
        filt = _StubFilt()
        log = logging.getLogger("test_metar_line")
        with mock.patch.object(eng, "fetch_latest_metar", return_value=rec), \
                mock.patch.object(eng, "metar_age_seconds", return_value=600.0), \
                self.assertLogs(log, level="INFO") as cm:
            eng._periodic_ztd_tie(filt, args, lat_deg=41.9, alt_m=200.0,
                                  n_epochs=123, log=log)
        joined = "\n".join(cm.output)
        # the [METAR] anchor line is present with real Saastamoinen ZHD/ZTD
        self.assertIn("[METAR]", joined)
        self.assertIn("Pstn=", joined)
        self.assertIn("ZHD=", joined)
        self.assertIn("ZTD=", joined)
        self.assertIn("KORD", joined)
        # and the tie still applied (the [METAR] line didn't short-circuit it)
        self.assertIsNotNone(filt.tied)

    def test_metar_fetch_failure_skips_cleanly(self):
        args = types.SimpleNamespace(ztd_tie_sigma_mm=100.0,
                                     init_ztd_station="KORD")
        filt = _StubFilt()
        log = logging.getLogger("test_metar_fail")
        with mock.patch.object(eng, "fetch_latest_metar",
                               side_effect=eng.MetarReadError("no net")), \
                self.assertLogs(log, level="WARNING") as cm:
            eng._periodic_ztd_tie(filt, args, lat_deg=41.9, alt_m=200.0,
                                  n_epochs=1, log=log)
        # no [METAR] line on fetch failure; tie skipped (non-fatal)
        self.assertNotIn("[METAR]", "\n".join(cm.output))
        self.assertIsNone(filt.tied)


class TestRawCaptureBundleSetup(unittest.TestCase):
    """make_raw_capture_bundle: None when --raw-capture-dir unset; otherwise a
    bundle with a manifest pinning the operational conventions (§6)."""

    def test_returns_none_when_flag_unset(self):
        args = types.SimpleNamespace(raw_capture_dir=None)
        self.assertIsNone(eng.make_raw_capture_bundle(
            args, ["gps"], logging.getLogger("t")))

    def test_creates_bundle_and_manifest(self):
        import tempfile
        import tomllib
        with tempfile.TemporaryDirectory() as d:
            args = types.SimpleNamespace(
                raw_capture_dir=d, init_ztd_station="KORD",
                known_pos="-2730000,-4440000,3975000")
            bundle = eng.make_raw_capture_bundle(
                args, ["gps", "gal"], logging.getLogger("t"))
            self.assertIsNotNone(bundle)
            bundle.close()
            with open(os.path.join(d, "manifest.toml"), "rb") as f:
                m = tomllib.load(f)
            self.assertEqual(m["conventions"]["ztd_station"], "KORD")
            # sorted → deterministic regardless of input order (set or list)
            self.assertEqual(m["conventions"]["systems"], ["gal", "gps"])
            self.assertEqual(m["conventions"]["known_pos"],
                             "-2730000,-4440000,3975000")
            self.assertNotEqual(m["software"]["git_rev"], "unknown")


if __name__ == "__main__":
    unittest.main()

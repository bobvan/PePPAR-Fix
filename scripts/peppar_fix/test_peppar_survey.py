"""Tests for the peppar-survey CLI tool.

The --from-ppp backend was removed 2026-05-18 — "survey" is reserved
for external authoritative observation sources (OPUS / PRIDE / CORS)
per docs/position-state-and-monitoring.md.  Until a real backend
lands, peppar-survey errors out explicitly.

These tests cover the surviving surfaces: UID auto-discovery and the
no-backend-implemented error path.
"""

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

# Make scripts/peppar_survey.py importable when run via the venv.
_REPO = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", ".."))
_SCRIPTS = os.path.join(_REPO, "scripts")
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import peppar_survey  # noqa: E402


def _write_receiver_json(d, uid, mount_sn=0):
    with open(os.path.join(d, f"{uid}.json"), "w") as f:
        json.dump({"unique_id": int(uid) if str(uid).isdigit() else uid,
                   "mount_sn": mount_sn}, f)


class TestDiscoverSingleReceiverUid(unittest.TestCase):

    def test_zero_receivers_returns_none(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertIsNone(
                peppar_survey.discover_single_receiver_uid(d))

    def test_one_receiver_returns_uid(self):
        with tempfile.TemporaryDirectory() as d:
            _write_receiver_json(d, "12345")
            self.assertEqual(
                peppar_survey.discover_single_receiver_uid(d), "12345")

    def test_synthetic_uid_filename(self):
        """Synthetic UIDs (e.g. synth_D30GD1PE) for receivers without
        SEC-UNIQID should be discoverable just like decimal UIDs."""
        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "synth_D30GD1PE.json"), "w") as f:
                json.dump({"unique_id": "synth_D30GD1PE",
                           "mount_sn": 0}, f)
            self.assertEqual(
                peppar_survey.discover_single_receiver_uid(d),
                "synth_D30GD1PE")

    def test_multiple_receivers_returns_none(self):
        with tempfile.TemporaryDirectory() as d:
            _write_receiver_json(d, "12345")
            _write_receiver_json(d, "67890")
            self.assertIsNone(
                peppar_survey.discover_single_receiver_uid(d))

    def test_bak_files_ignored(self):
        with tempfile.TemporaryDirectory() as d:
            _write_receiver_json(d, "12345")
            with open(os.path.join(d,
                      "12345.json.day0424.bak"), "w") as f:
                f.write("{}")
            self.assertEqual(
                peppar_survey.discover_single_receiver_uid(d), "12345")

    def test_missing_dir_returns_none(self):
        self.assertIsNone(
            peppar_survey.discover_single_receiver_uid("/nope/nope"))


class TestMainNoBackendError(unittest.TestCase):
    """Until a real backend (PRIDE / OPUS / CORS / RTKLIB) lands,
    peppar-survey errors out so operators don't think it silently
    succeeded."""

    def setUp(self):
        self.recv = tempfile.mkdtemp()
        self.pos = tempfile.mkdtemp()
        self.uid = "77777"
        _write_receiver_json(self.recv, self.uid, mount_sn=2)

    def tearDown(self):
        import shutil
        shutil.rmtree(self.recv)
        shutil.rmtree(self.pos)

    def test_explicit_uid_returns_error_no_backend(self):
        rc = peppar_survey.main([
            "--receiver-uid", self.uid,
            "--positions-dir", self.pos,
            "--receivers-dir", self.recv,
        ])
        self.assertEqual(rc, 2)
        self.assertEqual(os.listdir(self.pos), [])

    def test_auto_discover_uid_returns_error_no_backend(self):
        rc = peppar_survey.main([
            "--positions-dir", self.pos,
            "--receivers-dir", self.recv,
        ])
        self.assertEqual(rc, 2)
        self.assertEqual(os.listdir(self.pos), [])

    def test_no_receivers_returns_one(self):
        empty = tempfile.mkdtemp()
        try:
            rc = peppar_survey.main([
                "--positions-dir", self.pos,
                "--receivers-dir", empty,
            ])
            self.assertEqual(rc, 1)
        finally:
            import shutil
            shutil.rmtree(empty)


class BaselineCliTest(unittest.TestCase):
    """--baseline base source: --base auto-detects station-code vs RINEX path;
    exactly one of --base / --base-ntrip-host is required (I-071401)."""

    def _run(self, base_args):
        with tempfile.TemporaryDirectory() as td:
            obs = Path(td) / "rover-2026140.obs"
            obs.write_text("x")
            with mock.patch(
                    "peppar_fix.peppar_survey_rtklib.run_rtklib_backend",
                    return_value=0) as rb:
                rc = peppar_survey.main(
                    ["--baseline", "--rinex-glob", str(obs),
                     "--receiver-uid", "U"] + base_args)
            return rc, rb

    def test_base_station_code_detected(self):
        rc, rb = self._run(["--base", "dsp1"])
        self.assertEqual(rc, 0)
        kw = rb.call_args.kwargs
        self.assertEqual(kw["mode"], "rtk")
        self.assertEqual(kw["cors_station"], "dsp1")
        self.assertIsNone(kw["cors_rinex_path"])

    def test_base_rinex_path_detected(self):
        with tempfile.TemporaryDirectory() as td:
            bp = str(Path(td) / "base.obs")
            rc, rb = self._run(["--base", bp])
        self.assertEqual(rc, 0)
        kw = rb.call_args.kwargs
        self.assertIsNone(kw["cors_station"])
        self.assertEqual(str(kw["cors_rinex_path"]), bp)

    def test_base_realization_forwarded(self):
        rc, rb = self._run(["--base", "dsp1", "--base-realization", "ETRS89"])
        self.assertEqual(rc, 0)
        self.assertEqual(rb.call_args.kwargs["base_realization"], "ETRS89")

    def test_requires_exactly_one_base_source(self):
        self.assertEqual(self._run(["--base", "d", "--base-ntrip-host", "h"])[0], 2)
        self.assertEqual(self._run([])[0], 2)


if __name__ == "__main__":
    unittest.main()

"""Tests for the peppar-survey CLI tool."""

import json
import os
import sys
import tempfile
import unittest

# Make scripts/peppar_survey.py importable when run via the venv.
_REPO = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", ".."))
_SCRIPTS = os.path.join(_REPO, "scripts")
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import peppar_survey  # noqa: E402

from peppar_fix.position_state import (  # noqa: E402
    PositionState, load_ppp_state, load_survey_state, save_ppp_state,
    utc_now_iso,
)


def _write_receiver_json(d, uid, mount_sn=0):
    with open(os.path.join(d, f"{uid}.json"), "w") as f:
        json.dump({"unique_id": int(uid), "mount_sn": mount_sn}, f)


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

    def test_multiple_receivers_returns_none(self):
        with tempfile.TemporaryDirectory() as d:
            _write_receiver_json(d, "12345")
            _write_receiver_json(d, "67890")
            self.assertIsNone(
                peppar_survey.discover_single_receiver_uid(d))

    def test_bak_files_ignored(self):
        with tempfile.TemporaryDirectory() as d:
            _write_receiver_json(d, "12345")
            # Mimic the lab convention .json.<tag>.bak.
            with open(os.path.join(d,
                      "12345.json.day0424.bak"), "w") as f:
                f.write("{}")
            self.assertEqual(
                peppar_survey.discover_single_receiver_uid(d), "12345")

    def test_missing_dir_returns_none(self):
        self.assertIsNone(
            peppar_survey.discover_single_receiver_uid("/nope/nope"))


class TestSurveyFromPpp(unittest.TestCase):

    def setUp(self):
        self.d = tempfile.mkdtemp()
        self.uid = "55555"
        # Seed a .ppp.toml as if the engine had been running.
        src = PositionState(
            mount_sn=3,
            ecef_m=(157469.3814, -4756189.0729, 4232768.5274),
            sigma_m=0.012,
            updated=utc_now_iso(),
            source="peppar_fix_engine AntPosEst",
            extra={"n_epochs": 12345},
        )
        save_ppp_state(src, self.uid, positions_dir=self.d)

    def tearDown(self):
        import shutil
        shutil.rmtree(self.d)

    def test_writes_survey_toml(self):
        rc = peppar_survey.survey_from_ppp(
            self.uid, positions_dir=self.d)
        self.assertEqual(rc, 0)
        loaded = load_survey_state(self.uid, positions_dir=self.d)
        self.assertIsNotNone(loaded)
        self.assertEqual(loaded.kind, "survey")
        self.assertEqual(loaded.mount_sn, 3)
        self.assertAlmostEqual(loaded.sigma_m, 0.012, places=6)
        self.assertEqual(loaded.ecef_m,
                         (157469.3814, -4756189.0729, 4232768.5274))
        self.assertIn("--from-ppp", loaded.source)

    def test_provenance_extras_preserved(self):
        peppar_survey.survey_from_ppp(self.uid, positions_dir=self.d)
        loaded = load_survey_state(self.uid, positions_dir=self.d)
        self.assertEqual(loaded.extra.get("from_ppp_n_epochs"), 12345)
        self.assertIn("from_ppp_source", loaded.extra)
        self.assertIn("from_ppp_updated", loaded.extra)

    def test_atomic_write_no_tmp_leftover(self):
        peppar_survey.survey_from_ppp(self.uid, positions_dir=self.d)
        self.assertIn(f"{self.uid}.survey.toml", os.listdir(self.d))
        self.assertNotIn(f"{self.uid}.survey.toml.tmp",
                         os.listdir(self.d))

    def test_missing_ppp_returns_nonzero(self):
        # Wipe the .ppp.toml first.
        os.remove(os.path.join(self.d, f"{self.uid}.ppp.toml"))
        rc = peppar_survey.survey_from_ppp(
            self.uid, positions_dir=self.d)
        self.assertEqual(rc, 1)
        # No .survey.toml created.
        self.assertIsNone(load_survey_state(self.uid,
                                            positions_dir=self.d))

    def test_dry_run_does_not_write(self):
        rc = peppar_survey.survey_from_ppp(
            self.uid, positions_dir=self.d, dry_run=True)
        self.assertEqual(rc, 0)
        self.assertIsNone(load_survey_state(self.uid,
                                            positions_dir=self.d))


class TestMainArgsRouting(unittest.TestCase):
    """End-to-end through the argparse main()."""

    def setUp(self):
        self.recv = tempfile.mkdtemp()
        self.pos = tempfile.mkdtemp()
        self.uid = "77777"
        _write_receiver_json(self.recv, self.uid, mount_sn=2)
        src = PositionState(
            mount_sn=2,
            ecef_m=(1.0, 2.0, 3.0),
            sigma_m=0.05,
            updated=utc_now_iso(),
            source="test",
        )
        save_ppp_state(src, self.uid, positions_dir=self.pos)

    def tearDown(self):
        import shutil
        shutil.rmtree(self.recv)
        shutil.rmtree(self.pos)

    def test_explicit_uid_from_ppp(self):
        rc = peppar_survey.main([
            "--receiver-uid", self.uid,
            "--positions-dir", self.pos,
            "--from-ppp",
        ])
        self.assertEqual(rc, 0)
        self.assertIsNotNone(load_survey_state(
            self.uid, positions_dir=self.pos))

    def test_auto_discover_uid_from_ppp(self):
        rc = peppar_survey.main([
            "--receivers-dir", self.recv,
            "--positions-dir", self.pos,
            "--from-ppp",
        ])
        self.assertEqual(rc, 0)
        self.assertIsNotNone(load_survey_state(
            self.uid, positions_dir=self.pos))

    def test_no_backend_errors(self):
        with self.assertRaises(SystemExit):
            peppar_survey.main([
                "--receiver-uid", self.uid,
                "--positions-dir", self.pos,
            ])


if __name__ == "__main__":
    unittest.main()

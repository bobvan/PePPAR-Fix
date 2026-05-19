"""Tests for the peppar-survey --pride backend."""
from __future__ import annotations

import json
import os
import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import mock

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.peppar_survey_pride import (
    PrideRunResult, doy_from_obs_name, expected_pos_name,
    process_one_obs, run_pride_backend, site_from_obs_header,
    write_survey_from_running,
)
from peppar_fix.arp_history import (
    RunningArp, append_solution, apply_quality_filter, running_mean,
)
from peppar_fix.position_state import (
    PositionState, load_survey_state, save_survey_state,
)
from peppar_fix.pride_pos_reader import parse_pos


# Bundled real-world PRIDE samples (skip individual tests when absent).
SAMPLE_POS_DIRS = [
    Path("/home/bob/git/timelab/surveys/data/pride-f9t-ptpmon-20260504"),
]
SAMPLE_POS_FILE = Path(
    "/home/bob/git/timelab/surveys/data/pride/pos_2026116_ufo1")


# ─── tiny helpers ───────────────────────────────────────────────── #


def _has_real_pos_samples() -> bool:
    return SAMPLE_POS_FILE.is_file()


# ─── filename / header parsing ──────────────────────────────────── #


class DoyFromObsNameTest(unittest.TestCase):

    def test_madhat_pattern(self):
        self.assertEqual(
            doy_from_obs_name(Path("data/rinex/MadHat-2026133.obs")),
            (2026, 133))

    def test_ufo1_pattern(self):
        self.assertEqual(
            doy_from_obs_name(Path("ufo1-2026126.obs")),
            (2026, 126))

    def test_no_match_returns_none(self):
        self.assertIsNone(
            doy_from_obs_name(Path("daily.obs")))
        self.assertIsNone(
            doy_from_obs_name(Path("MadHat-corrupt.obs")))


class ExpectedPosNameTest(unittest.TestCase):

    def test_format(self):
        self.assertEqual(
            expected_pos_name(2026, 133, "mad1"),
            "pos_2026133_mad1")
        # Zero-padding holds at boundaries
        self.assertEqual(
            expected_pos_name(2026, 5, "x"),
            "pos_2026005_x")


class SiteFromObsHeaderTest(unittest.TestCase):

    def test_marker_name_extracted(self):
        with TemporaryDirectory() as td:
            obs = Path(td) / "fake.obs"
            obs.write_text(
                "     3.04           OBSERVATION DATA    M (MIXED)           "
                "RINEX VERSION / TYPE\n"
                "MADHAT                                                      "
                "MARKER NAME\n"
                "                                                            "
                "END OF HEADER\n"
            )
            self.assertEqual(site_from_obs_header(obs), "madh")

    def test_no_marker_returns_none(self):
        with TemporaryDirectory() as td:
            obs = Path(td) / "noname.obs"
            obs.write_text(
                "     3.04           OBSERVATION DATA    M (MIXED)           "
                "RINEX VERSION / TYPE\n"
                "                                                            "
                "END OF HEADER\n"
            )
            self.assertIsNone(site_from_obs_header(obs))


# ─── process_one_obs (pdp3 invocation mocked) ───────────────────── #


class ProcessOneObsTest(unittest.TestCase):

    @unittest.skipUnless(_has_real_pos_samples(),
                         "PRIDE sample pos file not available")
    def test_success_first_attempt(self):
        """When pdp3 succeeds on -sys GREC, parse + return solution."""

        def fake_pdp3(obs_file, work_dir, sys_str, **kw):
            # Pretend pdp3 produced the bundled sample.  Caller will parse it.
            return PrideRunResult(
                obs_file=obs_file, sys_attempted=sys_str,
                returncode=0,
                pos_path=SAMPLE_POS_FILE,
                log_path=None,
            )

        with TemporaryDirectory() as td:
            sol, last = process_one_obs(
                Path(td) / "fake.obs",
                Path(td) / "work",
                sys_attempts=("GREC", "GR"),
                pdp3_runner=fake_pdp3,
            )
        self.assertIsNotNone(sol)
        self.assertEqual(last.sys_attempted, "GREC")
        # Sanity check on parsed values: real UFO1 sample is ARP-class.
        self.assertEqual(sol.mode, "Static")

    def test_grec_fails_gr_succeeds_falls_back(self):
        """When -sys GREC errors, -sys GR is attempted next."""
        calls = []

        def fake_pdp3(obs_file, work_dir, sys_str, **kw):
            calls.append(sys_str)
            if sys_str == "GREC":
                return PrideRunResult(
                    obs_file=obs_file, sys_attempted=sys_str,
                    returncode=1, pos_path=None, log_path=None,
                    error="simulated BDS failure",
                )
            # GR succeeds
            return PrideRunResult(
                obs_file=obs_file, sys_attempted=sys_str,
                returncode=0,
                pos_path=SAMPLE_POS_FILE if _has_real_pos_samples() else None,
                log_path=None,
            )

        with TemporaryDirectory() as td:
            sol, last = process_one_obs(
                Path(td) / "fake.obs",
                Path(td) / "work",
                sys_attempts=("GREC", "GR"),
                pdp3_runner=fake_pdp3,
            )
        self.assertEqual(calls, ["GREC", "GR"])
        if _has_real_pos_samples():
            self.assertIsNotNone(sol)
        self.assertEqual(last.sys_attempted, "GR")

    def test_all_attempts_fail_returns_none(self):
        """When every -sys fails, return (None, last_result)."""

        def fake_pdp3(obs_file, work_dir, sys_str, **kw):
            return PrideRunResult(
                obs_file=obs_file, sys_attempted=sys_str,
                returncode=1, pos_path=None, log_path=None,
                error="every attempt failed",
            )

        with TemporaryDirectory() as td:
            sol, last = process_one_obs(
                Path(td) / "fake.obs",
                Path(td) / "work",
                sys_attempts=("GREC", "GR"),
                pdp3_runner=fake_pdp3,
            )
        self.assertIsNone(sol)
        self.assertIsNotNone(last)
        self.assertEqual(last.sys_attempted, "GR")


# ─── append → running_mean → survey.toml round-trip ─────────────── #


@unittest.skipUnless(_has_real_pos_samples(),
                     "PRIDE sample pos file not available")
class AppendRunningMeanRoundtripTest(unittest.TestCase):

    def test_single_solution_round_trip(self):
        """One real PRIDE pos → history.jsonl → running_mean → survey.toml."""
        sol = parse_pos(SAMPLE_POS_FILE)
        with TemporaryDirectory() as td:
            hist = Path(td) / "history.jsonl"
            rec = append_solution(hist, sol, mount_sn=0)
            self.assertEqual(rec["mount_sn"], 0)
            # quality_ok respects the formal-σ + n_obs gates
            self.assertEqual(rec["quality_ok"],
                             apply_quality_filter(sol))

            running = running_mean(hist, n_days=7, mount_sn=0,
                                   require_quality_ok=False)
            self.assertIsNotNone(running)
            # Single record → mean ECEF == sample ECEF
            for got, want in zip(running.ecef_m, sol.ecef_m):
                self.assertAlmostEqual(got, want, places=4)
            self.assertEqual(running.count, 1)

            state, path = write_survey_from_running(
                running, uid="testuid",
                positions_dir=td,
            )
        self.assertEqual(state.kind, "survey")
        self.assertEqual(state.mount_sn, 0)
        self.assertAlmostEqual(state.sigma_m,
                               running.sigma_3d_m, places=8)

    def test_dry_run_does_not_write(self):
        sol = parse_pos(SAMPLE_POS_FILE)
        with TemporaryDirectory() as td:
            hist = Path(td) / "history.jsonl"
            append_solution(hist, sol, mount_sn=0)
            running = running_mean(hist, mount_sn=0,
                                   require_quality_ok=False)
            assert running is not None
            state, path = write_survey_from_running(
                running, uid="testuid",
                positions_dir=td, dry_run=True,
            )
        self.assertEqual(state.kind, "survey")
        self.assertFalse(os.path.exists(path))

    def test_extras_recorded(self):
        sol = parse_pos(SAMPLE_POS_FILE)
        with TemporaryDirectory() as td:
            hist = Path(td) / "history.jsonl"
            append_solution(hist, sol, mount_sn=0)
            running = running_mean(hist, mount_sn=0,
                                   require_quality_ok=False)
            assert running is not None
            state, _ = write_survey_from_running(
                running, uid="testuid",
                positions_dir=td,
            )
        # Verify extras pass through
        self.assertIn("pride_window_count", state.extra)
        self.assertIn("pride_sigma_x_m", state.extra)
        self.assertIn("pride_oldest_mjd", state.extra)


# ─── full backend orchestrator ───────────────────────────────────── #


@unittest.skipUnless(_has_real_pos_samples(),
                     "PRIDE sample pos file not available")
class RunPrideBackendTest(unittest.TestCase):

    def _fake_runner_returning(self, pos_path):
        def fake_pdp3(obs_file, work_dir, sys_str, **kw):
            return PrideRunResult(
                obs_file=obs_file, sys_attempted=sys_str,
                returncode=0,
                pos_path=pos_path,
                log_path=None,
            )
        return fake_pdp3

    def test_single_file_end_to_end(self):
        """One RINEX → pos parse → history append → survey.toml written."""
        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = tdp / "MadHat-2026133.obs"
            obs.write_text("")  # content doesn't matter; pdp3 mocked
            positions_dir = tdp / "positions"
            history_dir = tdp / "arp"

            rc = run_pride_backend(
                obs_files=[obs],
                work_dir=tdp / "work",
                receiver_uid="testuid",
                positions_dir=str(positions_dir),
                history_dir=str(history_dir),
                mount_sn=0,
                pdp3_runner=self._fake_runner_returning(SAMPLE_POS_FILE),
                pdp3_bin="/dev/null",  # not actually called via mock
            )
            # pdp3_bin path check happens before the runner, so this
            # would normally fail rc=3.  Override: pass a path that
            # exists.  Use /bin/true as a safe always-present binary.
            # Re-running with that.
        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = tdp / "MadHat-2026133.obs"
            obs.write_text("")
            positions_dir = tdp / "positions"
            history_dir = tdp / "arp"
            rc = run_pride_backend(
                obs_files=[obs],
                work_dir=tdp / "work",
                receiver_uid="testuid",
                positions_dir=str(positions_dir),
                history_dir=str(history_dir),
                mount_sn=0,
                pdp3_runner=self._fake_runner_returning(SAMPLE_POS_FILE),
                pdp3_bin="/bin/true",
            )
            self.assertEqual(rc, 0)
            survey = positions_dir / "testuid.survey.toml"
            self.assertTrue(survey.is_file())
            text = survey.read_text()
            self.assertIn("mount_sn = 0", text)
            self.assertIn("ecef_m = [", text)
            self.assertIn("sigma_m =", text)
            # History file should also exist
            self.assertTrue((history_dir / "testuid" / "history.jsonl").is_file())

    def test_empty_obs_list_returns_1(self):
        with TemporaryDirectory() as td:
            rc = run_pride_backend(
                obs_files=[],
                work_dir=Path(td),
                receiver_uid="testuid",
                pdp3_bin="/bin/true",
            )
        self.assertEqual(rc, 1)

    def test_missing_pdp3_binary_returns_3(self):
        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = tdp / "MadHat-2026133.obs"
            obs.write_text("")
            rc = run_pride_backend(
                obs_files=[obs],
                work_dir=tdp,
                receiver_uid="testuid",
                pdp3_bin="/nonexistent/pdp3",
            )
        self.assertEqual(rc, 3)

    def test_all_obs_fail_returns_2(self):
        def fake_pdp3(obs_file, work_dir, sys_str, **kw):
            return PrideRunResult(
                obs_file=obs_file, sys_attempted=sys_str,
                returncode=1, pos_path=None, log_path=None,
                error="all failures all the time",
            )
        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = tdp / "MadHat-2026133.obs"
            obs.write_text("")
            rc = run_pride_backend(
                obs_files=[obs],
                work_dir=tdp / "work",
                receiver_uid="testuid",
                positions_dir=str(tdp / "positions"),
                history_dir=str(tdp / "arp"),
                pdp3_runner=fake_pdp3,
                pdp3_bin="/bin/true",
            )
        self.assertEqual(rc, 2)


if __name__ == "__main__":
    unittest.main()

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
    PrideRunResult, WUM_KINDS, WUM_VARIANTS, brdm_filename,
    doy_from_obs_name, expected_pos_name, gnsswhu_url, invoke_pdp3,
    process_one_obs, resolve_brdm_source, resolve_wum_source,
    run_pride_backend, site_from_obs_header, stage_wum_products,
    wum_filename, write_survey_from_running, ydoy_to_gpsweek,
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


# ─── --brdm-source plumbing ─────────────────────────────────────── #
#
# Background: pdp3.sh's IGS-MGEX multi-GNSS broadcast-nav download
# chain (gnsswhu/IGN/DLR) intermittently falls through to GPS-only
# nav.  Without GAL/BDS ephemeris, pdp3's elevation() returns dist=-1
# for every non-GPS SV → DEL_BADRANGE → zero kin solution rows.
# Diagnosed 2026-05-19 in prideBadRangeDiagnostic-main thread.  The
# --brdm-source workaround stages an operator-supplied multi-GNSS
# nav file into pdp3's work_dir under the canonical name pdp3.sh
# looks for at lines 2110-2117, bypassing the download.


class BrdmFilenameTest(unittest.TestCase):

    def test_canonical_format(self):
        """brdm{doy:03d}0.{yy:02d}p — matches pdp3.sh:1578."""
        self.assertEqual(brdm_filename(2026, 139), "brdm1390.26p")
        self.assertEqual(brdm_filename(2024, 1), "brdm0010.24p")
        self.assertEqual(brdm_filename(2099, 365), "brdm3650.99p")


class ResolveBrdmSourceTest(unittest.TestCase):

    def test_none_returns_none(self):
        self.assertIsNone(resolve_brdm_source(None, 2026, 139))

    def test_file_path_returned_as_is(self):
        """An explicit file path is used verbatim regardless of doy."""
        with TemporaryDirectory() as td:
            p = Path(td) / "some-brdm.26p"
            p.write_text("dummy")
            got = resolve_brdm_source(p, 2026, 139)
            self.assertEqual(got, p)
            # Also works when the filename doesn't match canonical name —
            # operator's responsibility to feed the right day.
            got2 = resolve_brdm_source(p, 2099, 1)
            self.assertEqual(got2, p)

    def test_directory_resolves_per_day(self):
        """A directory: look up brdm{doy}0.{yy}p inside."""
        with TemporaryDirectory() as td:
            d = Path(td)
            (d / "brdm1390.26p").write_text("day139")
            (d / "brdm1400.26p").write_text("day140")
            self.assertEqual(resolve_brdm_source(d, 2026, 139),
                             d / "brdm1390.26p")
            self.assertEqual(resolve_brdm_source(d, 2026, 140),
                             d / "brdm1400.26p")

    def test_directory_missing_per_day_returns_none(self):
        """Directory without the requested day → warn, return None."""
        with TemporaryDirectory() as td:
            d = Path(td)
            # Empty directory; no brdm files
            self.assertIsNone(resolve_brdm_source(d, 2026, 139))

    def test_nonexistent_path_returns_none(self):
        """Garbage path: not crash; not stage; pdp3 falls back to download."""
        self.assertIsNone(
            resolve_brdm_source("/nonexistent/path", 2026, 139))


class InvokePdp3BrdmStagingTest(unittest.TestCase):
    """invoke_pdp3 copies brdm_source into work_dir before invoking pdp3.

    Uses mock.patch on subprocess.run so we don't depend on the pdp3
    binary being installed in CI; the assertion is on work_dir state
    AT THE TIME subprocess.run is called.
    """

    def _make_obs_file(self, td: Path, name: str = "MadHat-2026139.obs"):
        p = td / name
        # Minimal valid stub — invoke_pdp3 only copies the file, doesn't
        # parse it before pdp3 runs.
        p.write_text("dummy obs file\n")
        return p

    def test_file_source_staged_under_canonical_name(self):
        """File brdm_source copies into work_dir as brdmDDD0.YYp."""
        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = self._make_obs_file(tdp)
            brdm = tdp / "operator-staged.26p"
            brdm.write_text("multi-gnss nav contents")
            work = tdp / "work"

            captured: dict = {}

            def fake_run(cmd, **kw):
                # Snapshot work_dir contents at invocation time so we can
                # assert the brdm is staged BEFORE pdp3 runs.
                captured["files"] = sorted(p.name for p in work.iterdir())
                # Match the subprocess.CompletedProcess shape invoke_pdp3
                # expects on a no-op success.
                class _Result:
                    returncode = 0
                    stdout = ""
                    stderr = ""
                return _Result()

            with mock.patch(
                "peppar_fix.peppar_survey_pride.subprocess.run", fake_run
            ):
                invoke_pdp3(
                    obs, work, "GREC",
                    pdp3_bin="/tmp/fake-pdp3",
                    brdm_source=brdm,
                )

            self.assertIn("brdm1390.26p", captured["files"])
            self.assertIn("MadHat-2026139.obs", captured["files"])
            # Content matches the operator's file
            self.assertEqual(
                (work / "brdm1390.26p").read_text(),
                "multi-gnss nav contents",
            )

    def test_directory_source_picks_matching_day(self):
        """Directory brdm_source picks brdmDDD0.YYp per obs day."""
        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = self._make_obs_file(tdp)
            archive = tdp / "brdm-archive"
            archive.mkdir()
            (archive / "brdm1380.26p").write_text("day 138")
            (archive / "brdm1390.26p").write_text("day 139 — match")
            (archive / "brdm1400.26p").write_text("day 140")
            work = tdp / "work"

            with mock.patch(
                "peppar_fix.peppar_survey_pride.subprocess.run",
                lambda *a, **kw: mock.Mock(returncode=0, stdout="", stderr=""),
            ):
                invoke_pdp3(
                    obs, work, "GREC",
                    pdp3_bin="/tmp/fake-pdp3",
                    brdm_source=archive,
                )

            staged = work / "brdm1390.26p"
            self.assertTrue(staged.is_file())
            self.assertEqual(staged.read_text(), "day 139 — match")

    def test_no_brdm_source_no_staging(self):
        """Without --brdm-source, no brdm file ends up in work_dir."""
        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = self._make_obs_file(tdp)
            work = tdp / "work"

            captured: dict = {}

            def fake_run(cmd, **kw):
                captured["files"] = sorted(p.name for p in work.iterdir())
                return mock.Mock(returncode=0, stdout="", stderr="")

            with mock.patch(
                "peppar_fix.peppar_survey_pride.subprocess.run", fake_run
            ):
                invoke_pdp3(
                    obs, work, "GREC",
                    pdp3_bin="/tmp/fake-pdp3",
                    brdm_source=None,
                )

            self.assertNotIn("brdm1390.26p", captured["files"])

    def test_unparseable_obs_name_skips_brdm_staging(self):
        """RINEX-2 style filename (no YYYYDDD pattern): warn + skip
        staging.  pdp3 falls back to its normal download.  No crash."""
        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = self._make_obs_file(tdp, name="time1390.26o")  # RINEX 2 style
            brdm = tdp / "operator-staged.26p"
            brdm.write_text("nav")
            work = tdp / "work"

            captured: dict = {}

            def fake_run(cmd, **kw):
                captured["files"] = sorted(p.name for p in work.iterdir())
                return mock.Mock(returncode=0, stdout="", stderr="")

            with mock.patch(
                "peppar_fix.peppar_survey_pride.subprocess.run", fake_run
            ):
                invoke_pdp3(
                    obs, work, "GREC",
                    pdp3_bin="/tmp/fake-pdp3",
                    brdm_source=brdm,
                )

            # No staging happened (canonical brdm-name absent)
            self.assertFalse(
                any(name.startswith("brdm") for name in captured["files"])
            )


class ProcessOneObsBrdmPassthroughTest(unittest.TestCase):
    """brdm_source threads through process_one_obs → pdp3_runner."""

    def test_brdm_source_reaches_runner(self):
        seen_brdm = []

        def fake_pdp3(obs_file, work_dir, sys_str, *,
                      brdm_source=None, **kw):
            seen_brdm.append(brdm_source)
            return PrideRunResult(
                obs_file=obs_file, sys_attempted=sys_str,
                returncode=1, pos_path=None, log_path=None,
                error="stub",
            )

        with TemporaryDirectory() as td:
            tdp = Path(td)
            brdm = tdp / "nav.26p"
            brdm.write_text("dummy")
            process_one_obs(
                tdp / "MadHat-2026139.obs",
                tdp / "work",
                sys_attempts=("GREC",),
                pdp3_runner=fake_pdp3,
                brdm_source=brdm,
            )
        self.assertEqual(len(seen_brdm), 1)
        self.assertEqual(seen_brdm[0], brdm)


# ─── --wum-source plumbing ──────────────────────────────────────── #
#
# Mirrors the --brdm-source test shape.  Same idea: operator pre-
# fetches WUM orbit/clock/bias/ERP into a directory; peppar-survey
# gunzips matching files into pdp3's product/common before each run.


class WumFilenameTest(unittest.TestCase):

    def test_orbit_filename_format(self):
        self.assertEqual(
            wum_filename("orbit", "RTS", 2026, 140),
            "WUM0MGXRTS_20261400000_01D_05M_ORB.SP3",
        )
        self.assertEqual(
            wum_filename("orbit", "FIN", 2026, 140, gzipped=True),
            "WUM0MGXFIN_20261400000_01D_05M_ORB.SP3.gz",
        )

    def test_clock_variant_dependent_interval(self):
        """RAP/FIN clocks have 30S, RTS has 05S (per pdp3.sh)."""
        self.assertEqual(
            wum_filename("clock", "RAP", 2026, 140),
            "WUM0MGXRAP_20261400000_01D_30S_CLK.CLK",
        )
        self.assertEqual(
            wum_filename("clock", "RTS", 2026, 140),
            "WUM0MGXRTS_20261400000_01D_05S_CLK.CLK",
        )
        self.assertEqual(
            wum_filename("clock", "FIN", 2026, 140),
            "WUM0MGXFIN_20261400000_01D_30S_CLK.CLK",
        )

    def test_bias_variant_dependent_interval(self):
        """RAP/FIN biases are 01D, RTS is 05M (per pdp3.sh)."""
        self.assertEqual(
            wum_filename("bias", "RAP", 2026, 140),
            "WUM0MGXRAP_20261400000_01D_01D_OSB.BIA",
        )
        self.assertEqual(
            wum_filename("bias", "RTS", 2026, 140),
            "WUM0MGXRTS_20261400000_01D_05M_OSB.BIA",
        )

    def test_erp_filename_format(self):
        self.assertEqual(
            wum_filename("erp", "RTS", 2026, 140),
            "WUM0MGXRTS_20261400000_01D_01D_ERP.ERP",
        )

    def test_unknown_kind_raises(self):
        with self.assertRaises(ValueError):
            wum_filename("position", "RTS", 2026, 140)

    def test_unknown_variant_raises(self):
        with self.assertRaises(ValueError):
            wum_filename("orbit", "NEAT", 2026, 140)


class YdoyToGpsweekTest(unittest.TestCase):

    def test_gps_epoch(self):
        # 1980-01-06 is GPS week 0 day 0
        self.assertEqual(ydoy_to_gpsweek(1980, 6), 0)

    def test_known_2026_140(self):
        # 2026 DOY 140 = 2026-05-20 = GPS week 2419 (verified against
        # the dayplan note "weeks 2418 and prior at writing").
        self.assertEqual(ydoy_to_gpsweek(2026, 140), 2419)

    def test_week_boundary(self):
        # 2026-01-03 is the last day of GPS week 2399 (Saturday);
        # 2026-01-04 is the first day of week 2400 (Sunday).
        # DOY 3 = Jan 3; DOY 4 = Jan 4.
        self.assertEqual(ydoy_to_gpsweek(2026, 3), 2399)
        self.assertEqual(ydoy_to_gpsweek(2026, 4), 2400)


class GnsswhuUrlTest(unittest.TestCase):

    def test_rts_under_phasebias_per_kind(self):
        """RTS lives under /pub/whu/phasebias/{year}/{subdir}/ where
        subdir varies by kind (orbit/clock/bias/orbit-for-ERP)."""
        self.assertEqual(
            gnsswhu_url("orbit", "RTS", 2026, 140),
            "ftp://igs.gnsswhu.cn/pub/whu/phasebias/2026/orbit/"
            "WUM0MGXRTS_20261400000_01D_05M_ORB.SP3.gz",
        )
        self.assertEqual(
            gnsswhu_url("clock", "RTS", 2026, 140),
            "ftp://igs.gnsswhu.cn/pub/whu/phasebias/2026/clock/"
            "WUM0MGXRTS_20261400000_01D_05S_CLK.CLK.gz",
        )
        self.assertEqual(
            gnsswhu_url("bias", "RTS", 2026, 140),
            "ftp://igs.gnsswhu.cn/pub/whu/phasebias/2026/bias/"
            "WUM0MGXRTS_20261400000_01D_05M_OSB.BIA.gz",
        )
        self.assertEqual(
            gnsswhu_url("erp", "RTS", 2026, 140),
            "ftp://igs.gnsswhu.cn/pub/whu/phasebias/2026/orbit/"
            "WUM0MGXRTS_20261400000_01D_01D_ERP.ERP.gz",
        )

    def test_fin_under_products_mgex_by_gpsweek(self):
        """FIN lives under /pub/gps/products/mgex/{gpsweek}/."""
        # DOY 140 of 2026 → GPS week 2419
        url = gnsswhu_url("bias", "FIN", 2026, 140)
        self.assertEqual(
            url,
            "ftp://igs.gnsswhu.cn/pub/gps/products/mgex/2419/"
            "WUM0MGXFIN_20261400000_01D_01D_OSB.BIA.gz",
        )


class ResolveWumSourceTest(unittest.TestCase):

    def _make_gz(self, path: Path, content: bytes = b"dummy\n") -> None:
        import gzip
        with gzip.open(path, "wb") as fout:
            fout.write(content)

    def test_none_returns_empty(self):
        self.assertEqual(resolve_wum_source(None, 2026, 140), {})

    def test_non_directory_returns_empty(self):
        with TemporaryDirectory() as td:
            p = Path(td) / "not-a-dir"
            p.write_text("oops")
            self.assertEqual(resolve_wum_source(p, 2026, 140), {})
        self.assertEqual(
            resolve_wum_source("/nonexistent/path", 2026, 140), {})

    def test_empty_directory_returns_empty(self):
        with TemporaryDirectory() as td:
            self.assertEqual(resolve_wum_source(Path(td), 2026, 140), {})

    def test_matching_rts_files_picked_up(self):
        with TemporaryDirectory() as td:
            d = Path(td)
            self._make_gz(d / wum_filename("orbit", "RTS", 2026, 140,
                                            gzipped=True))
            self._make_gz(d / wum_filename("clock", "RTS", 2026, 140,
                                            gzipped=True))
            self._make_gz(d / wum_filename("bias", "RTS", 2026, 140,
                                            gzipped=True))
            self._make_gz(d / wum_filename("erp", "RTS", 2026, 140,
                                            gzipped=True))
            got = resolve_wum_source(d, 2026, 140)
            self.assertEqual(sorted(got.keys()), sorted(WUM_KINDS))

    def test_other_day_files_ignored(self):
        with TemporaryDirectory() as td:
            d = Path(td)
            self._make_gz(d / wum_filename("orbit", "RTS", 2026, 139,
                                            gzipped=True))
            self.assertEqual(resolve_wum_source(d, 2026, 140), {})

    def test_decompressed_files_also_resolve(self):
        """Operator may have already gunzipped — accept either form."""
        with TemporaryDirectory() as td:
            d = Path(td)
            (d / wum_filename("orbit", "RTS", 2026, 140)).write_text("plain")
            got = resolve_wum_source(d, 2026, 140)
            self.assertEqual(set(got.keys()), {"orbit"})

    def test_variant_preference_order(self):
        """RAP > RTS > FIN when multiple variants present for the same
        kind."""
        with TemporaryDirectory() as td:
            d = Path(td)
            self._make_gz(d / wum_filename("orbit", "RAP", 2026, 140,
                                            gzipped=True),
                          b"rap")
            self._make_gz(d / wum_filename("orbit", "RTS", 2026, 140,
                                            gzipped=True),
                          b"rts")
            got = resolve_wum_source(d, 2026, 140)
            self.assertIn("RAP", got["orbit"].name)


class StageWumProductsTest(unittest.TestCase):

    def _make_gz(self, path: Path, content: bytes) -> None:
        import gzip
        with gzip.open(path, "wb") as fout:
            fout.write(content)

    def test_gunzips_into_target_dir(self):
        with TemporaryDirectory() as td:
            src_dir = Path(td) / "src"
            tgt_dir = Path(td) / "target"
            src_dir.mkdir()
            gz_name = wum_filename("orbit", "RTS", 2026, 140, gzipped=True)
            self._make_gz(src_dir / gz_name, b"orbit-payload\n")
            n = stage_wum_products({"orbit": src_dir / gz_name}, tgt_dir)
            self.assertEqual(n, 1)
            staged = tgt_dir / gz_name[:-3]  # without .gz
            self.assertTrue(staged.is_file())
            self.assertEqual(staged.read_bytes(), b"orbit-payload\n")

    def test_already_decompressed_files_copy_verbatim(self):
        with TemporaryDirectory() as td:
            src_dir = Path(td) / "src"
            tgt_dir = Path(td) / "target"
            src_dir.mkdir()
            name = wum_filename("orbit", "RTS", 2026, 140)
            (src_dir / name).write_text("plain payload")
            n = stage_wum_products({"orbit": src_dir / name}, tgt_dir)
            self.assertEqual(n, 1)
            self.assertEqual((tgt_dir / name).read_text(), "plain payload")

    def test_creates_target_dir(self):
        with TemporaryDirectory() as td:
            src = Path(td) / "src.gz"
            self._make_gz(src, b"data")
            # Note: the dummy src.gz name doesn't matter — staging
            # writes to the .gz-stripped name.
            tgt = Path(td) / "deeply" / "nested" / "target"
            stage_wum_products({"orbit": src}, tgt)
            self.assertTrue(tgt.is_dir())

    def test_empty_dict_no_op(self):
        with TemporaryDirectory() as td:
            tgt = Path(td) / "target"
            n = stage_wum_products({}, tgt)
            self.assertEqual(n, 0)
            self.assertTrue(tgt.is_dir())  # still created


class InvokePdp3WumStagingTest(unittest.TestCase):
    """invoke_pdp3 stages WUM products into pdp3's product/common
    sibling dir before invoking pdp3."""

    def _make_gz(self, path: Path, content: bytes = b"x") -> None:
        import gzip
        with gzip.open(path, "wb") as fout:
            fout.write(content)

    def _make_obs_file(self, td: Path, name: str = "MadHat-2026140.obs"):
        p = td / name
        p.write_text("dummy obs file\n")
        return p

    def test_wum_source_staged_to_product_common_sibling(self):
        """WUM files go into work_dir.parent / product / common/."""
        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = self._make_obs_file(tdp)
            wum_dir = tdp / "wum-cache"
            wum_dir.mkdir()
            orbit_gz = wum_dir / wum_filename(
                "orbit", "RTS", 2026, 140, gzipped=True)
            self._make_gz(orbit_gz, b"orb")
            # work_dir is the per-attempt dir; product/common lives in
            # the parent (shared across sys_attempts).
            outer = tdp / "outer"
            work = outer / "sys_GREC"

            with mock.patch(
                "peppar_fix.peppar_survey_pride.subprocess.run",
                lambda *a, **kw: mock.Mock(returncode=0, stdout="", stderr=""),
            ):
                invoke_pdp3(
                    obs, work, "GREC",
                    pdp3_bin="/tmp/fake-pdp3",
                    wum_source=wum_dir,
                )

            expected = outer / "product" / "common" / wum_filename(
                "orbit", "RTS", 2026, 140)
            self.assertTrue(expected.is_file(),
                            f"WUM orbit not staged at {expected}")
            self.assertEqual(expected.read_bytes(), b"orb")

    def test_no_wum_source_no_product_dir(self):
        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = self._make_obs_file(tdp)
            outer = tdp / "outer"
            work = outer / "sys_GREC"
            with mock.patch(
                "peppar_fix.peppar_survey_pride.subprocess.run",
                lambda *a, **kw: mock.Mock(returncode=0, stdout="", stderr=""),
            ):
                invoke_pdp3(
                    obs, work, "GREC",
                    pdp3_bin="/tmp/fake-pdp3",
                    wum_source=None,
                )
            self.assertFalse((outer / "product" / "common").exists())

    def test_unparseable_obs_name_skips_wum_staging(self):
        with TemporaryDirectory() as td:
            tdp = Path(td)
            # No YYYYDDD pattern in the name.
            obs = self._make_obs_file(tdp, name="rinex2-style.26o")
            wum_dir = tdp / "wum-cache"
            wum_dir.mkdir()
            self._make_gz(
                wum_dir / wum_filename("orbit", "RTS", 2026, 140,
                                       gzipped=True),
                b"orb")
            outer = tdp / "outer"
            work = outer / "sys_GREC"
            with mock.patch(
                "peppar_fix.peppar_survey_pride.subprocess.run",
                lambda *a, **kw: mock.Mock(returncode=0, stdout="", stderr=""),
            ):
                invoke_pdp3(
                    obs, work, "GREC",
                    pdp3_bin="/tmp/fake-pdp3",
                    wum_source=wum_dir,
                )
            self.assertFalse((outer / "product" / "common").exists())

    def test_brdm_and_wum_can_be_combined(self):
        """Both --brdm-source and --wum-source plumbing in one invocation."""
        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = self._make_obs_file(tdp)
            brdm = tdp / "nav.26p"
            brdm.write_text("multi-gnss nav")
            wum_dir = tdp / "wum-cache"
            wum_dir.mkdir()
            self._make_gz(
                wum_dir / wum_filename("bias", "RTS", 2026, 140,
                                       gzipped=True),
                b"phase-bias")
            outer = tdp / "outer"
            work = outer / "sys_GREC"
            with mock.patch(
                "peppar_fix.peppar_survey_pride.subprocess.run",
                lambda *a, **kw: mock.Mock(returncode=0, stdout="", stderr=""),
            ):
                invoke_pdp3(
                    obs, work, "GREC",
                    pdp3_bin="/tmp/fake-pdp3",
                    brdm_source=brdm,
                    wum_source=wum_dir,
                )
            # brdm went into work_dir (cwd)
            self.assertTrue((work / "brdm1400.26p").is_file())
            # WUM went into product/common sibling
            self.assertTrue((outer / "product" / "common" / wum_filename(
                "bias", "RTS", 2026, 140)).is_file())


class ProcessOneObsWumPassthroughTest(unittest.TestCase):
    """wum_source threads through process_one_obs → pdp3_runner."""

    def test_wum_source_reaches_runner(self):
        seen = []

        def fake_pdp3(obs_file, work_dir, sys_str, *,
                      brdm_source=None, wum_source=None, **kw):
            seen.append(wum_source)
            return PrideRunResult(
                obs_file=obs_file, sys_attempted=sys_str,
                returncode=1, pos_path=None, log_path=None,
                error="stub",
            )

        with TemporaryDirectory() as td:
            tdp = Path(td)
            wum = tdp / "wum-cache"
            wum.mkdir()
            process_one_obs(
                tdp / "MadHat-2026140.obs",
                tdp / "work",
                sys_attempts=("GREC",),
                pdp3_runner=fake_pdp3,
                wum_source=wum,
            )
        self.assertEqual(seen, [wum])


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

"""Tests for the peppar-survey --rtklib backend.

Parallels test_peppar_survey_pride.py — same shape: parsing,
quality-filter pass-through, end-to-end orchestration with a mocked
rnx2rtkp invocation, CORS fetcher with mocked HTTP.
"""
from __future__ import annotations

import gzip
import os
import sys
import unittest
from datetime import datetime, timezone
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import mock

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.peppar_survey_rtklib import (
    RtklibPosEpoch, RtklibRunResult, RtklibSolution,
    _datetime_to_mjd,
    aggregate_solution, doy_from_obs_name, fetch_cors_rinex,
    invoke_rnx2rtkp, parse_pos, process_one_obs,
    rtklib_to_pride_solution, run_rtklib_backend, write_config,
)


# ── Helpers ────────────────────────────────────────────────────── #


def _make_pos_line(year, mo, d, h, mi, sec,
                   lat, lon, h_m, q=6, ns=12,
                   sdn=0.005, sde=0.005, sdu=0.012) -> str:
    """Format one .pos line (RTKLIB lat/lon/height format)."""
    return (f" {year:04d}/{mo:02d}/{d:02d} {h:02d}:{mi:02d}:{sec:06.3f}"
            f"   {lat:.8f}   {lon:.8f}    {h_m:.4f}"
            f"   {q}  {ns:3d}  {sdn:.4f}  {sde:.4f}  {sdu:.4f}"
            f"   0.0001  -0.0002   0.0003  0.00   0.0\n")


def _write_pos_file(path: Path, rows: list[tuple]) -> None:
    """Write a synthetic .pos file with the given epoch rows.
    Each tuple = (year, mo, d, h, mi, sec, lat, lon, h, q, ns)."""
    with open(path, "w") as f:
        f.write("% program   : RTKLIB ver.2.4.3 b34\n")
        f.write("% inp file  : rover.obs\n")
        f.write("% (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,2:float,"
                "3:sbas,4:dgps,5:single,6:ppp)\n")
        f.write("% GPST                latitude(deg) longitude(deg)   "
                "height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)  ...\n")
        for r in rows:
            f.write(_make_pos_line(*r))


# ── parse_pos ──────────────────────────────────────────────────── #


class ParsePosTest(unittest.TestCase):

    def test_parses_three_rows(self):
        with TemporaryDirectory() as td:
            p = Path(td) / "out.pos"
            _write_pos_file(p, [
                (2026, 5, 20, 14, 0, 0.0, 40.001, -90.001, 200.1, 6, 12),
                (2026, 5, 20, 14, 0, 30.0, 40.002, -90.002, 200.2, 6, 13),
                (2026, 5, 20, 14, 1, 0.0, 40.003, -90.003, 200.3, 1, 14),
            ])
            epochs = parse_pos(p)
        self.assertEqual(len(epochs), 3)
        self.assertEqual(epochs[0].q, 6)
        self.assertEqual(epochs[2].q, 1)
        self.assertAlmostEqual(epochs[0].lat, 40.001, places=6)
        self.assertEqual(epochs[1].n_sats, 13)

    def test_skips_header_and_blank(self):
        with TemporaryDirectory() as td:
            p = Path(td) / "out.pos"
            p.write_text(
                "% header line\n"
                "% another header line\n"
                "\n"
                + _make_pos_line(2026, 5, 20, 14, 0, 0.0, 40.0, -90.0, 200.0)
            )
            epochs = parse_pos(p)
        self.assertEqual(len(epochs), 1)

    def test_nonexistent_file_returns_empty(self):
        self.assertEqual(parse_pos(Path("/nonexistent/foo.pos")), [])


# ── aggregate_solution ─────────────────────────────────────────── #


def _make_epoch(unix_t, lat, lon, height, q=6, ns=12) -> RtklibPosEpoch:
    return RtklibPosEpoch(
        utc=datetime.fromtimestamp(unix_t, tz=timezone.utc),
        lat=lat, lon=lon, height=height, q=q, n_sats=ns,
        sdn=0.005, sde=0.005, sdu=0.012,
    )


class AggregateSolutionTest(unittest.TestCase):

    def test_uses_trailing_window_for_converged_estimate(self):
        # 60 epochs, 30s apart: total span = 1770s.  Default trailing
        # window = 600s.  Early epochs are noisy, trailing converges.
        epochs = []
        # Early-noise + tail-converged trajectory at placeholder coords
        for i in range(40):
            jitter = 0.0001 * (40 - i)  # decreasing scatter
            epochs.append(_make_epoch(
                1779000000 + i * 30,
                40.0 + jitter, -90.0 + jitter, 200.0 + jitter * 10,
            ))
        for i in range(20):
            # Tail: tightly clustered around (40.001, -90.001, 200.1)
            epochs.append(_make_epoch(
                1779000000 + (40 + i) * 30,
                40.001 + 0.0000001 * i, -90.001, 200.1,
            ))
        sol = aggregate_solution(epochs, mode_label="rtklib_ppp")
        self.assertIsNotNone(sol)
        # Result should sit at the tail (much closer to 40.001 than
        # the noisy-tail of the early epochs).
        self.assertAlmostEqual(sol.lat, 40.001, places=4)
        self.assertEqual(sol.mode, "rtklib_ppp")
        self.assertEqual(sol.extras["rtklib_n_epochs_total"], 60)

    def test_filters_out_bad_quality(self):
        # 20 epochs alternating q=6 (ppp, kept) and q=5 (single, dropped)
        epochs = []
        for i in range(20):
            q = 6 if i % 2 == 0 else 5
            epochs.append(_make_epoch(
                1779000000 + i * 30,
                40.0, -90.0, 200.0, q=q,
            ))
        sol = aggregate_solution(epochs, mode_label="rtklib_ppp")
        self.assertIsNotNone(sol)
        self.assertEqual(sol.n_obs, 10)  # only the q=6 kept

    def test_returns_none_when_no_quality_epochs(self):
        # All epochs are q=5 (single) — none accepted
        epochs = [_make_epoch(1779000000 + i * 30, 40.0, -90.0, 200.0, q=5)
                  for i in range(10)]
        self.assertIsNone(aggregate_solution(epochs, mode_label="rtklib_ppp"))

    def test_empty_list_returns_none(self):
        self.assertIsNone(aggregate_solution([], mode_label="rtklib_ppp"))


# ── rtklib_to_pride_solution ───────────────────────────────────── #


class AdapterTest(unittest.TestCase):

    def test_converts_to_pridesolution_shape(self):
        rtk = RtklibSolution(
            first_epoch=datetime(2026, 5, 20, 14, 0, 0, tzinfo=timezone.utc),
            lat=40.0, lon=-90.0, height=200.0,
            sigma_3d_m=0.05, sig0_m=0.05, n_obs=120,
            mode="rtklib_ppp",
        )
        adapter = rtklib_to_pride_solution(rtk, src_path="/data/host-2026140.obs")
        self.assertEqual(adapter.mode, "Static")
        self.assertEqual(adapter.n_obs, 120)
        self.assertEqual(adapter.sig0_m, 0.05)
        # ECEF should be near the WGS-84 ECEF for (40°, -90°, 200m).
        # Quick sanity: X ≈ 0 (lon=-90° puts x≈0)
        self.assertLess(abs(adapter.ecef_m[0]), 100)
        # sigma_xyz_m total magnitude should equal sigma_3d_m
        import math
        recovered_3d = math.sqrt(sum(s*s for s in adapter.sigma_xyz_m))
        self.assertAlmostEqual(recovered_3d, 0.05, places=4)


# ── invoke_rnx2rtkp with mocked subprocess ─────────────────────── #


class InvokeRnx2rtkpTest(unittest.TestCase):

    def test_ppp_mode_no_base_required(self):
        """PPP mode runs without base_obs; subprocess gets called
        with the expected cmd shape."""
        captured: dict = {}

        def fake_run(cmd, cwd, **kw):
            captured["cmd"] = cmd
            captured["cwd"] = cwd
            # Pretend rnx2rtkp succeeded — write a .pos file with one row
            pos_path = Path(cwd) / Path(cmd[cmd.index("-o") + 1]).name
            _write_pos_file(pos_path, [
                (2026, 5, 20, 14, 0, 0.0, 40.0, -90.0, 200.0, 6, 12),
            ])
            class _R: returncode = 0; stdout = ""; stderr = ""
            return _R()

        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = tdp / "rover-2026140.obs"
            obs.write_text("stub")
            with mock.patch(
                "peppar_fix.peppar_survey_rtklib.subprocess.run", fake_run
            ):
                result = invoke_rnx2rtkp(
                    obs, tdp / "work", "ppp",
                    rnx2rtkp_bin="/tmp/fake-rnx2rtkp",
                )
            self.assertEqual(result.returncode, 0)
            self.assertIsNotNone(result.pos_path)
            self.assertEqual(result.mode, "ppp")
            # Config file should have been written
            self.assertTrue((tdp / "work" / "rnx2rtkp-ppp.conf").exists())

    def test_rtk_mode_requires_base(self):
        """rtk mode without base_obs errors before subprocess fires."""
        with TemporaryDirectory() as td:
            obs = Path(td) / "rover-2026140.obs"
            obs.write_text("stub")
            result = invoke_rnx2rtkp(
                obs, Path(td) / "work", "rtk",
                rnx2rtkp_bin="/tmp/fake-rnx2rtkp",
            )
        self.assertEqual(result.returncode, -1)
        self.assertIn("base_obs", result.error)

    def test_unknown_mode_errors_cleanly(self):
        with TemporaryDirectory() as td:
            obs = Path(td) / "rover-2026140.obs"
            obs.write_text("stub")
            result = invoke_rnx2rtkp(
                obs, Path(td) / "work", "unknown",
            )
        self.assertEqual(result.returncode, -1)
        self.assertIn("unknown mode", result.error)

    def test_nonzero_exit_records_error(self):
        def fake_run(cmd, **kw):
            class _R:
                returncode = 5
                stdout = ""
                stderr = "rnx2rtkp: invalid config"
            return _R()
        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = tdp / "rover-2026140.obs"
            obs.write_text("stub")
            with mock.patch(
                "peppar_fix.peppar_survey_rtklib.subprocess.run", fake_run
            ):
                result = invoke_rnx2rtkp(
                    obs, tdp / "work", "ppp",
                    rnx2rtkp_bin="/tmp/fake-rnx2rtkp",
                )
        self.assertEqual(result.returncode, 5)
        self.assertIsNone(result.pos_path)
        self.assertIn("invalid config", result.error)


# ── CORS RINEX fetcher ─────────────────────────────────────────── #


class FetchCorsRinexTest(unittest.TestCase):

    def test_url_template_expansion(self):
        captured: dict = {}

        def fake_fetcher(url, local_gz_path):
            captured["url"] = url
            # Create a gzipped stub at the target path
            with gzip.open(local_gz_path, "wb") as f:
                f.write(b"stub RINEX contents\n")

        with TemporaryDirectory() as td:
            tdp = Path(td)
            path = fetch_cors_rinex(
                "DSP1", 2026, 140, tdp,
                fetcher=fake_fetcher,
            )
            self.assertIsNotNone(path)
            self.assertIn("/2026/140/dsp1/dsp11400.26o.gz", captured["url"])
            # Decompressed file present + gzip removed
            self.assertTrue(path.exists())
            self.assertEqual(path.read_text(), "stub RINEX contents\n")
            self.assertFalse((path.parent / (path.name + ".gz")).exists())

    def test_cached_file_returned_without_refetch(self):
        called = {"n": 0}

        def fake_fetcher(url, local_gz_path):
            called["n"] += 1

        with TemporaryDirectory() as td:
            tdp = Path(td)
            # Pre-create the expected target file
            cached = tdp / "dsp11400.26o"
            cached.write_text("already here")
            path = fetch_cors_rinex(
                "dsp1", 2026, 140, tdp, fetcher=fake_fetcher,
            )
        self.assertEqual(path, cached)
        self.assertEqual(called["n"], 0)

    def test_fetcher_failure_returns_none(self):
        def fake_fetcher(url, local_gz_path):
            raise OSError("simulated network failure")
        with TemporaryDirectory() as td:
            path = fetch_cors_rinex(
                "dsp1", 2026, 140, Path(td), fetcher=fake_fetcher,
            )
        self.assertIsNone(path)


# ── doy_from_obs_name ──────────────────────────────────────────── #


class DoyFromObsNameTest(unittest.TestCase):

    def test_yyyy_ddd_pattern(self):
        self.assertEqual(
            doy_from_obs_name(Path("MadHat-2026140.obs")),
            (2026, 140))

    def test_no_match_returns_none(self):
        self.assertIsNone(doy_from_obs_name(Path("random.obs")))


# ── process_one_obs ────────────────────────────────────────────── #


class ProcessOneObsTest(unittest.TestCase):

    def _make_runner_returning_pos(self, pos_rows):
        """Build a fake rnx2rtkp runner that writes a synthetic .pos
        file and returns a successful RtklibRunResult."""
        def fake_runner(obs_file, work_dir, mode, **kw):
            work_dir.mkdir(parents=True, exist_ok=True)
            pos_path = work_dir / (obs_file.stem + ".pos")
            _write_pos_file(pos_path, pos_rows)
            return RtklibRunResult(
                obs_file=obs_file, mode=mode,
                returncode=0, pos_path=pos_path,
            )
        return fake_runner

    def test_ppp_success_returns_solution(self):
        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = tdp / "host-2026140.obs"
            obs.write_text("stub")
            rows = [
                (2026, 5, 20, 14, 0, 0.0 + i, 40.0, -90.0, 200.0, 6, 12)
                for i in range(60)
            ]
            sol, last = process_one_obs(
                obs, tdp / "work", mode="ppp",
                rnx2rtkp_runner=self._make_runner_returning_pos(rows),
            )
        self.assertIsNotNone(sol)
        self.assertEqual(sol.mode, "rtklib_ppp")
        self.assertEqual(last.returncode, 0)

    def test_rtk_requires_cors_source(self):
        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = tdp / "host-2026140.obs"
            obs.write_text("stub")
            sol, last = process_one_obs(
                obs, tdp / "work", mode="rtk",
                rnx2rtkp_runner=self._make_runner_returning_pos([]),
            )
        self.assertIsNone(sol)
        self.assertIn("--cors-station", last.error)

    def test_rtk_with_cors_station_calls_fetcher(self):
        called = {}

        def fake_fetcher(station, year, doy, work_dir, **kw):
            called["station"] = station
            called["year_doy"] = (year, doy)
            base = work_dir / "fake-base.obs"
            work_dir.mkdir(parents=True, exist_ok=True)
            base.write_text("base stub")
            return base

        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = tdp / "host-2026140.obs"
            obs.write_text("stub")
            rows = [
                (2026, 5, 20, 14, 0, 0.0 + i, 40.0, -90.0, 200.0, 6, 12)
                for i in range(60)
            ]
            sol, last = process_one_obs(
                obs, tdp / "work", mode="rtk",
                cors_station="dsp1",
                rnx2rtkp_runner=self._make_runner_returning_pos(rows),
                cors_fetcher=fake_fetcher,
            )
        self.assertIsNotNone(sol)
        self.assertEqual(sol.mode, "rtklib_cors")
        self.assertEqual(called["station"], "dsp1")
        self.assertEqual(called["year_doy"], (2026, 140))

    def test_rtk_cors_fetch_failure(self):
        def fake_fetcher(station, year, doy, work_dir, **kw):
            return None  # simulate network failure
        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = tdp / "host-2026140.obs"
            obs.write_text("stub")
            sol, last = process_one_obs(
                obs, tdp / "work", mode="rtk",
                cors_station="dsp1",
                rnx2rtkp_runner=lambda *a, **kw: None,
                cors_fetcher=fake_fetcher,
            )
        self.assertIsNone(sol)
        self.assertIn("CORS fetch failed", last.error)

    def test_no_quality_epochs_returns_none_solution(self):
        with TemporaryDirectory() as td:
            tdp = Path(td)
            obs = tdp / "host-2026140.obs"
            obs.write_text("stub")
            # All q=5 (single point) — none accepted
            rows = [
                (2026, 5, 20, 14, 0, 0.0 + i, 40.0, -90.0, 200.0, 5, 8)
                for i in range(30)
            ]
            sol, last = process_one_obs(
                obs, tdp / "work", mode="ppp",
                rnx2rtkp_runner=self._make_runner_returning_pos(rows),
            )
        self.assertIsNone(sol)
        self.assertIsNotNone(last)
        self.assertEqual(last.returncode, 0)


# ── write_config + _datetime_to_mjd ────────────────────────────── #


class WriteConfigTest(unittest.TestCase):

    def test_ppp_config_has_ppp_posmode(self):
        with TemporaryDirectory() as td:
            cfg = write_config(Path(td), "ppp")
            content = cfg.read_text()
            self.assertIn("pos1-posmode       =ppp-static", content)

    def test_rtk_config_has_static_posmode(self):
        with TemporaryDirectory() as td:
            cfg = write_config(Path(td), "rtk")
            content = cfg.read_text()
            self.assertIn("pos1-posmode       =static", content)


class DatetimeToMjdTest(unittest.TestCase):

    def test_mjd_epoch(self):
        # MJD 0 = 1858-11-17 00:00 UTC
        self.assertEqual(
            _datetime_to_mjd(datetime(1858, 11, 17, tzinfo=timezone.utc)),
            0.0)

    def test_known_date(self):
        # MJD 61180 = 2026-05-20 00:00 UTC
        self.assertEqual(
            _datetime_to_mjd(datetime(2026, 5, 20, tzinfo=timezone.utc)),
            61180.0)


if __name__ == "__main__":
    unittest.main()

"""Tests for ARP history accumulator (I-200645-bravo)."""
from __future__ import annotations

import json
import math
import os
import sys
import unittest
from datetime import datetime, timezone
from pathlib import Path
from tempfile import TemporaryDirectory

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.arp_history import (
    DEFAULT_MAX_SIG0_M,
    DEFAULT_MIN_N_OBS,
    DEFAULT_N_DAYS,
    RunningArp,
    append_solution,
    apply_quality_filter,
    read_history,
    running_mean,
)
from peppar_fix.pride_pos_reader import PrideSolution


def _make_sol(*, mjd, ecef, sig0=4.4, n_obs=29920,
              cof_diag=1e-9, sig_xyz=None,
              ambig=True, name="ufo1") -> PrideSolution:
    """Construct a minimal PrideSolution for testing."""
    cof_xyz = (cof_diag, cof_diag, cof_diag)
    if sig_xyz is None:
        sig_xyz = tuple(sig0 * math.sqrt(c) for c in cof_xyz)
    return PrideSolution(
        name=name, mjd=mjd, ecef_m=ecef,
        cofactor_xyz=cof_xyz,
        cofactor_off=(0.0, 0.0, 0.0),
        sig0_m=sig0,
        sigma_xyz_m=sig_xyz,
        n_obs=n_obs,
        mode="Static",
        first_epoch=datetime(2026, 4, 26, tzinfo=timezone.utc),
        ambiguity_enabled=ambig,
        ambiguity_fixing={"GPS": 58, "GAL": 0},
        receiver_type="LEICA GRX1200GGPRO",
        antenna_type="SFESPK6618H     NONE",
        products={"orbit": "WUM0_orb.SP3", "clock": "WUM0_clk.CLK"},
        src_path="/tmp/pos_synthetic",
    )


class QualityFilterTest(unittest.TestCase):

    def test_healthy_passes(self):
        sol = _make_sol(mjd=61156.5, ecef=(1.57e5, -4.76e6, 4.23e6))
        self.assertTrue(apply_quality_filter(sol))

    def test_high_sig0_rejects(self):
        sol = _make_sol(mjd=61156.5, ecef=(1.57e5, -4.76e6, 4.23e6),
                        sig0=15.0)
        self.assertFalse(apply_quality_filter(sol))

    def test_low_n_obs_rejects(self):
        sol = _make_sol(mjd=61156.5, ecef=(1.57e5, -4.76e6, 4.23e6),
                        n_obs=500)
        self.assertFalse(apply_quality_filter(sol))

    def test_nan_sig0_rejects(self):
        sol = _make_sol(mjd=61156.5, ecef=(1.57e5, -4.76e6, 4.23e6))
        sol.sig0_m = float("nan")
        self.assertFalse(apply_quality_filter(sol))

    def test_zero_sig0_rejects(self):
        sol = _make_sol(mjd=61156.5, ecef=(1.57e5, -4.76e6, 4.23e6))
        sol.sig0_m = 0.0
        self.assertFalse(apply_quality_filter(sol))

    def test_custom_thresholds(self):
        # Tighten — sig0=4.4 should now reject.
        sol = _make_sol(mjd=61156.5, ecef=(1.57e5, -4.76e6, 4.23e6))
        self.assertFalse(apply_quality_filter(sol, max_sig0_m=2.0))
        self.assertFalse(apply_quality_filter(sol, min_n_obs=100_000))


class AppendAndReadTest(unittest.TestCase):

    def test_append_creates_directory_and_file(self):
        with TemporaryDirectory() as td:
            p = Path(td) / "deep" / "nested" / "history.jsonl"
            sol = _make_sol(mjd=61156.5, ecef=(1.57e5, -4.76e6, 4.23e6))
            rec = append_solution(p, sol)
            self.assertTrue(p.is_file())
            self.assertEqual(rec["mjd"], 61156.5)

    def test_round_trip(self):
        with TemporaryDirectory() as td:
            p = Path(td) / "history.jsonl"
            sol = _make_sol(mjd=61156.5, ecef=(1.57e5, -4.76e6, 4.23e6))
            append_solution(p, sol)
            recs = read_history(p)
            self.assertEqual(len(recs), 1)
            r = recs[0]
            self.assertAlmostEqual(r["ecef_m"][0], 1.57e5)
            self.assertEqual(r["mount_id"], 0)
            self.assertTrue(r["quality_ok"])
            self.assertEqual(r["name"], "ufo1")

    def test_quality_ok_recorded_when_explicit(self):
        with TemporaryDirectory() as td:
            p = Path(td) / "history.jsonl"
            sol = _make_sol(mjd=61156.5, ecef=(1.57e5, -4.76e6, 4.23e6),
                            sig0=15.0)  # would auto-fail
            rec = append_solution(p, sol, quality_ok=False)
            self.assertFalse(rec["quality_ok"])

    def test_read_history_missing_file_returns_empty(self):
        self.assertEqual(read_history("/nonexistent/foo.jsonl"), [])

    def test_read_history_skips_blank_and_malformed(self):
        with TemporaryDirectory() as td:
            p = Path(td) / "history.jsonl"
            p.write_text(
                '{"mjd": 1.0, "ecef_m":[1,2,3], "mount_id":0, '
                '"quality_ok":true, "sigma_xyz_m":[0,0,0]}\n'
                '\n'
                'this-is-not-json\n'
                '{"mjd": 2.0, "ecef_m":[4,5,6], "mount_id":0, '
                '"quality_ok":true, "sigma_xyz_m":[0,0,0]}\n'
            )
            recs = read_history(p)
            self.assertEqual([r["mjd"] for r in recs], [1.0, 2.0])


class RunningMeanTest(unittest.TestCase):
    # Synthetic 6-day OPUS-like dataset around UFO1's mean
    # to exercise cross-day std calculation.
    _BASE_ECEF = (157470.222, -4756189.544, 4232767.952)

    def _seed_history(self, td, offsets, n_days_offset_start=0):
        """Append solutions at mjd_base+i with +offsets[i] mm in each
        ECEF axis."""
        p = Path(td) / "history.jsonl"
        for i, off_mm in enumerate(offsets):
            sol = _make_sol(
                mjd=61156.0 + i + n_days_offset_start,
                ecef=(self._BASE_ECEF[0] + off_mm * 1e-3,
                      self._BASE_ECEF[1] + off_mm * 1e-3,
                      self._BASE_ECEF[2] + off_mm * 1e-3),
            )
            append_solution(p, sol)
        return p

    def test_empty_history_returns_none(self):
        with TemporaryDirectory() as td:
            self.assertIsNone(running_mean(Path(td) / "h.jsonl"))

    def test_single_solution_uses_formal_sigma(self):
        with TemporaryDirectory() as td:
            p = self._seed_history(td, [0.0])
            r = running_mean(p)
            self.assertIsNotNone(r)
            self.assertEqual(r.count, 1)
            # Formal σ on the single solution: sig0=4.4 × sqrt(1e-9) ≈ 1.4e-4 m
            for s in r.sigma_xyz_m:
                self.assertAlmostEqual(s, 4.4 * math.sqrt(1e-9), places=10)

    def test_six_day_mean_matches_expectation(self):
        # Six small offsets centered at zero.
        offsets = [-5.0, -3.0, 0.0, 0.0, 3.0, 5.0]  # mm
        with TemporaryDirectory() as td:
            p = self._seed_history(td, offsets)
            r = running_mean(p, n_days=7)
            self.assertEqual(r.count, 6)
            # Mean of offsets ≈ 0 (sum zero).
            for axis in range(3):
                self.assertAlmostEqual(r.ecef_m[axis],
                                       self._BASE_ECEF[axis], places=6)
            # Cross-day σ — sample-std of [-5,-3,0,0,3,5] mm = ~3.69 mm
            expected_sigma_mm = 3.69
            for s_m in r.sigma_xyz_m:
                self.assertAlmostEqual(s_m * 1000, expected_sigma_mm,
                                       places=1)

    def test_n_days_window_drops_old(self):
        offsets = [-100.0, -50.0, 0.0, 0.0, 50.0, 100.0]
        with TemporaryDirectory() as td:
            p = self._seed_history(td, offsets)
            # 3-day window keeps the last 3 (zero, +50, +100 mm).
            r = running_mean(p, n_days=3)
            self.assertEqual(r.count, 3)
            # Mean of last 3 offsets = +50 mm.
            for axis in range(3):
                self.assertAlmostEqual(
                    r.ecef_m[axis] - self._BASE_ECEF[axis],
                    0.050, places=6)

    def test_mount_id_partition(self):
        with TemporaryDirectory() as td:
            p = Path(td) / "h.jsonl"
            # mount 0: tight cluster.
            for i in range(3):
                append_solution(p, _make_sol(
                    mjd=61156.0 + i,
                    ecef=(self._BASE_ECEF[0], self._BASE_ECEF[1], self._BASE_ECEF[2]),
                ), mount_id=0)
            # mount 1: 100 m east shift.
            for i in range(3):
                append_solution(p, _make_sol(
                    mjd=61159.0 + i,
                    ecef=(self._BASE_ECEF[0] + 100.0,
                          self._BASE_ECEF[1], self._BASE_ECEF[2]),
                ), mount_id=1)
            r0 = running_mean(p, mount_id=0)
            r1 = running_mean(p, mount_id=1)
            self.assertEqual(r0.count, 3)
            self.assertEqual(r1.count, 3)
            self.assertAlmostEqual(r0.ecef_m[0], self._BASE_ECEF[0])
            self.assertAlmostEqual(r1.ecef_m[0], self._BASE_ECEF[0] + 100.0)

    def test_quality_filter_excludes_failed_runs(self):
        with TemporaryDirectory() as td:
            p = Path(td) / "h.jsonl"
            # Two healthy solutions.
            append_solution(p, _make_sol(mjd=61156.0,
                            ecef=(self._BASE_ECEF[0],
                                  self._BASE_ECEF[1],
                                  self._BASE_ECEF[2])))
            append_solution(p, _make_sol(mjd=61157.0,
                            ecef=(self._BASE_ECEF[0] + 0.001,
                                  self._BASE_ECEF[1] + 0.001,
                                  self._BASE_ECEF[2] + 0.001)))
            # One failure (position 1km off, marked bad).
            bad = _make_sol(mjd=61158.0,
                            ecef=(self._BASE_ECEF[0] + 1000.0,
                                  self._BASE_ECEF[1],
                                  self._BASE_ECEF[2]),
                            sig0=15.0)
            append_solution(p, bad)
            r = running_mean(p)
            # The 1km-off solution must be excluded from the mean.
            self.assertEqual(r.count, 2)
            for axis in range(3):
                self.assertLess(
                    abs(r.ecef_m[axis] - self._BASE_ECEF[axis]), 0.005)

    def test_sigma_3d_is_rss_of_per_axis(self):
        with TemporaryDirectory() as td:
            p = self._seed_history(td, [-5.0, 0.0, 5.0])
            r = running_mean(p)
            self.assertAlmostEqual(
                r.sigma_3d_m,
                math.sqrt(sum(s * s for s in r.sigma_xyz_m)),
                places=10,
            )


if __name__ == "__main__":
    unittest.main()

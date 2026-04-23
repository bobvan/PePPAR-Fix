"""Smoke tests for the regression runner.

Confirms the runner is importable, has correct argparse plumbing,
and can be exercised with `--help`.  End-to-end execution with real
data is gated on PRIDE_DATA_DIR + REGRESSION_NAV env vars (a NAV
file isn't bundled with PRIDE-PPPAR — it has to be downloaded
separately, see scripts/regression/README.md).
"""

from __future__ import annotations

import os
import subprocess
import sys
import unittest
from pathlib import Path


_RUNNER = Path("scripts/regression/run_regression.py")


class HelpSmokeTest(unittest.TestCase):
    """`--help` exits cleanly with usage text — confirms imports work."""

    def test_help(self):
        env = dict(os.environ)
        env["PYTHONPATH"] = "scripts"
        result = subprocess.run(
            [sys.executable, str(_RUNNER), "--help"],
            env=env, capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0,
                         f"--help failed: {result.stderr}")
        self.assertIn("--obs", result.stdout)
        self.assertIn("--nav", result.stdout)
        self.assertIn("--truth", result.stdout)
        self.assertIn("--profile", result.stdout)


# Optional end-to-end integration: requires PRIDE bundled OBS plus a
# user-downloaded NAV file (see README.md for download instructions).
@unittest.skipIf(
    not (os.environ.get("PRIDE_DATA_DIR") and os.environ.get("REGRESSION_NAV")),
    "set PRIDE_DATA_DIR and REGRESSION_NAV to run the end-to-end test"
)
class EndToEndIntegrationTest(unittest.TestCase):
    def test_abmf_2020_001_first_50_epochs(self):
        obs = Path(os.environ["PRIDE_DATA_DIR"]) / "2020/001/abmf0010.20o"
        nav = Path(os.environ["REGRESSION_NAV"])
        if not obs.exists():
            self.skipTest(f"{obs} not found")
        if not nav.exists():
            self.skipTest(f"{nav} not found")

        env = dict(os.environ)
        env["PYTHONPATH"] = "scripts"
        result = subprocess.run(
            [sys.executable, str(_RUNNER),
             "--obs", str(obs),
             "--nav", str(nav),
             "--truth", "2919785.79086,-5383744.95943,1774604.85992",
             "--tolerance-m", "20",        # very loose: 50 epochs of
                                            # 30 s = 25 min, float-PPP
                                            # converges slowly with
                                            # broadcast orbits only
             "--max-epochs", "50",
             "--profile", "l5"],
            env=env, capture_output=True, text=True, timeout=180,
        )
        # We don't assert PASS here — float-PPP at 25 min with broadcast
        # orbits could easily be 5-20 m off.  We just assert the runner
        # ran to completion without crashing.
        self.assertIn("Regression result", result.stdout,
                      f"runner produced no result block:\n{result.stdout}\n"
                      f"stderr:\n{result.stderr}")
        self.assertIn("Final error 3D:", result.stdout)


class ProfileParserTest(unittest.TestCase):
    """_parse_profile handles uniform and per-constellation forms."""

    def setUp(self):
        # Import lazily so the subprocess tests above don't suffer an
        # import cost when only they run.
        sys.path.insert(0, "scripts")
        from regression.run_regression import _parse_profile
        from regression.rinex_reader import L5_PROFILE, L2_PROFILE
        self._parse = _parse_profile
        self._L5 = L5_PROFILE
        self._L2 = L2_PROFILE

    def test_uniform_l5(self):
        p = self._parse("l5")
        self.assertEqual(p, self._L5)

    def test_uniform_l2(self):
        p = self._parse("l2")
        self.assertEqual(p, self._L2)

    def test_uniform_whitespace_and_case(self):
        p = self._parse("  L5  ")
        self.assertEqual(p, self._L5)

    def test_per_constellation_gps_l2_gal_l5(self):
        p = self._parse("gps:l2,gal:l5")
        self.assertEqual(p["GPS"], self._L2["GPS"])
        self.assertEqual(p["GAL"], self._L5["GAL"])
        self.assertNotIn("BDS", p)

    def test_per_constellation_three_systems(self):
        p = self._parse("gps:l2,gal:l5,bds:l5")
        self.assertEqual(p["GPS"], self._L2["GPS"])
        self.assertEqual(p["GAL"], self._L5["GAL"])
        self.assertEqual(p["BDS"], self._L5["BDS"])

    def test_unknown_system_rejected(self):
        with self.assertRaisesRegex(ValueError, "unknown system"):
            self._parse("xyz:l5")

    def test_unknown_profile_rejected(self):
        with self.assertRaisesRegex(ValueError, "unknown profile"):
            self._parse("gps:l9")

    def test_missing_colon_rejected(self):
        with self.assertRaisesRegex(ValueError, "expected 'l5', 'l2'"):
            self._parse("gpsl2")

    def test_empty_rejected(self):
        with self.assertRaisesRegex(ValueError, "empty --profile"):
            self._parse("")


class RankDeficiencyGateTest(unittest.TestCase):
    """Verify the single-constellation-with-SP3 gate refuses to run.

    Rationale: at ABMF the (rx_clock, ZTD·m_wet, mean-ambiguity)
    triple forms a near-null vector under smooth precise clocks.
    GPS-only with SP3+CLK drifts 15+ m over 50 min while GPS+GAL
    stays bounded.  See
    project_to_main_pride_gps_filter_degeneracy_20260423.
    """

    def test_gal_only_plus_sp3_rejects(self):
        """With --sp3 and a single constellation, runner exits 2."""
        env = dict(os.environ)
        env["PYTHONPATH"] = "scripts"
        pride = os.environ.get("PRIDE_DATA_DIR")
        sp3 = os.environ.get("REGRESSION_SP3")
        if not (pride and sp3):
            self.skipTest("set PRIDE_DATA_DIR and REGRESSION_SP3 "
                          "to run the gate test")
        obs = Path(pride) / "2020/001/abmf0010.20o"
        if not obs.exists():
            self.skipTest(f"{obs} not found")
        if not Path(sp3).exists():
            self.skipTest(f"{sp3} not found")

        result = subprocess.run(
            [sys.executable, str(_RUNNER),
             "--obs", str(obs),
             "--sp3", str(sp3),
             "--truth", "2919785.79086,-5383744.95943,1774604.85992",
             "--profile", "l5",
             "--systems", "gal",
             "--tolerance-m", "1",
             "--max-epochs", "10"],
            env=env, capture_output=True, text=True, timeout=60,
        )
        self.assertEqual(result.returncode, 2,
                         f"expected exit 2; stdout:\n{result.stdout}\n"
                         f"stderr:\n{result.stderr}")
        self.assertIn("rank-deficient", result.stderr,
                      "gate error message should mention rank-deficient")
        self.assertIn("project_to_main_pride_gps_filter_degeneracy",
                      result.stderr,
                      "gate error should reference the memo")

    def test_nav_single_constellation_allowed(self):
        """With --nav (not --sp3), single-constellation is allowed.

        NAV's 2-hour polynomial discontinuities inject the rank
        information that smooth SP3 clocks can't, so GPS-only-NAV
        is a legitimate diagnostic configuration.
        """
        env = dict(os.environ)
        env["PYTHONPATH"] = "scripts"
        pride = os.environ.get("PRIDE_DATA_DIR")
        nav = os.environ.get("REGRESSION_NAV")
        if not (pride and nav):
            self.skipTest("set PRIDE_DATA_DIR and REGRESSION_NAV "
                          "to run the NAV-path check")
        obs = Path(pride) / "2020/001/abmf0010.20o"
        if not obs.exists() or not Path(nav).exists():
            self.skipTest(f"bundled data missing ({obs} or {nav})")

        result = subprocess.run(
            [sys.executable, str(_RUNNER),
             "--obs", str(obs),
             "--nav", str(nav),
             "--truth", "2919785.79086,-5383744.95943,1774604.85992",
             "--profile", "l5",
             "--systems", "gal",
             "--tolerance-m", "100",
             "--max-epochs", "10"],
            env=env, capture_output=True, text=True, timeout=60,
        )
        # Gate does not fire on --nav — the runner proceeds and
        # emits its standard result block.
        self.assertIn("Regression result", result.stdout,
                      f"runner should have proceeded past the gate.  "
                      f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}")


if __name__ == "__main__":
    unittest.main()

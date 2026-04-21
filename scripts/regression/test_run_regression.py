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


if __name__ == "__main__":
    unittest.main()

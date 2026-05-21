"""Tests for r_calibration: per-host R-matrix model."""
from __future__ import annotations

import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.r_calibration import RCalibration, _default_sigma_pr_m, _default_sigma_td_m


class DefaultsTest(unittest.TestCase):
    """No TOML → falls back to legacy formulas."""

    def test_default_sigma_pr_at_zenith(self):
        c = RCalibration()
        # Zenith: 3.0/sin(90°) = 3.0 m (matches legacy SIGMA_P_IF/elev_factor)
        self.assertAlmostEqual(c.sigma_pr_m('gps', 90.0), 3.0, places=2)

    def test_default_sigma_pr_at_low_elev(self):
        c = RCalibration()
        # 15°: 3.0/sin(15°) ≈ 11.6 m
        s = c.sigma_pr_m('gps', 15.0)
        self.assertAlmostEqual(s, 3.0 / math.sin(math.radians(15.0)),
                               places=2)

    def test_default_sigma_td_at_zenith(self):
        c = RCalibration()
        # 0.3 / sin(90°) = 0.3
        self.assertAlmostEqual(c.sigma_td_m('gps', 90.0), 0.3, places=4)

    def test_default_sigma_td_at_low_elev_clamps(self):
        c = RCalibration()
        # max(0.2, sin(5°)=0.087) = 0.2 → 0.3/0.2 = 1.5
        self.assertAlmostEqual(c.sigma_td_m('gps', 5.0), 1.5, places=4)


class TomlLoadTest(unittest.TestCase):
    """from_toml() loads per-system models; unspecified systems fall back."""

    def _write_toml(self, content):
        tf = tempfile.NamedTemporaryFile(mode='w', suffix='.toml',
                                          delete=False)
        tf.write(content)
        tf.close()
        return Path(tf.name)

    def test_loads_gps_model_and_uses_it(self):
        path = self._write_toml("""
[gps]
elev_bins_deg = [10, 20, 30, 40, 50, 60, 70, 90]
sigma_pr_m = [2.50, 2.00, 1.60, 1.20, 0.80, 0.71, 0.71]
sigma_td_m = [0.019, 0.047, 0.024, 0.025, 0.015, 0.007, 0.007]
""")
        try:
            c = RCalibration.from_toml(path)
            # 65° is in bin 60-70: sigma_pr=0.71, sigma_td=0.007
            self.assertAlmostEqual(c.sigma_pr_m('gps', 65.0), 0.71, places=4)
            self.assertAlmostEqual(c.sigma_td_m('gps', 65.0), 0.007, places=4)
            # 15° is in bin 10-20: sigma_pr=2.50, sigma_td=0.019
            self.assertAlmostEqual(c.sigma_pr_m('gps', 15.0), 2.50, places=4)
            self.assertAlmostEqual(c.sigma_td_m('gps', 15.0), 0.019, places=4)
        finally:
            path.unlink()

    def test_missing_system_falls_back_to_defaults(self):
        path = self._write_toml("""
[gps]
elev_bins_deg = [10, 90]
sigma_pr_m = [1.0]
sigma_td_m = [0.01]
""")
        try:
            c = RCalibration.from_toml(path)
            # GAL not in TOML → default applies
            self.assertAlmostEqual(c.sigma_pr_m('gal', 90.0), 3.0, places=2)
            self.assertAlmostEqual(c.sigma_td_m('gal', 90.0), 0.3, places=4)
            # GPS present → uses the calibrated value
            self.assertAlmostEqual(c.sigma_pr_m('gps', 45.0), 1.0, places=4)
        finally:
            path.unlink()

    def test_floor_clamps_unreasonably_small_calibration(self):
        path = self._write_toml("""
pr_floor_m = 0.10
td_floor_m = 0.005
[gps]
elev_bins_deg = [10, 90]
sigma_pr_m = [0.001]
sigma_td_m = [0.0001]
""")
        try:
            c = RCalibration.from_toml(path)
            self.assertEqual(c.sigma_pr_m('gps', 45.0), 0.10)
            self.assertEqual(c.sigma_td_m('gps', 45.0), 0.005)
        finally:
            path.unlink()

    def test_zero_bin_falls_back_to_default(self):
        """A bin with 0.0 sigma (e.g. insufficient data) → use default."""
        path = self._write_toml("""
[gps]
elev_bins_deg = [10, 20, 30, 90]
sigma_pr_m = [1.0, 0.0, 0.5]
sigma_td_m = [0.01, 0.02, 0.005]
""")
        try:
            c = RCalibration.from_toml(path)
            # 25° → bin 20-30 → sigma_pr=0 → default for that elev
            self.assertAlmostEqual(c.sigma_pr_m('gps', 25.0),
                                    _default_sigma_pr_m(25.0), places=2)
            # 15° → sigma_pr=1.0
            self.assertAlmostEqual(c.sigma_pr_m('gps', 15.0), 1.0, places=4)
        finally:
            path.unlink()


class BoundariesTest(unittest.TestCase):

    def test_elev_above_top_bin_uses_top_bin(self):
        c = RCalibration.from_toml(self._make())
        # 80° is in [70,90) → sigma_pr=0.5
        self.assertAlmostEqual(c.sigma_pr_m('gps', 80.0), 0.5, places=4)

    def test_elev_exactly_at_top_edge_uses_top_bin(self):
        c = RCalibration.from_toml(self._make())
        self.assertAlmostEqual(c.sigma_pr_m('gps', 90.0), 0.5, places=4)

    def _make(self):
        import tempfile
        tf = tempfile.NamedTemporaryFile(mode='w', suffix='.toml',
                                          delete=False)
        tf.write("""
[gps]
elev_bins_deg = [10, 50, 90]
sigma_pr_m = [1.5, 0.5]
sigma_td_m = [0.02, 0.005]
""")
        tf.close()
        self.addCleanup(lambda: Path(tf.name).unlink(missing_ok=True))
        return Path(tf.name)


if __name__ == "__main__":
    unittest.main()

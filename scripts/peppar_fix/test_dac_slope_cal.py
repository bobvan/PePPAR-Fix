"""Tests for dac_slope_cal: linear-region detection + state-file merge."""
from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from dac_slope_cal import detect_linear_region  # noqa: E402


# MadHat IsoTemp OCXO-33 measured cal (code → ppb), saturates -26.5
# ppb above code ~45000.
_MADHAT = [
    {'code': 5000,  'ppb': 397.5, 'n_pairs': 32},
    {'code': 10000, 'ppb': 338.0, 'n_pairs': 59},
    {'code': 15000, 'ppb': 280.2, 'n_pairs': 60},
    {'code': 20000, 'ppb': 223.9, 'n_pairs': 59},
    {'code': 25000, 'ppb': 168.6, 'n_pairs': 59},
    {'code': 30000, 'ppb': 114.4, 'n_pairs': 60},
    {'code': 32768, 'ppb': 84.7,  'n_pairs': 59},
    {'code': 35000, 'ppb': 60.8,  'n_pairs': 59},
    {'code': 40000, 'ppb': 7.5,   'n_pairs': 60},
    {'code': 45000, 'ppb': -26.5, 'n_pairs': 59},
    {'code': 50000, 'ppb': -26.6, 'n_pairs': 60},
    {'code': 55000, 'ppb': -26.5, 'n_pairs': 59},
    {'code': 60000, 'ppb': -26.5, 'n_pairs': 60},
]


class DetectLinearRegionTest(unittest.TestCase):

    def test_excludes_saturated_tail(self):
        fit = detect_linear_region(_MADHAT, center_code=32768)
        self.assertIsNotNone(fit)
        # Linear region should stop before the saturated points (45000+)
        self.assertLessEqual(fit['code_max'], 42000)
        self.assertEqual(fit['code_min'], 5000)
        # Saturated points excluded
        self.assertGreaterEqual(_MADHAT.__len__() - fit['n_linear'], 3)

    def test_recovers_true_slope(self):
        """True linear slope ~-0.01114, not the -0.008 a full-range fit
        gives by averaging in the flat saturated tail."""
        fit = detect_linear_region(_MADHAT, center_code=32768)
        self.assertAlmostEqual(fit['slope'], -0.01114, delta=0.0005)

    def test_clean_linear_data_uses_all_points(self):
        """No saturation → all points in the region."""
        clean = [{'code': c, 'ppb': (c - 32768) * -0.01, 'n_pairs': 30}
                 for c in range(10000, 55000, 5000)]
        fit = detect_linear_region(clean, center_code=32768)
        self.assertEqual(fit['n_linear'], len(clean))

    def test_emits_ppb_at_code_min(self):
        """noMagicCenterCode: the fit reports the edge-anchored pull, and it
        is independent of the (arbitrary) center_code passed in."""
        clean = [{'code': c, 'ppb': (c - 32768) * -0.01, 'n_pairs': 30}
                 for c in range(10000, 55000, 5000)]
        fit = detect_linear_region(clean, center_code=32768)
        # line is ppb = -0.01·(code - 32768) → at code_min=10000: +227.68
        self.assertAlmostEqual(fit['ppb_at_code_min'],
                               -0.01 * (fit['code_min'] - 32768), places=3)
        # ppb_at_code_min must not depend on where the intercept was anchored:
        fit2 = detect_linear_region(clean, center_code=0)
        self.assertAlmostEqual(fit['ppb_at_code_min'],
                               fit2['ppb_at_code_min'], places=6)

    def test_flat_data_returns_none(self):
        """DAC not steering the DO → all-flat → no region."""
        flat = [{'code': c, 'ppb': 5.0, 'n_pairs': 30}
                for c in range(10000, 55000, 5000)]
        self.assertIsNone(detect_linear_region(flat, center_code=32768))


class StateFileMergeTest(unittest.TestCase):
    """The cal must preserve a pre-existing 'characterization' section
    (the freerun floor from do_freerun_char.py)."""

    def test_merge_preserves_characterization(self):
        # Simulate the read-modify-write the cal performs.
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'do.json')
            # Pre-existing state with a freerun characterization
            with open(path, 'w') as f:
                json.dump({
                    'unique_id': 'test-do',
                    'characterization': {
                        'captured': '2026-05-27T16:52:00Z',
                        'sources': {'DO PPS (chA vs TICC Rb)': {
                            'tdev_ns_by_tau_s': {'1.0': 0.047}}},
                    },
                }, f)
            # The merge logic from dac_slope_cal main()
            existing = json.loads(open(path).read())
            existing.update({
                'dac_ppb_per_code': -0.01114,
                'dac_code_min': 5000,
                'dac_code_max': 42000,
            })
            with open(path, 'w') as f:
                json.dump(existing, f)
            # Verify both sections present
            result = json.loads(open(path).read())
            self.assertIn('characterization', result)
            self.assertEqual(
                result['characterization']['sources'][
                    'DO PPS (chA vs TICC Rb)']['tdev_ns_by_tau_s']['1.0'],
                0.047)
            self.assertEqual(result['dac_code_min'], 5000)


if __name__ == "__main__":
    unittest.main()

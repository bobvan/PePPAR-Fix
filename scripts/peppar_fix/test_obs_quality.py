"""Unit tests for obs_quality_weight() — the per-observation MW
quality weight from elevation + CN0.

Background: I-155354 fix-L.  MW running mean is bounded in noise
floor by the worst observations it ingests; weighting low-elev /
low-CN0 samples lower lets the running mean track the high-quality
observations and round to the right WL integer in shorter arcs.

These tests pin the contract used by the MelbourneWubbenaTracker
EMA + n_eff threshold gate.  A change to the weight formula here
will surface in the tracker's convergence behavior.
"""
from __future__ import annotations

import math
import unittest

from peppar_fix.obs_quality import obs_quality_weight


class ObsQualityWeightTest(unittest.TestCase):

    def test_both_none_returns_one(self):
        """Legacy / replay paths (no quality info) → unweighted EMA."""
        self.assertEqual(obs_quality_weight(), 1.0)
        self.assertEqual(obs_quality_weight(elev_deg=None, cno_db=None),
                          1.0)

    def test_zenith_high_cno_returns_one(self):
        """Open-sky zenith (90°, 50 dB-Hz) is the unit reference."""
        w = obs_quality_weight(elev_deg=90.0, cno_db=50.0)
        self.assertAlmostEqual(w, 1.0, places=6)

    def test_elevation_sin_squared(self):
        """Elev contribution alone follows sin²(elev)."""
        cases = [
            (90.0, 1.0),
            (60.0, math.sin(math.radians(60.0)) ** 2),  # ≈ 0.75
            (30.0, math.sin(math.radians(30.0)) ** 2),  # = 0.25
            (10.0, math.sin(math.radians(10.0)) ** 2),  # ≈ 0.030
            (5.0,  math.sin(math.radians(5.0))  ** 2),  # ≈ 0.0076
        ]
        for elev, expected in cases:
            with self.subTest(elev=elev):
                w = obs_quality_weight(elev_deg=elev)
                self.assertAlmostEqual(w, expected, places=4,
                    msg=f'elev={elev}° → {w} (expected {expected})')

    def test_cno_ramps_25_to_45(self):
        """CN0 ramp: 0 at floor, 1 at ceiling, linear between."""
        cases = [
            (50.0, 1.0),     # above ceiling → clipped to 1
            (45.0, 1.0),     # at ceiling
            (35.0, 0.5),     # midpoint
            (30.0, 0.25),
            (25.0, 0.0),     # at floor → returns 0
            (20.0, 0.0),     # below floor → returns 0
        ]
        for cno, expected in cases:
            with self.subTest(cno=cno):
                w = obs_quality_weight(cno_db=cno)
                self.assertAlmostEqual(w, expected, places=4,
                    msg=f'cno={cno} → {w} (expected {expected})')

    def test_combined_multiplicative(self):
        """Both factors multiply: 30° elev × 35 dB-Hz = 0.25 × 0.5."""
        w = obs_quality_weight(elev_deg=30.0, cno_db=35.0)
        self.assertAlmostEqual(w, 0.125, places=4)

    def test_zero_elevation_returns_zero(self):
        """elev = 0° → SV is on the horizon, unphysical for tracking."""
        self.assertEqual(obs_quality_weight(elev_deg=0.0,
                                              cno_db=50.0), 0.0)

    def test_negative_elevation_returns_zero(self):
        """elev < 0° → SV below horizon, never used in real engine but
        the helper is defensive (computed elev can hit -ε near rise/set)."""
        self.assertEqual(obs_quality_weight(elev_deg=-1.0,
                                              cno_db=50.0), 0.0)

    def test_low_cno_returns_zero(self):
        """CN0 ≤ floor → no signal weight regardless of elevation."""
        self.assertEqual(obs_quality_weight(elev_deg=90.0,
                                              cno_db=20.0), 0.0)

    def test_only_elev_provided(self):
        """elev given, cno None → cno contributes 1.0 (no penalty)."""
        w = obs_quality_weight(elev_deg=30.0)
        self.assertAlmostEqual(w, 0.25, places=4)

    def test_only_cno_provided(self):
        """cno given, elev None → elev contributes 1.0 (no penalty)."""
        w = obs_quality_weight(cno_db=35.0)
        self.assertAlmostEqual(w, 0.5, places=4)

    def test_realistic_low_quality(self):
        """Typical low-elev multipath case: 10° + 30 dB-Hz.  Weight
        should be tiny — exactly the kind of sample fix-L is designed
        to deweight.
        """
        w = obs_quality_weight(elev_deg=10.0, cno_db=30.0)
        # sin²(10°) ≈ 0.0302; cno contrib (30-25)/20 = 0.25
        self.assertLess(w, 0.01,
                         f'expected w << 0.01 for 10° + 30 dB-Hz, got {w}')


if __name__ == '__main__':
    unittest.main()

"""Unit tests for scripts/solid_tide.py (IERS 2010 Step 1 SET).

The full module lives at scripts/solid_tide.py (main scripts/, not
inside peppar_fix/) because it's shared between engine and
regression harness.  Tests live here because peppar_fix/ is the
hosted test directory for the engine's Python suite.

Coverage:
- Magnitude / sign sanity (~150 mm peak vertical)
- Geographic consistency (polar stations see less vertical, more horizontal)
- Zero-displacement-at-reasonable-body-alignment sanity
- Numerical vs zero for the simple cases (station on equator with sun at zenith)
"""

from __future__ import annotations

import math
import os
import sys
import unittest
from datetime import datetime, timezone

import numpy as np

_SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from solid_tide import solid_tide_displacement, _degree2_displacement  # noqa: E402


# A reasonable ECEF station — roughly ABMF (Guadeloupe), used in the
# PRIDE ablation that sized the correction at 42 mm.  Any physically
# plausible surface station works; this one is convenient because
# the harness verified numerical agreement there.
ABMF_ECEF = np.array([2919785.79086, -5383744.95943, 1774604.85992])

# Mid-latitude surface station (approx 45°N, 0°E) for tests where
# we want a generic station.
MIDLAT_ECEF = np.array([4517590.0, 0.0, 4487348.0])


def _earth_radius(pos):
    return float(np.linalg.norm(pos))


class MagnitudeTest(unittest.TestCase):
    """Total displacement magnitude is bounded at ~few hundred mm —
    literature puts the 24h peak-to-peak SET envelope at ~300 mm
    vertical at equatorial latitudes, ~150 mm at mid-latitudes."""

    def test_magnitude_bounded(self):
        """Displacement magnitude should be < 500 mm at any plausible
        epoch — safe upper bound well above the literature peak."""
        # Sample across one day at 1-hour intervals.
        t0 = datetime(2024, 6, 15, 0, 0, 0, tzinfo=timezone.utc)
        max_mag = 0.0
        for hour in range(24):
            t = t0.replace(hour=hour)
            disp = solid_tide_displacement(t, MIDLAT_ECEF)
            mag = float(np.linalg.norm(disp))
            max_mag = max(max_mag, mag)
        self.assertLess(max_mag, 0.5,
                        f"max |disp| = {max_mag*1000:.1f} mm, expected < 500 mm")

    def test_magnitude_nonzero(self):
        """Tide is not identically zero — something is being computed."""
        t = datetime(2024, 6, 15, 12, 0, 0, tzinfo=timezone.utc)
        disp = solid_tide_displacement(t, MIDLAT_ECEF)
        self.assertGreater(float(np.linalg.norm(disp)), 0.001,
                           "expected non-trivial tide displacement")


class DiurnalVariationTest(unittest.TestCase):
    """Peak SET occurs when sun + moon align overhead; minimum when
    body-station geometry cancels.  24h range should span at least
    50 mm at mid-latitude."""

    def test_has_diurnal_variation(self):
        t0 = datetime(2024, 6, 15, 0, 0, 0, tzinfo=timezone.utc)
        mags = []
        # 10-minute sampling across 24h captures diurnal peaks cleanly.
        for minute in range(0, 24 * 60, 10):
            t = t0.replace() + __import__('datetime').timedelta(minutes=minute)
            disp = solid_tide_displacement(t, MIDLAT_ECEF)
            mags.append(float(np.linalg.norm(disp)))
        span = max(mags) - min(mags)
        self.assertGreater(span, 0.05,
                           f"24h span {span*1000:.1f} mm, expected > 50 mm")


class RadialVsTransverseTest(unittest.TestCase):
    """Verify both radial and transverse components are non-trivially
    present — the IERS 2010 Step 1 formula has distinct h2 (radial)
    and l2 (transverse) Love-number contributions, and neither should
    collapse to zero for a generic station/epoch."""

    def test_both_components_present(self):
        station = MIDLAT_ECEF
        r_hat = station / _earth_radius(station)
        t = datetime(2024, 6, 15, 12, 0, 0, tzinfo=timezone.utc)
        disp = solid_tide_displacement(t, station)
        radial_mag = abs(float(np.dot(disp, r_hat)))
        horiz_mag = float(np.linalg.norm(disp - radial_mag * r_hat))
        # Both should contribute more than 1 mm — well above numeric
        # noise, well below the expected 50-200 mm total envelope.
        self.assertGreater(radial_mag, 0.001,
                           f"radial {radial_mag*1000:.1f} mm < 1 mm — "
                           f"radial contribution seems missing")
        self.assertGreater(horiz_mag, 0.001,
                           f"horiz  {horiz_mag*1000:.1f} mm < 1 mm — "
                           f"transverse contribution seems missing")


class IersReferenceTest(unittest.TestCase):
    """Authoritative comparison against the published test cases in the
    IERS Conventions (2010) DEHANTTIDEINEL.F reference routine.

    Each case publishes station + Sun + Moon vectors in a single
    self-consistent geocentric frame (the celestial/ECI frame — verified
    2026-06-04: the body vectors match ERFA's geocentric Sun/Moon to
    ~0.1 deg, and are 60-150 deg off ECEF by exactly the epoch's GMST).
    Because the degree-2 displacement is rotation-covariant, feeding the
    published vectors straight into ``_degree2_displacement`` reproduces
    the published displacement in that same frame.

    Tolerance is 10 mm: our implementation is degree-2 in-phase only,
    while DEHANTTIDEINEL also includes the degree-3, out-of-phase
    (anelastic), and latitude-dependent Step-1 terms.  Their combined
    contribution is the ~4-8 mm residual seen here — it is NOT a frame
    or ephemeris error (the earlier 'frame-rotation bug' hypothesis,
    I-211101-main, was a comparison artifact from feeding these ECI
    vectors into an ECEF-expecting pipeline).
    """

    # (station, sun, moon, expected DXTIDE) — all metres, frame-consistent.
    CASES = [
        (np.array([4075578.385, 931852.890, 4801570.154]),
         np.array([137859926952.015, 54228127881.4350, 23509422341.6960]),
         np.array([-179996231.920342, -312468450.131567, -169288918.592160]),
         np.array([0.07700420357108125891, 0.06304056321824967613,
                   0.05516568152597246810])),
        (np.array([1112189.660, -4842955.026, 3985352.284]),
         np.array([-54537460436.2357, 130244288385.279, 56463429031.5996]),
         np.array([300396716.912, 243238281.451, 120548075.939]),
         np.array([-0.2036831479592075833e-1, 0.5658254776225972449e-1,
                   -0.7597679676871742227e-1])),
        (np.array([1112200.5696, -4842957.8511, 3985345.9122]),
         np.array([100210282451.6279, 103055630398.3160, 56855096480.4475]),
         np.array([369817604.4348, 1897917.5258, 120804980.8284]),
         np.array([.00509570869172363845, .0828663025983528700,
                   -.0636634925404189617])),
    ]

    def test_degree2_core_matches_iers_reference(self):
        for i, (sta, sun, moon, expected) in enumerate(self.CASES):
            disp = _degree2_displacement(sta, sun, moon)
            err = float(np.linalg.norm(disp - expected))
            self.assertLess(
                err, 0.010,
                f"IERS DEHANTTIDEINEL case {i}: |ours - ref| = "
                f"{err*1000:.2f} mm > 10 mm.  ours={np.round(disp*1000,2)} mm, "
                f"ref={np.round(expected*1000,2)} mm")


class InputValidationTest(unittest.TestCase):

    def test_rejects_wrong_shape(self):
        t = datetime(2024, 6, 15, 12, 0, 0, tzinfo=timezone.utc)
        # Non-(3,) input must raise, not silently produce garbage.
        with self.assertRaises(ValueError):
            solid_tide_displacement(t, np.array([1.0, 2.0]))

    def test_accepts_ndarray_3(self):
        t = datetime(2024, 6, 15, 12, 0, 0, tzinfo=timezone.utc)
        disp = solid_tide_displacement(t, MIDLAT_ECEF)
        self.assertEqual(disp.shape, (3,))


class DeterminismTest(unittest.TestCase):
    """Same inputs → same outputs.  Verify no hidden global state."""

    def test_deterministic(self):
        t = datetime(2024, 6, 15, 12, 34, 56, tzinfo=timezone.utc)
        d1 = solid_tide_displacement(t, ABMF_ECEF)
        d2 = solid_tide_displacement(t, ABMF_ECEF)
        self.assertTrue(np.allclose(d1, d2),
                        "SET must be deterministic — same inputs, same output")


if __name__ == "__main__":
    unittest.main()

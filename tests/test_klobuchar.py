"""Tests for the Klobuchar single-frequency ionospheric delay model.

Validates the algorithm against the worked example in ICD-GPS-200N
§20.3.3.5.2.6 ('Sample Computation') plus property tests for the
expected behaviour at extremes (high elevation, polar latitude,
frequency scaling).
"""

from __future__ import annotations

import math
import os
import sys
import unittest

_SCRIPTS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'scripts'))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from klobuchar import (  # noqa: E402
    KlobucharParams,
    slant_iono_delay_l1_m,
    slant_iono_delay_m,
)


# A nominal set of broadcast Klobuchar params, representative of typical
# CONUS-mid-solar values.  Used as the default fixture.
_NOMINAL_ALPHA = (1.397e-8, 0.0, -5.96e-8, 5.96e-8)
_NOMINAL_BETA = (1.229e5, 0.0, -2.621e5, 1.966e5)


def _params(alpha=_NOMINAL_ALPHA, beta=_NOMINAL_BETA) -> KlobucharParams:
    return KlobucharParams(alpha=alpha, beta=beta)


class IcdSampleComputationTest(unittest.TestCase):
    """ICD-GPS-200N §20.3.3.5.2.6 sample-computation inputs.

    Reproduces the worked example with:
      α = (3.82e-8, 1.49e-8, -1.79e-7, 0)
      β = (1.43e5, 0, -3.28e5, 1.13e5)
      φ_u = 40°N (= 0.222 semicircles)
      λ_u = -100°W (= -0.556 semicircles)
      E = 20°, A = 210°, t_GPS = 593100 s

    Step-by-step the algorithm produces (per Misra & Enge,
    *Global Positioning System: Signals, Measurements, and Performance*,
    and matched independently by RTKLIB's ionmodel() reference impl):
      ψ ≈ 0.0400 sc
      φ_i ≈ 0.187 sc
      λ_i ≈ -0.580 sc
      φ_m ≈ 0.239 sc
      t_local ≈ 49660 s
      AMP ≈ 3.154e-8 s
      PER ≈ 125700 s
      x ≈ -0.037 rad
      F ≈ 2.18
      T_iono ≈ 7.95e-8 s ≈ 23.8 m at L1

    Acceptance: within 0.5 m of 23.8 m.  Tighter tolerance isn't
    meaningful — Klobuchar itself is a 50%-RMS model.  The test is
    primarily to catch unit-conversion regressions.
    """

    def test_icd_worked_example(self):
        params = _params(
            alpha=(3.82e-8, 1.49e-8, -1.79e-7, 0.0),
            beta=(1.43e5, 0.0, -3.28e5, 1.13e5),
        )
        delay_m = slant_iono_delay_l1_m(
            params,
            t_gps_s=593100.0,
            rx_lat_rad=math.radians(40.0),
            rx_lon_rad=math.radians(-100.0),
            sv_elev_rad=math.radians(20.0),
            sv_az_rad=math.radians(210.0),
        )
        self.assertAlmostEqual(delay_m, 23.8, delta=0.5,
                               msg=f"ICD sample computation expected ~23.8 m "
                                   f"at L1, got {delay_m:.3f} m")


class ElevationDependenceTest(unittest.TestCase):
    """The slant factor F = 1 + 16·(0.53 − E_sc)^3 grows monotonically
    as elevation drops from zenith (E=π/2) toward the horizon (E=0).

    A zenith-pointed SV has F=1 (vertical delay only); a 5°-elevation
    SV has F ~3, multiplying the vertical delay accordingly.
    """

    def _delay_at(self, elev_deg: float) -> float:
        return slant_iono_delay_l1_m(
            _params(),
            t_gps_s=43200.0,
            rx_lat_rad=math.radians(45.0),
            rx_lon_rad=math.radians(-90.0),
            sv_elev_rad=math.radians(elev_deg),
            sv_az_rad=math.radians(180.0),
        )

    def test_zenith_least_delay(self):
        zenith = self._delay_at(89.9)
        low = self._delay_at(10.0)
        self.assertLess(zenith, low,
                        f"zenith delay {zenith:.3f} m should be < low-elev "
                        f"{low:.3f} m")

    def test_low_elev_amplifies(self):
        """Slant factor F at 5° elev is ~3.6; at 90° is ~1.0.  Slant
        delay should be ~3-4× zenith delay."""
        zenith = self._delay_at(89.9)
        low = self._delay_at(5.0)
        ratio = low / zenith if zenith > 0 else float('inf')
        self.assertGreater(ratio, 2.0,
                           f"low-elev:zenith ratio {ratio:.2f} should be > 2 "
                           f"(slant factor)")


class FrequencyScalingTest(unittest.TestCase):
    """First-order iono delay scales as 1/f².  L5 (1176.45 MHz) should
    have ~1.79× the L1 delay (since (f_L1/f_L5)^2 = (1575.42/1176.45)^2
    ≈ 1.79)."""

    F_L1 = 1575.42e6
    F_L2 = 1227.60e6
    F_L5 = 1176.45e6

    def test_l1_unscaled(self):
        """slant_iono_delay_m at f=L1 should equal slant_iono_delay_l1_m."""
        params = _params()
        l1 = slant_iono_delay_l1_m(
            params, t_gps_s=43200.0,
            rx_lat_rad=math.radians(45.0),
            rx_lon_rad=math.radians(-90.0),
            sv_elev_rad=math.radians(45.0),
            sv_az_rad=math.radians(135.0))
        f_l1 = slant_iono_delay_m(
            params, t_gps_s=43200.0,
            rx_lat_rad=math.radians(45.0),
            rx_lon_rad=math.radians(-90.0),
            sv_elev_rad=math.radians(45.0),
            sv_az_rad=math.radians(135.0),
            freq_hz=self.F_L1)
        self.assertAlmostEqual(l1, f_l1, places=6)

    def test_l5_amplification(self):
        params = _params()
        kwargs = dict(
            t_gps_s=43200.0,
            rx_lat_rad=math.radians(45.0),
            rx_lon_rad=math.radians(-90.0),
            sv_elev_rad=math.radians(45.0),
            sv_az_rad=math.radians(135.0))
        l1 = slant_iono_delay_m(params, freq_hz=self.F_L1, **kwargs)
        l5 = slant_iono_delay_m(params, freq_hz=self.F_L5, **kwargs)
        ratio = l5 / l1 if l1 > 0 else float('inf')
        # Expected (f_L1 / f_L5)^2 = 1.793
        self.assertAlmostEqual(ratio, (self.F_L1 / self.F_L5) ** 2, places=3)

    def test_invalid_freq_raises(self):
        with self.assertRaises(ValueError):
            slant_iono_delay_m(
                _params(), t_gps_s=0.0,
                rx_lat_rad=0.0, rx_lon_rad=0.0,
                sv_elev_rad=0.5, sv_az_rad=1.0,
                freq_hz=0.0)


class FloorBehaviorTest(unittest.TestCase):
    """The algorithm clamps several quantities; verify the clamps fire
    at the right inputs."""

    def test_amp_floor_zero(self):
        """If the α-polynomial evaluates negative, AMP is forced to 0
        and the delay floors at the F·5e-9 baseline."""
        # Choose params that make the α polynomial negative at the
        # geomag latitude.  α0 = -1e-7 dominates for small φ_m.
        params = _params(
            alpha=(-1.0e-7, 0.0, 0.0, 0.0),
            beta=(72000.0, 0.0, 0.0, 0.0),
        )
        delay = slant_iono_delay_l1_m(
            params, t_gps_s=43200.0,
            rx_lat_rad=math.radians(0.0),
            rx_lon_rad=math.radians(0.0),
            sv_elev_rad=math.radians(45.0),
            sv_az_rad=math.radians(0.0))
        # The 5e-9-second baseline at L1 with F~1 is ~1.5 m.  The negative
        # AMP is clamped, so we get this floor and not a negative delay.
        self.assertGreater(delay, 0.0)
        self.assertLess(delay, 3.0)

    def test_per_floor_72000(self):
        """If the β-polynomial evaluates < 72000 s, PER is forced to
        72000 s.  The minimum period prevents the cosine from
        oscillating arbitrarily fast.  Functionally the test passes
        as long as the function returns a finite, positive number for
        a low-period setup."""
        params = _params(beta=(0.0, 0.0, 0.0, 0.0))  # β polynomial = 0
        delay = slant_iono_delay_l1_m(
            params, t_gps_s=43200.0,
            rx_lat_rad=math.radians(0.0),
            rx_lon_rad=math.radians(0.0),
            sv_elev_rad=math.radians(45.0),
            sv_az_rad=math.radians(0.0))
        self.assertTrue(math.isfinite(delay))
        self.assertGreater(delay, 0.0)


class GeometricEdgeCasesTest(unittest.TestCase):
    """Edge inputs that could cause numerical trouble — verify they
    don't NaN, raise, or produce negative delays."""

    def test_polar_observer(self):
        """Receiver at the pole — cos(φ_i) can hit zero if the iono
        pierce-point latitude is forced to ±0.416.  Verify the divide
        guard works."""
        params = _params()
        delay = slant_iono_delay_l1_m(
            params, t_gps_s=43200.0,
            rx_lat_rad=math.radians(89.9),
            rx_lon_rad=math.radians(0.0),
            sv_elev_rad=math.radians(45.0),
            sv_az_rad=math.radians(180.0))
        self.assertTrue(math.isfinite(delay))
        self.assertGreaterEqual(delay, 0.0)

    def test_zenith_sv(self):
        """SV directly overhead — slant factor = 1.0, delay = vertical
        delay only."""
        params = _params()
        delay = slant_iono_delay_l1_m(
            params, t_gps_s=43200.0,
            rx_lat_rad=math.radians(45.0),
            rx_lon_rad=math.radians(-90.0),
            sv_elev_rad=math.radians(89.99),
            sv_az_rad=math.radians(0.0))
        # Vertical delay typical at 45° lat / mid solar / nominal params:
        # ~0.5-3 m depending on time-of-day.
        self.assertGreater(delay, 0.0)
        self.assertLess(delay, 10.0)

    def test_finite_at_low_elev(self):
        """5° elevation — slant factor F ~3.6; delay should still be
        finite and bounded."""
        params = _params()
        delay = slant_iono_delay_l1_m(
            params, t_gps_s=43200.0,
            rx_lat_rad=math.radians(45.0),
            rx_lon_rad=math.radians(-90.0),
            sv_elev_rad=math.radians(5.0),
            sv_az_rad=math.radians(0.0))
        self.assertTrue(math.isfinite(delay))
        # At 5° elev, even mid-solar Klobuchar shouldn't exceed ~30 m.
        self.assertLess(delay, 30.0)


if __name__ == '__main__':
    unittest.main()

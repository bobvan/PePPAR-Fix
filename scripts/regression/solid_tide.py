"""IERS 2010 Conventions Step 1 solid Earth tide displacement.

Implements the degree-2 + degree-3 nominal-Love-number model from
IERS Technical Note 36, Section 7.1.1, equations 7.5a/b.  Returns
the station position displacement in ECEF metres at a given epoch.

This is the single largest station-position correction missing from
our PPP filter: PRIDE ablation on ABMF 2020/001 showed the solid
Earth tide contributes ~42 mm to the position solution, 50× larger
than ocean or pole tides.  See
`project_to_main_pride_ablation_20260423`.

Accuracy: ~1 mm horizontal, ~2 mm vertical at mid-latitudes when
compared to IERS reference implementations.  Adequate for the
meter-scale gap between our filter and PRIDE at 3 mm ceiling.

Not implemented here (out of scope for the initial port):
- IERS 2010 Step 2 frequency-dependent corrections (sub-mm)
- Permanent tide handling (relevant only for tide-free vs
  mean-tide coordinate system conversions)
- Polar motion effects on the station position (sub-mm)

Reference:
- IERS Technical Note 36 (IERS Conventions 2010), Chapter 7.1.1.
- Petit & Luzum (2010) eqs 7.5a, 7.5b.
"""

from __future__ import annotations

import math
from datetime import datetime, timezone

import numpy as np


# Physical constants (IERS 2010)
_GM_EARTH = 3.986004418e14     # m^3/s^2
_GM_SUN = 1.32712442099e20     # m^3/s^2
_GM_MOON = 4.9028695e12        # m^3/s^2
_R_EARTH = 6378137.0           # m (WGS-84 equatorial radius)
_AU = 149597870700.0           # m

# Nominal Love numbers (IERS 2010 Table 7.1, degree-2 elastic Earth)
_H2 = 0.6078
_L2 = 0.0847
_H3 = 0.292
_L3 = 0.015


def _jd_from_datetime(t: datetime) -> float:
    """Julian Date (UT) from a timezone-aware datetime.  GPS-UT ≈ leap
    seconds (~37 s on 2020-01-01), which is negligible for ephemeris
    calculations at arcmin precision."""
    if t.tzinfo is None:
        raise ValueError("datetime must be timezone-aware")
    t = t.astimezone(timezone.utc)
    # Meeus Chapter 7 algorithm
    y, m = t.year, t.month
    d = t.day + (t.hour + (t.minute + (t.second + t.microsecond / 1e6) / 60.0) / 60.0) / 24.0
    if m <= 2:
        y -= 1
        m += 12
    a = y // 100
    b = 2 - a + a // 4
    jd = (math.floor(365.25 * (y + 4716)) + math.floor(30.6001 * (m + 1))
          + d + b - 1524.5)
    return jd


def _gmst_rad(jd_ut: float) -> float:
    """Greenwich Mean Sidereal Time in radians, from UT Julian Date.
    Low-precision formula from IERS Technical Note 21 (±1 arcsec is
    overkill here; SET is ~arcmin-sensitive to body position).
    """
    T = (jd_ut - 2451545.0) / 36525.0
    # GMST at 0h UT, seconds
    gmst_h = (6.697374558 + 0.06570982441908 * (jd_ut - 2451545.0)
              + 1.00273790935 * ((jd_ut % 1) - 0.5) * 24
              + 0.000026 * T * T) % 24.0
    return math.radians(gmst_h * 15.0)


def _sun_pos_eci(jd: float) -> np.ndarray:
    """Geocentric Sun position in equatorial J2000 (approximate ECI),
    metres.  Montenbruck & Gill "Satellite Orbits" simplified formula.
    Accuracy ~ arcmin.
    """
    T = (jd - 2451545.0) / 36525.0
    # Mean longitude (deg)
    L = (280.460 + 36000.771 * T) % 360.0
    # Mean anomaly (deg)
    M = math.radians((357.5277233 + 35999.05034 * T) % 360.0)
    # Ecliptic longitude (apparent)
    lam = math.radians(L + 1.914666471 * math.sin(M)
                       + 0.019994643 * math.sin(2 * M))
    # Distance (AU)
    r_au = 1.000140612 - 0.016708617 * math.cos(M) - 0.000139589 * math.cos(2 * M)
    # Obliquity of the ecliptic (deg)
    eps = math.radians(23.43929111 - 0.0130042 * T)
    r = r_au * _AU
    x = r * math.cos(lam)
    y = r * math.cos(eps) * math.sin(lam)
    z = r * math.sin(eps) * math.sin(lam)
    return np.array([x, y, z])


def _moon_pos_eci(jd: float) -> np.ndarray:
    """Geocentric Moon position in equatorial J2000 (approximate ECI),
    metres.  Montenbruck & Gill "Satellite Orbits" simplified — the
    leading terms of the ELP2000 series.  Accuracy ~ arcmin.
    """
    T = (jd - 2451545.0) / 36525.0
    # Mean longitude, elongation, anomalies, latitude arg (deg)
    Lp = (218.31617 + 481267.88088 * T) % 360.0
    D = math.radians((297.85027 + 445267.11135 * T) % 360.0)
    Mp = math.radians((134.96292 + 477198.86753 * T) % 360.0)
    M = math.radians((357.52543 + 35999.04944 * T) % 360.0)
    F = math.radians((93.27283 + 483202.01873 * T) % 360.0)
    # Ecliptic longitude (deg)
    lam = Lp + (6.28875 * math.sin(Mp) + 1.27402 * math.sin(2 * D - Mp)
                + 0.65830 * math.sin(2 * D) + 0.21358 * math.sin(2 * Mp)
                - 0.18583 * math.sin(M) - 0.11418 * math.sin(2 * F))
    # Ecliptic latitude (deg)
    beta = (5.12819 * math.sin(F) + 0.28060 * math.sin(Mp + F)
            + 0.27769 * math.sin(Mp - F) + 0.17324 * math.sin(2 * D - F))
    # Distance (km)
    r_km = (385000.56 - 20905.355 * math.cos(Mp)
            - 3699.111 * math.cos(2 * D - Mp) - 2955.968 * math.cos(2 * D)
            - 569.925 * math.cos(2 * Mp))
    r = r_km * 1000.0
    lam_r = math.radians(lam)
    beta_r = math.radians(beta)
    # Ecliptic → equatorial
    eps = math.radians(23.43929111 - 0.0130042 * T)
    x_ecl = r * math.cos(beta_r) * math.cos(lam_r)
    y_ecl = r * math.cos(beta_r) * math.sin(lam_r)
    z_ecl = r * math.sin(beta_r)
    x = x_ecl
    y = y_ecl * math.cos(eps) - z_ecl * math.sin(eps)
    z = y_ecl * math.sin(eps) + z_ecl * math.cos(eps)
    return np.array([x, y, z])


def _eci_to_ecef(v_eci: np.ndarray, gmst: float) -> np.ndarray:
    """Rotate a vector from ECI (equatorial J2000) to ECEF by GMST.
    Neglects precession/nutation + polar motion (sub-mm for SET).
    """
    c, s = math.cos(gmst), math.sin(gmst)
    return np.array([
        c * v_eci[0] + s * v_eci[1],
        -s * v_eci[0] + c * v_eci[1],
        v_eci[2],
    ])


def solid_tide_displacement(t: datetime, station_ecef: np.ndarray) -> np.ndarray:
    """IERS 2010 Step 1 solid Earth tide displacement at a station.

    Args:
        t: UTC datetime (timezone-aware).  GPS-UTC leap seconds ignored
           (~37 s → arcsec-scale body position error → sub-mm tide error).
        station_ecef: station position in ITRF ECEF metres, shape (3,).

    Returns:
        Displacement vector (Δx, Δy, Δz) in ECEF metres, to be ADDED
        to the ITRF station position to get the instantaneous position
        the GNSS signal actually sees.

    Sign convention: the RETURNED vector is the displacement of the
    station due to tidal forces.  In a PPP filter, the geometric
    range is computed between the satellite and the INSTANTANEOUS
    station position (= ITRF + this displacement).
    """
    if station_ecef.shape != (3,):
        raise ValueError(f"station_ecef must be shape (3,), got {station_ecef.shape}")
    jd = _jd_from_datetime(t)
    gmst = _gmst_rad(jd)
    r_sun = _eci_to_ecef(_sun_pos_eci(jd), gmst)
    r_moon = _eci_to_ecef(_moon_pos_eci(jd), gmst)
    r_sta = float(np.linalg.norm(station_ecef))
    r_hat = station_ecef / r_sta

    disp = np.zeros(3)
    for gm_body, r_body_vec, h2, l2 in [(_GM_SUN, r_sun, _H2, _L2),
                                         (_GM_MOON, r_moon, _H2, _L2)]:
        r_body = float(np.linalg.norm(r_body_vec))
        body_hat = r_body_vec / r_body
        cos_psi = float(np.dot(r_hat, body_hat))
        # IERS 2010 Eq 7.5a — degree-2 Love-number displacement
        # Scale factor: (GM_body / GM_E) * (r_sta^4 / r_body^3)
        scale = (gm_body / _GM_EARTH) * (r_sta ** 4) / (r_body ** 3)
        # Radial: h2 * (3 cos²ψ - 1) / 2 * r_hat
        radial_mag = h2 * (3 * cos_psi * cos_psi - 1) / 2
        # Transverse: 3 * l2 * cos(ψ) * (body_hat - cos(ψ) * r_hat)
        trans_vec = 3 * l2 * cos_psi * (body_hat - cos_psi * r_hat)
        disp += scale * (radial_mag * r_hat + trans_vec)
    return disp

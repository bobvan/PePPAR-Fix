"""Saastamoinen tropospheric delay model for METAR-seeded ZTD priors.

Computes zenith hydrostatic + wet delays from station weather:
  - Magnus formula for water-vapor pressure from dewpoint.
  - Standard-atmosphere altimeter→station-pressure for METAR
    altim_hPa (sea-level adjusted) → site pressure at elev.
  - Saastamoinen (1972) ZHD with the standard latitude / height
    f(φ,h) gravity correction.
  - Saastamoinen-Davis ZWD form (Davis et al. 1985).

These are the textbook formulae used by RTKLIB, GipsyX, and IGS
products.  Accuracy ~10 mm on ZHD and 10-30 mm on ZWD when the
input weather actually represents the site (here we use KDPA
METAR + lapse-rate spatial correction to UFO1).

Units: distances in meters; pressures in hPa; temperatures in
Kelvin or Celsius as marked.  Station inputs are *site* pressure
(post-altimeter conversion), not sea-level.

References:
  Saastamoinen 1972 "Atmospheric correction for the troposphere
    and stratosphere in radio ranging of satellites."
  Davis et al. 1985 "Geodesy by radio interferometry" (eq. for ZWD)
"""
from __future__ import annotations

import math


def vapor_pressure_from_dewpoint(dewp_C: float) -> float:
    """Magnus-Tetens water-vapor pressure (hPa) from dewpoint (°C).

    e = 6.1078 × exp(17.27 × Td / (Td + 237.3))

    Valid for −45 °C < Td < +60 °C with ≤ 0.4 % error vs the
    Goff-Gratch reference.
    """
    return 6.1078 * math.exp(17.27 * dewp_C / (dewp_C + 237.3))


def station_pressure_from_altimeter(altim_hPa: float, elev_m: float) -> float:
    """Convert METAR altimeter setting (sea-level, hPa) to site
    pressure (hPa) at given elevation, using the standard
    atmosphere temperature lapse.

    P_station = altim × (1 − 0.0065 × h / 288.15)^5.255

    KDPA METAR altim_hPa is the QNH-equivalent (already corrected
    by the airport to sea-level).  We bring it back down to the
    actual receiver elevation using the standard-atmosphere
    relationship.  Acceptable for elev < 1500 m; for higher sites
    use temperature-corrected formulas.
    """
    return altim_hPa * (1.0 - 0.0065 * elev_m / 288.15) ** 5.255


def saastamoinen_zhd(P_hPa: float, lat_deg: float, h_km: float) -> float:
    """Zenith hydrostatic (dry) delay (m) from station pressure.

    ZHD = 0.0022768 × P / f(φ, h)
    f(φ, h) = 1 − 0.00266·cos(2φ) − 0.00028·h_km

    φ in radians for the cos; we accept degrees and convert.
    Accurate to ~1 mm given accurate site pressure.
    """
    phi = math.radians(lat_deg)
    f = 1.0 - 0.00266 * math.cos(2.0 * phi) - 0.00028 * h_km
    return 0.0022768 * P_hPa / f


def saastamoinen_zwd(T_K: float, e_hPa: float) -> float:
    """Zenith wet delay (m) from station temperature + vapor pressure.

    ZWD = 0.002277 × (1255/T + 0.05) × e

    T in Kelvin, e in hPa.  Saastamoinen (1972) form.  Davis 1985
    refinement uses (1255/T + 0.05); some references use 0.002277·
    (1255/T_m + 0.05)·e where T_m is the water-vapor weighted mean
    temperature (~0.74·T_surface in mid-latitudes).  We use surface
    T directly here — yields ZWD slightly biased compared to the
    Tm-corrected form, but the bias is absorbed by the residual
    state on top of the engine's 2.3 m hydrostatic apriori.
    """
    return 0.002277 * (1255.0 / T_K + 0.05) * e_hPa


def metar_to_ztd(
    temp_C: float,
    dewp_C: float,
    altim_hPa: float,
    lat_deg: float,
    elev_m: float,
) -> tuple[float, float, float]:
    """Compute (ZHD, ZWD, ZTD) in meters from METAR-style weather +
    site location.

    Inputs are the columns from KDPA cron CSV:
      temp_C, dewp_C, altim_hPa
    plus lat / elev of the actual receiver site.

    Returns total ZTD = ZHD + ZWD plus the components for diagnostic
    logging.
    """
    P_station = station_pressure_from_altimeter(altim_hPa, elev_m)
    e = vapor_pressure_from_dewpoint(dewp_C)
    T_K = temp_C + 273.15
    h_km = elev_m / 1000.0
    zhd = saastamoinen_zhd(P_station, lat_deg, h_km)
    zwd = saastamoinen_zwd(T_K, e)
    return zhd, zwd, zhd + zwd


# Engine's hardcoded apriori zenith dry delay (solve_ppp.py:692).
# The IDX_ZTD residual state rides on top of this, so the seed we
# return is the difference (ZTD − APRIORI), not the raw ZTD.
ENGINE_ZTD_APRIORI_M = 2.3


def metar_to_init_ztd_residual(
    temp_C: float,
    dewp_C: float,
    altim_hPa: float,
    lat_deg: float,
    elev_m: float,
) -> tuple[float, dict]:
    """Compute the *residual* ZTD (m) to seed PPPFilter / FixedPosFilter
    state with, given METAR + site location.

    Returns (residual_m, diag) where diag is a dict suitable for
    logging the components.  The residual is signed: positive when
    actual ZTD exceeds the engine's 2.3 m hydrostatic apriori
    (typical: a few cm in temperate climate), negative under low-
    pressure systems where ZHD dips below 2.3 m.
    """
    zhd, zwd, ztd = metar_to_ztd(temp_C, dewp_C, altim_hPa,
                                 lat_deg, elev_m)
    residual = ztd - ENGINE_ZTD_APRIORI_M
    return residual, {
        'temp_C': temp_C,
        'dewp_C': dewp_C,
        'altim_hPa': altim_hPa,
        'lat_deg': lat_deg,
        'elev_m': elev_m,
        'P_station_hPa': station_pressure_from_altimeter(altim_hPa, elev_m),
        'e_hPa': vapor_pressure_from_dewpoint(dewp_C),
        'zhd_m': zhd,
        'zwd_m': zwd,
        'ztd_m': ztd,
        'apriori_m': ENGINE_ZTD_APRIORI_M,
        'residual_m': residual,
    }

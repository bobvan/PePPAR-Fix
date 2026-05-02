"""Klobuchar single-frequency ionospheric delay model.

Standard model for single-frequency receivers per ICD-GPS-200 §20.3.3.5.2.5.
Computes the slant ionospheric delay at a given (SV, receiver, epoch) using
8 broadcast parameters (α[0..3], β[0..3]) transmitted in GPS subframe 4
page 18.

This is a model, not a measurement.  Typical accuracy is ~50% RMS reduction
of the actual ionospheric delay — i.e., post-correction residual ~3 m at
solar maximum, ~1 m at solar minimum.  That is **far worse** than what an
ionosphere-free (IF) dual-frequency combination achieves (sub-cm cancellation
of first-order iono).

This module exists for the **single-frequency float-PPP** case (I-182352-
charlie): observations from receivers that emit only one frequency for a
given (SV, signal) — e.g., F9T TIM 2.20 outputting BDS B1I single-freq, or
F9P tracking GLO L1+L2 with no GLO SSR bias — cannot form an IF combination
and therefore have no automatic iono cancellation.  Klobuchar's modelled
delay is added to the observation model as a known correction; the residual
~1-3 m iono error is absorbed by the float ambiguity.

For AR (LAMBDA) we still require dual-freq IF observations where the iono
cancels naturally.  Single-freq float-PPP contributes geometry to the float
position; it does not contribute integer-fixable ambiguities.  See
docs/ac-datum-mixing.md for the float-vs-AR architectural framing.

Reference: ICD-GPS-200N §20.3.3.5.2.5 (Ionospheric Correction).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Speed of light in vacuum (m/s).  Use the same value as ssr_corrections /
# solve_ppp for numerical consistency.
_C_M_PER_S = 299792458.0

# GPS L1 frequency, used as the reference for cross-frequency scaling.
_F_L1_HZ = 1575.42e6


@dataclass(frozen=True)
class KlobucharParams:
    """The 8 broadcast parameters from GPS LNAV subframe 4 page 18.

    Units (per ICD-GPS-200N):
      alpha[0]: seconds
      alpha[1]: seconds / semicircle
      alpha[2]: seconds / semicircle^2
      alpha[3]: seconds / semicircle^3
      beta[0]:  seconds
      beta[1]:  seconds / semicircle
      beta[2]:  seconds / semicircle^2
      beta[3]:  seconds / semicircle^3

    A 'semicircle' is the GPS-ICD angular unit equal to π radians, i.e. one
    half-revolution; angles in this module that end in '_sc' are in
    semicircles, ranging [0, 2] for azimuths and [-1, 1] for latitudes.
    """
    alpha: tuple[float, float, float, float]
    beta: tuple[float, float, float, float]


def slant_iono_delay_l1_m(
    params: KlobucharParams,
    t_gps_s: float,
    rx_lat_rad: float,
    rx_lon_rad: float,
    sv_elev_rad: float,
    sv_az_rad: float,
) -> float:
    """Compute the L1 slant ionospheric delay (metres) for one (SV, epoch).

    Inputs:
      params     — broadcast Klobuchar parameters
      t_gps_s    — GPS time of week (seconds; the algorithm uses
                   (t mod 86400 s) so values outside [0, 604800) work too)
      rx_lat_rad — receiver geodetic latitude (radians, [-π/2, π/2])
      rx_lon_rad — receiver geodetic longitude (radians, [-π, π])
      sv_elev_rad — SV elevation above receiver horizon (radians, [0, π/2])
      sv_az_rad  — SV azimuth measured eastward from north (radians, [0, 2π])

    Returns the slant iono delay at L1, in metres.  Multiply by
    ``(f_L1 / f) ** 2`` for other frequency bands — see
    ``slant_iono_delay_m`` for the convenience wrapper.

    The algorithm follows ICD-GPS-200N §20.3.3.5.2.5 step-by-step;
    intermediate variables retain the ICD's notation (ψ, φ_i, λ_i,
    φ_m, t, AMP, PER, x, F) for ease of cross-reference.
    """
    # Convert to semicircles, the ICD's working unit.
    rx_lat_sc = rx_lat_rad / math.pi
    rx_lon_sc = rx_lon_rad / math.pi
    sv_elev_sc = sv_elev_rad / math.pi
    sv_az_sc = sv_az_rad / math.pi

    # Step 1: Earth-centred angle (semicircles).
    psi_sc = 0.0137 / (sv_elev_sc + 0.11) - 0.022

    # Step 2: subionospheric latitude (semicircles), clamped to ±0.416.
    phi_i_sc = rx_lat_sc + psi_sc * math.cos(sv_az_rad)
    if phi_i_sc > 0.416:
        phi_i_sc = 0.416
    elif phi_i_sc < -0.416:
        phi_i_sc = -0.416

    # Step 3: subionospheric longitude (semicircles).
    cos_phi_i = math.cos(phi_i_sc * math.pi)
    if abs(cos_phi_i) < 1e-12:
        # Degenerate at polar projection — fall back to the receiver's lon.
        lambda_i_sc = rx_lon_sc
    else:
        lambda_i_sc = rx_lon_sc + (psi_sc * math.sin(sv_az_rad)) / cos_phi_i

    # Step 4: geomagnetic latitude (semicircles).
    phi_m_sc = phi_i_sc + 0.064 * math.cos((lambda_i_sc - 1.617) * math.pi)

    # Step 5: local time at the ionospheric pierce point (seconds, mod 86400).
    t_local = 4.32e4 * lambda_i_sc + t_gps_s
    t_local = t_local - 86400.0 * math.floor(t_local / 86400.0)

    # Step 6: amplitude of the cosine term (seconds).  Sum α_n · φ_m^n.
    amp = (params.alpha[0]
           + params.alpha[1] * phi_m_sc
           + params.alpha[2] * phi_m_sc * phi_m_sc
           + params.alpha[3] * phi_m_sc ** 3)
    if amp < 0.0:
        amp = 0.0

    # Step 7: period of the cosine term (seconds).  Sum β_n · φ_m^n.
    per = (params.beta[0]
           + params.beta[1] * phi_m_sc
           + params.beta[2] * phi_m_sc * phi_m_sc
           + params.beta[3] * phi_m_sc ** 3)
    if per < 72000.0:
        per = 72000.0

    # Step 8: phase argument (radians).
    x = 2.0 * math.pi * (t_local - 50400.0) / per

    # Step 9: obliquity (slant) factor.  F is dimensionless.
    f_obliquity = 1.0 + 16.0 * (0.53 - sv_elev_sc) ** 3

    # Step 10: vertical iono delay (seconds), multiplied by F to slant.
    if abs(x) < 1.57:
        # Cosine-with-Taylor (matches the ICD's reference impl exactly).
        t_iono_s = f_obliquity * (5.0e-9 + amp * (1.0 - x * x / 2.0
                                                  + x ** 4 / 24.0))
    else:
        # Daytime/zenith fall-off.
        t_iono_s = f_obliquity * 5.0e-9

    return t_iono_s * _C_M_PER_S


def slant_iono_delay_m(
    params: KlobucharParams,
    t_gps_s: float,
    rx_lat_rad: float,
    rx_lon_rad: float,
    sv_elev_rad: float,
    sv_az_rad: float,
    freq_hz: float,
) -> float:
    """Compute the slant ionospheric delay at the requested frequency (m).

    The first-order iono delay scales as ``1 / f^2``; the L1-relative
    scaling factor is ``(f_L1 / f) ** 2``.  Frequencies higher than L1
    return smaller delays; lower frequencies return larger delays.
    """
    delay_l1_m = slant_iono_delay_l1_m(
        params, t_gps_s, rx_lat_rad, rx_lon_rad, sv_elev_rad, sv_az_rad)
    if freq_hz <= 0:
        raise ValueError(f"freq_hz must be > 0, got {freq_hz!r}")
    return delay_l1_m * (_F_L1_HZ / freq_hz) ** 2

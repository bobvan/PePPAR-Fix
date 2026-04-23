"""End-to-end regression harness for PePPAR Fix's PPP pipeline.

Threads RINEX OBS + RINEX NAV (+ optional Bias-SINEX OSB) through
`PPPFilter` epoch-by-epoch and reports the final position error
against an independent truth coordinate.

## Usage

Float-PPP only (no AR), broadcast orbits, no SSR biases.  Loose
tolerance — confirms the position pipeline computes a reasonable
solution from RINEX inputs:

    python scripts/regression/run_regression.py \
        --obs /path/to/abmf0010.20o \
        --nav /path/to/brdc0010.20p \
        --truth "2919785.79086,-5383744.95943,1774604.85992" \
        --tolerance-m 10 \
        --max-epochs 200 \
        --profile l5

Add SSR biases (when a .BIA file is available):

    ... --bia /path/to/file.BIA --tolerance-m 1

Returns 0 on pass, non-zero on fail.  Reports per-axis errors and
RMS to stdout.

## Scope

This first cut is **float-PPP only**.  No MW tracker, no LAMBDA,
no per-SV state machine.  The goal is to validate that the basic
position-computation pipeline (filter + sat-position propagation
from broadcast NAV + observation ingest) produces an answer
consistent with the IGS-published truth coordinate.

Known TODO before the runner can actually converge against truth
within tight tolerance:

- **Receiver-clock initialization** — at startup, the real receiver
  carries a clock bias of microseconds-to-milliseconds, which shows
  up as a uniform per-SV pseudorange offset.  Float-PPP without the
  filter's clock state pre-seeded sees this as huge residuals on
  every observation and rejects most of them.  Use `solve_ppp.ls_init`
  on the first epoch to get a position+clock seed before launching
  the filter, the way `peppar_fix_engine.run_bootstrap` does.
- **SSR phase- and code-bias application** — `bias_sinex_reader`
  parses these but the runner doesn't yet apply them to obs.  Once
  wired, ~10 m → ~10 cm.
- **MW + LAMBDA + state machine** — once float-PPP converges, run
  the AR path against the same data and tighten to mm-level.

Until those land, the runner is useful as plumbing validation only.
"""

from __future__ import annotations

import argparse
import logging
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import numpy as np

from regression.rinex_reader import (
    iter_epochs, parse_header as parse_obs_header,
    extract_dual_freq, L5_PROFILE, L2_PROFILE,
)
from regression.rinex_nav_reader import load_into_ephemeris

log = logging.getLogger("regression")


C_LIGHT = 299_792_458.0


def _parse_truth(s: str) -> np.ndarray:
    parts = [float(x) for x in s.split(',')]
    if len(parts) != 3:
        raise ValueError(f"truth must be 'X,Y,Z' in meters: {s!r}")
    return np.array(parts)


_SYS_TO_LOWER = {'GPS': 'gps', 'GAL': 'gal', 'BDS': 'bds',
                 'GLO': 'glo', 'QZS': 'qzs'}


_UNIFORM_PROFILES = {'l5': L5_PROFILE, 'l2': L2_PROFILE}


def _parse_profile(spec: str) -> dict[str, list[tuple[str, str]]]:
    """Parse the --profile spec into a per-constellation profile dict.

    Two formats supported:

    - **Uniform**: ``l5`` or ``l2`` — apply one profile to every
      constellation in the profile dict (legacy behaviour, equivalent
      to the original ``args.profile`` semantics).
    - **Per-constellation**: ``gps:l2,gal:l5,bds:l5`` — assign each
      listed constellation a specific profile.  Omitted constellations
      are absent from the returned dict (observations from them are
      dropped at `extract_dual_freq` time, same path as `--systems`).

    The per-constellation form is the fix for ABMF 2020/001's
    5.7 m GPS+GAL L5 residual: CODE's GPS precise clocks are
    referenced to IF(L1, L2W), so GPS observations must use the
    L2 profile even when GAL is using L5.  See
    `project_to_main_pride_gps_filter_degeneracy_20260423` and
    `project_to_main_part_a_result_20260423`.
    """
    spec = (spec or "").strip().lower()
    if spec in _UNIFORM_PROFILES:
        return _UNIFORM_PROFILES[spec]
    out: dict[str, list[tuple[str, str]]] = {}
    for pair in spec.split(','):
        pair = pair.strip()
        if not pair:
            continue
        if ':' not in pair:
            raise ValueError(
                f"bad --profile entry {pair!r}: expected 'l5', 'l2', "
                f"or 'sys:profile' (e.g. 'gps:l2,gal:l5')"
            )
        sys_spec, prof_spec = pair.split(':', 1)
        sys_upper = sys_spec.strip().upper()
        prof_lower = prof_spec.strip().lower()
        if sys_upper not in ('GPS', 'GAL', 'BDS'):
            raise ValueError(
                f"unknown system in --profile: {sys_spec!r} "
                f"(expected one of gps, gal, bds)"
            )
        if prof_lower not in _UNIFORM_PROFILES:
            raise ValueError(
                f"unknown profile in --profile: {prof_spec!r} "
                f"(expected one of {sorted(_UNIFORM_PROFILES)})"
            )
        out[sys_upper] = _UNIFORM_PROFILES[prof_lower][sys_upper]
    if not out:
        raise ValueError(f"empty --profile spec: {spec!r}")
    return out


# L2C-family tracking modes (L, S, X) and L5 I-or-combined (Q, X) all
# target the same physical signal, and analysis centers typically
# publish one bias value that covers all tracking variants.  CODE's
# IAR products for 2020/001 specifically publish L2X as the canonical
# L2C attribute and verifiably use **identical** numeric values for
# L2C/L2W/L2X (e.g. G08 all three = 0.70203 ns).  RINEX OBS files,
# however, typically record whichever variant the receiver happened to
# track — L2L on a Septentrio.  Without this fallback, every L2L/L2S
# lookup misses and the harness processes uncorrected phase.
_OSB_ATTR_FALLBACK = {
    'L5Q': ('L5X',), 'L5X': ('L5Q',),
    'L2L': ('L2X', 'L2C'), 'L2S': ('L2X', 'L2C'),
    'L2C': ('L2X',), 'L2X': ('L2C',),
    'C5Q': ('C5X',), 'C5X': ('C5Q',),
    'C2L': ('C2X', 'C2C'), 'C2S': ('C2X', 'C2C'),
    'C2C': ('C2X',), 'C2X': ('C2C',),
    'C1C': ('C1X',), 'C1X': ('C1C',),
    'L1C': ('L1X',), 'L1X': ('L1C',),
}


def _osb_get(osb, sv: str, code: str):
    """OSB lookup with tracking-attribute fallback for CODE-style BIA files."""
    v = osb.get_osb(sv, code)
    if v is not None:
        return v
    for alt in _OSB_ATTR_FALLBACK.get(code, ()):
        v = osb.get_osb(sv, alt)
        if v is not None:
            return v
    return None


def _build_obs_for_filter(rx_obs, gps_time, osb=None):
    """Convert SvObservation list to the dict format PPPFilter.update
    expects (matches realtime_ppp.serial_reader output, including the
    lowercase 'sys' name convention).

    If an OSBParser is supplied, satellite-side code + phase biases are
    subtracted from the raw observations before the IF combination is
    formed — matching what `solve_ppp.load_ppp_epochs` does for the
    RAWX path.  Without this correction, per-SV L1-L5 ISC biases of
    several meters leak into pseudorange residuals."""
    try:
        from solve_ppp import SIG_TO_RINEX
    except ImportError:
        SIG_TO_RINEX = {}
    out = []
    for o in rx_obs:
        # Compute IF combination coefficients
        f1 = C_LIGHT / o.wl_f1
        f2 = C_LIGHT / o.wl_f2
        a1 = f1 * f1 / (f1 * f1 - f2 * f2)
        a2 = -f2 * f2 / (f1 * f1 - f2 * f2)
        pr1 = o.pr1_m
        pr2 = o.pr2_m
        phi1_m = o.phi1_cyc * o.wl_f1
        phi2_m = o.phi2_cyc * o.wl_f2
        if osb is not None:
            rinex_f1 = SIG_TO_RINEX.get(o.f1_sig_name)
            rinex_f2 = SIG_TO_RINEX.get(o.f2_sig_name)
            if rinex_f1 and rinex_f2:
                c1 = _osb_get(osb, o.sv, rinex_f1[0])
                c2 = _osb_get(osb, o.sv, rinex_f2[0])
                if c1 is not None and c2 is not None:
                    pr1 -= c1
                    pr2 -= c2
                p1 = _osb_get(osb, o.sv, rinex_f1[1])
                p2 = _osb_get(osb, o.sv, rinex_f2[1])
                if p1 is not None and p2 is not None:
                    phi1_m -= p1
                    phi2_m -= p2
        pr_if = a1 * pr1 + a2 * pr2
        phi_if_m = a1 * phi1_m + a2 * phi2_m
        out.append({
            'sv': o.sv,
            'sys': _SYS_TO_LOWER.get(o.sys, o.sys.lower()),
            'pr_if': pr_if,
            'phi_if_m': phi_if_m,
            'cno': o.cno,
            'lock_duration_ms': o.lock_duration_ms,
            'half_cyc_ok': o.half_cyc_ok,
            'phi1_cyc': o.phi1_cyc,
            'phi2_cyc': o.phi2_cyc,
            'phi1_raw_cyc': o.phi1_raw_cyc,
            'phi2_raw_cyc': o.phi2_raw_cyc,
            'pr1_m': o.pr1_m,
            'pr2_m': o.pr2_m,
            'wl_f1': o.wl_f1,
            'wl_f2': o.wl_f2,
            'f1_lock_ms': o.f1_lock_ms,
            'f2_lock_ms': o.f2_lock_ms,
            'f1_sig_name': o.f1_sig_name,
            'f2_sig_name': o.f2_sig_name,
        })
    return out


# Profile → IGS ANTEX frequency-id pairs per system.  Keyed by the
# RINEX band/attribute chars we use in extract_dual_freq's profile
# dicts (see rinex_reader.L2_PROFILE / L5_PROFILE).
_PROFILE_FREQ_IDS = {
    ('G', '1C'): 'G01', ('G', '1W'): 'G01', ('G', '1X'): 'G01',
    ('G', '2L'): 'G02', ('G', '2W'): 'G02', ('G', '2X'): 'G02',
    ('G', '5Q'): 'G05', ('G', '5X'): 'G05',
    ('E', '1C'): 'E01', ('E', '1X'): 'E01',
    ('E', '5Q'): 'E05', ('E', '5X'): 'E05',
    ('E', '7Q'): 'E07', ('E', '7X'): 'E07',
    ('C', '2I'): 'C02', ('C', '2X'): 'C02',
    ('C', '7I'): 'C07', ('C', '5P'): 'C05', ('C', '5D'): 'C05',
    ('C', '5X'): 'C05', ('C', '7X'): 'C07',
}


def _compute_pcv_correction(o, sat_pos_ecef, receiver_ecef, antex, ant_type,
                            epoch_t, sun_pos_ecef=None):
    """Compute the IF-combined PCV+PCO range correction for one
    observation dict.

    Returns (Δrange_m, ok) — add Δrange to both pr_if and phi_if_m
    (same signal path, same phase-center geometry) to correct the
    observation from ARP-to-ARP range to MPC-to-MPC range.  ok is
    False when any antenna pattern lookup missed (either freq not in
    ANTEX or receiver antenna type not found) — caller should skip
    applying in that case.
    """
    from regression.antex import ecef_to_enu_matrix, nadir_angle_deg, sat_body_frame
    import math

    sv = o['sv']
    sys_char = sv[0]

    # Map RINEX (band, attr) → ANTEX freq_id for each of f1, f2.  The
    # band/attr comes from o['f1_sig_name'] / o['f2_sig_name'] which
    # are internal names like 'GPS-L1CA'.  Parse the attr from the
    # existing RINEX info stored in wl_f1 / wl_f2 isn't direct, so
    # look it up from the profile map we threaded through.
    # Alternative: use the sig_name suffix.
    f1_name = o.get('f1_sig_name', '')
    f2_name = o.get('f2_sig_name', '')

    # Map internal signal names to ANTEX freq IDs.
    SIG_TO_ANTEX = {
        'GPS-L1CA': 'G01', 'GPS-L1W': 'G01', 'GPS-L1X': 'G01',
        'GPS-L2CL': 'G02', 'GPS-L2W': 'G02', 'GPS-L2X': 'G02',
        'GPS-L5Q': 'G05', 'GPS-L5X': 'G05',
        'GAL-E1C': 'E01', 'GAL-E1X': 'E01',
        'GAL-E5aQ': 'E05', 'GAL-E5aX': 'E05',
        'GAL-E5bQ': 'E07', 'GAL-E5bX': 'E07',
        'BDS-B1I': 'C02', 'BDS-B1X': 'C02',
        'BDS-B2I': 'C07', 'BDS-B2aP': 'C05', 'BDS-B2aD': 'C05',
        'BDS-B2aX': 'C05',
    }
    fid1 = SIG_TO_ANTEX.get(f1_name)
    fid2 = SIG_TO_ANTEX.get(f2_name)
    if fid1 is None or fid2 is None:
        return 0.0, False

    # Geometry at receiver: LOS from receiver up to satellite
    dx = sat_pos_ecef - receiver_ecef
    rho = float(np.linalg.norm(dx))
    if rho < 1.0:
        return 0.0, False
    los_rcv_to_sat = dx / rho
    R_enu = ecef_to_enu_matrix(receiver_ecef)
    los_enu = R_enu @ los_rcv_to_sat
    elev_rad = math.asin(max(-1.0, min(1.0, los_enu[2])))
    elev_deg = math.degrees(elev_rad)
    zen_deg = 90.0 - elev_deg
    az_deg = math.degrees(math.atan2(los_enu[0], los_enu[1])) % 360.0

    # Satellite-side nadir angle
    nadir_deg = nadir_angle_deg(sat_pos_ecef, receiver_ecef)

    # Satellite body-frame unit vectors (nominal yaw-steering) for
    # projecting body-XYZ PCO components onto the LOS.  Caller should
    # pass sun_pos_ecef once per epoch; skip the full-3-axis projection
    # if absent (falls back to U-only).
    e_x = e_y = e_z = None
    los_sat_to_rcv_body = None
    if sun_pos_ecef is not None:
        e_x, e_y, e_z = sat_body_frame(sat_pos_ecef, sun_pos_ecef)
        los_sat_to_rcv_ecef = (receiver_ecef - sat_pos_ecef) / rho
        los_sat_to_rcv_body = np.array([
            float(np.dot(los_sat_to_rcv_ecef, e_x)),
            float(np.dot(los_sat_to_rcv_ecef, e_y)),
            float(np.dot(los_sat_to_rcv_ecef, e_z)),
        ])

    # Per-freq contributions.  Satellite-side: simplified nominal-yaw
    # (project PCO_U along nadir axis; ignore body N/E because true
    # yaw angle needs an attitude model we don't carry).  Receiver-
    # side: full 3-component PCO_enu projection onto los_enu, plus
    # PCV(zen, az).
    corrs = []
    for fid in (fid1, fid2):
        sat_pat = antex.get_sat_pattern_fallback(sv, fid, epoch_t)
        rcv_pat = antex.get_recv_pattern(ant_type, fid)  # already has fallback
        if sat_pat is None or rcv_pat is None:
            return 0.0, False
        # Range correction to ADD to observed phase/pseudorange so it
        # represents ARP-to-ARP geometry (matching what the filter's
        # rho_pred computes).  Sign: if MPC is closer to the other end
        # than ARP is, the observed range is SHORTER than ARP-to-ARP,
        # so we need to ADD the offset to get ARP range.
        # Satellite side: project PCO_body onto LOS_sat_to_rcv_body.
        #   ANTEX "NORTH" = body-X, "EAST" = body-Y, "UP" = body-Z.
        #   If sun direction isn't available (no ephemeris), fall back
        #   to U-only projection via cos(nadir); this approximates the
        #   dominant term (body-Z is typically 1-2 m vs dm N/E).
        sat_pco = sat_pat.pco_m  # [X_body, Y_body, Z_body] in meters
        if los_sat_to_rcv_body is not None:
            sat_pco_proj = float(np.dot(sat_pco, los_sat_to_rcv_body))
        else:
            sat_pco_proj = sat_pco[2] * math.cos(math.radians(nadir_deg))
        sat_term = sat_pco_proj + sat_pat.pcv(nadir_deg)
        # Receiver: PCO in ENU, LOS_rcv_to_sat in ENU.  Projection
        #   dot(PCO_enu, LOS_enu) is how much MPC is offset along LOS.
        #   Add to obs to get ARP range.
        rcv_pco_enu = rcv_pat.pco_m  # (N, E, U)
        # ANTEX line 1 is North, East, Up.  Our ecef_to_enu returns
        # rows [E, N, U] so los_enu = [E, N, U].  Align accordingly:
        rcv_dot = (rcv_pco_enu[0] * los_enu[1]   # N_pco × N_los
                   + rcv_pco_enu[1] * los_enu[0]   # E_pco × E_los
                   + rcv_pco_enu[2] * los_enu[2])  # U_pco × U_los
        rcv_term = rcv_dot + rcv_pat.pcv(zen_deg, az_deg)
        corrs.append(sat_term + rcv_term)

    # IF combine using the coefficients from the obs (wavelengths
    # already stored).  α₁ = f1² / (f1² − f2²), α₂ = −f2² / (f1² − f2²).
    f1_hz = C_LIGHT / o.get('wl_f1', 1.0)
    f2_hz = C_LIGHT / o.get('wl_f2', 1.0)
    denom = f1_hz * f1_hz - f2_hz * f2_hz
    if abs(denom) < 1.0:
        return 0.0, False
    a1 = f1_hz * f1_hz / denom
    a2 = -f2_hz * f2_hz / denom
    # pr/phi_if = a1*f1 + a2*f2 so correction combines same way
    corr_if = a1 * corrs[0] + a2 * corrs[1]
    return corr_if, True


def _apply_tie_updates(filt, ztd_tie_sigma: Optional[float],
                       mean_amb_tie_sigma: Optional[float]) -> None:
    """Apply optional ZTD and mean-ambiguity pseudo-measurements as
    rank-1 EKF updates on the filter state after the regular
    observation update.

    Motivation: the (rx_clock, ZTD·m_wet, mean-ambiguity) triple
    forms a near-null vector at some receiver geometries (GPS+GAL
    at ABMF 2020/001 is the documented case).  Observations alone
    can't constrain the null direction, so the filter oscillates in
    that subspace and position error grows.  These pseudo-
    measurements constrain two of the three null-vector components
    directly.

    Both args default to None (tie disabled).  σ units are metres.
    Recommended values from the harness prototype (ABMF 2020/001):

    - ztd_tie_sigma=0.05 (5 cm): bounds the null-mode ZTD wander
      while accommodating the empirically observed real atmospheric
      ZTD variation of 18 cm / 24 h at ABMF (σ ~6 cm).
    - mean_amb_tie_sigma=0.10 (10 cm): only enable if ztd_tie alone
      isn't enough — weaker physical justification (mean N legitimately
      drifts with visible-SV set as satellites rise and set).

    See `project_to_main_pride_gps_filter_degeneracy_20260423` and
    `project_to_main_ztd_pseudomeasurement_proposal_20260423`.
    """
    # Late imports keep the helper testable without engine deps.
    from solve_ppp import N_BASE, IDX_ZTD

    if ztd_tie_sigma is not None and ztd_tie_sigma > 0:
        # H = unit row at IDX_ZTD; z = 0; R = sigma^2
        # y = 0 - x[IDX_ZTD]
        # S = P[IDX_ZTD, IDX_ZTD] + R
        # K = P[:, IDX_ZTD] / S
        # x += K * y
        # P -= outer(K, P[IDX_ZTD, :])
        y = -float(filt.x[IDX_ZTD])
        S = float(filt.P[IDX_ZTD, IDX_ZTD]) + ztd_tie_sigma ** 2
        K = filt.P[:, IDX_ZTD] / S
        filt.x = filt.x + K * y
        filt.P = filt.P - np.outer(K, filt.P[IDX_ZTD, :])

    if mean_amb_tie_sigma is not None and mean_amb_tie_sigma > 0:
        n = len(filt.x)
        n_amb = n - N_BASE
        if n_amb == 0:
            return
        # H = (1/n_amb) at each ambiguity index, 0 elsewhere
        H = np.zeros(n)
        H[N_BASE:] = 1.0 / n_amb
        HP = H @ filt.P                       # 1 x n
        y = -float(np.mean(filt.x[N_BASE:]))  # 0 - mean(amb)
        S = float(HP @ H) + mean_amb_tie_sigma ** 2
        K = (filt.P @ H) / S                  # n
        filt.x = filt.x + K * y
        filt.P = filt.P - np.outer(K, HP)


def run(args) -> int:
    """Run one regression scenario.  Returns process exit code."""
    # Late imports so the module is importable without engine deps
    from broadcast_eph import BroadcastEphemeris
    from solve_ppp import (
        PPPFilter, ls_init, ecef_to_enu,
        N_BASE, IDX_CLK, IDX_ISB_GAL, IDX_ISB_BDS, IDX_ZTD,
    )
    from ppp_ar import MelbourneWubbenaTracker

    truth_ecef = _parse_truth(args.truth)
    try:
        profile = _parse_profile(args.profile)
    except ValueError as e:
        log.error("%s", e)
        return 2
    # Filter-tuning overrides: apply before any PPPFilter is built so the
    # class attribute is live when .predict() / .update() look it up.
    q_pos_override = getattr(args, "q_pos_converged", None)
    if q_pos_override is not None:
        from solve_ppp import PPPFilter
        PPPFilter.Q_POS_CONVERGED = q_pos_override
        log.info("PPPFilter.Q_POS_CONVERGED overridden: %.3e (default 1e-4)",
                 q_pos_override)
    sig_pr_override = getattr(args, "sigma_pr", None)
    if sig_pr_override is not None:
        import solve_ppp as _sp
        _sp._SIGMA_P_IF_OVERRIDE = sig_pr_override
        log.info("SIGMA_P_IF overridden: %.3f m (default 3.0)", sig_pr_override)
    sig_phi_override = getattr(args, "sigma_phi", None)
    if sig_phi_override is not None:
        import solve_ppp as _sp
        _sp._SIGMA_PHI_IF_OVERRIDE = sig_phi_override
        log.info("SIGMA_PHI_IF overridden: %.4f m (default 0.03)", sig_phi_override)
    ou_tau = getattr(args, "ztd_ou_tau", None)
    ou_sig = getattr(args, "ztd_ou_sigma", None)
    if (ou_tau is not None) ^ (ou_sig is not None):
        log.error("--ztd-ou-tau and --ztd-ou-sigma must both be set or both omitted")
        return 2
    if ou_tau is not None and ou_sig is not None:
        from solve_ppp import PPPFilter
        PPPFilter.ZTD_OU_TAU_S = ou_tau
        PPPFilter.ZTD_OU_SIGMA_STEADY_M = ou_sig
        log.info("ZTD OU process: τ=%.0f s (%.2f h), σ_steady=%.3f m",
                 ou_tau, ou_tau / 3600.0, ou_sig)
    wl_only = bool(getattr(args, "wl_only", False))
    position_csv_path = getattr(args, "position_csv", None)
    position_csv_writer = None
    position_csv_fh = None
    if position_csv_path:
        import csv
        position_csv_fh = open(position_csv_path, "w", newline="")
        position_csv_writer = csv.writer(position_csv_fh)
        # Headers: epoch index, UTC timestamp, per-axis ECEF
        # error (m), per-axis ENU error (m), 3D / H / V norms,
        # SV counts so we can correlate residual shape with
        # geometry.  This is the direct per-epoch bias trace
        # for the Q1 systematic-bias question.
        position_csv_writer.writerow([
            "ep_idx", "utc",
            "err_ecef_x", "err_ecef_y", "err_ecef_z",
            "err_e", "err_n", "err_u",
            "err_3d", "err_h", "err_v",
            "n_used", "n_filter_svs", "n_wl_fixed",
        ])

    state_csv_path = getattr(args, "state_csv", None)
    state_csv_writer = None
    state_csv_fh = None
    if state_csv_path:
        import csv
        state_csv_fh = open(state_csv_path, "w", newline="")
        state_csv_writer = csv.writer(state_csv_fh)
        # Full filter state snapshot per epoch.  Used to diff
        # the NAV-path vs SP3-path filter trajectories on
        # identical observations — the divergence point tells
        # us which state component carries the GPS-SP3 bug.
        # pos_x/y/z are filter state (not error vs truth).
        # amb_* columns summarize the ambiguity block so we
        # can watch for wild drift there too.
        state_csv_writer.writerow([
            "ep_idx", "utc",
            "pos_x", "pos_y", "pos_z",
            "clk_m", "isb_gal_m", "isb_bds_m", "ztd_m",
            "n_amb", "amb_mean_m", "amb_std_m",
            "err_3d_m",
            # Reported uncertainty (Q2 σ-trust analysis): filter's
            # diagonal P entries expressed as σ.  Compare to
            # err_3d_m to test whether reported σ envelopes the
            # empirical error vs truth.  sigma_3d is the 3D
            # position sigma sqrt(trace(P_pos)) — a filter that's
            # "strong and wrong" reports tiny sigma_3d while
            # err_3d_m stays large.
            "sigma_3d_m", "sigma_clk_m", "sigma_ztd_m",
        ])

    residuals_csv_path = getattr(args, "residuals_csv", None)
    residuals_csv_writer = None
    residuals_csv_fh = None
    if residuals_csv_path:
        import csv
        residuals_csv_fh = open(residuals_csv_path, "w", newline="")
        residuals_csv_writer = csv.writer(residuals_csv_fh)
        # Per-measurement row at every processed epoch.  Each
        # filt.update() call yields one row per PR measurement
        # and one per phase measurement per SV used.  Aligned
        # with `filt.last_residual_labels` (sv, kind, elev) so we
        # can do per-SV + per-signal analysis post-run:
        # clusters of same-signed residuals → common-mode
        # (reference frame / clock); per-SV scatter → per-SV
        # bias table (DCB / TGD / ISC).
        residuals_csv_writer.writerow([
            "ep_idx", "utc", "sv", "sys", "kind", "elev_deg",
            "post_resid_m",
        ])

    # Header — gives us the receiver's APPROX POSITION as seed if no
    # explicit seed; gives us the observation interval too.
    obs_path = Path(args.obs)
    obs_hdr = parse_obs_header(obs_path)
    interval_s = obs_hdr.interval_s or 30.0

    # Ephemeris source: SP3 precise orbits when available (sub-cm
    # accuracy), broadcast NAV otherwise (~1–2 m).  Both provide the
    # same `sat_position(sv, t) → (pos, clk)` interface, so the filter
    # doesn't care which it gets.
    if args.sp3:
        from solve_pseudorange import SP3
        sp3 = SP3(args.sp3)
        log.info("Loaded SP3: %d epochs, %d SVs",
                 len(sp3.epochs), len(sp3.positions))
        eph_source = sp3
    else:
        nav_path = Path(args.nav) if args.nav else None
        if nav_path is None:
            log.error("must provide --nav or --sp3")
            return 2
        beph = BroadcastEphemeris()
        n_eph = load_into_ephemeris(nav_path, beph)
        log.info("Loaded %d broadcast ephemeris records (%d SVs)",
                 n_eph, beph.n_satellites)
        eph_source = beph

    # Optional high-rate satellite clock file.  30 s RINEX CLK files
    # from analysis centers override the 300 s SP3 clocks with ~30–50 ps
    # accuracy — essential for sub-dm PPP since the 300 s SP3 clock
    # interpolation error can be several ns of pseudorange.
    clk_file = None
    if args.clk:
        from ppp_corrections import CLKFile
        clk_file = CLKFile(args.clk)
        log.info("Loaded CLK: %d SVs", len(clk_file._t0))

    # Optional satellite-side code + phase bias file (Bias-SINEX OSB).
    # CODE, WUM, and CNES all publish these; applying them removes the
    # per-SV L1-L5 ISC biases and (for phase) enables PPP-AR downstream.
    osb = None
    if args.bia:
        from ppp_corrections import OSBParser
        osb = OSBParser(args.bia)
        log.info("Loaded OSB: %d (PRN, signal) bias entries across %d SVs",
                 len(osb.biases), len(osb.prns()))

    # Optional ANTEX PCV/PCO correction file.  When --antex is set, the
    # harness applies satellite + receiver phase-center corrections at
    # observation ingest.  The receiver antenna type is read from the
    # RINEX OBS file's ANT # / TYPE record.
    antex = None
    recv_ant_type = None
    if getattr(args, "antex", None):
        from regression.antex import ANTEXParser
        antex = ANTEXParser(args.antex)
        log.info("Loaded ANTEX: %d sat keys, %d rcv keys",
                 len(antex.sat_patterns), len(antex.recv_patterns))
        # Read receiver antenna type from RINEX OBS header
        with open(args.obs, encoding='latin-1') as _f:
            for _line in _f:
                if _line[60:80].rstrip() == 'ANT # / TYPE':
                    recv_ant_type = _line[20:40].rstrip()
                    break
                if 'END OF HEADER' in _line:
                    break
        if recv_ant_type:
            log.info("Receiver antenna type: %r", recv_ant_type)
        else:
            log.warning("Could not read ANT # / TYPE from %s; PCV disabled",
                        args.obs)
            antex = None

    # Filter is initialised lazily on the first usable epoch — we use
    # ls_init() to seed both position AND receiver clock from that
    # epoch's pseudoranges.  Seeding clock=0 (the previous behavior)
    # leaves the filter facing a microsecond-to-millisecond receiver
    # clock bias on every observation, which it rejects as outliers
    # before its EKF can converge.
    filt: Optional[PPPFilter] = None
    systems_lower = {_SYS_TO_LOWER.get(s, s.lower()) for s in profile.keys()}
    # Optional per-constellation gate — lets us isolate which
    # constellation drives any systematic bias.  --systems gps,gal
    # keeps only GPS and Galileo observations; SVs from other
    # constellations are dropped at _build_obs_for_filter time.
    # Name convention matches the engine's --systems flag.
    systems_filter: Optional[set[str]] = None
    if getattr(args, "systems", None):
        systems_filter = {
            _SYS_TO_LOWER.get(s.strip().upper(), s.strip().lower())
            for s in args.systems.split(",")
        }
        # Also narrow the filter-init set so the filter doesn't
        # allocate ISB states for systems we're skipping.
        systems_lower = systems_lower & systems_filter
        log.info("Systems filter: %s (profile had %s)",
                 sorted(systems_filter),
                 sorted({_SYS_TO_LOWER.get(s, s.lower())
                         for s in profile.keys()}))

    # Rank-deficiency gate: single-constellation + smooth precise
    # clocks (SP3/CLK) reliably falls into a near-singular drift mode
    # on static receivers.  The (rx_clock, ZTD·m_wet, mean-ambiguity)
    # triple forms a near-null vector that smooth SP3 clocks cannot
    # break; NAV's 2-hour polynomial discontinuities inject the rank
    # information that breaks it.  Observed at ABMF 2020/001: GPS-only
    # SP3+CLK drifts 15+ m over 50 min while GAL-only is identical and
    # GPS+GAL is <50 cm.  See
    # `project_to_main_pride_gps_filter_degeneracy_20260423`.
    if args.sp3 and len(systems_lower) < 2:
        log.error(
            "Single-constellation %s with --sp3 falls into a "
            "rank-deficient drift mode (15+ m over 50 min).  Use "
            "--systems with ≥ 2 constellations when --sp3 is "
            "active, or switch to --nav for single-constellation "
            "runs (NAV page-boundary discontinuities break the "
            "null mode).  See "
            "project_to_main_pride_gps_filter_degeneracy_20260423.",
            sorted(systems_lower),
        )
        return 2

    seed_offset: Optional[float] = None

    # Melbourne-Wubbena wide-lane tracker.  Per-SV WL integer fixing
    # plus jump (cycle-slip) detection.  In WL-only / float mode we
    # don't apply any pseudo-measurement to the float IF state from
    # MW — the win is purely from cycle-slip detection: when MW
    # detects a jump on an SV, we drop and re-add that SV's
    # ambiguity in the float filter so the slip doesn't poison the
    # float estimate for the rest of the run.
    #
    # Engine reference: `peppar_fix_engine.py:1911-1928` for the
    # MW.update call, `:1929-1980` for slip handling on already-
    # WL-fixed SVs (post-fix drift monitor).  We adapt the same
    # pattern but skip the drift monitor — for the harness the
    # simpler `MelbourneWubbenaTracker.detect_jump` is enough.
    mw = MelbourneWubbenaTracker()
    n_wl_fixed_max = 0       # high-water mark of concurrent WL fixes
    n_slip_resets = 0        # SV ambiguity resets due to MW jump
    if wl_only:
        log.info("WL-only mode: MW slip detection on, no NL constraint")

    # Iterate epochs
    prev_t = None
    n_processed = 0
    n_skipped_empty = 0
    n_skipped_too_few = 0
    last_pos = truth_ecef
    lock_accum: dict = {}
    # Convergence-checkpoint reporting: drop a one-line per-epoch
    # summary at each of these processed-epoch counts so the gate
    # ladder is self-documenting.  FINAL is reported at end.  The
    # epochs are spaced log-style — early epochs converge fast,
    # later epochs reveal long-tail biases.
    checkpoint_epochs = sorted({100, 500, 1000, 2000, 4000, 8000, 16000})
    checkpoint_results: list[tuple[int, float, float, float]] = []

    for ep_idx, ep in enumerate(iter_epochs(obs_path)):
        if args.max_epochs and ep_idx >= args.max_epochs:
            break

        t = ep.ts.replace(tzinfo=timezone.utc)
        sv_obs_list = extract_dual_freq(
            ep, profile=profile, interval_s=interval_s,
            lock_accum=lock_accum,
        )
        if not sv_obs_list:
            n_skipped_empty += 1
            continue

        observations = _build_obs_for_filter(sv_obs_list, t, osb=osb)
        if systems_filter is not None:
            observations = [o for o in observations
                            if o['sys'] in systems_filter]
            if not observations:
                n_skipped_empty += 1
                continue

        # First-usable-epoch bootstrap via ls_init: solves for
        # position + receiver-clock offset from the IF pseudoranges
        # alone.  Without this seed, the filter starts with clk=0
        # but the real receiver carries a μs–ms clock bias that
        # shows up as huge per-SV pseudorange residuals.
        if filt is None:
            try:
                ls_result, ls_ok, ls_n = ls_init(
                    observations, eph_source, t, clk_file=clk_file,
                )
            except Exception as e:
                log.warning("ls_init failed at epoch %d: %s", ep_idx, e)
                continue
            if not ls_ok or ls_n < 4:
                log.debug("ls_init not converged at epoch %d (ok=%s n=%d)",
                          ep_idx, ls_ok, ls_n)
                continue
            init_ecef = np.array(ls_result[:3])
            init_clk = float(ls_result[3])
            seed_offset = float(np.linalg.norm(init_ecef - truth_ecef))
            log.info("ls_init bootstrap: pos=%s, clk=%.3e s "
                     "(%.2f m from truth, n_used=%d)",
                     init_ecef.tolist(), init_clk / C_LIGHT,
                     seed_offset, ls_n)
            filt = PPPFilter()
            filt.initialize(init_ecef, init_clk, systems=systems_lower)

        # Filter prediction step
        if prev_t is not None:
            dt = (t - prev_t).total_seconds()
            if dt > 0:
                filt.predict(dt)
        prev_t = t

        # Ambiguity management.  Must happen BEFORE filt.update
        # so phase observations contribute (the update loop
        # gates `if sv in self.sv_to_idx` for phase).  Mirrors
        # what `solve_ppp.__main__`'s steady-state loop does at
        # lines 1172-1178; the existing harness lacked this so
        # it was silently running PR-only PPP.
        current_svs = {o['sv'] for o in observations}
        tracked = set(filt.sv_to_idx.keys())
        # SVs that vanished this epoch — drop their ambiguity
        # state so stale float values don't linger as SV rises
        # again (cycle slip + arc gap semantics are handled via
        # MW's detect_jump + explicit remove above).
        for sv in tracked - current_svs:
            filt.remove_ambiguity(sv)
        # New SVs this epoch — seed the ambiguity from the first
        # phase observation.  N_init = phi_if - pr_if is the
        # standard cold-start for a float IF ambiguity
        # (meters), leverages the PR estimate of range so the
        # initial phase ambiguity is close to truth.
        for o in observations:
            if (o['sv'] not in filt.sv_to_idx
                    and o.get('phi_if_m') is not None):
                filt.add_ambiguity(
                    o['sv'], o['phi_if_m'] - o['pr_if'],
                )

        # MW wide-lane update (per SV).  The pre-update step here
        # mirrors the engine's order: MW first so the slip detector
        # sees current observations against the pre-update average,
        # then filter update absorbs the (possibly slip-flushed)
        # observations.  Slip detection on an SV that's already in
        # the float filter triggers a remove + re-add: dropping the
        # ambiguity flushes its float estimate so the post-slip
        # observations don't anchor against a now-wrong integer.
        # Filter re-adds the SV's ambiguity on the next observation
        # at line 502-503 of solve_ppp.py.
        mw._current_epoch = ep_idx
        slip_resets_this_ep = 0
        for o in observations:
            sv = o['sv']
            phi1 = o.get('phi1_cyc')
            phi2 = o.get('phi2_cyc')
            pr1 = o.get('pr1_m')
            pr2 = o.get('pr2_m')
            wl1 = o.get('wl_f1')
            wl2 = o.get('wl_f2')
            if not all(v is not None for v in (phi1, phi2, pr1, pr2, wl1, wl2)):
                continue
            f1_hz = C_LIGHT / wl1
            f2_hz = C_LIGHT / wl2
            # Slip detection BEFORE update: detect_jump compares the
            # incoming MW sample against the existing tracker state;
            # a real cycle slip lands many σ outside the rolling
            # residual window.
            try:
                jump = mw.detect_jump(o)
            except Exception:
                jump = None
            jumped = bool(jump and jump.get('is_slip'))
            if jumped and sv in filt.sv_to_idx:
                # Reset MW state and remove the ambiguity from the
                # filter.  Filter re-adds with a fresh float estimate
                # on the next phase observation for this SV.
                mw.reset(sv)
                filt.remove_ambiguity(sv)
                slip_resets_this_ep += 1
                n_slip_resets += 1
            mw.update(sv, phi1, phi2, pr1, pr2, f1_hz, f2_hz)

        # Track WL-fix high-water mark for diagnostics.
        n_wl_now = mw.n_fixed
        if n_wl_now > n_wl_fixed_max:
            n_wl_fixed_max = n_wl_now

        # Filter update — eph_source supplies sat_position which returns
        # (pos, clk).  clk_file overrides the clock when given (high-rate
        # CLK product); otherwise the filter uses the clock value from
        # sat_position.
        # PCV/PCO correction: adjust phi_if and pr_if for each SV before
        # the filter sees them.  Skip silently per-SV on ANTEX lookup
        # miss (e.g. BDS SVs when ANTEX lacks entries).
        if antex is not None and recv_ant_type is not None:
            from datetime import timedelta as _td
            from solve_pseudorange import C as _C_PR
            # Per-epoch Sun position (ECEF) for nominal-yaw body frame.
            # Reused across all SVs at this epoch.
            from regression.solid_tide import (_jd_from_datetime,
                                                _gmst_rad,
                                                _sun_pos_eci,
                                                _eci_to_ecef)
            _jd = _jd_from_datetime(t)
            _gmst = _gmst_rad(_jd)
            _sun_ecef = _eci_to_ecef(_sun_pos_eci(_jd), _gmst)
            n_pcv_applied = 0
            rcv_pos_now = filt.x[:3] if filt is not None else truth_ecef
            for o in observations:
                tau_approx = o['pr_if'] / _C_PR if 'pr_if' in o else 0.075
                t_tx = t - _td(seconds=tau_approx)
                sat_pos, _ = eph_source.sat_position(o['sv'], t_tx)
                if sat_pos is None:
                    continue
                delta, ok = _compute_pcv_correction(
                    o, sat_pos, rcv_pos_now, antex, recv_ant_type, t,
                    sun_pos_ecef=_sun_ecef,
                )
                if ok:
                    o['pr_if'] += delta
                    if o.get('phi_if_m') is not None:
                        o['phi_if_m'] += delta
                    n_pcv_applied += 1
            if ep_idx == 0 or (ep_idx % 500 == 0 and n_pcv_applied > 0):
                log.debug("PCV applied to %d SVs at epoch %d", n_pcv_applied, ep_idx)

        # Solid Earth tide: when --solid-tide is enabled, the filter
        # estimates the ITRF position but observations see an
        # instantaneous position displaced by up to ~150 mm vertical.
        # Compute the displacement at this epoch against the current
        # filter position (accuracy of that position is irrelevant to
        # sub-µm precision of the SET formula).
        set_offset = None
        if getattr(args, "solid_tide", False):
            from regression.solid_tide import solid_tide_displacement
            set_offset = solid_tide_displacement(t, filt.x[:3])

        try:
            n_used, resid, sys_counts = filt.update(
                observations, eph_source, t, clk_file=clk_file,
                receiver_offset_ecef=set_offset,
            )
        except Exception as e:
            log.error("filt.update failed at epoch %d (%s): %s",
                      ep_idx, t, e)
            continue

        if n_used < 4:
            n_skipped_too_few += 1
            continue

        # Optional pseudo-measurement ties.  Applied after the regular
        # observation update so they act as a prior constraint on the
        # post-observation state rather than competing with it in a
        # joint update.  Both default-off; see `_apply_tie_updates`.
        ztd_tie = getattr(args, "ztd_tie", None)
        mean_amb_tie = getattr(args, "mean_amb_tie", None)
        if ztd_tie or mean_amb_tie:
            _apply_tie_updates(filt, ztd_tie, mean_amb_tie)

        n_processed += 1
        last_pos = filt.x[:3].copy()

        # Per-SV residual dump.  Runs after filt.update so the
        # residuals are post-fit.  Labels are (sv, kind, elev)
        # and align with the `resid` vector returned above.
        # Matching labels against resid is O(n_used); n_used is
        # typically ~10-25, so per-epoch overhead is negligible.
        if residuals_csv_writer is not None:
            labels_out = getattr(filt, "last_residual_labels", [])
            for (sv, kind, elev), r in zip(labels_out, resid):
                residuals_csv_writer.writerow([
                    ep_idx, t.strftime("%Y-%m-%dT%H:%M:%S"),
                    sv, sv[0], kind, f"{elev:.1f}",
                    f"{float(r):.4f}",
                ])

        # Per-epoch error tracking.  ENU decomposition anchored at
        # the truth point (not the filter estimate) so the ENU
        # values are the true east / north / up components of our
        # bias, not of our uncertainty ellipse.
        err_ecef = last_pos - truth_ecef
        err_enu = ecef_to_enu(err_ecef, truth_ecef)

        if state_csv_writer is not None:
            amb_slice = filt.x[N_BASE:] if len(filt.x) > N_BASE else np.array([])
            # Diagonal σ extraction: sqrt of variance on each scalar
            # state; for the 3D position we use sqrt of the trace of
            # the position block (sum of per-axis variances).
            sigma_3d = float(np.sqrt(
                filt.P[0, 0] + filt.P[1, 1] + filt.P[2, 2]
            ))
            sigma_clk = float(np.sqrt(filt.P[IDX_CLK, IDX_CLK]))
            sigma_ztd = float(np.sqrt(filt.P[IDX_ZTD, IDX_ZTD]))
            state_csv_writer.writerow([
                ep_idx, t.strftime("%Y-%m-%dT%H:%M:%S"),
                f"{filt.x[0]:.4f}", f"{filt.x[1]:.4f}", f"{filt.x[2]:.4f}",
                f"{filt.x[IDX_CLK]:.4f}",
                f"{filt.x[IDX_ISB_GAL]:.4f}",
                f"{filt.x[IDX_ISB_BDS]:.4f}",
                f"{filt.x[IDX_ZTD]:.4f}",
                len(amb_slice),
                f"{amb_slice.mean():.4f}" if len(amb_slice) else "0",
                f"{amb_slice.std():.4f}" if len(amb_slice) else "0",
                f"{float(np.linalg.norm(err_ecef)):.4f}",
                f"{sigma_3d:.4f}", f"{sigma_clk:.4f}", f"{sigma_ztd:.4f}",
            ])

        if position_csv_writer is not None:
            position_csv_writer.writerow([
                ep_idx, t.strftime("%Y-%m-%dT%H:%M:%S"),
                f"{err_ecef[0]:.4f}", f"{err_ecef[1]:.4f}",
                f"{err_ecef[2]:.4f}",
                f"{err_enu[0]:.4f}", f"{err_enu[1]:.4f}",
                f"{err_enu[2]:.4f}",
                f"{float(np.linalg.norm(err_ecef)):.4f}",
                f"{float(np.linalg.norm(err_ecef[:2])):.4f}",
                f"{float(abs(err_ecef[2])):.4f}",
                n_used, len(filt.sv_to_idx), n_wl_now,
            ])

        # Convergence-checkpoint capture: snapshot the position error
        # at each milestone so the gate ladder reports show how the
        # error decays over time.
        while checkpoint_epochs and n_processed == checkpoint_epochs[0]:
            checkpoint_results.append((
                n_processed,
                float(np.linalg.norm(err_ecef)),
                float(np.linalg.norm(err_ecef[:2])),
                float(abs(err_ecef[2])),
            ))
            checkpoint_epochs.pop(0)

        if n_processed == 1 or n_processed % 20 == 0:
            err_h = float(np.linalg.norm(err_ecef[:2]))
            err_v = float(abs(err_ecef[2]))
            slip_frag = (f" slips={slip_resets_this_ep}"
                         if slip_resets_this_ep else "")
            log.info("epoch %4d  t=%s  n_used=%2d  err_h=%6.2fm "
                     "err_v=%6.2fm  wl_fixed=%d/%d%s",
                     ep_idx, t.strftime("%H:%M:%S"), n_used, err_h, err_v,
                     n_wl_now, len(filt.sv_to_idx), slip_frag)

    if position_csv_fh is not None:
        position_csv_fh.close()
        log.info("Wrote per-epoch position errors to %s", position_csv_path)

    if residuals_csv_fh is not None:
        residuals_csv_fh.close()
        log.info("Wrote per-SV residuals to %s", residuals_csv_path)

    if state_csv_fh is not None:
        state_csv_fh.close()
        log.info("Wrote per-epoch filter state to %s", state_csv_path)

    # Final assessment
    err = last_pos - truth_ecef
    err_3d = float(np.linalg.norm(err))
    err_h = float(np.linalg.norm(err[:2]))
    err_v = float(abs(err[2]))

    print(f"\n{'=' * 60}")
    print(f"Regression result")
    print(f"{'=' * 60}")
    print(f"Profile:           {args.profile}")
    print(f"AR mode:           {'wl-only' if wl_only else 'float'}")
    print(f"Epochs processed:  {n_processed}")
    print(f"Epochs skipped:    {n_skipped_empty} (empty), "
          f"{n_skipped_too_few} (too-few-SVs)")
    print(f"Initial seed err:  "
          f"{seed_offset:.3f} m" if seed_offset is not None else "n/a")
    print(f"Max concurrent WL: {n_wl_fixed_max}")
    print(f"MW slip resets:    {n_slip_resets}")
    if checkpoint_results:
        print(f"\nConvergence ladder (3D / H / V error in m):")
        for n_ep, e3, eh, ev in checkpoint_results:
            print(f"  @ {n_ep:>5d} ep:  {e3:7.3f}  /  {eh:7.3f}  /  {ev:7.3f}")
    print(f"\nFinal position:    {last_pos.tolist()}")
    print(f"Truth position:    {truth_ecef.tolist()}")
    print(f"Final error 3D:    {err_3d:.3f} m")
    print(f"Final error H:     {err_h:.3f} m")
    print(f"Final error V:     {err_v:.3f} m")
    print(f"Tolerance:         {args.tolerance_m:.3f} m (3D)")
    if n_processed == 0:
        print("FAIL — no epochs processed (check NAV file, observation "
              "format, or systems-filter settings)")
        return 2
    if err_3d <= args.tolerance_m:
        print("PASS")
        return 0
    print("FAIL")
    return 1


def main():
    ap = argparse.ArgumentParser(
        description="Run a regression scenario through PePPAR Fix's PPP pipeline"
    )
    ap.add_argument("--obs", required=True,
                    help="RINEX 3.x OBS file (PRIDE-PPPAR or IGS MGEX)")
    ap.add_argument("--nav", default=None,
                    help="RINEX 3.x NAV file (broadcast ephemeris).  "
                         "Either --nav or --sp3 must be provided.")
    ap.add_argument("--sp3", default=None,
                    help="SP3 precise orbit file (e.g. CODE com20863.eph).  "
                         "If provided, overrides --nav as the orbit source "
                         "and gives sub-cm satellite position accuracy.")
    ap.add_argument("--clk", default=None,
                    help="RINEX CLK file with high-rate precise clocks "
                         "(e.g. CODE com20863.clk at 30 s).  Overrides the "
                         "clock values from --sp3 / --nav.  Required for "
                         "sub-dm results.")
    ap.add_argument("--bia", default=None,
                    help="Optional Bias-SINEX OSB file")
    ap.add_argument("--truth", required=True,
                    help="Truth ECEF position 'X,Y,Z' in meters")
    ap.add_argument("--tolerance-m", type=float, default=5.0,
                    help="3D position-error tolerance in meters (default 5)")
    ap.add_argument("--profile", default="l5",
                    help="Receiver profile selection.  Two forms: "
                         "(1) uniform — 'l5' (F9T-L5: L1CA+L5Q per-"
                         "system) or 'l2' (F9T-L2: L1CA+L2CL per-"
                         "system) applies one profile to every "
                         "constellation; (2) per-constellation — "
                         "e.g. 'gps:l2,gal:l5,bds:l5' assigns a "
                         "different profile to each listed "
                         "constellation.  Per-constellation is the "
                         "fix for GPS-on-L5-profile vs CODE's "
                         "IF(L1,L2) GPS clocks — GPS must use L2 "
                         "even when GAL/BDS use L5.  Omitted "
                         "constellations drop their observations.")
    ap.add_argument("--max-epochs", type=int, default=None,
                    help="Limit epoch count for quick runs (default: full file)")
    ap.add_argument("--systems", default=None,
                    help="Comma-separated constellation filter "
                         "(e.g. 'gps', 'gal', 'gps,gal').  "
                         "Drops observations from constellations "
                         "not in the list before they reach the "
                         "filter.  Default: all constellations "
                         "in the active --profile.  Used to "
                         "isolate per-constellation systematic "
                         "bias signatures.  With --sp3, the runner "
                         "refuses single-constellation filters "
                         "because (rx_clock, ZTD, mean-ambiguity) "
                         "form a near-null mode under smooth "
                         "precise clocks.  Use --nav for single-"
                         "constellation diagnostics.")
    ap.add_argument("--state-csv", default=None,
                    help="If set, write one row per processed "
                         "epoch with the full filter state "
                         "(position, clock, ISBs, ZTD, ambiguity "
                         "summary, 3D error vs truth).  Used to "
                         "diff NAV vs SP3 filter trajectories to "
                         "isolate where they diverge.")
    ap.add_argument("--residuals-csv", default=None,
                    help="If set, write one row per per-SV-per-"
                         "epoch post-fit residual to this CSV.  "
                         "Columns: ep_idx, utc, sv, sys, kind "
                         "(pr/phi), elev_deg, post_resid_m.  "
                         "Use to diff per-SV signatures between "
                         "constellations and pin down systematic "
                         "biases (common-mode = reference-frame "
                         "issue; per-SV scatter = DCB/TGD/ISC).")
    ap.add_argument("--position-csv", default=None,
                    help="If set, write one row per processed "
                         "epoch to this CSV file.  Columns: "
                         "ep_idx, utc, ECEF error (x, y, z), ENU "
                         "error (e, n, u), 3D / H / V norms, SV "
                         "counts.  ENU is anchored at the truth "
                         "point — the values are the systematic "
                         "bias, east / north / up, of our "
                         "solution vs ITRF14 at every epoch.  "
                         "Intended for post-run bias-signature "
                         "analysis: trend, periodicity, per-axis "
                         "breakdown.  Not enabled by default; the "
                         "CSV is ~30 KB per 1000 epochs.")
    ap.add_argument("--wl-only", action="store_true",
                    help="WL-only AR mode: enable Melbourne-Wubbena slip "
                         "detection (resets ambiguity in the float filter "
                         "on a detected jump) but apply no NL fixing.  "
                         "Mirrors the engine's --wl-only contract: MW "
                         "tracks per-SV WL fix status for diagnostic "
                         "reporting (high-water mark + slip count) but "
                         "the float IF filter receives no pseudo-"
                         "measurement on its ambiguity state.  The win "
                         "vs pure float-PPP is purely from cycle-slip "
                         "detection preventing slips from poisoning the "
                         "float ambiguity for the rest of the run.  "
                         "Without this, undetected slips on a 24h run "
                         "leave the float ambiguity stuck at a wrong "
                         "value, biasing position by 10 cm-1 m for "
                         "the affected SV.  Target: ABMF 2020 DOY 001 "
                         "≤ 20 cm 3D vs ITRF14.")
    ap.add_argument("--ztd-tie", type=float, default=None, metavar="SIGMA",
                    help="If set, apply a per-epoch ZTD pseudo-"
                         "measurement (z=0 against the filter's "
                         "residual-ZTD state) with standard deviation "
                         "SIGMA metres.  Bounds the rank-deficient "
                         "(rx_clock, ZTD·m_wet, mean-ambiguity) null "
                         "mode that causes GPS+GAL SP3 runs at ABMF "
                         "to oscillate.  Empirically safe value for "
                         "tropical stations: 0.05 m (5 cm).  Larger "
                         "(0.10 m) is more conservative for "
                         "mid-latitude stations where real ZTD can "
                         "swing 30-50 cm during frontal passages.")
    ap.add_argument("--mean-amb-tie", type=float, default=None,
                    metavar="SIGMA",
                    help="If set, apply a per-epoch mean-ambiguity "
                         "pseudo-measurement (z=0 against Σ N_i / "
                         "n_SVs) with standard deviation SIGMA.  "
                         "Constrains the common-mode ambiguity drift "
                         "component of the null mode.  Physical "
                         "justification is weaker than --ztd-tie "
                         "(real mean N drifts with SV rise/set), so "
                         "prefer validating with --ztd-tie alone "
                         "first; only enable this if --ztd-tie "
                         "doesn't reach the convergence target.  "
                         "Recommended SIGMA: 0.10 m if enabled.")
    ap.add_argument("--q-pos-converged", type=float, default=None,
                    metavar="VAR",
                    help="Override the converged-regime position "
                         "process-noise variance (m² per epoch) in "
                         "PPPFilter.predict.  Default in production "
                         "filter is 1e-4 (σ_step ≈ 55 mm per 30 s "
                         "→ unbounded position random walk over 24 h).  "
                         "For static receivers, tighter values (1e-8 "
                         "to 1e-10) are more defensible and test "
                         "whether the Q2 overconfidence is driven by "
                         "position-state wander vs other mechanisms.")
    ap.add_argument("--ztd-ou-tau", type=float, default=None, metavar="SECONDS",
                    help="Mean-reversion time constant τ (seconds) for "
                         "the ZTD Ornstein-Uhlenbeck process model.  "
                         "Both --ztd-ou-tau and --ztd-ou-sigma must be "
                         "set to enable OU; otherwise the filter uses "
                         "the default random-walk ZTD model.  τ should "
                         "be hours-scale (3600–43200) to match real "
                         "tropospheric coherence time.  Smaller τ "
                         "fights real weather; larger τ degenerates "
                         "toward random walk.")
    ap.add_argument("--ztd-ou-sigma", type=float, default=None, metavar="SIGMA_M",
                    help="Steady-state standard deviation of the ZTD "
                         "OU process (metres).  Sets the amplitude of "
                         "allowed ZTD variation around the Saastamoinen "
                         "a priori.  ABMF's measured 24 h variation is "
                         "~18 cm so σ_steady=0.05–0.10 m is a starting "
                         "range.")
    ap.add_argument("--sigma-pr", type=float, default=None, metavar="SIGMA_M",
                    help="Override SIGMA_P_IF (IF pseudorange measurement "
                         "noise, default 3.0 m).  Used to test whether "
                         "R-inflation honestly reports σ when obs-model "
                         "gaps produce non-measurement-noise residuals.")
    ap.add_argument("--sigma-phi", type=float, default=None, metavar="SIGMA_M",
                    help="Override SIGMA_PHI_IF (IF carrier-phase "
                         "measurement noise, default 0.03 m).")
    ap.add_argument("--antex", default=None, metavar="PATH",
                    help="Optional IGS ANTEX 1.4 file (e.g. IGS14.atx) "
                         "for satellite + receiver phase-center "
                         "corrections.  When set, applies PCV/PCO "
                         "correction to pr_if and phi_if at ingest.  "
                         "Receiver antenna type is read from the RINEX "
                         "OBS header's ANT # / TYPE record.  Expected "
                         "improvement per PRIDE ablation: ~10-25 mm on "
                         "clean geometries.")
    ap.add_argument("--solid-tide", action="store_true",
                    help="Apply IERS 2010 Step-1 solid Earth tide "
                         "displacement to the station position when "
                         "computing geometric range.  ~42 mm single-"
                         "largest missing-model correction per PRIDE "
                         "ablation; see "
                         "project_to_main_pride_ablation_20260423.  "
                         "Harness-side only; engine impact requires "
                         "porting the solid_tide module there too.")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    return run(args)


if __name__ == "__main__":
    sys.exit(main())

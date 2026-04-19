#!/usr/bin/env python3
"""
solve_ppp.py — Multi-GNSS static-position PPP using an Extended Kalman Filter
with ionosphere-free (IF) pseudorange + carrier phase measurements.

State vector:
    x = [X, Y, Z, c*dt_gps, ISB_gal, ISB_bds, N_IF_1, ..., N_IF_n]
    - Position (3): static, tiny process noise
    - GPS receiver clock (1): random walk
    - Inter-system biases (2): GAL-GPS, BDS-GPS (slowly varying)
    - Float ambiguities (n): one per tracked dual-freq satellite

Uses GPS L1+L5, Galileo E1+E5a, and BDS B1I+B2a IF combinations.
BDS-2 GEO/IGSO satellites (PRN < 19) are excluded due to poor SP3 orbits.

Usage:
    python solve_ppp.py data/rawx_1h_top_20260303.csv data/gfz_mgx_062.sp3 \\
        --known-pos "LAT,LON,ALT" --out data/pos_ppp_mgx.csv
"""

import argparse
import csv
import logging
import math
import sys
from collections import defaultdict
from datetime import timedelta

import numpy as np

log = logging.getLogger(__name__)

from solve_pseudorange import (
    SP3, C, OMEGA_E, ecef_to_lla, ecef_to_enu, lla_to_ecef,
    timestamp_to_gpstime,
)
from solve_dualfreq import (
    F_L1, F_L2, F_E5B, F_B1I, F_B2I, IF_PAIRS,
    ALPHA_L1_L2, ALPHA_L2, ALPHA_E1, ALPHA_E5B, ALPHA_B1I_B2I, ALPHA_B2I,
)
from ppp_corrections import OSBParser, CLKFile

# IF wavelengths for carrier phase
WL_L1 = C / F_L1
WL_L2 = C / F_L2
WL_E5B = C / F_E5B
WL_B1I = C / F_B1I
WL_B2I = C / F_B2I

# Per-system IF carrier phase wavelength pairs
# GPS: L1 + L2C
# GAL: E1 + E5b
# BDS: B1I + B2I
IF_WL = {
    'G': (WL_L1, WL_L2, ALPHA_L1_L2, ALPHA_L2),
    'E': (WL_L1, WL_E5B, ALPHA_E1, ALPHA_E5B),
    'C': (WL_B1I, WL_B2I, ALPHA_B1I_B2I, ALPHA_B2I),
}

# Measurement noise (meters)
SIGMA_P_IF = 3.0
SIGMA_PHI_IF = 0.03

ELEV_MASK = 10.0  # degrees.  Tried 15° on 2026-04-17: with GAL-only the SV count dropped to 6/epoch (from ~8–10 at 10°) and LAMBDA couldn't fix NL at all — all three hosts went into persistent NAV2-reset cycling.  Low-elevation SVs do cause wrong-integer poisoning, but the remedy can't be a harder hard cut on SV-limited runs.  Next try: elevation-dependent measurement weighting (already partially present via `cno_factor * elev_factor` in SIGMA_P_IF weighting) or per-SV exclusion in the NL resolver rather than the observation stream.
BDS_MIN_PRN = 19  # Exclude BDS-2 GEO/IGSO

# F9T signal name → RINEX observation code mapping
SIG_TO_RINEX = {
    'GPS-L1CA': ('C1C', 'L1C'),   # Code, Phase
    'GPS-L2CL': ('C2L', 'L2L'),
    'GPS-L2CM': ('C2S', 'L2S'),
    'GPS-L5Q':  ('C5Q', 'L5Q'),
    'GAL-E1C':  ('C1C', 'L1C'),
    'GAL-E5bQ': ('C7Q', 'L7Q'),
    'GAL-E5aQ': ('C5Q', 'L5Q'),
    'BDS-B1I':  ('C2I', 'L2I'),
    'BDS-B2I':  ('C7I', 'L7I'),
    'BDS-B2aI': ('C5I', 'L5I'),   # u-blox B2aI → RINEX C5I
}

# EKF state layout
IDX_X, IDX_Y, IDX_Z = 0, 1, 2
IDX_CLK = 3
IDX_ISB_GAL = 4
IDX_ISB_BDS = 5
IDX_ZTD = 6
N_BASE = 7


# ── Determine system from SV prefix ──────────────────────────────────────── #
def sv_sys(sv):
    if sv[0] == 'G': return 'gps'
    if sv[0] == 'E': return 'gal'
    if sv[0] == 'C': return 'bds'
    return 'gps'


# ── Load multi-GNSS PPP epochs ────────────────────────────────────────────── #
def load_ppp_epochs(csv_path, systems=None, osb=None):
    """Load RAWX CSV and form multi-GNSS IF pseudorange + carrier phase.

    If osb is provided (OSBParser), apply observable-specific signal bias
    corrections to both code and phase before forming IF combination.

    Returns list of (timestamp_str, [{sv, pr_if, phi_if_m, cno, lock_duration_ms,
                                       half_cyc_ok, sys}, ...])
    """
    if systems is None:
        systems = {'gps', 'gal', 'bds'}

    sys_map = {'G': 'gps', 'E': 'gal', 'C': 'bds'}

    sig_lookup = {}
    for gnss_id, sig_f1, sig_f2, prefix, a1, a2 in IF_PAIRS:
        sig_lookup[sig_f1] = (gnss_id, prefix, 'f1', a1, a2, sig_f1)
        sig_lookup[sig_f2] = (gnss_id, prefix, 'f2', a1, a2, sig_f2)

    raw = defaultdict(lambda: defaultdict(dict))
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['pr_valid'] != '1':
                continue
            sig = row['signal_id']
            if sig not in sig_lookup:
                continue
            gnss_id, prefix, role, a1, a2, sig_name = sig_lookup[sig]
            if sys_map.get(prefix) not in systems:
                continue
            ts = row['timestamp']
            sv_num = int(row['sv_id'])
            if prefix == 'C' and sv_num < BDS_MIN_PRN:
                continue
            sv = f"{prefix}{sv_num:02d}"
            try:
                pr = float(row['pseudorange_m'])
                cno = float(row['cno_dBHz'])
                cp = float(row['carrier_phase_cy']) if row.get('cp_valid') == '1' else None
                lock_ms = float(row['lock_duration_ms']) if row.get('lock_duration_ms') else 0.0
                half_cyc = row.get('half_cyc', '0')
            except (ValueError, KeyError):
                continue
            if pr < 1e6 or pr > 4e7:
                continue
            raw[ts][sv][role] = {
                'pr': pr, 'cno': cno, 'cp': cp,
                'lock_ms': lock_ms, 'half_cyc': half_cyc,
                'alpha_f1': a1, 'alpha_f2': a2,
                'sig_name': sig_name,
            }

    result = []
    n_osb_applied = 0
    for ts in sorted(raw.keys()):
        obs = []
        for sv, roles in raw[ts].items():
            if 'f1' not in roles or 'f2' not in roles:
                continue
            f1 = roles['f1']
            f2 = roles['f2']
            if f1['cp'] is None or f2['cp'] is None:
                continue
            if f1['half_cyc'] != '1' or f2['half_cyc'] != '1':
                continue

            a1 = f1['alpha_f1']
            a2 = f1['alpha_f2']
            prefix = sv[0]

            # Apply OSB corrections (subtract bias from observations)
            pr_f1 = f1['pr']
            pr_f2 = f2['pr']
            cp_f1 = f1['cp']
            cp_f2 = f2['cp']

            if osb is not None:
                rinex_f1 = SIG_TO_RINEX.get(f1['sig_name'])
                rinex_f2 = SIG_TO_RINEX.get(f2['sig_name'])
                if rinex_f1 and rinex_f2:
                    code_osb_f1 = osb.get_osb(sv, rinex_f1[0])
                    code_osb_f2 = osb.get_osb(sv, rinex_f2[0])
                    phase_osb_f1 = osb.get_osb(sv, rinex_f1[1])
                    phase_osb_f2 = osb.get_osb(sv, rinex_f2[1])
                    if code_osb_f1 is not None and code_osb_f2 is not None:
                        pr_f1 -= code_osb_f1
                        pr_f2 -= code_osb_f2
                        n_osb_applied += 1
                    wl_f1, wl_f2, _, _ = IF_WL[prefix]
                    if phase_osb_f1 is not None and phase_osb_f2 is not None:
                        cp_f1 -= phase_osb_f1 / wl_f1
                        cp_f2 -= phase_osb_f2 / wl_f2

            # IF pseudorange
            pr_if = a1 * pr_f1 - a2 * pr_f2

            # IF carrier phase in meters
            wl_f1, wl_f2, _, _ = IF_WL[prefix]
            phi_if_m = a1 * wl_f1 * cp_f1 - a2 * wl_f2 * cp_f2

            obs.append({
                'sv': sv,
                'sys': sv_sys(sv),
                'pr_if': pr_if,
                'phi_if_m': phi_if_m,
                'cno': min(f1['cno'], f2['cno']),
                'lock_duration_ms': min(f1['lock_ms'], f2['lock_ms']),
                'half_cyc_ok': True,
            })
        if len(obs) >= 4:
            result.append((ts, obs))

    if osb is not None and n_osb_applied > 0:
        n_unique = len(set(sv for _, obs_list in result for o in obs_list for sv in [o['sv']]))
        print(f"  OSB corrections applied to {n_unique} satellites", file=sys.stderr)

    return result


# ── Multi-GNSS PPP EKF ────────────────────────────────────────────────────── #
class PPPFilter:
    """Multi-GNSS static-position PPP EKF with ISBs and float ambiguities."""

    def __init__(self):
        self.x = None
        self.P = None
        self.sv_to_idx = {}
        self.prev_obs = {}
        self.initialized = False

    def initialize(self, pos_ecef, clock_m, isb_gal=0.0, isb_bds=0.0,
                   systems=None):
        """Initialize filter state.

        systems: optional iterable of constellations that will feed
        observations ('gps', 'gal', 'bds').  Used to pin inter-system
        biases that would otherwise be rank-1 degenerate with IDX_CLK.
        If GPS is absent, GAL ISB is pinned to zero (IDX_CLK absorbs the
        GAL clock).  If both GPS and GAL are absent (BDS-only), BDS ISB
        is also pinned.  Without this, GAL-only or BDS-only runs can
        settle anywhere along the rank-1 ridge (filter reports tight σ
        on positions tens to hundreds of metres off the truth — see
        2026-04-17 overnight investigation).
        """
        self.x = np.zeros(N_BASE)
        self.x[:3] = pos_ecef
        self.x[IDX_CLK] = clock_m
        self.x[IDX_ISB_GAL] = isb_gal
        self.x[IDX_ISB_BDS] = isb_bds
        self.x[IDX_ZTD] = 0.0  # residual ZTD (a priori model handles bulk)
        self.P = np.diag([
            100.0**2, 100.0**2, 100.0**2,
            1e8,
            1e6,
            1e6,
            0.5**2,  # ZTD residual: 0.5m initial sigma
        ])
        # Pin ISBs whose reference system isn't present.  Priority order:
        # GPS > GAL > BDS — the highest-priority present system is the
        # reference and its x[IDX_CLK] absorbs the clock.
        self._pinned_isbs = set()
        if systems is not None:
            sys_set = set(systems)
            has_gps = 'gps' in sys_set
            if not has_gps and 'gal' in sys_set:
                # GAL is the reference — pin its ISB to 0
                self._pinned_isbs.add(IDX_ISB_GAL)
                self.x[IDX_ISB_GAL] = 0.0
                self.P[IDX_ISB_GAL, IDX_ISB_GAL] = 1e-6
            if not has_gps and 'gal' not in sys_set and 'bds' in sys_set:
                # BDS is the reference — pin its ISB to 0
                self._pinned_isbs.add(IDX_ISB_BDS)
                self.x[IDX_ISB_BDS] = 0.0
                self.P[IDX_ISB_BDS, IDX_ISB_BDS] = 1e-6
        self.sv_to_idx = {}
        self.prev_obs = {}
        self.initialized = True

    def predict(self, dt):
        if dt <= 0:
            dt = 1.0
        n = len(self.x)
        Q = np.zeros((n, n))
        # Adaptive position process noise: large during convergence,
        # small once converged. Prevents filter from freezing position
        # before carrier phase has corrected the LS init error.
        pos_var = max(self.P[0, 0], self.P[1, 1], self.P[2, 2])
        pos_sigma = math.sqrt(pos_var)
        if pos_sigma > 10.0:
            q_pos = 1.0          # Early: allow large corrections
        elif pos_sigma > 1.0:
            q_pos = 0.01         # Converging: moderate
        else:
            q_pos = 1e-4         # Converged: static with breathing room
        for i in range(3):
            Q[i, i] = q_pos * dt
        Q[IDX_CLK, IDX_CLK] = 1e6 * dt
        # Pinned ISBs (single-constellation runs) get no process noise so
        # they stay fixed at zero; IDX_CLK absorbs the reference system clock.
        pinned = getattr(self, '_pinned_isbs', set())
        if IDX_ISB_GAL not in pinned:
            Q[IDX_ISB_GAL, IDX_ISB_GAL] = 1.0 * dt
        if IDX_ISB_BDS not in pinned:
            Q[IDX_ISB_BDS, IDX_ISB_BDS] = 1.0 * dt
        Q[IDX_ZTD, IDX_ZTD] = (5e-5)**2 * dt  # ~5 cm/hour RMS (IGS standard)
        self.P = self.P + Q

    def add_ambiguity(self, sv, N_init_m):
        idx = len(self.x) - N_BASE
        self.sv_to_idx[sv] = idx
        self.x = np.append(self.x, N_init_m)
        n = len(self.x)
        P_new = np.zeros((n, n))
        P_new[:n-1, :n-1] = self.P
        P_new[n-1, n-1] = 100.0**2
        self.P = P_new

    def remove_ambiguity(self, sv):
        if sv not in self.sv_to_idx:
            return
        idx = N_BASE + self.sv_to_idx[sv]
        self.x = np.delete(self.x, idx)
        self.P = np.delete(np.delete(self.P, idx, axis=0), idx, axis=1)
        removed_idx = self.sv_to_idx[sv]
        del self.sv_to_idx[sv]
        for s in self.sv_to_idx:
            if self.sv_to_idx[s] > removed_idx:
                self.sv_to_idx[s] -= 1

    def inflate_ambiguity(self, sv, sigma_m=100.0):
        """Inflate an ambiguity's covariance without removing the state.

        Used by the post-fix residual monitor to "soft-unfix" — undoes
        the tight constraint that NL fix applied, so the filter can
        re-converge to the correct integer, while preserving sv_to_idx
        order and the rest of the state.
        """
        if sv not in self.sv_to_idx:
            return
        si = N_BASE + self.sv_to_idx[sv]
        if si >= self.P.shape[0]:
            return
        self.P[si, :] = 0.0
        self.P[:, si] = 0.0
        self.P[si, si] = sigma_m ** 2

    def detect_cycle_slips(self, current_obs, prev_obs):
        slipped = set()
        for o in current_obs:
            sv = o['sv']
            if sv in prev_obs:
                if o['lock_duration_ms'] < prev_obs[sv]['lock_duration_ms']:
                    slipped.add(sv)
        return slipped

    def compute_elevation(self, receiver_pos, sat_pos):
        dx = sat_pos - receiver_pos
        r = np.linalg.norm(dx)
        if r < 1.0:
            return 90.0
        e = dx / r
        up = receiver_pos / np.linalg.norm(receiver_pos)
        sin_elev = np.dot(e, up)
        return math.degrees(math.asin(max(-1.0, min(1.0, sin_elev))))

    def compute_azimuth(self, receiver_pos, sat_pos):
        """Azimuth of the satellite from the receiver, degrees clockwise
        from geodetic north.  0 = north, 90 = east, 180 = south.

        Used by the Bead 4 validation promoter to measure how much
        satellite geometry has changed since the NL fix — a wrong
        integer that passes LAMBDA at one az can reveal itself once the
        SV has moved ~15°.
        """
        dx = sat_pos - receiver_pos
        r = np.linalg.norm(dx)
        if r < 1.0:
            return 0.0
        e = dx / r
        x, y, z = receiver_pos
        rho = math.sqrt(x * x + y * y)
        if rho < 1.0:
            # Over a pole; any azimuth convention collapses.  Return 0.
            return 0.0
        # ENU basis at receiver (geocentric approximation — fine for
        # Δaz applications at the ~degree scale we care about here).
        east = np.array([-y / rho, x / rho, 0.0])
        lat = math.atan2(z, rho)
        sin_lat = math.sin(lat)
        cos_lat = math.cos(lat)
        north = np.array([-sin_lat * x / rho,
                          -sin_lat * y / rho,
                          cos_lat])
        az = math.degrees(math.atan2(np.dot(e, east), np.dot(e, north)))
        if az < 0:
            az += 360.0
        return az

    def tropo_delay(self, elevation_deg):
        if elevation_deg < 5.0:
            elevation_deg = 5.0
        return 2.3 / math.sin(math.radians(elevation_deg))

    @staticmethod
    def wet_mapping(elevation_deg):
        """Wet tropospheric mapping function: 1/sin(e)."""
        e = max(elevation_deg, 5.0)
        return 1.0 / math.sin(math.radians(e))

    def isb_index(self, sys_name):
        if sys_name == 'gal': return IDX_ISB_GAL
        if sys_name == 'bds': return IDX_ISB_BDS
        return None

    def update(self, observations, sp3, t, clk_file=None):
        H_rows = []
        z_rows = []
        R_diag = []
        labels = []  # (sv, 'pr'|'phi', elev_deg) aligned with rows — elev for PFR diagnostics
        n_used = 0
        sys_counts = defaultdict(int)
        receiver_pos = self.x[:3]

        for obs in observations:
            sv = obs['sv']
            # Evaluate satellite position at TRANSMISSION time, not reception
            # time.  The satellite moves ~300m during the ~77ms signal travel
            # time; computing at t_rx creates a per-satellite range error
            # proportional to radial velocity (±60m), which was the dominant
            # source of the systematic 50m PPP position bias.
            tau_approx = obs['pr_if'] / C if 'pr_if' in obs else 0.075
            t_tx = t - timedelta(seconds=tau_approx)
            sat_pos, sat_clk_sp3 = sp3.sat_position(sv, t_tx)
            if sat_pos is None:
                continue
            # Use CLK file for satellite clock if available, else SP3
            if clk_file is not None:
                sat_clk = clk_file.sat_clock(sv, t_tx)
                if sat_clk is None:
                    sat_clk = sat_clk_sp3
            else:
                sat_clk = sat_clk_sp3
            if sat_clk is None:
                continue
            if abs(sat_clk) > 0.002:  # 2ms; normal GNSS clocks are < 1ms
                continue

            dx = sat_pos - receiver_pos
            rho = np.linalg.norm(dx)
            tau = rho / C
            rot = OMEGA_E * tau
            sat_rot = np.array([
                sat_pos[0] * math.cos(rot) + sat_pos[1] * math.sin(rot),
                -sat_pos[0] * math.sin(rot) + sat_pos[1] * math.cos(rot),
                sat_pos[2]
            ])
            dx = sat_rot - receiver_pos
            rho = np.linalg.norm(dx)

            elev = self.compute_elevation(receiver_pos, sat_rot)
            if elev < ELEV_MASK:
                continue

            tropo = self.tropo_delay(elev)
            m_wet = self.wet_mapping(elev)
            e_los = dx / rho

            # Clock + ISB
            isb_idx = self.isb_index(obs['sys'])
            clk_val = self.x[IDX_CLK]
            if isb_idx is not None:
                clk_val += self.x[isb_idx]

            rho_pred = rho + clk_val - sat_clk * C + tropo + self.x[IDX_ZTD] * m_wet

            cno_factor = 10 ** ((obs['cno'] - 35) / 20)
            elev_factor = math.sin(math.radians(elev))
            w = max(0.01, cno_factor * elev_factor)

            # --- IF Pseudorange ---
            dz_pr = obs['pr_if'] - rho_pred
            # One-time diagnostic: log per-satellite residuals at first epoch
            if not hasattr(self, '_diag_logged'):
                log.info("  DIAG %s: elev=%.0f° rho=%.0f sat_clk=%.6fs "
                         "tropo=%.1f pr_if=%.1f rho_pred=%.1f "
                         "dz_pr=%.1f m",
                         sv, elev, rho, sat_clk, tropo,
                         obs['pr_if'], rho_pred, dz_pr)
            h_pr = np.zeros(len(self.x))
            h_pr[0] = -e_los[0]
            h_pr[1] = -e_los[1]
            h_pr[2] = -e_los[2]
            h_pr[IDX_CLK] = 1.0
            if isb_idx is not None:
                h_pr[isb_idx] = 1.0
            h_pr[IDX_ZTD] = m_wet
            H_rows.append(h_pr)
            z_rows.append(dz_pr)
            R_diag.append((SIGMA_P_IF / w) ** 2)
            labels.append((sv, 'pr', elev))

            # --- IF Carrier phase ---
            if sv in self.sv_to_idx:
                amb_idx = N_BASE + self.sv_to_idx[sv]
                dz_phi = obs['phi_if_m'] - rho_pred - self.x[amb_idx]
                h_phi = np.zeros(len(self.x))
                h_phi[0] = -e_los[0]
                h_phi[1] = -e_los[1]
                h_phi[2] = -e_los[2]
                h_phi[IDX_CLK] = 1.0
                if isb_idx is not None:
                    h_phi[isb_idx] = 1.0
                h_phi[IDX_ZTD] = m_wet
                h_phi[amb_idx] = 1.0
                H_rows.append(h_phi)
                z_rows.append(dz_phi)
                R_diag.append((SIGMA_PHI_IF / w) ** 2)
                labels.append((sv, 'phi', elev))

            n_used += 1
            sys_counts[obs['sys']] += 1

        if not hasattr(self, '_diag_logged'):
            self._diag_logged = True

        if n_used < 4:
            return n_used, np.array([]), {}

        H = np.array(H_rows)
        z = np.array(z_rows)
        R = np.diag(R_diag)

        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return n_used, z, dict(sys_counts)

        self.x = self.x + K @ z
        I_KH = np.eye(len(self.x)) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        post_resid = z - H @ (K @ z)
        # Store per-measurement (sv, type) labels aligned with post_resid so
        # callers can map residuals back to their satellites — used by the
        # post-fix residual monitor to detect wrong integer fixes.
        self.last_residual_labels = labels
        return n_used, post_resid, dict(sys_counts)


# ── Fixed-position clock estimator (time-differenced carrier phase) ────── #
class FixedPosFilter:
    """Fixed-position PPP filter using time-differenced carrier phase.

    State: [c*dt_rx, c*dt_dot]  (just 2 states — no ambiguities!)

    Uses pseudorange for absolute clock level, and time-differenced
    carrier phase (Δφ = φ(t) - φ(t-1)) for precise clock change.
    Time differencing cancels ambiguities and most multipath (which
    barely changes over 1-second intervals).

    Δφ_IF(t) - Δρ(t) - Δ(sat_clk(t)) ≈ Δ(rx_clk(t)) + noise
    """

    IDX_CLK = 0
    IDX_CLK_RATE = 1
    IDX_ZTD = 2
    IDX_ISB_GAL = 3
    IDX_ISB_BDS = 4
    N_STATES = 5

    def __init__(self, pos_ecef):
        self.pos = np.array(pos_ecef)
        self.x = np.zeros(self.N_STATES)     # [clock, clock_rate, dZTD, isb_gal, isb_bds] in meters
        self.P = np.diag([1e18, 1e6, 0.5**2, 1e8, 1e8])  # dZTD: 0.5m initial sigma
        self.prev_geo = {}  # sv → {rho_corr, sat_clk_m, phi_if_m, tropo}
        self.initialized = False  # Will seed clock from first epoch

    def predict(self, dt):
        if dt <= 0:
            dt = 1.0
        # Clock propagation (ISB is constant — no prediction needed)
        self.x[self.IDX_CLK] += self.x[self.IDX_CLK_RATE] * dt
        F = np.eye(self.N_STATES)
        F[0, 1] = dt
        self.P = F @ self.P @ F.T
        # Process noise: TCXO model + ZTD drift + small ISB walk
        Q = np.zeros((self.N_STATES, self.N_STATES))
        Q[0, 0] = 0.01 * dt      # phase noise (m²/s)
        Q[1, 1] = 0.01 * dt      # frequency noise (m²/s³)
        Q[self.IDX_ZTD, self.IDX_ZTD] = (5e-5)**2 * dt  # ~5 cm/hour RMS (IGS standard)
        Q[self.IDX_ISB_GAL, self.IDX_ISB_GAL] = 1e-6 * dt  # GAL ISB random walk
        Q[self.IDX_ISB_BDS, self.IDX_ISB_BDS] = 1e-6 * dt  # BDS ISB random walk
        self.P += Q

    def compute_geometry(self, sv, sp3, t, clk_file, pr_m=None):
        """Compute corrected range and satellite clock for one SV."""
        # Transmission time: approximate with pseudorange or default 77ms
        tau_approx = pr_m / C if pr_m else 0.075
        t_tx = t - timedelta(seconds=tau_approx)
        sat_pos, sat_clk_sp3 = sp3.sat_position(sv, t_tx)
        if sat_pos is None:
            return None
        if clk_file is not None:
            sat_clk = clk_file.sat_clock(sv, t_tx)
            if sat_clk is None:
                sat_clk = sat_clk_sp3
        else:
            sat_clk = sat_clk_sp3
        if sat_clk is None or abs(sat_clk) > 0.002:
            return None

        dx = sat_pos - self.pos
        rho = np.linalg.norm(dx)
        tau = rho / C
        rot = OMEGA_E * tau
        sat_rot = np.array([
            sat_pos[0]*math.cos(rot) + sat_pos[1]*math.sin(rot),
            -sat_pos[0]*math.sin(rot) + sat_pos[1]*math.cos(rot),
            sat_pos[2]])
        dx = sat_rot - self.pos
        rho = np.linalg.norm(dx)

        # Elevation
        r_dx = np.linalg.norm(dx)
        up = self.pos / np.linalg.norm(self.pos)
        sin_elev = np.dot(dx / r_dx, up) if r_dx > 1 else 1.0
        elev = math.degrees(math.asin(max(-1, min(1, sin_elev))))
        if elev < ELEV_MASK:
            return None

        sin_elev = math.sin(math.radians(max(5, elev)))
        tropo = 2.3 / sin_elev
        m_wet = 1.0 / sin_elev  # wet mapping function
        return {
            'rho': rho,
            'sat_clk_m': sat_clk * C,
            'tropo': tropo,
            'm_wet': m_wet,
            'elev': elev,
        }

    def update(self, observations, sp3, t, clk_file=None):
        H_rows, z_rows, R_diag = [], [], []
        n_pr = 0
        n_td = 0
        current_geo = {}

        # Seed clock from first epoch's pseudorange residuals.
        # Prefer GPS (no ISB), but fall back to Galileo if GPS unavailable.
        # When seeding from GPS, also seed ISBs from other constellations.
        if not self.initialized:
            sys_resid = {}  # sys_name → list of residuals
            for obs in observations:
                sv = obs['sv']
                geo = self.compute_geometry(sv, sp3, t, clk_file,
                                           pr_m=obs.get('pr_if'))
                if geo is None:
                    continue
                rho_corr = geo['rho'] - geo['sat_clk_m'] + geo['tropo']
                r = obs['pr_if'] - rho_corr
                sys_name = obs.get('sys', 'gps')
                sys_resid.setdefault(sys_name, []).append(r)

            gps_resid = sys_resid.get('gps', [])
            gal_resid = sys_resid.get('gal', [])
            bds_resid = sys_resid.get('bds', [])

            if len(gps_resid) >= 3:
                # Seed from GPS (reference constellation, no ISB)
                self.x[0] = float(np.median(gps_resid))
                spread = np.std(gps_resid) if len(gps_resid) > 1 else 100.0
                self.P[0, 0] = max(spread, 50.0) ** 2
                log.info(f"Clock seeded from {len(gps_resid)} GPS PRs: "
                         f"{self.x[0]/C*1e6:.1f} µs "
                         f"(P[0,0] reset to {self.P[0,0]:.0f} m²)")
                # Seed ISBs from other constellations: ISB = median(sys) - clock
                if len(gal_resid) >= 2:
                    self.x[self.IDX_ISB_GAL] = float(np.median(gal_resid)) - self.x[0]
                    log.info(f"  ISB GAL seeded: {self.x[self.IDX_ISB_GAL]/C*1e9:+.1f} ns")
                if len(bds_resid) >= 2:
                    self.x[self.IDX_ISB_BDS] = float(np.median(bds_resid)) - self.x[0]
                    log.info(f"  ISB BDS seeded: {self.x[self.IDX_ISB_BDS]/C*1e9:+.1f} ns")
                self.initialized = True
            elif len(gal_resid) >= 3:
                # Seed from Galileo — can't separate clock from ISB_GAL.
                # Set clock = median(GAL), ISB_GAL = 0, resolve in filter.
                self.x[0] = float(np.median(gal_resid))
                self.x[self.IDX_ISB_GAL] = 0.0
                spread = np.std(gal_resid) if len(gal_resid) > 1 else 100.0
                self.P[0, 0] = max(spread, 50.0) ** 2
                self.P[self.IDX_ISB_GAL, self.IDX_ISB_GAL] = 1e8
                if len(bds_resid) >= 2:
                    self.x[self.IDX_ISB_BDS] = float(np.median(bds_resid)) - self.x[0]
                    log.info(f"  ISB BDS seeded: {self.x[self.IDX_ISB_BDS]/C*1e9:+.1f} ns")
                log.info(f"Clock seeded from {len(gal_resid)} GAL PRs: "
                         f"{self.x[0]/C*1e6:.1f} µs "
                         f"(P[0,0] reset to {self.P[0,0]:.0f} m², "
                         f"ISB GAL unresolved)")
                self.initialized = True

        for obs in observations:
            sv = obs['sv']
            geo = self.compute_geometry(sv, sp3, t, clk_file,
                                       pr_m=obs.get('pr_if'))
            if geo is None:
                continue

            current_geo[sv] = {
                'rho': geo['rho'],
                'sat_clk_m': geo['sat_clk_m'],
                'tropo': geo['tropo'],
                'm_wet': geo['m_wet'],
                'ztd_corr': self.x[self.IDX_ZTD] * geo['m_wet'],
                'phi_if_m': obs.get('phi_if_m'),
            }

            elev = geo['elev']
            cno_factor = 10 ** ((obs['cno'] - 35) / 20)
            elev_factor = math.sin(math.radians(elev))
            w = max(0.01, cno_factor * elev_factor)

            # Predicted range (without receiver clock), including ZTD residual
            m_wet = geo['m_wet']
            rho_corr = geo['rho'] - geo['sat_clk_m'] + geo['tropo'] + self.x[self.IDX_ZTD] * m_wet

            # ISB: select the appropriate inter-system bias for this constellation
            sys_name = obs.get('sys', 'gps')
            isb_val = 0.0
            isb_idx = None
            if sys_name == 'gal':
                isb_idx = self.IDX_ISB_GAL
                isb_val = self.x[self.IDX_ISB_GAL]
            elif sys_name == 'bds':
                isb_idx = self.IDX_ISB_BDS
                isb_val = self.x[self.IDX_ISB_BDS]

            # --- Pseudorange: absolute clock level ---
            dz_pr = obs['pr_if'] - rho_corr - self.x[0] - isb_val
            h_pr = np.zeros(self.N_STATES)
            h_pr[0] = 1.0
            h_pr[self.IDX_ZTD] = m_wet
            if isb_idx is not None:
                h_pr[isb_idx] = 1.0
            H_rows.append(h_pr)
            z_rows.append(dz_pr)
            R_diag.append((SIGMA_P_IF / w) ** 2)
            n_pr += 1

            # --- Time-differenced carrier phase ---
            # Only include TD observations after clock is seeded from PRs.
            # Before seeding, prev_clock is 0 (meaningless), causing huge
            # residuals that blow up the covariance matrix.
            if (self.initialized and
                    obs.get('phi_if_m') is not None and
                    sv in self.prev_geo and
                    self.prev_geo[sv]['phi_if_m'] is not None):
                prev = self.prev_geo[sv]
                delta_phi = obs['phi_if_m'] - prev['phi_if_m']
                # Use stored rho_corr (includes previous ZTD contribution)
                prev_rho_corr = prev['rho'] - prev['sat_clk_m'] + prev['tropo'] + prev.get('ztd_corr', 0.0)
                delta_rho_corr = rho_corr - prev_rho_corr
                dz_td = (delta_phi - delta_rho_corr + self.prev_clock) - self.x[0]
                h_td = np.zeros(self.N_STATES)
                h_td[0] = 1.0
                h_td[self.IDX_ZTD] = m_wet  # ZTD change maps through wet MF
                # ISB cancels in time difference (constant bias)
                H_rows.append(h_td)
                z_rows.append(dz_td)
                sigma_td = 0.3 / max(0.2, elev_factor)
                R_diag.append((sigma_td / w) ** 2)
                n_td += 1

        if n_pr < 1:
            self.prev_geo = current_geo
            self.prev_clock = self.x[0]
            return 0, np.array([]), 0

        H = np.array(H_rows)
        z = np.array(z_rows)
        R = np.diag(R_diag)

        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.prev_geo = current_geo
            self.prev_clock = self.x[0]
            return n_pr, z, n_td

        self.x = self.x + K @ z
        I_KH = np.eye(self.N_STATES) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        post_resid = z - H @ (K @ z)

        # Store for next epoch's time differencing
        self.prev_geo = current_geo
        self.prev_clock = self.x[0]

        return n_pr + n_td, post_resid, n_td


# ── LS init (multi-GNSS, IF pseudorange) ─────────────────────────────────── #
def ls_init(observations, sp3, t, clk_file=None):
    """Weighted LS solve for initial position from IF pseudoranges.
    Adaptively estimates ISBs for present systems.
    Includes residual-based outlier rejection to handle satellites
    with inaccurate broadcast clocks (e.g. Galileo E11/E19)."""
    present = set(o['sys'] for o in observations)
    # x[3] is the reference-system clock.  Inter-system biases (ISBs)
    # are offsets from that reference.  When GPS is present it's the
    # reference; when GPS is absent the next-present system becomes
    # the reference (no ISB for it).  Otherwise the ISB column and the
    # clock column become degenerate and ls_init silently fails.
    n_params = 4
    gal_col = bds_col = None
    has_gps = 'gps' in present
    if 'gal' in present and has_gps:
        gal_col = n_params; n_params += 1
    if 'bds' in present and (has_gps or 'gal' in present):
        bds_col = n_params; n_params += 1

    x = np.zeros(n_params)
    x[2] = 6371000.0

    sat_positions = []
    for obs in observations:
        tau_approx = obs.get('pr_if', 23e6) / C
        t_tx = t - timedelta(seconds=tau_approx)
        sp, _ = sp3.sat_position(obs['sv'], t_tx)
        if sp is not None:
            sat_positions.append(sp)
    if len(sat_positions) >= 4:
        avg = np.mean(sat_positions, axis=0)
        r = np.linalg.norm(avg)
        if r > 0:
            x[:3] = avg / r * 6371000.0

    excluded_svs = set()

    for iteration in range(20):
        H = []
        dz = []
        W = []
        sv_list = []
        for obs in observations:
            if obs['sv'] in excluded_svs:
                continue
            tau_approx = obs.get('pr_if', 23e6) / C
            t_tx = t - timedelta(seconds=tau_approx)
            sat_pos, sat_clk_sp3 = sp3.sat_position(obs['sv'], t_tx)
            if sat_pos is None:
                continue
            if clk_file is not None:
                sat_clk = clk_file.sat_clock(obs['sv'], t_tx)
                if sat_clk is None:
                    sat_clk = sat_clk_sp3
            else:
                sat_clk = sat_clk_sp3
            if sat_clk is None:
                continue
            if abs(sat_clk) > 0.002:  # 2ms; normal GNSS clocks are < 1ms
                continue
            dx = sat_pos - x[:3]
            rho = np.linalg.norm(dx)
            if rho < 1e6:
                rho = 2e7
            tau = rho / C
            rot = OMEGA_E * tau
            sat_rot = np.array([
                sat_pos[0] * math.cos(rot) + sat_pos[1] * math.sin(rot),
                -sat_pos[0] * math.sin(rot) + sat_pos[1] * math.cos(rot),
                sat_pos[2]
            ])
            dx = sat_rot - x[:3]
            rho = np.linalg.norm(dx)
            e = -dx / rho

            clk = x[3]
            if obs['sys'] == 'gal' and gal_col is not None:
                clk += x[gal_col]
            elif obs['sys'] == 'bds' and bds_col is not None:
                clk += x[bds_col]

            pr_pred = rho + clk - sat_clk * C
            h = np.zeros(n_params)
            h[0] = e[0]; h[1] = e[1]; h[2] = e[2]; h[3] = 1.0
            if obs['sys'] == 'gal' and gal_col is not None:
                h[gal_col] = 1.0
            elif obs['sys'] == 'bds' and bds_col is not None:
                h[bds_col] = 1.0

            H.append(h)
            dz.append(obs['pr_if'] - pr_pred)
            W.append(10 ** ((obs['cno'] - 30) / 20))
            sv_list.append(obs['sv'])

        if len(H) < n_params:
            return np.zeros(6), False, len(H)

        H = np.array(H)
        dz_arr = np.array(dz)
        W_mat = np.diag(W)
        try:
            HTW = H.T @ W_mat
            dx = np.linalg.solve(HTW @ H, HTW @ dz_arr)
        except np.linalg.LinAlgError:
            result = np.zeros(6)
            result[:4] = x[:4]
            return result, False, len(H)

        x += dx

        # Outlier rejection after position has roughly converged (iteration >= 5)
        # Reject satellites with residuals > 50m (broadcast accuracy is ~2-5m)
        if iteration >= 5 and np.linalg.norm(dx[:3]) < 1000:
            post_resid = dz_arr - H @ dx
            for i, sv in enumerate(sv_list):
                if abs(post_resid[i]) > 50.0:
                    excluded_svs.add(sv)
                    log.debug(f"LS outlier: {sv} resid={post_resid[i]:.1f}m")

        if np.linalg.norm(dx[:3]) < 0.01:
            result = np.zeros(6)
            result[:4] = x[:4]
            if gal_col is not None: result[4] = x[gal_col]
            if bds_col is not None: result[5] = x[bds_col]
            n_used = len(H) - len(excluded_svs)
            return result, True, n_used

    result = np.zeros(6)
    result[:4] = x[:4]
    if gal_col is not None: result[4] = x[gal_col]
    if bds_col is not None: result[5] = x[bds_col]
    return result, True, len(H)


# ── Main ──────────────────────────────────────────────────────────────────── #
def main():
    ap = argparse.ArgumentParser(
        description="Multi-GNSS PPP EKF with IF pseudorange + carrier phase")
    ap.add_argument("rawx", help="RAWX CSV file (testAnt format)")
    ap.add_argument("sp3", help="SP3 precise orbit file (multi-GNSS)")
    ap.add_argument("--out", default=None)
    ap.add_argument("--skip", type=int, default=0)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--known-pos", default=None,
                    help="Known position as lat,lon,alt")
    ap.add_argument("--leap", type=int, default=18)
    ap.add_argument("--systems", default="gps,gal,bds",
                    help="Comma-separated systems (gps,gal,bds)")
    ap.add_argument("--no-phase", action='store_true',
                    help="Pseudorange-only (no carrier phase)")
    ap.add_argument("--osb", default=None,
                    help="SINEX BIAS file for OSB corrections")
    ap.add_argument("--clk", default=None,
                    help="RINEX CLK file for high-rate satellite clocks")
    ap.add_argument("--phase-delay", type=int, default=0,
                    help="Delay carrier phase by N epochs after init")
    ap.add_argument("--fix-pos", action='store_true',
                    help="Fixed-position mode: use known-pos, estimate clock only")
    args = ap.parse_args()

    enabled = set(args.systems.split(','))

    print(f"Loading SP3: {args.sp3}", file=sys.stderr)
    sp3 = SP3(args.sp3)
    n_g = sum(1 for s in sp3.positions if s.startswith('G'))
    n_e = sum(1 for s in sp3.positions if s.startswith('E'))
    n_c = sum(1 for s in sp3.positions if s.startswith('C'))
    print(f"  {len(sp3.epochs)} epochs, {len(sp3.positions)} sats "
          f"(G{n_g} E{n_e} C{n_c})", file=sys.stderr)

    osb = None
    if args.osb:
        print(f"Loading OSB: {args.osb}", file=sys.stderr)
        osb = OSBParser(args.osb)
        print(f"  {len(osb.prns())} satellites with biases", file=sys.stderr)

    clk_file = None
    if args.clk:
        print(f"Loading CLK: {args.clk}", file=sys.stderr)
        clk_file = CLKFile(args.clk)
        clk_prns = sorted(clk_file.prns())
        print(f"  {len(clk_prns)} satellites, "
              f"{clk_file.n_epochs(clk_prns[0])} epochs", file=sys.stderr)

    print(f"Loading RAWX: {args.rawx} (systems: {enabled})", file=sys.stderr)
    epochs = load_ppp_epochs(args.rawx, systems=enabled, osb=osb)
    print(f"  {len(epochs)} epochs with ≥4 dual-freq SVs", file=sys.stderr)

    if not epochs:
        print("ERROR: No valid PPP epochs", file=sys.stderr)
        sys.exit(1)

    sig_counts = defaultdict(int)
    for o in epochs[0][1]:
        sig_counts[o['sys']] += 1
    print(f"  First epoch: {dict(sig_counts)}", file=sys.stderr)

    known_ecef = None
    if args.known_pos:
        lat, lon, alt = [float(v) for v in args.known_pos.split(',')]
        known_ecef = lla_to_ecef(lat, lon, alt)
        print(f"  Known position: {lat:.6f}, {lon:.6f}, {alt:.1f}m",
              file=sys.stderr)

    leap_delta = timedelta(seconds=args.leap)

    # ── Fixed-position mode ──
    if args.fix_pos:
        if known_ecef is None:
            print("ERROR: --fix-pos requires --known-pos", file=sys.stderr)
            sys.exit(1)
        print(f"\n  Fixed-position mode: TD carrier phase + pseudorange",
              file=sys.stderr)
        filt = FixedPosFilter(known_ecef)
        filt.prev_clock = 0.0  # Initialize previous clock for TD
        clk_estimates = []
        start = args.skip
        end = len(epochs) if args.limit == 0 else min(len(epochs), start + args.limit)
        prev_t = None
        for i in range(start, end):
            ts_str, obs = epochs[i]
            t = timestamp_to_gpstime(ts_str) + leap_delta

            if prev_t is not None:
                dt = (t - prev_t).total_seconds()
                filt.predict(dt)
            prev_t = t

            # Strip phase if --no-phase
            if args.no_phase:
                obs = [dict(o, phi_if_m=None) for o in obs]

            result = filt.update(obs, sp3, t, clk_file=clk_file)
            n_used = result[0]
            resid = result[1] if len(result) > 1 else np.array([])
            n_td = result[2] if len(result) > 2 else 0

            clk_ns = filt.x[filt.IDX_CLK] / C * 1e9
            clk_rate_ppb = filt.x[filt.IDX_CLK_RATE] / C * 1e9
            clk_sigma = math.sqrt(filt.P[filt.IDX_CLK, filt.IDX_CLK]) / C * 1e9
            rms = np.sqrt(np.mean(resid**2)) if len(resid) > 0 else 0

            clk_estimates.append((ts_str, clk_ns, clk_sigma, n_used, n_td, rms))

            if (i - start + 1) % 60 == 0:
                print(f"  [{i-start+1}/{end-start}] {ts_str[:19]} "
                      f"clk={clk_ns:.1f}ns ±{clk_sigma:.2f}ns rate={clk_rate_ppb:.3f}ppb "
                      f"n={n_used} td={n_td} rms={rms:.3f}m", file=sys.stderr)

        # Summary
        print(f"\n{'='*60}", file=sys.stderr)
        print(f"  Fixed-position PPP clock estimation (TD carrier phase)",
              file=sys.stderr)
        print(f"  Carrier phase: {'disabled' if args.no_phase else 'time-differenced'}",
              file=sys.stderr)
        print(f"  Epochs: {len(clk_estimates)}", file=sys.stderr)
        clks = np.array([c[1] for c in clk_estimates])
        sigs = np.array([c[2] for c in clk_estimates])
        print(f"  Clock: {np.mean(clks):.1f} ± {np.std(clks):.2f} ns",
              file=sys.stderr)
        print(f"  Clock sigma (filter): {np.mean(sigs):.3f} ns",
              file=sys.stderr)
        # Clock rate (drift)
        if len(clks) > 100:
            dt_s = np.arange(len(clks))
            slope = np.polyfit(dt_s, clks, 1)[0]
            print(f"  Clock rate: {slope:.3f} ns/epoch ({slope*1e3:.1f} ps/epoch)",
                  file=sys.stderr)
            # Detrended stability
            detrended = clks - np.polyval(np.polyfit(dt_s, clks, 1), dt_s)
            print(f"  Detrended std: {np.std(detrended):.3f} ns ({np.std(detrended)*1e3:.1f} ps)",
                  file=sys.stderr)
            # Last 25% stability
            n_last = max(1, len(clks) // 4)
            last_clks = clks[-n_last:]
            last_dt = dt_s[-n_last:]
            last_slope = np.polyfit(last_dt, last_clks, 1)[0]
            last_detrend = last_clks - np.polyval(np.polyfit(last_dt, last_clks, 1), last_dt)
            print(f"  Last {n_last} epochs: rate={last_slope:.3f} ns/epoch, "
                  f"std={np.std(last_detrend):.3f} ns ({np.std(last_detrend)*1e3:.1f} ps)",
                  file=sys.stderr)
        print(f"{'='*60}", file=sys.stderr)

        if args.out:
            with open(args.out, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['timestamp', 'clock_ns', 'clock_sigma_ns',
                            'n_meas', 'n_td', 'rms_m'])
                for row in clk_estimates:
                    w.writerow([row[0], f'{row[1]:.3f}', f'{row[2]:.4f}',
                                row[3], row[4], f'{row[5]:.4f}'])
            print(f"  Output: {args.out}", file=sys.stderr)
        return

    out_f = None
    out_w = None
    if args.out:
        out_f = open(args.out, 'w', newline='')
        out_w = csv.writer(out_f)
        out_w.writerow(['timestamp', 'lat_deg', 'lon_deg', 'alt_m',
                        'clock_bias_ns', 'isb_gal_ns', 'isb_bds_ns',
                        'n_sv', 'n_gps', 'n_gal', 'n_bds',
                        'sigma_e_m', 'sigma_n_m', 'sigma_u_m',
                        'east_err_m', 'north_err_m', 'up_err_m',
                        'n_ambiguities'])

    ekf = PPPFilter()
    errors = []
    positions = []
    n_total = 0
    prev_t = None
    converge_epoch = None

    start = args.skip
    end = len(epochs) if args.limit == 0 else min(len(epochs), start + args.limit)

    for i in range(start, end):
        ts_str, obs = epochs[i]
        t = timestamp_to_gpstime(ts_str) + leap_delta
        n_total += 1

        if args.no_phase or (args.phase_delay > 0 and n_total <= args.phase_delay):
            obs = [dict(o, phi_if_m=None) for o in obs]

        if not ekf.initialized:
            x_ls, converged, n_sv = ls_init(obs, sp3, t, clk_file=clk_file)
            if not converged or n_sv < 4:
                continue
            ekf.initialize(x_ls[:3], x_ls[3], x_ls[4], x_ls[5])
            print(f"  EKF initialized at epoch {n_total} with {n_sv} SVs",
                  file=sys.stderr)
            prev_t = t

            for o in obs:
                if o.get('phi_if_m') is not None:
                    N_init = o['phi_if_m'] - o['pr_if']
                    ekf.add_ambiguity(o['sv'], N_init)
                    ekf.prev_obs[o['sv']] = o

            n_used, _, sys_counts = ekf.update(obs, sp3, t, clk_file=clk_file)
            if n_used < 4:
                ekf = PPPFilter()
                continue
        else:
            dt = (t - prev_t).total_seconds()
            ekf.predict(dt)
            prev_t = t

            slipped = ekf.detect_cycle_slips(obs, ekf.prev_obs)
            for sv in slipped:
                ekf.remove_ambiguity(sv)
                for o in obs:
                    if o['sv'] == sv and o.get('phi_if_m') is not None:
                        ekf.add_ambiguity(sv, o['phi_if_m'] - o['pr_if'])
                        break

            current_svs = {o['sv'] for o in obs}
            tracked = set(ekf.sv_to_idx.keys())
            for sv in tracked - current_svs:
                ekf.remove_ambiguity(sv)
            for o in obs:
                if o['sv'] not in ekf.sv_to_idx and o.get('phi_if_m') is not None:
                    ekf.add_ambiguity(o['sv'], o['phi_if_m'] - o['pr_if'])

            n_used, _, sys_counts = ekf.update(obs, sp3, t, clk_file=clk_file)
            for o in obs:
                ekf.prev_obs[o['sv']] = o

        pos = ekf.x[:3]
        lat, lon, alt = ecef_to_lla(pos[0], pos[1], pos[2])
        clk_ns = ekf.x[IDX_CLK] / C * 1e9
        isb_gal_ns = ekf.x[IDX_ISB_GAL] / C * 1e9
        isb_bds_ns = ekf.x[IDX_ISB_BDS] / C * 1e9
        n_amb = len(ekf.sv_to_idx)
        n_gps = sys_counts.get('gps', 0)
        n_gal = sys_counts.get('gal', 0)
        n_bds = sys_counts.get('bds', 0)

        sigma_e = sigma_n = sigma_u = 0
        if ekf.P is not None:
            P_pos = ekf.P[:3, :3]
            lat_r = math.radians(lat)
            lon_r = math.radians(lon)
            R_enu = np.array([
                [-math.sin(lon_r), math.cos(lon_r), 0],
                [-math.sin(lat_r)*math.cos(lon_r), -math.sin(lat_r)*math.sin(lon_r), math.cos(lat_r)],
                [math.cos(lat_r)*math.cos(lon_r), math.cos(lat_r)*math.sin(lon_r), math.sin(lat_r)]
            ])
            P_enu = R_enu @ P_pos @ R_enu.T
            sigma_e = math.sqrt(max(0, P_enu[0, 0]))
            sigma_n = math.sqrt(max(0, P_enu[1, 1]))
            sigma_u = math.sqrt(max(0, P_enu[2, 2]))

        e_err = n_err = u_err = 0
        if known_ecef is not None:
            enu = ecef_to_enu(pos - known_ecef, known_ecef)
            e_err, n_err, u_err = enu
            errors.append(enu)
            err_3d = math.sqrt(e_err**2 + n_err**2 + u_err**2)
            if converge_epoch is None and err_3d < 0.1:
                converge_epoch = n_total

        positions.append((lat, lon, alt))

        if out_w:
            out_w.writerow([ts_str, f'{lat:.7f}', f'{lon:.7f}', f'{alt:.2f}',
                            f'{clk_ns:.1f}', f'{isb_gal_ns:.1f}', f'{isb_bds_ns:.1f}',
                            n_used, n_gps, n_gal, n_bds,
                            f'{sigma_e:.4f}', f'{sigma_n:.4f}', f'{sigma_u:.4f}',
                            f'{e_err:.3f}', f'{n_err:.3f}', f'{u_err:.3f}',
                            n_amb])

        if n_total % 60 == 0:
            err_str = ""
            if known_ecef is not None:
                err_str = (f" err=({e_err:+.2f},{n_err:+.2f},{u_err:+.2f})m"
                           f" sig=({sigma_e:.2f},{sigma_n:.2f},{sigma_u:.2f})m")
            print(f"  [{n_total}/{end-start}] {ts_str[:19]} "
                  f"n_sv={n_used}(G{n_gps}E{n_gal}C{n_bds}) "
                  f"n_amb={n_amb}{err_str}",
                  file=sys.stderr)

    if out_f:
        out_f.close()

    # Summary
    print(f"\n{'='*60}", file=sys.stderr)
    sys_str = '+'.join(s.upper() for s in sorted(enabled))
    print(f"  PPP EKF — {sys_str} IF pseudorange + carrier phase",
          file=sys.stderr)
    print(f"  Carrier phase: {'disabled' if args.no_phase else 'enabled'}",
          file=sys.stderr)
    print(f"  Epochs processed: {n_total}", file=sys.stderr)
    print(f"  Solutions:        {len(positions)}", file=sys.stderr)

    if positions:
        lats = [p[0] for p in positions]
        lons = [p[1] for p in positions]
        alts = [p[2] for p in positions]
        print(f"  Lat:  {np.mean(lats):.7f} ± {np.std(lats)*111000:.2f}m",
              file=sys.stderr)
        cos_lat = math.cos(math.radians(np.mean(lats)))
        print(f"  Lon:  {np.mean(lons):.7f} ± {np.std(lons)*111000*cos_lat:.2f}m",
              file=sys.stderr)
        print(f"  Alt:  {np.mean(alts):.1f} ± {np.std(alts):.2f}m",
              file=sys.stderr)

    if errors:
        errors = np.array(errors)
        print(f"\n  vs known position:", file=sys.stderr)
        print(f"    East:  {np.mean(errors[:,0]):+.3f} ± {np.std(errors[:,0]):.3f}m",
              file=sys.stderr)
        print(f"    North: {np.mean(errors[:,1]):+.3f} ± {np.std(errors[:,1]):.3f}m",
              file=sys.stderr)
        print(f"    Up:    {np.mean(errors[:,2]):+.3f} ± {np.std(errors[:,2]):.3f}m",
              file=sys.stderr)
        print(f"    3D RMS: {np.sqrt(np.mean(errors**2)):.3f}m", file=sys.stderr)

        if converge_epoch is not None:
            print(f"    Convergence to <0.1m at epoch {converge_epoch}",
                  file=sys.stderr)
        else:
            print(f"    Did not converge to <0.1m within {n_total} epochs",
                  file=sys.stderr)

        n_last = max(1, len(errors) // 4)
        last_errs = errors[-n_last:]
        print(f"\n  Last {n_last} epochs (post-convergence):", file=sys.stderr)
        print(f"    East:  {np.mean(last_errs[:,0]):+.3f} ± {np.std(last_errs[:,0]):.3f}m",
              file=sys.stderr)
        print(f"    North: {np.mean(last_errs[:,1]):+.3f} ± {np.std(last_errs[:,1]):.3f}m",
              file=sys.stderr)
        print(f"    Up:    {np.mean(last_errs[:,2]):+.3f} ± {np.std(last_errs[:,2]):.3f}m",
              file=sys.stderr)
        print(f"    3D RMS: {np.sqrt(np.mean(last_errs**2)):.3f}m",
              file=sys.stderr)

    print(f"{'='*60}", file=sys.stderr)


if __name__ == "__main__":
    main()

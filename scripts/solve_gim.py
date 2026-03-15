#!/usr/bin/env python3
"""
solve_gim.py — Multi-GNSS single-frequency PPP with GIM ionosphere correction.

Instead of dual-frequency IF combination (which cancels iono but limits us to
~4 GPS L5-capable satellites), this uses single-frequency L1-band observations
from ALL constellations (GPS L1CA + Galileo E1C + BDS B1I ≈ 15+ satellites)
and applies ionosphere corrections from IGS Global Ionosphere Maps (GIM/IONEX).

Trade-off: GIM accuracy is ~2-5 TECU (~0.3-0.8m at L1 zenith), but with 3-4x
more satellites the improved geometry more than compensates.

State vector (EKF):
    x = [X, Y, Z, c*dt_gps, ISB_gal, ISB_bds, N1, N2, ..., Nn]
    - Position (3): static, tiny process noise
    - GPS receiver clock (1): random walk
    - Inter-system biases (2): GAL-GPS, BDS-GPS offsets (slowly varying)
    - Float ambiguities (n): one per tracked satellite, zero process noise

Usage:
    python solve_gim.py data/rawx_1h_top_20260303.csv data/gfz_mgx_062.sp3 \\
        data/COD0OPSRAP_062.INX \\
        --known-pos "41.8430626,-88.1037190,201.671" --out data/pos_gim.csv
"""

import argparse
import csv
import math
import re
import sys
from collections import defaultdict
from datetime import datetime, timezone, timedelta

import numpy as np

from solve_pseudorange import (
    SP3, C, OMEGA_E, ecef_to_lla, ecef_to_enu, lla_to_ecef,
    timestamp_to_gpstime,
)

# ── Frequencies ────────────────────────────────────────────────────────────── #
F_GPS_L1 = 1575.42e6     # GPS L1C/A (Hz)
F_GAL_E1 = 1575.42e6     # Galileo E1 (same as GPS L1)
F_BDS_B1I = 1561.098e6   # BDS B1I (slightly different)

# Iono delay coefficient: delay_m = IONO_COEFF / f^2 * STEC_TECU
# where IONO_COEFF = 40.3e16
IONO_COEFF = 40.3e16

# Per-frequency iono scale (m per TECU at zenith)
IONO_SCALE_GPS = IONO_COEFF / F_GPS_L1**2   # ~0.1624 m/TECU
IONO_SCALE_GAL = IONO_COEFF / F_GAL_E1**2   # same as GPS
IONO_SCALE_BDS = IONO_COEFF / F_BDS_B1I**2  # ~0.1654 m/TECU

# L1-band wavelengths for carrier phase
WL_GPS_L1 = C / F_GPS_L1
WL_GAL_E1 = C / F_GAL_E1
WL_BDS_B1I = C / F_BDS_B1I

# Signal configs: (signal_id, sv_prefix, iono_scale, wavelength)
SIGNAL_CONFIG = [
    ('GPS-L1CA', 'G', IONO_SCALE_GPS, WL_GPS_L1, 'gps'),
    ('GAL-E1C',  'E', IONO_SCALE_GAL, WL_GAL_E1, 'gal'),
    ('BDS-B1I',  'C', IONO_SCALE_BDS, WL_BDS_B1I, 'bds'),
]

# Measurement noise
SIGMA_PR = 3.0       # pseudorange (m)
SIGMA_CP = 0.01      # carrier phase (m)

ELEV_MASK = 10.0     # degrees

# BDS-2 GEO/IGSO satellites have poor SP3 orbits — exclude them
# BDS-3 MEO (PRN >= 19) have good orbits
BDS_MIN_PRN = 19

# EKF state layout
IDX_X, IDX_Y, IDX_Z = 0, 1, 2
IDX_CLK = 3          # GPS clock
IDX_ISB_GAL = 4      # Galileo ISB
IDX_ISB_BDS = 5      # BDS ISB
N_BASE = 6           # fixed states before ambiguities


# ── IONEX Parser ──────────────────────────────────────────────────────────── #
class IONEX:
    """Parse IONEX GIM file and interpolate VTEC at any location and time."""

    def __init__(self, path):
        self.maps = []       # list of (datetime, 2D-array of VTEC in TECU)
        self.lat_range = None  # (lat1, lat2, dlat)
        self.lon_range = None  # (lon1, lon2, dlon)
        self.height = 450.0    # single-layer height (km)
        self.dcb = {}          # {sv: dcb_ns} satellite differential code biases
        self._parse(path)

    def dcb_meters(self, sv):
        """Return DCB correction in meters for a satellite.
        DCB stored in ns, convert to meters."""
        if sv in self.dcb:
            return self.dcb[sv] * 1e-9 * C
        return 0.0

    def _parse(self, path):
        with open(path) as f:
            lines = f.readlines()

        i = 0
        in_dcb = False
        # Parse header + DCB section
        while i < len(lines):
            line = lines[i]
            if 'HGT1 / HGT2 / DHGT' in line:
                self.height = float(line[:8])
            elif 'LAT1 / LAT2 / DLAT' in line:
                parts = line[:60].split()
                self.lat_range = (float(parts[0]), float(parts[1]), float(parts[2]))
            elif 'LON1 / LON2 / DLON' in line:
                parts = line[:60].split()
                self.lon_range = (float(parts[0]), float(parts[1]), float(parts[2]))
            elif 'START OF AUX DATA' in line:
                in_dcb = True
            elif 'END OF AUX DATA' in line:
                in_dcb = False
            elif in_dcb and 'PRN / BIAS / RMS' in line:
                # Parse satellite DCB: "   G01     3.569     0.091"
                sv = line[3:6].strip()
                try:
                    dcb_ns = float(line[6:16])
                    self.dcb[sv] = dcb_ns
                except ValueError:
                    pass
            elif 'END OF HEADER' in line:
                i += 1
                break
            i += 1

        # Parse TEC maps
        while i < len(lines):
            line = lines[i]
            if 'START OF TEC MAP' in line:
                i += 1
                # Epoch line
                parts = lines[i].split()
                epoch = datetime(int(parts[0]), int(parts[1]), int(parts[2]),
                                 int(parts[3]), int(parts[4]), int(parts[5]),
                                 tzinfo=timezone.utc)
                i += 1
                tec_map = self._parse_map(lines, i)
                self.maps.append((epoch, tec_map))
                # Skip to end of this map
                while i < len(lines) and 'END OF TEC MAP' not in lines[i]:
                    i += 1
            i += 1

    def _parse_map(self, lines, start):
        """Parse one TEC map starting at given line index."""
        lat1, lat2, dlat = self.lat_range
        lon1, lon2, dlon = self.lon_range
        n_lat = int((lat2 - lat1) / dlat) + 1
        n_lon = int((lon2 - lon1) / dlon) + 1

        tec = np.zeros((n_lat, n_lon))
        i = start
        lat_idx = 0

        while lat_idx < n_lat and i < len(lines):
            line = lines[i]
            if 'LAT/LON1/LON2/DLON/H' in line:
                # Read VTEC values for this latitude
                i += 1
                values = []
                while len(values) < n_lon and i < len(lines):
                    vals = lines[i].split()
                    if any(keyword in lines[i] for keyword in
                           ['LAT/LON', 'END OF TEC', 'START OF']):
                        break
                    values.extend([float(v) for v in vals])
                    i += 1
                # IONEX stores VTEC * exponent (usually 10^-1, so values are in 0.1 TECU)
                tec[lat_idx, :len(values)] = values[:n_lon]
                lat_idx += 1
            else:
                i += 1

        # Convert from IONEX units (0.1 TECU) to TECU
        tec *= 0.1
        return tec

    def vtec(self, lat_deg, lon_deg, t):
        """Interpolate VTEC at geodetic lat/lon and time t (datetime).

        Returns VTEC in TECU.
        """
        # Temporal interpolation: find bracketing maps
        if len(self.maps) < 2:
            return self.maps[0][1] if self.maps else 0.0

        t0, t1 = None, None
        map0, map1 = None, None
        for j in range(len(self.maps) - 1):
            if self.maps[j][0] <= t <= self.maps[j + 1][0]:
                t0, map0 = self.maps[j]
                t1, map1 = self.maps[j + 1]
                break

        if t0 is None:
            # Outside range — use nearest
            if t < self.maps[0][0]:
                return self._interp_spatial(self.maps[0][1], lat_deg, lon_deg)
            else:
                return self._interp_spatial(self.maps[-1][1], lat_deg, lon_deg)

        dt_total = (t1 - t0).total_seconds()
        dt = (t - t0).total_seconds()
        frac = dt / dt_total if dt_total > 0 else 0.0

        v0 = self._interp_spatial(map0, lat_deg, lon_deg)
        v1 = self._interp_spatial(map1, lat_deg, lon_deg)
        return v0 * (1 - frac) + v1 * frac

    def _interp_spatial(self, tec_map, lat_deg, lon_deg):
        """Bilinear interpolation in lat/lon grid."""
        lat1, lat2, dlat = self.lat_range
        lon1, lon2, dlon = self.lon_range

        # Normalize longitude to grid range
        while lon_deg < lon1:
            lon_deg += 360
        while lon_deg > lon2:
            lon_deg -= 360

        # Grid indices (float)
        fi = (lat_deg - lat1) / dlat
        fj = (lon_deg - lon1) / dlon

        n_lat, n_lon = tec_map.shape
        fi = max(0, min(fi, n_lat - 1.001))
        fj = max(0, min(fj, n_lon - 1.001))

        i0 = int(fi)
        j0 = int(fj)
        i1 = min(i0 + 1, n_lat - 1)
        j1 = min(j0 + 1, n_lon - 1)

        di = fi - i0
        dj = fj - j0

        v = (tec_map[i0, j0] * (1 - di) * (1 - dj) +
             tec_map[i1, j0] * di * (1 - dj) +
             tec_map[i0, j1] * (1 - di) * dj +
             tec_map[i1, j1] * di * dj)
        return v

    def iono_delay(self, receiver_ecef, sat_ecef, t, freq):
        """Compute ionospheric delay for a signal at given frequency.

        Returns delay in meters (positive = signal is delayed).
        """
        # Compute IPP (ionospheric pierce point) and zenith angle
        ipp_lat, ipp_lon, mf = self._ipp_and_mapping(receiver_ecef, sat_ecef)

        # Get VTEC at IPP
        vtec = self.vtec(ipp_lat, ipp_lon, t)

        # STEC = VTEC * mapping_function
        stec = vtec * mf

        # Delay = 40.3e16 / f^2 * STEC (TECU)
        delay = IONO_COEFF / (freq ** 2) * stec
        return delay

    def _ipp_and_mapping(self, receiver_ecef, sat_ecef):
        """Compute ionospheric pierce point and mapping function.

        Uses single-layer model at self.height km.
        Returns (ipp_lat_deg, ipp_lon_deg, mapping_factor).
        """
        R_E = 6371.0  # km
        H_ion = self.height  # km (from IONEX header)

        # Receiver geodetic position
        lat_r, lon_r, alt_r = ecef_to_lla(
            receiver_ecef[0], receiver_ecef[1], receiver_ecef[2])
        lat_r_rad = math.radians(lat_r)
        lon_r_rad = math.radians(lon_r)

        # Satellite direction from receiver
        dx = sat_ecef - receiver_ecef
        rho = np.linalg.norm(dx)
        e = dx / rho  # unit vector to satellite

        # Elevation angle
        up = receiver_ecef / np.linalg.norm(receiver_ecef)
        sin_elev = np.dot(e, up)
        elev = math.asin(max(-1.0, min(1.0, sin_elev)))

        # Zenith angle at receiver
        z = math.pi / 2 - elev

        # Zenith angle at IPP (single-layer model)
        sin_z_prime = R_E / (R_E + H_ion) * math.sin(z)
        sin_z_prime = min(sin_z_prime, 0.9999)
        z_prime = math.asin(sin_z_prime)

        # Mapping function
        mf = 1.0 / math.cos(z_prime)

        # IPP location (approximate using azimuth)
        # Earth-central angle from receiver to IPP
        psi = math.pi / 2 - elev - z_prime

        # Azimuth of satellite
        # Project satellite direction into local ENU
        enu = np.array([
            -math.sin(lon_r_rad) * dx[0] + math.cos(lon_r_rad) * dx[1],
            (-math.sin(lat_r_rad) * math.cos(lon_r_rad) * dx[0]
             - math.sin(lat_r_rad) * math.sin(lon_r_rad) * dx[1]
             + math.cos(lat_r_rad) * dx[2]),
            (math.cos(lat_r_rad) * math.cos(lon_r_rad) * dx[0]
             + math.cos(lat_r_rad) * math.sin(lon_r_rad) * dx[1]
             + math.sin(lat_r_rad) * dx[2]),
        ])
        az = math.atan2(enu[0], enu[1])

        # IPP latitude (geocentric approximation, as per IONEX convention)
        ipp_lat = math.asin(
            math.sin(lat_r_rad) * math.cos(psi) +
            math.cos(lat_r_rad) * math.sin(psi) * math.cos(az))
        ipp_lat_deg = math.degrees(ipp_lat)

        # IPP longitude
        ipp_lon = lon_r_rad + math.asin(
            math.sin(psi) * math.sin(az) / math.cos(ipp_lat))
        ipp_lon_deg = math.degrees(ipp_lon)

        return ipp_lat_deg, ipp_lon_deg, mf


# ── Multi-GNSS L1 RAWX loader ────────────────────────────────────────────── #
def load_l1_epochs(csv_path):
    """Load RAWX CSV, extract L1-band observations from all constellations.

    Returns list of (timestamp_str, [{sv, pr, cp_m, cno, lock_ms, half_cyc,
                                       signal, sys, iono_scale, wavelength}, ...])
    """
    sig_map = {}
    for sig_id, prefix, iono_sc, wl, sys_name in SIGNAL_CONFIG:
        sig_map[sig_id] = (prefix, iono_sc, wl, sys_name)

    raw = defaultdict(list)
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            sig = row['signal_id']
            if sig not in sig_map:
                continue
            if row['pr_valid'] != '1':
                continue
            prefix, iono_sc, wl, sys_name = sig_map[sig]
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
            sv_num = int(row['sv_id'])
            # Filter out BDS-2 GEO/IGSO (poor SP3 orbits)
            if sys_name == 'bds' and sv_num < BDS_MIN_PRN:
                continue
            sv = f"{prefix}{sv_num:02d}"
            raw[row['timestamp']].append({
                'sv': sv,
                'pr': pr,
                'cp_m': cp * wl if cp is not None else None,
                'cno': cno,
                'lock_ms': lock_ms,
                'half_cyc': half_cyc,
                'signal': sig,
                'sys': sys_name,
                'iono_scale': iono_sc,
                'wavelength': wl,
            })

    result = []
    for ts in sorted(raw.keys()):
        obs = raw[ts]
        if len(obs) >= 6:  # need enough for 3D + clock + ISBs
            result.append((ts, obs))
    return result


# ── GIM-corrected PPP EKF ─────────────────────────────────────────────────── #
class GIMFilter:
    """Multi-GNSS EKF with GIM ionosphere correction and carrier phase."""

    def __init__(self):
        self.x = None
        self.P = None
        self.sv_to_idx = {}
        self.prev_obs = {}
        self.initialized = False

    def initialize(self, pos_ecef, clock_m):
        self.x = np.zeros(N_BASE)
        self.x[:3] = pos_ecef
        self.x[IDX_CLK] = clock_m
        self.x[IDX_ISB_GAL] = 0.0
        self.x[IDX_ISB_BDS] = 0.0
        self.P = np.diag([
            100.0**2, 100.0**2, 100.0**2,  # position
            1e8,                             # clock
            1e6,                             # ISB GAL
            1e6,                             # ISB BDS
        ])
        self.sv_to_idx = {}
        self.prev_obs = {}
        self.initialized = True

    def predict(self, dt):
        if dt <= 0:
            dt = 1.0
        n = len(self.x)
        Q = np.zeros((n, n))
        for i in range(3):
            Q[i, i] = 1e-6 * dt
        Q[IDX_CLK, IDX_CLK] = 1e6 * dt
        Q[IDX_ISB_GAL, IDX_ISB_GAL] = 1.0 * dt   # ISBs drift slowly
        Q[IDX_ISB_BDS, IDX_ISB_BDS] = 1.0 * dt
        # Ambiguities: zero process noise
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

    def tropo_delay(self, elevation_deg):
        if elevation_deg < 5.0:
            elevation_deg = 5.0
        return 2.3 / math.sin(math.radians(elevation_deg))

    def compute_elevation(self, receiver_pos, sat_pos):
        dx = sat_pos - receiver_pos
        r = np.linalg.norm(dx)
        if r < 1.0:
            return 90.0
        e = dx / r
        up = receiver_pos / np.linalg.norm(receiver_pos)
        sin_elev = np.dot(e, up)
        return math.degrees(math.asin(max(-1.0, min(1.0, sin_elev))))

    def isb_index(self, sys_name):
        """Return the state index for this system's clock/ISB."""
        if sys_name == 'gps':
            return IDX_CLK
        elif sys_name == 'gal':
            return IDX_ISB_GAL
        elif sys_name == 'bds':
            return IDX_ISB_BDS
        return IDX_CLK

    def update(self, observations, sp3, t, ionex, t_utc):
        """Measurement update with GIM ionosphere correction.

        For each satellite:
          - Compute geometric range, tropo, iono (from GIM)
          - Form pseudorange and carrier phase residuals
          - Build H matrix including ISB columns
        """
        H_rows = []
        z_rows = []
        R_diag = []
        n_used = 0
        receiver_pos = self.x[:3]

        for obs in observations:
            sv = obs['sv']
            sat_pos, sat_clk = sp3.sat_position(sv, t)
            if sat_pos is None or sat_clk is None:
                continue
            if sat_clk > 0.9:
                continue

            # Geometric range with Earth rotation correction
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

            # GIM ionosphere correction
            iono = ionex.iono_delay(receiver_pos, sat_rot, t_utc,
                                    C / obs['wavelength'])

            e_los = dx / rho

            # Clock + ISB for this system
            clk_idx = self.isb_index(obs['sys'])
            if obs['sys'] == 'gps':
                clk_val = self.x[IDX_CLK]
            else:
                clk_val = self.x[IDX_CLK] + self.x[clk_idx]

            # Predicted pseudorange (iono-corrected)
            rho_pred = rho + clk_val - sat_clk * C + tropo + iono

            # Weight by elevation and C/N0
            cno_factor = 10 ** ((obs['cno'] - 35) / 20)
            elev_factor = math.sin(math.radians(elev))
            w = max(0.01, cno_factor * elev_factor)

            # --- Pseudorange ---
            dz_pr = obs['pr'] - rho_pred
            h_pr = np.zeros(len(self.x))
            h_pr[0] = -e_los[0]
            h_pr[1] = -e_los[1]
            h_pr[2] = -e_los[2]
            h_pr[IDX_CLK] = 1.0
            if obs['sys'] != 'gps':
                h_pr[clk_idx] = 1.0
            H_rows.append(h_pr)
            z_rows.append(dz_pr)
            R_diag.append((SIGMA_PR / w) ** 2)

            # --- Carrier phase ---
            if (obs['cp_m'] is not None and obs['half_cyc'] == '1'
                    and sv in self.sv_to_idx):
                amb_idx = N_BASE + self.sv_to_idx[sv]
                # Carrier phase: iono has opposite sign for phase; DCB doesn't apply to phase
                rho_pred_phi = rho + clk_val - sat_clk * C + tropo - iono
                dz_phi = obs['cp_m'] - rho_pred_phi - self.x[amb_idx]
                h_phi = np.zeros(len(self.x))
                h_phi[0] = -e_los[0]
                h_phi[1] = -e_los[1]
                h_phi[2] = -e_los[2]
                h_phi[IDX_CLK] = 1.0
                if obs['sys'] != 'gps':
                    h_phi[clk_idx] = 1.0
                h_phi[amb_idx] = 1.0
                H_rows.append(h_phi)
                z_rows.append(dz_phi)
                R_diag.append((SIGMA_CP / w) ** 2)

            n_used += 1

        if n_used < 4:
            return n_used, np.array([]), {}

        H = np.array(H_rows)
        z = np.array(z_rows)
        R = np.diag(R_diag)

        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return n_used, z, {}

        self.x = self.x + K @ z
        I_KH = np.eye(len(self.x)) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        post_resid = z - H @ (K @ z)

        # Count by system
        sys_counts = defaultdict(int)
        for obs in observations:
            sv = obs['sv']
            sat_pos, _ = sp3.sat_position(sv, t)
            if sat_pos is not None:
                sys_counts[obs['sys']] += 1

        return n_used, post_resid, dict(sys_counts)


# ── Initial LS solve (multi-GNSS, pseudorange-only) ──────────────────────── #
def ls_solve_multigps(observations, sp3, t, ionex, t_utc):
    """Weighted LS pseudorange solve for initial position.

    Adaptively solves for [X, Y, Z, c*dt_gps] + ISBs for present systems.
    Returns state as full 6-element vector [X, Y, Z, clk, ISB_gal, ISB_bds].
    """
    # Determine which systems are present
    present_sys = set(o['sys'] for o in observations)
    # Build column mapping: always X,Y,Z,clk; optionally ISB_gal, ISB_bds
    col_map = {0: 0, 1: 1, 2: 2, 3: 3}  # pos + clock always present
    n_params = 4
    gal_col = bds_col = None
    if 'gal' in present_sys:
        gal_col = n_params
        n_params += 1
    if 'bds' in present_sys:
        bds_col = n_params
        n_params += 1

    x = np.zeros(n_params)
    x[2] = 6371000.0

    # Initialize from satellite centroid
    sat_positions = []
    for obs in observations:
        sp, _ = sp3.sat_position(obs['sv'], t)
        if sp is not None:
            sat_positions.append(sp)
    if len(sat_positions) >= 4:
        avg = np.mean(sat_positions, axis=0)
        r = np.linalg.norm(avg)
        if r > 0:
            x[:3] = avg / r * 6371000.0

    for iteration in range(20):
        H = []
        dz = []
        W = []

        for obs in observations:
            sat_pos, sat_clk = sp3.sat_position(obs['sv'], t)
            if sat_pos is None or sat_clk is None:
                continue
            if sat_clk > 0.9:
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

            # System clock value
            clk = x[3]
            if obs['sys'] == 'gal' and gal_col is not None:
                clk += x[gal_col]
            elif obs['sys'] == 'bds' and bds_col is not None:
                clk += x[bds_col]

            pr_pred = rho + clk - sat_clk * C
            h = np.zeros(n_params)
            h[0] = e[0]
            h[1] = e[1]
            h[2] = e[2]
            h[3] = 1.0
            if obs['sys'] == 'gal' and gal_col is not None:
                h[gal_col] = 1.0
            elif obs['sys'] == 'bds' and bds_col is not None:
                h[bds_col] = 1.0

            H.append(h)
            dz.append(obs['pr'] - pr_pred)
            w = 10 ** ((obs['cno'] - 30) / 20)
            W.append(w)

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
            result[:min(len(x), 6)] = x[:min(len(x), 6)]
            return result, False, len(H)

        x += dx
        if np.linalg.norm(dx[:3]) < 0.01:
            # Map back to full 6-element state
            result = np.zeros(6)
            result[:4] = x[:4]
            if gal_col is not None:
                result[4] = x[gal_col]
            if bds_col is not None:
                result[5] = x[bds_col]
            return result, True, len(H)

    result = np.zeros(6)
    result[:4] = x[:4]
    if gal_col is not None:
        result[4] = x[gal_col]
    if bds_col is not None:
        result[5] = x[bds_col]
    return result, True, len(H)


# ── Main ──────────────────────────────────────────────────────────────────── #
def main():
    ap = argparse.ArgumentParser(
        description="Multi-GNSS single-freq PPP with GIM ionosphere correction")
    ap.add_argument("rawx", help="RAWX CSV file (testAnt format)")
    ap.add_argument("sp3", help="SP3 precise orbit file (multi-GNSS)")
    ap.add_argument("ionex", help="IONEX GIM file")
    ap.add_argument("--out", default=None)
    ap.add_argument("--skip", type=int, default=0)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--known-pos", default=None,
                    help="Known position as lat,lon,alt for error stats")
    ap.add_argument("--leap", type=int, default=18)
    ap.add_argument("--no-phase", action='store_true',
                    help="Pseudorange-only (no carrier phase)")
    ap.add_argument("--iono-scale", type=float, default=1.0,
                    help="Scale factor for GIM VTEC (for diagnostics)")
    ap.add_argument("--systems", default="gps,gal,bds",
                    help="Comma-separated list of systems to use (gps,gal,bds)")
    args = ap.parse_args()

    print(f"Loading SP3: {args.sp3}", file=sys.stderr)
    sp3 = SP3(args.sp3)
    print(f"  {len(sp3.epochs)} epochs, {len(sp3.positions)} satellites",
          file=sys.stderr)

    # Count by constellation
    n_g = sum(1 for s in sp3.positions if s.startswith('G'))
    n_e = sum(1 for s in sp3.positions if s.startswith('E'))
    n_c = sum(1 for s in sp3.positions if s.startswith('C'))
    print(f"  GPS: {n_g}, GAL: {n_e}, BDS: {n_c}", file=sys.stderr)

    print(f"Loading IONEX: {args.ionex}", file=sys.stderr)
    ionex = IONEX(args.ionex)
    if args.iono_scale != 1.0:
        print(f"  Scaling VTEC by {args.iono_scale}", file=sys.stderr)
        for j, (epoch, tec_map) in enumerate(ionex.maps):
            ionex.maps[j] = (epoch, tec_map * args.iono_scale)
    print(f"  {len(ionex.maps)} maps, height={ionex.height}km", file=sys.stderr)
    print(f"  Time: {ionex.maps[0][0]} to {ionex.maps[-1][0]}", file=sys.stderr)

    enabled_systems = set(args.systems.split(','))
    print(f"Loading RAWX: {args.rawx} (systems: {enabled_systems})", file=sys.stderr)
    epochs = load_l1_epochs(args.rawx)

    # Filter to enabled systems
    filtered_epochs = []
    for ts, obs in epochs:
        filt = [o for o in obs if o['sys'] in enabled_systems]
        if len(filt) >= 4:
            filtered_epochs.append((ts, filt))
    epochs = filtered_epochs
    print(f"  {len(epochs)} epochs with ≥4 L1-band SVs", file=sys.stderr)

    if not epochs:
        print("ERROR: No valid epochs", file=sys.stderr)
        sys.exit(1)

    # Count signals in first epoch
    sig_counts = defaultdict(int)
    for obs in epochs[0][1]:
        sig_counts[obs['sys']] += 1
    print(f"  First epoch: {dict(sig_counts)}", file=sys.stderr)

    # Determine which ISBs are needed
    has_gal = 'gal' in enabled_systems
    has_bds = 'bds' in enabled_systems

    known_ecef = None
    if args.known_pos:
        lat, lon, alt = [float(v) for v in args.known_pos.split(',')]
        known_ecef = lla_to_ecef(lat, lon, alt)
        print(f"  Known position: {lat:.6f}, {lon:.6f}, {alt:.1f}m",
              file=sys.stderr)

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

    leap_delta = timedelta(seconds=args.leap)
    ekf = GIMFilter()
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
        t_utc = timestamp_to_gpstime(ts_str)  # UTC for IONEX lookup
        n_total += 1

        if args.no_phase:
            for o in obs:
                o['cp_m'] = None

        if not ekf.initialized:
            x_ls, converged, n_sv = ls_solve_multigps(obs, sp3, t, ionex, t_utc)
            if not converged or n_sv < 6:
                continue
            ekf.initialize(x_ls[:3], x_ls[3])
            ekf.x[IDX_ISB_GAL] = x_ls[4]
            ekf.x[IDX_ISB_BDS] = x_ls[5]
            print(f"  EKF initialized at epoch {n_total} with {n_sv} SVs",
                  file=sys.stderr)
            prev_t = t

            # Initialize ambiguities
            for o in obs:
                if o['cp_m'] is not None and o['half_cyc'] == '1':
                    N_init = o['cp_m'] - o['pr']
                    ekf.add_ambiguity(o['sv'], N_init)
                    ekf.prev_obs[o['sv']] = o

            n_used, _, sys_counts = ekf.update(obs, sp3, t, ionex, t_utc)
            if n_used < 4:
                ekf = GIMFilter()
                continue
        else:
            dt = (t - prev_t).total_seconds()
            ekf.predict(dt)
            prev_t = t

            # Detect cycle slips
            for o in obs:
                sv = o['sv']
                if sv in ekf.prev_obs:
                    if o['lock_ms'] < ekf.prev_obs[sv]['lock_ms']:
                        ekf.remove_ambiguity(sv)
                        if o['cp_m'] is not None and o['half_cyc'] == '1':
                            N_init = o['cp_m'] - o['pr']
                            ekf.add_ambiguity(sv, N_init)

            # Manage ambiguities
            current_svs = {o['sv'] for o in obs}
            tracked = set(ekf.sv_to_idx.keys())
            for sv in tracked - current_svs:
                ekf.remove_ambiguity(sv)
            for o in obs:
                if o['sv'] not in ekf.sv_to_idx:
                    if o['cp_m'] is not None and o['half_cyc'] == '1':
                        N_init = o['cp_m'] - o['pr']
                        ekf.add_ambiguity(o['sv'], N_init)

            n_used, _, sys_counts = ekf.update(obs, sp3, t, ionex, t_utc)
            for o in obs:
                ekf.prev_obs[o['sv']] = o

        # Output
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
            if converge_epoch is None and err_3d < 1.0:
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
    print(f"  Multi-GNSS L1 PPP + GIM ionosphere correction", file=sys.stderr)
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
            print(f"    Convergence to <1.0m at epoch {converge_epoch}",
                  file=sys.stderr)
        else:
            print(f"    Did not converge to <1.0m within {n_total} epochs",
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

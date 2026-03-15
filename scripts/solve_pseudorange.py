#!/usr/bin/env python3
"""
solve_pseudorange.py — GPS pseudorange-only least-squares position solver.

First step toward PPP-AR: validate the observation pipeline by computing
a position fix from testAnt RAWX CSV data + IGS SP3 precise orbits.

Usage:
    python solve_pseudorange.py data/rawx_1h_top_20260303.csv data/igs_rap_062.sp3

Outputs a position time series to stdout and optionally a CSV file.
"""

import argparse
import csv
import gzip
import math
import sys
from collections import defaultdict
from datetime import datetime, timezone, timedelta

import numpy as np

# ── Constants ──────────────────────────────────────────────────────────────── #
C = 299792458.0            # speed of light (m/s)
OMEGA_E = 7.2921151467e-5  # Earth rotation rate (rad/s)
GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
F_L1 = 1575.42e6           # GPS L1 frequency (Hz)
WL_L1 = C / F_L1           # GPS L1 wavelength (m)


# ── SP3 Parser ─────────────────────────────────────────────────────────────── #
class SP3:
    """Parse and interpolate SP3 precise orbit file (GPS satellites)."""

    def __init__(self, path):
        self.epochs = []       # list of datetime
        self.positions = {}    # {sv: [(x,y,z), ...]} in meters
        self.clocks = {}       # {sv: [dt_s, ...]} in seconds
        self._parse(path)
        self._epoch_seconds = np.array([
            (e - self.epochs[0]).total_seconds() for e in self.epochs
        ])

    def _parse(self, path):
        opener = gzip.open if path.endswith('.gz') else open
        with opener(path, 'rt') as f:
            epoch_idx = -1
            for line in f:
                if line.startswith('*'):
                    parts = line.split()
                    y, m, d = int(parts[1]), int(parts[2]), int(parts[3])
                    h, mn = int(parts[4]), int(parts[5])
                    s = float(parts[6])
                    si = int(s)
                    us = int((s - si) * 1e6)
                    epoch = datetime(y, m, d, h, mn, si, us, tzinfo=timezone.utc)
                    self.epochs.append(epoch)
                    epoch_idx += 1
                elif line.startswith('P') and len(line) > 60 and line[1] in 'GECRSJ':
                    sv = line[1:4].strip()
                    x = float(line[4:18]) * 1000   # km → m
                    y = float(line[18:32]) * 1000
                    z = float(line[32:46]) * 1000
                    clk = float(line[46:60]) * 1e-6  # μs → s
                    if sv not in self.positions:
                        self.positions[sv] = []
                        self.clocks[sv] = []
                    self.positions[sv].append((x, y, z))
                    self.clocks[sv].append(clk)

        # Convert to numpy arrays
        for sv in self.positions:
            self.positions[sv] = np.array(self.positions[sv])
            self.clocks[sv] = np.array(self.clocks[sv])

    def sat_position(self, sv, t):
        """Interpolate satellite position at time t (datetime).
        Returns (x, y, z) in ECEF meters, or None if sv not in file.
        """
        if sv not in self.positions:
            return None, None
        t_sec = (t - self.epochs[0]).total_seconds()
        pos = self.positions[sv]
        clk = self.clocks[sv]
        n = len(self.epochs)

        # Find bracketing interval
        idx = np.searchsorted(self._epoch_seconds, t_sec) - 1
        idx = max(0, min(idx, n - 2))

        # 9-point Lagrange interpolation (or fewer near edges)
        half_win = 4
        i0 = max(0, idx - half_win + 1)
        i1 = min(n, i0 + 2 * half_win)
        i0 = max(0, i1 - 2 * half_win)

        ts = self._epoch_seconds[i0:i1]
        xs = pos[i0:i1, 0]
        ys = pos[i0:i1, 1]
        zs = pos[i0:i1, 2]
        cs = clk[i0:i1]

        x_interp = _lagrange(ts, xs, t_sec)
        y_interp = _lagrange(ts, ys, t_sec)
        z_interp = _lagrange(ts, zs, t_sec)
        c_interp = _lagrange(ts, cs, t_sec)

        return np.array([x_interp, y_interp, z_interp]), c_interp


def _lagrange(x_pts, y_pts, x):
    """Lagrange polynomial interpolation."""
    n = len(x_pts)
    result = 0.0
    for i in range(n):
        term = y_pts[i]
        for j in range(n):
            if i != j:
                term *= (x - x_pts[j]) / (x_pts[i] - x_pts[j])
        result += term
    return result


# ── RAWX CSV Reader ────────────────────────────────────────────────────────── #
def load_rawx_epochs(csv_path, signal='GPS-L1CA'):
    """Load RAWX CSV, group by epoch timestamp, filter to one signal.
    Returns list of (timestamp_str, [{sv_id, pseudorange_m, cno_dBHz}, ...])
    """
    epochs = defaultdict(list)
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['signal_id'] != signal:
                continue
            if row['pr_valid'] != '1':
                continue
            try:
                pr = float(row['pseudorange_m'])
                cno = float(row['cno_dBHz'])
            except (ValueError, KeyError):
                continue
            if pr < 1e6 or pr > 4e7:  # sanity check
                continue
            # Map sv_id to SP3 format: "G03"
            sv_num = int(row['sv_id'])
            sv = f"G{sv_num:02d}"
            epochs[row['timestamp']].append({
                'sv': sv,
                'pr': pr,
                'cno': cno,
            })

    # Sort by timestamp and return
    result = []
    for ts in sorted(epochs.keys()):
        obs = epochs[ts]
        if len(obs) >= 4:  # need at least 4 SVs for 3D + clock
            result.append((ts, obs))
    return result


# ── Coordinate Transforms ──────────────────────────────────────────────────── #
def ecef_to_lla(x, y, z):
    """ECEF to geodetic (WGS84). Returns (lat_deg, lon_deg, alt_m)."""
    a = 6378137.0
    f = 1 / 298.257223563
    e2 = 2 * f - f * f
    lon = math.atan2(y, x)
    p = math.sqrt(x * x + y * y)
    lat = math.atan2(z, p * (1 - e2))
    for _ in range(10):
        N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
        lat = math.atan2(z + e2 * N * math.sin(lat), p)
    alt = p / math.cos(lat) - N
    return math.degrees(lat), math.degrees(lon), alt


def lla_to_ecef(lat_deg, lon_deg, alt_m):
    """Geodetic to ECEF (WGS84)."""
    a = 6378137.0
    f = 1 / 298.257223563
    e2 = 2 * f - f * f
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    x = (N + alt_m) * math.cos(lat) * math.cos(lon)
    y = (N + alt_m) * math.cos(lat) * math.sin(lon)
    z = (N * (1 - e2) + alt_m) * math.sin(lat)
    return np.array([x, y, z])


def timestamp_to_gpstime(ts_str):
    """Parse ISO timestamp to GPS time (week, seconds-of-week, datetime)."""
    # Handle timezone offset
    ts = ts_str.replace('+00:00', '+0000').replace('Z', '+0000')
    try:
        dt = datetime.fromisoformat(ts_str)
    except ValueError:
        dt = datetime.strptime(ts[:26], '%Y-%m-%dT%H:%M:%S.%f').replace(
            tzinfo=timezone.utc)
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt


# ── Solver ─────────────────────────────────────────────────────────────────── #
def solve_epoch(obs_list, sp3, t, x0=None):
    """Weighted least-squares pseudorange position solution.

    obs_list: list of {sv, pr, cno}
    sp3: SP3 object
    t: datetime of observation
    x0: initial position estimate [x, y, z, cdt] (ECEF meters + clock*c)

    Returns (position_ecef, clock_bias_m, residuals, n_sv, converged)
    """
    if x0 is None:
        # Initialize near Earth's surface (use average of satellite positions
        # as a rough approximation — better than center of Earth)
        sat_positions = []
        for obs in obs_list:
            sp, sc = sp3.sat_position(obs['sv'], t)
            if sp is not None:
                sat_positions.append(sp)
        if len(sat_positions) >= 4:
            # Use geometric center of satellites projected to Earth surface
            avg = np.mean(sat_positions, axis=0)
            # Scale to Earth radius (~6371 km)
            r = np.linalg.norm(avg)
            x0 = np.zeros(4)
            x0[:3] = avg / r * 6371000.0 if r > 0 else np.array([0, 0, 6371000.0])
        else:
            x0 = np.array([0.0, 0.0, 6371000.0, 0.0])

    x = x0.copy()
    max_iter = 20  # need more iterations from cold start

    for iteration in range(max_iter):
        H = []  # design matrix
        dz = []  # observation - predicted
        W = []  # weights

        for obs in obs_list:
            sat_pos, sat_clk = sp3.sat_position(obs['sv'], t)
            if sat_pos is None or sat_clk is None:
                continue
            if sat_clk > 0.9:  # 999999 = bad clock
                continue

            # Geometric range
            dx = sat_pos - x[:3]
            rho = np.linalg.norm(dx)
            if rho < 1e6:  # too close, bad initial position
                rho = 2e7  # approximate

            # Earth rotation correction during signal transit time
            tau = rho / C
            rot_angle = OMEGA_E * tau
            sat_rot = np.array([
                sat_pos[0] * math.cos(rot_angle) + sat_pos[1] * math.sin(rot_angle),
                -sat_pos[0] * math.sin(rot_angle) + sat_pos[1] * math.cos(rot_angle),
                sat_pos[2]
            ])

            dx = sat_rot - x[:3]
            rho = np.linalg.norm(dx)

            # Predicted pseudorange = geometric range + receiver clock - satellite clock
            pr_pred = rho + x[3] - sat_clk * C

            # Direction cosines
            e = -dx / rho
            H.append([e[0], e[1], e[2], 1.0])
            dz.append(obs['pr'] - pr_pred)

            # Weight by C/N0 (simple: w = 10^(cno/10) / 10^(45/10))
            w = 10 ** ((obs['cno'] - 30) / 20)  # higher C/N0 = higher weight
            W.append(w)

        if len(H) < 4:
            return x, 0, [], len(H), False

        H = np.array(H)
        dz = np.array(dz)
        W = np.diag(W)

        # Weighted least squares: dx = (H^T W H)^{-1} H^T W dz
        try:
            HTW = H.T @ W
            N = HTW @ H
            dx = np.linalg.solve(N, HTW @ dz)
        except np.linalg.LinAlgError:
            return x, 0, dz, len(H), False

        x += dx

        if np.linalg.norm(dx[:3]) < 0.01:  # converged to 1 cm
            residuals = dz - H @ dx
            return x, x[3] / C, residuals, len(H), True

    residuals = dz - H @ np.linalg.solve(H.T @ W @ H, H.T @ W @ dz)
    return x, x[3] / C, residuals, len(H), True


# ── Main ───────────────────────────────────────────────────────────────────── #
def main():
    ap = argparse.ArgumentParser(
        description="GPS pseudorange-only position solver")
    ap.add_argument("rawx", help="RAWX CSV file (testAnt format)")
    ap.add_argument("sp3", help="SP3 precise orbit file")
    ap.add_argument("--signal", default="GPS-L1CA",
                    help="Signal to use (default GPS-L1CA)")
    ap.add_argument("--out", default=None,
                    help="Output CSV file (default: stdout summary)")
    ap.add_argument("--skip", type=int, default=0,
                    help="Skip first N epochs")
    ap.add_argument("--limit", type=int, default=0,
                    help="Process at most N epochs (0=all)")
    ap.add_argument("--known-pos", default=None,
                    help="Known position as lat,lon,alt for error stats")
    ap.add_argument("--leap", type=int, default=18,
                    help="GPS-UTC leap seconds (default 18, current as of 2026)")
    args = ap.parse_args()

    print(f"Loading SP3: {args.sp3}", file=sys.stderr)
    sp3 = SP3(args.sp3)
    print(f"  {len(sp3.epochs)} epochs, {len(sp3.positions)} satellites",
          file=sys.stderr)
    print(f"  Time span: {sp3.epochs[0]} to {sp3.epochs[-1]}", file=sys.stderr)

    print(f"Loading RAWX: {args.rawx} (signal={args.signal})", file=sys.stderr)
    epochs = load_rawx_epochs(args.rawx, signal=args.signal)
    print(f"  {len(epochs)} epochs with ≥4 SVs", file=sys.stderr)

    if not epochs:
        print("ERROR: No valid epochs found", file=sys.stderr)
        sys.exit(1)

    # Known position for error computation
    known_ecef = None
    if args.known_pos:
        lat, lon, alt = [float(x) for x in args.known_pos.split(',')]
        known_ecef = lla_to_ecef(lat, lon, alt)
        print(f"  Known position: {lat:.6f}, {lon:.6f}, {alt:.1f}m",
              file=sys.stderr)

    # Output CSV
    out_f = None
    out_w = None
    if args.out:
        out_f = open(args.out, 'w', newline='')
        out_w = csv.writer(out_f)
        out_w.writerow(['timestamp', 'lat_deg', 'lon_deg', 'alt_m',
                        'x_ecef', 'y_ecef', 'z_ecef',
                        'clock_bias_ns', 'n_sv', 'rms_m',
                        'east_err_m', 'north_err_m', 'up_err_m'])

    # Process epochs
    x_prev = None
    positions = []
    n_converged = 0
    n_total = 0
    errors = []

    start = args.skip
    end = len(epochs) if args.limit == 0 else min(len(epochs), start + args.limit)

    leap_delta = timedelta(seconds=args.leap)

    for i in range(start, end):
        ts_str, obs = epochs[i]
        t = timestamp_to_gpstime(ts_str) + leap_delta  # UTC → GPS time
        n_total += 1

        # Use previous solution as starting point
        x, clk_s, resid, n_sv, converged = solve_epoch(obs, sp3, t, x_prev)

        if not converged or n_sv < 4:
            continue

        n_converged += 1
        x_prev = x.copy()

        lat, lon, alt = ecef_to_lla(x[0], x[1], x[2])
        clk_ns = clk_s * 1e9
        rms = np.sqrt(np.mean(resid ** 2)) if len(resid) > 0 else 0

        # Position error vs known
        e_err = n_err = u_err = 0
        if known_ecef is not None:
            enu = ecef_to_enu(x[:3] - known_ecef, known_ecef)
            e_err, n_err, u_err = enu
            errors.append(enu)

        positions.append((lat, lon, alt))

        if out_w:
            out_w.writerow([ts_str, f'{lat:.7f}', f'{lon:.7f}', f'{alt:.2f}',
                            f'{x[0]:.3f}', f'{x[1]:.3f}', f'{x[2]:.3f}',
                            f'{clk_ns:.1f}', n_sv, f'{rms:.2f}',
                            f'{e_err:.2f}', f'{n_err:.2f}', f'{u_err:.2f}'])

        # Progress every 60 epochs
        if n_total % 60 == 0:
            print(f"  [{n_total}/{end-start}] {ts_str[:19]} "
                  f"lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}m "
                  f"clk={clk_ns:.0f}ns n={n_sv} rms={rms:.1f}m",
                  file=sys.stderr)

    if out_f:
        out_f.close()

    # Summary
    print(f"\n{'='*60}", file=sys.stderr)
    print(f"  Epochs processed: {n_total}", file=sys.stderr)
    print(f"  Converged:        {n_converged}", file=sys.stderr)

    if positions:
        lats = [p[0] for p in positions]
        lons = [p[1] for p in positions]
        alts = [p[2] for p in positions]
        print(f"  Lat:  {np.mean(lats):.7f} ± {np.std(lats)*111000:.1f}m",
              file=sys.stderr)
        print(f"  Lon:  {np.mean(lons):.7f} ± {np.std(lons)*111000*math.cos(math.radians(np.mean(lats))):.1f}m",
              file=sys.stderr)
        print(f"  Alt:  {np.mean(alts):.1f} ± {np.std(alts):.1f}m",
              file=sys.stderr)

    if errors:
        errors = np.array(errors)
        print(f"\n  vs known position:", file=sys.stderr)
        print(f"    East:  {np.mean(errors[:,0]):+.1f} ± {np.std(errors[:,0]):.1f}m",
              file=sys.stderr)
        print(f"    North: {np.mean(errors[:,1]):+.1f} ± {np.std(errors[:,1]):.1f}m",
              file=sys.stderr)
        print(f"    Up:    {np.mean(errors[:,2]):+.1f} ± {np.std(errors[:,2]):.1f}m",
              file=sys.stderr)
        print(f"    3D RMS: {np.sqrt(np.mean(errors**2)):.1f}m",
              file=sys.stderr)

    print(f"{'='*60}", file=sys.stderr)


def ecef_to_enu(dxyz, ref_ecef):
    """Convert ECEF difference vector to local ENU at reference position."""
    lat, lon, _ = ecef_to_lla(ref_ecef[0], ref_ecef[1], ref_ecef[2])
    lat_r = math.radians(lat)
    lon_r = math.radians(lon)

    R = np.array([
        [-math.sin(lon_r), math.cos(lon_r), 0],
        [-math.sin(lat_r)*math.cos(lon_r), -math.sin(lat_r)*math.sin(lon_r), math.cos(lat_r)],
        [math.cos(lat_r)*math.cos(lon_r), math.cos(lat_r)*math.sin(lon_r), math.sin(lat_r)]
    ])
    return R @ dxyz


if __name__ == "__main__":
    main()

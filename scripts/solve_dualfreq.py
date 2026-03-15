#!/usr/bin/env python3
"""
solve_dualfreq.py — Dual-frequency ionosphere-free pseudorange position solver.

Uses GPS L1+L5 ionosphere-free combination to eliminate first-order
ionospheric delay. This is the second step toward PPP-AR: after validating
the observation pipeline with single-frequency, demonstrate that dual-freq
IF combination materially improves position accuracy.

The IF pseudorange combination for two frequencies f1, f2:
    P_IF = (f1² * P1 - f2² * P2) / (f1² - f2²)

This eliminates the ionospheric delay (which is proportional to 1/f²)
at the cost of amplified noise (~3x for L1+L5).

Usage:
    python solve_dualfreq.py data/rawx_1h_top_20260303.csv data/igs_rap_062.sp3
    python solve_dualfreq.py data/rawx_1h_top_20260303.csv data/igs_rap_062.sp3 \
        --known-pos "41.8430626,-88.1037190,201.671" --out data/pos_if.csv
"""

import argparse
import csv
import math
import sys
from collections import defaultdict
from datetime import datetime, timezone, timedelta

import numpy as np

# Import from the single-frequency solver
from solve_pseudorange import (
    SP3, C, OMEGA_E, ecef_to_lla, ecef_to_enu, lla_to_ecef,
    timestamp_to_gpstime,
)

# ── Frequencies ────────────────────────────────────────────────────────────── #
F_L1 = 1575.42e6    # GPS L1 / GAL E1 (Hz) — same frequency
F_L5 = 1176.45e6    # GPS L5 / GAL E5a / BDS B2a (Hz) — same frequency
F_B1I = 1561.098e6  # BDS B1I (Hz) — slightly different from L1

# IF combination coefficients (same for GPS L1+L5 and GAL E1+E5a)
ALPHA_L1 = F_L1**2 / (F_L1**2 - F_L5**2)    # ≈ 2.261
ALPHA_L5 = F_L5**2 / (F_L1**2 - F_L5**2)    # ≈ 1.261
# P_IF = ALPHA_L1 * P_L1 - ALPHA_L5 * P_L5
# Noise amplification factor: sqrt(ALPHA_L1² + ALPHA_L5²) ≈ 2.59

# BDS B1I+B2a IF coefficients (different from GPS/GAL because B1I ≠ L1)
ALPHA_B1I = F_B1I**2 / (F_B1I**2 - F_L5**2)    # ≈ 2.332
ALPHA_B2A = F_L5**2 / (F_B1I**2 - F_L5**2)     # ≈ 1.332

# Signal pairs for IF combination: (gnss_id, sig_f1, sig_f2, sv_prefix, alpha_f1, alpha_f2)
IF_PAIRS = [
    ('GPS', 'GPS-L1CA', 'GPS-L5Q', 'G', ALPHA_L1, ALPHA_L5),
    ('GAL', 'GAL-E1C', 'GAL-E5aQ', 'E', ALPHA_L1, ALPHA_L5),
    ('BDS', 'BDS-B1I', 'BDS-B2aI', 'C', ALPHA_B1I, ALPHA_B2A),
]


# ── Load dual-frequency epochs ────────────────────────────────────────────── #
def load_dualfreq_epochs(csv_path):
    """Load RAWX CSV and form GPS L1+L5 ionosphere-free pseudoranges.

    For each epoch, matches L1CA and L5Q observations by SV, computes
    the IF combination, and also retains carrier phase for future use.

    Returns list of (timestamp_str, [{sv, pr_if, pr_l1, pr_l5,
                                       cp_l1, cp_l5, cno_l1, cno_l5}, ...])
    """
    # Build lookup: signal_id → (gnss_id, sv_prefix, role, alpha_f1, alpha_f2)
    sig_lookup = {}
    for gnss_id, sig_f1, sig_f2, prefix, a1, a2 in IF_PAIRS:
        sig_lookup[sig_f1] = (gnss_id, prefix, 'f1', a1, a2)
        sig_lookup[sig_f2] = (gnss_id, prefix, 'f2', a1, a2)

    # First pass: group by (timestamp, sv)
    raw = defaultdict(lambda: defaultdict(dict))
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['pr_valid'] != '1':
                continue
            sig = row['signal_id']
            if sig not in sig_lookup:
                continue
            gnss_id, prefix, role, a1, a2 = sig_lookup[sig]
            ts = row['timestamp']
            sv_num = int(row['sv_id'])
            sv = f"{prefix}{sv_num:02d}"
            try:
                pr = float(row['pseudorange_m'])
                cno = float(row['cno_dBHz'])
                cp = float(row['carrier_phase_cy']) if row.get('cp_valid') == '1' else None
            except (ValueError, KeyError):
                continue
            if pr < 1e6 or pr > 4e7:
                continue
            raw[ts][sv][role] = {'pr': pr, 'cno': cno, 'cp': cp,
                                  'alpha_f1': a1, 'alpha_f2': a2}

    # Second pass: form IF combinations
    result = []
    for ts in sorted(raw.keys()):
        obs = []
        for sv, roles in raw[ts].items():
            if 'f1' not in roles or 'f2' not in roles:
                continue
            f1 = roles['f1']
            f2 = roles['f2']
            pr_if = f1['alpha_f1'] * f1['pr'] - f1['alpha_f2'] * f2['pr']
            obs.append({
                'sv': sv,
                'pr_if': pr_if,
                'pr_l1': f1['pr'],
                'pr_l5': f2['pr'],
                'cp_l1': f1['cp'],
                'cp_l5': f2['cp'],
                'cno_l1': f1['cno'],
                'cno_l5': f2['cno'],
                'cno': min(f1['cno'], f2['cno']),
            })
        if len(obs) >= 4:
            result.append((ts, obs))
    return result


# ── Solver (same as single-freq but uses pr_if) ───────────────────────────── #
def solve_epoch(obs_list, sp3, t, x0=None):
    """Weighted least-squares IF pseudorange position solution."""
    if x0 is None:
        sat_positions = []
        for obs in obs_list:
            sp, sc = sp3.sat_position(obs['sv'], t)
            if sp is not None:
                sat_positions.append(sp)
        if len(sat_positions) >= 4:
            avg = np.mean(sat_positions, axis=0)
            r = np.linalg.norm(avg)
            x0 = np.zeros(4)
            x0[:3] = avg / r * 6371000.0 if r > 0 else np.array([0, 0, 6371000.0])
        else:
            x0 = np.array([0.0, 0.0, 6371000.0, 0.0])

    x = x0.copy()

    for iteration in range(20):
        H, dz, W = [], [], []

        for obs in obs_list:
            sat_pos, sat_clk = sp3.sat_position(obs['sv'], t)
            if sat_pos is None or sat_clk is None:
                continue
            if sat_clk > 0.9:
                continue

            dx = sat_pos - x[:3]
            rho = np.linalg.norm(dx)
            if rho < 1e6:
                rho = 2e7

            # Earth rotation correction
            tau = rho / C
            rot = OMEGA_E * tau
            sat_rot = np.array([
                sat_pos[0] * math.cos(rot) + sat_pos[1] * math.sin(rot),
                -sat_pos[0] * math.sin(rot) + sat_pos[1] * math.cos(rot),
                sat_pos[2]
            ])
            dx = sat_rot - x[:3]
            rho = np.linalg.norm(dx)

            pr_pred = rho + x[3] - sat_clk * C
            e = -dx / rho
            H.append([e[0], e[1], e[2], 1.0])
            dz.append(obs['pr_if'] - pr_pred)
            w = 10 ** ((obs['cno'] - 30) / 20)
            W.append(w)

        if len(H) < 4:
            return x, 0, [], len(H), False

        H = np.array(H)
        dz = np.array(dz)
        W_mat = np.diag(W)

        try:
            HTW = H.T @ W_mat
            delta = np.linalg.solve(HTW @ H, HTW @ dz)
        except np.linalg.LinAlgError:
            return x, 0, dz, len(H), False

        x += delta

        if np.linalg.norm(delta[:3]) < 0.01:
            residuals = dz - H @ delta
            return x, x[3] / C, residuals, len(H), True

    return x, x[3] / C, dz, len(H), True


# ── Main ───────────────────────────────────────────────────────────────────── #
def main():
    ap = argparse.ArgumentParser(
        description="Dual-frequency ionosphere-free pseudorange solver")
    ap.add_argument("rawx", help="RAWX CSV file (testAnt format)")
    ap.add_argument("sp3", help="SP3 precise orbit file")
    ap.add_argument("--out", default=None, help="Output CSV file")
    ap.add_argument("--skip", type=int, default=0)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--known-pos", default=None,
                    help="Known position as lat,lon,alt for error stats")
    ap.add_argument("--leap", type=int, default=18,
                    help="GPS-UTC leap seconds (default 18)")
    args = ap.parse_args()

    print(f"Loading SP3: {args.sp3}", file=sys.stderr)
    sp3 = SP3(args.sp3)
    print(f"  {len(sp3.epochs)} epochs, {len(sp3.positions)} satellites",
          file=sys.stderr)

    print(f"Loading RAWX: {args.rawx} (GPS L1+L5 IF)", file=sys.stderr)
    epochs = load_dualfreq_epochs(args.rawx)
    print(f"  {len(epochs)} epochs with ≥4 dual-freq SVs", file=sys.stderr)

    if not epochs:
        print("ERROR: No valid dual-frequency epochs", file=sys.stderr)
        sys.exit(1)

    known_ecef = None
    if args.known_pos:
        lat, lon, alt = [float(x) for x in args.known_pos.split(',')]
        known_ecef = lla_to_ecef(lat, lon, alt)
        print(f"  Known position: {lat:.6f}, {lon:.6f}, {alt:.1f}m",
              file=sys.stderr)

    out_f = None
    out_w = None
    if args.out:
        out_f = open(args.out, 'w', newline='')
        out_w = csv.writer(out_f)
        out_w.writerow(['timestamp', 'lat_deg', 'lon_deg', 'alt_m',
                        'clock_bias_ns', 'n_sv', 'rms_m',
                        'east_err_m', 'north_err_m', 'up_err_m'])

    leap_delta = timedelta(seconds=args.leap)
    x_prev = None
    positions = []
    errors = []
    n_total = 0
    n_converged = 0

    start = args.skip
    end = len(epochs) if args.limit == 0 else min(len(epochs), start + args.limit)

    for i in range(start, end):
        ts_str, obs = epochs[i]
        t = timestamp_to_gpstime(ts_str) + leap_delta
        n_total += 1

        x, clk_s, resid, n_sv, converged = solve_epoch(obs, sp3, t, x_prev)

        if not converged or n_sv < 4:
            continue

        n_converged += 1
        x_prev = x.copy()

        lat, lon, alt = ecef_to_lla(x[0], x[1], x[2])
        clk_ns = clk_s * 1e9
        rms = np.sqrt(np.mean(resid ** 2)) if len(resid) > 0 else 0

        e_err = n_err = u_err = 0
        if known_ecef is not None:
            enu = ecef_to_enu(x[:3] - known_ecef, known_ecef)
            e_err, n_err, u_err = enu
            errors.append(enu)

        positions.append((lat, lon, alt))

        if out_w:
            out_w.writerow([ts_str, f'{lat:.7f}', f'{lon:.7f}', f'{alt:.2f}',
                            f'{clk_ns:.1f}', n_sv, f'{rms:.2f}',
                            f'{e_err:.2f}', f'{n_err:.2f}', f'{u_err:.2f}'])

        if n_total % 60 == 0:
            print(f"  [{n_total}/{end-start}] {ts_str[:19]} "
                  f"lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}m "
                  f"n={n_sv} rms={rms:.1f}m",
                  file=sys.stderr)

    if out_f:
        out_f.close()

    # Summary
    print(f"\n{'='*60}", file=sys.stderr)
    print(f"  Dual-frequency IF (GPS L1+L5)", file=sys.stderr)
    print(f"  IF coefficients: α_L1={ALPHA_L1:.3f}, α_L5={ALPHA_L5:.3f}",
          file=sys.stderr)
    print(f"  Noise amplification: {math.sqrt(ALPHA_L1**2 + ALPHA_L5**2):.2f}x",
          file=sys.stderr)
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


if __name__ == "__main__":
    main()

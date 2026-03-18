#!/usr/bin/env python3
"""peppar-find-position: Bootstrap position using PPP.

Uses PPPFilter (full position+clock+ISB+ambiguity EKF) to converge position
from scratch. No PHC or PTP hardware needed — just serial + NTRIP.

Exit codes:
    0 = converged (position printed / saved)
    1 = timeout (did not converge within --timeout)
    2 = no signals (no observations received)
    3 = error
"""

import argparse
import csv
import json
import logging
import math
import os
import queue
import signal
import sys
import threading
import time
from datetime import datetime, timezone, timedelta

import numpy as np

from solve_pseudorange import C, OMEGA_E, ecef_to_lla, lla_to_ecef
from solve_ppp import PPPFilter, IDX_CLK, IDX_ISB_GAL, IDX_ISB_BDS, N_BASE, ls_init
from broadcast_eph import BroadcastEphemeris
from ssr_corrections import SSRState, RealtimeCorrections
from ntrip_client import NtripStream
from realtime_ppp import serial_reader, ntrip_reader
from peppar_fix import save_position, load_position

log = logging.getLogger("find_position")

EXIT_CONVERGED = 0
EXIT_TIMEOUT = 1
EXIT_NO_SIGNALS = 2
EXIT_ERROR = 3


# ── Convergence detection ────────────────────────────────────────────────── #

def position_sigma_3d(P):
    """Compute 3D position sigma from EKF covariance matrix."""
    P_pos = P[:3, :3]
    return math.sqrt(P_pos[0, 0] + P_pos[1, 1] + P_pos[2, 2])


# ── Main loop ────────────────────────────────────────────────────────────── #

def run_find_position(args):
    """Run position bootstrap loop."""

    # Shared state
    beph = BroadcastEphemeris()
    ssr = SSRState()
    corrections = RealtimeCorrections(beph, ssr)
    obs_queue = queue.Queue(maxsize=100)
    stop_event = threading.Event()

    # Handle SIGTERM gracefully
    def on_signal(signum, frame):
        log.info("Signal received, shutting down")
        stop_event.set()
    signal.signal(signal.SIGTERM, on_signal)

    # Start NTRIP threads
    use_tls = getattr(args, 'tls', False) or args.port == 443

    if args.eph_mount:
        eph_stream = NtripStream(
            caster=args.caster, port=args.port,
            mountpoint=args.eph_mount,
            user=args.user, password=args.password,
            tls=use_tls,
        )
        t_eph = threading.Thread(
            target=ntrip_reader,
            args=(eph_stream, beph, ssr, stop_event, "EPH"),
            daemon=True,
        )
        t_eph.start()
        log.info(f"Ephemeris stream: {args.caster}:{args.port}/{args.eph_mount}")

    if args.ssr_mount:
        ssr_stream = NtripStream(
            caster=args.caster, port=args.port,
            mountpoint=args.ssr_mount,
            user=args.user, password=args.password,
            tls=use_tls,
        )
        t_ssr = threading.Thread(
            target=ntrip_reader,
            args=(ssr_stream, beph, ssr, stop_event, "SSR"),
            daemon=True,
        )
        t_ssr.start()
        log.info(f"SSR stream: {args.caster}:{args.port}/{args.ssr_mount}")

    # Wait for initial ephemeris
    if args.eph_mount:
        log.info("Waiting for broadcast ephemeris...")
        warmup_start = time.time()
        while beph.n_satellites < 8 and time.time() - warmup_start < 120:
            if stop_event.is_set():
                return EXIT_ERROR
            time.sleep(1)
            if int(time.time() - warmup_start) % 10 == 0:
                log.info(f"  Warmup: {beph.summary()}")
        log.info(f"Warmup complete: {beph.summary()}")

    # Parse systems filter
    systems = set(args.systems.split(',')) if args.systems else None
    log.info(f"Systems: {systems}")

    # Start serial reader
    t_serial = threading.Thread(
        target=serial_reader,
        args=(args.serial, args.baud, obs_queue, stop_event, beph, systems, ssr),
        daemon=True,
    )
    t_serial.start()
    log.info(f"Serial: {args.serial} at {args.baud} baud")

    # Seed position
    seed_ecef = None
    if args.seed_pos:
        lat, lon, alt = [float(v) for v in args.seed_pos.split(',')]
        seed_ecef = lla_to_ecef(lat, lon, alt)
        log.info(f"Seed position: {lat:.6f}, {lon:.6f}, {alt:.1f}m")

    # Initialize filter
    filt = PPPFilter()
    filt_initialized = False

    # CSV output
    out_f = None
    out_w = None
    if args.out:
        out_f = open(args.out, 'w', newline='')
        out_w = csv.writer(out_f)
        out_w.writerow(['timestamp', 'lat', 'lon', 'alt_m',
                        'sigma_3d_m', 'n_meas', 'rms_m', 'n_ambiguities'])

    # Main loop
    prev_t = None
    prev_pos_ecef = None
    n_epochs = 0
    n_empty = 0
    converged = False
    converged_at = None
    start_time = time.time()

    try:
        while not stop_event.is_set():
            elapsed = time.time() - start_time

            # Timeout check
            if args.timeout and elapsed > args.timeout:
                log.warning(f"Timeout after {elapsed:.0f}s without convergence")
                break

            try:
                gps_time, observations = obs_queue.get(timeout=5)
            except queue.Empty:
                n_empty += 1
                if n_empty > 12:  # 60s with no observations
                    log.error("No observations received for 60s")
                    stop_event.set()
                    break
                continue
            n_empty = 0

            # Initialize filter on first epoch with enough satellites
            if not filt_initialized:
                if seed_ecef is not None:
                    init_pos = seed_ecef
                    init_clk = 0.0
                else:
                    # Use LS solve for initial position
                    x_ls, ok, n_sv = ls_init(observations, corrections, gps_time,
                                              clk_file=corrections)
                    if not ok or n_sv < 4:
                        log.info(f"Waiting for enough satellites (got {n_sv})")
                        continue
                    init_pos = x_ls[:3]
                    init_clk = x_ls[3]
                    log.info(f"LS init: {n_sv} SVs, pos error ~km-level")

                filt.initialize(init_pos, init_clk)
                filt_initialized = True
                prev_t = gps_time
                log.info("PPPFilter initialized, starting convergence")
                continue

            # EKF predict
            dt = (gps_time - prev_t).total_seconds()
            if dt <= 0 or dt > 30:
                log.warning(f"Suspicious dt={dt:.1f}s, skipping")
                prev_t = gps_time
                continue
            filt.predict(dt)
            prev_t = gps_time

            # Manage ambiguities — add new SVs, remove cycle-slipped
            current_svs = {o['sv'] for o in observations}
            if filt.prev_obs:
                slipped = filt.detect_cycle_slips(observations, filt.prev_obs)
                for sv in slipped:
                    filt.remove_ambiguity(sv)
                    log.debug(f"Cycle slip: removed {sv}")

            for obs in observations:
                sv = obs['sv']
                if sv not in filt.sv_to_idx and obs.get('phi_if_m') is not None:
                    # Initialize ambiguity from pseudorange - carrier difference
                    sat_pos, sat_clk = corrections.sat_position(sv, gps_time)
                    if sat_pos is not None:
                        dx = sat_pos - filt.x[:3]
                        rho = np.linalg.norm(dx)
                        N_init = obs['phi_if_m'] - obs['pr_if']
                        filt.add_ambiguity(sv, N_init)

            # Store for cycle slip detection
            filt.prev_obs = {o['sv']: o for o in observations}

            # Remove ambiguities for satellites no longer visible
            for sv in list(filt.sv_to_idx.keys()):
                if sv not in current_svs:
                    filt.remove_ambiguity(sv)

            # EKF update
            n_used, resid, sys_counts = filt.update(
                observations, corrections, gps_time, clk_file=corrections)

            if n_used < 4:
                continue

            n_epochs += 1

            # Extract position and sigma
            pos_ecef = filt.x[:3]
            sigma_3d = position_sigma_3d(filt.P)
            lat, lon, alt = ecef_to_lla(pos_ecef[0], pos_ecef[1], pos_ecef[2])

            # CSV output
            rms = np.sqrt(np.mean(resid ** 2)) if len(resid) > 0 else 0
            if out_w:
                out_w.writerow([
                    gps_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:23],
                    f'{lat:.7f}', f'{lon:.7f}', f'{alt:.3f}',
                    f'{sigma_3d:.4f}', n_used, f'{rms:.4f}',
                    len(filt.sv_to_idx),
                ])

            # Status every 5 epochs
            if n_epochs % 5 == 0:
                log.info(
                    f"  [{n_epochs}] σ={sigma_3d:.3f}m "
                    f"pos=({lat:.6f}, {lon:.6f}, {alt:.1f}) "
                    f"n={n_used} amb={len(filt.sv_to_idx)} "
                    f"rms={rms:.3f}m [{elapsed:.0f}s]"
                )

            # Convergence check: sigma below threshold AND position stable
            # Position stability: 3D movement < sigma between epochs
            pos_stable = True
            if prev_pos_ecef is not None:
                pos_delta = np.linalg.norm(pos_ecef - prev_pos_ecef)
                pos_stable = pos_delta < args.sigma
            prev_pos_ecef = pos_ecef.copy()

            if sigma_3d < args.sigma and pos_stable and rms < 10.0:
                if converged_at is None:
                    converged_at = n_epochs
                # Require convergence to hold for 30 consecutive epochs
                if n_epochs - converged_at >= 30:
                    converged = True
                    log.info(f"CONVERGED at epoch {n_epochs} "
                             f"(σ={sigma_3d:.4f}m, rms={rms:.3f}m)")
                    break
            else:
                converged_at = None

    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        stop_event.set()
        if out_f:
            out_f.close()

    if n_epochs == 0:
        log.error("No valid epochs processed")
        return EXIT_NO_SIGNALS

    # Final position
    pos_ecef = filt.x[:3]
    sigma_3d = position_sigma_3d(filt.P)
    lat, lon, alt = ecef_to_lla(pos_ecef[0], pos_ecef[1], pos_ecef[2])
    elapsed = time.time() - start_time

    # Print result to stdout (machine-readable)
    result = {
        "lat": round(lat, 7),
        "lon": round(lon, 7),
        "alt_m": round(alt, 3),
        "ecef_m": [round(float(pos_ecef[0]), 3),
                    round(float(pos_ecef[1]), 3),
                    round(float(pos_ecef[2]), 3)],
        "sigma_m": round(float(sigma_3d), 4),
        "epochs": n_epochs,
        "elapsed_s": round(elapsed, 1),
        "converged": converged,
    }
    print(json.dumps(result, indent=2))

    # Save if requested
    if args.save and converged:
        save_path = args.position_file
        save_position(save_path, pos_ecef, sigma_3d, "ppp_bootstrap",
                       note=f"Converged in {n_epochs} epochs ({elapsed:.0f}s)")
        log.info(f"Position saved to {save_path}")

    if converged:
        log.info(f"Position: {lat:.7f}, {lon:.7f}, {alt:.3f}m "
                 f"(σ={sigma_3d:.4f}m, {n_epochs} epochs, {elapsed:.0f}s)")
        return EXIT_CONVERGED
    else:
        log.warning(f"Did not converge: σ={sigma_3d:.3f}m > {args.sigma}m "
                     f"after {n_epochs} epochs ({elapsed:.0f}s)")
        return EXIT_TIMEOUT


# ── CLI ──────────────────────────────────────────────────────────────────── #

def main():
    ap = argparse.ArgumentParser(
        description="Bootstrap GNSS position using PPP (no PHC required)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exit codes:
  0  Converged — position printed to stdout (and saved if --save)
  1  Timeout — did not converge within --timeout
  2  No signals — no GNSS observations received
  3  Error
""",
    )

    # Position
    ap.add_argument("--seed-pos",
                    help="Seed position as lat,lon,alt (optional, speeds convergence)")
    ap.add_argument("--sigma", type=float, default=0.1,
                    help="Convergence threshold in meters (default: 0.1)")
    ap.add_argument("--timeout", type=int, default=3600,
                    help="Timeout in seconds (default: 3600)")
    ap.add_argument("--systems", default="gps,gal,bds",
                    help="GNSS systems (default: gps,gal,bds)")
    ap.add_argument("--leap", type=int, default=18,
                    help="UTC-GPS leap seconds (default: 18)")

    # Output
    ap.add_argument("--save", action="store_true",
                    help="Save converged position to position file")
    ap.add_argument("--position-file", default="data/position.json",
                    help="Position file path (default: data/position.json)")
    ap.add_argument("--out", help="CSV log file for convergence tracking")

    # Serial
    serial = ap.add_argument_group("Serial")
    serial.add_argument("--serial", required=True,
                        help="Serial port for F9T (e.g. /dev/gnss-bot)")
    serial.add_argument("--baud", type=int, default=115200)

    # NTRIP
    ntrip = ap.add_argument_group("NTRIP corrections")
    ntrip.add_argument("--ntrip-conf", help="NTRIP config file (INI format)")
    ntrip.add_argument("--caster", help="NTRIP caster hostname")
    ntrip.add_argument("--port", type=int, default=2101)
    ntrip.add_argument("--tls", action="store_true")
    ntrip.add_argument("--eph-mount", help="Broadcast ephemeris mountpoint")
    ntrip.add_argument("--ssr-mount", help="SSR corrections mountpoint")
    ntrip.add_argument("--user", help="NTRIP username")
    ntrip.add_argument("--password", help="NTRIP password")

    ap.add_argument("-v", "--verbose", action="store_true")

    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        stream=sys.stderr,
    )

    # Load NTRIP config file if specified
    if args.ntrip_conf:
        import configparser
        conf = configparser.ConfigParser()
        conf.read(args.ntrip_conf)
        if 'ntrip' in conf:
            s = conf['ntrip']
            if not args.caster:
                args.caster = s.get('caster', args.caster)
            if args.port == 2101 and s.get('port'):
                args.port = int(s.get('port'))
            if not args.user:
                args.user = s.get('user', args.user)
            if not args.password:
                args.password = s.get('password', args.password)
            if not args.tls and s.getboolean('tls', False):
                args.tls = True
            if not args.ssr_mount and s.get('mount'):
                args.ssr_mount = s.get('mount')

    if not args.caster and not args.eph_mount:
        log.warning("No NTRIP source — will use broadcast ephemeris from receiver only")

    sys.exit(run_find_position(args))


if __name__ == "__main__":
    main()

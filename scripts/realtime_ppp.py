#!/usr/bin/env python3
"""
realtime_ppp.py — Real-time PPP clock estimation from live GNSS observations.

Milestone 4: Combines live F9T RXM-RAWX observations with real-time SSR
corrections (via NTRIP) to produce continuous sub-ns clock estimates.

Architecture:
    F9T serial ──→ UBX parser ──→ RXM-RAWX → IF observations
                                  RXM-SFRBX → broadcast ephemeris
    NTRIP caster ──→ RTCM3 ──→ SSR orbit/clock/bias corrections
                               broadcast ephemeris (1019/1045/1042)
                     ↓                    ↓
              FixedPosFilter ←────────────┘
                     ↓
              Clock estimate (ns) → CSV + console

Usage:
    python realtime_ppp.py --serial /dev/gnss-bot --baud 115200 \\
        --known-pos "41.8430626,-88.1037190,201.671" \\
        --caster products.igs-ip.net --port 2101 \\
        --eph-mount BCEP00BKG0 --ssr-mount SSRA00CNE0 \\
        --user myuser --password mypass \\
        --duration 3600 --out data/realtime_test.csv

    # Ephemeris-only mode (no SSR, broadcast-quality corrections):
    python realtime_ppp.py --serial /dev/gnss-bot --baud 115200 \\
        --known-pos "41.8430626,-88.1037190,201.671" \\
        --caster products.igs-ip.net --port 2101 \\
        --eph-mount BCEP00BKG0 \\
        --duration 3600

    # File-based replay mode (for development/testing):
    python realtime_ppp.py --replay data/rawx_1h_top_20260303.csv \\
        --sp3 data/gfz_mgx_062.sp3 --clk data/GFZ0MGXRAP_062_30S.CLK \\
        --osb data/GFZ0MGXRAP_062_OSB.BIA \\
        --known-pos "41.8430626,-88.1037190,201.671"
"""

import argparse
import csv
import logging
import math
import os
import queue
import sys
import threading
import time
from collections import defaultdict
from datetime import datetime, timezone, timedelta

import numpy as np

# Project imports
from solve_pseudorange import (
    SP3, C, OMEGA_E, lla_to_ecef, timestamp_to_gpstime,
)
from solve_dualfreq import IF_PAIRS
from solve_ppp import (
    FixedPosFilter, SIG_TO_RINEX, IF_WL, ELEV_MASK,
    SIGMA_P_IF, SIGMA_PHI_IF, BDS_MIN_PRN,
    load_ppp_epochs,
)
from ppp_corrections import OSBParser, CLKFile
from broadcast_eph import BroadcastEphemeris
from ssr_corrections import SSRState, RealtimeCorrections
from ntrip_client import NtripStream

log = logging.getLogger(__name__)

# Broadcast ephemeris RTCM message types
EPH_MSG_TYPES = {'1019', '1042', '1045', '1046'}

# SSR message types (IGS SSR 4076 + standard RTCM)
SSR_MSG_TYPES = set()
for _sub in range(21, 28):
    SSR_MSG_TYPES.add(f'4076_{_sub:03d}')
for _sub in range(61, 68):
    SSR_MSG_TYPES.add(f'4076_{_sub:03d}')
for _sub in range(101, 108):
    SSR_MSG_TYPES.add(f'4076_{_sub:03d}')
for _mt in range(1057, 1069):
    SSR_MSG_TYPES.add(str(_mt))
for _mt in range(1240, 1264):
    SSR_MSG_TYPES.add(str(_mt))


# ── Serial observation reader ──────────────────────────────────────────────── #

def serial_reader(port, baud, obs_queue, stop_event, beph, systems=None,
                   ssr=None):
    """Read UBX messages from F9T serial port.

    Puts (timestamp, observations_list) tuples onto obs_queue for each
    RXM-RAWX epoch. Also feeds RXM-SFRBX to broadcast ephemeris.

    Args:
        systems: set of system names to include (e.g. {'gps', 'gal', 'bds'}).
                 None means all systems.
        ssr: SSRState instance for real-time code bias corrections.
             If provided, biases are applied to raw pseudoranges before
             IF combination (same as OSB in the file-based pipeline).
    """
    try:
        from pyubx2 import UBXReader
        import serial as pyserial
    except ImportError:
        log.error("pyubx2/pyserial not installed")
        stop_event.set()
        return

    log.info(f"Opening serial {port} at {baud} baud")
    ser = pyserial.Serial(port, baud, timeout=2)
    ubr = UBXReader(ser, protfilter=2)  # UBX only

    # Signal name mapping (same as log_observations.py)
    SIG_NAMES = {
        (0, 0): 'GPS-L1CA', (0, 3): 'GPS-L2CL', (0, 4): 'GPS-L2CM',
        (0, 6): 'GPS-L5I', (0, 7): 'GPS-L5Q',
        (2, 0): 'GAL-E1C', (2, 1): 'GAL-E1B',
        (2, 5): 'GAL-E5aI', (2, 6): 'GAL-E5aQ',
        (2, 2): 'GAL-E5bI', (2, 3): 'GAL-E5bQ',
        (3, 0): 'BDS-B1I', (3, 1): 'BDS-B1C',
        (3, 5): 'BDS-B2aI', (3, 7): 'BDS-B2I',
    }
    SYS_MAP = {0: 'G', 2: 'E', 3: 'C'}

    sig_lookup = {}
    for gnss_id, sig_f1, sig_f2, prefix, a1, a2 in IF_PAIRS:
        sig_lookup[sig_f1] = (gnss_id, prefix, 'f1', a1, a2, sig_f1)
        sig_lookup[sig_f2] = (gnss_id, prefix, 'f2', a1, a2, sig_f2)

    epoch_data = {}   # sv → {f1: {...}, f2: {...}}
    epoch_ts = None
    n_epochs = 0

    while not stop_event.is_set():
        try:
            raw, parsed = ubr.read()
            if parsed is None:
                continue

            msg_id = parsed.identity

            # Broadcast ephemeris from SFRBX
            # (We'll rely on NTRIP for ephemeris; SFRBX decoding is complex)

            if msg_id == 'RXM-RAWX':
                # New RAWX epoch — process and enqueue
                ts = datetime.now(timezone.utc)  # Use wall clock for now
                rcvTow = parsed.rcvTow
                week = parsed.week
                leapS = parsed.leapS
                numMeas = parsed.numMeas

                # Build observation set
                raw_obs = defaultdict(dict)  # sv → role → data
                for i in range(1, numMeas + 1):
                    i2 = f"{i:02d}"
                    gnss_id = getattr(parsed, f'gnssId_{i2}', None)
                    sig_id = getattr(parsed, f'sigId_{i2}', None)
                    sv_id = getattr(parsed, f'svId_{i2}', None)
                    if gnss_id is None or sig_id is None:
                        continue

                    sig_name = SIG_NAMES.get((gnss_id, sig_id))
                    if sig_name is None or sig_name not in sig_lookup:
                        continue

                    _, prefix, role, a1, a2, _ = sig_lookup[sig_name]
                    sv = f"{prefix}{int(sv_id):02d}"

                    # BDS-2 GEO/IGSO exclusion
                    if prefix == 'C' and int(sv_id) < BDS_MIN_PRN:
                        continue

                    pr = getattr(parsed, f'prMes_{i2}', None)
                    cp = getattr(parsed, f'cpMes_{i2}', None)
                    cno = getattr(parsed, f'cno_{i2}', None)
                    lock_ms = getattr(parsed, f'locktime_{i2}', 0)
                    pr_valid = getattr(parsed, f'prValid_{i2}', 0)
                    cp_valid = getattr(parsed, f'cpValid_{i2}', 0)
                    half_cyc = getattr(parsed, f'halfCyc_{i2}', 0)

                    if not pr_valid or pr is None:
                        continue
                    if pr < 1e6 or pr > 4e7:
                        continue

                    raw_obs[sv][role] = {
                        'pr': pr, 'cno': cno,
                        'cp': cp if cp_valid else None,
                        'lock_ms': lock_ms or 0.0,
                        'half_cyc': half_cyc,
                        'alpha_f1': a1, 'alpha_f2': a2,
                        'sig_name': sig_name,
                    }

                # Form IF observations
                observations = []
                PREFIX_TO_SYS = {'G': 'gps', 'E': 'gal', 'C': 'bds'}
                for sv, roles in raw_obs.items():
                    prefix = sv[0]
                    sys_name = PREFIX_TO_SYS.get(prefix)

                    # System filter
                    if systems and sys_name not in systems:
                        continue

                    if 'f1' not in roles or 'f2' not in roles:
                        continue
                    f1 = roles['f1']
                    f2 = roles['f2']
                    if f1['cp'] is None or f2['cp'] is None:
                        continue
                    # Half-cycle ambiguity check
                    if not f1['half_cyc'] or not f2['half_cyc']:
                        continue

                    a1 = f1['alpha_f1']
                    a2 = f1['alpha_f2']

                    pr_f1 = f1['pr']
                    pr_f2 = f2['pr']
                    cp_f1 = f1['cp']
                    cp_f2 = f2['cp']

                    # Apply SSR code biases before IF combination
                    if ssr is not None:
                        rinex_f1 = SIG_TO_RINEX.get(f1['sig_name'])
                        rinex_f2 = SIG_TO_RINEX.get(f2['sig_name'])
                        if rinex_f1 and rinex_f2:
                            cb_f1 = ssr.get_code_bias(sv, rinex_f1[0])
                            cb_f2 = ssr.get_code_bias(sv, rinex_f2[0])
                            if cb_f1 is not None and cb_f2 is not None:
                                pr_f1 -= cb_f1
                                pr_f2 -= cb_f2
                                if n_epochs < 2:
                                    log.info(f"    BIAS {sv}: {rinex_f1[0]}={cb_f1:.3f}m "
                                             f"{rinex_f2[0]}={cb_f2:.3f}m "
                                             f"IF_delta={(a1*cb_f1 - a2*cb_f2):.3f}m")

                    pr_if = a1 * pr_f1 - a2 * pr_f2
                    wl_f1, wl_f2, _, _ = IF_WL[prefix]
                    phi_if_m = a1 * wl_f1 * cp_f1 - a2 * wl_f2 * cp_f2

                    observations.append({
                        'sv': sv,
                        'sys': sys_name,
                        'pr_if': pr_if,
                        'phi_if_m': phi_if_m,
                        'cno': min(f1['cno'], f2['cno']),
                        'lock_duration_ms': min(f1['lock_ms'], f2['lock_ms']),
                        'half_cyc_ok': True,
                    })

                # Diagnostic dump (first 3 epochs, then every 60)
                if n_epochs < 3 or n_epochs % 60 == 0:
                    log.info(f"Serial diag epoch {n_epochs}: "
                             f"raw_obs={len(raw_obs)} SVs, "
                             f"IF_obs={len(observations)} SVs, "
                             f"systems_filter={systems}")
                    for sv, roles in sorted(raw_obs.items()):
                        prefix = sv[0]
                        sys_name = PREFIX_TO_SYS.get(prefix, '?')
                        filtered = systems and sys_name not in systems
                        has_dual = 'f1' in roles and 'f2' in roles
                        f1_pr = roles.get('f1', {}).get('pr', 0)
                        f2_pr = roles.get('f2', {}).get('pr', 0)
                        f1_sig = roles.get('f1', {}).get('sig_name', '?')
                        f2_sig = roles.get('f2', {}).get('sig_name', '?')
                        log.info(f"  {sv} sys={sys_name} "
                                 f"{'FILTERED' if filtered else 'PASS'} "
                                 f"dual={'Y' if has_dual else 'N'} "
                                 f"f1={f1_sig}:{f1_pr:.1f} "
                                 f"f2={f2_sig}:{f2_pr:.1f}")

                if len(observations) >= 3:
                    # Compute GPS time from RAWX header
                    gps_epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
                    gps_time = gps_epoch + timedelta(weeks=week, seconds=rcvTow)
                    obs_queue.put((gps_time, observations))
                    n_epochs += 1

                    if n_epochs % 60 == 0:
                        log.info(f"Serial: {n_epochs} epochs, "
                                 f"last had {len(observations)} SVs")

        except Exception as e:
            if not stop_event.is_set():
                log.error(f"Serial reader error: {e}")
            break

    ser.close()
    log.info(f"Serial reader stopped after {n_epochs} epochs")


# ── NTRIP correction reader ────────────────────────────────────────────────── #

def ntrip_reader(stream, beph, ssr, stop_event, label="NTRIP"):
    """Read RTCM3 messages from an NtripStream.

    Routes broadcast ephemeris messages to BroadcastEphemeris and SSR
    messages to SSRState.
    """
    msg_counts = defaultdict(int)
    n_total = 0

    try:
        for msg in stream.messages():
            if stop_event.is_set():
                break

            identity = str(getattr(msg, 'identity', ''))
            msg_counts[identity] += 1
            n_total += 1

            # Route to appropriate handler
            if identity in EPH_MSG_TYPES:
                prn = beph.update_from_rtcm(msg)
                if prn and beph.n_satellites % 10 == 0:
                    log.debug(f"[{label}] {beph.summary()}")

            elif identity in SSR_MSG_TYPES or identity.startswith('4076_'):
                ssr.update_from_rtcm(msg)

            if n_total % 100 == 0:
                log.info(f"[{label}] {n_total} msgs | {beph.summary()} | {ssr.summary()}")

    except Exception as e:
        if not stop_event.is_set():
            log.error(f"[{label}] Error: {e}")

    stream.disconnect()
    log.info(f"[{label}] Stopped. Total: {n_total} msgs. "
             f"Types: {dict(msg_counts)}")


# ── File-based replay mode ──────────────────────────────────────────────────── #

def run_replay(args):
    """Run in file replay mode using SP3/CLK (same as M3, validates streaming)."""
    log.info("=== Replay mode (file-based corrections) ===")

    lat, lon, alt = [float(v) for v in args.known_pos.split(',')]
    known_ecef = lla_to_ecef(lat, lon, alt)
    leap_delta = timedelta(seconds=args.leap)

    sp3 = SP3(args.sp3)
    log.info(f"SP3: {len(sp3.epochs)} epochs, {len(sp3.positions)} sats")

    clk_file = CLKFile(args.clk) if args.clk else None
    if clk_file:
        log.info(f"CLK: {len(clk_file.prns())} satellites")

    osb = OSBParser(args.osb) if args.osb else None
    if osb:
        log.info(f"OSB: {len(osb.prns())} satellites")

    systems = set(args.systems.split(','))
    epochs = load_ppp_epochs(args.replay, systems=systems, osb=osb)
    log.info(f"Loaded {len(epochs)} epochs")

    filt = FixedPosFilter(known_ecef)
    filt.prev_clock = 0.0

    out_f = None
    out_w = None
    if args.out:
        out_f = open(args.out, 'w', newline='')
        out_w = csv.writer(out_f)
        out_w.writerow(['timestamp', 'clock_ns', 'clock_sigma_ns',
                        'n_meas', 'n_td', 'rms_m', 'correction_source'])

    prev_t = None
    results = []
    for i, (ts_str, obs) in enumerate(epochs):
        t = timestamp_to_gpstime(ts_str) + leap_delta

        if prev_t is not None:
            dt = (t - prev_t).total_seconds()
            filt.predict(dt)
        prev_t = t

        n_used, resid, n_td = filt.update(obs, sp3, t, clk_file=clk_file)

        clk_ns = filt.x[filt.IDX_CLK] / C * 1e9
        clk_sigma = math.sqrt(filt.P[filt.IDX_CLK, filt.IDX_CLK]) / C * 1e9
        rms = np.sqrt(np.mean(resid ** 2)) if len(resid) > 0 else 0

        results.append((ts_str, clk_ns, clk_sigma, n_used, n_td, rms))

        if out_w:
            out_w.writerow([ts_str, f'{clk_ns:.3f}', f'{clk_sigma:.4f}',
                            n_used, n_td, f'{rms:.4f}', 'SP3+CLK'])

        if (i + 1) % 60 == 0:
            log.info(f"  [{i+1}/{len(epochs)}] {ts_str[:19]} "
                     f"clk={clk_ns:.1f}ns ±{clk_sigma:.2f}ns "
                     f"n={n_used} td={n_td} rms={rms:.3f}m")

    if out_f:
        out_f.close()

    # Summary
    clks = np.array([r[1] for r in results])
    log.info(f"\n{'='*60}")
    log.info(f"  Replay complete: {len(results)} epochs")
    log.info(f"  Clock: {np.mean(clks):.1f} ± {np.std(clks):.2f} ns")
    if len(clks) > 100:
        dt_s = np.arange(len(clks))
        detrended = clks - np.polyval(np.polyfit(dt_s, clks, 1), dt_s)
        log.info(f"  Detrended std: {np.std(detrended):.3f} ns")
    log.info(f"{'='*60}")


# ── Real-time mode ──────────────────────────────────────────────────────────── #

def run_realtime(args):
    """Run in real-time mode with serial F9T + NTRIP corrections."""
    log.info("=== Real-time mode ===")

    lat, lon, alt = [float(v) for v in args.known_pos.split(',')]
    known_ecef = lla_to_ecef(lat, lon, alt)
    log.info(f"Known position: {lat:.6f}, {lon:.6f}, {alt:.1f}m")

    # Shared state
    beph = BroadcastEphemeris()
    ssr = SSRState()
    corrections = RealtimeCorrections(beph, ssr)
    obs_queue = queue.Queue(maxsize=100)
    stop_event = threading.Event()

    # Start NTRIP threads
    ntrip_threads = []

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
        ntrip_threads.append(t_eph)
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
        ntrip_threads.append(t_ssr)
        log.info(f"SSR stream: {args.caster}:{args.port}/{args.ssr_mount}")

    # Wait for initial ephemeris before starting serial
    if args.eph_mount:
        log.info("Waiting for broadcast ephemeris...")
        warmup_start = time.time()
        while beph.n_satellites < 8 and time.time() - warmup_start < 120:
            time.sleep(1)
            if int(time.time() - warmup_start) % 10 == 0:
                log.info(f"  Warmup: {beph.summary()}")
        log.info(f"Warmup complete: {beph.summary()}")

    # Parse systems filter
    systems = set(args.systems.split(',')) if args.systems else None
    log.info(f"Systems filter: {systems}")

    # Start serial reader
    t_serial = threading.Thread(
        target=serial_reader,
        args=(args.serial, args.baud, obs_queue, stop_event, beph, systems, ssr),
        daemon=True,
    )
    t_serial.start()
    log.info(f"Serial: {args.serial} at {args.baud} baud")

    # Initialize filter
    filt = FixedPosFilter(known_ecef)
    filt.prev_clock = 0.0

    out_f = None
    out_w = None
    if args.out:
        out_f = open(args.out, 'w', newline='')
        out_w = csv.writer(out_f)
        out_w.writerow(['timestamp', 'clock_ns', 'clock_sigma_ns',
                        'n_meas', 'n_td', 'rms_m', 'correction_source',
                        'n_ssr_orbit', 'n_ssr_clock', 'n_beph'])

    # Main processing loop
    prev_t = None
    n_epochs = 0
    start_time = time.time()
    try:
        while not stop_event.is_set():
            # Check duration limit
            if args.duration and (time.time() - start_time) > args.duration:
                log.info(f"Duration limit reached ({args.duration}s)")
                break

            try:
                gps_time, observations = obs_queue.get(timeout=5)
            except queue.Empty:
                continue

            # EKF predict
            if prev_t is not None:
                dt = (gps_time - prev_t).total_seconds()
                if dt <= 0 or dt > 30:
                    log.warning(f"Suspicious dt={dt:.1f}s, skipping")
                    prev_t = gps_time
                    continue
                filt.predict(dt)
            prev_t = gps_time

            # EKF update — use RealtimeCorrections (SP3-compatible interface)
            n_used, resid, n_td = filt.update(
                observations, corrections, gps_time,
                clk_file=corrections,  # RealtimeCorrections implements sat_clock()
            )

            clk_ns = filt.x[filt.IDX_CLK] / C * 1e9
            clk_sigma = math.sqrt(filt.P[filt.IDX_CLK, filt.IDX_CLK]) / C * 1e9
            rms = np.sqrt(np.mean(resid ** 2)) if len(resid) > 0 else 0
            n_epochs += 1

            # Determine correction source
            source = 'broadcast'
            if ssr.n_clock > 0:
                source = 'SSR'

            ts_str = gps_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:23]

            if out_w:
                out_w.writerow([
                    ts_str, f'{clk_ns:.3f}', f'{clk_sigma:.4f}',
                    n_used, n_td, f'{rms:.4f}', source,
                    ssr.n_orbit, ssr.n_clock, beph.n_satellites,
                ])

            # Console status every 10 epochs
            if n_epochs % 10 == 0:
                elapsed = time.time() - start_time
                log.info(
                    f"  [{n_epochs}] {ts_str[:19]} "
                    f"clk={clk_ns:+.1f}ns ±{clk_sigma:.2f}ns "
                    f"n={n_used} td={n_td} rms={rms:.3f}m "
                    f"[{source}] "
                    f"beph={beph.n_satellites} ssr_clk={ssr.n_clock}"
                )

    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        stop_event.set()
        if out_f:
            out_f.close()

    elapsed = time.time() - start_time
    log.info(f"\n{'='*60}")
    log.info(f"  Real-time PPP complete")
    log.info(f"  Duration: {elapsed:.0f}s, Epochs: {n_epochs}")
    log.info(f"  {corrections.summary()}")
    log.info(f"{'='*60}")


# ── Main ──────────────────────────────────────────────────────────────────── #

def main():
    ap = argparse.ArgumentParser(
        description="Real-time PPP clock estimation (PePPAR Fix M4)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Position
    ap.add_argument("--known-pos", required=True,
                    help="Known position as lat,lon,alt (e.g. '41.843,-88.104,201.7')")
    ap.add_argument("--leap", type=int, default=18,
                    help="UTC-GPS leap seconds (default: 18)")
    ap.add_argument("--systems", default="gps,gal,bds",
                    help="GNSS systems to use (default: gps,gal,bds)")

    # Serial (real-time mode)
    serial = ap.add_argument_group("Serial (real-time)")
    serial.add_argument("--serial", help="Serial port for F9T (e.g. /dev/gnss-bot)")
    serial.add_argument("--baud", type=int, default=115200)

    # NTRIP (real-time mode)
    ntrip = ap.add_argument_group("NTRIP corrections")
    ntrip.add_argument("--ntrip-conf", help="NTRIP config file (INI format)")
    ntrip.add_argument("--caster", help="NTRIP caster hostname")
    ntrip.add_argument("--port", type=int, default=2101)
    ntrip.add_argument("--tls", action="store_true", help="Use TLS (auto for port 443)")
    ntrip.add_argument("--eph-mount", help="Mountpoint for broadcast ephemeris")
    ntrip.add_argument("--ssr-mount", help="Mountpoint for SSR corrections")
    ntrip.add_argument("--user", help="NTRIP username")
    ntrip.add_argument("--password", help="NTRIP password")

    # Replay (file-based mode)
    replay = ap.add_argument_group("Replay (file-based)")
    replay.add_argument("--replay", help="RAWX CSV file for replay mode")
    replay.add_argument("--sp3", help="SP3 orbit file (replay mode)")
    replay.add_argument("--clk", help="RINEX CLK file (replay mode)")
    replay.add_argument("--osb", help="SINEX BIAS file (replay mode)")

    # Output
    ap.add_argument("--out", help="Output CSV file")
    ap.add_argument("--duration", type=int, default=0,
                    help="Duration in seconds (0 = unlimited)")
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

    if args.replay:
        if not args.sp3:
            ap.error("--replay requires --sp3")
        run_replay(args)
    elif args.serial:
        if not args.caster and not args.eph_mount:
            log.warning("No NTRIP source — will use broadcast ephemeris from receiver only")
        run_realtime(args)
    else:
        ap.error("Must specify either --serial (real-time) or --replay (file-based)")


if __name__ == "__main__":
    main()

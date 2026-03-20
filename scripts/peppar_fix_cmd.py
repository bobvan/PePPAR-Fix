#!/usr/bin/env python3
"""peppar-fix: Unified GNSS-disciplined clock process.

Single process with two phases:

Phase 1 — Bootstrap (no known position):
  PPPFilter estimates position from scratch. Solution logged.
  When converged: save position, transition to phase 2.
  Skipped if --position-file points to an existing converged position.

Phase 2 — Steady state (position known):
  FixedPosFilter estimates clock. Solution logged.
  Optional consumers:
  - PHC servo (--servo /dev/ptp0): disciplines hardware clock from dt_rx
  - NTRIP caster (--caster :2102): streams RTCM to clients (future)

Usage:
    peppar-fix --serial /dev/gnss-top --ntrip-conf ntrip.conf \\
        --position-file data/position.json \\
        --servo /dev/ptp0 --pps-pin 1 \\
        --out solution.csv --systems gps,gal,bds

    # Bootstrap only (no servo):
    peppar-fix --serial /dev/gnss-top --ntrip-conf ntrip.conf \\
        --position-file data/position.json --out bootstrap.csv

    # With existing position (skip bootstrap):
    peppar-fix --serial /dev/gnss-top --ntrip-conf ntrip.conf \\
        --position-file data/position.json --servo /dev/ptp0 --pps-pin 1
"""

import argparse
import csv
import logging
import math
import queue
import signal
import sys
import threading
import time

import numpy as np

from solve_pseudorange import C, ecef_to_lla, lla_to_ecef
from solve_ppp import PPPFilter, FixedPosFilter, ls_init
from broadcast_eph import BroadcastEphemeris
from ssr_corrections import SSRState, RealtimeCorrections
from ntrip_client import NtripStream
from realtime_ppp import serial_reader, ntrip_reader, QErrStore
from peppar_fix import save_position, load_position, PositionWatchdog

log = logging.getLogger("peppar-fix")


# ── Convergence detection (from peppar_find_position) ─────────────────── #

def position_sigma_3d(P):
    """Compute 3D position sigma from EKF covariance matrix."""
    P_pos = P[:3, :3]
    return math.sqrt(P_pos[0, 0] + P_pos[1, 1] + P_pos[2, 2])


# ── NTRIP config loading ─────────────────────────────────────────────── #

def load_ntrip_config(args):
    """Load NTRIP configuration from config file, merging with CLI args."""
    if args.ntrip_conf:
        import configparser
        conf = configparser.ConfigParser()
        conf.read(args.ntrip_conf)
        if 'ntrip' in conf:
            s = conf['ntrip']
            if not args.ntrip_caster:
                args.ntrip_caster = s.get('caster', args.ntrip_caster)
            if args.ntrip_port == 2101 and s.get('port'):
                args.ntrip_port = int(s.get('port'))
            if not args.ntrip_user:
                args.ntrip_user = s.get('user', args.ntrip_user)
            if not args.ntrip_password:
                args.ntrip_password = s.get('password', args.ntrip_password)
            if not args.ntrip_tls and s.getboolean('tls', False):
                args.ntrip_tls = True
            if not args.ssr_mount and s.get('mount'):
                args.ssr_mount = s.get('mount')


# ── Shared infrastructure setup ──────────────────────────────────────── #

def start_ntrip_threads(args, beph, ssr, stop_event):
    """Start NTRIP threads for ephemeris and SSR corrections."""
    threads = []
    use_tls = args.ntrip_tls or args.ntrip_port == 443

    if args.eph_mount:
        eph_stream = NtripStream(
            caster=args.ntrip_caster, port=args.ntrip_port,
            mountpoint=args.eph_mount,
            user=args.ntrip_user, password=args.ntrip_password,
            tls=use_tls,
        )
        t = threading.Thread(
            target=ntrip_reader,
            args=(eph_stream, beph, ssr, stop_event, "EPH"),
            daemon=True,
        )
        t.start()
        threads.append(t)
        log.info(f"Ephemeris stream: {args.ntrip_caster}:{args.ntrip_port}/{args.eph_mount}")

    if args.ssr_mount:
        ssr_stream = NtripStream(
            caster=args.ntrip_caster, port=args.ntrip_port,
            mountpoint=args.ssr_mount,
            user=args.ntrip_user, password=args.ntrip_password,
            tls=use_tls,
        )
        t = threading.Thread(
            target=ntrip_reader,
            args=(ssr_stream, beph, ssr, stop_event, "SSR"),
            daemon=True,
        )
        t.start()
        threads.append(t)
        log.info(f"SSR stream: {args.ntrip_caster}:{args.ntrip_port}/{args.ssr_mount}")

    return threads


def wait_for_ephemeris(beph, stop_event, systems=None, timeout_s=120):
    """Wait for broadcast ephemeris — each configured system must have >= 8 SVs."""
    SYS_TO_PREFIX = {'gps': 'G', 'gal': 'E', 'bds': 'C'}
    required = {SYS_TO_PREFIX[s] for s in (systems or {'gps', 'gal', 'bds'}) if s in SYS_TO_PREFIX}
    required.add('G')
    log.info(f"Waiting for broadcast ephemeris (need {required})...")
    warmup_start = time.time()
    while time.time() - warmup_start < timeout_s:
        if stop_event.is_set():
            return False
        by_sys = {}
        for prn in beph.satellites:
            s = prn[0]
            by_sys[s] = by_sys.get(s, 0) + 1
        if all(by_sys.get(p, 0) >= 8 for p in required):
            break
        time.sleep(1)
        if int(time.time() - warmup_start) % 10 == 0:
            log.info(f"  Warmup: {beph.summary()}")
    log.info(f"Warmup complete: {beph.summary()}")
    return True


# ── Phase 1: Bootstrap ─────────────────────────────────────────────────── #

def run_bootstrap(args, obs_queue, corrections, stop_event, out_w=None):
    """Run PPPFilter to estimate position from scratch.

    Returns:
        (ecef, sigma_m) on convergence, or None on timeout/error.
    """
    log.info("=== Phase 1: Position bootstrap (PPPFilter) ===")

    # Seed position
    seed_ecef = None
    if args.seed_pos:
        lat, lon, alt = [float(v) for v in args.seed_pos.split(',')]
        seed_ecef = lla_to_ecef(lat, lon, alt)
        log.info(f"Seed position: {lat:.6f}, {lon:.6f}, {alt:.1f}m")

    filt = PPPFilter()
    filt_initialized = False

    prev_t = None
    prev_pos_ecef = None
    n_epochs = 0
    n_empty = 0
    converged_at = None
    start_time = time.time()

    while not stop_event.is_set():
        elapsed = time.time() - start_time

        if args.timeout and elapsed > args.timeout:
            log.warning(f"Bootstrap timeout after {elapsed:.0f}s")
            return None

        try:
            gps_time, observations = obs_queue.get(timeout=5)
        except queue.Empty:
            n_empty += 1
            if n_empty > 12:
                log.error("No observations for 60s during bootstrap")
                return None
            continue
        n_empty = 0

        # Initialize filter on first epoch with enough satellites
        if not filt_initialized:
            if seed_ecef is not None:
                init_pos = seed_ecef
                init_clk = 0.0
            else:
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

        # Manage ambiguities
        current_svs = {o['sv'] for o in observations}
        if filt.prev_obs:
            slipped = filt.detect_cycle_slips(observations, filt.prev_obs)
            for sv in slipped:
                filt.remove_ambiguity(sv)

        for obs in observations:
            sv = obs['sv']
            if sv not in filt.sv_to_idx and obs.get('phi_if_m') is not None:
                sat_pos, sat_clk = corrections.sat_position(sv, gps_time)
                if sat_pos is not None:
                    N_init = obs['pr_if'] - obs['phi_if_m']
                    filt.add_ambiguity(sv, N_init)

        filt.prev_obs = {o['sv']: o for o in observations}

        for sv in list(filt.sv_to_idx.keys()):
            if sv not in current_svs:
                filt.remove_ambiguity(sv)

        # EKF update
        n_used, resid, sys_counts = filt.update(
            observations, corrections, gps_time, clk_file=corrections)

        if n_used < 4:
            continue

        n_epochs += 1

        pos_ecef = filt.x[:3]
        sigma_3d = position_sigma_3d(filt.P)
        lat, lon, alt = ecef_to_lla(pos_ecef[0], pos_ecef[1], pos_ecef[2])
        rms = np.sqrt(np.mean(resid ** 2)) if len(resid) > 0 else 0

        # CSV output
        if out_w:
            out_w.writerow([
                gps_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:23],
                'bootstrap',
                f'{lat:.7f}', f'{lon:.7f}', f'{alt:.3f}',
                f'{sigma_3d:.4f}', '', '',
                n_used, f'{rms:.4f}',
                '', '', '', len(filt.sv_to_idx),
            ])

        if n_epochs % 5 == 0:
            log.info(
                f"  [{n_epochs}] σ={sigma_3d:.3f}m "
                f"pos=({lat:.6f}, {lon:.6f}, {alt:.1f}) "
                f"n={n_used} amb={len(filt.sv_to_idx)} "
                f"rms={rms:.3f}m [{elapsed:.0f}s]"
            )

        # Convergence check
        pos_stable = True
        if prev_pos_ecef is not None:
            pos_delta = np.linalg.norm(pos_ecef - prev_pos_ecef)
            pos_stable = pos_delta < args.sigma
        prev_pos_ecef = pos_ecef.copy()

        if sigma_3d < args.sigma and pos_stable:
            if converged_at is None:
                converged_at = n_epochs
            if n_epochs - converged_at >= 30:
                log.info(f"CONVERGED at epoch {n_epochs} "
                         f"(σ={sigma_3d:.4f}m, rms={rms:.3f}m)")
                return (pos_ecef, float(sigma_3d))
        else:
            converged_at = None

    return None


# ── Phase 2: Steady state ────────────────────────────────────────────── #

def run_steady_state(args, known_ecef, obs_queue, corrections, beph, ssr,
                     stop_event, out_w=None):
    """Run FixedPosFilter for clock estimation with optional servo.

    This is the steady-state phase: position is known, we estimate clock
    offset and optionally discipline a PHC.
    """
    log.info("=== Phase 2: Steady state (FixedPosFilter) ===")
    lat, lon, alt = ecef_to_lla(known_ecef[0], known_ecef[1], known_ecef[2])
    log.info(f"Position: {lat:.6f}, {lon:.6f}, {alt:.1f}m")

    filt = FixedPosFilter(known_ecef)
    filt.prev_clock = 0.0
    watchdog = PositionWatchdog(threshold_m=args.watchdog_threshold)

    # Optional servo setup (PTP imports only loaded when needed)
    servo_ctx = None
    if args.servo:
        servo_ctx = _setup_servo(args, known_ecef)
        if servo_ctx is None:
            log.error("Failed to set up PHC servo, continuing without it")

    prev_t = None
    n_epochs = 0
    start_time = time.time()

    try:
        while not stop_event.is_set():
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

            # EKF update
            n_used, resid, n_td = filt.update(
                observations, corrections, gps_time,
                clk_file=corrections,
            )

            if n_used < 4:
                continue

            # Watchdog
            resid_rms = float(np.sqrt(np.mean(resid ** 2))) if len(resid) > 0 else 0.0
            watchdog.update(resid_rms, n_used)
            if watchdog.alarmed:
                log.error("POSITION WATCHDOG ALARM: antenna may have moved! "
                          "Servo disabled. Restart with correct position or "
                          "delete position file to re-bootstrap.")
                break

            dt_rx_ns = filt.x[filt.IDX_CLK] / C * 1e9
            p_clk = filt.P[filt.IDX_CLK, filt.IDX_CLK]
            dt_rx_sigma = math.sqrt(max(0, p_clk)) / C * 1e9
            n_epochs += 1

            # Extract ISBs for logging
            isb_gal_ns = 0.0
            isb_bds_ns = 0.0
            if hasattr(filt, 'IDX_ISB_GAL') and filt.x.shape[0] > filt.IDX_ISB_GAL:
                isb_gal_ns = filt.x[filt.IDX_ISB_GAL] / C * 1e9
            if hasattr(filt, 'IDX_ISB_BDS') and filt.x.shape[0] > getattr(filt, 'IDX_ISB_BDS', 999):
                isb_bds_ns = filt.x[filt.IDX_ISB_BDS] / C * 1e9

            # Correction source
            source = 'SSR' if ssr.n_clock > 0 else 'broadcast'
            ts_str = gps_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:23]

            # CSV output
            if out_w:
                out_w.writerow([
                    ts_str, 'steady',
                    f'{lat:.7f}', f'{lon:.7f}', f'{alt:.3f}',
                    '', f'{dt_rx_ns:.3f}', f'{dt_rx_sigma:.4f}',
                    n_used, f'{resid_rms:.4f}',
                    source, f'{isb_gal_ns:.3f}', f'{isb_bds_ns:.3f}',
                    n_td,
                ])

            # Feed servo if active
            if servo_ctx is not None:
                _servo_epoch(servo_ctx, args, filt, gps_time, n_epochs,
                             dt_rx_ns, dt_rx_sigma, n_used, known_ecef,
                             resid_rms, isb_gal_ns, isb_bds_ns)

            # Console status every 10 epochs
            if n_epochs % 10 == 0:
                elapsed = time.time() - start_time
                log.info(
                    f"  [{n_epochs}] {ts_str[:19]} "
                    f"clk={dt_rx_ns:+.1f}ns ±{dt_rx_sigma:.2f}ns "
                    f"n={n_used} rms={resid_rms:.3f}m "
                    f"[{source}]"
                )

    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        stop_event.set()
        if servo_ctx is not None:
            _cleanup_servo(servo_ctx)

    elapsed = time.time() - start_time
    log.info(f"Steady state complete: {elapsed:.0f}s, {n_epochs} epochs")


# ── Servo helpers (conditional PTP import) ────────────────────────────── #

def _setup_servo(args, known_ecef):
    """Set up PHC servo. Returns context dict or None on failure."""
    try:
        from peppar_fix import PtpDevice, PIServo, DisciplineScheduler
        from peppar_fix import compute_error_sources
    except ImportError:
        log.error("peppar_fix library not available for servo")
        return None

    try:
        ptp = PtpDevice(args.servo)
    except OSError as e:
        log.error(f"Cannot open PTP device {args.servo}: {e}")
        return None

    caps = ptp.get_caps()
    log.info(f"PHC: {args.servo}, max_adj={caps['max_adj']} ppb, "
             f"n_extts={caps['n_ext_ts']}, n_pins={caps['n_pins']}")

    ptp.adjfine(0.0)
    log.info("PHC adjfine reset to 0")

    # Import PTP constants for pin setup
    from peppar_fix.ptp_device import PTP_PF_EXTTS

    extts_channel = 0
    try:
        ptp.set_pin_function(args.pps_pin, PTP_PF_EXTTS, extts_channel)
    except OSError:
        log.info("Pin config not supported by driver (igc uses implicit mapping)")
    ptp.enable_extts(extts_channel, rising_edge=True)
    log.info(f"EXTTS enabled: pin={args.pps_pin}, channel={extts_channel}")

    servo = PIServo(args.track_kp, args.track_ki, max_ppb=caps['max_adj'])
    scheduler = DisciplineScheduler(
        base_interval=args.discipline_interval,
        adaptive=args.adaptive_interval,
        min_interval=args.min_interval,
        max_interval=args.max_interval,
    )

    # QErr store for TIM-TP
    qerr_store = QErrStore()

    # PPS event queue
    pps_queue = queue.Queue(maxsize=10)
    stop_pps = threading.Event()

    def extts_reader():
        while not stop_pps.is_set():
            event = ptp.read_extts(timeout_ms=1500)
            if event is None:
                continue
            phc_sec, phc_nsec, _idx = event
            try:
                pps_queue.put_nowait((phc_sec, phc_nsec))
            except queue.Full:
                while not pps_queue.empty():
                    try:
                        pps_queue.get_nowait()
                    except queue.Empty:
                        break
                pps_queue.put_nowait((phc_sec, phc_nsec))

    t_extts = threading.Thread(target=extts_reader, daemon=True)
    t_extts.start()
    log.info("EXTTS reader started")

    # Servo log file
    log_f = None
    log_w = None
    if args.servo_log:
        log_f = open(args.servo_log, 'w', newline='')
        log_w = csv.writer(log_f)
        log_w.writerow([
            'timestamp', 'gps_second', 'phc_sec', 'phc_nsec',
            'dt_rx_ns', 'dt_rx_sigma_ns', 'pps_error_ns', 'qerr_ns',
            'source', 'source_error_ns', 'source_confidence_ns',
            'adjfine_ppb', 'phase', 'n_meas', 'gain_scale',
            'discipline_interval', 'n_accumulated', 'watchdog_alarm',
            'isb_gal_ns', 'isb_bds_ns',
        ])

    return {
        'ptp': ptp,
        'servo': servo,
        'scheduler': scheduler,
        'qerr_store': qerr_store,
        'pps_queue': pps_queue,
        'stop_pps': stop_pps,
        'extts_channel': extts_channel,
        'caps': caps,
        'log_f': log_f,
        'log_w': log_w,
        'phase': 'warmup',
        'adjfine_ppb': 0.0,
        'gain_scale': 1.0,
        'prev_source': None,
        'warmup_epochs': args.warmup,
        'tmode_set': False,
        'position_saved': False,
        'compute_error_sources': compute_error_sources,
    }


def _pps_fractional_error(phc_nsec):
    """Compute PHC error from PPS fractional second."""
    if phc_nsec <= 500_000_000:
        return float(phc_nsec)
    else:
        return float(phc_nsec) - 1_000_000_000


def _phc_gps_offset_s(phc_sec, phc_nsec, gps_unix_sec):
    """Whole-second offset: PHC_time - GPS_time."""
    phc_rounded = phc_sec if phc_nsec < 500_000_000 else phc_sec + 1
    return phc_rounded - gps_unix_sec


def _servo_epoch(ctx, args, filt, gps_time, n_epochs,
                 dt_rx_ns, dt_rx_sigma, n_used, known_ecef,
                 resid_rms, isb_gal_ns, isb_bds_ns):
    """Process one servo epoch: read PPS, compute error, steer PHC."""
    ptp = ctx['ptp']
    servo = ctx['servo']
    scheduler = ctx['scheduler']
    qerr_store = ctx['qerr_store']
    pps_queue = ctx['pps_queue']
    log_w = ctx['log_w']
    compute_error_sources = ctx['compute_error_sources']

    STEP_THRESHOLD_NS = 10_000
    BASE_KP = args.track_kp
    BASE_KI = args.track_ki
    GAIN_REF_SIGMA = args.gain_ref_sigma
    GAIN_MIN_SCALE = 0.1
    GAIN_MAX_SCALE = 1.0
    CONVERGE_ERROR_NS = 500
    CONVERGE_MIN_SCALE = 2.0

    # Once filter converges: switch F9T to timing mode
    if n_epochs >= 300 and dt_rx_sigma < 100.0:
        sigma_m = dt_rx_sigma * 1e-9 * C

        if args.position_file and not ctx['position_saved'] and sigma_m < 0.1:
            save_position(
                args.position_file, known_ecef,
                sigma_m=sigma_m, source="peppar_fix",
                note=f"saved after {n_epochs} epochs, dt_rx_sigma={dt_rx_sigma:.2f}ns",
            )
            ctx['position_saved'] = True
            log.info(f"Position saved to {args.position_file} "
                     f"(sigma={sigma_m:.4f}m)")

        if not ctx['tmode_set'] and sigma_m < 0.1:
            try:
                from phc_servo import build_tmode_fixed_msg
                tmode_msg = build_tmode_fixed_msg(known_ecef)
                if tmode_msg is not None:
                    # Would need config_queue to serial_reader — skip for now
                    # The F9T timing mode can be set separately via configure_f9t.py
                    ctx['tmode_set'] = True
                    lat, lon, alt = ecef_to_lla(
                        known_ecef[0], known_ecef[1], known_ecef[2])
                    log.info(f"F9T timing mode ready "
                             f"({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
            except ImportError:
                ctx['tmode_set'] = True

    # Get PPS event
    try:
        phc_sec, phc_nsec = pps_queue.get(timeout=0.5)
    except queue.Empty:
        if n_epochs % 10 == 0:
            log.info(f"  [{n_epochs}] No PPS event for this epoch")
        return

    gps_unix_sec = int(round(gps_time.timestamp()))
    ts_str = gps_time.strftime('%Y-%m-%d %H:%M:%S')
    pps_error_ns = _pps_fractional_error(phc_nsec)

    qerr_ns, _ = qerr_store.get()

    sources = compute_error_sources(
        pps_error_ns, qerr_ns, dt_rx_ns, dt_rx_sigma,
    )
    best = sources[0]

    # Warmup phase
    if ctx['phase'] == 'warmup':
        if n_epochs >= ctx['warmup_epochs']:
            epoch_offset = _phc_gps_offset_s(phc_sec, phc_nsec, gps_unix_sec)
            log.info(f"  Servo warmup complete ({n_epochs} epochs, "
                     f"best={best}, epoch_offset={epoch_offset}s)")
            if epoch_offset != 0 or abs(best.error_ns) > STEP_THRESHOLD_NS:
                ctx['phase'] = 'step'
            else:
                ctx['phase'] = 'tracking'
                log.info(f"  → tracking (no step needed)")
        elif n_epochs % 10 == 0:
            log.info(f"  [{n_epochs}] servo warmup: best={best} "
                     f"dt_rx={dt_rx_ns:+.1f}±{dt_rx_sigma:.1f}ns")
        _log_servo(log_w, ts_str, gps_unix_sec, phc_sec, phc_nsec,
                   dt_rx_ns, dt_rx_sigma, pps_error_ns, qerr_ns, best,
                   ctx['adjfine_ppb'], ctx['phase'], n_used,
                   ctx['gain_scale'], scheduler, isb_gal_ns, isb_bds_ns)
        return

    # Step phase
    if ctx['phase'] == 'step':
        epoch_offset = _phc_gps_offset_s(phc_sec, phc_nsec, gps_unix_sec)
        total_offset_ns = epoch_offset * 1_000_000_000 + best.error_ns
        log.info(f"  STEP: epoch_offset={epoch_offset}s, "
                 f"source={best}, total={total_offset_ns:+.0f}ns")

        import subprocess
        adj_s = -total_offset_ns / 1_000_000_000
        result = subprocess.run(
            ['/usr/sbin/phc_ctl', args.servo, '--',
             'adj', f'{adj_s:.9f}'],
            capture_output=True, text=True,
        )
        if result.returncode == 0:
            log.info(f"  phc_ctl adj {adj_s:.6f}s: {result.stdout.strip()}")
        else:
            log.error(f"  phc_ctl adj failed: {result.stderr.strip()}")

        # Reset servo and scheduler
        from peppar_fix import PIServo, DisciplineScheduler
        ctx['servo'] = PIServo(BASE_KP, BASE_KI, max_ppb=ctx['caps']['max_adj'])
        servo = ctx['servo']
        ctx['scheduler'] = DisciplineScheduler(
            base_interval=args.discipline_interval,
            adaptive=args.adaptive_interval,
            min_interval=args.min_interval,
            max_interval=args.max_interval,
        )
        scheduler = ctx['scheduler']
        ctx['phase'] = 'tracking'

        # Flush stale PPS events
        time.sleep(2)
        while not pps_queue.empty():
            try:
                pps_queue.get_nowait()
            except queue.Empty:
                break
        return

    # Tracking phase
    if abs(best.error_ns) > 500 and not scheduler._converging:
        log.warning(f"  Outlier: {best}, skipping")
        return

    scheduler.accumulate(best.error_ns, best.confidence_ns, best.name)

    if ctx['prev_source'] != best.name:
        if ctx['prev_source'] is not None:
            log.info(f"  Source: {ctx['prev_source']} → {best.name} "
                     f"(confidence {best.confidence_ns:.1f}ns)")
        ctx['prev_source'] = best.name

    if scheduler.should_correct():
        avg_error, avg_confidence, n_samples = scheduler.flush()

        gain_scale = max(GAIN_MIN_SCALE, min(GAIN_MAX_SCALE,
                         GAIN_REF_SIGMA / avg_confidence))
        if abs(avg_error) > CONVERGE_ERROR_NS:
            gain_scale = max(gain_scale, CONVERGE_MIN_SCALE)

        servo.kp = BASE_KP * gain_scale
        servo.ki = BASE_KI * gain_scale

        adjfine_ppb = -servo.update(avg_error, dt=float(n_samples))
        # Anti-windup: if adjfine is at the rail, reset integral
        # to prevent windup-driven oscillation
        if abs(adjfine_ppb) >= ctx['caps']['max_adj'] * 0.95:
            servo.integral = -adjfine_ppb / servo.ki if servo.ki != 0 else 0
            log.warning(f'  Anti-windup: adj={adjfine_ppb:+.0f}ppb at rail, integral reset')
        ptp.adjfine(adjfine_ppb)
        ctx['adjfine_ppb'] = adjfine_ppb
        ctx['gain_scale'] = gain_scale

        scheduler.update_drift_rate(time.monotonic(), adjfine_ppb)
        scheduler.compute_adaptive_interval(avg_confidence)

        if n_epochs % 10 == 0:
            log.info(f"  [{n_epochs}] {best.name}: "
                     f"err={avg_error:+.1f}ns (avg {n_samples}) "
                     f"adj={adjfine_ppb:+.1f}ppb "
                     f"gain={gain_scale:.2f}x "
                     f"interval={scheduler.interval}")
    else:
        if n_epochs % 10 == 0:
            log.info(f"  [{n_epochs}] {best.name}: "
                     f"err={best.error_ns:+.1f}ns "
                     f"coast ({scheduler.n_accumulated}/{scheduler.interval}) "
                     f"adj={ctx['adjfine_ppb']:+.1f}ppb")

    _log_servo(log_w, ts_str, gps_unix_sec, phc_sec, phc_nsec,
               dt_rx_ns, dt_rx_sigma, pps_error_ns, qerr_ns, best,
               ctx['adjfine_ppb'], ctx['phase'], n_used,
               ctx['gain_scale'], scheduler, isb_gal_ns, isb_bds_ns)


def _log_servo(log_w, ts_str, gps_unix_sec, phc_sec, phc_nsec,
               dt_rx_ns, dt_rx_sigma, pps_error_ns, qerr_ns, best,
               adjfine_ppb, phase, n_used, gain_scale, scheduler,
               isb_gal_ns, isb_bds_ns):
    """Write one servo log row."""
    if log_w is None:
        return
    log_w.writerow([
        ts_str, gps_unix_sec, phc_sec, phc_nsec,
        f'{dt_rx_ns:.3f}', f'{dt_rx_sigma:.3f}',
        f'{pps_error_ns:.1f}', f'{qerr_ns:.3f}' if qerr_ns is not None else '',
        best.name, f'{best.error_ns:.3f}', f'{best.confidence_ns:.3f}',
        f'{adjfine_ppb:.3f}', phase, n_used, f'{gain_scale:.3f}',
        scheduler.interval, scheduler.n_accumulated, 0,
        f'{isb_gal_ns:.3f}', f'{isb_bds_ns:.3f}',
    ])


def _cleanup_servo(ctx):
    """Clean up servo resources."""
    ctx['stop_pps'].set()
    ptp = ctx['ptp']
    try:
        ptp.adjfine(0.0)
    except Exception:
        pass
    ptp.disable_extts(ctx['extts_channel'])
    ptp.close()
    if ctx['log_f']:
        ctx['log_f'].close()
    log.info("PHC servo cleaned up")


# ── Main ──────────────────────────────────────────────────────────────── #

def run(args):
    """Main entry point: bootstrap → steady state."""
    stop_event = threading.Event()

    def on_signal(signum, frame):
        log.info("Signal received, shutting down")
        stop_event.set()
    signal.signal(signal.SIGTERM, on_signal)

    # Shared state
    beph = BroadcastEphemeris()
    ssr = SSRState()
    corrections = RealtimeCorrections(beph, ssr)
    obs_queue = queue.Queue(maxsize=100)

    # QErr store (shared with serial reader if servo is active)
    qerr_store = QErrStore() if args.servo else None

    # Load NTRIP config
    load_ntrip_config(args)

    if not args.ntrip_caster and not args.eph_mount:
        log.warning("No NTRIP source — using broadcast ephemeris from receiver only")

    # Start NTRIP threads
    start_ntrip_threads(args, beph, ssr, stop_event)

    # Parse systems filter (needed before warmup)
    systems = set(args.systems.split(',')) if args.systems else None
    log.info(f"Systems: {systems}")

    # Wait for ephemeris
    if args.eph_mount:
        if not wait_for_ephemeris(beph, stop_event, systems=systems):
            return 1

    # Start serial reader
    serial_kwargs = {}
    if qerr_store:
        serial_kwargs['qerr_store'] = qerr_store
    t_serial = threading.Thread(
        target=serial_reader,
        args=(args.serial, args.baud, obs_queue, stop_event, beph, systems, ssr),
        kwargs=serial_kwargs,
        daemon=True,
    )
    t_serial.start()
    log.info(f"Serial: {args.serial} at {args.baud} baud")

    # Open CSV output
    out_f = None
    out_w = None
    if args.out:
        out_f = open(args.out, 'w', newline='')
        out_w = csv.writer(out_f)
        out_w.writerow([
            'timestamp', 'phase',
            'lat', 'lon', 'alt_m',
            'sigma_3d_m', 'clock_ns', 'clock_sigma_ns',
            'n_meas', 'rms_m',
            'correction_source', 'isb_gal_ns', 'isb_bds_ns',
            'n_ambiguities',
        ])

    # Determine starting phase
    known_ecef = None

    if args.known_pos:
        lat, lon, alt = [float(v) for v in args.known_pos.split(',')]
        known_ecef = lla_to_ecef(lat, lon, alt)
        log.info(f"Position (CLI): {lat:.6f}, {lon:.6f}, {alt:.1f}m")
    elif args.position_file:
        known_ecef = load_position(args.position_file)
        if known_ecef is not None:
            lat, lon, alt = ecef_to_lla(known_ecef[0], known_ecef[1], known_ecef[2])
            log.info(f"Position (file): {lat:.6f}, {lon:.6f}, {alt:.1f}m")

    try:
        if known_ecef is None:
            # Phase 1: Bootstrap
            result = run_bootstrap(args, obs_queue, corrections, stop_event,
                                   out_w=out_w)
            if result is None:
                log.error("Bootstrap failed — no converged position")
                return 1

            pos_ecef, sigma_m = result
            known_ecef = pos_ecef

            # Save position
            if args.position_file:
                save_position(args.position_file, pos_ecef, sigma_m,
                              "ppp_bootstrap",
                              note="converged during unified run")
                log.info(f"Position saved to {args.position_file}")

        if stop_event.is_set():
            return 0

        # Validate loaded position against live pseudorange fix.
        # A tampered or stale position file would send the FixedPosFilter
        # into 100+ km residuals without any warning.
        if args.position_file or args.known_pos:
            log.info('Validating loaded position against live LS fix...')
            for _attempt in range(30):
                if stop_event.is_set():
                    return 0
                try:
                    gps_time, observations = obs_queue.get(timeout=5)
                except Exception:
                    continue
                if len(observations) < 6:
                    continue
                x_ls, ok, n_sv = ls_init(observations, corrections, gps_time,
                                          clk_file=corrections)
                if not ok or n_sv < 6:
                    continue
                ls_ecef = x_ls[:3]
                import numpy as _np
                separation_m = _np.linalg.norm(ls_ecef - known_ecef)
                ls_lat, ls_lon, ls_alt = ecef_to_lla(ls_ecef[0], ls_ecef[1], ls_ecef[2])
                log.info(f'  LS check: ({ls_lat:.4f}, {ls_lon:.4f}, {ls_alt:.0f}m) '
                         f'separation={separation_m:.0f}m from loaded position')
                if separation_m > 100:
                    log.error(f'Position file disagrees with live LS fix by {separation_m:.0f}m '
                              f'(threshold 100m). File may be stale or corrupted. '
                              f'Falling back to bootstrap.')
                    known_ecef = None
                    result = run_bootstrap(args, obs_queue, corrections, stop_event,
                                           out_w=out_w)
                    if result is None:
                        log.error('Bootstrap failed')
                        return 1
                    pos_ecef, sigma_m = result
                    known_ecef = pos_ecef
                    if args.position_file:
                        save_position(args.position_file, pos_ecef, sigma_m,
                                      'ppp_bootstrap', note='re-bootstrapped after position validation failure')
                else:
                    log.info(f'  Position validated (within {separation_m:.0f}m of LS fix)')
                break

        if stop_event.is_set():
            return 0

        # Phase 2: Steady state
        run_steady_state(args, known_ecef, obs_queue, corrections, beph, ssr,
                         stop_event, out_w=out_w)

    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        stop_event.set()
        if out_f:
            out_f.close()

    return 0


# ── CLI ──────────────────────────────────────────────────────────────── #

def main():
    ap = argparse.ArgumentParser(
        description="Unified peppar-fix: GNSS position bootstrap + clock discipline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Two-phase operation:
  Phase 1 (Bootstrap): PPPFilter estimates position from scratch.
          Skipped if --known-pos or --position-file provides a position.
  Phase 2 (Steady state): FixedPosFilter estimates clock.
          Optional: --servo for PHC discipline, --out for CSV logging.
""",
    )

    # Position
    pos = ap.add_argument_group("Position")
    pos.add_argument("--known-pos",
                     help="Known position as lat,lon,alt (skips bootstrap)")
    pos.add_argument("--seed-pos",
                     help="Seed position for bootstrap (speeds convergence)")
    pos.add_argument("--position-file", default="data/position.json",
                     help="Position file for save/load (default: data/position.json)")
    pos.add_argument("--sigma", type=float, default=0.1,
                     help="Bootstrap convergence threshold in meters (default: 0.1)")
    pos.add_argument("--timeout", type=int, default=3600,
                     help="Bootstrap timeout in seconds (default: 3600)")
    pos.add_argument("--watchdog-threshold", type=float, default=0.5,
                     help="Position watchdog threshold in meters (default: 0.5)")

    # Serial
    serial = ap.add_argument_group("Serial")
    serial.add_argument("--serial", required=True,
                        help="Serial port for F9T (e.g. /dev/gnss-top)")
    serial.add_argument("--baud", type=int, default=115200)

    # GNSS
    gnss = ap.add_argument_group("GNSS")
    gnss.add_argument("--systems", default="gps,gal,bds",
                      help="GNSS systems (default: gps,gal,bds)")
    gnss.add_argument("--leap", type=int, default=18,
                      help="UTC-GPS leap seconds (default: 18)")

    # NTRIP (corrections input)
    ntrip = ap.add_argument_group("NTRIP corrections")
    ntrip.add_argument("--ntrip-conf", help="NTRIP config file (INI format)")
    ntrip.add_argument("--ntrip-caster", help="NTRIP caster hostname")
    ntrip.add_argument("--ntrip-port", type=int, default=2101)
    ntrip.add_argument("--ntrip-tls", action="store_true")
    ntrip.add_argument("--eph-mount", help="Broadcast ephemeris mountpoint")
    ntrip.add_argument("--ssr-mount", help="SSR corrections mountpoint")
    ntrip.add_argument("--ntrip-user", help="NTRIP username")
    ntrip.add_argument("--ntrip-password", help="NTRIP password")

    # PHC servo (optional)
    servo = ap.add_argument_group("PHC servo (optional)")
    servo.add_argument("--servo", default=None,
                       help="PTP device for PHC servo (e.g. /dev/ptp0)")
    servo.add_argument("--pps-pin", type=int, default=1,
                       help="SDP pin for PPS input (default: 1 = SDP1)")
    servo.add_argument("--warmup", type=int, default=20,
                       help="Servo warmup epochs (default: 20)")
    servo.add_argument("--track-kp", type=float, default=0.3,
                       help="PI servo Kp gain (default: 0.3)")
    servo.add_argument("--track-ki", type=float, default=0.1,
                       help="PI servo Ki gain (default: 0.1)")
    servo.add_argument("--gain-ref-sigma", type=float, default=2.0,
                       help="Reference confidence for gain scale=1.0 (default: 2.0)")
    servo.add_argument("--discipline-interval", type=int, default=1,
                       help="Fixed discipline interval (default: 1)")
    servo.add_argument("--adaptive-interval", action="store_true",
                       help="Enable adaptive discipline interval")
    servo.add_argument("--max-interval", type=int, default=120,
                       help="Maximum discipline interval (default: 120)")
    servo.add_argument("--min-interval", type=int, default=1,
                       help="Minimum discipline interval (default: 1)")
    servo.add_argument("--servo-log", default=None,
                       help="CSV log file for servo data")

    # NTRIP caster output (optional, future)
    caster = ap.add_argument_group("NTRIP caster output (optional)")
    caster.add_argument("--caster", default=None,
                        help="NTRIP caster listen address (e.g. :2102) [not yet implemented]")

    # Output
    out = ap.add_argument_group("Output")
    out.add_argument("--out", help="Solution CSV output file")
    out.add_argument("--duration", type=int, default=None,
                     help="Run duration in seconds (0 = unlimited)")
    out.add_argument("-v", "--verbose", action="store_true")

    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        stream=sys.stderr,
    )

    if args.caster:
        log.warning(f"NTRIP caster output ({args.caster}) not yet implemented")

    sys.exit(run(args))


if __name__ == "__main__":
    main()

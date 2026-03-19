#!/usr/bin/env python3
"""peppar-fix: Unified GNSS-disciplined clock tool.

Single process that bootstraps position (Phase 1) then runs steady-state
clock estimation with optional PHC discipline (Phase 2).

Phase 1 — Bootstrap (no known position):
  PPPFilter estimates position from scratch. When converged: save position,
  transition to Phase 2.

Phase 2 — Steady state (position known):
  FixedPosFilter estimates clock. Solution logged to CSV.
  --servo: PI servo disciplines PHC via PPS + competitive error sources
  --caster: (future, stub — prints warning)

Skip Phase 1 if --position-file exists and contains a valid position.

Usage:
    peppar-fix --serial /dev/gnss-top --ntrip-conf ntrip.conf \
               --eph-mount BCEP00BKG0 --ssr-mount SSRA00BKG0 \
               [--position-file data/position.json] \
               [--servo /dev/ptp0 --pps-pin 1] \
               [--out solution.csv] \
               [--systems gps,gal,bds]
"""

import argparse
import configparser
import csv
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

from solve_pseudorange import C, ecef_to_lla, lla_to_ecef
from solve_ppp import PPPFilter, FixedPosFilter, ls_init
from broadcast_eph import BroadcastEphemeris
from ssr_corrections import SSRState, RealtimeCorrections
from ntrip_client import NtripStream
from realtime_ppp import serial_reader, ntrip_reader, QErrStore
from peppar_fix import save_position, load_position

log = logging.getLogger("peppar-fix")


# ── Convergence detection ────────────────────────────────────────────── #

def position_sigma_3d(P):
    """Compute 3D position sigma from EKF covariance matrix."""
    P_pos = P[:3, :3]
    return math.sqrt(P_pos[0, 0] + P_pos[1, 1] + P_pos[2, 2])


# ── NTRIP config loading ────────────────────────────────────────────── #

def load_ntrip_config(args):
    """Merge NTRIP config file with CLI args (CLI takes precedence)."""
    if args.ntrip_conf:
        conf = configparser.ConfigParser()
        conf.read(args.ntrip_conf)
        if 'ntrip' in conf:
            s = conf['ntrip']
            if not args.caster_host:
                args.caster_host = s.get('caster', args.caster_host)
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


# ── Shared infrastructure ────────────────────────────────────────────── #

def start_ntrip_threads(args, beph, ssr, stop_event):
    """Start NTRIP ephemeris and SSR correction threads. Returns thread list."""
    threads = []
    use_tls = args.tls or args.port == 443

    if args.eph_mount:
        eph_stream = NtripStream(
            caster=args.caster_host, port=args.port,
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
        threads.append(t_eph)
        log.info(f"NTRIP ephemeris: {args.caster_host}:{args.port}/{args.eph_mount}")

    if args.ssr_mount:
        ssr_stream = NtripStream(
            caster=args.caster_host, port=args.port,
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
        threads.append(t_ssr)
        log.info(f"NTRIP SSR: {args.caster_host}:{args.port}/{args.ssr_mount}")

    return threads


def wait_for_ephemeris(beph, stop_event, min_sats=8, timeout_s=120):
    """Block until broadcast ephemeris has enough satellites."""
    log.info("Waiting for broadcast ephemeris...")
    start = time.time()
    while beph.n_satellites < min_sats and time.time() - start < timeout_s:
        if stop_event.is_set():
            return False
        time.sleep(1)
        if int(time.time() - start) % 10 == 0:
            log.info(f"  Warmup: {beph.summary()}")
    log.info(f"Warmup complete: {beph.summary()}")
    return True


# ── Phase 1: Bootstrap position ──────────────────────────────────────── #

def run_bootstrap(obs_queue, stop_event, corrections, beph, args, systems):
    """Run PPPFilter to converge position from scratch.

    Returns:
        (pos_ecef, sigma_3d) on convergence, or (None, None) on timeout/error.
    """
    log.info("=== Phase 1: Position bootstrap ===")

    filt = PPPFilter()
    filt_initialized = False

    # Seed position (optional, speeds convergence)
    seed_ecef = None
    if args.seed_pos:
        lat, lon, alt = [float(v) for v in args.seed_pos.split(',')]
        seed_ecef = lla_to_ecef(lat, lon, alt)
        log.info(f"Seed position: {lat:.6f}, {lon:.6f}, {alt:.1f}m")

    # CSV output
    out_f = None
    out_w = None
    if args.out:
        out_f = open(args.out, 'w', newline='')
        out_w = csv.writer(out_f)
        out_w.writerow(['phase', 'timestamp', 'lat', 'lon', 'alt_m',
                        'sigma_3d_m', 'n_meas', 'rms_m', 'n_ambiguities'])

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

            if args.timeout and elapsed > args.timeout:
                log.warning(f"Bootstrap timeout after {elapsed:.0f}s")
                break

            try:
                gps_time, observations = obs_queue.get(timeout=5)
            except queue.Empty:
                n_empty += 1
                if n_empty > 12:
                    log.error("No observations for 60s")
                    break
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
                    log.info(f"LS init: {n_sv} SVs")

                filt.initialize(init_pos, init_clk)
                filt_initialized = True
                prev_t = gps_time
                log.info("PPPFilter initialized, starting convergence")
                continue

            # EKF predict
            dt = (gps_time - prev_t).total_seconds()
            if dt <= 0 or dt > 30:
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

            if out_w:
                out_w.writerow([
                    'bootstrap',
                    gps_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:23],
                    f'{lat:.7f}', f'{lon:.7f}', f'{alt:.3f}',
                    f'{sigma_3d:.4f}', n_used, f'{rms:.4f}',
                    len(filt.sv_to_idx),
                ])

            if n_epochs % 5 == 0:
                log.info(
                    f"  [{n_epochs}] sigma={sigma_3d:.3f}m "
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

            if sigma_3d < args.sigma and pos_stable and rms < 10.0:
                if converged_at is None:
                    converged_at = n_epochs
                if n_epochs - converged_at >= 30:
                    converged = True
                    log.info(f"CONVERGED at epoch {n_epochs} "
                             f"(sigma={sigma_3d:.4f}m, rms={rms:.3f}m)")
                    break
            else:
                converged_at = None

    except KeyboardInterrupt:
        log.info("Bootstrap interrupted")
    finally:
        if out_f:
            out_f.close()

    if not converged or n_epochs == 0:
        return None, None

    pos_ecef = filt.x[:3]
    sigma_3d = position_sigma_3d(filt.P)
    elapsed = time.time() - start_time

    # Save position
    if args.position_file:
        save_position(args.position_file, pos_ecef, sigma_3d, "ppp_bootstrap",
                       note=f"Converged in {n_epochs} epochs ({elapsed:.0f}s)")
        log.info(f"Position saved to {args.position_file}")

    lat, lon, alt = ecef_to_lla(pos_ecef[0], pos_ecef[1], pos_ecef[2])
    log.info(f"Bootstrap complete: {lat:.7f}, {lon:.7f}, {alt:.3f}m "
             f"(sigma={sigma_3d:.4f}m, {n_epochs} epochs, {elapsed:.0f}s)")

    return pos_ecef, sigma_3d


# ── Phase 2: Steady-state clock estimation + optional servo ──────────── #

def run_steady_state(obs_queue, stop_event, corrections, known_ecef, args,
                     systems, qerr_store, config_queue):
    """Run FixedPosFilter for continuous clock estimation.

    If --servo is specified, disciplines PHC using competitive error sources.
    """
    from peppar_fix import (PIServo, DisciplineScheduler, PositionWatchdog,
                            compute_error_sources, save_position)

    servo_enabled = args.servo is not None
    log.info(f"=== Phase 2: Steady state (servo={'ON' if servo_enabled else 'OFF'}) ===")

    lat, lon, alt = ecef_to_lla(known_ecef[0], known_ecef[1], known_ecef[2])
    log.info(f"Position: {lat:.7f}, {lon:.7f}, {alt:.3f}m")

    # PTP device (lazy import, only when --servo)
    ptp = None
    pps_queue = None
    if servo_enabled:
        from peppar_fix import PtpDevice
        from peppar_fix.ptp_device import PTP_PF_EXTTS

        ptp = PtpDevice(args.servo)
        caps = ptp.get_caps()
        log.info(f"PHC: {args.servo}, max_adj={caps['max_adj']} ppb")

        ptp.adjfine(0.0)
        log.info("PHC adjfine reset to 0")

        extts_channel = 0
        try:
            ptp.set_pin_function(args.pps_pin, PTP_PF_EXTTS, extts_channel)
        except OSError:
            log.info("Pin config not supported by driver (igc uses implicit mapping)")
        ptp.enable_extts(extts_channel, rising_edge=True)
        log.info(f"EXTTS enabled: pin={args.pps_pin}, channel={extts_channel}")

        # PPS event reader thread
        pps_queue = queue.Queue(maxsize=10)

        def extts_reader():
            while not stop_event.is_set():
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

    # Initialize filter
    filt = FixedPosFilter(known_ecef)
    filt.prev_clock = 0.0

    # Watchdog
    watchdog = PositionWatchdog(threshold_m=args.watchdog_threshold)

    # Servo components (only used with --servo)
    servo = None
    scheduler = None
    if servo_enabled:
        STEP_THRESHOLD_NS = 10_000
        BASE_KP = 0.3
        BASE_KI = 0.1
        GAIN_REF_SIGMA = 2.0
        GAIN_MIN_SCALE = 0.1
        GAIN_MAX_SCALE = 3.0
        CONVERGE_ERROR_NS = 500
        CONVERGE_MIN_SCALE = 2.0

        servo = PIServo(BASE_KP, BASE_KI, max_ppb=caps['max_adj'])
        scheduler = DisciplineScheduler(base_interval=1)

    # CSV output
    out_f = None
    out_w = None
    if args.out:
        mode = 'a' if os.path.exists(args.out) else 'w'
        out_f = open(args.out, mode, newline='')
        out_w = csv.writer(out_f)
        if mode == 'w':
            header = ['phase', 'timestamp', 'clock_ns', 'clock_sigma_ns',
                      'n_meas', 'rms_m']
            if servo_enabled:
                header += ['source', 'source_error_ns', 'adjfine_ppb']
            out_w.writerow(header)

    prev_t = None
    n_epochs = 0
    warmup_epochs = 20
    phase = 'warmup' if servo_enabled else 'tracking'
    prev_source = None
    adjfine_ppb = 0.0
    gain_scale = 1.0
    tmode_set = False
    start_time = time.time()

    def pps_fractional_error(phc_sec, phc_nsec):
        if phc_nsec <= 500_000_000:
            return float(phc_nsec)
        else:
            return float(phc_nsec) - 1_000_000_000

    def phc_gps_offset_s(phc_sec, phc_nsec, gps_unix_sec):
        phc_rounded = phc_sec if phc_nsec < 500_000_000 else phc_sec + 1
        return phc_rounded - gps_unix_sec

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
                    prev_t = gps_time
                    continue
                filt.predict(dt)
            prev_t = gps_time

            # EKF update
            n_used, resid, n_td = filt.update(
                observations, corrections, gps_time, clk_file=corrections)

            if n_used < 4:
                continue

            # Watchdog
            resid_rms = float(np.sqrt(np.mean(resid ** 2))) if len(resid) > 0 else 0.0
            watchdog.update(resid_rms, n_used)
            if watchdog.alarmed:
                log.warning("POSITION WATCHDOG ALARM: antenna may have moved!")
                return 'revert'  # Signal to revert to Phase 1

            dt_rx_ns = filt.x[filt.IDX_CLK] / C * 1e9
            p_clk = filt.P[filt.IDX_CLK, filt.IDX_CLK]
            dt_rx_sigma = math.sqrt(max(0, p_clk)) / C * 1e9
            n_epochs += 1

            clk_ns = dt_rx_ns
            clk_sigma_ns = dt_rx_sigma
            ts_str = gps_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:23]

            # Switch F9T to timing mode once filter is stable
            if n_epochs >= 300 and dt_rx_sigma < 100.0 and not tmode_set:
                sigma_m = dt_rx_sigma * 1e-9 * C
                if sigma_m < 0.1 and config_queue is not None:
                    tmode_msg = _build_tmode_fixed_msg(known_ecef)
                    if tmode_msg is not None:
                        config_queue.put(tmode_msg)
                        tmode_set = True
                        log.info(f"F9T -> fixed-position timing mode")

            # ── Servo path ───────────────────────────────────────────
            source_name = ''
            source_error_ns = 0.0

            if servo_enabled:
                # Get PPS event
                try:
                    phc_sec, phc_nsec = pps_queue.get(timeout=0.5)
                except queue.Empty:
                    # No PPS event — log clock-only
                    if out_w:
                        out_w.writerow([
                            'steady', ts_str, f'{clk_ns:.3f}', f'{clk_sigma_ns:.4f}',
                            n_used, f'{resid_rms:.4f}', '', '', f'{adjfine_ppb:.3f}',
                        ])
                    if n_epochs % 10 == 0:
                        log.info(f"  [{n_epochs}] clk={clk_ns:+.1f}ns "
                                 f"sigma={clk_sigma_ns:.2f}ns n={n_used} (no PPS)")
                    continue

                gps_unix_sec = int(round(gps_time.timestamp()))
                pps_error_ns = pps_fractional_error(phc_sec, phc_nsec)
                qerr_ns, _ = qerr_store.get()

                sources = compute_error_sources(
                    pps_error_ns, qerr_ns, dt_rx_ns, dt_rx_sigma)
                best = sources[0]
                source_name = best.name
                source_error_ns = best.error_ns

                # Warmup phase
                if phase == 'warmup':
                    if n_epochs >= warmup_epochs:
                        epoch_offset = phc_gps_offset_s(phc_sec, phc_nsec, gps_unix_sec)
                        log.info(f"  Warmup complete ({n_epochs} epochs, best={best})")
                        if epoch_offset != 0 or abs(best.error_ns) > STEP_THRESHOLD_NS:
                            phase = 'step'
                        else:
                            phase = 'tracking'
                            log.info("  -> tracking (no step needed)")
                    elif n_epochs % 10 == 0:
                        log.info(f"  [{n_epochs}] warmup: best={best}")
                    if out_w:
                        out_w.writerow([
                            'steady', ts_str, f'{clk_ns:.3f}', f'{clk_sigma_ns:.4f}',
                            n_used, f'{resid_rms:.4f}',
                            best.name, f'{best.error_ns:.3f}', f'{adjfine_ppb:.3f}',
                        ])
                    continue

                # Step phase
                if phase == 'step':
                    epoch_offset = phc_gps_offset_s(phc_sec, phc_nsec, gps_unix_sec)
                    total_offset_ns = epoch_offset * 1_000_000_000 + best.error_ns
                    log.info(f"  STEP: {total_offset_ns:+.0f}ns (source={best})")

                    import subprocess
                    adj_s = -total_offset_ns / 1_000_000_000
                    result = subprocess.run(
                        ['/usr/sbin/phc_ctl', args.servo, '--',
                         'adj', f'{adj_s:.9f}'],
                        capture_output=True, text=True,
                    )
                    if result.returncode == 0:
                        log.info(f"  phc_ctl adj {adj_s:.6f}s")
                    else:
                        log.error(f"  phc_ctl adj failed: {result.stderr.strip()}")

                    servo = PIServo(BASE_KP, BASE_KI, max_ppb=caps['max_adj'])
                    scheduler = DisciplineScheduler(base_interval=1)
                    phase = 'tracking'
                    time.sleep(2)
                    while not pps_queue.empty():
                        try:
                            pps_queue.get_nowait()
                        except queue.Empty:
                            break
                    continue

                # Tracking phase
                if abs(best.error_ns) > 5000 and not scheduler._converging:
                    log.warning(f"  Outlier: {best}, skipping")
                    continue

                scheduler.accumulate(best.error_ns, best.confidence_ns, best.name)

                if prev_source != best.name:
                    if prev_source is not None:
                        log.info(f"  Source: {prev_source} -> {best.name}")
                    prev_source = best.name

                if scheduler.should_correct():
                    avg_error, avg_confidence, n_samples = scheduler.flush()

                    gain_scale = max(GAIN_MIN_SCALE, min(GAIN_MAX_SCALE,
                                     GAIN_REF_SIGMA / avg_confidence))
                    if abs(avg_error) > CONVERGE_ERROR_NS:
                        gain_scale = max(gain_scale, CONVERGE_MIN_SCALE)

                    servo.kp = BASE_KP * gain_scale
                    servo.ki = BASE_KI * gain_scale
                    adjfine_ppb = -servo.update(avg_error, dt=float(n_samples))
                    ptp.adjfine(adjfine_ppb)

                    scheduler.update_drift_rate(time.monotonic(), adjfine_ppb)
                    scheduler.compute_adaptive_interval(avg_confidence)

                    if n_epochs % 10 == 0:
                        log.info(f"  [{n_epochs}] {best.name}: "
                                 f"err={avg_error:+.1f}ns adj={adjfine_ppb:+.1f}ppb "
                                 f"gain={gain_scale:.2f}x")
                else:
                    if n_epochs % 10 == 0:
                        log.info(f"  [{n_epochs}] {best.name}: "
                                 f"err={best.error_ns:+.1f}ns "
                                 f"coast ({scheduler.n_accumulated}/{scheduler.interval})")

            else:
                # No servo — just log clock estimate
                if n_epochs % 10 == 0:
                    log.info(f"  [{n_epochs}] {ts_str[:19]} "
                             f"clk={clk_ns:+.1f}ns +/-{clk_sigma_ns:.2f}ns "
                             f"n={n_used} rms={resid_rms:.3f}m")

            # CSV output
            if out_w:
                row = ['steady', ts_str, f'{clk_ns:.3f}', f'{clk_sigma_ns:.4f}',
                       n_used, f'{resid_rms:.4f}']
                if servo_enabled:
                    row += [source_name, f'{source_error_ns:.3f}', f'{adjfine_ppb:.3f}']
                out_w.writerow(row)

    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        if servo_enabled and ptp is not None:
            try:
                ptp.adjfine(0.0)
            except Exception:
                pass
            ptp.disable_extts(0)
            ptp.close()
        if out_f:
            out_f.close()

    elapsed = time.time() - start_time
    log.info(f"Steady state complete: {n_epochs} epochs, {elapsed:.0f}s")
    return 'done'


# ── F9T timing mode ──────────────────────────────────────────────────── #

def _build_tmode_fixed_msg(ecef):
    """Build UBX CFG-VALSET to switch F9T to fixed-position timing mode."""
    try:
        from pyubx2 import UBXMessage, SET
    except ImportError:
        return None

    x_cm = int(ecef[0] * 100)
    y_cm = int(ecef[1] * 100)
    z_cm = int(ecef[2] * 100)
    x_hp = int(round((ecef[0] * 100 - x_cm) * 100))
    y_hp = int(round((ecef[1] * 100 - y_cm) * 100))
    z_hp = int(round((ecef[2] * 100 - z_cm) * 100))

    cfg_data = [
        ("CFG_TMODE_MODE", 2),
        ("CFG_TMODE_POS_TYPE", 0),
        ("CFG_TMODE_ECEF_X", x_cm),
        ("CFG_TMODE_ECEF_Y", y_cm),
        ("CFG_TMODE_ECEF_Z", z_cm),
        ("CFG_TMODE_ECEF_X_HP", x_hp),
        ("CFG_TMODE_ECEF_Y_HP", y_hp),
        ("CFG_TMODE_ECEF_Z_HP", z_hp),
        ("CFG_TMODE_FIXED_POS_ACC", 100),
    ]
    msg = UBXMessage.config_set(7, 0, cfg_data)
    return msg.serialize()


# ── Main ─────────────────────────────────────────────────────────────── #

def run(args):
    """Main entry point: bootstrap if needed, then run steady state."""

    # Shared state
    beph = BroadcastEphemeris()
    ssr = SSRState()
    corrections = RealtimeCorrections(beph, ssr)
    obs_queue = queue.Queue(maxsize=100)
    stop_event = threading.Event()
    qerr_store = QErrStore()
    config_queue = queue.Queue(maxsize=10)

    # SIGTERM handler
    def on_signal(signum, frame):
        log.info("Signal received, shutting down")
        stop_event.set()
    signal.signal(signal.SIGTERM, on_signal)

    # Load NTRIP config
    load_ntrip_config(args)

    if not args.caster_host and not args.eph_mount:
        log.warning("No NTRIP source configured")

    # Caster stub
    if args.caster_out:
        log.warning(f"--caster {args.caster_out} requested but NTRIP caster output "
                    "is not yet implemented. Ignoring.")

    # Parse systems filter
    systems = set(args.systems.split(',')) if args.systems else None
    log.info(f"Systems: {systems}")

    # Start NTRIP threads (shared across phases)
    start_ntrip_threads(args, beph, ssr, stop_event)

    # Wait for ephemeris
    if args.eph_mount:
        if not wait_for_ephemeris(beph, stop_event):
            return 1

    # Start serial reader (shared across phases)
    serial_kwargs = {}
    if args.servo:
        serial_kwargs['qerr_store'] = qerr_store
        serial_kwargs['config_queue'] = config_queue

    t_serial = threading.Thread(
        target=serial_reader,
        args=(args.serial, args.baud, obs_queue, stop_event, beph, systems, ssr),
        kwargs=serial_kwargs,
        daemon=True,
    )
    t_serial.start()
    log.info(f"Serial: {args.serial} at {args.baud} baud")

    # Determine starting position
    known_ecef = None

    # Try loading from position file
    if args.position_file:
        known_ecef = load_position(args.position_file)
        if known_ecef is not None:
            lat, lon, alt = ecef_to_lla(known_ecef[0], known_ecef[1], known_ecef[2])
            log.info(f"Position loaded from {args.position_file}: "
                     f"{lat:.7f}, {lon:.7f}, {alt:.3f}m")

    # Phase 1: Bootstrap if no known position
    while known_ecef is None and not stop_event.is_set():
        pos, sigma = run_bootstrap(obs_queue, stop_event, corrections, beph, args, systems)
        if pos is None:
            log.error("Bootstrap failed — no position converged")
            stop_event.set()
            return 1
        known_ecef = pos

    if stop_event.is_set():
        return 1

    # Phase 2: Steady state (may revert to Phase 1 on watchdog alarm)
    while not stop_event.is_set():
        result = run_steady_state(
            obs_queue, stop_event, corrections, known_ecef, args,
            systems, qerr_store, config_queue)

        if result == 'revert':
            log.warning("Reverting to Phase 1 (position bootstrap)")
            # Clear stale position file
            if args.position_file and os.path.exists(args.position_file):
                os.remove(args.position_file)
                log.info(f"Removed stale position file: {args.position_file}")
            known_ecef = None
            pos, sigma = run_bootstrap(obs_queue, stop_event, corrections, beph, args, systems)
            if pos is None:
                log.error("Re-bootstrap failed")
                return 1
            known_ecef = pos
        else:
            break

    stop_event.set()
    return 0


# ── CLI ──────────────────────────────────────────────────────────────── #

def main():
    ap = argparse.ArgumentParser(
        prog='peppar-fix',
        description="Unified GNSS-disciplined clock: bootstrap + steady-state + optional PHC servo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
  # Bootstrap + steady-state clock logging:
  peppar-fix --serial /dev/gnss-top --ntrip-conf ntrip.conf \\
             --eph-mount BCEP00BKG0 --ssr-mount SSRA00BKG0 \\
             --position-file data/position.json --out solution.csv

  # With PHC discipline:
  peppar-fix --serial /dev/gnss-top --ntrip-conf ntrip.conf \\
             --eph-mount BCEP00BKG0 --ssr-mount SSRA00BKG0 \\
             --position-file data/position.json \\
             --servo /dev/ptp0 --pps-pin 1

  # Skip bootstrap (position already known):
  peppar-fix --serial /dev/gnss-top --ntrip-conf ntrip.conf \\
             --eph-mount BCEP00BKG0 \\
             --position-file data/position.json \\
             --servo /dev/ptp0 --pps-pin 1
""",
    )

    # Serial (required)
    ap.add_argument("--serial", required=True,
                    help="Serial port for F9T (e.g. /dev/gnss-top)")
    ap.add_argument("--baud", type=int, default=115200,
                    help="Serial baud rate (default: 115200)")

    # NTRIP corrections
    ntrip = ap.add_argument_group("NTRIP corrections")
    ntrip.add_argument("--ntrip-conf",
                       help="NTRIP config file (INI format)")
    ntrip.add_argument("--caster-host",
                       help="NTRIP caster hostname")
    ntrip.add_argument("--port", type=int, default=2101,
                       help="NTRIP port (default: 2101)")
    ntrip.add_argument("--tls", action="store_true",
                       help="Use TLS (auto for port 443)")
    ntrip.add_argument("--eph-mount",
                       help="Broadcast ephemeris mountpoint")
    ntrip.add_argument("--ssr-mount",
                       help="SSR corrections mountpoint")
    ntrip.add_argument("--user", help="NTRIP username")
    ntrip.add_argument("--password", help="NTRIP password")

    # Position
    pos = ap.add_argument_group("Position")
    pos.add_argument("--position-file", default="data/position.json",
                     help="Position file for save/load (default: data/position.json)")
    pos.add_argument("--seed-pos",
                     help="Seed position as lat,lon,alt (speeds bootstrap)")
    pos.add_argument("--sigma", type=float, default=0.1,
                     help="Bootstrap convergence threshold in meters (default: 0.1)")
    pos.add_argument("--timeout", type=int, default=3600,
                     help="Bootstrap timeout in seconds (default: 3600)")
    pos.add_argument("--watchdog-threshold", type=float, default=0.5,
                     help="Position watchdog threshold in meters (default: 0.5)")

    # PHC servo (optional)
    phc = ap.add_argument_group("PHC servo (optional)")
    phc.add_argument("--servo", default=None, metavar="PTP_DEV",
                     help="PTP device to discipline (e.g. /dev/ptp0)")
    phc.add_argument("--pps-pin", type=int, default=1,
                     help="SDP pin for PPS input (default: 1)")

    # Caster output (future)
    ap.add_argument("--caster", dest="caster_out", default=None, metavar="PORT",
                    help="NTRIP caster output port (future, not yet implemented)")

    # General
    ap.add_argument("--systems", default="gps,gal,bds",
                    help="GNSS systems (default: gps,gal,bds)")
    ap.add_argument("--out", help="CSV output file")
    ap.add_argument("--duration", type=int, default=None,
                    help="Run duration in seconds (default: unlimited)")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="Debug logging")

    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
        stream=sys.stderr,
    )

    sys.exit(run(args))


if __name__ == "__main__":
    main()

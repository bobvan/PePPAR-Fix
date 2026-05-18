#!/usr/bin/env python3
"""phc_bootstrap.py — DO bootstrap via PHC API.

See docs/glossary.md for term definitions (DO, PHC, rx TCXO, etc.).

Requires a receiver state file with last_known_position.  Measures DO
frequency from PPS, runs a short PPP clock solution, then evaluates
DO phase and frequency.  Intervenes only if they disagree with
GNSS-derived estimates: optimal stopping for the best achievable
phase step, then a glide slope for smooth servo handoff.

Exit codes:
    0 — PHC is ready for servo (blessed or stepped)
    1 — fatal error (no position, no GNSS, hardware failure)
    2 — position sanity check failed (stored position may be wrong)

Usage:
    python3 phc_bootstrap.py \
        --serial /dev/gnss-top --baud 115200 --port-type USB \
        --ntrip-conf ntrip.conf --eph-mount BCEP00BKG0 \
        --systems gps,gal \
        --ptp-dev /dev/ptp_i226 --ptp-profile i226
"""

import argparse
import json
import logging
import math
import os
import queue
import signal
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
from peppar_fix.ptp_device import PtpDevice, DualEdgeFilter
from peppar_fix.receiver import ensure_receiver_ready
from solve_ppp import FixedPosFilter, ls_init

log = logging.getLogger("phc_bootstrap")
C = 299_792_458.0

# ── Helpers ──────────────────────────────────────────────────────── #




def lla_to_ecef(lat, lon, alt):
    from solve_ppp import lla_to_ecef as _lla
    return _lla(lat, lon, alt)


def ecef_to_lla(x, y, z):
    from solve_ppp import ecef_to_lla as _lla
    return _lla(x, y, z)


def _realtime_to_phc_offset_s(phc_timescale, leap, tai_minus_gps):
    """Seconds to add to CLOCK_REALTIME (UTC) to get PHC timescale.

    CLOCK_REALTIME tracks UTC.  GPS = UTC + leap.  TAI = GPS + 19 = UTC + leap + 19.
    """
    if phc_timescale == "utc":
        return 0
    if phc_timescale == "gps":
        return leap
    if phc_timescale == "tai":
        return leap + tai_minus_gps
    raise ValueError(f"Unsupported timescale: {phc_timescale}")


# ── PPS OUT (PEROUT) ─────────────────────────────────────────────── #


def _set_pin_function_sysfs(ptp_dev, pin_name, func, channel):
    """Set PTP pin function via sysfs (fallback when ioctl not supported).

    The E810 ice driver rejects PTP_PIN_SETFUNC ioctl but accepts writes
    to /sys/class/ptp/ptpN/pins/<name> in "func channel" format.
    """
    import glob
    ptp_base = os.path.basename(ptp_dev)
    pin_paths = sorted(glob.glob(f"/sys/class/ptp/{ptp_base}/pins/*"))
    if pin_name.isdigit():
        idx = int(pin_name)
        if idx < len(pin_paths):
            pin_path = pin_paths[idx]
        else:
            return False
    else:
        pin_path = f"/sys/class/ptp/{ptp_base}/pins/{pin_name}"
    try:
        with open(pin_path, 'w') as f:
            f.write(f"{func} {channel}\n")
        log.info("Pin %s set to func=%d channel=%d via sysfs", pin_path, func, channel)
        return True
    except (OSError, IOError) as e:
        log.warning("sysfs pin set failed for %s: %s", pin_path, e)
        return False


# Map pin index to E810 sysfs name.
# Different kernel versions expose different names — newer ice driver
# uses SDP20-SDP23 (after the GNSS pin), older versions used SMA1/SMA2.
# We pass the index as a digit string and let _set_pin_function_sysfs
# look up the actual file via sorted glob, which works regardless of
# the names the kernel happens to use.
_E810_PIN_NAMES = {0: "0", 1: "1", 2: "2", 3: "3", 4: "4"}


# _ticc_check_perout_phase moved to peppar_fix/perout_setup.py
# (canonical home; used by phc_bootstrap, perout_kick, and
# tools/calibrate_do.py).  See dayplan: calToolPhcPerout-main.


def _enable_pps_out(ptp, args):
    """Enable PEROUT (PPS OUT) if configured.

    Must be called after any DO phase step — stepping the PHC clock
    invalidates the PEROUT alignment, stopping the output pulse.

    Wrapper around ``peppar_fix.perout_setup.setup_perout`` that adapts
    the bootstrap's argparse namespace to the helper's keyword args.
    The actual pin/PEROUT/verify chain lives in the shared module so
    ``tools/calibrate_do.py`` and ``scripts/perout_kick.py`` can use
    the same code path.  See ``dayplan: calToolPhcPerout-main``.
    """
    if args.pps_out_pin < 0:
        return
    from peppar_fix.perout_setup import setup_perout
    setup_perout(
        ptp,
        pin_index=args.pps_out_pin,
        channel=args.pps_out_channel,
        program_pin=getattr(args, 'program_pin', True),
        ptp_dev_path=getattr(args, 'ptp_dev', None),
        verify_via_ticc_port=getattr(args, 'ticc_port', None),
    )


# ── PPS frequency measurement ────────────────────────────────────── #


def measure_pps_frequency(ptp, channel, n_samples=5, timeout_s=8):
    """Measure DO frequency error from first-to-last PPS fractional second.

    Captures n_samples+1 PPS events and computes the total fractional-second
    drift over the full baseline.  With N PPS intervals (N seconds), the
    frequency error is:

        ppb = (last_nsec - first_nsec ± wrap_correction) / N

    This uses the full N-second baseline for maximum precision, rather than
    averaging individual 1-second interval measurements.

    Returns (freq_ppb, sigma_ppb, n_intervals) or (None, None, 0)
    if not enough samples.  sigma is estimated from the per-interval
    residuals after removing the measured frequency trend.
    """
    ptp.enable_extts(channel, rising_edge=True)
    # Store (elapsed_sec, nsec) relative to first PPS.  Keeps all values
    # small enough for float64 without losing nanosecond precision.
    samples = []  # (elapsed_sec, nsec)
    first_sec = None
    deadline = time.monotonic() + timeout_s
    # i226 dual-edge quirk: with a wide PPS pulse the kernel reports both
    # rising and falling edges, doubling the apparent rate and corrupting
    # the frequency estimate.  Filter the falling edges out.
    dedup = DualEdgeFilter()

    for _ in range(n_samples + 1):
        if time.monotonic() > deadline:
            break
        remaining_ms = max(50, int((deadline - time.monotonic()) * 1000))
        event = ptp.read_extts_dedup(dedup, timeout_ms=min(2000, remaining_ms))
        if event is None:
            continue
        sec, nsec, _idx, _mono, _qr, _pa = event
        if first_sec is None:
            first_sec = sec
        samples.append((sec - first_sec, nsec))

    ptp.disable_extts(channel)
    if dedup.dropped:
        log.info("PPS frequency measurement: dual-edge filter dropped %d events",
                 dedup.dropped)

    if len(samples) < 2:
        return None, None, 0

    # Full-baseline frequency: fractional-second drift over N PPS intervals.
    n_intervals = samples[-1][0]  # elapsed integer seconds
    if n_intervals <= 0:
        return None, None, 0

    # Fractional-second drift, handling wrap through zero.
    nsec_drift = samples[-1][1] - samples[0][1]
    if nsec_drift > 500_000_000:
        nsec_drift -= 1_000_000_000
        n_intervals += 1
    elif nsec_drift < -500_000_000:
        nsec_drift += 1_000_000_000
        n_intervals -= 1

    freq_ppb = nsec_drift / n_intervals

    # Estimate per-interval jitter from detrended residuals.
    if len(samples) >= 3:
        residuals = []
        for elapsed_s, nsec in samples[1:]:
            if elapsed_s <= 0:
                continue
            predicted_nsec = samples[0][1] + freq_ppb * elapsed_s
            actual_nsec = nsec
            # Handle fractional-second wrap in the comparison
            diff = actual_nsec - predicted_nsec
            if diff > 500_000_000:
                diff -= 1_000_000_000
            elif diff < -500_000_000:
                diff += 1_000_000_000
            residuals.append(diff)
        import statistics
        sigma_ppb = statistics.stdev(residuals) if len(residuals) >= 2 else 0.0
    else:
        sigma_ppb = 0.0

    return (freq_ppb, sigma_ppb, len(samples) - 1)


# ── Main ─────────────────────────────────────────────────────────── #


def _apply_bootstrap_profile(args):
    """Apply step parameters from PTP profile in config/receivers.toml."""
    import tomllib

    # Find config file
    config_path = args.device_config
    if config_path is None:
        for candidate in [
            os.path.join(os.path.dirname(__file__), "..", "config", "receivers.toml"),
            os.path.join(os.path.dirname(__file__), "config", "receivers.toml"),
        ]:
            if os.path.exists(candidate):
                config_path = candidate
                break
    if config_path is None or not os.path.exists(config_path):
        log.warning("PTP profile config not found (tried config/receivers.toml)")
        return

    try:
        with open(config_path, "rb") as f:
            cfg = tomllib.load(f)
    except Exception as e:
        log.warning("Failed to load PTP profile config: %s", e)
        return

    profile = cfg.get("ptp", {}).get(args.ptp_profile)
    if not profile:
        log.warning("PTP profile '%s' not found in %s", args.ptp_profile, config_path)
        return

    log.info("Applying PTP profile '%s' from %s", args.ptp_profile, config_path)

    # Step parameters — only apply if the user didn't override on CLI
    if args.phc_settime_lag_ns == 0:
        args.phc_settime_lag_ns = profile.get("phc_settime_lag_ns", args.phc_settime_lag_ns)
    if args.phc_step_threshold_ns == 10000:
        args.phc_step_threshold_ns = profile.get("phc_step_threshold_ns", args.phc_step_threshold_ns)

    # Pin/EXTTS parameters
    if args.pps_pin is None:
        args.pps_pin = profile.get("pps_pin", args.pps_pin)
    if args.extts_channel == 0:
        args.extts_channel = profile.get("extts_channel", args.extts_channel)
    if not args.program_pin:
        args.program_pin = bool(profile.get("program_pin", False))
    if args.phc_timescale == "tai":
        args.phc_timescale = profile.get("timescale", args.phc_timescale)
    if args.pps_out_pin == -1:
        args.pps_out_pin = profile.get("pps_out_pin", args.pps_out_pin)
    if args.pps_out_channel == 0:
        args.pps_out_channel = profile.get("pps_out_channel", args.pps_out_channel)

    # Servo gains for glide frequency computation
    if args.track_kp == 0.01:
        args.track_kp = profile.get("track_kp", args.track_kp)
    if args.track_ki == 0.001:
        args.track_ki = profile.get("track_ki", args.track_ki)
    if args.glide_zeta == 0.7:
        args.glide_zeta = profile.get("glide_zeta", args.glide_zeta)
    if args.track_max_ppb == 0:
        args.track_max_ppb = profile.get("track_max_ppb", 100000.0)
    if args.phc_optimal_stop_limit_s == 1.0:
        args.phc_optimal_stop_limit_s = profile.get("phc_optimal_stop_limit_s", args.phc_optimal_stop_limit_s)

    # ClockMatrix I2C parameters (optional — only on OTC hardware)
    cm_bus = profile.get("clockmatrix_bus")
    if cm_bus is not None:
        args.clockmatrix_bus = cm_bus
        args.clockmatrix_addr = profile.get("clockmatrix_addr", "0x58")
        args.clockmatrix_dpll_actuator = profile.get(
            "clockmatrix_dpll_actuator", 3)

    log.info("  phc_settime_lag_ns=%d phc_step_threshold_ns=%d phc_optimal_stop_limit=%.1fs",
             args.phc_settime_lag_ns, args.phc_step_threshold_ns, args.phc_optimal_stop_limit_s)
    log.info("  track_kp=%.4f track_ki=%.4f glide_zeta=%.2f track_max=%.0f ppb",
             args.track_kp, args.track_ki, args.glide_zeta, args.track_max_ppb)
    log.info("  phc_optimal_stop_limit=%.1fs", args.phc_optimal_stop_limit_s)


def main():
    ap = argparse.ArgumentParser(description="PHC bootstrap")
    ap.add_argument("--serial", required=True)
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--port-type", default="USB",
                    choices=["UART", "UART2", "USB", "SPI", "I2C"],
                    help="Receiver port type for message routing")
    ap.add_argument("--systems", default="gps,gal")
    ap.add_argument("--ntrip-conf", required=True)
    ap.add_argument("--eph-mount", required=True)
    ap.add_argument("--ssr-mount", default=None)
    ap.add_argument("--ptp-dev", required=True)
    ap.add_argument("--ptp-profile", default=None,
                    help="PTP profile name from config/receivers.toml (e.g. i226, e810)")
    ap.add_argument("--device-config", default=None,
                    help="Path to receivers.toml (default: auto-detect)")
    ap.add_argument("--extts-channel", type=int, default=0)
    ap.add_argument("--pps-pin", type=int, default=None)
    ap.add_argument("--pps-out-pin", type=int, default=-1,
                    help="SDP pin for PPS OUT (PEROUT), -1 = none")
    ap.add_argument("--pps-out-channel", type=int, default=0,
                    help="PEROUT channel for PPS OUT")
    ap.add_argument("--ticc-port", default=None,
                    help="TICC serial port for PEROUT phase verification")
    ap.add_argument("--program-pin", action="store_true")
    ap.add_argument("--phc-timescale", default="tai",
                    choices=["gps", "utc", "tai"])
    ap.add_argument("--leap", type=int, default=18)
    ap.add_argument("--tai-minus-gps", type=int, default=19)
    ap.add_argument("--epochs", type=int, default=10,
                    help="Number of filter epochs before evaluating PHC")
    ap.add_argument("--freq-tolerance-ppb", type=float, default=10.0,
                    help="Frequency sanity threshold in ppb")
    ap.add_argument("--phc-step-threshold-ns", type=int, default=10000,
                    help="Expected step accuracy (ns) — skip bootstrap if phase error already within this")
    ap.add_argument("--phc-settime-lag-ns", type=int, default=0,
                    help="Mean clock_settime-to-PHC landing lag in ns (aim correction)")
    ap.add_argument("--max-pps-iterations", type=int, default=8,
                    help="Max PPS feedback iterations for step convergence (default: 8)")
    ap.add_argument("--position-check-m", type=float, default=100.0,
                    help="Max acceptable LS-vs-stored position delta in meters")
    ap.add_argument("--phc-optimal-stop-limit-s", type=float, default=1.0,
                    help="Phase step optimal stopping search budget in seconds")
    ap.add_argument("--glide-zeta", type=float, default=0.7,
                    help="Target damping ratio for servo glide (0.5-1.0)")
    ap.add_argument("--no-glide", action="store_true",
                    help="Skip glide slope — set adjfine to base frequency only "
                         "(for freerun characterization)")
    ap.add_argument("--track-kp", type=float, default=0.01,
                    help="Servo Kp for glide computation (overridden by profile)")
    ap.add_argument("--track-ki", type=float, default=0.001,
                    help="Servo Ki for glide computation (overridden by profile)")
    ap.add_argument("--track-max-ppb", type=float, default=0,
                    help="Servo max adjfine — glide is clamped to this (from profile)")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")

    # Apply PTP profile defaults for step parameters
    if args.ptp_profile:
        _apply_bootstrap_profile(args)

    # Ensure receiver is producing dual-frequency observations.
    # Done first so we have the receiver identity for position loading.
    systems = set(args.systems.split(","))
    import os as _os
    _base = _os.path.basename(args.serial)
    _is_kernel_gnss = _base.startswith("gnss") and _base[4:].isdigit()
    _sfrbx = 0 if _is_kernel_gnss else getattr(args, 'sfrbx_rate', 1)
    _rate_ms = 2000 if _is_kernel_gnss else getattr(args, 'measurement_rate_ms', 1000)
    driver, _identity = ensure_receiver_ready(
        args.serial, args.baud, port_type=args.port_type, systems=systems,
        sfrbx_rate=_sfrbx, measurement_rate_ms=_rate_ms)
    if driver is None:
        log.error("Receiver not producing dual-frequency observations — "
                  "cannot proceed. See docs/receiver-signals.md.")
        return 1
    log.info("Receiver ready: %s", driver.name)

    # Load position — receiver state is primary, legacy file is fallback.
    from peppar_fix.receiver_state import load_position_from_receiver
    known_ecef = None
    receiver_uid = _identity.get("unique_id") if _identity else None
    if receiver_uid is not None:
        known_ecef = load_position_from_receiver(receiver_uid)
        if known_ecef is not None:
            lat, lon, alt = ecef_to_lla(*known_ecef)
            log.info("Loaded position (receiver state): %.6f, %.6f, %.1fm", lat, lon, alt)
    if known_ecef is None:
        log.error("No position available — run position bootstrap first")
        return 1

    # Load drift — DO state is primary, legacy file is fallback
    drift = None
    try:
        from peppar_fix.do_state import load_drift_from_state
        drift = load_drift_from_state(args.ptp_dev)
        if drift:
            log.info("Drift (DO state): adjfine=%.1f ppb (from %s)",
                     drift["adjfine_ppb"], drift.get("timestamp", "?"))
    except Exception:
        pass
    if drift is None:
        log.info("No DO state found — bootstrapping from scratch")

    # Open PHC
    ptp = PtpDevice(args.ptp_dev)
    caps = ptp.get_caps()
    log.info("PHC: %s, max_adj=%d ppb", args.ptp_dev, caps["max_adj"])

    if args.pps_pin is not None:
        from peppar_fix.ptp_device import PTP_PF_EXTTS
        # Try ioctl first (i226), fall back to sysfs (E810 ice driver
        # rejects PTP_PIN_SETFUNC ioctl but accepts sysfs writes).
        pin_set = False
        if args.program_pin:
            try:
                ptp.set_pin_function(args.pps_pin, PTP_PF_EXTTS, args.extts_channel)
                log.info("Pin %d programmed for EXTTS channel %d via ioctl",
                         args.pps_pin, args.extts_channel)
                pin_set = True
            except OSError as e:
                log.warning("EXTTS ioctl failed: %s", e)
        if not pin_set:
            pin_name = _E810_PIN_NAMES.get(args.pps_pin, str(args.pps_pin))
            if _set_pin_function_sysfs(args.ptp_dev, pin_name,
                                       PTP_PF_EXTTS, args.extts_channel):
                log.info("Pin %s programmed for EXTTS channel %d via sysfs",
                         pin_name, args.extts_channel)

    # Read current PHC time
    try:
        phc_ns, sys_ns = ptp.read_phc_ns()
        log.info("PHC read OK: %d ns", phc_ns)
    except OSError as e:
        log.error("Cannot read PHC: %s", e)
        ptp.close()
        return 1

    # Measure PPS frequency
    log.info("Measuring DO frequency from PPS intervals...")
    pps_freq_ppb, pps_freq_sigma, pps_freq_n = measure_pps_frequency(
        ptp, args.extts_channel, n_samples=args.epochs, timeout_s=args.epochs + 3)
    if pps_freq_ppb is not None:
        pps_freq_unc = pps_freq_sigma / math.sqrt(pps_freq_n)
        log.info("PPS frequency error: %.1f ±%.1f ppb (σ=%.1f, n=%d)",
                 pps_freq_ppb, pps_freq_unc, pps_freq_sigma, pps_freq_n)
    else:
        pps_freq_unc = None
        log.error("No PPS events received — check EXTTS wiring, pin config, and PTP device")
        ptp.close()
        return 1

    # Start NTRIP and observation infrastructure
    from realtime_ppp import serial_reader, ntrip_reader
    from ntrip_client import NtripStream
    from broadcast_eph import BroadcastEphemeris
    from ssr_corrections import SSRState, RealtimeCorrections

    beph = BroadcastEphemeris()
    ssr = SSRState()
    corrections = RealtimeCorrections(beph, ssr)
    obs_queue = queue.Queue(maxsize=20)
    stop_event = threading.Event()

    # Parse NTRIP config
    import configparser
    nc = configparser.ConfigParser()
    nc.read(args.ntrip_conf)
    s = nc["ntrip"]
    ntrip_caster = s.get("caster")
    ntrip_port = int(s.get("port", 2101))
    ntrip_user = s.get("user", "")
    ntrip_password = s.get("password", "")
    ntrip_tls = s.get("tls", "false").lower() == "true"
    ssr_mount = s.get("mount", args.ssr_mount)

    # Start ephemeris stream
    eph_stream = NtripStream(
        caster=ntrip_caster, port=ntrip_port,
        mountpoint=args.eph_mount,
        user=ntrip_user, password=ntrip_password, tls=ntrip_tls)
    t_eph = threading.Thread(
        target=ntrip_reader,
        args=(eph_stream, beph, ssr, stop_event, "EPH"),
        daemon=True)
    t_eph.start()
    log.info("Ephemeris stream: %s:%d/%s", ntrip_caster, ntrip_port, args.eph_mount)

    # Start SSR stream if configured
    if ssr_mount:
        ssr_stream = NtripStream(
            caster=ntrip_caster, port=ntrip_port,
            mountpoint=ssr_mount,
            user=ntrip_user, password=ntrip_password, tls=ntrip_tls)
        t_ssr = threading.Thread(
            target=ntrip_reader,
            args=(ssr_stream, beph, ssr, stop_event, "SSR"),
            daemon=True)
        t_ssr.start()

    # Wait for broadcast ephemeris
    log.info("Waiting for broadcast ephemeris...")
    for _ in range(60):
        if stop_event.is_set():
            break
        by_sys = {}
        for prn in beph.satellites:
            by_sys[prn[0]] = by_sys.get(prn[0], 0) + 1
        if by_sys.get('G', 0) >= 8:
            break
        time.sleep(1)
    else:
        log.error("Timeout waiting for broadcast ephemeris")
        ptp.close()
        return 1
    log.info("Broadcast ephemeris ready: %s", beph.summary())

    # Start observation reader (driver from ensure_receiver_ready)
    t_serial = threading.Thread(
        target=serial_reader,
        args=(args.serial, args.baud, obs_queue, stop_event, beph, systems, ssr),
        kwargs={'driver': driver},
        daemon=True,
    )
    t_serial.start()

    # Run FixedPosFilter for N epochs
    filt = FixedPosFilter(known_ecef)
    prev_t = None
    n_epochs = 0
    dt_rx_ns = None
    dt_rx_sigma_ns = None

    log.info("Running PPP clock solution for %d epochs...", args.epochs)
    dt_rx_series = []  # collect for inter-oscillator drift estimation
    for _ in range(args.epochs * 3):  # allow some dropped epochs
        try:
            gps_time, observations = obs_queue.get(timeout=5)
        except queue.Empty:
            continue

        if prev_t is not None:
            dt = (gps_time - prev_t).total_seconds()
            if dt <= 0 or dt > 30:
                prev_t = gps_time
                continue
            filt.predict(dt)
        prev_t = gps_time

        n_used, resid, n_td = filt.update(
            observations, corrections, gps_time, clk_file=corrections)
        if n_used < 4:
            continue

        dt_rx_ns = filt.x[filt.IDX_CLK] / C * 1e9
        p_clk = filt.P[filt.IDX_CLK, filt.IDX_CLK]
        dt_rx_sigma_ns = math.sqrt(max(0, p_clk)) / C * 1e9
        dt_rx_series.append(dt_rx_ns)
        n_epochs += 1

        if n_epochs % 5 == 0 or n_epochs == args.epochs:
            log.info("  [%d/%d] dt_rx=%.1f ±%.1f ns  n_used=%d",
                     n_epochs, args.epochs, dt_rx_ns, dt_rx_sigma_ns, n_used)

        if n_epochs >= args.epochs:
            break

    stop_event.set()

    if dt_rx_ns is None:
        log.error("Filter did not converge in %d epochs", args.epochs)
        ptp.close()
        return 1

    log.info("Clock estimate: dt_rx=%.1f ±%.1f ns after %d epochs",
             dt_rx_ns, dt_rx_sigma_ns, n_epochs)

    # Sanity check position via LS
    # (skipped for now — the filter ran with the stored position,
    #  large residuals would indicate a bad position)

    # ── Evaluate PHC ──────────────────────────────────────────────── #

    # Read PHC phase at the most recent PPS edge.  read_one_rising_edge
    # collects events for one full PPS period and applies dual-edge
    # filtering so the returned event is a confirmed rising edge — never
    # the i226's spurious falling-edge timestamp.
    ptp.enable_extts(args.extts_channel, rising_edge=True)
    pps_event = ptp.read_one_rising_edge(timeout_s=3.0)
    pps_realtime_ns = time.clock_gettime_ns(time.CLOCK_REALTIME)
    ptp.disable_extts(args.extts_channel)

    if pps_event is None:
        log.error("No PPS event received — cannot evaluate DO phase")
        ptp.close()
        return 1

    phc_sec, phc_nsec, _idx, _mono, _qr, _pa = pps_event
    phc_rounded_sec = phc_sec if phc_nsec < 500_000_000 else phc_sec + 1

    # Determine which second this PPS belongs to using CLOCK_REALTIME.
    # PPS fires on the second boundary; CLOCK_REALTIME (UTC via NTP) tells
    # us which one — avoids stale gps_time from the filter loop.
    offset_s = _realtime_to_phc_offset_s(
        args.phc_timescale, args.leap, args.tai_minus_gps)
    utc_sec = round(pps_realtime_ns / 1_000_000_000)
    target_sec = utc_sec + offset_s

    epoch_offset = phc_rounded_sec - target_sec
    pps_error_ns = phc_nsec if phc_nsec < 500_000_000 else phc_nsec - 1_000_000_000

    # Total phase error including whole-second offset
    phase_error_ns = epoch_offset * 1_000_000_000 + pps_error_ns

    log.info("DO phase: epoch_offset=%ds, pps_error=%+.0f ns, "
             "total_phase_error=%+.0f ns (realtime_utc=%d, target=%d)",
             epoch_offset, pps_error_ns, phase_error_ns, utc_sec, target_sec)

    # Frequency sanity check — PPS measurement alone is sufficient.
    # pps_freq_ppb is the PHC's total frequency error (crystal + current
    # adjfine) as seen by PPS intervals.  If it's outside tolerance, the
    # DO frequency needs correction regardless of whether we have a drift
    # file.
    freq_sane = True
    if pps_freq_ppb is not None:
        if abs(pps_freq_ppb) > args.freq_tolerance_ppb:
            log.warning("Frequency error: PPS=%.1f ±%.1f ppb — outside ±%.1f ppb",
                        pps_freq_ppb, pps_freq_unc, args.freq_tolerance_ppb)
            freq_sane = False
        else:
            log.info("Frequency sane: PPS=%.1f ±%.1f ppb (within ±%.1f ppb)",
                     pps_freq_ppb, pps_freq_unc, args.freq_tolerance_ppb)

    # Phase sanity check
    phase_sane = abs(phase_error_ns) < args.phc_step_threshold_ns

    if phase_sane and freq_sane:
        log.info("PHC state is sane — blessing without intervention")
        log.info("  Phase error: %+.0f ns (within %d ns)",
                 phase_error_ns, args.phc_step_threshold_ns)
        _enable_pps_out(ptp, args)
        ptp.close()
        return 0

    # ── Intervene ─────────────────────────────────────────────────── #
    #
    # Three steps:
    #
    # 1. Phase step — use optimal stopping to get the best achievable
    #    DO phase within a fixed search budget.  The step error has a
    #    log-normal distribution (fixed minimum kernel path + multiplicative
    #    scheduling jitter).  Optimal stopping learns the distribution
    #    during the first 37% of the budget, then accepts the first
    #    result at or below the 5th percentile.
    #
    # 2. PPS measurement — capture one PPS event to measure the true
    #    residual φ₀ (the readback used by optimal stopping has a
    #    systematic bias from PTP_SYS_OFFSET asymmetry; PPS is truth).
    #
    # 3. Glide slope — set a DO frequency that drives φ₀ toward zero
    #    at the rate the servo expects for a near-critically-damped
    #    handoff:  dφ/dt₀ = -ζ·ωₙ·φ₀  where ωₙ = √Ki.

    # Compute base frequency (the on-rate adjfine, excluding transient glide)
    current_adj = ptp.read_adjfine()
    if not freq_sane:
        if pps_freq_ppb is not None and pps_freq_unc is not None:
            if drift and pps_freq_unc > args.freq_tolerance_ppb:
                base_freq = drift["adjfine_ppb"]
                freq_source = ("drift file (PPS sigma/sqrt(n)=%.1f ppb too high)"
                               % pps_freq_unc)
            else:
                base_freq = current_adj - pps_freq_ppb
                freq_source = ("PPS correction: %.1f - %.1f"
                               % (current_adj, pps_freq_ppb))
        elif drift:
            base_freq = drift["adjfine_ppb"]
            freq_source = "drift file"
        else:
            base_freq = 0.0
            freq_source = "default (no data)"
        log.info("Base frequency: %.1f ppb (%s)", base_freq, freq_source)
    else:
        base_freq = current_adj
        log.info("Frequency sane: base_freq = %.1f ppb", base_freq)

    # ── Measure F9T TCXO frequency correction ────────────────────── #
    #
    # The PHC and F9T use different oscillators.  We measure both rates
    # independently against GPS time and store them separately:
    #   phc_freq_corr = base_freq (the adjfine that keeps PHC on GPS rate)
    #   tcxo_freq_corr = slope of dt_rx series (F9T TCXO offset from GPS)
    #
    # The engine combines them: D = phc_freq_corr - tcxo_freq_corr
    # to correct the Carrier Phase servo for the oscillator differential.
    tcxo_freq_corr_ppb = None
    if len(dt_rx_series) >= 3:
        # Linear regression on dt_rx series → TCXO rate in ppb
        n_dt = len(dt_rx_series)
        sx = sum(range(n_dt))
        sy = sum(dt_rx_series)
        sxy = sum(i * v for i, v in enumerate(dt_rx_series))
        sxx = sum(i * i for i in range(n_dt))
        denom = n_dt * sxx - sx * sx
        if denom != 0:
            tcxo_freq_corr_ppb = (n_dt * sxy - sx * sy) / denom  # ns/epoch = ppb
            log.info("F9T TCXO freq correction: %.1f ppb (from %d dt_rx samples)",
                     tcxo_freq_corr_ppb, n_dt)

    phi_0 = phase_error_ns
    did_step = False

    if not phase_sane:
        # Step: apply the PPS-measured phase error as a relative correction.
        # ADJ_SETOFFSET is precise on both i226 and E810 — no readback or
        # system clock cross-referencing needed.  Verify via the next PPS.
        #
        # i226 PEROUT safety: disable any pre-existing PEROUT before stepping.
        # Per kernel netdev consensus, stepping the PHC while PEROUT is active
        # causes the hardware to oscillate at 62.5 MHz or lock up.  The
        # resulting corrupted state persists across disable/enable cycles.
        if args.pps_out_pin >= 0:
            ptp.disable_perout(args.pps_out_channel)
            log.info("Disabled PEROUT before PHC step")
        try:
            log.info("Stepping PHC by %+.0f ns (ADJ_SETOFFSET)", -phase_error_ns)
            ptp.adj_setoffset(-phase_error_ns)
            did_step = True
        except OSError as e:
            log.warning("ADJ_SETOFFSET failed (%s), falling back to optimal stopping",
                        e)
            pps_anchor_ns = target_sec * 1_000_000_000
            log.info("Stepping PHC (optimal_stop, limit=%.1fs, lag=%d ns)",
                     args.phc_optimal_stop_limit_s, args.phc_settime_lag_ns)
            residual, attempts, met = ptp.step_to(
                pps_anchor_ns=pps_anchor_ns,
                pps_realtime_ns=pps_realtime_ns,
                phc_optimal_stop_limit_s=args.phc_optimal_stop_limit_s,
                phc_settime_lag_ns=args.phc_settime_lag_ns,
            )
            log.info("Step: residual=%+.0f ns, attempts=%d, %s (limit=%.1fs)",
                     residual, attempts, "ACCEPTED" if met else "DEADLINE",
                     args.phc_optimal_stop_limit_s)
            did_step = True

        # Verify: measure true residual φ₀ from the next PPS edge.
        # read_one_rising_edge filters out the i226 dual-edge falling
        # timestamp; see DualEdgeFilter and read_one_rising_edge docstrings.
        ptp.enable_extts(args.extts_channel, rising_edge=True)
        evt = ptp.read_one_rising_edge(timeout_s=3.0)
        if evt is not None:
            v_realtime_ns = time.clock_gettime_ns(time.CLOCK_REALTIME)
            v_sec, v_nsec = evt[0], evt[1]
            v_rounded = v_sec if v_nsec < 500_000_000 else v_sec + 1
            v_target = round(v_realtime_ns / 1_000_000_000) + offset_s
            v_epoch_off = v_rounded - v_target
            v_sub_ns = v_nsec if v_nsec < 500_000_000 else v_nsec - 1_000_000_000
            phi_0 = v_epoch_off * 1_000_000_000 + v_sub_ns
            log.info("PPS verify: phi_0 = %+.0f ns (epoch_offset=%d)",
                     phi_0, v_epoch_off)
        else:
            log.warning("No PPS event — assuming step landed at 0")
            phi_0 = 0
        ptp.disable_extts(args.extts_channel)
    else:
        log.info("Phase OK (%+.0f ns) — frequency-only correction", phase_error_ns)

    # Step 3: Glide slope — set frequency to drive φ₀ toward zero.
    glide_offset = 0.0
    if args.no_glide:
        log.info("Glide disabled (--no-glide): adjfine = base frequency only")
    elif not phase_sane and args.track_ki > 0:
        omega_n = math.sqrt(args.track_ki)
        zeta = args.glide_zeta
        glide_offset = -zeta * omega_n * phi_0
        # Clamp so |base + glide| stays within servo's track_max_ppb.
        # If the glide exceeds control authority, the servo saturates and
        # the integral winds up, destroying the smooth handoff.
        max_glide = args.track_max_ppb - abs(base_freq)
        if abs(glide_offset) > max_glide:
            clamped = math.copysign(max_glide, glide_offset)
            log.warning("Glide clamped: %.0f → %.0f ppb (track_max=%.0f)",
                        glide_offset, clamped, args.track_max_ppb)
            glide_offset = clamped
        t_cross = abs(phi_0 / glide_offset) if glide_offset != 0 else float('inf')
        log.info("Glide: zeta=%.2f, omega_n=%.4f rad/s, phi_0=%+.0f ns, "
                 "offset=%+.1f ppb, zero-crossing ~%.0fs",
                 zeta, omega_n, phi_0, glide_offset, t_cross)

    target_freq = base_freq + glide_offset

    # ── ClockMatrix handoff ──────────────────────────────────────── #
    #
    # On Timebeat OTC hardware, the ClockMatrix drives the i226's 25 MHz.
    # The DO counts these cycles. We set the DO phase above (step) and
    # now transfer the frequency correction from PHC adjfine to the
    # ClockMatrix FCW, so the engine can steer the clock tree directly.
    #
    # 1. Read the ClockMatrix's current frequency state (FOD_FREQ)
    #    to account for any existing offset from EEPROM or Timebeat
    # 2. Zero adjfine — PHC runs at raw 25 MHz rate
    # 3. Set DPLL_3 to write_freq mode with FCW = measured offset + glide
    #
    # If ClockMatrix is not configured (no clockmatrix_bus), fall through
    # to the normal adjfine path.

    cm_bus = getattr(args, 'clockmatrix_bus', None)
    if cm_bus is not None:
        try:
            from peppar_fix.clockmatrix import ClockMatrixI2C
            from peppar_fix.clockmatrix_actuator import ClockMatrixActuator, ppb_to_fcw

            cm_addr = int(getattr(args, 'clockmatrix_addr', '0x58'), 0)
            cm_dpll = getattr(args, 'clockmatrix_dpll_actuator', 3)
            cm_i2c = ClockMatrixI2C(cm_bus, cm_addr)

            # Read current DPLL_3 state to account for existing frequency offset
            dpll_bases = {0: 0xC3B0, 1: 0xC400, 2: 0xC438, 3: 0xC480}
            dpll_freq_bases = {0: 0xC838, 1: 0xC840, 2: 0xC848, 3: 0xC850}
            mode_reg = dpll_bases[cm_dpll] + 0x37
            freq_reg = dpll_freq_bases[cm_dpll]

            cm_mode = cm_i2c.read(mode_reg, 1)[0]
            cm_pll_mode = (cm_mode >> 3) & 0x07
            cm_fcw_data = cm_i2c.read(freq_reg, 6)
            cm_fcw_raw = int.from_bytes(cm_fcw_data, 'little')
            cm_fcw_val = cm_fcw_raw & 0x3FFFFFFFFFF
            if cm_fcw_val & (1 << 41):
                cm_fcw_val -= (1 << 42)

            log.info("ClockMatrix DPLL_%d: MODE=0x%02X (pll_mode=%d), "
                     "current FCW=%d",
                     cm_dpll, cm_mode, cm_pll_mode, cm_fcw_val)

            # The measured frequency offset (base_freq) captures the TOTAL
            # offset as seen by EXTTS: OCXO natural drift + any existing
            # ClockMatrix correction. We apply the full measured offset
            # as the FCW, regardless of what was there before.

            # Set adjfine to 0 — all frequency steering goes through FCW
            log.info("Zeroing PHC adjfine (frequency goes to ClockMatrix FCW)")
            ptp.adjfine(0.0)

            # Switch DPLL to write_freq mode and set FCW
            actuator = ClockMatrixActuator(cm_i2c, dpll_id=cm_dpll)
            actuator.setup()
            actuator.adjust_frequency_ppb(target_freq)

            log.info("ClockMatrix FCW set: %.1f ppb (base=%.1f + glide=%.1f)",
                     target_freq, base_freq, glide_offset)

            # Save to DO state
            try:
                from peppar_fix.do_state import phc_unique_id, save_do_freq_offset
                save_do_freq_offset(phc_unique_id(args.ptp_dev), base_freq)
                log.info("DO freq saved: base=%.1f ppb, dt_rx=%.1f ns",
                         base_freq, dt_rx_ns)
            except Exception as e:
                log.warning("Failed to save DO state: %s", e)

            # Don't close cm_i2c — the engine will reopen its own handle.
            # Don't teardown actuator — engine inherits write_freq mode.
            cm_i2c.close()

            _enable_pps_out(ptp, args)
            ptp.close()
            log.info("PHC bootstrap complete (ClockMatrix) — servo may start")
            return 0

        except ImportError:
            log.warning("smbus2 not available — falling back to PHC adjfine")
        except Exception as e:
            log.error("ClockMatrix handoff failed: %s — falling back to PHC adjfine", e)

    # Normal PHC-only path (TimeHat, E810, or ClockMatrix fallback)
    log.info("Setting adjfine: %.1f ppb (base=%.1f + glide=%.1f)",
             target_freq, base_freq, glide_offset)
    ptp.adjfine(target_freq)

    # Save to DO state (not the transient glide offset)
    try:
        from peppar_fix.do_state import phc_unique_id, save_do_freq_offset
        save_do_freq_offset(phc_unique_id(args.ptp_dev), base_freq)
        log.info("DO freq saved: base=%.1f ppb, dt_rx=%.1f ns",
                 base_freq, dt_rx_ns)
    except Exception as e:
        log.warning("Failed to save DO state: %s", e)

    _enable_pps_out(ptp, args)
    ptp.close()
    log.info("PHC bootstrap complete — servo may start")
    return 0


if __name__ == "__main__":
    sys.exit(main())

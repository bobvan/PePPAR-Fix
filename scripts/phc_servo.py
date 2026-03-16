#!/usr/bin/env python3
"""
phc_servo.py — PHC discipline loop for PePPAR Fix M5.

Disciplines the TimeHAT i226 PHC (/dev/ptp0) using carrier-phase
clock estimates from the PPP filter (realtime_ppp.py).

Architecture:
    F9T PPS → SDP1 → extts event (PHC timestamp of PPS edge)
    PPP filter → dt_rx (receiver clock offset from GPS time)
    PHC error = phc_timestamp - (GPS_second + dt_rx)
    PI servo → adjfine() on /dev/ptp0 to steer TCXO frequency

    Output: SDP0 → disciplined PPS (SMA J4 → TICC chA for measurement)

The servo reads PPS timestamps via the PTP_EXTTS_EVENT ioctl and
correlates them with PPP clock estimates at the same GPS second.
A PI controller drives adjfine to minimize the PHC-GPS offset.

Usage:
    python phc_servo.py --serial /dev/gnss-top --baud 9600 \\
        --known-pos '41.8430626,-88.1037190,201.671' \\
        --ntrip-conf ntrip.conf --eph-mount BCEP00BKG0 --ssr-mount SSRA00BKG0 \\
        --systems gps,gal --duration 3600 \\
        --ptp-dev /dev/ptp0 --extts-pin 1 \\
        --log servo_log.csv

    # Without NTRIP (broadcast ephemeris only, ~25m RMS floor):
    python phc_servo.py --serial /dev/gnss-top --baud 9600 \\
        --known-pos '41.8430626,-88.1037190,201.671' \\
        --caster products.igs-ip.net --port 2101 \\
        --eph-mount BCEP00BKG0 \\
        --systems gps,gal --duration 3600 \\
        --ptp-dev /dev/ptp0 --extts-pin 1
"""

import argparse
import array
import csv
import ctypes
import ctypes.util
import fcntl
import logging
import math
import os
import queue
import select
import struct
import sys
import threading
import time
from datetime import datetime, timezone, timedelta

import numpy as np

# Local imports (same scripts/ directory)
from solve_pseudorange import C, lla_to_ecef
from solve_ppp import FixedPosFilter
from ntrip_client import NtripStream
from broadcast_eph import BroadcastEphemeris
from ssr_corrections import SSRState, RealtimeCorrections
from realtime_ppp import serial_reader, ntrip_reader

log = logging.getLogger("phc_servo")


# ── PTP ioctl constants ─────────────────────────────────────────────────── #
# From linux/ptp_clock.h

PTP_CLK_MAGIC = ord('=')

# ioctl number encoding (Linux _IOC macro)
_IOC_WRITE = 1
_IOC_READ = 2

def _IOC(direction, typ, nr, size):
    return (direction << 30) | (size << 16) | (typ << 8) | nr

def _IOR(typ, nr, size):
    return _IOC(_IOC_READ, typ, nr, size)

def _IOW(typ, nr, size):
    return _IOC(_IOC_WRITE, typ, nr, size)

# struct ptp_extts_request { unsigned int index; unsigned int flags; unsigned int rsv[2]; }
PTP_EXTTS_REQUEST = _IOW(PTP_CLK_MAGIC, 2, 16)
PTP_EXTTS_REQUEST2 = _IOW(PTP_CLK_MAGIC, 11, 16)

# struct ptp_extts_event { ptp_clock_time t; unsigned int index; unsigned int flags; }
# ptp_clock_time = { __s64 sec; __u32 nsec; __u32 reserved; } = 16 bytes
# Full event = 16 + 4 + 4 = 24? No — kernel ptp_extts_event is:
#   struct ptp_clock_time t (16 bytes) + unsigned int index (4) + unsigned int flags (4) = 24
# But older kernels: t(16) + index(4) = 20, no flags field. Read 32 to be safe.
PTP_EXTTS_EVENT_SIZE = 32

# struct ptp_clock_caps (80 bytes)
PTP_CLOCK_GETCAPS = _IOR(PTP_CLK_MAGIC, 1, 80)

# struct ptp_pin_desc { char name[64]; unsigned int index, func, chan; unsigned int rsv[5]; } = 96 bytes
PTP_PIN_SETFUNC = _IOW(PTP_CLK_MAGIC, 7, 96)

PTP_ENABLE_FEATURE = (1 << 0)
PTP_RISING_EDGE = (1 << 1)

PTP_PF_NONE = 0
PTP_PF_EXTTS = 1
PTP_PF_PEROUT = 2

# clock_adjtime constants
ADJ_FREQUENCY = 0x0002
ADJ_SETOFFSET = 0x0100
ADJ_NANO = 0x2000

# Clock ID encoding for /dev/ptp FDs
def _clock_id_from_fd(fd):
    """Encode PTP device fd as clockid_t for clock_adjtime."""
    return (~fd << 3) | 3


# ── PTP device wrapper ──────────────────────────────────────────────────── #

class PtpDevice:
    """Low-level interface to a Linux PTP hardware clock."""

    def __init__(self, dev_path="/dev/ptp0"):
        self.path = dev_path
        self.fd = os.open(dev_path, os.O_RDWR)
        self.clock_id = _clock_id_from_fd(self.fd)
        self._libc = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)

    def close(self):
        os.close(self.fd)

    def get_caps(self):
        """Query PTP clock capabilities."""
        buf = array.array('b', b'\x00' * 80)
        fcntl.ioctl(self.fd, PTP_CLOCK_GETCAPS, buf, True)
        raw = buf.tobytes()
        max_adj = struct.unpack_from('<i', raw, 0)[0]
        n_ext_ts = struct.unpack_from('<i', raw, 12)[0]
        n_per_out = struct.unpack_from('<i', raw, 16)[0]
        n_pins = struct.unpack_from('<i', raw, 24)[0]
        return {
            'max_adj': max_adj,
            'n_ext_ts': n_ext_ts,
            'n_per_out': n_per_out,
            'n_pins': n_pins,
        }

    def set_pin_function(self, pin_index, func, channel):
        """Configure an SDP pin (EXTTS, PEROUT, or NONE)."""
        # struct ptp_pin_desc: name[64] + index + func + chan + rsv[5]
        buf = bytearray(96)
        struct.pack_into('<64sIII', buf, 0, b'', pin_index, func, channel)
        fcntl.ioctl(self.fd, PTP_PIN_SETFUNC, bytes(buf))

    def enable_extts(self, channel, rising_edge=True):
        """Enable external timestamp capture on a channel."""
        flags = PTP_ENABLE_FEATURE
        if rising_edge:
            flags |= PTP_RISING_EDGE
        buf = struct.pack('<IIII', channel, flags, 0, 0)
        try:
            fcntl.ioctl(self.fd, PTP_EXTTS_REQUEST2, buf)
        except OSError:
            # Fall back to legacy ioctl
            fcntl.ioctl(self.fd, PTP_EXTTS_REQUEST, buf)

    def disable_extts(self, channel):
        """Disable external timestamp capture."""
        buf = struct.pack('<IIII', channel, 0, 0, 0)
        try:
            fcntl.ioctl(self.fd, PTP_EXTTS_REQUEST2, buf)
        except OSError:
            fcntl.ioctl(self.fd, PTP_EXTTS_REQUEST, buf)

    def read_extts(self, timeout_ms=1500):
        """Read one external timestamp event. Returns (sec, nsec, index) or None."""
        r, _, _ = select.select([self.fd], [], [], timeout_ms / 1000.0)
        if not r:
            return None
        data = os.read(self.fd, PTP_EXTTS_EVENT_SIZE)
        if len(data) < 20:  # minimum: ptp_clock_time(16) + index(4)
            return None
        # ptp_clock_time: s64 sec (8) + u32 nsec (4) + u32 reserved (4) = 16 bytes
        # then: u32 index (4)
        sec, nsec, _reserved, index = struct.unpack_from('<qIII', data, 0)
        return (sec, nsec, index)

    def adjfine(self, ppb):
        """Adjust PHC frequency by ppb (parts per billion).

        Uses clock_adjtime with ADJ_FREQUENCY. The kernel Timex.freq field
        is in units of 2^-16 ppm = 1/65536 ppm ≈ 0.0153 ppb.
        """
        # Convert ppb to scaled ppm (Timex.freq units)
        freq = int(ppb * 65.536)
        # Timex struct (simplified): modes(u32) + padding + offset(i64) + freq(i64) + ...
        # Full struct is 208 bytes on 64-bit Linux
        # We only set modes and freq, rest is zero
        timex_size = 208
        buf = bytearray(timex_size)
        # modes at offset 0 (unsigned int, 4 bytes)
        struct.pack_into('<I', buf, 0, ADJ_FREQUENCY)
        # freq at offset 16 on 64-bit: modes(4) + pad(4) + offset(8) + freq(8)
        struct.pack_into('<q', buf, 16, freq)

        ret = self._libc.clock_adjtime(
            ctypes.c_int32(self.clock_id),
            ctypes.c_char_p(bytes(buf)),
        )
        if ret < 0:
            errno = ctypes.get_errno()
            raise OSError(errno, f"clock_adjtime failed: {os.strerror(errno)}")
        return ppb

    def step_time(self, offset_ns):
        """Step the PHC by offset_ns nanoseconds using ADJ_SETOFFSET."""
        sec = int(offset_ns // 1_000_000_000)
        nsec = int(offset_ns % 1_000_000_000)
        if nsec < 0:
            sec -= 1
            nsec += 1_000_000_000

        timex_size = 208
        buf = bytearray(timex_size)
        struct.pack_into('<I', buf, 0, ADJ_SETOFFSET | ADJ_NANO)
        # time.tv_sec at offset 72, time.tv_usec at offset 80 (with ADJ_NANO = nsec)
        struct.pack_into('<q', buf, 72, sec)
        struct.pack_into('<q', buf, 80, nsec)

        ret = self._libc.clock_adjtime(
            ctypes.c_int32(self.clock_id),
            ctypes.c_char_p(bytes(buf)),
        )
        if ret < 0:
            errno = ctypes.get_errno()
            raise OSError(errno, f"clock_adjtime ADJ_SETOFFSET failed: {os.strerror(errno)}")


# ── PI Servo ─────────────────────────────────────────────────────────────── #

class PIServo:
    """Proportional-integral controller for PHC frequency steering.

    Modeled after SatPulse's PI servo with anti-windup clamping.
    """

    def __init__(self, kp, ki, max_ppb=62_500_000.0, initial_freq=0.0):
        self.kp = kp
        self.ki = ki
        self.max_ppb = max_ppb
        # Initialize integral accumulator for bumpless transfer
        if ki != 0:
            self.integral = -initial_freq / ki
        else:
            self.integral = 0.0
        self.freq = initial_freq

    def update(self, offset_ns):
        """Process one sample. Returns frequency adjustment in ppb."""
        output = self.kp * offset_ns + self.ki * (self.integral + offset_ns)

        # Anti-windup: only integrate if output stays in bounds
        if abs(output) < self.max_ppb:
            self.integral += offset_ns

        self.freq = max(-self.max_ppb, min(self.max_ppb, output))
        return self.freq

    def reset(self, current_freq):
        """Reset for bumpless transfer at mode change."""
        if self.ki != 0:
            self.integral = -current_freq / self.ki
        self.freq = current_freq


# ── Servo modes ──────────────────────────────────────────────────────────── #

class ServoMode:
    WARMUP = "warmup"          # Waiting for PPP filter to converge
    STEP = "step"              # Large offset → step clock, then converge
    CONVERGING = "converging"  # Aggressive PI gains
    TRACKING = "tracking"      # Stable, low gains


# ── Main servo loop ──────────────────────────────────────────────────────── #

def run_servo(args):
    """Main PHC discipline loop integrating PPP filter + PTP extts."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    # Parse known position
    parts = args.known_pos.split(',')
    lat, lon, alt = float(parts[0]), float(parts[1]), float(parts[2])
    known_ecef = lla_to_ecef(lat, lon, alt)
    log.info(f"Position: {lat:.6f}, {lon:.6f}, {alt:.1f}m")

    # Open PTP device
    ptp = PtpDevice(args.ptp_dev)
    caps = ptp.get_caps()
    log.info(f"PHC: {args.ptp_dev}, max_adj={caps['max_adj']} ppb, "
             f"n_extts={caps['n_ext_ts']}, n_pins={caps['n_pins']}")

    # Reset adjfine to 0 (clear stale settings from previous runs)
    ptp.adjfine(0.0)
    log.info("PHC adjfine reset to 0")

    # Configure SDP pin for extts
    extts_channel = 0  # extts channel (not the pin index)
    try:
        ptp.set_pin_function(args.extts_pin, PTP_PF_EXTTS, extts_channel)
    except OSError:
        log.info("Pin config not supported by driver (igc uses implicit mapping)")
    ptp.enable_extts(extts_channel, rising_edge=True)
    log.info(f"EXTTS enabled: pin={args.extts_pin}, channel={extts_channel}")

    # Set up PPP infrastructure
    beph = BroadcastEphemeris()
    ssr = SSRState()
    corrections = RealtimeCorrections(beph, ssr)
    obs_queue = queue.Queue(maxsize=100)
    stop_event = threading.Event()

    # Read NTRIP config if provided
    ntrip_kwargs = {}
    if args.ntrip_conf:
        import configparser
        cfg = configparser.ConfigParser()
        cfg.read(args.ntrip_conf)
        ntrip_kwargs = {
            'caster': cfg.get('ntrip', 'caster', fallback='products.igs-ip.net'),
            'port': cfg.getint('ntrip', 'port', fallback=2101),
            'user': cfg.get('ntrip', 'user', fallback=''),
            'password': cfg.get('ntrip', 'password', fallback=''),
            'tls': cfg.getboolean('ntrip', 'tls', fallback=False),
        }
    else:
        ntrip_kwargs = {
            'caster': args.caster,
            'port': args.port,
            'user': args.user or '',
            'password': args.password or '',
            'tls': args.tls,
        }

    # Start NTRIP threads
    threads = []

    eph_stream = NtripStream(
        caster=ntrip_kwargs['caster'], port=ntrip_kwargs['port'],
        mountpoint=args.eph_mount,
        user=ntrip_kwargs['user'], password=ntrip_kwargs['password'],
        tls=ntrip_kwargs['tls'],
    )
    t_eph = threading.Thread(
        target=ntrip_reader,
        args=(eph_stream, beph, ssr, stop_event, "EPH"),
        daemon=True,
    )
    t_eph.start()
    threads.append(t_eph)
    log.info(f"NTRIP ephemeris: {args.eph_mount}")

    if args.ssr_mount:
        ssr_stream = NtripStream(
            caster=ntrip_kwargs['caster'], port=ntrip_kwargs['port'],
            mountpoint=args.ssr_mount,
            user=ntrip_kwargs['user'], password=ntrip_kwargs['password'],
            tls=ntrip_kwargs['tls'],
        )
        t_ssr = threading.Thread(
            target=ntrip_reader,
            args=(ssr_stream, beph, ssr, stop_event, "SSR"),
            daemon=True,
        )
        t_ssr.start()
        threads.append(t_ssr)
        log.info(f"NTRIP SSR: {args.ssr_mount}")

    # Wait for ephemeris warmup
    log.info("Waiting for broadcast ephemeris...")
    while beph.n_satellites < 8 and not stop_event.is_set():
        time.sleep(2)
        log.info(f"  Warmup: {beph.summary()}")
    log.info(f"Warmup complete: {beph.summary()}")

    # Parse systems filter
    systems = set(args.systems.split(',')) if args.systems else None

    # Start serial reader
    t_serial = threading.Thread(
        target=serial_reader,
        args=(args.serial, args.baud, obs_queue, stop_event, beph, systems, ssr),
        daemon=True,
    )
    t_serial.start()
    log.info(f"Serial: {args.serial} at {args.baud} baud")

    # Initialize PPP filter
    filt = FixedPosFilter(known_ecef)
    filt.prev_clock = 0.0

    # Servo parameters
    STEP_THRESHOLD_NS = 10_000     # 10 µs — step if offset larger
    CONVERGE_THRESHOLD_NS = 500    # 500 ns — switch to tracking
    CONVERGE_WINDOW = 10           # consecutive samples below threshold

    # PI gains (from SatPulse, adapted)
    CONVERGE_KP = 0.7
    CONVERGE_KI = 0.3
    TRACK_KP = 0.3
    TRACK_KI = 0.1

    servo = PIServo(CONVERGE_KP, CONVERGE_KI, max_ppb=caps['max_adj'])
    mode = ServoMode.WARMUP
    consecutive_good = 0
    prev_t = None
    n_epochs = 0
    warmup_epochs = 20  # let filter converge before steering

    # PPS event queue: extts reader puts events, main loop consumes 1:1
    pps_queue = queue.Queue(maxsize=10)

    def extts_reader():
        """Background thread reading PPS timestamps from PHC."""
        while not stop_event.is_set():
            event = ptp.read_extts(timeout_ms=1500)
            if event is None:
                continue
            phc_sec, phc_nsec, _idx = event
            try:
                pps_queue.put_nowait((phc_sec, phc_nsec))
            except queue.Full:
                # Drain stale events and put the new one
                while not pps_queue.empty():
                    try:
                        pps_queue.get_nowait()
                    except queue.Empty:
                        break
                pps_queue.put_nowait((phc_sec, phc_nsec))

    t_extts = threading.Thread(target=extts_reader, daemon=True)
    t_extts.start()
    log.info("EXTTS reader started")

    def pps_fractional_error(phc_sec, phc_nsec):
        """Compute PHC error from PPS fractional second.

        The F9T PPS fires at GPS second boundaries. The PHC timestamps it.
        If PHC is perfectly aligned, phc_nsec == 0 (or very close).
        phc_nsec near 0 → PHC is slightly ahead (positive error)
        phc_nsec near 1e9 → PHC is slightly behind (negative error)
        """
        if phc_nsec <= 500_000_000:
            return float(phc_nsec)       # PHC ahead
        else:
            return float(phc_nsec) - 1_000_000_000  # PHC behind

    def phc_gps_offset_s(phc_sec, phc_nsec, gps_unix_sec):
        """Compute whole-second offset between PHC epoch and GPS time.

        Returns (PHC_time - GPS_time) in seconds (integer).
        The PPS fires at gps_unix_sec. PHC reads phc_sec.phc_nsec.
        The nearest integer second in PHC time is round(phc_sec + nsec/1e9).
        """
        phc_rounded = phc_sec if phc_nsec < 500_000_000 else phc_sec + 1
        return phc_rounded - gps_unix_sec

    # Open log file
    log_f = None
    log_w = None
    if args.log:
        log_f = open(args.log, 'w', newline='')
        log_w = csv.writer(log_f)
        log_w.writerow([
            'timestamp', 'gps_second', 'phc_sec', 'phc_nsec',
            'dt_rx_ns', 'dt_rx_sigma_ns', 'phc_error_ns',
            'adjfine_ppb', 'mode', 'n_meas',
        ])

    start_time = time.time()
    stepped = False  # True after initial clock step

    try:
        while not stop_event.is_set():
            if args.duration and (time.time() - start_time) > args.duration:
                log.info(f"Duration limit reached ({args.duration}s)")
                break

            try:
                gps_time, observations = obs_queue.get(timeout=5)
            except queue.Empty:
                continue

            # EKF predict + update
            if prev_t is not None:
                dt = (gps_time - prev_t).total_seconds()
                if dt <= 0 or dt > 30:
                    log.warning(f"Suspicious dt={dt:.1f}s, skipping")
                    prev_t = gps_time
                    continue
                filt.predict(dt)
            prev_t = gps_time

            n_used, resid, n_td = filt.update(
                observations, corrections, gps_time,
                clk_file=corrections,
            )

            if n_used < 4:
                continue

            dt_rx_ns = filt.x[filt.IDX_CLK] / C * 1e9
            p_clk = filt.P[filt.IDX_CLK, filt.IDX_CLK]
            dt_rx_sigma = math.sqrt(max(0, p_clk)) / C * 1e9
            n_epochs += 1

            # Get the PPS event for this epoch (1:1 pairing)
            try:
                phc_sec, phc_nsec = pps_queue.get(timeout=0.5)
            except queue.Empty:
                if n_epochs % 10 == 0:
                    log.info(f"  [{n_epochs}] No PPS event for this epoch")
                continue

            # GPS integer second for this PPS
            gps_unix_sec = int(round(gps_time.timestamp()))

            # PHC fractional-second error (standard PPS discipline)
            # This works regardless of PHC epoch, as long as PPS fires at
            # GPS second boundaries (which it does for timing receivers).
            phc_error_ns = pps_fractional_error(phc_sec, phc_nsec)

            # Check if PHC integer seconds match GPS (needed for step)
            epoch_offset = phc_gps_offset_s(phc_sec, phc_nsec, gps_unix_sec)

            # Mode state machine
            adjfine_ppb = servo.freq
            ts_str = gps_time.strftime('%Y-%m-%d %H:%M:%S')

            if mode == ServoMode.WARMUP:
                if n_epochs >= warmup_epochs and dt_rx_sigma < 50.0:
                    log.info(f"  Warmup complete ({n_epochs} epochs, "
                             f"σ={dt_rx_sigma:.1f}ns, "
                             f"phc_frac_err={phc_error_ns:+.0f}ns, "
                             f"epoch_offset={epoch_offset}s)")
                    if epoch_offset != 0:
                        mode = ServoMode.STEP
                        log.info(f"  PHC epoch offset: {epoch_offset}s — need to step")
                    elif abs(phc_error_ns) > STEP_THRESHOLD_NS:
                        mode = ServoMode.STEP
                    else:
                        mode = ServoMode.CONVERGING
                        servo = PIServo(CONVERGE_KP, CONVERGE_KI,
                                        max_ppb=caps['max_adj'])
                elif n_epochs % 10 == 0:
                    log.info(f"  [{n_epochs}] Warmup: "
                             f"dt_rx={dt_rx_ns:+.1f}ns ±{dt_rx_sigma:.1f}ns "
                             f"phc_frac={phc_error_ns:+.0f}ns "
                             f"epoch_off={epoch_offset}s")

            elif mode == ServoMode.STEP:
                # Use phc_ctl for reliable clock stepping
                # Total offset = epoch_offset (whole seconds) + fractional error
                total_offset_ns = epoch_offset * 1_000_000_000 + phc_error_ns
                log.info(f"  STEP: epoch_offset={epoch_offset}s, "
                         f"frac_err={phc_error_ns:+.0f}ns, "
                         f"total={total_offset_ns:+.0f}ns")

                # Use phc_ctl adj for the step (reliable, uses clock_settime)
                # phc_ctl adj takes SECONDS (float), not nanoseconds
                import subprocess
                adj_s = -total_offset_ns / 1_000_000_000
                result = subprocess.run(
                    ['/usr/sbin/phc_ctl', args.ptp_dev, '--',
                     'adj', f'{adj_s:.9f}'],
                    capture_output=True, text=True,
                )
                if result.returncode == 0:
                    log.info(f"  phc_ctl adj {adj_s:.6f}s: {result.stdout.strip()}")
                else:
                    log.error(f"  phc_ctl adj failed (rc={result.returncode}): "
                              f"{result.stderr.strip()} {result.stdout.strip()}")
                stepped = True
                mode = ServoMode.CONVERGING
                servo = PIServo(CONVERGE_KP, CONVERGE_KI,
                                max_ppb=caps['max_adj'])
                # Skip a few samples for the step to settle
                time.sleep(2)
                continue

            elif mode == ServoMode.CONVERGING:
                adjfine_ppb = servo.update(phc_error_ns)
                ptp.adjfine(adjfine_ppb)

                if abs(phc_error_ns) < CONVERGE_THRESHOLD_NS:
                    consecutive_good += 1
                else:
                    consecutive_good = 0

                if consecutive_good >= CONVERGE_WINDOW:
                    log.info(f"  Converged! Switching to tracking mode "
                             f"(err={phc_error_ns:+.0f}ns)")
                    mode = ServoMode.TRACKING
                    servo = PIServo(TRACK_KP, TRACK_KI,
                                    max_ppb=caps['max_adj'],
                                    initial_freq=adjfine_ppb)
                    consecutive_good = 0

                if n_epochs % 5 == 0:
                    log.info(f"  [{n_epochs}] CONVERGE: "
                             f"phc_err={phc_error_ns:+.0f}ns "
                             f"adj={adjfine_ppb:+.1f}ppb "
                             f"σ={dt_rx_sigma:.1f}ns")

            elif mode == ServoMode.TRACKING:
                # Outlier rejection: skip if error > 10× typical
                if abs(phc_error_ns) > 5000:  # 5 µs = clear outlier
                    log.warning(f"  Outlier: phc_err={phc_error_ns:+.0f}ns, skipping")
                    consecutive_good = 0
                    continue

                adjfine_ppb = servo.update(phc_error_ns)
                ptp.adjfine(adjfine_ppb)

                if n_epochs % 10 == 0:
                    log.info(f"  [{n_epochs}] TRACK: "
                             f"phc_err={phc_error_ns:+.0f}ns "
                             f"adj={adjfine_ppb:+.1f}ppb "
                             f"σ={dt_rx_sigma:.1f}ns "
                             f"n={n_used}")

            # Log
            if log_w:
                log_w.writerow([
                    ts_str, gps_unix_sec, phc_sec, phc_nsec,
                    f'{dt_rx_ns:.3f}', f'{dt_rx_sigma:.3f}',
                    f'{phc_error_ns:.1f}',
                    f'{adjfine_ppb:.3f}', mode, n_used,
                ])

    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        stop_event.set()
        try:
            ptp.adjfine(0.0)  # Don't leave PHC at non-zero rate
        except Exception:
            pass
        ptp.disable_extts(extts_channel)
        ptp.close()
        if log_f:
            log_f.close()

    elapsed = time.time() - start_time
    log.info(f"\n{'='*60}")
    log.info(f"  PHC servo complete")
    log.info(f"  Duration: {elapsed:.0f}s, Epochs: {n_epochs}")
    log.info(f"  Final mode: {mode}, adjfine: {adjfine_ppb:+.3f} ppb")
    log.info(f"{'='*60}")


# ── CLI ──────────────────────────────────────────────────────────────────── #

def main():
    ap = argparse.ArgumentParser(
        description="PHC discipline loop using PPP clock estimates (M5)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Position
    ap.add_argument("--known-pos", required=True,
                    help="Known position as lat,lon,alt")
    ap.add_argument("--leap", type=int, default=18,
                    help="UTC-GPS leap seconds (default: 18)")
    ap.add_argument("--systems", default="gps,gal",
                    help="GNSS systems to use (default: gps,gal)")

    # Serial
    ap.add_argument("--serial", required=True,
                    help="F9T serial port (e.g. /dev/gnss-top)")
    ap.add_argument("--baud", type=int, default=9600,
                    help="Serial baud rate (default: 9600)")

    # NTRIP (direct args or config file)
    ap.add_argument("--ntrip-conf", help="NTRIP config file (INI format)")
    ap.add_argument("--caster", default="products.igs-ip.net")
    ap.add_argument("--port", type=int, default=2101)
    ap.add_argument("--user", default=None)
    ap.add_argument("--password", default=None)
    ap.add_argument("--tls", action="store_true")
    ap.add_argument("--eph-mount", required=True,
                    help="Broadcast ephemeris mountpoint")
    ap.add_argument("--ssr-mount", default=None,
                    help="SSR corrections mountpoint (optional)")

    # PTP
    ap.add_argument("--ptp-dev", default="/dev/ptp0",
                    help="PTP device (default: /dev/ptp0)")
    ap.add_argument("--extts-pin", type=int, default=1,
                    help="SDP pin for PPS input (default: 1 = SDP1)")

    # Servo tuning
    ap.add_argument("--warmup", type=int, default=20,
                    help="Warmup epochs before steering (default: 20)")
    ap.add_argument("--step-threshold", type=float, default=10000,
                    help="Step clock if offset > this (ns, default: 10000)")

    # Output
    ap.add_argument("--log", default=None,
                    help="CSV log file for servo data")
    ap.add_argument("--duration", type=int, default=None,
                    help="Run duration in seconds")

    args = ap.parse_args()
    run_servo(args)


if __name__ == "__main__":
    main()

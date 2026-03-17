#!/usr/bin/env python3
"""
phc_servo.py — PHC discipline loop for PePPAR Fix M7.

Disciplines the TimeHAT i226 PHC (/dev/ptp0) using competitive
error source selection: PPS-only, PPS+qErr, and carrier-phase
estimates compete at every epoch based on confidence.

M7 adds adaptive discipline interval: instead of calling adjfine every
second (which injects ~7.5 ppb of correction jitter), the servo
accumulates error samples over N epochs and applies one averaged
correction.  This reduces TDEV at short tau while preserving tracking
bandwidth.  Use --discipline-interval N for fixed interval or
--adaptive-interval to let the scheduler choose based on drift rate
vs measurement noise.

Architecture:
    F9T PPS → SDP1 → extts event (PHC timestamp of PPS edge)
    F9T TIM-TP → qErr (PPS quantization error, ~3 ns precision)
    PPP filter → dt_rx (receiver clock offset from GPS time)

    Error sources (compete by confidence):
      1. PPS-only:    error = pps_frac(phc)           ±20 ns
      2. PPS + qErr:  error = pps_frac(phc) + qErr    ±3 ns
      3. Carrier-phase: error = pps_frac(phc) + dt_rx  ±0.1 ns

    PI servo → adjfine() on /dev/ptp0, gains scaled by confidence
    DisciplineScheduler → accumulates samples, decides when to correct

    Output: SDP0 → disciplined PPS (SMA J4 → TICC chA for measurement)

The servo reads PPS timestamps via the PTP_EXTTS_EVENT ioctl and
correlates them with PPP clock estimates at the same GPS second.
A PI controller drives adjfine to minimize the PHC-GPS offset.

GNSS receiver requirements:
    The PPP filter uses ionosphere-free (IF) combination of dual-frequency
    observations. This REQUIRES two frequencies per satellite:
        GPS:     L1 C/A + L5Q
        Galileo: E1C + E5aQ
        BDS:     B1I + B2aI

    Single-frequency satellites are silently dropped. The filter needs
    at least 4 dual-frequency SVs per epoch from any combination of
    constellations.

    L1-only operation is NOT supported — the ionosphere-free combination
    is fundamental to the PPP approach. Without it, ionospheric delay
    (up to ~50 ns at zenith, worse at low elevation) would dominate the
    clock estimate.

    GPS L5 availability: only GPS Block IIF/III satellites transmit L5
    (~15 of 32 SVs). The signal is flagged "unhealthy" in the nav message;
    the receiver needs the L5 health override (u-blox App Note UBX-21038688,
    key 0x10320001). IMPORTANT: the override is saved to flash but does NOT
    take effect until the receiver is warm-restarted. configure_f9t.py
    handles this automatically. Without the restart, GPS delivers only L1
    (single-freq, dropped by filter) even though the config ACK'd correctly.

    With GPS L5 enabled: ~8 GPS + ~7 Galileo = ~15 dual-freq SVs per epoch.
    Without GPS L5: ~7 Galileo only — still sufficient for the filter.

    Run configure_f9t.py to set up the receiver:
        python scripts/configure_f9t.py /dev/gnss-top --port-type USB

    Correction stream requirements:
        - Broadcast ephemeris (RTCM 1019/1042/1045/1046) — required
        - SSR orbit + clock corrections — recommended for sub-meter accuracy
        - SSR code biases — applied when available (improves convergence)

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
from realtime_ppp import serial_reader, ntrip_reader, QErrStore

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

    def update(self, offset_ns, dt=1.0):
        """Process one sample. Returns frequency adjustment in ppb.

        Args:
            offset_ns: measured offset in nanoseconds
            dt: seconds since last correction (scales integral contribution)
        """
        output = self.kp * offset_ns + self.ki * (self.integral + offset_ns * dt)

        # Anti-windup: only integrate if output stays in bounds
        if abs(output) < self.max_ppb:
            self.integral += offset_ns * dt

        self.freq = max(-self.max_ppb, min(self.max_ppb, output))
        return self.freq

    def reset(self, current_freq):
        """Reset for bumpless transfer at mode change."""
        if self.ki != 0:
            self.integral = -current_freq / self.ki
        self.freq = current_freq


# ── Error source competition (M6) ────────────────────────────────────────── #

class ErrorSource:
    """One candidate error estimate with its confidence."""
    __slots__ = ('name', 'error_ns', 'confidence_ns')

    def __init__(self, name, error_ns, confidence_ns):
        self.name = name
        self.error_ns = error_ns
        self.confidence_ns = confidence_ns

    def __repr__(self):
        return f"{self.name}({self.error_ns:+.1f}ns ±{self.confidence_ns:.1f})"


def compute_error_sources(pps_error_ns, qerr_ns, dt_rx_ns, dt_rx_sigma_ns,
                          pps_confidence=20.0, qerr_confidence=3.0,
                          carrier_max_sigma=50.0):
    """Compute all available error sources and return sorted by confidence.

    Args:
        pps_error_ns: fractional-second PHC error from PPS timestamp
        qerr_ns: quantization error from TIM-TP (None if unavailable)
        dt_rx_ns: receiver clock offset from carrier-phase filter
        dt_rx_sigma_ns: filter's confidence in dt_rx (None if unavailable)
        pps_confidence: assumed PPS-only confidence (ns)
        qerr_confidence: assumed PPS+qErr confidence (ns)
        carrier_max_sigma: max sigma to accept carrier-phase (ns)

    Returns:
        List of ErrorSource, sorted by confidence (best first).
    """
    sources = []

    # 1. PPS-only: always available
    sources.append(ErrorSource('pps', pps_error_ns, pps_confidence))

    # 2. PPS + qErr: available when TIM-TP has been received
    if qerr_ns is not None:
        # Validated sign convention (testAnt): corrected = raw + qerr
        # Positive qErr means PPS fired early; adding qErr compensates.
        sources.append(ErrorSource('pps+qerr',
                                   pps_error_ns + qerr_ns,
                                   qerr_confidence))

    # 3. Carrier-phase: available when filter has converged
    if dt_rx_sigma_ns is not None and dt_rx_sigma_ns < carrier_max_sigma:
        # dt_rx is the receiver clock offset: positive = receiver ahead
        # PPS fires early by dt_rx; add it to get PHC error vs true GPS time
        sources.append(ErrorSource('carrier',
                                   pps_error_ns + dt_rx_ns,
                                   dt_rx_sigma_ns))

    sources.sort(key=lambda s: s.confidence_ns)
    return sources


# ── Discipline scheduler (M7) ─────────────────────────────────────────── #

class DisciplineScheduler:
    """Accumulates error samples and decides when to apply a correction.

    M7: instead of correcting every epoch, buffer N samples and apply one
    averaged correction.  This reduces correction jitter while preserving
    tracking bandwidth.

    Supports fixed interval (--discipline-interval N) or adaptive mode
    (--adaptive-interval) where the interval is chosen based on TCXO drift
    rate vs measurement noise.
    """

    def __init__(self, base_interval=1, adaptive=False,
                 min_interval=1, max_interval=120):
        self.base_interval = base_interval
        self.adaptive = adaptive
        self.min_interval = min_interval
        self.max_interval = max_interval

        # Current discipline interval (may change in adaptive mode)
        self.interval = base_interval

        # Sample buffer for current interval
        self._errors = []
        self._confidences = []
        self._sources = []

        # Drift rate EMA for adaptive mode
        self._drift_rate = 0.1  # ppb/s — conservative startup default
        self._drift_alpha = 0.05  # EMA smoothing factor
        self._prev_adjfine = None
        self._prev_adjfine_t = None
        self._adjfine_history_s = 0.0  # seconds of adjfine history

        # Convergence override threshold (ns)
        self._converge_threshold = 500.0

    @property
    def n_accumulated(self):
        """Number of samples in the current buffer."""
        return len(self._errors)

    def accumulate(self, error_ns, confidence_ns, source_name):
        """Buffer one error sample."""
        self._errors.append(error_ns)
        self._confidences.append(confidence_ns)
        self._sources.append(source_name)

    def should_correct(self):
        """True when it's time to flush the buffer and correct.

        Triggers:
          1. Buffer is full (n_accumulated >= interval)
          2. Convergence override: large error needs immediate correction
          3. Source transition mid-interval (different source than first sample)
        """
        n = len(self._errors)
        if n == 0:
            return False

        # Buffer full
        if n >= self.interval:
            return True

        # Convergence override: large error → force immediate correction
        if abs(self._errors[-1]) > self._converge_threshold:
            return True

        # Source transition mid-interval
        if n > 1 and self._sources[-1] != self._sources[0]:
            return True

        return False

    def flush(self):
        """Return averaged error, confidence, and sample count; reset buffer.

        Returns:
            (avg_error_ns, avg_confidence_ns, n_samples)
        """
        if not self._errors:
            return (0.0, 0.0, 0)

        n = len(self._errors)
        avg_error = sum(self._errors) / n
        avg_confidence = sum(self._confidences) / n
        self._errors.clear()
        self._confidences.clear()
        self._sources.clear()
        return (avg_error, avg_confidence, n)

    def update_drift_rate(self, timestamp, adjfine_ppb):
        """Update EMA of |delta_adjfine / delta_t| for adaptive scheduling.

        Args:
            timestamp: monotonic time in seconds
            adjfine_ppb: current adjfine value
        """
        if self._prev_adjfine is not None and self._prev_adjfine_t is not None:
            dt = timestamp - self._prev_adjfine_t
            if dt > 0:
                rate = abs(adjfine_ppb - self._prev_adjfine) / dt
                self._drift_rate = (self._drift_alpha * rate +
                                    (1.0 - self._drift_alpha) * self._drift_rate)
                self._adjfine_history_s += dt

        self._prev_adjfine = adjfine_ppb
        self._prev_adjfine_t = timestamp

    def compute_adaptive_interval(self, measurement_sigma_ns):
        """Compute optimal discipline interval from drift rate and noise.

        Uses tau = (2 * sigma / drift_rate)^(2/5), clamped to [min, max].
        The 2/5 exponent comes from the crossover between white noise
        averaging (tau^-1/2) and random-walk frequency drift (tau^1/2).

        Only adapts after 60s of adjfine history; uses base_interval before.
        """
        if not self.adaptive:
            return self.base_interval

        # Need enough history for a meaningful drift estimate
        if self._adjfine_history_s < 60.0:
            return self.base_interval

        # Guard against zero drift rate
        if self._drift_rate < 1e-6:
            return self.max_interval

        tau = (2.0 * measurement_sigma_ns / self._drift_rate) ** 0.4
        tau = max(self.min_interval, min(self.max_interval, int(round(tau))))
        self.interval = tau
        return tau


# ── Main servo loop ──────────────────────────────────────────────────────── #

def run_servo(args):
    """Main PHC discipline loop with competitive error source selection (M6).

    Three error sources compete at every epoch:
      1. PPS-only    (~20 ns confidence, always available)
      2. PPS + qErr  (~3 ns, when TIM-TP is available)
      3. Carrier-phase (~0.1 ns, when PPP filter has converged)

    The source with the lowest confidence interval drives the servo.
    PI gains scale with selected confidence: better measurement → more
    aggressive correction.  No discrete mode transitions after the
    initial warmup/step bootstrap.
    """
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

    # QErrStore for TIM-TP quantization error (M6)
    qerr_store = QErrStore()

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

    # Start serial reader (with qerr_store for TIM-TP extraction)
    t_serial = threading.Thread(
        target=serial_reader,
        args=(args.serial, args.baud, obs_queue, stop_event, beph, systems, ssr),
        kwargs={'qerr_store': qerr_store},
        daemon=True,
    )
    t_serial.start()
    log.info(f"Serial: {args.serial} at {args.baud} baud")

    # ── Receiver signal diagnostic ──────────────────────────────────────
    log.info("Checking receiver signals (3 epochs)...")
    sys_counts = {}
    for _diag_i in range(3):
        try:
            _t, _obs = obs_queue.get(timeout=10)
        except queue.Empty:
            log.warning("  No observations received — check serial connection "
                        "and receiver configuration")
            break
        for o in _obs:
            s = o.get('sys', '?')
            sys_counts[s] = sys_counts.get(s, 0) + 1
        obs_queue.put((_t, _obs))

    if sys_counts:
        parts = [f"{s.upper()}={n//3}" for s, n in sorted(sys_counts.items())]
        log.info(f"  Dual-freq SVs per epoch: {', '.join(parts)}")

        if 'gps' in (systems or set()) and sys_counts.get('gps', 0) == 0:
            log.warning(
                "  NO GPS dual-frequency observations! GPS L5 is not being tracked.\n"
                "  The receiver needs: (1) GPS L5 signal enabled, (2) L5 health override\n"
                "  (App Note UBX-21038688), and (3) a warm restart after the override.\n"
                "  Run: python scripts/configure_f9t.py <port> --port-type USB\n"
                "  (Full factory reset + configure + L5 override + restart)")
        if 'gal' in (systems or set()) and sys_counts.get('gal', 0) == 0:
            log.warning(
                "  NO Galileo dual-frequency observations!\n"
                "  Run: python scripts/configure_f9t.py <port> --port-type USB --skip-reset")
        n_total = sum(sys_counts.values()) // 3
        if n_total < 4:
            log.warning(
                f"  Only {n_total} dual-freq SVs per epoch — filter needs ≥4.\n"
                "  Check antenna, receiver config, and sky view.")
    else:
        log.warning("  No observation epochs received in 30s. Check:\n"
                    "  1. Serial port and baud rate\n"
                    "  2. Receiver is in timing mode (run configure_f9t.py)\n"
                    "  3. UBX messages enabled (RXM-RAWX, RXM-SFRBX)")

    # Initialize PPP filter
    filt = FixedPosFilter(known_ecef)
    filt.prev_clock = 0.0

    # Servo parameters
    STEP_THRESHOLD_NS = 10_000     # 10 µs — step if offset larger

    # PI gains — base values, scaled by error source confidence at runtime
    BASE_KP = args.track_kp        # default 0.3
    BASE_KI = args.track_ki        # default 0.1

    # Gain scaling: gain_factor = clamp(REF_SIGMA / source_confidence)
    # REF_SIGMA chosen so gains = 1× at PPS+qErr quality (~2 ns)
    GAIN_REF_SIGMA = args.gain_ref_sigma
    GAIN_MIN_SCALE = 0.1           # floor (PPS-only: gentle)
    GAIN_MAX_SCALE = 3.0           # ceiling (excellent carrier: aggressive)

    # During convergence (large error), ensure minimum gain aggressiveness
    # so pull-in doesn't stall at PPS-only quality
    CONVERGE_ERROR_NS = 500        # above this, boost gains
    CONVERGE_MIN_SCALE = 2.0       # minimum gain scale during convergence

    servo = PIServo(BASE_KP, BASE_KI, max_ppb=caps['max_adj'])
    scheduler = DisciplineScheduler(
        base_interval=args.discipline_interval,
        adaptive=args.adaptive_interval,
        min_interval=args.min_interval,
        max_interval=args.max_interval,
    )
    phase = 'warmup'
    prev_t = None
    n_epochs = 0
    warmup_epochs = args.warmup    # default 20
    prev_source = None

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

        phc_nsec near 0 → PHC slightly ahead (positive error)
        phc_nsec near 1e9 → PHC slightly behind (negative error)
        """
        if phc_nsec <= 500_000_000:
            return float(phc_nsec)
        else:
            return float(phc_nsec) - 1_000_000_000

    def phc_gps_offset_s(phc_sec, phc_nsec, gps_unix_sec):
        """Whole-second offset: PHC_time - GPS_time."""
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
            'dt_rx_ns', 'dt_rx_sigma_ns', 'pps_error_ns', 'qerr_ns',
            'source', 'source_error_ns', 'source_confidence_ns',
            'adjfine_ppb', 'phase', 'n_meas', 'gain_scale',
            'discipline_interval', 'n_accumulated',
        ])

    start_time = time.time()
    adjfine_ppb = 0.0
    gain_scale = 1.0

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

            gps_unix_sec = int(round(gps_time.timestamp()))
            ts_str = gps_time.strftime('%Y-%m-%d %H:%M:%S')
            pps_error_ns = pps_fractional_error(phc_sec, phc_nsec)

            # Get qErr from TIM-TP (None if stale or unavailable)
            qerr_ns, _ = qerr_store.get()

            # Compute competitive error sources (M6)
            sources = compute_error_sources(
                pps_error_ns, qerr_ns, dt_rx_ns, dt_rx_sigma,
            )
            best = sources[0]

            # ── Bootstrap: warmup ──────────────────────────────────────
            if phase == 'warmup':
                if n_epochs >= warmup_epochs:
                    epoch_offset = phc_gps_offset_s(phc_sec, phc_nsec, gps_unix_sec)
                    log.info(f"  Warmup complete ({n_epochs} epochs, "
                             f"best={best}, epoch_offset={epoch_offset}s)")
                    if epoch_offset != 0 or abs(best.error_ns) > STEP_THRESHOLD_NS:
                        phase = 'step'
                    else:
                        phase = 'tracking'
                        log.info(f"  → tracking (no step needed)")
                elif n_epochs % 10 == 0:
                    log.info(f"  [{n_epochs}] warmup: best={best} "
                             f"dt_rx={dt_rx_ns:+.1f}±{dt_rx_sigma:.1f}ns")
                # Log but don't steer during warmup
                if log_w:
                    log_w.writerow([
                        ts_str, gps_unix_sec, phc_sec, phc_nsec,
                        f'{dt_rx_ns:.3f}', f'{dt_rx_sigma:.3f}',
                        f'{pps_error_ns:.1f}', f'{qerr_ns:.3f}' if qerr_ns is not None else '',
                        best.name, f'{best.error_ns:.3f}', f'{best.confidence_ns:.3f}',
                        f'{adjfine_ppb:.3f}', phase, n_used, f'{gain_scale:.3f}',
                        scheduler.interval, 0,
                    ])
                continue

            # ── Bootstrap: step ────────────────────────────────────────
            if phase == 'step':
                epoch_offset = phc_gps_offset_s(phc_sec, phc_nsec, gps_unix_sec)
                # Use the best available error source for the step
                total_offset_ns = epoch_offset * 1_000_000_000 + best.error_ns
                log.info(f"  STEP: epoch_offset={epoch_offset}s, "
                         f"source={best}, total={total_offset_ns:+.0f}ns")

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

                # Reset servo and scheduler for clean start after step
                servo = PIServo(BASE_KP, BASE_KI, max_ppb=caps['max_adj'])
                scheduler = DisciplineScheduler(
                    base_interval=args.discipline_interval,
                    adaptive=args.adaptive_interval,
                    min_interval=args.min_interval,
                    max_interval=args.max_interval,
                )
                phase = 'tracking'
                # Flush stale PPS events
                time.sleep(2)
                while not pps_queue.empty():
                    try:
                        pps_queue.get_nowait()
                    except queue.Empty:
                        break
                continue

            # ── Continuous tracking with competitive error sources ──────
            # Outlier rejection
            if abs(best.error_ns) > 5000:
                log.warning(f"  Outlier: {best}, skipping")
                continue

            # Accumulate sample into discipline scheduler (M7)
            scheduler.accumulate(best.error_ns, best.confidence_ns, best.name)

            # Log source transitions
            if prev_source != best.name:
                if prev_source is not None:
                    log.info(f"  Source: {prev_source} → {best.name} "
                             f"(confidence {best.confidence_ns:.1f}ns)")
                prev_source = best.name

            if scheduler.should_correct():
                # Flush buffer: get averaged error and confidence
                avg_error, avg_confidence, n_samples = scheduler.flush()

                # Gain scaling by averaged confidence
                gain_scale = max(GAIN_MIN_SCALE, min(GAIN_MAX_SCALE,
                                 GAIN_REF_SIGMA / avg_confidence))

                # Boost gains during convergence (large error) to ensure
                # pull-in doesn't stall when using low-confidence sources
                if abs(avg_error) > CONVERGE_ERROR_NS:
                    gain_scale = max(gain_scale, CONVERGE_MIN_SCALE)

                servo.kp = BASE_KP * gain_scale
                servo.ki = BASE_KI * gain_scale

                # Negate: positive error (PHC ahead) → negative adjfine (slow down)
                # dt = n_samples seconds since last correction
                adjfine_ppb = -servo.update(avg_error, dt=float(n_samples))
                ptp.adjfine(adjfine_ppb)

                # Update drift rate tracker for adaptive mode
                scheduler.update_drift_rate(time.monotonic(), adjfine_ppb)

                # Adapt interval for next cycle
                scheduler.compute_adaptive_interval(avg_confidence)

                if n_epochs % 10 == 0:
                    src_summary = ' '.join(f'{s.name}={s.error_ns:+.1f}' for s in sources)
                    log.info(f"  [{n_epochs}] {best.name}: "
                             f"err={avg_error:+.1f}ns (avg {n_samples}) "
                             f"adj={adjfine_ppb:+.1f}ppb "
                             f"gain={gain_scale:.2f}x "
                             f"interval={scheduler.interval} "
                             f"[{src_summary}]")
            else:
                # Coast epoch: don't call adjfine, just log
                n_samples = 0
                if n_epochs % 10 == 0:
                    src_summary = ' '.join(f'{s.name}={s.error_ns:+.1f}' for s in sources)
                    log.info(f"  [{n_epochs}] {best.name}: "
                             f"err={best.error_ns:+.1f}ns "
                             f"coast ({scheduler.n_accumulated}/{scheduler.interval}) "
                             f"adj={adjfine_ppb:+.1f}ppb "
                             f"[{src_summary}]")

            # CSV log (every epoch, including coast)
            if log_w:
                log_w.writerow([
                    ts_str, gps_unix_sec, phc_sec, phc_nsec,
                    f'{dt_rx_ns:.3f}', f'{dt_rx_sigma:.3f}',
                    f'{pps_error_ns:.1f}', f'{qerr_ns:.3f}' if qerr_ns is not None else '',
                    best.name, f'{best.error_ns:.3f}', f'{best.confidence_ns:.3f}',
                    f'{adjfine_ppb:.3f}', phase, n_used, f'{gain_scale:.3f}',
                    scheduler.interval, scheduler.n_accumulated,
                ])

    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        stop_event.set()
        try:
            ptp.adjfine(0.0)
        except Exception:
            pass
        ptp.disable_extts(extts_channel)
        ptp.close()
        if log_f:
            log_f.close()

    elapsed = time.time() - start_time
    log.info(f"\n{'='*60}")
    log.info(f"  PHC servo complete (M7 adaptive discipline interval)")
    log.info(f"  Duration: {elapsed:.0f}s, Epochs: {n_epochs}")
    log.info(f"  Last source: {prev_source}, adjfine: {adjfine_ppb:+.3f} ppb")
    log.info(f"{'='*60}")


# ── CLI ──────────────────────────────────────────────────────────────────── #

def main():
    ap = argparse.ArgumentParser(
        description="PHC discipline loop with competitive error sources and adaptive discipline interval (M7)",
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
    ap.add_argument("--track-kp", type=float, default=0.3,
                    help="Tracking mode Kp gain (default: 0.3)")
    ap.add_argument("--track-ki", type=float, default=0.1,
                    help="Tracking mode Ki gain (default: 0.1)")
    ap.add_argument("--gain-ref-sigma", type=float, default=2.0,
                    help="Reference confidence (ns) for gain scale=1.0 (default: 2.0)")

    # Discipline interval (M7)
    ap.add_argument("--discipline-interval", type=int, default=1,
                    help="Fixed discipline interval in epochs (default: 1 = M6 behavior)")
    ap.add_argument("--adaptive-interval", action="store_true",
                    help="Enable adaptive discipline interval based on drift rate")
    ap.add_argument("--max-interval", type=int, default=120,
                    help="Maximum discipline interval in epochs (default: 120)")
    ap.add_argument("--min-interval", type=int, default=1,
                    help="Minimum discipline interval in epochs (default: 1)")

    # Output
    ap.add_argument("--log", default=None,
                    help="CSV log file for servo data")
    ap.add_argument("--duration", type=int, default=None,
                    help="Run duration in seconds")

    args = ap.parse_args()
    run_servo(args)


if __name__ == "__main__":
    main()

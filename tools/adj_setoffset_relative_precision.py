#!/usr/bin/env python3
"""Measure ADJ_SETOFFSET single-shot precision in pure relative form.

The test in tools/step_method_comparison.py conflates PHC-vs-realtime
drift with ADJ_SETOFFSET imprecision: it computes "expected = target +
elapsed realtime" which assumes PHC runs at exactly realtime frequency.
With a -135 ppb TCXO that's wrong by hundreds of ns over a 1-second
trial.

This tool measures the ioctl's atomicity in pure relative form:

    read (phc_a, mono_a) tightly via PTP_SYS_OFFSET_PRECISE
    adj_setoffset(+random_offset)
    read (phc_b, mono_b) tightly
    residual = (phc_b - phc_a) - offset - (mono_b - mono_a)

The (mono_b - mono_a) term subtracts whatever time elapsed during the
syscall, so the residual is purely "did ADJ_SETOFFSET advance the PHC
by *exactly* the requested offset, atomic in monotonic time."

Usage:
    sudo python3 tools/adj_setoffset_relative_precision.py /dev/ptp_i226
    sudo python3 tools/adj_setoffset_relative_precision.py /dev/ptp_i226 --trials 200
"""
from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import os
import random
import statistics
import struct
import sys

PTP_CLK_MAGIC = ord('=')
_IOC_READ = 2
_IOC_WRITE = 1
_IOC_READWRITE = 3


def _IOC(direction, typ, nr, size):
    return (direction << 30) | (size << 16) | (typ << 8) | nr


def _IOWR(typ, nr, size):
    return _IOC(_IOC_READWRITE, typ, nr, size)


# struct ptp_sys_offset_precise: { ptp_clock_time device, sys_realtime, monoraw; }
# 3 * 16 = 48 bytes
PTP_SYS_OFFSET_PRECISE_SIZE = 48
PTP_SYS_OFFSET_PRECISE = _IOWR(PTP_CLK_MAGIC, 8, PTP_SYS_OFFSET_PRECISE_SIZE)


def _parse_ptp_clock_time(buf, offset):
    sec = struct.unpack_from('<q', buf, offset)[0]
    nsec = struct.unpack_from('<I', buf, offset + 8)[0]
    return sec * 1_000_000_000 + nsec


def read_phc_precise(fd):
    """Returns (phc_ns, sys_realtime_ns, mono_raw_ns) atomically.

    Uses PTP_SYS_OFFSET_PRECISE which kernel-correlates PHC with
    sys_realtime + mono_raw via cross-timestamping.
    """
    import fcntl
    buf = bytearray(PTP_SYS_OFFSET_PRECISE_SIZE)
    fcntl.ioctl(fd, PTP_SYS_OFFSET_PRECISE, buf, True)
    return (_parse_ptp_clock_time(buf, 0),
            _parse_ptp_clock_time(buf, 16),
            _parse_ptp_clock_time(buf, 32))


def adj_setoffset(fd, offset_ns):
    """ADJ_SETOFFSET in nanosecond mode."""
    ADJ_SETOFFSET = 0x0100
    ADJ_NANO = 0x2000

    phc_clkid = (~fd << 3) | 3

    sec = int(offset_ns // 1_000_000_000)
    nsec = int(offset_ns % 1_000_000_000)
    if offset_ns < 0:
        sec = -int((-offset_ns) // 1_000_000_000)
        nsec = -int((-offset_ns) % 1_000_000_000)
        if nsec < 0 and sec == 0:
            sec = -1
            nsec = 1_000_000_000 + nsec

    librt = ctypes.CDLL(ctypes.util.find_library("rt"), use_errno=True)

    class timeval(ctypes.Structure):
        _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]

    class timex(ctypes.Structure):
        _fields_ = [
            ("modes", ctypes.c_uint),
            ("offset", ctypes.c_long),
            ("freq", ctypes.c_long),
            ("maxerror", ctypes.c_long),
            ("esterror", ctypes.c_long),
            ("status", ctypes.c_int),
            ("constant", ctypes.c_long),
            ("precision", ctypes.c_long),
            ("tolerance", ctypes.c_long),
            ("time", timeval),
            ("tick", ctypes.c_long),
            ("ppsfreq", ctypes.c_long),
            ("jitter", ctypes.c_long),
            ("shift", ctypes.c_int),
            ("stabil", ctypes.c_long),
            ("jitcnt", ctypes.c_long),
            ("calcnt", ctypes.c_long),
            ("errcnt", ctypes.c_long),
            ("stbcnt", ctypes.c_long),
            ("tai", ctypes.c_int),
        ]

    tx = timex()
    tx.modes = ADJ_SETOFFSET | ADJ_NANO
    tx.time.tv_sec = sec
    tx.time.tv_usec = nsec  # nsec when ADJ_NANO is set

    ret = librt.clock_adjtime(phc_clkid, ctypes.byref(tx))
    if ret < 0:
        errno = ctypes.get_errno()
        raise OSError(f"clock_adjtime(ADJ_SETOFFSET) failed: errno={errno}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("ptp_dev", help="PTP device, e.g. /dev/ptp_i226")
    ap.add_argument("--trials", type=int, default=200,
                    help="Number of trials (default 200)")
    ap.add_argument("--offset-range-us", type=float, default=100.0,
                    help="Random offset uniform in ±range µs (default 100)")
    ap.add_argument("--restore", action="store_true", default=True,
                    help="Apply -offset after each test to leave PHC ~unchanged")
    args = ap.parse_args()

    fd = os.open(args.ptp_dev, os.O_RDWR)
    try:
        residuals_ns = []
        offsets_used_ns = []
        elapsed_mono_ns = []

        for i in range(args.trials):
            offset_ns = int(random.uniform(-args.offset_range_us * 1000.0,
                                           args.offset_range_us * 1000.0))
            phc_a, _sys_a, mono_a = read_phc_precise(fd)
            adj_setoffset(fd, offset_ns)
            phc_b, _sys_b, mono_b = read_phc_precise(fd)

            elapsed_mono = mono_b - mono_a
            phc_delta = phc_b - phc_a
            residual = phc_delta - offset_ns - elapsed_mono

            residuals_ns.append(residual)
            offsets_used_ns.append(offset_ns)
            elapsed_mono_ns.append(elapsed_mono)

            if args.restore:
                adj_setoffset(fd, -offset_ns)

        abs_res = [abs(r) for r in residuals_ns]
        abs_res.sort()
        n = len(abs_res)

        def pct(p):
            i = max(0, min(n - 1, round(p / 100.0 * (n - 1))))
            return abs_res[i]

        print(f"Device: {args.ptp_dev}")
        print(f"Trials: {n}")
        print(f"Offset range: ±{args.offset_range_us:.0f} µs (uniform)")
        print(f"Mean elapsed_mono per trial: "
              f"{statistics.mean(elapsed_mono_ns)/1000:.1f} µs "
              f"(median {statistics.median(elapsed_mono_ns)/1000:.1f} µs)")
        print(f"\n|residual| (relative ADJ_SETOFFSET precision):")
        print(f"  min    = {abs_res[0]:>8d} ns")
        print(f"  p5     = {pct(5):>8d} ns")
        print(f"  median = {pct(50):>8d} ns")
        print(f"  p95    = {pct(95):>8d} ns")
        print(f"  max    = {abs_res[-1]:>8d} ns")
        print(f"\nSigned residual stats:")
        print(f"  mean   = {statistics.mean(residuals_ns):+.0f} ns")
        print(f"  stdev  = {statistics.stdev(residuals_ns):.0f} ns")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()

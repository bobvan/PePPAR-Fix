#!/usr/bin/env python3
"""
tdc_reader.py — Read Renesas 8A34002 ClockMatrix TDC phase error via I2C.

Reads the Time-to-Digital Converter (TDC) phase measurement from the
Renesas 8A34002 on Timebeat OTC hardware.  The TDC measures the phase
offset between the F9T PPS input and the OCXO-derived feedback clock
with sub-50 ps resolution (per AN-1010).

This is a read-only measurement tool — it does NOT steer the DCO.

Hardware setup:
    - Timebeat OTC board (otcBob1 or ptBoat)
    - Renesas 8A34002 on I2C bus 1, address 0x58
    - Timebeat service stopped (systemctl stop timebeat) to release I2C

Register model:
    The 8A34002 uses 16-bit register addresses accessed via I2C page
    selection.  Write the high byte to the PAGE register (0xFD), then
    read/write registers at the low-byte offset within that page.

    TDC phase status registers (DPLL0):
        0xD294..0xD298  DPLL0_PHASE_STATUS (5 bytes, little-endian)
        Format: signed fixed-point nanoseconds
            [31:0]  fractional ns (1 LSB = 1/2^32 ns ≈ 0.233 ps)
            [39:32] integer ns (signed byte, ±127 ns range)
        Effective single-shot resolution: ~20–50 ps (TDC analog noise)

Usage:
    # Stop timebeat first:
    #   sudo systemctl stop timebeat

    # Sample at 1 Hz for 5 minutes (default):
    python tdc_reader.py

    # Custom duration and output:
    python tdc_reader.py --duration 600 --csv tdc_log.csv

    # Different I2C bus/address:
    python tdc_reader.py --bus 1 --addr 0x58

    # Probe mode — single read, verify register access:
    python tdc_reader.py --probe

Output (stdout, one line per second):
    epoch_s,phase_ps
    1710892800.123,42.7
    1710892801.125,-15.3
    ...
"""

from __future__ import annotations

import argparse
import csv
import sys
import time

try:
    from smbus2 import SMBus
except ImportError:
    sys.exit("smbus2 not installed.  pip install smbus2")


# ── 8A34002 register constants ──────────────────────────────────────── #

PAGE_REG = 0xFD              # Page select register (write high byte here)

# Device ID register: address 0x0002–0x0003 (2 bytes, little-endian).
# Expected value for 8A34002: 0x3400.
DEVICE_ID_PAGE   = 0x00
DEVICE_ID_OFFSET = 0x02
DEVICE_ID_LEN    = 2

# DPLL0 phase status: 5 bytes, little-endian.
# Address 0xD294 → page 0xD2, offset 0x94..0x98.
# Format: [31:0] = fractional ns (unsigned), [39:32] = integer ns (signed).
DPLL0_PHASE_STATUS_PAGE   = 0xD2
DPLL0_PHASE_STATUS_OFFSET = 0x94
DPLL0_PHASE_STATUS_LEN    = 5

# DPLL0 lock status register (1 byte): bit 0 = locked.
# Address 0xD280 → page 0xD2, offset 0x80.
DPLL0_LOCK_STATUS_OFFSET = 0x80

# Phase status encoding: 40-bit signed fixed-point nanoseconds.
# Lower 32 bits = fractional ns (1 LSB = 1/2^32 ns ≈ 0.233 ps).
# Upper 8 bits = integer ns (signed).
FRAC_BITS = 32
FRAC_SCALE_PS = 1e3 / (1 << FRAC_BITS)   # convert fractional ns to ps: 1e3 ps/ns / 2^32


# ── I2C helpers ──────────────────────────────────────────────────────── #

class ClockMatrix:
    """Thin wrapper for paged I2C access to the Renesas 8A34002."""

    def __init__(self, bus: int = 1, addr: int = 0x58):
        self.bus_num = bus
        self.addr = addr
        self._smbus: SMBus | None = None
        self._cur_page: int | None = None

    def __enter__(self) -> "ClockMatrix":
        self._smbus = SMBus(self.bus_num)
        self._cur_page = None
        return self

    def __exit__(self, *_) -> None:
        if self._smbus:
            self._smbus.close()

    def _set_page(self, page: int) -> None:
        """Select register page (high byte of 16-bit address)."""
        if page != self._cur_page:
            self._smbus.write_byte_data(self.addr, PAGE_REG, page)
            self._cur_page = page

    def read_bytes(self, page: int, offset: int, length: int) -> bytes:
        """Read `length` bytes starting at (page, offset)."""
        self._set_page(page)
        data = self._smbus.read_i2c_block_data(self.addr, offset, length)
        return bytes(data)

    def read_byte(self, page: int, offset: int) -> int:
        self._set_page(page)
        return self._smbus.read_byte_data(self.addr, offset)

    def read_device_id(self) -> int:
        """Read 16-bit device ID (little-endian). Expected 0x3400 for 8A34002."""
        raw = self.read_bytes(DEVICE_ID_PAGE, DEVICE_ID_OFFSET, DEVICE_ID_LEN)
        return int.from_bytes(raw, byteorder='little', signed=False)

    def dpll0_locked(self) -> bool:
        """Return True if DPLL0 reports locked status."""
        status = self.read_byte(DPLL0_PHASE_STATUS_PAGE, DPLL0_LOCK_STATUS_OFFSET)
        return bool(status & 0x01)

    def read_tdc_phase_raw(self) -> int:
        """Read 40-bit TDC phase status for DPLL0.

        Returns the raw integer value — a signed fixed-point number where
        bits [31:0] are fractional nanoseconds and bits [39:32] are integer
        nanoseconds (signed).
        """
        raw = self.read_bytes(
            DPLL0_PHASE_STATUS_PAGE,
            DPLL0_PHASE_STATUS_OFFSET,
            DPLL0_PHASE_STATUS_LEN,
        )
        # 5 bytes, little-endian, sign-extend from bit 39.
        val = int.from_bytes(raw, byteorder='little', signed=False)
        if val & (1 << 39):
            val -= (1 << 40)
        return val

    def read_tdc_phase_ps(self) -> float:
        """Read TDC phase offset in picoseconds.

        The raw value is a 40-bit signed fixed-point number:
            integer part  = raw >> 32    (nanoseconds, signed byte)
            fraction part = raw & 0xFFFFFFFF  (1 LSB = 1/2^32 ns ≈ 0.233 ps)
        Converting to ps: raw * (1e3 / 2^32)
        """
        return self.read_tdc_phase_raw() * FRAC_SCALE_PS


# ── Main ─────────────────────────────────────────────────────────────── #

def main():
    ap = argparse.ArgumentParser(
        description="Read Renesas 8A34002 TDC phase error via I2C")
    ap.add_argument("--bus",      type=int, default=1,
                    help="I2C bus number (default: 1)")
    ap.add_argument("--addr",     type=lambda x: int(x, 0), default=0x58,
                    help="I2C device address (default: 0x58)")
    ap.add_argument("--duration", type=int, default=300,
                    help="Sampling duration in seconds (default: 300 = 5 min)")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="Sampling interval in seconds (default: 1.0)")
    ap.add_argument("--csv",      type=str, default=None, dest="csv_path",
                    help="Write CSV output to file (default: stdout)")
    ap.add_argument("--raw",      action="store_true",
                    help="Also output raw TDC counts")
    ap.add_argument("--probe",    action="store_true",
                    help="Probe mode: read device ID and one phase sample, then exit")
    args = ap.parse_args()

    with ClockMatrix(bus=args.bus, addr=args.addr) as cm:
        # Always verify device ID first.
        dev_id = cm.read_device_id()
        print(f"# Device ID: 0x{dev_id:04X}", file=sys.stderr)
        if (dev_id & 0xFF00) != 0x3400:
            print(f"# WARNING: unexpected device ID (expected 0x34xx for 8A34002)",
                  file=sys.stderr)

        locked = cm.dpll0_locked()
        print(f"# DPLL0 locked: {locked}", file=sys.stderr)

        raw = cm.read_tdc_phase_raw()
        phase_ps = raw * FRAC_SCALE_PS
        int_ns  = raw >> FRAC_BITS
        frac_ns = (raw & 0xFFFFFFFF) * (1.0 / (1 << FRAC_BITS))
        print(f"# Phase: {int_ns} ns + {frac_ns:.6f} ns = {phase_ps:.1f} ps",
              file=sys.stderr)
        print(f"# LSB: {FRAC_SCALE_PS:.4f} ps", file=sys.stderr)

        if args.probe:
            print(f"# Raw: 0x{raw & 0xFFFFFFFFFF:010X} ({raw})", file=sys.stderr)
            if not locked:
                print("# WARNING: DPLL0 not locked — phase reading may be invalid",
                      file=sys.stderr)
            return

        if not locked:
            print("# WARNING: DPLL0 not locked — phase readings may be invalid",
                  file=sys.stderr)

        print(f"# Sampling at {args.interval}s intervals for {args.duration}s",
              file=sys.stderr)

    # Reopen for sampling (keeps the context manager scope clean).
    csv_file = open(args.csv_path, "w", newline="") if args.csv_path else sys.stdout
    try:
        fieldnames = ["epoch_s", "phase_ps"]
        if args.raw:
            fieldnames.append("raw_counts")
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        with ClockMatrix(bus=args.bus, addr=args.addr) as cm:
            n_samples = int(args.duration / args.interval)
            t_start = time.time()

            for i in range(n_samples):
                t_target = t_start + (i + 1) * args.interval
                t_now = time.time()
                if t_target > t_now:
                    time.sleep(t_target - t_now)

                t_sample = time.time()
                raw = cm.read_tdc_phase_raw()
                phase_ps = raw * FRAC_SCALE_PS

                row = {"epoch_s": f"{t_sample:.3f}", "phase_ps": f"{phase_ps:.1f}"}
                if args.raw:
                    row["raw_counts"] = str(raw)
                writer.writerow(row)

                if csv_file is not sys.stdout:
                    csv_file.flush()

            elapsed = time.time() - t_start
            print(f"# Done: {n_samples} samples in {elapsed:.1f}s", file=sys.stderr)

    finally:
        if csv_file is not sys.stdout:
            csv_file.close()


if __name__ == "__main__":
    main()

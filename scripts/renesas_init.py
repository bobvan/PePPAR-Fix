#!/usr/bin/env python3
"""renesas_init.py — Initialize Renesas 8A34002 clock tree for peppar-fix.

Replays the clock tree configuration captured from Timebeat so we can
run the DPLL in write_phase_set mode (open loop) without Timebeat.

After initialization:
  - TDC measures phase between CLK14 (PPS) and CLK12 (OCXO)
  - DPLL0 is in write_phase_set mode (pll_mode=3)
  - Phase corrections written to DPLL_CTRL_0.PHASE_OFFSET
  - ITDC_UI ≈ 340 ps (fbd_integer=92), fine ≈ 2.7 ps

Usage:
    # Initialize from captured config:
    python3 renesas_init.py --config data/clocktree_ptboat.json --bus 16

    # Initialize and start reading phase:
    python3 renesas_init.py --config data/clocktree_ptboat.json --bus 16 --read 60

    # Just read phase (assume already initialized):
    python3 renesas_init.py --bus 16 --read-only 60
"""

import argparse
import json
import sys
import time

try:
    import smbus2
except ImportError:
    print("pip install smbus2", file=sys.stderr)
    sys.exit(1)


ITDC_UI_PS = 340.0  # From fbd_integer=92: 1e12/(32*92e6)


class ClockMatrixInit:
    """Initialize and read Renesas 8A34002 ClockMatrix."""

    def __init__(self, bus_num, addr=0x58):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = addr

    def close(self):
        self._restore_page()
        self.bus.close()

    def _set_page(self, page):
        self.bus.write_byte_data(self.addr, 0xFC, page)
        time.sleep(0.002)

    def _restore_page(self):
        try:
            self.bus.write_byte_data(self.addr, 0xFC, 0x00)
        except Exception:
            pass

    def write_config(self, config):
        """Write captured config to the chip.

        Config is a dict of {name: {page, offset, data}} from dump_clocktree.py.
        Skips STATUS registers (read-only).
        Writes trigger registers (DPLL_MODE, INPUT_TDC_CTRL) last.
        """
        # Separate trigger registers (must be written last)
        triggers = {}
        regular = {}
        for name, reg in config.items():
            if name.startswith("STATUS."):
                continue  # Skip read-only status
            if "MODE" in name and "REF_MODE" not in name:
                triggers[name] = reg
            elif "CTRL" in name and "FBD_CTRL" not in name:
                triggers[name] = reg
            else:
                regular[name] = reg

        # Write regular registers first
        for name, reg in regular.items():
            page = reg["page"]
            offset = reg["offset"]
            data = reg["data"]
            try:
                self._set_page(page)
                self.bus.write_i2c_block_data(self.addr, offset, data)
                print(f"  wrote {name} (page 0x{page:02X}, offset 0x{offset:02X}, {len(data)} bytes)")
            except Exception as e:
                print(f"  FAILED {name}: {e}")

        # Write trigger registers last (triggers hardware activation)
        for name, reg in triggers.items():
            page = reg["page"]
            offset = reg["offset"]
            data = reg["data"]
            try:
                self._set_page(page)
                self.bus.write_i2c_block_data(self.addr, offset, data)
                print(f"  triggered {name} (page 0x{page:02X}, offset 0x{offset:02X})")
            except Exception as e:
                print(f"  FAILED trigger {name}: {e}")

        self._restore_page()
        time.sleep(0.1)  # Let hardware settle

    def read_phase_coarse(self):
        """Read DPLL0_PHASE_STATUS: signed 36-bit in ITDC_UIs."""
        try:
            self._set_page(0xC0)
            time.sleep(0.001)
            data = self.bus.read_i2c_block_data(self.addr, 0xDC, 5)
            self._restore_page()

            val = int.from_bytes(bytes(data[:4]), 'little')
            val |= (data[4] & 0x0F) << 32
            if val & (1 << 35):
                val -= (1 << 36)
            if val == 0x7FFFFFFFF:
                return None, None

            ns = val * ITDC_UI_PS / 1000.0
            return val, ns
        except Exception:
            self._restore_page()
            return None, None

    def read_phase_fine(self):
        """Read DPLL0_FILTER_STATUS: signed 48-bit in ITDC_UI/128."""
        try:
            self._set_page(0xC0)
            time.sleep(0.001)
            data = self.bus.read_i2c_block_data(self.addr, 0x24, 6)
            self._restore_page()

            val = int.from_bytes(bytes(data), 'little', signed=True)
            if val == 0x7FFFFFFFFFFF:
                return None, None

            ns = val * ITDC_UI_PS / 128.0 / 1000.0
            return val, ns
        except Exception:
            self._restore_page()
            return None, None


def main():
    ap = argparse.ArgumentParser(description="Initialize 8A34002 clock tree")
    ap.add_argument("--config", help="Clock tree config JSON (from dump_clocktree.py)")
    ap.add_argument("--bus", type=int, default=16)
    ap.add_argument("--addr", type=int, default=0x58)
    ap.add_argument("--read", type=int, help="Read phase for N seconds after init")
    ap.add_argument("--read-only", type=int, help="Just read phase (skip init)")
    args = ap.parse_args()

    cm = ClockMatrixInit(args.bus, args.addr)

    try:
        if args.config and not args.read_only:
            print(f"Loading config from {args.config}...")
            with open(args.config) as f:
                config = json.load(f)
            print(f"Writing {len(config)} registers...")
            cm.write_config(config)
            print("Clock tree initialized.")
            print()

        duration = args.read or args.read_only
        if duration:
            print(f"Reading phase for {duration}s (ITDC_UI={ITDC_UI_PS:.1f}ps)...")
            print(f"{'#':>4}  {'Coarse (ns)':>14}  {'Fine (ns)':>14}")
            print(f"{'----':>4}  {'----------':>14}  {'----------':>14}")

            for i in range(duration):
                raw_c, ns_c = cm.read_phase_coarse()
                raw_f, ns_f = cm.read_phase_fine()
                c_str = f"{ns_c:14.3f}" if ns_c is not None else "      LOS/err"
                f_str = f"{ns_f:14.6f}" if ns_f is not None else "      LOS/err"
                print(f"{i:>4}  {c_str}  {f_str}")
                sys.stdout.flush()
                time.sleep(1)

    except KeyboardInterrupt:
        pass
    finally:
        cm.close()


if __name__ == "__main__":
    main()

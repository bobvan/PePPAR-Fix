#!/usr/bin/env python3
"""Count TIM-TM2 (F9T EXTINT) and TICC chA/chB events per second in realtime.

Use this while tuning divider termination.  A clean 1 PPS produces
exactly **one rising edge per second** on each measurement chain.
Two/sec means ringing is double-triggering the timestamper — exactly
the artifact diagnosed on PiFace overnight 2026-05-24 → 25 (F9T
TIM-TM2 ``count`` incrementing by 2/sec, while clkPoC3's increments
by 1/sec).

The script does not write the F9T config.  Most lab hosts already
have ``CFG_MSGOUT_UBX_TIM_TM2_*=1`` set from a prior engine run
(the setting persists in receiver RAM).  If you see 0 TIM-TM2/sec
even with PPS connected, start the engine briefly to enable it, or
run with ``--enable-tim-tm2`` to push the config bit ourselves.

Stop the engine before running so the GNSS and TICC serial ports
are free.

Usage on PiFace (typical)::

    sudo systemctl stop peppar-fix     # or however the engine is run
    cd ~/peppar-fix
    venv/bin/python tools/divider_rate_check.py \\
        --gnss-port /dev/gnss-top --gnss-baud 115200 \\
        --ticc-port /dev/ticc1
"""
from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "scripts"))

import serial  # noqa: E402
from pyubx2 import UBX_PROTOCOL, UBXMessage, UBXReader  # noqa: E402

from ticc import Ticc  # type: ignore  # noqa: E402

_stop = threading.Event()


class Counter:
    """Thread-safe per-second event counter with latest-message metadata."""

    def __init__(self):
        self.lock = threading.Lock()
        self.delta = 0          # rising-edge count delta within this 1-sec bucket
        self.msgs = 0           # TIM-TM2 message count (for sanity, separate from edges)
        self.last_flags = None
        self.last_count_cum = None

    def add_edges(self, n_edges, flags=None, cum_count=None):
        with self.lock:
            self.delta += n_edges
            self.msgs += 1
            if flags is not None:
                self.last_flags = flags
            if cum_count is not None:
                self.last_count_cum = cum_count

    def snapshot_reset(self):
        with self.lock:
            d = self.delta
            m = self.msgs
            f = self.last_flags
            c = self.last_count_cum
            self.delta = 0
            self.msgs = 0
            return d, m, f, c


extint = Counter()
chA_cnt = Counter()
chB_cnt = Counter()


def gnss_thread(port, baud, enable_cfg, port_name):
    """Listen for TIM-TM2 and feed the EXTINT counter with rising-edge deltas."""
    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"  [error] could not open GNSS port {port}: {e}", file=sys.stderr)
        return
    try:
        if enable_cfg:
            try:
                key = f"CFG_MSGOUT_UBX_TIM_TM2_{port_name}"
                msg = UBXMessage.config_set(layers=1, transaction=0,
                                            cfgData=[(key, 1)])
                ser.write(msg.serialize())
                print(f"  [info] wrote {key}=1 (RAM layer)", file=sys.stderr)
            except Exception as e:
                print(f"  [warn] could not enable TIM-TM2: {e}", file=sys.stderr)

        rdr = UBXReader(ser, protfilter=UBX_PROTOCOL)
        prev_count = None
        for _raw, parsed in rdr:
            if _stop.is_set():
                break
            if parsed is None or getattr(parsed, "identity", "") != "TIM-TM2":
                continue
            cum = int(getattr(parsed, "count", 0))
            flags = int(getattr(parsed, "flags", 0))
            if prev_count is None:
                # First message: anchor the counter, don't credit historical edges.
                prev_count = cum
                extint.add_edges(0, flags=flags, cum_count=cum)
                continue
            n_new = cum - prev_count
            if n_new < 0:
                # count is u32; wrap is astronomically unlikely but handle anyway
                n_new = 0
            prev_count = cum
            extint.add_edges(n_new, flags=flags, cum_count=cum)
    except Exception as e:
        print(f"  [error] GNSS reader: {e}", file=sys.stderr)
    finally:
        try:
            ser.close()
        except Exception:
            pass


def ticc_thread(port):
    try:
        with Ticc(port, wait_for_boot=False) as ticc:
            for ch, _ref_sec, _ref_ps in ticc:
                if _stop.is_set():
                    break
                if ch == "chA":
                    chA_cnt.add_edges(1)
                elif ch == "chB":
                    chB_cnt.add_edges(1)
    except Exception as e:
        print(f"  [error] TICC reader: {e}", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--gnss-port", default="/dev/gnss-top",
                    help="F9T serial device (default: /dev/gnss-top)")
    ap.add_argument("--gnss-baud", type=int, default=115200,
                    help="F9T serial baud (default: 115200)")
    ap.add_argument("--gnss-port-name", default="UART1",
                    choices=["UART1", "UART2", "USB", "I2C", "SPI"],
                    help="F9T port name used in CFG keys (default: UART1)")
    ap.add_argument("--enable-tim-tm2", action="store_true",
                    help="Push CFG_MSGOUT_UBX_TIM_TM2_<port>=1 (RAM layer) at start. "
                         "Only needed if TIM-TM2 is not already enabled.")
    ap.add_argument("--ticc-port", default="/dev/ticc1",
                    help="TICC serial device (default: /dev/ticc1)")
    ap.add_argument("--no-ticc", action="store_true",
                    help="Skip TICC reader (EXTINT only)")
    ap.add_argument("--no-gnss", action="store_true",
                    help="Skip GNSS reader (TICC only)")
    args = ap.parse_args()

    if args.no_ticc and args.no_gnss:
        print("error: nothing to do with --no-ticc and --no-gnss", file=sys.stderr)
        return 1

    signal.signal(signal.SIGINT, lambda *_: _stop.set())
    signal.signal(signal.SIGTERM, lambda *_: _stop.set())

    print("divider_rate_check — Ctrl-C to stop\n")
    if not args.no_gnss:
        print(f"  GNSS:  {args.gnss_port} @ {args.gnss_baud} (port {args.gnss_port_name})")
    if not args.no_ticc:
        print(f"  TICC:  {args.ticc_port}")
    print()

    threads = []
    if not args.no_gnss:
        t = threading.Thread(target=gnss_thread,
                             args=(args.gnss_port, args.gnss_baud,
                                   args.enable_tim_tm2, args.gnss_port_name),
                             daemon=True)
        t.start()
        threads.append(t)
    if not args.no_ticc:
        t = threading.Thread(target=ticc_thread, args=(args.ticc_port,), daemon=True)
        t.start()
        threads.append(t)

    # Header
    print(f"{'time':>8}  {'EXTINT/s':>9}  {'flags':>6}  {'count':>9}  {'msgs/s':>6}"
          f"  {'chA/s':>6}  {'chB/s':>6}  notes")
    print("-" * 78)

    next_t = time.monotonic() + 1.0
    while not _stop.is_set():
        sleep_left = next_t - time.monotonic()
        if sleep_left > 0:
            _stop.wait(min(sleep_left, 0.1))
            continue
        next_t += 1.0

        e_edges, e_msgs, fl, cum = extint.snapshot_reset()
        a_edges, _, _, _ = chA_cnt.snapshot_reset()
        b_edges, _, _, _ = chB_cnt.snapshot_reset()
        ts = time.strftime("%H:%M:%S")
        flag_s = f"0x{fl:02X}" if fl is not None else "-"
        cum_s = str(cum) if cum is not None else "-"

        notes = []
        for name, n in (("EXTINT", e_edges), ("chA", a_edges), ("chB", b_edges)):
            if n > 1:
                notes.append(f"{name}={n}")
        warn = ("  ← MULTIPLE EDGES: " + ", ".join(notes)) if notes else ""

        gnss_alive = "" if (e_msgs > 0 or args.no_gnss) else "  ← no TIM-TM2 messages"
        if not warn and gnss_alive:
            warn = gnss_alive

        print(f"{ts:>8}  {e_edges:>9d}  {flag_s:>6}  {cum_s:>9}  {e_msgs:>6d}"
              f"  {a_edges:>6d}  {b_edges:>6d}{warn}")
        sys.stdout.flush()

    return 0


if __name__ == "__main__":
    sys.exit(main())

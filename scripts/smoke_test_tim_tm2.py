#!/usr/bin/env python3
"""TIM-TM2 smoke test for the gnss-phase-experiment branch.

Setup (do these BEFORE running this script):
  1. Wire DO PPS to the F9T's EXTINT pin (one new wire on PiFace).
  2. Bootstrap PHC so DO PPS edges fire near integer GPS seconds —
     run the orchestration wrapper briefly on a branch with a working
     servo, let it lock to within tens of ns of GPS time, then stop it.
  3. Confirm no other process owns the F9T serial port.

What this script does:
  * Opens the F9T at the configured port and baud.
  * Sends a VALSET (RAM-only — non-persistent) enabling
    CFG-MSGOUT-UBX_TIM_TM2_USB so each EXTINT edge produces a
    TIM-TM2 message.
  * Listens for TIM-TM2 messages and prints, for each:
      - rising-edge GPS time (week, towMs, towSubMs)
      - F9T's accuracy estimate (accEst, ns)
      - how far from the top of GPS second the edge actually was
        (signed, ns; small means PHC is well-aligned)

Expected output if the wiring is right and PHC is reasonably
aligned: TIM-TM2 messages once per second with `in-second` residuals
in the tens-of-ns to single-µs range.

If you see *no* TIM-TM2 messages: the F9T isn't seeing the DO PPS
edge — check the EXTINT wire, edge polarity, or whether EXTINT is
already configured for some other purpose (e.g., wakeup).

If you see TIM-TM2 messages but `in-second` is many ms or seconds:
PHC isn't bootstrapped to GPS time.  Run the engine to bootstrap
first, then re-run this script.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.receiver import open_receiver, send_cfg


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/serial/by-path/platform-fe9c0000.xhci-usb-0:1.2:1.0",
                   help="F9T serial device (default: PiFace USB by-path)")
    p.add_argument("--baud", type=int, default=460800,
                   help="baud rate (default: 460800 for F9T at TIM 2.25)")
    p.add_argument("--duration", type=int, default=30,
                   help="seconds to listen (default: 30)")
    p.add_argument("--ubx-port", default="USB",
                   choices=["USB", "UART1", "UART2", "I2C", "SPI"],
                   help="F9T port to enable TIM-TM2 output on (default: USB)")
    args = p.parse_args()

    print(f"# Opening {args.port} at {args.baud} baud")
    ser, ubr = open_receiver(args.port, args.baud)

    # RAM-layer VALSET (layers=1) so this enable doesn't persist —
    # smoke test only.  Production wiring will land via the proper
    # configure_messages path on the gnss-phase-experiment branch.
    cfg = {f"CFG_MSGOUT_UBX_TIM_TM2_{args.ubx_port}": 1}
    print(f"# Enabling TIM-TM2 on {args.ubx_port} (RAM only)")
    ok = send_cfg(ser, ubr, cfg, "TIM-TM2 smoke test enable", layers=1)
    if not ok:
        print("# WARN: VALSET ACK not received — proceeding to listen anyway",
              file=sys.stderr)

    print(f"# Listening for TIM-TM2 for {args.duration} seconds...")
    print("# fields: count, week, towMs, towSubMs, accEst, in-second-ns")
    deadline = time.monotonic() + args.duration
    n_tm2 = 0
    n_other = 0
    while time.monotonic() < deadline:
        try:
            raw, parsed = ubr.read()
        except Exception:
            continue
        if parsed is None:
            continue
        if parsed.identity != "TIM-TM2":
            n_other += 1
            continue
        n_tm2 += 1
        wnR = getattr(parsed, "wnR", 0)
        towMsR = getattr(parsed, "towMsR", 0)
        towSubMsR = getattr(parsed, "towSubMsR", 0)
        accEst = getattr(parsed, "accEst", -1)
        count = getattr(parsed, "count", 0)
        flags = getattr(parsed, "flags", 0)
        # towMs is in milliseconds, towSubMs is in nanoseconds.
        # in-second residual = where in the GPS second the edge fell.
        ms_in_second = towMsR % 1000
        in_second_ns = ms_in_second * 1_000_000 + towSubMsR
        # Wrap so values close to the next second show as small negatives.
        if in_second_ns > 500_000_000:
            in_second_ns -= 1_000_000_000
        print(f"TIM-TM2 #{n_tm2:3d} count={count:5d} "
              f"wn={wnR} towMs={towMsR} towSubMs={towSubMsR:>12d}ns "
              f"accEst={accEst:>6d}ns flags=0x{flags:02x} "
              f"→ in-second={in_second_ns:+d} ns")

    ser.close()
    print(f"# Done.  TIM-TM2 received: {n_tm2}, other UBX: {n_other}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Hold a SparkFun GNSSDO+ (SXT-D) OCXO FREE-RUNNING for characterization.

To measure the STP3593LF's free-running noise floor (for do_freerun_char /
build_do_characterization → the DO's Kalman Q), the OCXO must be disciplined
by NEITHER SparkFun's internal loop NOR PePPAR-Fix.  This tool:

  1. takes external control ($E,1) so SparkFun's loop stops steering, and
  2. re-writes the SAME control word on a timer (every --interval seconds),
     which changes nothing frequency-wise but KICKS the firmware watchdog so
     it doesn't reclaim the oscillator and resume internal discipline.

The oscillator then free-runs at a fixed control word (constant frequency
command) — its phase drifts and wanders with its intrinsic noise, which the
TICC (chA = GNSSDO+ PPS OUT vs a stable reference) captures.  Analyse chA
detrended (do_freerun_char / tools/plot_chA_tdev_goldilocks.py) for the
free-run ADEV/TDEV.

On exit (Ctrl-C, SIGTERM, or --duration elapsed) it hands the oscillator back
to SparkFun's internal discipline ($E,0).  The firmware watchdog is a second
safety net: if this process dies, internal discipline resumes within the
watchdog timeout regardless.

Example (run alongside a TICC capture of chA):
    scripts/gnssdo_freerun_hold.py --serial /dev/ttyUSB0 --interval 10
"""
import argparse
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from peppar_fix.gnssdo_actuator import GnssdoActuator  # noqa: E402

# STP3593LF datasheet default; the exact value is irrelevant for a HOLD (we
# write the anchor word, delta 0), but the actuator requires a non-zero slope.
_DEFAULT_PPB_PER_CODE = 7.885e-4


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", default="/dev/ttyUSB0",
                    help="ESP32 console serial device (default /dev/ttyUSB0)")
    ap.add_argument("--tcp", default=None, metavar="HOST:PORT",
                    help="Use the mosaic-T IPS console backdoor instead of serial")
    ap.add_argument("--interval", type=float, default=10.0,
                    help="Seconds between watchdog-kick re-writes (default 10; "
                         "must be < --watchdog)")
    ap.add_argument("--watchdog", type=int, default=60,
                    help="Firmware fail-safe watchdog to request via $T "
                         "(default 60 s; must exceed --interval)")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="Seconds to hold, then hand back (0 = until "
                         "interrupted; default 0)")
    ap.add_argument("--ppb-per-code", type=float, default=_DEFAULT_PPB_PER_CODE,
                    help="Actuator slope (only needed to construct the actuator; "
                         "a HOLD writes the anchor word regardless)")
    args = ap.parse_args()

    if args.interval >= args.watchdog:
        ap.error("--interval (%.1f) must be < --watchdog (%d) or the firmware "
                 "will reclaim the oscillator between kicks"
                 % (args.interval, args.watchdog))

    tcp = None
    if args.tcp:
        host, _, port = args.tcp.partition(":")
        tcp = (host, int(port) if port else 28784)

    act = GnssdoActuator(serial_port=None if tcp else args.serial, tcp=tcp,
                         ppb_per_code=args.ppb_per_code,
                         watchdog_s=args.watchdog)

    stop = {"now": False}

    def _handle(signum, _frame):
        stop["now"] = True
    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)

    act.setup()   # $T,<watchdog>, $E,1, anchor at the current word
    print("FREERUN HOLD: external control taken; holding control word %d "
          "(re-write every %.0fs, watchdog %ds).  The OCXO is now free-running "
          "— capture chA on the TICC.  Ctrl-C to hand back."
          % (act.current_word, args.interval, args.watchdog), flush=True)

    t0 = time.monotonic()
    n = 0
    try:
        while not stop["now"]:
            # Writing the anchor word (0 ppb delta) is a no-op frequency-wise
            # but re-arms the watchdog and re-asserts the held word.
            act.adjust_frequency_ppb(0.0)
            n += 1
            if n % 6 == 0:   # ~once a minute at 10 s interval
                try:
                    ext, word, bias, state = act._status()
                    print("  held: word=%d bias=%.3es state=%s ext=%d (%.0fs)"
                          % (word, bias, state, ext, time.monotonic() - t0),
                          flush=True)
                except Exception as e:   # noqa: BLE001 - status is best-effort
                    print("  (status read failed: %s)" % e, flush=True)
            if args.duration and (time.monotonic() - t0) >= args.duration:
                break
            # Sleep in short slices so a signal is handled promptly.
            slept = 0.0
            while slept < args.interval and not stop["now"]:
                time.sleep(min(0.5, args.interval - slept))
                slept += 0.5
    finally:
        act.teardown()   # $E,0 — internal discipline resumes
        print("FREERUN HOLD: released; internal discipline resumed.", flush=True)


if __name__ == "__main__":
    main()

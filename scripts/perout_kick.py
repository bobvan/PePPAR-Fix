#!/usr/bin/env python3
"""One-off PEROUT enable for hosts where the engine bootstrap aborted.

Background (2026-05-02): the engine-absorption refactor (Apr 14-15)
moved phc_bootstrap.py's PEROUT-enable into ``_do_bootstrap_phc``
which is gated on EXTTS-PPS being available at startup.  When EXTTS
sees no PPS at startup (TimeHat / MadHat documented behaviour), the
engine's bootstrap aborts and the PEROUT setup chain is never called.

Originally (2026-05-02) this utility just issued ``enable_perout`` and
relied on the pin function persisting from a prior process.  That
broke (2026-05-16) when the engine's clean teardown disabled PEROUT
between runs — re-enabling alone doesn't restore the SDP pin
assignment and the i226 half-period latch bug isn't worked around.

Now (2026-05-16) this utility runs the **full setup chain** —
PIN_SETFUNC (with sysfs fallback for E810), enable_perout, and
optional TICC-driven phase verification with retry — via
``peppar_fix.perout_setup.setup_perout``.  Same code path as the
engine's bootstrap and ``tools/calibrate_do.py``.  See dayplan
item ``calToolPhcPerout-main``.

Usage:
    PYTHONPATH=scripts python3 scripts/perout_kick.py
    PYTHONPATH=scripts python3 scripts/perout_kick.py \\
        --ptp-dev /dev/ptp_i226 --channel 0 --sdp-pin 0
    PYTHONPATH=scripts python3 scripts/perout_kick.py \\
        --ptp-dev /dev/ptp_i226 --channel 0 --sdp-pin 0 \\
        --verify-via-ticc /dev/ticc1
    PYTHONPATH=scripts python3 scripts/perout_kick.py --disable
"""

from __future__ import annotations

import argparse
import logging
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument(
        "--ptp-dev", default="/dev/ptp_i226",
        help="PTP device (default: /dev/ptp_i226)")
    parser.add_argument(
        "--channel", type=int, default=0,
        help="PEROUT channel (default: 0)")
    parser.add_argument(
        "--sdp-pin", type=int, default=0,
        help="SDP pin index for PEROUT output (default: 0 = SDP0)")
    parser.add_argument(
        "--period-ns", type=int, default=1_000_000_000,
        help="Pulse period in ns (default: 1e9 = 1Hz / 1PPS)")
    parser.add_argument(
        "--no-program-pin", dest="program_pin", action="store_false",
        default=True,
        help="Skip the PIN_SETFUNC ioctl (use if pin is already "
             "assigned and you only need PEROUT re-enabled)")
    parser.add_argument(
        "--verify-via-ticc", default=None, metavar="TICC_PORT",
        help="Read this TICC port to verify PEROUT phase alignment "
             "after programming (e.g. /dev/ticc1).  Retries with "
             "alternate start_nsec if the i226 half-period latch "
             "lands on the wrong polarity.")
    parser.add_argument(
        "--disable", action="store_true",
        help="Disable the PEROUT instead of enabling it")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s %(levelname)s %(message)s')
    log = logging.getLogger("perout_kick")

    from peppar_fix.ptp_device import PtpDevice

    ptp = PtpDevice(args.ptp_dev)
    log.info("Opened %s", args.ptp_dev)

    if args.disable:
        ptp.disable_perout(args.channel)
        log.info("Disabled PEROUT on channel %d", args.channel)
        return 0

    from peppar_fix.perout_setup import setup_perout
    ok = setup_perout(
        ptp,
        pin_index=args.sdp_pin,
        channel=args.channel,
        period_ns=args.period_ns,
        program_pin=args.program_pin,
        ptp_dev_path=args.ptp_dev,
        verify_via_ticc_port=args.verify_via_ticc,
    )
    if not ok:
        log.error("PEROUT setup failed")
        return 1
    if not args.verify_via_ticc:
        log.info("Verify by reading TICC chA timestamps over the next "
                 "few seconds — pulses should arrive at 1 Hz.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

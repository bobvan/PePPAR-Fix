#!/usr/bin/env python3
"""One-off PEROUT enable for hosts where the engine bootstrap aborted.

Background (2026-05-02): the engine-absorption refactor (Apr 14-15)
moved phc_bootstrap.py's PEROUT-enable into ``_do_bootstrap_phc``
which is gated on EXTTS-PPS being available at startup.  When EXTTS
sees no PPS at startup (TimeHat / MadHat documented behaviour), the
engine's bootstrap aborts and ``enable_perout`` is never called.
Pin function persists from a prior process so ``/sys/class/ptp/...
/pins/SDP0`` still shows function=2 (PEROUT), but no
``PTP_PEROUT_REQUEST`` ioctl ever ran, so no pulses emit.  TICC
chA stays empty.

This utility opens the PTP device alongside the running engine and
issues the missing PEROUT request.  Multiple processes can hold the
PTP fd; once the request lands, the hardware pulses regardless of
which process originated it.  Safe to run while the engine is live —
the engine doesn't manage PEROUT (its bootstrap aborted), so no
conflict.

Usage:
    PYTHONPATH=scripts python3 scripts/perout_kick.py
    PYTHONPATH=scripts python3 scripts/perout_kick.py --ptp-dev /dev/ptp_i226 --channel 0
    PYTHONPATH=scripts python3 scripts/perout_kick.py --disable

The "real" structural fix is to decouple ``enable_perout`` from
the bootstrap-success path in ``_do_bootstrap_phc``.  See the
companion dayplan item.
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
        "--period-ns", type=int, default=1_000_000_000,
        help="Pulse period in ns (default: 1e9 = 1Hz / 1PPS)")
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

    ptp.enable_perout(args.channel, period_ns=args.period_ns)
    log.info("PEROUT enabled on channel %d (period=%d ns)",
             args.channel, args.period_ns)
    log.info("Verify by reading TICC chA timestamps over the next few "
             "seconds — pulses should arrive at 1 Hz.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""peppar-arp-moved — declare that the antenna has physically moved.

Run this after you physically relocate an antenna (or swap which antenna a
receiver is connected to) **with the engine stopped**.

Why it's needed: the engine's NAV2 watchdog already detects a move and bumps
``mount_sn`` automatically — but only while it's running.  The ordinary
physical case is the opposite: stop the engine, unplug the antenna, drive to
the new site, plug it back in.  Nothing was watching, so nothing bumped
``mount_sn``, and at the next launch the pre-move ``.survey.toml`` still
matches the (unbumped) current mount_sn and seeds the engine at the OLD site.
The engine then pins a position that is kilometres wrong while reporting a
centimetre σ, because a stale pin's σ is *small*.  That is the ptBoat
stale-Chicago-pin failure, and nothing about it is loud.

What it does (the same state effects as WATCHDOG_STEP_AUTO_MOVE, minus the
servo demotion and exit-5 — there's no running engine to demote):

  * bumps ``mount_sn`` in ``state/receivers/<uid>.json``, so every pre-move
    ``.survey.toml`` / ``.ppp.toml`` is filtered out of seed selection
  * deletes ``state/positions/<uid>.ppp.toml``

It deliberately leaves the old ``.survey.toml`` in place: it is the
authoritative record of the old site, and mount_sn filtering already makes it
unusable as a seed.

Usage:

    peppar_arp_moved.py                    # single-receiver host: auto-discover
    peppar_arp_moved.py --receiver-uid ... # explicit
    peppar_arp_moved.py --dry-run          # show what would change
"""

from __future__ import annotations

import argparse
import logging
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from peppar_fix.position_state import (  # noqa: E402
    declare_arp_moved, load_current_mount_sn,
)
from peppar_survey import discover_single_receiver_uid  # noqa: E402

log = logging.getLogger("peppar-arp-moved")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Run with the engine stopped, after the antenna is in its "
               "new position.",
    )
    ap.add_argument("--receiver-uid",
                    help="Receiver SEC-UNIQID or synthetic UID.  "
                         "Auto-discovered from state/receivers/ when there's "
                         "exactly one receiver.")
    ap.add_argument("--receivers-dir", default=None,
                    help="Override state/receivers/ directory.")
    ap.add_argument("--positions-dir", default=None,
                    help="Override state/positions/ directory.")
    ap.add_argument("--dry-run", action="store_true",
                    help="Report what would change; write nothing.")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    uid = args.receiver_uid
    if uid is None:
        from peppar_fix.receiver_state import DEFAULT_STATE_DIR
        uid = discover_single_receiver_uid(args.receivers_dir
                                           or DEFAULT_STATE_DIR)
        if uid is None:
            return 1
        log.info("Auto-discovered receiver_uid=%s", uid)

    if args.dry_run:
        old = load_current_mount_sn(uid, receivers_dir=args.receivers_dir)
        log.info("[DRY RUN] would bump mount_sn %d → %d for uid=%s and "
                 "delete its .ppp.toml", old, old + 1, uid)
        return 0

    result = declare_arp_moved(uid, receivers_dir=args.receivers_dir,
                               positions_dir=args.positions_dir)
    print(f"mount_sn: {result['old_mount_sn']} -> {result['new_mount_sn']}")
    print(f"ppp invalidated: {result['ppp_invalidated']}")
    print("Next: re-survey the new position (peppar-survey --auto ...); "
          "until it lands the engine bootstraps from NAV2.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

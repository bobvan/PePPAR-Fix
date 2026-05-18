#!/usr/bin/env python3
"""peppar-survey — write state/positions/<uid>.survey.toml from
external authoritative observations.

Optional companion to the engine, per
``docs/position-state-and-monitoring.md``.  Triggered by a systemd
timer, ``@daily`` cron, or any external orchestrator.

The contract is:

  Input:  external authoritative observation source — captured
          RINEX submitted to OPUS or processed by PRIDE, or a
          quick NTRIP CORS-RTK check against a nearby reference.
  Output: atomic temp+rename write of
          state/positions/<uid>.survey.toml with current mount_sn
          tag, fresh ECEF, sigma, and provenance metadata.

Naming convention: "survey" is reserved for external authoritative
observations.  The engine's own AntPosEst output is a "PPP
solution" and goes to .ppp.toml (written by the engine, not by
peppar-survey).  This CLI cannot write a .survey.toml from a PPP
snapshot — that confusion was the root cause of the TimeHat
mount_sn=1 incident 2026-05-18 (an early-epoch PPP snapshot
promoted to "survey" via the now-removed --from-ppp backend
seeded the engine with a 0.95 m sigma estimate that AntPosEst
later detected was 1.5 m off, triggering an auto-step that
ultimately did the right thing but cost an engine restart).

Backends (none implemented in this skeleton):

  --pride <work_dir>  Wrap PRIDE PPP-AR over the captured RINEX,
                      extract the daily solution, append to a
                      history.jsonl, and aggregate via running_mean
                      from scripts/peppar_fix/arp_history.py.
  --opus              Submit to NGS OPUS-Static and parse the result.
  --cors              Quick NTRIP CORS-RTK check against a nearby
                      reference station (minutes-class accuracy).
  --rtklib            Local RTKLIB PPP run.

When a real backend lands, the contract above stays the same — the
caller (this CLI) is unchanged; only the backend module differs.
Until then, peppar-survey errors out at argparse stage: there's no
backend to run.  This is by design — installations without survey
infrastructure simply rely on AntPosEst's own warm-start via
.ppp.toml.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys


log = logging.getLogger("peppar-survey")

# Make peppar_fix importable when run from the repo root.
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))


def discover_single_receiver_uid(receivers_dir: str) -> str | None:
    """Find the single receiver UID from state/receivers/.  Returns
    None when zero or multiple receiver files exist — caller must
    pass --receiver-uid explicitly in those cases."""
    if not os.path.isdir(receivers_dir):
        return None
    uids = []
    for name in os.listdir(receivers_dir):
        if name.endswith(".json") and not name.endswith(".bak"):
            # Skip the *.json.<tag>.bak files even though they don't
            # end in .bak — common pattern is e.g. ".day0424i.bak".
            if ".bak." in name or name.endswith(".bak"):
                continue
            uid = name[:-5]  # strip ".json"
            uids.append(uid)
    if len(uids) == 1:
        return uids[0]
    if len(uids) == 0:
        log.error("No receivers found in %s", receivers_dir)
    else:
        log.error("Multiple receivers found in %s (%s) — pass "
                  "--receiver-uid", receivers_dir, ", ".join(uids))
    return None


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=("Detailed usage in the module docstring at the top "
                "of this file."),
    )
    ap.add_argument("--receiver-uid",
                    help="Receiver SEC-UNIQID (decimal string) or "
                         "synthetic UID (e.g. synth_D30GD1PE).  "
                         "Auto-discovered from state/receivers/ "
                         "when there's exactly one receiver.")
    ap.add_argument("--positions-dir", default=None,
                    help="Override state/positions/ directory.")
    ap.add_argument("--receivers-dir", default=None,
                    help="Override state/receivers/ directory.")
    ap.add_argument("--dry-run", action="store_true",
                    help="Compute the survey snapshot, log what would "
                         "be written, but don't write the file.  "
                         "(No-op until a backend is implemented.)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    # Auto-discover receiver UID (still useful — future backends will
    # want it, and this exercises that path).
    if args.receiver_uid is None:
        from peppar_fix.receiver_state import DEFAULT_STATE_DIR
        rdir = args.receivers_dir or DEFAULT_STATE_DIR
        uid = discover_single_receiver_uid(rdir)
        if uid is None:
            return 1
        args.receiver_uid = uid
        log.info("Auto-discovered receiver_uid=%s", uid)

    # No backend implemented yet.  Error out explicitly so operators
    # see this is a stub, not a silent success.
    log.error(
        "peppar-survey has no backend implemented yet.  "
        "Future backends: --pride, --opus, --cors, --rtklib.  "
        "Until one lands, the engine's own .ppp.toml warm-start is "
        "the only seeding path."
    )
    return 2


if __name__ == "__main__":
    sys.exit(main())

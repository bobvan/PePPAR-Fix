#!/usr/bin/env python3
"""peppar-survey — write state/positions/<uid>.survey.toml.

Optional companion to the engine, per
``docs/position-state-and-monitoring.md``.  Triggered by a systemd
timer, ``@daily`` cron, or any external orchestrator.

The contract is just:

  Input:  zero or more data/rinex/<uid>-*.obs files (or whatever
          your backend understands).
  Output: atomic temp+rename write of
          state/positions/<uid>.survey.toml with current mount_sn
          tag, fresh ECEF, sigma, and provenance metadata.

This skeleton supports one backend today:

  --from-ppp   Copy the engine's own .ppp.toml to .survey.toml.
               Useful for bootstrapping the warm-start path on hosts
               that don't yet have PRIDE / OPUS / RTKLIB wired up.
               The "survey" produced is no better than the engine's
               own AntPosEst convergence, but it does exercise the
               .survey.toml read path and gets the engine into
               survey-driven mode on next restart.

Future backends (not implemented in slice 8):

  --pride <work_dir>  Wrap PRIDE PPP-AR over the captured RINEX,
                      extract the daily solution, append to a
                      history.jsonl, and aggregate via running_mean
                      from scripts/peppar_fix/arp_history.py.
  --opus              Submit to NGS OPUS-Static and parse the result.
  --rtklib            Local RTKLIB PPP run.

When a real backend lands, the contract above stays the same — the
caller (this CLI) is unchanged; only the backend module differs.

Operator usage:

  # Just write .survey.toml from the last .ppp.toml snapshot
  ./scripts/peppar_survey.py --receiver-uid 12345 --from-ppp

  # Dry-run (compute what would be written, don't write)
  ./scripts/peppar_survey.py --receiver-uid 12345 --from-ppp --dry-run

  # Auto-discover receiver UID from state/receivers/ (single receiver)
  ./scripts/peppar_survey.py --from-ppp
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


def survey_from_ppp(uid: str,
                    *,
                    positions_dir: str | None = None,
                    dry_run: bool = False) -> int:
    """Copy state/positions/<uid>.ppp.toml to .survey.toml.

    Returns shell-style exit code (0 success, non-zero error).
    """
    from peppar_fix.position_state import (
        PositionState, load_ppp_state, save_ppp_state, utc_now_iso,
    )

    src = load_ppp_state(uid, positions_dir=positions_dir)
    if src is None:
        log.error("No .ppp.toml for uid=%s (engine hasn't written one "
                  "yet, or σ never dropped below the write gate)", uid)
        return 1

    # Build the survey snapshot.  Preserves the PPP estimate's ECEF
    # and sigma, retags source/extra to make it clear this came from
    # the engine's own PPP rather than an independent survey.
    out = PositionState(
        mount_sn=src.mount_sn,
        ecef_m=src.ecef_m,
        sigma_m=src.sigma_m,
        updated=utc_now_iso(),
        source="peppar-survey --from-ppp (engine PPP snapshot)",
        kind="ppp",  # save_ppp_state requires "ppp"; renamed for survey write below
        extra={
            "from_ppp_n_epochs": src.extra.get("n_epochs", 0),
            "from_ppp_source": src.source,
            "from_ppp_updated": src.updated,
        },
    )

    if dry_run:
        log.info("DRY RUN — would write:")
        log.info("  mount_sn = %d", out.mount_sn)
        log.info("  ecef_m   = (%.4f, %.4f, %.4f)", *out.ecef_m)
        log.info("  sigma_m  = %.6f", out.sigma_m)
        log.info("  source   = %s", out.source)
        log.info("  extra    = %s", out.extra)
        return 0

    # save_ppp_state writes to .ppp.toml; we want .survey.toml.  The
    # actual write is small and the schema is identical, so call the
    # same TOML formatter and target the survey path directly.
    from peppar_fix.position_state import (
        _format_toml, DEFAULT_POSITIONS_DIR,
    )
    d = positions_dir or DEFAULT_POSITIONS_DIR
    os.makedirs(d, exist_ok=True)
    survey_path = os.path.join(d, f"{uid}.survey.toml")
    tmp = survey_path + ".tmp"
    with open(tmp, "w") as f:
        f.write(_format_toml(out))
    os.replace(tmp, survey_path)
    log.info("Wrote %s (mount_sn=%d, σ=%.4fm)",
             survey_path, out.mount_sn, out.sigma_m)
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=("Detailed usage in the module docstring at the top "
                "of this file."),
    )
    ap.add_argument("--receiver-uid",
                    help="Receiver SEC-UNIQID (decimal string).  "
                         "Auto-discovered from state/receivers/ "
                         "when there's exactly one receiver.")
    ap.add_argument("--positions-dir", default=None,
                    help="Override state/positions/ directory.")
    ap.add_argument("--receivers-dir", default=None,
                    help="Override state/receivers/ directory.")
    ap.add_argument("--from-ppp", action="store_true",
                    help="Use the engine's own .ppp.toml as the "
                         "survey source.")
    ap.add_argument("--dry-run", action="store_true",
                    help="Compute the survey snapshot, log what would "
                         "be written, but don't write the file.")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    # Auto-discover receiver UID.
    if args.receiver_uid is None:
        from peppar_fix.receiver_state import DEFAULT_STATE_DIR
        rdir = args.receivers_dir or DEFAULT_STATE_DIR
        uid = discover_single_receiver_uid(rdir)
        if uid is None:
            return 1
        args.receiver_uid = uid
        log.info("Auto-discovered receiver_uid=%s", uid)

    if not args.from_ppp:
        ap.error("No backend selected.  Pass --from-ppp for the "
                 "bootstrap backend (other backends not yet "
                 "implemented in this slice).")

    return survey_from_ppp(
        args.receiver_uid,
        positions_dir=args.positions_dir,
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    sys.exit(main())

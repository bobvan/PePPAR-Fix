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
                         "be written, but don't write the file.")
    ap.add_argument("-v", "--verbose", action="store_true")

    # --- backends --- #
    backends = ap.add_argument_group(
        "backends",
        "Select exactly one backend.  Each one consumes its own "
        "additional flags from the relevant group below.",
    )
    backends.add_argument(
        "--pride", action="store_true",
        help="Use PRIDE-PPP-AR over captured RINEX.  Requires "
             "--rinex-glob and pdp3 installed (PEPPAR_PDP3_BIN env var "
             "or default ~/.PRIDE_PPPAR_BIN/pdp3).",
    )
    pride = ap.add_argument_group("--pride options")
    pride.add_argument(
        "--rinex-glob", default=None,
        help="Glob matching the daily RINEX obs files for this "
             "receiver, e.g. 'data/rinex/MadHat-*.obs'.",
    )
    pride.add_argument(
        "--pride-work-dir", default=None,
        help="Scratch directory for pdp3 (per-day subdirs created "
             "inside).  Default: $TMPDIR/peppar-survey-pride/.",
    )
    pride.add_argument(
        "--history-dir", default=None,
        help="Override state/arp/ directory (history.jsonl lives "
             "under <history-dir>/<uid>/history.jsonl).",
    )
    pride.add_argument(
        "--mount-sn", type=int, default=0,
        help="Antenna mount serial (per-mount history partition).  "
             "Default 0.",
    )
    pride.add_argument(
        "--n-days", type=int, default=7,
        help="Window size for the running ARP mean (default 7).",
    )
    pride.add_argument(
        "--pride-sys", default="GREC,GR",
        help="Comma-separated -sys strings to try in order.  Default "
             "'GREC,GR' (multi-GNSS first, GPS+GLO fallback).",
    )
    pride.add_argument(
        "--max-sig0-m", type=float, default=None,
        help="Quality gate: reject solutions with sig0_m above this "
             "(formal-precision check).  Default 10.0 m (catches "
             "catastrophically bad days while accepting weather-noisy "
             "ones).  Lower for stricter labs; higher to accept short "
             "fragment captures from a non-24h-continuous engine.",
    )
    pride.add_argument(
        "--min-n-obs", type=int, default=None,
        help="Quality gate: reject solutions with fewer than N "
             "observations USED by PRIDE.  Default 10000 (defends "
             "against truncated runs; a healthy 24h F9T capture "
             "clears this 2-10x).  Lower (e.g. 100) when validating "
             "against short engine-fragment captures.",
    )

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

    if args.pride:
        return _run_pride(args)

    log.error(
        "No backend selected.  Pass one of: --pride, --opus, --cors, "
        "--rtklib.  Only --pride is implemented today."
    )
    return 2


def _run_pride(args) -> int:
    """Dispatch to the PRIDE backend with CLI args."""
    import tempfile
    from glob import glob
    from pathlib import Path

    from peppar_fix.arp_history import DEFAULT_MAX_SIG0_M, DEFAULT_MIN_N_OBS
    from peppar_fix.peppar_survey_pride import run_pride_backend

    if not args.rinex_glob:
        log.error("--pride requires --rinex-glob "
                  "(e.g. 'data/rinex/MadHat-*.obs')")
        return 2
    obs_files = [Path(p) for p in sorted(glob(args.rinex_glob))]
    if not obs_files:
        log.error("--rinex-glob %r matched no files", args.rinex_glob)
        return 1
    work_dir = Path(args.pride_work_dir or os.path.join(
        tempfile.gettempdir(), "peppar-survey-pride"))

    sys_attempts = tuple(s.strip() for s in args.pride_sys.split(",") if s.strip())

    return run_pride_backend(
        obs_files=obs_files,
        work_dir=work_dir,
        receiver_uid=args.receiver_uid,
        positions_dir=args.positions_dir,
        history_dir=args.history_dir,
        mount_sn=args.mount_sn,
        sys_attempts=sys_attempts,
        n_days=args.n_days,
        max_sig0_m=(args.max_sig0_m if args.max_sig0_m is not None
                    else DEFAULT_MAX_SIG0_M),
        min_n_obs=(args.min_n_obs if args.min_n_obs is not None
                   else DEFAULT_MIN_N_OBS),
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    sys.exit(main())

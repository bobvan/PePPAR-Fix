#!/usr/bin/env python3
"""CLI for pride_engine_diff — see scripts/peppar_fix/pride_engine_diff.py
docstring for the full design rationale (prideEpochDiff-charlie).

Usage:
  ./scripts/pride_engine_diff.py \
      --kin /tmp/pride-kinematic/PiFace/2026/140/kin_2026140_ufo1 \
      --engine-log /home/bob/gt/peppar-fix-data/engine-logs/day0520nlExp-timehat.log \
      --out /tmp/diff.csv \
      [--tz-offset-hours -5] \
      [--match-tolerance-s 30] \
      [--event-window-s 60] \
      [--divergence-threshold-m 0.10]

Output: CSV at --out with one row per matched (engine sample → nearest
PRIDE epoch) pair, including Δ3D and engine events within the window.
A summary line on stdout reports the first divergence (if any).
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from peppar_fix.pride_engine_diff import (
    first_divergence, match_engine_to_pride, parse_engine_log, parse_kin,
    write_csv,
)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kin", required=True, type=Path,
                    help="Path to a PRIDE kin_<doy>_<station> file.")
    ap.add_argument("--engine-log", required=True, type=Path,
                    help="Engine log containing [AntPosEst N] lines.")
    ap.add_argument("--out", required=True, type=Path,
                    help="Output CSV path.")
    ap.add_argument("--tz-offset-hours", type=int, default=-5,
                    help="Engine local-time TZ offset in hours.  Default "
                         "-5 (CDT, America/Chicago summer).")
    ap.add_argument("--match-tolerance-s", type=float, default=30.0,
                    help="Skip engine samples that have no PRIDE epoch "
                         "within this many seconds.  Default 30 (matches "
                         "PRIDE's 30s sampling).")
    ap.add_argument("--event-window-s", type=float, default=60.0,
                    help="Collect engine events within ±this many seconds "
                         "of each matched sample.  Default 60.")
    ap.add_argument("--divergence-threshold-m", type=float, default=0.10,
                    help="Δ3D threshold for the first-divergence report.  "
                         "Default 0.10 m.")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    log = logging.getLogger("pride_engine_diff.cli")

    log.info("Parsing PRIDE kin: %s", args.kin)
    kin = parse_kin(args.kin)
    log.info("  → %d epochs", len(kin))
    if not kin:
        log.error("kin file has no data rows; can't diff against an empty "
                  "reference.  Check that pdp3 produced solutions; see "
                  "pridemadhatF10tNoSolution-charlie for the failure mode.")
        return 2

    log.info("Parsing engine log: %s", args.engine_log)
    samples, events = parse_engine_log(
        args.engine_log, tz_offset_hours=args.tz_offset_hours)
    log.info("  → %d AntPosEst samples, %d events", len(samples), len(events))
    if not samples:
        log.error("no [AntPosEst N] lines found.  Check the log format "
                  "matches the regex in pride_engine_diff.py.")
        return 2

    records = match_engine_to_pride(
        samples, kin, events,
        window_s=args.event_window_s,
        match_tolerance_s=args.match_tolerance_s,
    )
    log.info("Matched %d engine samples to PRIDE epochs", len(records))
    if not records:
        log.error("no matches.  Engine + PRIDE windows don't overlap.  "
                  "Check timestamps + --tz-offset-hours.")
        return 2

    write_csv(records, args.out)
    log.info("CSV written: %s", args.out)

    div = first_divergence(records, threshold_m=args.divergence_threshold_m)
    if div is None:
        log.info("✓ engine + PRIDE agree to within %.2f m across "
                 "all %d matched epochs (worst Δ3D %.4f m)",
                 args.divergence_threshold_m, len(records),
                 max(r.delta_m for r in records))
    else:
        log.warning("⚠ first divergence (Δ3D > %.2f m) at engine epoch "
                    "%d, %s — Δ3D=%.3f m, σ=%.3f m, NL_fixed=%d, "
                    "events within ±%.0fs: %s",
                    args.divergence_threshold_m,
                    div.epoch_n, div.engine_iso, div.delta_m, div.sigma_3d,
                    div.nl_fixed, args.event_window_s,
                    ", ".join(div.events_in_window) or "(none)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

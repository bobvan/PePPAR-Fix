#!/usr/bin/env python3
"""Replay engine cat-reject cascades through the proposed L3 mode classifier.

Reads a peppar-fix engine log, extracts every CATASTROPHIC_REJECT cascade,
applies the L3 mode classifier from docs/cat-reject-resilience-design.md,
and reports what the new graduated-response ladder WOULD have done.

This is the validation gate for the slipDetectUnified-main design:
before deploying the L0-L5 ladder, replay the design against captured
log data to verify the classifier behaves as predicted.

Usage:
    python3 tools/analysis/replay_cat_reject.py /path/to/engine.log
    python3 tools/analysis/replay_cat_reject.py --threshold-cv 0.05 \\
        /path/to/engine.log /path/to/other.log

The classifier returns one of:
    STATIONARY_CHIPSLIP — CV<threshold AND mean is near integer chip multiple
                          → L3 holds state, escalates to L4 on 120s timeout
    DRIFTING_OUTLIER    — CV>20% → L3 attempts per-SV outlier exclusion
    UNKNOWN             — neither bucket; current code (consecutive=30 → L5)

Output: per-cascade summary plus aggregate rollup suitable for a PR
review or dayplan post.
"""

from __future__ import annotations

import argparse
import re
import statistics
import sys
from dataclasses import dataclass
from typing import List, Optional


# GPS L1 chip = 293.05248062 m exactly (c / 1023000 Hz).
# Speed of light = 299,792,458 m/s exactly.
CHIP_M = 293.05248062
MS_M   = 299_792.458


_CR_RE = re.compile(
    r"^(?P<ts>\S+ \S+) ERROR \[CATASTROPHIC_REJECT\] "
    r"median \|PR\|=(?P<mag>[0-9.]+)m "
    r"> (?P<mult>[0-9.]+)×baseline (?P<base>[0-9.]+)m "
    r"\(consecutive rejects: (?P<n>\d+)\)"
)


@dataclass
class CatReject:
    ts:    str       # ISO-ish timestamp from log
    mag:   float     # median |PR| in meters
    base:  float     # baseline used for the comparison
    mult:  float     # threshold multiplier (typically 20)
    n:     int       # consecutive counter at this epoch

    @property
    def chips(self) -> float:
        return self.mag / CHIP_M

    @property
    def ms(self) -> float:
        return self.mag / MS_M


@dataclass
class Cascade:
    rejects: List[CatReject]

    @property
    def start_ts(self) -> str:
        return self.rejects[0].ts

    @property
    def end_ts(self) -> str:
        return self.rejects[-1].ts

    @property
    def n_rejects(self) -> int:
        return len(self.rejects)

    def magnitudes_m(self) -> List[float]:
        return [r.mag for r in self.rejects]


def parse_log(path: str) -> List[CatReject]:
    """Stream the engine log; return all CATASTROPHIC_REJECT events
    in arrival order.
    """
    out: List[CatReject] = []
    with open(path) as f:
        for line in f:
            m = _CR_RE.match(line)
            if not m:
                continue
            out.append(CatReject(
                ts=m["ts"],
                mag=float(m["mag"]),
                base=float(m["base"]),
                mult=float(m["mult"]),
                n=int(m["n"]),
            ))
    return out


def group_cascades(rejects: List[CatReject]) -> List[Cascade]:
    """Group sequential CatRejects into cascades.

    A cascade starts when `n` resets to 1 (after the previous one ended).
    The engine emits each cat-reject with a monotonically increasing
    counter; reset to 1 marks a new event.
    """
    cascades: List[Cascade] = []
    cur: List[CatReject] = []
    for r in rejects:
        if r.n == 1 and cur:
            cascades.append(Cascade(cur))
            cur = []
        cur.append(r)
    if cur:
        cascades.append(Cascade(cur))
    return cascades


@dataclass
class Classification:
    """Result of L3 mode classification on a cascade."""

    cascade:        Cascade
    mode:           str           # STATIONARY_CHIPSLIP, DRIFTING_OUTLIER, UNKNOWN
    mean_m:         float
    sigma_m:        float
    cv:             float
    chips_mean:     float
    chips_int_dist: float         # |chips_mean - round(chips_mean)|
    ladder_action:  str           # narrative of what L0-L5 would do

    def __str__(self):
        return (f"{self.cascade.start_ts}  n={self.cascade.n_rejects:>2}  "
                f"mean={self.mean_m:>12.1f}m  σ={self.sigma_m:>10.1f}m  "
                f"CV={self.cv*100:>6.2f}%  chips={self.chips_mean:>10.1f}  "
                f"int_dist={self.chips_int_dist:>5.2f}  → {self.mode}")


def classify(cascade: Cascade,
             cv_stationary: float = 0.05,
             cv_drifting: float = 0.20,
             min_mean_m: float = 100.0,
             chip_int_tol: float = 0.5) -> Classification:
    """Apply the L3 mode classifier (per docs design)."""
    mags = cascade.magnitudes_m()
    mean = statistics.mean(mags)
    sd   = statistics.stdev(mags) if len(mags) > 1 else 0.0
    cv   = sd / mean if mean > 0 else float('inf')
    chips = mean / CHIP_M
    int_dist = abs(chips - round(chips))

    if cv < cv_stationary and mean > min_mean_m and int_dist < chip_int_tol:
        mode = "STATIONARY_CHIPSLIP"
        action = ("L3 holds EKF state, no consecutive counter increment, "
                  "re-engage when median drops below threshold for ≥5 clean "
                  "epochs.  L4 (state-flush) on 120s timeout.")
    elif cv > cv_drifting:
        mode = "DRIFTING_OUTLIER"
        action = ("L0+L2 (NAV-SIG-driven per-SV exclusion) should catch "
                  "this BEFORE it reaches cat-reject.  If we still reach "
                  "here, L3 attempts to mute the worst-PR-contributor SV; "
                  "L4 state-flush on persistence.")
    else:
        mode = "UNKNOWN"
        action = ("Behave like current code: at consecutive=30, L5 "
                  "(exit-5 + wrapper relaunch).")

    return Classification(
        cascade=cascade,
        mode=mode,
        mean_m=mean,
        sigma_m=sd,
        cv=cv,
        chips_mean=chips,
        chips_int_dist=int_dist,
        ladder_action=action,
    )


def replay_file(path: str, **classifier_kwargs) -> List[Classification]:
    """Parse → group → classify a single log."""
    rejects = parse_log(path)
    cascades = group_cascades(rejects)
    return [classify(c, **classifier_kwargs) for c in cascades]


def render_rollup(classifications: List[Classification], path: str) -> None:
    """Print per-cascade detail + aggregate rollup."""
    print(f"\n{'='*100}")
    print(f"File: {path}")
    print(f"Cascades: {len(classifications)}")
    print(f"{'='*100}")
    print(f"  {'time':>20}  {'n':>3}  {'mean (m)':>14}  {'σ (m)':>12}  "
          f"{'CV%':>7}  {'chips':>12}  {'int_dist':>9}  {'mode':<22}")
    print("  " + "-" * 110)

    counts = {"STATIONARY_CHIPSLIP": 0, "DRIFTING_OUTLIER": 0, "UNKNOWN": 0}
    for cls in classifications:
        print(f"  {cls.cascade.start_ts:>20}  {cls.cascade.n_rejects:>3}  "
              f"{cls.mean_m:>14.1f}  {cls.sigma_m:>12.1f}  "
              f"{cls.cv*100:>6.2f}%  {cls.chips_mean:>12.1f}  "
              f"{cls.chips_int_dist:>9.2f}  {cls.mode:<22}")
        counts[cls.mode] += 1

    print()
    print(f"  Rollup:")
    n = len(classifications)
    for mode, k in counts.items():
        pct = 100 * k / n if n > 0 else 0
        print(f"    {mode:<22}  {k:>3} / {n}  ({pct:>5.1f}%)")

    sta = counts["STATIONARY_CHIPSLIP"]
    print(f"\n  Predicted L0-L2-handled (before reaching L3): "
          f"{counts['DRIFTING_OUTLIER']}/{n} cascades (NAV-SIG should "
          f"have flagged offending SVs).")
    print(f"  Predicted L3-handled (held state, no exit-5): "
          f"{sta}/{n} cascades.")
    print(f"  Predicted L5 (exit-5) needed: {counts['UNKNOWN']}/{n} cascades.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="engine log file(s)")
    ap.add_argument("--cv-stationary", type=float, default=0.05,
                    help="CV threshold for STATIONARY_CHIPSLIP (default 0.05)")
    ap.add_argument("--cv-drifting", type=float, default=0.20,
                    help="CV threshold for DRIFTING_OUTLIER (default 0.20)")
    ap.add_argument("--chip-int-tol", type=float, default=0.5,
                    help="Distance from nearest integer chip multiple "
                         "to count as STATIONARY_CHIPSLIP (default 0.5)")
    args = ap.parse_args()

    aggregate_counts = {"STATIONARY_CHIPSLIP": 0, "DRIFTING_OUTLIER": 0,
                        "UNKNOWN": 0}
    total = 0
    for path in args.logs:
        try:
            cls = replay_file(
                path,
                cv_stationary=args.cv_stationary,
                cv_drifting=args.cv_drifting,
                chip_int_tol=args.chip_int_tol,
            )
        except FileNotFoundError:
            print(f"WARNING: not found: {path}", file=sys.stderr)
            continue
        render_rollup(cls, path)
        for c in cls:
            aggregate_counts[c.mode] += 1
            total += 1

    if len(args.logs) > 1 and total > 0:
        print(f"\n{'='*100}")
        print(f"Aggregate across {len(args.logs)} log files: {total} cascades")
        print(f"{'='*100}")
        for mode, k in aggregate_counts.items():
            pct = 100 * k / total
            print(f"    {mode:<22}  {k:>3} / {total}  ({pct:>5.1f}%)")


if __name__ == "__main__":
    main()

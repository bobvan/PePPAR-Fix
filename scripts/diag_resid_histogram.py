#!/usr/bin/env python3
"""Post-hoc: aggregate [RESID_PR] / [RESID_PHI] lines across an engine
log to build per-SV residual histograms.

Enabled by the 60-epoch per-SV residual emission in the engine
(commit 21e2b77).  Purpose: identify which SVs (or which signal types)
carry systematic residual offsets that bias the float-PPP solution —
candidate sources of the observation-model bias producing the F9T-20B
altitude basin.

Usage:

    ./diag_resid_histogram.py --log data/day0424e-madhat.log \\
        [--from-epoch N] [--to-epoch M] [--top N]

Reports per SV, per kind (PR / PHI):
  count, mean, std, min, max, |mean|/std ratio.

SVs with |mean| > 3·std and sample count ≥ 20 are the suspects —
systematic offset, not noise.

Safe to run on live logs — accumulates all [RESID_...] lines seen
up to the current file position.
"""

from __future__ import annotations

import argparse
import re
import sys
from collections import defaultdict

import numpy as np


# Matches either [RESID_PR NNNN] or [RESID_PHI NNNN].  Body is
# "SV:+0.02  SV:+0.03 ..." — each token is "<SV>:<signed_float>".
LINE_RE = re.compile(
    r"\[RESID_(?P<kind>PR|PHI)\s+(?P<epoch>\d+)\]\s+\d+:\s+(?P<body>.*)$")
TOKEN_RE = re.compile(r"(?P<sv>[A-Z]\d\d):(?P<val>[+-]?\d+\.\d+)")


def parse_log(path, from_epoch=None, to_epoch=None):
    """Yield (epoch, kind, sv, value) tuples from the log."""
    with open(path) as f:
        for line in f:
            m = LINE_RE.search(line)
            if not m:
                continue
            epoch = int(m['epoch'])
            if from_epoch is not None and epoch < from_epoch:
                continue
            if to_epoch is not None and epoch > to_epoch:
                continue
            kind = m['kind'].lower()
            for tm in TOKEN_RE.finditer(m['body']):
                yield epoch, kind, tm['sv'], float(tm['val'])


def summarise(records):
    """records = [(epoch, kind, sv, val), ...] → nested summary dicts.

    Returns {kind: {sv: {count, mean, std, min, max, abs_mean_over_std}}}.
    """
    by_kind_sv = defaultdict(lambda: defaultdict(list))
    for _ep, k, sv, v in records:
        by_kind_sv[k][sv].append(v)
    out = {}
    for k, sv_map in by_kind_sv.items():
        out[k] = {}
        for sv, vals in sv_map.items():
            arr = np.asarray(vals, dtype=float)
            n = int(arr.size)
            mu = float(arr.mean())
            sd = float(arr.std()) if n > 1 else 0.0
            mn, mx = float(arr.min()), float(arr.max())
            ratio = abs(mu) / sd if sd > 0 else float('inf') if mu != 0 else 0.0
            out[k][sv] = dict(count=n, mean=mu, std=sd,
                               min=mn, max=mx, abs_mean_over_std=ratio)
    return out


def print_table(summary, kind, top=None, sort_by='abs_mean'):
    rows = summary.get(kind, {})
    if not rows:
        print(f"No {kind.upper()} samples.")
        return
    items = list(rows.items())
    if sort_by == 'abs_mean':
        items.sort(key=lambda kv: -abs(kv[1]['mean']))
    elif sort_by == 'ratio':
        items.sort(key=lambda kv: -kv[1]['abs_mean_over_std'])
    elif sort_by == 'count':
        items.sort(key=lambda kv: -kv[1]['count'])
    if top:
        items = items[:top]
    units = 'm' if kind == 'pr' else 'm'
    print(f"{'sv':>4} {'n':>5} {'mean':>8} {'std':>7} {'min':>8} "
          f"{'max':>8} {'|μ|/σ':>6}  (units={units})")
    print("-" * 58)
    for sv, s in items:
        suspect = " *" if s['abs_mean_over_std'] > 3.0 and s['count'] >= 20 else "  "
        print(f"{sv:>4} {s['count']:>5} {s['mean']:>+8.3f} {s['std']:>7.3f} "
              f"{s['min']:>+8.3f} {s['max']:>+8.3f} {s['abs_mean_over_std']:>6.1f}"
              f"{suspect}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True, help="engine log file")
    ap.add_argument("--from-epoch", type=int, default=None)
    ap.add_argument("--to-epoch", type=int, default=None)
    ap.add_argument("--top", type=int, default=None,
                    help="report only top-N SVs per kind")
    ap.add_argument("--sort", choices=("abs_mean", "ratio", "count"),
                    default="abs_mean",
                    help="sort SVs by this metric (default: |mean|)")
    args = ap.parse_args()

    records = list(parse_log(args.log, args.from_epoch, args.to_epoch))
    if not records:
        print("No [RESID_PR] / [RESID_PHI] lines found in log.",
              file=sys.stderr)
        print("Confirm the engine was built with commit 21e2b77 or "
              "later and has run for at least 60 epochs.",
              file=sys.stderr)
        sys.exit(2)

    epochs = sorted({r[0] for r in records})
    print(f"Samples: {len(records)} across {len(epochs)} snapshots")
    print(f"Epoch range: {epochs[0]} → {epochs[-1]}")
    print()

    summary = summarise(records)

    print("=== PR residuals (post-fit, meters) ===")
    print_table(summary, 'pr', top=args.top, sort_by=args.sort)
    print()
    print("=== Phase residuals (post-fit, meters) ===")
    print_table(summary, 'phi', top=args.top, sort_by=args.sort)

    # Suspects
    suspects = []
    for k in ('pr', 'phi'):
        for sv, s in summary.get(k, {}).items():
            if s['abs_mean_over_std'] > 3.0 and s['count'] >= 20:
                suspects.append((k, sv, s))
    if suspects:
        print()
        print(f"=== Systematic-bias suspects ({len(suspects)}) ===")
        print("(|mean| > 3·std and n ≥ 20 — consistent offset, not noise)")
        for k, sv, s in sorted(suspects,
                                 key=lambda t: -abs(t[2]['mean'])):
            print(f"  {k.upper():3s} {sv} n={s['count']:3d} "
                  f"mean={s['mean']:+.3f} std={s['std']:.3f} "
                  f"|μ|/σ={s['abs_mean_over_std']:.1f}")


if __name__ == '__main__':
    main()

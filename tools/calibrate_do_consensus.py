#!/usr/bin/env python3
"""Combine multiple ``calibrate_do.py`` runs into a consensus.

Compares N independent characterization JSONs of the same DO and
reports reproducibility metrics:

  - Median + spread of measured gain (measured/commanded slope)
  - Median + spread of offset (measured freq at commanded=0)
  - Median + spread of tuning range
  - Pairwise relative variation (target: ≤ 1%)
  - Suggested consensus values for the host TOML

Usage:
    python3 tools/calibrate_do_consensus.py \\
        path/to/run1.json path/to/run2.json path/to/run3.json

Or with a glob:
    python3 tools/calibrate_do_consensus.py 'data/clkpoc3-cal-*.json'

Emits a small report to stdout and, with --emit-toml, prints a TOML
fragment ready to paste into ``config/<host>.toml``.
"""

from __future__ import annotations

import argparse
import glob
import json
import statistics
import sys
from pathlib import Path


def _stat_or_none(values, op):
    return op(values) if len(values) >= 2 else None


def load_run(path):
    with open(path) as f:
        d = json.load(f)
    # Required fields (calibrate_do.py output schema)
    return {
        'path':                path,
        'actuator':            d.get('actuator'),
        'gain':                float(d['gain']),
        'offset_ppb':          float(d['offset_ppb']),
        'range_ppb':           float(d['range_ppb']),
        'max_residual_ppb':    float(d['max_residual_ppb']),
        'resolution_ppb':      float(d['resolution_ppb']),
        'ppb_per_code_initial':   d.get('ppb_per_code_initial'),
        'ppb_per_code_corrected': d.get('ppb_per_code_corrected'),
        'timestamp':           d.get('timestamp'),
        'n_points':            len(d.get('points', [])),
    }


def report(runs, emit_toml=False):
    print(f"\n=== Multi-run consensus across {len(runs)} runs ===")
    print(f"{'run':>4}  {'gain':>10}  {'offset (ppb)':>14}  "
          f"{'range (ppb)':>13}  {'max_resid (ppb)':>16}  "
          f"{'ppb/code emp':>15}  {'timestamp'}")
    for i, r in enumerate(runs):
        emp = r.get('ppb_per_code_corrected')
        print(f"  {i+1:>3}  {r['gain']:>10.5f}  {r['offset_ppb']:>+14.3f}  "
              f"{r['range_ppb']:>13.2f}  {r['max_residual_ppb']:>16.3f}  "
              f"{emp if emp is None else f'{emp:>15.6f}'}  "
              f"{r['timestamp'] or '-'}")

    if len(runs) < 2:
        print("\nNeed ≥2 runs for reproducibility analysis.")
        return runs[0] if runs else None

    # Compute per-metric stats
    def col(key):
        return [r[key] for r in runs if r.get(key) is not None]

    gains   = col('gain')
    offs    = col('offset_ppb')
    ranges  = col('range_ppb')
    emps    = col('ppb_per_code_corrected')

    def med_spread(values, name, unit):
        if not values:
            return
        m = statistics.median(values)
        if len(values) >= 2:
            mn, mx = min(values), max(values)
            spread = mx - mn
            rel = (spread / abs(m) * 100) if m != 0 else float('nan')
            stdev = statistics.stdev(values) if len(values) > 1 else 0.0
            print(f"  {name:>20s}  median={m:>10.5f}  "
                  f"range=[{mn:.5f}, {mx:.5f}]  "
                  f"spread={spread:.5g} {unit}  "
                  f"rel={rel:.3f}%   stdev={stdev:.5g}")
        else:
            print(f"  {name:>20s}  single value={m:.5f} {unit}")

    print("\nReproducibility:")
    med_spread(gains,  'gain',                  '')
    med_spread(offs,   'offset (ppb)',          'ppb')
    med_spread(ranges, 'range (ppb)',           'ppb')
    med_spread(emps,   'ppb_per_code emp',      'ppb/code')

    # Decision: does this meet 1% spread?
    if emps and len(emps) >= 2:
        m = statistics.median(emps)
        spread = max(emps) - min(emps)
        rel = (spread / abs(m) * 100) if m != 0 else float('inf')
        print(f"\nReproducibility verdict: "
              f"ppb/code spread {rel:.3f}% "
              f"({'PASS — ≤1%' if rel <= 1.0 else 'FAIL — >1%'})")

    # Consensus values (use median, more robust to outliers than mean)
    consensus = {
        'gain':                  statistics.median(gains) if gains else None,
        'offset_ppb':            statistics.median(offs) if offs else None,
        'range_ppb':             statistics.median(ranges) if ranges else None,
        'ppb_per_code':          statistics.median(emps) if emps else None,
    }

    print("\nConsensus (median across runs):")
    for k, v in consensus.items():
        if v is not None:
            print(f"  {k:>20s} = {v:+.6f}")

    if emit_toml and consensus['ppb_per_code'] is not None:
        # Pull dac_max_ppb from the per-run "range_ppb" — should be the
        # tunable range from center.  Half-range is the symmetric ±
        # value.  Round down slightly for safety margin from rails.
        half_range = consensus['range_ppb'] / 2.0
        max_ppb_safe = int(half_range * 0.99 + 0.5)  # 1% margin

        print(f"\n--- TOML fragment ---")
        print(f'dac_ppb_per_code = "{consensus["ppb_per_code"]:.5f}"')
        print(f'dac_max_ppb = {max_ppb_safe:.1f}')
        print(f'# Empirical from {len(runs)} runs; '
              f'consensus is median across runs.')
        print(f'# Free-running OCXO offset at center: '
              f'{consensus["offset_ppb"]:+.1f} ppb')
        print(f'# Full tuning range: {consensus["range_ppb"]:.1f} ppb '
              f'(half-range {half_range:.1f} ppb)')

    return consensus


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('paths', nargs='+',
                    help='Paths or globs to calibrate_do JSON outputs')
    ap.add_argument('--emit-toml', action='store_true',
                    help='Print a TOML fragment with consensus values')
    args = ap.parse_args()

    paths = []
    for p in args.paths:
        matches = glob.glob(p) if '*' in p else [p]
        paths.extend(sorted(matches))
    if not paths:
        print("No JSON files found.", file=sys.stderr)
        return 1

    runs = []
    for p in paths:
        try:
            runs.append(load_run(p))
        except (KeyError, ValueError, FileNotFoundError) as e:
            print(f"  Skipping {p}: {type(e).__name__}: {e}", file=sys.stderr)
            continue

    if not runs:
        print("No valid runs.", file=sys.stderr)
        return 1

    report(runs, emit_toml=args.emit_toml)
    return 0


if __name__ == '__main__':
    sys.exit(main())

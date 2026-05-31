#!/usr/bin/env python3
"""Cross-host phase-agreement CDF: empirical CDF of |Δ| between two clocks.

Reads ticc_capture-format CSV(s) with chA + chB samples (chA = host A
PPS OUT, chB = host B PPS OUT, both into a shared-reference TICC),
pairs by ref_sec, removes constant offset + linear drift (wiring/
cable-length artifacts, not clock-agreement noise), and plots
P(|Δ| ≤ T) vs T — the empirical CDF of |chA − chB|.

The 95 % horizontal line's crossing with the curve IS the moonshot's
95 % excursion bound (per docs/two-site-sync-budget.md): shared-
antenna target |Δ| ≤ 1 ns @ 95 %, separate-antenna target |Δ| ≤ 2 ns
@ 95 %.  Read directly off the plot.

Usage:
    python3 tools/plot_xhost_agreement_cdf.py \\
        --input data/day0530-xhost-run2-2026-05-30.csv \\
        --output data/day0530-xhost-run2-agreement.png

    # Multi-file overlay (each input rendered as a separate curve):
    python3 tools/plot_xhost_agreement_cdf.py \\
        --input data/ticc4-pifaceA-clkpoc3B-2026-05-27.csv \\
        --input data/ticc4-pifaceA-clkpoc3B-2026-05-28.csv \\
        --label '2026-05-27' --label '2026-05-28' \\
        --output data/xhost-agreement-may27-28.png

Input CSV columns expected (ticc_capture.py output):
    ts_iso, channel, ref_sec, ref_ps, recv_mono
Engine TICC logs (peppar-fix --ticc-log) use the same shape; either
works.
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')   # headless: lab + CI safe
import matplotlib.pyplot as plt


_PS_PER_S = 10 ** 12
_PAIR_TOLERANCE_PS = 5 * 10 ** 11   # 500 ms — anything beyond is not a pair


def load_pairs(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load chA/chB, pair each chA with closest-in-time chB, return
    (sample_index, Δ_in_ps).

    PRECISION INVARIANT (Bob, 2026-05-30): TICC timestamps MUST be
    handled as integer (sec, ps) throughout.  Building a combined
    "total_ps = sec * 1e12 + ps" in float64 quantizes at a level that
    SCALES WITH RUNTIME — every order of magnitude in ref_sec robs
    an order of magnitude from the picosecond count.  Float64 has a
    53-bit mantissa ≈ 16 decimal digits of relative precision; the
    integer-second count consumes log10(sec) of them, the ps count
    gets what's left:

         ref_sec      total_ps    float64 ps quantization
         10^0  (1 s)  10^12       ~10^-4 ps    (fine)
         10^3  (17m)  10^15       ~1 ps        (borderline)
         10^6  (12d)  10^18       ~100 ps      (kills sub-ns work)
         10^9  (32y)  10^21       ~100 µs      (useless)

    Our TICC ref_sec values run in the 10^5 – 10^6 range today
    (host uptime), so float64 totals would land at the ~10-100 ps
    quantization floor.

    To preserve precision while still pairing across the ref_sec
    boundary (chA at (N, ~999e9 ps) and chB at (N+1, ~0 ps) are
    really the same PPS edge ≤ 1 ms apart), we:

      1. Build per-event Python ``int total_ps = sec * _PS_PER_S + ps``.
         Python int is arbitrary-precision — no rounding.
      2. Sort chA + chB independently; bisect to find chA's
         nearest-in-time chB neighbour (left + right candidates).
      3. Pick the closer of the two candidates; reject if > 500 ms
         (clearly not the same PPS edge).
      4. Δ_ps = chA_total - chB_total in Python int.  Exact.  Cast
         to int64 only AFTER the subtraction; |Δ| ≤ 5e11 fits with
         room to spare.

    Numpy float64 is fine for the residual Δ values (≤ 5e11) since
    float64 has ~10^-7 relative precision and we're typically
    looking at ns-scale agreement.
    """
    import bisect

    chA_list: list[int] = []
    chB_list: list[int] = []
    with open(path) as f:
        for row in csv.DictReader(f):
            s = int(row['ref_sec'])
            p = int(row['ref_ps'])
            total = s * _PS_PER_S + p   # Python int — exact
            if row['channel'] == 'chA':
                chA_list.append(total)
            elif row['channel'] == 'chB':
                chB_list.append(total)

    if not chA_list or not chB_list:
        return np.array([], dtype=np.int64), np.array([], dtype=np.int64)

    chA_list.sort()
    chB_list.sort()
    n_b = len(chB_list)

    deltas: list[int] = []
    for a in chA_list:
        idx = bisect.bisect_left(chB_list, a)
        # Compare with both neighbours (left + right of insertion point)
        cands: list[int] = []
        if idx > 0:
            cands.append(chB_list[idx - 1])
        if idx < n_b:
            cands.append(chB_list[idx])
        # subtract-separately pattern (preserved by working in Python int)
        best = min(cands, key=lambda b: abs(a - b))
        diff = a - best   # Python int subtraction, exact
        if abs(diff) > _PAIR_TOLERANCE_PS:
            continue
        deltas.append(diff)

    if not deltas:
        return np.array([], dtype=np.int64), np.array([], dtype=np.int64)
    # |Δ| ≤ 5e11 fits in int64 trivially; safe to convert here.
    return (np.arange(len(deltas), dtype=np.int64),
            np.array(deltas, dtype=np.int64))


def detrend_and_abs(t: np.ndarray, delta: np.ndarray) -> np.ndarray:
    """Linear-detrend (offset + drift) then |.|

    Constant offset = cable-length / RF-cable propagation delay
    difference between the two PPS sources at the TICC.
    Linear drift = TICC-reference-Rb vs the two-clock pair common-
    mode rate (already canceled by the differential at the TICC).
    Both are wiring artifacts, not clock-agreement noise.
    """
    if len(delta) == 0:
        return np.array([])
    d_f = delta.astype(np.float64)
    tf = t.astype(np.float64)
    poly = np.polyfit(tf, d_f, 1)
    residual = d_f - np.polyval(poly, tf)
    return np.abs(residual)


def cdf_xy(abs_residual: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if len(abs_residual) == 0:
        return np.array([]), np.array([])
    sorted_x = np.sort(abs_residual)
    y_pct = np.arange(1, len(sorted_x) + 1) / len(sorted_x) * 100.0
    return sorted_x, y_pct


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--input', action='append', required=True, type=Path,
                    help='ticc_capture-format CSV.  Repeat for overlay.')
    ap.add_argument('--label', action='append', default=None,
                    help='Label for each --input (positional pairing). '
                         'Defaults to file stem.')
    ap.add_argument('--output', required=True, type=Path,
                    help='Output PNG path.')
    ap.add_argument('--title', default='Two-clock phase-agreement CDF')
    ap.add_argument('--xmin-ps', type=float, default=60.0,
                    help='Lower x-axis bound in ps (default 60 = TICC res).')
    ap.add_argument('--xmax-ps', type=float, default=None,
                    help='Upper x-axis bound (default: 1.2× worst data).')
    args = ap.parse_args()

    labels = args.label if args.label else [p.stem for p in args.input]
    if len(labels) != len(args.input):
        ap.error(f'--label count ({len(labels)}) != --input count '
                 f'({len(args.input)})')

    fig, ax = plt.subplots(figsize=(9, 6))
    max_x_seen = args.xmin_ps
    rows_printed = False
    for path, label in zip(args.input, labels):
        t, delta = load_pairs(path)
        n = len(delta)
        if n < 60:
            print(f'WARN: {path} has only {n} paired samples — skipping',
                  file=sys.stderr)
            continue
        abs_r = detrend_and_abs(t, delta)
        sorted_x, y_pct = cdf_xy(abs_r)
        max_x_seen = max(max_x_seen, float(sorted_x[-1]))
        p50 = float(np.percentile(abs_r, 50))
        p95 = float(np.percentile(abs_r, 95))
        p99 = float(np.percentile(abs_r, 99))
        if not rows_printed:
            print(f'{"label":<40s} {"n":>7s} {"p50":>10s} {"p95":>10s} '
                  f'{"p99":>10s} {"max":>10s}', file=sys.stderr)
            print('-' * 92, file=sys.stderr)
            rows_printed = True
        print(f'{label:<40s} {n:>7d} {p50:>10.1f} {p95:>10.1f} '
              f'{p99:>10.1f} {sorted_x[-1]:>10.1f}', file=sys.stderr)
        ax.plot(sorted_x, y_pct, label=f'{label} (n={n})', lw=1.8)

    if not rows_printed:
        print('No usable inputs.', file=sys.stderr)
        return 1

    xmax = args.xmax_ps if args.xmax_ps is not None else max_x_seen * 1.2
    ax.set_xscale('log')
    ax.set_xlim(args.xmin_ps, xmax)
    ax.set_ylim(0, 100)
    ax.set_xlabel('|Δ| (ps) — log scale')
    ax.set_ylabel('Cumulative % of paired epochs (P(|Δ| ≤ T))')
    ax.set_title(args.title)
    ax.grid(True, which='both', ls=':', alpha=0.4)

    # Reference verticals
    if args.xmin_ps <= 60 <= xmax:
        ax.axvline(60, color='red', ls=':', alpha=0.65, lw=1.0)
        ax.text(60, 1.5, ' TICC res\n (60 ps)',
                fontsize=8, color='red', va='bottom')
    if args.xmin_ps <= 1000 <= xmax:
        ax.axvline(1000, color='darkgreen', ls=':', alpha=0.65, lw=1.0)
        ax.text(1000, 1.5, ' shared-ant\n (1 ns)',
                fontsize=8, color='darkgreen', va='bottom')
    if args.xmin_ps <= 2000 <= xmax:
        ax.axvline(2000, color='blue', ls=':', alpha=0.65, lw=1.0)
        ax.text(2000, 1.5, ' separate-ant\n (2 ns)',
                fontsize=8, color='blue', va='bottom')

    # Reference horizontals at common percentile bands
    for q in (50, 95, 99):
        ax.axhline(q, color='gray', ls=':', alpha=0.55, lw=0.8)
        ax.text(args.xmin_ps * 1.04, q + 0.7, f'{q}%',
                fontsize=8, color='gray')

    ax.legend(loc='lower right', fontsize=9)
    fig.tight_layout()
    fig.savefig(args.output, dpi=130)
    print(f'\nWrote {args.output}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())

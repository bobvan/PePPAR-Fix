#!/usr/bin/env python3
"""Back-yard vs lab chA TDEV(τ), segment-aware with re-bootstrap annotation.

Motivation (day0606): PiFace ran its OCXO DO outdoors (back yard) while
clkPoC3 ran the same DO class indoors (lab control).  The outdoor thermal
gradients tripped the EKF state-sanity guard twice, forcing exit-5
re-bootstraps.  A single global linear detrend would smear those phase
steps into TDEV at every τ, so we:

  * split each chA series at TICC ref_sec gaps (> --gap-split-s), which
    fall exactly on the re-bootstrap relaunch windows,
  * linearly detrend WITHIN each clean segment,
  * compute TDEV on the longest clean segment per host, and
  * draw a companion phase-vs-time panel with vertical markers at each
    re-bootstrap so the thermal-stress story is visible alongside the
    clean TDEV curves.

Usage:
    python3 tools/plot_backyard_vs_lab_tdev.py \\
        --host 'PiFace (back yard, OCXO)' data/day0606-backyard-ticc.csv \\
        --host 'clkPoC3 (lab, OCXO)'      data/day0606-labcmp-clkpoc3-ticc.csv \\
        --out data/day0606_backyard_vs_lab_tdev.png
"""
from __future__ import annotations

import argparse
import csv
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import allantools
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

_PS_PER_S = 10 ** 12


def load_chA(path: Path):
    """Return (ts[list[datetime]], ref_sec[int64], total_ps[list[int]]) for chA."""
    ts, secs, totals = [], [], []
    with open(path) as f:
        r = csv.DictReader(f)
        ts_col = 'host_timestamp' if 'host_timestamp' in r.fieldnames else 'ts_iso'
        for row in r:
            if row['channel'] != 'chA':
                continue
            ts.append(datetime.fromisoformat(row[ts_col].replace('Z', '+00:00')))
            secs.append(int(row['ref_sec']))
            totals.append(int(row['ref_sec']) * _PS_PER_S + int(row['ref_ps']))
    if not totals:
        raise ValueError(f"{path}: no chA samples")
    return ts, np.array(secs, dtype=np.int64), totals


def segments(secs: np.ndarray, gap_split_s: int):
    """Indices [(start,end)] of contiguous runs where ref_sec steps <= gap."""
    breaks = np.where(np.diff(secs) > gap_split_s)[0]
    bounds = [0, *(int(b) + 1 for b in breaks), len(secs)]
    return [(bounds[i], bounds[i + 1]) for i in range(len(bounds) - 1)]


def detrend_seg(secs: np.ndarray, totals, lo: int, hi: int) -> np.ndarray:
    """Linearly detrend one segment; return phase residual in seconds."""
    x = (secs[lo:hi] - secs[lo]).astype(np.float64)
    y = np.array([t - totals[lo] for t in totals[lo:hi]], dtype=np.float64)
    slope, intercept = np.polyfit(x, y, 1)
    return (y - (slope * x + intercept)) * 1e-12


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--host', action='append', nargs=2, metavar=('LABEL', 'CSV'),
                   required=True, help='Host label + TICC CSV. Repeat per host.')
    p.add_argument('--out', required=True, type=Path)
    p.add_argument('--gap-split-s', type=int, default=5,
                   help='ref_sec gap (s) that marks a re-bootstrap (default 5)')
    p.add_argument('--rate-hz', type=float, default=1.0)
    args = p.parse_args()

    colors = ['C1', 'C0', 'C2', 'C3']
    fig, (axp, axt) = plt.subplots(1, 2, figsize=(15, 6))

    print(f'{"host":32s} {"segs":>5s} {"longest_h":>9s} {"n_long":>7s} '
          f'{"TDEV(1s)":>9s} {"min TDEV":>9s} @ {"τ":>6s}', file=sys.stderr)

    for i, (label, csv_path) in enumerate(args.host):
        color = colors[i % len(colors)]
        ts, secs, totals = load_chA(Path(csv_path))
        t0 = ts[0]
        segs = segments(secs, args.gap_split_s)

        # ---- Phase-vs-time panel: per-segment detrend, drawn at true time ----
        for (lo, hi) in segs:
            resid = detrend_seg(secs, totals, lo, hi)
            elapsed_h = np.array([(ts[j] - t0).total_seconds() / 3600
                                  for j in range(lo, hi)])
            axp.plot(elapsed_h, resid * 1e9, '-', color=color, lw=0.6, alpha=0.8)
        # Re-bootstrap markers = gap boundaries (segment starts after seg 0)
        reboot_labels = []
        for (lo, hi) in segs[1:]:
            t_reb = ts[lo]
            eh = (t_reb - t0).total_seconds() / 3600
            axp.axvline(eh, color='red', ls='--', lw=1.1, alpha=0.7)
            reboot_labels.append(t_reb.strftime('%H:%MZ'))
            axp.text(eh, axp.get_ylim()[1], ' re-boot\n ' + t_reb.strftime('%H:%MZ'),
                     color='red', fontsize=7, va='top', ha='left')

        # ---- TDEV on the longest clean segment ----
        lo, hi = max(segs, key=lambda s: s[1] - s[0])
        resid = detrend_seg(secs, totals, lo, hi)
        n_long = hi - lo
        span_h = n_long / args.rate_hz / 3600
        taus_req = np.logspace(0, np.log10(span_h * 3600 / 4), 40)
        taus, tdev, _, _ = allantools.tdev(
            resid, rate=args.rate_hz, data_type='phase', taus=taus_req)
        i_min = int(np.argmin(tdev))
        seg_note = f' (longest of {len(segs)} segs, {span_h*60:.0f} min)' \
            if len(segs) > 1 else f' ({span_h:.2f} h, no re-boots)'
        axt.loglog(taus, tdev * 1e9, '-', color=color, lw=1.8,
                   label=label + seg_note)
        axt.loglog([taus[i_min]], [tdev[i_min] * 1e9], 'o', color=color,
                   markerfacecolor='white', markeredgecolor=color, markersize=8,
                   markeredgewidth=1.5)
        tdev_1s = float(np.interp(1.0, taus, tdev)) * 1e9
        print(f'{label:32s} {len(segs):>5d} {span_h:>9.2f} {n_long:>7d} '
              f'{tdev_1s:>7.3f}ns {tdev[i_min]*1e9:>7.3f}ns @ {taus[i_min]:>5.0f}s'
              + (f'  re-boots: {", ".join(reboot_labels)}' if reboot_labels else ''),
              file=sys.stderr)

    axp.set_xlabel('elapsed (hours)')
    axp.set_ylabel('chA phase residual (ns, per-segment detrended)')
    axp.set_title('chA phase vs time\n(red dashed = exit-5 re-bootstrap)')
    axp.grid(True, alpha=0.3)

    # WFM/RWFM reference slopes
    tau_ref = np.array([1.0, 10000.0])
    axt.loglog(tau_ref, 0.3 * tau_ref ** -0.5, ':', color='gray', alpha=0.5, lw=0.8)
    axt.text(8, 0.12, 'WFM (−½)', fontsize=8, color='gray', alpha=0.7)
    axt.loglog(tau_ref, 0.04 * tau_ref ** 0.5, ':', color='gray', alpha=0.5, lw=0.8)
    axt.text(120, 0.55, 'RWFM (+½)', fontsize=8, color='gray', alpha=0.7)
    axt.set_xlabel('τ (s)')
    axt.set_ylabel('TDEV (ns)')
    axt.set_title('chA TDEV(τ) — longest clean segment per host\n'
                  '(circles = empirical minimum)')
    axt.grid(True, which='both', alpha=0.3)
    axt.legend(fontsize=8, loc='best')

    plt.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(args.out, dpi=120)
    print(f'\nWrote {args.out}', file=sys.stderr)


if __name__ == '__main__':
    sys.exit(main())

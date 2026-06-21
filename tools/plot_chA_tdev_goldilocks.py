#!/usr/bin/env python3
"""TDEV(τ) of TICC chA per host, with PR #124's predicted Goldilocks τ*
overlaid as a reference line.

Each input is a TICC CSV (chA + chB rows).  We extract chA — the
disciplined-output PPS edge measured in the TICC's local timescale —
detrend linearly (cancels TICC ref boot offset + the per-host
small constant rate difference between TICC ref and 1 Hz), and compute
TDEV with allantools at log-spaced τ.

Caveat for PiFace 2026-06-02 overnight: PiFace's TICC #3 was on a
private OCXO reference (portable-clock prep), not the lab Rb standard.
Its TDEV floor is therefore set by max(private OCXO noise, DO noise),
not DO noise alone — expected to read higher than clkPoC3 even if the
DOs are equally clean.

Reads CSV format: ts_iso/host_timestamp, host_monotonic, ref_sec,
ref_ps, channel.

Usage:
    python3 tools/plot_chA_tdev_goldilocks.py \\
        --host 'PiFace (priv OCXO ref)' data/foo-piface-ticc.csv 1.97 \\
        --host 'clkPoC3 (lab Rb ref)'   data/bar-clkpoc3-ticc.csv 1.45 \\
        --out data/chA_tdev_goldilocks.png \\
        --skip-before 2026-06-03T03:00:00Z
"""
from __future__ import annotations

import argparse
import csv
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import allantools
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Provenance stamp (analysis versioning + git-hash footer).
sys.path.insert(0, str(Path(__file__).resolve().parent))
from analysis_provenance import (stamp, provenance_line,  # noqa: E402
                                 skip_comment_lines)

_TOOLNAME = 'plot_chA_tdev_goldilocks.py'

_PS_PER_S = 10 ** 12


def load_chA_phase_s(path: Path, skip_before: datetime | None) -> np.ndarray:
    """Return chA phase in seconds, linearly detrended to remove TICC
    boot offset + the small per-host constant rate vs nominal 1 Hz.

    Returns a 1-D ndarray of phase samples at the natural 1-s spacing."""
    secs: list[int] = []
    totals: list[int] = []
    with open(path) as f:
        r = csv.DictReader(skip_comment_lines(f))
        ts_col = 'host_timestamp' if 'host_timestamp' in r.fieldnames else 'ts_iso'
        for row in r:
            if row['channel'] != 'chA':
                continue
            ts = datetime.fromisoformat(row[ts_col].replace('Z', '+00:00'))
            if skip_before and ts < skip_before:
                continue
            secs.append(int(row['ref_sec']))
            totals.append(int(row['ref_sec']) * _PS_PER_S + int(row['ref_ps']))
    if not totals:
        raise ValueError(f"{path}: no chA samples after skip-before")
    secs_arr = np.array(secs, dtype=np.int64)
    # Use Python-int linear detrend to keep precision; cast to float at the end.
    n = len(totals)
    # Fit a line in (ref_sec - ref_sec[0], total_ps - total_ps[0]) — both
    # zero-anchored to avoid float precision loss at TICC's ~10^6 boot epochs.
    x = (secs_arr - secs_arr[0]).astype(np.float64)
    y_int = [t - totals[0] for t in totals]   # python int, exact
    y = np.array(y_int, dtype=np.float64)     # ~10^14 ps max, fits float64 OK
    slope, intercept = np.polyfit(x, y, 1)
    resid_ps = y - (slope * x + intercept)
    phase_s = resid_ps * 1e-12
    return phase_s


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--host', action='append', nargs=3,
                   metavar=('LABEL', 'CSV', 'TAU_STAR'),
                   required=True,
                   help='Host label, TICC CSV path, predicted τ* (s). '
                        'Repeat for each host.')
    p.add_argument('--out', required=True, type=Path)
    p.add_argument('--skip-before', default=None,
                   help='UTC ISO time; ignore samples before this epoch')
    p.add_argument('--rate-hz', type=float, default=1.0)
    args = p.parse_args()

    print(provenance_line(_TOOLNAME), file=sys.stderr)

    skip_before = (datetime.fromisoformat(
        args.skip_before.replace('Z', '+00:00')) if args.skip_before else None)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    colors = ['C0', 'C1', 'C2', 'C3', 'C4']

    print(f'{"host":40s}  {"n":>6s}  {"span_h":>7s}  '
          f'{"σ_phase_RMS":>11s}  {"τ_min":>8s}  {"TDEV(τ_min)":>11s}  '
          f'{"TDEV(τ*)":>10s}  {"τ*":>6s}',
          file=sys.stderr)

    for i, (label, csv_path, tau_star_s) in enumerate(args.host):
        color = colors[i % len(colors)]
        tau_star = float(tau_star_s)
        phase_s = load_chA_phase_s(Path(csv_path), skip_before)
        n = len(phase_s)
        span_h = n / args.rate_hz / 3600

        # TDEV — log-spaced τ from 1 s up to a useful fraction of the span.
        # allantools.tdev returns (taus, tdev, terr, ns).
        taus_request = np.logspace(0, np.log10(span_h * 3600 / 4), 50)
        taus, tdev, terr, _ = allantools.tdev(
            phase_s, rate=args.rate_hz, data_type='phase', taus=taus_request)
        # Find the empirical minimum
        i_min = int(np.argmin(tdev))
        tau_min = float(taus[i_min])
        tdev_min = float(tdev[i_min])
        # TDEV at the predicted Goldilocks τ* (interpolate in log-log)
        log_t = np.log(taus)
        log_y = np.log(tdev)
        tdev_at_taustar = float(np.exp(np.interp(np.log(tau_star), log_t, log_y)))
        sigma_rms_ns = float(np.std(phase_s) * 1e9)
        print(f'{label:40s}  {n:>6d}  {span_h:>7.2f}  '
              f'{sigma_rms_ns:>9.3f} ns  {tau_min:>6.1f} s  '
              f'{tdev_min*1e9:>9.3f} ns  '
              f'{tdev_at_taustar*1e9:>8.3f} ns  {tau_star:>4.2f} s',
              file=sys.stderr)

        # Left: TDEV curves
        ax = axes[0]
        ax.loglog(taus, tdev * 1e9, '-', color=color, lw=1.6, label=label)
        ax.loglog([tau_star], [tdev_at_taustar * 1e9], 's', color=color,
                  markersize=10, markeredgecolor='black', markeredgewidth=1.0,
                  label=f'  predicted τ*={tau_star:g}s')
        ax.loglog([tau_min], [tdev_min * 1e9], 'o', color=color, markersize=8,
                  markerfacecolor='white', markeredgecolor=color,
                  markeredgewidth=1.5,
                  label=f'  empirical min τ={tau_min:.1f}s, '
                        f'TDEV={tdev_min*1e9:.2f}ns')
        # Slopes for reference
        if i == 0:
            # Reference slope lines (white-FM = -1/2, RWFM = +1/2, FFM = 0)
            tau_ref = np.array([1, 10000])
            # WFM at TDEV=1 ns at τ=1s
            ax.loglog(tau_ref, 1.0 * tau_ref ** (-0.5), '--',
                      color='gray', alpha=0.4, lw=0.8)
            ax.text(10, 0.32, 'WFM (−½)',
                    fontsize=8, color='gray', alpha=0.6)
            # RWFM at TDEV=0.05 ns at τ=1s
            ax.loglog(tau_ref, 0.05 * tau_ref ** (0.5), '--',
                      color='gray', alpha=0.4, lw=0.8)
            ax.text(100, 0.7, 'RWFM (+½)',
                    fontsize=8, color='gray', alpha=0.6)

    ax = axes[0]
    ax.set_xlabel('τ (s)')
    ax.set_ylabel('TDEV (ns)')
    ax.set_title('chA TDEV(τ) per host\n'
                 '(squares = PR #124 predicted Goldilocks τ*, '
                 'circles = empirical minimum)')
    ax.grid(True, which='both', alpha=0.3)
    ax.legend(fontsize=8, loc='upper left')

    # Right: zoom on the τ=0.5..10s region where the Goldilocks bowl
    # is supposed to live
    ax = axes[1]
    for i, (label, csv_path, tau_star_s) in enumerate(args.host):
        color = colors[i % len(colors)]
        tau_star = float(tau_star_s)
        phase_s = load_chA_phase_s(Path(csv_path), skip_before)
        taus_request = np.logspace(0, np.log10(60), 30)
        taus, tdev, _, _ = allantools.tdev(
            phase_s, rate=args.rate_hz, data_type='phase', taus=taus_request)
        ax.loglog(taus, tdev * 1e9, '-', color=color, lw=1.6, label=label)
        # Mark τ* with a vertical line
        ax.axvline(tau_star, color=color, ls=':', alpha=0.5, lw=1.0)
    ax.set_xlabel('τ (s)')
    ax.set_ylabel('TDEV (ns)')
    ax.set_title('Zoom: τ = 1..60 s (the Goldilocks neighborhood)\n'
                 '(dotted verticals = each host\'s predicted τ*)')
    ax.grid(True, which='both', alpha=0.3)
    ax.legend(fontsize=8, loc='best')

    plt.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    stamp(fig, _TOOLNAME)
    plt.savefig(args.out, dpi=120)
    print(f'\nWrote {args.out}', file=sys.stderr)


if __name__ == '__main__':
    sys.exit(main())

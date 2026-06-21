#!/usr/bin/env python3
"""Virtual cross-host diff when the two TICCs are on INDEPENDENT
reference clocks.

The sibling tool ``plot_xhost_virtual.py`` assumes both TICCs share a
common 10 MHz Rb reference — then chA − chA across hosts is the right
metric after linear detrend (constant cable offset + tiny rate diff
absorbed by the slope removal).

That assumption broke 2026-06-02 when PiFace's TICC #3 moved to a
private OCXO reference for the portable-clock experiment.  Now
PiFace's TICC timescale drifts non-linearly against clkPoC3's
Rb-referenced TICC #5 timescale — orders of magnitude beyond what a
single-slope detrend can absorb.

Common-view subtraction: each TICC's chB channel records the F9T
GNSS PPS edge.  Since both F9Ts share the lab antenna via the GUS
splitter, both chB streams measure the SAME physical GPS PPS edges
(modulo a constant splitter delay).  Per host:

    residual_host = chA_total_ps − chB_total_ps
                  = (DO_pps - TICC_ref) - (GNSS_pps - TICC_ref)
                  = DO_pps - GNSS_pps      [TICC_ref cancels]

Differencing residuals across hosts then cancels the GPS-time
common-mode, leaving only the per-host DO-vs-GPS tracking error
difference — independent of each host's local TICC reference.

    cross_host = residual_PiFace − residual_clkPoC3
               = (PiFace_DO - GNSS) − (clkPoC3_DO - GNSS)
               = PiFace_DO − clkPoC3_DO

Tradeoff vs the shared-Rb tool:

    + Works with TICCs on independent references (any rate diff OK)
    + Removes per-host TICC reference noise entirely
    − F9T GNSS PPS noise enters BOTH residuals: √2 × σ_F9T_PPS ≈ 3 ns
      at τ=1s instead of the shared-Rb tool's ~120 ps quadrature floor
    − Need long τ averaging (≥ 100 s) for sub-ns sensitivity

PRECISION INVARIANT (same as sibling tool): per-event ``total_ps``
stays in Python int through every subtraction; cast to float only
when divided into ns for downstream stats/plotting.

Usage:
    python3 tools/plot_xhost_indep_refs.py \\
        --input-A piface-ticc.csv  --label-A 'PiFace ticc3 (OCXO ref)' \\
        --input-B clkpoc3-ticc.csv --label-B 'clkPoC3 ticc5 (Rb ref)' \\
        --output  /tmp/piface-clkpoc3-indep.png \\
        --skip-before 2026-06-03T03:30:00Z
"""
from __future__ import annotations

import argparse
import csv
import sys
from datetime import datetime, timezone, timedelta
from pathlib import Path
from typing import NamedTuple

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Provenance stamp (analysis versioning + git-hash footer).
sys.path.insert(0, str(Path(__file__).resolve().parent))
from analysis_provenance import (stamp, provenance_line,  # noqa: E402
                                 skip_comment_lines)

_TOOLNAME = 'plot_xhost_indep_refs.py'

_PS_PER_S = 10 ** 12
_PAIR_KEY_RESOLUTION_S = 0.5


class Sample(NamedTuple):
    ts: datetime
    total_ps: int   # Python int — arbitrary precision


def _round_key(ts: datetime) -> float:
    return round(ts.timestamp() / _PAIR_KEY_RESOLUTION_S) * _PAIR_KEY_RESOLUTION_S


def load_per_host_residuals(
    path: Path,
    skip_before: datetime | None = None,
) -> tuple[list[datetime], list[int]]:
    """Load chA and chB rows from one host's TICC CSV; return per-epoch
    residual = chA_total_ps − chB_total_ps.

    chA and chB rows arrive interleaved (consecutive PPS edge measurements
    on the same TICC second); we pair them on the 0.5 s rounded-ts grid
    the same way the cross-host pairing works downstream.
    """
    # Within one host: chA and chB rows from the SAME physical PPS edge
    # share a ``ref_sec`` (TICC's boot-relative second).  Their
    # ``host_timestamp`` columns differ by ~5 ms (USB serial latency
    # between the two consecutive PPS edge reports).  Pair by ref_sec —
    # NOT by rounded wall-clock — so we don't accidentally match chA
    # from second N with chB from second N+1 when host_timestamp jitters
    # across the 0.5 s rounding boundary.
    a_by_sec: dict[int, Sample] = {}
    b_by_sec: dict[int, Sample] = {}
    with open(path) as f:
        reader = csv.DictReader(skip_comment_lines(f))
        if 'ts_iso' in reader.fieldnames:
            ts_col = 'ts_iso'
        elif 'host_timestamp' in reader.fieldnames:
            ts_col = 'host_timestamp'
        else:
            raise ValueError(
                f'{path}: expected ts_iso or host_timestamp column, '
                f'got {reader.fieldnames}')
        for row in reader:
            ts = datetime.fromisoformat(row[ts_col].replace('Z', '+00:00'))
            if skip_before and ts < skip_before:
                continue
            ref_sec = int(row['ref_sec'])
            total = ref_sec * _PS_PER_S + int(row['ref_ps'])
            sample = Sample(ts, total)
            if row['channel'] == 'chA':
                a_by_sec[ref_sec] = sample
            elif row['channel'] == 'chB':
                b_by_sec[ref_sec] = sample
    common = sorted(set(a_by_sec) & set(b_by_sec))
    ts_out: list[datetime] = []
    res_out: list[int] = []
    for s in common:
        sa = a_by_sec[s]
        sb = b_by_sec[s]
        ts_out.append(sa.ts)
        res_out.append(sa.total_ps - sb.total_ps)  # Python int, exact
    return ts_out, res_out


def cross_host_diff(
    A_times: list[datetime], A_res: list[int],
    B_times: list[datetime], B_res: list[int],
) -> tuple[list[datetime], np.ndarray]:
    """Pair per-host residuals across the two TICCs, differentiate.

    Both residuals are in ps in each host's local TICC timescale, but
    since each residual is chA−chB on the SAME TICC, the TICC reference
    cancels — so the difference between them is reference-independent.
    """
    a_by_key = {_round_key(t): (t, r) for t, r in zip(A_times, A_res)}
    b_by_key = {_round_key(t): (t, r) for t, r in zip(B_times, B_res)}
    common = sorted(set(a_by_key) & set(b_by_key))
    times: list[datetime] = []
    deltas: list[int] = []
    for k in common:
        ta, ra = a_by_key[k]
        _, rb = b_by_key[k]
        times.append(ta)
        deltas.append(ra - rb)
    return times, np.array(deltas, dtype=np.int64)


def _rolling_stdev_ns(times: list[datetime], series_ns: np.ndarray,
                      window_s: float) -> tuple[list[datetime], np.ndarray]:
    """5-minute (or other) rolling stdev of a ns-units series."""
    ts = np.array([t.timestamp() for t in times])
    out_t: list[datetime] = []
    out_s: list[float] = []
    i_lo = 0
    for i in range(len(series_ns)):
        while ts[i] - ts[i_lo] > window_s:
            i_lo += 1
        if i - i_lo >= 8:
            out_t.append(times[i])
            out_s.append(float(np.std(series_ns[i_lo:i + 1])))
    return out_t, np.array(out_s)


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--input-A', required=True, type=Path)
    p.add_argument('--input-B', required=True, type=Path)
    p.add_argument('--label-A', default='host A')
    p.add_argument('--label-B', default='host B')
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--skip-before', default=None,
                   help='UTC ISO time; ignore samples before this epoch')
    args = p.parse_args()

    print(provenance_line(_TOOLNAME), file=sys.stderr)

    skip_before = None
    if args.skip_before:
        skip_before = datetime.fromisoformat(
            args.skip_before.replace('Z', '+00:00'))

    A_t, A_r = load_per_host_residuals(args.input_A, skip_before)
    B_t, B_r = load_per_host_residuals(args.input_B, skip_before)
    print(f'{args.label_A}: n_chA∩chB={len(A_t)}  '
          f'span={(A_t[-1] - A_t[0]).total_seconds() / 3600:.2f} h',
          file=sys.stderr)
    print(f'{args.label_B}: n_chA∩chB={len(B_t)}  '
          f'span={(B_t[-1] - B_t[0]).total_seconds() / 3600:.2f} h',
          file=sys.stderr)

    times, deltas_ps = cross_host_diff(A_t, A_r, B_t, B_r)
    print(f'\ncross-host paired epochs: {len(times)}', file=sys.stderr)

    # Constant-offset detrend ONLY.  Slope detrend is the wrong move
    # here: a real frequency disagreement between disciplined DOs IS
    # exactly what we want to surface, and there's no shared-Rb-related
    # slope to absorb (chB common-view already cancelled both TICC refs).
    mean_offset_ps = int(np.mean(deltas_ps))
    deltas_centered_ps = deltas_ps - mean_offset_ps
    deltas_ns = deltas_centered_ps.astype(np.float64) / 1000.0
    print(f'mean offset removed: {mean_offset_ps / 1000:+.2f} ns '
          '(cable + splitter delay difference)', file=sys.stderr)

    # Per-host residuals in ns, centered (each host's own mean off,
    # so the per-host residual panel shows the time-varying part only).
    A_res_ns = np.array(A_r, dtype=np.float64) / 1000.0
    A_res_ns -= np.mean(A_res_ns)
    B_res_ns = np.array(B_r, dtype=np.float64) / 1000.0
    B_res_ns -= np.mean(B_res_ns)

    abs_ns = np.abs(deltas_ns)
    rms_ns = float(np.sqrt(np.mean(deltas_ns ** 2)))
    p95_ns = float(np.percentile(abs_ns, 95))
    p99_ns = float(np.percentile(abs_ns, 99))
    n_zc = int(np.sum(np.diff(np.sign(deltas_ns)) != 0))
    lead = float(np.mean(deltas_ns > 0)) * 100
    lag = float(np.mean(deltas_ns < 0)) * 100
    print(f'\nSigned-Δ statistics (cross-host DO−DO, refs cancelled):',
          file=sys.stderr)
    print(f'  RMS                : {rms_ns:.2f} ns', file=sys.stderr)
    print(f'  p95|·|             : {p95_ns:.2f} ns', file=sys.stderr)
    print(f'  p99|·|             : {p99_ns:.2f} ns', file=sys.stderr)
    print(f'  zero-crossings     : {n_zc}', file=sys.stderr)
    print(f'  lead fraction (>0) : {lead:.1f} %  (A ahead of B)',
          file=sys.stderr)
    print(f'  lag  fraction (<0) : {lag:.1f} %  (A behind B)',
          file=sys.stderr)

    # Rolling 5-min stdev
    roll_t, roll_s = _rolling_stdev_ns(times, deltas_ns, 300.0)
    if len(roll_s) > 0:
        print(f'\nRolling 5-min stdev of signed Δ (ns):', file=sys.stderr)
        print(f'  min/median/max     : '
              f'{np.min(roll_s):.2f}  /  {np.median(roll_s):.2f}  /  '
              f'{np.max(roll_s):.2f}', file=sys.stderr)

    A_p95 = float(np.percentile(np.abs(A_res_ns), 95))
    B_p95 = float(np.percentile(np.abs(B_res_ns), 95))
    A_rms = float(np.sqrt(np.mean(A_res_ns ** 2)))
    B_rms = float(np.sqrt(np.mean(B_res_ns ** 2)))
    print(f'\nPer-host residuals (chA − chB, centered) ns:', file=sys.stderr)
    print(f'  {args.label_A:30s} : rms={A_rms:.3f} ns  p95|·|={A_p95:.3f} ns',
          file=sys.stderr)
    print(f'  {args.label_B:30s} : rms={B_rms:.3f} ns  p95|·|={B_p95:.3f} ns',
          file=sys.stderr)

    # ── Plot ────────────────────────────────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    hrs = [(t - times[0]).total_seconds() / 3600 for t in times]
    A_hrs = [(t - A_t[0]).total_seconds() / 3600 for t in A_t]
    B_hrs = [(t - B_t[0]).total_seconds() / 3600 for t in B_t]

    # Top: signed cross-host Δ
    ax = axes[0]
    ax.plot(hrs, deltas_ns, lw=0.6, color='C0')
    ax.axhline(0, color='gray', lw=0.5)
    ax.set_ylabel('signed Δ (ns)\n' + r'$\Delta = $ ' +
                  f'{args.label_A} − {args.label_B}')
    ax.set_title(f'{args.label_A}  ↔  {args.label_B}  — '
                 'independent-refs cross-host (chB common-view subtraction)\n'
                 f'n={len(times)}, RMS={rms_ns:.2f} ns, '
                 f'p95={p95_ns:.2f} ns, '
                 f'lead={lead:.1f}% / lag={lag:.1f}%')
    ax.grid(True, alpha=0.3)

    # Middle: per-host residuals (chA-chB) overlaid
    ax = axes[1]
    ax.plot(A_hrs, A_res_ns, lw=0.5, color='C1', label=args.label_A, alpha=0.7)
    ax.plot(B_hrs, B_res_ns, lw=0.5, color='C2', label=args.label_B, alpha=0.7)
    ax.axhline(0, color='gray', lw=0.5)
    ax.set_ylabel('per-host residual (ns)\n(chA − chB, centered)')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # Bottom: rolling 5-min stdev
    ax = axes[2]
    if len(roll_s) > 0:
        roll_hrs = [(t - times[0]).total_seconds() / 3600 for t in roll_t]
        ax.plot(roll_hrs, roll_s, lw=0.8, color='C3')
        ax.set_ylabel('rolling 5-min\nσ of signed Δ (ns)')
    ax.set_xlabel('hours since first paired epoch')
    ax.set_title('Stability tracker — lower = more stable')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    stamp(fig, _TOOLNAME)
    plt.savefig(args.output, dpi=120)
    print(f'\nWrote {args.output}', file=sys.stderr)


if __name__ == '__main__':
    sys.exit(main())

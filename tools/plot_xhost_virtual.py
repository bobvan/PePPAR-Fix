#!/usr/bin/env python3
"""Virtual cross-host diff from two per-host TICC chA streams.

The lab's TICCs share a common 10 MHz Rb reference, so each TICC's
timescale differs from the others only by a constant offset (cable
length, boot epoch) and a near-zero rate (all locked to the shared
Rb).  This means we can compare ANY pair of disciplined-clock PPS
edges by subtracting the per-host TICC-chA timestamp streams — we
don't need a dedicated cross-host TICC (TICC4 in the lab) to wire
the two PPS signals into a single time-interval counter.

Validated 2026-05-31 by direct comparison of (PiFace TICC3 − clkPoC3
TICC5) virtual diff against TICC4's direct read on the same epochs:
RMS/p95/p99 agreed within 0.1 %; Pearson r = 0.9987; sample-by-sample
agreement was 565 ps RMS, consistent with the two-TICC TDC noise
quadrature (~85 ps per sample × √N).

Compared to a dedicated cross-host TICC the virtual diff has:
  + supports any pair of hosts sharing the lab Rb (no rewiring needed)
  + bias-free aggregate stats (constant offset + linear drift removed)
  − ~30 % more sample-level measurement noise (two TDCs in quadrature)
  − more zero-crossings due to the extra noise (use for CDF/RMS/p95;
    don't use raw zero-crossing counts as a primary metric)

Inputs:
- Two per-host TICC csvs containing chA samples.  Schema-agnostic —
  accepts both ``ticc_capture.py`` format (``ts_iso, channel,
  ref_sec, ref_ps, recv_mono``) and engine TICC-log format
  (``host_timestamp, host_monotonic, ref_sec, ref_ps, channel``).

Outputs:
- PNG with three panels: signed Δ over time, per-host residuals
  (5-min block detrend), rolling 5-min stdev.
- Stats to stderr: zero-crossings, lead/lag fractions, signed-Δ
  RMS/p95/p99, per-host residual RMS/p95/fast-fraction.
- Optional ``--validate-vs PATH`` accepts a dedicated cross-host TICC
  csv (e.g., TICC4) and prints how closely the virtual diff matches.

PRECISION INVARIANT
-------------------
Per the lesson from ``plot_xhost_agreement_cdf.py``: per-event
``total_ps = sec * 1e12 + ps`` is built in Python int (arbitrary
precision); the pairing subtract Δ = A_total − B_total stays in
Python int; result cast to int64 only after the subtraction.  Each
decade of TICC ref_sec runtime costs a decade of ps precision in
float64 (sec ~10⁶ → ~100 ps quantization), so sec and ps must stay
separate until after subtraction.

Usage:
    python3 tools/plot_xhost_virtual.py \\
        --input-A piface-ticc.csv  --label-A 'PiFace ticc3' \\
        --input-B clkpoc3-ticc.csv --label-B 'clkPoC3 ticc5' \\
        --output  /tmp/piface-clkpoc3-virtual.png \\
        --skip-before 2026-05-31T03:30:00Z \\
        --validate-vs ticc4-direct.csv     # optional
"""
from __future__ import annotations

import argparse
import bisect
import csv
import sys
from datetime import datetime, timezone
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

_TOOLNAME = 'plot_xhost_virtual.py'

_PS_PER_S = 10 ** 12
_PAIR_TOLERANCE_PS = 5 * 10 ** 11
_PAIR_KEY_RESOLUTION_S = 0.5   # NTP-synced hosts agree within ~ms; 0.5s grid
                               # gives slack for occasional outliers.


class Sample(NamedTuple):
    ts: datetime
    total_ps: int   # Python int — arbitrary precision


def _round_key(ts: datetime) -> float:
    """Round a UTC timestamp to a _PAIR_KEY_RESOLUTION_S grid for pairing."""
    return round(ts.timestamp() / _PAIR_KEY_RESOLUTION_S) * _PAIR_KEY_RESOLUTION_S


def load_ticc_cha(path: Path,
                  skip_before: datetime | None = None) -> list[Sample]:
    """Read chA samples — schema-agnostic across ticc_capture.py and engine logs."""
    samples: list[Sample] = []
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
            if row['channel'] != 'chA':
                continue
            ts = datetime.fromisoformat(row[ts_col].replace('Z', '+00:00'))
            if skip_before and ts < skip_before:
                continue
            total = int(row['ref_sec']) * _PS_PER_S + int(row['ref_ps'])
            samples.append(Sample(ts, total))
    return samples


def virtual_diff(A: list[Sample], B: list[Sample]
                 ) -> tuple[list[datetime], np.ndarray]:
    """Compute virtual diff Δ = A − B at matching (rounded-ts) epochs.

    Each TICC ref_sec is relative to its own boot, so we can't pair by
    ref_sec across streams.  Both hosts are NTP-disciplined, so the
    ``ts_iso`` / ``host_timestamp`` wall-clock columns agree to within
    milliseconds — sufficient for pairing at a 0.5 s grid.
    """
    A_by_key = {_round_key(s.ts): s for s in A}
    B_by_key = {_round_key(s.ts): s for s in B}
    common = sorted(set(A_by_key) & set(B_by_key))
    times: list[datetime] = []
    deltas: list[int] = []
    for k in common:
        sa = A_by_key[k]
        sb = B_by_key[k]
        # Subtract-separately Python int math — exact, no precision loss.
        # No magnitude-based outlier check here: the boot-epoch differential
        # between the two TICCs (different boot times, different cable
        # lengths to the shared Rb) can easily run to seconds.  That
        # constant + the slow drift slope are removed by linear detrend
        # downstream.  Pairing-quality is enforced by the ts-rounded key.
        times.append(sa.ts)
        deltas.append(sa.total_ps - sb.total_ps)
    return times, np.array(deltas, dtype=np.int64)


def load_direct_xhost_diff(path: Path,
                           skip_before: datetime | None = None
                           ) -> tuple[list[datetime], np.ndarray]:
    """Read a dedicated cross-host TICC csv (chA = host A PPS, chB = host B)."""
    chA: list[Sample] = []
    chB: list[Sample] = []
    with open(path) as f:
        reader = csv.DictReader(skip_comment_lines(f))
        ts_col = ('ts_iso' if 'ts_iso' in reader.fieldnames
                  else 'host_timestamp')
        for row in reader:
            ts = datetime.fromisoformat(row[ts_col].replace('Z', '+00:00'))
            if skip_before and ts < skip_before:
                continue
            total = int(row['ref_sec']) * _PS_PER_S + int(row['ref_ps'])
            if row['channel'] == 'chA':
                chA.append(Sample(ts, total))
            elif row['channel'] == 'chB':
                chB.append(Sample(ts, total))
    chB_sorted = sorted(s.total_ps for s in chB)
    times: list[datetime] = []
    deltas: list[int] = []
    for sa in chA:
        idx = bisect.bisect_left(chB_sorted, sa.total_ps)
        cands = []
        if idx > 0: cands.append(chB_sorted[idx - 1])
        if idx < len(chB_sorted): cands.append(chB_sorted[idx])
        if not cands: continue
        best = min(cands, key=lambda b: abs(sa.total_ps - b))
        diff = sa.total_ps - best
        if abs(diff) > _PAIR_TOLERANCE_PS:
            continue
        times.append(sa.ts)
        deltas.append(diff)
    return times, np.array(deltas, dtype=np.int64)


def detrend_signed(times: list[datetime],
                   delta_ps: np.ndarray
                   ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Linear-detrend the signed Δ series.  Removes the constant offset
    (cable / boot epoch differential) and any residual rate skew.
    Returns (t_sec_since_start, detrended_delta_ps_float, poly_coeffs).
    """
    if not times:
        return np.array([]), np.array([]), np.array([0.0, 0.0])
    t0 = times[0]
    t_sec = np.array([(t - t0).total_seconds() for t in times])
    d_f = delta_ps.astype(np.float64)
    poly = np.polyfit(t_sec, d_f, 1)
    return t_sec, d_f - np.polyval(poly, t_sec), poly


def per_host_block_residuals(path: Path,
                             block_s: int = 300,
                             skip_before: datetime | None = None
                             ) -> tuple[list[datetime], np.ndarray]:
    """5-min-block-detrended residuals for a single host's chA stream.

    Subtract the integer 1Hz rate (block-local) in int math, then
    polyfit each block in float64.  Residuals returned in ns.  The
    block length is short enough that the local TICC Rb drift is
    negligible at this scale.

    Gap handling: blocks are restarted at any discontinuity in
    ``ref_sec`` (engine restart, intentional disturbance-window
    edit on the host).  Without this, the index-based 1 Hz rate
    subtraction inflates the per-host residual by ~PS_PER_S × gap_s
    on the post-gap samples (caught by the 2026-05-31 MadHat
    overnight test: disturbance trim created a 1621-s gap).
    """
    samples = load_ticc_cha(path, skip_before=skip_before)
    if len(samples) < 60:
        return [], np.array([])
    # Identify contiguous runs by checking ref_sec strides between
    # consecutive samples.  A gap-of-1 is the normal 1 Hz step; anything
    # > 2 s starts a new run.
    secs = [s.total_ps // _PS_PER_S for s in samples]
    run_starts: list[int] = [0]
    for i in range(1, len(samples)):
        if secs[i] - secs[i - 1] > 2:   # any gap > 1 normal step
            run_starts.append(i)
    run_starts.append(len(samples))

    times = [s.ts for s in samples]
    residuals = np.zeros(len(samples), dtype=np.float64)

    # Per-run, then per-block within the run.
    for rs_idx in range(len(run_starts) - 1):
        run_lo, run_hi = run_starts[rs_idx], run_starts[rs_idx + 1]
        # Block-local origin so the rate subtraction is per-run.
        run_s0 = int(samples[run_lo].total_ps // _PS_PER_S)
        run_p0 = int(samples[run_lo].total_ps % _PS_PER_S)
        for start in range(run_lo, run_hi, block_s):
            end = min(start + block_s, run_hi)
            if end - start < 60:
                continue
            block = samples[start:end]
            dev_int = np.empty(end - start, dtype=np.int64)
            for i, s in enumerate(block):
                sec = int(s.total_ps // _PS_PER_S)
                ps = int(s.total_ps % _PS_PER_S)
                # Block-local 1Hz rate from the RUN origin (not the
                # whole-file origin) so a prior gap doesn't poison this
                # block's math.
                offset_in_run = (start - run_lo) + i
                dev_int[i] = ((sec - run_s0 - offset_in_run) * _PS_PER_S
                              + (ps - run_p0))
            dev_f = dev_int.astype(np.float64)
            x = np.arange(end - start, dtype=np.float64)
            poly = np.polyfit(x, dev_f, 1)
            residuals[start:end] = dev_f - np.polyval(poly, x)
    return times, residuals * 1e-3   # ps → ns


def load_scheduler_interval(arm_state_path: Path | None,
                            skip_before: datetime | None = None
                            ) -> tuple[np.ndarray, np.ndarray]:
    """Read (t_h, dt_actual_s) from an arm-state.csv.

    ``dt_actual_s`` is the engine's effective scheduler interval per
    EKF-update epoch — the wall-clock time elapsed since the previous
    update.  When the discipline scheduler is in tight-cadence mode
    it sits at 1 s; when relaxed it can climb to many tens of seconds.
    Plotting this lets the operator see how the scheduler reacts to
    convergence + transients across the run.

    Returns (t_hours_since_first, dt_actual_s_array).  Empty arrays
    if the file is missing.
    """
    if arm_state_path is None or not arm_state_path.exists():
        return np.array([]), np.array([])
    times: list[datetime] = []
    intervals: list[float] = []
    with open(arm_state_path) as f:
        reader = csv.DictReader(skip_comment_lines(f))
        for row in reader:
            ts_raw = row.get('host_timestamp', '').strip()
            if not ts_raw:
                continue
            ts = datetime.fromisoformat(ts_raw.replace('Z', '+00:00'))
            if skip_before is not None and ts < skip_before:
                continue
            v = row.get('dt_actual_s', '').strip()
            if not v:
                continue
            try:
                intervals.append(float(v))
            except ValueError:
                continue
            times.append(ts)
    if not times:
        return np.array([]), np.array([])
    t0 = times[0]
    t_h = np.array([(t - t0).total_seconds() / 3600.0 for t in times])
    return t_h, np.array(intervals, dtype=np.float64)


def fmt_ns(v: float) -> str:
    if abs(v) < 1: return f'{v * 1e3:.0f} ps'
    if abs(v) < 1e3: return f'{v:.2f} ns'
    if abs(v) < 1e6: return f'{v / 1e3:.2f} µs'
    return f'{v / 1e6:.2f} ms'


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--input-A', required=True, type=Path,
                    help='Host A TICC chA csv')
    ap.add_argument('--input-B', required=True, type=Path,
                    help='Host B TICC chA csv')
    ap.add_argument('--label-A', default='Host A')
    ap.add_argument('--label-B', default='Host B')
    ap.add_argument('--output', required=True, type=Path,
                    help='Output PNG path')
    ap.add_argument('--title', default=None)
    ap.add_argument('--skip-before', default=None,
                    help='ISO-8601 UTC timestamp; drop earlier samples '
                         '(bootstrap trim)')
    ap.add_argument('--validate-vs', default=None, type=Path,
                    help='Optional path to a dedicated cross-host TICC '
                         'csv (chA = host A, chB = host B); the tool '
                         'prints how the virtual diff matches.')
    ap.add_argument('--arm-state-A', default=None, type=Path,
                    help='Per-host arm-state.csv for host A.  If omitted, '
                         "the tool tries to auto-infer by replacing "
                         "'-ticc.csv' with '-arm-state.csv' in --input-A. "
                         'Used to plot the discipline scheduler interval '
                         '(dt_actual_s) panel.')
    ap.add_argument('--arm-state-B', default=None, type=Path,
                    help='Per-host arm-state.csv for host B (see --arm-state-A).')
    args = ap.parse_args()

    print(provenance_line(_TOOLNAME), file=sys.stderr)

    # Auto-infer arm-state paths if not given.
    if args.arm_state_A is None:
        guess = Path(str(args.input_A).replace('-ticc.csv', '-arm-state.csv'))
        if guess != args.input_A and guess.exists():
            args.arm_state_A = guess
    if args.arm_state_B is None:
        guess = Path(str(args.input_B).replace('-ticc.csv', '-arm-state.csv'))
        if guess != args.input_B and guess.exists():
            args.arm_state_B = guess

    skip = None
    if args.skip_before:
        skip = datetime.fromisoformat(args.skip_before.replace('Z', '+00:00'))

    A = load_ticc_cha(args.input_A, skip)
    B = load_ticc_cha(args.input_B, skip)
    print(f'{args.label_A}: n={len(A)}  '
          f'span={(A[-1].ts - A[0].ts).total_seconds() / 3600:.2f} h',
          file=sys.stderr)
    print(f'{args.label_B}: n={len(B)}  '
          f'span={(B[-1].ts - B[0].ts).total_seconds() / 3600:.2f} h',
          file=sys.stderr)

    times, delta_ps = virtual_diff(A, B)
    if not times:
        print('no overlap — aborting', file=sys.stderr)
        return 1
    print(f'\nvirtual paired epochs: {len(delta_ps)}', file=sys.stderr)

    t_sec, dev_dt, poly = detrend_signed(times, delta_ps)
    print(f'detrend slope removed:  {poly[0] * 1e3:+.2f} ps/ks',
          file=sys.stderr)
    print(f'detrend constant offset: {fmt_ns(poly[1] / 1e3)}', file=sys.stderr)

    # Aggregate stats on the detrended signed series
    rms_ns = float(np.sqrt(np.mean(dev_dt ** 2))) / 1e3
    p95 = float(np.percentile(np.abs(dev_dt), 95)) / 1e3
    p99 = float(np.percentile(np.abs(dev_dt), 99)) / 1e3
    zc = int(np.sum(np.diff(np.sign(dev_dt)) != 0))
    pos_frac = float(np.mean(dev_dt > 0)) * 100
    print(f'\nSigned-Δ statistics:', file=sys.stderr)
    print(f'  RMS                : {rms_ns:.2f} ns', file=sys.stderr)
    print(f'  p95|·|             : {p95:.2f} ns', file=sys.stderr)
    print(f'  p99|·|             : {p99:.2f} ns', file=sys.stderr)
    print(f'  zero-crossings     : {zc} (note: more sensitive to '
          f'two-TICC noise than to actual clock dynamics)', file=sys.stderr)
    print(f'  lead fraction (>0) : {pos_frac:.1f} %  (A ahead of B)',
          file=sys.stderr)
    print(f'  lag  fraction (<0) : {100 - pos_frac:.1f} %  (A behind B)',
          file=sys.stderr)

    # Rolling 5-min stdev
    W = 300
    stride = 60
    roll_t, roll_std = [], []
    for i in range(W, len(dev_dt), stride):
        roll_t.append(t_sec[i] / 3600)
        roll_std.append(float(np.std(dev_dt[i - W:i])) / 1e3)
    print(f'\nRolling 5-min stdev of signed Δ (ns):', file=sys.stderr)
    print(f'  min/median/max     : '
          f'{min(roll_std):.2f}  /  {np.median(roll_std):.2f}  /  '
          f'{max(roll_std):.2f}', file=sys.stderr)

    # Per-host residuals (5-min blocks)
    t_a, res_a = per_host_block_residuals(args.input_A, skip_before=skip)
    t_b, res_b = per_host_block_residuals(args.input_B, skip_before=skip)
    print(f'\nPer-host 5-min-block residuals (ns) vs each host local '
          f'TICC Rb:', file=sys.stderr)
    for label, res in [(args.label_A, res_a), (args.label_B, res_b)]:
        rms = float(np.sqrt(np.mean(res ** 2)))
        p95r = float(np.percentile(np.abs(res), 95))
        fast = float(np.mean(res > 0)) * 100
        print(f'  {label:25s}: rms={rms:.3f} ns  p95|·|={p95r:.3f} ns  '
              f'frac(running fast)={fast:.1f}%', file=sys.stderr)

    # Optional: validate against a dedicated cross-host TICC csv
    if args.validate_vs:
        ref_times, ref_delta = load_direct_xhost_diff(args.validate_vs, skip)
        _, ref_dt, _ = detrend_signed(ref_times, ref_delta)
        ref_by_key = {_round_key(t): v for t, v in zip(ref_times, ref_dt)}
        vir_by_key = {_round_key(t): v for t, v in zip(times, dev_dt)}
        shared = sorted(set(ref_by_key) & set(vir_by_key))
        v_p = np.array([vir_by_key[k] for k in shared])
        r_p = np.array([ref_by_key[k] for k in shared])
        d = v_p - r_p
        corr = float(np.corrcoef(v_p, r_p)[0, 1])
        print(f'\n── validation vs {args.validate_vs.name} '
              f'({len(shared)} shared epochs) ──', file=sys.stderr)
        print(f'  virtual − direct RMS  : {np.sqrt(np.mean(d ** 2)):.0f} ps',
              file=sys.stderr)
        print(f'  virtual − direct p95  : '
              f'{np.percentile(np.abs(d), 95):.0f} ps', file=sys.stderr)
        print(f'  Pearson r             : {corr:.6f}', file=sys.stderr)
        print(f'  virtual RMS  : {np.sqrt(np.mean(v_p ** 2)) / 1e3:.2f} ns   '
              f'direct RMS  : {np.sqrt(np.mean(r_p ** 2)) / 1e3:.2f} ns',
              file=sys.stderr)
        print(f'  virtual p95  : '
              f'{np.percentile(np.abs(v_p), 95) / 1e3:.2f} ns   '
              f'direct p95  : '
              f'{np.percentile(np.abs(r_p), 95) / 1e3:.2f} ns',
              file=sys.stderr)

    # Plot
    # Load scheduler interval series per host (optional)
    sched_t_a, sched_a = load_scheduler_interval(args.arm_state_A, skip)
    sched_t_b, sched_b = load_scheduler_interval(args.arm_state_B, skip)
    have_sched = len(sched_a) > 0 or len(sched_b) > 0
    if have_sched:
        print(f'\nDiscipline scheduler interval (dt_actual_s):',
              file=sys.stderr)
        for lbl, dt in [(args.label_A, sched_a), (args.label_B, sched_b)]:
            if len(dt) == 0:
                continue
            print(f'  {lbl:25s}: n={len(dt):5d}  median={np.median(dt):.2f} s  '
                  f'p95={np.percentile(dt, 95):.1f} s  '
                  f'max={dt.max():.1f} s', file=sys.stderr)

    title = args.title or f'{args.label_A} ↔ {args.label_B} — virtual cross-host'
    n_panels = 4 if have_sched else 3
    fig, axes = plt.subplots(n_panels, 1, figsize=(11, 2.8 * n_panels),
                              sharex=True)

    ax = axes[0]
    ax.plot(t_sec / 3600, dev_dt / 1e3, lw=0.4, color='#1f77b4')
    ax.axhline(0, color='black', lw=0.5)
    ax.set_ylabel(f'signed Δ = {args.label_A} − {args.label_B} (ns)\n'
                  f'[lead +  /  lag −]')
    ax.set_title(f'{title}\n'
                 f'n={len(dev_dt)}, RMS={rms_ns:.2f} ns, p95={p95:.2f} ns, '
                 f'lead={pos_frac:.1f}% / lag={100 - pos_frac:.1f}%')
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    n_show = min(len(res_a), len(res_b))
    t_h = np.arange(n_show) / 3600
    ax.plot(t_h, res_a[:n_show], lw=0.4, color='#ff7f0e',
            label=args.label_A, alpha=0.8)
    ax.plot(t_h, res_b[:n_show], lw=0.4, color='#2ca02c',
            label=args.label_B, alpha=0.8)
    ax.axhline(0, color='black', lw=0.5)
    ax.set_ylabel('per-host residual (ns)\n'
                  '[fast +  /  slow −]\n5-min block detrend')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    ax.plot(roll_t, roll_std, lw=0.6, color='#d62728')
    ax.set_ylabel('rolling 5-min\nstdev of signed Δ (ns)')
    if not have_sched:
        ax.set_xlabel('hours since first paired epoch')
    ax.set_title('Stability tracker — lower = more stable')
    ax.grid(True, alpha=0.3)

    if have_sched:
        ax = axes[3]
        if len(sched_a) > 0:
            ax.plot(sched_t_a, sched_a, lw=0.5, color='#ff7f0e',
                    label=args.label_A, alpha=0.85)
        if len(sched_b) > 0:
            ax.plot(sched_t_b, sched_b, lw=0.5, color='#2ca02c',
                    label=args.label_B, alpha=0.85)
        ax.set_ylabel('discipline scheduler\ninterval (s)\n[low = tight cadence]')
        ax.set_xlabel('hours since first paired epoch')
        ax.set_yscale('log')
        ax.legend(loc='upper right', fontsize=9)
        ax.grid(True, which='both', alpha=0.3)

    fig.tight_layout()
    stamp(fig, _TOOLNAME)
    fig.savefig(args.output, dpi=130)
    print(f'\nWrote {args.output}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())

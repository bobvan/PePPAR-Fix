#!/usr/bin/env python3
"""Per-clock stability overlay — TDEV / ADEV vs tau for a clock pair.

Visualizes the moonshot goal: at every tau, the disciplined DO output
should be as stable as the *best* of (DO free-running noise floor, GNSS
PPS noise floor).  Short-tau should ride the OCXO floor; long-tau
should ride the GPS-tracking floor; the transition should be smooth
with no servo-induced kinks.

Produces a single figure with TDEV and ADEV subplots, log-log axes.
For each host pair (typically two), overlays:

  * disciplined chA (TICC-measured DO PPS, last N hours, linear-detrended)
  * freerun chA from the host's pre-recorded char JSON (the noise floor
    we cannot beat via the servo)
  * GNSS PPS chB (typically one curve — F9T sawtooth dominates,
    similar across hosts)
  * (TODO) qErr-corrected chB once the engine logs per-epoch qErr.

Reference lines at:
  - τ = 1 s × 350 ps  (per-clock moonshot budget from
                       docs/two-site-sync-budget.md)
  - τ = 1 s × 60 ps   (TICC single-shot measurement resolution)

PRECISION INVARIANT — TICC timestamps are kept as integer (sec, ps)
throughout the per-epoch arithmetic.  See ``load_chA_phase``.

Usage:
    python3 tools/plot_clock_stability_stack.py \\
        --host-input 'PiFace=data/piface-ticc.csv,state/dos/ocxo-piface.json' \\
        --host-input 'MadHat=data/madhat-ticc.csv,state/dos/isotemp-ocxo-33-madhat.json' \\
        --skip-before 2026-05-31T03:30:00Z \\
        --output /tmp/clock-stability-stack.png \\
        --title 'PiFace + MadHat — moonshot stack overlay'

Each ``--host-input`` value is ``LABEL=TICC_CSV[,FREERUN_JSON]``.  The
freerun JSON must contain a ``characterization.sources['DO PPS (chA vs
TICC Rb)']`` block with ``adev_by_tau_s`` and ``tdev_ns_by_tau_s`` maps
(written by ``scripts/do_freerun_char.py``).  Omit the JSON path to
plot disciplined-only for that host.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import allantools


_PS_PER_S = 10 ** 12
_HOST_COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
_TAUS = np.array([1, 2, 5, 10, 30, 100, 300, 1000, 3000, 10000],
                 dtype=np.float64)


def _read_ts(row: dict, ts_col: str) -> datetime:
    return datetime.fromisoformat(row[ts_col].replace('Z', '+00:00'))


def load_chA_phase(path: Path, skip_before: datetime | None = None,
                   channel: str = 'chA') -> np.ndarray:
    """Load one channel's phase residual (ns), linear-detrended, from a
    standard TICC csv (engine TICC-log or ticc_capture.py).

    Subtract-separately invariant: per-event total_ps = sec * PS_PER_S +
    ps is built in Python int (arbitrary precision); the deviation from
    a perfect 1 Hz rate is computed in int64; only then cast to float64.
    See docs/moonshot-overnight-2026-05-31.md for the math.
    """
    samples: list[tuple[int, int]] = []
    with open(path) as f:
        reader = csv.DictReader(f)
        ts_col = ('ts_iso' if 'ts_iso' in reader.fieldnames
                  else 'host_timestamp' if 'host_timestamp' in reader.fieldnames
                  else None)
        for row in reader:
            if row['channel'] != channel:
                continue
            if skip_before is not None and ts_col is not None:
                if _read_ts(row, ts_col) < skip_before:
                    continue
            samples.append((int(row['ref_sec']), int(row['ref_ps'])))
    if len(samples) < 60:
        return np.array([])

    # Take the longest gap-free segment so 1Hz-index math stays correct.
    secs = np.array([s for s, _ in samples])
    diffs = np.diff(secs) if len(secs) > 1 else np.array([])
    bounds = ([0] + list(np.where(diffs != 1)[0] + 1) + [len(samples)]
              if len(diffs) else [0, len(samples)])
    segs = [(bounds[i], bounds[i + 1]) for i in range(len(bounds) - 1)]
    a, b = max(segs, key=lambda ab: ab[1] - ab[0])
    seg = samples[a:b]
    if len(seg) < 60:
        return np.array([])

    s0, p0 = seg[0]
    dev_int = np.array(
        [(s - s0 - i) * _PS_PER_S + (p - p0) for i, (s, p) in enumerate(seg)],
        dtype=np.int64)
    dev_ns = dev_int.astype(np.float64) * 1e-3
    # Linear-detrend to drop the constant freq offset (= integrated
    # rate difference between the DO and the TICC's Rb at this parked
    # state, not a stability metric).
    t = np.arange(len(dev_ns), dtype=np.float64)
    poly = np.polyfit(t, dev_ns, 1)
    return dev_ns - np.polyval(poly, t)


def adev_tdev_from_phase(phase_ns: np.ndarray
                         ) -> tuple[dict, dict]:
    """Compute ADEV and TDEV maps at _TAUS using allantools."""
    if len(phase_ns) < 60:
        return {}, {}
    phase_s = phase_ns * 1e-9
    # Allantools wants taus filtered to those with enough samples.
    feasible = _TAUS[_TAUS * 3 < len(phase_s)]
    if not len(feasible):
        return {}, {}
    taus_a, adev, _, _ = allantools.adev(phase_s, rate=1.0,
                                         taus=feasible, data_type='phase')
    taus_t, tdev, _, _ = allantools.tdev(phase_s, rate=1.0,
                                         taus=feasible, data_type='phase')
    return ({float(t): float(a) for t, a in zip(taus_a, adev)},
            {float(t): float(td * 1e9) for t, td in zip(taus_t, tdev)})


def load_freerun_char(path: Path
                      ) -> tuple[dict, dict] | tuple[None, None]:
    """Pull adev/tdev maps from a do_freerun_char.py JSON (chA-vs-Rb)."""
    if not path or not path.exists():
        return None, None
    data = json.loads(path.read_text())
    cha = (data.get('characterization', {}).get('sources', {})
                .get('DO PPS (chA vs TICC Rb)') or {})
    adev_map = cha.get('adev_by_tau_s') or {}
    tdev_map = cha.get('tdev_ns_by_tau_s') or {}
    # Map keys come in as strings ("1.0", "2.0"...) — normalize.
    adev = {float(k): float(v) for k, v in adev_map.items()
            if v is not None and v > 0}
    tdev = {float(k): float(v) for k, v in tdev_map.items()
            if v is not None and v > 0}
    return adev, tdev


def _parse_host_input(spec: str
                      ) -> tuple[str, Path, Path | None]:
    """``LABEL=TICC_CSV[,FREERUN_JSON]`` → (label, ticc_path, freerun_path)."""
    if '=' not in spec:
        raise ValueError(f'expected LABEL=PATH..., got {spec!r}')
    label, rest = spec.split('=', 1)
    parts = [s.strip() for s in rest.split(',')]
    ticc = Path(parts[0])
    freerun = Path(parts[1]) if len(parts) > 1 and parts[1] else None
    return label.strip(), ticc, freerun


def _plot_map(ax, x_taus: np.ndarray, ymap: dict, color: str,
              ls: str, label: str, lw: float = 1.8, marker: str = 'o',
              alpha: float = 1.0) -> None:
    if not ymap:
        return
    pts = sorted(ymap.items())
    xs = np.array([t for t, _ in pts])
    ys = np.array([v for _, v in pts])
    ax.plot(xs, ys, color=color, ls=ls, lw=lw, marker=marker, ms=4,
            label=label, alpha=alpha)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host-input', action='append', required=True,
                    help='LABEL=TICC_CSV[,FREERUN_JSON].  Repeat for each host.')
    ap.add_argument('--gnss-from', default=None,
                    help='Host label whose chB samples represent GNSS PPS '
                         '(plotted once if the chB streams are similar).  '
                         "Special value 'all' plots every host's chB "
                         "separately — useful when receivers differ "
                         '(F9T vs F10T) and produce different PPS noise.  '
                         "Default 'all' since the diagnostic value of "
                         'comparing receivers usually outweighs plot clutter.')
    ap.add_argument('--skip-before', default=None,
                    help='ISO 8601 UTC timestamp; drop earlier rows.')
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--title', default='Clock-pair stability overlay')
    args = ap.parse_args()

    skip_dt = None
    if args.skip_before:
        skip_dt = datetime.fromisoformat(
            args.skip_before.replace('Z', '+00:00'))

    hosts: list[tuple[str, Path, Path | None]] = []
    for spec in args.host_input:
        hosts.append(_parse_host_input(spec))

    print(f'{"host":<14s} {"chA n":>8s} {"chB n":>8s} '
          f'{"chA tdev(1s) ns":>18s} {"chB tdev(1s) ns":>18s}',
          file=sys.stderr)
    print('-' * 72, file=sys.stderr)

    per_host: dict[str, dict] = {}
    for label, ticc_path, freerun_path in hosts:
        chA = load_chA_phase(ticc_path, skip_dt, channel='chA')
        chB = load_chA_phase(ticc_path, skip_dt, channel='chB')
        adev_d, tdev_d = adev_tdev_from_phase(chA)
        adev_g, tdev_g = adev_tdev_from_phase(chB)
        adev_f, tdev_f = load_freerun_char(freerun_path)
        per_host[label] = {
            'disciplined': (adev_d, tdev_d),
            'gnss':        (adev_g, tdev_g),
            'freerun':     (adev_f, tdev_f),
        }
        print(f'{label:<14s} {len(chA):>8d} {len(chB):>8d} '
              f'{tdev_d.get(1.0, float("nan")):>18.2f} '
              f'{tdev_g.get(1.0, float("nan")):>18.2f}',
              file=sys.stderr)

    if args.gnss_from is None or args.gnss_from == 'all':
        gnss_labels = [h[0] for h in hosts]
    else:
        if args.gnss_from not in per_host:
            ap.error(f'--gnss-from {args.gnss_from!r} not in --host-input labels')
        gnss_labels = [args.gnss_from]

    fig, (ax_tdev, ax_adev) = plt.subplots(1, 2, figsize=(13, 6))
    for i, (label, _, freerun_path) in enumerate(hosts):
        color = _HOST_COLORS[i % len(_HOST_COLORS)]
        h = per_host[label]
        # Disciplined chA — solid
        _plot_map(ax_tdev, _TAUS, h['disciplined'][1], color, '-',
                  f'{label} disciplined', lw=2.0, marker='o')
        _plot_map(ax_adev, _TAUS, h['disciplined'][0], color, '-',
                  f'{label} disciplined', lw=2.0, marker='o')
        # Freerun chA — dashed, lighter
        if freerun_path:
            _plot_map(ax_tdev, _TAUS, h['freerun'][1], color, '--',
                      f'{label} freerun', lw=1.4, marker='s', alpha=0.7)
            _plot_map(ax_adev, _TAUS, h['freerun'][0], color, '--',
                      f'{label} freerun', lw=1.4, marker='s', alpha=0.7)

    # GNSS PPS — one dotted curve per host in gnss_labels.  Each host
    # uses its own colour (matching the disciplined curve) so the
    # reader can correlate "this host's GNSS source" with "this host's
    # disciplined output" at a glance.
    for gnss_label in gnss_labels:
        i = next(j for j, (lbl, _, _) in enumerate(hosts) if lbl == gnss_label)
        color = _HOST_COLORS[i % len(_HOST_COLORS)]
        g = per_host[gnss_label]['gnss']
        _plot_map(ax_tdev, _TAUS, g[1], color, ':',
                  f'GNSS PPS ({gnss_label} chB)', lw=1.4, marker='^',
                  alpha=0.85)
        _plot_map(ax_adev, _TAUS, g[0], color, ':',
                  f'GNSS PPS ({gnss_label} chB)', lw=1.4, marker='^',
                  alpha=0.85)

    for ax, ylabel, title, legend_loc in [
            (ax_tdev, 'TDEV (ns)', 'TDEV(τ)', 'lower right'),
            (ax_adev, 'ADEV (dimensionless)', 'ADEV(τ)', 'upper right')]:
        ax.set_xscale('log')
        ax.set_yscale('log')
        ax.set_xlabel('τ (s)')
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True, which='both', ls=':', alpha=0.4)
        ax.legend(loc=legend_loc, fontsize=8)

    # Reference horizontals on the TDEV plot.
    if ax_tdev.get_ylim()[0] < 0.35 < ax_tdev.get_ylim()[1]:
        ax_tdev.axhline(0.35, color='darkgreen', ls=':', alpha=0.55, lw=1.0)
        ax_tdev.text(_TAUS[0], 0.36, ' moonshot per-clock budget (350 ps)',
                     fontsize=8, color='darkgreen', va='bottom')
    ax_tdev.axhline(0.060, color='red', ls=':', alpha=0.55, lw=1.0)
    ax_tdev.text(_TAUS[0], 0.062, ' TICC single-shot resolution (60 ps)',
                 fontsize=8, color='red', va='bottom')

    fig.suptitle(args.title, fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(args.output, dpi=130)
    print(f'\nWrote {args.output}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())

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

# Provenance stamp (analysis versioning + git-hash footer).
sys.path.insert(0, str(Path(__file__).resolve().parent))
from analysis_provenance import (stamp, provenance_line,  # noqa: E402
                                 skip_comment_lines)

_TOOLNAME = 'plot_clock_stability_stack.py'

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
        reader = csv.DictReader(skip_comment_lines(f))
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
    """Pull adev/tdev maps from a do_freerun_char.py JSON (chA-vs-Rb).

    Raises ``FileNotFoundError`` if a path was supplied but the file
    doesn't exist — the previous silent-skip behaviour led to plots
    missing a host's freerun curve without any user-visible signal
    (caught 2026-05-31 when clkPoC3's JSON wasn't pulled to dev box).
    A caller that wants disciplined-only (no freerun curve) should
    omit the JSON from the --host-input spec entirely.
    """
    if not path:
        return None, None
    if not path.exists():
        raise FileNotFoundError(
            f'Freerun JSON not found: {path}.  Pass an existing path '
            f'or omit the JSON from --host-input to plot disciplined-only.')
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
                      ) -> tuple[str, Path, Path | None, Path | None]:
    """``LABEL=TICC_CSV[,FREERUN_JSON[,ARM_STATE_CSV]]`` →
    (label, ticc_path, freerun_path, arm_state_path).

    arm-state path is optional and is used to compute the TDCP-derived
    frequency-domain stability overlay (when ``arm_tdcp_used == 1``
    rows exist).  If absent, the tool tries to auto-infer it from the
    ticc-csv path by replacing ``-ticc.csv`` with ``-arm-state.csv``.
    TDCP overlay is silently skipped when no data is present (engine
    must run with ``--servo-input tdcp`` to populate ``tdcp_freq_ppb``).
    """
    if '=' not in spec:
        raise ValueError(f'expected LABEL=PATH..., got {spec!r}')
    label, rest = spec.split('=', 1)
    parts = [s.strip() for s in rest.split(',')]
    ticc = Path(parts[0])
    freerun = Path(parts[1]) if len(parts) > 1 and parts[1] else None
    arm_state = Path(parts[2]) if len(parts) > 2 and parts[2] else None
    # Auto-infer arm-state path: foo-ticc.csv → foo-arm-state.csv
    if arm_state is None:
        guess = Path(str(ticc).replace('-ticc.csv', '-arm-state.csv'))
        if guess != ticc and guess.exists():
            arm_state = guess
    return label.strip(), ticc, freerun, arm_state


def load_tdcp_freq_ppb(arm_state_path: Path | None,
                       skip_before: datetime | None = None
                       ) -> np.ndarray:
    """Extract per-epoch TDCP frequency observations (ppb) from arm-state.csv.

    TDCP (Time-Differenced Carrier Phase) is the cleanest GNSS-side
    frequency reference — reaches ~15 ps TDEV(1 s) in prototype
    captures, well below the ~2-5 ns of raw chB GNSS-PPS.  Engine
    column ``tdcp_freq_ppb`` is the per-epoch Arm 5 observation,
    populated only when ``arm_tdcp_used == 1`` (i.e. when the engine
    was started with ``--servo-input tdcp`` and the per-epoch gate
    admitted the observation).

    Returns the freq series in ppb (fractional frequency × 1e9).  Empty
    array if the file is missing or no rows have valid TDCP.
    """
    if arm_state_path is None or not arm_state_path.exists():
        return np.array([])
    samples: list[float] = []
    with open(arm_state_path) as f:
        reader = csv.DictReader(skip_comment_lines(f))
        for row in reader:
            if row.get('arm_tdcp_used') != '1':
                continue
            if skip_before is not None:
                if _read_ts(row, 'host_timestamp') < skip_before:
                    continue
            v = row.get('tdcp_freq_ppb', '').strip()
            if not v:
                continue
            try:
                samples.append(float(v))
            except ValueError:
                continue
    return np.array(samples, dtype=np.float64)


def adev_tdev_from_freq_ppb(freq_ppb: np.ndarray
                            ) -> tuple[dict, dict]:
    """ADEV/TDEV from a fractional-frequency series given in ppb.

    Allantools wants fractional frequency (dimensionless) so ppb is
    scaled by 1e-9.  Output ADEV is dimensionless; TDEV is converted
    from seconds to ns for plotting parity with the phase-domain
    curves.
    """
    if len(freq_ppb) < 60:
        return {}, {}
    fractional = freq_ppb * 1e-9
    feasible = _TAUS[_TAUS * 3 < len(fractional)]
    if not len(feasible):
        return {}, {}
    taus_a, adev, _, _ = allantools.adev(fractional, rate=1.0,
                                         taus=feasible, data_type='freq')
    taus_t, tdev, _, _ = allantools.tdev(fractional, rate=1.0,
                                         taus=feasible, data_type='freq')
    return ({float(t): float(a) for t, a in zip(taus_a, adev)},
            {float(t): float(td * 1e9) for t, td in zip(taus_t, tdev)})


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

    print(provenance_line(_TOOLNAME), file=sys.stderr)

    skip_dt = None
    if args.skip_before:
        skip_dt = datetime.fromisoformat(
            args.skip_before.replace('Z', '+00:00'))

    hosts: list[tuple[str, Path, Path | None, Path | None]] = []
    for spec in args.host_input:
        hosts.append(_parse_host_input(spec))

    print(f'{"host":<14s} {"chA n":>8s} {"chB n":>8s} {"tdcp n":>8s} '
          f'{"chA tdev(1s) (ns)":>18s} {"chB tdev(1s) (ns)":>18s} '
          f'{"tdcp tdev(1s) (ns)":>19s}',
          file=sys.stderr)
    print('-' * 96, file=sys.stderr)

    per_host: dict[str, dict] = {}
    for label, ticc_path, freerun_path, arm_state_path in hosts:
        chA = load_chA_phase(ticc_path, skip_dt, channel='chA')
        chB = load_chA_phase(ticc_path, skip_dt, channel='chB')
        adev_d, tdev_d = adev_tdev_from_phase(chA)
        adev_g, tdev_g = adev_tdev_from_phase(chB)
        adev_f, tdev_f = load_freerun_char(freerun_path)
        tdcp_freq = load_tdcp_freq_ppb(arm_state_path, skip_dt)
        adev_t, tdev_t = adev_tdev_from_freq_ppb(tdcp_freq)
        per_host[label] = {
            'disciplined': (adev_d, tdev_d),
            'gnss':        (adev_g, tdev_g),
            'freerun':     (adev_f, tdev_f),
            'tdcp':        (adev_t, tdev_t),
        }
        print(f'{label:<14s} {len(chA):>8d} {len(chB):>8d} {len(tdcp_freq):>8d} '
              f'{tdev_d.get(1.0, float("nan")):>18.3f} '
              f'{tdev_g.get(1.0, float("nan")):>18.3f} '
              f'{tdev_t.get(1.0, float("nan")):>19.3f}',
              file=sys.stderr)

    if args.gnss_from is None or args.gnss_from == 'all':
        gnss_labels = [h[0] for h in hosts]
    else:
        if args.gnss_from not in per_host:
            ap.error(f'--gnss-from {args.gnss_from!r} not in --host-input labels')
        gnss_labels = [args.gnss_from]
    # Convenience map for the GNSS overlay loop below (avoids unpacking
    # the now-4-tuple hosts list and forgetting an element).
    host_index = {h[0]: i for i, h in enumerate(hosts)}

    fig, (ax_tdev, ax_adev) = plt.subplots(1, 2, figsize=(13, 6))
    for i, (label, _, freerun_path, arm_state_path) in enumerate(hosts):
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
        # TDCP — dash-dot, lighter still.  Only plotted if data exists.
        if h['tdcp'][0]:
            _plot_map(ax_tdev, _TAUS, h['tdcp'][1], color, '-.',
                      f'{label} TDCP', lw=1.4, marker='D', alpha=0.7)
            _plot_map(ax_adev, _TAUS, h['tdcp'][0], color, '-.',
                      f'{label} TDCP', lw=1.4, marker='D', alpha=0.7)

    # GNSS PPS — one dotted curve per host in gnss_labels.  Each host
    # uses its own colour (matching the disciplined curve) so the
    # reader can correlate "this host's GNSS source" with "this host's
    # disciplined output" at a glance.
    for gnss_label in gnss_labels:
        i = host_index[gnss_label]
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
    stamp(fig, _TOOLNAME)
    fig.savefig(args.output, dpi=130)
    print(f'\nWrote {args.output}', file=sys.stderr)
    return 0


def render_pair_stability(ticc_csv: Path, label_a: str, label_b: str,
                          output: Path,
                          skip_before: datetime | None = None,
                          title: str | None = None) -> dict:
    """Head-to-head TDEV/ADEV overlay for a two-channel TICC capture.

    chA = clock A, chB = clock B on the SAME shared-reference TICC (the
    ref-immune differential measurement).  Reuses ``load_chA_phase`` and
    ``adev_tdev_from_phase`` — the SAME stability math the per-host stack
    uses — so compare_clocks.py stays consistent with the campaign tools.

    Returns ``{label: {'tdev': {τ: ns}, 'adev': {τ: dimensionless}}}``.
    """
    chA = load_chA_phase(ticc_csv, skip_before, channel='chA')
    chB = load_chA_phase(ticc_csv, skip_before, channel='chB')
    adev_a, tdev_a = adev_tdev_from_phase(chA)
    adev_b, tdev_b = adev_tdev_from_phase(chB)

    print(f'{"clock":<24s} {"n":>8s} {"tdev(1s) (ns)":>14s}', file=sys.stderr)
    print('-' * 48, file=sys.stderr)
    print(f'{label_a:<24s} {len(chA):>8d} '
          f'{tdev_a.get(1.0, float("nan")):>14.3f}', file=sys.stderr)
    print(f'{label_b:<24s} {len(chB):>8d} '
          f'{tdev_b.get(1.0, float("nan")):>14.3f}', file=sys.stderr)

    fig, (ax_tdev, ax_adev) = plt.subplots(1, 2, figsize=(13, 6))
    _plot_map(ax_tdev, _TAUS, tdev_a, _HOST_COLORS[0], '-',
              f'{label_a} (chA)', lw=2.0, marker='o')
    _plot_map(ax_tdev, _TAUS, tdev_b, _HOST_COLORS[1], '-',
              f'{label_b} (chB)', lw=2.0, marker='s')
    _plot_map(ax_adev, _TAUS, adev_a, _HOST_COLORS[0], '-',
              f'{label_a} (chA)', lw=2.0, marker='o')
    _plot_map(ax_adev, _TAUS, adev_b, _HOST_COLORS[1], '-',
              f'{label_b} (chB)', lw=2.0, marker='s')

    for ax, ylabel, subtitle, legend_loc in [
            (ax_tdev, 'TDEV (ns)', 'TDEV(τ)', 'lower right'),
            (ax_adev, 'ADEV (dimensionless)', 'ADEV(τ)', 'upper right')]:
        ax.set_xscale('log')
        ax.set_yscale('log')
        ax.set_xlabel('τ (s)')
        ax.set_ylabel(ylabel)
        ax.set_title(subtitle)
        ax.grid(True, which='both', ls=':', alpha=0.4)
        ax.legend(loc=legend_loc, fontsize=8)

    if ax_tdev.get_ylim()[0] < 0.35 < ax_tdev.get_ylim()[1]:
        ax_tdev.axhline(0.35, color='darkgreen', ls=':', alpha=0.55, lw=1.0)
        ax_tdev.text(_TAUS[0], 0.36, ' moonshot per-clock budget (350 ps)',
                     fontsize=8, color='darkgreen', va='bottom')
    ax_tdev.axhline(0.060, color='red', ls=':', alpha=0.55, lw=1.0)
    ax_tdev.text(_TAUS[0], 0.062, ' TICC single-shot resolution (60 ps)',
                 fontsize=8, color='red', va='bottom')

    fig.suptitle(title or f'{label_a} vs {label_b} — stability head-to-head',
                 fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    stamp(fig, _TOOLNAME)
    fig.savefig(output, dpi=130)
    print(f'\nWrote {output}', file=sys.stderr)
    return {label_a: {'tdev': tdev_a, 'adev': adev_a},
            label_b: {'tdev': tdev_b, 'adev': adev_b}}


if __name__ == '__main__':
    sys.exit(main())

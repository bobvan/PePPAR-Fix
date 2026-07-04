#!/usr/bin/env python3
"""two_clock_report.py — one-command 4-page PDF for a two-clock head-to-head.

Given a single shared-reference TICC capture (chA = clock A PPS, chB =
clock B PPS, both timestamped against the TICC's reference oscillator,
e.g. an FE-5680A Rb), produce a 4-page 16:9 landscape PDF:

  Page 1  Mean-removed phase-agreement CDF        P(|Δ − mean| ≤ T)
  Page 2  Phase difference over time              raw + linear-detrended
  Page 3  Individual clock TDEV & ADEV vs ref     chA-vs-Rb, chB-vs-Rb
  Page 4  Three-cornered hat                       chA / chB / Rb individual

This is a REPORT COMPOSER, not new analysis.  Every page's numbers come
from the same provenance-stamped functions the campaign tools already
use, so a figure here is apples-to-apples with compare_clocks.py and the
standalone plot_* tools:

  * page 1  → plot_xhost_agreement_cdf.load_pairs / cdf_xy
  * page 3  → plot_clock_stability_stack.load_chA_phase / adev_tdev_from_phase
  * page 4  → plot_tch.compute_tch

Only page 2 (the phase-difference time series) is drawn here directly —
it is the one view the standalone tools didn't already emit.

The excursion grade (moonshot bounds, docs/two-site-sync-budget.md) is
the shared-antenna |Δ| ≤ 1 ns @ 95 % / separate-antenna |Δ| ≤ 2 ns @ 95 %
read off page 1.

Window trimming (both ends) is applied ONCE by materializing a windowed
CSV and feeding it to every page, so all four pages analyze exactly the
same epochs — no chance of one page including a transient another dropped.

Usage:
    python3 scripts/two_clock_report.py \\
        --from-csv data/gnssdoplus-vs-ota166-20260702-2026-07-02.csv \\
        --label-a 'GNSSDO+AtomiChron' --label-b 'OTA166' \\
        --out report.pdf \\
        --skip-before 2026-07-02T14:47:28Z \\
        --skip-after  2026-07-02T19:50:00Z \\
        --budget-ns 1.0
"""
from __future__ import annotations

import argparse
import csv
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

_SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPT_DIR))
sys.path.insert(0, str(_SCRIPT_DIR.parent / 'tools'))

from analysis_provenance import (stamp, provenance_line,  # noqa: E402
                                 skip_comment_lines)
from plot_xhost_agreement_cdf import load_pairs, cdf_xy  # noqa: E402
from plot_clock_stability_stack import (  # noqa: E402
    load_chA_phase, adev_tdev_from_phase, _plot_map, _TAUS)
from plot_tch import (  # noqa: E402
    compute_tch, _plot_node_curve, _COLORS)

# Reuse the canonical TICC-single-shot ⊕ FE-5680A-Rb measurement floor
# (the "bathtub") from plot_stability_slide — the SAME floor the campaign
# deviation plots draw (plot_tracking_companion) — but WITHOUT inheriting
# that module's top-level rcParams mutation (it sets presentation-sized
# fonts at import time, which would otherwise leak into every page here).
import matplotlib as _mpl  # noqa: E402
_saved_rc = _mpl.rcParams.copy()
from plot_stability_slide import measurement_floor  # noqa: E402
_mpl.rcParams.update(_saved_rc)

_TOOLNAME = 'two_clock_report.py'
_PS_PER_S = 10 ** 12
_FIGSIZE = (12.8, 7.2)   # 16:9 landscape (12.8 / 7.2 = 1.7778)


def _parse_iso(s: str) -> datetime:
    """Parse an ISO-8601 UTC timestamp to an offset-AWARE datetime.

    A bare ``2026-07-02T14:47:28`` (which the ``--skip-before/after`` help
    invites) parses naive; the capture's ``ts_iso`` is offset-aware, so
    comparing them would raise. Force UTC when no offset is given.
    """
    dt = datetime.fromisoformat(s.replace('Z', '+00:00'))
    return dt if dt.tzinfo is not None else dt.replace(tzinfo=timezone.utc)


def _window_csv(src: Path, dst: Path, skip_before: datetime | None,
                skip_after: datetime | None) -> int:
    """Copy ``src`` → ``dst`` keeping only data rows whose timestamp is in
    [skip_before, skip_after), preserving header/comment lines.  Returns
    the number of data rows kept.

    Materializing the window ONCE and feeding it to every page guarantees
    all four pages see identical epochs.  The imported page loaders only
    support skip-before; doing the trim here also gives us skip-after
    (cut a late disturbance) for free without touching them.
    """
    kept = 0
    with open(src) as fin, open(dst, 'w') as fout:
        # Detect the timestamp column from the (non-comment) header.
        # Comment lines (capture_version banner) start with '#'.
        header_seen = False
        ts_idx = None
        for line in fin:
            s = line.rstrip('\n')
            if s.startswith('#'):
                fout.write(line)
                continue
            if not header_seen:
                cols = s.split(',')
                for cand in ('ts_iso', 'host_timestamp'):
                    if cand in cols:
                        ts_idx = cols.index(cand)
                        break
                if ts_idx is None and (skip_before or skip_after):
                    print(f'{_TOOLNAME}: WARNING — no ts_iso/host_timestamp '
                          f'column in {src.name}; skip-before/after NOT '
                          f'applied (all rows kept).', file=sys.stderr)
                fout.write(line)
                header_seen = True
                continue
            cols = s.split(',')
            if ts_idx is not None and ts_idx < len(cols):
                try:
                    ts = _parse_iso(cols[ts_idx])
                except ValueError:
                    continue
                if skip_before is not None and ts < skip_before:
                    continue
                if skip_after is not None and ts >= skip_after:
                    continue
            fout.write(line)
            kept += 1
    return kept


def _load_diff_timed(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(t_sec, delta_ns)`` for chA − chB, paired by ref_sec.

    Same subtract-separately integer invariant as the campaign loaders:
    each event's total phase ``sec*PS_PER_S + ps`` is built in Python int
    (exact); the per-second chA − chB difference is taken in int, then
    cast to ns.  The Rb reference cancels in the difference.  Time axis is
    ``ref_sec − ref_sec[0]`` (seconds of host uptime elapsed).
    """
    a: dict[int, int] = {}
    b: dict[int, int] = {}
    with open(path) as f:
        reader = csv.DictReader(skip_comment_lines(f))
        for row in reader:
            ch = row['channel']
            if ch not in ('chA', 'chB'):
                continue
            s = int(row['ref_sec'])
            total = s * _PS_PER_S + int(row['ref_ps'])   # Python int — exact
            (a if ch == 'chA' else b)[s] = total
    common = sorted(set(a) & set(b))
    if not common:
        return np.array([]), np.array([])
    secs = np.array(common, dtype=np.float64)
    delta_ps = np.array([a[s] - b[s] for s in common], dtype=np.int64)
    return secs - secs[0], delta_ps.astype(np.float64) * 1e-3


def _finish(pdf: PdfPages, fig) -> None:
    """Stamp provenance and append the page, then free the figure."""
    stamp(fig, _TOOLNAME)
    pdf.savefig(fig)
    plt.close(fig)


def _add_measurement_floor(ax, kind: str) -> None:
    """Shade the TICC single-shot (white PM) ⊕ FE-5680A Rb measurement
    floor — the 'bathtub' — behind the data, matching the campaign
    deviation plots (tools/plot_tracking_companion).  ``kind`` is
    'tdev' (ns) or 'adev' (dimensionless).

    Below this curve a device-under-test cannot be distinguished from the
    measurement chain (TICC 60 ps resolution + FE-5680A reference noise):
    on the chA-vs-Rb / chB-vs-Rb page this marks where an individual
    curve is measurement-limited (and why the TCH page is needed to
    recover stability past it).  Call AFTER the data is plotted and the
    axis is set log-log, so the shade fills from the data-driven ymin.
    """
    ymin, ymax = ax.get_ylim()
    tau_grid = np.logspace(0, 4, 200)
    floor = measurement_floor(tau_grid, kind)
    ax.fill_between(tau_grid, ymin, np.clip(floor, ymin, ymax),
                    color='0.82', alpha=0.6, zorder=0, lw=0)
    ax.plot(tau_grid, floor, color='0.5', lw=1.0, zorder=1,
            label='TICC ⊕ FE-5680A Rb floor')
    ax.set_ylim(ymin, ymax)


# --------------------------------------------------------------------------
# Page 1 — mean-removed phase-agreement CDF
# --------------------------------------------------------------------------
def page_cdf(pdf: PdfPages, path: Path, label_a: str, label_b: str,
             budget_ns: float) -> dict:
    t, delta = load_pairs(path)          # nearest-neighbour pairing, exact int
    n = len(delta)
    fig, ax = plt.subplots(figsize=_FIGSIZE)
    stats: dict = {'n': n}
    if n >= 60:
        d = delta.astype(np.float64)
        resid = np.abs(d - d.mean())     # MEAN-removed (constant offset only)
        sx, ypct = cdf_xy(resid)
        p50, p95, p99 = (float(np.percentile(resid, q)) for q in (50, 95, 99))
        stats.update(p50=p50, p95=p95, p99=p99, max=float(sx[-1]))
        grade = 'PASS' if p95 <= budget_ns * 1000.0 else 'FAIL'
        stats['grade'] = grade
        ax.plot(sx, ypct, lw=1.9, color='#1f77b4', label=f'|Δ − mean|  (n={n})')
        ax.set_xscale('log')
        ax.set_xlim(60.0, max(sx[-1] * 1.2, 2200.0))
        ax.set_ylim(0, 100)
        # Budget / resolution reference verticals.
        for x, c, txt in ((60, 'red', 'TICC res\n(60 ps)'),
                          (1000, 'darkgreen', 'shared-ant\n(1 ns)'),
                          (2000, 'blue', 'separate-ant\n(2 ns)')):
            if ax.get_xlim()[0] <= x <= ax.get_xlim()[1]:
                ax.axvline(x, color=c, ls=':', alpha=0.65, lw=1.0)
                ax.text(x, 1.5, f' {txt}', fontsize=8, color=c, va='bottom')
        for q in (50, 95, 99):
            ax.axhline(q, color='gray', ls=':', alpha=0.55, lw=0.8)
            ax.text(62, q + 0.7, f'{q}%', fontsize=8, color='gray')
        ax.text(0.015, 0.97,
                f'p50 {p50:.0f} ps\np95 {p95:.0f} ps\np99 {p99:.0f} ps\n'
                f'budget {budget_ns:g} ns → {grade}',
                transform=ax.transAxes, va='top', fontsize=10,
                bbox=dict(boxstyle='round', fc='white', alpha=0.85))
        ax.legend(loc='lower right', fontsize=9)
    else:
        ax.text(0.5, 0.5, f'insufficient paired samples (n={n})',
                ha='center', va='center', transform=ax.transAxes)
    ax.set_xlabel('|Δ − mean|  (ps) — log scale')
    ax.set_ylabel('Cumulative % of paired epochs   P(|Δ − mean| ≤ T)')
    ax.set_title(f'{label_a} − {label_b}   ·   mean-removed phase-agreement CDF',
                 fontsize=12)
    ax.grid(True, which='both', ls=':', alpha=0.4)
    _finish(pdf, fig)
    return stats


# --------------------------------------------------------------------------
# Page 2 — phase difference over time (raw + detrended)
# --------------------------------------------------------------------------
def page_timeseries(pdf: PdfPages, path: Path, label_a: str,
                    label_b: str) -> dict:
    t_sec, d_ns = _load_diff_timed(path)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=_FIGSIZE, sharex=True)
    stats: dict = {'n': len(d_ns)}
    if len(d_ns) >= 2:
        mins = t_sec / 60.0
        offset = float(d_ns.mean())
        poly = np.polyfit(t_sec, d_ns, 1)          # linear detrend basis
        trend = np.polyval(poly, t_sec)
        resid = d_ns - trend
        std_dt = float(resid.std())
        p95 = float(np.percentile(np.abs(d_ns - np.median(d_ns)), 95))
        stats.update(offset_ns=offset, drift_ps_s=float(poly[0] * 1000.0),
                     detrended_std_ns=std_dt, p95_ns=p95)

        ax1.plot(mins, d_ns, lw=0.5, color='#1f77b4')
        ax1.plot(mins, trend, 'r--', lw=1.0,
                 label=f'linear drift {poly[0]*1000:.3f} ps/s')
        ax1.axhline(offset, color='0.6', lw=0.8, ls=':')
        ax1.set_ylabel(f'{label_a} − {label_b}  (ns)')
        ax1.legend(loc='upper right', fontsize=9)
        ax1.grid(alpha=0.3)
        ax1.text(0.008, 0.96,
                 f'offset {offset:+.2f} ns\npk-pk {np.ptp(d_ns):.2f} ns\n'
                 f'p95 |Δ−med| {p95:.2f} ns',
                 transform=ax1.transAxes, va='top', fontsize=9,
                 bbox=dict(boxstyle='round', fc='white', alpha=0.85))

        ax2.plot(mins, resid, lw=0.5, color='#2ca02c')
        ax2.axhline(0, color='0.6', lw=0.8, ls=':')
        ax2.set_ylabel('detrended (ns)')
        ax2.text(0.008, 0.96, f'detrended std {std_dt:.3f} ns',
                 transform=ax2.transAxes, va='top', fontsize=9,
                 bbox=dict(boxstyle='round', fc='white', alpha=0.85))
    else:
        ax1.text(0.5, 0.5, f'insufficient paired samples (n={len(d_ns)})',
                 ha='center', va='center', transform=ax1.transAxes)
    ax2.set_xlabel('minutes into (windowed) run')
    ax1.set_title(f'{label_a} − {label_b}   ·   phase difference over time '
                  f'(raw, top; linear-detrended, bottom)', fontsize=12)
    _finish(pdf, fig)
    return stats


# --------------------------------------------------------------------------
# Page 3 — individual clock TDEV & ADEV vs the reference
# --------------------------------------------------------------------------
def page_stability(pdf: PdfPages, path: Path, label_a: str, label_b: str,
                   ref_label: str) -> dict:
    chA = load_chA_phase(path, None, channel='chA')     # chA vs Rb, detrended
    chB = load_chA_phase(path, None, channel='chB')     # chB vs Rb, detrended
    adev_a, tdev_a = adev_tdev_from_phase(chA)
    adev_b, tdev_b = adev_tdev_from_phase(chB)

    fig, (ax_t, ax_a) = plt.subplots(1, 2, figsize=_FIGSIZE)
    _plot_map(ax_t, _TAUS, tdev_a, _COLORS['chA'], '-', f'{label_a} (chA)',
              marker='o')
    _plot_map(ax_t, _TAUS, tdev_b, _COLORS['chB'], '-', f'{label_b} (chB)',
              marker='s')
    _plot_map(ax_a, _TAUS, adev_a, _COLORS['chA'], '-', f'{label_a} (chA)',
              marker='o')
    _plot_map(ax_a, _TAUS, adev_b, _COLORS['chB'], '-', f'{label_b} (chB)',
              marker='s')
    for ax, ylabel, sub, loc, kind in (
            (ax_t, 'TDEV (ns)', 'TDEV(τ)', 'lower right', 'tdev'),
            (ax_a, 'ADEV (dimensionless)', 'ADEV(τ)', 'upper right', 'adev')):
        ax.set_xscale('log')
        ax.set_yscale('log')
        ax.set_xlabel('τ (s)')
        ax.set_ylabel(ylabel)
        ax.set_title(sub)
        ax.grid(True, which='both', ls=':', alpha=0.4)
        _add_measurement_floor(ax, kind)   # TICC ⊕ Rb bathtub, behind data
        ax.legend(loc=loc, fontsize=8)
    if ax_t.get_ylim()[0] < 0.35 < ax_t.get_ylim()[1]:
        ax_t.axhline(0.35, color='darkgreen', ls=':', alpha=0.55, lw=1.0)
        ax_t.text(_TAUS[0], 0.36, ' moonshot per-clock budget (350 ps)',
                  fontsize=8, color='darkgreen', va='bottom')
    fig.suptitle(f'{label_a} & {label_b}   ·   individual stability vs '
                 f'{ref_label} reference   (chA n={len(chA)}, chB n={len(chB)})',
                 fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    _finish(pdf, fig)
    return {label_a: {'tdev': tdev_a, 'adev': adev_a},
            label_b: {'tdev': tdev_b, 'adev': adev_b}}


# --------------------------------------------------------------------------
# Page 4 — three-cornered hat
# --------------------------------------------------------------------------
def page_tch(pdf: PdfPages, path: Path, label_a: str, label_b: str,
             ref_label: str, groslambert: bool) -> dict:
    res = compute_tch(path, None, groslambert)
    fig, (ax_t, ax_a) = plt.subplots(1, 2, figsize=_FIGSIZE)
    taus = res.get('taus', np.array([]))
    if len(taus):
        tdev, adev = res['tdev_ns'], res['adev']
        node_label = {'chA': f'{label_a} (chA)', 'chB': f'{label_b} (chB)',
                      'Rb': f'{ref_label} (reference)'}
        markers = {'chA': 'o', 'chB': 's', 'Rb': '^'}
        for ax, ymap in ((ax_t, tdev), (ax_a, adev)):
            for key in ('chA', 'chB', 'Rb'):
                _plot_node_curve(ax, taus, ymap.get(key, np.array([])),
                                 _COLORS[key], node_label[key], markers[key])
        if ax_t.get_ylim()[0] < 0.35 < ax_t.get_ylim()[1]:
            ax_t.axhline(0.35, color='darkgreen', ls=':', alpha=0.55, lw=1.0)
            ax_t.text(taus[0], 0.36, ' moonshot per-clock budget (350 ps)',
                      fontsize=8, color='darkgreen', va='bottom')
    else:
        for ax in (ax_t, ax_a):
            ax.text(0.5, 0.5, f'insufficient aligned data (n={res.get("n", 0)})',
                    ha='center', va='center', transform=ax.transAxes)
    for ax, ylabel, sub, kind in (
            (ax_t, 'TDEV (ns)', 'TDEV(τ) — individual nodes', 'tdev'),
            (ax_a, 'ADEV (dimensionless)', 'ADEV(τ) — individual nodes', 'adev')):
        ax.set_xscale('log')
        ax.set_yscale('log')
        ax.set_xlabel('τ (s)')
        ax.set_ylabel(ylabel)
        ax.set_title(sub)
        ax.grid(True, which='both', ls=':', alpha=0.4)
        if len(taus):
            _add_measurement_floor(ax, kind)   # TICC ⊕ Rb bathtub, behind nodes
        ax.legend(loc='best', fontsize=8)
    est = res.get('estimator', 'closed-form')
    neg = res.get('neg_count', 0)
    fig.suptitle(f'Three-cornered hat: {label_a} / {label_b} / {ref_label}   '
                 f'({est}; {neg} neg-var τ guarded→NaN)', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    _finish(pdf, fig)
    return res


def build_report(csv_path: Path, out_pdf: Path, label_a: str, label_b: str,
                 ref_label: str = 'Rb', budget_ns: float = 1.0,
                 skip_before: datetime | None = None,
                 skip_after: datetime | None = None,
                 groslambert: bool = False,
                 work_dir: Path | None = None) -> dict:
    """Materialize the analysis window (once) and write the 4-page PDF.

    Returns a summary dict {cdf, timeseries, stability, tch}.  Reusable
    from tests / other tools without going through argparse.
    """
    out_pdf.parent.mkdir(parents=True, exist_ok=True)
    work_path = csv_path
    if skip_before is not None or skip_after is not None:
        wd = work_dir or out_pdf.parent
        wd.mkdir(parents=True, exist_ok=True)
        work_path = wd / (out_pdf.stem + '.windowed.csv')
        kept = _window_csv(csv_path, work_path, skip_before, skip_after)
        print(f'{_TOOLNAME}: windowed {kept} data rows → {work_path}',
              file=sys.stderr)

    summary: dict = {}
    with PdfPages(out_pdf) as pdf:
        summary['cdf'] = page_cdf(pdf, work_path, label_a, label_b, budget_ns)
        summary['timeseries'] = page_timeseries(pdf, work_path, label_a, label_b)
        summary['stability'] = page_stability(pdf, work_path, label_a, label_b,
                                              ref_label)
        summary['tch'] = page_tch(pdf, work_path, label_a, label_b, ref_label,
                                  groslambert)
        d = pdf.infodict()
        d['Title'] = f'{label_a} vs {label_b} — two-clock report'
        d['Creator'] = _TOOLNAME
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--from-csv', required=True, type=Path,
                    help='Two-channel (chA+chB) shared-reference TICC CSV.')
    ap.add_argument('--out', required=True, type=Path,
                    help='Output PDF path (4 pages, 16:9 landscape).')
    ap.add_argument('--label-a', default='clock A', help='Label for chA.')
    ap.add_argument('--label-b', default='clock B', help='Label for chB.')
    ap.add_argument('--ref-label', default='Rb',
                    help="Label for the TICC reference node (default 'Rb').")
    ap.add_argument('--budget-ns', type=float, default=1.0,
                    help='p95 excursion budget in ns for the page-1 grade '
                         '(1.0 shared-antenna, 2.0 separate-antenna).')
    ap.add_argument('--skip-before', default=None,
                    help='ISO-8601 UTC; drop earlier rows (bootstrap trim).')
    ap.add_argument('--skip-after', default=None,
                    help='ISO-8601 UTC; drop later rows (cut a late '
                         'disturbance, e.g. after a step event).')
    ap.add_argument('--groslambert', action='store_true',
                    help='TCH page: use the Groslambert-covariance estimator.')
    args = ap.parse_args()

    if not args.from_csv.exists():
        ap.error(f'--from-csv path does not exist: {args.from_csv}')
    sb = _parse_iso(args.skip_before) if args.skip_before else None
    sa = _parse_iso(args.skip_after) if args.skip_after else None

    print(provenance_line(_TOOLNAME), file=sys.stderr)
    summary = build_report(args.from_csv, args.out, args.label_a, args.label_b,
                           ref_label=args.ref_label, budget_ns=args.budget_ns,
                           skip_before=sb, skip_after=sa,
                           groslambert=args.groslambert)

    cdf = summary['cdf']
    ts = summary['timeseries']
    if 'p95' in cdf:
        print(f'VERDICT: p95 |Δ−mean| = {cdf["p95"]/1000:.3f} ns vs '
              f'{args.budget_ns:g} ns → {cdf["grade"]}  '
              f'(n={cdf["n"]}, detrended std {ts.get("detrended_std_ns", float("nan")):.3f} ns)')
    print(f'{_TOOLNAME}: wrote {args.out}  (4 pages)', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())

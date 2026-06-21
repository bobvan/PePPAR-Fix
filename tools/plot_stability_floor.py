#!/usr/bin/env python3
"""Stability-floor overlay — per-clock TDEV with χ² bands + Rb/TICC floor.

A single log-log TDEV-vs-τ figure (ALL IN NANOSECONDS) for a two-channel
shared-reference TICC capture (chA, chB vs the TICC's shared Rb).  Unlike
the head-to-head stack, this figure is built so a reader can draw a
conclusion WITHOUT ambiguity about where the measurement floor sits and
how trustworthy each TDEV point is:

  1. chA TDEV and chB TDEV (per-clock, detrended) — each a LINE with a
     68% χ² CONFIDENCE BAND (shaded).  The band widens at long τ where
     few independent windows remain — that widening is the point: it
     tells the reader not to over-read a long-τ bump.

  2. Rb + TICC measurement floor, two clearly labelled components:
       * TICC instrument floor — a flat reference at ~60 ps
         (docs/ticc-baseline-2026-04-01.md / ticc-calibration-2026-03-19.md
         report ~52–60 ps single-shot resolution).  Labelled
         "TICC floor ~60 ps".
       * Rb floor estimate — the three-cornered-hat (TCH) Rb-node TDEV
         for THIS dataset (tools/plot_tch solver), dashed, labelled
         "Rb (TCH estimate)".  The TCH split is ILL-CONDITIONED when the
         two clocks are very disparate (negative-variance → NaN); those
         τ points are left as GAPS and the caveat is carried in the
         title/legend.

  3. Budget reference lines: 350 ps (per-clock moonshot budget,
     docs/two-site-sync-budget.md) and 60 ps (TICC floor).

The title carries the analysis-provenance stamp + dataset + "TDEV (ns)".

This tool is ADDITIVE VISUALIZATION over the existing analysis: it adds
confidence bands and a floor overlay but does NOT change any TDEV / CDF /
TCH number.  It reuses the existing loaders and solvers verbatim:

  * plot_clock_stability_stack.load_chA_phase  (subtract-separately int
    ps precision + longest-gap-free segment + linear detrend)
  * plot_clock_stability_stack.adev_tdev_from_phase  (the TDEV math)
  * plot_tch.compute_tch  (the Rb-node TCH estimate)

so it cannot drift from the campaign metrics.  ANALYSIS_VERSION is NOT
bumped — no analysis math changes here.

Usage:
    python3 tools/plot_stability_floor.py \\
        --from-csv data/otcbob1-vs-m600-2026-06-21.csv \\
        --label-a otcBob1 --label-b M600 \\
        --output /tmp/stability_floor.png \\
        --dataset 'otcbob1-vs-m600 2026-06-21'
"""
from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')   # headless: lab + CI safe
import matplotlib.pyplot as plt
from scipy.stats import chi2

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analysis_provenance import (stamp, provenance_line,  # noqa: E402
                                 analysis_provenance)
from plot_clock_stability_stack import (load_chA_phase,  # noqa: E402
                                        adev_tdev_from_phase)
from plot_tch import compute_tch  # noqa: E402

_TOOLNAME = 'plot_stability_floor.py'

# TICC single-shot measurement floor.  ~52–60 ps per
# docs/ticc-baseline-2026-04-01.md and docs/ticc-calibration-2026-03-19.md;
# we draw the round 60 ps reference (the conservative end).
_TICC_FLOOR_NS = 0.060
# Per-clock moonshot budget (docs/two-site-sync-budget.md).
_BUDGET_NS = 0.350
_RATE = 1.0

_COLOR_A = '#1f77b4'
_COLOR_B = '#ff7f0e'
_COLOR_RB = '#2ca02c'


def chi2_tdev_ci(tdev: np.ndarray, edf: np.ndarray, ci: float = 0.6827
                 ) -> tuple[np.ndarray, np.ndarray]:
    """χ² confidence interval for a TDEV estimate with ``edf`` equivalent
    degrees of freedom at each τ.  Returns ``(lo, hi)`` arrays.

    Same form as scripts/tdev_threearm_clkpoc3.chi2_ci — kept here as a
    reusable helper so the band math lives in one tested place.  The band
    widens sharply as edf → few (long τ), which is the whole point.
    """
    p = (1.0 - ci) / 2.0
    edf = np.asarray(edf, dtype=np.float64)
    tdev = np.asarray(tdev, dtype=np.float64)
    lo = tdev * np.sqrt(edf / chi2.ppf(1.0 - p, edf))
    hi = tdev * np.sqrt(edf / chi2.ppf(p, edf))
    return lo, hi


def edf_from_taus(taus: np.ndarray, n: int, rate: float = _RATE
                  ) -> np.ndarray:
    """Equivalent degrees of freedom for a TDEV estimate at each τ from the
    INDEPENDENT (non-overlapping) window count of a length-``n`` record.

    A TDEV/MDEV second difference spans 3 averaging factors, so a record
    of N points holds ~N/m − 2 independent looks at a τ = m/rate
    fluctuation (m = τ·rate).  This collapses at long τ — exactly the
    uncertainty the band must surface.  Floored at 1 so χ² stays defined.
    Same estimate as scripts/tdev_threearm_clkpoc3.py.
    """
    taus = np.asarray(taus, dtype=np.float64)
    return np.maximum(1.0, n / (taus * rate) - 2.0)


def tdev_with_band(phase_ns: np.ndarray
                   ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Per-clock TDEV (ns) at the campaign τ grid plus its 68% χ² band.

    Reuses ``adev_tdev_from_phase`` for the TDEV values (so the numbers are
    identical to the head-to-head stack), then attaches a χ² CI computed
    from the independent-window count of this phase record.

    Returns ``(taus, tdev_ns, lo_ns, hi_ns)`` sorted by τ; empty arrays if
    the record is too short to produce any TDEV point.
    """
    _, tdev_map = adev_tdev_from_phase(phase_ns)
    if not tdev_map:
        empty = np.array([])
        return empty, empty, empty, empty
    items = sorted(tdev_map.items())
    taus = np.array([t for t, _ in items], dtype=np.float64)
    tdev = np.array([v for _, v in items], dtype=np.float64)
    edf = edf_from_taus(taus, len(phase_ns))
    lo, hi = chi2_tdev_ci(tdev, edf)
    return taus, tdev, lo, hi


def _plot_with_band(ax, taus, tdev, lo, hi, color, label, marker) -> None:
    """Plot a TDEV line with its shaded χ² band (skip non-positive/NaN)."""
    if not len(taus):
        return
    good = np.isfinite(tdev) & (tdev > 0)
    if not np.any(good):
        return
    ax.fill_between(taus[good], lo[good], hi[good], color=color, alpha=0.18,
                    zorder=1, label=f'{label} 68% χ² band')
    ax.plot(taus[good], tdev[good], color=color, ls='-', lw=2.0, marker=marker,
            ms=4, zorder=3, label=label)


def _rb_tdev_curve(csv_path: Path, skip_before: datetime | None
                   ) -> tuple[np.ndarray, np.ndarray, int]:
    """Rb-node TDEV (ns) from the TCH solver for this dataset.

    Reuses ``plot_tch.compute_tch`` verbatim.  Returns ``(taus, rb_tdev_ns,
    neg_count)``; NaN-guarded τ points are kept as NaN so the caller leaves
    them as GAPS in the line (per the ill-conditioning caveat).
    """
    res = compute_tch(csv_path, skip_before=skip_before)
    taus = res.get('taus', np.array([]))
    rb = res.get('tdev_ns', {}).get('Rb', np.array([]))
    return np.asarray(taus, dtype=np.float64), np.asarray(rb, dtype=np.float64), \
        int(res.get('neg_count', 0))


def render_stability_floor(csv_path: Path, output: Path,
                           label_a: str = 'chA', label_b: str = 'chB',
                           skip_before: datetime | None = None,
                           dataset: str | None = None,
                           title: str | None = None) -> dict:
    """Build the stability-floor overlay figure and write it (stamped).

    Returns a summary dict::

        {
          label_a: {'taus':…, 'tdev':…, 'lo':…, 'hi':…},
          label_b: {…},
          'rb': {'taus':…, 'tdev':…, 'neg_count':int},
          'ticc_floor_ns': 0.060,
          'budget_ns': 0.350,
        }

    All TDEV values are NANOSECONDS.  Does not change any TDEV/TCH number —
    it overlays bands + the floor on the existing math.
    """
    chA = load_chA_phase(csv_path, skip_before, channel='chA')
    chB = load_chA_phase(csv_path, skip_before, channel='chB')
    taus_a, tdev_a, lo_a, hi_a = tdev_with_band(chA)
    taus_b, tdev_b, lo_b, hi_b = tdev_with_band(chB)
    rb_taus, rb_tdev, neg_count = _rb_tdev_curve(csv_path, skip_before)

    def _at_1s(taus, tdev):
        for t, v in zip(taus, tdev):
            if abs(t - 1.0) < 1e-9:
                return float(v)
        return float('nan')

    # stderr summary (units explicit).
    print(provenance_line(_TOOLNAME), file=sys.stderr)
    print(f'{"clock":<24s} {"n":>8s} {"tdev(1s) (ns)":>14s}', file=sys.stderr)
    print('-' * 48, file=sys.stderr)
    print(f'{label_a:<24s} {len(chA):>8d} {_at_1s(taus_a, tdev_a):>14.4f}',
          file=sys.stderr)
    print(f'{label_b:<24s} {len(chB):>8d} {_at_1s(taus_b, tdev_b):>14.4f}',
          file=sys.stderr)
    print(f'Rb (TCH estimate): {np.count_nonzero(np.isfinite(rb_tdev))} finite '
          f'τ points, {neg_count} negative-variance points (NaN gaps)',
          file=sys.stderr)

    fig, ax = plt.subplots(figsize=(10, 7))

    _plot_with_band(ax, taus_a, tdev_a, lo_a, hi_a, _COLOR_A,
                    f'{label_a} (chA)', 'o')
    _plot_with_band(ax, taus_b, tdev_b, lo_b, hi_b, _COLOR_B,
                    f'{label_b} (chB)', 's')

    # Rb floor — TCH estimate, dashed; NaN-guarded points left as gaps.
    rb_good = np.isfinite(rb_tdev) & (rb_tdev > 0)
    if np.any(rb_good):
        ax.plot(rb_taus[rb_good], rb_tdev[rb_good], color=_COLOR_RB, ls='--',
                lw=1.8, marker='^', ms=5, alpha=0.9,
                label='Rb (TCH estimate)')
    else:
        # No usable Rb points at all (fully ill-conditioned) — still note it
        # in the legend via an invisible proxy handle.
        ax.plot([], [], color=_COLOR_RB, ls='--', lw=1.8, marker='^',
                label='Rb (TCH estimate — all NaN, ill-conditioned)')

    # TICC instrument floor — flat reference.
    ax.axhline(_TICC_FLOOR_NS, color='red', ls=':', alpha=0.7, lw=1.2,
               label='TICC floor ~60 ps')
    # Per-clock moonshot budget reference.
    ax.axhline(_BUDGET_NS, color='darkgreen', ls=':', alpha=0.7, lw=1.2,
               label='moonshot per-clock budget (350 ps)')

    ax.set_xscale('log')
    ax.set_yscale('log')
    # Sub-decade τ grid so crossings read cleanly.
    ax.grid(True, which='both', ls=':', alpha=0.4)
    ax.set_xlabel('τ (s)')
    ax.set_ylabel('TDEV (ns)')

    # Title carries provenance + dataset + "TDEV (ns)" + the TCH caveat.
    ds = dataset or csv_path.stem
    caveat = ''
    if neg_count > 0:
        caveat = (f'  ·  Rb-TCH ill-conditioned at {neg_count} τ '
                  f'(disparate clocks → neg-var NaN gaps)')
    default_title = (f'{label_a} vs {label_b} — TDEV (ns) with χ² bands + '
                     f'Rb/TICC floor\n{ds}  ·  analysis '
                     f'{analysis_provenance()}{caveat}')
    ax.set_title(title or default_title, fontsize=10)
    ax.legend(loc='best', fontsize=8)

    fig.tight_layout()
    stamp(fig, _TOOLNAME)
    fig.savefig(output, dpi=130)
    print(f'\nWrote {output}', file=sys.stderr)

    return {
        label_a: {'taus': taus_a, 'tdev': tdev_a, 'lo': lo_a, 'hi': hi_a},
        label_b: {'taus': taus_b, 'tdev': tdev_b, 'lo': lo_b, 'hi': hi_b},
        'rb': {'taus': rb_taus, 'tdev': rb_tdev, 'neg_count': neg_count},
        'ticc_floor_ns': _TICC_FLOOR_NS,
        'budget_ns': _BUDGET_NS,
    }


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--from-csv', required=True, type=Path,
                    help='Two-channel (chA+chB) shared-reference TICC CSV.')
    ap.add_argument('--output', required=True, type=Path,
                    help='Output PNG path.')
    ap.add_argument('--label-a', default='chA',
                    help="Label for chA (e.g. 'otcBob1').")
    ap.add_argument('--label-b', default='chB',
                    help="Label for chB (e.g. 'M600').")
    ap.add_argument('--dataset', default=None,
                    help='Dataset name for the title (default: CSV stem).')
    ap.add_argument('--skip-before', default=None,
                    help='ISO 8601 UTC timestamp; drop earlier rows.')
    ap.add_argument('--title', default=None)
    args = ap.parse_args()

    if not args.from_csv.exists():
        ap.error(f'--from-csv path does not exist: {args.from_csv}')

    skip_dt = None
    if args.skip_before:
        skip_dt = datetime.fromisoformat(args.skip_before.replace('Z', '+00:00'))

    res = render_stability_floor(
        args.from_csv, args.output, args.label_a, args.label_b,
        skip_before=skip_dt, dataset=args.dataset, title=args.title)
    # Success if at least one per-clock TDEV curve was produced.
    ok = any(len(res[k].get('tdev', [])) for k in (args.label_a, args.label_b))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

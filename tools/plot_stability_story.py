#!/usr/bin/env python3
"""The "everything" stability slide — six stories in one 16:9 figure.

Builds on plot_stability_slide.  Per panel (TDEV left, ADEV right) it layers the
six things that bound what PePPAR-Fix can do, audience-first (no jargon):

  1. Measurement noise floor (grey shading) — below this the TICC + FE-5680A Rb
     reference can't resolve anything, so no conclusion is drawable there.
  2. Free-running oscillator (dashed, per clock) — the best discipline could
     ever do at short τ; the servo cannot beat the crystal's own noise.
  3. Raw GNSS PPS (dark, dotted) — the timing information actually available to
     the discipline loop from the receiver's 1 PPS edge.
  4. Theoretical ideal GNSS (purple dashed) — what GPS could give with perfect
     information (white-PM asymptote: τ⁻¹ ADEV, τ⁻¹ᐟ² TDEV).
  5/6. Disciplined DO output (bold solid, per clock) — what PePPAR-Fix actually
     delivers.  The OCXO beats ideal GNSS at short τ; the TCXO only matches the
     ideal slope and cannot correct its own inherent instability.

The shaded per-clock region between the disciplined output and raw GNSS PPS is
the improvement discipline delivers — drawn ONLY where the disciplined output is
actually better than raw PPS, so the big blue (OCXO) region reads as a genuine
win and the near-absent orange (TCXO) region honestly says "no short/mid-τ win;
the TCXO's value is holdover, i.e. the gap down from its own free-run dashed
line at long τ."

Usage:
    python3 tools/plot_stability_story.py \\
        --trace 'PePPAR-Fix on an OCXO (PulsePuppy)=clkpoc3.csv,freerun-ocxo.json' \\
        --trace 'PePPAR-Fix on a TCXO (TimeHAT)=timehat.csv,freerun-tcxo.json' \\
        --skip-before 2026-06-15T03:20:00Z --output peppar-stability-story
"""
from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plot_clock_stability_stack import load_chA_phase, load_freerun_char  # noqa: E402
from plot_stability_slide import (  # noqa: E402  (also applies shared rcParams)
    TRACE_COLORS, GNSS_PPS_COLOR, IDEAL_COLOR,
    make_taus, dev_with_err, measurement_floor, at_tau, loglog_interp,
)


def _plot_map(ax, m: dict, **kw) -> None:
    """Plot a {tau: value} map (sorted) — used for the free-run char curves."""
    if not m:
        return
    xs = np.array(sorted(m))
    ax.plot(xs, np.array([m[x] for x in xs]), **kw)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--trace', action='append', required=True,
                    help="'Display Label=TICC_CSV[,FREERUN_JSON]'.  Repeat per clock.")
    ap.add_argument('--gnss-pps', default=None,
                    help="'Label=TICC_CSV' raw GNSS PPS ref; default first trace chB.")
    ap.add_argument('--skip-before', default=None)
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--title', default='PePPAR-Fix clock stability — the full picture')
    ap.add_argument('--taus', default='dense')
    args = ap.parse_args()

    skip_dt = (datetime.fromisoformat(args.skip_before.replace('Z', '+00:00'))
               if args.skip_before else None)

    clocks = []  # (label, chA_phase, freerun_adev, freerun_tdev)
    for spec in args.trace:
        label, rest = spec.split('=', 1)
        parts = [p.strip() for p in rest.split(',')]
        cha = load_chA_phase(Path(parts[0]), skip_dt, channel='chA')
        if not len(cha):
            ap.error(f'no chA data in {parts[0]}')
        fr_a, fr_t = (load_freerun_char(Path(parts[1]))
                      if len(parts) > 1 and parts[1] else (None, None))
        clocks.append((label.strip(), cha, fr_a or {}, fr_t or {}))
        print(f'{label.strip():42s} chA {len(cha):>7d}  freerun τ '
              f'{sorted((fr_t or {}).keys())}', file=sys.stderr)

    glabel, gpath = ((args.gnss_pps.split('=', 1)[0].strip(),
                      args.gnss_pps.split('=', 1)[1].strip()) if args.gnss_pps
                     else ('Raw GNSS PPS (F9T)', args.trace[0].split('=', 1)[1].split(',')[0].strip()))
    g_phase = load_chA_phase(Path(gpath), skip_dt, channel='chB')

    fig, (ax_t, ax_a) = plt.subplots(1, 2, figsize=(16, 9))
    tau_grid = np.logspace(0, 4, 240)
    n_max = max(len(p) for _, p, _, _ in clocks)

    for ax, kind, ylabel, title, ylim in [
            (ax_t, 'tdev', 'TDEV  (ns)', 'Time stability  —  TDEV(τ)', (5e-3, 5e1)),
            (ax_a, 'adev', 'ADEV  (fractional)', 'Frequency stability  —  ADEV(τ)', (1e-12, 1e-8))]:
        ymin, ymax = ylim
        taus_spec = make_taus(n_max, args.taus)

        # (1) Measurement floor — behind everything.
        floor = measurement_floor(tau_grid, kind)
        ax.fill_between(tau_grid, ymin, np.clip(floor, ymin, ymax),
                        color='0.82', alpha=0.6, zorder=0, lw=0)
        ax.plot(tau_grid, floor, color='0.5', lw=1.0, zorder=1)

        # (3) Raw GNSS PPS + (4) the ideal anchored to it.
        gi = None
        if len(g_phase):
            gt, gd, _ = dev_with_err(g_phase, kind, taus_spec)
            ax.plot(gt, gd, color=GNSS_PPS_COLOR, lw=2.0, ls=':', marker='.',
                    ms=5, alpha=0.9, zorder=3, label=glabel)
            a1 = at_tau(gt, gd, 1.0)
            if a1:
                p = -1.0 if kind == 'adev' else -0.5
                ax.plot(tau_grid, a1 * tau_grid ** p, color=IDEAL_COLOR,
                        lw=1.8, ls='--', zorder=2,
                        label=('Ideal GNSS  (τ⁻¹)' if kind == 'adev'
                               else 'Ideal GNSS  (τ⁻¹ᐟ²)'))
            gi = loglog_interp(gt, gd, tau_grid)

        # Per clock: improvement shading, (5/6) disciplined output, (2) free-run.
        for i, (label, cha, fr_a, fr_t) in enumerate(clocks):
            c = TRACE_COLORS[i % len(TRACE_COLORS)]
            ct, cd, cerr = dev_with_err(cha, kind, taus_spec)

            # Improvement region — only where disciplined actually beats raw PPS.
            if gi is not None:
                ci = loglog_interp(ct, cd, tau_grid)
                win = np.isfinite(ci) & np.isfinite(gi) & (ci < gi)
                if win.any():
                    ax.fill_between(tau_grid[win], ci[win], gi[win], color=c,
                                    alpha=0.15, zorder=1.5, lw=0,
                                    label=('PePPAR-Fix improvement over raw PPS'
                                           if i == 0 else None))

            # (2) Free-running oscillator — the floor discipline can't beat.
            _plot_map(ax, fr_t if kind == 'tdev' else fr_a, color=c, lw=1.8,
                      ls=(0, (6, 2)), alpha=0.75, marker='s', ms=4, zorder=3,
                      label=f'{label.split("(")[0].strip().split()[-1]} free-running')

            # (5/6) Disciplined DO output — the hero.
            ax.fill_between(ct, np.maximum(cd - cerr, cd * 0.3), cd + cerr,
                            color=c, alpha=0.16, zorder=4, lw=0)
            ax.plot(ct, cd, color=c, lw=3.6, zorder=5, solid_capstyle='round',
                    label=label)

        ax.set_xscale('log'); ax.set_yscale('log')
        ax.set_xlim(1, 1e4); ax.set_ylim(ymin, ymax)
        ax.set_box_aspect(1)
        ax.set_xlabel('Averaging time  τ  (s)')
        ax.set_ylabel(ylabel); ax.set_title(title, pad=10)
        ax.grid(True, which='both', ls=':', alpha=0.35)
        ax.text(1.3, ymin * (ymax / ymin) ** 0.025,
                'TICC + FE-5680A Rb measurement floor', fontsize=12,
                color='0.30', style='italic', va='bottom')
        h, lab = ax.get_legend_handles_labels()
        order = sorted(range(len(lab)),
                       key=lambda k: (0 if lab[k].startswith('PePPAR-Fix on') else
                                      1 if 'free-running' in lab[k] else
                                      2 if 'improvement' in lab[k] else 3))
        ax.legend([h[k] for k in order], [lab[k] for k in order],
                  loc='upper right', framealpha=0.93, fontsize=12)

    fig.suptitle(args.title, fontsize=23, fontweight='bold', y=0.97)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    for ext, dpi in (('.png', 200), ('.pdf', None)):
        fig.savefig(str(args.output) + ext, dpi=dpi, facecolor='white')
        print(f'wrote {args.output}{ext}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())

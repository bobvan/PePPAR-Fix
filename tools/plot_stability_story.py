#!/usr/bin/env python3
"""Two-slide stability story — "the win" and "the limits".

Splits the six bounding stories across two audience-first 16:9 slides (no
jargon, no "tracking", no σ-band shading).  Solid = disciplined DO output
("what we did"); colour per clock.

--mode win    — what PePPAR-Fix delivers vs the naive baseline:
  (1) measurement floor (grey)   (3) raw GNSS PPS (dotted)
  (4) ideal GNSS (purple dashed) (5/6) disciplined output (bold solid)
  + solid improvement shading wherever the disciplined output beats raw PPS
    (big blue OCXO win; near-absent orange TCXO = honest "no win over GNSS").

--mode limits — what bounds us and where the room is:
  (1) measurement floor (grey)   (2) free-running oscillator (dashed)
  (4) ideal GNSS (purple dashed) (5/6) disciplined output (bold solid)
  + HATCHED remaining-room region between the disciplined output and the
    free-running oscillator floor (clamped to the measurement floor — you
    can't discipline a DO below its own noise, and can't claim below what the
    reference can resolve).  OCXO shows real mid-τ loop headroom (the
    discipline-noise bump); the TCXO shows ~none — it's oscillator-bound, so
    only a better crystal helps, not loop tuning.

Usage:
    python3 tools/plot_stability_story.py --mode win \\
        --trace 'PePPAR-Fix on an OCXO (PulsePuppy)=clkpoc3.csv,freerun-ocxo.json' \\
        --trace 'PePPAR-Fix on a TCXO (TimeHAT)=timehat.csv,freerun-tcxo.json' \\
        --skip-before 2026-06-15T03:20:00Z --output peppar-stability-win
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
import matplotlib.colors as mcolors

plt.rcParams['hatch.linewidth'] = 1.1

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plot_clock_stability_stack import load_chA_phase, load_freerun_char  # noqa: E402
from plot_stability_slide import (  # noqa: E402  (also applies shared rcParams)
    TRACE_COLORS, GNSS_PPS_COLOR, IDEAL_COLOR,
    make_taus, dev_with_err, measurement_floor, at_tau, loglog_interp,
)


def _short(label: str) -> str:
    """'PePPAR-Fix on an OCXO (PulsePuppy)' → 'OCXO'."""
    return label.split('(')[0].strip().split()[-1]


def _map_to_grid(m: dict, grid: np.ndarray) -> np.ndarray:
    if not m:
        return np.full(len(grid), np.nan)
    xs = np.array(sorted(m))
    return loglog_interp(xs, np.array([m[x] for x in xs]), grid)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mode', required=True, choices=('win', 'limits'))
    ap.add_argument('--trace', action='append', required=True,
                    help="'Display Label=TICC_CSV[,FREERUN_JSON]'.  Repeat per clock.")
    ap.add_argument('--gnss-pps', default=None,
                    help="'Label=TICC_CSV' raw GNSS PPS ref; default first trace chB.")
    ap.add_argument('--skip-before', default=None)
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--title', default=None)
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

    glabel, gpath = ((args.gnss_pps.split('=', 1)[0].strip(),
                      args.gnss_pps.split('=', 1)[1].strip()) if args.gnss_pps
                     else ('Raw GNSS PPS (F9T)', args.trace[0].split('=', 1)[1].split(',')[0].strip()))
    g_phase = load_chA_phase(Path(gpath), skip_dt, channel='chB')

    title = args.title or (
        'PePPAR-Fix — the win: disciplined output vs raw GNSS'
        if args.mode == 'win' else
        'PePPAR-Fix — the limits: noise floor, free-run oscillator, remaining room')

    fig, (ax_t, ax_a) = plt.subplots(1, 2, figsize=(16, 9))
    tau_grid = np.logspace(0, 4, 240)
    n_max = max(len(p) for _, p, _, _ in clocks)

    for ax, kind, ylabel, ptitle, ylim in [
            (ax_t, 'tdev', 'TDEV  (ns)', 'Time stability  —  TDEV(τ)', (5e-3, 5e1)),
            (ax_a, 'adev', 'ADEV  (fractional)', 'Frequency stability  —  ADEV(τ)', (1e-12, 1e-8))]:
        ymin, ymax = ylim
        taus_spec = make_taus(n_max, args.taus)
        floor = measurement_floor(tau_grid, kind)

        # (1) Measurement floor — behind everything (both slides).
        ax.fill_between(tau_grid, ymin, np.clip(floor, ymin, ymax),
                        color='0.82', alpha=0.6, zorder=0, lw=0)
        ax.plot(tau_grid, floor, color='0.5', lw=1.0, zorder=1)

        # (4) Ideal GNSS, anchored to raw PPS at τ=1 s (both slides); raw PPS
        # itself is drawn only on the win slide (it is the baseline we beat).
        gi = None
        if len(g_phase):
            gt, gd, _ = dev_with_err(g_phase, kind, taus_spec)
            if args.mode == 'win':
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

        for i, (label, cha, fr_a, fr_t) in enumerate(clocks):
            c = TRACE_COLORS[i % len(TRACE_COLORS)]
            ct, cd, _ = dev_with_err(cha, kind, taus_spec)
            ci = loglog_interp(ct, cd, tau_grid)

            if args.mode == 'win' and gi is not None:
                # Solid improvement shading where disciplined beats raw PPS.
                win = np.isfinite(ci) & np.isfinite(gi) & (ci < gi)
                if win.any():
                    ax.fill_between(tau_grid[win], ci[win], gi[win], color=c,
                                    alpha=0.18, zorder=1.5, lw=0,
                                    label=('PePPAR-Fix improvement over raw PPS'
                                           if i == 0 else None))

            if args.mode == 'limits':
                # (2) Free-running oscillator — the floor discipline can't beat.
                fr = fr_t if kind == 'tdev' else fr_a
                if fr:
                    xs = np.array(sorted(fr))
                    ax.plot(xs, np.array([fr[x] for x in xs]), color=c, lw=1.8,
                            ls=(0, (6, 2)), alpha=0.8, marker='s', ms=4, zorder=3,
                            label=f'{_short(label)} free-running')
                # Hatched remaining room: disciplined → max(free-run, floor),
                # only where the loop hasn't reached the oscillator floor.
                fri = _map_to_grid(fr, tau_grid)
                bottom = np.maximum(fri, floor)
                room = np.isfinite(ci) & np.isfinite(bottom) & (ci > bottom)
                if room.any():
                    ax.fill_between(tau_grid[room], bottom[room], ci[room],
                                    facecolor=mcolors.to_rgba(c, 0.08),
                                    edgecolor=mcolors.to_rgba(c, 0.55),
                                    hatch='////', lw=0.0, zorder=1.5,
                                    label=('Remaining room for improvement'
                                           if i == 0 else None))

            # (5/6) Disciplined DO output — the hero (no σ band).
            ax.plot(ct, cd, color=c, lw=3.6, zorder=5, solid_capstyle='round',
                    label=label)

        ax.set_xscale('log'); ax.set_yscale('log')
        ax.set_xlim(1, 1e4); ax.set_ylim(ymin, ymax)
        ax.set_box_aspect(1)
        ax.set_xlabel('Averaging time  τ  (s)')
        ax.set_ylabel(ylabel); ax.set_title(ptitle, pad=10)
        ax.grid(True, which='both', ls=':', alpha=0.35)
        ax.text(1.3, ymin * (ymax / ymin) ** 0.025,
                'TICC + FE-5680A Rb measurement floor', fontsize=12,
                color='0.30', style='italic', va='bottom')
        h, lab = ax.get_legend_handles_labels()
        order = sorted(range(len(lab)),
                       key=lambda k: (0 if lab[k].startswith('PePPAR-Fix on') else
                                      1 if 'free-running' in lab[k] else
                                      2 if ('improvement' in lab[k] or 'Remaining' in lab[k]) else 3))
        ax.legend([h[k] for k in order], [lab[k] for k in order],
                  loc='upper right', framealpha=0.93, fontsize=12)

    fig.suptitle(title, fontsize=23, fontweight='bold', y=0.97)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    for ext, dpi in (('.png', 200), ('.pdf', None)):
        fig.savefig(str(args.output) + ext, dpi=dpi, facecolor='white')
        print(f'wrote {args.output}{ext}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())

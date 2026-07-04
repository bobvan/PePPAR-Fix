#!/usr/bin/env python3
"""Presentation slide: PePPAR-Fix clock stability (TDEV + ADEV) from TICC logs.

16:9 slide, two log-log panels (TDEV left, ADEV right), styled to match the
Delta presentation deck (large fonts, bold traces, PDF+PNG).  Built for the
"PePPAR-Fix on a TCXO vs on an OCXO" contrast; audience-facing display labels
replace host names.

Layers, most prominent first:
  * disciplined chA per clock (bold, wide, ±1σ confidence band) — the heroes.
  * raw GNSS PPS chB (muted, thin) — shows receiver PPS quantization at short τ.
  * GNSS time-transfer ideal — white-PM asymptote anchored to the raw PPS:
      τ⁻¹ on the ADEV panel, τ⁻¹ᐟ² on the TDEV panel (SAME white-PM noise; TDEV
      ≡ τ·MDEV/√3 re-weights white PM to a −½ slope — that's why ADEV curves can
      ride a −1 line but TDEV curves cannot).
  * measurement floor (shaded) — TICC single-shot (60 ps, white PM) in
    quadrature with the FE-5680A Rb reference (1.4e-11·τ⁻¹ᐟ²); a bathtub in
    TDEV, the region we cannot resolve below.

Axes are forced to equal decades-per-side (box_aspect=1) so a slope-1 line
renders at ~45°.

Usage:
    python3 tools/plot_stability_slide.py \\
        --trace 'PePPAR-Fix on an OCXO (PulsePuppy)=/tmp/clkpoc3.csv' \\
        --trace 'PePPAR-Fix on a TCXO (TimeHAT)=/tmp/timehat.csv' \\
        --gnss-pps 'Raw GNSS PPS (F9T)=/tmp/clkpoc3.csv' \\
        --skip-before 2026-06-15T03:20:00Z --output /tmp/peppar-stability-slide
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
from plot_clock_stability_stack import load_chA_phase  # noqa: E402
# Spec measurement-floor model factored into a shared helper so the
# stability-floor overlay can draw the SAME a-priori floor (math unchanged).
from measurement_floor_model import (measurement_floor,  # noqa: E402,F401
                                     TICC_FLOOR_PS, RB_ADEV_1S,
                                     RB_FLICKER_FLOOR)
import allantools  # noqa: E402

plt.rcParams.update({
    "font.size": 18, "axes.titlesize": 24, "axes.labelsize": 20,
    "xtick.labelsize": 16, "ytick.labelsize": 16, "legend.fontsize": 14,
    "axes.linewidth": 1.4, "figure.facecolor": "white",
})

TRACE_COLORS = ["#0072B2", "#D55E00", "#009E73", "#CC79A7"]  # Okabe-Ito
GNSS_PPS_COLOR = "#555555"
IDEAL_COLOR = "#9467bd"
# TICC_FLOOR_PS / RB_ADEV_1S / RB_FLICKER_FLOOR / measurement_floor are
# imported from measurement_floor_model (re-exported above) — single source
# of truth shared with the stability-floor overlay.


def make_taus(n: int, mode: str):
    if mode in ('all', 'octave', 'decade'):
        return mode
    m = max(2, n // 3)
    return np.unique(np.round(np.logspace(0, np.log10(m), 90)).astype(int)).astype(float)


def dev_with_err(phase_ns, kind, taus):
    """(taus, dev, err) for 'adev'|'tdev' from a phase series (ns)."""
    if len(phase_ns) < 60:
        return np.array([]), np.array([]), np.array([])
    phase_s = phase_ns * 1e-9
    fn = allantools.tdev if kind == 'tdev' else allantools.adev
    t, dev, err, _ = fn(phase_s, rate=1.0, taus=taus, data_type='phase')
    if kind == 'tdev':
        dev, err = dev * 1e9, err * 1e9     # s → ns
    keep = np.isfinite(dev) & (dev > 0)
    return t[keep], dev[keep], err[keep]


def at_tau(taus, vals, x):
    if not len(taus):
        return None
    j = int(np.argmin(np.abs(taus - x)))
    return vals[j]


def loglog_interp(t, dev, grid):
    """Interpolate (t, dev) onto grid in log-log; NaN outside the data range."""
    if len(t) < 2:
        return np.full(len(grid), np.nan)
    out = np.interp(np.log10(grid), np.log10(t), np.log10(dev),
                    left=np.nan, right=np.nan)
    return 10.0 ** out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--trace', action='append', required=True,
                    help="'Display Label=TICC_CSV' (chA hero).  Repeat.")
    ap.add_argument('--gnss-pps', default=None,
                    help="'Label=TICC_CSV' — raw GNSS PPS chB (muted) + ideal anchor.")
    ap.add_argument('--skip-before', default=None)
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--title', default='PePPAR-Fix clock stability — disciplined output, overnight')
    ap.add_argument('--taus', default='dense', help="dense|all|octave|decade (default dense)")
    args = ap.parse_args()

    skip_dt = (datetime.fromisoformat(args.skip_before.replace('Z', '+00:00'))
               if args.skip_before else None)

    heroes = []
    for spec in args.trace:
        label, path = spec.split('=', 1)
        ph = load_chA_phase(Path(path.strip()), skip_dt, channel='chA')
        if not len(ph):
            ap.error(f'no chA data in {path}')
        heroes.append((label.strip(), ph))
        print(f'{label.strip():42s} chA {len(ph):>7d} pts', file=sys.stderr)

    gnss = None
    if args.gnss_pps:
        glabel, gpath = args.gnss_pps.split('=', 1)
        gph = load_chA_phase(Path(gpath.strip()), skip_dt, channel='chB')
        if len(gph):
            gnss = (glabel.strip(), gph)
            print(f'{glabel.strip():42s} chB {len(gph):>7d} pts', file=sys.stderr)

    fig, (ax_t, ax_a) = plt.subplots(1, 2, figsize=(16, 9))
    tau_grid = np.logspace(0, 4, 240)

    for ax, kind, ylabel, title, ylim in [
            (ax_t, 'tdev', 'TDEV  (ns)', 'Time stability  —  TDEV(τ)', (5e-3, 5e1)),
            (ax_a, 'adev', 'ADEV  (fractional)', 'Frequency stability  —  ADEV(τ)', (1e-12, 1e-8))]:
        ymin, ymax = ylim
        n_max = max(len(p) for _, p in heroes)
        taus_spec = make_taus(n_max, args.taus)

        # Measurement-floor shading (behind everything).
        floor = measurement_floor(tau_grid, kind)
        ax.fill_between(tau_grid, ymin, np.clip(floor, ymin, ymax),
                        color='0.82', alpha=0.6, zorder=0, lw=0)
        ax.plot(tau_grid, floor, color='0.5', lw=1.0, zorder=1)

        # Raw GNSS PPS (chB) — muted, thin; and anchor the ideal to its τ=1s.
        anchor1s = None
        g_t = g_dev = None
        if gnss:
            glabel, gph = gnss
            g_t, g_dev, _ = dev_with_err(gph, kind, taus_spec)
            ax.plot(g_t, g_dev, color=GNSS_PPS_COLOR, lw=2.0, ls='-', marker='.',
                    ms=5, alpha=0.85, zorder=3, label=glabel)
            anchor1s = at_tau(g_t, g_dev, 1.0)

        # GNSS time-transfer ideal (white PM): τ⁻¹ on ADEV, τ⁻¹ᐟ² on TDEV.
        if anchor1s:
            p = -1.0 if kind == 'adev' else -0.5
            ax.plot(tau_grid, anchor1s * tau_grid ** p, color=IDEAL_COLOR,
                    lw=1.8, ls='--', zorder=2,
                    label=('GNSS ideal  (τ⁻¹)' if kind == 'adev'
                           else 'GNSS ideal  (τ⁻¹ᐟ²)'))

        # Compute hero deviations up front (needed for the improvement band).
        hero_dev = [(label, *dev_with_err(ph, kind, taus_spec))
                    for label, ph in heroes]

        # Improvement shading — the gap between raw GNSS PPS and the OCXO
        # output is the stability PePPAR-Fix delivers over the bare receiver.
        if g_t is not None and len(g_t):
            idx = next((k for k, h in enumerate(hero_dev)
                        if 'OCXO' in h[0].upper()), 0)
            _, o_t, o_dev, _ = hero_dev[idx]
            gi = loglog_interp(g_t, g_dev, tau_grid)
            oi = loglog_interp(o_t, o_dev, tau_grid)
            band = np.isfinite(gi) & np.isfinite(oi) & (gi > oi)
            if band.any():
                ax.fill_between(tau_grid[band], oi[band], gi[band],
                                color=TRACE_COLORS[idx % len(TRACE_COLORS)],
                                alpha=0.15, zorder=1.5, lw=0,
                                label='Improvement over raw PPS')

        # Hero traces — bold + ±1σ band.
        for i, (label, t, dev, err) in enumerate(hero_dev):
            c = TRACE_COLORS[i % len(TRACE_COLORS)]
            ax.fill_between(t, np.maximum(dev - err, dev * 0.3), dev + err,
                            color=c, alpha=0.18, zorder=4, lw=0)
            ax.plot(t, dev, color=c, lw=3.6, label=label, zorder=5,
                    solid_capstyle='round')

        ax.set_xscale('log'); ax.set_yscale('log')
        ax.set_xlim(1, 1e4); ax.set_ylim(ymin, ymax)
        ax.set_box_aspect(1)                       # equal decades/side → 45°
        ax.set_xlabel('Averaging time  τ  (s)')
        ax.set_ylabel(ylabel); ax.set_title(title, pad=10)
        ax.grid(True, which='both', ls=':', alpha=0.35)
        ax.text(1.3, ymin * (ymax / ymin) ** 0.025,
                'TICC + FE-5680A Rb measurement floor', fontsize=12,
                color='0.30', style='italic', va='bottom')
        h, lab = ax.get_legend_handles_labels()
        order = sorted(range(len(lab)),
                       key=lambda k: 0 if lab[k].startswith('PePPAR-Fix') else 1)
        ax.legend([h[k] for k in order], [lab[k] for k in order],
                  loc='upper right', framealpha=0.93)

    fig.suptitle(args.title, fontsize=23, fontweight='bold', y=0.97)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    for ext, dpi in (('.png', 200), ('.pdf', None)):
        fig.savefig(str(args.output) + ext, dpi=dpi, facecolor='white')
        print(f'wrote {args.output}{ext}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())

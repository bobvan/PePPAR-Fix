#!/usr/bin/env python3
"""Companion figure: DO→GPS tracking error (chA−chB) vs disciplined output (chA).

Answers the question the main stability slide raises — "is the TCXO's large
long-τ output gap a *discipline* deficiency or an *oscillator* limit?"

chA−chB subtracts the two TICC channels, so the common Rb timebase cancels and
what remains is DO PPS − GNSS PPS = the servo's tracking error against GPS (the
work the loop actually did).  Overlaying it with the absolute output chA, the
raw GNSS PPS chB, and the GNSS white-PM ideal makes the punchline visible:

  * At short τ the tracking error rides the raw GNSS PPS — the loop deliberately
    does NOT chase the receiver sawtooth, letting the DO's own stability show.
  * At long τ the loop averages GPS down and BOTH clocks track GPS about equally
    well (the chA−chB curves converge low) — discipline works regardless of the
    oscillator.
  * Therefore the TCXO's much larger chA *output* gap at long τ is its own
    above-loop-bandwidth oscillator noise, not a tracking failure.  No servo
    tuning removes it; only a better oscillator does.

Styling, helpers, and the measurement floor are shared with
``plot_stability_slide`` so the two figures read as a matched pair.

Usage:
    python3 tools/plot_tracking_companion.py \\
        --trace 'PePPAR-Fix on an OCXO (PulsePuppy)=/tmp/clkpoc3.csv' \\
        --trace 'PePPAR-Fix on a TCXO (TimeHAT)=/tmp/timehat.csv' \\
        --skip-before 2026-06-15T03:20:00Z \\
        --output /tmp/peppar-tracking-companion --title '...'
"""
from __future__ import annotations

import argparse
import csv
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plot_clock_stability_stack import load_chA_phase  # noqa: E402
from plot_stability_slide import (  # noqa: E402  (also applies shared rcParams)
    TRACE_COLORS, GNSS_PPS_COLOR,
    make_taus, dev_with_err, measurement_floor,
)

_HALF_PS = 500_000_000_000
_FULL_PS = 1_000_000_000_000


def _short_tag(label: str) -> str:
    up = label.upper()
    return 'OCXO' if 'OCXO' in up else 'TCXO' if 'TCXO' in up else label


def load_diff_phase(path: Path, skip_before: datetime | None = None) -> np.ndarray:
    """chA − chB per shared ref_sec (DO PPS − GNSS PPS, ns), linear-detrended.

    Both channels are timed against the same TICC Rb second, so the difference
    is purely the sub-second ps offset (Rb cancels).  Folded into (−0.5s, +0.5s]
    to remove the second-boundary wrap ambiguity, then detrended to drop the
    constant cable/parked offset and any residual rate.
    """
    a: dict[int, int] = {}
    b: dict[int, int] = {}
    with open(path) as f:
        reader = csv.DictReader(f)
        cols = reader.fieldnames or []
        ts_col = ('ts_iso' if 'ts_iso' in cols
                  else 'host_timestamp' if 'host_timestamp' in cols else None)
        for row in reader:
            if skip_before is not None and ts_col is not None:
                if datetime.fromisoformat(
                        row[ts_col].replace('Z', '+00:00')) < skip_before:
                    continue
            sec, ps, ch = int(row['ref_sec']), int(row['ref_ps']), row['channel']
            if ch == 'chA':
                a[sec] = ps
            elif ch == 'chB':
                b[sec] = ps

    common = np.array(sorted(set(a) & set(b)))
    if len(common) < 60:
        return np.array([])
    # Longest gap-free run of shared seconds, so the metric stays on a
    # contiguous 1 Hz series.
    d = np.diff(common)
    bnds = [0] + list(np.where(d != 1)[0] + 1) + [len(common)]
    segs = [(bnds[i], bnds[i + 1]) for i in range(len(bnds) - 1)]
    i0, i1 = max(segs, key=lambda ab: ab[1] - ab[0])
    seg = common[i0:i1]
    if len(seg) < 60:
        return np.array([])

    diff_ps = np.array(
        [((a[int(s)] - b[int(s)] + _HALF_PS) % _FULL_PS) - _HALF_PS for s in seg],
        dtype=np.int64)
    diff_ns = diff_ps.astype(np.float64) * 1e-3
    t = np.arange(len(diff_ns), dtype=np.float64)
    return diff_ns - np.polyval(np.polyfit(t, diff_ns, 1), t)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--trace', action='append', required=True,
                    help="'Display Label=TICC_CSV'.  Repeat per host.")
    ap.add_argument('--gnss-pps', default=None,
                    help="'Label=TICC_CSV' for the raw GNSS PPS reference; "
                         'defaults to the first --trace csv (chB).')
    ap.add_argument('--skip-before', default=None)
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--title', default='PePPAR-Fix — DO→GPS tracking vs output')
    ap.add_argument('--taus', default='dense', help='dense|all|octave|decade')
    args = ap.parse_args()

    skip_dt = (datetime.fromisoformat(args.skip_before.replace('Z', '+00:00'))
               if args.skip_before else None)

    hosts = []
    for spec in args.trace:
        label, path = spec.split('=', 1)
        p = Path(path.strip())
        cha = load_chA_phase(p, skip_dt, channel='chA')
        diff = load_diff_phase(p, skip_dt)
        if not len(cha) or not len(diff):
            ap.error(f'no paired chA/chB data in {path}')
        hosts.append((label.strip(), cha, diff))
        print(f'{label.strip():42s} chA {len(cha):>7d}  chA-chB {len(diff):>7d}',
              file=sys.stderr)

    # GNSS PPS reference: explicit --gnss-pps, else first trace's chB.
    if args.gnss_pps:
        glabel, gpath = args.gnss_pps.split('=', 1)
        glabel, gpath = glabel.strip(), gpath.strip()
    else:
        glabel, gpath = 'Raw GNSS PPS (F9T)', args.trace[0].split('=', 1)[1].strip()
    g_phase = load_chA_phase(Path(gpath), skip_dt, channel='chB')

    fig, (ax_t, ax_a) = plt.subplots(1, 2, figsize=(16, 9))
    tau_grid = np.logspace(0, 4, 240)
    n_max = max(len(d) for _, _, d in hosts)

    for ax, kind, ylabel, title, ylim in [
            (ax_t, 'tdev', 'TDEV  (ns)', 'Tracking stability  —  TDEV(τ)', (5e-3, 5e1)),
            (ax_a, 'adev', 'ADEV  (fractional)', 'Tracking stability  —  ADEV(τ)', (1e-12, 1e-8))]:
        ymin, ymax = ylim
        taus_spec = make_taus(n_max, args.taus)

        # Measurement-floor shading (behind everything).
        floor = measurement_floor(tau_grid, kind)
        ax.fill_between(tau_grid, ymin, np.clip(floor, ymin, ymax),
                        color='0.82', alpha=0.6, zorder=0, lw=0)
        ax.plot(tau_grid, floor, color='0.5', lw=1.0, zorder=1)

        # Raw GNSS PPS reference (the operative reference for tracking error).
        # The τ⁻¹ᐟ² ideal from the main slide is intentionally omitted: real
        # GNSS-vs-Rb flattens to a floor at long τ, so the ideal would mislead
        # about how much tracking headroom remains.
        if len(g_phase):
            gt, gd, _ = dev_with_err(g_phase, kind, taus_spec)
            ax.plot(gt, gd, color=GNSS_PPS_COLOR, lw=2.0, marker='.', ms=5,
                    alpha=0.85, zorder=2, label=glabel)

        # Per host: tracking error chA−chB (bold solid) + output chA (dashed).
        # Where the two coincide the noise is the DO's own (GPS can't help it);
        # where tracking dives below the output, the output's floor is inherited
        # reference noise (FE-5680A Rb), not the oscillator.
        for i, (label, cha, diff) in enumerate(hosts):
            c = TRACE_COLORS[i % len(TRACE_COLORS)]
            tag = _short_tag(label)
            dt, dd, derr = dev_with_err(diff, kind, taus_spec)
            ax.fill_between(dt, np.maximum(dd - derr, dd * 0.3), dd + derr,
                            color=c, alpha=0.16, zorder=4, lw=0)
            ax.plot(dt, dd, color=c, lw=3.6, zorder=5, solid_capstyle='round',
                    label=f'{tag} — DO→GPS tracking (chA−chB)')
            ct, cd, _ = dev_with_err(cha, kind, taus_spec)
            ax.plot(ct, cd, color=c, lw=2.2, ls=(0, (6, 2)), alpha=0.9,
                    zorder=4, label=f'{tag} — DO output (chA)')

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
                       key=lambda k: 0 if 'tracking' in lab[k] else
                       1 if 'output' in lab[k] else 2)
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

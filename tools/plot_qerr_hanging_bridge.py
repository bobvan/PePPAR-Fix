#!/usr/bin/env python3
"""plot_qerr_hanging_bridge.py — the F9T PPS-quantization sawtooth + "hanging
bridge", the receiver-side measurement correction (qErr / TIM-TP).

Story-1 (sub-15 ns onion) measurement-chain item, and the receiver-side
counterpart to the satellite-side SSR plots.  A u-blox timing receiver
generates its PPS by dividing an internal clock, so the pulse lands on a
clock edge — the "quantization error" qErr (reported in TIM-TP, ps) is how
far the requested edge missed.  Raw, that's an ~8 ns sawtooth; apply qErr and
you recover sub-ns.

A "hanging bridge" forms when the rx-TCXO frequency sweeps through an exact
1-Hz boundary (zero relative offset): the sawtooth slope is positive on one
side, flattens through a gradual zero-slope curve, then goes negative — the
TCXO is acting as a thermometer (frequency follows temperature).  Two panels:

  TOP    raw qErr — the ~8 ns sawtooth + the hanging-bridge flat spot
  BOTTOM np.unwrap(qErr) — continuous accumulated phase: the parabola is the
         TCXO's actual phase excursion vs GPS ("an expensive thermometer")

Usage:
    python3 tools/plot_qerr_hanging_bridge.py /tmp/f9t_qerr_24h.npz \
        --label "F9T (ptBoat)" --out-dir plots [--start S --window 500] [--period-ps 8000]
"""
import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.size": 17, "axes.titlesize": 22, "axes.labelsize": 20,
    "xtick.labelsize": 18, "ytick.labelsize": 18, "legend.fontsize": 16,
})


def load(path):
    z = np.load(path)
    qe = z["qe"].astype(float)
    tow = z["tow"].astype(float) if "tow" in z else np.arange(len(qe)) * 1000.0
    return qe, tow


def find_bridge(qe, period, win):
    """Pick the window whose unwrapped phase has the sharpest parabolic hump
    (a frequency sign-reversal = the hanging bridge)."""
    phase = np.unwrap(qe, period=period)
    half = win // 2
    best, best_score = half, -1.0
    step = max(1, win // 10)
    for c in range(half, len(qe) - half, step):
        seg = phase[c - half:c + half]
        # curvature: how far the midpoint sits above/below the chord ends
        chord = 0.5 * (seg[0] + seg[-1])
        score = abs(seg.mean() - chord)
        if score > best_score:
            best, best_score = c, score
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("npz", help="qErr npz with arrays qe (ps) [, tow (ms)]")
    ap.add_argument("--label", default="F9T")
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--start", type=int, default=None, help="window start sample (default: auto)")
    ap.add_argument("--window", type=int, default=500, help="window length (samples ≈ s)")
    ap.add_argument("--period-ps", type=float, default=8000.0, help="qErr wrap period (PPS quantum)")
    args = ap.parse_args()

    qe, tow = load(args.npz)
    win = args.window
    if args.start is not None:
        c0 = args.start
    else:
        c0 = find_bridge(qe, args.period_ps, win) - win // 2
    c0 = max(0, min(c0, len(qe) - win))
    seg = qe[c0:c0 + win]
    t = np.arange(win)
    phase = np.unwrap(seg, period=args.period_ps)
    phase = phase - phase[0]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 9), height_ratios=[1.15, 1])
    ax1.plot(t, seg, ".-", ms=3, lw=0.8, color="tab:blue")
    pp = seg.max() - seg.min()
    ax1.set_ylabel("qErr  [ps]")
    ax1.set_title(f"{args.label}: PPS quantization sawtooth + hanging bridge  "
                  f"(≈{pp/1000:.1f} ns p-p)")
    ax1.grid(alpha=0.3)
    ax1.annotate("hanging bridge\n(TCXO freq through 1 Hz boundary)",
                 xy=(0.5, 0.04), xycoords="axes fraction", ha="center", fontsize=15,
                 bbox=dict(boxstyle="round", fc="#fff3d6", ec="0.6"))

    ax2.plot(t, phase / 1000.0, lw=2.6, color="tab:red")
    ax2.set_xlabel("elapsed time  [s]")
    ax2.set_ylabel("unwrapped qErr  [ns]")
    exc = phase.max() - phase.min()
    ax2.set_title(f"accumulated phase vs GPS = the TCXO is an expensive thermometer  "
                  f"(≈{exc/1000:.0f} ns excursion)")
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    os.makedirs(args.out_dir, exist_ok=True)
    out = os.path.join(args.out_dir, "qerr_hanging_bridge")
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png  window [{c0},{c0+win}]  p-p {pp:.0f} ps  excursion {exc:.0f} ps")


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Visualize the Goldilocks cadence tradeoff.

X axis: scheduler activation interval τ (s, log scale)
Y axis: contribution to disciplined-output phase noise (ns, log scale)

Two curves cross at the Goldilocks τ:

  Down-sloping (actuation injection per unit time):
      σ_act(τ) = σ_q / √τ
      Each firing injects σ_q (the actuator quantum).  At interval τ
      that's 1/τ firings per second; the per-second injected variance
      is σ_q²/τ → σ_RMS = σ_q/√τ.  Fire faster → more total noise.

  Up-sloping (DO free-running drift over the coast period):
      σ_coast(τ) = σ_DO(1s) · τ^slope     (slope = 0.5 for white FM)
      The DO drifts by TDEV_DO(τ) between fires.  Coast longer → more
      accumulated wander.

  Goldilocks τ:  σ_act(τ*) = σ_coast(τ*)
                ⇒ τ* = (σ_q / σ_DO(1s))^(2 / (2·slope + 1))
  For slope=0.5: τ* = σ_q / σ_DO(1s)

Lab fleet examples:

  OCXO + 16-bit DAC:    σ_q ≈ 18 ps, σ_DO(1s) ≈ 18 ps   ⇒ τ* ≈ 1 s
  OCXO + 18-bit DAC:    σ_q ≈ 4 ps,  σ_DO(1s) ≈ 18 ps   ⇒ τ* ≈ 0.25 s
  TCXO + i226 PHC:      σ_q ≈ 15 fs, σ_DO(1s) ≈ 1.17 ns ⇒ τ* ≈ 13 µs

The TCXO+PHC case is degenerate: τ* lands far below any sensible
firing cadence (1 Hz from GNSS), so the envelope minimum is just
"fire as fast as data arrives."  The lesson: the Goldilocks tradeoff
only matters when the actuator's quantum is comparable to the DO's
1-s drift.  For a fine-quantum actuator like adjfine, the coast
curve dominates everywhere in range.

Run:
    python3 tools/plot_goldilocks_cadence.py
    python3 tools/plot_goldilocks_cadence.py --out data/goldilocks.png
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# Physical constants for plotted scenarios
# (See module docstring for derivation of each σ_q.)
SCENARIOS = {
    "OCXO + 16-bit DAC": dict(
        sigma_q_ns=0.018,        # 16-bit DAC LSB over ~4 ppm pull range
        sigma_do_1s_ns=0.018,    # OCXO TDEV(1s) ≈ 18 ps
        color='C0',
        annotation=(
            "16-bit DAC: 1 LSB ≈ 0.018 ppb\n"
            "→ σ_q = 0.018 ppb × 1 s = 18 ps"),
    ),
    "TCXO + i226 PHC adjfine": dict(
        sigma_q_ns=1.5e-5,       # adjfine LSB: 2^-16 ppm ≈ 1.5e-5 ppb
        sigma_do_1s_ns=1.17,     # TimeHat TCXO TDEV(1s) = 1.17 ns
        color='C3',
        annotation=(
            "adjfine: 1 LSB = 2^-16 ppm ≈ 1.5×10^-5 ppb\n"
            "→ σ_q = 1.5×10^-5 ppb × 1 s ≈ 15 fs\n"
            "(6 orders of magnitude finer than DAC)"),
    ),
}


def goldilocks_tau(sigma_q_ns, sigma_do_1s_ns, slope=0.5):
    """τ* where σ_q/√τ = σ_DO(1s)·τ^slope.

    Solving:
        σ_q² / τ = σ_DO(1s)² · τ^(2·slope)
        τ^(2·slope + 1) = (σ_q / σ_DO(1s))^2
        τ = (σ_q / σ_DO(1s))^(2 / (2·slope + 1))
    """
    if sigma_do_1s_ns <= 0:
        return None
    ratio = sigma_q_ns / sigma_do_1s_ns
    return ratio ** (2.0 / (2.0 * slope + 1.0))


def plot_tradeoff(ax, sigma_q_ns, sigma_do_1s_ns, slope, color,
                  title, annotation, min_interval_s=1.0,
                  tau_range=(1e-4, 1e3), y_range=(1e-5, 1e2)):
    """Render the two-line tradeoff + envelope on one set of axes."""
    tau = np.logspace(np.log10(tau_range[0]), np.log10(tau_range[1]), 600)
    sigma_act   = sigma_q_ns / np.sqrt(tau)
    sigma_coast = sigma_do_1s_ns * tau ** slope
    envelope    = np.sqrt(sigma_act ** 2 + sigma_coast ** 2)

    ax.loglog(tau, sigma_act, '--', color=color, alpha=0.7, linewidth=1.5,
              label=r"actuation  $\sigma_q / \sqrt{\tau}$ (negative slope)")
    ax.loglog(tau, sigma_coast, ':', color=color, alpha=0.7, linewidth=1.5,
              label=fr"coast  $\sigma_{{DO}}(1s) \cdot \tau^{{{slope:.1f}}}$ (positive slope)")
    ax.loglog(tau, envelope, '-', color=color, linewidth=2.5,
              label=r"total envelope  $\sqrt{\sigma_{act}^2 + \sigma_{coast}^2}$")

    # Mark Goldilocks point if inside the visible range
    tau_star = goldilocks_tau(sigma_q_ns, sigma_do_1s_ns, slope)
    if tau_star is not None and tau_range[0] <= tau_star <= tau_range[1]:
        sigma_min = sigma_q_ns / np.sqrt(tau_star) * np.sqrt(2)
        ax.plot([tau_star], [sigma_min], 'o', color='black', markersize=10,
                markerfacecolor=color, markeredgecolor='black',
                markeredgewidth=1.5, zorder=10)
        ax.annotate(fr"$\tau^* = {tau_star:.2g}$ s",
                    xy=(tau_star, sigma_min),
                    xytext=(tau_star * 2.0, sigma_min * 2.5),
                    fontsize=11, fontweight='bold',
                    arrowprops=dict(arrowstyle="->", color='black',
                                    alpha=0.6, lw=1.0))

    # Mark min_interval (the 1 Hz floor from GNSS data cadence)
    ax.axvline(min_interval_s, color='gray', linewidth=1.0,
               linestyle='-.', alpha=0.6)
    ax.text(min_interval_s, y_range[0] * 1.5,
            f"min_interval\n= {min_interval_s:g} s",
            fontsize=8, color='gray', ha='center',
            verticalalignment='bottom')

    # Mark σ_q level
    ax.axhline(sigma_q_ns, color='gray', linewidth=0.5, alpha=0.4)
    ax.text(tau_range[1] / 1.5, sigma_q_ns,
            fr"$\sigma_q$ = {sigma_q_ns*1000:g} ps",
            fontsize=8, color='gray',
            verticalalignment='bottom', horizontalalignment='right')

    ax.set_xlim(tau_range)
    ax.set_ylim(y_range)
    ax.set_xlabel("scheduler interval τ (s)")
    ax.set_ylabel("phase-noise contribution (ns)")
    ax.set_title(title, fontsize=11)
    ax.grid(True, which='major', alpha=0.4)
    ax.grid(True, which='minor', alpha=0.15)
    ax.legend(fontsize=8, loc='upper left', framealpha=0.9)

    # Annotation box with parameters
    ax.text(0.97, 0.03, annotation,
            transform=ax.transAxes, fontsize=8,
            horizontalalignment='right',
            verticalalignment='bottom',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='white',
                      edgecolor='gray', alpha=0.85))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default="data/goldilocks_cadence.png",
                   help="Output PNG path (default: data/goldilocks_cadence.png)")
    p.add_argument("--slope", type=float, default=0.5,
                   help="TDEV slope (0.5=WFM, 1.0=FFM, default 0.5)")
    p.add_argument("--min-interval-s", type=float, default=1.0,
                   help="Physical min cadence floor (default 1.0 s = 1 Hz GNSS)")
    args = p.parse_args()

    fig, axes = plt.subplots(1, 2, figsize=(15, 7), sharey=True)

    # Left: OCXO + 16-bit DAC — the textbook bowl
    s = SCENARIOS["OCXO + 16-bit DAC"]
    plot_tradeoff(
        axes[0],
        sigma_q_ns=s["sigma_q_ns"],
        sigma_do_1s_ns=s["sigma_do_1s_ns"],
        slope=args.slope,
        color=s["color"],
        title=("OCXO + 16-bit DAC (lab fleet — clkPoC3, MadHat)\n"
               r"$\sigma_q \approx \sigma_{DO}(1s) \approx 18$ ps  "
               r"$\Rightarrow$  $\tau^* \approx 1$ s"),
        annotation=s["annotation"],
        min_interval_s=args.min_interval_s,
    )

    # Right: TCXO + i226 PHC adjfine — coast dominates everywhere
    s = SCENARIOS["TCXO + i226 PHC adjfine"]
    plot_tradeoff(
        axes[1],
        sigma_q_ns=s["sigma_q_ns"],
        sigma_do_1s_ns=s["sigma_do_1s_ns"],
        slope=args.slope,
        color=s["color"],
        title=("TCXO + i226 PHC adjfine (TimeHat)\n"
               r"$\sigma_q \approx 15$ fs $\ll \sigma_{DO}(1s) \approx 1.17$ ns  "
               r"$\Rightarrow$  $\tau^* \approx 13$ µs"),
        annotation=s["annotation"],
        min_interval_s=args.min_interval_s,
    )

    # Add an interpretive note over the TCXO panel showing why this is degenerate
    axes[1].annotate(
        "no Goldilocks knee in operating range\n"
        "→ fire as fast as data arrives\n"
        "(coast curve dominates everywhere)",
        xy=(args.min_interval_s, 1.17 * args.min_interval_s ** args.slope),
        xytext=(0.005, 5.0),
        fontsize=9, color='darkred', fontweight='bold',
        arrowprops=dict(arrowstyle="->", color='darkred', alpha=0.7, lw=1.2),
        bbox=dict(boxstyle='round,pad=0.4', facecolor='mistyrose',
                  edgecolor='darkred', alpha=0.9))

    fig.suptitle("Goldilocks cadence: actuation noise vs DO coast drift",
                 fontsize=13, fontweight='bold')
    plt.tight_layout()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out, dpi=120)
    print(f"wrote {out}")

    # Print τ* table to stdout
    print()
    print("Goldilocks τ* table:")
    print(f"  {'scenario':32s}  {'σ_q (ns)':>10s}  {'σ_DO(1s) (ns)':>14s}  {'τ* (s)':>10s}")
    for label, s in SCENARIOS.items():
        ts = goldilocks_tau(s["sigma_q_ns"], s["sigma_do_1s_ns"], args.slope)
        print(f"  {label:32s}  {s['sigma_q_ns']:10.6f}  {s['sigma_do_1s_ns']:14.4f}  {ts:10.2e}")


if __name__ == "__main__":
    sys.exit(main())

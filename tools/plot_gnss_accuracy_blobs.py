#!/usr/bin/env python3
"""Plot the GNSS observation-time vs accuracy tradeoff as overlapping blobs.

Visualises the regimes from docs/gnss-accuracy-ladder.md: each
positioning method occupies a region in the (observation time,
position accuracy) plane.  Overlapping blobs make it visible that
the methods are continuous tradeoffs, not crisp tiers.  Time
accuracy on the secondary y-axis is the c-equivalent of the position
accuracy (1 m ≈ 3.33 ns) so the reader can read off the corresponding
timing precision at a glance; method-specific time-accuracy values
are annotated on each blob where they diverge from the c-equivalent.

Output: docs/gnss-accuracy-blobs.png
"""
from __future__ import annotations

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgba

C_MPS = 2.998e8  # speed of light for position↔time equivalence


# Each blob is a working-regime rectangle in (log obs_time, log position).
# x_lo/x_hi: observation time in seconds.
# y_lo/y_hi: horizontal position accuracy in metres.
# t_acc: method-specific time accuracy (one-σ), as a label string.
BLOBS = [
    dict(label="PR (SPP)",
         note="t≈30–300 ns",
         x_lo=2,         x_hi=120,        y_lo=2.0,    y_hi=5.0,
         color="#d97706"),
    # F9T-class timing receiver: carrier-smoothed SPP for position
    # (PR-class), but onboard PPS + UBX-TIM-TP qErr resolves the
    # 128 MHz hardware tick to sub-ns at the PPS edge → time accuracy
    # decoupled from position accuracy.  NOT float PPP — the MCU
    # doesn't fetch precise orbits / clocks; the algorithm is
    # broadcast-eph + Klobuchar/NeQuick iono + Kalman PVT.
    dict(label="Timing rx (F9T PPS+qErr)",
         note="t≈2–10 ns",
         x_lo=1,         x_hi=30,         y_lo=1.0,    y_hi=3.0,
         color="#0d9488"),
    dict(label="Local NTRIP / RTK",
         note="t≈3–10 ns",
         x_lo=5,         x_hi=180,        y_lo=0.01,   y_hi=0.05,
         color="#0ea5e9"),
    dict(label="Float PPP",
         note="t≈0.3–3 ns",
         x_lo=300,       x_hi=14400,      y_lo=0.05,   y_hi=1.0,
         color="#65a30d"),
    dict(label="PPP-AR",
         note="t≈0.05–0.3 ns",
         x_lo=300,       x_hi=3600,       y_lo=0.005,  y_hi=0.05,
         color="#7c3aed"),
    dict(label="OPUS / multi-day post-proc",
         note="t≈<0.1 ns",
         x_lo=86400,     x_hi=518400,     y_lo=0.003,  y_hi=0.01,
         color="#dc2626"),
]

# Position accuracy below this needs external corrections — RTK base,
# NTRIP/SSR caster for PPP, precise products for PPP-AR, OPUS upload
# for post-processed.  Above the line: receiver-only operation
# (broadcast eph + onboard algorithms) is sufficient.
INTERNET_THRESHOLD_M = 0.10


def _log_ellipse(xc, yc, xw_log, yh_log, n=120):
    """Polygon approximating an ellipse in log-log space."""
    th = np.linspace(0, 2 * np.pi, n)
    x = 10 ** (np.log10(xc) + xw_log * np.cos(th))
    y = 10 ** (np.log10(yc) + yh_log * np.sin(th))
    return x, y


def _fmt_time(s: float) -> str:
    if s < 1:      return f"{s:.0f}s"
    if s < 60:     return f"{int(s)}s"
    if s < 3600:   return f"{int(s/60)}min"
    if s < 86400:  return f"{int(s/3600)}h"
    return f"{int(s/86400)}d"


def _fmt_time_acc(t: float) -> str:
    if t < 1e-9:   return f"{t*1e12:.0f} ps"
    if t < 1e-6:   return f"{t*1e9:.0f} ns"
    if t < 1e-3:   return f"{t*1e6:.0f} μs"
    return f"{t*1e3:.0f} ms"


def main():
    fig, ax = plt.subplots(figsize=(11.5, 7.5))
    ax.set_xscale("log")
    ax.set_yscale("log")

    # Internet-corrections-required band (drawn FIRST so blobs sit on top).
    # Below INTERNET_THRESHOLD_M position accuracy requires real-time
    # corrections (NTRIP base / SSR caster / IGS products); above it,
    # the receiver's onboard algorithms with broadcast ephemeris suffice.
    ax.axhspan(2e-3, INTERNET_THRESHOLD_M, facecolor="#fef3c7",
               edgecolor="none", alpha=0.55, zorder=0)
    # Dashed boundary line for visibility.
    ax.axhline(INTERNET_THRESHOLD_M, color="#a16207", linewidth=1.0,
               linestyle="--", alpha=0.7, zorder=0)
    # Boundary label on the right side of the band.
    ax.text(0.985, INTERNET_THRESHOLD_M * 1.18,
            "↑  Receiver-only (broadcast eph)",
            transform=ax.get_yaxis_transform(),
            ha="right", va="bottom", fontsize=8.5, color="#92400e",
            fontstyle="italic")
    ax.text(0.985, INTERNET_THRESHOLD_M / 1.18,
            "↓  Internet corrections required  (NTRIP / SSR / IGS / OPUS)",
            transform=ax.get_yaxis_transform(),
            ha="right", va="top", fontsize=8.5, color="#92400e",
            fontstyle="italic", fontweight="bold")

    # Blobs in order; later ones drawn on top.
    for b in BLOBS:
        xc = np.sqrt(b["x_lo"] * b["x_hi"])
        yc = np.sqrt(b["y_lo"] * b["y_hi"])
        xw_log = np.log10(b["x_hi"] / b["x_lo"]) / 2
        yh_log = np.log10(b["y_hi"] / b["y_lo"]) / 2
        x, y = _log_ellipse(xc, yc, xw_log, yh_log)
        rgba = to_rgba(b["color"], alpha=0.35)
        edge = to_rgba(b["color"], alpha=0.95)
        ax.fill(x, y, color=rgba, edgecolor=edge, linewidth=2.0)
        # Label inside the blob: method on top, time-acc below.
        ax.text(xc, yc * 1.18, b["label"], ha="center", va="center",
                fontsize=10.5, fontweight="bold", color=edge)
        ax.text(xc, yc / 1.18, b["note"], ha="center", va="center",
                fontsize=9, color=edge)

    ax.set_xlim(1, 1e6)
    ax.set_ylim(2e-3, 20)

    # X ticks: human-readable observation times (major).
    xticks = [1, 10, 60, 600, 3600, 86400, 604800]
    ax.set_xticks(xticks)
    ax.set_xticklabels([_fmt_time(s) for s in xticks])
    # Y ticks: explicit decade labels in human-readable cm/m units.
    yticks = [0.01, 0.10, 1.0, 10.0]
    ytick_labels = ["1 cm", "10 cm", "1 m", "10 m"]
    ax.set_yticks(yticks)
    ax.set_yticklabels(ytick_labels)
    # Minor ticks: the in-decade subdivisions of the log scale (2, 3,
    # ..., 9 × each decade) — drawn but unlabeled, so the eye can
    # judge intermediate positions.
    from matplotlib.ticker import LogLocator, NullFormatter
    ax.yaxis.set_minor_locator(LogLocator(base=10, subs=range(2, 10)))
    ax.yaxis.set_minor_formatter(NullFormatter())

    ax.set_xlabel("Observation time", fontsize=12)
    ax.set_ylabel("Horizontal position accuracy (1-σ)", fontsize=12)
    # Decade gridlines pronounced; in-decade subdivisions present but
    # subtle so the decade structure reads at a glance.
    ax.grid(True, which="major", alpha=0.55, linewidth=0.9,
            linestyle="-", color="#94a3b8")
    ax.grid(True, which="minor", alpha=0.18, linewidth=0.4,
            linestyle=":", color="#94a3b8")
    ax.set_axisbelow(True)
    ax.set_title("GNSS positioning: observation-time vs accuracy regimes",
                 fontsize=13.5, pad=14)

    # Annotate the c-equivalence reference at the top, above the blobs.
    ax.text(0.015, 0.97,
            "Secondary y: c-equivalent timing (1 m ≈ 3.33 ns).\n"
            "Method-specific timing labelled inside each blob.",
            transform=ax.transAxes, ha="left", va="top",
            fontsize=8.5, color="#475569",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white",
                      edgecolor="#cbd5e1", alpha=0.9))

    # Secondary y-axis: c-equivalent time accuracy.
    ax2 = ax.twinx()
    ax2.set_yscale("log")
    ax2.set_ylim(2e-3 / C_MPS, 20 / C_MPS)
    y2_ticks_m = [1e-2, 1e-1, 1, 10]
    ax2.set_yticks([m / C_MPS for m in y2_ticks_m])
    ax2.set_yticklabels([_fmt_time_acc(m / C_MPS) for m in y2_ticks_m])
    ax2.set_ylabel("c-equivalent time accuracy", fontsize=12, color="#475569")
    ax2.tick_params(axis="y", colors="#475569")

    # Output.
    out = os.path.abspath(os.path.join(
        os.path.dirname(__file__), os.pardir, "docs",
        "gnss-accuracy-blobs.png"))
    plt.tight_layout()
    plt.savefig(out, dpi=140, bbox_inches="tight")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()

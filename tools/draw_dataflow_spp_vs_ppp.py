#!/usr/bin/env python3
"""draw_dataflow_spp_vs_ppp.py — SPP-vs-PPP *data-flow* slide (16:9).

Companion to draw_receiver_block_diagram.py ("one set of raw observations ->
two timing solutions").  Where that slide shows the hardware split, this one
follows the *data*: what each estimator consumes.

  TOP lane  — SPP: code pseudorange + broadcast ephemeris/clock + a simple
              (broadcast-grade) iono/tropo model -> snapshot code least-squares
              -> metre-class PVT/PPS.
  BOTTOM lane — PPP: code + carrier phase, the SAME broadcast ephemeris, plus a
              real-time SSR correction stream, precise tropo (mapping + estimated
              ZTD), tide models, and antenna/relativity models -> sequential
              filter -> cm / sub-ns time.

Colour links the elements common to both flows (yellow = raw observations,
blue = broadcast nav message) and flags the PPP-only inputs (orange = real-time
precise corrections, green = precise physical models, grey = broadcast-grade
models).

The PPP lane makes the key relationship explicit: SSR orbit/clock are *deltas
applied on top of* the broadcast ephemeris (IODE-matched) — broadcast ⊕ SSR Δ
= precise orbit/clock.  PPP does not discard broadcast; code & phase biases are
absolute.

16:9, vector PDF + PNG.
"""
import os
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

# palette
C_OBS = "#f7dc6f"      # raw observations (shared)
C_BC = "#aed6f1"       # broadcast nav message (shared)
C_SIMPLE = "#d5d8dc"   # broadcast-grade models (SPP)
C_SSR = "#e59866"      # real-time precise corrections (PPP)
C_PRECISE = "#abebc0"  # precise physical models (PPP)
C_MERGE = "#d2b4de"    # broadcast ⊕ SSR = precise orbit/clock
C_EST = "#fef9e7"      # estimator
C_OUT_SPP = "#eaeded"
C_OUT_PPP = "#d4efdf"


def box(ax, xc, yc, w, h, text, fc, fs=9.5, ec="#333333", lw=1.6, weight="normal"):
    ax.add_patch(FancyBboxPatch((xc - w / 2, yc - h / 2), w, h,
                                boxstyle="round,pad=0.02,rounding_size=0.06",
                                fc=fc, ec=ec, lw=lw, zorder=3))
    ax.text(xc, yc, text, ha="center", va="center", fontsize=fs, zorder=4,
            weight=weight)


def feed(ax, p1, p2, color):
    """coloured connector line (no head) carrying a data block into the bus."""
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=color, lw=2.6,
            solid_capstyle="round", zorder=2)


def arrow(ax, p1, p2, color="#222222", lw=2.8):
    ax.annotate("", xy=p2, xytext=p1,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=lw,
                                shrinkA=2, shrinkB=2, mutation_scale=22), zorder=5)


def main():
    out_dir = sys.argv[sys.argv.index("--out-dir") + 1] if "--out-dir" in sys.argv else "."
    fig, ax = plt.subplots(figsize=(16, 9))
    ax.set_xlim(0, 16); ax.set_ylim(0, 9); ax.axis("off")

    # ── title + colour legend ───────────────────────────────────────────────
    ax.text(8.0, 8.72, "Two data flows, one antenna — SPP vs PPP",
            ha="center", va="center", fontsize=23, weight="bold")
    legend = [(C_OBS, "raw observations (shared)"),
              (C_BC, "broadcast nav message (shared)"),
              (C_SIMPLE, "broadcast-grade models (SPP)"),
              (C_SSR, "real-time precise corrections (PPP)"),
              (C_PRECISE, "precise physical models (PPP)")]
    lx = 0.5
    for fc, lab in legend:
        ax.add_patch(FancyBboxPatch((lx, 8.22), 0.32, 0.26,
                                    boxstyle="round,pad=0.01,rounding_size=0.04",
                                    fc=fc, ec="#555", lw=1.0, zorder=3))
        ax.text(lx + 0.42, 8.35, lab, ha="left", va="center", fontsize=9.3)
        lx += 0.42 + 0.072 * len(lab) + 0.45

    ax.axhline(4.95, 0.015, 0.985, color="#cccccc", lw=1.2, ls=(0, (4, 4)))

    # ── lane labels ─────────────────────────────────────────────────────────
    ax.text(0.32, 6.65, "SPP", rotation=90, ha="center", va="center",
            fontsize=18, weight="bold", color="#5d6d7e")
    ax.text(0.62, 6.65, "receiver-internal", rotation=90, ha="center",
            va="center", fontsize=10.5, color="#5d6d7e")
    ax.text(0.32, 2.55, "PPP", rotation=90, ha="center", va="center",
            fontsize=18, weight="bold", color="#117a65")
    ax.text(0.62, 2.55, "external engine + streams", rotation=90, ha="center",
            va="center", fontsize=10.5, color="#117a65")

    IXC, IW = 3.0, 3.3            # input column centre / width

    # ════ SPP lane ══════════════════════════════════════════════════════════
    spp_in = [
        (7.70, C_OBS, "Code pseudorange  (PR)"),
        (7.00, C_BC, "Broadcast ephemeris\n+ satellite clock"),
        (6.30, C_SIMPLE, "Ionosphere:  Klobuchar\n(broadcast coefficients)"),
        (5.60, C_SIMPLE, "Troposphere:  Saastamoinen\n(standard atmosphere)"),
    ]
    for y, fc, txt in spp_in:
        box(ax, IXC, y, IW, 0.58, txt, fc, fs=9.3)
    bus_x = 6.25
    ax.plot([bus_x, bus_x], [5.60, 7.70], color="#888", lw=2.0, zorder=1)
    for y, fc, _ in spp_in:
        feed(ax, (IXC + IW / 2, y), (bus_x, y), fc)
    box(ax, 8.7, 6.65, 3.0, 1.15,
        "Single-Point Positioning\ncode least-squares · snapshot", C_EST,
        fs=11, weight="bold", lw=2.0)
    arrow(ax, (bus_x, 6.65), (7.2, 6.65))
    box(ax, 12.7, 6.65, 2.7, 1.0, "PVT  +  PPS\n≈ 1–5 m · tens of ns",
        C_OUT_SPP, fs=11.5, weight="bold", lw=2.0)
    arrow(ax, (10.2, 6.65), (11.35, 6.65))

    # ════ PPP lane ══════════════════════════════════════════════════════════
    ppp_in = [
        (4.32, C_OBS, "Code + carrier phase  (PR, φ)"),
        (3.64, C_BC, "Broadcast ephemeris\n+ satellite clock"),
        (2.96, C_SSR, "SSR stream (NTRIP):\norbit Δ · clock Δ · code/phase bias"),
        (2.28, C_PRECISE, "Troposphere:  VMF/GMF\nmapping + estimated ZTD"),
        (1.60, C_PRECISE, "Tide models:  solid Earth ·\nocean loading · pole"),
        (0.92, C_PRECISE, "Antenna PCO/PCV ·\nphase wind-up · relativity"),
    ]
    for y, fc, txt in ppp_in:
        box(ax, IXC, y, IW, 0.58, txt, fc, fs=9.0)

    # broadcast ⊕ SSR -> precise orbit/clock merge node
    mnode = (6.05, 3.30)
    box(ax, mnode[0], mnode[1], 1.6, 1.0,
        "Precise satellite:\norbit · clock · biases\n(broadcast + SSR)", C_MERGE,
        fs=8.6, weight="bold", lw=1.8)
    feed(ax, (IXC + IW / 2, 3.64), (mnode[0] - 0.75, 3.55), C_BC)
    feed(ax, (IXC + IW / 2, 2.96), (mnode[0] - 0.75, 3.05), C_SSR)
    ax.text(mnode[0] - 0.95, 3.30, "⊕", ha="center", va="center", fontsize=15,
            weight="bold", color="#6c3483", zorder=6)

    bus_xp = 7.05
    ax.plot([bus_xp, bus_xp], [0.92, 4.32], color="#888", lw=2.0, zorder=1)
    feed(ax, (IXC + IW / 2, 4.32), (bus_xp, 4.32), C_OBS)            # obs
    feed(ax, (mnode[0] + 0.75, 3.30), (bus_xp, 3.30), C_MERGE)      # merged orb/clk
    for y in (2.28, 1.60, 0.92):                                    # precise models
        feed(ax, (IXC + IW / 2, y), (bus_xp, y), C_PRECISE)

    box(ax, 9.55, 2.62, 3.5, 1.6,
        "Sequential PPP filter  (Kalman)\nposition · clock · ZTD · ambiguities\n"
        "float  →  ambiguity-resolved", C_EST, fs=10.5, weight="bold", lw=2.0)
    arrow(ax, (bus_xp, 2.62), (7.8, 2.62))
    box(ax, 13.3, 2.62, 2.8, 1.15, "Precise clock & time\n≈ cm · sub-ns",
        C_OUT_PPP, fs=11.5, weight="bold", lw=2.2)
    arrow(ax, (11.3, 2.62), (11.9, 2.62))

    # ── shared-colour reminder + the broadcast⊕SSR footnote ─────────────────
    ax.text(8.0, 0.42,
            "Matching colours = the same data feeding both flows: PPP is built "
            "on the broadcast nav message, not instead of it.",
            ha="center", va="center", fontsize=10.5, color="#555", style="italic")
    ax.text(8.0, 0.13,
            "SSR orbit/clock are Δ-corrections applied on top of the broadcast "
            "ephemeris (IODE-matched) — precise = broadcast ⊕ SSR Δ;  code & "
            "phase biases are absolute (applied to the observations).",
            ha="center", va="center", fontsize=10.5, color="#333")

    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "dataflow_spp_vs_ppp")
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png")


if __name__ == "__main__":
    main()

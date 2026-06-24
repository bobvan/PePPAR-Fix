#!/usr/bin/env python3
"""draw_dataflow_spp_vs_ppp.py — SPP-vs-PPP *data-flow* slide (16:9).

Companion to draw_receiver_block_diagram.py ("one set of raw observations ->
two timing solutions").  Where that slide shows the hardware split, this one
follows the *data*: what each estimator consumes.

  TOP lane  — SPP: code pseudorange + broadcast ephemeris/clock + a *simple*
              (broadcast-grade) atmosphere model, with NO tide or antenna
              models -> snapshot code least-squares -> metre-class PVT/PPS.
  BOTTOM lane — PPP: code + carrier phase, the SAME broadcast ephemeris, plus a
              real-time SSR correction stream and *advanced, location-specific*
              models (estimated troposphere, Earth-tide, antenna, wind-up,
              relativity) -> sequential filter -> cm / sub-ns time.

Both data flows start at one antenna (observations + broadcast nav are decoded
from the live signal); the SSR stream arrives separately from an internet
caster.  Colour links the elements common to both flows.

Sized for projection: large fonts, few boxes.

16:9, vector PDF + PNG.
"""
import os
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon, Circle, Ellipse, Arc

# palette
C_OBS = "#f7dc6f"      # raw observations (shared)
C_BC = "#aed6f1"       # broadcast nav message (shared)
C_SIMPLE = "#d5d8dc"   # simple / broadcast-grade models (SPP)
C_SSR = "#e59866"      # real-time precise corrections (PPP)
C_ADV = "#abebc0"      # advanced, location-specific models (PPP)
C_MERGE = "#d2b4de"    # broadcast + SSR = precise satellite
C_EST = "#fef9e7"      # estimator
C_OUT_SPP = "#eaeded"
C_OUT_PPP = "#d4efdf"


def box(ax, xc, yc, w, h, text, fc, fs=12, ec="#333333", lw=1.8, weight="normal"):
    ax.add_patch(FancyBboxPatch((xc - w / 2, yc - h / 2), w, h,
                                boxstyle="round,pad=0.02,rounding_size=0.06",
                                fc=fc, ec=ec, lw=lw, zorder=3))
    ax.text(xc, yc, text, ha="center", va="center", fontsize=fs, zorder=4,
            weight=weight)


def feed(ax, p1, p2, color):
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=color, lw=3.0,
            solid_capstyle="round", zorder=2)


def arrow(ax, p1, p2, color="#222222", lw=3.0):
    ax.annotate("", xy=p2, xytext=p1,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=lw,
                                shrinkA=2, shrinkB=2, mutation_scale=24), zorder=5)


def thin_arrow(ax, p1, p2, color="#666666"):
    ax.annotate("", xy=p2, xytext=p1,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.8,
                                shrinkA=1, shrinkB=2, mutation_scale=15), zorder=4)


def draw_antenna(ax, cx, cy, s=1.0):
    ax.plot([cx, cx], [cy - 0.5 * s, cy + 0.22 * s], color="#333", lw=3.0, zorder=4)
    ax.add_patch(Polygon([[cx - 0.17 * s, cy + 0.22 * s], [cx + 0.17 * s, cy + 0.22 * s],
                          [cx, cy + 0.6 * s]], closed=True, fc="#cfd2d4",
                         ec="#333", lw=1.8, zorder=4))
    ax.plot([cx - 0.22 * s, cx + 0.22 * s], [cy - 0.5 * s, cy - 0.5 * s],
            color="#333", lw=3.0, zorder=4)
    for r in (0.34, 0.52, 0.70):
        ax.add_patch(Arc((cx, cy + 0.6 * s), r * s, r * s, angle=0,
                         theta1=18, theta2=72, color="#2e86c1", lw=2.0, zorder=3))
    ax.text(cx, cy - 0.66 * s, "GNSS\nantenna", ha="center", va="top",
            fontsize=11.5, weight="bold")


def draw_cloud(ax, cx, cy, s=1.0, fc="#eaf2fb"):
    for dx, dy, r in [(-0.42, 0.02, 0.34), (-0.02, 0.14, 0.42),
                      (0.42, 0.04, 0.32), (0.0, -0.12, 0.5)]:
        ax.add_patch(Circle((cx + dx * s, cy + dy * s), r * s, fc=fc, ec="none",
                            zorder=3))
    ax.add_patch(Ellipse((cx, cy - 0.2 * s), 1.7 * s, 0.5 * s, fc=fc, ec="none",
                         zorder=3))
    ax.text(cx, cy + 0.02 * s, "Internet\nSSR caster", ha="center", va="center",
            fontsize=10.5, zorder=5, weight="bold")


def main():
    out_dir = sys.argv[sys.argv.index("--out-dir") + 1] if "--out-dir" in sys.argv else "."
    fig, ax = plt.subplots(figsize=(16, 9))
    ax.set_xlim(0, 16); ax.set_ylim(0, 9); ax.axis("off")

    # ── title + colour legend ───────────────────────────────────────────────
    ax.text(8.0, 8.70, "Two data flows, one antenna — SPP vs PPP",
            ha="center", va="center", fontsize=25, weight="bold")
    legend = [(C_OBS, "observations (shared)"),
              (C_BC, "broadcast nav (shared)"),
              (C_SIMPLE, "simple models — SPP"),
              (C_SSR, "precise corrections — PPP"),
              (C_ADV, "advanced models — PPP")]
    lx = 0.6
    for fc, lab in legend:
        ax.add_patch(FancyBboxPatch((lx, 8.16), 0.34, 0.28,
                                    boxstyle="round,pad=0.01,rounding_size=0.04",
                                    fc=fc, ec="#555", lw=1.0, zorder=3))
        ax.text(lx + 0.46, 8.30, lab, ha="left", va="center", fontsize=11)
        lx += 0.46 + 0.075 * len(lab) + 0.55

    ax.axhline(4.95, 0.02, 0.98, color="#cccccc", lw=1.2, ls=(0, (4, 4)))

    # ── lane labels ─────────────────────────────────────────────────────────
    ax.text(0.30, 6.70, "SPP", rotation=90, ha="center", va="center",
            fontsize=20, weight="bold", color="#5d6d7e")
    ax.text(0.30, 2.55, "PPP", rotation=90, ha="center", va="center",
            fontsize=20, weight="bold", color="#117a65")

    # ── data-flow sources (the start) ───────────────────────────────────────
    draw_antenna(ax, 1.15, 5.15, s=1.05)
    draw_cloud(ax, 1.2, 2.2, s=1.0)

    BXL = 2.15                    # input-box left edge
    IXC, IW = 3.55, 2.8          # input column centre / width

    # ════ SPP lane ══════════════════════════════════════════════════════════
    spp = [
        (7.70, C_OBS, "Code pseudorange  (PR)", 12),
        (6.70, C_BC, "Broadcast ephemeris\n& satellite clock", 12),
        (5.65, C_SIMPLE,
         "Simple models\n(broadcast-grade atmosphere)\nno tide · no antenna models", 11),
    ]
    for y, fc, txt, fs in spp:
        box(ax, IXC, y, IW, 0.92, txt, fc, fs=fs)
    bus_x = 6.85
    ax.plot([bus_x, bus_x], [5.65, 7.70], color="#888", lw=2.2, zorder=1)
    for y, fc, _, _ in spp:
        feed(ax, (IXC + IW / 2, y), (bus_x, y), fc)
    box(ax, 8.85, 6.68, 2.9, 1.25,
        "Single-Point Positioning\ncode least-squares · snapshot", C_EST,
        fs=13, weight="bold", lw=2.2)
    arrow(ax, (bus_x, 6.68), (7.4, 6.68))
    box(ax, 12.85, 6.68, 2.8, 1.15, "PVT  +  PPS\n≈ 1–5 m · tens of ns",
        C_OUT_SPP, fs=14, weight="bold", lw=2.2)
    arrow(ax, (10.3, 6.68), (11.45, 6.68))

    # ════ PPP lane ══════════════════════════════════════════════════════════
    box(ax, IXC, 4.15, IW, 0.86, "Code + carrier phase  (PR, φ)", C_OBS, fs=12)
    box(ax, IXC, 3.20, IW, 0.86, "Broadcast ephemeris\n& satellite clock", C_BC, fs=12)
    box(ax, IXC, 2.25, IW, 0.86,
        "SSR precise corrections (NTRIP)\norbit Δ · clock Δ · code/phase bias",
        C_SSR, fs=10.5)
    box(ax, IXC, 1.10, IW, 1.0,
        "Advanced, location-specific models\nestimated troposphere (ZTD) ·\n"
        "tide · antenna · wind-up · relativity", C_ADV, fs=10.5)

    # broadcast + SSR -> precise-satellite merge node
    mnode = (5.85, 2.72)
    box(ax, mnode[0], mnode[1], 1.6, 1.05,
        "Precise satellite\norbit · clock · biases\n(broadcast + SSR)", C_MERGE,
        fs=10, weight="bold", lw=2.0)
    feed(ax, (IXC + IW / 2, 3.20), (mnode[0] - 0.8, 3.05), C_BC)
    feed(ax, (IXC + IW / 2, 2.25), (mnode[0] - 0.8, 2.40), C_SSR)
    ax.text(mnode[0] - 0.95, 2.72, "+", ha="center", va="center", fontsize=20,
            weight="bold", color="#6c3483", zorder=6)

    bus_xp = 6.85
    ax.plot([bus_xp, bus_xp], [1.10, 4.15], color="#888", lw=2.2, zorder=1)
    feed(ax, (IXC + IW / 2, 4.15), (bus_xp, 4.15), C_OBS)
    feed(ax, (mnode[0] + 0.8, 2.72), (bus_xp, 2.72), C_MERGE)
    feed(ax, (IXC + IW / 2, 1.10), (bus_xp, 1.10), C_ADV)
    box(ax, 9.15, 2.55, 3.3, 1.7,
        "Sequential PPP filter  (Kalman)\nposition · clock · ZTD · ambiguities\n"
        "float  →  ambiguity-resolved", C_EST, fs=12, weight="bold", lw=2.2)
    arrow(ax, (bus_xp, 2.55), (7.5, 2.55))
    box(ax, 13.2, 2.55, 2.8, 1.3, "Precise clock & time\n≈ cm · sub-ns",
        C_OUT_PPP, fs=14, weight="bold", lw=2.4)
    arrow(ax, (10.8, 2.55), (11.8, 2.55))

    # ── source -> box arrows (everything starts at the antenna / caster) ────
    for y in (7.70, 6.70, 4.15, 3.20):       # obs + broadcast eph, both lanes
        thin_arrow(ax, (1.62, 5.15), (BXL - 0.02, y))
    thin_arrow(ax, (1.95, 2.2), (BXL - 0.02, 2.25))   # caster -> SSR

    # ── footnotes ───────────────────────────────────────────────────────────
    ax.text(8.2, 0.40,
            "Matching colours = the same data feeding both flows — PPP builds on "
            "the broadcast nav message, not instead of it.",
            ha="center", va="center", fontsize=12, color="#555", style="italic")
    ax.text(8.2, 0.12,
            "SSR orbit/clock are Δ-corrections applied on top of broadcast "
            "(IODE-matched);  code & phase biases are absolute.",
            ha="center", va="center", fontsize=12, color="#333")

    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "dataflow_spp_vs_ppp")
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png")


if __name__ == "__main__":
    main()

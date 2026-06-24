#!/usr/bin/env python3
"""draw_dataflow_spp_vs_ppp.py — SPP-vs-PPP *data-flow* slide (16:9).

Companion to draw_receiver_block_diagram.py ("one set of raw observations ->
two timing solutions").  Where that slide shows the hardware split, this one
follows the *data*: what each estimator consumes.

  TOP lane  — SPP: code pseudorange + broadcast ephemeris/clock + a *simple*
              (broadcast-grade) atmosphere model, with NO tide or antenna
              models -> snapshot code least-squares -> metre-class PVT/PPS,
              available immediately (single epoch).
  BOTTOM lane — PPP: code + carrier phase, the SAME broadcast ephemeris, plus a
              real-time precise-correction stream and *advanced, location-
              specific* models (estimated troposphere, tide, antenna, wind-up,
              relativity) -> sequential filter that takes tens of minutes to
              converge -> cm / sub-ns time.

Sources: one antenna (observations + broadcast nav, decoded from the live
signal) and the correction streams.  Precise corrections arrive as NTRIP SSR
(internet) or Galileo HAS (dashed) — HAS either over the E6-B signal (via the
antenna) or over IDD (internet).

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
C_HAS = "#138d75"      # Galileo HAS path (dashed)
C_LOOP = "#b9770e"     # convergence-time feedback loop / clock
DASH = (0, (5, 3))


def box(ax, xc, yc, w, h, text, fc, fs=12, ec="#333333", lw=1.8, weight="normal"):
    ax.add_patch(FancyBboxPatch((xc - w / 2, yc - h / 2), w, h,
                                boxstyle="round,pad=0.02,rounding_size=0.06",
                                fc=fc, ec=ec, lw=lw, zorder=3))
    ax.text(xc, yc, text, ha="center", va="center", fontsize=fs, zorder=4,
            weight=weight)


def feed(ax, p1, p2, color):
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=color, lw=3.0,
            solid_capstyle="round", zorder=2)


def arrow(ax, p1, p2, color="#222222", lw=3.0, ls="-"):
    ax.annotate("", xy=p2, xytext=p1,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, linestyle=ls,
                                shrinkA=2, shrinkB=2, mutation_scale=24), zorder=5)


def thin_arrow(ax, p1, p2, color="#666666", ls="-"):
    ax.annotate("", xy=p2, xytext=p1,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.8, linestyle=ls,
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
    ax.text(cx, cy - 0.62 * s, "GNSS\nantenna", ha="center", va="top",
            fontsize=11, weight="bold")


def draw_cloud(ax, cx, cy, label, s=0.72, fc="#eaf2fb"):
    for dx, dy, r in [(-0.42, 0.02, 0.34), (-0.02, 0.14, 0.42),
                      (0.42, 0.04, 0.32), (0.0, -0.12, 0.5)]:
        ax.add_patch(Circle((cx + dx * s, cy + dy * s), r * s, fc=fc, ec="none",
                            zorder=3))
    ax.add_patch(Ellipse((cx, cy - 0.2 * s), 1.7 * s, 0.5 * s, fc=fc, ec="none",
                         zorder=3))
    ax.text(cx, cy + 0.0 * s, label, ha="center", va="center", fontsize=10,
            zorder=5, weight="bold")


def draw_clock(ax, cx, cy, r=0.22):
    ax.add_patch(Circle((cx, cy), r, fc="white", ec=C_LOOP, lw=2.4, zorder=7))
    ax.plot([cx, cx], [cy, cy + r * 0.62], color=C_LOOP, lw=2.2, zorder=8)
    ax.plot([cx, cx + r * 0.5], [cy, cy + r * 0.08], color=C_LOOP, lw=2.2, zorder=8)
    ax.add_patch(Circle((cx, cy), r * 0.09, fc=C_LOOP, ec="none", zorder=9))


def main():
    out_dir = sys.argv[sys.argv.index("--out-dir") + 1] if "--out-dir" in sys.argv else "."
    fig, ax = plt.subplots(figsize=(16, 9))
    ax.set_xlim(0, 16); ax.set_ylim(0, 9); ax.axis("off")

    # ── title + colour legend ───────────────────────────────────────────────
    ax.text(8.0, 8.72, "Two data flows — SPP vs PPP",
            ha="center", va="center", fontsize=25, weight="bold")
    legend = [(C_OBS, "observations (shared)"),
              (C_BC, "broadcast nav (shared)"),
              (C_SIMPLE, "simple models — SPP"),
              (C_SSR, "precise corrections — PPP"),
              (C_ADV, "advanced models — PPP")]
    lx = 0.6
    for fc, lab in legend:
        ax.add_patch(FancyBboxPatch((lx, 8.20), 0.32, 0.26,
                                    boxstyle="round,pad=0.01,rounding_size=0.04",
                                    fc=fc, ec="#555", lw=1.0, zorder=3))
        ax.text(lx + 0.44, 8.33, lab, ha="left", va="center", fontsize=10.5)
        lx += 0.44 + 0.072 * len(lab) + 0.5
    # dashed HAS key
    ax.plot([lx, lx + 0.5], [8.33, 8.33], color=C_HAS, lw=2.2, ls=DASH, zorder=3)
    ax.text(lx + 0.6, 8.33, "Galileo HAS", ha="left", va="center", fontsize=10.5,
            color=C_HAS)

    ax.axhline(4.95, 0.02, 0.98, color="#cccccc", lw=1.2, ls=(0, (4, 4)))

    ax.text(0.30, 6.55, "SPP", rotation=90, ha="center", va="center",
            fontsize=20, weight="bold", color="#5d6d7e")
    ax.text(0.30, 2.55, "PPP", rotation=90, ha="center", va="center",
            fontsize=20, weight="bold", color="#117a65")

    # ── sources ─────────────────────────────────────────────────────────────
    draw_antenna(ax, 1.30, 5.55, s=1.0)
    draw_cloud(ax, 1.35, 3.20, "Internet\nNTRIP")
    draw_cloud(ax, 1.35, 1.45, "Internet\nHAS · IDD")

    BXL, IXC, IW = 2.30, 3.65, 2.70      # box left edge / centre / width

    # ════ SPP lane ══════════════════════════════════════════════════════════
    spp = [
        (7.50, 0.78, C_OBS, "Code pseudorange  (PR)", 12),
        (6.55, 0.78, C_BC, "Broadcast ephemeris\n& satellite clock", 12),
        (5.55, 0.88, C_SIMPLE,
         "Simple models\n(broadcast-grade atmosphere)\nno tide · no antenna models", 11),
    ]
    for y, h, fc, txt, fs in spp:
        box(ax, IXC, y, IW, h, txt, fc, fs=fs)
    bus_x = 7.05
    ax.plot([bus_x, bus_x], [5.55, 7.50], color="#888", lw=2.2, zorder=1)
    for y, _, fc, _, _ in spp:
        feed(ax, (IXC + IW / 2, y), (bus_x, y), fc)
    box(ax, 8.65, 6.50, 2.6, 1.2,
        "Single-Point Positioning\ncode least-squares\n(snapshot)", C_EST,
        fs=12.5, weight="bold", lw=2.2)
    arrow(ax, (bus_x, 6.50), (7.35, 6.50))
    box(ax, 11.95, 6.50, 2.7, 1.1, "PVT  +  PPS\n≈ 1–5 m · tens of ns",
        C_OUT_SPP, fs=13.5, weight="bold", lw=2.2)
    arrow(ax, (9.95, 6.50), (10.6, 6.50))
    ax.text(11.95, 5.74, "available immediately  (1 epoch)", ha="center",
            va="center", fontsize=10.5, color="#1e8449", style="italic")

    # ════ PPP lane ══════════════════════════════════════════════════════════
    box(ax, IXC, 4.20, IW, 0.78, "Code + carrier phase  (PR, φ)", C_OBS, fs=12)
    box(ax, IXC, 3.30, IW, 0.78, "Broadcast ephemeris\n& satellite clock", C_BC, fs=12)
    box(ax, IXC, 2.40, IW, 0.84,
        "Precise corrections (SSR · HAS)\norbit Δ · clock Δ · code/phase bias",
        C_SSR, fs=10.5)
    box(ax, IXC, 1.25, IW, 0.90,
        "Advanced, location-specific models\nestimated troposphere (ZTD) ·\n"
        "tide · antenna · wind-up · relativity", C_ADV, fs=10.5)

    BXR = IXC + IW / 2                                    # box right edge = 5.0
    # broadcast + precise corr -> precise-satellite merge node
    mnode = (6.22, 2.85)
    box(ax, mnode[0], mnode[1], 1.84, 1.1,
        "Precise satellite\norbit · clock · biases\n(broadcast + SSR)", C_MERGE,
        fs=9.5, weight="bold", lw=2.0)
    feed(ax, (BXR, 3.30), (mnode[0] - 0.92, 3.10), C_BC)
    feed(ax, (BXR, 2.40), (mnode[0] - 0.92, 2.55), C_SSR)
    ax.text(5.18, 2.85, "+", ha="center", va="center", fontsize=22,
            weight="bold", color="#6c3483", zorder=6)

    bus_xp = 7.28
    ax.plot([bus_xp, bus_xp], [1.25, 4.20], color="#888", lw=2.2, zorder=1)
    feed(ax, (BXR, 4.20), (bus_xp, 4.20), C_OBS)
    feed(ax, (mnode[0] + 0.92, 2.85), (bus_xp, 2.85), C_MERGE)
    feed(ax, (BXR, 1.25), (bus_xp, 1.25), C_ADV)
    box(ax, 9.10, 2.60, 3.2, 1.65,
        "Sequential PPP filter  (Kalman)\nposition · clock · ZTD ·\n"
        "ambiguities  (float → fixed)", C_EST, fs=11.5, weight="bold", lw=2.2)
    arrow(ax, (bus_xp, 2.60), (7.50, 2.60))
    box(ax, 12.60, 2.60, 2.8, 1.25, "Precise clock & time\n≈ cm · sub-ns",
        C_OUT_PPP, fs=13.5, weight="bold", lw=2.4)
    arrow(ax, (10.70, 2.60), (11.20, 2.60))

    # convergence-time feedback loop (output -> input through a clock)
    ax.plot([9.85, 9.85], [1.78, 1.15], color=C_LOOP, lw=2.2, zorder=4)
    ax.plot([9.85, 8.10], [1.15, 1.15], color=C_LOOP, lw=2.2, zorder=4)
    arrow(ax, (8.10, 1.15), (8.10, 1.76), color=C_LOOP, lw=2.2)
    draw_clock(ax, 8.97, 1.15, r=0.21)
    ax.text(9.0, 0.74, "elapsed time:  tens of minutes to converge",
            ha="center", va="center", fontsize=10.5, color=C_LOOP, weight="bold")

    # ── source -> box arrows ────────────────────────────────────────────────
    for y in (7.50, 6.55, 4.20, 3.30):       # antenna -> obs + broadcast eph
        thin_arrow(ax, (1.78, 5.40), (BXL - 0.02, y))
    thin_arrow(ax, (1.85, 3.05), (BXL - 0.02, 2.52))            # NTRIP -> precise (solid)
    thin_arrow(ax, (1.85, 1.60), (BXL - 0.02, 2.30), C_HAS, ls=DASH)   # HAS IDD -> precise
    # HAS via E6-B: from the antenna, dashed, down the left margin into the box
    ax.plot([1.30, 0.66, 0.66], [5.05, 5.05, 2.6], color=C_HAS, lw=2.0,
            ls=DASH, zorder=2)
    thin_arrow(ax, (0.66, 2.60), (BXL - 0.02, 2.46), C_HAS, ls=DASH)
    ax.text(0.5, 3.7, "HAS · E6-B", rotation=90, ha="center", va="center",
            fontsize=9.5, color=C_HAS, weight="bold")

    # ── footnotes ───────────────────────────────────────────────────────────
    ax.text(8.2, 0.40,
            "Matching colours = the same data feeding both flows — PPP builds on "
            "the broadcast nav message, not instead of it.",
            ha="center", va="center", fontsize=11.5, color="#555", style="italic")
    ax.text(8.2, 0.13,
            "SSR/HAS orbit/clock are Δ-corrections applied on top of broadcast "
            "(IODE-matched);  code & phase biases are absolute.",
            ha="center", va="center", fontsize=11.5, color="#333")

    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "dataflow_spp_vs_ppp")
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png")


if __name__ == "__main__":
    main()

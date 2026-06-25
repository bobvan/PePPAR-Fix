#!/usr/bin/env python3
"""draw_dofreqest_wiring.py — DOFreqEst state/arm wiring schematic (16:9).

A clearer alternative to dofreqest_viz.svg / dofreqest-state-arm-map.svg.

Two big state boxes — the rx TCXO and the DO — each bisected by a dotted line
into a PHASE half and a FREQUENCY half (the four EKF states x0..x3).  Both are
enclosed by the DOFreqEst filter (4-state EKF + LQR).  Around them sit the
physical signal chain (antenna → GNSS receiver → DO → TICC / ClockMatrix) and
the PPP engine, wired in by the SEVEN measurement arms; the LQR control output
is the actuation that steers the DO (the servo loop), not an arm:

  x0 φ_rx (rx phase)   ← Arm 1 PPP dt_rx
  x1 f_rx (rx freq)    ← Arm 2 qErr, Arm 5 TDCP
  x2 φ_do (DO phase)   ← Arm 3 EXTINT, Arm 4 TICC (couples x0), Arm 6 holdover,
                          Arm 7 cm_phase (ClockMatrix PFD — Timebeat OTC only)
  x3 f_do (DO freq)    ← no arm — hidden, recovered through the predict step
  LQR reads x2,x3 → adjfine / DAC / ClockMatrix combo bus → steers the DO

Arm 7 (cm_phase, added for Timebeat OTC — otcBob1 / ptBoat) reads the DO-vs-F9T
phase straight off the ClockMatrix DPLL's on-chip phase-frequency detector
(~50 ps) over I²C — same observation state as EXTINT (x2), but with no TICC and
no EXTINT wire, and the same chip steers the DO via the combo bus.

Predict each epoch folds frequency into phase: φ += f·dt (rx) and
φ_do −= (f_do+adjfine)·dt (DO), which is how the hidden x3 becomes observable.

16:9, vector PDF + PNG.
"""
import os
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon, Arc, FancyArrowPatch

C_RX = "#2e86c1"       # rx-TCXO arms / box
C_DO = "#1e8449"       # DO arms / box
C_CTRL = "#b9560f"     # control (LQR → actuator)
C_HW = "#eaecee"       # hardware boxes
C_CM = "#0e6655"       # ClockMatrix (Timebeat OTC variant)
C_PPP = "#e8daef"      # PPP engine
C_FILT = "#fbfcfd"     # filter background
C_COUP = "#7d3c98"     # TICC coupling / predict


def box(ax, x0, y0, x1, y1, text, fc, fs=10, ec="#333", lw=1.6, ls="-",
        weight="normal", tva="center", ty=None):
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0,
                                boxstyle="round,pad=0.02,rounding_size=0.05",
                                fc=fc, ec=ec, lw=lw, ls=ls, zorder=3))
    ax.text((x0 + x1) / 2, ty if ty is not None else (y0 + y1) / 2, text,
            ha="center", va=tva, fontsize=fs, zorder=4, weight=weight)


def arrow(ax, p1, p2, color="#333", lw=2.0, ls="-", z=5):
    ax.annotate("", xy=p2, xytext=p1,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, linestyle=ls,
                                shrinkA=2, shrinkB=2, mutation_scale=18), zorder=z)


def elbow(ax, pts, color, lw=2.0, ls="-"):
    for i in range(len(pts) - 2):
        ax.plot([pts[i][0], pts[i + 1][0]], [pts[i][1], pts[i + 1][1]],
                color=color, lw=lw, ls=ls, solid_capstyle="round", zorder=4)
    arrow(ax, pts[-2], pts[-1], color=color, lw=lw, ls=ls)


def label(ax, x, y, text, color, fs=9):
    ax.text(x, y, text, ha="center", va="center", fontsize=fs, color=color,
            weight="bold", zorder=8,
            bbox=dict(boxstyle="round,pad=0.18", fc="white", ec="none", alpha=0.92))


def draw_antenna(ax, cx, cy, s=1.0):
    ax.plot([cx, cx], [cy - 0.4 * s, cy + 0.18 * s], color="#333", lw=2.4, zorder=4)
    ax.add_patch(Polygon([[cx - 0.14 * s, cy + 0.18 * s], [cx + 0.14 * s, cy + 0.18 * s],
                          [cx, cy + 0.5 * s]], closed=True, fc="#cfd2d4", ec="#333",
                         lw=1.5, zorder=4))
    ax.plot([cx - 0.18 * s, cx + 0.18 * s], [cy - 0.4 * s, cy - 0.4 * s],
            color="#333", lw=2.4, zorder=4)
    for r in (0.28, 0.44):
        ax.add_patch(Arc((cx, cy + 0.5 * s), r * s, r * s, theta1=20, theta2=70,
                         color="#2e86c1", lw=1.6, zorder=3))


def statebox(ax, x0, y0, x1, y1, title, fc, left, right):
    """A state box bisected into phase (left) / frequency (right) halves."""
    box(ax, x0, y0, x1, y1, "", fc, lw=2.2)
    xm = (x0 + x1) / 2
    ax.plot([xm, xm], [y0 + 0.06, y1 - 0.06], color="#555", lw=1.4,
            ls=(0, (3, 3)), zorder=4)
    ax.text(xm, y1 + 0.16, title, ha="center", va="bottom", fontsize=11.5,
            weight="bold", zorder=5)
    ax.text((x0 + xm) / 2, (y0 + y1) / 2, left, ha="center", va="center",
            fontsize=10, zorder=5)
    ax.text((xm + x1) / 2, (y0 + y1) / 2, right, ha="center", va="center",
            fontsize=10, zorder=5)
    return xm


def main():
    out_dir = sys.argv[sys.argv.index("--out-dir") + 1] if "--out-dir" in sys.argv else "."
    fig, ax = plt.subplots(figsize=(16, 9))
    ax.set_xlim(0, 16); ax.set_ylim(0, 9); ax.axis("off")

    ax.text(8.0, 8.68, "DOFreqEst — two oscillators, four states, seven measurement arms",
            ha="center", va="center", fontsize=21, weight="bold")

    # ── filter enclosure ────────────────────────────────────────────────────
    box(ax, 5.0, 0.7, 15.6, 7.95, "", C_FILT, lw=2.6, ec="#34495e")
    ax.text(5.2, 7.7, "DOFreqEst  ·  4-state EKF + LQR", ha="left", va="center",
            fontsize=13, weight="bold", color="#34495e")

    # ── two state boxes ─────────────────────────────────────────────────────
    rxm = statebox(ax, 6.6, 5.0, 11.2, 6.7, "rx TCXO  (estimated state)", "#d6eaf8",
                   "x0\nφ_rx\nphase\n(ns)", "x1\nf_rx\nfrequency\n(ppb)")
    dom = statebox(ax, 6.6, 2.2, 11.2, 3.9, "DO  (estimated state)", "#d5f0e0",
                   "x2\nφ_do\nphase\n(ns)", "x3\nf_do\nfrequency\n(ppb)*")

    # predict: frequency folds into phase each epoch (curved arrow R→L inside)
    for (y0, y1, txt, col) in ((5.0, 6.7, "φ_rx += f_rx·dt", C_RX),
                               (2.2, 3.9, "φ_do −= (f_do+adjfine)·dt", C_DO)):
        ax.add_patch(FancyArrowPatch((9.6, y1 - 0.18), (8.2, y1 - 0.18),
                     connectionstyle="arc3,rad=-0.5", arrowstyle="-|>",
                     mutation_scale=14, lw=1.6, color="#7f8c8d", zorder=6))
    ax.text(8.9, 4.62, "predict: φ += f·dt", ha="center", fontsize=8.5,
            color="#7f8c8d", style="italic", zorder=7)
    ax.text(8.9, 1.92, "predict folds f_do (+adjfine) into φ_do", ha="center",
            fontsize=8.5, color="#7f8c8d", style="italic", zorder=7)

    # TICC coupling between φ_rx (x0) and φ_do (x2)
    ax.add_patch(FancyArrowPatch((7.7, 5.0), (7.7, 3.9),
                 connectionstyle="arc3,rad=0.0", arrowstyle="<|-|>",
                 mutation_scale=14, lw=1.8, color=C_COUP, ls=(0, (4, 2)), zorder=6))
    label(ax, 7.7, 4.45, "couples\nφ_rx ↔ φ_do", C_COUP, fs=8)

    # ── LQR control + actuation ─────────────────────────────────────────────
    box(ax, 12.0, 2.45, 14.9, 3.65,
        "LQR control\nreads x2, x3\n→ actuator", "#fdebd0", fs=10,
        weight="bold", lw=1.8, ec=C_CTRL)
    arrow(ax, (11.2, 3.05), (12.0, 3.05), color=C_CTRL, lw=2.4)
    label(ax, 11.6, 3.32, "control", C_CTRL, fs=8.5)

    # ── physical hardware (left column) ─────────────────────────────────────
    draw_antenna(ax, 0.95, 7.55, s=1.0)
    box(ax, 0.55, 6.25, 4.55, 7.05,
        "GNSS receiver (F9T)\nraw obs · gnss_pps · qErr · EXTINT-in", C_HW, fs=9.5,
        weight="bold")
    box(ax, 0.55, 5.25, 4.55, 5.95,
        "PPP engine\ndt_rx (rx phase) · TDCP (rx freq)", C_PPP, fs=9.5, weight="bold")
    box(ax, 0.55, 4.30, 4.55, 4.95, "TICC\nchA do_pps − chB gnss_pps", C_HW, fs=9.5,
        weight="bold")
    box(ax, 0.55, 2.95, 4.55, 3.65,
        "ClockMatrix DPLL  (Timebeat OTC)\non-chip PFD: do_pps vs gnss_pps (~50 ps)\n"
        "· also the combo-bus actuator", "#d0ece7", fs=8.3, ec=C_CM, lw=1.7,
        ls=(0, (5, 2)), weight="bold")
    box(ax, 0.55, 1.55, 4.55, 2.25, "Disciplined Oscillator (DO)\n→ do_pps    ← steer",
        "#d5f0e0", fs=9.5, weight="bold")

    arrow(ax, (0.95, 7.35), (0.95, 7.07), color="#333", lw=1.8)          # antenna->rx
    arrow(ax, (1.4, 6.25), (1.4, 5.97), color="#555", lw=1.6)            # raw obs->PPP
    label(ax, 2.05, 6.10, "raw obs", "#555", fs=8)
    # do_pps: DO -> ClockMatrix -> TICC (both phase-comparators see do_pps)
    arrow(ax, (1.3, 2.25), (1.3, 2.93), color="#555", lw=1.6)
    arrow(ax, (1.3, 3.65), (1.3, 4.28), color="#555", lw=1.6)
    label(ax, 1.3, 2.62, "do_pps", "#555", fs=8)
    # gnss_pps -> ClockMatrix ref + TICC chB, down the far-left margin
    elbow(ax, [(0.85, 6.25), (0.40, 6.0), (0.40, 3.3), (0.55, 3.3)], "#555", lw=1.4)
    arrow(ax, (0.40, 4.62), (0.55, 4.62), color="#555", lw=1.4)
    ax.text(0.27, 4.95, "gnss_pps", rotation=90, ha="center", va="center",
            fontsize=7.5, color="#555", zorder=6)

    # ── the seven measurement arms ──────────────────────────────────────────
    # rx-TCXO arms (1, 2, 5) enter the rx box's LEFT edge; labels name the state.
    arrow(ax, (4.55, 6.55), (6.6, 6.4), color=C_RX, lw=2.0)             # Arm2 qErr -> x1
    label(ax, 5.6, 6.62, "Arm 2  qErr → x1", C_RX)
    arrow(ax, (4.55, 5.6), (6.6, 5.95), color=C_RX, lw=2.0)            # Arm1 PPP -> x0
    label(ax, 5.55, 5.82, "Arm 1  PPP dt_rx → x0", C_RX)
    arrow(ax, (4.55, 5.4), (6.6, 5.3), color=C_RX, lw=2.0)             # Arm5 TDCP -> x1
    label(ax, 5.55, 5.12, "Arm 5  TDCP → x1", C_RX)

    # DO arms (3, 4, 6, 7) enter the DO box's LEFT edge / bottom.
    arrow(ax, (4.55, 4.6), (6.6, 3.6), color=C_DO, lw=2.0)             # Arm4 TICC -> x2
    label(ax, 5.6, 3.98, "Arm 4  TICC → x2", C_DO)
    # Arm 7 cm_phase — ClockMatrix PFD -> x2 (dashed: host-dependent, Timebeat OTC)
    arrow(ax, (4.55, 3.3), (6.6, 3.22), color=C_DO, lw=2.0, ls=(0, (5, 2)))
    label(ax, 5.62, 3.02, "Arm 7  cm_phase → x2", C_DO)
    # Arm3 EXTINT: the receiver timestamps do_pps -> reaches down to the DO state
    elbow(ax, [(4.55, 6.4), (4.78, 6.4), (4.78, 2.9), (6.6, 2.9)], C_DO, lw=2.0)
    label(ax, 4.74, 4.5, "Arm 3\nEXTINT\n→ x2", C_DO)
    # Arm 6: holdover pseudo-obs -> x2 (synthetic, only during HOLDOVER)
    box(ax, 5.5, 0.95, 7.6, 1.65, "holdover integrator\n(TDCP-synth φ_do)",
        "#fdf2e9", fs=8, ec=C_DO, lw=1.4)
    elbow(ax, [(6.55, 1.65), (6.55, 2.2)], C_DO, lw=1.8, ls=(0, (4, 2)))
    label(ax, 6.55, 1.9, "Arm 6  pseudo → x2", C_DO, fs=8)

    # actuation feedback: LQR -> DO hardware (servo loop)
    elbow(ax, [(14.9, 2.45), (14.9, 0.45), (2.55, 0.45), (2.55, 1.55)],
          C_CTRL, lw=2.0)
    label(ax, 8.5, 0.45,
          "adjfine / DAC / ClockMatrix combo bus  →  steer the DO  (servo loop)",
          C_CTRL, fs=9)

    # ── Arm 7 callout (upper-right) ─────────────────────────────────────────
    ax.text(13.45, 6.85,
            "Arm 7 · cm_phase   (Timebeat OTC — otcBob1 / ptBoat)\n"
            "DO-vs-F9T phase straight off the ClockMatrix DPLL's\n"
            "on-chip PFD (~50 ps), read over I²C.  Same H as EXTINT\n"
            "(observes x2) — but needs no TICC and no EXTINT wire,\n"
            "and the SAME chip steers the DO via the combo bus.",
            ha="center", va="center", fontsize=9, color="#0b5345",
            bbox=dict(boxstyle="round,pad=0.4", fc="#e8f6f3", ec=C_CM, lw=1.4))

    # ── hidden-state note ───────────────────────────────────────────────────
    ax.text(11.4, 3.0, "*", fontsize=14, color="#c0392b", weight="bold", zorder=6)
    ax.text(15.45, 4.95,
            "* x3 (f_do) has NO arm.\n  It is the state we ACTUATE\n  on, yet never measure"
            " directly —\n  recovered only through the\n  predict step folding it into x2.",
            ha="right", va="center", fontsize=9, color="#c0392b",
            bbox=dict(boxstyle="round,pad=0.35", fc="#fdf2f0", ec="#c0392b", lw=1.2))

    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "dofreqest_wiring")
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png")


if __name__ == "__main__":
    main()

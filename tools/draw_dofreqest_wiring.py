#!/usr/bin/env python3
"""draw_dofreqest_wiring.py — fusion-filter / two-oscillator slide (16:9).

A presentation-first alternative to dofreqest_viz.svg / dofreqest-state-arm-
map.svg: large fonts, few words, no code-level indices.

Two big state boxes — the rx TCXO and the DO — each bisected by a dotted line
into a PHASE half and a FREQUENCY half.  Both are enclosed by the four-state
EKF + LQR.  Around them sit the physical signal chain (antenna → GNSS receiver
→ DO → TICC / ClockMatrix) and the PPP engine, wired in by the SEVEN measurement
arms; the LQR control output is the actuation that steers the DO (the servo
loop), not an arm:

  rx TCXO phase   ← PPP dt_rx
  rx TCXO freq    ← qErr, TDCP
  DO phase        ← EXTINT, TICC (couples rx phase), holdover,
                     ClockMatrix on-chip phase (Timebeat OTC only)
  DO freq         ← no arm — recovered through the predict step
  LQR → steer the DO (adjfine / DAC / ClockMatrix combo bus) — the servo loop

The ClockMatrix arm (cm_phase, added for Timebeat OTC — otcBob1 / ptBoat) reads
the DO-vs-F9T phase straight off the ClockMatrix DPLL's on-chip phase-frequency
detector (~50 ps) over I²C — same observation state as EXTINT, but with no TICC
and no EXTINT wire, and the same chip steers the DO via the combo bus.

Predict each epoch folds frequency into phase (φ ← φ + f·dt), which is how the
unobserved DO frequency state becomes recoverable.

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


def statebox(ax, x0, y0, x1, y1, title, fc, left, right, title_fs=17, inner_fs=15):
    """A state box bisected into phase (left) / frequency (right) halves."""
    box(ax, x0, y0, x1, y1, "", fc, lw=2.6)
    xm = (x0 + x1) / 2
    ax.plot([xm, xm], [y0 + 0.06, y1 - 0.06], color="#555", lw=1.4,
            ls=(0, (3, 3)), zorder=4)
    ax.text(xm, y1 + 0.16, title, ha="center", va="bottom", fontsize=title_fs,
            weight="bold", zorder=5)
    ax.text((x0 + xm) / 2, (y0 + y1) / 2, left, ha="center", va="center",
            fontsize=inner_fs, zorder=5)
    ax.text((xm + x1) / 2, (y0 + y1) / 2, right, ha="center", va="center",
            fontsize=inner_fs, zorder=5)
    return xm


def main():
    out_dir = sys.argv[sys.argv.index("--out-dir") + 1] if "--out-dir" in sys.argv else "."
    fig, ax = plt.subplots(figsize=(16, 9))
    ax.set_xlim(0, 16); ax.set_ylim(0, 9); ax.axis("off")

    ax.text(8.0, 8.66,
            "Fusion Filter Tracking Two Coupled Oscillators and Servo",
            ha="center", va="center", fontsize=26, weight="bold")

    # ── filter enclosure ────────────────────────────────────────────────────
    box(ax, 5.0, 0.7, 15.6, 7.95, "", C_FILT, lw=2.8, ec="#34495e")
    ax.text(5.25, 7.62, "Four-State Extended Kalman Filter + LQR", ha="left",
            va="center", fontsize=18, weight="bold", color="#34495e")

    # ── two state boxes (the oscillators) ───────────────────────────────────
    rxm = statebox(ax, 6.6, 5.0, 11.2, 6.7, "rx TCXO", "#d6eaf8",
                   "φ_rx\nphase", "f_rx\nfrequency")
    dom = statebox(ax, 6.6, 2.2, 11.2, 3.9, "DO", "#d5f0e0",
                   "φ_do\nphase", "f_do\nfrequency")

    # predict: frequency folds into phase each epoch (curved arrow R→L inside)
    for (y0, y1) in ((5.0, 6.7), (2.2, 3.9)):
        ax.add_patch(FancyArrowPatch((9.6, y1 - 0.18), (8.2, y1 - 0.18),
                     connectionstyle="arc3,rad=-0.5", arrowstyle="-|>",
                     mutation_scale=15, lw=1.8, color="#7f8c8d", zorder=6))
    ax.text(8.9, 4.6, "predict:  φ ← φ + f·dt", ha="center", fontsize=11,
            color="#7f8c8d", style="italic", zorder=7)

    # coupling between φ_rx and φ_do
    ax.add_patch(FancyArrowPatch((7.7, 5.0), (7.7, 3.9),
                 connectionstyle="arc3,rad=0.0", arrowstyle="<|-|>",
                 mutation_scale=16, lw=2.0, color=C_COUP, ls=(0, (4, 2)), zorder=6))
    label(ax, 7.7, 4.45, "φ_rx ↔ φ_do", C_COUP, fs=10.5)

    # ── LQR control + actuation (emphasised) ────────────────────────────────
    box(ax, 12.1, 2.4, 15.1, 3.7, "LQR control", "#fdebd0", fs=17,
        weight="bold", lw=2.4, ec=C_CTRL)
    arrow(ax, (11.2, 3.05), (12.1, 3.05), color=C_CTRL, lw=2.6)
    label(ax, 11.65, 3.32, "state", C_CTRL, fs=11)

    # ── physical hardware (left column, emphasised) ─────────────────────────
    draw_antenna(ax, 0.95, 7.6, s=1.0)
    box(ax, 0.55, 6.25, 4.55, 7.05, "GNSS receiver  (F9T)\nraw obs · qErr", C_HW,
        fs=13, weight="bold")
    box(ax, 0.55, 5.25, 4.55, 5.95, "PPP engine\ndt_rx · TDCP", C_PPP, fs=13,
        weight="bold")
    box(ax, 0.55, 4.30, 4.55, 4.95, "TICC\ndo_pps − gnss_pps", C_HW, fs=13,
        weight="bold")
    box(ax, 0.55, 2.95, 4.55, 3.65, "ClockMatrix DPLL\n(Timebeat OTC)", "#d0ece7",
        fs=12.5, ec=C_CM, lw=2.0, ls=(0, (5, 2)), weight="bold")
    box(ax, 0.55, 1.55, 4.55, 2.25, "Disciplined Oscillator\n→ do_pps    ← steer",
        "#d5f0e0", fs=13, weight="bold")

    arrow(ax, (0.95, 7.4), (0.95, 7.07), color="#333", lw=2.0)          # antenna->rx
    arrow(ax, (1.5, 6.25), (1.5, 5.97), color="#555", lw=1.8)           # raw obs->PPP
    label(ax, 2.2, 6.11, "raw obs", "#555", fs=9)

    # EXTINT: the DO's do_pps is wired up into the F9T's EXTINT input
    elbow(ax, [(0.55, 2.0), (0.28, 2.0), (0.28, 6.62), (0.55, 6.62)], "#444", lw=2.2)
    ax.text(0.46, 4.2, "EXTINT", rotation=90, ha="center", va="center",
            fontsize=12, color="#444", weight="bold", zorder=8,
            bbox=dict(boxstyle="round,pad=0.18", fc="white", ec="none", alpha=0.95))

    # ── the seven measurement arms (no Arm numbers, no state indices) ────────
    # rx-TCXO arms enter the rx box's LEFT edge.
    arrow(ax, (4.55, 6.55), (6.6, 6.4), color=C_RX, lw=2.2)             # qErr
    label(ax, 5.55, 6.62, "qErr", C_RX, fs=12)
    arrow(ax, (4.55, 5.6), (6.6, 5.95), color=C_RX, lw=2.2)            # PPP dt_rx
    label(ax, 5.55, 5.84, "PPP  dt_rx", C_RX, fs=12)
    arrow(ax, (4.55, 5.4), (6.6, 5.3), color=C_RX, lw=2.2)             # TDCP
    label(ax, 5.6, 5.12, "TDCP", C_RX, fs=12)

    # DO arms enter the DO box's LEFT edge / bottom.
    arrow(ax, (4.55, 4.6), (6.6, 3.6), color=C_DO, lw=2.2)             # TICC
    label(ax, 5.6, 4.02, "TICC", C_DO, fs=12)
    # ClockMatrix on-chip phase -> DO (dashed: host-dependent, Timebeat OTC)
    arrow(ax, (4.55, 3.3), (6.6, 3.22), color=C_DO, lw=2.2, ls=(0, (5, 2)))
    label(ax, 5.7, 3.0, "ClockMatrix", C_DO, fs=12)
    # EXTINT: the receiver's timestamp of do_pps -> reaches down to the DO state
    elbow(ax, [(4.55, 6.4), (4.85, 6.4), (4.85, 2.9), (6.6, 2.9)], C_DO, lw=2.2)
    label(ax, 4.85, 4.55, "EXTINT", C_DO, fs=12)
    # holdover pseudo-obs -> DO (synthetic, only during HOLDOVER)
    box(ax, 5.5, 0.95, 7.7, 1.65, "holdover", "#fdf2e9", fs=12, ec=C_DO, lw=1.6,
        weight="bold")
    elbow(ax, [(6.6, 1.65), (6.6, 2.2)], C_DO, lw=2.0, ls=(0, (4, 2)))

    # actuation feedback: LQR -> DO hardware (servo loop, emphasised)
    elbow(ax, [(14.9, 2.4), (14.9, 0.42), (2.55, 0.42), (2.55, 1.55)],
          C_CTRL, lw=2.8)
    label(ax, 9.0, 0.42, "steer the DO   ·   servo loop", C_CTRL, fs=15)

    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "dofreqest_wiring")
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png")


if __name__ == "__main__":
    main()

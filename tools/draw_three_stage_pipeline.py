#!/usr/bin/env python3
"""draw_three_stage_pipeline.py — the estimate → fuse → steer pipeline (16:9).

PePPAR Fix runs TWO Kalman filters, but three conceptual STAGES, left to right:

  (1) POSITION  — an EKF that estimates where the antenna is (and the nuisance
      states that come with a PPP solution: clock, biases, troposphere, one
      carrier-phase ambiguity per satellite).
  (2) TIME / measurement-fusion — an EKF (DOFreqEst) that fuses every available
      measurement into a 4-state estimate of the two oscillators: how far the
      clock is from GPS truth, and how fast it is drifting.  This is the
      "estimate the error between goal and reality" stage.
  (3) CONTROL — the LQR half of DOFreqEst: given the error estimate and its
      confidence, decide the one graceful steering command.  This is the
      "figure out what to do about it" stage.

Each estimator is shown BY CLASS, not by individual variable: one row per class
of state (with a count) and per class of measurement arm (with a count), plus a
note that every state carries a ±1σ confidence (the covariance).

Vector PDF (scales cleanly to 4K) + PNG preview.  Large fonts for projection.
"""
import os
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon, Arc, Circle

# stage palettes: (header, fill)
C_POS = ("#2e64a8", "#e7f0fb")
C_TIME = ("#1e8449", "#e6f5ec")
C_CTRL = ("#b9560f", "#fdeede")
C_INK = "#222222"
C_MUTE = "#5a5a5a"


def panel(ax, px0, px1, py0, py1, header, fill, num, title, sub, tagline):
    """Draw a stage panel: rounded body + colored header band + tagline."""
    ax.add_patch(FancyBboxPatch((px0, py0), px1 - px0, py1 - py0,
                                boxstyle="round,pad=0.02,rounding_size=0.10",
                                fc=fill, ec=header, lw=2.6, zorder=2))
    hb = 1.02  # header band height
    ax.add_patch(FancyBboxPatch((px0 + 0.04, py1 - hb), px1 - px0 - 0.08, hb - 0.02,
                                boxstyle="round,pad=0.02,rounding_size=0.10",
                                fc=header, ec="none", zorder=3))
    cx = (px0 + px1) / 2
    ax.add_patch(Circle((px0 + 0.52, py1 - hb / 2), 0.27, fc="white", ec="none", zorder=4))
    ax.text(px0 + 0.52, py1 - hb / 2, num, ha="center", va="center", fontsize=21,
            weight="bold", color=header, zorder=5)
    ax.text(px0 + 1.0, py1 - 0.38, title, ha="left", va="center", fontsize=21,
            weight="bold", color="white", zorder=5)
    ax.text(px0 + 1.0, py1 - 0.76, sub, ha="left", va="center", fontsize=12.5,
            color="white", zorder=5)
    ax.text(cx, py1 - hb - 0.28, tagline, ha="center", va="center", fontsize=14.5,
            color=C_MUTE, style="italic", zorder=5)


def section(ax, px0, y, text, color):
    ax.text(px0, y, text, ha="left", va="center", fontsize=14.5, weight="bold",
            color=color, zorder=5)


def row(ax, px0, px1, y, name, count, fs=13.5, color=C_INK):
    ax.text(px0, y, name, ha="left", va="center", fontsize=fs, color=color, zorder=5)
    if count is not None:
        ax.text(px1, y, count, ha="right", va="center", fontsize=fs, weight="bold",
                color=color, zorder=5)


def divider(ax, px0, px1, y, color):
    ax.plot([px0, px1], [y, y], color=color, lw=1.0, ls=(0, (3, 3)), zorder=4)


def total(ax, px1, y, text, color):
    ax.text(px1, y, text, ha="right", va="center", fontsize=15, weight="bold",
            color=color, zorder=5)


def confidence_note(ax, px0, y, color):
    ax.text(px0, y, "every state carries  ± 1σ  (covariance)", ha="left",
            va="center", fontsize=11.5, color=color, style="italic", zorder=5)


def arrow(ax, p1, p2, color, lw=3.4, ls="-"):
    ax.annotate("", xy=p2, xytext=p1,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, linestyle=ls,
                                shrinkA=1, shrinkB=1, mutation_scale=28), zorder=6)


def gap_label(ax, x, y, text, color, fs=11.5):
    ax.text(x, y, text, ha="center", va="center", fontsize=fs, color=color,
            weight="bold", zorder=8,
            bbox=dict(boxstyle="round,pad=0.18", fc="white", ec=color, lw=1.1, alpha=0.97))


def draw_clock(ax, cx, cy, r, color):
    ax.add_patch(Circle((cx, cy), r, fc="white", ec=color, lw=2.6, zorder=6))
    ax.plot([cx, cx], [cy, cy + r * 0.6], color=color, lw=2.4, zorder=7)
    ax.plot([cx, cx + r * 0.42], [cy, cy + r * 0.12], color=color, lw=2.4, zorder=7)


def draw_antenna(ax, cx, cy, s=1.0, color="#333"):
    ax.plot([cx, cx], [cy - 0.4 * s, cy + 0.16 * s], color=color, lw=2.6, zorder=6)
    ax.add_patch(Polygon([[cx - 0.13 * s, cy + 0.16 * s], [cx + 0.13 * s, cy + 0.16 * s],
                          [cx, cy + 0.46 * s]], closed=True, fc="#cfd2d4", ec=color,
                         lw=1.6, zorder=6))
    ax.plot([cx - 0.17 * s, cx + 0.17 * s], [cy - 0.4 * s, cy - 0.4 * s], color=color,
            lw=2.6, zorder=6)
    for rr in (0.26, 0.42):
        ax.add_patch(Arc((cx, cy + 0.46 * s), rr * s, rr * s, theta1=20, theta2=70,
                         color="#2e64a8", lw=1.8, zorder=5))


def main():
    out_dir = sys.argv[sys.argv.index("--out-dir") + 1] if "--out-dir" in sys.argv else "."
    fig, ax = plt.subplots(figsize=(16, 9))
    ax.set_xlim(0, 16); ax.set_ylim(0, 9); ax.axis("off")

    ax.text(8.0, 8.62, "Estimate the error  →  then steer it out, gracefully",
            ha="center", va="center", fontsize=27, weight="bold")
    ax.text(8.0, 8.04, "two Kalman estimators feed one optimal controller — "
            "each shown by class of state and class of measurement",
            ha="center", va="center", fontsize=13.5, color=C_MUTE)

    P1 = (1.30, 5.20)
    P2 = (5.95, 9.85)
    P3 = (10.60, 14.50)
    PY0, PY1 = 1.55, 7.35
    pad = 0.32

    # ── Stage 1: POSITION ───────────────────────────────────────────────────
    h, f = C_POS
    panel(ax, P1[0], P1[1], PY0, PY1, h, f, "1", "POSITION",
          "EKF  ·  PPP position", "where is the antenna?")
    lx, rx = P1[0] + pad, P1[1] - pad
    section(ax, lx, 5.78, "Estimates  ·  states", h)
    row(ax, lx, rx, 5.42, "position  (X Y Z)", "3")
    row(ax, lx, rx, 5.12, "receiver clock", "1")
    row(ax, lx, rx, 4.82, "inter-system bias", "2")
    row(ax, lx, rx, 4.52, "troposphere  (ZTD)", "1")
    row(ax, lx, rx, 4.22, "ambiguity, per satellite", "N")
    divider(ax, lx, rx, 4.00, h)
    total(ax, rx, 3.76, "7 + N states", h)
    section(ax, lx, 3.34, "Measures  ·  arms", h)
    row(ax, lx, rx, 2.98, "code range  —  every SV", None)
    row(ax, lx, rx, 2.68, "carrier phase  —  every SV", None)
    row(ax, lx, rx, 2.38, "(+ NAV2 & weather ties)", None, color=C_MUTE)
    divider(ax, lx, rx, 2.16, h)
    total(ax, rx, 1.92, "2 arms  ×  ~30 SVs", h)
    confidence_note(ax, lx, 1.70, C_MUTE)

    # ── Stage 2: TIME / measurement-fusion ──────────────────────────────────
    h, f = C_TIME
    panel(ax, P2[0], P2[1], PY0, PY1, h, f, "2", "TIME",
          "EKF  ·  measurement fusion", "how far off is the clock?")
    lx, rx = P2[0] + pad, P2[1] - pad
    section(ax, lx, 5.78, "Estimates  ·  states", h)
    row(ax, lx, rx, 5.42, "rx-TCXO  phase", "1")
    row(ax, lx, rx, 5.12, "rx-TCXO  frequency", "1")
    row(ax, lx, rx, 4.82, "DO  phase", "1")
    row(ax, lx, rx, 4.52, "DO  frequency", "1")
    divider(ax, lx, rx, 4.30, h)
    total(ax, rx, 4.06, "4 states", h)
    section(ax, lx, 3.62, "Measures  ·  fuses", h)
    row(ax, lx, rx, 3.28, "PPP time · qErr · TDCP", None)
    row(ax, lx, rx, 2.98, "EXTINT · TICC · holdover", None)
    row(ax, lx, rx, 2.68, "ClockMatrix on-chip phase", None)
    divider(ax, lx, rx, 2.46, h)
    total(ax, rx, 2.22, "7 measurement arms", h)
    confidence_note(ax, lx, 1.92, C_MUTE)
    ax.text(lx, 1.70, "(each arm observes one state class)", ha="left", va="center",
            fontsize=11.5, color=C_MUTE, style="italic", zorder=5)

    # ── Stage 3: CONTROL (LQR) ──────────────────────────────────────────────
    h, f = C_CTRL
    panel(ax, P3[0], P3[1], PY0, PY1, h, f, "3", "CONTROL",
          "LQR  ·  optimal control", "what to do about it?")
    lx, rx = P3[0] + pad, P3[1] - pad
    section(ax, lx, 5.78, "Reads", h)
    row(ax, lx, rx, 5.42, "the 4 time-filter states", None)
    row(ax, lx, rx, 5.12, "+ their  ± 1σ  confidence", None)
    section(ax, lx, 4.66, "Decides", h)
    row(ax, lx, rx, 4.30, "one steering command", None)
    row(ax, lx, rx, 4.00, "adjfine / DAC / combo bus", None, color=C_MUTE)
    section(ax, lx, 3.54, "Goal", h)
    ax.text(lx, 3.18, "drive phase & frequency error", ha="left", va="center",
            fontsize=14, color=C_INK, zorder=5)
    ax.text(lx, 2.88, "to zero — smoothly", ha="left", va="center", fontsize=14,
            color=C_INK, zorder=5)
    ax.text(lx, 2.50, "·  no overshoot", ha="left", va="center", fontsize=13,
            color=C_MUTE, zorder=5)
    ax.text(lx, 2.22, "·  weight each move by confidence", ha="left",
            va="center", fontsize=13, color=C_MUTE, zorder=5)

    # ── input: raw observations into Stage 1 ────────────────────────────────
    draw_antenna(ax, 0.66, 4.55, s=1.0)
    arrow(ax, (0.66, 4.10), (1.30, 3.85), C_MUTE, lw=2.6)
    ax.text(0.66, 3.52, "raw GNSS\nobservations", ha="center", va="center",
            fontsize=11.5, color=C_MUTE, weight="bold", zorder=7)

    # ── inter-stage arrows (to the right), labels fit inside the gaps ────────
    arrow(ax, (5.22, 4.45), (5.93, 4.45), C_POS[0])
    gap_label(ax, 5.575, 5.12, "clock\n& rate", C_POS[0])
    arrow(ax, (9.87, 4.45), (10.58, 4.45), C_TIME[0])
    gap_label(ax, 10.225, 5.12, "error\n± σ", C_TIME[0])

    # ── output + closed loop ────────────────────────────────────────────────
    draw_clock(ax, 13.55, 0.85, 0.42, C_CTRL[0])
    ax.text(13.55, 0.18, "Disciplined Oscillator", ha="center", va="center",
            fontsize=12.5, weight="bold", color=C_CTRL[0], zorder=7)
    arrow(ax, (13.05, 1.55), (13.55, 1.30), C_CTRL[0], lw=2.8)          # LQR -> DO
    arrow(ax, (13.99, 0.85), (15.10, 0.85), "#38761d", lw=2.8)          # DO -> PPS OUT
    ax.text(15.45, 0.85, "PPS\nOUT", ha="center", va="center", fontsize=12.5,
            weight="bold", color="#38761d", zorder=7)
    # feedback: the steered DO is re-measured by Stage 2's arms
    ax.annotate("", xy=(7.95, 1.55), xytext=(13.12, 0.85),
                arrowprops=dict(arrowstyle="-|>", color=C_MUTE, lw=2.2,
                                linestyle=(0, (5, 3)),
                                connectionstyle="arc3,rad=0.16",
                                shrinkA=8, shrinkB=2, mutation_scale=20), zorder=5)
    ax.text(10.25, 0.34, "closed loop:  the steered DO is re-measured by the time filter",
            ha="center", va="center", fontsize=12.5, color=C_MUTE, style="italic",
            zorder=7)

    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "three_stage_pipeline")
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png")


if __name__ == "__main__":
    main()

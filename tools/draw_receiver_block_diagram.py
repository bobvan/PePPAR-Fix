#!/usr/bin/env python3
"""draw_receiver_block_diagram.py — GNSS receiver internal block diagram for the
"one set of raw observations, two timing solutions" slide.

Narrative:
  antenna -> RF front-end -> correlator bank (the specialized hardware that
  demodulates the PR code via the DLL and tracks carrier phase via the PLL)
  -> RAW OBSERVATIONS.  At the raw observations the path SPLITS:
    - internal branch: the receiver's own SPP solution (code least-squares +
      carrier smoothing) -> a timing receiver's best fix WITHOUT PPP;
    - external branch: the raw observations leave the receiver and are fused
      with real-time SSR corrections in an external PPP engine -> a more
      precise time fix.
PPP internals are deferred to other slides.

16:9, vector PDF + PNG.
"""
import os
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon

# palette
C_HW   = "#9fc5e8"   # RF / correlator (specialized hardware)
C_RAW  = "#ffd966"   # raw observations (the pivot)
C_INT  = "#d9d9d9"   # internal SPP path (good-enough)
C_PPP  = "#b6d7a8"   # external PPP path (precise)
C_SSR  = "#f6b26b"   # SSR corrections
C_OUT  = "#93c47d"   # precise PPS output
C_RXED = "#1c4587"   # receiver-boundary edge
C_EXT  = "#cc6600"   # the "leaves the receiver" arrow


def box(ax, cx, cy, w, h, text, fc, fs=12.5, ec="#333333", lw=1.8, weight="normal"):
    ax.add_patch(FancyBboxPatch((cx - w / 2, cy - h / 2), w, h,
                                boxstyle="round,pad=0.02,rounding_size=0.10",
                                fc=fc, ec=ec, lw=lw, zorder=3))
    ax.text(cx, cy, text, ha="center", va="center", fontsize=fs, zorder=4, weight=weight)


def arrow(ax, p1, p2, color="#333333", lw=2.4, ls="-", label=None, lpos=None,
          lcolor=None, fs=11.5):
    ax.annotate("", xy=p2, xytext=p1,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, linestyle=ls,
                                shrinkA=3, shrinkB=3, mutation_scale=24), zorder=2)
    if label:
        lx, ly = lpos if lpos else ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
        ax.text(lx, ly, label, ha="center", va="center", fontsize=fs,
                color=lcolor or color, zorder=5, weight="bold",
                bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="none", alpha=0.9))


def main():
    out_dir = sys.argv[sys.argv.index("--out-dir") + 1] if "--out-dir" in sys.argv else "."
    fig, ax = plt.subplots(figsize=(16, 9))
    ax.set_xlim(0, 16); ax.set_ylim(0, 9); ax.axis("off")

    # ── receiver boundary ───────────────────────────────────────────────
    ax.add_patch(FancyBboxPatch((1.2, 2.5), 7.9, 5.3,
                                boxstyle="round,pad=0.02,rounding_size=0.15",
                                fc="#f0f6ff", ec=C_RXED, lw=2.6, ls=(0, (6, 4)), zorder=1))
    ax.text(1.45, 7.55, "GNSS  RECEIVER   (silicon)", ha="left", va="center",
            fontsize=15, color=C_RXED, weight="bold", zorder=4)

    # ── antenna (outside, left) ─────────────────────────────────────────
    ax.add_patch(Polygon([(0.55, 6.55), (0.95, 6.55), (0.75, 7.15)],
                         closed=True, fc="#cccccc", ec="#333333", lw=1.6, zorder=3))
    ax.plot([0.75, 0.75], [6.05, 6.55], color="#333333", lw=2.2, zorder=3)
    ax.text(0.75, 5.85, "antenna", ha="center", va="center", fontsize=12)

    # ── signal chain (top row, inside) ──────────────────────────────────
    box(ax, 2.45, 6.6, 1.7, 1.0, "RF front-end\nLNA · mixer · ADC", C_HW, fs=12)
    box(ax, 5.05, 6.6, 2.7, 1.7,
        "Correlator bank  (per-SV)\n— specialized hardware —\n"
        "DLL:  code  →  pseudorange\nPLL:  carrier  →  carrier phase",
        C_HW, fs=12)
    box(ax, 7.95, 6.6, 1.7, 1.5,
        "RAW\nOBSERVATIONS\nPR · φ · Doppler\n· C/N0", C_RAW, fs=12.5, lw=2.6, weight="bold")

    # ── internal SPP branch (lower, inside) ─────────────────────────────
    box(ax, 4.7, 3.7, 4.0, 1.7,
        "Internal SPP solution\ncode least-squares  +  carrier smoothing\n"
        "→  PVT / PPS\n(timing receiver — best WITHOUT PPP)", C_INT, fs=12)

    # ── external PPP branch (right, outside) ────────────────────────────
    box(ax, 11.9, 6.6, 3.2, 1.5,
        "PPP engine  (host)\nfloat / ambiguity-resolved filter\n→ precise time", C_PPP,
        fs=12.5, lw=2.2, weight="bold")
    box(ax, 11.9, 3.7, 3.2, 1.4,
        "SSR corrections  (NTRIP)\nprecise orbit · clock\n· code & phase bias", C_SSR, fs=12)
    box(ax, 14.9, 6.6, 1.7, 1.3, "Precise\ntime / PPS\n(PPP)", C_OUT, fs=12.5, weight="bold")

    # ── arrows ──────────────────────────────────────────────────────────
    arrow(ax, (0.95, 6.6), (1.6, 6.6))                       # antenna -> RF
    arrow(ax, (3.3, 6.6), (3.7, 6.6))                        # RF -> correlator
    arrow(ax, (6.4, 6.6), (7.1, 6.6))                        # correlator -> raw
    # split: internal (down)
    arrow(ax, (7.6, 5.85), (5.6, 4.55), color="#666666", lw=2.6,
          label="internal\npath", lpos=(7.15, 5.15), lcolor="#444444", fs=11)
    # split: external (leaves the receiver, crossing the dashed boundary)
    arrow(ax, (8.8, 6.6), (10.3, 6.6), color=C_EXT, lw=3.0,
          label="raw obs exported\nout of the receiver\n(e.g. UBX RXM-RAWX)",
          lpos=(9.55, 7.45), lcolor=C_EXT, fs=11.5)
    arrow(ax, (11.9, 4.4), (11.9, 5.85), color=C_EXT, lw=2.6,
          label="+ SSR", lpos=(12.45, 5.1), lcolor=C_EXT)
    arrow(ax, (13.5, 6.6), (14.05, 6.6), color="#38761d", lw=2.6)   # PPP -> precise PPS

    # ── title + legend + caption ────────────────────────────────────────
    ax.text(8.0, 8.62, "One set of raw observations  →  two timing solutions",
            ha="center", va="center", fontsize=23, weight="bold")
    ax.plot([1.4, 2.0], [1.65, 1.65], color="#666666", lw=3.5)
    ax.text(2.15, 1.65, "internal SPP + carrier smoothing  (no PPP)", ha="left",
            va="center", fontsize=13)
    ax.plot([8.3, 8.9], [1.65, 1.65], color=C_EXT, lw=3.5)
    ax.text(9.05, 1.65, "external PPP fused with real-time SSR  (more precise)",
            ha="left", va="center", fontsize=13)
    ax.text(8.0, 0.7,
            "The same raw carrier-phase + code observations either drive the receiver's own "
            "SPP fix, or leave the receiver for an\nexternal PPP engine fused with SSR.  "
            "(How PPP turns SSR + raw observations into a precise time fix:  later slides.)",
            ha="center", va="center", fontsize=12.5, color="#333333")

    fig.tight_layout()
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "receiver_block_diagram")
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png")


if __name__ == "__main__":
    main()

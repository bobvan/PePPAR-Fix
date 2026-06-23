#!/usr/bin/env python3
"""plot_receiver_signal_coverage.py — u-blox receiver signal-band coverage,
laid out on the GNSS frequency axis (à la PocketSDR's "GNSS Signal Bands").

Conveys the coverage *differences* across the u-blox receivers we've tested:
one lane per receiver, a shaded block over each RF band group it tracks, on a
broken frequency axis (lower L5/E5b/L2/E6 bands + the L1 band).  Coverage is
from docs/f9t-firmware-capabilities.md (the capability matrix + the "what each
variant can actually do" summary).

  solid  = tracked
  hatch  = either/or (2-band part: L2 *or* L5, pick one; or switchable)
  faint  = uncertain / not confirmed
  empty  = not tracked

Usage: plot_receiver_signal_coverage.py --out-dir plots
"""
import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# RF band groups: (f_lo, f_hi, short label, signals text, center)
BANDS = [
    (1164, 1189, "L5 / E5a / B2a", "GPS L5 · GAL E5a · BDS B2a · NavIC/QZSS L5", 1176.45),
    (1197, 1217, "E5b / B2I",      "GAL E5b · BDS B2I", 1207.14),
    (1217, 1254, "L2",             "GPS L2C · GLONASS L2 · QZSS L2C", 1227.6),
    (1258, 1300, "E6 / B3",        "GAL E6 · BDS B3I · QZSS L6", 1273.0),
    (1559, 1610, "L1 / E1 / B1",   "GPS L1 · GAL E1 · BDS B1I/B1C · GLO L1 · QZSS/NavIC L1", 1575.42),
]
BKEY = ["L5", "E5b", "L2", "E6", "L1"]

# coverage state per receiver per band: 1=tracked, 0.5=either/or-or-switchable,
# 0.25=uncertain, 0=no.  Source: docs/f9t-firmware-capabilities.md.
RX = [
    ("ZED-X20P (HPG 2.02)",        dict(L5=1, E5b=0.25, L2=1, E6=1,   L1=1)),
    ("ZED-F9T-20B (TIM 2.25)",     dict(L5=1, E5b=0,    L2=0, E6=0,   L1=1)),
    ("ZED-F9T (TIM 2.20, L5-hw)",  dict(L5=0.5, E5b=0,  L2=0.5, E6=0, L1=1)),
    ("ZED-F9T (TIM 2.20, L2-hw)",  dict(L5=0, E5b=0.25, L2=1, E6=0,   L1=1)),
    ("LEA-F9T-11B",                dict(L5=0.5, E5b=0,  L2=0.5, E6=0, L1=1)),
    ("ZED-F9P (HPG 1.51)",         dict(L5=0, E5b=1,    L2=1, E6=0,   L1=1)),
    ("NEO-F10T (TIM 3.01)",        dict(L5=1, E5b=0,    L2=0, E6=0,   L1=1)),
]

C_FULL = "#2e7d32"; C_ALT = "#f9a825"; C_UNC = "#bdbdbd"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", default=".")
    args = ap.parse_args()

    plt.rcParams.update({"font.size": 13})
    # broken x-axis: lower bands (left) + L1 band (right), shared receiver lanes
    fig, (axL, axR) = plt.subplots(
        1, 2, figsize=(16, 9), sharey=True,
        gridspec_kw=dict(width_ratios=[(1300 - 1160), (1612 - 1556)], wspace=0.03))
    axL.set_xlim(1160, 1300); axR.set_xlim(1556, 1612)
    n = len(RX); axL.set_ylim(-0.6, n + 1.4)
    ytick = [n - 1 - i for i in range(n)]
    axL.set_yticks(ytick); axL.set_yticklabels([r[0] for r in RX])

    # header strip (band labels + signals) above the lanes
    for (lo, hi, lab, sig, ctr) in BANDS:
        ax = axL if lo < 1400 else axR
        ax.add_patch(Rectangle((lo, n - 0.35), hi - lo, 0.7, fc="#e8eef7",
                               ec="#90a4d4", lw=1.0, zorder=1))
        ax.text((lo + hi) / 2, n + 0.55, lab, ha="center", va="bottom",
                fontsize=12, fontweight="bold")
        ax.text((lo + hi) / 2, n, sig, ha="center", va="center", fontsize=8.5,
                color="#33415c", zorder=2)

    # receiver coverage lanes
    for i, (name, cov) in enumerate(RX):
        y = n - 1 - i
        for (lo, hi, lab, sig, ctr), key in zip(BANDS, BKEY):
            ax = axL if lo < 1400 else axR
            ax.add_patch(Rectangle((lo, y - 0.32), hi - lo, 0.64, fc="none",
                                   ec="#cccccc", lw=0.6, zorder=1))
            s = cov.get(key, 0)
            if s >= 1:
                ax.add_patch(Rectangle((lo, y - 0.32), hi - lo, 0.64, fc=C_FULL,
                                       ec="#1b5e20", lw=1.0, alpha=0.85, zorder=2))
            elif s == 0.5:
                ax.add_patch(Rectangle((lo, y - 0.32), hi - lo, 0.64, fc=C_ALT,
                                       ec="#e65100", lw=1.0, alpha=0.7, hatch="///", zorder=2))
            elif s == 0.25:
                ax.add_patch(Rectangle((lo, y - 0.32), hi - lo, 0.64, fc=C_UNC,
                                       ec="#888", lw=0.8, alpha=0.5, hatch="..", zorder=2))

    for ax in (axL, axR):
        ax.grid(axis="x", alpha=0.25)
        ax.set_xlabel("frequency  [MHz]")
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    axR.tick_params(left=False)
    axL.text(1300, -0.5, "//", ha="center", va="center", fontsize=14, color="0.5")
    axR.text(1556, -0.5, "//", ha="center", va="center", fontsize=14, color="0.5")

    from matplotlib.patches import Patch
    leg = [Patch(fc=C_FULL, alpha=0.85, label="tracked"),
           Patch(fc=C_ALT, alpha=0.7, hatch="///", label="either/or (2-band: L2 OR L5)"),
           Patch(fc=C_UNC, alpha=0.5, hatch="..", label="uncertain / unconfirmed")]
    axR.legend(handles=leg, loc="lower right", fontsize=10, framealpha=0.95)
    fig.suptitle("u-blox receiver signal-band coverage  (on the GNSS frequency axis)\n"
                 "X20P spans 4 bands incl. E6/HAS · F9P is L1+L2 (no L5) · timing F9T/F10T lean L1+L5",
                 fontsize=17, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.subplots_adjust(left=0.17)
    os.makedirs(args.out_dir, exist_ok=True)
    out = os.path.join(args.out_dir, "receiver_signal_coverage")
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png")


if __name__ == "__main__":
    sys.exit(main())

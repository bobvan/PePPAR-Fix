#!/usr/bin/env python3
"""otcBob1 cross-day A/B: peppar-fix combo vs timebeat hardware DPLL.

Ref-immune chA-chB TDEV on PiPuss TICC#4, both nights vs the SAME GNSSDO+
AtomiChron gold reference (chA).  Plots the discipline comparison (bold) plus
the GNSSDO+ control (thin) so you can see the control holds short/mid-τ and
diverges long-τ.  capture_version=1, ticc_capture @0275ad3 both legs.
"""
import csv, os, subprocess
import numpy as np
import allantools
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ANALYSIS_VERSION = "otc-pepparfix-vs-timebeat-1"
LEGS = {
    "timebeat (hardware DPLL, 06-22)":
        ("/home/bob/gt/otc-timebeat-leg2-20260622/gnssdoplus-vs-otcbob1-2026-06-22.csv", "tab:gray"),
    "peppar-fix (combo servo, 06-26)":
        ("/home/bob/gt/otc-pepparfix-leg1-20260626/otc-overnight-2026-06-26.csv", "tab:blue"),
}
OUT = os.path.expanduser("~/gt/datasheets/otc-pepparfix-vs-timebeat.png")


def load(path):
    A, B = {}, {}
    for r in csv.reader(open(path)):
        if not r or r[0].startswith("#") or r[0] == "ts_iso":
            continue
        ch, sec, ps = r[1], int(r[2]), int(r[3])
        (A if ch == "chA" else B)[sec] = sec * 1e12 + ps
    return A, B


def tdev(d):
    secs = sorted(d)
    ph = np.array([d[s] for s in secs]); x = np.array(secs, float) - secs[0]
    ph = ph - np.polyval(np.polyfit(x, ph, 1), x)
    t, td, _, _ = allantools.tdev(ph * 1e-12, rate=1.0, data_type="phase", taus="octave")
    return t, td * 1e12


fig, ax = plt.subplots(figsize=(13.33, 7.5))
for name, (path, color) in LEGS.items():
    A, B = load(path)
    common = sorted(set(A) & set(B))
    diff = {s: A[s] - B[s] for s in common}
    t, td = tdev(diff)
    ax.loglog(t, td, "o-", color=color, lw=3, markersize=8,
              label="%s — otcBob1 disciplined" % name)
    tc, tdc = tdev(A)
    ax.loglog(tc, tdc, ":", color=color, lw=1.5, alpha=0.7,
              label="   GNSSDO+ control (chA)")

ax.axhline(354, color="red", ls="--", lw=1.3)
ax.text(150, 365, "354 ps per-clock budget", fontsize=13, color="red")
ax.axvspan(1, 32, color="green", alpha=0.06)
ax.text(2, 1500, "servo-loop region\n(peppar-fix wins 20–30%)", fontsize=12, color="darkgreen")
ax.axvspan(64, 256, color="orange", alpha=0.06)
ax.text(70, 1500, "mid-τ realization hump\n(loses 8–24%)", fontsize=12, color="darkorange")

ax.set_xlabel("τ (s)", fontsize=18)
ax.set_ylabel("TDEV (ps)", fontsize=18)
ax.set_title("otcBob1 OTC discipline: peppar-fix combo vs timebeat hardware DPLL\n"
             "ref-immune chA−chB vs GNSSDO+AtomiChron, TICC#4 (same wiring both nights)",
             fontsize=18)
ax.tick_params(which="both", labelsize=15)
ax.grid(True, which="both", alpha=0.3)
ax.legend(fontsize=12, loc="lower right", framealpha=0.95)

cav = ("CAVEATS:  • chA−chB inherits the GNSSDO+ floor (~20–57 ps); the comparison is\n"
       "valid because both nights share the SAME reference.  • Control (dotted) matches to\n"
       "τ≈256 s then diverges — long-τ is control-limited; TCH w/ FE-5680A Rb separates it.\n"
       "• Cross-day: atmospheric/sky differ night-to-night (separate antennas).")
ax.text(0.015, 0.97, cav, transform=ax.transAxes, fontsize=8, va="top",
        family="monospace", bbox=dict(boxstyle="round", fc="#fffbe6", ec="gray", alpha=0.95))

sha = subprocess.getoutput("git -C %s rev-parse --short HEAD" % os.path.dirname(__file__))
fig.text(0.5, 0.005, "ANALYSIS_VERSION=%s  git=%s  data: otc-timebeat-leg2-20260622 + "
         "otc-pepparfix-leg1-20260626" % (ANALYSIS_VERSION, sha), ha="center",
         fontsize=6.5, color="gray")
fig.tight_layout(rect=(0, 0.02, 1, 1))
fig.savefig(OUT, dpi=125)
print("wrote", OUT)

#!/usr/bin/env python3
"""otcBob1 INDIVIDUAL stability (TCH) — peppar-fix vs timebeat, floors removed.

Three-cornered hat over {chA=GNSSDO+, chB=otcBob1, Rb=FE-5680A} on each night's
TICC#4 capture (the TICC's Rb timebase makes the 2-channel CSV a complete triad).
Recovers otcBob1's OWN TDEV, free of the GNSSDO+ and Rb reference floors, so the
two nights compare as oscillators rather than as differences-vs-a-shared-ref.

closed-form: σ²_Y = ½(σ²_XY + σ²_YZ − σ²_XZ), X=chA Y=chB Z=Rb.
VALID short/mid-τ; both nodes are GPS-disciplined ⇒ correlated at long τ (the
shared GPS component leaks into the Rb estimate), so τ≳100 s is shaded as
TCH-unreliable.  Rb floor (dotted) shown to mark where the raw chA−chB was
reference-limited, not otcBob1-limited.
"""
import csv, os, subprocess
import numpy as np
import allantools
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ANALYSIS_VERSION = "otc-tch-individual-1"
NIGHTS = {
    "timebeat (hardware DPLL, 06-22)":
        ("/home/bob/gt/otc-timebeat-leg2-20260622/gnssdoplus-vs-otcbob1-2026-06-22.csv", "tab:gray"),
    "peppar-fix (combo servo, 06-26)":
        ("/home/bob/gt/otc-pepparfix-leg1-20260626/otc-overnight-2026-06-26.csv", "tab:blue"),
}
OUT = os.path.expanduser("~/gt/datasheets/otc-tch-individual.png")
TCH_VALID_TAU = 100.0   # beyond this the two GPS-locked nodes correlate


def load(path):
    A, B = {}, {}
    for r in csv.reader(open(path)):
        if not r or r[0].startswith("#") or r[0] == "ts_iso":
            continue
        ch, sec, ps = r[1], int(r[2]), int(r[3])
        (A if ch == "chA" else B)[sec] = sec * 1e12 + ps
    return A, B


def tvar(d):
    """Return (taus, TVAR=TDEV^2 in s^2) octave."""
    secs = sorted(d)
    ph = np.array([d[s] for s in secs]); x = np.array(secs, float) - secs[0]
    ph = ph - np.polyval(np.polyfit(x, ph, 1), x)
    t, td, _, _ = allantools.tdev(ph * 1e-12, rate=1.0, data_type="phase", taus="octave")
    return t, td ** 2


def aligned_tvars(A, B):
    common = sorted(set(A) & set(B))
    chA = {s: A[s] for s in common}
    chB = {s: B[s] for s in common}
    dXY = {s: A[s] - B[s] for s in common}
    tX, vXZ = tvar(chA)        # GNSSDO+ - Rb
    tY, vYZ = tvar(chB)        # otcBob1 - Rb
    tD, vXY = tvar(dXY)        # GNSSDO+ - otcBob1
    n = min(len(tX), len(tY), len(tD))
    t = tX[:n]
    vY = 0.5 * (vXY[:n] + vYZ[:n] - vXZ[:n])     # otcBob1 individual
    vZ = 0.5 * (vXZ[:n] + vYZ[:n] - vXY[:n])     # Rb individual
    return t, vY, vZ


fig, ax = plt.subplots(figsize=(13.33, 7.5))
for name, (path, color) in NIGHTS.items():
    A, B = load(path)
    t, vY, vZ = aligned_tvars(A, B)
    tdY = np.sqrt(np.clip(vY, 0, None)) * 1e12   # ps; clip neg-variance
    okY = vY > 0
    ax.loglog(t[okY], tdY[okY], "o-", color=color, lw=3, markersize=8,
              label="%s — otcBob1 (TCH individual)" % name)
    tdZ = np.sqrt(np.clip(vZ, 0, None)) * 1e12
    okZ = vZ > 0
    ax.loglog(t[okZ], tdZ[okZ], ":", color=color, lw=1.4, alpha=0.6,
              label="   FE-5680A Rb floor (TCH)")

ax.axhline(354, color="red", ls="--", lw=1.3)
ax.text(2, 365, "354 ps per-clock budget", fontsize=13, color="red")
ax.axvspan(1, 32, color="green", alpha=0.06)
ax.text(1.4, 60, "peppar-fix wins\n20–28%", fontsize=12, color="darkgreen")
ax.axvspan(64, TCH_VALID_TAU, color="orange", alpha=0.06)
ax.text(64, 60, "mid-τ hump\n+18%", fontsize=12, color="darkorange")
ax.axvspan(TCH_VALID_TAU, 1e5, color="gray", alpha=0.12)
ax.text(160, 1500, "TCH unreliable\n(GPS-correlated nodes;\nRb RWFM dominates)",
        fontsize=11, color="dimgray")

ax.set_xlabel("τ (s)", fontsize=18)
ax.set_ylabel("TDEV (ps)", fontsize=18)
ax.set_title("otcBob1 OCXO — individual stability (TCH), peppar-fix vs timebeat\n"
             "three-cornered hat over {GNSSDO+, otcBob1, FE-5680A Rb}, TICC#4",
             fontsize=18)
ax.tick_params(which="both", labelsize=15)
ax.grid(True, which="both", alpha=0.3)
ax.legend(fontsize=12, loc="lower right", framealpha=0.95)
ax.set_ylim(20, 3e3)

cav = ("Rb floor (dotted) recovers ~27 ps @1s, climbing to ~0.5 ns @1000 s (FE-5680A\n"
       "RWFM) — that floor was what limited the raw chA−chB long-τ, NOT otcBob1.  Rb\n"
       "matches both nights τ≤100 s → TCH validated there.  Both nodes GPS-disciplined,\n"
       "so τ>100 s is shaded unreliable (shared GPS leaks into the split).")
ax.text(0.985, 0.97, cav, transform=ax.transAxes, fontsize=8, va="top", ha="right",
        family="monospace", bbox=dict(boxstyle="round", fc="#fffbe6", ec="gray", alpha=0.95))

sha = subprocess.getoutput("git -C %s rev-parse --short HEAD" % os.path.dirname(__file__))
fig.text(0.5, 0.005, "ANALYSIS_VERSION=%s  git=%s  data: otc-timebeat-leg2-20260622 + "
         "otc-pepparfix-leg1-20260626 (TCH)" % (ANALYSIS_VERSION, sha), ha="center",
         fontsize=6.5, color="gray")
fig.tight_layout(rect=(0, 0.02, 1, 1))
fig.savefig(OUT, dpi=125)
print("wrote", OUT)

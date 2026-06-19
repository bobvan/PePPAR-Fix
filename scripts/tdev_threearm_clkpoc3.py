#!/usr/bin/env python3
"""clkPoC3 (three-arm control) chA TDEV per arm — default vs no-extint vs
no-ticc.  Sub-decade tau grid; skips the convergence transient.  Prints a
table with 1-sigma errors and writes a plot with confidence shading.

Metric per CLAUDE.md: TICC chA alone, detrended (absolute DO phase stability
the downstream consumer sees), NOT chA-chB.

The 1-sigma band is allantools' TDEV error estimate, which widens at long tau
where a ~3 h run yields few independent estimates — exactly the uncertainty
to show.
"""
import csv
import sys
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np
import allantools
from scipy.stats import chi2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
from plot_chA_tdev_goldilocks import load_chA_phase_s  # noqa: E402

ARMS = ["default", "no-extint", "no-ticc"]
LBL = {"default": "default (TICC+EXTINT)",
       "no-extint": "no-extint (TICC only)",
       "no-ticc": "no-ticc (EXTINT only)"}
COL = {"default": "C0", "no-extint": "C1", "no-ticc": "C2"}
BASE = "/home/bob/gt/threearm-analysis/clkpoc3-{}-ticc.csv"
OUT = "/home/bob/gt/threearm-analysis/clkpoc3-tdev-3arm.png"
SKIP_S = 600
RATE = 1.0
TAUS = np.array([1, 2, 3, 5, 7, 10, 15, 20, 30, 50, 70, 100, 150, 200,
                 300, 500, 700, 1000, 1500, 2000], dtype=float)


def first_ts(path):
    for row in csv.DictReader(open(path)):
        if row["channel"] == "chA":
            return datetime.fromisoformat(
                row["host_timestamp"].replace("Z", "+00:00"))
    raise ValueError("no chA")


def chi2_ci(dev, edf, ci=0.6827):
    """Chi-squared CI for a deviation estimate with `edf` degrees of freedom.
    Returns (lo, hi).  Widens sharply as edf -> few."""
    p = (1 - ci) / 2
    lo = dev * np.sqrt(edf / chi2.ppf(1 - p, edf))
    hi = dev * np.sqrt(edf / chi2.ppf(p, edf))
    return lo, hi


res = {}
for arm in ARMS:
    p = BASE.format(arm)
    ph = load_chA_phase_s(Path(p), first_ts(p) + timedelta(seconds=SKIP_S))
    N = len(ph)
    taus, tdev, terr, ns = allantools.tdev(
        ph, rate=RATE, data_type="phase", taus=TAUS)
    # Honest CI from the number of INDEPENDENT (non-overlapping) tau-windows.
    # TDEV's 2nd difference spans 3 averaging factors, so a length-N record
    # holds ~N/m - 2 independent looks at a tau = m/rate fluctuation.  This
    # collapses at long tau, which is the uncertainty we want to show.
    edf = np.maximum(1.0, N / (taus * RATE) - 2.0)
    lo, hi = chi2_ci(tdev, edf)
    res[arm] = dict(taus=taus, tdev=tdev, lo=lo, hi=hi, edf=edf)

taus = res["default"]["taus"]
print(f"{'tau_s':>6s} {'edf':>5s} | " +
      " | ".join(f"{LBL[a].split()[0]:>24s}" for a in ARMS))
print("-" * 96)
for i, t in enumerate(taus):
    edf_i = res["default"]["edf"][i]
    cells = " | ".join(
        f"{res[a]['tdev'][i]*1e12:6.0f} [{res[a]['lo'][i]*1e12:5.0f},"
        f"{res[a]['hi'][i]*1e12:5.0f}]ps" for a in ARMS)
    print(f"{t:>6.0f} {edf_i:>5.0f} | {cells}")

# Plot with chi-squared 1-sigma confidence shading (widens at long tau).
plt.figure(figsize=(9.5, 6.5))
for a in ARMS:
    r = res[a]
    plt.loglog(r["taus"], r["tdev"] * 1e12, "o-", color=COL[a], label=LBL[a],
               ms=4, zorder=3)
    plt.fill_between(r["taus"], r["lo"] * 1e12, r["hi"] * 1e12,
                    color=COL[a], alpha=0.18, zorder=1)
plt.xlabel("τ (s)")
plt.ylabel("chA TDEV (ps)")
plt.title("clkPoC3 (clean-EXTINT control) — chA TDEV by servo arm\n"
          "3-arm test 2026-06-18/19 · 10 min convergence skipped · "
          "68% χ² band from independent-window count (collapses at long τ)")
plt.grid(True, which="both", alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig(OUT, dpi=120)
print(f"\nwrote {OUT}")

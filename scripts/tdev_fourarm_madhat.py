#!/usr/bin/env python3
"""MadHat four-arm F9T-20B chA TDEV — the real (detrended chA-alone) comparison.

Metric per CLAUDE.md: TICC chA alone, detrended.  Sub-decade tau, chi^2
confidence bands from the independent-window count (3 h arms).

Arms: ticc-hwqerr (TICC hardware qErr, --router-qvir) / ticc-swqerr (TICC sw
qErr) / no-ticc (EXTINT only, level-shifted) / default (TICC+EXTINT).

The `default` arm is CAPPED at the ceiling-fan-on time (2026-06-21 00:40 UTC =
19:40 CDT) — the post-fan tail is a thermal confound (intermittent: on 19:40 /
off / on 19:48); excluded so it doesn't poison the arm's long-tau TDEV.
"""
import csv
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np
import allantools
from scipy.stats import chi2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
from plot_chA_tdev_goldilocks import load_chA_phase_s  # noqa: E402

BASE = "/home/bob/gt/fourarm-madhat-20260620-080233"
ARMS = ["ticc-hwqerr", "ticc-swqerr", "no-ticc", "default"]
SKIP_S = 600
# default-arm fan-on cutoff (UTC); rows at/after this are excluded.
FAN_ON_UTC = datetime(2026, 6, 21, 0, 40, 1, tzinfo=timezone.utc)
TAUS = np.array([1, 2, 5, 10, 20, 50, 100, 200, 500, 1000], dtype=float)


def first_ts(path):
    for row in csv.DictReader(open(path)):
        if row["channel"] == "chA":
            return datetime.fromisoformat(row["host_timestamp"].replace("Z", "+00:00"))
    raise ValueError("no chA")


def load_capped(path, skip_before, cap_after=None):
    """chA phase (s), detrended, between skip_before and cap_after."""
    secs, totals = [], []
    _PS = 1_000_000_000_000
    for row in csv.DictReader(open(path)):
        if row["channel"] != "chA":
            continue
        ts = datetime.fromisoformat(row["host_timestamp"].replace("Z", "+00:00"))
        if ts < skip_before:
            continue
        if cap_after is not None and ts >= cap_after:
            break
        secs.append(int(row["ref_sec"]))
        totals.append(int(row["ref_sec"]) * _PS + int(row["ref_ps"]))
    if len(totals) < 100:
        raise ValueError(f"{path}: too few samples ({len(totals)})")
    s = np.array(secs, dtype=np.int64)
    x = (s - s[0]).astype(np.float64)
    y = np.array([t - totals[0] for t in totals], dtype=np.float64)
    # A single GLOBAL linear detrend over a 3h arm can leave slow thermal
    # curvature in the residual, which inflates long-tau (>~500s) TDEV
    # (bravo #207).  The fan-on cap on the default arm removes the worst
    # case; the long-tau numbers on the other arms should be read as upper
    # bounds.  The robust comparisons (Q1 mid-tau, Q2 short-tau crossover)
    # are below the curvature scale and unaffected.
    slope, intc = np.polyfit(x, y, 1)
    return (y - (slope * x + intc)) * 1e-12


def chi2_ci(dev, edf, ci=0.6827):
    p = (1 - ci) / 2
    return dev * np.sqrt(edf / chi2.ppf(1 - p, edf)), dev * np.sqrt(edf / chi2.ppf(p, edf))


res = {}
for arm in ARMS:
    p = f"{BASE}/{arm}/ticc.csv"
    sb = first_ts(p) + timedelta(seconds=SKIP_S)
    cap = FAN_ON_UTC if arm == "default" else None
    ph = load_capped(p, sb, cap)
    taus, td, _, _ = allantools.tdev(ph, rate=1.0, data_type="phase", taus=TAUS)
    edf = np.maximum(1.0, len(ph) / taus - 2.0)
    lo, hi = chi2_ci(td, edf)
    res[arm] = dict(taus=taus, td=td, lo=lo, hi=hi, n=len(ph))
    note = " (capped at fan-on 19:40)" if arm == "default" else ""
    print(f"# {arm:12s} n={len(ph)} ({len(ph)/3600:.2f} h){note}", file=sys.stderr)

print(f"\n{'tau_s':>6s} | " + " | ".join(f"{a:>12s}" for a in ARMS))
print("-" * 72)
for i, t in enumerate(TAUS):
    cells = " | ".join(
        f"{res[a]['td'][i]*1e12:8.0f}    " if i < len(res[a]["td"]) else f"{'-':>12s}"
        for a in ARMS)
    print(f"{t:>6.0f} | " + cells)

# Q1: hw vs sw qErr ratio; Q2: EXTINT(no-ticc) vs TICC arms.
print("\nQ1 (hw/sw qErr ratio, <1 = hw better):")
for i, t in enumerate(TAUS):
    hw, sw = res["ticc-hwqerr"]["td"][i], res["ticc-swqerr"]["td"][i]
    print(f"   tau={t:>5.0f}s: hw={hw*1e12:6.0f} sw={sw*1e12:6.0f}  ratio={hw/sw:.2f}")

import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
plt.figure(figsize=(9, 6))
col = {"ticc-hwqerr": "C3", "ticc-swqerr": "C2", "no-ticc": "C1", "default": "C0"}
for a in ARMS:
    r = res[a]
    lab = a + (" (pre-fan)" if a == "default" else "")
    plt.loglog(r["taus"], r["td"]*1e12, "o-", color=col[a], label=lab, ms=4)
    plt.fill_between(r["taus"], r["lo"]*1e12, r["hi"]*1e12, color=col[a], alpha=0.15)
plt.xlabel("τ (s)"); plt.ylabel("chA TDEV (ps)")
plt.title("MadHat four-arm F9T-20B — chA TDEV by arm\n"
          "2026-06-20, 68% χ² bands; default capped at fan-on 19:40 CDT")
plt.grid(True, which="both", alpha=0.3); plt.legend()
plt.tight_layout()
out = "/home/bob/gt/fourarm-madhat-tdev.png"
plt.savefig(out, dpi=120)
print(f"\nwrote {out}")

#!/usr/bin/env python3
"""Interleaved Q1/Q2 chA-TDEV — time-of-day-common-mode comparison.

Each arm was sampled in multiple short (~25min) segments spread across the run.
Method: compute chA TDEV PER SEGMENT (detrend each separately, skip the
post-restart convergence), then AVERAGE the per-segment TDEV across same-arm
segments.  Because segments are interleaved in time, the time-of-day exposure is
common-mode between arms — the fix for the sequential-arm confound.

Segments are ~20min usable → TDEV is trustworthy to tau ~ 100-200s (covers Q1's
mid-tau and the short side of Q2's crossover; long-tau needs a non-interleaved
run).  Per-segment spread is reported as the reproducibility band.
"""
import csv
import glob
import sys
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np
import allantools

HOSTS = {
    "MadHat(F9T,LS)":   "/home/bob/gt/interleave-madhat-20260620-212712",
    "PiFace(F9T,OD)":   "/home/bob/gt/interleave-piface-20260620-212713",
    "clkPoC3(X20P)":    "/home/bob/gt/interleave-clkpoc3-20260620-212714",
}
ARMS = ["hwqerr", "swqerr", "noticc"]
SKIP_S = 300
TAUS = np.array([1, 2, 5, 10, 20, 50, 100, 200], dtype=float)
_PS = 1_000_000_000_000


def seg_tdev(path):
    """Per-segment chA TDEV (detrended), or None if too short."""
    secs, totals = [], []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            if row["channel"] != "chA":
                continue
            secs.append(int(row["ref_sec"]))
            totals.append(int(row["ref_sec"]) * _PS + int(row["ref_ps"]))
    if len(totals) < SKIP_S + 400:
        return None
    s = np.array(secs[SKIP_S:], dtype=np.int64)
    y = np.array([t - totals[SKIP_S] for t in totals[SKIP_S:]], dtype=np.float64)
    x = (s - s[0]).astype(np.float64)
    sl, ic = np.polyfit(x, y, 1)
    ph = (y - (sl * x + ic)) * 1e-12
    taus, td, _, _ = allantools.tdev(ph, rate=1.0, data_type="phase", taus=TAUS)
    out = np.full(len(TAUS), np.nan)
    for i, t in enumerate(TAUS):
        j = np.argmin(np.abs(taus - t))
        if abs(taus[j] - t) < 0.5:
            out[i] = td[j]
    return out


res = {}   # (host, arm) -> (mean_tdev[ntau], nseg)
for host, base in HOSTS.items():
    for arm in ARMS:
        segs = sorted(glob.glob(f"{base}/{arm}-c*/ticc.csv"))
        curves = [c for c in (seg_tdev(p) for p in segs) if c is not None]
        if not curves:
            continue
        arr = np.vstack(curves)
        res[(host, arm)] = (np.nanmean(arr, axis=0), len(curves))

# table
for host in HOSTS:
    print(f"\n=== {host} — interleaved chA TDEV (ps), per-segment mean ===")
    hdr = "  tau_s | " + " | ".join(f"{a:>10s}" for a in ARMS)
    print(hdr); print("  " + "-" * (len(hdr) - 2))
    for i, t in enumerate(TAUS):
        cells = []
        for a in ARMS:
            r = res.get((host, a))
            cells.append(f"{r[0][i]*1e12:8.0f}  " if r and not np.isnan(r[0][i]) else f"{'-':>10s}")
        print(f"  {t:>5.0f} | " + " | ".join(cells))
    nseg = {a: res[(host, a)][1] for a in ARMS if (host, a) in res}
    print(f"  (segments averaged: {nseg})")

# Q1: hw/sw ratio per F9T host (time-of-day common-mode now)
print("\n--- Q1 (hw/sw qErr ratio, <1 = hw better; interleaved, de-confounded) ---")
for host in HOSTS:
    if (host, "hwqerr") in res and (host, "swqerr") in res:
        hw, sw = res[(host, "hwqerr")][0], res[(host, "swqerr")][0]
        row = "  ".join(f"τ{int(TAUS[i])}={hw[i]/sw[i]:.2f}"
                        for i in range(len(TAUS)) if not np.isnan(hw[i]) and not np.isnan(sw[i]))
        print(f"  {host}: {row}")

# Q2: EXTINT(noticc) vs best TICC, per host
print("\n--- Q2 (noticc EXTINT / best-TICC ratio, <1 = EXTINT better) ---")
for host in HOSTS:
    if (host, "noticc") not in res:
        continue
    ex = res[(host, "noticc")][0]
    ticcs = [res[(host, a)][0] for a in ("hwqerr", "swqerr") if (host, a) in res]
    best = np.nanmin(np.vstack(ticcs), axis=0) if ticcs else None
    if best is None:
        continue
    row = "  ".join(f"τ{int(TAUS[i])}={ex[i]/best[i]:.2f}"
                    for i in range(len(TAUS)) if not np.isnan(ex[i]) and not np.isnan(best[i]))
    print(f"  {host}: {row}")

# Level-shifter probe: MadHat(LS) vs PiFace(OD) noticc, both F9T
print("\n--- Level-shifter probe: noticc (EXTINT) MadHat(LS) vs PiFace(OD), both F9T ---")
m = res.get(("MadHat(F9T,LS)", "noticc")); p = res.get(("PiFace(F9T,OD)", "noticc"))
if m and p:
    for i, t in enumerate(TAUS):
        if not np.isnan(m[0][i]) and not np.isnan(p[0][i]):
            print(f"  τ={int(t):>4}s: MadHat={m[0][i]*1e12:6.0f}ps  PiFace={p[0][i]*1e12:6.0f}ps  "
                  f"ratio={p[0][i]/m[0][i]:.2f}")

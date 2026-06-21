#!/usr/bin/env python3
"""latestQErrChiSelect A/B chA-TDEV parity analysis.

Per-segment chA-TDEV averaged across same-arm segments (interleaved → time-of-day
common-mode), comparing --qerr-latest-chi (NEW comparative-gate, latest-unmatched
qErr) vs --router-qvir (matched qErr).  Question: does latest-chi reach PARITY
with matched qvir at the qErr-relevant short/mid-tau?

Segments ~20min usable → tau to ~200s.  Per-segment std = the band; parity is
read as |latestchi - qvir| within the combined band.
"""
import csv
import glob
import sys
from pathlib import Path

import numpy as np
import allantools

HOSTS = {
    "MadHat(F9T,hwqErr)": "/home/bob/gt/qerrab-madhat-20260621-150945",
    "PiFace(F9T)":        "/home/bob/gt/qerrab-piface-20260621-151712",
    "clkPoC3(X20P)":      "/home/bob/gt/qerrab-clkpoc3-20260621-150948",
}
ARMS = ["latestchi", "qvir"]
SKIP = 300
TAUS = np.array([1, 2, 5, 10, 20, 50, 100, 200], dtype=float)
_PS = 1_000_000_000_000


def seg_tdev(path):
    secs, tot = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            if row["channel"] != "chA":
                continue
            secs.append(int(row["ref_sec"]))
            tot.append(int(row["ref_sec"]) * _PS + int(row["ref_ps"]))
    if len(tot) < SKIP + 400:
        return None
    s = np.array(secs[SKIP:], dtype=np.int64)
    y = np.array([t - tot[SKIP] for t in tot[SKIP:]], dtype=np.float64)
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


res = {}
for host, base in HOSTS.items():
    for arm in ARMS:
        curves = [c for c in (seg_tdev(p) for p in sorted(glob.glob(f"{base}/{arm}-c*/ticc.csv"))) if c is not None]
        if curves:
            a = np.vstack(curves)
            res[(host, arm)] = (np.nanmean(a, axis=0), np.nanstd(a, axis=0), len(curves))

for host in HOSTS:
    print(f"\n=== {host} — chA TDEV (ps), mean±per-segment-std ===")
    print("  tau_s |   latestchi(new) |    qvir(matched) | latest/qvir")
    print("  " + "-" * 56)
    for i, t in enumerate(TAUS):
        lc = res.get((host, "latestchi")); qv = res.get((host, "qvir"))
        if not (lc and qv):
            continue
        l, ls = lc[0][i] * 1e12, lc[1][i] * 1e12
        q, qs = qv[0][i] * 1e12, qv[1][i] * 1e12
        ratio = l / q if q else float("nan")
        print(f"  {t:>5.0f} | {l:7.0f} ±{ls:5.0f}   | {q:7.0f} ±{qs:5.0f}   | {ratio:5.2f}")
    nseg = {a: res[(host, a)][2] for a in ARMS if (host, a) in res}
    print(f"  (segments: {nseg})")

# Parity verdict per host: is |latest - qvir| within the combined std at each tau?
print("\n--- PARITY (latest-chi vs qvir; within combined band = parity) ---")
for host in HOSTS:
    lc = res.get((host, "latestchi")); qv = res.get((host, "qvir"))
    if not (lc and qv):
        continue
    within = []
    for i in range(len(TAUS)):
        d = abs(lc[0][i] - qv[0][i])
        band = lc[1][i] + qv[1][i]
        within.append(d <= band or d < 0.05e-9)  # within band, or <50ps abs
    npar = sum(within)
    print(f"  {host}: parity at {npar}/{len(TAUS)} tau "
          f"(mean ratio {np.nanmean(lc[0]/qv[0]):.2f})")

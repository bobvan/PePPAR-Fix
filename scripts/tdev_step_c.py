#!/usr/bin/env python3
"""STEP C per-arm long-τ chA-TDEV (midTauTrackingResidual I-102153).

Per-arm chA-alone detrended TDEV (CLAUDE.md metric; NOT chA-chB), averaged
across same-arm interleaved segments (time-of-day common-mode), with χ² bands.
Compares the realization arms (baseline / coast / stiffQ / both) against the
host's free-running DO floor (the target the loop should approach at mid-τ).

Usage:
    tdev_step_c.py <base_dir> [--floor 199,352,990,2728]   # freerun 100/200/500/1000s
base_dir/<arm>-c<N>/ticc.csv  per interleaved segment (matches step_c_driver.py).
ANALYSIS gitsha is stamped in the header.
"""
import csv
import glob
import subprocess
import sys
from pathlib import Path

import numpy as np
import allantools
from scipy.stats import chi2

ARMS = ["baseline", "coast", "stiffQ", "both"]
TAUS = np.array([1, 2, 5, 10, 20, 50, 100, 200, 500, 1000], dtype=float)
SKIP_S = 300          # drop post-restart convergence per segment
_PS = 1_000_000_000_000
BUDGET_PS = 354.0


def _gitsha():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], text=True).strip()
    except Exception:
        return "unknown"


def seg_tdev(path, taus):
    """Per-segment chA TDEV (chA-alone, detrended on ref_sec), or None."""
    secs, tot = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            if row.get("channel") != "chA":
                continue
            secs.append(int(row["ref_sec"]))
            tot.append(int(row["ref_sec"]) * _PS + int(row["ref_ps"]))
    if len(tot) < SKIP_S + 400:
        return None
    s = np.array(secs[SKIP_S:], dtype=np.int64)
    y = np.array([t - tot[SKIP_S] for t in tot[SKIP_S:]], dtype=np.float64)
    x = (s - s[0]).astype(np.float64)
    sl, ic = np.polyfit(x, y, 1)                 # detrend on ref_sec, not index
    ph = (y - (sl * x + ic)) * 1e-12
    tt, td, _, _ = allantools.tdev(ph, rate=1.0, data_type="phase", taus=taus)
    out = np.full(len(taus), np.nan)
    for i, t in enumerate(taus):
        j = int(np.argmin(np.abs(tt - t)))
        if abs(tt[j] - t) < 0.5:
            out[i] = td[j]
    return out


def main():
    if len(sys.argv) < 2:
        print(__doc__); return 2
    base = sys.argv[1]
    floor = None
    if "--floor" in sys.argv:
        floor = [float(v) for v in sys.argv[sys.argv.index("--floor") + 1].split(",")]
    print(f"# STEP C per-arm chA-TDEV — {base}  (gitsha {_gitsha()})\n")
    res = {}
    for arm in ARMS:
        curves = [c for c in (seg_tdev(p, TAUS)
                              for p in sorted(glob.glob(f"{base}/{arm}-c*/ticc.csv")))
                  if c is not None]
        if curves:
            a = np.vstack(curves)
            res[arm] = (np.nanmean(a, axis=0), np.nanstd(a, axis=0), len(curves))
    if not res:
        print("no per-arm segments found under", base); return 1
    hdr = "  ".join(f"{a:>11}" for a in res)
    print(f"{'tau_s':>6} | {hdr} | {'floor':>7} | budget")
    print("-" * (10 + 14 * len(res) + 18))
    for i, t in enumerate(TAUS):
        cells = []
        for a in res:
            m, sd, _ = res[a]
            cells.append(f"{m[i]*1e12:6.0f}±{sd[i]*1e12:<4.0f}")
        fl = f"{floor[[100,200,500,1000].index(int(t))]:6.0f}" if (
            floor and int(t) in (100, 200, 500, 1000)) else "     —"
        print(f"{t:>6.0f} | " + "  ".join(f"{c:>11}" for c in cells)
              + f" | {fl:>7} | {BUDGET_PS:5.0f}")
    print("\nsegments/arm:", {a: res[a][2] for a in res})
    # verdict: which arm minimizes mid-τ (mean over 100-1000s) vs baseline
    midi = [i for i, t in enumerate(TAUS) if 100 <= t <= 1000]
    base_mid = np.nanmean(res["baseline"][0][midi]) if "baseline" in res else None
    print("\nmid-τ (100–1000s) mean chA-TDEV per arm vs baseline:")
    for a in res:
        m = np.nanmean(res[a][0][midi]) * 1e12
        rel = f" ({m/(base_mid*1e12):.2f}× baseline)" if base_mid else ""
        flag = " ✓≤budget" if m <= BUDGET_PS else ""
        print(f"  {a:>9}: {m:6.0f} ps{rel}{flag}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

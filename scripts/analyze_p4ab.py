#!/usr/bin/env python3
"""Grade the P4 combo-servo A/B: chA−chB detrended TDEV per arm, combo vs
timebeat-baseline.  chA=GNSSDO+ (Rb ref), chB=otcBob1 PPS OUT (DUT); the
ref-immune differential is chA−chB.  Per CLAUDE.md: detrend on ref_sec (not
index), pair by ref_sec, TDEV reported in ns, guard >1 µs.
"""
import csv
import sys
from datetime import datetime, timezone

import numpy as np
import allantools

CAP = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/bob/gt/datasheets/p4-otcbob1-ab/p4ab-2026-06-25.csv"
BND = sys.argv[2] if len(sys.argv) > 2 else \
    "/home/bob/gt/datasheets/p4-otcbob1-ab/ab_boundaries.csv"
TRIM_S = 60   # drop the first 60 s of each arm (settle/convergence transient)


def _t(s):
    return datetime.fromisoformat(s.replace("Z", "+00:00")).timestamp()


# arm windows from the boundary marks
events = {}
with open(BND) as f:
    for row in csv.DictReader(f):
        events[row["event"]] = _t(row["boundary_utc"])
ARMS = [("A1 timebeat", "A1_start", "A1_end"),
        ("B1 combo",    "B1_start", "B1_end"),
        ("A2 timebeat", "A2_start", "A2_end"),
        ("B2 combo",    "B2_start", "B2_end")]

# load capture: per-channel {ref_sec: ref_ps}, plus epoch time per ref_sec
A, B, EP = {}, {}, {}
with open(CAP) as f:
    for line in f:
        if line[0] == "#" or line.startswith("ts_iso"):
            continue
        p = line.rstrip().split(",")
        if len(p) < 4:
            continue
        try:
            ep = _t(p[0]); ch = p[1]; sec = int(p[2]); ps = int(p[3])
        except ValueError:
            continue
        (A if ch == "chA" else B)[sec] = ps
        EP[sec] = ep


def tdev_ns(diff_ps, rate=1.0):
    t, d, _, _ = allantools.tdev(np.asarray(diff_ps) * 1e-12, rate=rate,
                                 data_type="phase", taus="octave")
    return {int(x): y * 1e9 for x, y in zip(t, d)}   # ns


results = {}
for label, ev0, ev1 in ARMS:
    lo, hi = events[ev0] + TRIM_S, events[ev1]
    secs = sorted(s for s in (A.keys() & B.keys())
                  if lo <= EP[s] <= hi)
    if len(secs) < 40:
        print(f"{label:12s}: too few paired samples ({len(secs)})")
        continue
    diff = np.array([A[s] - B[s] for s in secs], float)        # chA−chB, ps
    x = np.array(secs, float)
    # detrend on ref_sec (remove constant offset + slow drift)
    dd = diff - np.polyval(np.polyfit(x, diff, 1), x)
    slope = np.polyfit(x, diff, 1)[0]
    td = tdev_ns(dd)
    results[label] = td
    print(f"\n=== {label}  (n={len(secs)}, drift={slope:+.1f} ps/s, "
          f"std={dd.std():.0f} ps) ===")
    print("  TDEV ns:", {k: round(v, 3) for k, v in td.items()
                         if v < 1000})   # guard >1 µs


# combo vs timebeat comparison at shared taus
print("\n=== COMBO vs TIMEBEAT (chA−chB TDEV ns) ===")
tb = {**results.get("A1 timebeat", {}), **results.get("A2 timebeat", {})}
cb = {**results.get("B1 combo", {}), **results.get("B2 combo", {})}
A_all = [results[k] for k in ("A1 timebeat", "A2 timebeat") if k in results]
B_all = [results[k] for k in ("B1 combo", "B2 combo") if k in results]
taus = sorted(set().union(*[set(r) for r in A_all + B_all]))
print(f"{'tau_s':>7} {'timebeat':>12} {'combo':>12} {'ratio c/t':>10}  verdict")
for tau in taus:
    tvals = [r[tau] for r in A_all if tau in r and r[tau] < 1000]
    cvals = [r[tau] for r in B_all if tau in r and r[tau] < 1000]
    if not tvals or not cvals:
        continue
    tmean, cmean = np.mean(tvals), np.mean(cvals)
    ratio = cmean / tmean
    verdict = "WIN" if ratio < 0.95 else ("tie" if ratio < 1.15 else "lose")
    print(f"{tau:>7} {tmean:>10.3f}ns {cmean:>10.3f}ns {ratio:>9.2f}x  {verdict}")


# ── TDEV overlay plot ─────────────────────────────────────────────── #
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 5.5))
    for k, c, m in (("A1 timebeat", "#c62828", "o"), ("A2 timebeat", "#ef6c00", "o"),
                    ("B1 combo", "#1565c0", "s"), ("B2 combo", "#2e7d32", "s")):
        if k in results:
            r = {t: v for t, v in results[k].items() if v < 1000}
            ax.loglog(list(r), list(r.values()), m + "-", color=c, label=k, alpha=0.8)
    ax.axhline(0.354, ls=":", color="grey", label="0.354 ns per-clock budget")
    ax.set_xlabel("τ (s)"); ax.set_ylabel("chA−chB TDEV (ns)")
    ax.set_title("P4: combo servo vs hardware DPLL — otcBob1 PPS OUT vs GNSSDO+ (TICC#4)\n"
                 "combo WINS τ=1–16s (carrier-phase ref), loses long-τ")
    ax.grid(True, which="both", alpha=0.3); ax.legend(fontsize=8)
    ax.annotate("ANALYSIS_VERSION=p4ab-1  git=" +
                __import__("subprocess").getoutput("git -C %s rev-parse --short HEAD"
                % __import__("os").path.dirname(__file__)),
                xy=(0.5, -0.13), xycoords="axes fraction", ha="center", fontsize=7, color="grey")
    out = "/home/bob/gt/datasheets/p4-otcbob1-ab/p4ab_tdev.png"
    fig.tight_layout(); fig.savefig(out, dpi=120)
    print(f"\nwrote {out}")
except Exception as e:  # noqa: BLE001
    print(f"(plot skipped: {e})")

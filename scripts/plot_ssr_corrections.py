#!/usr/bin/env python3
"""plot_ssr_corrections.py — per-component SSR-correction figures from a
log_ssr_corrections.py CSV (see docs/visual-stories.md "SSR corrections").

Four separate figures, one per correction component, each:
  - **detrended** (per-SV mean removed) so variance sits symmetrically about
    zero and shows the correction's *character*, not the per-SV datum offset;
  - on a **common vertical scale** (default ±10 ns) so magnitudes can be
    compared across the four figures; a trace exceeding the scale is clipped
    and its peak is called out in the title rather than hidden;
  - clock & orbit get a **gap-break** — SSR coverage dropouts render as gaps,
    not held-value flat lines. Code/phase bias are genuinely quasi-static, so a
    value-change detector can't tell "held fresh" from "stale"; they are NOT
    gap-broken (a real bias-gap detector needs correction age, a future logger
    field).

Outputs: ssr_clock.png, ssr_orbit.png, ssr_codebias.png, ssr_phasebias.png.
x-axis is the full capture by default; --hours sets a finer common window.

Usage:
    python3 scripts/plot_ssr_corrections.py ssr_corrections.csv --out-dir plots
"""
import argparse
import csv
import os
import sys
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DEFAULT_SVS = ["G24", "G04", "E26", "E36", "R02", "R20", "C21", "C36"]
SYS_COLOR = {"G": "tab:blue", "E": "tab:green", "R": "tab:red", "C": "tab:orange"}
SYS_NAME = {"G": "GPS", "E": "Galileo", "R": "GLONASS", "C": "BeiDou"}
PREF_CODE = {"G": ["C1C", "C1W", "C1P"], "E": ["C1C", "C1B"],
             "R": ["C1C", "C1P"], "C": ["C2I", "C1I", "C5I"]}
PREF_PHASE = {"G": ["L1C", "L1W"], "E": ["L1C", "L1B"],
              "R": ["L1C", "L1P"], "C": ["L2I", "L1I", "L5I"]}
GAP_THRESH_S = 60.0   # value held longer than this = SSR coverage dropout, not data
YLIM_NS = 10.0        # common vertical scale across all four figures

# (component, title, signal-pref map, gap_break, output filename)
PANELS = [
    ("clock_c0",     "clock c0",     None,       True,  "ssr_clock.png"),
    ("orbit_radial", "orbit radial", None,       True,  "ssr_orbit.png"),
    ("code_bias",    "code bias",    PREF_CODE,  False, "ssr_codebias.png"),
    ("phase_bias",   "phase bias",   PREF_PHASE, False, "ssr_phasebias.png"),
]


def _break_stale(t, v, thresh=GAP_THRESH_S):
    """NaN samples held > thresh s, so SSR coverage dropouts render as gaps
    rather than held-value flat lines (the logger snapshots live state and
    holds the last value across a dropout). Valid only for components that
    change every update (clock, orbit)."""
    out = np.asarray(v, dtype=float).copy()
    orig = np.asarray(v, dtype=float)
    run_start = t[0]
    for i in range(1, len(out)):
        if orig[i] != orig[i - 1]:
            run_start = t[i]
        elif t[i] - run_start > thresh:
            out[i] = np.nan
    return out


def load(path):
    """Return (data, t0) where data[(prn, component, signal)] = (t_s, ns)."""
    ts = defaultdict(list)
    vs = defaultdict(list)
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["mono_s"])
                v = float(row["value_ns"])
            except (ValueError, KeyError):
                continue
            key = (row["prn"], row["component"], row["signal"])
            ts[key].append(t)
            vs[key].append(v)
    data = {}
    mins = []
    for key in ts:
        t = np.asarray(ts[key])
        v = np.asarray(vs[key])
        order = np.argsort(t)
        data[key] = (t[order], v[order])
        mins.append(t.min())
    return data, (min(mins) if mins else 0.0)


def pick_signal(data, prn, component, pref):
    avail = sorted(k[2] for k in data if k[0] == prn and k[1] == component and k[2])
    for s in pref.get(prn[0], []):
        if s in avail:
            return s
    return avail[0] if avail else None


def fig_component(data, t0, span_h, svs, component, title, pref, gap_break,
                  out, ylim=YLIM_NS, hours=None):
    win = hours if hours else span_h
    fig, ax = plt.subplots(figsize=(12, 5))
    offscale = []
    for prn in svs:
        if pref:
            sig = pick_signal(data, prn, component, pref)
            if not sig:
                continue
            key, lbl = (prn, component, sig), f"{prn} {sig}"
        else:
            key, lbl = (prn, component, ""), f"{prn} ({SYS_NAME[prn[0]]})"
        if key not in data:
            continue
        t, v = data[key]
        if gap_break:
            v = _break_stale(t, v)
        v = v - np.nanmean(v)
        th = (t - t0) / 3600.0
        m = th <= win + 1e-9
        if not np.any(m):
            continue
        ax.plot(th[m], v[m], lw=0.8, color=SYS_COLOR[prn[0]], alpha=0.85, label=lbl)
        peak = np.nanmax(np.abs(v[m]))
        if peak > ylim:
            offscale.append((prn, peak))
    ax.axhline(0, color="gray", lw=0.4)
    ax.set_ylim(-ylim, ylim)
    ax.set_xlim(0, win)
    ax.set_xlabel(f"hours since capture start  (window {win:.1f} h, 10 s snapshots)")
    ax.set_ylabel(f"{title} − per-SV mean  [ns]")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=7, ncol=4)
    sub = ("gaps = no SSR correction provided (coverage dropout)" if gap_break
           else "quasi-static — held between rare updates (not gap-broken)")
    note = ("   •   off-scale: " + ", ".join(f"{p} peaks ±{m:.0f} ns" for p, m in offscale)
            if offscale else "")
    ax.set_title(f"SSR {title} by constellation — detrended, common ±{ylim:.0f} ns scale\n"
                 f"{sub}{note}", fontsize=11)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"wrote {out}" + (f"   off-scale={[(p, round(m,1)) for p,m in offscale]}"
                            if offscale else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="CSV from log_ssr_corrections.py")
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--svs", default=",".join(DEFAULT_SVS),
                    help="comma-separated SVs (1-2 per constellation)")
    ap.add_argument("--ylim", type=float, default=YLIM_NS,
                    help="common vertical scale, ±ns (default 10)")
    ap.add_argument("--hours", type=float, default=None,
                    help="common x-axis window in hours (default: full capture)")
    args = ap.parse_args()

    data, t0 = load(args.csv)
    if not data:
        print("no data parsed", file=sys.stderr)
        return 2
    svs = [s.strip() for s in args.svs.split(",") if s.strip()]
    span_h = (max(t.max() for t, _ in data.values())
              - min(t.min() for t, _ in data.values())) / 3600.0
    print(f"loaded {len(data)} series, span {span_h:.2f} h")
    os.makedirs(args.out_dir, exist_ok=True)
    for component, title, pref, gap_break, name in PANELS:
        fig_component(data, t0, span_h, svs, component, title, pref, gap_break,
                      os.path.join(args.out_dir, name), ylim=args.ylim, hours=args.hours)
    return 0


if __name__ == "__main__":
    sys.exit(main())

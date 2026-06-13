#!/usr/bin/env python3
"""plot_ssr_corrections.py — SSR-correction figures from a log_ssr_corrections.py
CSV (see docs/visual-stories.md "SSR corrections").

Five figures, sized 16:9 with large fonts for auditorium projection, each as
PDF (vector) + PNG:

  ssr_clock / ssr_orbit / ssr_codebias / ssr_phasebias — one correction
    component each, detrended (per-SV mean removed) so variance is symmetric
    about zero (character, not datum), on a common **±5 ns** scale so
    magnitudes compare across figures.  GLONASS deliberately clips; any figure
    with clipping carries a per-constellation **peak-|value| table** so the
    real ranges are still on the slide.  Clock & orbit are gap-broken
    (coverage dropouts -> gaps); code/phase bias are quasi-static and not.

  ssr_broadcast_error — the with/without-corrections contrast: per-SV broadcast
    clock+orbit *time error* (= the SSR correction, i.e. what a broadcast-only
    filter suffers; steps are broadcast-ephemeris/IOD transitions) vs the
    SSR-corrected residual (~0, the black line).  Phase bias is excluded — it
    isn't a broadcast quantity, so a no-SSR filter never sees its jumps.

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

# Large fonts for projection (~2x matplotlib defaults).
plt.rcParams.update({
    "font.size": 18,
    "axes.titlesize": 24,
    "axes.labelsize": 22,
    "xtick.labelsize": 20,
    "ytick.labelsize": 20,
    "legend.fontsize": 17,
})

DEFAULT_SVS = ["G24", "G04", "E26", "E36", "R02", "R20", "C21", "C36"]
SYS_COLOR = {"G": "tab:blue", "E": "tab:green", "R": "tab:red", "C": "tab:orange"}
SYS_NAME = {"G": "GPS", "E": "Galileo", "R": "GLONASS", "C": "BeiDou"}
CONST_ORDER = [("G", "GPS"), ("E", "Galileo"), ("R", "GLONASS"), ("C", "BeiDou")]
PREF_CODE = {"G": ["C1C", "C1W", "C1P"], "E": ["C1C", "C1B"],
             "R": ["C1C", "C1P"], "C": ["C2I", "C1I", "C5I"]}
PREF_PHASE = {"G": ["L1C", "L1W"], "E": ["L1C", "L1B"],
              "R": ["L1C", "L1P"], "C": ["L2I", "L1I", "L5I"]}
GAP_THRESH_S = 60.0   # value held longer than this = SSR coverage dropout, not data
YLIM_NS = 5.0         # common vertical scale across figures

# (component, title, signal-pref map, gap_break, output basename)
PANELS = [
    ("clock_c0",     "clock c0",     None,       True,  "ssr_clock"),
    ("orbit_radial", "orbit radial", None,       True,  "ssr_orbit"),
    ("code_bias",    "code bias",    PREF_CODE,  False, "ssr_codebias"),
    ("phase_bias",   "phase bias",   PREF_PHASE, False, "ssr_phasebias"),
]


def _break_stale(t, v, thresh=GAP_THRESH_S):
    """NaN samples held > thresh s so SSR coverage dropouts render as gaps, not
    held-value flat lines.  Valid only for components that change every update
    (clock, orbit)."""
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


def _peak_table(ax, peaks, ylim, label="peak |value|"):
    """Per-constellation peak-|value| table (so clipped ranges stay on the slide)."""
    lines = [f"{label}:"]
    for s, name in CONST_ORDER:
        if s in peaks:
            mark = "  < clips" if peaks[s] > ylim else ""
            lines.append(f"{name:<8s}{peaks[s]:6.1f} ns{mark}")
    ax.text(0.012, 0.975, "\n".join(lines), transform=ax.transAxes,
            va="top", ha="left", family="monospace", fontsize=18,
            bbox=dict(boxstyle="round", fc="white", ec="0.5", alpha=0.92))


def _finish(fig, out):
    fig.tight_layout()
    fig.savefig(out + ".pdf")
    fig.savefig(out + ".png", dpi=200)
    plt.close(fig)


def fig_component(data, t0, span_h, svs, component, title, pref, gap_break,
                  out, ylim=YLIM_NS, hours=None):
    win = hours if hours else span_h
    fig, ax = plt.subplots(figsize=(16, 9))
    peaks = {}
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
        ax.plot(th[m], v[m], lw=1.0, color=SYS_COLOR[prn[0]], alpha=0.85, label=lbl)
        peaks[prn[0]] = max(peaks.get(prn[0], 0.0), float(np.nanmax(np.abs(v[m]))))
    ax.axhline(0, color="gray", lw=0.6)
    ax.set_ylim(-ylim, ylim)
    ax.set_xlim(0, win)
    ax.set_xlabel(f"hours since capture start   (window {win:.1f} h)")
    ax.set_ylabel(f"{title} − per-SV mean  [ns]")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", ncol=4)
    sub = ("gaps = no SSR correction (coverage dropout)" if gap_break
           else "quasi-static — held between rare updates (not gap-broken)")
    ax.set_title(f"SSR {title} by constellation — detrended, ±{ylim:.0f} ns\n{sub}")
    if any(p > ylim for p in peaks.values()):
        _peak_table(ax, peaks, ylim)
    _finish(fig, out)
    print(f"wrote {out}.pdf/.png   peaks={ {k: round(v,1) for k,v in peaks.items()} }")


def fig_broadcast_error(data, t0, span_h, svs, out, ylim=YLIM_NS, hours=None):
    win = hours if hours else span_h
    fig, ax = plt.subplots(figsize=(16, 9))
    peaks = {}
    for prn in svs:
        kc, ko = (prn, "clock_c0", ""), (prn, "orbit_radial", "")
        if kc not in data or ko not in data:
            continue
        tc, vc = data[kc]
        to, vo = data[ko]
        err = _break_stale(tc, vc) + np.interp(tc, to, vo)   # broadcast clock+orbit error
        err = err - np.nanmean(err)                          # drop the calibratable offset
        th = (tc - t0) / 3600.0
        m = th <= win + 1e-9
        if not np.any(m):
            continue
        ax.plot(th[m], err[m], lw=1.0, color=SYS_COLOR[prn[0]], alpha=0.85,
                label=f"{prn} ({SYS_NAME[prn[0]]})")
        peaks[prn[0]] = max(peaks.get(prn[0], 0.0), float(np.nanmax(np.abs(err[m]))))
    ax.axhline(0, color="black", lw=3.2, label="WITH SSR corrections (~0)")
    ax.set_ylim(-ylim, ylim)
    ax.set_xlim(0, win)
    ax.set_xlabel(f"hours since capture start   (window {win:.1f} h)")
    ax.set_ylabel("time error − per-SV mean  [ns]")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", ncol=3)
    ax.set_title("Time error WITHOUT (coloured) vs WITH (black) SSR corrections\n"
                 "broadcast clock+orbit error; steps = broadcast-ephemeris (IOD) transitions")
    _peak_table(ax, peaks, ylim, label="broadcast error peak")
    _finish(fig, out)
    print(f"wrote {out}.pdf/.png   peaks={ {k: round(v,1) for k,v in peaks.items()} }")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="CSV from log_ssr_corrections.py")
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--svs", default=",".join(DEFAULT_SVS))
    ap.add_argument("--ylim", type=float, default=YLIM_NS, help="common ±ns scale (default 5)")
    ap.add_argument("--hours", type=float, default=None, help="x window (default: full)")
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
    fig_broadcast_error(data, t0, span_h, svs,
                        os.path.join(args.out_dir, "ssr_broadcast_error"),
                        ylim=args.ylim, hours=args.hours)
    return 0


if __name__ == "__main__":
    sys.exit(main())

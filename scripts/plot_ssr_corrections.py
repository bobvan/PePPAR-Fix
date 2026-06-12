#!/usr/bin/env python3
"""plot_ssr_corrections.py — render the SSR-corrections visual story from a
CSV produced by log_ssr_corrections.py.

Two figures (see docs/visual-stories.md "SSR corrections" entry):

  ssr_components.png    — component contrast: 4 stacked panels (clock / orbit
                          radial / code bias / phase bias), all in ns, sharing
                          a time axis, one representative SV per constellation.
                          Shows the magnitude x timescale separation (clock is
                          the only fast mover; biases are quasi-static).
                          A zoom inset on the clock panel reveals the
                          seconds-scale noise.

  ssr_constellation_clock.png — constellation contrast: clock c0 in ns for the
                          cherry-picked SVs, coloured by constellation, to show
                          the clock-oscillator story (Galileo PHM clean <<
                          GPS Rb < GLONASS Cs).

Reads value_ns directly (1 m = 3.3356 ns). x-axis = hours since capture start.

Usage:
    python3 scripts/plot_ssr_corrections.py ssr_corrections.csv --out-dir plots
"""
import argparse
import csv
import sys
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Cherry-picked newer-generation SVs (one+ per constellation).
DEFAULT_SVS = ["G24", "G04", "E26", "E36", "R02", "R20", "C21", "C36"]
# One representative SV per constellation for the component-contrast figure.
DEFAULT_REP = ["G04", "E26", "R02", "C21"]

SYS_COLOR = {"G": "tab:blue", "E": "tab:green", "R": "tab:red", "C": "tab:orange"}
SYS_NAME = {"G": "GPS", "E": "Galileo", "R": "GLONASS", "C": "BeiDou"}

# Preferred L1-band reference signal per constellation for the bias panels.
PREF_CODE = {"G": ["C1C", "C1W", "C1P"], "E": ["C1C", "C1B"],
             "R": ["C1C", "C1P"], "C": ["C2I", "C1I", "C5I"]}
PREF_PHASE = {"G": ["L1C", "L1W"], "E": ["L1C", "L1B"],
              "R": ["L1C", "L1P"], "C": ["L2I", "L1I", "L5I"]}


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


def fig_components(data, t0, rep_svs, out):
    panels = [("clock_c0", "Clock c0", None),
              ("orbit_radial", "Orbit radial", None),
              ("code_bias", "Code bias", PREF_CODE),
              ("phase_bias", "Phase bias", PREF_PHASE)]
    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    span_h = 0.0
    for ax, (comp, title, pref) in zip(axes, panels):
        for prn in rep_svs:
            if pref:
                sig = pick_signal(data, prn, comp, pref)
                if not sig:
                    continue
                key, lbl = (prn, comp, sig), f"{prn} {sig}"
            else:
                key, lbl = (prn, comp, ""), prn
            if key not in data:
                continue
            t, v = data[key]
            th = (t - t0) / 3600.0
            span_h = max(span_h, th.max() if th.size else 0.0)
            ax.plot(th, v, lw=0.9, color=SYS_COLOR[prn[0]], label=lbl)
        ax.set_ylabel(f"{title}\n[ns]")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=7, ncol=4)
    axes[-1].set_xlabel("hours since capture start")
    fig.suptitle("SSR corrections — component contrast (ns)\n"
                 "clock is the only fast mover; orbit slow; biases quasi-static",
                 fontsize=12)

    # Zoom inset on the clock panel to reveal seconds-scale structure.
    if span_h > 0.25:  # only if we have >15 min of data
        try:
            win_h = min(0.5, span_h)
            axin = axes[0].inset_axes([0.58, 0.55, 0.40, 0.42])
            for prn in rep_svs:
                key = (prn, "clock_c0", "")
                if key not in data:
                    continue
                t, v = data[key]
                th = (t - t0) / 3600.0
                m = th <= win_h
                axin.plot(th[m], v[m], lw=0.8, color=SYS_COLOR[prn[0]])
            axin.set_title(f"clock zoom: first {win_h*60:.0f} min", fontsize=7)
            axin.tick_params(labelsize=6)
            axes[0].indicate_inset_zoom(axin, edgecolor="gray")
        except Exception as e:
            print(f"  (clock inset skipped: {e})", file=sys.stderr)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"wrote {out}")


def fig_constellation_clock(data, t0, svs, out):
    fig, ax = plt.subplots(figsize=(12, 6))
    seen = set()
    for prn in svs:
        key = (prn, "clock_c0", "")
        if key not in data:
            continue
        t, v = data[key]
        v = v - v.mean()              # center each SV: show clock character, not the datum
        th = (t - t0) / 3600.0
        sysc = prn[0]
        lbl = SYS_NAME[sysc] if sysc not in seen else None
        seen.add(sysc)
        ax.plot(th, v, lw=0.9, color=SYS_COLOR[sysc], alpha=0.85,
                label=f"{prn} ({SYS_NAME[sysc]})")
    ax.set_xlabel("hours since capture start")
    ax.set_ylabel("clock c0 − per-SV mean  [ns]")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=8, ncol=2)
    ax.set_title("SSR clock corrections by constellation — per-SV mean removed (ns)\n"
                 "clock *character*: Galileo PHM smooth << GPS Rb < GLONASS Cs",
                 fontsize=12)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="CSV from log_ssr_corrections.py")
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--svs", default=",".join(DEFAULT_SVS),
                    help="comma-separated SVs for the constellation figure")
    ap.add_argument("--rep-svs", default=",".join(DEFAULT_REP),
                    help="comma-separated representative SVs (1/constellation) "
                    "for the component figure")
    args = ap.parse_args()

    data, t0 = load(args.csv)
    if not data:
        print("no data parsed", file=sys.stderr)
        return 2
    svs = [s.strip() for s in args.svs.split(",") if s.strip()]
    rep = [s.strip() for s in args.rep_svs.split(",") if s.strip()]
    span_h = (max(t.max() for t, _ in data.values())
              - min(t.min() for t, _ in data.values())) / 3600.0
    print(f"loaded {len(data)} series, span {span_h:.2f} h")

    import os
    os.makedirs(args.out_dir, exist_ok=True)
    fig_components(data, t0, rep, os.path.join(args.out_dir, "ssr_components.png"))
    fig_constellation_clock(data, t0, svs,
                            os.path.join(args.out_dir, "ssr_constellation_clock.png"))
    return 0


if __name__ == "__main__":
    sys.exit(main())

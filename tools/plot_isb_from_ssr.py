#!/usr/bin/env python3
"""plot_isb_from_ssr.py — inter-system bias drift from an SSR clock stream.

The time-varying part of inter-system bias (ISB) is a *system* quantity — the
offset between GST/BDT and GPS time — carried in the correction stream, not
invented by the receiver.  Each constellation's SSR clock corrections are
referenced to that constellation's system time, so the difference of the
per-constellation *mean* clock correction tracks the inter-system time offset
(GGTO / BGTO).  This needs only an NTRIP feed — runnable on ptpmon, no GNSS
receiver or DO — and works on the SSR captures we already have.

Reads a log_ssr_corrections.py CSV, bins clock_c0 by time and constellation,
and plots (GAL-GPS) and (BDS-GPS) mean-clock offset vs time.  Mean-removed so
the *drift* (the point) is visible; the absolute level is datum/hardware
dependent and not meaningful across sources.

Usage:
    plot_isb_from_ssr.py ssr_corrections.csv --out-dir plots [--bin-s 300]
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

plt.rcParams.update({
    "font.size": 18, "axes.titlesize": 22, "axes.labelsize": 21,
    "xtick.labelsize": 18, "ytick.labelsize": 18, "legend.fontsize": 17,
})


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--bin-s", type=float, default=300.0)
    args = ap.parse_args()

    # stream the (large) CSV once; aggregate clock_c0 mean per (bin, sys)
    acc = defaultdict(lambda: [0.0, 0])      # (bin, sys) -> [sum_ns, count]
    t0 = None
    with open(args.csv, newline="") as f:
        r = csv.reader(f)
        hdr = next(r)
        iM, iS, iC, iV = (hdr.index("mono_s"), hdr.index("sys"),
                          hdr.index("component"), hdr.index("value_ns"))
        for row in r:
            if row[iC] != "clock_c0":
                continue
            try:
                m = float(row[iM]); v = float(row[iV])
            except (ValueError, IndexError):
                continue
            if t0 is None:
                t0 = m
            b = int((m - t0) // args.bin_s)
            a = acc[(b, row[iS])]
            a[0] += v; a[1] += 1

    nb = max(b for b, _ in acc) + 1
    hrs = np.arange(nb) * args.bin_s / 3600.0
    means = {}
    for s in ("G", "E", "C", "R"):
        y = np.full(nb, np.nan)
        for (b, ss), (sm, ct) in acc.items():
            if ss == s and ct:
                y[b] = sm / ct
        means[s] = y

    fig, ax = plt.subplots(figsize=(16, 9))
    ax.axhline(0, color="gray", lw=0.6)
    span_h = hrs[-1]
    # GLONASS drawn first (it's the largest/noisiest — Cs clocks, FDMA) so the
    # tighter GAL/BDS traces stay on top and legible.
    for s, name, col in (("R", "GLO−GPS", "tab:red"),
                         ("C", "BDS−GPS", "tab:orange"),
                         ("E", "GAL−GPS", "tab:green")):
        if np.isfinite(means[s]).sum() < 10 or np.isfinite(means["G"]).sum() < 10:
            continue
        d = means[s] - means["G"]
        m = np.isfinite(d)
        d = d - np.nanmean(d[m])                     # mean-removed: show the drift
        s_h = np.polyfit(hrs[m], d[m], 1)[0]
        ax.plot(hrs[m], d[m], lw=2.6, color=col,
                label=f"{name}   (drift {s_h:+.2f} ns/h, span {np.nanmax(d[m])-np.nanmin(d[m]):.1f} ns)")
    ax.set_xlim(0, span_h)
    ax.set_xlabel(f"elapsed time  [h]   (SSR capture {span_h:.1f} h)")
    ax.set_ylabel("inter-system time offset − mean  [ns]")
    ax.grid(alpha=0.3)
    ax.legend(loc="best")
    ax.set_title("Inter-system bias drift, straight from the SSR clock stream\n"
                 "per-constellation mean clock correction vs GPS — an NTRIP-only quantity "
                 "(no receiver / DO)")
    fig.tight_layout()
    os.makedirs(args.out_dir, exist_ok=True)
    out = os.path.join(args.out_dir, "isb_from_ssr")
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png   span={span_h:.1f} h, bins={nb}")


if __name__ == "__main__":
    sys.exit(main())

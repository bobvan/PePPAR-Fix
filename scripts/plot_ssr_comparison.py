#!/usr/bin/env python3
"""plot_ssr_comparison.py — overlay multiple SSR correction *sources* per SV.

Implements the figure in docs/has-ssr-correction-comparison.md: compare how
different correction sources (broadcast baseline, Galileo HAS via E6 SIS, HAS
via IDD internet, CNES SSR) each correct the broadcast clock+orbit, in the time
(ns) domain.  Companion to plot_ssr_corrections.py (single-source).

Each source is a CSV from log_ssr_corrections.py.  Per SV we plot the broadcast
clock+orbit *time error* that source implies — `−(c0 + orbit_radial)`, detrended
(per-SV mean removed) — as one line; **constellation = colour, source =
linestyle**.  Where an SV's source-lines coincide, the sources agree on the
correction; where they spread, they disagree (e.g. HAS noisier than CNES).
Broadcast itself is the y=0 baseline (no correction).

Focus is **GPS + Galileo** — HAS serves only those.  Two figures: the full view
(±5 ns) and a **Galileo ±1 ns zoom** (the Galileo story is sub-ns: broadcast is
already so good that HAS barely moves it).

Usage:
  plot_ssr_comparison.py --source CNES=cnes.csv --source HAS-E6=hase6.csv \
      [--source HAS-IDD=idd.csv] --out-dir plots
"""
import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

sys.path.insert(0, "scripts")
import plot_ssr_corrections as P  # load, _break_stale, SYS_COLOR/NAME, PLOT_ORDER, LINE_W

LINESTYLES = ["-", "--", ":", "-."]
DEFAULT_SVS = ["G24", "G04", "E26", "E36"]   # GPS + Galileo (the HAS constellations)

plt.rcParams.update({
    "font.size": 18, "axes.titlesize": 24, "axes.labelsize": 22,
    "xtick.labelsize": 20, "ytick.labelsize": 20, "legend.fontsize": 15,
})


def broadcast_error(data, t0, prn):
    """Per-SV broadcast clock+orbit time error implied by a source (ns, detrended)."""
    kc, ko = (prn, "clock_c0", ""), (prn, "orbit_radial", "")
    if kc not in data or ko not in data:
        return None
    tc, vc = data[kc]
    to, vo = data[ko]
    err = P._break_stale(tc, vc) + np.interp(tc, to, vo)
    return (tc - t0) / 3600.0, err - np.nanmean(err)


def _figure(sources, svs, span_h, ylim, out, title, galileo_only=False):
    fig, ax = plt.subplots(figsize=(16, 9))
    plot_svs = [s for s in svs if (s[0] == "E" if galileo_only else True)]
    peaks = {}
    for si, (label, data, t0) in enumerate(sources):
        ls = LINESTYLES[si % len(LINESTYLES)]
        for prn in sorted(plot_svs, key=lambda p: P.PLOT_ORDER.get(p[0], 9)):
            be = broadcast_error(data, t0, prn)
            if be is None:
                continue
            th, err = be
            m = th <= span_h + 1e-9
            ax.plot(th[m], err[m], ls=ls, lw=P.LINE_W.get(prn[0], 1.4),
                    color=P.SYS_COLOR[prn[0]], alpha=0.85)
            peaks[prn[0]] = max(peaks.get(prn[0], 0.0), float(np.nanmax(np.abs(err[m]))))
    ax.axhline(0, color="black", lw=2.2)
    ax.set_ylim(-ylim, ylim)
    ax.set_xlim(0, span_h)
    ax.set_xlabel(f"hours since capture start   (window {span_h:.1f} h)")
    ax.set_ylabel("time error − per-SV mean  [ns]")
    ax.grid(alpha=0.3)
    # two-key legend: colour = constellation, linestyle = source
    seen = [p[0] for p in plot_svs]
    col_handles = [Line2D([], [], color=P.SYS_COLOR[s], lw=3, label=P.SYS_NAME[s])
                   for s in dict.fromkeys(seen)]
    src_handles = [Line2D([], [], color="0.3", lw=2.2, ls=LINESTYLES[i % len(LINESTYLES)],
                          label=lbl) for i, (lbl, _, _) in enumerate(sources)]
    leg1 = ax.legend(handles=col_handles, loc="upper left", title="constellation")
    ax.add_artist(leg1)
    ax.legend(handles=src_handles, loc="upper right", title="source")
    ax.set_title(title)
    if any(p > ylim for p in peaks.values()):
        P._peak_table(ax, peaks, ylim, label="broadcast error peak")
    fig.tight_layout()
    fig.savefig(out + ".pdf")
    fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png   peaks={ {k: round(v,1) for k,v in peaks.items()} }")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", action="append", required=True, metavar="LABEL=CSV",
                    help="correction source, e.g. --source CNES=cnes.csv (repeatable)")
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--svs", default=",".join(DEFAULT_SVS))
    ap.add_argument("--ylim", type=float, default=5.0)
    ap.add_argument("--zoom-ylim", type=float, default=1.0, help="Galileo zoom ±ns")
    args = ap.parse_args()

    sources = []
    span_h = 0.0
    for spec in args.source:
        if "=" not in spec:
            print(f"ERROR: --source must be LABEL=CSV, got {spec!r}", file=sys.stderr)
            return 2
        label, path = spec.split("=", 1)
        data, t0 = P.load(path)
        if not data:
            print(f"ERROR: no data in {path}", file=sys.stderr)
            return 2
        sources.append((label, data, t0))
        span_h = max(span_h, (max(t.max() for t, _ in data.values()) - t0) / 3600.0)
    svs = [s.strip() for s in args.svs.split(",") if s.strip()]
    os.makedirs(args.out_dir, exist_ok=True)

    srcs = ", ".join(l for l, _, _ in sources)
    _figure(sources, svs, span_h, args.ylim,
            os.path.join(args.out_dir, "ssr_compare"),
            f"Correction sources vs broadcast — clock+orbit time error (ns)\n"
            f"colour = constellation · linestyle = source ({srcs}) · black = broadcast (0)")
    _figure(sources, svs, span_h, args.zoom_ylim,
            os.path.join(args.out_dir, "ssr_compare_galileo_zoom"),
            f"Galileo zoom — broadcast is already sub-ns, so corrections barely move it\n"
            f"linestyle = source ({srcs})", galileo_only=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())

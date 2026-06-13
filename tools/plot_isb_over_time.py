#!/usr/bin/env python3
"""plot_isb_over_time.py — inter-system bias (ISB) drift over time.

The PPP filter estimates a per-constellation receiver-clock offset relative to
GPS: ISB(GAL-GPS) and ISB(BDS-GPS), logged as isb_gal_ns / isb_bds_ns by the
engine.  ISB = the realized system-time offset (GGTO / BGTO, broadcast and
slowly drifting) + a stable receiver hardware bias.  Over hours it walks ~ns;
over >24 h the diurnal / multi-day system-time-offset trend shows — which is
the point of this slide (you can't treat ISB as a fixed constant for a
sub-ns time mission; it must be estimated or it leaks into the clock).

Plots isb_gal_ns and isb_bds_ns vs elapsed time, annotates the drift rate
(linear fit, ns/h).  A series that is all ~0 (constellation not estimated) is
labelled rather than drawn.  16:9.

Usage:
    plot_isb_over_time.py CSV [CSV ...] --out-dir plots [--label LABEL]
"""
import argparse
import csv
import os
import sys
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.size": 18, "axes.titlesize": 23, "axes.labelsize": 21,
    "xtick.labelsize": 18, "ytick.labelsize": 18, "legend.fontsize": 17,
})


def _t(s):
    return datetime.fromisoformat(s.replace("Z", "+00:00"))


def load(path):
    t0 = None
    hrs, gal, bds = [], [], []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            ts = r.get("timestamp") or r.get("utc_iso")
            if not ts:
                continue
            try:
                dt = _t(ts)
                g = float(r.get("isb_gal_ns", "nan"))
                b = float(r.get("isb_bds_ns", "nan"))
            except (ValueError, TypeError):
                continue
            if t0 is None:
                t0 = dt
            hrs.append((dt - t0).total_seconds() / 3600.0)
            gal.append(g); bds.append(b)
    return np.array(hrs), np.array(gal), np.array(bds)


def _series(h, y):
    """Masked (h, y, drift ns/h) for a series; exact-0 = filter re-init / off,
    so it's dropped (real ISB is never exactly 0).  Returns None if not estimated."""
    m = np.isfinite(y) & (y != 0.0)
    if m.sum() < 10 or np.std(y[m]) < 1e-6:
        return None
    s, _ = np.polyfit(h[m], y[m], 1)
    return h[m], y[m], s


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--label", default=None)
    args = ap.parse_args()

    fig, ax = plt.subplots(figsize=(16, 9))
    ax.axhline(0, color="gray", lw=0.6)
    span = 0.0
    notes = []
    for ci, path in enumerate(args.csv):
        h, gal, bds = load(path)
        if not len(h):
            continue
        span = max(span, h.max())
        tag = args.label or os.path.basename(path).replace(".csv", "")
        for y, name, col in ((gal, "GAL−GPS", "tab:green"), (bds, "BDS−GPS", "tab:orange")):
            r = _series(h, y)
            if r is None:
                notes.append(f"{name}: not estimated (constellation off)")
                continue
            hm, ym, s = r
            ax.plot(hm, ym, lw=2.6, color=col,
                    label=f"{name}   (drift {s:+.2f} ns/h, span {ym.max()-ym.min():.1f} ns)")
    ax.set_xlim(0, span)
    ax.set_xlabel(f"elapsed time  [h]   (capture {span:.1f} h)")
    ax.set_ylabel("inter-system bias vs GPS  [ns]")
    ax.grid(alpha=0.3)
    ax.legend(loc="best")
    ax.set_title("Inter-system bias (ISB) drift over time\n"
                 "ISB = system-time offset (GGTO/BGTO, drifts) + stable receiver bias — "
                 "not a fixed constant")
    if notes:
        ax.text(0.012, 0.04, "\n".join(notes), transform=ax.transAxes, fontsize=14,
                va="bottom", family="monospace",
                bbox=dict(boxstyle="round", fc="white", ec="0.6", alpha=0.9))
    fig.tight_layout()
    os.makedirs(args.out_dir, exist_ok=True)
    out = os.path.join(args.out_dir, "isb_over_time")
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png   span={span:.1f} h")


if __name__ == "__main__":
    sys.exit(main())

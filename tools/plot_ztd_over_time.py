#!/usr/bin/env python3
"""plot_ztd_over_time.py — zenith tropospheric delay over time, in ns.

Companion to the NRCAN/CSRS-PPP "Estimated Tropospheric Zenith Delay" plot, but
with the vertical axis in **time (ns)** instead of distance (m) so it sits in
the same ns-scale family as the other correction slides (1 m = 3.336 ns).

Source is the engine's ZTD filter state (`ztd_mm`, with `ztd_sigma_mm`) from a
lab run — ZTD is *estimated*, not delivered by SSR.  NB: the real-time engine
ZTD is weakly observable (the pos/ZTD/clk null), so it is noisier than NRCAN's
post-processed PPP; a PRIDE/CSRS `htg_` ZTD is the clean post-processed analog.

Plots the per-epoch estimate (light) + a rolling-median trend (bold) + the
formal-sigma band.  Optionally adds a modeled hydrostatic baseline (--zhd-ns)
so the y-axis reads as *total* ZTD (~8 ns) like NRCAN; default shows the state
(the variable/wet part) about zero.

Usage:
  plot_ztd_over_time.py engine.csv --out-dir plots [--zhd-ns 7.57] [--smooth-s 120]
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

MM_TO_NS = 1e9 / 299792458.0 * 1e-3   # 0.0033356 ns per mm

plt.rcParams.update({
    "font.size": 18, "axes.titlesize": 22, "axes.labelsize": 21,
    "xtick.labelsize": 18, "ytick.labelsize": 18, "legend.fontsize": 16,
})


def _rolling_median(y, win):
    if win < 3:
        return y
    out = np.full_like(y, np.nan)
    h = win // 2
    for i in range(len(y)):
        out[i] = np.nanmedian(y[max(0, i - h):i + h + 1])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--label", default="PePPAR engine (lab run)")
    ap.add_argument("--zhd-ns", type=float, default=0.0,
                    help="modeled hydrostatic baseline (ns) to add → total ZTD")
    ap.add_argument("--smooth-s", type=float, default=120.0)
    ap.add_argument("--skip-h", type=float, default=0.25, help="drop initial convergence")
    args = ap.parse_args()

    t, z, s = [], [], []
    with open(args.csv, newline="") as f:
        for r in csv.DictReader(f):
            ts = r.get("host_timestamp") or r.get("timestamp")
            try:
                dt = datetime.fromisoformat(ts.replace("Z", "+00:00"))
                zz = float(r["ztd_mm"]); ss = float(r.get("ztd_sigma_mm", "nan"))
            except (ValueError, TypeError, KeyError, AttributeError):
                continue
            t.append(dt); z.append(zz); s.append(ss)
    if len(t) < 10:
        print("too few ZTD samples", file=sys.stderr)
        return 2
    t0 = t[0]
    h = np.array([(x - t0).total_seconds() / 3600.0 for x in t])
    z = np.array(z) * MM_TO_NS + args.zhd_ns
    s = np.array(s) * MM_TO_NS
    keep = h >= args.skip_h
    h, z, s = h[keep], z[keep], s[keep]

    dt_s = np.median(np.diff(h)) * 3600.0 if len(h) > 1 else 1.0
    win = max(3, int(args.smooth_s / max(dt_s, 1e-3)))
    zsm = _rolling_median(z, win)

    fig, ax = plt.subplots(figsize=(16, 9))
    ax.fill_between(h, z - s, z + s, color="0.7", alpha=0.5, label="formal σ")
    ax.plot(h, z, lw=0.5, color="tab:blue", alpha=0.35, label="per-epoch")
    ax.plot(h, zsm, lw=2.8, color="tab:blue",
            label=f"{args.smooth_s:.0f}-s median")
    ax.set_xlim(0, h.max())
    ax.set_xlabel(f"elapsed time  [h]   (lab run, {h.max():.1f} h)")
    ylab = "total ZTD" if args.zhd_ns else "ZTD estimate − a-priori (wet/variable part)"
    ax.set_ylabel(f"{ylab}   [ns]")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")
    pp = np.nanmax(zsm) - np.nanmin(zsm)
    ax.set_title("Zenith tropospheric delay over time — in ns, not metres\n"
                 f"{args.label}: estimated (not SSR); weakly observable in real time "
                 f"(trend p-p ≈ {pp:.1f} ns)")
    fig.tight_layout()
    os.makedirs(args.out_dir, exist_ok=True)
    out = os.path.join(args.out_dir, "ztd_over_time")
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png  span={h.max():.1f}h trend p-p={pp:.2f}ns "
          f"raw σ_scatter={np.nanstd(z - zsm):.2f}ns")


if __name__ == "__main__":
    sys.exit(main())

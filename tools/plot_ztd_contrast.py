#!/usr/bin/env python3
"""plot_ztd_contrast.py — ZTD stability: real-time vs post-processed, in ns.

Two-panel contrast for the slide:
  TOP    post-processed PPP — the NRCAN/CSRS-PPP "Estimated Tropospheric Zenith
         Delay" for the London 24 h capture (2026-06-12).  Curve traced from
         the published NRCAN report figure (values in metres) and converted to
         ns; it's smooth to ~0.2 ns.
  BOTTOM real-time — the PePPAR engine's own ZTD filter state from a lab run
         (ztd_mm), the best a real-time loop does; weakly observable (the
         pos/ZTD/clk null) so it wanders ~10x more.

Both mean-removed (the absolute baseline is site/model dependent; the point is
the *variation*) and on a common ns axis so the contrast is direct: post-
processing pins ZTD to ~0.2 ns, real-time wanders ~±1.5 ns.

NB: the post-processed analog from *our own* pipeline is PRIDE pdp3 in
kinematic mode; that run is blocked on WUM product-server reachability, so the
NRCAN report (same London data) stands in as the post-processed reference here.

Usage:
  plot_ztd_contrast.py engine.csv --out-dir plots
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

M_TO_NS = 1e9 / 299792458.0          # 3.33564 ns/m
MM_TO_NS = M_TO_NS * 1e-3

plt.rcParams.update({
    "font.size": 17, "axes.titlesize": 20, "axes.labelsize": 19,
    "xtick.labelsize": 16, "ytick.labelsize": 16, "legend.fontsize": 15,
})

# NRCAN/CSRS-PPP ZTD for London 2026-06-12, traced from the report figure.
# (hours from 12:00 UTC, total ZTD in metres)
NRCAN_H = [0,0.5,1,2,3,3.5,4.5,5,6,7,7.5,8,8.5,9,10,10.5,11,11.5,12,12.5,
           13,13.5,14,15,15.5,16,16.5,17,18,19,20,21]
NRCAN_M = [2.490,2.500,2.495,2.480,2.460,2.455,2.460,2.450,2.420,2.415,2.418,
           2.420,2.415,2.410,2.410,2.415,2.420,2.415,2.420,2.423,2.420,2.415,
           2.410,2.410,2.413,2.412,2.415,2.410,2.405,2.400,2.398,2.400]


def _rolling_median(y, win):
    out = np.full_like(y, np.nan)
    h = win // 2
    for i in range(len(y)):
        out[i] = np.nanmedian(y[max(0, i - h):i + h + 1])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="engine CSV with host_timestamp + ztd_mm")
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--label", default="PePPAR engine (MadHat, i226)")
    args = ap.parse_args()

    # --- our real-time engine ZTD ---
    t, z = [], []
    with open(args.csv, newline="") as f:
        for r in csv.DictReader(f):
            ts = r.get("host_timestamp") or r.get("timestamp")
            try:
                dt = datetime.fromisoformat(ts.replace("Z", "+00:00"))
                zz = float(r["ztd_mm"])
            except (ValueError, TypeError, KeyError, AttributeError):
                continue
            t.append(dt); z.append(zz)
    h = np.array([(x - t[0]).total_seconds() / 3600.0 for x in t])
    z = np.array(z) * MM_TO_NS
    keep = h >= 0.25                      # drop initial convergence
    h, z = h[keep], z[keep]
    dt_s = np.median(np.diff(h)) * 3600.0
    zsm = _rolling_median(z, max(3, int(300 / max(dt_s, 1e-3))))   # 5-min median
    zsm = zsm - np.nanmean(zsm)
    rt_pp = np.nanmax(zsm) - np.nanmin(zsm)

    # --- NRCAN post-processed ---
    nh = np.array(NRCAN_H)
    nz = np.array(NRCAN_M) * M_TO_NS
    nz = nz - nz.mean()
    pp_pp = nz.max() - nz.min()

    ylim = 1.15 * max(np.nanmax(np.abs(zsm)), np.nanmax(np.abs(nz)))
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 9), sharey=True)

    ax1.plot(nh, nz, "-o", ms=4, lw=2.6, color="tab:green")
    ax1.axhline(0, color="gray", lw=0.5); ax1.grid(alpha=0.3)
    ax1.set_ylim(-ylim, ylim)
    ax1.set_ylabel("ZTD − mean  [ns]")
    ax1.set_title(f"POST-PROCESSED PPP  (NRCAN/CSRS, London 2026-06-12, traced) "
                  f"— smooth, p-p ≈ {pp_pp:.2f} ns")

    ax2.plot(h, zsm, lw=2.4, color="tab:red")
    ax2.axhline(0, color="gray", lw=0.5); ax2.grid(alpha=0.3)
    ax2.set_ylim(-ylim, ylim)
    ax2.set_xlabel("elapsed time  [h]")
    ax2.set_ylabel("ZTD − mean  [ns]")
    ax2.set_title(f"REAL-TIME engine  ({args.label}) — weakly observable, "
                  f"p-p ≈ {rt_pp:.1f} ns  (~{rt_pp/max(pp_pp,1e-3):.0f}×)")

    fig.suptitle("Zenith tropospheric delay — real-time vs post-processed (in ns)",
                 fontsize=23, fontweight="bold", y=0.985)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    os.makedirs(args.out_dir, exist_ok=True)
    out = os.path.join(args.out_dir, "ztd_contrast")
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png  post-proc p-p={pp_pp:.2f}ns  real-time p-p={rt_pp:.2f}ns "
          f"({rt_pp/max(pp_pp,1e-3):.0f}x)")


if __name__ == "__main__":
    sys.exit(main())

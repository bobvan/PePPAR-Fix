#!/usr/bin/env python3
"""plot_solid_tide.py — solid Earth tide as a *timing* correction (ns).

Part of the "corrections the SSR stream does NOT carry" story (companion to
plot_ssr_corrections.py).  The SSR plots show satellite-side corrections;
this shows a *site-displacement* correction that is purely deterministic
(no stream, no receiver) yet is essential below ~1 ns: the solid Earth tide
bodily moves the antenna up to ~30 cm, and that displacement maps straight
into the range / clock solution.

We compute the IERS degree-2 displacement at the surveyed ARP over a 48 h
window, project onto local up / east / north, and convert metres -> ns
(1 m = 3.3356 ns).  Detrended (mean removed) and drawn on the SAME ±5 ns
scale as the SSR figures so the magnitude compares directly: the dominant
~12.42 h semidiurnal + diurnal tide is a clean ~sub-ns-to-~1 ns swing — far
below the ±15 ns you'd shrug at from a bare F9T PPS, but a sky-high mountain
once you want sub-ns.

Usage:
    python3 tools/plot_solid_tide.py --out-dir plots [--hours 48] [--ecef X Y Z]
"""
import argparse
import glob
import json
import math
import os
import sys
from datetime import datetime, timedelta, timezone

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))
from solid_tide import solid_tide_displacement

C = 299792458.0
M2NS = 1e9 / C  # 3.3356 ns/m

plt.rcParams.update({
    "font.size": 18, "axes.titlesize": 24, "axes.labelsize": 22,
    "xtick.labelsize": 20, "ytick.labelsize": 20, "legend.fontsize": 18,
})

# ARP coordinates are NOT hardcoded here — they live in the gitignored
# timelab/antennas.json (per CLAUDE.md "operational positions are not
# hardcoded").  We load the requested label's ECEF at runtime.
_ANTENNAS_JSON_CANDIDATES = [
    "~/git/timelab/antennas.json", "~/timelab/antennas.json",
    "~/peppar-fix/timelab/antennas.json", "timelab/antennas.json",
    "../timelab/antennas.json",
]


def load_arp(label, path=None):
    """Return (ecef[3] np.array, lat_rad, lon_rad) for an antennas.json label."""
    cands = [path] if path else _ANTENNAS_JSON_CANDIDATES
    for c in cands:
        f = os.path.expanduser(c)
        if os.path.isfile(f):
            d = json.load(open(f))
            if label in d:
                a = d[label]
                e = a["ecef_m"]
                return (np.array([e["x"], e["y"], e["z"]]),
                        math.radians(a["lat"]), math.radians(a["lon"]))
    raise SystemExit(f"ARP label {label!r} not found in antennas.json "
                     f"(looked in {cands}); pass --ecef X Y Z to override.")


def enu_basis(lat, lon):
    up = np.array([math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat)])
    east = np.array([-math.sin(lon), math.cos(lon), 0.0])
    north = np.array([-math.sin(lat) * math.cos(lon), -math.sin(lat) * math.sin(lon), math.cos(lat)])
    return up, east, north


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--hours", type=float, default=48.0)
    ap.add_argument("--step-min", type=float, default=10.0)
    ap.add_argument("--ylim", type=float, default=5.0)
    ap.add_argument("--arp-label", default="ufo1",
                    help="antenna label to load from antennas.json (default ufo1)")
    ap.add_argument("--antennas-json", default=None,
                    help="path to antennas.json (default: auto-discover)")
    ap.add_argument("--ecef", type=float, nargs=3, default=None,
                    help="station ECEF X Y Z (overrides --arp-label)")
    ap.add_argument("--start", default="2026-06-13T00:00:00",
                    help="UTC start (deterministic, so the figure is reproducible)")
    args = ap.parse_args()

    if args.ecef:
        ecef = np.array(args.ecef)
        r = np.linalg.norm(ecef)
        lat = math.asin(ecef[2] / r); lon = math.atan2(ecef[1], ecef[0])
    else:
        ecef, lat, lon = load_arp(args.arp_label, args.antennas_json)
    up, east, north = enu_basis(lat, lon)

    t0 = datetime.fromisoformat(args.start).replace(tzinfo=timezone.utc)
    n = int(args.hours * 60 / args.step_min) + 1
    th = np.empty(n); u = np.empty(n); e = np.empty(n); nn = np.empty(n)
    for i in range(n):
        t = t0 + timedelta(minutes=i * args.step_min)
        d = solid_tide_displacement(t, ecef)      # ECEF displacement, metres
        th[i] = i * args.step_min / 60.0
        u[i] = d @ up * M2NS
        e[i] = d @ east * M2NS
        nn[i] = d @ north * M2NS
    u -= u.mean(); e -= e.mean(); nn -= nn.mean()

    fig, ax = plt.subplots(figsize=(16, 9))
    ax.axhline(0, color="gray", lw=0.6)
    ax.plot(th, u, lw=2.6, color="tab:purple", label="up (vertical)")
    ax.plot(th, nn, lw=1.6, color="tab:blue", alpha=0.85, label="north")
    ax.plot(th, e, lw=1.6, color="tab:green", alpha=0.85, label="east")
    ax.set_ylim(-args.ylim, args.ylim)
    ax.set_xlim(0, args.hours)
    ax.set_xlabel(f"hours since {args.start}Z   (window {args.hours:.0f} h)")
    ax.set_ylabel("range-equivalent − mean  [ns]")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", ncol=3)
    pk = max(np.abs(u).max(), np.abs(e).max(), np.abs(nn).max())
    ax.set_title("Solid Earth tide as a timing correction — detrended, ±%.0f ns\n"
                 "deterministic site displacement (no stream); up peak ≈ %.2f ns "
                 "≈ %.0f cm, period ~12.42 h" % (args.ylim, np.abs(u).max(),
                                                 np.abs(u).max() / M2NS * 100))
    src = "--ecef" if args.ecef else f"{args.arp_label} ARP"
    ax.text(0.012, 0.04, f"{src} · IERS degree-2 · 1 m = 3.336 ns",
            transform=ax.transAxes, fontsize=15, family="monospace",
            bbox=dict(boxstyle="round", fc="white", ec="0.6", alpha=0.9))
    fig.tight_layout()
    os.makedirs(args.out_dir, exist_ok=True)
    out = os.path.join(args.out_dir, "nonssr_solid_tide")
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png   up peak={np.abs(u).max():.3f} ns "
          f"({np.abs(u).max()/M2NS*100:.1f} cm), e/n peak={max(np.abs(e).max(),np.abs(nn).max()):.3f} ns")


if __name__ == "__main__":
    sys.exit(main())

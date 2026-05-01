#!/usr/bin/env python3
"""Position-deviation scatter: where does the engine's position estimate
sit over the course of a run?

Two scatter plots from one engine log:
  - Horizontal: E/W (x) vs N/S (y), square aspect for shape integrity
  - Vertical:   E/W (x) vs U/D (y)

Position is plotted as deviation from the run's per-axis median (robust
reference; doesn't require an external ARP).  Color encodes elapsed
time since the first epoch (perceptually-uniform colormap).  Alpha is
fixed transparency so dense regions show through.

Default axis limits are auto-fit so 90% of points are visible per
axis (10% clip per axis ⇒ ~81-90% inside each box depending on how
correlated the deviations are).  --cover overrides the fraction;
explicit --xlim / --ylim / --zlim override individual axes.

Usage:
    pos_scatter.py <engine.log> [-o out.png]            # auto-zoom 90%
    pos_scatter.py <engine.log> --cover 0.95            # tighter — 5% clip
    pos_scatter.py <engine.log> --xlim 0.05 --ylim 0.05 # decimeter zoom override

Engine log shape parsed:
    [AntPosEst N] positionσ=X.XXXm pos=(LAT, LON, ALT) ...
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


# Per-epoch [AntPosEst N] line.  Engine source:
# scripts/peppar_fix_engine.py AntPosEstThread.
_ANTPOSEST_RE = re.compile(
    r"(\d{4}-\d{2}-\d{2})\s+(\d{2}:\d{2}:\d{2})[,.]?\d*\s+\S+\s+"
    r"\[AntPosEst\s+\d+\]\s+positionσ=([\d.]+)m\s+"
    r"pos=\(([-+]?\d+\.\d+),\s*([-+]?\d+\.\d+),\s*([-+]?\d+\.\d+)\)"
)


def parse_log(path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return (t_epoch_s, sigma_m, lat_deg, lon_deg, alt_m) arrays."""
    ts, sigmas, lats, lons, alts = [], [], [], [], []
    with open(path) as f:
        for line in f:
            m = _ANTPOSEST_RE.search(line)
            if not m:
                continue
            d, t, sig, lat, lon, alt = m.groups()
            ep = datetime.fromisoformat(f"{d}T{t}").timestamp()
            ts.append(ep)
            sigmas.append(float(sig))
            lats.append(float(lat))
            lons.append(float(lon))
            alts.append(float(alt))
    if not ts:
        sys.exit("No [AntPosEst] lines parsed from log.")
    return (np.asarray(ts), np.asarray(sigmas),
            np.asarray(lats), np.asarray(lons), np.asarray(alts))


def lla_to_local_meters(lat_deg: np.ndarray, lon_deg: np.ndarray,
                        alt_m: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, tuple[float, float, float]]:
    """Convert lat/lon/alt arrays to (E, N, U) meter offsets from the
    per-axis median.  Returns (e_m, n_m, u_m, (lat0, lon0, alt0))."""
    lat0 = float(np.median(lat_deg))
    lon0 = float(np.median(lon_deg))
    alt0 = float(np.median(alt_m))
    # Local tangent-plane approximation valid for sub-km extents.
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = 111_320.0 * math.cos(math.radians(lat0))
    n_m = (lat_deg - lat0) * m_per_deg_lat
    e_m = (lon_deg - lon0) * m_per_deg_lon
    u_m = alt_m - alt0
    return e_m, n_m, u_m, (lat0, lon0, alt0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="engine log file with [AntPosEst N] lines")
    ap.add_argument("-o", "--output", default=None,
                    help="output image (default: <log_basename>.scatter.png)")
    ap.add_argument("--cover", type=float, default=0.9,
                    help="fraction of points to keep visible per axis when "
                         "axis bounds are auto (default: 0.9 = 10%% clip per axis)")
    ap.add_argument("--xlim", type=float, default=None,
                    help="explicit E/W half-range in meters (default: auto)")
    ap.add_argument("--ylim", type=float, default=None,
                    help="explicit N/S half-range in meters (default: auto)")
    ap.add_argument("--zlim", type=float, default=None,
                    help="explicit U/D half-range in meters (default: auto)")
    ap.add_argument("--alpha", type=float, default=0.3,
                    help="point transparency (default: 0.3)")
    ap.add_argument("--cmap", default="viridis",
                    help="matplotlib colormap (default: viridis)")
    ap.add_argument("--marker-size", type=float, default=8.0,
                    help="point size (default: 8.0)")
    args = ap.parse_args()

    ts, sigmas, lats, lons, alts = parse_log(args.log)
    e_m, n_m, u_m, (lat0, lon0, alt0) = lla_to_local_meters(lats, lons, alts)

    elapsed_s = ts - ts[0]
    elapsed_h = elapsed_s / 3600.0
    n_total = len(ts)

    # Auto-fit axis bounds so --cover fraction of points fit per axis.
    # Per-axis percentile of |deviation| is the smallest symmetric
    # bound that keeps that fraction of points visible on that axis;
    # combined per-box coverage is typically 81-90% depending on how
    # correlated the deviations are.
    auto_pct = max(0.0, min(100.0, 100.0 * args.cover))
    if args.xlim is None:
        args.xlim = float(np.percentile(np.abs(e_m), auto_pct)) or 0.01
    if args.ylim is None:
        args.ylim = float(np.percentile(np.abs(n_m), auto_pct)) or 0.01
    if args.zlim is None:
        args.zlim = float(np.percentile(np.abs(u_m), auto_pct)) or 0.01

    n_clipped_h = int(np.sum((np.abs(e_m) > args.xlim) | (np.abs(n_m) > args.ylim)))
    n_clipped_v = int(np.sum((np.abs(e_m) > args.xlim) | (np.abs(u_m) > args.zlim)))

    fig, (ax_h, ax_v) = plt.subplots(1, 2, figsize=(14, 6))

    # Horizontal scatter — square aspect for true shape.
    sc_h = ax_h.scatter(e_m, n_m, c=elapsed_h, cmap=args.cmap,
                        alpha=args.alpha, s=args.marker_size, edgecolors='none')
    ax_h.axhline(0, color='gray', lw=0.5, alpha=0.5)
    ax_h.axvline(0, color='gray', lw=0.5, alpha=0.5)
    ax_h.set_xlim(-args.xlim, args.xlim)
    ax_h.set_ylim(-args.ylim, args.ylim)
    ax_h.set_aspect('equal', adjustable='box')
    ax_h.set_xlabel('East / West (m)')
    ax_h.set_ylabel('North / South (m)')
    ax_h.set_title(f'Horizontal — {n_total} pts, {n_clipped_h} clipped')
    ax_h.grid(True, alpha=0.3)

    # Vertical scatter — E/W vs U/D.
    sc_v = ax_v.scatter(e_m, u_m, c=elapsed_h, cmap=args.cmap,
                        alpha=args.alpha, s=args.marker_size, edgecolors='none')
    ax_v.axhline(0, color='gray', lw=0.5, alpha=0.5)
    ax_v.axvline(0, color='gray', lw=0.5, alpha=0.5)
    ax_v.set_xlim(-args.xlim, args.xlim)
    ax_v.set_ylim(-args.zlim, args.zlim)
    ax_v.set_xlabel('East / West (m)')
    ax_v.set_ylabel('Up / Down (m)')
    ax_v.set_title(f'Vertical — {n_total} pts, {n_clipped_v} clipped')
    ax_v.grid(True, alpha=0.3)

    cbar = fig.colorbar(sc_v, ax=[ax_h, ax_v], label='Elapsed runtime (h)',
                        shrink=0.8, pad=0.02)

    log_name = Path(args.log).name
    fig.suptitle(
        f'{log_name}  —  reference (median): '
        f'lat={lat0:.7f}, lon={lon0:.7f}, alt={alt0:.2f} m  '
        f'(span={elapsed_h[-1]:.1f} h, n={n_total})',
        fontsize=10,
    )

    out = args.output or str(Path(args.log).with_suffix('.scatter.png'))
    fig.savefig(out, dpi=120, bbox_inches='tight')
    print(f"Wrote {out}")
    print(f"  Horizontal: {n_clipped_h}/{n_total} ({100.0*n_clipped_h/n_total:.1f}%) "
          f"clipped at ±{args.xlim:.3f}m E/W × ±{args.ylim:.3f}m N/S")
    print(f"  Vertical:   {n_clipped_v}/{n_total} ({100.0*n_clipped_v/n_total:.1f}%) "
          f"clipped at ±{args.xlim:.3f}m E/W × ±{args.zlim:.3f}m U/D")
    return 0


if __name__ == "__main__":
    sys.exit(main())

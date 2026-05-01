#!/usr/bin/env python3
"""Cross-host position-deviation comparison from engine [AntPosEst] logs.

Three vertically-stacked plots (lat, lon, alt) sharing the time axis.
One line per host; each host plotted as deviation from its own median
(default) or from a cross-host shared median (--reference shared).

Use case: two hosts on a shared antenna (TimeHat + MadHat on UFO1).
Both should see the same sky.  Patterns that move both lines together
are environmental (SV geometry, atmospheric, multipath shared-RF);
patterns where the lines diverge are host-specific (hardware, cable
length, software).  Studying time-of-day bias is the canonical use.

Default reference is per-host median (each line centered at 0).
Switch via --reference shared to use a single cross-host median —
preserves inter-host static offset visibility, useful for finding
constant per-host biases.

Usage:
    pos_compare.py LOG1 LOG2 [LOG3 ...] -o out.png
    pos_compare.py LOG1 LOG2 --reference shared
    pos_compare.py LOG1 LOG2 --label TimeHat MadHat

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

import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import numpy as np


_ANTPOSEST_RE = re.compile(
    r"(\d{4}-\d{2}-\d{2})\s+(\d{2}:\d{2}:\d{2})[,.]?\d*\s+\S+\s+"
    r"\[AntPosEst\s+\d+\]\s+positionσ=([\d.]+)m\s+"
    r"pos=\(([-+]?\d+\.\d+),\s*([-+]?\d+\.\d+),\s*([-+]?\d+\.\d+)\)"
)


def parse_log(path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return (datetime_obj_array, lat, lon, alt) — datetime as numpy
    object array (matplotlib accepts these directly on date axes)."""
    dts, lats, lons, alts = [], [], [], []
    with open(path) as f:
        for line in f:
            m = _ANTPOSEST_RE.search(line)
            if not m:
                continue
            d, t, _sig, lat, lon, alt = m.groups()
            dts.append(datetime.fromisoformat(f"{d}T{t}"))
            lats.append(float(lat))
            lons.append(float(lon))
            alts.append(float(alt))
    if not dts:
        sys.exit(f"No [AntPosEst] lines parsed from {path}")
    return (np.asarray(dts, dtype=object),
            np.asarray(lats), np.asarray(lons), np.asarray(alts))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="2+ engine logs to compare")
    ap.add_argument("-o", "--output", default="/tmp/pos-compare.png",
                    help="output PNG (default: /tmp/pos-compare.png)")
    ap.add_argument("--label", nargs="+", default=None,
                    help="host labels (default: derived from filename)")
    ap.add_argument("--reference", choices=["per-host", "shared"],
                    default="per-host",
                    help="median reference: per-host (each line centered at 0; "
                         "best for time-of-day-bias studies) or shared "
                         "(cross-host median; preserves inter-host static offsets)")
    ap.add_argument("--alpha", type=float, default=0.5,
                    help="line transparency (default: 0.5)")
    ap.add_argument("--linewidth", type=float, default=0.8,
                    help="line width (default: 0.8)")
    ap.add_argument("--ylim-lat", type=float, default=None,
                    help="explicit lat half-range in meters (default: auto-fit)")
    ap.add_argument("--ylim-lon", type=float, default=None,
                    help="explicit lon half-range in meters (default: auto-fit)")
    ap.add_argument("--ylim-alt", type=float, default=None,
                    help="explicit alt half-range in meters (default: auto-fit)")
    ap.add_argument("--cover", type=float, default=0.95,
                    help="auto-fit fraction per axis (default: 0.95 = 5%% clip)")
    args = ap.parse_args()

    if len(args.logs) < 2:
        sys.exit("Need at least 2 logs to compare.")
    if args.label and len(args.label) != len(args.logs):
        sys.exit(f"--label expects {len(args.logs)} entries, got {len(args.label)}")

    labels = args.label or [Path(p).name.split('-')[0] for p in args.logs]

    # Parse all hosts.
    hosts = []
    for path, label in zip(args.logs, labels):
        dts, lats, lons, alts = parse_log(path)
        hosts.append({'label': label, 'dts': dts, 'lats': lats,
                      'lons': lons, 'alts': alts})
        print(f"  {label}: {len(dts)} points, "
              f"{dts[0].isoformat()} → {dts[-1].isoformat()}")

    # Reference point for the meters-per-degree conversion.  Use the
    # cross-host median lat to avoid one host's outliers dominating.
    all_lats = np.concatenate([h['lats'] for h in hosts])
    lat_ref = float(np.median(all_lats))
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = 111_320.0 * math.cos(math.radians(lat_ref))

    # Compute deviations.  Per-host vs shared reference.
    if args.reference == "shared":
        all_lons = np.concatenate([h['lons'] for h in hosts])
        all_alts = np.concatenate([h['alts'] for h in hosts])
        lat0 = float(np.median(all_lats))
        lon0 = float(np.median(all_lons))
        alt0 = float(np.median(all_alts))
        for h in hosts:
            h['n_m'] = (h['lats'] - lat0) * m_per_deg_lat
            h['e_m'] = (h['lons'] - lon0) * m_per_deg_lon
            h['u_m'] = h['alts'] - alt0
        ref_note = f"shared median: lat={lat0:.7f}, lon={lon0:.7f}, alt={alt0:.2f} m"
    else:
        # per-host: each host centered at its own median
        for h in hosts:
            lat0 = float(np.median(h['lats']))
            lon0 = float(np.median(h['lons']))
            alt0 = float(np.median(h['alts']))
            h['n_m'] = (h['lats'] - lat0) * m_per_deg_lat
            h['e_m'] = (h['lons'] - lon0) * m_per_deg_lon
            h['u_m'] = h['alts'] - alt0
            h['ref'] = (lat0, lon0, alt0)
        ref_note = "per-host median (each centered at 0)"

    # Auto-fit y-axis bounds across all hosts at --cover fraction.
    auto_pct = max(0.0, min(100.0, 100.0 * args.cover))
    all_n = np.concatenate([h['n_m'] for h in hosts])
    all_e = np.concatenate([h['e_m'] for h in hosts])
    all_u = np.concatenate([h['u_m'] for h in hosts])
    ylim_n = args.ylim_lat or float(np.percentile(np.abs(all_n), auto_pct)) or 0.01
    ylim_e = args.ylim_lon or float(np.percentile(np.abs(all_e), auto_pct)) or 0.01
    ylim_u = args.ylim_alt or float(np.percentile(np.abs(all_u), auto_pct)) or 0.01

    # Plot.
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    colors = plt.cm.tab10(np.linspace(0, 1, max(len(hosts), 3)))

    for h, color in zip(hosts, colors):
        axes[0].plot(h['dts'], h['n_m'], color=color, alpha=args.alpha,
                     linewidth=args.linewidth, label=h['label'])
        axes[1].plot(h['dts'], h['e_m'], color=color, alpha=args.alpha,
                     linewidth=args.linewidth, label=h['label'])
        axes[2].plot(h['dts'], h['u_m'], color=color, alpha=args.alpha,
                     linewidth=args.linewidth, label=h['label'])

    for ax, title, ylim in [
        (axes[0], 'Lat (N/S)  [m]', ylim_n),
        (axes[1], 'Lon (E/W)  [m]', ylim_e),
        (axes[2], 'Alt (U/D)  [m]', ylim_u),
    ]:
        ax.axhline(0, color='gray', lw=0.5, alpha=0.5)
        ax.set_ylim(-ylim, ylim)
        ax.set_ylabel(title)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')

    axes[2].xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
    axes[2].xaxis.set_major_locator(mdates.HourLocator(interval=1))
    axes[2].set_xlabel('Time of day (UTC)')
    fig.autofmt_xdate()

    fig.suptitle(
        f'Cross-host position deviation  —  {ref_note}\n'
        f'{" / ".join(h["label"] for h in hosts)}',
        fontsize=11,
    )

    fig.savefig(args.output, dpi=120, bbox_inches='tight')
    print(f"Wrote {args.output}")
    print(f"  ylim auto-fit at {auto_pct:.0f}th-percentile-of-|deviation| per axis:")
    print(f"    lat: ±{ylim_n:.3f} m")
    print(f"    lon: ±{ylim_e:.3f} m")
    print(f"    alt: ±{ylim_u:.3f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Post-hoc: bin cycle-slip events by satellite azimuth.

Reads a peppar-fix engine log with ``slip: sv=... reasons=... conf=...``
lines and an SP3 orbit file covering the log's time range, computes
the azimuth of each slipping SV at its slip epoch relative to a
known receiver ARP, and reports counts per azimuth octant within
user-specified UTC windows.

Tests the "sunrise-eastern-slip-preference" hypothesis
(``project_sunrise_slip_storm_lambda_fallback.md``).  Azimuth
convention: 0° = north, 90° = east, 180° = south, 270° = west.

Usage:

    ./diag_slip_azimuth.py \\
        --log data/day0419h-timehat.log \\
        --sp3 data/GFZ0MGXRAP_20261100000_01D_05M_ORB.SP3.gz \\
        --receiver-pos LAT,LON,ALT \\
        --tz-offset-hours 5 \\
        --window sunrise=12:00,14:00 \\
        --window quiet=05:00,07:00

(Supply LAT,LON,ALT from timelab/antennas.json — coords aren't in
the repo.)

Log timestamps are assumed local; ``--tz-offset-hours`` converts to
UTC.  SP3 is UTC native.  Window labels are free-form; any number
of ``--window`` pairs are allowed.

Report is a compact table per window + a summary cross-window.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
from collections import defaultdict
from datetime import datetime, timedelta, timezone

import numpy as np

_SCRIPTS_DIR = os.path.abspath(os.path.dirname(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from solve_pseudorange import SP3, lla_to_ecef, ecef_to_enu  # noqa: E402


SLIP_RE = re.compile(
    r"^(?P<date>\d{4}-\d{2}-\d{2})\s+(?P<time>\d{2}:\d{2}:\d{2}),\d+\s+"
    r"INFO\s+slip:\s+sv=(?P<sv>\S+)\s+reasons=(?P<reasons>\S+)\s+"
    r"conf=(?P<conf>\S+)"
)

OCTANTS = [
    ("N-NE",  0.0,  45.0),
    ("NE-E",  45.0, 90.0),
    ("E-SE",  90.0, 135.0),
    ("SE-S",  135.0, 180.0),
    ("S-SW",  180.0, 225.0),
    ("SW-W",  225.0, 270.0),
    ("W-NW",  270.0, 315.0),
    ("NW-N",  315.0, 360.0),
]


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True, help="engine log file")
    ap.add_argument("--sp3", required=True,
                    help="SP3 file (plain or .gz) covering log time range")
    ap.add_argument("--receiver-pos", required=True,
                    help="receiver LLA: lat,lon,alt (deg,deg,m)")
    ap.add_argument("--tz-offset-hours", type=float, default=5.0,
                    help="hours to SUBTRACT from log timestamps to get UTC "
                         "(CDT=5, CST=6, UTC=0; default 5)")
    ap.add_argument("--window", action="append", required=True,
                    help="label=HH:MM,HH:MM UTC (can repeat)")
    return ap.parse_args()


def parse_windows(specs):
    """[(label, start_utc_seconds_of_day, end_utc_seconds_of_day), ...]"""
    out = []
    for spec in specs:
        label, rng = spec.split("=", 1)
        start_s, end_s = rng.split(",")
        sh, sm = [int(x) for x in start_s.split(":")]
        eh, em = [int(x) for x in end_s.split(":")]
        out.append((label, sh * 3600 + sm * 60, eh * 3600 + em * 60))
    return out


def parse_slips(log_path, tz_offset_hours):
    """Generator of (dt_utc, sv) from slip lines.

    --tz-offset-hours is the value (in hours) that the local timezone
    trails UTC by — CDT = 5 (i.e., UTC = local + 5h).  CST = 6, UTC = 0.
    """
    offset = timedelta(hours=tz_offset_hours)
    with open(log_path) as f:
        for line in f:
            m = SLIP_RE.match(line)
            if m is None:
                continue
            local = datetime.strptime(
                f"{m['date']} {m['time']}", "%Y-%m-%d %H:%M:%S")
            utc = (local + offset).replace(tzinfo=timezone.utc)
            yield utc, m["sv"]


def az_el(receiver_ecef, sat_ecef):
    dxyz = np.array(sat_ecef) - np.array(receiver_ecef)
    enu = ecef_to_enu(dxyz, np.array(receiver_ecef))
    e, n, u = float(enu[0]), float(enu[1]), float(enu[2])
    r = math.sqrt(e * e + n * n)
    el = math.degrees(math.atan2(u, r))
    az = math.degrees(math.atan2(e, n))
    if az < 0:
        az += 360.0
    return az, el


def octant(az_deg):
    for label, lo, hi in OCTANTS:
        if lo <= az_deg < hi:
            return label
    return "NW-N"  # handle az == 360


def in_window(dt_utc, start_s, end_s):
    sec = dt_utc.hour * 3600 + dt_utc.minute * 60 + dt_utc.second
    return start_s <= sec < end_s


def main():
    args = parse_args()
    lat, lon, alt = [float(x) for x in args.receiver_pos.split(",")]
    recv_ecef = np.array(lla_to_ecef(lat, lon, alt))
    windows = parse_windows(args.window)
    print(f"Loading SP3: {args.sp3}")
    sp3 = SP3(args.sp3)
    print(f"  SP3 coverage: {sp3.epochs[0]} → {sp3.epochs[-1]}")
    print(f"  Receiver ECEF: {recv_ecef}")
    print(f"  Receiver LLA:  ({lat}, {lon}, {alt})")
    print()

    # Accumulate per-window: {octant: {"count": N, "by_sv": {sv: count},
    #                                  "elevs": [e...], "skipped_no_sp3": N}}
    totals = {w[0]: {
        "bins": defaultdict(lambda: {"count": 0, "by_sv": defaultdict(int),
                                       "elevs": []}),
        "total": 0, "skipped": 0,
    } for w in windows}

    for dt_utc, sv in parse_slips(args.log, args.tz_offset_hours):
        for label, start_s, end_s in windows:
            if not in_window(dt_utc, start_s, end_s):
                continue
            sat_pos, _sat_clk = sp3.sat_position(sv, dt_utc)
            if sat_pos is None:
                totals[label]["skipped"] += 1
                continue
            az, el = az_el(recv_ecef, sat_pos)
            oct_lbl = octant(az)
            bin_ = totals[label]["bins"][oct_lbl]
            bin_["count"] += 1
            bin_["by_sv"][sv] += 1
            bin_["elevs"].append(el)
            totals[label]["total"] += 1

    for label, _start, _end in windows:
        t = totals[label]
        print(f"=== window: {label} ===")
        print(f"  total slips: {t['total']}  (skipped, no SP3: {t['skipped']})")
        header = f"  {'oct':6s} {'count':>6s} {'%':>6s} {'top_svs':<25s} {'elev_med':>9s}"
        print(header)
        print("  " + "-" * (len(header) - 2))
        for oct_lbl, _lo, _hi in OCTANTS:
            bin_ = t["bins"].get(oct_lbl, {"count": 0})
            cnt = bin_["count"]
            pct = 100.0 * cnt / t["total"] if t["total"] else 0.0
            if cnt == 0:
                print(f"  {oct_lbl:6s} {0:>6d} {0:>5.1f}% {'—':<25s} {'—':>9s}")
                continue
            by_sv = bin_["by_sv"]
            top = sorted(by_sv.items(), key=lambda kv: -kv[1])[:3]
            top_str = " ".join(f"{s}:{n}" for s, n in top)
            elev_med = float(np.median(bin_["elevs"]))
            print(f"  {oct_lbl:6s} {cnt:>6d} {pct:>5.1f}% {top_str:<25s} "
                  f"{elev_med:>8.1f}°")
        print()

    # Cross-window summary: eastern (45–135°) vs. western (225–315°).
    print("=== east-vs-west summary ===")
    print(f"  {'window':10s} {'east':>8s} {'west':>8s} {'E/W':>8s}")
    for label, _s, _e in windows:
        bins = totals[label]["bins"]
        east = bins.get("NE-E", {}).get("count", 0) + bins.get("E-SE", {}).get("count", 0)
        west = bins.get("SW-W", {}).get("count", 0) + bins.get("W-NW", {}).get("count", 0)
        ratio = float(east) / west if west else float("inf") if east else float("nan")
        print(f"  {label:10s} {east:>8d} {west:>8d} {ratio:>8.2f}")


if __name__ == "__main__":
    main()

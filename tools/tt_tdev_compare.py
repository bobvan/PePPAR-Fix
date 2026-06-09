#!/usr/bin/env python3
"""TDEV comparison of receiver-clock dt_rx between correction arms.

For the HAS-vs-broadcast time-transfer experiment
(docs/has-time-transfer-experiment.md).  Parses the engine's
`[FIXEDPOS_ZTD] ... dt_rx=<x>ns` log line (emitted ~every 30 epochs on a
DO-less host, where --servo-log/--dt-rx-log don't write) from one or more
engine logs, linearly detrends each dt_rx series, and computes TDEV with
allantools — then prints a side-by-side table across τ.

    tt_tdev_compare.py broadcast=tt_bcast.log has=tt_has.log [bkg=tt_bkg.log]

Caveat: arms captured in different wall-clock windows (one X20 USB port)
share neither sky nor the same rx-TCXO realization, so treat a first-look
comparison as indicative, not definitive — the long-τ satellite-clock
contribution (what HAS corrects) is the signal of interest.
"""
import re
import sys
from datetime import datetime

import numpy as np

_TS = re.compile(r"^(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d),(\d+)")
_DTRX = re.compile(r"dt_rx=([-\d.]+)ns")


def parse_dtrx(path):
    """Return (times_s, dt_rx_ns) arrays from [FIXEDPOS_ZTD] log lines."""
    ts, dt = [], []
    for line in open(path):
        if "FIXEDPOS_ZTD" not in line:
            continue
        m = _DTRX.search(line)
        t = _TS.match(line)
        if not m or not t:
            continue
        wall = datetime.strptime(t.group(1), "%Y-%m-%d %H:%M:%S")
        ts.append(wall.timestamp() + int(t.group(2)) / 1000.0)
        dt.append(float(m.group(1)))
    return np.array(ts), np.array(dt)


def tdev_curve(times_s, dt_rx_ns, taus):
    """Linearly-detrended TDEV (seconds) of dt_rx at the given taus."""
    import allantools
    if len(dt_rx_ns) < 8:
        return None, None
    rate = 1.0 / np.median(np.diff(times_s))          # samples/s
    phase = dt_rx_ns * 1e-9                            # seconds
    # remove the (large) linear clock ramp; TDEV doesn't remove freq offset
    coeffs = np.polyfit(np.arange(len(phase)), phase, 1)
    phase = phase - np.polyval(coeffs, np.arange(len(phase)))
    t2, td, _err, _n = allantools.tdev(phase, rate=rate, data_type="phase",
                                       taus=taus)
    return t2, td


def main():
    arms = {}
    for a in sys.argv[1:]:
        if "=" not in a:
            continue
        name, path = a.split("=", 1)
        arms[name] = path
    if not arms:
        print(__doc__)
        sys.exit(1)

    taus = [30, 60, 120, 240, 480, 960]
    series = {}
    for name, path in arms.items():
        ts, dt = parse_dtrx(path)
        print("%-10s %d dt_rx samples, span %.0f min, median Δt=%.0fs"
              % (name, len(dt), (ts[-1] - ts[0]) / 60 if len(ts) > 1 else 0,
                 np.median(np.diff(ts)) if len(ts) > 1 else 0))
        t2, td = tdev_curve(ts, dt, taus)
        if td is not None:
            series[name] = dict(zip([round(x) for x in t2], td))

    print("\nTDEV (ps), linearly-detrended dt_rx:")
    hdr = "  %-7s" % "tau(s)" + "".join("%12s" % n for n in arms)
    print(hdr)
    for tau in taus:
        row = "  %-7d" % tau
        for n in arms:
            v = series.get(n, {}).get(tau)
            row += "%12s" % ("%.1f" % (v * 1e12) if v is not None else "-")
        print(row)
    # headline ratio if both broadcast and has present
    if "broadcast" in series and "has" in series:
        print("\nHAS/broadcast TDEV ratio (lower=HAS better):")
        for tau in taus:
            b = series["broadcast"].get(tau)
            h = series["has"].get(tau)
            if b and h:
                print("  tau=%-5d  %.2f" % (tau, h / b))


if __name__ == "__main__":
    main()

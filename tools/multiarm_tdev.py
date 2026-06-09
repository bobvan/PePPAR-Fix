#!/usr/bin/env python3
"""Pairwise-difference TDEV for the multi-arm dt_rx comparison.

Consumes the CSV from tools/has_multiarm_compare.py (columns
dt_rx_<arm>_ns) and, for each correction arm, computes TDEV of the
*difference* against a reference arm (default: broadcast).  On identical
observations the rx-TCXO realization is common to all arms, so the
difference cancels it — leaving the differential correction stability
(the thing we actually want to compare).  The mean difference is the
inter-source clock-datum offset (removed before TDEV).

    multiarm_tdev.py multiarm.csv [--ref broadcast] [--skip 120]
"""
import argparse
import csv

import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv")
    ap.add_argument("--ref", default="broadcast")
    ap.add_argument("--skip", type=int, default=0,
                    help="drop the first N epochs (warm-up / arm not yet engaged)")
    ap.add_argument("--rate", type=float, default=1.0, help="samples/s")
    args = ap.parse_args()

    rows = list(csv.DictReader(open(args.csv)))[args.skip:]
    arms = [c[len("dt_rx_"):-len("_ns")] for c in rows[0]
            if c.startswith("dt_rx_")]
    d = {a: np.array([float(r["dt_rx_%s_ns" % a]) for r in rows]) for a in arms}
    print("epochs=%d  arms=%s  ref=%s" % (len(rows), arms, args.ref))

    import allantools
    taus = [1, 2, 4, 8, 16, 32, 64, 128, 256]

    def _tdev_ps(series):
        x = np.arange(len(series))
        s = series - np.polyval(np.polyfit(x, series, 1), x)
        t2, td, _e, _n = allantools.tdev(s * 1e-9, rate=args.rate,
                                         data_type="phase", taus=taus)
        return {round(t): v for t, v in zip(t2, td)}

    # ABSOLUTE TDEV of the reference arm — the actual receiver-clock
    # stability (rx-TCXO-dominated on a DO-less host).  Printed first so the
    # tiny DIFFERENCE numbers below are read in context: they are NOT clock
    # stability — the pairwise difference cancels this rx-TCXO floor to
    # expose the (much smaller) inter-source correction differential.
    try:
        absc = _tdev_ps(d[args.ref])
        print("\nABSOLUTE TDEV(dt_rx_%s) — the real clock, rx-TCXO floor (ps):"
              % args.ref)
        print("  %-9s %9s" % ("", "") + "".join("%9d" % t for t in taus))
        print("  %-9s %9s" % ("clock", "") +
              "".join("%9.0f" % (absc.get(t, float('nan')) * 1e12) for t in taus))
    except Exception:
        pass

    print("\nΔ vs %s: mean (ns, =datum offset) and TDEV of detrended Δ (ps)"
          "  [below the rx-TCXO floor above — inter-source differential, not"
          " clock stability]:" % args.ref)
    hdr = "  %-9s %9s" % ("arm", "mean_ns") + "".join("%9d" % t for t in taus)
    print(hdr)
    for a in arms:
        if a == args.ref:
            continue
        diff = d[a] - d[args.ref]
        mean = diff.mean()
        # _tdev_ps detrends linear (removes datum offset + slow common ramp)
        try:
            cells = _tdev_ps(diff)
        except Exception:
            cells = {}
        row = "  %-9s %+9.3f" % (a, mean)
        for t in taus:
            v = cells.get(t)
            row += "%9s" % ("%.1f" % (v * 1e12) if v is not None else "-")
        print(row)
    print("\n(lower TDEV = arm's dt_rx is more stable than broadcast at that τ; "
          "rx-TCXO cancels in the difference.)")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""analyze_utc_link — compare our receiver clock against a national timescale.

Differences two PRIDE ``rck_*`` receiver-clock series that were produced from
the SAME precise products, which is what makes the comparison meaningful:

    rck(ours) = [our rx clock  - product clock datum]
    rck(ref)  = [UTC(k)        - product clock datum]
    ---------------------------------------------------------------
    difference = [our rx clock - UTC(k)]      datum cancels exactly

**Both sides must come from one product set.** Differencing our real-time
SSR solution against BIPM's published final-product .gpi does NOT cancel the
datum — IGS-RTS and IGS-final have different satellite-clock datums whose
offset wanders at the ns level, i.e. exactly the size of the thing being
measured.  Run both ends through the same pdp3 invocation instead; that is
why BIPM runs every lab through one common product set.

Reference stations whose receiver clock IS a national timescale (site logs
verified 2026-08-05, all free/anonymous from BKG's IGS obs tree):

    NIST00USA   UTC(NIST)     H-maser, Boulder CO
    USN700USA   UTC(USNO)     H-maser on Master Clock MC2
    AMC400USA   UTC(USNO) alt Alternate Master Clock
    NLIB00USA   (unlabelled)  H-maser, North Liberty VLBA — method test only

WHAT CAN BE CLAIMED WITHOUT A DELAY CALIBRATION:
  * frequency, at full accuracy — an unknown constant delay is an offset,
    not a rate, so it cancels in every rate measurement
  * stability (ADEV/TDEV/MDEV), holdover, step detection
  NOT any sentence beginning "traceable to".  The absolute offset carries
  uncalibrated receiver/cable/antenna delays worth tens to hundreds of ns.

WHAT THIS MEASURES, PRECISELY: the *receiver* clock in the RINEX, not the
disciplined oscillator.  On a PePPAR-Fix host the F9T runs on its own rx
TCXO; the DO is steered separately.  Extending this to the DO needs the
TICC chA-chB term added on top.  Say "rx clock", not "our clock".

Usage:
    analyze_utc_link.py --ours rck_2026206_piface --ref rck_2026206_nist \\
        [--ref-name UTC(NIST)] [--plot out.png]
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from datetime import datetime, timezone

TOOL_VERSION = "1.0.0"          # stamp on every plot/report (lab convention)
C_M_PER_S = 299792458.0

_SYS_COL = {"GPS": 6, "GLONASS": 7, "Galileo": 8, "BDS-2": 9,
            "BDS-3": 10, "QZSS": 11}


def read_rck(path: str, system: str = "GPS"):
    """PRIDE rck_* → (list[datetime], list[float ns]).

    Values in the file are metres of receiver clock offset; converted to ns.
    Rows where the requested constellation is exactly 0.0 are dropped —
    PRIDE writes 0.0 for constellations it did not solve, and a literal
    zero is never a real clock offset here.
    """
    col = _SYS_COL[system]
    ts, ns = [], []
    with open(path, errors="replace") as f:
        for line in f:
            p = line.split()
            if len(p) <= col:
                continue
            try:
                y, mo, d, h, mi = (int(p[0]), int(p[1]), int(p[2]),
                                   int(p[3]), int(p[4]))
                sec = float(p[5])
                val = float(p[col])
            except ValueError:
                continue          # header / comment line
            if not (1980 <= y <= 2100) or val == 0.0:
                continue
            ts.append(datetime(y, mo, d, h, mi, int(sec),
                               tzinfo=timezone.utc))
            ns.append(val / C_M_PER_S * 1e9)
    return ts, ns


def align(t_a, v_a, t_b, v_b, max_gap_s: float = 60.0):
    """Interpolate series B onto series A's timestamps → (times, a, b_interp).

    Exact-timestamp matching does NOT work here.  A geodetic reference
    station stamps epochs on integer seconds (00.000, 30.000); PePPAR-Fix's
    RINEX writer stamps them at whatever sub-second phase the run happened
    to start on (e.g. 06.987), so the two grids can be offset by seconds and
    never coincide — zero common epochs even though both are 30 s series.

    Linear interpolation of the *reference* onto our epochs is the right
    fix, not merely a workaround: a maser on a national timescale is smooth
    at these intervals, so interpolation error is far below the noise we are
    trying to measure.  Points more than ``max_gap_s`` from a reference
    sample are dropped rather than extrapolated.
    """
    t, a, b = [], [], []
    j = 0
    n = len(t_b)
    for ti, ai in zip(t_a, v_a):
        while j + 1 < n and t_b[j + 1] < ti:
            j += 1
        if j + 1 >= n:
            break
        t0, t1 = t_b[j], t_b[j + 1]
        if ti < t0:
            continue
        span = (t1 - t0).total_seconds()
        if span <= 0 or span > max_gap_s:
            continue
        if (ti - t0).total_seconds() > max_gap_s:
            continue
        w = (ti - t0).total_seconds() / span
        t.append(ti)
        a.append(ai)
        b.append(v_b[j] + w * (v_b[j + 1] - v_b[j]))
    return t, a, b


def linear_fit(x, y):
    """Least-squares slope+intercept without numpy (kept dependency-light)."""
    n = len(x)
    mx = sum(x) / n
    my = sum(y) / n
    sxx = sum((xi - mx) ** 2 for xi in x)
    sxy = sum((xi - mx) * (yi - my) for xi, yi in zip(x, y))
    slope = sxy / sxx if sxx else 0.0
    return slope, my - slope * mx


def stability(vals_ns, tau0_s):
    """ADEV + TDEV vs tau via allantools, on a phase series in ns."""
    try:
        import allantools
        import numpy as np
    except ImportError:
        return None
    phase_s = np.asarray(vals_ns, dtype=float) * 1e-9
    taus, adev, _, _ = allantools.oadev(
        phase_s, rate=1.0 / tau0_s, data_type="phase", taus="octave")
    _, tdev, _, _ = allantools.tdev(
        phase_s, rate=1.0 / tau0_s, data_type="phase", taus="octave")
    return list(zip(taus, adev, tdev))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--ours", required=True, help="our PRIDE rck_* file")
    ap.add_argument("--ref", required=True, help="reference-station rck_* file")
    ap.add_argument("--ref-name", default="UTC(ref)")
    ap.add_argument("--ours-name", default="our rx clock")
    ap.add_argument("--system", default="GPS", choices=sorted(_SYS_COL))
    ap.add_argument("--plot", default=None, help="write a PNG here")
    args = ap.parse_args(argv)

    ta, va = read_rck(args.ours, args.system)
    tb, vb = read_rck(args.ref, args.system)
    if not ta or not tb:
        print("error: one of the series is empty", file=sys.stderr)
        return 1
    t, a, b = align(ta, va, tb, vb)
    if len(t) < 10:
        print(f"error: only {len(t)} common epochs", file=sys.stderr)
        return 1

    diff = [ai - bi for ai, bi in zip(a, b)]
    tau0 = (t[1] - t[0]).total_seconds()
    secs = [(ti - t[0]).total_seconds() for ti in t]
    slope_ns_per_s, _ = linear_fit(secs, diff)
    resid = [d - (slope_ns_per_s * s) for d, s in zip(diff, secs)]
    mean_r = sum(resid) / len(resid)
    rms = math.sqrt(sum((r - mean_r) ** 2 for r in resid) / len(resid))

    print("=" * 68)
    print(f"analyze_utc_link v{TOOL_VERSION}   system={args.system}")
    print(f"  ours : {os.path.basename(args.ours)}  ({len(ta)} epochs)")
    print(f"  ref  : {os.path.basename(args.ref)}  ({len(tb)} epochs)")
    print(f"  common epochs: {len(t)}   tau0 = {tau0:.0f} s")
    print(f"  span : {t[0]:%Y-%m-%d %H:%M} .. {t[-1]:%H:%M} UTC")
    print("=" * 68)
    print(f"\n[{args.ours_name}] - [{args.ref_name}]")
    print(f"  mean offset      : {sum(diff)/len(diff):+.1f} ns   "
          "(UNCALIBRATED — includes receiver/cable/antenna delay)")
    print(f"  peak-to-peak     : {max(diff)-min(diff):.1f} ns")
    print(f"  frequency offset : {slope_ns_per_s*1e-9:+.3e}  "
          f"({slope_ns_per_s*86400:+.1f} ns/day)   <- claimable at full accuracy")
    print(f"  RMS about the fit: {rms:.2f} ns")

    st = stability(resid, tau0)
    if st:
        print(f"\n  stability of the difference (detrended):")
        print(f"  {'tau (s)':>9}  {'ADEV':>11}  {'TDEV (ns)':>11}")
        for tau, ad, td in st:
            if tau > (secs[-1] / 4):
                break
            print(f"  {tau:9.0f}  {ad:11.3e}  {td*1e9:11.3f}")
    else:
        print("\n  (allantools not installed — no stability table)")

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            hrs = [s / 3600.0 for s in secs]
            fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=False)
            ax[0].plot(hrs, diff, lw=0.7)
            ax[0].set_xlabel("hours from start")
            ax[0].set_ylabel("ns")
            ax[0].set_title(f"[{args.ours_name}] - [{args.ref_name}]   "
                            f"{t[0]:%Y-%m-%d}")
            ax[0].grid(alpha=.3)
            if st:
                taus = [x[0] for x in st]
                tdevs = [x[2] * 1e9 for x in st]
                ax[1].loglog(taus, tdevs, "o-")
                ax[1].set_xlabel("tau (s)")
                ax[1].set_ylabel("TDEV (ns)")
                ax[1].grid(alpha=.3, which="both")
            fig.suptitle(f"analyze_utc_link v{TOOL_VERSION}  "
                         f"system={args.system}  n={len(t)}", fontsize=8)
            fig.tight_layout()
            fig.savefig(args.plot, dpi=110)
            print(f"\n  plot: {args.plot}")
        except ImportError:
            print("\n  (matplotlib not installed — no plot)")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Characterize a FREE-RUNNING oscillator from PRIDE receiver-clock files.

When a GNSS receiver is clocked by an oscillator that nothing is steering,
the receiver-clock estimate PRIDE recovers IS that oscillator measured
against GPS time.  No TICC, no extra wiring, no second channel — the
observable falls out of the PPP solution for free.  That is what this
script analyzes.

Companion to ``analyze_utc_link.py``, which differences TWO clock series
(ours vs a reference station).  This one takes ONE series, or several
contiguous segments of one series, and reports its stability.

What it does, in order:

1. **Stitch** the segments in time order.  PRIDE processes per UTC day, so
   a multi-day hold arrives as several files.  Segments are checked for
   overlap and for gaps; a gap does not invalidate the analysis but it is
   reported, because a gap that straddles a tau bin quietly biases that
   bin.
2. **Remove the frequency offset** by a least-squares line.  A free-running
   oscillator sits at some arbitrary fractional-frequency offset from GPS;
   that offset is a calibration fact, not a stability one, and if left in
   it dominates every statistic.  The fitted slope IS the frequency offset
   and is reported.  Curvature (drift/aging) is reported but NOT removed —
   removing it would hide real oscillator behaviour.
3. **ADEV and TDEV** vs tau, via allantools on the phase series.

Units: PRIDE rck columns are in METRES of light travel.  They are divided
by c here, exactly as analyze_utc_link.py does.  (Getting this backwards
turns a perfectly good oscillator into a spectacular or a catastrophic
one, which is why it is stated here and asserted below.)

Usage:
  analyze_freerun_clock.py --rck rck_ol220 rck_ol221 \\
      --name "STP3593LF free-running" --plot freerun.png

Reports ADEV because for a free-running oscillator the frequency-domain
question ("how good is this crystal?") is the one being asked.  TDEV is
printed alongside since the rest of the lab's budget work is in TDEV.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import subprocess
import sys

TOOL_VERSION = "1.0.0"          # stamp on every plot/report (lab convention)
C_M_PER_S = 299792458.0

# Column index of the per-system receiver clock in a PRIDE rck_* row.
_SYS_COL = {"GPS": 6, "GLONASS": 7, "Galileo": 8, "BDS-2": 9,
            "BDS-3": 10, "QZSS": 11}


def git_sha() -> str:
    """Short SHA of the tree this ran from, for the plot stamp."""
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=10,
                             cwd=__file__.rsplit("/", 2)[0])
        if out.returncode == 0:
            return out.stdout.strip() or "unknown"
    except Exception:
        pass
    return "unknown"


def read_rck(path: str, system: str = "GPS"):
    """Return (epoch_datetimes, phase_ns) from a PRIDE rck_* file.

    Rows are data only when they start with a plausible year and carry the
    full column set; the file's header lines are skipped by that test.
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
                continue
            if y < 1980:
                continue
            if val == 0.0:
                # PRIDE writes 0.0 for systems it did not solve; a genuine
                # clock estimate of exactly zero does not occur.
                continue
            t = (_dt.datetime(y, mo, d, h, mi, tzinfo=_dt.timezone.utc)
                 + _dt.timedelta(seconds=sec))
            ts.append(t)
            ns.append(val / C_M_PER_S * 1e9)
    return ts, ns


def stitch(segments):
    """Merge per-file (times, ns) in time order; report gaps and overlaps."""
    allpts = []
    for path, (ts, ns) in segments:
        if not ts:
            print(f"  WARNING: {path} yielded no usable rows", file=sys.stderr)
            continue
        allpts.extend(zip(ts, ns, [path] * len(ts)))
    allpts.sort(key=lambda r: r[0])

    notes = []
    for i in range(1, len(allpts)):
        dt = (allpts[i][0] - allpts[i - 1][0]).total_seconds()
        if dt <= 0:
            notes.append(f"overlap/duplicate at {allpts[i][0]} "
                         f"({allpts[i-1][2]} -> {allpts[i][2]})")
    return allpts, notes


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rck", required=True, nargs="+",
                    help="one or more PRIDE rck_* files (contiguous segments "
                         "of the SAME oscillator; stitched in time order)")
    ap.add_argument("--name", default="free-running oscillator",
                    help="label for reports and the plot")
    ap.add_argument("--system", default="GPS", choices=sorted(_SYS_COL))
    ap.add_argument("--plot", default=None, help="write a PNG here")
    args = ap.parse_args(argv)

    try:
        import numpy as np
    except ImportError:
        print("numpy required (use the repo venv)", file=sys.stderr)
        return 2

    segments = [(p, read_rck(p, args.system)) for p in args.rck]
    for p, (ts, _) in segments:
        span = ((ts[-1] - ts[0]).total_seconds() / 3600.0) if ts else 0.0
        print(f"  {p}: {len(ts)} epochs, {span:.2f} h"
              + (f", {ts[0]:%Y-%m-%d %H:%M} -> {ts[-1]:%H:%M} UTC" if ts else ""))

    pts, notes = stitch(segments)
    if len(pts) < 10:
        print("not enough data to analyze", file=sys.stderr)
        return 1
    for n in notes:
        print(f"  NOTE: {n}")

    t0 = pts[0][0]
    secs = np.array([(r[0] - t0).total_seconds() for r in pts], dtype=float)
    phase_ns = np.array([r[1] for r in pts], dtype=float)

    # Sampling interval: the mode of the diffs, not the mean — a single
    # large gap would drag the mean and mislabel every tau.
    d = np.diff(secs)
    tau0 = float(np.median(d))
    gaps = d[d > tau0 * 1.5]
    span_h = secs[-1] / 3600.0

    print(f"\n=== {args.name} ===")
    print(f"  epochs            : {len(pts)}")
    print(f"  span              : {span_h:.2f} h "
          f"({t0:%Y-%m-%d %H:%M} -> {pts[-1][0]:%Y-%m-%d %H:%M} UTC)")
    print(f"  sample interval   : {tau0:.1f} s")
    if len(gaps):
        print(f"  gaps              : {len(gaps)}, largest {gaps.max():.0f} s "
              f"({gaps.sum():.0f} s total missing)")
    else:
        print(f"  gaps              : none")

    # --- frequency offset (the line) and drift (the curvature) ----------
    # Fit in seconds-of-phase vs seconds-of-time so the slope is directly
    # fractional frequency, dimensionless.
    phase_s = phase_ns * 1e-9
    lin = np.polyfit(secs, phase_s, 1)
    y0 = float(lin[0])                       # fractional frequency offset
    quad = np.polyfit(secs, phase_s, 2)
    drift_per_day = float(quad[0]) * 2.0 * 86400.0   # d(freq)/dt over a day

    print(f"\n  frequency offset  : {y0:+.3e}  ({y0*1e9:+.3f} ppb)")
    print(f"  drift (aging+temp): {drift_per_day:+.3e} /day")
    print(f"  phase excursion   : {phase_ns.max()-phase_ns.min():.1f} ns raw, "
          f"{np.ptp(phase_s - np.polyval(lin, secs))*1e9:.1f} ns detrended")

    resid_ns = (phase_s - np.polyval(lin, secs)) * 1e9

    # --- ADEV / TDEV ----------------------------------------------------
    try:
        import allantools
    except ImportError:
        print("\nallantools not installed — install the [analysis] extra "
              "for ADEV/TDEV", file=sys.stderr)
        return 0

    # Feed the DETRENDED phase: the constant frequency offset is a
    # calibration, not an instability, and at long tau it would otherwise
    # swamp ADEV entirely.
    x = resid_ns * 1e-9
    rate = 1.0 / tau0
    taus_out, ad, _, _ = allantools.oadev(x, rate=rate, data_type="phase",
                                          taus="octave")
    taus_t, td, _, _ = allantools.tdev(x, rate=rate, data_type="phase",
                                       taus="octave")
    tdmap = dict(zip(taus_t, td))

    print(f"\n  {'tau (s)':>9}  {'ADEV':>11}  {'TDEV (ps)':>11}")
    for tau, a in zip(taus_out, ad):
        t = tdmap.get(tau)
        print(f"  {tau:9.0f}  {a:11.3e}  "
              + (f"{t*1e12:11.1f}" if t is not None else f"{'—':>11}"))

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not installed — skipping plot", file=sys.stderr)
            return 0
        hrs = secs / 3600.0
        fig, ax = plt.subplots(2, 1, figsize=(10, 7))
        ax[0].plot(hrs, resid_ns, lw=0.7)
        ax[0].set_xlabel("hours from start")
        ax[0].set_ylabel("ns")
        ax[0].set_title(f"{args.name} — phase vs GPS, frequency offset "
                        f"({y0*1e9:+.2f} ppb) removed")
        ax[0].grid(alpha=0.3)

        ax[1].loglog(taus_out, ad, "o-", label="ADEV")
        ax[1].set_xlabel("tau (s)")
        ax[1].set_ylabel("ADEV")
        ax[1].set_title("Allan deviation, free-running")
        ax[1].grid(alpha=0.3, which="both")
        ax[1].legend()

        fig.suptitle(f"analyze_freerun_clock v{TOOL_VERSION}  "
                     f"git {git_sha()}  {span_h:.1f} h @ {tau0:.0f} s  "
                     f"{args.system}", fontsize=8)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=120)
        print(f"\n  plot: {args.plot}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

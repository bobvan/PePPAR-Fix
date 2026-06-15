#!/usr/bin/env python3
"""ppp_time_transfer.py — manual GNSS PPP time transfer + pairwise ADEV/TDEV.

The AtomiChron-suggested experiment, runnable: PPP each site's RINEX (CSRS-PPP
or our PRIDE pdp3), each gives a receiver-clock-vs-IGS-time series; difference
them pairwise to get inter-clock offsets (the common IGS timescale cancels);
ADEV/TDEV of each difference tells the stability story.

  your clock − PTBB   → your clock vs PTB's H-maser
  your clock − SPT0   → vs RISE/Borås H-maser
  PTBB    − SPT0       → the *method noise floor* (two great masers → the scatter
                         is the PPP transfer noise, not the clocks)

Each input is a 2-column clock series "epoch_s  clock" extracted from a PPP run:
  - PRIDE pdp3:  the `rck_YYYYDDD_<site>` file (receiver clock; check units —
                 PRIDE writes clock in metres or ns, set --scale-to-ns).
  - CSRS-PPP:    the per-epoch clock column of the .pos / full output (the same
                 series the report's "Station Clock Offset" plot draws).
Use --col / --time-col / --scale-to-ns to map whatever columns your file has;
defaults assume "epoch_s, clock_ns".

Phase offset vs stability: the differenced series IS the phase offset (ns) — but
its absolute level carries each receiver's uncalibrated equipment delay (a
constant). The *changes* (drift) and ADEV/TDEV are calibration-free.

Usage:
  ppp_time_transfer.py --source MINE=mine.clk --source PTBB=ptbb.clk \
      --source SPT0=spt0.clk --out-dir plots
  ppp_time_transfer.py --demo --out-dir /tmp/tt    # synthetic self-test
"""
import argparse
import itertools
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

C = 299792458.0


def load_series(path, time_col=0, col=1, scale_to_ns=1.0):
    """Return (epoch_s [int-safe float], clock_ns) from a 2+ column text file.
    Skips header/comment lines that don't parse as numbers."""
    t, y = [], []
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) <= max(time_col, col):
                continue
            try:
                t.append(float(p[time_col])); y.append(float(p[col]) * scale_to_ns)
            except ValueError:
                continue
    return np.asarray(t), np.asarray(y)


def align(series):
    """Align N (t, y) series onto their common epochs (rounded to 1 s)."""
    keyed = [{int(round(ti)): yi for ti, yi in zip(t, y)} for t, y in series]
    common = sorted(set.intersection(*[set(k) for k in keyed]))
    if len(common) < 16:
        raise SystemExit(f"only {len(common)} common epochs — need overlapping spans")
    tc = np.asarray(common, dtype=float)
    cols = [np.asarray([k[e] for e in common]) for k in keyed]
    return tc, cols


def allan(phase_ns, rate_hz, metric):
    """(taus, value) for the chosen metric, using that metric's own tau grid."""
    import allantools
    ph_s = np.asarray(phase_ns) * 1e-9
    fn = allantools.tdev if metric == "tdev" else allantools.oadev
    taus, val, _, _ = fn(ph_s, rate=rate_hz, data_type="phase", taus="octave")
    return taus, val


def _demo():
    """Synthetic: two H-masers (tiny noise) + one OCXO (noisier) on a 1 s grid."""
    rng = np.random.default_rng(7)
    n = 86400
    t = np.arange(n, dtype=float)
    def maser():           # white phase ~30 ps + slow random-walk freq
        wp = rng.normal(0, 0.03, n)
        rw = np.cumsum(rng.normal(0, 2e-6, n))      # ns
        return wp + rw
    def ocxo():            # white FM → random-walk phase, ~0.5 ns/√s-ish
        return np.cumsum(rng.normal(0, 0.05, n))    # ns
    return t, {"PTBB": maser(), "SPT0": maser(), "MINE-OCXO": ocxo() + 12.0}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", action="append", default=[], metavar="LABEL=FILE")
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--time-col", type=int, default=0)
    ap.add_argument("--col", type=int, default=1)
    ap.add_argument("--scale-to-ns", type=float, default=1.0,
                    help="multiply clock col by this to get ns (PRIDE metres → 1e9/c)")
    ap.add_argument("--rate-hz", type=float, default=None, help="default: infer from epochs")
    ap.add_argument("--metric", choices=["adev", "tdev"], default="tdev")
    ap.add_argument("--demo", action="store_true")
    args = ap.parse_args()

    if args.demo:
        t, named = _demo()
        labels = list(named); series = [(t, named[l]) for l in labels]
        rate = 1.0
    else:
        if len(args.source) < 2:
            print("need ≥2 --source LABEL=FILE", file=sys.stderr); return 2
        labels, series = [], []
        for spec in args.source:
            lab, path = spec.split("=", 1)
            labels.append(lab)
            series.append(load_series(path, args.time_col, args.col, args.scale_to_ns))
        tc, cols = align(series)
        series = [(tc, c) for c in cols]
        rate = args.rate_hz or 1.0 / np.median(np.diff(series[0][0]))

    tc, cols = align([(t, y) for t, y in series]) if not args.demo else (t, [named[l] for l in labels])

    fig, ax = plt.subplots(figsize=(16, 9))
    print(f"pairwise {args.metric.upper()} (rate {rate:g} Hz):")
    for i, j in itertools.combinations(range(len(labels)), 2):
        diff = cols[i] - cols[j]
        diff = diff - diff.mean()
        taus, y = allan(diff, rate, args.metric)
        lab = f"{labels[i]} − {labels[j]}"
        ax.loglog(taus, y, "-o", ms=4, lw=2.2, label=f"{lab}  (1s {y[0]*1e12 if args.metric=='tdev' else y[0]:.0f}"
                  + (" ps)" if args.metric == "tdev" else " )"))
        print(f"  {lab:22s} τ=1s {args.metric}={y[0]:.3e}")
    ax.set_xlabel("averaging time τ  [s]")
    ax.set_ylabel("TDEV  [s]" if args.metric == "tdev" else "overlapping ADEV")
    ax.grid(alpha=0.3, which="both")
    ax.legend(loc="best")
    ax.set_title("PPP time transfer — pairwise " + args.metric.upper() + "\n"
                 "maser↔maser pair = method noise floor; your clock sits above it")
    fig.tight_layout()
    os.makedirs(args.out_dir, exist_ok=True)
    out = os.path.join(args.out_dir, "ppp_time_transfer")
    fig.savefig(out + ".pdf"); fig.savefig(out + ".png", dpi=200)
    plt.close(fig)
    print(f"wrote {out}.pdf/.png")


if __name__ == "__main__":
    sys.exit(main())

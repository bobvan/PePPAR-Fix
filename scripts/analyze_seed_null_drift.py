#!/usr/bin/env python3
"""Analyze a seed-at-truth null-drift experiment (I-211101 follow-up).

Bob's test (2026-06-05): seed AntPosEst at the surveyed truth with the
position state FREE to move (``--known-pos`` truth, NO ``--pin-position``,
DEFAULT ``--q-pos-converged``), then watch.  A null is a property of
(geometry, model), independent of the seed:

  * Seed-bias hypothesis  -> seeded at truth, STAYS at truth.
  * Null-drift hypothesis -> seeded at truth, DRIFTS away anyway.

They make opposite predictions, so one run falsifies the other.  This
script parses the engine's ``[AntPosEst]`` log lines from one or more
hosts (two on the SAME antenna for the common-mode check) and prints a
prose verdict against the three-outcome decision tree.  Drift slope is
the seed-independent signal and needs NO truth; pass ``--truth-alt-m``
only to also report the offset-from-truth.

The decisive built-in signal is ``worstσ`` vs ``positionσ``: the engine
documents "a rising worstσ without matching rise in positionσ" as the
null-mode-excitation signature (peppar_fix_engine.py, Bravo 2026-04-23
PRIDE arc).  We surface that divergence directly.

Output is prose (Bob is often away from plots); ``--png PATH`` adds an
optional overlay.

Usage:
    analyze_seed_null_drift.py HOSTA.log [HOSTB.log] [--truth-alt-m ALT] \\
        [--labels clkPoC3,PiFace] [--png drift.png]

    (ALT = surveyed ARP ellipsoidal height from timelab/antennas.json — do
    NOT hardcode lab coordinates here; the repo guard rejects them.)
"""

from __future__ import annotations

import argparse
import math
import re
from datetime import datetime

import numpy as np

# [AntPosEst N] positionσ=0.043m pos=(LAT, LON, ALT) n=18 amb=...
#   ... ZTD=-425±11mm ... tide=187mm(U+186) ... worstσ=1.3m
# (LAT/LON are decimal degrees, ALT ellipsoidal metres; placeholders here
#  because the repo guard rejects committed lab coordinates.)
_TS = r"(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}[,.]\d{3})"
_LINE = re.compile(
    _TS + r".*\[AntPosEst\s+\d+\]\s+positionσ=([\d.]+)m\s+"
    r"pos=\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)"
)
_ZTD = re.compile(r"ZTD=([+-]?\d+)±(\d+)mm")
_WORST = re.compile(r"worstσ=([\d.]+)m")
_TIDE = re.compile(r"tide=(\d+)mm\(U([+-]?\d+)\)")


def _parse_ts(s: str) -> datetime:
    return datetime.strptime(s.replace(",", "."), "%Y-%m-%d %H:%M:%S.%f")


def parse_log(path: str) -> dict:
    """Return parallel arrays t_hours, alt_m, ztd_mm, posig_m, worst_m, tide_up_mm."""
    t0 = None
    th, alt, ztd, posig, worst, tide = [], [], [], [], [], []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            m = _LINE.search(line)
            if not m:
                continue
            ts = _parse_ts(m.group(1))
            if t0 is None:
                t0 = ts
            th.append((ts - t0).total_seconds() / 3600.0)
            posig.append(float(m.group(2)))
            alt.append(float(m.group(5)))
            z = _ZTD.search(line)
            ztd.append(float(z.group(1)) if z else math.nan)
            w = _WORST.search(line)
            worst.append(float(w.group(1)) if w else math.nan)
            td = _TIDE.search(line)
            tide.append(float(td.group(2)) if td else math.nan)
    return {
        "t": np.array(th), "alt": np.array(alt), "ztd": np.array(ztd),
        "posig": np.array(posig), "worst": np.array(worst),
        "tide_up": np.array(tide), "n": len(th),
    }


def _fit_slope(t: np.ndarray, y: np.ndarray):
    """Linear fit y = a*t + b.  Returns (slope_per_hr, intercept,
    resid_std, slope_sigma).  NaNs dropped."""
    ok = np.isfinite(t) & np.isfinite(y)
    t, y = t[ok], y[ok]
    if len(t) < 3 or np.ptp(t) <= 0:
        return math.nan, math.nan, math.nan, math.nan
    A = np.vstack([t, np.ones_like(t)]).T
    (a, b), res, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = A @ np.array([a, b])
    resid = y - pred
    rstd = float(np.std(resid, ddof=2)) if len(t) > 2 else math.nan
    # slope 1-sigma from the standard OLS covariance
    sxx = float(np.sum((t - t.mean()) ** 2))
    sa = rstd / math.sqrt(sxx) if (sxx > 0 and math.isfinite(rstd)) else math.nan
    return float(a), float(b), rstd, sa


def summarize_host(label: str, d: dict, truth_alt: float | None,
                   skip_hr: float = 0.0) -> dict:
    span_hr = np.ptp(d["t"]) if d["n"] > 1 else 0.0
    # Drop the warmup window: the AntPosEst filter re-converges from the
    # seed over the first ~10-20 min (Phase-1 bootstrap transient), during
    # which worstσ can be 1000+ m and the position settles — neither is
    # the steady-state drift we're measuring.  Also drops the per-relaunch
    # transient if the wrapper relaunched mid-run.
    keep = d["t"] >= skip_hr
    n_kept = int(keep.sum())
    if n_kept < 3:
        keep = np.ones_like(d["t"], dtype=bool)  # too short — keep all
        n_kept = int(keep.sum())
    t, alt, ztd = d["t"][keep], d["alt"][keep], d["ztd"][keep]
    worst, posig, tide = d["worst"][keep], d["posig"][keep], d["tide_up"][keep]

    a, b, rstd, sa = _fit_slope(t, alt)
    za, zb, zrstd, zsa = _fit_slope(t, ztd)
    # correlation alt vs ztd
    ok = np.isfinite(alt) & np.isfinite(ztd)
    with np.errstate(invalid="ignore", divide="ignore"):
        corr = (float(np.corrcoef(alt[ok], ztd[ok])[0, 1])
                if ok.sum() > 3 and np.std(ztd[ok]) > 0 and np.std(alt[ok]) > 0
                else math.nan)
    worst_max = float(np.nanmax(worst)) if np.isfinite(worst).any() else math.nan
    posig_max = float(np.nanmax(posig)) if n_kept else math.nan
    tide_rng = (float(np.nanmax(tide) - np.nanmin(tide))
                if np.isfinite(tide).any() else math.nan)
    # Rebind so the rest of the function reads the post-warmup series.
    d = {**d, "t": t, "alt": alt, "ztd": ztd, "worst": worst,
         "posig": posig, "tide_up": tide, "n": n_kept}
    # slope significance: |slope| vs 3-sigma
    sig = (abs(a) > 3 * sa) if (math.isfinite(a) and math.isfinite(sa) and sa > 0) else None

    print(f"\n=== {label}  ({d['n']} epochs over {span_hr:.2f} h) ===")
    if d["n"] < 3:
        print("  too few [AntPosEst] epochs parsed — check the log path/format.")
        return {"slope": math.nan, "alt_series": d}
    print(f"  alt drift slope : {a*1000:+.1f} mm/h  (±{sa*1000:.1f} mm/h 1σ, "
          f"resid {rstd*1000:.0f} mm)  "
          + ("SIGNIFICANT" if sig else "not distinguishable from 0" if sig is False else ""))
    print(f"  alt over window : {d['alt'][0]:.3f} -> {d['alt'][-1]:.3f} m  "
          f"(net {(d['alt'][-1]-d['alt'][0])*1000:+.0f} mm)")
    if truth_alt is not None:
        print(f"  alt vs truth    : start {(d['alt'][0]-truth_alt)*1000:+.0f} mm, "
              f"end {(d['alt'][-1]-truth_alt)*1000:+.0f} mm  (truth={truth_alt:.3f} m)")
    print(f"  ZTD resid slope : {za:+.1f} mm/h ; corr(alt,ZTD)={corr:+.2f}  "
          f"(strong anti-corr => height/tropo trade along the null)")
    print(f"  worstσ max      : {worst_max:.2f} m   positionσ max: {posig_max:.3f} m   "
          f"ratio {worst_max/posig_max:.1f}x" if math.isfinite(worst_max) and posig_max
          else f"  worstσ max      : {worst_max}")
    if math.isfinite(tide_rng):
        print(f"  tide(up) range  : {tide_rng:.0f} mm over the window "
              f"(context: this much real motion is what tide ON removes)")
    return {"slope": a, "slope_sig": sig, "alt_series": d, "corr": corr,
            "worst_ratio": (worst_max / posig_max) if posig_max else math.nan}


def verdict(hosts: list[tuple[str, dict]]):
    print("\n" + "=" * 64)
    print("VERDICT (three-outcome decision tree)")
    print("=" * 64)
    slopes = [(lbl, h["slope"]) for lbl, h in hosts if math.isfinite(h.get("slope", math.nan))]
    if not slopes:
        print("  insufficient data.")
        return
    # Drift threshold: 10 mm/h is well above per-epoch noise yet small
    # enough to catch the +0.5-1.0 m/h walks the handoff reported.
    THR = 0.010
    drifting = [(lbl, s) for lbl, s in slopes if abs(s) > THR]
    if not drifting:
        print("  [ROW 1] Held at truth (all |slope| <= 10 mm/h).")
        print("    => No null, no residual bias post-fixes.  Seed-bias premise is")
        print("       DEAD and the '+13 cm' was the Q_pos=1e-9 + tide confound.")
    else:
        signs = {math.copysign(1, s) for _, s in drifting}
        consistent = len(signs) == 1
        print(f"  Drift detected: " + ", ".join(f"{lbl} {s*1000:+.0f} mm/h" for lbl, s in drifting))
        if len(slopes) >= 2:
            # Common-mode vs differential, generalized to N hosts:
            # common-mode = mean slope across all hosts; spread = the
            # max per-host deviation from that mean.  Small spread vs
            # |common-mode| => the hosts move together (shared cause);
            # large spread => at least one host diverges (per-host cause).
            vals = np.array([s for _, s in slopes])
            # Consensus via MEDIAN (robust — one outlier can't drag it the
            # way a mean would, which matters with 3 hosts where one diverges).
            consensus = float(np.median(vals))
            spread = float(np.max(np.abs(vals - consensus)))
            tol = max(0.5 * abs(consensus), THR)  # also tolerate near-zero consensus
            outliers = [lbl for lbl, s in slopes if abs(s - consensus) > tol]
            print(f"    consensus (median) slope = {consensus*1000:+.0f} mm/h ; "
                  f"max dev = {spread*1000:.0f} mm/h ({len(slopes)} hosts)")
            for lbl, s in slopes:
                flag = "  <-- outlier" if lbl in outliers else ""
                print(f"      {lbl}: {s*1000:+.0f} mm/h  (dev {(s-consensus)*1000:+.0f}){flag}")
            if not outliers:
                print("    => COMMON-MODE drift (hosts track together): shared cause")
                print("       (geometry / atmosphere / orbit-clock), receiver-independent.")
            else:
                print(f"    => DIVERGENT: {', '.join(outliers)} depart the consensus")
                print("       => per-host cause (receiver / multipath / group-delay).")
        if consistent:
            print("  [ROW 3] Consistent one-directional drift from truth.")
            print("    => Seed-bias FALSIFIED. A real unmodeled SYSTEMATIC is forcing the")
            print("       weak axis -> hunt the bias (L5/E6 PCV detail, multipath, an")
            print("       uncorrected delay).  A better seed would NOT have helped.")
        else:
            print("  [ROW 2] Drift present but not one-directional (random walk).")
            print("    => Seed-bias FALSIFIED. True/near-null + process-noise walk ->")
            print("       observability fix (tighten Q along the null, ZTD prior, geometry).")
    print("\n  (worstσ >> positionσ on any host independently corroborates a")
    print("   near-rank-deficient / null-excited solution.)")


def maybe_plot(hosts, png):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:  # noqa: BLE001
        print(f"\n(plot skipped: {e})")
        return
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for lbl, h in hosts:
        d = h["alt_series"]
        ax1.plot(d["t"], (d["alt"] - d["alt"][0]) * 1000, label=lbl)
        ax2.plot(d["t"], d["ztd"], label=f"{lbl} ZTD-resid")
    ax1.set_ylabel("alt - alt0 (mm)"); ax1.legend(); ax1.grid(alpha=.3)
    ax1.set_title("Seed-at-truth null-drift test")
    ax2.set_ylabel("ZTD residual (mm)"); ax2.set_xlabel("hours"); ax2.legend(); ax2.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(png, dpi=110)
    print(f"\nplot written: {png}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="one or more engine log files (one per host)")
    ap.add_argument("--truth-alt-m", type=float, default=None,
                    help="surveyed ARP ellipsoidal height (m), for offset reporting")
    ap.add_argument("--labels", default=None, help="comma-sep host labels")
    ap.add_argument("--png", default=None)
    ap.add_argument("--skip-warmup-min", type=float, default=20.0,
                    help="drop the first N minutes (filter re-convergence "
                         "transient) before fitting (default 20)")
    args = ap.parse_args()

    labels = (args.labels.split(",") if args.labels
              else [p.split("/")[-1] for p in args.logs])
    hosts = []
    for lbl, path in zip(labels, args.logs):
        d = parse_log(path)
        hosts.append((lbl, summarize_host(lbl, d, args.truth_alt_m,
                                          skip_hr=args.skip_warmup_min / 60.0)))
    verdict(hosts)
    if args.png:
        maybe_plot(hosts, args.png)


if __name__ == "__main__":
    main()

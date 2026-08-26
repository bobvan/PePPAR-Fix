#!/usr/bin/env python3
"""Absolute signal-quality metrics from raw GNSS observations.

Computes code multipath and carrier-phase noise from a logged observation
stream, in a form that is comparable **across stations, receivers and sites** —
which is what a mean-removed code-minus-carrier figure is not.

    ./obs_quality.py --label PTBB  logs/ptbb_20260826_00.rtcm3
    ./obs_quality.py --label UFO1  logs/prod/ufo1_msm7_20260826_02.rtcm3

Input format (RTCM3 MSM or Septentrio SBF) is auto-detected.

WHAT IT MEASURES

*Code multipath* uses the TEQC-style dual-frequency MP combination

    MP1 = P1 - (1 + 2/(a-1))*L1 + (2/(a-1))*L2,     a = (f1/f2)^2

which cancels geometry, satellite and receiver clocks, troposphere **and the
ionosphere exactly**, leaving multipath + code noise + a per-arc constant.  Only
the arc constant is removed, so the result is absolute.

Contrast with `cmc_std` in bobvan/testAnt, which removes a per-arc *mean* from a
single-frequency P - lambda*phi.  That retains the arc's ionospheric trend.  It
is the right metric for a simultaneous A/B antenna comparison (iono is
common-mode and cancels in the difference) and the wrong one to quote as an
absolute number or compare against a fixed threshold.

*Phase noise* uses the 1 s scatter of the geometry-free combination L1-L2.  Over
one second the ionosphere is effectively frozen, so the epoch-to-epoch
difference is dominated by phase noise on the two signals.

ALWAYS REPORT THE MASK.  The same antenna measured on one dataset gives 0.79 m
unweighted over all arcs and 0.19 m restricted to C/N0 >= 50 dBHz -- a factor of
four from itself.  A code-multipath figure without its elevation or C/N0 cut is
not comparable to anything, which is why the C/N0 breakdown is printed by
default rather than being an option.

See docs/antenna-quality-metrics-and-timing.md.
"""
import argparse, collections, io, os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np

C = 299792458.0
# (f1, f2) pairs to form MP from, per constellation.  Both must be present in
# the same epoch for that SV.
PAIRS = (("GPS-L1CA", "GPS-L2W"), ("GAL-E1C", "GAL-E5aQ"))
CN0_BINS = ((0, 38), (38, 44), (44, 50), (50, 99))
MIN_ARC = 300          # epochs; shorter arcs give unstable RMS
SLIP_M = 2.0           # jump in MP that means a cycle slip / new arc


def load_epochs(path, limit_bytes):
    """-> {tow_s: {sv: {sig: (pr_m, cp_m, freq_hz, cno)}}}"""
    data = open(path, "rb").read(limit_bytes)
    ep = collections.defaultdict(lambda: collections.defaultdict(dict))
    if data[:1] == b"\xd3":
        from pyrtcm import RTCMReader
        from peppar_fix.rtcm_msm_obs import decode_msm_obs
        fmt = "RTCM3 MSM"
        for _raw, msg in RTCMReader(io.BytesIO(data), quitonerror=0):
            if msg is None or str(msg.identity) not in ("1077", "1087", "1097", "1127"):
                continue
            r = decode_msm_obs(msg)
            if not r:
                continue
            _, tow, cells = r
            _stash(ep, tow, cells)
    elif data[:2] == b"$@":
        from pysbf2 import SBFReader
        from peppar_fix.sbf_obs import decode_meas_epoch
        fmt = "Septentrio SBF"
        for _raw, msg in SBFReader(io.BytesIO(data), quitonerror=0):
            if msg is None:
                continue
            r = decode_meas_epoch(msg)
            if r is None:
                continue
            tow, cells = r
            _stash(ep, tow, cells)
    else:
        sys.exit(f"{path}: not RTCM3 (0xD3) or SBF ($@) — first bytes {data[:4]!r}")
    return ep, fmt


def _stash(ep, tow_ms, cells):
    for c in cells:
        pr, cp, f = c.get("pr_m"), c.get("cp_cyc"), c.get("freq_hz")
        if not pr or not cp or not f:
            continue
        ep[tow_ms / 1000.0][c["sv"]][c["sig_name"]] = (pr, cp * C / f, f, c.get("cno"))


def metrics(ep, first_only=False):
    mp = collections.defaultdict(list)
    gf = collections.defaultdict(list)
    for tow in sorted(ep):
        for sv, s in ep[tow].items():
            for s1, s2 in PAIRS:
                if s1 in s and s2 in s:
                    P1, L1, f1, c1 = s[s1]
                    P2, L2, f2, c2 = s[s2]
                    a = (f1 / f2) ** 2
                    k = 2.0 / (a - 1.0)
                    mp[(sv, s1)].append((tow, P1 - (1 + k) * L1 + k * L2, c1))
                    if not first_only:
                        mp[(sv, s2)].append((tow, P2 - (2 * a / (a - 1)) * L1
                                             + (2 * a / (a - 1) - 1) * L2, c2))
                    gf[(sv, s1)].append((tow, L1 - L2))

    bysig = collections.defaultdict(list)
    bins = collections.defaultdict(list)
    for (sv, sig), pts in mp.items():
        t = np.array([p[0] for p in pts])
        y = np.array([p[1] for p in pts])
        cn = np.array([(p[2] or 0) for p in pts], dtype=float)
        brk = np.where((np.diff(t) > 5) | (np.abs(np.diff(y)) > SLIP_M))[0]
        for seg in np.split(np.arange(len(y)), brk + 1):
            if len(seg) < MIN_ARC:
                continue
            r = y[seg] - y[seg].mean()
            bysig[sig].append(np.std(r))
            for lo, hi in CN0_BINS:
                m = (cn[seg] >= lo) & (cn[seg] < hi)
                if m.sum() > 60:
                    bins[(lo, hi)].append(np.std(r[m]))

    scatter = []
    for _key, pts in gf.items():
        t = np.array([p[0] for p in pts])
        y = np.array([p[1] for p in pts])
        d, dt = np.diff(y), np.diff(t)
        ok = (dt == 1.0) & (np.abs(d) < 0.05)
        if ok.sum() > 500:
            scatter.append(np.std(d[ok]))
    # two signals differenced once: divide by 2 for a per-signal figure
    phase_mm = (np.median(scatter) / 2 * 1000) if scatter else float("nan")
    return bysig, bins, phase_mm, len(scatter)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--label", default=None)
    ap.add_argument("--signals", choices=("all", "l1"), default="all",
                    help="'l1' restricts to the first signal of each pair "
                         "(GPS-L1CA / GAL-E1C). The L2/E5 signals carry "
                         "markedly worse code multipath, so this choice moves "
                         "the answer -- state which you used.")
    ap.add_argument("--max-mb", type=float, default=12.0,
                    help="bytes to read; keeps a long log to a sane runtime")
    a = ap.parse_args()
    label = a.label or os.path.basename(a.path)

    ep, fmt = load_epochs(a.path, int(a.max_mb * 1e6))
    bysig, bins, phase_mm, n_ph = metrics(ep, first_only=(a.signals == "l1"))
    allv = [v for vs in bysig.values() for v in vs]

    print(f"== {label}   ({fmt}, {len(ep)} epochs, signals={a.signals})")
    if not allv:
        print("   no dual-frequency arcs long enough — need both signals of a pair")
        return
    for sig in sorted(bysig):
        v = np.array(bysig[sig])
        print(f"   {sig:10s} {len(v):3d} arcs   MP RMS {v.mean():.3f} m")
    print(f"   {'ALL':10s} {len(allv):3d} arcs   MP RMS {np.mean(allv):.3f} m   (unweighted — see mask note)")
    print("   by C/N0:  " + "   ".join(
        f"{lo}-{hi}: {np.mean(bins[(lo,hi)]):.3f}" for lo, hi in CN0_BINS if bins.get((lo, hi))))
    print(f"   phase noise {phase_mm:.2f} mm = {phase_mm*1e-3/C*1e12:.2f} ps   ({n_ph} arcs)")


if __name__ == "__main__":
    main()

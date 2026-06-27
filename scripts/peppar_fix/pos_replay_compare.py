#!/usr/bin/env python3
"""Offline comparison for a pos_replay capture — the first truth-ingest piece.

Scores our captured position trajectory against an external truth using the
**same** divergence monitor as ``pos_sim`` (the shared scoring layer promised
in docs/simulators-and-replay.md): far AND still growing → "no point
continuing".  So the synthetic sim and the real replay are graded identically.

Position truth is **static** — the receiver is fixed — so the position
comparison needs only the truth ARP + its σ; we score the sequence of
``[PPP_STATE]`` estimates (the per-epoch PPPFilter position + σ logged by the
engine under ``--ppp-state-log``) against it.  Two normalizations matter
(manifest §3): error vs the filter's *own* σ (the false-confidence signal) and,
optionally, error vs (own-σ ⊕ truth-σ).

The time-varying **ZTD** comparison vs NRCan's external ZTD(t) (manifest §5)
is also here: ``ztd_series_from_tro`` reads the CSRS-PPP ``.tro`` total ZTD,
``interpolate_ztd`` puts that ~5-min truth onto our 1 Hz timestamps, and
``compare_ztd`` removes the constant lag+apriori offset and scores the
time-varying departure with the same DivergenceMonitor.  Explicit total-ZTD
assembly (ZHD from ``[METAR]`` + mapping, to compare our *residual* wet ZTD
against the truth *total* without leaning on offset-removal) is the remaining
refinement.
"""
from __future__ import annotations

import argparse
import math
import os
import re
import sys
from dataclasses import dataclass
from typing import Iterable, Optional

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

# Matches the engine's [PPP_STATE] line anywhere in a log line (after the
# logger prefix).  Format (peppar_fix_engine, AntPosEstThread):
#   [PPP_STATE] epoch=N n=M ecef=X,Y,Z sigma_pos=Sm ztd=+Zm sigma_ztd=Sm
_PPP_RE = re.compile(
    r"\[PPP_STATE\]\s+(?:gps=(\S+)\s+)?epoch=(\d+)\s+n=(-?\d+)\s+"
    r"ecef=(-?[\d.eE+]+),(-?[\d.eE+]+),(-?[\d.eE+]+)\s+"
    r"sigma_pos=([\d.eE+-]+)m\s+ztd=([-+\d.eE]+)m\s+sigma_ztd=([\d.eE+-]+)m"
)

# The engine's [METAR] line (peppar_fix_engine _periodic_ztd_tie): Saastamoinen
# ZHD/ZWD/ZTD from surface weather, keyed by the same epoch= counter as
# [PPP_STATE].  The ZHD here is the hydrostatic anchor (manifest §5 trap 1);
# the ZTD is an independent weather-model total used as a cross-check.
_METAR_RE = re.compile(
    r"\[METAR\]\s+epoch=(\d+)\s+\S+\s+age=[\d.]+min\s+.*?"
    r"ZHD=([-+\d.eE]+)m\s+ZWD=([-+\d.eE]+)m\s+ZTD=([-+\d.eE]+)m"
)


@dataclass
class PppRow:
    epoch: int
    n_used: int
    ecef_m: tuple
    sigma_pos_m: float
    ztd_m: float          # residual wet ZTD (engine IDX_ZTD), metres
    sigma_ztd_m: float
    gps: Optional[object] = None   # datetime (GPS-time) if the log carried gps=


_ZTD_APRIORI_RE = re.compile(r"\[ZTD_APRIORI\]\s+m=([-+\d.eE]+)")


def parse_ztd_apriori(lines) -> Optional[float]:
    """The engine's logged ZTD apriori (``[ZTD_APRIORI] m=…``), or None if the
    log predates logZtdApriori.  Lets the total-ZTD assembly use the run's
    ACTUAL apriori instead of assuming the current ``ENGINE_ZTD_APRIORI_M`` — so
    a capture made before an apriori retune still assembles correctly."""
    for ln in lines:
        m = _ZTD_APRIORI_RE.search(ln)
        if m:
            try:
                return float(m.group(1))
            except ValueError:
                return None
    return None


@dataclass
class StaticTruth:
    """A fixed reference position with its own 1-σ (survey / PRIDE formal)."""
    ecef_m: tuple
    sigma_m: float = 0.0


def parse_ppp_state(lines: Iterable[str]) -> list:
    """Parse every ``[PPP_STATE]`` line from an engine log into PppRows."""
    rows = []
    for ln in lines:
        m = _PPP_RE.search(ln)
        if not m:
            continue
        gps = None
        if m.group(1):
            try:
                from datetime import datetime as _dt
                gps = _dt.fromisoformat(m.group(1))
            except ValueError:
                gps = None
        rows.append(PppRow(
            epoch=int(m.group(2)), n_used=int(m.group(3)),
            ecef_m=(float(m.group(4)), float(m.group(5)), float(m.group(6))),
            sigma_pos_m=float(m.group(7)),
            ztd_m=float(m.group(8)), sigma_ztd_m=float(m.group(9)), gps=gps))
    return rows


def compare_position(rows, truth: StaticTruth, *, k_sigma: float = 3.0,
                     window: int = 120, include_truth_sigma: bool = True) -> dict:
    """Score the position trajectory in ``rows`` against ``truth`` with the
    pos_sim DivergenceMonitor.

    Returns per-epoch error/σ, the final error, and the monitor verdict
    (fires only on confident-AND-wrong-AND-growing — Bob's "no point
    continuing" rule).
    """
    from peppar_fix.pos_sim import DivergenceMonitor
    mon = DivergenceMonitor(k_sigma=k_sigma, window=window)
    tx, ty, tz = truth.ecef_m
    errors, sigmas = [], []
    for r in rows:
        dx = r.ecef_m[0] - tx
        dy = r.ecef_m[1] - ty
        dz = r.ecef_m[2] - tz
        err = math.sqrt(dx * dx + dy * dy + dz * dz)
        sig = r.sigma_pos_m
        if include_truth_sigma and truth.sigma_m > 0:
            sig = math.sqrt(sig * sig + truth.sigma_m * truth.sigma_m)
        errors.append(err)
        sigmas.append(sig)
        # report the ACTUAL engine epoch (greppable in the log), not the row
        # index — detection is index-relative, so this is reporting-only.
        mon.update(r.epoch, err, sig)
    return {
        "n": len(rows),
        "window": window,
        "errors_m": errors,
        "sigmas_m": sigmas,
        "final_error_m": errors[-1] if errors else None,
        "final_sigma_m": sigmas[-1] if sigmas else None,
        "verdict": mon.verdict,
    }


@dataclass
class ZtdPoint:
    t_s: float            # seconds (GPS-time scale) — the alignment key
    ztd_m: float
    sigma_ztd_m: float


@dataclass
class MetarRow:
    epoch: int
    zhd_m: float          # hydrostatic (dry) delay from station pressure
    zwd_m: float          # wet delay from the weather model
    ztd_m: float          # ZHD + ZWD — the independent weather-model total


def parse_metar(lines: Iterable[str]) -> list:
    """Parse every ``[METAR]`` line into MetarRows (keyed by epoch)."""
    out = []
    for ln in lines:
        m = _METAR_RE.search(ln)
        if m:
            out.append(MetarRow(epoch=int(m.group(1)), zhd_m=float(m.group(2)),
                                zwd_m=float(m.group(3)), ztd_m=float(m.group(4))))
    return out


def ztd_total_series_from_ppp(rows, *, apriori_m=None) -> list:
    """Our filter's **total** zenith ZTD series = apriori + residual.

    The engine's ``[PPP_STATE] ztd=`` is the *residual* wet ZTD (IDX_ZTD),
    which rides on a fixed hydrostatic apriori (``ENGINE_ZTD_APRIORI_M``, 2.3 m)
    — so the filter's total zenith ZTD is ``apriori + residual`` (at zenith the
    wet mapping is 1, so no mapping enters a total-vs-total comparison —
    manifest §5 trap 2 is a slant-domain concern, not a zenith one).

    Assembling the *total* makes the constant 2.3 m apriori EXPLICIT instead of
    letting ``compare_ztd``'s offset-removal silently absorb it together with
    the real-time lag — which is what would otherwise hide a genuine constant
    bias (e.g. a wrong antenna height → wrong ZHD).  Only ``gps=``-keyed rows
    yield points (they need the time axis).
    """
    if apriori_m is None:
        from peppar_fix.saastamoinen import ENGINE_ZTD_APRIORI_M
        apriori_m = ENGINE_ZTD_APRIORI_M
    out = []
    for r in rows:
        if r.gps is None:
            continue
        out.append(ZtdPoint(t_s=r.gps.timestamp(), ztd_m=apriori_m + r.ztd_m,
                            sigma_ztd_m=r.sigma_ztd_m))
    return out


def ztd_series_from_ppp(rows) -> list:
    """Our time-keyed ZTD series from ``[PPP_STATE]`` rows that carried ``gps=``.

    This is the engine's *residual* wet ZTD (IDX_ZTD).  The lag-aware compare
    below absorbs the constant residual-vs-total apriori difference into its
    offset term, so this residual series is directly usable against an external
    *total* ZTD; explicit total-ZTD assembly (ZHD from [METAR] + mapping) is a
    later refinement.
    """
    out = []
    for r in rows:
        if r.gps is None:
            continue
        out.append(ZtdPoint(t_s=r.gps.timestamp(), ztd_m=r.ztd_m,
                            sigma_ztd_m=r.sigma_ztd_m))
    return out


def ztd_series_from_tro(path, *, site=None, time_offset_s: float = 0.0) -> list:
    """External *total* ZTD truth series from an NRCan CSRS-PPP ``.tro`` file."""
    from peppar_fix.nrcan_tro_reader import parse_tro
    return [ZtdPoint(t_s=p.t_s, ztd_m=p.ztd_m, sigma_ztd_m=p.sigma_ztd_m)
            for p in parse_tro(path, site=site, time_offset_s=time_offset_s)]


def interpolate_ztd(series, at_times_s) -> list:
    """Linearly interpolate a (sparse) ZTD ``series`` onto ``at_times_s``.

    NRCan ``.tro`` ZTD is ~5-min cadence; our ``[PPP_STATE]`` ZTD is 1 Hz.
    Nearest-within-tol alignment would use only our points near each 5-min
    mark and drop the rest (Charlie's #233 Minor 2).  Interpolating the
    truth onto our timestamps lets the **whole** series be compared, 1:1.

    Points outside the truth's covered span ``[t0, tN]`` are dropped (no
    extrapolation — a truth value we don't have isn't invented).  σ is
    interpolated linearly alongside the value.
    """
    if not series:
        return []
    s = sorted(series, key=lambda p: p.t_s)
    ts = [p.t_s for p in s]
    import bisect
    out = []
    for t in at_times_s:
        if t < ts[0] or t > ts[-1]:
            continue
        j = bisect.bisect_left(ts, t)
        if j < len(ts) and ts[j] == t:
            out.append(ZtdPoint(t_s=t, ztd_m=s[j].ztd_m,
                                sigma_ztd_m=s[j].sigma_ztd_m))
            continue
        a, b = s[j - 1], s[j]                 # bracketing samples (a.t < t < b.t)
        f = (t - a.t_s) / (b.t_s - a.t_s)
        out.append(ZtdPoint(
            t_s=t,
            ztd_m=a.ztd_m + f * (b.ztd_m - a.ztd_m),
            sigma_ztd_m=a.sigma_ztd_m + f * (b.sigma_ztd_m - a.sigma_ztd_m)))
    return out


def compare_ztd(our, truth, *, k_sigma: float = 3.0, window: int = 120,
                align_tol_s: float = 60.0, abs_bias_warn_m=None) -> dict:
    """Lag-aware comparison of two time-keyed ZTD series (manifest §5).

    NRCan ZTD is batch-smoothed and legitimately *leads* our real-time
    random-walk ZTD, so a constant offset is expected and NOT a fault — as is
    the constant residual-vs-total apriori difference.  We therefore remove the
    **median (our − truth) offset** (lag + apriori, both constant) and score
    the **detrended** residual: a *growing* detrended residual is a real
    time-varying departure the DivergenceMonitor flags; a steady offset is not.
    Align by nearest truth point within ``align_tol_s``.

    ``abs_bias_warn_m`` enables the **absolute-bias** check that total-ZTD
    assembly unlocks: when ``our`` is a *total* series (``ztd_total_series_from_ppp``),
    the removed offset is no longer a meaningless residual-vs-total mix but the
    physical ``our_total − truth_total`` median = real-time lag + any genuine
    *constant* bias (wrong antenna height → wrong ZHD, a mapping/apriori error).
    The detrended verdict structurally CANNOT see a constant bias (offset-removal
    eats it); this flags ``|offset| > abs_bias_warn_m`` so a real constant total
    bias surfaces instead of hiding inside the lag.  None disables (a raw
    *residual* series has a meaningless ~2.3 m offset, so don't warn on it).

    NOTE: ``align_tol_s`` only bites when ``truth`` is a *raw* (non-interpolated)
    series.  The CLI path runs ``interpolate_ztd`` first, putting truth on our
    exact timestamps, so every pair aligns with ``d=0`` and the tolerance is
    moot there — it exists for callers comparing two natively-sampled series.
    """
    import bisect
    import statistics
    from peppar_fix.pos_sim import DivergenceMonitor

    _empty = {"n_aligned": 0, "inconclusive": True, "offset_m": None,
              "abs_bias_warned": False,
              "verdict": {"fired": False, "fired_epoch": None}}
    if not our or not truth:
        return _empty
    ts = sorted(truth, key=lambda p: p.t_s)
    tt = [p.t_s for p in ts]
    pairs = []
    for p in our:
        j = bisect.bisect_left(tt, p.t_s)
        best = None
        for k in (j - 1, j):
            if 0 <= k < len(ts):
                d = abs(ts[k].t_s - p.t_s)
                if d <= align_tol_s and (best is None or d < best[0]):
                    best = (d, ts[k])
        if best is not None:
            pairs.append((p, best[1]))
    if not pairs:
        return _empty
    residuals = [op.ztd_m - tp.ztd_m for op, tp in pairs]
    offset = statistics.median(residuals)
    mon = DivergenceMonitor(k_sigma=k_sigma, window=window)
    detrended = []
    for (op, _tp), res in zip(pairs, residuals):
        d = abs(res - offset)
        detrended.append(d)
        # Report the aligned GPS-time (int unix seconds), not the pair index,
        # so verdict.fired_epoch is an honest, greppable timestamp — detection
        # is index-relative, so this is reporting-only (mirrors compare_position
        # passing the real engine epoch).
        mon.update(int(op.t_s), d, max(op.sigma_ztd_m, 1e-6))
    return {
        "n_aligned": len(pairs),
        "window": window,
        # < window aligned pairs can never fill the monitor's window, so the
        # verdict is necessarily "no fire" — surfaced as inconclusive rather
        # than an implicit pass (the exact false-confidence this tool exists to
        # catch).
        "inconclusive": len(pairs) < window,
        "offset_m": offset,
        # |offset| beyond the warn level is a real constant total bias the
        # detrended verdict can't catch (only meaningful for a *total* series —
        # None disables, since a residual series' offset is the ~2.3 m apriori).
        "abs_bias_warned": (abs_bias_warn_m is not None
                            and abs(offset) > abs_bias_warn_m),
        "detrended_m": detrended,
        "final_detrended_m": detrended[-1],
        "verdict": mon.verdict,
    }


def truth_from_pride(pos_path: str) -> StaticTruth:
    """Static truth from a PRIDE PPP-AR ``.pos`` file (reuses pride_pos_reader)."""
    from peppar_fix.pride_pos_reader import parse_pos
    sol = parse_pos(pos_path)
    return StaticTruth(ecef_m=tuple(sol.ecef_m), sigma_m=sol.sigma_3d_m())


def truth_from_ecef(ecef_str: str, sigma_m: float = 0.0) -> StaticTruth:
    """Static truth from an explicit 'X,Y,Z' ECEF (e.g. an antennas.json ARP)."""
    x, y, z = (float(v) for v in ecef_str.split(","))
    return StaticTruth(ecef_m=(x, y, z), sigma_m=float(sigma_m))


def _print_position(rows, args):
    truth = (truth_from_pride(args.pride) if args.pride
             else truth_from_ecef(args.ecef, args.truth_sigma_m))
    res = compare_position(rows, truth, k_sigma=args.k_sigma, window=args.window)
    v = res["verdict"]
    print(f"[PPP_STATE] rows: {res['n']}")
    if res["final_error_m"] is not None:
        _fs = res["final_sigma_m"] or 0.0
        _ratio = f"{res['final_error_m'] / _fs:.2f}" if _fs > 0 else "n/a (σ=0)"
        print(f"position final error: {res['final_error_m']:.3f} m  "
              f"σ: {res['final_sigma_m']:.3f} m  err/σ: {_ratio}")
    if res["n"] < res["window"]:
        # the monitor needs a full window before it can fire — fewer rows than
        # that always reads "in corridor", which would be false reassurance.
        print(f"position verdict: INCONCLUSIVE — {res['n']} rows < "
              f"window={res['window']} (too short for the monitor to fire)")
    else:
        print("position verdict: " + (
            f"DIVERGED @ epoch {v['fired_epoch']} "
            "(confident, wrong, and growing — no point continuing)"
            if v["fired"] else "stayed in corridor"))


def _print_ztd(rows, metar_rows, args):
    # Total assembly (default): our total = apriori + residual, so the removed
    # offset is the physical our_total − truth_total (lag + any real bias), and
    # the absolute-bias check below can fire.  --ztd-residual compares the raw
    # residual instead (offset then ~2.3 m apriori, abs-bias disabled).
    if args.ztd_residual:
        our = ztd_series_from_ppp(rows)
        kind, warn = "residual", None
    else:
        our = ztd_total_series_from_ppp(rows, apriori_m=args.ztd_apriori_m)
        kind, warn = "total", args.ztd_abs_bias_warn_m
    if not our:
        print("ZTD: no gps=-keyed [PPP_STATE] rows (need a recent engine "
              "build); skipping ZTD comparison")
        return
    truth_raw = ztd_series_from_tro(args.tro, site=args.tro_site,
                                    time_offset_s=args.tro_time_offset_s)
    # Interpolate the sparse (~5-min) truth onto our 1 Hz timestamps so the
    # whole series compares 1:1 (Charlie #233 Minor 2), then a tight tol.
    truth = interpolate_ztd(truth_raw, [p.t_s for p in our])
    res = compare_ztd(our, truth, k_sigma=args.k_sigma, window=args.window,
                      align_tol_s=1.0, abs_bias_warn_m=warn)
    print(f"\nZTD ({kind}): our {len(our)} pts, truth {len(truth_raw)} pts → "
          f"{res['n_aligned']} aligned")
    if res["offset_m"] is not None:
        _lbl = ("absolute bias (lag + any height/apriori error)"
                if kind == "total" else "offset removed (lag+apriori)")
        print(f"ZTD {_lbl}: {res['offset_m'] * 1e3:+.1f} mm"
              f"   final detrended: {res['final_detrended_m'] * 1e3:.1f} mm")
        if res.get("abs_bias_warned"):
            print(f"ZTD WARNING: |absolute bias| > {warn * 1e3:.0f} mm — a real "
                  "CONSTANT total bias (height/apriori/mapping?), invisible to "
                  "the detrended verdict")
    # Independent weather-model cross-check (manifest §5 trap 1): the [METAR]
    # Saastamoinen total vs NRCan, report-only, validates the truth's ballpark.
    if kind == "total" and metar_rows:
        _print_metar_crosscheck(rows, metar_rows, truth_raw, args)
    if res.get("inconclusive"):
        print(f"ZTD verdict: INCONCLUSIVE — {res['n_aligned']} aligned < "
              f"window={args.window}")
    else:
        zv = res["verdict"]
        print("ZTD verdict: " + (
            f"DIVERGED @ gps-time {zv['fired_epoch']} "
            "(time-varying departure growing — ZTD misallocation)"
            if zv["fired"] else "stayed in corridor"))


def _print_metar_crosscheck(rows, metar_rows, truth_raw, args):
    """[METAR] weather-model total vs NRCan truth — an independent sanity check
    on the truth's absolute level (NOT the filter's ZTD, which is what the main
    comparison scores).  Joins [METAR] to [PPP_STATE] by epoch for a GPS time."""
    gps_by_epoch = {r.epoch: r.gps for r in rows if r.gps is not None}
    wx = [ZtdPoint(t_s=gps_by_epoch[m.epoch].timestamp(), ztd_m=m.ztd_m,
                   sigma_ztd_m=0.0)
          for m in metar_rows if m.epoch in gps_by_epoch]
    if not wx:
        return
    truth = interpolate_ztd(truth_raw, [p.t_s for p in wx])
    if not truth:
        return
    import statistics
    # interpolate_ztd stores t_s as the EXACT wx time it was interpolated at, so
    # key on the exact float — no round-to-second (which would silently drop a
    # point whose sub-second t_s straddles an integer boundary).  Out-of-span wx
    # points simply aren't in `by_t`.
    by_t = {p.t_s: p.ztd_m for p in truth}
    diffs = [w.ztd_m - by_t[w.t_s] for w in wx if w.t_s in by_t]
    if diffs:
        print(f"ZTD [METAR] cross-check: weather-model total − NRCan median "
              f"{statistics.median(diffs) * 1e3:+.1f} mm over {len(diffs)} pts "
              "(independent of the filter)")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("log", help="engine run.log containing [PPP_STATE] lines")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--pride", help="PRIDE .pos file → static position truth")
    g.add_argument("--ecef", help="explicit 'X,Y,Z' ECEF truth (m)")
    ap.add_argument("--truth-sigma-m", type=float, default=0.0)
    ap.add_argument("--tro", help="NRCan CSRS-PPP .tro file → external ZTD truth")
    ap.add_argument("--tro-site", help="station code if the .tro has >1 site")
    ap.add_argument("--tro-time-offset-s", type=float, default=0.0,
                    help="add to .tro epochs (e.g. a known GPS−UTC offset)")
    ap.add_argument("--ztd-residual", action="store_true",
                    help="compare our raw residual ZTD instead of assembling "
                         "the total (apriori+residual); disables abs-bias check")
    ap.add_argument("--ztd-apriori-m", type=float, default=None,
                    help="ZTD apriori for total assembly (default: engine's 2.3 m)")
    ap.add_argument("--ztd-abs-bias-warn-m", type=float, default=0.10,
                    help="warn if |our_total − truth_total| median exceeds this "
                         "(a constant bias the detrended verdict can't see)")
    ap.add_argument("--k-sigma", type=float, default=3.0)
    ap.add_argument("--window", type=int, default=120)
    args = ap.parse_args()

    if not (args.pride or args.ecef or args.tro):
        ap.error("need a truth source: --pride/--ecef (position) and/or --tro (ZTD)")

    with open(args.log) as f:
        lines = f.readlines()
    rows = parse_ppp_state(lines)
    metar_rows = parse_metar(lines)
    # logZtdApriori: prefer an explicit --ztd-apriori-m, else the run's logged
    # [ZTD_APRIORI] (the actual apriori the capture used), else the constant.
    if args.ztd_apriori_m is None:
        args.ztd_apriori_m = parse_ztd_apriori(lines)
    if args.pride or args.ecef:
        _print_position(rows, args)
    if args.tro:
        _print_ztd(rows, metar_rows, args)


if __name__ == "__main__":
    main()

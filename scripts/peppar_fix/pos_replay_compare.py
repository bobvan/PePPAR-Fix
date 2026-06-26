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

The time-varying **ZTD** comparison vs NRCan's external ZTD(t) is the next
increment (it needs a per-epoch GPS-time key the ``[PPP_STATE]`` line doesn't
yet carry, and the §5 ZHD/mapping/height/lag conventions).
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


@dataclass
class PppRow:
    epoch: int
    n_used: int
    ecef_m: tuple
    sigma_pos_m: float
    ztd_m: float          # residual wet ZTD (engine IDX_ZTD), metres
    sigma_ztd_m: float
    gps: Optional[object] = None   # datetime (GPS-time) if the log carried gps=


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


def compare_ztd(our, truth, *, k_sigma: float = 3.0, window: int = 120,
                align_tol_s: float = 60.0) -> dict:
    """Lag-aware comparison of two time-keyed ZTD series (manifest §5).

    NRCan ZTD is batch-smoothed and legitimately *leads* our real-time
    random-walk ZTD, so a constant offset is expected and NOT a fault — as is
    the constant residual-vs-total apriori difference.  We therefore remove the
    **median (our − truth) offset** (lag + apriori, both constant) and score
    the **detrended** residual: a *growing* detrended residual is a real
    time-varying departure the DivergenceMonitor flags; a steady offset is not.
    Align by nearest truth point within ``align_tol_s``.
    """
    import bisect
    import statistics
    from peppar_fix.pos_sim import DivergenceMonitor

    _empty = {"n_aligned": 0, "offset_m": None,
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
    for i, ((op, _tp), res) in enumerate(zip(pairs, residuals)):
        d = abs(res - offset)
        detrended.append(d)
        mon.update(i, d, max(op.sigma_ztd_m, 1e-6))
    return {
        "n_aligned": len(pairs),
        "window": window,
        "offset_m": offset,
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


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("log", help="engine run.log containing [PPP_STATE] lines")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--pride", help="PRIDE .pos file → static position truth")
    g.add_argument("--ecef", help="explicit 'X,Y,Z' ECEF truth (m)")
    ap.add_argument("--truth-sigma-m", type=float, default=0.0)
    ap.add_argument("--k-sigma", type=float, default=3.0)
    ap.add_argument("--window", type=int, default=120)
    args = ap.parse_args()

    with open(args.log) as f:
        rows = parse_ppp_state(f)
    truth = (truth_from_pride(args.pride) if args.pride
             else truth_from_ecef(args.ecef, args.truth_sigma_m))
    res = compare_position(rows, truth, k_sigma=args.k_sigma, window=args.window)
    v = res["verdict"]
    print(f"[PPP_STATE] rows: {res['n']}")
    if res["final_error_m"] is not None:
        _fs = res["final_sigma_m"] or 0.0
        _ratio = f"{res['final_error_m'] / _fs:.2f}" if _fs > 0 else "n/a (σ=0)"
        print(f"final error: {res['final_error_m']:.3f} m  "
              f"σ: {res['final_sigma_m']:.3f} m  err/σ: {_ratio}")
    if res["n"] < res["window"]:
        # the monitor needs a full window before it can fire — fewer rows than
        # that always reads "in corridor", which would be false reassurance.
        print(f"verdict: INCONCLUSIVE — {res['n']} rows < window={res['window']}"
              " (too short for the divergence monitor to fire)")
    else:
        print("verdict: " + (
            f"DIVERGED @ epoch {v['fired_epoch']} "
            "(confident, wrong, and growing — no point continuing)"
            if v["fired"] else "stayed in corridor"))


if __name__ == "__main__":
    main()

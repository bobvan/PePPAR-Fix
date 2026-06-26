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
    r"\[PPP_STATE\]\s+epoch=(\d+)\s+n=(-?\d+)\s+"
    r"ecef=(-?[\d.eE+]+),(-?[\d.eE+]+),(-?[\d.eE+]+)\s+"
    r"sigma_pos=([\d.eE+-]+)m\s+ztd=([-+\d.eE]+)m\s+sigma_ztd=([\d.eE+-]+)m"
)


@dataclass
class PppRow:
    epoch: int
    n_used: int
    ecef_m: tuple
    sigma_pos_m: float
    ztd_m: float
    sigma_ztd_m: float


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
        rows.append(PppRow(
            epoch=int(m.group(1)), n_used=int(m.group(2)),
            ecef_m=(float(m.group(3)), float(m.group(4)), float(m.group(5))),
            sigma_pos_m=float(m.group(6)),
            ztd_m=float(m.group(7)), sigma_ztd_m=float(m.group(8))))
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

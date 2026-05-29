#!/usr/bin/env python3
"""CLI for the closed-loop DO/GNSS servo simulator.

Deterministic test bed for servo / gate / fusion design — drives the
REAL peppar_fix.do_freq_est.DOFreqEst + OcxoTrustedGate in a closed loop
(see peppar_fix/servo_sim.py and docs/closed-loop-servo-sim.md).

Examples:
    # One run from a faithfulness preset, print TDEV table + lock status.
    scripts/servo_sim.py --preset clkpoc3

    # Sweep the coast interval (the longTauGnssCoupling test bed).
    scripts/servo_sim.py --preset piface-ungated --sweep coast_interval_s=1,30,60,100

    # Sweep the OCXO gate K factor.
    scripts/servo_sim.py --preset piface-v1 --sweep gate_k_sigma=5,10,20,50

    # Two-clock cross-host differential (the 1 ns shared-antenna bound).
    scripts/servo_sim.py --two-clock clkpoc3 piface-v1

    # Reproduce the whole faithfulness bar.
    scripts/servo_sim.py --faithfulness

    # Dump the per-epoch log and a TDEV plot.
    scripts/servo_sim.py --preset clkpoc3 --out /tmp/run.csv --plot /tmp/tdev.png
"""
from __future__ import annotations

import argparse
import csv
import logging
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from peppar_fix.servo_sim import (  # noqa: E402
    ClosedLoopSim, SimConfig, preset, run_two_clock, _DEFAULT_TAUS)

_SUMMARY_TAUS = (1, 4, 16, 64, 128, 256, 1024)


def _coerce(v: str):
    """Coerce a CLI override value to bool / float / int / str."""
    if v.lower() in ("true", "false"):
        return v.lower() == "true"
    if v.lower() in ("none", "null"):
        return None
    try:
        f = float(v)
        return int(f) if f.is_integer() and "." not in v and "e" not in v.lower() else f
    except ValueError:
        return v


def _build_cfg(args) -> SimConfig:
    if args.preset:
        cfg = preset(args.preset)
    else:
        cfg = SimConfig()
    cfg.duration_s = args.duration
    if args.coast is not None:
        cfg.coast_interval_s = args.coast
    if args.seed is not None:
        cfg.seed = args.seed
    for kv in args.set or []:
        k, _, v = kv.partition("=")
        if not hasattr(cfg, k):
            raise SystemExit(f"unknown config field: {k}")
        setattr(cfg, k, _coerce(v))
    return cfg


def _tdev_row(res, taus=_SUMMARY_TAUS) -> str:
    tt, td = res.tdev(taus=list(taus))
    return "  ".join(f"{int(t)}s={v*1e3:.0f}ps" for t, v in zip(tt, td))


def _summary(res, cfg) -> None:
    sim = ClosedLoopSim(cfg)
    fr = sim.freerun_tdev_1s()
    state = ("DIVERGED" if res.diverged() else
             ("LOCKED" if res.locked() else "NOT-LOCKED"))
    print(f"[{res.label}] {state}")
    print(f"  freerun TDEV(1s) = {fr*1e3:.1f} ps   (DO floor)")
    print(f"  output  TDEV     = {_tdev_row(res)}")
    print(f"  max excursion (post-300s) = {res.max_excursion_ns():.2f} ns")
    if res.gate_stats:
        g = res.gate_stats
        print(f"  OCXO gate: rejected={g.get('n_rejected')} "
              f"accepted={g.get('n_accepted')} "
              f"pre_age={g.get('n_skipped_pre_age')} "
              f"(σ={g.get('sigma_short_tau_ns')}ns K={g.get('k_sigma')})")
    inn = res.ticc_innov_ns[~np.isnan(res.ticc_innov_ns)]
    if len(inn):
        print(f"  TICC innov: median|.|={np.median(np.abs(inn)):.2f}ns "
              f"std={inn.std():.2f}ns")


def _write_csv(res, path: str) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "phi_do_true_ns", "phi_rx_true_ns", "adjfine_ppb",
                    "est_phi_do_ns", "est_f_do_ppb", "ticc_innov_ns",
                    "gate_rejected"])
        for i in range(len(res.t_s)):
            w.writerow([f"{res.t_s[i]:.3f}", f"{res.phi_do_true_ns[i]:.6f}",
                        f"{res.phi_rx_true_ns[i]:.6f}", f"{res.adjfine_ppb[i]:.6f}",
                        f"{res.est_phi_do_ns[i]:.6f}", f"{res.est_f_do_ppb[i]:.6f}",
                        "" if np.isnan(res.ticc_innov_ns[i]) else f"{res.ticc_innov_ns[i]:.6f}",
                        int(res.gate_rejected[i])])
    print(f"wrote {path}  ({len(res.t_s)} epochs)")


def _plot_tdev(results, path: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 6))
    for res in results:
        tt, td = res.tdev(taus=list(_DEFAULT_TAUS))
        if len(tt):
            ax.loglog(tt, td * 1e9, "o-", label=res.label)
    ax.set_xlabel("τ (s)")
    ax.set_ylabel("TDEV (ns)")
    ax.set_title("Closed-loop sim — output TDEV (chA-equivalent)")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"wrote {path}")


def _do_sweep(args, base_cfg) -> list:
    field, _, vals = args.sweep.partition("=")
    if not hasattr(base_cfg, field):
        raise SystemExit(f"unknown sweep field: {field}")
    results = []
    print(f"=== sweep {field} ∈ {{{vals}}} ===")
    for raw in vals.split(","):
        cfg = _build_cfg(args)
        setattr(cfg, field, _coerce(raw))
        cfg.label = f"{base_cfg.label}:{field}={raw}"
        res = ClosedLoopSim(cfg).run()
        state = ("DIVERGED" if res.diverged() else
                 ("lock" if res.locked() else "NO-LOCK"))
        print(f"  {field}={raw:>8}  {state:9s}  TDEV: {_tdev_row(res)}  "
              f"maxexc={res.max_excursion_ns():.2f}ns")
        results.append(res)
    return results


def _do_two_clock(args) -> list:
    name_a, name_b = args.two_clock
    cfg_a = preset(name_a, duration_s=args.duration)
    cfg_b = preset(name_b, duration_s=args.duration)
    res_a, res_b, diff = run_two_clock(cfg_a, cfg_b, share_gnss=not args.independent_gnss)
    skip = int(300 / cfg_a.dt_s)
    d = diff[skip:]
    p95 = float(np.percentile(np.abs(d), 95))
    print(f"=== two-clock differential: {name_a} vs {name_b} "
          f"({'shared' if not args.independent_gnss else 'independent'} GNSS) ===")
    print(f"  {name_a}: {'LOCKED' if res_a.locked() else 'NOT-LOCKED'}  "
          f"TDEV(1s)={res_a.tdev_1s()*1e3:.0f}ps")
    print(f"  {name_b}: {'LOCKED' if res_b.locked() else 'NOT-LOCKED'}  "
          f"TDEV(1s)={res_b.tdev_1s()*1e3:.0f}ps")
    print(f"  |Δ| (post-300s):  max={np.max(np.abs(d)):.2f}ns  "
          f"95%={p95:.2f}ns  RMS={np.sqrt(np.mean(d**2)):.2f}ns")
    print(f"  CLAUDE.md shared-antenna bound: |Δ|≤1ns @95%  →  "
          f"{'PASS' if p95 <= 1.0 else 'FAIL'}")
    print("  NOTE: run_two_clock is NOT yet faithful (v1 shares the whole "
          "RNG) — this |Δ| is a plumbing smoke test, not the bound (see docs).")
    return [res_a, res_b]


def _do_faithfulness(args) -> None:
    """Reproduce the dayplan faithfulness bar and print a verdict table."""
    bar = [
        ("clkpoc3",        "no-gate, locks ~104ps",      "lock",   (60, 200)),
        ("clkpoc3-gated",  "gated, never-locks ~1.74ns", "nolock", None),
        ("piface-ungated", "no-gate ~565ps",             "lock",   (150, 900)),
        ("piface-v1",      "v1 gate, locks ~74ps",       "lock",   (40, 200)),
    ]
    print("=== faithfulness bar (per dayplan closedLoopServoSim) ===")
    print(f"{'preset':16s} {'TDEV(1s)':>10s} {'state':>10s}  expectation")
    for name, expect, want, _band in bar:
        cfg = preset(name, duration_s=args.duration)
        res = ClosedLoopSim(cfg).run()
        t1 = res.tdev_1s() * 1e3
        state = ("DIVERGED" if res.diverged() else
                 ("LOCKED" if res.locked() else "NOT-LOCKED"))
        print(f"{name:16s} {t1:8.0f}ps {state:>10s}  {expect}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--preset", help="faithfulness preset name "
                    "(clkpoc3, clkpoc3-gated, piface-ungated, piface-v1)")
    ap.add_argument("--duration", type=float, default=4000.0, help="seconds")
    ap.add_argument("--coast", type=float, default=None,
                    help="coast interval (s); DO free-runs between corrections")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--set", action="append", metavar="field=value",
                    help="override any SimConfig field (repeatable)")
    ap.add_argument("--sweep", metavar="field=v1,v2,...",
                    help="sweep one SimConfig field over comma-separated values")
    ap.add_argument("--two-clock", nargs=2, metavar=("PRESET_A", "PRESET_B"),
                    help="cross-host differential of two presets sharing one GNSS")
    ap.add_argument("--independent-gnss", action="store_true",
                    help="two-clock: separate-antenna (independent GNSS realizations)")
    ap.add_argument("--faithfulness", action="store_true",
                    help="reproduce the dayplan faithfulness bar and print verdicts")
    ap.add_argument("--out", help="write per-epoch CSV log")
    ap.add_argument("--plot", help="write a TDEV loglog PNG")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="show EKF gate log lines (noisy)")
    args = ap.parse_args(argv)

    if not args.verbose:
        logging.disable(logging.CRITICAL)

    results = []
    if args.faithfulness:
        _do_faithfulness(args)
    elif args.two_clock:
        results = _do_two_clock(args)
    elif args.sweep:
        base = _build_cfg(args)
        results = _do_sweep(args, base)
    else:
        cfg = _build_cfg(args)
        res = ClosedLoopSim(cfg).run()
        results = [res]
        _summary(res, cfg)

    if args.out and results:
        _write_csv(results[0], args.out)
    if args.plot and results:
        _plot_tdev(results, args.plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())

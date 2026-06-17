#!/usr/bin/env python3
"""Coast-cap A/B at the honest char-derived Q (bravo PR #83 follow-up).

Background: the test_coast_before_freq_convergence_diverges test was
captured at sigma_do_freq_ppb=0.01 (engine default) with coast_interval_s
above 60s, and showed a diverge↔locked flip at the edge of stability.
PR #83 (qFromCharPerActuator) now derives Q[3,3] = sigma_do_freq² from
the DO's measured ADEV via sigma_do_freq_ppb = √3·ADEV(τ)·1e9/√τ
(do_state._sigma_do_freq_ppb_from_adev).  For OCXO-class hosts at the
long-τ tail this lands around 0.0014–0.0017 ppb — substantially TIGHTER
than 0.01.

This A/B asks whether the >60s coast (the previously-diverging regime)
is now safe with:
  (a) the honest Q on the filter,
  (b) the matching truth-side RWFM `do_rwfm_ppb_per_sqrt_s` on the plant
      (so the truth's RWFM strength matches what Q claims to model),
  (c) the coast-cap (PR #86) enabled with t_budget_ns=5.

Regimes:
  - loose-Q (sigma_do_freq=0.01)            ← the diverge-baseline
  - honest-Q (sigma_do_freq=0.0014)         ← char-derived from
                                              clkPoC3 ADEV(100s)≈8e-12
Across coast intervals: 1s, 30s, 60s, 120s.

Output: which combos diverge, locked TDEV(1s), and the coast-cap's
actual effective interval (since the cap may shorten the requested
coast).
"""
from __future__ import annotations

import math
import sys

import numpy as np

from peppar_fix.servo_sim import ClosedLoopSim, preset


def adev_to_sigma_do_freq_ppb(adev: float, tau: float) -> float:
    """√3 · ADEV(τ)·1e9 / √τ  — same as do_state._sigma_do_freq_ppb_from_adev."""
    return math.sqrt(3.0) * adev * 1e9 / math.sqrt(tau)


HONEST_Q_PPB = adev_to_sigma_do_freq_ppb(adev=8e-12, tau=100.0)   # ≈ 0.00139 ppb
LOOSE_Q_PPB = 0.01                                                # the diverge baseline


def run_cell(*, q_ppb: float, coast_interval_s: float, with_cap: bool,
             duration_s: float = 4000.0):
    cfg = preset("piface-ungated", duration_s=duration_s,
                 coast_interval_s=coast_interval_s,
                 sigma_do_freq_ppb=q_ppb,
                 do_rwfm_ppb_per_sqrt_s=q_ppb,
                 coast_cap_enabled=with_cap,
                 coast_cap_t_budget_ns=5.0)
    res = ClosedLoopSim(cfg).run()
    return res


def fmt(x: float, fmt_str: str = "{:.3f}") -> str:
    return "       —" if not np.isfinite(x) else fmt_str.format(x)


def summarize(res) -> dict:
    inn = res.ticc_innov_ns[~np.isnan(res.ticc_innov_ns)]
    return {
        "locked": res.locked(),
        "diverged": res.diverged(),
        "tdev_1s_ns": res.tdev_1s(),
        "max_excursion_ns": res.max_excursion_ns(),
        "innov_rms_ns": float(np.sqrt((inn**2).mean())) if len(inn) else float("nan"),
    }


def main() -> int:
    rows = []
    for q_label, q in (("loose-Q (0.01)", LOOSE_Q_PPB),
                       ("honest-Q (char)", HONEST_Q_PPB)):
        for coast in (1.0, 30.0, 60.0, 120.0):
            for cap_label, cap in (("cap-off", False), ("cap-on", True)):
                res = run_cell(q_ppb=q, coast_interval_s=coast, with_cap=cap)
                s = summarize(res)
                rows.append((q_label, q, coast, cap_label,
                             s["locked"], s["diverged"],
                             s["tdev_1s_ns"], s["max_excursion_ns"],
                             s["innov_rms_ns"]))

    print()
    print(f"honest Q (char-derived from ADEV(100s)≈8e-12): {HONEST_Q_PPB:.5f} ppb")
    print(f"loose  Q (old default):                        {LOOSE_Q_PPB:.5f} ppb")
    print()
    header = f"{'Q regime':18s} {'coast(s)':>9s} {'cap':>8s}  " \
             f"{'locked':>7s}  {'diverged':>9s}  {'TDEV(1s)ns':>11s}  " \
             f"{'max|Δ|ns':>10s}  {'innov RMS ns':>14s}"
    print(header)
    print("-" * len(header))
    for q_label, _q, coast, cap, locked, diverged, tdev, mx, inn in rows:
        print(f"{q_label:18s} {coast:>9.0f} {cap:>8s}  "
              f"{str(locked):>7s}  {str(diverged):>9s}  "
              f"{fmt(tdev):>11s}  {fmt(mx):>10s}  {fmt(inn):>14s}")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())

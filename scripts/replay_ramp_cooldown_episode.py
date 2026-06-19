#!/usr/bin/env python3
"""Replay the pifaceDriftEpisodes cascade (2026-06-18 22:16 UTC) through the
real `_servo_outlier_decision` + `ResetBudget` to show rampReacquireCooldown
converts the 6-resets-in-30s → exit-5 cascade into a SINGLE re-acquire.

Trace source: run-cap.log per-epoch "Outlier:"/"re-acquire" pps_err lines —
a clean monotonic same-sign ramp +511 → +565 ns over 35 one-Hz epochs (the
DO discontinuity the held-actuator reset never corrected).

This is a WORST-CASE replay: it feeds the *recorded* open-loop error sequence
unchanged to the NEW logic too.  In reality the cooldown lets each observable
through to the wide-P EKF, so the actuator (held frozen in the recorded run)
would actively pull the error back — the real NEW trajectory is strictly
better than this replay shows.  Even under that pessimistic assumption the
cooldown removes the cascade.
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
from peppar_fix_engine import _servo_outlier_decision, _RAMP_COOLDOWN_S  # noqa: E402
from peppar_fix.reset_budget import ResetBudget  # noqa: E402

# pps_err_ticc_ns at 1 Hz, t=0..34 s (from run-cap.log 17:16:19→17:16:53 CDT).
ERR_NS = [
    511, 513, 515, 516, 518, 520, 522, 524, 526, 527, 530, 531, 533, 535, 537,
    538, 540, 542, 544, 546, 548, 549, 551, 553, 555, 556, 558, 560, 562, 556,
    558, 560, 562, 563, 565,
]
TRACK_OUTLIER_NS = 200.0   # below the whole ramp → every epoch is an outlier,
                           # reproducing the recorded run (all 35 logged as such)
RAMP_ACCEPT_N = 5


def run(cooldown: bool):
    ctx = {'consecutive_outliers': 0}
    budget = ResetBudget(max_resets=6, window_s=300.0, enabled=True)
    resets, reset_ts, exit5_at = 0, [], None
    for t, err in enumerate(ERR_NS):
        mono = float(t) if cooldown else None
        decision, _ = _servo_outlier_decision(
            ctx, outlier_observable_ns=float(err),
            track_outlier_ns=TRACK_OUTLIER_NS, converging=False,
            gap_recovery=False, ramp_accept_n=RAMP_ACCEPT_N, mono_time=mono)
        if decision == "ramp_accept":
            if budget.request('ramp_reacquire', now=float(t)):
                resets += 1
                reset_ts.append(t)
                ctx['consecutive_outliers'] = 0
                if cooldown:
                    ctx['_ramp_cooldown_until'] = float(t) + _RAMP_COOLDOWN_S
            else:
                exit5_at = t
                break
    return resets, reset_ts, exit5_at


def main():
    print(f"Episode: {len(ERR_NS)} epochs, err {ERR_NS[0]}→{ERR_NS[-1]} ns "
          f"(monotonic same-sign ramp), track_outlier={TRACK_OUTLIER_NS} ns, "
          f"ramp_accept_n={RAMP_ACCEPT_N}, cooldown={_RAMP_COOLDOWN_S} s\n")
    for label, cd in (("OLD (no cooldown)", False),
                      ("NEW (rampReacquireCooldown)", True)):
        resets, ts, exit5 = run(cd)
        verdict = (f"EXIT-5 cascade at t={exit5}s (budget exhausted)"
                   if exit5 is not None else "no exit-5 — cascade broken")
        print(f"{label:32s}: {resets} re-acquire(s) at t={ts}s → {verdict}")

    # Assertions encode the expected outcome so this doubles as a check.
    old_r, _, old_e5 = run(False)
    new_r, _, new_e5 = run(True)
    assert old_e5 is not None, "OLD logic should reproduce the exit-5 cascade"
    assert new_e5 is None, "NEW logic must NOT reach exit-5 in the episode"
    assert new_r == 1, f"NEW logic should re-acquire exactly once, got {new_r}"
    print("\nPASS: OLD reproduces the recorded 6-reset→exit-5 cascade; "
          "NEW converts it to a single re-acquire with no exit-5.")


if __name__ == "__main__":
    main()

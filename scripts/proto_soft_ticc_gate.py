#!/usr/bin/env python3
"""softGateMidTau (I-092034) servo_sim go/no-go: soft vs hard TICC chi² gate.

Closed-loop ClosedLoopSim A/B on the over-budget OCXO hosts (clkpoc3,
piface-ungated).  Each run injects the realistic ~2.3 ns GNSS-PPS-jitter
tick-crossing heavy tails (the innovations the hard gate rejects) PLUS a few
induced measurement-drop windows (coast → re-acquire → the hard-gate
blackout+snap).  Metric = chA-equivalent TDEV (detrended DO output).

GO criterion (Main's PHASE-1 acceptance): soft gate reduces mid-τ
(100–1000 s) chA-TDEV vs the hard gate, WITHOUT regressing short-τ (≤2 s).
The hard gate's mid-τ bulge is the open-loop coast during the gate blackout
+ the snap when it reopens; the soft gate admits every sample down-weighted,
so there is no blackout and no snap.

Averaged over several seeds for stability.  Pure-Python EKF; runs in a few
seconds per arm.  Not a replacement for the lab A/B — a fast pre-flight.
"""
import logging
import sys
from pathlib import Path

import numpy as np

# Quiet the per-epoch hard-gate "update skipped" WARNINGs and soft-gate INFO
# lines — the proto's signal is the TDEV table, not the gate chatter.
logging.disable(logging.WARNING)

sys.path.insert(0, str(Path(__file__).resolve().parent / "peppar_fix"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from peppar_fix.servo_sim import preset, ClosedLoopSim  # noqa: E402

SHORT_TAUS = [1.0, 2.0]
MID_TAUS = [100.0, 200.0, 500.0, 1000.0]
ALL_TAUS = SHORT_TAUS + MID_TAUS

HOSTS = ["clkpoc3", "piface-ungated"]
SEEDS = [0, 1, 2, 3]
DURATION_S = 8000.0
# A few coast→reacquire windows well after convergence — each forces the
# hard gate into an open-loop blackout + snap that the soft gate avoids.
DROP_WINDOWS = [(2000.0, 2080.0), (4000.0, 4090.0), (6000.0, 6100.0)]


def _avg_tdev(host, soft, taus):
    """Mean chA-equivalent TDEV (ns) across seeds, per tau."""
    acc = np.zeros(len(taus))
    for sd in SEEDS:
        cfg = preset(host, seed=sd, duration_s=DURATION_S,
                     soft_ticc_gate=soft,
                     induced_drop_windows=list(DROP_WINDOWS))
        res = ClosedLoopSim(cfg).run()
        _, td = res.tdev(taus=taus)
        acc += np.asarray(td, dtype=float)
    return acc / len(SEEDS)


def main():
    print("softGateMidTau go/no-go — soft vs hard TICC chi² gate "
          f"({len(SEEDS)} seeds, {DURATION_S:.0f}s, "
          f"{len(DROP_WINDOWS)} coast/reacquire windows)\n")
    overall_go = True
    for host in HOSTS:
        hard = _avg_tdev(host, soft=False, taus=ALL_TAUS)
        soft = _avg_tdev(host, soft=True, taus=ALL_TAUS)
        print(f"=== {host} — chA-equivalent TDEV (ps), mean over seeds ===")
        print(f"  {'tau_s':>6s} | {'hard':>9s} | {'soft':>9s} | {'soft/hard':>9s}")
        print("  " + "-" * 44)
        ratios = {}
        for i, t in enumerate(ALL_TAUS):
            r = soft[i] / hard[i] if hard[i] else float("nan")
            ratios[t] = r
            tag = "  short" if t in SHORT_TAUS else "  mid"
            print(f"  {t:>6.0f} | {hard[i]*1e3:9.1f} | {soft[i]*1e3:9.1f} "
                  f"| {r:9.3f}{tag}")
        mid_ratio = np.mean([ratios[t] for t in MID_TAUS])
        short_ratio = np.mean([ratios[t] for t in SHORT_TAUS])
        # GO: mid-τ improved (ratio < 1) and short-τ not regressed (≤1.05).
        host_go = (mid_ratio < 1.0) and (short_ratio <= 1.05)
        overall_go = overall_go and host_go
        print(f"  → mid-τ mean soft/hard = {mid_ratio:.3f} "
              f"(want <1.0), short-τ = {short_ratio:.3f} (want ≤1.05) "
              f"→ {'GO' if host_go else 'NO-GO'}\n")
    print("=" * 50)
    print(f"OVERALL: {'GO — soft gate improves mid-τ without short-τ regress'
                      if overall_go else 'NO-GO — investigate before lab A/B'}")
    return 0 if overall_go else 1


if __name__ == "__main__":
    sys.exit(main())

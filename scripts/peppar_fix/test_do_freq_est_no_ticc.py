"""gnss-phase-experiment: DOFreqEst.update with optional measurement arms.

Originally written when the EKF still took a positional offset_ns;
this file pinned the "tolerate offset_ns=None" path that proved the
servo could be invoked without a TICC measurement.  After the
2026-05-06 four-arm refactor (per docs/dofreq-est-measurement-
ladder.md), every arm is keyword-only and conditionally gated on
availability — these tests now exercise the *PPP-only* arm
combination via the same physical scenarios.
"""
from __future__ import annotations

import math
import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.do_freq_est import DOFreqEst


class PPPOnlyArmTest(unittest.TestCase):

    def test_first_epoch_ppp_only_does_not_crash(self):
        # Only Arm 1 is fed.  No TICC, no EXTINT, no qErr arm.
        # The first call must not crash; LQR runs against the
        # initial state; output is finite.
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        adj = servo.update(dt=1.0, dt_rx_ns=0.0, dt_rx_sigma_ns=0.1)
        self.assertTrue(math.isfinite(adj))

    def test_steady_state_ppp_only_returns_finite_adjfine(self):
        # 100 epochs of PPP-only.  No other arm — x[2]/x[3] are
        # unobservable through measurement (only via dynamics +
        # LQR feedback).  Adjfine should stay bounded by LQR cap.
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        last_adj = 0.0
        for k in range(100):
            adj = servo.update(dt=1.0,
                               dt_rx_ns=float(k),
                               dt_rx_sigma_ns=0.1)
            self.assertTrue(math.isfinite(adj),
                            f"adj diverged at epoch {k}: {adj}")
            last_adj = adj
        self.assertLess(abs(last_adj), servo.max_ppb + 1.0)

    def test_seed_via_ticc_then_ppp_only(self):
        # Warm-disconnect scenario: first epoch TICC seeds x[2],
        # subsequent epochs are PPP-only (TICC arm gone).
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        adj0 = servo.update(dt=1.0,
                            dt_rx_ns=0.0, dt_rx_sigma_ns=0.1,
                            ticc_diff_ns=-50.0)
        self.assertFalse(servo._need_phc_seed)
        self.assertTrue(math.isfinite(adj0))
        for k in range(1, 50):
            adj = servo.update(dt=1.0,
                               dt_rx_ns=float(k) * 0.1,
                               dt_rx_sigma_ns=0.1)
            self.assertTrue(math.isfinite(adj),
                            f"adj diverged at epoch {k}: {adj}")

    def test_seed_flag_persists_until_x2_observable(self):
        # When neither EXTINT nor TICC arm is fed and only PPP arrives,
        # x[2] never gets a direct measurement so the seed flag must
        # stay True (filter still hopes for a future seed).
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        self.assertTrue(servo._need_phc_seed)
        servo.update(dt=1.0, dt_rx_ns=0.0, dt_rx_sigma_ns=0.1)
        self.assertTrue(servo._need_phc_seed,
                        "seed flag must persist until EXTINT or TICC arrives")


if __name__ == "__main__":
    unittest.main()

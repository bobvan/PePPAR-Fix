"""DOFreqEst four-arm Kalman fusion tests.

Each arm has an independent test: feeding only that arm should
move the corresponding state in the predicted direction with the
predicted gain.  Per docs/dofreq-est-measurement-ladder.md.

  Arm 1 (PPP)     observes x[0]
  Arm 2 (qErr)    observes x[1]
  Arm 3 (EXTINT)  observes x[2]
  Arm 4 (TICC)    observes -x[2] - qerr(x[0])  (nonlinear coupling)

Tests intentionally use loose numerical tolerances — the EKF's
exact gains depend on Q, R, and dynamics.  These tests pin
*direction* and *gross magnitude*, not numerical values.
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


class IndividualArmObservabilityTest(unittest.TestCase):

    def test_ppp_arm_pulls_x0_toward_measurement(self):
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        # Force x[0] far from a measurement, then feed PPP at the
        # opposite value.  x[0] should move significantly toward it.
        servo.x[0] = 100.0
        before = servo.x[0]
        servo.update(dt=1.0, dt_rx_ns=-100.0, dt_rx_sigma_ns=0.1)
        after = servo.x[0]
        self.assertLess(after, before,
                        "PPP measurement of -100 should pull x[0] down from +100")
        self.assertLess(abs(after - (-100.0)), abs(before - (-100.0)),
                        "x[0] should be closer to measurement after update")

    def test_qerr_freq_arm_pulls_x1_toward_measurement(self):
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        # Force x[1] to one frequency, observe the qErr arm pulling
        # toward another.
        servo.x[1] = 100.0
        before = servo.x[1]
        # Tight sigma → high gain on x[1].
        servo.update(dt=1.0, qerr_freq_ppb=0.0, qerr_freq_sigma_ppb=0.001)
        after = servo.x[1]
        self.assertLess(abs(after), abs(before),
                        "qErr-freq arm should pull x[1] toward measurement (0)")

    def test_extint_arm_pulls_x2_toward_measurement(self):
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        # Seed via the EXTINT arm so x[2] starts near a known value,
        # then feed a different EXTINT measurement and verify the
        # filter follows.
        servo.update(dt=1.0, extint_phase_ns=200.0, extint_sigma_ns=10.0)
        self.assertFalse(servo._need_phc_seed,
                         "EXTINT arm should fulfill the seed")
        # Should be near 200 after seed.
        self.assertAlmostEqual(servo.x[2], 200.0, delta=15.0)
        # Now feed a series of measurements at -100 and watch x[2]
        # converge toward it.
        for _ in range(30):
            servo.update(dt=1.0, extint_phase_ns=-100.0, extint_sigma_ns=10.0)
        self.assertAlmostEqual(servo.x[2], -100.0, delta=20.0,
                               msg="x[2] should converge toward EXTINT measurements")

    def test_extint_arm_alone_makes_x2_observable(self):
        # The structural payoff: PPP alone leaves x[2] unobservable
        # (yesterday's failure).  EXTINT alone makes x[2] observable
        # to its measurement noise floor.
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        # Run 50 epochs with EXTINT only (no PPP, no TICC, no qErr).
        for k in range(50):
            servo.update(dt=1.0,
                         extint_phase_ns=0.0,
                         extint_sigma_ns=10.0)
        # x[2] should be tightly held near 0 by the EXTINT arm.
        self.assertLess(abs(servo.x[2]), 30.0,
                        f"EXTINT-only should hold x[2] near 0; got {servo.x[2]}")

    def test_ticc_arm_seed_then_track(self):
        # TICC arm seeds via the existing nonlinear path, then tracks.
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        servo.update(dt=1.0, ticc_diff_ns=-50.0)
        self.assertFalse(servo._need_phc_seed)
        self.assertTrue(math.isfinite(servo.x[2]))


class ArmGatingTest(unittest.TestCase):
    """Each arm must be independently gated.  Passing None for an arm
    must skip that update without affecting the others."""

    def test_no_arms_predict_only_succeeds(self):
        # All arms None — predict step runs, no measurement updates.
        # Must not crash, must return finite adjfine.
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        adj = servo.update(dt=1.0)
        self.assertTrue(math.isfinite(adj))

    def test_predict_only_increments_uncertainty(self):
        # No measurements → P should grow due to process noise Q.
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        before = servo.P[2, 2]
        for _ in range(5):
            servo.update(dt=1.0)
        after = servo.P[2, 2]
        self.assertGreater(after, before,
                           "P[2,2] should grow without measurements")

    def test_partial_arms_do_not_crash(self):
        # Various arm combinations must all succeed.
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        # PPP + EXTINT, no TICC, no qErr
        adj = servo.update(dt=1.0,
                           dt_rx_ns=1.0, dt_rx_sigma_ns=0.1,
                           extint_phase_ns=10.0, extint_sigma_ns=5.0)
        self.assertTrue(math.isfinite(adj))
        # PPP + qErr + EXTINT, no TICC
        adj = servo.update(dt=1.0,
                           dt_rx_ns=2.0, dt_rx_sigma_ns=0.1,
                           qerr_freq_ppb=10.0, qerr_freq_sigma_ppb=0.1,
                           extint_phase_ns=12.0, extint_sigma_ns=5.0)
        self.assertTrue(math.isfinite(adj))
        # PPP + qErr + EXTINT + TICC
        adj = servo.update(dt=1.0,
                           dt_rx_ns=3.0, dt_rx_sigma_ns=0.1,
                           qerr_freq_ppb=10.0, qerr_freq_sigma_ppb=0.1,
                           extint_phase_ns=14.0, extint_sigma_ns=5.0,
                           ticc_diff_ns=-14.0, ticc_sigma_ns=0.1)
        self.assertTrue(math.isfinite(adj))


class SeedPreferenceTest(unittest.TestCase):
    """The seed prefers EXTINT over TICC (linear measurement is
    cleaner than nonlinear coupling for first-epoch initialization)."""

    def test_extint_seeds_when_both_arms_fed(self):
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        # Feed both EXTINT (=200) and TICC (which would seed to a
        # different value via -ticc - qerr(x[0])).  Verify the seed
        # lands near the EXTINT measurement.
        servo.update(dt=1.0,
                     extint_phase_ns=200.0, extint_sigma_ns=10.0,
                     ticc_diff_ns=-50.0)
        self.assertAlmostEqual(servo.x[2], 200.0, delta=20.0,
                               msg="seed must prefer EXTINT")
        self.assertFalse(servo._need_phc_seed)

    def test_ticc_seeds_when_extint_absent(self):
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        # No EXTINT → TICC seeds.  ticc_diff_ns=-50, x[0]=0 → qerr=0,
        # so seed is x[2] = -(-50) - 0 = 50.
        servo.update(dt=1.0, ticc_diff_ns=-50.0)
        self.assertAlmostEqual(servo.x[2], 50.0, delta=5.0,
                               msg="TICC seed: x[2] = -ticc_diff - qerr(x[0])")
        self.assertFalse(servo._need_phc_seed)


if __name__ == "__main__":
    unittest.main()

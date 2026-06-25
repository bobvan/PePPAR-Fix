"""DOFreqEst Arm 7 (ClockMatrix PHASE_STATUS) tests.

Arm 7 observes x[2] = DO phase, identical H to Arm 3 (EXTINT) but read
on-chip from the ClockMatrix DPLL PFD (no TICC, no EXTINT wire) — the
TICC-free DO-phase observer for Timebeat OTC hosts.  These pin the
arm's observability, seeding, convergence, and per-arm logging.
"""
from __future__ import annotations

import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.do_freq_est import DOFreqEst


class CmPhaseArmTest(unittest.TestCase):

    def test_seeds_x2(self):
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        servo.update(dt=1.0, cm_phase_ns=200.0, cm_phase_sigma_ns=0.05)
        self.assertFalse(servo._need_phc_seed,
                         "cm_phase arm should fulfill the x[2] seed")
        self.assertAlmostEqual(servo.x[2], 200.0, delta=5.0)

    def test_seed_prefers_cm_phase_over_extint(self):
        # When both DO-phase observers arrive the same epoch, the cleaner
        # on-chip cm_phase (~50 ps) seeds x[2], not the noisier EXTINT.
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        servo.update(dt=1.0,
                     cm_phase_ns=200.0, cm_phase_sigma_ns=0.05,
                     extint_phase_ns=-500.0, extint_sigma_ns=10.0)
        self.assertFalse(servo._need_phc_seed)
        self.assertLess(abs(servo.x[2] - 200.0), abs(servo.x[2] - (-500.0)),
                        "x[2] seed should come from cm_phase, not EXTINT")

    def test_pulls_x2_toward_measurement(self):
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        servo.update(dt=1.0, cm_phase_ns=200.0, cm_phase_sigma_ns=0.05)
        for _ in range(30):
            servo.update(dt=1.0, cm_phase_ns=-100.0, cm_phase_sigma_ns=0.05)
        self.assertAlmostEqual(servo.x[2], -100.0, delta=20.0,
                               msg="x[2] should converge toward cm_phase")

    def test_makes_x2_observable_alone(self):
        # cm_phase alone (no TICC/EXTINT/pseudo) must render x[2] observable:
        # its covariance should shrink when only cm_phase is fed.
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        servo.update(dt=1.0, cm_phase_ns=0.0, cm_phase_sigma_ns=0.05)
        p_after_seed = servo.P[2, 2]
        for _ in range(10):
            servo.update(dt=1.0, cm_phase_ns=0.0, cm_phase_sigma_ns=0.05)
        self.assertLess(servo.P[2, 2], p_after_seed + 1e-9,
                        "x[2] covariance should not grow unbounded with cm_phase")
        self.assertLess(servo.P[2, 2], 1.0,
                        "x[2] should be well-observed (P[2,2] small) via cm_phase")

    def test_logs_per_arm_innovation(self):
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        servo.update(dt=1.0, cm_phase_ns=50.0, cm_phase_sigma_ns=0.05)  # seed
        servo.update(dt=1.0, cm_phase_ns=60.0, cm_phase_sigma_ns=0.05)
        self.assertIn('cm_phase', servo.last_arm_innov)
        self.assertIsNotNone(servo.last_arm_innov['cm_phase'],
                             "cm_phase arm should log its innovation")

    def test_not_sole_observer_blocks_ticc_solo_override(self):
        # With cm_phase present, TICC is not the SOLE DO-phase observer, so
        # the ticc-solo chi² override path must not treat it as such.
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        # Seed + a cm_phase obs alongside a wild TICC outlier; the run must
        # not raise and x[2] must stay anchored near cm_phase, not the outlier.
        servo.update(dt=1.0, cm_phase_ns=0.0, cm_phase_sigma_ns=0.05)
        for _ in range(5):
            servo.update(dt=1.0, cm_phase_ns=0.0, cm_phase_sigma_ns=0.05,
                         ticc_diff_ns=10000.0, ticc_sigma_ns=0.06)
        self.assertLess(abs(servo.x[2]), 100.0,
                        "cm_phase should anchor x[2] against a TICC outlier")

    def test_absent_arm_is_noop(self):
        # Not passing cm_phase must change nothing vs. the existing arms.
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        servo.update(dt=1.0, extint_phase_ns=10.0, extint_sigma_ns=5.0)
        x2 = servo.x[2]
        servo.update(dt=1.0)  # no cm_phase, no arms
        self.assertIsNone(servo.last_arm_innov['cm_phase'])
        self.assertAlmostEqual(servo.x[2], x2, delta=5.0)


if __name__ == "__main__":
    unittest.main()

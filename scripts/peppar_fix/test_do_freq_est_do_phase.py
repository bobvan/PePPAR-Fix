"""DOFreqEst Arm 8 (do_phase) tests.

Arm 8 observes x[2] = DO phase directly from a receiver-clock measurement,
for topologies where the receiver clock IS the disciplined oscillator
(GNSSDO+/SXT-D: the mosaic-T runs off the steered OCXO, so its PPP
carrier-phase dt_rx observes the DO phase, not a separate rx TCXO).  Same
linear H as Arm 3 (EXTINT).  These pin the arm's observability, seeding,
convergence, per-arm logging, and — critically — the closed-loop sign.
"""
from __future__ import annotations

import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.do_freq_est import DOFreqEst


class DoPhaseArmTest(unittest.TestCase):

    def test_seeds_x2(self):
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        servo.update(dt=1.0, do_phase_ns=200.0, do_phase_sigma_ns=0.1)
        self.assertFalse(servo._need_phc_seed,
                         "do_phase arm should fulfill the x[2] seed")
        self.assertAlmostEqual(servo.x[2], 200.0, delta=5.0)

    def test_pulls_x2_toward_measurement(self):
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        servo.update(dt=1.0, do_phase_ns=200.0, do_phase_sigma_ns=0.1)
        for _ in range(30):
            servo.update(dt=1.0, do_phase_ns=-100.0, do_phase_sigma_ns=0.1)
        self.assertAlmostEqual(servo.x[2], -100.0, delta=20.0,
                               msg="x[2] should converge toward do_phase")

    def test_makes_x2_observable_alone(self):
        # do_phase alone (no TICC/EXTINT/cm_phase) must render x[2]
        # observable — the whole point for a GNSSDO+ obs-only servo.
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        servo.update(dt=1.0, do_phase_ns=0.0, do_phase_sigma_ns=0.1)
        p_after_seed = servo.P[2, 2]
        for _ in range(10):
            servo.update(dt=1.0, do_phase_ns=0.0, do_phase_sigma_ns=0.1)
        self.assertLess(servo.P[2, 2], p_after_seed + 1e-9,
                        "x[2] covariance must not grow unbounded with do_phase")
        self.assertLess(servo.P[2, 2], 1.0,
                        "x[2] should be well-observed via do_phase alone")

    def test_logs_per_arm_innovation(self):
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        servo.update(dt=1.0, do_phase_ns=50.0, do_phase_sigma_ns=0.1)  # seed
        servo.update(dt=1.0, do_phase_ns=60.0, do_phase_sigma_ns=0.1)
        self.assertIn('do_phase', servo.last_arm_innov)
        self.assertIsNotNone(servo.last_arm_innov['do_phase'],
                             "do_phase arm should log its innovation")

    def test_closed_loop_sign_drives_phase_to_zero(self):
        # The safety-critical property: a persistent positive DO phase error
        # must produce an actuator command (−update()) of the sign that
        # reduces it.  Simulate the loop: freq command feeds back into the
        # phase at the next epoch.  With correct signs the phase converges
        # to ~0; a sign error would diverge.
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        C_NS_PER_PPB_PER_S = 1.0  # 1 ppb ≈ 1 ns/s of phase accrual
        phase_ns = 500.0          # start 500 ns late
        servo.update(dt=1.0, do_phase_ns=phase_ns, do_phase_sigma_ns=0.1)  # seed
        applied_ppb = 0.0
        for _ in range(200):
            # phase accrues at the (negative of the) applied frequency:
            # a +freq command speeds the DO up, reducing a positive lateness.
            phase_ns += -applied_ppb * C_NS_PER_PPB_PER_S * 1.0
            freq_est = servo.update(dt=1.0, do_phase_ns=phase_ns,
                                    do_phase_sigma_ns=0.1)
            applied_ppb = -freq_est   # engine applies −update()
        self.assertLess(abs(phase_ns), 50.0,
                        f"closed loop should null the phase; got {phase_ns:.1f} ns "
                        f"(a divergence here means a sign error)")


if __name__ == "__main__":
    unittest.main()

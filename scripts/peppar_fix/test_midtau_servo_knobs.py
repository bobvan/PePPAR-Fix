"""midTauServoKnobs (I-084500, bravo↔delta): tests for the servo_sim
mechanism-probe tooling and the do_freq_clamp x[3] anti-windup backstop.

These lock in the servo_sim findings that redirected delta's mid-τ-hump
investigation (see docs/mid-tau-servo-knobs.md):

  * a dt_rx (PPP rx-clock) glitch is DECOUPLED from DO steering — it moves
    x[0] but leaves the DO output unchanged below the actuator LSB, because
    the DO-phase observation is GPS-referenced (qErr sub-tick only), not
    rx-relative;
  * a DO-phase-observation bias ramp DOES grow the DO output, but by
    gating the DO-phase arms out → the servo COASTS (actuator stays
    bounded), NOT by winding the actuator to the rail;
  * do_freq_clamp_ppb bounds x[3] to ±clamp of nominal (default off →
    byte-identical).
"""
import unittest

import numpy as np

from peppar_fix.do_freq_est import DOFreqEst
from peppar_fix.servo_sim import preset, ClosedLoopSim

class TestDoFreqClamp(unittest.TestCase):
    """The x[3] anti-windup backstop (DOFreqEst unit level)."""

    def _ekf(self, **kw):
        # initial_freq=0 → x[3] nominal = 0, so the clamp bounds |x[3]|.
        return DOFreqEst(initial_freq=0.0, initial_dt_rx_ns=0.0, **kw)

    def test_clamp_bounds_x3(self):
        ekf = self._ekf(do_freq_clamp_ppb=5.0)
        ekf.x[3] = 100.0                 # non-physical freq estimate
        ekf.update(dt=1.0)               # predict-only; clamp fires
        self.assertLessEqual(abs(ekf.x[3]), 5.0 + 1e-9)
        self.assertTrue(ekf.last_do_freq_clamped)
        self.assertEqual(ekf.n_do_freq_clamped, 1)

    def test_clamp_default_off_does_not_bound(self):
        ekf = self._ekf()                # no clamp
        ekf.x[3] = 100.0
        ekf.update(dt=1.0)
        # Without the clamp x[3] is NOT pulled back to ±5 (well under the
        # 1e6 gross-sanity bound, so the sanity guard leaves it alone too).
        self.assertGreater(abs(ekf.x[3]), 50.0)
        self.assertFalse(ekf.last_do_freq_clamped)
        self.assertEqual(ekf.n_do_freq_clamped, 0)

    def test_clamp_inactive_when_in_range(self):
        ekf = self._ekf(do_freq_clamp_ppb=50.0)
        ekf.x[3] = 3.0                   # within ±50
        ekf.update(dt=1.0)
        self.assertFalse(ekf.last_do_freq_clamped)


class TestSimMechanismProbes(unittest.TestCase):
    """servo_sim injection hooks that produced the mid-τ findings."""

    def _run(self, duration_s=1600.0, **ov):
        return ClosedLoopSim(preset("piface-current", duration_s=duration_s, **ov)).run()

    def test_dt_rx_glitch_is_decoupled_from_DO_output(self):
        """A dt_rx (PPP) glitch perturbs the rx-clock estimate but leaves
        the DO output unchanged below the actuator LSB — the decoupling
        finding that says dt_rx innovation gating is the wrong arm."""
        r0 = self._run()
        r1 = self._run(dt_rx_glitch=(1200.0, -12.0))
        dphi = float(np.max(np.abs(r0.phi_do_true_ns - r1.phi_do_true_ns)))
        dx2 = float(np.max(np.abs(r0.est_phi_do_ns - r1.est_phi_do_ns)))
        # The glitch DID enter the filter (x[2] moved a hair via qErr
        # coupling)…
        self.assertGreater(dx2, 0.0)
        # …but the DO output is unchanged: the perturbation is below the
        # adjfine quantum, so it never reaches the DO.
        self.assertEqual(dphi, 0.0)

    def test_do_obs_bias_ramp_coasts_not_rails(self):
        """A DO-phase-obs bias ramp grows the DO output (gate-out→coast)
        but the ACTUATOR stays bounded — the sim's servo does not rail,
        so delta's +386 ppb rail is not reproducible here."""
        r = self._run(duration_s=2400.0, do_obs_bias_ramp=(1200.0, 0.5))
        m = r.t_s >= 1300
        do_exc = float(np.nanmax(np.abs(r.phi_do_true_ns[m])))
        adj_max = float(np.nanmax(np.abs(r.adjfine_ppb[m])))
        self.assertGreater(do_exc, 100.0)     # DO output blows up (coast)
        self.assertLess(adj_max, 10.0)        # …but actuator stays bounded


if __name__ == "__main__":
    unittest.main()

"""Tests for the chi-squared innovation gate on Arm 4 (TICC)."""

import numpy as np
import unittest

from peppar_fix.do_freq_est import DOFreqEst, _CHI2_GATE_THRESHOLD


def _baseline(**kw):
    defaults = dict(
        sigma_ticc_ns=0.060,
        sigma_do_phase_ns=0.92,
        sigma_do_freq_ppb=0.01,
        sigma_tcxo_phase_ns=2.0,
        sigma_tcxo_freq_ppb=0.1,
        initial_freq=0.0,
    )
    defaults.update(kw)
    return DOFreqEst(**defaults)


class TestChi2Gate(unittest.TestCase):

    def test_huge_innov_rejected_when_P_tight(self):
        """After P converges, a 1M ns innovation is rejected."""
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 0.0, 0.0, 0.0])
        # Tighten P to simulate converged filter
        f.P = np.diag([1.0, 0.01, 1.0, 0.01])
        x2_before = f.x[2]
        f.update(dt=1.0, ticc_diff_ns=1e6, ticc_sigma_ns=0.060)
        self.assertAlmostEqual(f.x[2], x2_before, delta=10.0,
                               msg="State should barely move when gate rejects")
        self.assertIsNotNone(f.last_arm_innov.get('ticc'))

    def test_large_innov_accepted_when_P_large(self):
        """During bootstrap (P large), even 500 ns innovation passes."""
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 0.0, -500.0, 0.0])
        # P[2,2] = 5000² from default init → S is huge → χ² is tiny
        x2_before = f.x[2]
        f.update(dt=1.0, ticc_diff_ns=0.0, ticc_sigma_ns=0.060)
        self.assertNotAlmostEqual(f.x[2], x2_before, places=0,
                                  msg="500 ns innov should pass when P is large")

    def test_normal_innov_passes(self):
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 0.0, 0.0, 0.0])
        f.update(dt=1.0, ticc_diff_ns=-44.0, ticc_sigma_ns=0.060)
        self.assertGreater(abs(f.x[2]), 1.0)

    def test_gate_threshold_constant(self):
        self.assertEqual(_CHI2_GATE_THRESHOLD, 100.0)

    def test_innov_monitor_not_fed_on_rejection(self):
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 0.0, 0.0, 0.0])
        f.P = np.diag([1.0, 0.01, 1.0, 0.01])
        n_before = len(f.innov_monitor._innov)
        f.update(dt=1.0, ticc_diff_ns=1e6, ticc_sigma_ns=0.060)
        self.assertEqual(len(f.innov_monitor._innov), n_before)

    def test_convergence_from_500ns_error(self):
        """Cold start with 500 ns error converges — P is large enough
        that the gate doesn't block the corrections."""
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 0.0, -500.0, 0.0])
        for _ in range(15):
            f.update(dt=1.0, ticc_diff_ns=0.0, ticc_sigma_ns=0.060)
        self.assertLess(abs(f.x[2]), 10.0)


class TestStateSanityGuard(unittest.TestCase):

    def test_normal_state_not_flagged(self):
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([100.0, -15.0, -3.0, -135.0])
        f.update(dt=1.0, ticc_diff_ns=-3.0, ticc_sigma_ns=0.060)
        self.assertFalse(f._state_corrupted)

    def test_huge_x0_flagged(self):
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([2e8, 0.0, 0.0, 0.0])
        f.update(dt=1.0)
        self.assertTrue(f._state_corrupted)

    def test_huge_x1_flagged(self):
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 20000.0, 0.0, 0.0])
        f.update(dt=1.0)
        self.assertTrue(f._state_corrupted)


if __name__ == "__main__":
    unittest.main()

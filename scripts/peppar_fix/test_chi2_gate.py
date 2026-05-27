"""Tests for the adaptive innovation gate on Arm 4 (TICC)."""

import numpy as np
import unittest

from peppar_fix.do_freq_est import DOFreqEst, _CHI2_GATE_NSIGMA


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


def _warm_up_gate(f, n=10, ticc_val=-5.0):
    """Feed n clean TICC epochs to fill the innovation buffer."""
    for _ in range(n):
        f.update(dt=1.0, ticc_diff_ns=ticc_val, ticc_sigma_ns=0.060)


class TestAdaptiveGate(unittest.TestCase):

    def test_first_epochs_always_accepted(self):
        """Before buffer has 5 samples, gate doesn't fire."""
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 0.0, -500.0, 0.0])
        x2_before = f.x[2]
        f.update(dt=1.0, ticc_diff_ns=0.0, ticc_sigma_ns=0.060)
        self.assertNotAlmostEqual(f.x[2], x2_before, places=0,
                                  msg="Large innov should pass before buffer fills")

    def test_huge_innov_rejected_after_warmup(self):
        """23M ns innovation rejected once buffer has clean history."""
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 0.0, 0.0, 0.0])
        _warm_up_gate(f, n=10, ticc_val=-5.0)
        x2_before = f.x[2]
        f.x[2] = -1e6
        f.update(dt=1.0, ticc_diff_ns=0.0, ticc_sigma_ns=0.060)
        self.assertAlmostEqual(f.x[2], x2_before - 1e6, delta=100.0,
                               msg="State should barely move when gate rejects")

    def test_normal_innov_passes_after_warmup(self):
        """Small TICC innovation passes through after warmup."""
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 0.0, 0.0, 0.0])
        _warm_up_gate(f, n=10)
        f.update(dt=1.0, ticc_diff_ns=-44.0, ticc_sigma_ns=0.060)
        self.assertGreater(abs(f.x[2]), 1.0)

    def test_gate_adapts_during_convergence(self):
        """Large bootstrap error converges without gate rejections."""
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 0.0, -500.0, 0.0])
        for i in range(15):
            f.update(dt=1.0, ticc_diff_ns=0.0, ticc_sigma_ns=0.060)
        self.assertLess(abs(f.x[2]), 10.0,
                        msg="Filter should converge from -500 ns error "
                            "without gate blocking corrections")

    def test_gate_nsigma_constant(self):
        self.assertEqual(_CHI2_GATE_NSIGMA, 10.0)

    def test_innov_monitor_not_fed_on_rejection(self):
        """InnovControlMonitor should not see rejected innovations."""
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 0.0, 0.0, 0.0])
        _warm_up_gate(f, n=10, ticc_val=-5.0)
        n_before = len(f.innov_monitor._innov)
        f.x[2] = -1e6
        f.update(dt=1.0, ticc_diff_ns=0.0, ticc_sigma_ns=0.060)
        self.assertEqual(len(f.innov_monitor._innov), n_before)


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

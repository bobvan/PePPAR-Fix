"""Tests for DoVsRxTcxoOffset — rolling DO-vs-rx_TCXO offset capture."""

import math
import unittest

from peppar_fix.do_vs_rxtcxo_offset import DoVsRxTcxoOffset


class TestDoVsRxTcxoOffsetBasics(unittest.TestCase):

    def test_first_update_seeds_offset(self):
        cap = DoVsRxTcxoOffset(tau_s=60.0)
        cap.update(x0_phi_rx_ns=100.0, x2_phi_do_ns=105.0,
                   p00=0.01, p22=0.04, mono_s=1000.0)
        self.assertAlmostEqual(cap.last_offset_ns, 5.0)
        self.assertAlmostEqual(cap.last_offset_sigma_ns,
                               math.sqrt(0.05), places=6)
        self.assertEqual(cap.n_samples, 1)
        self.assertEqual(cap.last_capture_mono, 1000.0)

    def test_none_before_first_update(self):
        cap = DoVsRxTcxoOffset()
        self.assertIsNone(cap.last_offset_ns)
        self.assertIsNone(cap.last_capture_mono)
        self.assertEqual(cap.n_samples, 0)
        self.assertFalse(cap.ready)

    def test_ema_converges_to_steady_state(self):
        cap = DoVsRxTcxoOffset(tau_s=10.0)
        for i in range(200):
            cap.update(x0_phi_rx_ns=0.0, x2_phi_do_ns=42.0,
                       p00=0.01, p22=0.01, mono_s=float(i))
        self.assertAlmostEqual(cap.last_offset_ns, 42.0, places=3)

    def test_ema_tracks_step_change(self):
        cap = DoVsRxTcxoOffset(tau_s=10.0)
        for i in range(100):
            cap.update(x0_phi_rx_ns=0.0, x2_phi_do_ns=10.0,
                       p00=0.01, p22=0.01, mono_s=float(i))
        self.assertAlmostEqual(cap.last_offset_ns, 10.0, places=2)
        for i in range(100, 200):
            cap.update(x0_phi_rx_ns=0.0, x2_phi_do_ns=20.0,
                       p00=0.01, p22=0.01, mono_s=float(i))
        self.assertAlmostEqual(cap.last_offset_ns, 20.0, places=2)

    def test_ema_time_constant_affects_responsiveness(self):
        fast = DoVsRxTcxoOffset(tau_s=5.0)
        slow = DoVsRxTcxoOffset(tau_s=60.0)
        for i in range(50):
            val = 10.0 if i < 25 else 20.0
            fast.update(0.0, val, 0.01, 0.01, float(i))
            slow.update(0.0, val, 0.01, 0.01, float(i))
        self.assertLess(abs(fast.last_offset_ns - 20.0),
                        abs(slow.last_offset_ns - 20.0))

    def test_negative_tau_raises(self):
        with self.assertRaises(ValueError):
            DoVsRxTcxoOffset(tau_s=-1.0)
        with self.assertRaises(ValueError):
            DoVsRxTcxoOffset(tau_s=0.0)

    def test_ready_requires_10_samples(self):
        cap = DoVsRxTcxoOffset(tau_s=60.0)
        for i in range(9):
            cap.update(0.0, 5.0, 0.01, 0.01, float(i))
            self.assertFalse(cap.ready)
        cap.update(0.0, 5.0, 0.01, 0.01, 9.0)
        self.assertTrue(cap.ready)


class TestDoVsRxTcxoOffsetHoldoverSigma(unittest.TestCase):

    def test_holdover_sigma_at_zero_duration(self):
        cap = DoVsRxTcxoOffset(tau_s=60.0)
        cap.update(0.0, 5.0, 0.01, 0.04, 0.0)
        sigma = cap.holdover_sigma_ns(0.0)
        self.assertAlmostEqual(sigma, math.sqrt(0.05), places=6)

    def test_holdover_sigma_grows_with_time(self):
        cap = DoVsRxTcxoOffset(tau_s=60.0)
        cap.update(0.0, 5.0, 0.01, 0.01, 0.0)
        s60 = cap.holdover_sigma_ns(60.0)
        s300 = cap.holdover_sigma_ns(300.0)
        s3600 = cap.holdover_sigma_ns(3600.0)
        self.assertGreater(s300, s60)
        self.assertGreater(s3600, s300)

    def test_holdover_sigma_s1_matches_spec(self):
        """PiFace σ_y=0.026 ppb: S1(60s) = 0.026 × √60 = 0.201 ns."""
        cap = DoVsRxTcxoOffset(tau_s=60.0)
        cap.update(0.0, 0.0, 0.0, 0.0, 0.0)
        sigma = cap.holdover_sigma_ns(60.0, tdcp_sigma_y_ppb=0.026)
        expected_s1 = 0.026 * math.sqrt(60.0)
        self.assertAlmostEqual(sigma, expected_s1, places=3)

    def test_holdover_sigma_with_bias_adds_linear_term(self):
        """If measured_bias_ppb = 0.01, S3(3600) = 0.01 × 3600 = 36 ns."""
        cap = DoVsRxTcxoOffset(tau_s=60.0, measured_bias_ppb=0.01)
        cap.update(0.0, 0.0, 0.0, 0.0, 0.0)
        sigma = cap.holdover_sigma_ns(3600.0, tdcp_sigma_y_ppb=0.026)
        s1 = 0.026 * math.sqrt(3600.0)
        s3 = 0.01 * 3600.0
        expected = math.sqrt(s1 ** 2 + s3 ** 2)
        self.assertAlmostEqual(sigma, expected, places=2)
        self.assertGreater(sigma, 35.0)

    def test_holdover_sigma_zero_bias_no_s3(self):
        """With zero bias, S3 = 0; only S1 + S2 contribute."""
        cap = DoVsRxTcxoOffset(tau_s=60.0, measured_bias_ppb=0.0)
        cap.update(0.0, 0.0, 0.01, 0.01, 0.0)
        sigma_no_bias = cap.holdover_sigma_ns(300.0)
        cap_bias = DoVsRxTcxoOffset(tau_s=60.0, measured_bias_ppb=0.005)
        cap_bias.update(0.0, 0.0, 0.01, 0.01, 0.0)
        sigma_with_bias = cap_bias.holdover_sigma_ns(300.0)
        self.assertGreater(sigma_with_bias, sigma_no_bias)

    def test_holdover_sigma_354ps_crossover_ocxo(self):
        """Verify ~2-5 min crossover for 354 ps on OCXO-class (PiFace)."""
        cap = DoVsRxTcxoOffset(tau_s=120.0)
        cap.update(0.0, 0.0, 0.001, 0.001, 0.0)
        at_60s = cap.holdover_sigma_ns(60.0, tdcp_sigma_y_ppb=0.026)
        at_300s = cap.holdover_sigma_ns(300.0, tdcp_sigma_y_ppb=0.026)
        self.assertLess(at_60s, 0.354)
        self.assertGreater(at_300s, 0.354)

    def test_holdover_sigma_1ns_crossover_ocxo(self):
        """Verify ~10-30 min crossover for 1 ns on OCXO-class."""
        cap = DoVsRxTcxoOffset(tau_s=120.0)
        cap.update(0.0, 0.0, 0.001, 0.001, 0.0)
        at_300s = cap.holdover_sigma_ns(300.0, tdcp_sigma_y_ppb=0.026)
        at_1800s = cap.holdover_sigma_ns(1800.0, tdcp_sigma_y_ppb=0.026)
        self.assertLess(at_300s, 1.0)
        self.assertGreater(at_1800s, 1.0)


class TestDoVsRxTcxoOffsetProperties(unittest.TestCase):

    def test_tau_s_exposed(self):
        cap = DoVsRxTcxoOffset(tau_s=200.0)
        self.assertEqual(cap.tau_s, 200.0)

    def test_measured_bias_exposed(self):
        cap = DoVsRxTcxoOffset(measured_bias_ppb=0.003)
        self.assertEqual(cap.measured_bias_ppb, 0.003)

    def test_default_bias_is_zero(self):
        cap = DoVsRxTcxoOffset()
        self.assertEqual(cap.measured_bias_ppb, 0.0)

    def test_default_tau_is_120(self):
        cap = DoVsRxTcxoOffset()
        self.assertEqual(cap.tau_s, 120.0)


if __name__ == "__main__":
    unittest.main()

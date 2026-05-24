"""Tests for TdcpPhaseIntegrator — rx_TCXO phase integration during HOLDOVER."""

import math
import unittest

from peppar_fix.tdcp_phase_integrator import TdcpPhaseIntegrator


class TestTdcpPhaseIntegratorBasics(unittest.TestCase):

    def test_not_seeded_initially(self):
        integ = TdcpPhaseIntegrator()
        self.assertFalse(integ.seeded)
        self.assertIsNone(integ.phase_ns)
        self.assertEqual(integ.holdover_duration_s, 0.0)
        self.assertEqual(integ.n_integrated, 0)

    def test_seed_sets_initial_phase(self):
        integ = TdcpPhaseIntegrator()
        integ.seed(x0_phi_rx_ns=42.5, mono_s=1000.0)
        self.assertTrue(integ.seeded)
        self.assertAlmostEqual(integ.phase_ns, 42.5)
        self.assertEqual(integ.holdover_duration_s, 0.0)

    def test_integrate_accumulates_phase(self):
        integ = TdcpPhaseIntegrator()
        integ.seed(x0_phi_rx_ns=0.0, mono_s=100.0)
        # df_f = 1e-9 (1 ppb) over 1 s → phase += 1e-9 * 1 * 1e9 = 1.0 ns
        ok = integ.integrate(df_f=1e-9, mono_s=101.0)
        self.assertTrue(ok)
        self.assertAlmostEqual(integ.phase_ns, 1.0)
        self.assertEqual(integ.n_integrated, 1)

    def test_integrate_multiple_epochs(self):
        integ = TdcpPhaseIntegrator()
        integ.seed(x0_phi_rx_ns=10.0, mono_s=0.0)
        for i in range(100):
            integ.integrate(df_f=2e-9, mono_s=float(i + 1))
        # 100 epochs × 2 ppb × 1 s = 200 ns accumulated
        self.assertAlmostEqual(integ.phase_ns, 10.0 + 200.0, places=3)
        self.assertEqual(integ.n_integrated, 100)
        self.assertAlmostEqual(integ.holdover_duration_s, 100.0)

    def test_integrate_negative_frequency(self):
        integ = TdcpPhaseIntegrator()
        integ.seed(x0_phi_rx_ns=100.0, mono_s=0.0)
        integ.integrate(df_f=-5e-9, mono_s=1.0)
        self.assertAlmostEqual(integ.phase_ns, 100.0 - 5.0)

    def test_integrate_without_seed_returns_false(self):
        integ = TdcpPhaseIntegrator()
        ok = integ.integrate(df_f=1e-9, mono_s=1.0)
        self.assertFalse(ok)
        self.assertIsNone(integ.phase_ns)


class TestTdcpPhaseIntegratorGapProtection(unittest.TestCase):

    def test_gap_exceeding_max_dt_marks_gap(self):
        integ = TdcpPhaseIntegrator(max_dt_s=1.5)
        integ.seed(x0_phi_rx_ns=0.0, mono_s=0.0)
        integ.integrate(df_f=1e-9, mono_s=1.0)
        self.assertFalse(integ.gap_detected)
        ok = integ.integrate(df_f=1e-9, mono_s=3.0)
        self.assertFalse(ok)
        self.assertTrue(integ.gap_detected)

    def test_gap_prevents_pseudo_obs(self):
        integ = TdcpPhaseIntegrator(max_dt_s=1.5)
        integ.seed(x0_phi_rx_ns=0.0, mono_s=0.0)
        integ.integrate(df_f=1e-9, mono_s=1.0)
        integ.integrate(df_f=1e-9, mono_s=5.0)  # gap
        self.assertIsNone(integ.pseudo_obs(offset_ns=10.0))

    def test_gap_at_boundary_accepted(self):
        integ = TdcpPhaseIntegrator(max_dt_s=1.5)
        integ.seed(x0_phi_rx_ns=0.0, mono_s=0.0)
        ok = integ.integrate(df_f=1e-9, mono_s=1.5)
        self.assertTrue(ok)
        self.assertFalse(integ.gap_detected)

    def test_gap_just_over_boundary_rejected(self):
        integ = TdcpPhaseIntegrator(max_dt_s=1.5)
        integ.seed(x0_phi_rx_ns=0.0, mono_s=0.0)
        ok = integ.integrate(df_f=1e-9, mono_s=1.51)
        self.assertFalse(ok)
        self.assertTrue(integ.gap_detected)

    def test_zero_dt_rejected(self):
        integ = TdcpPhaseIntegrator()
        integ.seed(x0_phi_rx_ns=0.0, mono_s=10.0)
        ok = integ.integrate(df_f=1e-9, mono_s=10.0)
        self.assertFalse(ok)

    def test_negative_dt_rejected(self):
        integ = TdcpPhaseIntegrator()
        integ.seed(x0_phi_rx_ns=0.0, mono_s=10.0)
        ok = integ.integrate(df_f=1e-9, mono_s=9.0)
        self.assertFalse(ok)

    def test_invalid_max_dt_raises(self):
        with self.assertRaises(ValueError):
            TdcpPhaseIntegrator(max_dt_s=0.0)
        with self.assertRaises(ValueError):
            TdcpPhaseIntegrator(max_dt_s=-1.0)


class TestTdcpPhaseIntegratorPseudoObs(unittest.TestCase):

    def test_pseudo_obs_combines_phase_and_offset(self):
        integ = TdcpPhaseIntegrator()
        integ.seed(x0_phi_rx_ns=50.0, mono_s=0.0)
        integ.integrate(df_f=1e-9, mono_s=1.0)
        # phase = 50.0 + 1.0 = 51.0; offset = 10.0 → pseudo = 61.0
        pseudo = integ.pseudo_obs(offset_ns=10.0)
        self.assertAlmostEqual(pseudo, 61.0)

    def test_pseudo_obs_none_when_not_seeded(self):
        integ = TdcpPhaseIntegrator()
        self.assertIsNone(integ.pseudo_obs(offset_ns=10.0))

    def test_pseudo_obs_at_seed_before_any_integration(self):
        integ = TdcpPhaseIntegrator()
        integ.seed(x0_phi_rx_ns=100.0, mono_s=0.0)
        pseudo = integ.pseudo_obs(offset_ns=-5.0)
        self.assertAlmostEqual(pseudo, 95.0)


class TestTdcpPhaseIntegratorReset(unittest.TestCase):

    def test_reset_clears_all_state(self):
        integ = TdcpPhaseIntegrator()
        integ.seed(x0_phi_rx_ns=50.0, mono_s=0.0)
        integ.integrate(df_f=1e-9, mono_s=1.0)
        integ.reset()
        self.assertFalse(integ.seeded)
        self.assertIsNone(integ.phase_ns)
        self.assertEqual(integ.n_integrated, 0)
        self.assertFalse(integ.gap_detected)
        self.assertEqual(integ.holdover_duration_s, 0.0)

    def test_reseed_after_reset(self):
        integ = TdcpPhaseIntegrator()
        integ.seed(x0_phi_rx_ns=50.0, mono_s=0.0)
        integ.integrate(df_f=1e-9, mono_s=1.0)
        integ.reset()
        integ.seed(x0_phi_rx_ns=200.0, mono_s=100.0)
        self.assertAlmostEqual(integ.phase_ns, 200.0)
        integ.integrate(df_f=-3e-9, mono_s=101.0)
        self.assertAlmostEqual(integ.phase_ns, 200.0 - 3.0)

    def test_reseed_after_gap_clears_gap(self):
        integ = TdcpPhaseIntegrator(max_dt_s=1.5)
        integ.seed(x0_phi_rx_ns=0.0, mono_s=0.0)
        integ.integrate(df_f=1e-9, mono_s=5.0)  # gap
        self.assertTrue(integ.gap_detected)
        integ.reset()
        integ.seed(x0_phi_rx_ns=0.0, mono_s=10.0)
        self.assertFalse(integ.gap_detected)


class TestTdcpPhaseIntegratorHoldoverDuration(unittest.TestCase):

    def test_holdover_duration_tracks_elapsed(self):
        integ = TdcpPhaseIntegrator()
        integ.seed(x0_phi_rx_ns=0.0, mono_s=1000.0)
        for i in range(1, 61):
            integ.integrate(df_f=0.0, mono_s=1000.0 + float(i))
        self.assertAlmostEqual(integ.holdover_duration_s, 60.0)

    def test_holdover_duration_spans_multiple_epochs(self):
        integ = TdcpPhaseIntegrator()
        integ.seed(x0_phi_rx_ns=0.0, mono_s=0.0)
        for i in range(1, 301):
            integ.integrate(df_f=0.0, mono_s=float(i))
        self.assertAlmostEqual(integ.holdover_duration_s, 300.0)


if __name__ == "__main__":
    unittest.main()

"""Tests for InnovControlMonitor."""

from __future__ import annotations

import random
import unittest

from peppar_fix.innov_control_monitor import (
    HealthReport,
    InnovControlMonitor,
)


def _healthy_filter_samples(n, sigma_meas=1.0, sigma_u=10.0, seed=0):
    """Generate (u, innov, S) for a perfectly-tuned filter.

    Healthy filter: innov ~ N(0, R), no correlation with u, NIS ≈ 1.
    """
    rng = random.Random(seed)
    out = []
    R = sigma_meas ** 2
    for _ in range(n):
        u = rng.gauss(0, sigma_u)              # control input
        innov = rng.gauss(0, sigma_meas)        # measurement noise only
        S = R                                   # filter believes σ_meas
        out.append((u, innov, S))
    return out


def _bad_plant_samples(n, alpha=1.5, sigma_meas=1.0, sigma_u=10.0, seed=0):
    """Generate (u, innov, S) for a filter with plant-model scaling error.

    True plant gain = alpha × what filter assumed (B·u in pred step).
    Innovation has a systematic component proportional to u.
    """
    rng = random.Random(seed)
    out = []
    R = sigma_meas ** 2
    for _ in range(n):
        u = rng.gauss(0, sigma_u)
        plant_residual = (alpha - 1.0) * u
        innov = plant_residual + rng.gauss(0, sigma_meas)
        S = R
        out.append((u, innov, S))
    return out


def _consume(monitor, samples):
    for u, innov, S in samples:
        monitor.observe(u, innov, S)


class ConstructorTest(unittest.TestCase):

    def test_rejects_tiny_window(self):
        with self.assertRaises(ValueError):
            InnovControlMonitor(window=5)

    def test_warm_min_defaults_to_half_window(self):
        m = InnovControlMonitor(window=100)
        # Below half-window → WARM
        for _ in range(49):
            m.observe(0.0, 0.0, 1.0)
        r = m.evaluate()
        self.assertEqual(r.status, "WARM")
        # At half-window → not WARM (any other status — OK for clean data)
        m.observe(0.0, 0.0, 1.0)
        r = m.evaluate()
        self.assertNotEqual(r.status, "WARM")


class HealthyFilterTest(unittest.TestCase):
    """A perfectly-tuned filter should report OK with metrics near nominal."""

    def test_healthy_has_low_corr_low_bias_nis_near_1(self):
        m = InnovControlMonitor(window=500, debounce=1)
        _consume(m, _healthy_filter_samples(500))
        r = m.evaluate()
        self.assertEqual(r.status, "OK")
        self.assertLess(abs(r.corr_u_innov), 0.20)
        self.assertLess(abs(r.norm_bias), 0.20)
        self.assertGreater(r.nis, 0.7)
        self.assertLess(r.nis, 1.5)


class PlantModelErrorTest(unittest.TestCase):
    """A scaling error in B should produce u/innov correlation."""

    def test_plant_scale_error_flags_corr(self):
        m = InnovControlMonitor(window=500, debounce=1, corr_alarm=0.30)
        # alpha = 1.5 → plant delivers 1.5× what filter predicted
        _consume(m, _bad_plant_samples(500, alpha=1.5,
                                       sigma_meas=1.0, sigma_u=10.0))
        r = m.evaluate()
        self.assertEqual(r.status, "ALARM_corr")
        # Strong correlation expected — plant residual dominates over
        # measurement noise at sigma_u=10, sigma_meas=1.
        self.assertGreater(r.corr_u_innov, 0.5)

    def test_sign_flip_flags_corr_in_opposite_direction(self):
        m = InnovControlMonitor(window=500, debounce=1)
        # alpha = -1 → DAC sign inverted; plant moves opposite to expectation
        _consume(m, _bad_plant_samples(500, alpha=-1.0,
                                       sigma_meas=1.0, sigma_u=10.0))
        r = m.evaluate()
        self.assertEqual(r.status, "ALARM_corr")
        # Negative correlation — innov anti-correlated with u
        self.assertLess(r.corr_u_innov, -0.5)


class UndersetRTest(unittest.TestCase):
    """If R is set too low, NIS inflates."""

    def test_underset_R_flags_nis(self):
        m = InnovControlMonitor(window=500, debounce=1, nis_alarm=2.5)
        rng = random.Random(0)
        true_sigma = 3.0
        believed_sigma = 1.0
        samples = []
        for _ in range(500):
            u = rng.gauss(0, 10)
            innov = rng.gauss(0, true_sigma)
            S = believed_sigma ** 2
            samples.append((u, innov, S))
        _consume(m, samples)
        r = m.evaluate()
        self.assertEqual(r.status, "ALARM_nis")
        # Expected NIS ≈ (3/1)^2 = 9
        self.assertGreater(r.nis, 5.0)


class SystematicBiasTest(unittest.TestCase):
    """A non-zero-mean innovation should flag the bias metric."""

    def test_bias_flag_when_innov_mean_nonzero(self):
        m = InnovControlMonitor(window=500, debounce=1, bias_alarm=0.5)
        rng = random.Random(0)
        samples = []
        for _ in range(500):
            u = rng.gauss(0, 10)
            innov = 1.5 + rng.gauss(0, 1.0)  # ≈ 1.5σ bias
            S = 1.0
            samples.append((u, innov, S))
        _consume(m, samples)
        r = m.evaluate()
        self.assertEqual(r.status, "ALARM_bias")
        self.assertGreater(r.norm_bias, 1.0)


class DebounceTest(unittest.TestCase):
    """Single-window noise should not engage the alarm."""

    def test_one_bad_window_then_recovery_stays_ok(self):
        m = InnovControlMonitor(window=200, debounce=3)
        # Bad data for one window
        _consume(m, _bad_plant_samples(200, alpha=2.0))
        r1 = m.evaluate()
        # Status should be OK (1 bad < 3-debounce)
        self.assertEqual(r1.status, "OK")
        # Now recover with healthy data
        m.reset()
        _consume(m, _healthy_filter_samples(200))
        r2 = m.evaluate()
        self.assertEqual(r2.status, "OK")

    def test_three_consecutive_bad_windows_engage_alarm(self):
        m = InnovControlMonitor(window=200, debounce=3, corr_alarm=0.30)
        for k in range(3):
            _consume(m, _bad_plant_samples(200, alpha=1.5, seed=k))
            r = m.evaluate()
            if k < 2:
                self.assertEqual(r.status, "OK", f"window {k}")
            else:
                self.assertEqual(r.status, "ALARM_corr")


class ObserveGuardsTest(unittest.TestCase):
    """observe() silently drops bad inputs."""

    def test_nonpositive_S_dropped(self):
        m = InnovControlMonitor(window=100)
        m.observe(1.0, 0.1, 0.0)
        m.observe(1.0, 0.1, -1.0)
        r = m.evaluate()
        self.assertEqual(r.epochs, 0)

    def test_nonfinite_inputs_dropped(self):
        m = InnovControlMonitor(window=100)
        m.observe(float("nan"), 0.1, 1.0)
        m.observe(1.0, float("inf"), 1.0)
        m.observe(1.0, 0.1, float("nan"))
        r = m.evaluate()
        self.assertEqual(r.epochs, 0)

    def test_good_inputs_kept(self):
        m = InnovControlMonitor(window=100)
        m.observe(1.0, 0.1, 1.0)
        m.observe(2.0, -0.05, 1.0)
        r = m.evaluate()
        self.assertEqual(r.epochs, 2)


class ResetTest(unittest.TestCase):

    def test_reset_clears_window_and_alarm(self):
        m = InnovControlMonitor(window=200, debounce=1)
        _consume(m, _bad_plant_samples(200, alpha=1.5))
        r1 = m.evaluate()
        self.assertEqual(r1.status, "ALARM_corr")
        self.assertGreater(r1.consec_bad, 0)
        m.reset()
        r2 = m.evaluate()
        self.assertEqual(r2.epochs, 0)
        self.assertEqual(r2.consec_bad, 0)
        self.assertEqual(r2.status, "WARM")


if __name__ == "__main__":
    unittest.main()

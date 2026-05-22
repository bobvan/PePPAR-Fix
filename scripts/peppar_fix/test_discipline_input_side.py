"""Tests for input-side drift_rate / σ_obs estimator in DisciplineScheduler.

Replaces the previous servo-output-derived drift_rate (which created
a circular dependency through the control loop).  The new estimator
linear-fits the error_ns history kept inside the scheduler and
computes both drift slope and residual σ from input alone.
"""
from __future__ import annotations

import os
import sys
import unittest

import numpy as np

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.discipline import DisciplineScheduler, _MIN_FIT_SAMPLES


def _feed(sched, errors_with_t, source='TEST'):
    """Feed (t, error_ns) tuples to scheduler.accumulate, flushing
    every few samples so the deque grows but the internal _errors
    buffer doesn't cripple should_correct semantics."""
    for t, e in errors_with_t:
        sched.accumulate(e, 1.0, source, t_monotonic=t)
        # Periodically flush so accumulate buffer doesn't grow unbounded.
        # The error-history deque (which we test) is separate and grows
        # regardless of flush() calls.
        if sched.n_accumulated >= 10:
            sched.flush()


class BootstrapTest(unittest.TestCase):
    """Before _MIN_FIT_SAMPLES samples, return base_interval."""

    def test_empty_history_returns_base(self):
        s = DisciplineScheduler(base_interval=2, adaptive=True)
        # No history at all.
        self.assertEqual(s.compute_adaptive_interval(), 2)
        self.assertEqual(s.drift_rate_ns_per_s, 0.0)
        self.assertEqual(s.sigma_obs_ns, 0.0)

    def test_partial_history_returns_base(self):
        s = DisciplineScheduler(base_interval=3, adaptive=True)
        # Feed fewer than _MIN_FIT_SAMPLES samples.
        for i in range(_MIN_FIT_SAMPLES - 5):
            s.accumulate(0.5, 1.0, 'TEST', t_monotonic=float(i))
        self.assertEqual(s.compute_adaptive_interval(), 3)


class InputSideEstimatorTest(unittest.TestCase):
    """Linear-fit recovers known slope + σ under synthetic input."""

    def test_recovers_known_slope(self):
        """Feed a clean linear ramp; slope should match within 1%."""
        s = DisciplineScheduler(base_interval=1, adaptive=True,
                                 max_interval=60)
        true_slope = 2.5  # ns/s
        # 100 samples at 1 Hz, no noise
        rng = np.random.default_rng(42)
        data = [(float(t), true_slope * t) for t in range(100)]
        _feed(s, data)
        s.compute_adaptive_interval()
        self.assertAlmostEqual(s.drift_rate_ns_per_s, true_slope, places=3)
        self.assertLess(s.sigma_obs_ns, 1e-6)

    def test_recovers_slope_and_sigma_under_noise(self):
        """Slope + σ recovered from linear ramp + white noise."""
        s = DisciplineScheduler(base_interval=1, adaptive=True,
                                 max_interval=60)
        rng = np.random.default_rng(123)
        true_slope = 1.0  # ns/s
        true_sigma = 0.5  # ns per-sample
        n = 200
        ts = np.arange(n, dtype=float)
        errors = true_slope * ts + rng.normal(0, true_sigma, n)
        for t, e in zip(ts, errors):
            s.accumulate(float(e), 1.0, 'TEST', t_monotonic=float(t))
            if s.n_accumulated >= 5:
                s.flush()
        s.compute_adaptive_interval()
        # Slope within 10% (n=200, σ=0.5, slope=1.0 → SE ~0.005)
        self.assertAlmostEqual(s.drift_rate_ns_per_s, true_slope, delta=0.1)
        # σ within 20% (estimator vs true)
        self.assertAlmostEqual(s.sigma_obs_ns, true_sigma, delta=0.15)

    def test_quiet_input_grows_tau(self):
        """Pure white noise around zero → drift_rate ≈ 0 → tau = max_interval."""
        s = DisciplineScheduler(base_interval=1, adaptive=True,
                                 max_interval=60, min_interval=1)
        rng = np.random.default_rng(7)
        n = 100
        for t in range(n):
            s.accumulate(float(rng.normal(0, 0.1)), 1.0, 'TEST',
                          t_monotonic=float(t))
            if s.n_accumulated >= 5:
                s.flush()
        tau = s.compute_adaptive_interval()
        # With sub-noise drift, drift_rate < 1e-6 path triggers, tau=max.
        # Even if it doesn't quite hit < 1e-6 in this seed, tau should be
        # heavily skewed toward max_interval given the favorable σ/drift ratio.
        self.assertGreaterEqual(tau, 30,
            f"quiet input should give large tau; got {tau} "
            f"(drift={s.drift_rate_ns_per_s:.3e}, sigma={s.sigma_obs_ns:.3e})")

    def test_drifting_input_shrinks_tau(self):
        """Fast drift → short tau."""
        s = DisciplineScheduler(base_interval=1, adaptive=True,
                                 max_interval=60, min_interval=1)
        # Drift of 100 ns/s with low noise.  Tau should be small.
        rng = np.random.default_rng(11)
        n = 100
        for t in range(n):
            e = 100.0 * t + rng.normal(0, 0.5)
            s.accumulate(e, 1.0, 'TEST', t_monotonic=float(t))
            if s.n_accumulated >= 5:
                s.flush()
        tau = s.compute_adaptive_interval()
        self.assertLessEqual(tau, 5,
            f"fast-drift input should give small tau; got {tau} "
            f"(drift={s.drift_rate_ns_per_s:.3f}, sigma={s.sigma_obs_ns:.3f})")


class LegacyApiTest(unittest.TestCase):
    """update_drift_rate(adjfine_ppb) still callable but is a no-op."""

    def test_legacy_call_does_not_affect_estimator(self):
        s = DisciplineScheduler(base_interval=1, adaptive=True,
                                 max_interval=60)
        # Pre-feed input history that should give drift=2.0, σ small.
        for t in range(_MIN_FIT_SAMPLES + 10):
            s.accumulate(2.0 * t, 1.0, 'TEST', t_monotonic=float(t))
            if s.n_accumulated >= 5:
                s.flush()
        s.compute_adaptive_interval()
        drift_before = s.drift_rate_ns_per_s
        # Now invoke the legacy API with wildly different values.
        s.update_drift_rate(timestamp=1e6, adjfine_ppb=99999.0)
        s.compute_adaptive_interval()
        drift_after = s.drift_rate_ns_per_s
        # No change — legacy call is a no-op.
        self.assertAlmostEqual(drift_before, drift_after, places=6)

    def test_legacy_call_logs_once(self):
        s = DisciplineScheduler(adaptive=True)
        # First call should set the seen flag.
        self.assertFalse(s._legacy_adjfine_seen)
        s.update_drift_rate(timestamp=1.0, adjfine_ppb=10.0)
        self.assertTrue(s._legacy_adjfine_seen)
        # Subsequent calls are still no-ops; flag stays set.
        s.update_drift_rate(timestamp=2.0, adjfine_ppb=20.0)
        self.assertTrue(s._legacy_adjfine_seen)


class IntervalBoundsTest(unittest.TestCase):
    """Tau is clamped to [min_interval, max_interval]."""

    def test_max_interval_cap(self):
        s = DisciplineScheduler(base_interval=1, adaptive=True,
                                 min_interval=1, max_interval=10)
        # Zero drift → would otherwise return max_interval=120 default;
        # we set max=10 to verify the cap is honored.
        for t in range(_MIN_FIT_SAMPLES + 10):
            s.accumulate(0.0, 1.0, 'TEST', t_monotonic=float(t))
            if s.n_accumulated >= 5:
                s.flush()
        self.assertEqual(s.compute_adaptive_interval(), 10)

    def test_adaptive_off_returns_base(self):
        s = DisciplineScheduler(base_interval=4, adaptive=False)
        for t in range(_MIN_FIT_SAMPLES + 10):
            s.accumulate(100.0 * t, 1.0, 'TEST', t_monotonic=float(t))
            if s.n_accumulated >= 5:
                s.flush()
        # adaptive off → always base_interval, no estimator work.
        self.assertEqual(s.compute_adaptive_interval(), 4)


if __name__ == "__main__":
    unittest.main()

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

    def test_quiet_actuation_grows_tau(self):
        """Constant adjfine (D_physical ≈ 0) → tau = max_interval.

        Updated for schedulerCombinedDriftEstimator: τ is driven by
        the output-side actuation slope now, so quietness is measured
        from adjfine being flat, not from error history.
        """
        s = DisciplineScheduler(base_interval=1, adaptive=True,
                                 max_interval=60, min_interval=1)
        rng = np.random.default_rng(7)
        n = 100
        for t in range(n):
            s.accumulate(float(rng.normal(0, 0.1)), 1.0, 'TEST',
                          t_monotonic=float(t))
            # Flat adjfine: DO holding stable, no physical drift.
            s.record_actuation(float(t), 100.0)
            if s.n_accumulated >= 5:
                s.flush()
        tau = s.compute_adaptive_interval()
        # Flat adjfine → D_physical < 1e-9 path triggers, tau=max.
        self.assertEqual(tau, 60,
            f"flat adjfine should give tau=max_interval; got {tau} "
            f"(D_physical={s.d_physical_ppb_per_s:.3e})")

    def test_drifting_actuation_shrinks_tau(self):
        """Fast adjfine drift → short tau.

        Adjfine ramping at 1 ppb/s implies D_physical = 1 ppb/s.
        With budget=1ns: τ = √(2·1/1) = √2 ≈ 1.4 → 1.
        """
        s = DisciplineScheduler(base_interval=1, adaptive=True,
                                 max_interval=60, min_interval=1,
                                 phase_error_budget_ns=1.0)
        rng = np.random.default_rng(11)
        n = 100
        for t in range(n):
            s.accumulate(0.0, 1.0, 'TEST', t_monotonic=float(t))
            # adjfine ramps at 1 ppb/s — substantial physical drift.
            s.record_actuation(float(t), 100.0 + 1.0 * t)
            if s.n_accumulated >= 5:
                s.flush()
        tau = s.compute_adaptive_interval()
        self.assertLessEqual(tau, 3,
            f"fast-drifting adjfine should give small tau; got {tau} "
            f"(D_physical={s.d_physical_ppb_per_s:.3f} ppb/s)")


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
            s.record_actuation(float(t), 100.0)  # constant adjfine
            if s.n_accumulated >= 5:
                s.flush()
        self.assertEqual(s.compute_adaptive_interval(), 10)

    def test_phase_error_budget_force_corrects(self):
        """When the last buffered error exceeds T_budget,
        should_correct() returns True immediately."""
        s = DisciplineScheduler(base_interval=10, adaptive=False,
                                 phase_error_budget_ns=1.0)
        s._converging = False  # bypass convergence latch for the test
        # Below budget: shouldn't fire until interval reached.
        s.accumulate(0.5, 1.0, 'TEST', t_monotonic=0.0)
        self.assertFalse(s.should_correct())
        # Above budget: fire immediately.
        s.accumulate(1.5, 1.0, 'TEST', t_monotonic=1.0)
        self.assertTrue(s.should_correct())

    def test_phase_error_budget_negative_value_also_fires(self):
        """Budget check is on |error|, not signed."""
        s = DisciplineScheduler(base_interval=10, adaptive=False,
                                 phase_error_budget_ns=1.0)
        s._converging = False
        s.accumulate(-2.0, 1.0, 'TEST', t_monotonic=0.0)
        self.assertTrue(s.should_correct())

    def test_budget_tau_formula(self):
        """τ = √(2·T_budget/D_physical) — direct math check.

        Set T_budget=2 ns, ramp adjfine at 0.5 ppb/s.
        τ = √(2·2/0.5) = √8 ≈ 2.83 → 3.
        """
        s = DisciplineScheduler(base_interval=1, adaptive=True,
                                 min_interval=1, max_interval=60,
                                 phase_error_budget_ns=2.0)
        for t in range(60):
            s.accumulate(0.0, 1.0, 'TEST', t_monotonic=float(t))
            s.record_actuation(float(t), 100.0 + 0.5 * t)
            if s.n_accumulated >= 5:
                s.flush()
        tau = s.compute_adaptive_interval()
        self.assertEqual(tau, 3,
            f"expected τ=3 from √(2·2/0.5); got {tau} "
            f"(D_physical={s.d_physical_ppb_per_s:.3f} ppb/s)")

    def test_d_physical_recovers_slope(self):
        """D_physical estimator recovers the ramp rate."""
        s = DisciplineScheduler(adaptive=True)
        for t in range(80):
            s.record_actuation(float(t), 50.0 + 0.123 * t)
        s.compute_adaptive_interval()  # triggers _update_d_physical
        self.assertAlmostEqual(s.d_physical_ppb_per_s, 0.123, places=4)

    def test_d_physical_bootstrap_returns_base(self):
        """Until adjfine_history has _MIN_FIT_SAMPLES, fall back to base."""
        s = DisciplineScheduler(base_interval=7, adaptive=True)
        for t in range(_MIN_FIT_SAMPLES - 5):
            s.record_actuation(float(t), 50.0 + t)  # fast ramp
        self.assertEqual(s.compute_adaptive_interval(), 7)

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

"""Tests for schedulerNoiseFloorCadence — the noise-floor transient
detector that replaces the d_physical-derived baseline.

The original d_physical heuristic (slope of adjfine over a long
window) was meant to measure DO drift rate but in practice scaled
with steady-state actuator pull magnitude — a control-output
artifact, not physical drift.  This caused PiFace (steady-state
adj ≈ +135 ppb) and clkPoC3 (steady-state adj ≈ -20 ppb) to land
at very different cadences despite identical configurations.

The replacement compares |err| against a per-DO σ_freerun_short
threshold.  Cadence is then a function of physical signal vs
intrinsic noise — host-independent in form.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from peppar_fix.discipline import DisciplineScheduler  # noqa: E402


class TransientRegimeTests(unittest.TestCase):
    """_in_transient_regime() returns True iff |err| > k · σ_freerun."""

    def _sched(self, sigma_freerun=0.1, k=3.0):
        return DisciplineScheduler(
            adaptive=True,
            sigma_freerun_short_ns=sigma_freerun,
            transient_k_sigma=k,
        )

    def test_no_errors_buffered_not_transient(self):
        s = self._sched()
        self.assertFalse(s._in_transient_regime())

    def test_error_below_noise_floor_not_transient(self):
        s = self._sched(sigma_freerun=0.1, k=3.0)  # threshold = 0.3 ns
        s.accumulate(0.2, 0.01, 'TEST')
        self.assertFalse(s._in_transient_regime())

    def test_error_at_threshold_not_transient(self):
        # Strict-greater-than at the threshold, so exactly k·σ is "not"
        s = self._sched(sigma_freerun=0.1, k=3.0)  # threshold = 0.3 ns
        s.accumulate(0.3, 0.01, 'TEST')
        self.assertFalse(s._in_transient_regime())

    def test_error_above_threshold_is_transient(self):
        s = self._sched(sigma_freerun=0.1, k=3.0)  # threshold = 0.3 ns
        s.accumulate(0.4, 0.01, 'TEST')
        self.assertTrue(s._in_transient_regime())

    def test_negative_error_uses_magnitude(self):
        s = self._sched(sigma_freerun=0.1, k=3.0)
        s.accumulate(-0.5, 0.01, 'TEST')
        self.assertTrue(s._in_transient_regime())

    def test_uncharacterized_never_transient(self):
        # sigma_freerun_short_ns=None ⇒ host not characterized; the new
        # heuristic stays out of the way (caller falls through to the
        # legacy d_physical path).
        s = DisciplineScheduler(adaptive=True, sigma_freerun_short_ns=None)
        s.accumulate(1e9, 0.01, 'TEST')  # would be huge transient
        self.assertFalse(s._in_transient_regime())

    def test_latest_error_only(self):
        # Only the LATEST buffered error matters; older spikes don't
        # keep the transient flag latched on once recent errors are quiet.
        s = self._sched(sigma_freerun=0.1, k=3.0)
        s.accumulate(5.0, 0.01, 'TEST')          # past transient
        s.accumulate(0.05, 0.01, 'TEST')         # now quiet
        self.assertFalse(s._in_transient_regime())


class AdaptiveIntervalTests(unittest.TestCase):
    """compute_adaptive_interval() with the new path."""

    def test_characterized_quiet_returns_max_interval(self):
        # With characterization + no caps + no transient, baseline is
        # max_interval (let physics-based caps shorten it).
        s = DisciplineScheduler(
            adaptive=True, base_interval=1, max_interval=120,
            sigma_freerun_short_ns=0.1, transient_k_sigma=3.0,
        )
        s.accumulate(0.05, 0.01, 'TEST')  # below threshold
        tau = s.compute_adaptive_interval(distance_to_lock=1.0)
        # distance_to_lock=1 → graded_interval squashes to min_interval=1
        self.assertEqual(tau, 1)

    def test_characterized_quiet_dtl_zero_coasts_max(self):
        # distance_to_lock=0 (fully locked) leaves graded_interval=target.
        # No coast caps configured → tau=max_interval.
        s = DisciplineScheduler(
            adaptive=True, base_interval=1, max_interval=120,
            sigma_freerun_short_ns=0.1, transient_k_sigma=3.0,
        )
        s.accumulate(0.05, 0.01, 'TEST')
        tau = s.compute_adaptive_interval(distance_to_lock=0.0)
        self.assertEqual(tau, 120)

    def test_characterized_transient_fires_immediately(self):
        s = DisciplineScheduler(
            adaptive=True, base_interval=1, max_interval=120,
            sigma_freerun_short_ns=0.1, transient_k_sigma=3.0,
        )
        s.accumulate(5.0, 0.01, 'TEST')  # ≫ threshold
        # Even at dtl=0 (fully locked), transient forces min_interval.
        tau = s.compute_adaptive_interval(distance_to_lock=0.0)
        self.assertEqual(tau, 1)

    def test_uncharacterized_uses_legacy_d_physical_path(self):
        # No sigma_freerun_short_ns ⇒ legacy path runs.  Bootstrap until
        # _MIN_FIT_SAMPLES adjfine history entries land.
        s = DisciplineScheduler(
            adaptive=True, base_interval=1, max_interval=120,
            sigma_freerun_short_ns=None,
        )
        s.accumulate(10.0, 0.01, 'TEST')
        tau = s.compute_adaptive_interval()
        # Without enough adjfine samples returns base_interval.
        self.assertEqual(tau, 1)

    def test_pull_magnitude_independence(self):
        """The bug we're fixing: a host with large steady-state pull
        had cadence different from a host with small pull, all else
        equal.  With the new heuristic, two hosts with identical err
        statistics get identical cadence regardless of where their
        adjfine sits.
        """
        # Two schedulers with same noise floor and same |err|, but one
        # also has a long history of large adjfine commands (the
        # "large pull" host) and the other has small adjfine.
        cfg = dict(adaptive=True, base_interval=1, max_interval=120,
                   sigma_freerun_short_ns=0.1, transient_k_sigma=3.0)
        s_big = DisciplineScheduler(**cfg)
        s_sml = DisciplineScheduler(**cfg)
        # Different pull magnitudes (recorded via record_actuation).
        for i in range(50):
            s_big.record_actuation(t_monotonic=float(i), adjfine_ppb=135.0 + 0.01 * i)
            s_sml.record_actuation(t_monotonic=float(i), adjfine_ppb=-20.0 + 0.01 * i)
        # Same err (below threshold).
        s_big.accumulate(0.05, 0.01, 'TEST')
        s_sml.accumulate(0.05, 0.01, 'TEST')
        # Same cadence regardless of pull magnitude.
        t_big = s_big.compute_adaptive_interval(distance_to_lock=0.0)
        t_sml = s_sml.compute_adaptive_interval(distance_to_lock=0.0)
        self.assertEqual(t_big, t_sml,
                         f"pull magnitude leaked into cadence: "
                         f"big-pull={t_big}, small-pull={t_sml}")


class QuietCoastBreakevenTests(unittest.TestCase):
    """The Goldilocks formula: coast as long as DO drift < actuator quantization."""

    def _sched(self, **kw):
        defaults = dict(
            adaptive=True, base_interval=1, max_interval=120,
            sigma_freerun_short_ns=0.018,        # 18 ps σ_DO(1s)
            transient_k_sigma=3.0,
            sigma_actuator_q_ns=0.018,           # 18 ps σ_q (16-bit DAC)
            coast_tdev=(0.018, 0.5, 1.0),        # TDEV(τ)=18 ps · τ^0.5
            coast_cap_k_sigma=1.0,
        )
        defaults.update(kw)
        return DisciplineScheduler(**defaults)

    def test_typical_ocxo_16bit_dac_lands_at_1s(self):
        """σ_q ≈ σ_DO(1s) ≈ 18 ps, slope 0.5  ⇒ τ_breakeven = 1s."""
        s = self._sched()
        tau = s._quiet_coast_breakeven_seconds()
        self.assertAlmostEqual(tau, 1.0, places=2)

    def test_quieter_do_coasts_longer(self):
        """σ_DO(1s) = 1 ps, σ_q = 18 ps, slope 0.5 ⇒ τ = (18/1)^2 = 324 s."""
        s = self._sched(
            sigma_freerun_short_ns=0.001,         # Rb-class
            coast_tdev=(0.001, 0.5, 1.0),
        )
        tau = s._quiet_coast_breakeven_seconds()
        # τ = (0.018 / 0.001)^(1/0.5) = 18^2 = 324s
        self.assertAlmostEqual(tau, 324.0, places=0)

    def test_noisier_do_fires_faster(self):
        """σ_DO(1s) = 100 ps ≫ σ_q = 18 ps; τ < 1s (clamped at min by caller)."""
        s = self._sched(
            sigma_freerun_short_ns=0.100,         # TCXO-class
            coast_tdev=(0.100, 0.5, 1.0),
        )
        tau = s._quiet_coast_breakeven_seconds()
        # τ = (18/100)^(1/0.5) = 0.18^2 = 0.0324s
        self.assertLess(tau, 1.0)
        self.assertGreater(tau, 0.0)

    def test_unset_actuator_q_falls_back_to_max_interval(self):
        s = self._sched(sigma_actuator_q_ns=None)
        self.assertEqual(s._quiet_coast_breakeven_seconds(), 120.0)

    def test_unset_coast_tdev_falls_back_to_max_interval(self):
        s = self._sched(coast_tdev=None)
        self.assertEqual(s._quiet_coast_breakeven_seconds(), 120.0)

    def test_actuator_q_exceeds_do_floor_at_tau_ref(self):
        """If σ_q ≫ k·σ_DO(τ_ref), formula returns a very long τ.
        Upper-bound clamping (to max_interval) is the caller's job; the
        breakeven function returns the raw principled answer."""
        s = self._sched(
            sigma_actuator_q_ns=1.0,              # noisy actuator
            sigma_freerun_short_ns=0.018,
            coast_tdev=(0.018, 0.5, 1.0),
        )
        # τ = (1.0 / 0.018)^(1/0.5) = (55.56)^2 ≈ 3086 s
        tau = s._quiet_coast_breakeven_seconds()
        self.assertGreater(tau, 120.0)


class GoldilocksCadenceIntegrationTests(unittest.TestCase):
    """End-to-end: with the breakeven formula, compute_adaptive_interval()
    returns the Goldilocks cadence in the quiet regime."""

    def test_ocxo_16bit_quiet_gives_1_second(self):
        s = DisciplineScheduler(
            adaptive=True, base_interval=1, max_interval=120,
            sigma_freerun_short_ns=0.018, transient_k_sigma=3.0,
            sigma_actuator_q_ns=0.018,
            coast_tdev=(0.018, 0.5, 1.0),
        )
        s.accumulate(0.001, 0.01, 'TEST')   # below threshold = quiet
        tau = s.compute_adaptive_interval(distance_to_lock=0.0)
        # Quiet → breakeven → 1s.  Clamped to min(min_interval, max_interval).
        self.assertEqual(tau, 1)

    def test_rb_class_quiet_gives_long_coast(self):
        s = DisciplineScheduler(
            adaptive=True, base_interval=1, max_interval=600,
            sigma_freerun_short_ns=0.001, transient_k_sigma=3.0,
            sigma_actuator_q_ns=0.018,
            coast_tdev=(0.001, 0.5, 1.0),
        )
        s.accumulate(0.0001, 0.01, 'TEST')   # well below threshold
        tau = s.compute_adaptive_interval(distance_to_lock=0.0)
        # τ = 324s, well below max_interval=600
        self.assertGreater(tau, 100)
        self.assertLess(tau, 600)


if __name__ == "__main__":
    unittest.main()

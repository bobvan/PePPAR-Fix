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


if __name__ == "__main__":
    unittest.main()

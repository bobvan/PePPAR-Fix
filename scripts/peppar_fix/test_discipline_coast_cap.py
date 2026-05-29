"""Tests for the longTauGnssCoupling coast-cap + graded-taper policy
primitives in discipline.py.

These are pure functions (no DisciplineScheduler state).  They cap the
coast interval by the DO's *stochastic* phase growth — the piece the
deterministic √(2·T/D) budget ignores — and replace the binary
converging/tracking latch with a continuous, derived-metric taper.
Integration into the live scheduler + closed-loop A/B is deferred to
closedLoopServoSim; this is the unit layer.
"""
from __future__ import annotations

import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.discipline import (
    coast_cap_from_tdev,
    coast_cap_from_p22,
    normalized_convergence,
    graded_interval,
)


class CoastCapFromTdevTest(unittest.TestCase):
    """τ_cap = tau_ref·(budget/(k·tdev_ref))**(1/slope) for slope>0."""

    def test_quiet_ocxo_coasts_long(self):
        # PiFace-class freerun: 85 ps @ 1 s, white-FM (slope +0.5).
        # τ_cap = (1.0/0.085)**2 ≈ 138 s — a quiet OCXO can coast.
        cap = coast_cap_from_tdev(t_budget_ns=1.0, tdev_ref_ns=0.085,
                                  tdev_slope=0.5)
        self.assertAlmostEqual(cap, (1.0 / 0.085) ** 2, places=3)
        self.assertGreater(cap, 120.0)

    def test_tcxo_cannot_coast(self):
        # TimeHAT TCXO-class: 1.17 ns @ 1 s, white-FM.  Already near
        # budget at 1 s → τ_cap < 1 s (must correct every epoch).
        cap = coast_cap_from_tdev(t_budget_ns=1.0, tdev_ref_ns=1.17,
                                  tdev_slope=0.5)
        self.assertAlmostEqual(cap, (1.0 / 1.17) ** 2, places=4)
        self.assertLess(cap, 1.0)

    def test_flat_or_negative_slope_no_constraint(self):
        # White/flicker-PM (slope ≤ 0): wander doesn't grow → no cap.
        self.assertEqual(coast_cap_from_tdev(1.0, 0.085, 0.0),
                         float("inf"))
        self.assertEqual(coast_cap_from_tdev(1.0, 0.085, -0.5),
                         float("inf"))

    def test_zero_tdev_ref_no_constraint(self):
        self.assertEqual(coast_cap_from_tdev(1.0, 0.0, 0.5),
                         float("inf"))

    def test_k_sigma_tightens_cap(self):
        # A 3σ confidence factor shrinks the cap by 3**(1/slope)=9x.
        c1 = coast_cap_from_tdev(1.0, 0.085, 0.5, k_sigma=1.0)
        c3 = coast_cap_from_tdev(1.0, 0.085, 0.5, k_sigma=3.0)
        self.assertAlmostEqual(c1 / c3, 9.0, places=3)
        self.assertLess(c3, c1)

    def test_larger_budget_loosens_cap(self):
        small = coast_cap_from_tdev(0.5, 0.085, 0.5)
        large = coast_cap_from_tdev(2.0, 0.085, 0.5)
        self.assertLess(small, large)

    def test_random_walk_fm_steeper(self):
        # RW-FM (slope +1.5) caps shorter than white-FM for the same
        # ref because the wander grows faster.
        wf = coast_cap_from_tdev(1.0, 0.5, 0.5)
        rw = coast_cap_from_tdev(1.0, 0.5, 1.5)
        self.assertLess(rw, wf)


class CoastCapFromP22Test(unittest.TestCase):
    """Largest integer τ with k·√(P22(τ)) ≤ budget, P22 monotone."""

    @staticmethod
    def _rw_p22(sigma_rate_ns_per_sqrt_s):
        # Random-walk phase: var grows linearly, σ(τ)=rate·√τ.
        s2 = sigma_rate_ns_per_sqrt_s ** 2
        return lambda tau: s2 * tau

    def test_quiet_filter_hits_max(self):
        # σ(τ)=0.05·√τ → over budget(1ns) at τ=400 s; cap to max=120.
        cap = coast_cap_from_p22(1.0, self._rw_p22(0.05), max_tau_s=120)
        self.assertEqual(cap, 120)

    def test_moderate_growth_bites(self):
        # σ(τ)=0.2·√τ ≤ 1 ns → τ ≤ 25 s.
        cap = coast_cap_from_p22(1.0, self._rw_p22(0.2), max_tau_s=120)
        self.assertEqual(cap, 25)

    def test_even_shortest_over_budget_returns_min(self):
        # σ(1 s)=5 ns ≫ 1 ns budget → can't honor; floor to min.
        cap = coast_cap_from_p22(1.0, self._rw_p22(5.0),
                                 min_tau_s=1, max_tau_s=120)
        self.assertEqual(cap, 1)

    def test_k_sigma_tightens(self):
        # k=3: 3·0.2·√τ ≤ 1 → 0.6√τ ≤ 1 → τ ≤ 2.77 → 2.
        cap = coast_cap_from_p22(1.0, self._rw_p22(0.2), k_sigma=3.0)
        self.assertEqual(cap, 2)

    def test_boundary_exact(self):
        # σ(τ)=0.1·√τ; at τ=100, σ=1.0 == budget → included (≤).
        cap = coast_cap_from_p22(1.0, self._rw_p22(0.1), max_tau_s=200)
        self.assertEqual(cap, 100)

    def test_freq_rw_integrated_growth(self):
        # Main's correction (qFromCharPerActuator): during coast the
        # phase variance is dominated by the FREQUENCY random-walk
        # integrated into phase, not the phase process term — so
        # Var(φ,τ) = q_f·τ³/3 (σ ∝ τ^1.5), steeper than √τ.  The cap
        # function is agnostic to the growth law (callable), so the
        # physically-correct cubic-variance shape works unchanged.
        q_f = 3.75e-4  # ns²/s³; Var(20)=q_f·8000/3=1.0 == budget²
        cap = coast_cap_from_p22(1.0, lambda tau: q_f * tau ** 3 / 3.0,
                                 max_tau_s=120)
        self.assertEqual(cap, 20)


class NormalizedConvergenceTest(unittest.TestCase):
    """Linear [converged,far] → [0,1] ramp; clamped, monotone."""

    def test_converged_floor(self):
        self.assertEqual(normalized_convergence(0.05, 0.1, 1.0), 0.0)
        self.assertEqual(normalized_convergence(0.1, 0.1, 1.0), 0.0)

    def test_far_ceiling(self):
        self.assertEqual(normalized_convergence(1.0, 0.1, 1.0), 1.0)
        self.assertEqual(normalized_convergence(5.0, 0.1, 1.0), 1.0)

    def test_midpoint(self):
        m = normalized_convergence(0.55, 0.1, 1.0)
        self.assertAlmostEqual(m, 0.5, places=6)

    def test_monotone(self):
        xs = [0.1, 0.3, 0.5, 0.7, 0.9]
        ms = [normalized_convergence(x, 0.1, 1.0) for x in xs]
        self.assertEqual(ms, sorted(ms))

    def test_invalid_range_raises(self):
        with self.assertRaises(ValueError):
            normalized_convergence(0.5, 1.0, 1.0)
        with self.assertRaises(ValueError):
            normalized_convergence(0.5, 1.0, 0.5)


class GradedIntervalTest(unittest.TestCase):
    """Geometric taper: m=0 → target, m=1 → min, monotone, clamped."""

    def test_converged_uses_full_target(self):
        self.assertEqual(graded_interval(120, 0.0, min_interval=1), 120)

    def test_far_uses_min(self):
        self.assertEqual(graded_interval(120, 1.0, min_interval=1), 1)

    def test_geometric_midpoint(self):
        # m=0.5 → round(120·(1/120)**0.5) = round(√120) = 11.
        self.assertEqual(graded_interval(120, 0.5, min_interval=1), 11)

    def test_monotone_non_increasing_in_m(self):
        ms = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
        taus = [graded_interval(120, m, min_interval=1) for m in ms]
        for a, b in zip(taus, taus[1:]):
            self.assertGreaterEqual(a, b)

    def test_clamps_metric(self):
        self.assertEqual(graded_interval(60, -1.0, min_interval=2), 60)
        self.assertEqual(graded_interval(60, 2.0, min_interval=2), 2)

    def test_target_at_or_below_min(self):
        self.assertEqual(graded_interval(1, 0.0, min_interval=1), 1)
        self.assertEqual(graded_interval(1, 1.0, min_interval=5), 5)

    def test_no_cliff_smoothness(self):
        # Adjacent small steps in m never jump more than the latch's
        # full 1→target cliff: the whole point of the taper.
        prev = graded_interval(120, 0.0, min_interval=1)
        worst = 0
        m = 0.0
        while m <= 1.0:
            cur = graded_interval(120, m, min_interval=1)
            worst = max(worst, abs(prev - cur))
            prev = cur
            m += 0.02
        # The binary latch would jump 119 in one step; the taper's
        # largest single 0.02-step jump is far smaller.
        self.assertLess(worst, 30)


class CompositionTest(unittest.TestCase):
    """Intended engine use: τ = graded(min(drift, tdev_cap, p22_cap), m)."""

    def test_quiet_converged_coasts_long(self):
        # Quiet OCXO, fully converged → long coast.
        drift_tau = 120
        tdev_cap = coast_cap_from_tdev(1.0, 0.085, 0.5)        # ~138
        p22_cap = coast_cap_from_p22(
            1.0, lambda tau: (0.05 ** 2) * tau)                # 120 (max)
        target = min(drift_tau, int(min(tdev_cap, 1e9)), p22_cap)
        m = normalized_convergence(0.08, 0.1, 1.0)             # converged
        tau = graded_interval(target, m)
        self.assertEqual(tau, 120)

    def test_far_from_converged_corrects_fast(self):
        # Same quiet DO but far from converged → taper forces tight.
        target = 120
        m = normalized_convergence(0.9, 0.1, 1.0)              # ~0.89
        tau = graded_interval(target, m)
        self.assertLessEqual(tau, 3)

    def test_tcxo_cap_dominates(self):
        # Noisy TCXO: the tdev cap (<1 s) dominates regardless of
        # convergence — the loop must correct every epoch.
        tdev_cap = coast_cap_from_tdev(1.0, 1.17, 0.5)         # ~0.73
        target = max(1, int(min(120, tdev_cap)))               # floors to 1
        self.assertEqual(target, 1)
        self.assertEqual(graded_interval(target, 0.0), 1)


if __name__ == "__main__":
    unittest.main()

"""Tests for assess_coast_cap_safety — the t_budget guardrail that
catches the silent-fail-open regime Charlie flagged in the #93 sim A/B
(--coast-cap on with t_budget > √P22(max_tau): cap predicate never
engages, divergence re-emerges).
"""
from __future__ import annotations

import math
import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.discipline import assess_coast_cap_safety


class P22ReachTest(unittest.TestCase):
    """√P22(max_tau) from σ_phase + σ_freq, ignoring initial P (Q-only)."""

    def test_phase_only_growth_linear(self):
        # σ_freq=0 ⇒ Var(φ,τ)=σ_phase²·τ; √Var=σ_phase·√τ.
        s = assess_coast_cap_safety(
            t_budget_ns=10.0, max_tau_s=100,
            sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.0)
        self.assertAlmostEqual(
            s["p22_sqrt_max_ns"], 0.92 * math.sqrt(100), places=6)

    def test_freq_only_growth_cubic_in_variance(self):
        # σ_phase=0 ⇒ Var(φ,τ)=σ_freq²·τ³/3.
        sigma_f = 0.01
        tau = 120
        s = assess_coast_cap_safety(
            t_budget_ns=10.0, max_tau_s=tau,
            sigma_do_phase_ns=0.0, sigma_do_freq_ppb=sigma_f)
        expected = math.sqrt(sigma_f ** 2 * tau ** 3 / 3.0)
        self.assertAlmostEqual(s["p22_sqrt_max_ns"], expected, places=6)

    def test_combined_growth_sums_variances(self):
        # Both contribute additively in variance space.
        s = assess_coast_cap_safety(
            t_budget_ns=10.0, max_tau_s=120,
            sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01)
        expected = math.sqrt(0.92 ** 2 * 120 + 0.01 ** 2 * 120 ** 3 / 3.0)
        self.assertAlmostEqual(s["p22_sqrt_max_ns"], expected, places=6)


class P22CapActiveTest(unittest.TestCase):
    """k·√P22(max) ≥ margin·t_budget ⇒ cap will engage within max_tau."""

    def test_charlie_failure_regime(self):
        # Charlie's reported divergence: σ_freq=0.01, max=120, budget=20.
        # √P22(120) = √(0.92²·120 + 0.01²·120³/3) ≈ √(101.5 + 57.6) ≈ 12.6 ns
        # 12.6 < 2·20 = 40 ⇒ NOT active.
        s = assess_coast_cap_safety(
            t_budget_ns=20.0, max_tau_s=120,
            sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01)
        self.assertFalse(s["p22_cap_active"])
        self.assertFalse(s["ok"])  # no tdev cap configured either

    def test_default_safe_regime(self):
        # Default t_budget=1 ns; same Q ⇒ 12.6 ≥ 2·1 ⇒ active.
        s = assess_coast_cap_safety(
            t_budget_ns=1.0, max_tau_s=120,
            sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01)
        self.assertTrue(s["p22_cap_active"])
        self.assertTrue(s["ok"])

    def test_zero_budget_inactive(self):
        # Edge case: t_budget=0 ⇒ no cap (and would divide-by-zero
        # advice).  Treated as inactive.
        s = assess_coast_cap_safety(
            t_budget_ns=0.0, max_tau_s=120,
            sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01)
        self.assertFalse(s["p22_cap_active"])

    def test_k_sigma_inflates_reach(self):
        # k=10 multiplies the cap's reach; budget that was inactive at
        # k=1 may become active at k=10.
        s1 = assess_coast_cap_safety(
            t_budget_ns=20.0, max_tau_s=120,
            sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01,
            k_sigma=1.0)
        s10 = assess_coast_cap_safety(
            t_budget_ns=20.0, max_tau_s=120,
            sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01,
            k_sigma=10.0)
        self.assertFalse(s1["p22_cap_active"])
        self.assertTrue(s10["p22_cap_active"])


class TdevCapActiveTest(unittest.TestCase):
    """k·TDEV(max_tau) ≥ margin·t_budget ⇒ TDEV cap will engage."""

    def test_white_fm_ocxo_at_max_tau(self):
        # TDEV ∝ √τ; 0.085 ns @ τ=1 ⇒ at τ=120: 0.085·√120 ≈ 0.93 ns.
        s = assess_coast_cap_safety(
            t_budget_ns=0.1, max_tau_s=120,
            sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01,
            coast_tdev=(0.085, 0.5, 1.0))
        self.assertAlmostEqual(
            s["tdev_at_max_ns"], 0.085 * math.sqrt(120), places=6)
        # 0.93 ≥ 2·0.1 = 0.2 ⇒ active.
        self.assertTrue(s["tdev_cap_active"])

    def test_tcxo_tight_budget_active(self):
        # TCXO 1.17 @ 1 s; budget 1 ns; TDEV(120)≈12.8 ⇒ 12.8 ≥ 2 ⇒ active.
        s = assess_coast_cap_safety(
            t_budget_ns=1.0, max_tau_s=120,
            sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01,
            coast_tdev=(1.17, 0.5, 1.0))
        self.assertTrue(s["tdev_cap_active"])

    def test_flat_slope_no_tdev_cap(self):
        # slope=0 ⇒ TDEV is flat; cap returns +inf; report inactive.
        s = assess_coast_cap_safety(
            t_budget_ns=1.0, max_tau_s=120,
            sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01,
            coast_tdev=(0.085, 0.0, 1.0))
        self.assertIsNone(s["tdev_cap_active"])

    def test_either_cap_active_ok(self):
        # P22 cap inactive, TDEV cap active ⇒ ok overall.
        s = assess_coast_cap_safety(
            t_budget_ns=20.0, max_tau_s=120,
            sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01,
            coast_tdev=(0.085, 1.5, 1.0))  # RWFM ⇒ steep growth
        # tdev(120) = 0.085·120^1.5 ≈ 0.085·1314.5 ≈ 111.7 ⇒ active.
        self.assertFalse(s["p22_cap_active"])
        self.assertTrue(s["tdev_cap_active"])
        self.assertTrue(s["ok"])


class AdviceTest(unittest.TestCase):

    def test_advice_uses_larger_reach(self):
        # advised_max_budget_ns = k·max(p22_sqrt, tdev_at_max)/margin.
        s = assess_coast_cap_safety(
            t_budget_ns=100.0, max_tau_s=120,
            sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01,
            coast_tdev=(0.085, 0.5, 1.0))
        p22 = s["p22_sqrt_max_ns"]
        tdev = s["tdev_at_max_ns"]
        # P22 reach (~12.6) >> TDEV reach (~0.93); advice anchored on P22.
        self.assertAlmostEqual(
            s["advised_max_budget_ns"], max(p22, tdev) / 2.0, places=6)


if __name__ == "__main__":
    unittest.main()

"""Tests for σ-weighted Bayesian ARP blend (I-125649 Stage 2/4).

Locks in the math and the σ_pin floor that prevents the
over-confidence trap discovered 2026-05-09 on MadHat.
"""

from __future__ import annotations

import math
import os
import sys
import unittest

import numpy as np

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.arp_blend import (  # noqa: E402
    bayesian_arp_blend,
    ArpBlendResult,
)


# Lab-realistic test point near the surveyed CHOKE1 ARP so that
# any geographic bug surfaces (not exactly origin or a round number).
_LAB_ECEF = np.array([157469.37, -4756188.96, 4232768.46])


class _AlphaEffMath(unittest.TestCase):

    def test_equal_sigmas_gives_half(self):
        """σ_pin == σ_source ⇒ α_eff = 0.5 (50/50 weighted)."""
        out = bayesian_arp_blend(
            _LAB_ECEF, 0.5,
            _LAB_ECEF + np.array([0.1, 0, 0]), 0.5,
        )
        self.assertEqual(out.action, "applied")
        self.assertAlmostEqual(out.alpha_eff, 0.5, places=6)

    def test_loose_pin_high_confidence_source_takes_full_step(self):
        """σ_pin = 5 m, σ_source = 0.1 m ⇒ α_eff ≈ 1 (trust source)."""
        out = bayesian_arp_blend(
            _LAB_ECEF, 5.0,
            _LAB_ECEF + np.array([0.1, 0, 0]), 0.1,
        )
        self.assertEqual(out.action, "applied")
        # α = 25 / (25 + 0.01) = 0.9996...
        self.assertGreater(out.alpha_eff, 0.999)
        # Step should be near-full delta (0.1 m east)
        self.assertAlmostEqual(out.step_3d_m, 0.1, places=2)

    def test_tight_pin_low_confidence_source_takes_tiny_step(self):
        """σ_pin = 0.01 m (surveyed), σ_source = 1 m ⇒ α_eff ≈ 0."""
        out = bayesian_arp_blend(
            _LAB_ECEF, 0.01,
            _LAB_ECEF + np.array([0.5, 0, 0]), 1.0,
        )
        self.assertEqual(out.action, "applied")
        # α = 1e-4 / (1e-4 + 1) ≈ 0.0001
        self.assertLess(out.alpha_eff, 0.001)
        # Step is near zero — surveyed pin defended
        self.assertLess(out.step_3d_m, 0.001)


class _SigmaPinUpdate(unittest.TestCase):

    def test_floor_at_source_sigma(self):
        """σ_pin posterior is floored at σ_source (default behavior).

        Prevents the over-confidence trap: σ_pin must not shrink
        below σ_source after a single update, since the next
        publication of the source might genuinely differ by
        σ_source-scale and we'd reject it as outlier.
        """
        out = bayesian_arp_blend(
            _LAB_ECEF, 0.5,
            _LAB_ECEF + np.array([0.1, 0, 0]), 0.3,
        )
        # Kalman update: σ_pin' = √(0.25 × 0.09 / (0.25 + 0.09))
        #                       = √(0.0225 / 0.34) ≈ 0.257 m
        # Floor at σ_source = 0.3 m
        self.assertAlmostEqual(out.sigma_pin_new_m, 0.3, places=6)

    def test_no_floor_returns_kalman_value(self):
        """floor_pin_at_source=False returns the raw Kalman update."""
        out = bayesian_arp_blend(
            _LAB_ECEF, 0.5,
            _LAB_ECEF + np.array([0.1, 0, 0]), 0.3,
            floor_pin_at_source=False,
        )
        # σ_pin' = √(0.25 × 0.09 / 0.34) = √0.06618 ≈ 0.2572 m
        expected = math.sqrt(0.25 * 0.09 / 0.34)
        self.assertAlmostEqual(out.sigma_pin_new_m, expected, places=4)
        # The unfloored value must be < σ_source (that's the
        # over-confidence path)
        self.assertLess(out.sigma_pin_new_m, 0.3)

    def test_floor_no_op_when_kalman_already_above_source(self):
        """When σ_pin > σ_source, Kalman update lies above σ_source;
        the floor is a no-op."""
        out = bayesian_arp_blend(
            _LAB_ECEF, 5.0,
            _LAB_ECEF + np.array([0.5, 0, 0]), 0.5,
        )
        # σ_pin' = √(25 × 0.25 / 25.25) ≈ 0.498 m, just below σ_source
        # but √(σ_pin²σ_source² / Σ) → σ_source when σ_pin >> σ_source
        # so floor is approximately equal to source σ here too.
        self.assertGreaterEqual(out.sigma_pin_new_m, 0.5 - 1e-9)

    def test_steady_state_sigma_pin_tracks_source(self):
        """After many updates with constant σ_source, σ_pin → σ_source.

        Validates the architectural decoupling: time filter never
        becomes more confident than the source it's learning from.
        """
        sigma_pin = 5.0
        sigma_source = 0.3
        ecef = _LAB_ECEF.copy()
        for _ in range(100):
            out = bayesian_arp_blend(
                ecef, sigma_pin,
                _LAB_ECEF + np.random.uniform(-0.1, 0.1, 3),
                sigma_source,
            )
            ecef = out.arp_new
            sigma_pin = out.sigma_pin_new_m
        self.assertAlmostEqual(sigma_pin, sigma_source, places=6)


class _OutlierGate(unittest.TestCase):

    def test_within_3sigma_accepted(self):
        out = bayesian_arp_blend(
            _LAB_ECEF, 0.5,
            _LAB_ECEF + np.array([1.0, 0, 0]),  # 1 m delta
            0.5,
        )
        # gate = 3 × (0.5 + 0.5) = 3 m, delta = 1 m, accepted
        self.assertEqual(out.action, "applied")

    def test_beyond_3sigma_rejected(self):
        out = bayesian_arp_blend(
            _LAB_ECEF, 0.5,
            _LAB_ECEF + np.array([5.0, 0, 0]),  # 5 m delta
            0.5,
        )
        # gate = 3 × 1.0 = 3 m, delta = 5 m, rejected
        self.assertEqual(out.action, "rejected_outlier")
        # ARP unchanged
        np.testing.assert_array_equal(out.arp_new, _LAB_ECEF)
        # σ_pin unchanged
        self.assertEqual(out.sigma_pin_new_m, 0.5)

    def test_custom_n_sigma(self):
        # 5σ gate accepts what 3σ would reject
        out_3 = bayesian_arp_blend(
            _LAB_ECEF, 0.5,
            _LAB_ECEF + np.array([3.5, 0, 0]),
            0.5,
        )
        self.assertEqual(out_3.action, "rejected_outlier")
        out_5 = bayesian_arp_blend(
            _LAB_ECEF, 0.5,
            _LAB_ECEF + np.array([3.5, 0, 0]),
            0.5,
            n_sigma_gate=5.0,
        )
        self.assertEqual(out_5.action, "applied")

    def test_disable_gate_with_inf(self):
        out = bayesian_arp_blend(
            _LAB_ECEF, 0.5,
            _LAB_ECEF + np.array([100.0, 0, 0]),  # huge delta
            0.5,
            n_sigma_gate=math.inf,
        )
        self.assertEqual(out.action, "applied")


class _RegimeStartup(unittest.TestCase):
    """The cold-start trajectory: NAV2 seed → progressively tighter."""

    def test_nav2_class_warm_start(self):
        """σ_pin starts at 5 m (NAV2-class), shrinks fast on first
        published source at σ ≈ 1 m, settles toward 1 m."""
        ecef = _LAB_ECEF.copy()
        sigma_pin = 5.0
        sigma_source = 1.0  # NAV2 hAcc

        # First update: big shrink
        out1 = bayesian_arp_blend(
            ecef, sigma_pin,
            _LAB_ECEF + np.array([0.5, 0, 0]),
            sigma_source,
        )
        self.assertEqual(out1.action, "applied")
        self.assertGreater(out1.alpha_eff, 0.95)  # NAV2 dominates
        # Floored at σ_source = 1.0
        self.assertAlmostEqual(out1.sigma_pin_new_m, 1.0, places=6)

        # 10 more updates at σ_source = 1.0 — σ_pin floor stays at 1.0
        for _ in range(10):
            out = bayesian_arp_blend(
                out1.arp_new, out1.sigma_pin_new_m,
                _LAB_ECEF + np.random.uniform(-0.5, 0.5, 3),
                sigma_source,
            )
            out1 = out
        self.assertAlmostEqual(out1.sigma_pin_new_m, 1.0, places=6)


class _Validation(unittest.TestCase):

    def test_wrong_shape_raises(self):
        with self.assertRaises(ValueError):
            bayesian_arp_blend(
                np.array([1.0, 2.0]), 0.5,  # 2D, not 3D
                _LAB_ECEF, 0.5,
            )
        with self.assertRaises(ValueError):
            bayesian_arp_blend(
                _LAB_ECEF, 0.5,
                np.array([1.0, 2.0, 3.0, 4.0]), 0.5,  # 4D
            )

    def test_negative_sigma_raises(self):
        with self.assertRaises(ValueError):
            bayesian_arp_blend(
                _LAB_ECEF, -1.0,
                _LAB_ECEF, 0.5,
            )
        with self.assertRaises(ValueError):
            bayesian_arp_blend(
                _LAB_ECEF, 0.5,
                _LAB_ECEF, -1.0,
            )

    def test_both_sigmas_zero_no_op(self):
        """Degenerate case: σ_pin = σ_source = 0.  Treat as
        perfect-agreement no-op."""
        out = bayesian_arp_blend(
            _LAB_ECEF, 0.0,
            _LAB_ECEF, 0.0,
        )
        self.assertEqual(out.action, "applied")
        np.testing.assert_array_equal(out.arp_new, _LAB_ECEF)
        self.assertEqual(out.sigma_pin_new_m, 0.0)


if __name__ == "__main__":
    unittest.main()

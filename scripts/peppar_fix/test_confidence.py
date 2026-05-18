"""Unit tests for peppar_fix.confidence."""

import math
import unittest

from peppar_fix.confidence import (
    ClockClassPromoter,
    FrequencyConfidence,
    INITIALIZED_DEMOTE_NS,
    INITIALIZED_PROMOTE_NS,
    LOCKED_DEMOTE_NS,
    LOCKED_PROMOTE_NS,
    PhaseConfidence,
    SIGMA_POS_NS_PER_M,
    TotalConfidence,
    compute_frequency_confidence,
    compute_phase_confidence,
    compute_total_confidence,
)


class TestPhaseConfidenceSourcePriority(unittest.TestCase):
    """Best available σ_position source picked, with conservative
    σ_pos / 30 cm geometry factor applied."""

    def _antpos(self, *, active=True, sigma_mean_m=None, sigma_3d_m=None):
        return {
            "active": active,
            "sigma_mean_m": sigma_mean_m,
            "sigma_3d_m": sigma_3d_m,
        }

    def test_antpos_mean_wins_when_armed(self):
        p = compute_phase_confidence(
            antpos_state=self._antpos(sigma_mean_m=0.002, sigma_3d_m=0.015),
            seed_sigma_m=0.10,
            nav2_h_acc_m=1.0,
        )
        self.assertEqual(p.source, "antpos_mean")
        self.assertAlmostEqual(p.pos_sigma_m, 0.002, places=6)
        self.assertAlmostEqual(p.sigma_ns,
                               0.002 * SIGMA_POS_NS_PER_M, places=6)

    def test_antpos_epoch_used_when_unarmed(self):
        p = compute_phase_confidence(
            antpos_state=self._antpos(active=False, sigma_3d_m=2.0),
            seed_sigma_m=0.10,
            nav2_h_acc_m=1.0,
        )
        self.assertEqual(p.source, "antpos_epoch")
        self.assertAlmostEqual(p.pos_sigma_m, 2.0)

    def test_seed_used_when_no_antpos(self):
        p = compute_phase_confidence(
            antpos_state=None, seed_sigma_m=0.087, nav2_h_acc_m=1.0,
        )
        self.assertEqual(p.source, "seed")
        self.assertAlmostEqual(p.pos_sigma_m, 0.087)

    def test_nav2_used_when_only_source(self):
        p = compute_phase_confidence(
            antpos_state=None, seed_sigma_m=None, nav2_h_acc_m=1.4,
        )
        self.assertEqual(p.source, "nav2_hAcc")
        self.assertAlmostEqual(p.sigma_ns,
                               1.4 * SIGMA_POS_NS_PER_M, places=6)

    def test_unknown_when_no_sources(self):
        p = compute_phase_confidence()
        self.assertEqual(p.source, "unknown")
        self.assertTrue(math.isinf(p.sigma_ns))

    def test_antpos_state_present_but_no_sigma_falls_through_to_seed(self):
        p = compute_phase_confidence(
            antpos_state=self._antpos(),  # both sigmas None
            seed_sigma_m=0.05,
        )
        self.assertEqual(p.source, "seed")

    def test_zero_or_negative_sigma_treated_as_unavailable(self):
        # Defensive: don't propagate a nonsensical zero σ as "best".
        p = compute_phase_confidence(
            antpos_state=self._antpos(sigma_mean_m=0.0),
            seed_sigma_m=0.05,
        )
        self.assertEqual(p.source, "seed")

    def test_geometry_factor_value(self):
        # Sanity: 1 m → 3.33 ns to 2 decimals.
        p = compute_phase_confidence(seed_sigma_m=1.0)
        self.assertAlmostEqual(p.sigma_ns, 3.333333, places=4)


class TestFrequencyConfidence(unittest.TestCase):

    def test_dt_rx_sigma_passes_through(self):
        f = compute_frequency_confidence(
            dt_rx_sigma_ns=0.84, scheduler_settled=True)
        self.assertAlmostEqual(f.sigma_ns, 0.84)
        self.assertAlmostEqual(f.dt_rx_sigma_ns, 0.84)
        self.assertTrue(f.scheduler_settled)

    def test_missing_dt_rx_sigma_is_inf(self):
        f = compute_frequency_confidence(dt_rx_sigma_ns=None)
        self.assertTrue(math.isinf(f.sigma_ns))

    def test_negative_dt_rx_sigma_is_inf(self):
        f = compute_frequency_confidence(dt_rx_sigma_ns=-1.0)
        self.assertTrue(math.isinf(f.sigma_ns))


class TestTotalConfidence(unittest.TestCase):

    def test_rss_combination(self):
        p = PhaseConfidence(sigma_ns=3.0, pos_sigma_m=0.9, source="seed")
        f = FrequencyConfidence(sigma_ns=4.0, dt_rx_sigma_ns=4.0,
                                scheduler_settled=False)
        t = compute_total_confidence(p, f)
        self.assertAlmostEqual(t.sigma_ns, 5.0)  # 3-4-5 triangle

    def test_inf_phase_propagates(self):
        p = PhaseConfidence(sigma_ns=float("inf"),
                            pos_sigma_m=float("inf"), source="unknown")
        f = FrequencyConfidence(sigma_ns=1.0, dt_rx_sigma_ns=1.0,
                                scheduler_settled=False)
        self.assertTrue(math.isinf(compute_total_confidence(p, f).sigma_ns))

    def test_inf_freq_propagates(self):
        p = PhaseConfidence(sigma_ns=1.0, pos_sigma_m=0.3, source="seed")
        f = FrequencyConfidence(sigma_ns=float("inf"),
                                dt_rx_sigma_ns=float("inf"),
                                scheduler_settled=False)
        self.assertTrue(math.isinf(compute_total_confidence(p, f).sigma_ns))


class TestClockClassPromoter(unittest.TestCase):
    """Hysteresis behaviour: no transition on a single σ value
    triggers both promote-and-demote, and stable σ values inside the
    dead bands don't flap."""

    def test_initial_freerun(self):
        p = ClockClassPromoter()
        self.assertEqual(p.current_class, "freerun")

    def test_freerun_to_initialized_below_promote(self):
        p = ClockClassPromoter("freerun")
        out = p.evaluate(INITIALIZED_PROMOTE_NS - 0.1)
        self.assertEqual(out, "initialized")
        self.assertEqual(p.current_class, "initialized")

    def test_freerun_to_initialized_at_threshold_is_no_op(self):
        """Threshold is strict-less-than — at exactly the threshold,
        no transition fires."""
        p = ClockClassPromoter("freerun")
        out = p.evaluate(INITIALIZED_PROMOTE_NS)
        self.assertEqual(out, "freerun")

    def test_initialized_to_locked_below_promote(self):
        p = ClockClassPromoter("initialized")
        out = p.evaluate(LOCKED_PROMOTE_NS - 0.1)
        self.assertEqual(out, "locked")

    def test_locked_no_flap_in_dead_band(self):
        """In 20–30 ns dead band, σ wobbling doesn't flap class 6."""
        p = ClockClassPromoter("locked")
        for sigma in (21.0, 25.0, 29.9, 22.0, 28.0):
            self.assertEqual(p.evaluate(sigma), "locked")

    def test_locked_demote_above_threshold(self):
        p = ClockClassPromoter("locked")
        out = p.evaluate(LOCKED_DEMOTE_NS + 0.1)
        self.assertEqual(out, "initialized")

    def test_initialized_no_flap_in_outer_dead_band(self):
        """In 800–1200 ns dead band, σ wobbling doesn't flap class 52
        to/from freerun."""
        p = ClockClassPromoter("initialized")
        for sigma in (801.0, 1000.0, 1199.9, 900.0, 1100.0):
            self.assertEqual(p.evaluate(sigma), "initialized")

    def test_initialized_demote_above_threshold(self):
        p = ClockClassPromoter("initialized")
        out = p.evaluate(INITIALIZED_DEMOTE_NS + 0.1)
        self.assertEqual(out, "freerun")

    def test_inf_sigma_demotes_from_locked_to_initialized(self):
        """Infinity is greater than every finite threshold; expect a
        single demote per evaluate call (not skipping initialized)."""
        p = ClockClassPromoter("locked")
        out = p.evaluate(float("inf"))
        # First call: 6 → 52
        self.assertEqual(out, "initialized")
        # Second call: 52 → 248
        out2 = p.evaluate(float("inf"))
        self.assertEqual(out2, "freerun")

    def test_zero_sigma_promotes_freerun_to_initialized_to_locked(self):
        p = ClockClassPromoter("freerun")
        self.assertEqual(p.evaluate(0.0), "initialized")
        self.assertEqual(p.evaluate(0.0), "locked")

    def test_holdover_is_sticky(self):
        """When externally set to holdover (event-driven), the
        promoter doesn't auto-step out on its own."""
        p = ClockClassPromoter("freerun")
        p.override_class("holdover")
        # σ-driven evaluate is a no-op while in holdover.
        out = p.evaluate(0.5)  # would otherwise promote
        self.assertEqual(out, "holdover")
        # Caller is expected to override_class("freerun") or similar
        # when the event clears, then evaluate() can do its thing.
        p.override_class("freerun")
        self.assertEqual(p.evaluate(0.5), "initialized")

    def test_override_class_state_persists(self):
        p = ClockClassPromoter("locked")
        p.override_class("freerun")
        self.assertEqual(p.current_class, "freerun")


if __name__ == "__main__":
    unittest.main()

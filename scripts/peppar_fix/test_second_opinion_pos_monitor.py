"""Unit tests for SecondOpinionPosMonitor."""
from __future__ import annotations

import unittest

import numpy as np

from peppar_fix.second_opinion_pos_monitor import (
    SecondOpinionPosMonitor,
    select_reset_target,
)


# CHOKE1 surveyed truth (NAD83(2011), 2026-05-11 5-day OPUS mean).
_CHOKE1_ECEF = np.array(
    [157469.3712, -4756188.9612, 4232768.4586], dtype=float)
# NAV2's typical reported ECEF on MadHat — ~4 m horizontal east + ~3.8 m
# vertical low from CHOKE1 truth (wrongIntBasin-charlie 2026-05-11).
_NAV2_BIASED_ECEF = _CHOKE1_ECEF + np.array([2.5, 3.1, -3.5], dtype=float)


class SelectResetTargetTest(unittest.TestCase):
    """select_reset_target chooses pin vs NAV2 correctly."""

    def test_pin_position_with_known_pos_returns_pin(self):
        ecef, label = select_reset_target(
            pin_position=True,
            pin_ecef=_CHOKE1_ECEF,
            nav2_ecef=_NAV2_BIASED_ECEF,
        )
        self.assertEqual(label, "known_pos")
        np.testing.assert_array_equal(ecef, _CHOKE1_ECEF)

    def test_no_pin_returns_nav2(self):
        ecef, label = select_reset_target(
            pin_position=False,
            pin_ecef=None,
            nav2_ecef=_NAV2_BIASED_ECEF,
        )
        self.assertEqual(label, "NAV2")
        np.testing.assert_array_equal(ecef, _NAV2_BIASED_ECEF)

    def test_pin_position_without_pin_ecef_falls_back_to_nav2(self):
        # --pin-position set but no --known-pos: helper falls back to
        # NAV2 as a safe default rather than guessing the pin.
        ecef, label = select_reset_target(
            pin_position=True,
            pin_ecef=None,
            nav2_ecef=_NAV2_BIASED_ECEF,
        )
        self.assertEqual(label, "NAV2")
        np.testing.assert_array_equal(ecef, _NAV2_BIASED_ECEF)

    def test_no_pin_with_stale_pin_ecef_still_returns_nav2(self):
        # Defence-in-depth: caller forgot to clear pin_ecef but didn't
        # set pin_position — we still pick NAV2 (the pin_position flag
        # is the authoritative gate).
        ecef, label = select_reset_target(
            pin_position=False,
            pin_ecef=_CHOKE1_ECEF,
            nav2_ecef=_NAV2_BIASED_ECEF,
        )
        self.assertEqual(label, "NAV2")
        np.testing.assert_array_equal(ecef, _NAV2_BIASED_ECEF)

    def test_returns_ndarray_float(self):
        # Helper coerces to float64 ndarray so downstream code (filter
        # initialize) doesn't have to.
        ecef, _ = select_reset_target(
            pin_position=True,
            pin_ecef=tuple(_CHOKE1_ECEF.tolist()),
            nav2_ecef=tuple(_NAV2_BIASED_ECEF.tolist()),
        )
        self.assertIsInstance(ecef, np.ndarray)
        self.assertEqual(ecef.dtype, np.float64)

    def test_nav2_3d_displacement_irrelevant_to_choice(self):
        # The helper's job is purely about WHICH target to use; the
        # SECOND_OPINION_POS monitor is what decides WHEN to trip.
        # Verify the helper doesn't try to be clever about distance.
        ecef, label = select_reset_target(
            pin_position=True,
            pin_ecef=_CHOKE1_ECEF,
            nav2_ecef=_CHOKE1_ECEF + np.array([100.0, 0.0, 0.0]),
        )
        self.assertEqual(label, "known_pos")
        np.testing.assert_array_equal(ecef, _CHOKE1_ECEF)


class SecondOpinionPosMonitorTest(unittest.TestCase):

    def test_below_threshold_silent(self):
        m = SecondOpinionPosMonitor(threshold_m=5.0, sustained_epochs=3)
        for ep in range(10):
            self.assertIsNone(m.evaluate(ep, 2.0))
        self.assertEqual(m.streak(), 0)

    def test_none_does_not_reset_streak(self):
        """A missing NAV2 opinion shouldn't break the streak — the
        engine sometimes gets sparse opinions and we don't want a
        single missing sample to abort a real divergence."""
        m = SecondOpinionPosMonitor(threshold_m=5.0, sustained_epochs=3)
        m.evaluate(0, 7.0)
        m.evaluate(1, 7.0)
        m.evaluate(2, None)  # no opinion — preserve streak
        self.assertEqual(m.streak(), 2)
        ev = m.evaluate(3, 7.0)
        self.assertIsNotNone(ev)

    def test_below_threshold_clears_streak(self):
        m = SecondOpinionPosMonitor(threshold_m=5.0, sustained_epochs=3)
        m.evaluate(0, 7.0)
        m.evaluate(1, 7.0)
        m.evaluate(2, 1.0)  # cleared
        self.assertEqual(m.streak(), 0)
        # And re-fires after another sustained disagreement
        for ep in range(3, 6):
            m.evaluate(ep, 7.0)
        self.assertEqual(m.streak(), 3)

    def test_fires_at_sustained(self):
        m = SecondOpinionPosMonitor(threshold_m=5.0, sustained_epochs=3)
        self.assertIsNone(m.evaluate(0, 7.0))
        self.assertIsNone(m.evaluate(1, 7.0))
        ev = m.evaluate(2, 7.0)
        self.assertIsNotNone(ev)
        self.assertEqual(ev['reason'], 'second_opinion_3d')
        self.assertAlmostEqual(ev['nav2_delta_3d_m'], 7.0)
        self.assertAlmostEqual(ev['threshold_m'], 5.0)
        self.assertEqual(ev['sustained_epochs'], 3)

    def test_only_fires_once_per_period(self):
        """After firing, don't re-fire until the gate drops below
        threshold (caller acts, then nav2Δ presumably resolves)."""
        m = SecondOpinionPosMonitor(threshold_m=5.0, sustained_epochs=3)
        m.evaluate(0, 7.0)
        m.evaluate(1, 7.0)
        ev = m.evaluate(2, 7.0)
        self.assertIsNotNone(ev)
        # Subsequent above-threshold samples don't re-fire
        for ep in range(3, 20):
            self.assertIsNone(m.evaluate(ep, 7.0))

    def test_recovery_re_arms(self):
        """After note_recovery and one above-threshold sample, the
        streak restarts so a future divergence can fire again."""
        m = SecondOpinionPosMonitor(threshold_m=5.0, sustained_epochs=3)
        m.evaluate(0, 7.0)
        m.evaluate(1, 7.0)
        m.evaluate(2, 7.0)
        m.note_recovery()
        self.assertFalse(m.is_armed())
        # New divergence — fires again
        m.evaluate(10, 7.0)
        m.evaluate(11, 7.0)
        ev = m.evaluate(12, 7.0)
        self.assertIsNotNone(ev)

    def test_negative_displacement_treated_as_magnitude(self):
        m = SecondOpinionPosMonitor(threshold_m=5.0, sustained_epochs=3)
        m.evaluate(0, -7.0)
        m.evaluate(1, -7.0)
        ev = m.evaluate(2, -7.0)
        self.assertIsNotNone(ev)

    # ── hAcc quality gate (workstream C1, dayplan I-115539) ─── #

    def test_hacc_gate_holds_off_when_hacc_too_high(self):
        """Streak hits sustained but rolling-mean hAcc is above the
        gate threshold — should hold off without firing.  Mirrors the
        operational case where NAV2's own opinion is uncertain (multipath
        transient, geometry change), so its disagreement with PPP-AR
        is unreliable evidence of a wrong PPP-AR lock."""
        m = SecondOpinionPosMonitor(
            threshold_m=5.0, sustained_epochs=3,
            hacc_threshold_m=1.5,
        )
        # nav2Δ above threshold + hAcc bad — gate holds off
        for ep in range(5):
            ev = m.evaluate(ep, 7.0, nav2_h_acc_m=3.0)
            self.assertIsNone(ev, f"epoch {ep}: gate should hold off")
        # Streak is still preserved — once NAV2 settles, next call fires
        self.assertGreaterEqual(m.streak(), 3)

    def test_hacc_gate_releases_when_hacc_settles(self):
        """Once rolling-mean hAcc drops below threshold while the
        streak is still above sustained, the next call fires."""
        m = SecondOpinionPosMonitor(
            threshold_m=5.0, sustained_epochs=3,
            hacc_threshold_m=1.5,
            hacc_window_epochs=4,
        )
        # 4 epochs of bad hAcc — gate held off
        for ep in range(4):
            self.assertIsNone(
                m.evaluate(ep, 7.0, nav2_h_acc_m=3.0))
        # Now NAV2 settles — feed 4 epochs of good hAcc to
        # roll the bad ones out of the deque
        for ep in range(4, 7):
            m.evaluate(ep, 7.0, nav2_h_acc_m=0.5)
        ev = m.evaluate(7, 7.0, nav2_h_acc_m=0.5)
        # Some epoch in 4-7 should have fired (depends on rolling mean
        # crossing threshold).  At minimum, by epoch 7 the deque is
        # all good values and the gate should be open.
        self.assertTrue(
            ev is not None or m._fired_this_period,
            "gate never released even after NAV2 settled",
        )

    def test_hacc_gate_disabled_back_compat(self):
        """Setting hacc_threshold_m=None disables the gate.  Calls
        without nav2_h_acc_m work as before — the existing engine
        behavior is preserved for callers that haven't been updated."""
        m = SecondOpinionPosMonitor(
            threshold_m=5.0, sustained_epochs=3,
            hacc_threshold_m=None,
        )
        # No hAcc passed at all — should fire on streak alone
        m.evaluate(0, 7.0)
        m.evaluate(1, 7.0)
        ev = m.evaluate(2, 7.0)
        self.assertIsNotNone(ev)

    def test_hacc_unset_skips_gate(self):
        """If the gate is enabled but the caller never provides hAcc
        (older Nav2PositionStore version, pre-update), the gate is
        skipped — fires on streak alone.  Back-compat for the
        transition window."""
        m = SecondOpinionPosMonitor(
            threshold_m=5.0, sustained_epochs=3,
            hacc_threshold_m=1.5,
        )
        # Caller provides nav2_delta but not nav2_h_acc — gate is
        # in "no samples ever arrived" mode → permissive
        m.evaluate(0, 7.0)
        m.evaluate(1, 7.0)
        ev = m.evaluate(2, 7.0)
        self.assertIsNotNone(ev)

    def test_hacc_history_persists_across_below_threshold(self):
        """hAcc samples accumulate continuously, including epochs
        where nav2Δ is below the trip threshold.  This way when a
        streak does start, the rolling mean already represents NAV2's
        recent quality rather than only the streak window."""
        m = SecondOpinionPosMonitor(
            threshold_m=5.0, sustained_epochs=3,
            hacc_threshold_m=1.5,
            hacc_window_epochs=10,
        )
        # 10 epochs below trip threshold but with bad hAcc
        for ep in range(10):
            m.evaluate(ep, 1.0, nav2_h_acc_m=3.0)
        # Now nav2Δ jumps above threshold — gate should hold off
        # because rolling-mean hAcc is bad
        for ep in range(10, 14):
            ev = m.evaluate(ep, 7.0, nav2_h_acc_m=3.0)
            self.assertIsNone(ev)

    def test_hacc_gate_event_carries_rolling_hacc(self):
        """Trip event includes rolling_hacc_m for diagnostic visibility."""
        m = SecondOpinionPosMonitor(
            threshold_m=5.0, sustained_epochs=3,
            hacc_threshold_m=1.5,
        )
        m.evaluate(0, 7.0, nav2_h_acc_m=0.5)
        m.evaluate(1, 7.0, nav2_h_acc_m=0.7)
        ev = m.evaluate(2, 7.0, nav2_h_acc_m=0.6)
        self.assertIsNotNone(ev)
        self.assertIn('rolling_hacc_m', ev)
        self.assertAlmostEqual(ev['rolling_hacc_m'], 0.6, places=2)

    def test_recovery_clears_hacc_history(self):
        """After note_recovery, hAcc history starts fresh — otherwise
        the post-reset window might still be poisoned by the pre-reset
        bad values that triggered the original event."""
        m = SecondOpinionPosMonitor(
            threshold_m=5.0, sustained_epochs=3,
            hacc_threshold_m=1.5,
        )
        # Pre-reset: build some history
        m.evaluate(0, 7.0, nav2_h_acc_m=3.0)
        m.evaluate(1, 7.0, nav2_h_acc_m=3.0)
        m.note_recovery()
        # History cleared
        self.assertIsNone(m._rolling_hacc())


if __name__ == "__main__":
    unittest.main()

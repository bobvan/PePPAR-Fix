"""Tests for the chi-squared innovation gate on Arm 4 (TICC)."""

import numpy as np
import unittest

from peppar_fix.do_freq_est import DOFreqEst, _CHI2_GATE_THRESHOLD


def _baseline(**kw):
    defaults = dict(
        sigma_ticc_ns=0.060,
        sigma_do_phase_ns=0.92,
        sigma_do_freq_ppb=0.01,
        sigma_tcxo_phase_ns=2.0,
        sigma_tcxo_freq_ppb=0.1,
        initial_freq=0.0,
    )
    defaults.update(kw)
    return DOFreqEst(**defaults)


class TestChi2Gate(unittest.TestCase):

    def test_huge_innov_rejected_when_P_tight(self):
        """After P converges, a 1M ns TICC innovation is rejected — WHEN a
        redundant DO-phase observer (EXTINT) is present to fall back on.
        (soloObserverChiGate: with no redundancy the sole TICC is accepted
        instead — see test_sole_ticc_huge_innov_accepted.)"""
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 0.0, 0.0, 0.0])
        # Tighten P to simulate converged filter
        f.P = np.diag([1.0, 0.01, 1.0, 0.01])
        x2_before = f.x[2]
        f.update(dt=1.0, ticc_diff_ns=1e6, ticc_sigma_ns=0.060,
                 extint_phase_ns=0.0, extint_sigma_ns=10.0)
        self.assertAlmostEqual(f.x[2], x2_before, delta=10.0,
                               msg="TICC rejected when EXTINT redundancy present")
        self.assertIsNotNone(f.last_arm_innov.get('ticc'))

    def test_sole_ticc_huge_innov_accepted(self):
        """soloObserverChiGate: with NO EXTINT (TICC is the SOLE DO-phase
        observer), a large innovation that WOULD chi²-reject is instead
        ACCEPTED — rejecting the sole observer starves x[2] (drift →
        re-acquire cycle → contaminated TDEV)."""
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 0.0, 0.0, 0.0])
        f.P = np.diag([1.0, 0.01, 1.0, 0.01])  # tight → would chi²-reject
        x2_before = f.x[2]
        f.update(dt=1.0, ticc_diff_ns=1000.0, ticc_sigma_ns=0.060)  # no extint
        self.assertNotAlmostEqual(f.x[2], x2_before, places=0,
                                  msg="sole TICC must be accepted, not starved")

    def test_large_innov_accepted_when_P_large(self):
        """During bootstrap (P large), even 500 ns innovation passes."""
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 0.0, -500.0, 0.0])
        # P[2,2] = 5000² from default init → S is huge → χ² is tiny
        x2_before = f.x[2]
        f.update(dt=1.0, ticc_diff_ns=0.0, ticc_sigma_ns=0.060)
        self.assertNotAlmostEqual(f.x[2], x2_before, places=0,
                                  msg="500 ns innov should pass when P is large")

    def test_normal_innov_passes(self):
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 0.0, 0.0, 0.0])
        f.update(dt=1.0, ticc_diff_ns=-44.0, ticc_sigma_ns=0.060)
        self.assertGreater(abs(f.x[2]), 1.0)

    def test_gate_threshold_constant(self):
        self.assertEqual(_CHI2_GATE_THRESHOLD, 100.0)

    def test_innov_monitor_not_fed_on_rejection(self):
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 0.0, 0.0, 0.0])
        f.P = np.diag([1.0, 0.01, 1.0, 0.01])
        n_before = len(f.innov_monitor._innov)
        # redundant EXTINT present → TICC chi²-rejected → monitor not fed
        f.update(dt=1.0, ticc_diff_ns=1e6, ticc_sigma_ns=0.060,
                 extint_phase_ns=0.0, extint_sigma_ns=10.0)
        self.assertEqual(len(f.innov_monitor._innov), n_before)

    def test_convergence_from_500ns_error(self):
        """Cold start with 500 ns error converges — P is large enough
        that the gate doesn't block the corrections."""
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 0.0, -500.0, 0.0])
        for _ in range(15):
            f.update(dt=1.0, ticc_diff_ns=0.0, ticc_sigma_ns=0.060)
        self.assertLess(abs(f.x[2]), 10.0)


class TestStateSanityGuard(unittest.TestCase):

    def test_normal_state_not_flagged(self):
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([100.0, -15.0, -3.0, -135.0])
        f.update(dt=1.0, ticc_diff_ns=-3.0, ticc_sigma_ns=0.060)
        self.assertFalse(f._state_corrupted)

    def test_huge_x0_flagged(self):
        """ekfStateSanityNoRecovery: _state_corrupted is set only after
        consec_threshold sustained violations.  Single-epoch violations
        increment the counter but don't yet trip the engine-side reset
        signal — the chi² gate is allowed to catch transient outliers
        on its own."""
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([2e8, 0.0, 0.0, 0.0])
        for _ in range(f._state_sanity_consec_threshold):
            f.update(dt=1.0)
        self.assertTrue(f._state_corrupted)
        self.assertTrue(f.is_state_corrupted())

    def test_huge_x1_flagged(self):
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([0.0, 20000.0, 0.0, 0.0])
        for _ in range(f._state_sanity_consec_threshold):
            f.update(dt=1.0)
        self.assertTrue(f._state_corrupted)
        self.assertTrue(f.is_state_corrupted())


class TestStateSanityConsecutiveCounter(unittest.TestCase):
    """ekfStateSanityNoRecovery (Main I-065411, 2026-06-03).

    Single-epoch outliers are the chi² gate's responsibility; the
    state-sanity guard catches SUSTAINED corruption that the gate
    didn't pull back from.  These tests pin the consecutive-counter
    semantics that distinguish the two.
    """

    def test_one_violation_does_not_trip_corrupted(self):
        """N=1 violation increments the counter but doesn't yet set
        the engine-side reset signal."""
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([2e8, 0.0, 0.0, 0.0])
        f.update(dt=1.0)
        self.assertEqual(f._state_sanity_consec, 1)
        self.assertFalse(f.is_state_corrupted())

    def test_threshold_consecutive_violations_trip_corrupted(self):
        """At exactly threshold consecutive violations, is_state_corrupted()
        flips to True so the engine's poll sees it."""
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([2e8, 0.0, 0.0, 0.0])
        # threshold-1 violations: not yet corrupted
        for _ in range(f._state_sanity_consec_threshold - 1):
            f.update(dt=1.0)
            self.assertFalse(f.is_state_corrupted())
        # The threshold-th violation: corrupted flips True
        f.update(dt=1.0)
        self.assertTrue(f.is_state_corrupted())

    def test_clean_epoch_resets_consecutive_counter(self):
        """A clean epoch between violations resets the counter — the
        guard fires only on SUSTAINED corruption, not on flickering
        violations across a window."""
        f = _baseline()
        f._need_phc_seed = False
        # Two violations
        f.x = np.array([2e8, 0.0, 0.0, 0.0])
        f.update(dt=1.0)
        f.update(dt=1.0)
        self.assertEqual(f._state_sanity_consec, 2)
        # One clean epoch — state-sanity-check passes
        f.x = np.array([100.0, 0.0, 0.0, 0.0])
        f.update(dt=1.0)
        self.assertEqual(f._state_sanity_consec, 0)
        # Another violation now starts a fresh count
        f.x = np.array([2e8, 0.0, 0.0, 0.0])
        f.update(dt=1.0)
        self.assertEqual(f._state_sanity_consec, 1)
        self.assertFalse(f.is_state_corrupted())

    def test_lqr_short_circuits_on_violation(self):
        """The LQR step is bypassed when state-sanity fires — applying
        the LQR of a corrupt state is exactly what cascaded the
        day0602-pr125b-madhat 22:21 runaway.  update() returns the
        previously-commanded freq unchanged."""
        f = _baseline()
        f._need_phc_seed = False
        # Establish a known last freq from a normal epoch.
        f.x = np.array([0.0, 0.0, 0.0, 0.0])
        f.update(dt=1.0, ticc_diff_ns=0.0, ticc_sigma_ns=0.060)
        prior_freq = f.freq
        # Now corrupt the state.
        f.x = np.array([2e8, 0.0, 0.0, 0.0])
        returned = f.update(dt=1.0)
        # update() returned the unchanged self.freq.
        self.assertEqual(returned, prior_freq)
        self.assertEqual(f.freq, prior_freq)

    def test_reset_clears_corrupted_and_consec(self):
        """servo.reset() — the engine's recovery action — clears both
        the consecutive counter and the corrupted flag so the next
        epoch's check starts fresh."""
        f = _baseline()
        f._need_phc_seed = False
        f.x = np.array([2e8, 0.0, 0.0, 0.0])
        for _ in range(f._state_sanity_consec_threshold):
            f.update(dt=1.0)
        self.assertTrue(f.is_state_corrupted())
        self.assertGreater(f._state_sanity_consec, 0)
        f.reset()
        self.assertFalse(f.is_state_corrupted())
        self.assertEqual(f._state_sanity_consec, 0)

    def test_is_state_corrupted_is_false_on_fresh_filter(self):
        """A newly-constructed filter is never corrupted."""
        f = _baseline()
        self.assertFalse(f.is_state_corrupted())
        self.assertEqual(f._state_sanity_consec, 0)


if __name__ == "__main__":
    unittest.main()

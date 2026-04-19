"""Unit tests for the per-SV state machine (docs/sv-lifecycle-and-pfr-split.md).

Covers every legal edge in `_LEGAL_EDGES` plus one illegal edge per
originating state.  Also exercises the three monitors' eligibility and
threshold behavior.  Pure-python; no hardware, no SSR, no filter.

Run:  python -m unittest scripts.peppar_fix.test_sv_state
or:   PYTHONPATH=scripts python -m unittest peppar_fix.test_sv_state
"""

from __future__ import annotations

import math
import unittest

from peppar_fix.sv_state import SvAmbState, SvStateTracker, InvalidTransition
from peppar_fix.provisional_validator import (
    ProvisionalValidator, elev_weighted_threshold,
)
from peppar_fix.retirement_gate import RetirementGate
from peppar_fix.host_rms_alarm import HostRmsAlarm


class LegalTransitionsTest(unittest.TestCase):
    """Every edge in _LEGAL_EDGES must succeed."""

    def setUp(self):
        self.t = SvStateTracker()

    def _drive(self, sv: str, path):
        """Walk the SV through a chain of states, asserting success."""
        for i, target in enumerate(path):
            self.t.transition(sv, target, epoch=10 + i, reason=f"step{i}")
            self.assertIs(self.t.state(sv), target)

    def test_happy_path_through_retirement(self):
        self._drive("G01", [
            SvAmbState.WL_FIXED,
            SvAmbState.NL_PROVISIONAL,
            SvAmbState.NL_VALIDATED,
            SvAmbState.RETIRING,
            SvAmbState.FLOAT,
        ])

    def test_job_a_rejection_back_to_float(self):
        self._drive("G02", [
            SvAmbState.WL_FIXED,
            SvAmbState.NL_PROVISIONAL,
            SvAmbState.FLOAT,   # Job A demote
        ])

    def test_wl_fixed_back_to_float(self):
        self._drive("G03", [
            SvAmbState.WL_FIXED,
            SvAmbState.FLOAT,   # slip or MW reset
        ])

    def test_provisional_direct_retirement(self):
        # A provisional SV can hit Job B's elev_mask trigger before
        # being validated — legitimate edge.
        self._drive("G04", [
            SvAmbState.WL_FIXED,
            SvAmbState.NL_PROVISIONAL,
            SvAmbState.RETIRING,
        ])

    def test_validated_back_to_float(self):
        self._drive("G05", [
            SvAmbState.WL_FIXED,
            SvAmbState.NL_PROVISIONAL,
            SvAmbState.NL_VALIDATED,
            SvAmbState.FLOAT,   # cycle slip LOW-conf
        ])

    def test_any_nl_to_blacklisted(self):
        self._drive("G06", [
            SvAmbState.WL_FIXED,
            SvAmbState.NL_PROVISIONAL,
            SvAmbState.BLACKLISTED,   # HIGH-conf slip
        ])

    def test_retiring_to_blacklisted(self):
        self._drive("G07", [
            SvAmbState.WL_FIXED,
            SvAmbState.NL_PROVISIONAL,
            SvAmbState.RETIRING,
            SvAmbState.BLACKLISTED,
        ])

    def test_blacklisted_recovery(self):
        self._drive("G08", [
            SvAmbState.BLACKLISTED,
            SvAmbState.FLOAT,
        ])

    def test_float_to_blacklisted_direct(self):
        self._drive("G09", [SvAmbState.BLACKLISTED])

    def test_self_transition_is_noop(self):
        """Repeating the current state should not raise or log."""
        sv = "G10"
        self.t.transition(sv, SvAmbState.WL_FIXED, epoch=1, reason="first")
        # Same state — should be silently accepted (the tracker
        # explicitly handles this).
        self.t.transition(sv, SvAmbState.WL_FIXED, epoch=2, reason="same")
        self.assertIs(self.t.state(sv), SvAmbState.WL_FIXED)


class IllegalTransitionsTest(unittest.TestCase):
    """At least one illegal edge per originating state must raise."""

    def setUp(self):
        self.t = SvStateTracker()

    def _put(self, sv, state):
        """Force SV into a given state for the test setup."""
        self.t.get(sv).state = state

    def test_float_cannot_skip_to_nl(self):
        self._put("X01", SvAmbState.FLOAT)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X01", SvAmbState.NL_PROVISIONAL, epoch=1)

    def test_float_cannot_retire(self):
        self._put("X02", SvAmbState.FLOAT)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X02", SvAmbState.RETIRING, epoch=1)

    def test_wl_fixed_cannot_jump_to_validated(self):
        self._put("X03", SvAmbState.WL_FIXED)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X03", SvAmbState.NL_VALIDATED, epoch=1)

    def test_provisional_cannot_rewind_to_wl(self):
        self._put("X04", SvAmbState.NL_PROVISIONAL)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X04", SvAmbState.WL_FIXED, epoch=1)

    def test_validated_cannot_rewind_to_provisional(self):
        self._put("X05", SvAmbState.NL_VALIDATED)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X05", SvAmbState.NL_PROVISIONAL, epoch=1)

    def test_retiring_cannot_become_validated_again(self):
        self._put("X06", SvAmbState.RETIRING)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X06", SvAmbState.NL_VALIDATED, epoch=1)

    def test_blacklisted_cannot_skip_to_wl(self):
        self._put("X07", SvAmbState.BLACKLISTED)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X07", SvAmbState.WL_FIXED, epoch=1)


class ElevThresholdTest(unittest.TestCase):
    """The elev-weighting formula is self-consistent."""

    def test_at_or_above_clamp_returns_base(self):
        self.assertAlmostEqual(elev_weighted_threshold(2.0, 45.0), 2.0)
        self.assertAlmostEqual(elev_weighted_threshold(2.0, 90.0), 2.0)

    def test_below_clamp_inflates(self):
        # 1/sin(45) / 1/sin(30) = sin(30)/sin(45) = 0.5/0.707 ≈ 0.707
        # Inverse: threshold at 30° = 2.0 * (sin(45)/sin(30))
        #                           = 2.0 * sqrt(2) ≈ 2.828
        self.assertAlmostEqual(
            elev_weighted_threshold(2.0, 30.0), 2.0 * math.sqrt(2), places=3,
        )

    def test_none_elev_returns_base(self):
        # Preserves the "no elev info, trust base" contract.
        self.assertEqual(elev_weighted_threshold(3.0, None), 3.0)


class ProvisionalValidatorTest(unittest.TestCase):
    """Job A fires only on NL_PROVISIONAL SVs, at eval epochs, above threshold."""

    def setUp(self):
        self.t = SvStateTracker()
        self.v = ProvisionalValidator(
            self.t, base_threshold_m=2.0, min_samples=5, eval_every=10,
        )

    def _to_provisional(self, sv):
        self.t.transition(sv, SvAmbState.WL_FIXED, epoch=1)
        self.t.transition(sv, SvAmbState.NL_PROVISIONAL, epoch=2)

    def test_ignores_svs_not_in_provisional(self):
        # SV in FLOAT — should be ignored by Job A.
        labels = [("G01", 'pr', 45.0)]
        self.v.ingest(10, [5.0], labels)
        events = self.v.evaluate(10)
        self.assertEqual(events, [])

    def test_fires_on_sustained_exceed(self):
        self._to_provisional("G02")
        labels = [("G02", 'pr', 90.0)]   # zenith → base threshold 2.0 m
        for e in range(3, 13):
            self.v.ingest(e, [3.0], labels)   # mean 3.0 m > 2.0 m
        events = self.v.evaluate(10)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['sv'], "G02")
        self.assertAlmostEqual(events[0]['mean_resid_m'], 3.0)
        # Tracker moved the SV back to FLOAT.
        self.assertIs(self.t.state("G02"), SvAmbState.FLOAT)

    def test_does_not_fire_below_threshold(self):
        self._to_provisional("G03")
        labels = [("G03", 'pr', 90.0)]
        for e in range(3, 13):
            self.v.ingest(e, [1.5], labels)
        events = self.v.evaluate(10)
        self.assertEqual(events, [])

    def test_does_not_fire_on_non_eval_epochs(self):
        self._to_provisional("G04")
        labels = [("G04", 'pr', 90.0)]
        for e in range(3, 12):
            self.v.ingest(e, [5.0], labels)
        # eval_every=10, so epoch 7 is not an eval moment.
        self.assertEqual(self.v.evaluate(7), [])

    def test_elev_weighting_raises_bar(self):
        self._to_provisional("G05")
        # elev=25° inflates threshold by sin(45)/sin(25) ≈ 1.67 → 3.35 m.
        # A 3 m mean passes the inflated bar.
        labels = [("G05", 'pr', 25.0)]
        for e in range(3, 13):
            self.v.ingest(e, [3.0], labels)
        self.assertEqual(self.v.evaluate(10), [])


class RetirementGateTest(unittest.TestCase):
    """Job B fires on elev-below-mask or elev-weighted residual exceed."""

    def setUp(self):
        self.t = SvStateTracker()
        self.g = RetirementGate(
            self.t,
            base_threshold_m=3.0,
            retirement_mask_deg=18.0,
            min_samples=5,
            eval_every=10,
        )

    def _to_provisional(self, sv):
        self.t.transition(sv, SvAmbState.WL_FIXED, epoch=1)
        self.t.transition(sv, SvAmbState.NL_PROVISIONAL, epoch=2)

    def test_retires_on_elev_below_mask(self):
        self._to_provisional("G10")
        labels = [("G10", 'pr', 15.0)]   # below 18° mask
        for e in range(3, 13):
            self.g.ingest(e, [0.1], labels)   # residual is fine
        events = self.g.evaluate(10)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['reason'], 'elev_mask')
        self.assertIs(self.t.state("G10"), SvAmbState.RETIRING)

    def test_retires_on_elev_weighted_resid(self):
        self._to_provisional("G11")
        labels = [("G11", 'pr', 30.0)]   # above mask
        # elev-weighted base at 30° = 3.0 * sin(45)/sin(30) ≈ 4.243 m.
        # Mean 5 m exceeds it.
        for e in range(3, 13):
            self.g.ingest(e, [5.0], labels)
        events = self.g.evaluate(10)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['reason'], 'elev_weighted_resid')
        self.assertIs(self.t.state("G11"), SvAmbState.RETIRING)

    def test_ignores_non_nl_svs(self):
        # SV in FLOAT — not eligible.
        labels = [("G12", 'pr', 10.0)]
        for e in range(3, 13):
            self.g.ingest(e, [5.0], labels)
        self.assertEqual(self.g.evaluate(10), [])


class HostRmsAlarmTest(unittest.TestCase):
    """Host alarm requires RMS threshold sustained, no recent Job A/B, no cooldown."""

    def setUp(self):
        self.t = SvStateTracker()
        self.alarm = HostRmsAlarm(
            self.t,
            rms_threshold_m=5.0,
            min_samples_in_window=3,
            eval_every=10,
            cooldown_epochs=60,
            suppress_if_jobs_fired_within=60,
        )

    def _provision(self, sv):
        self.t.transition(sv, SvAmbState.WL_FIXED, epoch=1)
        self.t.transition(sv, SvAmbState.NL_PROVISIONAL, epoch=2)

    def test_fires_when_sustained_rms_exceeds(self):
        self._provision("G20")
        self._provision("G21")
        labels = [("G20", 'pr', 45.0), ("G21", 'pr', 45.0)]
        for e in range(3, 13):
            self.alarm.ingest(e, [6.0, 7.0], labels)
        ev = self.alarm.evaluate(10)
        self.assertIsNotNone(ev)
        self.assertGreater(ev['window_rms_m'], 5.0)

    def test_suppressed_by_recent_job_ab_transition(self):
        self._provision("G22")
        # Simulate Job A / Job B just moved an SV to FLOAT.
        self.t.transition("G22", SvAmbState.FLOAT, epoch=5,
                          reason="job_a:synthetic")
        labels = [("G22", 'pr', 45.0)]
        # Put SV back in NL so its residuals count.
        self._provision("G23")
        for e in range(6, 13):
            self.alarm.ingest(e, [6.0], [("G23", 'pr', 45.0)])
        # Epoch 10 is within suppress_if_jobs_fired_within=60 of G22's
        # FLOAT transition at epoch 5 — alarm stays silent.
        self.assertIsNone(self.alarm.evaluate(10))

    def test_respects_cooldown(self):
        self._provision("G24")
        labels = [("G24", 'pr', 45.0)]
        for e in range(3, 13):
            self.alarm.ingest(e, [6.0], labels)
        ev1 = self.alarm.evaluate(10)
        self.assertIsNotNone(ev1)
        self.alarm.record_fire(10)
        # Immediate re-check must be suppressed by cooldown.
        for e in range(11, 21):
            self.alarm.ingest(e, [6.0], labels)
        self.assertIsNone(self.alarm.evaluate(20))


if __name__ == "__main__":
    unittest.main()

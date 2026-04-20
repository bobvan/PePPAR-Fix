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
from peppar_fix.false_fix_monitor import (
    FalseFixMonitor, elev_weighted_threshold,
)
from peppar_fix.setting_sv_drop_monitor import SettingSvDropMonitor
from peppar_fix.fix_set_integrity_alarm import FixSetIntegrityAlarm


class LegalTransitionsTest(unittest.TestCase):
    """Every edge in _LEGAL_EDGES must succeed."""

    def setUp(self):
        self.t = SvStateTracker()

    def _drive(self, sv: str, path):
        """Walk the SV through a chain of states, asserting success."""
        for i, target in enumerate(path):
            self.t.transition(sv, target, epoch=10 + i, reason=f"step{i}")
            self.assertIs(self.t.state(sv), target)

    def test_happy_path_with_promotion(self):
        self._drive("G01", [
            SvAmbState.FLOAT,
            SvAmbState.WL_FIXED,
            SvAmbState.NL_SHORT_FIXED,
            SvAmbState.NL_LONG_FIXED,
        ])

    def test_tracking_admit_to_float(self):
        # Fresh record defaults to TRACKING; admit transitions to FLOAT.
        self._drive("G02", [SvAmbState.FLOAT])
        # After admit, state is FLOAT.
        self.assertIs(self.t.state("G02"), SvAmbState.FLOAT)

    def test_false_fix_rejection_short_to_float(self):
        self._drive("G03", [
            SvAmbState.FLOAT,
            SvAmbState.WL_FIXED,
            SvAmbState.NL_SHORT_FIXED,
            SvAmbState.FLOAT,   # false-fix rejection
        ])

    def test_setting_sv_drop_long_to_float(self):
        self._drive("G04", [
            SvAmbState.FLOAT,
            SvAmbState.WL_FIXED,
            SvAmbState.NL_SHORT_FIXED,
            SvAmbState.NL_LONG_FIXED,
            SvAmbState.FLOAT,   # setting-SV drop
        ])

    def test_wl_fixed_back_to_float(self):
        self._drive("G05", [
            SvAmbState.FLOAT,
            SvAmbState.WL_FIXED,
            SvAmbState.FLOAT,   # slip LOW or MW reset
        ])

    def test_nl_states_to_squelched(self):
        # HIGH-conf slip from either NL state goes to SQUELCHED.
        self._drive("G06", [
            SvAmbState.FLOAT,
            SvAmbState.WL_FIXED,
            SvAmbState.NL_SHORT_FIXED,
            SvAmbState.SQUELCHED,
        ])
        self._drive("G07", [
            SvAmbState.FLOAT,
            SvAmbState.WL_FIXED,
            SvAmbState.NL_SHORT_FIXED,
            SvAmbState.NL_LONG_FIXED,
            SvAmbState.SQUELCHED,
        ])

    def test_squelched_cooldown_recovery(self):
        self._drive("G08", [
            SvAmbState.FLOAT,
            SvAmbState.SQUELCHED,   # slip from FLOAT (MW-only phase)
            SvAmbState.FLOAT,       # cooldown expired
        ])

    def test_float_to_squelched_direct(self):
        self._drive("G09", [SvAmbState.FLOAT, SvAmbState.SQUELCHED])

    def test_wl_fixed_to_squelched(self):
        self._drive("G10", [
            SvAmbState.FLOAT,
            SvAmbState.WL_FIXED,
            SvAmbState.SQUELCHED,
        ])

    def test_self_transition_is_noop(self):
        """Repeating the current state should not raise or log."""
        sv = "G11"
        self.t.transition(sv, SvAmbState.FLOAT, epoch=1, reason="first")
        self.t.transition(sv, SvAmbState.WL_FIXED, epoch=2, reason="wl")
        # Same state — should be silently accepted.
        self.t.transition(sv, SvAmbState.WL_FIXED, epoch=3, reason="same")
        self.assertIs(self.t.state(sv), SvAmbState.WL_FIXED)


class IllegalTransitionsTest(unittest.TestCase):
    """At least one illegal edge per originating state must raise."""

    def setUp(self):
        self.t = SvStateTracker()

    def _put(self, sv, state):
        """Force SV into a given state for the test setup."""
        self.t.get(sv).state = state

    def test_tracking_cannot_skip_to_wl(self):
        self._put("X01", SvAmbState.TRACKING)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X01", SvAmbState.WL_FIXED, epoch=1)

    def test_tracking_cannot_squelch(self):
        # SVs that haven't been admitted can't be squelched — there's
        # no integer state to protect.
        self._put("X02", SvAmbState.TRACKING)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X02", SvAmbState.SQUELCHED, epoch=1)

    def test_float_cannot_skip_to_nl(self):
        self._put("X03", SvAmbState.FLOAT)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X03", SvAmbState.NL_SHORT_FIXED, epoch=1)

    def test_wl_fixed_cannot_jump_to_long(self):
        self._put("X04", SvAmbState.WL_FIXED)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X04", SvAmbState.NL_LONG_FIXED, epoch=1)

    def test_short_cannot_rewind_to_wl(self):
        self._put("X05", SvAmbState.NL_SHORT_FIXED)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X05", SvAmbState.WL_FIXED, epoch=1)

    def test_long_cannot_rewind_to_short(self):
        self._put("X06", SvAmbState.NL_LONG_FIXED)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X06", SvAmbState.NL_SHORT_FIXED, epoch=1)

    def test_squelched_cannot_skip_to_wl(self):
        self._put("X07", SvAmbState.SQUELCHED)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X07", SvAmbState.WL_FIXED, epoch=1)


class FixSetMembershipTest(unittest.TestCase):
    """SvStateTracker membership helpers match the spec."""

    def setUp(self):
        self.t = SvStateTracker()

    def test_short_and_long_term_counts(self):
        # Two in short-term, one in long-term, one unfixed.
        for sv, state in [
            ("G01", SvAmbState.NL_SHORT_FIXED),
            ("G02", SvAmbState.NL_SHORT_FIXED),
            ("G03", SvAmbState.NL_LONG_FIXED),
            ("G04", SvAmbState.FLOAT),
        ]:
            self.t.get(sv).state = state
        self.assertEqual(len(self.t.short_term_members()), 2)
        self.assertEqual(len(self.t.long_term_members()), 1)
        self.assertEqual(len(self.t.fix_set_members()), 3)


class ElevThresholdTest(unittest.TestCase):
    """The elev-weighting formula is self-consistent."""

    def test_at_or_above_clamp_returns_base(self):
        self.assertAlmostEqual(elev_weighted_threshold(2.0, 45.0), 2.0)
        self.assertAlmostEqual(elev_weighted_threshold(2.0, 90.0), 2.0)

    def test_below_clamp_inflates(self):
        # threshold at 30° = 2.0 * (sin(45)/sin(30)) = 2.0 * sqrt(2) ≈ 2.828
        self.assertAlmostEqual(
            elev_weighted_threshold(2.0, 30.0), 2.0 * math.sqrt(2), places=3,
        )

    def test_none_elev_returns_base(self):
        self.assertEqual(elev_weighted_threshold(3.0, None), 3.0)


class FalseFixMonitorTest(unittest.TestCase):
    """False-fix monitor fires only on NL_SHORT_FIXED SVs, at eval epochs, above threshold."""

    def setUp(self):
        self.t = SvStateTracker()
        self.m = FalseFixMonitor(
            self.t, base_threshold_m=2.0, min_samples=5, eval_every=10,
        )

    def _to_short(self, sv):
        self.t.transition(sv, SvAmbState.FLOAT, epoch=0)
        self.t.transition(sv, SvAmbState.WL_FIXED, epoch=1)
        self.t.transition(sv, SvAmbState.NL_SHORT_FIXED, epoch=2)

    def test_ignores_svs_not_in_short(self):
        # SV in FLOAT — should be ignored by the false-fix monitor.
        self.t.transition("G01", SvAmbState.FLOAT, epoch=0)
        labels = [("G01", 'pr', 45.0)]
        self.m.ingest(10, [5.0], labels)
        events = self.m.evaluate(10)
        self.assertEqual(events, [])

    def test_fires_on_sustained_exceed(self):
        self._to_short("G02")
        labels = [("G02", 'pr', 90.0)]   # zenith → base threshold 2.0 m
        for e in range(3, 13):
            self.m.ingest(e, [3.0], labels)   # mean 3.0 m > 2.0 m
        events = self.m.evaluate(10)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['sv'], "G02")
        self.assertAlmostEqual(events[0]['mean_resid_m'], 3.0)
        # Tracker moved the SV back to FLOAT.
        self.assertIs(self.t.state("G02"), SvAmbState.FLOAT)

    def test_does_not_fire_below_threshold(self):
        self._to_short("G03")
        labels = [("G03", 'pr', 90.0)]
        for e in range(3, 13):
            self.m.ingest(e, [1.5], labels)
        events = self.m.evaluate(10)
        self.assertEqual(events, [])

    def test_does_not_fire_on_non_eval_epochs(self):
        self._to_short("G04")
        labels = [("G04", 'pr', 90.0)]
        for e in range(3, 12):
            self.m.ingest(e, [5.0], labels)
        # eval_every=10, so epoch 7 is not an eval moment.
        self.assertEqual(self.m.evaluate(7), [])

    def test_elev_weighting_raises_bar(self):
        self._to_short("G05")
        # elev=25° inflates threshold by sin(45)/sin(25) ≈ 1.67 → 3.35 m.
        # A 3 m mean passes the inflated bar.
        labels = [("G05", 'pr', 25.0)]
        for e in range(3, 13):
            self.m.ingest(e, [3.0], labels)
        self.assertEqual(self.m.evaluate(10), [])


class SettingSvDropMonitorTest(unittest.TestCase):
    """Setting-SV drop fires on elev-below-mask or elev-weighted residual exceed."""

    def setUp(self):
        self.t = SvStateTracker()
        self.m = SettingSvDropMonitor(
            self.t,
            base_threshold_m=3.0,
            drop_mask_deg=18.0,
            min_samples=5,
            eval_every=10,
        )

    def _to_short(self, sv):
        self.t.transition(sv, SvAmbState.FLOAT, epoch=0)
        self.t.transition(sv, SvAmbState.WL_FIXED, epoch=1)
        self.t.transition(sv, SvAmbState.NL_SHORT_FIXED, epoch=2)

    def test_drops_on_elev_below_mask(self):
        self._to_short("G10")
        labels = [("G10", 'pr', 15.0)]   # below 18° mask
        for e in range(3, 13):
            self.m.ingest(e, [0.1], labels)   # residual is fine
        events = self.m.evaluate(10)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['reason'], 'elev_mask')
        # Now transitions straight back to FLOAT (no RETIRING in-between).
        self.assertIs(self.t.state("G10"), SvAmbState.FLOAT)

    def test_drops_on_elev_weighted_resid(self):
        self._to_short("G11")
        labels = [("G11", 'pr', 30.0)]   # above mask
        # elev-weighted base at 30° = 3.0 * sin(45)/sin(30) ≈ 4.243 m.
        # Mean 5 m exceeds it.
        for e in range(3, 13):
            self.m.ingest(e, [5.0], labels)
        events = self.m.evaluate(10)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['reason'], 'elev_weighted_resid')
        self.assertIs(self.t.state("G11"), SvAmbState.FLOAT)

    def test_ignores_non_nl_svs(self):
        # SV in FLOAT — not eligible.
        self.t.transition("G12", SvAmbState.FLOAT, epoch=0)
        labels = [("G12", 'pr', 10.0)]
        for e in range(3, 13):
            self.m.ingest(e, [5.0], labels)
        self.assertEqual(self.m.evaluate(10), [])


class FixSetIntegrityAlarmTest(unittest.TestCase):
    """Alarm requires RMS threshold sustained, no recent per-SV monitor, no cooldown."""

    def setUp(self):
        self.t = SvStateTracker()
        self.alarm = FixSetIntegrityAlarm(
            self.t,
            rms_threshold_m=5.0,
            min_samples_in_window=3,
            eval_every=10,
            cooldown_epochs=60,
            suppress_if_monitors_fired_within=60,
        )

    def _to_short(self, sv):
        self.t.transition(sv, SvAmbState.FLOAT, epoch=0)
        self.t.transition(sv, SvAmbState.WL_FIXED, epoch=1)
        self.t.transition(sv, SvAmbState.NL_SHORT_FIXED, epoch=2)

    def test_fires_when_sustained_rms_exceeds(self):
        self._to_short("G20")
        self._to_short("G21")
        labels = [("G20", 'pr', 45.0), ("G21", 'pr', 45.0)]
        for e in range(3, 13):
            self.alarm.ingest(e, [6.0, 7.0], labels)
        ev = self.alarm.evaluate(10)
        self.assertIsNotNone(ev)
        self.assertGreater(ev['window_rms_m'], 5.0)

    def test_suppressed_by_recent_per_sv_transition(self):
        self._to_short("G22")
        # Simulate a false-fix monitor just moved an SV to FLOAT.
        self.t.transition("G22", SvAmbState.FLOAT, epoch=5,
                          reason="false_fix:synthetic")
        # Put another SV in short-term so its residuals count.
        self._to_short("G23")
        for e in range(6, 13):
            self.alarm.ingest(e, [6.0], [("G23", 'pr', 45.0)])
        # Epoch 10 is within suppress_if_monitors_fired_within=60 of
        # G22's FLOAT transition at epoch 5 — alarm stays silent.
        self.assertIsNone(self.alarm.evaluate(10))

    def test_respects_cooldown(self):
        self._to_short("G24")
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

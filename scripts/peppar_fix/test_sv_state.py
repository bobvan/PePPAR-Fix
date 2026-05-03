"""Unit tests for the per-SV state machine (docs/sv-lifecycle-and-pfr-split.md).

Covers every legal edge in `_LEGAL_EDGES` plus one illegal edge per
originating state.  Also exercises the three monitors' eligibility and
threshold behavior.  Pure-python; no hardware, no SSR, no filter.

Run:  python -m unittest scripts.peppar_fix.test_sv_state
or:   PYTHONPATH=scripts python -m unittest peppar_fix.test_sv_state
"""

from __future__ import annotations

import logging
import math
import unittest

from peppar_fix.sv_state import SvAmbState, SvStateTracker, InvalidTransition
from peppar_fix.false_fix_monitor import (
    FalseFixMonitor, elev_weighted_threshold,
)
from peppar_fix.setting_sv_drop_monitor import SettingSvDropMonitor
from peppar_fix.fix_set_integrity_monitor import FixSetIntegrityMonitor


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
            SvAmbState.FLOATING,
            SvAmbState.CONVERGING,
            SvAmbState.ANCHORING,
            SvAmbState.ANCHORED,
        ])

    def test_tracking_admit_to_float(self):
        # Fresh record defaults to TRACKING; admit transitions to FLOATING.
        self._drive("G02", [SvAmbState.FLOATING])
        # After admit, state is FLOATING.
        self.assertIs(self.t.state("G02"), SvAmbState.FLOATING)

    def test_false_fix_rejection_short_to_float(self):
        self._drive("G03", [
            SvAmbState.FLOATING,
            SvAmbState.CONVERGING,
            SvAmbState.ANCHORING,
            SvAmbState.FLOATING,   # false-fix rejection
        ])

    def test_setting_sv_drop_long_to_float(self):
        self._drive("G04", [
            SvAmbState.FLOATING,
            SvAmbState.CONVERGING,
            SvAmbState.ANCHORING,
            SvAmbState.ANCHORED,
            SvAmbState.FLOATING,   # setting-SV drop
        ])

    def test_wl_fixed_back_to_float(self):
        self._drive("G05", [
            SvAmbState.FLOATING,
            SvAmbState.CONVERGING,
            SvAmbState.FLOATING,   # slip LOW or MW reset
        ])

    def test_nl_states_to_squelched(self):
        # HIGH-conf slip from either NL state goes to WAITING.
        self._drive("G06", [
            SvAmbState.FLOATING,
            SvAmbState.CONVERGING,
            SvAmbState.ANCHORING,
            SvAmbState.WAITING,
        ])
        self._drive("G07", [
            SvAmbState.FLOATING,
            SvAmbState.CONVERGING,
            SvAmbState.ANCHORING,
            SvAmbState.ANCHORED,
            SvAmbState.WAITING,
        ])

    def test_squelched_cooldown_recovery(self):
        self._drive("G08", [
            SvAmbState.FLOATING,
            SvAmbState.WAITING,   # slip from FLOATING (MW-only phase)
            SvAmbState.FLOATING,       # cooldown expired
        ])

    def test_float_to_squelched_direct(self):
        self._drive("G09", [SvAmbState.FLOATING, SvAmbState.WAITING])

    def test_wl_fixed_to_squelched(self):
        self._drive("G10", [
            SvAmbState.FLOATING,
            SvAmbState.CONVERGING,
            SvAmbState.WAITING,
        ])

    def test_self_transition_is_noop(self):
        """Repeating the current state should not raise or log."""
        sv = "G11"
        self.t.transition(sv, SvAmbState.FLOATING, epoch=1, reason="first")
        self.t.transition(sv, SvAmbState.CONVERGING, epoch=2, reason="wl")
        # Same state — should be silently accepted.
        self.t.transition(sv, SvAmbState.CONVERGING, epoch=3, reason="same")
        self.assertIs(self.t.state(sv), SvAmbState.CONVERGING)


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
            self.t.transition("X01", SvAmbState.CONVERGING, epoch=1)

    def test_tracking_cannot_squelch(self):
        # SVs that haven't been admitted can't be squelched — there's
        # no integer state to protect.
        self._put("X02", SvAmbState.TRACKING)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X02", SvAmbState.WAITING, epoch=1)

    def test_float_cannot_skip_to_nl(self):
        self._put("X03", SvAmbState.FLOATING)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X03", SvAmbState.ANCHORING, epoch=1)

    def test_wl_fixed_cannot_jump_to_long(self):
        self._put("X04", SvAmbState.CONVERGING)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X04", SvAmbState.ANCHORED, epoch=1)

    def test_short_cannot_rewind_to_wl(self):
        self._put("X05", SvAmbState.ANCHORING)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X05", SvAmbState.CONVERGING, epoch=1)

    def test_long_cannot_rewind_to_short(self):
        self._put("X06", SvAmbState.ANCHORED)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X06", SvAmbState.ANCHORING, epoch=1)

    def test_squelched_cannot_skip_to_wl(self):
        self._put("X07", SvAmbState.WAITING)
        with self.assertRaises(InvalidTransition):
            self.t.transition("X07", SvAmbState.CONVERGING, epoch=1)


class FixSetMembershipTest(unittest.TestCase):
    """SvStateTracker membership helpers match the spec."""

    def setUp(self):
        self.t = SvStateTracker()

    def test_anchoring_and_anchored_counts(self):
        # Two in short-term, one in long-term, one unfixed.
        for sv, state in [
            ("G01", SvAmbState.ANCHORING),
            ("G02", SvAmbState.ANCHORING),
            ("G03", SvAmbState.ANCHORED),
            ("G04", SvAmbState.FLOATING),
        ]:
            self.t.get(sv).state = state
        self.assertEqual(len(self.t.anchoring_svs()), 2)
        self.assertEqual(len(self.t.anchored_svs()), 1)
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
    """False-fix monitor fires only on ANCHORING SVs, at eval epochs, above threshold."""

    def setUp(self):
        self.t = SvStateTracker()
        self.m = FalseFixMonitor(
            self.t, base_threshold_m=2.0, min_samples=5, eval_every=10,
        )

    def _to_short(self, sv):
        self.t.transition(sv, SvAmbState.FLOATING, epoch=0)
        self.t.transition(sv, SvAmbState.CONVERGING, epoch=1)
        self.t.transition(sv, SvAmbState.ANCHORING, epoch=2)

    def test_ignores_svs_not_in_short(self):
        # SV in FLOATING — should be ignored by the false-fix monitor.
        self.t.transition("G01", SvAmbState.FLOATING, epoch=0)
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
        # Tracker moved the SV to WAITING (not FLOATING) with a per-SV
        # cooldown chosen by elev — high-elev (90°) is unexpected #1.
        self.assertIs(self.t.state("G02"), SvAmbState.WAITING)
        self.assertEqual(events[0]['tag'], "unexpected #1")
        # First unexpected → progression[0] = 120 epochs default.
        self.assertEqual(events[0]['squelch_epochs'], 120)

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


class FalseFixStratifiedSquelchTest(unittest.TestCase):
    """Elevation-stratified squelch: low-elev gets short cooldown,
    high-elev escalates through the progression."""

    def setUp(self):
        self.t = SvStateTracker()
        self.m = FalseFixMonitor(
            self.t, base_threshold_m=2.0, min_samples=5, eval_every=10,
            reliable_elev_deg=45.0,
            low_elev_squelch_epochs=60,
            unexpected_squelch_progression=(120, 300, 86400),
        )

    def _to_short(self, sv):
        self.t.transition(sv, SvAmbState.FLOATING, epoch=0)
        self.t.transition(sv, SvAmbState.CONVERGING, epoch=1)
        self.t.transition(sv, SvAmbState.ANCHORING, epoch=2)

    def test_low_elev_false_fix_is_expected(self):
        self._to_short("G01")
        labels = [("G01", 'pr', 30.0)]   # below 45° → expected
        # base 2.0 m elev-weighted at 30°: 2.0 * sin(45)/sin(30) = 2.83
        # Push mean to 3.5 m to exceed.
        for e in range(3, 13):
            self.m.ingest(e, [3.5], labels)
        events = self.m.evaluate(10)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['tag'], "expected")
        self.assertEqual(events[0]['squelch_epochs'], 60)
        # Counter should NOT have incremented (low-elev).
        rec = self.t.get("G01")
        self.assertEqual(rec.unexpected_ff_this_arc, 0)

    def test_high_elev_escalates_progression(self):
        # Simulate three successive unexpected false-fixes on the same
        # SV by manually driving the state machine through each cycle.
        sv = "G02"
        for cycle in range(3):
            # Put SV into ANCHORING for the next cycle.
            if cycle == 0:
                self._to_short(sv)
            else:
                # After a WAITING → put back through the happy path.
                # Manually transition back to FLOATING and up to ANCHORING.
                self.t.transition(sv, SvAmbState.FLOATING,
                                  epoch=1000 * cycle,
                                  reason="test:reset for cycle")
                self.t.transition(sv, SvAmbState.CONVERGING,
                                  epoch=1000 * cycle + 1)
                self.t.transition(sv, SvAmbState.ANCHORING,
                                  epoch=1000 * cycle + 2)
                # Clear false-fix window for the monitor.
                self.m.forget(sv)
            # Feed high-elev residuals to trigger false-fix.
            labels = [(sv, 'pr', 60.0)]   # 60° ≥ 45°
            base_epoch = 1000 * cycle + 3
            for e in range(base_epoch, base_epoch + 10):
                self.m.ingest(e, [3.0], labels)
            eval_epoch = 1000 * cycle + 10  # % 10 == 0
            events = self.m.evaluate(eval_epoch)
            self.assertEqual(len(events), 1)
            expected_duration = (120, 300, 86400)[cycle]
            expected_tag = ("unexpected #1", "unexpected #2",
                            "unexpected #3 arc-squelched")[cycle]
            self.assertEqual(events[0]['squelch_epochs'], expected_duration,
                             f"cycle {cycle}: expected {expected_duration}")
            self.assertEqual(events[0]['tag'], expected_tag)
            # Counter should have incremented.
            self.assertEqual(self.t.get(sv).unexpected_ff_this_arc, cycle + 1)


class SquelchCooldownSweepTest(unittest.TestCase):
    """check_squelch_cooldowns sweeps records whose expiry has passed."""

    def setUp(self):
        self.t = SvStateTracker()

    def test_sweeps_expired(self):
        self.t.transition("G01", SvAmbState.FLOATING, epoch=0)
        self.t.transition("G01", SvAmbState.WAITING, epoch=10,
                          cooldown_epochs=50)
        # Not yet expired.
        recovered = self.t.check_squelch_cooldowns(epoch=30)
        self.assertEqual(recovered, [])
        self.assertIs(self.t.state("G01"), SvAmbState.WAITING)
        # Expired.
        recovered = self.t.check_squelch_cooldowns(epoch=61)
        self.assertEqual(recovered, ["G01"])
        self.assertIs(self.t.state("G01"), SvAmbState.FLOATING)

    def test_ignores_non_squelched(self):
        self.t.transition("G02", SvAmbState.FLOATING, epoch=0)
        recovered = self.t.check_squelch_cooldowns(epoch=1000)
        self.assertEqual(recovered, [])


class ForgetStaleTest(unittest.TestCase):
    """forget_stale drops records not seen in the stale window.

    Implements arc-boundary detection: an SV that hasn't been observed
    for ≥ N epochs is presumed set; its record is forgotten so the
    next arc starts fresh (unexpected_ff_this_arc=0).
    """

    def test_drops_stale(self):
        t = SvStateTracker()
        t.transition("G01", SvAmbState.FLOATING, epoch=0)
        t.mark_seen("G01", epoch=100)
        # At epoch 1000 with stale_after=600, SV last seen 900 epochs
        # ago → forget.
        dropped = t.forget_stale(epoch=1000, stale_after_epochs=600)
        self.assertEqual(dropped, ["G01"])
        # Tracker no longer has the record; a fresh get() creates a
        # new one in TRACKING state.
        self.assertIs(t.state("G01"), SvAmbState.TRACKING)

    def test_keeps_fresh(self):
        t = SvStateTracker()
        t.transition("G02", SvAmbState.FLOATING, epoch=0)
        t.mark_seen("G02", epoch=900)
        dropped = t.forget_stale(epoch=1000, stale_after_epochs=600)
        self.assertEqual(dropped, [])


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
        self.t.transition(sv, SvAmbState.FLOATING, epoch=0)
        self.t.transition(sv, SvAmbState.CONVERGING, epoch=1)
        self.t.transition(sv, SvAmbState.ANCHORING, epoch=2)

    def test_drops_on_elev_below_mask(self):
        self._to_short("G10")
        labels = [("G10", 'pr', 15.0)]   # below 18° mask
        for e in range(3, 13):
            self.m.ingest(e, [0.1], labels)   # residual is fine
        events = self.m.evaluate(10)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['reason'], 'elev_mask')
        # Now transitions straight back to FLOATING (no RETIRING in-between).
        self.assertIs(self.t.state("G10"), SvAmbState.FLOATING)

    def test_drops_on_elev_weighted_resid(self):
        self._to_short("G11")
        # PR breach plus IF breach: the gate fires.  Pass both
        # residuals so the IF gate has data to evaluate.  IF=0.20m
        # >> 0.05m threshold = legitimate wrong-integer pathology.
        labels = [("G11", 'pr', 30.0), ("G11", 'phi', 30.0)]
        for e in range(3, 13):
            self.m.ingest(e, [5.0, 0.20], labels)
        events = self.m.evaluate(10)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['reason'], 'elev_weighted_resid')
        self.assertIs(self.t.state("G11"), SvAmbState.FLOATING)

    def test_if_clean_suppresses_pr_drop(self):
        """I-161514, 2026-05-02: PR-multipath false positive.  PR
        residual breaches the elev-weighted threshold, but the IF
        residual is sub-cm — the carrier-phase integer is correct.
        The gate must suppress the drop.  This is the canonical
        same-second-wasted-evict pattern from day0501 overnight
        (24/24 across TimeHat + MadHat)."""
        self._to_short("G12")
        labels = [("G12", 'pr', 30.0), ("G12", 'phi', 30.0)]
        # PR mean 5.0 m exceeds threshold; IF mean 0.012 m ≪ 0.05 m
        # threshold = integer is correct.
        for e in range(3, 13):
            self.m.ingest(e, [5.0, 0.012], labels)
        events = self.m.evaluate(10)
        self.assertEqual(len(events), 0,
                         "PR breach with IF clean must NOT evict")
        # SV stays in ANCHORING — not transitioned to FLOATING.
        self.assertIs(self.t.state("G12"), SvAmbState.ANCHORING)

    def test_if_breach_allows_pr_drop(self):
        """Mirror of test_if_clean_suppresses_pr_drop: when the IF
        residual is also breached, the gate doesn't suppress and
        the eviction proceeds.  Real wrong-integer pathology has
        IF >> 50mm."""
        self._to_short("G13")
        labels = [("G13", 'pr', 30.0), ("G13", 'phi', 30.0)]
        # Both PR and IF breach: real integer pathology.
        for e in range(3, 13):
            self.m.ingest(e, [5.0, 0.15], labels)
        events = self.m.evaluate(10)
        self.assertEqual(len(events), 1,
                         "PR + IF both breach → eviction must fire")
        self.assertEqual(events[0]['reason'], 'elev_weighted_resid')
        self.assertIs(self.t.state("G13"), SvAmbState.FLOATING)

    def test_no_if_samples_falls_through_to_pr_only(self):
        """Backward-compat: if no phi residuals are streamed (early-
        arc state, monitor wired without IF labels, etc.), the gate
        falls through to the legacy PR-only behaviour.  Do not
        suppress what we can't classify."""
        self._to_short("G14")
        labels = [("G14", 'pr', 30.0)]   # PR only, no phi
        for e in range(3, 13):
            self.m.ingest(e, [5.0], labels)
        events = self.m.evaluate(10)
        self.assertEqual(len(events), 1,
                         "no IF samples → legacy PR-only behavior")
        self.assertEqual(events[0]['reason'], 'elev_weighted_resid')

    def test_ignores_non_nl_svs(self):
        # SV in FLOATING — not eligible.
        self.t.transition("G12", SvAmbState.FLOATING, epoch=0)
        labels = [("G12", 'pr', 10.0)]
        for e in range(3, 13):
            self.m.ingest(e, [5.0], labels)
        self.assertEqual(self.m.evaluate(10), [])

    def test_no_drop_above_residual_ceiling(self):
        """High-elev SV with bad residual: setting_sv_drop must NOT
        fire — that SV isn't physically setting.  Residual issues
        above the ceiling are FalseFixMonitor's domain, or a
        filter-health issue.  Dropping a high-elev anchor here
        sacrifices ZTD/altitude disambiguation geometry."""
        self._to_short("G30")
        # elev=60° is above the default 30° residual ceiling.
        # Residual of 5 m would trigger a drop if below the ceiling
        # (elev-weighted threshold at 60° is 3.0 m, clamped).
        labels = [("G30", 'pr', 60.0)]
        for e in range(3, 13):
            self.m.ingest(e, [5.0], labels)
        events = self.m.evaluate(10)
        self.assertEqual(
            events, [],
            "setting_sv_drop fired above the residual ceiling — "
            "regression against the day0421 high-elev anchor-drop "
            "pattern that motivated this gate",
        )
        # SV stays in ANCHORING, not dropped.
        self.assertIs(self.t.state("G30"), SvAmbState.ANCHORING)

    def test_drop_allowed_at_ceiling_boundary(self):
        """elev == ceiling still qualifies as the setting band (the
        gate is strictly '>', not '≥').  Preserves the existing
        behavior at elev=30° tested in test_drops_on_elev_weighted_resid."""
        self._to_short("G31")
        labels = [("G31", 'pr', 30.0)]   # exactly at the ceiling
        for e in range(3, 13):
            self.m.ingest(e, [5.0], labels)
        events = self.m.evaluate(10)
        self.assertEqual(len(events), 1)

    def test_repeat_drop_logs_warning(self):
        """An SV can only truly set once per pass.  A second residual
        drop on the same SV is diagnostic — log a REPEAT warning so
        operators can investigate multipath / filter health."""
        self._to_short("G32")
        # elev=25° is inside the setting band (between drop_mask=18° and
        # residual_ceiling=30°).  At 25°, elev-weighted threshold is
        # 3.0 * sin(45°)/sin(25°) ≈ 5.02 m, so residual=6.0 triggers.
        labels = [("G32", 'pr', 25.0)]
        for e in range(3, 13):
            self.m.ingest(e, [6.0], labels)
        # First drop — count=1, no REPEAT warning.
        events1 = self.m.evaluate(10)
        self.assertEqual(len(events1), 1)
        self.assertEqual(events1[0]['drop_count_session'], 1)

        # Re-admit and drop again to trigger the repeat warning.
        self._to_short("G32")
        for e in range(13, 23):
            self.m.ingest(e, [6.0], labels)
        with self.assertLogs(
            'peppar_fix.setting_sv_drop_monitor', level='WARNING',
        ) as lm:
            events2 = self.m.evaluate(20)
        self.assertEqual(len(events2), 1)
        self.assertEqual(events2[0]['drop_count_session'], 2)
        self.assertTrue(
            any('SETTING_SV_DROP_REPEAT' in r.message for r in lm.records),
            "second drop should emit the REPEAT warning",
        )


class FixSetIntegrityMonitorTest(unittest.TestCase):
    """Alarm requires RMS threshold sustained, no recent per-SV monitor, no cooldown."""

    def setUp(self):
        self.t = SvStateTracker()
        self.alarm = FixSetIntegrityMonitor(
            self.t,
            rms_threshold_m=5.0,
            min_samples_in_window=3,
            eval_every=10,
            cooldown_epochs=60,
            suppress_if_monitors_fired_within=60,
        )

    def _to_short(self, sv):
        self.t.transition(sv, SvAmbState.FLOATING, epoch=0)
        self.t.transition(sv, SvAmbState.CONVERGING, epoch=1)
        self.t.transition(sv, SvAmbState.ANCHORING, epoch=2)

    def test_fires_when_sustained_rms_exceeds(self):
        self._to_short("G20")
        self._to_short("G21")
        # Pass both PR and phi residuals.  PR-RMS breaches at ~6.5m;
        # IF-RMS at ~0.20m breaches the 0.05m gate too — real
        # integer pathology.
        labels = [
            ("G20", 'pr', 45.0), ("G21", 'pr', 45.0),
            ("G20", 'phi', 45.0), ("G21", 'phi', 45.0),
        ]
        for e in range(3, 13):
            self.alarm.ingest(e, [6.0, 7.0, 0.20, 0.20], labels)
        ev = self.alarm.evaluate(10)
        self.assertIsNotNone(ev)
        self.assertGreater(ev['window_rms_m'], 5.0)
        # Event dict carries the IF fields (I-162353) for diagnostics.
        self.assertIn('if_window_rms_m', ev)
        self.assertGreater(ev['if_window_rms_m'], 0.05)

    def test_window_rms_if_clean_suppresses(self):
        """I-162353, 2026-05-02: PR-multipath false positive on the
        fix-set RMS path.  PR-RMS over the window breaches threshold
        but the IF-RMS is sub-cm — integer-clean PR storm.  The gate
        must suppress.  Empirically 67% of TimeHat day0501 trips fit
        this pattern (8 of 12, IF-RMS 4-80mm)."""
        self._to_short("G24")
        self._to_short("G25")
        labels = [
            ("G24", 'pr', 45.0), ("G25", 'pr', 45.0),
            ("G24", 'phi', 45.0), ("G25", 'phi', 45.0),
        ]
        # PR-RMS ~6.5m breach; IF-RMS ~0.020m well under 0.05m gate.
        for e in range(3, 13):
            self.alarm.ingest(e, [6.0, 7.0, 0.018, 0.022], labels)
        ev = self.alarm.evaluate(10)
        self.assertIsNone(ev,
                          "PR-RMS breach with IF-RMS clean must not trip")

    def test_window_rms_no_if_samples_falls_through(self):
        """Backward-compat: when no phi residuals are streamed
        (early-arc state or test using PR-only labels), the gate
        falls through to the legacy PR-only check.  Don't suppress
        what we can't classify."""
        self._to_short("G26")
        self._to_short("G27")
        labels = [("G26", 'pr', 45.0), ("G27", 'pr', 45.0)]
        for e in range(3, 13):
            self.alarm.ingest(e, [6.0, 7.0], labels)
        ev = self.alarm.evaluate(10)
        self.assertIsNotNone(ev,
                             "no IF samples → legacy PR-only behavior")
        # Event still emits, with if_* fields = None.
        self.assertIsNone(ev['if_window_rms_m'])
        self.assertEqual(ev['if_n_samples'], 0)

    def test_window_rms_replay_timehat_day0501(self):
        """Regression test against the TimeHat day0501morning
        empirical trip table (I-162353 discuss line, 2026-05-02).

        The 12 window_rms trips from a 21h overnight, classified by
        the offline analysis script (window_rms_attribution.py) using
        a 100mm IF-RMS truth threshold:
          - 8 FALSE_POS (IF-RMS 4-80mm)  ← integer-clean PR storms
          - 3 JUSTIFIED (IF-RMS 122-226mm) ← real integer pathology
          - 1 AMBIGUOUS (n=0 NL samples; not modeled here)

        Operational gate is 50mm (IF_STEP NEW scale, conservative
        inner edge of the FALSE_POS / JUSTIFIED gap per Bravo's ack):
          - IF < 50mm → trip suppressed (6 of 8 FALSE_POS suppressed)
          - IF in 50-80mm → trip fires (2 FALSE_POS leak through;
            accepted by Bravo's "conservative" framing — err on the
            side of catching real trips, not suppressing them)
          - IF ≥ 122mm → trip fires (3 of 3 JUSTIFIED kept)

        Net under the 50mm gate: 5 trips of 11 cases (6 suppressed),
        a 55% reduction in this empirical sample.  Locks in the
        behavior so future tuning can't silently broaden the gate."""
        cases = [
            # (desc, pr_rms_m, if_rms_m, expected_trip under 50mm gate)
            ('11:30:22 FP/IF=20mm',           5.3, 0.020, False),
            ('14:15:12 JUSTIFIED/IF=226mm', 10.8, 0.226, True),
            ('20:47:58 FP/IF=4mm',            5.6, 0.004, False),
            ('22:04:52 JUSTIFIED/IF=122mm', 12.6, 0.122, True),
            ('23:17:44 FP/IF=80mm leaked',    6.0, 0.080, True),
            ('03:27:46 FP/IF=27mm',         12.3, 0.027, False),
            ('04:44:40 JUSTIFIED/IF=201mm',   5.4, 0.201, True),
            ('05:55:44 FP/IF=20mm',           7.5, 0.020, False),
            ('06:39:56 FP/IF=15mm',           8.7, 0.015, False),
            ('07:34:45 FP/IF=49mm',         10.6, 0.049, False),
            ('09:19:53 FP/IF=72mm leaked',    5.3, 0.072, True),
        ]
        n_trips = 0
        for desc, pr_rms, if_rms, expected_trip in cases:
            with self.subTest(case=desc):
                t = SvStateTracker()
                m = FixSetIntegrityMonitor(
                    t,
                    rms_threshold_m=5.0,
                    if_rms_threshold_m=0.05,
                    min_samples_in_window=3,
                    eval_every=1,
                    cooldown_epochs=0,
                )
                # Pre-fill the histories directly with the documented
                # window-mean values.  Direct internal-state access is
                # acceptable in a regression unit test that pins the
                # trip behavior against a frozen empirical dataset.
                for _ in range(30):
                    m._rms_hist.append(pr_rms)
                    m._if_rms_hist.append(if_rms)
                ev = m.evaluate(100)
                if expected_trip:
                    self.assertIsNotNone(
                        ev, f"{desc}: expected trip, got None")
                    self.assertEqual(ev['reason'], 'window_rms')
                    n_trips += 1
                else:
                    self.assertIsNone(
                        ev, f"{desc}: expected suppress, got {ev}")
        # Quantitative pin: 5 of 11 modeled cases trip.
        self.assertEqual(
            n_trips, 5,
            f"Empirical regression: expected 5 trips of 11, got {n_trips}. "
            "If this number changes, the gate was retuned — update the "
            "expected_trip column in the case table.")

    def test_suppressed_by_recent_per_sv_transition(self):
        self._to_short("G22")
        # Simulate a false-fix monitor just moved an SV to FLOATING.
        self.t.transition("G22", SvAmbState.FLOATING, epoch=5,
                          reason="false_fix:synthetic")
        # Put another SV in short-term so its residuals count.
        self._to_short("G23")
        for e in range(6, 13):
            self.alarm.ingest(e, [6.0], [("G23", 'pr', 45.0)])
        # Epoch 10 is within suppress_if_monitors_fired_within=60 of
        # G22's FLOATING transition at epoch 5 — alarm stays silent.
        self.assertIsNone(self.alarm.evaluate(10))

    def test_respects_cooldown(self):
        self._to_short("G24")
        labels = [("G24", 'pr', 45.0)]
        for e in range(3, 13):
            self.alarm.ingest(e, [6.0], labels)
        ev1 = self.alarm.evaluate(10)
        self.assertIsNotNone(ev1)
        self.alarm.record_trip(10)
        # Immediate re-check must be suppressed by cooldown.
        for e in range(11, 21):
            self.alarm.ingest(e, [6.0], labels)
        self.assertIsNone(self.alarm.evaluate(20))


class ZtdImpossibilityTripTest(unittest.TestCase):
    """evaluate(..., ztd_m=X) returns a ztd_impossible trip event
    when |ZTD| > threshold is sustained, re-arms after the trip
    is recorded, and returns None for sub-threshold / transient
    / None inputs."""

    def setUp(self):
        self.t = SvStateTracker()
        # eval_every=1 so every epoch is a sampling opportunity,
        # ztd_sustained_epochs=5 for quick test.
        self.m = FixSetIntegrityMonitor(
            self.t,
            ztd_trip_threshold_m=0.7,
            ztd_sustained_epochs=5,
            eval_every=1,
            cooldown_epochs=0,
        )

    def test_fires_at_sustained_above_threshold(self):
        # ZTD = -986 mm sustained for 5 epochs.  Monitor must return
        # an event on epoch 105 (since the first above-threshold
        # epoch is 100, and 105 - 100 = 5 == sustained_epochs).
        ev = None
        for e in range(100, 106):
            result = self.m.evaluate(e, ztd_m=-0.986)
            if result is not None:
                ev = result
        self.assertIsNotNone(ev)
        self.assertEqual(ev['reason'], 'ztd_impossible')
        self.assertAlmostEqual(ev['ztd_m'], -0.986)
        self.assertAlmostEqual(ev['threshold_m'], 0.7)
        self.assertGreaterEqual(ev['sustained_epochs'], 5)

    def test_does_not_fire_when_below_threshold(self):
        # |ZTD| = 400 mm < 700 mm — never fires, even for 100 epochs.
        for e in range(100, 200):
            self.assertIsNone(self.m.evaluate(e, ztd_m=-0.400))

    def test_transient_spike_does_not_fire(self):
        # ZTD spikes to -900 mm for 3 epochs (< sustained=5), then
        # back to nominal.  No event should fire.
        for e in range(100, 103):
            self.assertIsNone(self.m.evaluate(e, ztd_m=-0.900))
        for e in range(103, 110):
            self.assertIsNone(self.m.evaluate(e, ztd_m=-0.100))

    def test_rearms_after_record_trip(self):
        # After firing once and calling record_trip, a second
        # sustained window should fire again.
        fired_first = False
        for e in range(100, 106):
            if self.m.evaluate(e, ztd_m=-0.986) is not None:
                fired_first = True
                self.m.record_trip(e)
                break
        self.assertTrue(fired_first)
        # Now a second sustained window.
        fired_second = False
        for e in range(200, 210):
            if self.m.evaluate(e, ztd_m=-0.986) is not None:
                fired_second = True
                break
        self.assertTrue(fired_second, "re-arm after record_trip")

    def test_none_ztd_skips_check(self):
        # Caller may not always have a ZTD state.  None must not
        # cause a trip and must not advance the timer.
        for e in range(100, 200):
            self.assertIsNone(self.m.evaluate(e, ztd_m=None))
        self.assertIsNone(self.m._ztd_above_since)

    def test_no_latch_required(self):
        # ZTD trip is explicitly latch-independent — it fires
        # regardless of reached_anchoring / reached_anchored.  No
        # ape_state_machine reference passed here; trip still works.
        for e in range(100, 106):
            result = self.m.evaluate(e, ztd_m=+0.986)  # positive works too
            if result is not None:
                self.assertEqual(result['reason'], 'ztd_impossible')
                return
        self.fail("ZTD trip should fire without a latch set")


class WlOnlyClampTest(unittest.TestCase):
    """WL-only mode: SvStateTracker refuses promotion past CONVERGING.

    The NL resolver is separately gated off in WL-only mode so these
    transitions shouldn't normally occur; the clamp is belt-and-
    suspenders for any caller (bootstrap, monitors) that still tries.
    """

    def test_converging_to_anchoring_refused(self):
        t = SvStateTracker(wl_only=True)
        # Walk up to CONVERGING normally.
        t.transition("G01", SvAmbState.FLOATING, epoch=10, reason="admit")
        t.transition("G01", SvAmbState.CONVERGING, epoch=20, reason="wl_fix")
        # Promotion to ANCHORING must be refused without mutation.
        t.transition("G01", SvAmbState.ANCHORING, epoch=30, reason="nl_fix")
        self.assertEqual(t.state("G01"), SvAmbState.CONVERGING)

    def test_converging_to_anchored_refused(self):
        t = SvStateTracker(wl_only=True)
        t.transition("G01", SvAmbState.FLOATING, epoch=10, reason="admit")
        t.transition("G01", SvAmbState.CONVERGING, epoch=20, reason="wl_fix")
        t.transition("G01", SvAmbState.ANCHORED, epoch=30, reason="promoted")
        self.assertEqual(t.state("G01"), SvAmbState.CONVERGING)

    def test_demotion_still_works(self):
        # WL-only must not break demotion paths — if a slip or squelch
        # fires, the SV must still be able to leave CONVERGING.
        t = SvStateTracker(wl_only=True)
        t.transition("G01", SvAmbState.FLOATING, epoch=10, reason="admit")
        t.transition("G01", SvAmbState.CONVERGING, epoch=20, reason="wl_fix")
        t.transition("G01", SvAmbState.FLOATING, epoch=30, reason="slip")
        self.assertEqual(t.state("G01"), SvAmbState.FLOATING)

    def test_wl_only_default_off(self):
        # Without the flag, the legal ANCHORING promotion works.
        t = SvStateTracker()  # default wl_only=False
        t.transition("G01", SvAmbState.FLOATING, epoch=10, reason="admit")
        t.transition("G01", SvAmbState.CONVERGING, epoch=20, reason="wl_fix")
        t.transition("G01", SvAmbState.ANCHORING, epoch=30,
                     az_deg=180.0, reason="nl_fix")
        self.assertEqual(t.state("G01"), SvAmbState.ANCHORING)


class ZtdCyclingEscalationTest(unittest.TestCase):
    """When ZTD trip fires repeatedly inside the escalation window
    without the filter escaping, evaluate() escalates the reason
    from ztd_impossible to ztd_cycling so the caller can execute
    a full WL flush (MW reset for every SV)."""

    def setUp(self):
        self.t = SvStateTracker()
        # Small windows for fast tests.  Threshold=2 trips within
        # 100 epochs → the 2nd trip inside that window escalates.
        # Matches the production default tightened 2026-04-22 after
        # day0422d showed one NL-only revert was sufficient evidence.
        self.m = FixSetIntegrityMonitor(
            self.t,
            ztd_trip_threshold_m=0.7,
            ztd_sustained_epochs=5,
            ztd_escalate_threshold=2,
            ztd_escalate_window_epochs=100,
            eval_every=1,
            cooldown_epochs=0,
        )

    def _trip(self, base_epoch: int) -> dict:
        """Drive one ZTD trip, record it, return the event dict."""
        ev = None
        for e in range(base_epoch, base_epoch + 6):
            result = self.m.evaluate(e, ztd_m=-0.986)
            if result is not None:
                ev = result
                self.m.record_trip(e)
                break
        assert ev is not None, f"trip failed to fire at base {base_epoch}"
        return ev

    def test_first_trip_is_ztd_impossible(self):
        ev1 = self._trip(100)
        self.assertEqual(ev1['reason'], 'ztd_impossible')
        self.assertEqual(ev1['recent_trip_count'], 1)

    def test_second_trip_in_window_escalates_to_cycling(self):
        self._trip(100)
        ev2 = self._trip(150)  # within 100-epoch window
        self.assertEqual(ev2['reason'], 'ztd_cycling')
        self.assertEqual(ev2['recent_trip_count'], 2)

    def test_trip_outside_window_does_not_count(self):
        # Two trips spaced 150 epochs apart (> window=100).  The
        # first trip is outside the window when the second evaluates,
        # so the second trip reports count=1 and stays ztd_impossible.
        self._trip(100)
        ev2 = self._trip(250)
        self.assertEqual(ev2['reason'], 'ztd_impossible')
        self.assertEqual(ev2['recent_trip_count'], 1)

    def test_escalation_clears_history(self):
        # After escalating once, the count resets to 1 — the next
        # ztd_cycling again requires a fresh threshold count inside
        # the window.
        self._trip(100)
        ev2 = self._trip(150)
        self.assertEqual(ev2['reason'], 'ztd_cycling')
        ev3 = self._trip(200)
        self.assertEqual(ev3['reason'], 'ztd_impossible')
        self.assertEqual(ev3['recent_trip_count'], 1)


if __name__ == "__main__":
    unittest.main()

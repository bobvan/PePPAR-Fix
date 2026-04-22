"""Unit tests for the AntPosEst latch pair, three-regime gate
selection, and the anchor-collapse FixSetIntegrityAlarm trigger.

Covers the regime table (post-lifecycle-rename):

| reached_anchored | anchored_count | Gate regime      |
|---|---|---|
| False | any | bootstrap       |
| True  | ≥ N | strong_anchor   |
| True  | < N | thin_anchor     |
| True  | = 0 for M epochs | FixSetIntegrityAlarm fires |

Plus the ``reached_anchoring`` latch which answers the weaker
question "has the filter ever produced integer fixes?" — set on
first entry to ANCHORING, monotonic within a lifetime between
alarm trips.

These are unit-level tests.  The end-to-end behaviour against
real lab data is covered by the live-log runs.
"""

from __future__ import annotations

import os
import sys
import unittest

import numpy as np

_SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from ppp_ar import NarrowLaneResolver           # noqa: E402
from solve_ppp import N_BASE                    # noqa: E402

from peppar_fix.sv_state import (               # noqa: E402
    SvAmbState, SvStateTracker,
)
from peppar_fix.states import AntPosEst, AntPosEstState  # noqa: E402
from peppar_fix.fix_set_integrity_alarm import FixSetIntegrityAlarm  # noqa: E402


# ── Helpers ──────────────────────────────────────────────────────── #

def _latch_anchoring(sm: AntPosEst) -> None:
    """Walk the state machine into ANCHORING — latches
    ``reached_anchoring`` but not ``reached_anchored``."""
    sm.transition(AntPosEstState.VERIFYING, "test")
    sm.transition(AntPosEstState.CONVERGING, "test")
    sm.transition(AntPosEstState.ANCHORING, "test")


def _latch_anchored(sm: AntPosEst) -> None:
    """Walk the state machine through to ANCHORED — latches both
    ``reached_anchoring`` and ``reached_anchored``."""
    sm.transition(AntPosEstState.VERIFYING, "test")
    sm.transition(AntPosEstState.CONVERGING, "test")
    sm.transition(AntPosEstState.ANCHORING, "test")
    sm.transition(AntPosEstState.ANCHORED, "test")


def _long_term(tracker: SvStateTracker, sv: str, epoch: int = 0,
               elev: float = 60.0) -> None:
    """Walk an SV through the legal edges to ANCHORED."""
    tracker.transition(sv, SvAmbState.FLOATING, epoch=epoch, reason="admit")
    tracker.transition(sv, SvAmbState.CONVERGING, epoch=epoch, reason="mw")
    tracker.transition(sv, SvAmbState.ANCHORING, epoch=epoch,
                       reason="lambda", az_deg=90.0, elev_deg=elev)
    tracker.transition(sv, SvAmbState.ANCHORED, epoch=epoch,
                       reason="delta_az", elev_deg=elev)


# ── AntPosEst latches ────────────────────────────────────────────── #

class ReachedAnchoringLatchTest(unittest.TestCase):
    """``AntPosEst.reached_anchoring``: latches on first ANCHORING
    entry.  Cleared only by ``clear_latches()`` (called from
    ``FixSetIntegrityAlarm.record_fire``)."""

    def test_starts_false(self):
        sm = AntPosEst()
        self.assertFalse(sm.reached_anchoring)

    def test_latches_on_anchoring(self):
        sm = AntPosEst()
        _latch_anchoring(sm)
        self.assertTrue(sm.reached_anchoring)

    def test_latches_indirectly_via_anchored(self):
        """Reaching ANCHORED via any path (including a direct
        jump in a test) must also latch reached_anchoring — the
        ≥ 4 ANCHORED milestone implies ≥ 4 NL fixed."""
        sm = AntPosEst()
        _latch_anchored(sm)
        self.assertTrue(sm.reached_anchoring)
        self.assertTrue(sm.reached_anchored)

    def test_stays_true_on_backwards_transitions(self):
        """Falling back to CONVERGING after ANCHORING must NOT
        clear the latch — that's normal flap behaviour, not a
        reason to invalidate the earned trust."""
        sm = AntPosEst()
        _latch_anchoring(sm)
        sm.transition(AntPosEstState.CONVERGING, "flap")
        self.assertTrue(sm.reached_anchoring)
        sm.transition(AntPosEstState.ANCHORING, "re-flap")
        self.assertTrue(sm.reached_anchoring)

    def test_cleared_by_explicit_api(self):
        sm = AntPosEst()
        _latch_anchoring(sm)
        sm.clear_latches(reason="test")
        self.assertFalse(sm.reached_anchoring)


class ReachedAnchoredLatchTest(unittest.TestCase):
    """``AntPosEst.reached_anchored``: latches only on first
    ANCHORED entry (≥ 4 ANCHORED validated).  This is the
    latch that gates the anchor-collapse trigger — firing only on
    filters that genuinely earned the state."""

    def test_starts_false(self):
        sm = AntPosEst()
        self.assertFalse(sm.reached_anchored)

    def test_does_not_latch_on_anchoring(self):
        """ANCHORING is the fallback path (≥ 4 NL fixed, pre-
        validation).  ``reached_anchored`` must stay False —
        that's the core of the day0421f fix.  Gating
        anchor-collapse on this latch must not fire from the
        ANCHORING-only state, since long_term_members() may be
        0 throughout that state."""
        sm = AntPosEst()
        _latch_anchoring(sm)
        self.assertTrue(sm.reached_anchoring)
        self.assertFalse(sm.reached_anchored)

    def test_latches_on_anchored(self):
        sm = AntPosEst()
        _latch_anchored(sm)
        self.assertTrue(sm.reached_anchored)

    def test_stays_true_on_backwards_transitions(self):
        """ANCHORED → ANCHORING → CONVERGING is normal slip-storm
        behaviour; neither step clears ``reached_anchored``."""
        sm = AntPosEst()
        _latch_anchored(sm)
        sm.transition(AntPosEstState.ANCHORING, "anchor lost")
        self.assertTrue(sm.reached_anchored)
        sm.transition(AntPosEstState.CONVERGING, "also fix count lost")
        self.assertTrue(sm.reached_anchored)

    def test_cleared_by_explicit_api(self):
        sm = AntPosEst()
        _latch_anchored(sm)
        sm.clear_latches(reason="test")
        self.assertFalse(sm.reached_anchored)

    def test_clear_latches_clears_both(self):
        """The alarm's record_fire calls clear_latches() once —
        both latches must drop together."""
        sm = AntPosEst()
        _latch_anchored(sm)
        self.assertTrue(sm.reached_anchoring)
        self.assertTrue(sm.reached_anchored)
        sm.clear_latches(reason="test")
        self.assertFalse(sm.reached_anchoring)
        self.assertFalse(sm.reached_anchored)

    def test_clear_is_idempotent(self):
        sm = AntPosEst()
        sm.clear_latches(reason="test")   # no-op — both already False
        self.assertFalse(sm.reached_anchoring)
        self.assertFalse(sm.reached_anchored)


# ── Regime selection ─────────────────────────────────────────────── #

class RegimeSelectionTest(unittest.TestCase):
    """``_active_regime()`` and ``_active_gates()`` select correctly
    across the three rows of the gate table.  Post-rename the gate
    is keyed on ``reached_anchored`` (the strict latch) not
    ``reached_anchoring`` — bootstrap regime persists through the
    ANCHORING-only window until the first ANCHORED promotion."""

    def setUp(self):
        self.tracker = SvStateTracker()
        self.sm = AntPosEst()
        self.resolver = NarrowLaneResolver(
            sv_state=self.tracker,
            ape_state_machine=self.sm,
            strong_anchor_min=3,
            lambda_min_p_bootstrap=0.97,
            bootstrap_lambda_min_p=0.999,
        )

    def test_bootstrap_when_not_reached_anchored(self):
        self.assertEqual(self.resolver._active_regime(), 'bootstrap')

    def test_bootstrap_persists_through_anchoring(self):
        """Key guarantee of the rename: while in ANCHORING (not yet
        ANCHORED), the regime stays bootstrap — we haven't earned
        the trust that anchored_by_svs / anchored_by_position
        presume."""
        _latch_anchoring(self.sm)   # latches reached_anchoring only
        self.assertEqual(self.resolver._active_regime(), 'bootstrap')

    def test_bootstrap_regime_uses_bootstrap_params(self):
        """Regime selector plumbs the ``bootstrap_*`` overrides
        through.  Defaults now match normal mode (no extra gate
        skepticism during bootstrap), but the mechanism must still
        work when a caller explicitly tightens — setUp passes
        ``bootstrap_lambda_min_p=0.999`` so that's what we expect."""
        self.assertEqual(
            self.resolver._active_gates()['lambda_min_p_bootstrap'],
            0.999,
        )

    def test_thin_anchor_when_anchored_and_few_anchors(self):
        _latch_anchored(self.sm)
        _long_term(self.tracker, "E05")   # 1 anchor
        _long_term(self.tracker, "E06")   # 2 anchors
        # strong_anchor_min=3 → still thin with only 2
        self.assertEqual(self.resolver._active_regime(), 'thin_anchor')

    def test_strong_anchor_when_anchored_and_enough_anchors(self):
        _latch_anchored(self.sm)
        for i in range(3):
            _long_term(self.tracker, f"E0{i+5}")
        self.assertEqual(self.resolver._active_regime(), 'strong_anchor')

    def test_strong_and_thin_use_normal_gates(self):
        """Post-anchored regimes don't tighten gates — the
        strictness there comes from the join-test variant, not the
        rectangle/corner params."""
        _latch_anchored(self.sm)
        # thin_anchor (no anchors)
        self.assertEqual(
            self.resolver._active_gates()['lambda_min_p_bootstrap'],
            0.97,
        )
        for i in range(3):
            _long_term(self.tracker, f"E0{i+5}")
        # strong_anchor
        self.assertEqual(
            self.resolver._active_gates()['lambda_min_p_bootstrap'],
            0.97,
        )

    def test_bootstrap_bypasses_join_test(self):
        """``_join_test`` in bootstrap regime: always pass, no
        computation.  Even a candidate that would fail the SV-
        anchored test (if one ran) gets admitted."""
        # Set up a long-term anchor but keep reached_anchored False.
        _long_term(self.tracker, "E05")
        # Shape P so Δx[2] would be huge (reject on SV-anchored).
        import numpy as _np
        filt = _FakeFilter(N_BASE + 2)
        filt.P[2, N_BASE] = 10.0
        filt.P[N_BASE, 2] = 10.0
        filt.P[N_BASE, N_BASE] = 1.0
        filt.last_H_by_sv = {
            "E05": _np.array([0.0] * (N_BASE + 2)),
        }
        filt.last_H_by_sv["E05"][2] = 1.0
        ok, *_ = self.resolver._join_test(filt, N_BASE, fixed_value=1.0)
        self.assertTrue(ok, "bootstrap must bypass join test")


# ── Position-anchored join test ──────────────────────────────────── #

class PositionAnchoredJoinTest(unittest.TestCase):
    """``_position_anchored_join_test``: reject when Kalman-
    predicted position shift exceeds k · σ_pos."""

    def setUp(self):
        self.tracker = SvStateTracker()
        self.sm = AntPosEst()
        self.resolver = NarrowLaneResolver(
            sv_state=self.tracker,
            ape_state_machine=self.sm,
            strong_anchor_min=3,
            position_join_k=3.0,
        )
        _latch_anchored(self.sm)   # post-anchored, no anchors ⇒ thin_anchor

    def test_passes_small_position_shift(self):
        """Candidate whose predicted Δ_pos is < k·σ_pos passes."""
        filt = _FakeFilter(N_BASE + 1)
        filt.P[:3, :3] = np.eye(3) * 0.01  # σ_pos ≈ 0.17 m
        filt.P[N_BASE, N_BASE] = 0.04
        # H for ambiguity
        filt.last_H_by_sv = {
            "E05": np.zeros(N_BASE + 1),
        }
        # Cross-covariance that induces a tiny Δ_pos
        filt.P[0, N_BASE] = 0.001
        filt.P[N_BASE, 0] = 0.001
        ok, *_ = self.resolver._position_anchored_join_test(
            filt, N_BASE, fixed_value=1.0,
        )
        self.assertTrue(ok)

    def test_rejects_large_position_shift(self):
        """Candidate whose Δ_pos exceeds k·σ_pos is rejected."""
        filt = _FakeFilter(N_BASE + 1)
        filt.P[:3, :3] = np.eye(3) * 0.01  # σ_pos ≈ 0.17 m
        filt.P[N_BASE, N_BASE] = 0.04
        filt.last_H_by_sv = {
            "E05": np.zeros(N_BASE + 1),
        }
        # Large cross-covariance; Δ_pos >> k·σ_pos
        filt.P[0, N_BASE] = 1.0
        filt.P[N_BASE, 0] = 1.0
        ok, *_ = self.resolver._position_anchored_join_test(
            filt, N_BASE, fixed_value=1.0,
        )
        self.assertFalse(ok)


# ── Anchor-collapse trigger ──────────────────────────────────────── #

class AnchorCollapseTriggerTest(unittest.TestCase):
    """FixSetIntegrityAlarm fires on anchor collapse when
    ``reached_anchored=True`` + long_term=0 for N epochs.  The
    ``reached_anchored`` gate is the day0421f fix — pre-rename
    ``reached_resolved`` also fired on fallback ANCHORING without
    any long-term anchors, producing the spurious trip cycle."""

    def setUp(self):
        self.tracker = SvStateTracker()
        self.sm = AntPosEst()
        self.alarm = FixSetIntegrityAlarm(
            self.tracker,
            ape_state_machine=self.sm,
            anchor_collapse_epochs=30,
            eval_every=1,     # so we can step epoch by epoch
            cooldown_epochs=0,  # tests don't need the cooldown window
        )

    def test_does_not_fire_before_reached_anchored(self):
        """Pre-bootstrap / CONVERGING / ANCHORING-without-anchors:
        no anchors is normal, no alarm."""
        for epoch in range(1, 100):
            self.assertIsNone(self.alarm.evaluate(epoch))

    def test_does_not_fire_while_only_reached_anchoring(self):
        """The day0421f reproduction: latch reached_anchoring via
        the fallback path with zero long-term members.  The alarm
        must NOT fire regardless of how long this state persists
        — the gate is on ``reached_anchored``, not
        ``reached_anchoring``.  Pre-rename ``reached_resolved``
        latch would fire here, producing 60-epoch spurious trips."""
        _latch_anchoring(self.sm)
        self.assertTrue(self.sm.reached_anchoring)
        self.assertFalse(self.sm.reached_anchored)
        # 200 epochs well past the 30-epoch trip threshold.
        for epoch in range(1, 201):
            self.assertIsNone(
                self.alarm.evaluate(epoch),
                f"spurious anchor_collapse trip at epoch {epoch}",
            )

    def test_does_not_fire_with_anchors_present(self):
        _latch_anchored(self.sm)
        _long_term(self.tracker, "E05")
        for epoch in range(1, 100):
            self.assertIsNone(self.alarm.evaluate(epoch))

    def test_fires_after_threshold_epochs_of_collapse(self):
        _latch_anchored(self.sm)
        # Need ≥ 4 anchors initially to justify the "collapse" framing.
        for sv in ("E05", "E06", "E07", "E08"):
            _long_term(self.tracker, sv)
        for epoch in range(1, 11):
            self.assertIsNone(self.alarm.evaluate(epoch))
        for sv in ("E05", "E06", "E07", "E08"):
            self.tracker.transition(sv, SvAmbState.FLOATING,
                                    epoch=11, reason="drop")
        # Timer starts at the next evaluate() call where lt_count=0.
        # First such call here is evaluate(12), so fire happens at
        # the first epoch e where (e - 12) >= 30, i.e. epoch 42.
        for epoch in range(12, 42):
            self.assertIsNone(self.alarm.evaluate(epoch))
        ev = self.alarm.evaluate(42)
        self.assertIsNotNone(ev)
        self.assertEqual(ev['reason'], 'anchor_collapse')
        self.assertGreaterEqual(ev['anchor_collapse_epochs'], 30)

    def test_anchor_return_resets_timer(self):
        """If the anchor comes back and evaluate() sees it, the
        timer resets — the filter recovered on its own.

        Once ``reached_anchored`` has latched, a single returning
        anchor is enough to reset the collapse timer (the inner
        gate is ``lt_count > 0``, not ``lt_count >= 4``).
        """
        _latch_anchored(self.sm)
        for sv in ("E05", "E06", "E07", "E08"):
            _long_term(self.tracker, sv)
        self.alarm.evaluate(10)
        for sv in ("E05", "E06", "E07", "E08"):
            self.tracker.transition(sv, SvAmbState.FLOATING,
                                    epoch=11, reason="drop")
        # 10 epochs without anchor — nowhere near threshold (30).
        for epoch in range(12, 22):
            self.assertIsNone(self.alarm.evaluate(epoch))
        # One anchor returns; walk through the legal edges.
        self.tracker.transition("E05", SvAmbState.CONVERGING,
                                epoch=22, reason="mw")
        self.tracker.transition("E05", SvAmbState.ANCHORING,
                                epoch=22, reason="lambda",
                                az_deg=90.0, elev_deg=60.0)
        self.tracker.transition("E05", SvAmbState.ANCHORED,
                                epoch=22, reason="delta_az",
                                elev_deg=60.0)
        # Alarm observes the recovery → timer resets.
        self.assertIsNone(self.alarm.evaluate(22))
        # Anchor drops again.
        self.tracker.transition("E05", SvAmbState.FLOATING,
                                epoch=23, reason="drop again")
        # 20 more epochs of collapse shouldn't fire — timer restarted
        # at 23, need 30 more to reach threshold.
        for epoch in range(24, 44):
            self.assertIsNone(self.alarm.evaluate(epoch))

    def test_record_fire_clears_both_latches(self):
        """Post-fire, both latches must be clear so the resolver
        reverts to bootstrap gates on the re-initialised filter
        and the anchor-collapse trigger can't re-fire until the
        filter has re-earned ANCHORED status from scratch."""
        _latch_anchored(self.sm)
        self.assertTrue(self.sm.reached_anchoring)
        self.assertTrue(self.sm.reached_anchored)
        self.alarm.record_fire(epoch=100)
        self.assertFalse(self.sm.reached_anchoring)
        self.assertFalse(self.sm.reached_anchored)

    def test_anchor_collapse_event_dict_has_required_fields(self):
        """The anchor-collapse event dict must carry the fields
        the engine-side ``_apply_fix_set_alarm`` reads: ``reason``,
        ``anchor_collapse_epochs``, ``since_epoch``.  Regression
        test for the 2026-04-21 day0421b crash where the engine
        read ``ev['window_rms_m']`` unconditionally and KeyError'd
        out of AntPosEstThread on first anchor-collapse fire."""
        _latch_anchored(self.sm)
        for sv in ("E05", "E06", "E07", "E08"):
            _long_term(self.tracker, sv)
        # Observe the anchored state so the anchor-collapse check
        # is active on subsequent evaluate() calls.
        self.alarm.evaluate(0)
        for sv in ("E05", "E06", "E07", "E08"):
            self.tracker.transition(sv, SvAmbState.FLOATING,
                                    epoch=1, reason="drop")
        for epoch in range(2, 32):
            self.alarm.evaluate(epoch)
        ev = self.alarm.evaluate(32)
        self.assertIsNotNone(ev)
        self.assertEqual(ev['reason'], 'anchor_collapse')
        # Engine-side caller contract: these keys MUST exist.  If
        # the dict ever loses a field, downstream code that
        # formats a log line will crash the thread.
        self.assertIn('anchor_collapse_epochs', ev)
        self.assertIn('since_epoch', ev)

    def test_window_rms_event_dict_carries_reason(self):
        """The legacy window-RMS event dict must also carry
        ``reason='window_rms'`` so the engine-side caller can
        branch cleanly."""
        # Feed artificial residuals to drive the window RMS above
        # threshold (default 5.0 m).  First mark a SV as
        # ANCHORING so ingest accepts its residuals.
        self.tracker.transition("E05", SvAmbState.FLOATING,
                                epoch=0, reason="admit")
        self.tracker.transition("E05", SvAmbState.CONVERGING,
                                epoch=0, reason="mw")
        self.tracker.transition("E05", SvAmbState.ANCHORING,
                                epoch=0, reason="lambda",
                                az_deg=90.0, elev_deg=60.0)
        # Rebuild the alarm with default rms_threshold (5.0) and
        # a modest window so the test converges quickly.
        alarm = FixSetIntegrityAlarm(
            self.tracker, ape_state_machine=self.sm,
            eval_every=1, cooldown_epochs=0,
            min_samples_in_window=5, window_epochs=10,
            anchor_collapse_epochs=10**6,  # suppress the new path
        )
        labels = [("E05", "pr")]
        resid = [6.0]
        ev = None
        for epoch in range(1, 20):
            alarm.ingest(epoch, resid, labels)
            ev = alarm.evaluate(epoch)
            if ev is not None:
                break
        self.assertIsNotNone(ev)
        self.assertEqual(ev['reason'], 'window_rms')
        self.assertIn('window_rms_m', ev)
        self.assertIn('rms_m', ev)
        self.assertIn('n_samples', ev)

    def test_record_fire_resets_collapse_timer(self):
        """After a fire, the timer resets (not stuck at "already
        fired").  Required for a clean re-bootstrap."""
        _latch_anchored(self.sm)
        for sv in ("E05", "E06", "E07", "E08"):
            _long_term(self.tracker, sv)
        self.alarm.evaluate(0)    # observe the anchored state
        for sv in ("E05", "E06", "E07", "E08"):
            self.tracker.transition(sv, SvAmbState.FLOATING,
                                    epoch=1, reason="drop")
        # Timer starts at evaluate(2); fires at epoch 32 (32-2 >= 30).
        for epoch in range(2, 32):
            self.alarm.evaluate(epoch)
        ev = self.alarm.evaluate(32)
        self.assertIsNotNone(ev)
        self.alarm.record_fire(epoch=32)
        # Both latches cleared → reached_anchored=False, so the
        # trigger can't be met regardless of anchor count.
        self.assertIsNone(self.alarm.evaluate(33))


# ── Fake filter (shared util) ────────────────────────────────────── #

class _FakeFilter:
    def __init__(self, n_states):
        self.x = np.zeros(n_states)
        self.P = np.eye(n_states)


if __name__ == "__main__":
    unittest.main()

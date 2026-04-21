"""Unit tests for the three-regime gate selection and the
anchor-collapse FixSetIntegrityAlarm trigger.

Covers the regime table:

| reached_resolved | long_term_count | Gate regime      |
|---|---|---|
| False | any | bootstrap       |
| True  | ≥ N | strong_anchor   |
| True  | < N | thin_anchor     |
| True  | = 0 for M epochs | FixSetIntegrityAlarm fires |

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


def _latch_resolved(sm: AntPosEst) -> None:
    """Walk the state machine into RESOLVED so the latch sets."""
    sm.transition(AntPosEstState.VERIFYING, "test")
    sm.transition(AntPosEstState.VERIFIED, "test")
    sm.transition(AntPosEstState.CONVERGING, "test")
    sm.transition(AntPosEstState.RESOLVED, "test")


def _long_term(tracker: SvStateTracker, sv: str, epoch: int = 0,
               elev: float = 60.0) -> None:
    """Walk an SV through the legal edges to NL_LONG_FIXED."""
    tracker.transition(sv, SvAmbState.FLOAT, epoch=epoch, reason="admit")
    tracker.transition(sv, SvAmbState.WL_FIXED, epoch=epoch, reason="mw")
    tracker.transition(sv, SvAmbState.NL_SHORT_FIXED, epoch=epoch,
                       reason="lambda", az_deg=90.0, elev_deg=elev)
    tracker.transition(sv, SvAmbState.NL_LONG_FIXED, epoch=epoch,
                       reason="delta_az", elev_deg=elev)


# ── AntPosEst latch ──────────────────────────────────────────────── #

class ReachedResolvedLatchTest(unittest.TestCase):
    """AntPosEst.reached_resolved: latches on first RESOLVED entry,
    cleared only by explicit API call from the alarm."""

    def test_starts_false(self):
        sm = AntPosEst()
        self.assertFalse(sm.reached_resolved)

    def test_latches_on_resolved(self):
        sm = AntPosEst()
        _latch_resolved(sm)
        self.assertTrue(sm.reached_resolved)

    def test_stays_true_on_backwards_transitions(self):
        """Falling back to CONVERGING after RESOLVED must NOT clear
        the latch — that's normal flap behavior, not a reason to
        lose the earned-trust state."""
        sm = AntPosEst()
        _latch_resolved(sm)
        sm.transition(AntPosEstState.CONVERGING, "flap")
        self.assertTrue(sm.reached_resolved)
        sm.transition(AntPosEstState.RESOLVED, "re-flap")
        self.assertTrue(sm.reached_resolved)

    def test_cleared_by_explicit_api(self):
        sm = AntPosEst()
        _latch_resolved(sm)
        sm.clear_reached_resolved(reason="test")
        self.assertFalse(sm.reached_resolved)

    def test_clear_is_idempotent(self):
        sm = AntPosEst()
        sm.clear_reached_resolved(reason="test")  # no-op — already False
        self.assertFalse(sm.reached_resolved)


# ── Regime selection ─────────────────────────────────────────────── #

class RegimeSelectionTest(unittest.TestCase):
    """`_active_regime()` and `_active_gates()` select correctly
    across the three rows of the gate table."""

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

    def test_bootstrap_when_not_reached_resolved(self):
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

    def test_thin_anchor_when_resolved_and_few_anchors(self):
        _latch_resolved(self.sm)
        _long_term(self.tracker, "E05")   # 1 anchor
        _long_term(self.tracker, "E06")   # 2 anchors
        # strong_anchor_min=3 → still thin with only 2
        self.assertEqual(self.resolver._active_regime(), 'thin_anchor')

    def test_strong_anchor_when_resolved_and_enough_anchors(self):
        _latch_resolved(self.sm)
        for i in range(3):
            _long_term(self.tracker, f"E0{i+5}")
        self.assertEqual(self.resolver._active_regime(), 'strong_anchor')

    def test_strong_and_thin_use_normal_gates(self):
        """Post-resolved regimes don't tighten gates — the
        strictness there comes from the join-test variant, not the
        rectangle/corner params."""
        _latch_resolved(self.sm)
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
        """`_join_test` in bootstrap regime: always pass, no
        computation.  Even a candidate that would fail the SV-
        anchored test (if one ran) gets admitted."""
        # Set up a long-term anchor but keep reached_resolved False.
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
    """Thin-anchor regime: join test uses |Δx_pos| vs k·σ_pos."""

    def setUp(self):
        self.tracker = SvStateTracker()
        self.sm = AntPosEst()
        _latch_resolved(self.sm)  # enter post-resolved state
        self.resolver = NarrowLaneResolver(
            sv_state=self.tracker,
            ape_state_machine=self.sm,
            strong_anchor_min=3,
            position_join_k=3.0,
        )
        # No anchors → thin_anchor regime.

    def test_passes_small_position_shift(self):
        """Candidate whose predicted Δ_pos is < k·σ_pos passes."""
        filt = _FakeFilter(N_BASE + 2)
        # Small P[pos, ambig] coupling → small Δ_pos for fixed_value=1.
        filt.P[0, N_BASE] = 0.001
        filt.P[N_BASE, 0] = 0.001
        filt.P[N_BASE, N_BASE] = 1.0
        # Set position covariance so σ_pos is not tiny.
        filt.P[0, 0] = 1.0
        filt.P[1, 1] = 1.0
        filt.P[2, 2] = 1.0
        ok, worst_sv, abs_dr, thr = self.resolver._join_test(
            filt, N_BASE, fixed_value=1.0,
        )
        self.assertTrue(ok)
        self.assertIsNone(worst_sv)
        self.assertLess(abs_dr, thr)

    def test_rejects_large_position_shift(self):
        """Candidate whose Δ_pos exceeds k·σ_pos is rejected."""
        filt = _FakeFilter(N_BASE + 2)
        # Large P[0, ambig] coupling → large Δx[0].
        filt.P[0, N_BASE] = 10.0
        filt.P[N_BASE, 0] = 10.0
        filt.P[N_BASE, N_BASE] = 1.0
        # Tight position σ so threshold is small.
        filt.P[0, 0] = 0.01
        filt.P[1, 1] = 0.01
        filt.P[2, 2] = 0.01
        ok, worst_sv, abs_dr, thr = self.resolver._join_test(
            filt, N_BASE, fixed_value=1.0,
        )
        self.assertFalse(ok)
        self.assertIsNone(worst_sv)     # no per-SV attribution in this path
        self.assertGreater(abs_dr, thr)


# ── Anchor-collapse trigger ──────────────────────────────────────── #

class AnchorCollapseTriggerTest(unittest.TestCase):
    """FixSetIntegrityAlarm fires on anchor collapse when
    reached_resolved=True + long_term=0 for N epochs."""

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

    def test_does_not_fire_before_reached_resolved(self):
        """Pre-bootstrap: no anchors is normal, no alarm."""
        for epoch in range(1, 100):
            self.assertIsNone(self.alarm.evaluate(epoch))

    def test_does_not_fire_with_anchors_present(self):
        _latch_resolved(self.sm)
        _long_term(self.tracker, "E05")
        for epoch in range(1, 100):
            self.assertIsNone(self.alarm.evaluate(epoch))

    def test_fires_after_threshold_epochs_of_collapse(self):
        _latch_resolved(self.sm)
        _long_term(self.tracker, "E05")
        for epoch in range(1, 11):
            self.assertIsNone(self.alarm.evaluate(epoch))
        self.tracker.transition("E05", SvAmbState.FLOAT,
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

        The alarm's state is updated inside evaluate(), so the
        recovery must be observed by at least one evaluate() call
        while anchors are present — otherwise from the alarm's
        POV the collapse was continuous.
        """
        _latch_resolved(self.sm)
        _long_term(self.tracker, "E05")
        self.alarm.evaluate(10)
        self.tracker.transition("E05", SvAmbState.FLOAT,
                                epoch=11, reason="drop")
        # 10 epochs without anchor — nowhere near threshold (30).
        for epoch in range(12, 22):
            self.assertIsNone(self.alarm.evaluate(epoch))
        # Anchor returns; walk through the legal edges.
        self.tracker.transition("E05", SvAmbState.WL_FIXED,
                                epoch=22, reason="mw")
        self.tracker.transition("E05", SvAmbState.NL_SHORT_FIXED,
                                epoch=22, reason="lambda",
                                az_deg=90.0, elev_deg=60.0)
        self.tracker.transition("E05", SvAmbState.NL_LONG_FIXED,
                                epoch=22, reason="delta_az",
                                elev_deg=60.0)
        # Alarm observes the recovery → timer resets.
        self.assertIsNone(self.alarm.evaluate(22))
        # Anchor drops again.
        self.tracker.transition("E05", SvAmbState.FLOAT,
                                epoch=23, reason="drop again")
        # 20 more epochs of collapse shouldn't fire — timer restarted
        # at 23, need 30 more to reach threshold.
        for epoch in range(24, 44):
            self.assertIsNone(self.alarm.evaluate(epoch))

    def test_record_fire_clears_reached_resolved(self):
        """Post-fire, the latch must be clear so the resolver
        reverts to bootstrap gates on the re-initialised filter."""
        _latch_resolved(self.sm)
        self.assertTrue(self.sm.reached_resolved)
        self.alarm.record_fire(epoch=100)
        self.assertFalse(self.sm.reached_resolved)

    def test_anchor_collapse_event_dict_has_required_fields(self):
        """The anchor-collapse event dict must carry the fields
        the engine-side ``_apply_fix_set_alarm`` reads: ``reason``,
        ``anchor_collapse_epochs``, ``since_epoch``.  This is a
        regression test for the 2026-04-21 day0421b crash where
        the engine read ``ev['window_rms_m']`` unconditionally and
        KeyError'd out of AntPosEstThread on first anchor-collapse
        fire."""
        _latch_resolved(self.sm)
        _long_term(self.tracker, "E05")
        self.tracker.transition("E05", SvAmbState.FLOAT,
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
        branch cleanly.  Before b3e9fcb the dict had no reason
        field — a downstream ``ev.get('reason', 'window_rms')``
        works either way, but asserting the field is present
        future-proofs the contract."""
        # Feed artificial residuals to drive the window RMS above
        # threshold (default 5.0 m).  Stateful across epochs.
        # First mark a SV as NL_SHORT_FIXED so ingest accepts
        # its residuals.
        self.tracker.transition("E05", SvAmbState.FLOAT,
                                epoch=0, reason="admit")
        self.tracker.transition("E05", SvAmbState.WL_FIXED,
                                epoch=0, reason="mw")
        self.tracker.transition("E05", SvAmbState.NL_SHORT_FIXED,
                                epoch=0, reason="lambda",
                                az_deg=90.0, elev_deg=60.0)
        # Rebuild the alarm with default rms_threshold (5.0) and
        # a modest window so the test converges quickly.
        alarm = FixSetIntegrityAlarm(
            self.tracker, ape_state_machine=self.sm,
            eval_every=1, cooldown_epochs=0,
            min_samples_in_window=5, window_epochs=10,
            anchor_collapse_epochs=10**6,  # suppress the new path
            suppress_if_monitors_fired_within=0,
        )
        import types
        # Synthetic ingest: drive the RMS window with high residuals.
        for epoch in range(1, 20):
            labels = [("E05", "pr", 60.0)]
            resid = [10.0]  # 10 m residual — well above 5 m threshold
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
        _latch_resolved(self.sm)
        _long_term(self.tracker, "E05")
        self.tracker.transition("E05", SvAmbState.FLOAT,
                                epoch=1, reason="drop")
        # Timer starts at evaluate(2); fires at epoch 32 (32-2 >= 30).
        for epoch in range(2, 32):
            self.alarm.evaluate(epoch)
        ev = self.alarm.evaluate(32)
        self.assertIsNotNone(ev)
        self.alarm.record_fire(epoch=32)
        # Latch was cleared → reached_resolved=False now, so the
        # trigger condition can't be met regardless of anchor count.
        self.assertIsNone(self.alarm.evaluate(33))


# ── Fake filter (shared util) ────────────────────────────────────── #

class _FakeFilter:
    def __init__(self, n_states):
        self.x = np.zeros(n_states)
        self.P = np.eye(n_states)


if __name__ == "__main__":
    unittest.main()

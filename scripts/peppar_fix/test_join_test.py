"""Unit tests for NarrowLaneResolver._join_test.

The join test is the pre-commit defense against biased integer
re-admissions that put ANCHORED anchors out of residual-
consistency.  See `docs/sv-lifecycle-and-pfr-split.md` and the
overnight data in `project_to_main_defensive_mechanisms_20260421.md`
(day0420e 01:05-01:55 50-min trap).

These tests exercise the method directly against a stub filter
and a real SvStateTracker — no EKF update, no sat positions.  We
fabricate `filt.P` so the closed-form Δx computation has known
values, then assert that join_test returns the expected reject/pass
outcome.
"""

from __future__ import annotations

import os
import sys
import unittest

import numpy as np

# ppp_ar lives in scripts/ — same path trick as test_phase_bias_gate
_SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from ppp_ar import NarrowLaneResolver           # noqa: E402
from solve_ppp import N_BASE                    # noqa: E402

from peppar_fix.sv_state import (               # noqa: E402
    SvAmbState, SvStateTracker,
)
from peppar_fix.states import AntPosEst, AntPosEstState  # noqa: E402


def _ape_sm(reached_anchored: bool = True) -> AntPosEst:
    """Build an AntPosEst state machine in the requested latch state.

    reached_anchored=True: walk through ANCHORED (both latches set
    — the state machine latches on first entry to each milestone).
    reached_anchored=False: leave the machine in its default
    SURVEYING state.

    Tests that want to exercise post-anchored behavior pass the
    resulting object as NarrowLaneResolver(ape_state_machine=...).
    """
    sm = AntPosEst()
    if reached_anchored:
        sm.transition(AntPosEstState.VERIFYING, "test")
        sm.transition(AntPosEstState.CONVERGING, "test")
        sm.transition(AntPosEstState.ANCHORING, "test")
        sm.transition(AntPosEstState.ANCHORED, "test")
    return sm


class _FakeFilter:
    """Bare-minimum PPPFilter stand-in for join-test unit tests.

    Needs .x, .P, and optionally .last_H_by_sv (the cache PPPFilter
    populates after an EKF update).
    """

    def __init__(self, n_states, H_by_sv=None):
        self.x = np.zeros(n_states)
        # Identity covariance so diagonal inversion is trivial; we'll
        # set specific off-diagonal entries in tests that need them
        # to shape Δx.
        self.P = np.eye(n_states)
        if H_by_sv is not None:
            self.last_H_by_sv = {sv: np.array(h, dtype=float)
                                 for sv, h in H_by_sv.items()}


def _put_in_long_term(tracker: SvStateTracker, sv: str,
                      elev_deg: float) -> None:
    """Walk an SV through the legal state chain to ANCHORED."""
    tracker.transition(sv, SvAmbState.FLOATING, epoch=0, reason="admit")
    tracker.transition(sv, SvAmbState.CONVERGING, epoch=1, reason="mw")
    tracker.transition(sv, SvAmbState.ANCHORING,
                       epoch=2, reason="lambda",
                       az_deg=90.0, elev_deg=elev_deg)
    tracker.transition(sv, SvAmbState.ANCHORED,
                       epoch=3, reason="delta_az >= 8°",
                       elev_deg=elev_deg)


class JoinTestTest(unittest.TestCase):
    """The join_test rejects candidates that would break anchors."""

    def setUp(self):
        self.tracker = SvStateTracker()
        # reached_anchored=True drives the strong-anchor / thin-anchor
        # regimes.  `strong_anchor_min=1` keeps the SV-anchored path
        # active for tests that only instantiate 1-2 anchors (the
        # historical default).  The production value is 3; the
        # regime-selection test class below exercises both sides of
        # that boundary directly.
        self.ape_sm = _ape_sm(reached_anchored=True)
        self.resolver = NarrowLaneResolver(
            sv_state=self.tracker,
            ape_state_machine=self.ape_sm,
            join_test_base_m=2.0,
            strong_anchor_min=1,
        )

    # ── Fast-path bypasses ───────────────────────────────────────── #

    def test_passes_when_no_tracker(self):
        """Resolver without a tracker → gate is a no-op."""
        r = NarrowLaneResolver()  # no sv_state
        filt = _FakeFilter(N_BASE + 2)
        ok, *_ = r._join_test(filt, N_BASE, fixed_value=0.0)
        self.assertTrue(ok)

    def test_disabled_flag_short_circuits(self):
        """`join_test_enabled=False` must bypass the test even when
        an anchor would otherwise fail.  This is what the A/B arm
        uses to turn the gate off for a controlled comparison."""
        _put_in_long_term(self.tracker, "E05", elev_deg=60.0)
        h_e05 = [0.0] * N_BASE + [0.0, 0.0]
        h_e05[2] = 1.0
        filt = _FakeFilter(N_BASE + 2, H_by_sv={"E05": h_e05})
        si = N_BASE
        # Same configuration as test_large_delta_rejects — would
        # reject with the gate enabled.
        filt.P[2, si] = 10.0
        filt.P[si, 2] = 10.0
        filt.P[si, si] = 1.0
        # Instantiate a resolver with the disable flag.
        r_off = NarrowLaneResolver(
            sv_state=self.tracker,
            join_test_enabled=False,
        )
        ok, worst_sv, abs_dr, thr = r_off._join_test(
            filt, si, fixed_value=1.0,
        )
        self.assertTrue(ok)
        self.assertIsNone(worst_sv)
        self.assertEqual(abs_dr, 0.0)
        self.assertEqual(thr, 0.0)

    def test_passes_when_no_long_term_members(self):
        """Bootstrap: no anchors yet → join test trivially passes."""
        filt = _FakeFilter(N_BASE + 2)
        ok, *_ = self.resolver._join_test(filt, N_BASE, fixed_value=0.0)
        self.assertTrue(ok)

    def test_passes_when_filter_has_no_cached_H(self):
        """Before the first EKF update → no H to project against →
        pass through (nothing to test)."""
        _put_in_long_term(self.tracker, "E05", elev_deg=60.0)
        filt = _FakeFilter(N_BASE + 2)  # no H_by_sv
        # Make the candidate's ambiguity column non-zero so Δx would
        # actually be non-zero if H existed.
        filt.x[N_BASE] = 0.0
        ok, *_ = self.resolver._join_test(filt, N_BASE, fixed_value=5.0)
        self.assertTrue(ok)

    # ── Core behavior ────────────────────────────────────────────── #

    def test_small_delta_passes(self):
        """A candidate that shifts state by sub-threshold amounts
        should pass (Δresidual on all anchors < 2.0 m)."""
        _put_in_long_term(self.tracker, "E05", elev_deg=60.0)
        # H row for E05: pure position-z sensitivity.  Δresid =
        # h · dx.  With this H = [0,0,1,0,...,0] (row), Δresid =
        # Δx[2].  Δx comes from the Kalman closed-form:
        # Δx = P[:, si] * (fixed - x[si]) / (P[si,si] + R).
        # We'll set P[2, si] = 0.001 so a 1.0 m fix produces
        # ~0.001 m Δresid — far below 2.0 m threshold.
        h_e05 = [0.0] * N_BASE + [0.0, 0.0]
        h_e05[2] = 1.0  # position-z sensitivity
        filt = _FakeFilter(N_BASE + 2, H_by_sv={"E05": h_e05})
        # Shape P so candidate at index N_BASE produces small Δx[2]
        si = N_BASE
        filt.P[2, si] = 0.001
        filt.P[si, 2] = 0.001
        ok, worst_sv, abs_dr, thr = self.resolver._join_test(
            filt, si, fixed_value=1.0,
        )
        self.assertTrue(ok)
        self.assertLess(abs_dr, thr)

    def test_large_delta_rejects(self):
        """A candidate that would push an anchor's PR residual past
        the elev-weighted threshold must be rejected."""
        _put_in_long_term(self.tracker, "E05", elev_deg=60.0)
        # Same H as above but we'll crank P[2, si] so Δx[2] is huge.
        h_e05 = [0.0] * N_BASE + [0.0, 0.0]
        h_e05[2] = 1.0
        filt = _FakeFilter(N_BASE + 2, H_by_sv={"E05": h_e05})
        si = N_BASE
        # P[2, si] = 10 → Δx[2] ≈ (10 / 1) * (fixed - x[si])
        # With fixed = 1 and x[si] = 0, Δx[2] ≈ 10 m → Δresid = 10 m.
        filt.P[2, si] = 10.0
        filt.P[si, 2] = 10.0
        filt.P[si, si] = 1.0  # keep denominator = 1 + R ≈ 1
        ok, worst_sv, abs_dr, thr = self.resolver._join_test(
            filt, si, fixed_value=1.0,
        )
        self.assertFalse(ok)
        self.assertEqual(worst_sv, "E05")
        self.assertGreater(abs_dr, thr)

    def test_low_elev_threshold_relaxes(self):
        """Threshold scales up by 1/sin(elev) below 45°, matching
        FalseFixMonitor.  An anchor at 15° tolerates a larger
        Δresidual than one at 60° for the same base=2.0 m."""
        _put_in_long_term(self.tracker, "E05", elev_deg=15.0)
        h_e05 = [0.0] * N_BASE + [0.0, 0.0]
        h_e05[2] = 1.0
        filt = _FakeFilter(N_BASE + 2, H_by_sv={"E05": h_e05})
        si = N_BASE
        # Make Δx[2] ≈ 3.0 m — would reject at zenith (>2.0 m) but
        # should pass at 15° (threshold ≈ 5.46 m per docstring).
        filt.P[2, si] = 3.0
        filt.P[si, 2] = 3.0
        filt.P[si, si] = 1.0
        ok, worst_sv, abs_dr, thr = self.resolver._join_test(
            filt, si, fixed_value=1.0,
        )
        self.assertTrue(ok, f"expected pass: Δ={abs_dr:.2f} thr={thr:.2f}")
        self.assertGreater(thr, 2.0)  # threshold was relaxed
        self.assertAlmostEqual(abs_dr, 3.0, places=1)

    def test_ignores_short_term_members(self):
        """Short-term members are *not* anchors in the SV-anchored
        path — they're what the test is protecting *from*.  A
        candidate whose P-coupling is only against ANCHORING
        should pass.  We stay in strong_anchor regime by adding an
        unrelated long-term member whose H-row decouples entirely
        from this candidate."""
        # ANCHORING member with the coupling we'd want the test
        # to reject (if it were an anchor).
        self.tracker.transition("E05", SvAmbState.FLOATING,
                                epoch=0, reason="admit")
        self.tracker.transition("E05", SvAmbState.CONVERGING,
                                epoch=1, reason="mw")
        self.tracker.transition("E05", SvAmbState.ANCHORING,
                                epoch=2, reason="lambda",
                                az_deg=90.0, elev_deg=60.0)
        # Long-term member with zero H-coupling to the candidate —
        # keeps us in strong_anchor regime while providing no basis
        # for the SV-anchored gate to reject.
        _put_in_long_term(self.tracker, "E11", elev_deg=60.0)
        h_e05 = [0.0] * N_BASE + [0.0, 0.0]
        h_e05[2] = 1.0
        h_e11_all_zero = [0.0] * N_BASE + [0.0, 0.0]
        filt = _FakeFilter(
            N_BASE + 2,
            H_by_sv={"E05": h_e05, "E11": h_e11_all_zero},
        )
        si = N_BASE
        filt.P[2, si] = 10.0
        filt.P[si, 2] = 10.0
        filt.P[si, si] = 1.0
        ok, *_ = self.resolver._join_test(filt, si, fixed_value=1.0)
        self.assertTrue(ok, "short-term members must not anchor the test")

    def test_multiple_anchors_uses_worst(self):
        """With two long-term members, the reject is triggered by
        the one whose Δresid exceeds threshold, not by any that pass."""
        _put_in_long_term(self.tracker, "E05", elev_deg=60.0)
        _put_in_long_term(self.tracker, "E10", elev_deg=60.0)
        # E05 has near-zero sensitivity; E10 has huge sensitivity.
        h_e05 = [0.0] * N_BASE + [0.0, 0.0]
        h_e05[2] = 1.0
        h_e10 = [0.0] * N_BASE + [0.0, 0.0]
        h_e10[0] = 1.0  # position-x sensitivity
        filt = _FakeFilter(
            N_BASE + 2, H_by_sv={"E05": h_e05, "E10": h_e10},
        )
        si = N_BASE
        filt.P[2, si] = 0.001       # E05: tiny Δresid via z
        filt.P[si, 2] = 0.001
        filt.P[0, si] = 10.0        # E10: large Δresid via x
        filt.P[si, 0] = 10.0
        filt.P[si, si] = 1.0
        ok, worst_sv, abs_dr, thr = self.resolver._join_test(
            filt, si, fixed_value=1.0,
        )
        self.assertFalse(ok)
        self.assertEqual(worst_sv, "E10")


class PredictFixDxTest(unittest.TestCase):
    """Closed-form Δx matches what _apply_fix actually produces.

    Makes sure the prediction the join test relies on agrees with
    the full Kalman update _apply_fix does.  Any divergence would
    make the join test lie — rejecting fixes that would be fine,
    or admitting fixes that would cause damage.
    """

    def test_predict_matches_apply(self):
        r = NarrowLaneResolver()
        filt_a = _FakeFilter(N_BASE + 2)
        # Shape P with cross-covariance between state[2] and
        # candidate state[N_BASE].
        filt_a.P[2, N_BASE] = 0.7
        filt_a.P[N_BASE, 2] = 0.7
        filt_a.P[N_BASE, N_BASE] = 2.0
        # Copy before mutation.
        filt_b = _FakeFilter(N_BASE + 2)
        filt_b.P = filt_a.P.copy()
        filt_b.x = filt_a.x.copy()
        # Predicted Δx.
        dx_predicted = NarrowLaneResolver._predict_fix_dx(
            filt_a, N_BASE, fixed_value=5.0,
        )
        # Actual Δx from _apply_fix.
        x_before = filt_b.x.copy()
        r._apply_fix(filt_b, N_BASE, 5.0)
        dx_actual = filt_b.x - x_before
        # Must match closely (float roundoff OK).
        np.testing.assert_allclose(dx_predicted, dx_actual, atol=1e-9)


if __name__ == "__main__":
    unittest.main()

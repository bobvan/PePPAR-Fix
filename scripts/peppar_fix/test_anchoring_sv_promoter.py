"""Unit tests for the AnchoringSvPromoter (short-term → long-term promotion).

Covers Δaz accumulation, the 15° threshold, clean-window enforcement
against prior false-fix rejections, eligibility (ANCHORING only),
and the circular az-delta helper.
"""

from __future__ import annotations

import unittest

from peppar_fix.sv_state import SvAmbState, SvStateTracker
from peppar_fix.anchoring_sv_promoter import AnchoringSvPromoter, _az_delta


class AzDeltaTest(unittest.TestCase):
    def test_basic(self):
        self.assertAlmostEqual(_az_delta(10.0, 25.0), 15.0)
        self.assertAlmostEqual(_az_delta(25.0, 10.0), 15.0)

    def test_wraparound(self):
        self.assertAlmostEqual(_az_delta(5.0, 355.0), 10.0)
        self.assertAlmostEqual(_az_delta(355.0, 5.0), 10.0)

    def test_max_is_180(self):
        # A diameter is the worst case; half-circle in either direction.
        self.assertAlmostEqual(_az_delta(0.0, 180.0), 180.0)
        self.assertAlmostEqual(_az_delta(90.0, 270.0), 180.0)


class PromoterEligibilityTest(unittest.TestCase):
    def setUp(self):
        self.t = SvStateTracker()
        self.p = AnchoringSvPromoter(
            self.t,
            dphi_threshold_deg=15.0,
            clean_window_epochs=30,
            eval_every=10,
        )

    def _to_short(self, sv, az_at_fix):
        self.t.transition(sv, SvAmbState.FLOATING, epoch=0)
        self.t.transition(sv, SvAmbState.CONVERGING, epoch=1)
        self.t.transition(
            sv, SvAmbState.ANCHORING,
            epoch=2, reason="test", az_deg=az_at_fix,
        )

    def test_ignores_svs_not_in_short(self):
        # SV in FLOATING — ingest is a no-op and doesn't fire.
        self.t.transition("G01", SvAmbState.FLOATING, epoch=0)
        self.p.ingest_az("G01", 30.0)
        self.p.ingest_az("G01", 60.0)
        events = self.p.evaluate(10)
        self.assertEqual(events, [])

    def test_does_not_promote_below_threshold(self):
        self._to_short("E01", az_at_fix=100.0)
        self.p.ingest_az("E01", 100.0)
        self.p.ingest_az("E01", 105.0)   # 5°
        self.p.ingest_az("E01", 110.0)   # +5° → total 10°
        self.assertEqual(self.p.evaluate(10), [])
        self.assertIs(self.t.state("E01"), SvAmbState.ANCHORING)

    def test_promotes_on_threshold(self):
        self._to_short("E02", az_at_fix=200.0)
        self.p.ingest_az("E02", 200.0)
        self.p.ingest_az("E02", 210.0)   # 10°
        self.p.ingest_az("E02", 220.0)   # +10° → total 20°
        events = self.p.evaluate(10)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['sv'], "E02")
        self.assertGreaterEqual(events[0]['accumulated_dphi_deg'], 15.0)
        self.assertIs(self.t.state("E02"), SvAmbState.ANCHORED)

    def test_does_not_fire_on_non_eval_epochs(self):
        self._to_short("E03", az_at_fix=0.0)
        for a in (5.0, 10.0, 15.0, 20.0, 25.0):
            self.p.ingest_az("E03", a)
        # Epoch 7 isn't an eval moment; no promotion even though
        # threshold reached.
        self.assertEqual(self.p.evaluate(7), [])
        # Next eval at 10 fires.
        events = self.p.evaluate(10)
        self.assertEqual(len(events), 1)


class PromoterCleanWindowTest(unittest.TestCase):
    def setUp(self):
        self.t = SvStateTracker()
        self.p = AnchoringSvPromoter(
            self.t,
            dphi_threshold_deg=15.0,
            clean_window_epochs=30,
            eval_every=10,
        )

    def _fresh_fix(self, sv, az, *, epoch=1):
        # Use unique epoch per call so the tracker doesn't no-op on
        # a repeated same-state transition.
        if self.t.state(sv) is SvAmbState.TRACKING:
            self.t.transition(sv, SvAmbState.FLOATING, epoch=epoch - 1)
        elif self.t.state(sv) is not SvAmbState.FLOATING:
            self.t.transition(sv, SvAmbState.FLOATING, epoch=epoch - 1,
                              reason="test-reset")
        self.t.transition(sv, SvAmbState.CONVERGING, epoch=epoch)
        self.t.transition(
            sv, SvAmbState.ANCHORING,
            epoch=epoch + 1, reason="test", az_deg=az,
        )

    def test_stalls_promotion_after_false_fix_rejection(self):
        self._fresh_fix("E10", 0.0)
        # Reach threshold.
        for a in (5.0, 10.0, 15.0, 20.0):
            self.p.ingest_az("E10", a)
        # False-fix rejects it; tracker goes ANCHORING → FLOATING.
        self.p.note_false_fix_rejection("E10", epoch=10)
        self.t.transition("E10", SvAmbState.FLOATING, epoch=10,
                          reason="false_fix:synthetic")
        self.assertEqual(self.p.evaluate(10), [])  # not short-term

        # A fresh WL → ANCHORING at a different az (new fix).
        self._fresh_fix("E10", 100.0, epoch=15)
        # Accumulate Δaz quickly.
        for a in (105.0, 110.0, 115.0, 120.0):
            self.p.ingest_az("E10", a)
        # Within the clean window (epoch - last_rej < 30); promotion
        # should still be stalled.
        self.assertEqual(self.p.evaluate(30), [])
        self.assertIs(self.t.state("E10"), SvAmbState.ANCHORING)

    def test_promotes_after_clean_window_elapses(self):
        self._fresh_fix("E11", 0.0)
        self.p.note_false_fix_rejection("E11", epoch=5)
        # Fresh fix after the rejection.
        self.t.transition("E11", SvAmbState.FLOATING, epoch=5,
                          reason="false_fix:synthetic")
        self._fresh_fix("E11", 50.0, epoch=20)
        for a in (55.0, 60.0, 65.0, 70.0):
            self.p.ingest_az("E11", a)
        # Eval at epoch 40 — clean window is 30, last_rej at 5 →
        # 40 - 5 = 35 ≥ 30, promote.
        events = self.p.evaluate(40)
        self.assertEqual(len(events), 1)
        self.assertIs(self.t.state("E11"), SvAmbState.ANCHORED)


class PromoterInteractionTest(unittest.TestCase):
    def test_preserves_candidate_across_slip_and_refix(self):
        """Day0419i bug: cycle slips on frequently-slipping SVs were
        dropping the promoter's accumulator, so 8° Δaz never completed
        even when integers were fine.  Accumulator now persists across
        transient state excursions (slip → FLOATING → re-fix).
        """
        t = SvStateTracker()
        p = AnchoringSvPromoter(t, dphi_threshold_deg=8.0,
                             clean_window_epochs=30, eval_every=10)
        t.transition("E20", SvAmbState.FLOATING, epoch=0)
        t.transition("E20", SvAmbState.CONVERGING, epoch=1)
        t.transition("E20", SvAmbState.ANCHORING, epoch=2,
                     reason="test", az_deg=0.0)
        # Accumulate 5° across ANCHORING epochs.
        for a in (1.0, 2.0, 3.0, 4.0, 5.0):
            p.ingest_az("E20", a)
        # Cycle slip takes SV back to FLOATING mid-probation.
        t.transition("E20", SvAmbState.FLOATING, epoch=6,
                     reason="slip:mw_jump conf=LOW")
        # During slip, ingest_az is a no-op.  Candidate preserved.
        p.ingest_az("E20", 6.0)
        # SV re-fixes shortly after.
        t.transition("E20", SvAmbState.CONVERGING, epoch=7)
        t.transition("E20", SvAmbState.ANCHORING, epoch=8,
                     reason="test:refix", az_deg=6.0)
        # Accumulate another 4° post-refix.
        for a in (7.0, 8.0, 9.0, 10.0):
            p.ingest_az("E20", a)
        # Total Δaz: 5° (pre-slip) + 4° (post-refix) ≥ 8° threshold.
        # Before the fix, re-fix would have dropped the candidate and
        # reset accumulated_dphi to 0, so promotion would not fire.
        events = p.evaluate(10)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['sv'], "E20")
        self.assertIs(t.state("E20"), SvAmbState.ANCHORED)

    def test_forget_drops_candidate(self):
        """Arc-boundary forget() (called from engine after forget_stale)
        must drop the candidate so the next arc starts clean.
        """
        t = SvStateTracker()
        p = AnchoringSvPromoter(t, dphi_threshold_deg=8.0,
                             clean_window_epochs=30, eval_every=10)
        t.transition("E21", SvAmbState.FLOATING, epoch=0)
        t.transition("E21", SvAmbState.CONVERGING, epoch=1)
        t.transition("E21", SvAmbState.ANCHORING, epoch=2,
                     reason="test", az_deg=0.0)
        for a in (5.0, 10.0):
            p.ingest_az("E21", a)
        # Arc boundary: engine decides this SV is set and drops it.
        p.forget("E21")
        # Candidate should be gone.
        self.assertNotIn("E21", p._cands)


if __name__ == "__main__":
    unittest.main()

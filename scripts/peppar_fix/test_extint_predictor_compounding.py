#!/usr/bin/env python3
"""The late-edge predictor must advance with skipped edges, not with calls.

``ExtintLateEdgeFilter`` extrapolates from the last *accepted* edge.  If the
prediction advances by only one median step per call, then after a rejection
it sits one step low — which makes the next GENUINE edge look one step late,
which rejects it, which puts the prediction two steps low.  It self-sustains.

The trigger condition is sharp: it bites as soon as the per-edge drift exceeds
``reject_ns`` (default 50 ns, so 50 ppb at 1 Hz).  That is any free-running or
re-acquiring DO — precisely when EXTINT matters most on a ``--no-ticc`` host
where it is the sole observer.

Before the fix this was masked by ``max_consecutive_rejects``: the compounding
drove rejections consecutive, the escape hatch fired, and the filter
re-baselined.  So the hatch was quietly absorbing a predictor artefact rather
than doing the job its docstring claims (catching a real phase step).

Found by BBGresearch 2026-09-19 from the same bug in their own predictor,
where it did not self-limit because theirs interleaved rejects with accepts.
"""
from __future__ import annotations

import unittest

from peppar_fix.extint_reader import ExtintLateEdgeFilter


def _run(drift_ns_per_edge, n=40, ring_at=10, ring_ns=300.0):
    """One genuine ringing artifact at `ring_at`; everything else is real."""
    f = ExtintLateEdgeFilter()
    phase, rejected = 0.0, []
    for i in range(n):
        phase += drift_ns_per_edge
        observed = phase + (ring_ns if i == ring_at else 0.0)
        if not f.accept(observed):
            rejected.append(i)
    return rejected, f


class TestPredictorDoesNotCompound(unittest.TestCase):

    def test_one_artifact_costs_one_sample_at_every_drift_rate(self):
        # 1200 ns/edge is a free-running i226 TCXO (~1.2 ppm, measured on
        # TimeHat).  60 is just past reject_ns, where the old bug switched on.
        for drift in (2.0, 20.0, 60.0, 200.0, 1200.0):
            with self.subTest(drift_ns_per_edge=drift):
                rejected, f = _run(drift)
                self.assertEqual(
                    rejected, [10],
                    f"only the injected artifact should be rejected; "
                    f"extra rejections at drift={drift} ns/edge mean the "
                    f"predictor is compounding")
                self.assertEqual(
                    f.n_rebaseline, 0,
                    "a single artifact must not force a re-baseline")

    def test_genuine_phase_step_still_rebaselines(self):
        """The escape hatch must still mean what its docstring says."""
        f = ExtintLateEdgeFilter()
        phase = 0.0
        for i in range(40):
            phase += 5.0
            if i == 15:
                phase += 5000.0          # a real 5 us phase step
            f.accept(phase)
        self.assertEqual(f.n_rebaseline, 1,
                         "a persistent late offset is a real step and must "
                         "force exactly one re-baseline")

    def test_early_edges_are_never_rejected(self):
        """One-sided: a genuine DO speed-up must always pass."""
        f = ExtintLateEdgeFilter()
        phase = 0.0
        rejected = []
        for i in range(30):
            phase += 100.0
            observed = phase - (10_000.0 if i == 20 else 0.0)   # very early
            if not f.accept(observed):
                rejected.append(i)
        self.assertNotIn(20, rejected, "early edges are never ringing")


if __name__ == "__main__":
    unittest.main()


class TestServoCurvatureLimitation(unittest.TestCase):
    """The linear predictor cannot track a servo-curved trajectory.

    Documents a KNOWN LIMITATION rather than asserting correct behaviour: a
    servo changes the DO frequency every epoch, so the phase trajectory is
    curved by design, and a median-step predictor lags it by ~a*window/2.
    These tests pin the threshold so a future change to `reject_ns` or
    `window` cannot move it silently.
    """

    @staticmethod
    def _slew(accel_ppb_per_s, n=60):
        """No ringing at all — pure servo curvature."""
        f = ExtintLateEdgeFilter()
        phase, vel, rejected = 0.0, 0.0, []
        for i in range(n):
            vel += accel_ppb_per_s
            phase += vel
            if not f.accept(phase):
                rejected.append(i)
        return [i for i in rejected if i >= f.warmup], f

    def test_gentle_slew_is_tracked_cleanly(self):
        for accel in (0.0, 1.0, 3.0, 6.0):
            with self.subTest(ppb_per_s=accel):
                rejected, f = self._slew(accel)
                self.assertEqual(rejected, [], "no ringing present, so no "
                                 "genuine edge should be rejected")
                self.assertEqual(f.n_rebaseline, 0)

    def test_fast_slew_rejects_genuine_edges(self):
        """Above ~2*reject_ns/window the filter fights the servo."""
        rejected, f = self._slew(10.0)
        self.assertGreater(len(rejected), 10,
                           "a 10 ppb/s slew should provoke the documented "
                           "rejection cascade; if this now passes cleanly the "
                           "limitation was fixed — update the class docstring")

    def test_threshold_matches_the_documented_formula(self):
        f = ExtintLateEdgeFilter()
        predicted = 2.0 * f.reject_ns / f.window
        self.assertAlmostEqual(predicted, 6.25, places=2,
                               msg="docstring quotes 6.25 ppb/s for the "
                                   "defaults; reject_ns or window changed")


class TestStatsAreObservable(unittest.TestCase):
    """The counters must be readable — see stats() docstring for why."""

    def test_stats_exposes_reject_rate_and_rebaseline(self):
        rejected, f = TestServoCurvatureLimitation._slew(25.0)
        s = f.stats()
        for key in ("n_accepted", "n_rejected", "n_rebaseline",
                    "reject_rate", "reject_ns", "window"):
            self.assertIn(key, s)
        self.assertGreater(s["reject_rate"], 0.0)
        self.assertEqual(s["n_rejected"] + s["n_accepted"],
                         f.n_rejected + f.n_accepted)

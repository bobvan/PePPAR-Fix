"""Rate-limited position glide for large survey upgrades (I-071400 E3b).

A large survey upgrade on the SAME antenna (e.g. a --nav2-bootstrap ~10 m seed
→ a cm survey pin) glides the pinned position instead of a respawn STEP, so the
FixedPosFilter clock reference — and the DO — glides instead of stepping.
Covers the GLIDE classification, the watch_loop dispatch, and the pure
per-epoch glide-step geometry.
"""
import os
import sys
import unittest
from collections import deque

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

import numpy as np  # noqa: E402
from peppar_fix.survey_state_watcher import (  # noqa: E402
    RefreshAction, SurveyRefresh, decide_refresh_action, watch_loop)
import peppar_fix_engine as eng  # noqa: E402

ANCHOR = np.array([3979160.0, -4257.0, 4968043.0])


def _refresh(ecef, sigma=0.01, mount_sn=1, mtime=1000.0):
    return SurveyRefresh(new_ecef=tuple(float(c) for c in ecef),
                         new_sigma_m=sigma, new_mount_sn=mount_sn,
                         file_mtime=mtime)


class DecideGlideTest(unittest.TestCase):
    def setUp(self):
        self.prev = _refresh(ANCHOR, mount_sn=1, mtime=1000.0)
        self.arp = ANCHOR.copy()

    def _act(self, curr, **kw):
        return decide_refresh_action(self.prev, curr, self.arp, **kw)[0]

    def test_large_delta_glide_off_is_step(self):
        curr = _refresh(ANCHOR + [10, 0, 0], mount_sn=1, mtime=2000.0)
        self.assertIs(self._act(curr), RefreshAction.STEP)

    def test_large_delta_glide_on_is_glide(self):
        curr = _refresh(ANCHOR + [10, 0, 0], mount_sn=1, mtime=2000.0)
        self.assertIs(self._act(curr, glide_large_delta=True),
                      RefreshAction.GLIDE)

    def test_mount_bump_steps_even_with_glide_on(self):
        # A physical antenna move is not a refinement of the same pin → STEP
        curr = _refresh(ANCHOR + [10, 0, 0], mount_sn=2, mtime=2000.0)
        self.assertIs(self._act(curr, glide_large_delta=True),
                      RefreshAction.STEP)

    def test_small_delta_is_slew_regardless(self):
        curr = _refresh(ANCHOR + [0.05, 0, 0], mount_sn=1, mtime=2000.0)
        self.assertIs(self._act(curr, glide_large_delta=True),
                      RefreshAction.SLEW)


class WatchLoopGlideDispatchTest(unittest.TestCase):
    def _run(self, snapshots, *, on_glide_set, glide_on):
        calls = {"slew": [], "step": [], "glide": []}
        snaps = deque(snapshots)
        watch_loop(
            survey_path="ignored",
            current_arp_ecef_fn=lambda: ANCHOR.copy(),
            on_slew=lambda r, d: calls["slew"].append(d),
            on_step=lambda r, d: calls["step"].append(d),
            on_glide=(lambda r, d: calls["glide"].append(d)) if on_glide_set
            else None,
            glide_large_delta=glide_on,
            sleep_fn=lambda _t: None,
            load_fn=lambda _p: snaps.popleft() if snaps else None,
            max_iterations=len(snapshots),
        )
        return calls

    def test_large_delta_glide_on_fires_on_glide(self):
        snaps = [_refresh(ANCHOR, mtime=1000.0),
                 _refresh(ANCHOR + [10, 0, 0], mtime=2000.0)]
        calls = self._run(snaps, on_glide_set=True, glide_on=True)
        self.assertEqual(len(calls["glide"]), 1)
        self.assertEqual(calls["step"], [])

    def test_glide_falls_back_to_on_step_when_no_on_glide(self):
        # Defensive: GLIDE action with on_glide=None routes to on_step.
        snaps = [_refresh(ANCHOR, mtime=1000.0),
                 _refresh(ANCHOR + [10, 0, 0], mtime=2000.0)]
        calls = self._run(snaps, on_glide_set=False, glide_on=True)
        self.assertEqual(len(calls["step"]), 1)
        self.assertEqual(calls["glide"], [])

    def test_glide_off_still_steps(self):
        snaps = [_refresh(ANCHOR, mtime=1000.0),
                 _refresh(ANCHOR + [10, 0, 0], mtime=2000.0)]
        calls = self._run(snaps, on_glide_set=True, glide_on=False)
        self.assertEqual(len(calls["step"]), 1)
        self.assertEqual(calls["glide"], [])


class GlideStepTest(unittest.TestCase):
    def test_moves_by_step_toward_target_not_reached(self):
        cur = np.array([0.0, 0.0, 0.0])
        tgt = np.array([10.0, 0.0, 0.0])
        new, reached = eng._glide_step(cur, tgt, 0.9)
        self.assertFalse(reached)
        np.testing.assert_allclose(new, [0.9, 0.0, 0.0])

    def test_unit_direction_diagonal(self):
        cur = np.array([0.0, 0.0, 0.0])
        tgt = np.array([3.0, 4.0, 0.0])          # |·| = 5
        new, reached = eng._glide_step(cur, tgt, 1.0)
        self.assertFalse(reached)
        np.testing.assert_allclose(new, [0.6, 0.8, 0.0])   # unit step

    def test_snaps_and_reaches_within_step(self):
        cur = np.array([9.7, 0.0, 0.0])
        tgt = np.array([10.0, 0.0, 0.0])          # 0.3 remaining < 0.9 step
        new, reached = eng._glide_step(cur, tgt, 0.9)
        self.assertTrue(reached)
        np.testing.assert_allclose(new, tgt)      # exact target, no overshoot

    def test_zero_distance_is_reached(self):
        p = np.array([1.0, 2.0, 3.0])
        new, reached = eng._glide_step(p, p, 0.9)
        self.assertTrue(reached)
        np.testing.assert_allclose(new, p)


class _FakeFilt:
    def __init__(self):
        self.pos = None


class GlideStepApplyTest(unittest.TestCase):
    """The dt-scaled glide wiring (Charlie #1 + Main dt-scaling, #272): steps
    rate·dt, clamps dt, writes filt.pos + arp_box, clears target on arrival."""

    def setUp(self):
        self.filt = _FakeFilt()
        self.arp = [np.array([0.0, 0.0, 0.0])]
        self.tgt = np.array([100.0, 0.0, 0.0])   # far target (won't reach)
        self.rate = 0.9                          # m/s

    def _apply(self, known, dt_s):
        return eng._glide_step_apply(known, self.tgt, self.rate, dt_s,
                                     self.filt, self.arp)

    def test_dt_scales_the_step_and_writes_pin(self):
        new, tgt = self._apply(np.array([0.0, 0.0, 0.0]), dt_s=1.0)
        np.testing.assert_allclose(new, [0.9, 0.0, 0.0])       # rate·1 s
        np.testing.assert_allclose(self.filt.pos, new)          # filt.pos written
        np.testing.assert_allclose(self.arp[0], new)            # arp_box written
        self.assertIsNotNone(tgt)                               # still gliding

    def test_sub_second_dt_scales_down(self):
        new, _ = self._apply(np.array([0.0, 0.0, 0.0]), dt_s=0.1)
        np.testing.assert_allclose(new, [0.09, 0.0, 0.0])       # rate·0.1 s

    def test_large_dt_is_clamped(self):
        # A stall (dt=100 s) must NOT jump the pin — clamp to max_dt_s=2 s.
        new, _ = self._apply(np.array([0.0, 0.0, 0.0]), dt_s=100.0)
        np.testing.assert_allclose(new, [1.8, 0.0, 0.0])        # rate·2 s cap

    def test_negative_dt_no_move(self):
        new, _ = self._apply(np.array([5.0, 0.0, 0.0]), dt_s=-3.0)
        np.testing.assert_allclose(new, [5.0, 0.0, 0.0])        # clamped to 0

    def test_reaches_and_clears_target(self):
        # Close to a near target: one step snaps exactly and clears (None).
        near = np.array([0.0, 0.0, 0.0])
        self.tgt = np.array([0.5, 0.0, 0.0])     # 0.5 m < rate·1 s = 0.9 m
        new, tgt = eng._glide_step_apply(near, self.tgt, self.rate, 1.0,
                                         self.filt, self.arp)
        np.testing.assert_allclose(new, [0.5, 0.0, 0.0])        # exact, no overshoot
        self.assertIsNone(tgt)                                  # target cleared
        np.testing.assert_allclose(self.filt.pos, [0.5, 0.0, 0.0])

    def test_none_arp_box_tolerated(self):
        new, _ = eng._glide_step_apply(np.array([0.0, 0.0, 0.0]), self.tgt,
                                       self.rate, 1.0, self.filt, None)
        np.testing.assert_allclose(self.filt.pos, new)          # no arp_box crash


class TestSurveyRefreshSigmaR(unittest.TestCase):
    """The tighten-on-refine WIRING (delta #277 review): `_survey_refresh_sigma_r`
    is the drain's (event → new σ_r) mapping that feeds confidence.  A regression
    here — or dropping the drain update — silently kills 'clockAccuracy tightens
    as the survey refines' (the confidence-math tests in test_confidence.py can't
    catch that; this can)."""

    def _event(self, action, sigma):
        return (action, _refresh(ANCHOR + [1, 0, 0], sigma=sigma), 1.0)

    def test_positive_survey_sigma_replaces_current(self):
        # A refined survey (cm σ) tightens the live σ_r from the bootstrap 10 m.
        self.assertAlmostEqual(
            eng._survey_refresh_sigma_r(self._event("slew", 0.02), 10.0), 0.02)
        # Same mapping on a GLIDE event (both drain sites share this helper).
        self.assertAlmostEqual(
            eng._survey_refresh_sigma_r(self._event("glide", 0.05), 10.0), 0.05)

    def test_can_also_coarsen(self):
        # Bidirectional: a later coarser survey coarsens the advertised σ_r —
        # more honest, matches delta's observation.
        self.assertAlmostEqual(
            eng._survey_refresh_sigma_r(self._event("slew", 0.30), 0.02), 0.30)

    def test_nonpositive_sigma_keeps_current(self):
        for bad in (0.0, -1.0):
            self.assertEqual(
                eng._survey_refresh_sigma_r(self._event("slew", bad), 7.0), 7.0)

    def test_malformed_event_keeps_current(self):
        # Missing/short/None event → unchanged, never raises.
        self.assertEqual(eng._survey_refresh_sigma_r(("slew",), 7.0), 7.0)
        self.assertEqual(
            eng._survey_refresh_sigma_r(("slew", object(), 1.0), 7.0), 7.0)
        self.assertIsNone(eng._survey_refresh_sigma_r(("slew",), None))


if __name__ == "__main__":
    unittest.main()

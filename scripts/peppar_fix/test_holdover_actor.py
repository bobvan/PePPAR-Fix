"""Tests for HoldoverActor — TRACKING/HOLDOVER/REACQUIRED state machine."""

import unittest

from peppar_fix.holdover_actor import HoldoverActor, HoldoverMode


class TestHoldoverActorTracking(unittest.TestCase):

    def test_starts_in_tracking(self):
        actor = HoldoverActor()
        self.assertEqual(actor.mode, HoldoverMode.TRACKING)
        self.assertFalse(actor.in_holdover)
        self.assertFalse(actor.wants_pseudo_obs)

    def test_healthy_arms_stay_tracking(self):
        actor = HoldoverActor()
        for i in range(100):
            t = actor.tick(hw_arm_healthy=True, mono_s=float(i))
            self.assertIsNone(t)
        self.assertEqual(actor.mode, HoldoverMode.TRACKING)

    def test_brief_arm_loss_no_transition(self):
        actor = HoldoverActor(t_enter_s=5.0)
        actor.tick(hw_arm_healthy=True, mono_s=0.0)
        actor.tick(hw_arm_healthy=False, mono_s=1.0)
        actor.tick(hw_arm_healthy=False, mono_s=3.0)
        actor.tick(hw_arm_healthy=True, mono_s=4.0)
        self.assertEqual(actor.mode, HoldoverMode.TRACKING)

    def test_arm_recovery_resets_absent_timer(self):
        actor = HoldoverActor(t_enter_s=5.0)
        actor.tick(hw_arm_healthy=False, mono_s=0.0)
        actor.tick(hw_arm_healthy=False, mono_s=4.0)
        actor.tick(hw_arm_healthy=True, mono_s=4.5)
        actor.tick(hw_arm_healthy=False, mono_s=5.0)
        actor.tick(hw_arm_healthy=False, mono_s=9.0)
        self.assertEqual(actor.mode, HoldoverMode.TRACKING)


class TestHoldoverActorTransitionToHoldover(unittest.TestCase):

    def test_enters_holdover_after_t_enter(self):
        actor = HoldoverActor(t_enter_s=5.0)
        actor.tick(hw_arm_healthy=False, mono_s=0.0,
                   offset_ns=42.0, offset_ready=True)
        for i in range(1, 5):
            t = actor.tick(hw_arm_healthy=False, mono_s=float(i),
                           offset_ns=42.0, offset_ready=True)
            self.assertIsNone(t)
        t = actor.tick(hw_arm_healthy=False, mono_s=5.0,
                       offset_ns=42.0, offset_ready=True)
        self.assertIsNotNone(t)
        self.assertEqual(t.prev, HoldoverMode.TRACKING)
        self.assertEqual(t.curr, HoldoverMode.HOLDOVER)
        self.assertEqual(actor.mode, HoldoverMode.HOLDOVER)
        self.assertTrue(actor.in_holdover)
        self.assertTrue(actor.wants_pseudo_obs)
        self.assertAlmostEqual(actor.frozen_offset_ns, 42.0)

    def test_no_holdover_without_offset_ready(self):
        actor = HoldoverActor(t_enter_s=5.0)
        for i in range(20):
            actor.tick(hw_arm_healthy=False, mono_s=float(i),
                       offset_ns=42.0, offset_ready=False)
        self.assertEqual(actor.mode, HoldoverMode.TRACKING)

    def test_no_holdover_without_offset_value(self):
        actor = HoldoverActor(t_enter_s=5.0)
        for i in range(20):
            actor.tick(hw_arm_healthy=False, mono_s=float(i),
                       offset_ns=None, offset_ready=True)
        self.assertEqual(actor.mode, HoldoverMode.TRACKING)


class TestHoldoverActorHoldoverMode(unittest.TestCase):

    def _enter_holdover(self, actor):
        actor.tick(hw_arm_healthy=False, mono_s=0.0,
                   offset_ns=10.0, offset_ready=True)
        for i in range(1, 6):
            actor.tick(hw_arm_healthy=False, mono_s=float(i),
                       offset_ns=10.0, offset_ready=True)
        assert actor.mode == HoldoverMode.HOLDOVER

    def test_stays_in_holdover_while_arms_absent(self):
        actor = HoldoverActor(t_enter_s=5.0)
        self._enter_holdover(actor)
        for i in range(6, 100):
            t = actor.tick(hw_arm_healthy=False, mono_s=float(i),
                           holdover_sigma_ns=0.5)
            self.assertIsNone(t)
        self.assertEqual(actor.mode, HoldoverMode.HOLDOVER)

    def test_sigma_tracked_during_holdover(self):
        actor = HoldoverActor(t_enter_s=5.0)
        self._enter_holdover(actor)
        actor.tick(hw_arm_healthy=False, mono_s=10.0,
                   holdover_sigma_ns=0.25)
        self.assertAlmostEqual(actor.last_sigma_ns, 0.25)
        actor.tick(hw_arm_healthy=False, mono_s=20.0,
                   holdover_sigma_ns=1.5)
        self.assertAlmostEqual(actor.last_sigma_ns, 1.5)

    def test_clock_class_during_holdover(self):
        actor = HoldoverActor(t_enter_s=5.0)
        self._enter_holdover(actor)
        actor.tick(hw_arm_healthy=False, mono_s=10.0,
                   holdover_sigma_ns=0.2)
        self.assertEqual(actor.clock_class(), 6)
        actor.tick(hw_arm_healthy=False, mono_s=20.0,
                   holdover_sigma_ns=0.5)
        self.assertEqual(actor.clock_class(), 7)
        actor.tick(hw_arm_healthy=False, mono_s=30.0,
                   holdover_sigma_ns=5.0)
        self.assertEqual(actor.clock_class(), 52)
        actor.tick(hw_arm_healthy=False, mono_s=40.0,
                   holdover_sigma_ns=15.0)
        self.assertEqual(actor.clock_class(), 248)

    def test_clock_class_tracking_always_6(self):
        actor = HoldoverActor()
        self.assertEqual(actor.clock_class(), 6)


class TestHoldoverActorReacquired(unittest.TestCase):

    def _enter_holdover(self, actor, t_start=0.0):
        t = t_start
        actor.tick(hw_arm_healthy=False, mono_s=t,
                   offset_ns=10.0, offset_ready=True)
        for i in range(1, 6):
            actor.tick(hw_arm_healthy=False, mono_s=t + float(i),
                       offset_ns=10.0, offset_ready=True)
        assert actor.mode == HoldoverMode.HOLDOVER
        return t + 5.0

    def test_arm_return_enters_reacquired(self):
        actor = HoldoverActor(t_enter_s=5.0)
        t = self._enter_holdover(actor)
        trans = actor.tick(hw_arm_healthy=True, mono_s=t + 1.0)
        self.assertIsNotNone(trans)
        self.assertEqual(trans.prev, HoldoverMode.HOLDOVER)
        self.assertEqual(trans.curr, HoldoverMode.REACQUIRED)
        self.assertEqual(actor.mode, HoldoverMode.REACQUIRED)
        self.assertTrue(actor.in_reacquired)
        self.assertTrue(actor.wants_pseudo_obs)

    def test_reacquired_exits_on_low_p22(self):
        actor = HoldoverActor(t_enter_s=5.0, sigma_blend_done_ns2=1.0)
        t = self._enter_holdover(actor)
        actor.tick(hw_arm_healthy=True, mono_s=t + 1.0)
        self.assertEqual(actor.mode, HoldoverMode.REACQUIRED)
        trans = actor.tick(hw_arm_healthy=True, mono_s=t + 2.0, p22=0.5)
        self.assertIsNotNone(trans)
        self.assertEqual(trans.curr, HoldoverMode.TRACKING)
        self.assertFalse(actor.wants_pseudo_obs)

    def test_reacquired_exits_on_t_blend_timeout(self):
        actor = HoldoverActor(t_enter_s=5.0, t_blend_s=30.0,
                              sigma_blend_done_ns2=0.001)
        t = self._enter_holdover(actor)
        actor.tick(hw_arm_healthy=True, mono_s=t + 1.0)
        self.assertEqual(actor.mode, HoldoverMode.REACQUIRED)
        for i in range(2, 30):
            actor.tick(hw_arm_healthy=True, mono_s=t + float(i), p22=5.0)
        self.assertEqual(actor.mode, HoldoverMode.REACQUIRED)
        trans = actor.tick(hw_arm_healthy=True, mono_s=t + 31.0, p22=5.0)
        self.assertIsNotNone(trans)
        self.assertEqual(trans.curr, HoldoverMode.TRACKING)

    def test_reacquired_falls_back_to_holdover_on_arm_loss(self):
        actor = HoldoverActor(t_enter_s=5.0)
        t = self._enter_holdover(actor)
        actor.tick(hw_arm_healthy=True, mono_s=t + 1.0)
        self.assertEqual(actor.mode, HoldoverMode.REACQUIRED)
        trans = actor.tick(hw_arm_healthy=False, mono_s=t + 2.0)
        self.assertIsNotNone(trans)
        self.assertEqual(trans.prev, HoldoverMode.REACQUIRED)
        self.assertEqual(trans.curr, HoldoverMode.HOLDOVER)


class TestHoldoverActorSymmetricBlend(unittest.TestCase):
    """Charlie's review requirement: REACQUIRED blend must converge
    cleanly whether x[2]_pseudo > x[2]_hw or x[2]_pseudo < x[2]_hw.
    Both directions must reach TRACKING via the same P[2,2] gate."""

    def _enter_holdover_and_reacquire(self, actor, t_start=0.0):
        t = t_start
        actor.tick(hw_arm_healthy=False, mono_s=t,
                   offset_ns=10.0, offset_ready=True)
        for i in range(1, 6):
            actor.tick(hw_arm_healthy=False, mono_s=t + float(i),
                       offset_ns=10.0, offset_ready=True)
        actor.tick(hw_arm_healthy=True, mono_s=t + 6.0)
        assert actor.mode == HoldoverMode.REACQUIRED
        return t + 6.0

    def test_blend_converges_pseudo_above_hw(self):
        """Simulate P[2,2] decaying from 10 → 0.5 (pseudo was above hw)."""
        actor = HoldoverActor(t_enter_s=5.0, sigma_blend_done_ns2=1.0)
        t = self._enter_holdover_and_reacquire(actor)
        p22_decay = [10.0, 5.0, 2.0, 1.5, 1.1, 0.8]
        for i, p in enumerate(p22_decay):
            trans = actor.tick(hw_arm_healthy=True, mono_s=t + 1.0 + i, p22=p)
            if p <= 1.0:
                self.assertIsNotNone(trans)
                self.assertEqual(trans.curr, HoldoverMode.TRACKING)
                break
        else:
            self.fail("Should have reached TRACKING")

    def test_blend_converges_pseudo_below_hw(self):
        """Same decay profile — blend direction is irrelevant to the
        P[2,2] gate, which only checks magnitude."""
        actor = HoldoverActor(t_enter_s=5.0, sigma_blend_done_ns2=1.0)
        t = self._enter_holdover_and_reacquire(actor)
        p22_decay = [8.0, 4.0, 2.5, 1.2, 0.9]
        for i, p in enumerate(p22_decay):
            trans = actor.tick(hw_arm_healthy=True, mono_s=t + 1.0 + i, p22=p)
            if p <= 1.0:
                self.assertIsNotNone(trans)
                self.assertEqual(trans.curr, HoldoverMode.TRACKING)
                break
        else:
            self.fail("Should have reached TRACKING")

    def test_blend_both_directions_same_threshold(self):
        """Directly verify: the threshold is sigma_blend_done_ns2,
        regardless of the sign of (pseudo - hw)."""
        actor = HoldoverActor(t_enter_s=5.0, sigma_blend_done_ns2=2.0)
        t = self._enter_holdover_and_reacquire(actor)
        trans = actor.tick(hw_arm_healthy=True, mono_s=t + 1.0, p22=2.1)
        self.assertIsNone(trans)
        trans = actor.tick(hw_arm_healthy=True, mono_s=t + 2.0, p22=1.9)
        self.assertIsNotNone(trans)
        self.assertEqual(trans.curr, HoldoverMode.TRACKING)


class TestHoldoverActorFullCycle(unittest.TestCase):

    def test_tracking_holdover_reacquired_tracking(self):
        actor = HoldoverActor(t_enter_s=3.0, sigma_blend_done_ns2=1.0)
        # TRACKING
        for i in range(10):
            actor.tick(hw_arm_healthy=True, mono_s=float(i))
        self.assertEqual(actor.mode, HoldoverMode.TRACKING)
        # Arms go down → HOLDOVER
        actor.tick(hw_arm_healthy=False, mono_s=10.0,
                   offset_ns=5.0, offset_ready=True)
        actor.tick(hw_arm_healthy=False, mono_s=11.0,
                   offset_ns=5.0, offset_ready=True)
        actor.tick(hw_arm_healthy=False, mono_s=12.0,
                   offset_ns=5.0, offset_ready=True)
        t = actor.tick(hw_arm_healthy=False, mono_s=13.0,
                       offset_ns=5.0, offset_ready=True)
        self.assertIsNotNone(t)
        self.assertEqual(actor.mode, HoldoverMode.HOLDOVER)
        self.assertAlmostEqual(actor.frozen_offset_ns, 5.0)
        # Stay in holdover
        actor.tick(hw_arm_healthy=False, mono_s=20.0,
                   holdover_sigma_ns=0.8)
        self.assertEqual(actor.mode, HoldoverMode.HOLDOVER)
        # Arms return → REACQUIRED
        t = actor.tick(hw_arm_healthy=True, mono_s=25.0)
        self.assertEqual(actor.mode, HoldoverMode.REACQUIRED)
        # P[2,2] drops → TRACKING
        t = actor.tick(hw_arm_healthy=True, mono_s=26.0, p22=0.5)
        self.assertIsNotNone(t)
        self.assertEqual(actor.mode, HoldoverMode.TRACKING)
        self.assertIsNone(actor.frozen_offset_ns)

    def test_double_holdover_cycle(self):
        actor = HoldoverActor(t_enter_s=3.0, sigma_blend_done_ns2=1.0)
        for cycle in range(2):
            base = cycle * 100.0
            for i in range(5):
                actor.tick(hw_arm_healthy=True, mono_s=base + i)
            actor.tick(hw_arm_healthy=False, mono_s=base + 5,
                       offset_ns=float(cycle), offset_ready=True)
            for i in range(3):
                actor.tick(hw_arm_healthy=False, mono_s=base + 6 + i,
                           offset_ns=float(cycle), offset_ready=True)
            self.assertEqual(actor.mode, HoldoverMode.HOLDOVER)
            actor.tick(hw_arm_healthy=True, mono_s=base + 20)
            self.assertEqual(actor.mode, HoldoverMode.REACQUIRED)
            actor.tick(hw_arm_healthy=True, mono_s=base + 21, p22=0.5)
            self.assertEqual(actor.mode, HoldoverMode.TRACKING)


class TestHoldoverActorEdgeCases(unittest.TestCase):

    def test_last_transition_stored(self):
        actor = HoldoverActor(t_enter_s=3.0)
        self.assertIsNone(actor.last_transition)
        actor.tick(hw_arm_healthy=False, mono_s=0.0,
                   offset_ns=1.0, offset_ready=True)
        for i in range(1, 4):
            actor.tick(hw_arm_healthy=False, mono_s=float(i),
                       offset_ns=1.0, offset_ready=True)
        self.assertIsNotNone(actor.last_transition)
        self.assertEqual(actor.last_transition.curr, HoldoverMode.HOLDOVER)

    def test_frozen_offset_cleared_on_tracking_reentry(self):
        actor = HoldoverActor(t_enter_s=3.0, sigma_blend_done_ns2=1.0)
        actor.tick(hw_arm_healthy=False, mono_s=0.0,
                   offset_ns=99.0, offset_ready=True)
        for i in range(1, 4):
            actor.tick(hw_arm_healthy=False, mono_s=float(i),
                       offset_ns=99.0, offset_ready=True)
        self.assertAlmostEqual(actor.frozen_offset_ns, 99.0)
        actor.tick(hw_arm_healthy=True, mono_s=10.0)
        actor.tick(hw_arm_healthy=True, mono_s=11.0, p22=0.5)
        self.assertEqual(actor.mode, HoldoverMode.TRACKING)
        self.assertIsNone(actor.frozen_offset_ns)


if __name__ == "__main__":
    unittest.main()

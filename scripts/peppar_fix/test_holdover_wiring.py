"""Integration tests for holdover_wiring — full Phase D stack."""

import math
import types
import unittest

import numpy as np

from peppar_fix.holdover_wiring import (
    HoldoverState, TickResult, init_holdover, tick_holdover,
    add_holdover_args, EXIT_NO_ANCHOR,
)
from peppar_fix.holdover_actor import HoldoverMode


def _mock_args(**overrides):
    """Create a minimal args namespace for holdover init."""
    defaults = {
        'servo': '/dev/ptp0',
        'servo_input': 'tdcp',
        'no_tdcp_arm': False,
        'holdover_offset_tau_s': 120.0,
        'holdover_bias_ppb': 0.0,
        'holdover_max_gap_s': 1.5,
        'holdover_t_enter_s': 5.0,
        'holdover_t_blend_s': 30.0,
        'holdover_sigma_blend_done_ns2': 1.0,
        'holdover_sigma_cc6_ns': 0.354,
        'holdover_sigma_cc7_ns': 1.0,
        'holdover_sigma_cc52_ns': 10.0,
    }
    defaults.update(overrides)
    return types.SimpleNamespace(**defaults)


def _mock_servo(x0=0.0, x1=0.0, x2=0.0, x3=0.0,
                p00=0.01, p11=0.01, p22=0.01, p33=0.01):
    """Create a mock DOFreqEst with x and P arrays."""
    servo = types.SimpleNamespace()
    servo.x = np.array([x0, x1, x2, x3])
    servo.P = np.diag([p00, p11, p22, p33])
    return servo


class TestInitHoldover(unittest.TestCase):

    def test_creates_state_with_tdcp_servo_input(self):
        args = _mock_args()
        state = init_holdover(args)
        self.assertIsNotNone(state)
        self.assertIsNotNone(state.offset_cap)
        self.assertIsNotNone(state.integrator)
        self.assertIsNotNone(state.actor)

    def test_works_without_phc_servo(self):
        """DAC+OCXO hosts have servo=None but holdover still works."""
        args = _mock_args(servo=None)
        state = init_holdover(args)
        self.assertIsNotNone(state)

    def test_returns_none_with_default_servo_input(self):
        args = _mock_args(servo_input='default')
        self.assertIsNone(init_holdover(args))

    def test_returns_none_with_no_tdcp_arm(self):
        args = _mock_args(no_tdcp_arm=True)
        self.assertIsNone(init_holdover(args))

    def test_passes_tau_s_to_offset_cap(self):
        args = _mock_args(holdover_offset_tau_s=200.0)
        state = init_holdover(args)
        self.assertEqual(state.offset_cap.tau_s, 200.0)

    def test_passes_bias_to_offset_cap(self):
        args = _mock_args(holdover_bias_ppb=0.005)
        state = init_holdover(args)
        self.assertEqual(state.offset_cap.measured_bias_ppb, 0.005)


class TestTickHoldoverTracking(unittest.TestCase):

    def test_tracking_feeds_offset_ema(self):
        state = init_holdover(_mock_args())
        servo = _mock_servo(x0=100.0, x2=105.0)
        result = tick_holdover(state, servo, None, 0.026, 1.0, True)
        self.assertEqual(result.mode, HoldoverMode.TRACKING)
        self.assertAlmostEqual(state.offset_cap.last_offset_ns, 5.0)
        self.assertIsNone(result.pseudo_obs_ns)
        self.assertEqual(result.clock_class, 6)

    def test_no_pseudo_obs_during_tracking(self):
        state = init_holdover(_mock_args())
        servo = _mock_servo()
        for i in range(20):
            result = tick_holdover(state, servo, 1e-9, 0.026, float(i), True)
        self.assertIsNone(result.pseudo_obs_ns)


class TestTickHoldoverFullCycle(unittest.TestCase):

    def _warm_up_tracking(self, state, servo, n=15):
        """Feed enough tracking epochs for offset to be ready."""
        for i in range(n):
            tick_holdover(state, servo, 1e-9, 0.026, float(i), True)
        self.assertTrue(state.offset_cap.ready)
        return float(n)

    def test_tracking_to_holdover_to_reacquired_to_tracking(self):
        args = _mock_args(holdover_t_enter_s=3.0,
                          holdover_sigma_blend_done_ns2=1.0)
        state = init_holdover(args)
        servo = _mock_servo(x0=10.0, x2=15.0)

        # Warm up offset EMA.
        t = self._warm_up_tracking(state, servo)

        # Arms go down → after T_enter → HOLDOVER (1s spacing).
        for i in range(4):
            result = tick_holdover(state, servo, 1e-9, 0.026,
                                   t + float(i), False)
        self.assertEqual(result.mode, HoldoverMode.HOLDOVER)

        # Pseudo-obs should be available during holdover (1s later).
        result = tick_holdover(state, servo, 1e-9, 0.026,
                               t + 4.0, False)
        self.assertEqual(result.mode, HoldoverMode.HOLDOVER)
        self.assertIsNotNone(result.pseudo_obs_ns)
        self.assertGreater(result.holdover_sigma_ns, 0.0)

        # Arms return → REACQUIRED (1s later).
        result = tick_holdover(state, servo, 1e-9, 0.026,
                               t + 5.0, True)
        self.assertEqual(result.mode, HoldoverMode.REACQUIRED)

        # P[2,2] drops → TRACKING.
        servo.P[2, 2] = 0.5
        result = tick_holdover(state, servo, 1e-9, 0.026,
                               t + 6.0, True)
        self.assertEqual(result.mode, HoldoverMode.TRACKING)
        self.assertIsNone(result.pseudo_obs_ns)

    def test_holdover_sigma_grows_with_duration(self):
        args = _mock_args(holdover_t_enter_s=3.0)
        state = init_holdover(args)
        servo = _mock_servo(x0=0.0, x2=5.0)
        t = self._warm_up_tracking(state, servo)

        for i in range(4):
            tick_holdover(state, servo, 1e-9, 0.026, t + i, False)

        r1 = tick_holdover(state, servo, 1e-9, 0.026, t + 10.0, False)
        r2 = tick_holdover(state, servo, 1e-9, 0.026, t + 60.0, False)
        self.assertGreater(r2.holdover_sigma_ns, r1.holdover_sigma_ns)

    def test_clock_class_degrades_with_sigma(self):
        args = _mock_args(holdover_t_enter_s=3.0)
        state = init_holdover(args)
        servo = _mock_servo(x0=0.0, x2=5.0, p22=0.001)
        t = self._warm_up_tracking(state, servo)

        for i in range(4):
            tick_holdover(state, servo, 1e-9, 0.026, t + i, False)

        # Short holdover → cc6.
        r = tick_holdover(state, servo, 1e-9, 0.026, t + 5.0, False)
        self.assertEqual(r.clock_class, 6)

    def test_gap_in_holdover_mutes_pseudo_obs(self):
        args = _mock_args(holdover_t_enter_s=3.0, holdover_max_gap_s=1.5)
        state = init_holdover(args)
        servo = _mock_servo(x0=0.0, x2=5.0)
        t = self._warm_up_tracking(state, servo)

        # Enter holdover with 1s spacing.
        for i in range(4):
            tick_holdover(state, servo, 1e-9, 0.026, t + i, False)

        # Normal epoch (1s after last = no gap).
        r = tick_holdover(state, servo, 1e-9, 0.026, t + 4.0, False)
        self.assertIsNotNone(r.pseudo_obs_ns)

        # Gap (3 s > 1.5 s max from t+4 to t+7).
        r = tick_holdover(state, servo, 1e-9, 0.026, t + 7.0, False)
        self.assertIsNone(r.pseudo_obs_ns)

    def test_holdover_duration_from_wall_clock_not_integrator(self):
        """Charlie's review: use mono from tick, not integrator."""
        args = _mock_args(holdover_t_enter_s=3.0, holdover_max_gap_s=1.5)
        state = init_holdover(args)
        servo = _mock_servo(x0=0.0, x2=5.0)
        t = self._warm_up_tracking(state, servo)

        # Enter holdover at 1s spacing.
        for i in range(4):
            tick_holdover(state, servo, 1e-9, 0.026, t + i, False)
        # Holdover entered at t+3.  Normal epoch at t+4.
        tick_holdover(state, servo, 1e-9, 0.026, t + 4.0, False)
        # Gap at t+7 (3s > 1.5s) → integrator stops advancing.
        tick_holdover(state, servo, 1e-9, 0.026, t + 7.0, False)
        # After gap: wall clock is t+20 (17 s since holdover entry at t+3).
        r = tick_holdover(state, servo, 1e-9, 0.026, t + 20.0, False)
        # σ should reflect 17 s of wall-clock holdover, not integrator's
        # frozen duration.
        expected_sigma = state.offset_cap.holdover_sigma_ns(
            17.0, tdcp_sigma_y_ppb=0.026)
        self.assertAlmostEqual(r.holdover_sigma_ns, expected_sigma, places=3)

    def test_no_holdover_without_ready_offset(self):
        """Arms absent but offset not ready → stays TRACKING."""
        args = _mock_args(holdover_t_enter_s=3.0)
        state = init_holdover(args)
        servo = _mock_servo(x0=0.0, x2=5.0)
        # Only 5 updates (need ≥10 for ready).
        for i in range(5):
            tick_holdover(state, servo, 1e-9, 0.026, float(i), True)
        for i in range(5, 20):
            r = tick_holdover(state, servo, 1e-9, 0.026, float(i), False)
        self.assertEqual(r.mode, HoldoverMode.TRACKING)


class TestExitCode(unittest.TestCase):

    def test_exit_no_anchor_constant(self):
        self.assertEqual(EXIT_NO_ANCHOR, 7)


if __name__ == "__main__":
    unittest.main()

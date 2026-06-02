#!/usr/bin/env python3
"""Tests for ``_request_servo_reset`` — the single funnel that routes
the binary layer + the B/C/D outlier/restep cascades through the
shared ResetBudget (exitFiveToServoReset).

Verifies the helper's contract:
  - budget absent / disabled  → "exit5" (legacy behavior preserved)
  - within budget             → "reset", EKF reset, actuator preserved,
                                convergence signal reset
  - over budget               → "exit5" (fall through to wrapper)
  - clock injected            → window logic is deterministic
"""
import sys
import types
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))


def _install_stubs():
    if "serial" not in sys.modules:
        m = types.ModuleType("serial")

        class _Serial:
            pass

        class SerialException(Exception):
            pass

        m.Serial = _Serial
        m.SerialException = SerialException
        sys.modules["serial"] = m


_install_stubs()

import importlib.util  # noqa: E402

_ENGINE_PATH = REPO / "scripts" / "peppar_fix_engine.py"
spec = importlib.util.spec_from_file_location("peppar_fix_engine", _ENGINE_PATH)
engine = importlib.util.module_from_spec(spec)
spec.loader.exec_module(engine)

from peppar_fix.do_freq_est import DOFreqEst  # noqa: E402
from peppar_fix.reset_budget import ResetBudget  # noqa: E402


class _StubConvergence:
    def __init__(self):
        self.reset_called = 0

    def reset(self):
        self.reset_called += 1


def _ctx(budget, *, with_convergence=True):
    servo = DOFreqEst()
    servo.freq = -37.5  # distinct, to confirm it's preserved
    ctx = {"servo": servo, "reset_budget": budget}
    if with_convergence:
        ctx["convergence"] = _StubConvergence()
    return ctx


class RequestServoResetTests(unittest.TestCase):

    def test_no_budget_returns_exit5(self):
        ctx = {"servo": DOFreqEst(), "reset_budget": None}
        self.assertEqual(engine._request_servo_reset(ctx, "ekf_outlier"),
                         "exit5")

    def test_disabled_budget_returns_exit5(self):
        b = ResetBudget(max_resets=3, window_s=300.0, enabled=False)
        ctx = _ctx(b)
        self.assertEqual(engine._request_servo_reset(ctx, "ekf_outlier"),
                         "exit5")
        # Disabled → the servo was never touched.
        self.assertEqual(b.total_allowed, 0)

    def test_within_budget_resets_and_preserves_actuator(self):
        b = ResetBudget(max_resets=3, window_s=300.0, enabled=True)
        ctx = _ctx(b)
        ctx["servo"].x[2] = 999.0  # dirty state to confirm the rebuild

        out = engine._request_servo_reset(ctx, "ekf_outlier",
                                          now=lambda: 1000.0)

        self.assertEqual(out, "reset")
        # Actuator command held at the last freq (reset(initial_freq=None)
        # defaults to self.freq).
        self.assertEqual(ctx["servo"].freq, -37.5)
        # EKF state rebuilt.
        self.assertEqual(ctx["servo"].x[2], 0.0)
        self.assertFalse(ctx["servo"]._tcxo_initialized)
        # Convergence signal reset to "far from lock".
        self.assertEqual(ctx["convergence"].reset_called, 1)
        self.assertEqual(b.cumulative_by_reason["ekf_outlier"], 1)

    def test_over_budget_falls_through_to_exit5(self):
        b = ResetBudget(max_resets=2, window_s=300.0, enabled=True)
        ctx = _ctx(b)
        t = [1000.0]
        nowf = lambda: t[0]  # noqa: E731
        self.assertEqual(engine._request_servo_reset(ctx, "ekf_outlier",
                                                     now=nowf), "reset")
        t[0] = 1010.0
        self.assertEqual(engine._request_servo_reset(ctx, "cm_outlier",
                                                     now=nowf), "reset")
        # 3rd within the window, from any reason → exit5 (shared budget).
        t[0] = 1020.0
        self.assertEqual(engine._request_servo_reset(ctx, "phc_restep",
                                                     now=nowf), "exit5")
        self.assertEqual(b.total_allowed, 2)
        self.assertEqual(b.total_denied, 1)

    def test_works_without_convergence_in_ctx(self):
        b = ResetBudget(max_resets=3, window_s=300.0, enabled=True)
        ctx = _ctx(b, with_convergence=False)
        # No 'convergence' key → helper must not KeyError.
        self.assertEqual(engine._request_servo_reset(ctx, "binary_layer",
                                                     now=lambda: 1.0),
                         "reset")


if __name__ == "__main__":
    unittest.main()

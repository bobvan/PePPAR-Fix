"""Tests for FixedPosFilter q_clk_step / q_clk_rate_step constructor knobs.

qClockTuning Phase A: expose Q[CLK,CLK] and Q[CLK_RATE,CLK_RATE] as
constructor kwargs (defaults match legacy 0.01 / 0.01 — no behavior
change without explicit opt-in).  These represent the F9T rx-TCXO
white-FM noise model; same chip across all F9T hosts so one global
value is expected, not per-host.
"""
from __future__ import annotations

import os
import sys
import unittest

import numpy as np

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from solve_ppp import FixedPosFilter

_BASE_ECEF = np.array([157470.222, -4756189.544, 4232767.952])


class QClockKwargsTest(unittest.TestCase):

    def test_default_matches_legacy(self):
        """No kwargs → legacy 0.01 / 0.01 (matches hardcoded values
        from before qClockTuning)."""
        f = FixedPosFilter(_BASE_ECEF)
        self.assertEqual(f.q_clk_step, 0.01)
        self.assertEqual(f.q_clk_rate_step, 0.01)

    def test_kwargs_override(self):
        f = FixedPosFilter(_BASE_ECEF,
                           q_clk_step=1e-4, q_clk_rate_step=1e-3)
        self.assertEqual(f.q_clk_step, 1e-4)
        self.assertEqual(f.q_clk_rate_step, 1e-3)

    def test_predict_uses_configured_q(self):
        """Predict() Q matrix gets the configured value × dt."""
        f = FixedPosFilter(_BASE_ECEF, q_clk_step=2e-3, q_clk_rate_step=5e-3)
        # Initialize P with a known value so we can read Q's addition
        # P[0,0] = 0 means after predict, P[0,0] = Q[0,0] * dt
        f.P = np.zeros((f.N_STATES, f.N_STATES))
        f.predict(1.0)
        # Q[0,0] * 1.0 = 2e-3, plus any propagation contribution (zero
        # here because P_pre was zero).
        # Use the stash from filter-state-log instrumentation:
        self.assertAlmostEqual(f.last_q_clk_step, 2e-3, places=6)
        # P[0,0] post-predict should equal q_clk_step * dt
        self.assertAlmostEqual(
            f.P[f.IDX_CLK, f.IDX_CLK], 2e-3, places=6)

    def test_predict_scales_with_dt(self):
        f = FixedPosFilter(_BASE_ECEF, q_clk_step=1e-3)
        f.P = np.zeros((f.N_STATES, f.N_STATES))
        f.predict(5.0)
        # last_q_clk_step is q_clk_step * dt = 5e-3
        self.assertAlmostEqual(f.last_q_clk_step, 5e-3, places=6)

    def test_none_kwargs_use_default(self):
        """Explicit None falls back to default — engine threads
        None through when CLI/TOML didn't set the value."""
        f = FixedPosFilter(_BASE_ECEF, q_clk_step=None, q_clk_rate_step=None)
        self.assertEqual(f.q_clk_step, 0.01)
        self.assertEqual(f.q_clk_rate_step, 0.01)

    def test_zero_q_is_allowed(self):
        """Q=0 is a legitimate setting (extreme: pure deterministic
        clock model).  Constructor must not coerce to default."""
        f = FixedPosFilter(_BASE_ECEF, q_clk_step=0.0, q_clk_rate_step=0.0)
        self.assertEqual(f.q_clk_step, 0.0)
        self.assertEqual(f.q_clk_rate_step, 0.0)


if __name__ == "__main__":
    unittest.main()

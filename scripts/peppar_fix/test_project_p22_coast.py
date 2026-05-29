"""Tests for DOFreqEst.project_p22_coast (longTauGnssCoupling engine
wiring): predict-only P[2,2] growth over an open-loop coast, the table
that feeds coast_cap_from_p22.
"""
from __future__ import annotations

import os
import sys
import unittest

import numpy as np

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.do_freq_est import DOFreqEst


class ProjectP22CoastTest(unittest.TestCase):

    def _servo(self):
        # dt=1 by default; F couples x[3] (freq) into x[2] (phase).
        return DOFreqEst(initial_dt_rx_ns=0.0)

    def test_constant_freq_variance_grows_quadratic(self):
        # No process noise, only a constant freq-state variance σf²=4.
        # Phase = ∫ freq ⇒ Var(φ,τ) = σf²·τ²  →  [0, 4, 16, 36].
        f = self._servo()
        f.P = np.diag([0.0, 0.0, 0.0, 4.0])
        f.Q = np.zeros((4, 4))
        self.assertEqual(f.project_p22_coast(3), [0.0, 4.0, 16.0, 36.0])

    def test_index0_is_current_p22_and_length(self):
        f = self._servo()
        f.P = np.diag([1.0, 1.0, 2.5, 1.0])
        out = f.project_p22_coast(10)
        self.assertEqual(len(out), 11)
        self.assertEqual(out[0], 2.5)

    def test_freq_random_walk_superlinear(self):
        # Freq RW (Q[3,3]=1) integrated into phase → super-linear growth
        # (increments accelerate), the τ³-ish coast divergence shape.
        f = self._servo()
        f.P = np.zeros((4, 4))
        f.Q = np.diag([0.0, 0.0, 0.0, 1.0])
        out = f.project_p22_coast(20)
        self.assertTrue(all(b >= a for a, b in zip(out, out[1:])))
        self.assertGreater(out[20] - out[19], out[10] - out[9])

    def test_zero_tau_returns_current(self):
        f = self._servo()
        f.P = np.diag([0.0, 0.0, 7.0, 0.0])
        self.assertEqual(f.project_p22_coast(0), [7.0])


if __name__ == "__main__":
    unittest.main()

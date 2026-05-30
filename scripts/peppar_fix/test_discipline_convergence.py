"""Tests for the DisciplineConvergence signal (disciplineModeFsm
increment #1)."""
from __future__ import annotations

import math
import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.discipline_convergence import DisciplineConvergence


class ConstructionTest(unittest.TestCase):

    def test_bootstrap_is_far(self):
        # At startup, before any update, the signal reads "far" — the
        # safe direction (consumer policies will not assume "locked").
        c = DisciplineConvergence(converged_ns=1.0, far_ns=10.0)
        self.assertEqual(c.distance_to_lock, 1.0)

    def test_far_must_exceed_converged(self):
        with self.assertRaises(ValueError):
            DisciplineConvergence(converged_ns=1.0, far_ns=1.0)
        with self.assertRaises(ValueError):
            DisciplineConvergence(converged_ns=1.0, far_ns=0.5)

    def test_converged_must_be_non_negative(self):
        with self.assertRaises(ValueError):
            DisciplineConvergence(converged_ns=-0.1, far_ns=1.0)


class UpdateFromP22Test(unittest.TestCase):

    def _c(self):
        return DisciplineConvergence(converged_ns=1.0, far_ns=10.0)

    def test_locked_returns_zero(self):
        c = self._c()
        # √(P22) = 1.0 ns = converged → m = 0.
        self.assertEqual(c.update_from_p22(1.0 ** 2), 0.0)
        self.assertEqual(c.distance_to_lock, 0.0)
        # Tighter than converged still 0 (clamped).
        self.assertEqual(c.update_from_p22(0.25 ** 2), 0.0)

    def test_far_returns_one(self):
        c = self._c()
        # √(P22) = 10 ns = far → m = 1.
        self.assertEqual(c.update_from_p22(10.0 ** 2), 1.0)
        # Beyond far still 1.
        self.assertEqual(c.update_from_p22(100.0 ** 2), 1.0)

    def test_midpoint(self):
        c = self._c()
        # √(P22) = 5.5 ns = midpoint of [1, 10] → m = 0.5.
        self.assertAlmostEqual(c.update_from_p22(5.5 ** 2), 0.5,
                               places=10)

    def test_monotone_in_p22(self):
        c = self._c()
        prev = c.update_from_p22(0.0)
        for p in (0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 64.0):
            m = c.update_from_p22(p ** 2)
            self.assertGreaterEqual(m, prev)
            prev = m

    def test_non_finite_or_negative_keeps_prior(self):
        # Transient bad reads must not jump the signal.
        c = self._c()
        c.update_from_p22(4.0 ** 2)        # m = (4-1)/9 = 0.333…
        prior = c.distance_to_lock
        self.assertAlmostEqual(prior, 1.0 / 3.0, places=6)
        for bad in (None, float("nan"), float("inf"), -1.0, "x"):
            self.assertEqual(c.update_from_p22(bad), prior)
            self.assertEqual(c.distance_to_lock, prior)

    def test_property_exposure(self):
        c = DisciplineConvergence(converged_ns=0.5, far_ns=5.0)
        self.assertEqual(c.converged_ns, 0.5)
        self.assertEqual(c.far_ns, 5.0)


class IntegrationWithGradedIntervalTest(unittest.TestCase):
    """Spot-check the consumer relationship: the signal feeds
    graded_interval (and other policies, future increments)."""

    def test_locked_signal_drives_full_coast(self):
        from peppar_fix.discipline import graded_interval
        c = DisciplineConvergence(converged_ns=1.0, far_ns=10.0)
        c.update_from_p22(0.25 ** 2)  # locked
        # m=0 → graded keeps target.
        self.assertEqual(graded_interval(120, c.distance_to_lock,
                                         min_interval=1), 120)

    def test_far_signal_drives_min_interval(self):
        from peppar_fix.discipline import graded_interval
        c = DisciplineConvergence(converged_ns=1.0, far_ns=10.0)
        c.update_from_p22(20.0 ** 2)  # far
        self.assertEqual(graded_interval(120, c.distance_to_lock,
                                         min_interval=1), 1)


if __name__ == "__main__":
    unittest.main()

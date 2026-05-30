"""Tests for the continuous-engagement + observability-override paths
added to OcxoTrustedGate (disciplineModeFsm increment #3).

The legacy age-based path is exercised by test_ocxo_trusted_gate.py;
this file covers the new ``distance_to_lock`` and ``is_sole_observer``
behavior.  Default (no new kwargs) is byte-identical to legacy.
"""
from __future__ import annotations

import math
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.dirname(_HERE)
for p in (_SCRIPTS, _HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

from peppar_fix.ocxo_trusted_gate import OcxoTrustedGate


def _g(**kw):
    """σ=0.054 ns, k=10 ⇒ legacy threshold = 0.54 ns at τ=1 s."""
    defaults = dict(sigma_short_tau_ns=0.054, k_sigma=10.0, min_age_s=0.0)
    defaults.update(kw)
    return OcxoTrustedGate(**defaults)


class SoleObserverOverrideTest(unittest.TestCase):
    """is_sole_observer=True ⇒ always admit, regardless of innov / m /
    age (Main's observability refinement on #89; sole-observer memory)."""

    def test_admits_huge_innov(self):
        g = _g(is_sole_observer=True)
        accept, reason = g.evaluate(innov_ns=10000.0, dt_s=1.0, age_s=1000.0)
        self.assertTrue(accept)
        self.assertEqual(reason, "sole_observer")
        self.assertEqual(g.n_admitted_sole_observer, 1)
        self.assertEqual(g.n_rejected, 0)

    def test_admits_under_distance_to_lock(self):
        # Observability beats m: even when locked (m=0, gate fully on),
        # a sole carrier is admitted.
        g = _g(is_sole_observer=True)
        accept, reason = g.evaluate(innov_ns=10000.0, dt_s=1.0,
                                    distance_to_lock=0.0)
        self.assertTrue(accept)
        self.assertEqual(reason, "sole_observer")

    def test_default_off(self):
        # Default is_sole_observer=False ⇒ legacy behavior.
        g = _g()
        self.assertFalse(g.is_sole_observer)
        self.assertIn("is_sole_observer", g.stats)
        self.assertFalse(g.stats["is_sole_observer"])


class ContinuousEngagementTest(unittest.TestCase):
    """distance_to_lock ∈ [0,1] (0=locked, 1=far) supersedes age_s.
    Engagement = max(0, 1−m); threshold = k·σ·√dt / engagement."""

    def test_locked_matches_legacy_threshold(self):
        # m=0 ⇒ engagement=1 ⇒ threshold = today's k·σ·√dt = 0.54 ns.
        g = _g()
        self.assertTrue(g.evaluate(innov_ns=0.5, dt_s=1.0,
                                    distance_to_lock=0.0)[0])
        self.assertFalse(g.evaluate(innov_ns=0.6, dt_s=1.0,
                                     distance_to_lock=0.0)[0])

    def test_far_admits_all(self):
        # m=1 ⇒ engagement=0 ⇒ gate effectively off.
        g = _g()
        accept, reason = g.evaluate(innov_ns=10000.0, dt_s=1.0,
                                    distance_to_lock=1.0)
        self.assertTrue(accept)
        self.assertEqual(reason, "off_far_from_lock")
        self.assertEqual(g.n_admitted_continuous_off, 1)

    def test_midpoint_doubles_threshold(self):
        # m=0.5 ⇒ engagement=0.5 ⇒ threshold = 2·k·σ·√dt = 1.08 ns.
        g = _g()
        self.assertTrue(g.evaluate(innov_ns=1.0, dt_s=1.0,
                                    distance_to_lock=0.5)[0])
        self.assertFalse(g.evaluate(innov_ns=1.1, dt_s=1.0,
                                     distance_to_lock=0.5)[0])

    def test_monotone_loosening_in_m(self):
        # For a fixed innov just above the locked threshold, the gate
        # rejects when locked but admits as m grows.
        innov = 0.6  # > 0.54 ns locked threshold
        results = []
        for m in (0.0, 0.1, 0.3, 0.5, 0.9, 1.0):
            g = _g()
            accept, _ = g.evaluate(innov_ns=innov, dt_s=1.0,
                                    distance_to_lock=m)
            results.append((m, accept))
        # First False (locked), eventually True as m grows; once True,
        # stays True (monotone loosening).
        self.assertEqual(results[0][1], False)
        self.assertEqual(results[-1][1], True)
        # Monotone: no flip from True back to False as m increases.
        seen_true = False
        for _, a in results:
            if a:
                seen_true = True
            elif seen_true:
                self.fail(f"non-monotone gate decisions: {results}")

    def test_distance_supersedes_age_when_both(self):
        # min_age_s=999 (legacy gate would skip pre-age), but
        # distance_to_lock=0 forces the continuous path → threshold
        # check applies even though age_s < min_age.
        g = _g(min_age_s=999.0)
        accept, _ = g.evaluate(innov_ns=10.0, dt_s=1.0, age_s=1.0,
                               distance_to_lock=0.0)
        self.assertFalse(accept)  # would have been "pre_min_age" legacy

    def test_legacy_path_preserved_without_distance(self):
        # No distance_to_lock → legacy age-based behavior.
        g = _g(min_age_s=999.0)
        accept, reason = g.evaluate(innov_ns=10.0, dt_s=1.0, age_s=1.0)
        self.assertTrue(accept)
        self.assertEqual(reason, "pre_min_age")


class StatsTest(unittest.TestCase):

    def test_new_counters_in_stats(self):
        g = _g()
        s = g.stats
        self.assertIn("n_admitted_sole_observer", s)
        self.assertIn("n_admitted_continuous_off", s)
        self.assertIn("is_sole_observer", s)

    def test_counters_tracked(self):
        g_sole = _g(is_sole_observer=True)
        g_sole.evaluate(innov_ns=1.0, dt_s=1.0, age_s=1000.0)
        g_sole.evaluate(innov_ns=100.0, dt_s=1.0, age_s=1000.0)
        self.assertEqual(g_sole.stats["n_admitted_sole_observer"], 2)

        g_far = _g()
        g_far.evaluate(innov_ns=10.0, dt_s=1.0, distance_to_lock=1.0)
        g_far.evaluate(innov_ns=10.0, dt_s=1.0, distance_to_lock=1.0)
        self.assertEqual(g_far.stats["n_admitted_continuous_off"], 2)


if __name__ == "__main__":
    unittest.main()

"""Tests for the disciplineModeFsm binary layer — gross-fault detector
+ in-process reset machinery (increment #4).
"""
from __future__ import annotations

import os
import sys
import unittest

import numpy as np

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.binary_layer import BinaryLayer
from peppar_fix.discipline_convergence import DisciplineConvergence


# ─────────────────────────────────────────────────────────── BinaryLayer ──

class BinaryLayerConstructionTest(unittest.TestCase):

    def test_defaults_off(self):
        b = BinaryLayer()
        self.assertFalse(b.enabled)
        self.assertEqual(b.consec, 0)
        self.assertEqual(b.n_gross_fault_resets, 0)

    def test_rejects_invalid_consec(self):
        with self.assertRaises(ValueError):
            BinaryLayer(consec_max_epochs=0)
        with self.assertRaises(ValueError):
            BinaryLayer(consec_max_epochs=-1)

    def test_disabled_evaluates_to_none(self):
        b = BinaryLayer(enabled=False, consec_max_epochs=2)
        # Even sustained-at-1.0 input doesn't fire when disabled.
        for _ in range(10):
            self.assertIsNone(b.evaluate(distance_to_lock=1.0,
                                          in_holdover=False))
        self.assertEqual(b.consec, 0)


class BinaryLayerEvaluateTest(unittest.TestCase):

    def _b(self, **kw):
        d = dict(consec_max_epochs=3, enabled=True)
        d.update(kw)
        return BinaryLayer(**d)

    def test_sustained_max_fires(self):
        b = self._b()
        self.assertIsNone(b.evaluate(1.0, False))   # consec=1
        self.assertIsNone(b.evaluate(1.0, False))   # consec=2
        # 3rd consecutive at max → fire.
        self.assertEqual(b.evaluate(1.0, False), "gross_fault")

    def test_dip_below_max_resets_counter(self):
        b = self._b()
        b.evaluate(1.0, False)
        b.evaluate(1.0, False)
        self.assertEqual(b.consec, 2)
        # Single drop below the rail clears the counter.
        b.evaluate(0.5, False)
        self.assertEqual(b.consec, 0)
        # Re-accumulate from scratch.
        b.evaluate(1.0, False)
        self.assertEqual(b.consec, 1)

    def test_holdover_suppresses(self):
        # In-holdover ⇒ counter stays at 0 even with distance=1.0.
        b = self._b()
        for _ in range(10):
            self.assertIsNone(b.evaluate(1.0, in_holdover=True))
        self.assertEqual(b.consec, 0)

    def test_holdover_exit_then_fault_fires(self):
        # During holdover: no accrual.  After holdover ends: accrual
        # restarts and the fault fires after consec_max more epochs.
        b = self._b()
        for _ in range(5):
            b.evaluate(1.0, in_holdover=True)
        self.assertEqual(b.consec, 0)
        # GNSS returns; distance still 1.0 (sole-x[2]-carrier hasn't
        # caught up yet, or the filter is broken — binary layer says
        # "let's give it consec_max_epochs before declaring fault").
        b.evaluate(1.0, False)
        b.evaluate(1.0, False)
        self.assertEqual(b.evaluate(1.0, False), "gross_fault")

    def test_none_signal_clears_counter(self):
        # distance_to_lock is None when --graded-taper is off; binary
        # layer shouldn't accumulate a fault from a missing signal.
        b = self._b()
        b.evaluate(1.0, False)
        b.evaluate(1.0, False)
        b.evaluate(None, False)
        self.assertEqual(b.consec, 0)

    def test_max_consec_high_water_mark(self):
        b = self._b(consec_max_epochs=100)  # never fires here
        for _ in range(7):
            b.evaluate(1.0, False)
        self.assertEqual(b.consec, 7)
        self.assertEqual(b.max_consec, 7)
        # Dip resets consec but not max_consec.
        b.evaluate(0.0, False)
        for _ in range(3):
            b.evaluate(1.0, False)
        self.assertEqual(b.consec, 3)
        self.assertEqual(b.max_consec, 7)

    def test_clear_after_reset(self):
        b = self._b()
        b.evaluate(1.0, False)
        b.evaluate(1.0, False)
        b.evaluate(1.0, False)  # fires
        b.clear_after_reset()
        self.assertEqual(b.consec, 0)
        self.assertEqual(b.n_gross_fault_resets, 1)
        # Re-accumulate cleanly from a fresh window.
        for _ in range(2):
            self.assertIsNone(b.evaluate(1.0, False))
        self.assertEqual(b.evaluate(1.0, False), "gross_fault")

    def test_disabled_to_enabled_does_not_leak_counter(self):
        # Counter is forced to 0 every evaluation when disabled — no
        # carry-over if the feature is toggled mid-run.
        b = self._b(enabled=False)
        for _ in range(5):
            b.evaluate(1.0, False)
        self.assertEqual(b.consec, 0)


class BinaryLayerStatsTest(unittest.TestCase):

    def test_stats_shape(self):
        b = BinaryLayer(consec_max_epochs=5, enabled=True)
        s = b.stats
        for k in ("enabled", "consec_max_epochs", "consec",
                  "max_consec", "n_gross_fault_resets"):
            self.assertIn(k, s)
        self.assertTrue(s["enabled"])
        self.assertEqual(s["consec_max_epochs"], 5)


# ─────────────────────────────────────────────────── DisciplineConvergence ──

class DisciplineConvergenceResetTest(unittest.TestCase):

    def test_reset_clears_to_far(self):
        c = DisciplineConvergence(converged_ns=1.0, far_ns=10.0)
        c.update_from_p22(0.25 ** 2)  # locked → m=0
        self.assertEqual(c.distance_to_lock, 0.0)
        c.reset()
        self.assertEqual(c.distance_to_lock, 1.0)

    def test_reset_idempotent(self):
        c = DisciplineConvergence(converged_ns=1.0, far_ns=10.0)
        c.reset()
        c.reset()
        self.assertEqual(c.distance_to_lock, 1.0)


# ─────────────────────────────────────────────────────────────── DOFreqEst ──

# DOFreqEst is heavyweight to construct (numpy + state file work).  Use
# the same minimal-construct pattern as test_routed_qerr_arm.

from peppar_fix.do_freq_est import DOFreqEst  # noqa: E402


def _make_servo(**kw):
    defaults = dict(
        sigma_ticc_ns=0.060,
        sigma_do_phase_ns=0.05,
        sigma_do_freq_ppb=0.01,
        sigma_tcxo_phase_ns=2.0,
        sigma_tcxo_freq_ppb=0.1,
        initial_freq=42.0,
        initial_dt_rx_ns=0.0,
    )
    defaults.update(kw)
    f = DOFreqEst(**defaults)
    f._need_phc_seed = False
    return f


class DOFreqEstResetTest(unittest.TestCase):

    def test_reset_re_inits_state(self):
        f = _make_servo()
        # Mangle state to look "diverged".
        f.x = np.array([100.0, 5.0, 999.0, 50.0])
        f.P = np.diag([1e6, 1e3, 1e6, 1e3])
        f.reset()
        # x[2] (phi_do) re-init to 0; x[3] = -crystal_freq = -42.
        self.assertAlmostEqual(f.x[2], 0.0)
        self.assertAlmostEqual(f.x[3], -42.0)
        # Default reset uses the WIDER bootstrap covariance (the
        # conservative choice: gross-fault means we don't trust the
        # prior dt_rx_ns either; _tcxo_initialized → False).
        self.assertAlmostEqual(f.P[2, 2], 1000.0 ** 2)
        self.assertFalse(f._tcxo_initialized)

    def test_reset_with_known_dt_rx_uses_narrow_p(self):
        # When the caller still trusts dt_rx_ns (e.g. PPP is still
        # converged), pass it explicitly → narrow bootstrap P.
        f = _make_servo()
        f.reset(initial_dt_rx_ns=0.0)
        self.assertAlmostEqual(f.P[2, 2], 5000.0 ** 2)
        self.assertTrue(f._tcxo_initialized)

    def test_reset_preserves_actuator_freq(self):
        # Default reset (initial_freq=None) preserves the LAST freq.
        f = _make_servo(initial_freq=42.0)
        f.freq = -88.5  # simulate the servo having steered to -88.5
        f.reset()
        # _last_u and x[3] both reflect the preserved freq.
        self.assertAlmostEqual(f._last_u, -88.5)
        self.assertAlmostEqual(f.x[3], 88.5)  # x[3] = -crystal_freq
        self.assertAlmostEqual(f.freq, -88.5)

    def test_reset_with_explicit_freq(self):
        f = _make_servo()
        f.reset(initial_freq=10.0)
        self.assertAlmostEqual(f._last_u, 10.0)
        self.assertAlmostEqual(f.x[3], -10.0)

    def test_reset_clears_innov_history(self):
        f = _make_servo()
        f.last_innov = 12.0
        f.last_arm_innov["ticc"] = 3.4
        f.last_arm_S["ticc"] = 0.1
        f.last_ocxo_gate_rejected = True
        f.last_ocxo_gate_reason = "test"
        f.last_ticc_route = "ext"
        f.reset()
        self.assertEqual(f.last_innov, 0.0)
        self.assertIsNone(f.last_arm_innov["ticc"])
        self.assertIsNone(f.last_arm_S["ticc"])
        self.assertFalse(f.last_ocxo_gate_rejected)
        self.assertEqual(f.last_ocxo_gate_reason, "")
        self.assertEqual(f.last_ticc_route, "int")

    def test_reset_preserves_route_counters(self):
        # Cumulative observability counters survive reset so the
        # operator sees history across resets.
        f = _make_servo()
        f.n_route_ext = 42
        f.n_route_int = 7
        f.n_route_raw = 3
        f.reset()
        self.assertEqual(f.n_route_ext, 42)
        self.assertEqual(f.n_route_int, 7)
        self.assertEqual(f.n_route_raw, 3)


if __name__ == "__main__":
    unittest.main()

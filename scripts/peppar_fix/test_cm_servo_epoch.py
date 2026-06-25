"""Integration tests for _cm_servo_epoch — the ClockMatrix servo epoch that
feeds on-chip PHASE_STATUS into DOFreqEst (Arm 7) and drives the actuator.

Uses a real DOFreqEst with fake phase source / actuator / scheduler, so these
exercise the actual EKF arm + sign + gating glue (the part the unit tests in
test_do_freq_est_cm_phase.py can't cover because it lives in the engine).
"""
from __future__ import annotations

import os
import sys
import types
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

import peppar_fix_engine as eng
from peppar_fix.do_freq_est import DOFreqEst


class _FakePhase:
    def __init__(self, phase_ns):
        self.phase_ns = phase_ns

    def read_phase_ns(self):
        return self.phase_ns


class _FakeActuator:
    def __init__(self):
        self.commands = []

    def adjust_frequency_ppb(self, ppb):
        self.commands.append(ppb)
        return ppb


class _FakeScheduler:
    def __init__(self):
        self.interval = 1
        self.actuations = []

    def record_actuation(self, mono, ppb):
        self.actuations.append((mono, ppb))

    def compute_adaptive_interval(self):
        pass


def _args(**over):
    base = dict(track_outlier_ns=None, track_max_ppb=244_000.0,
               freerun=False, cm_phase_arm=True)
    base.update(over)
    return types.SimpleNamespace(**base)


def _ctx(phase_ns):
    return {
        'cm_phase_source': _FakePhase(phase_ns),
        'servo': DOFreqEst(initial_dt_rx_ns=0.0),
        'scheduler': _FakeScheduler(),
        'actuator': _FakeActuator(),
        'log_w': None,
    }


class CmServoEpochTest(unittest.TestCase):

    def test_feeds_phase_into_dofreqest_and_drives_actuator(self):
        ctx = _ctx(phase_ns=200.0)
        rc = eng._cm_servo_epoch(ctx, _args(), n_epochs=1,
                                 dt_rx_ns=0.0, dt_rx_sigma=0.1)
        self.assertEqual(rc, "ok")
        # cm_phase seeded x[2] inside the EKF.
        self.assertFalse(ctx['servo']._need_phc_seed)
        self.assertAlmostEqual(ctx['servo'].x[2], 200.0, delta=10.0)
        # Actuator was commanded (DO steered).
        self.assertEqual(len(ctx['actuator'].commands), 1)
        # Arm 7 logged its innovation path (fired this/subsequent epoch).
        self.assertIn('cm_phase', ctx['servo'].last_arm_innov)

    def test_no_cm_phase_gates_arm7_off(self):
        ctx = _ctx(phase_ns=200.0)
        eng._cm_servo_epoch(ctx, _args(cm_phase_arm=False), n_epochs=1,
                            dt_rx_ns=0.0, dt_rx_sigma=0.1)
        # With Arm 7 gated, cm_phase never fired → no seed from it, no innov.
        self.assertIsNone(ctx['servo'].last_arm_innov['cm_phase'])
        # Still runs the epoch (PPP-only) and commands the actuator.
        self.assertEqual(len(ctx['actuator'].commands), 1)

    def test_freerun_does_not_command_actuator(self):
        ctx = _ctx(phase_ns=50.0)
        eng._cm_servo_epoch(ctx, _args(freerun=True), n_epochs=1,
                            dt_rx_ns=0.0, dt_rx_sigma=0.1)
        self.assertEqual(len(ctx['actuator'].commands), 0)

    def test_no_phase_reading_returns_no_phase(self):
        ctx = _ctx(phase_ns=None)
        rc = eng._cm_servo_epoch(ctx, _args(), n_epochs=10,
                                 dt_rx_ns=0.0, dt_rx_sigma=0.1)
        self.assertEqual(rc, "no_phase")
        self.assertEqual(len(ctx['actuator'].commands), 0)

    def test_gross_outlier_is_gated(self):
        # phase beyond track_outlier_ns → gated, actuator not driven.
        ctx = _ctx(phase_ns=5000.0)
        rc = eng._cm_servo_epoch(ctx, _args(track_outlier_ns=1000.0),
                                 n_epochs=1, dt_rx_ns=0.0, dt_rx_sigma=0.1)
        self.assertEqual(rc, "outlier")
        self.assertEqual(len(ctx['actuator'].commands), 0)

    def test_correction_opposes_phase_error_sign(self):
        # A late DO (positive phase) → speed it up.  The EKF output drives the
        # actuator; over a few epochs at a steady +phase the commanded pull
        # should be positive (combo/adjfine +ppb = faster).
        ctx = _ctx(phase_ns=100.0)
        for _ in range(8):
            ctx['cm_phase_source'].phase_ns = 100.0
            eng._cm_servo_epoch(ctx, _args(), n_epochs=1,
                                dt_rx_ns=0.0, dt_rx_sigma=0.1)
        self.assertGreater(ctx['actuator'].commands[-1], 0.0,
                           "a persistently late DO should be commanded faster")


class BuildClockMatrixActuatorTest(unittest.TestCase):
    """_build_clockmatrix_actuator selects combo vs FCW from the measured
    characterization (the P3 actuator-selection glue)."""

    def test_combo_params_selects_combo_actuator(self):
        from peppar_fix.clockmatrix_combo_actuator import ClockMatrixComboActuator
        i2c = object()  # constructor only stores it; no hardware touched
        act, atype, one_dpll = eng._build_clockmatrix_actuator(
            i2c, cm_dpll=3, combo_params={'combo_gain': 0.978})
        self.assertIsInstance(act, ClockMatrixComboActuator)
        self.assertEqual(atype, "clockmatrix_combo")
        self.assertTrue(one_dpll)
        self.assertAlmostEqual(act._combo_gain, 0.978)
        self.assertEqual(act._dpll_id, 3)

    def test_none_selects_fcw_actuator(self):
        from peppar_fix.clockmatrix_actuator import ClockMatrixActuator
        i2c = object()
        act, atype, one_dpll = eng._build_clockmatrix_actuator(
            i2c, cm_dpll=3, combo_params=None)
        self.assertIsInstance(act, ClockMatrixActuator)
        self.assertEqual(atype, "clockmatrix")
        self.assertFalse(one_dpll)


if __name__ == "__main__":
    unittest.main()

"""Tests for the phase-conditional TADD ARM helper.

The helper lives in peppar_fix_engine but is testable via mocking the
ARM call + the phase measurement.  We import it via the engine module.
"""

from __future__ import annotations

import argparse
import sys
import unittest
from unittest.mock import patch

# Engine imports pyserial at import time; tests can't run without it.
# bin/test runs under the venv so pyserial is available.

# Bring the engine module on path.
import os
_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix_engine import _maybe_step_divider_on_phase


def _args(**kwargs):
    """Build an argparse.Namespace with the relevant fields."""
    defaults = dict(
        tadd_gpio=16,
        tadd_hold_s=1.1,
        no_divider_step=False,
        divider_step_threshold_us=5.0,
        ticc_port="/dev/ticc5",
        ticc_baud=115200,
        ticc_phc_channel="chA",
        ticc_ref_channel="chB",
    )
    defaults.update(kwargs)
    return argparse.Namespace(**defaults)


class _GateRespect(unittest.TestCase):
    """The helper should respect --no-divider-step and missing tadd_gpio."""

    def test_no_divider_step_flag_skips_arm(self):
        with patch("peppar_fix_engine._do_tadd_arm") as arm:
            self.assertFalse(_maybe_step_divider_on_phase(
                _args(no_divider_step=True)))
            arm.assert_not_called()

    def test_no_tadd_gpio_returns_false_without_arming(self):
        with patch("peppar_fix_engine._do_tadd_arm") as arm:
            self.assertFalse(_maybe_step_divider_on_phase(
                _args(tadd_gpio=None)))
            arm.assert_not_called()


class _PhaseDecision(unittest.TestCase):
    """Threshold logic on the measured phase."""

    def test_phase_above_threshold_arms(self):
        # Median phase 87 µs >> 5 µs threshold → ARM
        with patch("peppar_fix.timestamper.measure_differential_phase",
                   return_value=(87_000.0, 3)) as meas, \
             patch("peppar_fix_engine._do_tadd_arm") as arm:
            result = _maybe_step_divider_on_phase(_args())
            self.assertTrue(result)
            arm.assert_called_once()
            meas.assert_called_once()

    def test_phase_below_threshold_does_not_arm(self):
        # 0.5 µs phase < 5 µs threshold → slew, no ARM
        with patch("peppar_fix.timestamper.measure_differential_phase",
                   return_value=(500.0, 3)), \
             patch("peppar_fix_engine._do_tadd_arm") as arm:
            result = _maybe_step_divider_on_phase(_args())
            self.assertFalse(result)
            arm.assert_not_called()

    def test_phase_exactly_at_threshold_does_not_arm(self):
        # |phase| == threshold → slew, no ARM (>, not >=)
        with patch("peppar_fix.timestamper.measure_differential_phase",
                   return_value=(5_000.0, 3)), \
             patch("peppar_fix_engine._do_tadd_arm") as arm:
            self.assertFalse(_maybe_step_divider_on_phase(_args()))
            arm.assert_not_called()

    def test_negative_phase_uses_absolute_value(self):
        # DO ahead of ref by 10 µs (negative diff) → still ARM
        with patch("peppar_fix.timestamper.measure_differential_phase",
                   return_value=(-10_000.0, 3)), \
             patch("peppar_fix_engine._do_tadd_arm") as arm:
            self.assertTrue(_maybe_step_divider_on_phase(_args()))
            arm.assert_called_once()

    def test_custom_threshold_respected(self):
        # Same phase, different threshold — should NOT ARM at 50 µs.
        with patch("peppar_fix.timestamper.measure_differential_phase",
                   return_value=(20_000.0, 3)), \
             patch("peppar_fix_engine._do_tadd_arm") as arm:
            self.assertFalse(_maybe_step_divider_on_phase(
                _args(divider_step_threshold_us=50.0)))
            arm.assert_not_called()


class _Fallbacks(unittest.TestCase):
    """When measurement fails, default to ARM (conservative)."""

    def test_no_ticc_port_defaults_to_arm(self):
        with patch("peppar_fix_engine._do_tadd_arm") as arm:
            self.assertTrue(_maybe_step_divider_on_phase(
                _args(ticc_port=None)))
            arm.assert_called_once()

    def test_measure_returns_none_defaults_to_arm(self):
        with patch("peppar_fix.timestamper.measure_differential_phase",
                   return_value=(None, 0)), \
             patch("peppar_fix_engine._do_tadd_arm") as arm:
            self.assertTrue(_maybe_step_divider_on_phase(_args()))
            arm.assert_called_once()

    def test_measure_raises_defaults_to_arm(self):
        with patch("peppar_fix.timestamper.measure_differential_phase",
                   side_effect=RuntimeError("ticc offline")), \
             patch("peppar_fix_engine._do_tadd_arm") as arm:
            self.assertTrue(_maybe_step_divider_on_phase(_args()))
            arm.assert_called_once()


if __name__ == "__main__":
    unittest.main()

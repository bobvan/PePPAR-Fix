#!/usr/bin/env python3
"""Tests for ``_servo_outlier_decision`` — the servo PPS-side outlier
state machine, extracted as a pure helper from ``_servo_epoch`` so it
can be exercised without mocking the full per-epoch context.

The decision branches:

  - ``gap_recovery=True``         → reset counter, ok (PR-side bypass
                                    mirror for the PR #88 clkPoC3
                                    gap_recovery completeness fix).
  - threshold disabled            → ok.
  - converging                    → ok (scheduler bypass).
  - in-range                      → reset counter, ok.
  - out-of-range, count < 30      → increment, outlier (not exit5).
  - out-of-range, count == 30     → outlier WITH exit5 flag.
"""

import sys
import types
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))


def _install_stubs():
    """The engine imports several heavy modules at module load.  Only
    the symbols this test reaches are ``_servo_outlier_decision`` and
    the ``ctx`` dict it mutates.  Stub anything the engine module
    drags in that the test environment doesn't supply.
    """
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

# Direct import via filesystem path — peppar_fix_engine isn't packaged.
import importlib.util
_ENGINE_PATH = REPO / "scripts" / "peppar_fix_engine.py"
spec = importlib.util.spec_from_file_location("peppar_fix_engine", _ENGINE_PATH)
engine = importlib.util.module_from_spec(spec)
try:
    spec.loader.exec_module(engine)
except Exception:
    # The engine module imports a lot at top level; if any import
    # fails we still want to exercise the pure helper.  Pull just the
    # function we test via a regex extract as a fallback.
    src = _ENGINE_PATH.read_text()
    import re
    m = re.search(
        r"def _servo_outlier_decision\(.*?\n\n\ndef ",
        src, re.DOTALL)
    if m:
        ns = {}
        exec(m.group(0).rsplit("\n\ndef ", 1)[0], ns)
        engine = types.SimpleNamespace(
            _servo_outlier_decision=ns["_servo_outlier_decision"])
    else:
        raise


class ServoOutlierDecisionTests(unittest.TestCase):

    def setUp(self):
        self.ctx = {"consecutive_outliers": 0}

    def test_gap_recovery_resets_and_ok_even_when_far_out_of_range(self):
        # The clkPoC3 exit-5 scenario: pps_err =~+500 ns post-gap.
        # gap_recovery=True must NOT trip outlier (let the EKF absorb)
        # AND must reset any accumulated cascade count.
        self.ctx["consecutive_outliers"] = 29   # one shy of exit-5
        out, exit5 = engine._servo_outlier_decision(
            self.ctx,
            outlier_observable_ns=508.0,
            track_outlier_ns=100.0,
            converging=False,
            gap_recovery=True,
        )
        self.assertEqual(out, "ok")
        self.assertIsNone(exit5)
        self.assertEqual(self.ctx["consecutive_outliers"], 0)

    def test_threshold_disabled_is_ok(self):
        out, exit5 = engine._servo_outlier_decision(
            self.ctx,
            outlier_observable_ns=1e9,   # absurdly large
            track_outlier_ns=None,
            converging=False,
            gap_recovery=False,
        )
        self.assertEqual(out, "ok")
        self.assertIsNone(exit5)

    def test_converging_is_ok(self):
        # Outlier-class observable but scheduler is converging.
        out, exit5 = engine._servo_outlier_decision(
            self.ctx,
            outlier_observable_ns=500.0,
            track_outlier_ns=100.0,
            converging=True,
            gap_recovery=False,
        )
        self.assertEqual(out, "ok")
        self.assertIsNone(exit5)

    def test_converging_resets_accumulated_cascade(self):
        # Bravo's #109 review catch: the original inline code reset
        # consecutive_outliers in the catchall else, which covered the
        # converging branch.  Re-entering convergence means prior
        # cascade state is stale and should be thrown away — without
        # this reset, the cascade resumes from its accumulated value
        # when the scheduler exits convergence mode.
        self.ctx["consecutive_outliers"] = 25
        out, _ = engine._servo_outlier_decision(
            self.ctx,
            outlier_observable_ns=500.0,
            track_outlier_ns=100.0,
            converging=True,
            gap_recovery=False,
        )
        self.assertEqual(out, "ok")
        self.assertEqual(self.ctx["consecutive_outliers"], 0)
        # A subsequent non-converging outlier starts at 1, not 26.
        out2, exit5_2 = engine._servo_outlier_decision(
            self.ctx,
            outlier_observable_ns=500.0,
            track_outlier_ns=100.0,
            converging=False,
            gap_recovery=False,
        )
        self.assertEqual(out2, "outlier")
        self.assertFalse(exit5_2)
        self.assertEqual(self.ctx["consecutive_outliers"], 1)

    def test_in_range_resets_counter(self):
        self.ctx["consecutive_outliers"] = 5
        out, exit5 = engine._servo_outlier_decision(
            self.ctx,
            outlier_observable_ns=50.0,
            track_outlier_ns=100.0,
            converging=False,
            gap_recovery=False,
        )
        self.assertEqual(out, "ok")
        self.assertEqual(self.ctx["consecutive_outliers"], 0)

    def test_out_of_range_increments(self):
        out, exit5 = engine._servo_outlier_decision(
            self.ctx,
            outlier_observable_ns=200.0,
            track_outlier_ns=100.0,
            converging=False,
            gap_recovery=False,
        )
        self.assertEqual(out, "outlier")
        self.assertFalse(exit5)
        self.assertEqual(self.ctx["consecutive_outliers"], 1)

    def test_30_consecutive_outliers_triggers_exit5(self):
        self.ctx["consecutive_outliers"] = 29
        out, exit5 = engine._servo_outlier_decision(
            self.ctx,
            outlier_observable_ns=200.0,
            track_outlier_ns=100.0,
            converging=False,
            gap_recovery=False,
        )
        self.assertEqual(out, "outlier")
        self.assertTrue(exit5)
        self.assertEqual(self.ctx["consecutive_outliers"], 30)

    def test_clkpoc3_scenario_30_outliers_with_gap_recovery_first(self):
        # End-to-end replay of clkPoC3 exit-5 #1 with the fix in place:
        # first epoch is gap_recovery (pps_err=508 ns flat), should be
        # absorbed; the EKF then steers the DO back within range over
        # subsequent epochs.  Simulate by feeding the first epoch as
        # gap_recovery=True then the remaining 29 as ok-range.
        out, _ = engine._servo_outlier_decision(
            self.ctx,
            outlier_observable_ns=508.0,
            track_outlier_ns=100.0,
            converging=False,
            gap_recovery=True,
        )
        self.assertEqual(out, "ok")
        # In the original bug, all 30 epochs were outlier-class and
        # the cascade fired; with the fix, even if the next epoch is
        # still outlier-class (slow actuator response), the cascade
        # starts from 0, not 30.
        for _ in range(29):
            out, exit5 = engine._servo_outlier_decision(
                self.ctx,
                outlier_observable_ns=508.0,
                track_outlier_ns=100.0,
                converging=False,
                gap_recovery=False,
            )
            self.assertEqual(out, "outlier")
            self.assertFalse(exit5)   # counter ≤ 29 ⇒ no exit-5
        # Counter is now 29 — one more would trip, but in practice
        # the EKF + actuator pull pps_err back into range well before
        # this point.
        self.assertEqual(self.ctx["consecutive_outliers"], 29)


if __name__ == "__main__":
    unittest.main()

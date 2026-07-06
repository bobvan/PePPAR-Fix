#!/usr/bin/env python3
"""Unit tests for obs-defer wedge self-recovery.

The correlation gate defers observations that can't be matched to a PPS.  Before
2026-07-06 a *persistent* defer had no recovery path: ``run_steady_state`` warned
once and then ``continue``d forever, so a transient correlation wedge on otcBob1
became an ~11 h stall (the DO free-ran to a 146 us excursion) until a manual
restart.  The fix escalates: after ``--obs-stall-recovery-s`` wedged, flush PPS+obs
history and re-anchor the gate; after ``--obs-stall-max-recoveries`` such attempts,
exit-5 for a clean re-bootstrap.

``_obs_stall_recovery_due`` is the pure timing predicate that spaces the attempts;
these tests cover it and the escalation ladder that the loop builds on top of it.
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

_due = engine._obs_stall_recovery_due


class ObsStallRecoveryDueTest(unittest.TestCase):
    def test_disabled_never_fires(self):
        # recovery_s of 0 or None disables recovery entirely, even when wedged.
        self.assertFalse(_due(9999.0, 0.0, 9999.0, 0))
        self.assertFalse(_due(9999.0, 0.0, 9999.0, None))

    def test_not_stalled_long_enough(self):
        # Stalled 30 s but threshold is 60 s -> not due.
        self.assertFalse(_due(30.0, 0.0, 30.0, 60.0))

    def test_first_attempt_due_at_threshold(self):
        # Stalled 60 s, no prior attempt -> due.
        self.assertTrue(_due(60.0, 0.0, 60.0, 60.0))

    def test_spacing_suppresses_back_to_back_attempts(self):
        # An attempt just fired at now=60; only 5 s later we're still wedged,
        # but < recovery_s since the last attempt -> not due (don't flush every
        # loop iteration).
        self.assertFalse(_due(65.0, 60.0, 65.0, 60.0))

    def test_next_attempt_due_after_spacing_elapses(self):
        # Last attempt at 60; now 120 (60 s later), still wedged -> due again.
        self.assertTrue(_due(120.0, 60.0, 120.0, 60.0))


class EscalationLadderTest(unittest.TestCase):
    """Drive the REAL ObsStallEscalator the loop delegates to (not a re-impl):
    with recovery_s=60 and max=3, a continuous stall recovers at 60/120/180 s
    then exits-5 at 240 s (4th attempt exceeds the budget), and a consumed obs
    resets the ladder."""

    def test_recovers_three_times_then_exit5(self):
        esc = engine.ObsStallEscalator(recovery_s=60.0, max_recoveries=3)
        actions = []
        for now in range(400):
            action = esc.on_defer(float(now), float(now))  # continuous stall
            if action:
                actions.append((action, now))
            if action == "exit5":
                break
        self.assertEqual([a[0] for a in actions],
                         ["recover", "recover", "recover", "exit5"])
        self.assertEqual([a[1] for a in actions], [60, 120, 180, 240])

    def test_consumed_obs_resets_ladder(self):
        # Recovers at 60/120; at 130 an obs finally correlates -> on_consume()
        # resets attempts, and the host stays healthy -> never reaches exit-5.
        esc = engine.ObsStallEscalator(recovery_s=60.0, max_recoveries=3)
        exit5 = False
        for now in range(400):
            if now == 130:
                esc.on_consume()
            stall_s = float(now) if now < 130 else 0.0
            if esc.on_defer(stall_s, float(now)) == "exit5":
                exit5 = True
                break
        self.assertFalse(exit5)
        self.assertEqual(esc.attempts, 0)

    def test_disabled_escalator_never_acts(self):
        esc = engine.ObsStallEscalator(recovery_s=0, max_recoveries=3)
        self.assertIsNone(esc.on_defer(9999.0, 9999.0))
        self.assertEqual(esc.attempts, 0)


if __name__ == "__main__":
    unittest.main()

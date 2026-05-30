#!/usr/bin/env python3
"""Regression test for the holdover-path servo reset call.

``_enter_obs_holdover`` is the safe-state path taken on a sustained
observation outage (``obs_idle_timeout_s`` elapsed with no obs epochs;
see the call site in ``run_steady_state``).  It resets the EKF while
holding the actuator at the last commanded adjfine.

#107 (disciplineModeFsm binary layer) rewrote ``DOFreqEst.reset`` from
the positional ``reset(current_freq)`` to keyword-only
``reset(*, initial_freq=None, initial_dt_rx_ns=None)`` but did NOT
update this older call site, which still passed ``last_freq``
positionally — so every holdover entry raised

    TypeError: DOFreqEst.reset() takes 1 positional argument but 2 were given

crashing the engine on an observation outage instead of entering
holdover.  The fix passes ``initial_freq=last_freq``, restoring the
pre-#107 semantics (hold the commanded freq at the last adjfine while
the filter rebuilds from the wide bootstrap covariance).
"""

import queue
import sys
import threading
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

# Direct import via filesystem path — peppar_fix_engine isn't packaged.
import importlib.util  # noqa: E402

_ENGINE_PATH = REPO / "scripts" / "peppar_fix_engine.py"
spec = importlib.util.spec_from_file_location("peppar_fix_engine", _ENGINE_PATH)
engine = importlib.util.module_from_spec(spec)
spec.loader.exec_module(engine)

from peppar_fix.do_freq_est import DOFreqEst  # noqa: E402


def _holdover_args():
    """Minimal args namespace for the DisciplineScheduler that
    ``_enter_obs_holdover`` constructs."""
    return types.SimpleNamespace(
        discipline_interval=1,
        adaptive_interval=False,
        min_interval=1,
        max_interval=64,
        scheduler_converge_threshold_ns=5.0,
        scheduler_settle_window=10,
        scheduler_unconverge_factor=2.0,
        phase_error_budget_ns=100.0,
    )


def _holdover_ctx(servo, last_freq):
    return {
        "holdover": {
            "active": False,
            "reason": None,
            "entered": 0,
            "reasons": {},
        },
        "adjfine_ppb": last_freq,
        "pmc": None,  # _set_clock_class no-ops
        "pps_history_lock": threading.Lock(),
        "pps_history": [],
        "pps_queue": queue.Queue(),
        "servo": servo,
    }


class TestEnterObsHoldoverReset(unittest.TestCase):
    def test_holdover_does_not_raise_and_preserves_actuator(self):
        last_freq = -42.5
        servo = DOFreqEst()
        servo.freq = 999.0  # distinct from last_freq so we can see the reset
        ctx = _holdover_ctx(servo, last_freq)

        # Pre-fix this raised TypeError from the positional reset() call.
        engine._enter_obs_holdover(ctx, _holdover_args(), "no_obs_input",
                                   "no observation epochs for 30.0s")

        self.assertTrue(ctx["holdover"]["active"])
        self.assertEqual(ctx["holdover"]["reason"], "no_obs_input")
        self.assertEqual(ctx["holdover"]["entered"], 1)
        # Actuator held at the last commanded adjfine, not zeroed.
        self.assertEqual(servo.freq, last_freq)
        # EKF state rebuilt (wide bootstrap covariance, tcxo uninitialised).
        self.assertFalse(servo._tcxo_initialized)
        # Scheduler replaced.
        self.assertIsNotNone(ctx.get("scheduler"))

    def test_holdover_is_idempotent_when_already_active(self):
        servo = DOFreqEst()
        ctx = _holdover_ctx(servo, 0.0)
        ctx["holdover"]["active"] = True
        ctx["holdover"]["entered"] = 1

        engine._enter_obs_holdover(ctx, _holdover_args(), "no_obs_input", "x")

        # Early return: no second entry counted, no scheduler built.
        self.assertEqual(ctx["holdover"]["entered"], 1)
        self.assertIsNone(ctx.get("scheduler"))

    def test_reset_is_keyword_only(self):
        """Pin the contract that broke #107's older call site: the
        positional form must raise so a future regression is caught."""
        servo = DOFreqEst()
        with self.assertRaises(TypeError):
            servo.reset(0.0)  # positional — the pre-fix call shape


if __name__ == "__main__":
    unittest.main()

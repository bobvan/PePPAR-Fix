"""Tests for _persist_do_runtime — the shared periodic/shutdown runtime save.

periodicRuntimeSave: the persisted operating point (freq offset + DAC code) is
the warm-start seed.  Saving it WHILE LOCKED on a cadence means a restart —
clean OR crash/power-loss — seeds from the converged operating point and nails
the landing, not a stale bootstrap value (Bob 2026-06-18).  The helper is the
single writer shared by the in-loop periodic save and the clean-shutdown save.
"""
from __future__ import annotations

import os
import sys
import unittest
from unittest import mock

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix_engine import _persist_do_runtime  # noqa: E402

_FREQ = "peppar_fix.runtime_state_resolve.update_runtime_freq"
_CODE = "peppar_fix.runtime_state_resolve.update_runtime_dac_code"


class _Act:
    def __init__(self, code):
        self.current_code = code


class TestPersistDoRuntime(unittest.TestCase):

    def test_persists_freq_and_code(self):
        with mock.patch(_FREQ) as mf, mock.patch(_CODE) as mc:
            ok = _persist_do_runtime("ocxo-x", 12.5, _Act(36381))
        self.assertTrue(ok)
        mf.assert_called_once_with("ocxo-x", 12.5)
        mc.assert_called_once_with("ocxo-x", 36381)

    def test_no_do_uid_is_noop(self):
        with mock.patch(_FREQ) as mf, mock.patch(_CODE) as mc:
            ok = _persist_do_runtime(None, 12.5, _Act(36381))
        self.assertFalse(ok)
        mf.assert_not_called()
        mc.assert_not_called()

    def test_no_actuator_still_saves_freq(self):
        # PHC path / no DAC actuator: still persist the freq offset.
        with mock.patch(_FREQ) as mf, mock.patch(_CODE) as mc:
            ok = _persist_do_runtime("phc-x", -3.0, None)
        self.assertTrue(ok)
        mf.assert_called_once_with("phc-x", -3.0)
        mc.assert_not_called()

    def test_freq_save_failure_is_swallowed(self):
        # A save error must not crash the servo loop (returns False / no raise).
        with mock.patch(_FREQ, side_effect=OSError("disk full")), \
                mock.patch(_CODE):
            ok = _persist_do_runtime("ocxo-x", 1.0, _Act(100))
        self.assertFalse(ok)


if __name__ == "__main__":
    unittest.main()

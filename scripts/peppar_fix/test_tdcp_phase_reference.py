"""phaseReferenceRequired: TDCP (Arm 5) needs a real phase reference.

TDCP is a frequency-only observation (rx-clock freq vs SV-clock ensemble);
it cannot anchor DO phase, so it requires EXTINT or TICC.  The startup
guard `_check_tdcp_phase_reference` refuses `--servo-input tdcp` with both
phase observers disabled (the old invalid 'tdcp-only' arm).
"""
import os
import sys
import unittest
from argparse import Namespace

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix_engine import _check_tdcp_phase_reference  # noqa: E402


def _args(**kw):
    base = dict(servo_input='default', no_tdcp_arm=False,
                no_extint=False, no_ticc=False)
    base.update(kw)
    return Namespace(**base)


class TdcpPhaseReferenceTest(unittest.TestCase):

    def test_tdcp_without_any_phase_ref_raises(self):
        """The offending config: TDCP + no EXTINT + no TICC → SystemExit."""
        with self.assertRaises(SystemExit):
            _check_tdcp_phase_reference(
                _args(servo_input='tdcp', no_extint=True, no_ticc=True))

    def test_tdcp_with_ticc_ok(self):
        _check_tdcp_phase_reference(
            _args(servo_input='tdcp', no_extint=True, no_ticc=False))

    def test_tdcp_with_extint_ok(self):
        _check_tdcp_phase_reference(
            _args(servo_input='tdcp', no_extint=False, no_ticc=True))

    def test_tdcp_with_both_ok(self):
        _check_tdcp_phase_reference(_args(servo_input='tdcp'))

    def test_default_servo_input_not_checked(self):
        """Not TDCP → guard doesn't apply (no Arm 5 needing a phase anchor)."""
        _check_tdcp_phase_reference(
            _args(servo_input='default', no_extint=True, no_ticc=True))

    def test_tdcp_arm_disabled_ok(self):
        """--no-tdcp-arm means Arm 5 isn't active → no requirement."""
        _check_tdcp_phase_reference(
            _args(servo_input='tdcp', no_tdcp_arm=True,
                  no_extint=True, no_ticc=True))

    def test_error_message_is_actionable(self):
        try:
            _check_tdcp_phase_reference(
                _args(servo_input='tdcp', no_extint=True, no_ticc=True))
            self.fail("expected SystemExit")
        except SystemExit as e:
            msg = str(e)
            self.assertIn("phase reference", msg)
            self.assertIn("EXTINT or TICC", msg)


if __name__ == "__main__":
    unittest.main()

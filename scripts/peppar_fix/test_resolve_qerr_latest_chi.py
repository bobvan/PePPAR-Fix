"""Tests for _resolve_qerr_latest_chi (latestQErrChiDefaultOn).

latest-chi is the DEFAULT qErr path.  Unspecified → on (unless an alternative
router is chosen); --qerr-latest-chi forces on; --no-qerr-latest-chi → off
(legacy matched default).  An explicit --qerr-latest-chi + a router is a
conflict (SystemExit).
"""
import os
import sys
import unittest

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix_engine import _resolve_qerr_latest_chi as R  # noqa: E402


class ResolveQErrLatestChiTest(unittest.TestCase):

    def test_default_unspecified_is_on(self):
        """No flag, no router → default-on."""
        self.assertTrue(R(None, router_qvir=False, routed_qerr_arm=False))

    def test_explicit_on(self):
        self.assertTrue(R(True, router_qvir=False, routed_qerr_arm=False))

    def test_no_flag_escape_hatch_off(self):
        """--no-qerr-latest-chi → off (legacy matched default)."""
        self.assertFalse(R(False, router_qvir=False, routed_qerr_arm=False))

    def test_router_qvir_defers_default(self):
        """Unspecified + --router-qvir → latest-chi deferred (off)."""
        self.assertFalse(R(None, router_qvir=True, routed_qerr_arm=False))

    def test_routed_qerr_arm_defers_default(self):
        self.assertFalse(R(None, router_qvir=False, routed_qerr_arm=True))

    def test_explicit_on_plus_router_conflicts(self):
        """Explicit --qerr-latest-chi AND a router → SystemExit."""
        with self.assertRaises(SystemExit):
            R(True, router_qvir=True, routed_qerr_arm=False)
        with self.assertRaises(SystemExit):
            R(True, router_qvir=False, routed_qerr_arm=True)

    def test_no_flag_plus_router_no_conflict(self):
        """--no-qerr-latest-chi + a router is NOT a conflict (off + router runs)."""
        self.assertFalse(R(False, router_qvir=True, routed_qerr_arm=False))


if __name__ == "__main__":
    unittest.main()

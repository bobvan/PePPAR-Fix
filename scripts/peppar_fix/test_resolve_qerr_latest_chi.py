"""Tests for _resolve_qerr_latest_chi (latestQErrChiDefaultOn).

latest-chi is the DEFAULT and only routed qErr path (the matched v1/qVIR-v2
routers were retired in retireQerrMatchingCode I-064807).  Unspecified → on;
--qerr-latest-chi → on; --no-qerr-latest-chi → off (internal-qerr(x0) path).
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
        """No flag → default-on."""
        self.assertTrue(R(None))

    def test_explicit_on(self):
        """--qerr-latest-chi → on."""
        self.assertTrue(R(True))

    def test_no_flag_escape_hatch_off(self):
        """--no-qerr-latest-chi → off (internal-qerr(x0) path)."""
        self.assertFalse(R(False))


if __name__ == "__main__":
    unittest.main()

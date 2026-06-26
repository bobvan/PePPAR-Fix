"""Tests for AntPosEstThread._process_epoch — the per-epoch EKF step factored
out of _run_inner for pos_replay stage 2b.

The extraction converted the body's two EPOCH-LEVEL `continue`s (EKF dt-guard,
n_used<4) to `return` (skip-this-epoch) while the 8 `continue`s inside the
per-SV `for` loops must STAY `continue`.  A blanket conversion would silently
corrupt the filter, so these tests pin (1) the dt-guard return behaviorally and
(2) the for-loop-vs-epoch-level classification structurally."""
import ast
import inspect
import os
import sys
import unittest
from datetime import datetime, timezone, timedelta
from unittest.mock import Mock

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import peppar_fix_engine as eng  # noqa: E402

_UTC = timezone.utc


def _stub_thread(prev_t):
    t = eng.AntPosEstThread.__new__(eng.AntPosEstThread)
    t._filt = Mock()
    t._mw = Mock()
    t._nl = Mock()
    t._corrections = Mock()
    t._prev_t = prev_t
    return t


class TestDtGuardReturn(unittest.TestCase):
    """The EKF dt-guard `continue`→`return`: a non-monotone or too-large dt
    must skip the epoch (no predict/update) but advance _prev_t — exactly the
    old loop's `continue` semantics."""

    def test_non_monotone_dt_skips_epoch(self):
        prev = datetime(2026, 1, 1, 0, 0, 5, tzinfo=_UTC)
        t = _stub_thread(prev)
        gps = datetime(2026, 1, 1, 0, 0, 3, tzinfo=_UTC)   # earlier → dt < 0
        t._process_epoch(gps, [], None)
        t._filt.predict.assert_not_called()                # epoch skipped
        t._filt.update.assert_not_called()
        self.assertEqual(t._prev_t, gps)                   # _prev_t advanced

    def test_too_large_dt_skips_epoch(self):
        prev = datetime(2026, 1, 1, 0, 0, 0, tzinfo=_UTC)
        t = _stub_thread(prev)
        gps = prev + timedelta(seconds=200)                # dt > 120 → skip
        t._process_epoch(gps, [], None)
        t._filt.predict.assert_not_called()
        self.assertEqual(t._prev_t, gps)


class TestContinueClassificationIsStructural(unittest.TestCase):
    """Pin the extraction outcome: exactly 8 `continue`s survive (the nested
    per-SV for-loop ones) and the 2 epoch-level ones became `return`.  (That
    every survivor is inside a loop is already guaranteed — Python SyntaxErrors
    on a continue outside a loop, and the module imported.)  This count catches
    a future edit that flips a nested continue to return or vice-versa."""

    def test_continue_and_return_counts(self):
        import textwrap
        src = textwrap.dedent(
            inspect.getsource(eng.AntPosEstThread._process_epoch))
        fn = ast.parse(src).body[0]
        conts = sum(isinstance(n, ast.Continue) for n in ast.walk(fn))
        rets = sum(isinstance(n, ast.Return) for n in ast.walk(fn))
        self.assertEqual(conts, 8)        # the nested-for continues stay
        self.assertGreaterEqual(rets, 2)  # the 2 epoch-level ex-continues


if __name__ == "__main__":
    unittest.main()

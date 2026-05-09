"""Tests for is_pr_only_slip() — the PR-only-slip gate (I-155354 fix-I).

The audit (scripts/replay/slip_vs_wl_audit.py + docs/
wl-integer-vs-pride-comparison.md) showed 47.8 % of slip events on
day0506 MadHat were mw_jump alone (no carrier-phase corroboration),
all conf=LOW, all driven by PR-side noise spikes.  Each one
flushed the MW tracker, which then re-fixed the WL integer to a
wrong value 99.6 % of the time.

is_pr_only_slip(reasons) is the gate the engine uses to skip the
WL-tracker reset for these events.  This test pins:

  - Exactly {'mw_jump'} → True
  - mw_jump combined with carrier indicators → False
  - Any carrier indicator alone → False
  - arc_gap → False (legitimate gap, ambiguity discontinuous)
  - Empty reasons → False (defensive)
"""
from __future__ import annotations

import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.cycle_slip import is_pr_only_slip


class IsPrOnlySlipTest(unittest.TestCase):

    def test_mw_jump_alone_is_pr_only(self):
        """The whole point — mw_jump alone is the false-positive
        signature.  47.8 % of audit events match this."""
        self.assertTrue(is_pr_only_slip(["mw_jump"]))
        self.assertTrue(is_pr_only_slip({"mw_jump"}))
        self.assertTrue(is_pr_only_slip(("mw_jump",)))

    def test_mw_jump_plus_carrier_not_pr_only(self):
        """mw_jump WITH carrier corroboration is a real slip; flush."""
        self.assertFalse(is_pr_only_slip(["mw_jump", "gf_jump"]))
        self.assertFalse(is_pr_only_slip(
            ["gf_jump", "mw_jump", "ubx_locktime_drop"]))
        self.assertFalse(is_pr_only_slip(["arc_gap", "mw_jump"]))

    def test_carrier_alone_is_not_pr_only(self):
        """gf_jump / ubx_locktime_drop / arc_gap WITHOUT mw_jump are
        carrier-confirmed slips and must flush as before."""
        self.assertFalse(is_pr_only_slip(["gf_jump"]))
        self.assertFalse(is_pr_only_slip(["ubx_locktime_drop"]))
        self.assertFalse(is_pr_only_slip(["arc_gap"]))
        self.assertFalse(is_pr_only_slip(
            ["gf_jump", "ubx_locktime_drop"]))

    def test_empty_is_not_pr_only(self):
        """Defensive: empty reasons isn't a slip at all; gate must
        not treat it as filterable."""
        self.assertFalse(is_pr_only_slip([]))
        self.assertFalse(is_pr_only_slip(set()))

    def test_unknown_reason_not_pr_only(self):
        """Future detector reasons we don't know about — treat
        conservatively as 'not PR-only', let the existing flush
        path handle them."""
        self.assertFalse(is_pr_only_slip(["unknown_detector"]))
        self.assertFalse(is_pr_only_slip(
            ["mw_jump", "unknown_detector"]))


class SlipReasonCoverageFromAuditTest(unittest.TestCase):
    """Sanity: the eight reason combinations observed in the day0506
    MadHat audit each get the right verdict.

    From scripts/replay/slip_vs_wl_audit.py output:
        mw_jump                              1275  (PR-only → True)
        gf_jump,ubx_locktime_drop             595  (carrier → False)
        arc_gap                               276  (gap → False)
        arc_gap,mw_jump                       163  (gap → False)
        gf_jump                               143  (carrier → False)
        gf_jump,mw_jump                        87  (carrier → False)
        gf_jump,mw_jump,ubx_locktime_drop      66  (carrier → False)
        ubx_locktime_drop                      64  (carrier → False)
    """

    AUDIT_REASON_VERDICTS = [
        # (reason_set, expected_pr_only)
        ({"mw_jump"}, True),
        ({"gf_jump", "ubx_locktime_drop"}, False),
        ({"arc_gap"}, False),
        ({"arc_gap", "mw_jump"}, False),
        ({"gf_jump"}, False),
        ({"gf_jump", "mw_jump"}, False),
        ({"gf_jump", "mw_jump", "ubx_locktime_drop"}, False),
        ({"ubx_locktime_drop"}, False),
    ]

    def test_each_audit_reason_combination(self):
        for reasons, expected in self.AUDIT_REASON_VERDICTS:
            with self.subTest(reasons=reasons):
                self.assertEqual(
                    is_pr_only_slip(reasons), expected,
                    f"reasons={reasons} expected pr_only={expected}")


if __name__ == "__main__":
    unittest.main()

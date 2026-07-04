"""nav2FloorNeedsAnchor: --nav2-floor warns when the NAV2 soft-anchor is off.

The free-position floor (--nav2-floor) re-opens covariance for the NAV2 anchor
to re-centre; it runs inside _apply_nav2_anchor, so with --no-nav2-soft-anchor
it silently never fires.  `_check_nav2_floor_needs_anchor` warns (does not raise)
in exactly that case.  Charlie review, #261.
"""
import os
import sys
import unittest
from argparse import Namespace

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix_engine import _check_nav2_floor_needs_anchor  # noqa: E402


def _args(**kw):
    base = dict(nav2_floor=False, nav2_soft_anchor=True)
    base.update(kw)
    return Namespace(**base)


class Nav2FloorNeedsAnchorTest(unittest.TestCase):

    def test_floor_on_anchor_off_warns(self):
        """The inert config: --nav2-floor with the anchor disabled → warns."""
        with self.assertLogs("peppar-fix", level="WARNING") as cm:
            warned = _check_nav2_floor_needs_anchor(
                _args(nav2_floor=True, nav2_soft_anchor=False))
        self.assertTrue(warned)
        self.assertTrue(any("NO EFFECT" in m for m in cm.output))

    def test_floor_on_anchor_on_ok(self):
        """The normal config (anchor default on): no warning."""
        self.assertFalse(
            _check_nav2_floor_needs_anchor(
                _args(nav2_floor=True, nav2_soft_anchor=True)))

    def test_floor_off_never_warns(self):
        """Floor off: nothing to warn about regardless of the anchor."""
        self.assertFalse(
            _check_nav2_floor_needs_anchor(
                _args(nav2_floor=False, nav2_soft_anchor=False)))
        self.assertFalse(
            _check_nav2_floor_needs_anchor(
                _args(nav2_floor=False, nav2_soft_anchor=True)))

    def test_defaults_are_safe(self):
        """Engine defaults (floor off, anchor on): no warning."""
        self.assertFalse(_check_nav2_floor_needs_anchor(_args()))


if __name__ == "__main__":
    unittest.main()

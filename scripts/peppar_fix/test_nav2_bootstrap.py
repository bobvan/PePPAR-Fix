"""NAV2 single-point bootstrap for `--no-antposest` (I-071400 E1).

The time-only engine never blocks on a survey: when no trusted seed exists and
`--nav2-bootstrap` is set, it starts from a coarse NAV2 fix with an HONEST σ_r
(floored to absorb NAV2's 1.5-4 m bias) so σ_t is honest, not over-confident.
These cover the seed-application floor + the bootstrap store-fallthrough logic
(wait_for_nav2_seed is monkeypatched — its own polling is tested elsewhere).
"""
import os
import sys
import types
import unittest
from argparse import Namespace

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

import numpy as np
import peppar_fix_engine as eng  # noqa: E402


def _ape_sm():
    """Minimal AntPosEst state machine stand-in (only .transition is used)."""
    calls = []
    return types.SimpleNamespace(transition=lambda *a: calls.append(a),
                                 _calls=calls)


_ECEF = (3979160.0, -4257.0, 4968043.0)


class ApplyPositionSeedFloorTest(unittest.TestCase):
    """_apply_position_seed's honest-σ floor (explicit override + arg default)."""

    def test_explicit_floor_inflates_optimistic_hacc(self):
        # hAcc 3 m (precision-only) but floor 10 m (absorbs NAV2 bias) → σ=10 m
        _, sigma, source = eng._apply_position_seed(
            (np.asarray(_ECEF), 3.0, 12), "NAV2-bootstrap",
            Namespace(nav2_seed_sigma_floor_m=0.0), _ape_sm(), sigma_floor_m=10.0)
        self.assertEqual(sigma, 10.0)
        self.assertIn("NAV2-bootstrap", source)

    def test_floor_only_inflates_never_shrinks(self):
        # hAcc 15 m already looser than the 10 m floor → σ stays 15 m
        _, sigma, _ = eng._apply_position_seed(
            (np.asarray(_ECEF), 15.0, 8), "NAV2-bootstrap",
            Namespace(nav2_seed_sigma_floor_m=0.0), _ape_sm(), sigma_floor_m=10.0)
        self.assertEqual(sigma, 15.0)

    def test_no_override_uses_arg_default_zero_is_raw_hacc(self):
        # sigma_floor_m=None + arg floor 0 → σ = raw hAcc (legacy behavior)
        _, sigma, _ = eng._apply_position_seed(
            (np.asarray(_ECEF), 2.5, 9), "NAV2",
            Namespace(nav2_seed_sigma_floor_m=0.0), _ape_sm())
        self.assertEqual(sigma, 2.5)


class Nav2BootstrapSeedTest(unittest.TestCase):
    """_nav2_bootstrap_seed store fallthrough + relaxed-bar / floor plumbing."""

    def setUp(self):
        self._orig = eng.wait_for_nav2_seed
        self.args = Namespace(nav2_bootstrap_hacc_max_m=30.0,
                              nav2_bootstrap_sigma_floor_m=10.0,
                              nav2_seed_sigma_floor_m=0.0)

    def tearDown(self):
        eng.wait_for_nav2_seed = self._orig

    def _patch(self, per_store):
        """per_store: dict store-id → (ecef,hacc,nsv) or None. Records the
        hacc_max_m each call was made with."""
        seen = []
        def fake(store, stop_event, timeout_s=60.0, hacc_max_m=5.0,
                 source_label="NAV2", **kw):
            seen.append((source_label, hacc_max_m))
            return per_store.get(id(store))
        eng.wait_for_nav2_seed = fake
        return seen

    def test_nav2_store_fix_applies_floored_sigma_and_relaxed_bar(self):
        nav2, navpvt = object(), object()
        seen = self._patch({id(nav2): (np.asarray(_ECEF), 3.0, 11)})
        ecef, sigma, source = eng._nav2_bootstrap_seed(
            self.args, nav2, navpvt, stop_event=object(), ape_sm=_ape_sm())
        self.assertIsNotNone(ecef)
        self.assertEqual(sigma, 10.0)                 # floored to honest σ_r
        self.assertIn("NAV2-bootstrap", source)
        self.assertEqual(seen[0], ("NAV2-bootstrap", 30.0))  # relaxed bar used

    def test_falls_through_nav2_to_navpvt(self):
        nav2, navpvt = object(), object()
        self._patch({id(navpvt): (np.asarray(_ECEF), 4.0, 7)})  # only NAV-PVT fixes
        ecef, sigma, source = eng._nav2_bootstrap_seed(
            self.args, nav2, navpvt, stop_event=object(), ape_sm=_ape_sm())
        self.assertIsNotNone(ecef)
        self.assertIn("NAV-PVT-bootstrap", source)

    def test_no_fix_returns_none_triple(self):
        nav2, navpvt = object(), object()
        self._patch({})                               # neither store fixes
        self.assertEqual(
            eng._nav2_bootstrap_seed(self.args, nav2, navpvt,
                                     stop_event=object(), ape_sm=_ape_sm()),
            (None, None, None))

    def test_none_stores_skipped_without_calling_wait(self):
        seen = self._patch({})
        result = eng._nav2_bootstrap_seed(
            self.args, None, None, stop_event=object(), ape_sm=_ape_sm())
        self.assertEqual(result, (None, None, None))
        self.assertEqual(seen, [])                     # no wait on None stores


if __name__ == "__main__":
    unittest.main()

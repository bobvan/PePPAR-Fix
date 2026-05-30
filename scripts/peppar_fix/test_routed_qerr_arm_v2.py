"""Tests for the v2 (qVIR-routed) routedQErrArm selector.

Design: docs/routed-qerr-router-v2-qvir.md (PR #98).  Drops v1's
'internal' candidate (H=[-1,0,-1,0] — the rx-TCXO leakage path); two
candidates only ('ext' and 'raw'), both H=[0,0,-1,0], routed by the
engine-side ``ticc_ext_correlated`` boolean (qVIR > threshold).
"""
from __future__ import annotations

import os
import sys
import unittest

import numpy as np

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.do_freq_est import DOFreqEst


def _f(v2=True, **kw):
    defaults = dict(
        sigma_ticc_ns=0.060,
        sigma_do_phase_ns=0.05,
        sigma_do_freq_ppb=0.01,
        sigma_tcxo_phase_ns=2.0,
        sigma_tcxo_freq_ppb=0.1,
        initial_freq=0.0,
        initial_dt_rx_ns=0.0,
        routed_qerr_v2=v2,
    )
    defaults.update(kw)
    f = DOFreqEst(**defaults)
    f._need_phc_seed = False
    return f


class V2SelectorTest(unittest.TestCase):
    """The pure selector logic (independent of the EKF predict/update)."""

    def test_correlated_selects_ext(self):
        f = _f()
        x_pred = np.array([2.5, 0.0, -10.0, 0.0])
        z, h, H, R, name = f._route_ticc_arm_v2(
            x_pred, ticc_diff_ns=7.5, ticc_qerr_ns=2.5,
            ticc_ext_correlated=True, R_base=0.0036)
        self.assertEqual(name, 'ext')
        # z = ticc_diff + ext_qerr = 7.5 + 2.5 = 10.0
        self.assertAlmostEqual(z, 10.0, places=6)
        # h = -x[2] = 10.0
        self.assertAlmostEqual(h, 10.0, places=6)
        # H = [0,0,-1,0] — no x[0] coupling
        self.assertTrue(np.array_equal(H, [[0.0, 0.0, -1.0, 0.0]]))
        # R = R_base (no sawtooth penalty)
        self.assertAlmostEqual(R, 0.0036, places=6)

    def test_uncorrelated_selects_raw(self):
        f = _f()
        x_pred = np.array([2.5, 0.0, -10.0, 0.0])
        z, h, H, R, name = f._route_ticc_arm_v2(
            x_pred, ticc_diff_ns=7.5, ticc_qerr_ns=2.5,
            ticc_ext_correlated=False, R_base=0.0036)
        self.assertEqual(name, 'raw')
        # z = ticc_diff (no ext correction)
        self.assertAlmostEqual(z, 7.5, places=6)
        # Same H as ext (no x[0] coupling either!).
        self.assertTrue(np.array_equal(H, [[0.0, 0.0, -1.0, 0.0]]))
        # R = R_base + tick²/12 (sawtooth variance for uncorrected qErr)
        self.assertAlmostEqual(R, 0.0036 + (8.0 ** 2) / 12.0, places=6)

    def test_no_qerr_routes_to_raw_regardless(self):
        # Bootstrap regime: even if ticc_ext_correlated says True,
        # there's no qErr to use → raw.
        f = _f()
        x_pred = np.array([2.5, 0.0, -10.0, 0.0])
        _, _, _, _, name = f._route_ticc_arm_v2(
            x_pred, ticc_diff_ns=7.5, ticc_qerr_ns=None,
            ticc_ext_correlated=True, R_base=0.0036)
        self.assertEqual(name, 'raw')

    def test_no_internal_candidate(self):
        # The v2 design's defining feature: no H=[-1,0,-1,0] candidate.
        # Sweep all combinations and confirm only ext|raw return.
        f = _f()
        x_pred = np.array([2.5, 0.0, -10.0, 0.0])
        names_seen = set()
        for ext_corr in (True, False):
            for qerr in (None, 2.5):
                _, _, _, _, name = f._route_ticc_arm_v2(
                    x_pred, ticc_diff_ns=7.5, ticc_qerr_ns=qerr,
                    ticc_ext_correlated=ext_corr, R_base=0.0036)
                names_seen.add(name)
        self.assertEqual(names_seen, {'ext', 'raw'})
        self.assertNotIn('int', names_seen)


class V2UpdateIntegrationTest(unittest.TestCase):
    """The selector wired through DOFreqEst.update."""

    def test_v2_correlated_uses_ext_route(self):
        f = _f()
        f.x = np.array([2.5, 0.0, -10.0, 0.0])
        f.update(dt=1.0, ticc_diff_ns=7.5, ticc_qerr_ns=2.5,
                 ticc_sigma_ns=0.060, ticc_ext_correlated=True)
        self.assertEqual(f.last_ticc_route, 'ext')
        self.assertEqual(f.n_route_ext, 1)
        self.assertEqual(f.n_route_int, 0)
        self.assertEqual(f.n_route_raw, 0)

    def test_v2_uncorrelated_uses_raw_route(self):
        f = _f()
        f.x = np.array([2.5, 0.0, -10.0, 0.0])
        f.update(dt=1.0, ticc_diff_ns=7.5, ticc_qerr_ns=2.5,
                 ticc_sigma_ns=0.060, ticc_ext_correlated=False)
        self.assertEqual(f.last_ticc_route, 'raw')
        self.assertEqual(f.n_route_raw, 1)
        self.assertEqual(f.n_route_int, 0)

    def test_v2_default_no_qvir_signal_routes_raw(self):
        # update() called without ticc_ext_correlated kwarg
        # ⇒ default False ⇒ raw.  Bootstrap window not yet full uses
        # this path naturally.
        f = _f()
        f.x = np.array([2.5, 0.0, -10.0, 0.0])
        f.update(dt=1.0, ticc_diff_ns=7.5, ticc_qerr_ns=2.5,
                 ticc_sigma_ns=0.060)
        self.assertEqual(f.last_ticc_route, 'raw')

    def test_v2_does_not_increment_int_counter(self):
        # The 'internal' counter must stay at zero under v2 — the
        # candidate doesn't exist.  Operator looking at the
        # [ROUTED_QERR] log sees ext/raw only.
        f = _f()
        for ext_corr in (True, False, True, False):
            f.x = np.array([2.5, 0.0, -10.0, 0.0])
            f.update(dt=1.0, ticc_diff_ns=7.5, ticc_qerr_ns=2.5,
                     ticc_sigma_ns=0.060, ticc_ext_correlated=ext_corr)
        self.assertEqual(f.n_route_int, 0)
        self.assertEqual(f.n_route_ext + f.n_route_raw, 4)


class V2VsV1MutualExclusionTest(unittest.TestCase):
    """v1 and v2 paths are mutually exclusive at construction.  The
    engine is responsible for refusing both flags at the same time;
    DOFreqEst itself takes both bools but v2 wins (the design's
    successor takes precedence).
    """

    def test_v2_wins_when_both_set(self):
        f = _f(v2=True)
        f._routed_qerr = True  # simulate both flags somehow set
        self.assertTrue(f._routed_qerr_v2)
        f.x = np.array([2.5, 0.0, -10.0, 0.0])
        # ext_correlated=False under v2 → raw; v1 would have routed to
        # ext or int based on chi².  Behavior must match v2.
        f.update(dt=1.0, ticc_diff_ns=7.5, ticc_qerr_ns=2.5,
                 ticc_sigma_ns=0.060, ticc_ext_correlated=False)
        self.assertEqual(f.last_ticc_route, 'raw')


class V2DefaultOffTest(unittest.TestCase):
    """When neither v1 nor v2 is enabled, behavior is byte-identical
    to the legacy single-candidate internal path (#79's default-off
    contract carried forward)."""

    def test_default_off_legacy_int(self):
        f = _f(v2=False)
        self.assertFalse(f._routed_qerr_v2)
        f.x = np.array([2.5, 0.0, -10.0, 0.0])
        f.update(dt=1.0, ticc_diff_ns=7.5, ticc_qerr_ns=2.5,
                 ticc_sigma_ns=0.060, ticc_ext_correlated=True)
        self.assertEqual(f.last_ticc_route, 'int')
        self.assertEqual(f.n_route_int, 1)


if __name__ == "__main__":
    unittest.main()

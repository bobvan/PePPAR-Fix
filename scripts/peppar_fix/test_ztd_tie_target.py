"""Tests for ZTD-tie target unification (I-132038).

Both PPPFilter.apply_ztd_tie and FixedPosFilter.apply_ztd_tie now
accept a target_m parameter (default 0.0 to preserve prior behaviour).
A shared TropoState (cross-thread, lock-protected) carries the
METAR-derived target so both filters' ties pull toward the same
atmosphere.
"""
from __future__ import annotations

import math
import os
import sys
import threading
import unittest

import numpy as np

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from solve_ppp import FixedPosFilter, IDX_ZTD, PPPFilter
from peppar_fix.tropo_state import TropoState, TropoTarget


_BASE_ECEF = np.array([157470.222, -4756189.544, 4232767.952])


# --- PPPFilter target_m -------------------------------------------------- #


class PppFilterApplyZtdTieTargetTest(unittest.TestCase):

    def _fresh(self):
        f = PPPFilter()
        f.initialize(_BASE_ECEF, clock_m=0.0)
        return f

    def test_default_target_zero_unchanged_behavior(self):
        # Pre-unification: tie pulled state toward 0.  Default
        # target_m=0.0 must preserve that behaviour.
        f = self._fresh()
        f.x[IDX_ZTD] = 0.10                # 100 mm residual
        f.apply_ztd_tie(sigma_m=0.05)      # tight prior toward 0
        # State pulled most of the way toward 0 (K ≈ P/(P+σ²) where P is large at init).
        self.assertLess(abs(f.x[IDX_ZTD]), 0.10)
        self.assertGreater(f.x[IDX_ZTD], 0.0,
                           "shouldn't overshoot toward negative")

    def test_target_pulls_state_toward_target(self):
        # Tie with target_m=+34mm should pull state toward +34mm.
        f = self._fresh()
        f.x[IDX_ZTD] = 0.0
        f.apply_ztd_tie(sigma_m=0.05, target_m=0.034)
        self.assertGreater(f.x[IDX_ZTD], 0.0)
        # Direction matches sign of target.
        self.assertLess(f.x[IDX_ZTD], 0.034 + 1e-6)

    def test_negative_target_pulls_negative(self):
        # Low-pressure-system case — target negative.
        f = self._fresh()
        f.x[IDX_ZTD] = 0.0
        f.apply_ztd_tie(sigma_m=0.05, target_m=-0.080)
        self.assertLess(f.x[IDX_ZTD], 0.0)

    def test_zero_offset_no_change_when_state_at_target(self):
        f = self._fresh()
        f.x[IDX_ZTD] = 0.034
        f.apply_ztd_tie(sigma_m=0.05, target_m=0.034)
        self.assertAlmostEqual(f.x[IDX_ZTD], 0.034)

    def test_no_op_when_sigma_invalid(self):
        f = self._fresh()
        f.x[IDX_ZTD] = 0.10
        f.apply_ztd_tie(sigma_m=0.0, target_m=0.034)
        self.assertAlmostEqual(f.x[IDX_ZTD], 0.10)
        f.apply_ztd_tie(sigma_m=None, target_m=0.034)
        self.assertAlmostEqual(f.x[IDX_ZTD], 0.10)


# --- FixedPosFilter apply_ztd_tie --------------------------------------- #


class FixedPosFilterApplyZtdTieTest(unittest.TestCase):

    def _fresh(self):
        return FixedPosFilter(_BASE_ECEF)

    def test_default_target_pulls_toward_zero(self):
        f = self._fresh()
        f.x[FixedPosFilter.IDX_ZTD] = 0.10
        f.apply_ztd_tie(sigma_m=0.05)
        self.assertLess(abs(f.x[FixedPosFilter.IDX_ZTD]), 0.10)

    def test_target_pulls_state_toward_target(self):
        f = self._fresh()
        f.x[FixedPosFilter.IDX_ZTD] = 0.0
        f.apply_ztd_tie(sigma_m=0.05, target_m=0.034)
        self.assertGreater(f.x[FixedPosFilter.IDX_ZTD], 0.0)

    def test_negative_target_pulls_negative(self):
        f = self._fresh()
        f.x[FixedPosFilter.IDX_ZTD] = 0.0
        f.apply_ztd_tie(sigma_m=0.05, target_m=-0.080)
        self.assertLess(f.x[FixedPosFilter.IDX_ZTD], 0.0)

    def test_no_op_when_sigma_invalid(self):
        f = self._fresh()
        f.x[FixedPosFilter.IDX_ZTD] = 0.10
        f.apply_ztd_tie(sigma_m=0.0, target_m=0.034)
        self.assertAlmostEqual(f.x[FixedPosFilter.IDX_ZTD], 0.10)


# --- Cross-filter consistency ------------------------------------------- #


class CrossFilterTargetConsistencyTest(unittest.TestCase):
    """The whole point of I-132038: when both filters tie to the SAME
    target with the SAME σ, their IDX_ZTD residuals should converge to
    the SAME steady-state value modulo per-filter observation noise.

    These tests verify the primitive — `apply_ztd_tie` produces the
    same Kalman-update-direction on both classes given the same target.
    Cross-filter convergence at the engine level (with both filters
    seeing observations) is a property of the coupled system; this
    suite tests the tie itself, not the full filter dynamics.
    """

    def test_same_target_yields_same_pull_sign(self):
        ppp = PPPFilter()
        ppp.initialize(_BASE_ECEF, clock_m=0.0)
        fix = FixedPosFilter(_BASE_ECEF)
        ppp.x[IDX_ZTD] = 0.10
        fix.x[FixedPosFilter.IDX_ZTD] = 0.10
        target = -0.020  # METAR says actual residual is −20 mm
        ppp.apply_ztd_tie(sigma_m=0.05, target_m=target)
        fix.apply_ztd_tie(sigma_m=0.05, target_m=target)
        # Both filters started at 0.10 and got pulled toward -0.020
        # → both should now be < 0.10 (moved in the same direction).
        self.assertLess(ppp.x[IDX_ZTD], 0.10)
        self.assertLess(fix.x[FixedPosFilter.IDX_ZTD], 0.10)
        # Both should still be above target (one pull, partial K).
        self.assertGreater(ppp.x[IDX_ZTD], target)
        self.assertGreater(fix.x[FixedPosFilter.IDX_ZTD], target)


# --- TropoState shared store -------------------------------------------- #


class TropoStateTest(unittest.TestCase):

    def test_cold_start_default(self):
        s = TropoState()
        t = s.get()
        self.assertEqual(t.target_m, 0.0)
        self.assertEqual(t.sigma_m, 0.05)
        self.assertFalse(t.valid)

    def test_set_and_get_roundtrip(self):
        s = TropoState()
        s.set(target_m=0.034, sigma_m=0.050,
              valid=True, metar_age_s=180.0, epoch=42)
        t = s.get()
        self.assertAlmostEqual(t.target_m, 0.034)
        self.assertAlmostEqual(t.sigma_m, 0.050)
        self.assertTrue(t.valid)
        self.assertEqual(t.last_metar_age_s, 180.0)
        self.assertEqual(t.last_refresh_epoch, 42)

    def test_set_preserves_sigma_when_none(self):
        s = TropoState()
        s.set(target_m=0.020, sigma_m=0.030, valid=True)
        s.set(target_m=0.040, sigma_m=None, valid=True)
        t = s.get()
        self.assertAlmostEqual(t.target_m, 0.040)
        self.assertAlmostEqual(t.sigma_m, 0.030,
                               msg="sigma_m=None should preserve prior σ")

    def test_get_returns_snapshot_not_ref(self):
        s = TropoState()
        s.set(target_m=0.020, sigma_m=0.050, valid=True)
        snap = s.get()
        # Mutating the snapshot must not bleed back to the shared state.
        snap.target_m = 999.0
        self.assertNotEqual(s.get().target_m, 999.0)

    def test_threadsafe_concurrent_access(self):
        """Many threads reading + one writer; reads must produce a
        consistent (target_m, sigma_m, valid) tuple even mid-write."""
        s = TropoState()
        s.set(target_m=0.034, sigma_m=0.050, valid=True)

        stop = threading.Event()
        bad_reads: list[tuple] = []

        def reader():
            while not stop.is_set():
                t = s.get()
                # All values from a single get() must come from the
                # same set() call.  We seed alternating valid pairs:
                # (0.034, 0.050) and (0.080, 0.030).  Any cross-pair
                # mix is a torn read.
                pair = (round(t.target_m, 4), round(t.sigma_m, 4))
                if pair not in {(0.034, 0.050), (0.080, 0.030)}:
                    bad_reads.append(pair)

        def writer():
            for i in range(500):
                if i % 2 == 0:
                    s.set(target_m=0.034, sigma_m=0.050, valid=True)
                else:
                    s.set(target_m=0.080, sigma_m=0.030, valid=True)
            stop.set()

        threads = [threading.Thread(target=reader) for _ in range(4)]
        threads.append(threading.Thread(target=writer))
        for th in threads:
            th.start()
        for th in threads:
            th.join()
        self.assertEqual(bad_reads, [],
                         f"torn reads under concurrent access: {bad_reads[:5]}")


if __name__ == "__main__":
    unittest.main()

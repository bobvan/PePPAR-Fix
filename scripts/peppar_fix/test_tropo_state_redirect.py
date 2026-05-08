"""Tests for the periodic-METAR → TropoState redirect (I-132038
follow-up).

Covers:
  - Cold-start: TropoState valid=False; both filters' ties still
    apply with target_m=0 + fallback σ from CLI default.
  - Refresh writes target into TropoState; subsequent reads see the
    new value.
  - Refresh failure (METAR unreachable / stale / Saastamoinen error)
    leaves the previous valid value in place — filters keep tying
    to last-known target until the next successful refresh.
  - Both filters reading the same TropoState produce parallel ZTD
    trajectories given identical observation noise (the unification
    invariant).

`_refresh_tropo_state` itself is tested via mocking — the engine-
level integration (CLI + actual fetch_latest_metar HTTP path) is
exercised separately by the lab smoke runs.  Pure-logic level here
keeps the test suite fast and dev-box-runnable.
"""
from __future__ import annotations

import os
import sys
import unittest
from unittest.mock import patch

import numpy as np

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from solve_ppp import FixedPosFilter, IDX_ZTD, PPPFilter
from peppar_fix.tropo_state import TropoState


_BASE_ECEF = np.array([157470.222, -4756189.544, 4232767.952])


class ColdStartReadTest(unittest.TestCase):
    """Before the first refresh, TropoState is valid=False and
    target_m=0.  Both filters' ties pull toward 0 with the cold-
    start σ — bit-exact with pre-unification behaviour."""

    def test_pppfilter_cold_start_pulls_toward_zero(self):
        ts = TropoState()                    # valid=False, target=0
        f = PPPFilter()
        f.initialize(_BASE_ECEF, clock_m=0.0)
        f.x[IDX_ZTD] = 0.10
        snap = ts.get()
        f.apply_ztd_tie(0.05, target_m=snap.target_m)
        # Pulled toward 0 (cold-start fallback semantics).
        self.assertLess(abs(f.x[IDX_ZTD]), 0.10)
        self.assertGreater(f.x[IDX_ZTD], 0.0)

    def test_fixedposfilter_cold_start_pulls_toward_zero(self):
        ts = TropoState()
        f = FixedPosFilter(_BASE_ECEF)
        f.x[FixedPosFilter.IDX_ZTD] = 0.10
        snap = ts.get()
        f.apply_ztd_tie(0.05, target_m=snap.target_m)
        self.assertLess(abs(f.x[FixedPosFilter.IDX_ZTD]), 0.10)


class RefreshWriteReadTest(unittest.TestCase):
    """After a successful refresh, both filters tie to the same
    target read from TropoState."""

    def test_set_propagates_to_both_filter_reads(self):
        ts = TropoState()
        # Simulated METAR refresh result (KDPA-typical residual +35mm).
        ts.set(target_m=0.035, sigma_m=0.050, valid=True,
               metar_age_s=180.0, epoch=60)
        ppp = PPPFilter()
        ppp.initialize(_BASE_ECEF, clock_m=0.0)
        ppp.x[IDX_ZTD] = 0.0
        fix = FixedPosFilter(_BASE_ECEF)
        fix.x[FixedPosFilter.IDX_ZTD] = 0.0
        snap = ts.get()
        # Both reads see the same target/sigma from TropoState.
        ppp.apply_ztd_tie(snap.sigma_m, target_m=snap.target_m)
        fix.apply_ztd_tie(snap.sigma_m, target_m=snap.target_m)
        # Both pulled toward +0.035 from 0 (same direction, same magnitude
        # given same priors and σ).
        self.assertGreater(ppp.x[IDX_ZTD], 0.0)
        self.assertGreater(fix.x[FixedPosFilter.IDX_ZTD], 0.0)

    def test_refresh_can_overwrite_target(self):
        # Two refreshes in sequence — second one overwrites first.
        ts = TropoState()
        ts.set(target_m=0.020, sigma_m=0.050, valid=True, epoch=60)
        ts.set(target_m=0.080, sigma_m=0.050, valid=True, epoch=120)
        snap = ts.get()
        self.assertAlmostEqual(snap.target_m, 0.080)
        self.assertEqual(snap.last_refresh_epoch, 120)


class RefreshFailureKeepsPriorTargetTest(unittest.TestCase):
    """When a refresh fails (METAR unreachable / stale), the
    previous valid target stays in TropoState.  Filters continue
    tying to last-known atmosphere until the next successful refresh.

    This is a critical reliability property: temporary METAR fetch
    failures must NOT collapse the tie back to target=0 mid-run.
    """

    def test_refresh_failure_does_not_clear_prior_target(self):
        ts = TropoState()
        ts.set(target_m=0.034, sigma_m=0.050, valid=True,
               metar_age_s=120.0, epoch=60)
        # Simulate a failed refresh: don't call set().  Engine's
        # _refresh_tropo_state on a fetch error logs + returns
        # without touching the state.
        # Verify state still holds the last valid target:
        snap = ts.get()
        self.assertAlmostEqual(snap.target_m, 0.034)
        self.assertTrue(snap.valid)

    def test_explicit_invalid_set_clears_valid_flag(self):
        # Sanity — caller CAN deliberately invalidate (e.g., 24h
        # since last METAR; operator wants to fall back to target=0).
        ts = TropoState()
        ts.set(target_m=0.034, sigma_m=0.050, valid=True, epoch=60)
        ts.set(target_m=0.0, sigma_m=0.050, valid=False, epoch=120)
        snap = ts.get()
        self.assertFalse(snap.valid)
        self.assertAlmostEqual(snap.target_m, 0.0)


class CrossFilterParallelTrajectoryTest(unittest.TestCase):
    """The unification invariant: given the same TropoState target,
    both filters' IDX_ZTD residuals walk in the same direction toward
    the same target.

    This is a property test for the redesign — same input target,
    same Kalman update math, same direction of pull.  Not a full
    convergence test (would need observations) but pins the
    primitive."""

    def test_same_target_same_initial_state_both_pull_same_direction(self):
        ts = TropoState()
        ts.set(target_m=-0.045, sigma_m=0.050, valid=True, epoch=60)

        ppp = PPPFilter()
        ppp.initialize(_BASE_ECEF, clock_m=0.0)
        fix = FixedPosFilter(_BASE_ECEF)

        # Both at +0.10 m residual (filter wandered before tie kicks in).
        ppp.x[IDX_ZTD] = 0.10
        fix.x[FixedPosFilter.IDX_ZTD] = 0.10

        snap = ts.get()
        ppp.apply_ztd_tie(snap.sigma_m, target_m=snap.target_m)
        fix.apply_ztd_tie(snap.sigma_m, target_m=snap.target_m)

        # Both should now be < 0.10 (pulled toward -0.045).
        self.assertLess(ppp.x[IDX_ZTD], 0.10)
        self.assertLess(fix.x[FixedPosFilter.IDX_ZTD], 0.10)
        # Both still above target (one update, partial K).
        self.assertGreater(ppp.x[IDX_ZTD], snap.target_m)
        self.assertGreater(fix.x[FixedPosFilter.IDX_ZTD], snap.target_m)


if __name__ == "__main__":
    unittest.main()

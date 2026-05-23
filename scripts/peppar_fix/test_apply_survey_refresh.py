"""Tests for the engine's _apply_survey_refresh helper.

The helper sits at the boundary between survey_state_watcher events
(coming off the watcher thread's queue) and the engine's
FixedPosFilter / known_ecef / sigma_pin_m state.  It's the only
new logic the integration adds inside run_steady_state, so it's
the one piece worth a focused unit test.
"""
from __future__ import annotations

import os
import sys
import unittest
from unittest import mock

import numpy as np

# scripts/ is on PYTHONPATH when the engine runs; tests need the same.
_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.survey_state_watcher import SurveyRefresh


class _FakeFilt:
    """Stand-in for FixedPosFilter exposing only .pos (what
    _apply_survey_refresh touches)."""

    def __init__(self, pos):
        self.pos = np.asarray(pos, dtype=float)


# Generic anchor (not a real lab antenna).
ANCHOR = np.array([0.0, -4862789.5, 4100000.0])


def _refresh(ecef=None, sigma=0.012, mount_sn=0, mtime=1000.0):
    if ecef is None:
        ecef = tuple(ANCHOR)
    return SurveyRefresh(
        new_ecef=tuple(float(c) for c in ecef),
        new_sigma_m=sigma,
        new_mount_sn=mount_sn,
        file_mtime=mtime,
    )


def _import_helper():
    """Lazy-import the engine module's _apply_survey_refresh.

    peppar_fix_engine.py is heavyweight at import time (pulls in
    serial, pyubx2, lots of side effects), so this is scoped to the
    test entry rather than module-level.
    """
    import peppar_fix_engine as eng
    return eng._apply_survey_refresh


class ApplySurveyRefreshTest(unittest.TestCase):

    def test_step_returns_exit_code_5(self):
        apply = _import_helper()
        filt = _FakeFilt(ANCHOR)
        event = ("step", _refresh(), 1.5)
        new_ecef, new_sigma, exit_code = apply(
            event, ANCHOR.copy(), 0.05, filt)
        self.assertEqual(exit_code, 5)
        # State unchanged on STEP — restart will re-seed from disk.
        np.testing.assert_array_equal(new_ecef, ANCHOR)
        self.assertEqual(new_sigma, 0.05)
        # filt.pos NOT mutated on STEP.
        np.testing.assert_array_equal(filt.pos, ANCHOR)

    def test_slew_small_delta_applies_blend(self):
        apply = _import_helper()
        filt = _FakeFilt(ANCHOR)
        # Refresh 5 cm east; σ_pin=10cm, σ_src=1cm → blend mostly
        # adopts source (small step toward it).
        event = ("slew",
                 _refresh(ecef=ANCHOR + np.array([0.05, 0, 0]),
                          sigma=0.01),
                 0.05)
        old_ecef = ANCHOR.copy()
        new_ecef, new_sigma, exit_code = apply(
            event, old_ecef, 0.10, filt)
        self.assertIsNone(exit_code)
        # ARP moved toward source.
        self.assertGreater(new_ecef[0], old_ecef[0])
        self.assertLess(new_ecef[0], old_ecef[0] + 0.05 + 1e-6)
        # σ_pin tightened (Bayesian Kalman update reduces it).
        self.assertLess(new_sigma, 0.10)
        # filt.pos mutated to match new ARP.
        np.testing.assert_array_almost_equal(filt.pos, new_ecef)

    def test_slew_extreme_delta_rejected_by_outlier_gate(self):
        apply = _import_helper()
        filt = _FakeFilt(ANCHOR)
        # 10 m delta with σ_pin=σ_src=1 cm — gate = 3 × (0.01 + 0.01)
        # = 0.06 m.  10 m >> 0.06 m → REJECTED.  bayesian_arp_blend
        # returns action='rejected_outlier'; our helper leaves state
        # unchanged.
        event = ("slew",
                 _refresh(ecef=ANCHOR + np.array([10.0, 0, 0]),
                          sigma=0.01),
                 10.0)
        old_ecef = ANCHOR.copy()
        old_sigma = 0.01
        new_ecef, new_sigma, exit_code = apply(
            event, old_ecef, old_sigma, filt)
        self.assertIsNone(exit_code)
        np.testing.assert_array_equal(new_ecef, old_ecef)
        self.assertEqual(new_sigma, old_sigma)
        np.testing.assert_array_equal(filt.pos, ANCHOR)

    def test_unknown_event_kind_warns_and_no_op(self):
        apply = _import_helper()
        filt = _FakeFilt(ANCHOR)
        event = ("nonsense", _refresh(), 0.0)
        old_ecef = ANCHOR.copy()
        new_ecef, new_sigma, exit_code = apply(
            event, old_ecef, 0.05, filt)
        self.assertIsNone(exit_code)
        np.testing.assert_array_equal(new_ecef, old_ecef)
        self.assertEqual(new_sigma, 0.05)


if __name__ == "__main__":
    unittest.main()

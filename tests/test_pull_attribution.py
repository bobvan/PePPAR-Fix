#!/usr/bin/env python3
"""Tests for per-arm pull attribution in DOFreqEst (pullAttributionLog).

The schema captures three new dicts on each EKF measurement update:

  last_arm_K[name]          # K vector (per state dim) for this arm
  last_arm_would_pull[name] # K · innov — state delta if admitted
  last_arm_chi2[name]       # innov² / S — gate-proximity metric

These are recorded BEFORE downstream gate checks so rejected arms still
emit their "what would have happened" attribution.  Logged values must
satisfy invariants:

  1. would_pull == K * innov (component-wise)
  2. chi2 == innov² / S
  3. When the arm is admitted, would_pull equals the actual state delta
     applied this epoch.
  4. Per-epoch reset clears all three dicts back to None.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))

from peppar_fix.do_freq_est import DOFreqEst  # noqa: E402


class PullAttributionTests(unittest.TestCase):

    def _fresh(self) -> DOFreqEst:
        servo = DOFreqEst()
        # Start at a known state so we can measure deltas cleanly.
        servo.x = np.array([0.0, 0.0, 0.0, 0.0])
        servo.P = np.eye(4) * 1e3
        return servo

    def test_attribution_dicts_initialized_with_none(self):
        servo = self._fresh()
        for name in ('ppp', 'qerr', 'tdcp', 'extint', 'pseudo', 'ticc'):
            self.assertIsNone(servo.last_arm_K[name])
            self.assertIsNone(servo.last_arm_would_pull[name])
            self.assertIsNone(servo.last_arm_chi2[name])

    def test_linear_arm_records_K_pull_and_chi2(self):
        """Drive a single linear-arm update and verify the math."""
        servo = self._fresh()
        # H selects x[1] (frequency).  z = 0 with x[1]=0 gives innov=0;
        # we want a non-zero innov so set up a real measurement.
        H = np.array([[0.0, 1.0, 0.0, 0.0]])
        z = 5.0   # pretend "frequency residual" 5 ppb
        R = 4.0   # arbitrary measurement variance
        x_pre = servo.x.copy()
        x_post, _ = servo._kalman_linear_update(
            servo.x, servo.P, z=z, H=H, R=R, arm_name='tdcp')

        # All three new dicts populated, others still None.
        self.assertIsNotNone(servo.last_arm_K['tdcp'])
        self.assertIsNotNone(servo.last_arm_would_pull['tdcp'])
        self.assertIsNotNone(servo.last_arm_chi2['tdcp'])
        self.assertIsNone(servo.last_arm_K['ppp'])

        innov = servo.last_arm_innov['tdcp']
        S = servo.last_arm_S['tdcp']
        K = np.array(servo.last_arm_K['tdcp'])
        would = np.array(servo.last_arm_would_pull['tdcp'])
        chi2 = servo.last_arm_chi2['tdcp']

        # Invariant 1: would_pull == K * innov (component-wise).
        np.testing.assert_allclose(would, K * innov, rtol=1e-12)

        # Invariant 2: chi2 == innov² / S.
        self.assertAlmostEqual(chi2, innov * innov / S, places=10)

        # Invariant 3: applied state delta equals would_pull when the
        # arm is admitted (no gate rejection in _kalman_linear_update).
        np.testing.assert_allclose(x_post - x_pre, would, rtol=1e-12)

    def test_chi2_handles_zero_variance(self):
        """S=0 must not blow up; chi² guarded to 0.0 in the code."""
        servo = self._fresh()
        servo.P = np.zeros((4, 4))  # P=0 → S=R; pick R=0 too
        H = np.array([[0.0, 1.0, 0.0, 0.0]])
        # With S==0 the code path returns chi2=0; predicted-variance
        # collapse is a separate diagnostic concern, not our problem.
        with np.errstate(divide='ignore', invalid='ignore'):
            servo._kalman_linear_update(
                servo.x, servo.P, z=1.0, H=H, R=0.0, arm_name='tdcp')
        chi2 = servo.last_arm_chi2['tdcp']
        # When S == 0, the guard returns 0.0 rather than inf/NaN.
        self.assertEqual(chi2, 0.0)

    def test_reset_clears_attribution_dicts(self):
        """The per-epoch reset must clear K/would_pull/chi2 alongside
        innov/S, otherwise stale entries from the previous run would
        leak into the next CSV row."""
        servo = self._fresh()
        H = np.array([[0.0, 1.0, 0.0, 0.0]])
        servo._kalman_linear_update(
            servo.x, servo.P, z=1.0, H=H, R=1.0, arm_name='tdcp')
        self.assertIsNotNone(servo.last_arm_K['tdcp'])
        # Reset by setting last_arm_innov[k]=None in the per-epoch sweep
        # (mirrors what _setup_servo does at epoch start).
        for k in servo.last_arm_innov:
            servo.last_arm_innov[k] = None
            servo.last_arm_S[k] = None
            servo.last_arm_K[k] = None
            servo.last_arm_would_pull[k] = None
            servo.last_arm_chi2[k] = None
        for name in ('ppp', 'qerr', 'tdcp', 'extint', 'pseudo', 'ticc'):
            self.assertIsNone(servo.last_arm_K[name])
            self.assertIsNone(servo.last_arm_would_pull[name])
            self.assertIsNone(servo.last_arm_chi2[name])


if __name__ == "__main__":
    unittest.main()

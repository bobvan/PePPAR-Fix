"""Tests for FixedPosFilter q_ztd_step constructor knob.

fixedPosFilterNoiseLocate finding: ZTD walk leaks ~50% into clock
estimate via the m_wet coupling in PR rows.  Tightening Q_ZTD slows
ZTD drift, reducing the coupling effect on dt_rx.  The legacy
default (8.33e-4)² ≈ 6.94e-7 m²/s matches the IGS 5 cm/hr standard
for tropo-mission filters; pinned-position clock-mission usage can
run tighter.
"""
from __future__ import annotations

import os
import sys
import unittest

import numpy as np

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from solve_ppp import FixedPosFilter

_BASE_ECEF = np.array([157470.222, -4756189.544, 4232767.952])


class QZtdKwargsTest(unittest.TestCase):

    def test_default_matches_legacy(self):
        """No kwarg → legacy (8.33e-4)² ≈ 6.94e-7."""
        f = FixedPosFilter(_BASE_ECEF)
        self.assertAlmostEqual(f.q_ztd_step, (8.33e-4) ** 2, places=12)

    def test_kwarg_override(self):
        f = FixedPosFilter(_BASE_ECEF, q_ztd_step=1e-9)
        self.assertEqual(f.q_ztd_step, 1e-9)

    def test_predict_uses_configured_q_ztd(self):
        """Predict() injects q_ztd_step × dt into P[ZTD,ZTD]."""
        f = FixedPosFilter(_BASE_ECEF, q_ztd_step=2e-7)
        # Set P to zero so predict's injection is the only thing in P.
        f.P = np.zeros((f.N_STATES, f.N_STATES))
        f.predict(1.0)
        self.assertAlmostEqual(f.P[f.IDX_ZTD, f.IDX_ZTD], 2e-7, places=12)

    def test_predict_scales_with_dt(self):
        f = FixedPosFilter(_BASE_ECEF, q_ztd_step=1e-7)
        f.P = np.zeros((f.N_STATES, f.N_STATES))
        f.predict(5.0)
        self.assertAlmostEqual(f.P[f.IDX_ZTD, f.IDX_ZTD], 5e-7, places=12)

    def test_none_kwarg_uses_default(self):
        f = FixedPosFilter(_BASE_ECEF, q_ztd_step=None)
        self.assertAlmostEqual(f.q_ztd_step, (8.33e-4) ** 2, places=12)

    def test_zero_q_ztd_allowed(self):
        """Q_ZTD=0 = ZTD treated as constant.  Legitimate extreme for
        pinned-position experiments."""
        f = FixedPosFilter(_BASE_ECEF, q_ztd_step=0.0)
        self.assertEqual(f.q_ztd_step, 0.0)
        f.P = np.zeros((f.N_STATES, f.N_STATES))
        f.predict(10.0)
        self.assertEqual(f.P[f.IDX_ZTD, f.IDX_ZTD], 0.0)


if __name__ == "__main__":
    unittest.main()

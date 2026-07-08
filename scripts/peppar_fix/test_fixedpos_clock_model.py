"""FixedPosFilter clock_model — wno cascade-collapse for single-osc GNSSDO.

For a GNSSDO+ the DO servo consumes dt_rx = FixedPosFilter.x[IDX_CLK].  The
default 2-state (phase+rate) 'random_walk' clock SMOOTHS that estimate — a
second filter in series with the servo (the cascade Bravo's #301 sim
reproduced).  'wno' collapses it: each predict resets the clock-PHASE prior
(decouple + wide), so dt_rx carries no inter-epoch memory/lag.  The clock-RATE
state is untouched.  See docs/gnssdo-servo-loop-bandwidth.md.
"""
import os
import sys
import unittest

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for _p in (_ROOT, os.path.join(_ROOT, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from solve_ppp import FixedPosFilter, WNO_CLK_PRIOR_VAR

_POS = [3979160.0, -4256.0, 4968042.0]   # ufoLondon-ish ECEF


class FixedPosClockModelTest(unittest.TestCase):

    def test_default_is_random_walk(self):
        self.assertEqual(FixedPosFilter(_POS).clock_model, "random_walk")

    def test_calibrated_white_falls_back_to_random_walk(self):
        # Not meaningful for the 2-state FixedPos clock; must not crash.
        self.assertEqual(
            FixedPosFilter(_POS, clock_model="calibrated_white").clock_model,
            "random_walk")

    def test_wno_accepted(self):
        self.assertEqual(
            FixedPosFilter(_POS, clock_model="wno").clock_model, "wno")

    def test_wno_resets_clock_phase_prior_each_predict(self):
        f = FixedPosFilter(_POS, clock_model="wno")
        i = f.IDX_CLK
        # Seed a tight (converged-looking) clock phase variance + cross-cov.
        f.P[i, i] = 1e-4
        f.P[i, f.IDX_CLK_RATE] = 0.5
        f.P[f.IDX_CLK_RATE, i] = 0.5
        f.predict(1.0)
        # Phase prior re-opened wide, cross-covariance zeroed → clock phase is
        # an independent unknown this epoch (no accumulated smoothing memory).
        self.assertAlmostEqual(f.P[i, i], WNO_CLK_PRIOR_VAR, delta=1.0)
        self.assertEqual(f.P[i, f.IDX_CLK_RATE], 0.0)
        self.assertEqual(f.P[f.IDX_CLK_RATE, i], 0.0)

    def test_random_walk_preserves_phase_rate_coupling(self):
        # The legacy 2-state path PROPAGATES phase↔rate coupling (that
        # integration is the smoothing memory / the cascade); it does NOT
        # reset to the wide wno prior.  Contrast test_wno_resets_* which
        # zeroes the cross-covariance and re-opens the prior.
        f = FixedPosFilter(_POS, clock_model="random_walk")
        i, r = f.IDX_CLK, f.IDX_CLK_RATE
        f.P[i, i] = 1e-4
        f.P[r, r] = 1e-4
        f.P[i, r] = f.P[r, i] = 0.5
        f.predict(1.0)
        self.assertNotEqual(f.P[i, r], 0.0)                  # coupling kept
        self.assertNotAlmostEqual(f.P[i, i], WNO_CLK_PRIOR_VAR, delta=1e6)

    def test_wno_leaves_clock_rate_dynamics(self):
        # The RATE state keeps its own process noise (frequency tracking
        # unaffected) — only the PHASE is whitened.
        f = FixedPosFilter(_POS, clock_model="wno")
        r = f.IDX_CLK_RATE
        p0 = f.P[r, r]
        f.predict(1.0)
        self.assertGreater(f.P[r, r], p0)   # rate variance grew by its Q (not reset)
        self.assertLess(f.P[r, r], WNO_CLK_PRIOR_VAR)


if __name__ == "__main__":
    unittest.main()

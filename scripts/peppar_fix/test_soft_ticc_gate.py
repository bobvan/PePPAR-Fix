"""Tests for the soft TICC chi² gate (softGateMidTau, I-092034).

The legacy Arm-4 gate is BINARY: innov²/S over _CHI2_GATE_THRESHOLD (10σ)
skips the update entirely → open-loop coast → snap-on-reopen → mid-τ TDEV
bulge.  The soft gate (--soft-ticc-gate, default OFF) instead inflates R by
max(1, χ²/K²) and ALWAYS admits, so an outlier is down-weighted but still
pulls — no blackout, no snap.

These pin: (a) below-knee → identical to the hard path; (b) over-knee →
hard rejects (no pull) while soft admits a bounded, nonzero, down-weighted
pull; (c) far outlier → heavy down-weight but nonzero; (d) default-off is
byte-identical to the hard gate; plus the inflation-factor relationship,
the sole-observer interaction, and innov-monitor feeding.
"""

import numpy as np
import unittest

from peppar_fix.do_freq_est import DOFreqEst, _CHI2_GATE_THRESHOLD


def _baseline(**kw):
    defaults = dict(
        sigma_ticc_ns=0.060,
        sigma_do_phase_ns=0.92,
        sigma_do_freq_ppb=0.01,
        sigma_tcxo_phase_ns=2.0,
        sigma_tcxo_freq_ppb=0.1,
        initial_freq=0.0,
    )
    defaults.update(kw)
    return DOFreqEst(**defaults)


def _converged(soft):
    """A filter at a tight, converged P (so a modest innov over-knees)."""
    f = _baseline(soft_ticc_gate=soft)
    f._need_phc_seed = False
    f.x = np.array([0.0, 0.0, 0.0, 0.0])
    f.P = np.diag([1.0, 0.01, 1.0, 0.01])
    return f


# A TICC innovation comfortably over the 10σ knee at the converged P above,
# with EXTINT present (so the hard gate genuinely rejects rather than taking
# the sole-observer override).
_OVER_KNEE_NS = 100.0
_EXTINT = dict(extint_phase_ns=0.0, extint_sigma_ns=10.0)


class TestSoftTiccGate(unittest.TestCase):

    # ---- (d) default-off byte-identical to the hard gate --------------------

    def test_default_off_rejects_overknee_like_hard_gate(self):
        """soft_ticc_gate=False: an over-knee innov with a redundant EXTINT
        observer is REJECTED (x[2] unchanged) — the legacy hard behavior."""
        f = _converged(soft=False)
        x2_before = f.x[2]
        f.update(dt=1.0, ticc_diff_ns=_OVER_KNEE_NS, ticc_sigma_ns=0.060,
                 **_EXTINT)
        self.assertAlmostEqual(f.x[2], x2_before, delta=1.0)
        self.assertEqual(f.last_ticc_R_inflation, 1.0)

    def test_default_off_matches_unflagged_state_exactly(self):
        """soft=False produces the same post-update state as a filter built
        with no soft kwarg at all — proves the default path is untouched."""
        f_off = _converged(soft=False)
        f_ref = _baseline()  # no soft_ticc_gate kwarg → default
        f_ref._need_phc_seed = False
        f_ref.x = np.array([0.0, 0.0, 0.0, 0.0])
        f_ref.P = np.diag([1.0, 0.01, 1.0, 0.01])
        for f in (f_off, f_ref):
            f.update(dt=1.0, ticc_diff_ns=_OVER_KNEE_NS, ticc_sigma_ns=0.060,
                     **_EXTINT)
        np.testing.assert_array_almost_equal(f_off.x, f_ref.x, decimal=12)
        np.testing.assert_array_almost_equal(f_off.P, f_ref.P, decimal=12)

    # ---- (a) below-knee: soft == hard ---------------------------------------

    def test_below_knee_identical_to_hard(self):
        """An in-budget innovation (χ² < knee) gives an IDENTICAL state
        update under soft and hard gates, and inflation stays 1.0."""
        small = 8.0  # well under the ~20 ns knee at this P
        f_hard = _converged(soft=False)
        f_soft = _converged(soft=True)
        for f in (f_hard, f_soft):
            f.update(dt=1.0, ticc_diff_ns=small, ticc_sigma_ns=0.060, **_EXTINT)
        # Confirm we were actually below the knee (else the test is vacuous).
        self.assertLess(f_soft.last_arm_chi2['ticc'], _CHI2_GATE_THRESHOLD)
        self.assertEqual(f_soft.last_ticc_R_inflation, 1.0)
        np.testing.assert_array_almost_equal(f_soft.x, f_hard.x, decimal=12)
        np.testing.assert_array_almost_equal(f_soft.P, f_hard.P, decimal=12)

    # ---- (b) over-knee: hard rejects, soft admits bounded/nonzero -----------

    def test_overknee_hard_rejects_soft_admits(self):
        """At an over-knee innov with EXTINT redundancy: hard gate → no x[2]
        pull; soft gate → nonzero x[2] pull, but SMALLER than the full-weight
        would_pull (down-weighted, not the hard-gate snap)."""
        f_hard = _converged(soft=False)
        f_soft = _converged(soft=True)
        x2_0 = 0.0
        f_hard.update(dt=1.0, ticc_diff_ns=_OVER_KNEE_NS, ticc_sigma_ns=0.060,
                      **_EXTINT)
        f_soft.update(dt=1.0, ticc_diff_ns=_OVER_KNEE_NS, ticc_sigma_ns=0.060,
                      **_EXTINT)
        # Hard: rejected → x[2] essentially unchanged.
        self.assertAlmostEqual(f_hard.x[2], x2_0, delta=1.0)
        # Soft: admitted → x[2] moved.
        soft_pull = abs(f_soft.x[2] - x2_0)
        self.assertGreater(soft_pull, 0.0)
        # Down-weighted: smaller than the full-weight pull the arm would have
        # applied (recorded in would_pull regardless of the gate).
        full_pull = abs(f_soft.last_arm_would_pull['ticc'][2])
        self.assertLess(soft_pull, full_pull)
        self.assertGreater(f_soft.last_ticc_R_inflation, 1.0)

    def test_inflation_grows_with_innovation(self):
        """Bigger outlier → bigger R-inflation → smaller applied pull
        (monotone down-weighting, the anti-snap mechanism)."""
        f_small = _converged(soft=True)
        f_big = _converged(soft=True)
        f_small.update(dt=1.0, ticc_diff_ns=50.0, ticc_sigma_ns=0.060, **_EXTINT)
        f_big.update(dt=1.0, ticc_diff_ns=500.0, ticc_sigma_ns=0.060, **_EXTINT)
        self.assertGreater(f_big.last_ticc_R_inflation,
                           f_small.last_ticc_R_inflation)

    # ---- (c) far outlier: heavy down-weight but nonzero ---------------------

    def test_far_outlier_heavy_downweight_nonzero(self):
        """A 1e6 ns outlier: inflation is enormous, the applied pull is a
        tiny fraction of full-weight — but NOT zero (never a full reject)."""
        f = _converged(soft=True)
        f.update(dt=1.0, ticc_diff_ns=1e6, ticc_sigma_ns=0.060, **_EXTINT)
        pull = abs(f.x[2] - 0.0)
        self.assertGreater(pull, 0.0)
        full_pull = abs(f.last_arm_would_pull['ticc'][2])
        self.assertLess(pull, 0.01 * full_pull)  # heavily down-weighted
        self.assertGreater(f.last_ticc_R_inflation, 100.0)

    # ---- inflation-factor relationship --------------------------------------

    def test_inflation_factor_matches_chi2_ratio(self):
        """last_ticc_R_inflation == max(1, χ²/K²) for the recorded χ²."""
        f = _converged(soft=True)
        f.update(dt=1.0, ticc_diff_ns=_OVER_KNEE_NS, ticc_sigma_ns=0.060,
                 **_EXTINT)
        chi2 = f.last_arm_chi2['ticc']
        expected = max(1.0, chi2 / _CHI2_GATE_THRESHOLD)
        self.assertAlmostEqual(f.last_ticc_R_inflation, expected, places=6)

    # ---- sole-observer interaction ------------------------------------------

    def test_sole_observer_soft_admits_downweighted(self):
        """With NO EXTINT (sole DO-phase observer), the soft gate still
        admits an over-knee innov (x[2] moves) — never starves x[2]."""
        f = _converged(soft=True)
        f.update(dt=1.0, ticc_diff_ns=_OVER_KNEE_NS, ticc_sigma_ns=0.060)
        self.assertNotAlmostEqual(f.x[2], 0.0, places=2)
        self.assertGreater(f.last_ticc_R_inflation, 1.0)

    # ---- innov-monitor feeding ----------------------------------------------

    def test_innov_monitor_fed_on_soft_admit(self):
        """The soft gate admits the outlier, so the innovation monitor IS
        fed (contrast: the hard gate rejects → monitor not fed)."""
        f = _converged(soft=True)
        n_before = len(f.innov_monitor._innov)
        f.update(dt=1.0, ticc_diff_ns=_OVER_KNEE_NS, ticc_sigma_ns=0.060,
                 **_EXTINT)
        self.assertEqual(len(f.innov_monitor._innov), n_before + 1)


if __name__ == "__main__":
    unittest.main()

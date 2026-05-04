"""Tests for PPPFilter physics-tight priors (I-024532-charlie).

Per I-133648-main consensus (Q1-Q4 + Bravo's design):
  - Initial position σ from caller (NAV2.hAcc), default 10 m
    (was 100 m — wide enough to absorb systematic δ_PB into position).
  - Initial residual ZTD σ = 0.2 m (was 0.5 m — wide enough to absorb
    SSR phase-bias residuals into ZTD, producing the doom-loop
    cascade observed on TimeHat 2026-04-29 overnight, 36/73 trips).
  - Q[IDX_ZTD] random-walk PSD = 1 cm² / min so the filter can
    follow real weather without resisting legitimate ZTD movement.

These tests pin the physics-tight values to lock down the scope of
the I-024532-charlie change.  Loosening any of them in the future
should require revisiting the integrity-trip cascade evidence.
"""

from __future__ import annotations

import os
import sys
import unittest

import numpy as np

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from solve_ppp import FixedPosFilter, PPPFilter, IDX_ZTD


class InitialPriorTest(unittest.TestCase):
    """PPPFilter.initialize prior values."""

    def test_default_position_sigma_is_10m(self):
        """The default seeds at 10 m σ — appropriate for an SPP-grade
        seed.  Not as wide as the old 100 m default."""
        f = PPPFilter()
        f.initialize([1e6, 0.0, 0.0], clock_m=0.0)
        for axis in range(3):
            self.assertAlmostEqual(f.P[axis, axis], 10.0**2)

    def test_default_ztd_sigma_is_200mm(self):
        """The default residual-ZTD σ is 200 mm — physical envelope of
        residual wet delay at sea level."""
        f = PPPFilter()
        f.initialize([1e6, 0.0, 0.0], clock_m=0.0)
        self.assertAlmostEqual(f.P[IDX_ZTD, IDX_ZTD], 0.2**2)

    def test_pos_sigma_m_override(self):
        """Caller overrides pos_sigma_m (engine passes NAV2.hAcc)."""
        f = PPPFilter()
        f.initialize([1e6, 0.0, 0.0], clock_m=0.0, pos_sigma_m=1.5)
        for axis in range(3):
            self.assertAlmostEqual(f.P[axis, axis], 1.5**2)

    def test_ztd_sigma_m_override(self):
        """Caller can supply a tighter or looser ztd_sigma_m."""
        f = PPPFilter()
        f.initialize([1e6, 0.0, 0.0], clock_m=0.0, ztd_sigma_m=0.1)
        self.assertAlmostEqual(f.P[IDX_ZTD, IDX_ZTD], 0.1**2)
        self.assertAlmostEqual(f._ztd_sigma_m, 0.1)


class InitZtdSeedTest(unittest.TestCase):
    """init_ztd_m parameter on PPPFilter.initialize + FixedPosFilter ctor.

    Per I-024942 (METAR-seeded ZTD prior).  The seed lands the
    filter close to atmospheric truth at epoch 1, avoiding the
    multi-meter ZTD cold-start transient observed under
    --pin-position on 2026-05-04.
    """

    def test_pppfilter_default_init_ztd_zero(self):
        f = PPPFilter()
        f.initialize([1e6, 0.0, 0.0], clock_m=0.0)
        self.assertAlmostEqual(f.x[IDX_ZTD], 0.0)

    def test_pppfilter_init_ztd_seed_lands(self):
        f = PPPFilter()
        f.initialize([1e6, 0.0, 0.0], clock_m=0.0, init_ztd_m=0.034)
        self.assertAlmostEqual(f.x[IDX_ZTD], 0.034)

    def test_pppfilter_init_ztd_signed(self):
        # Negative residuals (low-pressure systems) must be accepted.
        f = PPPFilter()
        f.initialize([1e6, 0.0, 0.0], clock_m=0.0, init_ztd_m=-0.080)
        self.assertAlmostEqual(f.x[IDX_ZTD], -0.080)

    def test_pppfilter_init_ztd_orthogonal_to_sigma(self):
        # init_ztd_m sets the mean; ztd_sigma_m sets the σ.
        f = PPPFilter()
        f.initialize([1e6, 0.0, 0.0], clock_m=0.0,
                     init_ztd_m=0.10, ztd_sigma_m=0.05)
        self.assertAlmostEqual(f.x[IDX_ZTD], 0.10)
        self.assertAlmostEqual(f.P[IDX_ZTD, IDX_ZTD], 0.05**2)

    def test_fixedposfilter_default_init_ztd_zero(self):
        f = FixedPosFilter([1e6, 0.0, 0.0])
        self.assertAlmostEqual(f.x[FixedPosFilter.IDX_ZTD], 0.0)
        # Default σ preserved at 0.5 m.
        self.assertAlmostEqual(
            f.P[FixedPosFilter.IDX_ZTD, FixedPosFilter.IDX_ZTD],
            0.5**2)

    def test_fixedposfilter_init_ztd_seed_lands(self):
        f = FixedPosFilter([1e6, 0.0, 0.0],
                           init_ztd_m=0.034, init_ztd_sigma_m=0.05)
        self.assertAlmostEqual(f.x[FixedPosFilter.IDX_ZTD], 0.034)
        self.assertAlmostEqual(
            f.P[FixedPosFilter.IDX_ZTD, FixedPosFilter.IDX_ZTD],
            0.05**2)

    def test_fixedposfilter_init_ztd_negative(self):
        f = FixedPosFilter([1e6, 0.0, 0.0],
                           init_ztd_m=-0.080, init_ztd_sigma_m=0.05)
        self.assertAlmostEqual(f.x[FixedPosFilter.IDX_ZTD], -0.080)


class ZtdProcessNoiseTest(unittest.TestCase):
    """Q[IDX_ZTD] in the default RW (non-PWC) regime."""

    def test_random_walk_psd_is_1cm2_per_minute(self):
        """The random-walk PSD admits ~1 cm² per minute → ~7.7 cm/hour
        σ growth.  Physical bound for real-weather ZTD movement
        without standing-baseline-loose."""
        f = PPPFilter()
        f.initialize([1e6, 0.0, 0.0], clock_m=0.0)
        # Snapshot P[ZTD] before predict; predict for 60 s; check
        # that ΔP[ZTD] = (1.29e-3)² · 60 = 1e-4 m².
        p_before = float(f.P[IDX_ZTD, IDX_ZTD])
        # Pin position so the predict's adaptive q_pos branch picks
        # the converged regime — keeps ΔP[ZTD] isolated from cross
        # terms.
        f.P[0, 0] = 1e-4
        f.P[1, 1] = 1e-4
        f.P[2, 2] = 1e-4
        f.predict(dt=60.0)
        p_after = float(f.P[IDX_ZTD, IDX_ZTD])
        delta = p_after - p_before
        # Expect ΔP ≈ (1.29e-3)² · 60 = 1.0e-4 m² (within 5%).
        self.assertAlmostEqual(delta, 1.0e-4, delta=5e-6)

    def test_pwc_segment_boundary_uses_instance_ztd_sigma(self):
        """PWC-N segment boundary inflates P[IDX_ZTD] to
        self._ztd_sigma_m² (not the old hardcoded 0.5²).  Verifies
        the per-instance value is propagated."""
        f = PPPFilter()
        f.initialize([1e6, 0.0, 0.0], clock_m=0.0, ztd_sigma_m=0.15)
        f.ZTD_PWC_WINDOW_S = 60.0
        # Drive past one segment boundary.
        f.P[0, 0] = 1e-4
        f.P[1, 1] = 1e-4
        f.P[2, 2] = 1e-4
        # Walk the variance up first so we can see the inflate clamp it.
        f.P[IDX_ZTD, IDX_ZTD] = 9.0  # 3-m σ — much larger than 0.15²
        f.predict(dt=60.0)
        # Boundary fired: P should be reset to ztd_sigma_m².
        self.assertAlmostEqual(f.P[IDX_ZTD, IDX_ZTD], 0.15**2)


class Nav2AnchorTest(unittest.TestCase):
    """PPPFilter.apply_nav2_anchor — per-epoch NAV2 covariance update.

    Validates the variance-weighted Kalman measurement update that
    consumes NAV2's SPP fix as a 3D position observation.  See
    docs/filter-stiffness-redesign.md and dayplan I-200128-main.

    Misnomer note (I-051234, 2026-05-02): despite the legacy method
    name 'anchor', the math is a standard EKF update — not a
    truth-pull anchor.  Once filter P shrinks below NAV2's R, the
    Kalman gain → 0 and this update contributes ~nothing per epoch.
    A static offset between filter basin and NAV2 SPP fix is
    invisible.  See apply_nav2_anchor docstring + I-051234 sub-B
    for the planned true truth-pull anchor (separate method).
    """

    # UFO1-class ECEF (lab roof, 41.84°N).  Magnitudes are realistic
    # for the Earth-radius scale; details don't matter for the math.
    _BASE_ECEF = np.array([157468.4, -4756190.5, 4232770.7])

    def _fresh_filter(self, pos_sigma_m=10.0):
        f = PPPFilter()
        f.initialize(self._BASE_ECEF, 0.0, pos_sigma_m=pos_sigma_m)
        return f

    def test_pulls_position_toward_nav2(self):
        """Soft-anchor with hAcc<<σ_pos should pull state toward NAV2."""
        f = self._fresh_filter(pos_sigma_m=10.0)
        # Offset state ECEF so we have something to pull.  Pure
        # vertical offset of +5 m to make the test transparent.
        up = self._BASE_ECEF / np.linalg.norm(self._BASE_ECEF)
        f.x[:3] = self._BASE_ECEF + up * 5.0
        # NAV2 reports the truth at hAcc=1m.  Anchor should pull
        # state most of the way back: K ≈ σ_pos²/(σ_pos² + hAcc²) = 0.99.
        f.apply_nav2_anchor(self._BASE_ECEF, h_acc_m=1.0, v_acc_m=2.0)
        # State should now be close to NAV2 truth.
        diff = f.x[:3] - self._BASE_ECEF
        self.assertLess(np.linalg.norm(diff), 0.5,
                        f"position not pulled toward NAV2: {diff}")

    def test_zero_offset_no_change(self):
        """No movement when state already matches NAV2."""
        f = self._fresh_filter(pos_sigma_m=10.0)
        x_before = f.x.copy()
        f.apply_nav2_anchor(self._BASE_ECEF, h_acc_m=1.0)
        np.testing.assert_allclose(f.x, x_before, atol=1e-9)

    def test_loose_hacc_weak_pull(self):
        """When hAcc >> σ_pos, anchor barely moves the state."""
        f = self._fresh_filter(pos_sigma_m=0.1)  # very tight prior
        up = self._BASE_ECEF / np.linalg.norm(self._BASE_ECEF)
        f.x[:3] = self._BASE_ECEF + up * 5.0
        # Loose NAV2 (hAcc=10m) vs tight state (σ=0.1m): K ≈ 0.0001,
        # state should barely move.
        f.apply_nav2_anchor(self._BASE_ECEF, h_acc_m=10.0, v_acc_m=20.0)
        diff = f.x[:3] - (self._BASE_ECEF + up * 5.0)
        self.assertLess(np.linalg.norm(diff), 0.01,
                        f"loose anchor moved tight state too much: {diff}")

    def test_invalid_inputs_noop(self):
        """h_acc<=0 or None: no-op without error."""
        f = self._fresh_filter()
        x_before = f.x.copy()
        f.apply_nav2_anchor(self._BASE_ECEF, h_acc_m=0.0)
        np.testing.assert_allclose(f.x, x_before)
        f.apply_nav2_anchor(self._BASE_ECEF, h_acc_m=None)
        np.testing.assert_allclose(f.x, x_before)
        f.apply_nav2_anchor(self._BASE_ECEF, h_acc_m=-1.0)
        np.testing.assert_allclose(f.x, x_before)

    def test_uninitialized_position_noop(self):
        """If filter state is uninitialized (norm < 1km), bail."""
        f = PPPFilter()
        # Don't call initialize — x is None or zeros (depends on EKF
        # internals).  Force x to all zeros to simulate uninitialized.
        f.x = np.zeros(7)
        f.P = np.eye(7)
        f.apply_nav2_anchor(self._BASE_ECEF, h_acc_m=1.0)
        # Should remain at zero (no pull applied).
        np.testing.assert_allclose(f.x[:3], np.zeros(3))

    def test_covariance_tightens(self):
        """Anchor should reduce position variance (positive Kalman gain)."""
        f = self._fresh_filter(pos_sigma_m=10.0)
        P_before_diag = np.diag(f.P[:3, :3]).copy()
        f.apply_nav2_anchor(self._BASE_ECEF, h_acc_m=1.0, v_acc_m=2.0)
        P_after_diag = np.diag(f.P[:3, :3])
        for i in range(3):
            self.assertLess(P_after_diag[i], P_before_diag[i],
                            f"axis {i}: variance should shrink after anchor")

    def test_covariance_remains_symmetric(self):
        """Numerical symmetry preserved after the update."""
        f = self._fresh_filter(pos_sigma_m=10.0)
        f.apply_nav2_anchor(self._BASE_ECEF, h_acc_m=1.0, v_acc_m=2.0)
        np.testing.assert_allclose(f.P, f.P.T, atol=1e-12)

    def test_default_vacc_is_2x_hacc(self):
        """When v_acc_m is None, default to 2× h_acc_m."""
        f1 = self._fresh_filter(pos_sigma_m=10.0)
        f2 = self._fresh_filter(pos_sigma_m=10.0)
        # Apply with explicit v_acc_m=2.0 vs implicit (None) at h_acc_m=1.0.
        f1.apply_nav2_anchor(self._BASE_ECEF, h_acc_m=1.0, v_acc_m=2.0)
        f2.apply_nav2_anchor(self._BASE_ECEF, h_acc_m=1.0, v_acc_m=None)
        np.testing.assert_allclose(f1.x, f2.x, atol=1e-12)
        np.testing.assert_allclose(f1.P, f2.P, atol=1e-12)


if __name__ == "__main__":
    unittest.main()

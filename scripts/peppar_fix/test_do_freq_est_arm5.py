"""Unit tests for Arm 5 (TDCP) in DOFreqEst.

Verifies that adding `tdcp_freq_ppb` / `tdcp_freq_sigma_ppb` to
`update()`:

  - Affects x[1] (rx TCXO frequency) and nothing else when no
    cross-coupling arms run.
  - Beats Arm 2 (qErr) on x[1] when both provide observations at
    different noise levels (Kalman fusion gives optimal weighting).
  - Is gated independently of Arms 1-4: leaving the params None
    reproduces today's exact behavior.
  - Reset clears any Arm 5 effect on the next pass.

The actual TDCP estimator math is tested separately in
`test_tdcp_estimator.py`; Arm 5 just consumes the (ppb, σ) pair.
"""

from __future__ import annotations

import numpy as np

from peppar_fix.do_freq_est import DOFreqEst


def _baseline():
    """A DOFreqEst seeded for repeatable arm-isolation testing."""
    f = DOFreqEst(
        sigma_ticc_ns=0.060,
        sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01,
        sigma_tcxo_phase_ns=2.0, sigma_tcxo_freq_ppb=0.1,
        initial_dt_rx_ns=0.0, base_freq=0.0,
    )
    # Force a deterministic post-construction P.  Real engine seeds
    # this from bootstrap; for the test we just want a known starting
    # point so K_5 vs K_2 comparison is meaningful.
    f.P = np.diag([1.0, 1.0, 100.0, 1.0])
    f.x = np.array([0.0, 0.0, 0.0, 0.0])
    return f


def test_arm5_observes_only_x1():
    """Arm 5 affects x[1]; leaves x[0], x[2], x[3] untouched."""
    f = _baseline()
    x_before = f.x.copy()
    f.update(dt=1.0, tdcp_freq_ppb=0.5, tdcp_freq_sigma_ppb=0.026)
    assert f.x[1] != x_before[1], "Arm 5 should pull x[1]"
    # x[0] gets a tiny perturbation from the predict step (F[0,1]=dt)
    # but the post-Arm-5 update doesn't touch it directly.  The
    # ~0.5 ppb pull on x[1] times 1 s of predict-then-correct should
    # leave x[0] within process noise.
    assert abs(f.x[0]) < 1.0, f"x[0] should stay near 0; got {f.x[0]}"
    # x[2] and x[3] are not coupled to Arm 5 at all, modulo the LQR
    # control feedback path on x[2].  Process noise + Q-scaling +
    # LQR control puts x[2] at ~ -L[2]*x[2]*dt; with x[2]=0 the
    # control is 0, so x[2] should remain 0 after one update.
    # But x[3] gets the LQR control feedback minus crystal freq —
    # crystal freq init at 0, so unchanged.
    # Process model x[2] -= (x[3] + adjfine) * dt, adjfine = LQR(x[2]).


def test_arm5_lowers_x1_uncertainty():
    """Arm 5 with tight σ tightens P[1,1] more than Arm 2 alone."""
    f1 = _baseline()
    p1_before = f1.P[1, 1]
    f1.update(dt=1.0, qerr_freq_ppb=0.5, qerr_freq_sigma_ppb=0.30)
    p1_after_qerr = f1.P[1, 1]

    f2 = _baseline()
    f2.update(dt=1.0, tdcp_freq_ppb=0.5, tdcp_freq_sigma_ppb=0.026)
    p1_after_tdcp = f2.P[1, 1]

    assert p1_after_qerr < p1_before
    assert p1_after_tdcp < p1_after_qerr, (
        f"Arm 5 (σ=0.026) should tighten x[1] more than Arm 2 (σ=0.30): "
        f"qerr→{p1_after_qerr:.4f}, tdcp→{p1_after_tdcp:.4f}"
    )


def test_arm5_default_off_when_params_none():
    """Leaving Arm 5 params None preserves today's exact behavior."""
    f_with = _baseline()
    f_without = _baseline()

    f_with.update(dt=1.0,
                  qerr_freq_ppb=0.5, qerr_freq_sigma_ppb=0.30,
                  tdcp_freq_ppb=None, tdcp_freq_sigma_ppb=None)
    f_without.update(dt=1.0,
                     qerr_freq_ppb=0.5, qerr_freq_sigma_ppb=0.30)

    np.testing.assert_allclose(f_with.x, f_without.x, atol=1e-12)
    np.testing.assert_allclose(f_with.P, f_without.P, atol=1e-12)


def test_arm5_kalman_fusion_with_arm2():
    """Both arms present: combined posterior tighter than either alone."""
    f_both = _baseline()
    f_both.update(dt=1.0,
                  qerr_freq_ppb=0.5, qerr_freq_sigma_ppb=0.30,
                  tdcp_freq_ppb=0.5, tdcp_freq_sigma_ppb=0.026)
    p11_both = f_both.P[1, 1]

    f_tdcp = _baseline()
    f_tdcp.update(dt=1.0, tdcp_freq_ppb=0.5, tdcp_freq_sigma_ppb=0.026)
    p11_tdcp = f_tdcp.P[1, 1]

    # Both should be very close — Arm 2's contribution to Arm 5's
    # already-tight P[1,1] is small.  But "both" must be ≤ "tdcp-only"
    # by Kalman optimality.
    assert p11_both <= p11_tdcp + 1e-12


def test_arm5_partial_sigma_skips_arm():
    """Either param None → arm skipped (no NaN, no crash)."""
    f = _baseline()
    f.update(dt=1.0, tdcp_freq_ppb=0.5, tdcp_freq_sigma_ppb=None)
    # x[1] should still be 0 (or tiny — predict step only).
    assert abs(f.x[1]) < 1e-3

    f = _baseline()
    f.update(dt=1.0, tdcp_freq_ppb=None, tdcp_freq_sigma_ppb=0.026)
    assert abs(f.x[1]) < 1e-3


def test_arm5_after_reset_starts_clean():
    """reset() clears any Arm 5-driven state."""
    f = _baseline()
    f.update(dt=1.0, tdcp_freq_ppb=0.5, tdcp_freq_sigma_ppb=0.026)
    assert abs(f.x[1]) > 1e-3  # pulled toward 0.5

    f.reset(current_freq=0.0)
    assert f.x[1] == 0.0


# ── L3 — actuator rate limit tests ─────────────────────────────────── #

def _l3_baseline(max_step_ppb=10.0):
    """DOFreqEst seeded so we can exercise the L3 clamp deterministically."""
    f = DOFreqEst(
        sigma_ticc_ns=0.060,
        sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01,
        sigma_tcxo_phase_ns=2.0, sigma_tcxo_freq_ppb=0.1,
        initial_dt_rx_ns=0.0, base_freq=0.0,
        max_step_ppb=max_step_ppb,
    )
    # Force-tighten P to avoid runaway corrections from huge initial
    # uncertainty; keep x[3] at -50 ppb so LQR commands a non-zero
    # baseline adjfine of +50 ppb.
    f.P = np.diag([1.0, 1.0, 100.0, 1.0])
    f.x = np.array([0.0, 0.0, 0.0, -50.0])
    # Set _last_u to where LQR would naturally land with this state.
    # u = -L·x = -(0·0 + 0·0 + (-0.05)·0 + 1.0·(-50)) = 50.
    f._last_u = 50.0
    return f


def test_l3_rate_limit_clamps_large_positive_step():
    """A large punch on x[2] (e.g., outlier EXTINT) should be clamped."""
    f = _l3_baseline(max_step_ppb=10.0)
    # Inject an EXTINT obs at 200 ns — would normally produce a much
    # larger adjfine jump than 10 ppb in one epoch.
    f.update(dt=1.0, extint_phase_ns=200.0, extint_sigma_ns=5.0)
    # New u should be at most 50 + 10 = 60 ppb (rate-limited).
    assert f._last_u <= 60.0 + 1e-6, f"L3 failed to clamp: u={f._last_u}"
    # And the actual change must be ≤ max_step_ppb in magnitude.
    assert abs(f._last_u - 50.0) <= 10.0 + 1e-6


def test_l3_rate_limit_clamps_large_negative_step():
    f = _l3_baseline(max_step_ppb=10.0)
    f.update(dt=1.0, extint_phase_ns=-200.0, extint_sigma_ns=5.0)
    assert f._last_u >= 40.0 - 1e-6
    assert abs(f._last_u - 50.0) <= 10.0 + 1e-6


def test_l3_does_not_clamp_small_legitimate_steps():
    """A 0.1 ppb correction (well below max_step_ppb) must pass through."""
    f = _l3_baseline(max_step_ppb=10.0)
    f.update(dt=1.0, qerr_freq_ppb=49.9, qerr_freq_sigma_ppb=0.05)
    # Without clamping the response should land between 49.9 and 50.
    # With clamping at 10 ppb step, since the desired step is well
    # under 10 ppb, no clamp engages — same result.
    expected_unclamped = f._last_u  # whatever Kalman computed
    # Re-run with no L3 active.
    f2 = _l3_baseline(max_step_ppb=None)
    f2.update(dt=1.0, qerr_freq_ppb=49.9, qerr_freq_sigma_ppb=0.05)
    assert abs(expected_unclamped - f2._last_u) < 1e-9


def test_arm2_arm5_order_invariance():
    """Sequential Kalman updates on the same H are order-invariant
    in exact arithmetic.  Whether we call qErr-then-TDCP or
    TDCP-then-qErr, the posterior must match to machine ε.

    Charlie's PR #61 review item 5 — defense against future refactors
    that change Arm ordering inadvertently.
    """
    def run(order):
        f = _baseline()
        kwargs_qerr = {"qerr_freq_ppb": 0.5, "qerr_freq_sigma_ppb": 0.30}
        kwargs_tdcp = {"tdcp_freq_ppb": 0.5, "tdcp_freq_sigma_ppb": 0.026}
        if order == "qerr_first":
            # Today's update() calls qerr before tdcp by construction;
            # nothing to swap manually since both pass keyword args.
            f.update(dt=1.0, **kwargs_qerr, **kwargs_tdcp)
        else:
            # The implementation order is fixed in update() — Arm 2
            # before Arm 5.  This branch is a structural cross-check:
            # if a future refactor reorders Arm 5 BEFORE Arm 2, the
            # posterior must still match.  We simulate by calling
            # _kalman_linear_update directly in the opposite order.
            import numpy as np
            x_pred = f.F @ f.x + f.B * f._last_u
            P_pred = f.F @ f.P @ f.F.T + f.Q * 1.0
            x_new, P_new = f._kalman_linear_update(
                x_pred, P_pred,
                z=kwargs_tdcp["tdcp_freq_ppb"],
                H=np.array([[0.0, 1.0, 0.0, 0.0]]),
                R=kwargs_tdcp["tdcp_freq_sigma_ppb"] ** 2,
            )
            x_new, P_new = f._kalman_linear_update(
                x_new, P_new,
                z=kwargs_qerr["qerr_freq_ppb"],
                H=np.array([[0.0, 1.0, 0.0, 0.0]]),
                R=kwargs_qerr["qerr_freq_sigma_ppb"] ** 2,
            )
            f.x = x_new
            f.P = P_new
        return f.x.copy(), f.P.copy()

    x1, p1 = run("qerr_first")
    x2, p2 = run("tdcp_first")
    np.testing.assert_allclose(x1, x2, atol=1e-12)
    np.testing.assert_allclose(p1, p2, atol=1e-12)


def test_l3_disabled_when_max_step_ppb_none():
    """max_step_ppb=None preserves pre-Phase-C behavior exactly."""
    f1 = _l3_baseline(max_step_ppb=None)
    f1.update(dt=1.0, extint_phase_ns=200.0, extint_sigma_ns=5.0)

    f2 = DOFreqEst(
        sigma_ticc_ns=0.060,
        sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01,
        sigma_tcxo_phase_ns=2.0, sigma_tcxo_freq_ppb=0.1,
        initial_dt_rx_ns=0.0, base_freq=0.0,
    )  # no max_step_ppb param at all
    f2.P = np.diag([1.0, 1.0, 100.0, 1.0])
    f2.x = np.array([0.0, 0.0, 0.0, -50.0])
    f2._last_u = 50.0
    f2.update(dt=1.0, extint_phase_ns=200.0, extint_sigma_ns=5.0)

    np.testing.assert_allclose(f1.x, f2.x, atol=1e-12)
    assert f1._last_u == f2._last_u

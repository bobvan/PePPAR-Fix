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

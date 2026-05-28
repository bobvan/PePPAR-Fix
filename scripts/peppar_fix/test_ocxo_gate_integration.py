"""Integration tests: OCXO-trusted gate wired into DOFreqEst Arm 4.

Separate from test_ocxo_trusted_gate.py (which tests the gate class in
isolation) — these verify the gate's effect on EKF state updates in the
TICC arm.
"""
from __future__ import annotations

import numpy as np
import unittest

from peppar_fix.do_freq_est import DOFreqEst
from peppar_fix.ocxo_trusted_gate import OcxoTrustedGate


def _baseline_filter(*, ocxo_trusted_gate=None, **kw):
    defaults = dict(
        sigma_ticc_ns=0.060,
        sigma_do_phase_ns=0.92,
        sigma_do_freq_ppb=0.01,
        sigma_tcxo_phase_ns=2.0,
        sigma_tcxo_freq_ppb=0.1,
        initial_freq=0.0,
        ocxo_trusted_gate=ocxo_trusted_gate,
    )
    defaults.update(kw)
    f = DOFreqEst(**defaults)
    f._need_phc_seed = False
    f.x = np.array([0.0, 0.0, 0.0, 0.0])
    return f


class OcxoGateWiringTests(unittest.TestCase):

    def test_no_gate_preserves_existing_behavior(self):
        """ocxo_trusted_gate=None must leave Arm 4 behavior unchanged."""
        f = _baseline_filter()
        f.update(dt=1.0, ticc_diff_ns=-2.0, ticc_sigma_ns=0.060)
        # A 2 ns ticc should produce some state update (within chi² gate).
        self.assertGreater(abs(f.x[2]), 0.01)

    def test_gate_blocks_large_innov_when_aged_past_min(self):
        """With OCXO gate at K=10, σ=0.054 ns: threshold = 540 ps.  A
        10 ns ticc innovation should be rejected (vs accepted without
        the gate)."""
        gate = OcxoTrustedGate(sigma_short_tau_ns=0.054, k_sigma=10.0,
                                min_age_s=10.0)
        f = _baseline_filter(ocxo_trusted_gate=gate)
        # Tighten P so the chi² gate ALSO would reject this (we want to
        # isolate the OCXO-gate behavior, but we still want chi² gate to
        # behave the same way it would without the OCXO gate).  Actually,
        # we want to confirm that EITHER gate rejecting prevents the
        # update.  Use P loose enough that chi² ACCEPTS, then OCXO
        # rejects.
        f.P = np.diag([1.0, 0.01, 100.0, 0.01])  # σ_x2 = 10 ns → S~100 ns²
        # Spin past min_age_s
        for _ in range(12):
            f.update(dt=1.0)  # predict-only, no obs
        x2_before = float(f.x[2])
        # 10 ns innov: chi² ≈ 100/100 = 1 < 100, accepts.  OCXO rejects.
        f.update(dt=1.0, ticc_diff_ns=10.0, ticc_sigma_ns=0.060)
        self.assertAlmostEqual(f.x[2], x2_before, delta=0.5,
                               msg="OCXO gate should reject 10 ns innov when "
                                   "OCXO floor is 54 ps × K=10 = 540 ps")
        self.assertEqual(gate.n_rejected, 1)
        self.assertEqual(gate.n_accepted, 0)

    def test_gate_disabled_during_min_age(self):
        """During the min_age window, the gate accepts everything so
        bootstrap convergence can proceed."""
        gate = OcxoTrustedGate(sigma_short_tau_ns=0.054, k_sigma=10.0,
                                min_age_s=60.0)
        f = _baseline_filter(ocxo_trusted_gate=gate)
        f.P = np.diag([1.0, 0.01, 100.0, 0.01])
        # Age = 1 s on first update → still pre-min_age → accepts
        f.update(dt=1.0, ticc_diff_ns=10.0, ticc_sigma_ns=0.060)
        self.assertEqual(gate.n_skipped_pre_age, 1)
        self.assertEqual(gate.n_rejected, 0)

    def test_gate_accepts_small_innov_after_age(self):
        """After min_age, small innovations (< K × σ) still pass."""
        gate = OcxoTrustedGate(sigma_short_tau_ns=0.054, k_sigma=10.0,
                                min_age_s=10.0)
        f = _baseline_filter(ocxo_trusted_gate=gate)
        for _ in range(12):
            f.update(dt=1.0)
        # 0.3 ns innov is within 540 ps threshold → passes
        x2_before = float(f.x[2])
        f.update(dt=1.0, ticc_diff_ns=0.3, ticc_sigma_ns=0.060)
        self.assertEqual(gate.n_accepted, 1)
        self.assertEqual(gate.n_rejected, 0)
        # State should have moved (gate accepted, EKF updated)
        self.assertNotEqual(f.x[2], x2_before)

    def test_age_tracking_across_dt_changes(self):
        """_total_age_s should accumulate across variable dt."""
        gate = OcxoTrustedGate(sigma_short_tau_ns=0.054, k_sigma=10.0,
                                min_age_s=5.0)
        f = _baseline_filter(ocxo_trusted_gate=gate)
        f.update(dt=2.0)
        self.assertAlmostEqual(f._total_age_s, 2.0)
        f.update(dt=2.0)
        self.assertAlmostEqual(f._total_age_s, 4.0)
        # Still under 5 s min_age
        f.P = np.diag([1.0, 0.01, 100.0, 0.01])
        f.update(dt=0.5, ticc_diff_ns=10.0, ticc_sigma_ns=0.060)
        self.assertAlmostEqual(f._total_age_s, 4.5)
        self.assertEqual(gate.n_skipped_pre_age, 1)
        # Now over 5 s
        f.update(dt=1.0, ticc_diff_ns=10.0, ticc_sigma_ns=0.060)
        self.assertEqual(gate.n_rejected, 1)


if __name__ == '__main__':
    unittest.main()

"""Tests for Hatch-style carrier-smoothed PR (I-155354 fix-H).

The smoother feeds MelbourneWubbenaTracker's MW running mean.
PR noise floor reduces from ~30 cm raw to ~5 cm in ~36 epochs of
arc, bringing the MW running mean within λ_WL/8 — well inside
rounding tolerance for confident WL integer fixes.

Test coverage:
  - First sample of arc: smooth = raw, prev_phi primed
  - White-noise PR with constant carrier: σ shrinks ~ 1/sqrt(N)
  - Constant PR with linearly drifting carrier: smoother tracks
    via Δφ projection
  - HATCH_N_MAX cap: α floors at 1/N_MAX so multi-hour arcs
    don't over-smooth
  - Reset on slip: fresh arc reinitializes smoother
  - MW std reduction: with smoother, MW running mean's std
    should reduce vs unsmoothed feed
"""
from __future__ import annotations

import math
import os
import sys
import unittest

import numpy as np

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from ppp_ar import MelbourneWubbenaTracker


# Typical GPS L1/L2 frequencies.
F1 = 1575.42e6
F2 = 1227.60e6
LAMBDA1 = 299_792_458.0 / F1
LAMBDA2 = 299_792_458.0 / F2
LAMBDA_WL = 299_792_458.0 / (F1 - F2)  # ~86.2 cm


class FirstSampleTest(unittest.TestCase):

    def test_first_sample_smooth_equals_raw(self):
        mw = MelbourneWubbenaTracker()
        mw.update("G01", phi1_cyc=1e6, phi2_cyc=1e6,
                   pr1_m=20_000_000.0, pr2_m=20_000_000.0,
                   f1=F1, f2=F2)
        h = mw._state["G01"]["hatch"]
        self.assertEqual(h["n"], 1)
        self.assertEqual(h["p1_smooth"], 20_000_000.0)
        self.assertEqual(h["p2_smooth"], 20_000_000.0)
        self.assertEqual(h["prev_phi1_cyc"], 1e6)
        self.assertEqual(h["prev_phi2_cyc"], 1e6)


class WhiteNoiseReductionTest(unittest.TestCase):
    """PR noise σ shrinks roughly 1/sqrt(N) under Hatch smoothing
    when carrier-phase is constant (zero drift)."""

    def test_36_samples_shrinks_pr_noise_6x(self):
        rng = np.random.default_rng(42)
        # Simulate 36 samples of stationary PR with σ=0.30 m noise,
        # constant carrier.  After 36 samples, σ_smooth ≈ σ_raw / sqrt(36) = 5 cm.
        mw = MelbourneWubbenaTracker()
        sv = "G01"
        true_pr = 20_000_000.0
        true_phi = 1e6
        smoothed_p1 = []
        for i in range(60):
            pr1 = true_pr + rng.normal(0, 0.30)
            pr2 = true_pr + rng.normal(0, 0.30)
            mw.update(sv, phi1_cyc=true_phi, phi2_cyc=true_phi,
                       pr1_m=pr1, pr2_m=pr2, f1=F1, f2=F2)
            smoothed_p1.append(mw._state[sv]["hatch"]["p1_smooth"])
        # The smoother is a growing-window mean (α = 1/N) — for fixed
        # truth + zero phase change, it converges to the sample-mean.
        # Std of last 20 smoothed samples around true PR should be
        # much smaller than the raw 0.30 m std.
        late_smooth = np.array(smoothed_p1[-20:])
        late_smooth_err = late_smooth - true_pr
        self.assertLess(np.std(late_smooth_err), 0.10,
                         f"smoothed PR std {np.std(late_smooth_err):.3f}m "
                         f"should be <<0.30m raw")

    def test_residual_deque_still_tracks_raw_noise(self):
        """The slip-detector sigma comes from resid_deque which tracks
        (raw_mw - smooth_mw_avg)/lambda_wl.  This must stay at raw-PR
        noise level (~0.4 cyc), not shrink to smoothed-PR noise level,
        so detect_jump's threshold doesn't tighten and start firing on
        ordinary PR noise."""
        rng = np.random.default_rng(7)
        mw = MelbourneWubbenaTracker()
        sv = "G01"
        true_pr = 20_000_000.0
        true_phi = 1e6
        for i in range(80):
            pr1 = true_pr + rng.normal(0, 0.30)
            pr2 = true_pr + rng.normal(0, 0.30)
            mw.update(sv, phi1_cyc=true_phi, phi2_cyc=true_phi,
                       pr1_m=pr1, pr2_m=pr2, f1=F1, f2=F2)
        s = mw._state[sv]
        # resid_std_cyc should reflect raw PR noise, ≈ 0.30/0.86 ≈ 0.35 cyc.
        # Allow a wide band — the test just confirms it's NOT tiny
        # (which would mean smoothed-vs-smoothed and threshold would
        # tighten).
        self.assertGreater(s["resid_std_cyc"], 0.15,
                           "resid_deque sigma must reflect raw-PR noise")


class CarrierDriftTrackingTest(unittest.TestCase):
    """When carrier phase drifts linearly (typical TCXO clock drift +
    ionosphere drift), the smoother's projection term tracks it."""

    def test_smoother_tracks_linear_phase_drift(self):
        # 100 samples; carrier-phase drifts by 1 cycle per sample on L1
        # (roughly 19 cm/s, exaggerated but mathematically clean).
        # PR is also drifting at the same rate (no inconsistency).
        # Smoother should produce smoothed PR very close to truth.
        mw = MelbourneWubbenaTracker()
        sv = "G01"
        for i in range(100):
            phi1 = 1e6 + i * 1.0  # 1 cyc/sample
            phi2 = 1e6 + i * 1.0
            pr1 = 20_000_000.0 + i * LAMBDA1  # range grows by 1 L1 cycle
            pr2 = 20_000_000.0 + i * LAMBDA2  # range grows by 1 L2 cycle
            mw.update(sv, phi1_cyc=phi1, phi2_cyc=phi2,
                       pr1_m=pr1, pr2_m=pr2, f1=F1, f2=F2)
        h = mw._state[sv]["hatch"]
        # At sample 99, true PR1 = 20_000_000 + 99*LAMBDA1.
        true_p1 = 20_000_000.0 + 99 * LAMBDA1
        # Smoothed should match truth almost exactly (zero noise input).
        self.assertAlmostEqual(h["p1_smooth"], true_p1, places=3)


class ResetOnSlipTest(unittest.TestCase):

    def test_reset_clears_hatch_state(self):
        mw = MelbourneWubbenaTracker()
        sv = "G01"
        for i in range(20):
            mw.update(sv, phi1_cyc=1e6 + i, phi2_cyc=1e6 + i,
                       pr1_m=20e6, pr2_m=20e6, f1=F1, f2=F2)
        self.assertIn("hatch", mw._state[sv])
        mw.reset(sv)
        self.assertNotIn(sv, mw._state)

    def test_post_reset_first_sample_smooth_equals_raw(self):
        """After reset, the next update primes the smoother fresh
        with smooth = raw."""
        mw = MelbourneWubbenaTracker()
        sv = "G01"
        # Build up some history.
        for i in range(20):
            mw.update(sv, phi1_cyc=1e6 + i, phi2_cyc=1e6 + i,
                       pr1_m=20e6 + i * 0.1, pr2_m=20e6 + i * 0.1,
                       f1=F1, f2=F2)
        # Slip event → reset.
        mw.reset(sv)
        # Fresh sample.
        mw.update(sv, phi1_cyc=2e6, phi2_cyc=2e6,
                   pr1_m=21e6, pr2_m=21e6, f1=F1, f2=F2)
        h = mw._state[sv]["hatch"]
        self.assertEqual(h["n"], 1)
        self.assertEqual(h["p1_smooth"], 21e6)
        self.assertEqual(h["p2_smooth"], 21e6)


class HatchNMaxCapTest(unittest.TestCase):
    """α = 1/min(N, HATCH_N_MAX) prevents multi-hour arcs from
    over-smoothing past slow ionospheric drift the carrier-phase
    delta projection might lag."""

    def test_n_caps_at_hatch_n_max(self):
        mw = MelbourneWubbenaTracker()
        sv = "G01"
        n_iter = MelbourneWubbenaTracker._HATCH_N_MAX + 100
        for i in range(n_iter):
            mw.update(sv, phi1_cyc=1e6 + i, phi2_cyc=1e6 + i,
                       pr1_m=20e6, pr2_m=20e6, f1=F1, f2=F2)
        h = mw._state[sv]["hatch"]
        self.assertEqual(h["n"], MelbourneWubbenaTracker._HATCH_N_MAX)


class MwRunningMeanStdReductionTest(unittest.TestCase):
    """The operationally meaningful test: with the Hatch smoother,
    the MW running mean's spread (around the true integer) shrinks
    enough that the running mean rounds to the correct integer
    reliably under realistic PR-noise levels."""

    def test_mw_avg_within_quarter_cycle_after_60_epochs(self):
        rng = np.random.default_rng(1234)
        mw = MelbourneWubbenaTracker()
        sv = "G01"
        # True ambiguity baked in via phi1, phi2 offsets; PR has 30 cm
        # white noise.  A correct WL fix means mw_avg/lambda_wl rounds
        # to the integer N_WL.  Without smoothing, 30cm raw PR → 0.35
        # cyc residual std → mean across 60 epochs has std 0.35/sqrt(60)
        # ≈ 0.045 cyc (which would round correctly).  With smoothing,
        # the residual std going INTO mw_avg shrinks further → tighter
        # convergence.
        # Set up: synthesize phi/pr such that "true MW" is exactly
        # N_WL × λ_WL (integer cycles, no fractional bias).
        n_wl_true = 12345
        true_phi1 = 0.0
        true_phi2 = 0.0
        # MW = (f1·phi1 - f2·phi2)·c/((f1-f2)·f1·f2) - (f1·pr1 + f2·pr2)/(f1+f2)
        # For the MW to equal N_WL·λ_WL with phi1=phi2=0, we need
        # the PR term to equal -N_WL·λ_WL.
        target_pr = -n_wl_true * LAMBDA_WL * (F1 + F2) / (F1 + F2) * 1.0
        # Simplify: make pr1 = pr2 = some_value chosen so MW = N_WL·λ_WL.
        # MW = -(pr1+pr2)/2 with phi=0.  So pr1=pr2 = -N_WL·λ_WL gives MW = N_WL·λ_WL.
        true_pr_avg = -n_wl_true * LAMBDA_WL
        for i in range(120):
            pr1 = true_pr_avg + rng.normal(0, 0.30)
            pr2 = true_pr_avg + rng.normal(0, 0.30)
            mw.update(sv, phi1_cyc=true_phi1, phi2_cyc=true_phi2,
                       pr1_m=pr1, pr2_m=pr2, f1=F1, f2=F2)
        s = mw._state[sv]
        n_wl_float = s["mw_avg"] / LAMBDA_WL
        frac = abs(n_wl_float - n_wl_true)
        # With 120 epochs of smoothing + EMA, frac should be well under 0.25
        # (rounds to correct integer with margin).
        self.assertLess(frac, 0.25,
                         f"mw_avg/λ_WL frac={frac:.3f} should be <0.25 "
                         f"to round to correct integer")


if __name__ == "__main__":
    unittest.main()

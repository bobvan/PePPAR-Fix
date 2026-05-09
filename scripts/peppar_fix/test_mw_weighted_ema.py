"""Unit tests for the MelbourneWubbenaTracker's weighted EMA (fix-L).

Background: I-155354 fix-L.  The MW running mean was an unweighted
EMA — every per-epoch sample contributed equally regardless of
elevation or CN0.  Low-elev / low-CN0 samples have higher noise
(meters of multipath, multi-cycle PR error) and pull the running
mean off the correct integer.  Fix-L scales the EMA's alpha by a
quality weight in [0, 1]; a weight of 1.0 reproduces the legacy
behavior exactly.

Tests cover:
  - weight=None (or absent) ≡ unweighted (legacy path).
  - weight=1.0 always ≡ unweighted (no semantic drift).
  - Low-weight samples advance n_eff (and thus the fix-readiness
    gate) slowly while still ticking n_epochs for arc-tracking.
  - High-weight samples converge mw_avg quickly to the integer.
  - Mixed high+low weight: running mean follows the high-weight
    samples' truth, ignores the low-weight outliers.
"""
from __future__ import annotations

import unittest

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ppp_ar import MelbourneWubbenaTracker, C


# Galileo E1/E5a frequencies — λ_WL ≈ 0.751 m
F_L1 = 1575.42e6
F_E5A = 1176.45e6
LAMBDA_WL = C / (F_L1 - F_E5A)


def _mw_obs_for_int(n_wl, pr_offset_m=0.0, phi_offset_cyc=0.0):
    """Synthesize MW-formula inputs that produce an MW value of exactly
    n_wl × λ_WL (in meters) when fed to the tracker's _mw_meters().

    The engine's _mw_meters reduces to:

      MW = (φ1_cyc - φ2_cyc) × λ_WL - (f1·P1 + f2·P2) / (f1+f2)

    With pr1 = pr2 = pr_offset and φ2 = 0:

      MW = φ1·λ_WL - pr_offset·(f1+f2)/(f1+f2)
         = φ1·λ_WL - pr_offset

    So MW = n_wl·λ_WL when φ1 = n_wl + pr_offset/λ_WL.  With
    pr_offset=0 this is just φ1 = n_wl, φ2 = 0.
    """
    phi1 = n_wl + pr_offset_m / LAMBDA_WL + phi_offset_cyc
    phi2 = 0.0
    return phi1, phi2, pr_offset_m, pr_offset_m


class WeightedEmaConvergenceTest(unittest.TestCase):

    def setUp(self):
        # min_epochs=10 so tests don't need to run hundreds of samples
        # to cross the fix gate; tau_s default doesn't matter for n_eff
        # gate (we don't need cap behavior in these short tests).
        self.tracker = MelbourneWubbenaTracker(
            tau_s=600.0, min_epochs=10, fix_threshold=0.15)

    def test_weight_none_matches_unweighted(self):
        """weight=None (default) reproduces legacy unweighted EMA."""
        sv = 'G01'
        n_wl_truth = 17
        phi1, phi2, pr1, pr2 = _mw_obs_for_int(n_wl_truth)
        for _ in range(20):
            self.tracker.update(sv, phi1, phi2, pr1, pr2, F_L1, F_E5A)
        s = self.tracker._state[sv]
        # mw_avg should equal n_wl_truth × λ_WL exactly (no noise)
        expected_mw = n_wl_truth * LAMBDA_WL
        self.assertAlmostEqual(s['mw_avg'], expected_mw, places=6)
        self.assertEqual(s['n_eff'], 20.0)
        self.assertTrue(s['fixed'])
        self.assertEqual(s['n_wl'], n_wl_truth)

    def test_weight_one_matches_unweighted(self):
        """Explicit weight=1.0 reproduces legacy unweighted EMA."""
        sv = 'G02'
        n_wl_truth = 17
        phi1, phi2, pr1, pr2 = _mw_obs_for_int(n_wl_truth)
        for _ in range(20):
            self.tracker.update(sv, phi1, phi2, pr1, pr2,
                                  F_L1, F_E5A, weight=1.0)
        s = self.tracker._state[sv]
        self.assertEqual(s['n_eff'], 20.0)
        self.assertTrue(s['fixed'])
        self.assertEqual(s['n_wl'], n_wl_truth)

    def test_low_weight_delays_fix_readiness(self):
        """Low-weight samples accumulate n_eff slowly; fix gate doesn't
        fire until n_eff ≥ min_epochs even though n_epochs ≥ min_epochs.
        """
        sv = 'G03'
        n_wl_truth = 17
        phi1, phi2, pr1, pr2 = _mw_obs_for_int(n_wl_truth)
        # weight 0.1 per sample → 100 samples needed for n_eff = 10
        for _ in range(50):
            self.tracker.update(sv, phi1, phi2, pr1, pr2,
                                  F_L1, F_E5A, weight=0.1)
        s = self.tracker._state[sv]
        # After 50 samples × 0.1 weight, n_eff = 5 (< min_epochs=10)
        # so the SV is NOT yet fixed.
        self.assertAlmostEqual(s['n_eff'], 5.0, places=4)
        self.assertEqual(s['n_epochs'], 50)
        self.assertFalse(s['fixed'])

        # Continue another 60 samples to push n_eff over min_epochs
        for _ in range(60):
            self.tracker.update(sv, phi1, phi2, pr1, pr2,
                                  F_L1, F_E5A, weight=0.1)
        s = self.tracker._state[sv]
        self.assertGreaterEqual(s['n_eff'], 10.0)
        self.assertTrue(s['fixed'])
        self.assertEqual(s['n_wl'], n_wl_truth)

    def test_zero_weight_does_not_advance_running_mean(self):
        """weight=0 samples leave mw_avg untouched but tick n_epochs.

        Useful when an SV is below the elev/CN0 floor — engine still
        sees it for arc-tracking (slip detector, etc.) but it must not
        contribute to the integer running mean.
        """
        sv = 'G04'
        # Seed with high-quality samples
        phi1, phi2, pr1, pr2 = _mw_obs_for_int(17)
        for _ in range(5):
            self.tracker.update(sv, phi1, phi2, pr1, pr2,
                                  F_L1, F_E5A, weight=1.0)
        s = self.tracker._state[sv]
        seed_mw_avg = s['mw_avg']
        seed_n_eff = s['n_eff']

        # Inject 10 zero-weight outlier samples (with very different
        # truth integer) — running mean should NOT move.
        phi1_outlier, phi2_outlier, _, _ = _mw_obs_for_int(99)
        for _ in range(10):
            self.tracker.update(sv, phi1_outlier, phi2_outlier,
                                  pr1, pr2, F_L1, F_E5A, weight=0.0)
        s = self.tracker._state[sv]
        self.assertAlmostEqual(s['mw_avg'], seed_mw_avg, places=6,
            msg='zero-weight samples should not move mw_avg')
        self.assertEqual(s['n_eff'], seed_n_eff,
            msg='zero-weight samples should not advance n_eff')
        self.assertEqual(s['n_epochs'], 15,
            msg='zero-weight samples DO tick n_epochs (arc-tracking)')

    def test_high_weight_outweighs_low_weight_outliers(self):
        """Mix of high-weight truth + low-weight outliers — mw_avg
        converges to truth.  Empirically the situation MW faces with
        a few low-elev SVs feeding noisy cycles into an otherwise-clean
        running mean.
        """
        sv = 'G05'
        n_wl_truth = 17
        phi1, phi2, pr1, pr2 = _mw_obs_for_int(n_wl_truth)
        phi1_off, phi2_off, _, _ = _mw_obs_for_int(n_wl_truth + 5)
        # 100 high-weight truth samples + interspersed low-weight outliers
        for i in range(100):
            self.tracker.update(sv, phi1, phi2, pr1, pr2,
                                  F_L1, F_E5A, weight=1.0)
            if i % 10 == 0:
                # 10 outliers, each 1/10 the weight of the truth samples
                self.tracker.update(sv, phi1_off, phi2_off, pr1, pr2,
                                      F_L1, F_E5A, weight=0.1)
        s = self.tracker._state[sv]
        # mw_avg should be well within λ_WL/2 rounding tolerance.  With
        # 100 truth samples × 1.0 + 10 outliers × 0.1 = 1.0 cumulative
        # outlier weight in a 101-sample window, EMA-recency biases the
        # final mw_avg by a few percent of the outlier offset (5 cyc),
        # producing ~0.1-0.15 cyc residual.  Far less than the 0.5 cyc
        # rounding tolerance — the integer still rounds correctly.
        truth_mw = n_wl_truth * LAMBDA_WL
        offset_cyc = (s['mw_avg'] - truth_mw) / LAMBDA_WL
        self.assertLess(abs(offset_cyc), 0.30,
            f'truth-weighted mw_avg drifted {offset_cyc:.3f} cycles — '
            f'expected < 0.30 (well inside λ_WL/2 rounding tolerance)')
        # And it fixed to the right integer.
        self.assertTrue(s['fixed'])
        self.assertEqual(s['n_wl'], n_wl_truth)

    def test_first_sample_seeds_n_eff_at_first_weight(self):
        """First sample of an arc seeds n_eff at the first weight,
        not at 1.0 — so a low-quality arc-start doesn't overstate
        the evidence.
        """
        sv = 'G06'
        phi1, phi2, pr1, pr2 = _mw_obs_for_int(17)
        self.tracker.update(sv, phi1, phi2, pr1, pr2,
                              F_L1, F_E5A, weight=0.3)
        s = self.tracker._state[sv]
        self.assertEqual(s['n_eff'], 0.3)
        self.assertEqual(s['n_epochs'], 1)


if __name__ == '__main__':
    unittest.main()

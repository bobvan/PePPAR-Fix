"""Tests for FixedPosFilter catastrophic residual gate (I-202649 v2).

Residual-consistency design: trip on transient spike from a stable
baseline.  Warmup-quiet during the first N epochs so bootstrap
clock-rate convergence (which produces a linear ramp of residuals)
doesn't trigger spurious rejects.

Motivated by MadHat's death 2026-05-08 14:49 UTC — a single-epoch
F9T serial-corruption burst injected coordinated +339 m PR
residuals on three SVs that the existing MAD outlier path couldn't
catch (residuals all agreed with each other → MAD-vs-median saw
no outlier).

The earlier fixed-threshold v1 design tripped during DO bootstrap
when residuals legitimately grew at the bare-TCXO rate per epoch
until the rate state converged; that's a linear ramp, not a burst,
and v2 distinguishes between them via the rolling-baseline ratio.
"""
from __future__ import annotations

import math
import os
import sys
import unittest
from datetime import datetime, timedelta, timezone
from unittest.mock import MagicMock

import numpy as np

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from solve_ppp import FixedPosFilter

_BASE_ECEF = np.array([157470.222, -4756189.544, 4232767.952])


def _sat_pos_above_site(elev_deg, az_deg=0.0, range_m=22e6):
    e = math.radians(elev_deg)
    a = math.radians(az_deg)
    up = _BASE_ECEF / np.linalg.norm(_BASE_ECEF)
    east = np.cross(np.array([0., 0., 1.]), up)
    east /= np.linalg.norm(east)
    north = np.cross(up, east)
    site_frame = (math.cos(e) * (math.sin(a) * east + math.cos(a) * north)
                  + math.sin(e) * up)
    return _BASE_ECEF + range_m * site_frame


def _make_obs(sv, sys_name, pr_if_m, phi_if_m=None, cno_db=45.0):
    return {
        'sv': sv,
        'sys': sys_name,
        'pr_if': float(pr_if_m),
        'phi_if_m': None if phi_if_m is None else float(phi_if_m),
        'cno': float(cno_db),
    }


def _build_filter_with_history(n_warmup_epochs=8, pr_offset_m=0.5):
    """Initialized FixedPosFilter with `n_warmup_epochs` of history.

    Returns (filter, sp3, ranges, last_t).  History buffer contains
    `n_warmup_epochs` median |PR| values around `pr_offset_m`, so
    the baseline-ratio gate is active when the test starts.
    """
    f = FixedPosFilter(_BASE_ECEF)
    f.x[FixedPosFilter.IDX_CLK] = 0.0
    f.P[FixedPosFilter.IDX_CLK, FixedPosFilter.IDX_CLK] = 100.0 ** 2
    f.initialized = True
    f.prev_clock = 0.0
    sat_positions = {
        "G01": _sat_pos_above_site(60.0, az_deg=0.0),
        "G02": _sat_pos_above_site(45.0, az_deg=90.0),
        "G03": _sat_pos_above_site(30.0, az_deg=180.0),
        "G04": _sat_pos_above_site(70.0, az_deg=270.0),
    }
    sp3 = MagicMock()
    sp3.sat_position = MagicMock(
        side_effect=lambda sv, t_tx: (
            np.asarray(sat_positions[sv], dtype=float), 0.0))
    ranges = {sv: float(np.linalg.norm(sat_positions[sv] - _BASE_ECEF))
              for sv in sat_positions}
    t = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
    # Run warmup epochs with stable, sub-meter residuals — populates
    # _pr_median_history so subsequent epochs can be gated.
    for k in range(n_warmup_epochs):
        obs = [_make_obs(sv, "gps",
                         ranges[sv] + pr_offset_m,
                         phi_if_m=ranges[sv] + 0.001)
               for sv in ranges]
        f.update(obs, sp3, t + timedelta(seconds=k), clk_file=None)
    return f, sp3, ranges, t + timedelta(seconds=n_warmup_epochs - 1)


class CatastrophicGateTest(unittest.TestCase):

    def test_clean_observations_pass_through(self):
        """Stable filter, sub-meter residuals: gate never trips."""
        f, sp3, ranges, t_last = _build_filter_with_history()
        x_before = f.x.copy()
        # One more clean epoch.
        obs = [_make_obs(sv, "gps",
                         ranges[sv] + 0.5,
                         phi_if_m=ranges[sv] + 0.001)
               for sv in ranges]
        f.update(obs, sp3, t_last + timedelta(seconds=1), clk_file=None)
        # Update happened — state moved, counter clean.
        self.assertTrue(np.any(f.x != x_before),
                        "clean update should move state")
        self.assertEqual(f._consecutive_catastrophic_rejects, 0)

    def test_339m_burst_after_stable_rejects(self):
        """The MadHat 2026-05-08 14:49 failure mode: stable filter, then
        one epoch with all 4 SVs at +339m.  After warmup, baseline
        is sub-meter; ratio = 339 / 5 (floor) = 68× → trip."""
        f, sp3, ranges, t_last = _build_filter_with_history()
        x_before = f.x.copy()
        prev_clock_before = f.prev_clock
        history_len_before = len(f._pr_median_history)
        # All 4 SVs corrupted at +339 m — coordinated burst.
        obs = [_make_obs(sv, "gps",
                         ranges[sv] + 339.0,
                         phi_if_m=ranges[sv] + 339.0)
               for sv in ranges]
        n_pr, z, n_td = f.update(
            obs, sp3, t_last + timedelta(seconds=1), clk_file=None)
        # Gate rejected — return value is the n_pr=0 / empty-z signal.
        self.assertEqual(n_pr, 0)
        self.assertEqual(len(z), 0)
        # State, prev_clock untouched.
        np.testing.assert_array_equal(f.x, x_before)
        self.assertEqual(f.prev_clock, prev_clock_before)
        # Pre-fit residuals exposed for diagnostic visibility.
        self.assertEqual(f.last_n_pr, 4)
        self.assertEqual(len(f.last_resid_pr), 4)
        self.assertTrue(np.all(np.abs(f.last_resid_pr) > 100.0))
        # Counter incremented.
        self.assertEqual(f._consecutive_catastrophic_rejects, 1)
        # History NOT updated by rejected epoch.
        self.assertEqual(len(f._pr_median_history), history_len_before)

    def test_linear_rate_divergence_passes_through(self):
        """The DO-bootstrap-clock-estimate scenario that broke v1:
        filter just got seeded with a fresh clock estimate but its
        rate state is still loose; each subsequent epoch's residual
        grows linearly because the rate hasn't been learned yet.
        Pattern: 107 → 214 → 321 → 428 → 535 m, each ~107 m larger
        than the prior.

        v1's fixed 100m threshold tripped on the very first epoch and
        locked the filter out — preventing the clock-rate state from
        ever converging.  v2's warmup-quiet design starts gate-disabled
        (no history yet) and lets the filter learn.  Tests the path
        that bit MadHat 2026-05-08 16:05."""
        # Fresh filter, no warmup — simulates the just-seeded state
        # where DO bootstrap is running its 10-epoch clock-estimate
        # phase.
        f = FixedPosFilter(_BASE_ECEF)
        f.x[FixedPosFilter.IDX_CLK] = 0.0
        f.P[FixedPosFilter.IDX_CLK, FixedPosFilter.IDX_CLK] = 1e8
        f.initialized = True
        f.prev_clock = 0.0
        sat_positions = {
            "G01": _sat_pos_above_site(60.0, az_deg=0.0),
            "G02": _sat_pos_above_site(45.0, az_deg=90.0),
            "G03": _sat_pos_above_site(30.0, az_deg=180.0),
            "G04": _sat_pos_above_site(70.0, az_deg=270.0),
        }
        sp3 = MagicMock()
        sp3.sat_position = MagicMock(
            side_effect=lambda sv, t_tx: (
                np.asarray(sat_positions[sv], dtype=float), 0.0))
        ranges = {sv: float(np.linalg.norm(sat_positions[sv] - _BASE_ECEF))
                  for sv in sat_positions}
        t = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
        # Linear ramp: each epoch's residuals grow by ~107 m.
        for i, growth_m in enumerate([107.0, 214.0, 321.0, 428.0, 535.0]):
            obs = [_make_obs(sv, "gps",
                             ranges[sv] + growth_m,
                             phi_if_m=ranges[sv] + growth_m)
                   for sv in ranges]
            n_pr, _, _ = f.update(
                obs, sp3, t + timedelta(seconds=i), clk_file=None)
            self.assertGreater(n_pr, 0,
                f"linear-rate-divergence epoch {i} (growth={growth_m}m) "
                f"must not trip the gate; v1's fixed threshold broke "
                f"MadHat bootstrap by tripping here")
        # No rejects accumulated — gate stayed inactive across the
        # full warmup-quiet window.
        self.assertEqual(f._consecutive_catastrophic_rejects, 0)

    def test_first_epoch_catastrophic_passes_warmup(self):
        """Documented tradeoff: the gate is warmup-quiet for the first
        CATASTROPHIC_HISTORY_MIN epochs.  A first-epoch catastrophic
        residual gets accepted; the existing 30-outlier servo cascade
        catches it downstream (~30 s detection latency).

        The lockout pathology v1 had during DO bootstrap is what this
        tradeoff buys us — the cure was worse than the disease."""
        f = FixedPosFilter(_BASE_ECEF)
        f.x[FixedPosFilter.IDX_CLK] = 0.0
        f.P[FixedPosFilter.IDX_CLK, FixedPosFilter.IDX_CLK] = 100.0 ** 2
        f.initialized = True
        f.prev_clock = 0.0
        sat_positions = {
            "G01": _sat_pos_above_site(60.0, az_deg=0.0),
            "G02": _sat_pos_above_site(45.0, az_deg=90.0),
            "G03": _sat_pos_above_site(30.0, az_deg=180.0),
            "G04": _sat_pos_above_site(70.0, az_deg=270.0),
        }
        sp3 = MagicMock()
        sp3.sat_position = MagicMock(
            side_effect=lambda sv, t_tx: (
                np.asarray(sat_positions[sv], dtype=float), 0.0))
        ranges = {sv: float(np.linalg.norm(sat_positions[sv] - _BASE_ECEF))
                  for sv in sat_positions}
        # First-epoch catastrophic: no history yet, gate inactive.
        obs = [_make_obs(sv, "gps",
                         ranges[sv] + 339.0,
                         phi_if_m=ranges[sv] + 339.0)
               for sv in sat_positions]
        t = datetime(2026, 5, 5, 12, 0, 0, tzinfo=timezone.utc)
        n_pr, _, _ = f.update(obs, sp3, t, clk_file=None)
        # Gate did NOT fire (no history).
        self.assertGreater(n_pr, 0)
        self.assertEqual(f._consecutive_catastrophic_rejects, 0)

    def test_one_bad_sv_among_clean_passes_through(self):
        """One bad SV among 4 clean ones: median |PR| dominated by
        the 3 clean residuals (sub-meter), ratio stays low, gate
        does not fire.  MAD outlier rejection (when enabled) is the
        right defence here."""
        f, sp3, ranges, t_last = _build_filter_with_history()
        x_before = f.x.copy()
        obs = []
        for i, sv in enumerate(ranges):
            offset = 339.0 if i == 0 else 0.5
            obs.append(_make_obs(sv, "gps",
                                  ranges[sv] + offset,
                                  phi_if_m=ranges[sv] + 0.001))
        f.update(obs, sp3, t_last + timedelta(seconds=1), clk_file=None)
        # Update happened.
        self.assertTrue(np.any(f.x != x_before),
                        "1-of-4 outlier must not trip the gate; MAD "
                        "is the right defence")

    def test_sustained_burst_increments_counter_without_baseline_pollution(self):
        """5 consecutive 339 m bursts: each rejected, counter climbs to
        5, history stays uncontaminated so the gate keeps firing
        instead of slowly accepting the burst as a new baseline."""
        f, sp3, ranges, t_last = _build_filter_with_history()
        history_before = list(f._pr_median_history)
        for i in range(5):
            obs = [_make_obs(sv, "gps",
                             ranges[sv] + 339.0,
                             phi_if_m=ranges[sv] + 339.0)
                   for sv in ranges]
            f.update(obs, sp3, t_last + timedelta(seconds=i + 1),
                     clk_file=None)
        # 5 consecutive rejects accumulated.
        self.assertEqual(f._consecutive_catastrophic_rejects, 5)
        # History unchanged — sustained burst didn't pollute baseline.
        self.assertEqual(list(f._pr_median_history), history_before)

    def test_counter_resets_on_clean_epoch(self):
        """After a burst rejects then a clean epoch arrives, counter
        resets to 0 (not stuck at the prior rejection count)."""
        f, sp3, ranges, t_last = _build_filter_with_history()
        # One burst → counter = 1.
        burst_obs = [_make_obs(sv, "gps",
                               ranges[sv] + 339.0,
                               phi_if_m=ranges[sv] + 339.0)
                     for sv in ranges]
        f.update(burst_obs, sp3, t_last + timedelta(seconds=1), clk_file=None)
        self.assertEqual(f._consecutive_catastrophic_rejects, 1)
        # Clean epoch → counter resets.
        clean_obs = [_make_obs(sv, "gps",
                               ranges[sv] + 0.5,
                               phi_if_m=ranges[sv] + 0.001)
                     for sv in ranges]
        f.update(clean_obs, sp3, t_last + timedelta(seconds=2), clk_file=None)
        self.assertEqual(f._consecutive_catastrophic_rejects, 0)


if __name__ == "__main__":
    unittest.main()

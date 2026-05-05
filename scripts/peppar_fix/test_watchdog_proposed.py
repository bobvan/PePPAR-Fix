"""Tests for PositionWatchdogProposed (I-145915 PR/CP gate split).

The shadow gate runs alongside PositionWatchdog and exposes the
verdict-divergence signal that lets the operator validate the new
gate's behavior on real captures before flipping it active.
"""
from __future__ import annotations

import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.watchdog import (
    PositionWatchdog,
    PositionWatchdogProposed,
)


def _learn_baseline(wd, *, pr_rms, td_rms, n_used=12, epochs=30):
    """Drive the proposed watchdog through `epochs` baseline epochs."""
    for _ in range(epochs):
        wd.update(pr_rms, td_rms, n_used)


class BaselineLearningTest(unittest.TestCase):

    def test_no_trip_during_learn_in_window(self):
        wd = PositionWatchdogProposed()
        for _ in range(29):
            td_ok, pr_dist = wd.update(0.5, 0.05, 12)
            self.assertTrue(td_ok)
            self.assertFalse(pr_dist)
        self.assertFalse(wd.alarmed)

    def test_under_4_meas_returns_ok_no_count(self):
        wd = PositionWatchdogProposed()
        td_ok, pr_dist = wd.update(0.5, 1.0, 3)
        self.assertTrue(td_ok)
        self.assertFalse(pr_dist)


class TdGateTripTest(unittest.TestCase):

    def test_td_only_disturbance_trips(self):
        # Healthy PR but TD-CP rms blows up → proposed gate trips.
        wd = PositionWatchdogProposed(threshold_m=0.05, alarm_count=10)
        _learn_baseline(wd, pr_rms=0.5, td_rms=0.005)
        # PR stays healthy at baseline; TD jumps to 1.0 m sustained.
        for _ in range(15):
            td_ok, pr_dist = wd.update(0.5, 1.0, 12)
        self.assertTrue(wd.alarmed)
        self.assertFalse(pr_dist,
                         "PR is at baseline → no PR_DISTURBANCE expected")

    def test_pr_disturbance_does_not_trip_proposed(self):
        # PR explodes (multipath burst), TD stays healthy.  The whole
        # POINT of the proposed gate: this case does NOT trip anymore.
        wd = PositionWatchdogProposed(threshold_m=0.05, alarm_count=10,
                                      pr_threshold_m=0.5)
        _learn_baseline(wd, pr_rms=0.5, td_rms=0.005)
        # PR jumps 4x baseline; TD stays at baseline.
        for _ in range(15):
            td_ok, pr_dist = wd.update(2.5, 0.005, 12)
            self.assertTrue(td_ok,
                            "TD-CP fine → proposed gate must not trip")
            self.assertTrue(pr_dist,
                            "PR over baseline×3 → PR_DISTURBANCE flag")
        self.assertFalse(wd.alarmed)


class CompareToActiveGateTest(unittest.TestCase):
    """The behavioral difference between the active and proposed gates
    on the two characteristic scenarios.  These tests pin the redesign's
    motivating claim: the active gate trips on PR-domain noise that
    the proposed gate ignores."""

    def test_pr_only_burst_active_trips_proposed_does_not(self):
        active = PositionWatchdog(threshold_m=0.5, alarm_count=10)
        proposed = PositionWatchdogProposed(threshold_m=0.05,
                                            alarm_count=10,
                                            pr_threshold_m=0.5)
        # Learn-in: typical i226-class healthy ratios — PR ~0.5 m,
        # TD ~5 mm.  Combined RMS dominated by PR.
        for _ in range(30):
            n_pr, n_td = 8, 6
            pr_sq = n_pr * 0.5 ** 2
            td_sq = n_td * 0.005 ** 2
            combined_rms = ((pr_sq + td_sq) / (n_pr + n_td)) ** 0.5
            active.update(combined_rms, n_pr + n_td)
            proposed.update(0.5, 0.005, n_pr + n_td)
        # Burst: PR jumps 6× baseline → combined > active gate's
        # max(baseline×3, baseline+0.5).  TD-CP unchanged.  This is
        # the multipath/SSR-burst scenario the redesign targets.
        for _ in range(15):
            n_pr, n_td = 8, 6
            pr_sq = n_pr * 3.0 ** 2          # PR 0.5 → 3.0
            td_sq = n_td * 0.005 ** 2
            combined_rms = ((pr_sq + td_sq) / (n_pr + n_td)) ** 0.5
            active.update(combined_rms, n_pr + n_td)
            proposed.update(3.0, 0.005, n_pr + n_td)
        self.assertTrue(active.alarmed,
                        "active gate trips on combined-RMS bump")
        self.assertFalse(proposed.alarmed,
                         "proposed gate ignores PR-only disturbance")

    def test_td_blowup_both_trip(self):
        active = PositionWatchdog(threshold_m=0.5, alarm_count=10)
        proposed = PositionWatchdogProposed(threshold_m=0.05,
                                            alarm_count=10,
                                            pr_threshold_m=0.5)
        # Learn-in.
        for _ in range(30):
            n_pr, n_td = 8, 6
            pr_sq = n_pr * 0.5 ** 2
            td_sq = n_td * 0.005 ** 2
            combined_rms = ((pr_sq + td_sq) / (n_pr + n_td)) ** 0.5
            active.update(combined_rms, n_pr + n_td)
            proposed.update(0.5, 0.005, n_pr + n_td)
        # Filter mismatch — TD-CP residuals balloon to multi-meter
        # (real filter divergence: position+clock co-mismatched).
        # Both gates should now trip — PR baseline is 0.5m so combined
        # RMS at 5m × √(6/14) on TD rows alone is ~3.3 m, well over
        # the active gate's 1.5 m limit.
        for _ in range(15):
            n_pr, n_td = 8, 6
            pr_sq = n_pr * 0.5 ** 2
            td_sq = n_td * 5.0 ** 2          # TD 0.005 → 5.0 (filter blowup)
            combined_rms = ((pr_sq + td_sq) / (n_pr + n_td)) ** 0.5
            active.update(combined_rms, n_pr + n_td)
            proposed.update(0.5, 5.0, n_pr + n_td)
        self.assertTrue(active.alarmed)
        self.assertTrue(proposed.alarmed,
                        "TD-CP blowup → proposed gate trips correctly")


class ResetTest(unittest.TestCase):

    def test_reset_clears_alarm_and_baselines(self):
        wd = PositionWatchdogProposed(threshold_m=0.05, alarm_count=10)
        _learn_baseline(wd, pr_rms=0.5, td_rms=0.005)
        for _ in range(15):
            wd.update(0.5, 1.0, 12)
        self.assertTrue(wd.alarmed)
        wd.reset()
        self.assertFalse(wd.alarmed)
        # Post-reset, must re-learn baselines from scratch.
        td_ok, _ = wd.update(0.5, 1.0, 12)
        self.assertTrue(td_ok,
                        "fresh post-reset epoch is during learn-in, no trip")


if __name__ == "__main__":
    unittest.main()

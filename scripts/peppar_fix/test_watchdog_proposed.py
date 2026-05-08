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


class AutoClearTest(unittest.TestCase):
    """Auto-clear semantics added 2026-05-05.  Once a disturbance ends
    and TD-CP residuals fall back below threshold for `release_count`
    consecutive epochs, the alarm clears so the gate is ready to flag
    the next disturbance.  Without this the gate latched permanently
    after a brief reacquire transient on clkPoC3 even though residuals
    recovered to baseline within seconds."""

    def test_alarm_clears_after_release_count_clean_epochs(self):
        wd = PositionWatchdogProposed(threshold_m=0.05, alarm_count=10,
                                      release_count=30)
        _learn_baseline(wd, pr_rms=0.5, td_rms=0.005)
        # Trip on sustained TD-CP elevation.
        for _ in range(15):
            wd.update(0.5, 1.0, 12)
        self.assertTrue(wd.alarmed, "trips after 10 elevated epochs")
        # Residuals back to clean for less than release_count epochs:
        # alarm should NOT clear yet.
        for _ in range(29):
            td_ok, _ = wd.update(0.5, 0.005, 12)
        self.assertTrue(wd.alarmed, "still latched at release_count - 1")
        self.assertFalse(td_ok)
        # One more clean epoch hits release_count → alarm clears.
        td_ok, _ = wd.update(0.5, 0.005, 12)
        self.assertFalse(wd.alarmed, "auto-cleared at release_count")
        self.assertTrue(td_ok)

    def test_re_trip_after_auto_clear(self):
        # Verify the gate can detect a SECOND disturbance after the
        # first one cleared — exactly the scenario where the original
        # latch failed (proposed gate latched and missed any later
        # event).
        wd = PositionWatchdogProposed(threshold_m=0.05, alarm_count=10,
                                      release_count=30)
        _learn_baseline(wd, pr_rms=0.5, td_rms=0.005)
        # First trip + auto-clear.
        for _ in range(15):
            wd.update(0.5, 1.0, 12)
        self.assertTrue(wd.alarmed)
        for _ in range(30):
            wd.update(0.5, 0.005, 12)
        self.assertFalse(wd.alarmed)
        # Second elevation should trip again.
        for _ in range(15):
            wd.update(0.5, 1.0, 12)
        self.assertTrue(wd.alarmed, "second disturbance trips post-clear")

    def test_brief_below_threshold_does_not_clear(self):
        # If residuals dip below threshold for fewer than release_count
        # epochs and then go back over, the alarm stays latched and
        # the dip doesn't reset bad_count progress toward re-asserting.
        wd = PositionWatchdogProposed(threshold_m=0.05, alarm_count=10,
                                      release_count=30)
        _learn_baseline(wd, pr_rms=0.5, td_rms=0.005)
        for _ in range(15):
            wd.update(0.5, 1.0, 12)
        self.assertTrue(wd.alarmed)
        # Brief dip (5 clean epochs) then back to elevated:
        for _ in range(5):
            wd.update(0.5, 0.005, 12)
        self.assertTrue(wd.alarmed, "brief dip < release_count, still alarmed")
        for _ in range(5):
            wd.update(0.5, 1.0, 12)
        self.assertTrue(wd.alarmed, "back over threshold, still alarmed")

    def test_below_count_resets_when_residual_re_elevates(self):
        # Internal bookkeeping: if the dip-then-spike pattern doesn't
        # accumulate clean-epoch credit toward release, the gate must
        # require a fresh 30 clean epochs after the second spike.
        wd = PositionWatchdogProposed(threshold_m=0.05, alarm_count=10,
                                      release_count=30)
        _learn_baseline(wd, pr_rms=0.5, td_rms=0.005)
        for _ in range(15):
            wd.update(0.5, 1.0, 12)
        self.assertTrue(wd.alarmed)
        # Dip 20 (would have been close to release), then spike 1 ep,
        # then dip 29.  After dip-spike-dip sequence the second dip
        # alone (29 < 30) must NOT clear.
        for _ in range(20):
            wd.update(0.5, 0.005, 12)
        wd.update(0.5, 1.0, 12)            # one over-threshold epoch
        for _ in range(29):
            wd.update(0.5, 0.005, 12)
        self.assertTrue(wd.alarmed, "below_count was reset by spike")
        # One more clean epoch (now 30 since spike) → clear.
        wd.update(0.5, 0.005, 12)
        self.assertFalse(wd.alarmed)


class FloorTest(unittest.TestCase):
    """The floor bump (threshold_m default 0.05 → 0.50) lets the gate
    tolerate normal cycle-slip cleanup and reacquire transients on
    clean-baseline receivers.  The original 50 mm floor caused
    clkPoC3 (F10T, baseline ~3 mm) to false-trip on a 60 s reacquire."""

    def test_default_floor_is_500mm(self):
        wd = PositionWatchdogProposed()
        self.assertEqual(wd.threshold_m, 0.50)

    def test_clean_baseline_500mm_floor_passes_short_transient(self):
        # Simulate the clkPoC3 reacquire profile (TD-CP residual decays
        # from hundreds of metres back to ~0.2 m over ~10 epochs)
        # against the new 500 mm floor.  Without the bump the gate
        # tripped at 10 elevated epochs; with the bump it should not.
        wd = PositionWatchdogProposed()  # default threshold_m=0.50
        _learn_baseline(wd, pr_rms=5.0, td_rms=0.003)
        # Reacquire transient — residuals high but decaying.  Most are
        # well over 500 mm floor for the first ~6 epochs.
        for td in [620.0, 280.0, 90.0, 35.0, 16.0, 8.0, 4.5, 3.0, 2.0,
                   0.4, 0.21, 0.18, 0.15, 0.12, 0.08]:
            wd.update(5.0, td, 12)
        # Last several epochs at 0.4-0.08 m are below the 500 mm floor
        # → bad_count resets; only the first 9 over-threshold epochs
        # accumulated, fewer than alarm_count=10.  No trip.
        self.assertFalse(wd.alarmed,
                         "500 mm floor + decaying transient → no trip")


if __name__ == "__main__":
    unittest.main()

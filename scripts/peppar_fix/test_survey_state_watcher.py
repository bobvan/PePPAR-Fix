"""Tests for survey_state_watcher.

Pure-function tests use synthetic SurveyRefresh tuples + numpy ARP
arrays.  Loop tests use injected sleep/load callables so they don't
depend on real time or the filesystem.
"""
from __future__ import annotations

import unittest
from collections import deque
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from peppar_fix.survey_state_watcher import (
    DEFAULT_SLEW_THRESHOLD_M,
    RefreshAction,
    SurveyRefresh,
    decide_refresh_action,
    load_current_snapshot,
    watch_loop,
)


# Lab-relevant placeholder ECEF (40°N, 90°W, 200 m elevation — generic,
# not a real lab antenna).  Tests use offsets from this anchor.
ANCHOR = np.array([0.0, -4862789.5, 4100000.0])


def _refresh(ecef=None, sigma=0.012, mount_sn=0, mtime=1000.0):
    if ecef is None:
        ecef = tuple(ANCHOR)
    return SurveyRefresh(
        new_ecef=tuple(float(c) for c in ecef),
        new_sigma_m=sigma,
        new_mount_sn=mount_sn,
        file_mtime=mtime,
    )


# ── decide_refresh_action ─────────────────────────────────────────── #


class DecideTest(unittest.TestCase):

    def test_first_read_returns_none(self):
        curr = _refresh()
        action, delta = decide_refresh_action(None, curr, ANCHOR)
        self.assertIs(action, RefreshAction.NONE)
        self.assertEqual(delta, 0.0)

    def test_mtime_unchanged_returns_none(self):
        prev = _refresh(mtime=1000.0)
        curr = _refresh(mtime=1000.0, ecef=ANCHOR + np.array([5.0, 0, 0]))
        # Even with a huge delta, if mtime didn't advance, no action.
        action, delta = decide_refresh_action(prev, curr, ANCHOR)
        self.assertIs(action, RefreshAction.NONE)

    def test_mtime_went_backwards_returns_none(self):
        # File replaced with one whose mtime is OLDER (e.g., a restore).
        # We're conservative: not a real refresh.
        prev = _refresh(mtime=2000.0)
        curr = _refresh(mtime=1000.0)
        action, _ = decide_refresh_action(prev, curr, ANCHOR)
        self.assertIs(action, RefreshAction.NONE)

    def test_small_delta_returns_slew(self):
        # 5 cm delta, well below the 0.20 m default threshold.
        prev = _refresh(mtime=1000.0)
        curr = _refresh(mtime=2000.0,
                        ecef=ANCHOR + np.array([0.03, 0.04, 0.0]))
        action, delta = decide_refresh_action(prev, curr, ANCHOR)
        self.assertIs(action, RefreshAction.SLEW)
        self.assertAlmostEqual(delta, 0.05, places=4)

    def test_large_delta_returns_step(self):
        # 2 m delta, well above 0.20 m default threshold.
        prev = _refresh(mtime=1000.0)
        curr = _refresh(mtime=2000.0,
                        ecef=ANCHOR + np.array([2.0, 0.0, 0.0]))
        action, delta = decide_refresh_action(prev, curr, ANCHOR)
        self.assertIs(action, RefreshAction.STEP)
        self.assertAlmostEqual(delta, 2.0, places=4)

    def test_threshold_boundary_uses_strict_less_than(self):
        # Exactly at threshold → STEP.  Strict-less-than for the
        # slew side keeps the boundary clean.
        prev = _refresh(mtime=1000.0)
        curr = _refresh(mtime=2000.0,
                        ecef=ANCHOR + np.array([DEFAULT_SLEW_THRESHOLD_M,
                                                0, 0]))
        action, _ = decide_refresh_action(prev, curr, ANCHOR)
        self.assertIs(action, RefreshAction.STEP)

    def test_mount_sn_bump_forces_step_even_with_tiny_delta(self):
        # New survey has identical ECEF (within 1mm) but mount_sn bumped.
        # mount_sn semantics take precedence — operator action means
        # the engine's pin is suspect regardless of where the new
        # survey lands.
        prev = _refresh(mtime=1000.0, mount_sn=2)
        curr = _refresh(mtime=2000.0,
                        ecef=ANCHOR + np.array([0.001, 0, 0]),
                        mount_sn=3)
        action, delta = decide_refresh_action(prev, curr, ANCHOR)
        self.assertIs(action, RefreshAction.STEP)
        self.assertAlmostEqual(delta, 0.001, places=4)

    def test_custom_slew_threshold(self):
        # 0.5 m delta — would STEP at default 0.20 m, SLEW at 1.0 m.
        prev = _refresh(mtime=1000.0)
        curr = _refresh(mtime=2000.0,
                        ecef=ANCHOR + np.array([0.5, 0, 0]))
        action, _ = decide_refresh_action(prev, curr, ANCHOR,
                                          slew_threshold_m=1.0)
        self.assertIs(action, RefreshAction.SLEW)

    def test_current_arp_drifted_from_disk_state(self):
        # The engine has been blending from NAV2 since startup, so
        # its in-memory ARP is 10 cm ahead of the on-disk file (the
        # state file's value).  A fresh survey lands matching the
        # disk state (no on-disk change vs the engine's last-read).
        # The watcher's Δ should be 10 cm vs the engine's in-memory
        # ARP, not 0 vs the prior disk state.
        prev = _refresh(mtime=1000.0)
        curr = _refresh(mtime=2000.0, ecef=tuple(ANCHOR))
        engine_arp = ANCHOR + np.array([0.10, 0, 0])
        action, delta = decide_refresh_action(prev, curr, engine_arp)
        self.assertIs(action, RefreshAction.SLEW)
        self.assertAlmostEqual(delta, 0.10, places=4)


# ── load_current_snapshot — real filesystem, real TOML ────────────── #


class LoadSnapshotTest(unittest.TestCase):

    def test_missing_file_returns_none(self):
        with TemporaryDirectory() as td:
            p = Path(td) / "ghost.survey.toml"
            self.assertIsNone(load_current_snapshot(str(p)))

    def test_malformed_toml_returns_none(self):
        with TemporaryDirectory() as td:
            p = Path(td) / "bad.survey.toml"
            p.write_text("this = is not [ valid toml")
            self.assertIsNone(load_current_snapshot(str(p)))

    def test_missing_required_fields_returns_none(self):
        with TemporaryDirectory() as td:
            p = Path(td) / "incomplete.survey.toml"
            p.write_text('mount_sn = 0\nsigma_m = 0.01\n')  # no ecef_m
            self.assertIsNone(load_current_snapshot(str(p)))

    def test_well_formed_file_returns_snapshot(self):
        with TemporaryDirectory() as td:
            p = Path(td) / "good.survey.toml"
            p.write_text(
                'mount_sn = 2\n'
                'ecef_m = [40.0, -4862789.5, 4100000.0]\n'
                'sigma_m = 0.012\n'
                'updated = "2026-05-22T15:00:00Z"\n'
                'source = "peppar-survey --pride"\n'
                'frame = "ITRF2020@2026.39"\n'  # required since step 5
            )
            r = load_current_snapshot(str(p))
            self.assertIsNotNone(r)
            self.assertEqual(r.new_mount_sn, 2)
            self.assertEqual(r.new_ecef,
                             (40.0, -4862789.5, 4100000.0))
            self.assertAlmostEqual(r.new_sigma_m, 0.012)
            self.assertGreater(r.file_mtime, 0)


# ── watch_loop — injected sleep, load_fn, callbacks ───────────────── #


class _LoopHarness:
    """Helpers to drive watch_loop deterministically in tests."""

    def __init__(self, snapshots: list, max_iters: int = 20):
        # Snapshots queue: one per iteration.  None means "load
        # returned None" (file missing / malformed) for that
        # iteration.
        self.snapshots = deque(snapshots)
        self.slew_calls: list = []
        self.step_calls: list = []
        self.arp = ANCHOR.copy()
        self.max_iters = max_iters

    def load_fn(self, _path):
        if not self.snapshots:
            return None
        return self.snapshots.popleft()

    def arp_fn(self):
        return self.arp

    def on_slew(self, refresh, delta):
        self.slew_calls.append((refresh, delta))

    def on_step(self, refresh, delta):
        self.step_calls.append((refresh, delta))

    def sleep_fn(self, _t):
        pass  # never actually sleep in tests

    def run(self):
        watch_loop(
            survey_path="ignored",
            current_arp_ecef_fn=self.arp_fn,
            on_slew=self.on_slew,
            on_step=self.on_step,
            sleep_fn=self.sleep_fn,
            load_fn=self.load_fn,
            max_iterations=self.max_iters,
        )


class WatchLoopTest(unittest.TestCase):

    def test_first_read_fires_nothing(self):
        h = _LoopHarness([_refresh()], max_iters=1)
        h.run()
        self.assertEqual(h.slew_calls, [])
        self.assertEqual(h.step_calls, [])

    def test_small_delta_second_read_fires_slew(self):
        first = _refresh(mtime=1000.0)
        second = _refresh(mtime=2000.0,
                          ecef=ANCHOR + np.array([0.05, 0, 0]))
        h = _LoopHarness([first, second], max_iters=2)
        h.run()
        self.assertEqual(len(h.slew_calls), 1)
        self.assertEqual(h.step_calls, [])
        _, delta = h.slew_calls[0]
        self.assertAlmostEqual(delta, 0.05, places=4)

    def test_large_delta_second_read_fires_step(self):
        first = _refresh(mtime=1000.0)
        second = _refresh(mtime=2000.0,
                          ecef=ANCHOR + np.array([3.0, 0, 0]))
        h = _LoopHarness([first, second], max_iters=2)
        h.run()
        self.assertEqual(len(h.step_calls), 1)
        self.assertEqual(h.slew_calls, [])

    def test_mount_sn_bump_fires_step(self):
        first = _refresh(mtime=1000.0, mount_sn=1)
        second = _refresh(mtime=2000.0, mount_sn=2)
        h = _LoopHarness([first, second], max_iters=2)
        h.run()
        self.assertEqual(len(h.step_calls), 1)
        self.assertEqual(h.slew_calls, [])

    def test_mtime_stable_fires_nothing(self):
        first = _refresh(mtime=1000.0)
        # Same mtime but different ECEF — file content drift without
        # mtime advance, suspicious but per-spec ignored (peppar-survey
        # writes via atomic temp+rename so a real refresh always
        # advances mtime).
        second = _refresh(mtime=1000.0,
                          ecef=ANCHOR + np.array([2.0, 0, 0]))
        h = _LoopHarness([first, second], max_iters=2)
        h.run()
        self.assertEqual(h.slew_calls, [])
        self.assertEqual(h.step_calls, [])

    def test_loader_failure_silently_continues(self):
        # First poll: file missing.  Second poll: valid refresh.
        # Watcher should be tolerant of intermittent loader failures.
        first = None
        second = _refresh(mtime=1000.0)
        third = _refresh(mtime=2000.0,
                         ecef=ANCHOR + np.array([0.05, 0, 0]))
        h = _LoopHarness([first, second, third], max_iters=3)
        h.run()
        # First poll: None → no decision, no prev recorded.
        # Second poll: first valid read → prev=None decision → NO_CHANGE.
        # Third poll: valid read with small Δ → SLEW.
        self.assertEqual(len(h.slew_calls), 1)
        self.assertEqual(h.step_calls, [])

    def test_stop_fn_terminates_loop(self):
        # Drive 5 iterations and then stop.  No callbacks fire
        # because we keep returning the same snapshot.
        snapshots = [_refresh()] * 10
        h = _LoopHarness(snapshots, max_iters=100)
        call_count = [0]

        def stopping_arp():
            call_count[0] += 1
            return h.arp

        def stop_after_5():
            return call_count[0] >= 5

        watch_loop(
            survey_path="ignored",
            current_arp_ecef_fn=stopping_arp,
            on_slew=h.on_slew,
            on_step=h.on_step,
            sleep_fn=h.sleep_fn,
            stop_fn=stop_after_5,
            load_fn=h.load_fn,
            max_iterations=100,
        )
        # call_count is incremented inside the loop's action-decision
        # path; the test asserts the loop terminated before
        # max_iterations.
        self.assertLess(call_count[0], 100)

    def test_arp_drift_between_polls_changes_decision(self):
        # First read seeds prev.  Second read: file unchanged ecef,
        # but engine's ARP has drifted 30 cm in the meantime →
        # watcher should report Δ=30cm and SLEW.
        first = _refresh(mtime=1000.0)
        second = _refresh(mtime=2000.0)  # same ecef
        h = _LoopHarness([first, second], max_iters=2)
        # After first iter, harness ARP drifts.
        original_arp_fn = h.arp_fn
        call_n = [0]

        def drifting_arp():
            call_n[0] += 1
            if call_n[0] >= 2:
                return ANCHOR + np.array([0.10, 0, 0])
            return ANCHOR

        watch_loop(
            survey_path="ignored",
            current_arp_ecef_fn=drifting_arp,
            on_slew=h.on_slew,
            on_step=h.on_step,
            sleep_fn=h.sleep_fn,
            load_fn=h.load_fn,
            max_iterations=2,
        )
        self.assertEqual(len(h.slew_calls), 1)
        _, delta = h.slew_calls[0]
        self.assertAlmostEqual(delta, 0.10, places=4)


if __name__ == "__main__":
    unittest.main()

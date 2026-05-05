"""Tests for ARP state watcher (I-145846 prototype).

Two layers:
  - decide_transition() unit tests covering every (prev, curr) shape
    enumerated in docs/runtime-arp-transition.md.
  - watch_loop() integration tests with synthetic state-file rewrites
    and a mocked sleep, verifying callback dispatch + mtime-only
    short-circuit behavior.
"""
from __future__ import annotations

import json
import os
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.arp_state_watcher import (
    ArpSnapshot,
    Trigger,
    TransitionDecision,
    decide_transition,
    load_snapshot,
    watch_loop,
)


def _snap(*, ecef=(157470.222, -4756189.544, 4232767.952),
          sigma_m=0.10, mount_id=0, mtime=1000.0):
    return ArpSnapshot(ecef_m=tuple(ecef), sigma_m=sigma_m,
                       mount_id=mount_id, mtime=mtime)


# ── decide_transition unit tests ─────────────────────────────────── #


class DecideTransitionTest(unittest.TestCase):

    def test_initial_read_no_change(self):
        """First read with no prior snapshot — engine just launched
        with the file as-is; no decision."""
        d = decide_transition(None, _snap(), 0.10, 0.05)
        self.assertEqual(d.trigger, Trigger.NO_CHANGE)
        self.assertFalse(d.engine_should_restart)

    def test_mtime_unchanged_no_change(self):
        """If mtime didn't advance, the file content can't have
        changed — short-circuit to NO_CHANGE."""
        s = _snap(mtime=1000.0)
        d = decide_transition(s, s, 0.10, 0.05)
        self.assertEqual(d.trigger, Trigger.NO_CHANGE)
        self.assertFalse(d.engine_should_restart)

    def test_threshold_first_cross_fires(self):
        """σ_arp transitioned from 0.50 → 0.05; crossed 0.10 threshold."""
        prev = _snap(sigma_m=0.50, mtime=1000.0)
        curr = _snap(sigma_m=0.05, mtime=1100.0)
        d = decide_transition(prev, curr, 0.10, 0.05)
        self.assertEqual(d.trigger, Trigger.THRESHOLD_CROSS)
        self.assertTrue(d.engine_should_restart)
        self.assertIn("σ_arp", d.reason)

    def test_threshold_already_crossed_no_retrigger(self):
        """Once below threshold, further tightening doesn't re-fire.

        Per docs: 'first time only — subsequent tightening does not
        re-trigger'."""
        prev = _snap(sigma_m=0.05, mtime=1000.0)
        curr = _snap(sigma_m=0.03, mtime=1100.0)
        d = decide_transition(prev, curr, 0.10, 0.05)
        self.assertEqual(d.trigger, Trigger.NO_CHANGE)
        self.assertFalse(d.engine_should_restart)

    def test_threshold_uncross_no_trigger(self):
        """If σ_arp WORSENS back above threshold (rare but possible
        on watchdog inflation), THIS function does not trigger.  The
        orchestrator handles re-survey via mount_id increment, not
        via threshold-uncross."""
        prev = _snap(sigma_m=0.05, mtime=1000.0)
        curr = _snap(sigma_m=0.50, mtime=1100.0)
        d = decide_transition(prev, curr, 0.10, 0.05)
        self.assertEqual(d.trigger, Trigger.NO_CHANGE)
        self.assertFalse(d.engine_should_restart)

    def test_drift_above_epsilon_fires(self):
        """ECEF moved 80 mm — exceeds default 50 mm epsilon."""
        prev = _snap(ecef=(157470.222, -4756189.544, 4232767.952),
                     sigma_m=0.05, mtime=1000.0)
        # Move 80 mm in X
        curr = _snap(ecef=(157470.302, -4756189.544, 4232767.952),
                     sigma_m=0.05, mtime=1100.0)
        d = decide_transition(prev, curr, 0.10, 0.05)
        self.assertEqual(d.trigger, Trigger.DRIFT)
        self.assertTrue(d.engine_should_restart)
        self.assertIn("Δecef", d.reason)

    def test_drift_below_epsilon_no_trigger(self):
        """40 mm drift — under 50 mm epsilon, normal day-to-day variation."""
        prev = _snap(ecef=(157470.222, -4756189.544, 4232767.952),
                     sigma_m=0.05, mtime=1000.0)
        curr = _snap(ecef=(157470.262, -4756189.544, 4232767.952),
                     sigma_m=0.05, mtime=1100.0)
        d = decide_transition(prev, curr, 0.10, 0.05)
        self.assertEqual(d.trigger, Trigger.NO_CHANGE)
        self.assertFalse(d.engine_should_restart)

    def test_mount_move_fires(self):
        """mount_id incremented from 0 → 1; antenna moved."""
        prev = _snap(mount_id=0, mtime=1000.0)
        # Antenna moved a meter in X (typical mount-move scale)
        curr = _snap(ecef=(157471.222, -4756189.544, 4232767.952),
                     mount_id=1, mtime=1100.0)
        d = decide_transition(prev, curr, 0.10, 0.05)
        self.assertEqual(d.trigger, Trigger.MOUNT_MOVE)
        self.assertTrue(d.engine_should_restart)
        self.assertIn("mount_id 0 → 1", d.reason)

    def test_mount_move_priority_over_drift(self):
        """When mount_id increments AND drift is above epsilon,
        MOUNT_MOVE wins (more specific signal)."""
        prev = _snap(mount_id=0, mtime=1000.0)
        curr = _snap(ecef=(157471.222, -4756189.544, 4232767.952),
                     mount_id=1, mtime=1100.0)
        d = decide_transition(prev, curr, 0.10, 0.05)
        self.assertEqual(d.trigger, Trigger.MOUNT_MOVE)

    def test_drift_priority_over_threshold(self):
        """When drift fires AND σ_arp first-crosses, DRIFT wins
        (the new ECEF is the bigger correctness signal)."""
        prev = _snap(sigma_m=0.50, ecef=(157470.222, -4756189.544, 4232767.952),
                     mtime=1000.0)
        curr = _snap(sigma_m=0.05, ecef=(157470.302, -4756189.544, 4232767.952),
                     mtime=1100.0)
        d = decide_transition(prev, curr, 0.10, 0.05)
        self.assertEqual(d.trigger, Trigger.DRIFT)


# ── load_snapshot file-IO tests ─────────────────────────────────── #


class LoadSnapshotTest(unittest.TestCase):

    def setUp(self):
        import tempfile
        self.tmpdir = tempfile.mkdtemp()
        self.path = Path(self.tmpdir) / "state.json"

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmpdir)

    def _write(self, data: dict) -> None:
        with open(self.path, "w") as f:
            json.dump(data, f)

    def test_missing_file_returns_none(self):
        """Cold mount: file doesn't exist yet."""
        self.assertIsNone(load_snapshot(self.path))

    def test_full_schema_parses(self):
        self._write({
            "schema_version": 1,
            "receiver_uid": 136395244089,
            "antenna_id": "UFO1",
            "ecef_m": [157470.222, -4756189.544, 4232767.952],
            "sigma_m": 0.012,
            "mount_id": 0,
        })
        s = load_snapshot(self.path)
        self.assertIsNotNone(s)
        self.assertAlmostEqual(s.sigma_m, 0.012)
        self.assertEqual(s.mount_id, 0)
        self.assertEqual(s.antenna_id, "UFO1")
        self.assertEqual(s.ecef_m, (157470.222, -4756189.544, 4232767.952))

    def test_missing_required_fields_returns_none(self):
        """Schema mismatch — log warning, return None, don't crash."""
        self._write({"schema_version": 1})  # no ecef_m, sigma_m
        self.assertIsNone(load_snapshot(self.path))

    def test_malformed_json_returns_none(self):
        with open(self.path, "w") as f:
            f.write("{not valid json")
        self.assertIsNone(load_snapshot(self.path))

    def test_mount_id_defaults_to_zero(self):
        """Older state files without mount_id default to 0."""
        self._write({
            "schema_version": 1,
            "ecef_m": [1.0, 2.0, 3.0],
            "sigma_m": 0.05,
        })
        s = load_snapshot(self.path)
        self.assertEqual(s.mount_id, 0)


# ── watch_loop integration test ─────────────────────────────────── #


class WatchLoopTest(unittest.TestCase):

    def setUp(self):
        import tempfile
        self.tmpdir = tempfile.mkdtemp()
        self.path = Path(self.tmpdir) / "state.json"
        # Initial state: σ_arp = 0.5 m (above threshold; engine in re-survey)
        with open(self.path, "w") as f:
            json.dump({
                "schema_version": 1,
                "receiver_uid": 1,
                "antenna_id": "TEST",
                "ecef_m": [157470.222, -4756189.544, 4232767.952],
                "sigma_m": 0.5,
                "mount_id": 0,
            }, f)

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmpdir)

    def _rewrite(self, **overrides):
        """Atomic rewrite that bumps mtime."""
        import json as _json
        with open(self.path) as f:
            data = _json.load(f)
        data.update(overrides)
        tmp = self.path.with_suffix(".tmp")
        with open(tmp, "w") as f:
            _json.dump(data, f)
        os.rename(tmp, self.path)

    def test_threshold_cross_fires_callback(self):
        """Tighten σ_arp from 0.5 to 0.05 (across the 0.10 threshold);
        watch_loop should fire the callback once."""
        events: list[TransitionDecision] = []

        def cb(decision, prev, curr):
            events.append(decision)

        # Rewrite the file BEFORE watch_loop starts so the first
        # post-sleep mtime check sees the change.
        self._rewrite(sigma_m=0.05)

        # mtime will only advance by os.stat's clock granularity.
        # Force a new mtime by setting it explicitly.
        new_mtime = self.path.stat().st_mtime + 1.0
        os.utime(self.path, (new_mtime, new_mtime))

        # Sleep is mocked — the loop runs as fast as we let it.
        sleep_fn = MagicMock()

        # First load_snapshot is at top of watch_loop, before any sleep,
        # and reads sigma_m=0.05 (the rewritten value).  So prev=0.05.
        # We need to re-rewrite back to 0.5 first so prev sees 0.5,
        # then tick + see 0.05.
        # Easier: write 0.5, capture as prev manually, then start loop.
        self._rewrite(sigma_m=0.5)
        os.utime(self.path, (1000.0, 1000.0))
        # Now schedule a mtime advance + sigma=0.05 to happen on
        # iteration 1 by mocking sleep_fn to rewrite the file.
        def sleep_then_rewrite(_secs):
            self._rewrite(sigma_m=0.05)
            os.utime(self.path, (1100.0, 1100.0))
            sleep_fn.side_effect = None  # only fire once

        sleep_fn.side_effect = sleep_then_rewrite

        watch_loop(self.path, cb, pin_threshold_m=0.10,
                   drift_epsilon_m=0.05, poll_interval_s=1.0,
                   max_iterations=2, sleep_fn=sleep_fn)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].trigger, Trigger.THRESHOLD_CROSS)

    def test_no_change_no_callback(self):
        """File mtime doesn't advance → no callback."""
        events = []

        def cb(decision, prev, curr):
            events.append(decision)

        sleep_fn = MagicMock()
        watch_loop(self.path, cb, max_iterations=3, sleep_fn=sleep_fn,
                   poll_interval_s=1.0)
        self.assertEqual(len(events), 0)

    def test_mount_move_fires_callback(self):
        events = []

        def cb(decision, prev, curr):
            events.append(decision)

        # Stable initial state.
        os.utime(self.path, (1000.0, 1000.0))

        sleep_fn = MagicMock()
        def fire_mount_move(_secs):
            self._rewrite(mount_id=1,
                          ecef_m=[157471.222, -4756189.544, 4232767.952])
            os.utime(self.path, (1100.0, 1100.0))
            sleep_fn.side_effect = None
        sleep_fn.side_effect = fire_mount_move

        watch_loop(self.path, cb, max_iterations=2, sleep_fn=sleep_fn,
                   poll_interval_s=1.0)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].trigger, Trigger.MOUNT_MOVE)


if __name__ == "__main__":
    unittest.main()

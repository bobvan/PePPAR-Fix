"""Tests for split_ticc_log."""

from __future__ import annotations

import csv
import os
import tempfile
import unittest

import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from split_ticc_log import find_boundaries, segment_rows


def _row(ts: str, mono: float, ref_sec: int, ref_ps: int, ch: str) -> dict:
    return {
        'host_timestamp':  ts,
        'host_monotonic':  f"{mono:.9f}",
        'ref_sec':         str(ref_sec),
        'ref_ps':          str(ref_ps),
        'channel':         ch,
    }


class _CleanLog(unittest.TestCase):

    def test_no_boundaries_no_jumps(self):
        rows = [
            _row("2026-05-12T20:00:00+00:00", 100.0, 1000, 0, 'chA'),
            _row("2026-05-12T20:00:00+00:00", 100.1, 1000, 0, 'chB'),
            _row("2026-05-12T20:00:01+00:00", 101.0, 1001, 0, 'chA'),
            _row("2026-05-12T20:00:01+00:00", 101.1, 1001, 0, 'chB'),
            _row("2026-05-12T20:00:02+00:00", 102.0, 1002, 0, 'chA'),
        ]
        bs = find_boundaries(rows)
        self.assertEqual(bs, [])
        segs = segment_rows(rows, bs)
        self.assertEqual(len(segs['chA']), 1)
        self.assertEqual(segs['chA'][0].n_samples, 3)
        self.assertEqual(len(segs['chB']), 1)
        self.assertEqual(segs['chB'][0].n_samples, 2)


class _TiccReboot(unittest.TestCase):
    """ref_sec stepping backward = Arduino DTR-reset → new segment."""

    def test_ref_sec_backward_step_is_boundary(self):
        rows = [
            _row("2026-05-12T20:00:00+00:00", 100.0, 5000, 0, 'chA'),
            _row("2026-05-12T20:00:01+00:00", 101.0, 5001, 0, 'chA'),
            # TICC rebooted — counter back to small
            _row("2026-05-12T20:00:20+00:00", 120.0, 10,   0, 'chA'),
            _row("2026-05-12T20:00:21+00:00", 121.0, 11,   0, 'chA'),
        ]
        bs = find_boundaries(rows, host_gap_s=60.0)  # avoid host-gap firing too
        # ref_sec_drop AND host_gap likely both fire here.  Filter to ref_sec_drop.
        ref_drops = [b for b in bs if b.kind == 'ref_sec_drop']
        self.assertEqual(len(ref_drops), 1)
        self.assertEqual(ref_drops[0].idx, 2)

    def test_per_channel_ref_sec_independent(self):
        # chA continues monotonically while chB has a reboot.
        rows = [
            _row("2026-05-12T20:00:00+00:00", 100.0, 5000, 0, 'chA'),
            _row("2026-05-12T20:00:00+00:00", 100.0, 5000, 0, 'chB'),
            _row("2026-05-12T20:00:01+00:00", 101.0, 5001, 0, 'chA'),
            _row("2026-05-12T20:00:01+00:00", 101.0, 10,   0, 'chB'),  # chB reboot
            _row("2026-05-12T20:00:02+00:00", 102.0, 5002, 0, 'chA'),
        ]
        bs = find_boundaries(rows)
        ref_drops = [b for b in bs if b.kind == 'ref_sec_drop']
        self.assertEqual(len(ref_drops), 1)
        self.assertIn('chB', ref_drops[0].detail)


class _EngineGap(unittest.TestCase):
    """Large host_timestamp jump = engine restart → new segment."""

    def test_host_gap_is_boundary(self):
        rows = [
            _row("2026-05-12T20:00:00+00:00", 100.0, 1000, 0, 'chA'),
            _row("2026-05-12T20:00:01+00:00", 101.0, 1001, 0, 'chA'),
            # 30-second host_timestamp gap (engine downtime + relaunch)
            _row("2026-05-12T20:00:31+00:00", 131.0, 1031, 0, 'chA'),
            _row("2026-05-12T20:00:32+00:00", 132.0, 1032, 0, 'chA'),
        ]
        bs = find_boundaries(rows, host_gap_s=10.0)
        host_gaps = [b for b in bs if b.kind == 'host_gap']
        self.assertEqual(len(host_gaps), 1)
        self.assertEqual(host_gaps[0].idx, 2)

    def test_small_gap_not_boundary(self):
        # 1-sec gap should NOT trigger with default 10s threshold.
        rows = [
            _row("2026-05-12T20:00:00+00:00", 100.0, 1000, 0, 'chA'),
            _row("2026-05-12T20:00:01+00:00", 101.0, 1001, 0, 'chA'),
            _row("2026-05-12T20:00:02+00:00", 102.0, 1002, 0, 'chA'),
        ]
        bs = find_boundaries(rows, host_gap_s=10.0)
        self.assertEqual([b for b in bs if b.kind == 'host_gap'], [])


class _SegmentBuild(unittest.TestCase):

    def test_segments_emit_correct_n_samples(self):
        rows = [
            _row("2026-05-12T20:00:00+00:00", 100.0, 1000, 0, 'chA'),
            _row("2026-05-12T20:00:01+00:00", 101.0, 1001, 0, 'chA'),
            _row("2026-05-12T20:00:50+00:00", 150.0, 1050, 0, 'chA'),  # engine restart
            _row("2026-05-12T20:00:51+00:00", 151.0, 1051, 0, 'chA'),
            _row("2026-05-12T20:00:52+00:00", 152.0, 1052, 0, 'chA'),
        ]
        bs = find_boundaries(rows)
        segs = segment_rows(rows, bs)
        self.assertEqual(len(segs['chA']), 2)
        self.assertEqual(segs['chA'][0].n_samples, 2)
        self.assertEqual(segs['chA'][1].n_samples, 3)
        # Span: first seg is 1 sec, second is 2 sec
        self.assertAlmostEqual(segs['chA'][0].span_s, 1.0, places=1)
        self.assertAlmostEqual(segs['chA'][1].span_s, 2.0, places=1)


if __name__ == "__main__":
    unittest.main()

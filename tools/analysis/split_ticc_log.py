#!/usr/bin/env python3
"""Split a TICC log into per-instance segments.

PePPAR-Fix's TICC CSV logs are opened in append mode (recoveryRetry-main)
so they survive wrapper-driven engine relaunches.  That means a single
log file can hold data from multiple consecutive engine instances —
and possibly multiple TICC sessions, since opening the TICC's USB
serial port may DTR-reset the Arduino, restarting its ref_sec counter
from zero.

When analyzing such a log (TDEV, phase residual, drift slopes), each
SEGMENT must be treated independently — straddling a boundary creates
nonsense numbers (e.g., a `ref_sec` jump backward looks like a -gigayears
slope).

This tool identifies segment boundaries via two signals:
  1. ``ref_sec`` decrease     (TICC re-booted; counter reset)
  2. ``host_timestamp`` gap   (engine restart with port held)

Output: per-channel list of (start_idx, end_idx, n_samples, span_s)
tuples plus optional CSV writeback split into separate files.

Usage::

    python3 tools/analysis/split_ticc_log.py path/to/ticc.csv
    python3 tools/analysis/split_ticc_log.py --write-segments \
        path/to/ticc.csv --out-dir /tmp/ticc-segs/

A segment must contain at least ``--min-samples`` rows on the channel
of interest to be reported.  Default 5.

The boundary detector reports two boundaries per kind so you can tune
``--host-gap-s`` (default 10 s) and ``--ref-sec-drop`` (default 0 — any
backward step) on a per-host basis.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from dataclasses import dataclass
from datetime import datetime
from typing import Iterator


@dataclass
class Boundary:
    idx:        int      # row index just AFTER the boundary
    kind:       str      # 'ref_sec_drop' | 'host_gap'
    detail:     str      # human-readable context


@dataclass
class Segment:
    start_idx:   int
    end_idx:     int     # exclusive
    channel:     str
    n_samples:   int
    first_ts:    datetime | None
    last_ts:     datetime | None

    @property
    def span_s(self) -> float:
        if self.first_ts is None or self.last_ts is None:
            return 0.0
        return (self.last_ts - self.first_ts).total_seconds()


def _read_rows(path: str) -> list[dict]:
    out = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            out.append(r)
    return out


def _ts(row: dict) -> datetime | None:
    s = row.get('host_timestamp')
    if not s:
        return None
    try:
        return datetime.fromisoformat(s)
    except ValueError:
        return None


def find_boundaries(rows: list[dict],
                    *,
                    host_gap_s: float = 10.0,
                    ref_sec_drop: int = 0) -> list[Boundary]:
    """Walk per-channel ref_sec and host_timestamp; report boundaries.

    A boundary is set at row index `i` when row[i] starts a NEW
    segment vs row[i-1] on the SAME channel.  Cross-channel jumps
    (chA after chB) are ignored — events from both channels interleave
    in the file but each channel's stream is independent.
    """
    boundaries: list[Boundary] = []
    last_by_ch: dict[str, dict] = {}  # channel → previous row dict

    for i, r in enumerate(rows):
        ch = r.get('channel') or ''
        prev = last_by_ch.get(ch)
        last_by_ch[ch] = r
        if prev is None:
            continue

        # ref_sec decreasing on the same channel → TICC reboot
        try:
            ref_curr = int(r['ref_sec'])
            ref_prev = int(prev['ref_sec'])
            if ref_curr - ref_prev < -ref_sec_drop:
                boundaries.append(Boundary(
                    idx=i, kind='ref_sec_drop',
                    detail=f"{ch} ref_sec {ref_prev} → {ref_curr}"))
                continue
        except (KeyError, ValueError):
            pass

        # host_timestamp gap → engine downtime
        ts_curr, ts_prev = _ts(r), _ts(prev)
        if ts_curr is not None and ts_prev is not None:
            gap = (ts_curr - ts_prev).total_seconds()
            if gap > host_gap_s:
                boundaries.append(Boundary(
                    idx=i, kind='host_gap',
                    detail=f"{ch} {gap:.1f}s gap @ {ts_curr.isoformat()}"))

    return boundaries


def segment_rows(rows: list[dict],
                 boundaries: list[Boundary]) -> dict[str, list[Segment]]:
    """Group rows into per-(channel, segment) ranges."""
    # Global boundary indices (we'll split each channel at these too).
    bidx = sorted({b.idx for b in boundaries})
    # Sentinel for the final segment.
    bidx.append(len(rows))

    out: dict[str, list[Segment]] = {}
    prev_idx = 0
    for end in bidx:
        # Each (prev_idx..end) range is a segment.  Walk and bucket
        # by channel, since a single segment can hold both chA and chB.
        ch_rows: dict[str, list[int]] = {}
        for j in range(prev_idx, end):
            r = rows[j]
            ch = r.get('channel') or ''
            ch_rows.setdefault(ch, []).append(j)
        for ch, idxs in ch_rows.items():
            first_ts = _ts(rows[idxs[0]])
            last_ts = _ts(rows[idxs[-1]])
            out.setdefault(ch, []).append(Segment(
                start_idx=idxs[0],
                end_idx=idxs[-1] + 1,
                channel=ch,
                n_samples=len(idxs),
                first_ts=first_ts,
                last_ts=last_ts,
            ))
        prev_idx = end
    return out


def write_segment(rows: list[dict], seg: Segment, out_path: str) -> None:
    fields = list(rows[0].keys())
    with open(out_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for j in range(seg.start_idx, seg.end_idx):
            r = rows[j]
            if r.get('channel') == seg.channel:
                w.writerow(r)


def render_summary(path: str,
                   rows: list[dict],
                   boundaries: list[Boundary],
                   segments: dict[str, list[Segment]],
                   min_samples: int) -> None:
    print(f"\n{'='*78}")
    print(f"File: {path}")
    print(f"Total rows: {len(rows)}")
    print(f"Boundaries found: {len(boundaries)}")
    if boundaries:
        print("\nBoundaries:")
        for b in boundaries:
            print(f"  row {b.idx:>7d}  {b.kind:<14s}  {b.detail}")

    for ch in sorted(segments):
        segs = [s for s in segments[ch] if s.n_samples >= min_samples]
        print(f"\nChannel {ch!r}: {len(segs)} segments "
              f"(threshold {min_samples} samples)")
        print(f"  {'idx range':>17s}  {'n':>6s}  {'span':>8s}  "
              f"{'first_ts':<26s}  last_ts")
        for s in segs:
            ft = s.first_ts.isoformat() if s.first_ts else '?'
            lt = s.last_ts.isoformat() if s.last_ts else '?'
            span = f"{s.span_s/60:.1f}min"
            print(f"  {s.start_idx:>7d}..{s.end_idx:<7d}  "
                  f"{s.n_samples:>6d}  {span:>8s}  {ft:<26s}  {lt}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", help="TICC csv log")
    ap.add_argument("--host-gap-s", type=float, default=10.0,
                    help="host_timestamp gap (s) flagging an engine "
                         "downtime / restart (default 10)")
    ap.add_argument("--ref-sec-drop", type=int, default=0,
                    help="ref_sec backward step beyond this is a TICC "
                         "reboot boundary.  Default 0 (any backward "
                         "step triggers).")
    ap.add_argument("--min-samples", type=int, default=5,
                    help="Skip segments with fewer than this many "
                         "samples on a channel (default 5)")
    ap.add_argument("--write-segments", action="store_true",
                    help="Write each segment to a separate CSV "
                         "(channel + start_idx in filename)")
    ap.add_argument("--out-dir", default=None,
                    help="Directory for split CSVs.  Default: alongside "
                         "the input")
    args = ap.parse_args()

    rows = _read_rows(args.path)
    boundaries = find_boundaries(
        rows,
        host_gap_s=args.host_gap_s,
        ref_sec_drop=args.ref_sec_drop)
    segments = segment_rows(rows, boundaries)
    render_summary(args.path, rows, boundaries, segments, args.min_samples)

    if args.write_segments:
        out_dir = args.out_dir or os.path.dirname(os.path.abspath(args.path))
        base = os.path.splitext(os.path.basename(args.path))[0]
        os.makedirs(out_dir, exist_ok=True)
        n_written = 0
        for ch in sorted(segments):
            segs = [s for s in segments[ch] if s.n_samples >= args.min_samples]
            for k, s in enumerate(segs):
                fname = f"{base}.{ch}.seg{k:02d}.csv"
                out_path = os.path.join(out_dir, fname)
                write_segment(rows, s, out_path)
                n_written += 1
        print(f"\nWrote {n_written} segment files to {out_dir}")


if __name__ == "__main__":
    main()

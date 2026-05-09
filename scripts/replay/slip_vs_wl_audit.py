#!/usr/bin/env python3
"""Audit cycle-slip detector vs WL-tracker re-fix outcomes.

I-155354 follow-up to wl-integer-vs-pride-comparison.md.  The bias
diagnosis flagged the engine fixing WL to wrong integers; this
script asks WHY the tracker is re-fixing.

For every slip event in an engine log:
  - by reason combination (mw_jump-only vs corroborated)
  - by confidence (HIGH vs LOW)
For every WL_FIX_LIFE entry that follows a slip on the same SV:
  - integer at re-fix vs integer at last fix on the same SV
  - was the slip carrier-phase-confirmed (gf_jump) or MW-only?

The hypothesis: mw_jump-only slips are PR-noise false positives.
They flush the MW tracker; the post-flush re-fix uses fresh MW
samples that are still PR-noise-contaminated, landing on a wrong
integer.  Multiple consecutive false positives drift the integer
further.
"""
from __future__ import annotations

import argparse
import re
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path


_SLIP_RE = re.compile(
    r'^(?P<date>\d{4}-\d{2}-\d{2})\s+'
    r'(?P<time>\d{2}:\d{2}:\d{2}),(?P<ms>\d+)\s+\S+\s+'
    r'slip:\s+sv=(?P<sv>[A-Z]\d{2,3})\s+'
    r'reasons=(?P<reasons>\S+)\s+'
    r'conf=(?P<conf>\w+)\s+'
    r'lock=(?P<lock>[-\d.]+)ms\s+'
    r'cno=(?P<cno>[-\d.]+)\s+'
    r'elev=(?P<elev>\S+)\s+'
    r'az=(?P<az>\S+)\s+'
    r'ipp_sza=(?P<ipp>\S+)\s+'
    r'gap=(?P<gap>\S+)\s+'
    r'gf=(?P<gf>\S+)\s+'
    r'mw=(?P<mw>\S+)'
)

_FIX_RE = re.compile(
    r'^(?P<date>\d{4}-\d{2}-\d{2})\s+'
    r'(?P<time>\d{2}:\d{2}:\d{2}),(?P<ms>\d+)\s+\S+\s+'
    r'\[WL_FIX_LIFE\]\s+event=(?P<event>\w+)\s+'
    r'sv=(?P<sv>[A-Z]\d{2,3})\s+'
    r'n_wl=(?P<n_wl>-?\d+|None)'
)


@dataclass
class SlipEvent:
    ts: datetime
    sv: str
    reasons: tuple[str, ...]
    conf: str

    @property
    def reasons_key(self) -> str:
        return ",".join(sorted(self.reasons))

    @property
    def carrier_confirmed(self) -> bool:
        """True iff the slip has carrier-phase corroboration —
        i.e., 'gf_jump' or 'ubx_locktime_drop' is in the reason set.
        mw_jump-alone is PR-side; arc_gap is genuine; both confirmed."""
        return ('gf_jump' in self.reasons or
                'ubx_locktime_drop' in self.reasons or
                'arc_gap' in self.reasons)


@dataclass
class FixEvent:
    ts: datetime
    sv: str
    event: str
    n_wl: int | None


def _parse_ts(date: str, time: str) -> datetime:
    return datetime.fromisoformat(f"{date}T{time}+00:00")


def parse(path: Path) -> tuple[list[SlipEvent], list[FixEvent]]:
    slips: list[SlipEvent] = []
    fixes: list[FixEvent] = []
    with open(path) as f:
        for line in f:
            if 'slip: sv=' in line:
                m = _SLIP_RE.search(line)
                if m:
                    slips.append(SlipEvent(
                        ts=_parse_ts(m.group('date'), m.group('time')),
                        sv=m.group('sv'),
                        reasons=tuple(m.group('reasons').split(',')),
                        conf=m.group('conf'),
                    ))
            elif '[WL_FIX_LIFE]' in line:
                m = _FIX_RE.search(line)
                if m:
                    n_wl_str = m.group('n_wl')
                    fixes.append(FixEvent(
                        ts=_parse_ts(m.group('date'), m.group('time')),
                        sv=m.group('sv'),
                        event=m.group('event'),
                        n_wl=None if n_wl_str == 'None' else int(n_wl_str),
                    ))
    return slips, fixes


def reason_breakdown(slips: list[SlipEvent]) -> None:
    print("=== Slip events by reason set ===")
    counter = Counter(s.reasons_key for s in slips)
    conf_by_reason: dict[str, Counter] = defaultdict(Counter)
    for s in slips:
        conf_by_reason[s.reasons_key][s.conf] += 1
    print(f"{'reason':<40}  {'count':>6}  {'conf=HIGH':>9}  {'conf=LOW':>8}  carrier?")
    print('-' * 80)
    for reasons, count in sorted(counter.items(), key=lambda x: -x[1]):
        cb = conf_by_reason[reasons]
        carrier = (
            'gf_jump' in reasons.split(',') or
            'ubx_locktime_drop' in reasons.split(',') or
            'arc_gap' in reasons.split(',')
        )
        print(f"{reasons:<40}  {count:>6}  {cb.get('HIGH', 0):>9}  "
              f"{cb.get('LOW', 0):>8}  {'YES' if carrier else 'no'}")


def post_slip_refix(slips: list[SlipEvent],
                     fixes: list[FixEvent],
                     window_s: float = 300.0) -> None:
    """For each slip, find the next WL re-fix on the same SV within
    `window_s`, and report whether the integer matches the prior fix
    (slip → same integer = noise filter; slip → different integer =
    tracker re-anchored, possibly wrongly)."""
    fixes_by_sv: dict[str, list[FixEvent]] = defaultdict(list)
    for f in fixes:
        fixes_by_sv[f.sv].append(f)
    # Stats: per (carrier_confirmed, refix outcome).
    counts = Counter()
    cycle_diffs: dict[str, list[int]] = defaultdict(list)
    last_fix_n: dict[str, int] = {}
    for slip in slips:
        # Find the LAST 'enter' WL fix on this SV before the slip.
        prior = None
        for f in fixes_by_sv.get(slip.sv, []):
            if f.ts >= slip.ts:
                break
            if f.event == 'enter' and f.n_wl is not None:
                prior = f
        if prior is None:
            continue
        # Next 'enter' fix after the slip, within window.
        post = None
        for f in fixes_by_sv.get(slip.sv, []):
            if f.ts <= slip.ts:
                continue
            if (f.ts - slip.ts).total_seconds() > window_s:
                break
            if f.event == 'enter' and f.n_wl is not None:
                post = f
                break
        carrier = 'CARRIER' if slip.carrier_confirmed else 'mw-only '
        if post is None:
            counts[(carrier, 'no_refix')] += 1
            continue
        delta = post.n_wl - prior.n_wl
        if delta == 0:
            outcome = 'same_int'
        else:
            outcome = 'DIFF_int'
        counts[(carrier, outcome)] += 1
        if outcome == 'DIFF_int':
            cycle_diffs[carrier].append(delta)
    print()
    print(f"=== Post-slip WL re-fix outcomes (window {window_s:.0f}s) ===")
    print(f"{'category':<10}  {'no_refix':>9}  {'same_int':>9}  {'DIFF_int':>9}  notes")
    print('-' * 65)
    for cat in ('CARRIER', 'mw-only '):
        n_no = counts.get((cat, 'no_refix'), 0)
        n_same = counts.get((cat, 'same_int'), 0)
        n_diff = counts.get((cat, 'DIFF_int'), 0)
        notes = ''
        if n_diff and cycle_diffs.get(cat):
            ds = cycle_diffs[cat]
            ds_abs = sorted(abs(d) for d in ds)
            median = ds_abs[len(ds_abs) // 2]
            mx = max(ds_abs)
            notes = f"|Δcyc| median={median}  max={mx}"
        print(f"{cat:<10}  {n_no:>9}  {n_same:>9}  {n_diff:>9}  {notes}")


def per_sv_drift(slips: list[SlipEvent],
                  fixes: list[FixEvent],
                  top_n: int = 10) -> None:
    """Per-SV summary of mw-only-slip rate and integer-drift extent."""
    sv_slip_counts: dict[str, dict[str, int]] = defaultdict(
        lambda: {'carrier': 0, 'mw_only': 0})
    for s in slips:
        bucket = 'carrier' if s.carrier_confirmed else 'mw_only'
        sv_slip_counts[s.sv][bucket] += 1
    enters_by_sv: dict[str, list[int]] = defaultdict(list)
    for f in fixes:
        if f.event == 'enter' and f.n_wl is not None:
            enters_by_sv[f.sv].append(f.n_wl)
    print()
    print(f"=== Per-SV mw-only slip rate + integer drift "
          f"(top {top_n} by mw-only count) ===")
    print(f"{'sv':<5}  {'carrier_slips':>13}  {'mw_only_slips':>13}  "
          f"{'fixes':>5}  {'min_n_wl':>8}  {'max_n_wl':>8}  {'range':>6}")
    print('-' * 78)
    rows = []
    for sv, sc in sv_slip_counts.items():
        ents = enters_by_sv.get(sv, [])
        if not ents:
            continue
        rows.append((sv, sc['carrier'], sc['mw_only'], len(ents),
                     min(ents), max(ents), max(ents) - min(ents)))
    rows.sort(key=lambda x: -x[2])
    for sv, n_car, n_mw, n_fix, mn, mx, rng in rows[:top_n]:
        print(f"{sv:<5}  {n_car:>13}  {n_mw:>13}  {n_fix:>5}  "
              f"{mn:>+8d}  {mx:>+8d}  {rng:>6d}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--engine-log', required=True, type=Path)
    ap.add_argument('--window-s', type=float, default=300.0,
                    help='post-slip refix-search window in seconds')
    ap.add_argument('--top-n', type=int, default=10)
    args = ap.parse_args()

    slips, fixes = parse(args.engine_log)
    print(f"Parsed {len(slips)} slip events, {len(fixes)} WL_FIX_LIFE events")
    reason_breakdown(slips)
    post_slip_refix(slips, fixes, window_s=args.window_s)
    per_sv_drift(slips, fixes, top_n=args.top_n)
    return 0


if __name__ == '__main__':
    sys.exit(main())

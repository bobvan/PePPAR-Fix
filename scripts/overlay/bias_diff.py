#!/usr/bin/env python3
"""Cross-host SSR bias-application diff from engine [CB_APPLIED] /
[PB_APPLIED] logs.

When two hosts on a shared antenna run with the same SSR streams,
every (SV, signal) tracked by both should have IDENTICAL bias values
applied.  Mismatches expose dual-mount routing bugs (e.g., one host
applies CNES bias while the other applies WHU bias for the same
SV+signal because the routing-logic merge failed).

Per Main's P1 logging (commit 87e5da3, 2026-05-02): each [CB_APPLIED]
/ [PB_APPLIED] line now carries an optional ``src=<mount>`` field
identifying which NTRIP mount delivered the applied bias.

Per Bravo's review of I-122350 Q1: emit was specced as delta-on-
change with 60-epoch heartbeat, so steady-state values appear at
predictable cadence even when unchanged.

USAGE:
    bias_diff.py LOG_A LOG_B [--label A B] [--detail | --summary] [--top N]

The "summary" mode (default) prints a per-(SV, signal, kind) table
with epoch counts: A_only / B_only / match / value_mismatch /
src_mismatch.  "--detail" additionally prints the first N mismatch
lines per (SV, signal) for forensic context.

EXIT STATUS:
    0 if no value mismatches found across hosts
    1 if any value mismatch (suspect dual-mount routing bug)

The src_mismatch count is informational only (same value, different
src tag) and does NOT affect exit status.
"""
from __future__ import annotations

import argparse
import re
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path


# Engine log line format (from realtime_ppp.py, post 87e5da3):
#   [CB_APPLIED] G18 f1=GPS-L1CA→C1C val=+1.234m src=SSR
#                    f2=GPS-L5Q →C5Q val=-0.567m src=SSR-BIAS
# The arrow → may have variable surrounding whitespace.  Capture
# both bands per epoch.
_BIAS_RE = re.compile(
    r"(?P<date>\d{4}-\d{2}-\d{2})\s+(?P<time>\d{2}:\d{2}:\d{2})[,.]?\d*\s+\S+\s+"
    r"\[(?P<kind>CB_APPLIED|PB_APPLIED)\]\s+"
    r"(?P<sv>[A-Z]\d{2,3})\s+"
    r"f1=(?P<f1_sig>\S+?)\s*→\s*(?P<f1_rinex>\S+)\s+"
    r"val=(?P<f1_val>MISS|[-+]?[\d.]+)m"
    r"(?:\s+src=(?P<f1_src>\S+))?\s+"
    r"f2=(?P<f2_sig>\S+?)\s*→\s*(?P<f2_rinex>\S+)\s+"
    r"val=(?P<f2_val>MISS|[-+]?[\d.]+)m"
    r"(?:\s+src=(?P<f2_src>\S+))?"
)


def parse_log(path: str) -> list[dict]:
    """Return [{ts, sv, kind, sig_label, val, src}, ...] one entry per
    band (so each [CB_APPLIED]/[PB_APPLIED] line yields 2 entries).
    val is float or None for MISS; src is str or None when absent."""
    out = []
    with open(path) as f:
        for line in f:
            m = _BIAS_RE.search(line)
            if not m:
                continue
            ts = datetime.fromisoformat(
                f"{m.group('date')}T{m.group('time')}").timestamp()
            sv = m.group('sv')
            kind = m.group('kind')
            for band in ('f1', 'f2'):
                sig = m.group(f'{band}_sig')
                rin = m.group(f'{band}_rinex')
                val_str = m.group(f'{band}_val')
                src = m.group(f'{band}_src')
                val = None if val_str == 'MISS' else float(val_str)
                out.append({
                    'ts': ts, 'sv': sv, 'kind': kind,
                    'sig_label': sig,        # e.g., GPS-L1CA
                    'rinex': rin,            # e.g., C1C
                    'val': val, 'src': src,
                })
    return out


def index_by_key(events: list[dict]) -> dict:
    """Group events by (sv, sig_label, kind) → list of (ts, val, src)
    tuples sorted by ts."""
    idx = defaultdict(list)
    for e in events:
        key = (e['sv'], e['sig_label'], e['kind'])
        idx[key].append((e['ts'], e['val'], e['src']))
    for key in idx:
        idx[key].sort(key=lambda x: x[0])
    return idx


def diff_two_hosts(
    a_idx: dict, b_idx: dict,
    label_a: str, label_b: str,
    match_window_s: float = 5.0,
) -> dict:
    """For each (sv, sig_label, kind) key present in either host,
    compute counts: a_only / b_only / match / value_mismatch /
    src_mismatch.  Each "host event" within match_window_s of a
    contemporaneous host event on the other side is paired; a
    value match means same numeric value (within 1mm) AND both
    non-MISS, OR both MISS."""
    all_keys = set(a_idx) | set(b_idx)
    summary = {}
    for key in sorted(all_keys):
        a_evs = a_idx.get(key, [])
        b_evs = b_idx.get(key, [])
        a_only = b_only = match = value_mismatch = src_mismatch = 0
        mismatches = []
        # Two-pointer walk over time-sorted lists.
        i = j = 0
        while i < len(a_evs) and j < len(b_evs):
            ta, va, sa = a_evs[i]
            tb, vb, sb = b_evs[j]
            dt = tb - ta
            if dt < -match_window_s:
                # B event before A's window: B is unmatched.
                b_only += 1
                j += 1
            elif dt > match_window_s:
                # A event before B's window: A is unmatched.
                a_only += 1
                i += 1
            else:
                # Within window: pair.
                if (va is None) != (vb is None):
                    value_mismatch += 1
                    mismatches.append((ta, va, sa, tb, vb, sb))
                elif va is None and vb is None:
                    match += 1  # both MISS
                elif abs(va - vb) > 0.001:
                    value_mismatch += 1
                    mismatches.append((ta, va, sa, tb, vb, sb))
                else:
                    match += 1
                    if sa != sb:
                        src_mismatch += 1
                i += 1
                j += 1
        a_only += len(a_evs) - i
        b_only += len(b_evs) - j
        summary[key] = {
            'a_only': a_only,
            'b_only': b_only,
            'match': match,
            'value_mismatch': value_mismatch,
            'src_mismatch': src_mismatch,
            'mismatch_examples': mismatches,
        }
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log_a", help="engine log A")
    ap.add_argument("log_b", help="engine log B")
    ap.add_argument("--label", nargs=2, default=None,
                    help="host labels (default: derived from filename)")
    ap.add_argument("--match-window", type=float, default=5.0,
                    help="max seconds between A and B events to pair (default: 5.0)")
    ap.add_argument("--detail", action="store_true",
                    help="print first N mismatch examples per (SV, signal)")
    ap.add_argument("--top", type=int, default=3,
                    help="--detail: examples to print per key (default: 3)")
    args = ap.parse_args()

    label_a, label_b = args.label or [
        Path(args.log_a).stem.split('-')[0].split('.')[0],
        Path(args.log_b).stem.split('-')[0].split('.')[0],
    ]

    print(f"Parsing {label_a} ← {args.log_a} ...")
    a_events = parse_log(args.log_a)
    print(f"  {len(a_events)} bias-band events")
    print(f"Parsing {label_b} ← {args.log_b} ...")
    b_events = parse_log(args.log_b)
    print(f"  {len(b_events)} bias-band events")

    a_idx = index_by_key(a_events)
    b_idx = index_by_key(b_events)

    print(f"\n{label_a}: {len(a_idx)} (SV, signal, kind) keys")
    print(f"{label_b}: {len(b_idx)} (SV, signal, kind) keys")

    summary = diff_two_hosts(
        a_idx, b_idx, label_a, label_b,
        match_window_s=args.match_window,
    )

    # Tally totals.
    total_match = sum(s['match'] for s in summary.values())
    total_value_mismatch = sum(s['value_mismatch'] for s in summary.values())
    total_src_mismatch = sum(s['src_mismatch'] for s in summary.values())
    total_a_only = sum(s['a_only'] for s in summary.values())
    total_b_only = sum(s['b_only'] for s in summary.values())

    print(f"\n══ Cross-host diff totals ══")
    print(f"  match (value identical):       {total_match}")
    print(f"  value mismatch:                {total_value_mismatch}")
    print(f"  src mismatch (same val, diff): {total_src_mismatch}")
    print(f"  {label_a}-only events:                {total_a_only}")
    print(f"  {label_b}-only events:                {total_b_only}")

    # Per-key detail.
    print(f"\n══ Per-(SV, signal, kind) breakdown ══")
    print(f"{'SV':>4}  {'signal':>14}  {'kind':>10}  "
          f"{'match':>6}  {'val_mis':>7}  {'src_mis':>7}  "
          f"{label_a+'-only':>9}  {label_b+'-only':>9}")
    print("-" * 96)
    interesting = []
    for key in sorted(summary):
        sv, sig_label, kind = key
        s = summary[key]
        if s['value_mismatch'] > 0 or s['a_only'] > 5 or s['b_only'] > 5:
            interesting.append(key)
        if (s['match'] + s['value_mismatch'] + s['a_only']
                + s['b_only'] + s['src_mismatch']) == 0:
            continue
        marker = " ⚠" if s['value_mismatch'] > 0 else ""
        print(f"{sv:>4}  {sig_label:>14}  {kind:>10}  "
              f"{s['match']:>6}  {s['value_mismatch']:>7}  {s['src_mismatch']:>7}  "
              f"{s['a_only']:>9}  {s['b_only']:>9}{marker}")

    # Optional detail.
    if args.detail and interesting:
        print(f"\n══ First {args.top} mismatch examples per flagged key ══")
        for key in interesting:
            sv, sig_label, kind = key
            s = summary[key]
            if not s['mismatch_examples']:
                continue
            print(f"\n  {sv} {sig_label} {kind}:")
            for ta, va, sa, tb, vb, sb in s['mismatch_examples'][:args.top]:
                ta_str = datetime.fromtimestamp(ta).strftime('%H:%M:%S')
                tb_str = datetime.fromtimestamp(tb).strftime('%H:%M:%S')
                va_str = f"{va:+.3f}m" if va is not None else "MISS"
                vb_str = f"{vb:+.3f}m" if vb is not None else "MISS"
                print(f"    {ta_str} {label_a}: val={va_str} src={sa or '-'}")
                print(f"    {tb_str} {label_b}: val={vb_str} src={sb or '-'}")

    return 0 if total_value_mismatch == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

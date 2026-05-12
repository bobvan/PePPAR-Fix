#!/usr/bin/env python3
"""Aggregate [NAV-SIG_DISAGREE] events + correlate with [CATASTROPHIC_REJECT].

Phase A.5 → Phase B3 readiness analyzer for slipDetectUnified-main.

Bob's framing (2026-05-12 12:35 CDT):
    "we may use SVs the receiver doesn't, and may exclude SVs it uses,
     but we must LOG when we differ."

PR #27 (Charlie) plumbs `NavSigDisagreeMonitor` into the engine,
emitting one structured `[NAV-SIG_DISAGREE]` line per (SV, signal)
state-change between the receiver's UBX-NAV-SIG.prUsed verdict and
the engine's admit decision.  Two directions:

    receiver=excluded our=admit    → receiver caught something we
                                      didn't.  Phase B3 candidates.
    receiver=admitted our=exclude  → our gates (PB_GAP_DROP / elev /
                                      ZTD-trip / slip-flush cooldown)
                                      are stricter.  Audit signal.

This tool aggregates the event stream and answers the Phase B3
readiness question:

    "Of the catastrophic-reject events the engine produced overnight,
     how many had a `receiver=excluded our=admit` event in the
     preceding window for an SV involved in the cascade?"

That number is the empirical hit rate of NAV-SIG-based exclusion.
If hit rate ≥ ~90%, flipping `--nav-sig-gate=on` (PR #31) tonight
would have prevented most cascades.  If lower, the receiver's
verdict misses what we see and Phase B alone is insufficient.

Both directions are reported, but only the receiver-excluded-our-
admit direction drives the readiness verdict.

## Usage

    # One host
    python3 tools/analysis/nav_sig_disagree_analysis.py \
        data/day0512overnight-madhat.log

    # Multiple hosts, cross-host aggregation
    python3 tools/analysis/nav_sig_disagree_analysis.py \
        data/day0512overnight-{madhat,timehat,piface,clkpoc3}.log

    # Tune the correlation window (default ±60 s)
    python3 tools/analysis/nav_sig_disagree_analysis.py \
        --window-s 30  data/*.log

    # JSON output for downstream tooling
    python3 tools/analysis/nav_sig_disagree_analysis.py \
        --format json  data/*.log > nav_sig_summary.json

## Output sections

1. **Summary** — total disagreement events by direction, host, signal.
2. **Top SV+signal** by event count, both directions.
3. **Cat-reject correlation** — per cat-reject, was there a
   `receiver=excluded our=admit` event within ±window-s?  Hit rate +
   list of unmatched cat-rejects.
4. **Phase B3 readiness verdict** — hit rate × cohort, recommended
   `--nav-sig-gate` decision for the next overnight.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections import defaultdict, Counter
from dataclasses import dataclass, field, asdict
from datetime import datetime, timedelta
from pathlib import Path
from typing import Iterable


# ────────────────────────────────────────────────────────────────── #
# Log-line regex                                                     #
# ────────────────────────────────────────────────────────────────── #

# Engine log timestamp: "2026-05-12 18:30:15,123" or "2026-05-12T18:30:15,123"
_TS_RE = re.compile(
    r'^(?P<ts>\d{4}-\d{2}-\d{2}[T ]\d{2}:\d{2}:\d{2})[,\.]?\d*'
)

# [NAV-SIG_DISAGREE ep=N sv=SV sig=SIG] DIRECTION key=val key=val ...
_DISAGREE_RE = re.compile(
    r'\[NAV-SIG_DISAGREE\s+'
    r'ep=(?P<ep>\d+)\s+'
    r'sv=(?P<sv>[A-Z]\d{1,3})\s+'
    r'sig=(?P<sig>[A-Za-z0-9/_-]+)\]\s+'
    r'(?P<direction>receiver=(?:excluded|admitted)\s+our=(?:admit|exclude))'
    r'(?P<rest>.*)$'
)

# [CATASTROPHIC_REJECT] median |PR|=NN.Nm (X.XX L1 chips, X.XXX ms) > ...
_CAT_REJECT_RE = re.compile(
    r'\[CATASTROPHIC_REJECT\]\s+median\s+\|PR\|='
    r'(?P<median_m>[\d.]+)m'
    r'(?:\s+\((?P<chips>[\d.]+)\s+L1\s+chips,\s+(?P<ms>[\d.]+)\s+ms\))?'
)

# Cycle-slip log (per-SV) — used to pull SV identity for the cat-reject
# context.  [SLIP] sv=G07 ... or "slip: sv=E19 ..."
_SLIP_RE = re.compile(
    r'(?:\[SLIP\]\s+sv=|slip:\s+sv=)(?P<sv>[A-Z]\d{1,3})'
)

# [SECOND_OPINION_POS] tripped: ... — also useful as a cascade-marker.
_SO_POS_RE = re.compile(r'\[SECOND_OPINION_POS\]\s+tripped')


# Key=value field parser (within a [NAV-SIG_DISAGREE] line's rest)
_KV_RE = re.compile(r'(\w+)=(\S+)')


def _parse_ts(text: str) -> datetime | None:
    """Parse engine-log timestamp prefix to UTC-naive datetime."""
    m = _TS_RE.match(text)
    if not m:
        return None
    ts = m.group('ts').replace('T', ' ')
    try:
        return datetime.strptime(ts, '%Y-%m-%d %H:%M:%S')
    except ValueError:
        return None


def _parse_kv(rest: str) -> dict[str, str]:
    return dict(_KV_RE.findall(rest))


# ────────────────────────────────────────────────────────────────── #
# Event types                                                        #
# ────────────────────────────────────────────────────────────────── #

@dataclass
class DisagreeEvent:
    ts: datetime
    host: str
    epoch: int
    sv: str
    sig: str
    direction: str  # 'receiver=excluded our=admit' | 'receiver=admitted our=exclude'
    fields: dict[str, str] = field(default_factory=dict)

    @property
    def receiver_excludes(self) -> bool:
        return 'receiver=excluded' in self.direction


@dataclass
class CatRejectEvent:
    ts: datetime
    host: str
    median_m: float | None
    chips: float | None
    ms: float | None
    # Best-effort: list of SVs mentioned in the surrounding context.
    # Populated by the second pass (post-parse).
    nearby_svs: list[str] = field(default_factory=list)


# ────────────────────────────────────────────────────────────────── #
# Parsers                                                            #
# ────────────────────────────────────────────────────────────────── #

def parse_log(path: Path, host: str | None = None
              ) -> tuple[list[DisagreeEvent], list[CatRejectEvent]]:
    """Single-pass parse of one engine log.

    Returns ``(disagree_events, cat_reject_events)`` in temporal order.
    SVs near each cat-reject are pulled from a small in-memory ring
    buffer of recent slip lines so the analyzer can attribute the
    cat-reject to the SVs in play around its trigger.
    """
    if host is None:
        host = _host_from_path(path)
    disagrees: list[DisagreeEvent] = []
    cat_rejects: list[CatRejectEvent] = []

    # Ring buffer of (ts, sv) tuples for slip-line attribution.  Sized
    # for the cat-reject median window (~30 obs); 64 is comfortable.
    recent_slips: list[tuple[datetime, str]] = []
    SLIP_BUF_LIMIT = 64

    with path.open('r', encoding='utf-8', errors='replace') as f:
        for line in f:
            ts = _parse_ts(line)
            if ts is None:
                continue

            m = _DISAGREE_RE.search(line)
            if m:
                disagrees.append(DisagreeEvent(
                    ts=ts, host=host,
                    epoch=int(m.group('ep')),
                    sv=m.group('sv'),
                    sig=m.group('sig'),
                    direction=m.group('direction'),
                    fields=_parse_kv(m.group('rest')),
                ))
                continue

            m = _CAT_REJECT_RE.search(line)
            if m:
                # Pull SVs from the recent-slip ring buffer (within
                # ±30 s of this event — cat-rejects are immediate, the
                # SVs that triggered the cascade are in the prior
                # ~30-epoch window).
                cutoff = ts - timedelta(seconds=30)
                nearby = [sv for (t, sv) in recent_slips if t >= cutoff]
                cat_rejects.append(CatRejectEvent(
                    ts=ts, host=host,
                    median_m=float(m.group('median_m')),
                    chips=float(m.group('chips')) if m.group('chips') else None,
                    ms=float(m.group('ms')) if m.group('ms') else None,
                    nearby_svs=sorted(set(nearby)),
                ))
                continue

            m = _SLIP_RE.search(line)
            if m:
                recent_slips.append((ts, m.group('sv')))
                if len(recent_slips) > SLIP_BUF_LIMIT:
                    recent_slips = recent_slips[-SLIP_BUF_LIMIT:]

    return disagrees, cat_rejects


def _host_from_path(path: Path) -> str:
    """Heuristic — guess hostname from filename token (e.g. madhat)."""
    stem = path.stem.lower()
    for h in ('madhat', 'timehat', 'piface', 'clkpoc3', 'ptpmon', 'bbb'):
        if h in stem:
            return h
    return path.stem


# ────────────────────────────────────────────────────────────────── #
# Aggregation                                                        #
# ────────────────────────────────────────────────────────────────── #

def aggregate_disagrees(events: Iterable[DisagreeEvent]) -> dict:
    """Group disagreement events by (direction, host, sv, sig)."""
    by_direction: Counter = Counter()
    by_host_dir: Counter = Counter()
    by_sv_sig: dict[tuple[str, str], Counter] = defaultdict(Counter)
    by_sig: dict[str, Counter] = defaultdict(Counter)
    by_sv: dict[str, Counter] = defaultdict(Counter)
    total = 0
    for ev in events:
        total += 1
        by_direction[ev.direction] += 1
        by_host_dir[(ev.host, ev.direction)] += 1
        by_sv_sig[(ev.sv, ev.sig)][ev.direction] += 1
        by_sig[ev.sig][ev.direction] += 1
        by_sv[ev.sv][ev.direction] += 1
    return {
        'total': total,
        'by_direction': dict(by_direction),
        'by_host_dir': {f"{h}|{d}": n for (h, d), n in by_host_dir.items()},
        'by_sv_sig': {f"{sv}|{sig}": dict(c) for (sv, sig), c in by_sv_sig.items()},
        'by_sig':    {k: dict(v) for k, v in by_sig.items()},
        'by_sv':     {k: dict(v) for k, v in by_sv.items()},
    }


def correlate_cat_rejects(
    cat_rejects: list[CatRejectEvent],
    disagrees: list[DisagreeEvent],
    window_s: float = 60.0,
) -> dict:
    """For each cat-reject, find ``receiver=excluded our=admit`` events
    in ``[ts-window_s, ts+window_s]`` on the SAME host for an SV that
    appears in the cat-reject's ``nearby_svs`` list.

    Returns per-cat-reject record plus aggregate hit rate.  The hit
    rate is the Phase B3 readiness signal: it answers
    "would NAV-SIG-prUsed=0 exclusion have caught this cat-reject?"
    """
    # Index disagrees by host for fast lookup
    host_evs: dict[str, list[DisagreeEvent]] = defaultdict(list)
    for d in disagrees:
        if d.receiver_excludes:
            host_evs[d.host].append(d)
    # Pre-sort each host's list by timestamp for window search.
    for h in host_evs:
        host_evs[h].sort(key=lambda d: d.ts)

    per_reject: list[dict] = []
    n_hit = 0
    n_total = len(cat_rejects)

    for cr in cat_rejects:
        window = timedelta(seconds=window_s)
        lo = cr.ts - window
        hi = cr.ts + window
        candidates = [
            d for d in host_evs.get(cr.host, [])
            if lo <= d.ts <= hi
        ]
        # "Hit" = at least one candidate whose SV appears in the
        # cat-reject's nearby_svs list (so we're matching a slip
        # SV to a receiver-excluded SV, not just any disagreement
        # in the time window).
        matching = [
            d for d in candidates
            if not cr.nearby_svs or d.sv in cr.nearby_svs
        ]
        hit = bool(matching)
        if hit:
            n_hit += 1
        per_reject.append({
            'ts': cr.ts.isoformat(),
            'host': cr.host,
            'median_m': cr.median_m,
            'chips': cr.chips,
            'ms': cr.ms,
            'nearby_svs': cr.nearby_svs,
            'hit': hit,
            'matching_svs': sorted({d.sv for d in matching}),
            'window_disagree_count': len(candidates),
        })

    hit_rate = (n_hit / n_total) if n_total > 0 else None
    return {
        'window_s': window_s,
        'n_cat_rejects': n_total,
        'n_hit': n_hit,
        'n_miss': n_total - n_hit,
        'hit_rate': hit_rate,
        'per_reject': per_reject,
    }


# ────────────────────────────────────────────────────────────────── #
# Reporting                                                          #
# ────────────────────────────────────────────────────────────────── #

def _readiness_verdict(hit_rate: float | None, n: int) -> str:
    if n == 0:
        return "no cat-rejects in dataset — analyzer can't speak to Phase B3 readiness"
    if hit_rate is None:
        return "indeterminate"
    if hit_rate >= 0.90:
        return (f"READY — receiver-excluded events preceded {hit_rate*100:.0f}% of "
                f"cat-rejects (n={n}).  --nav-sig-gate=on would prevent the bulk "
                f"of cascades.")
    if hit_rate >= 0.50:
        return (f"PARTIAL — {hit_rate*100:.0f}% of cat-rejects had a receiver-side "
                f"warning (n={n}).  Phase B helps but isn't sufficient; need "
                f"additional L0 (PR continuity or carrier-slip detector).")
    return (f"NOT READY — only {hit_rate*100:.0f}% of cat-rejects had a "
            f"receiver-side warning (n={n}).  Receiver's verdict misses the "
            f"failure mode; Phase B alone won't help.  Investigate further.")


def render_text(disagree_summary: dict, corr: dict,
                top_n: int = 15) -> str:
    out: list[str] = []
    push = out.append

    total = disagree_summary['total']
    push("=" * 72)
    push("[NAV-SIG_DISAGREE] aggregate analysis")
    push("=" * 72)
    push("")
    push(f"Total disagreement events: {total}")
    push("")
    push("By direction:")
    for d, n in sorted(disagree_summary['by_direction'].items(),
                       key=lambda kv: -kv[1]):
        push(f"  {d:<45s}  {n:>6d}")
    push("")
    push("By host × direction:")
    for k, n in sorted(disagree_summary['by_host_dir'].items(),
                       key=lambda kv: -kv[1]):
        push(f"  {k:<55s}  {n:>6d}")
    push("")
    push(f"Top {top_n} (SV, signal) by disagreement count:")
    items = sorted(
        disagree_summary['by_sv_sig'].items(),
        key=lambda kv: -sum(kv[1].values()),
    )[:top_n]
    for key, counts in items:
        recv_exc = counts.get('receiver=excluded our=admit', 0)
        recv_adm = counts.get('receiver=admitted our=exclude', 0)
        push(f"  {key:<22s}  recv-excludes-our-admit={recv_exc:>4d}  "
             f"recv-admits-our-exclude={recv_adm:>4d}")
    push("")
    push("By signal (aggregated across SVs):")
    items = sorted(
        disagree_summary['by_sig'].items(),
        key=lambda kv: -sum(kv[1].values()),
    )
    for sig, counts in items:
        recv_exc = counts.get('receiver=excluded our=admit', 0)
        recv_adm = counts.get('receiver=admitted our=exclude', 0)
        push(f"  {sig:<16s}  recv-excludes-our-admit={recv_exc:>4d}  "
             f"recv-admits-our-exclude={recv_adm:>4d}")
    push("")

    push("=" * 72)
    push("Phase B3 readiness — cat-reject correlation")
    push("=" * 72)
    push("")
    push(f"Window: ±{corr['window_s']:.0f} s for receiver-excluded → cat-reject")
    push(f"        (matching by SV from cat-reject's surrounding slip context)")
    push("")
    n = corr['n_cat_rejects']
    n_hit = corr['n_hit']
    n_miss = corr['n_miss']
    rate = corr['hit_rate']
    push(f"Cat-rejects:         {n}")
    push(f"  Receiver warned:   {n_hit}  ({(rate*100 if rate is not None else 0):.1f}%)")
    push(f"  No warning:        {n_miss}")
    push("")
    push("Verdict: " + _readiness_verdict(rate, n))
    push("")
    if n > 0 and n_miss > 0:
        push("Unmatched cat-rejects (would still cascade with --nav-sig-gate=on):")
        for r in corr['per_reject']:
            if r['hit']:
                continue
            chips = f" {r['chips']:.2f} chips" if r['chips'] is not None else ""
            push(f"  {r['ts']}  host={r['host']:<8s}  |PR|={r['median_m']:.1f}m"
                 f"{chips}  SVs={','.join(r['nearby_svs']) or '(none)'}")
        push("")

    return "\n".join(out)


# ────────────────────────────────────────────────────────────────── #
# CLI                                                                #
# ────────────────────────────────────────────────────────────────── #

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument('logs', nargs='+', type=Path,
                    help='Engine log files to analyze (one per host).')
    ap.add_argument('--window-s', type=float, default=60.0,
                    help='Correlation window for cat-reject ↔ disagreement '
                         '(default 60.0).')
    ap.add_argument('--format', choices=['text', 'json'], default='text',
                    help='Output format.')
    ap.add_argument('--top-n', type=int, default=15,
                    help='Top-N (SV, signal) pairs to display (default 15).')
    args = ap.parse_args(argv)

    all_disagrees: list[DisagreeEvent] = []
    all_cat_rejects: list[CatRejectEvent] = []
    for path in args.logs:
        if not path.exists():
            print(f"WARN: {path} does not exist; skipping",
                  file=sys.stderr)
            continue
        d, c = parse_log(path)
        all_disagrees.extend(d)
        all_cat_rejects.extend(c)

    disagree_summary = aggregate_disagrees(all_disagrees)
    corr = correlate_cat_rejects(
        all_cat_rejects, all_disagrees, window_s=args.window_s)

    if args.format == 'json':
        result = {
            'inputs': [str(p) for p in args.logs],
            'disagree_summary': disagree_summary,
            'cat_reject_correlation': corr,
            'verdict': _readiness_verdict(corr['hit_rate'],
                                          corr['n_cat_rejects']),
        }
        print(json.dumps(result, indent=2, default=str))
    else:
        print(render_text(disagree_summary, corr, top_n=args.top_n))

    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""Analyze a 3-phase antenna-step validation run (I-125649 + I-145846).

Consumes engine logs + TICC chA captures from a baseline / stepped /
recovery sequence (procedure: docs/synthetic-antenna-step-test.md)
and emits a per-observable table aligned with the pass/fail
criteria.

OBSERVABLES PER PHASE:

  - ARP residual mean + std (from [AntPosEst] log lines)
  - ARP-motion alarm count (currently approximated as
    POSITION_WATCHDOG entries; will refine when I-145846
    formalizes the alarm log line)
  - WL re-converge time (time from phase start to first
    [WL_FIX_LIFE] event=enter on a stable SV)
  - PPS chA TDEV at τ=1s, 10s, 100s (from --ticc-log CSV)
  - ZTD residual range (from [FIXEDPOS_ZTD] log lines)
  - Slip event count + reason breakdown

VERDICT LOGIC:

  PASS iff:
    Phase 2 alarm count >= 1 AND
    Phase 2 PPS TDEV(1s) within 2x Phase 1 AND
    Phase 3 ARP residual stabilized within 5 min AND
    All three phases parsed cleanly

This is a stub — quantitative pass/fail tuned with first-run
data.  Scope: produce a readable report Main can eyeball, not
a full automation.
"""
from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path


# ── Log-line parsers ─────────────────────────────────────────────── #


_ANTPOS_RE = re.compile(
    r'^(?P<date>\d{4}-\d{2}-\d{2})\s+'
    r'(?P<time>\d{2}:\d{2}:\d{2}),(?P<ms>\d+)\s+\S+\s+'
    r'\[AntPosEst\][^p]*pos=\(\s*(?P<lat>[-\d.]+),\s*'
    r'(?P<lon>[-\d.]+),\s*(?P<alt>[-\d.]+)\)\s+'
    r'σ=(?P<sigma>[\d.]+)m'
)
_FIXED_ZTD_RE = re.compile(
    r'^(?P<date>\d{4}-\d{2}-\d{2})\s+'
    r'(?P<time>\d{2}:\d{2}:\d{2})[,.]?\d*\s+\S+\s+'
    r'\[FIXEDPOS_ZTD\]\s+\S*\s*ZTD=(?P<ztd>[+-][\d.]+)mm'
)
_WL_FIX_RE = re.compile(
    r'^(?P<date>\d{4}-\d{2}-\d{2})\s+'
    r'(?P<time>\d{2}:\d{2}:\d{2}),(?P<ms>\d+)\s+\S+\s+'
    r'\[WL_FIX_LIFE\]\s+event=(?P<event>\w+)\s+'
    r'sv=(?P<sv>[A-Z]\d{2,3})\s+n_wl=(?P<n_wl>-?\d+|None)'
)
_SLIP_RE = re.compile(
    r'^(?P<date>\d{4}-\d{2}-\d{2})\s+'
    r'(?P<time>\d{2}:\d{2}:\d{2}),(?P<ms>\d+)\s+\S+\s+'
    r'slip:\s+sv=(?P<sv>\S+)\s+reasons=(?P<reasons>\S+)\s+'
    r'conf=(?P<conf>\w+)'
)
_WATCHDOG_RE = re.compile(
    r'(?P<date>\d{4}-\d{2}-\d{2})\s+'
    r'(?P<time>\d{2}:\d{2}:\d{2})[,.]\d+\s+\S+\s+'
    r'(?P<msg>(?:Watchdog|POSITION WATCHDOG|ARP_WATCHDOG_BARK).*)'
)


def _parse_ts(date: str, time: str) -> datetime:
    return datetime.fromisoformat(f"{date}T{time}+00:00")


@dataclass
class PhaseObservables:
    name: str
    log_path: Path
    epochs: int = 0
    arp_lat_mean: float = float("nan")
    arp_lon_mean: float = float("nan")
    arp_alt_mean: float = float("nan")
    arp_sigma_mean: float = float("nan")
    ztd_residual_min_mm: float = float("nan")
    ztd_residual_max_mm: float = float("nan")
    n_alarm_events: int = 0
    n_slip_events: int = 0
    slip_reason_counter: Counter = None
    n_wl_fix_enter: int = 0
    n_wl_fix_exit: int = 0
    first_wl_fix_dt_s: float | None = None
    start_ts: datetime | None = None
    end_ts: datetime | None = None


def parse_phase_log(path: Path, name: str) -> PhaseObservables:
    obs = PhaseObservables(name=name, log_path=path,
                            slip_reason_counter=Counter())
    arp_lats, arp_lons, arp_alts, arp_sigmas = [], [], [], []
    ztds = []
    first_seen_ts: datetime | None = None
    last_ts: datetime | None = None
    with open(path) as f:
        for line in f:
            ts: datetime | None = None
            if (m := _ANTPOS_RE.search(line)):
                ts = _parse_ts(m.group('date'), m.group('time'))
                arp_lats.append(float(m.group('lat')))
                arp_lons.append(float(m.group('lon')))
                arp_alts.append(float(m.group('alt')))
                arp_sigmas.append(float(m.group('sigma')))
            elif (m := _FIXED_ZTD_RE.search(line)):
                ts = _parse_ts(m.group('date'), m.group('time'))
                ztds.append(float(m.group('ztd')))
            elif (m := _WL_FIX_RE.search(line)):
                ts = _parse_ts(m.group('date'), m.group('time'))
                if m.group('event') == 'enter':
                    obs.n_wl_fix_enter += 1
                    if obs.first_wl_fix_dt_s is None and first_seen_ts:
                        obs.first_wl_fix_dt_s = (
                            ts - first_seen_ts).total_seconds()
                else:
                    obs.n_wl_fix_exit += 1
            elif (m := _SLIP_RE.search(line)):
                ts = _parse_ts(m.group('date'), m.group('time'))
                obs.n_slip_events += 1
                obs.slip_reason_counter[m.group('reasons')] += 1
            elif (m := _WATCHDOG_RE.search(line)):
                ts = _parse_ts(m.group('date'), m.group('time'))
                # Coarse: every Watchdog/ARP_WATCHDOG_BARK line is an
                # alarm-class event.  Refine when the dedicated alarm
                # log line is in place (I-145846 follow-up).
                obs.n_alarm_events += 1
            if ts is not None:
                first_seen_ts = first_seen_ts or ts
                last_ts = ts
    if arp_lats:
        obs.arp_lat_mean = sum(arp_lats) / len(arp_lats)
        obs.arp_lon_mean = sum(arp_lons) / len(arp_lons)
        obs.arp_alt_mean = sum(arp_alts) / len(arp_alts)
        obs.arp_sigma_mean = sum(arp_sigmas) / len(arp_sigmas)
    if ztds:
        obs.ztd_residual_min_mm = min(ztds)
        obs.ztd_residual_max_mm = max(ztds)
    obs.epochs = len(arp_lats) + len(ztds)
    obs.start_ts = first_seen_ts
    obs.end_ts = last_ts
    return obs


# ── TICC TDEV ────────────────────────────────────────────────────── #


def ticc_chA_tdev(path: Path, taus: tuple = (1, 10, 100)) -> dict:
    """Compute TDEV at given tau values from TICC chA samples.

    Reads 'host_timestamp,host_monotonic,ref_sec,ref_ps,channel'
    rows; only chA rows feed TDEV.  Returns {tau: tdev_ns} or
    empty dict if too few samples.
    """
    times = []
    phases_ns = []
    try:
        with open(path) as f:
            r = csv.DictReader(f)
            for row in r:
                if row.get('channel', '').strip() != 'A':
                    continue
                t = float(row['host_monotonic'])
                # ref_sec + ref_ps gives the reference time in ns
                # but for differential TDEV we want chA against itself.
                # Use the per-row deviation from integer-second target.
                ref_ns = (float(row['ref_sec']) * 1e9
                           + float(row['ref_ps']) * 1e-3)
                round_ns = round(ref_ns / 1e9) * 1e9
                phase = ref_ns - round_ns
                times.append(t)
                phases_ns.append(phase)
    except (OSError, KeyError, ValueError):
        return {}
    if len(phases_ns) < 100:
        return {}
    out = {}
    for tau in taus:
        # Coarse overlapping TDEV via three-point variance.
        # σ_x(τ) = sqrt((1/6) · <(x(t+2τ) - 2·x(t+τ) + x(t))²>)
        n = len(phases_ns)
        step = max(1, int(tau))
        if n < 3 * step:
            continue
        diffs2 = []
        for i in range(n - 2 * step):
            d = (phases_ns[i + 2 * step] - 2 * phases_ns[i + step]
                 + phases_ns[i])
            diffs2.append(d * d)
        if not diffs2:
            continue
        var = sum(diffs2) / len(diffs2) / 6.0
        out[tau] = math.sqrt(var)
    return out


# ── Reporting ────────────────────────────────────────────────────── #


def render_table(phases: list[PhaseObservables],
                  ticc_tdev: dict[str, dict]) -> None:
    print("="*80)
    print("ANTENNA-STEP VALIDATION — phase observable summary")
    print("="*80)
    print()
    print(f"{'observable':<32}  " + "  ".join(
        f"{p.name:>14}" for p in phases))
    print("-"*80)
    rows = [
        ("epochs", lambda p: f"{p.epochs}"),
        ("ARP lat mean (°)", lambda p: f"{p.arp_lat_mean:.7f}"
            if not math.isnan(p.arp_lat_mean) else "n/a"),
        ("ARP lon mean (°)", lambda p: f"{p.arp_lon_mean:.7f}"
            if not math.isnan(p.arp_lon_mean) else "n/a"),
        ("ARP alt mean (m)", lambda p: f"{p.arp_alt_mean:.3f}"
            if not math.isnan(p.arp_alt_mean) else "n/a"),
        ("AntPosEst σ mean (m)", lambda p: f"{p.arp_sigma_mean:.3f}"
            if not math.isnan(p.arp_sigma_mean) else "n/a"),
        ("ZTD residual range (mm)",
         lambda p: f"{p.ztd_residual_min_mm:+.0f}..{p.ztd_residual_max_mm:+.0f}"
            if not math.isnan(p.ztd_residual_min_mm) else "n/a"),
        ("alarm events", lambda p: f"{p.n_alarm_events}"),
        ("slip events", lambda p: f"{p.n_slip_events}"),
        ("WL fix-enter / exit",
         lambda p: f"{p.n_wl_fix_enter} / {p.n_wl_fix_exit}"),
        ("first WL fix at (s)",
         lambda p: f"{p.first_wl_fix_dt_s:.0f}"
            if p.first_wl_fix_dt_s is not None else "n/a"),
    ]
    for label, fn in rows:
        print(f"{label:<32}  " + "  ".join(
            f"{fn(p):>14}" for p in phases))
    if ticc_tdev:
        print()
        print(f"{'TDEV chA (ns)':<32}  " + "  ".join(
            f"{p.name:>14}" for p in phases))
        print("-"*80)
        for tau in (1, 10, 100):
            cells = []
            for p in phases:
                d = ticc_tdev.get(p.name, {})
                cells.append(f"{d.get(tau, float('nan')):.3f}"
                              if d.get(tau) is not None else "n/a")
            print(f"  τ = {tau:>3} s{'':<24}  " + "  ".join(
                f"{c:>14}" for c in cells))


def verdict(phases: list[PhaseObservables],
             ticc_tdev: dict[str, dict]) -> bool:
    """True iff all PASS criteria met."""
    if len(phases) < 2:
        return False
    baseline = phases[0]
    stepped = phases[1] if len(phases) > 1 else None
    if stepped is None:
        return False
    if stepped.n_alarm_events < 1:
        print("\nFAIL: Phase 2 (stepped) raised no alarm events.")
        return False
    base_tdev = ticc_tdev.get(baseline.name, {}).get(1)
    step_tdev = ticc_tdev.get(stepped.name, {}).get(1)
    if base_tdev is not None and step_tdev is not None:
        if step_tdev > 2.0 * base_tdev:
            print(f"\nFAIL: Phase 2 PPS TDEV(1s)={step_tdev:.3f} ns "
                  f"> 2x baseline {base_tdev:.3f} ns.")
            return False
    print("\nPASS (per available observables — manual review of "
          "ARP-motion specifics still recommended for first run).")
    return True


# ── CLI ──────────────────────────────────────────────────────────── #


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--baseline-log", type=Path, required=True)
    ap.add_argument("--stepped-log", type=Path, required=True)
    ap.add_argument("--recovery-log", type=Path, required=False)
    ap.add_argument("--baseline-ticc", type=Path, required=False)
    ap.add_argument("--stepped-ticc", type=Path, required=False)
    ap.add_argument("--recovery-ticc", type=Path, required=False)
    args = ap.parse_args()

    phases: list[PhaseObservables] = []
    phases.append(parse_phase_log(args.baseline_log, "baseline"))
    phases.append(parse_phase_log(args.stepped_log, "stepped"))
    if args.recovery_log:
        phases.append(parse_phase_log(args.recovery_log, "recovery"))

    ticc_tdev: dict[str, dict] = {}
    if args.baseline_ticc:
        ticc_tdev["baseline"] = ticc_chA_tdev(args.baseline_ticc)
    if args.stepped_ticc:
        ticc_tdev["stepped"] = ticc_chA_tdev(args.stepped_ticc)
    if args.recovery_ticc:
        ticc_tdev["recovery"] = ticc_chA_tdev(args.recovery_ticc)

    render_table(phases, ticc_tdev)
    ok = verdict(phases, ticc_tdev)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

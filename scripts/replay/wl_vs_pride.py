#!/usr/bin/env python3
"""Compare engine WL-fix integers against PRIDE-PPP float WL ambiguities.

Diagnostic for I-155354 — systematic-bias investigation in the
streaming PPP filter's WL solutions.  Hypothesis #1: the streaming
MW tracker fixes WL ambiguities to wrong integers for some SVs;
PRIDE's batch lsq↔redig produces tighter float WL estimates from
the same RINEX.  Comparing the two reveals per-SV cycle disagreements
that contribute multi-meter position bias.

INPUTS:

  --engine-log PATH   Engine output log with [WL_FIX_LIFE] events.
                      Each event records:
                        timestamp, sv, n_wl (engine integer fix),
                        elev, consistency, int_history.
  --pride-amb PATH    PRIDE amb_<DOY>_<site> file with per-arc float
                      WL ambiguities (WLamb column) + sigmas (SigWL).

OUTPUT:

  Prints a per-SV table to stdout (and CSV if --csv given) showing:
    sv, pride_wl_float, pride_sigma, pride_int (rounded), engine_n_wl,
    cycles_diff (engine - pride_int), confidence (sigma-based),
    pos_bias_contribution_m (cycles_diff × WL_wavelength projected by
    elevation cos for IF-combination contribution).

The engine WL_FIX_LIFE events within the PRIDE arc window
(arc.MjdS, arc.MjdE) for each SV are matched against the arc's
float WL.  A single PRIDE arc may overlap multiple engine fix
events — the analysis reports the most recent engine n_wl per arc
(what the filter believed was correct at arc-end).
"""
from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path


# WL wavelength.  λ_WL = c / (f1 - f2).
# GPS L1=1575.42 MHz, L2=1227.60 MHz → λ_WL ≈ 86.2 cm
# GAL E1=1575.42, E5a=1176.45 → λ_WL ≈ 75.1 cm  (different L5 freq)
# BDS B1I=1561.098, B3I=1268.52 → λ_WL ≈ 102.5 cm
# We compute per-system below.
_C = 299_792_458.0
_F1_GPS, _F2_GPS = 1575.42e6, 1227.60e6
_F1_GAL, _F2_GAL = 1575.42e6, 1176.45e6
_F1_BDS, _F2_BDS = 1561.098e6, 1268.52e6


def _wl_wavelength_m(sys_letter: str) -> float:
    if sys_letter == 'G':
        return _C / (_F1_GPS - _F2_GPS)
    if sys_letter == 'E':
        return _C / (_F1_GAL - _F2_GAL)
    if sys_letter == 'C':
        return _C / (_F1_BDS - _F2_BDS)
    # GLONASS / QZSS / IRNSS — fall back to GPS-class.
    return _C / (_F1_GPS - _F2_GPS)


# ── PRIDE amb-file parser ────────────────────────────────────────── #


@dataclass
class PrideArc:
    sv: str
    if_amb: float          # IF combination float ambiguity (cycle)
    wl_amb: float          # WL float ambiguity (cycle)
    mjd_start: float
    mjd_end: float
    sigma_if: float
    sigma_wl: float
    elev: float

    @property
    def wl_int(self) -> int:
        return int(round(self.wl_amb))

    @property
    def wl_residual(self) -> float:
        return self.wl_amb - self.wl_int

    @property
    def confident_int(self) -> bool:
        """True iff PRIDE's float WL is within 0.25 cycles of an integer
        AND its σ is < 0.25 cycles.  Otherwise the rounded integer is
        not a confident anchor for comparison."""
        return abs(self.wl_residual) < 0.25 and self.sigma_wl < 0.25


_AMB_DATA_RE = re.compile(
    r'^\s*([A-Z]\d{2,3})\s+'
    r'([-+]?\d+\.\d+)\s+'      # IFamb
    r'([-+]?\d+\.\d+)\s+'      # WLamb
    r'(\d+\.\d+)\s+'            # MjdS
    r'(\d+\.\d+)\s+'            # MjdE
    r'(\d+\.\d+)\s+'            # SigIF
    r'(\d+\.\d+)\s+'            # SigWL
    r'([-+]?\d+\.\d+)\s*$'      # Elev
)


def parse_pride_amb(path: Path) -> list[PrideArc]:
    arcs: list[PrideArc] = []
    in_data = False
    with open(path) as f:
        for line in f:
            if 'END OF HEADER' in line:
                in_data = True
                continue
            if not in_data:
                continue
            m = _AMB_DATA_RE.match(line)
            if m is None:
                continue
            arcs.append(PrideArc(
                sv=m.group(1),
                if_amb=float(m.group(2)),
                wl_amb=float(m.group(3)),
                mjd_start=float(m.group(4)),
                mjd_end=float(m.group(5)),
                sigma_if=float(m.group(6)),
                sigma_wl=float(m.group(7)),
                elev=float(m.group(8)),
            ))
    return arcs


# ── Engine WL_FIX_LIFE parser ────────────────────────────────────── #


@dataclass
class EngineWlEvent:
    ts: datetime
    event: str              # "enter" | "exit"
    sv: str
    n_wl: int
    elev: float | None
    consistency: str
    int_history: list[int]  # may be empty for exit events


_WL_EVENT_RE = re.compile(
    r'^(?P<date>\d{4}-\d{2}-\d{2})\s+'
    r'(?P<time>\d{2}:\d{2}:\d{2}),(?P<ms>\d+)\s+'
    r'\S+\s+\[WL_FIX_LIFE\]\s+'
    r'event=(?P<event>\w+)\s+'
    r'sv=(?P<sv>[A-Z]\d{2,3})\s+'
    r'n_wl=(?P<n_wl>-?\d+|None)\s+'
    r'elev=(?P<elev>[-+]?\d+\.?\d*|\?)\s+'
    r'(?:consistency=(?P<consistency>\w+)\s+)?'
    r'int_history=(?P<history>\[[-\d, ]*\])?'
)


def parse_engine_log(path: Path) -> list[EngineWlEvent]:
    out: list[EngineWlEvent] = []
    with open(path) as f:
        for line in f:
            if '[WL_FIX_LIFE]' not in line:
                continue
            m = _WL_EVENT_RE.search(line)
            if m is None:
                continue
            ts = datetime.fromisoformat(
                f"{m.group('date')}T{m.group('time')}+00:00")
            n_wl_str = m.group('n_wl')
            if n_wl_str == 'None':
                continue
            elev_str = m.group('elev')
            elev = (float(elev_str) if elev_str not in ('?', None)
                    else None)
            history_str = m.group('history') or '[]'
            history = [int(x.strip()) for x in
                       history_str.strip('[]').split(',')
                       if x.strip()]
            out.append(EngineWlEvent(
                ts=ts,
                event=m.group('event'),
                sv=m.group('sv'),
                n_wl=int(n_wl_str),
                elev=elev,
                consistency=m.group('consistency') or '',
                int_history=history,
            ))
    return out


# ── MJD ↔ datetime helpers ───────────────────────────────────────── #


_MJD_EPOCH = datetime(1858, 11, 17, tzinfo=timezone.utc)


def datetime_to_mjd(dt: datetime) -> float:
    delta = dt - _MJD_EPOCH
    return delta.total_seconds() / 86400.0


def mjd_to_datetime(mjd: float) -> datetime:
    from datetime import timedelta
    return _MJD_EPOCH + timedelta(days=mjd)


# ── Comparison ───────────────────────────────────────────────────── #


@dataclass
class Comparison:
    sv: str
    arc_start: datetime
    arc_end: datetime
    pride_wl_float: float
    pride_sigma_wl: float
    pride_wl_int: int
    pride_confident: bool
    engine_events_in_arc: int
    engine_n_wl_first: int | None     # earliest engine fix in arc
    engine_n_wl_last: int | None      # latest engine fix in arc
    cycles_diff_first: int | None
    cycles_diff_last: int | None
    pos_bias_first_m: float | None
    pos_bias_last_m: float | None
    elev: float


def compare(arcs: list[PrideArc],
             events: list[EngineWlEvent]) -> list[Comparison]:
    # All events per SV (both enter and exit) sorted by time, so we
    # can determine which fix was ACTIVE at any moment in the arc.
    by_sv_all: dict[str, list[EngineWlEvent]] = defaultdict(list)
    for e in events:
        by_sv_all[e.sv].append(e)
    for sv in by_sv_all:
        by_sv_all[sv].sort(key=lambda x: x.ts)
    out: list[Comparison] = []
    for arc in arcs:
        sv_events_all = by_sv_all.get(arc.sv, [])
        # In-arc enters AND the active fix at arc start (most recent
        # enter before arc, with no exit between then and arc start).
        in_arc: list[EngineWlEvent] = []
        active_at_start: EngineWlEvent | None = None
        for e in sv_events_all:
            mjd = datetime_to_mjd(e.ts)
            if mjd > arc.mjd_end:
                break
            if mjd < arc.mjd_start:
                if e.event == 'enter':
                    active_at_start = e
                elif e.event == 'exit':
                    active_at_start = None
            elif e.event == 'enter':
                in_arc.append(e)
        # If the engine entered before the arc and never exited, that
        # fix was active at arc start — count it as the first event.
        if active_at_start is not None:
            in_arc.insert(0, active_at_start)
        e_first = in_arc[0] if in_arc else None
        e_last = in_arc[-1] if in_arc else None
        wl_lambda_m = _wl_wavelength_m(arc.sv[0])
        # Position-bias contribution: a cycle-diff in WL maps to
        # ~λ_WL of geometric range error projected onto the receiver-
        # SV unit vector.  Without the per-SV residual breakdown we
        # report the worst-case (full λ × cycles).  Real bias is
        # smaller after multi-SV LSQ partial cancellation.
        cycles_first = (e_first.n_wl - arc.wl_int
                        if e_first and arc.confident_int else None)
        cycles_last = (e_last.n_wl - arc.wl_int
                       if e_last and arc.confident_int else None)
        bias_first = (cycles_first * wl_lambda_m
                      if cycles_first is not None else None)
        bias_last = (cycles_last * wl_lambda_m
                     if cycles_last is not None else None)
        out.append(Comparison(
            sv=arc.sv,
            arc_start=mjd_to_datetime(arc.mjd_start),
            arc_end=mjd_to_datetime(arc.mjd_end),
            pride_wl_float=arc.wl_amb,
            pride_sigma_wl=arc.sigma_wl,
            pride_wl_int=arc.wl_int,
            pride_confident=arc.confident_int,
            engine_events_in_arc=len(in_arc),
            engine_n_wl_first=e_first.n_wl if e_first else None,
            engine_n_wl_last=e_last.n_wl if e_last else None,
            cycles_diff_first=cycles_first,
            cycles_diff_last=cycles_last,
            pos_bias_first_m=bias_first,
            pos_bias_last_m=bias_last,
            elev=arc.elev,
        ))
    return out


# ── CLI / output ─────────────────────────────────────────────────── #


def print_table(rows: list[Comparison]) -> None:
    print(f"{'sv':>4}  {'arc_start':>20}  {'arc_end':>9}  "
          f"{'pride_wl':>10}  {'σ':>5}  {'int':>5}  "
          f"{'eng_n':>5}  {'Δcyc':>5}  {'bias_m':>7}  {'elev':>5}  conf")
    print('-' * 110)
    for r in sorted(rows, key=lambda x: (x.sv, x.arc_start)):
        eng_n = (str(r.engine_n_wl_last)
                  if r.engine_n_wl_last is not None else '-')
        dcyc = (f"{r.cycles_diff_last:+d}"
                if r.cycles_diff_last is not None else '-')
        bias = (f"{r.pos_bias_last_m:+7.2f}"
                 if r.pos_bias_last_m is not None else '       -')
        conf = 'PRIDE' if r.pride_confident else '???'
        print(f"{r.sv:>4}  "
              f"{r.arc_start.strftime('%Y-%m-%d %H:%M:%S'):>20}  "
              f"{r.arc_end.strftime('%H:%M:%S'):>9}  "
              f"{r.pride_wl_float:+10.4f}  "
              f"{r.pride_sigma_wl:5.3f}  "
              f"{r.pride_wl_int:>+5d}  "
              f"{eng_n:>5}  {dcyc:>5}  {bias:>7}  "
              f"{r.elev:5.1f}  {conf}")


def print_summary(rows: list[Comparison]) -> None:
    confident = [r for r in rows if r.pride_confident]
    matched = [r for r in confident if r.cycles_diff_last is not None]
    print()
    print(f"PRIDE arcs total:                 {len(rows)}")
    print(f"  confident (round-able):         {len(confident)}")
    print(f"  with engine WL_FIX in arc:      {len(matched)}")
    if matched:
        agreed = sum(1 for r in matched if r.cycles_diff_last == 0)
        disagreed = len(matched) - agreed
        biases = [abs(r.pos_bias_last_m) for r in matched
                  if r.pos_bias_last_m is not None]
        print(f"  engine integer == PRIDE:        {agreed}")
        print(f"  engine integer != PRIDE:        {disagreed}")
        if biases:
            print(f"  median |Δcyc × λ_WL|:           "
                  f"{sorted(biases)[len(biases)//2]:.2f} m")
            print(f"  max    |Δcyc × λ_WL|:           "
                  f"{max(biases):.2f} m")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--engine-log', required=True, type=Path)
    ap.add_argument('--pride-amb', required=True, type=Path)
    ap.add_argument('--csv', type=Path,
                    help='optional CSV output of comparison rows')
    args = ap.parse_args()

    arcs = parse_pride_amb(args.pride_amb)
    events = parse_engine_log(args.engine_log)
    print(f"Parsed {len(arcs)} PRIDE arcs from {args.pride_amb}")
    print(f"Parsed {len(events)} WL_FIX_LIFE events from {args.engine_log}")
    comparisons = compare(arcs, events)

    print_table(comparisons)
    print_summary(comparisons)

    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['sv', 'arc_start_utc', 'arc_end_utc',
                        'pride_wl_float', 'pride_sigma_wl', 'pride_wl_int',
                        'pride_confident', 'engine_events_in_arc',
                        'engine_n_wl_first', 'engine_n_wl_last',
                        'cycles_diff_first', 'cycles_diff_last',
                        'pos_bias_first_m', 'pos_bias_last_m', 'elev_deg'])
            for r in comparisons:
                w.writerow([
                    r.sv,
                    r.arc_start.isoformat(),
                    r.arc_end.isoformat(),
                    f"{r.pride_wl_float:.6f}",
                    f"{r.pride_sigma_wl:.4f}",
                    r.pride_wl_int,
                    r.pride_confident,
                    r.engine_events_in_arc,
                    r.engine_n_wl_first if r.engine_n_wl_first is not None else '',
                    r.engine_n_wl_last if r.engine_n_wl_last is not None else '',
                    r.cycles_diff_first if r.cycles_diff_first is not None else '',
                    r.cycles_diff_last if r.cycles_diff_last is not None else '',
                    f"{r.pos_bias_first_m:.3f}" if r.pos_bias_first_m is not None else '',
                    f"{r.pos_bias_last_m:.3f}" if r.pos_bias_last_m is not None else '',
                    f"{r.elev:.1f}",
                ])
        print(f"\nCSV written to {args.csv}")
    return 0


if __name__ == '__main__':
    sys.exit(main())

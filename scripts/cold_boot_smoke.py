#!/usr/bin/env python3
"""Cold-boot smoke loop — does the engine reliably converge close to truth?

Drives ``peppar-fix`` in a loop on the local host.  Per iteration:

  1. PID-targeted kill any running engine.
  2. Stash receiver state files (``state/receivers/*.json``).
  3. Launch ``./scripts/peppar-fix`` in background; capture log.
  4. Wait for ``=== Phase 2: Steady state`` log line (Phase 1 convergence).
  5. Continue tailing for ``settle_s`` after Phase 2 entry.
  6. Kill engine.
  7. Parse log; emit one JSON record per run with the headline metrics
     against the supplied truth LLA.

Pass criterion (per run, with default tolerances; override via flags):

  - Phase 1 converged                                   yes/no
  - Phase 2 entry position within --p2-tol-m of truth   yes/no
  - Position at +settle_s within --settle-tol-m         yes/no
  - σ at +settle_s below --settle-sigma-m               yes/no
  - Zero SO_POS / integrity trips in the settle window  yes/no

A run passes only if all five hold.

Per run state-machine timeline visible in JSON: Phase-1 method
(NAV2 / LS-init / args.seed_pos), Phase-1 σ at convergence, nav2_h
at convergence, Phase-2 entry position, +settle_s position + σ,
trips during settle.

Usage (on the lab host, in ~/peppar-fix):

  python3 scripts/cold_boot_smoke.py --runs 10 \\
    --truth-lla LAT,LON,ALT \\
    --out data/cold_boot_smoke_$(date +%Y%m%d_%H%M%S).json

(LAT,LON,ALT from timelab/antennas.json or surveys/, never hardcoded
in the repo.  Lab usage: source the value from antennas.json at the
host's repo root.)
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path


_RE_PHASE1_HEAD = re.compile(r'=== Phase 1: Position bootstrap')
_RE_PPP_INIT = re.compile(
    r'PPPFilter initialized via (?P<source>[^,]+),\s*σ_pos=(?P<sigma>[\d.]+)m'
)
_RE_CONVERGED = re.compile(
    r'CONVERGED at epoch (?P<ep>\d+)\s+\(σ=(?P<sigma>[\d.]+)m,'
    r'.*?nav2_h=(?P<nav2_h>[\d.]+)m,.*?gate=(?P<gate>[^)]+)\)'
)
_RE_PHASE2_HEAD = re.compile(r'=== Phase 2: Steady state')
_RE_ANTPOS = re.compile(
    r'\[AntPosEst\s+(?P<epoch>\d+)\]\s+'
    r'positionσ=(?P<sigma>[\d.]+)m\s+'
    r'pos=\(\s*(?P<lat>[-\d.]+),\s*(?P<lon>[-\d.]+),\s*(?P<alt>[-\d.]+)\s*\)'
    r'.*?nav2Δ=(?P<nav2d>[\d.]+)m'
)
_RE_TIME_RAW = re.compile(r'^(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d)')
_RE_TRIP = re.compile(r'\[FIX_SET_INTEGRITY\] TRIPPED|\[SECOND_OPINION_POS\] tripped')


def lla_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> tuple[float, float, float]:
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2 - f)
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    N = a / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
    x = (N + alt_m) * math.cos(lat) * math.cos(lon)
    y = (N + alt_m) * math.cos(lat) * math.sin(lon)
    z = (N * (1 - e2) + alt_m) * math.sin(lat)
    return (x, y, z)


def ecef_distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


@dataclass
class RunResult:
    run: int
    started_at: str
    log: str
    phase1_seed_source: str | None = None
    phase1_seed_sigma_m: float | None = None
    phase1_converged: bool = False
    phase1_sigma_m: float | None = None
    phase1_nav2_h_m: float | None = None
    phase1_gate: str | None = None
    phase2_entered_s: float | None = None
    phase2_entry_pos_3d_m: float | None = None
    settle_pos_3d_m: float | None = None
    settle_sigma_m: float | None = None
    trips_during_settle: int = 0
    trip_reasons: list[str] = field(default_factory=list)
    pass_phase1: bool = False
    pass_phase2_entry: bool = False
    pass_settle_pos: bool = False
    pass_settle_sigma: bool = False
    pass_no_trips: bool = False
    passed: bool = False
    error: str | None = None


def kill_engine() -> None:
    """PID-targeted kill via ps + grep."""
    try:
        out = subprocess.run(
            ['pgrep', '-f', 'venv/bin/python.*peppar_fix_engine'],
            capture_output=True, text=True, timeout=5,
        )
        for pid in out.stdout.split():
            try:
                os.kill(int(pid), signal.SIGTERM)
            except (ProcessLookupError, ValueError):
                pass
    except subprocess.TimeoutExpired:
        pass
    time.sleep(2)
    try:
        out = subprocess.run(
            ['pgrep', '-f', 'venv/bin/python.*peppar_fix_engine'],
            capture_output=True, text=True, timeout=5,
        )
        for pid in out.stdout.split():
            try:
                os.kill(int(pid), signal.SIGKILL)
            except (ProcessLookupError, ValueError):
                pass
    except subprocess.TimeoutExpired:
        pass


def stash_state(repo: Path) -> None:
    """Move state/receivers/*.json out of the way so engine cold-starts."""
    rcv_dir = repo / 'state' / 'receivers'
    if not rcv_dir.exists():
        return
    stamp = time.strftime('%Y%m%d_%H%M%S')
    for f in rcv_dir.glob('*.json'):
        # Don't re-stash already-stashed files.
        if '.bak' in f.name:
            continue
        target = f.with_suffix(f.suffix + f'.cold_smoke_{stamp}.bak')
        f.rename(target)


def launch_engine(repo: Path, log_path: Path) -> subprocess.Popen:
    """Launch peppar-fix; return Popen handle."""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_fp = log_path.open('w')
    proc = subprocess.Popen(
        [str(repo / 'scripts' / 'peppar-fix')],
        stdout=log_fp, stderr=subprocess.STDOUT,
        cwd=str(repo),
        start_new_session=True,
    )
    return proc


def parse_log(log_path: Path, truth_ecef: tuple[float, float, float],
              settle_s: float) -> dict:
    """Parse engine log to extract per-stage metrics."""
    out = {}
    phase2_entry_ts = None
    settle_pos = None
    settle_sigma = None
    settle_nav2d = None
    trip_count = 0
    trip_reasons = []

    if not log_path.exists():
        return out

    for raw in log_path.read_text(errors='replace').splitlines():
        m = _RE_TIME_RAW.match(raw)
        if not m:
            continue
        ts_s = time.mktime(time.strptime(m.group(1), '%Y-%m-%d %H:%M:%S'))

        m = _RE_PPP_INIT.search(raw)
        if m:
            out['phase1_seed_source'] = m.group('source').strip()
            out['phase1_seed_sigma_m'] = float(m.group('sigma'))
            continue

        m = _RE_CONVERGED.search(raw)
        if m:
            out['phase1_converged'] = True
            out['phase1_sigma_m'] = float(m.group('sigma'))
            out['phase1_nav2_h_m'] = float(m.group('nav2_h'))
            out['phase1_gate'] = m.group('gate').strip()
            continue

        if _RE_PHASE2_HEAD.search(raw):
            phase2_entry_ts = ts_s
            out['_phase2_entry_ts'] = ts_s
            continue

        m = _RE_ANTPOS.search(raw)
        if m and phase2_entry_ts is not None:
            pos = (float(m.group('lat')), float(m.group('lon')),
                   float(m.group('alt')))
            ecef = lla_to_ecef(*pos)
            d = ecef_distance(ecef, truth_ecef)
            sigma = float(m.group('sigma'))
            nav2d = float(m.group('nav2d'))

            if 'phase2_entry_pos_3d_m' not in out:
                out['phase2_entry_pos_3d_m'] = d
                out['_phase2_entry_pos'] = pos
            if ts_s - phase2_entry_ts <= settle_s:
                # Update with the most-recent within-window sample.
                settle_pos = d
                settle_sigma = sigma
                settle_nav2d = nav2d

        if phase2_entry_ts is not None and _RE_TRIP.search(raw):
            if ts_s - phase2_entry_ts <= settle_s:
                trip_count += 1
                trip_reasons.append(raw.strip().split()[-1])

    if settle_pos is not None:
        out['settle_pos_3d_m'] = settle_pos
        out['settle_sigma_m'] = settle_sigma
        out['settle_nav2d_m'] = settle_nav2d
    out['trips_during_settle'] = trip_count
    out['trip_reasons'] = trip_reasons
    return out


def evaluate(metrics: dict, args: argparse.Namespace) -> dict:
    """Apply pass/fail tolerances."""
    p1 = bool(metrics.get('phase1_converged'))
    p2_entry = (
        metrics.get('phase2_entry_pos_3d_m') is not None
        and metrics['phase2_entry_pos_3d_m'] < args.p2_tol_m
    )
    settle_pos_ok = (
        metrics.get('settle_pos_3d_m') is not None
        and metrics['settle_pos_3d_m'] < args.settle_tol_m
    )
    settle_sigma_ok = (
        metrics.get('settle_sigma_m') is not None
        and metrics['settle_sigma_m'] < args.settle_sigma_m
    )
    no_trips = metrics.get('trips_during_settle', 0) == 0

    return {
        'pass_phase1': p1,
        'pass_phase2_entry': p2_entry,
        'pass_settle_pos': settle_pos_ok,
        'pass_settle_sigma': settle_sigma_ok,
        'pass_no_trips': no_trips,
        'passed': all([p1, p2_entry, settle_pos_ok, settle_sigma_ok, no_trips]),
    }


def run_one(args: argparse.Namespace, run_idx: int,
            truth_ecef: tuple[float, float, float]) -> RunResult:
    repo = Path(args.repo).expanduser().resolve()
    stamp = time.strftime('%Y%m%d_%H%M%S')
    log_path = repo / 'data' / f'cold_smoke_run{run_idx:02d}_{stamp}.log'

    print(f"--- run {run_idx}/{args.runs} ---", flush=True)
    kill_engine()
    stash_state(repo)
    print(f"  launching, log={log_path.name}", flush=True)
    proc = launch_engine(repo, log_path)

    # Wait for Phase 2 entry, then settle window.
    deadline = time.time() + args.timeout_s
    phase2_at = None
    while time.time() < deadline:
        if phase2_at is None:
            text = log_path.read_text(errors='replace') if log_path.exists() else ''
            if _RE_PHASE2_HEAD.search(text):
                phase2_at = time.time()
                print(f"  Phase 2 entered at +{phase2_at - (deadline - args.timeout_s):.1f}s",
                      flush=True)
        else:
            if time.time() - phase2_at >= args.settle_s:
                break
        if proc.poll() is not None:
            print("  engine exited prematurely", flush=True)
            break
        time.sleep(2)

    print("  killing engine", flush=True)
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    kill_engine()

    metrics = parse_log(log_path, truth_ecef, args.settle_s)
    verdict = evaluate(metrics, args)

    result = RunResult(
        run=run_idx,
        started_at=stamp,
        log=str(log_path.name),
        phase1_seed_source=metrics.get('phase1_seed_source'),
        phase1_seed_sigma_m=metrics.get('phase1_seed_sigma_m'),
        phase1_converged=bool(metrics.get('phase1_converged')),
        phase1_sigma_m=metrics.get('phase1_sigma_m'),
        phase1_nav2_h_m=metrics.get('phase1_nav2_h_m'),
        phase1_gate=metrics.get('phase1_gate'),
        phase2_entered_s=(
            metrics['_phase2_entry_ts'] - (deadline - args.timeout_s)
            if metrics.get('_phase2_entry_ts') and phase2_at is not None
            else None),
        phase2_entry_pos_3d_m=metrics.get('phase2_entry_pos_3d_m'),
        settle_pos_3d_m=metrics.get('settle_pos_3d_m'),
        settle_sigma_m=metrics.get('settle_sigma_m'),
        trips_during_settle=metrics.get('trips_during_settle', 0),
        trip_reasons=metrics.get('trip_reasons', []),
        **verdict,
    )

    print(f"  → seed={result.phase1_seed_source}, "
          f"phase2_entry={result.phase2_entry_pos_3d_m}m, "
          f"settle={result.settle_pos_3d_m}m, "
          f"σ={result.settle_sigma_m}m, "
          f"trips={result.trips_during_settle}, "
          f"PASSED={result.passed}",
          flush=True)
    return result


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--repo', default=str(Path.home() / 'peppar-fix'),
                   help='peppar-fix repo root (default ~/peppar-fix)')
    p.add_argument('--runs', type=int, default=10,
                   help='number of cold boots to run (default 10)')
    p.add_argument('--truth-lla', required=True,
                   help='ground-truth ARP as lat,lon,alt_m')
    p.add_argument('--timeout-s', type=int, default=300,
                   help='per-run hard timeout (default 300 s)')
    p.add_argument('--settle-s', type=float, default=60.0,
                   help='settle window after Phase 2 entry (default 60 s)')
    p.add_argument('--p2-tol-m', type=float, default=3.0,
                   help='Phase 2 entry tolerance (default 3 m)')
    p.add_argument('--settle-tol-m', type=float, default=1.0,
                   help='+settle_s position tolerance (default 1 m)')
    p.add_argument('--settle-sigma-m', type=float, default=0.5,
                   help='+settle_s σ tolerance (default 0.5 m)')
    p.add_argument('--out', default=None,
                   help='output JSON path (default: stdout-only)')
    args = p.parse_args(argv)

    lat, lon, alt = (float(v) for v in args.truth_lla.split(','))
    truth_ecef = lla_to_ecef(lat, lon, alt)

    results: list[dict] = []
    for i in range(1, args.runs + 1):
        try:
            r = run_one(args, i, truth_ecef)
            results.append(asdict(r))
        except Exception as e:
            print(f"  ERROR: {e}", flush=True)
            results.append({'run': i, 'error': str(e)})

    summary = {
        'truth_lla': [lat, lon, alt],
        'truth_ecef': list(truth_ecef),
        'tolerances': {
            'p2_tol_m': args.p2_tol_m,
            'settle_tol_m': args.settle_tol_m,
            'settle_sigma_m': args.settle_sigma_m,
            'settle_s': args.settle_s,
        },
        'runs': results,
        'pass_count': sum(1 for r in results if r.get('passed')),
        'total': len(results),
    }
    payload = json.dumps(summary, indent=2, default=str)
    if args.out:
        Path(args.out).write_text(payload)
        print(f"\nWrote {args.out}", flush=True)
    print(f"\nPASS: {summary['pass_count']}/{summary['total']}")
    return 0


if __name__ == '__main__':
    sys.exit(main())

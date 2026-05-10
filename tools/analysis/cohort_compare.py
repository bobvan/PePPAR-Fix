#!/usr/bin/env python3
"""cohort_compare.py — quantitative cohort overnight comparison.

Reads peppar_fix_engine logs from multiple lab hosts and produces a
per-host summary plus a cross-host comparison rubric.  Intended to
replace the morning grep-by-hand pass with a structured 2-minute
review.

Usage:
    python3 tools/analysis/cohort_compare.py \\
        host1=/path/to/host1-overnight.log \\
        host2=/path/to/host2-overnight.log \\
        host3=/path/to/host3-overnight.log \\
        [--truth-ecef X,Y,Z]      # surveyed ARP truth, optional
        [--out cohort.md]         # write markdown report

The comparison axes:

  Time-side health:
    σ_dt_rx        median, p95, max
    FIXEDPOS_ZTD   median, p95, max excursion from METAR baseline
    Catastrophic / FIX_SET_ALARM / Tracebacks (must be 0)

  Position-side health:
    AntPosEst σ_pos   median, convergence trajectory
    nav2Δ             median, peak
    ZTD-trip count    SECOND_OPINION_POS, FIX_SET_INTEGRITY
    Position offset from truth (if --truth-ecef given)

  Blend health:
    σ_pin trajectory
    Blend update count vs reject count
    σ_pin widen events (Stage 6 self-healing)

  Slip detector (if bravo's fix-I deployed):
    Total flushes, mw-jump-only count

  BDS phase-bias:
    [PB_APPLIED] BDS MISS event count

Pass/fail rubric per host:
    σ_dt_rx p95 < 1.0 ns           [time-side budget]
    FIXEDPOS_ZTD median < 200 mm  [METAR-baseline tracking]
    Catastrophic / FIX_SET_ALARM = 0
    Tracebacks = 0

Output: stdout summary plus optional markdown file for archival.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, field, asdict
from pathlib import Path
from statistics import median, quantiles
from typing import Any


# Regex patterns lifted from engine log lines.  Centralised here so
# log-format changes only need updating in one place.

# [STATUS] AntPosEst=converging(σ=0.10m, 8 WL, 0 NL) DOFreqEst=tracking(adj=+941.7ppb, err=+0.4ns, ...)
_RE_STATUS = re.compile(
    r"\[STATUS\] AntPosEst=\w+\(σ=([\d.]+)m,\s*"
    r"(\d+) WL,\s*(\d+) NL\)\s*"
    r"DOFreqEst=\w+\(adj=([+-][\d.]+)ppb,\s*"
    r"err=([+-][\d.]+)ns")

# [FIXEDPOS_ZTD] epoch=300 ZTD=+233±7mm dt_rx=-10247834.990ns σ=0.0937ns
_RE_FIXEDPOS_ZTD = re.compile(
    r"\[FIXEDPOS_ZTD\] epoch=(\d+) ZTD=([+-]?\d+)±\d+mm\s+"
    r"dt_rx=([+-][\d.]+)ns σ=([\d.]+)ns")

# [AntPosEst NNNN] positionσ=X.XXXm pos=(LAT, LON, ALT) ... nav2Δ=Y.Ym ZTD=±ZZZmm
_RE_ANTPOSEST = re.compile(
    r"\[AntPosEst (\d+)\] positionσ=([\d.]+)m\s+pos="
    r"\(([\d.-]+),\s*([\d.-]+),\s*([\d.-]+)\)\s+"
    r".*?nav2Δ=([\d.]+)m\s+ZTD=([+-]?\d+)")

# Source labels appear in blend log lines: "NAV2 blend:" / "AntPosEst blend:"
# also "NAV2 blend outlier rejected" / "AntPosEst blend outlier rejected"
_RE_BLEND_APPLIED = re.compile(
    r"(NAV2|AntPosEst) blend: Δ=([\d.]+)m α=([\d.]+) "
    r"step=([\d.]+)mm σ_pin=([\d.]+)m σ_src=([\d.]+)m")
_RE_BLEND_REJECTED = re.compile(
    r"(NAV2|AntPosEst) blend outlier rejected: "
    r"Δ=([\d.]+)m > [\d.]+σ=([\d.]+)m")
_RE_BLEND_WIDEN = re.compile(
    r"(NAV2|AntPosEst) blend: \d+ consecutive rejects "
    r"— widening σ_pin ([\d.]+) → ([\d.]+) m")

_RE_SLIP_FLUSH = re.compile(r"cycle slip flush: sv=(\S+) .*reason=(\S+)")
_RE_SECOND_OPINION = re.compile(
    r"\[SECOND_OPINION_POS\] tripped: nav2Δ=([\d.]+)m")
_RE_FIX_SET_ALARM = re.compile(r"FIX_SET_(?:INTEGRITY|ALARM)")
_RE_CATASTROPHIC = re.compile(r"CATASTROPHIC[_ ]REJECT")
_RE_TRACEBACK = re.compile(r"Traceback|Exception")
_RE_BDS_MISS = re.compile(r"\[PB_APPLIED\] C\d+ .*BDS-B2aI.*MISSm")
_RE_AR_MODE = re.compile(r"AR mode:\s+(\w+)")
_RE_BLEND_SOURCE = re.compile(
    r"--position-blend-source[= ](\w+)|"
    r"σ_pin initial:\s+[\d.]+\s+m\s+\(source=(\S+)")
_RE_PIN_INITIAL = re.compile(r"σ_pin initial:\s+([\d.]+)\s*m")


@dataclass
class HostStats:
    """Aggregate metrics for one host."""
    name: str
    log_path: str
    n_lines: int = 0
    ar_mode: str | None = None
    blend_source: str | None = None
    sigma_pin_init_m: float | None = None

    # Time-side
    fixedpos_ztd_mm: list[int] = field(default_factory=list)
    fixedpos_dt_rx_sigma_ns: list[float] = field(default_factory=list)
    n_fixedpos_lines: int = 0

    # Position-side
    antposest_sigma_m: list[float] = field(default_factory=list)
    antposest_nav2delta_m: list[float] = field(default_factory=list)
    antposest_ztd_mm: list[int] = field(default_factory=list)
    antposest_lat: list[float] = field(default_factory=list)
    antposest_lon: list[float] = field(default_factory=list)
    antposest_alt: list[float] = field(default_factory=list)

    # Status (DOFreqEst)
    dofreq_err_ns: list[float] = field(default_factory=list)
    dofreq_adj_ppb: list[float] = field(default_factory=list)

    # Blend
    blend_applied: int = 0
    blend_rejected: int = 0
    blend_widen: int = 0

    # Slip detector
    slip_flushes: int = 0
    slip_mwjump_only: int = 0

    # Alarms
    n_second_opinion: int = 0
    n_fix_set_alarm: int = 0
    n_catastrophic: int = 0
    n_traceback: int = 0
    n_bds_miss: int = 0


def parse_log(path: Path, host_name: str) -> HostStats:
    """Single-pass scan of an engine log file."""
    s = HostStats(name=host_name, log_path=str(path))
    with path.open() as f:
        for line in f:
            s.n_lines += 1

            if s.ar_mode is None:
                m = _RE_AR_MODE.search(line)
                if m:
                    s.ar_mode = m.group(1)

            if s.sigma_pin_init_m is None:
                m = _RE_PIN_INITIAL.search(line)
                if m:
                    s.sigma_pin_init_m = float(m.group(1))

            m = _RE_FIXEDPOS_ZTD.search(line)
            if m:
                s.n_fixedpos_lines += 1
                s.fixedpos_ztd_mm.append(int(m.group(2)))
                s.fixedpos_dt_rx_sigma_ns.append(float(m.group(4)))
                continue

            m = _RE_ANTPOSEST.search(line)
            if m:
                s.antposest_sigma_m.append(float(m.group(2)))
                s.antposest_lat.append(float(m.group(3)))
                s.antposest_lon.append(float(m.group(4)))
                s.antposest_alt.append(float(m.group(5)))
                s.antposest_nav2delta_m.append(float(m.group(6)))
                s.antposest_ztd_mm.append(int(m.group(7)))
                continue

            m = _RE_STATUS.search(line)
            if m:
                s.dofreq_err_ns.append(abs(float(m.group(5))))
                s.dofreq_adj_ppb.append(float(m.group(4)))
                continue

            if _RE_BLEND_APPLIED.search(line):
                s.blend_applied += 1
                continue
            if _RE_BLEND_REJECTED.search(line):
                s.blend_rejected += 1
                continue
            if _RE_BLEND_WIDEN.search(line):
                s.blend_widen += 1
                continue

            m = _RE_SLIP_FLUSH.search(line)
            if m:
                s.slip_flushes += 1
                if m.group(2) == "mw_jump":
                    s.slip_mwjump_only += 1
                continue

            if _RE_SECOND_OPINION.search(line):
                s.n_second_opinion += 1
                continue
            if _RE_CATASTROPHIC.search(line):
                s.n_catastrophic += 1
                continue
            if _RE_FIX_SET_ALARM.search(line):
                s.n_fix_set_alarm += 1
                continue
            if _RE_TRACEBACK.search(line):
                s.n_traceback += 1
                continue
            if _RE_BDS_MISS.search(line):
                s.n_bds_miss += 1
                continue

    return s


def _stat_summary(values: list[float], label: str) -> str:
    """median / p95 / max with units stripped."""
    if not values:
        return f"  {label:35s} (no data)"
    med = median(values)
    if len(values) >= 20:
        p95 = quantiles(values, n=20)[18]  # 95th percentile
    else:
        p95 = max(values)
    mx = max(values)
    return f"  {label:35s} median={med:.3f}  p95={p95:.3f}  max={mx:.3f}"


def _ll_to_meters(lat1: float, lon1: float, alt1: float,
                  lat0: float, lon0: float, alt0: float) -> tuple[float, float, float]:
    """Convert (lat, lon, alt) offsets to (E, N, U) metres at lat0."""
    import math
    dlat = lat1 - lat0
    dlon = lon1 - lon0
    dalt = alt1 - alt0
    n_m = dlat * 111320.0
    e_m = dlon * 111320.0 * math.cos(math.radians(lat0))
    return e_m, n_m, dalt


def _eval_pass_fail(s: HostStats) -> tuple[list[str], list[str]]:
    """Return (passes, fails) string lists."""
    passes, fails = [], []

    # Time-side health
    if s.fixedpos_dt_rx_sigma_ns:
        p95 = (quantiles(s.fixedpos_dt_rx_sigma_ns, n=20)[18]
               if len(s.fixedpos_dt_rx_sigma_ns) >= 20
               else max(s.fixedpos_dt_rx_sigma_ns))
        if p95 < 1.0:
            passes.append(f"σ_dt_rx p95={p95:.3f}ns < 1.0ns")
        else:
            fails.append(f"σ_dt_rx p95={p95:.3f}ns ≥ 1.0ns")
    else:
        fails.append("no FIXEDPOS_ZTD lines")

    if s.fixedpos_ztd_mm:
        med = abs(median(s.fixedpos_ztd_mm))
        if med < 200:
            passes.append(f"FIXEDPOS_ZTD |median|={med:.0f}mm < 200mm")
        else:
            fails.append(f"FIXEDPOS_ZTD |median|={med:.0f}mm ≥ 200mm")

    if s.n_catastrophic == 0:
        passes.append("0 catastrophic rejects")
    else:
        fails.append(f"{s.n_catastrophic} catastrophic rejects")

    if s.n_fix_set_alarm == 0:
        passes.append("0 FIX_SET_ALARMs")
    else:
        fails.append(f"{s.n_fix_set_alarm} FIX_SET_ALARMs")

    if s.n_traceback == 0:
        passes.append("0 tracebacks")
    else:
        fails.append(f"{s.n_traceback} tracebacks")

    return passes, fails


def render_host(s: HostStats, truth_ecef: tuple[float, float, float] | None) -> str:
    """Per-host markdown block."""
    lines = []
    lines.append(f"## {s.name}")
    lines.append("")
    lines.append(f"**Log**: `{s.log_path}` ({s.n_lines:,} lines)")
    cfg = []
    if s.ar_mode: cfg.append(f"--ar-mode {s.ar_mode}")
    if s.blend_source: cfg.append(f"source={s.blend_source}")
    if s.sigma_pin_init_m is not None:
        cfg.append(f"σ_pin_init={s.sigma_pin_init_m:.3f}m")
    if cfg:
        lines.append(f"**Config**: " + ", ".join(cfg))
    lines.append("")

    lines.append("### Time-side")
    lines.append("```")
    lines.append(_stat_summary(s.fixedpos_dt_rx_sigma_ns,
                               "σ_dt_rx (ns)"))
    if s.fixedpos_ztd_mm:
        lines.append(_stat_summary(
            [abs(z) for z in s.fixedpos_ztd_mm],
            "|FIXEDPOS_ZTD| (mm)"))
    lines.append(_stat_summary(s.dofreq_err_ns,
                               "|DOFreqEst err| (ns)"))
    lines.append("```")
    lines.append("")

    lines.append("### Position-side")
    lines.append("```")
    lines.append(_stat_summary(s.antposest_sigma_m,
                               "AntPosEst σ_pos (m)"))
    lines.append(_stat_summary(s.antposest_nav2delta_m,
                               "AntPosEst nav2Δ (m)"))
    if s.antposest_ztd_mm:
        lines.append(_stat_summary(
            [abs(z) for z in s.antposest_ztd_mm],
            "|AntPosEst ZTD| (mm)"))
    if truth_ecef and s.antposest_lat:
        # Mean position vs truth
        from statistics import mean
        avg_lat = mean(s.antposest_lat)
        avg_lon = mean(s.antposest_lon)
        avg_alt = mean(s.antposest_alt)
        # Convert truth_ecef to LLA
        # (skip; just compare to operator-supplied lat/lon/alt)
        lines.append(f"  AntPosEst mean pos                 "
                     f"({avg_lat:.7f}, {avg_lon:.7f}, {avg_alt:.2f})")
    lines.append("```")
    lines.append("")

    lines.append("### Blend (Stage 5/6)")
    lines.append(f"- applied: {s.blend_applied}")
    lines.append(f"- rejected: {s.blend_rejected}")
    lines.append(f"- σ_pin widen events (Stage 6 self-heal): "
                 f"{s.blend_widen}")
    lines.append("")

    lines.append("### Slip detector (bravo fix-I)")
    lines.append(f"- total flushes: {s.slip_flushes}")
    lines.append(f"- mw_jump-only (false positives): "
                 f"{s.slip_mwjump_only}")
    if s.slip_flushes > 0:
        pct = 100.0 * s.slip_mwjump_only / s.slip_flushes
        lines.append(f"- mw_jump-only rate: {pct:.1f}%")
    lines.append("")

    lines.append("### Alarms")
    lines.append(f"- SECOND_OPINION_POS trips: {s.n_second_opinion}")
    lines.append(f"- FIX_SET_ALARM: {s.n_fix_set_alarm}")
    lines.append(f"- catastrophic rejects: {s.n_catastrophic}")
    lines.append(f"- tracebacks: {s.n_traceback}")
    lines.append(f"- BDS PB MISS events: {s.n_bds_miss}")
    lines.append("")

    passes, fails = _eval_pass_fail(s)
    if not fails:
        lines.append("**VERDICT: PASS** ✅")
    else:
        lines.append("**VERDICT: FAIL** ❌")
    lines.append("")
    if passes:
        lines.append("Passes:")
        for p in passes:
            lines.append(f"- ✅ {p}")
    if fails:
        lines.append("Fails:")
        for f in fails:
            lines.append(f"- ❌ {f}")
    lines.append("")
    return "\n".join(lines)


def render_cohort(host_stats: list[HostStats]) -> str:
    """Cross-host comparison table."""
    lines = []
    lines.append("## Cohort comparison")
    lines.append("")
    lines.append("| Host | AR mode | source | σ_dt_rx p95 ns | "
                 "|FIXEDPOS_ZTD| med mm | AntPosEst σ med m | "
                 "SECOND_OPINION | catastrophic | verdict |")
    lines.append("|---|---|---|---|---|---|---|---|---|")
    for s in host_stats:
        if s.fixedpos_dt_rx_sigma_ns:
            p95 = (quantiles(s.fixedpos_dt_rx_sigma_ns, n=20)[18]
                   if len(s.fixedpos_dt_rx_sigma_ns) >= 20
                   else max(s.fixedpos_dt_rx_sigma_ns))
            sigma_str = f"{p95:.3f}"
        else:
            sigma_str = "—"
        ztd_str = (f"{abs(median(s.fixedpos_ztd_mm)):.0f}"
                   if s.fixedpos_ztd_mm else "—")
        ape_sigma_str = (f"{median(s.antposest_sigma_m):.3f}"
                         if s.antposest_sigma_m else "—")
        passes, fails = _eval_pass_fail(s)
        verdict = "✅" if not fails else "❌"
        lines.append(
            f"| {s.name} | {s.ar_mode or '?'} | {s.blend_source or '?'} | "
            f"{sigma_str} | {ztd_str} | {ape_sigma_str} | "
            f"{s.n_second_opinion} | {s.n_catastrophic} | {verdict} |")
    lines.append("")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(
        description="Compare engine logs across lab cohort hosts.",
    )
    ap.add_argument("hosts", nargs="+",
                    help="host=path/to/log entries")
    ap.add_argument("--truth-ecef", default=None,
                    help="Surveyed ARP truth as 'X,Y,Z' ECEF metres")
    ap.add_argument("--out", default=None,
                    help="Write markdown report to this file")
    args = ap.parse_args()

    truth_ecef = None
    if args.truth_ecef:
        truth_ecef = tuple(float(v) for v in args.truth_ecef.split(","))

    host_stats = []
    for spec in args.hosts:
        if "=" not in spec:
            print(f"error: '{spec}' must be host=path", file=sys.stderr)
            return 1
        name, path = spec.split("=", 1)
        p = Path(path)
        if not p.exists():
            print(f"error: {path} not found", file=sys.stderr)
            return 1
        s = parse_log(p, name)
        host_stats.append(s)

    out_lines = []
    out_lines.append("# Cohort overnight comparison")
    out_lines.append("")
    out_lines.append(f"Generated by `cohort_compare.py` on "
                     f"{Path(__file__).parent.parent.parent.name}")
    out_lines.append("")
    out_lines.append(render_cohort(host_stats))
    for s in host_stats:
        out_lines.append(render_host(s, truth_ecef))

    output = "\n".join(out_lines)
    print(output)

    if args.out:
        Path(args.out).write_text(output)
        print(f"\n[written to {args.out}]", file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(main())

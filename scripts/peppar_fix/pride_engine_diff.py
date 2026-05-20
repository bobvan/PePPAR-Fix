"""Epoch-by-epoch reconciliation: PRIDE kinematic kin_* vs engine
[AntPosEst N] log lines.

Why this exists (prideEpochDiff-charlie, 2026-05-20):
positionBiasFilterSideEvidence-charlie showed PRIDE on shared-antenna
captures agrees cross-host to 15.9 cm while the engine's AntPosEst
diverged by 4.6 m on the same hosts.  That localized the bias to the
filter side.  This module is the next-level bisection tool: align
PRIDE's per-epoch kin solution with the engine's per-AntPosEst-N log
sample, compute Δ3D, and emit a CSV that lets us identify the EPOCH
at which the engine started diverging — plus what engine events
fired in that window (FALSE_FIX, ANCHORING, FIXED_LAMBDA, IF_STEP,
GF_STEP, slip events, etc.).

The output CSV makes "the engine diverged at some point" actionable
into "at epoch N, IF_STEP unfixed G15 and the running mean jumped
0.4 m horizontal."

Consumed by tools/pride_engine_diff_cli.py (or run as
``python -m peppar_fix.pride_engine_diff``).
"""
from __future__ import annotations

import csv
import logging
import math
import re
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Iterable

log = logging.getLogger("pride_engine_diff")


# ── PRIDE kin file parsing ──────────────────────────────────────── #

# MJD epoch: 1858-11-17 00:00:00 UTC.  Add (mjd-40587)*86400 to UNIX epoch.
_MJD_UNIX_OFFSET = 40587


@dataclass
class KinEpoch:
    """One per-epoch position from a PRIDE kin_* file."""
    mjd: int
    sod: float            # Seconds of day (GPS time)
    x: float              # ECEF (meters)
    y: float
    z: float
    lat: float            # WGS-84 degrees
    lon: float
    height: float         # WGS-84 ellipsoidal height (meters)
    pdop: float

    @property
    def gps_unix(self) -> float:
        """UNIX timestamp in GPS time scale (no leap correction)."""
        return (self.mjd - _MJD_UNIX_OFFSET) * 86400.0 + self.sod

    @property
    def utc_unix(self) -> float:
        """UNIX timestamp in UTC.  GPS-UTC = +18 s as of 2026."""
        return self.gps_unix - 18.0


def parse_kin(path: Path | str) -> list[KinEpoch]:
    """Read a PRIDE kin_<doy>_<station> file.  Returns one KinEpoch
    per data row (skipping the header).
    """
    path = Path(path)
    epochs: list[KinEpoch] = []
    in_data = False
    with open(path) as f:
        for line in f:
            if "END OF HEADER" in line:
                in_data = True
                continue
            if not in_data:
                continue
            # Skip the column-key line that starts with '*' at col 0.
            if line.startswith("*") or not line.strip():
                continue
            # PRIDE inserts a '*' quality-flag between sod and x in
            # float-only kinematic output (-m K -f mode).  Strip any
            # standalone '*' tokens before column-positional parsing.
            parts = [p for p in line.split() if p != "*"]
            if len(parts) < 10:
                continue
            try:
                epochs.append(KinEpoch(
                    mjd=int(parts[0]),
                    sod=float(parts[1]),
                    x=float(parts[2]),
                    y=float(parts[3]),
                    z=float(parts[4]),
                    lat=float(parts[5]),
                    lon=float(parts[6]),
                    height=float(parts[7]),
                    # parts[8..14] = Nsat per system; skip
                    pdop=float(parts[-1]),
                ))
            except (ValueError, IndexError) as e:
                log.warning("kin parse skip at %s: %s", line[:60].strip(), e)
    return epochs


# ── Engine log parsing ──────────────────────────────────────────── #

# 2026-05-20 07:30:33,072 INFO   [AntPosEst 10] positionσ=0.032m
#   pos=(LAT, LON, ALT) n=12 amb=13 WL: 0/14 fixed
#   NL: 0 fixed ...
# (Real-engine emits decimal-degree LAT/LON/ALT in the parens; pattern
# uses placeholders here to dodge the pre-commit secrets guard.)
_ANTPOSEST_RE = re.compile(
    r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}),\d+ INFO\s+"
    r"\[AntPosEst (\d+)\] position\S+=([\d.]+)m\s+"
    r"pos=\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)\s+"
    r"n=(\d+)\s+amb=(\d+)\s+"
    r"WL:\s*(\d+)/(\d+)\s+fixed\s+"
    r"NL:\s*(\d+)\s+fixed"
)

# Engine events worth flagging in the per-epoch window.  Each tuple:
# (label, regex).  Add more as the bisection surfaces interesting ones.
_EVENT_PATTERNS: list[tuple[str, re.Pattern[str]]] = [
    ("FALSE_FIX", re.compile(r"FALSE_FIX")),
    ("ANCHORING", re.compile(r"ANCHORING\b|state.*anchoring")),
    ("FIXED_LAMBDA", re.compile(r"FIXED_LAMBDA")),
    ("IF_STEP", re.compile(r"IF_STEP")),
    ("GF_STEP", re.compile(r"GF_STEP")),
    ("WL_DRIFT", re.compile(r"WL_DRIFT")),
    ("SLIP", re.compile(r"\bcycle slip\b|\bSLIP_")),
    ("RESET_GUARD", re.compile(r"reset_guard|reset.*guard")),
    ("CAT_REJECT", re.compile(r"catastrophic.?reject|CAT_REJECT")),
]

_TIMESTAMP_RE = re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})")


@dataclass
class EngineSample:
    """One [AntPosEst N] sample from the engine log."""
    wall_dt: datetime     # Engine wall-clock (local).
    epoch_n: int          # AntPosEst's per-epoch counter.
    sigma_3d: float       # positionσ in meters.
    lat: float
    lon: float
    height: float
    n_used: int
    n_amb: int
    wl_fixed: int
    wl_total: int
    nl_fixed: int


@dataclass
class EngineEvent:
    """One engine-log event marker (FALSE_FIX, IF_STEP, etc.)."""
    wall_dt: datetime
    label: str
    raw_line: str


def parse_engine_log(path: Path | str, tz_offset_hours: int = -5
                     ) -> tuple[list[EngineSample], list[EngineEvent]]:
    """Walk an engine log, return (AntPosEst samples, engine events).

    ``tz_offset_hours``: the engine logs in local time without TZ
    info.  Default −5 = CDT (America/Chicago summer).  Override for
    other deployments.
    """
    path = Path(path)
    tz = timezone(timedelta(hours=tz_offset_hours))
    samples: list[EngineSample] = []
    events: list[EngineEvent] = []
    with open(path) as f:
        for line in f:
            m = _ANTPOSEST_RE.search(line)
            if m:
                ts = datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S")
                ts = ts.replace(tzinfo=tz)
                samples.append(EngineSample(
                    wall_dt=ts,
                    epoch_n=int(m.group(2)),
                    sigma_3d=float(m.group(3)),
                    lat=float(m.group(4)),
                    lon=float(m.group(5)),
                    height=float(m.group(6)),
                    n_used=int(m.group(7)),
                    n_amb=int(m.group(8)),
                    wl_fixed=int(m.group(9)),
                    wl_total=int(m.group(10)),
                    nl_fixed=int(m.group(11)),
                ))
                continue
            # Event scan: cheap re check per line.
            tm = _TIMESTAMP_RE.match(line)
            if tm is None:
                continue
            for label, pat in _EVENT_PATTERNS:
                if pat.search(line):
                    ts = datetime.strptime(tm.group(1), "%Y-%m-%d %H:%M:%S")
                    ts = ts.replace(tzinfo=tz)
                    events.append(EngineEvent(
                        wall_dt=ts, label=label, raw_line=line.rstrip()))
                    break
    return samples, events


# ── Geodesy ─────────────────────────────────────────────────────── #

_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)


def lla_to_ecef(lat_deg: float, lon_deg: float, h: float
                ) -> tuple[float, float, float]:
    """WGS-84 LLA → ECEF.  Inputs: degrees, degrees, meters."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sl = math.sin(lat)
    cl = math.cos(lat)
    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sl * sl)
    x = (n + h) * cl * math.cos(lon)
    y = (n + h) * cl * math.sin(lon)
    z = (n * (1.0 - _WGS84_E2) + h) * sl
    return x, y, z


def ecef_dist(a: tuple[float, float, float],
              b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


# ── Reconciliation ──────────────────────────────────────────────── #

@dataclass
class DiffRecord:
    """One matched per-engine-sample pair: engine + nearest PRIDE epoch."""
    utc_unix: float
    engine_iso: str
    epoch_n: int
    engine_ecef: tuple[float, float, float]
    pride_ecef: tuple[float, float, float]
    pride_mjd: int
    pride_sod: float
    delta_m: float          # 3D Euclidean distance, meters
    sigma_3d: float         # engine reported σ at this sample
    nl_fixed: int
    wl_fixed: int
    n_used: int
    events_in_window: list[str] = field(default_factory=list)


def match_engine_to_pride(samples: list[EngineSample],
                          kin: list[KinEpoch],
                          events: list[EngineEvent],
                          window_s: float = 60.0,
                          match_tolerance_s: float = 30.0,
                          ) -> list[DiffRecord]:
    """Pair each engine sample with the nearest-in-UTC PRIDE epoch
    and compute Δ3D.

    ``match_tolerance_s``: if no PRIDE epoch lies within this many
    seconds of the engine wall-clock, the sample is skipped (the
    engine ran past the PRIDE capture window).
    ``window_s``: width of the engine-event window collected around
    each matched sample.
    """
    if not kin:
        return []
    # Sort kin by utc_unix for bisect-style lookup
    kin_by_unix = sorted(kin, key=lambda k: k.utc_unix)
    kin_unix = [k.utc_unix for k in kin_by_unix]

    out: list[DiffRecord] = []
    for s in samples:
        s_unix = s.wall_dt.timestamp()
        # Binary search for nearest kin epoch
        import bisect
        i = bisect.bisect_left(kin_unix, s_unix)
        candidates: list[KinEpoch] = []
        if i > 0:
            candidates.append(kin_by_unix[i - 1])
        if i < len(kin_by_unix):
            candidates.append(kin_by_unix[i])
        if not candidates:
            continue
        nearest = min(candidates, key=lambda k: abs(k.utc_unix - s_unix))
        if abs(nearest.utc_unix - s_unix) > match_tolerance_s:
            continue

        engine_ecef = lla_to_ecef(s.lat, s.lon, s.height)
        pride_ecef = (nearest.x, nearest.y, nearest.z)
        delta = ecef_dist(engine_ecef, pride_ecef)

        # Events in the ±window_s window around this sample
        window_events = [
            e.label for e in events
            if abs(e.wall_dt.timestamp() - s_unix) <= window_s
        ]

        out.append(DiffRecord(
            utc_unix=s_unix,
            engine_iso=s.wall_dt.isoformat(),
            epoch_n=s.epoch_n,
            engine_ecef=engine_ecef,
            pride_ecef=pride_ecef,
            pride_mjd=nearest.mjd,
            pride_sod=nearest.sod,
            delta_m=delta,
            sigma_3d=s.sigma_3d,
            nl_fixed=s.nl_fixed,
            wl_fixed=s.wl_fixed,
            n_used=s.n_used,
            events_in_window=window_events,
        ))
    return out


def first_divergence(records: Iterable[DiffRecord],
                     threshold_m: float = 0.10
                     ) -> DiffRecord | None:
    """First record where Δ3D exceeds ``threshold_m``.  None if all
    records stay under the threshold (engine + PRIDE in agreement)."""
    for r in records:
        if r.delta_m > threshold_m:
            return r
    return None


def write_csv(records: list[DiffRecord], out_path: Path | str) -> None:
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "utc_unix", "engine_iso", "epoch_n",
            "engine_x", "engine_y", "engine_z",
            "pride_x", "pride_y", "pride_z",
            "pride_mjd", "pride_sod",
            "delta_m", "sigma_3d_m",
            "nl_fixed", "wl_fixed", "n_used",
            "events_in_window",
        ])
        for r in records:
            w.writerow([
                f"{r.utc_unix:.3f}", r.engine_iso, r.epoch_n,
                f"{r.engine_ecef[0]:.4f}", f"{r.engine_ecef[1]:.4f}",
                f"{r.engine_ecef[2]:.4f}",
                f"{r.pride_ecef[0]:.4f}", f"{r.pride_ecef[1]:.4f}",
                f"{r.pride_ecef[2]:.4f}",
                r.pride_mjd, f"{r.pride_sod:.3f}",
                f"{r.delta_m:.4f}", f"{r.sigma_3d:.4f}",
                r.nl_fixed, r.wl_fixed, r.n_used,
                ";".join(r.events_in_window),
            ])

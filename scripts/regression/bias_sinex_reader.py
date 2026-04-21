"""IGS Bias-SINEX (BIA) reader — parses OSB / DSB / ISB bias files.

Used by the regression harness to provide phase-bias and code-bias
corrections without a live RTCM SSR stream.  Wuhan University's WUM
Bias-SINEX files (the ones PRIDE-PPPAR's `pdp3` script downloads)
contain the equivalent information that we'd otherwise consume from
CNES's SSRA00CNE0 RTCM 1265/1266 phase-bias messages.

## Format reference

IGS Bias-SINEX 1.00 spec:
  https://files.igs.org/pub/data/format/sinex_bias_100.pdf

Each bias entry is a fixed-format line in the `+BIAS/SOLUTION` block:

    BIAS  SVN_   PRN STATION__ OBS1 OBS2 BIAS_START____ BIAS_END______ UNIT __VALUE______ _STDDEV____
    OSB                  G01   G063           2020:001:00000 2020:002:00000 ns      -0.6850     0.0000

Field summary (column-anchored per spec):
- BIAS    [1:5]    (3-char bias type label like "OSB", "DSB", "ISB")
- SVN     [6:10]   (4-char satellite vehicle number, often blank)
- PRN     [11:14]  (3-char satellite PRN, e.g., "G01"; for station-bias
                    entries this is blank and STATION is set)
- STATION [15:24]  (9-char station ID; blank for satellite biases)
- OBS1    [25:28]  (4-char observation code: "C1C", "L1C", "L5Q", ...)
- OBS2    [30:33]  (4-char optional second observation code; blank for OSB)
- START   [35:48]  ("YYYY:DOY:SSSSS" timestamp)
- END     [50:63]  ("YYYY:DOY:SSSSS" timestamp)
- UNIT    [65:68]  ("ns", "cyc", "m", ...)
- VALUE   [70:90]  (signed float)
- STDDEV  [92:111] (signed float)

We extract per-SV per-signal phase and code biases.  The output is a
dict layered as `bias[sv][obs_code] = (value, unit)`, plus a list of
all entries for diagnostics.

## Output → engine integration

The regression harness wires this reader's output into a SSRState-
compatible adapter so the rest of the PPP code path doesn't need
to change.  Phase bias values from BIA convert to the same
"meters added to phi*_cyc" semantics that our SSR phase-bias
correction produces.

## Where to get a file

WUM products (PRIDE's preferred source):
  ftps://bdspride.com/wum/<YYYY>/<DOY>/

  WUM0MGXRAP_YYYYDDDHHMM_01D_01D_OSB.BIA

CNES products (an alternative, also useful for comparing AC datums):
  ftp://igs.ign.fr/pub/igs/products/mgex/<GPSWEEK>/

  GRG0MGXFIN_YYYYDDDHHMM_01D_01D_OSB.BIA

Both are publicly available.  The PRIDE example dataset bundles
RINEX obs but not the BIA — `pdp3` fetches them on demand.
"""

from __future__ import annotations

import logging
import re
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Iterator, Optional

log = logging.getLogger(__name__)


@dataclass
class BiasEntry:
    """One bias-table row — usually per satellite, per signal."""
    bias_type: str             # "OSB", "DSB", "ISB"
    sv: str                    # "G01", "E05", "C19" (or "" for station bias)
    station: str               # "ABMF" etc. (or "" for satellite bias)
    obs1: str                  # "C1C", "L1C", "L5Q" ...
    obs2: str                  # blank for OSB; second code for DSB
    start: datetime            # validity start (UTC)
    end: datetime              # validity end (UTC)
    unit: str                  # "ns", "cyc", "m"
    value: float
    stddev: float

    def is_phase(self) -> bool:
        return self.obs1.startswith("L")

    def is_code(self) -> bool:
        return self.obs1.startswith("C")

    def as_meters(self) -> Optional[float]:
        """Convert this entry's value to meters.

        Phase-bias values in cycles need wavelength to convert; this
        method returns None for those and lets the caller pass in the
        wavelength explicitly via `cycles_to_meters`.

        Code-bias values in ns convert directly via c·dt.
        """
        if self.unit == "m":
            return self.value
        if self.unit == "ns":
            return self.value * 1e-9 * 299_792_458.0
        return None


def cycles_to_meters(cycles: float, signal: str) -> Optional[float]:
    """Convert a phase-bias value in cycles to meters using the
    wavelength of the named signal.  Signal naming follows our
    internal convention (e.g. 'GPS-L1CA') used in
    `peppar_fix.signal_wavelengths`.
    """
    # Local import: only needed when we actually convert.  Keeps the
    # parser importable from environments without the engine.
    try:
        from signal_wavelengths import SIG_WAVELENGTH
    except ImportError:
        return None
    wl = SIG_WAVELENGTH.get(signal)
    if wl is None:
        return None
    return cycles * wl


# ── Time parsing ─────────────────────────────────────────────────── #

_SINEX_TIME = re.compile(r"(\d{4}):(\d{3}):(\d{5})")


def _parse_sinex_time(s: str) -> Optional[datetime]:
    """Parse 'YYYY:DOY:SSSSS' Bias-SINEX timestamp → UTC datetime.

    Special case: the spec allows '0000:000:00000' to denote "valid
    forever" — these become None so callers can treat them as
    open-ended.
    """
    s = s.strip()
    if not s or s == "0000:000:00000":
        return None
    m = _SINEX_TIME.fullmatch(s)
    if not m:
        return None
    year = int(m.group(1))
    doy = int(m.group(2))
    secs = int(m.group(3))
    base = datetime(year, 1, 1, tzinfo=timezone.utc)
    return base + timedelta(days=doy - 1, seconds=secs)


# ── Block detection / line parsing ───────────────────────────────── #

_BLOCK_START = re.compile(r"^\+BIAS/SOLUTION\b")
_BLOCK_END = re.compile(r"^-BIAS/SOLUTION\b")


def _parse_bias_line(line: str) -> Optional[BiasEntry]:
    """Parse one '+BIAS/SOLUTION' data line into a BiasEntry.

    Returns None for comment / header lines (those starting with '*'
    or with insufficient content).  Field positions follow the
    Bias-SINEX 1.00 spec.
    """
    if not line or line.startswith("*"):
        return None
    if len(line) < 90:
        # Probably the column-header row inside the block.
        return None
    bias_type = line[1:5].strip()
    if bias_type not in ("OSB", "DSB", "ISB"):
        return None
    sv = line[11:14].strip()
    station = line[15:24].strip()
    obs1 = line[25:29].strip()
    obs2 = line[30:34].strip()
    start_str = line[35:49].strip()
    end_str = line[50:64].strip()
    unit = line[65:68].strip()
    try:
        value = float(line[70:90].strip())
    except ValueError:
        return None
    try:
        stddev = float(line[92:111].strip()) if len(line) >= 111 else 0.0
    except ValueError:
        stddev = 0.0
    start = _parse_sinex_time(start_str) or datetime(
        1970, 1, 1, tzinfo=timezone.utc)
    end = _parse_sinex_time(end_str) or datetime(
        9999, 12, 31, tzinfo=timezone.utc)
    return BiasEntry(
        bias_type=bias_type, sv=sv, station=station,
        obs1=obs1, obs2=obs2, start=start, end=end,
        unit=unit, value=value, stddev=stddev,
    )


def iter_bias_entries(path: Path) -> Iterator[BiasEntry]:
    """Yield BiasEntry rows from a Bias-SINEX file.

    Reads the +BIAS/SOLUTION block; ignores all other sections.
    """
    with open(path) as f:
        in_block = False
        for line in f:
            if _BLOCK_START.match(line):
                in_block = True
                continue
            if _BLOCK_END.match(line):
                in_block = False
                continue
            if not in_block:
                continue
            entry = _parse_bias_line(line.rstrip("\n"))
            if entry is not None:
                yield entry


# ── High-level lookup table ─────────────────────────────────────── #

@dataclass
class BiasTable:
    """Per-SV per-signal bias lookup, with code/phase split.

    Built by `load_bias_table(path)`.  Provides O(1) lookup keyed by
    (sv, obs_code).  Validity windows are recorded but not enforced
    on lookup; the harness can filter by epoch if needed.

    Entries from station-only biases (no SV) are dropped — only
    satellite-side biases are useful for our PPP path.
    """
    # per_sv[sv][obs_code] = BiasEntry
    per_sv: dict[str, dict[str, BiasEntry]] = field(default_factory=dict)
    n_loaded: int = 0
    n_phase: int = 0
    n_code: int = 0
    n_skipped_station: int = 0
    n_skipped_dsb: int = 0   # DSB and ISB skipped — we want OSB only
    timespan: tuple[Optional[datetime], Optional[datetime]] = (None, None)

    def get(self, sv: str, obs_code: str) -> Optional[BiasEntry]:
        return self.per_sv.get(sv, {}).get(obs_code)

    def has_sv(self, sv: str) -> bool:
        return sv in self.per_sv

    def signals_for(self, sv: str) -> list[str]:
        """List of obs codes this SV has biases for."""
        return list(self.per_sv.get(sv, {}).keys())


def load_bias_table(path: Path) -> BiasTable:
    """Load a Bias-SINEX file into a BiasTable lookup.  Only OSB
    entries with a satellite ID (no station-only biases) are kept."""
    table = BiasTable()
    earliest: Optional[datetime] = None
    latest: Optional[datetime] = None
    for entry in iter_bias_entries(path):
        table.n_loaded += 1
        if entry.bias_type != "OSB":
            table.n_skipped_dsb += 1
            continue
        if not entry.sv:
            table.n_skipped_station += 1
            continue
        per = table.per_sv.setdefault(entry.sv, {})
        per[entry.obs1] = entry
        if entry.is_phase():
            table.n_phase += 1
        elif entry.is_code():
            table.n_code += 1
        if entry.start.year > 1970 and (earliest is None or entry.start < earliest):
            earliest = entry.start
        if entry.end.year < 9999 and (latest is None or entry.end > latest):
            latest = entry.end
    table.timespan = (earliest, latest)
    return table

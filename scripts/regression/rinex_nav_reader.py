"""RINEX 3.x NAV reader — parses broadcast-ephemeris files and produces
in-memory ephemeris dicts compatible with `broadcast_eph.BroadcastEphemeris`.

Used by the regression harness so PePPAR Fix can replay a PRIDE-bundled
dataset (or any IGS MGEX station-day) without needing a live RTCM stream
for broadcast ephemeris.

## Covered systems

- **GPS (G)** — RINEX NAV "LNAV" record (8 lines per record)
- **Galileo (E)** — RINEX NAV "I/NAV" or "F/NAV" (8 lines)
- **BeiDou (C)** — RINEX NAV "D1/D2" (8 lines); TGD1 + TGD2 parsed
- **GLONASS (R)** — not parsed (different format, not used in our AR path)

## Output

`iter_nav_records()` yields `(prn, eph_dict)` tuples.  The eph_dict is
shaped to match what `BroadcastEphemeris.update_from_rtcm` stores per
SV after its semicircle → radian conversion — i.e. angles are in
radians, ready to feed into `_kepler_ecef` / `_sat_clock`.

`load_into_ephemeris(path, beph)` is the convenience wrapper: loads
records and pushes them directly into a `BroadcastEphemeris` instance's
internal store, with the same 2-eph-per-SV retention the RTCM path
uses.

## Format reference

RINEX 3.04 spec:
  https://files.igs.org/pub/data/format/rinex304.pdf

GPS record layout (8 lines, 4-space leading indent on continuations):

    PRN yyyy mm dd hh mm ss  af0             af1             af2
        IODE           Crs           delta_n        M0
        Cuc            e             Cus            sqrt_a
        toe            Cic           omega0         Cis
        i0             Crc           omega          omega_dot
        i_dot          L2_codes      week           L2_P_flag
        accuracy       SV_health     TGD            IODC
        trans_time     fit_interval

Numeric fields are 19 chars each, F19.12 with "D" exponent marker (the
parser swaps "D" → "E" before float()).
"""

from __future__ import annotations

import logging
import math
import re
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterator, Optional

log = logging.getLogger(__name__)


# ── Time/week helpers ───────────────────────────────────────────────── #

# GPS epoch: 1980-01-06 00:00:00 UTC (no leap seconds)
_GPS_EPOCH = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)
_SECONDS_PER_WEEK = 604800.0
# BDS epoch: 2006-01-01 00:00:00 UTC (BDT = GPST − 14s)
_BDS_EPOCH = datetime(2006, 1, 1, 0, 0, 0, tzinfo=timezone.utc)
_BDS_GPS_OFFSET_S = 14.0


def _toc_to_sow(dt: datetime, system: str) -> tuple[int, float]:
    """Convert a RINEX NAV epoch (datetime, UTC) to (week, seconds-of-week)
    in the reference frame of that constellation's time system.

    GPS: GPS weeks since 1980-01-06.
    Galileo: same as GPS (GST aligned to GPST mod week at rollover).
    BDS: BDT = GPST − 14 s, weeks since 2006-01-01.
    """
    dt_utc = dt.replace(tzinfo=timezone.utc)
    if system == 'C':
        delta = dt_utc - _BDS_EPOCH
    else:
        delta = dt_utc - _GPS_EPOCH
    total_s = delta.total_seconds()
    if system == 'C':
        total_s -= _BDS_GPS_OFFSET_S  # BDT is behind GPST
    # Actually: RINEX stores GPS (and Galileo) epochs in GPS time directly
    # For BDS, RINEX 3 stores BDT (already shifted), so the epoch arithmetic
    # above should NOT add an extra -14s.  Per RINEX 3.04 §6.11 the epoch
    # for BDS is in BDT seconds of BDT week.
    # Correction: revert the -14s for BDS since RINEX already gives BDT.
    if system == 'C':
        total_s += _BDS_GPS_OFFSET_S  # undo the adjustment above
    week = int(total_s // _SECONDS_PER_WEEK)
    sow = total_s - week * _SECONDS_PER_WEEK
    return week, sow


# ── Numeric parsing helpers ─────────────────────────────────────────── #

def _parse_d(s: str) -> Optional[float]:
    """Parse a RINEX D-format float ('1.234D-05' → 1.234e-5).  Empty
    / blank fields return None.
    """
    s = s.strip()
    if not s:
        return None
    # RINEX uses 'D' instead of 'E'
    s2 = s.replace('D', 'E').replace('d', 'e')
    try:
        return float(s2)
    except ValueError:
        return None


# Each NAV field is 19 chars in the continuation lines; the first 4 chars
# of each continuation line are leading spaces, so the fields start at
# column 4 and repeat every 19 chars: [4..23], [23..42], [42..61], [61..80].
_FIELD_STARTS = (4, 23, 42, 61)
_FIELD_WIDTH = 19


def _parse_continuation(line: str) -> list[Optional[float]]:
    """Parse a NAV continuation line into up to 4 floats."""
    out: list[Optional[float]] = []
    for start in _FIELD_STARTS:
        chunk = line[start:start + _FIELD_WIDTH]
        out.append(_parse_d(chunk))
    return out


# Epoch line for GPS/GAL/BDS in RINEX 3 uses fixed columns:
#   [0:3]   SVn (e.g. "G01")
#   [4:23]  YYYY MM DD HH MM SS (19 chars)
#   [23:42] af0 (F19.12 D-notation)
#   [42:61] af1
#   [61:80] af2
# A plain regex can't split concatenated negative floats like
# "...E-04-5.002..." because the '-' doubles as a separator, so we
# parse columns directly.  Regex is used only for the SV-letter test.
_EPOCH_HEAD = re.compile(r"^([GECJIS])(\d{2})\b")


def _parse_epoch_line(line: str):
    """Parse a RINEX 3 NAV epoch line.

    Returns (sys_char, prn_num, datetime, af0, af1, af2) or None.
    """
    m = _EPOCH_HEAD.match(line)
    if not m:
        return None
    sys_char = m.group(1)
    prn_num = int(m.group(2))
    # Date/time fields are space-delimited in [4:23]; parse by split.
    ts_str = line[4:23].strip()
    try:
        parts = ts_str.split()
        if len(parts) != 6:
            return None
        year, mon, day, hh, mm, ss = [int(float(p)) for p in parts]
        dt = datetime(year, mon, day, hh, mm, ss)
    except (ValueError, IndexError):
        return None
    # Three clock coefficients in fixed columns.
    af0 = _parse_d(line[23:42]) if len(line) >= 42 else None
    af1 = _parse_d(line[42:61]) if len(line) >= 61 else None
    af2 = _parse_d(line[61:80]) if len(line) >= 80 else None
    return sys_char, prn_num, dt, af0, af1, af2


# ── Record parsers ─────────────────────────────────────────────────── #

# Angular keys (semicircles in RINEX, converted to radians on output)
_ANGULAR_KEYS = ('M0', 'delta_n', 'omega', 'omega0', 'i0', 'i_dot',
                 'omega_dot')


def _finalize(eph: dict) -> dict:
    """Common post-processing: semicircle → radian; add `system`; etc."""
    for key in _ANGULAR_KEYS:
        if key in eph and eph[key] is not None:
            eph[key] = eph[key] * math.pi
    return eph


def _parse_gps_record(epoch_line: str, cont_lines: list[str]) -> tuple[str, dict]:
    """Parse a GPS LNAV record (8 lines total: 1 epoch + 7 continuations).

    Returns (prn, eph_dict).  eph_dict includes standard BroadcastEphemeris
    fields with angular values already in radians.
    """
    parsed = _parse_epoch_line(epoch_line)
    assert parsed is not None, f"bad GPS epoch line: {epoch_line!r}"
    _, prn_num, dt, af0, af1, af2 = parsed
    week, toc = _toc_to_sow(dt, 'G')

    eph: dict = {
        'sat_id': int(prn_num),
        'system': 'G',
        'week': week,
        'toc': toc,
        'af0': af0,
        'af1': af1,
        'af2': af2,
    }
    # Continuation lines: each has up to 4 fields in the column layout.
    f = [_parse_continuation(line) for line in cont_lines]
    # GPS LNAV field layout:
    #   line 1: IODE,     Crs,      delta_n,  M0
    #   line 2: Cuc,      e,        Cus,      sqrt_a
    #   line 3: toe,      Cic,      omega0,   Cis
    #   line 4: i0,       Crc,      omega,    omega_dot
    #   line 5: i_dot,    L2_codes, week,     L2_P_flag
    #   line 6: accuracy, health,   TGD,      IODC
    #   line 7: trans_t,  fit_int
    eph['iode'] = f[0][0]
    eph['Crs']     = f[0][1]
    eph['delta_n'] = f[0][2]
    eph['M0']      = f[0][3]
    eph['Cuc']    = f[1][0]
    eph['e']      = f[1][1]
    eph['Cus']    = f[1][2]
    eph['sqrt_a'] = f[1][3]
    eph['toe']    = f[2][0]
    eph['Cic']    = f[2][1]
    eph['omega0'] = f[2][2]
    eph['Cis']    = f[2][3]
    eph['i0']       = f[3][0]
    eph['Crc']      = f[3][1]
    eph['omega']    = f[3][2]
    eph['omega_dot'] = f[3][3]
    eph['i_dot']    = f[4][0]
    eph['L2_codes'] = f[4][1]
    # f[4][2] is week again (broadcast week); use it to override if we
    # missed a rollover in the epoch conversion.
    if f[4][2] is not None:
        eph['week'] = int(f[4][2])
    eph['accuracy'] = f[5][0]
    eph['health']   = int(f[5][1]) if f[5][1] is not None else 0
    eph['tgd']      = f[5][2]
    eph['iodc']     = f[5][3]
    return f"G{int(prn_num):02d}", _finalize(eph)


def _parse_gal_record(epoch_line: str, cont_lines: list[str]) -> tuple[str, dict]:
    """Parse a Galileo I/NAV or F/NAV record.

    Galileo record layout is similar to GPS, with differences:
    - Line 5 field 1 ("i_dot") followed by "Data source" in field 2
    - Line 6: SISA, SV_health, BGD_E1_E5a, BGD_E1_E5b
    - Line 7: transmission time (plus spare)
    """
    parsed = _parse_epoch_line(epoch_line)
    assert parsed is not None, f"bad GAL epoch line: {epoch_line!r}"
    _, prn_num, dt, af0, af1, af2 = parsed
    week, toc = _toc_to_sow(dt, 'E')

    eph: dict = {
        'sat_id': int(prn_num),
        'system': 'E',
        'week': week,
        'toc': toc,
        'af0': af0,
        'af1': af1,
        'af2': af2,
    }
    f = [_parse_continuation(line) for line in cont_lines]
    eph['iod']     = f[0][0]
    eph['Crs']     = f[0][1]
    eph['delta_n'] = f[0][2]
    eph['M0']      = f[0][3]
    eph['Cuc']    = f[1][0]
    eph['e']      = f[1][1]
    eph['Cus']    = f[1][2]
    eph['sqrt_a'] = f[1][3]
    eph['toe']    = f[2][0]
    eph['Cic']    = f[2][1]
    eph['omega0'] = f[2][2]
    eph['Cis']    = f[2][3]
    eph['i0']       = f[3][0]
    eph['Crc']      = f[3][1]
    eph['omega']    = f[3][2]
    eph['omega_dot'] = f[3][3]
    eph['i_dot']       = f[4][0]
    eph['data_source'] = f[4][1]
    if f[4][2] is not None:
        eph['week'] = int(f[4][2])
    eph['sisa']      = f[5][0]
    eph['health']    = int(f[5][1]) if f[5][1] is not None else 0
    eph['tgd']       = f[5][2]        # BGD_E1_E5a — matches DF312 field name
    eph['bgd_e5b']   = f[5][3]        # BGD_E1_E5b — F9T-L2 profile uses this
    return f"E{int(prn_num):02d}", _finalize(eph)


def _parse_bds_record(epoch_line: str, cont_lines: list[str]) -> tuple[str, dict]:
    """Parse a BeiDou D1/D2 NAV record.

    BDS record layout mirrors GPS LNAV, with:
    - Line 6: accuracy, SV_health, TGD1 (B1I − B3I), TGD2 (B2I − B3I)
    - Line 7: transmission time, AODC
    """
    parsed = _parse_epoch_line(epoch_line)
    assert parsed is not None, f"bad BDS epoch line: {epoch_line!r}"
    _, prn_num, dt, af0, af1, af2 = parsed
    week, toc = _toc_to_sow(dt, 'C')

    eph: dict = {
        'sat_id': int(prn_num),
        'system': 'C',
        'week': week,
        'toc': toc,
        'af0': af0,
        'af1': af1,
        'af2': af2,
    }
    f = [_parse_continuation(line) for line in cont_lines]
    eph['iode']    = f[0][0]     # AODE
    eph['Crs']     = f[0][1]
    eph['delta_n'] = f[0][2]
    eph['M0']      = f[0][3]
    eph['Cuc']    = f[1][0]
    eph['e']      = f[1][1]
    eph['Cus']    = f[1][2]
    eph['sqrt_a'] = f[1][3]
    eph['toe']    = f[2][0]
    eph['Cic']    = f[2][1]
    eph['omega0'] = f[2][2]
    eph['Cis']    = f[2][3]
    eph['i0']       = f[3][0]
    eph['Crc']      = f[3][1]
    eph['omega']    = f[3][2]
    eph['omega_dot'] = f[3][3]
    eph['i_dot']    = f[4][0]
    if f[4][2] is not None:
        eph['week'] = int(f[4][2])
    eph['accuracy'] = f[5][0]
    eph['health']   = int(f[5][1]) if f[5][1] is not None else 0
    eph['tgd']      = f[5][2]        # TGD1 (B1I vs B3I)
    eph['tgd2']     = f[5][3]        # TGD2 (B2I vs B3I)
    return f"C{int(prn_num):02d}", _finalize(eph)


# Record length (number of lines after the epoch line) by system.
_RECORD_CONT_LINES = {'G': 7, 'E': 7, 'C': 7, 'J': 7, 'I': 7}


# ── Header + iteration ─────────────────────────────────────────────── #

@dataclass
class RinexNavHeader:
    version: str = ""
    file_type: str = ""           # "N" for single-system, "N M" for mixed
    leap_seconds: Optional[int] = None


_HDR_VERSION = re.compile(
    r"^\s*(\d+\.\d+)\s+(N.*?)(M|G|R|E|C|J|I|S)?\s+RINEX VERSION / TYPE"
)
_HDR_LEAP = re.compile(r"^\s+(\d+)\s+.*LEAP SECONDS")
_HDR_END = re.compile(r"^\s*END OF HEADER")


def parse_header(path: Path) -> RinexNavHeader:
    hdr = RinexNavHeader()
    with open(path) as f:
        for line in f:
            if _HDR_END.search(line):
                return hdr
            m = _HDR_VERSION.match(line)
            if m:
                hdr.version = m.group(1)
                hdr.file_type = (m.group(2) or "").strip()
                continue
            m = _HDR_LEAP.match(line)
            if m:
                hdr.leap_seconds = int(m.group(1))
                continue
    return hdr


def iter_nav_records(path: Path) -> Iterator[tuple[str, dict]]:
    """Yield (prn, eph_dict) for each record in the NAV file.

    Skips GLONASS (R) records — our regression path doesn't use them.
    """
    with open(path) as f:
        # Skip through header
        for line in f:
            if _HDR_END.search(line):
                break
        while True:
            epoch_line = f.readline()
            if not epoch_line:
                return
            epoch_line = epoch_line.rstrip("\n")
            if not epoch_line.strip():
                continue
            m = _EPOCH_HEAD.match(epoch_line)
            if not m:
                # Unexpected line; skip.
                continue
            sys_char = m.group(1)
            n_cont = _RECORD_CONT_LINES.get(sys_char)
            if n_cont is None:
                # GLO or unknown — read and skip continuation lines.
                for _ in range(3):  # GLO has 3 continuation lines
                    if not f.readline():
                        return
                continue
            cont_lines = []
            for _ in range(n_cont):
                ln = f.readline()
                if not ln:
                    return
                cont_lines.append(ln.rstrip("\n"))
            try:
                if sys_char == 'G':
                    prn, eph = _parse_gps_record(epoch_line, cont_lines)
                elif sys_char == 'E':
                    prn, eph = _parse_gal_record(epoch_line, cont_lines)
                elif sys_char == 'C':
                    prn, eph = _parse_bds_record(epoch_line, cont_lines)
                else:
                    continue
                yield prn, eph
            except (AssertionError, ValueError) as e:
                log.warning("skipping malformed NAV record: %s", e)
                continue


def load_into_ephemeris(path: Path, beph) -> int:
    """Populate a BroadcastEphemeris instance from a RINEX NAV file.

    Uses the same 2-eph-per-SV retention as `BroadcastEphemeris
    .update_from_rtcm`: if multiple records share the same `toc` for an
    SV, keep the latest; otherwise retain the two most recent (by toc).

    Returns the number of records loaded.  Adds GM per system, matching
    what the RTCM path does.
    """
    # Import locally to avoid a hard dep from the reader itself (so it
    # can be tested without the full engine tree).
    from broadcast_eph import GM_GPS, GM_GAL, GM_BDS

    GM_BY_SYS = {'G': GM_GPS, 'E': GM_GAL, 'C': GM_BDS}
    n = 0
    for prn, eph in iter_nav_records(path):
        eph['gm'] = GM_BY_SYS.get(eph['system'])
        # Same retention logic as BroadcastEphemeris.update_from_rtcm.
        existing = beph._ephs.get(prn)
        if existing is None:
            beph._ephs[prn] = [eph]
        else:
            new_toc = eph.get('toc', 0)
            replaced = False
            for i, e in enumerate(existing):
                if e.get('toc', 0) == new_toc:
                    existing[i] = eph
                    replaced = True
                    break
            if not replaced:
                existing.append(eph)
                if len(existing) > 2:
                    existing.sort(key=lambda e: e.get('toc', 0))
                    beph._ephs[prn] = existing[-2:]
        n += 1
    return n

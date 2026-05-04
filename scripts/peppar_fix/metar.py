"""METAR cron-CSV reader for METAR-seeded ZTD prior (I-024942).

The cron capture format produced by ~/met/cron/fetch.sh on TimeHat
(live since 2026-05-04) is one row per fetch with columns:
  fetch_ts, report_ts, temp_C, dewp_C, altim_hPa, slp_hPa, raw_metar

Each row is the latest decoded KDPA METAR.  Multiple fetch_ts can
share the same report_ts when the cron polls more frequently than
KDPA reports.

This module reads the *latest* row, parses the numeric weather
fields, and offers freshness checking.  Spatial correction from
KDPA→site is handled by the caller via saastamoinen.metar_to_*
which take site lat/elev directly — KDPA's coordinates aren't
recorded in the CSV (would require a sibling lookup if we ever
add multi-site METAR sources).
"""
from __future__ import annotations

import csv
from datetime import datetime, timezone
from pathlib import Path


DEFAULT_KDPA_CSV = Path("/home/bob/met/kdpa.csv")


class MetarReadError(Exception):
    """Raised when METAR file is missing, empty, or malformed."""


def read_latest_metar(path: str | Path = DEFAULT_KDPA_CSV) -> dict:
    """Read the last complete row from a METAR cron CSV.

    Returns dict with keys:
      fetch_ts (datetime, UTC), report_ts (datetime, UTC),
      temp_C, dewp_C, altim_hPa, slp_hPa, raw_metar
    Numeric fields are float.

    Raises MetarReadError if the file doesn't exist, is empty,
    or the last row can't be parsed.
    """
    p = Path(path)
    if not p.is_file():
        raise MetarReadError(f"METAR file not found: {p}")
    last = None
    try:
        with open(p, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                last = row
    except OSError as exc:
        raise MetarReadError(f"reading {p}: {exc}") from exc
    if last is None:
        raise MetarReadError(f"no rows in {p}")
    try:
        return {
            'fetch_ts': _parse_iso(last['fetch_ts']),
            'report_ts': _parse_iso(last['report_ts']),
            'temp_C': float(last['temp_C']),
            'dewp_C': float(last['dewp_C']),
            'altim_hPa': float(last['altim_hPa']),
            'slp_hPa': float(last['slp_hPa']) if last.get('slp_hPa') else None,
            'raw_metar': last.get('raw_metar', ''),
        }
    except (KeyError, ValueError) as exc:
        raise MetarReadError(
            f"malformed METAR row in {p}: {exc} (row={last!r})") from exc


def metar_age_seconds(record: dict, now: datetime | None = None) -> float:
    """Age of the METAR *report* (not fetch) in seconds.

    Using report_ts means we don't reward stale data just because
    cron retrieved it recently.  KDPA reports hourly with occasional
    SPECIs; ages above ~3600 s mean cron isn't keeping up.
    """
    now = now or datetime.now(timezone.utc)
    return (now - record['report_ts']).total_seconds()


def _parse_iso(s: str) -> datetime:
    """Parse the ISO-8601 timestamps the cron emits.

    Accepts both 'Z' suffix and milliseconds variants; the cron has
    been observed to emit:
      2026-05-04T01:56:20Z
      2026-05-04T02:00:00.000Z
    """
    s = s.strip()
    if s.endswith('Z'):
        s = s[:-1] + '+00:00'
    return datetime.fromisoformat(s)

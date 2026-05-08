"""METAR HTTP fetcher for METAR-seeded ZTD prior (I-024942).

The engine queries https://aviationweather.gov/api/data/metar at filter
init to get a current temperature / dewpoint / altimeter reading from
a nearby airport (KDPA for the lab, ~7 km from UFO1).  The returned
weather feeds Saastamoinen so the IDX_ZTD residual state starts within
physical envelope rather than absorbing meters of clock-state error
during cold start.

HTTP-only by design: no cron, no local CSV, no per-host file setup.
This makes --init-ztd-from-met work on a fresh host with nothing more
than network connectivity (which the engine already requires for
NTRIP / SSR).  On HTTP failure the engine logs and falls back to a
wide ZTD prior — METAR is bonus information, not a hard dependency.
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request
from datetime import datetime, timezone


DEFAULT_STATION = "KDPA"
DEFAULT_TIMEOUT_S = 5.0
API_URL = "https://aviationweather.gov/api/data/metar?ids={station}&format=json"


class MetarReadError(Exception):
    """Raised when METAR fetch / parse fails."""


def fetch_latest_metar(station: str = DEFAULT_STATION,
                       timeout: float = DEFAULT_TIMEOUT_S) -> dict:
    """Fetch the latest METAR for `station` from aviationweather.gov.

    Returns dict with keys:
      report_ts (datetime, UTC), temp_C, dewp_C, altim_hPa,
      slp_hPa (may be None), raw_metar, station

    Raises MetarReadError on network failure, HTTP error, JSON parse
    failure, empty response, or any required field missing / non-
    numeric.  Caller is expected to handle the exception by logging
    and falling back to the legacy wide ZTD prior.
    """
    url = API_URL.format(station=station)
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            body = resp.read()
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise MetarReadError(f"HTTP fetch failed for {station}: {exc}") from exc
    try:
        data = json.loads(body)
    except json.JSONDecodeError as exc:
        raise MetarReadError(f"JSON parse failed: {exc}") from exc
    if not isinstance(data, list) or not data:
        raise MetarReadError(f"empty METAR response for {station}")
    rec = data[0]
    try:
        return {
            'report_ts': _parse_iso(rec['reportTime']),
            'temp_C': float(rec['temp']),
            'dewp_C': float(rec['dewp']),
            'altim_hPa': float(rec['altim']),
            'slp_hPa': float(rec['slp']) if rec.get('slp') is not None else None,
            'raw_metar': rec.get('rawOb', ''),
            'station': rec.get('icaoId', station),
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise MetarReadError(
            f"malformed METAR record for {station}: {exc} "
            f"(rec={rec!r})") from exc


def metar_age_seconds(record: dict, now: datetime | None = None) -> float:
    """Age of the METAR *report* in seconds.

    Using report_ts (not fetch time) means we don't reward stale data
    just because the API returned it recently.  KDPA reports hourly
    with occasional SPECIs; ages above ~3600 s mean we're seeing a
    delayed report (rare in practice when querying online).
    """
    now = now or datetime.now(timezone.utc)
    return (now - record['report_ts']).total_seconds()


def _parse_iso(s: str) -> datetime:
    """Parse the ISO-8601 timestamps the API emits.

    Accepts 'Z' suffix.  API observed to emit:
      2026-05-04T14:00:00.000Z
      2026-05-04T13:56:24.992Z
    """
    s = s.strip()
    if s.endswith('Z'):
        s = s[:-1] + '+00:00'
    return datetime.fromisoformat(s)

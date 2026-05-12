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

## Retry + cache (2026-05-12, wrong-int-basin overnight aftermath)

The 2026-05-11/12 overnight on TimeHat + MadHat lost most of its
diagnostic value because the KDPA HTTP request timed out at cold
start on both hosts simultaneously.  Without METAR, both engines
fell back to the ``ztd=0 σ=200mm`` wide-prior path and spent 4-5h
oscillating through negative-ZTD basins before recovering.

To prevent that confound from poisoning future overnights:

1. ``fetch_latest_metar()`` retries on transient HTTP failures
   (3 attempts at 1s / 2s / 5s backoff) before raising
   ``MetarReadError``.

2. On a successful fetch, the parsed record is written to
   ``$XDG_CACHE_HOME/peppar-fix/metar-{station}.json`` (or
   ``~/.cache/peppar-fix/metar-{station}.json`` if XDG unset).
   Atomic write via tmpfile + rename.

3. On a failed fetch, the cache is read.  If the cached METAR
   is < 2h old, return it (logged as cached so the engine knows
   it didn't get a fresh fetch).  Older cache → MetarReadError.

The cache is per-host, per-user, automatic; no operator setup.
The engine's existing 2h staleness gate in ``metar_age_seconds()``
applies to cached records just as to live ones.
"""
from __future__ import annotations

import json
import logging
import os
import tempfile
import time as time_module
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path


DEFAULT_STATION = "KDPA"
DEFAULT_TIMEOUT_S = 5.0
API_URL = "https://aviationweather.gov/api/data/metar?ids={station}&format=json"

# Retry policy on transient HTTP failures.  Three attempts at 1s/2s/5s
# = 8s max wall-clock before giving up — safely within engine cold-
# start budget (typical 30-60s before EKF needs ZTD).
_RETRY_DELAYS_S = (1.0, 2.0, 5.0)

# Cache TTL.  Matches the engine's staleness gate in metar_age_seconds()
# — there's no point keeping cache entries the consumer will reject.
_CACHE_MAX_AGE_S = 7200.0

log = logging.getLogger(__name__)


class MetarReadError(Exception):
    """Raised when METAR fetch / parse fails (live + cache)."""


def _cache_dir() -> Path:
    """Per-user cache directory for METAR records.  Auto-created."""
    base = os.environ.get('XDG_CACHE_HOME')
    if base:
        root = Path(base) / "peppar-fix"
    else:
        root = Path.home() / ".cache" / "peppar-fix"
    root.mkdir(parents=True, exist_ok=True)
    return root


def _cache_path(station: str) -> Path:
    return _cache_dir() / f"metar-{station}.json"


def _record_to_cache_json(rec: dict) -> str:
    """Serialize a parsed METAR record for the cache file."""
    return json.dumps({
        'report_ts': rec['report_ts'].isoformat(),
        'temp_C': rec['temp_C'],
        'dewp_C': rec['dewp_C'],
        'altim_hPa': rec['altim_hPa'],
        'slp_hPa': rec.get('slp_hPa'),
        'raw_metar': rec.get('raw_metar', ''),
        'station': rec['station'],
    })


def _cache_json_to_record(text: str) -> dict:
    """Inverse of ``_record_to_cache_json``."""
    raw = json.loads(text)
    raw['report_ts'] = _parse_iso(raw['report_ts'])
    return raw


def _write_cache(station: str, rec: dict) -> None:
    """Atomic-write a METAR record to the cache.  Best effort."""
    path = _cache_path(station)
    try:
        # Write to tmpfile + rename so a concurrent reader never sees
        # a partial write.
        tmp = tempfile.NamedTemporaryFile(
            mode='w', dir=str(path.parent),
            prefix=f"metar-{station}.", suffix='.tmp',
            delete=False,
        )
        try:
            tmp.write(_record_to_cache_json(rec))
            tmp.flush()
            os.fsync(tmp.fileno())
        finally:
            tmp.close()
        os.replace(tmp.name, str(path))
    except OSError as exc:
        # Cache is best-effort.  Don't fail the live fetch just
        # because we couldn't write the cache.
        log.warning("METAR cache write failed (%s): %s", path, exc)


def _read_cache(station: str, max_age_s: float = _CACHE_MAX_AGE_S
                ) -> dict | None:
    """Read cached METAR if recent enough.  Returns None otherwise."""
    path = _cache_path(station)
    if not path.exists():
        return None
    try:
        text = path.read_text()
        rec = _cache_json_to_record(text)
    except (OSError, ValueError, KeyError) as exc:
        log.warning("METAR cache read failed (%s): %s", path, exc)
        return None
    age_s = metar_age_seconds(rec)
    if age_s > max_age_s:
        # Cache present but report itself is too old.  Same gate the
        # engine applies to live fetches — return None so caller
        # raises MetarReadError.
        return None
    return rec


def _fetch_once(station: str, timeout: float) -> dict:
    """Single HTTP attempt — raises MetarReadError on any failure."""
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


def fetch_latest_metar(station: str = DEFAULT_STATION,
                       timeout: float = DEFAULT_TIMEOUT_S,
                       use_cache: bool = True,
                       retry_delays_s: tuple = _RETRY_DELAYS_S,
                       _sleep=time_module.sleep,
                       _now=time_module.monotonic) -> dict:
    """Fetch the latest METAR for `station`, with retry + cache fallback.

    Returns dict with keys:
      report_ts (datetime, UTC), temp_C, dewp_C, altim_hPa,
      slp_hPa (may be None), raw_metar, station

    Resilience layers:
      1. ``len(retry_delays_s)+1`` HTTP attempts with the given
         inter-attempt sleeps (default: 1s / 2s / 5s — 4 total
         attempts, 8s max wall-clock).
      2. On a successful fetch, write the record to the per-user
         disk cache for the next cold-start.
      3. If every retry fails AND ``use_cache=True``, fall back to
         the cache.  Cache entries < 2h old are returned (gated by
         ``metar_age_seconds()``); older entries raise.
      4. If no cache or cache too stale, raise ``MetarReadError``
         with the last live-fetch exception's message.

    The ``_sleep`` and ``_now`` injection points exist for unit
    tests; production callers should leave them at defaults.

    Caller is expected to handle ``MetarReadError`` by logging and
    falling back to the legacy wide ZTD prior — METAR is bonus
    information, not a hard dependency.
    """
    last_exc: MetarReadError | None = None
    delays = list(retry_delays_s) + [None]  # final attempt has no follow-up sleep
    for i, delay_after in enumerate(delays):
        try:
            rec = _fetch_once(station, timeout=timeout)
            if use_cache:
                _write_cache(station, rec)
            if i > 0:
                log.info(
                    "METAR %s: live fetch succeeded on retry %d/%d",
                    station, i + 1, len(delays))
            return rec
        except MetarReadError as exc:
            last_exc = exc
            if delay_after is None:
                break
            log.info(
                "METAR %s: attempt %d/%d failed (%s); retrying in %.1fs",
                station, i + 1, len(delays), exc, delay_after)
            _sleep(delay_after)

    # All retries failed.  Cache fallback.
    if use_cache:
        cached = _read_cache(station)
        if cached is not None:
            log.warning(
                "METAR %s: live fetch failed after %d attempts (%s); "
                "using cached report from %s",
                station, len(delays), last_exc,
                cached['report_ts'].isoformat())
            return cached

    raise MetarReadError(
        f"METAR fetch failed for {station} after {len(delays)} attempts: "
        f"{last_exc}") from last_exc


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

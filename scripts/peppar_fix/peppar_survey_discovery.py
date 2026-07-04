"""peppar-survey base discovery (S2 of I-071401): find a reference base near a
target position and fetch its RINEX so the --baseline backend (S1) can run.

Two halves, kept separate so each is testable in isolation:

  1. **Sourcetable discovery** — fetch an NTRIP caster's sourcetable, parse the
     STR records, and Haversine-rank the mounts by distance to a target APC.
     (Promoted from the 2026-07-03 scratchpad find_base.py.)

  2. **Region -> source table + archive fetchers** — map a target lat/lon to the
     right open archive (NGS CORS for North America, EUREF for Europe), each
     carrying the base's regional datum realization (the value S1 pre-converts
     from -> ITRF2020@epoch).  Fetchers pull a named station's daily/hourly
     RINEX for the offline baseline.

The top-level ``discover_base`` glues them: pick the region's source, rank the
nearest station, and return a descriptor S1's --baseline consumes.  Network I/O
is injected (``fetcher`` / ``sourcetable_fetcher``) so the ranking + selection
logic is deterministic under test — heuristics only buy speed; the returned
base is verified by the actual baseline solve (S1's ITRF2020 pinning + n_used).
"""
from __future__ import annotations

import logging
import math
import socket
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence

# Reuse S1's NGS CORS daily fetcher rather than duplicate it.
from peppar_fix.peppar_survey_rtklib import fetch_cors_rinex

log = logging.getLogger("peppar-survey.discovery")


# ── geometry ───────────────────────────────────────────────────────── #


def haversine_km(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance (km) between two lat/lon points."""
    R = 6371.0088
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


# ── NTRIP sourcetable discovery (promoted find_base.py) ─────────────── #


@dataclass(frozen=True)
class Mount:
    """One NTRIP sourcetable STR record (the fields we rank on)."""
    mount: str
    fmt: str
    nav: str
    country: str
    lat: float
    lon: float
    vrs: bool          # True = network/VRS mount (needs GGA/NMEA, no fixed base)


def fetch_sourcetable(host: str, port: int, timeout: int = 25) -> str:
    """Fetch an NTRIP 2.0 sourcetable (the caster's mount catalogue)."""
    req = (f"GET / HTTP/1.1\r\nHost: {host}:{port}\r\n"
           "Ntrip-Version: Ntrip/2.0\r\n"
           "User-Agent: NTRIP peppar-fix/1.0\r\nConnection: close\r\n\r\n")
    s = socket.create_connection((host, port), timeout=timeout)
    s.settimeout(timeout)
    try:
        s.sendall(req.encode())
        buf = b""
        while True:
            try:
                chunk = s.recv(65536)
            except socket.timeout:
                break
            if not chunk:
                break
            buf += chunk
    finally:
        s.close()
    return buf.decode("latin-1", "replace")


def parse_sourcetable(text: str) -> list[Mount]:
    """Parse STR records from a sourcetable.  Fields (0-indexed): 1=mount
    3=format 6=nav 8=country 9=lat 10=lon 11=nmea(1=needs GGA) 12=solution
    (1=network/VRS).  Skips malformed rows and 0,0 / out-of-range coords."""
    mounts: list[Mount] = []
    for ln in text.splitlines():
        if not ln.startswith("STR;"):
            continue
        f = ln.split(";")
        if len(f) < 13:
            continue
        try:
            lat = float(f[9])
            lon = float(f[10])
        except ValueError:
            continue
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            continue
        if lat == 0.0 and lon == 0.0:
            continue
        mounts.append(Mount(
            mount=f[1], fmt=f[3], nav=f[6], country=f[8],
            lat=lat, lon=lon, vrs=(f[11] == "1" or f[12] == "1")))
    return mounts


def rank_by_distance(
    mounts: Sequence[Mount],
    lat: float,
    lon: float,
    *,
    max_km: float = 80.0,
    exclude_vrs: bool = True,
) -> list[tuple[float, Mount]]:
    """(distance_km, Mount) sorted nearest-first, within ``max_km``.

    VRS/network mounts are excluded by default — a fixed single-base station is
    what the relative baseline wants (a VRS needs a live GGA feed, not an offline
    daily/hourly RINEX)."""
    out: list[tuple[float, Mount]] = []
    for m in mounts:
        if exclude_vrs and m.vrs:
            continue
        d = haversine_km(lat, lon, m.lat, m.lon)
        if d <= max_km:
            out.append((d, m))
    out.sort(key=lambda t: t[0])
    return out


# ── region -> source table + archive fetchers ──────────────────────── #

# EUREF near-real-time hourly RINEX archive (URL structure mapped 2026-07-03).
DEFAULT_EUREF_NRT_URL_TMPL = (
    "https://igs.bkg.bund.de/root_ftp/EUREF/nrt/{doy:03d}/{hour:02d}/"
    "{station_lc}{doy:03d}{hour:02d}.{yy:02d}o.gz")


def fetch_euref_nrt_rinex(
    station: str,
    year: int,
    doy: int,
    hour: int,
    work_dir: Path,
    *,
    url_template: str = DEFAULT_EUREF_NRT_URL_TMPL,
    fetcher: Callable = urllib.request.urlretrieve,
) -> Path | None:
    """Download a EUREF nrt hourly RINEX obs file to work_dir (gunzipped), or
    None on failure.  Mirrors fetch_cors_rinex's shape (cache + gunzip)."""
    import gzip
    import shutil

    work_dir.mkdir(parents=True, exist_ok=True)
    yy = year % 100
    station_lc = station.lower()[:4]
    local_name = f"{station_lc}{doy:03d}{hour:02d}.{yy:02d}o"
    local_path = work_dir / local_name
    if local_path.exists():
        return local_path
    url = url_template.format(
        doy=doy, hour=hour, yy=yy, station_lc=station_lc)
    gz_path = work_dir / (local_name + ".gz")
    try:
        fetcher(url, str(gz_path))
    except Exception as e:  # noqa: BLE001 - network/HTTP errors are non-fatal
        log.warning("EUREF nrt fetch failed (%s): %s", url, e)
        return None
    try:
        with gzip.open(gz_path, "rb") as gz, open(local_path, "wb") as out:
            shutil.copyfileobj(gz, out)
    except OSError as e:
        log.warning("EUREF nrt gunzip failed: %s", e)
        return None
    gz_path.unlink()
    return local_path


@dataclass(frozen=True)
class RegionSource:
    """One region -> open-archive mapping.

    ``bbox`` = (lat_min, lat_max, lon_min, lon_max).  ``base_realization`` is the
    datum the archive's station coordinates carry — the value S1's --baseline
    pre-converts from -> ITRF2020@epoch (NAD83(2011) for NGS CORS, ETRS89 for
    EUREF).  ``kind`` selects the fetcher.
    """
    name: str
    kind: str                    # "ngs_cors" | "euref_nrt"
    bbox: tuple[float, float, float, float]
    base_realization: str

    def contains(self, lat: float, lon: float) -> bool:
        lo_a, hi_a, lo_o, hi_o = self.bbox
        return lo_a <= lat <= hi_a and lo_o <= lon <= hi_o


# Ordered most-specific-first; source_for_position returns the first match.
REGION_SOURCES: tuple[RegionSource, ...] = (
    RegionSource("NGS CORS (North America)", "ngs_cors",
                 (15.0, 72.0, -170.0, -50.0), "NAD83(2011)"),
    RegionSource("EUREF (Europe)", "euref_nrt",
                 (34.0, 72.0, -12.0, 40.0), "ETRS89"),
)


def source_for_position(lat: float, lon: float) -> RegionSource | None:
    """The open-archive source whose region contains (lat, lon), or None
    (caller falls back to the PRIDE floor / an explicit --base)."""
    for src in REGION_SOURCES:
        if src.contains(lat, lon):
            return src
    return None


@dataclass(frozen=True)
class BaseDescriptor:
    """A discovered base, ready to hand to S1's --baseline."""
    station: str
    distance_km: float
    source: RegionSource
    base_realization: str        # == source.base_realization (convenience)


def discover_base(
    lat: float,
    lon: float,
    *,
    caster_host: str | None = None,
    caster_port: int = 2101,
    max_km: float = 80.0,
    sourcetable_fetcher: Callable[[str, int], str] = fetch_sourcetable,
) -> BaseDescriptor | None:
    """Find the nearest usable base station to (lat, lon).

    Selects the region source (NGS CORS / EUREF) for logging + the datum
    realization, then — when a ``caster_host`` is given — ranks that caster's
    sourcetable to pick the nearest fixed (non-VRS) mount.  Returns a
    BaseDescriptor or None (no region / caster unreachable / nothing in range),
    in which case the caller falls back to the PRIDE floor.  Fetching the base's
    RINEX is the fetcher's job (fetch_cors_rinex / fetch_euref_nrt_rinex),
    driven by the descriptor's source.kind.
    """
    src = source_for_position(lat, lon)
    if src is None:
        log.info("No region source for %.4f,%.4f — fall back to PRIDE floor",
                 lat, lon)
        return None
    if caster_host is None:
        log.info("Region %s selected (datum %s) but no caster given for "
                 "sourcetable ranking", src.name, src.base_realization)
        return None
    try:
        table = sourcetable_fetcher(caster_host, caster_port)
    except Exception as e:  # noqa: BLE001 - unreachable caster is non-fatal
        log.warning("Sourcetable fetch from %s:%d failed: %s",
                    caster_host, caster_port, e)
        return None
    ranked = rank_by_distance(parse_sourcetable(table), lat, lon, max_km=max_km)
    if not ranked:
        log.info("No fixed base within %.0f km of %.4f,%.4f", max_km, lat, lon)
        return None
    dist, mount = ranked[0]
    log.info("Nearest base: %s @ %.1f km (%s, datum %s)",
             mount.mount, dist, src.name, src.base_realization)
    return BaseDescriptor(station=mount.mount, distance_km=dist, source=src,
                          base_realization=src.base_realization)


def fetch_base_rinex(
    descriptor: BaseDescriptor,
    year: int,
    doy: int,
    work_dir: Path,
    *,
    hour: int = 0,
    cors_fetcher: Callable = fetch_cors_rinex,
    euref_fetcher: Callable = fetch_euref_nrt_rinex,
) -> Path | None:
    """Fetch the discovered base's RINEX via its source's archive fetcher.

    The glue between discovery and S1's --baseline: hand the returned path to
    ``--base <path> --base-realization <descriptor.base_realization>`` and S1
    pre-converts that base to ITRF2020@epoch.  Returns None on fetch failure.
    """
    if descriptor.source.kind == "ngs_cors":
        return cors_fetcher(descriptor.station, year, doy, work_dir)
    if descriptor.source.kind == "euref_nrt":
        return euref_fetcher(descriptor.station, year, doy, hour, work_dir)
    log.warning("Unknown source kind %r for base %s",
                descriptor.source.kind, descriptor.station)
    return None


def _main(argv: list[str] | None = None) -> int:
    """Diagnostic CLI: rank the nearest fixed base to a target position.

    Example: peppar_survey_discovery.py 41.85 -88.10 --caster caster:2101
    """
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("lat", type=float)
    ap.add_argument("lon", type=float)
    ap.add_argument("--caster", default=None,
                    help="NTRIP caster host[:port] to rank the sourcetable of")
    ap.add_argument("--max-km", type=float, default=80.0)
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    host, port = None, 2101
    if args.caster:
        host, _, p = args.caster.partition(":")
        port = int(p) if p else 2101
    src = source_for_position(args.lat, args.lon)
    print(f"region source: {src.name if src else 'NONE (PRIDE-floor fallback)'}")
    desc = discover_base(args.lat, args.lon, caster_host=host, caster_port=port,
                         max_km=args.max_km)
    if desc is None:
        print("no base discovered")
        return 1
    print(f"base: {desc.station}  {desc.distance_km:.1f} km  "
          f"datum {desc.base_realization}  ({desc.source.kind})")
    return 0


__all__ = [
    "haversine_km", "Mount", "fetch_sourcetable", "parse_sourcetable",
    "rank_by_distance", "RegionSource", "REGION_SOURCES", "source_for_position",
    "fetch_euref_nrt_rinex", "fetch_cors_rinex", "BaseDescriptor",
    "discover_base", "fetch_base_rinex",
]


if __name__ == "__main__":
    import sys as _sys
    _sys.exit(_main())

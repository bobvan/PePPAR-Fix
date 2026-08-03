"""peppar-survey base discovery (S2 of I-071401): find a reference base near a
target position and fetch its RINEX so the --baseline backend (S1) can run.

Three halves (sic), kept separate so each is testable in isolation:

  1. **Sourcetable discovery** — fetch an NTRIP caster's sourcetable, parse the
     STR records, and Haversine-rank the mounts by distance to a target APC.
     (Promoted from the 2026-07-03 scratchpad find_base.py.)

  2. **Catalogue discovery** — rank an archive's *own* station catalogue.  Not
     every archive has a caster: NGS CORS is ~1650 operational stations that
     appear in no sourcetable anywhere, published instead as one daily file.
     Without this, a US site with a CORS 20 km away sees no base at all and
     pays the PRIDE floor's product latency for nothing (Newton WI, 2026-08-03).

  3. **Region -> source table + archive fetchers** — map a target lat/lon to the
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
import os
import re
import socket
import time
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

# EUREF near-real-time hourly RINEX archive (dir structure mapped 2026-07-03).
# It serves RINEX-3 long-named, HATANAKA-compressed obs, e.g.
#   AAER00FRA_R_20261851200_01H_30S_MO.crx.gz
# The data-source char (R = from receiver stream / S = from RINEX) varies per
# station, so we LIST the hour dir and match rather than hardcode it (main #268).
DEFAULT_EUREF_NRT_DIR_TMPL = (
    "https://igs.bkg.bund.de/root_ftp/EUREF/nrt/{doy:03d}/{hour:02d}/")


def _http_listing(url: str, timeout: int = 25) -> str:
    with urllib.request.urlopen(url, timeout=timeout) as r:  # noqa: S310
        return r.read().decode("latin-1", "replace")


def fetch_euref_nrt_rinex(
    station: str,
    year: int,
    doy: int,
    hour: int,
    work_dir: Path,
    *,
    dir_template: str = DEFAULT_EUREF_NRT_DIR_TMPL,
    lister: Callable[[str], str] = _http_listing,
    fetcher: Callable = urllib.request.urlretrieve,
) -> Path | None:
    """Fetch + de-Hatanaka a EUREF nrt hourly RINEX-3 obs → an rnx2rtkp-readable
    ``.rnx`` path, or None on failure.

    ``station`` is the mount / 9+char monument (STR mount ``SHOE00GBR0`` →
    monument ``SHOE00GBR``).  Lists ``nrt/{doy}/{hour}/`` and matches
    ``{MONUMENT9}_[RS]_{YYYY}{DDD}{HH}00_01H_30S_MO.crx.gz``, then decompresses
    the ``.crx.gz`` (gzip + Hatanaka in one) via the ``hatanaka`` package —
    a plain gunzip would leave a ``.crx`` rnx2rtkp can't read (main #268).
    """
    import re as _re

    monument = station.upper()[:9]
    dir_url = dir_template.format(doy=doy, hour=hour)
    try:
        listing = lister(dir_url)
    except Exception as e:  # noqa: BLE001 - unreachable archive is non-fatal
        log.warning("EUREF nrt dir list failed (%s): %s", dir_url, e)
        return None
    pat = _re.compile(
        rf"{_re.escape(monument)}_[RS]_{year:04d}{doy:03d}{hour:02d}00"
        r"_01H_30S_MO\.crx\.gz")
    m = pat.search(listing)
    if not m:
        log.info("EUREF nrt: no %s obs in %s", monument, dir_url)
        return None
    fname = m.group(0)
    work_dir.mkdir(parents=True, exist_ok=True)
    gz_path = work_dir / fname
    try:
        fetcher(dir_url + fname, str(gz_path))
    except Exception as e:  # noqa: BLE001 - network/HTTP errors are non-fatal
        log.warning("EUREF nrt fetch failed (%s): %s", fname, e)
        return None
    try:
        from hatanaka import decompress_on_disk
        rnx_path = decompress_on_disk(str(gz_path))
    except Exception as e:  # noqa: BLE001 - missing pkg / bad file is non-fatal
        log.warning("EUREF nrt de-Hatanaka failed (needs the 'hatanaka' pip "
                    "package): %s", e)
        return None
    return Path(rnx_path)


# ── station catalogues (archives with no NTRIP sourcetable) ────────── #

# NGS publishes every CORS ARP in one daily-refreshed file: ~2600 rows of
#   SITE EPOCH lat(d m s N/S) lon(d m s E/W) ellHt Vn Ve Vu ctry state status
# in ITRF2020 @ 2020.00 with computed velocities.  This is the *only* way to
# rank NGS CORS by distance — the archive is plain HTTP with no sourcetable.
DEFAULT_NGS_CORS_CATALOG_URL = (
    "https://geodesy.noaa.gov/corsdata/coord/coord_20/itrf2020_geo.comp.txt")

# Statuses worth offering as a base.  "Operational" is the only one that
# guarantees current data; the rest are historical or not-a-CORS.
CATALOG_USABLE_STATUS = ("Operational",)


@dataclass(frozen=True)
class CatalogStation:
    """One station from an archive's own catalogue (no NTRIP sourcetable).

    ``lat``/``lon``/``height_m`` are the ARP in the catalogue's native frame at
    ``epoch`` (ITRF2020 @ 2020.00 for NGS), with ``vel_mm_yr`` = (Vn, Ve, Vu).
    Discovery uses lat/lon for *ranking only* — the baseline solve still takes
    the base coordinate from the base RINEX header and pre-converts it from the
    region's ``base_realization``, so this frame never reaches the solution.
    """
    station: str                 # 4-char CORS ID, upper-case
    lat: float
    lon: float
    height_m: float
    epoch: float
    vel_mm_yr: tuple[float, float, float]
    country: str
    state: str
    status: str


def _dms(deg: str, minute: str, sec: str, hemi: str) -> float:
    v = abs(float(deg)) + float(minute) / 60.0 + float(sec) / 3600.0
    return -v if hemi.upper() in ("S", "W") else v


def parse_ngs_cors_catalog(text: str) -> list[CatalogStation]:
    """Parse NGS's ``itrf2020_geo.comp.txt`` into CatalogStations.

    Header/rule lines and any row that isn't the expected 17 whitespace-
    separated fields are skipped, so a format tweak degrades to "fewer
    stations" (→ PRIDE floor) rather than a crash.
    """
    out: list[CatalogStation] = []
    for ln in text.splitlines():
        f = ln.split()
        if len(f) != 17:
            continue
        try:
            lat = _dms(f[2], f[3], f[4], f[5])
            lon = _dms(f[6], f[7], f[8], f[9])
            station = CatalogStation(
                station=f[0].upper(), lat=lat, lon=lon,
                height_m=float(f[10]), epoch=float(f[1]),
                vel_mm_yr=(float(f[11]), float(f[12]), float(f[13])),
                country=f[14], state=f[15], status=f[16])
        except ValueError:
            continue
        if not (-90 <= station.lat <= 90 and -180 <= station.lon <= 180):
            continue
        out.append(station)
    return out


def default_catalog_cache_dir() -> Path:
    """Where fetched catalogues are cached between runs."""
    root = os.environ.get("XDG_CACHE_HOME") or os.path.expanduser("~/.cache")
    return Path(root) / "peppar-survey"


def fetch_catalog_text(
    url: str,
    *,
    cache_dir: Path | None = None,
    max_age_s: float = 30 * 86400.0,
    fetcher: Callable[[str], str] = _http_listing,
) -> str | None:
    """Fetch a catalogue, caching it on disk.

    The NGS catalogue is 300 KB and moves at the pace of new monuments, so a
    30-day cache is plenty and keeps a field run working offline.  A stale
    cache beats no catalogue: if the refetch fails we return the cached copy
    rather than dropping to the PRIDE floor for want of a network.
    """
    cache_dir = cache_dir or default_catalog_cache_dir()
    cache_path = cache_dir / re.sub(r"[^A-Za-z0-9._-]", "_", url.split("/")[-1])
    fresh = False
    try:
        age = time.time() - cache_path.stat().st_mtime
        fresh = age <= max_age_s
    except OSError:
        pass
    if fresh:
        try:
            return cache_path.read_text("latin-1")
        except OSError as e:  # noqa: BLE001 - unreadable cache just refetches
            log.debug("catalog cache unreadable (%s): %s", cache_path, e)
    try:
        text = fetcher(url)
    except Exception as e:  # noqa: BLE001 - unreachable archive is non-fatal
        log.warning("catalog fetch failed (%s): %s", url, e)
        try:
            return cache_path.read_text("latin-1")   # stale > nothing
        except OSError:
            return None
    try:
        cache_dir.mkdir(parents=True, exist_ok=True)
        cache_path.write_text(text, "latin-1")
    except OSError as e:  # noqa: BLE001 - un-cacheable is not fatal
        log.debug("catalog cache write failed (%s): %s", cache_path, e)
    return text


def rank_catalog_by_distance(
    stations: Sequence[CatalogStation],
    lat: float,
    lon: float,
    *,
    max_km: float = 80.0,
    usable_status: Sequence[str] = CATALOG_USABLE_STATUS,
) -> list[tuple[float, CatalogStation]]:
    """(distance_km, CatalogStation) sorted nearest-first, within ``max_km``.

    Decommissioned / Non-Operational / Suspended stations are dropped — they
    have no current data to difference against."""
    out: list[tuple[float, CatalogStation]] = []
    for st in stations:
        if usable_status and st.status not in usable_status:
            continue
        d = haversine_km(lat, lon, st.lat, st.lon)
        if d <= max_km:
            out.append((d, st))
    out.sort(key=lambda t: t[0])
    return out


# ── region -> archive mapping ──────────────────────────────────────── #


@dataclass(frozen=True)
class RegionSource:
    """One region -> open-archive mapping.

    ``bbox`` = (lat_min, lat_max, lon_min, lon_max).  ``base_realization`` is the
    datum the archive's station coordinates carry — the value S1's --baseline
    pre-converts from -> ITRF2020@epoch (NAD83(2011) for NGS CORS, ETRS89 for
    EUREF).  ``kind`` selects the fetcher.

    ``catalog_url`` is how the archive's stations are *discovered*.  An archive
    that publishes its own station catalogue (NGS CORS) sets it and needs no
    caster; one whose stations are only enumerable via an NTRIP sourcetable
    (EUREF, whose mounts carry the 9-char monument the archive is keyed by)
    leaves it None and relies on ``discover_base(caster_host=...)``.
    """
    name: str
    kind: str                    # "ngs_cors" | "euref_nrt"
    bbox: tuple[float, float, float, float]
    base_realization: str
    catalog_url: str | None = None

    def contains(self, lat: float, lon: float) -> bool:
        lo_a, hi_a, lo_o, hi_o = self.bbox
        return lo_a <= lat <= hi_a and lo_o <= lon <= hi_o


# Ordered most-specific-first; source_for_position returns the first match.
REGION_SOURCES: tuple[RegionSource, ...] = (
    RegionSource("NGS CORS (North America)", "ngs_cors",
                 (15.0, 72.0, -170.0, -50.0), "NAD83(2011)",
                 catalog_url=DEFAULT_NGS_CORS_CATALOG_URL),
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
    via: str = "sourcetable"     # "catalog" | "sourcetable" — how it was found


def _discover_from_catalog(
    lat: float,
    lon: float,
    src: RegionSource,
    max_km: float,
    catalog_fetcher: Callable[[str], str | None],
) -> BaseDescriptor | None:
    """Rank the region archive's own station catalogue."""
    text = catalog_fetcher(src.catalog_url)
    if not text:
        return None
    ranked = rank_catalog_by_distance(
        parse_ngs_cors_catalog(text), lat, lon, max_km=max_km)
    if not ranked:
        log.info("No operational %s station within %.0f km of %.4f,%.4f",
                 src.name, max_km, lat, lon)
        return None
    dist, st = ranked[0]
    log.info("Nearest base: %s @ %.1f km (%s catalogue, %s/%s, datum %s)",
             st.station, dist, src.name, st.country, st.state,
             src.base_realization)
    return BaseDescriptor(station=st.station, distance_km=dist, source=src,
                          base_realization=src.base_realization, via="catalog")


def discover_base(
    lat: float,
    lon: float,
    *,
    caster_host: str | None = None,
    caster_port: int = 2101,
    max_km: float = 80.0,
    sourcetable_fetcher: Callable[[str, int], str] = fetch_sourcetable,
    catalog_fetcher: Callable[[str], str | None] = fetch_catalog_text,
) -> BaseDescriptor | None:
    """Find the nearest usable base station to (lat, lon).

    Selects the region source (NGS CORS / EUREF) for logging + the datum
    realization, then ranks that region's stations two ways, catalogue first:

      1. **The archive's own station catalogue** (``src.catalog_url``), when it
         publishes one.  Required for NGS CORS: ~1650 operational stations that
         appear in *no* NTRIP sourcetable, so this is the only way to see them.
         It also keeps the station name a real 4-char CORS ID, which is what
         ``fetch_cors_rinex`` is keyed by — a caster mount name would 404.
      2. **An NTRIP caster's sourcetable**, when ``caster_host`` is given.  This
         is how EUREF is enumerated (its mounts carry the 9-char monument the
         archive is keyed by), and the fallback anywhere a catalogue misses.

    Returns a BaseDescriptor or None (no region / nothing reachable / nothing in
    range), in which case the caller falls back to the PRIDE floor.  Fetching
    the base's RINEX is the fetcher's job (fetch_cors_rinex /
    fetch_euref_nrt_rinex), driven by the descriptor's source.kind.
    """
    src = source_for_position(lat, lon)
    if src is None:
        log.info("No region source for %.4f,%.4f — fall back to PRIDE floor",
                 lat, lon)
        return None
    if src.catalog_url:
        found = _discover_from_catalog(lat, lon, src, max_km, catalog_fetcher)
        if found is not None:
            return found
    if caster_host is None:
        log.info("Region %s selected (datum %s) but no base found and no "
                 "caster given for sourcetable ranking",
                 src.name, src.base_realization)
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
                          base_realization=src.base_realization,
                          via="sourcetable")


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
          f"datum {desc.base_realization}  ({desc.source.kind}, via {desc.via})")
    return 0


__all__ = [
    "haversine_km", "Mount", "fetch_sourcetable", "parse_sourcetable",
    "rank_by_distance", "RegionSource", "REGION_SOURCES", "source_for_position",
    "fetch_euref_nrt_rinex", "fetch_cors_rinex", "BaseDescriptor",
    "discover_base", "fetch_base_rinex",
    "CatalogStation", "DEFAULT_NGS_CORS_CATALOG_URL", "CATALOG_USABLE_STATUS",
    "parse_ngs_cors_catalog", "fetch_catalog_text", "rank_catalog_by_distance",
    "default_catalog_cache_dir",
]


if __name__ == "__main__":
    import sys as _sys
    _sys.exit(_main())

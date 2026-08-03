"""Tests for peppar-survey base discovery (S2 of I-071401).

All network I/O is injected, so ranking + region-selection are deterministic.
"""
from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import mock

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.peppar_survey_discovery import (
    BaseDescriptor, Mount, RegionSource, discover_base, fetch_base_rinex,
    fetch_catalog_text, fetch_euref_nrt_rinex, haversine_km,
    parse_ngs_cors_catalog, parse_sourcetable, rank_by_distance,
    rank_catalog_by_distance, source_for_position,
)


def _str_record(mount, lat, lon, *, nmea="0", soln="0", fmt="RTCM 3.2",
                nav="GPS+GLO", country="USA"):
    # STR;mount;id;fmt;fmt-details;carrier;nav;network;country;lat;lon;nmea;soln;...
    f = ["STR", mount, mount, fmt, "1004", "2", nav, "NET", country,
         f"{lat}", f"{lon}", nmea, soln, "", "", "", "", "", ""]
    return ";".join(f)


class HaversineTest(unittest.TestCase):
    def test_known_distance(self):
        # London -> Paris ~ 344 km
        d = haversine_km(51.5074, -0.1278, 48.8566, 2.3522)
        self.assertAlmostEqual(d, 344, delta=5)

    def test_zero_distance(self):
        self.assertAlmostEqual(haversine_km(40, -90, 40, -90), 0.0, places=6)


class ParseSourcetableTest(unittest.TestCase):
    def test_parses_str_records(self):
        text = ("CAS;caster;...\n"
                + _str_record("NEAR", 41.85, -88.10) + "\n"
                + _str_record("FAR", 51.5, -0.06) + "\n"
                "NET;net;...\nENDSOURCETABLE\n")
        mounts = parse_sourcetable(text)
        self.assertEqual([m.mount for m in mounts], ["NEAR", "FAR"])
        self.assertEqual(mounts[0].lat, 41.85)

    def test_skips_malformed_and_null_island(self):
        text = ("STR;short;only;three\n"          # too few fields
                + _str_record("ZEROS", 0.0, 0.0) + "\n"     # null island
                + _str_record("BADLAT", 999.0, 10.0) + "\n"  # out of range
                + _str_record("OK", 41.85, -88.10) + "\n")
        self.assertEqual([m.mount for m in parse_sourcetable(text)], ["OK"])

    def test_vrs_flag(self):
        text = (_str_record("VRSNET", 41.85, -88.10, soln="1") + "\n"
                + _str_record("FIXED", 41.86, -88.11, soln="0") + "\n")
        by = {m.mount: m for m in parse_sourcetable(text)}
        self.assertTrue(by["VRSNET"].vrs)
        self.assertFalse(by["FIXED"].vrs)


class RankByDistanceTest(unittest.TestCase):
    def _mounts(self):
        return [
            Mount("A", "RTCM", "GPS", "USA", 41.90, -88.10, False),   # ~5 km
            Mount("B", "RTCM", "GPS", "USA", 42.20, -88.10, False),   # ~40 km
            Mount("FARAWAY", "RTCM", "GPS", "USA", 45.0, -88.10, False),  # ~350 km
            Mount("VRS", "RTCM", "GPS", "USA", 41.85, -88.10, True),   # ~0 km, VRS
        ]

    def test_nearest_first_within_max_km(self):
        ranked = rank_by_distance(self._mounts(), 41.85, -88.10, max_km=80)
        self.assertEqual([m.mount for _d, m in ranked], ["A", "B"])
        self.assertLess(ranked[0][0], ranked[1][0])

    def test_excludes_vrs_by_default(self):
        ranked = rank_by_distance(self._mounts(), 41.85, -88.10, max_km=80)
        self.assertNotIn("VRS", [m.mount for _d, m in ranked])

    def test_max_km_filters(self):
        ranked = rank_by_distance(self._mounts(), 41.85, -88.10, max_km=10)
        self.assertEqual([m.mount for _d, m in ranked], ["A"])


class SourceForPositionTest(unittest.TestCase):
    def test_chicago_is_ngs_cors_nad83(self):
        src = source_for_position(41.85, -88.10)
        self.assertEqual(src.kind, "ngs_cors")
        self.assertEqual(src.base_realization, "NAD83(2011)")

    def test_london_is_euref_etrs89(self):
        src = source_for_position(51.4946, -0.0613)
        self.assertEqual(src.kind, "euref_nrt")
        self.assertEqual(src.base_realization, "ETRS89")

    def test_mid_ocean_returns_none(self):
        self.assertIsNone(source_for_position(0.0, -30.0))


class EurefFetchTest(unittest.TestCase):
    # EUREF nrt serves RINEX-3 long-named Hatanaka obs; the R/S source char
    # varies per station, so the fetcher lists the hour dir and matches.
    _LISTING = (
        '<a href="OTHR00XXX_R_20261631000_01H_30S_MO.crx.gz">o</a>\n'
        '<a href="SHOE00GBR_S_20261631000_01H_30S_MO.crx.gz">o</a>\n'
        '<a href="SHOE00GBR_S_20261631000_01H_30S_MN.rnx.gz">nav</a>\n')

    def test_matches_rinex3_longname_and_de_hatanakas(self):
        captured = {}

        def fake_lister(url):
            captured["dir"] = url
            return self._LISTING

        def fake_fetcher(url, dest):
            captured["url"] = url
            Path(dest).write_bytes(b"fake crx.gz bytes")

        # A plain gunzip would leave a .crx rnx2rtkp can't read; assert the
        # hatanaka de-compress step runs and its .rnx output is returned.
        def fake_dehat(p):
            captured["dehat_in"] = p
            out = Path(p).with_name(Path(p).name.replace(".crx.gz", ".rnx"))
            out.write_bytes(b"RINEX3 obs")
            return str(out)

        with TemporaryDirectory() as td, \
                mock.patch("hatanaka.decompress_on_disk", side_effect=fake_dehat):
            out = fetch_euref_nrt_rinex(
                "SHOE00GBR0", 2026, 163, 10, Path(td),
                lister=fake_lister, fetcher=fake_fetcher)
            self.assertIsNotNone(out)
            self.assertTrue(str(out).endswith(".rnx"))
            self.assertEqual(out.read_bytes(), b"RINEX3 obs")
            self.assertTrue(captured["dir"].endswith("/nrt/163/10/"))
            # Matched the _S_ source char (varies) via the 9-char monument.
            self.assertIn("SHOE00GBR_S_20261631000_01H_30S_MO.crx.gz",
                          captured["url"])
            self.assertIn(".crx.gz", captured["dehat_in"])

    def test_no_matching_file_returns_none(self):
        with TemporaryDirectory() as td:
            self.assertIsNone(fetch_euref_nrt_rinex(
                "SHOE00GBR0", 2026, 999, 10, Path(td),
                lister=lambda u: self._LISTING, fetcher=lambda u, d: None))

    def test_fetch_failure_returns_none(self):
        def boom(url, dest):
            raise OSError("404")
        with TemporaryDirectory() as td:
            self.assertIsNone(fetch_euref_nrt_rinex(
                "SHOE00GBR0", 2026, 163, 10, Path(td),
                lister=lambda u: self._LISTING, fetcher=boom))

    def test_dir_list_failure_returns_none(self):
        def boom(url):
            raise OSError("archive down")
        with TemporaryDirectory() as td:
            self.assertIsNone(fetch_euref_nrt_rinex(
                "SHOE00GBR0", 2026, 163, 10, Path(td), lister=boom))


CATALOG_HEADER = (
    "August 02, 2026\n"
    "\n"
    "ITRF2020 Geod. CORS Pos. Ant. Ref. Point (ARP) [GRS80 Ellip.] "
    "Computed Vels.\n"
    "_______________________________________________________________\n"
    " SITE   EPOCH       Latitude          Longitude     Ellip. Ht.   "
    "Vn      Ve      Vu    Ctry.  State      Site \n"
    "                                                     meters    "
    "mm/yr   mm/yr   mm/yr   code    code     Status\n"
    "_______________________________________________________________\n"
)

# Verbatim rows from the real file (2026-08-02), plus a synthetic
# Decommissioned one to prove the status filter bites.
CATALOG_ROWS = (
    "WMTW  2020.00  44  9 15.47112 N   87 41 36.19955 W   179.585"
    "     0.1   -16.3    -3.4    US      WI      Operational\n"
    "WIWB  2020.00  43 25 13.99813 N   88  8 55.50334 W   234.087"
    "     0.0   -16.0    -3.0    US      WI      Operational\n"
    "DEAD  2020.00  44  0  0.00000 N   87 46  0.00000 W   180.000"
    "     0.0   -16.0    -3.0    US      WI      Decommissioned\n"
)
CATALOG_TEXT = CATALOG_HEADER + CATALOG_ROWS

# Newton, WI — the 2026-08-03 field site that motivated catalogue discovery.
NEWTON = (43.98, -87.78)


class ParseNgsCorsCatalogTest(unittest.TestCase):
    def test_parses_dms_and_metadata(self):
        rows = parse_ngs_cors_catalog(CATALOG_TEXT)
        self.assertEqual([r.station for r in rows], ["WMTW", "WIWB", "DEAD"])
        wmtw = rows[0]
        self.assertAlmostEqual(wmtw.lat, 44.154297, places=5)
        self.assertAlmostEqual(wmtw.lon, -87.693389, places=5)   # W -> negative
        self.assertAlmostEqual(wmtw.height_m, 179.585, places=3)
        self.assertAlmostEqual(wmtw.epoch, 2020.0, places=2)
        self.assertEqual(wmtw.vel_mm_yr, (0.1, -16.3, -3.4))
        self.assertEqual((wmtw.country, wmtw.state), ("US", "WI"))
        self.assertEqual(wmtw.status, "Operational")

    def test_skips_header_and_malformed_rows(self):
        text = CATALOG_HEADER + "GARBAGE not a row\n" + CATALOG_ROWS
        self.assertEqual(len(parse_ngs_cors_catalog(text)), 3)

    def test_empty_text_is_empty_list(self):
        self.assertEqual(parse_ngs_cors_catalog(""), [])


class RankCatalogTest(unittest.TestCase):
    def setUp(self):
        self.rows = parse_ngs_cors_catalog(CATALOG_TEXT)

    def test_nearest_first_and_distance(self):
        ranked = rank_catalog_by_distance(self.rows, *NEWTON, max_km=200)
        self.assertEqual([s.station for _d, s in ranked], ["WMTW", "WIWB"])
        # WMTW is ~20 km from Newton — the whole point of the exercise.
        self.assertAlmostEqual(ranked[0][0], 20.7, delta=1.5)

    def test_excludes_non_operational(self):
        # DEAD sits ~2 km away but is Decommissioned, so it must not win.
        ranked = rank_catalog_by_distance(self.rows, *NEWTON, max_km=200)
        self.assertNotIn("DEAD", [s.station for _d, s in ranked])

    def test_max_km_filters(self):
        ranked = rank_catalog_by_distance(self.rows, *NEWTON, max_km=25)
        self.assertEqual([s.station for _d, s in ranked], ["WMTW"])


class FetchCatalogTextTest(unittest.TestCase):
    def test_caches_and_reuses(self):
        calls = []

        def fetcher(url):
            calls.append(url)
            return CATALOG_TEXT

        with TemporaryDirectory() as td:
            kw = dict(cache_dir=Path(td), fetcher=fetcher)
            self.assertEqual(fetch_catalog_text("http://x/cat.txt", **kw),
                             CATALOG_TEXT)
            self.assertEqual(fetch_catalog_text("http://x/cat.txt", **kw),
                             CATALOG_TEXT)
        self.assertEqual(len(calls), 1)          # second call served from cache

    def test_stale_cache_beats_failed_fetch(self):
        """A field run with no network still gets a base."""
        def boom(url):
            raise OSError("no route to host")

        with TemporaryDirectory() as td:
            cache = Path(td)
            (cache / "cat.txt").write_text(CATALOG_TEXT, "latin-1")
            got = fetch_catalog_text("http://x/cat.txt", cache_dir=cache,
                                     max_age_s=-1, fetcher=boom)
        self.assertEqual(got, CATALOG_TEXT)

    def test_failed_fetch_without_cache_returns_none(self):
        def boom(url):
            raise OSError("no route to host")

        with TemporaryDirectory() as td:
            self.assertIsNone(fetch_catalog_text(
                "http://x/cat.txt", cache_dir=Path(td), fetcher=boom))


class DiscoverBaseTest(unittest.TestCase):
    # NGS CORS now has a catalogue, so every sourcetable-path test must say
    # explicitly that the catalogue found nothing — otherwise it would hit
    # the network.
    NO_CATALOG = staticmethod(lambda url: None)

    def test_picks_nearest_fixed_base_with_region_datum(self):
        table = (_str_record("VRSNET", 41.851, -88.101, soln="1") + "\n"
                 + _str_record("CLOSE", 41.90, -88.10) + "\n"
                 + _str_record("FAR", 42.5, -88.10) + "\n")
        desc = discover_base(
            41.85, -88.10, caster_host="caster.example", caster_port=2101,
            sourcetable_fetcher=lambda h, p: table,
            catalog_fetcher=self.NO_CATALOG)
        self.assertIsInstance(desc, BaseDescriptor)
        self.assertEqual(desc.station, "CLOSE")          # nearest non-VRS
        self.assertEqual(desc.base_realization, "NAD83(2011)")  # region datum
        self.assertEqual(desc.via, "sourcetable")

    def test_catalog_wins_over_caster_in_ngs_region(self):
        """Newton WI: the CORS catalogue must find WMTW even though the only
        caster mount in range is 200+ km away (the 2026-08-03 regression)."""
        far = _str_record("RTK2GO", 42.16, -88.29) + "\n"   # ~207 km
        desc = discover_base(
            *NEWTON, caster_host="rtk2go.com",
            sourcetable_fetcher=lambda h, p: far,
            catalog_fetcher=lambda url: CATALOG_TEXT)
        self.assertIsNotNone(desc)
        self.assertEqual(desc.station, "WMTW")
        self.assertEqual(desc.via, "catalog")
        self.assertLess(desc.distance_km, 25.0)

    def test_catalog_works_with_no_caster_at_all(self):
        """--auto in North America needs no --caster-host."""
        desc = discover_base(*NEWTON, catalog_fetcher=lambda url: CATALOG_TEXT)
        self.assertIsNotNone(desc)
        self.assertEqual(desc.station, "WMTW")

    def test_catalog_miss_falls_through_to_sourcetable(self):
        table = _str_record("CLOSE", 43.99, -87.79) + "\n"
        desc = discover_base(
            *NEWTON, caster_host="c", max_km=10,   # WMTW at 20 km is excluded
            sourcetable_fetcher=lambda h, p: table,
            catalog_fetcher=lambda url: CATALOG_TEXT)
        self.assertIsNotNone(desc)
        self.assertEqual(desc.station, "CLOSE")
        self.assertEqual(desc.via, "sourcetable")

    def test_euref_region_has_no_catalog_and_uses_caster(self):
        def no_catalog(url):
            raise AssertionError("EUREF must not consult a catalogue")

        table = _str_record("SHOE00GBR0", 51.50, -0.13, country="GBR") + "\n"
        desc = discover_base(
            51.5074, -0.1278, caster_host="c",
            sourcetable_fetcher=lambda h, p: table,
            catalog_fetcher=no_catalog)
        self.assertIsNotNone(desc)
        self.assertEqual(desc.base_realization, "ETRS89")
        self.assertEqual(desc.via, "sourcetable")

    def test_no_region_returns_none(self):
        self.assertIsNone(discover_base(
            0.0, -30.0, caster_host="c", sourcetable_fetcher=lambda h, p: "",
            catalog_fetcher=self.NO_CATALOG))

    def test_no_caster_and_no_catalog_returns_none(self):
        self.assertIsNone(discover_base(
            41.85, -88.10, catalog_fetcher=self.NO_CATALOG))

    def test_unreachable_caster_returns_none(self):
        def boom(h, p):
            raise OSError("refused")
        self.assertIsNone(discover_base(
            41.85, -88.10, caster_host="c", sourcetable_fetcher=boom,
            catalog_fetcher=self.NO_CATALOG))

    def test_nothing_in_range_returns_none(self):
        table = _str_record("FAR", 45.0, -88.10) + "\n"  # ~350 km
        self.assertIsNone(discover_base(
            41.85, -88.10, caster_host="c", max_km=80,
            sourcetable_fetcher=lambda h, p: table,
            catalog_fetcher=self.NO_CATALOG))


class FetchBaseRinexTest(unittest.TestCase):
    """The glue that feeds S1 a base: dispatch to the source's archive fetcher."""

    def _desc(self, kind, realization):
        src = RegionSource("R", kind, (0, 90, -180, 180), realization)
        return BaseDescriptor("STAT", 12.3, src, realization)

    def test_ngs_cors_dispatch(self):
        calls = {}

        def cors(station, year, doy, work_dir):
            calls["cors"] = (station, year, doy)
            return Path("/tmp/base.obs")

        def euref(*a, **k):  # must not fire
            calls["euref"] = True
            return None

        out = fetch_base_rinex(
            self._desc("ngs_cors", "NAD83(2011)"), 2026, 163, Path("/tmp"),
            cors_fetcher=cors, euref_fetcher=euref)
        self.assertEqual(str(out), "/tmp/base.obs")
        self.assertEqual(calls["cors"], ("STAT", 2026, 163))
        self.assertNotIn("euref", calls)

    def test_euref_dispatch_passes_hour(self):
        calls = {}

        def euref(station, year, doy, hour, work_dir):
            calls["euref"] = (station, year, doy, hour)
            return Path("/tmp/e.obs")

        out = fetch_base_rinex(
            self._desc("euref_nrt", "ETRS89"), 2026, 163, Path("/tmp"),
            hour=10, cors_fetcher=lambda *a: None, euref_fetcher=euref)
        self.assertEqual(str(out), "/tmp/e.obs")
        self.assertEqual(calls["euref"], ("STAT", 2026, 163, 10))


if __name__ == "__main__":
    unittest.main()

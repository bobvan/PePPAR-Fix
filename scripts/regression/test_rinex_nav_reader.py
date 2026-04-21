"""Unit tests for RINEX 3.x NAV reader."""

from __future__ import annotations

import math
import os
import tempfile
import unittest
from pathlib import Path

from regression.rinex_nav_reader import (
    parse_header, iter_nav_records, load_into_ephemeris,
    _parse_d, _parse_continuation,
)


# Minimal synthetic RINEX 3.04 NAV blob.  Real G01 2023-01-01 00:00 record
# taken from PRIDE's brdm0010.23p, formatted as RINEX 3.04 requires
# (4 leading spaces on continuation lines, 19-char F19.12 D-notation).
_SYNTHETIC_NAV = """     3.04           N: GNSS NAV DATA    M: MIXED            RINEX VERSION / TYPE
pytest              PePPAR              20260420 000000 UTC PGM / RUN BY / DATE
                                                            END OF HEADER
G01 2023 01 01 00 00 00 2.302187494934E-04-5.002220859751E-12 0.000000000000E+00
     7.900000000000E+01-6.525000000000E+01 3.858017844786E-09-1.740726997867E-01
    -3.341585397720E-06 1.216053694952E-02 8.400529623032E-06 5.153658443451E+03
     0.000000000000E+00-2.048909664154E-08-1.383791931506E+00 1.452863216400E-07
     9.890568716058E-01 2.307812500000E+02 9.379170737708E-01-8.009262189347E-09
    -1.464346710204E-10 1.000000000000E+00 2.243000000000E+03 0.000000000000E+00
     2.000000000000E+00 0.000000000000E+00 4.656612873077E-09 7.900000000000E+01
    -6.720000000000E+02 4.000000000000E+00
E01 2023 01 01 00 10 00-6.196172721684E-04-8.100187187246E-12 0.000000000000E+00
     5.300000000000E+01 4.175000000000E+01 2.725729878547E-09-1.659843497729E+00
     1.951679587364E-06 1.768948603421E-04 9.100511670113E-06 5.440609176636E+03
     6.000000000000E+02-3.166496753693E-08-8.872318349054E-01-4.284083843231E-08
     9.869527987823E-01 1.421250000000E+02-7.180137093714E-01-5.437023920020E-09
     5.303802830108E-10 5.160000000000E+02 2.243000000000E+03 0.000000000000E+00
     3.120000000000E+00 0.000000000000E+00-6.519258022308E-09-6.985664367676E-09
     6.000000000000E+02 0.000000000000E+00
C19 2023 01 01 00 00 00-5.823578080162E-04-1.014254257488E-11 0.000000000000E+00
     1.000000000000E+00-1.003125000000E+02 5.064459637820E-09-9.928810293000E-01
    -3.193085500000E-06 3.580130915716E-04 3.120628372440E-06 5.282614013672E+03
     5.184000000000E+05 7.683410644531E-08 2.181895310898E-01-3.166496753693E-08
     9.554517746773E-01-5.159375000000E+02 2.155927748510E+00-6.465938358829E-09
    -5.500155957900E-10 0.000000000000E+00 8.860000000000E+02 0.000000000000E+00
     2.000000000000E+00 0.000000000000E+00-6.400000000000E-09 3.300000000000E-10
     5.184000000000E+05 0.000000000000E+00
"""


class ParseHelpersTest(unittest.TestCase):
    def test_parse_d_standard_e(self):
        self.assertAlmostEqual(_parse_d(" 1.234E-05 "), 1.234e-5)

    def test_parse_d_fortran_d(self):
        self.assertAlmostEqual(_parse_d(" 1.234D-05 "), 1.234e-5)

    def test_parse_d_blank_returns_none(self):
        self.assertIsNone(_parse_d("   "))

    def test_parse_continuation_four_fields(self):
        # Standard RINEX 3 continuation line: 4 leading spaces, 4 values.
        line = (
            "     7.900000000000E+01"
            "-6.525000000000E+01"
            " 3.858017844786E-09"
            "-1.740726997867E-01"
        )
        vals = _parse_continuation(line)
        self.assertEqual(len(vals), 4)
        self.assertAlmostEqual(vals[0], 79.0)
        self.assertAlmostEqual(vals[1], -65.25)
        self.assertAlmostEqual(vals[2], 3.858017844786e-9)
        self.assertAlmostEqual(vals[3], -1.740726997867e-1)


class ParseHeaderTest(unittest.TestCase):
    def test_parses_synthetic(self):
        with tempfile.NamedTemporaryFile("w", suffix=".p", delete=False) as f:
            f.write(_SYNTHETIC_NAV)
            path = Path(f.name)
        try:
            hdr = parse_header(path)
            self.assertEqual(hdr.version, "3.04")
        finally:
            path.unlink()


class IterNavRecordsTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.NamedTemporaryFile(
            "w", suffix=".p", delete=False)
        tmp.write(_SYNTHETIC_NAV)
        tmp.close()
        self.path = Path(tmp.name)
        self.records = list(iter_nav_records(self.path))

    def tearDown(self):
        self.path.unlink()

    def test_finds_three_records(self):
        self.assertEqual(len(self.records), 3)
        prns = [r[0] for r in self.records]
        self.assertEqual(prns, ["G01", "E01", "C19"])

    def test_gps_record_has_expected_fields(self):
        prn, eph = next(r for r in self.records if r[0] == "G01")
        self.assertEqual(prn, "G01")
        self.assertEqual(eph["system"], "G")
        self.assertEqual(eph["sat_id"], 1)
        self.assertAlmostEqual(eph["sqrt_a"], 5.153658443451e3)
        self.assertAlmostEqual(eph["e"], 1.216053694952e-2)
        # Angular fields should have been multiplied by π.
        # M0 stored = -1.740726997867e-1 semicircles →
        # expected = -1.740726997867e-1 * π radians.
        self.assertAlmostEqual(
            eph["M0"], -1.740726997867e-1 * math.pi, places=6,
        )
        self.assertEqual(eph["health"], 0)
        self.assertIsNotNone(eph["tgd"])

    def test_gal_record(self):
        prn, eph = next(r for r in self.records if r[0] == "E01")
        self.assertEqual(eph["system"], "E")
        self.assertAlmostEqual(eph["sqrt_a"], 5.440609176636e3)
        # BGD E1-E5b separate from TGD (which is BGD E1-E5a).
        self.assertIsNotNone(eph["bgd_e5b"])

    def test_bds_record_has_both_tgds(self):
        prn, eph = next(r for r in self.records if r[0] == "C19")
        self.assertEqual(eph["system"], "C")
        self.assertIsNotNone(eph["tgd"])
        self.assertIsNotNone(eph["tgd2"])


class LoadIntoEphemerisTest(unittest.TestCase):
    """Integration with BroadcastEphemeris — records loaded via the
    reader should be usable for sat_position() computations."""

    def test_populates_broadcast_ephemeris(self):
        from broadcast_eph import BroadcastEphemeris

        with tempfile.NamedTemporaryFile("w", suffix=".p", delete=False) as f:
            f.write(_SYNTHETIC_NAV)
            path = Path(f.name)
        try:
            beph = BroadcastEphemeris()
            n = load_into_ephemeris(path, beph)
            self.assertEqual(n, 3)
            self.assertEqual(set(beph.satellites), {"G01", "E01", "C19"})
            # Verify GM is populated.
            self.assertIn("gm", beph._ephs["G01"][0])
            self.assertIn("gm", beph._ephs["C19"][0])
        finally:
            path.unlink()


# Integration test: only runs if the PRIDE dataset is on disk.
_PRIDE_PATH_ENV = "PRIDE_DATA_DIR"


@unittest.skipIf(
    not os.environ.get(_PRIDE_PATH_ENV),
    f"set {_PRIDE_PATH_ENV} to run the PRIDE-bundled integration test"
)
class PrideNavIntegrationTest(unittest.TestCase):
    def test_parse_2023_brdm(self):
        nav_path = (
            Path(os.environ[_PRIDE_PATH_ENV]) / "2023/brdm0010.23p"
        )
        if not nav_path.exists():
            self.skipTest(f"{nav_path} not found")
        n = 0
        gps_count = 0
        gal_count = 0
        bds_count = 0
        for prn, eph in iter_nav_records(nav_path):
            n += 1
            if eph["system"] == "G":
                gps_count += 1
            elif eph["system"] == "E":
                gal_count += 1
            elif eph["system"] == "C":
                bds_count += 1
        self.assertGreater(n, 100, f"expected >100 NAV records, got {n}")
        self.assertGreater(gps_count, 0)
        self.assertGreater(gal_count, 0)
        self.assertGreater(bds_count, 0)

    def test_satellite_position_reasonable(self):
        """Feed the 2023 NAV into BroadcastEphemeris and ask for a
        sat_position; the result should be an ECEF vector with
        magnitude matching a typical GNSS orbit radius (~26000 km for
        GPS, ~30000 km for BDS GEO).
        """
        from broadcast_eph import BroadcastEphemeris
        from datetime import datetime, timezone

        nav_path = (
            Path(os.environ[_PRIDE_PATH_ENV]) / "2023/brdm0010.23p"
        )
        if not nav_path.exists():
            self.skipTest(f"{nav_path} not found")

        beph = BroadcastEphemeris()
        load_into_ephemeris(nav_path, beph)
        # Query at 2023-01-01 12:00 UTC — well inside the ephemeris
        # validity window (toc = 00:00).
        t = datetime(2023, 1, 1, 12, 0, 0, tzinfo=timezone.utc)
        found = 0
        for prn in ["G01", "G05", "E01", "E11", "C23"]:
            if prn not in beph._ephs:
                continue
            pos, clk = beph.sat_position(prn, t)
            if pos is None:
                continue
            radius_km = (pos[0]**2 + pos[1]**2 + pos[2]**2) ** 0.5 / 1000
            # GPS / Galileo MEO: ~26,560 km; BDS MEO: ~27,900 km;
            # BDS IGSO/GEO: ~42,164 km.  Accept anything 20,000-50,000.
            self.assertGreater(
                radius_km, 20_000,
                f"{prn} radius {radius_km:.0f} km looks wrong"
            )
            self.assertLess(radius_km, 50_000)
            found += 1
        self.assertGreater(found, 0, "expected ≥1 satellite position")


if __name__ == "__main__":
    unittest.main()

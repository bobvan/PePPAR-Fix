"""Unit tests for the IGS Bias-SINEX OSB reader."""

from __future__ import annotations

import os
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path

from regression.bias_sinex_reader import (
    BiasEntry, BiasTable, iter_bias_entries, load_bias_table,
    _parse_bias_line, _parse_sinex_time, cycles_to_meters,
)


# Synthetic Bias-SINEX file with the minimum needed structure.  Field
# columns follow the Bias-SINEX 1.00 spec (BIA cols 1..111).  The
# "OSB" rows are the satellite biases we care about; we also add a
# station-only entry (PRN blank) and a DSB entry to verify they get
# correctly skipped.
#
# Column layout, per spec (1-indexed in spec; 0-indexed in code):
#   spec col 1     = "*" (comment) or " " (data)
#   spec cols 2-4  = bias type ("OSB", "DSB", "ISB")
#   spec col 5     = space
#   spec cols 6-9  = SVN (4-char satellite vehicle number)
#   spec col 10    = space
#   spec cols 11-13 = PRN (e.g. "G01")
#   spec col 14    = space
#   spec cols 15-23 = STATION (9-char)
#   spec col 24    = space
#   spec cols 25-28 = OBS1 (4-char observation code)
#   spec col 29    = space
#   spec cols 30-33 = OBS2 (4-char optional)
#   spec col 34    = space
#   spec cols 35-48 = BIAS_START "YYYY:DOY:SSSSS"
#   spec col 49    = space
#   spec cols 50-63 = BIAS_END
#   spec col 64    = space
#   spec cols 65-67 = UNIT
#   spec col 68    = space (sometimes 1)
#   spec cols 69-89 = BIAS value (right-aligned signed F21.4 typical)
#   spec col 90+    = STDDEV
_SYNTHETIC_BIA = (
    "%=BIA 1.00 PYT 2026:111:00000 PYT 2020:001:00000 2020:002:00000\n"
    "*-------------------------------------------------------------------\n"
    "+FILE/REFERENCE\n"
    " DESCRIPTION       PePPAR Fix regression-harness synthetic bias file\n"
    "-FILE/REFERENCE\n"
    "+BIAS/SOLUTION\n"
    "*BIAS SVN_ PRN STATION__ OBS1 OBS2 BIAS_START____ BIAS_END______ UNIT __ESTIMATED_VALUE____ _STD_DEV___________\n"
    " OSB  G063 G01           L1C       2020:001:00000 2020:002:00000 cyc                0.250000          0.0010\n"
    " OSB  G063 G01           L5Q       2020:001:00000 2020:002:00000 cyc               -0.140000          0.0010\n"
    " OSB  G063 G01           C1C       2020:001:00000 2020:002:00000 ns                 1.250000          0.0500\n"
    " OSB  G063 G01           C2W       2020:001:00000 2020:002:00000 ns                -0.870000          0.0500\n"
    " OSB  E101 E01           L1C       2020:001:00000 2020:002:00000 cyc                0.310000          0.0010\n"
    " OSB  E101 E01           L5Q       2020:001:00000 2020:002:00000 cyc                0.180000          0.0010\n"
    " OSB              ABMF   C1C       2020:001:00000 2020:002:00000 ns                 0.500000          0.0500\n"
    " DSB  G063 G01           C1W  C2W  2020:001:00000 2020:002:00000 ns                -1.450000          0.0500\n"
    "-BIAS/SOLUTION\n"
    "%END BIA\n"
)
# Note: The "ABMF" entry above is a station-only bias (PRN blank), so
# `load_bias_table` should drop it.  The DSB entry must also be skipped
# (we want only OSB satellite-side biases).


class ParseSinexTimeTest(unittest.TestCase):
    def test_yyyy_doy(self):
        # 2020-01-01 00:00:00 UTC = day 1
        dt = _parse_sinex_time("2020:001:00000")
        self.assertEqual(
            dt, datetime(2020, 1, 1, 0, 0, 0, tzinfo=timezone.utc),
        )

    def test_within_day(self):
        dt = _parse_sinex_time("2020:002:43200")  # day 2, noon
        self.assertEqual(
            dt, datetime(2020, 1, 2, 12, 0, 0, tzinfo=timezone.utc),
        )

    def test_zero_means_open(self):
        # Spec: '0000:000:00000' = "open / not specified".
        self.assertIsNone(_parse_sinex_time("0000:000:00000"))


class ParseBiasLineTest(unittest.TestCase):
    def test_phase_osb(self):
        line = (
            " OSB  G063 G01           L1C       "
            "2020:001:00000 2020:002:00000 cyc                "
            "0.250000          0.0010"
        )
        e = _parse_bias_line(line)
        self.assertIsNotNone(e)
        self.assertEqual(e.bias_type, "OSB")
        self.assertEqual(e.sv, "G01")
        self.assertEqual(e.obs1, "L1C")
        self.assertEqual(e.unit, "cyc")
        self.assertAlmostEqual(e.value, 0.25)
        self.assertTrue(e.is_phase())
        self.assertFalse(e.is_code())

    def test_code_osb_in_ns(self):
        line = (
            " OSB  G063 G01           C1C       "
            "2020:001:00000 2020:002:00000 ns                 "
            "1.250000          0.0500"
        )
        e = _parse_bias_line(line)
        self.assertIsNotNone(e)
        self.assertEqual(e.unit, "ns")
        self.assertTrue(e.is_code())
        # 1.25 ns × c = ~0.3747 m
        self.assertAlmostEqual(e.as_meters(), 1.25e-9 * 299_792_458, places=4)

    def test_skips_comment_line(self):
        e = _parse_bias_line("*BIAS SVN PRN STATION ...")
        self.assertIsNone(e)

    def test_skips_short_line(self):
        e = _parse_bias_line(" too short")
        self.assertIsNone(e)


class IterAndLoadTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.NamedTemporaryFile(
            "w", suffix=".bia", delete=False)
        tmp.write(_SYNTHETIC_BIA)
        tmp.close()
        self.path = Path(tmp.name)

    def tearDown(self):
        self.path.unlink()

    def test_iter_yields_entries_inside_block(self):
        entries = list(iter_bias_entries(self.path))
        # 8 data lines inside +BIAS/SOLUTION
        self.assertEqual(len(entries), 8)

    def test_load_table_filters_correctly(self):
        table = load_bias_table(self.path)
        # OSB satellite-only entries: 6 (4 GPS + 2 Galileo)
        self.assertEqual(table.n_phase, 4)  # L1C × 2 + L5Q × 2
        self.assertEqual(table.n_code, 2)   # C1C, C2W (G01)
        self.assertEqual(table.n_skipped_station, 1)  # ABMF entry
        self.assertEqual(table.n_skipped_dsb, 1)      # DSB entry

        # Per-SV lookup
        self.assertTrue(table.has_sv("G01"))
        self.assertTrue(table.has_sv("E01"))

        e = table.get("G01", "L1C")
        self.assertIsNotNone(e)
        self.assertAlmostEqual(e.value, 0.25)

        e = table.get("G01", "C2W")
        self.assertIsNotNone(e)
        self.assertEqual(e.unit, "ns")
        self.assertAlmostEqual(e.value, -0.87)

        # Signal list
        self.assertCountEqual(
            table.signals_for("G01"),
            ["L1C", "L5Q", "C1C", "C2W"],
        )

    def test_timespan_extracted(self):
        table = load_bias_table(self.path)
        start, end = table.timespan
        self.assertEqual(start, datetime(2020, 1, 1, tzinfo=timezone.utc))
        self.assertEqual(end, datetime(2020, 1, 2, tzinfo=timezone.utc))


class CyclesToMetersTest(unittest.TestCase):
    def test_known_signal(self):
        # GPS-L1CA wavelength ≈ 0.1903m.  0.5 cycles → ~0.0951m.
        result = cycles_to_meters(0.5, "GPS-L1CA")
        if result is None:
            self.skipTest("signal_wavelengths module not importable")
        self.assertAlmostEqual(result, 0.5 * 0.190293672, places=6)

    def test_unknown_signal_returns_none(self):
        result = cycles_to_meters(0.5, "BOGUS-SIG")
        # Either None (when imported) or None (when not).  Either way
        # the function does not raise.
        self.assertIsNone(result)


# Optional integration test: only runs if the user has put a real
# Bias-SINEX file at the path pointed to by REGRESSION_BIA env var.
@unittest.skipIf(
    not os.environ.get("REGRESSION_BIA"),
    "set REGRESSION_BIA=/path/to/file.BIA to run integration test",
)
class RealBiaIntegrationTest(unittest.TestCase):
    def test_load_real_file(self):
        path = Path(os.environ["REGRESSION_BIA"])
        if not path.exists():
            self.skipTest(f"{path} not found")
        table = load_bias_table(path)
        # Real WUM/CNES files typically carry biases for ~30+ GPS, ~20+
        # Galileo, ~30+ BeiDou satellites.
        self.assertGreater(len(table.per_sv), 50,
                           f"only {len(table.per_sv)} SVs loaded")
        self.assertGreater(table.n_phase, 100)
        self.assertGreater(table.n_code, 100)


if __name__ == "__main__":
    unittest.main()

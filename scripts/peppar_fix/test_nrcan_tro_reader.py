"""Tests for nrcan_tro_reader — parse CSRS-PPP SINEX_TRO total-ZTD samples."""
import os
import sys
import unittest
from datetime import datetime, timezone

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix import nrcan_tro_reader as tro  # noqa: E402

# A minimal but realistic SINEX_TRO 2.00 fragment (CSRS-PPP dialect).
_TRO = """\
%=TRO 2.00 NRC 26:001:43200 NRC 26:001:00000 26:001:00600 P MIX
+TROP/DESCRIPTION
 SAMPLING INTERVAL                            300
 TROP MAPPING FUNCTION         WET NIELL
 SOLUTION_FIELDS_1             TROTOT STDDEV TGNTOT STDDEV TGETOT STDDEV
-TROP/DESCRIPTION
+TROP/SOLUTION
*SITE ____EPOCH___ TROTOT STDDEV TGNTOT STDDEV TGETOT STDDEV
 ABCD 26:001:00000 2451.6    1.2   -0.4    0.3    0.1    0.3
 ABCD 26:001:00300 2452.0    1.1   -0.3    0.3    0.2    0.3
 ABCD 26:001:00600 2453.4    1.0   -0.2    0.3    0.2    0.3
-TROP/SOLUTION
%ENDTRO
"""


class TestTroParse(unittest.TestCase):
    def test_parses_solution_rows(self):
        pts = tro.parse_tro_lines(_TRO.splitlines())
        self.assertEqual(len(pts), 3)
        p = pts[0]
        self.assertAlmostEqual(p.ztd_m, 2.4516, places=6)   # 2451.6 mm → m
        self.assertAlmostEqual(p.sigma_ztd_m, 0.0012, places=6)
        self.assertEqual(p.site, "ABCD")

    def test_epoch_is_doy_seconds_utc_calendar(self):
        pts = tro.parse_tro_lines(_TRO.splitlines())
        # 26:001:00000 → 2026 DOY 1 00:00:00 UTC
        want = datetime(2026, 1, 1, 0, 0, 0, tzinfo=timezone.utc).timestamp()
        self.assertAlmostEqual(pts[0].t_s, want, places=3)
        # +300 s cadence
        self.assertAlmostEqual(pts[1].t_s - pts[0].t_s, 300.0, places=3)

    def test_time_offset_applied(self):
        a = tro.parse_tro_lines(_TRO.splitlines())
        b = tro.parse_tro_lines(_TRO.splitlines(), time_offset_s=18.0)
        self.assertAlmostEqual(b[0].t_s - a[0].t_s, 18.0, places=3)

    def test_site_filter(self):
        lines = _TRO.replace(" ABCD 26:001:00300", " WXYZ 26:001:00300").splitlines()
        self.assertEqual(len(tro.parse_tro_lines(lines, site="ABCD")), 2)
        self.assertEqual(len(tro.parse_tro_lines(lines, site="WXYZ")), 1)

    def test_default_layout_without_solution_fields(self):
        # drop the SOLUTION_FIELDS line → falls back to TROTOT,STDDEV first two
        lines = [ln for ln in _TRO.splitlines()
                 if "SOLUTION_FIELDS" not in ln]
        pts = tro.parse_tro_lines(lines)
        self.assertEqual(len(pts), 3)
        self.assertAlmostEqual(pts[0].ztd_m, 2.4516, places=6)

    def test_reordered_solution_fields(self):
        # TROTOT not first → indices must come from SOLUTION_FIELDS, not position
        text = _TRO.replace(
            "SOLUTION_FIELDS_1             TROTOT STDDEV TGNTOT STDDEV TGETOT STDDEV",
            "SOLUTION_FIELDS_1             TGNTOT STDDEV TROTOT STDDEV").replace(
            " ABCD 26:001:00000 2451.6    1.2   -0.4    0.3    0.1    0.3",
            " ABCD 26:001:00000   -0.4    0.3 2451.6    1.2").replace(
            " ABCD 26:001:00300 2452.0    1.1   -0.3    0.3    0.2    0.3",
            " ABCD 26:001:00300   -0.3    0.3 2452.0    1.1").replace(
            " ABCD 26:001:00600 2453.4    1.0   -0.2    0.3    0.2    0.3",
            " ABCD 26:001:00600   -0.2    0.3 2453.4    1.0")
        pts = tro.parse_tro_lines(text.splitlines())
        self.assertAlmostEqual(pts[0].ztd_m, 2.4516, places=6)
        self.assertAlmostEqual(pts[0].sigma_ztd_m, 0.0012, places=6)

    def test_ignores_lines_outside_solution_block(self):
        # a stray data-looking line in TROP/DESCRIPTION must not parse
        pts = tro.parse_tro_lines(_TRO.splitlines())
        self.assertTrue(all(p.site == "ABCD" for p in pts))


if __name__ == "__main__":
    unittest.main()

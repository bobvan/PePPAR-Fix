"""Tests for the Saastamoinen + METAR ZTD-seed helpers (I-024942)."""
from __future__ import annotations

import math
import os
import sys
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path
from tempfile import TemporaryDirectory

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.saastamoinen import (
    ENGINE_ZTD_APRIORI_M,
    metar_to_init_ztd_residual,
    metar_to_ztd,
    saastamoinen_zhd,
    saastamoinen_zwd,
    station_pressure_from_altimeter,
    vapor_pressure_from_dewpoint,
)
from peppar_fix.metar import (
    MetarReadError,
    metar_age_seconds,
    read_latest_metar,
)


class VaporPressureTest(unittest.TestCase):
    def test_zero_dewpoint(self):
        # Magnus at 0C: 6.1078 × exp(0) = 6.1078 hPa.
        self.assertAlmostEqual(vapor_pressure_from_dewpoint(0.0), 6.1078, places=4)

    def test_typical_warm_humid(self):
        # 20C dewpoint → ~23.4 hPa, the textbook number.
        self.assertAlmostEqual(vapor_pressure_from_dewpoint(20.0), 23.4, delta=0.1)

    def test_subzero(self):
        # −10C → ~2.86 hPa.
        self.assertAlmostEqual(vapor_pressure_from_dewpoint(-10.0), 2.87, delta=0.05)


class StationPressureTest(unittest.TestCase):
    def test_sea_level_no_change(self):
        # At elev=0 the conversion is a no-op.
        self.assertAlmostEqual(
            station_pressure_from_altimeter(1013.25, 0.0), 1013.25, places=2)

    def test_ufo1_elev(self):
        # Lab: elev=201m, altim=1005.8hPa → station ~982 hPa.
        p = station_pressure_from_altimeter(1005.8, 201.0)
        self.assertAlmostEqual(p, 982.1, delta=0.5)

    def test_higher_elevation_lower_pressure(self):
        # Pressure should fall monotonically with elevation.
        p_lo = station_pressure_from_altimeter(1013.25, 100.0)
        p_hi = station_pressure_from_altimeter(1013.25, 500.0)
        self.assertGreater(p_lo, p_hi)


class SaastamoinenZhdTest(unittest.TestCase):
    def test_standard_atmosphere_at_sea_level_lab_lat(self):
        # P=1013.25, lat=41.84, h=0 → ZHD ≈ 2.31 m.
        zhd = saastamoinen_zhd(1013.25, 41.84, 0.0)
        self.assertAlmostEqual(zhd, 2.305, delta=0.005)

    def test_low_pressure_yields_lower_zhd(self):
        # Low-pressure system at 980 hPa → ZHD < 2.3 m.
        zhd = saastamoinen_zhd(980.0, 41.84, 0.2)
        self.assertLess(zhd, 2.3)
        # Sanity bound — ZHD should be linear in pressure.
        self.assertAlmostEqual(zhd, 2.231, delta=0.005)

    def test_lat_dependence_is_small(self):
        # Latitude correction is sub-cm at temperate latitudes.
        z_eq = saastamoinen_zhd(1013.25, 0.0, 0.0)
        z_ufo1 = saastamoinen_zhd(1013.25, 41.84, 0.0)
        z_polar = saastamoinen_zhd(1013.25, 80.0, 0.0)
        self.assertLess(abs(z_ufo1 - z_eq), 0.01)
        self.assertLess(abs(z_polar - z_eq), 0.015)


class SaastamoinenZwdTest(unittest.TestCase):
    def test_dry_air_zwd_zero(self):
        # No water vapor → zero wet delay.
        self.assertEqual(saastamoinen_zwd(288.15, 0.0), 0.0)

    def test_temperate_humid(self):
        # T=288.15 K (15C), e=10 hPa → ~99 mm.
        zwd = saastamoinen_zwd(288.15, 10.0)
        self.assertAlmostEqual(zwd, 0.099, delta=0.005)

    def test_warmer_air_smaller_zwd_at_same_e(self):
        # At constant e, warmer air → SMALLER ZWD per Saastamoinen
        # (1255/T term shrinks as T grows).
        zwd_cool = saastamoinen_zwd(283.15, 10.0)  # 10 C
        zwd_warm = saastamoinen_zwd(303.15, 10.0)  # 30 C
        self.assertLess(zwd_warm, zwd_cool)


class MetarToZtdTest(unittest.TestCase):
    def test_lab_today_typical_values(self):
        # Sample matches dayplan coordination note: KDPA at 12.8C/7.2C/
        # 1003.8hPa, UFO1 at 41.84°/201m → ZTD ≈ 2.33 m, residual ~+34 mm.
        zhd, zwd, ztd = metar_to_ztd(
            temp_C=12.8, dewp_C=7.2, altim_hPa=1003.8,
            lat_deg=41.84, elev_m=201.0,
        )
        self.assertAlmostEqual(zhd, 2.231, delta=0.010)
        self.assertAlmostEqual(zwd, 0.103, delta=0.010)
        self.assertAlmostEqual(ztd, 2.334, delta=0.020)

    def test_residual_signed(self):
        # Residual = ZTD − 2.3 m apriori; can go either sign.
        # Hot+humid → positive (more wet delay than apriori covers).
        residual_hot, _ = metar_to_init_ztd_residual(
            temp_C=30.0, dewp_C=25.0, altim_hPa=1013.0,
            lat_deg=41.84, elev_m=201.0,
        )
        # Cold+dry+low-pressure → can go negative.
        residual_cold, _ = metar_to_init_ztd_residual(
            temp_C=-20.0, dewp_C=-25.0, altim_hPa=980.0,
            lat_deg=41.84, elev_m=201.0,
        )
        self.assertGreater(residual_hot, 0.0)
        self.assertLess(residual_cold, 0.0)
        # Both should be physically plausible (within ±1 m of apriori).
        self.assertLess(abs(residual_hot), 1.0)
        self.assertLess(abs(residual_cold), 1.0)

    def test_diag_dict_complete(self):
        residual, diag = metar_to_init_ztd_residual(
            temp_C=15.0, dewp_C=10.0, altim_hPa=1013.0,
            lat_deg=41.84, elev_m=201.0,
        )
        for key in ('temp_C', 'dewp_C', 'altim_hPa', 'lat_deg', 'elev_m',
                    'P_station_hPa', 'e_hPa', 'zhd_m', 'zwd_m', 'ztd_m',
                    'apriori_m', 'residual_m'):
            self.assertIn(key, diag)
        self.assertAlmostEqual(diag['apriori_m'], ENGINE_ZTD_APRIORI_M)
        self.assertAlmostEqual(diag['residual_m'], residual, places=6)


class MetarReaderTest(unittest.TestCase):
    """Test the cron-CSV reader against synthetic data."""

    HEADER = "fetch_ts,report_ts,temp_C,dewp_C,altim_hPa,slp_hPa,raw_metar"

    def _write_csv(self, tmpdir, body):
        p = Path(tmpdir) / "kdpa.csv"
        p.write_text(self.HEADER + "\n" + body)
        return p

    def test_reads_last_row(self):
        with TemporaryDirectory() as td:
            now_iso = "2026-05-04T13:00:00Z"
            body = (
                f"2026-05-04T12:55:01Z,{now_iso},12.8,7.2,1003.8,1003.6,METAR1\n"
                f"2026-05-04T13:00:01Z,{now_iso},12.5,7.0,1003.7,1003.5,METAR2\n"
            )
            p = self._write_csv(td, body)
            rec = read_latest_metar(p)
            self.assertAlmostEqual(rec['temp_C'], 12.5)
            self.assertAlmostEqual(rec['dewp_C'], 7.0)
            self.assertAlmostEqual(rec['altim_hPa'], 1003.7)
            self.assertEqual(rec['raw_metar'], "METAR2")
            self.assertEqual(rec['report_ts'].tzinfo, timezone.utc)

    def test_missing_file_raises(self):
        with self.assertRaises(MetarReadError):
            read_latest_metar("/nonexistent/path/kdpa.csv")

    def test_empty_file_raises(self):
        with TemporaryDirectory() as td:
            p = Path(td) / "empty.csv"
            p.write_text(self.HEADER + "\n")
            with self.assertRaises(MetarReadError):
                read_latest_metar(p)

    def test_malformed_row_raises(self):
        with TemporaryDirectory() as td:
            body = "not-a-date,not-a-date,oops,7.2,1003.8,1003.6,METAR\n"
            p = self._write_csv(td, body)
            with self.assertRaises(MetarReadError):
                read_latest_metar(p)

    def test_age_uses_report_not_fetch(self):
        rec = {
            'fetch_ts': datetime(2026, 5, 4, 13, 0, tzinfo=timezone.utc),
            'report_ts': datetime(2026, 5, 4, 12, 0, tzinfo=timezone.utc),
        }
        # Fixed "now" 1 minute past fetch; age from REPORT is ~1h, not 1 min.
        now = datetime(2026, 5, 4, 13, 1, tzinfo=timezone.utc)
        self.assertAlmostEqual(metar_age_seconds(rec, now), 3660.0)


if __name__ == "__main__":
    unittest.main()

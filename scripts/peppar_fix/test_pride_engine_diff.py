"""Tests for pride_engine_diff: PRIDE kin parsing, engine log
parsing, time-matching, and Δ3D computation."""
from __future__ import annotations

import os
import sys
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path
from tempfile import TemporaryDirectory

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.pride_engine_diff import (
    DiffRecord, EngineEvent, EngineSample, KinEpoch,
    ecef_dist, first_divergence, lla_to_ecef, match_engine_to_pride,
    parse_engine_log, parse_kin, write_csv,
)


# ─── PRIDE kin parser ──────────────────────────────────────────── #


def _write_kin(path: Path, rows: list[tuple]) -> None:
    """Write a minimal kin_* file with the given (mjd, sod, x, y, z,
    lat, lon, h, nsat_str, pdop) rows."""
    with open(path, "w") as f:
        f.write("ufo1                                                        STATION\n")
        f.write("Kinematic   10.000000 10.000000 10.000000                   POS MODE/PRIORI (meter)\n")
        f.write(" " * 60 + "END OF HEADER\n")
        f.write("* Mjd       Sod               X             Y             Z         Latitude        Longitude        Height            Nsat/GREC2C3J   PDOP\n")
        for r in rows:
            mjd, sod, x, y, z, lat, lon, h, nsat, pdop = r
            f.write(f"{mjd:5d}    {sod:6.2f}   {x:13.4f}  {y:13.4f}  {z:13.4f}   "
                    f"{lat:14.11f}  {lon:14.11f}      {h:8.4f}     {nsat:>20s}   {pdop:.2f}\n")


class ParseKinTest(unittest.TestCase):

    def test_parses_three_rows(self):
        # Test uses placeholder coordinates (40°N, 90°W) so the
        # repo's pre-commit secrets guard doesn't flag the literal
        # lab-host position.
        with TemporaryDirectory() as td:
            p = Path(td) / "kin"
            _write_kin(p, [
                (61180, 0.00, 0.111111, 0.222222, 0.333333,
                 40.001, -90.001, 200.1, "16 08 03 05", 1.48),
                (61180, 30.00, 0.111222, 0.222111, 0.333444,
                 40.002, -90.002, 200.2, "16 08 03 05", 1.42),
                (61180, 60.00, 0.111333, 0.222000, 0.333555,
                 40.003, -90.003, 200.3, "17 09 03 05", 1.38),
            ])
            epochs = parse_kin(p)
        self.assertEqual(len(epochs), 3)
        self.assertEqual(epochs[0].mjd, 61180)
        self.assertEqual(epochs[0].sod, 0.0)
        self.assertAlmostEqual(epochs[0].x, 0.1111, places=4)
        self.assertAlmostEqual(epochs[0].lat, 40.001, places=3)
        self.assertEqual(epochs[2].sod, 60.0)

    def test_gps_unix_conversion(self):
        """PRIDE kin file's MJD+SOD is in GPS time scale.  UTC is
        GPS−18s as of 2026 (leap second count).  The conversion
        properties should report GPS unix at the GPS-midnight instant
        and UTC unix 18 s earlier."""
        with TemporaryDirectory() as td:
            p = Path(td) / "kin"
            _write_kin(p, [(61180, 0.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, "1", 1.0)])
            (epoch,) = parse_kin(p)
        # MJD 61180 SOD 0 corresponds to GPS-midnight 2026-05-20.
        # GPS unix is UTC unix of the same wallclock instant (no
        # leap-correction added).  UTC unix at that wallclock instant
        # IS the UTC unix of 2026-05-20 00:00:00 — but the same GPS
        # instant happened 18 s EARLIER on the UTC wall clock.
        gps_midnight_2026_05_20 = datetime(
            2026, 5, 20, 0, 0, 0, tzinfo=timezone.utc).timestamp()
        self.assertAlmostEqual(epoch.gps_unix, gps_midnight_2026_05_20,
                               places=3)
        self.assertAlmostEqual(epoch.utc_unix,
                               gps_midnight_2026_05_20 - 18, places=3)

    def test_empty_file_returns_empty_list(self):
        with TemporaryDirectory() as td:
            p = Path(td) / "kin"
            p.write_text(" " * 60 + "END OF HEADER\n")
            self.assertEqual(parse_kin(p), [])


# ─── Engine log parser ──────────────────────────────────────────── #


_SAMPLE_ENGINE_LOG = """\
2026-05-20 07:29:53,086 INFO peppar-fix-engine git=a5a529175ede branch=main
2026-05-20 07:29:54,053 INFO [STATE] AntPosEst: → surveying (initial)
2026-05-20 07:30:33,072 INFO   [AntPosEst 10] positionσ=0.032m pos=(40.00100000, -90.00100000, 200.100) n=12 amb=13 WL: 0/14 fixed NL: 0 fixed nav2Δ=0.9m ZTD=+1283±175mm strength=0 tide=141mm(U-140) worstσ=1000.0m
2026-05-20 07:30:43,080 INFO   [AntPosEst 20] positionσ=0.033m pos=(40.00100012, -90.00100006, 200.099) n=12 amb=13 WL: 0/14 fixed NL: 0 fixed nav2Δ=0.9m ZTD=+139±30mm
2026-05-20 07:30:50,123 INFO   FALSE_FIX detected on G15 — wrong WL integer
2026-05-20 07:30:53,064 INFO   [AntPosEst 30] positionσ=0.034m pos=(40.00100006, -90.00100018, 200.108) n=12 amb=13 WL: 1/14 fixed NL: 0 fixed
2026-05-20 07:31:03,065 INFO   IF_STEP epoch=141 dt=2.5e-9
2026-05-20 07:31:16,065 INFO   [AntPosEst 50] positionσ=0.035m pos=(40.00100024, -90.00100013, 200.088) n=11 amb=12 WL: 0/14 fixed NL: 0 fixed
"""


class ParseEngineLogTest(unittest.TestCase):

    def test_extracts_4_samples_and_2_events(self):
        with TemporaryDirectory() as td:
            p = Path(td) / "engine.log"
            p.write_text(_SAMPLE_ENGINE_LOG)
            samples, events = parse_engine_log(p, tz_offset_hours=-5)
        self.assertEqual(len(samples), 4)
        self.assertEqual(samples[0].epoch_n, 10)
        self.assertEqual(samples[0].sigma_3d, 0.032)
        self.assertAlmostEqual(samples[0].lat, 40.001, places=3)
        self.assertEqual(samples[2].wl_fixed, 1)  # AntPosEst 30
        self.assertEqual(samples[2].wl_total, 14)
        # Events
        self.assertEqual(len(events), 2)
        labels = [e.label for e in events]
        self.assertIn("FALSE_FIX", labels)
        self.assertIn("IF_STEP", labels)

    def test_tz_offset_applied_correctly(self):
        """Engine logs in local time without TZ info; the parser
        applies the configured offset so wall_dt.timestamp() lands at
        the right UNIX epoch."""
        with TemporaryDirectory() as td:
            p = Path(td) / "engine.log"
            p.write_text(_SAMPLE_ENGINE_LOG)
            samples, _ = parse_engine_log(p, tz_offset_hours=-5)
        # First sample: 2026-05-20 07:30:33 CDT = 12:30:33 UTC
        expected = datetime(2026, 5, 20, 12, 30, 33,
                            tzinfo=timezone.utc).timestamp()
        self.assertAlmostEqual(samples[0].wall_dt.timestamp(),
                               expected, places=0)
        # Confirm extracted lat (placeholder coord, not the lab's).
        self.assertAlmostEqual(samples[0].lat, 40.001, places=3)


# ─── Geodesy ────────────────────────────────────────────────────── #


class GeodesyTest(unittest.TestCase):

    def test_lla_to_ecef_round_trip(self):
        """lla_to_ecef inverse: spot-check WGS-84 against a
        published example at lat=0, lon=0, h=0 (ECEF should equal
        the semi-major axis 6378137.0 m on the X axis) and against
        the north pole (Z ≈ b = 6356752.314 m)."""
        x, y, z = lla_to_ecef(0.0, 0.0, 0.0)
        self.assertAlmostEqual(x, 6378137.0, places=3)
        self.assertAlmostEqual(y, 0.0, places=3)
        self.assertAlmostEqual(z, 0.0, places=3)
        # North pole: Z should equal the WGS-84 semi-minor axis
        x, y, z = lla_to_ecef(90.0, 0.0, 0.0)
        self.assertAlmostEqual(x, 0.0, places=3)
        self.assertAlmostEqual(y, 0.0, places=3)
        self.assertAlmostEqual(z, 6356752.3142, places=3)

    def test_ecef_dist(self):
        a = (0.0, 0.0, 0.0)
        b = (3.0, 4.0, 0.0)
        self.assertEqual(ecef_dist(a, b), 5.0)
        c = (1.0, 2.0, 3.0)
        d = (1.001, 2.002, 3.003)
        # ~3.74 mm
        self.assertAlmostEqual(ecef_dist(c, d), 0.00374, places=4)


# ─── Reconciliation ─────────────────────────────────────────────── #


def _make_kin(unix_t: float, ecef: tuple[float, float, float]) -> KinEpoch:
    """Synthetic KinEpoch at a given UTC unix time + ECEF."""
    # Choose mjd + sod so utc_unix == unix_t.
    mjd_unix = 1858_11_17  # not relevant — utc_unix is recomputed
    from peppar_fix.pride_engine_diff import _MJD_UNIX_OFFSET
    # gps_unix = utc_unix + 18; pick mjd at recent value
    gps_unix = unix_t + 18.0
    mjd = int(gps_unix // 86400) + _MJD_UNIX_OFFSET
    sod = gps_unix - (mjd - _MJD_UNIX_OFFSET) * 86400.0
    return KinEpoch(mjd=mjd, sod=sod, x=ecef[0], y=ecef[1], z=ecef[2],
                    lat=0.0, lon=0.0, height=0.0, pdop=1.0)


def _make_sample(unix_t: float, epoch_n: int,
                 lla: tuple[float, float, float],
                 wl_fixed: int = 0, nl_fixed: int = 0) -> EngineSample:
    dt = datetime.fromtimestamp(unix_t, tz=timezone.utc)
    return EngineSample(
        wall_dt=dt, epoch_n=epoch_n, sigma_3d=0.05,
        lat=lla[0], lon=lla[1], height=lla[2],
        n_used=12, n_amb=13, wl_fixed=wl_fixed, wl_total=14,
        nl_fixed=nl_fixed,
    )


class MatchEngineToPrideTest(unittest.TestCase):

    def test_nearest_within_tolerance_is_matched(self):
        """Engine sample at t=100s, PRIDE epoch at t=95s — within
        the 30s default tolerance; matched and Δ3D computed.
        Uses placeholder coords; the lla→ecef round-trip should
        agree with the synthetic PRIDE ECEF to within a meter."""
        # Pick a placeholder LLA and compute the matching ECEF for
        # the synthetic PRIDE epoch.
        lla = (40.0, -90.0, 200.0)
        ecef = lla_to_ecef(*lla)
        kin = [_make_kin(95.0, ecef)]
        samples = [_make_sample(100.0, 1, lla)]
        records = match_engine_to_pride(samples, kin, [], match_tolerance_s=30)
        self.assertEqual(len(records), 1)
        # Same coords on both sides → Δ3D ≈ 0
        self.assertLess(records[0].delta_m, 0.001)

    def test_no_match_outside_tolerance(self):
        kin = [_make_kin(0.0, (1.0, 2.0, 3.0))]
        # Sample 60s later, tolerance 30s → no match
        samples = [_make_sample(60.0, 1, (0.0, 0.0, 0.0))]
        records = match_engine_to_pride(samples, kin, [], match_tolerance_s=30)
        self.assertEqual(records, [])

    def test_events_in_window_collected(self):
        kin = [_make_kin(1000.0, (1.0, 2.0, 3.0))]
        samples = [_make_sample(1000.0, 1, (40.0, -90.0, 200.0))]
        events = [
            EngineEvent(wall_dt=datetime.fromtimestamp(990.0, tz=timezone.utc),
                        label="FALSE_FIX", raw_line="..."),
            EngineEvent(wall_dt=datetime.fromtimestamp(1010.0, tz=timezone.utc),
                        label="IF_STEP", raw_line="..."),
            EngineEvent(wall_dt=datetime.fromtimestamp(2000.0, tz=timezone.utc),
                        label="GF_STEP", raw_line="..."),
        ]
        records = match_engine_to_pride(samples, kin, events,
                                        window_s=60.0, match_tolerance_s=30)
        self.assertEqual(len(records), 1)
        self.assertCountEqual(records[0].events_in_window,
                              ["FALSE_FIX", "IF_STEP"])

    def test_empty_kin_returns_empty(self):
        self.assertEqual(match_engine_to_pride([], [], []), [])


class FirstDivergenceTest(unittest.TestCase):

    def _record(self, delta_m: float, epoch_n: int = 0) -> DiffRecord:
        return DiffRecord(
            utc_unix=0.0, engine_iso="t0", epoch_n=epoch_n,
            engine_ecef=(0, 0, 0), pride_ecef=(0, 0, 0),
            pride_mjd=0, pride_sod=0.0,
            delta_m=delta_m, sigma_3d=0.05,
            nl_fixed=0, wl_fixed=0, n_used=10,
        )

    def test_returns_first_record_above_threshold(self):
        recs = [self._record(0.05, 1), self._record(0.08, 2),
                self._record(0.15, 3), self._record(0.50, 4)]
        first = first_divergence(recs, threshold_m=0.10)
        self.assertEqual(first.epoch_n, 3)

    def test_none_when_all_below(self):
        recs = [self._record(0.01), self._record(0.02), self._record(0.05)]
        self.assertIsNone(first_divergence(recs, threshold_m=0.10))


class WriteCsvTest(unittest.TestCase):

    def test_roundtrip_header_plus_one_row(self):
        rec = DiffRecord(
            utc_unix=1234567890.123, engine_iso="2026-05-20T12:00:00-05:00",
            epoch_n=42,
            engine_ecef=(100000.000, 200000.000, 300000.000),
            pride_ecef=(100000.001, 200000.002, 300000.003),
            pride_mjd=61180, pride_sod=30.0,
            delta_m=0.0017, sigma_3d=0.05,
            nl_fixed=3, wl_fixed=7, n_used=12,
            events_in_window=["FALSE_FIX", "IF_STEP"],
        )
        with TemporaryDirectory() as td:
            out = Path(td) / "diff.csv"
            write_csv([rec], out)
            text = out.read_text()
        # Header
        self.assertIn("utc_unix,engine_iso,epoch_n", text.splitlines()[0])
        # Row
        self.assertIn("1234567890.123", text)
        self.assertIn("FALSE_FIX;IF_STEP", text)


if __name__ == "__main__":
    unittest.main()

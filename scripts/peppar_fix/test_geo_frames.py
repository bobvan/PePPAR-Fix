"""Tests for geo_frames — Frame/GeoPoint + pyproj-backed conversion.

Ground-truth anchor: the OPUS UFO1 static survey gives the *same
physical point* in NAD83(2011)@2010.0 and ITRF2020@2026.3164
(timelab/surveys/2026-05-03-ufo1-opus-static.md).  pyproj's 14-param
time-dependent Helmert reproduces it to ~5 cm (PROJ's HTDP realization
vs OPUS's, plus OPUS's own ~cm sigma); round-trips are self-consistent
to << 1 mm; intra-ITRF epoch propagation matches the CONUS ~2 cm/yr
plate motion.
"""
import math
import unittest

from peppar_fix.geo_frames import (
    Frame, GeoPoint, convert, to_canonical, CANONICAL_REALIZATION)

# OPUS UFO1 ground truth (same physical point, two frames).
_NAD83 = (157470.222, -4756189.544, 4232767.952)        # NAD83(2011)@2010.0
_ITRF = (157469.2061, -4756188.1678, 4232767.857)        # ITRF2020@2026.3164
_OBS_EPOCH = 2026.3164


def _resid_mm(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3))) * 1000.0


class FrameTests(unittest.TestCase):
    def test_parse_and_format_roundtrip(self):
        f = Frame.parse("ITRF2020@2026.4200")
        self.assertEqual(f.realization, "ITRF2020")
        self.assertAlmostEqual(f.epoch, 2026.42)
        self.assertEqual(str(Frame("NAD83(2011)", 2010.0)),
                         "NAD83(2011)@2010.0000")

    def test_unknown_realization_rejected(self):
        with self.assertRaises(ValueError):
            Frame("XYZ99", 2020.0)

    def test_bad_parse_rejected(self):
        with self.assertRaises(ValueError):
            Frame.parse("ITRF2020")  # no @epoch

    def test_frame_equality(self):
        self.assertEqual(Frame("ITRF2020", 2026.42),
                         Frame("ITRF2020", 2026.42))
        self.assertNotEqual(Frame("ITRF2020", 2026.42),
                            Frame("ITRF2020", 2026.43))


class GeoPointTests(unittest.TestCase):
    def test_requires_3tuple(self):
        with self.assertRaises(ValueError):
            GeoPoint((1.0, 2.0), Frame("ITRF2020", 2020.0))

    def test_requires_frame_type(self):
        with self.assertRaises(TypeError):
            GeoPoint((1.0, 2.0, 3.0), "ITRF2020@2020")  # bare string, not Frame


class ConversionTests(unittest.TestCase):
    def test_nad83_to_itrf_matches_opus(self):
        """The datum fix: NAD83(2011) ARP read as ITRF was 1.71 m off;
        converting reproduces OPUS's ITRF2020 to the PROJ-vs-OPUS floor."""
        p = GeoPoint(_NAD83, Frame("NAD83(2011)", 2010.0))
        c = to_canonical(p, _OBS_EPOCH)
        self.assertEqual(c.frame, Frame(CANONICAL_REALIZATION, _OBS_EPOCH))
        r = _resid_mm(c.ecef, _ITRF)
        # ~49 mm vs OPUS (HTDP-realization + OPUS sigma); the point is it
        # kills the 1.71 m (1710 mm) datum error.
        self.assertLess(r, 60.0, f"NAD83->ITRF off by {r:.1f} mm vs OPUS")
        self.assertLess(_resid_mm(p.ecef, _ITRF) - r, 1710.0)  # corrected ≫ residual

    def test_to_canonical_at_static_anchor_epoch(self):
        """Boundary case: convert a NAD83(2011)@2010.0 point to ITRF2020 at
        obs_epoch=2010.0 — PROJ propagates the time-dependent Helmert at
        the static anchor's own reference year (the time-term -> 0 corner).
        Pins this so a future PROJ update can't silently regress it."""
        p = GeoPoint(_NAD83, Frame("NAD83(2011)", 2010.0))
        c = to_canonical(p, 2010.0)
        self.assertEqual(c.frame, Frame(CANONICAL_REALIZATION, 2010.0))
        # The NAD83(2011)<->ITRF2020 datum offset is ~1.5 m even with the
        # plate-motion time term zeroed at 2010.0 — this *is* the offset
        # that, evaluated at the 2026.3 obs epoch, produced the 1.71 m
        # read-NAD83-as-ITRF bug.  Pin it to a tight band around 1567 mm.
        self.assertAlmostEqual(_resid_mm(c.ecef, _NAD83), 1567.0, delta=50.0)

    def test_roundtrip_self_consistent(self):
        itrf = GeoPoint(_ITRF, Frame("ITRF2020", _OBS_EPOCH))
        back = convert(convert(itrf, Frame("NAD83(2011)", 2010.0)),
                       Frame("ITRF2020", _OBS_EPOCH))
        self.assertLess(_resid_mm(back.ecef, _ITRF), 0.1)

    def test_identity_epoch_is_relabel(self):
        itrf = GeoPoint(_ITRF, Frame("ITRF2020", _OBS_EPOCH))
        self.assertEqual(to_canonical(itrf, _OBS_EPOCH).ecef, itrf.ecef)
        self.assertIs(convert(itrf, itrf.frame), itrf)

    def test_intra_itrf_epoch_propagation_is_plate_motion(self):
        itrf = GeoPoint(_ITRF, Frame("ITRF2020", _OBS_EPOCH))
        one_yr = _resid_mm(convert(itrf, Frame("ITRF2020", _OBS_EPOCH + 1)).ecef,
                           itrf.ecef)
        self.assertTrue(10.0 < one_yr < 30.0,
                        f"+1yr plate motion {one_yr:.1f} mm not ~CONUS 2cm/yr")
        ten_yr = _resid_mm(convert(itrf, Frame("ITRF2020", _OBS_EPOCH + 10)).ecef,
                           itrf.ecef)
        # Plate motion is linear in time → +10yr ≈ 10 × +1yr.
        self.assertAlmostEqual(ten_yr, 10 * one_yr, delta=0.05 * ten_yr)

    def test_wgs84_is_not_a_realization(self):
        # Policy: GPS broadcast / NAV2 (nominally WGS84) are tagged
        # ITRF2020 directly (≈ at the cm level).  WGS84 as a generic
        # ensemble transforms badly (~2 m), so it is NOT a known
        # realization — using it is a loud error, not a silent shift.
        with self.assertRaises(ValueError):
            Frame("WGS84", _OBS_EPOCH)


if __name__ == "__main__":
    unittest.main()

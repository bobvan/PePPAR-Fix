"""The [PPP_STATE] format↔parse contract: format_ppp_state_line (the engine
emitter, and pos_replay stage-2b re-emitter) must round-trip through
pos_replay_compare's parser, so producer and consumer can't drift."""
import os
import sys
import unittest
from datetime import datetime, timezone

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix.ppp_state_line import format_ppp_state_line  # noqa: E402
from peppar_fix import pos_replay_compare as prc            # noqa: E402


class TestFormatParseRoundTrip(unittest.TestCase):
    def test_round_trips_through_parser(self):
        gps = datetime(2026, 6, 25, 12, 0, 1, tzinfo=timezone.utc)
        line = format_ppp_state_line(
            gps, n_epochs=42, n_used=9,
            ecef=(-2730000.1234, -4440000.5678, 3975000.9012),
            sigma_pos_m=0.1500, ztd_m=-0.0123, sigma_ztd_m=0.0300)
        # the engine logs it with a prefix; the parser searches anywhere
        rows = prc.parse_ppp_state(["2026-06-25 12:00:01 INFO " + line])
        self.assertEqual(len(rows), 1)
        r = rows[0]
        self.assertEqual(r.epoch, 42)
        self.assertEqual(r.n_used, 9)
        self.assertAlmostEqual(r.ecef_m[0], -2730000.1234, places=4)
        self.assertAlmostEqual(r.ecef_m[1], -4440000.5678, places=4)
        self.assertAlmostEqual(r.ecef_m[2], 3975000.9012, places=4)
        self.assertAlmostEqual(r.sigma_pos_m, 0.15, places=4)
        self.assertAlmostEqual(r.ztd_m, -0.0123, places=4)
        self.assertAlmostEqual(r.sigma_ztd_m, 0.03, places=4)
        self.assertIsNotNone(r.gps)
        self.assertEqual(r.gps.year, 2026)

    def test_exact_string_anchor(self):
        # Charlie #238: producer (printf) and consumer (regex) are separate
        # representations, so "can't drift" rests on THIS test staying
        # comprehensive.  An exact full-string anchor trips on any format change
        # the field-wise round-trip happens to tolerate (e.g. a reordered field,
        # a changed precision, a renamed key).
        gps = datetime(2026, 6, 25, 12, 0, 1, tzinfo=timezone.utc)
        line = format_ppp_state_line(
            gps, n_epochs=42, n_used=9,
            ecef=(-2730000.1234, -4440000.5678, 3975000.9012),
            sigma_pos_m=0.15, ztd_m=-0.0123, sigma_ztd_m=0.03)
        self.assertEqual(
            line,
            "[PPP_STATE] gps=2026-06-25T12:00:01+00:00 epoch=42 n=9 "
            "ecef=-2730000.1234,-4440000.5678,3975000.9012 "
            "sigma_pos=0.1500m ztd=-0.0123m sigma_ztd=0.0300m")

    def test_ztd_sign_and_gps_isoformat(self):
        gps = datetime(2026, 1, 1, tzinfo=timezone.utc)
        line = format_ppp_state_line(gps, 1, 8, (1.0, 2.0, 3.0),
                                     0.2, +0.05, 0.01)
        self.assertIn("ztd=+0.0500m", line)       # signed
        self.assertIn("gps=2026-01-01T00:00:00+00:00", line)  # tz-aware UTC

    def test_accepts_numpy_like_ecef(self):
        # filt.x[:3] is a numpy slice; floats() in the formatter must accept it
        class _Arr(list):
            pass
        gps = datetime(2026, 1, 1, tzinfo=timezone.utc)
        line = format_ppp_state_line(gps, 1, 8, _Arr([1.5, 2.5, 3.5]),
                                     0.2, 0.05, 0.01)
        self.assertIn("ecef=1.5000,2.5000,3.5000", line)


if __name__ == "__main__":
    unittest.main()

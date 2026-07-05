"""RTCM MSM encoder signal-ID correctness (I-042609).

rtcm_encoder.MSM_SIGNAL_ID must use the RTCM 10403.3 DF395 bit positions
(GPS Table 3.5-91, Galileo 3.5-96, BeiDou 3.5-104) — the same numbering pyrtcm
decodes with.  The prior values were non-standard (GPS-L2CL→9=2P, GAL-E5aI→12=
6Z, BDS-B2I→8=6I, …), so our NTRIP caster emitted MSM a standards-compliant
consumer mis-labels.

Two guards: a static table pinned to the RTCM standard (regression), and an
encode → pyrtcm round-trip asserting pyrtcm sees the standard signal IDs.
"""
import os
import sys
import unittest
from datetime import datetime, timezone

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from pyrtcm import RTCMReader  # noqa: E402

import rtcm_encoder as enc  # noqa: E402

_GT = datetime(2026, 7, 5, 12, 0, 0, tzinfo=timezone.utc)

# Standard DF395 signal IDs (RTCM 10403.3), independent of the encoder's map.
_STANDARD = {
    "GPS-L1CA": 2, "GPS-L2W": 10, "GPS-L2CM": 15, "GPS-L2CL": 16,
    "GPS-L5I": 22, "GPS-L5Q": 23,
    "GAL-E1C": 2, "GAL-E1B": 4, "GAL-E5bI": 14, "GAL-E5bQ": 15,
    "GAL-E5aI": 22, "GAL-E5aQ": 23,
    "BDS-B1I": 2, "BDS-B1C": 30, "BDS-B2I": 14, "BDS-B2aI": 22,
}


def _obs(sv, sig_name, pr, cp=1.2e8, cno=45, lock_ms=500):
    return dict(sv=sv, sig_name=sig_name, pr=pr, cp=cp, cno=cno, lock_ms=lock_ms)


def _sig_ids_present(prefix, obs):
    """Encode obs → pyrtcm-parse → the DF395 signal IDs pyrtcm sees."""
    msg = RTCMReader.parse(enc.encode_msm4(prefix, obs, _GT))
    m = int(msg.DF395)
    return [i for i in range(1, 33) if m & (1 << (32 - i))]


class StandardSigIdTest(unittest.TestCase):
    def test_matches_rtcm_10403_3(self):
        for name, sid in enc.MSM_SIGNAL_ID.items():
            self.assertEqual(sid, _STANDARD[name],
                             f"{name}: encoder {sid} != standard {_STANDARD[name]}")

    def test_l2cl_is_16_not_the_old_2p_slot(self):
        # the exact regression: 9 (=2P) was the bug; 16 (=2L) is correct
        self.assertEqual(enc.MSM_SIGNAL_ID["GPS-L2CL"], 16)


class RoundTripTest(unittest.TestCase):
    """Encode → pyrtcm sees the standard signal IDs (not the old wrong ones)."""

    def test_gps_l1_l2cl(self):
        ids = _sig_ids_present("G", [
            _obs("G05", "GPS-L1CA", 22_000_000.0),
            _obs("G05", "GPS-L2CL", 22_000_003.0)])
        self.assertEqual(ids, [2, 16])          # 1C, 2L (was 2, 9=2P)

    def test_gps_l1_l2w_geodetic(self):
        ids = _sig_ids_present("G", [
            _obs("G12", "GPS-L1CA", 21_000_000.0),
            _obs("G12", "GPS-L2W", 21_000_002.0)])
        self.assertEqual(ids, [2, 10])          # 1C, 2W

    def test_gps_l5q(self):
        ids = _sig_ids_present("G", [_obs("G03", "GPS-L5Q", 23_000_000.0)])
        self.assertEqual(ids, [23])             # 5Q (was 15=2S)

    def test_galileo_e1_e5a(self):
        ids = _sig_ids_present("E", [
            _obs("E11", "GAL-E1C", 23_000_000.0),
            _obs("E11", "GAL-E5aI", 23_000_004.0)])
        self.assertEqual(ids, [2, 22])          # 1C, 5I (E5a is the 5-band)

    def test_galileo_e5b_is_7band_not_altboc(self):
        ids = _sig_ids_present("E", [_obs("E11", "GAL-E5bI", 23_000_000.0)])
        self.assertEqual(ids, [14])             # 7I, not 18 (8I = E5-AltBOC)

    def test_beidou_b1i_b2a(self):
        ids = _sig_ids_present("C", [
            _obs("C21", "BDS-B1I", 24_000_000.0),
            _obs("C21", "BDS-B2aI", 24_000_004.0)])
        self.assertEqual(ids, [2, 22])          # 2I, 5D (B2a; was 14=7I)


if __name__ == "__main__":
    unittest.main()

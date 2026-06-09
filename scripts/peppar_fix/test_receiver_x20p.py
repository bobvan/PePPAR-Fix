"""Tests for ZED-X20P (Generation 10) support.

Covers the dev-box-completable parts of dayplan zedX20pSupport:
  - SEC-UNIQID v2 (6-byte ID) parsing alongside the legacy v1 5-byte ID.
  - get_driver() dispatch for the x20p aliases.
  - X20PDriver signal-name dispatch (GPS/GAL incl. the new GAL E6).

Hardware-gated parts (signal-config NAK behavior, BDS carrier re-survey,
auto-detect wiring) are validated separately on PiPuss.
"""

import unittest

from peppar_fix.receiver import (
    X20PDriver,
    get_driver,
    parse_sec_uniqid,
)


def _uniqid_payload(version, id_bytes):
    """Build a UBX-SEC-UNIQID payload: version + 3 reserved + ID."""
    return bytes([version, 0, 0, 0]) + bytes(id_bytes)


class TestParseSecUniqid(unittest.TestCase):

    def test_v1_five_byte_id(self):
        # F9 family: version 0x01, 5-byte ID (e.g. F9P 5d58b2dad4).
        payload = _uniqid_payload(0x01, bytes.fromhex("5d58b2dad4"))
        uid, uid_hex = parse_sec_uniqid(payload)
        self.assertEqual(uid_hex, "5d58b2dad4")
        self.assertEqual(uid, int.from_bytes(bytes.fromhex("5d58b2dad4"),
                                             "little"))

    def test_v2_six_byte_id_pipuss(self):
        # ZED-X20P on PiPuss 2026-06-08: version 0x02, 6-byte ID.
        payload = _uniqid_payload(0x02, bytes.fromhex("98f4d50f3e54"))
        uid, uid_hex = parse_sec_uniqid(payload)
        self.assertEqual(uid_hex, "98f4d50f3e54")
        self.assertEqual(uid, int.from_bytes(bytes.fromhex("98f4d50f3e54"),
                                             "little"))

    def test_id_length_follows_payload_not_version_byte(self):
        # The ID is everything after the 4-byte header — width is derived
        # from the payload, so it doesn't depend on reading the version.
        v1 = parse_sec_uniqid(_uniqid_payload(0x01, b"\x01\x02\x03\x04\x05"))
        v2 = parse_sec_uniqid(_uniqid_payload(0x02, b"\x01\x02\x03\x04\x05\x06"))
        self.assertEqual(len(bytes.fromhex(v1[1])), 5)
        self.assertEqual(len(bytes.fromhex(v2[1])), 6)

    def test_little_endian(self):
        uid, _ = parse_sec_uniqid(_uniqid_payload(0x01, b"\x01\x00\x00\x00\x00"))
        self.assertEqual(uid, 1)

    def test_too_short_returns_none(self):
        # Anything shorter than the 4-byte header + 5-byte v1 ID is junk.
        self.assertEqual(parse_sec_uniqid(b""), (None, None))
        self.assertEqual(parse_sec_uniqid(b"\x02\x00\x00\x00"), (None, None))
        self.assertEqual(parse_sec_uniqid(b"\x02\x00\x00\x00\xaa"),
                         (None, None))


class TestGetDriverDispatch(unittest.TestCase):

    def test_x20p_aliases(self):
        for name in ("x20p", "x20", "zed-x20p", "zed_x20p", "X20P", "ZED-X20P"):
            with self.subTest(name=name):
                self.assertIsInstance(get_driver(name), X20PDriver)

    def test_unknown_still_raises(self):
        with self.assertRaises(ValueError):
            get_driver("x30q")


class TestX20PDriverSignals(unittest.TestCase):

    def setUp(self):
        self.drv = X20PDriver()

    def test_metadata(self):
        self.assertEqual(self.drv.name, "ZED-X20P")
        self.assertEqual(self.drv.protver, "50")
        self.assertFalse(self.drv.supports_timing_mode)

    def test_gps_gal_signal_names(self):
        self.assertEqual(self.drv.signal_name(0, 0), "GPS-L1CA")
        self.assertEqual(self.drv.signal_name(0, 3), "GPS-L2CL")
        self.assertEqual(self.drv.signal_name(0, 4), "GPS-L2CM")
        self.assertEqual(self.drv.signal_name(0, 7), "GPS-L5Q")
        self.assertEqual(self.drv.signal_name(2, 0), "GAL-E1C")
        self.assertEqual(self.drv.signal_name(2, 4), "GAL-E5aQ")

    def test_new_gal_e6_signal(self):
        # GAL sigId=8 is new on the X20 and missing from the F9/F10 maps.
        self.assertEqual(self.drv.signal_name(2, 8), "GAL-E6")

    def test_survey_confirmed_bds_uses_f10_numbering(self):
        # PiPuss 2026-06-08 survey: BDS sigIds 4/5/7 resolve under the F10
        # convention, NOT the legacy F9T B1I/B2I numbering.
        self.assertEqual(self.drv.signal_name(3, 4), "BDS-B3I")
        self.assertEqual(self.drv.signal_name(3, 5), "BDS-B1CP")
        self.assertEqual(self.drv.signal_name(3, 7), "BDS-B2aP")

    def test_sbas_mapped(self):
        # SBAS L1CA is tracked on the X20 (survey) — mapped for honesty
        # even though the engine doesn't process SBAS.
        self.assertEqual(self.drv.signal_name(1, 0), "SBAS-L1CA")

    def test_cpmes_native_no_l1_ref_correction(self):
        # PiPuss 2026-06-08: X20 cpMes verified native for all signals
        # (incl. BDS B2a/B3I) — NOT the F9T B2a L1-reference quirk — so
        # no signals need the L1-ref-cycle correction.
        self.assertEqual(self.drv.bds_l1_ref_cycles, frozenset())

    def test_default_if_pairs_are_gps_gal_only(self):
        # BDS carrier mapping is unverified on X20 — kept out of the
        # default IF pairs until the hardware re-survey.
        systems = {pair[0] for pair in self.drv.if_pairs}
        self.assertEqual(systems, {"GPS", "GAL"})


if __name__ == "__main__":
    unittest.main()

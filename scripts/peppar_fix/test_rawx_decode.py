"""Parity + robustness tests for the vectorized RXM-RAWX decoder.

Asserts `rawx_decode.decode_rawx` is bit-exact vs pyubx2's reference
attribute-parse over a synthetic frame covering diverse field values and
trkStat validity-bit combinations.  This is the gate that guards the
serial_reader fast path (docs/x20-vcocxo-arm-comparison-2026-06-12.md).
"""

import struct
import unittest

import numpy as np

from peppar_fix import rawx_decode


def _ubx_checksum(body: bytes) -> bytes:
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes((ck_a, ck_b))


def _build_rawx(rcvTow, week, leapS, meas, rec_stat=0) -> bytes:
    """Assemble a complete UBX RXM-RAWX frame from a list of measurement
    dicts (gnssId, svId, sigId, freqId, prMes, cpMes, doMes, locktime,
    cno, trkStat).  ``rec_stat`` is the X1 recStat bitfield (bit1=clkReset).
    """
    hdr = struct.pack(
        "<dHbBBBH", rcvTow, week, leapS, len(meas), rec_stat, 1, 0
    )  # rcvTow, week, leapS, numMeas, recStat, version, reserved(2)
    blocks = b""
    for m in meas:
        blocks += struct.pack(
            "<ddfBBBBHBBBBBB",
            m["prMes"], m["cpMes"], m["doMes"],
            m["gnssId"], m["svId"], m["sigId"], m["freqId"],
            m["locktime"], m["cno"],
            m.get("prStdev", 0), m.get("cpStdev", 0), m.get("doStdev", 0),
            m["trkStat"], 0,
        )
    payload = hdr + blocks
    body = bytes((rawx_decode.RXM_RAWX_CLS, rawx_decode.RXM_RAWX_ID)) \
        + struct.pack("<H", len(payload)) + payload
    return b"\xb5\x62" + body + _ubx_checksum(body)


_MEAS = [
    dict(gnssId=0, svId=5, sigId=0, freqId=0, prMes=20000000.123,
         cpMes=105123456.789, doMes=-1234.5, locktime=64000, cno=45,
         trkStat=0b0111),   # prValid cpValid halfCyc, not subHalfCyc
    dict(gnssId=2, svId=11, sigId=1, freqId=0, prMes=23456789.0,
         cpMes=0.0, doMes=12.0, locktime=100, cno=30,
         trkStat=0b0001),   # prValid only
    dict(gnssId=6, svId=3, sigId=5, freqId=0, prMes=39999999.9,
         cpMes=-5.5, doMes=0.0, locktime=512, cno=22,
         trkStat=0b1111),   # all four bits
]


class TestRawxDecodeParity(unittest.TestCase):
    def setUp(self):
        from pyubx2 import UBXReader
        self.UBXReader = UBXReader
        self.frame = _build_rawx(123456.789, 2422, 18, _MEAS)
        self.dec = rawx_decode.decode_rawx(self.frame)
        self.ref = UBXReader.parse(self.frame)

    def test_is_rawx(self):
        self.assertTrue(rawx_decode.is_rawx(self.frame))
        self.assertFalse(rawx_decode.is_rawx(b"\xb5\x62\x01\x07ab"))  # NAV-PVT

    def test_header(self):
        self.assertAlmostEqual(self.dec.rcvTow, self.ref.rcvTow, places=6)
        self.assertEqual(self.dec.week, self.ref.week)
        self.assertEqual(self.dec.leapS, self.ref.leapS)
        self.assertEqual(self.dec.numMeas, self.ref.numMeas)
        self.assertEqual(self.dec.numMeas, len(_MEAS))

    def test_per_measurement_parity(self):
        for i in range(self.dec.numMeas):
            i2 = f"{i + 1:02d}"
            g = lambda name: getattr(self.ref, f"{name}_{i2}")
            self.assertEqual(int(self.dec.gnssId[i]), g("gnssId"), i)
            self.assertEqual(int(self.dec.svId[i]), g("svId"), i)
            self.assertEqual(int(self.dec.sigId[i]), g("sigId"), i)
            self.assertEqual(int(self.dec.cno[i]), g("cno"), i)
            self.assertEqual(int(self.dec.locktime[i]), g("locktime"), i)
            self.assertAlmostEqual(float(self.dec.prMes[i]), g("prMes"), places=6)
            self.assertAlmostEqual(float(self.dec.cpMes[i]), g("cpMes"), places=6)
            self.assertEqual(bool(self.dec.prValid[i]), bool(g("prValid")), i)
            self.assertEqual(bool(self.dec.cpValid[i]), bool(g("cpValid")), i)
            self.assertEqual(bool(self.dec.halfCyc[i]), bool(g("halfCyc")), i)
            self.assertEqual(bool(self.dec.subHalfCyc[i]), bool(g("subHalfCyc")), i)

    def test_trkstat_bits(self):
        # m0=0b0111, m1=0b0001, m2=0b1111
        np.testing.assert_array_equal(self.dec.prValid, [True, True, True])
        np.testing.assert_array_equal(self.dec.cpValid, [True, False, True])
        np.testing.assert_array_equal(self.dec.halfCyc, [True, False, True])
        np.testing.assert_array_equal(self.dec.subHalfCyc, [False, False, True])


class TestRawxDecodeRobustness(unittest.TestCase):
    def test_zero_measurements(self):
        frame = _build_rawx(1.0, 2400, 18, [])
        dec = rawx_decode.decode_rawx(frame)
        self.assertEqual(dec.numMeas, 0)
        self.assertEqual(len(dec.prMes), 0)

    def test_not_rawx_raises(self):
        with self.assertRaises(ValueError):
            rawx_decode.decode_rawx(b"\xb5\x62\x01\x07\x00\x00\x00\x00")

    def test_truncated_raises(self):
        frame = _build_rawx(1.0, 2400, 18, _MEAS)
        with self.assertRaises(ValueError):
            rawx_decode.decode_rawx(frame[:20])  # header claims 3 meas


class TestRawxClkReset(unittest.TestCase):
    """recStat.clkReset decode — regression guard for the #163 plumbing
    break that let the clkPoC3 21 ms wrap exit-5 (gracefulClkReset)."""

    def test_clk_reset_clear_by_default(self):
        dec = rawx_decode.decode_rawx(_build_rawx(1.0, 2400, 18, _MEAS, rec_stat=0))
        self.assertFalse(dec.clk_reset)

    def test_clk_reset_set(self):
        # recStat bit1 = clkReset; bit0 = leapSec (must NOT be read as clkReset)
        self.assertTrue(rawx_decode.decode_rawx(
            _build_rawx(1.0, 2400, 18, _MEAS, rec_stat=0b10)).clk_reset)
        self.assertFalse(rawx_decode.decode_rawx(
            _build_rawx(1.0, 2400, 18, _MEAS, rec_stat=0b01)).clk_reset)  # leapSec only
        self.assertTrue(rawx_decode.decode_rawx(
            _build_rawx(1.0, 2400, 18, _MEAS, rec_stat=0b11)).clk_reset)  # both

    def test_clk_reset_parity_vs_pyubx2(self):
        from pyubx2 import UBXReader
        for rs in (0b00, 0b01, 0b10, 0b11):
            frame = _build_rawx(1.0, 2400, 18, _MEAS, rec_stat=rs)
            dec = rawx_decode.decode_rawx(frame)
            ref = UBXReader.parse(frame)
            self.assertEqual(dec.clk_reset, bool(ref.clkReset), f"rec_stat={rs:#04b}")


if __name__ == "__main__":
    unittest.main()

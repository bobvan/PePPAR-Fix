"""Parity + integration tests for the vectorized NAV-SIG decoder.

`decode_nav_sig` must be bit-exact vs pyubx2's reference parse over a
synthetic frame (incl. the sigFlags bitfield, which pyubx2 expands into
prUsed_NN/health_NN/...), and `Nav2SignalStore.update_decoded` must
populate the store correctly — including the prUsed/health bits that the
legacy `update(parsed)` getattr path read as 0 from a real pyubx2 message.

See docs/x20-vcocxo-arm-comparison-2026-06-12.md.
"""

import struct
import unittest

from peppar_fix import nav_sig_decode


def _ubx_checksum(body: bytes) -> bytes:
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return bytes((a, b))


def _build_nav_sig(iTOW, sigs) -> bytes:
    hdr = struct.pack("<IBBH", iTOW, 0, len(sigs), 0)
    blocks = b""
    for s in sigs:
        blocks += struct.pack(
            "<BBBBhBBBBHI",
            s["gnssId"], s["svId"], s["sigId"], s.get("freqId", 0),
            s["prRes"], s["cno"], s.get("qualityInd", 4),
            s.get("corrSource", 0), s.get("ionoModel", 0),
            s["sigFlags"], 0,
        )
    payload = hdr + blocks
    body = bytes((nav_sig_decode.NAV_SIG_CLS, nav_sig_decode.NAV_SIG_ID)) \
        + struct.pack("<H", len(payload)) + payload
    return b"\xb5\x62" + body + _ubx_checksum(body)


# health=bits0-1, prSmoothed=bit2, prUsed=bit3, crUsed=bit4, doUsed=bit5
_SIGS = [
    dict(gnssId=0, svId=7, sigId=0, prRes=30, cno=44, qualityInd=7,
         sigFlags=0b001001),   # health=1, prUsed=1
    dict(gnssId=2, svId=19, sigId=5, prRes=-12, cno=39, qualityInd=4,
         sigFlags=0b111000),   # prUsed crUsed doUsed, health=0
    dict(gnssId=6, svId=3, sigId=0, prRes=4127, cno=20, qualityInd=1,
         sigFlags=0b000010),   # health=2, nothing used
]


class TestNavSigDecodeParity(unittest.TestCase):
    def setUp(self):
        from pyubx2 import UBXReader
        self.frame = _build_nav_sig(123456000, _SIGS)
        self.dec = nav_sig_decode.decode_nav_sig(self.frame)
        self.ref = UBXReader.parse(self.frame)

    def test_is_nav_sig(self):
        self.assertTrue(nav_sig_decode.is_nav_sig(self.frame))
        self.assertFalse(nav_sig_decode.is_nav_sig(b"\xb5\x62\x02\x15ab"))  # RAWX

    def test_header(self):
        self.assertEqual(self.dec.iTOW, self.ref.iTOW)
        self.assertEqual(self.dec.numSigs, self.ref.numSigs)
        self.assertEqual(self.dec.numSigs, len(_SIGS))

    def test_per_signal_parity(self):
        for i in range(self.dec.numSigs):
            i2 = f"{i + 1:02d}"
            g = lambda n: getattr(self.ref, f"{n}_{i2}")
            self.assertEqual(int(self.dec.gnssId[i]), g("gnssId"), i)
            self.assertEqual(int(self.dec.svId[i]), g("svId"), i)
            self.assertEqual(int(self.dec.sigId[i]), g("sigId"), i)
            self.assertEqual(int(self.dec.cno[i]), g("cno"), i)
            self.assertEqual(int(self.dec.qualityInd[i]), g("qualityInd"), i)
            # pyubx2 applies the 0.1 m scale to prRes; our decode keeps raw.
            self.assertAlmostEqual(int(self.dec.prRes[i]) * 0.1, g("prRes"), places=6)
            # sigFlags bits: pyubx2 expands them; we mask the raw u16.
            sf = int(self.dec.sigFlags[i])
            self.assertEqual(sf & 0x03, g("health"), i)
            self.assertEqual((sf >> 3) & 1, g("prUsed"), i)
            self.assertEqual((sf >> 4) & 1, g("crUsed"), i)
            self.assertEqual((sf >> 5) & 1, g("doUsed"), i)


class TestNavSigStoreIntegration(unittest.TestCase):
    def test_update_decoded_populates_flags(self):
        # The bug this fixes: legacy update(parsed) read sigFlags as 0.
        from realtime_ppp import Nav2SignalStore
        store = Nav2SignalStore()
        store.set_signal_names({(0, 0): "GPS-L1CA", (2, 5): "GAL-E5bQ"})
        epoch = nav_sig_decode.decode_nav_sig(_build_nav_sig(1000, _SIGS))
        store.update_decoded(epoch)
        snap = store.snapshot()
        g7 = snap[("G07", "GPS-L1CA")]
        self.assertTrue(g7.pr_used)          # sigFlags bit3 set
        self.assertEqual(g7.health, 1)
        self.assertEqual(g7.cno, 44)
        self.assertAlmostEqual(g7.pr_res_m, 3.0, places=6)   # raw 30 → 3.0 m
        e19 = snap[("E19", "GAL-E5bQ")]
        self.assertTrue(e19.pr_used and e19.cr_used and e19.do_used)
        self.assertEqual(e19.health, 0)

    def test_transitions_detected_across_epochs(self):
        from realtime_ppp import Nav2SignalStore
        store = Nav2SignalStore()
        store.set_signal_names({(0, 0): "GPS-L1CA"})
        on = [dict(gnssId=0, svId=7, sigId=0, prRes=0, cno=40, sigFlags=0b001000)]
        off = [dict(gnssId=0, svId=7, sigId=0, prRes=0, cno=40, sigFlags=0b000000)]
        store.update_decoded(nav_sig_decode.decode_nav_sig(_build_nav_sig(1, on)))
        tr = store.update_decoded(nav_sig_decode.decode_nav_sig(_build_nav_sig(2, off)))
        self.assertEqual(tr, [("G07", "GPS-L1CA", True, False)])


class TestNavSigDecodeRobustness(unittest.TestCase):
    def test_zero_sigs(self):
        dec = nav_sig_decode.decode_nav_sig(_build_nav_sig(1, []))
        self.assertEqual(dec.numSigs, 0)
        self.assertEqual(len(dec.gnssId), 0)

    def test_not_nav_sig_raises(self):
        with self.assertRaises(ValueError):
            nav_sig_decode.decode_nav_sig(b"\xb5\x62\x02\x15\x00\x00\x00\x00")

    def test_truncated_raises(self):
        frame = _build_nav_sig(1, _SIGS)
        with self.assertRaises(ValueError):
            nav_sig_decode.decode_nav_sig(frame[:18])


if __name__ == "__main__":
    unittest.main()

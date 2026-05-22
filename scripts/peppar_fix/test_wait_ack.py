"""Tests for wait_ack() — payload + checksum validation of UBX-ACK
frames.

Two scan paths are exercised:
  * raw-byte scan path (when the underlying stream exposes ``read_raw``)
  * pyubx2-parsed fallback path (when it doesn't)

Both must reject ACKs that don't match the expected class+id, AND
reject ACKs with bad checksums, AND accept the right ACK first time
when multiple ACKs are queued in the buffer.
"""
from __future__ import annotations

import unittest
from unittest import mock
from collections import deque
from typing import Iterable

from peppar_fix.receiver import (
    _UBX_CLSID_BY_NAME,
    _parse_ubx_ack_frame,
    _ubx_fletcher16,
    _wait_ack_parsed,
    wait_ack,
)


def make_ack_frame(cls_byte: int, id_byte: int, kind: str = "ACK",
                   *, bad_checksum: bool = False) -> bytes:
    """Build a raw UBX-ACK-ACK or UBX-ACK-NAK frame.

    The 2-byte payload (cls_byte, id_byte) identifies which message
    the ACK is for.  Set bad_checksum=True to flip the checksum and
    confirm that wait_ack rejects the frame.
    """
    ubx_id = 0x01 if kind == "ACK" else 0x00
    body = bytes([0x05, ubx_id, 0x02, 0x00, cls_byte, id_byte])
    ck_a, ck_b = _ubx_fletcher16(body)
    if bad_checksum:
        ck_a = (ck_a + 1) & 0xFF
    return b"\xb5\x62" + body + bytes([ck_a, ck_b])


class FakeStream:
    """Stream stand-in for a serial port.  Delivers byte chunks via
    ``read_raw(n)``, one chunk per call until queue empty.  Returns
    empty bytes after exhaustion (the wait_ack loop treats that as
    "no data this round" and keeps polling)."""

    def __init__(self, chunks: Iterable[bytes]):
        self._chunks = deque(chunks)

    def read_raw(self, n: int) -> bytes:
        if not self._chunks:
            return b""
        return self._chunks.popleft()


class FakeUbr:
    """UBXReader stand-in: exposes ._stream attribute that wait_ack
    introspects to find ``read_raw``."""

    def __init__(self, stream):
        self._stream = stream


# ─── _ubx_fletcher16 spot-checks ──────────────────────────────────── #


class FletcherTest(unittest.TestCase):
    def test_known_ack_ack_for_cfg_valset(self):
        # body = 05 01 02 00 06 8A (ACK-ACK ack'ing a CFG-VALSET)
        a, b = _ubx_fletcher16(bytes.fromhex("050102000688"))
        # 0x88 ≠ 0x8A on purpose; just a spot-check of the running sum.
        # 5,1,2,0,6,0x88 → a runs 5,6,8,8,14,150 → b runs 5,11,19,27,41,191.
        self.assertEqual((a, b), (150, 191))

    def test_round_trip_against_parser(self):
        """The strongest correctness check is that the bytes the
        Fletcher helper produces are accepted by the parser."""
        frame = make_ack_frame(0x06, 0x8A, "ACK")
        kind, _, _, _ = _parse_ubx_ack_frame(frame, 0)
        self.assertEqual(kind, "ACK")


# ─── _parse_ubx_ack_frame ─────────────────────────────────────────── #


class ParseAckFrameTest(unittest.TestCase):

    def test_valid_ack_ack_for_cfg_valset(self):
        frame = make_ack_frame(0x06, 0x8A, "ACK")
        kind, p_cls, p_id, end = _parse_ubx_ack_frame(frame, 0)
        self.assertEqual(kind, "ACK")
        self.assertEqual((p_cls, p_id), (0x06, 0x8A))
        self.assertEqual(end, 10)

    def test_valid_ack_nak_for_cfg_valset(self):
        frame = make_ack_frame(0x06, 0x8A, "NAK")
        kind, _, _, _ = _parse_ubx_ack_frame(frame, 0)
        self.assertEqual(kind, "NAK")

    def test_bad_checksum_rejected(self):
        frame = make_ack_frame(0x06, 0x8A, "ACK", bad_checksum=True)
        kind, _, _, end = _parse_ubx_ack_frame(frame, 0)
        self.assertIsNone(kind)
        self.assertEqual(end, 1)  # advance past sync1, try again

    def test_wrong_class_byte_rejected(self):
        # 0xb5 0x62 0x04 ... — class=0x04 isn't ACK
        frame = b"\xb5\x62\x04\x01\x02\x00\x06\x8a\xff\xff"
        kind, _, _, end = _parse_ubx_ack_frame(frame, 0)
        self.assertIsNone(kind)
        self.assertEqual(end, 1)

    def test_wrong_length_rejected(self):
        # length=4 instead of 2 — not an ACK frame
        body = bytes([0x05, 0x01, 0x04, 0x00, 0x06, 0x8A, 0x00, 0x00])
        a, b = _ubx_fletcher16(body)
        frame = b"\xb5\x62" + body + bytes([a, b])
        kind, _, _, end = _parse_ubx_ack_frame(frame, 0)
        self.assertIsNone(kind)
        self.assertEqual(end, 1)

    def test_short_buffer_returns_not_enough(self):
        # Only 5 of the 10 frame bytes present yet.
        kind, _, _, end = _parse_ubx_ack_frame(b"\xb5\x62\x05\x01\x02", 0)
        self.assertIsNone(kind)
        self.assertEqual(end, 0)  # stay put; caller waits for more

    def test_no_sync_at_start_returns_skip(self):
        kind, _, _, end = _parse_ubx_ack_frame(b"\xaa\xbb\xcc\xdd\xee\xff\x00\x11\x22\x33", 0)
        self.assertIsNone(kind)
        self.assertEqual(end, 1)


# ─── wait_ack raw-scan path ───────────────────────────────────────── #


class WaitAckRawScanTest(unittest.TestCase):

    def test_returns_true_on_valid_ack_for_cfg_valset(self):
        ack = make_ack_frame(0x06, 0x8A, "ACK")
        stream = FakeStream([ack])
        ok = wait_ack(FakeUbr(stream), "CFG", "VALSET", timeout=0.2)
        self.assertTrue(ok)

    def test_returns_false_on_valid_nak_for_cfg_valset(self):
        nak = make_ack_frame(0x06, 0x8A, "NAK")
        stream = FakeStream([nak])
        ok = wait_ack(FakeUbr(stream), "CFG", "VALSET", timeout=0.2)
        self.assertFalse(ok)

    def test_ignores_ack_for_wrong_class_id_then_times_out(self):
        # ACK for CFG-RST (0x06, 0x04), not VALSET — must NOT be matched.
        ack_for_rst = make_ack_frame(0x06, 0x04, "ACK")
        stream = FakeStream([ack_for_rst])
        ok = wait_ack(FakeUbr(stream), "CFG", "VALSET", timeout=0.2)
        self.assertFalse(ok)  # timed out — correct: wrong-payload ACK ignored

    def test_ignores_ack_for_wrong_class_id_then_accepts_right_one(self):
        # Stale ACK for CFG-RST followed by the real ACK for CFG-VALSET.
        # First must be ignored; second matched.
        wrong = make_ack_frame(0x06, 0x04, "ACK")
        right = make_ack_frame(0x06, 0x8A, "ACK")
        stream = FakeStream([wrong + right])
        ok = wait_ack(FakeUbr(stream), "CFG", "VALSET", timeout=0.2)
        self.assertTrue(ok)

    def test_bad_checksum_ignored(self):
        bad = make_ack_frame(0x06, 0x8A, "ACK", bad_checksum=True)
        stream = FakeStream([bad])
        ok = wait_ack(FakeUbr(stream), "CFG", "VALSET", timeout=0.2)
        self.assertFalse(ok)  # bad checksum → not matched → timeout

    def test_random_pattern_in_observation_payload_does_not_false_match(self):
        # An observation frame whose payload happens to contain the bytes
        # b5 62 05 01 02 00 06 8a — but the next 2 bytes are random and
        # don't form a valid Fletcher checksum.  Must NOT be matched.
        noise = b"\x00" * 32 + b"\xb5\x62\x05\x01\x02\x00\x06\x8a\x12\x34" + b"\x00" * 32
        stream = FakeStream([noise])
        ok = wait_ack(FakeUbr(stream), "CFG", "VALSET", timeout=0.2)
        self.assertFalse(ok)

    def test_frame_split_across_chunks(self):
        # The ACK arrives in two pieces: first 6 bytes, then last 4.
        ack = make_ack_frame(0x06, 0x8A, "ACK")
        stream = FakeStream([ack[:6], ack[6:]])
        ok = wait_ack(FakeUbr(stream), "CFG", "VALSET", timeout=0.5)
        self.assertTrue(ok)

    def test_obs_traffic_before_ack_doesnt_false_match(self):
        # Lots of UBX-NAV-PVT-shaped bytes followed by the real ACK.
        nav_pvt_like = b"\xb5\x62\x01\x07\x5c\x00" + b"\xaa" * 92 + b"\xab\xcd"
        ack = make_ack_frame(0x06, 0x8A, "ACK")
        stream = FakeStream([nav_pvt_like + ack])
        ok = wait_ack(FakeUbr(stream), "CFG", "VALSET", timeout=0.5)
        self.assertTrue(ok)

    def test_unknown_cls_msg_name_accepts_any_well_framed_ack(self):
        # If a caller passes a class/msg name we don't have a mapping
        # for, we warn but accept any well-framed ACK so callers aren't
        # blocked.  (Better: extend _UBX_CLSID_BY_NAME.)
        ack = make_ack_frame(0x06, 0x8A, "ACK")
        stream = FakeStream([ack])
        ok = wait_ack(FakeUbr(stream), "NOSUCH", "WHATEVER", timeout=0.2)
        self.assertTrue(ok)


# ─── pyubx2 fallback path ─────────────────────────────────────────── #


class _ParsedMsg:
    """Minimal stand-in for a pyubx2-parsed UBX-ACK message."""

    def __init__(self, identity: str, clsID: int, msgID: int):
        self.identity = identity
        self.clsID = clsID
        self.msgID = msgID


class FakeParsedUbr:
    """ubr.read()-style: yields (raw, parsed) tuples from a queue."""

    def __init__(self, msgs: Iterable):
        self._msgs = deque(msgs)

    def read(self):
        if not self._msgs:
            return b"", None
        return b"", self._msgs.popleft()


class WaitAckParsedFallbackTest(unittest.TestCase):

    def test_accepts_right_ack(self):
        ubr = FakeParsedUbr([_ParsedMsg("ACK-ACK", 0x06, 0x8A)])
        self.assertTrue(
            _wait_ack_parsed(ubr, "CFG", "VALSET", 0.5,
                             expected_payload=(0x06, 0x8A)))

    def test_rejects_nak_with_right_payload(self):
        ubr = FakeParsedUbr([_ParsedMsg("ACK-NAK", 0x06, 0x8A)])
        self.assertFalse(
            _wait_ack_parsed(ubr, "CFG", "VALSET", 0.5,
                             expected_payload=(0x06, 0x8A)))

    def test_ignores_ack_for_other_message(self):
        # ACK arrives but for CFG-RST, not VALSET.  Must be ignored.
        ubr = FakeParsedUbr([_ParsedMsg("ACK-ACK", 0x06, 0x04)])
        self.assertFalse(
            _wait_ack_parsed(ubr, "CFG", "VALSET", 0.2,
                             expected_payload=(0x06, 0x8A)))

    def test_skips_wrong_then_matches_right(self):
        ubr = FakeParsedUbr([
            _ParsedMsg("ACK-ACK", 0x06, 0x04),   # for CFG-RST, skipped
            _ParsedMsg("ACK-ACK", 0x06, 0x8A),   # for CFG-VALSET, matched
        ])
        self.assertTrue(
            _wait_ack_parsed(ubr, "CFG", "VALSET", 0.5,
                             expected_payload=(0x06, 0x8A)))

    def test_none_expected_payload_accepts_any_ack(self):
        ubr = FakeParsedUbr([_ParsedMsg("ACK-ACK", 0x99, 0x99)])
        self.assertTrue(
            _wait_ack_parsed(ubr, "CFG", "VALSET", 0.5,
                             expected_payload=None))


# ─── _UBX_CLSID_BY_NAME table sanity ──────────────────────────────── #


class ClsidTableTest(unittest.TestCase):
    """Spot-check the few entries downstream code depends on."""

    def test_cfg_valset_is_06_8a(self):
        self.assertEqual(_UBX_CLSID_BY_NAME[("CFG", "VALSET")], (0x06, 0x8A))

    def test_cfg_rst_is_06_04(self):
        self.assertEqual(_UBX_CLSID_BY_NAME[("CFG", "RST")], (0x06, 0x04))


if __name__ == "__main__":
    unittest.main()

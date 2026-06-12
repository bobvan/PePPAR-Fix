"""Fast vectorized UBX NAV-SIG decoder.

Like RXM-RAWX (see rawx_decode.py), UBX-NAV-SIG is a fixed 8-byte header
followed by a 16-byte repeating per-signal block.  On the X20 it carries
~104 signals/epoch, which pyubx2 deserializes field-by-field into ~1500
Python attributes at ~29 ms on a Pi 4 — on the serial-reader thread,
holding the GIL the whole time.  np.frombuffer over the block decodes the
whole epoch in ~7 µs (~4000× faster, parity-tested).

Note: pyubx2 expands the ``sigFlags`` X2 bitfield into ``prUsed_NN`` etc.
and does NOT expose a combined ``sigFlags_NN`` attribute, so the old
``getattr(parsed, 'sigFlags_NN', 0)`` path read 0 and left prUsed/health
silently false in production.  This decoder reads sigFlags from the bytes
directly, so Nav2SignalStore finally sees the real validity bits.

See docs/x20-vcocxo-arm-comparison-2026-06-12.md.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# UBX class/id for NAV-SIG (0x01 0x43).
NAV_SIG_CLS = 0x01
NAV_SIG_ID = 0x43

# Repeating 16-byte per-signal block, little-endian, per the UBX NAV-SIG
# spec.  prRes is signed 0.1 m units; sigFlags is an X2 bitfield.
_BLOCK = np.dtype([
    ("gnssId", "u1"), ("svId", "u1"), ("sigId", "u1"), ("freqId", "u1"),
    ("prRes", "<i2"), ("cno", "u1"), ("qualityInd", "u1"),
    ("corrSource", "u1"), ("ionoModel", "u1"),
    ("sigFlags", "<u2"), ("_rsv", "<u4"),
])
assert _BLOCK.itemsize == 16

# Fixed header: iTOW U4, version U1, numSigs U1, reserved0[2].
_HDR_LEN = 8


@dataclass
class NavSigEpoch:
    """One decoded NAV-SIG epoch.  Per-signal fields are numpy arrays of
    length ``numSigs`` (read-only views into the frame buffer)."""

    iTOW: int
    numSigs: int
    gnssId: np.ndarray
    svId: np.ndarray
    sigId: np.ndarray
    cno: np.ndarray
    qualityInd: np.ndarray
    prRes: np.ndarray       # raw signed int16, 0.1 m units
    sigFlags: np.ndarray    # raw uint16 bitfield


def is_nav_sig(raw: bytes) -> bool:
    """True iff ``raw`` is a UBX frame with the NAV-SIG class/id."""
    return len(raw) >= 4 and raw[2] == NAV_SIG_CLS and raw[3] == NAV_SIG_ID


def decode_nav_sig(raw: bytes) -> NavSigEpoch:
    """Decode a complete raw UBX NAV-SIG frame (sync … checksum).

    The frame is assumed already length-validated by the UBX framer.
    Raises ``ValueError`` for a non-NAV-SIG or truncated frame.
    """
    if not is_nav_sig(raw):
        raise ValueError("not a NAV-SIG frame")
    payload = raw[6:-2]
    if len(payload) < _HDR_LEN:
        raise ValueError("NAV-SIG payload shorter than header")
    iTOW = int(np.frombuffer(payload, dtype="<u4", count=1, offset=0)[0])
    numSigs = payload[5]
    need = _HDR_LEN + 16 * numSigs
    if len(payload) < need:
        raise ValueError(f"NAV-SIG truncated: payload {len(payload)} < {need}")
    m = np.frombuffer(payload, dtype=_BLOCK, count=numSigs, offset=_HDR_LEN)
    return NavSigEpoch(
        iTOW=iTOW, numSigs=int(numSigs),
        gnssId=m["gnssId"], svId=m["svId"], sigId=m["sigId"],
        cno=m["cno"], qualityInd=m["qualityInd"],
        prRes=m["prRes"], sigFlags=m["sigFlags"],
    )

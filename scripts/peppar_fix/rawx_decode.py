"""Fast vectorized UBX RXM-RAWX decoder.

pyubx2 deserializes an RXM-RAWX epoch field-by-field into one Python
attribute per field.  On the X20 a single epoch carries ~78 measurements
(× ~14 fields ≈ 1100 attribute-sets) and costs ~22 ms on a Pi 4 — and it
runs on the serial-reader thread, holding the GIL the whole time and
stalling the servo/correlation-gate main loop.

RXM-RAWX is a fixed 16-byte header followed by a 32-byte repeating
measurement block, so ``np.frombuffer`` with a structured dtype decodes the
whole epoch in one vectorized call: ~58 µs, 389× faster, bit-exact vs
pyubx2 (validated in ``test_rawx_decode`` and on live X20 frames).

See docs/x20-vcocxo-arm-comparison-2026-06-12.md (profiling section).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# UBX class/id for RXM-RAWX (0x02 0x15).
RXM_RAWX_CLS = 0x02
RXM_RAWX_ID = 0x15

# Repeating 32-byte measurement block, little-endian, per the UBX RXM-RAWX
# spec.  trkStat / *Stdev are single-byte bitfields decoded below.
_BLOCK = np.dtype([
    ("prMes", "<f8"), ("cpMes", "<f8"), ("doMes", "<f4"),
    ("gnssId", "u1"), ("svId", "u1"), ("sigId", "u1"), ("freqId", "u1"),
    ("locktime", "<u2"), ("cno", "u1"),
    ("prStdev", "u1"), ("cpStdev", "u1"), ("doStdev", "u1"),
    ("trkStat", "u1"), ("_rsv", "u1"),
])
assert _BLOCK.itemsize == 32

# Fixed header: rcvTow R8, week U2, leapS I1, numMeas U1, recStat X1,
# version U1, reserved[2].
_HDR_LEN = 16


@dataclass
class RawxEpoch:
    """One decoded RXM-RAWX epoch.

    Per-measurement fields are numpy arrays of length ``numMeas``.

    LIFETIME: these arrays are read-only **views into the source frame
    bytes**, not copies — decode is zero-copy by design.  They are valid
    only while the ``raw`` frame passed to ``decode_rawx`` is alive.  The
    serial-reader consumer reads scalars out synchronously within the same
    loop iteration (the frame outlives that), so this is safe.  A caller
    that wants to retain the epoch past the current frame must ``.copy()``
    the arrays first.
    """

    rcvTow: float
    week: int
    leapS: int
    numMeas: int
    gnssId: np.ndarray
    svId: np.ndarray
    sigId: np.ndarray
    freqId: np.ndarray
    prMes: np.ndarray
    cpMes: np.ndarray
    doMes: np.ndarray
    cno: np.ndarray
    locktime: np.ndarray
    prValid: np.ndarray     # bool
    cpValid: np.ndarray     # bool
    halfCyc: np.ndarray     # bool
    subHalfCyc: np.ndarray  # bool


def is_rawx(raw: bytes) -> bool:
    """True iff ``raw`` is a UBX frame with the RXM-RAWX class/id."""
    return len(raw) >= 4 and raw[2] == RXM_RAWX_CLS and raw[3] == RXM_RAWX_ID


def decode_rawx(raw: bytes) -> RawxEpoch:
    """Decode a complete raw UBX RXM-RAWX frame (sync … checksum).

    The frame is assumed already length-validated by the UBX framer
    (``UBXReader``), so the 2-byte checksum is trusted, not re-verified.

    Raises ``ValueError`` for a non-RAWX or truncated frame.
    """
    if not is_rawx(raw):
        raise ValueError("not an RXM-RAWX frame")
    payload = raw[6:-2]                       # strip 6-byte UBX hdr + 2-byte cksum
    if len(payload) < _HDR_LEN:
        raise ValueError("RAWX payload shorter than header")
    rcvTow = float(np.frombuffer(payload, dtype="<f8", count=1, offset=0)[0])
    week = int(np.frombuffer(payload, dtype="<u2", count=1, offset=8)[0])
    leapS = int(np.frombuffer(payload, dtype="i1", count=1, offset=10)[0])
    numMeas = payload[11]
    need = _HDR_LEN + 32 * numMeas
    if len(payload) < need:
        raise ValueError(f"RAWX truncated: payload {len(payload)} < {need}")
    m = np.frombuffer(payload, dtype=_BLOCK, count=numMeas, offset=_HDR_LEN)
    trk = m["trkStat"]
    return RawxEpoch(
        rcvTow=rcvTow, week=week, leapS=leapS, numMeas=int(numMeas),
        gnssId=m["gnssId"], svId=m["svId"], sigId=m["sigId"], freqId=m["freqId"],
        prMes=m["prMes"], cpMes=m["cpMes"], doMes=m["doMes"],
        cno=m["cno"], locktime=m["locktime"],
        prValid=(trk & 0x01).astype(bool),
        cpValid=((trk >> 1) & 0x01).astype(bool),
        halfCyc=((trk >> 2) & 0x01).astype(bool),
        subHalfCyc=((trk >> 3) & 0x01).astype(bool),
    )

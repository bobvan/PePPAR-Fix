"""Minimal PTP Management Client — talk directly to ptp4l's Unix domain socket.

Implements just enough of the IEEE 1588 management message protocol to
SET GRANDMASTER_SETTINGS_NP.  This avoids shelling out to the ``pmc``
command for latency-sensitive clock-class transitions inside the engine.

The ptp4l daemon listens on a Unix datagram socket (default
``/var/run/ptp4l``).  Each management message is a single datagram
containing a PTP header, management header, and management TLV.

References:
    - IEEE 1588-2008 §15 (management messages)
    - linuxptp msg.h, tlv.h, pmc_common.c
"""

from __future__ import annotations

import logging
import os
import socket
import struct
import tempfile

log = logging.getLogger("peppar-fix")

# PTP message types
MSG_MANAGEMENT = 0x0D

# Management action field (lower 4 bits of flags byte)
ACTION_SET = 0x01

# TLV types
TLV_MANAGEMENT = 0x0001

# Management IDs (linuxptp tlv.h)
MID_GRANDMASTER_SETTINGS_NP = 0xC001

# IEEE 1588 time_flags bits
FLAG_LEAP_61 = 1 << 0
FLAG_LEAP_59 = 1 << 1
FLAG_UTC_OFF_VALID = 1 << 2
FLAG_PTP_TIMESCALE = 1 << 3
FLAG_TIME_TRACEABLE = 1 << 4
FLAG_FREQ_TRACEABLE = 1 << 5

# IEEE 1588 clockAccuracy enumeration (selected values)
ACCURACY_25NS = 0x20
ACCURACY_100NS = 0x21
ACCURACY_250NS = 0x22
ACCURACY_1US = 0x23
ACCURACY_UNKNOWN = 0xFE

# σ_total → clockAccuracy quantization ladder (I-071400 E2 / I-060548).
# (upper-bound ns, enum) in ascending order.  A σ_total (which folds in the
# position σ_t = σ_r·SIGMA_POS_NS_PER_M via confidence.py) maps to the SMALLEST
# bucket whose bound ≥ σ_total — a conservative ceiling, matching the "advertise
# the honest worst case" framing used for clockClass.  Without this the
# advertised clockAccuracy is a fixed per-clock-class constant, so the 100 ns /
# 250 ns buckets never emit and e.g. a ~33 ns NAV2-bootstrap σ_t jumps straight
# to the 1 µs bucket.
_ACCURACY_LADDER = (
    (25.0, ACCURACY_25NS),
    (100.0, ACCURACY_100NS),
    (250.0, ACCURACY_250NS),
    (1000.0, ACCURACY_1US),
)


def clock_accuracy_for_sigma_ns(sigma_ns) -> int:
    """Quantize a σ_total (ns) to the smallest clockAccuracy enum whose bound
    covers it; ACCURACY_UNKNOWN above the ladder (>1 µs) or for None/NaN/≤0."""
    if sigma_ns is None:
        return ACCURACY_UNKNOWN
    try:
        s = float(sigma_ns)
    except (TypeError, ValueError):
        return ACCURACY_UNKNOWN
    if not (s > 0) or s != s:  # ≤0 or NaN — no honest bound
        return ACCURACY_UNKNOWN
    for bound_ns, enum in _ACCURACY_LADDER:
        if s <= bound_ns:
            return enum
    return ACCURACY_UNKNOWN

# IEEE 1588 timeSource enumeration (selected values)
TIME_SOURCE_GPS = 0x20
TIME_SOURCE_PTP = 0x40
TIME_SOURCE_HAND_SET = 0x60
TIME_SOURCE_INTERNAL_OSCILLATOR = 0xA0

# Current GPS-UTC offset (as of 2025, 18 leap seconds)
DEFAULT_UTC_OFFSET = 37


def _build_management_msg(
    sequence_id: int,
    action: int,
    management_id: int,
    data: bytes,
    domain: int = 0,
    boundary_hops: int = 0,
) -> bytes:
    """Build a complete PTP management message as a bytes object."""
    # --- PTP header (34 bytes) ---
    header = bytearray(34)
    header[0] = MSG_MANAGEMENT  # tsmt: transportSpecific=0 | messageType
    header[1] = 0x02  # ver: versionPTP=2
    # messageLength filled below
    header[4] = domain
    # flagField, correction, reserved2 = 0
    # sourcePortIdentity: clockIdentity=0, portNumber=pid
    struct.pack_into(">H", header, 28, os.getpid() & 0xFFFF)
    struct.pack_into(">H", header, 30, sequence_id)
    header[32] = 0x04  # control = management
    header[33] = 0x7F  # logMessageInterval = 0x7F per IEEE 1588

    # --- Management header (14 bytes) ---
    mgmt = bytearray(14)
    # targetPortIdentity: all-ones = wildcard (all clocks, all ports)
    mgmt[0:8] = b"\xff" * 8
    struct.pack_into(">H", mgmt, 8, 0xFFFF)
    mgmt[10] = boundary_hops  # startingBoundaryHops
    mgmt[11] = boundary_hops  # boundaryHops
    mgmt[12] = action & 0x0F  # flags: actionField in lower nibble

    # --- Management TLV ---
    tlv_data_len = 2 + len(data)  # managementId + data
    tlv = struct.pack(">HHH", TLV_MANAGEMENT, tlv_data_len, management_id) + data

    msg = bytes(header) + bytes(mgmt) + tlv
    # Patch messageLength
    msg = msg[:2] + struct.pack(">H", len(msg)) + msg[4:]
    return msg


def _build_grandmaster_settings(
    clock_class: int,
    clock_accuracy: int = ACCURACY_25NS,
    offset_scaled_log_variance: int = 0xFFFF,
    utc_offset: int = DEFAULT_UTC_OFFSET,
    time_flags: int = (
        FLAG_UTC_OFF_VALID | FLAG_PTP_TIMESCALE | FLAG_TIME_TRACEABLE | FLAG_FREQ_TRACEABLE
    ),
    time_source: int = TIME_SOURCE_GPS,
) -> bytes:
    """Pack a grandmaster_settings_np structure (8 bytes)."""
    return struct.pack(
        ">BBHhBB",
        clock_class,
        clock_accuracy,
        offset_scaled_log_variance,
        utc_offset,
        time_flags,
        time_source,
    )


# ── Preset clock-class configurations ──────────────────────────────── #

def gm_settings_locked() -> bytes:
    """clockClass 6: locked to primary GNSS reference."""
    return _build_grandmaster_settings(
        clock_class=6,
        clock_accuracy=ACCURACY_25NS,
        time_flags=FLAG_UTC_OFF_VALID | FLAG_PTP_TIMESCALE | FLAG_TIME_TRACEABLE | FLAG_FREQ_TRACEABLE,
        time_source=TIME_SOURCE_GPS,
    )


def gm_settings_initialized() -> bytes:
    """clockClass 52: PHC bootstrapped (phase/freq set), servo not yet settled."""
    return _build_grandmaster_settings(
        clock_class=52,
        clock_accuracy=ACCURACY_1US,
        time_flags=FLAG_UTC_OFF_VALID | FLAG_PTP_TIMESCALE | FLAG_TIME_TRACEABLE,
        time_source=TIME_SOURCE_GPS,
    )


def gm_settings_holdover() -> bytes:
    """clockClass 7: holdover from previously locked state."""
    return _build_grandmaster_settings(
        clock_class=7,
        clock_accuracy=ACCURACY_1US,
        time_flags=FLAG_UTC_OFF_VALID | FLAG_PTP_TIMESCALE | FLAG_TIME_TRACEABLE,
        time_source=TIME_SOURCE_GPS,
    )


def gm_settings_freerun() -> bytes:
    """clockClass 248: free-running / unlocked / untrusted."""
    return _build_grandmaster_settings(
        clock_class=248,
        clock_accuracy=ACCURACY_UNKNOWN,
        time_flags=FLAG_PTP_TIMESCALE,
        time_source=TIME_SOURCE_INTERNAL_OSCILLATOR,
    )


# ── Client ─────────────────────────────────────────────────────────── #

class PmcClient:
    """Send PTP management messages to ptp4l over its Unix domain socket.

    Usage::

        with PmcClient("/var/run/ptp4l") as pmc:
            pmc.set_grandmaster_class("locked")
    """

    def __init__(self, uds_path: str, domain: int = 0):
        self._server_path = uds_path
        self._domain = domain
        self._seq = 0
        self._sock = None
        self._client_path = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    def open(self):
        if self._sock is not None:
            return
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        # Bind to a temporary path so ptp4l can send a response (even
        # though we don't read it for SET operations).
        fd, self._client_path = tempfile.mkstemp(prefix="peppar-pmc-")
        os.close(fd)
        os.unlink(self._client_path)
        self._sock.bind(self._client_path)
        self._sock.settimeout(2.0)

    def close(self):
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        if self._client_path and os.path.exists(self._client_path):
            try:
                os.unlink(self._client_path)
            except OSError:
                pass
            self._client_path = None

    def _send(self, action: int, management_id: int, data: bytes) -> bool:
        """Send a management message.  Returns True on success."""
        if self._sock is None:
            self.open()
        msg = _build_management_msg(
            self._seq, action, management_id, data,
            domain=self._domain,
        )
        self._seq += 1
        try:
            self._sock.sendto(msg, self._server_path)
            return True
        except OSError as e:
            log.warning("pmc: sendto %s failed: %s", self._server_path, e)
            return False

    def set_grandmaster_settings(self, data: bytes) -> bool:
        """Send SET GRANDMASTER_SETTINGS_NP with pre-built *data*."""
        return self._send(ACTION_SET, MID_GRANDMASTER_SETTINGS_NP, data)

    def set_grandmaster_class(self, state: str,
                              clock_accuracy: int = None) -> bool:
        """Set clockClass by named state.

        States: 'locked' (6), 'initialized' (52), 'holdover' (7), 'freerun' (248).

        clock_accuracy: optional IEEE-1588 clockAccuracy enum override
        (I-071400 E2).  When given, the preset's fixed accuracy byte is
        replaced with this σ_total-derived value (clockClass — the
        traceability state — is unchanged), so the advertised accuracy
        reflects the actual time confidence instead of a per-class constant.
        When None, the preset's accuracy is used (byte-identical legacy).

        Returns True if the message was sent successfully.
        """
        presets = {
            "locked": gm_settings_locked,
            "initialized": gm_settings_initialized,
            "holdover": gm_settings_holdover,
            "freerun": gm_settings_freerun,
        }
        builder = presets.get(state)
        if builder is None:
            raise ValueError(f"Unknown GM state {state!r}, expected one of {list(presets)}")
        settings = builder()
        if clock_accuracy is not None:
            # grandmaster_settings_np is packed ">BBHhBB" (clock_class,
            # clock_accuracy, ...): patch byte 1 in place, leaving clockClass
            # and every other field exactly as the preset built them.
            settings = settings[:1] + bytes((clock_accuracy & 0xFF,)) + settings[2:]
        ok = self.set_grandmaster_settings(settings)
        if ok:
            if clock_accuracy is not None:
                log.info("pmc: SET clockClass → %s (accuracy=0x%02x)",
                         state, clock_accuracy & 0xFF)
            else:
                log.info("pmc: SET clockClass → %s", state)
        return ok

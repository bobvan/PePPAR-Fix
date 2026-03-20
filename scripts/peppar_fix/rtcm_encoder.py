"""RTCM 3.3 message encoding for NTRIP caster.

Encodes GNSS observations as RTCM MSM4 (message types 1074/1094/1124)
and station reference position as RTCM 1005.

RTCM MSM4 provides: full pseudoranges, full carrier phases, lock time
indicators, half-cycle ambiguity flags, and C/N0 for each signal.
"""

import math

# ── Constants ────────────────────────────────────────────────────────────────── #

C_LIGHT = 299792458.0  # speed of light (m/s)
C_LIGHT_MS = C_LIGHT / 1000.0  # m per millisecond

# GNSS frequencies (Hz)
FREQ = {
    'GPS-L1CA': 1575.42e6,
    'GPS-L5Q':  1176.45e6,
    'GAL-E1C':  1575.42e6,
    'GAL-E5aQ': 1176.45e6,
    'BDS-B1I':  1561.098e6,
    'BDS-B2aI': 1176.45e6,
}

# Wavelengths (m)
WAVELENGTH = {sig: C_LIGHT / f for sig, f in FREQ.items()}

# UBX signal name → RTCM MSM signal ID (1-based, per RTCM 3.3 Tables)
# GPS: Table 3.5-91, GAL: Table 3.5-93, BDS: Table 3.5-95 + Amendment 1
MSM_SIGNAL_ID = {
    'GPS-L1CA': 2,     # 1C
    'GPS-L5Q':  16,    # 5Q
    'GAL-E1C':  2,     # 1C
    'GAL-E5aQ': 12,    # 5Q
    'BDS-B1I':  2,     # 2I
    'BDS-B2aI': 14,    # 5D (B2a data)
}

# u-blox gnssId → system name
GNSS_ID_TO_SYS = {0: 'GPS', 2: 'GAL', 3: 'BDS'}

# System → MSM4 message type
SYS_MSG_TYPE = {'GPS': 1074, 'GAL': 1094, 'BDS': 1124}

# UBX (gnssId, sigId) → signal name (F9T PROTVER 29.x)
UBX_SIG_NAMES = {
    (0, 0): 'GPS-L1CA', (0, 7): 'GPS-L5Q',
    (2, 0): 'GAL-E1C',  (2, 4): 'GAL-E5aQ',
    (3, 0): 'BDS-B1I',  (3, 5): 'BDS-B2aI',
}

# BDS-2 GEO/IGSO PRNs to skip (same as realtime_ppp.py)
BDS_MIN_PRN = 19

# Lock time indicator thresholds in seconds (DF402, 4-bit)
_LOCK_THRESHOLDS = [
    0, 24, 72, 120, 240, 480, 720, 960,
    1920, 3840, 5760, 11520, 23040, 46080, 92160, 131072,
]


# ── CRC-24Q (RTCM standard) ─────────────────────────────────────────────────── #

_CRC24Q_TABLE = None


def _init_crc24q():
    global _CRC24Q_TABLE
    if _CRC24Q_TABLE is not None:
        return
    _CRC24Q_TABLE = [0] * 256
    for i in range(256):
        crc = i << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
        _CRC24Q_TABLE[i] = crc & 0xFFFFFF


def crc24q(data):
    """Compute CRC-24Q checksum for RTCM3 frame."""
    _init_crc24q()
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ _CRC24Q_TABLE[((crc >> 16) ^ b) & 0xFF]) & 0xFFFFFF
    return crc


# ── Bit-level packing ────────────────────────────────────────────────────────── #

class BitWriter:
    """MSB-first bit packing for RTCM message encoding."""

    __slots__ = ('_data', '_byte', '_pos')

    def __init__(self):
        self._data = bytearray()
        self._byte = 0
        self._pos = 0  # bits used in current byte

    def put_uint(self, value, nbits):
        """Write unsigned integer value in nbits (MSB first)."""
        for i in range(nbits - 1, -1, -1):
            self._byte = (self._byte << 1) | ((value >> i) & 1)
            self._pos += 1
            if self._pos == 8:
                self._data.append(self._byte)
                self._byte = 0
                self._pos = 0

    def put_int(self, value, nbits):
        """Write signed (two's complement) integer in nbits."""
        if value < 0:
            value += (1 << nbits)
        self.put_uint(value, nbits)

    def to_bytes(self):
        """Return packed bytes, zero-padding to byte boundary."""
        if self._pos > 0:
            result = bytearray(self._data)
            result.append(self._byte << (8 - self._pos))
            return bytes(result)
        return bytes(self._data)


# ── RTCM framing ─────────────────────────────────────────────────────────────── #

def rtcm_frame(payload):
    """Wrap payload bytes in an RTCM3 frame (preamble + length + CRC-24Q)."""
    length = len(payload)
    header = bytes([0xD3, (length >> 8) & 0x03, length & 0xFF])
    crc = crc24q(header + payload)
    return header + payload + bytes([(crc >> 16) & 0xFF,
                                     (crc >> 8) & 0xFF,
                                     crc & 0xFF])


# ── RTCM 1005: Stationary RTK Reference Station ARP ─────────────────────────── #

def encode_1005(station_id, ecef_x, ecef_y, ecef_z,
                gps=True, gal=True, bds=False):
    """Encode RTCM 1005 message (152-bit payload).

    Args:
        station_id: Reference station ID (0-4095)
        ecef_x, ecef_y, ecef_z: ARP coordinates in meters (ITRF/WGS84)
        gps, gal, bds: System indicator flags

    Returns:
        Complete RTCM3 frame bytes
    """
    bw = BitWriter()
    bw.put_uint(1005, 12)           # DF002: Message Number
    bw.put_uint(station_id, 12)     # DF003: Reference Station ID
    bw.put_uint(0, 6)               # DF021: ITRF Realization Year (0=unspecified)
    bw.put_uint(1 if gps else 0, 1) # DF022: GPS Indicator
    bw.put_uint(0, 1)               # DF023: GLONASS Indicator
    bw.put_uint(1 if gal else 0, 1) # DF024: Galileo Indicator
    bw.put_uint(0, 1)               # DF141: Reference-Station (0=real physical)
    bw.put_int(round(ecef_x * 10000), 38)  # DF025: ARP ECEF-X (0.0001 m)
    bw.put_uint(0, 1)               # DF142: Single Receiver Oscillator
    bw.put_uint(0, 1)               # DF001: Reserved
    bw.put_int(round(ecef_y * 10000), 38)  # DF026: ARP ECEF-Y (0.0001 m)
    bw.put_uint(0, 2)               # DF364: Quarter Cycle Indicator
    bw.put_int(round(ecef_z * 10000), 38)  # DF027: ARP ECEF-Z (0.0001 m)
    return rtcm_frame(bw.to_bytes())


# ── Lock time indicator ──────────────────────────────────────────────────────── #

def _lock_indicator(lock_ms):
    """Convert lock time in ms to 4-bit MSM4 indicator (DF402)."""
    lock_s = lock_ms / 1000.0
    ind = 0
    for i, thresh in enumerate(_LOCK_THRESHOLDS):
        if lock_s >= thresh:
            ind = i
        else:
            break
    return min(ind, 15)


# ── RTCM MSM4 encoding ──────────────────────────────────────────────────────── #

def encode_msm4(msg_type, station_id, epoch_ms, sat_obs, is_last=True):
    """Encode a single RTCM MSM4 message for one GNSS system.

    Args:
        msg_type: RTCM message type (1074/1094/1124)
        station_id: Reference station ID (0-4095)
        epoch_ms: GNSS system epoch time in ms (30-bit, TOW for GPS/GAL, BDT for BDS)
        sat_obs: dict {prn: {sig_name: {pr, cp, cno, lock_ms, half_cyc}}}
            prn: integer satellite PRN
            sig_name: signal name string (e.g., 'GPS-L1CA')
            pr: pseudorange in meters
            cp: carrier phase in cycles (None if invalid)
            cno: C/N0 in dB-Hz
            lock_ms: lock time in milliseconds
            half_cyc: 1 if half-cycle NOT resolved, 0 if resolved
        is_last: True if this is the last MSM message of the epoch

    Returns:
        Complete RTCM3 frame bytes
    """
    if not sat_obs:
        return b''

    # Determine satellite and signal sets
    prns = sorted(sat_obs.keys())
    all_sigs = set()
    for sigs in sat_obs.values():
        for sig_name in sigs:
            if sig_name in MSM_SIGNAL_ID:
                all_sigs.add(sig_name)
    if not all_sigs:
        return b''

    sig_ids = sorted(set(MSM_SIGNAL_ID[s] for s in all_sigs))
    n_sat = len(prns)
    n_sig = len(sig_ids)

    # Build satellite mask (64 bits, bit 63=PRN1, bit 0=PRN64)
    sat_mask = 0
    for prn in prns:
        sat_mask |= (1 << (64 - prn))

    # Build signal mask (32 bits, bit 31=sig1, bit 0=sig32)
    sig_mask = 0
    for sid in sig_ids:
        sig_mask |= (1 << (32 - sid))

    # Build cell mask and cell list
    # Cell order: for each satellite (in PRN order), for each signal (in ID order)
    cell_mask_bits = []
    cells = []  # (prn, sig_name) tuples for cells with data
    for prn in prns:
        sigs = sat_obs[prn]
        for sid in sig_ids:
            # Find signal name for this signal ID in this satellite's observations
            sig_name = None
            for sn in sigs:
                if sn in MSM_SIGNAL_ID and MSM_SIGNAL_ID[sn] == sid:
                    sig_name = sn
                    break
            if sig_name is not None:
                cell_mask_bits.append(1)
                cells.append((prn, sig_name))
            else:
                cell_mask_bits.append(0)

    if not cells:
        return b''

    # Compute rough ranges per satellite (from first available pseudorange)
    rough_ranges = {}  # prn → (int_ms, frac_1024)
    for prn in prns:
        sigs = sat_obs[prn]
        pr_m = None
        for sid in sig_ids:
            for sn in sigs:
                if sn in MSM_SIGNAL_ID and MSM_SIGNAL_ID[sn] == sid:
                    pr_m = sigs[sn].get('pr')
                    if pr_m is not None:
                        break
            if pr_m is not None:
                break
        if pr_m is None:
            rough_ranges[prn] = (255, 0)  # invalid
            continue
        pr_ms = pr_m / C_LIGHT_MS
        int_ms = int(math.floor(pr_ms))
        frac = pr_ms - int_ms
        frac_1024 = round(frac * 1024)
        if frac_1024 >= 1024:
            frac_1024 = 0
            int_ms += 1
        if int_ms > 254:
            rough_ranges[prn] = (255, 0)  # invalid (shouldn't happen for GNSS)
        else:
            rough_ranges[prn] = (int_ms, frac_1024)

    # ── Encode header ──────────────────────────────────────────────────────── #
    bw = BitWriter()
    bw.put_uint(msg_type, 12)          # DF002: Message Number
    bw.put_uint(station_id, 12)        # DF003: Reference Station ID
    bw.put_uint(epoch_ms & 0x3FFFFFFF, 30)  # DF004/248/427: Epoch Time (ms)
    bw.put_uint(0 if is_last else 1, 1)  # DF393: Multiple Message
    bw.put_uint(0, 3)                  # DF409: IODS
    bw.put_uint(0, 7)                  # DF001: Reserved
    bw.put_uint(0, 2)                  # DF411: Clock Steering (0=unknown)
    bw.put_uint(0, 2)                  # DF412: External Clock (0=unknown)
    bw.put_uint(0, 1)                  # DF417: Smoothing Type (0=no smoothing)
    bw.put_uint(0, 3)                  # DF418: Smoothing Interval (0=no smoothing)
    bw.put_uint(sat_mask >> 32, 32)    # DF394: Satellite Mask (upper 32)
    bw.put_uint(sat_mask & 0xFFFFFFFF, 32)  # DF394: Satellite Mask (lower 32)
    bw.put_uint(sig_mask, 32)          # DF395: Signal Mask
    for bit in cell_mask_bits:
        bw.put_uint(bit, 1)           # DF396: Cell Mask

    # ── Encode satellite data ──────────────────────────────────────────────── #
    for prn in prns:
        int_ms, frac_1024 = rough_ranges[prn]
        bw.put_uint(int_ms, 8)         # DF397: Rough Range integer ms
        bw.put_uint(frac_1024, 10)     # DF398: Rough Range modulo 1ms

    # ── Encode signal data (per cell) ──────────────────────────────────────── #
    # Fine pseudoranges (DF400)
    for prn, sig_name in cells:
        obs = sat_obs[prn][sig_name]
        pr_m = obs.get('pr')
        if pr_m is None:
            bw.put_int(-16384, 15)  # invalid
            continue
        int_ms, frac_1024 = rough_ranges[prn]
        if int_ms == 255:
            bw.put_int(-16384, 15)  # invalid satellite
            continue
        rough_ms = int_ms + frac_1024 / 1024.0
        pr_ms = pr_m / C_LIGHT_MS
        fine_ms = pr_ms - rough_ms
        fine_val = round(fine_ms / (2**-24))
        fine_val = max(-16383, min(16383, fine_val))
        bw.put_int(fine_val, 15)       # DF400: Fine Pseudorange

    # Fine carrier phases (DF401)
    for prn, sig_name in cells:
        obs = sat_obs[prn][sig_name]
        cp = obs.get('cp')
        if cp is None:
            bw.put_int(-2097152, 22)  # invalid
            continue
        int_ms, frac_1024 = rough_ranges[prn]
        if int_ms == 255:
            bw.put_int(-2097152, 22)  # invalid
            continue
        rough_ms = int_ms + frac_1024 / 1024.0
        wl = WAVELENGTH.get(sig_name, 0.19029)
        phase_ms = cp * wl / C_LIGHT_MS
        fine_ms = phase_ms - rough_ms
        # Wrap to representable range (±2^-8 ms)
        max_range = 2**-8
        while fine_ms > max_range:
            fine_ms -= 2 * max_range
        while fine_ms < -max_range:
            fine_ms += 2 * max_range
        fine_val = round(fine_ms / (2**-29))
        fine_val = max(-2097151, min(2097151, fine_val))
        bw.put_int(fine_val, 22)       # DF401: Fine PhaseRange

    # Lock time indicators (DF402)
    for prn, sig_name in cells:
        obs = sat_obs[prn][sig_name]
        bw.put_uint(_lock_indicator(obs.get('lock_ms', 0)), 4)

    # Half-cycle ambiguity (DF403)
    for prn, sig_name in cells:
        obs = sat_obs[prn][sig_name]
        bw.put_uint(obs.get('half_cyc', 0) & 1, 1)

    # Signal CNR (DF404)
    for prn, sig_name in cells:
        obs = sat_obs[prn][sig_name]
        cno = obs.get('cno', 0)
        bw.put_uint(min(63, max(0, round(cno))), 6)

    return rtcm_frame(bw.to_bytes())


# ── High-level epoch encoder ─────────────────────────────────────────────────── #

def encode_epoch(station_id, gps_tow_ms, raw_obs, position_ecef=None):
    """Encode a complete observation epoch as RTCM frames.

    Args:
        station_id: Reference station ID (0-4095)
        gps_tow_ms: GPS time of week in milliseconds
        raw_obs: dict {system: {prn: {sig_name: {pr, cp, cno, lock_ms, half_cyc}}}}
            system: 'GPS', 'GAL', or 'BDS'
        position_ecef: (x, y, z) in meters, or None to skip RTCM 1005

    Returns:
        bytes: concatenated RTCM frames for the epoch
    """
    frames = bytearray()

    # Optionally include RTCM 1005 (reference station position)
    if position_ecef is not None:
        frames.extend(encode_1005(station_id, *position_ecef))

    # Encode MSM4 for each system present
    systems = [s for s in ('GPS', 'GAL', 'BDS') if s in raw_obs and raw_obs[s]]
    for i, sys_name in enumerate(systems):
        msg_type = SYS_MSG_TYPE[sys_name]
        sat_obs = raw_obs[sys_name]
        is_last = (i == len(systems) - 1)

        # Compute system-specific epoch time
        if sys_name == 'BDS':
            epoch_ms = (gps_tow_ms - 14000) % 604800000
        else:
            epoch_ms = gps_tow_ms % 604800000

        frame = encode_msm4(msg_type, station_id, epoch_ms, sat_obs, is_last)
        if frame:
            frames.extend(frame)

    return bytes(frames)

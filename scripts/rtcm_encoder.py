#!/usr/bin/env python3
"""
rtcm_encoder.py — RTCM 3.3 message encoder for NTRIP caster.

Encodes raw GNSS observations into RTCM MSM4 messages (1074 GPS, 1094 Galileo,
1124 BDS) and RTCM 1005 (station ARP position). This enables PePPAR Fix to
act as an NTRIP caster for peer bootstrap.

RTCM MSM4 contains:
  - Full pseudorange and carrier phase for each satellite/signal
  - C/N0 and lock time indicators
  - Suitable for RTK/PPP processing by downstream receivers

References:
  - RTCM 10403.3 (RTCM Standard for Differential GNSS)
  - BKG NTRIP docs
  - IGS RTCM-SSR documentation
"""

import math
import struct
import time
from datetime import datetime, timezone, timedelta

# Reuse CRC-24Q from the existing ntrip_client
from ntrip_client import crc24q


# ── Bit-level writer ──────────────────────────────────────────────────────── #

class BitWriter:
    """Pack arbitrary bit-width fields into a byte buffer."""

    def __init__(self):
        self._bits = []

    def write(self, value, n_bits):
        """Write an unsigned integer of n_bits width."""
        if value < 0:
            # Two's complement for signed fields
            value = value & ((1 << n_bits) - 1)
        for i in range(n_bits - 1, -1, -1):
            self._bits.append((value >> i) & 1)

    def write_signed(self, value, n_bits):
        """Write a signed integer (two's complement)."""
        if value < 0:
            value = (1 << n_bits) + value
        self.write(value, n_bits)

    def to_bytes(self):
        """Return the packed bytes, padding the last byte with zeros."""
        # Pad to byte boundary
        while len(self._bits) % 8 != 0:
            self._bits.append(0)
        result = bytearray()
        for i in range(0, len(self._bits), 8):
            byte = 0
            for j in range(8):
                byte = (byte << 1) | self._bits[i + j]
            result.append(byte)
        return bytes(result)

    @property
    def bit_count(self):
        return len(self._bits)


# ── RTCM framing ─────────────────────────────────────────────────────────── #

def rtcm_frame(payload):
    """Wrap an RTCM payload in the standard frame: preamble + length + payload + CRC-24Q."""
    length = len(payload)
    if length > 1023:
        raise ValueError(f"RTCM payload too long: {length} > 1023")
    header = bytes([0xD3, (length >> 8) & 0x03, length & 0xFF])
    data = header + payload
    crc = crc24q(data)
    return data + bytes([(crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF])


# ── RTCM 1005: Stationary RTK Reference Station ARP ──────────────────────── #

# RTCM 1005 fields (DF002=1005, total 152 bits of payload):
#   DF002 (12): message number = 1005
#   DF003 (12): reference station ID
#   DF021 (6): ITRF realization year
#   DF022 (1): GPS indicator
#   DF023 (1): GLONASS indicator
#   DF024 (1): Galileo indicator
#   DF141 (1): reference station indicator (0=physical, 1=non-physical)
#   DF025 (38): ARP ECEF-X (0.0001m, signed)
#   DF142 (1): single receiver oscillator indicator
#   DF001 (1): reserved
#   DF026 (38): ARP ECEF-Y (0.0001m, signed)
#   DF364 (2): quarter cycle indicator
#   DF027 (38): ARP ECEF-Z (0.0001m, signed)

def encode_1005(ecef_m, station_id=0):
    """Encode RTCM 1005 (station reference position).

    Args:
        ecef_m: [x, y, z] ECEF position in meters
        station_id: reference station ID (0-4095)

    Returns:
        Complete RTCM frame bytes
    """
    bw = BitWriter()

    # DF002: message number
    bw.write(1005, 12)
    # DF003: station ID
    bw.write(station_id & 0xFFF, 12)
    # DF021: ITRF realization year (0 = unspecified)
    bw.write(0, 6)
    # DF022: GPS indicator (1 = GPS used)
    bw.write(1, 1)
    # DF023: GLONASS indicator (0)
    bw.write(0, 1)
    # DF024: Galileo indicator (1)
    bw.write(1, 1)
    # DF141: reference station indicator (0 = physical)
    bw.write(0, 1)
    # DF025: ECEF-X in 0.0001m units (38-bit signed)
    x_units = int(round(ecef_m[0] / 0.0001))
    bw.write_signed(x_units, 38)
    # DF142: single receiver oscillator (0)
    bw.write(0, 1)
    # DF001: reserved
    bw.write(0, 1)
    # DF026: ECEF-Y in 0.0001m units (38-bit signed)
    y_units = int(round(ecef_m[1] / 0.0001))
    bw.write_signed(y_units, 38)
    # DF364: quarter cycle indicator (0 = correction not applied)
    bw.write(0, 2)
    # DF027: ECEF-Z in 0.0001m units (38-bit signed)
    z_units = int(round(ecef_m[2] / 0.0001))
    bw.write_signed(z_units, 38)

    return rtcm_frame(bw.to_bytes())


# ── RTCM MSM4 encoding ───────────────────────────────────────────────────── #

# MSM message numbers by system
MSM4_MSG = {
    'G': 1074,  # GPS MSM4
    'E': 1094,  # Galileo MSM4
    'C': 1124,  # BDS MSM4
}

# Frequencies for converting carrier phase cycles to meters
# (needed for rough range and phase range computations)
C_LIGHT = 299792458.0  # m/s

# GNSS reference frequencies for MSM pseudorange/phase encoding
# MSM uses "rough range" in milliseconds (DF397/DF398) and fine residuals
RANGE_MS = C_LIGHT / 1000.0  # meters per millisecond of light travel time

# Signal IDs for MSM (RTCM 10403.3 Table 3.5-91 GPS, 3.5-96 GAL, 3.5-104 BDS)
# Map our signal names to RTCM MSM signal IDs
MSM_SIGNAL_ID = {
    # GPS signals
    'GPS-L1CA': 2,   # 1C
    'GPS-L2CL': 9,   # 2L
    'GPS-L2CM': 8,   # 2S (close enough)
    'GPS-L5I': 14,   # 5I
    'GPS-L5Q': 15,   # 5Q
    # Galileo signals
    'GAL-E1C': 2,    # 1C
    'GAL-E1B': 1,    # 1B (actually mapped to signal ID 1 in RTCM table)
    'GAL-E5aI': 12,  # 5I
    'GAL-E5aQ': 13,  # 5Q
    'GAL-E5bI': 7,   # 7I
    'GAL-E5bQ': 8,   # 7Q
    # BDS signals
    'BDS-B1I': 2,    # 2I
    'BDS-B1C': 4,    # 1D (approximation)
    'BDS-B2aI': 14,  # 5I (B2a maps to signal slot 14)
    'BDS-B2I': 8,    # 7I
}

# Signal frequencies in Hz (for carrier phase wavelength)
SIGNAL_FREQ = {
    'GPS-L1CA': 1575.42e6, 'GPS-L2CL': 1227.60e6, 'GPS-L2CM': 1227.60e6,
    'GPS-L5I': 1176.45e6, 'GPS-L5Q': 1176.45e6,
    'GAL-E1C': 1575.42e6, 'GAL-E1B': 1575.42e6,
    'GAL-E5aI': 1176.45e6, 'GAL-E5aQ': 1176.45e6,
    'GAL-E5bI': 1207.14e6, 'GAL-E5bQ': 1207.14e6,
    'BDS-B1I': 1561.098e6, 'BDS-B1C': 1575.42e6,
    'BDS-B2aI': 1176.45e6, 'BDS-B2I': 1207.14e6,
}


def _gps_tow_ms(gps_time):
    """Convert a datetime (GPS time) to GPS time-of-week in milliseconds."""
    gps_epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
    total_seconds = (gps_time - gps_epoch).total_seconds()
    week_seconds = total_seconds % (7 * 86400)
    return int(round(week_seconds * 1000))


def _gal_tow_ms(gps_time):
    """Galileo uses the same TOW as GPS (GST = GPS time)."""
    return _gps_tow_ms(gps_time)


def _bds_tow_ms(gps_time):
    """BDS time = GPS time - 14 seconds."""
    bds_time = gps_time - timedelta(seconds=14)
    return _gps_tow_ms(bds_time)


def _encode_msm_header(bw, msg_number, station_id, tow_ms, sync_flag,
                        sat_mask, signal_mask, cell_mask):
    """Encode MSM header (common to all MSM types).

    Args:
        bw: BitWriter
        msg_number: RTCM message number (e.g. 1074)
        station_id: reference station ID
        tow_ms: GNSS-specific epoch time in ms
        sync_flag: 1 if more MSM messages follow for same epoch
        sat_mask: 64-bit satellite mask (bit per satellite 1-64)
        signal_mask: 32-bit signal mask (bit per signal type 1-32)
        cell_mask: list of bits (n_sat * n_sig) indicating which cells exist
    """
    # DF002: message number (12 bits)
    bw.write(msg_number, 12)
    # DF003: station ID (12 bits)
    bw.write(station_id & 0xFFF, 12)

    # DF004/DF248/DF427: GNSS epoch time (30 bits)
    # GPS/GAL: DF004 (30 bits, ms of week)
    # BDS: DF427 (30 bits, ms of week in BDS time)
    bw.write(tow_ms & 0x3FFFFFFF, 30)

    # DF393: multiple message bit (1 = more MSM for this epoch)
    bw.write(sync_flag & 1, 1)
    # DF409: IODS (issue of data station, 3 bits)
    bw.write(0, 3)
    # DF001: reserved (7 bits)
    bw.write(0, 7)
    # DF411: clock steering indicator (2 bits, 0 = unknown)
    bw.write(0, 2)
    # DF412: external clock indicator (2 bits, 0 = unknown)
    bw.write(0, 2)
    # DF417: GNSS smoothing type (1 bit, 0 = no smoothing)
    bw.write(0, 1)
    # DF418: GNSS smoothing interval (3 bits, 0 = no smoothing)
    bw.write(0, 3)

    # Satellite mask (64 bits)
    bw.write((sat_mask >> 32) & 0xFFFFFFFF, 32)
    bw.write(sat_mask & 0xFFFFFFFF, 32)

    # Signal mask (32 bits)
    bw.write(signal_mask & 0xFFFFFFFF, 32)

    # Cell mask (n_sat * n_sig bits)
    for bit in cell_mask:
        bw.write(bit & 1, 1)


def encode_msm4(system_prefix, observations, gps_time, station_id=0,
                sync_flag=0):
    """Encode an RTCM MSM4 message for one GNSS system.

    MSM4 contains full pseudorange and carrier phase with extended resolution.

    Args:
        system_prefix: 'G' (GPS), 'E' (Galileo), or 'C' (BDS)
        observations: list of raw observation dicts from serial_reader.
            Each dict needs: sv, pr (pseudorange in m), cp (carrier phase in
            cycles), cno (C/N0 in dBHz), lock_ms (lock time in ms),
            sig_name (e.g. 'GPS-L1CA')
        gps_time: datetime of the epoch (GPS time)
        station_id: reference station ID
        sync_flag: 1 if more MSM messages follow for this epoch

    Returns:
        Complete RTCM frame bytes, or None if no observations for this system
    """
    msg_number = MSM4_MSG.get(system_prefix)
    if msg_number is None:
        return None

    # Filter observations for this system
    sys_obs = [o for o in observations if o.get('sv', '')[0:1] == system_prefix]
    if not sys_obs:
        return None

    # Compute epoch time
    if system_prefix == 'C':
        tow_ms = _bds_tow_ms(gps_time)
    elif system_prefix == 'E':
        tow_ms = _gal_tow_ms(gps_time)
    else:
        tow_ms = _gps_tow_ms(gps_time)

    # Build satellite and signal sets
    # sat_ids: 1-based satellite PRN numbers present
    # sig_ids: MSM signal IDs present
    sat_ids = sorted(set(int(o['sv'][1:]) for o in sys_obs))
    sig_names_present = sorted(set(o['sig_name'] for o in sys_obs
                                    if o.get('sig_name') in MSM_SIGNAL_ID))
    sig_ids = sorted(set(MSM_SIGNAL_ID[s] for s in sig_names_present))

    if not sat_ids or not sig_ids:
        return None

    n_sat = len(sat_ids)
    n_sig = len(sig_ids)

    # Build satellite mask (64 bits, bit 63 = satellite 1, bit 0 = satellite 64)
    sat_mask = 0
    for prn in sat_ids:
        if 1 <= prn <= 64:
            sat_mask |= (1 << (64 - prn))

    # Build signal mask (32 bits, bit 31 = signal 1, bit 0 = signal 32)
    signal_mask = 0
    for sid in sig_ids:
        if 1 <= sid <= 32:
            signal_mask |= (1 << (32 - sid))

    # Build cell mask and observation matrix
    # cell_mask[i_sat * n_sig + i_sig] = 1 if observation exists
    # Also build the data arrays indexed by (i_sat, i_sig)
    obs_lookup = {}
    for o in sys_obs:
        prn = int(o['sv'][1:])
        sig_name = o.get('sig_name')
        if sig_name not in MSM_SIGNAL_ID:
            continue
        sid = MSM_SIGNAL_ID[sig_name]
        obs_lookup[(prn, sid)] = o

    cell_mask = []
    cells = []  # ordered list of (prn, sid, obs) for present cells
    for prn in sat_ids:
        for sid in sig_ids:
            if (prn, sid) in obs_lookup:
                cell_mask.append(1)
                cells.append((prn, sid, obs_lookup[(prn, sid)]))
            else:
                cell_mask.append(0)

    n_cells = len(cells)
    if n_cells == 0:
        return None

    # ── Compute satellite data (per-satellite, MSM4) ──
    # For each satellite, compute the "rough range" from pseudorange
    # DF397: rough range integer ms (8 bits, unsigned)
    # DF398: rough range mod 1ms (10 bits, unsigned, 1/1024 ms resolution)

    sat_rough_range_ms = {}  # prn → rough range in ms
    for prn in sat_ids:
        # Use the first available observation for this satellite
        pr_m = None
        for sid in sig_ids:
            if (prn, sid) in obs_lookup:
                pr_m = obs_lookup[(prn, sid)]['pr']
                break
        if pr_m is None:
            sat_rough_range_ms[prn] = 0.0
            continue
        # Convert pseudorange to milliseconds of light travel
        range_ms = pr_m / RANGE_MS
        sat_rough_range_ms[prn] = range_ms

    # ── Encode ──
    bw = BitWriter()

    # MSM header
    _encode_msm_header(bw, msg_number, station_id, tow_ms, sync_flag,
                       sat_mask, signal_mask, cell_mask)

    # ── Satellite data block ──
    for prn in sat_ids:
        range_ms = sat_rough_range_ms[prn]
        # DF397: rough range integer ms (8 bits, 0-254, 255 = invalid)
        int_ms = int(range_ms) if range_ms > 0 else 255
        if int_ms > 254:
            int_ms = 255
        bw.write(int_ms, 8)

    for prn in sat_ids:
        range_ms = sat_rough_range_ms[prn]
        # DF398: rough range fractional ms (10 bits, 0-1023, units of 2^-10 ms)
        if range_ms > 0:
            frac_ms = range_ms - int(range_ms)
            frac_val = int(round(frac_ms * 1024)) & 0x3FF
        else:
            frac_val = 0
        bw.write(frac_val, 10)

    # ── Signal data block (MSM4: DF400 pseudorange, DF401 phase, DF402 lock, DF420 half-cycle, DF403 CNR) ──
    for prn, sid, obs in cells:
        range_ms = sat_rough_range_ms[prn]
        pr_m = obs['pr']
        pr_ms = pr_m / RANGE_MS

        # DF400: fine pseudorange (15 bits, signed, units of 2^-24 ms)
        # fine = (actual_range_ms - rough_range_ms) in units of 2^-24 ms
        if range_ms > 0:
            rough_ms = int(range_ms) + (int(round((range_ms - int(range_ms)) * 1024)) / 1024.0)
            delta_ms = pr_ms - rough_ms
            fine_pr = int(round(delta_ms * (1 << 24)))
            # Clamp to 15-bit signed range
            fine_pr = max(-(1 << 14), min((1 << 14) - 1, fine_pr))
        else:
            fine_pr = -16384  # invalid
        bw.write_signed(fine_pr, 15)

    for prn, sid, obs in cells:
        range_ms = sat_rough_range_ms[prn]
        cp = obs.get('cp')
        pr_m = obs['pr']

        # DF401: fine phase range (22 bits, signed, units of 2^-29 ms)
        if cp is not None and range_ms > 0:
            # Find signal frequency for wavelength
            sig_name = obs.get('sig_name', '')
            freq = SIGNAL_FREQ.get(sig_name, 1575.42e6)
            wavelength = C_LIGHT / freq

            # Phase in ms of light travel
            phase_ms = (cp * wavelength) / RANGE_MS

            rough_ms = int(range_ms) + (int(round((range_ms - int(range_ms)) * 1024)) / 1024.0)
            delta_ms = phase_ms - rough_ms
            fine_phase = int(round(delta_ms * (1 << 29)))
            # Clamp to 22-bit signed range
            fine_phase = max(-(1 << 21), min((1 << 21) - 1, fine_phase))
        else:
            fine_phase = -(1 << 21)  # invalid
        bw.write_signed(fine_phase, 22)

    for prn, sid, obs in cells:
        # DF402: phase range lock time indicator (4 bits)
        lock_ms = obs.get('lock_ms', 0) or 0
        # Encode lock time indicator per RTCM table 3.4-2
        if lock_ms < 24:
            lock_ind = 0
        elif lock_ms < 72:
            lock_ind = 1
        elif lock_ms < 168:
            lock_ind = 2
        elif lock_ms < 360:
            lock_ind = 3
        elif lock_ms < 744:
            lock_ind = 4
        elif lock_ms < 1512:
            lock_ind = 5
        elif lock_ms < 3048:
            lock_ind = 6
        elif lock_ms < 6120:
            lock_ind = 7
        elif lock_ms < 12264:
            lock_ind = 8
        elif lock_ms < 24552:
            lock_ind = 9
        elif lock_ms < 49128:
            lock_ind = 10
        elif lock_ms < 98280:
            lock_ind = 11
        elif lock_ms < 196584:
            lock_ind = 12
        elif lock_ms < 393192:
            lock_ind = 13
        elif lock_ms < 786408:
            lock_ind = 14
        else:
            lock_ind = 15
        bw.write(lock_ind, 4)

    for prn, sid, obs in cells:
        # DF420: half-cycle ambiguity indicator (1 bit)
        half_cyc = 1 if obs.get('half_cyc', False) else 0
        bw.write(half_cyc, 1)

    for prn, sid, obs in cells:
        # DF403: GNSS signal CNR (6 bits, 0-63 dBHz, resolution 1 dBHz)
        cno = obs.get('cno', 0) or 0
        cno_val = min(63, max(0, int(round(cno))))
        bw.write(cno_val, 6)

    return rtcm_frame(bw.to_bytes())


def encode_epoch(raw_observations, gps_time, station_id=0):
    """Encode a full epoch of observations as MSM4 messages.

    Returns a list of RTCM frame bytes (one per system present).
    Systems are ordered GPS, Galileo, BDS with sync_flag=1 on all but the last.

    Args:
        raw_observations: list of raw observation dicts with keys:
            sv (str), pr (float, meters), cp (float, cycles or None),
            cno (float, dBHz), lock_ms (float), sig_name (str),
            half_cyc (bool)
        gps_time: datetime of the epoch
        station_id: reference station ID
    """
    frames = []
    systems_present = sorted(set(o['sv'][0:1] for o in raw_observations
                                  if o.get('sv', '')[0:1] in MSM4_MSG))

    for i, sys_prefix in enumerate(systems_present):
        is_last = (i == len(systems_present) - 1)
        sync_flag = 0 if is_last else 1
        frame = encode_msm4(sys_prefix, raw_observations, gps_time,
                            station_id=station_id, sync_flag=sync_flag)
        if frame is not None:
            frames.append(frame)

    return frames


# ── Self-test ─────────────────────────────────────────────────────────────── #

def _self_test():
    """Verify CRC-24Q and basic encoding round-trip."""
    # Test CRC on known data
    test_data = b'\xD3\x00\x00'
    crc = crc24q(test_data)
    assert isinstance(crc, int) and 0 <= crc < (1 << 24), f"CRC out of range: {crc}"

    # Test 1005 encoding
    ecef = [882685.0, -4924395.0, 3944003.0]  # Approximate position
    frame = encode_1005(ecef)
    assert frame[0] == 0xD3, "Frame must start with preamble 0xD3"
    # Verify CRC
    length = ((frame[1] & 0x03) << 8) | frame[2]
    expected_crc = crc24q(frame[:3 + length])
    actual_crc = (frame[-3] << 16) | (frame[-2] << 8) | frame[-1]
    assert expected_crc == actual_crc, f"1005 CRC mismatch: {expected_crc} != {actual_crc}"
    # Verify message type in payload
    msg_type = (frame[3] << 4) | (frame[4] >> 4)
    assert msg_type == 1005, f"Expected msg type 1005, got {msg_type}"

    # Test MSM4 encoding with synthetic observations
    test_obs = [
        {'sv': 'G05', 'pr': 22000000.0, 'cp': 115000000.0,
         'cno': 42.0, 'lock_ms': 5000, 'sig_name': 'GPS-L1CA', 'half_cyc': True},
        {'sv': 'G05', 'pr': 22000100.0, 'cp': 88000000.0,
         'cno': 38.0, 'lock_ms': 5000, 'sig_name': 'GPS-L5Q', 'half_cyc': True},
        {'sv': 'G13', 'pr': 24000000.0, 'cp': 126000000.0,
         'cno': 35.0, 'lock_ms': 3000, 'sig_name': 'GPS-L1CA', 'half_cyc': True},
    ]
    test_time = datetime(2026, 3, 20, 12, 0, 0, tzinfo=timezone.utc)
    frames = encode_epoch(test_obs, test_time)
    assert len(frames) == 1, f"Expected 1 frame (GPS only), got {len(frames)}"
    frame = frames[0]
    assert frame[0] == 0xD3
    length = ((frame[1] & 0x03) << 8) | frame[2]
    expected_crc = crc24q(frame[:3 + length])
    actual_crc = (frame[-3] << 16) | (frame[-2] << 8) | frame[-1]
    assert expected_crc == actual_crc, f"MSM4 CRC mismatch"
    msg_type = (frame[3] << 4) | (frame[4] >> 4)
    assert msg_type == 1074, f"Expected msg type 1074, got {msg_type}"

    # Test multi-system encoding
    test_obs_multi = test_obs + [
        {'sv': 'E02', 'pr': 23000000.0, 'cp': 121000000.0,
         'cno': 40.0, 'lock_ms': 8000, 'sig_name': 'GAL-E1C', 'half_cyc': True},
    ]
    frames = encode_epoch(test_obs_multi, test_time)
    assert len(frames) == 2, f"Expected 2 frames (GPS+GAL), got {len(frames)}"

    print("rtcm_encoder: all self-tests passed")


if __name__ == "__main__":
    _self_test()

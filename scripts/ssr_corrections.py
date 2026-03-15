#!/usr/bin/env python3
"""
ssr_corrections.py — Real-time SSR correction state manager.

Maintains a live set of orbit, clock, and bias corrections received from an
SSR stream (IGS SSR via NTRIP, decoded by pyrtcm). Combines with broadcast
ephemeris to produce precise satellite positions and clocks.

Provides the same interface as SP3+CLKFile from the file-based M3 pipeline,
so FixedPosFilter can use either transparently.

Usage:
    from broadcast_eph import BroadcastEphemeris
    from ssr_corrections import SSRState, RealtimeCorrections

    beph = BroadcastEphemeris()
    ssr = SSRState()
    corrections = RealtimeCorrections(beph, ssr)

    # Feed RTCM messages from NTRIP
    beph.update_from_rtcm(msg)   # 1019/1042/1045/1046
    ssr.update_from_rtcm(msg)    # 4076_02x/06x/10x or 1057-1068

    # Query (same interface as SP3)
    pos, clk = corrections.sat_position('G01', t)
    clk_s = corrections.sat_clock('G01', t)
"""

import logging
import math
from collections import defaultdict
from datetime import datetime, timezone, timedelta

import numpy as np

log = logging.getLogger(__name__)

C = 299792458.0
MAX_ORBIT_AGE = 120.0     # seconds — discard orbit corrections older than this
MAX_CLOCK_AGE = 30.0      # seconds — discard clock corrections older than this
MAX_BIAS_AGE = 300.0      # seconds — biases change slowly


# ── IGS SSR message type → (system, subtype) mapping ─────────────────────── #
# IGS SSR uses message 4076 with subtypes grouped by GNSS system:
#   GPS: 021-027, GLONASS: 041-047, Galileo: 061-067, BDS: 101-107
_IGS_SSR_SYSTEMS = {
    '021': ('G', 'orbit'),   '022': ('G', 'clock'),   '023': ('G', 'combined'),
    '024': ('G', 'hr_clock'), '025': ('G', 'code_bias'), '026': ('G', 'phase_bias'),
    '061': ('E', 'orbit'),   '062': ('E', 'clock'),   '063': ('E', 'combined'),
    '064': ('E', 'hr_clock'), '065': ('E', 'code_bias'), '066': ('E', 'phase_bias'),
    '101': ('C', 'orbit'),   '102': ('C', 'clock'),   '103': ('C', 'combined'),
    '104': ('C', 'hr_clock'), '105': ('C', 'code_bias'), '106': ('C', 'phase_bias'),
}

# Standard RTCM SSR message types (non-IGS)
_RTCM_SSR = {
    '1057': ('G', 'orbit'),   '1058': ('G', 'clock'),
    '1059': ('G', 'code_bias'), '1060': ('G', 'combined'),
    '1062': ('G', 'hr_clock'), '1265': ('G', 'phase_bias'),
    '1063': ('R', 'orbit'),   '1064': ('R', 'clock'),
    '1065': ('R', 'code_bias'), '1066': ('R', 'combined'),
    '1067': ('R', 'phase_bias'),
    '1240': ('E', 'orbit'),   '1241': ('E', 'clock'),
    '1242': ('E', 'code_bias'), '1243': ('E', 'combined'),
    '1245': ('E', 'hr_clock'), '1267': ('E', 'phase_bias'),
    '1258': ('C', 'orbit'),   '1259': ('C', 'clock'),
    '1260': ('C', 'code_bias'), '1261': ('C', 'combined'),
    '1263': ('C', 'hr_clock'), '1270': ('C', 'phase_bias'),
}

# SSR signal tracking mode ID → RINEX observation code
# This mapping covers both IGS SSR and standard RTCM SSR signal IDs.
# Standard RTCM SSR uses the same GNSS Signal and Tracking Mode Indicator
# table as defined in RTCM 3.x (Table 3.5-91 for GPS, etc.)
_SSR_SIGNAL_MAP = {
    # GPS (RTCM SSR signal IDs)
    ('G', 0): 'C1C',   # L1 C/A
    ('G', 2): 'C1P',   # L1 P
    ('G', 5): 'C1W',   # L1 Z-tracking (P(Y))
    ('G', 7): 'C2C',   # L2 C/A
    ('G', 8): 'C2P',   # L2 P
    ('G', 9): 'C2W',   # L2 Z-tracking
    ('G', 11): 'C2W',  # L2 Z-tracking (alt)
    ('G', 14): 'C5Q',  # L5 Q
    ('G', 15): 'C5I',  # L5 I
    ('G', 16): 'L1C',  # L1 C/A phase
    ('G', 19): 'C1L',  # L1C (data+pilot)
    ('G', 21): 'L1W',  # L1 P(Y) phase
    ('G', 27): 'L2W',  # L2 P(Y) phase
    ('G', 30): 'L5Q',  # L5 Q phase
    ('G', 31): 'L5I',  # L5 I phase
    # GLONASS
    ('R', 0): 'C1C',   # G1 C/A
    ('R', 2): 'C1P',   # G1 P
    ('R', 3): 'C2C',   # G2 C/A
    ('R', 5): 'C2P',   # G2 P
    # Galileo
    ('E', 0): 'C1C',   # E1 C
    ('E', 1): 'C1B',   # E1 B
    ('E', 5): 'C5Q',   # E5a Q
    ('E', 6): 'C5I',   # E5a I
    ('E', 8): 'C7Q',   # E5b Q
    ('E', 9): 'C7I',   # E5b I
    ('E', 16): 'L1C',  # E1 C phase
    ('E', 17): 'L1B',  # E1 B phase
    ('E', 21): 'L5Q',  # E5a Q phase
    ('E', 22): 'L5I',  # E5a I phase
    # BDS
    ('C', 0): 'C2I',   # B1I
    ('C', 5): 'C5I',   # B2a I
    ('C', 9): 'C7I',   # B2b I
    ('C', 16): 'L2I',  # B1I phase
    ('C', 21): 'L5I',  # B2a I phase
}


class OrbitCorrection:
    """SSR orbit correction for one satellite at one epoch."""
    __slots__ = ('iod', 'epoch_s', 'radial', 'along', 'cross',
                 'dot_radial', 'dot_along', 'dot_cross', 'rx_time')

    def __init__(self, iod, epoch_s, radial, along, cross,
                 dot_radial=0.0, dot_along=0.0, dot_cross=0.0):
        self.iod = iod
        self.epoch_s = epoch_s
        self.radial = radial       # meters
        self.along = along         # meters
        self.cross = cross         # meters
        self.dot_radial = dot_radial
        self.dot_along = dot_along
        self.dot_cross = dot_cross
        self.rx_time = datetime.now(timezone.utc)


class ClockCorrection:
    """SSR clock correction for one satellite at one epoch."""
    __slots__ = ('epoch_s', 'c0', 'c1', 'c2', 'rx_time')

    def __init__(self, epoch_s, c0, c1=0.0, c2=0.0):
        self.epoch_s = epoch_s
        self.c0 = c0    # meters
        self.c1 = c1    # meters/s
        self.c2 = c2    # meters/s²
        self.rx_time = datetime.now(timezone.utc)


class BiasCorrection:
    """SSR code or phase bias for one satellite, one signal."""
    __slots__ = ('signal_code', 'bias_m', 'rx_time', 'is_phase',
                 'integer_indicator', 'wl_indicator', 'disc_counter')

    def __init__(self, signal_code, bias_m, is_phase=False,
                 integer_indicator=0, wl_indicator=0, disc_counter=0):
        self.signal_code = signal_code
        self.bias_m = bias_m
        self.is_phase = is_phase
        self.integer_indicator = integer_indicator
        self.wl_indicator = wl_indicator
        self.disc_counter = disc_counter
        self.rx_time = datetime.now(timezone.utc)


class SSRState:
    """Maintains live SSR corrections from NTRIP stream.

    Stores the most recent orbit, clock, and bias corrections per satellite.
    Automatically discards stale corrections.
    """

    def __init__(self):
        self._orbit = {}     # {prn: OrbitCorrection}
        self._clock = {}     # {prn: ClockCorrection}
        self._code_bias = defaultdict(dict)   # {prn: {signal_code: BiasCorrection}}
        self._phase_bias = defaultdict(dict)  # {prn: {signal_code: BiasCorrection}}
        self._update_counts = defaultdict(int)
        self._iod_ssr = None

    @property
    def n_orbit(self):
        return len(self._orbit)

    @property
    def n_clock(self):
        return len(self._clock)

    def update_from_rtcm(self, msg):
        """Ingest a decoded SSR RTCM message.

        Handles both IGS SSR (4076_*) and standard RTCM SSR (1057-1068, 1240-1263).
        Returns the correction type ('orbit', 'clock', 'code_bias', 'phase_bias')
        or None if not an SSR message.
        """
        identity = str(getattr(msg, 'identity', ''))

        # Determine system and subtype
        sys_prefix = None
        subtype = None

        if identity.startswith('4076_'):
            sub = identity.split('_')[1]
            entry = _IGS_SSR_SYSTEMS.get(sub)
            if entry:
                sys_prefix, subtype = entry
        elif identity in _RTCM_SSR:
            sys_prefix, subtype = _RTCM_SSR[identity]
        else:
            return None

        # Get SSR epoch and IOD
        epoch_s = getattr(msg, 'IDF003', None) or getattr(msg, 'DF385', 0)
        iod_ssr = getattr(msg, 'IDF007', None) or getattr(msg, 'DF413', 0)
        self._iod_ssr = iod_ssr

        n_sats = getattr(msg, 'IDF010', None) or getattr(msg, 'DF387', 0)

        if subtype == 'orbit':
            self._parse_orbit(msg, sys_prefix, epoch_s, n_sats)
        elif subtype == 'clock':
            self._parse_clock(msg, sys_prefix, epoch_s, n_sats)
        elif subtype == 'combined':
            self._parse_orbit(msg, sys_prefix, epoch_s, n_sats)
            self._parse_clock(msg, sys_prefix, epoch_s, n_sats)
        elif subtype == 'hr_clock':
            self._parse_clock(msg, sys_prefix, epoch_s, n_sats)
        elif subtype == 'code_bias':
            self._parse_code_bias(msg, sys_prefix, n_sats)
        elif subtype == 'phase_bias':
            self._parse_phase_bias(msg, sys_prefix, n_sats)

        self._update_counts[subtype] += 1
        return subtype

    @staticmethod
    def _get_sat_id(msg, i):
        """Get satellite ID from message, trying both IGS and standard RTCM fields."""
        for field in (f'IDF011_{i:02d}', f'DF068_{i:02d}', f'DF384_{i:02d}'):
            val = getattr(msg, field, None)
            if val is not None:
                return int(val)
        return None

    @staticmethod
    def _get_iod(msg, i):
        """Get IOD from message."""
        for field in (f'IDF012_{i:02d}', f'DF071_{i:02d}', f'DF392_{i:02d}'):
            val = getattr(msg, field, None)
            if val is not None:
                return int(val)
        return 0

    def _parse_orbit(self, msg, sys_prefix, epoch_s, n_sats):
        """Extract per-satellite orbit corrections from an SSR message.

        Supports both IGS SSR (IDF fields) and standard RTCM SSR (DF fields).
        Standard RTCM SSR orbit fields (pyrtcm):
          DF365 = radial (mm), DF366 = along-track (mm), DF367 = cross-track (mm)
          DF368/369/370 = velocity corrections (mm/s)
        IGS SSR fields: IDF013-018
        """
        for i in range(1, n_sats + 1):
            sat_id = self._get_sat_id(msg, i)
            if sat_id is None:
                continue
            prn = f"{sys_prefix}{sat_id:02d}"

            iod = self._get_iod(msg, i)

            # Try IGS SSR fields first, then standard RTCM SSR
            radial = getattr(msg, f'IDF013_{i:02d}', None)
            if radial is None:
                # Standard RTCM SSR: DF365-370 are in mm, convert to meters
                radial_mm = getattr(msg, f'DF365_{i:02d}', None)
                if radial_mm is not None:
                    radial = radial_mm / 1000.0
                    along = getattr(msg, f'DF366_{i:02d}', 0.0) / 1000.0
                    cross = getattr(msg, f'DF367_{i:02d}', 0.0) / 1000.0
                    dot_r = getattr(msg, f'DF368_{i:02d}', 0.0) / 1000.0
                    dot_a = getattr(msg, f'DF369_{i:02d}', 0.0) / 1000.0
                    dot_c = getattr(msg, f'DF370_{i:02d}', 0.0) / 1000.0
                else:
                    continue
            else:
                along = getattr(msg, f'IDF014_{i:02d}', 0.0)
                cross = getattr(msg, f'IDF015_{i:02d}', 0.0)
                dot_r = getattr(msg, f'IDF016_{i:02d}', 0.0)
                dot_a = getattr(msg, f'IDF017_{i:02d}', 0.0)
                dot_c = getattr(msg, f'IDF018_{i:02d}', 0.0)

            self._orbit[prn] = OrbitCorrection(
                iod=iod, epoch_s=epoch_s,
                radial=radial, along=along, cross=cross,
                dot_radial=dot_r, dot_along=dot_a, dot_cross=dot_c,
            )

    def _parse_clock(self, msg, sys_prefix, epoch_s, n_sats):
        """Extract per-satellite clock corrections from an SSR message.

        Standard RTCM SSR clock fields (pyrtcm):
          DF376 = C0 (mm), DF377 = C1 (mm/s), DF378 = C2 (mm/s²)
        IGS SSR fields: IDF019-021
        """
        for i in range(1, n_sats + 1):
            sat_id = self._get_sat_id(msg, i)
            if sat_id is None:
                continue
            prn = f"{sys_prefix}{sat_id:02d}"

            # Try IGS SSR fields first, then standard RTCM SSR
            c0 = getattr(msg, f'IDF019_{i:02d}', None)
            if c0 is None:
                c0_mm = getattr(msg, f'DF376_{i:02d}', None)
                if c0_mm is not None:
                    c0 = c0_mm / 1000.0
                    c1 = getattr(msg, f'DF377_{i:02d}', 0.0) / 1000.0
                    c2 = getattr(msg, f'DF378_{i:02d}', 0.0) / 1000.0
                else:
                    continue
            else:
                c1 = getattr(msg, f'IDF020_{i:02d}', 0.0)
                c2 = getattr(msg, f'IDF021_{i:02d}', 0.0)

            self._clock[prn] = ClockCorrection(
                epoch_s=epoch_s, c0=c0, c1=c1, c2=c2,
            )

    def _parse_code_bias(self, msg, sys_prefix, n_sats):
        """Extract per-satellite code bias corrections.

        Standard RTCM SSR code bias fields (pyrtcm):
          DF379 = num biases, DF380 = signal ID, DF383 = bias (m)
        IGS SSR fields: IDF023/024/025
        """
        for i in range(1, n_sats + 1):
            sat_id = self._get_sat_id(msg, i)
            if sat_id is None:
                continue
            prn = f"{sys_prefix}{sat_id:02d}"

            # Try IGS then standard
            n_biases = getattr(msg, f'IDF023_{i:02d}', None)
            if n_biases is None:
                n_biases = getattr(msg, f'DF379_{i:02d}', 0)
            n_biases = int(n_biases)

            for j in range(1, n_biases + 1):
                sig_id = getattr(msg, f'IDF024_{i:02d}_{j:02d}', None)
                if sig_id is None:
                    sig_id = getattr(msg, f'DF380_{i:02d}_{j:02d}', None)
                bias_m = getattr(msg, f'IDF025_{i:02d}_{j:02d}', None)
                if bias_m is None:
                    bias_m = getattr(msg, f'DF383_{i:02d}_{j:02d}', None)
                if sig_id is None or bias_m is None:
                    continue
                rinex_code = _SSR_SIGNAL_MAP.get((sys_prefix, int(sig_id)))
                if rinex_code is None:
                    continue
                self._code_bias[prn][rinex_code] = BiasCorrection(
                    signal_code=rinex_code, bias_m=float(bias_m), is_phase=False,
                )

    def _parse_phase_bias(self, msg, sys_prefix, n_sats):
        """Extract per-satellite phase bias corrections.

        Standard RTCM SSR uses 1265/1267/1270 for GPS/GAL/BDS phase biases.
        IGS SSR uses IDF fields.
        """
        for i in range(1, n_sats + 1):
            sat_id = self._get_sat_id(msg, i)
            if sat_id is None:
                continue
            prn = f"{sys_prefix}{sat_id:02d}"

            n_biases = getattr(msg, f'IDF023_{i:02d}', None)
            if n_biases is None:
                n_biases = getattr(msg, f'DF379_{i:02d}', 0)
            n_biases = int(n_biases)

            for j in range(1, n_biases + 1):
                sig_id = getattr(msg, f'IDF024_{i:02d}_{j:02d}', None)
                if sig_id is None:
                    sig_id = getattr(msg, f'DF380_{i:02d}_{j:02d}', None)
                bias_m = getattr(msg, f'IDF028_{i:02d}_{j:02d}', None)
                if bias_m is None:
                    bias_m = getattr(msg, f'DF383_{i:02d}_{j:02d}', None)
                if sig_id is None or bias_m is None:
                    continue
                rinex_code = _SSR_SIGNAL_MAP.get((sys_prefix, int(sig_id)))
                if rinex_code is None:
                    continue
                int_ind = getattr(msg, f'IDF029_{i:02d}_{j:02d}', 0)
                wl_ind = getattr(msg, f'IDF030_{i:02d}_{j:02d}', 0)
                disc = getattr(msg, f'IDF031_{i:02d}_{j:02d}', 0)
                self._phase_bias[prn][rinex_code] = BiasCorrection(
                    signal_code=rinex_code, bias_m=float(bias_m), is_phase=True,
                    integer_indicator=int_ind, wl_indicator=wl_ind,
                    disc_counter=disc,
                )

    def get_orbit(self, prn):
        """Return OrbitCorrection for a satellite, or None if stale/missing."""
        oc = self._orbit.get(prn)
        if oc is None:
            return None
        age = (datetime.now(timezone.utc) - oc.rx_time).total_seconds()
        if age > MAX_ORBIT_AGE:
            return None
        return oc

    def get_clock(self, prn):
        """Return ClockCorrection for a satellite, or None if stale/missing."""
        cc = self._clock.get(prn)
        if cc is None:
            return None
        age = (datetime.now(timezone.utc) - cc.rx_time).total_seconds()
        if age > MAX_CLOCK_AGE:
            return None
        return cc

    def get_code_bias(self, prn, signal_code):
        """Return code bias in meters for (prn, signal_code), or None."""
        bc = self._code_bias.get(prn, {}).get(signal_code)
        if bc is None:
            return None
        age = (datetime.now(timezone.utc) - bc.rx_time).total_seconds()
        if age > MAX_BIAS_AGE:
            return None
        return bc.bias_m

    def get_phase_bias(self, prn, signal_code):
        """Return phase bias in meters for (prn, signal_code), or None."""
        pb = self._phase_bias.get(prn, {}).get(signal_code)
        if pb is None:
            return None
        age = (datetime.now(timezone.utc) - pb.rx_time).total_seconds()
        if age > MAX_BIAS_AGE:
            return None
        return pb.bias_m

    def summary(self):
        """Return a string summarizing current SSR state."""
        return (f"SSR: {self.n_orbit} orbit, {self.n_clock} clock, "
                f"{sum(len(v) for v in self._code_bias.values())} code bias, "
                f"{sum(len(v) for v in self._phase_bias.values())} phase bias")


class RealtimeCorrections:
    """Unified interface combining BroadcastEphemeris + SSR corrections.

    Provides sat_position() and sat_clock() with the same signatures as
    SP3 and CLKFile, so FixedPosFilter works unchanged.
    """

    def __init__(self, broadcast_eph, ssr_state):
        """
        Args:
            broadcast_eph: BroadcastEphemeris instance
            ssr_state: SSRState instance
        """
        self.beph = broadcast_eph
        self.ssr = ssr_state
        self._ssr_applied = 0
        self._broadcast_only = 0

    def sat_position(self, prn, t):
        """Compute precise satellite position and clock.

        If SSR orbit corrections are available and IOD matches the current
        broadcast ephemeris, applies them. Otherwise falls back to broadcast-only.

        Returns:
            (np.array([x, y, z]), clock_seconds) or (None, None)
        """
        # Get broadcast position
        bcast_pos, bcast_clk = self.beph.sat_position(prn, t)
        if bcast_pos is None:
            return None, None

        # Try SSR orbit correction
        oc = self.ssr.get_orbit(prn)
        if oc is not None:
            # Check IOD consistency
            bcast_iod = self.beph.get_iod(prn)
            if bcast_iod is not None and oc.iod == bcast_iod:
                # Apply orbit correction in radial/along-track/cross-track frame
                corrected_pos = self._apply_orbit_correction(prn, t, bcast_pos, oc)
                if corrected_pos is not None:
                    bcast_pos = corrected_pos
                    self._ssr_applied += 1
            else:
                log.debug(f"{prn}: IOD mismatch (bcast={bcast_iod}, ssr={oc.iod})")
                self._broadcast_only += 1
        else:
            self._broadcast_only += 1

        # Apply SSR clock correction
        cc = self.ssr.get_clock(prn)
        if cc is not None:
            # SSR clock correction is in meters, convert to seconds
            # Convention: corrected = broadcast - delta_clock / C
            bcast_clk = bcast_clk - cc.c0 / C
        # else: use broadcast clock as-is

        return bcast_pos, bcast_clk

    def sat_clock(self, prn, t):
        """Return precise satellite clock in seconds (for CLKFile interface).

        This is separate from sat_position() for cases where the caller
        already has the position and just needs the clock.
        """
        _, clk = self.sat_position(prn, t)
        return clk

    def _apply_orbit_correction(self, prn, t, bcast_pos, oc):
        """Transform SSR orbit correction from RAC frame to ECEF delta.

        The SSR correction is given in a satellite-centered frame:
        - Radial: along the satellite-to-Earth-center direction
        - Along-track: along the satellite velocity vector
        - Cross-track: completes the right-handed system

        To convert to ECEF, we need the satellite velocity vector.
        """
        vel = self.beph.sat_velocity(prn, t)
        if vel is None:
            return None

        # Unit vectors for RAC frame
        r_hat = -bcast_pos / np.linalg.norm(bcast_pos)  # Radial (toward Earth center)
        c_hat = np.cross(r_hat, vel)
        c_norm = np.linalg.norm(c_hat)
        if c_norm < 1e-10:
            return None
        c_hat = c_hat / c_norm
        a_hat = np.cross(c_hat, r_hat)  # Along-track

        # Apply correction: positive radial = satellite moved away from Earth
        # SSR convention: subtract the correction from broadcast
        delta_ecef = (oc.radial * r_hat +
                      oc.along * a_hat +
                      oc.cross * c_hat)

        return bcast_pos - delta_ecef

    def get_osb(self, prn, signal_code):
        """Return code bias correction in meters (for OSBParser interface).

        Falls back to zero if SSR code bias is not available.
        """
        bias = self.ssr.get_code_bias(prn, signal_code)
        return bias if bias is not None else None

    def get_phase_bias(self, prn, signal_code):
        """Return phase bias correction in meters."""
        return self.ssr.get_phase_bias(prn, signal_code)

    def summary(self):
        """Return a string summarizing correction state."""
        return (f"{self.beph.summary()} | {self.ssr.summary()} | "
                f"Applied: {self._ssr_applied} SSR, {self._broadcast_only} broadcast-only")

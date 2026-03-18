#!/usr/bin/env python3
"""
broadcast_eph.py — Broadcast ephemeris computation for GPS, Galileo, and BeiDou.

Decodes RTCM3 ephemeris messages (1019/1042/1045/1046) via pyrtcm and computes
satellite ECEF positions + clock corrections using the standard Keplerian model.

Exposes the same interface as SP3 from solve_pseudorange.py:
    eph.sat_position(sv, t) → (np.array([x,y,z]), clock_seconds)

Usage:
    from broadcast_eph import BroadcastEphemeris
    eph = BroadcastEphemeris()
    eph.update_from_rtcm(decoded_msg)        # Feed RTCM 1019/1042/1045/1046
    pos, clk = eph.sat_position('G01', t)    # Query like SP3
"""

import math
from datetime import datetime, timezone, timedelta

import numpy as np

# ── Constants ──────────────────────────────────────────────────────────────── #
C = 299792458.0                   # Speed of light (m/s)
GM_GPS = 3.986005e14              # WGS84 gravitational parameter (m³/s²)
GM_GAL = 3.986004418e14           # Galileo gravitational parameter
GM_BDS = 3.986004418e14           # BDS gravitational parameter (GCJ)
OMEGA_E = 7.2921151467e-5         # Earth rotation rate (rad/s)
F_REL = -4.442807633e-10          # Relativistic correction constant (s/m^½)
GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
BDS_EPOCH = datetime(2006, 1, 1, tzinfo=timezone.utc)
GAL_EPOCH = datetime(1999, 8, 22, tzinfo=timezone.utc)  # GST epoch = GPS epoch - 1024 weeks

SECONDS_PER_WEEK = 604800
HALF_WEEK = 302400
BDT_GPST_OFFSET = 14.0  # BDT = GPST - 14 seconds (constant, no new leap seconds since BDS epoch)


def _check_week_crossover(dt):
    """Correct for beginning/end of week crossover."""
    if dt > HALF_WEEK:
        return dt - SECONDS_PER_WEEK
    elif dt < -HALF_WEEK:
        return dt + SECONDS_PER_WEEK
    return dt


def _kepler_ecef(eph, tk, gm):
    """Core Keplerian model: orbital elements → ECEF position.

    This implements IS-GPS-200 Table 20-IV (also valid for GAL/BDS with
    appropriate GM and reference frame constants).

    Args:
        eph: dict with Keplerian orbital elements
        tk: time since ephemeris reference epoch (seconds)
        gm: gravitational parameter for the GNSS system

    Returns:
        np.array([x, y, z]) in ECEF meters
    """
    a = eph['sqrt_a'] ** 2
    n0 = math.sqrt(gm / a ** 3)
    n = n0 + eph['delta_n']

    # Mean anomaly
    Mk = eph['M0'] + n * tk

    # Solve Kepler's equation iteratively
    Ek = Mk
    for _ in range(15):
        Ek_new = Mk + eph['e'] * math.sin(Ek)
        if abs(Ek_new - Ek) < 1e-14:
            break
        Ek = Ek_new
    Ek = Ek_new

    # True anomaly
    denom = 1.0 - eph['e'] * math.cos(Ek)
    sin_vk = math.sqrt(1.0 - eph['e'] ** 2) * math.sin(Ek) / denom
    cos_vk = (math.cos(Ek) - eph['e']) / denom
    vk = math.atan2(sin_vk, cos_vk)

    # Argument of latitude
    phik = vk + eph['omega']

    # Second harmonic perturbations
    sin2phi = math.sin(2.0 * phik)
    cos2phi = math.cos(2.0 * phik)
    delta_uk = eph['Cus'] * sin2phi + eph['Cuc'] * cos2phi
    delta_rk = eph['Crs'] * sin2phi + eph['Crc'] * cos2phi
    delta_ik = eph['Cis'] * sin2phi + eph['Cic'] * cos2phi

    uk = phik + delta_uk
    rk = a * denom + delta_rk
    ik = eph['i0'] + delta_ik + eph['i_dot'] * tk

    # Position in orbital plane
    xp = rk * math.cos(uk)
    yp = rk * math.sin(uk)

    # Corrected longitude of ascending node
    omega_k = (eph['omega0']
               + (eph['omega_dot'] - OMEGA_E) * tk
               - OMEGA_E * eph['toe'])

    # ECEF coordinates
    cos_ok = math.cos(omega_k)
    sin_ok = math.sin(omega_k)
    cos_ik = math.cos(ik)
    x = xp * cos_ok - yp * cos_ik * sin_ok
    y = xp * sin_ok + yp * cos_ik * cos_ok
    z = yp * math.sin(ik)

    return np.array([x, y, z]), Ek


def _sat_clock(eph, dt_clk, Ek):
    """Compute satellite clock correction from broadcast ephemeris.

    Args:
        eph: dict with af0, af1, af2, e, sqrt_a, tgd
        dt_clk: time since clock reference epoch (seconds)
        Ek: eccentric anomaly (for relativistic correction)

    Returns:
        clock correction in seconds (positive = sat ahead of system time)
    """
    # Relativistic correction
    delta_tr = F_REL * eph['e'] * eph['sqrt_a'] * math.sin(Ek)
    # Polynomial clock model + relativistic - group delay
    return (eph['af0'] + eph['af1'] * dt_clk + eph['af2'] * dt_clk ** 2
            + delta_tr - eph.get('tgd', 0.0))


# ── RTCM field → ephemeris dict mapping ────────────────────────────────────── #

# GPS (RTCM 1019) — DF field names from pyrtcm
_GPS_MAP = {
    'sat_id':   'DF009',
    'week':     'DF076',
    'i_dot':    'DF079',
    'iode':     'DF071',
    'toc':      'DF081',
    'af2':      'DF082',
    'af1':      'DF083',
    'af0':      'DF084',
    'Crs':      'DF086',
    'delta_n':  'DF087',
    'M0':       'DF088',
    'Cuc':      'DF089',
    'e':        'DF090',
    'Cus':      'DF091',
    'sqrt_a':   'DF092',
    'toe':      'DF093',
    'Cic':      'DF094',
    'omega0':   'DF095',
    'Cis':      'DF096',
    'i0':       'DF097',
    'Crc':      'DF098',
    'omega':    'DF099',
    'omega_dot': 'DF100',
    'tgd':      'DF101',
    'health':   'DF102',
}

# Galileo I/NAV (RTCM 1046) — DF field names from pyrtcm
_GAL_MAP = {
    'sat_id':   'DF252',
    'week':     'DF289',
    'iod':      'DF290',
    'i_dot':    'DF292',
    'toc':      'DF293',
    'af2':      'DF294',
    'af1':      'DF295',
    'af0':      'DF296',
    'Crs':      'DF297',
    'delta_n':  'DF298',
    'M0':       'DF299',
    'Cuc':      'DF300',
    'e':        'DF301',
    'Cus':      'DF302',
    'sqrt_a':   'DF303',
    'toe':      'DF304',
    'Cic':      'DF305',
    'omega0':   'DF306',
    'Cis':      'DF307',
    'i0':       'DF308',
    'Crc':      'DF309',
    'omega':    'DF310',
    'omega_dot': 'DF311',
    'tgd':      'DF312',    # BGD E1/E5a
    'health':   'DF287',
}

# Galileo F/NAV (RTCM 1045) — same structure, different message
_GAL_FNAV_MAP = dict(_GAL_MAP)  # Same DF numbers

# BDS (RTCM 1042) — DF field names from pyrtcm
_BDS_MAP = {
    'sat_id':   'DF488',
    'week':     'DF489',
    'i_dot':    'DF491',
    'iode':     'DF492',
    'toc':      'DF493',
    'af2':      'DF494',
    'af1':      'DF495',
    'af0':      'DF496',
    'Crs':      'DF498',
    'delta_n':  'DF499',
    'M0':       'DF500',
    'Cuc':      'DF501',
    'e':        'DF502',
    'Cus':      'DF503',
    'sqrt_a':   'DF504',
    'toe':      'DF505',
    'Cic':      'DF506',
    'omega0':   'DF507',
    'Cis':      'DF508',
    'i0':       'DF509',
    'Crc':      'DF510',
    'omega':    'DF511',
    'omega_dot': 'DF512',
    'tgd':      'DF513',    # TGD1
    'health':   'DF515',
}


def _extract_eph(msg, field_map):
    """Extract ephemeris parameters from a decoded pyrtcm message."""
    eph = {}
    for key, df_name in field_map.items():
        val = getattr(msg, df_name, None)
        if val is not None:
            eph[key] = val
    return eph


# ── Main class ──────────────────────────────────────────────────────────────── #

class BroadcastEphemeris:
    """Manages broadcast ephemeris sets and computes satellite positions.

    Stores the most recent ephemeris per satellite (keyed by PRN string like
    'G01', 'E05', 'C19'). Provides sat_position() with the same signature as
    the SP3 class from solve_pseudorange.py.
    """

    def __init__(self):
        # {prn_str: eph_dict}
        self._ephs = {}
        self._update_count = 0

    @property
    def n_satellites(self):
        return len(self._ephs)

    @property
    def satellites(self):
        return sorted(self._ephs.keys())

    def update_from_rtcm(self, msg):
        """Ingest a decoded pyrtcm RTCMMessage (1019/1042/1045/1046).

        Returns the PRN string if accepted, None otherwise.
        """
        identity = getattr(msg, 'identity', '')
        msg_type = str(identity)

        if msg_type == '1019':
            eph = _extract_eph(msg, _GPS_MAP)
            if 'sat_id' not in eph or 'sqrt_a' not in eph:
                return None
            prn = f"G{int(eph['sat_id']):02d}"
            eph['system'] = 'G'
            eph['gm'] = GM_GPS
        elif msg_type in ('1045', '1046'):
            field_map = _GAL_FNAV_MAP if msg_type == '1045' else _GAL_MAP
            eph = _extract_eph(msg, field_map)
            if 'sat_id' not in eph or 'sqrt_a' not in eph:
                return None
            prn = f"E{int(eph['sat_id']):02d}"
            eph['system'] = 'E'
            eph['gm'] = GM_GAL
        elif msg_type == '1042':
            eph = _extract_eph(msg, _BDS_MAP)
            if 'sat_id' not in eph or 'sqrt_a' not in eph:
                return None
            prn = f"C{int(eph['sat_id']):02d}"
            eph['system'] = 'C'
            eph['gm'] = GM_BDS
        else:
            return None

        # pyrtcm returns angular quantities in semi-circles; convert to radians
        _ANGULAR_KEYS = ('M0', 'delta_n', 'omega', 'omega0', 'i0',
                         'i_dot', 'omega_dot')
        for key in _ANGULAR_KEYS:
            if key in eph:
                eph[key] = eph[key] * math.pi

        self._ephs[prn] = eph
        self._update_count += 1
        return prn

    def _gps_seconds_of_week(self, t):
        """Convert datetime to GPS seconds-of-week."""
        gps_delta = (t - GPS_EPOCH).total_seconds()
        week = int(gps_delta // SECONDS_PER_WEEK)
        sow = gps_delta - week * SECONDS_PER_WEEK
        return week, sow

    def _bds_seconds_of_week(self, t):
        """Convert datetime to BDS seconds-of-week."""
        bds_delta = (t - BDS_EPOCH).total_seconds()
        week = int(bds_delta // SECONDS_PER_WEEK)
        sow = bds_delta - week * SECONDS_PER_WEEK
        return week, sow

    def sat_position(self, prn, t):
        """Compute satellite position and clock at time t.

        Args:
            prn: Satellite PRN string (e.g. 'G01', 'E05', 'C19')
            t: datetime (timezone-aware, GPS time)

        Returns:
            (np.array([x, y, z]), clock_seconds) or (None, None)
        """
        eph = self._ephs.get(prn)
        if eph is None:
            return None, None

        sys = eph['system']

        # Compute time since ephemeris reference epoch
        if sys == 'C':
            _, sow = self._bds_seconds_of_week(t)
        else:
            _, sow = self._gps_seconds_of_week(t)

        tk = _check_week_crossover(sow - eph['toe'])
        dt_clk = _check_week_crossover(sow - eph['toc'])

        # Keplerian position computation
        pos, Ek = _kepler_ecef(eph, tk, eph['gm'])

        # Satellite clock
        clk = _sat_clock(eph, dt_clk, Ek)

        # BDS time system correction: broadcast ephemeris clock is relative
        # to BDT, but the filter operates in GPST.  GPST = BDT + 14 s, so
        # dt_sv(GPST) = dt_sv(BDT) - 14 s.  Without this, the ~14 s offset
        # leaks into the ISB_BDS estimate and overwhelms the filter.
        if sys == 'C':
            clk -= BDT_GPST_OFFSET

        return pos, clk

    def get_iod(self, prn):
        """Return the Issue of Data for a satellite's current ephemeris.

        SSR corrections reference a specific IOD to ensure consistency
        between the broadcast ephemeris and the correction.
        """
        eph = self._ephs.get(prn)
        if eph is None:
            return None
        return eph.get('iode') or eph.get('iod')

    def sat_velocity(self, prn, t, dt=0.5):
        """Numerical satellite velocity via central difference.

        Needed for SSR orbit correction (radial/along-track/cross-track
        frame is defined relative to the velocity vector).

        Returns:
            np.array([vx, vy, vz]) in m/s, or None
        """
        pos_fwd, _ = self.sat_position(prn, t + timedelta(seconds=dt))
        pos_bck, _ = self.sat_position(prn, t - timedelta(seconds=dt))
        if pos_fwd is None or pos_bck is None:
            return None
        return (pos_fwd - pos_bck) / (2.0 * dt)

    def age_of_ephemeris(self, prn, t):
        """Return age (seconds) of ephemeris for a satellite at time t.

        Large ages (>2h for GPS, >4h for GAL) indicate stale ephemeris.
        """
        eph = self._ephs.get(prn)
        if eph is None:
            return None
        sys = eph['system']
        if sys == 'C':
            _, sow = self._bds_seconds_of_week(t)
        else:
            _, sow = self._gps_seconds_of_week(t)
        return abs(_check_week_crossover(sow - eph['toe']))

    def summary(self):
        """Return a string summarizing current ephemeris state."""
        by_sys = {'G': 0, 'E': 0, 'C': 0}
        for prn in self._ephs:
            sys = prn[0]
            if sys in by_sys:
                by_sys[sys] += 1
        return (f"BroadcastEph: G{by_sys['G']} E{by_sys['E']} C{by_sys['C']} "
                f"({self._update_count} updates)")

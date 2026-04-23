"""IGS ANTEX 1.4 parser + PCO/PCV correction for satellite and receiver
antennas.

ANTEX stores per-antenna calibration:
- PCO (phase center offset): vector from antenna reference point to
  mean phase center, in body frame (satellites) or NEU frame
  (receivers), millimetres.
- PCV (phase center variation): deviation from the mean phase center
  as a function of nadir angle (satellites, 1-D pattern) or
  (zenith, azimuth) grid (receivers, 2-D pattern), millimetres.

Apply to observed carrier phase at ingest:

  Δrange_pcv  =  (pco_sat_ecef · e_sat_to_rcv) + pcv_sat(nadir)
               + (pco_rcv_enu  · e_rcv_to_sat) + pcv_rcv(zen, az)

  phi_corrected = phi_observed + Δrange_pcv

Then IF-combine per-frequency corrections: Δrange_IF = α1·Δrange_L1 +
α2·Δrange_L2.  See IERS Conventions 2010 Section 7.1.4, IGS ANTEX
Format 1.4 specification.

Accuracy target: mm-level on PCO (dominant; up to 2.3 m for some
GPS blocks) and sub-mm on PCV (typically ±5 mm swing from mean).

This parser reads the subset of ANTEX fields our harness needs:
GPS + Galileo satellites (per-PRN, valid-epoch selection) and the
station's receiver antenna by RINEX-reported type.
"""

from __future__ import annotations

import math
from datetime import datetime, timezone
from typing import Optional

import numpy as np


# Frequency identifier → wavelength + system name map.  These are
# IGS ANTEX frequency identifiers (e.g. 'G01' = GPS L1, 'E01' = GAL E1,
# 'E05' = GAL E5a).  Full list per IGS ANTEX Format 1.4 spec §5.
ANTEX_FREQS = {
    'G01': ('gps', 'L1'),
    'G02': ('gps', 'L2'),
    'G05': ('gps', 'L5'),
    'E01': ('gal', 'E1'),
    'E05': ('gal', 'E5a'),
    'E07': ('gal', 'E5b'),
    'E06': ('gal', 'E6'),
    'E08': ('gal', 'E5'),
    'C01': ('bds', 'B1C'),
    'C02': ('bds', 'B1I'),
    'C05': ('bds', 'B2a'),
    'C06': ('bds', 'B3I'),
    'C07': ('bds', 'B2I'),
}


class AntennaPattern:
    """One antenna × one frequency's PCO + PCV."""

    def __init__(self, pco_mm: np.ndarray, dazi: float,
                 zen_start: float, zen_step: float, zen_end: float,
                 noazi_mm: list[float],
                 grid_mm: Optional[list[tuple[float, list[float]]]] = None):
        self.pco_m = np.asarray(pco_mm, dtype=float) * 1e-3  # mm → m
        self.dazi = dazi
        self.zen_start = zen_start
        self.zen_step = zen_step
        self.zen_end = zen_end
        self.noazi_m = np.asarray(noazi_mm, dtype=float) * 1e-3  # mm → m
        # grid_mm is [(az0, [pcv_per_zen]), (az5, [...]), ...] in mm
        if grid_mm:
            azs = np.array([row[0] for row in grid_mm])
            values = np.array([row[1] for row in grid_mm]) * 1e-3  # mm → m
            self.az_grid = azs
            self.pcv_grid = values
        else:
            self.az_grid = None
            self.pcv_grid = None

    def pcv(self, zen_deg: float, az_deg: float = 0.0) -> float:
        """Return PCV interpolated at the given zenith (or nadir) angle
        and (if available) azimuth.  Clamps zen to the table range.
        Returns metres."""
        z = max(self.zen_start, min(self.zen_end, zen_deg))
        # zen bin
        i = (z - self.zen_start) / self.zen_step
        i0 = int(math.floor(i))
        if i0 >= len(self.noazi_m) - 1:
            i0 = len(self.noazi_m) - 2
        frac_z = i - i0

        def _interp_zen(row):
            return row[i0] * (1.0 - frac_z) + row[i0 + 1] * frac_z

        if self.pcv_grid is None or self.dazi <= 0 or self.az_grid is None:
            return float(_interp_zen(self.noazi_m))
        # Bilinear in (zen, az); az wraps at 360
        az = az_deg % 360.0
        j = az / self.dazi
        j0 = int(math.floor(j)) % len(self.az_grid)
        j1 = (j0 + 1) % len(self.az_grid)
        frac_a = j - math.floor(j)
        v0 = _interp_zen(self.pcv_grid[j0])
        v1 = _interp_zen(self.pcv_grid[j1])
        return float(v0 * (1.0 - frac_a) + v1 * frac_a)


class ANTEXParser:
    """Parse an ANTEX 1.4 file.

    Stores satellite antennas keyed by (prn, freq_id) with valid-epoch
    metadata, and receiver antennas keyed by (antenna_type_14chars,
    freq_id).
    """

    def __init__(self, path: str):
        # {(prn, freq_id): list of (valid_from, valid_until, AntennaPattern)}
        self.sat_patterns: dict[tuple[str, str], list] = {}
        # {(ant_type, freq_id): AntennaPattern}  (receiver antennas rarely have validity ranges)
        self.recv_patterns: dict[tuple[str, str], AntennaPattern] = {}
        self._parse(path)

    def _parse(self, path: str):
        state = None
        header = {}
        freq_blocks = []
        current_freq = None
        current_noazi = None
        current_grid = []
        current_pco = None
        # Tags are written at cols 60-80 per ANTEX spec, but PCV data
        # rows are longer than 80 chars.  Look up tags via a membership
        # check on the known-tag set rather than relying on a fixed
        # column slice that overlaps PCV numeric data.
        known_tags = {
            'START OF ANTENNA', 'END OF ANTENNA', 'TYPE / SERIAL NO',
            'METH / BY / # / DATE', 'DAZI', 'ZEN1 / ZEN2 / DZEN',
            '# OF FREQUENCIES', 'VALID FROM', 'VALID UNTIL', 'SINEX CODE',
            'START OF FREQUENCY', 'NORTH / EAST / UP', 'END OF FREQUENCY',
            'COMMENT', 'ANTEX VERSION / SYST', 'PCV TYPE / REFANT',
            'END OF HEADER', 'END OF FILE',
        }
        with open(path, encoding='latin-1') as f:
            for line in f:
                # Identify the tag at cols 60-80 — but only if that slice
                # matches a known label (otherwise it's PCV numeric data).
                candidate = line[60:80].rstrip() if len(line) >= 60 else ''
                tag = candidate if candidate in known_tags else ''

                if tag == 'START OF ANTENNA':
                    header = {}
                    freq_blocks = []
                    state = 'in_antenna'
                    continue
                if tag == 'END OF ANTENNA':
                    self._store_antenna(header, freq_blocks)
                    state = None
                    continue
                if state != 'in_antenna':
                    continue
                if tag == 'TYPE / SERIAL NO':
                    header['type'] = line[0:20].rstrip()
                    header['prn'] = line[20:40].rstrip()
                elif tag == 'DAZI':
                    header['dazi'] = float(line[0:8])
                elif tag == 'ZEN1 / ZEN2 / DZEN':
                    header['zen1'] = float(line[0:8])
                    header['zen2'] = float(line[8:14])
                    header['dzen'] = float(line[14:20])
                elif tag == 'VALID FROM':
                    header['valid_from'] = _parse_epoch(line)
                elif tag == 'VALID UNTIL':
                    header['valid_until'] = _parse_epoch(line)
                elif tag == 'START OF FREQUENCY':
                    current_freq = line[3:6]
                    current_noazi = None
                    current_grid = []
                    current_pco = None
                elif tag == 'NORTH / EAST / UP':
                    n = float(line[0:10])
                    e = float(line[10:20])
                    u = float(line[20:30])
                    current_pco = (n, e, u)
                elif tag == 'END OF FREQUENCY':
                    if current_noazi is not None and current_pco is not None:
                        freq_blocks.append({
                            'freq_id': current_freq,
                            'pco': current_pco,
                            'noazi': current_noazi,
                            'grid': current_grid,
                        })
                    current_freq = None
                elif current_freq is not None and tag == '':
                    # One of the PCV rows.  "NOAZI" first, then az-indexed rows.
                    token = line[0:8].strip()
                    if token == 'NOAZI':
                        current_noazi = [float(x) for x in line[8:].split()]
                    else:
                        # Azimuth row
                        try:
                            az = float(token)
                            vals = [float(x) for x in line[8:].split()]
                            current_grid.append((az, vals))
                        except ValueError:
                            pass

    def _store_antenna(self, header, freq_blocks):
        ant_type = header.get('type', '').strip()
        prn = header.get('prn', '').strip()
        # Satellite antennas have PRN in the PRN field (e.g. 'G01', 'E05')
        is_sat = bool(prn) and prn[0] in 'GREJCIS' and len(prn) == 3 and prn[1:].isdigit()
        for fb in freq_blocks:
            pat = AntennaPattern(
                pco_mm=fb['pco'],
                dazi=header.get('dazi', 0.0),
                zen_start=header.get('zen1', 0.0),
                zen_step=header.get('dzen', 1.0),
                zen_end=header.get('zen2', 90.0),
                noazi_mm=fb['noazi'],
                grid_mm=fb['grid'] if fb['grid'] else None,
            )
            if is_sat:
                key = (prn, fb['freq_id'])
                entry = (header.get('valid_from'),
                         header.get('valid_until'), pat)
                self.sat_patterns.setdefault(key, []).append(entry)
            else:
                # Receiver antenna — ant_type is the antenna type string
                # (e.g. 'TRM57971.00     NONE').  Key by full 20-char type.
                key = (ant_type, fb['freq_id'])
                # Last one wins if multiple entries (receiver antennas are
                # typically unique).
                self.recv_patterns[key] = pat

    def get_sat_pattern(self, prn: str, freq_id: str,
                        t: datetime) -> Optional[AntennaPattern]:
        """Return the satellite AntennaPattern valid at epoch t, or None.
        """
        entries = self.sat_patterns.get((prn, freq_id))
        if not entries:
            return None
        for valid_from, valid_until, pat in entries:
            if (valid_from is None or t >= valid_from) and \
               (valid_until is None or t <= valid_until):
                return pat
        # No valid window — fall back to first entry (loose-matching
        # prevents a dropout for satellites that happen to be just
        # outside a valid range).
        return entries[0][2]

    def get_recv_pattern(self, ant_type: str,
                         freq_id: str) -> Optional[AntennaPattern]:
        """Return receiver AntennaPattern for a given antenna type string.
        ant_type should be the full 20-char TYPE field from ANTEX (e.g.
        'TRM57971.00     NONE').  Returns None if no match found.

        Standard IGS convention for receiver PCVs: most receiver
        antennas only publish GPS L1/L2 (G01/G02) patterns.  Non-GPS
        frequencies fall back to the same-band GPS pattern:

        - E01, C01, J01, R01  → G01 (shared L1 band)
        - E05, C05, J05, B2a  → G05 or G02 (close L5 band)
        - E07 (E5b), C07 (B2I) → G02 (same-frequency → same delay)
        - E06 (E6), C06 (B3I) → G02 (closest band)
        - E08 (E5)            → G05/G02

        If no match exists even after fallback, return None.
        """
        direct = self.recv_patterns.get((ant_type, freq_id))
        if direct is not None:
            return direct
        # Frequency fallback per IGS convention
        fallbacks = {
            'E01': ('G01',), 'C01': ('G01',),
            'E05': ('G05', 'G02', 'G01'), 'C05': ('G05', 'G02', 'G01'),
            'E07': ('G02', 'G01'), 'C07': ('G02', 'G01'),
            'E06': ('G02', 'G01'), 'C06': ('G02', 'G01'),
            'E08': ('G05', 'G02', 'G01'),
            'G05': ('G02', 'G01'),
        }.get(freq_id, ())
        for alt in fallbacks:
            pat = self.recv_patterns.get((ant_type, alt))
            if pat is not None:
                return pat
        return None

    def get_sat_pattern_fallback(self, prn: str, freq_id: str,
                                 t: datetime) -> Optional[AntennaPattern]:
        """Like get_sat_pattern but with frequency fallback (for SVs
        that don't have all frequencies characterised — especially
        L5 which is a later addition for most GPS blocks).  Returns
        (pattern, effective_freq_id)."""
        direct = self.get_sat_pattern(prn, freq_id, t)
        if direct is not None:
            return direct
        fallbacks = {
            'G05': ('G02', 'G01'),
            'E05': ('E01',), 'E07': ('E01',),
            'C05': ('C02', 'C01'), 'C07': ('C02', 'C01'),
        }.get(freq_id, ())
        for alt in fallbacks:
            pat = self.get_sat_pattern(prn, alt, t)
            if pat is not None:
                return pat
        return None


def _parse_epoch(line: str) -> Optional[datetime]:
    """Parse a VALID FROM / VALID UNTIL record in ANTEX format."""
    try:
        y = int(line[0:6])
        m = int(line[6:12])
        d = int(line[12:18])
        h = int(line[18:24])
        mn = int(line[24:30])
        s = float(line[30:43])
        si = int(s)
        us = int(round((s - si) * 1e6))
        us = min(999999, max(0, us))
        return datetime(y, m, d, h, mn, si, us, tzinfo=timezone.utc)
    except (ValueError, IndexError):
        return None


def ecef_to_enu_matrix(station_ecef: np.ndarray) -> np.ndarray:
    """Return the 3x3 matrix that converts an ECEF delta vector to ENU
    at the given station position.  Rows are East, North, Up unit
    vectors in ECEF coordinates."""
    x, y, z = station_ecef
    r_xy = math.sqrt(x * x + y * y)
    if r_xy < 1.0:
        # At a pole — punt.  PePPAR Fix's lab is not at a pole.
        return np.eye(3)
    lat = math.atan2(z, r_xy)
    lon = math.atan2(y, x)
    sl, cl = math.sin(lon), math.cos(lon)
    sp, cp = math.sin(lat), math.cos(lat)
    east = np.array([-sl, cl, 0.0])
    north = np.array([-sp * cl, -sp * sl, cp])
    up = np.array([cp * cl, cp * sl, sp])
    return np.vstack([east, north, up])


def sat_body_frame(sat_pos_ecef: np.ndarray,
                   sun_pos_ecef: np.ndarray) -> tuple[np.ndarray,
                                                      np.ndarray,
                                                      np.ndarray]:
    """Nominal yaw-steering satellite body frame (IGS convention).

    Returns the three orthonormal ECEF unit vectors (e_x, e_y, e_z)
    of the satellite body frame:

    - e_z: nadir direction (toward Earth center) = -sat_pos_hat
    - e_y: solar panel rotation axis (perpendicular to Sun-Earth-
      satellite plane) = normalize(e_z × e_sun)
    - e_x: completes a right-handed frame = e_y × e_z

    The ANTEX "NORTH" column corresponds to the body-X component,
    "EAST" to body-Y, "UP" to body-Z (IGS convention, per Montenbruck
    et al. 2015 and the IGS14 reprocessing campaign).

    This is the NOMINAL attitude.  During eclipse / near orbit-noon
    or orbit-midnight for satellites with small β angles, the
    actual attitude deviates from nominal by up to the yaw rate's
    saturation time — typically < 30 minutes per orbit.  PRIDE uses
    per-epoch attitude from the WUM ATT.OBX file to capture this;
    we approximate with nominal throughout, which is adequate at
    the cm scale of PCV effects for most epochs.
    """
    r_sat = float(np.linalg.norm(sat_pos_ecef))
    e_z = -sat_pos_ecef / r_sat
    e_sun = sun_pos_ecef - sat_pos_ecef
    e_sun = e_sun / float(np.linalg.norm(e_sun))
    cross = np.cross(e_z, e_sun)
    e_y = cross / float(np.linalg.norm(cross))
    e_x = np.cross(e_y, e_z)
    return e_x, e_y, e_z


def nadir_angle_deg(sat_pos_ecef: np.ndarray,
                    receiver_pos_ecef: np.ndarray) -> float:
    """Compute the nadir angle at the satellite for its LOS to the
    receiver.  Nadir = angle between the satellite's Earth-pointing
    (+Z body) axis and the line of sight to the receiver.  For GPS
    satellites at 20,200 km altitude, nadir ranges 0° (receiver
    directly below) to ~14.3° (Earth-limb grazing LOS).

    Nominal yaw-steering approximation: +Z-body = -sat_pos / |sat_pos|
    (points to Earth center).  Adequate at the mm-scale PCV precision
    we need.
    """
    sat_r = np.linalg.norm(sat_pos_ecef)
    if sat_r < 1.0:
        return 0.0
    body_z = -sat_pos_ecef / sat_r
    los = receiver_pos_ecef - sat_pos_ecef
    los_r = np.linalg.norm(los)
    if los_r < 1.0:
        return 0.0
    los_hat = los / los_r
    cos_nadir = float(np.dot(body_z, los_hat))
    cos_nadir = max(-1.0, min(1.0, cos_nadir))
    return math.degrees(math.acos(cos_nadir))

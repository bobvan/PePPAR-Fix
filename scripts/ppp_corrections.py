#!/usr/bin/env python3
"""
ppp_corrections.py — Parsers for PPP correction products.

Classes:
    OSBParser  — Parse SINEX BIAS (OSB.BIA) files for observable-specific signal biases
    CLKFile    — Parse RINEX clock files for satellite clock offsets with interpolation

Both are designed as standalone utilities for integration into solve_ppp.py.
"""

from datetime import datetime, timezone

import numpy as np

# Speed of light (m/s) — same constant as solve_pseudorange.C
C = 299792458.0


# ── OSB Parser ────────────────────────────────────────────────────────────── #

class OSBParser:
    """Parse a SINEX BIAS file and provide OSB lookups in meters.

    The file format (GFZ OSB.BIA) has lines like:
     OSB  C201 C19           C1X       2026:062:00000 2026:063:03600 ns  -6.3698  0.0700

    Fields: type SVN PRN station obs1 obs2 start end unit value stddev
    (station and obs2 are typically blank for satellite OSBs)
    """

    def __init__(self, path):
        # biases[(prn, signal_code)] = (value_ns, stddev_ns)
        self.biases = {}
        self._parse(path)

    def _parse(self, path):
        in_solution = False
        with open(path) as f:
            for line in f:
                if line.startswith('+BIAS/SOLUTION'):
                    in_solution = True
                    continue
                if line.startswith('-BIAS/SOLUTION'):
                    break
                if not in_solution:
                    continue
                if line.startswith('*') or len(line.strip()) == 0:
                    continue
                if not line.startswith(' OSB'):
                    continue

                # Fixed-width parse based on SINEX BIAS format
                # Cols:  1-4  type (OSB)
                #        6-9  SVN
                #       11-13 PRN
                #       15-23 station (blank for satellite)
                #       25-27 OBS1 (signal code)
                #       29-32 OBS2 (blank for OSB)
                #       34-47 BIAS_START
                #       49-62 BIAS_END
                #       64-65 UNIT
                #       68-87 ESTIMATED_VALUE
                #       89-99 STD_DEV
                try:
                    parts = line.split()
                    # parts: ['OSB', svn, prn, signal_code, start, end, unit, value, stddev]
                    # But station field may be empty, shifting things.
                    # Safer: find the key fields
                    bias_type = parts[0]  # 'OSB'
                    svn = parts[1]
                    prn = parts[2]
                    signal_code = parts[3]
                    # start, end, unit follow
                    # Find 'ns' to anchor
                    ns_idx = parts.index('ns')
                    value_ns = float(parts[ns_idx + 1])
                    stddev_ns = float(parts[ns_idx + 2])
                except (ValueError, IndexError):
                    continue

                self.biases[(prn, signal_code)] = (value_ns, stddev_ns)

    def get_osb(self, prn, signal_code):
        """Return the OSB for (prn, signal_code) in meters.

        Converts from ns to meters: bias_m = bias_ns * 1e-9 * C.
        Returns None if not found.
        """
        entry = self.biases.get((prn, signal_code))
        if entry is None:
            return None
        return entry[0] * 1e-9 * C

    def get_osb_ns(self, prn, signal_code):
        """Return the raw OSB in nanoseconds, or None."""
        entry = self.biases.get((prn, signal_code))
        if entry is None:
            return None
        return entry[0]

    def get_if_osb(self, prn, code_f1, code_f2, alpha_f1, alpha_f2):
        """Return IF-combined code OSB in meters.

        IF_OSB = alpha_f1 * OSB(code_f1) - alpha_f2 * OSB(code_f2)

        Returns None if either signal bias is missing.
        """
        osb_f1 = self.get_osb(prn, code_f1)
        osb_f2 = self.get_osb(prn, code_f2)
        if osb_f1 is None or osb_f2 is None:
            return None
        return alpha_f1 * osb_f1 - alpha_f2 * osb_f2

    def get_phase_if_osb(self, prn, phase_f1, phase_f2, alpha_f1, alpha_f2):
        """Return IF-combined phase OSB in meters.

        Same formula as get_if_osb but for phase signal codes (L1C, L5Q, etc.).
        Returns None if either phase bias is missing.
        """
        osb_f1 = self.get_osb(prn, phase_f1)
        osb_f2 = self.get_osb(prn, phase_f2)
        if osb_f1 is None or osb_f2 is None:
            return None
        return alpha_f1 * osb_f1 - alpha_f2 * osb_f2

    def prns(self):
        """Return set of all PRNs in the file."""
        return set(prn for prn, _ in self.biases)

    def signals(self, prn):
        """Return list of signal codes available for a given PRN."""
        return [sig for (p, sig) in self.biases if p == prn]


# ── RINEX CLK Parser ─────────────────────────────────────────────────────── #

class CLKFile:
    """Parse a RINEX clock file and provide interpolated satellite clock offsets.

    File format (AS = satellite clock):
    AS G01  2026  3  3  0  0  0.000000  1    0.324012876388E-03

    Stores clock offsets indexed by PRN with timestamps for interpolation.
    """

    def __init__(self, path):
        # clocks[prn] = [(datetime, offset_seconds), ...] sorted by time
        self._clocks = {}
        self._parse(path)
        # Build numpy arrays for fast interpolation
        self._t0 = {}       # {prn: first_epoch_datetime}
        self._tsec = {}     # {prn: np.array of seconds since t0}
        self._offsets = {}  # {prn: np.array of clock offsets in seconds}
        for prn, records in self._clocks.items():
            records.sort(key=lambda r: r[0])
            t0 = records[0][0]
            self._t0[prn] = t0
            self._tsec[prn] = np.array(
                [(r[0] - t0).total_seconds() for r in records]
            )
            self._offsets[prn] = np.array([r[1] for r in records])

    def _parse(self, path):
        in_header = True
        with open(path) as f:
            for line in f:
                if in_header:
                    if 'END OF HEADER' in line:
                        in_header = False
                    continue

                if not line.startswith('AS '):
                    continue

                try:
                    prn = line[3:7].strip()
                    year = int(line[8:12])
                    month = int(line[12:15])
                    day = int(line[15:18])
                    hour = int(line[18:21])
                    minute = int(line[21:24])
                    sec_str = line[24:34].strip()
                    sec = float(sec_str)
                    si = int(sec)
                    us = int(round((sec - si) * 1e6))

                    # n_values = int(line[34:39])  # not needed
                    offset_s = float(line[39:].strip())

                    epoch = datetime(year, month, day, hour, minute, si, us,
                                     tzinfo=timezone.utc)

                    if prn not in self._clocks:
                        self._clocks[prn] = []
                    self._clocks[prn].append((epoch, offset_s))
                except (ValueError, IndexError):
                    continue

    def sat_clock(self, prn, gps_time):
        """Return interpolated satellite clock offset in seconds.

        Uses linear interpolation between the two nearest epochs.
        Returns None if the PRN isn't in the file or time is out of range.

        Args:
            prn: Satellite PRN string (e.g. 'G01')
            gps_time: datetime object (should be timezone-aware UTC/GPS time)
        """
        if prn not in self._t0:
            return None

        t0 = self._t0[prn]
        tsec = self._tsec[prn]
        offsets = self._offsets[prn]

        t_query = (gps_time - t0).total_seconds()

        # Out of range check
        if t_query < tsec[0] or t_query > tsec[-1]:
            return None

        # Find bracketing indices
        idx = np.searchsorted(tsec, t_query, side='right') - 1
        idx = max(0, min(idx, len(tsec) - 2))

        t1 = tsec[idx]
        t2 = tsec[idx + 1]
        dt = t2 - t1
        if dt == 0:
            return float(offsets[idx])

        # Linear interpolation
        frac = (t_query - t1) / dt
        return float(offsets[idx] + frac * (offsets[idx + 1] - offsets[idx]))

    def prns(self):
        """Return set of all satellite PRNs in the file."""
        return set(self._t0.keys())

    def time_range(self, prn):
        """Return (first_epoch, last_epoch) for a PRN, or None."""
        if prn not in self._t0:
            return None
        t0 = self._t0[prn]
        last_sec = self._tsec[prn][-1]
        from datetime import timedelta
        return (t0, t0 + timedelta(seconds=float(last_sec)))

    def n_epochs(self, prn):
        """Return number of clock epochs for a PRN."""
        if prn not in self._tsec:
            return 0
        return len(self._tsec[prn])

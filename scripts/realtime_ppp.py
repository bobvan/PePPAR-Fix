#!/usr/bin/env python3
"""
realtime_ppp.py — Real-time PPP clock estimation from live GNSS observations.

Milestone 4: Combines live F9T RXM-RAWX observations with real-time SSR
corrections (via NTRIP) to produce continuous sub-ns clock estimates.

Architecture:
    F9T serial ──→ UBX parser ──→ RXM-RAWX → IF observations
                                  RXM-SFRBX → broadcast ephemeris
    NTRIP caster ──→ RTCM3 ──→ SSR orbit/clock/bias corrections
                               broadcast ephemeris (1019/1045/1042)
                     ↓                    ↓
              FixedPosFilter ←────────────┘
                     ↓
              Clock estimate (ns) → CSV + console

Usage:
    python realtime_ppp.py --serial /dev/gnss-bot --baud 115200 \\
        --known-pos "LAT,LON,ALT" \\
        --caster products.igs-ip.net --port 2101 \\
        --eph-mount BCEP00BKG0 --ssr-mount SSRA00CNE0 \\
        --user myuser --password mypass \\
        --duration 3600 --out data/realtime_test.csv

    # Ephemeris-only mode (no SSR, broadcast-quality corrections):
    python realtime_ppp.py --serial /dev/gnss-bot --baud 115200 \\
        --known-pos "LAT,LON,ALT" \\
        --caster products.igs-ip.net --port 2101 \\
        --eph-mount BCEP00BKG0 \\
        --duration 3600

    # File-based replay mode (for development/testing):
    python realtime_ppp.py --replay data/rawx_1h_top_20260303.csv \\
        --sp3 data/gfz_mgx_062.sp3 --clk data/GFZ0MGXRAP_062_30S.CLK \\
        --osb data/GFZ0MGXRAP_062_OSB.BIA \\
        --known-pos "LAT,LON,ALT"
"""

import argparse
import csv
import logging
import math
import os
import queue
import sys
import threading
import time
from collections import defaultdict, deque
from datetime import datetime, timezone, timedelta

import numpy as np

# Project imports
from solve_pseudorange import (
    SP3, C, OMEGA_E, lla_to_ecef, timestamp_to_gpstime,
)
from solve_dualfreq import (
    IF_PAIRS, F_L1, F_L2, F_L5, F_E5B, F_B1I, F_B2I,
    ALPHA_L1_L2, ALPHA_L2, ALPHA_L1, ALPHA_L5,
    ALPHA_E1, ALPHA_E5B, ALPHA_B1I, ALPHA_B2A,
    ALPHA_B1I_B2I, ALPHA_B2I,
)
from solve_ppp import (
    FixedPosFilter, SIG_TO_RINEX, IF_WL, ELEV_MASK,
    SIGMA_P_IF, SIGMA_PHI_IF, BDS_MIN_PRN,
    load_ppp_epochs,
)
from ppp_corrections import OSBParser, CLKFile
from broadcast_eph import BroadcastEphemeris
from ssr_corrections import SSRState, RealtimeCorrections
from ntrip_client import NtripStream
from peppar_fix.event_time import (
    ObservationEvent,
    RtcmEvent,
    estimator_sample_weight,
    estimate_correlation_confidence,
)
from peppar_fix.timebase_estimator import TimebaseRelationEstimator
from peppar_fix.fault_injection import get_delay_injector, get_source_mute_controller

log = logging.getLogger(__name__)

# Broadcast ephemeris RTCM message types
EPH_MSG_TYPES = {'1019', '1042', '1045', '1046'}

# SSR message types (IGS SSR 4076 + standard RTCM)
SSR_MSG_TYPES = set()
for _sub in range(21, 28):
    SSR_MSG_TYPES.add(f'4076_{_sub:03d}')
for _sub in range(61, 68):
    SSR_MSG_TYPES.add(f'4076_{_sub:03d}')
for _sub in range(101, 108):
    SSR_MSG_TYPES.add(f'4076_{_sub:03d}')
for _mt in range(1057, 1069):
    SSR_MSG_TYPES.add(str(_mt))
for _mt in range(1240, 1271):  # 1240-1263 (orbit/clock/code) + 1265-1270 (phase bias)
    SSR_MSG_TYPES.add(str(_mt))

# Bias subset — used when pairing two SSR mounts (orbit/clock from one AC,
# observable-specific biases from another AC that publishes OSBs).  Covers
# BOTH code-bias and phase-bias messages because per-observable code biases
# and phase biases must come from the same AC to stay internally consistent:
# CNES publishing C1C+L2W code biases while WHU publishes C5Q+L5Q phase
# biases would leave GPS L5Q pseudoranges uncorrected, producing tens of
# meters of position drift (observed 2026-04-18 before this fix).
#
# Standard RTCM:
#   1059 GPS code bias, 1065 GLO (legacy lanes), 1067 GLO phase bias,
#   1242 GAL code bias, 1264 BDS code bias,
#   1265–1270 per-constellation phase biases.
# IGS SSR 4076 subtypes 25/65/105 = code bias; 26/66/106 = phase bias
# (GPS/GAL/BDS ranges).
BIAS_MSG_TYPES = {
    '1059', '1065', '1067', '1242', '1264',
    '1265', '1266', '1267', '1268', '1269', '1270',
    '4076_025', '4076_026',
    '4076_065', '4076_066',
    '4076_105', '4076_106',
}


# u-blox F9T reports BDS-3 modernized-signal cpMes in L1-reference cycles
# (i.e. the reported cycle count, when scaled by λ_L1, equals the geometric
# path).  To obtain native-carrier cycles, multiply the reported cpMes by
# (λ_L1 / λ_native) = F_native/F_L1.  Verified empirically on ZED-F9T
# TIM 2.25 at MadHat 2026-04-19.  Legacy BDS-2 signals (B1I, B2I, B3I) are
# unaffected — they are reported in native cycles like GPS and GAL.
#
# B2a-I and B2a-Q are confirmed.  B1C is plausibly the same and listed here
# tentatively so a GF-DIAG on a B1C-tracking receiver will produce clean
# numbers if the quirk extends to it; drop from the set if diagnostics show
# B1C in native cycles.
_LAMBDA_L1 = C / F_L1
_BDS_L1_REF_CYCLES = {
    'BDS-B2aI', 'BDS-B2aQ',
    'BDS-B1C', 'BDS-B1CD', 'BDS-B1CP',  # tentative — not lab-confirmed yet
}

SIG_WAVELENGTH = {
    'GPS-L1CA': C / F_L1,
    'GPS-L2CL': C / F_L2,
    'GPS-L2CM': C / F_L2,
    'GPS-L5I': C / F_L5,
    'GPS-L5Q': C / F_L5,
    'GAL-E1C': C / F_L1,
    'GAL-E1B': C / F_L1,
    'GAL-E5aI': C / F_L5,
    'GAL-E5aQ': C / F_L5,
    'GAL-E5bI': C / F_E5B,
    'GAL-E5bQ': C / F_E5B,
    'BDS-B1I': C / F_B1I,
    'BDS-B2I': C / F_B2I,
    'BDS-B2aI': C / F_L5,
    'BDS-B2aQ': C / F_L5,
}

IF_PAIR_PARAMS = {
    ('GPS-L1CA', 'GPS-L2CL'): ('G', ALPHA_L1_L2, ALPHA_L2),
    ('GPS-L1CA', 'GPS-L5Q'): ('G', ALPHA_L1, ALPHA_L5),
    ('GAL-E1C', 'GAL-E5bQ'): ('E', ALPHA_E1, ALPHA_E5B),
    ('GAL-E1C', 'GAL-E5aQ'): ('E', ALPHA_L1, ALPHA_L5),
    ('BDS-B1I', 'BDS-B2I'): ('C', ALPHA_B1I_B2I, ALPHA_B2I),
    ('BDS-B1I', 'BDS-B2aI'): ('C', ALPHA_B1I, ALPHA_B2A),
}


class RtcmMessageView:
    """RTCM message wrapper that preserves decoded fields plus timing metadata."""

    def __init__(self, message, event):
        self._message = message
        self.recv_mono = event.recv_mono
        self.recv_utc = event.recv_utc
        self.queue_remains = event.queue_remains
        self.parse_age_s = event.parse_age_s
        self.correlation_confidence = event.correlation_confidence

    def __getattr__(self, name):
        return getattr(self._message, name)


# ── Serial observation reader ──────────────────────────────────────────────── #

class QErrStore:
    """Thread-safe history of TIM-TP quantization error samples.

    The optional ``log_writer`` is a csv.writer (or similar) configured
    with the header
    ``host_timestamp,host_monotonic,qerr_ns,tow_ms,qerr_invalid``.
    When set, every call to ``update()`` writes one row, capturing the
    full TIM-TP arrival stream as raw as possible — independent of
    servo epochs, EXTTS reads, or any downstream consumer.

    The intended use is for post-hoc index-matching against the TICC
    chB log: each TICC csv row already has ``host_monotonic`` (when the
    line arrived from the TICC over USB CDC ACM), and each qErr csv row
    has ``host_monotonic`` (when the TIM-TP UBX message was parsed
    after arrival from the F9T).  Both are CLOCK_MONOTONIC on the same
    host, so they can be matched directly without any wall-clock,
    GPS TOW, or sawtooth-dewrap heuristics — same way the engine's
    QErrStore.match_pps_mono matches qErr to PPS events at runtime.
    """

    def __init__(self, maxlen=128, log_writer=None, log_file=None):
        self._lock = threading.Lock()
        self._samples = deque(maxlen=maxlen)
        self._log_writer = log_writer
        self._log_file = log_file  # so .flush() works after every write
        # Sequential FIFO for TICC chB consumption.  Each TIM-TP is
        # consumed by exactly one chB event via consume_next().
        # TIM-TP(N) arrives ~0.9s before chB(N), so the FIFO ordering
        # is natural: TIM-TP enqueues first, chB dequeues second.
        self._fifo = deque(maxlen=8)
        # TIM-TP-initiated matching: the most recent fresh TIM-TP
        # sets _pending_for_chb.  The next fresh chB in [+800, +1100]ms
        # consumes it.
        self._pending_for_chb = None  # (host_mono, qerr_ns) or None

    @staticmethod
    def _normalize_tow_ms(tow_ms):
        if tow_ms is None:
            return None
        return int(round(float(tow_ms))) % (7 * 86400 * 1000)

    @staticmethod
    def gps_tow_ms(gps_time):
        """Convert GPS datetime to GPS time-of-week in milliseconds."""
        gps_epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
        total_seconds = (gps_time - gps_epoch).total_seconds()
        week_seconds = total_seconds % (7 * 86400)
        return int(round(week_seconds * 1000)) % (7 * 86400 * 1000)

    @staticmethod
    def _tow_delta_ms(a_ms, b_ms):
        if a_ms is None or b_ms is None:
            return None
        week_ms = 7 * 86400 * 1000
        delta = (int(a_ms) - int(b_ms)) % week_ms
        if delta > week_ms / 2:
            delta -= week_ms
        return int(delta)

    def update(self, qerr_ps, tow_ms, qerr_invalid=False):
        """Store new qErr (picoseconds from TIM-TP) as nanoseconds.

        Captures CLOCK_MONOTONIC at the moment the message is processed.
        If a log writer was provided at construction time, also emits
        one row to the qErr CSV log so post-processing can index-match
        against TICC chB events by monotonic time.

        ``qerr_invalid`` is the qErrInvalid flag from TIM-TP.  Invalid
        samples are still recorded to the log (for completeness) but
        are not appended to the in-memory deque the engine uses for
        live correlation.
        """
        host_time = time.monotonic()
        host_wall = time.time()
        norm_tow = self._normalize_tow_ms(tow_ms)
        qerr_ns = qerr_ps / 1000.0

        if self._log_writer is not None:
            try:
                self._log_writer.writerow([
                    f"{host_wall:.6f}",
                    f"{host_time:.9f}",
                    f"{qerr_ns:.3f}",
                    norm_tow if norm_tow is not None else "",
                    1 if qerr_invalid else 0,
                ])
                if self._log_file is not None:
                    self._log_file.flush()
            except (OSError, ValueError):
                pass  # never let logging break the stream

        if qerr_invalid:
            return  # don't pollute the in-memory store with invalid samples

        with self._lock:
            self._samples.append({
                "qerr_ns": qerr_ns,
                "tow_ms": norm_tow,
                "host_time": host_time,
            })
            self._fifo.append(qerr_ns)
            # Offer this TIM-TP for chB matching.  The chB side
            # checks the timing window — stale TIM-TP (queued)
            # will have a host_time too far in the past for the
            # window to match.
            self._pending_for_chb = (host_time, qerr_ns)

    def get_pending_for_chb(self):
        """Return the pending (host_mono, qerr_ns) or None."""
        with self._lock:
            return self._pending_for_chb

    def clear_pending(self):
        """Mark the pending TIM-TP as consumed by a chB event."""
        with self._lock:
            self._pending_for_chb = None

    def consume_next(self):
        """Pop the oldest unconsumed qerr from the FIFO.

        For sequential 1:1 pairing with TICC chB events.  Each TIM-TP
        sample is consumed exactly once.  Returns qerr_ns or None if
        the FIFO is empty (TIM-TP hasn't arrived yet for this epoch).
        """
        with self._lock:
            if not self._fifo:
                return None
            return self._fifo.popleft()

    def flush_fifo(self):
        """Discard all pending FIFO entries.

        Called when the TICC reader starts to discard stale TIM-TP
        samples that arrived before the first chB event.
        """
        with self._lock:
            self._fifo.clear()

    def get(self, max_age_s=2.0):
        """Return (qerr_ns, tow_ms) or (None, None) if stale/unavailable."""
        with self._lock:
            if not self._samples:
                return None, None
            latest = self._samples[-1]
            if time.monotonic() - latest["host_time"] > max_age_s:
                return None, None
            return latest["qerr_ns"], latest["tow_ms"]

    def snapshot(self, max_age_s=2.0):
        """Return latest qErr sample metadata or Nones if stale/unavailable."""
        with self._lock:
            if not self._samples:
                return None, None, None
            latest = self._samples[-1]
            age_s = time.monotonic() - latest["host_time"]
            if age_s > max_age_s:
                return None, None, None
            return latest["qerr_ns"], latest["tow_ms"], age_s

    def match_pps_mono(self, pps_recv_mono, expected_offset_s=0.9,
                       tolerance_s=0.2, max_age_s=5.0):
        """Match qErr to a PPS edge by host monotonic time.

        TIM-TP describes the *next* timepulse and arrives ~900 ms before
        the PPS edge it describes.  This correlates them solely by host
        clock, independent of GPS TOW, receiver clock bias, or servo
        state.

        Returns ``(qerr_ns, offset_s)`` or ``(None, None)`` when no
        sample falls within the tolerance window.
        """
        with self._lock:
            best = None
            for sample in reversed(self._samples):
                age_s = pps_recv_mono - sample["host_time"]
                if age_s > max_age_s:
                    break  # oldest-first insertion; older won't match
                offset_err = abs(age_s - expected_offset_s)
                if offset_err > tolerance_s:
                    continue
                if best is None or offset_err < best[0]:
                    best = (offset_err, sample, age_s)
            if best is None:
                return None, None
            _, sample, offset_s = best
            return sample["qerr_ns"], offset_s

    def match_gps_time(self, gps_time, max_age_s=30.0, max_tow_delta_ms=1000):
        """Return qErr matched to the GNSS epoch second.

        TIM-TP describes the timing of the *next* timepulse, so its towMS
        is 1 second ahead of the current epoch.  RAWX rcvTow includes
        receiver clock bias (~-10 ms on TimeHat), placing it just below
        the true integer second — round() recovers the correct second.

        Returns `(qerr_ns, tow_ms, age_s, tow_delta_ms)` or Nones when no
        sufficiently fresh, close TIM-TP sample is available.
        """
        target_tow_ms = self._normalize_tow_ms(
            int(round(self.gps_tow_ms(gps_time) / 1000.0)) * 1000
        )
        now = time.monotonic()
        with self._lock:
            best = None
            for sample in reversed(self._samples):
                age_s = now - sample["host_time"]
                if age_s > max_age_s:
                    continue
                tow_delta_ms = self._tow_delta_ms(sample["tow_ms"], target_tow_ms)
                if tow_delta_ms is None or abs(tow_delta_ms) > max_tow_delta_ms:
                    continue
                if best is None:
                    best = (sample, age_s, tow_delta_ms)
                    continue
                _, best_age_s, best_delta_ms = best
                rank = (abs(tow_delta_ms), age_s)
                best_rank = (abs(best_delta_ms), best_age_s)
                if rank < best_rank:
                    best = (sample, age_s, tow_delta_ms)
            if best is None:
                return None, None, None, None
            sample, age_s, tow_delta_ms = best
            return sample["qerr_ns"], sample["tow_ms"], age_s, tow_delta_ms

    def debug_match_gps_time(self, gps_time, max_age_s=30.0, max_tow_delta_ms=1000):
        """Return detailed debug info for qErr matching."""
        target_tow_ms = self._normalize_tow_ms(
            int(round(self.gps_tow_ms(gps_time) / 1000.0)) * 1000
        )
        now = time.monotonic()
        with self._lock:
            info = {
                "sample_count": len(self._samples),
                "target_tow_ms": target_tow_ms,
                "latest_tow_ms": None,
                "latest_age_s": None,
                "latest_qerr_ns": None,
                "best_tow_ms": None,
                "best_age_s": None,
                "best_tow_delta_ms": None,
                "best_qerr_ns": None,
            }
            if self._samples:
                latest = self._samples[-1]
                info["latest_tow_ms"] = latest["tow_ms"]
                info["latest_age_s"] = now - latest["host_time"]
                info["latest_qerr_ns"] = latest["qerr_ns"]
            best = None
            for sample in reversed(self._samples):
                age_s = now - sample["host_time"]
                if age_s > max_age_s:
                    continue
                tow_delta_ms = self._tow_delta_ms(sample["tow_ms"], target_tow_ms)
                if tow_delta_ms is None or abs(tow_delta_ms) > max_tow_delta_ms:
                    continue
                if best is None or (abs(tow_delta_ms), age_s) < (abs(best[2]), best[1]):
                    best = (sample, age_s, tow_delta_ms)
            if best is not None:
                sample, age_s, tow_delta_ms = best
                info["best_tow_ms"] = sample["tow_ms"]
                info["best_age_s"] = age_s
                info["best_tow_delta_ms"] = tow_delta_ms
                info["best_qerr_ns"] = sample["qerr_ns"]
            return info


class Nav2PositionStore:
    """Thread-safe latest position from the F9T's secondary navigation engine.

    The F9T has two independent nav engines.  The primary (NAV-*) runs in
    whatever mode we configure (typically TIME / fixed-position for timing).
    The secondary (NAV2-*) always computes a fresh position fix regardless
    of the primary's mode.

    We use NAV2-PVT as a "third opinion" on antenna position — independent
    of our PPP filter and of the primary engine's fixed-position assumption.
    See docs/architecture-vision.md "Three-source position consensus".
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._lat = None          # degrees
        self._lon = None
        self._height_m = None     # height above ellipsoid (m)
        self._h_acc_m = None      # horizontal accuracy estimate (m)
        self._v_acc_m = None      # vertical accuracy estimate (m)
        self._pdop = None         # position dilution of precision
        self._fix_type = None     # 0=no, 2=2D, 3=3D
        self._gnss_fix_ok = False # fix valid flag
        self._num_sv = 0
        self._host_mono = None
        self._update_count = 0

    def update(self, parsed_msg):
        """Store a fresh NAV2-PVT decoded message."""
        with self._lock:
            self._lat = getattr(parsed_msg, 'lat', None)
            self._lon = getattr(parsed_msg, 'lon', None)
            self._height_m = getattr(parsed_msg, 'height', None)
            if self._height_m is not None:
                self._height_m /= 1000.0  # mm → m
            self._h_acc_m = getattr(parsed_msg, 'hAcc', None)
            if self._h_acc_m is not None:
                self._h_acc_m /= 1000.0  # mm → m
            self._v_acc_m = getattr(parsed_msg, 'vAcc', None)
            if self._v_acc_m is not None:
                self._v_acc_m /= 1000.0
            self._pdop = getattr(parsed_msg, 'pDOP', None)
            self._fix_type = getattr(parsed_msg, 'fixType', None)
            self._gnss_fix_ok = bool(getattr(parsed_msg, 'gnssFixOk', 0))
            self._num_sv = getattr(parsed_msg, 'numSV', 0)
            self._host_mono = time.monotonic()
            self._update_count += 1

    def get_opinion(self, max_age_s=30.0):
        """Return a position opinion dict, or None if stale/unavailable.

        The opinion contains all confidence-relevant fields for the
        position confidence framework (see docs/position-confidence.md).
        """
        with self._lock:
            if self._host_mono is None:
                return None
            age = time.monotonic() - self._host_mono
            if age > max_age_s:
                return None
            if self._fix_type not in (2, 3) or self._lat is None:
                return None
            if not self._gnss_fix_ok:
                return None
            lat = self._lat
            lon = self._lon
            h = self._height_m or 0.0
            h_acc = self._h_acc_m
            v_acc = self._v_acc_m
            pdop = self._pdop
            num_sv = self._num_sv
            fix_type = self._fix_type
            n = self._update_count

        # WGS84 LLH -> ECEF
        import math
        a = 6378137.0
        f = 1 / 298.257223563
        e2 = 2 * f - f * f
        lat_r = math.radians(lat)
        lon_r = math.radians(lon)
        sin_lat = math.sin(lat_r)
        cos_lat = math.cos(lat_r)
        N = a / math.sqrt(1 - e2 * sin_lat * sin_lat)
        x = (N + h) * cos_lat * math.cos(lon_r)
        y = (N + h) * cos_lat * math.sin(lon_r)
        z = (N * (1 - e2) + h) * sin_lat

        import numpy as np
        return {
            'source': 'nav2',
            'ecef': np.array([x, y, z]),
            'lat': lat,
            'lon': lon,
            'alt_m': h,
            'h_acc_m': h_acc,
            'v_acc_m': v_acc,
            'pdop': pdop,
            'fix_type': fix_type,
            'num_sv': num_sv,
            'age_s': age,
            'n_updates': n,
        }

    def get_ecef(self, max_age_s=30.0):
        """Return (ecef_xyz, h_acc_m, age_s) or (None, None, None) if stale.

        Legacy interface — prefer get_opinion() for new code.
        """
        opinion = self.get_opinion(max_age_s=max_age_s)
        if opinion is None:
            return None, None, None
        return opinion['ecef'], opinion['h_acc_m'], opinion['age_s']

    def summary(self):
        """One-line status for logging."""
        with self._lock:
            if self._host_mono is None:
                return "NAV2: no data"
            age = time.monotonic() - self._host_mono
            return (f"NAV2: fix={self._fix_type} sv={self._num_sv} "
                    f"hAcc={self._h_acc_m:.1f}m vAcc={self._v_acc_m:.1f}m "
                    f"pDOP={self._pdop:.1f} age={age:.0f}s "
                    f"n={self._update_count}")


def serial_reader(port, baud, obs_queue, stop_event, beph, systems=None,
                   ssr=None, qerr_store=None, config_queue=None, driver=None,
                   raw_callback=None, nav2_store=None):
    """Read UBX messages from a GNSS device.

    Puts (timestamp, observations_list) tuples onto obs_queue for each
    RXM-RAWX epoch. Also feeds RXM-SFRBX to broadcast ephemeris.
    If qerr_store is provided, extracts TIM-TP qErr and stores it.
    If nav2_store is provided, captures NAV2-PVT for position consensus.

    Args:
        systems: set of system names to include (e.g. {'gps', 'gal', 'bds'}).
                 None means all systems.
        ssr: SSRState instance for real-time code bias corrections.
             If provided, biases are applied to raw pseudoranges before
             IF combination (same as OSB in the file-based pipeline).
        qerr_store: QErrStore instance for TIM-TP qErr extraction.
        config_queue: optional queue.Queue of bytes to write to the serial
             port (e.g. UBX CFG-VALSET messages from the main thread).
        driver: ReceiverDriver instance for signal ID mapping.
             Defaults to F9TDriver for backward compatibility.
        raw_callback: optional callable(parsed_msg) called with each
             RXM-RAWX message for raw observation access (e.g. NTRIP caster).
    """
    try:
        from pyubx2 import UBXReader
    except ImportError:
        log.error("pyubx2/pyserial not installed")
        stop_event.set()
        return

    from peppar_fix.gnss_stream import open_gnss

    # Default to F9T for backward compatibility
    if driver is None:
        from peppar_fix.receiver import F9TDriver
        driver = F9TDriver()

    stream, device_type = open_gnss(port, baud)
    log.info(
        f"Opening GNSS device {port} at {baud} baud "
        f"(type: {device_type}, driver: {driver.name})"
    )
    ser = stream
    ubr = UBXReader(ser, protfilter=2)  # UBX only

    # Signal name mapping from receiver driver
    SIG_NAMES = driver.signal_names
    SYS_MAP = driver.sys_map

    pair_config = getattr(driver, 'if_pairs', None) or IF_PAIRS
    sig_lookup = {}
    for gnss_id, sig_f1, sig_f2, prefix in pair_config:
        pair_params = IF_PAIR_PARAMS.get((sig_f1, sig_f2))
        if pair_params is None:
            raise ValueError(f"Unsupported IF pair for {driver.name}: {sig_f1} + {sig_f2}")
        _, a1, a2 = pair_params
        sig_lookup[sig_f1] = (gnss_id, prefix, 'f1', a1, a2, sig_f1)
        sig_lookup[sig_f2] = (gnss_id, prefix, 'f2', a1, a2, sig_f2)

    epoch_data = {}   # sv → {f1: {...}, f2: {...}}
    epoch_ts = None
    n_epochs = 0
    delay_injector = get_delay_injector()
    mute_controller = get_source_mute_controller()
    source_name = f"gnss:{port}"
    last_qerr_invalid_log = 0.0
    # GNSS delivery can legitimately batch by seconds on some hosts
    # (notably the kernel-GNSS path on ocxo), so keep this estimator broad.
    recv_estimator = TimebaseRelationEstimator(
        min_sigma_s=4.0,
        sigma_scale=4.0,
    )
    while not stop_event.is_set():
        # Drain config queue: write pending UBX messages to the receiver
        if config_queue is not None:
            while not config_queue.empty():
                try:
                    cfg_bytes = config_queue.get_nowait()
                    ser.write(cfg_bytes)
                    log.info(f"Config sent to receiver ({len(cfg_bytes)} bytes)")
                except queue.Empty:
                    break

        try:
            raw, parsed = ubr.read()
            if parsed is None:
                continue
            delay_injector.maybe_inject_delay(source_name)
            if mute_controller.should_drop(source_name):
                mute_controller.note_drop(source_name)
                continue

            msg_id = parsed.identity

            # Broadcast ephemeris from SFRBX
            # (We'll rely on NTRIP for ephemeris; SFRBX decoding is complex)

            # TIM-TP: extract PPS quantization error (qErr)
            if msg_id == 'TIM-TP' and qerr_store is not None:
                qerr_ps = getattr(parsed, 'qErr', None)
                tow_ms = getattr(parsed, 'towMS', None)
                # Prefer the decoded qErrInvalid field when pyubx2 exposes it.
                flags = getattr(parsed, 'flags', 0)
                decoded_invalid = getattr(parsed, 'qErrInvalid', None)
                if decoded_invalid is None:
                    qerr_invalid = bool(flags & 0x10) if isinstance(flags, int) else False
                else:
                    qerr_invalid = bool(decoded_invalid)
                if qerr_ps is not None:
                    # Forward to the store unconditionally so the qErr
                    # log captures invalid samples too — invalid ones
                    # are filtered out of the in-memory deque inside
                    # update(), but they're useful for accounting in
                    # post-processing.
                    qerr_store.update(qerr_ps, tow_ms,
                                      qerr_invalid=qerr_invalid)
                if qerr_invalid:
                    now = time.monotonic()
                    if now - last_qerr_invalid_log > 30.0:
                        log.warning(
                            "TIM-TP qErrInvalid=1 on %s; dropping qErr sample",
                            port,
                        )
                        last_qerr_invalid_log = now

            # NAV2-PVT: secondary navigation engine position fix
            if msg_id == 'NAV2-PVT' and nav2_store is not None:
                nav2_store.update(parsed)

            if msg_id == 'RXM-RAWX':
                # Fire raw callback before IF processing (for NTRIP caster)
                if raw_callback is not None:
                    try:
                        raw_callback(parsed)
                    except Exception as e:
                        log.debug(f"raw_callback error: {e}")

                # New RAWX epoch — process and enqueue
                ts = datetime.now(timezone.utc)  # Use wall clock for now
                rcvTow = parsed.rcvTow
                week = parsed.week
                leapS = parsed.leapS
                numMeas = parsed.numMeas

                # Build observation set
                raw_obs = defaultdict(dict)  # sv → role → data
                for i in range(1, numMeas + 1):
                    i2 = f"{i:02d}"
                    gnss_id = getattr(parsed, f'gnssId_{i2}', None)
                    sig_id = getattr(parsed, f'sigId_{i2}', None)
                    sv_id = getattr(parsed, f'svId_{i2}', None)
                    if gnss_id is None or sig_id is None:
                        continue

                    sig_name = SIG_NAMES.get((gnss_id, sig_id))
                    if sig_name is None or sig_name not in sig_lookup:
                        continue

                    _, prefix, role, a1, a2, _ = sig_lookup[sig_name]
                    sv = f"{prefix}{int(sv_id):02d}"

                    # BDS-2 GEO/IGSO exclusion
                    if prefix == 'C' and int(sv_id) < BDS_MIN_PRN:
                        continue

                    pr = getattr(parsed, f'prMes_{i2}', None)
                    cp = getattr(parsed, f'cpMes_{i2}', None)
                    cno = getattr(parsed, f'cno_{i2}', None)
                    lock_ms = getattr(parsed, f'locktime_{i2}', 0)
                    pr_valid = getattr(parsed, f'prValid_{i2}', 0)
                    cp_valid = getattr(parsed, f'cpValid_{i2}', 0)
                    half_cyc = getattr(parsed, f'halfCyc_{i2}', 0)

                    if not pr_valid or pr is None:
                        continue
                    if pr < 1e6 or pr > 4e7:
                        continue

                    # u-blox F9T quirk: BDS-3 modernized signal cpMes is
                    # reported in L1-reference cycles, not native carrier
                    # cycles.  Legacy BDS-2 signals (B1I, B2I) are in
                    # native cycles.  Confirmed 2026-04-19 on MadHat via
                    # GF-DIAG: cp * λ_native of B2a overshoots the code
                    # pseudorange by factor F_L1/F_B2a ≈ 1.339, producing
                    # unphysical 175 m per-epoch GF jumps that cascaded
                    # into 8.4 km nav2Δ on GAL+BDS runs (see
                    # memory/project_bds_gf_phase_units_bug.md).
                    #
                    # Convert to native cycles so every downstream
                    # consumer (GF slip detector, MW wide-lane, IF
                    # ambiguity, integer resolution) sees the physical
                    # quantity it was designed for.
                    if cp is not None and cp_valid and sig_name in _BDS_L1_REF_CYCLES:
                        # cp_native = cp_L1 * (f_native / f_L1)
                        #           = cp_L1 * λ_L1 / λ_native
                        cp *= _LAMBDA_L1 / SIG_WAVELENGTH[sig_name]

                    raw_obs[sv][role] = {
                        'pr': pr, 'cno': cno,
                        'cp': cp if cp_valid else None,
                        'lock_ms': lock_ms or 0.0,
                        'half_cyc': half_cyc,
                        'alpha_f1': a1, 'alpha_f2': a2,
                        'sig_name': sig_name,
                    }

                # Form IF observations
                observations = []
                PREFIX_TO_SYS = {'G': 'gps', 'E': 'gal', 'C': 'bds'}
                for sv, roles in raw_obs.items():
                    prefix = sv[0]
                    sys_name = PREFIX_TO_SYS.get(prefix)

                    # System filter
                    if systems and sys_name not in systems:
                        continue

                    if 'f1' not in roles or 'f2' not in roles:
                        continue
                    f1 = roles['f1']
                    f2 = roles['f2']
                    if f1['cp'] is None or f2['cp'] is None:
                        continue
                    # Half-cycle ambiguity check
                    if not f1['half_cyc'] or not f2['half_cyc']:
                        continue

                    a1 = f1['alpha_f1']
                    a2 = f1['alpha_f2']

                    pr_f1 = f1['pr']
                    pr_f2 = f2['pr']
                    cp_f1 = f1['cp']
                    cp_f2 = f2['cp']

                    # RINEX signal codes for SSR bias lookup
                    rinex_f1 = SIG_TO_RINEX.get(f1['sig_name'])
                    rinex_f2 = SIG_TO_RINEX.get(f2['sig_name'])

                    # Apply SSR code biases before IF combination
                    if ssr is not None and rinex_f1 and rinex_f2:
                        cb_f1 = ssr.get_code_bias(sv, rinex_f1[0])
                        cb_f2 = ssr.get_code_bias(sv, rinex_f2[0])
                        if cb_f1 is not None and cb_f2 is not None:
                            pr_f1 -= cb_f1
                            pr_f2 -= cb_f2
                        # Symmetric diagnostic to phase-bias lookup below:
                        # log once per (sv, f1_sig, f2_sig) whether both
                        # code biases are available.  Atomic application
                        # means a single miss leaves the IF pseudorange
                        # uncorrected — this is the mode that bit L5 hosts
                        # on WHU and is a prime suspect for the ptpmon L2
                        # drift.
                        if not hasattr(ssr, '_cb_lookup_logged'):
                            ssr._cb_lookup_logged = {}
                        # Key by (sv, f1_sig, f2_sig); value = last-logged
                        # (hit_f1, hit_f2, tuple(avail_cb)).  Re-log on any
                        # change so late-arriving bias-mount updates show up
                        # instead of being hidden by the initial MISS.
                        lk_cb = (sv, f1['sig_name'], f2['sig_name'])
                        avail_cb = tuple(sorted(ssr._code_bias.get(sv, {}).keys()))
                        snap = (cb_f1 is not None, cb_f2 is not None, avail_cb)
                        if ssr._cb_lookup_logged.get(lk_cb) != snap:
                            log.info("Code bias lookup: %s f1=%s→%s(%s) "
                                     "f2=%s→%s(%s) avail=%s",
                                     sv, f1['sig_name'], rinex_f1[0],
                                     "HIT" if cb_f1 is not None else "MISS",
                                     f2['sig_name'], rinex_f2[0],
                                     "HIT" if cb_f2 is not None else "MISS",
                                     list(avail_cb))
                            ssr._cb_lookup_logged[lk_cb] = snap

                    # Apply SSR phase biases before IF combination.
                    # Phase biases make float ambiguities integer-valued,
                    # enabling PPP-AR.  Requires a single-AC SSR source
                    # (e.g., CAS SSRA01CAS1).  Combined IGS streams have
                    # no phase biases (bias = 0, no effect).
                    wl_f1 = SIG_WAVELENGTH[f1['sig_name']]
                    wl_f2 = SIG_WAVELENGTH[f2['sig_name']]
                    # AR phase-bias availability: only true when BOTH signals
                    # of the IF combination have matched phase biases in the
                    # active SSR stream(s).  A single-signal match produces
                    # a biased IF ambiguity — the short-term promoter in
                    # ppp_ar.py uses this flag to exclude such SVs from
                    # integer-fix candidacy (they still contribute PR for
                    # geometry).  Defaults False when ssr is absent.
                    ar_phase_bias_ok = False
                    if ssr is not None and rinex_f1 and rinex_f2:
                        # Phase biases are indexed by code signal identifier
                        # in SSR (e.g., 'C1C' not 'L1C') — try both
                        pb_f1 = (ssr.get_phase_bias(sv, rinex_f1[1]) or
                                 ssr.get_phase_bias(sv, rinex_f1[0]))
                        pb_f2 = (ssr.get_phase_bias(sv, rinex_f2[1]) or
                                 ssr.get_phase_bias(sv, rinex_f2[0]))
                        if pb_f1 is not None:
                            cp_f1 -= pb_f1 / wl_f1  # meters → cycles
                        if pb_f2 is not None:
                            cp_f2 -= pb_f2 / wl_f2
                        ar_phase_bias_ok = (pb_f1 is not None
                                            and pb_f2 is not None)
                        # Phase B diagnostic: log lookup misses
                        if not hasattr(ssr, '_pb_lookup_logged'):
                            ssr._pb_lookup_logged = set()
                        lk = (sv, f1['sig_name'], f2['sig_name'])
                        if lk not in ssr._pb_lookup_logged:
                            avail = list(ssr._phase_bias.get(sv, {}).keys())
                            log.info("Phase bias lookup: %s f1=%s→%s(%s) "
                                     "f2=%s→%s(%s) avail=%s",
                                     sv, f1['sig_name'], rinex_f1,
                                     "HIT" if pb_f1 is not None else "MISS",
                                     f2['sig_name'], rinex_f2,
                                     "HIT" if pb_f2 is not None else "MISS",
                                     avail)
                            ssr._pb_lookup_logged.add(lk)

                    pr_if = a1 * pr_f1 - a2 * pr_f2
                    phi_if_m = a1 * wl_f1 * cp_f1 - a2 * wl_f2 * cp_f2

                    # Dual-freq raw-obs diagnostic for the BDS GF blow-up
                    # investigation (2026-04-19).  Logs two consecutive
                    # epochs of raw cp/pr/wl for the FIRST SV of each
                    # constellation, plus a manually-computed GF delta,
                    # so we can see whether the 60–190 m per-epoch GF on
                    # BDS is a unit/wavelength bug or a receiver-side
                    # measurement quirk.  One-shot per (sys, sv) pair.
                    if not hasattr(serial_reader, '_gf_diag'):
                        serial_reader._gf_diag = {}
                    diag = serial_reader._gf_diag
                    key = (sys_name, sv)
                    if key not in diag:
                        diag[key] = {'epoch1': None, 'epoch2': None}
                    slot = diag[key]
                    cp1_raw = f1['cp']
                    cp2_raw = f2['cp']
                    if slot['epoch1'] is None:
                        slot['epoch1'] = (cp1_raw, cp2_raw, wl_f1, wl_f2,
                                          pr_f1, pr_f2, f1['sig_name'],
                                          f2['sig_name'])
                    elif slot['epoch2'] is None:
                        slot['epoch2'] = (cp1_raw, cp2_raw, wl_f1, wl_f2,
                                          pr_f1, pr_f2, f1['sig_name'],
                                          f2['sig_name'])
                        e1 = slot['epoch1']
                        e2 = slot['epoch2']
                        gf1 = e1[0]*e1[2] - e1[1]*e1[3]
                        gf2 = e2[0]*e2[2] - e2[1]*e2[3]
                        d_phi1 = e2[0] - e1[0]
                        d_phi2 = e2[1] - e1[1]
                        d_pr1 = e2[4] - e1[4]
                        d_pr2 = e2[5] - e1[5]
                        log.info(
                            "GF-DIAG %s %s f1=%s(%.4fm) f2=%s(%.4fm) "
                            "cp1=[%.3f→%.3f Δ%.3f cyc] "
                            "cp2=[%.3f→%.3f Δ%.3f cyc] "
                            "pr1_Δ=%.2fm pr2_Δ=%.2fm "
                            "gf1=%.3fm gf2=%.3fm gf_Δ=%.3fm "
                            "expected_gf_Δ_from_phi=%.3fm",
                            sys_name, sv,
                            e1[6], e1[2], e1[7], e1[3],
                            e1[0], e2[0], d_phi1,
                            e1[1], e2[1], d_phi2,
                            d_pr1, d_pr2,
                            gf1, gf2, gf2 - gf1,
                            d_phi1*e1[2] - d_phi2*e1[3])
                    observations.append({
                        'sv': sv,
                        'sys': sys_name,
                        'pr_if': pr_if,
                        'phi_if_m': phi_if_m,
                        'cno': min(f1['cno'], f2['cno']),
                        'lock_duration_ms': min(f1['lock_ms'], f2['lock_ms']),
                        'half_cyc_ok': True,
                        # Per-frequency data for MW wide-lane (PPP-AR).
                        # phi*_cyc is bias-corrected; MW needs that so the
                        # wide-lane integer converges to the right value.
                        'phi1_cyc': cp_f1,
                        'phi2_cyc': cp_f2,
                        # Raw tracking phase BEFORE SSR phase-bias correction.
                        # CycleSlipMonitor's GF detector uses these — SSR
                        # phase-bias integer-indicator updates can step by a
                        # full wavelength, which spoofs gf_jump on bias-
                        # corrected phase (7-SV false-positive slip burst
                        # observed on ptpmon 2026-04-19).
                        'phi1_raw_cyc': f1['cp'],
                        'phi2_raw_cyc': f2['cp'],
                        'pr1_m': pr_f1,
                        'pr2_m': pr_f2,
                        'wl_f1': wl_f1,
                        'wl_f2': wl_f2,
                        # Per-signal lock duration (CycleSlipMonitor attributes
                        # an SV-wide slip to the signal with the lower lock).
                        'f1_lock_ms': f1['lock_ms'],
                        'f2_lock_ms': f2['lock_ms'],
                        'f1_sig_name': f1['sig_name'],
                        'f2_sig_name': f2['sig_name'],
                        'ar_phase_bias_ok': ar_phase_bias_ok,
                    })

                # Diagnostic dump (first 3 epochs, then every 60)
                if n_epochs < 3 or n_epochs % 60 == 0:
                    log.info(f"Serial diag epoch {n_epochs}: "
                             f"raw_obs={len(raw_obs)} SVs, "
                             f"IF_obs={len(observations)} SVs, "
                             f"systems_filter={systems}")
                    for sv, roles in sorted(raw_obs.items()):
                        prefix = sv[0]
                        sys_name = PREFIX_TO_SYS.get(prefix, '?')
                        filtered = systems and sys_name not in systems
                        has_dual = 'f1' in roles and 'f2' in roles
                        f1_pr = roles.get('f1', {}).get('pr', 0)
                        f2_pr = roles.get('f2', {}).get('pr', 0)
                        f1_sig = roles.get('f1', {}).get('sig_name', '?')
                        f2_sig = roles.get('f2', {}).get('sig_name', '?')
                        log.info(f"  {sv} sys={sys_name} "
                                 f"{'FILTERED' if filtered else 'PASS'} "
                                 f"dual={'Y' if has_dual else 'N'} "
                                 f"f1={f1_sig}:{f1_pr:.1f} "
                                 f"f2={f2_sig}:{f2_pr:.1f}")

                if len(observations) >= 4:
                    # Compute GPS time from RAWX header
                    gps_epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
                    gps_time = gps_epoch + timedelta(weeks=week, seconds=rcvTow)

                    recv_mono = None
                    queue_remains = None
                    if hasattr(stream, 'pop_packet_metadata'):
                        recv_mono, queue_remains = stream.pop_packet_metadata()
                    elif hasattr(stream, 'pop_packet_timestamp'):
                        recv_mono = stream.pop_packet_timestamp()
                    now_mono = time.monotonic()
                    if recv_mono is None:
                        recv_mono = now_mono
                    if queue_remains is None:
                        queue_remains = bool(getattr(stream, 'in_waiting', 0))
                    recv_utc = datetime.now(timezone.utc)
                    parse_age_s = max(0.0, now_mono - recv_mono)
                    base_confidence = estimate_correlation_confidence(
                        queue_remains=queue_remains,
                        parse_age_s=parse_age_s,
                    )
                    estimator_sample = recv_estimator.update(
                        gps_time.timestamp(),
                        recv_mono,
                        sample_weight=estimator_sample_weight(
                            queue_remains=queue_remains,
                            base_confidence=base_confidence,
                        ),
                    )
                    confidence = max(
                        0.05,
                        min(1.0, base_confidence * estimator_sample["confidence"]),
                    )
                    obs_queue.put(ObservationEvent(
                        gps_time=gps_time,
                        observations=observations,
                        recv_mono=recv_mono,
                        recv_utc=recv_utc,
                        queue_remains=queue_remains,
                        parse_age_s=parse_age_s,
                        correlation_confidence=confidence,
                        estimator_residual_s=estimator_sample["residual_s"],
                    ))
                    n_epochs += 1

                    if n_epochs % 60 == 0:
                        log.info(f"Serial: {n_epochs} epochs, "
                                 f"last had {len(observations)} SVs")

        except Exception as e:
            if not stop_event.is_set():
                log.error(f"Serial reader error: {e}")
            break

    ser.close()
    log.info(f"Serial reader stopped after {n_epochs} epochs")


# ── NTRIP correction reader ────────────────────────────────────────────────── #

def ntrip_reader(stream, beph, ssr, stop_event, label="NTRIP",
                 bias_only=False):
    """Read RTCM3 messages from an NtripStream.

    Routes broadcast ephemeris messages to BroadcastEphemeris and SSR
    messages to SSRState.

    bias_only: when True, forward ONLY observable-specific bias messages
    (BIAS_MSG_TYPES, code + phase biases) to SSRState, ignoring
    orbit/clock/ephemeris.  This is the dual-mount fusion path: pull
    biases from an AC that publishes OSBs matched to F9T signals (e.g.
    WHU OSBC00WHU1 covers C5Q/L5Q which CNES does not) while orbit/
    clock come from the primary mount (e.g. CNES SSRA00CNE0).

    Both code AND phase biases must come from the same AC here — a
    mix of CNES code biases with WHU phase biases leaves some F9T
    observables uncorrected (CNES has no C5Q code bias) and caused
    tens-of-metres filter drift in 2026-04-18 testing.  See
    docs/ssr-mount-survey.md.
    """
    msg_counts = defaultdict(int)
    n_skipped_non_phase_bias = 0
    n_total = 0
    delay_injector = get_delay_injector()
    mute_controller = get_source_mute_controller()
    source_name = f"ntrip:{label}"

    try:
        for msg, meta in stream.messages_with_metadata():
            if stop_event.is_set():
                break
            delay_injector.maybe_inject_delay(source_name)
            if mute_controller.should_drop(source_name):
                mute_controller.note_drop(source_name)
                continue

            identity = str(getattr(msg, 'identity', ''))
            event = RtcmEvent(
                identity=identity,
                message=None,
                recv_mono=meta["recv_mono"],
                recv_utc=datetime.now(timezone.utc),
                queue_remains=meta["queue_remains"],
                parse_age_s=meta["parse_age_s"],
                correlation_confidence=meta["correlation_confidence"],
                estimator_residual_s=meta.get("estimator_residual_s"),
            )
            msg_view = RtcmMessageView(msg, event)
            msg_counts[identity] += 1
            n_total += 1

            # Log first occurrence of each message type for debugging
            if n_total <= 3 or identity not in msg_counts or msg_counts[identity] <= 1:
                log.debug(f"[{label}] msg #{n_total}: identity={identity}")

            # Route to appropriate handler.  In bias_only mode, reject
            # everything except the bias message subset — the primary
            # mount owns orbit/clock/ephemeris.
            if bias_only:
                if identity in BIAS_MSG_TYPES:
                    result = ssr.update_from_rtcm(msg_view)
                    if n_total <= 5:
                        log.info(f"[{label}] bias routed: "
                                 f"{identity} → {result}")
                else:
                    n_skipped_non_phase_bias += 1
            elif identity in EPH_MSG_TYPES:
                prn = beph.update_from_rtcm(msg_view)
                if prn and beph.n_satellites % 10 == 0:
                    log.debug(f"[{label}] {beph.summary()}")
            elif identity in SSR_MSG_TYPES or identity.startswith('4076_'):
                result = ssr.update_from_rtcm(msg_view)
                if n_total <= 5:
                    log.info(f"[{label}] SSR routed: {identity} → {result}")

            if n_total % 100 == 0:
                suffix = (f" | skipped {n_skipped_non_phase_bias} non-bias"
                          if bias_only else "")
                log.info(f"[{label}] {n_total} msgs | {beph.summary()} | "
                         f"{ssr.summary()}{suffix}")

    except Exception as e:
        if not stop_event.is_set():
            log.error(f"[{label}] Error: {e}")

    stream.disconnect()
    log.info(f"[{label}] Stopped. Total: {n_total} msgs. "
             f"Types: {dict(msg_counts)}")


# ── File-based replay mode ──────────────────────────────────────────────────── #

def run_replay(args):
    """Run in file replay mode using SP3/CLK (same as M3, validates streaming)."""
    log.info("=== Replay mode (file-based corrections) ===")

    lat, lon, alt = [float(v) for v in args.known_pos.split(',')]
    known_ecef = lla_to_ecef(lat, lon, alt)
    leap_delta = timedelta(seconds=args.leap)

    sp3 = SP3(args.sp3)
    log.info(f"SP3: {len(sp3.epochs)} epochs, {len(sp3.positions)} sats")

    clk_file = CLKFile(args.clk) if args.clk else None
    if clk_file:
        log.info(f"CLK: {len(clk_file.prns())} satellites")

    osb = OSBParser(args.osb) if args.osb else None
    if osb:
        log.info(f"OSB: {len(osb.prns())} satellites")

    systems = set(args.systems.split(','))
    epochs = load_ppp_epochs(args.replay, systems=systems, osb=osb)
    log.info(f"Loaded {len(epochs)} epochs")

    filt = FixedPosFilter(known_ecef)
    filt.prev_clock = 0.0

    out_f = None
    out_w = None
    if args.out:
        out_f = open(args.out, 'w', newline='')
        out_w = csv.writer(out_f)
        out_w.writerow(['timestamp', 'clock_ns', 'clock_sigma_ns',
                        'n_meas', 'n_td', 'rms_m', 'correction_source'])

    prev_t = None
    results = []
    for i, (ts_str, obs) in enumerate(epochs):
        t = timestamp_to_gpstime(ts_str) + leap_delta

        if prev_t is not None:
            dt = (t - prev_t).total_seconds()
            filt.predict(dt)
        prev_t = t

        n_used, resid, n_td = filt.update(obs, sp3, t, clk_file=clk_file)

        clk_ns = filt.x[filt.IDX_CLK] / C * 1e9
        clk_sigma = math.sqrt(filt.P[filt.IDX_CLK, filt.IDX_CLK]) / C * 1e9
        rms = np.sqrt(np.mean(resid ** 2)) if len(resid) > 0 else 0

        results.append((ts_str, clk_ns, clk_sigma, n_used, n_td, rms))

        if out_w:
            out_w.writerow([ts_str, f'{clk_ns:.3f}', f'{clk_sigma:.4f}',
                            n_used, n_td, f'{rms:.4f}', 'SP3+CLK'])

        if (i + 1) % 60 == 0:
            log.info(f"  [{i+1}/{len(epochs)}] {ts_str[:19]} "
                     f"clk={clk_ns:.1f}ns ±{clk_sigma:.2f}ns "
                     f"n={n_used} td={n_td} rms={rms:.3f}m")

    if out_f:
        out_f.close()

    # Summary
    clks = np.array([r[1] for r in results])
    log.info(f"\n{'='*60}")
    log.info(f"  Replay complete: {len(results)} epochs")
    log.info(f"  Clock: {np.mean(clks):.1f} ± {np.std(clks):.2f} ns")
    if len(clks) > 100:
        dt_s = np.arange(len(clks))
        detrended = clks - np.polyval(np.polyfit(dt_s, clks, 1), dt_s)
        log.info(f"  Detrended std: {np.std(detrended):.3f} ns")
    log.info(f"{'='*60}")


# ── Real-time mode ──────────────────────────────────────────────────────────── #

def run_realtime(args):
    """Run in real-time mode with serial u-blox receiver + NTRIP corrections."""
    from peppar_fix.receiver import get_driver
    driver = get_driver(args.receiver)
    log.info(f"=== Real-time mode ({driver.name}) ===")

    lat, lon, alt = [float(v) for v in args.known_pos.split(',')]
    known_ecef = lla_to_ecef(lat, lon, alt)
    log.info(f"Known position: {lat:.6f}, {lon:.6f}, {alt:.1f}m")

    # Shared state
    beph = BroadcastEphemeris()
    ssr = SSRState()
    corrections = RealtimeCorrections(beph, ssr)
    obs_queue = queue.Queue(maxsize=100)
    stop_event = threading.Event()

    # Start NTRIP threads
    ntrip_threads = []

    use_tls = getattr(args, 'tls', False) or args.port == 443

    if args.eph_mount:
        eph_stream = NtripStream(
            caster=args.caster, port=args.port,
            mountpoint=args.eph_mount,
            user=args.user, password=args.password,
            tls=use_tls,
        )
        t_eph = threading.Thread(
            target=ntrip_reader,
            args=(eph_stream, beph, ssr, stop_event, "EPH"),
            daemon=True,
        )
        t_eph.start()
        ntrip_threads.append(t_eph)
        log.info(f"Ephemeris stream: {args.caster}:{args.port}/{args.eph_mount}")

    if args.ssr_mount:
        ssr_stream = NtripStream(
            caster=args.caster, port=args.port,
            mountpoint=args.ssr_mount,
            user=args.user, password=args.password,
            tls=use_tls,
        )
        t_ssr = threading.Thread(
            target=ntrip_reader,
            args=(ssr_stream, beph, ssr, stop_event, "SSR"),
            daemon=True,
        )
        t_ssr.start()
        ntrip_threads.append(t_ssr)
        log.info(f"SSR stream: {args.caster}:{args.port}/{args.ssr_mount}")

    # Wait for initial ephemeris before starting serial
    if args.eph_mount:
        log.info("Waiting for broadcast ephemeris...")
        warmup_start = time.time()
        while beph.n_satellites < 8 and time.time() - warmup_start < 120:
            time.sleep(1)
            if int(time.time() - warmup_start) % 10 == 0:
                log.info(f"  Warmup: {beph.summary()}")
        log.info(f"Warmup complete: {beph.summary()}")

    # Parse systems filter
    systems = set(args.systems.split(',')) if args.systems else None
    log.info(f"Systems filter: {systems}")

    # Start serial reader
    t_serial = threading.Thread(
        target=serial_reader,
        args=(args.serial, args.baud, obs_queue, stop_event, beph, systems, ssr),
        kwargs={'driver': driver},
        daemon=True,
    )
    t_serial.start()
    log.info(f"Serial: {args.serial} at {args.baud} baud")

    # Initialize filter
    filt = FixedPosFilter(known_ecef)
    filt.prev_clock = 0.0

    out_f = None
    out_w = None
    if args.out:
        out_f = open(args.out, 'w', newline='')
        out_w = csv.writer(out_f)
        out_w.writerow(['timestamp', 'clock_ns', 'clock_sigma_ns',
                        'n_meas', 'n_td', 'rms_m', 'correction_source',
                        'n_ssr_orbit', 'n_ssr_clock', 'n_beph'])

    # Main processing loop
    prev_t = None
    n_epochs = 0
    start_time = time.time()
    try:
        while not stop_event.is_set():
            # Check duration limit
            if args.duration and (time.time() - start_time) > args.duration:
                log.info(f"Duration limit reached ({args.duration}s)")
                break

            try:
                gps_time, observations = obs_queue.get(timeout=5)
            except queue.Empty:
                continue

            # EKF predict
            if prev_t is not None:
                dt = (gps_time - prev_t).total_seconds()
                if dt <= 0 or dt > 30:
                    log.warning(f"Suspicious dt={dt:.1f}s, skipping")
                    prev_t = gps_time
                    continue
                filt.predict(dt)
            prev_t = gps_time

            # EKF update — use RealtimeCorrections (SP3-compatible interface)
            n_used, resid, n_td = filt.update(
                observations, corrections, gps_time,
                clk_file=corrections,  # RealtimeCorrections implements sat_clock()
            )

            clk_ns = filt.x[filt.IDX_CLK] / C * 1e9
            clk_sigma = math.sqrt(filt.P[filt.IDX_CLK, filt.IDX_CLK]) / C * 1e9
            rms = np.sqrt(np.mean(resid ** 2)) if len(resid) > 0 else 0
            n_epochs += 1

            # Determine correction source
            source = 'broadcast'
            if ssr.n_clock > 0:
                source = 'SSR'

            ts_str = gps_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:23]

            if out_w:
                out_w.writerow([
                    ts_str, f'{clk_ns:.3f}', f'{clk_sigma:.4f}',
                    n_used, n_td, f'{rms:.4f}', source,
                    ssr.n_orbit, ssr.n_clock, beph.n_satellites,
                ])

            # Console status every 10 epochs
            if n_epochs % 10 == 0:
                elapsed = time.time() - start_time
                log.info(
                    f"  [{n_epochs}] {ts_str[:19]} "
                    f"clk={clk_ns:+.1f}ns ±{clk_sigma:.2f}ns "
                    f"n={n_used} td={n_td} rms={rms:.3f}m "
                    f"[{source}] "
                    f"beph={beph.n_satellites} ssr_clk={ssr.n_clock}"
                )

    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        stop_event.set()
        if out_f:
            out_f.close()

    elapsed = time.time() - start_time
    log.info(f"\n{'='*60}")
    log.info(f"  Real-time PPP complete")
    log.info(f"  Duration: {elapsed:.0f}s, Epochs: {n_epochs}")
    log.info(f"  {corrections.summary()}")
    log.info(f"{'='*60}")


# ── Main ──────────────────────────────────────────────────────────────────── #

def main():
    ap = argparse.ArgumentParser(
        description="Real-time PPP clock estimation (PePPAR Fix M4)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Position
    ap.add_argument("--known-pos", required=True,
                    help="Known position as lat,lon,alt (e.g. 'LAT,LON,ALT' — decimal degrees and meters)")
    ap.add_argument("--leap", type=int, default=18,
                    help="UTC-GPS leap seconds (default: 18)")
    ap.add_argument("--systems", default="gps,gal,bds",
                    help="GNSS systems to use (default: gps,gal,bds)")

    # Serial (real-time mode)
    serial = ap.add_argument_group("Serial (real-time)")
    serial.add_argument("--serial", help="Serial port (e.g. /dev/gnss-bot)")
    serial.add_argument("--receiver", default="f9t",
                        help="Receiver model: f9t, f10t (default: f9t)")
    serial.add_argument("--baud", type=int, default=115200)

    # NTRIP (real-time mode)
    ntrip = ap.add_argument_group("NTRIP corrections")
    ntrip.add_argument("--ntrip-conf", help="NTRIP config file (INI format)")
    ntrip.add_argument("--caster", help="NTRIP caster hostname")
    ntrip.add_argument("--port", type=int, default=2101)
    ntrip.add_argument("--tls", action="store_true", help="Use TLS (auto for port 443)")
    ntrip.add_argument("--eph-mount", help="Mountpoint for broadcast ephemeris")
    ntrip.add_argument("--ssr-mount", help="Mountpoint for SSR corrections")
    ntrip.add_argument("--user", help="NTRIP username")
    ntrip.add_argument("--password", help="NTRIP password")

    # Replay (file-based mode)
    replay = ap.add_argument_group("Replay (file-based)")
    replay.add_argument("--replay", help="RAWX CSV file for replay mode")
    replay.add_argument("--sp3", help="SP3 orbit file (replay mode)")
    replay.add_argument("--clk", help="RINEX CLK file (replay mode)")
    replay.add_argument("--osb", help="SINEX BIAS file (replay mode)")

    # Output
    ap.add_argument("--out", help="Output CSV file")
    ap.add_argument("--duration", type=int, default=0,
                    help="Duration in seconds (0 = unlimited)")
    ap.add_argument("-v", "--verbose", action="store_true")

    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        stream=sys.stderr,
    )

    # Load NTRIP config file if specified
    if args.ntrip_conf:
        import configparser
        conf = configparser.ConfigParser()
        conf.read(args.ntrip_conf)
        if 'ntrip' in conf:
            s = conf['ntrip']
            if not args.caster:
                args.caster = s.get('caster', args.caster)
            if args.port == 2101 and s.get('port'):
                args.port = int(s.get('port'))
            if not args.user:
                args.user = s.get('user', args.user)
            if not args.password:
                args.password = s.get('password', args.password)
            if not args.tls and s.getboolean('tls', False):
                args.tls = True
            if not args.ssr_mount and s.get('mount'):
                args.ssr_mount = s.get('mount')

    if args.replay:
        if not args.sp3:
            ap.error("--replay requires --sp3")
        run_replay(args)
    elif args.serial:
        if not args.caster and not args.eph_mount:
            log.warning("No NTRIP source — will use broadcast ephemeris from receiver only")
        run_realtime(args)
    else:
        ap.error("Must specify either --serial (real-time) or --replay (file-based)")


if __name__ == "__main__":
    main()

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
#
# Code-vs-phase split is exposed separately so cross-AC diagnostic flags
# (--no-ssr-code-bias, --no-ssr-phase-bias) can drop one class while keeping
# the other.  See docs/ssr-cross-ac-diagnostic-2026-04-25.md.
CODE_BIAS_MSG_TYPES = {
    '1059', '1065', '1242', '1264',
    '4076_025', '4076_065', '4076_105',
}
PHASE_BIAS_MSG_TYPES = {
    '1067',
    '1265', '1266', '1267', '1268', '1269', '1270',
    '4076_026', '4076_066', '4076_106',
}
BIAS_MSG_TYPES = CODE_BIAS_MSG_TYPES | PHASE_BIAS_MSG_TYPES


# Some u-blox firmware variants report BDS-3 modernized-signal cpMes
# in L1-reference cycles (i.e. the reported cycle count, when scaled by
# λ_L1, equals the geometric path).  To obtain native-carrier cycles,
# multiply the reported cpMes by (λ_L1 / λ_native) = F_native/F_L1.
#
# This is per-driver because the quirk is firmware-specific:
#   - F9T TIM 2.25: BDS-B2aI, BDS-B2aQ in L1-ref cycles (lab-confirmed
#     MadHat 2026-04-19).  Legacy BDS-2 signals are in native cycles.
#   - F10T TIM 3.01: native cycles for everything (lab-confirmed
#     clkPoC3 2026-05-05 via the GPS+GAL-vs-GPS+GAL+BDS A/B —
#     applying the F9T correction caused 25%-class carrier-phase bias
#     that compounded into ZTD blow-up of >50 m within a minute).
#
# The active set comes from driver.bds_l1_ref_cycles; this λ_L1 lookup
# stays in module scope because the correction math doesn't change.
_LAMBDA_L1 = C / F_L1

# B1C sits at the L1 carrier (1575.42 MHz); B2a sits at the L5 carrier
# (1176.45 MHz).  Wavelengths and IF coefficients reuse the GPS L1/L5
# constants exactly — the carriers are identical, only the modulation
# differs.
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
    'BDS-B1CP': C / F_L1,
    'BDS-B1CD': C / F_L1,
    'BDS-B2aP': C / F_L5,
    'BDS-B2aD': C / F_L5,
}

IF_PAIR_PARAMS = {
    ('GPS-L1CA', 'GPS-L2CL'): ('G', ALPHA_L1_L2, ALPHA_L2),
    ('GPS-L1CA', 'GPS-L5Q'): ('G', ALPHA_L1, ALPHA_L5),
    ('GAL-E1C', 'GAL-E5bQ'): ('E', ALPHA_E1, ALPHA_E5B),
    ('GAL-E1C', 'GAL-E5aQ'): ('E', ALPHA_L1, ALPHA_L5),
    ('BDS-B1I', 'BDS-B2I'): ('C', ALPHA_B1I_B2I, ALPHA_B2I),
    ('BDS-B1I', 'BDS-B2aI'): ('C', ALPHA_B1I, ALPHA_B2A),
    ('BDS-B1CP', 'BDS-B2aP'): ('C', ALPHA_L1, ALPHA_L5),
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

    def drain_since(self, host_time_floor):
        """Return [(host_time, qerr_ns), ...] for every retained sample
        whose host_time is strictly greater than ``host_time_floor``,
        in monotonic order.

        Used by RxTcxoTracker.update_from_qerr_store to walk the raw
        TIM-TP stream sample-by-sample for slope LS — replacing the
        per-PPS-edge match_pps_mono path that was vulnerable to FIFO
        sequence breaks across PPS gaps (see I-151540 Item 7).

        The store is bounded (maxlen samples); a consumer that lags
        beyond that horizon will see only the most-recent maxlen
        samples on its next call.  At default maxlen=128 and
        F9T-typical 1 Hz TIM-TP, that's ~2 minutes of headroom —
        plenty for any reasonable servo cadence.

        Caller maintains its own floor (typically the host_time of
        the last sample it consumed) and passes it on each call.
        ``host_time_floor=0.0`` returns every retained sample.
        """
        with self._lock:
            return [
                (s["host_time"], s["qerr_ns"])
                for s in self._samples
                if s["host_time"] > host_time_floor
            ]

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


# ── UBX-NAV-SIG: receiver's per-signal usage verdict ────────────────────── #
#
# F9T / F10T publish per-(SV, signal) usage flags via UBX-NAV-SIG.  prUsed
# is the receiver's own decision about whether to include that signal's
# pseudorange in its NAV2 solution.  Cascade-#6 evidence (day0511,
# clkPoC3): F10T NAV2 hAcc=0.74m stayed unchanged through a 21-ms chip-
# slip — the receiver excluded the bad SV via NAV-SIG.prUsed=0 while we
# kept admitting it.  See slipDetectUnified-main.
#
# This store mirrors Nav2PositionStore's pattern: thread-safe latest
# snapshot, polling getters, optional on_transition callbacks for the
# disagreement-instrumentation logger.
#
# sigFlags bit layout (per UBX-NAV-SIG spec):
#   bits 0-1: health (0=unknown, 1=healthy, 2=unhealthy)
#   bit 2:    prSmoothed
#   bit 3:    prUsed     ← KEY: used in NAV2 solution
#   bit 4:    crUsed
#   bit 5:    doUsed
#   bit 6:    prCorrUsed (DGNSS / RTCM)
#   bit 7:    crCorrUsed
#   bit 8:    doCorrUsed
_SIG_FLAG_HEALTH_MASK = 0x0003
_SIG_FLAG_PR_SMOOTHED = 0x0004
_SIG_FLAG_PR_USED     = 0x0008
_SIG_FLAG_CR_USED     = 0x0010
_SIG_FLAG_DO_USED     = 0x0020


class SigStatus:
    """Per-(SV, sigId) snapshot from a single NAV-SIG message.

    Lightweight value type — read-only after construction.  All fields
    optional because pyubx2 occasionally omits fields the receiver
    didn't populate.
    """
    __slots__ = ('gnss_id', 'sv_id', 'sig_id', 'sig_name',
                 'pr_used', 'cr_used', 'do_used',
                 'pr_smoothed', 'health', 'cno',
                 'quality_ind', 'pr_res_m', 'host_mono')

    def __init__(self, gnss_id, sv_id, sig_id, sig_name,
                  pr_used, cr_used, do_used,
                  pr_smoothed, health, cno,
                  quality_ind, pr_res_m, host_mono):
        self.gnss_id = gnss_id
        self.sv_id = sv_id
        self.sig_id = sig_id
        self.sig_name = sig_name
        self.pr_used = pr_used
        self.cr_used = cr_used
        self.do_used = do_used
        self.pr_smoothed = pr_smoothed
        self.health = health
        self.cno = cno
        self.quality_ind = quality_ind
        self.pr_res_m = pr_res_m
        self.host_mono = host_mono


class Nav2SignalStore:
    """Thread-safe latest per-(SV, sigId) usage verdict from the receiver.

    Updated on each UBX-NAV-SIG message.  Consumers:
      - polling: ``get(sv, sig_name)`` → SigStatus or None.
      - bulk:    ``snapshot()`` → dict {(sv, sig_name): SigStatus}.
      - on_transition: register a callback that fires when prUsed flips
        for a (SV, sigId).  Charlie's Phase A.5 disagreement logger
        consumes this — emits structured [NAV-SIG_DISAGREE] lines.

    SVs not in the latest NAV-SIG message are NOT removed automatically
    — callers can prune via ``mtime`` on the SigStatus host_mono field.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._by_key = {}  # (sv_label, sig_name) → SigStatus
        self._transition_cbs = []  # list of (callback, prev_state) pairs
        self._epoch = 0  # incremented per NAV-SIG message
        self._update_count = 0
        self._signal_names = None  # lazy-set by serial_reader from driver

    def set_signal_names(self, signal_names):
        """Driver-supplied (gnssId, sigId) → human-readable name map."""
        self._signal_names = signal_names

    def update(self, parsed_msg):
        """Store a freshly-decoded UBX-NAV-SIG message.

        Iterates per-signal entries and updates internal map.  Returns
        a list of (sv_label, sig_name, prev_pr_used, new_pr_used)
        tuples for prUsed transitions in this epoch — consumers (e.g.,
        Charlie's disagreement logger) can use this directly, or
        register on_transition callbacks that fire on each transition.
        """
        signal_names = self._signal_names or {}
        num_sigs = getattr(parsed_msg, 'numSigs', 0) or 0
        host_mono = time.monotonic()
        transitions = []  # (sv_label, sig_name, prev_pr_used, new_pr_used)
        epoch_snapshot = {}  # (sv_label, sig_name) → SigStatus

        for i in range(1, num_sigs + 1):
            i2 = f"{i:02d}"
            gnss_id = getattr(parsed_msg, f'gnssId_{i2}', None)
            sv_id   = getattr(parsed_msg, f'svId_{i2}',   None)
            sig_id  = getattr(parsed_msg, f'sigId_{i2}',  None)
            if gnss_id is None or sv_id is None or sig_id is None:
                continue
            sig_flags = getattr(parsed_msg, f'sigFlags_{i2}', 0) or 0
            cno       = getattr(parsed_msg, f'cno_{i2}',       None)
            qual      = getattr(parsed_msg, f'qualityInd_{i2}', None)
            pr_res    = getattr(parsed_msg, f'prRes_{i2}',     None)
            # prRes is signed 16-bit in units of 0.1 m per the UBX spec.
            # pyubx2 may already decode the scaling; we apply the 0.1
            # conversion defensively if the value looks like raw ticks.
            pr_res_m = None
            if pr_res is not None:
                # Heuristic: if |pr_res| > 100 we assume raw 0.1m units.
                pr_res_m = (pr_res * 0.1
                             if abs(pr_res) > 100
                             else float(pr_res))

            sig_name = signal_names.get((gnss_id, sig_id))
            if sig_name is None:
                sig_name = f"sys{gnss_id}/sig{sig_id}"

            sv_label = _sv_label(gnss_id, sv_id)
            status = SigStatus(
                gnss_id=gnss_id, sv_id=sv_id, sig_id=sig_id,
                sig_name=sig_name,
                pr_used=bool(sig_flags & _SIG_FLAG_PR_USED),
                cr_used=bool(sig_flags & _SIG_FLAG_CR_USED),
                do_used=bool(sig_flags & _SIG_FLAG_DO_USED),
                pr_smoothed=bool(sig_flags & _SIG_FLAG_PR_SMOOTHED),
                health=int(sig_flags & _SIG_FLAG_HEALTH_MASK),
                cno=cno, quality_ind=qual,
                pr_res_m=pr_res_m, host_mono=host_mono,
            )
            epoch_snapshot[(sv_label, sig_name)] = status

        with self._lock:
            # Compare against prior state to detect prUsed transitions.
            for key, new_st in epoch_snapshot.items():
                prev = self._by_key.get(key)
                if prev is not None and prev.pr_used != new_st.pr_used:
                    transitions.append(
                        (key[0], key[1], prev.pr_used, new_st.pr_used)
                    )
            self._by_key.update(epoch_snapshot)
            self._epoch += 1
            self._update_count += 1
            cbs = list(self._transition_cbs)
            epoch = self._epoch

        # Fire callbacks OUTSIDE the lock so a slow consumer can't
        # block other readers.  Errors per-callback don't fail others.
        for cb in cbs:
            for sv_label, sig_name, prev_pu, new_pu in transitions:
                try:
                    cb(sv_label, sig_name, prev_pu, new_pu,
                       epoch_snapshot[(sv_label, sig_name)])
                except Exception as e:
                    log.debug("NAV-SIG transition callback failed: %s", e)

        return transitions

    def get(self, sv_label, sig_name):
        """Return latest SigStatus for (sv_label, sig_name) or None."""
        with self._lock:
            return self._by_key.get((sv_label, sig_name))

    def snapshot(self):
        """Return shallow copy of the full map."""
        with self._lock:
            return dict(self._by_key)

    def on_transition(self, callback):
        """Register callback fired on each prUsed transition.

        Signature: cb(sv_label, sig_name, prev_pr_used, new_pr_used, status).
        """
        with self._lock:
            self._transition_cbs.append(callback)

    def epoch(self):
        with self._lock:
            return self._epoch


# ── UBX-NAV-CLOCK + NAV-TIMEGPS: receiver clock-state telemetry ──────── #
#
# Added for f9tClockTelemetry-bravo (recoveryRetry-main work item 2):
# diagnostic visibility into the receiver's own clock state during
# cascade events.  See I-115943-main / recoveryRetry-main thread for
# the chip-slip vs command-envelope hypotheses these telemetry streams
# help discriminate.

class NavClockStore:
    """Thread-safe latest UBX-NAV-CLOCK snapshot.

    Fields: clkB (clock bias, ns), clkD (clock drift, ns/s),
    tAcc (time accuracy, ns), fAcc (frequency accuracy, ps/s),
    iTOW (GPS time of week, ms).
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._clk_b_ns = None
        self._clk_d_ns_per_s = None
        self._t_acc_ns = None
        self._f_acc_ps_per_s = None
        self._itow_ms = None
        self._host_mono = None
        self._update_count = 0

    def update(self, parsed_msg):
        with self._lock:
            self._clk_b_ns = getattr(parsed_msg, 'clkB', None)
            self._clk_d_ns_per_s = getattr(parsed_msg, 'clkD', None)
            self._t_acc_ns = getattr(parsed_msg, 'tAcc', None)
            self._f_acc_ps_per_s = getattr(parsed_msg, 'fAcc', None)
            self._itow_ms = getattr(parsed_msg, 'iTOW', None)
            self._host_mono = time.monotonic()
            self._update_count += 1

    def get(self):
        with self._lock:
            if self._host_mono is None:
                return None
            return {
                'clk_b_ns': self._clk_b_ns,
                'clk_d_ns_per_s': self._clk_d_ns_per_s,
                't_acc_ns': self._t_acc_ns,
                'f_acc_ps_per_s': self._f_acc_ps_per_s,
                'itow_ms': self._itow_ms,
                'host_mono': self._host_mono,
                'age_s': time.monotonic() - self._host_mono,
                'n_updates': self._update_count,
            }


class NavTimeGpsStore:
    """Thread-safe latest UBX-NAV-TIMEGPS snapshot.

    Fields: iTOW (ms), fTOW (ns sub-ms), week, leapS, tAcc, valid flags.
    Useful for: detecting receiver-side time-discontinuity events that
    might correlate with the clkPoC3 / MadHat catastrophic cascades.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._itow_ms = None
        self._ftow_ns = None
        self._week = None
        self._leap_s = None
        self._t_acc_ns = None
        self._valid_tow = False
        self._valid_week = False
        self._valid_leap_s = False
        self._host_mono = None
        self._update_count = 0

    def update(self, parsed_msg):
        with self._lock:
            self._itow_ms = getattr(parsed_msg, 'iTOW', None)
            self._ftow_ns = getattr(parsed_msg, 'fTOW', None)
            self._week = getattr(parsed_msg, 'week', None)
            self._leap_s = getattr(parsed_msg, 'leapS', None)
            self._t_acc_ns = getattr(parsed_msg, 'tAcc', None)
            valid = getattr(parsed_msg, 'valid', 0) or 0
            self._valid_tow = bool(valid & 0x01)
            self._valid_week = bool(valid & 0x02)
            self._valid_leap_s = bool(valid & 0x04)
            self._host_mono = time.monotonic()
            self._update_count += 1

    def get(self):
        with self._lock:
            if self._host_mono is None:
                return None
            return {
                'itow_ms': self._itow_ms,
                'ftow_ns': self._ftow_ns,
                'week': self._week,
                'leap_s': self._leap_s,
                't_acc_ns': self._t_acc_ns,
                'valid_tow': self._valid_tow,
                'valid_week': self._valid_week,
                'valid_leap_s': self._valid_leap_s,
                'host_mono': self._host_mono,
                'age_s': time.monotonic() - self._host_mono,
                'n_updates': self._update_count,
            }


def _sv_label(gnss_id, sv_id):
    """GNSS id + SV id → 'G07', 'E19', 'C42', etc."""
    prefix = {0: 'G', 1: 'S', 2: 'E', 3: 'C', 5: 'J', 6: 'R'}.get(gnss_id)
    if prefix is None:
        return f"sys{gnss_id}_sv{sv_id}"
    return f"{prefix}{int(sv_id):02d}"


def _emit_nav_sig_log(store, parsed_msg):
    """Emit a structured per-epoch [NAV-SIG] log line.

    Format (one signal per line, multi-line on epochs >1 signal):
      [NAV-SIG ep=N] G07/L1CA: prUsed=1 prRes=+0.3m cno=44 health=1
                     E19/E5aQ: prUsed=0 prRes=+412.7m cno=39 health=1
    """
    epoch = store.epoch()
    snapshot = store.snapshot()
    if not snapshot:
        return
    # Sort for stable output order: by sv_label then sig_name.
    keys = sorted(snapshot.keys())
    header = f"[NAV-SIG ep={epoch}]"
    pad = ' ' * (len(header) + 1)
    for i, key in enumerate(keys):
        st = snapshot[key]
        sv_label, sig_name = key
        pr_res_str = (f"{st.pr_res_m:+.1f}m"
                      if st.pr_res_m is not None else "n/a")
        cno_str = f"{st.cno}" if st.cno is not None else "n/a"
        prefix = header if i == 0 else pad
        log.info("%s %s/%s: prUsed=%d crUsed=%d doUsed=%d "
                 "prRes=%s cno=%s health=%d",
                 prefix, sv_label, sig_name,
                 int(st.pr_used), int(st.cr_used), int(st.do_used),
                 pr_res_str, cno_str, st.health)


def _emit_nav_clock_log(parsed_msg):
    """Emit a per-epoch [NAV-CLOCK] log line."""
    clk_b  = getattr(parsed_msg, 'clkB', None)
    clk_d  = getattr(parsed_msg, 'clkD', None)
    t_acc  = getattr(parsed_msg, 'tAcc', None)
    f_acc  = getattr(parsed_msg, 'fAcc', None)
    itow   = getattr(parsed_msg, 'iTOW', None)
    log.info("[NAV-CLOCK] iTOW=%s clkB=%sns clkD=%sns/s tAcc=%sns fAcc=%sps/s",
             itow, clk_b, clk_d, t_acc, f_acc)


def _emit_nav_time_gps_log(parsed_msg):
    """Emit a per-epoch [NAV-TIMEGPS] log line."""
    itow  = getattr(parsed_msg, 'iTOW', None)
    ftow  = getattr(parsed_msg, 'fTOW', None)
    week  = getattr(parsed_msg, 'week', None)
    leap  = getattr(parsed_msg, 'leapS', None)
    t_acc = getattr(parsed_msg, 'tAcc', None)
    valid = getattr(parsed_msg, 'valid', 0) or 0
    log.info("[NAV-TIMEGPS] iTOW=%s fTOW=%s week=%s leapS=%s "
             "tAcc=%s validTow=%d validWeek=%d validLeap=%d",
             itow, ftow, week, leap, t_acc,
             bool(valid & 0x01), bool(valid & 0x02), bool(valid & 0x04))


def serial_reader(port, baud, obs_queue, stop_event, beph, systems=None,
                   ssr=None, qerr_store=None, config_queue=None, driver=None,
                   raw_callback=None, nav2_store=None, rinex_writer=None,
                   extint_store=None, nav_sig_store=None,
                   nav_clock_store=None, nav_time_gps_store=None):
    """Read UBX messages from a GNSS device.

    Puts (timestamp, observations_list) tuples onto obs_queue for each
    RXM-RAWX epoch. Also feeds RXM-SFRBX to broadcast ephemeris.
    If qerr_store is provided, extracts TIM-TP qErr and stores it.
    If nav2_store is provided, captures NAV2-PVT for position consensus.
    If nav_sig_store is provided, captures UBX-NAV-SIG (per-(SV, sigId)
    receiver usage verdict — prUsed, crUsed, etc.) and emits structured
    [NAV-SIG] log lines per epoch.  Consumed by slipDetectUnified-main
    Phase A + Charlie's Phase A.5 disagreement instrumentation.
    If nav_clock_store / nav_time_gps_store are provided, captures
    UBX-NAV-CLOCK and UBX-NAV-TIMEGPS for receiver-side clock-state
    telemetry (f9tClockTelemetry-bravo / recoveryRetry-main work
    item 2).
    If extint_store is provided, captures TIM-TM2 (DO PPS edge times
    from the F9T's GPS-time solution) for the gnss-phase-experiment
    EKF Arm 3.  Requires the F9T to be configured with
    CFG-MSGOUT-UBX_TIM_TM2_<port>=1 and DO PPS wired to F9T EXTINT.

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
        extint_store: TimTm2Store instance for DOFreqEst Arm 3 ingest.
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
    bds_l1_ref_cycles = driver.bds_l1_ref_cycles
    # Seed the NAV-SIG store with the driver's per-receiver signal map
    # so it can emit human-readable [NAV-SIG] log lines.
    if nav_sig_store is not None:
        nav_sig_store.set_signal_names(SIG_NAMES)

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

            # NAV-SIG: per-(SV, signal) usage verdict from the receiver.
            # Updates Nav2SignalStore + emits structured per-epoch log line
            # for the slipDetectUnified-main Phase A logger and Charlie's
            # Phase A.5 disagreement detector.  Sampled every NAV-SIG
            # epoch (default 1 Hz when CFG-MSGOUT-UBX_NAV_SIG_*=1).
            if msg_id == 'NAV-SIG' and nav_sig_store is not None:
                nav_sig_store.update(parsed)
                _emit_nav_sig_log(nav_sig_store, parsed)

            # NAV-CLOCK: receiver's own clock bias / drift / accuracy.
            # Diagnostic visibility into receiver-side time state for
            # the chip-slip / command-envelope cascade hypotheses
            # (recoveryRetry-main work item 2).
            if msg_id == 'NAV-CLOCK' and nav_clock_store is not None:
                nav_clock_store.update(parsed)
                _emit_nav_clock_log(parsed)

            # NAV-TIMEGPS: receiver's GPS time solution (week, leapS,
            # tAcc, valid flags).  Detects receiver-side time-
            # discontinuity events.
            if msg_id == 'NAV-TIMEGPS' and nav_time_gps_store is not None:
                nav_time_gps_store.update(parsed)
                _emit_nav_time_gps_log(parsed)

            # gnss-phase-experiment Arm 3: DO PPS edge time from
            # F9T's GPS-time solution.  See peppar_fix/extint_reader.py
            # and docs/dofreq-est-measurement-ladder.md.  Only fires
            # if the F9T is configured to emit TIM-TM2 AND DO PPS is
            # wired to F9T EXTINT.  No-op when those preconditions
            # don't hold (extint_store is None or no messages arrive).
            if msg_id == 'TIM-TM2' and extint_store is not None:
                extint_store.update(parsed)

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
                    if cp is not None and cp_valid and sig_name in bds_l1_ref_cycles:
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
                # Counters for the [OBS_COUNTS] emit downstream
                # (peppar_fix_engine.py reads them off the
                # ObservationEvent).  Per dayplan I-143806-main.
                n_off_const = 0
                n_single = 0
                PREFIX_TO_SYS = {'G': 'gps', 'E': 'gal', 'C': 'bds'}
                for sv, roles in raw_obs.items():
                    prefix = sv[0]
                    sys_name = PREFIX_TO_SYS.get(prefix)

                    # System filter
                    if systems and sys_name not in systems:
                        n_off_const += 1
                        continue

                    if 'f1' not in roles or 'f2' not in roles:
                        # Single-frequency only — PR-only contributor;
                        # invisible to MW/IF math but counted as
                        # 'Tracking' in the peppar-mon column.  We
                        # only count this for in-constellation SVs;
                        # off-constellation SVs are already accounted.
                        n_single += 1
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
                        if not hasattr(ssr, '_cb_lookup_heartbeat'):
                            ssr._cb_lookup_heartbeat = {}
                        # Key by (sv, f1_sig, f2_sig); value snapshot
                        # includes cb VALUES (rounded to mm) AND src_mount
                        # so we re-emit whenever the publisher's bias
                        # updates OR the source mount flips (e.g. dual-mount
                        # CNES↔WHU last-write-wins flip).  Critical for
                        # WHU stepwise bringup (I-122350 / I-003751): per-SV
                        # per-signal applied bias-value with source attribution
                        # is the diff signal we need.  See
                        # docs/ssr-cross-ac-diagnostic-2026-04-25.md.
                        lk_cb = (sv, f1['sig_name'], f2['sig_name'])
                        avail_cb = tuple(sorted(ssr._code_bias.get(sv, {}).keys()))
                        cb_f1_q = round(cb_f1, 3) if cb_f1 is not None else None
                        cb_f2_q = round(cb_f2, 3) if cb_f2 is not None else None
                        cb_f1_src = (ssr._code_bias.get(sv, {}).get(rinex_f1[0]).src_mount
                                     if cb_f1 is not None else None)
                        cb_f2_src = (ssr._code_bias.get(sv, {}).get(rinex_f2[0]).src_mount
                                     if cb_f2 is not None else None)
                        snap = (cb_f1_q, cb_f2_q, cb_f1_src, cb_f2_src, avail_cb)
                        # Heartbeat: re-emit every 60th invocation per-key
                        # even if value unchanged, so steady-state biases
                        # are visible in the log for diff/replay analysis
                        # (Bravo's spec on I-122350 review).
                        hb_count = ssr._cb_lookup_heartbeat.get(lk_cb, 0) + 1
                        ssr._cb_lookup_heartbeat[lk_cb] = hb_count
                        is_change = ssr._cb_lookup_logged.get(lk_cb) != snap
                        is_heartbeat = (hb_count % 60) == 0
                        if is_change or is_heartbeat:
                            log.info(
                                "[CB_APPLIED] %s f1=%s→%s val=%sm src=%s "
                                "f2=%s→%s val=%sm src=%s avail=%s%s",
                                sv, f1['sig_name'], rinex_f1[0],
                                f"{cb_f1:+.3f}" if cb_f1 is not None else "MISS",
                                cb_f1_src or "?",
                                f2['sig_name'], rinex_f2[0],
                                f"{cb_f2:+.3f}" if cb_f2 is not None else "MISS",
                                cb_f2_src or "?",
                                list(avail_cb),
                                " hb" if (is_heartbeat and not is_change) else "")
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
                    phase_bias_stepped = False
                    if ssr is not None and rinex_f1 and rinex_f2:
                        # Phase biases are indexed by code signal identifier
                        # in SSR (e.g., 'C1C' not 'L1C') — try both
                        pb_f1 = (ssr.get_phase_bias(sv, rinex_f1[1]) or
                                 ssr.get_phase_bias(sv, rinex_f1[0]))
                        pb_f2 = (ssr.get_phase_bias(sv, rinex_f2[1]) or
                                 ssr.get_phase_bias(sv, rinex_f2[0]))
                        # Path A bias-step detection (see
                        # docs/ssr-phase-bias-step-handling.md).  Compare
                        # current bias (meters) to the previously-applied
                        # value for the same (SV, sig); convert delta to
                        # cycles via wavelength; flag when |Δ| > 0.5 cyc.
                        # Flag rides on the obs dict to suppress a single
                        # MW-jump check at the slip detector — preserves
                        # MW WL-fix state across the AC's segment boundary
                        # while preventing the false-positive slip storm
                        # that destabilized day0426 smoke runs (E11/E12/
                        # E36 mw=5-7c with lock=64.5s, gf<1cm).
                        if not hasattr(ssr, '_pb_prev_value'):
                            ssr._pb_prev_value = {}
                        if not hasattr(ssr, '_pb_prev_disc'):
                            ssr._pb_prev_disc = {}
                        # Path B detection: compare IGS-SSR IDF031
                        # discontinuity counter against prior epoch's
                        # value.  AC increments on segment boundary
                        # (yaw maneuver, datum change, integer rollover).
                        # Catches sub-threshold AC-driven drift that
                        # Path A's |Δ|>0.5 cyc test misses.  Both
                        # triggers OR together — flag set on either
                        # condition.
                        disc_f1 = ssr.get_phase_bias_disc(
                            sv, rinex_f1[1]) if (
                            ssr is not None and rinex_f1) else None
                        if disc_f1 is None and rinex_f1:
                            disc_f1 = ssr.get_phase_bias_disc(
                                sv, rinex_f1[0])
                        disc_f2 = ssr.get_phase_bias_disc(
                            sv, rinex_f2[1]) if (
                            ssr is not None and rinex_f2) else None
                        if disc_f2 is None and rinex_f2:
                            disc_f2 = ssr.get_phase_bias_disc(
                                sv, rinex_f2[0])
                        if pb_f1 is not None:
                            prev = ssr._pb_prev_value.get((sv, 'f1'))
                            if prev is not None and wl_f1:
                                if abs((pb_f1 - prev) / wl_f1) > 0.5:
                                    phase_bias_stepped = True  # Path A
                            ssr._pb_prev_value[(sv, 'f1')] = pb_f1
                        if disc_f1 is not None:
                            prev_disc = ssr._pb_prev_disc.get(
                                (sv, 'f1'))
                            if (prev_disc is not None
                                    and disc_f1 != prev_disc):
                                phase_bias_stepped = True  # Path B
                            ssr._pb_prev_disc[(sv, 'f1')] = disc_f1
                        if pb_f2 is not None:
                            prev = ssr._pb_prev_value.get((sv, 'f2'))
                            if prev is not None and wl_f2:
                                if abs((pb_f2 - prev) / wl_f2) > 0.5:
                                    phase_bias_stepped = True  # Path A
                            ssr._pb_prev_value[(sv, 'f2')] = pb_f2
                        if disc_f2 is not None:
                            prev_disc = ssr._pb_prev_disc.get(
                                (sv, 'f2'))
                            if (prev_disc is not None
                                    and disc_f2 != prev_disc):
                                phase_bias_stepped = True  # Path B
                            ssr._pb_prev_disc[(sv, 'f2')] = disc_f2
                        if pb_f1 is not None:
                            cp_f1 -= pb_f1 / wl_f1  # meters → cycles
                        if pb_f2 is not None:
                            cp_f2 -= pb_f2 / wl_f2
                        ar_phase_bias_ok = (pb_f1 is not None
                                            and pb_f2 is not None)
                        if phase_bias_stepped:
                            log.info(
                                "[PB_STEP] %s phase-bias segment boundary "
                                "(suppressing MW jump check this epoch)",
                                sv)
                        # Phase-bias diagnostic: log VALUES on every
                        # change, not just first occurrence.  Lets the
                        # cross-AC compare pick up CNES vs WHU value
                        # differences per (SV, signal) over time.  See
                        # docs/ssr-cross-ac-diagnostic-2026-04-25.md.
                        if not hasattr(ssr, '_pb_lookup_logged'):
                            ssr._pb_lookup_logged = {}
                        if not hasattr(ssr, '_pb_lookup_heartbeat'):
                            ssr._pb_lookup_heartbeat = {}
                        lk = (sv, f1['sig_name'], f2['sig_name'])
                        avail_pb = tuple(sorted(ssr._phase_bias.get(sv, {}).keys()))
                        pb_f1_q = round(pb_f1, 3) if pb_f1 is not None else None
                        pb_f2_q = round(pb_f2, 3) if pb_f2 is not None else None
                        # Source-mount tagging: phase biases are looked up
                        # by code-signal first then phase-signal; reflect
                        # the actual applied value's source.  Per I-122350
                        # P1 / I-003751 stepwise bringup: cross-mount
                        # last-write-wins is the diagnostic axis we need.
                        def _pb_src(rinex):
                            for k in (rinex[1], rinex[0]):
                                bc = ssr._phase_bias.get(sv, {}).get(k)
                                if bc is not None:
                                    return bc.src_mount
                            return None
                        pb_f1_src = _pb_src(rinex_f1) if pb_f1 is not None else None
                        pb_f2_src = _pb_src(rinex_f2) if pb_f2 is not None else None
                        snap_pb = (pb_f1_q, pb_f2_q, pb_f1_src, pb_f2_src, avail_pb)
                        hb_count = ssr._pb_lookup_heartbeat.get(lk, 0) + 1
                        ssr._pb_lookup_heartbeat[lk] = hb_count
                        is_change = ssr._pb_lookup_logged.get(lk) != snap_pb
                        is_heartbeat = (hb_count % 60) == 0
                        if is_change or is_heartbeat:
                            # peppar-mon contract:
                            # peppar_mon/log_reader.py:_PB_APPLIED_RE
                            # parses this format.  HIT vs MISS is
                            # detected from the ``val=...m`` field —
                            # numeric value = HIT, literal "MISS" =
                            # MISS.  peppar-mon latches NL-capable
                            # constellations on first HIT-HIT.
                            # ``src=`` field added 2026-05-02 (P1 of
                            # I-122350); peppar-mon parser tolerates
                            # the additional field.
                            log.info(
                                "[PB_APPLIED] %s f1=%s→%s val=%sm src=%s "
                                "f2=%s→%s val=%sm src=%s avail=%s%s",
                                sv, f1['sig_name'], rinex_f1[0],
                                f"{pb_f1:+.3f}" if pb_f1 is not None else "MISS",
                                pb_f1_src or "?",
                                f2['sig_name'], rinex_f2[0],
                                f"{pb_f2:+.3f}" if pb_f2 is not None else "MISS",
                                pb_f2_src or "?",
                                list(avail_pb),
                                " hb" if (is_heartbeat and not is_change) else "")
                            ssr._pb_lookup_logged[lk] = snap_pb

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
                        # Set when an SSR phase-bias for this SV stepped
                        # by >0.5 cycles since the previous epoch — the
                        # AC published a new bias-segment.  MW jump
                        # detector skips this epoch to avoid spoofed
                        # multi-cycle slips (Path A; see
                        # docs/ssr-phase-bias-step-handling.md).
                        'phase_bias_stepped': phase_bias_stepped,
                    })

                # Per-epoch admit counters for peppar-mon Untracked /
                # Tracking / Dual columns.  Partition: raw = untracked +
                # tracking + dual.
                #   untracked = receiver delivered a usable signal but
                #               systems config rejects (off-constellation)
                #   tracking  = systems passes but only single-freq tracked
                #               (single-band SVs like BDS B1I on TIM 2.20),
                #               or dual-tracked but phase invalid this epoch
                #   dual      = len(observations) — full IF candidates
                _untracked = 0
                _tracking = 0
                for _sv, _roles in raw_obs.items():
                    _sys = PREFIX_TO_SYS.get(_sv[0])
                    if systems and _sys not in systems:
                        _untracked += 1
                    elif not ('f1' in _roles and 'f2' in _roles):
                        _tracking += 1
                    elif (_roles['f1'].get('cp') is None
                          or _roles['f2'].get('cp') is None
                          or not _roles['f1'].get('half_cyc')
                          or not _roles['f2'].get('half_cyc')):
                        _tracking += 1
                log.info(
                    f"[OBS_ADMIT] epoch={n_epochs} raw={len(raw_obs)} "
                    f"untracked={_untracked} tracking={_tracking} "
                    f"dual={len(observations)}"
                )

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

                    # Optional: write a RINEX OBS record for offline
                    # cross-engine verification (PRIDE PPP-AR, RTKLIB).
                    # Re-shape raw_obs from {sv:{role:{sig_name,...}}}
                    # to {sv:{sig_name:{pr,cp,cno,lock_ms,half_cyc}}}.
                    if rinex_writer is not None:
                        try:
                            rnx_obs = {}
                            for sv, roles in raw_obs.items():
                                per_sig = {}
                                for role, fields in roles.items():
                                    sig = fields.get('sig_name')
                                    if sig is None:
                                        continue
                                    per_sig[sig] = {
                                        'pr':       fields.get('pr'),
                                        'cp':       fields.get('cp'),
                                        'cno':      fields.get('cno'),
                                        'lock_ms':  fields.get('lock_ms'),
                                        'half_cyc': fields.get('half_cyc', True),
                                    }
                                if per_sig:
                                    rnx_obs[sv] = per_sig
                            rinex_writer.write_epoch(gps_time, rnx_obs)
                        except Exception as e:
                            log.warning(f"rinex_writer.write_epoch failed: {e}")

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
                        n_raw=len(raw_obs),
                        n_off_const=n_off_const,
                        n_single=n_single,
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
                 bias_only=False, skip_biases=False,
                 skip_code_biases=False, skip_phase_biases=False):
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

    skip_biases / skip_code_biases / skip_phase_biases: drop matching
    bias messages without routing them.  Diagnostic flags for isolating
    which correction class drives observed obs-model bias.  See
    docs/ssr-cross-ac-diagnostic-2026-04-25.md.  Typical use:
    `skip_biases=True` on the primary stream while a secondary stream
    supplies the biases — gives a clean "orbit/clock from A, biases
    from B" experiment.
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

            # Diagnostic skip filters apply uniformly: drop matching bias
            # messages before any other routing happens.
            if skip_code_biases and identity in CODE_BIAS_MSG_TYPES:
                continue
            if skip_phase_biases and identity in PHASE_BIAS_MSG_TYPES:
                continue
            if skip_biases and identity in BIAS_MSG_TYPES:
                continue

            # Route to appropriate handler.  In bias_only mode, reject
            # everything except the bias message subset — the primary
            # mount owns orbit/clock/ephemeris.  AND filter the
            # secondary-mount bias writes to gap-fill signals only:
            # cross-AC datum mixing on shared (SV, signal) breaks AR
            # because LAMBDA expects integer ambiguities in a single
            # datum but the persistent AC datum offset is non-integer.
            # See docs/ac-datum-mixing.md and ssr_corrections.GAP_FILL_SIGNALS.
            if bias_only:
                if identity in BIAS_MSG_TYPES:
                    result = ssr.update_from_rtcm(
                        msg_view, src_mount=label, gap_fill_only=True)
                    if n_total <= 5:
                        log.info(f"[{label}] bias routed (gap-fill only): "
                                 f"{identity} → {result}")
                else:
                    n_skipped_non_phase_bias += 1
            elif identity in EPH_MSG_TYPES:
                prn = beph.update_from_rtcm(msg_view)
                if prn and beph.n_satellites % 10 == 0:
                    log.debug(f"[{label}] {beph.summary()}")
            elif identity in SSR_MSG_TYPES or identity.startswith('4076_'):
                # I-122350-main P1 follow-up: pass label as src_mount on
                # the primary-mount path too.  Without this, every bias
                # delivered by the primary mount tags src=? in
                # [CB_APPLIED] / [PB_APPLIED] (only the bias_only
                # secondary-mount path was plumbing src_mount, which
                # makes the dual-mount diagnostic asymmetric — CNES
                # biases unlabelled, WHU biases labelled).  Also
                # confirmed against TimeHat day0502-cnes-baseline log
                # (184/184 entries with src=?).
                result = ssr.update_from_rtcm(msg_view, src_mount=label)
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
        # peppar-mon contract: peppar_mon/log_reader.py:_EPH_STREAM_RE
        # parses HOST:PORT/MOUNT.  Same format as the engine wrapper.
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
        # peppar-mon contract: peppar_mon/log_reader.py:_SSR_STREAM_RE
        # parses HOST:PORT/MOUNT.
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

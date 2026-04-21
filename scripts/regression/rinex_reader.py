"""RINEX 3.x OBS reader — yields per-epoch observation dicts.

Used by the regression harness to replay observations from a published
dataset (PRIDE-PPPAR examples, IGS MGEX stations) through PePPAR Fix's
PPP pipeline.  The truth-position check then runs against the station's
independently-published ITRF coordinate, not against any specific
PPP-AR package's output.

## Output format

Yields dicts roughly matching the UBX RXM-RAWX serial reader's output in
`scripts/realtime_ppp.py:serial_reader`.  Per epoch we emit a list of
per-SV dicts; each includes both frequencies when dual-band tracking
is present.

Fields we populate from RINEX:

    sv          str    — e.g. 'G17', 'E23', 'C34'
    sys         str    — 'GPS', 'GAL', 'BDS', 'GLO'
    gps_time    float  — seconds of GPS week (from RINEX epoch)
    pr1_m, pr2_m        float — pseudoranges
    phi1_cyc, phi2_cyc  float — carrier phases in cycles (raw; no SSR
                                bias applied — regression harness must
                                apply bias if exercising that path)
    phi1_raw_cyc, phi2_raw_cyc  same as phi*_cyc (identical for RINEX
                                since there's no upstream SSR correction)
    f1_sig_name, f2_sig_name    str — RINEX signal codes mapped to
                                our internal names (e.g. 'GPS-L1CA')
    wl_f1, wl_f2        float  — wavelengths (m)
    cno                 float  — min of the two signals
    lock_duration_ms    int    — synthesized from loss-of-lock indicator
                                 (see note below)
    half_cyc_ok         bool   — True unless half-cycle flag set

## Fields we cannot populate from RINEX alone

These are set to sentinels or synthesized:

    lock_duration_ms       — RINEX has a per-sample loss-of-lock
                              indicator (LLI) byte, not a running
                              lock-time.  We synthesize as (epochs
                              since last LLI > 0) × epoch_interval_ms.
                              Good enough for slip detection; not
                              comparable to UBX's hardware counter.

Bias-correction (phi1_cyc after SSR phase bias applied) must be done
by the caller after calling us, since the SSR data comes from a
separate source.

## Signal-code mapping

RINEX observation codes use a 3-char format: `<type><band><attr>`:
  - type: C (pseudorange), L (carrier phase), D (doppler), S (SNR)
  - band: 1, 2, 5, 6, 7, 8
  - attr: code letter (C=C/A, P=P(Y), L=L2CL, Q=Q-only pilot, I=I data,
          X=I+Q combined, W=Z-tracking, S=short, etc.)

Our engine's internal signal names (from `signal_wavelengths.py`) use
the "SYS-Band + attribute" form like `GPS-L1CA`, `GAL-E5aQ`, `BDS-B2aI`.
The table `_RINEX_TO_INTERNAL` maps between them for the signals our
F9T fleet tracks.
"""

from __future__ import annotations

import logging
import re
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Iterator, Optional

log = logging.getLogger(__name__)

# Physical constants
_C_LIGHT = 299_792_458.0


# ── Signal-code maps ───────────────────────────────────────────────── #

# RINEX obs-code → internal signal name + carrier frequency (Hz).
# Covers the signals our F9T fleet tracks (both L2 and L5 profiles).
# Extend as we encounter new stations/signals.
_RINEX_TO_INTERNAL: dict[str, tuple[str, float]] = {
    # GPS
    ("G", "1C"): ("GPS-L1CA", 1_575.42e6),  # C/A code on L1
    ("G", "1W"): ("GPS-L1W",  1_575.42e6),  # Z-tracking on L1 (semi-codeless)
    ("G", "1X"): ("GPS-L1X",  1_575.42e6),  # L1C-I+Q combined (RX-specific)
    ("G", "2L"): ("GPS-L2CL", 1_227.60e6),  # L2 civil long
    ("G", "2W"): ("GPS-L2W",  1_227.60e6),  # Z-tracking on L2
    ("G", "2X"): ("GPS-L2X",  1_227.60e6),  # L2C I+M combined
    ("G", "5Q"): ("GPS-L5Q",  1_176.45e6),  # L5 Q-channel (pilot)
    ("G", "5X"): ("GPS-L5X",  1_176.45e6),  # L5 I+Q combined
    # Galileo
    ("E", "1C"): ("GAL-E1C",  1_575.42e6),  # E1 pilot
    ("E", "1X"): ("GAL-E1X",  1_575.42e6),  # E1 B+C combined
    ("E", "5Q"): ("GAL-E5aQ", 1_176.45e6),  # E5a Q
    ("E", "5X"): ("GAL-E5aX", 1_176.45e6),  # E5a I+Q combined
    ("E", "7Q"): ("GAL-E5bQ", 1_207.14e6),  # E5b Q
    ("E", "7X"): ("GAL-E5bX", 1_207.14e6),  # E5b I+Q combined
    ("E", "8Q"): ("GAL-E5Q",  1_191.795e6), # E5 AltBOC Q
    ("E", "8X"): ("GAL-E5X",  1_191.795e6), # E5 AltBOC I+Q
    # BeiDou — BDS-2 and BDS-3 legacy signals are on B1I/B2I/B3I; BDS-3
    # modernised signals are on B1C/B2a.  F9T-L5 tracks B1I + B2a-I.
    # F9T-L2 (ptpmon) tracks B1I + B2I.
    ("C", "2I"): ("BDS-B1I",  1_561.098e6), # BDS-2/3 legacy B1I
    ("C", "2X"): ("BDS-B1X",  1_561.098e6), # B1I I+Q
    ("C", "7I"): ("BDS-B2I",  1_207.14e6),  # BDS-2 legacy B2I
    ("C", "5P"): ("BDS-B2aP", 1_176.45e6),  # BDS-3 modernised B2a pilot
    ("C", "5D"): ("BDS-B2aI", 1_176.45e6),  # BDS-3 modernised B2a data
    ("C", "5X"): ("BDS-B2aX", 1_176.45e6),  # BDS-3 B2a I+Q
    ("C", "6I"): ("BDS-B3I",  1_268.52e6),  # BDS-2/3 B3I
}


# Header regexes (RINEX 3+)
_HDR_VERSION = re.compile(r"^\s*(\d+\.\d+)\s+OBSERVATION DATA")
_HDR_MARKER = re.compile(r"^(\S+)\s+MARKER NAME")
_HDR_APPROX = re.compile(
    r"^\s*(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+APPROX POSITION XYZ"
)
_HDR_INTERVAL = re.compile(r"^\s+(\d+\.\d+)\s+INTERVAL")
_HDR_SYS_OBS = re.compile(r"^([GRESCIJ])\s+(\d+)\s+(.*?)\s+SYS / # / OBS TYPES")
_HDR_SYS_CONT = re.compile(r"^\s{6}(.*?)\s+SYS / # / OBS TYPES")
_HDR_END = re.compile(r"^\s*END OF HEADER")

# Epoch line: "> YYYY MM DD HH MM SS.sssssss EF NN"
_EPOCH = re.compile(
    r"^>\s+(\d{4})\s+(\d{1,2})\s+(\d{1,2})\s+(\d{1,2})\s+(\d{1,2})\s+"
    r"([\d.]+)\s+(\d+)\s+(\d+)"
)


@dataclass
class RinexHeader:
    version: str = ""
    marker: str = ""
    approx_xyz: Optional[tuple[float, float, float]] = None
    interval_s: float = 1.0
    # system → ordered list of 3-char RINEX obs codes (e.g. "C1C")
    sys_obs_types: dict[str, list[str]] = field(default_factory=dict)


@dataclass
class RinexEpoch:
    """One epoch from a RINEX OBS file."""
    ts: datetime          # UTC; caller converts to gps_time if needed
    epoch_flag: int       # 0 = OK, 1 = power failure, ...
    obs: dict[str, dict[str, tuple[float, int, int]]]
    # obs[sv][obs_code] = (value, lli, ssi)
    #   lli = loss-of-lock indicator (0 = no slip)
    #   ssi = signal strength indicator (1..9; 0 = undefined)


def _sys_prefix(code_type: str) -> str:
    """RINEX SV prefix ('G','E','C','R','J','S','I') → our sys name."""
    return {
        "G": "GPS", "E": "GAL", "C": "BDS", "R": "GLO",
        "J": "QZS", "S": "SBAS", "I": "IRNSS",
    }.get(code_type, "UNK")


def parse_header(path: Path) -> RinexHeader:
    """Parse RINEX 3.x OBS file header.  Returns without consuming body."""
    hdr = RinexHeader()
    pending_sys: Optional[str] = None
    pending_count: int = 0
    with open(path) as f:
        for line in f:
            if _HDR_END.search(line):
                return hdr
            m = _HDR_VERSION.match(line)
            if m:
                hdr.version = m.group(1)
                continue
            m = _HDR_MARKER.match(line)
            if m:
                hdr.marker = m.group(1)
                continue
            m = _HDR_APPROX.match(line)
            if m:
                hdr.approx_xyz = (
                    float(m.group(1)), float(m.group(2)), float(m.group(3)),
                )
                continue
            m = _HDR_INTERVAL.match(line)
            if m:
                hdr.interval_s = float(m.group(1))
                continue
            m = _HDR_SYS_OBS.match(line)
            if m:
                pending_sys = m.group(1)
                pending_count = int(m.group(2))
                hdr.sys_obs_types[pending_sys] = m.group(3).split()
                continue
            m = _HDR_SYS_CONT.match(line)
            if m and pending_sys is not None:
                hdr.sys_obs_types[pending_sys].extend(m.group(1).split())
                if len(hdr.sys_obs_types[pending_sys]) >= pending_count:
                    pending_sys = None
                continue
    return hdr


def _read_epoch(f, obs_types: dict[str, list[str]]) -> Optional[RinexEpoch]:
    """Read one epoch from an open RINEX file.  Returns None at EOF."""
    while True:
        line = f.readline()
        if not line:
            return None
        m = _EPOCH.match(line)
        if m:
            break

    year, mon, day, hr, mn, secs, flag_str, nsat_str = m.groups()
    ts = datetime(int(year), int(mon), int(day), int(hr), int(mn), int(float(secs)))
    epoch_flag = int(flag_str)
    nsat = int(nsat_str)

    obs: dict[str, dict[str, tuple[float, int, int]]] = {}
    for _ in range(nsat):
        row = f.readline().rstrip("\n")
        if not row:
            break
        sv = row[:3].strip()
        if not sv:
            continue
        sys_pref = sv[0]
        codes = obs_types.get(sys_pref)
        if not codes:
            continue
        # Each observation is 16 chars: 14 value + 1 LLI + 1 SSI.
        sv_obs: dict[str, tuple[float, int, int]] = {}
        for i, code in enumerate(codes):
            start = 3 + i * 16
            if start >= len(row):
                break
            chunk = row[start:start + 16]
            if len(chunk) < 14:
                continue
            value_str = chunk[:14].strip()
            if not value_str:
                continue
            try:
                value = float(value_str)
            except ValueError:
                continue
            lli = 0
            ssi = 0
            if len(chunk) >= 15 and chunk[14].strip():
                try:
                    lli = int(chunk[14])
                except ValueError:
                    pass
            if len(chunk) >= 16 and chunk[15].strip():
                try:
                    ssi = int(chunk[15])
                except ValueError:
                    pass
            sv_obs[code] = (value, lli, ssi)
        if sv_obs:
            obs[sv] = sv_obs
    return RinexEpoch(ts=ts, epoch_flag=epoch_flag, obs=obs)


def iter_epochs(path: Path) -> Iterator[RinexEpoch]:
    """Yield RinexEpoch records from a RINEX 3.x OBS file."""
    hdr = parse_header(path)
    with open(path) as f:
        # Skip through header
        for line in f:
            if _HDR_END.search(line):
                break
        while True:
            ep = _read_epoch(f, hdr.sys_obs_types)
            if ep is None:
                return
            yield ep


# ── Higher-level: select F9T-compatible signal pair per SV ─────────── #

@dataclass
class SvObservation:
    """Per-SV dual-frequency observation compatible with PPP filter.

    Shape roughly matches what `serial_reader` in realtime_ppp produces,
    but without the SSR-bias-corrected phi*_cyc fields (regression
    harness is responsible for applying bias corrections if exercising
    that path).
    """
    sv: str
    sys: str
    f1_sig_name: str
    f2_sig_name: str
    wl_f1: float
    wl_f2: float
    pr1_m: float
    pr2_m: float
    phi1_cyc: float         # raw (no SSR bias applied yet)
    phi2_cyc: float
    phi1_raw_cyc: float     # same as phi1_cyc for RINEX source
    phi2_raw_cyc: float
    cno: float
    lock_duration_ms: int
    f1_lock_ms: int
    f2_lock_ms: int
    half_cyc_ok: bool


# Preferred signal codes per system for each profile.  The regression
# harness picks the pair matching the profile of the receiver it's
# simulating.  Extend as we add test cases for other receivers.
L5_PROFILE = {
    "GPS": ("1C", "5Q"),    # L1CA + L5Q  (fallback: 5X if 5Q missing)
    "GAL": ("1C", "5Q"),    # E1C + E5aQ  (fallback: 5X, 7Q for E5b)
    "BDS": ("2I", "5P"),    # B1I + B2a-P (fallback: 5X, 5D)
}
L2_PROFILE = {
    "GPS": ("1C", "2L"),    # L1CA + L2CL
    "GAL": ("1C", "7Q"),    # E1C + E5bQ
    "BDS": ("2I", "7I"),    # B1I + B2I (BDS-2 legacy)
}


def _pick_signal_pair(
    sys_name: str, sv_obs: dict[str, tuple[float, int, int]],
    profile: dict[str, tuple[str, str]],
) -> Optional[tuple[str, str]]:
    """Pick the signal-code pair for this SV.  Returns ('1C', '5Q')
    etc. — the 2-char RINEX band+attribute codes — if both pseudorange
    and carrier phase are present for each.  None if missing."""
    pref = profile.get(sys_name)
    if pref is None:
        return None
    b1, b2 = pref
    pr1_key = f"C{b1}"
    ph1_key = f"L{b1}"
    pr2_key = f"C{b2}"
    ph2_key = f"L{b2}"
    if (pr1_key in sv_obs and ph1_key in sv_obs
            and pr2_key in sv_obs and ph2_key in sv_obs):
        return (b1, b2)
    return None


def extract_dual_freq(
    epoch: RinexEpoch, profile: dict[str, tuple[str, str]] = L5_PROFILE,
    interval_s: float = 1.0, lock_accum: Optional[dict[str, tuple[int, int]]] = None,
) -> list[SvObservation]:
    """Given a RinexEpoch, produce per-SV dual-frequency observations.

    `lock_accum` is an optional mutable dict tracking (f1_lock_ms,
    f2_lock_ms) across epochs, keyed by SV.  Caller should pass the
    same dict across successive calls; we reset on loss-of-lock (LLI=1)
    and accumulate otherwise.  None → synthesized as a large value
    (static stations rarely slip).
    """
    result: list[SvObservation] = []
    if lock_accum is None:
        lock_accum = {}
    for sv, sv_obs in epoch.obs.items():
        sys_name = _sys_prefix(sv[0])
        if sys_name not in profile:
            continue
        pair = _pick_signal_pair(sys_name, sv_obs, profile)
        if pair is None:
            continue
        b1, b2 = pair
        pr1 = sv_obs.get(f"C{b1}")
        pr2 = sv_obs.get(f"C{b2}")
        ph1 = sv_obs.get(f"L{b1}")
        ph2 = sv_obs.get(f"L{b2}")
        snr1 = sv_obs.get(f"S{b1}")
        snr2 = sv_obs.get(f"S{b2}")
        if not (pr1 and pr2 and ph1 and ph2):
            continue
        sig1_name, f1_hz = _RINEX_TO_INTERNAL.get((sv[0], b1), (None, None))
        sig2_name, f2_hz = _RINEX_TO_INTERNAL.get((sv[0], b2), (None, None))
        if not sig1_name or not sig2_name:
            continue
        wl1 = _C_LIGHT / f1_hz
        wl2 = _C_LIGHT / f2_hz
        # Accumulate lock time; reset on LLI odd bit (cycle slip).
        prev = lock_accum.get(sv, (0, 0))
        step_ms = int(interval_s * 1000)
        f1_lock = 0 if ph1[1] & 1 else prev[0] + step_ms
        f2_lock = 0 if ph2[1] & 1 else prev[1] + step_ms
        lock_accum[sv] = (f1_lock, f2_lock)
        # CNO: SNR indicator 0..9 is very coarse; if raw SNR present
        # use it (as some decoders emit dBHz directly in S field).
        # For RINEX 3 the S* field IS dB-Hz, not the 0..9 indicator.
        cno = min(snr1[0] if snr1 else 0.0, snr2[0] if snr2 else 0.0)
        half_cyc_ok = (ph1[1] & 2 == 0) and (ph2[1] & 2 == 0)
        result.append(SvObservation(
            sv=sv, sys=sys_name,
            f1_sig_name=sig1_name, f2_sig_name=sig2_name,
            wl_f1=wl1, wl_f2=wl2,
            pr1_m=pr1[0], pr2_m=pr2[0],
            phi1_cyc=ph1[0], phi2_cyc=ph2[0],
            phi1_raw_cyc=ph1[0], phi2_raw_cyc=ph2[0],
            cno=cno,
            lock_duration_ms=min(f1_lock, f2_lock),
            f1_lock_ms=f1_lock, f2_lock_ms=f2_lock,
            half_cyc_ok=half_cyc_ok,
        ))
    return result

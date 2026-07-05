"""Septentrio SBF observation adapter — the geodetic raw-obs bridge (I-030423).

The Fugro GNSSDO+ mosaic-T (and other Septentrio receivers) can't output RTCM
in the shipped firmware, but streams SBF, whose **MeasEpoch (block 4027)**
carries the full raw measurements (pseudorange, carrier phase, C/N0, lock) for
every tracked signal — including GPS L1P/L2P (RINEX L1W/L2W) that the RTCM CORS
streams don't provide.  This decodes MeasEpoch into the engine's per-SV
``raw_obs`` and forms IF observations via the SAME shared former the RTCM MSM
and UBX-RXM-RAWX paths use — so PPP / ZTD / AR / clock apply unchanged.

Uses ``pysbf2`` for SBF framing + block parsing (as ``rtcm_msm_obs`` uses
``pyrtcm``).  GPS/GAL/BDS are CDMA and drop straight into the model; GLONASS
(FDMA/IFB) is a deliberate follow-on.

MeasEpoch reconstruction (per the SBF reference / pysbf2 block doc):
    PRtype1 [m]     = (CodeMSB·2^32 + CodeLSB)·0.001
    Ltype1 [cyc]    = PRtype1/λ + (CarrierMSB·65536 + CarrierLSB)·0.001
    PRtype2 [m]     = PRtype1 + signed(CodeOffsetMSB·65536 + CodeOffsetLSB)·0.001
    Ltype2 [cyc]    = PRtype2/λ + (CarrierMSB·65536 + CarrierLSB)·0.001
A carrier with CarrierMSB == -128 is the Do-Not-Use sentinel (no phase lock).
"""
from __future__ import annotations

import logging

from pysbf2.sbftypes_decodes import SIGNAL_TYPE

from peppar_fix.rtcm_msm_obs import cells_to_raw_obs  # shared raw_obs assembly

log = logging.getLogger("peppar-fix.sbf")

_C = 299792458.0
_CARRIER_DNU = -128            # CarrierMSB sentinel → no carrier phase


def _svid_to_sv(svid):
    """Septentrio SVID → engine SV id (``G07``/``E11``/``C14``), or None for a
    constellation we don't ingest (GLONASS FDMA, SBAS, QZSS, NavIC)."""
    svid = int(svid)
    if 1 <= svid <= 37:
        return f"G{svid:02d}"
    if 71 <= svid <= 106:
        return f"E{svid - 70:02d}"
    if 141 <= svid <= 180:
        return f"C{svid - 140:02d}"
    return None


# Septentrio signal number (pysbf2 SIGNAL_TYPE) → engine sig_name.  Only signals
# with an engine sig_name + wavelength are listed; the rest decode to None and
# are dropped.  GPS L1P (sig 1, RINEX L1W) and L1C (sig 5) need catalog entries
# the engine doesn't have yet — a follow-on (L1W would match NRCan's phase bias).
_SIG_TO_SIGNAME = {
    0: "GPS-L1CA",    # L1CA (1C)
    2: "GPS-L2W",     # L2P  (2W)  — geodetic L2 Z-tracking
    3: "GPS-L2CL",    # L2C  (2L)
    4: "GPS-L5Q",     # L5   (5Q)
    17: "GAL-E1C",    # E1   (1C)
    20: "GAL-E5aQ",   # E5a  (5Q)
    21: "GAL-E5bQ",   # E5b  (7Q)
    28: "BDS-B1I",    # B1I  (2I)
}

# carrier frequency (Hz) per engine sig_name, for PR/λ → cycles.
_SIG_FREQ = {
    "GPS-L1CA": 1575.42e6, "GPS-L2W": 1227.60e6, "GPS-L2CL": 1227.60e6,
    "GPS-L5Q": 1176.45e6,
    "GAL-E1C": 1575.42e6, "GAL-E5aQ": 1176.45e6, "GAL-E5bQ": 1207.14e6,
    "BDS-B1I": 1561.098e6,
}


def _cn0_dbhz(raw, signame):
    """SBF C/N0 (0.25 dB-Hz units) → dB-Hz; L1CA and E5-AltBOC carry a +10 offset."""
    if raw is None:
        return 0.0
    return raw * 0.25 + (10.0 if signame == "GPS-L1CA" else 0.0)


def _signed(value, bits):
    """Interpret an unsigned int as two's-complement over ``bits`` bits."""
    if value >= (1 << (bits - 1)):
        value -= (1 << bits)
    return value


def decode_meas_epoch(msg):
    """A pysbf2-parsed MeasEpoch (block 4027) → ``(tow_ms, cells)``.

    ``cells`` is a list of ``{sv, sig_name, freq_hz, pr_m, cp_cyc, cno, lock_ms,
    half_ok}`` — the same shape ``rtcm_msm_obs.decode_msm_obs`` emits, so it
    feeds ``cells_to_raw_obs`` + the shared IF-former unchanged.  Returns None
    for a non-MeasEpoch message.  Signals with no engine sig_name are skipped.
    """
    if getattr(msg, "identity", "") != "MeasEpoch":
        return None
    tow_ms = int(msg.TOW)
    n1 = int(msg.N1)
    cells = []

    def t1(field, i):
        return getattr(msg, f"{field}_{i:02d}", None)

    def t2(field, j, i):
        # pysbf2 names nested Type2 sub-block fields channel-index FIRST, then
        # sub-block index: SigIdxLo_<i>_<j>.  (Getting this transposed reads a
        # neighbouring channel's sub-block — invisible for channel i=1 where
        # _01_01 is symmetric, corrupting every SV after it.  Caught by the
        # RTKLIB reference-decode value pin, I-110210.)
        return getattr(msg, f"{field}_{i:02d}_{j:02d}", None)

    for i in range(1, n1 + 1):
        sv = _svid_to_sv(t1("SVID", i))
        if sv is None:
            continue
        # --- Type1: the SV's main signal ---
        sig1 = int(t1("SigIdxLo", i) or 0) + (int(t1("SigIdxHi", i) or 0) << 5)
        pr1 = (int(t1("CodeMSB", i) or 0) * 4294967296
               + int(t1("CodeLSB", i) or 0)) * 0.001
        cell = _make_cell(
            sv, sig1, pr1,
            carr_msb=t1("CarrierMSB", i), carr_lsb=t1("CarrierLSB", i),
            cno=t1("CN0", i), lock=t1("LockTime", i),
            half=t1("HalfCycleAmbiguity", i))
        if cell is not None:
            cells.append(cell)
        # --- Type2: additional signals on the same SV ---
        for j in range(1, int(t1("N2", i) or 0) + 1):
            sig2 = (int(t2("SigIdxLo", j, i) or 0)
                    + (int(t2("SigIdxHi", j, i) or 0) << 5))
            off = _signed((int(t2("CodeOffsetMSB", j, i) or 0) << 16)
                          + int(t2("CodeOffsetLSB", j, i) or 0), 19)
            pr2 = pr1 + off * 0.001
            cell = _make_cell(
                sv, sig2, pr2,
                carr_msb=t2("CarrierMSB", j, i), carr_lsb=t2("CarrierLSB", j, i),
                cno=t2("CN0", j, i), lock=t2("LockTime", j, i),
                half=t2("HalfCycleAmbiguity", j, i))
            if cell is not None:
                cells.append(cell)
    return tow_ms, cells


def _make_cell(sv, sig_num, pr_m, *, carr_msb, carr_lsb, cno, lock, half):
    sig_name = _SIG_TO_SIGNAME.get(sig_num)
    if sig_name is None:
        return None
    if not cno:                       # no C/N0 → tracked-but-unmeasured slot
        return None
    if not (1e6 < pr_m < 4e7):        # PR sanity gate (same as the RTCM path)
        return None
    freq = _SIG_FREQ[sig_name]
    # carrier phase (cycles): CarrierMSB == -128 is the Do-Not-Use sentinel.
    cp_cyc = None
    if carr_msb is not None and int(carr_msb) != _CARRIER_DNU:
        carrier_off = (int(carr_msb) * 65536 + int(carr_lsb or 0)) * 0.001
        cp_cyc = pr_m / (_C / freq) + carrier_off
    return {
        "sv": sv,
        "sig_name": sig_name,
        "freq_hz": freq,
        "pr_m": pr_m,
        "cp_cyc": cp_cyc,
        "cno": _cn0_dbhz(cno, sig_name),
        "lock_ms": float(lock or 0) * 1000.0,   # SBF lock is seconds → ms
        "half_ok": int(half or 0) == 0,          # 1 = half-cycle ambiguity
    }


def meas_epoch_to_raw_obs(msg, sig_lookup, raw_obs=None):
    """Decode one MeasEpoch and merge it into ``raw_obs[sv][role]`` via the
    shared :func:`cells_to_raw_obs` (same assembly as the RTCM MSM path)."""
    dec = decode_meas_epoch(msg)
    if dec is None:
        return raw_obs if raw_obs is not None else {}
    return cells_to_raw_obs(dec[1], sig_lookup, raw_obs)

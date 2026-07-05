"""RTCM 3 MSM observation adapter — the geodetic-receiver bridge (I-030423).

Decodes RTCM 3 MSM (MSM4/5/6/7) GNSS observations into the engine's per-SV
``raw_obs[sv][role]`` structure and forms iono-free observations through the
SAME ``realtime_ppp.raw_obs_to_if_observations`` the UBX-RXM-RAWX path uses.
That shared IF-former is the format-parity invariant: once MSM obs are in the
model, PPP / ZTD-troposphere / cycle-slip / AR / clock all apply UNCHANGED —
no new tropo, no parallel obs pipeline.  This is the one adapter that unblocks
every non-u-blox receiver (Leica GRX1200, Trimble NetR9, Septentrio PolaRx)
and every NTRIP CORS, which all speak RTCM 3 MSM.

GPS-first (CDMA drops straight into the CDMA obs model).  GLONASS is FDMA
(per-SV wavelengths + inter-frequency biases) — a deliberate follow-on, not
smuggled in here.

Decode promoted from ``scripts/spike_rtcm_msm_ingest.py`` (main, PR #276;
live-proven ALIC 3.04 m / NAPERVILLE 2.3 m horizontal).  The ``msg`` argument
is a pyrtcm-decoded RTCM message (attributes ``DF...`` read via getattr), so
the decode is unit-testable against a lightweight fake message.
"""
from __future__ import annotations

import logging
from collections import defaultdict
from types import SimpleNamespace

log = logging.getLogger("peppar-fix.rtcm_msm")

_C = 299792458.0

# MSM signal ID → engine sig_name (the u-blox-flavored keys the obs model uses:
# SIG_WAVELENGTH / SIG_TO_RINEX / IF_PAIR_PARAMS).  GPS MSM signal IDs are per
# RTCM 10403.3 Table 3.5-91 (DF395 bit positions).  Only signals with an engine
# sig_name are listed; anything else decodes to None and is dropped.  Note L2W
# (Z-tracking, id 10) — the L2 signal geodetic/CORS receivers emit (u-blox does
# not), added to the shared catalog for this bridge.
_GPS_MSM_SIG = {
    2:  ("GPS-L1CA", 1575.42e6),   # 1C  L1 C/A
    10: ("GPS-L2W",  1227.60e6),   # 2W  L2 Z-tracking (geodetic L2)
    15: ("GPS-L2CM", 1227.60e6),   # 2S  L2 CM
    16: ("GPS-L2CL", 1227.60e6),   # 2L  L2 CL
    22: ("GPS-L5I",  1176.45e6),   # 5I
    23: ("GPS-L5Q",  1176.45e6),   # 5Q
}

# Dispatch by the MSM message-number base (num // 10): 1074/1077 → GPS.
# GAL (109x)/BDS (112x) tables are a deliberate follow-on alongside GLONASS.
_CONSTELL = {107: ("G", _GPS_MSM_SIG)}


def _bits(mask: int, width: int) -> list[int]:
    """1-based positions of set bits in a width-bit RTCM DFxxx mask, MSB-first."""
    return [i + 1 for i in range(width) if mask & (1 << (width - 1 - i))]


def _lock_ms(indicator, extended: bool) -> float:
    """RTCM 3 lock-time indicator → minimum lock time in ms.

    DF402 (MSM4/5, 4-bit, Table 3.5-74): min lock = 2**(n+4) ms, 0 at n=0.
    DF407 (MSM6/7, 10-bit extended, Table 3.5-75): piecewise doubling.  The
    engine only compares lock_ms across epochs to spot a *reset* (drop toward
    zero), so the RTCM "minimum lock time" convention is exactly what it wants.
    """
    if indicator is None:
        return 0.0
    n = int(indicator)
    if not extended:                      # DF402, 4-bit
        return 0.0 if n <= 0 else float(2 ** (n + 4))
    # DF407, 10-bit extended: L doubles its slope every 32 codes.
    if n < 64:
        return float(n)
    b = 0
    lo, hi, base, step = 64, 96, 64, 2
    while True:
        if n < hi:
            return float(base + (n - lo) * step)
        base += (hi - lo) * step
        lo, hi = hi, hi + 32
        step *= 2
        b += 1
        if b > 20:                        # 10-bit domain is bounded; guard anyway
            return float(base)


def decode_msm_obs(msg):
    """Reconstruct per-SV-per-signal observations from a pyrtcm MSM message.

    Returns ``(prefix, tow_ms, cells)`` where ``cells`` is a list of dicts
    ``{sv, sig_name, freq_hz, pr_m, cp_cyc, cno, lock_ms, half_ok}`` — or None
    for an unsupported constellation.  Standard MSM assembly: rough range (ms)
    from DF397+DF398, fine pseudorange (DF400 MSM4/5 or DF405 MSM6/7) and fine
    phase-range (DF401 / DF406) added, phase-range/λ → cycles.  ``half_ok`` is
    True when the phase is full-cycle unambiguous (RTCM DF420 == 0 — the
    polarity the engine's ``half_cyc`` gate wants: keep only unambiguous phase).
    """
    num = int(getattr(msg, "DF002"))
    disp = _CONSTELL.get(num // 10)
    if disp is None:
        return None
    prefix, sigtab = disp
    extended = (num % 10) in (6, 7)                 # MSM6/7 use DF405/406/407/408
    sats = _bits(int(msg.DF394), 64)                # PRNs present, in order
    sigs = _bits(int(msg.DF395), 32)                # MSM signal ids present
    nsat, nsig = len(sats), len(sigs)
    cellmask = int(msg.DF396)                       # nsat×nsig bits, sat-major
    tow_ms = int(getattr(msg, "DF004", 0))          # GPS epoch time (ms)
    cells = []
    cell = 0                                        # 1-based counter over SET cells
    for si in range(nsat):
        prn = sats[si]
        rough = float(getattr(msg, f"DF397_{si+1:02d}")) + \
            float(getattr(msg, f"DF398_{si+1:02d}"))
        for gj in range(nsig):
            grid = si * nsig + gj                   # position in the sat-major grid
            if not (cellmask & (1 << (nsat * nsig - 1 - grid))):
                continue
            cell += 1
            mapped = sigtab.get(sigs[gj])
            if mapped is None:                      # signal with no engine sig_name
                continue
            sig_name, freq = mapped
            fine_pr = getattr(msg, f"DF405_{cell:02d}", None) if extended \
                else getattr(msg, f"DF400_{cell:02d}", None)
            fine_ph = getattr(msg, f"DF406_{cell:02d}", None) if extended \
                else getattr(msg, f"DF401_{cell:02d}", None)
            cno = getattr(msg, f"DF408_{cell:02d}", None) if extended \
                else getattr(msg, f"DF403_{cell:02d}", None)
            lock = getattr(msg, f"DF407_{cell:02d}", None) if extended \
                else getattr(msg, f"DF402_{cell:02d}", None)
            half = getattr(msg, f"DF420_{cell:02d}", 0)
            if fine_pr is None:
                continue
            pr_m = _C * (rough + float(fine_pr)) / 1000.0
            cp_cyc = ((_C * (rough + float(fine_ph)) / 1000.0) * freq / _C
                      if fine_ph is not None else None)
            cells.append({
                "sv": f"{prefix}{prn:02d}",
                "sig_name": sig_name,
                "freq_hz": freq,
                "pr_m": pr_m,
                "cp_cyc": cp_cyc,
                "cno": None if cno is None else float(cno),
                "lock_ms": _lock_ms(lock, extended),
                "half_ok": int(half) == 0,          # DF420==0 → full-cycle → keep
            })
    return prefix, tow_ms, cells


def merge_msm_into_raw_obs(msg, sig_lookup, raw_obs=None):
    """Decode one MSM message and merge it into ``raw_obs[sv][role]`` in the
    exact shape ``realtime_ppp.rawx_to_observations`` builds (pr/cno/cp/lock_ms/
    half_cyc/alpha_f1/alpha_f2/sig_name), using ``sig_lookup`` for role + IF
    alphas.  Signals not in ``sig_lookup`` (not part of a configured IF pair)
    are skipped.  Accumulates across the MSM messages of one epoch so a full
    constellation set forms IF pairs together; returns ``raw_obs``.
    """
    if raw_obs is None:
        raw_obs = defaultdict(dict)
    dec = decode_msm_obs(msg)
    if dec is None:
        return raw_obs
    _, _tow_ms, cells = dec
    for c in cells:
        look = sig_lookup.get(c["sig_name"])
        if look is None:
            continue
        _gnss_id, _prefix, role, a1, a2, _name = look
        raw_obs[c["sv"]][role] = {
            "pr": c["pr_m"],
            "cno": 0 if c["cno"] is None else int(round(c["cno"])),
            "cp": c["cp_cyc"],
            "lock_ms": c["lock_ms"],
            "half_cyc": c["half_ok"],
            "alpha_f1": a1,
            "alpha_f2": a2,
            "sig_name": c["sig_name"],
        }
    return raw_obs


def msm_messages_to_observations(msgs, systems, ssr, sig_lookup):
    """Full adapter for one epoch: merge every MSM message into a raw_obs dict,
    then form IF observations through the shared, source-agnostic former.

    ``msgs`` is the epoch's MSM messages (e.g. GPS 1077 [+ future GAL/BDS]).
    Returns ``(observations, raw_obs, n_off_const, n_single)`` — identical shape
    and contents to ``realtime_ppp.rawx_to_observations`` for the same obs.
    """
    from realtime_ppp import raw_obs_to_if_observations
    raw_obs = defaultdict(dict)
    for msg in msgs:
        merge_msm_into_raw_obs(msg, sig_lookup, raw_obs)
    return raw_obs_to_if_observations(raw_obs, systems, ssr)


def default_gps_sig_lookup(l2_sig="GPS-L2W"):
    """A GPS L1 C/A + L2 IF sig_lookup for the common geodetic/CORS case.

    Default pairs L1 C/A with L2 Z-tracking (``GPS-L2W`` — what CORS streams
    carry); pass ``l2_sig='GPS-L2CL'`` for L2C receivers or ``'GPS-L5Q'`` for
    an L1/L5 stream.  Built via the engine's own ``build_sig_lookup`` so the
    roles/alphas match the RXM path exactly.
    """
    from realtime_ppp import build_sig_lookup
    driver = SimpleNamespace(
        name=f"rtcm-gps-{l2_sig}",
        if_pairs=[("GPS", "GPS-L1CA", l2_sig, "G")])
    return build_sig_lookup(driver)

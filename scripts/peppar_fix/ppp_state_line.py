"""The ``[PPP_STATE]`` line — one definition of its wire format.

The engine emits this per-epoch line (position-filter ECEF + σ, residual ZTD +
σ, GPS-time key); ``pos_replay_compare`` parses it; and pos_replay **stage 2b**
will re-emit it when it regenerates the engine output from a replayed bundle.

``format_ppp_state_line`` is the single **producer**, structurally shared by the
engine emitter and the stage-2b re-emitter (so *those two* can't diverge).  The
**consumer**, ``pos_replay_compare._PPP_RE``, is a separate representation — a
regex can't be derived from a printf — so producer↔consumer consistency is NOT
structural; it rests on ``test_ppp_state_line`` (an exact-string anchor + a
field-wise round-trip through the parser) staying comprehensive.  That test is
therefore load-bearing: **a field added here must grow the test** or the
guarantee lapses silently (Charlie #238).
"""
from __future__ import annotations


def format_ppp_state_line(gps_time, n_epochs: int, n_used: int, ecef,
                          sigma_pos_m: float, ztd_m: float,
                          sigma_ztd_m: float) -> str:
    """Render one ``[PPP_STATE]`` line.

    ``gps_time`` is a GPS-time ``datetime`` (``.isoformat()`` → the ``gps=`` key
    that joins our series to an external time axis); ``ecef`` is an (x, y, z)
    sequence in metres.  Caller owns the try/except — ``gps_time.isoformat()``
    can raise on a malformed value and logging must never crash the filter
    thread (engine emit site already guards this).
    """
    return ("[PPP_STATE] gps=%s epoch=%d n=%d ecef=%.4f,%.4f,%.4f "
            "sigma_pos=%.4fm ztd=%+.4fm sigma_ztd=%.4fm"
            % (gps_time.isoformat(), n_epochs, n_used,
               float(ecef[0]), float(ecef[1]), float(ecef[2]),
               float(sigma_pos_m), float(ztd_m), float(sigma_ztd_m)))


def format_anchor_line(gps_time, ecef, h_acc_m, v_acc_m=None) -> str:
    """Render one ``[NAV2_ANCHOR]`` line — the live NAV2 soft-anchor's per-epoch
    FIRING decision (emitted ONLY on the epochs it fired, with the ECEF / h_acc /
    v_acc it used).  pos_replay reads these back to reproduce the anchor
    deterministically instead of re-deriving it via ``get_opinion``, whose
    freshness-timing + amplification mismatch drove the dynamic-window
    realization divergence (I-215452).  ``gps=`` is the same GPS-time key as
    ``[PPP_STATE]`` so the replay matches by epoch."""
    return ("[NAV2_ANCHOR] gps=%s ecef=%.4f,%.4f,%.4f h_acc=%.4f v_acc=%s"
            % (gps_time.isoformat(), float(ecef[0]), float(ecef[1]),
               float(ecef[2]), float(h_acc_m),
               ("%.4f" % float(v_acc_m)) if v_acc_m is not None else "none"))


def parse_anchor_decisions(lines) -> dict:
    """Parse ``[NAV2_ANCHOR]`` lines → ``{gps_iso: (ecef_tuple, h_acc, v_acc|None)}``
    — the per-epoch fired-anchor decisions the replay applies deterministically.
    Epochs absent from the map did not fire (only fired epochs are logged)."""
    import re
    rx = re.compile(
        r"\[NAV2_ANCHOR\]\s+gps=(\S+)\s+ecef=(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)"
        r"\s+h_acc=([\d.]+)\s+v_acc=(\S+)")
    out = {}
    for ln in lines:
        m = rx.search(ln)
        if not m:
            continue
        out[m.group(1)] = (
            (float(m.group(2)), float(m.group(3)), float(m.group(4))),
            float(m.group(5)),
            None if m.group(6) == "none" else float(m.group(6)))
    return out

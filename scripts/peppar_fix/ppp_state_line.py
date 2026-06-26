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

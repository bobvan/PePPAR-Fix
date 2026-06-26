"""The ``[PPP_STATE]`` line — one definition of its wire format.

The engine emits this per-epoch line (position-filter ECEF + σ, residual ZTD +
σ, GPS-time key); ``pos_replay_compare`` parses it; and pos_replay **stage 2b**
will re-emit it when it regenerates the engine output from a replayed bundle.
Three consumers of one format → make the format a single shared function so the
emitter and the parser can't drift (the lesson of the duplicated ZTD apriori in
#235: a load-bearing constant — here a wire format — needs one definition).

``format_ppp_state_line`` is the producer; ``pos_replay_compare._PPP_RE`` is the
matching consumer.  ``test_ppp_state_line`` pins the round-trip.
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

"""Per-observation quality weighting for the MW running mean.

The Melbourne-Wubbena formula's running average converges to
``λ_WL × N_WL`` only when the per-epoch MW samples are themselves
close to that mean.  Two factors degrade individual MW samples
relative to the high-quality observations the running mean
"deserves":

  - **Low elevation**: tropospheric / ionospheric path length grows,
    multipath off the antenna's near-field rises sharply, and the
    receiver's tracking SNR drops.  Standard PPP weighting uses
    ``sin²(elev)`` to capture the geometry-driven noise growth — at
    90° (zenith) the weight is 1.0; at 30° it's 0.25; at 10° it's
    0.03; at 5° it's 0.008.

  - **Low CN0**: directly observable as receiver-reported
    signal-to-noise (dB-Hz).  Below ~30 dB-Hz the MW noise σ rises
    by a factor of 2-4× per CN0-decade, putting low-CN0 samples
    well outside the rounding tolerance the running mean needs.

This module returns a single scalar weight in [0, 1] that the
MelbourneWubbenaTracker uses to scale its EMA update.  Low-quality
samples advance the running mean (and the fix-readiness counter)
slowly; high-quality samples advance it quickly.

Defaults (legacy / replay paths with no quality info available)
return weight=1.0 — equivalent to the unweighted EMA the tracker
used before fix-L.

I-155354 fix-L; sharpens fix-H (Hatch-smoothed PR feeding MW) by
ensuring the MW running mean's noise floor reflects high-quality
observations even when the cycle-slip rate forces short arcs.
"""
from __future__ import annotations

import math


# CN0 below this threshold contributes effectively zero weight; above
# this the contribution ramps linearly to full weight at CN0 ≥ 45 dB-Hz.
# 25 dB-Hz is at the receiver-reportable floor (anything lower usually
# means the SV isn't really tracked); 45 dB-Hz is open-sky no-multipath.
_CNO_FLOOR_DB = 25.0
_CNO_CEIL_DB = 45.0
_CNO_RANGE_DB = _CNO_CEIL_DB - _CNO_FLOOR_DB


def obs_quality_weight(elev_deg=None, cno_db=None):
    """Per-observation quality weight in [0, 1].

    Combines elevation and CN0 contributions multiplicatively:

        w = sin²(elev) × clip((CN0 - 25) / 20, 0, 1)

    Either argument None → that factor contributes 1.0 (no penalty).
    Both None → returns 1.0 (legacy / replay path equivalence to the
    pre-fix-L unweighted EMA).

    The result is exactly 0.0 for unphysical inputs (elev ≤ 0 or
    CN0 ≤ floor); the caller must decide what 0-weight means
    (typically: don't update the running mean, but tick n_epochs for
    arc-tracking).  All other inputs return a positive weight.
    """
    elev_w = 1.0
    if elev_deg is not None:
        if elev_deg <= 0.0:
            return 0.0
        s = math.sin(math.radians(elev_deg))
        elev_w = s * s

    cno_w = 1.0
    if cno_db is not None:
        if cno_db <= _CNO_FLOOR_DB:
            return 0.0
        cno_w = min(1.0, (cno_db - _CNO_FLOOR_DB) / _CNO_RANGE_DB)

    return elev_w * cno_w

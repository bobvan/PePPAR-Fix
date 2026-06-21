#!/usr/bin/env python3
"""Shared a-priori measurement-floor spec model.

This is the SPEC (a-priori) measurement floor used by the stability slides:
the TAPR TICC single-shot resolution (white-PM) combined in quadrature with
the FE-5680A Rb reference's short-term ADEV.  It is an *idealized* model — in
particular it omits the Rb long-τ random-walk-FM (RWFM) upturn, so it is
OPTIMISTIC at long τ.

The math and constants here are the verbatim model previously inlined in
``plot_stability_slide.measurement_floor``; it was factored out so both the
slide tool and ``plot_stability_floor`` can draw the SAME spec floor without
either re-importing the other (and without inheriting the slide's global
matplotlib rcParams mutation).  Do NOT change the math or the constants here —
they are the published spec model.
"""
from __future__ import annotations

import numpy as np

TICC_FLOOR_PS = 60.0          # TAPR TICC single-shot resolution
RB_ADEV_1S = 1.4e-11          # FE-5680A short-term ADEV: 1.4e-11·τ^-1/2
RB_FLICKER_FLOOR = 1e-12      # Rb flicker floor (long τ)


def measurement_floor(tau, kind):
    """TICC single-shot (white PM) ⊕ FE-5680A Rb, in quadrature.

    ``kind`` is 'adev' (fractional) or 'tdev' (returns NANOSECONDS).  This is
    the a-priori SPEC model — idealized, omits the Rb long-τ RWFM upturn, so
    it is optimistic at long τ.
    """
    rb_adev = np.maximum(RB_ADEV_1S * tau ** -0.5, RB_FLICKER_FLOOR)
    if kind == 'adev':
        ticc = (np.sqrt(3) * TICC_FLOOR_PS * 1e-12) * tau ** -1.0   # white PM
        return np.sqrt(ticc ** 2 + rb_adev ** 2)
    ticc = (TICC_FLOOR_PS * 1e-12) * tau ** -0.5                    # white PM, s
    rb_tdev = tau * rb_adev / np.sqrt(3)                            # white FM, s
    return np.sqrt(ticc ** 2 + rb_tdev ** 2) * 1e9                  # → ns

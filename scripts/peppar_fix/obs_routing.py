"""Per-filter observation routing helpers.

The engine has two consumers of per-epoch observations:

  - **Position-side**: PPPFilter (float + WL/MW + LAMBDA) and the
    Melbourne-Wubbena tracker.  These need phase biases applied on
    BOTH bands of the IF combination.  When phase bias is MISS on
    one band, ``cp_f1`` (or ``cp_f2``) is left uncorrected at
    ``realtime_ppp.py:1030-1035``, and the half-corrected MW
    combination places WL integer search on a wrong cycle —
    m-scale per-SV bias propagates to multi-meter position basins.

  - **Time-side**: FixedPosFilter via TD-CP residuals
    (``solve_ppp.py:1389-1393``).  TD-CP differences ``phi_if_m``
    epoch-to-epoch — the phase-bias constant cancels — so the time
    filter is insensitive to MISS phase biases (only the
    time-derivative survives, which is mm-scale per minute even
    worst-case AC drift).

Per-filter gate (I-175645-charlie / I-165118-charlie):

  - Position consumers feed through ``obs_for_position(obs_list)``
    and drop SVs with ``ar_phase_bias_ok=False``.
  - Time consumers receive the full observation list unfiltered.

Empirical case: MadHat day0509-madhat-coldstart.log — 6-8 m basin
trap, ZTD residual +1146 mm, 102 BDS [PB_APPLIED] emits with
``src=?`` on L5/B2a-I.  See
``docs/bds-b2a-phase-bias-survey-2026-05-09.md`` for the AC-coverage
analysis showing no AC publishes BDS B2a-I phase biases on IGS-IP
today.
"""
from __future__ import annotations


def obs_for_position(observations):
    """Filter observations for position-side consumers (PPPFilter,
    MW tracker, WL bootstrap).

    Drops SVs whose phase biases are unavailable on either band
    (``ar_phase_bias_ok=False``).  Default ``True`` for the per-obs
    flag preserves legacy / replay paths (no SSR stream → no
    ``ar_phase_bias_ok`` field set) — every obs passes, same as
    before this fix.

    Returns the same dict objects as the input (no copy), so any
    in-place writes by upstream consumers (e.g., the AntPosEstThread
    PCV-correction path at ``peppar_fix_engine.py:2880-2885`` writes
    to ``obs['pr_if']`` before the filter call) land on the same
    dicts the filter then sees.
    """
    return [o for o in observations
            if o.get('ar_phase_bias_ok', True)]

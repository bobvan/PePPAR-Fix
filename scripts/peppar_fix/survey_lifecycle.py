"""Survey-provenance lifecycle for the time-only engine (I-071400 E3).

A *position-provenance* lifecycle, orthogonal to the AR filter's
``AntPosEstState`` (which names per-SV integer-fix convergence).  It tracks how
the engine's pinned position was obtained and how well it is known, per
``docs/time-only-architecture.md``:

    ACQUIRING  — running on a coarse bootstrap seed (NAV2 single-point,
                 σ_r ≳ trust floor).  Raw obs logged for peppar-survey.
    REFINING   — a survey-class estimate has tightened σ_r below the trust
                 floor, but the survey hasn't declared itself converged yet.
                 Raw obs still logged (better products may still land).
    SURVEYED   — the survey is ``converged`` (final multi-day mean stable);
                 hold the pin, stop logging raw obs.  Steady state.

The state is a pure function of (σ_r, converged): σ_r shrinks monotonically as
survey refinements are applied (``_apply_survey_refresh``) and ``converged`` is
a monotonic survey-set flag, so the derivation can't oscillate.  A gross antenna
move is handled out-of-band (mount_sn bump → wrapper respawn → a fresh process
starts back in ACQUIRING), so no back-transition logic is needed here.
"""
from __future__ import annotations

import enum
import logging

log = logging.getLogger("peppar-fix")

# σ_r at/above this (m) is a coarse bootstrap seed, not a survey pin.  Mirrors
# the engine's _TRUSTED_POSITION_SIGMA_M so ACQUIRING↔REFINING flips exactly at
# the "trusted position" boundary the seed resolver already uses.
DEFAULT_TRUSTED_SIGMA_M = 10.0


class SurveyLifecycle(enum.Enum):
    ACQUIRING = "ACQUIRING"
    REFINING = "REFINING"
    SURVEYED = "SURVEYED"


def classify(sigma_r_m, converged, *,
             trusted_sigma_m: float = DEFAULT_TRUSTED_SIGMA_M) -> SurveyLifecycle:
    """Derive the lifecycle state from σ_r (m) and the survey ``converged`` flag.

    SURVEYED iff the survey declared convergence; else REFINING once σ_r is
    inside the trust floor (a survey-class pin); else ACQUIRING (coarse seed).
    A None/invalid σ_r is treated as coarse (ACQUIRING) unless converged.
    """
    if converged:
        return SurveyLifecycle.SURVEYED
    try:
        s = float(sigma_r_m)
    except (TypeError, ValueError):
        return SurveyLifecycle.ACQUIRING
    if s == s and 0 < s < trusted_sigma_m:   # not NaN, positive, inside floor
        return SurveyLifecycle.REFINING
    return SurveyLifecycle.ACQUIRING


class SurveyLifecycleMachine:
    """Thin stateful wrapper: derives the state each tick and logs transitions.

    ``update(sigma_r_m, converged)`` returns ``(state, changed)``; ``changed`` is
    True on the tick a new state is entered (the caller acts on the edge, e.g.
    stops raw logging when SURVEYED is first entered).
    """

    def __init__(self, trusted_sigma_m: float = DEFAULT_TRUSTED_SIGMA_M):
        self._trusted_sigma_m = float(trusted_sigma_m)
        self.state: SurveyLifecycle = SurveyLifecycle.ACQUIRING
        self._seeded = False

    def update(self, sigma_r_m, converged) -> tuple:
        new = classify(sigma_r_m, converged,
                       trusted_sigma_m=self._trusted_sigma_m)
        changed = (not self._seeded) or new is not self.state
        if new is not self.state:
            log.info("[SURVEY_LIFECYCLE] %s → %s (σ_r=%s converged=%s)",
                     self.state.value, new.value,
                     ("%.3fm" % sigma_r_m) if isinstance(sigma_r_m, (int, float))
                     else sigma_r_m, bool(converged))
        elif not self._seeded:
            log.info("[SURVEY_LIFECYCLE] start %s (σ_r=%s converged=%s)",
                     new.value,
                     ("%.3fm" % sigma_r_m) if isinstance(sigma_r_m, (int, float))
                     else sigma_r_m, bool(converged))
        self.state = new
        self._seeded = True
        return new, changed

"""Discipline convergence — the single derived continuous signal every
policy reads (docs/discipline-convergence-state.md, increment #1).

Owns ``distance_to_lock`` ∈ [0, 1] (0 = locked, 1 = far) — a normalized
function of the EKF's honest DO-phase-state uncertainty √(P22).  Instead
of latched discipline modes (acquiring / verifying / tracking) with
edge transitions and dwell hysteresis, every policy reads this one
continuous signal and shapes its behavior as a continuous function of
it: the graded taper (replaces the binary converging/tracking latch and
its 1→max actuation cliff), the gate strength (replaces the OCXO gate
min_age trigger), source weighting (R as f(m)), etc.

A continuous policy of a derived metric **cannot chatter** the way a
thresholded state can — it sidesteps the hysteresis contrivance the v2
regime-gate dwell-latch fell into (Bob, 2026-05-29).

Increment #1 wires only the graded taper consumer; subsequent
increments wire the gate, source weighting, and the thin binary layer
(GNSS present/absent, gross-fault → reset).

`√(P22)` is the filter's honest accumulated DO-phase uncertainty — but
only when Q is set from char (qFromCharPerActuator); with an inflated
Q this signal reads converged too early, the same overconfidence that
defeated the coast-cap and the gate.
"""
from __future__ import annotations

import math

from peppar_fix.discipline import normalized_distance_to_lock


class DisciplineConvergence:
    """Single derived continuous convergence signal.

    Holds ``distance_to_lock`` ∈ [0, 1] (0 = locked, 1 = far) and the
    thresholds that map raw √(P22) onto it.  Bootstrap value 1.0 — at
    startup we are maximally far from lock.

    Thresholds are anchored to the same ``phase_error_budget_ns`` the
    coast-cap uses, so the whole discipline loop speaks one language:
    "within budget" = locked, "well outside budget" = far.  The far
    threshold's multiplier is a sim-tunable parameter (the design doc
    flags it for closedLoopServoSim pinning), not a magic constant.
    """

    def __init__(self, converged_ns, far_ns):
        if not (far_ns > converged_ns >= 0.0):
            raise ValueError(
                f"far_ns must be > converged_ns >= 0; "
                f"got converged={converged_ns}, far={far_ns}")
        self._converged_ns = float(converged_ns)
        self._far_ns = float(far_ns)
        self._distance_to_lock = 1.0

    @property
    def converged_ns(self):
        return self._converged_ns

    @property
    def far_ns(self):
        return self._far_ns

    @property
    def distance_to_lock(self):
        """The latest signal; 0 = locked, 1 = far.  Continuous, monotone
        in √(P22).  Read by every consumer policy."""
        return self._distance_to_lock

    def update_from_p22(self, p22_ns_sq):
        """Recompute the signal from the EKF DO-phase variance P[2,2]
        (units ns²).  Returns the new ``distance_to_lock``.

        Non-finite, negative, or None inputs leave the prior value
        unchanged — the signal degrades gracefully through transient
        bad reads (rather than jumping to 0 or 1 spuriously, which a
        consumer policy would amplify).
        """
        if p22_ns_sq is None:
            return self._distance_to_lock
        try:
            v = float(p22_ns_sq)
        except (TypeError, ValueError):
            return self._distance_to_lock
        if not math.isfinite(v) or v < 0.0:
            return self._distance_to_lock
        self._distance_to_lock = normalized_distance_to_lock(
            math.sqrt(v), self._converged_ns, self._far_ns)
        return self._distance_to_lock

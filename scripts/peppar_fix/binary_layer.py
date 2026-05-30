"""Discipline binary layer — the discrete-event tier above the
continuous DisciplineConvergence signal (disciplineModeFsm increment
#4, design: docs/discipline-convergence-state.md).

The continuous signal cannot distinguish:
  "drifting because GNSS is gone" (holdover, expected; the loop is
                                   coasting on the freq model)
  "drifting because the filter is broken" (gross fault, needs a reset)

Both look identical to the convergence metric: P[2,2] grows, so
distance_to_lock → 1.  The binary layer is what separates them: the
GNSS-present/absent flag tells the gross-fault detector to suppress
during holdover (where P growing is expected, not a fault) and to fire
in the non-holdover case (where it really is a divergence).

The fired action is an **in-process reset** of the filter state —
re-init the EKF's x/P/internal counters; preserve the actuator
command (the DO continues coasting at its last good freq while the
filter rebuilds).  This replaces the exit-5 wrapper-relaunch path with
something the engine survives.  Per Bob's design preference, the
binary layer is a "honest exception" — a *little* machine where the
world is genuinely binary, not a gradient.
"""
from __future__ import annotations

from typing import Optional


class BinaryLayer:
    """Gross-fault detector + in-process reset trigger.

    Watches the derived ``distance_to_lock`` signal.  When sustained at
    its maximum (1.0) for ``consec_max_epochs`` epochs *and* the
    engine is not in holdover, the filter has effectively diverged —
    emit a ``"gross_fault"`` event for the caller to act on.

    Default-OFF (``enabled=False``); opt-in via engine flag.  The
    caller is responsible for performing the reset (calling
    ``DOFreqEst.reset()``, ``DisciplineConvergence.reset()``, logging
    a ``[GROSS_FAULT_RESET]`` line, etc.) and then invoking
    :meth:`clear_after_reset` so the counter is consistent.

    Threshold rationale (zero new magic constants): distance_to_lock
    is already anchored to ``phase_error_budget_ns *
    lock_far_budget_ratio`` (#92), so "saturated at 1.0" means
    ``√(P22) >= far_ns`` — the same scale every other discipline
    policy already speaks in.  Sustaining is the only knob this layer
    introduces; expose it as ``--gross-fault-consec-epochs``.

    Suppression during holdover is the load-bearing piece: without it,
    every holdover would trip the fault (P grows freely with no
    measurement updates).  The caller passes ``in_holdover`` from the
    existing HoldoverActor (HoldoverMode.HOLDOVER predicate).
    """

    def __init__(self, *, consec_max_epochs: int = 60,
                 enabled: bool = False):
        if consec_max_epochs <= 0:
            raise ValueError(
                f"consec_max_epochs must be > 0, got {consec_max_epochs}")
        self._consec_max = int(consec_max_epochs)
        self._consec = 0
        self.enabled = bool(enabled)
        self.n_gross_fault_resets = 0
        self.max_consec = 0  # high-water-mark for diagnostics

    @property
    def consec(self) -> int:
        """Current consecutive-at-max count (read-only, for observability)."""
        return self._consec

    @property
    def consec_max_epochs(self) -> int:
        """Configured trigger threshold (consecutive epochs at max)."""
        return self._consec_max

    def evaluate(self, distance_to_lock: Optional[float],
                 in_holdover: bool) -> Optional[str]:
        """Per-epoch evaluation.

        Returns ``"gross_fault"`` when the caller should reset; ``None``
        otherwise.  Does NOT auto-clear after returning the trigger —
        caller must invoke :meth:`clear_after_reset` once the reset
        has happened, to prevent re-firing on the next epoch.

        Disabled or in-holdover ⇒ counter is reset to 0 (no spurious
        carry-over after the holdover or after toggling the feature).
        """
        if not self.enabled:
            self._consec = 0
            return None
        if in_holdover:
            self._consec = 0
            return None
        if distance_to_lock is None:
            # Signal not available yet (bootstrap / --graded-taper off)
            # → don't accrue a fault count from a missing signal.
            self._consec = 0
            return None
        # 1e-9 tolerance: distance_to_lock comes from a clamp at 1.0 in
        # normalized_distance_to_lock; a strict == comparison is safe
        # in practice but the epsilon costs nothing and survives any
        # downstream float-noise.
        if distance_to_lock >= 1.0 - 1e-9:
            self._consec += 1
            if self._consec > self.max_consec:
                self.max_consec = self._consec
            if self._consec >= self._consec_max:
                return "gross_fault"
        else:
            self._consec = 0
        return None

    def clear_after_reset(self) -> None:
        """Caller signals "reset done, start a fresh window."

        Increments the cumulative reset counter and clears the
        consecutive-at-max state so the next epoch starts fresh.
        """
        self._consec = 0
        self.n_gross_fault_resets += 1

    @property
    def stats(self) -> dict:
        return dict(
            enabled=self.enabled,
            consec_max_epochs=self._consec_max,
            consec=self._consec,
            max_consec=self.max_consec,
            n_gross_fault_resets=self.n_gross_fault_resets,
        )

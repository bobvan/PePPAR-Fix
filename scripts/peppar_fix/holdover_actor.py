"""HoldoverActor — state machine for TDCP-extended holdover.

Manages TRACKING → HOLDOVER → REACQUIRED → TRACKING transitions.
Coordinates DoVsRxTcxoOffset (PR 1/4) and TdcpPhaseIntegrator (PR 2/4)
to produce x[2]_pseudo during holdover and blend back to hardware
observations on reacquisition.

See docs/tdcp-holdover-anchor.md for the full design.
"""

import enum
import math
from typing import NamedTuple


class HoldoverMode(enum.Enum):
    TRACKING = "TRACKING"
    HOLDOVER = "HOLDOVER"
    REACQUIRED = "REACQUIRED"


class HoldoverTransition(NamedTuple):
    """Emitted on each mode change for logging/telemetry."""
    prev: HoldoverMode
    curr: HoldoverMode
    holdover_duration_s: float
    sigma_ns: float


# σ thresholds for PTP clockClass (from Phase D spec).
SIGMA_CLOCKCLASS_6 = 0.354   # ns — within moonshot per-clock budget
SIGMA_CLOCKCLASS_7 = 1.0     # ns — within excursion bound
SIGMA_CLOCKCLASS_52 = 10.0   # ns — degraded holdover


class HoldoverActor:
    """State machine for TDCP-extended holdover.

    This actor does NOT own DoVsRxTcxoOffset or TdcpPhaseIntegrator
    instances — the engine wiring layer (PR 4/4) creates those and
    passes data through this actor's methods.  This keeps the actor
    testable with mocks.
    """

    def __init__(
        self,
        t_enter_s: float = 5.0,
        t_blend_s: float = 30.0,
        sigma_blend_done_ns2: float = 1.0,
        sigma_clockclass_7_ns: float = SIGMA_CLOCKCLASS_6,
        sigma_clockclass_52_ns: float = SIGMA_CLOCKCLASS_7,
        sigma_freerun_ns: float = SIGMA_CLOCKCLASS_52,
    ):
        self._t_enter_s = t_enter_s
        self._t_blend_s = t_blend_s
        self._sigma_blend_done_ns2 = sigma_blend_done_ns2
        self._sigma_cc7 = sigma_clockclass_7_ns
        self._sigma_cc52 = sigma_clockclass_52_ns
        self._sigma_freerun = sigma_freerun_ns

        self._mode = HoldoverMode.TRACKING
        self._arms_absent_since: float | None = None
        self._reacquired_since: float | None = None
        self._frozen_offset_ns: float | None = None
        self._last_sigma_ns: float = 0.0
        self._last_transition: HoldoverTransition | None = None

    @property
    def mode(self) -> HoldoverMode:
        return self._mode

    @property
    def last_transition(self) -> HoldoverTransition | None:
        return self._last_transition

    @property
    def frozen_offset_ns(self) -> float | None:
        return self._frozen_offset_ns

    @property
    def last_sigma_ns(self) -> float:
        return self._last_sigma_ns

    def clock_class(self) -> int:
        """Map current σ to IEEE 1588 clockClass."""
        if self._mode == HoldoverMode.TRACKING:
            return 6
        s = self._last_sigma_ns
        if s <= self._sigma_cc7:
            return 6
        if s <= self._sigma_cc52:
            return 7
        if s <= self._sigma_freerun:
            return 52
        return 248

    def tick(
        self,
        hw_arm_healthy: bool,
        mono_s: float,
        offset_ns: float | None = None,
        offset_ready: bool = False,
        holdover_sigma_ns: float = 0.0,
        p22: float = 0.0,
    ) -> HoldoverTransition | None:
        """Advance the state machine by one epoch.

        Args:
            hw_arm_healthy: True if Arm 3 or Arm 4 delivered a valid
                observation this epoch.
            mono_s: CLOCK_MONOTONIC timestamp.
            offset_ns: current DoVsRxTcxoOffset.last_offset_ns (needed
                at TRACKING → HOLDOVER transition to freeze).
            offset_ready: DoVsRxTcxoOffset.ready (≥10 samples).
            holdover_sigma_ns: predicted σ from
                DoVsRxTcxoOffset.holdover_sigma_ns() — only meaningful
                during HOLDOVER.
            p22: current P[2,2] from the EKF (for REACQUIRED exit).

        Returns:
            A HoldoverTransition if a mode change occurred, else None.
        """
        prev = self._mode
        transition = None

        if self._mode == HoldoverMode.TRACKING:
            transition = self._tick_tracking(hw_arm_healthy, mono_s,
                                             offset_ns, offset_ready)
        elif self._mode == HoldoverMode.HOLDOVER:
            self._last_sigma_ns = holdover_sigma_ns
            transition = self._tick_holdover(hw_arm_healthy, mono_s)
        elif self._mode == HoldoverMode.REACQUIRED:
            transition = self._tick_reacquired(hw_arm_healthy, mono_s, p22)

        if transition is not None:
            self._last_transition = transition
        return transition

    def _tick_tracking(self, hw_healthy: bool, mono_s: float,
                       offset_ns: float | None,
                       offset_ready: bool) -> HoldoverTransition | None:
        if hw_healthy:
            self._arms_absent_since = None
            return None

        if self._arms_absent_since is None:
            self._arms_absent_since = mono_s
            return None

        elapsed = mono_s - self._arms_absent_since
        if elapsed < self._t_enter_s:
            return None

        if offset_ns is not None and offset_ready:
            self._frozen_offset_ns = offset_ns
            self._mode = HoldoverMode.HOLDOVER
            self._last_sigma_ns = 0.0
            self._arms_absent_since = None
            return HoldoverTransition(
                prev=HoldoverMode.TRACKING,
                curr=HoldoverMode.HOLDOVER,
                holdover_duration_s=0.0,
                sigma_ns=0.0,
            )

        return None

    def _tick_holdover(self, hw_healthy: bool,
                       mono_s: float) -> HoldoverTransition | None:
        if not hw_healthy:
            return None

        self._mode = HoldoverMode.REACQUIRED
        self._reacquired_since = mono_s
        sigma = self._last_sigma_ns
        return HoldoverTransition(
            prev=HoldoverMode.HOLDOVER,
            curr=HoldoverMode.REACQUIRED,
            holdover_duration_s=0.0,
            sigma_ns=sigma,
        )

    def _tick_reacquired(self, hw_healthy: bool, mono_s: float,
                         p22: float) -> HoldoverTransition | None:
        if not hw_healthy:
            self._mode = HoldoverMode.HOLDOVER
            self._reacquired_since = None
            return HoldoverTransition(
                prev=HoldoverMode.REACQUIRED,
                curr=HoldoverMode.HOLDOVER,
                holdover_duration_s=0.0,
                sigma_ns=self._last_sigma_ns,
            )

        if p22 <= self._sigma_blend_done_ns2:
            self._mode = HoldoverMode.TRACKING
            self._reacquired_since = None
            self._frozen_offset_ns = None
            self._last_sigma_ns = 0.0
            return HoldoverTransition(
                prev=HoldoverMode.REACQUIRED,
                curr=HoldoverMode.TRACKING,
                holdover_duration_s=0.0,
                sigma_ns=0.0,
            )

        if self._reacquired_since is not None:
            elapsed = mono_s - self._reacquired_since
            if elapsed >= self._t_blend_s:
                self._mode = HoldoverMode.TRACKING
                self._reacquired_since = None
                self._frozen_offset_ns = None
                self._last_sigma_ns = 0.0
                return HoldoverTransition(
                    prev=HoldoverMode.REACQUIRED,
                    curr=HoldoverMode.TRACKING,
                    holdover_duration_s=0.0,
                    sigma_ns=math.sqrt(max(0.0, p22)),
                )

        return None

    @property
    def in_holdover(self) -> bool:
        return self._mode == HoldoverMode.HOLDOVER

    @property
    def in_reacquired(self) -> bool:
        return self._mode == HoldoverMode.REACQUIRED

    @property
    def wants_pseudo_obs(self) -> bool:
        """True when the engine should inject x[2]_pseudo into the EKF."""
        return self._mode in (HoldoverMode.HOLDOVER, HoldoverMode.REACQUIRED)

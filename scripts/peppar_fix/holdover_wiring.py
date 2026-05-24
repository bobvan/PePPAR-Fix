"""Phase D engine wiring — connects holdover components to the servo loop.

Keeps all Phase D holdover integration in one module rather than
scattering it across the 10K-line engine.  The engine calls two
entry points:

    init_holdover(args) → HoldoverState
    tick_holdover(state, servo, tdcp_freq, mono, args) → dict

See docs/tdcp-holdover-anchor.md for the design.
"""

import logging
import math
import sys
from typing import NamedTuple

from peppar_fix.do_vs_rxtcxo_offset import DoVsRxTcxoOffset
from peppar_fix.holdover_actor import HoldoverActor, HoldoverMode
from peppar_fix.tdcp_phase_integrator import TdcpPhaseIntegrator

log = logging.getLogger(__name__)

EXIT_NO_ANCHOR = 7


class HoldoverState:
    """Mutable state bag for the holdover subsystem."""

    def __init__(self, offset_cap, integrator, actor):
        self.offset_cap: DoVsRxTcxoOffset = offset_cap
        self.integrator: TdcpPhaseIntegrator = integrator
        self.actor: HoldoverActor = actor
        self.holdover_entry_mono: float | None = None


def init_holdover(args) -> HoldoverState | None:
    """Create Phase D holdover objects from CLI args.

    Returns None if holdover is not enabled (no servo or no TDCP).
    """
    if not getattr(args, 'servo', None):
        return None
    if getattr(args, 'servo_input', 'default') != 'tdcp':
        return None
    if getattr(args, 'no_tdcp_arm', False):
        return None

    offset_cap = DoVsRxTcxoOffset(
        tau_s=getattr(args, 'holdover_offset_tau_s', 120.0),
        measured_bias_ppb=getattr(args, 'holdover_bias_ppb', 0.0),
    )
    integrator = TdcpPhaseIntegrator(
        max_dt_s=getattr(args, 'holdover_max_gap_s', 1.5),
    )
    actor = HoldoverActor(
        t_enter_s=getattr(args, 'holdover_t_enter_s', 5.0),
        t_blend_s=getattr(args, 'holdover_t_blend_s', 30.0),
        sigma_blend_done_ns2=getattr(args, 'holdover_sigma_blend_done_ns2', 1.0),
        sigma_max_cc6_ns=getattr(args, 'holdover_sigma_cc6_ns', 0.354),
        sigma_max_cc7_ns=getattr(args, 'holdover_sigma_cc7_ns', 1.0),
        sigma_max_cc52_ns=getattr(args, 'holdover_sigma_cc52_ns', 10.0),
    )
    return HoldoverState(offset_cap, integrator, actor)


class TickResult(NamedTuple):
    """What the engine needs after each holdover tick."""
    pseudo_obs_ns: float | None
    pseudo_obs_sigma_ns: float | None
    clock_class: int
    holdover_sigma_ns: float
    mode: HoldoverMode


def tick_holdover(
    state: HoldoverState,
    servo,
    tdcp_freq: float | None,
    tdcp_sigma_ppb: float,
    mono_s: float,
    hw_arm_healthy: bool,
) -> TickResult:
    """Advance the holdover subsystem by one servo epoch.

    Called after servo.update() each epoch.  Returns a TickResult
    telling the engine whether to inject a pseudo-obs.

    Args:
        state: the HoldoverState from init_holdover().
        servo: the DOFreqEst instance (for reading x[], P[]).
        tdcp_freq: latest TDCP df_f (dimensionless), or None.
        tdcp_sigma_ppb: per-host σ_y for holdover_sigma_ns().
        mono_s: CLOCK_MONOTONIC now.
        hw_arm_healthy: True if Arm 3 or Arm 4 delivered this epoch.
    """
    x = servo.x
    P = servo.P
    actor = state.actor
    offset_cap = state.offset_cap
    integrator = state.integrator

    # During TRACKING with healthy arms: feed the offset EMA.
    # Only when hardware arms are delivering observations is x[2]
    # well-determined enough to calibrate the DO-vs-rx_TCXO offset.
    if actor.mode == HoldoverMode.TRACKING and hw_arm_healthy:
        offset_cap.update(
            x0_phi_rx_ns=x[0],
            x2_phi_do_ns=x[2],
            p00=P[0, 0],
            p22=P[2, 2],
            mono_s=mono_s,
        )

    # Compute holdover duration from wall-clock, not integrator.
    if state.holdover_entry_mono is not None:
        holdover_dur = mono_s - state.holdover_entry_mono
    else:
        holdover_dur = 0.0

    holdover_sigma = offset_cap.holdover_sigma_ns(
        holdover_dur, tdcp_sigma_y_ppb=tdcp_sigma_ppb,
    )

    transition = actor.tick(
        hw_arm_healthy=hw_arm_healthy,
        mono_s=mono_s,
        offset_ns=offset_cap.last_offset_ns,
        offset_ready=offset_cap.ready,
        holdover_sigma_ns=holdover_sigma,
        p22=P[2, 2],
    )

    if transition is not None:
        _log_transition(transition, holdover_dur, actor)

        if transition.curr == HoldoverMode.HOLDOVER:
            integrator.seed(x0_phi_rx_ns=x[0], mono_s=mono_s)
            state.holdover_entry_mono = mono_s
            log.info(
                "[HOLDOVER] entry: frozen_offset=%.3f ns, σ_offset=%.3f ns",
                actor.frozen_offset_ns or 0.0,
                offset_cap.last_offset_sigma_ns,
            )

        elif transition.curr == HoldoverMode.TRACKING:
            integrator.reset()
            state.holdover_entry_mono = None

    # During HOLDOVER or REACQUIRED: integrate TDCP and produce pseudo-obs.
    pseudo_obs_ns = None
    pseudo_obs_sigma_ns = None

    if actor.wants_pseudo_obs:
        if tdcp_freq is not None:
            ok = integrator.integrate(df_f=tdcp_freq, mono_s=mono_s)
            if not ok and not integrator.gap_detected:
                pass
            elif integrator.gap_detected:
                log.warning(
                    "[HOLDOVER] TDCP gap — Arm 6 muted until REACQUIRE "
                    "(gap at %.1f s into holdover)",
                    holdover_dur,
                )

        frozen = actor.frozen_offset_ns
        pseudo = integrator.pseudo_obs(offset_ns=frozen)
        if pseudo is not None:
            pseudo_obs_ns = pseudo
            pseudo_obs_sigma_ns = max(holdover_sigma, 0.1)

    return TickResult(
        pseudo_obs_ns=pseudo_obs_ns,
        pseudo_obs_sigma_ns=pseudo_obs_sigma_ns,
        clock_class=actor.clock_class(),
        holdover_sigma_ns=holdover_sigma,
        mode=actor.mode,
    )


def _log_transition(transition, holdover_dur, actor):
    """Log a holdover mode transition."""
    if transition.curr == HoldoverMode.HOLDOVER:
        if transition.prev == HoldoverMode.TRACKING:
            log.warning(
                "[STATE] holdover: TRACKING → HOLDOVER "
                "(arms absent > T_enter)",
            )
        else:
            log.warning(
                "[STATE] holdover: REACQUIRED → HOLDOVER "
                "(arms lost during blend)",
            )
    elif transition.curr == HoldoverMode.REACQUIRED:
        log.info(
            "[STATE] holdover: HOLDOVER → REACQUIRED "
            "(arms returned after %.1f s holdover, σ=%.3f ns, cc=%d)",
            holdover_dur,
            transition.sigma_ns,
            actor.clock_class(),
        )
    elif transition.curr == HoldoverMode.TRACKING:
        log.info(
            "[STATE] holdover: REACQUIRED → TRACKING "
            "(P[2,2]=%.3e, blend complete)",
            transition.sigma_ns ** 2 if transition.sigma_ns > 0 else 0.0,
        )


def add_holdover_args(servo_group):
    """Add Phase D CLI flags to the servo argument group."""
    servo_group.add_argument(
        "--holdover-offset-tau-s", type=float, default=120.0,
        help="DoVsRxTcxoOffset EMA time constant (seconds).  "
             "Controls how quickly the DO-vs-rx_TCXO offset "
             "calibration tracks real drift during TRACKING.  "
             "Range 60-300 s recommended.  Default 120.")
    servo_group.add_argument(
        "--holdover-bias-ppb", type=float, default=0.0,
        help="Per-host TDCP bias from the prerequisite measurement "
             "(ppb).  Contributes a linear bias×T term to the "
             "holdover σ calculation.  Default 0.0 (no measured bias).")
    servo_group.add_argument(
        "--holdover-max-gap-s", type=float, default=1.5,
        help="TdcpPhaseIntegrator gap protection threshold (seconds).  "
             "If inter-epoch dt exceeds this, the integrator marks a "
             "gap and pseudo_obs returns None.  Default 1.5.")
    servo_group.add_argument(
        "--holdover-t-enter-s", type=float, default=5.0,
        help="Arms 3+4 must be absent for this many seconds before "
             "entering HOLDOVER.  Default 5.")
    servo_group.add_argument(
        "--holdover-t-blend-s", type=float, default=30.0,
        help="REACQUIRED → TRACKING blend timeout (seconds).  "
             "If P[2,2] doesn't drop below sigma_blend_done in this "
             "time, force transition to TRACKING anyway.  Default 30.")
    servo_group.add_argument(
        "--holdover-sigma-blend-done-ns2", type=float, default=1.0,
        help="P[2,2] threshold (ns²) for REACQUIRED → TRACKING.  "
             "Default 1.0 ns² (= 1 ns σ on DO phase).")
    servo_group.add_argument(
        "--holdover-sigma-cc6-ns", type=float, default=0.354,
        help="Max σ for clockClass 6 (locked).  Default 0.354 ns "
             "(moonshot per-clock budget).")
    servo_group.add_argument(
        "--holdover-sigma-cc7-ns", type=float, default=1.0,
        help="Max σ for clockClass 7 (holdover).  Default 1.0 ns "
             "(shared-antenna excursion bound).")
    servo_group.add_argument(
        "--holdover-sigma-cc52-ns", type=float, default=10.0,
        help="Max σ for clockClass 52 (degraded).  Default 10.0 ns.")

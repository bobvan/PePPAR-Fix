"""AntPosEst and DOFreqEst state machines with structured transition logging."""

import enum
import logging
import time

log = logging.getLogger("peppar_fix.states")


class AntPosEstState(enum.Enum):
    UNSURVEYED = "unsurveyed"
    VERIFYING = "verifying"
    VERIFIED = "verified"
    CONVERGING = "converging"
    RESOLVED = "resolved"
    MOVED = "moved"


class DOFreqEstState(enum.Enum):
    UNINITIALIZED = "uninitialized"
    PHASE_SETTING = "phase_setting"
    FREQ_VERIFYING = "freq_verifying"
    TRACKING = "tracking"
    HOLDOVER = "holdover"


class StateMachine:
    """Base state machine with structured transition logging."""

    def __init__(self, name, initial_state):
        self.name = name
        self.state = initial_state
        self._entered_at = time.monotonic()
        self._transition_count = 0
        log.info("[STATE] %s: → %s (initial)", name, initial_state.value)

    def transition(self, new_state, reason=""):
        old = self.state
        if new_state == old:
            return
        elapsed = time.monotonic() - self._entered_at
        self.state = new_state
        self._entered_at = time.monotonic()
        self._transition_count += 1
        reason_suffix = f" ({reason})" if reason else ""
        log.info("[STATE] %s: %s → %s after %.0fs%s",
                 self.name, old.value, new_state.value, elapsed, reason_suffix)

    @property
    def elapsed_in_state(self):
        return time.monotonic() - self._entered_at


class AntPosEst(StateMachine):
    """Antenna Position Estimator state machine.

    Carries an additional `reached_resolved` latch.  Semantics:

      * False on construction (filter has no earned position yet).
      * Latches True the first time the machine enters RESOLVED.
        That transition means ≥ N NL_LONG_FIXED members, each of
        which survived ≥ 8° of geometry change — a hard-won
        invariant signalling that the position solution is
        defensible.
      * Cleared only by FixSetIntegrityAlarm.record_fire() — the
        explicit "throw everything out and rebuild from scratch"
        event.  Ordinary anchor-count decay does NOT clear it;
        losing the SV anchors means we need a *different* defense
        (position-anchored join test, anchor-collapse trigger),
        not a bootstrap restart.

    Why here (not on SvStateTracker): reached_resolved is a host-
    level property of the position-solution state, not a per-SV
    fact.  The state machine is already the natural home for
    position-solution semantics; NarrowLaneResolver and
    FixSetIntegrityAlarm both receive a reference to this object
    so they can read the latch for gate selection / anchor-
    collapse detection.
    """

    def __init__(self):
        super().__init__("AntPosEst", AntPosEstState.UNSURVEYED)
        self.sigma_m = None
        self.n_wl_fixed = 0
        self.n_nl_fixed = 0
        self.n_sv_total = 0
        self.reached_resolved = False

    def transition(self, new_state, reason=""):
        super().transition(new_state, reason)
        # Latch on the first entry to RESOLVED.  Base class emits
        # the [STATE] log line; we don't need to log the latch
        # separately because every True-transition coincides with
        # an AntPosEst: ... → resolved line.
        if self.state is AntPosEstState.RESOLVED:
            self.reached_resolved = True

    def clear_reached_resolved(self, reason: str = "") -> None:
        """Called by FixSetIntegrityAlarm after re-init.  Don't call
        from anywhere else — the latch's whole point is that ordinary
        state changes don't reset it."""
        if self.reached_resolved:
            log.info("[STATE] AntPosEst: reached_resolved cleared (%s)",
                     reason or "no reason given")
            self.reached_resolved = False

    def update_metrics(self, sigma_m=None, n_wl=None, n_nl=None, n_sv=None):
        if sigma_m is not None:
            self.sigma_m = sigma_m
        if n_wl is not None:
            self.n_wl_fixed = n_wl
        if n_nl is not None:
            self.n_nl_fixed = n_nl
        if n_sv is not None:
            self.n_sv_total = n_sv

    def status_str(self):
        parts = [f"AntPosEst={self.state.value}"]
        if self.sigma_m is not None:
            parts.append(f"σ={self.sigma_m:.2f}m")
        if self.n_wl_fixed or self.n_nl_fixed:
            parts.append(f"{self.n_wl_fixed} WL")
            parts.append(f"{self.n_nl_fixed} NL")
        return "(" + ", ".join(parts[1:]) + ")" if len(parts) > 1 else ""


class DOFreqEst(StateMachine):
    """DO Frequency Estimator state machine."""

    def __init__(self):
        super().__init__("DOFreqEst", DOFreqEstState.UNINITIALIZED)
        self.adj_ppb = None
        self.err_ns = None
        self.interval = None
        self.n_sv_dual = 0

    def update_metrics(self, adj_ppb=None, err_ns=None, interval=None,
                       n_sv_dual=None):
        if adj_ppb is not None:
            self.adj_ppb = adj_ppb
        if err_ns is not None:
            self.err_ns = err_ns
        if interval is not None:
            self.interval = interval
        if n_sv_dual is not None:
            self.n_sv_dual = n_sv_dual

    def status_str(self):
        parts = [f"DOFreqEst={self.state.value}"]
        if self.adj_ppb is not None:
            parts.append(f"adj={self.adj_ppb:+.1f}ppb")
        if self.err_ns is not None:
            parts.append(f"err={self.err_ns:+.1f}ns")
        if self.interval is not None:
            parts.append(f"interval={self.interval}")
        return "(" + ", ".join(parts[1:]) + ")" if len(parts) > 1 else ""


def format_status(ape, dfe, ticc_ok=None, qvir=None):
    """Format the periodic [STATUS] line from both state machines."""
    parts = [
        f"AntPosEst={ape.state.value}{ape.status_str()}",
        f"DOFreqEst={dfe.state.value}{dfe.status_str()}",
    ]
    if ape.n_sv_total:
        sv_str = f"SVs={ape.n_sv_total}"
        if dfe.n_sv_dual:
            sv_str += f"(dual={dfe.n_sv_dual})"
        parts.append(sv_str)
    extras = []
    if ticc_ok is not None:
        extras.append(f"TICC={'ok' if ticc_ok else 'no'}")
    if qvir is not None:
        extras.append(f"qVIR={qvir:.1f}")
    if extras:
        parts.append(" ".join(extras))
    return " ".join(parts)

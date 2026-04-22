"""AntPosEst and DOFreqEst state machines with structured transition logging."""

import enum
import logging
import time

log = logging.getLogger("peppar_fix.states")


class AntPosEstState(enum.Enum):
    """Filter-level lifecycle.  Participle rule: present participle
    (``-ING``) names active work, past participle (``-ED``) names a
    milestone reached.

    Forward progression:

        SURVEYING → VERIFYING → CONVERGING → ANCHORING → ANCHORED

    ``ANCHORING`` entered when ≥ 4 NL-fixed SVs exist (fallback:
    any combination of ``SvAmbState.ANCHORING`` and
    ``SvAmbState.ANCHORED``).  Filter-level ``ANCHORED`` entered
    when ≥ 4 ``SvAmbState.ANCHORED`` SVs exist concurrently —
    each having survived the ≥ 8° Δaz geometry validation, the
    "we've truly earned this position" milestone.

    ``MOVED`` is a separate branch used when the position-
    discontinuity detector trips (antenna physically relocated) —
    position state is discarded and ``SURVEYING`` restarts the
    cycle.  Integrity-monitor trips do NOT go to ``MOVED``; they
    fall back to ``CONVERGING`` with position retained.
    """
    SURVEYING = "surveying"
    VERIFYING = "verifying"
    CONVERGING = "converging"
    ANCHORING = "anchoring"
    ANCHORED = "anchored"
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

    Carries two monotonic latches, separately answering:

      * ``reached_anchoring`` — "Has this filter ever produced
        integer fixes?"  Latches on first entry to ``ANCHORING``.
        Equivalent to the old ``reached_resolved`` field —
        fallback-count RESOLVED in pre-rename vocabulary, ≥ 4
        NL-fixed members (short or long) in new vocabulary.
      * ``reached_anchored`` — "Has this filter ever been
        geometry-validated?"  Latches on first entry to filter-
        level ``ANCHORED`` (≥ 4 ``SvAmbState.ANCHORED`` SVs
        concurrently, each having survived ≥ 8° Δaz).  The
        hard-won milestone that signals the position solution
        is defensible.

    Both are cleared only by ``clear_latches()`` — called from
    ``FixSetIntegrityMonitor.record_trip()``, the explicit
    "throw everything out and rebuild from scratch" event.
    Ordinary state regressions (ANCHORED → ANCHORING → CONVERGING
    on slip storm) do NOT clear them; losing anchors means we
    need a different defense (position-anchored join test,
    anchor-collapse trigger), not a bootstrap restart.

    The ``reached_anchored`` latch in particular gates the
    anchor-collapse trigger — firing the monitor only on filters
    that have genuinely earned the anchored state.  Gating on
    ``reached_anchoring`` instead would spuriously trip whenever
    the fallback count dropped to zero, which is the day0421f
    pattern that ``858f7da`` corrected.

    Why here (not on SvStateTracker): both latches are host-level
    properties of the position-solution state, not per-SV facts.
    The state machine is the natural home.
    """

    def __init__(self):
        super().__init__("AntPosEst", AntPosEstState.SURVEYING)
        self.sigma_m = None
        self.n_wl_fixed = 0
        self.n_nl_fixed = 0
        self.n_sv_total = 0
        self.reached_anchoring = False
        self.reached_anchored = False

    def transition(self, new_state, reason=""):
        super().transition(new_state, reason)
        # Latch on first entry to each milestone state.  Base-class
        # ``transition`` emits the ``[STATE]`` log line; we don't
        # log the latch separately because each True-transition
        # coincides with an AntPosEst: ... → anchoring / anchored
        # line.
        if self.state is AntPosEstState.ANCHORING:
            self.reached_anchoring = True
        elif self.state is AntPosEstState.ANCHORED:
            # Entering ANCHORED implies we must have passed
            # through ANCHORING, so belt-and-suspenders: latch
            # reached_anchoring too in case a code path ever
            # skips ANCHORING directly.
            self.reached_anchoring = True
            self.reached_anchored = True

    def clear_latches(self, reason: str = "") -> None:
        """Called by FixSetIntegrityMonitor after re-init.  Don't
        call from anywhere else — the latches' whole point is
        that ordinary state regressions don't reset them.

        Logs a single line covering whichever latches were set,
        for operator visibility when tracing a re-init.
        """
        cleared = []
        if self.reached_anchored:
            cleared.append("reached_anchored")
            self.reached_anchored = False
        if self.reached_anchoring:
            cleared.append("reached_anchoring")
            self.reached_anchoring = False
        if cleared:
            log.info("[STATE] AntPosEst: %s cleared (%s)",
                     " + ".join(cleared),
                     reason or "no reason given")

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

"""Per-SV integer-fix lifecycle state machine.

Implements the per-SV state machine specified in
`docs/sv-lifecycle-and-pfr-split.md`.  Orthogonal to the AntPosEst
solution state; answers "does this specific satellite's integer
ambiguity contribute to the fix set?" rather than "is the antenna
position trustworthy?".

Six states form this lifecycle:

    TRACKING ──admit──► FLOAT ──MW──► WL_FIXED ──LAMBDA──► NL_SHORT_FIXED ──Δaz≥8°──► NL_LONG_FIXED
                         ▲              ▲                      │ ▲                          │
                         │              │                      │ │  false-fix rejection     │
                         │              │                      │ │  or setting-SV drop      │
                         │              │                      │ │  or slip (LOW)           │
                         │              │ slip (LOW)           │ └──────────────────────────┘
                         │              ◄──────────────────────┤
                         │                                     ▼
                         │               slip (HIGH)   ┌──────────────┐  cooldown expires
                         └───────────────────◄─────────┤   SQUELCHED  ├────────► FLOAT
                                                       └──────────────┘

Key names:
  * TRACKING       — receiver sees the SV but no observations have yet
                     passed the admit gate (elevation + health + constellation).
  * FLOAT          — admitted; MW tracker accumulating; no integer fix.
  * WL_FIXED       — integer wide-lane fix landed; NL still float.
  * NL_SHORT_FIXED — integer NL fix accepted; short-term member of the fix
                     set; contributes to the position solution but does NOT
                     count toward the solution's RESOLVED declaration.
  * NL_LONG_FIXED  — integer NL fix has survived ≥8° of satellite azimuth
                     motion without a false-fix rejection; long-term member
                     of the fix set; counts toward RESOLVED.
  * SQUELCHED      — temporarily excluded from fix attempts after a high-
                     confidence cycle slip; cooldown-bound, not permanent.

Transitions not in this diagram are illegal and raise `InvalidTransition`.
The tracker logs every legal transition as one
`[SV_STATE] <sv>: <from> → <to> (epoch=N, elev=X°, reason=...)` line —
grep-friendly.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

log = logging.getLogger(__name__)


class SvAmbState(Enum):
    """Per-SV integer-fix state — one value per (system, PRN)."""
    TRACKING = "TRACKING"
    FLOAT = "FLOAT"
    WL_FIXED = "WL_FIXED"
    NL_SHORT_FIXED = "NL_SHORT_FIXED"
    NL_LONG_FIXED = "NL_LONG_FIXED"
    SQUELCHED = "SQUELCHED"


# Legal directed edges.  Built once at import; `transition()` consults it.
_LEGAL_EDGES: frozenset[tuple[SvAmbState, SvAmbState]] = frozenset({
    # Admit: receiver-tracked SV passes the elevation/health/constellation
    # gate and enters processing.
    (SvAmbState.TRACKING,        SvAmbState.FLOAT),

    # Normal progression.
    (SvAmbState.FLOAT,           SvAmbState.WL_FIXED),
    (SvAmbState.WL_FIXED,        SvAmbState.NL_SHORT_FIXED),
    (SvAmbState.NL_SHORT_FIXED,  SvAmbState.NL_LONG_FIXED),

    # False-fix rejection, setting-SV drop, or slip (LOW-confidence)
    # demotes back toward FLOAT.
    (SvAmbState.WL_FIXED,        SvAmbState.FLOAT),
    (SvAmbState.NL_SHORT_FIXED,  SvAmbState.FLOAT),
    (SvAmbState.NL_LONG_FIXED,   SvAmbState.FLOAT),

    # Squelch: HIGH-confidence cycle slip from any processing state.
    # TRACKING is pre-processing, so no squelch there — a slip before
    # admit is not meaningful (no integer state to protect).
    (SvAmbState.FLOAT,           SvAmbState.SQUELCHED),
    (SvAmbState.WL_FIXED,        SvAmbState.SQUELCHED),
    (SvAmbState.NL_SHORT_FIXED,  SvAmbState.SQUELCHED),
    (SvAmbState.NL_LONG_FIXED,   SvAmbState.SQUELCHED),

    # Squelch cooldown expires: re-enter at FLOAT (MW must reconverge).
    (SvAmbState.SQUELCHED,       SvAmbState.FLOAT),
})


class InvalidTransition(ValueError):
    """Raised when a requested transition isn't in the state-machine."""


@dataclass
class SvRecord:
    """Per-SV state-machine record.

    Kept minimal — anything that belongs elsewhere (MW state, NL integer,
    filter ambiguity slot) stays there.  This record carries only what
    the state machine itself needs plus enough context for log lines.
    """
    sv: str
    state: SvAmbState = SvAmbState.TRACKING
    # Epoch at which the current state was entered.  Used by the false-fix
    # monitor to scope "recent fix" residual windows.
    state_entered_epoch: int = 0
    # Azimuth at first NL fix.  The validation promoter diffs current az
    # against this to promote NL_SHORT_FIXED → NL_LONG_FIXED.  Populated
    # when transitioning into NL_SHORT_FIXED; None otherwise.
    first_fix_az_deg: Optional[float] = None
    # Most recent observed elevation.  Updated externally (the monitors
    # already have `elevations` per epoch).  Used for log lines and for
    # the setting-SV drop monitor.
    last_elev_deg: Optional[float] = None
    # Short history of (epoch, from_state, to_state, reason) — cap so
    # memory doesn't grow unbounded on long runs.  Useful for offline
    # forensics of cascading transitions.
    history: list[tuple[int, SvAmbState, SvAmbState, str]] = field(default_factory=list)
    # Epoch at which this SV's current SQUELCHED state expires.  None
    # when not squelched.  The tracker's check_squelch_cooldowns()
    # sweeps records and transitions expired ones to FLOAT.
    squelch_expires_epoch: Optional[int] = None
    # Count of unexpected false-fix events this arc (events at
    # elev ≥ reliable_elev_deg, defined in the false-fix monitor).
    # Drives the false-fix monitor's cooldown escalation: 1st → short,
    # 2nd → longer, 3rd+ → effectively rest-of-arc.  Resets when the
    # record is forgotten (tracking loss).
    unexpected_ff_this_arc: int = 0
    # Last epoch at which this SV was observed (MW update or any
    # external observation).  Engine uses this to forget records
    # after an arc-gap threshold, which implicitly resets the
    # unexpected_ff_this_arc counter.
    last_seen_epoch: int = 0

    _HISTORY_CAP: int = 16


class SvStateTracker:
    """Maintains one SvRecord per (system, PRN).

    Threading model: the tracker is *not* thread-safe.  Call from the
    AntPosEst / AR thread only.  The monitors (false-fix monitor,
    setting-SV drop monitor, fix-set integrity alarm) and the AR paths
    (MW, NL, slip) all run in that single thread.

    The tracker is authoritative for state transitions — it refuses
    illegal moves.  Callers pass a free-form `reason` that lands in
    the log line and in the history record.
    """

    def __init__(self):
        self._records: dict[str, SvRecord] = {}

    # ── Lookup / iteration ──────────────────────────────────────── #

    def get(self, sv: str) -> SvRecord:
        """Return the record for this SV, creating a TRACKING record if new."""
        rec = self._records.get(sv)
        if rec is None:
            rec = SvRecord(sv=sv)
            self._records[sv] = rec
        return rec

    def state(self, sv: str) -> SvAmbState:
        """Current state of this SV (creates TRACKING record on first query)."""
        return self.get(sv).state

    def svs_in(self, *states: SvAmbState) -> list[str]:
        """List of SVs currently in any of the given states."""
        wanted = set(states)
        return [sv for sv, rec in self._records.items() if rec.state in wanted]

    def count_in(self, *states: SvAmbState) -> int:
        return sum(1 for rec in self._records.values() if rec.state in set(states))

    def all_records(self):
        """Iterable view of (sv, record) pairs — read-only intent."""
        return self._records.items()

    # ── Fix-set membership helpers ───────────────────────────────── #

    def short_term_members(self) -> list[str]:
        """SVs whose integer fix is a short-term member of the fix set."""
        return self.svs_in(SvAmbState.NL_SHORT_FIXED)

    def long_term_members(self) -> list[str]:
        """SVs whose integer fix is a long-term member of the fix set."""
        return self.svs_in(SvAmbState.NL_LONG_FIXED)

    def fix_set_members(self) -> list[str]:
        """All SVs currently contributing an integer fix to the position solution."""
        return self.svs_in(SvAmbState.NL_SHORT_FIXED, SvAmbState.NL_LONG_FIXED)

    # ── State mutations ─────────────────────────────────────────── #

    def transition(
        self,
        sv: str,
        to: SvAmbState,
        *,
        epoch: int,
        reason: str = "",
        az_deg: Optional[float] = None,
        elev_deg: Optional[float] = None,
        cooldown_epochs: Optional[int] = None,
    ) -> None:
        """Move SV into `to` state, enforcing the legal-edge set.

        Idempotent for self-transitions (caller may re-declare a state
        without triggering a log line or history entry).  Illegal moves
        raise InvalidTransition — the caller decides whether to catch
        or let it crash tests.

        When `to` is SQUELCHED, `cooldown_epochs` sets the per-SV
        duration until `check_squelch_cooldowns` transitions it back
        to FLOAT.  Callers that don't pass this value get the legacy
        60-epoch default.  Logged in the reason frag as `squelch=<N>s`.
        """
        rec = self.get(sv)
        if elev_deg is not None:
            rec.last_elev_deg = float(elev_deg)
        if rec.state == to:
            return
        edge = (rec.state, to)
        if edge not in _LEGAL_EDGES:
            raise InvalidTransition(
                f"{sv}: illegal transition {rec.state.value} → {to.value}"
                f" (epoch={epoch}, reason={reason or '?'})"
            )
        from_state = rec.state
        rec.state = to
        rec.state_entered_epoch = int(epoch)
        if to is SvAmbState.NL_SHORT_FIXED and az_deg is not None:
            rec.first_fix_az_deg = float(az_deg)
        # Per-SV squelch cooldown: set the expiry epoch on entry to
        # SQUELCHED; clear it on any other transition.
        if to is SvAmbState.SQUELCHED:
            dur = int(cooldown_epochs) if cooldown_epochs is not None else 60
            rec.squelch_expires_epoch = int(epoch) + dur
        else:
            rec.squelch_expires_epoch = None
        if len(rec.history) >= rec._HISTORY_CAP:
            rec.history.pop(0)
        rec.history.append((int(epoch), from_state, to, reason))
        elev_frag = (
            f"elev={rec.last_elev_deg:.0f}°" if rec.last_elev_deg is not None else "elev=?"
        )
        dur_frag = ""
        if to is SvAmbState.SQUELCHED and cooldown_epochs is not None:
            dur_frag = f", squelch={int(cooldown_epochs)}s"
        log.info(
            "[SV_STATE] %s: %s → %s (epoch=%d, %s%s, reason=%s)",
            sv, from_state.value, to.value, epoch, elev_frag,
            dur_frag, reason or "?",
        )

    def check_squelch_cooldowns(self, epoch: int) -> list[str]:
        """Sweep SQUELCHED records; transition those whose cooldown
        has expired back to FLOAT.

        Returns the list of SVs that transitioned.  Called each eval
        cycle from the engine main loop.
        """
        recovered: list[str] = []
        for sv, rec in list(self._records.items()):
            if rec.state is not SvAmbState.SQUELCHED:
                continue
            if rec.squelch_expires_epoch is None:
                continue
            if epoch < rec.squelch_expires_epoch:
                continue
            try:
                self.transition(sv, SvAmbState.FLOAT,
                                epoch=epoch, reason="squelch cooldown expired")
                recovered.append(sv)
            except InvalidTransition:
                pass
        return recovered

    def mark_seen(self, sv: str, epoch: int) -> None:
        """Record that the receiver observed this SV at `epoch`.

        Engine calls this from the observation ingest path.  Used by
        `forget_stale(...)` to identify records whose SVs haven't been
        seen in a while (arc-gap → drop the record so the next arc
        starts with a zeroed unexpected-false-fix counter).
        """
        rec = self.get(sv)
        rec.last_seen_epoch = int(epoch)

    def forget_stale(self, epoch: int, stale_after_epochs: int) -> list[str]:
        """Forget records whose `last_seen_epoch` is older than
        `stale_after_epochs` ago.

        Returns the list of forgotten SVs.  Implements arc-boundary
        detection: when an SV hasn't been observed for long enough
        that it's probably set, drop its record so the next arc
        starts clean.
        """
        stale_cutoff = int(epoch) - int(stale_after_epochs)
        dropped = []
        for sv, rec in list(self._records.items()):
            # Records with last_seen_epoch == 0 were never observed
            # (TRACKING records created by monitor lookups).  Leave them.
            if rec.last_seen_epoch <= 0:
                continue
            if rec.last_seen_epoch < stale_cutoff:
                dropped.append(sv)
                self._records.pop(sv, None)
        return dropped

    def update_elev(self, sv: str, elev_deg: float) -> None:
        """Record the most recent elevation for this SV.

        Separate entry point so monitors that see elevations every epoch
        can stream them in without synthesizing fake state changes.
        """
        self.get(sv).last_elev_deg = float(elev_deg)

    def forget(self, sv: str) -> None:
        """Drop this SV's record entirely.

        Used when the receiver loses tracking for long enough that
        re-initialising state on re-acquisition is cleaner than resuming
        from stale history.  Cycle slips should use
        `transition(..., to=FLOAT)` (LOW-conf) or
        `transition(..., to=SQUELCHED)` (HIGH-conf) instead; `forget` is
        for true disappearance.
        """
        self._records.pop(sv, None)

    # ── Diagnostics ─────────────────────────────────────────────── #

    def summary(self) -> str:
        """One-line state-count summary for periodic logging."""
        counts = {st: 0 for st in SvAmbState}
        for rec in self._records.values():
            counts[rec.state] += 1
        parts = [f"{st.value}={counts[st]}" for st in SvAmbState if counts[st]]
        return "sv_state: " + (" ".join(parts) if parts else "empty")

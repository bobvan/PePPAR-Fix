"""Per-SV ambiguity lifecycle state machine.

Implements the per-SV state machine specified in
`docs/sv-lifecycle-and-pfr-split.md`.  Orthogonal to the host-level
`AntPosEstState`; answers "is this specific satellite's integer
ambiguity trustworthy?" rather than "is the host's position
trustworthy?".

The six states form this lifecycle:

    FLOAT ──MW──► WL_FIXED ──LAMBDA──► NL_PROVISIONAL ──Δaz──► NL_VALIDATED
      ▲              ▲                      │                      │
      │              │                      │ Job A reject         │ Job B
      │              │                      │ or elev < AR mask    │ retirement
      │              └──────────────────────┤                      ▼
      │                                     ▼             ┌──────────────┐
      │                                     │             │   RETIRING   │
      │                                     │             └──────┬───────┘
      └──────────── FLOAT ◄──────slip (LOW conf)───────────┘
      │
      │
      BLACKLISTED ◄────── slip (HIGH conf) or repeated Job A rejection
         │
         │ cooldown expires
         ▼
        FLOAT

Transitions that aren't in this diagram are illegal and raise
`InvalidTransition`.  The tracker logs every legal transition as
one `[SV_STATE] <sv>: <from> → <to> (epoch=N, elev=X°, reason=...)`
line — grep-friendly.

Bead 4 (NL_PROVISIONAL → NL_VALIDATED promotion based on azimuth
change) is deferred.  Until it lands, every NL fix stays in
NL_PROVISIONAL indefinitely; the Job A monitor still operates on it
and Job B's retirement logic treats PROVISIONAL and VALIDATED
identically.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

log = logging.getLogger(__name__)


class SvAmbState(Enum):
    """Per-SV ambiguity state — one value per (system, PRN)."""
    FLOAT = "FLOAT"
    WL_FIXED = "WL_FIXED"
    NL_PROVISIONAL = "NL_PROVISIONAL"
    NL_VALIDATED = "NL_VALIDATED"
    RETIRING = "RETIRING"
    BLACKLISTED = "BLACKLISTED"


# Legal directed edges.  Built once at import; `transition()` consults it.
_LEGAL_EDGES: frozenset[tuple[SvAmbState, SvAmbState]] = frozenset({
    (SvAmbState.FLOAT,           SvAmbState.WL_FIXED),
    (SvAmbState.FLOAT,           SvAmbState.BLACKLISTED),
    (SvAmbState.WL_FIXED,        SvAmbState.NL_PROVISIONAL),
    (SvAmbState.WL_FIXED,        SvAmbState.FLOAT),
    (SvAmbState.WL_FIXED,        SvAmbState.BLACKLISTED),
    (SvAmbState.NL_PROVISIONAL,  SvAmbState.NL_VALIDATED),
    (SvAmbState.NL_PROVISIONAL,  SvAmbState.FLOAT),
    (SvAmbState.NL_PROVISIONAL,  SvAmbState.RETIRING),
    (SvAmbState.NL_PROVISIONAL,  SvAmbState.BLACKLISTED),
    (SvAmbState.NL_VALIDATED,    SvAmbState.RETIRING),
    (SvAmbState.NL_VALIDATED,    SvAmbState.FLOAT),
    (SvAmbState.NL_VALIDATED,    SvAmbState.BLACKLISTED),
    (SvAmbState.RETIRING,        SvAmbState.FLOAT),
    (SvAmbState.RETIRING,        SvAmbState.BLACKLISTED),
    (SvAmbState.BLACKLISTED,     SvAmbState.FLOAT),
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
    state: SvAmbState = SvAmbState.FLOAT
    # Epoch at which the current state was entered.  Used by the Job A
    # validator to scope "recent fix" residual windows.
    state_entered_epoch: int = 0
    # Azimuth at first NL fix.  Bead 4 will diff current az against
    # this to promote to NL_VALIDATED.  Populated when transitioning
    # into NL_PROVISIONAL; None otherwise.
    first_fix_az_deg: Optional[float] = None
    # Most recent observed elevation.  Updated externally (the monitors
    # already have `elevations` per epoch).  Used for log lines and for
    # the retirement gate.
    last_elev_deg: Optional[float] = None
    # Short history of (epoch, from_state, to_state, reason) — cap so
    # memory doesn't grow unbounded on long runs.  Useful for offline
    # forensics of cascading transitions.
    history: list[tuple[int, SvAmbState, SvAmbState, str]] = field(default_factory=list)

    _HISTORY_CAP: int = 16


class SvStateTracker:
    """Maintains one SvRecord per (system, PRN).

    Threading model: the tracker is *not* thread-safe.  Call from the
    AntPosEst / AR thread only.  The monitors (provisional_validator,
    retirement_gate, host_rms_alarm) and the AR paths (MW, NL, slip)
    all run in that single thread.

    The tracker is authoritative for state transitions — it refuses
    illegal moves.  Callers pass a free-form `reason` that lands in
    the log line and in the history record.
    """

    def __init__(self):
        self._records: dict[str, SvRecord] = {}

    # ── Lookup / iteration ──────────────────────────────────────── #

    def get(self, sv: str) -> SvRecord:
        """Return the record for this SV, creating a FLOAT record if new."""
        rec = self._records.get(sv)
        if rec is None:
            rec = SvRecord(sv=sv)
            self._records[sv] = rec
        return rec

    def state(self, sv: str) -> SvAmbState:
        """Current state of this SV (creates FLOAT record on first query)."""
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
    ) -> None:
        """Move SV into `to` state, enforcing the legal-edge set.

        Idempotent for self-transitions (caller may re-declare a state
        without triggering a log line or history entry).  Illegal moves
        raise InvalidTransition — the caller decides whether to catch
        or let it crash tests.
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
        if to is SvAmbState.NL_PROVISIONAL and az_deg is not None:
            rec.first_fix_az_deg = float(az_deg)
        if len(rec.history) >= rec._HISTORY_CAP:
            rec.history.pop(0)
        rec.history.append((int(epoch), from_state, to, reason))
        elev_frag = (
            f"elev={rec.last_elev_deg:.0f}°" if rec.last_elev_deg is not None else "elev=?"
        )
        log.info(
            "[SV_STATE] %s: %s → %s (epoch=%d, %s, reason=%s)",
            sv, from_state.value, to.value, epoch, elev_frag, reason or "?",
        )

    def update_elev(self, sv: str, elev_deg: float) -> None:
        """Record the most recent elevation for this SV.

        Separate entry point so monitors that see elevations every epoch
        can stream them in without synthesizing fake state changes.
        """
        self.get(sv).last_elev_deg = float(elev_deg)

    def forget(self, sv: str) -> None:
        """Drop this SV's record entirely.

        Used when an SV leaves the sky for long enough that re-initialising
        its state on re-acquisition is cleaner than resuming from stale
        history.  Cycle slips should use `transition(..., to=FLOAT)`
        instead; `forget` is for true disappearance.
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

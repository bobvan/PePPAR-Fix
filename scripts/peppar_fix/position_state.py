"""Position state files — load/save .ppp.toml and .survey.toml.

Three files cooperate to seed the engine's starting position:

  state/receivers/<uid>.json        — engine-written, holds mount_id
  state/positions/<uid>.ppp.toml    — engine-written PPP filter snapshot
  state/positions/<uid>.survey.toml — peppar-survey-written (optional)

The two position files share the same flat schema; each tags its
``mount_id`` at write time.  At engine startup, readers compare the
embedded mount_id to the receiver's current mount_id (from
state/receivers/<uid>.json) and discard stale-mount entries.

Selection: ``pick_most_confident`` returns the file with the smallest
``sigma_m``, with a 2x tie-break in favor of survey when both are
within that factor — survey's error propagation is more rigorous
than the PPP filter's (which can under-report uncertainty under slow
position-state random walk).

See docs/position-state-and-monitoring.md for the full design.
"""

from __future__ import annotations

import logging
import os
import tomllib
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Optional

log = logging.getLogger(__name__)

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                          "..", ".."))
DEFAULT_POSITIONS_DIR = os.path.join(_REPO_ROOT, "state", "positions")

# Tie-break: prefer survey when its sigma is within this multiplicative
# factor of ppp's sigma.  Survey is "ground truth" with rigorous error
# propagation; ppp can be optimistic under random-walk position state.
SURVEY_TIE_BREAK_RATIO = 2.0


# ── Data type ─────────────────────────────────────────────────────── #


@dataclass(frozen=True)
class PositionState:
    """A snapshot of antenna position from one source.

    Attributes:
        mount_id: integer tag identifying the antenna mount this
            estimate is valid for.  Compared against the receiver's
            current mount_id at read time; mismatched entries are
            discarded as stale.
        ecef_m: (X, Y, Z) in meters, WGS84/ITRF.
        sigma_m: 1-sigma uncertainty on the ECEF position.
        updated: ISO 8601 UTC timestamp of when this estimate was
            written.
        source: human-readable origin label.  Examples:
            "peppar_fix_engine PPP-AR", "PRIDE 6-day mean".
        kind: "ppp" or "survey" — set by the loader, not stored in
            the file.  Used for tie-break logic.
        extra: any additional file-specific keys preserved verbatim
            for diagnostic logging.  Not part of the selection logic.
    """
    mount_id: int
    ecef_m: tuple[float, float, float]
    sigma_m: float
    updated: str
    source: str
    kind: str = "ppp"
    extra: dict = field(default_factory=dict)


# ── Paths ─────────────────────────────────────────────────────────── #


def _ppp_path(uid, positions_dir: Optional[str] = None) -> str:
    d = positions_dir or DEFAULT_POSITIONS_DIR
    return os.path.join(d, f"{uid}.ppp.toml")


def _survey_path(uid, positions_dir: Optional[str] = None) -> str:
    d = positions_dir or DEFAULT_POSITIONS_DIR
    return os.path.join(d, f"{uid}.survey.toml")


# ── Load ──────────────────────────────────────────────────────────── #


def _load_toml(path: str, kind: str) -> Optional[PositionState]:
    if not os.path.exists(path):
        return None
    try:
        with open(path, "rb") as f:
            data = tomllib.load(f)
    except (tomllib.TOMLDecodeError, OSError) as e:
        log.warning("Failed to load %s: %s", path, e)
        return None
    try:
        ecef = data["ecef_m"]
        if len(ecef) != 3:
            raise ValueError(f"ecef_m must be 3-tuple, got {len(ecef)}")
        # Extract known fields; preserve the rest in `extra`.
        known = {"mount_id", "ecef_m", "sigma_m", "updated", "source"}
        extra = {k: v for k, v in data.items() if k not in known}
        return PositionState(
            mount_id=int(data["mount_id"]),
            ecef_m=(float(ecef[0]), float(ecef[1]), float(ecef[2])),
            sigma_m=float(data["sigma_m"]),
            updated=str(data["updated"]),
            source=str(data["source"]),
            kind=kind,
            extra=extra,
        )
    except (KeyError, ValueError, TypeError) as e:
        log.warning("Malformed position state %s: %s", path, e)
        return None


def load_ppp_state(uid, positions_dir: Optional[str] = None
                   ) -> Optional[PositionState]:
    """Load .ppp.toml for a receiver UID.  Returns None if absent or
    malformed (with a warning logged in the malformed case)."""
    return _load_toml(_ppp_path(uid, positions_dir), "ppp")


def load_survey_state(uid, positions_dir: Optional[str] = None
                      ) -> Optional[PositionState]:
    """Load .survey.toml for a receiver UID.  Returns None if absent
    or malformed.  Absence is a first-class case — installations
    without peppar-survey simply never have a survey file."""
    return _load_toml(_survey_path(uid, positions_dir), "survey")


# ── Save (ppp only — survey is written by peppar-survey) ──────────── #


def _format_toml(state: PositionState) -> str:
    """Hand-render PositionState as TOML.  Flat schema, no nesting.

    We hand-format rather than depending on tomli_w because:
      - schema is fixed and trivial (5 required fields plus a flat
        optional extras dict)
      - no new dependency
      - operator-readable output we control exactly
    """
    lines = [
        f"mount_id = {int(state.mount_id)}",
        f"ecef_m = [{state.ecef_m[0]:.4f}, {state.ecef_m[1]:.4f}, "
        f"{state.ecef_m[2]:.4f}]",
        f"sigma_m = {state.sigma_m:.6f}",
        f"updated = \"{state.updated}\"",
        f"source = \"{state.source}\"",
    ]
    for k, v in sorted(state.extra.items()):
        if isinstance(v, str):
            lines.append(f"{k} = \"{v}\"")
        elif isinstance(v, bool):
            lines.append(f"{k} = {'true' if v else 'false'}")
        elif isinstance(v, int):
            lines.append(f"{k} = {v}")
        elif isinstance(v, float):
            lines.append(f"{k} = {v}")
        else:
            # Unsupported type — drop with a warning.  Won't reach
            # production in practice since extras come from us.
            log.warning("Dropping unsupported extra '%s' of type %s",
                        k, type(v).__name__)
    return "\n".join(lines) + "\n"


def save_ppp_state(state: PositionState, uid,
                   positions_dir: Optional[str] = None) -> None:
    """Atomically write .ppp.toml for a receiver UID.

    Writes via tmp+rename so a partial write never leaves a torn
    file visible to readers.  Creates the positions_dir if missing.
    """
    if state.kind != "ppp":
        # Defensive — saving a 'survey' state under .ppp.toml would
        # mis-tag it on the next reload.
        raise ValueError(f"save_ppp_state expects kind='ppp', "
                         f"got '{state.kind}'")
    d = positions_dir or DEFAULT_POSITIONS_DIR
    os.makedirs(d, exist_ok=True)
    path = _ppp_path(uid, positions_dir)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(_format_toml(state))
    os.replace(tmp, path)
    log.info("Saved PPP position state to %s", path)


def utc_now_iso() -> str:
    """Return ISO 8601 UTC timestamp string in the project convention."""
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# ── Selection ─────────────────────────────────────────────────────── #


def filter_current_mount(states: list[Optional[PositionState]],
                         current_mount_id: int) -> list[PositionState]:
    """Drop None and any state whose mount_id != current_mount_id.

    Stale-mount entries arise after antenna moves: the operator (or
    auto-move logic) bumps the receiver's mount_id and the existing
    position files become stale.  Filtering keyed on mount_id is the
    atomic discard mechanism.
    """
    out: list[PositionState] = []
    for s in states:
        if s is None:
            continue
        if s.mount_id != current_mount_id:
            log.info("Discarding stale-mount %s state "
                     "(file mount_id=%d, current=%d)",
                     s.kind, s.mount_id, current_mount_id)
            continue
        out.append(s)
    return out


def load_current_mount_id(uid,
                          receivers_dir: Optional[str] = None) -> int:
    """Read the receiver's current ``mount_id`` from
    ``state/receivers/<uid>.json``.  Returns 0 when the file is absent
    or the field is missing — this is the "first run after this
    feature lands" default and matches the fresh-install case.

    The mount_id lives in receivers/, not positions/, because it's a
    property of "which antenna is this receiver currently connected
    to" — receiver-side identity that the engine writes.
    """
    if uid is None:
        return 0
    try:
        from peppar_fix.receiver_state import load_receiver_state
    except ImportError:  # pragma: no cover — should never happen in production
        log.warning("Cannot import receiver_state; defaulting mount_id=0")
        return 0
    rstate = load_receiver_state(uid, state_dir=receivers_dir)
    if not rstate:
        return 0
    try:
        return int(rstate.get("mount_id", 0))
    except (TypeError, ValueError):
        log.warning("mount_id in receiver state is not an int; "
                    "defaulting to 0")
        return 0


def seed_from_state_files(
    uid,
    *,
    ignore_ppp: bool = False,
    ignore_survey: bool = False,
    positions_dir: Optional[str] = None,
    receivers_dir: Optional[str] = None,
) -> Optional[PositionState]:
    """One-shot helper for engine startup.  Reads .ppp.toml and
    .survey.toml, filters by current mount_id, returns the
    most-confident state (or None when no usable state exists).

    Caller behavior on None: fall back to NAV2 / bootstrap.

    Args:
        uid: receiver unique_id (int or str).  None disables the
            whole helper (returns None).
        ignore_ppp: skip the .ppp.toml file (CLI --ignore-ppp).
        ignore_survey: skip the .survey.toml file (CLI --ignore-survey).
        positions_dir: override DEFAULT_POSITIONS_DIR (for testing).
        receivers_dir: passed to load_current_mount_id for mount_id
            lookup (for testing).

    Returns:
        Most-confident PositionState, or None.
    """
    if uid is None:
        return None
    current_mount_id = load_current_mount_id(uid, receivers_dir=receivers_dir)
    candidates: list[Optional[PositionState]] = []
    if not ignore_ppp:
        candidates.append(load_ppp_state(uid, positions_dir=positions_dir))
    if not ignore_survey:
        candidates.append(load_survey_state(uid, positions_dir=positions_dir))
    usable = filter_current_mount(candidates, current_mount_id)
    return pick_most_confident(usable)


def pick_most_confident(states: list[PositionState]
                        ) -> Optional[PositionState]:
    """Choose the most-confident PositionState from the candidates.

    Selection rule:
      - smallest sigma_m wins
      - tie-break (within SURVEY_TIE_BREAK_RATIO = 2x) to survey
        because survey error propagation is more rigorous than the
        PPP filter's, which can under-report slow-drift uncertainty

    Returns None if the list is empty.  Caller falls back to NAV2.
    """
    if not states:
        return None
    if len(states) == 1:
        return states[0]
    # Find smallest sigma among survey-kind and ppp-kind separately
    survey = min((s for s in states if s.kind == "survey"),
                 key=lambda s: s.sigma_m, default=None)
    ppp = min((s for s in states if s.kind == "ppp"),
              key=lambda s: s.sigma_m, default=None)
    if survey is None:
        return ppp
    if ppp is None:
        return survey
    # Both present — tie-break.
    if survey.sigma_m <= ppp.sigma_m * SURVEY_TIE_BREAK_RATIO:
        return survey
    return ppp

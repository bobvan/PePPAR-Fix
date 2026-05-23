"""Watch ``state/positions/<uid>.survey.toml`` for refresh events.

Background: peppar-survey runs offline (daily PRIDE pipeline, ad-hoc
OPUS, RTKLIB-CORS) and writes a fresh ``.survey.toml`` whenever its
running mean has materially advanced.  The engine reads that file
**at startup only** — once running, it doesn't notice when
peppar-survey lands a refresh.  This module closes that gap.

Design model (peppar-survey-owned position):

  peppar-survey  ──── writes state/positions/<uid>.survey.toml
                                              │
                                  poll (60 s) │ mtime change
                                              ▼
                                    decide_refresh_action(prev, curr,
                                                          current_arp_ecef)
                                              │
                ┌─────────────────────────────┼───────────────────────────┐
                │                             │                           │
        NO_CHANGE                          SLEW                         STEP
        (no mtime advance,              Δ < slew_threshold       Δ ≥ slew_threshold
         or first read with             on_slew(refresh, Δ)            OR mount_sn
         seed agreement)                  │                              ┌──┘
                                          ▼                              ▼
                              bayesian_arp_blend            engine clean shutdown
                              (in engine main loop)         (orchestrator restarts;
                              filt.pos += α_eff × Δ          engine picks up the
                                                              new seed at startup)

Why polling not inotify: inotify has edge-cases on FUSE / NFS /
network-attached storage that some lab hosts use.  60 s is fine —
peppar-survey writes at most once per day under the standard pipeline.

mtime short-circuit: the watcher re-reads + decides only when the
state file's mtime has advanced since the last successful read.  Saves
I/O on the dominant "file stable" case.

Read-time atomicity: peppar-survey writes via temp-file + os.rename
(see ``peppar_fix.position_state.save_survey_state``), so any read
either sees the prior full content or the new full content.  No
partial-read protection needed here.

This module is the DETECTOR.  Engine wiring (spawn the watcher thread,
hook on_slew → ``bayesian_arp_blend`` at the existing call site,
hook on_step → clean-shutdown signal) lives in
``scripts/peppar_fix_engine.py`` in a stacked PR for separability —
the detector is reusable for a future standalone-orchestrator path.
"""
from __future__ import annotations

import enum
import logging
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import numpy as np

log = logging.getLogger(__name__)


# Defaults sized for the peppar-survey ARP class.  σ ≈ 12 mm
# (OPUS-Static), ≤ 50 mm (RTKLIB-CORS) — so a 20 cm refresh is
# plausibly-real-but-worth-blending; anything bigger is almost
# certainly antenna-moved and warrants a clean restart.
DEFAULT_SLEW_THRESHOLD_M = 0.20
DEFAULT_POLL_INTERVAL_S = 60.0


# ── Snapshot ──────────────────────────────────────────────────────── #


@dataclass(frozen=True)
class SurveyRefresh:
    """One read of ``state/positions/<uid>.survey.toml``.

    Captures the fields the watcher needs for action-decision, plus
    the on-disk mtime so subsequent polls can short-circuit when the
    file hasn't changed.  All other PositionState fields (source,
    extra, kind) are dropped — the watcher's decision doesn't depend
    on them.
    """
    new_ecef: tuple[float, float, float]
    new_sigma_m: float
    new_mount_sn: int
    file_mtime: float

    @classmethod
    def from_state_and_path(cls, state, path: str) -> "SurveyRefresh":
        """Build from a ``PositionState`` + the file path it was loaded from.

        Looks up the mtime separately so the snapshot carries it without
        re-opening the file.  Caller is responsible for handling the
        race where the file is renamed between state-load and stat —
        in practice the atomic temp+rename pattern peppar-survey uses
        means stat will see either the new file or ENOENT.
        """
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            mtime = 0.0
        return cls(
            new_ecef=tuple(state.ecef_m),
            new_sigma_m=float(state.sigma_m),
            new_mount_sn=int(state.mount_sn),
            file_mtime=mtime,
        )


# ── Decision ──────────────────────────────────────────────────────── #


class RefreshAction(enum.Enum):
    """What the orchestrator should do in response to a survey refresh."""
    NONE = "none"     # no action — first read, or mtime unchanged
    SLEW = "slew"     # Δ < slew_threshold; blend via bayesian_arp_blend
    STEP = "step"     # Δ ≥ slew_threshold OR mount_sn bumped; clean shutdown


def decide_refresh_action(
    prev: Optional[SurveyRefresh],
    curr: SurveyRefresh,
    current_arp_ecef: np.ndarray,
    *,
    slew_threshold_m: float = DEFAULT_SLEW_THRESHOLD_M,
) -> tuple[RefreshAction, float]:
    """Decide the action for one survey-refresh event.

    Args:
        prev: previous successful snapshot, or None for first read.
        curr: just-read snapshot.
        current_arp_ecef: the engine's in-memory ARP (the seed it's
            currently running against — may already be ahead of the
            on-disk file via in-process blending from other sources
            like NAV2).
        slew_threshold_m: small/large boundary.  Default 0.20 m —
            sized for the peppar-survey σ-class.

    Returns:
        (action, delta_3d_m):
          - NONE if first read (prev is None) OR mtime unchanged.
          - STEP if mount_sn bumped (operator action, always restart
            regardless of delta).
          - SLEW if Δ_3d < slew_threshold_m.
          - STEP if Δ_3d ≥ slew_threshold_m.

        delta_3d_m is the |new_ecef − current_arp_ecef| in meters; 0.0
        when the action is NONE.

    Pure function — no I/O, no side effects.  All thresholding lives
    here so the watch_loop driver stays trivial.
    """
    if prev is None:
        # First read since watcher startup.  The engine's startup-read
        # of the same file means current_arp_ecef should already be
        # close to curr.new_ecef.  Either way, the engine is running
        # against this state; no action until the file changes again.
        return RefreshAction.NONE, 0.0

    if curr.file_mtime <= prev.file_mtime:
        # No file change since the previous successful read.
        return RefreshAction.NONE, 0.0

    if curr.new_mount_sn != prev.new_mount_sn:
        # mount_sn bump = operator action (antenna physically moved
        # or replaced).  Always STEP — the engine's current pin is
        # known-wrong by the move amount and may not even be on the
        # same mount any more.  Δ may be tiny if the new survey is
        # still converging, but mount_sn semantics take precedence.
        delta = _delta_3d(curr.new_ecef, current_arp_ecef)
        return RefreshAction.STEP, delta

    delta = _delta_3d(curr.new_ecef, current_arp_ecef)
    if delta < slew_threshold_m:
        return RefreshAction.SLEW, delta
    return RefreshAction.STEP, delta


def _delta_3d(ecef: tuple[float, float, float],
              arp: np.ndarray) -> float:
    """3D distance between an ecef tuple and a numpy array."""
    e = np.asarray(ecef, dtype=float)
    return float(np.linalg.norm(e - arp))


# ── Load helpers ──────────────────────────────────────────────────── #


def load_current_snapshot(survey_path: str) -> Optional[SurveyRefresh]:
    """Read survey_path and return a SurveyRefresh, or None on
    missing / malformed.

    Wraps ``peppar_fix.position_state._load_toml`` (the canonical
    loader) so any schema-evolution stays in one place.  Returns
    None for any non-success — missing file, malformed TOML,
    required-fields-missing.  Callers treat None as "nothing
    actionable this poll."
    """
    # Lazy import: keep this module's import-time footprint small so
    # tests can mock the loader without dragging the whole position_state
    # transitive surface in.
    from peppar_fix.position_state import _load_toml
    state = _load_toml(survey_path, "survey")
    if state is None:
        return None
    return SurveyRefresh.from_state_and_path(state, survey_path)


# ── Polling driver ────────────────────────────────────────────────── #


def watch_loop(
    survey_path: str,
    current_arp_ecef_fn: Callable[[], np.ndarray],
    on_slew: Callable[[SurveyRefresh, float], None],
    on_step: Callable[[SurveyRefresh, float], None],
    *,
    slew_threshold_m: float = DEFAULT_SLEW_THRESHOLD_M,
    poll_interval_s: float = DEFAULT_POLL_INTERVAL_S,
    sleep_fn: Callable[[float], None] = time.sleep,
    stop_fn: Callable[[], bool] = lambda: False,
    max_iterations: Optional[int] = None,
    load_fn: Callable[[str], Optional[SurveyRefresh]] = load_current_snapshot,
) -> None:
    """Run the survey-watcher polling loop.

    Args:
        survey_path: filesystem path to the receiver's .survey.toml.
        current_arp_ecef_fn: callable returning the engine's current
            in-memory ARP as a numpy 3-vector.  Called on each
            decision so the watcher sees the live value.
        on_slew: callback for the SLEW action.  Receives the refresh
            snapshot and the |Δ| in meters.  Caller is responsible
            for invoking ``bayesian_arp_blend`` and updating the
            engine's state.
        on_step: callback for the STEP action.  Receives the refresh
            snapshot and |Δ|.  Caller is responsible for the
            clean-shutdown signal.
        slew_threshold_m: small/large boundary, default 0.20 m.
        poll_interval_s: seconds between mtime polls, default 60 s.
        sleep_fn: injectable sleep; default ``time.sleep``.  Tests
            substitute a no-op to drive the loop deterministically.
        stop_fn: callable returning True when the loop should exit.
            Default never stops.  Tests substitute a counter-driven
            stop; production wraps an Event.is_set() probe.
        max_iterations: bound on loop iterations.  None for unlimited
            (production); test harnesses set a small integer to
            guarantee termination even if stop_fn isn't fed correctly.
        load_fn: injectable file-reader.  Default reads the real file
            via ``load_current_snapshot``.  Tests substitute a
            queue-of-snapshots driver.

    The loop is fully driven by callbacks — no return value, no
    raised exceptions for normal operation.  Loader failures (None
    return) are silently skipped on each iteration; persistent
    failure is the operator's signal that the file is gone or
    malformed.
    """
    prev: Optional[SurveyRefresh] = None
    iters = 0
    while not stop_fn():
        if max_iterations is not None and iters >= max_iterations:
            return
        iters += 1
        curr = load_fn(survey_path)
        if curr is not None:
            action, delta_3d_m = decide_refresh_action(
                prev, curr, current_arp_ecef_fn(),
                slew_threshold_m=slew_threshold_m,
            )
            if action is RefreshAction.SLEW:
                log.info(
                    "[SURVEY_REFRESHED] mtime=%s mount_sn=%d Δ=%.3fm "
                    "σ=%.3fm — SLEW (Δ < %.3fm threshold)",
                    _fmt_mtime(curr.file_mtime), curr.new_mount_sn,
                    delta_3d_m, curr.new_sigma_m, slew_threshold_m,
                )
                on_slew(curr, delta_3d_m)
            elif action is RefreshAction.STEP:
                trigger = ("mount_sn bump" if prev and curr.new_mount_sn
                           != prev.new_mount_sn else
                           f"Δ ≥ {slew_threshold_m:.3f}m threshold")
                log.warning(
                    "[SURVEY_REFRESHED] mtime=%s mount_sn=%d Δ=%.3fm "
                    "σ=%.3fm — STEP (%s)",
                    _fmt_mtime(curr.file_mtime), curr.new_mount_sn,
                    delta_3d_m, curr.new_sigma_m, trigger,
                )
                on_step(curr, delta_3d_m)
            prev = curr
        sleep_fn(poll_interval_s)


def _fmt_mtime(t: float) -> str:
    """Render a float mtime as 'YYYY-MM-DD HH:MM:SS UTC' for logs."""
    if t <= 0:
        return "?"
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(t))

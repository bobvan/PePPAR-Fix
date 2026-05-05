"""Watch ``state/arp/<receiver_uid>.json`` and decide when to restart.

Per ``docs/runtime-arp-transition.md`` (I-145846): the engine adopts
a freshly-tightened ARP via clean shutdown + restart (option A) when
one of three triggers fires:

  1. σ_arp first-cross of ``arp_pin_threshold_m`` (default 0.10 m)
     — the case that flips the engine from re-survey into pinned mode.
  2. Mean-ECEF drift > ``drift_epsilon_m`` (default 0.05 m) — daily
     post-processing has converged the running mean to a noticeably
     different point than what the engine launched against.
  3. ``mount_id`` increment — operator action; antenna moved; the
     pinned ECEF is now wrong by the move distance and the engine
     must restart even before σ_arp re-converges on the new mount.

This module is **prototype scaffolding** for the orchestrator-side
trigger.  It does not itself restart the engine — the orchestrator
that owns engine lifecycle (or systemd) does that.  The watcher
detects the transition and emits a structured event the
orchestrator can act on.

Design choices:

- Polling, not inotify: state file mtime polling avoids inotify
  edge-cases on FUSE / NFS / network-attached storage that some
  lab hosts use.  Default poll interval is 60 s — the file is
  rewritten at most once per day (per the daily PRIDE pipeline,
  I-013307), so 60 s is plenty of resolution.
- mtime-only short-circuit: re-read + decide only when mtime has
  advanced since the last successful read.  Saves I/O when the
  file is stable (which is most of the time).
- Read-time atomicity: the orchestrator writes the state file
  via temp-file + rename, so any read either sees the prior
  full content or the new full content.  No partial-read
  protection needed in this watcher.
- No state in this module: the watcher passes (prev, curr) to a
  pure decide_transition() function and dispatches the verdict
  via a caller-supplied callback.  Test harnesses can call
  decide_transition() directly without spinning the loop.

This is the engine-orchestrator side of the I-145846 design.  The
engine-side polish (log σ_arp + mtime at startup; emit final
[FIXEDPOS_RESID_SUMMARY] on clean shutdown) is mentioned in
docs/runtime-arp-transition.md and lives separately.
"""
from __future__ import annotations

import enum
import json
import logging
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

log = logging.getLogger(__name__)

DEFAULT_PIN_THRESHOLD_M = 0.10
DEFAULT_DRIFT_EPSILON_M = 0.050
DEFAULT_POLL_INTERVAL_S = 60.0


# ── Snapshot ──────────────────────────────────────────────────────── #


@dataclass(frozen=True)
class ArpSnapshot:
    """A point-in-time read of state/arp/<uid>.json.

    All fields are pulled from the schema in docs/arp-lifecycle-state-schema.md.
    Fields the watcher doesn't gate on (history, watchdog block, etc.)
    are not surfaced — the orchestrator's other components handle them.
    """
    ecef_m: tuple[float, float, float]
    sigma_m: float
    mount_id: int
    mtime: float                    # filesystem mtime at read time
    schema_version: int = 1
    receiver_uid: Optional[str] = None
    antenna_id: Optional[str] = None

    def ecef_distance_m(self, other: "ArpSnapshot") -> float:
        """3D Euclidean distance between this snapshot's ECEF and another's."""
        dx = self.ecef_m[0] - other.ecef_m[0]
        dy = self.ecef_m[1] - other.ecef_m[1]
        dz = self.ecef_m[2] - other.ecef_m[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)


def load_snapshot(path: str | Path) -> Optional[ArpSnapshot]:
    """Parse state/arp/<uid>.json into an ArpSnapshot.

    Returns None when the file doesn't exist (cold mount: the
    orchestrator hasn't created it yet, engine launches in pure
    re-survey mode without a `--surveyed-position-from` argument).
    Returns None on any parse / value error so the watcher can
    keep polling rather than crash.
    """
    p = Path(path)
    try:
        st = p.stat()
    except FileNotFoundError:
        return None
    try:
        with open(p) as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        log.warning("arp_state_watcher: read failed for %s (%s)", p, e)
        return None
    try:
        ecef = data["ecef_m"]
        if not (isinstance(ecef, list) and len(ecef) == 3):
            raise ValueError("ecef_m must be a 3-list")
        sigma_m = float(data["sigma_m"])
        mount_id = int(data.get("mount_id", 0))
    except (KeyError, ValueError, TypeError) as e:
        log.warning("arp_state_watcher: schema mismatch in %s (%s)", p, e)
        return None
    return ArpSnapshot(
        ecef_m=(float(ecef[0]), float(ecef[1]), float(ecef[2])),
        sigma_m=sigma_m,
        mount_id=mount_id,
        mtime=st.st_mtime,
        schema_version=int(data.get("schema_version", 1)),
        receiver_uid=str(data.get("receiver_uid")) if "receiver_uid" in data else None,
        antenna_id=data.get("antenna_id"),
    )


# ── Decision ──────────────────────────────────────────────────────── #


class Trigger(enum.Enum):
    """The three restart triggers from docs/runtime-arp-transition.md."""
    NO_CHANGE = "no_change"
    THRESHOLD_CROSS = "threshold_cross"      # σ_arp first-cross of pin_threshold
    DRIFT = "drift"                          # mean ECEF moved > epsilon
    MOUNT_MOVE = "mount_move"                # mount_id increment


@dataclass(frozen=True)
class TransitionDecision:
    """A decision tuple — what fired and why.

    `engine_should_restart` is the orchestrator-actionable summary;
    `trigger` and `reason` carry the diagnostic detail.
    """
    trigger: Trigger
    reason: str
    engine_should_restart: bool


def decide_transition(prev: Optional[ArpSnapshot],
                      curr: Optional[ArpSnapshot],
                      pin_threshold_m: float = DEFAULT_PIN_THRESHOLD_M,
                      drift_epsilon_m: float = DEFAULT_DRIFT_EPSILON_M,
                      ) -> TransitionDecision:
    """Pure function: given two snapshots, decide whether to restart.

    First-launch cases (`prev` is None or the watcher just started):
    no decision — the engine already read the file at launch, so the
    *current* file is what it's running against.  Only subsequent
    changes drive restart decisions.

    Trigger priority when more than one fires (e.g. mount move +
    threshold cross at once):
      MOUNT_MOVE > DRIFT > THRESHOLD_CROSS

    Mount move always wins because the prior pinned ECEF is geometrically
    wrong; drift wins over threshold cross because the new ECEF is the
    bigger correctness signal.
    """
    if prev is None or curr is None:
        return TransitionDecision(
            Trigger.NO_CHANGE,
            "no prior snapshot (initial read or transient unavailability)",
            engine_should_restart=False,
        )
    if prev.mtime == curr.mtime:
        return TransitionDecision(
            Trigger.NO_CHANGE,
            "mtime unchanged",
            engine_should_restart=False,
        )

    # 1. Mount move — orchestrator incremented mount_id; engine MUST restart.
    if curr.mount_id != prev.mount_id:
        return TransitionDecision(
            Trigger.MOUNT_MOVE,
            f"mount_id {prev.mount_id} → {curr.mount_id}",
            engine_should_restart=True,
        )

    # 2. Drift — running mean moved by more than epsilon.
    drift = curr.ecef_distance_m(prev)
    if drift > drift_epsilon_m:
        return TransitionDecision(
            Trigger.DRIFT,
            f"|Δecef| = {drift*1000:.1f} mm > {drift_epsilon_m*1000:.0f} mm",
            engine_should_restart=True,
        )

    # 3. σ_arp first-cross of pin_threshold (was ≥ threshold, now <).
    crossed = prev.sigma_m >= pin_threshold_m and curr.sigma_m < pin_threshold_m
    if crossed:
        return TransitionDecision(
            Trigger.THRESHOLD_CROSS,
            f"σ_arp {prev.sigma_m:.4f}m → {curr.sigma_m:.4f}m "
            f"(crossed {pin_threshold_m:.3f}m pin_threshold)",
            engine_should_restart=True,
        )

    return TransitionDecision(
        Trigger.NO_CHANGE,
        f"file changed but no trigger: σ_arp {prev.sigma_m:.4f}→{curr.sigma_m:.4f}m, "
        f"|Δecef|={drift*1000:.1f}mm, mount_id stable",
        engine_should_restart=False,
    )


# ── Watch loop ─────────────────────────────────────────────────────── #


def watch_loop(state_path: str | Path,
               on_transition: Callable[[TransitionDecision, ArpSnapshot, ArpSnapshot], None],
               pin_threshold_m: float = DEFAULT_PIN_THRESHOLD_M,
               drift_epsilon_m: float = DEFAULT_DRIFT_EPSILON_M,
               poll_interval_s: float = DEFAULT_POLL_INTERVAL_S,
               max_iterations: Optional[int] = None,
               sleep_fn: Callable[[float], None] = time.sleep,
               ) -> None:
    """Poll state_path; call on_transition when a trigger fires.

    Args:
        state_path: path to state/arp/<uid>.json.
        on_transition: callback fired when decide_transition returns a
            decision with engine_should_restart=True.  Signature:
            (decision, prev_snapshot, curr_snapshot).
        pin_threshold_m: σ_arp threshold for THRESHOLD_CROSS trigger.
        drift_epsilon_m: ECEF distance for DRIFT trigger.
        poll_interval_s: seconds between filesystem mtime checks.
        max_iterations: stop after this many polls (testing only;
            None = forever).
        sleep_fn: pluggable sleep, useful for time-mocked tests.
    """
    prev = load_snapshot(state_path)
    iters = 0
    while max_iterations is None or iters < max_iterations:
        iters += 1
        sleep_fn(poll_interval_s)
        try:
            curr_mtime = os.stat(state_path).st_mtime
        except FileNotFoundError:
            continue
        if prev is not None and curr_mtime == prev.mtime:
            continue  # mtime short-circuit — no re-read needed
        curr = load_snapshot(state_path)
        if curr is None:
            continue
        decision = decide_transition(prev, curr, pin_threshold_m,
                                     drift_epsilon_m)
        if decision.engine_should_restart:
            log.info("[ARP_TRANSITION] %s — %s",
                     decision.trigger.value, decision.reason)
            on_transition(decision, prev, curr)
        # Always advance prev — including no-restart transitions —
        # so the *next* mtime change compares against the latest read.
        prev = curr

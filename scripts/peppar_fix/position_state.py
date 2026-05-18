"""Position state files — load/save .ppp.toml and .survey.toml.

Three files cooperate to seed the engine's starting position:

  state/receivers/<uid>.json        — engine-written, holds mount_sn
  state/positions/<uid>.ppp.toml    — engine-written PPP filter snapshot
  state/positions/<uid>.survey.toml — peppar-survey-written (optional)

The two position files share the same flat schema; each tags its
``mount_sn`` at write time.  At engine startup, readers compare the
embedded mount_sn to the receiver's current mount_sn (from
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
        mount_sn: integer tag identifying the antenna mount this
            estimate is valid for.  Compared against the receiver's
            current mount_sn at read time; mismatched entries are
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
    mount_sn: int
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
        known = {"mount_sn", "ecef_m", "sigma_m", "updated", "source"}
        extra = {k: v for k, v in data.items() if k not in known}
        return PositionState(
            mount_sn=int(data["mount_sn"]),
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
        f"mount_sn = {int(state.mount_sn)}",
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


# ── AntPosEst watchdog ────────────────────────────────────────────── #


# σ above which the per-epoch AntPosEst estimate is too noisy to
# feed the running mean.  Cold-start AntPosEst σ is meters; only
# once it drops below this do we start accumulating.
DEFAULT_ANTPOS_ACTIVATION_SIGMA_M = 0.05

# Running-mean window — σ on the mean tightens as √N for independent
# epochs.  60 at 1 Hz observation cadence (AntPosEst's own clock,
# typically /10 decimation = 6 sec wall time per epoch) is enough
# for the mean to be useful well before per-epoch σ would.
DEFAULT_ANTPOS_WINDOW_EPOCHS = 60


class AntPosEstWatchdog:
    """Running-mean position from AntPosEst output for use as a
    sub-cm watchdog against a pinned ARP.

    Updated every AntPosEst epoch by the thread.  State exposed as
    a dict snapshot via the ``state`` attribute — atomic replace
    makes cross-thread reads safe.

    Per docs/position-state-and-monitoring.md: instrumentation only
    in slice 6 (logs the displacement).  Slice 7 turns these
    numbers into slew (< 1 m) / step (≥ 1 m + mount_sn bump) action.

    Two arming gates layered:

      1. Per-epoch σ-gate: ``sigma_3d_m`` must be below
         ``activation_sigma_m`` to feed the running mean.  Filters
         out cold-start AntPosEst noise (meters-class σ) so a
         single noisy epoch doesn't poison the average.
      2. Running-mean N-gate: ``n_in_mean`` < some minimum means
         the mean isn't yet trustworthy.  This is a soft signal —
         we still emit the (small-N) mean for diagnostics; slice
         7 will compare ``displ_h_m`` against
         ``slew_step_threshold_m`` only once N is at the full
         window.
    """

    def __init__(self,
                 *,
                 activation_sigma_m: float = DEFAULT_ANTPOS_ACTIVATION_SIGMA_M,
                 window_epochs: int = DEFAULT_ANTPOS_WINDOW_EPOCHS):
        from collections import deque
        self.activation_sigma_m = float(activation_sigma_m)
        self.window_epochs = int(window_epochs)
        self._buf = deque(maxlen=self.window_epochs)
        self._pin: Optional[tuple] = None
        self.state = self._empty_state()
        # Diagnostic counters.
        self.n_updates = 0
        self.n_skipped_sigma = 0

    @staticmethod
    def _empty_state() -> dict:
        return {
            "active": False,
            "n_in_mean": 0,
            "sigma_3d_m": None,
            "sigma_mean_m": None,
            "displ_3d_m": None,
            "displ_h_m": None,
            "displ_v_m": None,
            "mean_ecef_m": None,  # running-mean (x, y, z) — slew target
        }

    def set_pin(self, pin_ecef) -> None:
        """Tell the watchdog where the pin is.  Called at startup
        after the engine settled on a pinned position.  Calling
        again replaces the pin — used by slice 7's slew action to
        update the pin in-place after a small ARP refinement."""
        self._pin = tuple(float(v) for v in pin_ecef)

    def update(self, pos_ecef, sigma_3d_m) -> None:
        """Feed one AntPosEst epoch.  Drops the per-epoch reading
        when σ > activation threshold (don't poison the running
        mean with high-σ junk).  Result published to ``self.state``
        as an atomic dict replace for cross-thread read."""
        import math
        if self._pin is None:
            return  # no pin yet — caller must set_pin first
        if sigma_3d_m is None or sigma_3d_m > self.activation_sigma_m:
            # Per-epoch σ too noisy — don't feed.  Still update the
            # `active` field so consumers see the disarmed state.
            old = self.state
            new = dict(old)
            new["active"] = False
            new["sigma_3d_m"] = (float(sigma_3d_m)
                                 if sigma_3d_m is not None else None)
            self.state = new
            self.n_skipped_sigma += 1
            return
        self._buf.append(tuple(float(v) for v in pos_ecef))
        n = len(self._buf)
        mx = sum(p[0] for p in self._buf) / n
        my = sum(p[1] for p in self._buf) / n
        mz = sum(p[2] for p in self._buf) / n
        d3, dh, dv = compute_horizontal_displacement(
            self._pin, (mx, my, mz))
        # σ on the mean tightens as √N if epochs are independent.
        # This is an estimate, not a rigorous covariance — good
        # enough for the slice 7 threshold check and for operator
        # visibility now.
        sigma_mean = float(sigma_3d_m) / math.sqrt(n) if n > 0 else None
        self.state = {
            "active": True,
            "n_in_mean": n,
            "sigma_3d_m": float(sigma_3d_m),
            "sigma_mean_m": sigma_mean,
            "displ_3d_m": d3,
            "displ_h_m": dh,
            "displ_v_m": dv,
            "mean_ecef_m": (mx, my, mz),
        }
        self.n_updates += 1


# ── Watchdog action decision (slice 7) ────────────────────────────── #


# Below this magnitude → slew (gentle, in-place pin update).
# Above → step (probable antenna move; mount_sn bump + restart).
# 1 m ≈ 3 ns equivalent clock-equivalent error via c ≈ 30 cm/ns.
DEFAULT_SLEW_STEP_THRESHOLD_M = 1.0

# Sustain windows.  At [CONFIDENCE] cadence of 60 epochs = ~60 s,
# nav2/antpos_sustain_s = 60 means "at least one full cycle of
# consistent bark".  Step requires much longer to avoid declaring
# a mount move on a transient that just happens to cross 1 m.
DEFAULT_NAV2_THRESHOLD_M = 0.5
DEFAULT_NAV2_SUSTAIN_S = 60.0
DEFAULT_ANTPOS_THRESHOLD_M = 0.03
DEFAULT_ANTPOS_SUSTAIN_S = 60.0

# Auto-move guard: how long sustained step-class bark before we
# declare an antenna move.  Set to 0 to disable auto-move (operator
# must intervene; the bark continues showing in [CONFIDENCE] logs).
DEFAULT_AUTO_MOVE_THRESHOLD_S = 3600.0


class WatchdogActor:
    """Decides slew / step / no-action from per-cycle watchdog state.

    Pure state machine — no side effects.  Caller (run_steady_state)
    consumes the decision dict and performs the actual actions
    (update FixedPosFilter.pos, bump mount_sn, invalidate ppp.toml,
    stop the engine).

    Inputs per evaluate() call:
      - t_now: wall-clock monotonic time
      - nav2_disp_3d_m: latest 3D NAV2-vs-pin displacement (None if
        NAV2 opinion stale)
      - antpos_state: dict snapshot of AntPosEstWatchdog.state

    Returns one of these decisions:
      {'action': 'none', ...}
      {'action': 'slew', 'source': 'nav2'|'antpos', 'displ_m': X,
       'new_pin_ecef': (x, y, z)}
      {'action': 'step', 'source': 'nav2'|'antpos', 'displ_m': X,
       'step_held_s': S}  (only after step sustain window elapses)
      {'action': 'step_pending', 'source': ..., 'displ_m': X,
       'remaining_s': T}  (large displacement seen; counting down)

    Sustained-bark mechanics:
      - Each watchdog has a separate bark-start timestamp; reset
        when the watchdog goes quiet (below its threshold).
      - "Sustained" = (t_now - bark_start_t) >= sustain_s.
      - For slew the sustain is short; for step the sustain is
        auto_move_threshold_s (1 hr default).
      - The `_slewed_for_bark` latch prevents re-slewing on the
        same bark; resets when bark goes quiet.
    """

    def __init__(self,
                 *,
                 slew_step_threshold_m: float = DEFAULT_SLEW_STEP_THRESHOLD_M,
                 nav2_threshold_m: float = DEFAULT_NAV2_THRESHOLD_M,
                 nav2_sustain_s: float = DEFAULT_NAV2_SUSTAIN_S,
                 antpos_threshold_m: float = DEFAULT_ANTPOS_THRESHOLD_M,
                 antpos_sustain_s: float = DEFAULT_ANTPOS_SUSTAIN_S,
                 auto_move_threshold_s: float = DEFAULT_AUTO_MOVE_THRESHOLD_S):
        self.slew_step_threshold_m = float(slew_step_threshold_m)
        self.nav2_threshold_m = float(nav2_threshold_m)
        self.nav2_sustain_s = float(nav2_sustain_s)
        self.antpos_threshold_m = float(antpos_threshold_m)
        self.antpos_sustain_s = float(antpos_sustain_s)
        self.auto_move_threshold_s = float(auto_move_threshold_s)
        # Per-watchdog bark-start timestamps (None = currently quiet).
        self._nav2_bark_start_t: Optional[float] = None
        self._antpos_bark_start_t: Optional[float] = None
        # Latch: once we slewed for a bark, don't slew again until the
        # bark goes quiet.  Indexed by source.
        self._slewed_sources: set = set()
        # Diagnostic counters.
        self.n_slew = 0
        self.n_step = 0

    def evaluate(self,
                 t_now: float,
                 nav2_disp_3d_m: Optional[float],
                 nav2_ecef_m: Optional[tuple],
                 antpos_state: dict) -> dict:
        """One evaluation cycle.  Updates bark-start timestamps and
        returns the action dict."""
        # Update NAV2 bark state.
        nav2_barking = (nav2_disp_3d_m is not None
                        and nav2_disp_3d_m > self.nav2_threshold_m)
        if nav2_barking:
            if self._nav2_bark_start_t is None:
                self._nav2_bark_start_t = t_now
        else:
            self._nav2_bark_start_t = None
            self._slewed_sources.discard("nav2")

        # Update AntPosEst bark state (only meaningful when armed).
        antpos_armed = bool(antpos_state.get("active"))
        antpos_disp_3d = antpos_state.get("displ_3d_m")
        antpos_barking = (antpos_armed
                          and antpos_disp_3d is not None
                          and antpos_disp_3d > self.antpos_threshold_m)
        if antpos_barking:
            if self._antpos_bark_start_t is None:
                self._antpos_bark_start_t = t_now
        else:
            self._antpos_bark_start_t = None
            self._slewed_sources.discard("antpos")

        # Compute sustain status for each source.
        candidates = []
        if (self._nav2_bark_start_t is not None
                and t_now - self._nav2_bark_start_t >= self.nav2_sustain_s):
            candidates.append(("nav2", float(nav2_disp_3d_m),
                               self._nav2_bark_start_t, nav2_ecef_m))
        if (self._antpos_bark_start_t is not None
                and t_now - self._antpos_bark_start_t
                >= self.antpos_sustain_s):
            candidates.append(("antpos", float(antpos_disp_3d),
                               self._antpos_bark_start_t,
                               antpos_state.get("mean_ecef_m")))

        if not candidates:
            return {"action": "none"}

        # Largest sustained displacement decides.
        source, disp, bark_start_t, new_pin = max(
            candidates, key=lambda c: c[1])
        bark_age_s = t_now - bark_start_t

        # Step path: large displacement, must hold for the full
        # auto_move_threshold_s window before we declare a move.
        # auto_move_threshold_s == 0 disables auto-move entirely;
        # the caller just sees step_pending forever and acts manually.
        if disp >= self.slew_step_threshold_m:
            if (self.auto_move_threshold_s > 0
                    and bark_age_s >= self.auto_move_threshold_s):
                self.n_step += 1
                return {"action": "step", "source": source,
                        "displ_m": disp, "step_held_s": bark_age_s}
            return {
                "action": "step_pending",
                "source": source,
                "displ_m": disp,
                "remaining_s": (self.auto_move_threshold_s - bark_age_s
                                if self.auto_move_threshold_s > 0
                                else float("inf")),
            }

        # Slew path: smaller displacement, sustained, latch not held.
        if source in self._slewed_sources:
            return {"action": "none"}
        if new_pin is None:
            # Source didn't give us a target ECEF.  Can't slew without
            # somewhere to slew to; treat as no-op.
            return {"action": "none"}
        self._slewed_sources.add(source)
        self.n_slew += 1
        return {"action": "slew", "source": source, "displ_m": disp,
                "new_pin_ecef": tuple(float(v) for v in new_pin)}


# ── Geometry: ECEF displacement decomposition ────────────────────── #


def compute_horizontal_displacement(
    point_a: tuple, point_b: tuple
) -> tuple[float, float, float]:
    """Decompose the ECEF displacement (point_a - point_b) into
    horizontal and vertical components, using geocentric-up at
    point_a as the local frame's up direction.

    Returns (disp_3d_m, disp_h_m, disp_v_m).

    The horizontal magnitude is what matters for antenna-move
    detection — physical mount moves are dominantly horizontal,
    while PPP vertical wander (mainly tropo/iono residuals) is
    not an antenna-move signature.  Established empirically on
    clkPoC3 in 2026-04-18 when altitude-based watchdog reset
    cascaded 22 times in 3 h.

    Geocentric-up is the unit vector at point_a; for positions
    near Earth's surface this is accurate to ~0.2% as a stand-in
    for true local up.  Good enough for any threshold we care
    about (cm-scale tolerances at km-scale baselines).
    """
    import math
    ax, ay, az = float(point_a[0]), float(point_a[1]), float(point_a[2])
    bx, by, bz = float(point_b[0]), float(point_b[1]), float(point_b[2])
    dx, dy, dz = ax - bx, ay - by, az - bz
    norm_a = math.sqrt(ax * ax + ay * ay + az * az)
    if norm_a == 0:
        # Pathological: point_a at Earth's centre.  Fall back to 3D mag
        # for horizontal, zero for vertical — caller will see a clearly
        # garbage NAV2 fix anyway.
        d3 = math.sqrt(dx * dx + dy * dy + dz * dz)
        return d3, d3, 0.0
    ux, uy, uz = ax / norm_a, ay / norm_a, az / norm_a
    vertical = dx * ux + dy * uy + dz * uz
    hx = dx - vertical * ux
    hy = dy - vertical * uy
    hz = dz - vertical * uz
    disp_h = math.sqrt(hx * hx + hy * hy + hz * hz)
    disp_v = abs(vertical)
    disp_3d = math.sqrt(dx * dx + dy * dy + dz * dz)
    return disp_3d, disp_h, disp_v


# ── Periodic writer ───────────────────────────────────────────────── #


# Default cadence: write at most every 10 min.  More frequent writes
# would still be correct but add filesystem churn for no useful
# improvement in cold-start seed quality.
DEFAULT_PPP_WRITE_PERIOD_S = 600.0

# Sigma above this is "worse than NAV2's typical hAcc" — writing
# would poison the next cold start because pick_most_confident would
# pick our noisy snapshot over NAV2's tighter live fix.
DEFAULT_PPP_WRITE_MAX_SIGMA_M = 1.0


class PppStateWriter:
    """Throttled + sigma-gated writer for state/positions/<uid>.ppp.toml.

    Lives inside the engine's AntPosEstThread.  Engine calls
    ``maybe_write(pos_ecef, sigma_3d, n_epochs)`` every AntPosEst
    epoch; this class decides whether to actually persist based on:

      - receiver_uid not None (else no-op for tests/dev)
      - sigma_3d <= max_sigma_m (else skip, don't poison next start)
      - time.monotonic() >= next_write_t (throttle)

    ``time_fn`` and ``save_fn`` are injectable for testing.
    """

    def __init__(self,
                 receiver_uid,
                 mount_sn: int,
                 *,
                 period_s: float = DEFAULT_PPP_WRITE_PERIOD_S,
                 max_sigma_m: float = DEFAULT_PPP_WRITE_MAX_SIGMA_M,
                 positions_dir: Optional[str] = None,
                 time_fn=None,
                 save_fn=None):
        import time as _time
        self.receiver_uid = receiver_uid
        self.mount_sn = int(mount_sn)
        self.period_s = float(period_s)
        self.max_sigma_m = float(max_sigma_m)
        self.positions_dir = positions_dir
        self._time_fn = time_fn or _time.monotonic
        self._save_fn = save_fn or save_ppp_state
        self._next_write_t: Optional[float] = None
        # Diagnostic counters for the engine's status reporting.
        self.n_writes = 0
        self.n_skipped_sigma = 0
        self.n_skipped_throttle = 0

    def maybe_write(self, pos_ecef, sigma_3d: float,
                    n_epochs: int) -> bool:
        """Write a PositionState snapshot if the gates allow.

        Returns True iff a write actually happened.
        """
        if self.receiver_uid is None:
            return False
        if sigma_3d > self.max_sigma_m:
            self.n_skipped_sigma += 1
            return False
        now = self._time_fn()
        if (self._next_write_t is not None
                and now < self._next_write_t):
            self.n_skipped_throttle += 1
            return False
        state = PositionState(
            mount_sn=self.mount_sn,
            ecef_m=(float(pos_ecef[0]), float(pos_ecef[1]),
                    float(pos_ecef[2])),
            sigma_m=float(sigma_3d),
            updated=utc_now_iso(),
            source="peppar_fix_engine AntPosEst",
            kind="ppp",
            extra={"n_epochs": int(n_epochs)},
        )
        try:
            self._save_fn(state, self.receiver_uid,
                          positions_dir=self.positions_dir)
        except OSError as e:
            log.warning("[PPP_STATE_WRITE] failed: %s", e)
            # Re-arm the throttle anyway so we don't spin retrying
            # on persistent filesystem failure.
            self._next_write_t = now + self.period_s
            return False
        self.n_writes += 1
        self._next_write_t = now + self.period_s
        log.info(
            "[PPP_STATE_WRITE] σ=%.4fm n_epochs=%d mount_sn=%d → "
            "state/positions/%s.ppp.toml",
            sigma_3d, n_epochs, self.mount_sn, self.receiver_uid,
        )
        return True


# ── Selection ─────────────────────────────────────────────────────── #


def filter_current_mount(states: list[Optional[PositionState]],
                         current_mount_sn: int) -> list[PositionState]:
    """Drop None and any state whose mount_sn != current_mount_sn.

    Stale-mount entries arise after antenna moves: the operator (or
    auto-move logic) bumps the receiver's mount_sn and the existing
    position files become stale.  Filtering keyed on mount_sn is the
    atomic discard mechanism.
    """
    out: list[PositionState] = []
    for s in states:
        if s is None:
            continue
        if s.mount_sn != current_mount_sn:
            log.info("Discarding stale-mount %s state "
                     "(file mount_sn=%d, current=%d)",
                     s.kind, s.mount_sn, current_mount_sn)
            continue
        out.append(s)
    return out


def bump_mount_sn(uid, receivers_dir: Optional[str] = None) -> int:
    """Atomically increment ``mount_sn`` in
    ``state/receivers/<uid>.json``, return the new value.

    Used by the step-action path (slice 7).  When the receiver state
    file is absent or lacks ``mount_sn``, we treat that as
    ``mount_sn = 0`` and write ``mount_sn = 1``.

    Refuses (and returns -1) when ``uid`` is None.
    """
    if uid is None:
        return -1
    from peppar_fix.receiver_state import (
        load_receiver_state, save_receiver_state,
    )
    state = load_receiver_state(uid, state_dir=receivers_dir)
    if state is None:
        # Build minimal state so the bump is recorded somewhere.
        state = {"unique_id": int(uid) if str(uid).isdigit() else uid,
                 "mount_sn": 0}
    current = int(state.get("mount_sn", 0))
    state["mount_sn"] = current + 1
    save_receiver_state(state, state_dir=receivers_dir)
    log.info("Bumped mount_sn for uid=%s: %d → %d",
             uid, current, current + 1)
    return current + 1


def invalidate_ppp_state(uid,
                         positions_dir: Optional[str] = None) -> bool:
    """Delete the receiver's .ppp.toml — the next engine launch falls
    back to .survey.toml (if present) or NAV2.  Used by the step
    action path (slice 7) when an antenna move makes the cached
    PPP snapshot stale.

    Returns True if a file was removed, False if there was nothing
    to remove (no-op).  No-ops on uid=None.
    """
    if uid is None:
        return False
    path = _ppp_path(uid, positions_dir)
    if not os.path.exists(path):
        return False
    try:
        os.remove(path)
        log.info("Invalidated PPP state: removed %s", path)
        return True
    except OSError as e:
        log.warning("Failed to remove %s: %s", path, e)
        return False


def load_current_mount_sn(uid,
                          receivers_dir: Optional[str] = None) -> int:
    """Read the receiver's current ``mount_sn`` from
    ``state/receivers/<uid>.json``.  Returns 0 when the file is absent
    or the field is missing — this is the "first run after this
    feature lands" default and matches the fresh-install case.

    The mount_sn lives in receivers/, not positions/, because it's a
    property of "which antenna is this receiver currently connected
    to" — receiver-side identity that the engine writes.
    """
    if uid is None:
        return 0
    try:
        from peppar_fix.receiver_state import load_receiver_state
    except ImportError:  # pragma: no cover — should never happen in production
        log.warning("Cannot import receiver_state; defaulting mount_sn=0")
        return 0
    rstate = load_receiver_state(uid, state_dir=receivers_dir)
    if not rstate:
        return 0
    try:
        return int(rstate.get("mount_sn", 0))
    except (TypeError, ValueError):
        log.warning("mount_sn in receiver state is not an int; "
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
    .survey.toml, filters by current mount_sn, returns the
    most-confident state (or None when no usable state exists).

    Caller behavior on None: fall back to NAV2 / bootstrap.

    Args:
        uid: receiver unique_id (int or str).  None disables the
            whole helper (returns None).
        ignore_ppp: skip the .ppp.toml file (CLI --ignore-ppp).
        ignore_survey: skip the .survey.toml file (CLI --ignore-survey).
        positions_dir: override DEFAULT_POSITIONS_DIR (for testing).
        receivers_dir: passed to load_current_mount_sn for mount_sn
            lookup (for testing).

    Returns:
        Most-confident PositionState, or None.
    """
    if uid is None:
        return None
    current_mount_sn = load_current_mount_sn(uid, receivers_dir=receivers_dir)
    candidates: list[Optional[PositionState]] = []
    if not ignore_ppp:
        candidates.append(load_ppp_state(uid, positions_dir=positions_dir))
    if not ignore_survey:
        candidates.append(load_survey_state(uid, positions_dir=positions_dir))
    usable = filter_current_mount(candidates, current_mount_sn)
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

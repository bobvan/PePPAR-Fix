"""Windowed reset budget — the safety net for the in-process reset.

The binary layer (disciplineModeFsm #4) and the B/C/D outlier/restep
cascades all recover a diverged servo with an *in-process* reset
(``DOFreqEst.reset()`` — preserve the actuator, rebuild the filter)
instead of ``exit(5)`` + wrapper relaunch.  That's the right move for a
transient fault.  But a host that needs reset after reset after reset
is broken in a way the wrapper's full re-bootstrap (re-survey, re-init
PEROUT, fresh state) handles better than endless in-process resets.

``ResetBudget`` bounds that: at most ``max_resets`` in any rolling
``window_s`` window.  The (N+1)th request inside the window is DENIED,
and the caller falls through to ``exit(5)``.  After ``window_s`` of
quiet the old resets age out and the budget recovers.

**Shared across all reset reasons.**  One budget covers the binary
layer, the PHC/EKF outlier cascade, the ClockMatrix outlier cascade,
and the PHC-error-restep cascade — they are symptoms of one illness.
A per-detector budget would let a host thrash ``N × (detectors)``
times before the wrapper gets a chance.  See
``docs/exit-five-to-servo-reset.md`` decision (c).

The clock is injected into :meth:`request` (``now`` is a monotonic
timestamp, never wall-clock — survives NTP steps; see
``docs/stream-timescale-correlation.md``) so the window logic is
deterministically testable without sleeping.
"""
from __future__ import annotations

import collections


class ResetBudget:
    """Rolling-window cap on in-process servo resets.

    Args:
        max_resets: Maximum resets allowed within any ``window_s``
            window.  Must be ≥ 1.
        window_s: Rolling-window length in seconds.  Must be > 0.
        enabled: When False, :meth:`request` always denies — the
            engine keeps its legacy exit-5 behavior.  The budget is
            only consulted when in-process reset is opted in (via
            ``--gross-fault-reset``).
    """

    def __init__(self, *, max_resets: int = 3, window_s: float = 300.0,
                 enabled: bool = False):
        if max_resets < 1:
            raise ValueError(f"max_resets must be ≥ 1, got {max_resets}")
        if window_s <= 0:
            raise ValueError(f"window_s must be > 0, got {window_s}")
        self.max_resets = int(max_resets)
        self.window_s = float(window_s)
        self.enabled = bool(enabled)
        # Monotonic timestamps of the resets we ALLOWED, pruned to the
        # window on each request.
        self._allowed_ts: collections.deque[float] = collections.deque()
        self.cumulative_by_reason: dict[str, int] = {}
        self.total_allowed: int = 0
        self.total_denied: int = 0

    def request(self, reason: str, now: float) -> bool:
        """Ask to perform a reset.  Returns True if within budget
        (caller resets in process), False if the window is full
        (caller should fall through to exit-5).

        A denied request is NOT recorded as an allowed reset — only
        accepted resets count toward the window, so a storm of denied
        requests can't itself keep the window saturated forever.

        A disabled budget always denies — the contract holds even if a
        caller reaches ``request()`` directly instead of through the
        engine's ``_request_servo_reset`` funnel (which also
        short-circuits on ``enabled``).
        """
        if not self.enabled:
            self.total_denied += 1
            return False
        cutoff = now - self.window_s
        while self._allowed_ts and self._allowed_ts[0] < cutoff:
            self._allowed_ts.popleft()
        if len(self._allowed_ts) >= self.max_resets:
            self.total_denied += 1
            return False
        self._allowed_ts.append(now)
        self.cumulative_by_reason[reason] = (
            self.cumulative_by_reason.get(reason, 0) + 1)
        self.total_allowed += 1
        return True

    @property
    def resets_in_window(self) -> int:
        """Allowed resets currently inside the window (as of the last
        :meth:`request`; does not re-prune)."""
        return len(self._allowed_ts)

    @property
    def stats(self) -> dict:
        return dict(
            enabled=self.enabled,
            max_resets=self.max_resets,
            window_s=self.window_s,
            resets_in_window=self.resets_in_window,
            total_allowed=self.total_allowed,
            total_denied=self.total_denied,
            cumulative_by_reason=dict(self.cumulative_by_reason),
        )

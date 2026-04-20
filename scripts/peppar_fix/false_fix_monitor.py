"""False-fix monitor — detects wrong integer NL fixes on short-term members.

Per `docs/sv-lifecycle-and-pfr-split.md` and the data-driven revision
in `project_pfr_event_analysis_20260419.md`: 62% of today's PFR L1
events are wrong-integer fixes — a recently-NL-fixed high-elev SV
starts showing 3–4 m PR residuals within minutes of the fix.  LAMBDA
believed the integer; reality disagrees.  Action: demote the SV back
to FLOAT so it re-accumulates MW/WL evidence.

Called a "false fix": an integer fix that was later shown to be wrong.

This monitor is deliberately **stateless** between evals.  The
previous `PostFixResidualMonitor` had a level-persistence bug: once
it cascaded to L3 it re-fired on every subsequent misfit, burning
~10 min of convergence per re-fire.  The new design evaluates per-SV
conditions each time and decides independently — no ladder, no
persistent escalation level, no memory of past actions except the
residual window itself (which is naturally time-bounded).

Threshold is per-SV and elevation-weighted.  At zenith the bar is
`base_m` (default 2.0 m — tighter than the old monolithic 3.0 m);
at low elev the bar relaxes by 1/sin(elev) to match physics
(troposphere and multipath scale that way).

Scope: watches NL_SHORT_FIXED only.  Long-term members (NL_LONG_FIXED)
have, by definition, already survived geometry-change validation;
the setting-SV drop monitor is the right gate for them as they
descend into multipath.
"""

from __future__ import annotations

import logging
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

from peppar_fix.sv_state import SvAmbState, SvStateTracker

log = logging.getLogger(__name__)


def elev_weighted_threshold(
    base_m: float, elev_deg: Optional[float], clamp_deg: float = 45.0,
) -> float:
    """Compute `base_m * max(1, csc_elev / csc_clamp)`.

    At elev ≥ clamp_deg (default 45°), returns `base_m`.
    Below clamp, scales up by 1/sin(elev) ÷ 1/sin(clamp).  Examples
    with clamp=45° and base=2.0 m: elev=30° → 2.83, elev=25° → 3.35,
    elev=15° → 5.46, elev=10° → 8.15.

    Returns `base_m` if elev is None (no elevation info → trust the
    base threshold).
    """
    if elev_deg is None or elev_deg >= clamp_deg:
        return float(base_m)
    sin_elev = math.sin(math.radians(max(elev_deg, 1.0)))
    sin_clamp = math.sin(math.radians(clamp_deg))
    return float(base_m) * (sin_clamp / sin_elev)


@dataclass
class _SvResidWindow:
    """Per-SV PR |residual| ring buffer with an elevation tag."""
    resids: deque = field(default_factory=lambda: deque(maxlen=30))
    last_elev_deg: Optional[float] = None


class FalseFixMonitor:
    """Detects wrong integer NL fixes on short-term members of the fix set.

    Usage:

        m = FalseFixMonitor(tracker, ...)
        m.ingest(epoch, resid, labels)   # every epoch with filter residuals
        events = m.evaluate(epoch)       # every `eval_every` epochs
        for ev in events:
            # caller unfixes the SV in NL resolver, inflates filter
            # ambiguity, squelches, etc.; tracker has already moved
            # the SV to FLOAT and logged [SV_STATE].
            ...

    Stateless between evals — no ladder, no cooldown that outlives
    the residual window.  If a transition fires, the caller handles
    the downstream effects (unfix, inflate, squelch) — this monitor
    only decides WHICH SVs to transition and updates the tracker.
    """

    def __init__(
        self,
        tracker: SvStateTracker,
        *,
        base_threshold_m: float = 2.0,
        elev_clamp_deg: float = 45.0,
        window_epochs: int = 30,
        min_samples: int = 10,
        eval_every: int = 10,
    ) -> None:
        self._tracker = tracker
        self._base = float(base_threshold_m)
        self._elev_clamp = float(elev_clamp_deg)
        self._window = int(window_epochs)
        self._min_samples = int(min_samples)
        self._eval_every = int(eval_every)
        self._per_sv: dict[str, _SvResidWindow] = {}

    # ── Data intake ─────────────────────────────────────────────── #

    def ingest(self, epoch: int, resid, labels) -> None:
        """Absorb one epoch of filter post-fit residuals.

        Args:
            epoch: monotonic epoch count from the AntPosEst thread.
            resid: iterable of residual magnitudes (meters, signed or
                unsigned — we take abs).
            labels: iterable aligned with `resid`; each entry is
                ``(sv, 'pr'|'phi', elev_deg_or_None)``.

        Only PR residuals for SVs currently in NL_SHORT_FIXED land in
        the window.  Other entries are ignored — the caller doesn't
        have to pre-filter.
        """
        if resid is None:
            return
        for lab, r in zip(labels, resid):
            sv, kind = lab[0], lab[1]
            elev = lab[2] if len(lab) > 2 else None
            if kind != 'pr':
                continue
            if self._tracker.state(sv) is not SvAmbState.NL_SHORT_FIXED:
                continue
            w = self._per_sv.get(sv)
            if w is None:
                w = _SvResidWindow(resids=deque(maxlen=self._window))
                self._per_sv[sv] = w
            w.resids.append(abs(float(r)))
            if elev is not None:
                w.last_elev_deg = float(elev)
                self._tracker.update_elev(sv, elev)

    # ── Evaluation ──────────────────────────────────────────────── #

    def evaluate(self, epoch: int) -> list[dict]:
        """Check each NL_SHORT_FIXED SV with enough samples.

        Returns a list of action dicts, one per SV that failed the
        gate:  ``{'sv': str, 'mean_resid_m': float, 'threshold_m': float,
        'elev_deg': float|None, 'n': int}``.

        Side effect: for each firing SV the tracker transitions
        NL_SHORT_FIXED → FLOAT (false-fix rejection).  The caller is
        responsible for the downstream teardown (NL unfix, ambiguity
        inflation, squelch).

        Returns an empty list on non-eval epochs.
        """
        if epoch % self._eval_every != 0:
            return []
        events: list[dict] = []
        # Iterate over a snapshot — we mutate the tracker (and thus the
        # "which SVs are NL_SHORT_FIXED" set) as we go.
        for sv, w in list(self._per_sv.items()):
            if self._tracker.state(sv) is not SvAmbState.NL_SHORT_FIXED:
                # SV left NL_SHORT_FIXED by some other path (cycle slip,
                # setting-SV drop, promotion to long-term).  Drop its
                # window so a future fix starts clean.
                self._per_sv.pop(sv, None)
                continue
            n = len(w.resids)
            if n < self._min_samples:
                continue
            mean = sum(w.resids) / n
            thr = elev_weighted_threshold(
                self._base, w.last_elev_deg, clamp_deg=self._elev_clamp,
            )
            if mean > thr:
                events.append({
                    'sv': sv,
                    'mean_resid_m': mean,
                    'threshold_m': thr,
                    'elev_deg': w.last_elev_deg,
                    'n': n,
                })
                reason = (
                    f"|PR resid|={mean:.2f}m > {thr:.2f}m"
                    f" (base {self._base:.1f}m, n={n})"
                )
                self._tracker.transition(
                    sv, SvAmbState.FLOAT,
                    epoch=epoch, reason="false_fix:" + reason,
                    elev_deg=w.last_elev_deg,
                )
                # Drop the window so the SV's re-fix starts with fresh
                # residual history rather than absorbing the old misfit.
                self._per_sv.pop(sv, None)
        return events

    # ── Housekeeping ────────────────────────────────────────────── #

    def forget(self, sv: str) -> None:
        """Drop residual history for an SV (e.g. on slip flush)."""
        self._per_sv.pop(sv, None)

    def summary(self) -> str:
        return f"false_fix: tracking {len(self._per_sv)} NL_SHORT_FIXED SVs"

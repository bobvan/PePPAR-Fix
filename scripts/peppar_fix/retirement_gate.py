"""Job B — setting-SV retirement gate.

Per `docs/sv-lifecycle-and-pfr-split.md`: an SV descending through the
retirement elevation band has multipath-inflated residuals that are
normal physics, not wrong integers.  The job of this monitor is to
release such SVs *gracefully* — transition them to RETIRING so the
filter stops depending on their integers, without touching the rest
of the AR population.

Two trigger conditions, either one fires:

1. **Elev-weighted PR residual exceeds threshold.**  Base 3.0 m at
   zenith (looser than Job A's 2.0 m because this is the "still
   correct but getting noisy" case, not "wrong integer"); scaled
   up by 1/sin(elev) with the same 45° clamp.
2. **Elev below absolute retirement mask.**  Independent of residual
   quality: below `retirement_mask_deg` (default 18°) we retire
   regardless.  Keeps stale low-elev integers from polluting the
   filter when residuals happen to look quiet for a moment.

Like Job A, this monitor is stateless between evals.  No cooldown,
no ladder.  Operates on both NL_PROVISIONAL and NL_VALIDATED SVs:
while Bead 4 hasn't landed, NL_VALIDATED is unreachable and all
NL fixes stay in NL_PROVISIONAL.  Once Bead 4 promotes eligible
fixes to NL_VALIDATED, retirement still works on both (it's the
same question: "is this SV setting?").
"""

from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

from peppar_fix.sv_state import SvAmbState, SvStateTracker
from peppar_fix.provisional_validator import elev_weighted_threshold

log = logging.getLogger(__name__)


@dataclass
class _SvResidWindow:
    resids: deque = field(default_factory=lambda: deque(maxlen=30))
    last_elev_deg: Optional[float] = None


class RetirementGate:
    """Job B monitor — retires NL SVs that have become unreliable.

    Usage mirrors ProvisionalValidator:

        g = RetirementGate(tracker, ...)
        g.ingest(epoch, resid, labels)
        events = g.evaluate(epoch)
        for ev in events:
            # caller releases the NL integer in the resolver (unfix +
            # gentle covariance growth).  Tracker has already moved the
            # SV to RETIRING.

    Stateless per-eval.  Preserves MW/WL history on retirement — the
    SV might rise again later in the same arc or a different arc.
    """

    # SVs eligible for retirement: any NL state.
    _ELIGIBLE = frozenset({SvAmbState.NL_PROVISIONAL, SvAmbState.NL_VALIDATED})

    def __init__(
        self,
        tracker: SvStateTracker,
        *,
        base_threshold_m: float = 3.0,
        elev_clamp_deg: float = 45.0,
        retirement_mask_deg: float = 18.0,
        window_epochs: int = 30,
        min_samples: int = 10,
        eval_every: int = 10,
    ) -> None:
        self._tracker = tracker
        self._base = float(base_threshold_m)
        self._elev_clamp = float(elev_clamp_deg)
        self._retire_mask = float(retirement_mask_deg)
        self._window = int(window_epochs)
        self._min_samples = int(min_samples)
        self._eval_every = int(eval_every)
        self._per_sv: dict[str, _SvResidWindow] = {}

    # ── Data intake ─────────────────────────────────────────────── #

    def ingest(self, epoch: int, resid, labels) -> None:
        if resid is None:
            return
        for lab, r in zip(labels, resid):
            sv, kind = lab[0], lab[1]
            elev = lab[2] if len(lab) > 2 else None
            if kind != 'pr':
                continue
            if self._tracker.state(sv) not in self._ELIGIBLE:
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
        """Return list of retirement events this eval.

        Event dict: ``{'sv': str, 'reason': 'elev_mask'|'elev_weighted_resid',
        'elev_deg': float|None, 'mean_resid_m': float|None,
        'threshold_m': float|None, 'n': int}``.

        Side effect: tracker transitions each firing SV to RETIRING.
        Caller must release the NL integer in the resolver.
        """
        if epoch % self._eval_every != 0:
            return []
        events: list[dict] = []
        for sv, w in list(self._per_sv.items()):
            if self._tracker.state(sv) not in self._ELIGIBLE:
                # SV is no longer eligible (fell to FLOAT via Job A or
                # cycle slip, or already retiring).  Flush its window.
                self._per_sv.pop(sv, None)
                continue
            # Condition 1: absolute elevation mask.  Fires regardless of
            # residual quality — sub-mask integers aren't worth the risk.
            if (
                w.last_elev_deg is not None
                and w.last_elev_deg < self._retire_mask
            ):
                events.append({
                    'sv': sv,
                    'reason': 'elev_mask',
                    'elev_deg': w.last_elev_deg,
                    'mean_resid_m': None,
                    'threshold_m': None,
                    'n': len(w.resids),
                })
                self._tracker.transition(
                    sv, SvAmbState.RETIRING,
                    epoch=epoch,
                    reason=f"job_b:elev={w.last_elev_deg:.0f}° < {self._retire_mask:.0f}°",
                    elev_deg=w.last_elev_deg,
                )
                self._per_sv.pop(sv, None)
                continue

            # Condition 2: elev-weighted PR residual exceeded.
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
                    'reason': 'elev_weighted_resid',
                    'elev_deg': w.last_elev_deg,
                    'mean_resid_m': mean,
                    'threshold_m': thr,
                    'n': n,
                })
                self._tracker.transition(
                    sv, SvAmbState.RETIRING,
                    epoch=epoch,
                    reason=(
                        f"job_b:|PR|={mean:.2f}m > {thr:.2f}m"
                        f" (base {self._base:.1f}m, n={n})"
                    ),
                    elev_deg=w.last_elev_deg,
                )
                self._per_sv.pop(sv, None)
        return events

    # ── Housekeeping ────────────────────────────────────────────── #

    def forget(self, sv: str) -> None:
        self._per_sv.pop(sv, None)

    def summary(self) -> str:
        return f"job_b: tracking {len(self._per_sv)} NL SVs"

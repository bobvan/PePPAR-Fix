"""Bead 4 — NL_SHORT_FIXED → NL_LONG_FIXED promotion.

Per `docs/sv-lifecycle-and-pfr-split.md`: an NL fix that has survived
≥ 15° of satellite-azimuth motion without triggering false-fix
rejection has demonstrated its integer is correct across enough
distinct geometry to graduate from probation — from short-term
member of the fix set to long-term.  The position solution declares
RESOLVED based on long-term member count, not raw NL-fix count, so
lucky-noise fixes that would flip the solution state prematurely
are filtered out.

Shape:

    LongTermPromoter(tracker, ...)
    .ingest_az(sv, az_deg)          per epoch, for every NL-state SV
    .note_false_fix_rejection(sv, ep)   called by the false-fix apply hook
    .evaluate(epoch)                at eval_every cadence
        → list of dicts describing promotions, tracker transitions
          NL_SHORT_FIXED → NL_LONG_FIXED already done.

The promoter is stateless between evals except for the Job-A-rejection
memory (per-SV last-rejection epoch), which is what lets us require a
"clean validation window".  It shares the per-SV state machine with
the other monitors; it doesn't duplicate or shadow state.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

from peppar_fix.sv_state import SvAmbState, SvStateTracker

log = logging.getLogger(__name__)


@dataclass
class _PromoCandidate:
    sv: str
    first_fix_az_deg: Optional[float] = None
    latest_az_deg: Optional[float] = None
    accumulated_dphi: float = 0.0   # |Δaz| accumulated since fix (not wrapped)
    last_false_fix_epoch: int = -10**9  # sentinel: never


def _az_delta(a: float, b: float) -> float:
    """Smallest |a - b| on the circle, in degrees (0..180]."""
    d = (a - b) % 360.0
    if d > 180.0:
        d = 360.0 - d
    return d


class LongTermPromoter:
    """Promotes NL_SHORT_FIXED SVs to NL_LONG_FIXED once geometry has
    changed enough to trust the integer.

    Defaults: Δaz ≥ 15°, clean-window = 180 epochs (≈3 min @ 1 Hz,
    matches false-fix's residual window).  eval_every=10 matches the
    other monitors.
    """

    def __init__(
        self,
        tracker: SvStateTracker,
        *,
        dphi_threshold_deg: float = 15.0,
        clean_window_epochs: int = 180,
        eval_every: int = 10,
    ) -> None:
        self._tracker = tracker
        self._dphi_threshold = float(dphi_threshold_deg)
        self._clean_window = int(clean_window_epochs)
        self._eval_every = int(eval_every)
        self._cands: dict[str, _PromoCandidate] = {}
        self._last_rejection_epoch_by_sv: dict[str, int] = {}

    # ── Data intake ─────────────────────────────────────────────── #

    def ingest_az(self, sv: str, az_deg: Optional[float]) -> None:
        """Record this SV's current azimuth this epoch.

        Only NL_SHORT_FIXED SVs are tracked; calls for other states are
        no-ops (reduces bookkeeping on SVs whose promotion isn't
        relevant, and avoids leaking state across slip → FLOAT → re-fix
        cycles).
        """
        if az_deg is None:
            return
        if self._tracker.state(sv) is not SvAmbState.NL_SHORT_FIXED:
            # Stop tracking if SV left NL_SHORT_FIXED.  Re-entering
            # NL_SHORT_FIXED from a fresh fix starts a new candidate
            # because the tracker's first_fix_az_deg will have been
            # re-populated on the transition.
            self._cands.pop(sv, None)
            return
        rec = self._tracker.get(sv)
        c = self._cands.get(sv)
        if c is None:
            c = _PromoCandidate(
                sv=sv,
                first_fix_az_deg=rec.first_fix_az_deg,
                latest_az_deg=float(az_deg),
                accumulated_dphi=0.0,
            )
            self._cands[sv] = c
            return
        if c.first_fix_az_deg is None:
            # The tracker's first_fix_az_deg wasn't set when the fix
            # was recorded — back-fill on the first az we see after the
            # transition so we at least have some baseline.
            c.first_fix_az_deg = float(az_deg)
            c.latest_az_deg = float(az_deg)
            return
        # Accumulate the per-epoch |Δaz|.  We use incremental
        # accumulation rather than |current − first| so a slowly-moving
        # SV that eventually traces ≥ 15° wins, even if the direct
        # angular distance drops back temporarily (e.g. a GEO-adjacent
        # SV with small net motion but real local changes).
        if c.latest_az_deg is not None:
            c.accumulated_dphi += _az_delta(az_deg, c.latest_az_deg)
        c.latest_az_deg = float(az_deg)

    def note_false_fix_rejection(self, sv: str, epoch: int) -> None:
        """Called by the false-fix apply hook: SV just got demoted back to
        FLOAT.  We record the epoch so any subsequent re-fix has to
        stay clean for `clean_window_epochs` before being promoted.

        Also drops the existing candidate — the SV's NL state changes,
        so the Δaz accumulator should restart when/if it comes back.
        """
        self._cands.pop(sv, None)
        self._last_rejection_epoch_by_sv[sv] = int(epoch)

    # ── Evaluation ──────────────────────────────────────────────── #

    def evaluate(self, epoch: int) -> list[dict]:
        """Promote eligible SVs, return a list of promotion event dicts.

        Event: ``{'sv': str, 'accumulated_dphi_deg': float,
        'first_fix_az_deg': float, 'latest_az_deg': float}``.

        Side effect: tracker transitions each event's SV from
        NL_SHORT_FIXED to NL_LONG_FIXED.  Caller usually does nothing
        else — host RESOLVED logic reads the tracker's count.
        """
        if epoch % self._eval_every != 0:
            return []
        events: list[dict] = []
        rej_map = self._last_rejection_epoch_by_sv
        for sv, c in list(self._cands.items()):
            if self._tracker.state(sv) is not SvAmbState.NL_SHORT_FIXED:
                self._cands.pop(sv, None)
                continue
            if c.accumulated_dphi < self._dphi_threshold:
                continue
            last_rej = rej_map.get(sv)
            if last_rej is not None and (epoch - last_rej) < self._clean_window:
                # Probation not yet clean — stall the promotion.
                continue
            event = {
                'sv': sv,
                'accumulated_dphi_deg': c.accumulated_dphi,
                'first_fix_az_deg': c.first_fix_az_deg,
                'latest_az_deg': c.latest_az_deg,
            }
            events.append(event)
            self._tracker.transition(
                sv, SvAmbState.NL_LONG_FIXED,
                epoch=epoch,
                reason=(
                    f"promoted after Δaz={c.accumulated_dphi:.1f}°"
                    f" (clean window {self._clean_window}ep)"
                ),
            )
            self._cands.pop(sv, None)
        return events

    # ── Housekeeping ────────────────────────────────────────────── #

    def forget(self, sv: str) -> None:
        self._cands.pop(sv, None)

    def summary(self) -> str:
        return f"promoter: tracking {len(self._cands)} NL_SHORT_FIXED SVs"

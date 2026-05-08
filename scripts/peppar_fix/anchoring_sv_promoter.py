"""Bead 4 — ANCHORING → ANCHORED promotion.

Per `docs/sv-lifecycle-and-pfr-split.md`: an NL fix that has survived
≥ 15° of satellite-azimuth motion AND kept its IF residuals within a
realistic envelope (no wrong-integer signature) has demonstrated its
integer is correct across enough distinct geometry to graduate from
probation — from short-term member of the fix set to long-term.  The
position solution declares RESOLVED based on long-term member count,
not raw NL-fix count, so lucky-noise fixes that would flip the
solution state prematurely are filtered out.

Empirical motivation (I-224945): day0506pm-piface-arm34-bias-v2 ran
the engine with the prior 8° default and produced a false promotion
of E12 at Δaz=8.2° while its NL fix was wrong-integer (later unfixed
by nl_resolver).  Replay (see /home/bob/i-224945-piface-replay/) shows
E12's IF residuals during ANCHORING were systematically biased
(signed mean +79.5 mm over the 31-min window) — the canonical
wrong-integer signature.  Raising the Δaz threshold to 15° AND
gating on a |signed-mean IF residual| ceiling blocks E12 plus four
other promotions in that 18-h log that show similar bias.

Shape:

    AnchoringSvPromoter(tracker, ...)
    .ingest_az(sv, az_deg)              per epoch, for every NL-state SV
    .ingest_if_resid(sv, if_resid_m, ep) per epoch when [NL_RESID] line emits
    .note_false_fix_rejection(sv, ep)   called by the false-fix apply hook
    .evaluate(epoch)                    at eval_every cadence
        → list of dicts describing promotions, tracker transitions
          ANCHORING → ANCHORED already done.

The promoter is stateless between evals except for the Job-A-rejection
memory (per-SV last-rejection epoch) and a small per-SV ring of recent
IF residuals.  It shares the per-SV state machine with the other
monitors; it doesn't duplicate or shadow state.
"""

from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass, field
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
    # Ring of recent (epoch, if_resid_m) for the IF-residual gate.
    # bounded to if_resid_window_epochs by ingest_if_resid; per-SV
    # deque is cheap (typical ~5-10 NL-fixed SVs * 60-180 entries).
    if_resids: deque = field(default_factory=lambda: deque())


def _az_delta(a: float, b: float) -> float:
    """Smallest |a - b| on the circle, in degrees (0..180]."""
    d = (a - b) % 360.0
    if d > 180.0:
        d = 360.0 - d
    return d


class AnchoringSvPromoter:
    """Promotes ANCHORING SVs to ANCHORED once geometry has
    changed enough to trust the integer AND IF residuals stay
    within a wrong-integer-free envelope.

    Defaults:
      - dphi_threshold_deg=15° (was 8° pre-I-224945; the engine's
        own comment at line 1884 already documented "≥ 15°" so this
        restores spec/code agreement).
      - if_resid_threshold_m=0.060 (60 mm signed-mean ceiling).
        Empirical: day0506pm-piface-arm34-bias-v2 had E12's wrong
        integer bias the IF residual to +79.5 mm signed mean over
        its 31-min ANCHORING window; clean integer SVs in the same
        log sat in the [-50, +25] mm range.
      - if_resid_window_epochs=60 (1 min at 1 Hz).  Shorter than the
        clean_window so the gate is responsive without being noisy.
      - clean_window_epochs=180 (≈3 min @ 1 Hz, matches false-fix's
        residual window).
      - eval_every=10 matches the other monitors.

    Disable the IF-residual filter by passing if_resid_threshold_m=None
    (e.g. for ablation testing or hosts with no NL-residual logger).
    """

    def __init__(
        self,
        tracker: SvStateTracker,
        *,
        dphi_threshold_deg: float = 15.0,
        if_resid_threshold_m: Optional[float] = 0.060,
        if_resid_window_epochs: int = 60,
        if_resid_min_samples: int = 30,
        clean_window_epochs: int = 180,
        eval_every: int = 10,
    ) -> None:
        self._tracker = tracker
        self._dphi_threshold = float(dphi_threshold_deg)
        self._if_resid_threshold = (
            None if if_resid_threshold_m is None
            else float(if_resid_threshold_m)
        )
        self._if_resid_window = int(if_resid_window_epochs)
        self._if_resid_min_samples = int(if_resid_min_samples)
        self._clean_window = int(clean_window_epochs)
        self._eval_every = int(eval_every)
        self._cands: dict[str, _PromoCandidate] = {}
        self._last_rejection_epoch_by_sv: dict[str, int] = {}

    # ── Data intake ─────────────────────────────────────────────── #

    def ingest_az(self, sv: str, az_deg: Optional[float]) -> None:
        """Record this SV's current azimuth this epoch.

        Only ANCHORING SVs get their accumulator updated; calls
        for other states skip the update but preserve the candidate
        state.  Candidates are dropped by:
          - `note_false_fix_rejection` (real integer problem)
          - `forget(sv)` called by the engine when the tracker forgets
            the record (arc boundary)

        Transient state excursions (cycle slip → FLOATING → re-fix) do
        NOT drop the candidate.  Day0419i data showed E23 slipping
        every 3-8 min; under the old "drop on state change" behavior
        the accumulator never completed 8° because each slip reset
        it.  Preserving it across slip/re-fix cycles lets a signal
        that's merely noisy still earn long-term promotion.
        """
        if az_deg is None:
            return
        if self._tracker.state(sv) is not SvAmbState.ANCHORING:
            # Candidate (if any) stays in _cands for later resumption.
            # The accumulated_dphi is preserved across the gap.
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
        # SV that eventually traces ≥ 8° wins, even if the direct
        # angular distance drops back temporarily (e.g. a GEO-adjacent
        # SV with small net motion but real local changes).
        if c.latest_az_deg is not None:
            c.accumulated_dphi += _az_delta(az_deg, c.latest_az_deg)
        c.latest_az_deg = float(az_deg)

    def ingest_if_resid(
        self, sv: str, if_resid_m: Optional[float], epoch: int,
    ) -> None:
        """Record this SV's post-fit IF residual this epoch (in metres).

        Called once per epoch per NL-state SV from the engine, right after
        the [NL_RESID] log line.  Only ANCHORING SVs accumulate samples;
        callers for other states are no-ops.  Samples older than
        ``if_resid_window_epochs`` are evicted lazily at evaluate time
        AND on each ingest, keeping the per-SV deque bounded.

        ``if_resid_m=None`` is a missing-sample marker (e.g. the filter
        didn't emit phi for this SV this epoch); silently skip — no
        zero-stuffing.
        """
        if if_resid_m is None:
            return
        if self._if_resid_threshold is None:
            return  # filter disabled
        if self._tracker.state(sv) is not SvAmbState.ANCHORING:
            return  # only ANCHORING accumulates
        c = self._cands.get(sv)
        if c is None:
            return  # promoter hasn't seen the az transition yet; ignore
        c.if_resids.append((int(epoch), float(if_resid_m)))
        # Evict samples older than the window from the FRONT of the deque.
        cutoff = int(epoch) - self._if_resid_window
        while c.if_resids and c.if_resids[0][0] < cutoff:
            c.if_resids.popleft()

    def note_false_fix_rejection(self, sv: str, epoch: int) -> None:
        """Called by the false-fix apply hook: SV just got demoted back to
        FLOATING.  We record the epoch so any subsequent re-fix has to
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
        ANCHORING to ANCHORED.  Caller usually does nothing
        else — host RESOLVED logic reads the tracker's count.
        """
        if epoch % self._eval_every != 0:
            return []
        events: list[dict] = []
        rej_map = self._last_rejection_epoch_by_sv
        for sv, c in list(self._cands.items()):
            if self._tracker.state(sv) is not SvAmbState.ANCHORING:
                self._cands.pop(sv, None)
                continue
            if c.accumulated_dphi < self._dphi_threshold:
                continue
            last_rej = rej_map.get(sv)
            if last_rej is not None and (epoch - last_rej) < self._clean_window:
                # Probation not yet clean — stall the promotion.
                continue

            # IF-residual filter: reject if recent residuals show a
            # wrong-integer-style sustained bias.  Computed as
            # |signed mean| over the last ``if_resid_window_epochs``
            # samples; need at least ``if_resid_min_samples`` for a
            # meaningful estimate.  Below the min sample count we DEFER
            # (don't promote yet) rather than waive — better to wait one
            # more eval cycle than to promote on an under-sampled SV.
            if_signed_mean = None
            if self._if_resid_threshold is not None:
                # Lazy window eviction: same logic as ingest, in case the
                # SV's last ingest was many epochs ago.
                cutoff = epoch - self._if_resid_window
                while c.if_resids and c.if_resids[0][0] < cutoff:
                    c.if_resids.popleft()
                n = len(c.if_resids)
                if n < self._if_resid_min_samples:
                    # Defer; not enough recent samples to judge.
                    continue
                if_signed_mean = sum(v for (_, v) in c.if_resids) / n
                if abs(if_signed_mean) > self._if_resid_threshold:
                    # Bias too large — wrong-integer signature.  Don't
                    # promote; leave candidate alive so it can re-evaluate
                    # at the next eval window if residuals recover.
                    log.info(
                        f"[ANCHORING_PROMOTE_DEFERRED] sv={sv} "
                        f"Δaz={c.accumulated_dphi:.1f}° "
                        f"if_resid_mean={if_signed_mean*1000:+.1f}mm "
                        f"(|threshold|={self._if_resid_threshold*1000:.0f}mm, "
                        f"n={n} samples)"
                    )
                    continue

            event = {
                'sv': sv,
                'accumulated_dphi_deg': c.accumulated_dphi,
                'first_fix_az_deg': c.first_fix_az_deg,
                'latest_az_deg': c.latest_az_deg,
                'if_resid_signed_mean_m': if_signed_mean,
            }
            events.append(event)
            reason_bits = [
                f"Δaz={c.accumulated_dphi:.1f}°",
                f"clean window {self._clean_window}ep",
            ]
            if if_signed_mean is not None:
                reason_bits.append(
                    f"if_resid_mean={if_signed_mean*1000:+.1f}mm"
                )
            self._tracker.transition(
                sv, SvAmbState.ANCHORED,
                epoch=epoch,
                reason=f"promoted after {' '.join(reason_bits)}",
            )
            self._cands.pop(sv, None)
        return events

    # ── Housekeeping ────────────────────────────────────────────── #

    def forget(self, sv: str) -> None:
        self._cands.pop(sv, None)

    def summary(self) -> str:
        return f"promoter: tracking {len(self._cands)} ANCHORING SVs"

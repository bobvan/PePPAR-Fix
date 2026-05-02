"""Setting-SV drop monitor — graceful drop as SVs descend.

Per `docs/sv-lifecycle-and-pfr-split.md`: an SV descending through
the retirement elevation band has multipath-inflated residuals that
are normal physics, not wrong integers.  The job of this monitor is
to drop such SVs from the fix set *gracefully* — transition them
back to FLOATING so the filter stops relying on their integers, without
touching the rest of the AR population.

Called a "setting-SV drop": the intentional removal of an SV from
the fix set as it descends into multipath-prone elevations.

Two trigger conditions, either one fires:

1. **Elev-weighted PR residual exceeds threshold in the setting
   band, AND IF residual also breaches.**  Base 3.0 m PR threshold
   at `elev_clamp_deg` (45° zenith cap); scaled up by 1/sin(elev)
   below the clamp.  Only fires when elev is in the setting band —
   between `drop_mask_deg` (default 18°) and `residual_ceiling_deg`
   (default 30°).  Above the ceiling an SV is not physically setting
   — residual issues at high elev are FalseFixMonitor's domain
   (tighter 2.0 m threshold, designed for wrong integers) or
   filter-health issues handled by the integrity monitor / join test.

   The IF gate (I-161514, 2026-05-02): require window-mean phi
   residual > `if_threshold_m` (default 0.05 m) in addition to the
   PR breach.  PR multipath at low elevation inflates the code
   residual without affecting the carrier-phase integer; the IF
   residual stays sub-cm even at 25-29° elev when the integer is
   correct.  Without this gate, day0501 overnight saw 100% of
   same-second wasted evictions (24/24 across TimeHat + MadHat) —
   evict + re-admit at the same integer 1 second later.  See
   docs/misnomers.md "Residual-domain mismatch" pattern.
2. **Elev below absolute drop mask.**  Independent of residual
   quality: below `drop_mask_deg` (default 18°) we drop regardless.
   Keeps stale low-elev integers from polluting the filter when
   residuals happen to look quiet for a moment.

Like the false-fix monitor, this is stateless between evals.  No
cooldown, no ladder.  Operates on both short-term (ANCHORING)
and long-term (ANCHORED) members — setting is setting, and
both kinds of fix deserve the same graceful drop.

The SV transitions to FLOATING on drop, not to an intermediate
"retiring" state.  MW/WL state is preserved by the MW tracker for
fast re-acquisition if the SV rises again (see slip-retain-freq
feedback memory).  If the SV has truly set out of tracking, the
engine forgets its record on the next observation cycle.

Diagnostic: an SV can only physically set once per pass.  A
second setting_sv_drop event on the same SV within a session
indicates either (a) a persistent multipath/obstruction at a
specific sky region, or (b) filter health issues masquerading as
setting (the filter's position/ZTD state drifted and the anchor's
PR residual followed).  Neither is real setting.  We track the
count and log a warning; a future refinement could use the count
to adjust per-SV behavior (skip the drop, raise the threshold,
or mark the sky region as multipath-prone).
"""

from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

from peppar_fix.sv_state import SvAmbState, SvStateTracker
from peppar_fix.false_fix_monitor import elev_weighted_threshold

log = logging.getLogger(__name__)


@dataclass
class _SvResidWindow:
    resids: deque = field(default_factory=lambda: deque(maxlen=30))
    if_resids: deque = field(default_factory=lambda: deque(maxlen=30))
    last_elev_deg: Optional[float] = None


class SettingSvDropMonitor:
    """Drops NL SVs that have become unreliable as they descend.

    Usage mirrors the false-fix monitor:

        m = SettingSvDropMonitor(tracker, ...)
        m.ingest(epoch, resid, labels)
        events = m.evaluate(epoch)
        for ev in events:
            # caller unfixes the NL integer in the resolver (gentle
            # covariance growth).  Tracker has already moved the SV
            # to FLOATING.

    Stateless per-eval.  Preserves MW/WL history on drop — the SV
    might rise again later in a different arc.
    """

    # SVs eligible for a drop: any NL member of the fix set.
    _ELIGIBLE = frozenset({SvAmbState.ANCHORING, SvAmbState.ANCHORED})

    def __init__(
        self,
        tracker: SvStateTracker,
        *,
        base_threshold_m: float = 3.0,
        elev_clamp_deg: float = 45.0,
        drop_mask_deg: float = 18.0,
        residual_ceiling_deg: float = 30.0,
        window_epochs: int = 30,
        min_samples: int = 10,
        eval_every: int = 10,
        if_threshold_m: float = 0.05,
    ) -> None:
        self._tracker = tracker
        self._base = float(base_threshold_m)
        self._elev_clamp = float(elev_clamp_deg)
        self._drop_mask = float(drop_mask_deg)
        self._resid_ceiling = float(residual_ceiling_deg)
        self._window = int(window_epochs)
        self._min_samples = int(min_samples)
        self._eval_every = int(eval_every)
        # IF-residual gate (I-161514, 2026-05-02): Condition 2 (elev-
        # weighted PR breach) requires the IF (carrier-phase) residual
        # to also breach this threshold.  Empirically false-positive
        # PR-multipath storms have IF residuals at sub-cm; legitimate
        # wrong-integer pathology has IF > 100mm.  50mm is the
        # conservative inner edge of that boundary.  See
        # docs/misnomers.md "Residual-domain mismatch" pattern.
        self._if_threshold = float(if_threshold_m)
        self._per_sv: dict[str, _SvResidWindow] = {}
        # Per-SV drop count across this session.  An SV only sets
        # once; a second drop is diagnostic for multipath or filter
        # health.  Survives state transitions (not flushed on drop).
        self._drop_count: dict[str, int] = {}

    # ── Data intake ─────────────────────────────────────────────── #

    def ingest(self, epoch: int, resid, labels) -> None:
        if resid is None:
            return
        for lab, r in zip(labels, resid):
            sv, kind = lab[0], lab[1]
            elev = lab[2] if len(lab) > 2 else None
            # Accept both PR (code) and phi (carrier-phase) residuals.
            # PR drives Condition 2's elev-weighted threshold; phi
            # provides the IF-residual gate that suppresses
            # PR-multipath false positives.
            if kind not in ('pr', 'phi'):
                continue
            if self._tracker.state(sv) not in self._ELIGIBLE:
                continue
            w = self._per_sv.get(sv)
            if w is None:
                w = _SvResidWindow(
                    resids=deque(maxlen=self._window),
                    if_resids=deque(maxlen=self._window),
                )
                self._per_sv[sv] = w
            if kind == 'pr':
                w.resids.append(abs(float(r)))
                if elev is not None:
                    w.last_elev_deg = float(elev)
                    self._tracker.update_elev(sv, elev)
            else:  # 'phi'
                w.if_resids.append(abs(float(r)))

    # ── Evaluation ──────────────────────────────────────────────── #

    def evaluate(self, epoch: int) -> list[dict]:
        """Return list of setting-SV drop events this eval.

        Event dict: ``{'sv': str, 'reason': 'elev_mask'|'elev_weighted_resid',
        'elev_deg': float|None, 'mean_resid_m': float|None,
        'threshold_m': float|None, 'n': int}``.

        Side effect: tracker transitions each firing SV to FLOATING
        (setting-SV drop).  Caller must release the NL integer in
        the resolver (unfix, gentle covariance growth).
        """
        if epoch % self._eval_every != 0:
            return []
        events: list[dict] = []
        for sv, w in list(self._per_sv.items()):
            if self._tracker.state(sv) not in self._ELIGIBLE:
                # SV is no longer eligible (fell to FLOATING via false-fix
                # monitor or cycle slip).  Flush its window.
                self._per_sv.pop(sv, None)
                continue
            # Condition 1: absolute elevation mask.  Fires regardless of
            # residual quality — sub-mask integers aren't worth the risk.
            if (
                w.last_elev_deg is not None
                and w.last_elev_deg < self._drop_mask
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
                    sv, SvAmbState.FLOATING,
                    epoch=epoch,
                    reason=f"setting_sv_drop:elev={w.last_elev_deg:.0f}° < {self._drop_mask:.0f}°",
                    elev_deg=w.last_elev_deg,
                )
                self._per_sv.pop(sv, None)
                continue

            # Condition 2: elev-weighted PR residual exceeded — but
            # only in the setting band (elev ≤ residual_ceiling_deg).
            # An SV above the ceiling is not physically setting; a
            # large mean residual at high elev is either a wrong
            # integer (FalseFixMonitor handles it with a stricter
            # 2.0 m base) or a filter-health issue (ZTD/altitude
            # coupling — integrity monitor or join test handles it).
            # Dropping a high-elev anchor here sacrifices the
            # geometry diversity that disentangles ZTD from altitude.
            if (
                w.last_elev_deg is not None
                and w.last_elev_deg > self._resid_ceiling
            ):
                continue
            n = len(w.resids)
            if n < self._min_samples:
                continue
            mean = sum(w.resids) / n
            thr = elev_weighted_threshold(
                self._base, w.last_elev_deg, clamp_deg=self._elev_clamp,
            )
            if mean > thr:
                # IF-residual gate (I-161514, 2026-05-02): suppress
                # PR-multipath false positives.  When the carrier-phase
                # integer is correct the IF residual stays sub-cm even
                # at low elevation; only a wrong integer breaches IF.
                # Require the IF window mean to exceed if_threshold
                # before evicting.  Need at least min_samples IF
                # samples too — if no IF samples are available (rare,
                # most likely an early-arc state), fall through to
                # the legacy PR-only behaviour (don't suppress what we
                # can't classify).
                n_if = len(w.if_resids)
                if n_if >= self._min_samples:
                    if_mean = sum(w.if_resids) / n_if
                    if if_mean <= self._if_threshold:
                        # PR breached but IF clean → integer is correct,
                        # PR is multipath.  Don't evict.
                        log.debug(
                            "[SETTING_SV_DROP_SUPPRESSED] %s elev=%s° "
                            "|PR|=%.2fm > %.2fm but |IF|=%.3fm ≤ %.3fm "
                            "(integer clean; PR multipath only)",
                            sv,
                            f"{w.last_elev_deg:.0f}" if w.last_elev_deg is not None else "?",
                            mean, thr, if_mean, self._if_threshold,
                        )
                        continue
                n_prior = self._drop_count.get(sv, 0) + 1
                self._drop_count[sv] = n_prior
                if n_prior > 1:
                    log.warning(
                        "[SETTING_SV_DROP_REPEAT] %s dropped %d× this "
                        "session (elev=%s°, |PR|=%.2fm > %.2fm); an SV "
                        "only truly sets once — suspect multipath "
                        "hotspot or filter-health drift",
                        sv, n_prior,
                        f"{w.last_elev_deg:.0f}" if w.last_elev_deg is not None else "?",
                        mean, thr,
                    )
                events.append({
                    'sv': sv,
                    'reason': 'elev_weighted_resid',
                    'elev_deg': w.last_elev_deg,
                    'mean_resid_m': mean,
                    'threshold_m': thr,
                    'n': n,
                    'drop_count_session': n_prior,
                })
                self._tracker.transition(
                    sv, SvAmbState.FLOATING,
                    epoch=epoch,
                    reason=(
                        f"setting_sv_drop:|PR|={mean:.2f}m > {thr:.2f}m"
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
        return f"setting_sv_drop: tracking {len(self._per_sv)} NL SVs"

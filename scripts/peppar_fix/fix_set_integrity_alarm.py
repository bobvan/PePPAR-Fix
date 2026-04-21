"""Fix-set integrity alarm — catches systemic failures the per-SV
monitors can't attribute to one satellite.

Per `docs/sv-lifecycle-and-pfr-split.md`: the new design expects
per-SV issues to be handled by the false-fix monitor and the
setting-SV drop monitor.  This fix-set-wide alarm fires only for
the residual case where *many* members misbehave at once without
any single one being the culprit — genuine systemic failure (bad
SSR correction batch, clock-datum change, reference-frame shift).
Expected rate: < 1/day.  If it fires more often, something is
broken at the correction-source level.

Old behaviour this replaces: `PostFixResidualMonitor`'s L1→L2→L3
ladder.  That design had a level-persistence bug — once cascaded
to L3 it re-fired on every subsequent misfit, losing ~10 min of
convergence per re-fire (see
`project_pfr_event_analysis_20260419.md`: 0/16 L3 events had a
fresh L1 precursor within 10 min).  This alarm is **stateless** —
each eval looks at the current window and decides independently,
no escalation state carried forward.

The alarm is deliberately conservative.  It requires:
  - Elevated RMS sustained over a window (not a single spike)
  - Minimum epoch gap since the last fire (`cooldown_epochs`)
    — so the re-init action has a chance to take effect before
    we re-evaluate
  - No false-fix or setting-SV-drop event in the same window
    (tracked via the tracker's `state_entered_epoch` — if many
    SVs just went to FLOAT, the per-SV monitors are already on it)

Fire action: full filter re-init at `known_ecef`.  Same as old L3.
Fix-set-wide; caller clears the NL resolver, MW tracker, and
re-seeds PPPFilter.  Expected < 1/day in steady state.
"""

from __future__ import annotations

import logging
import math
from collections import deque

from peppar_fix.sv_state import SvAmbState, SvStateTracker

log = logging.getLogger(__name__)


class FixSetIntegrityAlarm:
    """Fix-set-wide PR-RMS alarm, stateless per-eval.

    Two firing conditions:

      1. **Window-RMS** (historical): mean PR residual across fix-set
         members sustained above `rms_threshold_m` for the sampling
         window.  Catches the gross "many members are lying" case.
      2. **Anchor-collapse** (new, 2026-04-21): on a filter that has
         latched `reached_resolved=True`, the long-term anchor count
         drops to zero and stays there for
         `anchor_collapse_epochs`.  Day0421b showed the trap this
         closes — the filter drifts during a hollow-anchor window
         because the per-candidate join test has nothing to anchor
         to.  When both anchors are gone on a trusted-position
         filter, don't try to salvage — tear it down and rebuild
         from bootstrap.  See
         `project_day0421b_anchor_loss_trap_20260421.md`.

    The alarm's event dict carries a `reason` key so callers can
    log the trigger type.

    Usage:

        alarm = FixSetIntegrityAlarm(tracker, ape_sm, ...)
        alarm.ingest(epoch, resid, labels)
        ev = alarm.evaluate(epoch)
        if ev is not None:
            # caller executes the re-init: unfix all NL, reset MW,
            # reseed filter.  alarm.record_fire clears the
            # reached_resolved latch on the AntPosEst state machine.
            alarm.record_fire(epoch)

    `record_fire` is the only state the alarm carries forward — it's
    just the cooldown timestamp plus the latch clear.  No level
    ladder, no "next step" memory.
    """

    def __init__(
        self,
        tracker: SvStateTracker,
        *,
        ape_state_machine=None,
        rms_threshold_m: float = 5.0,
        window_epochs: int = 30,
        min_samples_in_window: int = 10,
        eval_every: int = 10,
        cooldown_epochs: int = 300,
        suppress_if_monitors_fired_within: int = 60,
        anchor_collapse_epochs: int = 60,
    ) -> None:
        self._tracker = tracker
        # Reference to AntPosEst state machine — used to read the
        # reached_resolved latch for the anchor-collapse trigger
        # and to clear it in record_fire.  None disables the
        # anchor-collapse trigger entirely — the window-RMS path
        # still works (legacy callers are unaffected).
        self._ape_sm = ape_state_machine
        self._threshold = float(rms_threshold_m)
        self._min_samples = int(min_samples_in_window)
        self._eval_every = int(eval_every)
        self._cooldown = int(cooldown_epochs)
        self._suppress_window = int(suppress_if_monitors_fired_within)
        self._anchor_collapse_epochs = int(anchor_collapse_epochs)
        self._rms_hist: deque = deque(maxlen=int(window_epochs))
        self._last_fire_epoch: int = -10**9
        # First epoch at which we observed zero long-term anchors on
        # a reached_resolved filter.  None whenever anchors are
        # present.  Reset on every fire.
        self._anchor_collapse_since: int | None = None

    # ── Data intake ─────────────────────────────────────────────── #

    def ingest(self, epoch: int, resid, labels) -> None:
        """Absorb PR residuals across all NL members for this epoch.

        Computes single-epoch RMS across SVs currently in either
        NL_SHORT_FIXED or NL_LONG_FIXED (the fix set).  SVs outside
        the fix set are excluded.
        """
        if resid is None:
            return
        vals: list[float] = []
        nl_states = {SvAmbState.NL_SHORT_FIXED, SvAmbState.NL_LONG_FIXED}
        for lab, r in zip(labels, resid):
            sv, kind = lab[0], lab[1]
            if kind != 'pr':
                continue
            if self._tracker.state(sv) not in nl_states:
                continue
            vals.append(abs(float(r)))
        if vals:
            rms = math.sqrt(sum(v * v for v in vals) / len(vals))
            self._rms_hist.append(rms)

    # ── Evaluation ──────────────────────────────────────────────── #

    def evaluate(self, epoch: int) -> dict | None:
        """Return an alarm event dict, or None if no fire.

        Event dict carries a ``reason`` key so callers can branch on
        trigger type:
          - ``reason='window_rms'`` — traditional PR-residual
            blow-up path (field ``window_rms_m``, ``rms_m``,
            ``n_samples``).
          - ``reason='anchor_collapse'`` — reached_resolved filter
            sat with zero long-term anchors for
            ``anchor_collapse_epochs`` (field ``anchor_collapse_epochs``,
            ``since_epoch``).

        Caller executes re-init and calls `record_fire(epoch)`
        exactly once per event.

        Suppression rules (applied only to the window-RMS path —
        anchor-collapse isn't suppressible, on the grounds that if
        we've already lost every anchor on a trusted-position
        filter, per-SV monitors aren't going to fix it):
          - fewer than `min_samples_in_window` RMS samples
          - window mean RMS ≤ threshold
          - within `cooldown_epochs` of last fire
          - any SV transitioned to FLOAT within
            `suppress_if_monitors_fired_within` epochs (the per-SV
            monitors are already handling it)
        """
        if epoch % self._eval_every != 0:
            return None
        # Cooldown applies to both triggers — the filter just got
        # re-initialized and needs time to settle before re-evaluating.
        if epoch < self._last_fire_epoch + self._cooldown:
            return None

        # ── Anchor-collapse trigger (checked first: cheaper, can
        # pre-empt the window-RMS path when both would fire).  Only
        # active when the AntPosEst state machine reports
        # reached_resolved=True — during bootstrap, the filter hasn't
        # earned a position to defend so zero anchors is expected.
        ap = self._ape_sm
        if ap is not None and getattr(ap, 'reached_resolved', False):
            lt_count = len(self._tracker.long_term_members())
            if lt_count == 0:
                if self._anchor_collapse_since is None:
                    self._anchor_collapse_since = epoch
                elif epoch - self._anchor_collapse_since >= self._anchor_collapse_epochs:
                    return {
                        'reason': 'anchor_collapse',
                        'anchor_collapse_epochs': (
                            epoch - self._anchor_collapse_since),
                        'since_epoch': self._anchor_collapse_since,
                    }
            else:
                # Anchor back — reset the timer.
                self._anchor_collapse_since = None

        # ── Window-RMS trigger (legacy).
        if len(self._rms_hist) < self._min_samples:
            return None
        window_mean = sum(self._rms_hist) / len(self._rms_hist)
        if window_mean <= self._threshold:
            return None

        # Suppress if a per-SV monitor fired recently: look for any
        # SV that transitioned to FLOAT within the suppress window.
        # (Setting-SV drops and false-fix rejections both land in FLOAT.)
        # The per-SV state_entered_epoch holds the last entry.
        suppress_cutoff = epoch - self._suppress_window
        for _sv, rec in self._tracker.all_records():
            if rec.state is SvAmbState.FLOAT:
                if rec.state_entered_epoch >= suppress_cutoff:
                    log.info(
                        "[FIX_SET_ALARM] suppressed: %s in %s since epoch %d"
                        " (per-SV monitor handling; window RMS=%.2fm)",
                        rec.sv, rec.state.value, rec.state_entered_epoch,
                        window_mean,
                    )
                    return None

        latest = self._rms_hist[-1]
        return {
            'reason': 'window_rms',
            'rms_m': latest,
            'window_rms_m': window_mean,
            'n_samples': len(self._rms_hist),
        }

    def record_fire(self, epoch: int) -> None:
        """Caller calls this after executing the re-init.

        Clears the window-RMS history (so the next eval starts
        fresh) and the anchor-collapse timer (re-initialisation
        invalidates both observations).  Clears
        `reached_resolved` on the AntPosEst state machine — the
        filter is back to bootstrap mode and must re-earn that
        latch through promotion, not through mere stabilization.
        """
        self._last_fire_epoch = int(epoch)
        self._rms_hist.clear()
        self._anchor_collapse_since = None
        if self._ape_sm is not None:
            self._ape_sm.clear_reached_resolved(reason="fix_set_alarm")
        log.warning("[FIX_SET_ALARM] fired at epoch %d — filter re-init", epoch)

    # ── Diagnostics ─────────────────────────────────────────────── #

    def summary(self) -> str:
        if not self._rms_hist:
            return "fix_set_alarm: no samples"
        window_mean = sum(self._rms_hist) / len(self._rms_hist)
        return (
            f"fix_set_alarm: window_rms={window_mean:.2f}m"
            f" (last={self._rms_hist[-1]:.2f}m, n={len(self._rms_hist)})"
        )

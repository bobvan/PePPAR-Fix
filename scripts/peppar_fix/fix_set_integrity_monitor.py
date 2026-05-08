"""Fix-set integrity monitor — catches systemic failures the per-SV
monitors can't attribute to one satellite.

Per `docs/sv-lifecycle-and-pfr-split.md`: the design expects
per-SV issues to be handled by the false-fix monitor and the
setting-SV drop monitor.  This fix-set-wide monitor trips only
for the residual case where *many* members misbehave at once
without any single one being the culprit — genuine systemic
failure (bad SSR correction batch, clock-datum change, reference-
frame shift).  Expected trip rate: < 1/day.  If trips happen more
often, something is broken at the correction-source level.

Old behaviour this replaces: `PostFixResidualMonitor`'s L1→L2→L3
ladder.  That design had a level-persistence bug — once cascaded
to L3 it re-fired on every subsequent misfit, losing ~10 min of
convergence per re-fire (see
`project_pfr_event_analysis_20260419.md`: 0/16 L3 events had a
fresh L1 precursor within 10 min).  This monitor is **stateless**
— each eval looks at the current window and decides independently,
no escalation state carried forward.

The monitor is deliberately conservative.  Triggering requires:
  - Elevated RMS sustained over a window (not a single spike)
  - Minimum epoch gap since the last trip (`cooldown_epochs`)
    — so the re-init action has a chance to take effect before
    we re-evaluate
  - No false-fix or setting-SV-drop event in the same window
    (tracked via the tracker's `state_entered_epoch` — if many
    SVs just went to FLOATING, the per-SV monitors are already
    on it)

Trip action: full filter re-init at `known_ecef`.  Same behaviour
as old L3.  Fix-set-wide; caller clears the NL resolver, MW
tracker, and re-seeds PPPFilter.  Expected < 1/day in steady
state.
"""

from __future__ import annotations

import logging
import math
from collections import deque

from peppar_fix.sv_state import SvAmbState, SvStateTracker

log = logging.getLogger(__name__)


class FixSetIntegrityMonitor:
    """Fix-set-wide PR-RMS monitor, stateless per-eval.

    Two trip conditions:

      1. **Window-RMS** (historical): mean PR residual across fix-set
         members sustained above ``rms_threshold_m`` for the sampling
         window.  Catches the gross "many members are lying" case.
      2. **Anchor-collapse** (2026-04-21): on a filter that has
         latched ``reached_anchored=True`` (has ever entered the
         ANCHORED state with ≥ 4 geometry-validated anchors), the
         anchor count drops to zero and stays there for
         ``anchor_collapse_epochs``.  Day0421b showed the trap
         this closes — the filter drifts during a hollow-anchor
         window because the per-candidate join test has nothing
         to anchor to.  When all anchors are gone on a filter
         that once had them, don't try to salvage — tear it down
         and rebuild from bootstrap.  See
         ``project_day0421b_anchor_loss_trap_20260421.md`` and
         ``project_landed_20260421_anchor_collapse_fix.md``
         (858f7da, which originally used a local ``_ever_anchored``
         flag migrated into the ``reached_anchored`` latch in
         Commit (a) of the lifecycle rename).

    The event dict returned by ``evaluate()`` carries a ``reason``
    key so callers can log the trigger type.  Edge-triggered: the
    monitor produces one event at the moment the threshold is
    crossed, then stays silent until ``record_trip`` is called.

    Usage:

        monitor = FixSetIntegrityMonitor(tracker, ape_sm, ...)
        monitor.ingest(epoch, resid, labels)
        ev = monitor.evaluate(epoch)
        if ev is not None:
            # caller executes the re-init: unfix all NL, reset MW,
            # reseed filter, emit the [FIX_SET_INTEGRITY] TRIPPED
            # log line, then tell the monitor it has tripped so
            # both latches clear and the cooldown starts.
            monitor.record_trip(epoch)

    ``record_trip`` is the only state the monitor carries forward
    — just the cooldown timestamp plus the latch clear.  No level
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
        ztd_trip_threshold_m: float = 0.7,
        ztd_sustained_epochs: int = 60,
        ztd_escalate_threshold: int = 2,
        ztd_escalate_window_epochs: int = 1200,
        pos_consensus_threshold_m: float = 0.20,
        pos_consensus_sustained_epochs: int = 30,
        ztd_consensus_threshold_m: float = 0.10,
        ztd_consensus_sustained_epochs: int = 60,
        if_rms_threshold_m: float = 0.05,
    ) -> None:
        self._tracker = tracker
        # Reference to AntPosEst state machine — used to read the
        # reached_anchored latch for the anchor-collapse trigger
        # and to clear both latches in record_trip.  None disables
        # the anchor-collapse trigger entirely — the window-RMS
        # path still works (legacy callers are unaffected).
        self._ape_sm = ape_state_machine
        self._threshold = float(rms_threshold_m)
        self._min_samples = int(min_samples_in_window)
        self._eval_every = int(eval_every)
        self._cooldown = int(cooldown_epochs)
        self._suppress_window = int(suppress_if_monitors_fired_within)
        self._anchor_collapse_epochs = int(anchor_collapse_epochs)
        self._rms_hist: deque = deque(maxlen=int(window_epochs))
        # IF (carrier-phase) RMS history parallel to the PR history
        # above.  Used by the window_rms trigger's IF gate (I-162353,
        # 2026-05-02): only trip on PR-RMS breach when the IF-RMS
        # also breaches.  PR-multipath storms with sub-cm IF
        # residuals are integer-clean; tripping on them produces a
        # full filter re-init with no integer pathology to recover
        # from (8/12 = 67% of TimeHat day0501 trips).  See
        # docs/misnomers.md "Residual-domain mismatch" pattern.
        self._if_rms_hist: deque = deque(maxlen=int(window_epochs))
        self._if_rms_threshold = float(if_rms_threshold_m)
        self._last_trip_epoch: int = -10**9
        # First epoch at which we observed zero long-term anchors on
        # a filter that has ever been ANCHORED.  None whenever
        # anchors are present or the filter has never been
        # anchored.  Reset on every trip.
        self._anchor_collapse_since: int | None = None
        # ZTD-impossibility diagnostic (no trip, no re-init — diagnostic
        # only).  See docs/ztd-impossibility-trigger-design.md for the
        # full proposed trip semantics.  This first cut logs when the
        # trigger *would* fire so we can calibrate the threshold from
        # overnight data before committing to the actual re-init path.
        # Cross-filter divergence check deferred pending shared state
        # between AntPosEstThread and FixedPosFilter.
        self._ztd_trip_threshold_m = float(ztd_trip_threshold_m)
        self._ztd_sustained_epochs = int(ztd_sustained_epochs)
        self._ztd_above_since: int | None = None
        # Trip-cycling escalation: if ZTD trips repeatedly without
        # escape, the NL-only revert is insufficient — likely wrong
        # WL integer(s) in the fix set, trapping every NL search in
        # the wrong integer subspace (NL search is bounded by WL:
        # N1-N5=N_WL, so a wrong N_WL excludes the correct NL
        # integer from the search).  Escalation fires a full WL
        # flush via MW reset for every tracked SV.  ``record_trip``
        # does not clear this history — cycling detection must
        # persist across the cooldown boundary or the trip-counter
        # resets before the next trip arrives.  The history is
        # cleared only when escalation fires (so a fresh count
        # starts toward any *further* escalation).
        self._ztd_escalate_threshold = int(ztd_escalate_threshold)
        self._ztd_escalate_window_epochs = int(ztd_escalate_window_epochs)
        self._ztd_trip_history: list[int] = []
        # Fleet consensus triggers (Part 2 of
        # docs/fleet-consensus-monitors.md).  Fire earlier than the
        # single-host physical-envelope monitors (ztd_impossible at
        # 700 mm absolute vs. ztd_consensus at 100 mm relative to the
        # cohort median) — tens-of-mm vs. hundreds-of-mm scale, so
        # we catch null-mode excursions minutes to tens of minutes
        # earlier on shared-antenna / shared-site cohorts.  Null
        # when the delta is None (no cohort available — either no
        # peers, or single-host run); monitor silently skips those
        # paths and the engine behaves exactly as before.
        self._pos_consensus_threshold_m = float(pos_consensus_threshold_m)
        self._pos_consensus_sustained_epochs = int(
            pos_consensus_sustained_epochs)
        self._pos_consensus_above_since: int | None = None
        self._ztd_consensus_threshold_m = float(ztd_consensus_threshold_m)
        self._ztd_consensus_sustained_epochs = int(
            ztd_consensus_sustained_epochs)
        self._ztd_consensus_above_since: int | None = None

    # ── Data intake ─────────────────────────────────────────────── #

    def ingest(self, epoch: int, resid, labels) -> None:
        """Absorb PR and phi residuals across all NL members for this epoch.

        Computes single-epoch RMS for each kind across SVs currently
        in either ANCHORING or ANCHORED (the fix set).  SVs outside
        the fix set are excluded.  Both histories are kept so the
        window_rms trip can gate on PR breach AND IF (carrier-phase)
        breach — see I-162353 and docs/misnomers.md "Residual-domain
        mismatch" pattern.
        """
        if resid is None:
            return
        pr_vals: list[float] = []
        if_vals: list[float] = []
        nl_states = {SvAmbState.ANCHORING, SvAmbState.ANCHORED}
        for lab, r in zip(labels, resid):
            sv, kind = lab[0], lab[1]
            if kind not in ('pr', 'phi'):
                continue
            if self._tracker.state(sv) not in nl_states:
                continue
            if kind == 'pr':
                pr_vals.append(abs(float(r)))
            else:  # 'phi'
                if_vals.append(abs(float(r)))
        if pr_vals:
            self._rms_hist.append(
                math.sqrt(sum(v * v for v in pr_vals) / len(pr_vals)))
        if if_vals:
            self._if_rms_hist.append(
                math.sqrt(sum(v * v for v in if_vals) / len(if_vals)))

    # ── Evaluation ──────────────────────────────────────────────── #

    def evaluate(self, epoch: int, ztd_m: float | None = None,
                 pos_consensus_delta_m: float | None = None,
                 ztd_consensus_delta_m: float | None = None) -> dict | None:
        """Return a trip event dict, or None if no trip.

        Event dict carries a ``reason`` key so callers can branch on
        trigger type:
          - ``reason='window_rms'`` — traditional PR-residual
            blow-up path (field ``window_rms_m``, ``rms_m``,
            ``n_samples``).  Action: full filter re-init.
          - ``reason='anchor_collapse'`` — filter that has ever
            reached ANCHORED sat with zero long-term anchors for
            ``anchor_collapse_epochs`` (field
            ``anchor_collapse_epochs``, ``since_epoch``).  Action:
            full filter re-init.
          - ``reason='ztd_impossible'`` — filter ZTD residual
            state past the physical envelope for a sustained
            window (field ``ztd_m``, ``threshold_m``,
            ``sustained_epochs``).  **Action: drop NL fixes only,
            preserve WL / MW / position / clock.**  Wrong NL
            integers are the likely driver of ZTD corruption —
            reverting to WL-only stops them from accumulating
            without discarding the fast-to-acquire state.
          - ``reason='ztd_cycling'`` — ZTD trip has fired
            ``ztd_escalate_threshold`` times within
            ``ztd_escalate_window_epochs`` without the filter
            escaping the biased-equilibrium basin.  The NL-only
            revert is clearly insufficient — most likely a wrong
            WL integer in the fix set is trapping every NL
            search in the wrong integer subspace.  **Action:
            full WL flush — reset MW tracker for every SV,
            transition NL-fixed SVs to FLOATING.  Position /
            clock / ISB preserved.**
          - ``reason='pos_consensus'`` — this host's position
            differs from the shared-ARP cohort median by more
            than ``pos_consensus_threshold_m`` for
            ``pos_consensus_sustained_epochs``.  Fires earlier
            (tens of cm) than ``anchor_collapse`` or
            ``ztd_impossible`` (meters), catching null-mode
            drift before it corrupts ZTD.  **Action: full filter
            re-init, same as anchor_collapse.**  Cohort is the
            reference when self disagrees.
          - ``reason='ztd_consensus'`` — this host's ZTD residual
            differs from the shared-atmosphere cohort median by
            more than ``ztd_consensus_threshold_m`` for
            ``ztd_consensus_sustained_epochs``.  Fires earlier
            (10 cm relative) than ``ztd_impossible``
            (700 mm absolute).  **Action: NL-drop + ZTD reset,
            same as ztd_impossible.**

        Pass ``ztd_m`` (the PPP filter's current ZTD residual in
        metres) to enable the ZTD trip.  When ``None``, the ZTD
        check is skipped.

        Pass ``pos_consensus_delta_m`` (self-minus-cohort-median
        3D distance in metres) to enable the position-consensus
        trip.  None (no cohort available, single-host run, etc.)
        silently skips this check.

        Pass ``ztd_consensus_delta_m`` (self-minus-cohort-median
        ZTD residual in metres, signed or absolute — we take the
        absolute value here) to enable the ZTD-consensus trip.
        None silently skips.

        Caller executes the appropriate recovery and calls
        `record_trip(epoch)` exactly once per event.

        Suppression rules (applied only to the window-RMS path —
        anchor-collapse and ZTD aren't suppressible, on the grounds
        that if the filter's own state is corrupt, per-SV monitors
        aren't going to fix it):
          - fewer than `min_samples_in_window` RMS samples
          - window mean RMS ≤ threshold
          - within `cooldown_epochs` of last trip
          - any SV transitioned to FLOATING within
            `suppress_if_monitors_fired_within` epochs (the per-SV
            monitors are already handling it)
        """
        if epoch % self._eval_every != 0:
            return None
        # Cooldown applies to all triggers — the filter just got
        # re-initialized and needs time to settle before re-evaluating.
        if epoch < self._last_trip_epoch + self._cooldown:
            return None

        # ── Anchor-collapse trigger (checked first: cheaper, can
        # pre-empt the window-RMS path when both would trip).  Only
        # active on filters that have ever reached the ANCHORED
        # state (≥ 4 ANCHORED validated).  During bootstrap,
        # CONVERGING, or ANCHORING with zero long-term anchors the
        # filter hasn't earned a position to defend, and triggering
        # here would cycle spuriously (day0421f L5 fleet: 6/8/15
        # trips per host in ~3h, all spurious, pre-rename).
        # ``reached_anchored`` latches only on actual ANCHORED
        # entry — the old ``reached_resolved`` latch fired on the
        # fallback path too and couldn't be used as the gate.
        ap = self._ape_sm
        if ap is not None and getattr(ap, 'reached_anchored', False):
            lt_count = len(self._tracker.anchored_svs())
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

        # ── Fleet-consensus triggers.  Skipped silently when the
        # delta is None (no cohort, single-host run).  Checked
        # ahead of the single-host physical-envelope monitors
        # because they fire at the tens-of-mm scale vs. hundreds-of-
        # mm, so they'd pre-empt anyway if both would fire.
        if pos_consensus_delta_m is not None:
            if pos_consensus_delta_m > self._pos_consensus_threshold_m:
                if self._pos_consensus_above_since is None:
                    self._pos_consensus_above_since = epoch
                elif (epoch - self._pos_consensus_above_since
                        >= self._pos_consensus_sustained_epochs):
                    return {
                        'reason': 'pos_consensus',
                        'delta_m': pos_consensus_delta_m,
                        'threshold_m': self._pos_consensus_threshold_m,
                        'sustained_epochs': (
                            epoch - self._pos_consensus_above_since),
                    }
            else:
                self._pos_consensus_above_since = None

        if ztd_consensus_delta_m is not None:
            delta_abs = abs(ztd_consensus_delta_m)
            if delta_abs > self._ztd_consensus_threshold_m:
                if self._ztd_consensus_above_since is None:
                    self._ztd_consensus_above_since = epoch
                elif (epoch - self._ztd_consensus_above_since
                        >= self._ztd_consensus_sustained_epochs):
                    return {
                        'reason': 'ztd_consensus',
                        'delta_m': ztd_consensus_delta_m,
                        'threshold_m': self._ztd_consensus_threshold_m,
                        'sustained_epochs': (
                            epoch - self._ztd_consensus_above_since),
                    }
            else:
                self._ztd_consensus_above_since = None

        # ── ZTD-impossibility trigger.  No latch dependency — ZTD
        # can drift at any lifecycle state.  Sustained-window
        # suppresses startup transients.  Fires when |ZTD residual|
        # > threshold for ztd_sustained_epochs.  Action (caller-
        # side): drop NL fixes, keep WL / MW / position / clock —
        # wrong NL integers are the likely corruption driver, so
        # reverting to WL only stops accumulation without
        # discarding fast-to-acquire state.  Design doc:
        # docs/ztd-impossibility-trigger-design.md.
        if ztd_m is not None:
            if abs(ztd_m) > self._ztd_trip_threshold_m:
                if self._ztd_above_since is None:
                    self._ztd_above_since = epoch
                elif (epoch - self._ztd_above_since
                        >= self._ztd_sustained_epochs):
                    # Prune trip history to current escalation window.
                    cutoff = epoch - self._ztd_escalate_window_epochs
                    self._ztd_trip_history = [
                        e for e in self._ztd_trip_history if e > cutoff
                    ]
                    # This trip counts toward the cycling count.
                    trips_incl_this = len(self._ztd_trip_history) + 1
                    reason = 'ztd_impossible'
                    if trips_incl_this >= self._ztd_escalate_threshold:
                        # Cycling: NL-only revert is insufficient.
                        # Escalate to full WL flush and clear the
                        # history so the *next* escalation again
                        # needs the full threshold count.
                        reason = 'ztd_cycling'
                        self._ztd_trip_history = []
                    else:
                        self._ztd_trip_history.append(epoch)
                    return {
                        'reason': reason,
                        'ztd_m': ztd_m,
                        'threshold_m': self._ztd_trip_threshold_m,
                        'sustained_epochs':
                            epoch - self._ztd_above_since,
                        'recent_trip_count': trips_incl_this,
                        'escalate_window_epochs':
                            self._ztd_escalate_window_epochs,
                    }
            else:
                self._ztd_above_since = None

        # ── Window-RMS trigger (legacy).
        if len(self._rms_hist) < self._min_samples:
            return None
        window_mean = sum(self._rms_hist) / len(self._rms_hist)
        if window_mean <= self._threshold:
            return None

        # IF-RMS gate (I-162353, 2026-05-02): require the carrier-phase
        # RMS over the same window to also exceed if_rms_threshold_m.
        # PR-multipath storms with sub-cm IF residuals are integer-
        # clean — full filter re-init has no integer pathology to
        # recover from.  Empirically (TimeHat day0501) 8 of 12 trips
        # had IF-RMS in 4-80 mm; only 3 had IF-RMS at 122-226 mm
        # (real integer pathology).  When no IF samples are
        # available (early-arc state), fall through to the legacy
        # PR-only check rather than gating on absent data.
        if len(self._if_rms_hist) >= self._min_samples:
            if_window_mean = sum(self._if_rms_hist) / len(self._if_rms_hist)
            if if_window_mean <= self._if_rms_threshold:
                log.info(
                    "[FIX_SET_INTEGRITY] suppressed: window-RMS PR=%.2fm "
                    "> %.2fm but IF=%.3fm ≤ %.3fm — integer-clean PR-"
                    "multipath storm, no re-init",
                    window_mean, self._threshold,
                    if_window_mean, self._if_rms_threshold,
                )
                return None

        # Suppress if a per-SV monitor fired recently: look for any
        # SV that transitioned to FLOATING within the suppress window.
        # (Setting-SV drops and false-fix rejections both land in FLOATING.)
        # The per-SV state_entered_epoch holds the last entry.
        suppress_cutoff = epoch - self._suppress_window
        for _sv, rec in self._tracker.all_records():
            if rec.state is SvAmbState.FLOATING:
                if rec.state_entered_epoch >= suppress_cutoff:
                    log.info(
                        "[FIX_SET_INTEGRITY] suppressed: %s in %s since epoch %d"
                        " (per-SV monitor handling; window RMS=%.2fm)",
                        rec.sv, rec.state.value, rec.state_entered_epoch,
                        window_mean,
                    )
                    return None

        latest = self._rms_hist[-1]
        if_latest = (self._if_rms_hist[-1]
                     if self._if_rms_hist else None)
        if_window_mean_for_event = (
            sum(self._if_rms_hist) / len(self._if_rms_hist)
            if self._if_rms_hist else None
        )
        return {
            'reason': 'window_rms',
            'rms_m': latest,
            'window_rms_m': window_mean,
            'n_samples': len(self._rms_hist),
            'if_rms_m': if_latest,
            'if_window_rms_m': if_window_mean_for_event,
            'if_n_samples': len(self._if_rms_hist),
        }

    def record_trip(self, epoch: int) -> None:
        """Caller calls this after executing the re-init.

        Clears the window-RMS history (so the next eval starts
        fresh) and the anchor-collapse timer (re-initialisation
        invalidates both observations).  Clears both latches on
        the AntPosEst state machine — the filter is back to
        bootstrap mode and must re-earn both milestones through
        promotion, not through mere stabilization.
        """
        self._last_trip_epoch = int(epoch)
        self._rms_hist.clear()
        self._if_rms_hist.clear()
        self._anchor_collapse_since = None
        self._ztd_above_since = None
        self._pos_consensus_above_since = None
        self._ztd_consensus_above_since = None
        if self._ape_sm is not None:
            self._ape_sm.clear_latches(reason="fix_set_integrity_trip")
        # The trip itself is announced by the caller as a single
        # [FIX_SET_INTEGRITY] TRIPPED line with reason + params.  We
        # don't re-announce here — keeps "monitor only speaks on
        # trip, once" as the interface contract.

    # ── Diagnostics ─────────────────────────────────────────────── #

    def summary(self) -> str:
        if not self._rms_hist:
            return "fix_set_integrity: no samples"
        window_mean = sum(self._rms_hist) / len(self._rms_hist)
        return (
            f"fix_set_integrity: window_rms={window_mean:.2f}m"
            f" (last={self._rms_hist[-1]:.2f}m, n={len(self._rms_hist)})"
        )

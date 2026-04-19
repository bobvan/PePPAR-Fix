"""Host-level PR-RMS alarm — catches systemic failures the per-SV
monitors can't attribute to one satellite.

Per `docs/sv-lifecycle-and-pfr-split.md`: the new design expects per-SV
issues to be handled by Job A (wrong-integer) and Job B (retirement).
The host-level alarm fires only for the residual case where *many*
SVs misbehave at once without any single one being the culprit —
genuine systemic failure (bad SSR correction batch, clock-datum
change, reference-frame shift).  Expected rate: < 1/day.  If it
fires more often, something is broken at the correction-source level.

Old behaviour this replaces: `PostFixResidualMonitor`'s L1→L2→L3
ladder.  That design had a level-persistence bug — once cascaded to
L3 it re-fired on every subsequent misfit, losing ~10 min of
convergence per re-fire (see `project_pfr_event_analysis_20260419.md`:
0/16 L3 events had a fresh L1 precursor within 10 min).  This
monitor is **stateless** — each eval looks at the current window
and decides independently, no escalation state carried forward.

The alarm is deliberately conservative.  It requires:
  - Elevated RMS sustained over a window (not a single spike)
  - Minimum epoch gap since the last fire (`cooldown_epochs`)
    — so the re-init action has a chance to take effect before
    we re-evaluate
  - No Job A / Job B event in the same window (tracked via the
    tracker's `state_entered_epoch` — if many SVs just went to
    FLOAT or RETIRING, Job A/B is already handling it)

Fire action: full filter re-init at `known_ecef`.  Same as old L3.
Host-level; caller clears the NL resolver, MW tracker, and re-seeds
PPPFilter.  Expected < 1/day in steady state.
"""

from __future__ import annotations

import logging
import math
from collections import deque

from peppar_fix.sv_state import SvAmbState, SvStateTracker

log = logging.getLogger(__name__)


class HostRmsAlarm:
    """Host-level PR-RMS alarm, stateless per-eval.

    Usage:

        alarm = HostRmsAlarm(tracker, ...)
        alarm.ingest(epoch, resid, labels)
        ev = alarm.evaluate(epoch)
        if ev is not None:
            # caller executes the re-init: unfix all NL, reset MW, reseed filter
            alarm.record_fire(epoch)

    `record_fire` is the only state the alarm carries forward — it's
    just the cooldown timestamp.  No level ladder, no "next step"
    memory.
    """

    def __init__(
        self,
        tracker: SvStateTracker,
        *,
        rms_threshold_m: float = 5.0,
        window_epochs: int = 30,
        min_samples_in_window: int = 10,
        eval_every: int = 10,
        cooldown_epochs: int = 300,
        suppress_if_jobs_fired_within: int = 60,
    ) -> None:
        self._tracker = tracker
        self._threshold = float(rms_threshold_m)
        self._min_samples = int(min_samples_in_window)
        self._eval_every = int(eval_every)
        self._cooldown = int(cooldown_epochs)
        self._suppress_window = int(suppress_if_jobs_fired_within)
        self._rms_hist: deque = deque(maxlen=int(window_epochs))
        self._last_fire_epoch: int = -10**9

    # ── Data intake ─────────────────────────────────────────────── #

    def ingest(self, epoch: int, resid, labels) -> None:
        """Absorb PR residuals across all NL-fixed SVs for this epoch.

        Computes single-epoch RMS across SVs currently in any NL state
        (PROVISIONAL or VALIDATED).  Retiring SVs are excluded — their
        residuals are expected to be inflated and shouldn't feed the
        systemic-failure signal.
        """
        if resid is None:
            return
        vals: list[float] = []
        nl_states = {SvAmbState.NL_PROVISIONAL, SvAmbState.NL_VALIDATED}
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

        Event dict: ``{'rms_m': float, 'window_rms_m': float,
        'n_samples': int}``.  Caller executes re-init and calls
        `record_fire(epoch)` exactly once.

        Suppression rules (any one silences the alarm):
          - fewer than `min_samples_in_window` RMS samples
          - window mean RMS ≤ threshold
          - within `cooldown_epochs` of last fire
          - any SV transitioned to FLOAT/RETIRING within
            `suppress_if_jobs_fired_within` epochs (Job A/B is already
            on the case)
        """
        if epoch % self._eval_every != 0:
            return None
        if len(self._rms_hist) < self._min_samples:
            return None
        if epoch < self._last_fire_epoch + self._cooldown:
            return None

        window_mean = sum(self._rms_hist) / len(self._rms_hist)
        if window_mean <= self._threshold:
            return None

        # Suppress if Job A or Job B already fired recently: look for any
        # SV that transitioned to FLOAT or RETIRING within the suppress
        # window.  The per-SV state_entered_epoch holds the last entry.
        suppress_cutoff = epoch - self._suppress_window
        for _sv, rec in self._tracker.all_records():
            if rec.state in (SvAmbState.FLOAT, SvAmbState.RETIRING):
                if rec.state_entered_epoch >= suppress_cutoff:
                    log.info(
                        "[HOST_ALARM] suppressed: %s in %s since epoch %d"
                        " (Jobs A/B handling; window RMS=%.2fm)",
                        rec.sv, rec.state.value, rec.state_entered_epoch,
                        window_mean,
                    )
                    return None

        latest = self._rms_hist[-1]
        return {
            'rms_m': latest,
            'window_rms_m': window_mean,
            'n_samples': len(self._rms_hist),
        }

    def record_fire(self, epoch: int) -> None:
        """Caller calls this after executing the re-init."""
        self._last_fire_epoch = int(epoch)
        self._rms_hist.clear()
        log.warning("[HOST_ALARM] fired at epoch %d — filter re-init", epoch)

    # ── Diagnostics ─────────────────────────────────────────────── #

    def summary(self) -> str:
        if not self._rms_hist:
            return "host_alarm: no samples"
        window_mean = sum(self._rms_hist) / len(self._rms_hist)
        return (
            f"host_alarm: window_rms={window_mean:.2f}m"
            f" (last={self._rms_hist[-1]:.2f}m, n={len(self._rms_hist)})"
        )

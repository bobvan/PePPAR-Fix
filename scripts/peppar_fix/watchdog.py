"""Position watchdog — detects antenna movement from PPP residuals.

Two implementations:

  PositionWatchdog        — current/active gate.  Trips on the RMS
                            of the COMBINED (PR + TD-CP) residual
                            vector.  Dominated by PR-domain noise
                            floor; trips on PR-domain disturbances
                            (multipath, code-bias drift, SSR latency)
                            even when CP-domain residuals are sub-cm.

  PositionWatchdogProposed — proposed (I-145915 / charlie's owner).
                            Same baseline-then-3×-or-+0.5m gating,
                            but on TD-CP-row RMS only.  PR_DISTURBANCE
                            logged when PR is high but TD-CP is fine.
                            Both run in parallel during the bake-in
                            period; engine logs both verdicts per
                            epoch so the operator can verify the
                            new gate's behavior on real captures
                            before the active gate is switched.

See docs/time-filter-pr-cp-port.md for design + decomposition.
"""

import numpy as np


class PositionWatchdog:
    """Monitors PPP filter residuals to detect antenna position changes.

    If the antenna moves, pseudorange residuals grow systematically.
    When the implied position shift exceeds threshold, stops servo steering.
    """

    def __init__(self, threshold_m=10.0, window=30, alarm_count=10):
        self.threshold_m = threshold_m
        self.window = window
        self.alarm_count = alarm_count
        self._residuals = []
        self._baseline_rms = None
        self._bad_count = 0
        self._alarmed = False

    def update(self, residuals_rms, n_used):
        """Feed one epoch's residual RMS. Returns True if position is OK."""
        if n_used < 4:
            return True

        if self._baseline_rms is None:
            self._residuals.append(residuals_rms)
            if len(self._residuals) >= self.window:
                self._baseline_rms = float(np.median(self._residuals))
                self._residuals.clear()
            return True

        limit = max(self._baseline_rms * 3.0,
                    self._baseline_rms + self.threshold_m)

        if residuals_rms > limit:
            self._bad_count += 1
            if self._bad_count >= self.alarm_count and not self._alarmed:
                self._alarmed = True
            return not self._alarmed
        else:
            self._bad_count = 0
            return True

    def reset(self):
        """Reset the watchdog after a filter re-seed.

        Clears the alarm state, the bad-epoch counter, and the baseline
        RMS so the watchdog re-learns a new baseline from the re-seeded
        filter's residuals.
        """
        self._alarmed = False
        self._bad_count = 0
        self._baseline_rms = None
        self._residuals = []

    @property
    def alarmed(self):
        return self._alarmed


class PositionWatchdogProposed:
    """Shadow watchdog that gates on TD-CP-row RMS only.

    Reframed 2026-05-05: this is a "trust the position right now?"
    detector, not an "antenna moved permanently" classifier.  Trips
    on any sustained TD-CP-domain disturbance (real ARP move, RF
    loss, sustained multipath) so the engine can mute PPS during
    the disturbance.  The cause is a diagnostic, not a gating
    concern.

    Two semantic changes from the original draft:

    1.  Floor raised 50 mm → 500 mm in max(baseline×3, baseline +
        threshold_m).  A clean F10T baseline of ~3 mm gave a 53 mm
        gate floor that tripped on a 60 s reacquire transient
        (clkPoC3 2026-05-05).  Bumping the floor to 500 mm lets
        normal cycle-slip cleanup pass; sustained TD-CP residuals
        beyond half a meter are real disturbances either way.
        One bump, no further tuning — even with a higher floor,
        a 30 s outage might trip while a 90 s one might not, and
        that's fine: the gate is correct to mute PPS during *any*
        sustained disturbance.

    2.  Auto-clear: once `alarm_count` consecutive over-threshold
        epochs trip the latch, `release_count` consecutive
        under-threshold epochs clear it.  Without auto-clear the
        gate latched permanently after a brief reacquire on
        2026-05-05 even though residuals recovered to baseline
        within seconds.  The gate is now event-driven, not
        permanent.  reset() still exists for explicit re-init.

    The companion PR_DISTURBANCE shadow (PR-row RMS over its own
    baseline gate) is unchanged — still informational, still
    captures cases where PR explodes but TD-CP is fine.
    """

    def __init__(self, threshold_m=0.50, window=30, alarm_count=10,
                 release_count=30, pr_threshold_m=0.5):
        self.threshold_m = threshold_m
        self.pr_threshold_m = pr_threshold_m
        self.window = window
        self.alarm_count = alarm_count
        self.release_count = release_count
        # TD-CP-domain trip state.
        self._td_residuals = []
        self._td_baseline_rms = None
        self._td_bad_count = 0
        self._td_below_count = 0
        self._alarmed = False
        # PR-domain shadow (for PR_DISTURBANCE logging only).
        self._pr_residuals = []
        self._pr_baseline_rms = None

    def update(self, resid_pr_rms, resid_td_rms, n_used):
        """Feed one epoch's split residual RMS.  Returns
        (td_ok, pr_disturbance) where td_ok is the gate verdict
        (True iff position is OK by TD-CP-only criterion) and
        pr_disturbance is True iff PR-row RMS would have tripped
        the original combined-residual gate at this epoch.

        n_used is the COMBINED count; the TD-only baseline still
        wants 4+ measurements per epoch to be meaningful.
        """
        if n_used < 4:
            return True, False

        # TD-CP baseline learn-in.
        if self._td_baseline_rms is None:
            self._td_residuals.append(resid_td_rms)
            if len(self._td_residuals) >= self.window:
                self._td_baseline_rms = float(np.median(self._td_residuals))
                self._td_residuals.clear()
        # PR shadow baseline learn-in (independent).
        if self._pr_baseline_rms is None:
            self._pr_residuals.append(resid_pr_rms)
            if len(self._pr_residuals) >= self.window:
                self._pr_baseline_rms = float(np.median(self._pr_residuals))
                self._pr_residuals.clear()

        # PR_DISTURBANCE verdict: PR-RMS exceeds its own baseline gate
        # by the same shape as the original combined gate.
        pr_disturbance = False
        if self._pr_baseline_rms is not None:
            pr_limit = max(self._pr_baseline_rms * 3.0,
                           self._pr_baseline_rms + self.pr_threshold_m)
            pr_disturbance = (resid_pr_rms > pr_limit)

        # TD-CP gate (the actual trip).
        if self._td_baseline_rms is None:
            return True, pr_disturbance
        td_limit = max(self._td_baseline_rms * 3.0,
                       self._td_baseline_rms + self.threshold_m)
        if resid_td_rms > td_limit:
            self._td_bad_count += 1
            self._td_below_count = 0
            if (self._td_bad_count >= self.alarm_count
                    and not self._alarmed):
                self._alarmed = True
            return (not self._alarmed), pr_disturbance
        # Residual back below threshold: count toward auto-clear.
        self._td_bad_count = 0
        self._td_below_count += 1
        if self._alarmed and self._td_below_count >= self.release_count:
            self._alarmed = False
        return (not self._alarmed), pr_disturbance

    def reset(self):
        """Reset both TD-CP and PR shadow baselines."""
        self._alarmed = False
        self._td_bad_count = 0
        self._td_below_count = 0
        self._td_baseline_rms = None
        self._td_residuals = []
        self._pr_baseline_rms = None
        self._pr_residuals = []

    @property
    def alarmed(self):
        return self._alarmed

"""Position watchdog — detects antenna movement from filter residuals.

Tripping signal: TD-CP-domain (carrier-phase, sub-cm noise floor)
post-fit residual RMS.  PR-domain residuals (pseudorange, m-scale
floor) are NOT used for tripping — they're noisy enough to cause
false alarms on multipath / SSR latency / code-bias drift while the
carrier-phase side stays sub-cm healthy.  See I-145915 /
docs/time-filter-pr-cp-port.md for the design history.

PR-domain disturbances are still surfaced via PR_DISTURBANCE log
warnings in the engine when resid_pr_rms exceeds
``pr_disturbance_threshold_m`` and dominates resid_td_rms — that
preserves postmortem visibility for the multipath / atmospheric
events without re-seeding the filter on PR-domain noise.

Constructor defaults reflect the post-port semantics:

  threshold_td_m = 0.05    # 5 cm above baseline → 10× sub-cm typical
  alarm_count    = 10      # 10 consecutive epochs above limit
  window         = 30      # 30 epochs to learn baseline TD-CP RMS

The PR-only ``threshold_m`` keyword is preserved for back-compat with
existing tests + callers; if both are passed, ``threshold_td_m``
takes precedence.  threshold_m's old default of 0.5 m was set when
the watchdog ran on mixed PR+TD-CP RMS; on TD-CP-only RMS that's
10× too loose, so the new default is 0.05 m.
"""

import numpy as np


class PositionWatchdog:
    """Monitors filter residuals to detect antenna position changes.

    If the antenna moves, time-differenced carrier-phase residuals
    grow systematically (range to satellite no longer matches the
    expected geometry).  PR residuals also grow, but they have a
    m-scale baseline noise floor so we don't trip on them.

    When the carrier-phase residual RMS sustains above
    ``max(baseline×3, baseline + threshold_td_m)`` for
    ``alarm_count`` consecutive epochs, the watchdog alarms.  The
    engine then either re-seeds the filter (NAV2 says antenna is
    stable) or surfaces an antenna-moved alarm (NAV2 disagrees).
    """

    def __init__(self, threshold_td_m=0.05, window=30, alarm_count=10,
                 pr_disturbance_threshold_m=2.0,
                 threshold_m=None):
        # Back-compat: if a caller passed threshold_m (the pre-port
        # mixed-RMS keyword), interpret it as threshold_td_m.  Don't
        # silently scale — the unit is the same (metres of residual
        # RMS), it's just that on TD-CP-only the floor is sub-cm so
        # the same number works much tighter.
        if threshold_m is not None:
            threshold_td_m = threshold_m
        self.threshold_td_m = threshold_td_m
        # Engine-side diagnostic: when resid_pr_rms exceeds this AND
        # dominates resid_td_rms, log [PR_DISTURBANCE] for postmortem
        # visibility.  Not used inside the watchdog itself.
        self.pr_disturbance_threshold_m = pr_disturbance_threshold_m
        self.window = window
        self.alarm_count = alarm_count
        self._residuals = []
        self._baseline_rms = None
        self._bad_count = 0
        self._alarmed = False

    # Back-compat alias — some tests / callers read `threshold_m`.
    @property
    def threshold_m(self):
        return self.threshold_td_m

    @threshold_m.setter
    def threshold_m(self, value):
        self.threshold_td_m = value

    def update(self, residuals_rms, n_used):
        """Feed one epoch's TD-CP-domain residual RMS.

        Returns True when position is healthy (under limit) or while
        the baseline is still being learned.  Returns False once the
        watchdog has alarmed (sustained over-limit residuals).

        ``residuals_rms`` is the post-fit RMS of TD-CP rows only —
        the engine computes it by sqrt(mean(resid_td ** 2)) on
        ``filt.update()``'s 4-tuple return.
        """
        if n_used < 4:
            return True

        if self._baseline_rms is None:
            self._residuals.append(residuals_rms)
            if len(self._residuals) >= self.window:
                self._baseline_rms = float(np.median(self._residuals))
                self._residuals.clear()
            return True

        limit = max(self._baseline_rms * 3.0,
                    self._baseline_rms + self.threshold_td_m)

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

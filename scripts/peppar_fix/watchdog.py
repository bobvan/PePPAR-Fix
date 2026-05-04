"""Position watchdog — detects antenna movement from PPP residuals."""

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

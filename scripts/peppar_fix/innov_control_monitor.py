"""Innovation-vs-control consistency monitor for the servo EKFs.

Detects when the filter's plant model disagrees with reality.  The
Kalman / EKF servo predicts ``x_pred = F·x + B·u`` using a plant-model
gain ``B`` that assumes "1 ppb commanded == 1 ppb delivered."  Any
error in that gain (DAC scaling miscalibration, sign flip, drifted
plant, mis-tuned R) leaves a systematic bias in the innovation
proportional to the control input — visible epoch-by-epoch, orders of
magnitude faster than waiting for actuator-near-rail to manifest as a
late-stage integral-windup symptom.

Three rolling-window health metrics, computed every ``evaluate()`` call:

  corr(u, innov)        — Pearson correlation between control input and
                          innovation.  Healthy ≈ 0 (control fully
                          captured by B·u prediction).  Sustained
                          ``|.| > corr_alarm`` flags plant-model error.
  mean(innov / √S)      — Normalized innovation bias.  Healthy ≈ 0.
                          Nonzero mean flags systematic prediction
                          error (could be plant model OR un-modeled
                          disturbance).
  NIS = mean(innov² / S) — Normalized innovation squared.  Healthy ≈ 1
                          for a well-tuned filter.  Inflation flags
                          R underset or prediction worse than expected.

The monitor is a pure observer.  No behavior change in the servo loop.
Operators consume the per-epoch log line (control + innovation + S)
and the per-window health report.  Tuning the alarm thresholds is
empirical — start permissive, tighten with field data.

References: ``docs/two-site-sync-budget.md`` (why plant-model error
eats the per-clock budget); dayplan item ``innovControlMonitor-main``.
"""

from __future__ import annotations

import collections
import math
from dataclasses import dataclass


@dataclass
class HealthReport:
    """Snapshot of the monitor's current state."""

    epochs:        int      # Samples currently in the window.
    corr_u_innov:  float    # Pearson r between u and innov over the window.
    norm_bias:     float    # mean(innov / sqrt(S)).
    nis:           float    # mean(innov^2 / S).
    status:        str      # "OK" | "WARM" | "ALARM_corr"
                            #     | "ALARM_bias" | "ALARM_nis"
    consec_bad:    int      # Consecutive evaluations out of band.


class InnovControlMonitor:
    """Rolling-window servo-filter health monitor.

    Stateless w.r.t. the servo: feed it ``observe(u, innov, S)`` per
    epoch from wherever the servo's measurement-update step runs.
    Periodically call ``evaluate()`` (e.g. every 60 s or every 100
    epochs) to get a ``HealthReport``.

    Args:
        window: Sample count for the rolling window.  Default 300
            (5 min @ 1 Hz; 30 s @ 10 Hz).
        corr_alarm: ``|corr_u_innov| > corr_alarm`` is the bound.
            Default 0.30.
        bias_alarm: ``|norm_bias| > bias_alarm`` is the bound.
            Default 0.50.
        nis_alarm: ``nis > nis_alarm`` is the bound.  NIS is naturally
            ≥ 0; the alarm catches inflation, not deflation.
            Default 2.5 (well-tuned filter sits near 1.0).
        debounce: Number of consecutive ``evaluate()`` calls a metric
            must remain out-of-band before the alarm state engages.
            Default 2 (suppresses single-window noise).
        warm_min: Minimum samples before any metric is computed.
            Below this, status stays "WARM".  Default ``window // 2``.
    """

    def __init__(self, *,
                 window:       int = 300,
                 corr_alarm:   float = 0.30,
                 bias_alarm:   float = 0.50,
                 nis_alarm:    float = 2.5,
                 debounce:     int = 2,
                 warm_min:     int | None = None):
        if window < 10:
            raise ValueError("window must be ≥ 10")
        self._window = window
        self._corr_alarm = float(corr_alarm)
        self._bias_alarm = float(bias_alarm)
        self._nis_alarm = float(nis_alarm)
        self._debounce = int(debounce)
        self._warm_min = warm_min if warm_min is not None else window // 2

        self._u:     collections.deque = collections.deque(maxlen=window)
        self._innov: collections.deque = collections.deque(maxlen=window)
        self._S:     collections.deque = collections.deque(maxlen=window)
        self._consec_bad: int = 0

    def observe(self, u: float, innov: float, S: float) -> None:
        """Record one epoch's (control input, innovation, S = HPHᵀ+R).

        Silently drops the epoch when ``S <= 0`` (a degenerate filter
        state that shouldn't reach here in practice).
        """
        if not (S > 0 and math.isfinite(S)
                and math.isfinite(u) and math.isfinite(innov)):
            return
        self._u.append(float(u))
        self._innov.append(float(innov))
        self._S.append(float(S))

    def evaluate(self) -> HealthReport:
        """Compute current health metrics and update the alarm latch."""
        n = len(self._innov)
        if n < self._warm_min:
            return HealthReport(
                epochs=n, corr_u_innov=0.0, norm_bias=0.0, nis=1.0,
                status="WARM", consec_bad=self._consec_bad,
            )

        u = list(self._u)
        innov = list(self._innov)
        S = list(self._S)

        # Pearson correlation between control input and innovation.
        # Use the numerically stable two-pass form.
        mu_u = sum(u) / n
        mu_i = sum(innov) / n
        var_u = sum((x - mu_u) ** 2 for x in u)
        var_i = sum((x - mu_i) ** 2 for x in innov)
        if var_u == 0.0 or var_i == 0.0:
            # No variation in u (filter idle?) or no innovation
            # variation — correlation undefined; treat as 0 for the
            # alarm gate.
            corr = 0.0
        else:
            cov_ui = sum((u[k] - mu_u) * (innov[k] - mu_i) for k in range(n))
            corr = cov_ui / math.sqrt(var_u * var_i)

        # Normalized innovation bias and NIS.
        norm_sum = 0.0
        nis_sum = 0.0
        for k in range(n):
            root_s = math.sqrt(S[k])
            norm_sum += innov[k] / root_s
            nis_sum += innov[k] * innov[k] / S[k]
        norm_bias = norm_sum / n
        nis = nis_sum / n

        # Decide alarm state.  Order matters for status label
        # specificity — pick the most actionable.
        triggers: list[str] = []
        if abs(corr) > self._corr_alarm:
            triggers.append("ALARM_corr")
        if abs(norm_bias) > self._bias_alarm:
            triggers.append("ALARM_bias")
        if nis > self._nis_alarm:
            triggers.append("ALARM_nis")

        if triggers:
            self._consec_bad += 1
        else:
            self._consec_bad = 0

        if self._consec_bad >= self._debounce:
            status = triggers[0]
        else:
            status = "OK"

        return HealthReport(
            epochs=n, corr_u_innov=corr,
            norm_bias=norm_bias, nis=nis,
            status=status, consec_bad=self._consec_bad,
        )

    def reset(self) -> None:
        """Clear all window state.  Use on warm-start / engine relaunch."""
        self._u.clear()
        self._innov.clear()
        self._S.clear()
        self._consec_bad = 0

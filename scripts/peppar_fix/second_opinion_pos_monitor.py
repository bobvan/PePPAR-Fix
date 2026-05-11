"""SecondOpinionPosMonitor — independent-witness check on PPP-AR position.

Per dayplan I-011533-main / 2026-04-28 evening: the existing per-epoch
``_check_nav2`` watchdog in the engine is **horizontal-only by design**
(altitude-based RESETs cascaded on clkPoC3 in earlier work, destroying
convergence).  That deliberate narrowing leaves a gap: an internally-
consistent wrong-integer commitment that absorbs error into altitude
flies under every existing trigger.  Tonight (2026-04-28) the fleet
demonstrated the gap — three hosts at sub-cm σ, ZTD residuals of +2 m,
altitudes off by 3 / 9 / 14 m, none of the FixSetIntegrityAlarm or
horizontal-only NAV2 watchdog triggers caught it.

This monitor watches the **3D** displacement between the engine's
PPP-AR position and NAV2's independent solution.  NAV2 doesn't share
PPP-AR's wrong-integer failure modes, so a sustained 3D nav2Δ above
threshold is a strong signal that the filter has locked into a wrong
basin.

Naming: kept distinct from FixSetIntegrityAlarm (which checks internal
consistency — residual RMS, ZTD bounds, anchor count, peer-cohort).
"Second opinion" captures the independent-witness semantics — not
"absolute" (NAV2 is not absolute truth, just independent).

The reaction is a full re-init at NAV2's LLA — drop NL fixes, reset MW,
reseed lat/lon/alt all together.  This is what the horizontal-only
``_check_nav2`` does too; the only difference is what we're willing to
let trip the gate.  With the GF_STEP / IF_STEP eviction monitors now
catching most spurious wrong-integer commits, the historical concern
(altitude-based reset cascade) is mitigated: the cohort-median demoters
won't allow a fresh basin to lock onto the same wrong altitude.

Quality gate (added per dayplan I-115539-main workstream C1):
NAV2's own hAcc estimate is included as an admissibility check.  When
hAcc is high (NAV2 itself is uncertain — multipath transient, brief
geometry change, sat just acquired), nav2Δ is unreliable as evidence
of a wrong PPP-AR lock.  The gate uses the rolling-mean hAcc over
the same window as the streak so a single quiet epoch doesn't unfairly
trip; symmetrically, a single noisy epoch doesn't unfairly hold off
once NAV2 has settled.  Default 1.5 m hAcc threshold matches the
typical F9T NAV2 hAcc on a clean RF in stable geometry; tighter will
hold off when geometry transitions briefly inflate hAcc, looser
admits noisier opinions as evidence.
"""

from __future__ import annotations

import logging
from collections import deque

import numpy as np

log = logging.getLogger(__name__)


def select_reset_target(
    pin_position: bool,
    pin_ecef,
    nav2_ecef,
):
    """Pick the ECEF to reset the filter to when SECOND_OPINION_POS trips.

    Under ``--pin-position`` with a surveyed ``--known-pos`` the pin IS
    truth.  NAV2 has documented biases (~4 m east on shared-antenna F9Ts
    per ``project_nav2_systematic_bias.md``; ~3-4 m vertical low on
    CHOKE1 per ``wrongIntBasin-charlie`` 2026-05-11 forensics) and is the
    wrong target to snap back to.  The monitor's trip still catches the
    "filter has drifted from NAV2" signal — we just snap to the right
    reference.  Symmetric with the engine's existing position-blend skip
    under ``--pin-position`` (engine.py around the blend site).

    Returns ``(reset_ecef, label)`` where label is ``"known_pos"`` when
    the pin was used, else ``"NAV2"``.
    """
    if pin_position and pin_ecef is not None:
        return np.asarray(pin_ecef, dtype=float), "known_pos"
    return np.asarray(nav2_ecef, dtype=float), "NAV2"


class SecondOpinionPosMonitor:
    """Per-host check: 3D PPP-AR position vs. NAV2 opinion.

    Usage::

        m = SecondOpinionPosMonitor()
        # When a fresh NAV2 opinion is available:
        ev = m.evaluate(epoch=N, nav2_delta_3d_m=disp_3d)
        if ev:
            # caller does the full re-init at NAV2 LLA

    ``evaluate`` returns ``None`` when the gate is below threshold or
    not yet sustained.  When the streak hits ``sustained_epochs`` and
    the disagreement is sustained, returns an event dict::

        {
          'reason': 'second_opinion_3d',
          'nav2_delta_3d_m': float,
          'threshold_m': float,
          'sustained_epochs': int,
        }

    After a caller acts on an event, call ``note_recovery()`` (or wait
    for the next nav2Δ to drop below threshold — auto-resets).
    """

    def __init__(
        self,
        threshold_m: float = 5.0,
        sustained_epochs: int = 30,
        hacc_threshold_m: float | None = 1.5,
        hacc_window_epochs: int | None = None,
    ) -> None:
        # 5 m on 3D displacement.  The known NAV2 east-bias on shared
        # antenna is ~4 m horizontal; 5 m on 3D allows that as noise
        # while still catching the kind of altitude-trapped lock we
        # observed on 2026-04-28 (3-14 m vertical errors).
        self._threshold = float(threshold_m)
        # 30 epochs ≈ 30 s at 1 Hz.  Long enough to suppress NAV2
        # short-term transients (multipath flicker, momentary geometry
        # change), short enough that an actual wrong-basin lock fires
        # before ZTD is too corrupt to recover from.
        self._sustained = int(sustained_epochs)
        # NAV2 hAcc quality gate.  None disables the gate (back-compat
        # for callers not passing nav2_h_acc_m).  1.5 m is the typical
        # NAV2 hAcc floor on a clean F9T NAV2 fix in stable geometry;
        # transient excursions above this often track real geometry
        # changes where nav2Δ is itself uncertain.  Held over a rolling
        # mean rather than per-epoch so a single noisy epoch doesn't
        # unfairly hold off once NAV2 has settled.
        self._hacc_threshold = (
            float(hacc_threshold_m) if hacc_threshold_m is not None else None)
        # Default rolling window matches the streak window so the gate
        # smooths over the same horizon the trip is being decided on.
        self._hacc_window = int(
            hacc_window_epochs if hacc_window_epochs is not None
            else sustained_epochs)
        self._hacc_history: deque[float] = deque(maxlen=self._hacc_window)
        self._streak = 0
        # Latch — only fire once per period of sustained disagreement.
        # Caller's reset action will clear by calling note_recovery,
        # or we auto-clear when the gate drops below threshold.
        self._fired_this_period = False
        self._last_event_epoch: int | None = None

    def evaluate(
        self,
        epoch: int,
        nav2_delta_3d_m: float | None,
        nav2_h_acc_m: float | None = None,
    ) -> dict | None:
        if nav2_delta_3d_m is None:
            # No NAV2 opinion this epoch — don't reset the streak,
            # NAV2 store typically has 30 s freshness window.
            return None
        # Track NAV2 hAcc whenever provided, even when below threshold —
        # the rolling-mean window needs continuous samples to be
        # meaningful when a streak does start.
        if nav2_h_acc_m is not None:
            self._hacc_history.append(float(nav2_h_acc_m))
        if abs(nav2_delta_3d_m) <= self._threshold:
            if self._streak > 0:
                log.info(
                    "[SECOND_OPINION_POS] gate cleared: nav2Δ=%.2fm ≤ %.2fm "
                    "(was streak=%d/%d)",
                    nav2_delta_3d_m, self._threshold,
                    self._streak, self._sustained,
                )
            self._streak = 0
            self._fired_this_period = False
            return None
        self._streak += 1
        if self._streak < self._sustained:
            return None
        if self._fired_this_period:
            return None
        # Quality gate: rolling-mean NAV2 hAcc must be tight before we
        # trust nav2Δ as wrong-PPP-AR-lock evidence.  When the gate is
        # disabled (hacc_threshold None) or no hAcc samples have ever
        # arrived (back-compat for callers not passing nav2_h_acc_m),
        # skip the check and fire as before.
        rolling_hacc = self._rolling_hacc()
        if (self._hacc_threshold is not None
                and rolling_hacc is not None
                and rolling_hacc > self._hacc_threshold):
            # Hold off — keep the streak so the moment NAV2 settles
            # and the rolling mean drops below threshold, the next
            # evaluate call fires.
            log.info(
                "[SECOND_OPINION_POS] hacc gate hold: nav2Δ=%.2fm > %.2fm "
                "streak=%d/%d but rolling-hAcc=%.2fm > %.2fm",
                nav2_delta_3d_m, self._threshold,
                self._streak, self._sustained,
                rolling_hacc, self._hacc_threshold,
            )
            return None
        self._fired_this_period = True
        self._last_event_epoch = epoch
        return {
            'reason': 'second_opinion_3d',
            'nav2_delta_3d_m': float(nav2_delta_3d_m),
            'threshold_m': self._threshold,
            'sustained_epochs': self._streak,
            'rolling_hacc_m': rolling_hacc,
        }

    def _rolling_hacc(self) -> float | None:
        """Mean of the recent NAV2 hAcc samples, or None if no samples
        have arrived (gate is off / disabled / pre-warm-up)."""
        if not self._hacc_history:
            return None
        return sum(self._hacc_history) / len(self._hacc_history)

    def note_recovery(self) -> None:
        """Caller signals after re-init so we can fire again on next divergence."""
        self._streak = 0
        self._fired_this_period = False
        # Drop hAcc history too — a fresh re-init starts the gate
        # window fresh; otherwise a stale-but-still-poor pre-init
        # window would prevent the next fire.
        self._hacc_history.clear()

    # ── Diagnostics ─────────────────────────────────────────────── #

    def streak(self) -> int:
        return self._streak

    def is_armed(self) -> bool:
        """True if currently above threshold but not yet at sustained count."""
        return self._streak > 0 and not self._fired_this_period

    def summary(self) -> str:
        rh = self._rolling_hacc()
        rh_str = f"{rh:.2f}m" if rh is not None else "n/a"
        gate = (
            f"≤{self._hacc_threshold:.1f}m"
            if self._hacc_threshold is not None else "off"
        )
        return (
            f"second_opinion_pos: threshold=±{self._threshold:.1f}m "
            f"sustained={self._sustained}ep streak={self._streak} "
            f"fired_this_period={self._fired_this_period} "
            f"rolling_hAcc={rh_str} gate={gate}"
        )

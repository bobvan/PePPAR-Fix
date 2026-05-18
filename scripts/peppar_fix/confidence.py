"""Phase / frequency / total confidence reporting and clockClass mapping.

GPS time has two independently-recovered components:

  - **Phase**: how well we know *absolute* GPS time.  Dominated by
    antenna-position uncertainty.  c ≈ 30 cm/ns, so 1 m of position
    σ contributes ≈ 3.33 ns of phase σ in the worst case (single
    SV; typical sky coverage with diverse satellites does better
    via GDOP, but we keep the conservative bound).
  - **Frequency**: how well we know the *rate* of GPS time.
    Dominated by servo tracking residual against PPS measurement.

This module:

  - Picks the best available σ_position source for phase confidence
    (AntPosEst running-mean → per-epoch → seed → NAV2 hAcc).
  - Wraps σ from the FixedPosFilter clock state for frequency
    confidence.
  - RSS-combines into a total σ.
  - Maps total σ to a target ptp4l clockClass via a stateful
    promoter with separate rising/falling thresholds (hysteresis
    dead bands prevent BMC flapping near boundaries).

Promotion gates are documented in
``docs/position-state-and-monitoring.md`` and
``docs/ptp4l-supervision.md``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

# c ≈ 30 cm/ns, so 1 m of σ_position is worst-case 1/0.30 ≈ 3.33 ns
# of σ_phase contribution.  This is conservative — true geometric
# dilution for the time component depends on SV constellation and is
# typically smaller, but the conservative bound is simpler to defend
# and "honest worst case" is the right framing for clockClass.
SIGMA_POS_NS_PER_M = 1.0 / 0.30

# Hysteresis thresholds (ns) — see docs/position-state-and-monitoring.md
# for the full table.  Asymmetric to avoid clockClass flapping when
# σ wobbles near the band edges.
LOCKED_PROMOTE_NS = 20.0      # rise to class 6 when σ_total < this
LOCKED_DEMOTE_NS = 30.0       # fall from 6 when σ_total > this
INITIALIZED_PROMOTE_NS = 800.0   # rise to class 52 when σ_total < this
INITIALIZED_DEMOTE_NS = 1200.0   # fall from 52 to 248 when σ_total > this


# ── Data classes (immutable snapshots) ─────────────────────────────── #


@dataclass(frozen=True)
class PhaseConfidence:
    sigma_ns: float
    pos_sigma_m: float
    source: str  # "antpos_mean" | "antpos_epoch" | "seed" | "nav2_hAcc" | "unknown"


@dataclass(frozen=True)
class FrequencyConfidence:
    sigma_ns: float
    dt_rx_sigma_ns: float
    scheduler_settled: bool


@dataclass(frozen=True)
class TotalConfidence:
    sigma_ns: float
    phase: PhaseConfidence
    freq: FrequencyConfidence


# ── Compute helpers ───────────────────────────────────────────────── #


def _unknown_phase() -> PhaseConfidence:
    return PhaseConfidence(
        sigma_ns=float("inf"),
        pos_sigma_m=float("inf"),
        source="unknown",
    )


def compute_phase_confidence(
    *,
    antpos_state: Optional[dict] = None,
    seed_sigma_m: Optional[float] = None,
    nav2_h_acc_m: Optional[float] = None,
) -> PhaseConfidence:
    """Pick the best-available σ_position source and convert to ns.

    Priority order (best → worst):
      1. AntPosEst running-mean σ (when the watchdog is armed):
         tightens to ~sub-cm at full window.
      2. AntPosEst per-epoch σ: meters-class cold-start, drops as
         the PPP filter converges.
      3. Seed σ from the .ppp.toml / .survey.toml chosen at startup.
      4. NAV2 hAcc — last resort, ~1–2 m.
      5. Unknown (cold-start gap before NAV2 first fix) → infinite σ.

    The conservative geometry factor is applied uniformly: 1 m of
    σ_pos → SIGMA_POS_NS_PER_M ≈ 3.33 ns of σ_phase.
    """
    if antpos_state is not None and antpos_state.get("active"):
        sm = antpos_state.get("sigma_mean_m")
        if sm is not None and sm > 0:
            return PhaseConfidence(
                sigma_ns=float(sm) * SIGMA_POS_NS_PER_M,
                pos_sigma_m=float(sm),
                source="antpos_mean",
            )
    if antpos_state is not None:
        se = antpos_state.get("sigma_3d_m")
        if se is not None and se > 0:
            return PhaseConfidence(
                sigma_ns=float(se) * SIGMA_POS_NS_PER_M,
                pos_sigma_m=float(se),
                source="antpos_epoch",
            )
    if seed_sigma_m is not None and seed_sigma_m > 0:
        return PhaseConfidence(
            sigma_ns=float(seed_sigma_m) * SIGMA_POS_NS_PER_M,
            pos_sigma_m=float(seed_sigma_m),
            source="seed",
        )
    if nav2_h_acc_m is not None and nav2_h_acc_m > 0:
        return PhaseConfidence(
            sigma_ns=float(nav2_h_acc_m) * SIGMA_POS_NS_PER_M,
            pos_sigma_m=float(nav2_h_acc_m),
            source="nav2_hAcc",
        )
    return _unknown_phase()


def compute_frequency_confidence(
    *,
    dt_rx_sigma_ns: Optional[float] = None,
    scheduler_settled: bool = False,
) -> FrequencyConfidence:
    """Wrap FixedPosFilter's dt_rx sigma + scheduler-settled flag.

    Future refinement: include recent servo tracking-error stdev as
    a second component (currently the dt_rx sigma alone underweights
    short-term servo noise).  Out of scope for slice 9.
    """
    if dt_rx_sigma_ns is None or dt_rx_sigma_ns < 0:
        return FrequencyConfidence(
            sigma_ns=float("inf"),
            dt_rx_sigma_ns=float("inf"),
            scheduler_settled=scheduler_settled,
        )
    return FrequencyConfidence(
        sigma_ns=float(dt_rx_sigma_ns),
        dt_rx_sigma_ns=float(dt_rx_sigma_ns),
        scheduler_settled=scheduler_settled,
    )


def compute_total_confidence(
    phase: PhaseConfidence,
    freq: FrequencyConfidence,
) -> TotalConfidence:
    """RSS the phase and frequency σ components.  Either being
    ``inf`` makes the total ``inf`` (caller's clockClass mapping
    will treat that as freerun-class)."""
    if math.isinf(phase.sigma_ns) or math.isinf(freq.sigma_ns):
        sigma = float("inf")
    else:
        sigma = math.sqrt(phase.sigma_ns ** 2 + freq.sigma_ns ** 2)
    return TotalConfidence(sigma_ns=sigma, phase=phase, freq=freq)


# ── Stateful promoter (hysteresis) ────────────────────────────────── #


class ClockClassPromoter:
    """Maps σ_total to a target ptp4l clockClass with hysteresis.

    Stateful: needs to remember the last decided class so the
    asymmetric thresholds form a dead band.  Call ``evaluate(sigma_ns)``
    once per [CONFIDENCE_TOTAL] cycle; the returned class is what
    the engine should pass to ``_set_clock_class``.

    Class transitions:

      freerun (248) ──[σ < 800]──> initialized (52)
      initialized (52) ──[σ < 20]──> locked (6)
      locked (6) ──[σ > 30]──> initialized (52)
      initialized (52) ──[σ > 1200]──> freerun (248)

    Holdover (class 7) is event-driven (observation idle) and is not
    set by this promoter — caller decides separately.  When the
    caller sets class 7 externally, it should also call
    ``override_class("holdover")`` so the promoter's internal state
    matches.
    """

    def __init__(self, initial_class: str = "freerun"):
        self._class = initial_class

    @property
    def current_class(self) -> str:
        return self._class

    def override_class(self, new_class: str) -> None:
        """Force the internal state to match an externally-set class
        (e.g., when the caller sets holdover or freerun via an
        event path)."""
        self._class = new_class

    def evaluate(self, sigma_ns: float) -> str:
        """Single hysteresis step.  Returns the new class name
        (may equal the previous one).

        Each transition crosses exactly one threshold; with the
        promote_ns < demote_ns gap, no σ value can trigger a
        simultaneous promote-and-demote on the same call."""
        c = self._class

        if c == "locked":
            if sigma_ns > LOCKED_DEMOTE_NS:
                c = "initialized"
        elif c == "initialized":
            if sigma_ns < LOCKED_PROMOTE_NS:
                c = "locked"
            elif sigma_ns > INITIALIZED_DEMOTE_NS:
                c = "freerun"
        elif c == "freerun":
            if sigma_ns < INITIALIZED_PROMOTE_NS:
                c = "initialized"
        # holdover: stay in holdover until the caller explicitly
        # overrides (observation flow resumed).
        self._class = c
        return c

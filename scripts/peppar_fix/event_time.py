"""Event envelopes for cross-stream correlation by local monotonic time."""

from dataclasses import dataclass
import math
from typing import Optional


def estimate_correlation_confidence(
    *,
    queue_remains: Optional[bool],
    parse_age_s: Optional[float],
    queued_base: float = 0.65,
    clear_base: float = 1.0,
    age_half_life_s: float = 0.5,
    min_confidence: float = 0.05,
) -> float:
    """Estimate confidence of source-time to host-monotonic mapping.

    Confidence is highest when the reader consumed an event without any queued
    bytes/events remaining and the event was handed to user mode immediately.
    It falls when backlog is visible or when the event sits in user space long
    enough for that receive timestamp to become stale.
    """
    base = clear_base if not queue_remains else queued_base
    if parse_age_s is None or parse_age_s <= 0.0:
        return max(min_confidence, min(1.0, base))
    age_half_life_s = max(1e-3, age_half_life_s)
    decay = math.exp(-math.log(2.0) * (parse_age_s / age_half_life_s))
    return max(min_confidence, min(1.0, base * decay))


def merge_correlation_confidence(*values: Optional[float]) -> float:
    """Combine per-stream confidence values conservatively."""
    present = [float(v) for v in values if v is not None]
    if not present:
        return 1.0
    return max(0.0, min(present))


def estimator_sample_weight(
    *,
    queue_remains: Optional[bool],
    base_confidence: float,
    min_weight: float = 0.0,
    queued_scale: float = 0.1,
) -> float:
    """Weight a source-time sample for constant-offset estimation.

    The offset between healthy same-unit timescales should be nearly constant.
    Visible backlog usually means the observed receive delay is dominated by
    queueing rather than a real change in offset, so those samples should move
    the estimator very little or not at all.
    """
    weight = max(0.0, min(1.0, float(base_confidence)))
    if queue_remains:
        weight *= queued_scale
    return max(min_weight, min(1.0, weight))


@dataclass(frozen=True)
class ObservationEvent:
    """One GNSS observation epoch with receive timestamps."""

    gps_time: object
    observations: list
    recv_mono: float
    recv_utc: object
    queue_remains: Optional[bool] = None
    parse_age_s: Optional[float] = None
    correlation_confidence: Optional[float] = None
    estimator_residual_s: Optional[float] = None
    # Pre-engine counters for the [OBS_COUNTS] emit per dayplan
    # I-143806-main.  Populated by realtime_ppp.py at obs-construction
    # time.  None when not provided (older producers / tests).
    #   n_raw          — count of receiver-visible SVs this epoch
    #   n_off_const    — rejected by the constellation filter
    #   n_single       — single-frequency (PR-only) SVs invisible to
    #                    MW/IF math; Bob's 'Tracking' column
    n_raw: Optional[int] = None
    n_off_const: Optional[int] = None
    n_single: Optional[int] = None

    def __iter__(self):
        yield self.gps_time
        yield self.observations


@dataclass(frozen=True)
class PpsEvent:
    """One PPS/EXTTS edge with receive timestamp."""

    phc_sec: int
    phc_nsec: int
    index: int
    recv_mono: float
    queue_remains: Optional[bool] = None
    parse_age_s: Optional[float] = None
    correlation_confidence: Optional[float] = None
    estimator_residual_s: Optional[float] = None

    def rounded_sec(self):
        """Integer second this PPS edge belongs to.

        A disciplined PHC has PPS near 0 ns (slightly late) or near
        1,000,000,000 ns (slightly early for the next second).  Both
        cases should resolve to the same integer second.  We round to
        the nearest whole second: nsec < 500ms → this second,
        nsec >= 500ms → next second.
        """
        return self.phc_sec if self.phc_nsec < 500_000_000 else self.phc_sec + 1

    def fractional_error_ns(self):
        """Signed PPS phase error in nanoseconds.

        Positive = PPS is late (PHC reads past the whole second).
        Negative = PPS is early (PHC reads just before the next second).
        """
        if self.phc_nsec < 500_000_000:
            return self.phc_nsec
        return self.phc_nsec - 1_000_000_000

    def __iter__(self):
        yield self.phc_sec
        yield self.phc_nsec
        yield self.index


@dataclass(frozen=True)
class RtcmEvent:
    """One RTCM message with receive timestamps."""

    identity: str
    message: object
    recv_mono: float
    recv_utc: object
    queue_remains: Optional[bool] = None
    parse_age_s: Optional[float] = None
    correlation_confidence: Optional[float] = None
    estimator_residual_s: Optional[float] = None


@dataclass(frozen=True)
class TiccEvent:
    """One TICC edge with receive timestamps."""

    channel: str
    ref_sec: int
    ref_ps: int
    recv_mono: float
    recv_utc: object = None
    queue_remains: Optional[bool] = None
    parse_age_s: Optional[float] = None
    correlation_confidence: Optional[float] = None
    estimator_residual_s: Optional[float] = None

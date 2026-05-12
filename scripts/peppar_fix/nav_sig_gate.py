"""NAV-SIG-based observation admission gate — Phase B engine-side L0.

Reads per-(SV, signal) usage verdict from `Nav2SignalStore` (populated
by the UBX-NAV-SIG parser) and decides whether to admit a pseudorange
observation into the position filter.

Per Bob's directive 2026-05-12:
  - We may use SVs the receiver excludes (different needs for time-
    only solving).
  - We may exclude SVs the receiver includes (our own quality checks
    may know something the receiver doesn't, e.g. PB_GAP_DROP, our
    ZTD-suspect status).
  - But we must LOG WHEN WE DIFFER so we can validate the composition
    empirically.

This module implements the position-filter side: by default the
position filter follows the receiver's verdict (drops SVs where
`prUsed=0` on either band).  The time filter's gate is separate and
defaults to off (in this module, the caller chooses not to consult).

Phase A.5 disagreement instrumentation is handled by Charlie's
`NavSigDisagreeMonitor` (PR #27) — this module emits only simple
counters for end-of-run summary stats and does not duplicate the
rich [NAV-SIG_DISAGREE] event format.

See docs/nav-sig-gate-spec.md for the full design and migration plan.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class GateResult:
    """Outcome of a single NAV-SIG admission decision."""

    admit:                bool
    reason:               str    # see REASON_* constants below
    receiver_excluded:    bool   # at least one band was prUsed=0
    receiver_consulted:   bool   # nav_sig_store had fresh data we read


# Reason codes — short, log-friendly.
REASON_ADMIT_BOTH_USED        = "admit"
REASON_ADMIT_NAV_SIG_STALE    = "admit_nav_sig_stale"
REASON_ADMIT_GATE_OFF         = "admit_gate_off"
REASON_DROP_NAV_SIG_F1_EXCL   = "drop_nav_sig_f1_excluded"
REASON_DROP_NAV_SIG_F2_EXCL   = "drop_nav_sig_f2_excluded"
REASON_DROP_NAV_SIG_UNHEALTHY = "drop_nav_sig_unhealthy"


def _is_fresh(sig_status, now_mono: float, max_age_s: float) -> bool:
    """A SigStatus is fresh if its host_mono is within max_age_s of now."""
    if sig_status is None:
        return False
    return (now_mono - sig_status.host_mono) <= max_age_s


def decide(nav_sig_store,
           sv: str,
           f1_sig_name: Optional[str],
           f2_sig_name: Optional[str],
           gate_enabled: bool,
           max_age_s: float,
           now_mono: float) -> GateResult:
    """Decide whether to admit this dual-frequency obs based on NAV-SIG.

    Args:
        nav_sig_store: Nav2SignalStore (or None — degraded mode).
        sv:            SV label (e.g. "G07", "E19", "C12").
        f1_sig_name:   Lower-frequency signal name (e.g. "L1CA", "E1C", "B1I").
        f2_sig_name:   Upper-frequency signal name (e.g. "L5Q", "E5aQ", "B2aI").
        gate_enabled:  --nav-sig-gate flag.
        max_age_s:     SigStatus older than this is treated as missing.
        now_mono:      time.monotonic() reference.

    Returns:
        GateResult: admit=True/False, reason+context for logging.
    """
    # Gate disabled → never block on NAV-SIG; the caller's own checks
    # are the only gate.  This is the Phase B1 default ship state.
    if not gate_enabled:
        return GateResult(
            admit=True,
            reason=REASON_ADMIT_GATE_OFF,
            receiver_excluded=False,
            receiver_consulted=False,
        )

    # Degraded mode: no store or no recent NAV-SIG → fall through.
    # This is the right call for receivers that don't emit NAV-SIG
    # (e.g. F10T on TIM 3.01 currently times out the CFG-VALSET).
    if nav_sig_store is None:
        return GateResult(
            admit=True,
            reason=REASON_ADMIT_NAV_SIG_STALE,
            receiver_excluded=False,
            receiver_consulted=False,
        )

    sig_f1 = nav_sig_store.get(sv, f1_sig_name) if f1_sig_name else None
    sig_f2 = nav_sig_store.get(sv, f2_sig_name) if f2_sig_name else None
    fresh_f1 = _is_fresh(sig_f1, now_mono, max_age_s)
    fresh_f2 = _is_fresh(sig_f2, now_mono, max_age_s)

    # Both bands stale or missing → fall through (no info to act on).
    # Operator gets a separate [NAV-SIG_STALE] warning at engine level
    # when stale-rate becomes alarming.
    if not fresh_f1 and not fresh_f2:
        return GateResult(
            admit=True,
            reason=REASON_ADMIT_NAV_SIG_STALE,
            receiver_excluded=False,
            receiver_consulted=False,
        )

    # Check each band that we DO have fresh data for.
    # Per Bob: "one bad signal doesn't make for a bad SV" — but the
    # IF combination requires BOTH bands.  If either band is excluded
    # by the receiver, the IF measurement is questionable; drop the
    # obs from this epoch.  (Single-frequency processing path that
    # would let us keep the good band is not enabled in the engine
    # today; revisit if that path lands.)
    f1_excluded = fresh_f1 and not sig_f1.pr_used
    f2_excluded = fresh_f2 and not sig_f2.pr_used
    f1_unhealthy = fresh_f1 and sig_f1.health == 2
    f2_unhealthy = fresh_f2 and sig_f2.health == 2

    if f1_unhealthy or f2_unhealthy:
        return GateResult(
            admit=False,
            reason=REASON_DROP_NAV_SIG_UNHEALTHY,
            receiver_excluded=True,
            receiver_consulted=True,
        )
    if f1_excluded:
        return GateResult(
            admit=False,
            reason=REASON_DROP_NAV_SIG_F1_EXCL,
            receiver_excluded=True,
            receiver_consulted=True,
        )
    if f2_excluded:
        return GateResult(
            admit=False,
            reason=REASON_DROP_NAV_SIG_F2_EXCL,
            receiver_excluded=True,
            receiver_consulted=True,
        )

    return GateResult(
        admit=True,
        reason=REASON_ADMIT_BOTH_USED,
        receiver_excluded=False,
        receiver_consulted=True,
    )


class NavSigGateCounters:
    """Simple per-epoch counters for end-of-run summary.

    Detailed disagreement logging is Charlie's NavSigDisagreeMonitor
    (PR #27) — this is just lightweight visibility into the gate's
    operation (how often does it fire, how often is it stale).
    """

    def __init__(self):
        self.epoch_admit_gate_off    = 0
        self.epoch_admit_both_used   = 0
        self.epoch_admit_stale       = 0
        self.epoch_drop_f1_excluded  = 0
        self.epoch_drop_f2_excluded  = 0
        self.epoch_drop_unhealthy    = 0

    def record(self, result: GateResult) -> None:
        if result.reason == REASON_ADMIT_GATE_OFF:
            self.epoch_admit_gate_off += 1
        elif result.reason == REASON_ADMIT_BOTH_USED:
            self.epoch_admit_both_used += 1
        elif result.reason == REASON_ADMIT_NAV_SIG_STALE:
            self.epoch_admit_stale += 1
        elif result.reason == REASON_DROP_NAV_SIG_F1_EXCL:
            self.epoch_drop_f1_excluded += 1
        elif result.reason == REASON_DROP_NAV_SIG_F2_EXCL:
            self.epoch_drop_f2_excluded += 1
        elif result.reason == REASON_DROP_NAV_SIG_UNHEALTHY:
            self.epoch_drop_unhealthy += 1

    def as_dict(self) -> dict:
        return {
            'admit_gate_off':   self.epoch_admit_gate_off,
            'admit_both_used':  self.epoch_admit_both_used,
            'admit_stale':      self.epoch_admit_stale,
            'drop_f1_excluded': self.epoch_drop_f1_excluded,
            'drop_f2_excluded': self.epoch_drop_f2_excluded,
            'drop_unhealthy':   self.epoch_drop_unhealthy,
        }

    def reset_epoch(self) -> None:
        """Reset per-epoch counters (call at start of each filter update)."""
        self.epoch_admit_gate_off = 0
        self.epoch_admit_both_used = 0
        self.epoch_admit_stale = 0
        self.epoch_drop_f1_excluded = 0
        self.epoch_drop_f2_excluded = 0
        self.epoch_drop_unhealthy = 0

"""Real-time per-SV cycle-slip detection and phase-state flush.

Design principle: on a detected slip, every per-SV PHASE-LIKE state is
erased; per-SV and shared FREQUENCY-LIKE state is retained.  A
carrier-phase slip injects an unknown integer cycle jump into the
tracking loop's accumulated phase, so any quantity that depends on a
pre-slip phase or an integer ambiguity becomes invalid.  Quantities
built from phase *differences* over short intervals (drift rates,
shared receiver clock states) do not change in a single-SV slip and
can be retained.

Per-SV PHASE-LIKE state flushed on slip (traced 2026-04-18):
  - PPPFilter.x[N_BASE + sv_to_idx[sv]]        (IF ambiguity)
  - PPPFilter.P[si, :], P[:, si], P[si, si]     (ambiguity covariance)
  - PPPFilter.sv_to_idx[sv]                     (ambiguity slot)
  - PPPFilter.prev_obs[sv]                      (last phi_if_m)
  - MelbourneWubbenaTracker._state[sv]          (MW avg, WL integer)
  - NarrowLaneResolver._fixed[sv]               (NL integer + a_if_fixed)
  - FixedPosFilter.prev_geo[sv]                 (time-diff last phi_if_m)
  - PostFixResidualMonitor._per_sv[sv],         (PR/phi residual deques)
    ._per_sv_phi[sv], ._per_sv_last_elev[sv]

State retained across slip (frequency-like or per-receiver):
  - PPPFilter.x[IDX_CLK, IDX_ISB_*, IDX_ZTD]    (shared clock/atmos)
  - RxTcxoTracker._accumulated_ns, _prev_dt_rx  (receiver TCXO phase/rate)
  - CarrierPhaseTracker drift_rate_ppb, anchor  (DO drift)
  - InBandNoiseEstimator                        (DO noise floor)
  - NarrowLaneResolver._blacklist[sv]           (temporal anti-lock-in)

Four independent detectors run per epoch.  Any detector firing triggers
flush_sv_phase() once per epoch per SV; a slip with ≥2 detectors firing
is tagged HIGH confidence (for antenna-quality reporting), else LOW.

Original design: "Flushing runs regardless of confidence — the cost of
a spurious flush is one ambiguity re-convergence, far cheaper than
contaminating NL."  THIS WAS EMPIRICALLY WRONG.  Audit on day0506 MadHat
(scripts/replay/slip_vs_wl_audit.py + docs/wl-integer-vs-pride-comparison.md)
showed:
  - 47.8 % (1275/2669) of slip events are mw_jump alone — driven by
    PR-side noise spikes, not real cycle slips.  MW formula isn't
    PR-free, so PR multipath shifts the running mean by λ_WL cycles.
  - On post-flush re-fix, even CARRIER-confirmed slips produced a
    DIFFERENT integer 99.6 % of the time (median 38 cycles ≈ 28 m of
    WL displacement).  The "cheap re-convergence" assumption was wrong:
    re-fix uses fresh MW samples that are still PR-noise-contaminated,
    landing on a wrong integer most of the time.

Corrected design (I-155354 fix-I, 2026-05-09): use is_pr_only_slip()
to identify mw_jump-only events and skip the WL-tracker reset for
those.  Carrier-confirmed slips (gf_jump, ubx_locktime_drop, arc_gap)
still flush as before.  Per-SV ambiguity preservation across PR-noise
spikes keeps the MW running mean's history available; the integer
isn't lost to a re-fix on noisy data.

  1. UBX locktime drop
     u-blox resets locktime_ms on tracking-loop cycle slip.  A drop
     > LOCKTIME_DROP_MS within ARC_GAP_MAX_S of the previous epoch
     signals a slip.  Ignored when the previous locktime was above
     UBX_LOCKTIME_CAP_MS (u-blox holds locktime steady above ~64s, so
     a "drop" there is just a cap-wrap artefact).

  2. Arc continuity
     Any gap > ARC_GAP_MAX_S since this SV was last seen restarts the
     tracking arc.  Receiver may report a fresh high locktime, but the
     phase is from a new arc and the old ambiguity is invalid.

  3. Geometry-free (GF) phase jump
     L1·λ1 − L5·λ5 drifts at most at the ionospheric rate (mm/s).  A
     jump larger than GF_JUMP_THRESHOLD_M (quarter-L1 wavelength, ~5cm)
     between consecutive epochs is a slip on at least one frequency.

  4. Melbourne-Wubbena residual jump
     MW = geometry-free, ionosphere-free wide-lane combination; its
     running average is λ_WL · N_WL.  A jump > 3·σ of the running
     residual catches slips that leave locktime untouched (within-arc
     half-cycle slips on well-locked tracking).  Delegated to
     MelbourneWubbenaTracker.detect_jump().
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Optional

log = logging.getLogger(__name__)


# ── Physical constants ────────────────────────────────────────────── #

_C_LIGHT = 299_792_458.0
_F_L1 = 1_575.42e6
_LAMBDA_L1 = _C_LIGHT / _F_L1                           # ~0.190 m


# ── Detector thresholds ──────────────────────────────────────────── #

LOCKTIME_DROP_MS = 500.0
ARC_GAP_MAX_S = 1.5                                     # ~1 epoch at 1 Hz
# Distinct from ARC_GAP_MAX_S: ARC_GAP_MAX_S is a RECENCY window for
# detectors that need adjacent epochs (ubx_locktime_drop, gf_jump).
# ARC_RESTART_GAP_S is the threshold for declaring the tracking arc
# itself broken.  Single packet drops at 1 Hz produce 2-4 s gaps
# without a real lock loss, and the lock_ms secondary gate doesn't
# reliably catch them (u-blox resets lock_ms for brief tracking-loop
# lapses that aren't cycle slips).  The ptpmon day0420e overnight and
# our L5-fleet day0419k both show ~55-71 % of "slips" are LOW-confidence
# arc_gaps under 10 s with the 1.5 s threshold — all likely false
# positives.  30 s still catches any genuine arc restart (SV setting,
# receiver resync, occlusion > 30 s), while the MW-jump detector
# independently catches real integer slips inside the 1.5 s – 30 s
# window.
ARC_RESTART_GAP_S = 30.0
GF_JUMP_THRESHOLD_M = _LAMBDA_L1 / 4.0                  # ~4.76 cm
MW_JUMP_N_SIGMA = 5.0   # combined with a 0.5-cyc sigma floor in
                         # MelbourneWubbenaTracker; threshold ≥ 2.5 cyc
                         # keeps MW a multi-cycle backstop while GF
                         # handles single-cycle slips.
UBX_LOCKTIME_CAP_MS = 64_000.0                          # u-blox capping band


# ── SlipEvent ────────────────────────────────────────────────────── #

@dataclass
class SlipEvent:
    """One detected slip, capturing enough context for antenna-quality
    reporting and for debugging a false positive after the fact."""
    sv: str
    epoch: int
    reasons: list[str]
    lock_ms: float = 0.0
    cno: float = 0.0
    elevation_deg: Optional[float] = None
    gap_s: Optional[float] = None
    gf_jump_m: Optional[float] = None
    mw_jump_cyc: Optional[float] = None
    # Sky geometry at slip time (Bob 2026-04-24).  Enables post-hoc
    # analysis by azimuth octant and by solar-zenith-angle at the
    # ionospheric pierce point — the latter is a cheap proxy for
    # "how close is this signal's IPP to the terminator right now."
    # SZA ≈ 90° flags the twilight band where dTEC/dt is largest.
    # Both None when the engine couldn't compute them (no filter
    # position, missing sat ephemeris, SV below horizon).
    azimuth_deg: Optional[float] = None
    ipp_sza_deg: Optional[float] = None

    @property
    def confidence(self) -> str:
        return "HIGH" if len(self.reasons) >= 2 else "LOW"

    def as_csv_row(self) -> list:
        return [
            self.epoch,
            self.sv,
            "|".join(self.reasons),
            self.confidence,
            f"{self.lock_ms:.0f}",
            f"{self.cno:.1f}",
            f"{self.elevation_deg:.1f}" if self.elevation_deg is not None else "",
            f"{self.azimuth_deg:.1f}" if self.azimuth_deg is not None else "",
            f"{self.ipp_sza_deg:.1f}" if self.ipp_sza_deg is not None else "",
            f"{self.gap_s:.2f}" if self.gap_s is not None else "",
            f"{self.gf_jump_m:.4f}" if self.gf_jump_m is not None else "",
            f"{self.mw_jump_cyc:.3f}" if self.mw_jump_cyc is not None else "",
        ]

    @staticmethod
    def csv_header() -> list[str]:
        return ["epoch", "sv", "reasons", "confidence", "lock_ms",
                "cno", "elev_deg", "az_deg", "ipp_sza_deg",
                "gap_s", "gf_jump_m", "mw_jump_cyc"]


# ── Monitor ──────────────────────────────────────────────────────── #

@dataclass
class _PrevObs:
    t_mono_s: float
    lock_ms: float
    phi1_cyc: Optional[float]
    phi2_cyc: Optional[float]
    wl_f1: Optional[float]
    wl_f2: Optional[float]


class CycleSlipMonitor:
    """Per-SV slip monitor — stateful across epochs.

    mw_tracker (optional): a MelbourneWubbenaTracker exposing
    detect_jump(obs) -> dict|None.  When supplied, MW-jump is a fourth
    detector; without it, only the first three detectors run.
    """

    def __init__(self, *, mw_tracker=None, stale_after_s: float = 60.0,
                 csv_writer=None, min_slip_interval_s: float = 0.0):
        self._prev: dict[str, _PrevObs] = {}
        self._mw_tracker = mw_tracker
        self._stale_after_s = stale_after_s
        # Optional CSV sink — one row per SlipEvent.
        self._csv_writer = csv_writer
        # Antenna-quality counters
        self._count_total: dict[str, int] = {}
        self._count_by_reason: dict[str, dict[str, int]] = {}
        # Slip-rate-limiter: suppress per-SV events that fire within
        # min_slip_interval_s of the previous emitted event for the same
        # SV.  Prevents the per-SV state machine from churning during a
        # multi-epoch disturbance (one E29-style slip storm currently fires
        # 4-10× consecutive events).  0.0 = disabled (legacy).  Bravo's
        # day0426 trajectory analysis recommends ~30-300 s; tighter values
        # accept some real consecutive slips, looser values prefer state-
        # machine stability.  Per-SV last-emit timestamp + suppressed count
        # are diagnostics-only (not in the event stream).
        self._min_slip_interval_s = float(min_slip_interval_s)
        self._last_slip_t: dict[str, float] = {}
        self._count_suppressed: dict[str, int] = {}

    # ── core ─────────────────────────────────────────────────────── #

    def check(self, observations, t_mono_s: float,
              epoch: int, elevations=None,
              azimuths=None, ipp_szas=None) -> list[SlipEvent]:
        """Return list of SlipEvent for SVs that slipped this epoch.

        observations : list of dicts with keys sv, lock_duration_ms, cno,
                       phi1_cyc, phi2_cyc, wl_f1, wl_f2, pr1_m, pr2_m.
        elevations : optional dict sv -> elev_deg for SlipEvent tagging.
        azimuths   : optional dict sv -> az_deg  (0°=N, 90°=E).
        ipp_szas   : optional dict sv -> IPP solar zenith angle (deg);
                     ~90° flags the terminator twilight band.
        """
        elevations = elevations or {}
        azimuths = azimuths or {}
        ipp_szas = ipp_szas or {}
        events: list[SlipEvent] = []
        current_svs: set[str] = set()

        for obs in observations:
            sv = obs['sv']
            current_svs.add(sv)
            lock_ms = float(obs.get('lock_duration_ms') or 0.0)
            cno = float(obs.get('cno') or 0.0)

            prev = self._prev.get(sv)
            reasons: list[str] = []
            gap_s: Optional[float] = None
            gf_delta: Optional[float] = None
            mw_delta_cyc: Optional[float] = None

            if prev is not None:
                gap_s = t_mono_s - prev.t_mono_s

                # 1. UBX locktime drop (only meaningful below the cap
                #    and when the previous epoch was recent).
                if (prev.lock_ms < UBX_LOCKTIME_CAP_MS
                        and gap_s <= ARC_GAP_MAX_S
                        and (prev.lock_ms - lock_ms) > LOCKTIME_DROP_MS):
                    reasons.append("ubx_locktime_drop")

                # 2. Arc continuity — a gap larger than ARC_RESTART_GAP_S
                #    may indicate a broken tracking arc.  Below that,
                #    the gap is almost certainly a packet drop upstream
                #    (observation-stream decimation, half-cycle
                #    transients, SSR bias lookup miss, constellation
                #    filter churn) even if lock_ms looks low, because
                #    u-blox resets lock_ms for brief tracking-loop
                #    lapses that aren't cycle slips.  Above the
                #    threshold, we still apply the secondary lock_ms
                #    gate: if current locktime substantially exceeds
                #    the gap, the SV was locked the whole time so
                #    don't flag.
                if gap_s > ARC_RESTART_GAP_S:
                    gap_ms = gap_s * 1000.0
                    if lock_ms < gap_ms + LOCKTIME_DROP_MS:
                        reasons.append("arc_gap")

                # 3. Geometry-free phase jump between consecutive epochs.
                #    Uses RAW tracking phase (phi*_raw_cyc) when available,
                #    falling back to phi*_cyc.  Raw matters because SSR
                #    phase biases can step by a full wavelength when the
                #    AC flips its integer-indicator — that is NOT a slip
                #    in the receiver's tracking loop, but it shows up as
                #    a 10–65 cm GF jump on the bias-corrected phase.
                #    Observed on ptpmon 2026-04-19: 7-SV simultaneous
                #    gf_jump flush at epoch 55, every SV with lock_ms
                #    stable at 64 s (receiver wasn't slipping).
                phi1 = obs.get('phi1_raw_cyc', obs.get('phi1_cyc'))
                phi2 = obs.get('phi2_raw_cyc', obs.get('phi2_cyc'))
                wl1, wl2 = obs.get('wl_f1'), obs.get('wl_f2')
                if (phi1 is not None and phi2 is not None and wl1 and wl2
                        and prev.phi1_cyc is not None
                        and prev.phi2_cyc is not None
                        and prev.wl_f1 and prev.wl_f2
                        and gap_s is not None and gap_s <= ARC_GAP_MAX_S):
                    gf_cur = phi1 * wl1 - phi2 * wl2
                    gf_prev = prev.phi1_cyc * prev.wl_f1 \
                        - prev.phi2_cyc * prev.wl_f2
                    gf_delta = gf_cur - gf_prev
                    if abs(gf_delta) > GF_JUMP_THRESHOLD_M:
                        reasons.append("gf_jump")

            # 4. Melbourne-Wubbena residual jump — independent of prev_obs
            #    history kept by this monitor; MWTracker holds its own.
            if self._mw_tracker is not None:
                try:
                    mw_info = self._mw_tracker.detect_jump(
                        obs, n_sigma=MW_JUMP_N_SIGMA)
                except AttributeError:
                    mw_info = None
                if mw_info is not None:
                    mw_delta_cyc = mw_info.get('delta_cyc')
                    if mw_info.get('is_slip'):
                        reasons.append("mw_jump")

            if reasons:
                # Slip-rate-limiter: if the last emitted event for this SV
                # was less than min_slip_interval_s ago, treat this jump as
                # continuation of the same disturbance.  Prevents the per-
                # SV state machine from cycling FLOAT→CONVERGING→ANCHORING
                # multiple times within one slip storm.  See the day0426
                # trajectory analysis memo.
                last_t = self._last_slip_t.get(sv)
                if (self._min_slip_interval_s > 0
                        and last_t is not None
                        and (t_mono_s - last_t) < self._min_slip_interval_s):
                    self._count_suppressed[sv] = (
                        self._count_suppressed.get(sv, 0) + 1)
                else:
                    ev = SlipEvent(
                        sv=sv,
                        epoch=epoch,
                        reasons=reasons,
                        lock_ms=lock_ms,
                        cno=cno,
                        elevation_deg=elevations.get(sv),
                        azimuth_deg=azimuths.get(sv),
                        ipp_sza_deg=ipp_szas.get(sv),
                        gap_s=gap_s,
                        gf_jump_m=gf_delta,
                        mw_jump_cyc=mw_delta_cyc,
                    )
                    events.append(ev)
                    self._last_slip_t[sv] = t_mono_s
                    self._count_total[sv] = self._count_total.get(sv, 0) + 1
                    per_reason = self._count_by_reason.setdefault(sv, {})
                    for r in reasons:
                        per_reason[r] = per_reason.get(r, 0) + 1
                    if self._csv_writer is not None:
                        try:
                            self._csv_writer.writerow(ev.as_csv_row())
                        except Exception:
                            log.warning("slip csv write failed",
                                        exc_info=True)

            # Store RAW phase for the next epoch's GF comparison — see
            # detector 3 above for why.
            self._prev[sv] = _PrevObs(
                t_mono_s=t_mono_s,
                lock_ms=lock_ms,
                phi1_cyc=obs.get('phi1_raw_cyc', obs.get('phi1_cyc')),
                phi2_cyc=obs.get('phi2_raw_cyc', obs.get('phi2_cyc')),
                wl_f1=obs.get('wl_f1'),
                wl_f2=obs.get('wl_f2'),
            )

        self._prune_stale(t_mono_s, current_svs)
        return events

    # ── maintenance ──────────────────────────────────────────────── #

    def forget(self, sv: str) -> None:
        """Drop prev-obs memory for sv.  Called by flush_sv_phase so the
        next epoch sees a fresh arc and detector 2 doesn't fire again."""
        self._prev.pop(sv, None)

    def _prune_stale(self, t_mono_s: float, current_svs: set[str]) -> None:
        stale = [sv for sv, p in self._prev.items()
                 if sv not in current_svs
                 and (t_mono_s - p.t_mono_s) > self._stale_after_s]
        for sv in stale:
            self._prev.pop(sv, None)

    # ── diagnostics ──────────────────────────────────────────────── #

    def stats(self) -> dict[str, dict]:
        return {
            sv: {'total': self._count_total[sv],
                 'by_reason': dict(self._count_by_reason.get(sv, {})),
                 'suppressed': self._count_suppressed.get(sv, 0)}
            for sv in self._count_total
        }

    def summary_line(self) -> str:
        if not self._count_total:
            return "slips: none"
        parts = []
        for sv in sorted(self._count_total):
            total = self._count_total[sv]
            reasons = self._count_by_reason.get(sv, {})
            rs = ",".join(f"{r}={c}" for r, c in sorted(reasons.items()))
            parts.append(f"{sv}:{total}[{rs}]")
        return "slips: " + " ".join(parts)


# ── Slip-classification helpers ──────────────────────────────────── #


# Slip-event reason strings the detectors emit.  See SlipEvent.reasons.
_PR_ONLY_REASON_SET = frozenset({"mw_jump"})


def is_pr_only_slip(reasons) -> bool:
    """True iff a slip event's reason set is exactly {'mw_jump'} —
    i.e., NO carrier-phase corroboration (gf_jump, ubx_locktime_drop,
    arc_gap).

    The MW formula
      MW = phase_WL - pseudorange_NL
    isn't PR-free.  PR multipath spikes shift the MW running mean by
    O(λ_WL) cycles even when carrier-phase is healthy.  The slip
    detector flags these as `mw_jump`-only events with conf=LOW.

    Engine consumers should use this to GATE WL-tracker reset:
    PR-only slips don't represent real ambiguity discontinuity, so
    flushing the MW state and re-fixing on fresh (still-noisy) MW
    samples lands on a wrong integer most of the time.

    See scripts/replay/slip_vs_wl_audit.py +
    docs/wl-integer-vs-pride-comparison.md for the empirical
    grounding (47.8 % of slip events on day0506 MadHat are
    PR-only; 99.6 % of post-flush re-fixes land on different
    integers).

    Accepts any iterable of reason strings.
    """
    return frozenset(reasons) == _PR_ONLY_REASON_SET


# ── Flush entry point ────────────────────────────────────────────── #

def flush_sv_phase(sv: str,
                   *,
                   filt=None,
                   mw_tracker=None,
                   nl_resolver=None,
                   pfr_monitor=None,
                   fixed_pos_filter=None,
                   slip_monitor=None,
                   sv_state=None,
                   confidence: str = "LOW",
                   reason: str = "",
                   epoch: int = 0) -> None:
    """Erase every per-SV phase-like state associated with sv.

    Each holder may be None (skip).  Frequency-like state elsewhere in
    the system (receiver clock family, TCXO, DO drift, noise floor) is
    intentionally untouched — see module docstring.

    Per-SV state machine (per docs/sv-lifecycle-and-pfr-split.md):
      - confidence=="HIGH" → any state → WAITING
      - confidence=="LOW"  → any state → FLOATING
    The tracker's transition is called BEFORE the downstream resets so
    the [SV_STATE] log line is coherent with the post-reset state.
    When sv_state is None the transition is skipped (legacy callers).
    """
    # Per-SV state transition runs first so downstream resets
    # (mw_tracker.reset, nl_resolver.unfix) don't re-fire their own
    # tracker transitions and produce duplicate log lines.
    if sv_state is not None:
        # Lazy import to avoid a circular dep between cycle_slip.py and
        # sv_state.py's future imports.
        from peppar_fix.sv_state import SvAmbState, InvalidTransition
        target = SvAmbState.WAITING if confidence == "HIGH" else SvAmbState.FLOATING
        try:
            sv_state.transition(
                sv, target, epoch=epoch,
                reason=f"slip:{reason or '?'} conf={confidence}",
            )
        except InvalidTransition:
            # The only illegal cycle-slip target is "already in target" —
            # which transition() treats as a no-op.  Anything else
            # (WAITING → FLOATING implied by slip during cooldown) is
            # a design question; for now, log and continue.
            log.debug("slip transition noop for %s (already in %s)",
                      sv, sv_state.state(sv).value)

    if filt is not None:
        # Removes x[N_BASE + si], P row/col, sv_to_idx[sv].  The caller's
        # normal ambiguity-management loop will call add_ambiguity() on
        # the next epoch with a fresh N_init derived from current phase.
        filt.remove_ambiguity(sv)
        prev_obs = getattr(filt, 'prev_obs', None)
        if isinstance(prev_obs, dict):
            prev_obs.pop(sv, None)

    if mw_tracker is not None:
        mw_tracker.reset(sv)          # clears mw_avg, n_wl, fixed flag

    if nl_resolver is not None:
        nl_resolver.unfix(sv)         # drops n1 integer + a_if_fixed
        # Real phase discontinuity (cycle-slip-flush is the canonical
        # source for this code path) invalidates the SV's pre-slip
        # NL integer reference — wipe trust so the next admission is
        # gated as NEW.  See I-172719 / NlAdmissionTier.forget_history.
        if hasattr(nl_resolver, 'note_slip'):
            nl_resolver.note_slip(sv)

    if pfr_monitor is not None:
        for attr in ('_per_sv', '_per_sv_phi', '_per_sv_last_elev'):
            d = getattr(pfr_monitor, attr, None)
            if isinstance(d, dict):
                d.pop(sv, None)

    if fixed_pos_filter is not None:
        prev_geo = getattr(fixed_pos_filter, 'prev_geo', None)
        if isinstance(prev_geo, dict):
            prev_geo.pop(sv, None)

    if slip_monitor is not None:
        slip_monitor.forget(sv)

    log.info("cycle slip flush: sv=%s epoch=%s reason=%s",
             sv, epoch, reason or "?")

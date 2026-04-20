#!/usr/bin/env python3
"""
ppp_ar.py — PPP-AR via Melbourne-Wubbena wide-lane + narrow-lane resolution.

Two-step ambiguity resolution that works with the existing IF PPPFilter:

  Step 1: MelbourneWubbenaTracker computes the geometry-free MW combination
          per satellite and fixes the wide-lane integer N_WL = N1 - N2.
          Converges in ~60 s from code-phase averaging.

  Step 2: NarrowLaneResolver extracts the narrow-lane integer N1 from
          the PPPFilter's float IF ambiguity using the known N_WL:
            N1 = (A_IF - alpha2 * lambda_WL * N_WL) / lambda_NL
          Fixes N1 when |frac| < threshold and sigma is small.

No changes to PPPFilter internals — AR sits alongside and constrains
the existing float ambiguity states.
"""

import logging
import math
from collections import deque

import numpy as np

from solve_pseudorange import C
from solve_ppp import N_BASE
from lambda_ar import lambda_resolve, lambda_decorrelate, bootstrap_success_rate

# Per-SV state machine — the AR paths drive transitions on fix / unfix
# / reset.  Tracker is optional (callers can pass None) so tests and
# legacy code paths keep working without wiring one up.
from peppar_fix.sv_state import SvAmbState, SvStateTracker
from peppar_fix.nl_diag import (
    NlDiagLogger,
    RESULT_CAND, RESULT_FIXED_LAMBDA, RESULT_FIXED_ROUNDING,
    RESULT_SKIP_ELEV, RESULT_SKIP_BLACKLIST, RESULT_SKIP_NO_WL,
    RESULT_SKIP_NO_FREQS, RESULT_SKIP_NO_PHASE_BIAS, RESULT_SKIP_PRESCREEN,
    RESULT_REJ_LAMBDA_RATIO, RESULT_REJ_LAMBDA_BOOTSTRAP,
    RESULT_REJ_LAMBDA_DISPLACEMENT, RESULT_REJ_CORNER, RESULT_REJ_RECT,
)

log = logging.getLogger(__name__)


class MelbourneWubbenaTracker:
    """Per-satellite Melbourne-Wubbena wide-lane averaging and fixing.

    MW = (f1*phi1 - f2*phi2)/(f1 - f2) - (f1*P1 + f2*P2)/(f1 + f2)
       = lambda_WL * N_WL  +  code_noise

    After averaging, N_WL = round(MW_avg / lambda_WL).
    """

    # Rolling residual window length for the slip-jump detector.
    _RESID_WIN = 60
    # Minimum samples before detect_jump will return an opinion.
    _MIN_EPOCHS_FOR_JUMP = 20
    # Floor on residual sigma (cycles).  MW single-epoch noise is
    # dominated by pseudorange code noise (~0.5 m RMS → ~0.67 cyc at
    # lambda_WL ≈ 0.75 m).  A too-tight floor triggers on PR noise and
    # SSR code-bias updates — not real carrier slips.  0.5 × 5σ gives a
    # 2.5 cyc threshold that only catches multi-cycle WL-only slips;
    # single-cycle carrier slips fall out of the GF detector anyway
    # because any realistic small integer change (Δ₁ or Δ₂ < ~100)
    # that moves WL also moves GF by >1 cm.
    _SIGMA_FLOOR_CYC = 0.50

    def __init__(self, tau_s=60.0, fix_threshold=0.15, min_epochs=60,
                 sv_state: SvStateTracker | None = None):
        self.tau_s = tau_s              # exponential averaging time constant
        self.fix_threshold = fix_threshold  # |frac| < this to fix
        self.min_epochs = min_epochs    # minimum epochs before fixing
        self._state = {}   # sv -> {mw_avg, n_epochs, n_wl, fixed, f1, f2, ...}
        # Optional tracker — when supplied, MW fix drives FLOAT → WL_FIXED
        # and reset drives WL_FIXED → FLOAT.
        self._sv_state = sv_state
        # External callers write to this before calling update(); the
        # tracker's transition log line includes the current epoch.  None
        # is fine — transitions are still legal, just logged with epoch=0.
        self._current_epoch = 0

    @staticmethod
    def _mw_meters(phi1_cyc, phi2_cyc, pr1_m, pr2_m, f1, f2):
        """Melbourne-Wubbena combination, meters."""
        return (
            (f1 * phi1_cyc * (C / f1) - f2 * phi2_cyc * (C / f2)) / (f1 - f2)
            - (f1 * pr1_m + f2 * pr2_m) / (f1 + f2)
        )

    @staticmethod
    def _freqs_from_obs(obs):
        """(f1, f2) in Hz from an observation dict with wl_f1/wl_f2."""
        wl1, wl2 = obs.get('wl_f1'), obs.get('wl_f2')
        if not wl1 or not wl2:
            return None
        return C / wl1, C / wl2

    def update(self, sv, phi1_cyc, phi2_cyc, pr1_m, pr2_m, f1, f2):
        """Update MW average for one satellite.

        Args:
            phi1_cyc, phi2_cyc: carrier phase in cycles (bias-corrected)
            pr1_m, pr2_m: pseudorange in meters
            f1, f2: frequencies in Hz
        """
        # Admit: receiver-tracked SV passing the elev/health/constellation
        # gate enters processing.  MW.update is the first hook that sees
        # a dual-frequency observation that has passed all those gates,
        # so it's the natural place to transition TRACKING → FLOAT.
        # Idempotent for SVs already in FLOAT or later states.
        if self._sv_state is not None:
            cur = self._sv_state.state(sv)
            if cur is SvAmbState.TRACKING:
                self._sv_state.transition(
                    sv, SvAmbState.FLOAT,
                    epoch=self._current_epoch, reason="admit",
                )

        lambda_wl = C / (f1 - f2)
        mw = self._mw_meters(phi1_cyc, phi2_cyc, pr1_m, pr2_m, f1, f2)

        if sv not in self._state:
            self._state[sv] = {
                'mw_avg': mw,
                'n_epochs': 1,
                'n_wl': None,
                'fixed': False,
                'f1': f1,
                'f2': f2,
                'resid_deque': deque(maxlen=self._RESID_WIN),
                'resid_std_cyc': None,
            }
            return

        s = self._state[sv]
        # Track residual from the *pre-update* average so jump detection
        # can read a sigma that hasn't absorbed the current sample yet.
        residual_cyc = (mw - s['mw_avg']) / lambda_wl
        rd = s.setdefault('resid_deque', deque(maxlen=self._RESID_WIN))
        rd.append(residual_cyc)
        if len(rd) >= 8:
            mean = sum(rd) / len(rd)
            var = sum((x - mean) ** 2 for x in rd) / len(rd)
            s['resid_std_cyc'] = math.sqrt(var)

        # Exponential moving average
        alpha = 1.0 / max(1.0, min(s['n_epochs'] + 1, self.tau_s))
        s['mw_avg'] = (1.0 - alpha) * s['mw_avg'] + alpha * mw
        s['n_epochs'] += 1

        if s['fixed']:
            return

        if s['n_epochs'] >= self.min_epochs:
            n_wl_float = s['mw_avg'] / lambda_wl
            frac = abs(n_wl_float - round(n_wl_float))
            if frac < self.fix_threshold:
                s['n_wl'] = round(n_wl_float)
                s['fixed'] = True
                # Retain fix-time quality for later diagnostics (PFR unfix
                # wants to know whether this WL was fixed at a marginal
                # frac / short epoch count — evidence of premature fix).
                s['fix_frac'] = float(frac)
                s['fix_n_epochs'] = int(s['n_epochs'])
                log.info("WL fixed: %s N_WL=%d (frac=%.3f, %d epochs)",
                         sv, s['n_wl'], frac, s['n_epochs'])
                # Per-SV state: FLOAT → WL_FIXED on MW convergence.  On
                # re-fix after a slip-induced reset the state is already
                # FLOAT, so this is the right edge.  SQUELCHED SVs are
                # skipped by caller (they shouldn't reach here), but if
                # they do, the transition is illegal and will raise —
                # caught loudly in tests, benign in production.
                if self._sv_state is not None:
                    cur = self._sv_state.state(sv)
                    if cur is SvAmbState.FLOAT:
                        self._sv_state.transition(
                            sv, SvAmbState.WL_FIXED,
                            epoch=self._current_epoch,
                            reason=f"MW converged (frac={frac:.3f}, {s['n_epochs']} ep)",
                        )

    def get_wl(self, sv):
        """Return fixed N_WL for satellite, or None if not yet fixed."""
        s = self._state.get(sv)
        if s is not None and s['fixed']:
            return s['n_wl']
        return None

    def get_freqs(self, sv):
        """Return (f1, f2) for satellite, or None."""
        s = self._state.get(sv)
        if s is not None:
            return s['f1'], s['f2']
        return None

    def reset(self, sv):
        """Reset state for a satellite (e.g. after cycle slip).

        The per-SV state-machine transition is NOT driven from here —
        slip-induced transitions come from CycleSlipMonitor so that the
        slip confidence (HIGH → SQUELCHED, LOW → FLOAT) reaches the
        tracker correctly.  reset() just drops MW internal state.
        """
        self._state.pop(sv, None)

    def detect_jump(self, obs, n_sigma=3.0):
        """Non-mutating slip check for one observation.

        Computes MW for the current observation and compares it to the
        running average.  A jump exceeding n_sigma·σ of recent residuals
        (with a floor of _SIGMA_FLOOR_CYC so a quiet window can't make
        the threshold arbitrarily tight) is reported as a slip.

        Returns dict {is_slip, delta_cyc, sigma_cyc} or None if there
        isn't yet enough history to compare against.  The caller — the
        CycleSlipMonitor — uses this as one of four independent
        detectors; it does not mutate tracker state.
        """
        sv = obs['sv']
        s = self._state.get(sv)
        if s is None or s['n_epochs'] < self._MIN_EPOCHS_FOR_JUMP:
            return None

        freqs = self._freqs_from_obs(obs)
        if freqs is None:
            return None
        f1, f2 = freqs
        lambda_wl = C / (f1 - f2)

        phi1, phi2 = obs.get('phi1_cyc'), obs.get('phi2_cyc')
        pr1, pr2 = obs.get('pr1_m'), obs.get('pr2_m')
        if None in (phi1, phi2, pr1, pr2):
            return None

        mw = self._mw_meters(phi1, phi2, pr1, pr2, f1, f2)
        delta_cyc = (mw - s['mw_avg']) / lambda_wl
        sigma_cyc = max(s.get('resid_std_cyc') or 0.0, self._SIGMA_FLOOR_CYC)
        is_slip = abs(delta_cyc) > n_sigma * sigma_cyc
        return {
            'is_slip': is_slip,
            'delta_cyc': delta_cyc,
            'sigma_cyc': sigma_cyc,
        }

    @property
    def n_fixed(self):
        return sum(1 for s in self._state.values() if s['fixed'])

    def summary(self):
        n_total = len(self._state)
        return f"WL: {self.n_fixed}/{n_total} fixed"


class NarrowLaneResolver:
    """Resolve narrow-lane integer N1 from float IF ambiguity + fixed N_WL.

    N1 = (A_IF - alpha2 * lambda_WL * N_WL) / lambda_NL

    Fixes the PPPFilter ambiguity state when N1 is close to integer.
    """

    def __init__(self, frac_threshold=0.10, sigma_threshold=0.12,
                 corner_margin_sum=1.6, blacklist_epochs=60,
                 ar_elev_mask_deg=20.0,
                 lambda_min_p_bootstrap=0.97,
                 sv_state: SvStateTracker | None = None,
                 nl_diag: NlDiagLogger | None = None):
        self.frac_threshold = frac_threshold    # |N1_frac| < this to fix
        self.sigma_threshold = sigma_threshold  # sigma_N1 < this to fix
        # AR-specific elevation mask.  Separate from the PPP measurement
        # mask (ELEV_MASK, 10°) — low-elev SVs still contribute
        # pseudorange/phase to the float filter but are excluded from
        # integer-ambiguity resolution.  RTKLIB calls this `arelmask`
        # and recommends it as the primary defense against wrong-
        # integer poisoning by multipath-prone low-elev satellites.
        # 20° matches PRIDE-PPPAR's published partial-AR cutoff and
        # is the most-cited value in the PAR literature (see
        # doi:10.3390/rs15133319, doi:10.1007/s10291-015-0473-1).
        self.ar_elev_mask_deg = ar_elev_mask_deg
        # Corner-margin gate on the rounding path: reject when
        # (frac/frac_cap) + (sigma/sigma_cap) ≥ corner_margin_sum.
        # 2.0 = top-right corner of the rectangle (= fully marginal).
        # Default 1.6 excludes the top-right ~20 % of the accept region
        # without tightening either threshold.  See E26 2026-04-17:
        # frac=0.100, sigma=0.114 → 1.95 (accepted under old logic,
        # rejected under margin).
        self.corner_margin_sum = corner_margin_sum
        self._fixed = {}  # sv -> {'n1': int, 'a_if_fixed': float}
        self.last_ratio = 0.0   # LAMBDA ratio test value (0 = not attempted)
        self.last_method = ""   # "lambda" or "rounding"
        self.last_success_rate = 0.0  # bootstrap success rate (0 = not computed)
        # LAMBDA P_bootstrap threshold.  Classical PPP-AR literature uses 0.999,
        # but day0419f NL_DIAG data (see project_nl_diag_classification_20260419
        # and project_ptpmon_nl_diag_result_20260419) showed P saturates at
        # 0.96-0.99 for n=4 batches on L5-fleet and ptpmon runs regardless of
        # per-SV σ tightness — a mechanical ceiling.  Lowering the threshold
        # to 0.97 lets LAMBDA succeed without the ratio test being loosened;
        # FFRT (active via ratio_threshold=None, P_fail=0.001) still provides
        # the primary reliability guard.
        self._lambda_min_p_bootstrap = float(lambda_min_p_bootstrap)
        # Anti-lock-in: after PFR (or any external agent) unfixes an SV,
        # skip it from candidates for this many epochs so the next NL
        # attempt doesn't immediately propose the same wrong integer.
        # Classic "fix-and-hold lock-in" mitigation — see rtklibexplorer
        # 2016/2021 posts and Laurichesse 2025 PPP-AR residual-monitoring
        # paper.
        self._blacklist_epochs = int(blacklist_epochs)
        self._blacklist = {}  # sv -> epoch_until
        self._epoch = 0
        # Optional tracker.  NL fix → WL_FIXED → NL_SHORT_FIXED.
        # NL unfix → any NL state → FLOAT.
        self._sv_state = sv_state
        # Optional per-attempt diagnostic.  When present, emits one
        # [NL_DIAG] line per SV per attempt + a [NL_DIAG_BATCH] line
        # per LAMBDA attempt.  Caller toggles via --nl-diag.
        self._nl_diag = nl_diag

    def tick(self):
        """Advance the resolver's epoch counter — call once per observation epoch.

        Used to drive blacklist expiry; no effect on the float filter state.
        """
        self._epoch += 1

    def blacklist(self, sv, epochs=None):
        """Mark an SV as temporarily ineligible for NL fixing."""
        n = self._blacklist_epochs if epochs is None else int(epochs)
        self._blacklist[sv] = self._epoch + n

    def is_blacklisted(self, sv):
        until = self._blacklist.get(sv)
        if until is None:
            return False
        if self._epoch >= until:
            del self._blacklist[sv]
            return False
        return True

    def attempt(self, filt, mw_tracker, elevations=None,
                ar_phase_bias_ok=None):
        """Try to fix ambiguities in the PPPFilter.

        Also re-constrains already-fixed ambiguities every epoch to prevent
        drift from process noise.

        Args:
            filt: PPPFilter instance with .x, .P, .sv_to_idx
            mw_tracker: MelbourneWubbenaTracker with fixed N_WL values
            elevations: optional dict {sv: elev_deg}.  SVs below
                self.ar_elev_mask_deg are excluded from NL-fix
                candidacy (they remain in the float filter for
                pseudorange/phase observations).  If None, the gate
                is skipped — same as ar_elev_mask_deg=0.
            ar_phase_bias_ok: optional dict {sv: bool}.  True iff the
                active SSR stream(s) provide matched phase biases for
                both signals of this SV's IF combination.  SVs flagged
                False are excluded from integer-fix candidacy (their
                float IF ambiguity carries an unknown phase-bias offset,
                so any "integer" fix lands on a biased value and
                poisons the position solution — most often as altitude
                bias).  They still contribute PR to the EKF.  If None,
                the gate is skipped — legacy callers unaffected.

        Returns:
            dict of newly fixed satellites: {sv: n1_int}
        """
        newly_fixed = {}
        elevations = elevations or {}
        elevations_for_diag = elevations  # alias for clarity below
        diag = self._nl_diag
        if diag is not None:
            diag.begin(self._epoch)
            wl_fixed_count = mw_tracker.n_fixed
        else:
            wl_fixed_count = None

        # Re-constrain already-fixed ambiguities every epoch.  Note:
        # the elevation gate intentionally does NOT apply here — a
        # fix that was made at high elevation stays valid as the SV
        # sets.  It can still be unfixed by cycle-slip flush or PFR.
        for sv, fix_info in list(self._fixed.items()):
            amb_idx = filt.sv_to_idx.get(sv)
            if amb_idx is None:
                continue
            si = N_BASE + amb_idx
            if si < len(filt.x):
                self._apply_fix(filt, si, fix_info['a_if_fixed'])

        # Collect WL-fixed, not-yet-NL-fixed, not-blacklisted candidates
        # above the AR elevation mask.
        cands = []  # list of (sv, amb_idx, si, f1, f2, n_wl, lambda_wl, lambda_nl, alpha2)
        skipped_by_elev = 0
        for sv, amb_idx in list(filt.sv_to_idx.items()):
            if sv in self._fixed:
                continue
            elev = elevations.get(sv)
            if self.is_blacklisted(sv):
                if diag is not None:
                    bl_rem = max(0, (self._blacklist.get(sv) or self._epoch) - self._epoch)
                    diag.record(sv=sv, elev_deg=elev,
                                wl_fixed_count=wl_fixed_count,
                                blacklist_remaining=bl_rem,
                                result=RESULT_SKIP_BLACKLIST)
                continue
            if (self.ar_elev_mask_deg > 0 and elev is not None
                    and elev < self.ar_elev_mask_deg):
                skipped_by_elev += 1
                if diag is not None:
                    diag.record(sv=sv, elev_deg=elev,
                                wl_fixed_count=wl_fixed_count,
                                result=RESULT_SKIP_ELEV,
                                reason=f"below {self.ar_elev_mask_deg:.0f}° AR mask")
                continue
            n_wl = mw_tracker.get_wl(sv)
            if n_wl is None:
                if diag is not None:
                    diag.record(sv=sv, elev_deg=elev,
                                wl_fixed_count=wl_fixed_count,
                                result=RESULT_SKIP_NO_WL)
                continue
            if (ar_phase_bias_ok is not None
                    and not ar_phase_bias_ok.get(sv, True)):
                if diag is not None:
                    diag.record(sv=sv, elev_deg=elev,
                                wl_fixed_count=wl_fixed_count,
                                result=RESULT_SKIP_NO_PHASE_BIAS)
                continue
            freqs = mw_tracker.get_freqs(sv)
            if freqs is None:
                if diag is not None:
                    diag.record(sv=sv, elev_deg=elev,
                                wl_fixed_count=wl_fixed_count,
                                result=RESULT_SKIP_NO_FREQS)
                continue
            f1, f2 = freqs
            lambda_wl = C / (f1 - f2)
            lambda_nl = C / (f1 + f2)
            alpha2 = f2**2 / (f1**2 - f2**2)
            si = N_BASE + amb_idx
            if si >= len(filt.x):
                continue
            cands.append((sv, amb_idx, si, f1, f2, n_wl,
                          lambda_wl, lambda_nl, alpha2))
        if skipped_by_elev > 0:
            self.last_skipped_by_elev = skipped_by_elev

        # Pre-screen: loose reject for obviously unconverged ambiguities
        screened = []
        for c in cands:
            sv, amb_idx, si, f1, f2, n_wl, lambda_wl, lambda_nl, alpha2 = c
            a_if_float = filt.x[si]
            sigma_a = math.sqrt(filt.P[si, si]) if si < filt.P.shape[0] else 999.0
            n1_float = (a_if_float - alpha2 * lambda_wl * n_wl) / lambda_nl
            n1_frac = abs(n1_float - round(n1_float))
            sigma_n1 = sigma_a / lambda_nl
            if diag is not None:
                diag.record(sv=sv, elev_deg=elevations_for_diag.get(sv),
                            wl_fixed_count=wl_fixed_count,
                            n1_frac=n1_frac, sigma_n1_cyc=sigma_n1,
                            result=RESULT_CAND)
            if n1_frac < 0.25 and sigma_n1 < 1.0:
                screened.append(c)
            elif diag is not None:
                # Pre-screen rejection: overwrite the CAND result.
                diag.update(sv, result=RESULT_SKIP_PRESCREEN,
                            reason=f"frac={n1_frac:.3f} sigma={sigma_n1:.3f}")

        # Try LAMBDA when >= 4 candidates pass pre-screen
        if len(screened) >= 4:
            newly_fixed = self._attempt_lambda(filt, screened)
            if newly_fixed:
                if diag is not None:
                    diag.emit()
                return newly_fixed

        # Fallback: per-satellite rounding for < 4 or if LAMBDA failed
        self.last_method = "rounding"
        for c in screened:
            sv, amb_idx, si, f1, f2, n_wl, lambda_wl, lambda_nl, alpha2 = c
            a_if_float = filt.x[si]
            sigma_a = math.sqrt(filt.P[si, si]) if si < filt.P.shape[0] else 999.0
            n1_float = (a_if_float - alpha2 * lambda_wl * n_wl) / lambda_nl
            n1_frac = abs(n1_float - round(n1_float))
            sigma_n1 = sigma_a / lambda_nl

            # Basic rectangle gate: both metrics must be under their caps.
            in_rect = (n1_frac < self.frac_threshold
                       and sigma_n1 < self.sigma_threshold)
            # Corner-margin gate: reject cases that sit near the top-right
            # of the accept rectangle, where BOTH frac and sigma are
            # simultaneously marginal (the classic "barely-everywhere"
            # premature fix).  The sum-of-normalized-ratios must have
            # some headroom from 2.0 (the corner).  1.6 excludes the
            # top-right ~20% of the rectangle.  Example: frac=0.10,
            # sigma=0.114 → 1.0 + 0.95 = 1.95 > 1.6 → reject.
            corner_ok = (
                (n1_frac / max(self.frac_threshold, 1e-9))
                + (sigma_n1 / max(self.sigma_threshold, 1e-9))
                < self.corner_margin_sum
            )
            if diag is not None:
                # Corner-margin sum is reported regardless of accept/reject;
                # useful for spotting marginal fixes that landed just inside
                # the envelope (common precursor to false-fix rejection later).
                corner_sum = (
                    (n1_frac / max(self.frac_threshold, 1e-9))
                    + (sigma_n1 / max(self.sigma_threshold, 1e-9))
                )
                diag.update(sv, corner_margin_sum=corner_sum)
            if in_rect and corner_ok:
                n1_int = round(n1_float)
                a_if_fixed = lambda_nl * n1_int + alpha2 * lambda_wl * n_wl
                self._apply_fix(filt, si, a_if_fixed)
                # Store fix-time NL quality alongside the integer so later
                # diagnostics can correlate PFR unfix events with how
                # marginal the original NL fix was.
                self._fixed[sv] = {
                    'n1': n1_int,
                    'a_if_fixed': a_if_fixed,
                    'fix_n1_frac': float(n1_frac),
                    'fix_sigma_n1': float(sigma_n1),
                    'fix_method': 'rounding',
                }
                newly_fixed[sv] = n1_int
                log.info("NL fixed (rounding): %s N1=%d (frac=%.3f, "
                         "sigma_N1=%.3f, A_IF: %.4f → %.4f m)",
                         sv, n1_int, n1_frac, sigma_n1,
                         a_if_float, a_if_fixed)
                self._note_nl_fix(sv, az_deg=None, elev_deg=None,
                                  reason=f"rounding (frac={n1_frac:.3f}, σ={sigma_n1:.3f})")
                if diag is not None:
                    diag.update(sv, result=RESULT_FIXED_ROUNDING)
            elif diag is not None:
                diag.update(
                    sv,
                    result=RESULT_REJ_CORNER if in_rect else RESULT_REJ_RECT,
                    reason=("corner" if in_rect else f"frac={n1_frac:.3f} sigma={sigma_n1:.3f}"),
                )

        if diag is not None:
            diag.emit()
        return newly_fixed

    def _attempt_lambda(self, filt, screened):
        """Try LAMBDA resolution on screened candidates.

        Args:
            filt: PPPFilter instance
            screened: list of (sv, amb_idx, si, f1, f2, n_wl,
                      lambda_wl, lambda_nl, alpha2) tuples

        Returns:
            dict of newly fixed satellites, or empty dict if failed
        """
        # Build NL float vector and covariance from filter state
        svs = []
        n1_floats = []
        state_indices = []
        params = []  # (lambda_wl, lambda_nl, alpha2, n_wl) per sat

        for c in screened:
            sv, amb_idx, si, f1, f2, n_wl, lambda_wl, lambda_nl, alpha2 = c
            a_if_float = filt.x[si]
            n1_float = (a_if_float - alpha2 * lambda_wl * n_wl) / lambda_nl
            svs.append(sv)
            n1_floats.append(n1_float)
            state_indices.append(si)
            params.append((lambda_wl, lambda_nl, alpha2, n_wl))

        n1_vec = np.array(n1_floats)
        si_arr = np.array(state_indices)

        # Extract NL covariance from IF ambiguity covariance
        # Qa_IF = P[si, si] submatrix; Qa_NL = Qa_IF / (lambda_nl^2)
        # When all sats have the same lambda_nl (same constellation+freq),
        # this is exact.  For mixed constellations, scale per element.
        n_amb = len(svs)
        Qa_nl = np.zeros((n_amb, n_amb))
        for i in range(n_amb):
            for j in range(n_amb):
                Qa_nl[i, j] = filt.P[si_arr[i], si_arr[j]] / (
                    params[i][1] * params[j][1])  # lambda_nl_i * lambda_nl_j

        # Compute bootstrap success rate for diagnostics before resolving
        eigvals = np.linalg.eigvalsh(Qa_nl)
        Qa_reg = Qa_nl.copy()
        if eigvals.min() <= 0:
            Qa_reg += np.eye(n_amb) * max(1e-10, -eigvals.min() * 2)
        _, _, D_diag = lambda_decorrelate(Qa_reg)
        self.last_success_rate = bootstrap_success_rate(D_diag)

        # ratio_threshold=None → FFRT-adaptive critical value at P_fail=0.001
        # (Wang & Feng 2013/2016).  The old fixed 2.0 was systematically
        # too loose for small n_amb, which is the regime we're in on
        # GAL-only: ratio=2.0 at n_amb=5 corresponds to ~5% failure rate,
        # not the intended 0.1%.
        # min_fixed=3 enables one extra PAR retry for n=4 batches: if full
        # 4-SV attempt fails (bootstrap, <2 candidates, or FFRT ratio),
        # drop worst and try 3.  Day0419g data showed REJECT_LAMBDA_RATIO
        # dominant at n=4 after the P_bootstrap fix — PAR at min_fixed=4
        # couldn't retry the subset.  FFRT's critical ratio scales with n,
        # so the 0.001 failure target still holds at 3.
        fixed_vec, n_fixed, ratio, mask = lambda_resolve(
            n1_vec, Qa_nl, ratio_threshold=None, min_fixed=3,
            min_success_rate=self._lambda_min_p_bootstrap)

        if fixed_vec is None:
            self.last_ratio = ratio
            if self.last_success_rate < self._lambda_min_p_bootstrap:
                log.debug("LAMBDA skipped: bootstrap P=%.4f (need %.3f), "
                          "%d candidates", self.last_success_rate,
                          self._lambda_min_p_bootstrap, n_amb)
                if self._nl_diag is not None:
                    self._nl_diag.set_lambda_batch_result(
                        svs, ratio=ratio, p_bootstrap=self.last_success_rate,
                        result=RESULT_REJ_LAMBDA_BOOTSTRAP,
                    )
                    self._nl_diag.set_lambda_batch_summary(
                        n=n_amb, ratio=ratio, p_bootstrap=self.last_success_rate,
                        result=RESULT_REJ_LAMBDA_BOOTSTRAP,
                    )
            else:
                if self._nl_diag is not None:
                    self._nl_diag.set_lambda_batch_result(
                        svs, ratio=ratio, p_bootstrap=self.last_success_rate,
                        result=RESULT_REJ_LAMBDA_RATIO,
                    )
                    self._nl_diag.set_lambda_batch_summary(
                        n=n_amb, ratio=ratio, p_bootstrap=self.last_success_rate,
                        result=RESULT_REJ_LAMBDA_RATIO,
                    )
            return {}

        self.last_ratio = ratio
        self.last_method = "lambda"

        # Position displacement check: save float state, apply fixes,
        # check if position jumps too far, roll back if it does.
        float_pos = filt.x[:3].copy()
        float_sigma = math.sqrt(max(filt.P[0, 0], filt.P[1, 1], filt.P[2, 2]))
        saved_x = filt.x.copy()
        saved_P = filt.P.copy()

        newly_fixed = {}
        for i in range(n_amb):
            if not mask[i]:
                continue
            sv = svs[i]
            n1_int = int(fixed_vec[i])
            lambda_wl, lambda_nl, alpha2, n_wl = params[i]
            a_if_fixed = lambda_nl * n1_int + alpha2 * lambda_wl * n_wl
            si = state_indices[i]
            self._apply_fix(filt, si, a_if_fixed)
            newly_fixed[sv] = (n1_int, a_if_fixed, si)

        # Check position displacement
        fixed_pos = filt.x[:3]
        displacement_m = np.linalg.norm(fixed_pos - float_pos)
        max_displacement = max(3.0 * float_sigma, 1.0)  # at least 1m floor

        if displacement_m > max_displacement:
            # Roll back — integers moved position too far
            log.warning("LAMBDA rejected: position displacement %.1fm "
                        "exceeds %.1fm (3σ=%.1fm). Rolling back %d fixes.",
                        displacement_m, max_displacement,
                        3.0 * float_sigma, len(newly_fixed))
            filt.x = saved_x
            filt.P = saved_P
            if self._nl_diag is not None:
                self._nl_diag.set_lambda_batch_result(
                    svs, ratio=ratio, p_bootstrap=self.last_success_rate,
                    result=RESULT_REJ_LAMBDA_DISPLACEMENT,
                )
                self._nl_diag.set_lambda_batch_summary(
                    n=n_amb, ratio=ratio, p_bootstrap=self.last_success_rate,
                    result=RESULT_REJ_LAMBDA_DISPLACEMENT,
                )
            return {}

        # Commit the fixes (store fix-time quality for PFR diagnostics)
        for sv, (n1_int, a_if_fixed, si) in newly_fixed.items():
            self._fixed[sv] = {
                'n1': n1_int,
                'a_if_fixed': a_if_fixed,
                'fix_ratio': float(ratio),
                'fix_success_rate': float(self.last_success_rate),
                'fix_displacement_m': float(displacement_m),
                'fix_method': 'lambda',
            }
            log.info("NL fixed (LAMBDA): %s N1=%d (ratio=%.1f, P=%.4f, "
                     "Δpos=%.1fm, %d/%d fixed, A_IF: %.4f → %.4f m)",
                     sv, n1_int, ratio, self.last_success_rate,
                     displacement_m, n_fixed, n_amb,
                     filt.x[si], a_if_fixed)
            self._note_nl_fix(
                sv, az_deg=None, elev_deg=None,
                reason=f"LAMBDA ratio={ratio:.1f} P={self.last_success_rate:.3f}",
            )

        if self._nl_diag is not None:
            # Mark fixed SVs as FIXED_LAMBDA; any SVs in the batch that
            # weren't fixed (partial-AR mask bit=0) stay at their current
            # record (CAND).  Batch summary captures the ratio/P/outcome.
            self._nl_diag.set_lambda_batch_result(
                list(newly_fixed.keys()),
                ratio=ratio, p_bootstrap=self.last_success_rate,
                result=RESULT_FIXED_LAMBDA,
            )
            self._nl_diag.set_lambda_batch_summary(
                n=n_amb, ratio=ratio, p_bootstrap=self.last_success_rate,
                result=RESULT_FIXED_LAMBDA,
            )

        return {sv: info[0] for sv, info in newly_fixed.items()}

    def _apply_fix(self, filt, state_idx, fixed_value):
        """Constrain a PPPFilter ambiguity state to a fixed value.

        Uses a tight pseudo-measurement update: H = [0...1...0],
        z = fixed_value, R = (0.001 m)^2.  This pulls the state to
        the fixed value and collapses its covariance without altering
        the filter's structure (the state remains, just tightly constrained).
        """
        n = len(filt.x)
        H = np.zeros((1, n))
        H[0, state_idx] = 1.0
        z = np.array([fixed_value - filt.x[state_idx]])
        R = np.array([[0.001**2]])  # 1 mm sigma — effectively fixed

        S = H @ filt.P @ H.T + R
        K = filt.P @ H.T / S[0, 0]
        filt.x = filt.x + K[:, 0] * z[0]
        I_KH = np.eye(n) - K @ H
        filt.P = I_KH @ filt.P @ I_KH.T + K @ R @ K.T
        filt.P = 0.5 * (filt.P + filt.P.T)

    def is_fixed(self, sv):
        return sv in self._fixed

    def _note_nl_fix(self, sv, *, az_deg=None, elev_deg=None, reason=""):
        """Drive the per-SV tracker on a successful NL fix.

        Called by both LAMBDA and rounding fix paths.  Transitions
        WL_FIXED → NL_SHORT_FIXED.  If the SV isn't in WL_FIXED (e.g.
        we re-fixed an SV that was still in NL_SHORT_FIXED because the
        filter constraint drifted and re-converged), the transition
        is a no-op at the tracker (self-edge).
        """
        if self._sv_state is None:
            return
        cur = self._sv_state.state(sv)
        if cur is SvAmbState.NL_SHORT_FIXED or cur is SvAmbState.NL_LONG_FIXED:
            return  # already counted; don't re-log per-epoch reconstraints
        if cur is SvAmbState.WL_FIXED:
            self._sv_state.transition(
                sv, SvAmbState.NL_SHORT_FIXED,
                epoch=self._epoch, reason="nl_fix: " + reason,
                az_deg=az_deg, elev_deg=elev_deg,
            )

    def unfix(self, sv):
        """Remove a fix (e.g. after cycle slip detected).

        Drives tracker back to FLOAT when an SV is actively unfixed
        here (e.g. by PFR L1 or a future Job-A caller that wants the
        NL resolver to forget the integer).  false-fix itself transitions
        the tracker directly, so it should call unfix() after; the
        no-op-on-self-edge rule keeps the log line count correct.
        """
        self._fixed.pop(sv, None)
        if self._sv_state is not None:
            cur = self._sv_state.state(sv)
            if cur in (SvAmbState.NL_SHORT_FIXED, SvAmbState.NL_LONG_FIXED):
                self._sv_state.transition(
                    sv, SvAmbState.FLOAT,
                    epoch=self._epoch, reason="nl_resolver.unfix",
                )
            # If cur is already FLOAT (e.g. false-fix got here first), nothing
            # to do.  If WL_FIXED/SQUELCHED: also nothing; unfix
            # doesn't imply an MW reset.

    def unfix_all(self, filt, inflate_sigma_m=100.0):
        """Unfix every NL-fixed ambiguity and inflate its covariance.

        Used by the post-fix residual monitor for Level 2 recovery —
        preserves PPPFilter position, clock, ISB, ZTD, and MW tracker
        history.  The float ambiguity estimate is retained (the state
        value), but its covariance is inflated so subsequent phase
        observations can pull it to the correct integer.
        """
        svs = list(self._fixed.keys())
        for sv in svs:
            filt.inflate_ambiguity(sv, sigma_m=inflate_sigma_m)
        self._fixed.clear()
        if self._sv_state is not None:
            for sv in svs:
                cur = self._sv_state.state(sv)
                if cur in (SvAmbState.NL_SHORT_FIXED, SvAmbState.NL_LONG_FIXED):
                    self._sv_state.transition(
                        sv, SvAmbState.FLOAT,
                        epoch=self._epoch, reason="nl_resolver.unfix_all",
                    )
        return svs

    def integrality(self, filt, mw_tracker):
        """Compute corrected integrality metric for all satellites.

        Returns list of (sv, n1_frac, sigma_n1, fixed) for diagnostics.
        """
        results = []
        for sv, amb_idx in filt.sv_to_idx.items():
            n_wl = mw_tracker.get_wl(sv)
            if n_wl is None:
                continue

            freqs = mw_tracker.get_freqs(sv)
            if freqs is None:
                continue
            f1, f2 = freqs

            lambda_wl = C / (f1 - f2)
            lambda_nl = C / (f1 + f2)
            alpha2 = f2**2 / (f1**2 - f2**2)

            si = N_BASE + amb_idx
            if si >= len(filt.x):
                continue

            a_if_float = filt.x[si]
            sigma_a = math.sqrt(filt.P[si, si]) if si < filt.P.shape[0] else 999.0

            n1_float = (a_if_float - alpha2 * lambda_wl * n_wl) / lambda_nl
            n1_frac = n1_float - round(n1_float)
            sigma_n1 = sigma_a / lambda_nl

            results.append((sv, n1_frac, sigma_n1, sv in self._fixed))

        return results

    @property
    def n_fixed(self):
        return len(self._fixed)

    def summary(self):
        s = f"NL: {self.n_fixed} fixed"
        if self.last_ratio > 0:
            s += f" R={self.last_ratio:.1f}"
        if self.last_success_rate > 0:
            s += f" P={self.last_success_rate:.3f}"
        return s

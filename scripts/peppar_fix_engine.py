#!/usr/bin/env python3
"""peppar-fix-engine: Unified GNSS clock engine.

Single process with two phases:

Phase 1 — Bootstrap (no known position):
  PPPFilter estimates position from scratch. Solution logged.
  When converged: save position, transition to phase 2.
  Skipped if --position-file points to an existing converged position.

Phase 2 — Steady state (position known):
  FixedPosFilter estimates clock. Solution logged.
  Optional consumers:
  - DO servo (--servo /dev/ptp0): disciplines oscillator via PHC adjfine
  - NTRIP caster (--caster :2102): streams RTCM to clients (future)

Usage:
    peppar-fix-engine --serial /dev/gnss-top --ntrip-conf ntrip.conf \\
        --servo /dev/ptp_i226 --pps-pin 1 \\
        --out solution.csv --systems gps,gal,bds

    # Bootstrap only (no servo):
    peppar-fix-engine --serial /dev/gnss-top --ntrip-conf ntrip.conf \\
        --out bootstrap.csv
"""

import argparse
from collections import deque
import csv
from dataclasses import dataclass
import json
import logging
import math
import os
import queue
import signal
from statistics import pvariance
import sys
import threading
import time
import tomllib
from datetime import datetime, timezone, timedelta

import numpy as np

from solve_pseudorange import C, ecef_to_lla, lla_to_ecef
from solve_ppp import PPPFilter, FixedPosFilter, ls_init, N_BASE, SIGMA_P_IF, IDX_ZTD as PPP_IDX_ZTD
from peppar_fix.obs_routing import obs_for_position
from solid_tide import solid_tide_displacement, sun_pos_ecef
import peer_publisher
from antex import ANTEXParser, compute_pcv_correction
from peppar_fix.arp_blend import bayesian_arp_blend
from peppar_fix.bootstrap_gate import (
    residuals_consistent, nav2_agrees, scrub_for_retry,
)
from broadcast_eph import BroadcastEphemeris
from ssr_corrections import SSRState, RealtimeCorrections
from ppp_ar import MelbourneWubbenaTracker, NarrowLaneResolver
from peppar_fix.cycle_slip import (
    CycleSlipMonitor, SlipEvent, flush_sv_phase, is_pr_only_slip,
)
from peppar_fix.sv_state import SvAmbState, SvStateTracker
from peppar_fix.false_fix_monitor import FalseFixMonitor
from peppar_fix.if_step_monitor import IfStepMonitor
from peppar_fix.setting_sv_drop_monitor import SettingSvDropMonitor
from peppar_fix.fix_set_integrity_monitor import FixSetIntegrityMonitor
from peppar_fix.wl_drift_monitor import WlDriftMonitor
from peppar_fix.gf_step_monitor import GfStepMonitor, gf_phase_m
from peppar_fix.second_opinion_pos_monitor import (
    SecondOpinionPosMonitor,
    select_reset_target,
)
from peppar_fix.wl_phase_admission_gate import WlPhaseAdmissionGate
from peppar_fix.wl_readmission_gate import WlReAdmissionGate
from peppar_fix.anchoring_sv_promoter import AnchoringSvPromoter
from peppar_fix.nl_diag import NlDiagLogger
from peppar_fix.saastamoinen import metar_to_init_ztd_residual
from peppar_fix.metar import (
    DEFAULT_STATION, MetarReadError,
    fetch_latest_metar, metar_age_seconds,
)


# Cycle-slip CSV sink, shared between Phase-1 (run_bootstrap) and Phase-2
# (AntPosEstThread).  Opened in main() if --slip-log is set; both the
# bootstrap and steady-state monitors write to the same file since the
# phases run in series.
_SLIP_CSV_FILE = None
_SLIP_CSV_WRITER = None


def _seed_ztd_from_metar(args, lat_deg, alt_m, log):
    """Compute init_ztd_m + init_ztd_sigma_m from latest METAR.

    Returns (init_ztd_m, init_ztd_sigma_m).  On any METAR-fetch or
    Saastamoinen failure, logs a warning and returns (0.0, 0.2) —
    equivalent to leaving --init-ztd-from-met unset.

    This deliberately does NOT abort the engine: METAR is bonus
    information; the filter without it converges as it always has
    (just slowly under --pin-position).  Operators see a single
    [INIT_ZTD] log line either way.
    """
    station = getattr(args, 'init_ztd_station', None) or DEFAULT_STATION
    # When --init-ztd-sigma-mm isn't specified, the METAR-auto path
    # uses 50 mm (captures METAR uncertainty + spatial coherence +
    # Saastamoinen model error).  The manual --init-ztd-mm path
    # interprets unset sigma as the legacy 500 mm wide prior.
    sigma_mm_arg = getattr(args, 'init_ztd_sigma_mm', None)
    sigma_mm = float(sigma_mm_arg) if sigma_mm_arg is not None else 50.0
    sigma_m = sigma_mm * 1e-3
    try:
        rec = fetch_latest_metar(station)
    except MetarReadError as exc:
        log.warning("[INIT_ZTD] METAR fetch failed (%s); falling back "
                    "to ztd=0 σ=200mm default", exc)
        return 0.0, 0.2
    age_s = metar_age_seconds(rec)
    if age_s > 7200:  # 2 h
        log.warning("[INIT_ZTD] METAR stale (age=%.0f min); falling "
                    "back to ztd=0 σ=200mm default", age_s / 60.0)
        return 0.0, 0.2
    try:
        residual_m, diag = metar_to_init_ztd_residual(
            temp_C=rec['temp_C'],
            dewp_C=rec['dewp_C'],
            altim_hPa=rec['altim_hPa'],
            lat_deg=lat_deg,
            elev_m=alt_m,
        )
    except (ValueError, ZeroDivisionError) as exc:
        log.warning("[INIT_ZTD] Saastamoinen compute failed (%s); "
                    "falling back to ztd=0 σ=200mm default", exc)
        return 0.0, 0.2
    log.info(
        "[INIT_ZTD] %s T=%.1fC dewp=%.1fC altim=%.1fhPa age=%.0fmin "
        "→ Pstation=%.1fhPa e=%.1fhPa ZHD=%.3fm ZWD=%.3fm ZTD=%.3fm "
        "residual=%+.0fmm σ=%.0fmm",
        rec.get('station', station),
        diag['temp_C'], diag['dewp_C'], diag['altim_hPa'], age_s / 60.0,
        diag['P_station_hPa'], diag['e_hPa'], diag['zhd_m'],
        diag['zwd_m'], diag['ztd_m'], residual_m * 1000.0, sigma_mm,
    )
    return residual_m, sigma_m


def _periodic_ztd_tie(filt, args, lat_deg, alt_m, n_epochs, log):
    """Refresh METAR + apply soft ZTD prior to bound per-SV bias absorption.

    Called every --ztd-tie-interval-s epochs (default 300, matches
    METAR cadence).  Fetches current weather, runs Saastamoinen for
    the receiver site, and applies a soft Kalman update pulling the
    filter's residual ZTD toward METAR truth with σ ≈ 100 mm.

    METAR fetch failure is non-fatal: log + skip this cycle.  ZTD
    state continues evolving from observations until the next tie.
    """
    sigma_mm = float(getattr(args, 'ztd_tie_sigma_mm', 100.0))
    sigma_m = sigma_mm * 1e-3
    if sigma_m <= 0:
        return
    station = getattr(args, 'init_ztd_station', None) or DEFAULT_STATION
    try:
        rec = fetch_latest_metar(station)
    except MetarReadError as exc:
        log.warning("[ZTD_TIE] epoch=%d METAR fetch failed (%s); skipping tie",
                    n_epochs, exc)
        return
    age_s = metar_age_seconds(rec)
    if age_s > 7200:
        log.warning("[ZTD_TIE] epoch=%d METAR stale (%.0f min); skipping tie",
                    n_epochs, age_s / 60.0)
        return
    try:
        target_m, diag = metar_to_init_ztd_residual(
            temp_C=rec['temp_C'], dewp_C=rec['dewp_C'],
            altim_hPa=rec['altim_hPa'],
            lat_deg=lat_deg, elev_m=alt_m,
        )
    except (ValueError, ZeroDivisionError) as exc:
        log.warning("[ZTD_TIE] epoch=%d Saastamoinen failed (%s); skipping tie",
                    n_epochs, exc)
        return
    pre_ztd_m = float(filt.x[filt.IDX_ZTD])
    pre_sigma_m = math.sqrt(max(0.0,
        float(filt.P[filt.IDX_ZTD, filt.IDX_ZTD])))
    filt.apply_ztd_tie(sigma_m, target_m=target_m)
    post_ztd_m = float(filt.x[filt.IDX_ZTD])
    post_sigma_m = math.sqrt(max(0.0,
        float(filt.P[filt.IDX_ZTD, filt.IDX_ZTD])))
    log.info(
        "[ZTD_TIE] epoch=%d %s age=%.0fmin target=%+.0fmm σ_tie=%.0fmm "
        "ZTD %+.0f→%+.0fmm (Δ=%+.0f) σ %.0f→%.0fmm",
        n_epochs, rec.get('station', station), age_s / 60.0,
        target_m * 1000.0, sigma_mm,
        pre_ztd_m * 1000.0, post_ztd_m * 1000.0,
        (post_ztd_m - pre_ztd_m) * 1000.0,
        pre_sigma_m * 1000.0, post_sigma_m * 1000.0,
    )


def _open_slip_csv(path):
    """Open the slip CSV file (header written once); return the csv.writer."""
    global _SLIP_CSV_FILE, _SLIP_CSV_WRITER
    if _SLIP_CSV_WRITER is not None:
        return _SLIP_CSV_WRITER
    _SLIP_CSV_FILE = open(path, 'w', newline='')
    _SLIP_CSV_WRITER = csv.writer(_SLIP_CSV_FILE)
    _SLIP_CSV_WRITER.writerow(SlipEvent.csv_header())
    _SLIP_CSV_FILE.flush()
    return _SLIP_CSV_WRITER


def _slip_csv_writer():
    """Return the slip CSV writer, or None if --slip-log was not set."""
    return _SLIP_CSV_WRITER


def _compute_sv_elevations(filt, corrections, observations, gps_time):
    """Per-SV elevation in degrees from the filter's current position.

    Used by CycleSlipMonitor for slip-event tagging and by
    NarrowLaneResolver for the AR elevation mask.

    Returns {} if the filter has no position yet or corrections can't
    provide satellite positions.  Missing elevations don't change slip
    detection or AR gating behavior — they just mean log output is
    less informative and the AR elev mask is inactive for SVs lacking
    a sat-position lookup.
    """
    if filt is None or not hasattr(filt, 'x') or len(filt.x) < 3:
        return {}
    pos = filt.x[:3]
    out = {}
    for obs in observations:
        sv = obs['sv']
        sat_pos, _ = corrections.sat_position(sv, gps_time)
        if sat_pos is None:
            continue
        out[sv] = filt.compute_elevation(pos, sat_pos)
    return out


def _compute_sv_azimuths(filt, corrections, observations, gps_time):
    """Per-SV azimuth in degrees (clockwise from geodetic north).

    Same return-contract as _compute_sv_elevations: empty dict if
    filter has no position or corrections can't provide satellite
    positions.  Used by the Bead 4 AnchoringSvPromoter to measure
    accumulated sky motion since NL fix.
    """
    if filt is None or not hasattr(filt, 'x') or len(filt.x) < 3:
        return {}
    pos = filt.x[:3]
    out = {}
    for obs in observations:
        sv = obs['sv']
        sat_pos, _ = corrections.sat_position(sv, gps_time)
        if sat_pos is None:
            continue
        out[sv] = filt.compute_azimuth(pos, sat_pos)
    return out


def _compute_sv_ipp_szas(filt, azimuths, elevations, gps_time):
    """Per-SV solar-zenith-angle at the ionospheric pierce point.

    ~90° flags the terminator twilight band where dTEC/dt is
    largest.  Enables post-hoc analysis of whether slips
    concentrate under terminator conditions, per the 2026-04-24
    sunrise-slip-storm investigation.

    Empty dict when filter has no position.  Per-SV None when the
    SV has no computed azimuth/elevation yet (e.g., sat ephemeris
    unavailable this epoch).
    """
    if filt is None or not hasattr(filt, 'x') or len(filt.x) < 3:
        return {}
    try:
        from peppar_fix.ipp_sza import ipp_solar_zenith_deg
    except Exception:
        return {}
    try:
        from solve_pseudorange import ecef_to_lla as _e2l
        lat, lon, _alt = _e2l(
            float(filt.x[0]), float(filt.x[1]), float(filt.x[2]))
    except Exception:
        return {}
    utc = gps_time if isinstance(gps_time, datetime) else gps_time  # already UTC-aware
    out = {}
    for sv, az in azimuths.items():
        el = elevations.get(sv)
        if el is None or az is None:
            continue
        try:
            out[sv] = ipp_solar_zenith_deg(lat, lon, az, el, utc)
        except Exception:
            continue
    return out
from ntrip_client import NtripStream
from realtime_ppp import (
    serial_reader, ntrip_reader, QErrStore, Nav2PositionStore,
    Nav2SignalStore, NavClockStore, NavTimeGpsStore,
)
from peppar_fix.extint_reader import TimTm2Store
from ticc import Ticc
from peppar_fix import (
    CorrectionFreshnessGate,
    PositionWatchdog,
    PositionWatchdogProposed,
    StrictCorrelationGate,
    TimebaseRelationEstimator,
    PPPCalibration,
    CarrierPhaseTracker,
    estimator_sample_weight,
    estimate_correlation_confidence,
    match_pps_event_from_history,
)
from peppar_fix.event_time import PpsEvent
from peppar_fix.fault_injection import get_delay_injector, get_source_mute_controller
from peppar_fix.receiver import get_driver
from peppar_fix.receiver_state import save_position_to_receiver
from peppar_fix.states import (
    AntPosEst, AntPosEstState,
    DOFreqEst, DOFreqEstState,
    format_status,
)

log = logging.getLogger("peppar-fix")


# Saved-position trust gate.  When loading state/receivers/<uid>.json
# at startup, positions tagged with σ < this threshold are treated as
# "trusted" and the live-LS-fix validation step is skipped (fast
# warm-start path).  Above this, the loaded position is still useful
# as a Phase-2 seed but gets re-validated against a fresh LS fix
# (100 m separation gate) before Phase 2 starts.
_TRUSTED_POSITION_SIGMA_M = 10.0

# Phase-1 bootstrap save floor.  The Phase-1 CONVERGED gate accepts
# σ up to --sigma (default 3 m) plus W2 horizontal disagreement up to
# --bootstrap-nav2-horiz-m (default 5 m); it is therefore *not* a
# claim of sub-decimeter quality.  If a Phase-1 σ value were written
# verbatim into the state file, the next session's trust gate above
# would skip validation on a position whose actual confidence is
# meter-scale — fine until something physical changes between runs
# (antenna reorientation, rewire).  We deliberately store Phase-1
# saves *above* the trust threshold so the next session always
# re-validates against a fresh LS fix.  Phase-2 steady-state saves
# (gated by σ < 0.1 m) are written verbatim — those are the
# legitimate fast-start path.
_PHASE1_BOOTSTRAP_SAVE_FLOOR_M = _TRUSTED_POSITION_SIGMA_M + 1.0


@dataclass
class BootstrapResult:
    """Result from run_bootstrap — includes live objects for AntPosEstThread."""
    ecef: np.ndarray
    sigma_m: float
    ppp_filter: object = None       # PPPFilter instance (converged)
    mw_tracker: object = None       # MelbourneWubbenaTracker
    nl_resolver: object = None      # NarrowLaneResolver


class QErrTimescaleTracker:
    """Track CLOCK_MONOTONIC offset between TIM-TP and TICC chB streams.

    TIM-TP for PPS epoch N arrives at mono_A.  TICC chB for the same
    epoch arrives at mono_B.  The offset = mono_B - mono_A is nearly
    constant (~0.95s on most hosts, depending on USB latency).

    Rather than dynamically searching for the best TIM-TP match per
    pulse (fragile — can grab the adjacent TIM-TP when the offset
    drifts), this tracker:
    1. Bootstraps the offset from the first few chB/TIM-TP pairs
    2. Maintains it with an exponential moving average
    3. Uses the tracked offset to index directly into the TIM-TP
       stream for each chB event — deterministic, no search ambiguity

    Since TIM-TP samples are ~1 second apart, the offset only needs
    to be accurate within ±0.3s to unambiguously pick the right one.
    """

    def __init__(self, initial_offset_s=0.95, alpha=0.005):
        self.offset_s = initial_offset_s
        self.calibrated = False
        self._alpha = alpha  # EMA weight (slow tracking)
        self._n = 0
        self._calibration_offsets = []
        self._logged = False
        self._last_consumed_mono = None

    def match_and_update(self, chb_recv_mono, qerr_store):
        """Find qerr for a chB event and update the offset estimate.

        Each TIM-TP sample is consumed at most once.  Two consecutive
        chB events cannot match the same TIM-TP — if the best match
        is the one we already consumed, return None for this epoch.

        Returns (qerr_ns, offset_s) or (None, None).
        """
        tol = 0.4 if not self.calibrated else 0.15
        qerr_ns, match_offset_s = qerr_store.match_pps_mono(
            chb_recv_mono,
            expected_offset_s=self.offset_s,
            tolerance_s=tol)

        if qerr_ns is None:
            return None, None

        # Deduplicate: the matched TIM-TP's host_time is approximately
        # chb_recv_mono - match_offset_s.  If it's within 0.1s of the
        # last consumed TIM-TP, it's the same sample — skip it.
        matched_tim_tp_mono = chb_recv_mono - match_offset_s
        if (self._last_consumed_mono is not None
                and abs(matched_tim_tp_mono - self._last_consumed_mono) < 0.5):
            return None, None
        self._last_consumed_mono = matched_tim_tp_mono

        actual_offset = match_offset_s
        if not self.calibrated:
            self._calibration_offsets.append(actual_offset)
            self._n += 1
            if self._n >= 10:
                median = sorted(self._calibration_offsets)[len(self._calibration_offsets) // 2]
                self.offset_s = median
                self.calibrated = True
        else:
            if abs(actual_offset - self.offset_s) < 0.2:
                self.offset_s += self._alpha * (actual_offset - self.offset_s)

        return qerr_ns, actual_offset


class RxTcxoTracker:
    """Track the rx TCXO's phase and frequency relative to GPS.

    Fuses two independent measurements of the same oscillator:
    - **qErr** (TIM-TP): sub-tick position to ~178 ps, but ambiguous
      by 8 ns (one 125 MHz tick).
    - **dt_rx** (PPP filter): full phase, but ~4 ns noise per epoch.

    The tracker unwraps qErr for smooth inter-epoch phase tracking
    and uses dt_rx to anchor the integer tick count via a complementary
    filter (low-pass dt_rx + high-pass qErr unwrapped).

    Outputs:
    - ``phase_ns()``: synthesized phase — qErr precision, dt_rx ambiguity
    - ``freq_ns_per_s()``: frequency offset from 30-sample sliding window
    - ``cross_validate_dt_rx()``: rate comparison to detect cycle slips
    """

    TICK_NS = 8.0  # 125 MHz = 8 ns period
    WRAP_THRESHOLD_NS = 4.0  # half-tick threshold for fallback unwrap
    # Two unwrap modes:
    #   1. Half-tick threshold (fallback): correct only for |drift| <
    #      TICK_NS/(2·dt) ≈ 3.5 ns/s at 1 Hz sampling.
    #   2. Frequency-aided: pass predicted_drift_ns_per_s to update();
    #      uses round((expected - raw)/TICK_NS) to recover integer-tick
    #      count. Correct for any drift as long as the predictor's
    #      uncertainty is < 4 ns/s ≈ 4 ppb.
    # Mode 2 is required for F9T rx TCXOs which run at 100s of ppb.

    def __init__(self, freq_window=30):
        self._prev_qerr_ns = None
        self._prev_t_s = None
        self._accumulated_ns = 0.0  # unwrapped phase
        self._n = 0
        # Ring buffers for frequency estimation (slope of unwrapped phase
        # against actual sample timestamps).  _t_buf parallel to _phase_buf;
        # both have the same length after every update().  When the caller
        # supplies an absolute timestamp via update(t_s=...), the slope is
        # computed against real time differences — the natural handling for
        # variable-cadence raw qErr streams from QErrStore (TIM-TP at 1 Hz
        # by default but can configure higher).  When t_s is omitted, _t_buf
        # synthesizes uniform 1 s spacing from `dt` for backward compat
        # with the legacy fixed-cadence path.
        self._phase_buf = deque(maxlen=freq_window)
        self._t_buf = deque(maxlen=freq_window)
        # For dt_rx cross-validation (rate comparison, not absolute)
        self._prev_dt_rx = None
        self._dt_rx_calibrated = False
        # Synthesized phase: complementary filter
        # Low-freq from dt_rx (full phase), high-freq from qErr (precise)
        self._synth_dt_rx_buf = deque(maxlen=10)
        self._synth_qerr_buf = deque(maxlen=10)
        self._rx_tcxo_phase = None

    def update(self, qerr_ns, t_s=None, predicted_drift_ns_per_s=None,
               dt=1.0):
        """Feed one qErr sample (nanoseconds).  Returns unwrapped phase in ns.

        Args:
            qerr_ns: F9T's qErr value, in [-4, +4] ns (the 8-ns sawtooth
                quantization residual).
            t_s: optional absolute sample timestamp in seconds (e.g.
                CLOCK_MONOTONIC from the TIM-TP arrival).  When given,
                slope LS uses real time differences and `dt` is derived
                as t_s − prev_t_s.  When None (legacy callers), uses
                the explicit `dt` arg below and synthesizes uniform t.
            predicted_drift_ns_per_s: optional rx TCXO frequency-offset
                predictor (typically PPP dt_rx rate).  Enables
                frequency-aided unwrap that handles arbitrary drift.
                Without it, falls back to half-tick threshold (correct
                only for drift < TICK_NS/(2·dt) ≈ 3.5 ppb at 1 Hz).
            dt: time elapsed since previous sample, in seconds (default
                1.0).  Ignored when t_s is supplied — derived from the
                t_s difference instead.

        Returns:
            Unwrapped cumulative phase since first sample, in ns.

        Per docs/dofreq-est-measurement-ladder.md "Arm 2".  The slope
        of the returned phase IS rx TCXO frequency offset in ppb
        (= ns/s), consumable by DOFreqEst.update(qerr_freq_ppb=...).
        """
        # Derive elapsed dt from absolute timestamps when available.
        if t_s is not None and self._prev_t_s is not None:
            dt = max(0.0, float(t_s - self._prev_t_s))

        if self._prev_qerr_ns is None:
            self._prev_qerr_ns = qerr_ns
            self._prev_t_s = t_s if t_s is not None else 0.0
            self._accumulated_ns = 0.0
            self._phase_buf.append(self._accumulated_ns)
            self._t_buf.append(self._prev_t_s)
            self._n = 1
            return self._accumulated_ns

        raw_delta = qerr_ns - self._prev_qerr_ns

        if predicted_drift_ns_per_s is not None:
            # Frequency-aided unwrap.  Recover the integer-tick count
            # such that raw_delta + n*TICK_NS ≈ expected_delta.
            # Robust as long as |predictor noise| < TICK_NS/2 = 4 ns/s.
            expected_delta = predicted_drift_ns_per_s * dt
            n_ticks = round((expected_delta - raw_delta) / self.TICK_NS)
            true_delta = raw_delta + n_ticks * self.TICK_NS
        else:
            # Half-tick fallback.
            if raw_delta > self.WRAP_THRESHOLD_NS:
                true_delta = raw_delta - self.TICK_NS
            elif raw_delta < -self.WRAP_THRESHOLD_NS:
                true_delta = raw_delta + self.TICK_NS
            else:
                true_delta = raw_delta

        self._accumulated_ns += true_delta
        self._prev_qerr_ns = qerr_ns
        # Advance _prev_t_s by either the absolute timestamp or by dt.
        # Synthetic time is fine for legacy callers; the slope LS uses
        # whatever spacing _t_buf holds.
        new_t_s = t_s if t_s is not None else self._prev_t_s + dt
        self._prev_t_s = new_t_s
        self._phase_buf.append(self._accumulated_ns)
        self._t_buf.append(new_t_s)
        self._n += 1
        return self._accumulated_ns

    def update_from_qerr_store(self, qerr_store, predicted_drift_ns_per_s=None):
        """Drain new qErr samples from QErrStore and feed each into the
        tracker.  The tracker's slope LS uses each sample's TIM-TP
        host_time as its x-axis, so variable-cadence streams are
        handled correctly.

        Replaces the legacy per-edge feed where the engine matched a
        single qErr to each PPS edge via QErrStore.match_pps_mono and
        called update(matched_qerr_ns).  The new path is robust to
        FIFO drift across PPS gaps — see I-151540 Item 7 + Bravo's
        2026-05-08 review note.

        Args:
            qerr_store: a QErrStore instance.
            predicted_drift_ns_per_s: optional rx TCXO frequency
                predictor (typically PPP dt_rx rate).  Same constant
                value applied to every drained sample's unwrap math —
                fine in practice because TCXO drift evolves slowly
                (~ppb/s at most) compared to TIM-TP cadence (1 Hz
                typically).

        Returns:
            (n_drained, last_phase_ns) — number of samples consumed
            this call, and the unwrapped phase after the final sample
            (or self._accumulated_ns if zero drained).
        """
        # drain_since uses strict t > floor.  On a fresh tracker we
        # want every retained sample; floor=0.0 worked accidentally in
        # production (real monotonic timestamps are positive) but
        # dropped legitimate t=0.0 samples in unit tests.  Use -inf as
        # the "before all samples" sentinel — any finite t is strictly
        # greater.
        floor = self._prev_t_s if self._prev_t_s is not None else float("-inf")
        samples = qerr_store.drain_since(floor)
        for host_time, qerr_ns in samples:
            self.update(qerr_ns, t_s=host_time,
                        predicted_drift_ns_per_s=predicted_drift_ns_per_s)
        return len(samples), self._accumulated_ns

    def freq_ns_per_s(self):
        """Estimate rx TCXO frequency offset in ns/s from recent window.

        Returns (freq_ns_per_s, n_samples) or (None, 0) if insufficient data.
        At 1 Hz sampling, each index step = 1 second.
        """
        slope, sigma, n = self._slope_with_sigma()
        if slope is None:
            return None, 0
        return slope, n

    def freq_ppb_with_sigma(self):
        """Same slope as freq_ns_per_s, but also returns slope sigma.

        slope sigma comes from residual variance about the linear fit
        divided by sxx — standard textbook OLS sigma.  In ppb (= ns/s).

        Returns (slope_ppb, sigma_ppb, n_samples) or (None, None, 0).
        Used by Arm 2 of the four-arm Kalman fusion to pass rx TCXO
        frequency observations into DOFreqEst.update(qerr_freq_ppb=...).
        """
        return self._slope_with_sigma()

    def _slope_with_sigma(self):
        n = len(self._phase_buf)
        if n < 5:
            return None, None, 0
        phases = list(self._phase_buf)
        # Use _t_buf timestamps as the regression x-axis when available.
        # Variable-cadence raw qErr streams (per-TIM-TP host_time) need
        # real time differences for the slope to come out in ns/s.
        # When _t_buf was synthesized from `dt` arg, t_buf entries are
        # uniform — same answer the legacy index-based regression gave.
        if len(self._t_buf) == n:
            ts = list(self._t_buf)
        else:
            # Backward-compat shim: should not happen post-refactor but
            # protects against any caller that pre-populated _phase_buf
            # without going through update().
            ts = list(range(n))
        t_mean = sum(ts) / n
        y_mean = sum(phases) / n
        sxx = sum((t - t_mean) ** 2 for t in ts)
        sxy = sum((ts[i] - t_mean) * (phases[i] - y_mean) for i in range(n))
        if sxx <= 0:
            return None, None, 0
        slope = sxy / sxx
        intercept = y_mean - slope * t_mean
        # Residual variance about the linear fit; (n-2) DoF.
        if n <= 2:
            return slope, float("inf"), n
        ss_resid = sum((phases[i] - intercept - slope * ts[i]) ** 2
                        for i in range(n))
        var_resid = ss_resid / (n - 2)
        sigma_slope = math.sqrt(max(var_resid, 0.0) / sxx)
        return slope, sigma_slope, n

    def accumulated_phase_ns(self):
        """Return current unwrapped accumulated phase in ns."""
        return self._accumulated_ns

    def cross_validate_dt_rx(self, dt_rx_ns):
        """Compare qErr frequency against PPP dt_rx rate (mod 8 ns).

        qErr unwrapped phase tracks dt_rx modulo the 8 ns tick period.
        The integer tick drift (N × 8 ns/s) is invisible to qErr.
        Cross-validation compares rates: dt_rx_rate mod 8 should match
        qErr frequency.  A discrepancy indicates a cycle slip, filter
        fault, or qErr unwrap error.

        Returns (rate_discrepancy_ns_s, calibrated) or (None, False).
        """
        if self._n < 5:
            self._prev_dt_rx = dt_rx_ns
            return None, False

        if self._prev_dt_rx is None:
            self._prev_dt_rx = dt_rx_ns
            return None, False

        # dt_rx rate over the frequency window
        dt_rx_rate = dt_rx_ns - self._prev_dt_rx  # ns/s at 1 Hz
        self._prev_dt_rx = dt_rx_ns

        qerr_freq, n = self.freq_ns_per_s()
        if qerr_freq is None:
            return None, False

        # Compare rates mod 8: both should agree on the sub-tick drift
        dt_rx_rate_mod8 = ((dt_rx_rate + 4.0) % self.TICK_NS) - 4.0
        qerr_freq_mod8 = ((qerr_freq + 4.0) % self.TICK_NS) - 4.0
        discrepancy = dt_rx_rate_mod8 - qerr_freq_mod8
        # Wrap discrepancy to [-4, +4] to handle boundary cases
        if discrepancy > self.TICK_NS / 2:
            discrepancy -= self.TICK_NS
        elif discrepancy < -self.TICK_NS / 2:
            discrepancy += self.TICK_NS

        self._dt_rx_calibrated = True
        return discrepancy, True

    def phase_ns(self, dt_rx_ns, qerr_ns):
        """Combine dt_rx (full phase, noisy) with qErr (precise, ambiguous).

        Complementary filter: low-frequency content from dt_rx (smoothed
        over 10 epochs to average out its ~4 ns noise), high-frequency
        content from the qErr unwrapped phase (sub-tick, ~178 ps).

        Result: synth = smooth(dt_rx) + (qerr_uw - smooth(qerr_uw))

        At tau=1s this gives qErr-level TDEV (~1.4 ns vs dt_rx's 4.0 ns).
        At tau>30s it converges to dt_rx's curve (no tick ambiguity).
        """
        if dt_rx_ns is None or qerr_ns is None:
            return self._rx_tcxo_phase

        qerr_uw = self._accumulated_ns  # current unwrapped qErr phase

        self._synth_dt_rx_buf.append(dt_rx_ns)
        self._synth_qerr_buf.append(qerr_uw)

        n = len(self._synth_dt_rx_buf)
        if n < 3:
            return self._rx_tcxo_phase

        dt_rx_smooth = sum(self._synth_dt_rx_buf) / n
        qerr_uw_smooth = sum(self._synth_qerr_buf) / n

        self._rx_tcxo_phase = dt_rx_smooth + (qerr_uw - qerr_uw_smooth)
        return self._rx_tcxo_phase

    @property
    def synth_phase_ns(self):
        return self._rx_tcxo_phase

    @property
    def n_samples(self):
        return self._n

    @property
    def calibrated(self):
        return self._dt_rx_calibrated


def _dt_rx_trend_predict(buf):
    """Linear regression on dt_rx buffer, predict at the latest epoch.

    dt_rx drifts at ~22 ppb (~22 ns/s), so a linear fit over 30s gives
    a smooth TCXO phase estimate that never hits tick boundaries.
    Returns the smoothed dt_rx at the last sample's time, or None.
    """
    n = len(buf)
    if n < 3:
        return None
    # Simple linear regression: y = a + b*t, t = 0..n-1, predict at t=n-1
    sx = n * (n - 1) / 2
    sxx = n * (n - 1) * (2 * n - 1) / 6
    sy = sum(buf)
    sxy = sum(i * v for i, v in enumerate(buf))
    denom = n * sxx - sx * sx
    if abs(denom) < 1e-10:
        return sy / n
    b = (n * sxy - sx * sy) / denom
    a = (sy - b * sx) / n
    return a + b * (n - 1)


class RunningVarianceWindow:
    """Small rolling variance tracker for alignment qVIR metrics."""

    def __init__(self, maxlen=32):
        self._values = deque(maxlen=maxlen)

    def add(self, value):
        if value is not None:
            self._values.append(float(value))

    def variance(self):
        if len(self._values) < 2:
            return None
        return float(pvariance(self._values))

    def detrended_variance(self):
        """Return residual variance after removing a linear trend."""
        n = len(self._values)
        if n < 3:
            return None
        ts = list(range(n))
        values = list(self._values)
        mt = sum(ts) / n
        mv = sum(values) / n
        cov = sum((t - mt) * (v - mv) for t, v in zip(ts, values))
        var_t = sum((t - mt) ** 2 for t in ts)
        slope = cov / var_t if var_t else 0.0
        intercept = mv - slope * mt
        residuals = [v - (slope * t + intercept) for t, v in zip(ts, values)]
        return float(pvariance(residuals))

    def diff_variance(self):
        """Variance of first-differences: var(x[n] - x[n-1]).

        Immune to linear drift, quadratic glide, or any smooth trend —
        only measures the epoch-to-epoch jitter.
        """
        n = len(self._values)
        if n < 3:
            return None
        values = list(self._values)
        diffs = [values[i] - values[i - 1] for i in range(1, n)]
        return float(pvariance(diffs))

    def count(self):
        return len(self._values)


@dataclass
class TiccPairMeasurement:
    phc_channel: str
    ref_channel: str
    ref_sec: int
    diff_ns: float
    recv_mono: float
    confidence: float
    ref_qerr_ns: float = None  # qerr matched to the ref (gnss_pps) edge


class TiccPairTracker:
    """Pair TICC channel edges by integer ref_sec for realtime use."""

    def __init__(self, phc_channel: str, ref_channel: str):
        self.phc_channel = phc_channel
        self.ref_channel = ref_channel
        self._pending = {phc_channel: {}, ref_channel: {}}
        self._ref_qerr = {}  # ref_sec → qerr_ns, matched at chB arrival
        self._latest = None
        self._lock = threading.Lock()
        self._last_seen = {phc_channel: None, ref_channel: None}
        self._counts = {phc_channel: 0, ref_channel: 0}
        self._armed = False
        self._buffered_drops = 0
        self._boot_ref_sec_discard = 2

    def set_pending_ref_qerr(self, ref_sec, qerr_ns):
        """Store qerr matched to a ref (gnss_pps) edge before pairing."""
        with self._lock:
            self._ref_qerr[ref_sec] = qerr_ns

    def ingest(self, event):
        other = self.ref_channel if event.channel == self.phc_channel else self.phc_channel
        with self._lock:
            self._last_seen[event.channel] = event.recv_mono
            self._counts[event.channel] = self._counts.get(event.channel, 0) + 1
            self._pending[event.channel][event.ref_sec] = event
            other_event = self._pending[other].pop(event.ref_sec, None)
            if other_event is None:
                cutoff = event.ref_sec - 4
                for channel in self._pending.values():
                    stale_keys = [k for k in channel.keys() if k < cutoff]
                    for key in stale_keys:
                        channel.pop(key, None)
                stale_qerr = [k for k in self._ref_qerr if k < cutoff]
                for key in stale_qerr:
                    self._ref_qerr.pop(key, None)
                return

            this_event = event
            if this_event.channel == self.phc_channel:
                phc_event = this_event
                ref_event = other_event
            else:
                phc_event = other_event
                ref_event = this_event

            # Preserve raw logging, but do not let the first few post-open TICC
            # seconds into the live servo path. Those lines are commonly boot/
            # reopen artifacts and are not meaningful for control quality.
            #
            # Do not key this on queue_remains: for TICC, a valid matched pair
            # often arrives while its sibling line is still buffered, so
            # queue_remains can stay true even after the stream is healthy.
            if not self._armed:
                if event.ref_sec <= self._boot_ref_sec_discard:
                    self._buffered_drops += 1
                    return
                self._armed = True

            diff_ps = (
                (phc_event.ref_sec - ref_event.ref_sec) * 1_000_000_000_000
                + phc_event.ref_ps
                - ref_event.ref_ps
            )
            self._latest = TiccPairMeasurement(
                phc_channel=self.phc_channel,
                ref_channel=self.ref_channel,
                ref_sec=event.ref_sec,
                diff_ns=diff_ps * 1e-3,
                recv_mono=max(phc_event.recv_mono, ref_event.recv_mono),
                confidence=min(
                    getattr(phc_event, "correlation_confidence", 1.0) or 1.0,
                    getattr(ref_event, "correlation_confidence", 1.0) or 1.0,
                ),
                ref_qerr_ns=self._ref_qerr.pop(ref_event.ref_sec, None),
            )

    def latest(self, now_mono: float, max_age_s: float):
        with self._lock:
            if self._latest is None:
                return None
            if now_mono - self._latest.recv_mono > max_age_s:
                return None
            return self._latest

    def latest_diff_ns(self):
        """Return the most recent differential measurement in ns, or None."""
        with self._lock:
            if self._latest is None:
                return None
            return self._latest.diff_ns

    def health(self):
        with self._lock:
            return {
                "last_seen": dict(self._last_seen),
                "counts": dict(self._counts),
                "armed": self._armed,
                "buffered_drops": self._buffered_drops,
            }


def apply_ptp_profile(args):
    """Apply PTP defaults from config/receivers.toml when requested."""
    if not args.ptp_profile:
        return
    # Resolve config path relative to script directory if not found at CWD
    if not os.path.exists(args.device_config):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        candidate = os.path.join(script_dir, "..", "config", "receivers.toml")
        if os.path.exists(candidate):
            args.device_config = candidate
    try:
        with open(args.device_config, "rb") as f:
            cfg = tomllib.load(f)
    except FileNotFoundError:
        log.warning(f"PTP profile config not found: {args.device_config}")
        return

    profile = cfg.get("ptp", {}).get(args.ptp_profile)
    if not profile:
        log.warning(f"PTP profile not found: {args.ptp_profile}")
        return

    if args.servo is None:
        args.servo = profile.get("device", args.servo)
    if args.pps_pin is None:
        args.pps_pin = profile.get("pps_pin", args.pps_pin)
    if args.extts_channel is None:
        args.extts_channel = profile.get("extts_channel", args.extts_channel)
    if args.phc_timescale is None:
        args.phc_timescale = profile.get("timescale", args.phc_timescale)
    if getattr(args, "track_kp", None) == 0.3:
        args.track_kp = profile.get("track_kp", args.track_kp)
    if getattr(args, "track_ki", None) == 0.1:
        args.track_ki = profile.get("track_ki", args.track_ki)
    if not args.program_pin:
        args.program_pin = bool(profile.get("program_pin", False))
    if args.max_broadcast_age_s is None:
        args.max_broadcast_age_s = profile.get(
            "max_broadcast_age_s", args.max_broadcast_age_s
        )
    if args.require_ssr is None:
        args.require_ssr = profile.get("require_ssr", args.require_ssr)
    if args.max_ssr_age_s is None:
        args.max_ssr_age_s = profile.get("max_ssr_age_s", args.max_ssr_age_s)
    if args.min_correlation_confidence is None:
        args.min_correlation_confidence = profile.get(
            "min_correlation_confidence", args.min_correlation_confidence
        )
    if getattr(args, 'max_correlation_window_s', None) is None:
        args.max_correlation_window_s = profile.get(
            "max_correlation_window_s", None
        )
    if args.min_broadcast_confidence is None:
        args.min_broadcast_confidence = profile.get(
            "min_broadcast_confidence", args.min_broadcast_confidence
        )
    if args.min_ssr_confidence is None:
        args.min_ssr_confidence = profile.get(
            "min_ssr_confidence", args.min_ssr_confidence
        )
    if args.track_max_ppb is None:
        args.track_max_ppb = profile.get("track_max_ppb", args.track_max_ppb)
    if args.track_restep_ns is None:
        args.track_restep_ns = profile.get("track_restep_ns", args.track_restep_ns)
    if args.phase_step_bias_ns is None:
        args.phase_step_bias_ns = profile.get("phase_step_bias_ns", args.phase_step_bias_ns)
    if args.obs_idle_timeout_s is None:
        args.obs_idle_timeout_s = profile.get("obs_idle_timeout_s", args.obs_idle_timeout_s)
    if args.carrier_max_sigma_ns is None:
        args.carrier_max_sigma_ns = profile.get(
            "carrier_max_sigma_ns", args.carrier_max_sigma_ns
        )
    if args.track_outlier_ns is None:
        args.track_outlier_ns = profile.get("track_outlier_ns", args.track_outlier_ns)
    if args.discipline_interval == 1:
        args.discipline_interval = profile.get("discipline_interval", args.discipline_interval)
    if not args.adaptive_interval:
        args.adaptive_interval = bool(profile.get("adaptive_interval", args.adaptive_interval))
    if args.min_interval == 1:
        args.min_interval = profile.get("min_interval", args.min_interval)
    if args.max_interval == 120:
        args.max_interval = profile.get("max_interval", args.max_interval)
    if args.gain_ref_sigma == 2.0:
        args.gain_ref_sigma = profile.get("gain_ref_sigma", args.gain_ref_sigma)
    if args.converge_error_ns == 500.0:
        args.converge_error_ns = profile.get("converge_error_ns", args.converge_error_ns)
    if args.converge_min_scale == 2.0:
        args.converge_min_scale = profile.get("converge_min_scale", args.converge_min_scale)
    if args.gain_min_scale == 0.1:
        args.gain_min_scale = profile.get("gain_min_scale", args.gain_min_scale)
    if args.gain_max_scale == 1.0:
        args.gain_max_scale = profile.get("gain_max_scale", args.gain_max_scale)
    if args.scheduler_converge_threshold_ns == 100.0:
        args.scheduler_converge_threshold_ns = profile.get(
            "scheduler_converge_threshold_ns", args.scheduler_converge_threshold_ns
        )
    if args.scheduler_settle_window == 10:
        args.scheduler_settle_window = profile.get(
            "scheduler_settle_window", args.scheduler_settle_window
        )
    if args.scheduler_unconverge_factor == 5.0:
        args.scheduler_unconverge_factor = profile.get(
            "scheduler_unconverge_factor", args.scheduler_unconverge_factor
        )
    if getattr(args, 'measurement_rate_ms', None) is None:
        args.measurement_rate_ms = profile.get("measurement_rate_ms", None)
    if getattr(args, 'sfrbx_rate', None) is None:
        args.sfrbx_rate = profile.get("sfrbx_rate", None)
    # ClockMatrix I2C actuator (optional — only on OTC hardware)
    if getattr(args, 'clockmatrix_bus', None) is None:
        cm_bus = profile.get("clockmatrix_bus")
        if cm_bus is not None:
            args.clockmatrix_bus = cm_bus
            args.clockmatrix_addr = profile.get("clockmatrix_addr", "0x58")
            args.clockmatrix_dpll_actuator = profile.get(
                "clockmatrix_dpll_actuator", 3)
            args.clockmatrix_dpll_phase = profile.get(
                "clockmatrix_dpll_phase", 2)
            args.clockmatrix_pps_clk = profile.get(
                "clockmatrix_pps_clk", 2)
    # Bootstrap parameters (from phc_bootstrap.py profile loading)
    if getattr(args, 'phc_settime_lag_ns', 0) == 0:
        args.phc_settime_lag_ns = profile.get(
            "phc_settime_lag_ns", getattr(args, 'phc_settime_lag_ns', 0))
    if getattr(args, 'phc_step_threshold_ns', 10000) == 10000:
        args.phc_step_threshold_ns = profile.get(
            "phc_step_threshold_ns",
            getattr(args, 'phc_step_threshold_ns', 10000))
    if getattr(args, 'phc_optimal_stop_limit_s', 1.0) == 1.0:
        args.phc_optimal_stop_limit_s = profile.get(
            "phc_optimal_stop_limit_s",
            getattr(args, 'phc_optimal_stop_limit_s', 1.0))
    if getattr(args, 'glide_zeta', 0.7) == 0.7:
        args.glide_zeta = profile.get(
            "glide_zeta", getattr(args, 'glide_zeta', 0.7))
    if getattr(args, 'pps_out_pin', -1) == -1:
        args.pps_out_pin = profile.get(
            "pps_out_pin", getattr(args, 'pps_out_pin', -1))
    if getattr(args, 'pps_out_channel', 0) == 0:
        args.pps_out_channel = profile.get(
            "pps_out_channel", getattr(args, 'pps_out_channel', 0))




# ── Convergence detection (from peppar_find_position) ─────────────────── #

def position_sigma_3d(P):
    """Compute 3D position sigma from EKF covariance matrix."""
    P_pos = P[:3, :3]
    return math.sqrt(P_pos[0, 0] + P_pos[1, 1] + P_pos[2, 2])


# ── NTRIP config loading ─────────────────────────────────────────────── #

def load_ntrip_config(args):
    """Load NTRIP configuration from config file, merging with CLI args.

    Supports a separate SSR credentials file (ssr_ntrip_conf) so the SSR
    stream can come from a different caster than broadcast ephemeris.
    E.g., ephemeris from Australian mirror, SSR from BKG/CNES.
    """
    import configparser
    if args.ntrip_conf:
        conf = configparser.ConfigParser()
        conf.read(args.ntrip_conf)
        if 'ntrip' in conf:
            s = conf['ntrip']
            if not args.ntrip_caster:
                args.ntrip_caster = s.get('caster', args.ntrip_caster)
            if args.ntrip_port == 2101 and s.get('port'):
                args.ntrip_port = int(s.get('port'))
            if not args.ntrip_user:
                args.ntrip_user = s.get('user', args.ntrip_user)
            if not args.ntrip_password:
                args.ntrip_password = s.get('password', args.ntrip_password)
            if not args.ntrip_tls and s.getboolean('tls', False):
                args.ntrip_tls = True
            if not args.ssr_mount and s.get('mount'):
                args.ssr_mount = s.get('mount')

    # Separate SSR caster credentials (e.g., CNES on products.igs-ip.net
    # while ephemeris comes from the Australian mirror).
    ssr_conf_path = getattr(args, 'ssr_ntrip_conf', None)
    if ssr_conf_path:
        conf = configparser.ConfigParser()
        conf.read(ssr_conf_path)
        if 'ntrip' in conf:
            s = conf['ntrip']
            args.ssr_caster = s.get('caster', getattr(args, 'ssr_caster', None))
            args.ssr_port = int(s.get('port', 443))
            args.ssr_user = s.get('user', getattr(args, 'ssr_user', None))
            args.ssr_password = s.get('password', getattr(args, 'ssr_password', None))
            args.ssr_tls = s.getboolean('tls', True)
            if s.get('mount'):
                args.ssr_mount = s.get('mount')
            log.info("SSR credentials loaded from %s (caster=%s, mount=%s)",
                     ssr_conf_path, args.ssr_caster, args.ssr_mount)

    # Optional secondary SSR mount that contributes PHASE BIASES ONLY —
    # pairs with the primary ssr_mount which provides orbit/clock.  The
    # design enables CNES SSRA00CNE0 (GAL biases correct for F9T) + WHU
    # OSBC00WHU1 (GPS OSB correct for F9T L5Q) to jointly unlock GPS+GAL
    # AR without waiting for a single AC that matches every F9T signal.
    # See docs/ssr-mount-survey.md.
    ssr_bias_conf_path = getattr(args, 'ssr_bias_ntrip_conf', None)
    if ssr_bias_conf_path:
        conf = configparser.ConfigParser()
        conf.read(ssr_bias_conf_path)
        if 'ntrip' in conf:
            s = conf['ntrip']
            args.ssr_bias_caster = s.get(
                'caster', getattr(args, 'ssr_bias_caster', None))
            args.ssr_bias_port = int(s.get('port', 443))
            args.ssr_bias_user = s.get(
                'user', getattr(args, 'ssr_bias_user', None))
            args.ssr_bias_password = s.get(
                'password', getattr(args, 'ssr_bias_password', None))
            args.ssr_bias_tls = s.getboolean('tls', True)
            if not getattr(args, 'ssr_bias_mount', None) and s.get('mount'):
                args.ssr_bias_mount = s.get('mount')
            log.info(
                "SSR bias credentials loaded from %s (caster=%s, mount=%s)",
                ssr_bias_conf_path,
                args.ssr_bias_caster, args.ssr_bias_mount)


# ── Shared infrastructure setup ──────────────────────────────────────── #

def start_ntrip_threads(args, beph, ssr, stop_event):
    """Start NTRIP threads for ephemeris and SSR corrections."""
    threads = []
    use_tls = args.ntrip_tls or args.ntrip_port == 443

    if args.eph_mount:
        eph_stream = NtripStream(
            caster=args.ntrip_caster, port=args.ntrip_port,
            mountpoint=args.eph_mount,
            user=args.ntrip_user, password=args.ntrip_password,
            tls=use_tls,
        )
        t = threading.Thread(
            target=ntrip_reader,
            args=(eph_stream, beph, ssr, stop_event, "EPH"),
            daemon=True,
        )
        t.start()
        threads.append(t)
        # peppar-mon contract: peppar_mon/log_reader.py:_EPH_STREAM_RE
        # parses HOST:PORT/MOUNT to display the eph mount name.
        log.info(f"Ephemeris stream: {args.ntrip_caster}:{args.ntrip_port}/{args.eph_mount}")

    if args.ssr_mount:
        # SSR can use a separate caster (e.g., products.igs-ip.net for CNES
        # while ephemeris comes from the Australian mirror)
        ssr_host = getattr(args, 'ssr_caster', None) or args.ntrip_caster
        ssr_p = getattr(args, 'ssr_port', None) or args.ntrip_port
        ssr_u = getattr(args, 'ssr_user', None) or args.ntrip_user
        ssr_pw = getattr(args, 'ssr_password', None) or args.ntrip_password
        ssr_tls = getattr(args, 'ssr_tls', None)
        if ssr_tls is None:
            ssr_tls = use_tls if ssr_p == args.ntrip_port else (ssr_p == 443)
        ssr_stream = NtripStream(
            caster=ssr_host, port=ssr_p,
            mountpoint=args.ssr_mount,
            user=ssr_u, password=ssr_pw,
            tls=ssr_tls,
        )
        t = threading.Thread(
            target=ntrip_reader,
            args=(ssr_stream, beph, ssr, stop_event, "SSR"),
            kwargs={
                'skip_biases':       getattr(args, 'no_primary_biases', False),
                'skip_code_biases':  getattr(args, 'no_ssr_code_bias', False),
                'skip_phase_biases': getattr(args, 'no_ssr_phase_bias', False),
            },
            daemon=True,
        )
        t.start()
        threads.append(t)
        # peppar-mon contract: peppar_mon/log_reader.py:_SSR_STREAM_RE
        # parses HOST:PORT/MOUNT to display the SSR mount name.
        log.info(f"SSR stream: {ssr_host}:{ssr_p}/{args.ssr_mount}")
        if getattr(args, 'no_primary_biases', False):
            log.info("Primary SSR biases (code + phase) suppressed "
                     "(--no-primary-biases)")
        if getattr(args, 'no_ssr_code_bias', False):
            log.info("All SSR code biases suppressed (--no-ssr-code-bias)")
        if getattr(args, 'no_ssr_phase_bias', False):
            log.info("All SSR phase biases suppressed (--no-ssr-phase-bias)")
        peer_publisher.publish_streams(
            ssr_mount=args.ssr_mount,
            eph_mount=getattr(args, 'eph_mount', None),
        )

    # Optional secondary SSR mount that contributes PHASE BIASES ONLY.
    # Orbit/clock/code-bias/ephemeris all come from the primary mount;
    # this stream's non-phase-bias messages are discarded in ntrip_reader.
    ssr_bias_mount = getattr(args, 'ssr_bias_mount', None)
    if ssr_bias_mount:
        bias_host = (getattr(args, 'ssr_bias_caster', None)
                     or getattr(args, 'ssr_caster', None)
                     or args.ntrip_caster)
        bias_p = (getattr(args, 'ssr_bias_port', None)
                  or getattr(args, 'ssr_port', None)
                  or args.ntrip_port)
        bias_u = (getattr(args, 'ssr_bias_user', None)
                  or getattr(args, 'ssr_user', None)
                  or args.ntrip_user)
        bias_pw = (getattr(args, 'ssr_bias_password', None)
                   or getattr(args, 'ssr_password', None)
                   or args.ntrip_password)
        bias_tls = getattr(args, 'ssr_bias_tls', None)
        if bias_tls is None:
            bias_tls = bias_p == 443
        bias_stream = NtripStream(
            caster=bias_host, port=bias_p,
            mountpoint=ssr_bias_mount,
            user=bias_u, password=bias_pw,
            tls=bias_tls,
        )
        t = threading.Thread(
            target=ntrip_reader,
            args=(bias_stream, beph, ssr, stop_event, "SSR-BIAS"),
            kwargs={
                'bias_only':         True,
                'skip_code_biases':  getattr(args, 'no_ssr_code_bias', False),
                'skip_phase_biases': getattr(args, 'no_ssr_phase_bias', False),
            },
            daemon=True,
        )
        t.start()
        threads.append(t)
        log.info(f"SSR bias stream (code+phase bias only): "
                 f"{bias_host}:{bias_p}/{ssr_bias_mount}")

    return threads


def wait_for_ephemeris(beph, stop_event, systems=None, timeout_s=120):
    """Wait for broadcast ephemeris — each configured system must have >= 8 SVs."""
    SYS_TO_PREFIX = {'gps': 'G', 'gal': 'E', 'bds': 'C'}
    required = {SYS_TO_PREFIX[s] for s in (systems or {'gps', 'gal', 'bds'}) if s in SYS_TO_PREFIX}
    required.add('G')
    log.info(f"Waiting for broadcast ephemeris (need {required})...")
    warmup_start = time.time()
    while time.time() - warmup_start < timeout_s:
        if stop_event.is_set():
            return False
        by_sys = {}
        for prn in beph.satellites:
            s = prn[0]
            by_sys[s] = by_sys.get(s, 0) + 1
        if all(by_sys.get(p, 0) >= 8 for p in required):
            break
        time.sleep(1)
        if int(time.time() - warmup_start) % 10 == 0:
            log.info(f"  Warmup: {beph.summary()}")
    log.info(f"Broadcast ephemeris ready: {beph.summary()}")
    return True


def wait_for_nav2_seed(nav2_store, stop_event, timeout_s=60.0,
                       hacc_max_m=5.0, max_age_s=5.0):
    """Wait for NAV2 to produce a usable position fix for filter seeding.

    NAV2 (the F9T's secondary nav engine) computes a fresh position
    every epoch independent of our PPP filter.  At cold start it
    typically reaches fixType=3 within 30-60 s on clean sky.  This
    helper polls Nav2PositionStore until the opinion meets the
    quality bar (fixType=3, hAcc < hacc_max_m, age < max_age_s) or
    the timeout expires.

    Returns:
        (ecef_ndarray, h_acc_m, num_sv) on success
        None on timeout or if the store isn't available

    Used by the engine's main loop in lieu of run_bootstrap when a
    live receiver with NAV2 is present (the typical lab case).
    Per I-024532-charlie #3: collapse the Phase 1/Phase 2 dichotomy
    by using NAV2 as the seed and skipping the LS-init bootstrap +
    CONVERGED ceremony.
    """
    if nav2_store is None:
        return None
    log.info("Waiting for NAV2 seed (fixType=3, hAcc<%.0fm) up to %.0fs...",
             hacc_max_m, timeout_s)
    deadline = time.time() + timeout_s
    last_log = 0.0
    while time.time() < deadline:
        if stop_event.is_set():
            return None
        opinion = nav2_store.get_opinion(max_age_s=max_age_s)
        if opinion is not None:
            ftype = opinion.get('fix_type')
            hacc = opinion.get('h_acc_m')
            if ftype == 3 and hacc is not None and hacc < hacc_max_m:
                ecef = np.asarray(opinion['ecef'], dtype=float)
                num_sv = opinion.get('num_sv', 0)
                log.info("NAV2 seed acquired: fixType=3 hAcc=%.2fm "
                         "nSV=%d after %.1fs",
                         hacc, num_sv, time.time() - (deadline - timeout_s))
                return ecef, float(hacc), int(num_sv)
        if time.time() - last_log >= 10.0:
            if opinion is None:
                log.info("  NAV2 not yet emitting NAV2-PVT...")
            else:
                log.info("  NAV2 fix=%s hAcc=%s — not yet at seed quality",
                         opinion.get('fix_type'),
                         f"{opinion.get('h_acc_m'):.2f}m"
                         if opinion.get('h_acc_m') is not None else "?")
            last_log = time.time()
        time.sleep(1.0)
    log.warning("NAV2 seed timed out after %.0fs — falling back to LS-init "
                "bootstrap.  This is the regression-harness path; live "
                "receivers should produce a NAV2 fix within ~30-60s.",
                timeout_s)
    return None


def _build_ppp_filter(args):
    """PPPFilter factory honouring --clock-model + --rx-tcxo-adev-1s
    + --nav-sig-gate.

    Keeps the two instantiation sites (bootstrap + AntPosEstThread
    warm-start) in lockstep.  Default args shape preserves legacy
    bit-exact behaviour (clock_model='random_walk', nav-sig gate off).

    args may carry a `nav_sig_store` attribute set by main() after
    Nav2SignalStore is instantiated; the factory forwards it when
    --nav-sig-gate is on.
    """
    kwargs = {}
    cm = getattr(args, "clock_model", "random_walk")
    if cm and cm != "random_walk":
        kwargs["clock_model"] = cm
        adev = getattr(args, "rx_tcxo_adev_1s", None)
        if adev is not None:
            kwargs["rx_tcxo_adev_1s"] = float(adev)
    # Phase B NAV-SIG L0 admission gate.  When --nav-sig-gate is set
    # AND main() stashed a nav_sig_store on args, forward both into
    # the filter so its per-obs loop can consult the receiver verdict.
    if getattr(args, "nav_sig_gate", False):
        store = getattr(args, "nav_sig_store", None)
        if store is not None:
            kwargs["nav_sig_store"] = store
            kwargs["nav_sig_gate_enabled"] = True
            max_age = getattr(args, "nav_sig_max_age_s", 2.0)
            kwargs["nav_sig_max_age_s"] = float(max_age)
    return PPPFilter(**kwargs)


# ── Phase 1: Bootstrap ─────────────────────────────────────────────────── #

def run_bootstrap(args, obs_queue, corrections, stop_event, out_w=None,
                  nav2_store=None):
    """Run PPPFilter to estimate position from scratch.

    Returns:
        (ecef, sigma_m) on convergence, or None on timeout/error.
    """
    log.info("=== Phase 1: Position bootstrap (PPPFilter) ===")
    cm = getattr(args, "clock_model", "random_walk")
    if cm == "random_walk":
        log.info(f"  clock_model={cm} (legacy Q_clk=1e6·dt)")
    else:
        adev = getattr(args, "rx_tcxo_adev_1s", None)
        log.info(f"  clock_model={cm} rx_tcxo_adev_1s="
                 f"{adev if adev is not None else '(default)'}")

    # Seed position
    seed_ecef = None
    if args.seed_pos:
        lat, lon, alt = [float(v) for v in args.seed_pos.split(',')]
        seed_ecef = lla_to_ecef(lat, lon, alt)
        log.info(f"Seed position: {lat:.6f}, {lon:.6f}, {alt:.1f}m")

    filt = _build_ppp_filter(args)
    filt_initialized = False
    correction_gate = CorrectionFreshnessGate()
    run_bootstrap.last_correction_gate_stats = correction_gate.stats.as_dict()

    # PPP-AR: Melbourne-Wubbena wide-lane + narrow-lane resolver
    mw_tracker = MelbourneWubbenaTracker()
    # Optional per-attempt NL diagnostic.  Off unless --nl-diag is set.
    # Shared across bootstrap and AntPosEst so the inherited resolver
    # keeps the same logger (otherwise an NL fix during bootstrap would
    # log to one and steady-state to another, breaking grep/awk).
    nl_diag = NlDiagLogger(enabled=bool(getattr(args, "nl_diag", False)))
    nl_resolver = NarrowLaneResolver(
        ar_elev_mask_deg=args.ar_elev_mask, nl_diag=nl_diag,
        join_test_enabled=bool(getattr(args, "join_test", True)),
        wl_only=bool(getattr(args, "wl_only", False)),
    )
    slip_monitor = CycleSlipMonitor(
        mw_tracker=mw_tracker, csv_writer=_slip_csv_writer(),
        min_slip_interval_s=float(getattr(args, 'slip_rate_limit_s', 0.0)))

    prev_t = None
    prev_pos_ecef = None
    n_epochs = 0
    n_empty = 0
    converged_at = None
    # Tracks the wall time at which we first saw a pre-init epoch
    # without a fresh NAV2 opinion.  Used to bound the
    # nav2-seed-timeout-s wait window: while elapsed-since-this <
    # timeout, we keep waiting (skip LS-init) so NAV2 can be the seed.
    nav2_wait_started: float | None = None
    # Set on the init epoch by the seed-source dispatch below.  Reads
    # by the convergence-gate logic distinguish NAV2-seeded fast-exit
    # (no 30-epoch wait, W2 redundant) from LS-init full ceremony
    # (30-epoch wait + W1 + W2).  Per I-024532-charlie #3, the NAV2
    # path collapses Phase 1's CONVERGED gate down to a 1-epoch sanity
    # rather than the legacy 30-epoch ceremony.
    bootstrap_seed_source = None
    start_time = time.time()
    # W1/W2/W3: retry accounting for the convergence gate.  Each abort
    # (residual inconsistency or NAV2 horizontal mismatch) triggers a
    # scrub and the filter tries again.  Bounded by --bootstrap-max-retries.
    gate_retries = 0
    last_gate_reason = None

    while not stop_event.is_set():
        elapsed = time.time() - start_time

        if args.timeout and elapsed > args.timeout:
            log.warning(f"Bootstrap timeout after {elapsed:.0f}s")
            return None

        try:
            gps_time, observations = obs_queue.get(timeout=5)
        except queue.Empty:
            n_empty += 1
            if n_empty > 12:
                log.error("No observations for 60s during bootstrap")
                return None
            continue
        n_empty = 0

        ok_corr, corr_reason, corr_snapshot = correction_gate.accept(
            corrections,
            max_broadcast_age_s=args.max_broadcast_age_s,
            require_ssr=args.require_ssr,
            max_ssr_age_s=args.max_ssr_age_s,
            min_broadcast_confidence=args.min_broadcast_confidence,
            min_ssr_confidence=args.min_ssr_confidence,
        )
        run_bootstrap.last_correction_gate_stats = correction_gate.stats.as_dict()
        if not ok_corr:
            if n_epochs % 10 == 0:
                log.info(
                    "Bootstrap waiting for fresh corrections: reason=%s "
                    "broadcast_age=%s",
                    corr_reason,
                    f"{corr_snapshot['broadcast_age_s']:.1f}s"
                    if corr_snapshot["broadcast_age_s"] is not None else "N/A",
                )
            continue

        # Initialize filter on first epoch with enough satellites.
        #
        # Seed-source priority (I-024532-charlie / I-133648-main):
        #   1. args.seed_pos — explicit operator override (kept).
        #   2. NAV2 fixType=3 + hAcc < 5 m — receiver's secondary nav
        #      engine; independent of our PPP filter, ~1 m hAcc on
        #      clean sky.  Initial P[0:3] is set from NAV2.hAcc so
        #      the filter is stiff-but-not-anchored from epoch 0
        #      and systematic SSR phase-bias residuals fall into
        #      ambiguities (correctly) instead of into position.
        #   3. LS init via broadcast ephemeris — fallback for cold
        #      starts where NAV2 hasn't converged yet, and for the
        #      regression harness running file-based without a live
        #      F9T.  Initial P[0:3] uses the legacy 100 m σ via the
        #      pos_sigma_m=100.0 override, since LS init is
        #      km-level on its first epoch.
        if not filt_initialized:
            init_pos_sigma_m = None
            seed_source = None
            if seed_ecef is not None:
                init_pos = seed_ecef
                init_clk = 0.0
                seed_source = "args.seed_pos"
                # Operator-supplied seed: caller knows the position
                # quality.  Default 10 m σ is a reasonable assumption.
                init_pos_sigma_m = 10.0
            elif nav2_store is not None:
                opinion = nav2_store.get_opinion(max_age_s=10.0)
                if (opinion is not None
                        and opinion.get('fix_type') == 3
                        and opinion.get('h_acc_m') is not None
                        and opinion['h_acc_m'] < 5.0):
                    init_pos = np.asarray(opinion['ecef'], dtype=float)
                    init_clk = 0.0
                    init_pos_sigma_m = float(opinion['h_acc_m'])
                    seed_source = (f"NAV2 (fix=3, hAcc={init_pos_sigma_m:.2f}m, "
                                   f"nSV={opinion.get('num_sv', '?')})")
                else:
                    # NAV2 not ready yet.  Wait up to
                    # --nav2-seed-timeout-s before falling back to
                    # LS-init.  Per Bob's directive 2026-04-30 +
                    # cold_boot_smoke run-1 finding (LS-init Phase-1
                    # converged at horiz-good + 2 km altitude error).
                    timeout_s = float(getattr(
                        args, 'nav2_seed_timeout_s', 30.0))
                    if timeout_s > 0:
                        if nav2_wait_started is None:
                            nav2_wait_started = time.time()
                            _fix = (opinion.get('fix_type')
                                    if opinion else None)
                            _hacc = (opinion.get('h_acc_m')
                                     if opinion else None)
                            log.info(
                                "NAV2 not ready (fix_type=%s, "
                                "h_acc=%s); waiting up to %.0fs "
                                "before LS-init fallback",
                                _fix, _hacc, timeout_s)
                        elapsed = time.time() - nav2_wait_started
                        if elapsed < timeout_s:
                            # Keep waiting; skip this epoch's init.
                            # Drop into the LS-init block below ONLY
                            # if timeout has expired.
                            continue

            if seed_source is None:
                if nav2_wait_started is not None:
                    log.warning(
                        "NAV2 wait timed out after %.1fs; falling "
                        "back to LS-init",
                        time.time() - nav2_wait_started)
                # Fallback: LS init via broadcast ephemeris.  SSR
                # orbit corrections poison the absolute LS solver, so
                # broadcast-only.  ~5 m accuracy, plenty for seeding
                # PPPFilter when NAV2 isn't ready.
                _beph = corrections.beph
                x_ls, ok, n_sv = ls_init(observations, _beph, gps_time,
                                          clk_file=None)
                if not ok or n_sv < 4:
                    log.info(f"Waiting for enough satellites (got {n_sv})")
                    continue
                init_pos = x_ls[:3]
                init_clk = x_ls[3]
                seed_source = f"LS init (n_sv={n_sv}, fallback)"
                # LS init is km-level on its first epoch; the legacy
                # 100 m default kept here so behaviour matches the
                # pre-NAV2-seed regression cases.  Once the engine
                # converges past 100 m via observations, the looser
                # initial P doesn't matter.
                init_pos_sigma_m = 100.0

            systems_set = (set(args.systems.split(',')) if args.systems
                           else None)
            filt.initialize(init_pos, init_clk, systems=systems_set,
                            pos_sigma_m=init_pos_sigma_m)
            filt_initialized = True
            bootstrap_seed_source = seed_source
            prev_t = gps_time
            log.info("PPPFilter initialized via %s, σ_pos=%.1fm",
                     seed_source, init_pos_sigma_m)
            continue

        # EKF predict
        dt = (gps_time - prev_t).total_seconds()
        if dt <= 0 or dt > 30:
            log.warning(f"Suspicious dt={dt:.1f}s, skipping")
            prev_t = gps_time
            continue
        filt.predict(dt)
        prev_t = gps_time

        # Manage ambiguities — flush per-SV phase state on any detected
        # cycle slip, retaining per-SV and shared frequency-like state.
        current_svs = {o['sv'] for o in observations}
        elevations = _compute_sv_elevations(filt, corrections,
                                                  observations, gps_time)
        azimuths = _compute_sv_azimuths(filt, corrections,
                                              observations, gps_time)
        ipp_szas = _compute_sv_ipp_szas(filt, azimuths, elevations, gps_time)
        slip_events = slip_monitor.check(
            observations, gps_time.timestamp(), n_epochs,
            elevations=elevations,
            azimuths=azimuths, ipp_szas=ipp_szas)
        for ev in slip_events:
            # PR-only slips (mw_jump alone) are PR-noise false positives;
            # don't flush WL state.  See is_pr_only_slip() docstring +
            # I-155354 fix-I + docs/wl-integer-vs-pride-comparison.md.
            if is_pr_only_slip(ev.reasons):
                log.info("[SLIP_FILTERED] sv=%s mw_jump-only conf=%s "
                         "mw=%s — PR-only, ambiguity preserved",
                         ev.sv, ev.confidence,
                         f"{ev.mw_jump_cyc:.2f}c"
                         if ev.mw_jump_cyc is not None else "-")
            else:
                flush_sv_phase(
                    ev.sv, filt=filt, mw_tracker=mw_tracker,
                    nl_resolver=nl_resolver, slip_monitor=slip_monitor,
                    reason="|".join(ev.reasons), epoch=n_epochs)
            log.info("slip: sv=%s reasons=%s conf=%s lock=%.0fms cno=%.1f"
                     " elev=%s az=%s ipp_sza=%s gap=%s gf=%s mw=%s",
                     ev.sv, ",".join(ev.reasons), ev.confidence,
                     ev.lock_ms, ev.cno,
                     f"{ev.elevation_deg:.0f}°" if ev.elevation_deg is not None else "?",
                     f"{ev.azimuth_deg:.0f}°" if ev.azimuth_deg is not None else "?",
                     f"{ev.ipp_sza_deg:.0f}°" if ev.ipp_sza_deg is not None else "?",
                     f"{ev.gap_s:.2f}s" if ev.gap_s is not None else "-",
                     f"{ev.gf_jump_m*100:.1f}cm" if ev.gf_jump_m is not None else "-",
                     f"{ev.mw_jump_cyc:.2f}c" if ev.mw_jump_cyc is not None else "-")
            peer_publisher.publish_slip_event(
                sv=ev.sv, reasons=ev.reasons, conf=ev.confidence,
                elev_deg=ev.elevation_deg,
                azimuth_deg=ev.azimuth_deg,
                ipp_sza_deg=ev.ipp_sza_deg,
                lock_duration_ms=int(ev.lock_ms) if ev.lock_ms is not None else None,
                gf_jump_m=ev.gf_jump_m, mw_jump_cyc=ev.mw_jump_cyc,
            )

        for obs in observations:
            sv = obs['sv']
            if sv not in filt.sv_to_idx and obs.get('phi_if_m') is not None:
                sat_pos, sat_clk = corrections.sat_position(sv, gps_time)
                if sat_pos is not None:
                    N_init = obs['pr_if'] - obs['phi_if_m']
                    filt.add_ambiguity(sv, N_init)

        filt.prev_obs = {o['sv']: o for o in observations}

        for sv in list(filt.sv_to_idx.keys()):
            if sv not in current_svs:
                filt.remove_ambiguity(sv)

        # EKF update — position-side consumes only SVs whose phase biases
        # are fully available; the per-filter gate (I-175645-charlie)
        # drops MISS-bias SVs to keep the half-bias-corrected MW
        # combination from poisoning WL integer search.  See
        # obs_for_position() docstring at module top.
        pos_observations = obs_for_position(observations)
        n_used, resid, sys_counts = filt.update(
            pos_observations, corrections, gps_time, clk_file=corrections)

        if n_used < 4:
            continue

        # Soft ZTD-residual prior (engine flag --ztd-tie-sigma).  Same
        # mechanism as the AntPosEstThread loop applies post-update.
        _ztd_tie = getattr(args, "ztd_tie_sigma", None)
        if _ztd_tie:
            filt.apply_ztd_tie(_ztd_tie)

        n_epochs += 1

        pos_ecef = filt.x[:3]
        sigma_3d = position_sigma_3d(filt.P)
        lat, lon, alt = ecef_to_lla(pos_ecef[0], pos_ecef[1], pos_ecef[2])
        rms = np.sqrt(np.mean(resid ** 2)) if len(resid) > 0 else 0

        # CSV output
        if out_w:
            out_w.writerow([
                gps_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:23],
                'bootstrap',
                f'{lat:.7f}', f'{lon:.7f}', f'{alt:.3f}',
                f'{sigma_3d:.4f}', '', '',
                n_used, f'{rms:.4f}',
                '', '', '', len(filt.sv_to_idx),
            ])

        # PPP-AR: Melbourne-Wubbena wide-lane update (every epoch).
        # Same per-filter gate (I-175645-charlie): MW combination places
        # WL integer search on a wrong cycle when band-2 phase bias is
        # MISS, so iterate only the position-filtered observation list.
        for obs in pos_observations:
            sv = obs['sv']
            phi1 = obs.get('phi1_cyc')
            phi2 = obs.get('phi2_cyc')
            pr1 = obs.get('pr1_m')
            pr2 = obs.get('pr2_m')
            wl1 = obs.get('wl_f1')
            wl2 = obs.get('wl_f2')
            if all(v is not None for v in (phi1, phi2, pr1, pr2, wl1, wl2)):
                f1_hz = C / wl1
                f2_hz = C / wl2
                mw_tracker.update(sv, phi1, phi2, pr1, pr2, f1_hz, f2_hz)

        # PPP-AR: narrow-lane resolution attempt (every epoch after warmup).
        # `elevations` was computed above for the cycle-slip monitor;
        # reuse it so the AR elevation mask excludes low-elev SVs from
        # integer fixing without recomputing sat positions.
        if n_epochs >= 30:
            # Per-SV phase-bias availability for the short-term promoter's
            # candidate gate.  Computed at obs-pack time in realtime_ppp.py;
            # default True keeps legacy/replay paths (no SSR stream) unchanged.
            ar_phase_bias_ok = {
                o['sv']: o.get('ar_phase_bias_ok', True)
                for o in observations
            }
            nl_resolver.attempt(filt, mw_tracker, elevations=elevations,
                                ar_phase_bias_ok=ar_phase_bias_ok)

        if n_epochs % 5 == 0:
            log.info(
                f"  [{n_epochs}] σ={sigma_3d:.3f}m "
                f"pos=({lat:.6f}, {lon:.6f}, {alt:.1f}) "
                f"n={n_used} amb={len(filt.sv_to_idx)} "
                f"rms={rms:.3f}m [{elapsed:.0f}s]"
            )
            # Log corrected integrality (WL/NL decomposition)
            if len(filt.sv_to_idx) > 0 and n_epochs % 10 == 0:
                int_results = nl_resolver.integrality(filt, mw_tracker)
                if int_results:
                    fracs = [abs(r[1]) for r in int_results]
                    n_fixable = sum(1 for r in int_results if abs(r[1]) < 0.15)
                    # Split by constellation
                    gal = [(sv, f, s, fx) for sv, f, s, fx in int_results if sv.startswith('E')]
                    gps = [(sv, f, s, fx) for sv, f, s, fx in int_results if sv.startswith('G')]
                    gal_frac = np.mean([abs(f) for _, f, _, _ in gal]) if gal else float('nan')
                    gps_frac = np.mean([abs(f) for _, f, _, _ in gps]) if gps else float('nan')
                    gal_sig = [s for _, _, s, _ in gal] if gal else []
                    gps_sig = [s for _, _, s, _ in gps] if gps else []
                    gal_sig_str = f"σ={min(gal_sig):.3f}-{max(gal_sig):.3f}" if gal_sig else ""
                    gps_sig_str = f"σ={min(gps_sig):.3f}-{max(gps_sig):.3f}" if gps_sig else ""
                    log.info(f"    AR: GAL|frac|={gal_frac:.3f}({len(gal)}) {gal_sig_str} "
                             f"GPS|frac|={gps_frac:.3f}({len(gps)}) {gps_sig_str} "
                             f"fixable={n_fixable} "
                             f"{mw_tracker.summary()} {nl_resolver.summary()}")
                    if n_epochs % 30 == 0:
                        for sv, frac, sigma, fixed in int_results:
                            tag = "FIXED" if fixed else ""
                            log.info(f"      {sv}: N1_frac={frac:+.3f} "
                                     f"σ_N1={sigma:.3f} {tag}")

        # Convergence check — σ and pos_stable are necessary but not
        # sufficient.  Before declaring CONVERGED we also require the
        # residual distribution to match our noise model (W1) and NAV2
        # to agree horizontally (W2) — see
        # docs/position-bootstrap-reliability-plan.md.
        pos_stable = True
        if prev_pos_ecef is not None:
            pos_delta = np.linalg.norm(pos_ecef - prev_pos_ecef)
            pos_stable = pos_delta < args.sigma
        prev_pos_ecef = pos_ecef.copy()

        if sigma_3d < args.sigma and pos_stable:
            if converged_at is None:
                converged_at = n_epochs
            # NAV2-seeded path bypasses the 30-epoch CONVERGED ceremony
            # (I-024532-charlie #3): NAV2 already vetted the seed at
            # initialize time, and the priors-tightening (commit f67871d)
            # leaves systematic δ_PB nowhere to absorb during convergence.
            # We require 1 stable epoch instead of 30 — the EKF's own
            # sigma_3d threshold is sufficient.  W2 (NAV2 cross-check)
            # is also redundant on this path since the seed itself was
            # NAV2-validated; we still run it to short-circuit on a
            # NAV2-vs-PPP disagreement that would have been caught by
            # the steady-state SO_POS monitor anyway, but the W1
            # residual gate carries the load.
            seeded_from_nav2 = (
                bootstrap_seed_source is not None
                and bootstrap_seed_source.startswith("NAV2"))
            stable_epochs_required = 1 if seeded_from_nav2 else 30
            if n_epochs - converged_at >= stable_epochs_required:
                # W1: residual-consistency gate.
                w1_ok, w1 = residuals_consistent(
                    filt, resid, SIGMA_P_IF,
                    pr_rms_k=args.bootstrap_rms_k)
                # W2: NAV2 horizontal cross-check.  Disabled when
                # --bootstrap-nav2-horiz-m is ≤ 0.  On NAV2-seeded
                # path, redundant by construction (we just used NAV2
                # to seed) but kept as belt-and-suspenders against a
                # post-init NAV2 disagreement.
                nav2_opinion = None
                if (nav2_store is not None
                        and args.bootstrap_nav2_horiz_m > 0):
                    nav2_opinion = nav2_store.get_opinion(max_age_s=30.0)
                w2_ok, w2 = nav2_agrees(
                    pos_ecef, nav2_opinion,
                    horiz_m=args.bootstrap_nav2_horiz_m,
                    vert_m=args.bootstrap_nav2_vert_m)

                if w1_ok and w2_ok:
                    log.info(
                        "CONVERGED at epoch %d (σ=%.4fm, rms=%.3fm, "
                        "pr_rms=%.2fm max=%.2fm, nav2_h=%.1fm "
                        "nav2_v=%.1fm%s, retries=%d, gate=%s)",
                        n_epochs, sigma_3d, rms,
                        w1['rms_pr'], w1['max_pr'],
                        w2['disp_h_m'], w2['disp_v_m'],
                        "" if w2['available'] else " (no NAV2)",
                        gate_retries,
                        "fast (NAV2 seed)" if seeded_from_nav2
                        else "ceremony (LS-init)",
                    )
                    run_bootstrap.last_correction_gate_stats = \
                        correction_gate.stats.as_dict()
                    return BootstrapResult(
                        ecef=pos_ecef,
                        sigma_m=float(sigma_3d),
                        ppp_filter=filt,
                        mw_tracker=mw_tracker,
                        nl_resolver=nl_resolver,
                    )

                # One of the extra gates rejected this candidate.
                reasons = [d['reason'] for d, ok in
                           ((w1, w1_ok), (w2, w2_ok)) if not ok]
                last_gate_reason = "; ".join(reasons)
                if gate_retries >= args.bootstrap_max_retries:
                    log.error(
                        "Phase-1 gate aborted after %d retries; giving "
                        "up. Last reasons: %s",
                        gate_retries, last_gate_reason)
                    run_bootstrap.last_correction_gate_stats = \
                        correction_gate.stats.as_dict()
                    return None

                # NAV2-seeded path: gate failure means "wait another
                # stable window, don't scrub" — scrubbing would re-seed
                # from the same NAV2 we just used.  Resetting
                # converged_at lets the convergence loop re-attempt on
                # the next sustained-stable window.  Bounded by the
                # same gate_retries budget so a degenerate stream
                # eventually aborts.  Per I-024532-charlie #3.
                if seeded_from_nav2:
                    log.warning(
                        "Phase-1 gate (NAV2-seeded) rejected at "
                        "epoch %d: %s — extending convergence "
                        "window (attempt %d/%d, no scrub)",
                        n_epochs, last_gate_reason,
                        gate_retries + 1, args.bootstrap_max_retries)
                    converged_at = None
                    prev_pos_ecef = None
                    gate_retries += 1
                    continue

                # LS-init path: legacy scrub-and-retry semantics.
                # Reseed from NAV2 when available — NAV2-pull replaces
                # the 100 m P-blowup (cf. MadHat 2026-04-29 22 m
                # altitude drop in 10 s after a SO_POS reset, where
                # the 100 m σ blew away earned confidence and let
                # single-freq iono bias re-walk).  Keep clock / ISB /
                # ZTD covariances; only position + amb scrub.  Per
                # I-024532-charlie #4.
                reseed = None
                reseed_pos_sigma_m = 100.0  # legacy default if no NAV2
                if (not w2_ok) and nav2_opinion is not None:
                    reseed = nav2_opinion['ecef']
                    nav2_h_acc = nav2_opinion.get('h_acc_m')
                    if nav2_h_acc is not None and nav2_h_acc > 0:
                        # NAV2-pull: use NAV2's claimed horizontal
                        # accuracy as the reseed σ.  Floor at 1 m so
                        # we don't over-trust an optimistic NAV2 hAcc
                        # reading (single-epoch SPP can claim
                        # sub-metre on an exceptionally clean fix).
                        reseed_pos_sigma_m = max(1.0, float(nav2_h_acc))
                log.warning(
                    "Phase-1 gate REJECTED convergence candidate at "
                    "epoch %d: %s — scrubbing and retrying "
                    "(attempt %d/%d, σ_pos=%.1fm%s)",
                    n_epochs, last_gate_reason,
                    gate_retries + 1, args.bootstrap_max_retries,
                    reseed_pos_sigma_m,
                    " from NAV2 hAcc" if reseed is not None else
                    " legacy P-blowup")
                scrub_for_retry(filt, N_BASE, reseed_ecef=reseed,
                                pos_sigma_m=reseed_pos_sigma_m)
                # Also reset the AR state — NL fixes built on the
                # rejected position must not carry over.
                for sv in list(nl_resolver._fixed.keys()):
                    nl_resolver.unfix(sv)
                converged_at = None
                prev_pos_ecef = None
                gate_retries += 1
        else:
            converged_at = None

    run_bootstrap.last_correction_gate_stats = correction_gate.stats.as_dict()
    return None



# ── AntPosEst background thread ─────────────────────────────────────── #


class AntPosEstThread(threading.Thread):
    """Background position refinement with AR.

    Keeps PPPFilter alive after bootstrap (or creates one for warm starts),
    fed decimated observations from the steady-state loop.  Runs MW+NL for
    ambiguity resolution.  Calls position_callback(ecef, sigma_m) when the
    position improves.

    The steady-state loop forwards observations to our queue — we don't
    touch obs_queue directly (single-consumer guarantee for ordering).
    """

    def __init__(self, known_ecef, corrections, stop_event, ape_sm,
                 bootstrap_result=None, position_callback=None,
                 resolved_decimation=10, resolve_threshold=4,
                 ar_elev_mask_deg=20.0,
                 nav2_store=None, nav2_tension_threshold=5.0,
                 nav2_alarm_count=3, systems=None,
                 nl_diag_enabled=False,
                 join_test_enabled=True,
                 wl_only=False,
                 solid_tide=True,
                 antex_parser=None,
                 receiver_antenna_type=None,
                 pcv_enabled=True,
                 clock_model="random_walk",
                 rx_tcxo_adev_1s=None,
                 phase_windup_enabled=False,
                 gmf_enabled=False,
                 slip_rate_limit_s=0.0,
                 ztd_tie_sigma=None,
                 pos_sigma_m=10.0,
                 nav2_anchor_enabled=True,
                 nav2_anchor_max_hacc_m=3.0,
                 pin_position=False,
                 nav_sig_store=None):
        super().__init__(daemon=True, name="AntPosEst")
        # Phase 3 wind-up (Wu 1993): per-SV cumulative wind-up tracker
        # + carrier-phase correction applied before filter.update().
        # Phase 4 GMF (Boehm 2006): hydrostatic + wet mapping functions
        # via PPPFilter._GMF_PROVIDER; the filter's tropo/wet_mapping
        # methods consult it when set (commit c00a6dd).  Both default
        # off to preserve bit-exact pre-port behaviour.
        self._phase_windup_enabled = bool(phase_windup_enabled)
        self._windup_tracker = None
        if self._phase_windup_enabled:
            from peppar_fix.phase_windup import PhaseWindupTracker
            self._windup_tracker = PhaseWindupTracker()
        self._gmf_enabled = bool(gmf_enabled)
        self._ztd_tie_sigma = (float(ztd_tie_sigma)
                               if ztd_tie_sigma is not None
                               and float(ztd_tie_sigma) > 0
                               else None)
        self._gmf_provider = None  # built lazily on first epoch with
                                    # usable position (needs rcv lat/lon)
        # ANTEX parser + receiver antenna type (Phase 2 of obs-model
        # completion plan).  PCV correction applies only when BOTH
        # parser and antenna type are provided AND pcv_enabled is
        # True.  See scripts/antex.py and
        # docs/obs-model-completion-plan.md.
        self._antex = antex_parser
        self._recv_ant_type = receiver_antenna_type
        self._pcv_enabled = bool(pcv_enabled) and antex_parser is not None \
            and receiver_antenna_type is not None
        # WL-only mode: clamp the lifecycle at CONVERGING, skip NL
        # resolution.  See docs/wl-only-foundation.md.
        self._wl_only = bool(wl_only)
        # Solid Earth tide (IERS 2010 Step 1).  On by default per
        # Phase 1 of docs/obs-model-completion-plan.md.  See
        # scripts/solid_tide.py.  When True, each epoch's filter
        # update passes the current tide displacement as
        # receiver_offset_ecef, which adjusts the PREDICTED receiver
        # position for range calculation without modifying the filter
        # state.  Expected delta: ~40 mm on clean-geometry hosts.
        self._solid_tide = bool(solid_tide)
        # Per-epoch PCV counter for the [AntPosEst] status tag.
        self._last_pcv_applied = 0
        self._last_pcv_skipped = 0
        self.obs_queue = queue.Queue(maxsize=50)
        self._corrections = corrections
        self._stop = stop_event
        self._ape_sm = ape_sm
        self._position_callback = position_callback
        self._resolved_decimation = resolved_decimation
        self._resolve_threshold = resolve_threshold  # min NL-fixed SVs for RESOLVED
        self._nav2_store = nav2_store
        self._nav2_tension_threshold = nav2_tension_threshold
        self._nav2_alarm_count = nav2_alarm_count  # consecutive checks before reset
        # NAV2 continuous covariance update: per-epoch variance-
        # weighted Kalman measurement update on the position state,
        # using NAV2's SPP fix and reported hAcc/vAcc as R.  Per Bob's
        # directive 2026-04-30 + I-200128-main + docs/filter-stiffness-
        # redesign.md.  Gated by NAV2 freshness (<10s) and quality
        # (hAcc < threshold).
        #
        # MISNOMER WARNING (I-051234, 2026-05-02): variable names below
        # say "anchor" but the math is variance-weighted update, not
        # truth-pull.  When filter P shrinks to (cm)² and NAV2 R ≈
        # (m)², Kalman gain → 0 and the update contributes ~nothing
        # per epoch.  A static offset between the filter's basin and
        # NAV2's SPP fix is invisible.  A separate true truth-pull
        # anchor (constant fractional pull, σ-blind) is on backlog as
        # I-051234 sub-B.
        self._nav2_anchor_enabled = bool(nav2_anchor_enabled)
        self._nav2_anchor_max_hacc_m = float(nav2_anchor_max_hacc_m)
        # --pin-position with surveyed --known-pos: snapshot the original
        # ECEF here so SECOND_OPINION_POS resets target the pin, not NAV2.
        # See wrongIntBasin-charlie (2026-05-11): NAV2 reset on shared-antenna
        # F9Ts is ~3-4 m biased from CHOKE1 truth, which becomes a basin pull.
        self._pin_position = bool(pin_position)
        self._pin_ecef = (np.asarray(known_ecef, dtype=float).copy()
                          if self._pin_position and known_ecef is not None
                          else None)
        # Phase A.5 of slipDetectUnified-main: log receiver vs engine
        # admit disagreement (see docs/wrong-int-basin-2026-05-11.md +
        # nav_sig_disagree module).  No-op when store is None.
        self._nav_sig_store = nav_sig_store
        from peppar_fix.nav_sig_disagree import NavSigDisagreeMonitor
        self._nav_sig_disagree = NavSigDisagreeMonitor()
        self._systems = systems  # {'gps','gal','bds'} subset — for ISB pinning

        # Initialize from bootstrap result or create fresh filter
        if bootstrap_result is not None and bootstrap_result.ppp_filter is not None:
            self._filt = bootstrap_result.ppp_filter
            self._mw = bootstrap_result.mw_tracker or MelbourneWubbenaTracker()
            self._nl = (bootstrap_result.nl_resolver
                        or NarrowLaneResolver(
                            ar_elev_mask_deg=ar_elev_mask_deg,
                            join_test_enabled=join_test_enabled,
                            ape_state_machine=self._ape_sm,
                            wl_only=self._wl_only))
            # If resolver was inherited from bootstrap, honour the
            # wl_only flag even if bootstrap was built before we
            # knew about it (belt-and-suspenders).
            if self._wl_only:
                self._nl._wl_only = True
            # Resolver inherited from bootstrap: patch in the
            # ape_state_machine reference retroactively so the
            # thin-anchor / strong-anchor regime selection works
            # post-bootstrap.  The bootstrap-construction site
            # doesn't own an AntPosEst machine yet.
            if getattr(self._nl, '_ape_state_machine', None) is None:
                self._nl._ape_state_machine = self._ape_sm
            # Inherit diag logger from bootstrap NL; if missing, attach one.
            if getattr(self._nl, "_nl_diag", None) is None:
                self._nl._nl_diag = NlDiagLogger(enabled=bool(nl_diag_enabled))
            elif nl_diag_enabled:
                # CLI requested on — honour it even if bootstrap's logger
                # was disabled (shouldn't happen, but be forgiving).
                self._nl._nl_diag.enabled = True
            log.info("AntPosEstThread: continuing from bootstrap PPPFilter "
                     "(amb=%d, %s)", len(self._filt.sv_to_idx), self._mw.summary())
        else:
            # Use the shared factory so --nav-sig-gate + --clock-model
            # are applied identically here and at the bootstrap site.
            # AntPosEstThread receives `args` indirectly via the
            # constructor's individual kwargs; we reconstruct a thin
            # shim with the bits the factory reads.
            class _FilterArgsShim:
                pass
            _shim = _FilterArgsShim()
            _shim.clock_model    = clock_model
            _shim.rx_tcxo_adev_1s = rx_tcxo_adev_1s
            _shim.nav_sig_gate   = getattr(self, '_nav_sig_gate_enabled', False)
            _shim.nav_sig_store  = getattr(self, '_nav_sig_store', None)
            _shim.nav_sig_max_age_s = getattr(self, '_nav_sig_max_age_s', 2.0)
            self._filt = _build_ppp_filter(_shim)
            log.info(
                "AntPosEstThread: fresh PPPFilter at known position "
                "(warm start) clock_model=%s%s",
                self._filt.clock_model,
                "" if self._filt.clock_model == "random_walk"
                else f" rx_tcxo_adev_1s={self._filt.rx_tcxo_adev_1s:g}")
            self._filt.initialize(known_ecef, 0.0, systems=self._systems,
                                  pos_sigma_m=pos_sigma_m)
            self._mw = MelbourneWubbenaTracker()
            self._nl = NarrowLaneResolver(
                ar_elev_mask_deg=ar_elev_mask_deg,
                nl_diag=NlDiagLogger(enabled=bool(nl_diag_enabled)),
                join_test_enabled=join_test_enabled,
                ape_state_machine=self._ape_sm,
                wl_only=self._wl_only,
            )

        self._n_epochs = 0
        self._prev_t = None
        self._best_sigma = position_sigma_3d(self._filt.P)
        self._nav2_tension_streak = 0  # consecutive high-tension checks
        self._nav2_cooldown_until = 0  # epoch number: skip checks until this
        # Per-SV ambiguity state machine + the three monitors that
        # replace the old PostFixResidualMonitor's L1/L2/L3 ladder.
        # See docs/sv-lifecycle-and-pfr-split.md.
        self._sv_state = SvStateTracker(wl_only=self._wl_only)
        # MW / NL may be inherited from bootstrap — attach the tracker
        # so their fix-time and unfix-time hooks drive transitions.  For
        # any pre-existing fixes carried over, pre-populate the tracker
        # so downstream monitors see the right state on epoch 0.
        self._mw._sv_state = self._sv_state
        self._nl._sv_state = self._sv_state
        for sv, st in self._mw._state.items():
            if st.get('fixed'):
                rec = self._sv_state.get(sv)
                rec.state = SvAmbState.CONVERGING
        for sv in self._nl._fixed:
            rec = self._sv_state.get(sv)
            rec.state = SvAmbState.ANCHORING
        # FalseFixMonitor is being retired as a demoter (per dayplan
        # I-221332-main / 2026-04-28 evening — same architectural
        # failure mode as wl_drift: PR-domain signal driving the
        # eviction).  observe_only=True keeps it logging the trip
        # candidates for analytical comparison via the v2 BNC validator
        # but skips the tracker transition.  IfStepMonitor below is
        # the canonical NL-layer demoter going forward.
        self._false_fix = FalseFixMonitor(self._sv_state, observe_only=True)
        # IF-residual NL-layer demoter — phase-only post-fix residual
        # with cohort-median subtraction.  Mirrors GfStepMonitor's
        # WL-layer architecture.  See docs/wl-drift-redesign-proposal.md
        # for the WL motivation; the NL story is the same misnomer
        # rerun (PR-domain signal masquerading as ambiguity-error
        # detection).
        self._if_step = IfStepMonitor()
        # SVs currently NL-fixed (ANCHORING ∪ ANCHORED) per the SV
        # state tracker.  Diffed against the snapshot each epoch to
        # drive note_fix / note_unfix on IfStepMonitor.
        self._if_step_prev_nl_fixed: set[str] = set()
        # SecondOpinionPosMonitor — 3D NAV2 vs PPP-AR position witness.
        # Catches 'stable wrong' integer locks that FixSetIntegrityAlarm's
        # internal-consistency triggers and the horizontal-only
        # _check_nav2 watchdog both miss.  See I-011533-main on
        # dayplan/2026-04-28.
        self._second_opinion = SecondOpinionPosMonitor()
        self._setting_drop = SettingSvDropMonitor(self._sv_state)
        # pos_consensus / ztd_consensus thresholds raised from their
        # design-doc defaults (0.20 m / 0.10 m) because lab obs-model
        # bias floor is currently ~3 m on GAL-only with CNES SSR
        # (project_pos_consensus_threshold_too_tight_20260424).  At
        # the design-doc thresholds, monitor fires full-reinit faster
        # than the filter can converge — actively harmful.  Should be
        # tightened back to design defaults once WHU phase biases
        # bring the lab floor into the sub-meter range.
        self._fix_set_integrity = FixSetIntegrityMonitor(
            self._sv_state, ape_state_machine=self._ape_sm,
            pos_consensus_threshold_m=5.0,
            ztd_consensus_threshold_m=2.0,
        )
        # Per-SV post-fix MW residual drift monitor — catches wrong
        # WL integer commits in the "pull phase" (minutes to tens of
        # minutes after a bad integer lands) before the aggregate
        # filter state has absorbed enough bias to trip the host-
        # level ztd_impossible monitor.  See
        # docs/position-strength-metric.md and
        # docs/wl-only-foundation.md for the motivating overnight
        # 2026-04-22/23 analysis.
        self._wl_drift = WlDriftMonitor()
        # GF step demoter — canonical post-fix integrity gate.
        # Uses per-epoch cohort-median Δgf to cancel common-mode iono;
        # trips on per-SV residual exceeding threshold for ≥ 2
        # consecutive epochs.  Replaces the retired MW-residual
        # rolling-mean demoter and the observe-only GF rolling-mean
        # sibling (both retired 2026-04-29 / I-202241).  See
        # ``docs/wl-drift-redesign-proposal.md`` for rationale.
        self._gf_step = GfStepMonitor()
        # Phase-residual admission gate — pre-WL-fix consistency
        # check that prevents PR-driven false admissions.  Per
        # dayplan I-115539 workstream A (2026-04-29 morning).  v1
        # gates MW's admission decision via per-SV post-fit phase
        # residual stats; full Kalman+LAMBDA replacement is a known
        # follow-up (constructor's pr_prior_sigma_m param is
        # reserved for that upgrade — currently no-op).  See
        # docs/wl-admission-phase-only-future.md.
        self._wl_phase_admit = WlPhaseAdmissionGate()
        # SVs currently tracked as WL-fixed by the drift monitor.
        # Diffed against mw._state each epoch to drive note_fix /
        # note_unfix — keeps monitor decoupled from the MW tracker's
        # internals.  Shared between WL and GF monitors since both
        # follow the same WL-fix lifecycle.
        self._wl_drift_prev_fixed: set[str] = set()
        # Fix-life logging — track per-SV monotonic time at entry so
        # we can emit duration_s on exit (per dayplan I-153334).
        # Pruned on exit; size-bounded by the fix-set size.
        self._wl_fix_life_t0: dict[str, float] = {}
        self._wl_fix_life_n_wl: dict[str, int | None] = {}
        # Layer 3 of the per-SV trust architecture
        # (docs/sv-trust-layers.md): after the drift monitor flushes
        # an SV, this gate blocks MW updates for that SV until its
        # elevation has moved ≥ 2° from the flush-time elevation.
        # Breaks the "re-fix the same wrong integer at the same
        # geometry" pattern.
        self._wl_readmit = WlReAdmissionGate()
        # Last-epoch solid Earth tide displacement, cached for the
        # [AntPosEst] status line.  Pair: 3D magnitude and radial
        # projection (~ local up), both in mm.  None when solid
        # tide is disabled or the filter position hasn't converged
        # enough to compute displacement (cold start guard).
        self._last_tide_3d_mm: float | None = None
        self._last_tide_up_mm: float | None = None
        # Bead 4 — promotes ANCHORING → ANCHORED after Δaz ≥ 15°
        # with a clean false-fix window.  Solution-state RESOLVED count (below) reads
        # ANCHORED from the tracker instead of raw NL-fix count.
        self._promoter = AnchoringSvPromoter(self._sv_state)
        self._slip_monitor = CycleSlipMonitor(
            mw_tracker=self._mw, csv_writer=_slip_csv_writer(),
            min_slip_interval_s=float(slip_rate_limit_s))

    @property
    def decimation(self):
        """Effective decimation rate: 1 until integer fixes are
        landing, then resolved_decimation.

        Until AR fixes are established, every observation accelerates
        convergence and WL/NL accumulation.  Once in ANCHORING or
        ANCHORED, we can afford to decimate — the position is stable
        and AR is locked.  ANCHORING (any NL fix) is enough; waiting
        for ANCHORED (geometry-validated) would waste CPU during the
        ~minutes between first fix and first Δaz promotion.

        **Currently disabled** (always returns 1).  Variable
        decimation breaks the implicit "1 epoch == 1 second"
        assumption used by several monitors
        (FixSetIntegrityMonitor sustained/cooldown/escalate windows,
        setting_sv_drop integration, etc.).  Rather than hunt down
        every place that conflates epoch count with wall-clock
        seconds, we run 1:1 for now.  Infrastructure preserved
        (resolved_decimation constructor arg, the state check
        below) so re-enabling is one line once the epoch-vs-time
        assumptions are fixed repo-wide.
        """
        return 1
        # Preserved for re-enable once epoch-vs-time is audited:
        #   if self._ape_sm.state in (AntPosEstState.ANCHORING,
        #                             AntPosEstState.ANCHORED):
        #       return self._resolved_decimation
        #   return 1

    def feed(self, gps_time, observations, obs_counts=None):
        """Called by steady-state loop to forward an observation.

        ``obs_counts`` is an optional dict carrying pre-engine
        counters from realtime_ppp — n_raw / n_off_const / n_single.
        Used to emit [OBS_COUNTS] per dayplan I-143806-main.
        Backward-compatible: callers that don't pass counters get
        the historical bare-tuple semantics.

        Non-blocking — drops if our queue is full (position refinement
        is best-effort, never blocks the servo).
        """
        try:
            self.obs_queue.put_nowait(
                (gps_time, observations, obs_counts))
        except queue.Full:
            pass

    def _check_nav2(self, filt, mw, nl, pos_ecef, sigma_3d, n_nl_fixed):
        """Horizontal antenna-movement watchdog using NAV2 as second opinion.

        NAV2's unique value is independence from our PPP filter — an
        in-receiver single-epoch code-only fix, unaffected by our float
        state, integer fixes, or discipline loop.  If the antenna
        physically moves, NAV2 notices right away while our fixed-pos
        PPP keeps reporting the old location and silently lets dt_rx
        absorb the position error as a clock bias.

        Scope deliberately narrowed (see session 2026-04-18):

        - NOT a wrong-integer detector — LAMBDA ratio/success-rate,
          corner-margin, anti-lock-in blacklist, PFR L1/L2/L3 and the
          CycleSlipMonitor form an internal defense-in-depth stack
          that's finer-grained and faster than a 30 s NAV2 streak.
        - NOT a bootstrap validator — Phase 1 has its own convergence
          checks.  A 20 m horizontal disagreement is still catchable
          here as an antenna-moved event even when NL=0, but there's
          no lower threshold just for bootstrap-sanity purposes.
        - NOT altitude-aware — vertical DOP is ~3× horizontal; PPP
          vertical wander isn't an antenna-moved signature.  Physical
          antenna moves are dominantly horizontal.  On clkPoC3,
          altitude-based RESET cascaded 22 times over 3 h destroying
          convergence repeatedly.

        Single gate: disp_h ≥ 10 m sustained for alarm_count consecutive
        checks.  On reset, reseed lat/lon from NAV2, keep the filter's
        altitude (NAV2 altitude is noisier than NAV2 horizontal).
        """
        opinion = self._nav2_store.get_opinion(max_age_s=30.0)
        if opinion is None:
            return

        nav2_ecef = opinion['ecef']
        nav2_h_acc = opinion['h_acc_m'] or 5.0

        # Project ECEF displacement onto the local tangent plane —
        # horizontal is the component perpendicular to "up" at our
        # position.  For small offsets this is great-circle distance.
        diff_ecef = pos_ecef - nav2_ecef
        up_hat = pos_ecef / np.linalg.norm(pos_ecef)
        vertical = float(np.dot(diff_ecef, up_hat))
        horiz_vec = diff_ecef - vertical * up_hat
        disp_h = float(np.linalg.norm(horiz_vec))
        disp_v = abs(vertical)

        # Tension uses horizontal quantities end-to-end.
        combined_unc = math.sqrt(sigma_3d ** 2 + nav2_h_acc ** 2)
        tension = disp_h / max(combined_unc, 0.1)

        # Single displacement gate.  Real antenna moves manifest as a
        # step, so requiring the streak filters out NAV2 transients.
        ANTENNA_MOVED_HORIZ_M = 10.0

        below_threshold = (tension <= self._nav2_tension_threshold or
                           disp_h < ANTENNA_MOVED_HORIZ_M)
        if below_threshold:
            if self._nav2_tension_streak > 0:
                log.info(
                    "NAV2 horizontal tension resolved: %.1f "
                    "(disp_h=%.1fm disp_v=%.1fm)",
                    tension, disp_h, disp_v)
            self._nav2_tension_streak = 0
            return

        self._nav2_tension_streak += 1
        log.warning(
            "NAV2 horizontal watchdog: tension=%.1f disp_h=%.1fm "
            "disp_v=%.1fm σ=%.2fm hAcc=%.1fm streak=%d/%d "
            "NAV2=(%.6f,%.6f,%.1f) pDOP=%.1f sv=%d",
            tension, disp_h, disp_v, sigma_3d, nav2_h_acc,
            self._nav2_tension_streak, self._nav2_alarm_count,
            opinion['lat'], opinion['lon'], opinion['alt_m'],
            opinion.get('pdop') or 0, opinion.get('num_sv') or 0,
        )

        if self._nav2_tension_streak < self._nav2_alarm_count:
            return

        # Antenna-moved RESET — preserve altitude.
        _, _, alt_ppp = ecef_to_lla(
            pos_ecef[0], pos_ecef[1], pos_ecef[2])
        nav2_lat = opinion['lat']
        nav2_lon = opinion['lon']
        reset_ecef = np.array(
            lla_to_ecef(nav2_lat, nav2_lon, alt_ppp), dtype=float)

        log.warning(
            "NAV2 antenna-moved RESET: AntPosEst horiz offset %.1fm "
            "from NAV2 (%.6f,%.6f) for %d consecutive checks. "
            "Reseeding lat/lon from NAV2 at PPP altitude %.1fm; "
            "unfixing %d NL.",
            disp_h, nav2_lat, nav2_lon,
            self._nav2_tension_streak, alt_ppp, len(nl._fixed),
        )

        for sv in list(nl._fixed.keys()):
            nl.unfix(sv)
        filt.initialize(reset_ecef, 0.0, systems=self._systems)
        self._prev_t = None
        self._best_sigma = 999.0
        self._nav2_tension_streak = 0
        self._nav2_cooldown_until = self._n_epochs + 120
        log.info("NAV2 check cooldown until epoch %d",
                 self._nav2_cooldown_until)
        if self._ape_sm.state in (AntPosEstState.ANCHORING,
                                  AntPosEstState.ANCHORED):
            self._ape_sm.transition(
                AntPosEstState.CONVERGING,
                f"antenna moved (horiz={disp_h:.0f}m)",
            )

    def _apply_false_fix(self, filt, mw, nl, ev):
        """False-fix monitor fired on one SV — the short-term integer
        was rejected as wrong.  The tracker already moved the SV to
        FLOATING; tear down the filter-side state (NL unfix + ambiguity
        inflate + squelch + MW reset) so the AR pipeline re-attempts
        cleanly.
        """
        sv = ev['sv']
        # Provenance: what was this SV's fix-time quality?  Helps spot
        # marginal LAMBDA ratios or corner-margin rounding fixes.
        nl_info = nl._fixed.get(sv, {})
        mw_info = mw._state.get(sv, {})
        provenance = []
        if 'fix_ratio' in nl_info:
            provenance.append(f"NL LAMBDA ratio={nl_info['fix_ratio']:.1f}"
                              f" P={nl_info.get('fix_success_rate', 0):.3f}")
        elif 'fix_n1_frac' in nl_info:
            provenance.append(
                f"NL rounding frac={nl_info['fix_n1_frac']:.3f}"
                f" σ={nl_info.get('fix_sigma_n1', 0):.3f}")
        if 'fix_frac' in mw_info:
            provenance.append(
                f"WL frac={mw_info['fix_frac']:.3f}"
                f" n={mw_info.get('fix_n_epochs', 0)}")
        prov_str = (" {" + "; ".join(provenance) + "}") if provenance else ""
        tag = ev.get('tag', 'false-fix')
        squelch = ev.get('squelch_epochs', 60)
        log.warning(
            "false-fix rejection: %s |PR|=%.2fm > %.2fm (n=%d, elev=%s, %s, squelch=%ds)%s",
            sv, ev['mean_resid_m'], ev['threshold_m'], ev['n'],
            f"{ev['elev_deg']:.0f}°" if ev['elev_deg'] is not None else "?",
            tag, squelch, prov_str,
        )
        # PR residual history at trip — last ~30 samples — so
        # tomorrow's analysis can see the time series leading up to
        # the false-fix decision (per dayplan I-221332-main).
        _hist = ev.get('resid_history_m', [])
        if _hist:
            _hist_str = " ".join(f"{r:+.2f}" for r in _hist)
            log.info(
                "[FALSE_FIX_RESID_HIST] sv=%s history_m=[%s]",
                sv, _hist_str,
            )
        # Tracker already moved the SV to WAITING with cooldown=squelch.
        # NL-side: unfix the integer and blacklist it for the same duration
        # so the resolver's eligibility check stays in sync with the
        # tracker state.
        nl.unfix(sv)
        nl.blacklist(sv, epochs=squelch)
        filt.inflate_ambiguity(sv)
        mw.reset(sv)
        # Bead 4: note the rejection so any subsequent re-fix has to
        # stay clean for clean_window_epochs before being promoted.
        self._promoter.note_false_fix_rejection(sv, self._n_epochs)

    def _apply_setting_sv_drop(self, filt, mw, nl, ev):
        """Setting-SV drop fired — transition the SV out of the fix set
        gracefully.  Tracker has already moved it back to FLOATING.  Release
        the NL integer with gentle covariance growth; keep MW state so
        the SV can be re-acquired if it rises again (a different arc).
        """
        sv = ev['sv']
        reason_frag = (
            f"elev={ev['elev_deg']:.0f}°"
            if ev['reason'] == 'elev_mask' else
            f"|PR|={ev['mean_resid_m']:.2f}m>{ev['threshold_m']:.2f}m"
            f" at elev={ev['elev_deg']:.0f}°"
        )
        log.info("setting-SV drop: %s (%s)", sv, reason_frag)
        nl.unfix(sv)
        # Preserve MW state — a drop is "this integer is done," not
        # "this SV is bad."  Next arc or re-acquisition re-uses it.

    def _apply_integrity_trip(self, filt, mw, nl, ev):
        """Fix-set integrity monitor tripped.  Action depends on
        ``ev['reason']``:

          * ``reason='window_rms'`` / ``'anchor_collapse'`` — full
            filter re-init at current AR position.  Drop NL fixes,
            reset MW entirely, transition all SVs to FLOATING,
            re-initialize PPPFilter.  Assumes systemic failure that
            the per-SV monitors can't fix.
          * ``reason='ztd_impossible'`` / ``'ztd_cycling'`` —
            **surgical ZTD-only reset (I-133619-main P3).**  Reset
            ``x[IDX_ZTD]`` to 0 and ``P[IDX_ZTD,IDX_ZTD]`` to
            ``filt._ztd_sigma_m ** 2``; keep position / clock / ISB /
            NL fixes / MW state / SV states untouched.

            Why: 2026-04-30 overnight evidence (37+62+82 trips across
            the lab fleet) showed ``|Δalt|`` at recovery = 0.05-0.06 m
            and ``Δhoriz`` = 0.02 m on every ZTD-class trip — i.e.
            the position/ambiguity scaffold was correct at trip time;
            only ZTD was off (the cross-SV mean of the missing
            SSR-PB-coverage error has no other state to land in
            once NAV2 anchor pins position).  The legacy NL-only-
            revert was throwing away genuinely-good NL fixes and
            ANCHORED state on every trip, costing the trust scaffold
            ~7 minutes of accumulation each time.  The legacy
            ztd_cycling WL flush was based on a "wrong WL integer
            traps NL search" hypothesis that the SSR-PB-gap analysis
            invalidates: WL integers are correct; ZTD just keeps
            re-absorbing the bias mean.  Surgical reset stops the
            collateral damage; the trip detector still fires (so
            the SSR-PB-gap signal stays measurable in the log) but
            the recovery is essentially free for the trust scaffold.

        Emits the single [FIX_SET_INTEGRITY] TRIPPED log line
        (monitor itself stays silent per the "monitor only speaks
        on trip, once" interface contract).
        """
        pos_ecef = filt.x[:3].copy()
        reason = ev.get('reason', 'window_rms')
        if reason == 'anchor_collapse':
            params = (
                f"anchor_collapse_epochs={ev['anchor_collapse_epochs']} "
                f"since_epoch={ev['since_epoch']}"
            )
        elif reason in ('ztd_impossible', 'ztd_cycling'):
            params = (
                f"ztd_m={ev['ztd_m']:+.3f} "
                f"threshold_m=±{ev['threshold_m']:.3f} "
                f"sustained_epochs={ev['sustained_epochs']} "
                f"recent_trip_count={ev.get('recent_trip_count', 1)}"
            )
        elif reason == 'pos_consensus':
            params = (
                f"delta_m={ev['delta_m']:.3f} "
                f"threshold_m={ev['threshold_m']:.3f} "
                f"sustained_epochs={ev['sustained_epochs']}"
            )
        elif reason == 'ztd_consensus':
            params = (
                f"delta_m={ev['delta_m']:+.3f} "
                f"threshold_m=±{ev['threshold_m']:.3f} "
                f"sustained_epochs={ev['sustained_epochs']}"
            )
        else:
            params = (
                f"window_rms_m={ev['window_rms_m']:.2f} "
                f"rms_m={ev['rms_m']:.2f} "
                f"n_samples={ev['n_samples']}"
            )
        log.warning(
            "[FIX_SET_INTEGRITY] TRIPPED reason=%s %s at pos=%s",
            reason, params, pos_ecef.tolist(),
        )
        # Pre-revert SV-state snapshot — captures who was ANCHORING /
        # ANCHORED at trip time so we can post-hoc evaluate whether
        # the trip was justified (per dayplan I-221332-main).  Names
        # only; counts come from SvStateTracker's existing summary.
        try:
            _states_at_trip = {
                _st.name: sorted(self._sv_state.svs_in(_st))
                for _st in (
                    SvAmbState.ANCHORED,
                    SvAmbState.ANCHORING,
                    SvAmbState.CONVERGING,
                )
            }
            log.warning(
                "[FIX_SET_INTEGRITY] pre-revert states: anchored=%s "
                "anchoring=%s converging=%s",
                _states_at_trip['ANCHORED'],
                _states_at_trip['ANCHORING'],
                _states_at_trip['CONVERGING'],
            )
        except Exception:
            log.exception("pre-revert state snapshot failed (non-fatal)")

        if reason in ('ztd_impossible', 'ztd_cycling'):
            # Surgical ZTD-only reset (I-133619-main P3).  Per
            # 2026-04-30 overnight evidence: ZTD-class trips
            # recover with |Δalt|=0.05m and Δhoriz=0.02m, meaning
            # the position/ambiguity scaffold is correct at trip
            # time and only ZTD needs the reset.  Surgical reset
            # preserves the trust scaffold (NL fixes, ANCHORED
            # state, MW tracker).  Trip detector still fires so
            # the SSR-PB-gap signal stays measurable, but the
            # recovery is free for the trust scaffold.
            ztd_idx = getattr(filt, 'IDX_ZTD', PPP_IDX_ZTD)
            ztd_sigma_m = float(getattr(filt, '_ztd_sigma_m', 0.2))
            if filt.x.shape[0] > ztd_idx:
                filt.x[ztd_idx] = 0.0
                filt.P[ztd_idx, ztd_idx] = ztd_sigma_m ** 2
            log.info(
                "ZTD trip recovery (surgical, P3): reset "
                "x[IDX_ZTD]=0 and σ_ZTD=%.3fm; "
                "kept %d NL fix(es), MW state, SV states, "
                "position, clock, ISB intact "
                "(recent ZTD trips=%d)",
                ztd_sigma_m, len(nl._fixed),
                ev.get('recent_trip_count', -1),
            )
            self._fix_set_integrity.record_trip(self._n_epochs)
            return

        if reason == 'ztd_consensus':
            # Peer-cohort agreement that ZTD is wrong.  This is a
            # stronger external signal than ztd_impossible's local
            # threshold breach; the cohort sees the same drift, so
            # something beyond local SSR-PB-gap may be at play.
            # Keep the legacy NL-revert + ZTD-reset behavior here
            # until we have evidence that surgical-only is enough
            # for cohort-confirmed cases.
            revert_count = 0
            for sv in list(nl._fixed.keys()):
                nl.unfix(sv)
                revert_count += 1
                cur = self._sv_state.state(sv)
                if cur in (SvAmbState.ANCHORING, SvAmbState.ANCHORED):
                    try:
                        self._sv_state.transition(
                            sv, SvAmbState.FLOATING,
                            epoch=self._n_epochs,
                            reason="ztd_consensus:revert_to_wl",
                        )
                    except Exception:
                        pass
            ztd_idx = getattr(filt, 'IDX_ZTD', PPP_IDX_ZTD)
            ztd_sigma_m = float(getattr(filt, '_ztd_sigma_m', 0.2))
            if filt.x.shape[0] > ztd_idx:
                filt.x[ztd_idx] = 0.0
                filt.P[ztd_idx, ztd_idx] = ztd_sigma_m ** 2
            log.info(
                "ZTD consensus recovery: reverted %d NL fixes, "
                "reset ZTD state (σ=%.3fm), preserved "
                "MW/position/clock",
                revert_count, ztd_sigma_m,
            )
            if self._ape_sm.state in (AntPosEstState.ANCHORING,
                                      AntPosEstState.ANCHORED):
                self._ape_sm.transition(
                    AntPosEstState.CONVERGING,
                    "ztd-consensus trip — NL revert, WL preserved",
                )
            self._fix_set_integrity.record_trip(self._n_epochs)
            return

        # Full re-init path for window_rms and anchor_collapse.
        # Drop all NL fixes (tracker → FLOATING for each via resolver hook),
        # clear MW state entirely (big reset), re-seed filter.
        for sv in list(nl._fixed.keys()):
            nl.unfix(sv)
        for sv in list(mw._state.keys()):
            mw.reset(sv)
            # MW.reset() intentionally doesn't touch the tracker; after
            # a fix-set-wide re-init every SV's state is meaningless,
            # so flatten to FLOATING explicitly.
            cur = self._sv_state.state(sv)
            if cur is not SvAmbState.FLOATING:
                try:
                    self._sv_state.transition(
                        sv, SvAmbState.FLOATING,
                        epoch=self._n_epochs,
                        reason="fix_set_integrity_trip:reinit",
                    )
                except Exception:
                    # WAITING → FLOATING is legal per the edge set, but
                    # defensive coding keeps the re-init path robust.
                    pass
        filt.initialize(pos_ecef, 0.0, systems=self._systems)
        self._prev_t = None
        self._best_sigma = 999.0
        if self._ape_sm.state in (AntPosEstState.ANCHORING,
                                  AntPosEstState.ANCHORED):
            self._ape_sm.transition(
                AntPosEstState.CONVERGING,
                "fix-set integrity trip — re-bootstrap",
            )
        self._fix_set_integrity.record_trip(self._n_epochs)

    @staticmethod
    def _null_mode_sigma_max(filt):
        """Largest σ (√ of largest eigenvalue) across P's coupled
        base-state block (position + clock + ISBs + ZTD) in meters.

        Captures the null-mode excitation signal: when
        (δclk, δZTD·m_wet, δN_common) drift together along the
        filter's near-rank-deficient direction, the covariance
        grows along that direction; the block's largest eigenvalue
        tracks it.  See docs/glossary.md 'null mode' for the
        mechanism and the 2026-04-23 PRIDE-ablation arc for the
        empirical characterization.

        Returns max σ in meters, or None when the filter hasn't
        built enough state for the computation (early cold-start).
        Diagnostic only — does not drive any filter action.
        """
        if filt is None or not hasattr(filt, 'P') or filt.P is None:
            return None
        # Base-state block: [X, Y, Z, clk, ISB_GAL, ISB_BDS, ZTD].
        # Skip ambiguities — interpretation gets murky at dim 20+
        # and the clock/ZTD part already captures the null-mode
        # direction per Bravo's state-trajectory data (clock ±12m,
        # ZTD ±2m co-drift).
        n = filt.P.shape[0]
        base_dim = min(N_BASE, n)
        if base_dim < 4:
            return None
        P_block = filt.P[:base_dim, :base_dim]
        try:
            # eigvalsh returns ascending; take last for max.
            max_eig = float(np.linalg.eigvalsh(P_block)[-1])
        except np.linalg.LinAlgError:
            return None
        if max_eig <= 0.0:
            return 0.0
        return math.sqrt(max_eig)

    def run(self):
        log.info("AntPosEstThread started (resolved_decimation=%d, resolve_threshold=%d)",
                 self._resolved_decimation, self._resolve_threshold)
        try:
            self._run_inner()
        except Exception:
            log.exception("AntPosEstThread crashed")

    def _run_inner(self):
        filt = self._filt
        mw = self._mw
        nl = self._nl
        corrections = self._corrections
        gate = CorrectionFreshnessGate()
        last_epoch_mono = time.monotonic()
        n_timeouts = 0
        n_corr_skip = 0

        while not self._stop.is_set():
            try:
                _qitem = self.obs_queue.get(timeout=30)
                # 3-tuple (with counters) or legacy 2-tuple — unpack
                # defensively so older producers / replay tooling that
                # haven't been updated still work.
                if len(_qitem) == 3:
                    gps_time, observations, obs_counts = _qitem
                else:
                    gps_time, observations = _qitem
                    obs_counts = None
            except queue.Empty:
                n_timeouts += 1
                idle_s = time.monotonic() - last_epoch_mono
                log.warning(
                    "AntPosEstThread heartbeat: no observations for %.0fs "
                    "(timeouts=%d, epochs=%d, corr_skips=%d, qsize=%d, "
                    "state=%s, prev_t=%s, amb=%d, stop=%s)",
                    idle_s, n_timeouts, self._n_epochs, n_corr_skip,
                    self.obs_queue.qsize(),
                    self._ape_sm.state.value,
                    self._prev_t,
                    len(filt.sv_to_idx) if filt.x is not None else -1,
                    self._stop.is_set(),
                )
                continue

            n_timeouts = 0
            last_epoch_mono = time.monotonic()

            # Correction freshness check (use relaxed thresholds — we're
            # background, not real-time)
            ok, _, _ = gate.accept(
                corrections,
                max_broadcast_age_s=600,
                require_ssr=False,
                max_ssr_age_s=600,
            )
            if not ok:
                n_corr_skip += 1
                continue

            # EKF predict
            if self._prev_t is not None:
                dt = (gps_time - self._prev_t).total_seconds()
                if dt <= 0 or dt > 120:
                    self._prev_t = gps_time
                    continue
                filt.predict(dt)
            self._prev_t = gps_time

            # Manage ambiguities — slip detector runs all four checks
            # (UBX locktime, arc gap, geometry-free jump, MW residual
            # jump) and flushes every per-SV phase-like state on any
            # detected slip.  Shared clock/ISB/ZTD and receiver TCXO
            # state are intentionally untouched.
            current_svs = {o['sv'] for o in observations}
            elevations = _compute_sv_elevations(
                filt, corrections, observations, gps_time)
            azimuths = _compute_sv_azimuths(
                filt, corrections, observations, gps_time)
            ipp_szas = _compute_sv_ipp_szas(
                filt, azimuths, elevations, gps_time)
            slip_events = self._slip_monitor.check(
                observations, gps_time.timestamp(), self._n_epochs,
                elevations=elevations,
                azimuths=azimuths, ipp_szas=ipp_szas)
            for ev in slip_events:
                # PR-only slips (mw_jump alone) are PR-noise false
                # positives; don't flush WL state or wind-up tracker.
                # See is_pr_only_slip() docstring + I-155354 fix-I +
                # docs/wl-integer-vs-pride-comparison.md.
                if is_pr_only_slip(ev.reasons):
                    log.info("[SLIP_FILTERED] sv=%s mw_jump-only "
                             "conf=%s mw=%s — PR-only, ambiguity "
                             "preserved",
                             ev.sv, ev.confidence,
                             f"{ev.mw_jump_cyc:.2f}c"
                             if ev.mw_jump_cyc is not None else "-")
                else:
                    flush_sv_phase(
                        ev.sv, filt=filt, mw_tracker=mw,
                        nl_resolver=nl,
                        slip_monitor=self._slip_monitor,
                        sv_state=self._sv_state,
                        confidence=ev.confidence,
                        reason="|".join(ev.reasons), epoch=self._n_epochs)
                    # Wind-up tracker state is per-SV cumulative; drop it
                    # on slip so the next update seeds fresh (absolute
                    # zero absorbed into the re-floating ambiguity).
                    if self._windup_tracker is not None:
                        self._windup_tracker.reset(ev.sv)
                log.info("slip: sv=%s reasons=%s conf=%s lock=%.0fms"
                         " cno=%.1f elev=%s az=%s ipp_sza=%s gap=%s gf=%s mw=%s",
                         ev.sv, ",".join(ev.reasons), ev.confidence,
                         ev.lock_ms, ev.cno,
                         f"{ev.elevation_deg:.0f}°" if ev.elevation_deg is not None else "?",
                         f"{ev.azimuth_deg:.0f}°" if ev.azimuth_deg is not None else "?",
                         f"{ev.ipp_sza_deg:.0f}°" if ev.ipp_sza_deg is not None else "?",
                         f"{ev.gap_s:.2f}s" if ev.gap_s is not None else "-",
                         f"{ev.gf_jump_m*100:.1f}cm" if ev.gf_jump_m is not None else "-",
                         f"{ev.mw_jump_cyc:.2f}c" if ev.mw_jump_cyc is not None else "-")
                peer_publisher.publish_slip_event(
                    sv=ev.sv, reasons=ev.reasons, conf=ev.confidence,
                    elev_deg=ev.elevation_deg,
                    azimuth_deg=ev.azimuth_deg,
                    ipp_sza_deg=ev.ipp_sza_deg,
                    lock_duration_ms=int(ev.lock_ms) if ev.lock_ms is not None else None,
                    gf_jump_m=ev.gf_jump_m, mw_jump_cyc=ev.mw_jump_cyc,
                )

            for obs in observations:
                sv = obs['sv']
                if sv not in filt.sv_to_idx and obs.get('phi_if_m') is not None:
                    sat_pos, sat_clk = corrections.sat_position(sv, gps_time)
                    if sat_pos is not None:
                        N_init = obs['pr_if'] - obs['phi_if_m']
                        filt.add_ambiguity(sv, N_init)

            filt.prev_obs = {o['sv']: o for o in observations}
            for sv in list(filt.sv_to_idx.keys()):
                if sv not in current_svs:
                    filt.remove_ambiguity(sv)

            # Solid Earth tide displacement (IERS 2010 Step 1).  Added
            # to the predicted receiver position for range calculation;
            # does NOT modify the filter state.  Guard against cold-
            # start where filt.x[:3] may be near-zero (no position yet)
            # — the tide module needs a physical Earth-radius station
            # to compute the r_hat unit vector.  100 km floor is a
            # comfortable margin above filter init noise.
            tide_offset = None
            self._last_tide_3d_mm = None
            self._last_tide_up_mm = None
            pos_ecef = filt.x[:3]
            pos_norm = float(np.linalg.norm(pos_ecef))
            pos_usable = pos_norm > 100_000.0
            if self._solid_tide and pos_usable:
                tide_offset = solid_tide_displacement(gps_time, pos_ecef)
                # Cache tide magnitude + radial projection for the
                # AntPosEst status line.  Radial ≈ local up for a
                # near-spherical station position; good to 0.1 mm.
                r_hat = pos_ecef / pos_norm
                self._last_tide_3d_mm = 1000.0 * float(
                    np.linalg.norm(tide_offset))
                self._last_tide_up_mm = 1000.0 * float(
                    np.dot(tide_offset, r_hat))

            # GMF update (Phase 4 of obs-model plan).  The filter's
            # tropo/wet_mapping methods consult
            # PPPFilter._GMF_PROVIDER when set (commit c00a6dd).
            # Build the provider lazily on first epoch with a usable
            # position — needs receiver lat/lon.  Update its epoch
            # every tick.
            if self._gmf_enabled and pos_usable:
                if self._gmf_provider is None:
                    from peppar_fix.gmf import GMFProvider
                    from solve_ppp import PPPFilter as _PF
                    lat_deg, lon_deg, height_m = ecef_to_lla(
                        pos_ecef[0], pos_ecef[1], pos_ecef[2])
                    lat_r = math.radians(lat_deg)
                    lon_r = math.radians(lon_deg)
                    self._gmf_provider = GMFProvider(
                        lat_r, lon_r, height_m)
                    _PF._GMF_PROVIDER = self._gmf_provider
                    log.info(
                        "[GMF] provider built (lat=%.4f° lon=%.4f° h=%.0f m)",
                        math.degrees(lat_r), math.degrees(lon_r), height_m)
                # Compute MJD from gps_time.  GPS epoch 1980-01-06 →
                # MJD 44244.  gps_time is a UTC-aware datetime; MJD
                # is a UT1 time tag but UTC-vs-UT1 gap < 1s doesn't
                # affect GMF's day-of-year interpolation.
                _unix = gps_time.timestamp()
                _mjd = 40587.0 + _unix / 86400.0
                self._gmf_provider.update_epoch(_mjd)

            # PCV correction (Phase 2 of obs-model plan).  Per-SV
            # antenna phase-center correction applied to obs pr_if +
            # phi_if_m in-place.  Requires ANTEX parser + receiver
            # antenna type + usable filter position.  Sun ECEF shared
            # with solid tide (computed once per epoch).
            self._last_pcv_applied = 0
            self._last_pcv_skipped = 0
            # Wind-up + PCV share sun_ecef, computed once per epoch.
            _sun_ecef_cached = None
            if (self._pcv_enabled or self._phase_windup_enabled) and pos_usable:
                _sun_ecef_cached = sun_pos_ecef(gps_time)

            # Phase wind-up (Phase 3 of obs-model plan).  Per-SV
            # cumulative wind-up tracker; apply correction to
            # phi_if_m in-place before filter.update().  Formula +
            # sign convention in peppar_fix/phase_windup.py;
            # lambda_eff for IF is c / (f1 + f2).
            if self._phase_windup_enabled and pos_usable and _sun_ecef_cached is not None:
                for obs in observations:
                    sv = obs['sv']
                    if obs.get('phi_if_m') is None:
                        continue
                    wl_f1 = obs.get('wl_f1')
                    wl_f2 = obs.get('wl_f2')
                    if not wl_f1 or not wl_f2:
                        continue
                    sat_pos, _ = corrections.sat_position(sv, gps_time)
                    if sat_pos is None:
                        continue
                    self._windup_tracker.update(
                        sv, np.asarray(sat_pos),
                        np.asarray(_sun_ecef_cached),
                        np.asarray(pos_ecef))
                    f1 = C / wl_f1
                    f2 = C / wl_f2
                    lambda_eff_m = C / (f1 + f2)
                    obs['phi_if_m'] += (
                        self._windup_tracker.correction_m(sv, lambda_eff_m))

            if self._pcv_enabled and pos_usable:
                sun_ecef = _sun_ecef_cached
                for obs in observations:
                    sv = obs['sv']
                    sat_pos, _ = corrections.sat_position(sv, gps_time)
                    if sat_pos is None:
                        self._last_pcv_skipped += 1
                        continue
                    if obs.get('pr_if') is None or obs.get('phi_if_m') is None:
                        self._last_pcv_skipped += 1
                        continue
                    delta, ok = compute_pcv_correction(
                        obs, np.asarray(sat_pos), pos_ecef,
                        self._antex, self._recv_ant_type, gps_time,
                        sun_pos_ecef=sun_ecef)
                    if ok:
                        obs['pr_if'] += delta
                        obs['phi_if_m'] += delta
                        self._last_pcv_applied += 1
                    else:
                        self._last_pcv_skipped += 1

            # EKF update — position-side per-filter gate
            # (I-175645-charlie); see obs_for_position() at module top.
            pos_observations = obs_for_position(observations)

            # Phase A.5 of slipDetectUnified-main: log disagreement
            # between receiver's NAV-SIG.prUsed verdict and engine's
            # admit decision.  Per (sv, sig_name) granularity from
            # f1_sig_name / f2_sig_name carried on each obs.  No
            # behavior change — pure logger.  No-op when store is None.
            if self._nav_sig_store is not None:
                admit_set: dict[tuple[str, str], dict] = {}
                for o in pos_observations:
                    sv = o.get('sv')
                    if sv is None:
                        continue
                    cno = o.get('cno')
                    pr_ok = o.get('ar_phase_bias_ok')
                    reason = ("ar_phase_bias_ok" if pr_ok
                              else "admit")
                    for f_key, sig_key, lock_key in (
                            ('f1', 'f1_sig_name', 'f1_lock_ms'),
                            ('f2', 'f2_sig_name', 'f2_lock_ms')):
                        sig_name = o.get(sig_key)
                        if not sig_name:
                            continue
                        admit_set[(sv, sig_name)] = {
                            'cno': cno,
                            'lock_time_ms': o.get(lock_key),
                            'our_admission_reason': reason,
                        }
                try:
                    self._nav_sig_disagree.check_epoch(
                        self._n_epochs, admit_set, self._nav_sig_store)
                except Exception:
                    # Pure logger — never let a bug here break the EKF.
                    log.exception(
                        "NavSigDisagreeMonitor.check_epoch failed; "
                        "continuing without it.")

            n_used, resid, sys_counts = filt.update(
                pos_observations, corrections, gps_time, clk_file=corrections,
                receiver_offset_ecef=tide_offset)
            if n_used < 4:
                continue

            # Soft prior on residual ZTD (post-update rank-1 EKF
            # pseudo-measurement z=0).  Constrains the position-altitude/
            # ZTD/clock null-mode that otherwise lets the filter wander
            # m-scale.  No-op when self._ztd_tie_sigma is None or <= 0.
            if getattr(self, "_ztd_tie_sigma", None):
                filt.apply_ztd_tie(self._ztd_tie_sigma)

            # NAV2 continuous covariance update (post-update 3D EKF
            # variance-weighted measurement).  Per Bob's directive
            # 2026-04-30 + I-200128-main.  Gated by NAV2 freshness +
            # quality.  See solve_ppp.PPPFilter.apply_nav2_anchor for
            # the misnomer note: this is variance-weighted Kalman, not
            # a truth-pull anchor.  Static offset between filter and
            # NAV2 is invisible once filter P has tightened.
            if (self._nav2_anchor_enabled
                    and self._nav2_store is not None):
                _nav2_op = self._nav2_store.get_opinion(max_age_s=10.0)
                if (_nav2_op is not None
                        and _nav2_op.get('fix_type') == 3
                        and _nav2_op.get('h_acc_m') is not None
                        and _nav2_op['h_acc_m']
                        < self._nav2_anchor_max_hacc_m):
                    filt.apply_nav2_anchor(
                        np.asarray(_nav2_op['ecef'], dtype=float),
                        float(_nav2_op['h_acc_m']),
                        (float(_nav2_op['v_acc_m'])
                         if _nav2_op.get('v_acc_m') is not None
                         else None),
                    )

            self._n_epochs += 1

            # Phase-residual admission gate ingest — feed this epoch's
            # post-fit phase residuals into WlPhaseAdmissionGate so it
            # has a current view when MW's admission decisions are
            # checked below.  Per dayplan I-115539 / 2026-04-29.
            _gate_labels = getattr(filt, 'last_residual_labels', [])
            for _lab, _r in zip(_gate_labels, resid if resid is not None else ()):
                if len(_lab) >= 2 and _lab[1] == 'phi':
                    self._wl_phase_admit.ingest(_lab[0], float(_r))
            # Drop history for SVs not observed this epoch — the gate's
            # per-SV deque should reflect actually-current data, not
            # stale residuals from arcs that have set.
            _observed_now = {o['sv'] for o in observations}
            self._wl_phase_admit.evict_unobserved(_observed_now)

            # MW wide-lane update.  Tell MW the current epoch so its
            # tracker-driven transitions (FLOATING → CONVERGING on fix) log
            # with a meaningful epoch field.
            mw._current_epoch = self._n_epochs
            nl._epoch = self._n_epochs  # resolver also uses _epoch for logs
            readmit_blocked: list[str] = []
            # Per-SV GF observation for this epoch.  Populated alongside
            # the MW update; consumed by the GF step demoter below.
            # Pure phase, geometry-free.  Per-filter gate
            # (I-175645-charlie): MW combination needs phase biases on
            # both bands, so iterate only the position-filtered list.
            gf_now: dict[str, float] = {}
            for obs in pos_observations:
                sv = obs['sv']
                phi1 = obs.get('phi1_cyc')
                phi2 = obs.get('phi2_cyc')
                pr1 = obs.get('pr1_m')
                pr2 = obs.get('pr2_m')
                wl1 = obs.get('wl_f1')
                wl2 = obs.get('wl_f2')
                if all(v is not None for v in (phi1, phi2, pr1, pr2, wl1, wl2)):
                    # Layer 3 re-admission gate: skip MW update if the
                    # drift monitor previously flushed this SV and its
                    # elevation hasn't moved enough yet.  See
                    # docs/sv-trust-layers.md.
                    if self._wl_readmit.is_blocked(sv, elevations.get(sv)):
                        readmit_blocked.append(sv)
                        continue
                    f1_hz = C / wl1
                    f2_hz = C / wl2
                    mw.update(sv, phi1, phi2, pr1, pr2, f1_hz, f2_hz)
                    gf_now[sv] = gf_phase_m(phi1, phi2, wl1, wl2)

            # WL drift monitor: detect wrong WL integer commits via
            # sustained post-fix MW residual drift.  Runs after MW
            # updates so the residuals we ingest are current-epoch.
            # Acting here (before NL attempt) keeps the NL path
            # from seeing a suspect WL this epoch.
            fixed_now = {sv for sv, st in mw._state.items() if st.get('fixed')}
            # Phase-residual admission gate — for each NEWLY-fixed SV
            # this epoch, check the per-SV phase consistency before
            # the fix is allowed to land in the lifecycle.  If the
            # gate refuses, revert the MW state so the SV stays in
            # FLOATING / CONVERGING and MW will try again at the
            # next epoch.  Per dayplan I-115539 workstream A.
            _newly_fixed = fixed_now - self._wl_drift_prev_fixed
            for sv in list(_newly_fixed):
                if not self._wl_phase_admit.is_phase_consistent(sv):
                    detail = self._wl_phase_admit.evaluation_detail(sv)
                    if detail is not None:
                        log.warning(
                            "[WL_PHASE_ADMIT_BLOCK] %s mean=%+.4fm "
                            "std=%.4fm mean_thr=±%.3fm std_thr=±%.3fm "
                            "cohort_med=%+.4fm (n=%d): MW proposed "
                            "admission rejected; phase residual not "
                            "yet consistent",
                            sv, detail['mean_m'], detail['std_m'],
                            detail['threshold_m'],
                            detail['std_threshold_m'],
                            detail['cohort_median_m'], detail['n_samples'],
                        )
                    # Revert MW state — clear the fix so the SV is not
                    # admitted this epoch.  MW will keep accumulating
                    # samples and may try to fix again next epoch.
                    _st = mw._state[sv]
                    _st['fixed'] = False
                    _st['n_wl'] = None
                    _st.pop('fix_frac', None)
                    _st.pop('fix_n_epochs', None)
                    fixed_now.discard(sv)
                    _newly_fixed.discard(sv)
            for sv in _newly_fixed:
                fix_n_wl = mw._state[sv].get('n_wl')
                self._wl_drift.note_fix(sv, n_wl=fix_n_wl)
                # Initialise the GF step demoter for this SV.  Skip
                # when the GF observation is missing this epoch (would
                # happen if the SV's MW updated from a prior-epoch
                # latch with the current obs missing fields).
                _gf_ref = gf_now.get(sv)
                if _gf_ref is not None:
                    self._gf_step.note_fix(sv, gf_initial_m=_gf_ref)
                # Newly-fixed SV passed the re-admission gate (or was
                # never held).  Clear any prior hold so a subsequent
                # drift-flush-readmit cycle starts fresh.
                self._wl_readmit.note_admitted(sv)
                # Fix-life logging — entry.  Records integer + elev +
                # consistency level for post-hoc analysis of K_short /
                # threshold tuning (per dayplan I-153334).
                _life_elev = elevations.get(sv)
                _life_cons = self._wl_drift.consistency_level(sv)
                _life_hist = self._wl_drift.integer_history(sv)
                log.info(
                    "[WL_FIX_LIFE] event=enter sv=%s n_wl=%s elev=%s "
                    "consistency=%s int_history=%s",
                    sv, fix_n_wl,
                    f"{_life_elev:.1f}" if _life_elev is not None else "?",
                    _life_cons, _life_hist,
                )
                self._wl_fix_life_t0[sv] = time.monotonic()
                self._wl_fix_life_n_wl[sv] = fix_n_wl
            for sv in self._wl_drift_prev_fixed - fixed_now:
                self._wl_drift.note_unfix(sv)
                self._gf_step.note_unfix(sv)
                # Fix-life logging — exit (slip, dropout, or other
                # exit path that didn't go through gf_step).
                _t0 = self._wl_fix_life_t0.pop(sv, None)
                _n_wl = self._wl_fix_life_n_wl.pop(sv, None)
                _life_elev = elevations.get(sv)
                _dur = time.monotonic() - _t0 if _t0 is not None else None
                log.info(
                    "[WL_FIX_LIFE] event=exit sv=%s n_wl=%s elev=%s "
                    "duration_s=%s reason=other consistency=%s",
                    sv, _n_wl,
                    f"{_life_elev:.1f}" if _life_elev is not None else "?",
                    f"{_dur:.1f}" if _dur is not None else "?",
                    self._wl_drift.consistency_level(sv),
                )
            drift_actions: list[str] = []

            # GF step demoter — phase-only, cohort-median Δgf
            # detection.  Per I-194752-main, this replaces the
            # MW-residual rolling-mean approach that the BNC
            # validation showed is uncorrelated with real slips
            # (Z = -0.17, p = 0.86).  Cohort-median across the
            # currently-fixed-SV cohort cancels common-mode
            # ionospheric drift (the dominant noise source on
            # long-fixed SVs) without an explicit Klobuchar / SSR
            # iono model.  Trips on per-SV residual exceeding
            # threshold (4 cm) for ≥ 2 consecutive epochs.
            gf_step_events = self._gf_step.update(gf_now)
            for ev in gf_step_events:
                sv = ev['sv']
                flush_elev = elevations.get(sv)
                log.warning(
                    "[GF_STEP] %s residual=%+.4fm cohort_median=%+.4fm "
                    "delta_gf=%+.4fm (n_cohort=%d, %d-epoch sustained, "
                    "thr=±%.3fm): flushing MW, demoting to FLOATING, "
                    "gate@elev=%s",
                    sv, ev['residual_m'], ev['cohort_median_m'],
                    ev['delta_gf_m'], ev['cohort_size'],
                    ev['consecutive_epochs'], ev['threshold_m'],
                    f"{flush_elev:.1f}°" if flush_elev is not None else "?",
                )
                # Fix-life exit before MW reset clears state.
                _t0 = self._wl_fix_life_t0.pop(sv, None)
                _n_wl_at_entry = self._wl_fix_life_n_wl.pop(sv, None)
                _dur = time.monotonic() - _t0 if _t0 is not None else None
                log.info(
                    "[WL_FIX_LIFE] event=exit sv=%s n_wl=%s elev=%s "
                    "duration_s=%s reason=gf_step consistency=%s",
                    sv, _n_wl_at_entry,
                    f"{flush_elev:.1f}" if flush_elev is not None else "?",
                    f"{_dur:.1f}" if _dur is not None else "?",
                    self._wl_drift.consistency_level(sv),
                )
                mw.reset(sv)
                self._wl_drift.note_unfix(sv)
                self._gf_step.note_unfix(sv)
                # GF step is a real phase discontinuity (L1 or L5
                # slipped) — wipe the SV's NL trust history so a
                # post-slip re-admission is gated as NEW rather than
                # held to its pre-slip integer.  I-172719.
                nl.note_slip(sv)
                # Layer 3: take a re-admission hold at the current
                # elevation.  MW updates for this SV are skipped
                # until elevation moves ≥ 2°.
                self._wl_readmit.note_flush(sv, flush_elev)
                fixed_now.discard(sv)
                # Demote the per-SV state.  CONVERGING → FLOATING is
                # a legal edge in the lifecycle; ANCHORING/ANCHORED
                # shouldn't be reachable in wl-only mode but the
                # path is defensive.
                try:
                    cur = self._sv_state.state(sv)
                    if cur in (SvAmbState.CONVERGING,
                               SvAmbState.ANCHORING,
                               SvAmbState.ANCHORED):
                        self._sv_state.transition(
                            sv, SvAmbState.FLOATING,
                            epoch=self._n_epochs,
                            reason=f"gf_step:{ev['residual_m']:+.3f}m",
                        )
                except Exception:
                    pass
                drift_actions.append(sv)
            self._wl_drift_prev_fixed = fixed_now

            # NL resolution attempt (after warmup).  Reuse elevations
            # computed earlier this epoch for the slip monitor so the
            # AR elevation mask can gate candidates.
            nl.tick()  # advance blacklist expiry
            if self._n_epochs >= 5:
                # Per-SV phase-bias availability for the short-term
                # promoter's candidate gate.  See Phase 1 call site
                # above for the rationale.
                ar_phase_bias_ok = {
                    o['sv']: o.get('ar_phase_bias_ok', True)
                    for o in observations
                }
                nl.attempt(filt, mw, elevations=elevations,
                           ar_phase_bias_ok=ar_phase_bias_ok)

            # Per-SV state machine: stream PR residuals into the monitors
            # and the host RMS alarm.  Each monitor is stateless per-eval
            # (no cascade) and drives SvStateTracker transitions directly.
            # See docs/sv-lifecycle-and-pfr-split.md.
            labels = getattr(filt, 'last_residual_labels', [])
            self._false_fix.ingest(self._n_epochs, resid, labels)
            self._setting_drop.ingest(self._n_epochs, resid, labels)
            self._fix_set_integrity.ingest(self._n_epochs, resid, labels)
            # NL-layer residual logger — per-epoch per-NL-fixed-SV PR
            # and IF (phi) residuals.  Feeds the empirical case for
            # IF-residual eviction (I-221332-main) — the IF residual
            # is what Charlie's detector should be acting on, not PR.
            # Volume: ~5-10 NL-fixed SVs * 2 residuals * 1 Hz =
            # ~20 lines/sec max; one combined line per SV per epoch.
            _sv_resid: dict[str, dict[str, float]] = {}
            for _i, _lbl in enumerate(labels):
                if _i >= len(resid):
                    break
                _sv, _kind, *_ = _lbl
                _sv_resid.setdefault(_sv, {})[_kind] = float(resid[_i])
            try:
                _nl_fixed_set = {
                    s for s in _sv_resid
                    if self._sv_state.state(s) in (
                        SvAmbState.ANCHORING, SvAmbState.ANCHORED,
                    )
                }
            except Exception:
                _nl_fixed_set = set()
            for _sv in sorted(_nl_fixed_set):
                _rs = _sv_resid[_sv]
                _pr = _rs.get('pr')
                _phi = _rs.get('phi')
                log.info(
                    "[NL_RESID] sv=%s pr_resid=%s if_resid=%s",
                    _sv,
                    f"{_pr:+.3f}m" if _pr is not None else "?",
                    f"{_phi:+.4f}m" if _phi is not None else "?",
                )
            # ZTD state for the integrity monitor's ztd_impossible
            # trigger.  PPPFilter carries a ZTD residual state; if
            # the filter has absorbed position error into ZTD past
            # the physical envelope, monitor returns an event with
            # reason='ztd_impossible' and we revert NL fixes only
            # (keep WL / MW / position).  See
            # docs/ztd-impossibility-trigger-design.md.
            ppp_ztd = None
            # PPPFilter's IDX_ZTD is a module-level constant, not an
            # instance/class attribute, so hasattr on the instance
            # returns False.  Fall through to the imported module
            # constant — matches the pattern used by the [AntPosEst]
            # log line below.
            ppp_ztd_idx = getattr(filt, 'IDX_ZTD', PPP_IDX_ZTD)
            if filt.x.shape[0] > ppp_ztd_idx:
                ppp_ztd = float(filt.x[ppp_ztd_idx])
            # Fleet-consensus deltas (Part 2 of
            # docs/fleet-consensus-monitors.md).  Computed every
            # epoch so the monitor's sustained-epoch counter
            # increments at the right rate.  None when no cohort
            # is available (peer_subscriber inactive, no peers
            # with matching antenna_ref/site_ref, or <2 members);
            # the monitor silently skips those paths.
            _cohort_pos_delta_m = None
            _cohort_ztd_delta_m = None
            try:
                import peer_subscriber
                if peer_subscriber.is_active():
                    _ant_ref = peer_subscriber.get_local_antenna_ref()
                    _site_ref = peer_subscriber.get_local_site_ref()
                    _self_pos_ecef = filt.x[:3]
                    _self_lat, _self_lon, _self_alt = ecef_to_lla(
                        _self_pos_ecef[0], _self_pos_ecef[1],
                        _self_pos_ecef[2])
                    _self_snap = peer_subscriber.build_self_snapshot(
                        antenna_ref=_ant_ref, site_ref=_site_ref,
                        lat_deg=_self_lat, lon_deg=_self_lon,
                        alt_m=_self_alt, ztd_m=ppp_ztd,
                    )
                    if _ant_ref:
                        _pm = peer_subscriber.cohort_median_position(
                            _ant_ref, self_snapshot=_self_snap)
                        if _pm is not None:
                            from peppar_bus.cohort import ecef_distance_m
                            _ml, _mo, _ma, _ = _pm
                            _, _cohort_pos_delta_m = ecef_distance_m(
                                _self_lat, _self_lon, _self_alt,
                                _ml, _mo, _ma)
                    if _site_ref and ppp_ztd is not None:
                        _zm = peer_subscriber.cohort_median_ztd(
                            _site_ref, self_snapshot=_self_snap)
                        if _zm is not None:
                            _med_ztd, _ = _zm
                            _cohort_ztd_delta_m = ppp_ztd - _med_ztd
            except Exception:
                log.exception("cohort delta compute failed (non-fatal)")
            # Bead 4: stream azimuths for ANCHORING SVs so the
            # promoter can accumulate Δaz toward the 15° threshold.  We
            # compute azimuths only when there's at least one SV in
            # ANCHORING — avoids the per-epoch sat_position call
            # on hosts that haven't fixed anything yet.
            prov_count = self._sv_state.count_in(SvAmbState.ANCHORING)
            if prov_count > 0:
                azimuths = _compute_sv_azimuths(
                    filt, corrections, observations, gps_time)
                for sv, az in azimuths.items():
                    self._promoter.ingest_az(sv, az)
            # FalseFixMonitor in observe-only mode: log trip candidates
            # but don't evict.  IfStepMonitor (below) is the canonical
            # NL-layer demoter.
            for ev in self._false_fix.evaluate(self._n_epochs):
                if ev.get('observe_only', False):
                    log.info(
                        "[FALSE_FIX] %s |PR|=%.2fm > %.2fm (n=%d, "
                        "elev=%s, %s) [observe-only — IF step is "
                        "canonical demoter]",
                        ev['sv'], ev['mean_resid_m'], ev['threshold_m'],
                        ev['n'],
                        f"{ev['elev_deg']:.0f}°"
                        if ev['elev_deg'] is not None else "?",
                        ev.get('tag', '?'),
                    )
                else:
                    self._apply_false_fix(filt, mw, nl, ev)

            # IfStepMonitor — NL-layer demoter via cohort-median
            # post-fit phase residual.  Mirrors GfStepMonitor (WL
            # layer) per dayplan I-221332-main.
            nl_fixed_now = set(self._sv_state.svs_in(
                SvAmbState.ANCHORING, SvAmbState.ANCHORED))
            for sv in nl_fixed_now - self._if_step_prev_nl_fixed:
                self._if_step.note_fix(sv)
            for sv in self._if_step_prev_nl_fixed - nl_fixed_now:
                self._if_step.note_unfix(sv)

            # Extract per-SV post-fit phase (IF) residuals from the
            # filter's last residual labels.  Filter to NL-fixed SVs
            # — IfStepMonitor's update() also filters by tracked set,
            # but doing it here keeps the cohort-median input clean.
            phi_resid_by_sv: dict[str, float] = {}
            for lab, r in zip(labels or (), resid if resid is not None else ()):
                if len(lab) >= 2 and lab[1] == 'phi' and lab[0] in nl_fixed_now:
                    phi_resid_by_sv[lab[0]] = float(r)
            # ANCHORED protection per I-140938-main: SVs that have
            # earned ANCHORED state (survived Δaz=15° validation) get
            # a 2× threshold, configurable via IfStepMonitor's
            # anchored_threshold_mult.  Caller (us) supplies the set
            # so the monitor stays decoupled from the SV state tracker.
            anchored_svs = set(self._sv_state.svs_in(SvAmbState.ANCHORED))
            if_step_events = self._if_step.update(
                phi_resid_by_sv, anchored_svs=anchored_svs)
            for ev in if_step_events:
                sv = ev['sv']
                elev = self._sv_state.get(sv).last_elev_deg
                log.warning(
                    "[IF_STEP] %s residual=%+.4fm cohort_residual=%+.4fm "
                    "cohort_median=%+.4fm (n_cohort=%d/%d, %d-epoch "
                    "sustained, thr=±%.3fm%s): NL unfix + ambiguity "
                    "inflate, demoting to WAITING, elev=%s",
                    sv, ev['residual_m'], ev['cohort_residual_m'],
                    ev['cohort_median_m'], ev['cohort_size'],
                    ev.get('cohort_size_total', ev['cohort_size']),
                    ev['consecutive_epochs'], ev['threshold_m'],
                    " ANCHORED-2x" if ev.get('anchored', False) else "",
                    f"{elev:.1f}°" if elev is not None else "?",
                )
                # Standard NL eviction action — same as
                # _apply_false_fix used to do, minus the PR-domain
                # logging path.
                cooldown = 120  # ~2 min default; tune later if needed
                try:
                    self._sv_state.transition(
                        sv, SvAmbState.WAITING,
                        epoch=self._n_epochs,
                        reason=f"if_step:{ev['cohort_residual_m']:+.3f}m",
                        elev_deg=elev,
                        cooldown_epochs=cooldown,
                    )
                except Exception:
                    pass
                nl.unfix(sv)
                # IF step is a real post-fix phase residual jump —
                # wipe the SV's NL trust so a post-slip re-admission
                # is gated as NEW.  I-172719.
                nl.note_slip(sv)
                nl.blacklist(sv, epochs=cooldown)
                filt.inflate_ambiguity(sv)
                mw.reset(sv)
                self._if_step.note_unfix(sv)
                self._promoter.note_false_fix_rejection(sv, self._n_epochs)
            self._if_step_prev_nl_fixed = nl_fixed_now

            for ev in self._setting_drop.evaluate(self._n_epochs):
                self._apply_setting_sv_drop(filt, mw, nl, ev)
            host_ev = self._fix_set_integrity.evaluate(
                self._n_epochs, ztd_m=ppp_ztd,
                pos_consensus_delta_m=_cohort_pos_delta_m,
                ztd_consensus_delta_m=_cohort_ztd_delta_m)
            if host_ev is not None:
                self._apply_integrity_trip(filt, mw, nl, host_ev)
            # Elevation-stratified squelch: sweep WAITING records
            # whose per-SV cooldown has expired and return them to FLOATING.
            # Also drop records for SVs that haven't been observed in
            # STALE_AFTER_EPOCHS — arc boundary, resets the unexpected-
            # false-fix counter on the next rise.
            self._sv_state.check_squelch_cooldowns(self._n_epochs)
            # Mark SVs we saw this epoch; forget those we haven't seen
            # for a while.  600 epochs ≈ 10 min at 1 Hz: long enough to
            # survive brief tracking gaps without clobbering state,
            # short enough to distinguish one arc from the next.
            for obs in observations:
                self._sv_state.mark_seen(obs['sv'], self._n_epochs)
            if self._n_epochs % 60 == 0:  # sweep every ~1 min
                dropped = self._sv_state.forget_stale_with_states(
                    self._n_epochs, 600)
                # Keep the promoter's candidate map in sync — when a
                # record is forgotten (arc boundary), any in-flight
                # candidate for that SV is also stale.  Also emit a
                # synthetic [SV_STATE] → SET transition so downstream
                # log readers (peppar-mon) can drop the SV from their
                # current-state view.  Without this, the SV stays
                # visible in peppar-mon forever in its last-observed
                # state — confusing because it sets out of view but
                # we never log a transition out.
                for sv, prev_state in dropped:
                    self._promoter.forget(sv)
                    self._false_fix.forget(sv)
                    self._setting_drop.forget(sv)
                    # peppar-mon contract: peppar_mon/log_reader.py
                    # treats ``→ SET`` as removal from sv_states.
                    # SET is not a real SvAmbState enum value; it's a
                    # synthetic transition tag the same _SV_STATE_LINE_RE
                    # parses from the standard transition format.
                    log.info(
                        "[SV_STATE] %s: %s → SET (epoch=%d, "
                        "reason=stale_obs:%d epochs)",
                        sv, prev_state.value, self._n_epochs, 600)
            for ev in self._promoter.evaluate(self._n_epochs):
                log.info(
                    "Promoted %s → ANCHORED (Δaz=%.1f°, first=%s, now=%.0f°)",
                    ev['sv'], ev['accumulated_dphi_deg'],
                    f"{ev['first_fix_az_deg']:.0f}°"
                    if ev['first_fix_az_deg'] is not None else "?",
                    ev['latest_az_deg'] or 0.0,
                )
                # I-004810-main: record past-anchored reputation so a
                # later integrity-trip / re-admit cycle gets the
                # PROVISIONAL boost over NEW.  Cleared by note_slip
                # (forget_history) on real cycle slips.
                nl.note_sv_anchored(ev['sv'])

            # Position quality
            sigma_3d = position_sigma_3d(filt.P)
            pos_ecef = filt.x[:3].copy()
            # Two separate counts drive the two thresholds:
            #   n_nl_fixed   — union of ANCHORING + ANCHORED.
            #                  Drives CONVERGING ↔ ANCHORING (fallback:
            #                  any NL integer committed, validated or not).
            #   n_anchored   — ANCHORED only, survived ≥ 8° Δaz.
            #                  Drives ANCHORING ↔ ANCHORED (the strict
            #                  "geometry-validated" milestone).
            n_anchored = self._sv_state.count_in(SvAmbState.ANCHORED)
            n_nl_fixed = sum(1 for sv in filt.sv_to_idx if nl.is_fixed(sv))

            # Update state-machine metrics.  n_nl reports the union
            # count — operators want the "NL fixes currently held"
            # number, not just the subset that's geometry-validated.
            self._ape_sm.update_metrics(
                sigma_m=sigma_3d,
                n_wl=mw.n_fixed,
                n_nl=n_nl_fixed,
                n_sv=len(filt.sv_to_idx),
            )

            # State transitions.  Message includes both counts so
            # operators can see the ramp: "5 anchored (8 fixed)" vs
            # "8 NL fixed" (pre-promotion).
            tag = (f"{n_anchored} anchored ({n_nl_fixed} fixed)"
                   if n_anchored > 0 else f"{n_nl_fixed} NL fixed")
            # Hysteresis: enter ANCHORED at ≥ 4 anchored, exit at < 3
            # (4↑/3↓).  Matches the anchored_by_svs / anchored_by_position regime
            # boundary in ppp_ar.NarrowLaneResolver (strong_anchor_min=3).
            # Enter ANCHORING at ≥ 4 NL fixed (any kind), exit at < 4.
            state = self._ape_sm.state
            if state == AntPosEstState.CONVERGING:
                if n_nl_fixed >= self._resolve_threshold:
                    self._ape_sm.transition(
                        AntPosEstState.ANCHORING,
                        f"{tag}, σ={sigma_3d:.3f}m",
                    )
            elif state == AntPosEstState.ANCHORING:
                if n_anchored >= self._resolve_threshold:
                    self._ape_sm.transition(
                        AntPosEstState.ANCHORED,
                        f"{tag}, σ={sigma_3d:.3f}m",
                    )
                elif n_nl_fixed < self._resolve_threshold - 1:
                    # Hysteresis exit at < 3 (threshold - 1),
                    # matching the ANCHORED → ANCHORING boundary.
                    # Both reverse transitions use the same 4↑/3↓
                    # pattern so a single dropped fix doesn't
                    # chatter the state back across the threshold.
                    self._ape_sm.transition(
                        AntPosEstState.CONVERGING,
                        f"NL fix count dropped to {n_nl_fixed} ({tag})",
                    )
            elif state == AntPosEstState.ANCHORED:
                if n_anchored < self._resolve_threshold - 1:
                    # Hysteresis exit at < 3 (threshold - 1); falls
                    # back to ANCHORING, not all the way to CONVERGING.
                    self._ape_sm.transition(
                        AntPosEstState.ANCHORING,
                        f"anchored count dropped to {n_anchored} ({tag})",
                    )

            # NAV2 position sanity check (every 10 epochs ≈ 10s)
            if (self._nav2_store is not None
                    and self._n_epochs % 10 == 0
                    and self._n_epochs >= 30
                    and self._n_epochs >= self._nav2_cooldown_until):
                self._check_nav2(filt, mw, nl, pos_ecef, sigma_3d, n_nl_fixed)

            # SecondOpinionPosMonitor — 3D nav2Δ above threshold,
            # sustained.  Catches the 'stable wrong' lock that
            # _check_nav2 (horizontal-only) and FixSetIntegrityAlarm
            # (internal-consistency) both miss.  See I-011533-main on
            # dayplan/2026-04-28.  Evaluated every epoch but the
            # NAV2 store is freshness-gated to ~30s.
            if (self._nav2_store is not None
                    and self._n_epochs >= 30
                    and self._n_epochs >= self._nav2_cooldown_until):
                _so_opinion = self._nav2_store.get_opinion(max_age_s=30.0)
                _so_delta = None
                _so_hacc = None
                if _so_opinion is not None:
                    _so_delta = float(np.linalg.norm(
                        pos_ecef - _so_opinion['ecef']))
                    _so_hacc = _so_opinion.get('h_acc_m')
                _so_ev = self._second_opinion.evaluate(
                    self._n_epochs, _so_delta, _so_hacc)
                if _so_ev is not None:
                    _rh = _so_ev.get('rolling_hacc_m')
                    _rh_tag = (
                        f" rolling_hAcc={_rh:.2f}m" if _rh is not None
                        else "")
                    _nav2_ecef = lla_to_ecef(_so_opinion['lat'],
                                             _so_opinion['lon'],
                                             _so_opinion['alt_m'])
                    _reset_ecef, _reset_label = select_reset_target(
                        self._pin_position, self._pin_ecef, _nav2_ecef,
                    )
                    if _reset_label == "known_pos":
                        _lat, _lon, _alt = ecef_to_lla(
                            _reset_ecef[0], _reset_ecef[1], _reset_ecef[2])
                        _reset_tag = (
                            f"known_pos LLA ({_lat:.6f},{_lon:.6f},"
                            f"{_alt:.1f}m) [pin]")
                    else:
                        _reset_tag = (
                            f"NAV2 LLA ({_so_opinion['lat']:.6f},"
                            f"{_so_opinion['lon']:.6f},"
                            f"{_so_opinion['alt_m']:.1f}m)")
                    log.warning(
                        "[SECOND_OPINION_POS] tripped: nav2Δ=%.2fm > "
                        "%.2fm sustained %d ep%s — full re-init at "
                        "%s; unfixing %d NL.",
                        _so_ev['nav2_delta_3d_m'],
                        _so_ev['threshold_m'],
                        _so_ev['sustained_epochs'],
                        _rh_tag,
                        _reset_tag,
                        len(nl._fixed),
                    )
                    for _sv in list(nl._fixed.keys()):
                        nl.unfix(_sv)
                    filt.initialize(_reset_ecef, 0.0,
                                    systems=self._systems)
                    self._prev_t = None
                    self._best_sigma = 999.0
                    self._second_opinion.note_recovery()
                    self._nav2_cooldown_until = self._n_epochs + 120
                    if self._ape_sm.state in (
                            AntPosEstState.ANCHORING,
                            AntPosEstState.ANCHORED):
                        self._ape_sm.transition(
                            AntPosEstState.CONVERGING,
                            reason="second_opinion_pos_reset",
                        )

            # Position callback when improved
            if sigma_3d < self._best_sigma:
                self._best_sigma = sigma_3d
                if self._position_callback is not None:
                    self._position_callback(pos_ecef, sigma_3d)

            # Log every 10 epochs
            if self._n_epochs % 10 == 0:
                rms = np.sqrt(np.mean(resid ** 2)) if len(resid) > 0 else 0
                lat, lon, alt = ecef_to_lla(pos_ecef[0], pos_ecef[1], pos_ecef[2])
                nav2_tag = ""
                nav2_opinion = None
                if self._nav2_store is not None:
                    nav2_opinion = self._nav2_store.get_opinion(max_age_s=30.0)
                    if nav2_opinion is not None:
                        d = float(np.linalg.norm(pos_ecef - nav2_opinion['ecef']))
                        nav2_tag = f" nav2Δ={d:.1f}m"
                ztd_tag = ""
                # PPPFilter stores IDX_ZTD as a module-level constant
                # (not a class attribute), so hasattr on the instance
                # returns False.  Use the imported constant directly.
                ztd_idx = getattr(filt, 'IDX_ZTD', PPP_IDX_ZTD)
                if filt.x.shape[0] > ztd_idx:
                    dztd_mm = filt.x[ztd_idx] * 1000.0
                    dztd_sigma_mm = math.sqrt(max(0.0,
                        filt.P[ztd_idx, ztd_idx])) * 1000.0
                    ztd_tag = f" ZTD={dztd_mm:+.0f}±{dztd_sigma_mm:.0f}mm"
                # Lock-in strength: WL_fixed / σ_3d.  Unweighted first
                # cut, per docs/position-strength-metric.md.  Logged
                # for post-hoc analysis; not used as a decision gate
                # (strength is orthogonal to correctness — high values
                # can coincide with the biased-equilibrium trap).
                wl_fixed_count = mw.n_fixed
                strength = (float(wl_fixed_count) / sigma_3d
                            if sigma_3d > 0 else 0.0)
                strength_tag = f" strength={strength:.0f}"
                readmit_held = self._wl_readmit.n_held()
                readmit_tag = (f" readmit={readmit_held}"
                               if readmit_held > 0 else "")
                # Solid Earth tide magnitude + radial (approx local up)
                # component for this epoch.  Radial is signed; total
                # is magnitude only.  Tag omitted when tide is disabled
                # or the filter hasn't produced a usable position yet.
                tide_tag = ""
                if self._last_tide_3d_mm is not None:
                    tide_tag = (f" tide={self._last_tide_3d_mm:.0f}mm"
                                f"(U{self._last_tide_up_mm:+.0f})")
                # PCV activity: N applied / M skipped this epoch.
                # Tag omitted when PCV is disabled entirely.
                pcv_tag = ""
                if self._pcv_enabled:
                    pcv_tag = (f" pcv={self._last_pcv_applied}"
                               f"/{self._last_pcv_applied + self._last_pcv_skipped}")
                # Worst-σ diagnostic: largest σ (√ of largest
                # eigenvalue) across P's base-state block.  Pairs
                # with positionσ on this line — they share the same
                # "σ in meters" semantics but scope differs.
                # positionσ = position-only uncertainty.
                # worstσ    = worst-case σ across ALL base states
                #             including clock, ISBs, ZTD, capturing
                #             near-rank-deficient coupling.
                # A rising worstσ without matching rise in positionσ
                # is the null-mode-excitation signature (Bravo
                # 2026-04-23 PRIDE arc).  Diagnostic only, no action.
                nm_sigma = self._null_mode_sigma_max(filt)
                worst_tag = (f" worstσ={nm_sigma:.1f}m"
                             if nm_sigma is not None else "")
                # peppar-mon contract: peppar_mon/log_reader.py:
                # _ANTPOSEST_LINE_RE parses this format (positionσ,
                # pos, n=, plus optional nav2Δ/ZTD/tide/worstσ tags).
                # The peppar-mon zombie-SV warning compares ``n=`` to
                # the count in sv_states; keep n= present and accurate.
                # Renaming or reordering fields requires updating the
                # regex + tests.
                log.info(
                    "  [AntPosEst %d] positionσ=%.3fm pos=(%.8f, %.8f, %.3f) "
                    "n=%d amb=%d %s %s%s%s%s%s%s%s%s",
                    self._n_epochs, sigma_3d, lat, lon, alt,
                    n_used, len(filt.sv_to_idx),
                    mw.summary(), nl.summary(), nav2_tag, ztd_tag,
                    strength_tag, readmit_tag, tide_tag, pcv_tag, worst_tag,
                )
                # [OBS_COUNTS] — fleet visibility breakdown for the
                # peppar-mon Untracked + Tracking columns per dayplan
                # I-143806-main.  Combines upstream counters from
                # realtime_ppp (n_raw / n_off_const / n_single) with
                # PPPFilter's per-iteration reject counters
                # (filt.last_reject_counts: no_eph / clock_bad /
                # below_mask).  Falls through gracefully when
                # obs_counts isn't supplied (legacy / replay tests).
                _filter_rej = getattr(filt, 'last_reject_counts', None)
                if obs_counts is not None and _filter_rej is not None:
                    _n_raw = obs_counts.get('n_raw')
                    _n_off = obs_counts.get('n_off_const')
                    _n_single = obs_counts.get('n_single')
                    log.info(
                        "  [OBS_COUNTS] raw=%s used=%d dual=%d "
                        "single=%s off_const=%s below_mask=%d "
                        "clock_bad=%d no_eph=%d",
                        _n_raw if _n_raw is not None else "?",
                        n_used, len(observations),
                        _n_single if _n_single is not None else "?",
                        _n_off if _n_off is not None else "?",
                        _filter_rej.get('below_mask', 0),
                        _filter_rej.get('clock_bad', 0),
                        _filter_rej.get('no_eph', 0),
                    )
                # WL Integer Bootstrap success rate (Teunissen 1998/1999).
                # Per Charlie's AR-readiness literature memo: WL P_IB is
                # the right gate for "is the float ready for WL AR?" —
                # σ_pos is downstream.  Geng et al. 2010 thresholds:
                # > 0.999 = full AR, > 0.99 = partial AR.  Logged every
                # epoch as a diagnostic; current --wl-only mode doesn't
                # gate AR on this yet (planned).
                if self._n_epochs % 10 == 0:  # 1/10 epochs to bound log volume
                    p_wl_ib, n_wl = mw.wl_bootstrap_success_rate()
                    if p_wl_ib is None:
                        log.info(
                            "  [WL_AR_READINESS] p_wl_ib=- n=0 "
                            "(no SVs warmed up)")
                    else:
                        log.info(
                            "  [WL_AR_READINESS] p_wl_ib=%.4f n=%d "
                            "(>0.99=PAR-ready, >0.999=full)",
                            p_wl_ib, n_wl,
                        )
                    # NL AR readiness — Charlie A1.  Pulled from
                    # NlResolver.last_ar_readiness (set every resolve_nl
                    # call, regardless of whether LAMBDA attempted).
                    p_nl_ib = getattr(nl, 'last_ar_readiness', None)
                    n_screened = getattr(nl, 'last_screened_count', 0)
                    if p_nl_ib is None:
                        log.info(
                            "  [AR_READINESS] p_nl_ib=- n=%d "
                            "(too few screened)", n_screened)
                    else:
                        log.info(
                            "  [AR_READINESS] p_nl_ib=%.4f n=%d "
                            "(>0.99=PAR-ready, >0.999=full)",
                            p_nl_ib, n_screened,
                        )
                # WL integrality snapshot every 60 epochs (~1/min).  Shows
                # what WL integers the MW tracker would commit — even in
                # wl-only mode where those commits aren't applied to the
                # filter.  Per-SV: frac (|n_wl - round(n_wl)|), epochs
                # averaged, fixed-flag (* suffix), MW residual std.
                # Diagnostic only — no effect on filter.  See
                # docs/pre-wl-foundation.md for the reframe that
                # motivated this.
                if self._n_epochs % 60 == 0:
                    snap = mw.integrality_snapshot()
                    if snap:
                        # Sort by frac ascending — lowest-frac SVs (most
                        # fix-eligible) first.
                        snap.sort(key=lambda s: s['frac'])
                        parts = []
                        for s in snap:
                            star = "*" if s['fixed'] else " "
                            rstd = s['resid_std_cyc']
                            rstd_str = (f"{rstd:.2f}"
                                        if rstd is not None else "—")
                            parts.append(
                                f"{s['sv']}:{s['frac']:.2f}"
                                f"/{s['n_epochs']}ep/{rstd_str}{star}")
                        n_would = sum(1 for s in snap if s['fixed'])
                        log.info(
                            "  [WL_INTEGRALITY %d] %d SVs, %d would-fix: %s",
                            self._n_epochs, len(snap), n_would,
                            " ".join(parts))

                # Per-SV post-fit residual snapshot every 60 epochs.
                # Bob 2026-04-24 root-cause hunt: when the float PPP
                # solution is biased by observation-model error
                # (phase bias mismatch, PCV offset, code bias residual,
                # hardware signal bias), individual SVs will show
                # systematic residual offsets.  Per-SV mean + std over
                # a rolling window would be ideal, but for a first
                # pass we emit the raw post-fit residual per SV per
                # measurement type (PR + phi) sorted by |resid|
                # descending — worst SVs at the head of each line.
                if self._n_epochs % 60 == 0 and resid is not None \
                        and len(resid) > 0:
                    _labels = getattr(filt, 'last_residual_labels', [])
                    # Pair each residual with (sv, kind).  Sort by
                    # |resid| so operators see the worst offenders first.
                    tagged = [
                        (lab[0], lab[1], float(r))
                        for lab, r in zip(_labels, resid)
                    ]
                    tagged.sort(key=lambda t: -abs(t[2]))
                    pr_parts = [f"{sv}:{v:+.2f}" for sv, k, v in tagged
                                if k == 'pr']
                    phi_parts = [f"{sv}:{v:+.3f}" for sv, k, v in tagged
                                 if k == 'phi']
                    if pr_parts:
                        log.info(
                            "  [RESID_PR %d] %d: %s",
                            self._n_epochs, len(pr_parts),
                            " ".join(pr_parts))
                    if phi_parts:
                        log.info(
                            "  [RESID_PHI %d] %d: %s",
                            self._n_epochs, len(phi_parts),
                            " ".join(phi_parts))
                # Peer-bus publish — mirrors the [AntPosEst] log line's
                # fields to any subscribers.  All three helpers no-op
                # when --peer-bus is disabled.
                peer_publisher.publish_position(
                    ant_pos_est_state=self._ape_sm.state.value,
                    lat_deg=lat, lon_deg=lon, alt_m=alt,
                    position_sigma_m=sigma_3d,
                    worst_sigma_m=nm_sigma,
                    reached_anchored=self._ape_sm.reached_anchored,
                )
                if filt.x.shape[0] > ztd_idx:
                    peer_publisher.publish_ztd(
                        ztd_m=float(filt.x[ztd_idx]),
                        ztd_sigma_mm=int(round(dztd_sigma_mm)),
                    )
                if self._last_tide_3d_mm is not None:
                    peer_publisher.publish_tide(
                        total_mm=int(round(self._last_tide_3d_mm)),
                        u_mm=int(round(self._last_tide_up_mm)),
                    )
                # SV state snapshot piggybacks the AntPosEst cadence.
                # Adopts the same name convention (uppercase enum value)
                # peppar-mon already parses.
                sv_states_snapshot = {
                    sv: rec.state.name
                    for sv, rec in self._sv_state.all_records()
                }
                peer_publisher.publish_sv_state(
                    sv_states=sv_states_snapshot,
                    nl_capable="".join(sorted(
                        getattr(self, '_nl_capable_constellations', set()) or set(),
                    )),
                )
                # Fleet consensus diagnostics (logging only in Part 1;
                # Part 2 wires these into FixSetIntegrityMonitor as
                # trip conditions).  Cohort medians are pulled via the
                # peer_subscriber's snapshot of peer state.  Include
                # self in the median so the cohort always has ≥ 2 when
                # at least one peer is present.  Per cohort semantics
                # in docs/fleet-consensus-monitors.md: position cohort
                # uses antenna_ref; ZTD cohort uses site_ref.
                import peer_subscriber
                if peer_subscriber.is_active():
                    _ant_ref = peer_subscriber.get_local_antenna_ref()
                    _site_ref = peer_subscriber.get_local_site_ref()
                    _ztd_m = (float(filt.x[ztd_idx])
                              if filt.x.shape[0] > ztd_idx else None)
                    _self_snap = peer_subscriber.build_self_snapshot(
                        antenna_ref=_ant_ref, site_ref=_site_ref,
                        lat_deg=lat, lon_deg=lon, alt_m=alt,
                        ztd_m=_ztd_m,
                    )
                    _pos_med = (peer_subscriber.cohort_median_position(
                                    _ant_ref, self_snapshot=_self_snap)
                                if _ant_ref else None)
                    _ztd_med = (peer_subscriber.cohort_median_ztd(
                                    _site_ref, self_snapshot=_self_snap)
                                if _site_ref else None)
                    if _pos_med is not None or _ztd_med is not None:
                        from peppar_bus.cohort import ecef_distance_m
                        parts = []
                        if _pos_med is not None:
                            med_lat, med_lon, med_alt, n_pos = _pos_med
                            dh, d3 = ecef_distance_m(
                                lat, lon, alt,
                                med_lat, med_lon, med_alt,
                            )
                            parts.append(
                                f"pos_cohort_n={n_pos} "
                                f"Δh={dh * 1000:.0f}mm Δ3d={d3 * 1000:.0f}mm")
                        if _ztd_med is not None and _ztd_m is not None:
                            med_ztd, n_ztd = _ztd_med
                            delta_mm = (_ztd_m - med_ztd) * 1000.0
                            parts.append(
                                f"ztd_cohort_n={n_ztd} "
                                f"Δztd={delta_mm:+.1f}mm")
                        if parts:
                            log.info("  [COHORT] %s", "  ".join(parts))
                # Full-precision NAV2 log line.  NAV2-PVT's native format is
                # LLA; lat/lon at 1e-7 deg (~1 cm resolution at our latitude)
                # and height in mm.  Deriving ECEF from LLA doesn't add
                # precision, so we emit LLA directly.  Post-hoc analysis of
                # the cross-host NAV2 ensemble (three F9Ts on the shared
                # antenna) uses these lines; each log entry is self-
                # contained so an aligner can join by timestamp.
                if nav2_opinion is not None:
                    h_acc = nav2_opinion.get('h_acc_m')
                    v_acc = nav2_opinion.get('v_acc_m')
                    pdop = nav2_opinion.get('pdop')
                    log.info(
                        "  [NAV2 %d] lat=%.7f lon=%.7f alt=%.3fm "
                        "hAcc=%s vAcc=%s pDOP=%s fix=%s sv=%d age=%.1fs",
                        self._n_epochs,
                        nav2_opinion['lat'],
                        nav2_opinion['lon'],
                        nav2_opinion['alt_m'],
                        f"{h_acc:.3f}m" if h_acc is not None else "n/a",
                        f"{v_acc:.3f}m" if v_acc is not None else "n/a",
                        f"{pdop:.2f}" if pdop is not None else "n/a",
                        nav2_opinion.get('fix_type', '?'),
                        nav2_opinion.get('num_sv', 0),
                        nav2_opinion.get('age_s', 0.0),
                    )

            # Periodic SV-state summary (replaces the old PFR per-SV
            # residual dump).  Emits a one-line histogram of states at
            # the same cadence; per-SV residual detail lives in the
            # [SV_STATE] transition log and the monitor event logs.
            if self._n_epochs % 60 == 0 and self._n_epochs > 0:
                log.info("  %s", self._sv_state.summary())

        log.info("AntPosEstThread stopped after %d epochs", self._n_epochs)


# ── Phase 2: Steady state ────────────────────────────────────────────── #


def run_steady_state(args, known_ecef, obs_queue, corrections, beph, ssr,
                     stop_event, qerr_store=None, out_w=None, nav2_store=None,
                     ape_sm=None, dfe_sm=None, ape_thread=None,
                     ar_position=None, ar_pos_lock=None,
                     extint_store=None,
                     pos_sigma_m=None, pos_source=None):
    """Run FixedPosFilter for clock estimation with optional servo.

    This is the steady-state phase: position is known, we estimate clock
    offset and optionally discipline a PHC.
    """
    log.info("=== Phase 2: Steady state (FixedPosFilter) ===")
    lat, lon, alt = ecef_to_lla(known_ecef[0], known_ecef[1], known_ecef[2])
    log.info(f"Position: {lat:.6f}, {lon:.6f}, {alt:.1f}m")

    # Compute METAR-seeded ZTD residual when --init-ztd-from-met is set.
    # Default 0.0 preserves prior behavior.  See I-024942 and 2026-05-04
    # overnight: under --pin-position the cold-start ZTD spent hours
    # walking back from multi-meter values because the wide ztd_sigma
    # prior centered at 0 absorbed clock-state error that had nowhere
    # else to go.  METAR-derived Saastamoinen seed lands the filter
    # within physical envelope at epoch 1.
    init_ztd_m = 0.0
    init_ztd_sigma_m = 0.5  # FixedPosFilter prior default
    # Manual override (--init-ztd-mm) takes precedence over METAR.  Useful
    # for the architectural-validation cross-check where the operator
    # wants to disentangle "is the filter behaving correctly under tight
    # ZTD prior?" from "is Saastamoinen-from-METAR producing the right
    # seed value?"  When neither flag is set, legacy default behavior
    # (0 m residual, 0.5 m σ wide prior) is preserved.
    if getattr(args, 'init_ztd_mm', None) is not None:
        init_ztd_m = float(args.init_ztd_mm) * 1e-3
        if getattr(args, 'init_ztd_sigma_mm', None) is not None:
            init_ztd_sigma_m = float(args.init_ztd_sigma_mm) * 1e-3
        log.info(
            "FixedPosFilter ZTD seed (manual): init=%+.0fmm σ=%.0fmm",
            args.init_ztd_mm,
            args.init_ztd_sigma_mm
            if args.init_ztd_sigma_mm is not None else 500.0,
        )
    elif getattr(args, 'init_ztd_from_met', False):
        init_ztd_m, init_ztd_sigma_m = _seed_ztd_from_metar(
            args, lat, alt, log)

    # Let FixedPosFilter's first-epoch PR seed run — bootstrap aligns
    # the host PHC to GPS, but the F9T's *internal* receiver clock
    # (the one stamping RXM-RAWX, hence dt_rx) can still be tens of ms
    # off from GPS.  Seeding x[CLK]=0 with σ=100m forced the Kalman
    # update to absorb millions of metres of clock-error during cold
    # start and, with a tight ZTD prior, bled into the ZTD state
    # (observed 2026-05-04: ZTD walked +32mm → -4494mm in 30 epochs).
    # The first-epoch median-PR seed inside FixedPosFilter.update()
    # places x[CLK] near truth before the Kalman gain attacks ZTD.
    filt = FixedPosFilter(known_ecef,
                          init_ztd_m=init_ztd_m,
                          init_ztd_sigma_m=init_ztd_sigma_m)
    filt.prev_clock = 0.0

    # σ_pin: time filter's uncertainty about its pinned ARP, used by
    # the σ-weighted Bayesian position-blend (I-125649 Stage 2/3).
    # Initial value comes from how the engine learned this position
    # (threaded from main()'s seed-source resolution):
    #   --pin-position with surveyed coords (sub-cm OPUS/PRIDE):
    #     σ_pin = 1 mm; blend is no-op, preserving pin
    #   --known-pos without --pin-position (warm-start refinement):
    #     σ_pin = 0.5 m (typical AntPosEst float convergence target)
    #   NAV2 seed (Stage 3):
    #     σ_pin = nav2_h_acc (typically 1-3 m on clean sky, up to 5 m
    #     near the wait_for_nav2_seed gate)
    #   Phase-1 PPP bootstrap fallback:
    #     σ_pin = bootstrap σ (sub-meter once converged)
    # The blend converges σ_pin toward σ_pos as updates accumulate;
    # see warm-start trajectory in dayplan I-125649-main.
    if getattr(args, 'pin_position', False):
        sigma_pin_m = 0.001  # 1 mm — surveyed pin, blend no-op
    elif pos_sigma_m is not None and pos_sigma_m > 0:
        # Use the actual seed σ from main() — NAV2 hAcc, Phase-1
        # bootstrap σ, or operator-supplied --known-pos with non-zero σ.
        sigma_pin_m = float(pos_sigma_m)
    elif getattr(args, 'known_pos', None) is not None:
        # --known-pos supplied without explicit σ; treat as warm-start
        # (operator's coords are not necessarily surveyed truth — that
        # would have used --pin-position).
        sigma_pin_m = 0.5
    else:
        sigma_pin_m = 5.0    # NAV2-class fallback
    log.info("Position-blend σ_pin initial: %.3f m "
             "(source=%s, --ar-mode=%s)",
             sigma_pin_m, pos_source or "default", args.ar_mode)

    # Stage 6 self-healing gate (I-125649): when the blend keeps
    # rejecting updates with the same sustained outlier delta, the
    # source has likely moved (e.g., AntPosEst was reset by the
    # SECOND_OPINION_POS watchdog) and σ_pin is now too tight to
    # let the gate re-engage with the new source position.  Track
    # consecutive rejections and, after a threshold, widen σ_pin
    # so the next blend can be applied.  Counter resets on any
    # accepted blend.
    consecutive_blend_rejects = 0
    SUSTAINED_REJECT_THRESHOLD = 5  # ≈ 50 s for NAV2 source (10-epoch)
    SIGMA_PIN_WIDEN_M = 5.0          # NAV2-class fallback ceiling
    watchdog = PositionWatchdog(threshold_m=args.watchdog_threshold)
    # Shadow gate: TD-CP-only watchdog runs in parallel for I-145915
    # bake-in.  Verdict logged per epoch but does NOT drive engine
    # behavior until charlie's owner change flips it active.  See
    # docs/time-filter-pr-cp-port.md + scripts/peppar_fix/watchdog.py.
    watchdog_proposed = PositionWatchdogProposed(
        threshold_m=0.05,                       # 50 mm TD-CP gate
        pr_threshold_m=args.watchdog_threshold, # match active gate
    )
    # Counters for the [FIXEDPOS_RESID_SUMMARY] line at engine exit.
    resid_log_stats = {
        'epochs': 0,
        'active_trips': 0,        # PositionWatchdog alarm transitions
        'proposed_trips': 0,      # PositionWatchdogProposed alarm transitions
        'pr_disturbances': 0,     # PR_DISTURBANCE flag epochs
        'both_alarmed': 0,        # both gates simultaneously alarmed
        'only_active': 0,         # active alarmed, proposed not
        'only_proposed': 0,       # proposed alarmed, active not
    }
    _prev_active_alarmed = False
    _prev_proposed_alarmed = False
    correction_gate = CorrectionFreshnessGate()

    # Optional servo setup (PTP imports only loaded when needed)
    # Enter the servo path when:
    #   1. --servo is set (PHC-based servo, original path), OR
    #   2. --ticc-drive is set without --servo (TICC-only servo, no PHC —
    #      e.g., clkPoC3 where TICC measures and DAC steers the OCXO,
    #      with ts2phc disciplining the PHC independently).
    servo_ctx = None
    ptp = None
    want_servo = args.servo or getattr(args, 'ticc_port', None) is not None
    if want_servo:
        if args.servo:
            # Open PTP device for bootstrap and servo
            try:
                from peppar_fix import PtpDevice
            except ImportError:
                log.error("peppar_fix library not available for servo")
                return 1
            try:
                ptp = PtpDevice(args.servo)
            except OSError as e:
                log.error(f"Cannot open PTP device {args.servo}: {e}")
                return 1
        else:
            log.info("TICC-only servo mode: no PHC in loop (ticc_port=%s, "
                     "servo=%s)", args.ticc_port, args.servo)

        # DO bootstrap: ensure phase ±10µs and frequency ±5ppb before
        # servo starts.  Skipped if --skip-bootstrap or --freerun.
        # For PHC DOs: phase step + adjfine.
        # For VCOCXO DOs: TADD ARM + DAC frequency seed.
        #
        # dacWarmStart-main: when --warm-start-if-fresh is set, gate the
        # bootstrap on the freshness of state/dos/<uid>.json.  Recent
        # last_known_freq_offset_ppb seeds _bootstrap_freq_ppb directly,
        # which _setup_servo's "Restored bootstrap frequency" path
        # re-applies after actuator.setup().  Saves 15-30s on watchdog
        # re-bootstrap where the saved freq is seconds old.
        skip_for_warm_start = False
        if (getattr(args, 'warm_start_if_fresh', False)
                and not args.freerun
                and not getattr(args, 'skip_bootstrap', False)):
            try:
                from peppar_fix.do_state import is_do_warm_startable
                _do_uid_ws = (getattr(args, 'do_unique_id', None)
                              or _resolve_do_uid(args))
                if _do_uid_ws is not None:
                    ok, info = is_do_warm_startable(
                        _do_uid_ws,
                        max_age_s=args.warm_start_max_age_s,
                        max_ppb=args.warm_start_max_ppb)
                    if ok:
                        args._bootstrap_freq_ppb = info["freq_ppb"]
                        skip_for_warm_start = True
                        log.info("Warm-start: skipping DO bootstrap "
                                 "(freq=%.1f ppb, age=%.0fs from state)",
                                 info["freq_ppb"], info["age_s"])
                        # Phase-conditional TADD ARM.  Warm-start skips
                        # _do_bootstrap_init() which is where ARM runs;
                        # check phase and step if beyond the threshold.
                        # Divider phase state isn't persisted in any
                        # state file, so any physical event between the
                        # last save and now can leave the divider at a
                        # wrong sub-second phase.  See
                        # project_warm_start_sign_blind_spot memory.
                        _maybe_step_divider_on_phase(args)
                        if dfe_sm is not None:
                            dfe_sm.transition(DOFreqEstState.TRACKING,
                                              "warm-start from saved freq")
                    else:
                        log.info("Warm-start gate failed (%s) — "
                                 "falling back to cold-start bootstrap",
                                 info.get("reason", "unknown"))
                else:
                    log.info("Warm-start: no do_unique_id resolvable — "
                             "falling back to cold-start bootstrap")
            except Exception as e:
                log.warning("Warm-start gate raised %s — "
                            "falling back to cold-start bootstrap", e)

        if (not skip_for_warm_start
                and not getattr(args, 'skip_bootstrap', False)
                and not args.freerun):
            if not _do_bootstrap_init(args, ptp, known_ecef, obs_queue,
                                        beph, ssr, stop_event,
                                        dfe_sm=dfe_sm):
                log.warning("DO bootstrap failed — proceeding to servo setup "
                            "(TICC reader, noise estimator will still start)")
            else:
                log.info("DO bootstrap succeeded (%s)", getattr(args, 'do_type', 'phc'))
                if dfe_sm is not None:
                    dfe_sm.transition(DOFreqEstState.TRACKING, "DO bootstrap succeeded")

        # Set up servo with PPS retry (absorbs wrapper's exit-code-3 retry).
        # No-PPS (return 3) is retryable — PPS may appear after a few seconds
        # if the receiver just started or the cable was reconnected.
        pps_max_retries = 3
        pps_backoff = 5
        servo_result = None
        for pps_attempt in range(1, pps_max_retries + 1):
            servo_result = _setup_servo(args, known_ecef, qerr_store,
                                        extint_store=extint_store, ptp=ptp)
            if not isinstance(servo_result, int) or servo_result != 3:
                break
            if pps_attempt < pps_max_retries:
                log.warning("No PPS — retry %d/%d in %ds",
                            pps_attempt, pps_max_retries, pps_backoff)
                time.sleep(pps_backoff)
                pps_backoff = min(pps_backoff * 2, 60)
            else:
                log.error("No PPS after %d attempts — giving up", pps_max_retries)
        # Promote clockClass to 52 (initialized) after successful bootstrap
        # and servo setup — mirrors what the wrapper's
        # promote_clock_class_initialized did.
        if not isinstance(servo_result, int) and servo_result.get('pmc'):
            _set_clock_class(servo_result, "initialized")
        if isinstance(servo_result, int):
            log.error("Failed to set up servo (exit code %d)", servo_result)
            return servo_result
        servo_ctx = servo_result
        servo_ctx["correlation_gate"] = StrictCorrelationGate()
        if dfe_sm is not None:
            servo_ctx["dfe_sm"] = dfe_sm

    prev_t = None
    n_epochs = 0
    n_epochs_total = 0  # counts all observation epochs, even when gate stalls
    start_time = time.time()
    skip_stats = {
        "gate_wait_obs": 0,
        "corr_wait": 0,
        "dt_suspicious": 0,
        "too_few_meas": 0,
        "servo_no_pps": 0,
        "servo_outlier": 0,
        "obs_idle_holdover": 0,
        "obs_input_timeouts": 0,
        "obs_deferred_stalls": 0,
        "obs_dropped_expired": 0,
        "obs_dropped_queued": 0,
        "ticc_missing_pair": 0,
        "catastrophic": 0,    # FixedPosFilter catastrophic-reject (Note B)
        "consumption_alarm": False,
    }
    # PPP-AR belongs in AntPosEst (the background PPPFilter that refines
    # position), not in DOFreqEst's FixedPosFilter loop.  DOFreqEst uses
    # time-differenced carrier phase — ambiguities cancel, no AR needed.
    # See docs/architecture-vision.md for the clear separation.
    # TODO: keep PPPFilter alive as AntPosEst background thread, feed it
    # decimated observations, run MW+NL there.

    last_skip_log = start_time
    last_obs_wall = time.monotonic()
    last_obs_input_wall = last_obs_wall
    last_usable_obs_wall = last_obs_wall
    obs_idle_alarm = False
    deferred_alarm = False
    # Queue monitoring: high-water marks (session max) + depth threshold alerts
    queue_hwm = {"obs_queue": 0, "obs_history": 0, "pps_history": 0}
    queue_alert_armed = {"obs_queue": True, "obs_history": True, "pps_history": True}
    queue_depth_threshold = getattr(args, "queue_depth_threshold", 5)
    queue_dump = getattr(args, "queue_depth_dump", False)
    last_hwm_log = start_time
    # Consumption rate monitor: detect when observation delivery persistently
    # falls behind PPS.  Growing recv_dt means the I2C/serial buffer is filling
    # faster than we drain it — we'll eventually lose observations and the
    # correlation window will fail.  This is a sanity check, not a retry:
    # the correct fix is to reduce message bandwidth (disable SFRBX, etc).
    recv_dt_history = deque(maxlen=30)
    consumption_alarm = False
    # Sink policy: steady-state + servo is a correlated-window consumer.
    # Preserve receive order here and let the correlator decide when an epoch
    # is too old to be useful, rather than draining the queue at phase entry.
    obs_history = deque()
    try:
        while not stop_event.is_set():
            if args.duration and (time.time() - start_time) > args.duration:
                log.info(f"Duration limit reached ({args.duration}s)")
                break

            try:
                added_obs = _append_queue_history(obs_history, obs_queue, timeout=5)
            except queue.Empty:
                skip_stats["obs_input_timeouts"] += 1
                idle_s = time.monotonic() - last_obs_wall
                if (
                    servo_ctx is not None and
                    args.obs_idle_timeout_s is not None and
                    idle_s >= args.obs_idle_timeout_s and
                    not obs_idle_alarm
                ):
                    skip_stats["obs_idle_holdover"] += 1
                    _enter_obs_holdover(
                        servo_ctx, args, "no_obs_input", f"no observation epochs for {idle_s:.1f}s"
                    )
                    obs_idle_alarm = True
                continue
            if added_obs:
                last_obs_input_wall = time.monotonic()

            # Forward observations to AntPosEst background thread.
            # Feed from obs_history head *before* the PPS correlation gate,
            # so AntPosEst runs even when the servo's gate is stalled (no
            # PPS events, EXTTS wedge, etc.).  AntPosEst only needs raw
            # GNSS observations for the PPPFilter — no PPS correlation.
            if ape_thread is not None and added_obs:
                # Peek at the newest observation in the history (rightmost).
                # Don't pop — the servo still needs it for correlation.
                newest = obs_history[-1]
                gps_time_ape, observations_ape = newest
                # Per dayplan I-143806-main: thread the obs counters
                # from the upstream ObservationEvent through to the
                # ape_thread so it can emit [OBS_COUNTS] each epoch.
                # ``newest`` may be an ObservationEvent (full counters
                # available) or a bare tuple (legacy / tests; counters
                # default to None and the engine's emit falls through
                # gracefully).
                obs_counts_ape = None
                if hasattr(newest, 'n_raw'):
                    obs_counts_ape = {
                        'n_raw': newest.n_raw,
                        'n_off_const': newest.n_off_const,
                        'n_single': newest.n_single,
                    }
                if n_epochs_total % ape_thread.decimation == 0:
                    ape_thread.feed(
                        gps_time_ape, observations_ape,
                        obs_counts=obs_counts_ape,
                    )
                n_epochs_total += 1

            # ── Queue depth monitoring ──
            _check_queue_depths(
                queue_hwm, queue_alert_armed, queue_depth_threshold, queue_dump,
                obs_queue=obs_queue,
                obs_history=obs_history,
                pps_history=servo_ctx.get("pps_history") if servo_ctx else None,
                pps_history_lock=servo_ctx.get("pps_history_lock") if servo_ctx else None,
                skip_stats=skip_stats,
                gate=servo_ctx.get("correlation_gate") if servo_ctx else None,
            )

            if servo_ctx is not None:
                gate = servo_ctx["correlation_gate"]
                dropped_before = gate.stats.dropped_unmatched
                obs_event, pps_match = gate.pop_observation_match(
                    obs_history,
                    target_sec_fn=lambda event: _target_timescale_sec(event.gps_time, args),
                    match_fn=lambda obs_event, target_sec, min_window_s=0.5, max_window_s=11.0:
                        _match_pps_event_from_history(
                            servo_ctx,
                            obs_event,
                            target_sec,
                            min_window_s=min_window_s,
                            max_window_s=max_window_s,
                        ),
                    min_confidence=args.min_correlation_confidence,
                )
                dropped_obs = gate.stats.dropped_unmatched - dropped_before
                queued_dropped = gate.stats.dropped_queued_behind
                if queued_dropped > skip_stats["obs_dropped_queued"]:
                    n_new = queued_dropped - skip_stats["obs_dropped_queued"]
                    log.info("  Skipped %d queued observations (unreliable recv_mono)", n_new)
                    skip_stats["obs_dropped_queued"] = queued_dropped
                if obs_event is None:
                    skip_stats["gate_wait_obs"] += 1
                    stall_s = time.monotonic() - last_usable_obs_wall
                    if (
                        args.obs_idle_timeout_s is not None and
                        stall_s >= args.obs_idle_timeout_s and
                        not deferred_alarm
                    ):
                        skip_stats["obs_deferred_stalls"] += 1
                        log.warning(
                            "Observation pipeline stalled without holdover: reason=obs_received_but_deferred "
                            "stalled_for=%.1fs queued=%d input_quiet_for=%.1fs",
                            stall_s,
                            len(obs_history),
                            time.monotonic() - last_obs_input_wall,
                        )
                        deferred_alarm = True
                    if added_obs and n_epochs % 10 == 0:
                        log.info(f"  [{n_epochs}] Awaiting correlatable observation "
                                 f"(queued={len(obs_history)})")
                    continue
            else:
                obs_event = obs_history.popleft()
                pps_match = None
                dropped_obs = 0

            last_obs_wall = time.monotonic()
            obs_idle_alarm = False
            deferred_alarm = False
            last_usable_obs_wall = last_obs_wall
            if servo_ctx is not None:
                _exit_holdover(servo_ctx, "fresh usable observation epoch received")

            # ── Consumption rate sanity check ──
            # Track recv_dt_s to detect persistent observation delivery lag.
            if pps_match is not None:
                _, _, match_recv_dt_s, _ = pps_match
                recv_dt_history.append(match_recv_dt_s)
                if len(recv_dt_history) >= 20 and not consumption_alarm:
                    # Linear trend of recv_dt over last 20 correlated epochs.
                    # Positive slope means observations arrive later each epoch.
                    dt_first = recv_dt_history[0]
                    dt_last = recv_dt_history[-1]
                    growth = dt_last - dt_first
                    if growth > 3.0:
                        consumption_alarm = True
                        skip_stats["consumption_alarm"] = True
                        log.error(
                            "CONSUMPTION RATE ALARM: observation delivery lag grew "
                            "%.1fs over %d epochs (recv_dt: %.1f → %.1fs). "
                            "The GNSS transport cannot sustain the configured "
                            "message rate. Observations will be lost. "
                            "Reduce I2C message bandwidth (disable SFRBX/PVT) "
                            "or check for transport bottlenecks.",
                            growth, len(recv_dt_history),
                            dt_first, dt_last,
                        )
                        _set_clock_class(servo_ctx, "freerun")

            ok_corr, corr_reason, corr_snapshot = correction_gate.accept(
                corrections,
                max_broadcast_age_s=args.max_broadcast_age_s,
                require_ssr=args.require_ssr,
                max_ssr_age_s=args.max_ssr_age_s,
                min_broadcast_confidence=args.min_broadcast_confidence,
                min_ssr_confidence=args.min_ssr_confidence,
            )
            if not ok_corr:
                skip_stats["corr_wait"] += 1
                if n_epochs % 10 == 0:
                    log.info(
                        "  [%s] Waiting for fresh corrections: reason=%s "
                        "broadcast_age=%s",
                        n_epochs,
                        corr_reason,
                        f"{corr_snapshot['broadcast_age_s']:.1f}s"
                        if corr_snapshot["broadcast_age_s"] is not None else "N/A",
                    )
                continue

            if dropped_obs and n_epochs % 10 == 0:
                log.info(f"  [{n_epochs}] Dropped {dropped_obs} expired observation epochs")
            skip_stats["obs_dropped_expired"] += dropped_obs
            gps_time, observations = obs_event

            # After a PHC step, the filter's clock state is stale.
            # Reset dt_rx to near-zero so the servo doesn't over-correct.
            if servo_ctx and servo_ctx.pop('filter_needs_clock_reset', False):
                filt.x[filt.IDX_CLK] = 0.0
                filt.P[filt.IDX_CLK, filt.IDX_CLK] = 2500.0 ** 2
                prev_t = None
                log.info("  EKF clock state reset after PHC step")

            # EKF predict
            if prev_t is not None:
                dt = (gps_time - prev_t).total_seconds()
                if dt <= 0:
                    skip_stats["dt_suspicious"] += 1
                    log.warning(f"Suspicious dt={dt:.1f}s, skipping")
                    prev_t = gps_time
                    continue
                if dt > 30:
                    # Gap recovery: reset filter time but don't skip the epoch.
                    # Clamp predict to 1s so the filter doesn't diverge, then
                    # let the update re-anchor from pseudoranges.
                    skip_stats["dt_suspicious"] += 1
                    log.warning(f"Gap dt={dt:.1f}s, resetting filter time (not skipping)")
                    filt.predict(1.0)
                else:
                    filt.predict(dt)
            prev_t = gps_time

            # EKF update
            n_used, resid, n_td = filt.update(
                observations, corrections, gps_time,
                clk_file=corrections,
            )

            # Catastrophic-reject cascade (I-202649 v2 + Note A engine
            # integration).  The filter rejects an epoch via the
            # residual-consistency gate by returning n_used=0 *and*
            # incrementing _consecutive_catastrophic_rejects.  An n_used=0
            # return without that counter advancing is the regular
            # n_pr<1 path (geometry / SP3 / clk_file gate); only the
            # gated-reject case bumps the counter.
            #
            # On individual rejects, log the count to skip_stats so
            # operators see catastrophic activity at engine exit (Note B).
            # On sustained rejects (>= filter's CATASTROPHIC_REJECT_LIMIT
            # consecutive), exit for wrapper-driven re-bootstrap — same
            # shape as the existing 30-outlier servo cascade.  Sustained
            # catastrophic rejects mean F9T input has lost integrity for
            # long enough that the DO has been free-running unmonitored;
            # better to clockClass-degrade and let the wrapper start
            # fresh than to keep skipping epochs forever.
            n_consec_cat = getattr(
                filt, '_consecutive_catastrophic_rejects', 0)
            if n_used == 0 and n_consec_cat > 0:
                skip_stats["catastrophic"] += 1
                cat_limit = getattr(
                    filt, 'CATASTROPHIC_REJECT_LIMIT', 30)
                if n_consec_cat >= cat_limit:
                    log.error(
                        "  %d consecutive catastrophic rejects "
                        "(>= limit %d) — F9T input stream lost "
                        "integrity.  Exiting for re-bootstrap "
                        "(exit code 5).",
                        n_consec_cat, cat_limit,
                    )
                    if dfe_sm is not None:
                        dfe_sm.transition(
                            DOFreqEstState.HOLDOVER,
                            f"{n_consec_cat} consecutive catastrophic rejects",
                        )
                    if servo_ctx is not None:
                        _set_clock_class(servo_ctx, "freerun")
                        servo_ctx['phc_diverged'] = True
                    return 5
                # Below the limit: skip this epoch and continue.
                continue

            if n_used < 4:
                skip_stats["too_few_meas"] += 1
                continue

            # Blend a refined position estimate into FixedPosFilter's ARP.
            #
            # --ar-mode wl (production default):
            #   σ-weighted Bayesian blend with selectable source via
            #   --position-blend-source {nav2, antposest, none}.  Math
            #   lives in scripts/peppar_fix/arp_blend.py and is unit-
            #   tested independently (test_arp_blend.py).
            #
            #   nav2 (default):  source = NAV2 running-mean.  Cold-start
            #     finding 2026-05-09 — WL-only AntPosEst converges to a
            #     6-8 m basin off NAV2 on our lab, with ZTD residual
            #     +1146 mm (PR-domain bias dumped into ZTD).  NAV2 is
            #     more accurate than WL-only AntPosEst until the
            #     systematic biases (I-155354, I-165118) are closed.
            #     Sampled every 10 epochs (~10 s) to throttle
            #     correlated-update overcount.
            #
            #   antposest:  source = AntPosEst published position.
            #     Available for the future restoration once systematic
            #     biases close — at that point switch the default.
            #     Today useful for research / regression A/B.
            #
            #   none:  no blend.  σ_pin frozen at init.  Useful when
            #     --known-pos is operator-supplied warm-start coords
            #     and the operator wants no live refinement.
            #
            # --ar-mode full (legacy, research-only):
            #   Original AntPosEst α=0.001 trickle preserved unchanged.
            #
            # Skipped under --pin-position regardless of mode: surveyed
            # known_pos is sub-cm truth and any blend source is worse.
            if not getattr(args, 'pin_position', False):
                if args.ar_mode == "wl":
                    blend_source = getattr(
                        args, 'position_blend_source', 'nav2')
                    src_ecef = None
                    src_sigma = None
                    src_label = None

                    if (blend_source == "nav2"
                            and nav2_store is not None
                            and n_epochs % 10 == 0):
                        nav2_op = nav2_store.get_opinion(max_age_s=10.0)
                        if (nav2_op is not None
                                and nav2_op.get('fix_type') == 3
                                and nav2_op.get('h_acc_m') is not None
                                and nav2_op['h_acc_m'] < 5.0):
                            src_ecef = np.asarray(nav2_op['ecef'],
                                                  dtype=float)
                            src_sigma = float(nav2_op['h_acc_m'])
                            src_label = "NAV2"
                    elif (blend_source == "antposest"
                            and ar_position is not None
                            and ar_pos_lock is not None):
                        with ar_pos_lock:
                            ae_ecef = ar_position.get('ecef')
                            ae_sigma = ar_position.get('sigma')
                        if (ae_ecef is not None and ae_sigma is not None
                                and ae_sigma < 1.0):
                            src_ecef = np.asarray(ae_ecef, dtype=float)
                            src_sigma = float(ae_sigma)
                            src_label = "AntPosEst"
                    # blend_source == "none": no source, no blend

                    if src_ecef is not None and src_sigma is not None:
                        blend = bayesian_arp_blend(
                            known_ecef, sigma_pin_m,
                            src_ecef, src_sigma,
                        )
                        if blend.action == "applied":
                            known_ecef = blend.arp_new
                            filt.pos = np.array(known_ecef)
                            sigma_pin_m = blend.sigma_pin_new_m
                            consecutive_blend_rejects = 0
                            if (n_epochs % 100 == 0
                                    and blend.delta_3d_m > 0.01):
                                log.info(
                                    "%s blend: Δ=%.2fm α=%.3f "
                                    "step=%.1fmm σ_pin=%.3fm "
                                    "σ_src=%.3fm",
                                    src_label, blend.delta_3d_m,
                                    blend.alpha_eff,
                                    blend.step_3d_m * 1000,
                                    sigma_pin_m, src_sigma,
                                )
                        else:
                            consecutive_blend_rejects += 1
                            if (consecutive_blend_rejects
                                    >= SUSTAINED_REJECT_THRESHOLD):
                                # Source has settled at a position
                                # outside the gate.  Widen σ_pin so
                                # the next blend can re-engage.
                                # This catches AntPosEst-source
                                # SECOND_OPINION_POS resets and
                                # similar discontinuities.  Counter
                                # resets after the widen so we don't
                                # log every epoch.
                                old_sigma = sigma_pin_m
                                sigma_pin_m = max(sigma_pin_m,
                                                  SIGMA_PIN_WIDEN_M)
                                log.warning(
                                    "%s blend: %d consecutive rejects "
                                    "— widening σ_pin %.3f → %.3f m "
                                    "(self-healing gate, I-125649 "
                                    "Stage 6)",
                                    src_label,
                                    consecutive_blend_rejects,
                                    old_sigma, sigma_pin_m,
                                )
                                consecutive_blend_rejects = 0
                            elif n_epochs % 100 == 0:
                                log.warning(
                                    "%s blend outlier rejected: "
                                    "Δ=%.2fm > %.1fσ=%.2fm "
                                    "(σ_pin=%.3f σ_src=%.3f) "
                                    "[%d consecutive]",
                                    src_label, blend.delta_3d_m,
                                    3.0, blend.gate_3sigma_m,
                                    sigma_pin_m, src_sigma,
                                    consecutive_blend_rejects,
                                )
                elif (ar_position is not None and ar_pos_lock is not None):
                    # --ar-mode full: legacy AntPosEst α=0.001 trickle
                    with ar_pos_lock:
                        ar_ecef = ar_position.get('ecef')
                        ar_sigma = ar_position.get('sigma')
                    if (ar_ecef is not None and ar_sigma is not None
                            and ar_sigma < 1.0):
                        delta = ar_ecef - known_ecef
                        delta_3d = float(np.linalg.norm(delta))
                        alpha = 0.001  # τ ≈ 1000 epochs (1000 s at 1 Hz)
                        step = alpha * delta
                        known_ecef += step
                        filt.pos = np.array(known_ecef)
                        step_mm = float(np.linalg.norm(step)) * 1000
                        if n_epochs % 100 == 0 and delta_3d > 0.01:
                            log.info("Position blend: Δ=%.2fm step=%.1fmm "
                                     "(AR σ=%.3fm)",
                                     delta_3d, step_mm, ar_sigma)

            # Watchdog with NAV2 position consensus
            resid_rms = float(np.sqrt(np.mean(resid ** 2))) if len(resid) > 0 else 0.0

            # PR / TD-CP residual split — populated by FixedPosFilter.update()
            # via instance attributes (I-145915).  Falls back to safe defaults
            # when the residual vector is empty.
            n_pr_used = int(getattr(filt, 'last_n_pr', 0))
            n_td_used = int(getattr(filt, 'last_n_td', 0))
            r_pr = getattr(filt, 'last_resid_pr', np.array([]))
            r_td = getattr(filt, 'last_resid_td', np.array([]))
            resid_pr_rms = (float(np.sqrt(np.mean(r_pr ** 2)))
                            if len(r_pr) > 0 else 0.0)
            resid_td_rms = (float(np.sqrt(np.mean(r_td ** 2)))
                            if len(r_td) > 0 else 0.0)

            # Shadow watchdog evaluation (no behavior change today).
            td_ok, pr_disturbance = watchdog_proposed.update(
                resid_pr_rms, resid_td_rms, n_used)
            resid_log_stats['epochs'] += 1
            if pr_disturbance:
                resid_log_stats['pr_disturbances'] += 1
            # Trip-transition counters for the summary at exit.
            if watchdog_proposed.alarmed and not _prev_proposed_alarmed:
                resid_log_stats['proposed_trips'] += 1
            _prev_proposed_alarmed = watchdog_proposed.alarmed

            # Per-epoch instrumentation line.  Sparse cadence to keep
            # the log readable; the bake-in goal is operator-visible
            # divergence between the two gates, not high-frequency
            # diagnostics.  Heartbeat every 60 epochs; always log
            # transition events (alarm / clear / pr_disturbance).
            _heartbeat = (n_epochs % 60 == 0)
            if _heartbeat or pr_disturbance:
                log.info(
                    "[FIXEDPOS_RESID] n_pr=%d resid_pr_rms=%.3fm "
                    "n_td=%d resid_td_rms=%.4fm "
                    "active_alarmed=%s proposed_alarmed=%s%s",
                    n_pr_used, resid_pr_rms, n_td_used, resid_td_rms,
                    watchdog.alarmed, watchdog_proposed.alarmed,
                    " PR_DISTURBANCE" if pr_disturbance else "",
                )

            watchdog.update(resid_rms, n_used)
            # Trip-transition counter for active gate.
            if watchdog.alarmed and not _prev_active_alarmed:
                resid_log_stats['active_trips'] += 1
                if watchdog_proposed.alarmed:
                    resid_log_stats['both_alarmed'] += 1
                else:
                    resid_log_stats['only_active'] += 1
            _prev_active_alarmed = watchdog.alarmed
            if watchdog.alarmed:
                # Before giving up: check NAV2 secondary engine position.
                # If NAV2 agrees with known_ecef, the antenna hasn't moved
                # and our FixedPosFilter just blew up — re-seed it instead
                # of exiting.  If NAV2 disagrees, the antenna actually
                # moved.  See docs/architecture-vision.md.
                reseed = False
                if nav2_store is not None:
                    nav2_ecef, nav2_acc, nav2_age = nav2_store.get_ecef(max_age_s=30.0)
                    if nav2_ecef is not None:
                        nav2_sep = float(np.linalg.norm(nav2_ecef - known_ecef))
                        log.warning(
                            "Watchdog: NAV2 position %.1fm from known_ecef "
                            "(hAcc=%.1fm, age=%.0fs) — %s",
                            nav2_sep, nav2_acc or -1, nav2_age,
                            "AGREES (re-seeding filter)" if nav2_sep < 50.0
                            else "DISAGREES (antenna may have moved)",
                        )
                        if nav2_sep < 50.0:
                            # NAV2 confirms antenna is fine.  Reset the
                            # filter state from known_ecef and continue.
                            log.info("Re-seeding FixedPosFilter from known_ecef "
                                     "(NAV2 consensus: antenna stable)")
                            filt = FixedPosFilter(
                                known_ecef,
                                init_ztd_m=init_ztd_m,
                                init_ztd_sigma_m=init_ztd_sigma_m,
                            )
                            prev_t = None
                            watchdog.reset()
                            watchdog_proposed.reset()
                            _prev_active_alarmed = False
                            _prev_proposed_alarmed = False
                            if servo_ctx is not None:
                                # Purge stale servo state so the servo
                                # doesn't act on the corrupted dt_rx
                                _purge_pps_state(servo_ctx)
                                servo_ctx['filter_needs_clock_reset'] = True
                            reseed = True
                    else:
                        log.warning("Watchdog: NAV2 position not available "
                                    "(%s) — cannot verify antenna position",
                                    nav2_store.summary())
                if not reseed:
                    if servo_ctx is not None:
                        _set_clock_class(servo_ctx, "freerun")
                    log.error("POSITION WATCHDOG ALARM: antenna may have moved! "
                              "Servo disabled. Restart with correct position or "
                              "delete position file to re-bootstrap.")
                    break

            dt_rx_ns = filt.x[filt.IDX_CLK] / C * 1e9
            p_clk = filt.P[filt.IDX_CLK, filt.IDX_CLK]
            dt_rx_sigma = math.sqrt(max(0, p_clk)) / C * 1e9
            n_epochs += 1

            # Extract ZTD and ISBs for logging
            dztd_m = 0.0
            dztd_sigma_m = 0.0
            if hasattr(filt, 'IDX_ZTD') and filt.x.shape[0] > filt.IDX_ZTD:
                dztd_m = filt.x[filt.IDX_ZTD]
                dztd_sigma_m = math.sqrt(max(0,
                    filt.P[filt.IDX_ZTD, filt.IDX_ZTD]))
            isb_gal_ns = 0.0
            isb_bds_ns = 0.0
            if hasattr(filt, 'IDX_ISB_GAL') and filt.x.shape[0] > filt.IDX_ISB_GAL:
                isb_gal_ns = filt.x[filt.IDX_ISB_GAL] / C * 1e9
            if hasattr(filt, 'IDX_ISB_BDS') and filt.x.shape[0] > getattr(filt, 'IDX_ISB_BDS', 999):
                isb_bds_ns = filt.x[filt.IDX_ISB_BDS] / C * 1e9

            # Periodic FixedPosFilter ZTD log emit — exposes the pinned
            # filter's ZTD state independently of the parallel AntPosEst
            # PPPFilter reporting.  Needed to evaluate I-175546-main
            # hypothesis "does pinning position settle ZTD?"  Cadence:
            # every 30 epochs (~30s at 1 Hz) to keep volume modest.
            if n_epochs % 30 == 0:
                log.info(
                    "[FIXEDPOS_ZTD] epoch=%d ZTD=%+.0f±%.0fmm "
                    "dt_rx=%+.3fns σ=%.4fns",
                    n_epochs,
                    dztd_m * 1000.0, dztd_sigma_m * 1000.0,
                    dt_rx_ns, dt_rx_sigma,
                )

            # Periodic ZTD soft-tie from refreshed METAR — bounds the
            # rate of ZTD-state drift driven by per-SV systematic-bias
            # absorption (SSR phase-bias substitution L5I→L5Q,
            # antenna PCV miscorrection, multipath).  See I-172521.
            tie_interval = getattr(args, 'ztd_tie_interval_s', 300)
            if (tie_interval and n_epochs > 0 and n_epochs % tie_interval == 0
                    and hasattr(filt, 'apply_ztd_tie')):
                _periodic_ztd_tie(filt, args, lat, alt, n_epochs, log)

            # Correction source
            source = 'SSR' if ssr.n_clock > 0 else 'broadcast'
            ts_str = gps_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:23]

            # CSV output
            if out_w:
                out_w.writerow([
                    ts_str, 'steady',
                    f'{lat:.7f}', f'{lon:.7f}', f'{alt:.3f}',
                    '', f'{dt_rx_ns:.3f}', f'{dt_rx_sigma:.4f}',
                    n_used, f'{resid_rms:.4f}',
                    source, f'{isb_gal_ns:.3f}', f'{isb_bds_ns:.3f}',
                    n_td,
                ])

            # Feed servo if active
            if servo_ctx is not None:
                servo_result = _servo_epoch(
                    servo_ctx, args, filt, obs_event, corr_snapshot, n_epochs,
                    dt_rx_ns, dt_rx_sigma, n_used, known_ecef,
                    resid_rms, isb_gal_ns, isb_bds_ns,
                    pps_match=pps_match,
                )
                if servo_result == "no_pps":
                    skip_stats["servo_no_pps"] += 1
                elif servo_result == "outlier":
                    skip_stats["servo_outlier"] += 1

            # PHC divergence: exit for re-bootstrap
            if servo_ctx is not None and servo_ctx.get('phc_diverged'):
                _set_clock_class(servo_ctx, "freerun")
                log.error("Shutting down — PHC needs re-bootstrap")
                return 5

            # Console status every 10 epochs
            if n_epochs % 10 == 0:
                elapsed = time.time() - start_time
                log.info(
                    f"  [{n_epochs}] {ts_str[:19]} "
                    f"clk={dt_rx_ns:+.1f}ns ±{dt_rx_sigma:.2f}ns "
                    f"n={n_used} rms={resid_rms:.3f}m "
                    f"[{source}]"
                )
            # Periodic [STATUS] line every 60 epochs
            if n_epochs % 60 == 0 and ape_sm is not None and dfe_sm is not None:
                ticc_ok = (servo_ctx.get('ticc_tracker') is not None) if servo_ctx else None
                qvir = servo_ctx.get('qvir') if servo_ctx else None
                log.info("[STATUS] %s", format_status(ape_sm, dfe_sm,
                         ticc_ok=ticc_ok, qvir=qvir))
            now = time.time()
            if now - last_skip_log >= 60.0:
                log.info(f"  Skip stats: {skip_stats}")
                last_skip_log = now
            if now - last_hwm_log >= 1200.0:  # every 20 minutes
                log.info(
                    "  Queue HWM (session max): obs_q=%d obs_hist=%d pps_hist=%d",
                    queue_hwm["obs_queue"], queue_hwm["obs_history"],
                    queue_hwm["pps_history"],
                )
                last_hwm_log = now

    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        stop_event.set()
        if servo_ctx is not None and servo_ctx.get("correlation_gate") is not None:
            gate_stats = {
                "strict_correlation": servo_ctx["correlation_gate"].stats.as_dict(),
                "correction_freshness": correction_gate.stats.as_dict(),
                "steady_state_skips": skip_stats,
                "holdover": dict(servo_ctx["holdover"]),
                "queue_high_water_marks": dict(queue_hwm),
            }
        else:
            gate_stats = {
                "correction_freshness": correction_gate.stats.as_dict(),
                "steady_state_skips": skip_stats,
                "queue_high_water_marks": dict(queue_hwm),
            }
        if servo_ctx is not None:
            _cleanup_servo(servo_ctx)

    elapsed = time.time() - start_time
    log.info(f"Steady state complete: {elapsed:.0f}s, {n_epochs} epochs")
    # I-145915 bake-in summary — side-by-side gate verdicts.
    rs = resid_log_stats
    log.info(
        "[FIXEDPOS_RESID_SUMMARY] epochs=%d active_trips=%d "
        "proposed_trips=%d both_alarmed=%d only_active=%d "
        "pr_disturbances=%d "
        "(only_active = trips that would have STAYED under proposed gate)",
        rs['epochs'], rs['active_trips'], rs['proposed_trips'],
        rs['both_alarmed'], rs['only_active'], rs['pr_disturbances'],
    )
    return gate_stats


# ── Servo helpers (conditional PTP import) ────────────────────────────── #

def _open_pmc(args):
    """Open a PMC client if --pmc is configured.  Returns PmcClient or None."""
    pmc_path = getattr(args, 'pmc', None)
    if not pmc_path:
        return None
    domain = getattr(args, 'pmc_domain', 0)
    try:
        from peppar_fix.pmc import PmcClient
        client = PmcClient(pmc_path, domain=domain)
        client.open()
        log.info("pmc: connected to %s (domain %d)", pmc_path, domain)
        return client
    except Exception as e:
        log.warning("pmc: failed to open %s: %s (clockClass management disabled)", pmc_path, e)
        return None


def _set_clock_class(ctx, state):
    """Set ptp4l clockClass if PMC is configured and state has changed."""
    pmc = ctx.get('pmc')
    if pmc is None:
        return
    if ctx.get('pmc_announced') == state:
        return
    if pmc.set_grandmaster_class(state):
        ctx['pmc_announced'] = state


def _save_osc_freq_corr(ctx):
    """Save refined oscillator frequency corrections on clean shutdown.

    DO adjfine goes to state/dos/<uid>.json; the F9T rx TCXO offset goes
    to state/receivers/<uid>.json.

    Skipped in --freerun mode: the live adjfine there is what the servo
    *would have* written if it were actuating, not a value the PHC has
    actually been steered to.  Persisting it would poison the next
    bootstrap with a fictitious frequency.
    """
    if ctx.get('freerun'):
        log.info("Skipping DO-state save (freerun mode)")
        return
    adjfine = ctx.get('adjfine_ppb', 0.0)
    carrier_tracker = ctx.get('carrier_tracker')

    tcxo_corr = None
    if (carrier_tracker is not None and carrier_tracker._n_d > 0
            and carrier_tracker.drift_rate_ppb != 0):
        tcxo_corr = adjfine - carrier_tracker.drift_rate_ppb

    # Save DO freq offset
    do_uid = ctx.get('do_unique_id')
    if do_uid is not None:
        try:
            from peppar_fix.do_state import save_do_freq_offset
            save_do_freq_offset(do_uid, adjfine)
            log.info("Saved DO freq offset: adjfine=%.1f ppb (DO %s)",
                     adjfine, do_uid)
        except Exception as e:
            log.warning("Failed to save DO state: %s", e)

    # Save rx TCXO offset + last known dt_rx to receiver state
    receiver_uid = ctx.get('receiver_unique_id')
    if receiver_uid is not None:
        try:
            from peppar_fix.receiver_state import (load_receiver_state,
                                                    save_receiver_state)
            state = load_receiver_state(receiver_uid)
            if state is not None:
                state.setdefault("tcxo", {})
                if tcxo_corr is not None:
                    state["tcxo"]["last_known_freq_offset_ppb"] = tcxo_corr
                if ctx.get('_prev_dt_rx_ns') is not None:
                    state["tcxo"]["last_known_dt_rx_ns"] = (
                        ctx['_prev_dt_rx_ns'])
                state["tcxo"]["updated"] = time.strftime(
                    "%Y-%m-%dT%H:%M:%SZ", time.gmtime())
                save_receiver_state(state)
        except Exception as e:
            log.warning("Failed to save rx TCXO state: %s", e)


def _log_do_characterization(args):
    """Log DO characterization summary from state or legacy file.

    Reports DO noise floor, dominant noise types, source crossovers,
    and a recommended loop bandwidth for the active servo input
    (informational only — does not auto-tune).
    """
    char = None

    # Try DO state first
    do_uid = _resolve_do_uid(args)
    if do_uid:
        try:
            from peppar_fix.do_state import load_do_state
            do_state = load_do_state(do_uid)
            if do_state is not None and do_state.get("characterization"):
                char = do_state["characterization"]
        except Exception:
            pass

    # Fall back to legacy file
    if char is None:
        char_path = getattr(args, 'do_char_file', None) or 'data/do_characterization.json'
        try:
            import json as _json
            with open(char_path) as f:
                char = _json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            log.info("DO characterization: not available "
                     "(run with --freerun to create one)")
            return

    log.info("DO characterization: %s @ %s (captured %s)",
             char.get('do_label', 'unknown'),
             char.get('host', 'unknown'),
             char.get('captured', 'unknown'))
    for name, src in char.get('sources', {}).items():
        slope = src.get('slope')
        slope_str = f"{slope:+.2f}" if slope is not None else "n/a"
        units = src.get('units', '?')
        log.info("  %-22s ASD@0.1Hz=%.4f %s/√Hz  slope=%s (%s)",
                 name,
                 src.get('asd_at_0.1Hz', 0.0),
                 units,
                 slope_str,
                 src.get('noise_type', 'unknown'))
    crossovers = char.get('crossovers', {})
    if crossovers:
        log.info("DO characterization crossovers:")
        for pair, hz in crossovers.items():
            log.info("  %s: %.4f Hz (~%.0fs timescale)",
                     pair, hz, 1.0 / hz if hz > 0 else 0)


def _init_noise_estimator(args):
    """Create InBandNoiseEstimator, warm-starting from saved state if available."""
    if args.freerun:
        return None
    from peppar_fix.noise_estimator import load_noise_state, InBandNoiseEstimator
    do_uid = _resolve_do_uid(args)
    if do_uid is not None:
        est = load_noise_state(do_uid)
        if est is not None:
            return est
    return InBandNoiseEstimator()


def _init_carrier_tracker(args):
    """Create CarrierPhaseTracker, seeding D from state or drift file.

    D = phc_freq_corr - tcxo_freq_corr, computed from two independently
    measured oscillator frequency corrections, loaded from DO state
    (phc_corr) and receiver state (tcxo_corr).
    """
    tracker = CarrierPhaseTracker()

    phc_corr = None
    tcxo_corr = None
    source = None

    # Try state files first
    do_uid = _resolve_do_uid(args)
    receiver_uid = getattr(args, 'receiver_unique_id', None)
    if do_uid:
        try:
            from peppar_fix.do_state import load_do_state
            do_state = load_do_state(do_uid)
            if do_state is not None:
                phc_corr = do_state.get("last_known_freq_offset_ppb")
        except Exception:
            pass
    if receiver_uid is not None:
        try:
            from peppar_fix.receiver_state import load_receiver_state
            rx_state = load_receiver_state(receiver_uid)
            if rx_state is not None:
                tcxo = rx_state.get("tcxo", {})
                tcxo_corr = tcxo.get("last_known_freq_offset_ppb")
        except Exception:
            pass
    if phc_corr is not None and tcxo_corr is not None:
        source = "state"
        d = phc_corr - tcxo_corr
        tracker.drift_rate_ppb = d
        log.info("Carrier tracker: D=%.1f ppb "
                 "(phc_corr=%.1f - tcxo_corr=%.1f) from %s",
                 d, phc_corr, tcxo_corr, source)
    return tracker


def _resolve_do_uid(args):
    """Resolve the DO unique_id from --do-label or PHC MAC.

    External DOs (VCOCXO, ClockMatrix) use --do-label.
    Bundled PHC+DO uses the PHC MAC address.
    """
    do_label = getattr(args, 'do_label', None)
    if do_label:
        return do_label
    phc_dev = getattr(args, 'servo', None)
    if phc_dev:
        try:
            from peppar_fix.do_state import phc_unique_id
            return phc_unique_id(phc_dev)
        except Exception:
            return phc_dev
    return None


# ── DO bootstrap (absorbed from phc_bootstrap.py) ─────────────────── #


def _bootstrap_measure_freq_and_clock(args, timestamper, known_ecef, obs_queue,
                                       beph, ssr, stop_event):
    """Shared bootstrap preamble: measure PPS frequency and run a short
    FixedPosFilter to estimate dt_rx.

    timestamper: a Timestamper instance (ExttsTimestamper or TiccTimestamper).
    The frequency measurement runs concurrently with obs_queue filling from
    the serial/NTRIP threads that are already active.

    Returns (pps_freq_ppb, pps_freq_unc, dt_rx_ns, dt_rx_series) on
    success, or None on fatal error.
    """
    # ── 1. Measure PPS frequency ──────────────────────────────────── #
    log.info("=== DO Bootstrap: measuring DO frequency from PPS ===")
    pps_freq_ppb, pps_freq_sigma, pps_freq_n = timestamper.measure_pps_frequency(
        n_samples=args.bootstrap_epochs,
        timeout_s=args.bootstrap_epochs + 15)  # +15 for TICC boot headroom
    if pps_freq_ppb is None:
        log.error("No PPS events — cannot bootstrap DO")
        return None
    pps_freq_unc = pps_freq_sigma / math.sqrt(max(1, pps_freq_n))
    log.info("PPS frequency error: %.1f ±%.1f ppb (σ=%.1f, n=%d)",
             pps_freq_ppb, pps_freq_unc, pps_freq_sigma, pps_freq_n)

    # ── 2. Short FixedPosFilter for dt_rx ─────────────────────────── #
    log.info("Running %d-epoch FixedPosFilter for clock estimate...",
             args.bootstrap_epochs)
    filt = FixedPosFilter(known_ecef)
    prev_t = None
    dt_rx_ns = None
    dt_rx_sigma_ns = None
    dt_rx_series = []
    n_epochs = 0

    for _ in range(args.bootstrap_epochs * 3):  # generous timeout
        if stop_event.is_set():
            return None
        try:
            gps_time, observations = obs_queue.get(timeout=5)
        except Exception:
            continue
        if len(observations) < 4:
            continue

        corrections = RealtimeCorrections(beph, ssr)
        dt = (gps_time - prev_t).total_seconds() if prev_t else 1.0
        if prev_t and 0 < dt <= 30:
            filt.predict(dt)
        prev_t = gps_time

        n_used, resid, n_td = filt.update(
            observations, corrections, gps_time, clk_file=corrections)
        if n_used < 4:
            continue

        dt_rx_ns = filt.x[filt.IDX_CLK] / C * 1e9
        p_clk = filt.P[filt.IDX_CLK, filt.IDX_CLK]
        dt_rx_sigma_ns = math.sqrt(max(0, p_clk)) / C * 1e9
        dt_rx_series.append(dt_rx_ns)
        n_epochs += 1

        if n_epochs % 5 == 0 or n_epochs == args.bootstrap_epochs:
            log.info("  [%d/%d] dt_rx=%.1f ±%.1f ns  n_used=%d",
                     n_epochs, args.bootstrap_epochs, dt_rx_ns,
                     dt_rx_sigma_ns, n_used)

        if n_epochs >= args.bootstrap_epochs:
            break

    if dt_rx_ns is None:
        log.error("Filter did not converge in %d epochs", args.bootstrap_epochs)
        return None

    log.info("Clock estimate: dt_rx=%.1f ±%.1f ns after %d epochs",
             dt_rx_ns, dt_rx_sigma_ns, n_epochs)

    return pps_freq_ppb, pps_freq_unc, dt_rx_ns, dt_rx_series


def _bootstrap_compute_base_freq(args, pps_freq_ppb, pps_freq_unc,
                                  current_adj_ppb, dt_rx_series):
    """Compute base frequency and rx TCXO correction from bootstrap data.

    Returns (base_freq, tcxo_freq_corr_ppb).
    """
    freq_sane = abs(pps_freq_ppb) <= args.freq_tolerance_ppb
    if freq_sane:
        log.info("Frequency sane: PPS=%.1f ±%.1f ppb (within ±%.1f ppb)",
                 pps_freq_ppb, pps_freq_unc, args.freq_tolerance_ppb)
    else:
        log.warning("Frequency error: PPS=%.1f ±%.1f ppb — outside ±%.1f ppb",
                    pps_freq_ppb, pps_freq_unc, args.freq_tolerance_ppb)

    if freq_sane:
        base_freq = current_adj_ppb
        log.info("Base frequency: %.1f ppb (current, freq sane)", base_freq)
    else:
        # Pull last-known frequency from DO state if we can't trust PPS.
        stored_adjfine = None
        do_uid = ctx_do_uid if (ctx_do_uid := getattr(args, 'do_unique_id', None)) else None
        if do_uid is None:
            do_uid = _resolve_do_uid(args)
        if do_uid is not None:
            try:
                from peppar_fix.do_state import load_do_state
                s = load_do_state(do_uid)
                if s is not None:
                    stored_adjfine = s.get("last_known_freq_offset_ppb")
            except Exception:
                pass
        if pps_freq_unc is not None and pps_freq_unc <= args.freq_tolerance_ppb:
            base_freq = current_adj_ppb - pps_freq_ppb
            freq_source = ("PPS correction: %.1f - %.1f"
                           % (current_adj_ppb, pps_freq_ppb))
        elif stored_adjfine is not None:
            base_freq = stored_adjfine
            freq_source = "DO state"
        else:
            base_freq = 0.0
            freq_source = "default (no data)"
        log.info("Base frequency: %.1f ppb (%s)", base_freq, freq_source)

    # rx TCXO frequency correction from dt_rx series
    tcxo_freq_corr_ppb = None
    if len(dt_rx_series) >= 3:
        n_dt = len(dt_rx_series)
        sx = sum(range(n_dt))
        sy = sum(dt_rx_series)
        sxy = sum(i * v for i, v in enumerate(dt_rx_series))
        sxx = sum(i * i for i in range(n_dt))
        denom = n_dt * sxx - sx * sx
        if denom != 0:
            tcxo_freq_corr_ppb = (n_dt * sxy - sx * sy) / denom
            log.info("F9T TCXO freq correction: %.1f ppb (from %d samples)",
                     tcxo_freq_corr_ppb, n_dt)

    return base_freq, tcxo_freq_corr_ppb


def _do_tadd_arm(args):
    """ARM the TADD divider if configured.  Extracted for reuse."""
    tadd_gpio = getattr(args, 'tadd_gpio', None)
    if tadd_gpio is not None:
        from peppar_fix.tadd import TADDDivider
        tadd_hold = getattr(args, 'tadd_hold_s', 1.1)
        tadd = TADDDivider(arm_gpio=tadd_gpio, arm_hold_s=tadd_hold)
        tadd.setup()
        try:
            tadd.arm()
            log.info("TADD ARM complete (GPIO%d) — DO PPS synced to GNSS PPS "
                     "(phase offset ≤%d ns)", tadd_gpio, tadd.max_phase_offset_ns)
        finally:
            tadd.teardown()
    else:
        log.info("No TADD GPIO configured — assuming DO PPS already synced")


def _maybe_step_divider_on_phase(args):
    """Phase-conditional TADD ARM for the warm-start path.

    Warm-start skips ``_do_bootstrap_init`` (which is where TADD ARM
    normally runs).  Divider phase state isn't persisted in any state
    file — after any physical event (cable swap, power glitch, even
    just a watchdog re-bootstrap on a host where the divider draws
    power from the engine board) the divider may be at a wrong
    sub-second phase.

    This helper measures the current DO-vs-ref phase via a brief TICC
    differential read and calls ``_do_tadd_arm`` if the magnitude
    exceeds ``--divider-step-threshold-us``.  Below threshold the
    EKF's normal slewing closes the gap in seconds; above threshold
    stepping is dramatically faster (≤400 ns post-step vs minutes of
    slew at max actuator authority).

    Returns True if a step was performed, False otherwise.
    """
    # Disabled by flag.
    if getattr(args, 'no_divider_step', False):
        log.info("--no-divider-step set — skipping phase-conditional ARM")
        return False
    # No divider configured — nothing to step.
    if getattr(args, 'tadd_gpio', None) is None:
        return False
    # TICC port is the only phase-measurement source on TICC-only
    # (VCOCXO) hosts.  Without it we can't decide; default to ARM
    # (conservative — small false-positive ARM cost is preferable to
    # missing a real phase divergence).
    ticc_port = getattr(args, 'ticc_port', None)
    if ticc_port is None:
        log.info("No TICC port — defaulting to ARM (no phase observation "
                 "available)")
        _do_tadd_arm(args)
        return True

    threshold_us = getattr(args, 'divider_step_threshold_us', 5.0)
    log.info("Warm-start phase check: measuring DO-vs-ref via TICC...")
    try:
        from peppar_fix.timestamper import measure_differential_phase
        ticc_baud = getattr(args, 'ticc_baud', 115200)
        do_ch = getattr(args, 'ticc_phc_channel', 'chA')
        ref_ch = getattr(args, 'ticc_ref_channel', 'chB')
        median_ns, n = measure_differential_phase(
            ticc_port, ticc_baud, do_ch, ref_ch,
            n_samples=3, timeout_s=20)
    except Exception as e:
        log.warning("Phase-conditional ARM measurement failed (%s) — "
                    "defaulting to ARM", e)
        _do_tadd_arm(args)
        return True

    if median_ns is None:
        log.warning("TICC produced no phase samples — defaulting to ARM")
        _do_tadd_arm(args)
        return True

    threshold_ns = threshold_us * 1000.0
    if abs(median_ns) > threshold_ns:
        log.info("Phase offset %.1f µs > %.1f µs threshold (n=%d) — "
                 "stepping divider via TADD ARM",
                 median_ns / 1000.0, threshold_us, n)
        _do_tadd_arm(args)
        return True
    else:
        log.info("Phase offset %.2f µs ≤ %.1f µs threshold (n=%d) — "
                 "within slew envelope, no ARM",
                 median_ns / 1000.0, threshold_us, n)
        return False


def _do_bootstrap_vcocxo(args, ptp, pps_freq_ppb, pps_freq_unc,
                          dt_rx_ns, dt_rx_series):
    """Bootstrap an external VCOCXO: ARM TADD divider, seed DAC frequency.

    The TADD ARM synchronizes the divider's 1 PPS output to the GNSS PPS.
    The DAC is set to compensate the measured frequency offset.
    No PHC phase step — the TADD ARM handles phase alignment.

    When called from the TICC-only path, TADD ARM was already done
    before the frequency measurement (the TICC needs DO PPS aligned
    first).  When called from the PHC path, TADD ARM happens here
    (EXTTS can measure the free-running DO before ARM).

    Returns True on success, False on fatal error.
    """
    do_label = getattr(args, 'do_label', None) or 'vcocxo'

    # ── 1. ARM the TADD divider (skipped if already done) ─────────── #
    if ptp is not None:
        # PHC path: ARM happens here, after EXTTS frequency measurement.
        _do_tadd_arm(args)
    # else: TICC-only path — ARM was done in _do_bootstrap_init before
    # the TICC frequency measurement.

    # ── 2. Seed DAC frequency ─────────────────────────────────────── #
    dac_bus = getattr(args, 'dac_bus', None)
    if dac_bus is None:
        log.error("VCOCXO bootstrap requires --dac-bus")
        return False

    from peppar_fix.dac_actuator import DacActuator

    dac_addr = int(getattr(args, 'dac_addr', '0x60'), 0)
    ppb_per_code = getattr(args, 'dac_ppb_per_code', None)
    if ppb_per_code is None:
        log.error("VCOCXO bootstrap requires --dac-ppb-per-code")
        return False

    dac = DacActuator(
        bus_num=dac_bus,
        addr=dac_addr,
        bits=getattr(args, 'dac_bits', 12),
        center_code=getattr(args, 'dac_center_code', None),
        ppb_per_code=ppb_per_code,
        max_ppb=getattr(args, 'dac_max_ppb', None),
        dac_type=getattr(args, 'dac_type', 'mcp4725'),
        dac_gain=getattr(args, 'dac_gain', 0) or 0,
    )
    dac.setup()

    # Compute base frequency from drift file or PPS measurement
    base_freq, tcxo_freq_corr_ppb = _bootstrap_compute_base_freq(
        args, pps_freq_ppb, pps_freq_unc, dac.read_frequency_ppb(),
        dt_rx_series)

    actual = dac.adjust_frequency_ppb(base_freq)
    log.info("DAC frequency set: requested=%.1f ppb, actual=%.1f ppb",
             base_freq, actual)

    # Stash the bootstrap frequency so _setup_servo can re-apply it
    # after the DacActuator's setup() (which resets to center).
    args._bootstrap_freq_ppb = base_freq

    # Close I2C bus but do NOT reset DAC to center.  The OCXO must
    # stay at the seeded frequency continuously — any gap causes phase
    # accumulation from the uncorrected free-running offset.
    # _setup_servo will open a new DacActuator and re-apply the
    # frequency, but the hardware DAC register retains the last code.
    if hasattr(dac, '_bus') and dac._bus is not None:
        dac._bus.close()
        dac._bus = None

    try:
        from peppar_fix.do_state import load_do_state, save_do_state
        from peppar_fix.do_state import new_do_state
        do_uid = do_label
        state = load_do_state(do_uid) or new_do_state(do_uid, label=do_label)
        state["last_known_freq_offset_ppb"] = base_freq
        state["updated"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        save_do_state(state)
        log.info("DO freq saved: base=%.1f ppb, dt_rx=%.1f ns",
                 base_freq, dt_rx_ns)
    except Exception as e:
        log.warning("Failed to save DO state: %s", e)

    log.info("VCOCXO bootstrap complete — servo may start")
    return True


def _do_bootstrap_phc(args, ptp, pps_freq_ppb, pps_freq_unc,
                       dt_rx_ns, dt_rx_series, stop_event):
    """Bootstrap a PHC-based DO: evaluate phase, step if needed, glide slope.

    This is the original PHC bootstrap path.
    Returns True on success, False on fatal error.
    """
    from phc_bootstrap import (
        _realtime_to_phc_offset_s, _enable_pps_out,
    )

    extts_ch = args.extts_channel

    # ── 3. Evaluate PHC phase ─────────────────────────────────────── #
    ptp.enable_extts(extts_ch, rising_edge=True)
    pps_event = ptp.read_one_rising_edge(timeout_s=3.0)
    pps_realtime_ns = time.clock_gettime_ns(time.CLOCK_REALTIME)
    ptp.disable_extts(extts_ch)

    if pps_event is None:
        log.error("No PPS event — cannot evaluate DO phase")
        return False

    phc_sec, phc_nsec, _idx, _mono, _qr, _pa = pps_event
    phc_rounded_sec = phc_sec if phc_nsec < 500_000_000 else phc_sec + 1

    offset_s = _realtime_to_phc_offset_s(
        args.phc_timescale, args.leap, args.tai_minus_gps)
    utc_sec = round(pps_realtime_ns / 1_000_000_000)
    target_sec = utc_sec + offset_s

    epoch_offset = phc_rounded_sec - target_sec
    pps_error_ns = (phc_nsec if phc_nsec < 500_000_000
                    else phc_nsec - 1_000_000_000)
    phase_error_ns = epoch_offset * 1_000_000_000 + pps_error_ns

    log.info("DO phase: epoch_offset=%ds, pps_error=%+.0f ns, "
             "total_phase_error=%+.0f ns",
             epoch_offset, pps_error_ns, phase_error_ns)

    # ── 4. Frequency + phase sanity check ─────────────────────────── #
    freq_sane = abs(pps_freq_ppb) <= args.freq_tolerance_ppb
    phase_sane = abs(phase_error_ns) < args.phc_step_threshold_ns

    if phase_sane and freq_sane:
        log.info("PHC state is sane — blessing without intervention")
        _enable_pps_out(ptp, args)
        return True

    # ── 5. Compute base frequency ─────────────────────────────────── #
    base_freq, tcxo_freq_corr_ppb = _bootstrap_compute_base_freq(
        args, pps_freq_ppb, pps_freq_unc, ptp.read_adjfine(),
        dt_rx_series)

    phi_0 = phase_error_ns

    if not phase_sane:
        # Disable PEROUT before stepping (i226 safety)
        if args.pps_out_pin >= 0:
            ptp.disable_perout(args.pps_out_channel)
            log.info("Disabled PEROUT before PHC step")
        try:
            log.info("Stepping PHC by %+.0f ns (ADJ_SETOFFSET)", -phase_error_ns)
            ptp.adj_setoffset(-phase_error_ns)
            # ADJ_SETOFFSET on i226 has ~1.7 µs late-application
            # propagation delay that decays in <1 ms.  Sleep 1 ms so any
            # subsequent readback / verify sees the offset fully applied.
            # Characterisation: tools/adj_setoffset_relative_precision.py
            # shows median/p95 drops from 1777/1906 ns at wait=0 to
            # 57/200 ns at wait=1 ms.
            time.sleep(0.001)
        except OSError as e:
            log.warning("ADJ_SETOFFSET failed (%s), falling back to optimal stopping", e)
            pps_anchor_ns = target_sec * 1_000_000_000
            residual, attempts, met = ptp.step_to(
                pps_anchor_ns=pps_anchor_ns,
                pps_realtime_ns=pps_realtime_ns,
                phc_optimal_stop_limit_s=args.phc_optimal_stop_limit_s,
                phc_settime_lag_ns=args.phc_settime_lag_ns,
            )
            log.info("Step: residual=%+.0f ns, attempts=%d, %s",
                     residual, attempts, "ACCEPTED" if met else "DEADLINE")

        # Verify via next PPS edge — pure PHC-domain measurement.
        # v_sub_ns = signed nanoseconds from the nearest PHC integer
        # second to the PPS edge.  Previously we computed v_target from
        # CLOCK_REALTIME which is freerunning on these hosts (no NTP /
        # PTP / ts2phc), accumulating ~600 ppb rate offset from the PHC
        # TCXO during the 3-second verify wait; that drift was being
        # reported as "phi_0" with µs-scale magnitude that did not
        # reflect the actual ADJ_SETOFFSET step precision (which the
        # tools/adj_setoffset_relative_precision characterization shows
        # is ~50 ns median / ~200 ns p95 on i226 once the offset has
        # propagated).  Using only v_sub_ns gives the real residual.
        # The remaining ±4 ns F9T qErr offset is uncorrected here; when
        # the engine threads qErr through to this point, subtract it.
        # See I-013346-main.
        ptp.enable_extts(extts_ch, rising_edge=True)
        evt = ptp.read_one_rising_edge(timeout_s=3.0)
        if evt is not None:
            v_nsec = evt[1]
            v_sub_ns = (v_nsec if v_nsec < 500_000_000
                        else v_nsec - 1_000_000_000)
            phi_0 = v_sub_ns
            log.info("PPS verify: phi_0 = %+.0f ns "
                     "(PHC offset from F9T PPS edge; ±4 ns qErr "
                     "uncorrected)", phi_0)
        else:
            log.warning("No PPS event — assuming step landed at 0")
            phi_0 = 0
        ptp.disable_extts(extts_ch)
    else:
        log.info("Phase OK (%+.0f ns) — frequency-only correction",
                 phase_error_ns)

    # Glide slope — disabled.  DOFreqEst handles phase convergence
    # internally via its LQR L[2] term.  The PI glide used args.track_ki
    # to compute a frequency ramp; the EKF doesn't need it and the
    # mismatch between initial_freq (with glide) and base_freq (without)
    # caused the DOFreqEst to diverge on PHC hosts.
    glide_offset = 0.0
    if False:
        omega_n = math.sqrt(args.track_ki)
        zeta = args.glide_zeta
        glide_offset = -zeta * omega_n * phi_0
        # Clamp within servo control authority
        if args.track_max_ppb:
            max_glide = args.track_max_ppb - abs(base_freq)
            if abs(glide_offset) > max_glide:
                clamped = math.copysign(max_glide, glide_offset)
                log.warning("Glide clamped: %.0f → %.0f ppb (track_max=%.0f)",
                            glide_offset, clamped, args.track_max_ppb)
                glide_offset = clamped
        t_cross = abs(phi_0 / glide_offset) if glide_offset != 0 else float('inf')
        log.info("Glide: zeta=%.2f, omega_n=%.4f, phi_0=%+.0f ns, "
                 "offset=%+.1f ppb, zero-crossing ~%.0fs",
                 zeta, omega_n, phi_0, glide_offset, t_cross)

    target_freq = base_freq + glide_offset

    # ClockMatrix handoff (OTC hardware)
    cm_bus = getattr(args, 'clockmatrix_bus', None)
    if cm_bus is not None:
        try:
            from peppar_fix.clockmatrix import ClockMatrixI2C
            from peppar_fix.clockmatrix_actuator import (
                ClockMatrixActuator, ppb_to_fcw)

            cm_addr = int(getattr(args, 'clockmatrix_addr', '0x58'), 0)
            cm_dpll = getattr(args, 'clockmatrix_dpll_actuator', 3)
            cm_i2c = ClockMatrixI2C(cm_bus, cm_addr)

            log.info("Zeroing PHC adjfine (frequency goes to ClockMatrix FCW)")
            ptp.adjfine(0.0)

            actuator = ClockMatrixActuator(cm_i2c, dpll_id=cm_dpll)
            actuator.setup()
            actuator.adjust_frequency_ppb(target_freq)

            log.info("ClockMatrix FCW set: %.1f ppb (base=%.1f + glide=%.1f)",
                     target_freq, base_freq, glide_offset)

            _save_phc_bootstrap_freq(args, base_freq, dt_rx_ns)
            cm_i2c.close()

            _enable_pps_out(ptp, args)
            log.info("PHC bootstrap complete (ClockMatrix)")
            return True

        except ImportError:
            log.warning("smbus2 not available — falling back to PHC adjfine")
        except Exception as e:
            log.error("ClockMatrix handoff failed: %s — falling back", e)

    # Normal PHC-only path
    log.info("Setting adjfine: %.1f ppb (base=%.1f + glide=%.1f)",
             target_freq, base_freq, glide_offset)
    ptp.adjfine(target_freq)

    _save_phc_bootstrap_freq(args, base_freq, dt_rx_ns)

    _enable_pps_out(ptp, args)
    log.info("PHC bootstrap complete — servo may start")
    return True


def _save_phc_bootstrap_freq(args, base_freq, dt_rx_ns):
    """Persist the computed base adjfine to DO state."""
    try:
        from peppar_fix.do_state import phc_unique_id, save_do_freq_offset
        save_do_freq_offset(phc_unique_id(args.servo), base_freq)
        log.info("DO freq saved: base=%.1f ppb, dt_rx=%.1f ns",
                 base_freq, dt_rx_ns)
    except Exception as e:
        log.warning("Failed to save DO state: %s", e)


def _do_bootstrap_init(args, ptp, known_ecef, obs_queue, beph, ssr,
                        stop_event, dfe_sm=None):
    """Bootstrap DO phase and frequency before servo starts.

    Dispatches to the appropriate bootstrap path based on --do-type:
    - phc: PHC phase step + adjfine (original path)
    - vcocxo: TADD ARM + DAC frequency seed
    - clockmatrix: PHC phase step + ClockMatrix FCW (handled within PHC path)

    Creates the appropriate Timestamper (EXTTS or TICC) for frequency
    measurement.  When ptp is None (TICC-only mode), only the VCOCXO
    path is valid.

    Returns True on success, False on fatal error.
    """
    from peppar_fix.timestamper import ExttsTimestamper, TiccDifferentialTimestamper

    if dfe_sm is not None:
        dfe_sm.transition(DOFreqEstState.PHASE_SETTING, "DO bootstrap starting")

    do_type = getattr(args, 'do_type', 'phc')

    if ptp is None:
        if do_type != 'vcocxo':
            log.error("TICC-only servo requires do_type=vcocxo (got %s). "
                      "PHC-based DOs need --servo.", do_type)
            return False
        # For VCOCXO without PHC: reset DAC to center, ARM TADD, then
        # measure frequency.  The DAC must be at a known state before
        # TICC measures — otherwise a stale DAC setting from a prior
        # run corrupts the frequency estimate (the TICC measures
        # crystal + old_dac, but _bootstrap_compute_base_freq assumes
        # current_adj=0).
        # TODO(cleanup): this duplicates DacActuator.setup() — the
        # TICC-drive refactor should unify the DAC lifecycle.
        dac_bus = getattr(args, 'dac_bus', None)
        if dac_bus is not None:
            from peppar_fix.dac_actuator import DacActuator
            _dac_reset = DacActuator(
                bus_num=dac_bus,
                addr=int(getattr(args, 'dac_addr', '0x60'), 0),
                bits=getattr(args, 'dac_bits', 12),
                ppb_per_code=getattr(args, 'dac_ppb_per_code', 1.0),
                dac_type=getattr(args, 'dac_type', 'mcp4725'),
                dac_gain=getattr(args, 'dac_gain', 0) or 0,
            )
            _dac_reset.setup()  # writes ctrl reg + center code
            _dac_reset.teardown()
            log.info("DAC reset to center before TICC measurement")
        _do_tadd_arm(args)
        settle_s = 3
        log.info("Waiting %ds for post-ARM settling before TICC measurement...",
                 settle_s)
        time.sleep(settle_s)
    else:
        # phc_bootstrap helpers expect args.ptp_dev; engine uses args.servo
        args.ptp_dev = args.servo

    # Build the timestamper for frequency measurement.
    # EXTTS is preferred for bootstrap: opening the TICC serial port
    # reboots the Arduino (HUPCL), which can cause USB re-enumeration
    # and crash the F9T serial reader on hosts sharing the USB bus.
    # TICC takes over as primary PPS source in the correlation gate
    # (started later, after the serial reader is established).
    ticc_port = getattr(args, 'ticc_port', None)
    if ptp is not None:
        timestamper = ExttsTimestamper(ptp, args.extts_channel,
                                       pps_pin=getattr(args, 'pps_pin', None))
        log.info("Bootstrap timestamper: EXTTS (channel %d, pin %s)",
                 args.extts_channel, args.pps_pin)
    elif ticc_port:
        ticc_baud = getattr(args, 'ticc_baud', 115200)
        timestamper = TiccDifferentialTimestamper(
            ticc_port, ticc_baud,
            do_channel=getattr(args, 'ticc_phc_channel', 'chA'),
            ref_channel=getattr(args, 'ticc_ref_channel', 'chB'))
        log.info("Bootstrap timestamper: TICC differential (%s, %s-%s)",
                 ticc_port, timestamper.do_channel, timestamper.ref_channel)
    else:
        log.error("No timestamper available: need --servo (EXTTS) or --ticc-port")
        return False

    # Shared preamble: measure PPS frequency and estimate dt_rx.
    # The TICC frequency measurement runs here — overlapping with the
    # obs_queue filling from serial/NTRIP threads already active.
    try:
        result = _bootstrap_measure_freq_and_clock(
            args, timestamper, known_ecef, obs_queue, beph, ssr, stop_event)
    except Exception as e:
        log.error("Bootstrap measurement failed: %s", e)
        return False
    if result is None:
        return False
    pps_freq_ppb, pps_freq_unc, dt_rx_ns, dt_rx_series = result

    if dfe_sm is not None:
        dfe_sm.transition(DOFreqEstState.FREQ_VERIFYING,
                          f"freq measured ({pps_freq_ppb:+.1f} ppb)")

    if do_type == 'vcocxo':
        return _do_bootstrap_vcocxo(
            args, ptp, pps_freq_ppb, pps_freq_unc,
            dt_rx_ns, dt_rx_series)
    else:
        # PHC and ClockMatrix both go through the PHC path
        # (ClockMatrix is a PHC + external frequency actuator)
        return _do_bootstrap_phc(
            args, ptp, pps_freq_ppb, pps_freq_unc,
            dt_rx_ns, dt_rx_series, stop_event)


def _load_timestamper_bias(args, key):
    """Read state/dos/<do_uid>.json and return the bias_ns for a key.

    Returns None when the file or key is absent — bias correction
    becomes a no-op for that arm.  Static calibration per
    docs/dofreq-est-measurement-ladder.md: characterize once, persist,
    use forever (until cabling / reference / chain changes).  The
    engine never auto-recalibrates; that's a separate operator-driven
    tool (scripts/calibrate_timestampers.py).
    """
    do_uid = _resolve_do_uid(args)
    if not do_uid:
        return None
    here = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(here)
    path = os.path.join(repo_root, "state", "dos", f"{do_uid}.json")
    if not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            data = json.load(f)
        ts = data.get("timestampers", {}).get(key)
        if ts is None:
            return None
        bias = ts.get("bias_ns")
        if bias is None:
            return None
        log.info("Timestamper bias loaded: %s = %+.3f ns "
                 "(calibrated_at=%s)",
                 key, float(bias), ts.get("calibrated_at", "unknown"))
        return float(bias)
    except (OSError, ValueError, TypeError) as e:
        log.warning("Failed to read timestamper bias for %s from %s: %s",
                    key, path, e)
        return None


def _setup_servo(args, known_ecef, qerr_store, *, extint_store=None, ptp=None):
    """Set up servo (PHC-based or TICC-only).

    Returns context dict on success, or an int exit code on failure:
    1 = fatal (device or library error), 3 = no PPS (retryable).

    If ptp is provided, uses that PtpDevice instead of opening a new one.
    If ptp is None and args.servo is set, opens the PTP device.
    If ptp is None and args.servo is not set (TICC-only mode), the servo
    runs without any PHC — TICC provides measurements and a DAC or
    ClockMatrix actuator steers the oscillator.
    """
    gate_stats = None
    try:
        from peppar_fix import DisciplineScheduler
        from peppar_fix.timestamper_state import TimestamperParams
    except ImportError:
        log.error("peppar_fix library not available for servo")
        return 1

    # Resolve timestamper noise parameters from state (or defaults)
    ts_params = TimestamperParams.resolve(args)
    log.info("Timestamper params: %s", ts_params)

    # ── PHC setup (skipped in TICC-only mode) ─────────────────────── #
    have_phc = ptp is not None or args.servo
    if ptp is None and args.servo:
        try:
            from peppar_fix import PtpDevice
            ptp = PtpDevice(args.servo)
        except OSError as e:
            log.error(f"Cannot open PTP device {args.servo}: {e}")
            return 1

    if have_phc:
        caps = ptp.get_caps()
        log.info(f"PHC: {args.servo}, max_adj={caps['max_adj']} ppb, "
                 f"n_extts={caps['n_ext_ts']}, n_pins={caps['n_pins']}")
        bootstrap_adj = ptp.read_adjfine()
        log.info("PHC adjfine from bootstrap: %.1f ppb", bootstrap_adj)
    else:
        # TICC-only: no PHC caps.  max_adj comes from the actuator later.
        caps = None
        bootstrap_adj = 0.0
        log.info("TICC-only servo: no PHC — measurements from TICC, "
                 "actuator must be DAC or ClockMatrix")

    _log_do_characterization(args)

    # ── Construct frequency actuator ──────────────────────────────── #
    # Priority: DAC > ClockMatrix > PHC adjfine (last resort, PHC only).
    actuator = None
    actuator_type = None

    if getattr(args, 'dac_bus', None) is not None:
        try:
            from peppar_fix.dac_actuator import DacActuator
            dac_addr = int(getattr(args, 'dac_addr', '0x60'), 0)
            ppb_per_code = getattr(args, 'dac_ppb_per_code', None)
            if ppb_per_code is None:
                log.error("--dac-ppb-per-code required for DAC actuator")
            else:
                actuator = DacActuator(
                    bus_num=args.dac_bus,
                    addr=dac_addr,
                    bits=getattr(args, 'dac_bits', 12),
                    center_code=getattr(args, 'dac_center_code', None),
                    ppb_per_code=ppb_per_code,
                    max_ppb=getattr(args, 'dac_max_ppb', None),
                    dac_type=getattr(args, 'dac_type', 'mcp4725'),
                    dac_gain=getattr(args, 'dac_gain', 0) or 0,
                )
                actuator_type = "dac"
                log.info("Using DAC actuator: bus=%d addr=0x%02x bits=%d ppb/code=%.4f",
                         args.dac_bus, dac_addr, args.dac_bits, ppb_per_code)
        except Exception as e:
            log.error("DAC actuator init failed: %s", e)
            actuator = None
    elif getattr(args, 'clockmatrix_bus', None) is not None:
        try:
            from peppar_fix.clockmatrix import ClockMatrixI2C
            from peppar_fix.clockmatrix_actuator import ClockMatrixActuator
            cm_i2c = ClockMatrixI2C(
                bus_num=args.clockmatrix_bus,
                addr=int(getattr(args, 'clockmatrix_addr', '0x58'), 0))
            cm_dpll = getattr(args, 'clockmatrix_dpll_actuator', 3)
            actuator = ClockMatrixActuator(cm_i2c, dpll_id=cm_dpll)
            actuator_type = "clockmatrix"
            log.info("Using ClockMatrix actuator: bus=%d dpll=%d",
                     args.clockmatrix_bus, cm_dpll)
        except Exception as e:
            log.error("ClockMatrix actuator init failed: %s", e)
            actuator = None

    # Fallback to PHC adjfine — only valid when we have a PHC.
    if actuator is None:
        if not have_phc:
            log.error("TICC-only servo requires a DAC or ClockMatrix actuator "
                      "(no PHC available for adjfine fallback)")
            return 1
        from peppar_fix.phc_actuator import PhcAdjfineActuator
        actuator = PhcAdjfineActuator(ptp)
        actuator_type = "phc_adjfine"
    actuator.setup()

    # Synthesize caps for TICC-only mode from actuator limits.
    if caps is None:
        max_adj = getattr(actuator, 'max_adj_ppb', 500.0)
        caps = {'max_adj': max_adj, 'n_ext_ts': 0, 'n_pins': 0}
        log.info("TICC-only caps: max_adj=%.0f ppb (from %s)", max_adj,
                 actuator_type)

    # For ClockMatrix: set up TDC phase source.
    # Bootstrap already set the FCW and zeroed adjfine — the actuator's
    # setup() inherits the FCW value. No transfer needed.
    cm_phase_source = None
    current_adj = actuator.read_frequency_ppb()
    # Re-apply bootstrap frequency if actuator.setup() reset it.
    # DacActuator.setup() resets to center (0 ppb); the bootstrap had
    # already seeded it.  PhcAdjfineActuator has the same issue —
    # bootstrap's adjfine is lost when a new PtpDevice is opened.
    bootstrap_freq = getattr(args, '_bootstrap_freq_ppb', None)
    if bootstrap_freq is not None and abs(bootstrap_freq) > 0.1:
        actual = actuator.adjust_frequency_ppb(bootstrap_freq)
        current_adj = actual
        log.info("Restored bootstrap frequency: %.1f ppb (actual=%.1f)",
                 bootstrap_freq, actual)
    elif current_adj == 0 and abs(bootstrap_adj) > 1.0:
        # Actuator doesn't have a frequency set — use bootstrap's value.
        # This happens with PhcAdjfineActuator (normal PHC path).
        current_adj = bootstrap_adj
    if getattr(args, 'clockmatrix_bus', None) is not None:
        try:
            from peppar_fix.clockmatrix_phase import ClockMatrixPhaseSource
            cm_phase_dpll = getattr(args, 'clockmatrix_dpll_phase', 2)
            cm_pps_clk = getattr(args, 'clockmatrix_pps_clk', 2)
            cm_phase_source = ClockMatrixPhaseSource(
                cm_i2c, dpll_id=cm_phase_dpll, pps_clk=cm_pps_clk)
            cm_phase_source.setup()
            log.info("Using ClockMatrix TDC phase source: DPLL_%d, CLK%d",
                     cm_phase_dpll, cm_pps_clk)
        except Exception as e:
            log.error("ClockMatrix phase source failed: %s — using EXTTS", e)
            cm_phase_source = None
    log.info("Actuator freq at start: %.1f ppb (%s)", current_adj,
             type(actuator).__name__)
    if args.freerun:
        log.info("FREERUN MODE: DO will not be steered. "
                 "Auto-stop at |pps_error| > %.0f ns",
                 args.freerun_max_error_ns or float('inf'))
        if not args.ticc_port:
            log.warning(
                "WARNING: freerun without --ticc-port. EXTTS TDEV measurements "
                "are unreliable — both i226 and E810 have ~8 ns effective "
                "resolution that masks real timing noise. Use --ticc-port for "
                "TDEV characterization, or pair with a separate TICC capture."
            )

    # ── EXTTS setup (skipped in TICC-only mode) ──────────────────── #
    from peppar_fix.ptp_device import DualEdgeFilter

    extts_channel = args.extts_channel
    extts_ok = False
    extts_dedup = DualEdgeFilter()

    if have_phc:
        from peppar_fix.ptp_device import PTP_PF_EXTTS
        # Try ioctl pin programming first (i226), fall back to sysfs
        # (E810 ice driver rejects PTP_PIN_SETFUNC ioctl but accepts
        # sysfs writes). This makes EXTTS Just Work on both platforms.
        pin_set = False
        if args.program_pin and caps['n_pins'] > 0:
            try:
                ptp.set_pin_function(args.pps_pin, PTP_PF_EXTTS, extts_channel)
                pin_set = True
                log.info("Pin %d programmed for EXTTS channel %d via ioctl",
                         args.pps_pin, extts_channel)
            except OSError:
                log.info("Pin config ioctl not supported; trying sysfs")
        if not pin_set and caps['n_pins'] > 0 and args.pps_pin is not None:
            from phc_bootstrap import _set_pin_function_sysfs, _E810_PIN_NAMES
            pin_name = _E810_PIN_NAMES.get(args.pps_pin, str(args.pps_pin))
            if _set_pin_function_sysfs(args.servo, pin_name,
                                       PTP_PF_EXTTS, extts_channel):
                pin_set = True
                log.info("Pin %s programmed for EXTTS channel %d via sysfs",
                         pin_name, extts_channel)
        if not pin_set:
            log.info("Skipping pin programming; using implicit EXTTS mapping")
        try:
            ptp.enable_extts(extts_channel, rising_edge=True)
            log.info(f"EXTTS enabled: pin={args.pps_pin}, channel={extts_channel}")
            extts_ok = True
        except OSError as e:
            if getattr(args, 'ticc_port', None) is not None:
                log.warning("EXTTS unavailable (%s) — TICC provides servo feedback instead", e)
            else:
                log.error("EXTTS failed: %s", e)
                ptp.close()
                return 1

        # Verify PPS is actually arriving before committing to the servo loop.
        if extts_ok:
            test_pps = ptp.read_extts_dedup(extts_dedup, timeout_ms=3000)
            if test_pps is None and getattr(args, 'ticc_port', None) is None:
                log.error("No PPS event within 3s after enabling EXTTS — "
                          "check PPS wiring, pin config, and PTP device")
                ptp.disable_extts(extts_channel)
                ptp.close()
                return 3  # no PPS — wrapper should retry
            elif test_pps is None:
                log.warning("No PPS on EXTTS — TICC will provide servo feedback")
        else:
            test_pps = None
        if test_pps is not None:
            phc_sec, phc_nsec = test_pps[0], test_pps[1]
            pps_err = phc_nsec if phc_nsec < 500_000_000 else phc_nsec - 1_000_000_000
            log.info("PPS verified: phc_sec=%d error=%+d ns", phc_sec, pps_err)
        else:
            log.info("PPS verification skipped — TICC provides servo feedback")
    else:
        log.info("TICC-only: skipping EXTTS setup (no PHC)")

    # ── DOFreqEst: 4-state EKF + LQR servo ──────────────────────── #
    # Always used.  Models both oscillators (rx TCXO + DO), fuses raw
    # TICC with PPP carrier phase, and applies optimal LQR control.
    from peppar_fix.do_freq_est import DOFreqEst
    sigma_ticc = ts_params.measurement_noise_ns
    # Seed TCXO phase from bootstrap dt_rx so the filter starts in
    # full 4-state mode from epoch 1 (no mid-run transition).  Pull
    # both from state (DO state → adjfine, receiver state → dt_rx).
    bootstrap_dt_rx_ns = None
    bootstrap_base_freq = None
    do_uid_local = getattr(args, 'do_unique_id', None) or _resolve_do_uid(args)
    if do_uid_local is not None:
        try:
            from peppar_fix.do_state import load_do_state
            s = load_do_state(do_uid_local)
            if s is not None:
                bootstrap_base_freq = s.get('last_known_freq_offset_ppb')
        except Exception:
            pass
    receiver_uid_local = getattr(args, 'receiver_unique_id', None)
    if receiver_uid_local is not None:
        try:
            from peppar_fix.receiver_state import load_receiver_state
            rx = load_receiver_state(receiver_uid_local)
            if rx is not None:
                bootstrap_dt_rx_ns = rx.get('tcxo', {}).get('last_known_dt_rx_ns')
        except Exception:
            pass
    if bootstrap_dt_rx_ns is not None:
        log.info("DOFreqEst: seeding φ_tcxo from receiver-state dt_rx=%.1f ns",
                 bootstrap_dt_rx_ns)
    else:
        log.warning("DOFreqEst: no bootstrap dt_rx — running 2-state only")
    if bootstrap_base_freq is not None:
        log.info("DOFreqEst: crystal freq from DO state: %.1f ppb "
                 "(current_adj=%.1f, glide=%.1f)",
                 bootstrap_base_freq, current_adj,
                 current_adj - bootstrap_base_freq)

    # doProcessNoiseFromChar-main: override sigma_do_{phase,freq} from
    # measured DO characterization when available.  Defaults (0.92 ns,
    # args.kalman_sigma_freq) describe a generic PHC + TCXO; real OCXO
    # noise floors are 10-100× tighter, and the gap drives over-
    # aggressive servo response.  Falls back to defaults if no char or
    # extraction fails.
    sigma_do_phase_ns_eff = 0.92
    sigma_do_freq_ppb_eff = args.kalman_sigma_freq
    if do_uid_local is not None:
        try:
            from peppar_fix.do_state import derive_do_process_noise
            _ds = load_do_state(do_uid_local)
            if _ds and _ds.get('characterization'):
                derived = derive_do_process_noise(_ds['characterization'])
                if derived:
                    p = derived.get('sigma_do_phase_ns')
                    f = derived.get('sigma_do_freq_ppb')
                    if p is not None:
                        log.info("DOFreqEst: sigma_do_phase override from "
                                 "characterization: %.4f ns/√s (source=%s, "
                                 "was %.4f)",
                                 p, derived.get('sigma_do_phase_source'),
                                 sigma_do_phase_ns_eff)
                        sigma_do_phase_ns_eff = p
                    if f is not None:
                        log.info("DOFreqEst: sigma_do_freq override from "
                                 "characterization: %.4f ppb/√s (source=%s, "
                                 "was %.4f)",
                                 f, derived.get('sigma_do_freq_source'),
                                 sigma_do_freq_ppb_eff)
                        sigma_do_freq_ppb_eff = f
        except Exception as e:
            log.warning("Could not derive DO process noise from "
                        "characterization: %s", e)

    servo = DOFreqEst(
        sigma_ticc_ns=sigma_ticc,
        sigma_do_phase_ns=sigma_do_phase_ns_eff,
        sigma_do_freq_ppb=sigma_do_freq_ppb_eff,
        sigma_tcxo_phase_ns=2.0,                          # rx TCXO PPS TDEV(1s)
        sigma_tcxo_freq_ppb=args.kalman_sigma_tcxo_freq,  # rx TCXO drift rate
        max_ppb=caps['max_adj'],
        initial_freq=current_adj,
        initial_dt_rx_ns=bootstrap_dt_rx_ns,
        base_freq=bootstrap_base_freq,
    )
    log.info("DOFreqEst 4-state: sigma_ticc=%.3f ns, "
             "sigma_do=[%.4f ns, %.4f ppb], "
             "sigma_tcxo=[2.0 ns, %.3f ppb], "
             "initial_freq=%.1f ppb, base_freq=%s, tcxo_init=%s",
             sigma_ticc, sigma_do_phase_ns_eff, sigma_do_freq_ppb_eff,
             args.kalman_sigma_tcxo_freq, current_adj,
             f"{bootstrap_base_freq:.1f}" if bootstrap_base_freq else "None",
             bootstrap_dt_rx_ns is not None)
    scheduler = DisciplineScheduler(
        base_interval=args.discipline_interval,
        adaptive=args.adaptive_interval,
        min_interval=args.min_interval,
        max_interval=args.max_interval,
        converge_threshold_ns=args.scheduler_converge_threshold_ns,
        settle_window=args.scheduler_settle_window,
        unconverge_factor=args.scheduler_unconverge_factor,
    )

    qerr_alignment = {
        # Litmus 1: EXTTS PPS + qErr (matched to EXTTS epoch)
        "pps_var": RunningVarianceWindow(),
        "pps_qerr_plus_var": RunningVarianceWindow(),
        "pps_qerr_minus_var": RunningVarianceWindow(),
    }

    # PPS event queue
    pps_queue = queue.Queue(maxsize=10)
    pps_history = deque(maxlen=32)
    pps_history_lock = threading.Lock()
    stop_pps = threading.Event()
    stop_ticc = threading.Event()
    delay_injector = get_delay_injector()
    pps_recv_estimator = TimebaseRelationEstimator()
    ticc_tracker = None
    ticc_log_f = None
    ticc_log_w = None

    last_dedup_log_mono = [time.monotonic()]

    def extts_reader():
        while not stop_pps.is_set():
            event = ptp.read_extts(timeout_ms=1500)
            if event is None:
                continue
            phc_sec, phc_nsec, index, recv_mono, queue_remains, parse_age_s = event
            # Drop the falling edge of the i226 dual-edge timestamping quirk:
            # when the PPS pulse is wide (e.g. F9T default ~100 ms), the i226
            # reports both rising and falling edges and the engine would
            # otherwise see ~2 events/sec.  See ptp_device.DualEdgeFilter and
            # docs/madhat-bringup-2026-04-07.md stumble #10.
            if not extts_dedup.accept(phc_sec, phc_nsec):
                # Periodic summary so the user can see the filter working,
                # without spamming once-per-event log lines.
                now = time.monotonic()
                if now - last_dedup_log_mono[0] > 60.0:
                    log.info("EXTTS dual-edge filter: dropped %d, accepted %d "
                             "(min_spacing=%.3fs)",
                             extts_dedup.dropped, extts_dedup.accepted,
                             extts_dedup.min_spacing_s)
                    last_dedup_log_mono[0] = now
                continue
            delay_injector.maybe_inject_delay(f"ptp:{args.servo}")
            base_confidence = estimate_correlation_confidence(
                queue_remains=queue_remains,
                parse_age_s=parse_age_s,
            )
            estimator_sample = pps_recv_estimator.update(
                phc_sec + (phc_nsec / 1_000_000_000.0),
                recv_mono,
                sample_weight=estimator_sample_weight(
                    queue_remains=queue_remains,
                    base_confidence=base_confidence,
                ),
            )
            pps_event = PpsEvent(
                phc_sec=phc_sec,
                phc_nsec=phc_nsec,
                index=index,
                recv_mono=recv_mono,
                queue_remains=queue_remains,
                parse_age_s=parse_age_s,
                correlation_confidence=max(
                    0.05,
                    min(1.0, base_confidence * estimator_sample["confidence"]),
                ),
                estimator_residual_s=estimator_sample["residual_s"],
            )
            with pps_history_lock:
                pps_history.append(pps_event)
            dropped = _queue_put_drop_oldest(pps_queue, pps_event)
            if dropped:
                log.debug("Dropped one stale PPS notification due to full queue")

    # TICC is the preferred PPS correlation source (60ps, immune to igc).
    # Only start EXTTS when no TICC is configured.
    if args.ticc_port:
        t_extts = None
        log.info("TICC chB is primary PPS correlation source (EXTTS disabled)")
    elif have_phc and extts_ok:
        t_extts = threading.Thread(target=extts_reader, daemon=True)
        t_extts.start()
        log.info("EXTTS reader started (PPS source for correlation, no TICC)")
    else:
        t_extts = None
        log.info("No PPS correlation source available (no TICC, no EXTTS)")

    if args.slip_log:
        _open_slip_csv(args.slip_log)
        log.info("cycle-slip CSV → %s", args.slip_log)

    if args.ticc_port:
        ticc_tracker = TiccPairTracker(args.ticc_phc_channel, args.ticc_ref_channel)
        if args.ticc_log:
            # Append mode so wrapper-driven re-launches (exit code 5
            # recovery) preserve prior session's data.  Write header
            # only on first open / empty file.  See recoveryRetry-main.
            _ticc_log_needs_header = (
                not os.path.exists(args.ticc_log)
                or os.path.getsize(args.ticc_log) == 0
            )
            ticc_log_f = open(args.ticc_log, 'a', newline='')
            ticc_log_w = csv.writer(ticc_log_f)
            if _ticc_log_needs_header:
                ticc_log_w.writerow([
                    'host_timestamp', 'host_monotonic', 'ref_sec', 'ref_ps', 'channel'
                ])
                ticc_log_f.flush()

        qerr_ticc_tracker = QErrTimescaleTracker()

        def ticc_reader():
            # When TICC-driven, the reference channel (chB) also generates
            # PpsEvent objects for the correlation gate — replacing EXTTS.
            # We derive the GPS second from host monotonic time + the
            # realtime-to-PHC offset known from bootstrap.
            ticc_pps_estimator = TimebaseRelationEstimator()

            # Stale-TICC detector (2026-05-12).  Track consecutive
            # reopens that produced ZERO events.  Spamming "TICC reader
            # started" every 2 s without measurements means the TICC
            # opened but isn't producing data — typically because one
            # of its PPS BNC inputs (chA = DO, chB = ref) was bumped
            # or unplugged, or the TICC firmware hung.  Emit a clear
            # [TICC_STALE] warning at threshold + on each subsequent
            # cycle rather than burying the operator in repeated INFOs.
            empty_reopens = 0
            STALE_THRESHOLD_REOPENS = 5      # ≈ 10 s with 2 s reopen cadence
            last_stale_log_mono = 0.0
            STALE_LOG_INTERVAL_S = 30.0      # how often to repeat the warning

            while not stop_ticc.is_set():
                events_this_open = 0
                try:
                    with Ticc(args.ticc_port, args.ticc_baud, wait_for_boot=True) as ticc:
                        log.info("TICC reader started on %s", args.ticc_port)
                        for event in ticc.iter_events():
                            events_this_open += 1
                            if stop_ticc.is_set():
                                return
                            if ticc_log_w is not None:
                                ticc_log_w.writerow([
                                    datetime.now(tz=timezone.utc).isoformat(),
                                    f"{event.recv_mono:.9f}",
                                    event.ref_sec,
                                    event.ref_ps,
                                    event.channel,
                                ])
                                ticc_log_f.flush()
                            # Ingest first so _armed is set after boot
                            # discard period completes.
                            was_armed = ticc_tracker._armed
                            ticc_tracker.ingest(event)

                            # Match qerr to chB by TIM-TP-initiated
                            # windowing.  See docs/qerr-correlation.md
                            # for the full design and rationale.
                            # See docs/stream-timescale-correlation.md.
                            #
                            # No queue_remains gate here — the timing
                            # window (0.8-1.1s) is the freshness check
                            # for qErr correlation.  queue_remains is
                            # only needed for timescale relationship
                            # estimation (case 1 in the design).  With
                            # correctly-aligned PEROUT, chA and chB
                            # arrive nearly simultaneously so the serial
                            # buffer always has data, making
                            # queue_remains=True on every chB event.
                            if event.channel == args.ticc_ref_channel:
                                _qerr = None
                                if was_armed:
                                    pending = qerr_store.get_pending_for_chb()
                                    if pending is not None:
                                        pend_mono, pend_qerr = pending
                                        delay = event.recv_mono - pend_mono
                                        if 0.8 <= delay <= 1.1:
                                            _qerr = pend_qerr
                                            qerr_store.clear_pending()
                                ticc_tracker.set_pending_ref_qerr(
                                    event.ref_sec, _qerr)

                            # TICC chB is the primary PPS source for the
                            # correlation gate.  EXTTS is only used when
                            # no TICC is configured (t_extts is None
                            # whenever TICC is present).
                            if t_extts is None and event.channel == args.ticc_ref_channel:
                                # Derive approximate PHC second from realtime
                                # (PPS fires at the GPS second boundary; recv_mono
                                # is within ~100ms of true PPS time)
                                rt_ns = time.clock_gettime_ns(time.CLOCK_REALTIME)
                                rt_sec = rt_ns // 1_000_000_000
                                # Apply TAI offset if PHC is in TAI timescale
                                offset_s = 0
                                if hasattr(args, 'phc_timescale'):
                                    if args.phc_timescale == 'tai':
                                        offset_s = getattr(args, 'leap', 18) + getattr(args, 'tai_minus_gps', 19)
                                    elif args.phc_timescale == 'gps':
                                        offset_s = getattr(args, 'tai_minus_gps', 19)
                                phc_sec_approx = rt_sec + offset_s

                                pps_event = PpsEvent(
                                    phc_sec=phc_sec_approx,
                                    phc_nsec=0,  # PPS is at the second boundary
                                    index=-1,    # not from EXTTS
                                    recv_mono=event.recv_mono,
                                    queue_remains=event.queue_remains,
                                    parse_age_s=event.parse_age_s,
                                    correlation_confidence=event.correlation_confidence,
                                    estimator_residual_s=event.estimator_residual_s,
                                )
                                with pps_history_lock:
                                    pps_history.append(pps_event)
                                _queue_put_drop_oldest(pps_queue, pps_event)

                except Exception as exc:
                    if stop_ticc.is_set():
                        return
                    log.warning("TICC reader reconnect after error: %s", exc)
                    time.sleep(1.0)

                # ── Stale-TICC detector ────────────────────────────── #
                # We exit the `with` block either via an exception
                # (above) or via iter_events() returning.  Either way,
                # if zero events were processed during this open, the
                # TICC opened cleanly but produced nothing.  Count
                # consecutive such opens; emit a clear warning at the
                # threshold and at STALE_LOG_INTERVAL_S afterward.
                if events_this_open == 0:
                    empty_reopens += 1
                else:
                    if empty_reopens >= STALE_THRESHOLD_REOPENS:
                        log.info("[TICC_STALE] /dev/ticc%s: recovered "
                                 "after %d empty reopens — events flowing again",
                                 args.ticc_port, empty_reopens)
                    empty_reopens = 0

                now_mono = time.monotonic()
                if (empty_reopens >= STALE_THRESHOLD_REOPENS
                        and (now_mono - last_stale_log_mono)
                            >= STALE_LOG_INTERVAL_S):
                    log.warning(
                        "[TICC_STALE] %s: opened cleanly but received 0 "
                        "events on the last %d reopens.  Likely cause: PPS "
                        "BNC input(s) disconnected, TICC firmware hung, or "
                        "no PPS pulses arriving on chA/chB.  Engine will "
                        "keep retrying.  Try wiggling/reseating the BNCs, "
                        "or DTR-reset via "
                        "`python -c \"import serial; serial.Serial('%s', "
                        "115200, dsrdtr=True).close()\"` then wait 15 s.",
                        args.ticc_port, empty_reopens, args.ticc_port)
                    last_stale_log_mono = now_mono

        t_ticc = threading.Thread(target=ticc_reader, daemon=True)
        t_ticc.start()
    else:
        t_ticc = None

    # Servo log file
    log_f = None
    log_w = None
    if args.servo_log:
        log_f = open(args.servo_log, 'w', newline='')
        log_w = csv.writer(log_f)
        log_w.writerow([
            'timestamp', 'gps_second', 'phc_sec', 'phc_nsec',
            'phc_rounded_sec', 'epoch_offset_s', 'timescale_error_ns',
            'extts_index', 'pps_match_delta_s', 'pps_match_recv_dt_s', 'pps_queue_depth',
            'obs_confidence', 'obs_estimator_residual_s',
            'pps_confidence', 'pps_estimator_residual_s', 'match_confidence',
            'broadcast_confidence', 'ssr_confidence',
            'dt_rx_ns', 'dt_rx_sigma_ns', 'pps_error_ns', 'qerr_ns',
            'qerr_offset_s', 'pps_err_ticc_ns', 'ticc_age_s',
            'ticc_confidence', 'pps_var_ns2',
            'pps_qerr_plus_var_ns2', 'pps_qerr_plus_ratio',
            'pps_qerr_minus_var_ns2', 'pps_qerr_minus_ratio',
            'carrier_error_ns',
            'adjfine_ppb', 'phase', 'n_meas', 'gain_scale',
            'discipline_interval', 'n_accumulated', 'watchdog_alarm',
            'tracking_mode', 'time_to_zero_s',
            'isb_gal_ns', 'isb_bds_ns',
            'phc_gettime_ns',
            'rx_tcxo_unwrapped_ns', 'rx_tcxo_freq_ns_s', 'rx_tcxo_dt_rx_rate_discrep_ns_s',
            'rx_tcxo_phase_ns',
        ])

    return {
        'ptp': ptp,
        'phc_dev': args.servo,
        'receiver_unique_id': getattr(args, 'receiver_unique_id', None),
        'do_unique_id': _resolve_do_uid(args),
        'ts_params': ts_params,
        'actuator': actuator,
        'cm_phase_source': cm_phase_source,
        'servo': servo,
        'scheduler': scheduler,
        'qerr_store': qerr_store,
        'extint_store': extint_store,
        'bias_extint_ns': _load_timestamper_bias(args, "extint:default"),
        'bias_ticc_ns':   _load_timestamper_bias(args, "ticc_diff:default"),
        'qerr_alignment': qerr_alignment,
        'pps_queue': pps_queue,
        'pps_history': pps_history,
        'pps_history_lock': pps_history_lock,
        'stop_pps': stop_pps,
        'stop_ticc': stop_ticc,
        'ticc_tracker': ticc_tracker,
        'qerr_ticc_tracker': qerr_ticc_tracker if args.ticc_port else None,
        'ticc_log_f': ticc_log_f,
        'extts_channel': extts_channel,
        'caps': caps,
        'log_f': log_f,
        'log_w': log_w,
        'freerun': args.freerun,
        'phase': 'freerun' if args.freerun else 'tracking',
        'adjfine_ppb': current_adj,
        'gain_scale': 1.0,
        'tmode_set': False,
        'position_saved': False,
        'ppp_cal': PPPCalibration(tick_ns=8.0, min_samples=10),
        'rx_tcxo': RxTcxoTracker(freq_window=30),
        'dt_rx_buffer': deque(maxlen=30),
        'carrier_tracker': _init_carrier_tracker(args),
        'tracking_large_error_count': 0,
        'tracking_mode': 'ekf',
        'pmc': _open_pmc(args),
        'pmc_announced': None,
        'consecutive_outliers': 0,
        'last_correction_mono': time.monotonic(),
        'noise_estimator': _init_noise_estimator(args),
        'noise_buffer_writer': None,    # set below if --noise-buffer-csv
        'noise_buffer_f': None,         # file handle for clean shutdown
        'holdover': {
            'active': False,
            'reason': '',
            'entered': 0,
            'exited': 0,
            'reasons': {},
        },
    }

    # ── Optional noise-buffer CSV (I-162848 Step 1) ───────────────── #
    # Per-epoch (mono, residual_ns, channel) export from
    # InBandNoiseEstimator.feed for offline gap-PSD characterization.
    # Side-by-side comparison with freerun captures in the overlap
    # band (f > 1/gap_window) validates the detrending; PSD mismatch
    # there means feed()'s "expected drift = adjfine·dt" model is
    # missing something (DO frequency wander beyond the linear ramp).
    nb_path = getattr(args, 'noise_buffer_csv', None)
    if nb_path:
        nb_f = open(nb_path, 'w', newline='')
        nb_w = csv.writer(nb_f)
        nb_w.writerow(['mono', 'residual_ns', 'channel'])
        nb_f.flush()
        ctx['noise_buffer_writer'] = nb_w
        ctx['noise_buffer_f'] = nb_f
        log.info("Noise-buffer CSV → %s", nb_path)

    # ── TICC sanity check ─────────────────────────────────────────── #
    # When TICC is present, wait for the first differential measurement
    # and verify it's within a sane range.  A large offset (e.g., ±500 ms)
    # indicates PEROUT misalignment or PHC not bootstrapped properly.
    if ticc_tracker is not None:
        TICC_SANITY_TIMEOUT = 10  # seconds
        TICC_SANITY_MAX_NS = 100_000_000  # 100 ms — beyond this, exit for re-bootstrap
        log.info("Waiting for TICC sanity check (first differential measurement)...")
        t0 = _time.monotonic()
        first_diff = None
        while _time.monotonic() - t0 < TICC_SANITY_TIMEOUT:
            diff = ticc_tracker.latest_diff_ns()
            if diff is not None:
                first_diff = diff
                break
            _time.sleep(0.5)

        if first_diff is None:
            log.error("TICC sanity check: no differential measurement in %ds — "
                      "check TICC wiring (chA=PEROUT, chB=PPS)", TICC_SANITY_TIMEOUT)
            _cleanup_servo(ctx)
            return 3  # retry
        elif abs(first_diff) > TICC_SANITY_MAX_NS:
            log.error("TICC sanity check FAILED: diff=%+.0f ns (limit ±%.0f ns). "
                      "PEROUT may be misaligned or PHC not bootstrapped. "
                      "Exiting for re-bootstrap (exit code 5).",
                      first_diff, TICC_SANITY_MAX_NS)
            ctx = ctx  # keep reference for cleanup
            _cleanup_servo(ctx)
            return 5  # PHC diverged — wrapper will re-bootstrap
        else:
            log.info("TICC sanity check passed: diff=%+.1f ns", first_diff)


def _pps_fractional_error(phc_nsec):
    """Compute PHC error from PPS fractional second."""
    if phc_nsec <= 500_000_000:
        return float(phc_nsec)
    else:
        return float(phc_nsec) - 1_000_000_000


def _enter_obs_holdover(ctx, args, reason_code, detail):
    """Return servo to a safe state after an observation outage."""
    holdover = ctx['holdover']
    if holdover['active']:
        return
    holdover['active'] = True
    holdover['reason'] = reason_code
    holdover['entered'] += 1
    holdover['reasons'][reason_code] = holdover['reasons'].get(reason_code, 0) + 1
    last_freq = ctx['adjfine_ppb']
    log.warning(
        "Entering holdover: reason=%s detail=%s; "
        "holding PHC at last adjfine=%.1f ppb (temperature-stable assumption)",
        reason_code,
        detail,
        last_freq,
    )
    _set_clock_class(ctx, "holdover")
    # Do NOT zero adjfine — the last known frequency is almost certainly
    # better than zero.  TCXO/OCXO drift is dominated by temperature;
    # if temperature hasn't changed, the old frequency is correct.
    _purge_pps_state(ctx)
    from peppar_fix import DisciplineScheduler
    ctx['servo'].reset(last_freq)
    ctx['scheduler'] = DisciplineScheduler(
        base_interval=args.discipline_interval,
        adaptive=args.adaptive_interval,
        min_interval=args.min_interval,
        max_interval=args.max_interval,
        converge_threshold_ns=args.scheduler_converge_threshold_ns,
        settle_window=args.scheduler_settle_window,
        unconverge_factor=args.scheduler_unconverge_factor,
    )
    ctx['phase'] = 'tracking'
    ctx['tracking_large_error_count'] = 0
    ctx['tracking_mode'] = 'ekf'


def _exit_holdover(ctx, detail):
    """Leave holdover after fresh usable observations return."""
    holdover = ctx['holdover']
    if not holdover['active']:
        return
    log.info(
        "Leaving holdover: reason=%s detail=%s",
        holdover['reason'],
        detail,
    )
    holdover['active'] = False
    holdover['reason'] = ''
    holdover['exited'] += 1


def _phc_gps_offset_s(phc_sec, phc_nsec, gps_unix_sec):
    """Whole-second offset: PHC_time - GPS_time."""
    phc_rounded = phc_sec if phc_nsec < 500_000_000 else phc_sec + 1
    return phc_rounded - gps_unix_sec


def _target_timescale_sec(gps_time, args):
    """Map a RAWX GPS epoch to the PPS second it aligns with.

    RAWX rcvTow is typically ~N.997 — just before the integer second.
    The PPS edge that aligns with this epoch is second N (floor), not
    N+1 (round).  Using round() here introduces a systematic +1s error
    in the epoch_offset calculation.
    """
    gps_sec = int(gps_time.timestamp())  # floor, not round
    if args.phc_timescale == "gps":
        return gps_sec
    if args.phc_timescale == "utc":
        return gps_sec - args.leap
    if args.phc_timescale == "tai":
        return gps_sec + args.tai_minus_gps
    raise ValueError(f"Unsupported PHC timescale: {args.phc_timescale}")


def _find_pps_event_for_obs(ctx, obs_event, target_sec, timeout=0.5,
                            min_window_s=0.5, max_window_s=11.0):
    """Correlate one observation epoch against PPS history.

    Prefer PPS events whose receive-monotonic timestamp is within an acceptable
    correlation window of the observation event. Among those, choose the event
    whose rounded PHC second best matches the target timescale second.
    """
    deadline = time.monotonic() + timeout

    while True:
        result = _match_pps_event_from_history(
            ctx, obs_event, target_sec,
            min_window_s=min_window_s,
            max_window_s=max_window_s,
        )
        event, delta, recv_dt = result[0], result[1], result[2]
        if event is not None:
            return event, delta, recv_dt

        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise queue.Empty
        ctx['pps_queue'].get(timeout=remaining)


def _match_pps_event_from_history(ctx, obs_event, target_sec,
                                  min_window_s=0.5, max_window_s=11.0):
    """Return the best PPS history match for an observation, if any."""
    with ctx['pps_history_lock']:
        return match_pps_event_from_history(
            ctx['pps_history'],
            obs_event,
            target_sec,
            min_window_s=min_window_s,
            max_window_s=max_window_s,
        )


def _cm_servo_epoch(ctx, args, n_epochs, dt_rx_ns, dt_rx_sigma):
    """ClockMatrix-only servo epoch: TDC phase → PI → FCW.

    Reads phase error from the ClockMatrix TDC (DPLL PFD measuring
    PPS vs clock tree) and steers frequency via FCW. No EXTTS, no PHC,
    no PPS correlation — everything is I2C.
    """
    cm_phase = ctx['cm_phase_source']
    servo = ctx['servo']
    scheduler = ctx['scheduler']
    log_w = ctx.get('log_w')

    phase_ns = cm_phase.read_phase_ns()
    if phase_ns is None:
        if n_epochs % 10 == 0:
            log.info("  [%d] No ClockMatrix phase reading", n_epochs)
        return "no_phase"

    # The TDC PFD measures PPS vs DPLL output.
    # Positive = output behind PPS = need to speed up.
    # Use TDC as the sole error source with 50 ps confidence.
    cm_tdc_confidence_ns = 0.050

    TRACK_OUTLIER_NS = args.track_outlier_ns
    if TRACK_OUTLIER_NS is not None and abs(phase_ns) > TRACK_OUTLIER_NS:
        ctx['consecutive_outliers'] = ctx.get('consecutive_outliers', 0) + 1
        if ctx['consecutive_outliers'] >= 30:
            log.error("  %d consecutive outliers — exiting (code 5)",
                      ctx['consecutive_outliers'])
            ctx['phc_diverged'] = True
            return "outlier"
        return "outlier"
    else:
        ctx['consecutive_outliers'] = 0

    scheduler.accumulate(phase_ns, cm_tdc_confidence_ns, 'CM_TDC')

    if scheduler.should_correct():
        avg_error, avg_confidence, n_samples = scheduler.flush()

        BASE_KP = args.track_kp
        BASE_KI = args.track_ki
        GAIN_REF_SIGMA = args.gain_ref_sigma
        GAIN_MIN_SCALE = args.gain_min_scale
        GAIN_MAX_SCALE = args.gain_max_scale
        gain_scale = max(GAIN_MIN_SCALE, min(GAIN_MAX_SCALE,
                         GAIN_REF_SIGMA / avg_confidence))
        servo.kp = BASE_KP * gain_scale
        servo.ki = BASE_KI * gain_scale

        # ClockMatrix FCW: positive error → positive freq (opposite of adjfine)
        freq_ppb = servo.update(avg_error, dt=float(n_samples))

        max_ppb = args.track_max_ppb or 244_000.0
        if abs(freq_ppb) > max_ppb:
            freq_ppb = math.copysign(max_ppb, freq_ppb)
        if abs(freq_ppb) >= max_ppb * 0.95:
            servo.integral = freq_ppb / servo.ki if servo.ki != 0 else 0
            log.warning('  Anti-windup: freq=%+.0fppb at rail', freq_ppb)

        if not args.freerun:
            ctx['actuator'].adjust_frequency_ppb(freq_ppb)
        ctx['adjfine_ppb'] = freq_ppb
        ctx['gain_scale'] = gain_scale

        scheduler.update_drift_rate(time.monotonic(), freq_ppb)
        scheduler.compute_adaptive_interval(avg_confidence)

        if n_epochs % 10 == 0:
            log.info("  [%d] CM_TDC: err=%+.1fns (avg %d) freq=%+.1fppb "
                     "gain=%.2fx interval=%d",
                     n_epochs, avg_error, n_samples, freq_ppb,
                     gain_scale, scheduler.interval)

    if log_w is not None:
        log_w.writerow({
            'epoch': n_epochs,
            'source': 'CM_TDC',
            'source_error_ns': phase_ns,
            'source_confidence_ns': 0.050,
            'adjfine_ppb': ctx.get('adjfine_ppb', 0),
            'gain_scale': ctx.get('gain_scale', 1.0),
            'discipline_interval': scheduler.interval,
        })

    return "ok"


def _servo_epoch(ctx, args, filt, obs_event, corr_snapshot, n_epochs,
                 dt_rx_ns, dt_rx_sigma, n_used, known_ecef,
                 resid_rms, isb_gal_ns, isb_bds_ns, pps_match=None):
    """Process one servo epoch: read PPS, compute error, steer PHC."""

    # ClockMatrix TDC fast path: read phase directly via I2C, skip EXTTS/PPS
    cm_phase = ctx.get('cm_phase_source')
    if cm_phase is not None:
        return _cm_servo_epoch(ctx, args, n_epochs, dt_rx_ns, dt_rx_sigma)

    ptp = ctx.get('ptp')  # None in TICC-only mode
    servo = ctx['servo']
    scheduler = ctx['scheduler']
    qerr_store = ctx['qerr_store']
    qerr_alignment = ctx['qerr_alignment']
    pps_queue = ctx['pps_queue']
    ticc_tracker = ctx.get('ticc_tracker')
    log_w = ctx['log_w']
    skip_stats = ctx.get('skip_stats')

    BASE_KP = args.track_kp
    BASE_KI = args.track_ki
    GAIN_REF_SIGMA = args.gain_ref_sigma
    GAIN_MIN_SCALE = args.gain_min_scale
    GAIN_MAX_SCALE = args.gain_max_scale
    # Convergence boost disabled — bootstrap handles convergence.
    # The boost caused oscillation with gentle gains (overnight run 2026-03-25).
    CONVERGE_ERROR_NS = 1_000_000
    CONVERGE_MIN_SCALE = args.converge_min_scale
    TRACK_RESTEP_NS = args.track_restep_ns
    TRACK_OUTLIER_NS = args.track_outlier_ns

    # Once filter converges: switch F9T to timing mode
    if n_epochs >= 300 and dt_rx_sigma < 100.0:
        sigma_m = dt_rx_sigma * 1e-9 * C

        if not ctx['position_saved'] and sigma_m < 0.1:
            uid = getattr(args, 'receiver_unique_id', None)
            if uid is not None:
                save_position_to_receiver(uid, known_ecef, sigma_m, "peppar_fix")
                log.info("Position saved to receiver state (sigma=%.4fm)", sigma_m)
            ctx['position_saved'] = True

        if not ctx['tmode_set'] and sigma_m < 0.1:
            try:
                from peppar_fix.receiver import get_driver as _get_driver
                _drv = _get_driver(args.receiver)
                tmode_msg = _drv.build_tmode_fixed_msg(known_ecef)
                if tmode_msg is not None:
                    # Would need config_queue to serial_reader — skip for now
                    # The F9T timing mode can be set separately via configure_f9t.py
                    ctx['tmode_set'] = True
                    lat, lon, alt = ecef_to_lla(
                        known_ecef[0], known_ecef[1], known_ecef[2])
                    log.info(f"F9T timing mode ready "
                             f"({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
            except ImportError:
                ctx['tmode_set'] = True

    gps_time = obs_event.gps_time
    target_sec = _target_timescale_sec(gps_time, args)
    if pps_match is not None:
        pps_event, pps_match_delta_s, pps_match_recv_dt_s, _match_confidence = pps_match
    else:
        try:
            pps_event, pps_match_delta_s, pps_match_recv_dt_s = _find_pps_event_for_obs(
                ctx, obs_event, target_sec, timeout=0.5
            )
        except queue.Empty:
            if n_epochs % 10 == 0:
                log.info(f"  [{n_epochs}] No PPS event for this epoch")
            return "no_pps"

    phc_sec, phc_nsec, extts_index = pps_event
    phc_rounded_sec = phc_sec if phc_nsec < 500_000_000 else phc_sec + 1
    epoch_offset = phc_rounded_sec - target_sec
    ts_str = gps_time.strftime('%Y-%m-%d %H:%M:%S')
    pps_err_extts_ns = _pps_fractional_error(phc_nsec)
    timescale_error_ns = epoch_offset * 1_000_000_000 + pps_err_extts_ns
    pps_queue_depth = pps_queue.qsize()

    # Match qErr to this PPS edge by host monotonic time.  TIM-TP
    # arrives ~900 ms before the PPS it describes; correlating by
    # monotonic clock avoids all GPS TOW / receiver clock bias issues.
    # See docs/stream-timescale-correlation.md "TICC–qErr epoch matching"
    # for why epoch alignment is critical and how it's verified.
    # Match qErr to the EXTTS PPS edge (for EXTTS qVIR and non-TICC paths).
    # See docs/stream-timescale-correlation.md for the full correlation model.
    # qerr values are OFFSETS (corrections), not timestamps.  Each is
    # matched to a specific PPS edge via CLOCK_MONOTONIC.  After adding
    # qerr to a timestamp, the result is still on the timestamp's
    # timescale (EXTTS/PHC or TICC) — the qerr just removes the F9T
    # PPS quantization from the reference edge.
    #
    # qerr_for_extts_pps_ns: matched to the EXTTS PPS edge (pps_event.recv_mono)
    # qerr_for_ticc_pps_ns:  matched to the TICC PPS edge (ticc_measurement.recv_mono)
    qerr_for_extts_pps_ns, qerr_offset_s = qerr_store.match_pps_mono(pps_event.recv_mono)
    qerr_for_ticc_pps_ns = None
    pps_err_ticc_ns = None
    ticc_age_s = None
    ticc_confidence = None
    if ticc_tracker is not None:
        ticc_measurement = ticc_tracker.latest(time.monotonic(), args.ticc_max_age_s)
        if ticc_measurement is not None:
            # PPS error from TICC, measured on TICC timescale.
            # diff_ns = chA-chB = do_pps-gnss_pps (positive = DO PPS late).
            # Negate so that pps_err_ticc_ns has the same sign as the
            # DOFreqEst measurement model: negative when DO is late.
            #
            # Auto-capture: on the first valid TICC measurement, set the
            # target to the current differential.  This zeros the initial
            # offset (cable delay, ARM alignment) so the servo starts at
            # ~0 error and tracks drift from there.  Only used when
            # --ticc-target-auto is explicitly set; default behaviour
            # is to actively pull chA-chB to zero (= PEROUT edge at
            # local TICC chA aligned with F9T PPS at TICC chB, which
            # is GPS top-of-second modulo F9T qErr).  See
            # docs/peroutVS-phc-time.md for what "GPS-aligned" means
            # in this system: it's the rising edge of PEROUT as seen
            # by the local TICC chA, not the PHC's internal counter
            # (those differ by a per-host PHC→PEROUT→cable delay).
            if (getattr(args, 'ticc_target_auto', False)
                    and ctx.get('ticc_target_auto') is None
                    and args.ticc_target_ns == 0.0):
                args.ticc_target_ns = ticc_measurement.diff_ns
                ctx['ticc_target_auto'] = ticc_measurement.diff_ns
                log.info("TICC target auto-captured: %.1f ns (initial chA-chB)",
                         args.ticc_target_ns)
            pps_err_ticc_ns = -(ticc_measurement.diff_ns - args.ticc_target_ns)
            # Sanity: if the TICC diff is larger than 100 ms, PEROUT is
            # grossly misaligned (e.g., 500 ms offset).  Log loudly and
            # do NOT silently use this as a servo input.
            if abs(pps_err_ticc_ns) > 100_000_000:
                log.error("TICC diff = %+.0f ns — PEROUT is misaligned "
                          "(raw diff_ns=%+.0f). NOT using as servo input.",
                          pps_err_ticc_ns, ticc_measurement.diff_ns)
                pps_err_ticc_ns = None
            ticc_age_s = max(0.0, time.monotonic() - ticc_measurement.recv_mono)
            ticc_confidence = ticc_measurement.confidence
            # qerr for this TICC measurement was matched at chB arrival
            # time in the ticc_reader thread — deterministic, no race
            # with latest().  See docs/stream-timescale-correlation.md.
            qerr_for_ticc_pps_ns = ticc_measurement.ref_qerr_ns
    if qerr_for_extts_pps_ns is None and n_epochs % 10 == 0:
        log.info("  [%s] qErr match miss (mono)", n_epochs)
    elif qerr_for_extts_pps_ns is not None and n_epochs % 10 == 0:
        log.info(
            "  [%s] qErr match ok: extts_offset=%.3fs extts_qerr=%+.1fns"
            "  ticc_qerr=%s",
            n_epochs,
            qerr_offset_s if qerr_offset_s is not None else -1.0,
            qerr_for_extts_pps_ns,
            f"{qerr_for_ticc_pps_ns:+.1f}ns" if qerr_for_ticc_pps_ns is not None else "None",
        )
    # ── rx_tcxo: unwrap phase and cross-validate against dt_rx ──
    rx_tcxo = ctx['rx_tcxo']
    _rx_tcxo_discrep = None
    _rx_tcxo_freq = None
    _rx_tcxo_phase = None
    # Frequency-aided unwrap: predict next qErr from dt_rx rate
    # (PPP carrier-phase has no 8-ns sawtooth ambiguity).  Without
    # this, F9T rx TCXOs at 100s of ppb defeat the half-tick
    # threshold and the unwrap goes wrong.
    # See docs/dofreq-est-measurement-ladder.md §"Arm 2".
    predicted_drift = None
    if dt_rx_ns is not None and dt_rx_sigma is not None and dt_rx_sigma < 2.0:
        prev_dt_rx = ctx.get('prev_dt_rx_for_unwrap')
        if prev_dt_rx is not None:
            # Single-sample dt_rx rate: sigma ~0.14 ns/s, well within
            # the 4-ns/s ambiguity bound.  No smoothing needed.
            predicted_drift = dt_rx_ns - prev_dt_rx
        ctx['prev_dt_rx_for_unwrap'] = dt_rx_ns
    # Drain raw TIM-TP stream and feed sample-by-sample (I-151540
    # Item 7).  Each TIM-TP message has its own host_time timestamp;
    # variable-cadence handled correctly by the tracker's t_s-aware
    # slope LS.  Replaces the legacy per-PPS-edge feed via
    # match_pps_mono(qerr_for_*_pps_ns), which was vulnerable to
    # FIFO sequence breaks across PPS gaps (the failure mode behind
    # qVIR=0 cascades on PiFace post-antenna-swap).
    qerr_store = ctx.get('qerr_store')
    n_drained = 0
    unwrapped_ns = rx_tcxo.accumulated_phase_ns()
    if qerr_store is not None:
        n_drained, unwrapped_ns = rx_tcxo.update_from_qerr_store(
            qerr_store, predicted_drift_ns_per_s=predicted_drift)
    if n_drained > 0 or rx_tcxo.n_samples > 0:
        _rx_tcxo_freq, freq_n = rx_tcxo.freq_ns_per_s()
        # Synthesize phase: dt_rx resolves tick, qErr gives sub-tick.
        # phase_ns retains its per-edge interface — synthesized phase
        # at the current epoch is computed from the last matched qErr,
        # not from the raw stream (different consumer; see Bravo's
        # I-151540 review note for the multiple-consumers finding).
        _qerr_for_synth = (qerr_for_ticc_pps_ns
                            if qerr_for_ticc_pps_ns is not None
                            else qerr_for_extts_pps_ns)
        if (dt_rx_ns is not None and dt_rx_sigma is not None
                and dt_rx_sigma < 2.0 and _qerr_for_synth is not None):
            _rx_tcxo_phase = rx_tcxo.phase_ns(dt_rx_ns, _qerr_for_synth)
            discrep, cal = rx_tcxo.cross_validate_dt_rx(dt_rx_ns)
            if cal:
                _rx_tcxo_discrep = discrep
            if n_epochs % 30 == 0:
                synth_str = f"synth={_rx_tcxo_phase:.1f}ns " if _rx_tcxo_phase is not None else ""
                if cal and discrep is not None:
                    log.info("  [%d] rx_tcxo: unwrapped=%.1f ns, "
                             "%sfreq=%.3f ns/s (%d samples, %d new), "
                             "dt_rx rate discrepancy=%+.2f ns/s",
                             n_epochs, unwrapped_ns, synth_str,
                             _rx_tcxo_freq if _rx_tcxo_freq is not None else 0.0,
                             freq_n, n_drained, discrep)
                elif not cal:
                    log.info("  [%d] rx_tcxo: unwrapped=%.1f ns, "
                             "%sfreq=%.3f ns/s (%d samples, %d new), calibrating",
                             n_epochs, unwrapped_ns, synth_str,
                             _rx_tcxo_freq if _rx_tcxo_freq is not None else 0.0,
                             freq_n, n_drained)

    # ── Litmus test 1: EXTTS PPS + qErr ──
    # Uses qerr_for_extts_pps_ns matched to the EXTTS PPS epoch.
    cum_adj = qerr_alignment.get("cumulative_adjfine_ns", 0.0)
    rate_compensated = pps_err_extts_ns - cum_adj
    qerr_alignment["cumulative_adjfine_ns"] = cum_adj + ctx['adjfine_ppb']
    qerr_alignment["pps_var"].add(rate_compensated)
    if qerr_for_extts_pps_ns is not None:
        qerr_alignment["pps_qerr_plus_var"].add(rate_compensated + qerr_for_extts_pps_ns)
        qerr_alignment["pps_qerr_minus_var"].add(rate_compensated - qerr_for_extts_pps_ns)
    # Carrier phase tracker: auto-init and accumulate adjfine
    carrier_tracker = ctx.get('carrier_tracker')
    if carrier_tracker is not None and not getattr(args, 'no_carrier', False):
        if dt_rx_ns is not None and dt_rx_sigma is not None:
            just_initialized = (not carrier_tracker.initialized
                                and carrier_tracker.try_auto_init(dt_rx_ns))
            if just_initialized:
                # Anchor the Carrier zero-point to PPS truth.  Without
                # this, the servo drives carrier_error to zero but
                # carries a hidden bias relative to pps_error.
                carrier_tracker.anchor_to_pps(pps_err_extts_ns, dt_rx_ns)
                if carrier_tracker._anchored:
                    log.info("Carrier tracker: anchored to PPS "
                             "(offset=%+.1f ns)",
                             carrier_tracker.phase_anchor_ns)
        if carrier_tracker.initialized:
            carrier_tracker.accumulate_adjfine(ctx['adjfine_ppb'])
            carrier_tracker.update_drift_estimate(
                dt_rx_ns, pps_err_extts_ns, ctx['adjfine_ppb'])

    # Time-differenced dt_rx tracking (for future 4-state filter).
    # The 2-state Kalman's frequency state conflates f_phc_drift and
    # f_tcxo, so Δdt_rx (which observes f_tcxo alone) can't be injected
    # as a frequency measurement without breaking the state semantics.
    # The 4-state DOFreqEst filter (see architecture-vision.md) will
    # properly separate these.  For now, just track Δdt_rx for logging.
    _delta_dt_rx_ns = None
    if dt_rx_ns is not None:
        prev_dt_rx = ctx.get('_prev_dt_rx_ns')
        if prev_dt_rx is not None:
            delta = dt_rx_ns - prev_dt_rx
            if abs(delta) < 5000.0:
                _delta_dt_rx_ns = delta
        ctx['_prev_dt_rx_ns'] = dt_rx_ns

    pps_var_ns2 = qerr_alignment["pps_var"].diff_variance()
    pps_qerr_plus_var_ns2 = qerr_alignment["pps_qerr_plus_var"].diff_variance()
    pps_qerr_minus_var_ns2 = qerr_alignment["pps_qerr_minus_var"].diff_variance()
    qerr_plus_ratio = None
    qerr_minus_ratio = None
    if (
        pps_var_ns2 is not None and
        pps_qerr_plus_var_ns2 is not None and
        pps_qerr_plus_var_ns2 > 0.0
    ):
        qerr_plus_ratio = pps_var_ns2 / pps_qerr_plus_var_ns2
    if (
        pps_var_ns2 is not None and
        pps_qerr_minus_var_ns2 is not None and
        pps_qerr_minus_var_ns2 > 0.0
    ):
        qerr_minus_ratio = pps_var_ns2 / pps_qerr_minus_var_ns2
    if n_epochs % 10 == 0:
        if qerr_plus_ratio is not None and qerr_alignment["pps_qerr_plus_var"].count() >= 8:
            label = ("good" if qerr_plus_ratio >= 1.5
                     else "ok" if qerr_plus_ratio >= 1.0
                     else "BAD")
            lvl = log.info if qerr_plus_ratio >= 1.0 else log.warning
            lvl("  [%s] EXTTS qVIR: Δvar(pps)/Δvar(pps+qErr) = %.2f (%s)",
                n_epochs, qerr_plus_ratio, label)

    # Feed PPP calibration: compare PPP-derived qerr against TIM-TP
    # qErr for the first ~10 epochs to determine the constant offset.
    ppp_cal = ctx['ppp_cal']
    if (not ppp_cal.calibrated and qerr_for_extts_pps_ns is not None
            and dt_rx_sigma is not None and dt_rx_sigma < args.carrier_max_sigma_ns):
        done = ppp_cal.add_sample(dt_rx_ns, qerr_for_extts_pps_ns)
        if done:
            log.info(f"  PPP calibration done: offset={ppp_cal.offset_ns:+.3f}ns "
                     f"({ppp_cal._n} samples)")

    # No warmup or step phases — PHC bootstrap handles phase and frequency.
    # PI tracking from epoch 1.

    # Let the scheduler adapt its interval naturally.  DOFreqEst receives
    # the actual elapsed wall-clock time (not n_samples) as dt, so it
    # handles any correction interval correctly.  Longer intervals reduce
    # actuation noise — the OCXO's superior short-term stability should
    # shine through unmolested between corrections.
    mode_time_to_zero_s = None
    mode_gain_floor = None

    # Outlier check on the raw observed phase error.  Prefer the TICC
    # chA-chB diff when present — it's the direct "DO PPS vs GNSS PPS"
    # measurement, valid in both PHC mode (chA driven by PEROUT) and
    # TICC-only mode (chA driven by DAC-steered OCXO).  Fall back to
    # EXTTS PHC fractional offset on hosts without a TICC.  Don't use
    # the EKF state x[2] — it lags by the filter's response and can
    # hide a sustained step.  Don't use pps_err_extts_ns alone — on
    # TICC-only hosts phc_nsec is synthesized as 0 so the EXTTS signal
    # is identically 0 and the outlier detector silently disables.
    outlier_observable_ns = (
        pps_err_ticc_ns if pps_err_ticc_ns is not None
        else pps_err_extts_ns
    )
    if (
        TRACK_OUTLIER_NS is not None and
        abs(outlier_observable_ns) > TRACK_OUTLIER_NS and
        not scheduler._converging
    ):
        ctx['consecutive_outliers'] += 1
        if ctx['consecutive_outliers'] >= 30:
            log.error("  %d consecutive outliers — servo has lost control. "
                      "Exiting for re-bootstrap (exit code 5).",
                      ctx['consecutive_outliers'])
            _dfe = ctx.get('dfe_sm')
            if _dfe is not None:
                _dfe.transition(DOFreqEstState.HOLDOVER,
                                "30 consecutive outliers")
            ctx['phc_diverged'] = True
            return "outlier"
        log.warning(f"  Outlier: pps_err={outlier_observable_ns:+.0f}ns, "
                    f"skipping ({ctx['consecutive_outliers']}/30)")
        return "outlier"
    else:
        ctx['consecutive_outliers'] = 0

    # Scheduler input: |x[2]| is the EKF's DO-phase-vs-GPS-time estimate,
    # the direct one-to-one replacement for the old source-mux output.
    # σ comes from sqrt(P[2,2]) — the filter's own uncertainty in x[2].
    scheduler.accumulate(
        abs(servo.estimated_phase_ns),
        max(servo.phase_uncertainty_ns, 0.001),
        'EKF',
    )

    # Feed in-band noise estimator (before correction decision)
    noise_est = ctx.get('noise_estimator')
    if noise_est is not None:
        # will_correct is a lookahead — True if scheduler will flush this epoch
        will_correct = scheduler.should_correct()
        noise_est.feed(pps_err_extts_ns, ctx['adjfine_ppb'], will_correct)
        # Optional CSV export of per-epoch (mono, residual_ns, channel)
        # samples for offline gap-PSD analysis (I-162848 Step 1).
        # Detrending math lives inside feed(); here we just persist
        # the freshly-computed residual + channel label.
        nb_writer = ctx.get('noise_buffer_writer')
        if nb_writer is not None and noise_est.last_channel is not None:
            nb_writer.writerow([
                f"{noise_est.last_mono:.9f}",
                f"{noise_est.last_residual_ns:.6f}",
                noise_est.last_channel,
            ])

    # Three-stage clockClass promotion: 248 (boot) → 52 (PHC bootstrapped,
    # set by wrapper after bootstrap) → 6 (servo settled).  Demote back
    # to 52 if the scheduler leaves settled state.
    # In freerun mode, clockClass stays at 248 — PHC is not disciplined.
    if not args.freerun:
        if not scheduler._converging:
            _set_clock_class(ctx, "locked")
        else:
            _set_clock_class(ctx, "initialized")

    if TRACK_RESTEP_NS is not None and not args.freerun:
        # Use pps_err_extts_ns (raw PHC fractional offset) for the restep
        # check.  After a step, dt_rx is stale and large while the filter
        # reconverges — checking it would cause spurious resteps.
        if abs(pps_err_extts_ns) >= TRACK_RESTEP_NS:
            ctx['tracking_large_error_count'] += 1
        else:
            ctx['tracking_large_error_count'] = 0
        if ctx['tracking_large_error_count'] >= 3:
            log.error(
                "PHC error above %.0fns for %d consecutive epochs — "
                "exiting for PHC re-bootstrap (exit code 5)",
                TRACK_RESTEP_NS,
                ctx['tracking_large_error_count'],
            )
            ctx['phc_diverged'] = True

    # Freerun auto-stop: exit when PPS error grows too large for
    # the correlation gate to work reliably.
    if args.freerun and args.freerun_max_error_ns is not None:
        if abs(pps_err_extts_ns) >= args.freerun_max_error_ns:
            log.info(
                "Freerun auto-stop: |pps_error|=%.0fns exceeds %.0fns threshold",
                abs(pps_err_extts_ns), args.freerun_max_error_ns,
            )
            ctx['phc_diverged'] = True

    # TODO(ta-e744, ta-7j06): Re-enable timescale restep once the step
    # source is GNSS-derived (not system clock).  The PI servo tracks
    # frequency well from any starting phase; absolute phase alignment
    # requires a reliable step source.

    # Post-step cooldown: skip frequency corrections while the filter
    # reconverges.  Without this, stale dt_rx drives the servo to
    # over-correct, undoing the step.
    cooldown = ctx.get('post_step_cooldown', 0)
    if cooldown > 0:
        ctx['post_step_cooldown'] = cooldown - 1
        scheduler.flush()  # drain accumulated samples
        return "cooldown"

    if scheduler.should_correct():
        avg_error, avg_confidence, n_samples = scheduler.flush()

        gain_scale = max(GAIN_MIN_SCALE, min(GAIN_MAX_SCALE,
                         GAIN_REF_SIGMA / avg_confidence))
        if abs(avg_error) > CONVERGE_ERROR_NS:
            gain_scale = max(gain_scale, CONVERGE_MIN_SCALE)
        if mode_gain_floor is not None:
            gain_scale = max(gain_scale, mode_gain_floor)

        # gnss-phase-experiment: four-arm Kalman fusion in DOFreqEst.
        # See docs/dofreq-est-measurement-ladder.md.  Each arm gated on
        # availability inside servo.update; predict step always runs.
        #
        # Wired today: Arm 1 (PPP), Arm 3 (TIM-TM2 → x[2]),
        #              Arm 4 (TICC chA-chB → couple x[0], x[2]).
        # Pending:    Arm 2 (qErr-as-frequency).
        extint_phase_ns = None
        extint_sigma_ns = None
        _ext = ctx.get('extint_store')
        if _ext is not None:
            sample = _ext.consume_latest()
            if sample is not None:
                extint_phase_ns, _acc = sample
                # accEst is reported as integer ns by the F9T;
                # treat it as the 1-σ for the Kalman update.
                extint_sigma_ns = float(max(_acc, 1))
        # Arm 4: pps_err_ticc_ns is already negated upstream so the
        # sign matches DOFreqEst's z_ticc = -x[2] - qerr(x[0]) model.
        # ticc_confidence is per-measurement σ in ns from the TICC
        # tracker.  --no-ticc gates the arm off for ablation.
        ticc_diff_ns = None
        ticc_sigma_ns = None
        if (not getattr(args, 'no_ticc', False)
                and pps_err_ticc_ns is not None
                and ticc_confidence is not None):
            ticc_diff_ns = pps_err_ticc_ns
            ticc_sigma_ns = float(max(ticc_confidence, 0.001))
        # Static-bias correction per timestamper: subtract the
        # operator-calibrated offset before the EKF sees the
        # measurement.  Calibrated values come from
        # state/dos/<do_uid>.json (see scripts/calibrate_timestampers.py
        # and docs/dofreq-est-measurement-ladder.md).  When no
        # calibration has been recorded the bias entries are None and
        # this is a no-op.  This keeps every arm pointed at the same
        # physical x[2] = 0 so ablation runs converge to the same
        # truth and P[2,2] is a real σ-vs-reference.
        _bias_ext = ctx.get('bias_extint_ns')
        if extint_phase_ns is not None and _bias_ext is not None:
            extint_phase_ns -= _bias_ext
        _bias_ticc = ctx.get('bias_ticc_ns')
        if ticc_diff_ns is not None and _bias_ticc is not None:
            ticc_diff_ns -= _bias_ticc
        now_mono = time.monotonic()
        dt_actual = now_mono - ctx['last_correction_mono']
        ctx['last_correction_mono'] = now_mono
        # ── Arm 2: rx TCXO frequency from unwrapped qErr slope ──
        # Skipped if --no-qerr-arm or if buffer not yet full enough.
        # slope is in ns/s = ppb of fractional frequency offset.
        qerr_freq_ppb = None
        qerr_freq_sigma_ppb = None
        if not getattr(args, 'no_qerr_arm', False):
            _slope, _sigma, _n = ctx['rx_tcxo'].freq_ppb_with_sigma()
            if _slope is not None and _sigma is not None and _n >= 10:
                qerr_freq_ppb = _slope
                # Floor on Arm 2 sigma.  Default 1.0 ppb — accounts for
                # rx_TCXO frequency wander within the slope window (real
                # F9T TCXOs wander 5-8 ppb on minute-scale timescales, so
                # OLS slope sigma assuming white-phase qErr noise — which
                # would be ~50 fppb — undersells the actual uncertainty
                # by 1000x).  Tunable via --qerr-freq-sigma-floor for
                # experimentation.
                qerr_freq_sigma_ppb = max(_sigma, args.qerr_freq_sigma_floor)
        adjfine_ppb = -servo.update(
            dt=dt_actual,
            dt_rx_ns=dt_rx_ns, dt_rx_sigma_ns=dt_rx_sigma,
            qerr_freq_ppb=qerr_freq_ppb, qerr_freq_sigma_ppb=qerr_freq_sigma_ppb,
            extint_phase_ns=extint_phase_ns,
            extint_sigma_ns=extint_sigma_ns,
            ticc_diff_ns=ticc_diff_ns,
            ticc_sigma_ns=ticc_sigma_ns,
        )
        max_track_ppb = min(
            ctx['caps']['max_adj'],
            args.track_max_ppb if args.track_max_ppb is not None else ctx['caps']['max_adj'],
        )
        if abs(adjfine_ppb) > max_track_ppb:
            adjfine_ppb = math.copysign(max_track_ppb, adjfine_ppb)
        if not args.freerun:
            ctx['actuator'].adjust_frequency_ppb(adjfine_ppb)
        ctx['adjfine_ppb'] = adjfine_ppb
        ctx['gain_scale'] = gain_scale
        # Update DOFreqEst state machine metrics for [STATUS] line
        _dfe = ctx.get('dfe_sm')
        if _dfe is not None:
            _dfe.update_metrics(adj_ppb=adjfine_ppb, err_ns=avg_error,
                                interval=scheduler.interval)

        scheduler.update_drift_rate(time.monotonic(), adjfine_ppb)
        scheduler.compute_adaptive_interval(avg_confidence)

        if n_epochs % 10 == 0:
            log.info(f"  [{n_epochs}] EKF: "
                     f"err={avg_error:+.1f}ns (avg {n_samples}) "
                     f"adj={adjfine_ppb:+.1f}ppb "
                     f"gain={gain_scale:.2f}x "
                     f"interval={scheduler.interval}")
    else:
        if n_epochs % 10 == 0:
            log.info(f"  [{n_epochs}] EKF: "
                     f"err={servo.estimated_phase_ns:+.1f}ns "
                     f"coast ({scheduler.n_accumulated}/{scheduler.interval}) "
                     f"adj={ctx['adjfine_ppb']:+.1f}ppb")

    # Log noise estimator summary periodically
    if noise_est is not None and n_epochs % 120 == 0 and noise_est.gap_samples >= 10:
        log.info("  [noise] %s", noise_est.summary())

    # PHC time at PPS edge (from EXTTS hardware timestamp).
    phc_gettime_ns = phc_sec * 1_000_000_000 + phc_nsec
    carrier_error_ns = None
    if carrier_tracker is not None and carrier_tracker.initialized:
        carrier_error_ns = carrier_tracker.compute_error(dt_rx_ns)
    _log_servo(log_w, ctx['log_f'], ts_str, target_sec, phc_sec, phc_nsec,
               phc_rounded_sec, epoch_offset, timescale_error_ns,
               extts_index, pps_match_delta_s, pps_match_recv_dt_s, pps_queue_depth,
               obs_event, pps_event, _match_confidence, corr_snapshot,
               dt_rx_ns, dt_rx_sigma, pps_err_extts_ns, qerr_for_extts_pps_ns, qerr_offset_s,
               pps_err_ticc_ns, ticc_age_s, ticc_confidence,
               pps_var_ns2, pps_qerr_plus_var_ns2, qerr_plus_ratio,
               pps_qerr_minus_var_ns2, qerr_minus_ratio,
               carrier_error_ns,
               ctx['adjfine_ppb'], ctx['phase'], n_used,
               ctx['gain_scale'], scheduler, isb_gal_ns, isb_bds_ns,
               ctx.get('tracking_mode'), mode_time_to_zero_s,
               phc_gettime_ns,
               rx_tcxo_unwrapped_ns=rx_tcxo.accumulated_phase_ns() if rx_tcxo.n_samples > 0 else None,
               rx_tcxo_freq_ns_s=_rx_tcxo_freq,
               rx_tcxo_discrep_ns_s=_rx_tcxo_discrep,
               rx_tcxo_phase_ns=_rx_tcxo_phase)
    return "logged"


def _log_servo(log_w, log_f, ts_str, gps_unix_sec, phc_sec, phc_nsec,
               phc_rounded_sec, epoch_offset_s, timescale_error_ns,
               extts_index, pps_match_delta_s, pps_match_recv_dt_s, pps_queue_depth,
               obs_event, pps_event, match_confidence, corr_snapshot,
               dt_rx_ns, dt_rx_sigma, pps_err_extts_ns, qerr_for_extts_pps_ns, qerr_offset_s,
               pps_err_ticc_ns, ticc_age_s, ticc_confidence,
               pps_var_ns2, pps_qerr_plus_var_ns2, qerr_plus_ratio,
               pps_qerr_minus_var_ns2, qerr_minus_ratio,
               carrier_error_ns,
               adjfine_ppb, phase, n_used, gain_scale, scheduler,
               isb_gal_ns, isb_bds_ns, tracking_mode, time_to_zero_s,
               phc_gettime_ns=None,
               rx_tcxo_unwrapped_ns=None, rx_tcxo_freq_ns_s=None,
               rx_tcxo_discrep_ns_s=None, rx_tcxo_phase_ns=None):
    """Write one servo log row."""
    if log_w is None:
        return
    obs_confidence = getattr(obs_event, 'correlation_confidence', None)
    obs_residual_s = getattr(obs_event, 'estimator_residual_s', None)
    pps_confidence = getattr(pps_event, 'correlation_confidence', None)
    pps_residual_s = getattr(pps_event, 'estimator_residual_s', None)
    broadcast_confidence = None
    ssr_confidence = None
    if corr_snapshot is not None:
        broadcast_confidence = corr_snapshot.get('broadcast_confidence')
        ssr_confidence = corr_snapshot.get('ssr_confidence')
    log_w.writerow([
        ts_str, gps_unix_sec, phc_sec, phc_nsec,
        phc_rounded_sec, epoch_offset_s, f'{timescale_error_ns:.1f}',
        extts_index, pps_match_delta_s,
        f'{pps_match_recv_dt_s:.3f}', pps_queue_depth,
        f'{obs_confidence:.3f}' if obs_confidence is not None else '',
        f'{obs_residual_s:.6f}' if obs_residual_s is not None else '',
        f'{pps_confidence:.3f}' if pps_confidence is not None else '',
        f'{pps_residual_s:.6f}' if pps_residual_s is not None else '',
        f'{match_confidence:.3f}' if match_confidence is not None else '',
        f'{broadcast_confidence:.3f}' if broadcast_confidence is not None else '',
        f'{ssr_confidence:.3f}' if ssr_confidence is not None else '',
        f'{dt_rx_ns:.3f}', f'{dt_rx_sigma:.3f}',
        f'{pps_err_extts_ns:.1f}', f'{qerr_for_extts_pps_ns:.3f}' if qerr_for_extts_pps_ns is not None else '',
        f'{qerr_offset_s:.3f}' if qerr_offset_s is not None else '',
        f'{pps_err_ticc_ns:.3f}' if pps_err_ticc_ns is not None else '',
        f'{ticc_age_s:.3f}' if ticc_age_s is not None else '',
        f'{ticc_confidence:.3f}' if ticc_confidence is not None else '',
        f'{pps_var_ns2:.3f}' if pps_var_ns2 is not None else '',
        f'{pps_qerr_plus_var_ns2:.3f}' if pps_qerr_plus_var_ns2 is not None else '',
        f'{qerr_plus_ratio:.3f}' if qerr_plus_ratio is not None else '',
        f'{pps_qerr_minus_var_ns2:.3f}' if pps_qerr_minus_var_ns2 is not None else '',
        f'{qerr_minus_ratio:.3f}' if qerr_minus_ratio is not None else '',
        f'{carrier_error_ns:.3f}' if carrier_error_ns is not None else '',
        f'{adjfine_ppb:.3f}', phase, n_used, f'{gain_scale:.3f}',
        scheduler.interval, scheduler.n_accumulated, 0,
        tracking_mode or '',
        f'{time_to_zero_s:.3f}' if time_to_zero_s is not None else '',
        f'{isb_gal_ns:.3f}', f'{isb_bds_ns:.3f}',
        str(phc_gettime_ns) if phc_gettime_ns is not None else '',
        f'{rx_tcxo_unwrapped_ns:.3f}' if rx_tcxo_unwrapped_ns is not None else '',
        f'{rx_tcxo_freq_ns_s:.3f}' if rx_tcxo_freq_ns_s is not None else '',
        f'{rx_tcxo_discrep_ns_s:.1f}' if rx_tcxo_discrep_ns_s is not None else '',
        f'{rx_tcxo_phase_ns:.3f}' if rx_tcxo_phase_ns is not None else '',
    ])
    if log_f is not None:
        log_f.flush()


def _cleanup_servo(ctx):
    """Clean up servo resources."""
    # Degrade clockClass to 248 (freerun) on engine exit
    _set_clock_class(ctx, "freerun")
    ctx['stop_pps'].set()
    if 'stop_ticc' in ctx:
        ctx['stop_ticc'].set()
    if not ctx.get('freerun'):
        try:
            ctx['actuator'].adjust_frequency_ppb(0.0)
        except Exception:
            pass
    try:
        ctx['actuator'].teardown()
    except Exception:
        pass
    cm_phase = ctx.get('cm_phase_source')
    if cm_phase is not None:
        try:
            cm_phase.teardown()
        except Exception:
            pass
    ptp = ctx.get('ptp')
    if ptp is not None:
        try:
            ptp.disable_extts(ctx['extts_channel'])
        except OSError:
            pass  # EXTTS may not have been enabled (TICC-only mode)
        ptp.close()
    if ctx['log_f']:
        ctx['log_f'].close()
    if ctx.get('ticc_log_f'):
        ctx['ticc_log_f'].close()
    if ctx.get('noise_buffer_f'):
        try:
            ctx['noise_buffer_f'].flush()
            ctx['noise_buffer_f'].close()
        except Exception:
            pass
    pmc = ctx.get('pmc')
    if pmc is not None:
        pmc.close()
    # Only save state on clean exit with sane adjfine.  Failed runs
    # (railed actuator, diverged EKF) should NOT poison the drift file
    # or DO state — the next run's bootstrap would inherit the bad value.
    adjfine = ctx.get('adjfine_ppb', 0.0)
    max_adj = ctx.get('caps', {}).get('max_adj', 62_500_000)
    adjfine_sane = abs(adjfine) < max_adj * 0.90
    if adjfine_sane:
        noise_est = ctx.get('noise_estimator')
        if noise_est is not None:
            do_uid = ctx.get('do_unique_id')
            if do_uid is not None:
                from peppar_fix.noise_estimator import save_noise_state
                save_noise_state(do_uid, noise_est)
        _save_osc_freq_corr(ctx)
    else:
        log.warning("Skipping state save: adjfine=%+.1f ppb is near rail "
                    "(max=%+.0f) — likely diverged", adjfine, max_adj)
    log.info("PHC servo cleaned up")


def _purge_pps_state(ctx):
    """Drop PPS events captured before a PHC step.

    Historical EXTS events are invalid after stepping the PHC because they were
    timestamped on the old PHC timescale.
    """
    with ctx['pps_history_lock']:
        ctx['pps_history'].clear()
    while True:
        try:
            ctx['pps_queue'].get_nowait()
        except queue.Empty:
            break


def _locked_len(collection, lock):
    """Return len(collection) while holding lock, or 0 if either is None."""
    if collection is None or lock is None:
        return 0
    with lock:
        return len(collection)


def _check_queue_depths(queue_hwm, alert_armed, threshold, dump,
                        obs_queue, obs_history,
                        pps_history=None, pps_history_lock=None,
                        skip_stats=None, gate=None):
    """Update queue high-water marks and fire threshold alerts.

    High-water marks track the session maximum depth for each queue.
    When any queue's current depth reaches *threshold*, a warning is
    logged (with optional diagnostic dump via *dump*).  The alert
    re-arms once the queue drains below half the threshold.
    """
    now_mono = time.monotonic()
    depths = {
        "obs_queue": obs_queue.qsize(),
        "obs_history": len(obs_history),
        "pps_history": _locked_len(pps_history, pps_history_lock),
    }
    for name, depth in depths.items():
        if depth > queue_hwm[name]:
            queue_hwm[name] = depth
        if depth >= threshold and alert_armed[name]:
            alert_armed[name] = False
            # Age of oldest item (seconds behind real-time)
            oldest_age_s = None
            if name == "obs_history" and obs_history:
                oldest_age_s = now_mono - obs_history[0].recv_mono
            elif name == "pps_history" and pps_history and pps_history_lock:
                with pps_history_lock:
                    if pps_history:
                        oldest_age_s = now_mono - pps_history[0].recv_mono
            age_str = f"{oldest_age_s:.1f}s" if oldest_age_s is not None else "N/A"
            log.warning(
                "Queue depth threshold: %s depth=%d (threshold=%d) "
                "oldest_age=%s  all_depths: obs_q=%d obs_hist=%d pps_hist=%d",
                name, depth, threshold, age_str,
                depths["obs_queue"], depths["obs_history"], depths["pps_history"],
            )
            if dump:
                _dump_queue_diagnostics(
                    name, depths, queue_hwm, obs_history,
                    pps_history, pps_history_lock,
                    skip_stats, gate, now_mono,
                )
        # Re-arm alert when queue drains below half threshold
        if depth < max(1, threshold // 2) and not alert_armed[name]:
            alert_armed[name] = True


def _dump_queue_diagnostics(trigger_queue, depths, queue_hwm, obs_history,
                            pps_history, pps_history_lock,
                            skip_stats, gate, now_mono):
    """Dump detailed diagnostic state when a queue depth threshold is breached."""
    lines = [f"--- Queue diagnostic dump (triggered by {trigger_queue}) ---"]
    lines.append(f"  Current depths: {depths}")
    lines.append(f"  Session HWMs:   {queue_hwm}")

    # obs_history age spread
    if obs_history:
        oldest = now_mono - obs_history[0].recv_mono
        newest = now_mono - obs_history[-1].recv_mono
        lines.append(f"  obs_history age range: oldest={oldest:.2f}s newest={newest:.2f}s")
        # Show first few items' GPS times and confidence
        for i, ev in enumerate(obs_history):
            if i >= 5:
                lines.append(f"    ... ({len(obs_history) - 5} more)")
                break
            age = now_mono - ev.recv_mono
            conf = getattr(ev, "correlation_confidence", None)
            qr = getattr(ev, "queue_remains", None)
            lines.append(
                f"    [{i}] gps={ev.gps_time} age={age:.2f}s conf={conf} queue_remains={qr}"
            )

    # pps_history age spread
    if pps_history and pps_history_lock:
        with pps_history_lock:
            pps_snap = list(pps_history)
        if pps_snap:
            oldest = now_mono - pps_snap[0].recv_mono
            newest = now_mono - pps_snap[-1].recv_mono
            lines.append(f"  pps_history age range: oldest={oldest:.2f}s newest={newest:.2f}s "
                         f"(n={len(pps_snap)})")

    if skip_stats:
        lines.append(f"  skip_stats: {skip_stats}")

    if gate:
        lines.append(f"  gate_stats: {gate.stats.as_dict()}")

    lines.append("--- end diagnostic dump ---")
    log.warning("\n".join(lines))


def _queue_put_drop_oldest(qobj, item):
    """Enqueue one item, dropping at most one oldest entry if full.

    This queue is only a wakeup/notification path for sinks that keep their own
    history. When full, preserve continuity by dropping one oldest wakeup
    rather than draining the queue and erasing recent timing context.
    """
    try:
        qobj.put_nowait(item)
        return 0
    except queue.Full:
        try:
            qobj.get_nowait()
        except queue.Empty:
            pass
        qobj.put_nowait(item)
        return 1


def _append_queue_history(history, qobj, timeout=0.5):
    """Append one or more queued items into a history deque."""
    history.append(qobj.get(timeout=timeout))
    added = 1
    while True:
        try:
            history.append(qobj.get_nowait())
            added += 1
        except queue.Empty:
            return added


# ── Main ──────────────────────────────────────────────────────────────── #

def run(args):
    """Main entry point: bootstrap → steady state."""
    stop_event = threading.Event()
    gate_stats = None
    exit_code = 0
    # Verify receiver config on open (defensive: re-applies if needed).
    # This opens/closes the serial port to check for dual-freq observations,
    # reconfigures if single-freq, then releases the port for serial_reader.
    from peppar_fix.receiver import ensure_receiver_ready
    port_type = getattr(args, 'port_type', 'USB') or 'USB'
    systems_for_check = set(args.systems.split(',')) if args.systems else {'gps', 'gal'}
    # Single-constellation + smooth-precise-clocks filter degeneracy
    # warning.  See project_to_main_pride_gps_filter_degeneracy_20260423
    # and the --systems help text.  The filter's (rx_clk, ZTD,
    # common-ambiguity) mode is near-singular under these conditions;
    # a second constellation's separate clock state breaks it.
    # Broadcast-only runs are not susceptible because NAV page-
    # boundary clock discontinuities inject the rank information
    # that SSR smooth interpolation lacks.
    if len(systems_for_check) < 2 and args.ssr_mount:
        log.warning(
            "Single-constellation run (%s) with SSR corrections (%s) is "
            "subject to a near-singular filter mode where rx_clk, ZTD, and "
            "common-ambiguity drift together.  Recommend ≥ 2 constellations "
            "(gps,gal or gps,gal,bds).  Continuing per explicit --systems "
            "request; monitor ZTD and altitude for drift.",
            ','.join(sorted(systems_for_check)), args.ssr_mount,
        )
    # If --receiver was explicitly set, force that driver (skip auto-detect).
    # args.receiver is None when unset (default applied later in main()).
    forced = get_driver(args.receiver) if args.receiver is not None else None
    driver, receiver_identity = ensure_receiver_ready(
        args.serial, args.baud, port_type=port_type,
        systems=systems_for_check,
        sfrbx_rate=args.sfrbx_rate,
        measurement_rate_ms=args.measurement_rate_ms,
        forced_driver=forced)
    if driver is None:
        driver = get_driver(args.receiver)
        log.warning("Receiver check failed — falling back to %s (may lack dual-freq)",
                    driver.name)
    # Stash receiver identity for position persistence
    args.receiver_unique_id = (receiver_identity.get("unique_id")
                               if receiver_identity else None)
    mute_controller = get_source_mute_controller()
    mute_controller.install_signal_handlers()

    # State machines for observability (log transitions, don't control flow)
    ape_sm = AntPosEst(wl_only=bool(getattr(args, "wl_only", False)))
    dfe_sm = DOFreqEst()

    # Load ANTEX for PCV correction (Phase 2 of obs-model plan).  We
    # parse at engine startup so the one-time cost (~1s on IGS14.atx)
    # happens before any real-time observation processing.  Loading
    # failures are warnings, not errors: the engine continues without
    # PCV correction.
    _ape_antex_parser = None
    antex_path = getattr(args, "antex_path", None)
    recv_ant = getattr(args, "receiver_antenna", None)
    pcv_flag = bool(getattr(args, "pcv", True))
    if pcv_flag and antex_path and recv_ant:
        try:
            _ape_antex_parser = ANTEXParser(antex_path)
            log.info("ANTEX loaded from %s; receiver antenna = %r",
                     antex_path, recv_ant)
        except Exception as e:
            log.warning("ANTEX load failed (%s); continuing without PCV", e)
            _ape_antex_parser = None
    elif pcv_flag and (antex_path or recv_ant):
        log.warning(
            "PCV correction needs both --antex-path and --receiver-antenna; "
            "got antex=%s receiver=%s; continuing without PCV",
            antex_path, recv_ant)
    elif not pcv_flag:
        log.info("PCV correction disabled via --no-pcv")

    def on_signal(signum, frame):
        log.info("Signal received, shutting down")
        stop_event.set()
    signal.signal(signal.SIGTERM, on_signal)
    if args.pid_file:
        with open(args.pid_file, "w") as f:
            f.write(f"{os.getpid()}\n")
        log.info("Wrote PID file: %s", args.pid_file)

    # Shared state
    beph = BroadcastEphemeris()
    ssr = SSRState()
    corrections = RealtimeCorrections(beph, ssr)
    obs_queue = queue.Queue(maxsize=100)

    # QErr store (shared with serial reader if servo is active).
    # If --qerr-log was specified, open a CSV that captures every
    # TIM-TP message with its CLOCK_MONOTONIC arrival time, for
    # post-hoc index-matching against TICC chB events.
    qerr_store = None
    qerr_log_f = None
    want_servo = args.servo or getattr(args, 'ticc_port', None) is not None
    if want_servo:
        qerr_log_writer = None
        if getattr(args, 'qerr_log', None):
            try:
                # Append mode for wrapper-restart preservation; see
                # recoveryRetry-main.
                _qerr_log_needs_header = (
                    not os.path.exists(args.qerr_log)
                    or os.path.getsize(args.qerr_log) == 0
                )
                qerr_log_f = open(args.qerr_log, 'a', newline='')
                qerr_log_writer = csv.writer(qerr_log_f)
                if _qerr_log_needs_header:
                    qerr_log_writer.writerow([
                        'host_timestamp', 'host_monotonic', 'qerr_ns',
                        'tow_ms', 'qerr_invalid',
                    ])
                    qerr_log_f.flush()
                log.info("qErr CSV log: %s", args.qerr_log)
            except OSError as e:
                log.error("Failed to open qerr_log %s: %s", args.qerr_log, e)
                qerr_log_writer = None
                qerr_log_f = None
        qerr_store = QErrStore(log_writer=qerr_log_writer,
                               log_file=qerr_log_f)

    # Load NTRIP config
    load_ntrip_config(args)

    # --no-ssr: silently strip the PRIMARY SSR mount picked up from
    # ntrip.conf so the engine takes orbit/clock from broadcast (BCEP)
    # only.  A secondary bias mount (--ssr-bias-ntrip-conf) is left
    # alone — pairing --no-ssr with --ssr-bias-ntrip-conf gives a
    # "broadcast orbit/clock + biases from X" cell that is the
    # zero-orbit/clock-AC corner of the 4-cell 2x2 diagnostic in
    # docs/ssr-cross-ac-diagnostic-2026-04-25.md.
    if getattr(args, "no_ssr", False):
        if args.ssr_mount:
            log.info(f"--no-ssr: discarding primary ssr_mount "
                     f"'{args.ssr_mount}' (broadcast orbit/clock)")
        args.ssr_mount = None

    if not args.ntrip_caster and not args.eph_mount:
        log.warning("No NTRIP source — using broadcast ephemeris from receiver only")

    # Start NTRIP threads
    start_ntrip_threads(args, beph, ssr, stop_event)

    # Parse systems filter (needed before warmup)
    systems = set(args.systems.split(',')) if args.systems else None
    log.info(f"Systems: {systems}")

    # Wait for ephemeris
    if args.eph_mount:
        if not wait_for_ephemeris(beph, stop_event, systems=systems):
            return 1

    # NAV2 position store — captures the F9T secondary engine's fresh
    # position fix for the position-consensus watchdog.
    nav2_store = Nav2PositionStore()

    # NAV-SIG store — per-(SV, signal) usage verdict from the receiver.
    # Consumed by slipDetectUnified-main Phase A logger + Charlie's
    # Phase A.5 disagreement instrumentation.  Per Bob's framing
    # 2026-05-12: "we may use SVs the receiver doesn't, and may exclude
    # SVs it uses, but we must LOG WHEN WE DIFFER."  The store is the
    # data plane; engine-side disagreement logging consumes via
    # on_transition() registration.
    nav_sig_store = Nav2SignalStore()
    # Stash on args so _build_ppp_filter can forward into PPPFilter
    # when --nav-sig-gate is set.  Off by default; flip per overnight.
    args.nav_sig_store = nav_sig_store

    # NAV-CLOCK + NAV-TIMEGPS stores — receiver clock-state telemetry
    # for f9tClockTelemetry-bravo (recoveryRetry-main work item 2).
    # Diagnostic visibility into receiver-side clock state for the
    # chip-slip / command-envelope cascade hypotheses (I-115943-main).
    nav_clock_store = NavClockStore()
    nav_time_gps_store = NavTimeGpsStore()

    # gnss-phase-experiment: TIM-TM2 store for DOFreqEst Arm 3.  When
    # --no-extint is set we don't allocate the store and don't enable
    # the F9T's TIM-TM2 message — the EKF arm stays gated off cleanly
    # (consume_latest returns None forever).
    extint_store = None
    extint_log_f = None
    if not args.no_extint:
        _ext_log_writer = None
        if getattr(args, 'extint_log', None):
            try:
                # Append mode for wrapper-restart preservation; see
                # recoveryRetry-main.
                _extint_log_needs_header = (
                    not os.path.exists(args.extint_log)
                    or os.path.getsize(args.extint_log) == 0
                )
                extint_log_f = open(args.extint_log, 'a', newline='')
                _ext_log_writer = csv.writer(extint_log_f)
                if _extint_log_needs_header:
                    _ext_log_writer.writerow([
                        'host_timestamp', 'host_monotonic',
                        'wn', 'tow_ms', 'tow_sub_ms_ns',
                        'acc_est_ns', 'phase_residual_ns', 'count', 'flags',
                    ])
                    extint_log_f.flush()
                log.info("TIM-TM2 CSV log: %s", args.extint_log)
            except OSError as e:
                log.error("Failed to open extint_log %s: %s", args.extint_log, e)
                _ext_log_writer = None
                extint_log_f = None
        extint_store = TimTm2Store(log_writer=_ext_log_writer,
                                   log_file=extint_log_f)

    # Enable NAV2 + diagnostic UBX messages on the F9T.  This is done
    # here (before serial_reader starts) because the ensure_receiver
    # step only runs the full config when dual-freq observations are
    # missing — if the F9T was already configured for L5 from a
    # previous run, these keys would never be sent.  The config burst
    # is harmless if values are already correct.
    #
    # NAV2 + NAV-SIG + NAV-CLOCK + NAV-TIMEGPS are unconditional: the
    # engine instantiates their consumer stores regardless of CLI
    # flags, and the cohort tonight needs the data plane reliably
    # populated on every engine start (slipDetectUnified Phase A,
    # recoveryRetry telemetry).
    #
    # TIM-TM2 is gated on extint_store (the --no-extint flag) per
    # the existing pattern.  Receiver-side enabling is also handled
    # by configure_messages(), but that only runs on first-time
    # config; this burst guarantees enablement on every restart.
    try:
        from peppar_fix.receiver import send_cfg
        from peppar_fix.gnss_stream import open_gnss
        from pyubx2 import UBXReader as _UBR
        _nav2_ser, _ = open_gnss(args.serial, args.baud)
        _nav2_ubr = _UBR(_nav2_ser, protfilter=2)
        # args.port_type is the user-facing port string ("UART",
        # "UART2", "USB", "SPI", "I2C" per the --ubx-port choices).
        # Translate to the u-blox CFG-VALSET key suffix.  Note "UART"
        # maps to "UART1" — UBX keys use the indexed form even though
        # the user-facing CLI / TOML uses bare "UART".
        # Bug history: the previous code used PORT_SUFFIX.get
        # (int-keyed) against this string, silently falling back to
        # "USB" on every host.  Caught 2026-05-12 on MadHat F10T,
        # where USB-suffixed keys NAK because the F10T has no USB
        # port (the host-side /dev/f10t is the ArduSimple board's
        # USB-UART bridge, not the F10T's own interface).
        _PORT_SUFFIX_FOR_VALSET = {
            "UART": "UART1", "UART2": "UART2",
            "USB": "USB", "SPI": "SPI", "I2C": "I2C",
        }
        _pname = _PORT_SUFFIX_FOR_VALSET.get(
            args.port_type, args.port_type or "USB")
        _cfg_keys = {
            "CFG_NAV2_OUT_ENABLED": 1,
            f"CFG_MSGOUT_UBX_NAV2_PVT_{_pname}": 5,
            f"CFG_MSGOUT_UBX_NAV_SIG_{_pname}": 1,
            f"CFG_MSGOUT_UBX_NAV_CLOCK_{_pname}": 1,
            f"CFG_MSGOUT_UBX_NAV_TIMEGPS_{_pname}": 5,
        }
        if extint_store is not None:
            _cfg_keys[f"CFG_MSGOUT_UBX_TIM_TM2_{_pname}"] = 1
        _nav2_ok = send_cfg(_nav2_ser, _nav2_ubr, _cfg_keys,
                            "NAV2 + NAV-SIG/CLOCK/TIMEGPS + TIM-TM2 enable")
        _nav2_ser.close()
        if _nav2_ok:
            extras = ", TIM-TM2" if extint_store is not None else ""
            log.info("NAV2 + NAV-SIG + NAV-CLOCK + NAV-TIMEGPS%s"
                     " enabled (position consensus + cascade diagnostics)",
                     extras)
        else:
            log.warning("Post-config burst failed "
                        "(position consensus + diagnostics may be unavailable)")
    except Exception as e:
        log.warning("Post-config burst attempt failed: %s (continuing)", e)

    # Warm TICC port BEFORE the serial reader starts.
    # Opening the TICC may reboot the Arduino (DTR edge), which causes
    # USB re-enumeration that can disconnect the F9T serial port on hosts
    # sharing the USB bus (clkPoC3, MadHat).  Warming absorbs the
    # reboot here, while nothing else is using USB serial.  Subsequent
    # TICC opens (bootstrap, reader thread) get the warm path.
    ticc_port = getattr(args, 'ticc_port', None)
    if ticc_port:
        try:
            from ticc import warm_ticc_port
            warm_ticc_port(ticc_port, getattr(args, 'ticc_baud', 115200))
        except Exception as e:
            log.warning("TICC warm failed: %s (will retry later)", e)

    # Start serial reader
    serial_kwargs = {}
    if qerr_store:
        serial_kwargs['qerr_store'] = qerr_store
    # Build the RINEX writer if --rinex-out was set.  All init logic
    # lives in peppar_fix.rinex_writer.make_writer_from_args so it's
    # unit-testable; that module's tests cover the missing-arg /
    # undefined-variable bug shapes that previously shipped (NameError
    # on receiver_state, UnboundLocalError on known_ecef — fixed in
    # 1471216 + 8fa49fb).
    from peppar_fix.rinex_writer import make_writer_from_args
    rinex_writer_obj = make_writer_from_args(args, log=log)
    if rinex_writer_obj is not None:
        log.info(f"RINEX OBS writer: {args.rinex_out}")
        serial_kwargs['rinex_writer'] = rinex_writer_obj
    t_serial = threading.Thread(
        target=serial_reader,
        args=(args.serial, args.baud, obs_queue, stop_event, beph, systems, ssr),
        kwargs={**serial_kwargs, 'driver': driver, 'nav2_store': nav2_store,
                'extint_store': extint_store,
                'nav_sig_store': nav_sig_store,
                'nav_clock_store': nav_clock_store,
                'nav_time_gps_store': nav_time_gps_store},
        daemon=True,
    )
    t_serial.start()
    log.info(f"Serial: {args.serial} at {args.baud} baud")

    # Open CSV output
    out_f = None
    out_w = None
    if args.out:
        out_f = open(args.out, 'w', newline='')
        out_w = csv.writer(out_f)
        out_w.writerow([
            'timestamp', 'phase',
            'lat', 'lon', 'alt_m',
            'sigma_3d_m', 'clock_ns', 'clock_sigma_ns',
            'n_meas', 'rms_m',
            'correction_source', 'isb_gal_ns', 'isb_bds_ns',
            'n_ambiguities',
        ])

    # Determine starting position.
    #
    # Per I-024532-charlie #3 (Phase 1 / Phase 2 collapse): position
    # is seeded from NAV2 by default — the F9T's secondary nav engine
    # is independent of our PPP filter and gives a hAcc ~1 m fix on
    # clean sky within 30-60 s.  No bootstrap CONVERGED ceremony.
    #
    # Priority order:
    #   1. --known-pos config — operator override (sigma_m=0, treated
    #      as anchor-quality).
    #   2. NAV2 seed — fixType=3 + hAcc < 5 m (typical lab path).
    #   3. LS-init bootstrap — fallback for the regression harness
    #      and any cold-start where NAV2 isn't available within 60s.
    #
    # Receiver state file (state/receivers/<uid>.json) is no longer
    # read for position — it's identity-only now (TCXO calibration,
    # qErr characterization, slot mapping).  The position field, if
    # present from older runs, is logged for diagnostic comparison
    # only and is NOT used to seed the filter.  Per Q3 of the
    # I-133648-main consensus call.
    known_ecef = None
    pos_source = None
    pos_sigma_m = None
    uid = getattr(args, 'receiver_unique_id', None)

    # Diagnostic: log what's in the state file (if anything) for
    # comparison to whatever seed we end up using.
    if uid is not None:
        from peppar_fix.receiver_state import load_position_detail_from_receiver
        _state_ecef, _state_sigma, _state_src = (
            load_position_detail_from_receiver(uid))
        if _state_ecef is not None:
            log.info("Receiver state file has position (%s, σ=%sm) — "
                     "informational only, no longer used for seeding",
                     _state_src, _state_sigma)

    # 1. --known-pos operator override.
    if args.known_pos:
        lat, lon, alt = [float(v) for v in args.known_pos.split(',')]
        known_ecef = lla_to_ecef(lat, lon, alt)
        pos_source = "known_pos (config)"
        ape_sm.transition(AntPosEstState.VERIFYING, "known_pos from config")
        pos_sigma_m = 0.0
        # Persist to receiver state so the file's last-known position
        # stays current as a diagnostic record (not for seeding).
        if uid is not None:
            save_position_to_receiver(uid, known_ecef, 0.0, "known_pos")

    # 2. NAV2 seed — the typical lab path.
    if known_ecef is None and nav2_store is not None:
        seed = wait_for_nav2_seed(nav2_store, stop_event,
                                  timeout_s=60.0, hacc_max_m=5.0)
        if seed is not None:
            nav2_ecef, nav2_h_acc, nav2_n_sv = seed
            known_ecef = nav2_ecef
            pos_sigma_m = nav2_h_acc
            pos_source = f"NAV2 (hAcc={nav2_h_acc:.2f}m, nSV={nav2_n_sv})"
            ape_sm.transition(AntPosEstState.VERIFYING,
                              f"NAV2 seed @ hAcc={nav2_h_acc:.2f}m")
    if known_ecef is not None:
        # --seed-pos-offset: apply (E,N,U) displacement in meters to
        # whatever known_ecef was loaded.  Used for seed-error
        # experiments — see project_to_main_seed_error_sweep_result.
        seed_offset = getattr(args, "seed_pos_offset", None)
        if seed_offset:
            try:
                e, n, u = [float(v) for v in seed_offset.split(",")]
            except (ValueError, AttributeError):
                log.error(f"--seed-pos-offset must be 'E,N,U' in metres; "
                          f"got {seed_offset!r}")
                return 1
            # Build ENU→ECEF rotation at known_ecef and apply offset.
            from solve_pseudorange import ecef_to_enu  # noqa: F401
            lat0, lon0, _ = ecef_to_lla(known_ecef[0], known_ecef[1], known_ecef[2])
            lat_r = math.radians(lat0)
            lon_r = math.radians(lon0)
            R_T = np.array([
                [-math.sin(lon_r),
                 -math.sin(lat_r) * math.cos(lon_r),
                 math.cos(lat_r) * math.cos(lon_r)],
                [math.cos(lon_r),
                 -math.sin(lat_r) * math.sin(lon_r),
                 math.cos(lat_r) * math.sin(lon_r)],
                [0.0,
                 math.cos(lat_r),
                 math.sin(lat_r)],
            ])
            offset_ecef = R_T @ np.array([e, n, u])
            known_ecef = np.asarray(known_ecef) + offset_ecef
            log.info(f"--seed-pos-offset applied: E={e:+g}m N={n:+g}m U={u:+g}m "
                     f"→ ECEF Δ=({offset_ecef[0]:+.3f}, {offset_ecef[1]:+.3f}, "
                     f"{offset_ecef[2]:+.3f})m")

        lat, lon, alt = ecef_to_lla(known_ecef[0], known_ecef[1], known_ecef[2])
        log.info(f"Position ({pos_source}): {lat:.6f}, {lon:.6f}, {alt:.1f}m")

    bootstrap_result = None  # Set by run_bootstrap, None on warm start
    try:
        if known_ecef is None:
            # Phase 1: Bootstrap
            result = run_bootstrap(args, obs_queue, corrections, stop_event,
                                   out_w=out_w, nav2_store=nav2_store)
            if result is None:
                log.error("Bootstrap failed — no converged position")
                return 1

            bootstrap_result = result
            known_ecef = bootstrap_result.ecef
            sigma_m = bootstrap_result.sigma_m
            pos_sigma_m = sigma_m  # threaded into run_steady_state for σ_pin init
            pos_source = pos_source or "ppp_bootstrap"
            ape_sm.transition(AntPosEstState.CONVERGING,
                              f"bootstrap converged (σ={sigma_m:.1f}m), entering steady state")

            # Save position.  Inflated σ — see
            # _PHASE1_BOOTSTRAP_SAVE_FLOOR_M above for why Phase-1
            # bootstrap saves opt out of the trust-skip-validation
            # fast-start path.
            uid = getattr(args, 'receiver_unique_id', None)
            if uid is not None:
                stored_sigma = max(sigma_m, _PHASE1_BOOTSTRAP_SAVE_FLOOR_M)
                save_position_to_receiver(
                    uid, known_ecef, stored_sigma, "ppp_bootstrap")
                log.info(
                    "Position saved to receiver state "
                    "(true σ=%.2fm, stored σ=%.2fm above trust "
                    "threshold so next start re-validates against "
                    "live LS fix)",
                    sigma_m, stored_sigma)

        if stop_event.is_set():
            return 0

        # Validate loaded position against live pseudorange fix.
        # A tampered or stale position file would send the FixedPosFilter
        # into 100+ km residuals without any warning.
        #
        # Skip validation for trusted positions:
        #   - --known-pos (sigma_m=0, operator override).
        #   - NAV2 seed — already validated by virtue of being a live
        #     fix from the receiver's secondary nav engine.  The
        #     hAcc<5m gate at seed acquisition (wait_for_nav2_seed)
        #     IS the LS validation, just done by the receiver's own
        #     SPP solver instead of ours.  Per I-024532-charlie #3.
        #
        # Receiver-state position-read is no longer a seeding path
        # (file is identity-only now), so the historical
        # _TRUSTED_POSITION_SIGMA_M trust-skip is mostly vestigial —
        # it only triggers when --known-pos is used (sigma=0 < threshold).
        seeded_from_nav2 = (pos_source is not None
                            and pos_source.startswith("NAV2"))
        skip_validation = (
            seeded_from_nav2
            or (pos_sigma_m is not None
                and pos_sigma_m < _TRUSTED_POSITION_SIGMA_M))
        uid = getattr(args, 'receiver_unique_id', None)
        if skip_validation:
            why = ("NAV2 seed (live-fix-validated)" if seeded_from_nav2
                   else f"trusted source (σ={pos_sigma_m:.1f}m)")
            log.info("Position from %s — skipping LS validation", why)
            ape_sm.transition(AntPosEstState.CONVERGING,
                              f"{why}, entering steady state")
        elif uid is not None or args.known_pos:
            log.info('Validating loaded position against live LS fix...')
            for _attempt in range(30):
                if stop_event.is_set():
                    return 0
                try:
                    gps_time, observations = obs_queue.get(timeout=5)
                except Exception:
                    continue
                if len(observations) < 6:
                    continue
                # Use broadcast-only ephemeris for the LS validation check,
                # NOT the full SSR-corrected RealtimeCorrections object.
                # CAS single-AC SSR orbit+clock corrections cause the LS
                # solver to produce wildly wrong positions (altitude -2000m)
                # when the SSR correction reference frame doesn't match the
                # broadcast ephemeris's reference.  FixedPosFilter is immune
                # (time differencing cancels the bias) but the LS solver's
                # absolute pseudorange model is not.  Using broadcast-only
                # for validation gives ~5-10m accuracy which is plenty for
                # the 100m threshold check.
                x_ls, ok, n_sv = ls_init(observations, beph, gps_time,
                                          clk_file=None)
                if not ok or n_sv < 6:
                    continue
                ls_ecef = x_ls[:3]
                import numpy as _np
                separation_m = _np.linalg.norm(ls_ecef - known_ecef)
                ls_lat, ls_lon, ls_alt = ecef_to_lla(ls_ecef[0], ls_ecef[1], ls_ecef[2])
                log.info(f'  LS check: ({ls_lat:.4f}, {ls_lon:.4f}, {ls_alt:.0f}m) '
                         f'separation={separation_m:.0f}m from loaded position')
                if separation_m > 100:
                    log.error(f'Position file disagrees with live LS fix by {separation_m:.0f}m '
                              f'(threshold 100m). File may be stale or corrupted. '
                              f'Falling back to bootstrap.')
                    known_ecef = None
                    result = run_bootstrap(args, obs_queue, corrections, stop_event,
                                           out_w=out_w)
                    if result is None:
                        log.error('Bootstrap failed')
                        return 1
                    bootstrap_result = result
                    known_ecef = bootstrap_result.ecef
                    sigma_m = bootstrap_result.sigma_m
                    uid = getattr(args, 'receiver_unique_id', None)
                    if uid is not None:
                        stored_sigma = max(
                            sigma_m, _PHASE1_BOOTSTRAP_SAVE_FLOOR_M)
                        save_position_to_receiver(
                            uid, known_ecef, stored_sigma, "ppp_bootstrap")
                        log.info(
                            "Position saved to receiver state "
                            "(re-bootstrapped; true σ=%.2fm, stored "
                            "σ=%.2fm above trust threshold)",
                            sigma_m, stored_sigma)
                else:
                    log.info(f'  Position validated (within {separation_m:.0f}m of LS fix)')
                    ape_sm.transition(AntPosEstState.CONVERGING,
                                      f"LS validation passed ({separation_m:.0f}m), entering steady state")
                break

        if stop_event.is_set():
            return 0

        # Start AntPosEst background thread — keeps PPPFilter alive for
        # continuous position refinement and AR.  On cold start, reuses
        # the converged PPPFilter from bootstrap.  On warm start (position
        # loaded from receiver state), creates a fresh PPPFilter at the
        # known position.
        # AntPosEst → DOFreqEst position migration via exponential blend.
        # The callback writes the latest AR position; the steady-state loop
        # blends it into filt.pos at alpha=0.001 per epoch (~17 min τ).
        # See docs/ppp-ar-design.md "Gradual position feed-in: the math".
        _ar_position = {'ecef': None, 'sigma': None}
        _ar_pos_lock = threading.Lock()

        def _position_improved(ecef, sigma_m):
            lat, lon, alt = ecef_to_lla(ecef[0], ecef[1], ecef[2])
            log.info("AntPosEst position improved: σ=%.3fm (%.6f, %.6f, %.1f)",
                     sigma_m, lat, lon, alt)
            with _ar_pos_lock:
                _ar_position['ecef'] = np.array(ecef, dtype=float)
                _ar_position['sigma'] = sigma_m

        # AntPosEstThread fresh-PPPFilter init (warm-start path) gets
        # the seed σ from whichever source supplied known_ecef.  When
        # bootstrap_result is supplied (legacy LS-init bootstrap path),
        # the thread inherits that filter's already-tightened P and
        # the pos_sigma_m argument is unused.  Per I-024532-charlie #3.
        ape_pos_sigma_m = (pos_sigma_m
                           if pos_sigma_m is not None and pos_sigma_m > 0
                           else 10.0)
        ape_thread = AntPosEstThread(
            known_ecef=known_ecef,
            corrections=corrections,
            stop_event=stop_event,
            ape_sm=ape_sm,
            bootstrap_result=bootstrap_result,
            position_callback=_position_improved,
            nav2_store=nav2_store,
            systems=set(args.systems.split(',')) if args.systems else None,
            ar_elev_mask_deg=args.ar_elev_mask,
            nl_diag_enabled=bool(getattr(args, "nl_diag", False)),
            join_test_enabled=bool(getattr(args, "join_test", True)),
            wl_only=bool(getattr(args, "wl_only", False)),
            solid_tide=bool(getattr(args, "solid_tide", True)),
            antex_parser=_ape_antex_parser,
            receiver_antenna_type=getattr(args, "receiver_antenna", None),
            pcv_enabled=bool(getattr(args, "pcv", True)),
            clock_model=getattr(args, "clock_model", "random_walk"),
            rx_tcxo_adev_1s=getattr(args, "rx_tcxo_adev_1s", None),
            phase_windup_enabled=bool(getattr(args, "phase_windup", False)),
            gmf_enabled=bool(getattr(args, "gmf", False)),
            slip_rate_limit_s=float(
                getattr(args, "slip_rate_limit_s", 0.0)),
            ztd_tie_sigma=getattr(args, "ztd_tie_sigma", None),
            pos_sigma_m=ape_pos_sigma_m,
            nav2_anchor_enabled=bool(
                getattr(args, "nav2_soft_anchor", True)),
            nav2_anchor_max_hacc_m=float(
                getattr(args, "nav2_anchor_max_hacc_m", 3.0)),
            pin_position=bool(getattr(args, "pin_position", False)),
            nav_sig_store=nav_sig_store,
        )
        # Phase B NAV-SIG L0 admission gate — attach store + flags to
        # the thread so its fresh-PPPFilter path (warm-start, not
        # bootstrap-inherited) picks them up via _build_ppp_filter.
        # When --nav-sig-gate is off these are inert (filter constructor
        # ignores them).
        ape_thread._nav_sig_store = nav_sig_store
        ape_thread._nav_sig_gate_enabled = bool(
            getattr(args, 'nav_sig_gate', False))
        ape_thread._nav_sig_max_age_s = float(
            getattr(args, 'nav_sig_max_age_s', 2.0))
        ape_thread.start()

        # Phase 2: Steady state.  On exit code 5 (catastrophic-reject
        # cascade or PHC divergence) the engine returns to the wrapper,
        # which relaunches a fresh process — that path correctly
        # re-initializes EPH / SSR / Serial / AntPosEst reader threads.
        # The previous in-process retry loop did NOT restart those
        # subsystems on attempt 2; it called run_steady_state again
        # with torn-down input threads, hit "DO bootstrap failed",
        # returned a 0-epoch gate_stats dict (not int 5), and the
        # outer logic exited with default code 0 — silently blinding
        # the wrapper to the failure.  See recoveryRetry-main.
        steady_result = run_steady_state(
            args,
            known_ecef,
            obs_queue,
            corrections,
            beph,
            ssr,
            stop_event,
            qerr_store=qerr_store,
            out_w=out_w,
            nav2_store=nav2_store,
            ape_sm=ape_sm,
            dfe_sm=dfe_sm,
            ape_thread=ape_thread,
            ar_position=_ar_position,
            ar_pos_lock=_ar_pos_lock,
            extint_store=extint_store,
            pos_sigma_m=pos_sigma_m,
            pos_source=pos_source,
        )
        # run_steady_state returns an int exit code on error
        # (e.g. 5 for catastrophic-reject cascade) or a gate_stats
        # dict on normal completion.
        if isinstance(steady_result, int):
            exit_code = steady_result
        else:
            gate_stats = steady_result

    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        stop_event.set()
        if out_f:
            out_f.close()
        if args.gate_stats and gate_stats is not None:
            with open(args.gate_stats, "w") as f:
                json.dump(gate_stats, f, indent=2, sort_keys=True)

    if args.pid_file:
        try:
            os.unlink(args.pid_file)
        except FileNotFoundError:
            pass
    return exit_code


# ── Host config resolution ────────────────────────────────────────── #


def _apply_host_config(args):
    """Apply host config TOML defaults to args that weren't set on CLI.

    Resolves config/<hostname>.toml (or explicit --host-config) and fills
    in any arg that is still at its argparse default.  CLI always wins.
    """
    import socket
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(script_dir)

    # Find config file
    candidates = []
    if args.host_config:
        candidates = [args.host_config]
    else:
        hostname = socket.gethostname().split(".")[0].lower()
        candidates = [
            os.path.join(repo_root, "config", f"{hostname}.toml"),
            "/etc/peppar-fix/config.toml",
        ]

    cfg = {}
    cfg_path = None
    for path in candidates:
        if os.path.exists(path):
            with open(path, "rb") as f:
                data = tomllib.load(f)
            cfg = data.get("peppar", {})
            cfg_path = path
            break

    if not cfg:
        return  # no host config found

    # Log after basicConfig (called later in main), so just stash the path.
    args._host_config_path = cfg_path

    # Map from TOML key → (argparse dest, type conversion).
    # Only apply if the CLI arg is still at its default (None or argparse
    # default).  This preserves "CLI overrides host config overrides defaults".
    _MAP = {
        "serial":           ("serial",           str),
        "baud":             ("baud",             int),
        "ubx_port":         ("port_type",        str),
        "port_type":        ("port_type",        str),
        "receiver":         ("receiver",         str),
        "ptp_profile":      ("ptp_profile",      str),
        "ptp_dev":          ("servo",            str),
        "ntrip_conf":       ("ntrip_conf",       str),
        "eph_mount":        ("eph_mount",        str),
        "ssr_mount":        ("ssr_mount",        str),
        "ssr_ntrip_conf":   ("ssr_ntrip_conf",   str),
        "ssr_bias_mount":     ("ssr_bias_mount",     str),
        "ssr_bias_ntrip_conf":("ssr_bias_ntrip_conf",str),
        "known_pos":        ("known_pos",        str),
        "systems":          ("systems",          str),
        "duration":         ("duration",         int),
        "log":              ("servo_log",        str),
        "do_label":         ("do_label",         str),
        "do_type":          ("do_type",          str),
        "dac_bus":          ("dac_bus",           int),
        "dac_addr":         ("dac_addr",         str),
        "dac_bits":         ("dac_bits",         int),
        "dac_center_code":  ("dac_center_code",  int),
        "dac_ppb_per_code": ("dac_ppb_per_code", float),
        "dac_max_ppb":      ("dac_max_ppb",      float),
        "dac_type":         ("dac_type",         str),
        "dac_gain":         ("dac_gain",         int),
        "tadd_gpio":        ("tadd_gpio",        int),
        "tadd_hold_s":      ("tadd_hold_s",      float),
        "ticc_port":        ("ticc_port",        str),
        "phase_step_bias_ns": ("phase_step_bias_ns", float),
        "ar_elev_mask_deg": ("ar_elev_mask",         float),
        "pmc_uds":          ("pmc",              str),
        "pmc_domain":       ("pmc_domain",       int),
        "rinex_out":        ("rinex_out",        str),
        "rinex_decimate_s": ("rinex_decimate_s", float),
    }

    for toml_key, (dest, conv) in _MAP.items():
        if toml_key not in cfg:
            continue
        current = getattr(args, dest, None)
        # Only apply if CLI didn't set it (still at default None or
        # argparse default for special cases)
        if current is not None:
            continue
        try:
            setattr(args, dest, conv(cfg[toml_key]))
        except (ValueError, TypeError) as e:
            log.warning("Host config: bad value for %s: %s", toml_key, e)

    # ticc_drive removed — TICC competes as a source whenever --ticc-port
    # is configured.  Legacy host configs with ticc_drive=true are ignored.


# ── CLI ──────────────────────────────────────────────────────────────── #

def main():
    ap = argparse.ArgumentParser(
        description="Unified peppar-fix: GNSS position bootstrap + clock discipline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Two-phase operation:
  Phase 1 (Bootstrap): PPPFilter estimates position from scratch.
          Skipped if --known-pos or --position-file provides a position.
  Phase 2 (Steady state): FixedPosFilter estimates clock.
          Optional: --servo for PHC discipline, --out for CSV logging.
""",
    )

    # Host config (auto-discovered by hostname or explicit)
    cfg = ap.add_argument_group("Host configuration")
    cfg.add_argument("--host-config", default=None,
                     help="Explicit host config TOML path. If omitted, auto-discovers "
                          "config/<hostname>.toml or /etc/peppar-fix/config.toml")

    # Position
    pos = ap.add_argument_group("Position")
    pos.add_argument("--known-pos",
                     help="Known position as lat,lon,alt (skips bootstrap)")
    pos.add_argument("--pin-position", action="store_true",
                     help="Disable the slow AntPosEst→known_ecef blend in "
                          "run_steady_state.  Use when --known-pos is a "
                          "surveyed truth (sub-cm σ) that should NOT be "
                          "drifted toward AntPosEst's ~m-class estimate.  "
                          "Default off — preserves the warm-start refine "
                          "behavior for rough-seed --known-pos use.  "
                          "Precursor to the full --surveyed-position mode "
                          "(I-013342-main).")
    pos.add_argument("--init-ztd-mm", type=float, default=None,
                     help="Manually seed FixedPosFilter ZTD residual "
                          "at this value (millimetres) at filter init "
                          "instead of the default 0.0.  Companion to "
                          "--init-ztd-sigma-mm.  Use 0.0 ± 50.0 to ask "
                          "'is the architecture sound under tight ZTD "
                          "prior?' independently of any atmospheric-data "
                          "source.  Bravo's I-024942-main MetAR-seeded "
                          "path will use the same FixedPosFilter API "
                          "with the Saastamoinen-derived value.")
    pos.add_argument("--init-ztd-sigma-mm", type=float, default=None,
                     help="1-σ uncertainty on --init-ztd-mm seed in "
                          "millimetres.  Default: legacy 500 mm wide "
                          "prior.  Tighter (e.g. 50) requires confidence "
                          "in the seed value (METAR/Saastamoinen) and "
                          "constrains the filter from drifting ZTD past "
                          "physical envelope while clock state catches "
                          "up.")
    pos.add_argument("--seed-pos",
                     help="Seed position for bootstrap (speeds convergence)")
    pos.add_argument("--sigma", type=float, default=3.0,
                     help="Bootstrap convergence threshold in meters "
                          "(default: 3.0).  This is the loosest σ that "
                          "still keeps the downstream sanity checks "
                          "meaningful: W2 NAV2 cross-check requires "
                          "horizontal disagreement < 5 m (so 3 m σ "
                          "gives W2 a 2 m discrimination margin), and "
                          "SecondOpinionPosMonitor trips at nav2Δ > 5 m "
                          "3D (so 3 m σ gives SO_POS a 2 m headroom on "
                          "entry).  Tighter than 3 m buys nothing for "
                          "Phase 2 reliability — Phase 2 inherits the "
                          "live PPPFilter and the EKF tightens "
                          "naturally regardless of handoff σ; the "
                          "skip-validation gate elsewhere only requires "
                          "<10 m.  Looser than 5 m makes W2 redundant "
                          "(σ gate accepts positions W2 would reject) "
                          "and risks immediate SO_POS trips at handoff.")
    pos.add_argument("--bootstrap-nav2-horiz-m", type=float, default=5.0,
                     help="Phase-1 convergence aborts if NAV2 horizontal "
                          "disagreement exceeds this, even when the EKF "
                          "reports σ < --sigma.  Set to 0 to disable the "
                          "NAV2 cross-check (not recommended).")
    pos.add_argument("--bootstrap-nav2-vert-m", type=float, default=10.0,
                     help="Phase-1 convergence aborts if NAV2 vertical "
                          "disagreement exceeds this.  Vertical PPP/SPP "
                          "error is typically ~2× horizontal due to vDOP "
                          "geometry; the default 10 m pairs with the 5 m "
                          "horizontal threshold.  Catches the failure mode "
                          "where Phase-1 converges with horizontal-good but "
                          "vertical wildly-off (e.g., LS-init seeding the "
                          "altitude axis to a 100+ m wrong basin and W2 "
                          "horizontal-only missing it).  Set to 0 to "
                          "disable.")
    pos.add_argument("--nav2-seed-timeout-s", type=float, default=30.0,
                     help="Pre-init: wait up to this long for NAV2 to "
                          "deliver a fixType=3 + hAcc<5m opinion before "
                          "falling back to LS-init.  NAV2 is the preferred "
                          "seed (~1 m hAcc on clean sky vs LS-init's "
                          "~5-10 m + km-scale altitude DOP).  On a warm "
                          "receiver NAV2 is typically ready immediately; "
                          "on a true cold start (factory reset) it can "
                          "take 30-60 s.  Set to 0 to disable the wait "
                          "and use LS-init immediately when NAV2 isn't "
                          "ready on the first epoch.")
    pos.add_argument("--no-nav2-soft-anchor",
                     dest="nav2_soft_anchor",
                     action="store_false", default=True,
                     help="Disable the per-epoch NAV2 covariance update "
                          "in AntPosEst.  By default the engine applies "
                          "a 3D variance-weighted Kalman measurement "
                          "update at every epoch with R = diag(hAcc², "
                          "hAcc², vAcc²) rotated to ECEF.  NOTE: despite "
                          "the legacy 'soft-anchor' flag name, this is "
                          "variance-weighted update math, not truth-pull "
                          "(static offsets persist once filter P "
                          "tightens — see I-051234).  Disable for "
                          "diagnostic A/B or when NAV2 is known to be "
                          "biased (rare).")
    pos.add_argument("--nav2-anchor-max-hacc-m", type=float, default=3.0,
                     help="Skip the NAV2 covariance update when NAV2's "
                          "reported hAcc exceeds this threshold (NAV2 too "
                          "noisy to trust as a continuous position witness).  "
                          "Default 3 m matches Bravo's SO_POS hAcc gate.")
    pos.add_argument("--init-ztd-from-met",
                     action="store_true", default=False,
                     help="Seed FixedPosFilter ZTD state from latest METAR "
                          "via Saastamoinen.  Queries aviationweather.gov "
                          "on demand at filter init (no cron / local file "
                          "needed; see --init-ztd-station to pick a "
                          "station).  Avoids the multi-meter ZTD cold-"
                          "start transient that pin-position mode exposes "
                          "(see I-024942 + 2026-05-04 overnight findings).  "
                          "Default off — preserves current init_ztd_m=0 "
                          "behavior.  Mutually exclusive with --init-ztd-mm "
                          "(manual seed).")
    pos.add_argument("--init-ztd-station", default=DEFAULT_STATION,
                     metavar="ICAO",
                     help=f"ICAO id of the airport for METAR fetch when "
                          f"--init-ztd-from-met is set (default "
                          f"{DEFAULT_STATION}, ~7 km from UFO1).  Pick a "
                          f"station within ~50 km of the antenna for "
                          f"meaningful pressure/temperature similarity.")
    pos.add_argument("--ztd-tie-interval-s", type=int, default=60,
                     help="Apply a soft METAR-derived prior on the "
                          "FixedPosFilter ZTD residual every N epochs "
                          "(approx N seconds at 1 Hz; default 60).  "
                          "Set to 0 to disable.  Bounds ZTD-state "
                          "drift driven by per-SV systematic-bias "
                          "absorption (SSR L5I→L5Q substitution, "
                          "antenna PCV miscorrection, multipath); see "
                          "I-172521.  More frequent than the 5-min "
                          "METAR report cadence is fine — the cached "
                          "report just gets re-applied, which still "
                          "compounds the soft pull against the "
                          "per-epoch bias-absorption walk.  Re-uses "
                          "--init-ztd-station for the source airport.")
    pos.add_argument("--ztd-tie-sigma-mm", type=float, default=30.0,
                     help="1-σ uncertainty (mm) on the periodic ZTD "
                          "tie target.  Default 30 mm — matches the "
                          "METAR + Saastamoinen + spatial-coherence "
                          "error budget (1 hPa → 2 mm ZHD; "
                          "Saastamoinen model ~1-3 cm; nearby airport "
                          "spatial coherence ~1 cm).  Smaller → "
                          "tighter pull (more rapid convergence but "
                          "may damp legitimate atmospheric tracking); "
                          "larger → softer (filter dominates and the "
                          "tie becomes nearly a noop, since the "
                          "filter's σ is bias-polluted not honest).")
    pos.add_argument("--bootstrap-rms-k", type=float, default=2.0,
                     help="Phase-1 convergence requires PR-residual RMS < "
                          "k × SIGMA_P_IF.  Catches locally-consistent but "
                          "wrong states where outliers have been downweighted.")
    pos.add_argument("--bootstrap-max-retries", type=int, default=3,
                     help="On W1 or W2 abort, scrub the filter and retry this "
                          "many times before giving up.  Default 3.")
    pos.add_argument("--ar-elev-mask", type=float, default=None,
                     help="Elevation mask for integer-ambiguity resolution, "
                          "in degrees.  Separate from the measurement "
                          "ELEV_MASK — SVs below this stay in the float "
                          "filter but don't attempt NL fixing.  Default "
                          "25° (antenna-dependent; low-multipath antennas "
                          "can go lower — override via host TOML "
                          "ar_elev_mask_deg or this CLI flag).  Set to 0 "
                          "to disable and let every WL-fixed SV attempt NL.")
    pos.add_argument("--nl-diag", action="store_true",
                     help="Enable per-SV NL-attempt diagnostic logging.  "
                          "Emits one [NL_DIAG] line per SV per attempt and "
                          "one [NL_DIAG_BATCH] line per LAMBDA attempt.  "
                          "Off by default to keep long runs clean.  Use to "
                          "diagnose NL-doesn't-land situations — see "
                          "scripts/peppar_fix/nl_diag.py for field semantics.")
    pos.add_argument("--no-join-test", dest="join_test",
                     action="store_false", default=True,
                     help="Disable the pre-commit join test that protects "
                          "ANCHORED anchors from biased re-admissions.  "
                          "On by default.  Used for the same-sky A/B that "
                          "isolates the join test's effect from other "
                          "branch-carried changes.  See "
                          "project_to_main_defensive_mechanisms_20260421.md.")
    pos.add_argument("--ar-mode", choices=["wl", "full"], default="wl",
                     help="Ambiguity resolution mode for the position "
                          "filter.  'wl' (default, production): fix WL "
                          "integers via MW tracker, skip NL entirely.  "
                          "SvAmbState and AntPosEstState lifecycles "
                          "clamp at CONVERGING.  'full' (research only): "
                          "WL + NL, including LAMBDA + bootstrap-prob + "
                          "ANCHORING/ANCHORED transitions.  See "
                          "docs/wl-only-foundation.md and dayplan "
                          "I-125649-main for why 'wl' is the production "
                          "default — six weeks of NL-AR pathology "
                          "(slip storms, ZTD-trip cascades, false-fix "
                          "lock-in) and FixedPosFilter's TD-CP design "
                          "make NL not load-bearing for clock transfer.")
    # Deprecated alias: --wl-only sets --ar-mode wl (suppressed help).
    pos.add_argument("--wl-only", dest="wl_only_legacy",
                     action="store_true",
                     help=argparse.SUPPRESS)
    pos.add_argument("--position-blend-source",
                     choices=["nav2", "antposest", "none"],
                     default="nav2",
                     help="Source for the FixedPosFilter ARP blend "
                          "under --ar-mode wl.  'nav2' (default): "
                          "NAV2 running mean — best precision available "
                          "today given WL-only AntPosEst basin trap "
                          "(see I-125649 cold-start finding 2026-05-09 "
                          "and umbrella I-155354).  'antposest': feed "
                          "from AntPosEst published position — for "
                          "research today, will become the default once "
                          "the systematic float/WL biases close "
                          "(I-155354 / I-165118).  'none': no blend, "
                          "σ_pin frozen at init.  Ignored under "
                          "--ar-mode full (legacy α=0.001 trickle "
                          "always uses AntPosEst there) and under "
                          "--pin-position (blend is always skipped, "
                          "surveyed truth pinned).")
    pos.add_argument("--no-solid-tide", dest="solid_tide",
                     action="store_false", default=True,
                     help="Disable the IERS 2010 Step 1 solid Earth "
                          "tide displacement correction.  On by default: "
                          "Bravo's harness measured −39 mm improvement on "
                          "clean GAL+BDS geometry on ABMF 2020/001, "
                          "matching PRIDE's ablation prediction.  Phase 1 "
                          "of docs/obs-model-completion-plan.md.  Use "
                          "this flag to A/B test the correction's impact "
                          "on lab performance.")
    pos.add_argument("--no-pcv", dest="pcv",
                     action="store_false", default=True,
                     help="Disable satellite + receiver antenna phase "
                          "center variation / offset (PCV / PCO) "
                          "correction.  On by default when --antex-path "
                          "is provided.  Bravo's harness measured "
                          "−117 mm improvement on clean GAL+BDS geometry "
                          "on ABMF 2020/001 (Phase 2 of obs-model-"
                          "completion-plan.md).  Requires --antex-path "
                          "and --receiver-antenna.")
    pos.add_argument("--antex-path", default=None,
                     help="Path to IGS ANTEX (.atx) file providing "
                          "antenna phase center calibrations.  Required "
                          "for PCV correction; when absent, PCV is "
                          "silently skipped.  IGS14.atx is ~2 MB, "
                          "freely available from "
                          "https://files.igs.org/pub/station/general/")
    pos.add_argument("--receiver-antenna", default=None,
                     help="ANTEX receiver-antenna type string for this "
                          "host's antenna, e.g. 'TRM57971.00     NONE' "
                          "(20-char TYPE field from ANTEX).  Required "
                          "for PCV correction; when absent, PCV is "
                          "skipped with a warning.")
    pos.add_argument("--clock-model", default="random_walk",
                     choices=("random_walk", "calibrated_white"),
                     help="rx TCXO process-noise model in PPPFilter.  "
                          "random_walk (default) uses the legacy "
                          "Q_clk = 1e6·dt — IDX_CLK is effectively "
                          "unconstrained.  calibrated_white uses "
                          "Q_clk = (c · σ_y(1s))² · dt derived from the "
                          "rx TCXO's fractional-frequency ADEV, "
                          "tightening the null-mode clock axis.  "
                          "Prefer σ_y(1s) from a lab-local F9T "
                          "characterization; the default "
                          "--rx-tcxo-adev-1s is intentionally "
                          "pessimistic.  See "
                          "docs/clock-state-modeling.md option (A).")
    pos.add_argument("--rx-tcxo-adev-1s", type=float, default=None,
                     help="Fractional-frequency Allan deviation at τ=1s "
                          "for the receiver's rx TCXO.  Consumed only "
                          "when --clock-model=calibrated_white.  When "
                          "omitted, uses PPPFilter's pessimistic "
                          "default (~1e-8).  Typical F9T datasheet "
                          "values: 5e-11 to 1e-10.  Our lab-measured "
                          "DO TCXO maps to σ_y(1s)≈2e-9 (see "
                          "docs/ticc-baseline-2026-04-01.md).")
    pos.add_argument("--nav-sig-gate", action="store_true",
                     help="Enable the NAV-SIG L0 admission gate in "
                          "the position filter (PPPFilter).  When set, "
                          "observations whose corresponding UBX-NAV-SIG "
                          "entries report prUsed=0 on either band (or "
                          "health=2 unhealthy) are dropped from the "
                          "filter update.  When NAV-SIG data is stale "
                          "(> --nav-sig-max-age-s) or missing entirely, "
                          "the gate falls through and our own quality "
                          "checks are the only gate.  Default off — "
                          "Phase B1 ships the code dormant.  Flip on "
                          "after Phase A empirical data confirms the "
                          "receiver's per-signal verdicts correlate "
                          "with our own cat-reject events.  See "
                          "docs/nav-sig-gate-spec.md.")
    pos.add_argument("--nav-sig-max-age-s", type=float, default=2.0,
                     help="SigStatus older than this (seconds, relative "
                          "to time.monotonic() at filter update) is "
                          "treated as missing — the gate falls through. "
                          "Default 2.0 covers a one-epoch lag at 1 Hz "
                          "NAV-SIG cadence.  Increase if NAV-SIG arrives "
                          "less frequently than RAWX on a given "
                          "transport.")
    pos.add_argument("--sigma-phi-if", type=float, default=None,
                     help="Override the IF carrier-phase measurement "
                          "noise sigma (meters).  Default uses "
                          "solve_ppp.SIGMA_PHI_IF (0.03 m).  Looser "
                          "values (e.g. 1.0) tell the filter to "
                          "trust phase observations less — useful "
                          "when the observation model has unmodeled "
                          "biases (CNES L5I-vs-F9T-L5Q phase bias, "
                          "missing wind-up / GMF / sat attitude).  "
                          "Tighter σ_phi + biased model amplifies "
                          "null-mode coupling into position / ZTD.  "
                          "Bravo's PRIDE/ABMF sweep (2026-04-24) "
                          "showed σ_phi=1.0-3.0 reduces 24h mean |H| "
                          "by 50%% on clean reference data.  See "
                          "project_to_main_sigma_phi_sweep_20260424.md.")
    pos.add_argument("--sigma-pr-if", type=float, default=None,
                     help="Override the IF pseudorange measurement "
                          "noise sigma (meters).  Default uses "
                          "solve_ppp.SIGMA_P_IF (3.0 m).  Companion "
                          "knob to --sigma-phi-if for end-to-end "
                          "filter-trust calibration.")
    pos.add_argument("--seed-pos-offset", default=None,
                     help="Add an (E,N,U) offset in meters to the "
                          "seed/known position before filter "
                          "initialization.  Format: 'E,N,U' (commas, "
                          "metres, signed).  Used for controlled "
                          "seed-error experiments — same as Bravo's "
                          "harness flag (commit a910d2d).  Combines "
                          "with --known-pos or --seed-pos.  "
                          "Example: '--seed-pos-offset 0,0,10' offsets "
                          "the seed by +10 m vertically.")
    pos.add_argument("--no-ssr", action="store_true",
                     help="Disable SSR corrections entirely.  Engine "
                          "uses broadcast ephemeris only — no SSR "
                          "orbit/clock corrections, no SSR code "
                          "biases, no SSR phase biases.  Used for "
                          "controlled experiments isolating "
                          "correction-stream effects from the engine's "
                          "intrinsic behavior.  Same SSR-mount config "
                          "in ntrip.conf is silently ignored.")
    pos.add_argument("--no-primary-biases", action="store_true",
                     help="Diagnostic: drop bias messages (code + phase) "
                          "from the PRIMARY SSR mount, keeping its "
                          "orbit/clock.  Pair with --ssr-bias-mount / "
                          "--ssr-bias-ntrip-conf to test 'orbit/clock "
                          "from A, biases from B' cleanly.  See "
                          "docs/ssr-cross-ac-diagnostic-2026-04-25.md.")
    pos.add_argument("--no-ssr-code-bias", action="store_true",
                     help="Diagnostic: drop ALL SSR code-bias messages "
                          "(both primary and secondary mounts).  "
                          "Isolates phase-bias contribution to obs-model "
                          "bias.  Pseudoranges fall back to broadcast "
                          "TGD only.")
    pos.add_argument("--no-ssr-phase-bias", action="store_true",
                     help="Diagnostic: drop ALL SSR phase-bias messages "
                          "(both primary and secondary mounts).  "
                          "Isolates code-bias contribution.  Carrier-"
                          "phase ambiguities are absorbed into the float "
                          "state without bias correction.")
    pos.add_argument("--phase-windup", action="store_true",
                     help="Apply Wu 1993 carrier-phase wind-up "
                          "correction per SV per epoch.  Default "
                          "off.  On clean PRIDE/ABMF data the "
                          "effect is ~1-5 mm per obs-model plan "
                          "(Bravo 2026-04-24, cb6e806).  Folded "
                          "into the engine to see whether lab "
                          "dirty inputs respond differently. "
                          "See docs/obs-model-completion-plan.md "
                          "Phase 3.")
    pos.add_argument("--gmf", action="store_true",
                     help="Use Boehm 2006 GMF hydrostatic + wet "
                          "mapping functions instead of the default "
                          "1/sin(elev) tropospheric mapping.  "
                          "Default off.  On clean data at joint-best "
                          "tuning, crosses sub-meter final-H for the "
                          "first time (0.993 m vs 1.157 m without). "
                          "Default 1/sin(e) is wrong by 3 m slant "
                          "delay at 5° elev — GMF closes that and "
                          "lets low-elev observations contribute "
                          "honestly.  See "
                          "docs/obs-model-completion-plan.md Phase 4.")
    pos.add_argument("--q-pos-converged", type=float, default=None,
                     help="Override the converged-mode position process "
                          "noise (m²/s).  Default 1e-4.  Tighter values "
                          "(e.g. 1e-7, 1e-8) tell the filter that "
                          "position truly does not move between epochs "
                          "— honest for static stations, prevents "
                          "trading position state for residual "
                          "reduction along the (pos, ZTD, clk) null "
                          "direction.  Bravo's PRIDE/ABMF Q_pos sweep "
                          "(2026-04-24) showed σ_phi=1.0 + Q_pos=1e-8 "
                          "achieves 6.2× total reduction vs defaults "
                          "(mean 3D 9.30 → 1.51 m).  See "
                          "project_to_main_qpos_sweep_20260424.md.")
    pos.add_argument("--ztd-tie-sigma", type=float, default=None,
                     metavar="SIGMA_M",
                     help="Soft prior on residual ZTD as a per-epoch "
                          "rank-1 EKF pseudo-measurement (z=0, σ=SIGMA_M). "
                          "Constrains the (position-altitude, ZTD, clock) "
                          "null direction that otherwise lets the streaming "
                          "EKF wander metres on lab data despite stable "
                          "lock.  Recommended ~0.05 m (matches the IGS-"
                          "standard Q_ZTD random-walk scale and the "
                          "empirical 18 cm / 24 h ABMF wet-residual "
                          "variation).  Default disabled.  See "
                          "docs/ssr-phase-bias-step-handling.md and the "
                          "harness ``--ztd-tie`` flag (same mechanism).")
    pos.add_argument("--ztd-pwc-window-s", type=float, default=None,
                     metavar="SECONDS",
                     help="Enable PRIDE-style piece-wise constant ZTD with "
                          "N-second segment length (3600 = PWC-60).  "
                          "Within a segment ZTD process noise ≈ 0; at "
                          "segment boundaries P[ZTD] is re-inflated to "
                          "seed sigma so a fresh estimate forms.  Default "
                          "disabled (random walk).  See "
                          "project_to_main_pride_single_pass_20260426.md.")
    pos.add_argument("--outlier-mad-k", type=float, default=None,
                     metavar="K",
                     help="Per-epoch MAD-based outlier rejection threshold "
                          "in PPPFilter.  Reject obs where "
                          "|residual - median| > K · MAD, computed "
                          "separately per kind (PR vs phase).  Default off "
                          "(K=0).  Recommended K=2.0 for production — "
                          "5.7× ABMF reduction (1.826m → 0.319m) over "
                          "no-MAD baseline at Q_pos=1e-12.  Aggressive "
                          "K=0.5 hits 0.191m on clean reference data but "
                          "may over-reject on noisy lab data.  See "
                          "project_mad_outlier_rejection_landed_20260426.md.")
    pos.add_argument("--timeout", type=int, default=3600,
                     help="Bootstrap timeout in seconds (default: 3600)")
    pos.add_argument("--watchdog-threshold", type=float, default=10.0,
                     help="Position watchdog threshold in meters (default: "
                          "10.0).  Watchdog trips when PR residual RMS "
                          "exceeds max(baseline*3, baseline+threshold) for "
                          "10 consecutive epochs.  10 m corresponds to a "
                          "30 ns time error — the bound below which we "
                          "tolerate riding through transient sky/SSR "
                          "noise without re-seeding a converged filter.")

    # Serial
    serial = ap.add_argument_group("Serial")
    serial.add_argument("--serial", default=None,
                        help="Serial port for F9T (e.g. /dev/gnss-top). "
                             "Required unless provided by host config.")
    serial.add_argument("--baud", type=int, default=None,
                        help="Baud rate (default: 115200, or from host config)")
    serial.add_argument("--receiver", default=None,
                        help="Receiver model/profile: f9t, f9t-l5, f9t-l2, f10t "
                             "(default: f9t = L5; f9t-l2 = L2, TIM 2.20 only)")
    serial.add_argument("--port-type", default=None,
                        choices=["UART", "UART2", "USB", "SPI", "I2C"],
                        help="Receiver port type for UBX message routing (default: USB)")
    serial.add_argument("--measurement-rate-ms", type=int, default=None,
                        help="F9T measurement rate in ms (profile default: 1000 for i226, 2000 for E810)")
    serial.add_argument("--sfrbx-rate", type=int, default=None,
                        help="SFRBX output rate (0=disabled, 1=every epoch; profile default: 1 for serial, 0 for E810 I2C)")

    # GNSS
    gnss = ap.add_argument_group("GNSS")
    gnss.add_argument("--systems", default=None,
                      help="GNSS systems (default: gps,gal,bds).  Comma-"
                           "separated list drawn from {gps,gal,bds}.  "
                           "WARNING: single-constellation runs with smooth "
                           "precise clocks (SSR-orbit+clock or SP3+CLK) are "
                           "subject to a near-singular filter mode where "
                           "(rx_clk, ZTD, common-ambiguity) drift together "
                           "with zero residual.  At least 2 constellations "
                           "provide the separate clock state needed to break "
                           "the degeneracy.  Broadcast-only (no SSR) is not "
                           "affected — NAV page-boundary clock discontinuities "
                           "inject the rank information.  See "
                           "project_to_main_pride_gps_filter_degeneracy_20260423.")
    gnss.add_argument("--leap", type=int, default=18,
                      help="GPS-UTC leap seconds (default: 18)")
    gnss.add_argument("--tai-minus-gps", type=int, default=19,
                      help="TAI-GPS offset in seconds (default: 19)")

    # NTRIP (corrections input)
    ntrip = ap.add_argument_group("NTRIP corrections")
    ntrip.add_argument("--ntrip-conf", help="NTRIP config file (INI format)")
    ntrip.add_argument("--ntrip-caster", help="NTRIP caster hostname")
    ntrip.add_argument("--ntrip-port", type=int, default=2101)
    ntrip.add_argument("--ntrip-tls", action="store_true")
    ntrip.add_argument("--eph-mount", help="Broadcast ephemeris mountpoint")
    ntrip.add_argument("--ssr-mount", help="SSR corrections mountpoint")
    ntrip.add_argument("--ssr-caster", help="SSR caster hostname (default: same as --ntrip-caster)")
    ntrip.add_argument("--ssr-port", type=int, help="SSR caster port (default: same as --ntrip-port)")
    ntrip.add_argument("--ssr-user", help="SSR caster username (default: same as --ntrip-user)")
    ntrip.add_argument("--ssr-password", help="SSR caster password (default: same as --ntrip-password)")
    ntrip.add_argument("--ssr-ntrip-conf", default=None,
                       help="Optional separate NTRIP credentials file for "
                            "the primary SSR mount (orbit/clock/biases).")
    ntrip.add_argument("--ssr-bias-mount", default=None,
                       help="Optional secondary SSR mountpoint that "
                            "contributes PHASE BIASES ONLY.  Pair e.g. "
                            "CNES orbit/clock with WHU OSB phase biases "
                            "keyed to F9T-tracked signals.  All non-"
                            "phase-bias messages from this stream are "
                            "ignored.  See docs/ssr-mount-survey.md.")
    ntrip.add_argument("--ssr-bias-ntrip-conf", default=None,
                       help="Credentials file for --ssr-bias-mount (same "
                            "INI format as --ntrip-conf).  Falls back to "
                            "the primary SSR credentials if unset.")
    ntrip.add_argument("--ntrip-user", help="NTRIP username")
    ntrip.add_argument("--ntrip-password", help="NTRIP password")
    ntrip.add_argument("--max-broadcast-age-s", type=float, default=None,
                       help="Maximum host-monotonic age for broadcast correction state "
                            "(default: 600). Past this, the freshness gate skips EKF "
                            "updates entirely. Sigma inflation (see "
                            "--corr-staleness-ns-per-s) handles graceful degradation "
                            "well before reaching this hard limit.")
    ntrip.add_argument("--corr-staleness-ns-per-s", type=float, default=0.1,
                       help="Linear inflation rate (ns of σ per second of NTRIP "
                            "correction age) applied to Carrier and PPS+PPP error "
                            "sources. Default 0.1 puts those sources past the "
                            "PPS+qErr 3 ns floor at ~30 s of NTRIP staleness, so "
                            "the source competition hands off automatically.")
    ntrip.add_argument("--require-ssr", action="store_true", default=None,
                       help="Require fresh SSR state before EKF updates")
    ntrip.add_argument("--max-ssr-age-s", type=float, default=None,
                       help="Maximum host-monotonic age for SSR state when --require-ssr is set (default: 30)")
    ntrip.add_argument("--min-broadcast-confidence", type=float, default=None,
                       help="Minimum acceptable confidence for broadcast correction timing")
    ntrip.add_argument("--min-ssr-confidence", type=float, default=None,
                       help="Minimum acceptable confidence for SSR correction timing")

    # PHC servo (optional)
    servo = ap.add_argument_group("PHC servo (optional)")
    servo.add_argument("--ptp-profile",
                       help="PTP NIC profile for default PHC/pin/channel settings")
    servo.add_argument("--device-config", default="config/receivers.toml",
                       help="Device/profile config TOML (default: config/receivers.toml)")
    servo.add_argument("--servo", default=None,
                       help="PTP device for PHC servo (e.g. /dev/ptp0)")
    servo.add_argument("--no-do", action="store_true",
                       help="Position-only mode: no DO, no servo, no PPS. "
                            "Overrides --servo and host config ptp_dev. "
                            "Useful for cold/warm position reproducibility testing.")
    servo.add_argument("--pps-pin", type=int, default=None,
                       help="PTP pin index for PPS input (profile/default if omitted)")
    servo.add_argument("--extts-channel", type=int, default=None,
                       help="PTP EXTS channel for PPS input (profile/default if omitted)")
    servo.add_argument("--program-pin", action="store_true",
                       help="Explicitly program PTP pin function before enabling EXTS")
    servo.add_argument("--phc-timescale", choices=["gps", "utc", "tai"], default=None,
                       help="Target PHC timescale for PPS alignment (profile/default if omitted)")
    servo.add_argument("--min-correlation-confidence", type=float, default=None,
                       help="Minimum acceptable confidence for observation/PPS correlation")
    servo.add_argument("--max-correlation-window-s", type=float, default=None,
                       help="Max recv_mono delta for obs/PPS correlation (default: 11s, increase for high-latency transports like E810 I2C)")
    servo.add_argument("--kalman-servo", action="store_true",
                       help="Use 2-state Kalman filter + LQR servo instead of PI. "
                            "Optimal pull-in (no overshoot) and noise-matched "
                            "steady-state tracking.  Noise parameters from "
                            "DO characterization + TICC+qErr measurement.")
    servo.add_argument("--do-freq-est", action="store_true",
                       help="Use 4-state DOFreqEst (architecture vision). "
                            "Fuses TICC+qErr with PPP dt_rx in a single filter "
                            "that models both oscillators (TCXO + DO).  Supersedes "
                            "--kalman-servo when both are given.")
    servo.add_argument("--kalman-q-weight", type=float, default=1.0,
                       help="Kalman process noise Q scale (>1 = more aggressive "
                            "tracking, <1 = smoother output)")
    servo.add_argument("--kalman-r-weight", type=float, default=1.0,
                       help="Kalman measurement noise R scale (>1 = trust "
                            "measurements less, <1 = trust them more)")
    servo.add_argument("--kalman-dead-zone", type=float, default=0.0,
                       help="Minimum adjfine change (ppb) to actually apply. "
                            "Below this, hold previous value to reduce noise. "
                            "Suggested: 0.5 (below DO floor)")
    servo.add_argument("--kalman-sigma-freq", type=float, default=0.01,
                       help="DO frequency random walk (ppb/epoch). Lower = "
                            "more stable frequency estimate, less wander. "
                            "Default 0.01 from ADEV characterization.")
    servo.add_argument("--kalman-sigma-tcxo-freq", type=float, default=0.1,
                       help="rx TCXO frequency random walk (ppb/epoch).  "
                            "Default 0.1 — empirically better than the 0.5 "
                            "value bravo's variance-matching math suggested.  "
                            "Tighter Q[1,1] means the filter SMOOTHS rx_TCXO "
                            "wander rather than chasing it; chasing it via "
                            "larger Q[1,1] degrades long-τ TDEV (PiFace "
                            "2026-05-08: τ=100s 1.117→1.758 ns at Q=0.5).  "
                            "Tunable; per-host value belongs in TOML "
                            "host-config eventually.")
    servo.add_argument("--track-kp", type=float, default=0.3,
                       help="PI servo Kp gain (default: 0.3)")
    servo.add_argument("--track-ki", type=float, default=0.1,
                       help="PI servo Ki gain (default: 0.1)")
    servo.add_argument("--gain-ref-sigma", type=float, default=2.0,
                       help="Reference confidence for gain scale=1.0 (default: 2.0)")
    servo.add_argument("--gain-min-scale", type=float, default=0.1,
                       help="Minimum gain scale in tracking (default: 0.1)")
    servo.add_argument("--gain-max-scale", type=float, default=1.0,
                       help="Maximum gain scale in tracking before convergence boost (default: 1.0)")
    servo.add_argument("--converge-error-ns", type=float, default=500.0,
                       help="Boost gains above this tracking error magnitude (default: 500)")
    servo.add_argument("--converge-min-scale", type=float, default=2.0,
                       help="Minimum gain scale while converging (default: 2.0)")
    servo.add_argument("--discipline-interval", type=int, default=1,
                       help="Fixed discipline interval (default: 1)")
    servo.add_argument("--adaptive-interval", action="store_true", default=True,
                       help="Enable adaptive discipline interval (default: on)")
    servo.add_argument("--max-interval", type=int, default=120,
                       help="Maximum discipline interval (default: 120)")
    servo.add_argument("--min-interval", type=int, default=1,
                       help="Minimum discipline interval (default: 1)")
    servo.add_argument("--scheduler-converge-threshold-ns", type=float, default=100.0,
                       help="Scheduler settled threshold in ns (default: 100)")
    servo.add_argument("--scheduler-settle-window", type=int, default=10,
                       help="Consecutive corrections required to declare settled (default: 10)")
    servo.add_argument("--scheduler-unconverge-factor", type=float, default=5.0,
                       help="Re-enter convergence when error exceeds threshold*f (default: 5.0)")
    servo.add_argument("--servo-log", default=None,
                       help="CSV log file for servo data")
    servo.add_argument("--track-max-ppb", type=float, default=None,
                       help="Clamp tracking corrections to this ppb magnitude")
    servo.add_argument("--track-outlier-ns", type=float, default=None,
                       help="Skip tracking updates above this error magnitude when settled")
    servo.add_argument("--track-restep-ns", type=float, default=None,
                       help="Re-enter step if |tracking error| exceeds this for 3 epochs")
    servo.add_argument("--phase-step-bias-ns", type=float, default=None,
                       help="Per-host bias compensation applied to PHC phase steps")
    servo.add_argument("--obs-idle-timeout-s", type=float, default=None,
                       help="Log and enter safe holdover if no observation epochs arrive for this long")
    servo.add_argument("--queue-depth-threshold", type=int, default=5,
                       help="Warn when any pipeline queue exceeds this depth (epochs, default: 5)")
    servo.add_argument("--queue-depth-dump", action="store_true",
                       help="Dump full diagnostic state when queue depth threshold is breached")
    servo.add_argument("--carrier-max-sigma-ns", type=float, default=None,
                       help="Maximum PPP sigma allowed to compete as a servo source")
    servo.add_argument("--pmc", default=None, metavar="UDS_PATH",
                       help="ptp4l UDS path for clockClass management (e.g. /var/run/ptp4l)")
    servo.add_argument("--pmc-domain", type=int, default=0,
                       help="PTP domain number for pmc messages (must match ptp4l config, default: 0)")
    servo.add_argument("--pid-file", default=None,
                       help="Write engine PID here for external test control")
    servo.add_argument("--freerun", action="store_true",
                       help="Run full pipeline without steering PHC. "
                            "Logs what the servo would do but never calls adjfine. "
                            "For characterizing EXTTS precision and oscillator stability.")
    servo.add_argument("--freerun-max-error-ns", type=float, default=None,
                       help="Auto-stop freerun when |pps_err_extts_ns| exceeds this "
                            "(default: 100000 for OCXO, 500000 for TCXO)")
    servo.add_argument("--no-qerr", action="store_true",
                       help="Disable qErr correction (PPS-only discipline)")
    servo.add_argument("--pps-corr", choices=["timtp", "ppp"], default=None,
                       help="PPS correction source: 'timtp' (default, TIM-TP qErr) "
                            "or 'ppp' (smoothed dt_rx drift model). "
                            "qErr corrects PPS quantization (discrete, ±4 ns). "
                            "PPP correction models the rx TCXO drift from dt_rx "
                            "(continuous, ~0.1 ns) — different physics, not qErr.")
    servo.add_argument("--no-ppp", action="store_true",
                       help="Disable PPP carrier-phase correction "
                            "(PPS+qErr only, no PPS+PPP source)")
    servo.add_argument("--no-carrier", action="store_true",
                       help="Disable PPP Carrier Phase servo drive "
                            "(Carrier source disabled, PPS+PPP still available)")
    servo.add_argument("--qerr-freq-sigma-floor", type=float, default=1.0,
                       help="Lower bound on Arm 2's qerr_freq_sigma_ppb in "
                            "ppb (default 1.0).  Captures real rx_TCXO "
                            "frequency wander within the slope window that "
                            "OLS-derived sigma misses.  Smaller = filter "
                            "trusts Arm 2 more.  See "
                            "docs/dofreq-est-measurement-ladder.md §Arm 2.")
    servo.add_argument("--no-qerr-arm", action="store_true",
                       help="Disable DOFreqEst Arm 2 (qErr slope → x[1] = "
                            "rx TCXO frequency).  When set, the EKF runs "
                            "without the qErr-as-frequency-slope arm.  "
                            "Useful for ablation runs comparing PPP-only "
                            "vs PPP+qErr-Arm2 servo behavior.  See "
                            "docs/dofreq-est-measurement-ladder.md §Arm 2.")
    servo.add_argument("--no-extint", action="store_true",
                       help="Disable DOFreqEst Arm 3 (TIM-TM2 → x[2]).  "
                            "When set, the engine does not allocate a "
                            "TimTm2Store, does not enable TIM-TM2 emission "
                            "on the F9T, and the EKF arm 3 stays None — "
                            "useful for ablation runs comparing PPP-only "
                            "to PPP+EXTINT performance.  See "
                            "docs/dofreq-est-measurement-ladder.md.")
    servo.add_argument("--no-ticc", action="store_true",
                       help="Disable DOFreqEst Arm 4 (TICC chA-chB → "
                            "couple x[0] and x[2]).  When set, the EKF "
                            "arm 4 stays None even if --ticc-port is "
                            "configured — TICC measurements are still "
                            "logged and used for diagnostics, but not "
                            "fed to the servo.  Used for ablation runs "
                            "comparing TIM-TM2-only against TICC+TIM-TM2 "
                            "performance.")
    servo.add_argument("--do-char-file", default="data/do_characterization.json",
                       help="Path to DO characterization JSON (read at startup)")
    servo.add_argument("--do-label", default=None,
                       help="Label for the disciplined oscillator (overrides auto-detected PHC "
                            "MAC as the DO unique_id in state persistence). Required for external "
                            "DOs (VCOCXO, ClockMatrix) that aren't bundled inside a PHC.")
    servo.add_argument("--do-type", default=None,
                       choices=["phc", "vcocxo", "clockmatrix"],
                       help="Type of disciplined oscillator: phc (default, NIC crystal via "
                            "adjfine), vcocxo (external OCXO/TCXO via DAC), clockmatrix "
                            "(Renesas 8A34002 via I2C FCW)")
    servo.add_argument("--dac-bus", type=int, default=None,
                       help="I2C bus for DAC-driven VCOCXO (e.g. 1 for /dev/i2c-1)")
    servo.add_argument("--dac-addr", default=None,
                       help="I2C address for DAC (e.g. 0x60)")
    servo.add_argument("--dac-bits", type=int, default=None,
                       help="DAC resolution in bits (default: 12 for MCP4725)")
    servo.add_argument("--dac-center-code", type=int, default=None,
                       help="DAC code for nominal frequency (default: midscale)")
    servo.add_argument("--dac-ppb-per-code", type=float, default=None,
                       help="Tuning sensitivity in ppb per DAC LSB (must be characterized)")
    servo.add_argument("--dac-max-ppb", type=float, default=None,
                       help="Maximum frequency adjustment in ppb (default: computed from range)")
    servo.add_argument("--dac-type", default=None,
                       choices=["mcp4725", "ad5693r", "generic"],
                       help="DAC chip type (default: mcp4725)")
    servo.add_argument("--dac-gain", type=int, default=None,
                       choices=[0, 1],
                       help="AD5693R GAIN bit: 0=1× output (0..Vref, default), "
                            "1=2× output (0..2×Vref).  Required for OCXOs that "
                            "need >Vref Vctrl to compensate freerun offset; "
                            "ignored for other DAC types.  See I-000711.")

    # DO bootstrap (absorbed from phc_bootstrap.py)
    boot = ap.add_argument_group("DO bootstrap (automatic when --servo)")
    boot.add_argument("--pps-out-pin", type=int, default=-1,
                      help="SDP pin for PPS OUT (PEROUT), -1 = none")
    boot.add_argument("--pps-out-channel", type=int, default=0,
                      help="PEROUT channel for PPS OUT (default: 0)")
    boot.add_argument("--phc-step-threshold-ns", type=int, default=10000,
                      help="Skip phase step if error already within this (default: 10000)")
    boot.add_argument("--phc-settime-lag-ns", type=int, default=0,
                      help="Mean clock_settime-to-PHC lag in ns (default: 0)")
    boot.add_argument("--phc-optimal-stop-limit-s", type=float, default=1.0,
                      help="Phase step optimal stopping budget in seconds (default: 1.0)")
    boot.add_argument("--glide-zeta", type=float, default=0.7,
                      help="Target damping ratio for servo glide (default: 0.7)")
    boot.add_argument("--no-glide", action="store_true",
                      help="Skip glide slope — set adjfine to base frequency only")
    boot.add_argument("--bootstrap-epochs", type=int, default=10,
                      help="Filter epochs before PHC evaluation (default: 10)")
    boot.add_argument("--freq-tolerance-ppb", type=float, default=10.0,
                      help="Frequency sanity threshold in ppb (default: 10.0)")
    boot.add_argument("--skip-bootstrap", action="store_true",
                      help="Skip DO bootstrap even when --servo is set")
    boot.add_argument("--warm-start-if-fresh", action="store_true",
                      help="Skip DO bootstrap when state/dos/<uid>.json has a "
                           "recent last_known_freq_offset_ppb (saves 15-30s "
                           "on watchdog re-bootstrap).  Gate: mtime age <= "
                           "--warm-start-max-age-s AND |freq| <= "
                           "--warm-start-max-ppb.  Falls back to cold-start "
                           "if the gate fails.  See dacWarmStart-main.")
    boot.add_argument("--warm-start-max-age-s", type=float, default=86400.0,
                      help="Max DO-state age (s) for warm-start gate "
                           "(default: 86400 = 24h)")
    boot.add_argument("--warm-start-max-ppb", type=float, default=500.0,
                      help="Physical sanity envelope for saved freq "
                           "(default: 500 ppb)")
    boot.add_argument("--tadd-gpio", type=int, default=None,
                      help="BCM GPIO pin for TADD-2 Mini ARM sync (enables TADD ARM "
                           "during bootstrap for external DOs with a divider)")
    boot.add_argument("--tadd-hold-s", type=float, default=1.1,
                      help="TADD ARM hold time in seconds (default: 1.1, spec requires >1s)")
    boot.add_argument("--divider-step-threshold-us", type=float, default=5.0,
                      help="Phase offset (µs) above which warm-start steps the "
                           "divider via TADD ARM instead of slewing.  At |phase| "
                           "≤ threshold the EKF closes the residual by slewing; "
                           "above threshold a step is dramatically faster "
                           "(≤400 ns post-ARM vs ~1.2 µs/s slew at max actuator "
                           "authority).  Default: 5.0 (5 µs).  See "
                           "project_warm_start_sign_blind_spot memory.")
    boot.add_argument("--no-divider-step", action="store_true",
                      help="Disable warm-start's phase-conditional TADD ARM.  "
                           "Use when divider wiring is broken or you want pure "
                           "slewing for diagnostic purposes.")

    ticc = ap.add_argument_group("TICC experimental input (optional)")
    ticc.add_argument("--ticc-port", default=None,
                      help="TICC serial port for experimental measurement/servo input")
    ticc.add_argument("--ticc-log", default=None,
                      help="Optional raw TICC CSV log path for lab analysis. "
                           "Each row records host_monotonic when the line "
                           "arrived from the TICC over USB, plus ref_sec/"
                           "ref_ps/channel.  Pair with --qerr-log to do "
                           "post-hoc qErr correction by index-matching on "
                           "CLOCK_MONOTONIC.")
    ticc.add_argument("--noise-buffer-csv", default=None,
                      help="Optional per-epoch noise-buffer CSV export "
                           "(I-162848 Step 1).  Each row records "
                           "(mono, residual_ns, channel) from the "
                           "InBandNoiseEstimator's detrended phase "
                           "samples — channel='gap' for no-correction "
                           "epochs (free-running DO observation), "
                           "'correction' for correction epochs.  Used "
                           "for offline gap-PSD analysis vs freerun "
                           "captures in the overlap band; PSD mismatch "
                           "indicates the detrending model "
                           "('expected drift = adjfine·dt') is missing "
                           "DO frequency wander beyond the linear ramp.")
    ticc.add_argument("--slip-log", default=None,
                      help="Optional cycle-slip CSV log.  One row per "
                           "SlipEvent captures epoch, sv, reasons "
                           "(ubx_locktime_drop|arc_gap|gf_jump|mw_jump), "
                           "confidence, lock_ms, cno, elev, gap_s, and "
                           "per-detector magnitudes.  Used for post-hoc "
                           "antenna/mount quality reporting.")
    ticc.add_argument("--slip-rate-limit-s", type=float, default=0.0,
                      metavar="SECONDS",
                      help="Per-SV slip-event rate limiter (seconds).  "
                           "When > 0, suppress slip events that fire "
                           "within N s of the previous emitted event "
                           "for the same SV — treats them as continuation "
                           "of one slip storm rather than separate events. "
                           "Prevents per-SV state machine churn during "
                           "multi-epoch disturbances (E29-style 4-10× "
                           "consecutive firings cycle FLOAT→ANCHORING "
                           "repeatedly, blocking convergence).  0 = "
                           "disabled (legacy).  Bravo's day0426 "
                           "trajectory analysis recommends 30-300 s; "
                           "see project_to_main_trajectory_analysis_"
                           "day0426_20260426.md.")
    ticc.add_argument("--qerr-log", default=None,
                      help="Optional raw qErr CSV log path.  Each row "
                           "captures one TIM-TP message from the F9T with "
                           "(host_timestamp, host_monotonic, qerr_for_extts_pps_ns, "
                           "tow_ms, qerr_invalid) — host_monotonic is "
                           "CLOCK_MONOTONIC at the moment the message was "
                           "parsed, the same clock the engine's "
                           "match_pps_mono uses internally.  Independent "
                           "of servo state; lets post-processing redo the "
                           "qErr ↔ TICC chB matching the engine does in "
                           "real time, without sawtooth dewrap heuristics.")
    ticc.add_argument("--extint-log", default=None,
                      help="Optional raw TIM-TM2 CSV log path for the "
                           "gnss-phase-experiment.  Each row captures "
                           "one TIM-TM2 message from the F9T with "
                           "(host_timestamp, host_monotonic, wn, tow_ms, "
                           "tow_sub_ms_ns, acc_est_ns, phase_residual_ns, "
                           "count, flags).  Pair with --ticc-log to "
                           "post-hoc compare TIM-TM2 (DOFreqEst Arm 3) "
                           "against TICC chA-chB (Arm 4) and quantify "
                           "their static offset + dynamic noise — the "
                           "ablation matrix from "
                           "docs/dofreq-est-measurement-ladder.md.")
    ticc.add_argument("--ticc-baud", type=int, default=115200,
                      help="TICC baud rate (default: 115200)")
    ticc.add_argument("--ticc-phc-channel", choices=["chA", "chB"], default="chA",
                      help="TICC channel carrying disciplined PHC PPS OUT (default: chA)")
    ticc.add_argument("--ticc-ref-channel", choices=["chA", "chB"], default="chB",
                      help="TICC channel carrying raw reference PPS (default: chB)")
    ticc.add_argument("--ticc-max-age-s", type=float, default=2.0,
                      help="Maximum age for a paired TICC measurement to be used")
    ticc.add_argument("--ticc-target-ns", type=float, default=0.0,
                      help="Target chPHC-chREF offset in ns for TICC-driven "
                           "servo mode.  Default 0 means actively pull "
                           "PEROUT-vs-F9T_PPS to zero (so the local TICC chA "
                           "rising edge marks GPS top-of-second, modulo F9T "
                           "qErr).  See docs/peroutVS-phc-time.md for the "
                           "implications: the PHC's internal counter remains "
                           "offset from this edge by PHC→PEROUT→cable "
                           "propagation latency, which is per-host fixed.  "
                           "To preserve the legacy auto-capture-of-initial-"
                           "diff behaviour (servo only corrects drift "
                           "around the bootstrap residual, not the residual "
                           "itself), pass --ticc-target-auto.")
    ticc.add_argument("--ticc-target-auto", action="store_true", default=False,
                      help="At the first epoch, capture the TICC chA-chB "
                           "diff and use that as the servo setpoint.  "
                           "Legacy behaviour, retained for diagnostic runs "
                           "where you want to track drift around an arbitrary "
                           "starting point rather than pulling chA-chB to zero.")
    ticc.add_argument("--ticc-confidence-ns", type=float, default=3.0,
                      help="Assumed confidence of TICC differential error when driving servo")
    # --ticc-drive removed: TICC competes as a source whenever --ticc-port
    # is configured.  No separate flag needed.

    # NTRIP caster output (optional, future)
    caster = ap.add_argument_group("NTRIP caster output (optional)")
    caster.add_argument("--caster", default=None,
                        help="NTRIP caster listen address (e.g. :2102) [not yet implemented]")

    # Output
    out = ap.add_argument_group("Output")
    out.add_argument("--out", help="Solution CSV output file")
    out.add_argument("--rinex-out", default=None,
                     help="Optional path to write a RINEX 3.04 OBS file "
                          "of the observations the engine processed.  "
                          "For offline cross-engine verification (PRIDE "
                          "PPP-AR, RTKLIB) — see "
                          "docs/ssr-cross-ac-diagnostic-2026-04-25.md.  "
                          "Off by default (no overhead).  "
                          "Path supports {date} (UTC YYYYDDD) and {host} "
                          "interpolation; using {date} enables UTC-midnight "
                          "rotation (one file per day) for I-013307's daily "
                          "PRIDE pipeline.")
    out.add_argument("--rinex-decimate-s", type=float, default=None,
                     help="Decimate RINEX output to this interval in seconds "
                          "(engine continues to run its PPP filter at full "
                          "rate; only the writer skips epochs).  Standard "
                          "PRIDE / RTKLIB input rate is 30 s.  Default: no "
                          "decimation — every engine epoch written.")
    out.add_argument("--duration", type=int, default=None,
                     help="Run duration in seconds (0 = unlimited)")
    out.add_argument("--gate-stats", help="Optional JSON output for strict sink gate statistics")
    out.add_argument("--log-out", default=None,
                     help="Append engine log messages to this file in "
                          "addition to stderr.  Append-mode and "
                          "independent of any shell `> ...log` redirect, "
                          "so the log survives wrapper-driven engine "
                          "relaunches without truncation.  Matches the "
                          "append-mode behavior of --ticc-log / "
                          "--extint-log / --qerr-log from "
                          "recoveryRetry-main.  Default: stderr only "
                          "(operator's shell redirect captures it).")
    out.add_argument("-v", "--verbose", action="store_true")

    peer = ap.add_argument_group("Peer state sharing")
    peer.add_argument(
        "--peer-bus", default="none",
        help=(
            "Publish state to a peer bus for fleet monitoring + "
            "(Phase 2b) peer integer-fix adoption.  Values: "
            "'none' (default, no publishing); "
            "'udp-multicast' (default LAN group 239.18.8.13:12468); "
            "'udp-multicast:GROUP:PORT' (override).  "
            "See docs/peer-state-sharing.md."
        ),
    )
    peer.add_argument(
        "--peer-antenna-ref", default="",
        metavar="NAME",
        help=(
            "Antenna identifier (e.g. 'UFO1') published in this "
            "host's heartbeat.  Peers with matching antenna_ref get "
            "cross-antenna fleet comparison.  Empty = don't claim."
        ),
    )
    peer.add_argument(
        "--peer-site-ref", default="",
        metavar="NAME",
        help=(
            "Site identifier (e.g. 'DuPage') published in this "
            "host's heartbeat.  Peers with matching site_ref form "
            "the shared-atmosphere cohort for ZTD consensus "
            "checks (see docs/fleet-consensus-monitors.md).  "
            "Empty = don't claim a site."
        ),
    )
    peer.add_argument(
        "--peer-host", default=None,
        metavar="NAME",
        help=(
            "Hostname published on the peer bus.  Defaults to "
            "socket.gethostname().  Must be unique in the fleet."
        ),
    )

    args = ap.parse_args()

    # Translate --ar-mode to internal `wl_only` flag.  --ar-mode is the
    # user-facing knob (I-125649-main); `wl_only` is the legacy internal
    # name that flows through the rest of the engine + sv_state +
    # NarrowLaneResolver + AntPosEst state machine.  Once --ar-mode lands
    # the same default everywhere, the internal `wl_only` rename is a
    # mechanical follow-up (out of scope for this commit).
    if getattr(args, "wl_only_legacy", False):
        # Deprecated --wl-only alias still used.  Treat as --ar-mode wl
        # (the only behavior --wl-only ever expressed).  No warning yet
        # — wait until the next commit cycle to surface deprecation.
        args.ar_mode = "wl"
    args.wl_only = (args.ar_mode == "wl")

    _apply_host_config(args)
    # Apply defaults for args that are None after CLI + host config.
    # These were made nullable so host config can override them.
    if args.baud is None:
        args.baud = 115200
    if args.receiver is None:
        args.receiver = "f9t"
    if args.port_type is None:
        args.port_type = "USB"
    if args.systems is None:
        args.systems = "gps,gal,bds"

    # σ_phi / σ_pr overrides — set the module-level globals in
    # solve_ppp before any PPPFilter is constructed.  Cherry-picked
    # from harness commit 43e25d7; same consumer hooks.  See Bravo's
    # ABMF sweep memo for the diagnostic motivation.  No log here —
    # logging.basicConfig hasn't fired yet at this point in main();
    # the visible confirmation log line lives after basicConfig below.
    if args.sigma_phi_if is not None:
        import solve_ppp as _spp
        _spp._SIGMA_PHI_IF_OVERRIDE = float(args.sigma_phi_if)
    if args.sigma_pr_if is not None:
        import solve_ppp as _spp
        _spp._SIGMA_P_IF_OVERRIDE = float(args.sigma_pr_if)
    if args.q_pos_converged is not None:
        # PPPFilter reads its Q_POS_CONVERGED class attribute lazily
        # in predict() (commit 3aa2358).  Setting at class level
        # before any instance is constructed propagates to every
        # filter (bootstrap + AntPosEstThread fresh).
        from solve_ppp import PPPFilter as _PF
        _PF.Q_POS_CONVERGED = float(args.q_pos_converged)
    if args.outlier_mad_k is not None:
        # PPPFilter reads OUTLIER_MAD_K lazily inside _compute_H_z_at.
        # Same propagation pattern as Q_POS_CONVERGED.
        from solve_ppp import PPPFilter as _PF
        _PF.OUTLIER_MAD_K = float(args.outlier_mad_k)
    if args.ztd_pwc_window_s is not None:
        from solve_ppp import PPPFilter as _PF
        _PF.ZTD_PWC_WINDOW_S = float(args.ztd_pwc_window_s)
    if args.ar_elev_mask is None:
        args.ar_elev_mask = 25.0
    if args.do_type is None:
        args.do_type = "phc"
    if args.dac_bits is None:
        args.dac_bits = 12
    if args.dac_type is None:
        args.dac_type = "mcp4725"
    apply_ptp_profile(args)
    # --no-do overrides --servo (including the profile default
    # restored by apply_ptp_profile above).  Must happen AFTER
    # apply_ptp_profile — otherwise the profile silently re-
    # populates args.servo to the receivers.toml default (e.g.
    # "/dev/ptp0" for the i226 profile), which then crashes the
    # engine at PPS-open time on any host whose actual PHC lives
    # at /dev/ptp_i226 (the udev symlink).  See
    # project_queued_ptp_dev_config_threading_20260422.md.
    if args.no_do:
        args.servo = None
    # TICC competes as a source whenever --ticc-port is configured.
    # No separate promotion flag needed.
    if args.pps_pin is None:
        args.pps_pin = 1
    if args.extts_channel is None:
        args.extts_channel = 0
    if args.phc_timescale is None:
        args.phc_timescale = "tai"
    if args.max_broadcast_age_s is None:
        # Generous so the sigma-inflation cascade has room to run.
        # Broadcast ephemeris is good for hours; the gate's job is a
        # last-resort safety, not graceful degradation.
        args.max_broadcast_age_s = 600.0
    if args.require_ssr is None:
        args.require_ssr = False
    if args.max_ssr_age_s is None:
        args.max_ssr_age_s = 30.0
    if args.min_correlation_confidence is None:
        args.min_correlation_confidence = 0.5
    if getattr(args, 'measurement_rate_ms', None) is None:
        _base = os.path.basename(args.serial)
        if _base.startswith("gnss") and _base[4:].isdigit():
            args.measurement_rate_ms = 2000  # kernel GNSS I2C: 0.5 Hz for lossless
        else:
            args.measurement_rate_ms = 1000
    if getattr(args, 'sfrbx_rate', None) is None:
        _base = os.path.basename(args.serial)
        if _base.startswith("gnss") and _base[4:].isdigit():
            args.sfrbx_rate = 0  # kernel GNSS I2C: disable SFRBX
        else:
            args.sfrbx_rate = 1
    if args.track_restep_ns is None:
        args.track_restep_ns = 100_000.0
    if args.phase_step_bias_ns is None:
        args.phase_step_bias_ns = 0.0
    if args.track_outlier_ns is None:
        args.track_outlier_ns = 500.0
    if args.obs_idle_timeout_s is None:
        args.obs_idle_timeout_s = 15.0
    if args.carrier_max_sigma_ns is None:
        args.carrier_max_sigma_ns = 50.0
    if args.min_broadcast_confidence is None:
        args.min_broadcast_confidence = 0.0
    if args.min_ssr_confidence is None:
        args.min_ssr_confidence = 0.0
    if args.freerun and args.freerun_max_error_ns is None:
        # Default auto-stop threshold: 100 µs for OCXO, 500 µs for TCXO.
        # Heuristic: if bootstrap adjfine is > 1000 ppb, likely a TCXO.
        args.freerun_max_error_ns = 500_000.0  # conservative default

    # Logging: always to stderr (inherits the wrapper's redirect).
    # When --log-out PATH is provided, also append to PATH directly.
    # The append-mode FileHandler is independent of the shell redirect,
    # so it survives wrapper-driven engine relaunches with no truncation
    # — fixes the overnight-log-loss observed 2026-05-11/12 on MadHat.
    _log_handlers = [logging.StreamHandler(sys.stderr)]
    if getattr(args, 'log_out', None):
        try:
            _fh = logging.FileHandler(args.log_out, mode='a', encoding='utf-8')
            _log_handlers.append(_fh)
        except OSError as _exc:
            # Best effort — fall back to stderr only.  Engine bootstrap
            # shouldn't fail because the log file is unwritable.
            print(f"WARNING: --log-out {args.log_out!r} open failed: {_exc}",
                  file=sys.stderr)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        handlers=_log_handlers,
    )

    # Log the git hash (and branch + dirty flag) at startup so post-hoc
    # analysis of any engine log can be matched against an exact commit
    # without grepping the host's working tree.  Especially valuable
    # after a wrapper-driven re-launch (recoveryRetry-main) since the
    # working tree could have been pulled between runs.  Best-effort:
    # silent fallback if git isn't available or the install isn't a
    # git checkout.
    try:
        import subprocess
        _script_dir = os.path.dirname(os.path.abspath(__file__))
        _repo_root = os.path.dirname(_script_dir)
        _git_kw = dict(cwd=_repo_root, capture_output=True, text=True, timeout=2)
        _sha = subprocess.run(
            ["git", "rev-parse", "--short=12", "HEAD"], **_git_kw)
        _branch = subprocess.run(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"], **_git_kw)
        _dirty = subprocess.run(
            ["git", "status", "--porcelain"], **_git_kw)
        if _sha.returncode == 0:
            sha = _sha.stdout.strip() or "unknown"
            branch = _branch.stdout.strip() if _branch.returncode == 0 else "?"
            dirty = "dirty" if (_dirty.returncode == 0
                                and _dirty.stdout.strip()) else "clean"
            log.info("peppar-fix-engine git=%s branch=%s tree=%s",
                     sha, branch, dirty)
    except Exception:
        pass  # Best-effort; absence of this line is not fatal.

    if getattr(args, '_host_config_path', None):
        log.info("Host config: %s", args._host_config_path)

    # AR mode (I-125649-main).  'wl' is the production default; 'full'
    # is research-only.  Visible in startup log so post-hoc analysis can
    # tell which AR pipeline was active without grepping CLI args.
    log.info("AR mode: %s (%s)", args.ar_mode,
             "WL only — NL/LAMBDA disabled" if args.wl_only
             else "WL + NL — full LAMBDA pipeline")

    # σ_phi / σ_pr override visibility (the override itself fires
    # earlier in main(), before basicConfig).
    if args.sigma_phi_if is not None:
        log.info(f"σ_phi_IF override: {args.sigma_phi_if:g} m "
                 f"(default 0.03 m)")
    if args.sigma_pr_if is not None:
        log.info(f"σ_pr_IF override: {args.sigma_pr_if:g} m "
                 f"(default 3.0 m)")
    if args.q_pos_converged is not None:
        log.info(f"Q_pos_converged override: {args.q_pos_converged:g} "
                 f"(default 1e-4)")
    if args.outlier_mad_k is not None:
        log.info(f"Outlier MAD K override: {args.outlier_mad_k:g} "
                 f"(default 0 = off; recommended 2.0)")
    if args.ztd_pwc_window_s is not None:
        log.info(f"ZTD PWC window: {args.ztd_pwc_window_s:g} s "
                 f"(default RW; PRIDE-style PWC-60 = 3600)")
    if getattr(args, "phase_windup", False):
        log.info("Phase wind-up correction: enabled (Wu 1993)")
    if getattr(args, "gmf", False):
        log.info("GMF tropospheric mapping: enabled (Boehm 2006)")

    # Peer-bus initialization (optional).  Publishes no-op until the
    # first call site fires; stays no-op for entire run when
    # --peer-bus is 'none' (default).  Intentionally early in main()
    # so ``peer_publisher.publish_*`` calls anywhere downstream see
    # a properly initialised bus.
    import socket as _s
    _peer_host = getattr(args, 'peer_host', None) or _s.gethostname()
    peer_publisher.initialize(
        getattr(args, 'peer_bus', 'none') or 'none',
        host=_peer_host,
        antenna_ref=getattr(args, 'peer_antenna_ref', '') or '',
        site_ref=getattr(args, 'peer_site_ref', '') or '',
        version="peppar-fix-engine",
    )
    # Peer subscriber piggybacks on the bus that peer_publisher
    # just opened.  Wires up callbacks for peer position + ztd so
    # the engine has a view of the fleet's state for consensus
    # monitors (Part 2 of docs/fleet-consensus-monitors.md).  No-op
    # when peer_publisher didn't open a bus.  Local antenna_ref +
    # site_ref are stored on the subscriber so ``cohort_*`` helpers
    # can be called from anywhere without re-threading config.
    import peer_subscriber
    peer_subscriber.initialize(
        antenna_ref=getattr(args, 'peer_antenna_ref', '') or '',
        site_ref=getattr(args, 'peer_site_ref', '') or '',
    )

    if not args.serial:
        log.error("--serial is required (via CLI or host config)")
        sys.exit(1)

    if args.caster:
        log.warning(f"NTRIP caster output ({args.caster}) not yet implemented")

    # Ensure state directories exist
    for d in ("state/receivers", "state/dos", "state/phcs",
              "state/timestampers", "data"):
        os.makedirs(d, exist_ok=True)

    sys.exit(run(args))


if __name__ == "__main__":
    main()

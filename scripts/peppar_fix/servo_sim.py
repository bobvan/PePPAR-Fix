"""Closed-loop DO/GNSS servo simulator (inspired by SatPulse 0.2).

A *closed-loop*, not a replay.  Unlike ``validate_ocxo_gate_phase4.py``
(which feeds recorded ``innov_ticc`` back through a fresh filter and
therefore cannot show loop dynamics), this simulator carries a
ground-truth oscillator state whose emitted measurements RESPOND to the
servo's commanded ``adjfine`` each epoch.  That feedback is the only way
to test acquisition, gate over-rejection, ringing, and coast dynamics
before touching lab hardware.

Faithfulness, not novelty: the sim drives the REAL
:class:`peppar_fix.do_freq_est.DOFreqEst` (its EKF, its six measurement
arms, its chi² gate) and the REAL
:class:`peppar_fix.ocxo_trusted_gate.OcxoTrustedGate`.  The only new code
here is the *plant* (truth-state evolution + measurement emission) and the
loop that wires plant ↔ servo together.  If a design change in the servo
breaks something, the sim breaks the same way the hardware would.

Sign conventions match DOFreqEst exactly:

    φ_do = DO "lateness" vs GPS (ns); positive = DO is late.
    adjfine speeds the DO up, so a correction REDUCES lateness:
        φ_do += -(f_do + adjfine) · dt           (truth, this module)
        φ_do -= (f_do + adjfine) · dt            (DOFreqEst.predict)
    f_do = DO crystal frequency drift (ppb) = -(steady-state adjfine).

The TICC measurement the plant emits is the same equation the filter
inverts:

    ticc_diff_ns = -φ_do - qerr(φ_rx) + v_ticc       (v_ticc ~ 60 ps)

so the rx-TCXO 8 ns sawtooth (qerr) rides on every TICC sample exactly as
on hardware.  When the PPP arm is fed, the filter tracks φ_rx and predicts
qerr(x0) accurately → small innovations.  When the PPP arm is suppressed
(PiFace's 100%-GNSS-PPS-reject config), x0 drifts across ticks, the qerr
prediction is wrong by up to ±tick/2, and the TICC innovations balloon to
~10 ns — which is exactly the regime the OCXO-trusted gate exists to
filter.  That is how the sim reproduces both PiFace's 702 ps ungated /
74 ps gated split and clkPoC3's gate-starvation-during-acquisition.

See dayplan item ``closedLoopServoSim`` (2026-05-28) and
docs/closed-loop-servo-sim.md.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from peppar_fix.binary_layer import BinaryLayer
from peppar_fix.reset_budget import ResetBudget
from peppar_fix.do_freq_est import DOFreqEst, _qerr
from peppar_fix.discipline import (
    coast_cap_from_p22, graded_interval, normalized_distance_to_lock)
from peppar_fix.ocxo_trusted_gate import OcxoTrustedGate


# ────────────────────────────────────────────────────────────────────
# Plant: ground-truth oscillator state
# ────────────────────────────────────────────────────────────────────
@dataclass
class TruthState:
    """Ground-truth oscillator phases/frequencies (the 'real hardware').

    Units match DOFreqEst's state vector: phases in ns, frequencies in
    ppb (= ns/s).  This is the truth the servo is *trying to estimate*;
    the sim measures it (with noise), never hands it to the filter.
    """
    phi_rx: float = 0.0   # rx TCXO phase vs GPS (ns)
    f_rx: float = 0.0     # rx TCXO frequency drift (ppb)
    phi_do: float = 0.0   # DO phase lateness vs GPS (ns)
    f_do: float = 0.0     # DO crystal frequency drift (ppb)


@dataclass
class SimConfig:
    """All sim knobs.  Defaults describe a clean OCXO host with PPP.

    The noise knobs are per-√Hz / per-√s amplitudes that are turned into
    per-epoch standard deviations inside the step (so changing ``dt`` keeps
    the same physical noise PSD).  Use :meth:`ClosedLoopSim.freerun_tdev_1s`
    to read back the resulting free-run floor and tune to a target.
    """
    duration_s: float = 4000.0
    dt_s: float = 1.0
    seed: int = 0
    tick_ns: float = 8.0
    # Coast interval (s): the servo applies a correction every
    # coast_interval_s; BETWEEN corrections the DO free-runs under the
    # held adjfine (the M7 scheduler's coast).  1.0 = correct every
    # epoch.  Sweeping this is the deterministic test bed for
    # longTauGnssCoupling — coasting ~100 s lets the DO wander unmeasured
    # and produces the τ~64-256 s TDEV hump.  The DO phase is still
    # recorded every dt_s (1 Hz), so TDEV sees the free-run wander.
    coast_interval_s: float = 1.0
    # The coast only engages once the loop has converged (the M7
    # scheduler's `_converging` flag holds interval=1 until lock).
    # Coast begins after |estimated φ_do| first drops below this (ns)
    # AND the DO-frequency uncertainty √P[3,3] drops below
    # coast_converge_freq_ppb; before that, correct every epoch.  Both
    # are required because x[3] (DO freq) is the HIDDEN state — coasting
    # while it is still uncertain amplifies the residual frequency error
    # over the gap and diverges.  This is exactly why the real M7
    # scheduler gates the coast on drift characterization, not just phase.
    coast_converge_ns: float = 5.0
    coast_converge_freq_ppb: float = 0.3

    # longTauGnssCoupling coast-cap (bravo PR #86): dynamically cap the
    # coast interval by the filter's predict-only sqrt(P[2,2]) growth so
    # the loop never coasts longer than its own honest phase-uncertainty
    # budget allows.  False = static `coast_interval_s` (the deterministic
    # divergence case my faithfulness work pinned).  True = consume
    # bravo's coast_cap_from_p22 with DOFreqEst.project_p22_coast.
    coast_cap_enabled: bool = False
    coast_cap_t_budget_ns: float = 5.0
    coast_cap_k_sigma: float = 1.0

    # disciplineModeFsm graded-taper (bravo PR #92 + #94 composed).  When
    # enabled, replace the binary 1→coast_every cliff at the converging
    # boundary with a continuous taper of `distance_to_lock` derived from
    # the EKF's √(P[2,2]).  Honest only with Q-from-char (#83); with an
    # inflated Q, √P22 reads locked too early.  Composes with coast-cap:
    # the taper produces the smooth approach, the cap bounds the ceiling.
    graded_taper_enabled: bool = False
    taper_converged_ns: float = 0.5
    taper_far_ns: float = 5.0

    # disciplineModeFsm #107 binary layer A/B harness.  When enabled,
    # construct a BinaryLayer (bravo's gross-fault detector), compute
    # distance_to_lock per epoch from the same √(P[2,2]) anchors as the
    # graded-taper, and call evaluate(distance_to_lock, in_holdover)
    # each epoch.  On 'gross_fault' the sim calls servo.reset() (the
    # in-process reset path) and binary_layer.clear_after_reset() —
    # the same call sequence as the engine wiring at
    # peppar_fix_engine.py: _binary_layer.evaluate / servo.reset /
    # _convergence.reset / _binary_layer.clear_after_reset.
    #
    # To induce sustained distance_to_lock=1.0 deterministically, use
    # drop_measurements_window_s = (t_start, t_end): for epochs inside
    # the window the sim skips emitting measurements (the EKF runs
    # predict-only, P[2,2] grows, distance_to_lock climbs to 1.0).
    # in_holdover_window_s flips the in_holdover input to True during
    # its window — overlap the two for the suppression A/B (holdover
    # → no reset); use drop alone for the gross-fault A/B (reset).
    binary_layer_enabled: bool = False
    binary_layer_consec_max_epochs: int = 60
    drop_measurements_window_s: Optional[tuple] = None
    in_holdover_window_s: Optional[tuple] = None

    # ekfStateSanityNoRecovery (PR #126) — closed-loop validation harness
    # (Main I-073222).  When True, poll `servo.is_state_corrupted()` each
    # epoch and route through `_request_servo_reset` on a True read
    # (same funnel as the binary-layer site, so the shared ResetBudget
    # applies when `reset_budget_enabled` is also on).
    #
    # `force_state_corruption_at_s` is the test hook that reliably
    # triggers the failure mode: at the specified epoch the sim
    # directly pokes `self.ekf.x[2] = force_state_corruption_value_ns`
    # (default 2e8 ns, past the 1e8 ns sanity bound).  Combine with a
    # `drop_measurements_window_s` that starts at the same t_s so no
    # measurement pulls x[2] back down — then predict-only propagation
    # keeps |x[2]| above the bound each epoch, the consec counter
    # accumulates, and the LQR short-circuit (PR #126) holds the
    # actuator at last-sane during the consec=1..N-1 epochs.
    # Reproduces the 22:21 MadHat failure mode without lab time.
    state_sanity_recovery_enabled: bool = False
    force_state_corruption_at_s: Optional[float] = None
    force_state_corruption_value_ns: float = 2e8

    # exitFiveToServoReset (bravo PR #121) — shared windowed ResetBudget
    # consumed by both the binary layer and any synthetic test-only
    # reset requests.  When `reset_budget_enabled` is True, the sim
    # constructs a ResetBudget and routes every reset request through
    # it: within budget → ekf.reset() (actuator preserved); over budget
    # → record "exit5" event + stop the run (matches the engine's
    # exit-5 + wrapper-relaunch semantic — the sim shouldn't keep
    # pumping the loop past the request).
    # Defaults match the engine CLI defaults (`--reset-budget-max 3`,
    # `--reset-budget-window-s 300`).  Default OFF preserves the v1
    # binary-layer sim path byte-identically (#111 A/B unchanged).
    reset_budget_enabled: bool = False
    reset_budget_max: int = 3
    reset_budget_window_s: float = 300.0
    # Multiple back-to-back drop windows.  Each window induces the
    # same sustained-d2l=1.0 pattern `drop_measurements_window_s`
    # does; stacking them is how the budget-exhaustion + recovery
    # tests provoke N reset requests in known time slots.  If both
    # `induced_drop_windows` and the single `drop_measurements_window_s`
    # are set, the single one is treated as the head of the list.
    induced_drop_windows: list = field(default_factory=list)
    # Synthetic direct reset requests at (t_s, reason).  Bypasses the
    # binary-layer mechanism so the budget's shared-across-reasons
    # property can be exercised with multiple `reason` strings without
    # modeling the B/C/D outlier-cascade detectors in the sim.  The
    # sim issues `_request_servo_reset(t, reason)` once when the loop
    # crosses each tuple's t_s value.
    induced_synthetic_requests: list = field(default_factory=list)

    # DO (disciplined oscillator) truth-noise model.
    #   do_wpm_ns:   white phase modulation, per-sample σ (ns).
    #   do_wfm_ppb:  white frequency modulation, σ_y(1s) (ppb) → random
    #                walk in phase; this sets the short-τ TDEV floor.
    #   do_rwfm_ppb_per_sqrt_s: random-walk FM (ppb/√s) → long-τ wander.
    #   do_aging_ppb_per_s:     deterministic secular drift (ppb/s).
    do_wpm_ns: float = 0.02
    do_wfm_ppb: float = 0.05
    do_rwfm_ppb_per_sqrt_s: float = 0.0
    do_aging_ppb_per_s: float = 0.0
    # DO crystal frequency offset the servo must cancel (ppb).  Real
    # hosts: clkPoC3 ~+21, PiFace ~-135, TimeHat ~-44 (steady-state
    # adjfine = -do_f0_ppb).
    do_f0_ppb: float = 0.0

    # rx TCXO truth-noise model (drives the GNSS PPS sawtooth dynamics).
    #   rx_wpm_ns:  white phase modulation per sample (ns).
    #   rx_wfm_ppb: white FM σ_y(1s) (ppb).
    #   rx_f0_ppb:  initial rx TCXO frequency offset (ppb) — makes φ_rx
    #               sweep across 8 ns ticks so the sawtooth is exercised.
    rx_wpm_ns: float = 0.3
    rx_wfm_ppb: float = 0.5
    rx_f0_ppb: float = 3.0

    # GNSS PPS-edge placement jitter (ns, 1-σ).  This is the F9T PPS
    # OUT edge noise that rides on the TICC measurement but is INVISIBLE
    # to the carrier-phase PPP arm (which sees φ_rx, not the PPS edge).
    # Empirically ~2.3 ns (CLAUDE.md F9T PPS TDEV(1s)); it is the reason
    # real TICC innovations are ns-scale-median even with PPP fed, and
    # therefore the reason the OCXO gate (threshold ~0.5 ns) starves the
    # loop unless it can carry x[2] on another arm.  Applied to the edge
    # phase BEFORE the 125 MHz tick quantization, so jitter that crosses
    # a tick boundary produces the heavy innovation tails seen on lab
    # hosts.
    gnss_pps_jitter_ns: float = 2.3

    # routedQErrArm (bravo PR #79).  When True, the sim emits an
    # external-qErr stream alongside TICC (modelling the F9T's TIM-TP
    # sub-tick report) and DOFreqEst's per-edge COMPARATIVE chi² router
    # picks 'ext'/'int'/'raw' per epoch.  ext_qerr_noise_ns models the
    # receiver-specific quality of that report: ~0.1 ns for F9T (clean
    # → 'ext' wins), ~3 ns for F10T (noisy → router falls back to
    # 'raw').  Default off preserves the single internal-qerr path.
    # (The matched qVIR-v2 router this once also exercised was retired in
    # retireQerrMatchingCode I-064807; routed_qerr is now comparative-only.)
    routed_qerr_enabled: bool = False
    ext_qerr_noise_ns: float = 0.1

    # Actuator (PHC adjfine / DAC) quantization step in ppb.  The servo
    # commands a continuous adjfine; the hardware applies the nearest
    # representable value.  The filter's predict uses the COMMANDED
    # value, so the quantization residual is real injected error.
    # 0.0 → ideal actuator.  i226 PHC LSB ≈ 0.073 ppb (docs/
    # i226-adjfine-resolution.md).
    adjfine_lsb_ppb: float = 0.0

    # Measurement-chain noise (per arm).
    sigma_ppp_ns: float = 0.1       # Arm 1 PPP dt_rx
    sigma_ticc_ns: float = 0.060    # Arm 4 TICC quantization (60 ps)
    sigma_extint_ns: float = 8.0    # Arm 3 EXTINT (~8 ns quantization)
    extint_quant_ns: float = 8.0    # EXTINT timestamp quantization step
    sigma_qerr_ppb: float = 0.3     # Arm 2 qErr-as-frequency
    sigma_tdcp_ppb: float = 0.026   # Arm 5 TDCP ensemble freq

    # Which arms the host feeds the filter.  PiFace's real config
    # suppresses the GNSS-PPS-derived arms (PPP/qErr) → use_ppp=False.
    use_ppp: bool = True
    use_qerr: bool = False
    use_tdcp: bool = False
    use_extint: bool = False
    use_ticc: bool = True

    # Bootstrap: acquisition offset and seed frequency handed to the EKF.
    initial_phi_do_ns: float = 200.0   # DO starts this late vs GPS
    initial_freq_ppb: float = 0.0      # bootstrap adjfine seed
    initial_dt_rx_ns: Optional[float] = 0.0  # None → 2-state mode

    # softGateMidTau (I-092034): enable the soft TICC chi² gate (R-inflation,
    # always-admit) instead of the binary hard reject.  Default off →
    # DOFreqEst(soft_ticc_gate=False), the hard gate.  Used by the go/no-go
    # proto (proto_soft_ticc_gate.py) to A/B mid-τ chA TDEV.
    soft_ticc_gate: bool = False

    # EKF process/measurement tuning passthrough (DOFreqEst kwargs).
    sigma_do_phase_ns: float = 0.92
    sigma_do_freq_ppb: float = 0.01
    sigma_tcxo_phase_ns: float = 2.0
    sigma_tcxo_freq_ppb: float = 0.1
    max_step_ppb: Optional[float] = None

    # OCXO-trusted gate.  None → no gate (chi² gate inside DOFreqEst
    # still runs).  Otherwise (sigma_short_tau_ns, k_sigma, min_age_s).
    # disciplineModeFsm #95: sole-observer override for the OCXO gate.
    # When the TICC arm is the only carrier of x[2] (no EXTINT, no
    # TDCP-holdover), the gate must always admit — soft-weighting or
    # rejecting a sole carrier starves the filter (clkpoc3 acquisition
    # never-locks; MadHat R-inflation diverged).  The engine infers
    # this automatically; the sim sets it explicitly to test both
    # gate paths.
    gate_sole_observer: bool = False
    gate_sigma_ns: Optional[float] = None
    gate_k_sigma: float = 10.0
    gate_min_age_s: float = 60.0

    # Label for logs/plots.
    label: str = "sim"


@dataclass
class SimResult:
    """Per-epoch logs from one closed-loop run."""
    t_s: np.ndarray                      # epoch time (s)
    phi_do_true_ns: np.ndarray           # truth DO lateness = chA-equivalent
    phi_rx_true_ns: np.ndarray           # truth rx TCXO phase
    adjfine_ppb: np.ndarray              # servo command each epoch
    est_phi_do_ns: np.ndarray            # filter x[2]
    est_f_do_ppb: np.ndarray             # filter x[3]
    ticc_innov_ns: np.ndarray            # Arm-4 innovation (NaN if not fired)
    gate_rejected: np.ndarray            # bool — OCXO gate rejected Arm 4
    label: str = "sim"
    gate_stats: dict = field(default_factory=dict)

    # ── derived metrics ──
    def tdev(self, taus=None):
        """Detrended TDEV of the DO output (chA-equivalent).  ns."""
        return _tdev(self.t_s, self.phi_do_true_ns, taus)

    def tdev_1s(self) -> float:
        t, td = self.tdev(taus=[1])
        return float(td[0]) if len(td) else float("nan")

    def max_excursion_ns(self, skip_s: float = 300.0) -> float:
        """Max |φ_do| after the acquisition transient (default 300 s)."""
        m = self.t_s >= skip_s
        if not m.any():
            return float("nan")
        return float(np.max(np.abs(self.phi_do_true_ns[m])))

    def locked(self, skip_s: float = 300.0, thresh_ns: float = 5.0) -> bool:
        """True if the post-transient RMS lateness is within thresh."""
        m = self.t_s >= skip_s
        if not m.any():
            return False
        rms = float(np.sqrt(np.mean(self.phi_do_true_ns[m] ** 2)))
        return rms < thresh_ns

    def diverged(self, bound_ns: float = 1e5) -> bool:
        """True if |φ_do| ever blows past bound_ns (loop went unstable)."""
        return bool(np.nanmax(np.abs(self.phi_do_true_ns)) > bound_ns)


_DEFAULT_TAUS = (1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096)


def _tdev(t_s, x_ns, taus=None):
    """Detrend a phase series and compute TDEV with allantools.

    Mirrors validate_ocxo_gate_phase4.compute_tdev: linear-detrend the
    phase, keep only τ < N/4, return (taus_used, tdev_ns).
    """
    import allantools
    taus = list(_DEFAULT_TAUS if taus is None else taus)
    x = np.asarray(x_ns, dtype=float)
    t = np.asarray(t_s, dtype=float)
    if len(x) < 8:
        return np.array([]), np.array([])
    slope, intercept = np.polyfit(t, x, 1)
    resid = x - (slope * t + intercept)
    valid = [tau for tau in taus if tau < len(resid) / 4]
    if not valid:
        return np.array([]), np.array([])
    rate = 1.0 / (t[1] - t[0]) if len(t) > 1 else 1.0
    try:
        t_out, td, _, _ = allantools.tdev(
            resid * 1e-9, rate=rate, data_type="phase", taus=valid)
        return np.asarray(t_out), np.asarray(td) * 1e9
    except Exception:
        return np.array([]), np.array([])


class ClosedLoopSim:
    """Wire the plant (TruthState) to the real servo (DOFreqEst)."""

    def __init__(self, cfg: SimConfig):
        self.cfg = cfg
        self.rng = np.random.default_rng(cfg.seed)
        self.truth = TruthState(
            phi_rx=0.0,
            f_rx=cfg.rx_f0_ppb,
            phi_do=cfg.initial_phi_do_ns,
            # DO crystal drift the servo must cancel (steady-state
            # adjfine settles to -f_do).
            f_do=cfg.do_f0_ppb,
        )
        gate = None
        if cfg.gate_sigma_ns is not None:
            gate = OcxoTrustedGate(
                sigma_short_tau_ns=cfg.gate_sigma_ns,
                k_sigma=cfg.gate_k_sigma,
                min_age_s=cfg.gate_min_age_s,
                is_sole_observer=cfg.gate_sole_observer,
                do_label=cfg.label)
        self.gate = gate
        self.ekf = DOFreqEst(
            sigma_ticc_ns=cfg.sigma_ticc_ns,
            sigma_do_phase_ns=cfg.sigma_do_phase_ns,
            sigma_do_freq_ppb=cfg.sigma_do_freq_ppb,
            sigma_tcxo_phase_ns=cfg.sigma_tcxo_phase_ns,
            sigma_tcxo_freq_ppb=cfg.sigma_tcxo_freq_ppb,
            tick_ns=cfg.tick_ns,
            initial_freq=cfg.initial_freq_ppb,
            # Pass the same value as base_freq so x[3] (DO freq estimate)
            # seeds to -initial_freq_ppb.  Hardcoding 0 made every
            # non-zero initial_freq_ppb start the filter with x[3]=0
            # disagreeing with truth — fine for cold-start A/Bs, broken
            # for warm-start scenarios like routedQErrArm's MadHat config.
            base_freq=cfg.initial_freq_ppb,
            initial_dt_rx_ns=cfg.initial_dt_rx_ns,
            max_step_ppb=cfg.max_step_ppb,
            ocxo_trusted_gate=gate,
            routed_qerr=cfg.routed_qerr_enabled,
            soft_ticc_gate=cfg.soft_ticc_gate)
        self.adjfine = cfg.initial_freq_ppb
        # Binary layer for the gross-fault A/B (#107 consumption).
        self.binary_layer = BinaryLayer(
            consec_max_epochs=cfg.binary_layer_consec_max_epochs,
            enabled=cfg.binary_layer_enabled)
        # Per-epoch gross-fault event log; populated only when the
        # binary layer fires.  Tests assert on the length + timestamps.
        self.gross_fault_events: list = []
        # ekfStateSanityNoRecovery (#126 + I-073222) recovery events:
        # one per sustained state-sanity violation that crossed the
        # consec threshold (servo.is_state_corrupted() == True).  Tuple
        # is (t_s, x2_before_reset).  Routes through the same
        # _request_servo_reset funnel so the ResetBudget applies.
        self.state_sanity_events: list = []
        # exitFiveToServoReset (#121) consumption: shared ResetBudget +
        # the reset_events log.  When `reset_budget_enabled` is False
        # the budget is still constructed (so the helper has a uniform
        # interface) but the helper takes the legacy path — bare
        # ekf.reset(), no exit5, no events.  This preserves the v1
        # binary-layer A/B behavior byte-identically.
        self.reset_budget = ResetBudget(
            max_resets=cfg.reset_budget_max,
            window_s=cfg.reset_budget_window_s,
            enabled=cfg.reset_budget_enabled)
        # Per-epoch reset events: (t_s, reason, outcome) where outcome
        # is "reset" or "exit5".  Mirrors gross_fault_events in shape.
        self.reset_events: list = []
        # Set to the t_s of the first denied reset request; the sim
        # stops the run at that epoch (mirrors the engine's exit-5 +
        # wrapper-relaunch — keeping the loop running past a denied
        # request would hide budget bugs).
        self.exit5_at = None
        # Pending synthetic requests, sorted by t_s; consumed when the
        # loop crosses each tuple's time.
        self._pending_synthetic = sorted(
            list(cfg.induced_synthetic_requests),
            key=lambda r: float(r[0]))

    # ── reset path (the sim's funnel; semantics overlap with the engine's
    # `_request_servo_reset` on the ENABLED branch only — see docstring) ──
    def _request_servo_reset(self, t_s: float, reason: str) -> str:
        """Funnel for every in-process reset.

        The ENABLED branch matches the engine's `_request_servo_reset`:
        within budget → reset + 'reset' event; over budget → 'exit5'
        event + set `self.exit5_at` so the run loop stops at this epoch.

        The DISABLED branch is the **#111 direct-reset baseline** —
        unconditional `ekf.reset()` + 'reset' event — NOT the engine
        helper's disabled→exit5 path (which serves the cascade callers
        the sim doesn't model).  The sim's "disabled" is the v1 A/B
        baseline that #111 tests assert against, not a model of the
        engine's no-opt-in behavior.
        """
        if not self.reset_budget.enabled:
            self.ekf.reset()
            self.reset_events.append((float(t_s), reason, "reset"))
            return "reset"
        if not self.reset_budget.request(reason, float(t_s)):
            self.reset_events.append((float(t_s), reason, "exit5"))
            if self.exit5_at is None:
                self.exit5_at = float(t_s)
            return "exit5"
        self.ekf.reset()
        self.reset_events.append((float(t_s), reason, "reset"))
        return "reset"

    # ── plant ──
    def _step_truth(self, dt: float, adjfine: float):
        c = self.cfg
        rng = self.rng
        # rx TCXO: white FM (random-walk phase) + white PM + drift.
        self.truth.phi_rx += (self.truth.f_rx * dt
                              + rng.normal(0.0, c.rx_wfm_ppb * math.sqrt(dt))
                              + rng.normal(0.0, c.rx_wpm_ns))
        self.truth.f_rx += rng.normal(0.0, 1e-3 * math.sqrt(dt))  # tiny RW-FM
        # DO: aging + RW-FM in frequency.
        self.truth.f_do += (c.do_aging_ppb_per_s * dt
                            + rng.normal(0.0, c.do_rwfm_ppb_per_sqrt_s
                                         * math.sqrt(dt)))
        # DO phase: servo response + white FM (sets short-τ floor) + white PM.
        self.truth.phi_do += (-(self.truth.f_do + adjfine) * dt
                              + rng.normal(0.0, c.do_wfm_ppb * math.sqrt(dt))
                              + rng.normal(0.0, c.do_wpm_ns))

    # ── sensors ──
    def _emit(self) -> dict:
        c = self.cfg
        rng = self.rng
        t = self.truth
        m: dict = {"dt": c.dt_s}
        if c.use_ppp:
            m["dt_rx_ns"] = t.phi_rx + rng.normal(0.0, c.sigma_ppp_ns)
            m["dt_rx_sigma_ns"] = c.sigma_ppp_ns
        if c.use_qerr:
            m["qerr_freq_ppb"] = t.f_rx + rng.normal(0.0, c.sigma_qerr_ppb)
            m["qerr_freq_sigma_ppb"] = c.sigma_qerr_ppb
        if c.use_tdcp:
            m["tdcp_freq_ppb"] = t.f_rx + rng.normal(0.0, c.sigma_tdcp_ppb)
            m["tdcp_freq_sigma_ppb"] = c.sigma_tdcp_ppb
        if c.use_extint:
            # DO PPS timestamped by F9T → ~8 ns quantized.
            raw = t.phi_do + rng.normal(0.0, c.sigma_extint_ns)
            q = c.extint_quant_ns
            m["extint_phase_ns"] = round(raw / q) * q if q > 0 else raw
            m["extint_sigma_ns"] = c.sigma_extint_ns
        if c.use_ticc:
            # The defining closed-loop coupling: TICC carries the
            # rx-TCXO sawtooth (qerr) on top of -φ_do.  The GNSS PPS
            # edge also carries placement jitter the PPP arm can't see;
            # applying it BEFORE the tick quantization reproduces both
            # the ns-scale median innovation and its heavy tails (jitter
            # crossing a tick boundary flips qerr by ~tick).
            edge = t.phi_rx + rng.normal(0.0, c.gnss_pps_jitter_ns)
            m["ticc_diff_ns"] = (-t.phi_do - _qerr(edge, c.tick_ns)
                                 + rng.normal(0.0, c.sigma_ticc_ns))
            m["ticc_sigma_ns"] = c.sigma_ticc_ns
            # External qErr stream — the F9T/F10T's TIM-TP report of
            # its own sub-tick at this edge.  qerr() of the actual edge
            # plus receiver-specific reporting noise.  Consumed by the
            # comparative router (routed_qerr_enabled).
            if c.routed_qerr_enabled:
                m["ticc_qerr_ns"] = (_qerr(edge, c.tick_ns)
                                     + rng.normal(0.0, c.ext_qerr_noise_ns))
        return m

    def run(self) -> SimResult:
        c = self.cfg
        n = int(round(c.duration_s / c.dt_s))
        t_s = np.empty(n)
        phi_do = np.empty(n)
        phi_rx = np.empty(n)
        adj = np.empty(n)
        est_phi = np.empty(n)
        est_f = np.empty(n)
        innov = np.full(n, np.nan)
        innov_s = np.full(n, np.nan)
        rejected = np.zeros(n, dtype=bool)
        routes = np.empty(n, dtype=object)
        routes[:] = ""

        coast_every = max(1, int(round(c.coast_interval_s / c.dt_s)))
        converged = (coast_every == 1)
        last_correction_k = 0
        # When the coast-cap is on this is reduced after each correction
        # to the largest τ where k·√P[2,2](τ) ≤ t_budget — bravo's
        # coast_cap_from_p22 fed by DOFreqEst.project_p22_coast.  Starts
        # at the requested ceiling; the cap only ever tightens it.
        dyn_coast_every = coast_every
        # ekfStateSanityNoRecovery test hook: poke x[2] to corruption
        # value when the loop crosses force_state_corruption_at_s.
        # One-shot via this flag so we don't re-poke each epoch.
        _corruption_pending = (c.force_state_corruption_at_s is not None)
        for k in range(n):
            # State-corruption injection (test hook).  Fires once when
            # t crosses force_state_corruption_at_s; pairs naturally
            # with a drop_measurements_window starting at the same time
            # (no measurement → predict-only keeps |x[2]| > bound).
            if (_corruption_pending
                    and (k * c.dt_s) >= float(c.force_state_corruption_at_s)):
                self.ekf.x[2] = float(c.force_state_corruption_value_ns)
                _corruption_pending = False
            # The servo corrects every `coast_every` epochs ONCE
            # converged; while still converging it corrects every epoch
            # (the M7 scheduler's _converging flag).  Between corrections
            # the DO free-runs under the held adjfine (the coast).
            # Ordering matches the engine: read → compute → actuate.  The
            # hardware adjfine is the NEGATED return of update()
            # (peppar_fix_engine.py:8148 adjfine_ppb=-servo.update).
            if (not converged
                    and abs(self.ekf.estimated_phase_ns) < c.coast_converge_ns
                    and self.ekf.freq_uncertainty_ppb < c.coast_converge_freq_ppb):
                converged = True
            # Coast-cap (when on) bounds the target; graded-taper (when
            # on) smooths the approach to that bound.  Composing both is
            # what my #92 A/B verdict showed restores baseline TDEV.
            target_every = (min(coast_every, dyn_coast_every)
                            if c.coast_cap_enabled else coast_every)
            if c.graded_taper_enabled:
                sqrt_p22 = math.sqrt(max(0.0, float(self.ekf.P[2, 2])))
                m = normalized_distance_to_lock(
                    sqrt_p22, c.taper_converged_ns, c.taper_far_ns)
                effective_every = graded_interval(target_every, m,
                                                  min_interval=1)
            else:
                effective_every = target_every if converged else 1
            t_now = k * c.dt_s
            # Stack the legacy single-tuple form ahead of the new list
            # form so existing callers (#111 binary-layer A/B) keep
            # working unchanged.
            _drop_wins = []
            if c.drop_measurements_window_s is not None:
                _drop_wins.append(c.drop_measurements_window_s)
            _drop_wins.extend(c.induced_drop_windows or [])
            in_drop_win = any(w[0] <= t_now <= w[1] for w in _drop_wins)
            if (k - last_correction_k) >= effective_every or k == 0:
                gap_s = max(c.dt_s, (k - last_correction_k) * c.dt_s)
                if in_drop_win:
                    # Predict-only correction: P grows, no arm fires,
                    # adjfine unchanged.  Induces sustained
                    # distance_to_lock=1.0 for the binary-layer A/B.
                    self.ekf.update(dt=gap_s)
                else:
                    meas = self._emit()
                    # dt handed to the filter is the gap since the last
                    # correction; predict grows P over the coast.
                    meas["dt"] = gap_s
                    self.adjfine = -self.ekf.update(**meas)
                last_correction_k = k
                if self.ekf.last_arm_innov.get("ticc") is not None:
                    innov[k] = self.ekf.last_arm_innov["ticc"]
                    _s = self.ekf.last_arm_S.get("ticc")
                    if _s is not None:
                        innov_s[k] = _s
                rejected[k] = self.ekf.last_ocxo_gate_rejected
                routes[k] = getattr(self.ekf, "last_ticc_route", "int")
                # Recompute the coast-cap on the post-update filter.  One
                # O(coast_every) projection per correction, not per epoch
                # (the closure looks up a precomputed table — addresses
                # bravo's perf note on coast_cap_from_p22).
                if converged and c.coast_cap_enabled:
                    table = self.ekf.project_p22_coast(coast_every)
                    p22_at_tau = lambda tau, _t=table: _t[int(tau)]
                    dyn_coast_every = coast_cap_from_p22(
                        c.coast_cap_t_budget_ns, p22_at_tau,
                        k_sigma=c.coast_cap_k_sigma,
                        min_tau_s=1, max_tau_s=coast_every)

            t_s[k] = k * c.dt_s
            phi_do[k] = self.truth.phi_do
            phi_rx[k] = self.truth.phi_rx
            adj[k] = self.adjfine
            est_phi[k] = self.ekf.estimated_phase_ns
            est_f[k] = self.ekf.estimated_freq_ppb

            # Hardware applies the nearest representable adjfine; the
            # filter's predict already used the (unquantized) command, so
            # the quantization residual is real injected error.  The DO
            # steps every dt_s under the held adjfine (free-run on
            # non-correction epochs).
            applied = self.adjfine
            if c.adjfine_lsb_ppb > 0:
                applied = round(applied / c.adjfine_lsb_ppb) * c.adjfine_lsb_ppb
            self._step_truth(c.dt_s, applied)

            # disciplineModeFsm #107 binary-layer per-epoch evaluation.
            # Mirrors the engine's call sequence at engine:8334-8360:
            # compute distance_to_lock from √(P[2,2]) with the taper
            # anchors, query the holdover state, evaluate; on
            # 'gross_fault' call servo.reset() (preserves the actuator
            # command) + binary_layer.clear_after_reset().  Default off
            # (BinaryLayer.evaluate returns None when disabled).
            if c.binary_layer_enabled:
                sqrt_p22 = math.sqrt(max(0.0, float(self.ekf.P[2, 2])))
                d2l = normalized_distance_to_lock(
                    sqrt_p22, c.taper_converged_ns, c.taper_far_ns)
                in_holdover = (c.in_holdover_window_s is not None
                               and c.in_holdover_window_s[0]
                               <= t_s[k] <= c.in_holdover_window_s[1])
                if self.binary_layer.evaluate(d2l, in_holdover) == "gross_fault":
                    self.gross_fault_events.append(
                        (float(t_s[k]), sqrt_p22))
                    # Route through the funnel: budget-disabled → bare
                    # ekf.reset() exactly as before (preserves v1
                    # behavior); budget-enabled → honor the cap.
                    self._request_servo_reset(t_s[k], "binary_layer")
                    self.binary_layer.clear_after_reset()

            # Fire any synthetic reset requests whose t has been
            # reached.  Bypasses the binary-layer machinery so the
            # shared-across-reasons budget property can be exercised
            # without modeling multiple detector types.
            while (self._pending_synthetic
                   and float(self._pending_synthetic[0][0]) <= t_s[k]):
                _t_req, _reason = self._pending_synthetic.pop(0)
                self._request_servo_reset(t_s[k], str(_reason))

            # ekfStateSanityNoRecovery (Main I-073222 — closed-loop
            # validation of PR #126).  Mirrors the engine site
            # (peppar_fix_engine.py: after servo.update, poll
            # is_state_corrupted, route through _request_servo_reset).
            # is_state_corrupted() flips True only after N consecutive
            # |x[i]| > bound violations (set in DOFreqEst with
            # _state_sanity_consec_threshold, default 3), so the
            # LQR-short-circuit inside DOFreqEst.update() will have
            # held adjfine at last-sane for the consec=1..N-1 epochs.
            if c.state_sanity_recovery_enabled and self.ekf.is_state_corrupted():
                x2_pre = float(self.ekf.x[2])
                self.state_sanity_events.append((float(t_s[k]), x2_pre))
                self._request_servo_reset(t_s[k], "state_sanity")

            # Stop the run if the budget denied a request — the engine's
            # exit-5 path tears down the process; the sim shouldn't
            # keep pumping the loop past the denial.
            if self.exit5_at is not None:
                # Truncate the result arrays to data up through this
                # epoch + return early.
                t_s = t_s[: k + 1]
                phi_do = phi_do[: k + 1]
                phi_rx = phi_rx[: k + 1]
                adj = adj[: k + 1]
                est_phi = est_phi[: k + 1]
                est_f = est_f[: k + 1]
                innov = innov[: k + 1]
                innov_s = innov_s[: k + 1]
                rejected = rejected[: k + 1]
                routes = routes[: k + 1]
                break

        # Routed-qErr diagnostics: per-epoch S for chi² computation +
        # the per-epoch route selection ('ext'/'int'/'raw').  Attached to
        # the instance (don't bloat SimResult; only the routed_qerr A/B
        # consumes them).
        self.last_innov_s = innov_s
        self.last_routes = routes
        return SimResult(
            t_s=t_s, phi_do_true_ns=phi_do, phi_rx_true_ns=phi_rx,
            adjfine_ppb=adj, est_phi_do_ns=est_phi, est_f_do_ppb=est_f,
            ticc_innov_ns=innov, gate_rejected=rejected, label=c.label,
            gate_stats=(self.gate.stats if self.gate else {}))

    def freerun_tdev_1s(self, duration_s: float = 2000.0) -> float:
        """Measure the DO's free-run TDEV(1s) for this config (no servo).

        Used to calibrate the DO noise knobs to a target floor.  Runs an
        open-loop plant (adjfine=0, f_do=0) and TDEVs the phase.
        """
        c = self.cfg
        rng = np.random.default_rng(c.seed + 99)
        n = int(round(duration_s / c.dt_s))
        phi = 0.0
        f = 0.0
        xs = np.empty(n)
        for k in range(n):
            f += (c.do_aging_ppb_per_s * c.dt_s
                  + rng.normal(0.0, c.do_rwfm_ppb_per_sqrt_s * math.sqrt(c.dt_s)))
            phi += (-f * c.dt_s
                    + rng.normal(0.0, c.do_wfm_ppb * math.sqrt(c.dt_s))
                    + rng.normal(0.0, c.do_wpm_ns))
            xs[k] = phi
        ts = np.arange(n, dtype=float) * c.dt_s
        _, td = _tdev(ts, xs, taus=[1])
        return float(td[0]) if len(td) else float("nan")


# ────────────────────────────────────────────────────────────────────
# Faithfulness presets — tuned to reproduce known empirical lock quality.
# ────────────────────────────────────────────────────────────────────
def preset(name: str, **overrides) -> SimConfig:
    """Return a SimConfig preset by name.

    Presets target the dayplan faithfulness bar:
      clkpoc3 no-gate ~104 ps, clkpoc3 gated never-locks ~1.74 ns,
      PiFace v1 ~74 ps, PiFace ungated ~565-702 ps.

    Noise knobs are calibrated so freerun_tdev_1s() lands near the
    per-host floor (PiFace 54 ps, clkPoC3 46 ps).
    """
    # do_wfm_ppb is calibrated so freerun_tdev_1s() ≈ the per-host floor
    # (clkPoC3 45 ps, PiFace 54 ps).  All lab hosts feed PPP+qErr+TICC,
    # plus EXTINT where wired (confirmed from day0527-disc2 arm-state
    # CSVs).  The ~2.3 ns GNSS PPS jitter (default) makes TICC
    # innovations ns-scale-median, matching the captures.
    _CLK = dict(do_wfm_ppb=0.079, do_wpm_ns=0.01, do_f0_ppb=21.0,
                rx_f0_ppb=3.0, use_ppp=True, use_qerr=True, use_ticc=True)
    _PIF = dict(do_wfm_ppb=0.095, do_wpm_ns=0.01, do_f0_ppb=-135.0,
                rx_f0_ppb=5.0, use_ppp=True, use_qerr=True, use_ticc=True)
    presets = {
        # clkPoC3 no-gate: TICC + PPP + qErr + EXTINT all update; the
        # ~2.3 ns PPS jitter injected through Arm 4 degrades the output
        # to ~2× the freerun floor (~104 ps).
        "clkpoc3": dict(label="clkpoc3", use_extint=True,
                        initial_phi_do_ns=200.0, **_CLK),
        # clkPoC3 + OCXO gate, TICC-PRIMARY (no EXTINT to carry x[2]).
        # The gate (K=10, σ=45 ps → ~0.45 ns threshold) rejects the
        # ns-scale TICC innovations, and with no other x[2] arm the DO
        # phase is uncontrolled → never locks (~1.74 ns).  This is the
        # clkpoc3GateOverRejectsBeforeLock starvation.
        "clkpoc3-gated": dict(label="clkpoc3-gated", use_extint=False,
                              initial_phi_do_ns=200.0,
                              gate_sigma_ns=0.045, gate_k_sigma=10.0,
                              gate_min_age_s=60.0, **_CLK),
        # PiFace no-gate: the ns-scale TICC innovations pass into x[2]/
        # x[3] and the OCXO inherits → degraded output (~565 ps).
        "piface-ungated": dict(label="piface-ungated", use_extint=True,
                               initial_phi_do_ns=200.0, **_PIF),
        # PiFace v1 gate WITH EXTINT carrying x[2]: the gate rejects the
        # noisy TICC, EXTINT (8 ns, low-pass-filtered by the loop) holds
        # x[2], and the good DO floor shines short-τ → near floor (~74 ps).
        # The contrast with clkpoc3-gated isolates the real lesson: gate
        # TICC, but keep a non-TICC x[2] carrier (disciplineModeFsm).
        "piface-v1": dict(label="piface-v1", use_extint=True,
                          initial_phi_do_ns=200.0,
                          gate_sigma_ns=0.054, gate_k_sigma=10.0,
                          gate_min_age_s=60.0, **_PIF),
        # ── CURRENT London hardware (mtbaseline 2026-07-05, I-084500) ──
        # Measured from the settled arm-state of the real engine run, NOT
        # the pre-London presets above (whose do_f0/gate no longer describe
        # the hardware after the host+DO+receiver shuffle).  PiFace now runs
        # an IsoTemp OCXO (ex-MadHat) with:
        #   do_f0=+2.30 ppb, NO OCXO gate (measured 0% reject — vs piface-v1's
        #   99.6% lockout), arms PPP+qErr+EXTINT+TICC with routed-qErr routing
        #   'ext' (clean qErr innov 0.036 ns), TICC innov 0.71 ns.
        # Real Q from state/dos/isotemp-ocxo131-100-madhat.toml:
        #   sigma_do_phase=0.0564, sigma_do_freq=0.009; DO char
        #   coast_tdev_ref=0.070 ns@1s slope 1.118 (strong OCXO RWFM), 16-bit
        #   AD5693R LSB 0.026 ppb.
        # CAVEAT (do NOT read absolute amplitude): with the do_wfm/do_rwfm
        # below the sim reproduces the real hump's SHAPE (mid-τ bump-and-
        # recover) but over-produces its amplitude ~6× (real chA hump is
        # 484 ps@32s; the DO mid-τ freerun the coast char extrapolates to is
        # unpinned).  Use for RELATIVE lever direction only; grade absolute
        # numbers by lab A/B on PiFace.  Re-grade verdict (relative, robust):
        # looser Q[3,3] (sigma_do_freq 0.009→~0.03) is the lead lever
        # (hump −52%, two-clock p95 12.6→6.2 ns); coast + stiffer-Q both
        # worsen it; Q[2,2] and softgate are moot (0% gate).
        "piface-current": dict(
            label="piface-current", do_f0_ppb=2.30,
            sigma_do_phase_ns=0.0564, sigma_do_freq_ppb=0.009,
            sigma_ticc_ns=0.060, sigma_tcxo_phase_ns=2.0, sigma_tcxo_freq_ppb=0.10,
            use_ppp=True, use_qerr=True, use_extint=True, use_ticc=True,
            use_tdcp=False, routed_qerr_enabled=True, ext_qerr_noise_ns=0.04,
            gnss_pps_jitter_ns=0.7, rx_wfm_ppb=1.0, adjfine_lsb_ppb=0.026,
            do_wpm_ns=0.01, do_wfm_ppb=0.11, do_rwfm_ppb_per_sqrt_s=0.05,
            initial_phi_do_ns=200.0),
    }
    if name not in presets:
        raise KeyError(f"unknown preset {name!r}; have {sorted(presets)}")
    cfg = SimConfig(**presets[name])
    for k, v in overrides.items():
        setattr(cfg, k, v)
    return cfg


# ────────────────────────────────────────────────────────────────────
# Two-clock differential — cross-host PPS-OUT agreement (1 ns bound).
# ────────────────────────────────────────────────────────────────────
def run_two_clock(cfg_a: SimConfig, cfg_b: SimConfig,
                  share_gnss=None, *, seed_a: Optional[int] = None,
                  seed_b: Optional[int] = None):
    """Faithful shared-antenna two-clock differential; return
    (res_a, res_b, diff_ns) where diff = φ_do,A − φ_do,B (the chA−chA
    PPS-OUT excursion the acceptance test measures on a shared TICC).

    Faithfulness (docs/two-site-sync-budget.md §2).  On a shared antenna
    the receiver clock (rx TCXO), DO free-running noise, and disciplining-
    loop noise are ALL in the *independent* column — only sky-side terms
    (orbit/clock, iono, tropo, multipath) cancel.  The sim's ``phi_rx``
    *is* the per-host rx TCXO and the sim injects no sky-side term, so the
    faithful shared-antenna model is two FULLY INDEPENDENT sims and

        σ²_Δ(τ) ≈ σ²_clock,A(τ) + σ²_clock,B(τ)          (= 2σ²_clock)

    matching the budget's §2 shared-antenna formula.  There is nothing to
    share (the common-mode sky term is modelled as zero-in-both, which is
    equivalent to identical-and-cancelling), so this harness carries no
    correlated stream and none of the v1 desync hazard.

    Independence is enforced by seeding the two plants from *different*
    RNG seeds: ``seed_a`` (default ``cfg_a.seed``) and ``seed_b`` (default
    ``cfg_b.seed``, bumped by a fixed offset when it collides with
    ``seed_a`` so two clocks built from the same ``preset()`` — which pins
    ``seed=0`` — are still independent rather than bit-identical).

    ``share_gnss`` is DEPRECATED and ignored.  The v1 behaviour it named
    (reseed B with A's whole RNG) was never faithful: it shared the DO
    noise too, collapsing the differential toward zero, and desynced the
    instant the two arm configs drew a different number of randoms per
    epoch.  A non-None value raises a warning so old callers surface.
    """
    if share_gnss is not None:
        import warnings
        warnings.warn(
            "run_two_clock(share_gnss=...) is deprecated and ignored; the "
            "harness is now faithful (independent per-host realizations per "
            "two-site-sync-budget.md §2). Remove the argument.",
            DeprecationWarning, stacklevel=2)
    sa = cfg_a.seed if seed_a is None else seed_a
    sb = cfg_b.seed if seed_b is None else seed_b
    if sb == sa:
        sb = sa + 10_007      # keep the two plants independent by default
    cfg_a.seed = sa
    cfg_b.seed = sb
    res_a = ClosedLoopSim(cfg_a).run()
    res_b = ClosedLoopSim(cfg_b).run()
    n = min(len(res_a.t_s), len(res_b.t_s))
    diff = res_a.phi_do_true_ns[:n] - res_b.phi_do_true_ns[:n]
    return res_a, res_b, diff


def two_clock_excursion_stats(diff_ns, t_s, skip_s: float = 300.0) -> dict:
    """Excursion statistics for the two-clock differential — the moonshot
    acceptance metric (docs/two-site-sync-budget.md §4.4: p95 ≤ 1 ns
    shared antenna).

    ``diff_ns`` is φ_do,A − φ_do,B; ``t_s`` its epoch axis.  Grades on the
    SETTLED, reset-free portion (t ≥ skip_s) exactly as the lab A/B does —
    the acquisition transient is not part of the bound.  Returns p50/p95/
    max of |Δ| and the RMS, all in ns, plus the sample count.
    """
    d = np.asarray(diff_ns, dtype=float)
    t = np.asarray(t_s, dtype=float)[: len(d)]
    m = t >= skip_s
    if not m.any():
        return dict(p50=float("nan"), p95=float("nan"), max=float("nan"),
                    rms=float("nan"), n=0)
    a = np.abs(d[m])
    return dict(
        p50=float(np.percentile(a, 50)),
        p95=float(np.percentile(a, 95)),
        max=float(np.max(a)),
        rms=float(np.sqrt(np.mean(d[m] ** 2))),
        n=int(m.sum()))

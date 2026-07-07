"""4-state DOFreqEst — EKF fusing raw TICC with PPP carrier phase.

See docs/glossary.md for term definitions (DO, rx TCXO, qerr, etc.).

Models both oscillators (rx TCXO + DO) and the 125 MHz tick
quantization that links them through PPS edges.  No external qErr
correction needed.

State vector:
    x = [φ_rx, f_rx, φ_do, f_do]

    φ_rx:  rx TCXO phase offset from GPS (ns) — tracked from dt_rx
    f_rx:  rx TCXO frequency drift rate (ppb)
    φ_do:  DO phase error from GPS (ns) — steered to zero
    f_do:  DO crystal frequency drift rate (ppb)

Process model (linear):
    φ_rx += f_rx · dt
    f_rx += w_f_rx  (random walk)
    φ_do -= (f_do + adjfine) · dt     ← NEGATIVE: φ_do is "lateness"
    f_do += w_f_do  (random walk)

Measurements:
    z_ppp  = φ_rx + v_ppp                    (PPP dt_rx, ~0.1 ns, linear)
    z_ticc = −φ_do − qerr(φ_rx) + v_ticc    (raw TICC, nonlinear via tick)

    qerr(φ) = φ − round(φ / tick) · tick    (sub-tick rx TCXO phase)

The nonlinearity is in qerr(): the rx TCXO's 125 MHz tick quantization
that determines where gnss_pps fires.  PPP constrains φ_rx to ~0.1 ns,
resolving the tick ambiguity — analogous to integer ambiguity resolution
in PPP-AR.  The filter performs qErr correction internally at PPP
precision, which should be better than TIM-TP qErr.

Between tick boundaries (98.75% of epochs when PPP sigma = 0.1 ns):
    ∂qerr/∂φ_rx = 1, so H_ticc = [−1, 0, −1, 0]

This couples the rx TCXO and DO states in the measurement — the key
insight that makes 4-state fusion work where 2-state failed.

State × measurement-arm incidence
---------------------------------
Which of the six conditional arms (see update()) observes which state.
● = direct (H has a 1 for that state); ◐ = intermittent coupling.
Diagram: docs/dofreqest-state-arm-map.svg
(regenerate via scripts/gen_dofreqest_state_arm_svg.py)

                  A1    A2    A5    A3    A6    A4
    state         PPP   qErr  TDCP  EXT   pseu  TICC    class
    x0 φ_rx        ●                             ◐¹     measured
    x1 f_rx              ●     ●                         measured
    x2 φ_do                          ●     ●     ●       measured
    x3 f_do                                              HIDDEN²

¹ Arm 4 (TICC) is the only non-diagonal arm: H = [−1,0,−1,0] normally,
  collapsing to [0,0,−1,0] right at a 125 MHz tick boundary (the qErr
  derivative vanishes there).  So its x0 coupling is intermittent.
² x3 (f_do) has NO arm pointing at it.  It is recovered purely through
  the predict step: φ_do -= (f_do + adjfine)·dt folds f_do into the
  measured x2 over time.  x3 is the state we actuate on (adjfine ≈ −f_do
  at steady state) yet the one we never directly measure.

Directly measured: x0, x1, x2.  Hidden (inferred via the dynamics): x3.
"""

import logging
import math
import numpy as np

from peppar_fix.innov_control_monitor import InnovControlMonitor

log = logging.getLogger(__name__)

_CHI2_GATE_THRESHOLD = 100.0  # 10σ; innov²/S > 100 skips Arm 4 update


def _qerr(phi_tcxo_ns, tick_ns=8.0):
    """Compute qErr from rx TCXO phase: sub-tick residual."""
    return phi_tcxo_ns - round(phi_tcxo_ns / tick_ns) * tick_ns


class DOFreqEst:
    """4-state EKF + LQR for DO frequency steering.

    Drop-in interface: update() takes offset_ns (raw TICC, no qErr),
    returns ppb.  Also takes dt_rx_ns for PPP carrier-phase fusion.
    """

    def __init__(self, sigma_ticc_ns=0.060,
                 sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01,
                 sigma_tcxo_phase_ns=2.0, sigma_tcxo_freq_ppb=0.1,
                 tick_ns=8.0,
                 max_ppb=62_500_000.0, initial_freq=0.0,
                 initial_dt_rx_ns=None, base_freq=None,
                 max_step_ppb=None,
                 ocxo_trusted_gate=None,
                 routed_qerr=False,
                 soft_ticc_gate=False,
                 do_freq_clamp_ppb=None,
                 lqr_phase_gain=-0.05):
        self.max_ppb = max_ppb
        # routedQErrArm (latestQErrChiSelect, docs/latest-qerr-chi-select.md):
        # per-edge COMPARATIVE chi² router for the TICC arm.  When enabled,
        # each TICC edge is routed to ONE of three candidates — external-qErr-
        # corrected, internal-qerr(x[0])-corrected, or raw — by smallest chi²
        # (a non-raw candidate wins only if it BEATS raw, not merely clears an
        # absolute threshold).  The comparative gate is what makes the
        # latest-unmatched qErr feed safe: an off-by-edge qErr is ~3.3 ns RMS
        # → chi²≈17 at lock, which would PASS an absolute-100 gate and poison
        # z, but loses to raw under the comparative gate.  Default off
        # preserves the single internal-qerr path (the --no-qerr-latest-chi
        # escape hatch).  The legacy absolute-gate v1 router and the qVIR-
        # gated v2 router were retired in retireQerrMatchingCode (I-064807);
        # qVIR survives only as an offline diagnostic.
        self._routed_qerr = routed_qerr
        # softGateMidTau (I-092034): replace the Arm-4 chi² BINARY reject
        # with a soft R-inflation gate (R·max(1, χ²/K²), always admit).
        # Default off → byte-identical hard gate.  See the gate block in
        # update() and docs ref two-site-sync-budget.md §3.2.1.
        self._soft_ticc_gate = bool(soft_ticc_gate)
        # L3 of the TDCP slip-protection stack: per-epoch actuator
        # rate limit.  None = disabled (today's behavior preserved).
        # When set, |adjfine_new - adjfine_prev| is clamped to this
        # value, providing belt-and-suspenders defense against any
        # arm — Arm 5 co-slip that defeats L1+L2, Arm 4 TICC outlier
        # punch, Arm 3 EXTINT outlier punch, or filter divergence.
        # The cost of "too low" is slower legitimate convergence
        # (multi-epoch pull on a real step); cost of "too high" is
        # less defense against catastrophic single-epoch errors.
        # Engaged only after the first adjfine has been computed —
        # the very first epoch's adjfine is unconstrained because
        # there's no prior to rate-limit against.
        self.max_step_ppb = max_step_ppb
        # midTauServoKnobs (I-084500, bravo↔delta): do_freq_clamp_ppb is a
        # GATED, DEFAULT-OFF safety backstop against the actuator-rail runaway
        # delta observed on the GNSSDO+ bench (x[3]→+386 ppb, adjfine word→
        # rail).  Clamp the DO-freq estimate x[3] to within ±do_freq_clamp_ppb
        # of its bootstrap nominal — an OCXO/DAC has a bounded pull range, so
        # x[3] physically cannot wander further.  A peer to max_step_ppb: a
        # cheap seatbelt that bounds a freq-state windup INSIDE the gross
        # state-sanity bounds (±1e6 ppb) while root cause is investigated.
        # None → byte-identical to pre-existing behavior.  x3 nominal set
        # below once crystal_freq is known.  See docs/mid-tau-servo-knobs.md.
        # NOTE: servo_sim (which drives this same DOFreqEst) does NOT reproduce
        # the x[3] rail from any modeled perturbation — dt_rx glitches are
        # decoupled from DO steering and DO-obs corruption gates-out→coasts
        # (actuator stays ≤2 ppb).  This clamp is therefore an unvalidated
        # HARDWARE seatbelt pending a captured repro, not a sim-proven fix.
        self._do_freq_clamp_ppb = do_freq_clamp_ppb
        self.n_do_freq_clamped = 0
        self.last_do_freq_clamped = False
        self.tick_ns = tick_ns
        self.dt = 1.0

        # State: [φ_rx, f_rx, φ_do, f_do]
        # f_do = DO crystal drift = negative of steady-state adjfine.
        # initial_freq = base_freq + glide (what bootstrap applied).
        # x[3] must use base_freq (the DO's true drift), not
        # initial_freq (which includes the transient glide offset).
        # _last_u uses initial_freq (the actual applied adjfine).
        phi_tcxo_init = initial_dt_rx_ns if initial_dt_rx_ns is not None else 0.0
        crystal_freq = base_freq if base_freq is not None else initial_freq
        self.x = np.array([phi_tcxo_init, 0.0, 0.0, -crystal_freq])
        # x[3] nominal for the do_freq_clamp anti-windup (see above): the
        # bootstrap DO-freq estimate.  The clamp bounds |x[3] - nominal|.
        self._x3_nominal = -crystal_freq

        # F matrix — note F[2,3] is NEGATIVE: positive f_do (crystal
        # running fast) makes φ_do decrease (less late), because φ_do is
        # defined as "lateness" = GPS_phase − DO_phase.
        self.F = np.array([
            [1.0, self.dt,  0.0, 0.0],
            [0.0, 1.0,      0.0, 0.0],
            [0.0, 0.0,      1.0, -self.dt],
            [0.0, 0.0,      0.0, 1.0],
        ])

        # B: adjfine only affects φ_do.  NEGATIVE because positive
        # adjfine speeds up the DO, reducing lateness (φ_do).
        self.B = np.array([0.0, 0.0, -self.dt, 0.0])

        # Measurement: PPP (linear)
        self.H_ppp = np.array([[1.0, 0.0, 0.0, 0.0]])

        # Measurement: TICC (EKF — Jacobian computed at each step)
        # Between tick boundaries: H_ticc ≈ [-1, 0, -1, 0]
        # This is the linearization of h(x) = -x[2] - qerr(x[0])

        # Process noise
        self.Q = np.diag([
            sigma_tcxo_phase_ns ** 2,
            sigma_tcxo_freq_ppb ** 2,
            sigma_do_phase_ns ** 2,
            sigma_do_freq_ppb ** 2,
        ])

        # Measurement noise
        self.R_ticc = np.array([[sigma_ticc_ns ** 2]])

        # Initial covariance — tuned to bootstrap knowledge:
        # x[0]: dt_rx from drift file, ~100 ns stale from rx TCXO drift
        # x[1]: rx TCXO rate, poorly known (temperature dependent)
        # x[2]: DO phase, ~5000 ns (bootstrap glide not converged yet)
        # x[3]: DO crystal freq, well-known from bootstrap adjfine (~1 ppb)
        if initial_dt_rx_ns is not None:
            self.P = np.diag([100.0**2, 10.0**2, 5000.0**2, 1.0**2])
        else:
            self.P = np.diag([1e6, 100.0**2, 1000.0**2, 100.0**2])

        # LQR: only DO states are controllable (x[2] = DO phase, x[3] = DO freq)
        # L[2] = phase gain (negative: positive φ_do = late → more u
        #         → more adjfine → speed up → reduce lateness).  Its
        #         magnitude sets the phase-loop BANDWIDTH: u = L[2]·φ per
        #         epoch → time-constant ≈ 1/|L[2]| epochs → corner ≈
        #         |L[2]|/2π Hz.  Default -0.05 → ~20 s corner.  For a
        #         low-noise DO whose free-run beats the GNSS reference out
        #         to long τ (GNSSDO+ Rakon OCXO), push |L[2]| DOWN (e.g.
        #         -0.001 → ~1000 s corner) so the DO free-runs below the
        #         corner and the loop only tracks GNSS above it — see
        #         docs/gnssdo-servo-loop-bandwidth.md.
        # L[3] = freq cancellation
        self.L = np.array([0.0, 0.0, lqr_phase_gain, 1.0])

        self.freq = initial_freq
        # _last_u is the LQR u value.  Engine applies u as adjfine
        # (after double negation: servo returns -u, engine negates).
        # At startup, bootstrap set adjfine = initial_freq, so
        # the last applied u = initial_freq.
        self._last_u = initial_freq

        # State-sanity guard (see update() for the per-epoch check).
        # _state_corrupted is the "wrapper should reset me" signal the
        # engine polls via is_state_corrupted().  It is set ONLY after
        # _state_sanity_consec_threshold consecutive violations — a
        # single-epoch outlier could be a transient the chi² gate would
        # catch on its own, but sustained violations indicate the EKF
        # has wandered into a region the gate alone can't pull back from
        # (the day0602-pr125b-madhat 22:21 failure mode:
        # ekfStateSanityNoRecovery).  Cleared by reset() and on any
        # clean epoch.
        self._state_corrupted = False
        self._state_sanity_consec = 0
        self._state_sanity_consec_threshold = 3

        # OCXO-trusted observation gate.  If provided, Arm 4 (TICC)
        # innovations whose magnitude exceeds the DO's physically-
        # plausible short-τ movement are rejected before the EKF
        # update.  Anchored on the DO's freerun-characterized noise
        # floor, independent of the filter's own R/Q tuning.  See
        # peppar_fix.ocxo_trusted_gate.  Default None preserves
        # pre-existing behavior (chi² gate alone).
        self._ocxo_gate = ocxo_trusted_gate
        # Total time the filter has been alive (sum of dt across all
        # update() calls).  Used to feed `age_s` into the OCXO gate.
        self._total_age_s = 0.0

        # Innov-vs-control consistency monitor (TICC arm, where u enters
        # the prediction).  See peppar_fix/innov_control_monitor.py.
        self.innov_monitor = InnovControlMonitor()
        self.last_innov = 0.0
        self.last_S = 0.0
        # Per-arm innovation logging — populated each epoch by the
        # arms that fire.  Reset to None at the top of update().  The
        # arm-state CSV log reads these to emit innov_{name} + s_{name}
        # columns alongside the existing arm_{name}_used flag.
        #
        # Names match the arm-naming convention used in the CSV columns
        # and docstrings (see "six conditional measurement arms" below):
        #   ppp     — Arm 1, PPP dt_rx → x[0]
        #   qerr    — Arm 2, qErr slope → x[1]
        #   tdcp    — Arm 5, TDCP ensemble freq → x[1]
        #   extint  — Arm 3, F9T TIM-TM2 → x[2]
        #   pseudo  — Arm 6, holdover-integrated synthetic x[2]
        #   ticc    — Arm 4, TICC chA-chB (nonlinear coupling)
        #   cm_phase — Arm 7, ClockMatrix DPLL PHASE_STATUS → x[2]
        self.last_arm_innov: dict[str, float | None] = {
            'ppp': None, 'qerr': None, 'tdcp': None,
            'extint': None, 'do_phase': None, 'pseudo': None, 'ticc': None,
            'cm_phase': None,
        }
        self.last_arm_S: dict[str, float | None] = {
            'ppp': None, 'qerr': None, 'tdcp': None,
            'extint': None, 'do_phase': None, 'pseudo': None, 'ticc': None,
            'cm_phase': None,
        }
        # Per-arm pull attribution (pullAttributionLog).  Captures K
        # (Kalman gain vector) and would_pull (K · innov — state delta
        # this arm *would* apply if admitted) for every arm that fires
        # this epoch, REGARDLESS of whether downstream gates accept
        # the update.  Lets post-mortem analysis separate "information
        # the loop consumed" from "information the gate rejected":
        #   admitted_pull = would_pull * arm_X_used
        #   rejected_info = would_pull * (1 - arm_X_used)
        # Kalman gain recovered post-hoc as K = would_pull / innov.
        self.last_arm_K: dict[str, list[float] | None] = {
            'ppp': None, 'qerr': None, 'tdcp': None,
            'extint': None, 'do_phase': None, 'pseudo': None, 'ticc': None,
            'cm_phase': None,
        }
        self.last_arm_would_pull: dict[str, list[float] | None] = {
            'ppp': None, 'qerr': None, 'tdcp': None,
            'extint': None, 'do_phase': None, 'pseudo': None, 'ticc': None,
            'cm_phase': None,
        }
        self.last_arm_chi2: dict[str, float | None] = {
            'ppp': None, 'qerr': None, 'tdcp': None,
            'extint': None, 'do_phase': None, 'pseudo': None, 'ticc': None,
            'cm_phase': None,
        }
        self.last_ocxo_gate_rejected: bool = False
        self.last_ocxo_gate_reason: str = ""
        # softGateMidTau: the R-inflation factor the soft TICC gate applied
        # last epoch (1.0 = no inflation / hard-gate mode / in-budget
        # sample).  The applied Arm-4 pull is down-weighted vs the
        # full-weight would_pull_ticc_* by S_base/S_eff (≤1; → 1/inflation
        # only when R dominates S), so logging the factor keeps the actual
        # weighting recoverable alongside the full-weight would_pull columns.
        self.last_ticc_R_inflation: float = 1.0
        # Which TICC candidate the routed-qErr selector picked last
        # epoch ('ext'/'int'/'raw').  'int' when routing is disabled.
        self.last_ticc_route: str = 'int'
        # Cumulative route counters — production observability for the
        # routed-qErr router (Main's #79 follow-up: shipped only the
        # last_ticc_route field; without counters we can't see what the
        # router is doing on hardware).  Engine surfaces these to the
        # arm-state CSV and a periodic [ROUTED_QERR] log.
        self.n_route_ext: int = 0
        self.n_route_int: int = 0
        self.n_route_raw: int = 0
        # rx TCXO state must be initialized at construction from bootstrap
        # dt_rx to avoid a mid-run measurement model transition that
        # causes divergence.  If dt_rx wasn't available at construction,
        # stay in 2-state mode permanently (no mid-run switch).
        self._tcxo_initialized = initial_dt_rx_ns is not None
        # DO phase (x[2]) must be seeded from the first TICC measurement
        # before any Kalman update.  Without this, x[2]=0 creates a huge
        # innovation that the coupled H=[-1,0,-1,0] splits between x[0]
        # and x[2], corrupting the rx TCXO state.
        self._need_phc_seed = self._tcxo_initialized

    def _h_ticc(self, x):
        """Nonlinear TICC measurement function.

        z_ticc = -φ_phc - qerr(φ_tcxo)

        When rx TCXO not yet initialized, treat as z_ticc = -φ_do
        (no qerr correction — degrades to 2-state equivalent).
        """
        if self._tcxo_initialized:
            return -x[2] - _qerr(x[0], self.tick_ns)
        else:
            return -x[2]

    def _H_ticc(self, x):
        """Jacobian of h_ticc at x.

        When rx TCXO is initialized (PPP has provided dt_rx):
            H = [-1, 0, -1, 0] — full coupling, EKF resolves tick.
        When rx TCXO is NOT initialized:
            H = [0, 0, -1, 0] — degrade to 2-state (TICC observes
            φ_do only, ignoring unknown rx TCXO contribution).
        """
        if self._tcxo_initialized:
            return np.array([[-1.0, 0.0, -1.0, 0.0]])
        else:
            return np.array([[0.0, 0.0, -1.0, 0.0]])

    def reset(self, *, initial_freq=None, initial_dt_rx_ns=None):
        """In-process reset of the EKF state — re-init x, P, and the
        per-epoch innovation/route history.  Preserves the actuator
        command (via ``initial_freq``; default = self.freq, i.e. the
        last commanded adjfine) and ALL configured Q/R/B/F/L (this is
        a state reset, not a re-configure).

        Used by the binary layer (disciplineModeFsm #4) when the
        gross-fault detector fires — sustained distance_to_lock=1.0
        outside holdover.  The replacement for the exit-5 wrapper-
        relaunch path: the DO continues coasting at its last good freq
        while the filter rebuilds.  Cumulative diagnostic counters
        (n_route_*, innov_monitor) are preserved so the operator can
        see history across resets.

        ``initial_dt_rx_ns`` defaults to None ⇒ rebuild the wider
        (uninitialised-tcxo) bootstrap covariance.  Pass a known
        dt_rx_ns to keep the tighter 4-state covariance.

        POST-RESET ACTUATOR DYNAMICS: the command is preserved AT THE
        INSTANT OF RESET (self.freq / _last_u carry forward), but
        subsequent ``update()`` calls operate on a freshly-bootstrapped
        wider-P filter — the LQR can step normally as it re-acquires.
        L3 actuator rate-limit (``max_step_ppb``, #91) bounds the
        per-epoch swing during that recovery window; if it's disabled,
        the first few post-reset adjfine commands can be larger than
        steady-state.  This is intentional (the filter needs latitude
        to re-converge).
        """
        if initial_freq is None:
            initial_freq = self.freq
        phi_tcxo_init = (initial_dt_rx_ns if initial_dt_rx_ns is not None
                         else 0.0)
        # Same convention as __init__: crystal_freq drives x[3] (DO
        # freq estimate); _last_u drives the LQR step.  At steady
        # state both are initial_freq → f_do + u = 0.
        crystal_freq = initial_freq
        self.x = np.array(
            [phi_tcxo_init, 0.0, 0.0, -crystal_freq])
        # Re-anchor the do_freq_clamp nominal to the (post-reset) bootstrap.
        self._x3_nominal = -crystal_freq
        if initial_dt_rx_ns is not None:
            self.P = np.diag([100.0**2, 10.0**2, 5000.0**2, 1.0**2])
            self._tcxo_initialized = True
        else:
            self.P = np.diag([1e6, 100.0**2, 1000.0**2, 100.0**2])
            self._tcxo_initialized = False
        self._last_u = initial_freq
        self.freq = initial_freq
        # Clear per-epoch innovation/diagnostics so the next epoch's
        # arm-state CSV row reflects "no arm has fired yet on the
        # post-reset filter" (vs stale entries from the diverged run).
        self.last_innov = 0.0
        for _k in self.last_arm_innov:
            self.last_arm_innov[_k] = None
            self.last_arm_S[_k] = None
            self.last_arm_K[_k] = None
            self.last_arm_would_pull[_k] = None
            self.last_arm_chi2[_k] = None
        self.last_ocxo_gate_rejected = False
        self.last_ocxo_gate_reason = ""
        self.last_ticc_R_inflation = 1.0
        self.last_ticc_route = "int"
        # Clear the state-sanity guard: post-reset x is clean, so the
        # consecutive-violations counter starts fresh and the
        # _state_corrupted signal the engine polls drops back to False.
        self._state_corrupted = False
        self._state_sanity_consec = 0

    def realign_rx_clock(self, step_ns, *, post_step_sigma_ns=100.0):
        """Step the rx-clock PHASE state by a known, self-inflicted offset.

        clkResetRealignsEkfRxState — a GNSS receiver clock reset (e.g.
        F9T RXM-RAWX recStat.clkReset, an integer-ms realignment) steps
        the receiver's local clock by a known amount.  The PPP filter
        absorbs that step into its dt_rx state, so the dt_rx value the
        servo consumes via Arm 1 jumps by ``step_ns``.  Without a matching
        realign here the EKF sees a spurious ~21 ms innovation, splits it
        across the rx states (x[1] = f_rx spiked to ~1e6 on the MadHat
        2026-06-20 03:19:15Z event), and the state-sanity guard fires a
        needless [SERVO_RESET] reason=state_sanity.

        This is a CLEAN HANDOFF, not a fault: step the rx-TCXO PHASE
        state (x[0]) by exactly the same offset the dt_rx feed jumped, so
        the next PPP innovation is back to baseline noise.  ``step_ns``
        is the SIGNED change in dt_rx_ns (= +offset_m/c in ns), i.e. add
        it directly to x[0].

        Deliberately scoped:

        - x[0] (φ_rx): stepped by ``step_ns`` — it tracks dt_rx 1:1.
        - x[1] (f_rx): UNCHANGED — a clock reset is a phase step, not a
          frequency change; the rx-TCXO drift rate is physically the
          same.  Its covariance cross-term with x[0] is cleared so the
          inflated x[0] uncertainty doesn't bleed into f_rx on the next
          update.
        - x[2], x[3] (DO phase/freq): UNCHANGED.  The DO is physically
          unaffected by a receiver clock reset; injecting a step here
          would be a real, wrong disturbance.

        Covariance: inflate ONLY P[0,0] to ``post_step_sigma_ns²`` and
        zero x[0]'s cross-covariances (mirrors the constructor's
        bootstrap σ_x0 = 100 ns and the WNO clock-prior reset pattern in
        FixedPosFilter) — the discrete realign breaks the smooth
        rx-TCXO phase model, so post-step phase uncertainty is widened
        while the DO-state block of P is left untouched.

        No-op when ``step_ns`` is falsy (0 or None) so the engine can
        call this unconditionally each epoch.
        """
        if not step_ns:
            return
        self.x[0] += float(step_ns)
        # Clear x[0] cross-covariances, then set its variance to the
        # post-step uncertainty.  Leaves the x[1:]×x[1:] block intact —
        # the DO states (and f_rx) keep their pre-realign covariance.
        self.P[0, :] = 0.0
        self.P[:, 0] = 0.0
        self.P[0, 0] = float(post_step_sigma_ns) ** 2
        # If the rx-TCXO state was never initialized (no PPP dt_rx at
        # construction) the realign still anchors x[0] for any later
        # transition; harmless because Arm 1 / the coupled TICC arm only
        # consult x[0] once dt_rx is flowing.
        log.info(
            "[EKF] rx-clock realign: x[0] += %.1f ns (clkReset handoff), "
            "P[0,0] inflated to (%.0f ns)² — DO states x[2]/x[3] "
            "untouched", float(step_ns), float(post_step_sigma_ns))

    def is_state_corrupted(self) -> bool:
        """True iff sustained state-sanity violations have crossed the
        consecutive-threshold and the engine should request a reset
        through _request_servo_reset.  Cleared by reset()."""
        return self._state_corrupted

    def project_p22_coast(self, max_tau_s):
        """Predict-only P[2,2] (ns²) for coasting 1..max_tau_s seconds.

        Propagates P with NO measurement update — the open-loop coast —
        so P[2,2] grows by the frequency random-walk integrated into
        phase (F[2,3] = -dt couples x[3]→x[2]) plus the direct phase Q.
        Returns a list indexed by whole seconds: ``out[k]`` = projected
        P[2,2] after coasting k s (``out[0]`` = current P[2,2]).

        Feeds the longTauGnssCoupling coast-cap (coast_cap_from_p22) as
        a precomputed table (one O(max_tau) projection per epoch instead
        of re-propagating per candidate τ).  Honest only when Q is set
        from char (qFromCharPerActuator); an inflated Q over-grows P22
        and caps the coast spuriously short.  Uses base Q (no pull-in
        boost) — the coast-cap matters in the tracking regime.
        """
        P = self.P.copy()
        out = [float(P[2, 2])]
        for _ in range(int(max(0, max_tau_s))):
            P = self.F @ P @ self.F.T + self.Q * self.dt
            out.append(float(P[2, 2]))
        return out

    def _route_ticc_arm(self, x_pred, P_pred, ticc_diff_ns, ticc_qerr_ns,
                        R_base, R_lin):
        """Per-edge routed-qErr selector for the TICC arm.

        Returns (z_eff, h_pred, H, R, name): the chosen candidate's
        effective measurement, model, Jacobian, noise, and label.

        Priority order (first to pass the chi² gate wins; raw is the
        always-accepted floor):

          external — z = ticc_diff + ext_qerr, h = -x[2],
                     H = [0,0,-1,0], R = R_base.  Most informative:
                     clean x[2]-only update, no x[0] coupling.  The
                     receiver's own sub-tick report replaces the
                     filter's qerr(x[0]) estimate.  Only when
                     ticc_qerr_ns is available.
          internal — today's behavior: h = -x[2] - qerr(x[0]),
                     H = [-1,0,-1,0] (coupled), R = R_base + R_lin.
          raw      — z = ticc_diff, h = -x[2], H = [0,0,-1,0],
                     R = R_base + tick²/12 (sawtooth variance: the
                     uncorrected sub-tick qErr is ~uniform on
                     [-tick/2,+tick/2]).  Robust floor, always taken.

        COMPARATIVE gate (latestQErrChiSelect): pick the candidate with the
        SMALLEST chi² (a non-raw candidate must BEAT raw, not merely clear an
        absolute threshold).  Mandatory for the latest-unmatched qErr feed: an
        off-by-edge qErr (~3.3 ns RMS) lands at chi²≈17 ≪ 100, so an absolute
        gate would ACCEPT it and poison z; the comparative gate routes it to
        raw because raw's chi² is smaller.  Pure argmin (no margin) — see
        docs/latest-qerr-chi-select.md and scripts/proto_latest_qerr_chi.py.

        (The legacy absolute-gate path and the qVIR-gated v2 router were
        removed in retireQerrMatchingCode I-064807; comparative is now the
        only routing mode.)
        """
        H_x2 = np.array([[0.0, 0.0, -1.0, 0.0]])

        candidates = []
        if ticc_qerr_ns is not None:
            z_ext = ticc_diff_ns + ticc_qerr_ns
            candidates.append(('ext', z_ext, -x_pred[2], H_x2, R_base))
        candidates.append((
            'int', ticc_diff_ns, self._h_ticc(x_pred),
            self._H_ticc(x_pred), R_base + R_lin))
        candidates.append((
            'raw', ticc_diff_ns, -x_pred[2], H_x2,
            R_base + (self.tick_ns ** 2) / 12.0))

        # Comparative: smallest chi² wins (argmin).  A non-raw candidate
        # is selected only if it beats raw — otherwise raw is the floor.
        best = None  # (chi2, tuple)
        for cand in candidates:
            name, z_eff, h_pred, H, R = cand
            S = (H @ P_pred @ H.T).item() + R
            chi2 = (z_eff - h_pred) ** 2 / S if S > 0 else 0.0
            if best is None or chi2 < best[0]:
                best = (chi2, cand)
        name, z_eff, h_pred, H, R = best[1]
        return z_eff, h_pred, H, R, name

    def update(self, *,
               dt=1.0,
               dt_rx_ns=None, dt_rx_sigma_ns=None,
               qerr_freq_ppb=None, qerr_freq_sigma_ppb=None,
               tdcp_freq_ppb=None, tdcp_freq_sigma_ppb=None,
               extint_phase_ns=None, extint_sigma_ns=None,
               do_phase_ns=None, do_phase_sigma_ns=None,
               ticc_diff_ns=None, ticc_sigma_ns=None,
               pseudo_phase_ns=None, pseudo_phase_sigma_ns=None,
               cm_phase_ns=None, cm_phase_sigma_ns=None,
               ticc_qerr_ns=None,
               distance_to_lock=None):
        """Process one epoch with up to seven conditional measurement arms.

        Per docs/dofreq-est-measurement-ladder.md and
        docs/tdcp-servo-integration.md.  All arms are keyword-only.
        Each arm is gated on its own availability; the predict step
        always runs.  Order: PPP → qErr → TDCP → EXTINT → (pseudo) →
        TICC, chosen so the linear arms run before the nonlinear TICC
        update at the most recent state estimate.  Arm numbers are
        kept stable across history; names are the preferred reference
        going forward (see CSV column naming `arm_<name>_used`).

        Arm PPP    (1) observes  x[0] = rx TCXO phase from GPS
        Arm qErr   (2) observes  x[1] = rx TCXO frequency from GPS rate
        Arm TDCP   (5) observes  x[1] = rx TCXO frequency, cleaner signal
        Arm EXTINT (3) observes  x[2] = DO phase from GPS
        Arm pseudo (6) observes  x[2] = DO phase, synthetic from TDCP-
                                 integrated phase during holdover
        Arm TICC   (4) observes  -x[2] - qerr(x[0])  (nonlinear coupling)
        Arm cm_phase(7) observes x[2] = DO phase, on-chip ClockMatrix
                                 DPLL PHASE_STATUS (DO PPS vs F9T PPS)

        Args:
            dt: seconds since last correction.
            dt_rx_ns, dt_rx_sigma_ns:
                Arm 1 — PPP carrier-phase dt_rx and its 1-σ (~0.1 ns).
            qerr_freq_ppb, qerr_freq_sigma_ppb:
                Arm 2 — slope of unwrapped TIM-TP qErr stream as a
                direct rx-TCXO-frequency observation.  No FIFO edge
                matching required (this arm is on time-averaged
                qErr, not per-edge correction).
            tdcp_freq_ppb, tdcp_freq_sigma_ppb:
                Arm 5 — TDCP ensemble fractional-frequency offset
                (rx TCXO − GPS) in ppb, derived from carrier-phase
                deltas across SVs by `peppar_fix.tdcp_estimator`.
                Observes the same state as Arm 2 (x[1]) but with a
                much lower noise floor (TDEV(1s) ~15-35 ps =
                σ_y ~0.026 ppb vs qErr's ~0.3 ppb).  The Kalman
                filter does optimal-weighted fusion automatically;
                Arm 5 dominates x[1] when both are present.
            extint_phase_ns, extint_sigma_ns:
                Arm 3 — DO PPS phase from GPS time, from TIM-TM2
                (DO PPS wired to F9T EXTINT).  Sigma typically
                accEst from the message itself (~5–15 ns).
            ticc_diff_ns, ticc_sigma_ns:
                Arm 4 — TICC chA-chB raw differential (DO PPS edge
                vs rx TCXO PPS edge), no external qErr correction.
                The qerr() correction is computed from the filter's
                own state estimate of x[0], not from a separately-
                matched qErr stream.  Sigma defaults to the
                constructor's R_ticc if not provided.
            cm_phase_ns, cm_phase_sigma_ns:
                Arm 7 — on-chip ClockMatrix DPLL PHASE_STATUS = the
                DO PPS vs F9T PPS phase error read from the DPLL's PFD
                (50 ps resolution, σ ~0.05 ns).  Same observation state
                as Arm 3 (x[2], DO phase, linear H), but measured on the
                same chip that carries the DO — no TICC, no EXTINT wire.
                The TICC-free DO-phase observer for Timebeat OTC hosts.

        Returns:
            Frequency in ppb to apply via adjfine.
        """
        if dt != self.dt:
            self.dt = dt
            self.F[0, 1] = dt
            self.F[2, 3] = -dt
            self.B[2] = -dt

        # Accumulate total filter age for the OCXO gate.
        self._total_age_s += dt

        # ── Seed x[2] from first available DO-phase measurement ──
        # The seed avoids a large first-epoch innovation that would
        # otherwise corrupt other states via covariance correlations.
        # Prefer EXTINT (linear, σ ~ ns) over TICC (nonlinear, σ ~ sub-ns
        # but only valid once x[0] is well-known via PPP) — but use
        # whichever arrives first.  If neither arrives, leave x[2] at
        # its constructor default and let the LQR develop it.
        if self._need_phc_seed:
            if cm_phase_ns is not None:
                # On-chip ClockMatrix PHASE_STATUS — the cleanest linear x[2]
                # observer (~50 ps) and, on OTC hosts, usually the only one.
                self.x[2] = cm_phase_ns
                self.P[2, 2] = max(cm_phase_sigma_ns ** 2 if cm_phase_sigma_ns
                                   else 100.0, 100.0)
                self._need_phc_seed = False
            elif do_phase_ns is not None:
                # GNSSDO+ topology: the receiver clock IS the DO,
                # so dt_rx directly seeds the DO-phase state x[2].
                self.x[2] = do_phase_ns
                self.P[2, 2] = max(do_phase_sigma_ns ** 2 if do_phase_sigma_ns
                                   else 100.0, 100.0)
                self._need_phc_seed = False
            elif extint_phase_ns is not None:
                self.x[2] = extint_phase_ns
                self.P[2, 2] = max(extint_sigma_ns ** 2 if extint_sigma_ns
                                   else 100.0, 100.0)
                self._need_phc_seed = False
            elif ticc_diff_ns is not None:
                self.x[2] = -ticc_diff_ns - _qerr(self.x[0], self.tick_ns)
                self.P[2, 2] = 10.0 ** 2
                self._need_phc_seed = False

        # Reset per-epoch innovation log.  Arms that fire below will
        # populate their entries; arms that don't fire stay None.  The
        # CSV writer emits empty fields for None entries.
        for _k in self.last_arm_innov:
            self.last_arm_innov[_k] = None
            self.last_arm_S[_k] = None
        self.last_ocxo_gate_rejected = False
        self.last_ocxo_gate_reason = ""
        # midTauServoKnobs per-epoch flag (cleared each epoch; set by the
        # x[3] clamp below).
        self.last_do_freq_clamped = False

        # ── Adaptive Q: boost during pull-in ──
        do_phase_abs = abs(self.x[2])
        if do_phase_abs > 50.0:
            q_scale = 10.0
        elif do_phase_abs < 10.0:
            q_scale = 1.0
        else:
            q_scale = 1.0 + 9.0 * (do_phase_abs - 10.0) / 40.0
        Q_scaled = self.Q.copy()
        # Only boost DO states during pull-in (rx TCXO states converge from PPP)
        Q_scaled[2, 2] *= q_scale
        Q_scaled[3, 3] *= q_scale

        # ── EKF predict ──
        x_pred = self.F @ self.x + self.B * self._last_u
        P_pred = self.F @ self.P @ self.F.T + Q_scaled * dt

        # ── Arm 1: PPP dt_rx (linear, observes x[0]) ──
        # Order rationale: PPP first keeps x[0] tightly constrained
        # before nonlinear TICC linearizes around it.
        if dt_rx_ns is not None and dt_rx_sigma_ns is not None:
            x_pred, P_pred = self._kalman_linear_update(
                x_pred, P_pred,
                z=dt_rx_ns,
                H=self.H_ppp,
                R=dt_rx_sigma_ns ** 2,
                arm_name='ppp',
            )

        # ── Arm 2: qErr-as-frequency (linear, observes x[1]) ──
        # Slope of unwrapped qErr time series.  No FIFO edge matching:
        # qErr is treated as a time-averaged property of rx TCXO
        # frequency, not as an edge-by-edge correction.
        if qerr_freq_ppb is not None and qerr_freq_sigma_ppb is not None:
            x_pred, P_pred = self._kalman_linear_update(
                x_pred, P_pred,
                z=qerr_freq_ppb,
                H=np.array([[0.0, 1.0, 0.0, 0.0]]),
                R=qerr_freq_sigma_ppb ** 2,
                arm_name='qerr',
            )

        # ── Arm 5: TDCP ensemble freq (linear, observes x[1]) ──
        # Same observation state as Arm 2 (rx TCXO frequency), but
        # σ ~0.026 ppb vs qErr's ~0.3 ppb.  Order after Arm 2 so the
        # cleaner observation runs against the qErr-tightened P[1,1].
        # Sequential linear updates on the same H are order-invariant
        # in exact arithmetic; this order is purely for readability.
        if tdcp_freq_ppb is not None and tdcp_freq_sigma_ppb is not None:
            x_pred, P_pred = self._kalman_linear_update(
                x_pred, P_pred,
                z=tdcp_freq_ppb,
                H=np.array([[0.0, 1.0, 0.0, 0.0]]),
                R=tdcp_freq_sigma_ppb ** 2,
                arm_name='tdcp',
            )

        # ── Arm 3: EXTINT (linear, observes x[2]) ──
        # DO PPS edge timestamped by F9T via TIM-TM2.  Independent
        # of TICC; can stand alone as the only x[2] observer.
        if extint_phase_ns is not None and extint_sigma_ns is not None:
            x_pred, P_pred = self._kalman_linear_update(
                x_pred, P_pred,
                z=extint_phase_ns,
                H=np.array([[0.0, 0.0, 1.0, 0.0]]),
                R=extint_sigma_ns ** 2,
                arm_name='extint',
            )

        # ── Arm 8: do_phase (linear, observes x[2]) ──
        # The DO's own clock phase vs GPS, measured DIRECTLY — for
        # topologies where the receiver clock IS the disciplined oscillator
        # (GNSSDO+/SXT-D: the mosaic-T runs off the steered OCXO, so its
        # PPP carrier-phase dt_rx observes the DO phase, not a separate rx
        # TCXO).  Same linear H as Arm 3 (EXTINT).  In this mode the PPP
        # arm (Arm 1, which observes x[0]) is disabled by the caller so the
        # single dt_rx measurement is not double-counted.
        if do_phase_ns is not None and do_phase_sigma_ns is not None:
            x_pred, P_pred = self._kalman_linear_update(
                x_pred, P_pred,
                z=do_phase_ns,
                H=np.array([[0.0, 0.0, 1.0, 0.0]]),
                R=do_phase_sigma_ns ** 2,
                arm_name='do_phase',
            )

        # ── Arm 7: ClockMatrix PHASE_STATUS (linear, observes x[2]) ──
        # On-chip DPLL PFD = DO PPS vs F9T PPS, read over I2C (50 ps res).
        # Same H as Arm 3 (EXTINT) — an independent x[2] observer that needs
        # no TICC and no EXTINT wire (the TICC-free arm for OTC hosts).  An
        # actuator on the SAME DPLL (ClockMatrixComboActuator) is fine: the
        # PFD measures DO-vs-reference, the combo bus injects frequency, and
        # the EKF closes the loop just as it does for a DAC + EXTINT pair.
        if cm_phase_ns is not None and cm_phase_sigma_ns is not None:
            x_pred, P_pred = self._kalman_linear_update(
                x_pred, P_pred,
                z=cm_phase_ns,
                H=np.array([[0.0, 0.0, 1.0, 0.0]]),
                R=cm_phase_sigma_ns ** 2,
                arm_name='cm_phase',
            )

        # ── Arm 6: holdover pseudo-obs (linear, observes x[2]) ──
        # Synthetic DO phase from TDCP-integrated rx_TCXO phase + frozen
        # DO-vs-rx_TCXO offset.  Same H as Arm 3; wider σ that grows
        # with holdover duration.  Only present during HOLDOVER/REACQUIRED.
        if pseudo_phase_ns is not None and pseudo_phase_sigma_ns is not None:
            x_pred, P_pred = self._kalman_linear_update(
                x_pred, P_pred,
                z=pseudo_phase_ns,
                H=np.array([[0.0, 0.0, 1.0, 0.0]]),
                R=pseudo_phase_sigma_ns ** 2,
                arm_name='pseudo',
            )

        # ── Arm 4: TICC (nonlinear, couples x[0] and x[2]) ──
        # The qerr() inside h_ticc uses the filter's own x[0] estimate
        # — no externally-matched qErr stream consumed here.  This is
        # what makes TICC robust to FIFO-matching failures.
        #
        # Linearization contract (per docs/dofreq-est-measurement-ladder.md
        # "Sequential update order"): TICC runs LAST so qerr(x[0]) is
        # linearized at the post-PPP x[0] — when PPP is converged this
        # gives P[0,0] ≪ 1 ns², well inside one F9T tick interval (8 ns)
        # and the qerr Jacobian is well-defined.  When PPP is degraded
        # or skipped, the pre-update x[0] uncertainty can straddle a
        # tick boundary and the Jacobian sign-flip can pollute the
        # update; R_ticc inflation is the absorbing mechanism.  Do NOT
        # reorder this arm in front of the linear arms.
        if ticc_diff_ns is not None:
            R_base = (ticc_sigma_ns ** 2 if ticc_sigma_ns is not None
                      else float(self.R_ticc[0, 0]))
            # State-dependent linearization-error inflation (I-131253
            # bravo follow-up).  Standard EKF S = H·P·H^T + R captures
            # first-order linearization uncertainty; what's missing is
            # qerr()'s second-order error near tick boundaries.  When
            # σ(x[0]) is well below the tick interval, qerr() is locally
            # linear and R_lin → 0.  When σ(x[0]) approaches or exceeds
            # the tick, qerr() bounces between adjacent ticks within ±σ
            # and the prediction can be off by up to tick/2 even with
            # the correct state — which the linear Jacobian doesn't see.
            # R_lin absorbs that variance, capped at the half-tick worst
            # case.  Degrades gracefully: PPP-converged sub-ns σ → no
            # change; multi-tick σ → R_lin ≈ (tick/2)² which de-rates
            # Arm 4 to tick-floor accuracy and lets Arm 3 (EXTINT) carry
            # x[2] in that regime as designed.
            sigma_x0 = math.sqrt(max(0.0, float(P_pred[0, 0])))
            tick_third = self.tick_ns / 3.0
            R_lin = ((self.tick_ns / 2.0) ** 2 *
                     min(1.0, (sigma_x0 / tick_third) ** 2))
            if self._routed_qerr:
                z_eff, h_pred, H_ticc, R_scalar, _routed = \
                    self._route_ticc_arm(x_pred, P_pred, ticc_diff_ns,
                                         ticc_qerr_ns, R_base, R_lin)
                self.last_ticc_route = _routed
                if _routed == 'ext':
                    self.n_route_ext += 1
                elif _routed == 'int':
                    self.n_route_int += 1
                else:
                    self.n_route_raw += 1
            else:
                z_eff = ticc_diff_ns
                h_pred = self._h_ticc(x_pred)
                H_ticc = self._H_ticc(x_pred)
                R_scalar = R_base + R_lin
                self.last_ticc_route = 'int'
                # Legacy single-candidate path counts as 'int' for
                # parity with last_ticc_route's default.
                self.n_route_int += 1
            R_ticc = np.array([[R_scalar]])
            S = (H_ticc @ P_pred @ H_ticc.T + R_ticc).item()
            innov_ticc = z_eff - h_pred

            # Record innov + S BEFORE the state update.  See note on
            # _kalman_linear_update — recording before any downstream
            # gate skips the update means rejected innovations still
            # appear in the log, which is essential for post-mortems
            # on chi-squared-gate trips (ekfInnovationGate-bravo).
            self.last_innov = innov_ticc
            self.last_S = S
            self.last_arm_innov['ticc'] = innov_ticc
            self.last_arm_S['ticc'] = S

            # Pull attribution (pullAttributionLog): compute K + the
            # would-have-applied state delta once here, BEFORE the gate
            # checks.  This captures the rejected-information time
            # series that the chi² and OCXO gates suppress.  The K
            # vector below is reused for the actual state update when
            # the gates accept — no double computation.
            K_ticc = (P_pred @ H_ticc.T) / S
            _K_ticc_flat = K_ticc.flatten()
            self.last_arm_K['ticc'] = _K_ticc_flat.tolist()
            self.last_arm_would_pull['ticc'] = (
                _K_ticc_flat * innov_ticc).tolist()

            _chi2 = innov_ticc ** 2 / S if S > 0 else 0.0
            self.last_arm_chi2['ticc'] = _chi2

            # ── chi² gate: soft R-inflation (softGateMidTau) vs hard reject ──
            # The values applied to the state below — S_apply / _K_apply_flat
            # — default to the base S / K (hard-gate path, byte-identical).
            # The soft gate replaces them with R-inflated effective values
            # and never rejects on chi².
            S_apply = S
            _K_apply_flat = _K_ticc_flat
            self.last_ticc_R_inflation = 1.0
            if self._soft_ticc_gate:
                # softGateMidTau (I-092034): the legacy gate is BINARY —
                # χ²=innov²/S over _CHI2_GATE_THRESHOLD (10σ) skips the Arm-4
                # update entirely, so during a transient the DO coasts
                # open-loop through the blackout and the EKF SNAPS to the
                # accumulated error when the gate finally reopens → a mid-τ
                # (100–1000s) TDEV bulge (the only over-budget component per
                # two-site-sync-budget.md §3.2.1).  The soft gate instead
                # INFLATES R by max(1, χ²/K²) (K²=_CHI2_GATE_THRESHOLD) and
                # ALWAYS admits: a sample at the knee is used at ~full weight;
                # a 3× knee innovation is down-weighted ~9× but still pulls a
                # little — no open-loop blackout, no snap (Huber / chi²-clamped
                # soft gate).  χ²≤knee → factor 1.0 → identical to a normal
                # accept, so only outliers differ from the hard path.  The
                # OCXO physical gate (below) is independent and still rejects.
                inflation = max(1.0, _chi2 / _CHI2_GATE_THRESHOLD)
                if inflation > 1.0:
                    self.last_ticc_R_inflation = inflation
                    R_ticc_eff = R_ticc * inflation
                    S_apply = (H_ticc @ P_pred @ H_ticc.T + R_ticc_eff).item()
                    _K_apply_flat = ((P_pred @ H_ticc.T) / S_apply).flatten()
                    # log.debug, not info (main #217 review nit 1): a
                    # sustained mid-τ lockout episode produces many
                    # consecutive over-knee admits — the very events we
                    # study in the A/B — so info would flood run.log and
                    # bury them.  last_ticc_R_inflation carries the same
                    # signal into the arm-state CSV for analysis.
                    log.debug(
                        "[EKF] Arm 4 soft gate: χ²=%.0f > %.0f — R inflated "
                        "×%.1f, admitted down-weighted (|innov|=%.1f ns, "
                        "√S=%.1f ns)",
                        _chi2, _CHI2_GATE_THRESHOLD, inflation,
                        abs(innov_ticc), math.sqrt(S))
                chi2_reject = False
            else:
                chi2_reject = _chi2 > _CHI2_GATE_THRESHOLD
            # soloObserverChiGate (2026-06-09): the chi² gate rejects an
            # outlier only when there's a REDUNDANT DO-phase observer to
            # fall back on.  When EXTINT (Arm 3) is absent this epoch the
            # TICC is the SOLE observer of x[2] (DO phase) — and rejecting
            # the sole observer STARVES the state (no other arm corrects
            # it → drift → re-acquire cycle → step-contaminated TDEV; the
            # recurring single-observer failure, e.g. clkPoC3 TICC-only).
            # Bound the gate by OBSERVABILITY, not magnitude
            # ([[sole-observer-arm-cannot-be-deweighted]]): with no
            # fallback, accept the TICC update even on a large innovation —
            # it IS the truth about x[2].  A sustained real divergence is
            # still caught upstream by the outlier-ramp re-acquire, and the
            # OCXO physical gate below keeps its own sole-carrier override.
            # (Moot under the soft gate, which never sets chi2_reject — it
            # already admits the sole observer, just down-weighted.)
            ticc_sole_do_observer = (extint_phase_ns is None
                                     and cm_phase_ns is None)
            if chi2_reject and ticc_sole_do_observer:
                log.warning(
                    "[EKF] Arm 4 chi² (χ²=%.0f, |innov|=%.1f ns, √S=%.1f ns) "
                    "would reject, but TICC is the SOLE DO-phase observer "
                    "this epoch — accepting to avoid starving x[2] "
                    "(soloObserverChiGate)",
                    _chi2, abs(innov_ticc), math.sqrt(S))
                chi2_reject = False
            # OCXO-trusted physical gate.  Anchored on the DO's
            # characterized freerun short-τ noise (not on √S), so it
            # rejects observations that imply physically impossible
            # DO movement even when the filter's own R/Q has loosened.
            ocxo_reject = False
            if self._ocxo_gate is not None:
                # disciplineModeFsm increment #3: when distance_to_lock
                # is fed, the gate engages continuously (1−m loosening)
                # and observability-overrides for sole carriers — both
                # supersede the legacy min_age trigger.  None → legacy.
                accept, reason = self._ocxo_gate.evaluate(
                    innov_ns=innov_ticc, dt_s=self.dt,
                    age_s=self._total_age_s,
                    distance_to_lock=distance_to_lock)
                if not accept:
                    ocxo_reject = True
                    self.last_ocxo_gate_rejected = True
                    self.last_ocxo_gate_reason = reason
                    log.warning("[EKF] Arm 4 %s — update skipped", reason)

            if chi2_reject:
                log.warning(
                    "[EKF] Arm 4 chi² gate: |innov|=%.1f ns, √S=%.1f ns, "
                    "χ²=%.0f > %.0f — update skipped",
                    abs(innov_ticc), math.sqrt(S), _chi2,
                    _CHI2_GATE_THRESHOLD,
                )

            if not (chi2_reject or ocxo_reject):
                # _K_apply_flat / S_apply default to the base K / S (hard
                # gate) and become the R-inflated effective values under the
                # soft gate; the full-weight _K_ticc_flat is still recorded
                # in last_arm_would_pull for pull attribution.
                x_pred = x_pred + _K_apply_flat * innov_ticc
                P_pred = P_pred - np.outer(_K_apply_flat, _K_apply_flat) * S_apply
                self.innov_monitor.observe(self._last_u, innov_ticc, S_apply)

        self.x = x_pred
        self.P = P_pred

        # midTauServoKnobs x[3] anti-windup clamp (default off).  Bound the
        # DO-freq estimate to ±do_freq_clamp_ppb of its bootstrap nominal —
        # an OCXO/DAC has a bounded pull range, so x[3] physically cannot
        # wander further.  Catches a slow ramp INSIDE the gross state-sanity
        # bounds (±1e6 ppb), which is where delta's runaway lived (x[3]
        # ramped to a few hundred ppb, well under the sanity bound, and
        # walked the actuator to the +386 ppb rail one max_step at a time).
        # Applied before the LQR reads x[3] so u reflects the clamped state.
        if self._do_freq_clamp_ppb is not None:
            lo = self._x3_nominal - self._do_freq_clamp_ppb
            hi = self._x3_nominal + self._do_freq_clamp_ppb
            if self.x[3] < lo or self.x[3] > hi:
                self.x[3] = min(hi, max(lo, self.x[3]))
                self.n_do_freq_clamped += 1
                self.last_do_freq_clamped = True

        # State-sanity guard (I-074400 defense #2 + ekfStateSanityNoRecovery
        # follow-up): detect state corruption that the chi-squared gate
        # didn't catch (e.g., corruption through a non-TICC path or
        # accumulated drift past what any single-epoch gate would catch).
        #
        # Sustained-violation counter: increments on any violation
        # (any |x[i]| > bound), resets on a clean epoch.  Sets
        # _state_corrupted = True only after consec_threshold violations
        # in a row — distinguishes a transient single-epoch outlier
        # (the chi² gate handles those) from sustained corruption (the
        # engine polls is_state_corrupted() and routes to
        # _request_servo_reset).  On any violation, ALSO short-circuit
        # the LQR step and return self.freq unchanged — applying the
        # LQR output of a corrupt state is exactly what cascaded the
        # day0602-pr125b-madhat 22:21 runaway.
        _X_BOUNDS = (1e8, 1e4, 1e8, 1e6)  # ns, ppb, ns, ppb
        _violated = False
        for _i, (_xi, _bound) in enumerate(zip(self.x, _X_BOUNDS)):
            if abs(_xi) > _bound:
                log.error(
                    "[EKF] state-sanity: |x[%d]|=%.1f > %.0f — "
                    "state corrupted (consec=%d/%d)",
                    _i, abs(_xi), _bound,
                    self._state_sanity_consec + 1,
                    self._state_sanity_consec_threshold)
                _violated = True
                break
        if _violated:
            self._state_sanity_consec += 1
            if (self._state_sanity_consec
                    >= self._state_sanity_consec_threshold):
                self._state_corrupted = True
            # Skip LQR: applying the LQR output of a corrupt state
            # cascades the runaway.  Hold the last commanded freq;
            # the engine polls is_state_corrupted() and resets the
            # filter (preserving self.freq) once the consecutive
            # threshold is crossed.
            return self.freq
        else:
            self._state_sanity_consec = 0
            # Defense-in-depth (main's #126 review): clear the flag in
            # the clean-epoch branch too so it can't go sticky if any
            # path ever lands a clean epoch between threshold-crossing
            # and the engine's poll.  Synchronous per-epoch polling
            # today makes this unreachable, but the invariant is
            # cheaper to maintain than to debug.
            self._state_corrupted = False

        # ── LQR control ──
        # Only L[2] (φ_phc) and L[3] (f_phc) are nonzero.
        # Sign convention: return -u, engine applies -(-u) = u.
        u = -(self.L @ self.x)
        adjfine = -u

        adjfine = max(-self.max_ppb, min(self.max_ppb, adjfine))

        # L3: per-epoch rate limit on the actual applied adjfine.
        # The engine applies u (after double-negation of -u return);
        # self._last_u stores the previous u.  So clamp |u_new - u_prev|.
        # max_step_ppb is None → behavior unchanged from pre-Phase-C.
        if self.max_step_ppb is not None:
            u_prev = self._last_u
            if u > u_prev + self.max_step_ppb:
                u = u_prev + self.max_step_ppb
            elif u < u_prev - self.max_step_ppb:
                u = u_prev - self.max_step_ppb
            adjfine = -u

        self._last_u = u
        self.freq = adjfine
        return self.freq

    def _kalman_linear_update(self, x_pred, P_pred, *, z, H, R,
                              arm_name=None):
        """One linear Kalman measurement update.

        Sequential update helper for the linear arms (PPP, qErr, TDCP,
        EXTINT, pseudo).  H is (1×4), z and R are scalars.  Returns
        updated (x, P).

        When `arm_name` is provided, the innovation and predicted
        measurement variance are recorded to `self.last_arm_innov[name]`
        and `self.last_arm_S[name]` BEFORE the state update is applied.
        Recording before any downstream gate decides to skip the update
        means rejected innovations still appear in the per-epoch log,
        which is essential for post-mortems on chi-squared-gate trips.
        """
        innov = z - (H @ x_pred).item()
        S = (H @ P_pred @ H.T + R).item()
        K = (P_pred @ H.T) / S
        if arm_name is not None:
            self.last_arm_innov[arm_name] = innov
            self.last_arm_S[arm_name] = S
            # Pull attribution (pullAttributionLog): record K and the
            # would-have-applied state delta before the caller's gate
            # checks.  These are the same K and (K·innov) the update
            # below applies, so logged values exactly match the state
            # delta when the arm is admitted.
            _K_flat = K.flatten()
            self.last_arm_K[arm_name] = _K_flat.tolist()
            self.last_arm_would_pull[arm_name] = (_K_flat * innov).tolist()
            self.last_arm_chi2[arm_name] = (innov * innov / S) if S > 0 else 0.0
        x_new = x_pred + K.flatten() * innov
        P_new = P_pred - np.outer(K.flatten(), K.flatten()) * S
        return x_new, P_new

    @property
    def estimated_phase_ns(self):
        return self.x[2]

    @property
    def estimated_freq_ppb(self):
        return self.x[3]

    @property
    def estimated_tcxo_phase_ns(self):
        return self.x[0]

    @property
    def estimated_tcxo_freq_ppb(self):
        return self.x[1]

    @property
    def phase_uncertainty_ns(self):
        return math.sqrt(max(0, self.P[2, 2]))

    @property
    def freq_uncertainty_ppb(self):
        return math.sqrt(max(0, self.P[3, 3]))

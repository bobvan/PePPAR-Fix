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
                 ocxo_trusted_gate=None):
        self.max_ppb = max_ppb
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
        #         → more adjfine → speed up → reduce lateness)
        # L[3] = freq cancellation
        self.L = np.array([0.0, 0.0, -0.05, 1.0])

        self.freq = initial_freq
        # _last_u is the LQR u value.  Engine applies u as adjfine
        # (after double negation: servo returns -u, engine negates).
        # At startup, bootstrap set adjfine = initial_freq, so
        # the last applied u = initial_freq.
        self._last_u = initial_freq

        self._state_corrupted = False

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
        self.last_arm_innov: dict[str, float | None] = {
            'ppp': None, 'qerr': None, 'tdcp': None,
            'extint': None, 'pseudo': None, 'ticc': None,
        }
        self.last_arm_S: dict[str, float | None] = {
            'ppp': None, 'qerr': None, 'tdcp': None,
            'extint': None, 'pseudo': None, 'ticc': None,
        }
        self.last_ocxo_gate_rejected: bool = False
        self.last_ocxo_gate_reason: str = ""
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

    def update(self, *,
               dt=1.0,
               dt_rx_ns=None, dt_rx_sigma_ns=None,
               qerr_freq_ppb=None, qerr_freq_sigma_ppb=None,
               tdcp_freq_ppb=None, tdcp_freq_sigma_ppb=None,
               extint_phase_ns=None, extint_sigma_ns=None,
               ticc_diff_ns=None, ticc_sigma_ns=None,
               pseudo_phase_ns=None, pseudo_phase_sigma_ns=None):
        """Process one epoch with up to six conditional measurement arms.

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
            if extint_phase_ns is not None:
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
            R_ticc = np.array([[R_base + R_lin]])
            h_pred = self._h_ticc(x_pred)
            H_ticc = self._H_ticc(x_pred)
            S = (H_ticc @ P_pred @ H_ticc.T + R_ticc).item()
            innov_ticc = ticc_diff_ns - h_pred

            # Record innov + S BEFORE the state update.  See note on
            # _kalman_linear_update — recording before any downstream
            # gate skips the update means rejected innovations still
            # appear in the log, which is essential for post-mortems
            # on chi-squared-gate trips (ekfInnovationGate-bravo).
            self.last_innov = innov_ticc
            self.last_S = S
            self.last_arm_innov['ticc'] = innov_ticc
            self.last_arm_S['ticc'] = S

            _chi2 = innov_ticc ** 2 / S if S > 0 else 0.0
            chi2_reject = _chi2 > _CHI2_GATE_THRESHOLD
            # OCXO-trusted physical gate.  Anchored on the DO's
            # characterized freerun short-τ noise (not on √S), so it
            # rejects observations that imply physically impossible
            # DO movement even when the filter's own R/Q has loosened.
            ocxo_reject = False
            if self._ocxo_gate is not None:
                accept, reason = self._ocxo_gate.evaluate(
                    innov_ns=innov_ticc, dt_s=self.dt,
                    age_s=self._total_age_s)
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
                K = (P_pred @ H_ticc.T) / S
                x_pred = x_pred + K.flatten() * innov_ticc
                P_pred = P_pred - np.outer(K.flatten(), K.flatten()) * S
                self.innov_monitor.observe(self._last_u, innov_ticc, S)

        self.x = x_pred
        self.P = P_pred

        # State-sanity guard (I-074400 defense #2): detect state
        # corruption that the chi-squared gate didn't catch (e.g.,
        # corruption through a non-TICC path or accumulated drift).
        _X_BOUNDS = (1e8, 1e4, 1e8, 1e6)  # ns, ppb, ns, ppb
        for _i, (_xi, _bound) in enumerate(zip(self.x, _X_BOUNDS)):
            if abs(_xi) > _bound:
                log.error(
                    "[EKF] state-sanity: |x[%d]|=%.1f > %.0f — "
                    "state corrupted", _i, abs(_xi), _bound)
                self._state_corrupted = True
                break

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
        if arm_name is not None:
            self.last_arm_innov[arm_name] = innov
            self.last_arm_S[arm_name] = S
        K = (P_pred @ H.T) / S
        x_new = x_pred + K.flatten() * innov
        P_new = P_pred - np.outer(K.flatten(), K.flatten()) * S
        return x_new, P_new

    def reset(self, current_freq):
        self.x = np.array([0.0, 0.0, 0.0, 0.0])
        self.P = np.diag([1e6, 100.0**2, 1000.0**2, 100.0**2])
        self._last_u = 0.0
        self._tcxo_initialized = False
        self.freq = current_freq
        self.innov_monitor.reset()
        self.last_innov = 0.0
        self.last_S = 0.0

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

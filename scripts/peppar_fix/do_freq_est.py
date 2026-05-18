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
"""

import math
import numpy as np

from peppar_fix.innov_control_monitor import InnovControlMonitor


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
                 initial_dt_rx_ns=None, base_freq=None):
        self.max_ppb = max_ppb
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

        # Innov-vs-control consistency monitor (TICC arm, where u enters
        # the prediction).  See peppar_fix/innov_control_monitor.py.
        self.innov_monitor = InnovControlMonitor()
        self.last_innov = 0.0
        self.last_S = 0.0
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
               extint_phase_ns=None, extint_sigma_ns=None,
               ticc_diff_ns=None, ticc_sigma_ns=None):
        """Process one epoch with up to four conditional measurement arms.

        Per docs/dofreq-est-measurement-ladder.md.  All arms are
        keyword-only.  Each arm is gated on its own availability;
        the predict step always runs.  Order: PPP → qErr → EXTINT
        → TICC, chosen so the linear arms run before the nonlinear
        TICC update at the most recent state estimate.

        Arm 1 (PPP)     observes  x[0] = rx TCXO phase from GPS
        Arm 2 (qErr)    observes  x[1] = rx TCXO frequency from GPS rate
        Arm 3 (EXTINT)  observes  x[2] = DO phase from GPS
        Arm 4 (TICC)    observes  -x[2] - qerr(x[0])  (nonlinear coupling)

        Args:
            dt: seconds since last correction.
            dt_rx_ns, dt_rx_sigma_ns:
                Arm 1 — PPP carrier-phase dt_rx and its 1-σ (~0.1 ns).
            qerr_freq_ppb, qerr_freq_sigma_ppb:
                Arm 2 — slope of unwrapped TIM-TP qErr stream as a
                direct rx-TCXO-frequency observation.  No FIFO edge
                matching required (this arm is on time-averaged
                qErr, not per-edge correction).
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
            K = (P_pred @ H_ticc.T) / S
            innov_ticc = ticc_diff_ns - h_pred
            x_pred = x_pred + K.flatten() * innov_ticc
            P_pred = P_pred - np.outer(K.flatten(), K.flatten()) * S

            # Feed the innov-vs-control monitor.  TICC arm is the one
            # that closes the loop on x[2] (DO phase), so its innov is
            # what carries plant-model error.
            self.innov_monitor.observe(self._last_u, innov_ticc, S)
            self.last_innov = innov_ticc
            self.last_S = S

        self.x = x_pred
        self.P = P_pred

        # ── LQR control ──
        # Only L[2] (φ_phc) and L[3] (f_phc) are nonzero.
        # Sign convention: return -u, engine applies -(-u) = u.
        u = -(self.L @ self.x)
        adjfine = -u

        adjfine = max(-self.max_ppb, min(self.max_ppb, adjfine))

        self._last_u = u
        self.freq = adjfine
        return self.freq

    def _kalman_linear_update(self, x_pred, P_pred, *, z, H, R):
        """One linear Kalman measurement update.

        Sequential update helper for arms 1/2/3 (the linear arms).
        H is (1×4), z and R are scalars.  Returns updated (x, P).
        """
        innov = z - (H @ x_pred).item()
        S = (H @ P_pred @ H.T + R).item()
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

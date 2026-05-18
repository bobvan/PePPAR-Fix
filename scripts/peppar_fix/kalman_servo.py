"""Kalman filter + LQR servo for DO frequency steering.

See docs/glossary.md for term definitions (DO, rx TCXO, etc.).

Replaces the PI servo with an optimal controller that uses a 2-state
model of the DO (phase + frequency) and known noise profiles to
compute the minimum-noise, minimum-overshoot frequency correction
at each epoch.

State model (discrete, per-epoch):
    phase[n+1] = phase[n] + (freq[n] + u[n]) * dt
    freq[n+1]  = freq[n] + w_freq[n]

where:
    phase = DO phase error relative to GPS (ns), i.e. gnss_pps - do_pps
    freq  = DO frequency offset (ppb) — the slowly-drifting crystal rate
    u     = adjfine correction we apply (ppb)
    w_freq = frequency random walk (process noise from DO crystal drift)

Measurement model:
    z[n] = phase[n] + v[n]

where:
    z = TICC+qErr measurement (or EXTTS+qErr, or Carrier)
    v = measurement noise

The Kalman filter estimates [phase, freq] optimally.  The LQR gain
computes the adjfine that minimizes a weighted cost of phase error
and correction magnitude (= noise injected into the DO).

Why this beats PI:
    - PI has no plant model — it reacts to error without predicting
      the effect of its own corrections.  The integrator winds up
      during pull-in and overshoots.
    - Kalman predicts: "I just applied -30 ppb, so phase will change
      by -30 ns next epoch."  The next correction accounts for this.
    - LQR gain is critically damped by construction — fastest
      convergence with zero overshoot, given the noise profile.
    - When measurement noise R changes (TICC vs EXTTS), the gain
      adapts automatically.  No manual kp/ki tuning.

Noise parameters (from lab measurements 2026-04-09):
    sigma_meas_ns:  measurement noise σ (TICC+qErr: 0.178 ns,
                    EXTTS+qErr: 1.9 ns)
    sigma_phase_ns: DO phase noise per epoch (0.92 ns from adjfine
                    noise test — the TCXO's free-running jitter)
    sigma_freq_ppb: DO frequency random walk per epoch (~0.01 ppb,
                    from ADEV characterization at τ=10-100s)
"""

import math
import numpy as np
from scipy.linalg import solve_discrete_are

from peppar_fix.innov_control_monitor import InnovControlMonitor


class KalmanServo:
    """2-state Kalman filter + LQR for DO frequency steering.

    Drop-in interface match with PIServo: takes offset_ns, returns ppb.
    """

    def __init__(self, sigma_meas_ns=0.178, sigma_phase_ns=0.92,
                 sigma_freq_ppb=0.01, max_ppb=62_500_000.0,
                 initial_freq=0.0, q_weight=1.0, r_weight=1.0,
                 dead_zone_ppb=0.0):
        """
        Args:
            sigma_meas_ns: measurement noise σ (ns).
                0.178 for TICC+qErr, 1.9 for EXTTS+qErr.
            sigma_phase_ns: DO phase noise per epoch (ns).
                From adjfine noise test or DO characterization TDEV(1s).
            sigma_freq_ppb: DO frequency random walk per epoch (ppb).
                From ADEV at τ~10s: ADEV(10) × 10 / sqrt(10) ≈ 0.01.
            max_ppb: maximum adjfine magnitude.
            initial_freq: bootstrap adjfine to seed the frequency state.
            q_weight: scale factor on process noise Q (tune > 1 for
                more aggressive tracking, < 1 for smoother output).
            r_weight: scale factor on measurement noise R (tune > 1 to
                trust measurements less, < 1 to trust them more).
            dead_zone_ppb: minimum adjfine change to actually apply.
                Below this, hold the previous adjfine to avoid injecting
                noise for negligible corrections.  From adjfine noise
                characterization: corrections < 0.92 ppb add noise below
                the DO floor.  Default 0 (no dead zone).
        """
        self.max_ppb = max_ppb
        self.dead_zone_ppb = dead_zone_ppb
        self.dt = 1.0  # epoch interval; updated by update() dt param

        # State: [phase_ns, freq_ppb]
        self.x = np.array([0.0, initial_freq])

        # State transition: phase += (freq + u) * dt, freq += noise
        # (u is applied separately after the gain computation)
        self.F = np.array([[1.0, self.dt],
                           [0.0, 1.0]])
        self.B = np.array([[self.dt],
                           [0.0]])
        self.H = np.array([[1.0, 0.0]])  # measure phase only

        # Process noise — adaptive: start with 10× sigma_freq for fast
        # pull-in, taper to sigma_freq once phase error is small.
        self._sigma_phase_ns = sigma_phase_ns
        self._sigma_freq_ppb = sigma_freq_ppb
        self._q_weight = q_weight
        self.Q_base = np.diag([sigma_phase_ns ** 2,
                               sigma_freq_ppb ** 2])
        # Start with boosted Q for pull-in (10× freq tracking)
        self._Q_pullin = np.diag([sigma_phase_ns ** 2,
                                  (sigma_freq_ppb * 10) ** 2]) * q_weight
        self._Q_settled = self.Q_base * q_weight
        self.Q = self._Q_pullin  # start aggressive

        # Measurement noise
        self.R_base = np.array([[sigma_meas_ns ** 2]])
        self.R = self.R_base * r_weight

        # Initialize covariance — start uncertain
        self.P = np.diag([1000.0 ** 2, 100.0 ** 2])

        # Precompute steady-state LQR gain for the control law.
        # Cost: J = Σ (x'Qc x + u'Rc u)
        # Qc weights state error, Rc weights control effort (noise cost).
        #
        # From the adjfine noise test (2026-04-09): corrections < 1 ppb
        # add unmeasurable noise to the DO.  So the control cost is VERY
        # low — we should correct aggressively whenever the Kalman filter
        # sees an error.  Rc is set small to reflect this.
        #
        # Qc penalizes both phase error and frequency error:
        #   Qc[0,0] = phase error weight (1/ns² units)
        #   Qc[1,1] = frequency error weight (1/ppb² units)
        # We weight phase strongly (we want sub-ns tracking) and
        # frequency moderately (we want to track the TCXO drift).
        self.Qc = np.diag([1.0 / (sigma_phase_ns ** 2 + 1e-30),
                           1.0 / (sigma_freq_ppb ** 2 + 1e-30)])
        # Rc: control cost.  From adjfine noise test, the noise
        # cost of a 1 ppb correction is ~1 ns (= sigma_phase).
        # So Rc = sigma_phase² means "a 1 ppb correction costs
        # about as much as one epoch of DO noise."  This produces
        # a moderately aggressive controller.
        self.Rc = np.array([[sigma_phase_ns ** 2]])

        try:
            P_lqr = solve_discrete_are(self.F, self.B, self.Qc, self.Rc)
            self.L = np.linalg.inv(
                self.B.T @ P_lqr @ self.B + self.Rc
            ) @ self.B.T @ P_lqr @ self.F
            # Sanity: for a pure integrator, L[1] must be ≥ 1.0 to
            # fully cancel the estimated frequency.  L[1] < 1.0 causes
            # runaway drift because the partial correction gets
            # attributed to an even higher frequency estimate.
            if self.L[0, 1] < 1.0:
                self.L[0, 1] = 1.0
        except Exception:
            pass

        # If Riccati didn't converge or produced unstable gains,
        # use analytically-derived gains for a discrete integrator:
        #   L[0] = phase gain (ppb per ns of error) — controls
        #          pull-in speed and settled-state noise injection.
        #          At 0.05: 5000 ns offset → 250 ppb correction (fast pull-in);
        #          1 ns settled error → 0.05 ppb (invisible, per adjfine test).
        #   L[1] = frequency gain — must be 1.0 for full cancellation
        #          of the estimated TCXO drift rate.
        if not hasattr(self, 'L') or self.L is None:
            self.L = np.array([[0.05, 1.0]])

        self.freq = initial_freq
        self._last_u = 0.0

        # Innovation-vs-control consistency monitor.  Pure observer;
        # fed in update() after the primary measurement-update step.
        # Catches plant-model errors (DAC scaling/sign, B-matrix coeff)
        # epoch-by-epoch rather than waiting for actuator-near-rail
        # symptoms.  See peppar_fix/innov_control_monitor.py.
        self.innov_monitor = InnovControlMonitor()
        self.last_innov = 0.0
        self.last_S = 0.0

    def update(self, offset_ns, dt=1.0,
              delta_dt_rx_ns=None, dt_rx_sigma_ns=None):
        """Process one measurement. Returns frequency adjustment in ppb.

        Args:
            offset_ns: measured phase offset (ns) from TICC+qErr or EXTTS.
            dt: seconds since last correction (discipline interval).
            delta_dt_rx_ns: time-differenced PPP dt_rx (ns).
                = dt_rx[n] - dt_rx[n-1]: how much the TCXO-to-GPS offset
                changed in one epoch.  This constrains the frequency state
                without introducing absolute bias (the differencing cancels
                the inter-oscillator drift rate D).
            dt_rx_sigma_ns: 1-sigma uncertainty of dt_rx from PPP (~0.1 ns).
                The differenced measurement noise is sqrt(2) * dt_rx_sigma.

        Returns:
            Frequency in ppb to apply via adjfine.
        """
        # Update dt-dependent matrices if interval changed
        if dt != self.dt:
            self.dt = dt
            self.F[0, 1] = dt
            self.B[0, 0] = dt

        # ── Adaptive Q: taper from pull-in to settled ──
        # When |phase error| > 50 ns, use pull-in Q (10× freq tracking).
        # When |phase error| < 10 ns, use settled Q (nominal).
        # Smooth blend in between.
        phase_abs = abs(self.x[0])
        if phase_abs > 50.0:
            self.Q = self._Q_pullin
        elif phase_abs < 10.0:
            self.Q = self._Q_settled
        else:
            # Linear blend between 10 and 50 ns
            alpha = (phase_abs - 10.0) / 40.0  # 0 at 10ns, 1 at 50ns
            self.Q = (1 - alpha) * self._Q_settled + alpha * self._Q_pullin

        # ── Kalman predict ──
        # Account for the control we applied last epoch
        x_pred = self.F @ self.x + self.B.flatten() * self._last_u
        P_pred = self.F @ self.P @ self.F.T + self.Q * dt

        # ── Kalman update: primary measurement (TICC+qErr) ──
        z = offset_ns
        innovation = z - (self.H @ x_pred).item()
        S = (self.H @ P_pred @ self.H.T + self.R).item()
        K = (P_pred @ self.H.T) / S  # 2×1 vector

        self.x = x_pred + K.flatten() * innovation
        self.P = P_pred - np.outer(K.flatten(), K.flatten()) * S

        # Feed the innov-vs-control consistency monitor.  The control
        # input is self._last_u (what we applied last epoch, which the
        # predict step assumed had B·u effect).
        self.innov_monitor.observe(self._last_u, innovation, S)
        self.last_innov = innovation
        self.last_S = S

        # ── Kalman update: time-differenced dt_rx (frequency constraint) ──
        # Δdt_rx = dt_rx[n] - dt_rx[n-1] measures how much the TCXO-to-GPS
        # offset changed in one epoch.  For the PHC, the expected phase
        # change is (adjfine + freq_drift) * dt.  The TCXO changed by
        # Δdt_rx.  So the TICC measurement should have changed by:
        #   Δticc_expected = (adjfine + freq_drift) * dt - Δdt_rx
        #
        # Rearranging: Δdt_rx + Δticc_expected = (adjfine + freq_drift) * dt
        # This constrains freq_drift.  The measurement model:
        #   z_freq = Δdt_rx / dt  (observed TCXO rate, ppb)
        #   H_freq = [0, 1]      (observes frequency state)
        #   expected = freq state (the PHC crystal drift the filter tracks)
        #
        # But we need to account for the fact that the filter's freq state
        # tracks the PHC drift, while Δdt_rx tracks the TCXO.  The
        # difference is the inter-oscillator rate.  Since the filter
        # already accounts for adjfine in the predict step (via B*u),
        # the innovation is:
        #   innovation = -(Δdt_rx / dt) - freq_state
        # because Δdt_rx > 0 means the TCXO fell behind GPS (its phase
        # increased), which looks like the PHC got relatively ahead
        # (negative contribution to the PHC frequency error we track).
        #
        # Noise: sqrt(2) * dt_rx_sigma (differencing two independent
        # samples), converted to ppb by dividing by dt.
        if delta_dt_rx_ns is not None and dt_rx_sigma_ns is not None:
            H_freq = np.array([[0.0, 1.0]])  # observes frequency
            freq_obs = -delta_dt_rx_ns / dt  # TCXO rate → PHC freq constraint
            R_freq = np.array([[(dt_rx_sigma_ns * math.sqrt(2) / dt) ** 2]])
            innov_f = freq_obs - (H_freq @ self.x).item()
            S_f = (H_freq @ self.P @ H_freq.T + R_freq).item()
            K_f = (self.P @ H_freq.T) / S_f
            self.x = self.x + K_f.flatten() * innov_f
            self.P = self.P - np.outer(K_f.flatten(), K_f.flatten()) * S_f

        # ── LQR control ──
        # u = -L @ x gives the total adjfine to apply.
        # L[0] × phase: proportional correction for phase error.
        # L[1] × freq: full cancellation of estimated TCXO drift.
        #
        # SIGN CONVENTION: the engine calls adjfine = -servo.update(),
        # expecting update() to return a value with the OPPOSITE sign
        # of the desired adjfine (PIServo convention: positive output
        # = "clock is ahead, slow down" = negative adjfine).  So we
        # return the negative of our computed adjfine here, and the
        # engine's negation makes it correct.
        u = -(self.L @ self.x).item()
        adjfine = -u  # engine will negate this back

        # Clamp
        adjfine = max(-self.max_ppb, min(self.max_ppb, adjfine))

        # Dead zone: if the change from current adjfine is below the
        # threshold, hold the previous value.  This prevents random-walk
        # noise accumulation from sub-floor corrections.  The Kalman
        # filter still updates its state normally (so the estimate stays
        # optimal), but the actuator doesn't move.
        if self.dead_zone_ppb > 0 and abs(adjfine - self.freq) < self.dead_zone_ppb:
            adjfine = self.freq
            u = -adjfine  # keep _last_u consistent

        self._last_u = u
        self.freq = adjfine
        return self.freq

    def reset(self, current_freq):
        """Reset for bumpless transfer (e.g., after PHC re-bootstrap)."""
        self.x = np.array([0.0, -current_freq])
        self.P = np.diag([1000.0 ** 2, 100.0 ** 2])
        self._last_u = 0.0
        self.freq = current_freq
        self.innov_monitor.reset()
        self.last_innov = 0.0
        self.last_S = 0.0

    @property
    def estimated_phase_ns(self):
        """Current best estimate of phase error (ns)."""
        return self.x[0]

    @property
    def estimated_freq_ppb(self):
        """Current best estimate of frequency offset (ppb)."""
        return self.x[1]

    @property
    def phase_uncertainty_ns(self):
        """1-σ uncertainty of the phase estimate (ns)."""
        return math.sqrt(max(0, self.P[0, 0]))

    @property
    def freq_uncertainty_ppb(self):
        """1-σ uncertainty of the frequency estimate (ppb)."""
        return math.sqrt(max(0, self.P[1, 1]))

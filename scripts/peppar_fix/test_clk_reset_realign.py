"""clkResetRealignsEkfRxState (I-064109-main).

A GNSS receiver clock reset (F9T RXM-RAWX recStat.clkReset, a ~21 ms
integer-millisecond realignment) is absorbed into the PPP filter's
dt_rx state.  The dt_rx the servo consumes via Arm 1 therefore jumps by
the same amount.  Without a matching realign of the DOFreqEst EKF's
rx-clock state, the EKF sees a spurious ~21 ms innovation, the rx-freq
state (x[1]) spikes past the state-sanity bound, and a needless
[SERVO_RESET] reason=state_sanity fires (observed MadHat 2026-06-20
03:19:15Z).

These tests:
  (a) WITHOUT realign — reproduce the |x[1]|>1e4 state-sanity trip;
  (b) WITH realign — the EKF keeps tracking, no state-sanity trip;
  (c) the DO states (x[2]/x[3]) are unchanged by the realign itself.
"""

import numpy as np
import unittest

from peppar_fix.do_freq_est import DOFreqEst


# F9T clkReset is an integer-ms step.  -21 ms ≈ -6 295 640 m of PR
# absorbed into dt_rx; in ns the dt_rx feed jumps by -21e6 ns.  Use the
# magnitude reported on the MadHat event.
CLK_RESET_STEP_NS = -21.0e6


def _locked_filter():
    """Construct a DOFreqEst and drive it to a converged, tracking lock
    with synthetic PPP dt_rx (Arm 1) + TICC (Arm 4) measurements.

    The rx-TCXO drifts at a steady rate so x[0]/x[1] develop a realistic
    converged cross-covariance (P[0,1] != 0) — that off-diagonal is what
    splits a dt_rx step into the x[1] spike.
    """
    # initial_dt_rx_ns engages the 4-state (tcxo-initialized) path.
    f = DOFreqEst(
        sigma_ticc_ns=0.060,
        sigma_do_phase_ns=0.92,
        sigma_do_freq_ppb=0.01,
        sigma_tcxo_phase_ns=2.0,
        sigma_tcxo_freq_ppb=0.1,
        initial_dt_rx_ns=0.0,
        initial_freq=0.0,
    )
    # Seed x[2] from the first TICC measurement (mirrors the engine).
    f._need_phc_seed = True
    rx_rate_ppb = 20.0          # rx TCXO drifts +20 ppb (≈ +20 ns/s)
    rx_phase = 0.0
    for _ in range(400):
        rx_phase += rx_rate_ppb  # ns/s at 1 Hz
        # DO held at GPS (φ_do ≈ 0) → TICC chA-chB ≈ -φ_do - qerr(φ_rx).
        ticc = -0.0 - (rx_phase - round(rx_phase / 8.0) * 8.0)
        f.update(
            dt=1.0,
            dt_rx_ns=rx_phase, dt_rx_sigma_ns=0.1,
            ticc_diff_ns=ticc, ticc_sigma_ns=0.060,
        )
    return f, rx_phase, rx_rate_ppb


class TestClkResetReproducesSpike(unittest.TestCase):
    """(a) Without the realign, a clkReset-sized dt_rx step trips the
    state-sanity guard over consecutive epochs."""

    def test_spike_without_realign(self):
        f, rx_phase, rx_rate = _locked_filter()
        # Sanity: converged and not corrupted before the event.
        self.assertFalse(f.is_state_corrupted())
        self.assertLess(abs(f.x[1]), 1e4)

        # Receiver clock reset: dt_rx steps by CLK_RESET_STEP_NS.  The
        # PPP filter absorbed it; the servo is NOT told.  Keep feeding
        # the post-step dt_rx for several epochs (the rx keeps drifting
        # from the new anchor).
        x1_spiked = False
        for k in range(1, 6):
            rx_phase += rx_rate
            stepped = rx_phase + CLK_RESET_STEP_NS
            ticc = -0.0 - (rx_phase - round(rx_phase / 8.0) * 8.0)
            f.update(
                dt=1.0,
                dt_rx_ns=stepped, dt_rx_sigma_ns=0.1,
                ticc_diff_ns=ticc, ticc_sigma_ns=0.060,
            )
            if abs(f.x[1]) > 1e4:
                x1_spiked = True

        self.assertTrue(
            x1_spiked,
            "without realign, the dt_rx step must drive |x[1]| past the "
            "state-sanity bound (reproduces the MadHat spike)")
        self.assertTrue(
            f.is_state_corrupted(),
            "sustained state-sanity violations must trip the corrupted "
            "flag → needless servo reset")


class TestClkResetRealignKeepsTracking(unittest.TestCase):
    """(b) With the realign forwarded to the EKF, the step is a clean
    handoff: no spurious innovation, no state-sanity trip."""

    def test_no_trip_with_realign(self):
        f, rx_phase, rx_rate = _locked_filter()
        x1_max = 0.0
        for k in range(1, 6):
            rx_phase += rx_rate
            stepped = rx_phase + CLK_RESET_STEP_NS
            ticc = -0.0 - (rx_phase - round(rx_phase / 8.0) * 8.0)
            # First post-reset epoch: realign the EKF rx-clock state by
            # the SAME step the dt_rx feed jumped (engine forwards
            # filt.last_clk_realign_m → ns here).
            if k == 1:
                f.realign_rx_clock(CLK_RESET_STEP_NS)
            f.update(
                dt=1.0,
                dt_rx_ns=stepped, dt_rx_sigma_ns=0.1,
                ticc_diff_ns=ticc, ticc_sigma_ns=0.060,
            )
            x1_max = max(x1_max, abs(f.x[1]))

        self.assertLess(
            x1_max, 1e4,
            "with realign, |x[1]| stays within the state-sanity bound")
        self.assertFalse(
            f.is_state_corrupted(),
            "with realign, no state-sanity trip → no needless servo reset")

    def test_x0_follows_step(self):
        """The realign steps x[0] by exactly the offset, so the next PPP
        innovation is baseline-sized, not ~21 ms."""
        f, rx_phase, _ = _locked_filter()
        x0_before = f.x[0]
        f.realign_rx_clock(CLK_RESET_STEP_NS)
        self.assertAlmostEqual(f.x[0], x0_before + CLK_RESET_STEP_NS,
                               delta=1.0)


class TestRealignScope(unittest.TestCase):
    """(c) The realign touches ONLY the rx-clock states — the DO is
    physically unaffected by a receiver clock reset."""

    def test_do_states_unchanged(self):
        f, _, _ = _locked_filter()
        x2_before = float(f.x[2])
        x3_before = float(f.x[3])
        f.realign_rx_clock(CLK_RESET_STEP_NS)
        self.assertEqual(f.x[2], x2_before,
                         "DO phase x[2] must be untouched by realign")
        self.assertEqual(f.x[3], x3_before,
                         "DO freq x[3] must be untouched by realign")

    def test_rx_freq_state_unchanged_by_realign(self):
        """A clock reset is a phase step, not a frequency change: x[1]
        (rx-TCXO drift rate) is left as-is by the realign itself."""
        f, _, _ = _locked_filter()
        x1_before = float(f.x[1])
        f.realign_rx_clock(CLK_RESET_STEP_NS)
        self.assertEqual(f.x[1], x1_before,
                         "rx-freq x[1] must not be stepped by realign")

    def test_do_covariance_block_preserved(self):
        """Only x[0]'s covariance row/col is reset; the DO-state block
        of P keeps its pre-realign values."""
        f, _, _ = _locked_filter()
        p22_before = float(f.P[2, 2])
        p33_before = float(f.P[3, 3])
        p23_before = float(f.P[2, 3])
        f.realign_rx_clock(CLK_RESET_STEP_NS, post_step_sigma_ns=100.0)
        self.assertEqual(f.P[2, 2], p22_before)
        self.assertEqual(f.P[3, 3], p33_before)
        self.assertEqual(f.P[2, 3], p23_before)
        # x[0] cross-covariances cleared, variance inflated.
        self.assertEqual(f.P[0, 1], 0.0)
        self.assertEqual(f.P[0, 2], 0.0)
        self.assertAlmostEqual(f.P[0, 0], 100.0 ** 2, places=6)

    def test_zero_step_is_noop(self):
        """The engine calls realign unconditionally; a 0/None step must
        do nothing (no covariance disturbance)."""
        f, _, _ = _locked_filter()
        x_before = f.x.copy()
        p_before = f.P.copy()
        f.realign_rx_clock(0.0)
        np.testing.assert_array_equal(f.x, x_before)
        np.testing.assert_array_equal(f.P, p_before)
        f.realign_rx_clock(None)
        np.testing.assert_array_equal(f.x, x_before)
        np.testing.assert_array_equal(f.P, p_before)


if __name__ == "__main__":
    unittest.main()

"""servoSimSingleOsc (I-122448): the single-oscillator servo_sim mode.

Models a GNSSDO+/SXT-D where the DO clocks the receiver — one oscillator,
the mosaic PPP dt_rx IS the DO-phase observation (sole observer, fed to the
x[2] arm), and the actuator→observation loop is closed by the plant.

The mode is the faithful single-oscillator TOPOLOGY (sole DO-phase observer,
no rx TCXO) plus two hardware-realism knobs — actuator_gain_true (imperfect
steering gain) and single_osc_obs_lag_s (the mosaic's internal clock-filter
lag). These tests lock in the topology + knobs + the honest finding that the
mode is stable at normal cadence and that its delay-driven instabilities are
loose-Q-driven — the OPPOSITE of delta's reported tight-Q rail, i.e. the sim
does not reproduce delta's rail (a perturbation-triggered hardware event; see
docs/mid-tau-servo-knobs.md "Single-oscillator mode").
"""
import logging
import unittest

import numpy as np

from peppar_fix.servo_sim import preset, ClosedLoopSim

logging.disable(logging.CRITICAL)

# SXT-D-like: stable OCXO DO, clean sole observer, tight measured-Q.
_SXTD = dict(single_oscillator=True, single_osc_obs_sigma_ns=0.16,
             sigma_do_freq_ppb=8.4e-5, sigma_do_phase_ns=0.0564,
             do_rwfm_ppb_per_sqrt_s=0.005, do_wfm_ppb=0.01, do_wpm_ns=0.01)


def _run(duration_s=1600.0, **ov):
    return ClosedLoopSim(preset("piface-current", duration_s=duration_s,
                                **{**_SXTD, **ov})).run()


class TestSingleOscTopology(unittest.TestCase):

    def test_sole_observer_only_do_phase_emitted(self):
        """In single-osc mode the sim emits ONLY the DO-phase obs (the x[2]
        arm), regardless of the two-oscillator use_* flags."""
        sim = ClosedLoopSim(preset("piface-current", duration_s=10.0,
                                   use_ppp=True, use_qerr=True, use_ticc=True,
                                   **_SXTD))
        meas = sim._emit()
        self.assertIn("extint_phase_ns", meas)      # the sole DO-phase obs
        for k in ("dt_rx_ns", "qerr_freq_ppb", "ticc_diff_ns", "tdcp_freq_ppb"):
            self.assertNotIn(k, meas)                # no rx-TCXO / TICC arms

    def test_normal_cadence_is_stable(self):
        """At normal cadence (correct every epoch) the single-osc loop is
        stable — the tight measured-Q does NOT rail on its own."""
        r = _run(coast_interval_s=1.0)
        self.assertFalse(r.diverged())
        m = r.t_s >= 200
        self.assertLess(float(np.nanmax(np.abs(r.adjfine_ppb[m]))), 20.0)


class TestSingleOscStability(unittest.TestCase):
    """The mode CAN destabilize under extreme loop delay (long coast) — a
    real property that bounds the safe operating region — but this is a
    generic delay instability, NOT single-oscillator-specific, and its
    Q-dependence is the OPPOSITE of delta's report."""

    def test_long_coast_rails(self):
        r = _run(duration_s=3600.0, coast_interval_s=60.0)
        self.assertTrue(r.diverged())

    def test_short_coast_stable(self):
        r = _run(duration_s=3600.0, coast_interval_s=15.0)
        self.assertFalse(r.diverged())

    def test_delta_tightQ_rail_does_NOT_reproduce(self):
        """The honest negative result: at normal cadence, tight measured-Q
        does not rail (with or without receiver-clock lag) — the sim, driving
        the real DOFreqEst, does not reproduce delta's tight-Q rail.  If any
        obs-lag makes tight-Q rail at normal cadence, this canary fires and
        the finding must be revisited."""
        for lag in (0.0, 10.0, 30.0):
            r = _run(duration_s=4000.0, coast_interval_s=1.0,
                     single_osc_obs_lag_s=lag, sigma_do_freq_ppb=8.4e-5)
            self.assertFalse(r.diverged(),
                             f"tight-Q railed at lag={lag} — revisit finding")


class TestActuatorGain(unittest.TestCase):

    def test_gain_true_scales_plant(self):
        """actuator_gain_true scales the DO's actual response; 1.0 is the
        matched (default) plant."""
        r1 = _run(duration_s=800.0, actuator_gain_true=1.0)
        r2 = _run(duration_s=800.0, actuator_gain_true=2.0)
        # Different true gain → different trajectory (plant responds
        # differently to the same commands).
        self.assertGreater(
            float(np.max(np.abs(r1.phi_do_true_ns - r2.phi_do_true_ns))), 0.0)


if __name__ == "__main__":
    unittest.main()

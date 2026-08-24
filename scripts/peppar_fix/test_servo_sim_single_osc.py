"""servoSimSingleOsc (I-122448): the single-oscillator servo_sim mode.

Models a GNSSDO+/SXT-D where the DO clocks the receiver — one oscillator,
the mosaic PPP dt_rx IS the DO-phase observation (sole observer, fed to the
x[2] arm), and the actuator→observation loop is closed by the plant.

The mode is the faithful single-oscillator TOPOLOGY (sole DO-phase observer,
no rx TCXO) plus hardware-realism knobs: actuator_gain_true, single_osc_obs_
lag_s, the mosaic clock filter (single_osc_mosaic_filter / mosaic_alpha /
mosaic_beta), and processing_stall.

Findings locked in here (see docs/mid-tau-servo-knobs.md "Single-oscillator
mode"):
  * the BARE topology (servo observes the DO directly) is well-damped and
    does NOT reproduce delta's rail — a stall/glitch is absorbed, and the
    only instability is loose-Q-driven (opposite of delta's report), so the
    rail is NOT tight-Q underdamping;
  * the load-bearing single-oscillator dynamic is the MOSAIC CLOCK FILTER
    (the observation is the mosaic's clock-bias state, a 2nd filter in series
    with our servo — grounded in the captures where our_dt_rx_ns ==
    mosaic_rxclkbias_ns).  With that cascade + a processing stall, the sim
    reproduces delta's stable → limit-cycle → rail spectrum, tuned by the
    mosaic filter bandwidth.
"""
import logging
import unittest

import numpy as np

from peppar_fix.servo_sim import preset, ClosedLoopSim

# The sim drives DOFreqEst into its `[EKF] state-sanity` tripwire thousands
# of times per file (by design — these tests probe the rails), which would
# bury any real failure report.  Silence THAT logger, scoped and restored.
#
# Do NOT use `logging.disable()` here: it writes `logging.Logger.manager.disable`,
# a single process-wide global with no owner.  pytest imports every test module
# during collection, before the first test runs, so a module-scope call poisons
# assertLogs()/caplog for the entire session no matter the ordering — that was
# fullSuiteTestPollution (I-153714), 21 spurious failures across 7 unrelated files.
_QUIET_LOGGER = logging.getLogger("peppar_fix.do_freq_est")
_QUIET_SAVED_LEVEL = logging.NOTSET


def setUpModule():
    global _QUIET_SAVED_LEVEL
    _QUIET_SAVED_LEVEL = _QUIET_LOGGER.level
    _QUIET_LOGGER.setLevel(logging.CRITICAL)


def tearDownModule():
    _QUIET_LOGGER.setLevel(_QUIET_SAVED_LEVEL)

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

    def test_bare_topology_tightQ_does_not_rail(self):
        """The bare topology (no mosaic cascade) is well-damped: at normal
        cadence, tight measured-Q does not rail with or without receiver-clock
        lag — so delta's rail is NOT tight-Q underdamping.  (The rail IS
        reproduced once the mosaic clock filter is added — see
        TestMosaicCascadeRail.)  Canary: if the bare topology ever rails at
        normal cadence, revisit the finding."""
        for lag in (0.0, 10.0, 30.0):
            r = _run(duration_s=4000.0, coast_interval_s=1.0,
                     single_osc_obs_lag_s=lag, sigma_do_freq_ppb=8.4e-5)
            self.assertFalse(r.diverged(),
                             f"tight-Q railed at lag={lag} — revisit finding")


class TestMosaicCascadeRail(unittest.TestCase):
    """The mosaic clock filter (a 2nd filter in series with our servo) is the
    cascaded-loop dynamic that reproduces delta's rail / limit cycle.  A
    processing stall tips the marginal cascade; how far it goes (stable →
    limit cycle → rail) is set by the mosaic filter bandwidth — matching
    delta's abA1 (railed) vs knob1 (limit-cycled) captures.  Grounded in the
    captures where our_dt_rx_ns == mosaic_rxclkbias_ns."""

    _CASCADE = dict(single_oscillator=True, single_osc_mosaic_filter=True,
                    single_osc_obs_sigma_ns=0.13, actuator_gain_true=1.0,
                    sigma_do_phase_ns=0.0726, sigma_do_freq_ppb=0.01,
                    do_rwfm_ppb_per_sqrt_s=0.005, do_wfm_ppb=0.01,
                    do_wpm_ns=0.01, coast_interval_s=1.0)

    def _run(self, **ov):
        return ClosedLoopSim(preset("piface-current", duration_s=5000.0,
                                    **{**self._CASCADE, **ov})).run()

    def test_stall_alone_without_mosaic_is_stable(self):
        """The stall does NOT rail without the mosaic cascade — the sim
        observing the DO directly is well-damped.  The mosaic filter is the
        load-bearing single-oscillator dynamic."""
        r = ClosedLoopSim(preset(
            "piface-current", duration_s=5000.0,
            **{**self._CASCADE, "single_osc_mosaic_filter": False,
               "processing_stall": (2000.0, 3.0)})).run()
        self.assertFalse(r.diverged())

    def test_fast_mosaic_filter_stall_stable(self):
        """A fast mosaic filter (wide BW) keeps the cascade well-damped: the
        stall is absorbed."""
        r = self._run(mosaic_alpha=0.5, mosaic_beta=0.05,
                      processing_stall=(2000.0, 3.0))
        self.assertFalse(r.diverged())

    def test_slow_mosaic_filter_stall_rails(self):
        """A slow mosaic filter (narrow BW) makes the cascade marginal → the
        SAME stall tips it into the rail (delta's abA1)."""
        r = self._run(mosaic_alpha=0.03, mosaic_beta=0.003,
                      processing_stall=(2000.0, 3.0))
        self.assertTrue(r.diverged())


class TestActuatorWordLimit(unittest.TestCase):
    """actuator_max_ppb saturates the applied command (DAC word-limit) while
    the filter's B uses the unclamped command (windup) — so a cascade rail
    saturates near ±max (delta's +386 ppb) instead of overshooting to the
    DOFreqEst max_ppb."""

    def test_word_limit_bounds_applied_command(self):
        base = dict(single_oscillator=True, single_osc_mosaic_filter=True,
                    mosaic_alpha=0.03, mosaic_beta=0.003,
                    single_osc_obs_sigma_ns=0.13, sigma_do_freq_ppb=0.01,
                    sigma_do_phase_ns=0.0726, do_rwfm_ppb_per_sqrt_s=0.005,
                    do_wfm_ppb=0.01, do_wpm_ns=0.01, coast_interval_s=1.0,
                    processing_stall=(2000.0, 3.0))
        unclamped = ClosedLoopSim(preset("piface-current", duration_s=3500.0,
                                          **base)).run()
        clamped = ClosedLoopSim(preset("piface-current", duration_s=3500.0,
                                        actuator_max_ppb=386.0, **base)).run()
        # Both rail, but the clamp saturates the trajectory far tighter than
        # the unclamped overshoot.
        self.assertGreater(np.nanmax(np.abs(unclamped.phi_do_true_ns)),
                           10 * np.nanmax(np.abs(clamped.phi_do_true_ns)))


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

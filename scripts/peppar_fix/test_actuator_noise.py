"""charlieActuatorNoise (I-084500): the servo_sim actuator settling-noise term.

charlie's OTC hardware shows a short-τ TDEV BUMP under looser Q that peaks at
τ≈4 s and grows with Q — the signature of DAC-write/DPLL settling, a bounded
phase transient kicked by each adjustment and scaling with the adjustment
magnitude, NOT white per-epoch noise.  The sim's clean frequency actuator
misses it entirely; this term models it (a bounded AR(1) phase transient,
kick std = gain·|Δapplied|, correlation time tau_s) so the sim can predict
the loose-Q short-τ penalty / the knee.  Default off (gain=0) → byte-identical.

Precise magnitude/scaling calibration awaits charlie's 5-point TDEV-vs-Q curve;
these tests lock in the qualitative behaviour.
"""
import unittest

import numpy as np

from peppar_fix.servo_sim import preset, ClosedLoopSim

# stable-OCXO two-oscillator base (charlie's OTC class).
_BASE = dict(do_rwfm_ppb_per_sqrt_s=0.0018, do_wpm_ns=0.06, do_wfm_ppb=0.03,
             sigma_do_phase_ns=0.0564)


def _tdev(tau_s, **ov):
    r = ClosedLoopSim(preset("piface-current", duration_s=6000.0,
                             **{**_BASE, **ov})).run()
    _t, td = r.tdev(taus=[tau_s])
    return float(td[0])


class TestActuatorNoise(unittest.TestCase):

    def test_off_is_byte_identical(self):
        r0 = ClosedLoopSim(preset("piface-current", duration_s=1500.0,
                                  sigma_do_freq_ppb=0.01)).run()
        r1 = ClosedLoopSim(preset("piface-current", duration_s=1500.0,
                                  sigma_do_freq_ppb=0.01,
                                  actuator_noise_gain=0.0)).run()
        self.assertEqual(
            float(np.max(np.abs(r0.phi_do_true_ns - r1.phi_do_true_ns))), 0.0)

    def test_on_adds_short_tau_tdev(self):
        """With the term on, short-τ TDEV rises above the gain=0 baseline."""
        base = _tdev(4.0, sigma_do_freq_ppb=0.03, actuator_noise_gain=0.0)
        on = _tdev(4.0, sigma_do_freq_ppb=0.03, actuator_noise_gain=10.0,
                   actuator_noise_tau_s=4.0)
        self.assertGreater(on, base * 1.5)

    def test_bump_grows_with_adjustment_magnitude(self):
        """The bump scales with |Δapplied|: looser Q makes bigger per-epoch
        adjustments → a bigger short-τ bump than tight Q (same gain)."""
        loose = _tdev(4.0, sigma_do_freq_ppb=0.03, actuator_noise_gain=10.0,
                      actuator_noise_tau_s=4.0)
        tight = _tdev(4.0, sigma_do_freq_ppb=1.4e-4, actuator_noise_gain=10.0,
                      actuator_noise_tau_s=4.0)
        self.assertGreater(loose, tight)

    def test_bump_scales_with_gain(self):
        lo = _tdev(4.0, sigma_do_freq_ppb=0.03, actuator_noise_gain=5.0,
                   actuator_noise_tau_s=4.0)
        hi = _tdev(4.0, sigma_do_freq_ppb=0.03, actuator_noise_gain=20.0,
                   actuator_noise_tau_s=4.0)
        self.assertGreater(hi, lo)


if __name__ == "__main__":
    unittest.main()

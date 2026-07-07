"""DOFreqEst LQR phase-gain (loop-bandwidth) knob tests.

|L[2]| sets the phase-loop bandwidth: a smaller magnitude → a slower,
gentler loop (lower corner frequency) that leaves a low-noise DO to
free-run below the corner.  See docs/gnssdo-servo-loop-bandwidth.md.
"""
import os
import sys
import unittest

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix.do_freq_est import DOFreqEst


class LqrPhaseGainTest(unittest.TestCase):

    def test_default_gain(self):
        self.assertAlmostEqual(DOFreqEst().L[2], -0.05)

    def test_gain_is_settable(self):
        self.assertAlmostEqual(DOFreqEst(lqr_phase_gain=-0.001).L[2], -0.001)

    def test_lower_gain_is_a_gentler_loop(self):
        # Feed the same persistent DO-phase error to two servos differing
        # only in the phase gain; the lower-gain (lower-BW) one must command
        # a SMALLER frequency correction per epoch → it fights the DO less.
        def one_step_cmd(gain):
            s = DOFreqEst(lqr_phase_gain=gain)
            s.update(dt=1.0, do_phase_ns=100.0, do_phase_sigma_ns=0.1)  # seed x[2]
            return abs(s.update(dt=1.0, do_phase_ns=100.0, do_phase_sigma_ns=0.1))
        hi_bw = one_step_cmd(-0.05)
        lo_bw = one_step_cmd(-0.005)
        self.assertLess(lo_bw, hi_bw,
                        "a smaller |L[2]| must produce a gentler (smaller) "
                        "correction — a lower-bandwidth loop")


if __name__ == "__main__":
    unittest.main()

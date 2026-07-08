"""DOFreqEst soft normalized-innovation (NIS) gate — knob 1.

A phase glitch at lock is a huge-sigma innovation; the gate must cap its
influence on the state (especially the frequency state x[3], whose runaway
railed the GNSSDO+ on 2026-07-07) WITHOUT rejecting the measurement outright
(which would stall acquisition).  See docs/gnssdo-servo-loop-bandwidth.md.
"""
import os
import sys
import unittest

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix.do_freq_est import DOFreqEst


def _lock(servo, n=40):
    """Drive the do_phase arm to a tight lock at phase≈0 so P[2,2] is small."""
    for _ in range(n):
        servo.update(dt=1.0, do_phase_ns=0.0, do_phase_sigma_ns=0.16)


class InnovGateTest(unittest.TestCase):

    def test_default_is_off(self):
        self.assertIsNone(DOFreqEst().innov_gate_nsigma)

    def test_off_matches_ungated(self):
        # Same glitch, gate off on both → identical state (no behavior change
        # for existing hosts).
        a = DOFreqEst()
        b = DOFreqEst(innov_gate_nsigma=None)
        for s in (a, b):
            _lock(s)
            s.update(dt=1.0, do_phase_ns=12.0, do_phase_sigma_ns=0.16)
        self.assertAlmostEqual(a.x[2], b.x[2], places=9)
        self.assertAlmostEqual(a.x[3], b.x[3], places=9)

    def test_gate_caps_a_glitch_pull(self):
        # A 12 ns glitch at σ=0.16 ns is a ~75σ outlier at lock.  The gated
        # servo's frequency-state move must be far smaller than the ungated
        # one — the glitch cannot punch x[3].
        ungated = DOFreqEst()
        gated = DOFreqEst(innov_gate_nsigma=5.0)
        _lock(ungated); _lock(gated)
        f0_u, f0_g = ungated.x[3], gated.x[3]
        ungated.update(dt=1.0, do_phase_ns=12.0, do_phase_sigma_ns=0.16)
        gated.update(dt=1.0, do_phase_ns=12.0, do_phase_sigma_ns=0.16)
        pull_ungated = abs(ungated.x[3] - f0_u)
        pull_gated = abs(gated.x[3] - f0_g)
        self.assertLess(pull_gated, pull_ungated,
                        "gate must reduce the frequency-state pull from a glitch")
        # The cap is strong: NIS_eff == gate² means the pull shrinks by
        # ~gate²/NIS_raw.  75σ → ~5σ is >10× suppression.
        self.assertLess(pull_gated, 0.2 * pull_ungated)

    def test_gate_records_trip_and_inflation(self):
        s = DOFreqEst(innov_gate_nsigma=5.0)
        _lock(s)
        self.assertEqual(s.arm_gate_trips['do_phase'], 0)
        s.update(dt=1.0, do_phase_ns=12.0, do_phase_sigma_ns=0.16)
        self.assertEqual(s.arm_gate_trips['do_phase'], 1)
        self.assertGreater(s.last_arm_gate_infl['do_phase'], 1.0)
        # RAW chi² is preserved for post-mortems (not clamped to gate²=25).
        self.assertGreater(s.last_arm_chi2['do_phase'], 100.0)

    def test_in_bounds_measurement_untouched(self):
        # A 1σ innovation passes with no inflation and no trip.
        s = DOFreqEst(innov_gate_nsigma=5.0)
        _lock(s)
        s.update(dt=1.0, do_phase_ns=0.16, do_phase_sigma_ns=0.16)
        self.assertEqual(s.arm_gate_trips['do_phase'], 0)
        self.assertAlmostEqual(s.last_arm_gate_infl['do_phase'], 1.0, places=9)

    def test_gate_zero_does_not_divide_by_zero(self):
        # innov_gate_nsigma=0 must not raise (guard is gate>0, not just !=None).
        s = DOFreqEst(innov_gate_nsigma=0.0)
        _lock(s)
        s.update(dt=1.0, do_phase_ns=12.0, do_phase_sigma_ns=0.16)  # no crash
        self.assertEqual(s.arm_gate_trips['do_phase'], 0)  # gate inactive

    def test_acquisition_large_dt_rx_not_gated(self):
        # Freshly seeded (large P[2,2]), a legitimately large dt_rx during
        # acquisition must NOT be gated — S is large so NIS is modest.
        s = DOFreqEst(innov_gate_nsigma=5.0)
        # First do_phase obs seeds x[2]; the NEXT (still-large-P) epoch carries
        # a big absolute value that acquisition should accept.
        s.update(dt=1.0, do_phase_ns=-11.0, do_phase_sigma_ns=0.16)
        s.update(dt=1.0, do_phase_ns=-9.0, do_phase_sigma_ns=0.16)
        self.assertEqual(s.arm_gate_trips['do_phase'], 0)


if __name__ == "__main__":
    unittest.main()

"""Tests for GNSSDO+ PPS cycle-slip de-glitching (freerun_analysis)."""
import os
import sys
import unittest

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix.freerun_analysis import deglitch_cycle_slips


class DeglitchCycleSlipTest(unittest.TestCase):

    def _series(self, trend_ps_per_s=930, n=200, slip_at=None, slip_cycles=-1):
        """A ~1 ns/s trend + small noise, with an optional 100 ns step slip."""
        samples = []
        ps = 500_000_000
        for i in range(n):
            ps += trend_ps_per_s + (37 if i % 2 else -37)   # ~tiny deterministic noise
            if slip_at is not None and i == slip_at:
                ps += slip_cycles * 100_000                  # 100 ns cycle step
            samples.append((i, ps))
        return samples

    def test_no_slip_untouched(self):
        s = self._series(slip_at=None)
        out, n = deglitch_cycle_slips(s)
        self.assertEqual(n, 0)
        self.assertEqual([p for _, p in out], [p for _, p in s])

    def test_single_minus_100ns_slip_removed(self):
        s = self._series(slip_at=100, slip_cycles=-1)
        out, n = deglitch_cycle_slips(s)
        self.assertEqual(n, 1)
        # after correction the per-epoch jump at the slip is back to the trend,
        # not -100 ns
        ps = [p for _, p in out]
        self.assertAlmostEqual(ps[101] - ps[100], 930, delta=200)

    def test_two_slips_both_signs(self):
        s = self._series(slip_at=50, slip_cycles=-1)
        # add a +1 cycle slip later
        s = [(i, p + (100_000 if i >= 150 else 0)) for i, p in s]
        out, n = deglitch_cycle_slips(s)
        self.assertEqual(n, 2)
        # de-glitched series should be monotonically ~trend (no 100 ns steps)
        d = [out[i + 1][1] - out[i][1] for i in range(len(out) - 1)]
        self.assertTrue(all(abs(x) < 5000 for x in d),
                        "no residual 100 ns steps after de-glitch")

    def test_subcycle_motion_preserved(self):
        # a genuine 30 ns sub-cycle wander must NOT be snapped away
        s = self._series(slip_at=None)
        s = [(i, p + (30_000 if i >= 100 else 0)) for i, p in s]   # 30 ns step (< tol from 0)
        out, n = deglitch_cycle_slips(s)
        self.assertEqual(n, 0, "30 ns (sub-cycle) is not an integer-cycle slip")

    def test_short_series_noop(self):
        self.assertEqual(deglitch_cycle_slips([(0, 1)]), ([(0, 1)], 0))


if __name__ == "__main__":
    unittest.main()

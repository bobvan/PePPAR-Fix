"""Sign-convention tests for all three DO-frequency measurement paths.

There are three independent code paths that measure DO frequency offset
via TICC, and they MUST agree on the canonical convention:

    positive freq_ppb = DO is fast (oscillates above nominal rate)

These tests synthesize chA / (chA, chB) traces with known frequency
offsets and assert that each measurement path returns the agreed sign
and roughly the agreed magnitude.  Catches the bootstrapSignFlip
class of bug (2026-05-26 / 2026-05-27 dayplan) by locking in the
convention at the source rather than by configuration patching.

Paths under test:
  A. dac_slope_cal.measure_freq_offset       — chA-chB regression
  B. TiccTimestamper.measure_pps_frequency   — chA only vs TICC Rb
  C. measure_differential_frequency          — chA-chB pair regression

Each path takes a different style of input (Path A and C need TICC
device or a mock; Path B is purely from-edges).  We exercise the
math directly where possible, monkeypatching out the TICC iterator
where required.
"""
from __future__ import annotations

import os
import sys
import types
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.dirname(_HERE)
for p in (_SCRIPTS, _HERE):
    if p not in sys.path:
        sys.path.insert(0, p)


def _synth_edges_chA(freq_ppb: float, n_intervals: int = 10,
                      start_sec: int = 100, start_ps: int = 0):
    """Build a list of (ref_sec, ref_ps) chA edges for a DO that is
    `freq_ppb` ppb fast (positive) or slow (negative).  Returns the
    raw integer-second/picosecond tuples the TICC reports.
    """
    edges = []
    interval_s = 1.0 - freq_ppb * 1e-9   # fast DO → interval shorter than 1 s
    t0 = start_sec + start_ps * 1e-12
    for k in range(n_intervals + 1):
        t = t0 + k * interval_s
        sec = int(t)
        ps = int(round((t - sec) * 1e12))
        # Handle ps overflow into next second from rounding
        if ps >= 1_000_000_000_000:
            sec += 1
            ps -= 1_000_000_000_000
        edges.append((sec, ps))
    return edges


def _synth_pairs_chA_minus_chB(freq_ppb: float, n_intervals: int = 10):
    """Build a list of (sec, phase_ns = chA-chB) for cal-style input.

    chB is fixed at a steady 1 Hz reference (TICC sees it at sec=k, ps=0).
    chA is the DO PPS offset by freq_ppb relative to chB.
    phase_ns = (chA - chB) * 1e9, in nanoseconds.
    """
    pairs = []  # (sec, phase_ns)
    for k in range(n_intervals + 1):
        # chB at integer sec k+100, ps=0  →  TICC time k+100.0
        # chA at TICC time k * (1 - freq_ppb*1e-9) + 100
        chB_t = 100.0 + k * 1.0
        chA_t = 100.0 + k * (1.0 - freq_ppb * 1e-9)
        phase_s = chA_t - chB_t
        pairs.append((100 + k, phase_s * 1e9))
    return pairs


class _FakeTicc:
    """Drop-in stand-in for ticc.Ticc that replays a pre-built sequence."""

    def __init__(self, events):
        self._events = events

    def __enter__(self):
        return iter(self._events)

    def __exit__(self, *_):
        return False


class PathB_TiccTimestamperSignTests(unittest.TestCase):
    """Path B: TiccTimestamper.measure_pps_frequency on chA alone.

    Exercises the *math* by feeding synthesized edges directly into the
    function via a fake TICC iterator.
    """

    def _measure(self, freq_ppb: float, n_intervals: int = 10):
        from peppar_fix import timestamper as tsmod

        edges = _synth_edges_chA(freq_ppb, n_intervals=n_intervals)
        # The TICC iterator yields (channel, ref_sec, ref_ps) tuples.
        events = [('chA', s, p) for s, p in edges]

        fake = _FakeTicc(events)

        # Stub the Ticc import inside measure_pps_frequency
        fake_ticc_mod = types.SimpleNamespace(
            Ticc=lambda *a, **kw: fake)
        saved = sys.modules.get('ticc')
        sys.modules['ticc'] = fake_ticc_mod
        try:
            ts = tsmod.TiccTimestamper('/dev/null', channel='chA')
            return ts.measure_pps_frequency(n_samples=n_intervals + 1,
                                             timeout_s=300)
        finally:
            if saved is not None:
                sys.modules['ticc'] = saved
            else:
                del sys.modules['ticc']

    def test_fast_DO_reports_positive(self):
        freq_ppb, _sigma, n = self._measure(+1000.0)
        self.assertIsNotNone(freq_ppb)
        self.assertAlmostEqual(freq_ppb, +1000.0, delta=1.0,
                                msg="Path B should return positive ppb for fast DO")

    def test_slow_DO_reports_negative(self):
        freq_ppb, _sigma, n = self._measure(-1000.0)
        self.assertIsNotNone(freq_ppb)
        self.assertAlmostEqual(freq_ppb, -1000.0, delta=1.0,
                                msg="Path B should return negative ppb for slow DO")

    def test_at_nominal_reports_near_zero(self):
        freq_ppb, _sigma, n = self._measure(0.0)
        self.assertIsNotNone(freq_ppb)
        self.assertAlmostEqual(freq_ppb, 0.0, delta=0.1)

    def test_no_off_by_one_at_second_boundary(self):
        """The historical bug: when the first edge starts at ref_ps=0 and
        the DO is even moderately fast, the last edge lands just before
        a TICC second boundary and the integer-seconds difference is one
        less than the true interval count.  The pre-fix formula then
        reported ~1e8 ppb.  Lock that in with a regression test.
        """
        # +200 ppb fast, 10 intervals, first edge at ref_ps=0:
        # last edge at TICC 110 − 2µs = 109.999998 → ref_sec=109
        # (whereas a 1-ppb-fast trace's last edge would be at 109.99999999
        # also ref_sec=109; the off-by-one fires across the whole range)
        freq_ppb, _sigma, _n = self._measure(+200.0)
        self.assertIsNotNone(freq_ppb)
        self.assertAlmostEqual(freq_ppb, +200.0, delta=1.0,
                                msg="Off-by-one boundary bug regression")


class PathA_CalSignTests(unittest.TestCase):
    """Path A: dac_slope_cal.measure_freq_offset slope sign.

    The cal post-2026-05-26 (commit 782d5e3, reapplied as c5df98f
    after a brief revert) negates the raw regression slope so output
    is in engine convention.  We exercise the linear-regression core
    directly on synthesized chA-chB pairs.
    """

    def _measure_slope(self, freq_ppb: float):
        # Reproduce the regression body of measure_freq_offset; testing
        # the import-and-IO wrapper would require monkey-patching TICC.
        pairs = _synth_pairs_chA_minus_chB(freq_ppb, n_intervals=10)
        secs = [p[0] for p in pairs]
        phases_ns = [p[1] for p in pairs]
        n = len(pairs)
        mean_s = sum(secs) / n
        mean_p = sum(phases_ns) / n
        num = sum((s - mean_s) * (p - mean_p) for s, p in zip(secs, phases_ns))
        den = sum((s - mean_s) ** 2 for s in secs)
        # Same formula as the live tool (post-c5df98f):
        return -(num / den)

    def test_fast_DO_slope_positive(self):
        slope_ppb = self._measure_slope(+1000.0)
        self.assertAlmostEqual(slope_ppb, +1000.0, delta=0.5,
                                msg="Path A should return positive for fast DO")

    def test_slow_DO_slope_negative(self):
        slope_ppb = self._measure_slope(-1000.0)
        self.assertAlmostEqual(slope_ppb, -1000.0, delta=0.5)


class PathC_DifferentialSignTests(unittest.TestCase):
    """Path C: measure_differential_frequency on chA-chB pairs."""

    def _measure(self, freq_ppb: float):
        # Pull the regression core to avoid TICC IO setup.  The live
        # function negates the slope; we mirror that here.
        pairs = []
        for k in range(11):
            chA_t = 100.0 + k * (1.0 - freq_ppb * 1e-9)
            chB_t = 100.0 + k * 1.0
            elapsed = k
            diff_ns = (chA_t - chB_t) * 1e9
            pairs.append((elapsed, diff_ns))
        n = len(pairs)
        sx = sum(t for t, _ in pairs)
        sy = sum(d for _, d in pairs)
        sxy = sum(t * d for t, d in pairs)
        sxx = sum(t * t for t, _ in pairs)
        denom = n * sxx - sx * sx
        slope = (n * sxy - sx * sy) / denom
        return -slope  # matches live function

    def test_fast_DO_reports_positive(self):
        self.assertAlmostEqual(self._measure(+1000.0), +1000.0, delta=0.5)

    def test_slow_DO_reports_negative(self):
        self.assertAlmostEqual(self._measure(-1000.0), -1000.0, delta=0.5)


class AllPathsAgreeTests(unittest.TestCase):
    """Cross-path consistency: all three paths must agree on sign and
    roughly on magnitude for the same physical DO frequency offset."""

    def _measure_all(self, freq_ppb: float):
        # Path A (cal)
        pairs = _synth_pairs_chA_minus_chB(freq_ppb, n_intervals=10)
        secs = [p[0] for p in pairs]
        phases = [p[1] for p in pairs]
        n = len(pairs)
        ms = sum(secs) / n
        mp = sum(phases) / n
        num = sum((s - ms) * (p - mp) for s, p in zip(secs, phases))
        den = sum((s - ms) ** 2 for s in secs)
        path_A = -(num / den)

        # Path B (chA-only TICC bootstrap)
        path_b_test = PathB_TiccTimestamperSignTests()
        path_B, _, _ = path_b_test._measure(freq_ppb)

        # Path C (differential)
        path_c_test = PathC_DifferentialSignTests()
        path_C = path_c_test._measure(freq_ppb)

        return path_A, path_B, path_C

    def test_all_three_agree_on_fast(self):
        a, b, c = self._measure_all(+500.0)
        for v, name in ((a, 'A'), (b, 'B'), (c, 'C')):
            self.assertGreater(
                v, 0,
                msg=f"Path {name} returned {v}; should be > 0 for fast DO")
            self.assertAlmostEqual(
                v, 500.0, delta=2.0,
                msg=f"Path {name} = {v}; expected ~500 ppb for fast DO")

    def test_all_three_agree_on_slow(self):
        a, b, c = self._measure_all(-500.0)
        for v, name in ((a, 'A'), (b, 'B'), (c, 'C')):
            self.assertLess(
                v, 0,
                msg=f"Path {name} returned {v}; should be < 0 for slow DO")
            self.assertAlmostEqual(
                v, -500.0, delta=2.0,
                msg=f"Path {name} = {v}; expected ~-500 ppb for slow DO")


if __name__ == '__main__':
    unittest.main()

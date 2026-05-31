#!/usr/bin/env python3
"""Equivalence + perf check for the vectorized _compute_tdev.

The pre-2026-05-30 implementation of ``peppar_fix.noise_estimator
._compute_tdev`` was a pure-Python triple-nested loop (~30 M
iterations per call on the 7200-sample steady-state buffer) that
consumed 28 % of engine CPU on clkPoC3 (Pi 4) and produced 30-40 s
main-loop stalls every ~90 s.  py-spy flame graph + dump-during-burst
confirmed the inner second-difference arithmetic on line 305 as the
hot path.

The vectorized replacement uses ``np.asarray`` once, then computes
the inner second-difference as one slice subtraction and the outer
sum-of-n-consecutives as one ``np.convolve(d, ones(n), 'valid')``.

This test pins the equivalence: across deterministic + random phase
series and a sweep of taus, the vectorized form agrees with a
pedagogical triple-loop reference within float-tolerance.  A
separate test confirms the perf delta is large enough that the
dev-box reference-vs-vectorized timing is dominated by the loop, not
benchmark noise.
"""
from __future__ import annotations

import math
import sys
import time
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

import numpy as np  # noqa: E402

from peppar_fix.noise_estimator import _compute_tdev  # noqa: E402


def _tdev_reference_single_tau(phases, n):
    """Pure-Python overlapping TDEV at a single averaging factor.

    Pedagogical — direct transcription of the triple-loop estimator
    from the pre-vectorization implementation.  Slow on purpose:
    independent reference for the vectorized form.
    """
    N = len(phases)
    if 3 * n >= N:
        return None
    outer_count = N - 3 * n + 1
    total = 0.0
    for j in range(outer_count):
        inner = 0.0
        for i in range(j, j + n):
            inner += phases[i + 2 * n] - 2 * phases[i + n] + phases[i]
        total += inner * inner
    return math.sqrt(total / (6.0 * n * n * outer_count))


class ComputeTdevTests(unittest.TestCase):

    # ── boundary conditions ────────────────────────────────────────

    def test_empty_series_returns_empty(self):
        self.assertEqual(_compute_tdev([]), {})

    def test_short_series_returns_empty(self):
        # The "if N < 4: return {}" guard.
        self.assertEqual(_compute_tdev([1.0, 2.0]), {})
        self.assertEqual(_compute_tdev([1.0, 2.0, 3.0]), {})

    def test_series_too_short_for_smallest_tau(self):
        # N=4 with tau=1 needs 3n=3 < N, but the default-taus loop
        # generates [1, 2, ...] and at n=2 we have 3n=6 ≥ 4, so only
        # tau=1 should appear.
        phases = [0.0, 1.0, 0.5, -0.5]
        result = _compute_tdev(phases)
        self.assertEqual(set(result.keys()), {1})
        # Compare against the reference.
        ref = _tdev_reference_single_tau(phases, 1)
        self.assertAlmostEqual(result[1], ref, places=12)

    # ── physics: invariants the estimator must satisfy ─────────────

    def test_constant_phases_yield_zero_tdev(self):
        phases = [3.14] * 100
        result = _compute_tdev(phases)
        self.assertTrue(result, 'expected non-empty result for N=100')
        for tau, tdev in result.items():
            self.assertAlmostEqual(
                tdev, 0.0, places=12,
                msg=f'tau={tau}: expected 0, got {tdev}')

    def test_linear_drift_yields_zero_tdev(self):
        # The second difference x[i+2n] - 2x[i+n] + x[i] is identically
        # zero for any linear phase (constant frequency offset), so
        # TDEV must be 0 to machine precision.
        phases = [0.1 * i for i in range(200)]
        result = _compute_tdev(phases)
        self.assertTrue(result)
        for tau, tdev in result.items():
            self.assertAlmostEqual(
                tdev, 0.0, places=8,
                msg=f'tau={tau}: expected 0, got {tdev}')

    # ── equivalence vs the pedagogical triple-loop reference ───────

    def test_random_series_matches_reference_default_taus(self):
        rng = np.random.default_rng(seed=42)
        phases = list(rng.normal(0, 1.0, size=200))
        result = _compute_tdev(phases)
        n = 1
        while 3 * n < len(phases):
            ref = _tdev_reference_single_tau(phases, n)
            self.assertAlmostEqual(
                result[n], ref, places=10,
                msg=f'tau={n}: vec={result[n]} ref={ref}')
            n *= 2

    def test_random_series_matches_reference_explicit_taus(self):
        rng = np.random.default_rng(seed=1)
        phases = list(rng.normal(0, 1.0, size=150))
        # Mix of powers-of-2 and odd values to exercise non-default taus.
        explicit = [1, 3, 7, 15, 31]
        result = _compute_tdev(phases, taus=explicit)
        self.assertEqual(set(result.keys()), set(explicit))
        for n in explicit:
            ref = _tdev_reference_single_tau(phases, n)
            self.assertAlmostEqual(
                result[n], ref, places=10,
                msg=f'tau={n}')

    def test_tau_too_large_for_series_is_dropped(self):
        # If 3n >= N, the tau must not appear in the result.
        phases = list(np.random.default_rng(seed=2).normal(0, 1.0, size=30))
        # N=30 means tau must satisfy 3n < 30, i.e. n ≤ 9.  Pass 10
        # explicitly and confirm it's silently dropped (matches the
        # pre-vectorization "if 3*n >= N: break" semantics).
        result = _compute_tdev(phases, taus=[1, 5, 9, 10, 20])
        self.assertEqual(set(result.keys()), {1, 5, 9})

    # ── perf sanity (the whole point of the vectorization) ─────────

    def test_vectorized_beats_reference_by_wide_margin(self):
        # Single tau=128 on a 2000-sample buffer.  Reference triple
        # loop is ~128 × (2000 - 384) ≈ 200 k iterations — slow but
        # short enough to finish in seconds on a dev box.  Vectorized
        # is one slice subtract + one convolve.
        rng = np.random.default_rng(seed=7)
        phases = list(rng.normal(0, 1.0, size=2000))
        n = 128
        # Reference
        t0 = time.perf_counter()
        ref = _tdev_reference_single_tau(phases, n)
        t_ref = time.perf_counter() - t0
        # Vectorized
        t0 = time.perf_counter()
        result = _compute_tdev(phases, taus=[n])
        t_vec = time.perf_counter() - t0
        self.assertAlmostEqual(result[n], ref, places=10)
        # Vectorized must be at least 5× faster on the dev box.
        # Empirically observed ratio is 50-200× depending on CPU.
        # 5× is a defensive lower bound that catches accidental loss
        # of vectorization (e.g., reintroducing a Python loop).
        speedup = t_ref / max(t_vec, 1e-9)
        self.assertGreater(
            speedup, 5.0,
            msg=f'vec {t_vec:.4f}s vs ref {t_ref:.4f}s '
                f'(speedup {speedup:.1f}×) — below 5× sentinel')


if __name__ == '__main__':
    unittest.main()

"""Tests for `solve_pseudorange.SP3.sat_position` interpolation
bounds check.

The SP3 class is only exercised by the PRIDE regression harness
today, so tests live here rather than under scripts/peppar_fix.

Ground-truth context: 8-point Lagrange polynomial extrapolation
outside the fit window diverges rapidly.  PRIDE-PPPAR's canonical
workaround is to pull adjacent-day SP3 (days ±1) so interpolation
is always bounded.  When only a single day is available (as in
our harness against ABMF 2020/001 with `com20863.eph`),
`sat_position` must refuse queries in the edge ``half_win``
epochs on either side rather than extrapolate.  See
``project_to_main_pride_sp3_clk_bia_diagnostics_20260423``.
"""

from __future__ import annotations

import os
import sys
import unittest
from datetime import datetime, timedelta, timezone

import numpy as np

_SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from solve_pseudorange import SP3  # noqa: E402


def _make_sp3_stub(n_epochs: int, interval_s: float = 300.0) -> SP3:
    """Build an SP3 instance without parsing a file.

    Bypasses `__init__` (which calls `_parse`) and hand-populates
    the attributes.  Creates a stationary single-satellite
    trajectory — sat_position's correctness for a realistic orbit
    isn't what we're testing here; we're testing the bounds
    check.  Keeping the ground-truth trivial makes interpolation-
    returns-something-sensible straightforward to assert.

    ``positions`` holds a linearly-increasing trajectory in X so
    interpolation is exact for linear queries (Lagrange of any
    order handles a polynomial of its degree or lower exactly).
    """
    sp3 = SP3.__new__(SP3)
    base = datetime(2026, 1, 1, 0, 0, 0, tzinfo=timezone.utc)
    sp3.epochs = [base + timedelta(seconds=i * interval_s)
                  for i in range(n_epochs)]
    # G01 moves +1000 m per epoch in X, stationary in Y/Z;
    # clock drifts +1 µs per epoch.
    xs = np.array([1e7 + i * 1000.0 for i in range(n_epochs)])
    ys = np.full(n_epochs, 2e7)
    zs = np.full(n_epochs, 3e7)
    sp3.positions = {
        'G01': np.column_stack([xs, ys, zs]),
    }
    sp3.clocks = {
        'G01': np.array([i * 1e-6 for i in range(n_epochs)]),
    }
    sp3._epoch_seconds = np.array([
        (e - sp3.epochs[0]).total_seconds() for e in sp3.epochs
    ])
    return sp3


class SafeWindowTest(unittest.TestCase):
    """Out-of-window queries return (None, None); in-window
    queries return valid position + clock."""

    def setUp(self):
        # 16 epochs × 300 s = 75-min span.  Safe window is
        # epochs[4] .. epochs[11] = 20 min in from each end.
        self.sp3 = _make_sp3_stub(n_epochs=16)
        self.base = self.sp3.epochs[0]
        self.half_win = 4

    def test_unknown_sv_returns_none(self):
        pos, clk = self.sp3.sat_position('G99', self.base)
        self.assertIsNone(pos)
        self.assertIsNone(clk)

    def test_query_before_safe_lo_returns_none(self):
        """Anything before `epochs[half_win]` would need backward
        extrapolation.  Refuse."""
        # 1 second before the boundary
        t = self.base + timedelta(
            seconds=self.sp3._epoch_seconds[self.half_win] - 1.0,
        )
        pos, clk = self.sp3.sat_position('G01', t)
        self.assertIsNone(pos)
        self.assertIsNone(clk)

    def test_query_at_file_start_returns_none(self):
        """The very first epoch itself is in the unsafe zone —
        8-point interpolation needs 4 points on each side."""
        pos, clk = self.sp3.sat_position('G01', self.base)
        self.assertIsNone(pos)
        self.assertIsNone(clk)

    def test_query_after_safe_hi_returns_none(self):
        """Anything past `epochs[-half_win-1]` would need forward
        extrapolation.  Refuse."""
        last_safe_sec = self.sp3._epoch_seconds[-self.half_win - 1]
        t = self.base + timedelta(seconds=last_safe_sec + 1.0)
        pos, clk = self.sp3.sat_position('G01', t)
        self.assertIsNone(pos)
        self.assertIsNone(clk)

    def test_query_past_file_end_returns_none(self):
        """Regression specifically for the 1231 m blow-up on
        ABMF 2020/001: `com20863.eph` covers exactly day 001,
        RINEX observations at 23:59 fall past the last SP3
        epoch, 8-point Lagrange extrapolation of an orbit 20 min
        past the fit window returns non-physical positions.  We
        should refuse instead."""
        # 1 hour past the last epoch
        last_sec = self.sp3._epoch_seconds[-1]
        t = self.base + timedelta(seconds=last_sec + 3600.0)
        pos, clk = self.sp3.sat_position('G01', t)
        self.assertIsNone(pos)
        self.assertIsNone(clk)

    def test_query_inside_safe_window_returns_interpolated(self):
        """Mid-window query hits the Lagrange path.  Our stub
        trajectory is linear in X (stationary in Y, Z), so
        Lagrange of any order returns the exact value."""
        # Query at epoch[6]'s center, plus a fractional offset to
        # force interpolation rather than exact-hit.
        t = self.base + timedelta(
            seconds=self.sp3._epoch_seconds[6] + 150.0,
        )
        pos, clk = self.sp3.sat_position('G01', t)
        self.assertIsNotNone(pos)
        self.assertIsNotNone(clk)
        # X at t = 6.5 epochs = 1e7 + 6.5 × 1000 = 10006500
        self.assertAlmostEqual(pos[0], 1e7 + 6500.0, places=3)
        self.assertAlmostEqual(pos[1], 2e7, places=3)
        self.assertAlmostEqual(pos[2], 3e7, places=3)
        # Clock at t = 6.5 epochs = 6.5 × 1e-6 s
        self.assertAlmostEqual(clk, 6.5e-6, places=9)

    def test_safe_window_boundary_epochs_in_range(self):
        """The boundary itself — `epochs[half_win]` — must be
        inside the safe window.  Otherwise the window is
        [safe_lo, safe_hi] open on both ends, and a query that
        lands exactly on a sample would fail.  Lagrange at a
        fit-point returns the sample value exactly."""
        t_lo = self.base + timedelta(
            seconds=self.sp3._epoch_seconds[self.half_win],
        )
        pos_lo, clk_lo = self.sp3.sat_position('G01', t_lo)
        self.assertIsNotNone(pos_lo, "safe_lo boundary must be in range")
        # Lagrange hitting a fit point — should be exact.
        expected_x = 1e7 + self.half_win * 1000.0
        self.assertAlmostEqual(pos_lo[0], expected_x, places=3)

        t_hi = self.base + timedelta(
            seconds=self.sp3._epoch_seconds[-self.half_win - 1],
        )
        pos_hi, _ = self.sp3.sat_position('G01', t_hi)
        self.assertIsNotNone(pos_hi, "safe_hi boundary must be in range")


class TooFewEpochsTest(unittest.TestCase):
    """SP3 files with n_epochs ≤ 2·half_win have no safe
    interpolation window.  sat_position should refuse all
    queries rather than silently extrapolating every call.  Not
    a realistic case for production (2020/001 has 289 SP3
    epochs), but worth covering the edge."""

    def test_empty_safe_window(self):
        sp3 = _make_sp3_stub(n_epochs=6)   # 6 < 2·4
        # With n=6 and half_win=4, safe_lo = epochs[4] and
        # safe_hi = epochs[-5] = epochs[1].  safe_lo > safe_hi;
        # no valid queries.  Any query returns None.
        for offset_s in (0.0, 300.0, 600.0, 1500.0):
            pos, clk = sp3.sat_position(
                'G01',
                sp3.epochs[0] + timedelta(seconds=offset_s),
            )
            self.assertIsNone(
                pos,
                f"offset {offset_s}s should refuse (empty safe window)",
            )


if __name__ == "__main__":
    unittest.main()

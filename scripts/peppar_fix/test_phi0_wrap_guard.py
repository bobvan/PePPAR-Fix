#!/usr/bin/env python3
"""The post-step phase residual must not be trusted near the wrap boundary.

``phi_0`` in ``phc_bootstrap`` is phase modulo one second, signed to ±500 ms,
so **+499.9 ms and −499.9 ms are adjacent** and a residual parked there has an
ambiguous sign.  The glide slope that consumes it would steer the wrong way at
full control authority.

This is not hypothetical.  The measured consequence of stepping the PHC with
PEROUT armed is a **500 ms PEROUT offset** — 9 of 10 TimeHat runs, see
``docs/i226-perout-500ms-bug.md``.  The failure mode and the wrap ambiguity are
the same number *by construction*, not by coincidence: a half-second shift on a
1 Hz PPS puts the rising edge exactly where the falling edge was.

Raised by BBGresearch 2026-09-19, who hit the identical arithmetic on an
independent i210 bootstrap and added the same guard.

Note this is the FAR end of the wrap.  The NEAR end — phase ≈ 0 with nsec
oscillating between 999999997 and 0 — is the separate ``rounded_sec``
zero-crossing bug.
"""
from __future__ import annotations

import unittest


class TestPhi0WrapGuard(unittest.TestCase):

    @property
    def guard_ns(self):
        import phc_bootstrap
        return phc_bootstrap.PHI0_WRAP_GUARD_NS

    def test_guard_trips_before_the_boundary_with_real_margin(self):
        self.assertLess(self.guard_ns, 500_000_000,
                        "guard must trip BEFORE the ±500 ms wrap boundary")
        margin_ns = 500_000_000 - self.guard_ns
        self.assertGreaterEqual(
            margin_ns, 50_000_000,
            "need real margin below the boundary, not a hair's breadth")

    def test_the_measured_500ms_perout_bug_is_caught(self):
        """The documented failure must trip the guard, at either sign."""
        for phi_0 in (500_000_000, -500_000_000,
                      499_000_000, -499_000_000,
                      450_000_000, -450_000_000):
            with self.subTest(phi_0=phi_0):
                self.assertGreater(abs(phi_0), self.guard_ns)

    def test_ordinary_step_residuals_are_not_blocked(self):
        """A normal bootstrap must pass — the guard is for the pathology."""
        # Observed residuals: ADJ_SETOFFSET lands ~ns, clock_settime ~ms.
        for phi_0 in (0, 363_000, -2_000, 20_000_000, -20_000_000,
                      116_676_000, -116_676_000):
            with self.subTest(phi_0=phi_0):
                self.assertLessEqual(abs(phi_0), self.guard_ns)


if __name__ == "__main__":
    unittest.main()

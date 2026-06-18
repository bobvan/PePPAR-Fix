"""Tests for the servoOutlierDoomLoop fix in _servo_outlier_decision.

Background (diagnosed 2026-06-08 from the ticcVsExtint overnight): when
the DO error exceeds track_outlier_ns while settled, the servo SKIPPED the
correction.  For a *real* (sustained) error that removes the only restoring
force, so the DO coasts at its free-running frequency offset and the error
ramps unbounded → 30 consecutive skips → exit-5 cascade.  The intended
re-step recovery (track_restep_ns=100µs) was unreachable before exit-5, and
on DAC-OCXO/TICC-only hosts it reads pps_err_extts_ns which is synthesized
as 0 there — doubly dead.

The fix: after `ramp_accept_n` consecutive SAME-SIGN outliers (a real ramp,
not a spike), return "ramp_accept" so the caller lets the EKF absorb the
error and arrest the ramp instead of skipping toward exit-5.
"""
from __future__ import annotations

import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix_engine import _servo_outlier_decision  # noqa: E402


def _decide(ctx, obs, n=5, thresh=500.0):
    return _servo_outlier_decision(
        ctx, outlier_observable_ns=obs, track_outlier_ns=thresh,
        converging=False, gap_recovery=False, ramp_accept_n=n)


class ServoOutlierDoomLoopTest(unittest.TestCase):
    def setUp(self):
        self.ctx = {'consecutive_outliers': 0}

    # --- the core fix --------------------------------------------------
    def test_sustained_same_sign_ramp_triggers_ramp_accept(self):
        """5 consecutive same-sign outliers → ramp_accept (not skip)."""
        decisions = []
        for k in range(5):
            obs = 600.0 + 14.0 * k            # ramping positive, > 500ns
            decisions.append(_decide(self.ctx, obs)[0])
        # first 4 are skipped spikes; the 5th recognises the ramp
        self.assertEqual(decisions[:4], ["outlier"] * 4)
        self.assertEqual(decisions[4], "ramp_accept")

    def test_ramp_accept_resets_counters(self):
        """After ramp_accept the cascade counter + ramp run are cleared."""
        for k in range(5):
            _decide(self.ctx, 600.0 + 14.0 * k)
        self.assertEqual(self.ctx['consecutive_outliers'], 0)
        self.assertEqual(self.ctx.get('_outlier_ramp', 0), 0)

    def test_negative_ramp_also_accepts(self):
        d = [_decide(self.ctx, -(600.0 + 14.0 * k))[0] for k in range(5)]
        self.assertEqual(d[4], "ramp_accept")

    # --- rampReacquireDriftAware (DRAFT): drift-rate estimate ----------
    def test_ramp_accept_estimates_drift_rate(self):
        """With mono_time, ramp_accept returns the drift rate (ns/s = ppb):
        slope of pps_err over the ramp baseline→now."""
        ctx = {'consecutive_outliers': 0}
        # 5 epochs, +20 ns/s ramp, 1 s apart: 600,620,...,680 at t=100..104.
        out = None
        for k in range(5):
            out = _servo_outlier_decision(
                ctx, outlier_observable_ns=600.0 + 20.0 * k,
                track_outlier_ns=500.0, converging=False, gap_recovery=False,
                ramp_accept_n=5, mono_time=100.0 + k)
        decision, drift = out
        self.assertEqual(decision, "ramp_accept")
        # baseline (t=100,e=600) → now (t=104,e=680): 80/4 = +20 ppb
        self.assertAlmostEqual(drift, 20.0, places=3)

    def test_ramp_accept_drift_none_without_time(self):
        """Back-compat: no mono_time → drift None (caller holds the freq)."""
        ctx = {'consecutive_outliers': 0}
        out = None
        for k in range(5):
            out = _decide(ctx, 600.0 + 20.0 * k)   # _decide passes no mono_time
        self.assertEqual(out, ("ramp_accept", None))

    def test_negative_ramp_drift_sign(self):
        """A negative (DO-fast) ramp yields a negative drift estimate."""
        ctx = {'consecutive_outliers': 0}
        out = None
        for k in range(5):
            out = _servo_outlier_decision(
                ctx, outlier_observable_ns=-(600.0 + 20.0 * k),
                track_outlier_ns=500.0, converging=False, gap_recovery=False,
                ramp_accept_n=5, mono_time=100.0 + k)
        self.assertEqual(out[0], "ramp_accept")
        self.assertAlmostEqual(out[1], -20.0, places=3)

    # --- spikes must still be skipped (not accepted) -------------------
    def test_isolated_spike_is_skipped_not_accepted(self):
        self.assertEqual(_decide(self.ctx, 9000.0)[0], "outlier")
        # back in range clears it
        self.assertEqual(_decide(self.ctx, 10.0)[0], "ok")
        self.assertEqual(self.ctx['consecutive_outliers'], 0)

    def test_sign_flipping_never_ramp_accepts_but_exit5_backstops(self):
        """Sign-random chaos forms no clean ramp → still hits exit-5."""
        exit5 = False
        for k in range(30):
            obs = 9000.0 if k % 2 == 0 else -9000.0   # alternating sign
            d, e5 = _decide(self.ctx, obs)
            self.assertEqual(d, "outlier")            # never ramp_accept
            exit5 = exit5 or bool(e5)
        self.assertTrue(exit5)                        # backstop still fires

    # --- the doom loop is broken: never reaches exit-5 on a real ramp --
    def test_real_ramp_never_cascades_to_exit5(self):
        """A monotonic ramp must recover repeatedly, never exit-5."""
        for k in range(200):                          # far past 30
            obs = 600.0 + 14.0 * (k % 5)              # keeps re-ramping
            _d, e5 = _decide(self.ctx, obs)
            self.assertFalse(e5, f"exit-5 fired at k={k} — doom loop intact")

    # --- the off switch / legacy behaviour -----------------------------
    def test_disabled_falls_back_to_skip_then_exit5(self):
        ctx = {'consecutive_outliers': 0}
        exit5 = False
        for k in range(30):
            d, e5 = _decide(ctx, 600.0 + 14.0 * k, n=0)   # disabled
            self.assertEqual(d, "outlier")                # only ever skip
            exit5 = exit5 or bool(e5)
        self.assertTrue(exit5)

    # --- in-range / gap / converging clear the ramp state --------------
    def test_in_range_clears_ramp_state(self):
        for k in range(3):                            # partial ramp
            _decide(self.ctx, 600.0 + 14.0 * k)
        self.assertEqual(_decide(self.ctx, 10.0)[0], "ok")
        self.assertEqual(self.ctx.get('_outlier_ramp', 0), 0)
        # a fresh partial ramp must start over, not resume
        d = [_decide(self.ctx, 600.0 + 14.0 * k)[0] for k in range(4)]
        self.assertEqual(d, ["outlier"] * 4)          # not yet 5 → no accept

    def test_gap_recovery_clears_ramp_state(self):
        for k in range(4):
            _decide(self.ctx, 600.0 + 14.0 * k)
        d, _ = _servo_outlier_decision(
            self.ctx, outlier_observable_ns=9000.0, track_outlier_ns=500.0,
            converging=False, gap_recovery=True, ramp_accept_n=5)
        self.assertEqual(d, "ok")
        self.assertEqual(self.ctx.get('_outlier_ramp', 0), 0)


if __name__ == "__main__":
    unittest.main()

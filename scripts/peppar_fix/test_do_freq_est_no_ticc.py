"""no-gnss-pps experiment: DOFreqEst.update tolerates offset_ns=None.

The existence-proof test: when the engine's source-competition
lockout suppresses every TICC- or PPS-edge-derived source, the
servo must still produce a finite adjfine from PPP dt_rx alone.

Pre-2026-05-06 behavior: the EKF's seed and TICC measurement update
both consumed offset_ns unconditionally; passing None would crash.
The engine's call site avoided that by holding the previous adjfine
when pps_err_ticc_ns was None — which produced the observed post-
chB-disconnect drift (DO free-running with a stale adjfine, not a
servo'd output).

After 2026-05-06: offset_ns is optional.  When None, DOFreqEst
skips the seed and TICC update, runs the PPP-only Kalman update +
EKF dynamics + LQR, and returns a frequency that depends only on
the PPP dt_rx history.
"""
from __future__ import annotations

import math
import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.do_freq_est import DOFreqEst


class TolerateNoneOffsetTest(unittest.TestCase):

    def test_first_epoch_with_none_does_not_crash(self):
        # No prior seed: EKF was shipped expecting the first call to
        # have offset_ns to seed φ_phc.  In the no-gnss-pps path, the
        # first call may have None — it must not crash.
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        adj = servo.update(None, dt=1.0, dt_rx_ns=0.0, dt_rx_sigma_ns=0.1)
        self.assertTrue(math.isfinite(adj))

    def test_steady_state_with_none_returns_finite_adjfine(self):
        # Run a hundred 1 Hz epochs with PPP-only measurements
        # (offset_ns=None on every call).  Adjfine must stay finite.
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        last_adj = 0.0
        for k in range(100):
            # Synthetic PPP signal: receiver clock drifting +1 ns/s,
            # measurement noise sigma 0.1 ns.
            dt_rx = float(k)
            adj = servo.update(None, dt=1.0,
                               dt_rx_ns=dt_rx, dt_rx_sigma_ns=0.1)
            self.assertTrue(math.isfinite(adj),
                            f"adj diverged at epoch {k}: {adj}")
            last_adj = adj
        # The exact adjfine depends on EKF tuning; assert only that
        # the magnitude is bounded (LQR's max_ppb cap should hold).
        self.assertLess(abs(last_adj), servo.max_ppb + 1.0)

    def test_seed_via_ticc_then_none(self):
        # Mimic the warm-disconnect scenario: first epoch has TICC
        # for the seed, subsequent epochs run PPP-only.  This is the
        # "start with chB connected, then physically pull chB" lab
        # path Bob proposed.
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        # First call seeds φ_phc from TICC.
        adj0 = servo.update(-50.0, dt=1.0, dt_rx_ns=0.0, dt_rx_sigma_ns=0.1)
        self.assertFalse(servo._need_phc_seed)
        self.assertTrue(math.isfinite(adj0))
        # Subsequent calls with None must continue to work.
        for k in range(1, 50):
            adj = servo.update(None, dt=1.0,
                               dt_rx_ns=float(k) * 0.1,
                               dt_rx_sigma_ns=0.1)
            self.assertTrue(math.isfinite(adj),
                            f"adj diverged at epoch {k}: {adj}")

    def test_no_phc_seed_attempted_when_none(self):
        # When offset_ns is None and _need_phc_seed is True, the seed
        # must NOT be attempted (no None - x[0] crash) — the flag
        # stays True until a real TICC measurement arrives.
        servo = DOFreqEst(initial_dt_rx_ns=0.0)
        self.assertTrue(servo._need_phc_seed)
        servo.update(None, dt=1.0, dt_rx_ns=0.0, dt_rx_sigma_ns=0.1)
        # Seed flag should still be True — no offset_ns ever came in.
        self.assertTrue(servo._need_phc_seed,
                        "seed flag must persist until TICC arrives")


if __name__ == "__main__":
    unittest.main()

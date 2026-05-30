"""Tests for the routedQErrArm route-counter observability (#79
follow-up).  Verifies that ``DOFreqEst.n_route_{ext,int,raw}`` are
incremented in lockstep with ``last_ticc_route`` so the engine can
surface cumulative route selection on the arm-state CSV and the
periodic ``[ROUTED_QERR]`` log.
"""
from __future__ import annotations

import os
import sys
import unittest

import numpy as np

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.do_freq_est import DOFreqEst


def _f(routed=True, **kw):
    defaults = dict(
        sigma_ticc_ns=0.060,
        sigma_do_phase_ns=0.05,
        sigma_do_freq_ppb=0.01,
        sigma_tcxo_phase_ns=2.0,
        sigma_tcxo_freq_ppb=0.1,
        initial_freq=0.0,
        initial_dt_rx_ns=0.0,
        routed_qerr=routed,
    )
    defaults.update(kw)
    f = DOFreqEst(**defaults)
    f._need_phc_seed = False
    return f


class RouteCounterInitTest(unittest.TestCase):

    def test_counters_start_at_zero(self):
        f = _f()
        self.assertEqual(f.n_route_ext, 0)
        self.assertEqual(f.n_route_int, 0)
        self.assertEqual(f.n_route_raw, 0)

    def test_counters_present_when_routing_disabled(self):
        # Legacy single-candidate path still increments n_route_int so
        # the engine doesn't need a separate "routing on?" check.
        f = _f(routed=False)
        self.assertEqual(f.n_route_ext, 0)
        self.assertEqual(f.n_route_int, 0)
        self.assertEqual(f.n_route_raw, 0)


class RouteCounterIncrementTest(unittest.TestCase):

    def test_ext_route_increments_ext_counter(self):
        # Clean external qErr → 'ext' wins (cf. test_routed_qerr_arm).
        f = _f()
        f.x = np.array([2.5, 0.0, -10.0, 0.0])
        f.update(dt=1.0, ticc_diff_ns=7.5, ticc_qerr_ns=2.5,
                 ticc_sigma_ns=0.060)
        self.assertEqual(f.last_ticc_route, 'ext')
        self.assertEqual(f.n_route_ext, 1)
        self.assertEqual(f.n_route_int, 0)
        self.assertEqual(f.n_route_raw, 0)

    def test_internal_route_increments_int_counter(self):
        # No external qErr → internal candidate runs.
        f = _f()
        f.x = np.array([2.5, 0.0, -10.0, 0.0])
        f.update(dt=1.0, ticc_diff_ns=7.5, ticc_sigma_ns=0.060)
        self.assertIn(f.last_ticc_route, ('int', 'raw'))
        if f.last_ticc_route == 'int':
            self.assertEqual(f.n_route_int, 1)
        else:
            self.assertEqual(f.n_route_raw, 1)
        self.assertEqual(f.n_route_ext, 0)

    def test_legacy_path_counts_as_internal(self):
        # routed_qerr=False ⇒ single internal-coupled path ⇒ n_route_int.
        f = _f(routed=False)
        f.x = np.array([2.5, 0.0, -10.0, 0.0])
        f.update(dt=1.0, ticc_diff_ns=7.5, ticc_sigma_ns=0.060)
        self.assertEqual(f.last_ticc_route, 'int')
        self.assertEqual(f.n_route_int, 1)
        self.assertEqual(f.n_route_ext, 0)
        self.assertEqual(f.n_route_raw, 0)

    def test_counters_accumulate_over_epochs(self):
        # Clean-ext sequence: every epoch routes to 'ext'.
        f = _f()
        for _ in range(5):
            f.x = np.array([2.5, 0.0, -10.0, 0.0])  # reset to sub-tick=2.5
            f.update(dt=1.0, ticc_diff_ns=7.5, ticc_qerr_ns=2.5,
                     ticc_sigma_ns=0.060)
        total = f.n_route_ext + f.n_route_int + f.n_route_raw
        self.assertEqual(total, 5)
        self.assertEqual(f.n_route_ext, 5)

    def test_no_increment_when_no_ticc_update(self):
        # An epoch without ticc_diff_ns doesn't go through the router.
        f = _f()
        f.update(dt=1.0)  # no TICC, no other arms
        self.assertEqual(f.n_route_ext, 0)
        self.assertEqual(f.n_route_int, 0)
        self.assertEqual(f.n_route_raw, 0)


if __name__ == "__main__":
    unittest.main()

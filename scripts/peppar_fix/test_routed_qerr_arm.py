"""Tests for the routedQErrArm per-edge TICC selector.

Each TICC edge routes to one of {external-qErr, internal-qerr(x[0]),
raw} based on chi².  External fires when well-correlated; a
mis-correlated (tick-scale) external qErr falls through to raw; an
F10T's uncorrelated qErr always routes to raw.  See dayplan
routedQErrArm.
"""
import numpy as np
import unittest

from peppar_fix.do_freq_est import DOFreqEst, _qerr


def _f(routed=True, **kw):
    # sigma_do_phase=0.05 ns = the char-derived steady-state Q the
    # router is designed to operate in.  With the 0.92 ns default, Q
    # re-inflates P[2,2] each predict to σ≈0.93 ns, where even a
    # full-tick (8 ns) mismatch is only ~8.6σ (χ²≈74 < 100) and would
    # be wrongly accepted — the router's tick-discrimination depends on
    # the honest small Q (see qFromCharPerActuator coupling).
    defaults = dict(
        sigma_ticc_ns=0.060,
        sigma_do_phase_ns=0.05,
        sigma_do_freq_ppb=0.01,
        sigma_tcxo_phase_ns=2.0,
        sigma_tcxo_freq_ppb=0.1,
        initial_freq=0.0,
        initial_dt_rx_ns=0.0,   # _tcxo_initialized=True (internal qerr live)
        routed_qerr=routed,
    )
    defaults.update(kw)
    f = DOFreqEst(**defaults)
    f._need_phc_seed = False
    return f


class RoutedQErrSelectorTest(unittest.TestCase):

    def test_well_correlated_external_fires(self):
        """A consistent external qErr → 'ext' route, clean x[2]-only."""
        f = _f()
        f.x = np.array([2.5, 0.0, -10.0, 0.0])  # x[0] sub-tick=2.5ns
        # True DO phase -10 ns; external qErr matches the sub-tick.
        # ticc_diff = -x[2] - true_sub_tick = 10 - 2.5 = 7.5;
        # corrected z = ticc_diff + qerr = 7.5 + 2.5 = 10 = -x[2]. innov≈0.
        f.update(dt=1.0, ticc_diff_ns=7.5, ticc_qerr_ns=2.5,
                 ticc_sigma_ns=0.060)
        self.assertEqual(f.last_ticc_route, 'ext')

    def test_miscorrelated_external_falls_through(self):
        """An external qErr wrong by ~a tick is a chi² outlier → not 'ext'.

        Steady-state P[2,2]=0.01 (σ_x2≈0.1 ns, realistic for converged
        chA TDEV(1s)~100 ps): an 8 ns tick-mismatch is ~80σ, χ²≈6400.
        """
        f = _f()
        f.x = np.array([2.5, 0.0, -10.0, 0.0])
        f.P = np.diag([0.01, 0.01, 0.01, 0.01])  # converged
        # ticc_diff consistent with sub-tick 2.5, but supplied qErr is
        # off by a full tick (8 ns) → corrected z wrong by 8 ns.
        f.update(dt=1.0, ticc_diff_ns=7.5, ticc_qerr_ns=2.5 - 8.0,
                 ticc_sigma_ns=0.060)
        self.assertNotEqual(f.last_ticc_route, 'ext')

    def test_no_external_uses_internal_or_raw(self):
        """Without ticc_qerr_ns, route is internal or raw, never ext."""
        f = _f()
        f.x = np.array([2.5, 0.0, -10.0, 0.0])
        f.update(dt=1.0, ticc_diff_ns=7.5, ticc_sigma_ns=0.060)
        self.assertIn(f.last_ticc_route, ('int', 'raw'))

    def test_f10t_uncorrelated_qerr_routes_to_raw(self):
        """Random (uncorrelated) external qErr → eventually raw floor."""
        f = _f()
        f.x = np.array([0.0, 0.0, 0.0, 0.0])
        f.P = np.diag([1.0, 0.01, 0.01, 0.01])  # converged x[2]
        rng = np.random.default_rng(0)
        routes = []
        for _ in range(30):
            # ticc_diff small (DO near 0); qErr random ±8ns (F10T 16ns step)
            qerr = float(rng.uniform(-8, 8))
            f.update(dt=1.0, ticc_diff_ns=0.2, ticc_qerr_ns=qerr,
                     ticc_sigma_ns=0.060)
            routes.append(f.last_ticc_route)
        # Most random-qErr edges should NOT take the ext route
        ext_frac = routes.count('ext') / len(routes)
        self.assertLess(ext_frac, 0.5)

    def test_disabled_routing_always_internal(self):
        """routed_qerr=False keeps the single internal path."""
        f = _f(routed=False)
        f.x = np.array([2.5, 0.0, -10.0, 0.0])
        f.update(dt=1.0, ticc_diff_ns=7.5, ticc_qerr_ns=2.5,
                 ticc_sigma_ns=0.060)
        self.assertEqual(f.last_ticc_route, 'int')

    def test_raw_R_is_sawtooth_variance(self):
        """The raw candidate's R = R_base + tick²/12, not (tick/2)²."""
        f = _f()
        f.x = np.array([0.0, 0.0, 0.0, 0.0])
        f.P = np.diag([1.0, 0.01, 1.0, 0.01])
        # Force raw: external wildly wrong, internal also wrong (huge
        # ticc_diff with tight P) → fall to raw.
        z_eff, h, H, R, name = f._route_ticc_arm(
            f.x, f.P, ticc_diff_ns=0.0, ticc_qerr_ns=50.0,
            R_base=0.0036, R_lin=0.0)
        # When raw is chosen its R must carry the sawtooth variance.
        if name == 'raw':
            self.assertAlmostEqual(R, 0.0036 + 64.0 / 12.0, places=4)

    def test_external_does_not_couple_x0(self):
        """The ext route uses H=[0,0,-1,0] — no x[0] coupling."""
        f = _f()
        f.x = np.array([100.0, 0.0, -10.0, 0.0])
        f.P = np.diag([50.0, 0.01, 1.0, 0.01])  # large P[0,0]
        x0_before = f.x[0]
        f.update(dt=1.0, ticc_diff_ns=7.5, ticc_qerr_ns=2.5,
                 ticc_sigma_ns=0.060)
        if f.last_ticc_route == 'ext':
            # ext update touches only x[2]; x[0] only moves via predict
            self.assertAlmostEqual(f.x[0], x0_before, delta=1.0)


if __name__ == "__main__":
    unittest.main()

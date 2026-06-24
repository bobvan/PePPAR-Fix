"""Tests for the COMPARATIVE-gate TICC router (latestQErrChiSelect).

The routed-qErr gate picks the smallest-chi² candidate (argmin): ext wins only
if it BEATS raw.  A stale/wrong ext loses to raw → no poison.  This is the ONLY
routing mode now that the legacy absolute-gate v1 router and the qVIR-gated v2
router were retired in retireQerrMatchingCode (I-064807); `routed_qerr=True`
selects it.  See docs/latest-qerr-chi-select.md and
scripts/proto_latest_qerr_chi.py (the servo_sim go/no-go).
"""
import numpy as np
import unittest

from peppar_fix.do_freq_est import DOFreqEst, _qerr, _CHI2_GATE_THRESHOLD


def _f(**kw):
    defaults = dict(
        sigma_ticc_ns=0.060,
        sigma_do_phase_ns=0.05,
        sigma_do_freq_ppb=0.01,
        sigma_tcxo_phase_ns=2.0,
        sigma_tcxo_freq_ppb=0.1,
        initial_freq=0.0,
        initial_dt_rx_ns=0.0,   # _tcxo_initialized=True
        routed_qerr=True,       # comparative is the only routed mode now
    )
    defaults.update(kw)
    f = DOFreqEst(**defaults)
    f._need_phc_seed = False
    return f


def _route(f, x, ticc_diff_ns, ticc_qerr_ns, p22=0.05**2):
    """Call _route_ticc_arm at a given state; return the chosen route name."""
    f.x = np.array(x, dtype=float)
    P = np.diag([4.0, 0.01, p22, 0.0001])
    R_base = 0.060 ** 2
    _, _, _, _, name = f._route_ticc_arm(f.x, P, ticc_diff_ns, ticc_qerr_ns,
                                         R_base, R_lin=0.0)
    return name


class ComparativeGateTest(unittest.TestCase):

    # P22 ≈ (0.8 ns)² — the realistic at-lock ext √S where an off-by-edge
    # ext lands at chi²≈17 (the poison an absolute-100 gate would let through).
    _P22_LOCK = 0.8 ** 2

    def test_correct_ext_wins(self):
        """A correct ext qErr (matches the rx sub-tick) beats raw → 'ext'."""
        f = _f()
        x = [2.5, 0.0, -10.0, 0.0]            # rx sub-tick 2.5 ns, DO -10 ns
        ticc_diff = -x[2] - _qerr(x[0], 8.0)  # = 10 - 2.5 = 7.5
        # correct ext qErr = +qerr(x0) = +2.5 → z_ext = 7.5+2.5 = 10 = -x2
        self.assertEqual(_route(f, x, ticc_diff, _qerr(x[0], 8.0)), "ext")

    def test_offbyedge_ext_NOT_selected(self):
        """An off-by-edge ext (~3.3 ns) is NOT selected by the comparative
        gate — it falls to int/raw (no poison)."""
        f = _f()
        x = [2.5, 0.0, -10.0, 0.0]
        ticc_diff = -x[2] - _qerr(x[0], 8.0)
        wrong = _qerr(x[0], 8.0) + 3.3        # off-by-edge ~3.3 ns
        self.assertNotEqual(
            _route(f, x, ticc_diff, wrong, p22=self._P22_LOCK), "ext")

    def test_offbyedge_ext_WOULD_pass_an_absolute_gate(self):
        """Why the comparative gate is mandatory: at the realistic at-lock
        P22, the off-by-edge ext's own chi² is ≈17 ≪ the absolute threshold
        (~100) — so a (now-removed) absolute gate would have ACCEPTED it and
        poisoned z.  The comparative gate instead routes it to int/raw because
        their chi² is smaller.  Computed inline (the absolute path is gone)."""
        f = _f()
        x = [2.5, 0.0, -10.0, 0.0]
        ticc_diff = -x[2] - _qerr(x[0], 8.0)
        wrong = _qerr(x[0], 8.0) + 3.3
        # ext candidate: z = ticc_diff + wrong, h = -x[2], H = [0,0,-1,0].
        P22 = self._P22_LOCK
        R_base = 0.060 ** 2
        z_ext = ticc_diff + wrong
        innov = z_ext - (-x[2])
        S = P22 + R_base               # H P Hᵀ + R for H=[0,0,-1,0]
        chi2_ext = innov ** 2 / S
        # The off-by-edge ext clears the absolute gate (would be accepted)...
        self.assertLess(chi2_ext, _CHI2_GATE_THRESHOLD)
        # ...but the comparative gate rejects it (routes to int/raw).
        self.assertNotEqual(
            _route(f, x, ticc_diff, wrong, p22=P22), "ext")

    def test_no_ext_qerr_routes_int_or_raw(self):
        """With no ext qErr, comparative picks the better of int/raw."""
        f = _f()
        x = [2.5, 0.0, -10.0, 0.0]
        ticc_diff = -x[2] - _qerr(x[0], 8.0)
        self.assertIn(_route(f, x, ticc_diff, None), ("int", "raw"))

    def test_routed_qerr_off_is_not_routed(self):
        """routed_qerr=False → the single internal path (no routing);
        _route_ticc_arm is not invoked (the --no-qerr-latest-chi behavior)."""
        f = _f(routed_qerr=False)
        self.assertFalse(f._routed_qerr)

    def test_wrong_ext_magnitude_sweep(self):
        """Across off-by-edge magnitudes, comparative never selects the wrong
        ext (routes int/raw) at the realistic at-lock P22."""
        f = _f()
        x = [2.5, 0.0, -10.0, 0.0]
        ticc_diff = -x[2] - _qerr(x[0], 8.0)
        base = _qerr(x[0], 8.0)
        for err in (2.0, 3.3, 4.0, 6.0, 8.0):
            self.assertNotEqual(
                _route(f, x, ticc_diff, base + err, p22=self._P22_LOCK), "ext",
                f"off-by {err} ns must not route ext")


if __name__ == "__main__":
    unittest.main()

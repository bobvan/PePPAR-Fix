"""Direct tests for rawx_to_observations — the RAWX→IF-observations transform
factored out of serial_reader for pos_replay stage 2b.  Previously this logic
had no direct coverage (only end-to-end through serial_reader)."""
import os
import sys
import types
import unittest

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import realtime_ppp as r  # noqa: E402

# Faithful GPS L1CA/L2CL config (matches IF_PAIR_PARAMS).
_A1, _A2 = r.IF_PAIR_PARAMS[("GPS-L1CA", "GPS-L2CL")][1:]
_SIG_NAMES = {(0, 0): "GPS-L1CA", (0, 3): "GPS-L2CL"}
_SIG_LOOKUP = {
    "GPS-L1CA": (0, "G", "f1", _A1, _A2, "GPS-L1CA"),
    "GPS-L2CL": (0, "G", "f2", _A1, _A2, "GPS-L2CL"),
}


def _rawx(gnss, sig, sv, pr, cp, *, cno=None, lock=None,
          prv=None, cpv=None, half=None, numMeas=None):
    n = numMeas if numMeas is not None else len(gnss)
    return types.SimpleNamespace(
        rcvTow=100.0, week=2300, leapS=18, numMeas=n,
        gnssId=gnss, sigId=sig, svId=sv, prMes=pr, cpMes=cp,
        cno=cno or [45] * n, locktime=lock or [5000.0] * n,
        prValid=prv or [True] * n, cpValid=cpv or [True] * n,
        halfCyc=half or [True] * n, clk_reset=False)


def _run(rawx, systems={"gps"}, ssr=None):
    return r.rawx_to_observations(rawx, systems, ssr, _SIG_NAMES, _SIG_LOOKUP,
                                  set())


class TestRawxToObservations(unittest.TestCase):
    def test_dual_freq_forms_if_observation(self):
        # one GPS SV (G01) on L1+L2 → one IF observation
        rawx = _rawx([0, 0], [0, 3], [1, 1],
                     [22_000_000.0, 22_000_000.0],
                     [115_000_000.0, 90_000_000.0])
        obs, raw_obs, n_off, n_single = _run(rawx)
        self.assertEqual(len(obs), 1)
        o = obs[0]
        self.assertEqual(o["sv"], "G01")
        self.assertEqual(o["sys"], "gps")
        # pr_if = a1*pr1 - a2*pr2; equal PRs → (a1-a2)*pr = 1.0*pr
        self.assertAlmostEqual(o["pr_if"], 22_000_000.0, places=0)
        self.assertIsNotNone(o["phi_if_m"])
        self.assertEqual(o["f1_sig_name"], "GPS-L1CA")
        self.assertEqual(o["f2_sig_name"], "GPS-L2CL")
        self.assertEqual(n_off, 0)
        self.assertEqual(n_single, 0)

    def test_system_filter_excludes_off_constellation(self):
        rawx = _rawx([0, 0], [0, 3], [1, 1],
                     [22_000_000.0, 22_000_000.0],
                     [115_000_000.0, 90_000_000.0])
        obs, _raw, n_off, n_single = _run(rawx, systems={"gal"})
        self.assertEqual(obs, [])
        self.assertEqual(n_off, 1)          # the GPS SV is off-constellation

    def test_single_freq_is_pr_only_not_if(self):
        # only L1 tracked → single-freq, counted but no IF observation
        rawx = _rawx([0], [0], [1], [22_000_000.0], [115_000_000.0])
        obs, _raw, n_off, n_single = _run(rawx)
        self.assertEqual(obs, [])
        self.assertEqual(n_single, 1)

    def test_invalid_pr_dropped(self):
        # prValid False on L2 → SV has no usable f2 → no IF obs
        rawx = _rawx([0, 0], [0, 3], [1, 1],
                     [22_000_000.0, 22_000_000.0],
                     [115_000_000.0, 90_000_000.0],
                     prv=[True, False])
        obs, _raw, _n_off, n_single = _run(rawx)
        self.assertEqual(obs, [])
        self.assertEqual(n_single, 1)       # only f1 survived → single-freq

    def test_out_of_range_pr_dropped(self):
        rawx = _rawx([0, 0], [0, 3], [1, 1],
                     [500.0, 22_000_000.0],          # f1 pr < 1e6 → dropped
                     [115_000_000.0, 90_000_000.0])
        obs, _raw, _n_off, n_single = _run(rawx)
        self.assertEqual(obs, [])
        self.assertEqual(n_single, 1)

    def test_unknown_signal_skipped(self):
        # sigId 9 not in _SIG_NAMES → measurement skipped entirely
        rawx = _rawx([0, 0], [9, 3], [1, 1],
                     [22_000_000.0, 22_000_000.0],
                     [115_000_000.0, 90_000_000.0])
        obs, _raw, _n_off, n_single = _run(rawx)
        self.assertEqual(obs, [])           # only f2 known → single-freq
        self.assertEqual(n_single, 1)


class TestPrefixToSysModuleLevel(unittest.TestCase):
    """#239 regression: PREFIX_TO_SYS was defined inside the extracted
    obs-build block but still used by serial_reader's downstream OBS_ADMIT/diag
    code → undefined-name NameError there (caught by pyflakes, missed by the
    suite).  Pin that it's module-level so both can resolve it."""

    def test_prefix_to_sys_is_module_level(self):
        self.assertTrue(hasattr(r, "PREFIX_TO_SYS"))
        self.assertEqual(r.PREFIX_TO_SYS["G"], "gps")
        self.assertEqual(r.PREFIX_TO_SYS["E"], "gal")
        self.assertEqual(r.PREFIX_TO_SYS["C"], "bds")


class TestGfDiagPersistsAcrossEpochs(unittest.TestCase):
    """Charlie #239: GF-DIAG is a two-epoch one-shot — its state dict must
    PERSIST across calls (set epoch1 on call 1, reach epoch2 on call 2), not
    reset every call.  An incomplete _gf_diag rename made hasattr() always
    False → the dict reset each call → epoch2 never reached → diagnostic
    silently never fired.  This pins persistence."""

    def setUp(self):
        # isolate from any state left by serial_reader / other tests
        if hasattr(r.rawx_to_observations, "_gf_diag"):
            del r.rawx_to_observations._gf_diag

    def _epoch(self):
        return _rawx([0, 0], [0, 3], [1, 1],
                     [22_000_000.0, 22_000_000.0],
                     [115_000_000.0, 90_000_000.0])

    def test_state_persists_and_reaches_epoch2(self):
        _run(self._epoch())                 # call 1 → epoch1 recorded
        diag = r.rawx_to_observations._gf_diag
        slot = diag[("gps", "G01")]
        self.assertIsNotNone(slot["epoch1"])
        self.assertIsNone(slot["epoch2"])   # not yet

        _run(self._epoch())                 # call 2 → epoch2 reached (persisted)
        slot = r.rawx_to_observations._gf_diag[("gps", "G01")]
        self.assertIsNotNone(slot["epoch2"])  # would be None if dict reset


if __name__ == "__main__":
    unittest.main()

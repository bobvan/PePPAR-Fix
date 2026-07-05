"""Tests for the RTCM 3 MSM observation adapter (I-030423).

Covers the MSM decode (sig-id map, observable reconstruction, MSM4 vs MSM7
fields, half-cycle polarity, cell-mask indexing), the raw_obs assembly, and —
the load-bearing one — format PARITY: the MSM path and the UBX-RXM-RAWX path
emit byte-identical obs dicts for the same underlying observables, because both
run the shared realtime_ppp.raw_obs_to_if_observations.

Fake pyrtcm messages are built as SimpleNamespace with DF* attributes (the same
lightweight-fixture approach test_rawx_to_observations.py uses for rawx), so no
network or captured binary blob is needed.
"""
import os
import sys
import types
import unittest
from types import SimpleNamespace

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import realtime_ppp as r  # noqa: E402
from peppar_fix import rtcm_msm_obs as m  # noqa: E402

_C = 299792458.0


def _msm(msg_num, sats, sigs, rough, cells, *, cellmask=None):
    """Build a fake pyrtcm MSM message.  `cells` are the SET cells in sat-major
    order; each is {pr_fine(ms), ph_fine(ms)|None, cno, lock, half}."""
    extended = (msg_num % 10) in (6, 7)
    nsat, nsig = len(sats), len(sigs)
    # per-constellation epoch field: GPS DF004, GAL DF248, BDS DF427
    epoch_df = {107: "DF004", 109: "DF248", 112: "DF427"}.get(msg_num // 10, "DF004")
    attrs = {
        "DF002": msg_num, epoch_df: 100000,
        "DF394": sum(1 << (64 - p) for p in sats),
        "DF395": sum(1 << (32 - s) for s in sigs),
        "DF396": (cellmask if cellmask is not None else (1 << (nsat * nsig)) - 1)}
    for i, rg in enumerate(rough):
        attrs[f"DF397_{i+1:02d}"] = int(rg)
        attrs[f"DF398_{i+1:02d}"] = rg - int(rg)
    for ci, c in enumerate(cells, start=1):
        pr_df, ph_df, cno_df, lk_df = (
            ("DF405", "DF406", "DF408", "DF407") if extended
            else ("DF400", "DF401", "DF403", "DF402"))
        attrs[f"{pr_df}_{ci:02d}"] = c["pr_fine"]
        attrs[f"{ph_df}_{ci:02d}"] = c.get("ph_fine")
        attrs[f"{cno_df}_{ci:02d}"] = c.get("cno")
        attrs[f"{lk_df}_{ci:02d}"] = c.get("lock")
        attrs[f"DF420_{ci:02d}"] = c.get("half", 0)
    return SimpleNamespace(**attrs)


class BitsAndLockTest(unittest.TestCase):
    def test_bits_msb_first_1based(self):
        self.assertEqual(m._bits(0b101, 3), [1, 3])
        self.assertEqual(m._bits(1 << 63, 64), [1])
        self.assertEqual(m._bits(1, 64), [64])

    def test_lock_ms_df402_table(self):
        self.assertEqual(m._lock_ms(0, False), 0.0)
        self.assertEqual(m._lock_ms(1, False), 32.0)
        self.assertEqual(m._lock_ms(5, False), 512.0)
        self.assertEqual(m._lock_ms(15, False), 524288.0)

    def test_lock_ms_df407_monotonic(self):
        vals = [m._lock_ms(n, True) for n in range(0, 512, 16)]
        self.assertTrue(all(b >= a for a, b in zip(vals, vals[1:])))
        self.assertEqual(m._lock_ms(0, True), 0.0)
        self.assertEqual(m._lock_ms(63, True), 63.0)   # linear region
        self.assertEqual(m._lock_ms(64, True), 64.0)   # slope-2 region starts

    def test_lock_ms_none(self):
        self.assertEqual(m._lock_ms(None, False), 0.0)


class DecodeMsmTest(unittest.TestCase):
    def _known(self, pr_m, freq):
        """DF fields (rough ms + fine ms) that reconstruct exactly to pr_m."""
        rng_ms = pr_m * 1000.0 / _C
        return int(rng_ms), rng_ms - int(rng_ms)

    def test_msm7_reconstructs_pr_cp_and_maps_signals(self):
        # G05 with L1CA (sig 2) + L2W (sig 10), full 1x2 grid, MSM7.
        f1, f2 = 1575.42e6, 1227.60e6
        pr1, pr2 = 22_000_000.123, 22_000_003.456
        r1i, r1f = self._known(pr1, f1)
        r2i, r2f = self._known(pr2, f2)
        # single rough per sat; fine PR differs per signal → share rough, adjust fine
        rough = float(r1i)                # integer ms shared
        msg = _msm(1077, [5], [2, 10], [rough],
                   [{"pr_fine": (pr1 * 1000 / _C) - rough, "ph_fine": (pr1 * 1000 / _C) - rough,
                     "cno": 47.0, "lock": 5, "half": 0},
                    {"pr_fine": (pr2 * 1000 / _C) - rough, "ph_fine": (pr2 * 1000 / _C) - rough,
                     "cno": 44.0, "lock": 5, "half": 0}])
        prefix, tow, cells = m.decode_msm_obs(msg)
        self.assertEqual(prefix, "G")
        self.assertEqual(tow, 100000)
        by_sig = {c["sig_name"]: c for c in cells}
        self.assertEqual(set(by_sig), {"GPS-L1CA", "GPS-L2W"})
        self.assertAlmostEqual(by_sig["GPS-L1CA"]["pr_m"], pr1, places=3)
        self.assertAlmostEqual(by_sig["GPS-L2W"]["pr_m"], pr2, places=3)
        # cp_cyc = phaserange_m * f / c ; phaserange == pr here
        self.assertAlmostEqual(by_sig["GPS-L1CA"]["cp_cyc"], pr1 * f1 / _C, places=1)
        self.assertEqual(by_sig["GPS-L1CA"]["sv"], "G05")
        self.assertEqual(by_sig["GPS-L1CA"]["lock_ms"], 5.0)   # MSM7 DF407 linear region
        self.assertTrue(by_sig["GPS-L1CA"]["half_ok"])

    def test_msm4_uses_df400_fields(self):
        msg = _msm(1074, [5], [2], [73.0],
                   [{"pr_fine": 0.0004, "ph_fine": 0.0004, "cno": 45.0,
                     "lock": 5, "half": 0}])
        _, _, cells = m.decode_msm_obs(msg)
        self.assertEqual(len(cells), 1)
        self.assertAlmostEqual(cells[0]["pr_m"], _C * 73.0004 / 1000.0, places=3)

    def test_half_cycle_polarity_df420_1_is_ambiguous(self):
        # DF420 == 1 → half-cycle ambiguity present → half_ok False (drop-worthy)
        msg = _msm(1077, [5], [2], [73.0],
                   [{"pr_fine": 0.0004, "ph_fine": 0.0004, "cno": 45.0,
                     "lock": 5, "half": 1}])
        _, _, cells = m.decode_msm_obs(msg)
        self.assertFalse(cells[0]["half_ok"])

    def test_unsupported_constellation_returns_none(self):
        # 108x = GLONASS (FDMA) — still out of scope (a deliberate follow-on)
        self.assertIsNone(m.decode_msm_obs(_msm(1087, [5], [2], [73.0],
                          [{"pr_fine": 0.0004}])))

    def test_galileo_msm7_decodes_with_df248_epoch(self):
        # E11 with E1C (sig 2) + E5aQ (sig 23); GAL epoch time is DF248, not DF004
        msg = _msm(1097, [11], [2, 23], [73.0],
                   [{"pr_fine": 0.0004, "ph_fine": 0.0004, "cno": 46, "lock": 5},
                    {"pr_fine": 0.0005, "ph_fine": 0.0005, "cno": 44, "lock": 5}])
        prefix, tow, cells = m.decode_msm_obs(msg)
        self.assertEqual(prefix, "E")
        self.assertEqual(tow, 100000)                 # read from DF248
        by = {c["sig_name"]: c for c in cells}
        self.assertEqual(set(by), {"GAL-E1C", "GAL-E5aQ"})
        self.assertEqual(by["GAL-E1C"]["sv"], "E11")
        self.assertAlmostEqual(by["GAL-E5aQ"]["freq_hz"], 1176.45e6)   # E5a band

    def test_galileo_e5b_is_7band(self):
        # sig 14 = 7I = E5bI (1207.14 MHz), NOT E5a/AltBOC
        msg = _msm(1097, [11], [14], [73.0], [{"pr_fine": 0.0004}])
        cells = m.decode_msm_obs(msg)[2]
        self.assertEqual(cells[0]["sig_name"], "GAL-E5bI")
        self.assertAlmostEqual(cells[0]["freq_hz"], 1207.14e6)

    def test_msm1_2_3_rejected(self):
        # MSM1/2/3 (num%10 in 1-3) have a compact/different field layout — must
        # be rejected, not misdecoded as MSM4 (bravo #281).
        for num in (1071, 1072, 1073):
            self.assertIsNone(
                m.decode_msm_obs(_msm(num, [5], [2], [73.0],
                                      [{"pr_fine": 0.0004}])),
                f"MSM{num % 10} should be rejected")
        # MSM4 (num%10==4) is still accepted
        self.assertIsNotNone(m.decode_msm_obs(_msm(1074, [5], [2], [73.0],
                             [{"pr_fine": 0.0004, "ph_fine": 0.0004}])))

    def test_unmapped_signal_dropped(self):
        # sig id 3 (1P) has no engine sig_name → not emitted
        msg = _msm(1077, [5], [2, 3], [73.0],
                   [{"pr_fine": 0.0004, "ph_fine": 0.0004, "cno": 45, "lock": 5},
                    {"pr_fine": 0.0005, "ph_fine": 0.0005, "cno": 45, "lock": 5}])
        _, _, cells = m.decode_msm_obs(msg)
        self.assertEqual([c["sig_name"] for c in cells], ["GPS-L1CA"])

    def test_cell_mask_partial_grid(self):
        # 2 sats x 2 sigs, but only cells (sat0,sig0) and (sat1,sig1) present.
        cm = 0b1001                       # sat-major MSB-first: s0g0=1, s1g1=1
        msg = _msm(1077, [5, 10], [2, 16], [70.0, 72.0],
                   [{"pr_fine": 0.0004, "ph_fine": 0.0004, "cno": 45, "lock": 5},
                    {"pr_fine": 0.0006, "ph_fine": 0.0006, "cno": 43, "lock": 5}],
                   cellmask=cm)
        _, _, cells = m.decode_msm_obs(msg)
        got = {(c["sv"], c["sig_name"]) for c in cells}
        self.assertEqual(got, {("G05", "GPS-L1CA"), ("G10", "GPS-L2CL")})


class MergeRawObsTest(unittest.TestCase):
    def test_builds_roles_and_alphas(self):
        look = m.default_gps_sig_lookup("GPS-L2CL")
        msg = _msm(1077, [5], [2, 16], [70.0],
                   [{"pr_fine": 0.0004, "ph_fine": 0.0004, "cno": 47, "lock": 5},
                    {"pr_fine": 0.0005, "ph_fine": 0.0005, "cno": 44, "lock": 5}])
        raw = m.merge_msm_into_raw_obs(msg, look)
        self.assertEqual(set(raw["G05"]), {"f1", "f2"})
        self.assertEqual(raw["G05"]["f1"]["sig_name"], "GPS-L1CA")
        self.assertEqual(raw["G05"]["f2"]["sig_name"], "GPS-L2CL")
        # alphas match the engine's L1/L2 IF params on both roles
        a1, a2 = r.IF_PAIR_PARAMS[("GPS-L1CA", "GPS-L2CL")][1:]
        self.assertAlmostEqual(raw["G05"]["f1"]["alpha_f1"], a1)
        self.assertAlmostEqual(raw["G05"]["f2"]["alpha_f2"], a2)
        self.assertEqual(raw["G05"]["f1"]["cno"], 47)


class DefaultSigLookupTest(unittest.TestCase):
    def test_default_is_l1_l2w(self):
        look = m.default_gps_sig_lookup()
        self.assertEqual(look["GPS-L1CA"][2], "f1")
        self.assertEqual(look["GPS-L2W"][2], "f2")

    def test_override_l2cl(self):
        look = m.default_gps_sig_lookup("GPS-L2CL")
        self.assertIn("GPS-L2CL", look)
        self.assertNotIn("GPS-L2W", look)

    def test_multignss_gps_gal(self):
        # systems={gps,gal} → both IF pairs in the lookup (E1C+E5aQ for GAL)
        look = m.default_sig_lookup({"gps", "gal"})
        self.assertEqual(look["GPS-L1CA"][2], "f1")
        self.assertEqual(look["GPS-L2W"][2], "f2")
        self.assertEqual(look["GAL-E1C"][2], "f1")
        self.assertEqual(look["GAL-E5aQ"][2], "f2")

    def test_gps_only_has_no_gal_pair(self):
        look = m.default_sig_lookup({"gps"})
        self.assertNotIn("GAL-E1C", look)


# --- test-only rawx fixture, matching test_rawx_to_observations ---
_A1, _A2 = r.IF_PAIR_PARAMS[("GPS-L1CA", "GPS-L2CL")][1:]
_SIG_NAMES = {(0, 0): "GPS-L1CA", (0, 3): "GPS-L2CL"}
_SIG_LOOKUP = {
    "GPS-L1CA": (0, "G", "f1", _A1, _A2, "GPS-L1CA"),
    "GPS-L2CL": (0, "G", "f2", _A1, _A2, "GPS-L2CL"),
}


def _rawx(pr, cp, sv=5, cno=45, lock=512.0):
    return types.SimpleNamespace(
        rcvTow=100.0, week=2300, leapS=18, numMeas=2,
        gnssId=[0, 0], sigId=[0, 3], svId=[sv, sv], prMes=pr, cpMes=cp,
        cno=[cno, cno], locktime=[lock, lock],
        prValid=[True, True], cpValid=[True, True],
        halfCyc=[True, True], clk_reset=False)


class ParityWithRawxTest(unittest.TestCase):
    """The load-bearing invariant: MSM and RXM emit IDENTICAL obs dicts for the
    same observables, because both run raw_obs_to_if_observations."""

    def test_msm_obs_equals_rawx_obs(self):
        # Build an MSM epoch for G05 L1CA+L2CL, decode the exact pr/cp it yields,
        # then feed those same pr/cp to the RXM path and require dict equality.
        look = m.default_gps_sig_lookup("GPS-L2CL")
        f1, f2 = 1575.42e6, 1227.60e6
        rough = 73.0
        msg = _msm(1077, [5], [2, 16], [rough],
                   [{"pr_fine": 0.0004, "ph_fine": 0.00042, "cno": 45, "lock": 9},
                    {"pr_fine": 0.0005, "ph_fine": 0.00051, "cno": 45, "lock": 9}])
        # what the MSM decode produced (so RXM gets byte-identical inputs)
        by = {c["sig_name"]: c for c in m.decode_msm_obs(msg)[2]}
        pr1, pr2 = by["GPS-L1CA"]["pr_m"], by["GPS-L2CL"]["pr_m"]
        cp1, cp2 = by["GPS-L1CA"]["cp_cyc"], by["GPS-L2CL"]["cp_cyc"]
        lock_ms = m._lock_ms(9, True)     # 512 → set rawx locktime to match

        msm_obs, _, _, _ = m.msm_messages_to_observations(
            [msg], {"gps"}, None, look)
        rawx_obs, _, _, _ = r.rawx_to_observations(
            _rawx([pr1, pr2], [cp1, cp2], cno=45, lock=lock_ms),
            {"gps"}, None, _SIG_NAMES, _SIG_LOOKUP, set())

        self.assertEqual(len(msm_obs), 1)
        self.assertEqual(len(rawx_obs), 1)
        self.assertEqual(msm_obs[0], rawx_obs[0])   # byte-identical obs dict

    def test_single_freq_msm_forms_no_if(self):
        # Only L1CA present → no f2 → no IF observation (matches RXM behavior).
        look = m.default_gps_sig_lookup("GPS-L2CL")
        msg = _msm(1077, [5], [2], [73.0],
                   [{"pr_fine": 0.0004, "ph_fine": 0.0004, "cno": 45, "lock": 5}])
        obs, _, _, n_single = m.msm_messages_to_observations(
            [msg], {"gps"}, None, look)
        self.assertEqual(obs, [])
        self.assertEqual(n_single, 1)

    def test_galileo_forms_if_obs_through_shared_former(self):
        # GAL 1097 (E1C+E5aQ) → the shared IF-former → an obs with sys='gal'.
        # The second constellation is what breaks the GPS-only degeneracy (AR).
        look = m.default_sig_lookup({"gps", "gal"})
        msg = _msm(1097, [11], [2, 23], [73.0],
                   [{"pr_fine": 0.0004, "ph_fine": 0.0004, "cno": 46, "lock": 5},
                    {"pr_fine": 0.0005, "ph_fine": 0.0005, "cno": 44, "lock": 5}])
        obs, _, _, _ = m.msm_messages_to_observations(
            [msg], {"gps", "gal"}, None, look)
        self.assertEqual(len(obs), 1)
        self.assertEqual(obs[0]["sv"], "E11")
        self.assertEqual(obs[0]["sys"], "gal")
        self.assertEqual(obs[0]["f1_sig_name"], "GAL-E1C")
        self.assertEqual(obs[0]["f2_sig_name"], "GAL-E5aQ")


if __name__ == "__main__":
    unittest.main()

"""Tests for the Septentrio SBF observation adapter (I-030423).

Decode is validated against a REAL MeasEpoch block captured from the lab
mosaic-T (testdata/mosaic_measepoch.sbf, 2026-07-05) — ground truth, not a
synthetic fixture.  Confirms per-SV-per-signal reconstruction, the SVID and
signal maps, and that the cells feed the SHARED IF-former (parity with the
RTCM MSM / RXM-RAWX paths).
"""
import io
import os
import sys
import unittest

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from pysbf2 import SBFReader  # noqa: E402

from peppar_fix import sbf_obs as s  # noqa: E402

_FIXTURE = os.path.join(os.path.dirname(__file__), "testdata",
                        "mosaic_measepoch.sbf")


def _load_meas_epoch():
    data = open(_FIXTURE, "rb").read()
    for _raw, parsed in SBFReader(io.BytesIO(data), quitonerror=0):
        if parsed is not None and parsed.identity == "MeasEpoch":
            return parsed
    raise AssertionError("no MeasEpoch in fixture")


class SvidMapTest(unittest.TestCase):
    def test_ranges(self):
        self.assertEqual(s._svid_to_sv(6), "G06")
        self.assertEqual(s._svid_to_sv(71), "E01")     # GAL PRN = SVID-70
        self.assertEqual(s._svid_to_sv(154), "C14")    # BDS PRN = SVID-140
        self.assertIsNone(s._svid_to_sv(40))           # GLONASS — out of scope


class SignedTest(unittest.TestCase):
    def test_twos_complement(self):
        self.assertEqual(s._signed(5, 4), 5)
        self.assertEqual(s._signed(0b1111, 4), -1)
        self.assertEqual(s._signed((1 << 18), 19), -(1 << 18))


class Cn0Test(unittest.TestCase):
    def test_l1ca_offset(self):
        self.assertAlmostEqual(s._cn0_dbhz(154, "GPS-L1CA"), 48.5)   # +10 offset
        self.assertAlmostEqual(s._cn0_dbhz(130, "GPS-L2CL"), 32.5)   # no offset


class DecodeRealMeasEpochTest(unittest.TestCase):
    def setUp(self):
        self.msg = _load_meas_epoch()
        self.dec = s.decode_meas_epoch(self.msg)

    def test_decodes_tow_and_cells(self):
        tow, cells = self.dec
        self.assertEqual(tow, 54341000)
        # mappable+measured signals only (mosaic also tracks L1P/GLONASS/BDS-B3I
        # we don't map, and tracked-but-unmeasured slots are dropped)
        self.assertGreater(len(cells), 15)

    def test_reconstructed_values_are_physical(self):
        _tow, cells = self.dec
        for c in cells:
            self.assertTrue(1e6 < c["pr_m"] < 4e7, c)       # PR in range
            self.assertTrue(10 <= c["cno"] <= 62, c)        # incl. weak/low-elev
            self.assertIn(c["sig_name"], s._SIG_FREQ)

    def test_multi_constellation_signals_present(self):
        _tow, cells = self.dec
        sysset = {c["sv"][0] for c in cells}
        self.assertIn("G", sysset)          # GPS
        self.assertIn("E", sysset)          # Galileo
        # every mapped signal is a real engine sig_name (feeds the shared former)
        from collections import Counter
        self.assertIn("GPS-L1CA", Counter(c["sig_name"] for c in cells))

    def test_valid_carrier_consistent_with_range(self):
        # (was test_carrier_dnu_yields_no_phase — a MISNOMER: it exercised the
        # VALID-carrier path, never the DNU sentinel.  The real DNU case is
        # pinned in ReferenceDecodeValuePinTest.)  A valid carrier gives
        # cp_cyc ≈ pr/λ (within a few thousand cycles).
        _tow, cells = self.dec
        for c in cells:
            if c["cp_cyc"] is not None:
                lam = 299792458.0 / c["freq_hz"]
                self.assertLess(abs(c["cp_cyc"] - c["pr_m"] / lam), 1e5, c)

    def test_forms_if_obs_through_shared_former(self):
        from realtime_ppp import raw_obs_to_if_observations
        from peppar_fix.rtcm_msm_obs import default_sig_lookup
        raw = s.meas_epoch_to_raw_obs(self.msg, default_sig_lookup({"gps", "gal"}))
        obs, _r, _no, _ns = raw_obs_to_if_observations(raw, {"gps", "gal"}, None)
        self.assertGreaterEqual(len(obs), 1)
        o = obs[0]
        self.assertIn(o["sys"], ("gps", "gal"))
        self.assertTrue(1e6 < o["pr_if"] < 4e7)
        self.assertIsNotNone(o["phi_if_m"])

    def test_non_measepoch_returns_none(self):
        from types import SimpleNamespace
        self.assertIsNone(s.decode_meas_epoch(SimpleNamespace(identity="PVTGeodetic")))


# Ground truth: mosaic_measepoch.sbf decoded by RTKLIB demo5 convbin 2.5.1
# (`convbin -r sbf -v 3.04`), first epoch TOW=54341000 (2026-07-05 15:05:41 GPS).
# RINEX obs codes → engine sig_name: GPS C1C/C2W/C2L = L1CA/L2W/L2CL;
# GAL C1C/C7Q/C5Q = E1C/E5bQ/E5aQ; BDS C2I = B1I.  Each value is
# (pseudorange_m, carrier_phase_cyc) — carrier None where RINEX has no phase
# (the receiver flagged Do-Not-Use, CarrierMSB=-128).  An independent decoder
# pinning the two subtle reconstructions the range/structure tests can't:
# the Type2 code-offset sign (the L2 half of the IF combo) and the fine
# carrier offset (the precision observable).  I-110210 (delta #287 review).
_REF_DECODE = {
    ("G06", "GPS-L1CA"): (21874273.281, 114950107.025),
    ("G06", "GPS-L2W"):  (21874275.333, 89571517.435),
    ("G11", "GPS-L2CL"): (21190399.120, 86771177.580),
    # E08 is decoder channel index i=10 — its Type2 signals (E1C, E5aQ) are
    # exactly what a transposed Type2 sub-block accessor (_j_i vs _i_j) drops
    # or cross-contaminates.  These three are the transposition regression pin.
    ("E08", "GAL-E1C"):  (25839086.724, 135785366.348),
    ("E08", "GAL-E5bQ"): (25839086.668, 104043365.299),
    ("E08", "GAL-E5aQ"): (25839089.098, 101398209.980),
    ("C14", "BDS-B1I"):  (23493984.297, 122339409.977),
    # G29 L1CA: valid pseudorange but the receiver flagged the carrier DNU —
    # RINEX has no L1C phase, so cp_cyc must be None (the real DNU-path pin).
    ("G29", "GPS-L1CA"): (24959135.824, None),
}


class ReferenceDecodeValuePinTest(unittest.TestCase):
    """Pin the SBF decode to RTKLIB ground truth (values, not just structure).

    Asserts specific (SV, signal) → PR/CP against an INDEPENDENT decoder so a
    sign/scale error (tens of m PR, thousands of cyc) that
    test_reconstructed_values_are_physical would wave through is caught.
    """

    def setUp(self):
        _tow, cells = s.decode_meas_epoch(_load_meas_epoch())
        self.by_key = {(c["sv"], c["sig_name"]): c for c in cells}

    def test_pseudorange_and_carrier_match_rtklib(self):
        for key, (pr, cp) in _REF_DECODE.items():
            with self.subTest(key=key):
                c = self.by_key.get(key)
                self.assertIsNotNone(c, f"{key} missing from decode")
                # PR: RINEX prints 3 decimals (mm); decode must match to <5 mm.
                self.assertAlmostEqual(c["pr_m"], pr, delta=5e-3)
                if cp is None:
                    self.assertIsNone(c["cp_cyc"],
                                      f"{key} carrier is DNU → cp must be None")
                else:
                    # CP: <0.01 cyc (RINEX 3-decimal rounding ~ 0.5 mcyc).
                    self.assertIsNotNone(c["cp_cyc"])
                    self.assertAlmostEqual(c["cp_cyc"], cp, delta=1e-2)

    def test_dnu_carrier_yields_none(self):
        # explicit DNU-path assertion (the misnamed old test never hit it):
        # G29 L1CA has a valid PR but CarrierMSB == -128.
        c = self.by_key[("G29", "GPS-L1CA")]
        self.assertGreater(c["pr_m"], 1e6)      # PR still decoded
        self.assertIsNone(c["cp_cyc"])          # carrier suppressed


if __name__ == "__main__":
    unittest.main()

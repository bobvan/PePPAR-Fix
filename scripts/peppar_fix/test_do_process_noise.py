"""Tests for derive_do_process_noise (doProcessNoiseFromChar-main)."""

from __future__ import annotations

import math
import unittest

from peppar_fix.do_state import derive_do_process_noise


class _Basics(unittest.TestCase):

    def test_none_input(self):
        self.assertIsNone(derive_do_process_noise(None))

    def test_non_dict_input(self):
        self.assertIsNone(derive_do_process_noise("not a dict"))
        self.assertIsNone(derive_do_process_noise(42))

    def test_empty_dict_returns_empty(self):
        self.assertEqual(derive_do_process_noise({}), {})

    def test_missing_sources(self):
        # Char dict with no 'sources' key — return empty (not None)
        self.assertEqual(derive_do_process_noise({"other": "stuff"}), {})


class _PhaseExtraction(unittest.TestCase):

    def test_carrier_preferred_when_present(self):
        char = {
            "sources": {
                "Carrier": {"units": "ns", "asd_at_0.1Hz": 0.085},
                "PPS":     {"units": "ns", "asd_at_0.1Hz": 0.220},
                "PPS+qErr":{"units": "ns", "asd_at_0.1Hz": 0.180},
            },
        }
        out = derive_do_process_noise(char)
        self.assertEqual(out["sigma_do_phase_ns"], 0.085)
        self.assertEqual(out["sigma_do_phase_source"], "Carrier")

    def test_pps_fallback_when_no_carrier(self):
        char = {
            "sources": {
                "PPS": {"units": "ns", "asd_at_0.1Hz": 0.220},
            },
        }
        out = derive_do_process_noise(char)
        self.assertEqual(out["sigma_do_phase_ns"], 0.220)
        self.assertEqual(out["sigma_do_phase_source"], "PPS")

    def test_pps_qerr_fallback_when_no_pps(self):
        char = {
            "sources": {
                "PPS+qErr": {"units": "ns", "asd_at_0.1Hz": 0.180},
            },
        }
        out = derive_do_process_noise(char)
        self.assertEqual(out["sigma_do_phase_ns"], 0.180)
        self.assertEqual(out["sigma_do_phase_source"], "PPS+qErr")

    def test_freerun_char_rb_key_recognized(self):
        """The key do_freerun_char.py actually writes must be read.
        Regression for the silent key-mismatch (derive returned {}
        for every real freerun characterization)."""
        char = {
            "sources": {
                "DO PPS (chA vs TICC Rb)": {
                    "units": "ns", "asd_at_0.1Hz": 0.0538},
            },
        }
        out = derive_do_process_noise(char)
        self.assertAlmostEqual(out["sigma_do_phase_ns"], 0.0538)
        self.assertEqual(out["sigma_do_phase_source"],
                         "DO PPS (chA vs TICC Rb)")

    def test_freerun_char_chacb_key_recognized(self):
        char = {
            "sources": {
                "DO PPS (chA-chB)": {"units": "ns", "asd_at_0.1Hz": 0.033},
            },
        }
        out = derive_do_process_noise(char)
        self.assertAlmostEqual(out["sigma_do_phase_ns"], 0.033)
        self.assertEqual(out["sigma_do_phase_source"], "DO PPS (chA-chB)")

    def test_carrier_preferred_over_rb(self):
        """Carrier (cleanest) still wins over the Rb-referenced key."""
        char = {
            "sources": {
                "Carrier": {"units": "ns", "asd_at_0.1Hz": 0.085},
                "DO PPS (chA vs TICC Rb)": {
                    "units": "ns", "asd_at_0.1Hz": 0.0538},
            },
        }
        out = derive_do_process_noise(char)
        self.assertEqual(out["sigma_do_phase_source"], "Carrier")

    def test_rb_preferred_over_chacb(self):
        """Rb-referenced (sawtooth-free) beats chA-chB (GNSS-ref)."""
        char = {
            "sources": {
                "DO PPS (chA-chB)": {"units": "ns", "asd_at_0.1Hz": 0.033},
                "DO PPS (chA vs TICC Rb)": {
                    "units": "ns", "asd_at_0.1Hz": 0.0538},
            },
        }
        out = derive_do_process_noise(char)
        self.assertEqual(out["sigma_do_phase_source"],
                         "DO PPS (chA vs TICC Rb)")

    def test_wrong_units_skipped(self):
        # PPS with units=ppb (wrong) should be skipped
        char = {
            "sources": {
                "PPS": {"units": "ppb", "asd_at_0.1Hz": 0.220},
            },
        }
        out = derive_do_process_noise(char)
        self.assertNotIn("sigma_do_phase_ns", out)

    def test_zero_or_negative_skipped(self):
        char = {
            "sources": {
                "Carrier": {"units": "ns", "asd_at_0.1Hz": 0.0},
                "PPS":     {"units": "ns", "asd_at_0.1Hz": -1.0},
                "PPS+qErr":{"units": "ns", "asd_at_0.1Hz": 0.100},
            },
        }
        out = derive_do_process_noise(char)
        # Falls through to PPS+qErr since Carrier=0 and PPS<0 are skipped
        self.assertEqual(out["sigma_do_phase_ns"], 0.100)
        self.assertEqual(out["sigma_do_phase_source"], "PPS+qErr")


class _FreqExtraction(unittest.TestCase):

    def test_freq_command_used_when_present(self):
        # Hardware-neutral actuator-command key (misnomers.md 2026-05-29).
        char = {
            "sources": {
                "freq_command": {"units": "ppb", "asd_at_0.1Hz": 0.005},
            },
        }
        out = derive_do_process_noise(char)
        self.assertEqual(out["sigma_do_freq_ppb"], 0.005)
        self.assertEqual(out["sigma_do_freq_source"], "freq_command")

    def test_adjfine_legacy_alias_still_read(self):
        # Back-compat: old characterizations keyed "adjfine".
        char = {
            "sources": {
                "adjfine": {"units": "ppb", "asd_at_0.1Hz": 0.005},
            },
        }
        out = derive_do_process_noise(char)
        self.assertEqual(out["sigma_do_freq_ppb"], 0.005)
        self.assertEqual(out["sigma_do_freq_source"], "adjfine")

    def test_freq_command_preferred_over_adjfine(self):
        char = {
            "sources": {
                "freq_command": {"units": "ppb", "asd_at_0.1Hz": 0.003},
                "adjfine": {"units": "ppb", "asd_at_0.1Hz": 0.009},
            },
        }
        out = derive_do_process_noise(char)
        self.assertEqual(out["sigma_do_freq_ppb"], 0.003)
        self.assertEqual(out["sigma_do_freq_source"], "freq_command")

    def test_adjfine_wrong_units(self):
        char = {
            "sources": {
                "adjfine": {"units": "ns", "asd_at_0.1Hz": 0.005},
            },
        }
        out = derive_do_process_noise(char)
        self.assertNotIn("sigma_do_freq_ppb", out)

    def test_no_adjfine(self):
        char = {
            "sources": {
                "PPS": {"units": "ns", "asd_at_0.1Hz": 0.220},
            },
        }
        out = derive_do_process_noise(char)
        # Got phase but not freq
        self.assertIn("sigma_do_phase_ns", out)
        self.assertNotIn("sigma_do_freq_ppb", out)


class _RealisticInput(unittest.TestCase):

    def test_piface_freerun_2026_05_07_pattern(self):
        # Matches schema from project_freerun_floors_2026_05_07 memory:
        # PiFace OCXO+DAC freerun (Rb-ref) numbers.  ASD@0.1Hz values
        # synthesised from those TDEV numbers as approximations.
        char = {
            "do_label": "ocxo-piface",
            "host": "PiFace",
            "captured": "2026-05-11T15:17:00Z",
            "sources": {
                "PPS":     {"units": "ns", "asd_at_0.1Hz": 0.085,
                            "slope": -1.70, "noise_type": "white_FM"},
                "Carrier": {"units": "ns", "asd_at_0.1Hz": 0.075,
                            "slope": -1.70, "noise_type": "white_FM"},
                "PPS+qErr":{"units": "ns", "asd_at_0.1Hz": 0.082,
                            "slope": -1.83, "noise_type": "white_FM"},
                "adjfine": {"units": "ppb", "asd_at_0.1Hz": 0.005,
                            "slope": -1.81, "noise_type": "white_FM"},
                "qerr (TIM-TP)": {"units": "ns", "asd_at_0.1Hz": 2.92,
                                  "slope": -0.05, "noise_type": "white_phase"},
            },
        }
        out = derive_do_process_noise(char)
        # Carrier preferred over PPS
        self.assertEqual(out["sigma_do_phase_ns"], 0.075)
        self.assertEqual(out["sigma_do_phase_source"], "Carrier")
        # adjfine gives freq
        self.assertEqual(out["sigma_do_freq_ppb"], 0.005)
        self.assertEqual(out["sigma_do_freq_source"], "adjfine")


class _FreqFromAdev(unittest.TestCase):
    """Q[3,3] (sigma_do_freq) from the rising-ADEV (RWFM) tail when a
    freerun char has no adjfine source — the qFromCharPerActuator gap.

    RWFM: σ_y²(τ)=q·τ/3 ⇒ sigma_do_freq_ppb = √3·ADEV(τ)·1e9/√τ,
    evaluated at the largest τ.
    """

    def test_freq_from_adev_rwfm_formula(self):
        char = {"sources": {
            "DO PPS (chA vs TICC Rb)": {
                "units": "ns", "asd_at_0.1Hz": 0.054,
                "adev_by_tau_s": {"100.0": 1e-11}}}}
        out = derive_do_process_noise(char)
        self.assertAlmostEqual(out["sigma_do_freq_ppb"],
                               math.sqrt(3.0) * 1e-3, places=9)
        self.assertIn("adev@100", out["sigma_do_freq_source"])
        self.assertIn("RWFM", out["sigma_do_freq_source"])

    def test_adev_uses_largest_tau(self):
        char = {"sources": {
            "DO PPS (chA vs TICC Rb)": {
                "units": "ns", "asd_at_0.1Hz": 0.054,
                "adev_by_tau_s": {"1.0": 2e-11, "100.0": 5e-12,
                                  "1024.0": 3e-11}}}}
        out = derive_do_process_noise(char)
        self.assertAlmostEqual(
            out["sigma_do_freq_ppb"],
            math.sqrt(3.0) * 3e-11 * 1e9 / math.sqrt(1024.0), places=9)
        self.assertIn("1024", out["sigma_do_freq_source"])

    def test_adjfine_preferred_over_adev(self):
        char = {"sources": {
            "adjfine": {"units": "ppb", "asd_at_0.1Hz": 0.005},
            "DO PPS (chA vs TICC Rb)": {
                "units": "ns", "asd_at_0.1Hz": 0.054,
                "adev_by_tau_s": {"1024.0": 3e-11}}}}
        out = derive_do_process_noise(char)
        self.assertEqual(out["sigma_do_freq_ppb"], 0.005)
        self.assertEqual(out["sigma_do_freq_source"], "adjfine")

    def test_adev_skips_nonfinite_and_nonpositive(self):
        char = {"sources": {
            "DO PPS (chA vs TICC Rb)": {
                "units": "ns", "asd_at_0.1Hz": 0.054,
                "adev_by_tau_s": {"1.0": 1e-11, "10.0": 0.0,
                                  "100.0": -1.0, "50.0": "nan"}}}}
        out = derive_do_process_noise(char)
        # Only τ=1.0 is valid → uses it.
        self.assertAlmostEqual(out["sigma_do_freq_ppb"],
                               math.sqrt(3.0) * 1e-11 * 1e9, places=9)
        self.assertIn("adev@1", out["sigma_do_freq_source"])

    def test_no_adev_no_adjfine_no_freq(self):
        char = {"sources": {
            "DO PPS (chA vs TICC Rb)": {
                "units": "ns", "asd_at_0.1Hz": 0.054}}}
        out = derive_do_process_noise(char)
        self.assertIn("sigma_do_phase_ns", out)
        self.assertNotIn("sigma_do_freq_ppb", out)

    def test_adev_from_non_chosen_phase_source(self):
        # Carrier wins phase but has no ADEV; pull ADEV from the
        # Rb-referenced source that does.
        char = {"sources": {
            "Carrier": {"units": "ns", "asd_at_0.1Hz": 0.075},
            "DO PPS (chA vs TICC Rb)": {
                "units": "ns", "asd_at_0.1Hz": 0.054,
                "adev_by_tau_s": {"512.0": 2e-11}}}}
        out = derive_do_process_noise(char)
        self.assertEqual(out["sigma_do_phase_source"], "Carrier")
        self.assertAlmostEqual(
            out["sigma_do_freq_ppb"],
            math.sqrt(3.0) * 2e-11 * 1e9 / math.sqrt(512.0), places=9)

    def test_realistic_freerun_char_schema(self):
        # Matches do_freerun_char.py output: one source with
        # asd_at_0.1Hz + adev_by_tau_s, no adjfine.
        char = {"do_label": "ocxo-33-madhat", "sources": {
            "DO PPS (chA vs TICC Rb)": {
                "units": "ns", "asd_at_0.1Hz": 0.0538,
                "slope": -2.1, "noise_type": "white_FM",
                "adev_by_tau_s": {"1.0": 4.6e-11, "10.0": 1.5e-11,
                                  "100.0": 8e-12, "1024.0": 2.5e-11}}}}
        out = derive_do_process_noise(char)
        self.assertEqual(out["sigma_do_phase_ns"], 0.0538)
        self.assertEqual(out["sigma_do_phase_source"],
                         "DO PPS (chA vs TICC Rb)")
        self.assertAlmostEqual(
            out["sigma_do_freq_ppb"],
            math.sqrt(3.0) * 2.5e-11 * 1e9 / math.sqrt(1024.0), places=9)


if __name__ == "__main__":
    unittest.main()

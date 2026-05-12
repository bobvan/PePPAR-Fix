"""Tests for derive_do_process_noise (doProcessNoiseFromChar-main)."""

from __future__ import annotations

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

    def test_adjfine_used_when_present(self):
        char = {
            "sources": {
                "adjfine": {"units": "ppb", "asd_at_0.1Hz": 0.005},
            },
        }
        out = derive_do_process_noise(char)
        self.assertEqual(out["sigma_do_freq_ppb"], 0.005)
        self.assertEqual(out["sigma_do_freq_source"], "adjfine")

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


if __name__ == "__main__":
    unittest.main()

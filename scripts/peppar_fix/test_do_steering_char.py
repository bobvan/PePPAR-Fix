"""Tests for do_steering_char: detect_linear_region fit → [steering] schema
mapping, and the end-to-end write through do_schema.update_characterization_section.

The hardware sweep (DAC writes + TICC measurement) is not unit-tested — it needs
I2C + a TICC — same as dac_slope_cal.  These cover the pure mapping + the schema
write contract, using dac_slope_cal's MadHat fixture-shaped fit.
"""
from __future__ import annotations

import os
import shutil
import sys
import tempfile
import textwrap
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from do_steering_char import steering_fields_from_fit  # noqa: E402
from peppar_fix import do_schema  # noqa: E402
from peppar_fix.do_schema import SchemaError, load_do_characterization  # noqa: E402


# A detect_linear_region-shaped result (matches MadHat's measured curve:
# slope ≈ -0.0263 ppb/code, GPS-rate near mid-scale).  ppb_at_code_min is
# the line evaluated at code_min: -7.0 + (-0.0263)·(8000 − 32768) = +644.4.
_FIT = {"slope": -0.0263, "intercept": -7.0, "rmse": 0.5,
        "code_min": 8000, "code_max": 60000, "n_linear": 11,
        "ppb_at_code_min": -7.0 + (-0.0263) * (8000 - 32768)}

_IDENTITY_ONLY = textwrap.dedent("""\
    schema_version = "1"

    [identity]
    do_uid = "ocxo-test"
    model = "Isotemp OCXO131-100"
    class = "OCXO"
    actuator_type = "DAC"
    nominal_freq_hz = 10000000
    """)


class _Tmp:
    def __enter__(self):
        self._d = tempfile.mkdtemp(prefix="do-steer-test-")
        return self._d

    def __exit__(self, *exc):
        shutil.rmtree(self._d, ignore_errors=True)


class TestSteeringFieldsFromFit(unittest.TestCase):
    def test_mapping_and_types(self):
        f = steering_fields_from_fit(_FIT, 32768)
        self.assertEqual(f["slope_ppb_per_code"], -0.0263)
        self.assertEqual(f["intercept_ppb_at_parked"], -7.0)
        self.assertEqual(f["parked_code"], 32768)
        self.assertIsInstance(f["parked_code"], int)
        self.assertEqual(f["code_min"], 8000)
        self.assertIsInstance(f["code_min"], int)
        self.assertEqual(f["code_max"], 60000)
        self.assertEqual(f["asymmetry_factor"], 1.0)
        self.assertEqual(f["rmse_ppb"], 0.5)
        self.assertIn("measured_at", f)
        # noMagicCenterCode: the edge anchor is emitted (canonical going fwd).
        self.assertAlmostEqual(f["ppb_at_code_min"],
                               -7.0 + (-0.0263) * (8000 - 32768))
        # source is the WRITER's job, not the mapper's.
        self.assertNotIn("source", f)

    def test_records_char_temps_when_provided(self):
        f = steering_fields_from_fit(
            _FIT, 32768, dac_gain=1,
            temps={"char_temp_ocxo_c": 41.2, "char_temp_cpu_c": 58.7})
        self.assertEqual(f["char_temp_ocxo_c"], 41.2)
        self.assertEqual(f["char_temp_cpu_c"], 58.7)
        self.assertEqual(f["dac_gain"], 1)

    def test_no_temps_when_absent(self):
        # No sensors → no temp keys (honest absence, not a fake 0).
        f = steering_fields_from_fit(_FIT, 32768)
        self.assertNotIn("char_temp_ocxo_c", f)
        self.assertNotIn("char_temp_cpu_c", f)

    def test_edge_anchor_round_trips_through_schema(self):
        # ppb_at_code_min + char temps survive the writer→loader round trip.
        with _Tmp() as d:
            with open(os.path.join(d, "ocxo-test.toml"), "w") as fh:
                fh.write(_IDENTITY_ONLY)
            fields = steering_fields_from_fit(
                _FIT, 32768, temps={"char_temp_cpu_c": 55.0})
            do_schema.update_characterization_section(
                "ocxo-test", "steering", fields, dos_dir=d)
            c = load_do_characterization("ocxo-test", dos_dir=d)
            self.assertAlmostEqual(c.steering["ppb_at_code_min"],
                                   -7.0 + (-0.0263) * (8000 - 32768))
            self.assertAlmostEqual(c.steering["char_temp_cpu_c"], 55.0)

    def test_end_to_end_write_and_load(self):
        # fit → fields → schema writer → loader sees measured steering.
        with _Tmp() as d:
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, "ocxo-test.toml"), "w") as fh:
                fh.write(_IDENTITY_ONLY)
            fields = steering_fields_from_fit(_FIT, 32768)
            do_schema.update_characterization_section(
                "ocxo-test", "steering", fields, dos_dir=d)
            c = load_do_characterization("ocxo-test", dos_dir=d)
            self.assertEqual(c.provenance["steering"], "measured")
            self.assertAlmostEqual(c.steering["slope_ppb_per_code"], -0.0263)
            self.assertEqual(c.steering["code_max"], 60000)

    def test_flat_or_degenerate_fit_would_be_rejected(self):
        # detect_linear_region never returns slope 0 (it returns None), but if
        # a caller forced a 0 slope through, the writer's validate must reject.
        with _Tmp() as d:
            with open(os.path.join(d, "ocxo-test.toml"), "w") as fh:
                fh.write(_IDENTITY_ONLY)
            bad = steering_fields_from_fit({**_FIT, "slope": 0.0}, 32768)
            with self.assertRaises(SchemaError):
                do_schema.update_characterization_section(
                    "ocxo-test", "steering", bad, dos_dir=d)


if __name__ == "__main__":
    unittest.main()

"""Tests for clockmatrix_combo_gain_char + the combo-gain schema/resolve wiring.

The hardware sweep (combo writes + PHASE_STATUS sampling) is not unit-tested —
it needs I2C on a ClockMatrix host.  These cover the pure fit math, the field
mapping, the [steering].combo_gain schema round-trip, and the engine's
refuse-to-actuate gate for a ClockMatrix_combo DO.
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

from clockmatrix_combo_gain_char import (  # noqa: E402
    ramp_to_realized_ppb, fit_phase_slope, fit_combo_gain, combo_steering_fields,
)
from peppar_fix import do_schema, do_char_resolve  # noqa: E402
from peppar_fix.do_schema import SchemaError, load_do_characterization  # noqa: E402


_COMBO_IDENTITY = textwrap.dedent("""\
    schema_version = "1"

    [identity]
    do_uid = "otc-test"
    model = "Timebeat OTC OCXO (8A34002 combo)"
    class = "OCXO"
    actuator_type = "ClockMatrix_combo"
    nominal_freq_hz = 10000000
    """)


class _Tmp:
    def __enter__(self):
        self._d = tempfile.mkdtemp(prefix="combo-gain-test-")
        return self._d

    def __exit__(self, *exc):
        shutil.rmtree(self._d, ignore_errors=True)


def _register_combo_do(d, uid="otc-test"):
    with open(os.path.join(d, f"{uid}.toml"), "w") as fh:
        fh.write(_COMBO_IDENTITY)


class TestFitMath(unittest.TestCase):

    def test_ramp_sign(self):
        # +combo => DO faster => phase ramps negative; realized = -slope.
        self.assertAlmostEqual(ramp_to_realized_ppb(-137.0), 137.0)
        self.assertAlmostEqual(ramp_to_realized_ppb(262.0), -262.0)

    def test_phase_slope(self):
        t = [0, 1, 2, 3, 4]
        p = [0, -10, -20, -30, -40]  # -10 ns/s
        self.assertAlmostEqual(fit_phase_slope(t, p), -10.0)

    def test_phase_slope_rejects_glitch_reads(self):
        # PHASE_STATUS glitch reads return ~0 mid-ramp; Theil-Sen must ignore
        # them where least-squares would be dragged toward 0.  True -200 ns/s.
        t = list(range(11))
        p = [-200.0 * k for k in t]
        p[3] = 0.0   # glitch
        p[7] = 1.8   # glitch
        self.assertAlmostEqual(fit_phase_slope(t, p), -200.0, places=6)

    def test_phase_slope_needs_two_points(self):
        with self.assertRaises(ValueError):
            fit_phase_slope([0], [0])

    def test_combo_gain_recovers_slope_and_offset(self):
        # realized = 0.7*naive + 5 (5 ppb OCXO free-run offset).
        pts = [(n, 0.7 * n + 5.0) for n in (-300, -200, -100, 100, 200, 300)]
        fit = fit_combo_gain(pts)
        self.assertAlmostEqual(fit["combo_gain"], 0.7, places=6)
        self.assertAlmostEqual(fit["freerun_offset_ppb"], 5.0, places=6)
        self.assertAlmostEqual(fit["r2"], 1.0, places=6)
        self.assertEqual(fit["n"], 6)

    def test_combo_gain_offset_does_not_bias_gain(self):
        # A large free-run offset must NOT leak into the gain (that's the whole
        # point of fitting an intercept rather than going through the origin).
        pts = [(n, 0.685 * n + 250.0) for n in (-300, 100, 300)]
        fit = fit_combo_gain(pts)
        self.assertAlmostEqual(fit["combo_gain"], 0.685, places=6)

    def test_combo_gain_needs_two_points(self):
        with self.assertRaises(ValueError):
            fit_combo_gain([(100.0, 70.0)])


class TestComboSteeringFields(unittest.TestCase):

    def test_mapping_and_types(self):
        fit = {"combo_gain": 0.7, "freerun_offset_ppb": 5.0,
               "rmse_ppb": 0.4, "r2": 0.999, "n": 6}
        f = combo_steering_fields(fit, [-200, 200], measure_s=30, settle_s=5)
        self.assertEqual(f["combo_gain"], 0.7)
        self.assertEqual(f["combo_freerun_offset_ppb"], 5.0)
        self.assertEqual(f["n_points"], 6)
        self.assertEqual(f["naive_steps_ppb"], "-200,200")
        self.assertEqual(f["measure_s"], 30)
        self.assertIn("measured_at", f)
        self.assertNotIn("source", f)  # source is the writer's job

    def test_records_temps_when_provided(self):
        fit = {"combo_gain": 0.7, "freerun_offset_ppb": 5.0,
               "rmse_ppb": 0.4, "r2": 0.999, "n": 6}
        f = combo_steering_fields(fit, [200], 30, 5,
                                  temps={"char_temp_cpu_c": 52.1})
        self.assertEqual(f["char_temp_cpu_c"], 52.1)


class TestSchemaRoundTrip(unittest.TestCase):

    def test_write_and_load_measured_combo_gain(self):
        with _Tmp() as d:
            _register_combo_do(d)
            fit = {"combo_gain": 0.685, "freerun_offset_ppb": 3.0,
                   "rmse_ppb": 0.5, "r2": 0.998, "n": 6}
            fields = combo_steering_fields(fit, [-200, 200], 30, 5)
            do_schema.update_characterization_section(
                "otc-test", "steering", fields, dos_dir=d)
            c = load_do_characterization("otc-test", dos_dir=d)
            self.assertEqual(c.provenance["steering"], "measured")
            self.assertAlmostEqual(c.steering["combo_gain"], 0.685)

    def test_combo_actuator_type_is_allowed(self):
        self.assertIn("ClockMatrix_combo", do_schema.ALLOWED_ACTUATOR_TYPES)

    def test_measured_steering_without_combo_gain_rejected(self):
        with _Tmp() as d:
            _register_combo_do(d)
            # A measured steering section missing combo_gain must fail (the
            # actuator-aware required-key check).
            with self.assertRaises(SchemaError):
                do_schema.update_characterization_section(
                    "otc-test", "steering",
                    {"rmse_ppb": 0.5, "r2": 0.99}, dos_dir=d)

    def test_zero_combo_gain_rejected(self):
        with _Tmp() as d:
            _register_combo_do(d)
            with self.assertRaises(SchemaError):
                do_schema.update_characterization_section(
                    "otc-test", "steering", {"combo_gain": 0.0}, dos_dir=d)


class TestRefuseGate(unittest.TestCase):

    def test_refuses_combo_without_measured_steering(self):
        with _Tmp() as d:
            _register_combo_do(d)  # identity only, no [steering]
            ec = do_char_resolve.resolve_engine_characterization(
                "otc-test", dos_dir=d)
            self.assertEqual(ec.actuator_type, "ClockMatrix_combo")
            self.assertTrue(do_char_resolve.should_refuse_for_steering(
                ec, has_servo=True))
            # And the construction-time resolver raises rather than defaulting.
            with self.assertRaises(SchemaError):
                do_char_resolve.resolve_combo_actuator_params("otc-test", dos_dir=d)

    def test_accepts_combo_with_measured_steering(self):
        with _Tmp() as d:
            _register_combo_do(d)
            fields = combo_steering_fields(
                {"combo_gain": 0.7, "freerun_offset_ppb": 0.0,
                 "rmse_ppb": 0.3, "r2": 0.999, "n": 6}, [200], 30, 5)
            do_schema.update_characterization_section(
                "otc-test", "steering", fields, dos_dir=d)
            ec = do_char_resolve.resolve_engine_characterization(
                "otc-test", dos_dir=d)
            self.assertFalse(do_char_resolve.should_refuse_for_steering(
                ec, has_servo=True))
            params = do_char_resolve.resolve_combo_actuator_params(
                "otc-test", dos_dir=d)
            self.assertAlmostEqual(params["combo_gain"], 0.7)

    def test_combo_resolver_defers_for_unregistered_do(self):
        # An unregistered DO (no .toml) must DEFER (return None), not raise —
        # so existing FCW/PHC ClockMatrix hosts that predate the schema keep
        # working.  (The engine still fails later via resolve_engine_char.)
        with _Tmp() as d:
            self.assertIsNone(
                do_char_resolve.resolve_combo_actuator_params(
                    "never-registered", dos_dir=d))

    def test_combo_resolver_returns_none_for_non_combo(self):
        # A DAC DO must not be sourced through the combo resolver.
        with _Tmp() as d:
            with open(os.path.join(d, "dac-test.toml"), "w") as fh:
                fh.write(_COMBO_IDENTITY.replace("otc-test", "dac-test")
                         .replace("ClockMatrix_combo", "DAC"))
            self.assertIsNone(
                do_char_resolve.resolve_combo_actuator_params("dac-test", dos_dir=d))

    def test_refuse_gate_exempts_clockmatrix_fcw(self):
        # FCW gain is an intrinsic constant — the gate must NOT fire on it.
        with _Tmp() as d:
            with open(os.path.join(d, "fcw-test.toml"), "w") as fh:
                fh.write(_COMBO_IDENTITY.replace("otc-test", "fcw-test")
                         .replace("ClockMatrix_combo", "ClockMatrix_FCW"))
            ec = do_char_resolve.resolve_engine_characterization(
                "fcw-test", dos_dir=d)
            self.assertFalse(do_char_resolve.should_refuse_for_steering(
                ec, has_servo=True))


if __name__ == "__main__":
    unittest.main()

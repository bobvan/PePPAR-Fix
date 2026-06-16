"""Tests for peppar_fix.do_schema.

Covers schema round-trip + every refusal case the design doc requires:

  - schema_version mismatch / missing  → refuse to load
  - missing [identity]                 → refuse to load
  - unknown DO class                   → refuse to load
  - receiver-side measurement_channel  → refuse to load
  - control-loop measurement_channel   → refuse to load
  - coast_tdev_slope ≤ 0               → refuse to load (fail-loud,
                                          would silently disable
                                          Goldilocks at runtime)
  - negative or zero sigma values      → refuse to load
  - measured steering with slope == 0  → refuse to load
  - class defaults present, correct shape, looser than measured units
  - runtime save round-trips
  - structural firewall: runtime save never touches characterization
"""
from __future__ import annotations

import os
import tempfile
import textwrap
import unittest

from peppar_fix import do_schema
from peppar_fix.do_schema import (
    ALLOWED_CLASSES,
    ALLOWED_MEASUREMENT_CHANNELS,
    CLASS_DEFAULTS,
    Characterization,
    RuntimeState,
    SCHEMA_VERSION,
    SchemaError,
    class_defaults,
    load_do_characterization,
    load_runtime_state,
    save_runtime_state,
    validate_characterization,
)


# A valid, fully-measured characterization payload — used as the
# baseline that individual tests mutate to trigger specific
# rejection cases.  Mirrors clkPoC3's actual measured values so
# the example doc-page and the test fixture stay close.
GOOD_CHAR_TOML = textwrap.dedent("""\
    schema_version = "1"

    [identity]
    do_uid = "ocxo-test"
    model = "Isotemp OCXO131-100"
    class = "OCXO"
    actuator_type = "DAC"
    dac_bits = 16
    nominal_freq_hz = 10000000
    registered = "2026-05-30T14:40:06Z"

    [steering]
    source = "measured"
    slope_ppb_per_code = 0.02569
    intercept_ppb_at_parked = 144.65
    parked_code = 32768
    code_min = 1024
    code_max = 64512
    asymmetry_factor = 1.0
    measured_at = "2026-05-30T14:40:06Z"

    [freerun_noise]
    source = "measured"
    measurement_channel = "DO PPS (chA vs TICC Rb)"
    sigma_do_phase_ns = 0.0425
    sigma_do_freq_ppb = 0.000382
    coast_tdev_ref_ns = 0.0425
    coast_tdev_slope = 0.530
    captured = "2026-05-30T14:40:06Z"
    duration_s = 3604

    [actuation_noise]
    source = "measured"
    sigma_q_ns = 0.05
    write_settle_ms = 1
    i2c_error_rate_per_million = 0
    measured_at = "2026-06-01T00:00:00Z"

    [aging]
    drift_ppb_per_year = 0.5
    last_cal_date = "2026-05-30"
    """)


class _TempDir:
    """Per-test temp dir helper.  Avoids pytest tmp_path coupling."""

    def __init__(self):
        self._td = None

    def __enter__(self):
        self._td = tempfile.mkdtemp(prefix="do-schema-test-")
        return self._td

    def __exit__(self, *exc):
        import shutil
        shutil.rmtree(self._td, ignore_errors=True)


def _write_char(dos_dir: str, uid: str, toml_text: str) -> str:
    """Write a characterization TOML to <dos_dir>/<uid>.toml."""
    os.makedirs(dos_dir, exist_ok=True)
    path = os.path.join(dos_dir, f"{uid}.toml")
    with open(path, "w") as f:
        f.write(toml_text)
    return path


class TestRoundTrip(unittest.TestCase):
    """Valid TOML loads cleanly and exposes the expected sections."""

    def test_load_good_char(self):
        with _TempDir() as td:
            _write_char(td, "ocxo-test", GOOD_CHAR_TOML)
            c = load_do_characterization("ocxo-test", dos_dir=td)
            self.assertEqual(c.do_uid, "ocxo-test")
            self.assertEqual(c.identity["class"], "OCXO")
            self.assertEqual(
                c.freerun_noise["measurement_channel"],
                "DO PPS (chA vs TICC Rb)")
            self.assertAlmostEqual(
                c.freerun_noise["sigma_do_phase_ns"], 0.0425)
            self.assertAlmostEqual(
                c.freerun_noise["coast_tdev_slope"], 0.530)
            self.assertAlmostEqual(c.actuation_noise["sigma_q_ns"], 0.05)
            self.assertEqual(c.provenance["freerun_noise"], "measured")
            self.assertEqual(c.provenance["steering"], "measured")
            self.assertEqual(c.provenance["actuation_noise"], "measured")


class TestSchemaVersion(unittest.TestCase):

    def test_missing_version(self):
        toml = GOOD_CHAR_TOML.replace(f'schema_version = "{SCHEMA_VERSION}"',
                                       "")
        with _TempDir() as td:
            _write_char(td, "ocxo-test", toml)
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("schema_version", str(cm.exception))

    def test_wrong_version(self):
        toml = GOOD_CHAR_TOML.replace(
            f'schema_version = "{SCHEMA_VERSION}"',
            'schema_version = "99"')
        with _TempDir() as td:
            _write_char(td, "ocxo-test", toml)
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("forward-compat", str(cm.exception))


class TestIdentity(unittest.TestCase):

    def test_missing_identity(self):
        toml = GOOD_CHAR_TOML.split("[steering]")[0].replace(
            "[identity]", "[bogus]")
        with _TempDir() as td:
            _write_char(td, "ocxo-test", toml + "\n[steering]\n"
                        + GOOD_CHAR_TOML.split("[steering]\n")[1])
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("[identity]", str(cm.exception))

    def test_unknown_class(self):
        toml = GOOD_CHAR_TOML.replace('class = "OCXO"',
                                       'class = "ElectricGoat"')
        with _TempDir() as td:
            _write_char(td, "ocxo-test", toml)
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("class", str(cm.exception))
            self.assertIn("ElectricGoat", str(cm.exception))


class TestFreerunChannelRejection(unittest.TestCase):
    """The PiFace-mistake-killer: rx-side and control-loop channels
    must be refused at write time."""

    def _make_with_channel(self, channel: str) -> str:
        return GOOD_CHAR_TOML.replace(
            'measurement_channel = "DO PPS (chA vs TICC Rb)"',
            f'measurement_channel = "{channel}"')

    def test_reject_carrier(self):
        with _TempDir() as td:
            _write_char(td, "ocxo-test", self._make_with_channel("Carrier"))
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("measurement_channel", str(cm.exception))
            self.assertIn("Receiver-side", str(cm.exception))

    def test_reject_dt_rx(self):
        with _TempDir() as td:
            _write_char(td, "ocxo-test",
                        self._make_with_channel("dt_rx (PPP)"))
            with self.assertRaises(SchemaError):
                load_do_characterization("ocxo-test", dos_dir=td)

    def test_reject_qerr(self):
        with _TempDir() as td:
            _write_char(td, "ocxo-test",
                        self._make_with_channel("qerr (TIM-TP)"))
            with self.assertRaises(SchemaError):
                load_do_characterization("ocxo-test", dos_dir=td)

    def test_reject_adjfine(self):
        with _TempDir() as td:
            _write_char(td, "ocxo-test", self._make_with_channel("adjfine"))
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("control-loop", str(cm.exception))

    def test_accept_cha_vs_rb(self):
        # Sanity-check the preferred channel passes (we already tested
        # this in TestRoundTrip but pin it here for symmetry).
        with _TempDir() as td:
            _write_char(td, "ocxo-test",
                        self._make_with_channel("DO PPS (chA vs TICC Rb)"))
            c = load_do_characterization("ocxo-test", dos_dir=td)
            self.assertEqual(c.freerun_noise["measurement_channel"],
                             "DO PPS (chA vs TICC Rb)")

    def test_accept_cha_chb(self):
        with _TempDir() as td:
            _write_char(td, "ocxo-test",
                        self._make_with_channel("DO PPS (chA-chB)"))
            c = load_do_characterization("ocxo-test", dos_dir=td)
            self.assertEqual(c.freerun_noise["measurement_channel"],
                             "DO PPS (chA-chB)")


class TestSlopeSignGuard(unittest.TestCase):
    """coast_tdev_slope ≤ 0 silently disables Goldilocks at runtime
    — refuse at write time."""

    def test_reject_negative_slope(self):
        # The doc's pre-round-2 example value (-0.571) was the falling
        # white-FM branch — exactly the bug we're guarding against.
        toml = GOOD_CHAR_TOML.replace("coast_tdev_slope = 0.530",
                                       "coast_tdev_slope = -0.571")
        with _TempDir() as td:
            _write_char(td, "ocxo-test", toml)
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("coast_tdev_slope", str(cm.exception))
            self.assertIn("> 0", str(cm.exception))
            self.assertIn("Goldilocks", str(cm.exception))

    def test_reject_zero_slope(self):
        toml = GOOD_CHAR_TOML.replace("coast_tdev_slope = 0.530",
                                       "coast_tdev_slope = 0.0")
        with _TempDir() as td:
            _write_char(td, "ocxo-test", toml)
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("coast_tdev_slope", str(cm.exception))


class TestSigmaPositivity(unittest.TestCase):
    """Sigma values must be > 0 — a measured 0 is a calibration
    failure, not a valid DO."""

    def test_reject_zero_phase_sigma(self):
        toml = GOOD_CHAR_TOML.replace("sigma_do_phase_ns = 0.0425",
                                       "sigma_do_phase_ns = 0.0")
        with _TempDir() as td:
            _write_char(td, "ocxo-test", toml)
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("sigma_do_phase_ns", str(cm.exception))

    def test_reject_negative_freq_sigma(self):
        toml = GOOD_CHAR_TOML.replace("sigma_do_freq_ppb = 0.000382",
                                       "sigma_do_freq_ppb = -0.001")
        with _TempDir() as td:
            _write_char(td, "ocxo-test", toml)
            with self.assertRaises(SchemaError):
                load_do_characterization("ocxo-test", dos_dir=td)

    def test_reject_zero_sigma_q(self):
        toml = GOOD_CHAR_TOML.replace("sigma_q_ns = 0.05",
                                       "sigma_q_ns = 0.0")
        with _TempDir() as td:
            _write_char(td, "ocxo-test", toml)
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("sigma_q_ns", str(cm.exception))


class TestSteeringValidation(unittest.TestCase):

    def test_reject_zero_steering_slope(self):
        toml = GOOD_CHAR_TOML.replace("slope_ppb_per_code = 0.02569",
                                       "slope_ppb_per_code = 0.0")
        with _TempDir() as td:
            _write_char(td, "ocxo-test", toml)
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("slope_ppb_per_code", str(cm.exception))


class TestClassDefaults(unittest.TestCase):

    def test_all_classes_have_defaults(self):
        for cls in ALLOWED_CLASSES:
            d = class_defaults(cls)
            for key in ("sigma_do_phase_ns", "sigma_do_freq_ppb",
                        "coast_tdev_ref_ns", "coast_tdev_slope",
                        "sigma_q_ns"):
                self.assertIn(key, d, f"{cls} missing {key}")
                self.assertGreater(d[key], 0,
                                    f"{cls}.{key} must be > 0")

    def test_class_default_slope_positive(self):
        # The same Goldilocks-disabling guard that applies to measured
        # values must apply to class defaults — a class default with a
        # non-positive slope would disable the scheduler on every
        # default-using host.
        for cls in ALLOWED_CLASSES:
            self.assertGreater(
                CLASS_DEFAULTS[cls]["coast_tdev_slope"], 0,
                f"{cls} coast_tdev_slope default must be > 0 "
                f"(else silently disables Goldilocks for default-using hosts)"
            )

    def test_tcxo_default_looser_than_timehat_measured(self):
        # Per Main's round-1 review: TimeHat measured ~2.6 ns/√s
        # freerun, so the TCXO class default must be looser.  This
        # test pins the lineage so a future tightening of the TCXO
        # default trips here and forces re-argumentation in PR review.
        self.assertGreaterEqual(
            CLASS_DEFAULTS["TCXO"]["sigma_do_phase_ns"], 2.6,
            "TCXO class default must be ≥ TimeHat measured (2.6 ns/√s)"
        )

    def test_class_default_loader_path(self):
        # Build a minimal characterization with only [identity], no
        # freerun_noise or actuation_noise.  The loader should fill
        # those from class defaults and tag provenance.
        toml = textwrap.dedent("""\
            schema_version = "1"

            [identity]
            do_uid = "ocxo-minimal"
            model = "Test"
            class = "OCXO"
            actuator_type = "DAC"
            nominal_freq_hz = 10000000

            [steering]
            source = "measured"
            slope_ppb_per_code = 0.02
            parked_code = 32768
            code_min = 0
            code_max = 65535
            """)
        with _TempDir() as td:
            _write_char(td, "ocxo-minimal", toml)
            c = load_do_characterization("ocxo-minimal", dos_dir=td)
            self.assertEqual(c.provenance["freerun_noise"],
                              "class-default[OCXO]")
            self.assertEqual(c.provenance["actuation_noise"],
                              "class-default[OCXO]")
            self.assertEqual(
                c.freerun_noise["sigma_do_phase_ns"],
                CLASS_DEFAULTS["OCXO"]["sigma_do_phase_ns"])
            self.assertEqual(
                c.actuation_noise["sigma_q_ns"],
                CLASS_DEFAULTS["OCXO"]["sigma_q_ns"])


class TestRuntimeRoundTrip(unittest.TestCase):

    def test_save_load(self):
        rs = RuntimeState(
            last_known_freq_offset_ppb=144.65,
            last_known_dac_code=32768,
            last_updated="2026-06-15T16:00:00Z",
        )
        with _TempDir() as td:
            save_runtime_state("ocxo-test", rs, dos_dir=td)
            loaded = load_runtime_state("ocxo-test", dos_dir=td)
            self.assertIsNotNone(loaded)
            self.assertAlmostEqual(
                loaded.last_known_freq_offset_ppb, 144.65)
            self.assertEqual(loaded.last_known_dac_code, 32768)
            self.assertEqual(loaded.last_updated,
                              "2026-06-15T16:00:00Z")

    def test_unknown_freq_roundtrips_as_none_not_zero(self):
        # A freq that was never measured must read back as None, not 0.0 —
        # else a DAC-code-only checkpoint would warm-start at 0.0 ppb.
        rs = RuntimeState(
            last_known_freq_offset_ppb=None,
            last_known_dac_code=40615,
            last_updated="2026-06-15T16:00:00Z",
        )
        with _TempDir() as td:
            save_runtime_state("ocxo-test", rs, dos_dir=td)
            with open(do_schema._runtime_path("ocxo-test", dos_dir=td)) as f:
                self.assertNotIn("last_known_freq_offset_ppb", f.read())
            loaded = load_runtime_state("ocxo-test", dos_dir=td)
            self.assertIsNone(loaded.last_known_freq_offset_ppb)
            self.assertEqual(loaded.last_known_dac_code, 40615)

    def test_runtime_save_does_not_touch_characterization(self):
        # The structural firewall — save_runtime_state must never
        # open <uid>.toml.  We assert this by writing the
        # characterization file with a sentinel and confirming the
        # runtime save leaves its contents bit-identical.
        rs = RuntimeState(last_known_freq_offset_ppb=42.0,
                           last_known_dac_code=12345,
                           last_updated="2026-06-15T00:00:00Z")
        with _TempDir() as td:
            char_path = _write_char(td, "ocxo-test", GOOD_CHAR_TOML)
            sentinel_bytes = open(char_path, "rb").read()
            save_runtime_state("ocxo-test", rs, dos_dir=td)
            after_bytes = open(char_path, "rb").read()
            self.assertEqual(sentinel_bytes, after_bytes,
                              "save_runtime_state must not touch <uid>.toml")

    def test_load_runtime_absent_returns_none(self):
        with _TempDir() as td:
            self.assertIsNone(load_runtime_state("nosuch", dos_dir=td))


class TestMeasuredRequiredFields(unittest.TestCase):
    """Main's MUST-FIX from round 3: a source='measured' section with a
    valid channel but missing consumed-key must be refused.  The
    class-default path builds the keys explicitly; the measured path
    must enforce the same completeness."""

    def _drop_line(self, key: str) -> str:
        # Remove the line "<key> = ..." from GOOD_CHAR_TOML.
        out = []
        for line in GOOD_CHAR_TOML.splitlines():
            if line.lstrip().startswith(f"{key} ="):
                continue
            out.append(line)
        return "\n".join(out) + "\n"

    def test_freerun_missing_sigma_phase(self):
        with _TempDir() as td:
            _write_char(td, "ocxo-test", self._drop_line("sigma_do_phase_ns"))
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("sigma_do_phase_ns", str(cm.exception))
            self.assertIn("required when source='measured'",
                          str(cm.exception))

    def test_freerun_missing_sigma_freq(self):
        with _TempDir() as td:
            _write_char(td, "ocxo-test",
                        self._drop_line("sigma_do_freq_ppb"))
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("sigma_do_freq_ppb", str(cm.exception))

    def test_freerun_missing_coast_ref(self):
        with _TempDir() as td:
            _write_char(td, "ocxo-test",
                        self._drop_line("coast_tdev_ref_ns"))
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("coast_tdev_ref_ns", str(cm.exception))

    def test_freerun_missing_coast_slope(self):
        with _TempDir() as td:
            _write_char(td, "ocxo-test",
                        self._drop_line("coast_tdev_slope"))
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("coast_tdev_slope", str(cm.exception))

    def test_actuation_missing_sigma_q(self):
        with _TempDir() as td:
            _write_char(td, "ocxo-test", self._drop_line("sigma_q_ns"))
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("sigma_q_ns", str(cm.exception))

    def test_steering_missing_slope(self):
        with _TempDir() as td:
            _write_char(td, "ocxo-test",
                        self._drop_line("slope_ppb_per_code"))
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("slope_ppb_per_code", str(cm.exception))


class TestDoUidFilenameMismatch(unittest.TestCase):
    """Main's SHOULD-FIX from round 3: the multi-file confusion
    (clkPoC3 v1/v2) that motivated this whole architecture would let
    a renamed file load silently under the wrong identity.  Fail
    loud."""

    def test_filename_mismatch_rejected(self):
        # Write a file whose contents say "ocxo-piface" but live at
        # "ocxo-test.toml".
        toml = GOOD_CHAR_TOML.replace('do_uid = "ocxo-test"',
                                       'do_uid = "ocxo-piface"')
        with _TempDir() as td:
            _write_char(td, "ocxo-test", toml)
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("do_uid", str(cm.exception))
            self.assertIn("does not match", str(cm.exception))
            self.assertIn("ocxo-piface", str(cm.exception))


class TestRuntimeSchemaVersion(unittest.TestCase):
    """Main's nit 1 from round 3: runtime loader should not silently
    load an unknown version.  Loads return None (cold start)."""

    def test_runtime_wrong_version_returns_none(self):
        with _TempDir() as td:
            path = os.path.join(td, "ocxo-test.runtime.toml")
            with open(path, "w") as f:
                f.write('schema_version = "99"\n\n'
                        '[operational_state]\n'
                        'last_known_freq_offset_ppb = 1.0\n'
                        'last_updated = "2026-06-15T00:00:00Z"\n')
            self.assertIsNone(
                load_runtime_state("ocxo-test", dos_dir=td))

    def test_runtime_no_version_returns_none(self):
        with _TempDir() as td:
            path = os.path.join(td, "ocxo-test.runtime.toml")
            with open(path, "w") as f:
                f.write('[operational_state]\n'
                        'last_known_freq_offset_ppb = 1.0\n'
                        'last_updated = "2026-06-15T00:00:00Z"\n')
            self.assertIsNone(
                load_runtime_state("ocxo-test", dos_dir=td))


class TestValidatorAlone(unittest.TestCase):
    """validate_characterization itself, in isolation from the loader.
    Useful for tools that want to validate before writing."""

    def test_empty_yields_errors(self):
        errs = validate_characterization({})
        self.assertGreater(len(errs), 0)

    def test_well_formed_no_errors(self):
        # Build the dict equivalent of GOOD_CHAR_TOML
        data = {
            "schema_version": "1",
            "identity": {
                "do_uid": "ocxo-test",
                "model": "Test",
                "class": "OCXO",
                "actuator_type": "DAC",
                "nominal_freq_hz": 10000000,
            },
            "freerun_noise": {
                "source": "measured",
                "measurement_channel": "DO PPS (chA vs TICC Rb)",
                "sigma_do_phase_ns": 0.0425,
                "sigma_do_freq_ppb": 0.000382,
                "coast_tdev_ref_ns": 0.0425,
                "coast_tdev_slope": 0.530,
            },
            "actuation_noise": {
                "source": "measured",
                "sigma_q_ns": 0.05,
            },
        }
        self.assertEqual(validate_characterization(data), [])


if __name__ == "__main__":
    unittest.main()

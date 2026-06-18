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

    def test_reject_out_of_range_dac_gain(self):
        # dac_gain records the DAC output-stage gain mode the slope was
        # measured at; only 0 (1×) and 1 (2×) are valid.  (Charlie #187
        # finding 1: the reject branch was untested.)
        toml = GOOD_CHAR_TOML.replace(
            "slope_ppb_per_code = 0.02569",
            "slope_ppb_per_code = 0.02569\ndac_gain = 2")
        with _TempDir() as td:
            _write_char(td, "ocxo-test", toml)
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("dac_gain", str(cm.exception))

    def test_accepts_valid_dac_gain(self):
        # Both 0 and 1 must pass (sanity-bracket the reject test).
        for g in (0, 1):
            toml = GOOD_CHAR_TOML.replace(
                "slope_ppb_per_code = 0.02569",
                f"slope_ppb_per_code = 0.02569\ndac_gain = {g}")
            with _TempDir() as td:
                _write_char(td, "ocxo-test", toml)
                c = load_do_characterization("ocxo-test", dos_dir=td)
                self.assertEqual(c.steering["dac_gain"], g)


class TestSteeringEdgeAnchor(unittest.TestCase):
    """noMagicCenterCode: ppb_at_code_min + char_temp_* fields."""

    def test_derives_ppb_at_code_min_for_legacy_file(self):
        # GOOD_CHAR_TOML has parked_code/intercept but no ppb_at_code_min.
        # The loader must derive it = the fitted line evaluated at code_min:
        #   intercept_ppb_at_parked + slope·(code_min − parked_code)
        with _TempDir() as td:
            _write_char(td, "ocxo-test", GOOD_CHAR_TOML)
            c = load_do_characterization("ocxo-test", dos_dir=td)
            expected = 144.65 + 0.02569 * (1024 - 32768)
            self.assertAlmostEqual(
                c.steering["ppb_at_code_min"], expected, places=4)

    def test_explicit_ppb_at_code_min_preserved(self):
        # When the file already carries ppb_at_code_min, the loader must
        # NOT overwrite it with a derivation.
        toml = GOOD_CHAR_TOML.replace(
            "slope_ppb_per_code = 0.02569",
            "slope_ppb_per_code = 0.02569\nppb_at_code_min = -670.85")
        with _TempDir() as td:
            _write_char(td, "ocxo-test", toml)
            c = load_do_characterization("ocxo-test", dos_dir=td)
            self.assertAlmostEqual(c.steering["ppb_at_code_min"], -670.85)

    def test_reject_nonnumeric_ppb_at_code_min(self):
        toml = GOOD_CHAR_TOML.replace(
            "slope_ppb_per_code = 0.02569",
            'slope_ppb_per_code = 0.02569\nppb_at_code_min = "nope"')
        with _TempDir() as td:
            _write_char(td, "ocxo-test", toml)
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("ppb_at_code_min", str(cm.exception))

    def test_accepts_char_temps(self):
        toml = GOOD_CHAR_TOML.replace(
            "slope_ppb_per_code = 0.02569",
            "slope_ppb_per_code = 0.02569\n"
            "char_temp_ocxo_c = 41.2\nchar_temp_cpu_c = 58.7\n"
            "char_temp_board_c = 55.0")
        with _TempDir() as td:
            _write_char(td, "ocxo-test", toml)
            c = load_do_characterization("ocxo-test", dos_dir=td)
            self.assertAlmostEqual(c.steering["char_temp_ocxo_c"], 41.2)
            self.assertAlmostEqual(c.steering["char_temp_cpu_c"], 58.7)
            self.assertAlmostEqual(c.steering["char_temp_board_c"], 55.0)

    def test_reject_out_of_range_char_temp(self):
        toml = GOOD_CHAR_TOML.replace(
            "slope_ppb_per_code = 0.02569",
            "slope_ppb_per_code = 0.02569\nchar_temp_ocxo_c = 999")
        with _TempDir() as td:
            _write_char(td, "ocxo-test", toml)
            with self.assertRaises(SchemaError) as cm:
                load_do_characterization("ocxo-test", dos_dir=td)
            self.assertIn("char_temp_ocxo_c", str(cm.exception))


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


class TestUidSanitization(unittest.TestCase):
    """doUidTomlPathSanitization: MAC/path uids must map to sanitized
    filenames (mirroring do_state._do_path), and equivalent uid spellings
    that map to one file must co-resolve."""

    def test_safe_uid_maps_colon_and_slash(self):
        self.assertEqual(do_schema.safe_uid("54:49:4d:45:00:6b"),
                         "54-49-4d-45-00-6b")
        self.assertEqual(do_schema.safe_uid("/dev/ptp_i226"),
                         "_dev_ptp_i226")
        self.assertEqual(do_schema.safe_uid("ocxo-clkpoc3"), "ocxo-clkpoc3")

    def test_char_path_sanitizes_no_absolute_escape(self):
        # The smoking gun: f"{uid}.toml" with a path uid escaped dos_dir.
        p = do_schema._char_path("/dev/ptp_i226", "/state/dos")
        self.assertEqual(p, "/state/dos/_dev_ptp_i226.toml")
        # Stem matches do_state._do_path (so .toml and legacy .json co-resolve).
        from peppar_fix import do_state
        self.assertEqual(
            os.path.basename(p),
            os.path.basename(do_state._do_path("/dev/ptp_i226", "/state/dos"))
            .replace(".json", ".toml"))

    def test_runtime_path_sanitizes(self):
        p = do_schema._runtime_path("54:49:4d:45:00:6b", "/x")
        self.assertEqual(p, "/x/54-49-4d-45-00-6b.runtime.toml")

    def test_colon_mac_request_loads_dash_mac_file(self):
        # File written under the dash form (as migrate does); engine resolves
        # the colon MAC (as phc_unique_id returns).  Must co-resolve, not
        # trip the identity guard.
        toml = GOOD_CHAR_TOML.replace('do_uid = "ocxo-test"',
                                       'do_uid = "54-49-4d-45-00-6b"')
        with _TempDir() as td:
            _write_char(td, "54-49-4d-45-00-6b", toml)
            c = load_do_characterization("54:49:4d:45:00:6b", dos_dir=td)
            self.assertEqual(c.identity["do_uid"], "54-49-4d-45-00-6b")

    def test_genuine_cross_do_mismatch_still_fatal(self):
        # Sanitize-compare must NOT mask a real cross-DO mismatch.
        toml = GOOD_CHAR_TOML.replace('do_uid = "ocxo-test"',
                                       'do_uid = "ocxo-OTHER"')
        with _TempDir() as td:
            _write_char(td, "ocxo-test", toml)
            with self.assertRaises(SchemaError):
                load_do_characterization("ocxo-test", dos_dir=td)

    def test_path_uid_roundtrip_save_load_runtime(self):
        with _TempDir() as td:
            rs = RuntimeState(last_known_freq_offset_ppb=152.24,
                              last_known_dac_code=None,
                              last_updated="2026-06-16T00:00:00Z")
            save_runtime_state("/dev/ptp_i226", rs, dos_dir=td)
            self.assertTrue(os.path.exists(
                os.path.join(td, "_dev_ptp_i226.runtime.toml")))
            loaded = load_runtime_state("/dev/ptp_i226", dos_dir=td)
            self.assertAlmostEqual(loaded.last_known_freq_offset_ppb, 152.24)


# Identity-only registered DO — the post-migration / post-register starting
# point a do_*_char tool writes a measured section into.
_IDENTITY_ONLY = textwrap.dedent("""\
    schema_version = "1"

    [identity]
    do_uid = "ocxo-test"
    model = "Isotemp OCXO131-100"
    class = "OCXO"
    actuator_type = "DAC"
    nominal_freq_hz = 10000000
    """)

# A measured [steering] section as do_steering_char supplies it (the writer
# adds `source`, not the caller).
_STEERING_FIELDS = dict(
    slope_ppb_per_code=-0.0263, intercept_ppb_at_parked=-7.0,
    parked_code=32768, code_min=8000, code_max=60000,
    asymmetry_factor=1.0, rmse_ppb=0.5, measured_at="2026-06-16T20:00:00Z")


class TestUpdateCharacterizationSection(unittest.TestCase):
    """update_characterization_section — the shared do_*_char writer (steering
    now; freerun/actuation as they migrate).  Read-modify-write ONE section,
    re-validate the whole file, atomic write, fail loud."""

    def test_writes_steering_into_registered_do(self):
        with _TempDir() as td:
            _write_char(td, "ocxo-test", _IDENTITY_ONLY)
            do_schema.update_characterization_section(
                "ocxo-test", "steering", dict(_STEERING_FIELDS), dos_dir=td)
            c = load_do_characterization("ocxo-test", dos_dir=td)
            self.assertEqual(c.provenance["steering"], "measured")
            self.assertAlmostEqual(c.steering["slope_ppb_per_code"], -0.0263)
            self.assertEqual(c.steering["code_min"], 8000)
            self.assertEqual(c.steering["source"], "measured")

    def test_preserves_other_sections(self):
        # Writing [steering] must not disturb a pre-existing measured
        # [freerun_noise] (the whole-file round-trip preserves it).
        with _TempDir() as td:
            _write_char(td, "ocxo-test", GOOD_CHAR_TOML)
            fields = dict(_STEERING_FIELDS, slope_ppb_per_code=-0.0299)
            do_schema.update_characterization_section(
                "ocxo-test", "steering", fields, dos_dir=td)
            c = load_do_characterization("ocxo-test", dos_dir=td)
            self.assertEqual(c.provenance["freerun_noise"], "measured")
            self.assertAlmostEqual(c.freerun_noise["sigma_do_freq_ppb"],
                                   0.000382)
            self.assertAlmostEqual(c.steering["slope_ppb_per_code"], -0.0299)

    def test_refuses_operational_state_firewall(self):
        with _TempDir() as td:
            _write_char(td, "ocxo-test", _IDENTITY_ONLY)
            with self.assertRaises(SchemaError):
                do_schema.update_characterization_section(
                    "ocxo-test", "operational_state",
                    {"last_known_freq_offset_ppb": 1.0}, dos_dir=td)

    def test_refuses_unknown_section(self):
        with _TempDir() as td:
            _write_char(td, "ocxo-test", _IDENTITY_ONLY)
            with self.assertRaises(SchemaError):
                do_schema.update_characterization_section(
                    "ocxo-test", "bogus", {"x": 1}, dos_dir=td)

    def test_refuses_unregistered_do(self):
        with _TempDir() as td:
            with self.assertRaises(SchemaError):
                do_schema.update_characterization_section(
                    "ocxo-ghost", "steering", dict(_STEERING_FIELDS),
                    dos_dir=td)

    def test_invalid_value_rejected_and_file_unchanged(self):
        # slope == 0 fails validation → SchemaError, and the on-disk file is
        # NOT modified (steering stays absent → provenance MISSING).
        with _TempDir() as td:
            path = _write_char(td, "ocxo-test", _IDENTITY_ONLY)
            before = open(path).read()
            bad = dict(_STEERING_FIELDS, slope_ppb_per_code=0.0)
            with self.assertRaises(SchemaError):
                do_schema.update_characterization_section(
                    "ocxo-test", "steering", bad, dos_dir=td)
            self.assertEqual(open(path).read(), before)
            self.assertEqual(
                load_do_characterization("ocxo-test", dos_dir=td)
                .provenance["steering"], "MISSING")

    def test_missing_required_field_rejected(self):
        # A measured steering missing slope_ppb_per_code is incomplete →
        # rejected by validate (the #178 measured-required guarantee).
        with _TempDir() as td:
            _write_char(td, "ocxo-test", _IDENTITY_ONLY)
            incomplete = {k: v for k, v in _STEERING_FIELDS.items()
                          if k != "slope_ppb_per_code"}
            with self.assertRaises(SchemaError):
                do_schema.update_characterization_section(
                    "ocxo-test", "steering", incomplete, dos_dir=td)

    def test_path_uid_writes_sanitized_file(self):
        # MAC/path uid resolves to a sanitized .toml (doUid fix); the writer
        # must hit the same sanitized path the loader reads.
        with _TempDir() as td:
            ident = _IDENTITY_ONLY.replace('do_uid = "ocxo-test"',
                                           'do_uid = "54:49:4d:45:00:6b"')
            _write_char(td, "54-49-4d-45-00-6b", ident)
            do_schema.update_characterization_section(
                "54:49:4d:45:00:6b", "steering", dict(_STEERING_FIELDS),
                dos_dir=td)
            c = load_do_characterization("54:49:4d:45:00:6b", dos_dir=td)
            self.assertEqual(c.provenance["steering"], "measured")


if __name__ == "__main__":
    unittest.main()

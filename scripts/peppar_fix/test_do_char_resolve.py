"""Tests for the DO characterization resolution bridge (PR 2a of
doSchemaEngineIntegration).  Covers the .toml-preferred / legacy-json
fallback / none ordering and provenance carry-through."""
import json
import os
import warnings

import pytest

from peppar_fix import do_char_resolve as dcr


# ── .toml writers ───────────────────────────────────────────────────── #

def _write_toml(dos_dir, uid, *, steering_measured=True,
                freerun_measured=False):
    body = [
        'schema_version = "1"',
        "[identity]",
        f'do_uid = "{uid}"',
        'model = "X"', 'class = "OCXO"', 'actuator_type = "DAC"',
        "nominal_freq_hz = 10000000",
    ]
    if steering_measured:
        body += ['[steering]', 'source = "measured"',
                 "slope_ppb_per_code = 0.025"]
    if freerun_measured:
        body += ['[freerun_noise]', 'source = "measured"',
                 'measurement_channel = "DO PPS (chA vs TICC Rb)"',
                 "sigma_do_phase_ns = 0.0425", "sigma_do_freq_ppb = 0.000382",
                 "coast_tdev_ref_ns = 0.0425", "coast_tdev_slope = 0.53"]
    p = os.path.join(dos_dir, f"{uid}.toml")
    with open(p, "w") as f:
        f.write("\n".join(body) + "\n")
    return p


def _write_legacy_json(dos_dir, uid, *, with_char=True):
    state = {"unique_id": uid, "label": uid,
             "last_known_freq_offset_ppb": -22.0}
    if with_char:
        state["characterization"] = {
            "sources": {
                "DO PPS (chA vs TICC Rb)": {
                    "units": "ns", "asd_at_0.1Hz": 0.0425,
                    "adev_by_tau_s": {"1": 4.0e-11, "100": 5.0e-11},
                    "tdev_ns_by_tau_s": {"1": 0.0425, "10": 0.08, "100": 0.2},
                }}}
    p = os.path.join(dos_dir, f"{uid}.json")
    with open(p, "w") as f:
        json.dump(state, f)
    return p


@pytest.fixture(autouse=True)
def _clear_warn_cache():
    dcr._warned_legacy.clear()
    yield
    dcr._warned_legacy.clear()


# ── ordering ────────────────────────────────────────────────────────── #

def test_toml_preferred_over_json(tmp_path):
    d = str(tmp_path)
    _write_toml(d, "ocxo-a", freerun_measured=True)
    _write_legacy_json(d, "ocxo-a")
    ec = dcr.resolve_engine_characterization("ocxo-a", dos_dir=d,
                                             json_state_dir=d)
    assert ec.schema == dcr.SCHEMA_TOML
    # measured freerun values from the toml, not the json heuristic
    assert ec.sigma_do_freq_ppb == pytest.approx(0.000382)
    assert ec.coast_tdev == (pytest.approx(0.0425), pytest.approx(0.53), 1.0)


def test_legacy_json_when_no_toml(tmp_path):
    d = str(tmp_path)
    _write_legacy_json(d, "ocxo-b")
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        ec = dcr.resolve_engine_characterization("ocxo-b", dos_dir=d,
                                                 json_state_dir=d,
                                                 dac_ppb_per_code=0.0257)
    assert ec.schema == dcr.SCHEMA_LEGACY
    assert ec.sigma_do_phase_ns is not None       # derived from char
    assert ec.sigma_q_ns == pytest.approx(0.0257)  # from dac_ppb_per_code
    assert ec.coast_tdev is not None
    assert any(issubclass(x.category, DeprecationWarning) for x in w)


def test_none_when_neither_present(tmp_path):
    ec = dcr.resolve_engine_characterization("ocxo-ghost", dos_dir=str(tmp_path),
                                             json_state_dir=str(tmp_path))
    assert ec.schema == dcr.SCHEMA_NONE
    assert ec.sigma_do_phase_ns is None
    assert ec.coast_tdev is None
    assert ec.sigma_q_ns is None


# ── provenance ──────────────────────────────────────────────────────── #

def test_toml_class_default_provenance(tmp_path):
    d = str(tmp_path)
    # steering measured but freerun absent → class-default freerun
    _write_toml(d, "ocxo-c", steering_measured=True, freerun_measured=False)
    ec = dcr.resolve_engine_characterization("ocxo-c", dos_dir=d)
    assert ec.provenance["freerun_noise"].startswith("class-default")
    assert ec.steering_provenance == "measured"
    # class-default OCXO values flow through
    assert ec.sigma_do_freq_ppb == pytest.approx(0.001)
    assert ec.sigma_q_ns == pytest.approx(0.05)


def test_toml_missing_steering_provenance(tmp_path):
    d = str(tmp_path)
    _write_toml(d, "ocxo-d", steering_measured=False)
    ec = dcr.resolve_engine_characterization("ocxo-d", dos_dir=d)
    assert ec.steering_provenance == "MISSING"


def test_legacy_steering_never_missing(tmp_path):
    # Legacy path must NOT report steering MISSING (the refuse gate only
    # applies on the .toml path during transition).
    d = str(tmp_path)
    _write_legacy_json(d, "ocxo-e")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        ec = dcr.resolve_engine_characterization("ocxo-e", dos_dir=d,
                                                 json_state_dir=d)
    assert ec.steering_provenance == "legacy-json"


def test_deprecation_warning_one_shot(tmp_path):
    d = str(tmp_path)
    _write_legacy_json(d, "ocxo-f")
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        dcr.resolve_engine_characterization("ocxo-f", dos_dir=d,
                                            json_state_dir=d)
        dcr.resolve_engine_characterization("ocxo-f", dos_dir=d,
                                            json_state_dir=d)
    assert sum(issubclass(x.category, DeprecationWarning) for x in w) == 1


# ── steering refuse policy ──────────────────────────────────────────── #

def _ec(schema, steering_prov):
    return dcr.EngineCharacterization(
        schema=schema, provenance={"steering": steering_prov})


def test_refuse_when_toml_missing_steering_and_servo():
    assert dcr.should_refuse_for_steering(
        _ec(dcr.SCHEMA_TOML, "MISSING"),
        has_servo=True, allow_default_steering=False)


def test_refuse_when_toml_class_default_steering():
    assert dcr.should_refuse_for_steering(
        _ec(dcr.SCHEMA_TOML, "class-default[OCXO]"),
        has_servo=True, allow_default_steering=False)


def test_no_refuse_when_measured():
    assert not dcr.should_refuse_for_steering(
        _ec(dcr.SCHEMA_TOML, "measured"),
        has_servo=True, allow_default_steering=False)


def test_no_refuse_when_escape_hatch():
    assert not dcr.should_refuse_for_steering(
        _ec(dcr.SCHEMA_TOML, "MISSING"),
        has_servo=True, allow_default_steering=True)


def test_no_refuse_without_servo():
    assert not dcr.should_refuse_for_steering(
        _ec(dcr.SCHEMA_TOML, "MISSING"),
        has_servo=False, allow_default_steering=False)


def test_no_refuse_on_legacy_path():
    # Legacy JSON never trips the gate during transition.
    assert not dcr.should_refuse_for_steering(
        _ec(dcr.SCHEMA_LEGACY, "legacy-json"),
        has_servo=True, allow_default_steering=False)


def test_no_refuse_on_none_schema():
    assert not dcr.should_refuse_for_steering(
        _ec(dcr.SCHEMA_NONE, "absent"),
        has_servo=True, allow_default_steering=False)


# ── provenance log formatting ───────────────────────────────────────── #

def test_format_provenance_log_nudges_class_default(tmp_path):
    d = str(tmp_path)
    _write_toml(d, "ocxo-g", steering_measured=True, freerun_measured=False)
    ec = dcr.resolve_engine_characterization("ocxo-g", dos_dir=d)
    lines = dcr.format_provenance_log("ocxo-g", ec)
    text = "\n".join(lines)
    assert "do_freerun_char.py" in text          # class-default nudge
    assert "source = toml" in text


def test_format_provenance_log_flags_missing_steering(tmp_path):
    d = str(tmp_path)
    _write_toml(d, "ocxo-h", steering_measured=False)
    ec = dcr.resolve_engine_characterization("ocxo-h", dos_dir=d)
    text = "\n".join(dcr.format_provenance_log("ocxo-h", ec))
    assert "refuse to actuate" in text

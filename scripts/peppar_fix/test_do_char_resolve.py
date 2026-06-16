"""Tests for the DO characterization engine adapter (do_char_resolve).

Post burn-down (PR 4): .toml-only — an unregistered DO (no .toml) RAISES,
a present-but-invalid .toml RAISES; there is no legacy-json fallback.
"""
import os

import pytest

from peppar_fix import do_char_resolve as dcr
from peppar_fix.do_schema import SchemaError


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


# ── resolve: .toml only ─────────────────────────────────────────────── #

def test_resolves_measured_toml(tmp_path):
    d = str(tmp_path)
    _write_toml(d, "ocxo-a", freerun_measured=True)
    ec = dcr.resolve_engine_characterization("ocxo-a", dos_dir=d)
    assert ec.schema == dcr.SCHEMA_TOML
    assert ec.sigma_do_freq_ppb == pytest.approx(0.000382)
    assert ec.coast_tdev == (pytest.approx(0.0425), pytest.approx(0.53), 1.0)


def test_unregistered_do_raises(tmp_path):
    # No .toml → refuse unregistered DO (no silent Q fallback).
    with pytest.raises(SchemaError) as e:
        dcr.resolve_engine_characterization("ocxo-ghost", dos_dir=str(tmp_path))
    assert "not registered" in str(e.value)


def test_legacy_json_is_ignored_now(tmp_path):
    # A leftover legacy .json must NOT be read — only the .toml counts, so
    # absent .toml still raises even with a .json present.
    d = str(tmp_path)
    import json
    with open(os.path.join(d, "ocxo-b.json"), "w") as f:
        json.dump({"unique_id": "ocxo-b",
                   "last_known_freq_offset_ppb": -22.0}, f)
    with pytest.raises(SchemaError):
        dcr.resolve_engine_characterization("ocxo-b", dos_dir=d)


def test_invalid_toml_propagates_schemaerror(tmp_path):
    # do_uid inside disagrees with the filename → SchemaError on load.
    d = str(tmp_path)
    with open(os.path.join(d, "ocxo-real.toml"), "w") as f:
        f.write('schema_version = "1"\n[identity]\n'
                'do_uid = "ocxo-WRONG"\nmodel = "X"\nclass = "OCXO"\n'
                'actuator_type = "DAC"\nnominal_freq_hz = 10000000\n')
    with pytest.raises(SchemaError):
        dcr.resolve_engine_characterization("ocxo-real", dos_dir=d)


# ── provenance ──────────────────────────────────────────────────────── #

def test_toml_class_default_provenance(tmp_path):
    d = str(tmp_path)
    # steering measured but freerun absent → class-default freerun
    _write_toml(d, "ocxo-c", steering_measured=True, freerun_measured=False)
    ec = dcr.resolve_engine_characterization("ocxo-c", dos_dir=d)
    assert ec.provenance["freerun_noise"].startswith("class-default")
    assert ec.steering_provenance == "measured"
    assert ec.sigma_do_freq_ppb == pytest.approx(0.001)
    assert ec.sigma_q_ns == pytest.approx(0.05)


def test_toml_missing_steering_provenance(tmp_path):
    d = str(tmp_path)
    _write_toml(d, "ocxo-d", steering_measured=False)
    ec = dcr.resolve_engine_characterization("ocxo-d", dos_dir=d)
    assert ec.steering_provenance == "MISSING"


# ── steering refuse policy ──────────────────────────────────────────── #

def _ec(steering_prov):
    return dcr.EngineCharacterization(
        schema=dcr.SCHEMA_TOML, provenance={"steering": steering_prov})


def test_refuse_when_missing_steering_and_servo():
    assert dcr.should_refuse_for_steering(
        _ec("MISSING"), has_servo=True, allow_default_steering=False)


def test_refuse_when_class_default_steering():
    assert dcr.should_refuse_for_steering(
        _ec("class-default[OCXO]"), has_servo=True,
        allow_default_steering=False)


def test_no_refuse_when_measured():
    assert not dcr.should_refuse_for_steering(
        _ec("measured"), has_servo=True, allow_default_steering=False)


def test_no_refuse_when_escape_hatch():
    assert not dcr.should_refuse_for_steering(
        _ec("MISSING"), has_servo=True, allow_default_steering=True)


def test_no_refuse_without_servo():
    assert not dcr.should_refuse_for_steering(
        _ec("MISSING"), has_servo=False, allow_default_steering=False)


# ── provenance log formatting ───────────────────────────────────────── #

def test_format_provenance_log_nudges_class_default(tmp_path):
    d = str(tmp_path)
    _write_toml(d, "ocxo-g", steering_measured=True, freerun_measured=False)
    ec = dcr.resolve_engine_characterization("ocxo-g", dos_dir=d)
    text = "\n".join(dcr.format_provenance_log("ocxo-g", ec))
    assert "do_freerun_char.py" in text          # class-default nudge
    assert "source = toml" in text


def test_format_provenance_log_flags_missing_steering(tmp_path):
    d = str(tmp_path)
    _write_toml(d, "ocxo-h", steering_measured=False)
    ec = dcr.resolve_engine_characterization("ocxo-h", dos_dir=d)
    text = "\n".join(dcr.format_provenance_log("ocxo-h", ec))
    assert "refuse to actuate" in text

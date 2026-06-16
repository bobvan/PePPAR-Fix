"""Tests for the runtime-state resolution bridge (PR 2c of
doSchemaEngineIntegration): .runtime.toml-preferred / legacy-json fallback
reads, .runtime.toml-only writes, and the warm-start check."""
import json
import os
import warnings

import pytest

from peppar_fix import do_schema
from peppar_fix import runtime_state_resolve as rsr


def _write_runtime_toml(dos_dir, uid, freq, code=None):
    rs = do_schema.RuntimeState(last_known_freq_offset_ppb=freq,
                                last_known_dac_code=code,
                                last_updated="2026-06-15T00:00:00Z")
    do_schema.save_runtime_state(uid, rs, dos_dir=dos_dir)


def _write_legacy_json(dos_dir, uid, freq=-22.0, code=None):
    state = {"unique_id": uid, "label": uid,
             "last_known_freq_offset_ppb": freq}
    if code is not None:
        state["dac_code_by_temperature"] = {"last": code}
    with open(os.path.join(dos_dir, f"{uid}.json"), "w") as f:
        json.dump(state, f)


@pytest.fixture(autouse=True)
def _clear_warn():
    rsr._warned_legacy.clear()
    yield
    rsr._warned_legacy.clear()


# ── read ordering ───────────────────────────────────────────────────── #

def test_runtime_toml_preferred(tmp_path):
    d = str(tmp_path)
    _write_runtime_toml(d, "ocxo-a", -21.3, code=32768)
    _write_legacy_json(d, "ocxo-a", freq=999.0, code=11111)
    rr = rsr.resolve_runtime_state("ocxo-a", dos_dir=d, json_state_dir=d)
    assert rr.source == rsr.SOURCE_RUNTIME_TOML
    assert rr.freq_offset_ppb == pytest.approx(-21.3)
    assert rr.dac_code == 32768


def test_legacy_fallback_with_dac_table(tmp_path):
    d = str(tmp_path)
    _write_legacy_json(d, "ocxo-b", freq=-44.5, code=40615)
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        rr = rsr.resolve_runtime_state("ocxo-b", dos_dir=d, json_state_dir=d)
    assert rr.source == rsr.SOURCE_LEGACY
    assert rr.freq_offset_ppb == pytest.approx(-44.5)
    assert rr.dac_code == 40615
    assert any(issubclass(x.category, DeprecationWarning) for x in w)


def test_none_when_neither(tmp_path):
    rr = rsr.resolve_runtime_state("ghost", dos_dir=str(tmp_path),
                                   json_state_dir=str(tmp_path))
    assert rr.source == rsr.SOURCE_NONE
    assert rr.freq_offset_ppb is None and rr.dac_code is None


def test_legacy_deprecation_one_shot(tmp_path):
    d = str(tmp_path)
    _write_legacy_json(d, "ocxo-c")
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        rsr.resolve_runtime_state("ocxo-c", dos_dir=d, json_state_dir=d)
        rsr.resolve_runtime_state("ocxo-c", dos_dir=d, json_state_dir=d)
    assert sum(issubclass(x.category, DeprecationWarning) for x in w) == 1


# ── writes go to .runtime.toml, preserving the other field ──────────── #

def test_update_freq_writes_toml_and_preserves_code(tmp_path):
    d = str(tmp_path)
    _write_runtime_toml(d, "ocxo-d", -20.0, code=30000)
    rsr.update_runtime_freq("ocxo-d", -25.5, dos_dir=d)
    rr = rsr.resolve_runtime_state("ocxo-d", dos_dir=d)
    assert rr.source == rsr.SOURCE_RUNTIME_TOML
    assert rr.freq_offset_ppb == pytest.approx(-25.5)
    assert rr.dac_code == 30000           # preserved


def test_update_dac_preserves_freq(tmp_path):
    d = str(tmp_path)
    _write_runtime_toml(d, "ocxo-e", -18.0, code=30000)
    rsr.update_runtime_dac_code("ocxo-e", 41085, dos_dir=d)
    rr = rsr.resolve_runtime_state("ocxo-e", dos_dir=d)
    assert rr.dac_code == 41085
    assert rr.freq_offset_ppb == pytest.approx(-18.0)   # preserved


def test_update_seeds_from_legacy_on_first_write(tmp_path):
    # First write with only a legacy .json present must seed the other
    # field from legacy, not clobber it.
    d = str(tmp_path)
    _write_legacy_json(d, "ocxo-f", freq=-30.0, code=35000)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        rsr.update_runtime_freq("ocxo-f", -31.0, dos_dir=d, json_state_dir=d)
    rr = rsr.resolve_runtime_state("ocxo-f", dos_dir=d, json_state_dir=d)
    assert rr.source == rsr.SOURCE_RUNTIME_TOML   # now toml-backed
    assert rr.freq_offset_ppb == pytest.approx(-31.0)
    assert rr.dac_code == 35000                   # seeded from legacy


def test_update_freq_writes_toml_not_json(tmp_path):
    d = str(tmp_path)
    rsr.update_runtime_freq("ocxo-g", -22.2, dos_dir=d)
    assert os.path.exists(do_schema._runtime_path("ocxo-g", d))
    assert not os.path.exists(os.path.join(d, "ocxo-g.json"))


# ── warm-start ──────────────────────────────────────────────────────── #

def test_warm_startable_from_toml(tmp_path):
    d = str(tmp_path)
    _write_runtime_toml(d, "ocxo-h", -22.0, code=32768)
    ok, info = rsr.is_warm_startable("ocxo-h", dos_dir=d, json_state_dir=d)
    assert ok and info["source"] == rsr.SOURCE_RUNTIME_TOML
    assert info["freq_ppb"] == pytest.approx(-22.0)


def test_warm_startable_legacy_fallback(tmp_path):
    d = str(tmp_path)
    _write_legacy_json(d, "ocxo-i", freq=-22.0)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        ok, info = rsr.is_warm_startable("ocxo-i", dos_dir=d, json_state_dir=d)
    assert ok and info["source"] == rsr.SOURCE_LEGACY


def test_warm_start_rejects_out_of_envelope(tmp_path):
    d = str(tmp_path)
    _write_runtime_toml(d, "ocxo-j", 5000.0)   # absurd freq
    ok, info = rsr.is_warm_startable("ocxo-j", dos_dir=d, json_state_dir=d)
    assert not ok and "out_of_envelope" in info["reason"]


def test_warm_start_no_file(tmp_path):
    ok, info = rsr.is_warm_startable("ghost", dos_dir=str(tmp_path),
                                     json_state_dir=str(tmp_path))
    assert not ok and info["reason"] == "no_state_file"

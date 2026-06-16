"""Tests for runtime_state_resolve (DO operational state).

Post burn-down (PR 4): .runtime.toml only — no legacy-json fallback.  A
missing runtime file is a cold start (SOURCE_NONE), NOT fatal.
"""
import os

import pytest

from peppar_fix import do_schema
from peppar_fix import runtime_state_resolve as rsr


def _write_runtime_toml(dos_dir, uid, freq, code=None):
    rs = do_schema.RuntimeState(last_known_freq_offset_ppb=freq,
                                last_known_dac_code=code,
                                last_updated="2026-06-15T00:00:00Z")
    do_schema.save_runtime_state(uid, rs, dos_dir=dos_dir)


# ── read ───────────────────────────────────────────────────────────── #

def test_reads_runtime_toml(tmp_path):
    d = str(tmp_path)
    _write_runtime_toml(d, "ocxo-a", -21.3, code=32768)
    rr = rsr.resolve_runtime_state("ocxo-a", dos_dir=d)
    assert rr.source == rsr.SOURCE_RUNTIME_TOML
    assert rr.freq_offset_ppb == pytest.approx(-21.3)
    assert rr.dac_code == 32768


def test_legacy_json_ignored(tmp_path):
    # A leftover legacy .json must NOT be read post burn-down → cold start.
    import json
    d = str(tmp_path)
    with open(os.path.join(d, "ocxo-b.json"), "w") as f:
        json.dump({"unique_id": "ocxo-b", "last_known_freq_offset_ppb": -44.5,
                   "dac_code_by_temperature": {"last": 40615}}, f)
    rr = rsr.resolve_runtime_state("ocxo-b", dos_dir=d)
    assert rr.source == rsr.SOURCE_NONE
    assert rr.freq_offset_ppb is None and rr.dac_code is None


def test_none_when_absent(tmp_path):
    rr = rsr.resolve_runtime_state("ghost", dos_dir=str(tmp_path))
    assert rr.source == rsr.SOURCE_NONE
    assert rr.freq_offset_ppb is None and rr.dac_code is None


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


def test_dac_only_write_no_prior_freq_leaves_freq_unknown(tmp_path):
    # update_runtime_dac_code with NO prior freq must NOT persist a
    # misleading 0.0 — freq stays unknown (Main's PR #176 nit).
    d = str(tmp_path)
    rsr.update_runtime_dac_code("ocxo-z", 33000, dos_dir=d)
    rr = rsr.resolve_runtime_state("ocxo-z", dos_dir=d)
    assert rr.dac_code == 33000
    assert rr.freq_offset_ppb is None          # NOT 0.0
    ok, info = rsr.is_warm_startable("ocxo-z", dos_dir=d)
    assert not ok and info["reason"] == "no_last_known_freq"


def test_update_freq_writes_toml_not_json(tmp_path):
    d = str(tmp_path)
    rsr.update_runtime_freq("ocxo-g", -22.2, dos_dir=d)
    assert os.path.exists(do_schema._runtime_path("ocxo-g", d))
    assert not os.path.exists(os.path.join(d, "ocxo-g.json"))


# ── warm-start ──────────────────────────────────────────────────────── #

def test_warm_startable_from_toml(tmp_path):
    d = str(tmp_path)
    _write_runtime_toml(d, "ocxo-h", -22.0, code=32768)
    ok, info = rsr.is_warm_startable("ocxo-h", dos_dir=d)
    assert ok and info["source"] == rsr.SOURCE_RUNTIME_TOML
    assert info["freq_ppb"] == pytest.approx(-22.0)


def test_warm_start_rejects_out_of_envelope(tmp_path):
    d = str(tmp_path)
    _write_runtime_toml(d, "ocxo-j", 5000.0)   # absurd freq
    ok, info = rsr.is_warm_startable("ocxo-j", dos_dir=d)
    assert not ok and "out_of_envelope" in info["reason"]


def test_warm_start_no_file(tmp_path):
    ok, info = rsr.is_warm_startable("ghost", dos_dir=str(tmp_path))
    assert not ok and info["reason"] == "no_state_file"

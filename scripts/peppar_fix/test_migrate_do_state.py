"""Tests for scripts/migrate_do_state.py (PR 3 of doSchemaEngineIntegration):
legacy DO-state JSON → per-section schema (.toml / .runtime.toml / .noise.toml).
"""
from __future__ import annotations

import json
import os
import sys

import pytest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

import migrate_do_state as mig
from peppar_fix import do_schema


# ── synthetic legacy JSON ───────────────────────────────────────────── #

def _ushape_tdev():
    # OCXO-like: flicker-phase falling, RWFM rising (min at 5 s).
    return {"1.0": 0.0474, "2.0": 0.0329, "5.0": 0.024,
            "10.0": 0.0276, "30.0": 0.0749, "100.0": 0.2825, "300.0": 0.6881}


def _do_source(extra=None):
    s = {"units": "ns", "rms": 1.95, "peak": 6.17, "asd_at_0.1Hz": 0.0105,
         "asd_at_0.01Hz": 0.5647, "slope": -0.65, "noise_type": "flicker_phase",
         "psd_curve": [[0.001, 24.07], [0.002, None], [0.005, 7.71]],
         "adev_by_tau_s": {"1.0": 8.2e-11, "10.0": 2.6e-11, "100.0": 5.0e-11},
         "tdev_ns_by_tau_s": _ushape_tdev(),
         "mdev_by_tau_s": {"1.0": 7.0e-11}}
    if extra:
        s.update(extra)
    return s


def _write_json(d, uid, *, sources=None, freq=-22.08, dac=39024, label=None,
                adjustment=None, updated="2026-06-15T08:43:06Z"):
    state = {"unique_id": uid, "label": label or uid,
             "last_known_freq_offset_ppb": freq, "updated": updated,
             "adjustment": adjustment}
    if sources is not None:
        state["characterization"] = {"sources": sources}
    if dac is not None:
        state["dac_code_by_temperature"] = {"last": dac}
    p = os.path.join(d, f"{uid}.json")
    with open(p, "w") as f:
        json.dump(state, f)
    return p


# ── derive_coast_params ─────────────────────────────────────────────── #

def test_coast_params_uses_actual_1s_and_rising_slope():
    ref, slope = mig.derive_coast_params(_ushape_tdev())
    assert ref == pytest.approx(0.0474)      # actual TDEV(1s), not extrapolated
    assert slope > 0                         # rising RWFM tail


def test_coast_params_none_on_monotone_falling():
    # No rising tail → cannot anchor a positive-slope coast law.
    falling = {"1.0": 0.1, "10.0": 0.05, "100.0": 0.02}
    assert mig.derive_coast_params(falling) is None


def test_tdev_at_1s_interpolates_when_absent():
    # τ=1 missing → log-log interpolate between 0.5 and 2.0.
    pairs = sorted([(0.5, 0.08), (2.0, 0.02), (5.0, 0.01), (50.0, 0.2)])
    v = mig._tdev_at_1s(pairs)
    assert 0.02 < v < 0.08


# ── source selection ────────────────────────────────────────────────── #

def test_select_prefers_chA_vs_Rb():
    sources = {"DO PPS (chA-chB)": _do_source(),
               "DO PPS (chA vs TICC Rb)": _do_source()}
    ch, src = mig.select_source({"characterization": {"sources": sources}})
    assert ch == "DO PPS (chA vs TICC Rb)"


def test_select_rejects_receiver_side_sources():
    # The PiFace channel-mix: receiver/control sources must NOT be selected.
    sources = {"Carrier": {"units": "ns", "asd_at_0.1Hz": 0.1},
               "adjfine": {"units": "ppb", "asd_at_0.1Hz": 0.2}}
    ch, src = mig.select_source({"characterization": {"sources": sources}})
    assert ch is None and src is None


def test_authoritative_prefers_do_pointing_variant(tmp_path):
    d = str(tmp_path)
    # v2 = contaminated (receiver-side only); base = DO-pointing.
    _write_json(d, "ocxo-x", sources={"DO PPS (chA vs TICC Rb)": _do_source()})
    with open(os.path.join(d, "ocxo-x-v2.json"), "w") as f:
        json.dump({"unique_id": "ocxo-x", "label": "ocxo-x",
                   "characterization": {"sources": {
                       "Carrier": {"units": "ns", "asd_at_0.1Hz": 0.1}}}}, f)
    chosen, data, ch, src, dropped = mig.select_authoritative("ocxo-x", d)
    assert ch == "DO PPS (chA vs TICC Rb)"
    assert any("v2" in p for p in dropped)


# ── full migration ──────────────────────────────────────────────────── #

def test_migrate_full_clkpoc3_like(tmp_path):
    d = str(tmp_path)
    _write_json(d, "ocxo-clkpoc3",
                sources={"DO PPS (chA vs TICC Rb)": _do_source()},
                label="ocxo-clkpoc3")
    s = mig.migrate_one("ocxo-clkpoc3", dos_dir=d, apply=True)
    assert not s["errors"], s["errors"]
    assert s["provenance"]["freerun_noise"].startswith("measured")
    assert s["provenance"]["steering"].startswith("MISSING")
    assert s["provenance"]["actuation_noise"].startswith("class-default")
    # New files exist + validate through the real loader.
    c = do_schema.load_do_characterization("ocxo-clkpoc3", dos_dir=d)
    assert c.freerun_noise["coast_tdev_ref_ns"] == pytest.approx(0.0474)
    assert c.freerun_noise["coast_tdev_slope"] > 0
    assert c.identity["class"] == "OCXO"          # inferred from label
    assert c.identity["actuator_type"] == "DAC"   # inferred from dac table
    rt = do_schema.load_runtime_state("ocxo-clkpoc3", dos_dir=d)
    assert rt.last_known_freq_offset_ppb == pytest.approx(-22.08)
    assert rt.last_known_dac_code == 39024
    assert os.path.exists(os.path.join(d, "ocxo-clkpoc3.noise.toml"))
    # Old JSON deleted.
    assert not os.path.exists(os.path.join(d, "ocxo-clkpoc3.json"))


def test_dry_run_writes_nothing(tmp_path):
    d = str(tmp_path)
    jp = _write_json(d, "ocxo-clkpoc3",
                     sources={"DO PPS (chA vs TICC Rb)": _do_source()},
                     label="ocxo-clkpoc3")
    s = mig.migrate_one("ocxo-clkpoc3", dos_dir=d, apply=False)
    assert not s["errors"]
    assert os.path.exists(jp)                                   # not deleted
    assert not os.path.exists(os.path.join(d, "ocxo-clkpoc3.toml"))


def test_class_inference_fails_loud_when_ambiguous(tmp_path):
    d = str(tmp_path)
    # label has no class keyword and no override → must error, not guess.
    _write_json(d, "54-49-4d-45-00-6b",
                sources={"DO PPS (chA vs TICC Rb)": _do_source()},
                label="device-on-mac", dac=None)
    s = mig.migrate_one("54-49-4d-45-00-6b", dos_dir=d, apply=True)
    assert any("class" in e for e in s["errors"])
    assert any("actuator" in e for e in s["errors"])


def test_overrides_supply_identity(tmp_path):
    d = str(tmp_path)
    _write_json(d, "host-mac",
                sources={"DO PPS (chA vs TICC Rb)": _do_source()},
                label="device", dac=None)
    s = mig.migrate_one("host-mac", dos_dir=d, do_class="TCXO",
                        actuator="PHC_adjfine", model="i226", apply=True)
    assert not s["errors"], s["errors"]
    c = do_schema.load_do_characterization("host-mac", dos_dir=d)
    assert c.identity["class"] == "TCXO"
    assert c.identity["actuator_type"] == "PHC_adjfine"


def test_incomplete_freerun_falls_to_class_default(tmp_path):
    d = str(tmp_path)
    # DO source present but no rising-tail TDEV → freerun can't be "measured".
    src = _do_source({"tdev_ns_by_tau_s": {"1.0": 0.1, "10.0": 0.05}})
    _write_json(d, "ocxo-clkpoc3", sources={"DO PPS (chA vs TICC Rb)": src},
                label="ocxo-clkpoc3")
    s = mig.migrate_one("ocxo-clkpoc3", dos_dir=d, apply=True)
    assert not s["errors"], s["errors"]
    assert s["provenance"]["freerun_noise"].startswith("class-default")
    # Loader fills class-default OCXO values.
    c = do_schema.load_do_characterization("ocxo-clkpoc3", dos_dir=d)
    assert c.provenance["freerun_noise"].startswith("class-default")


def test_steering_mapped_when_adjustment_present(tmp_path):
    d = str(tmp_path)
    _write_json(d, "ocxo-clkpoc3",
                sources={"DO PPS (chA vs TICC Rb)": _do_source()},
                label="ocxo-clkpoc3",
                adjustment={"slope_ppb_per_code": 0.02569,
                            "parked_code": 32768})
    s = mig.migrate_one("ocxo-clkpoc3", dos_dir=d, apply=True)
    assert s["provenance"]["steering"] == "measured"
    c = do_schema.load_do_characterization("ocxo-clkpoc3", dos_dir=d)
    assert c.steering["slope_ppb_per_code"] == pytest.approx(0.02569)

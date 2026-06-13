"""Tests for the multi-backend consensus survey layer (London technique)."""
import os
import tomllib

import numpy as np

from peppar_fix import peppar_survey_consensus as C
from peppar_fix.peppar_survey_consensus import (
    ConsensusMember, run_consensus, _ingest_csrs,
)
from peppar_fix.geo_frames import CANONICAL_REALIZATION, Frame

BASE = np.array([3979160.466, -4257.850, 4968043.152])


def _cm(name, dxyz, sig, grade, meta=None):
    return ConsensusMember(
        name, tuple(BASE + np.array(dxyz, dtype=float)), sig, grade,
        Frame(CANONICAL_REALIZATION, 2026.45), True, meta or {})


def _patch(monkeypatch, canned):
    monkeypatch.setattr(C, "_solve_member", lambda name, **k: canned.get(name))


def _read(d, uid):
    with open(os.path.join(str(d), f"{uid}.survey.toml"), "rb") as f:
        return tomllib.load(f)


def test_consensus_mean_excludes_broadcast(tmp_path, monkeypatch):
    canned = {
        "pride": _cm("pride", [0.010, 0, 0.012], 0.020, "rapid", {"iar_pct": 91.4}),
        "csrs": _cm("csrs", [-0.006, 0, -0.005], 0.008, "rapid", {"iar_pct": 89.5}),
        "rtklib": _cm("rtklib", [0.0, 0, 0.48], 0.05, "broadcast", {"mode": "ppp"}),
    }
    _patch(monkeypatch, canned)
    rc = run_consensus(backends=["pride", "csrs", "rtklib"], obs_files=[],
                       work_dir=str(tmp_path), receiver_uid="rx",
                       positions_dir=str(tmp_path), apc=True, provisional=True)
    assert rc == 0
    t = _read(tmp_path, "rx")
    # broadcast solver excluded from the mean (sanity-only) ...
    assert t["source"] == "consensus: pride(rapid) + csrs(rapid)"
    assert "rtklib_excluded_reason" in t
    # ... so the mean is the average of pride + csrs ONLY (rtklib's +0.48 m Z out)
    assert abs(t["ecef_m"][2] - (BASE[2] + (0.012 - 0.005) / 2)) < 1e-6
    assert t["consensus_3d_cm"] < 2.0
    assert t["csrs_iar_pct"] == 89.5
    assert t["kind_note"].startswith("APC")
    assert t["provisional"] is True
    assert t["finals_rerun_after_iso"].endswith("Z")


def test_consensus_gate_fail_writes_nothing(tmp_path, monkeypatch):
    canned = {"pride": _cm("pride", [0, 0, 0], 0.02, "rapid"),
              "csrs": _cm("csrs", [0.20, 0, 0.20], 0.02, "rapid")}  # 28 cm apart
    _patch(monkeypatch, canned)
    rc = run_consensus(backends=["pride", "csrs"], obs_files=[],
                       work_dir=str(tmp_path), receiver_uid="rx",
                       positions_dir=str(tmp_path), tol_3d_cm=3.0)
    assert rc == 1
    assert not (tmp_path / "rx.survey.toml").exists()


def test_consensus_needs_two_precise(tmp_path, monkeypatch):
    # one precise + one broadcast → broadcast can't be in the mean → fail
    canned = {"pride": _cm("pride", [0, 0, 0], 0.02, "rapid"),
              "rtklib": _cm("rtklib", [0, 0, 0.4], 0.05, "broadcast")}
    _patch(monkeypatch, canned)
    rc = run_consensus(backends=["pride", "rtklib"], obs_files=[],
                       work_dir=str(tmp_path), receiver_uid="rx",
                       positions_dir=str(tmp_path))
    assert rc == 1
    assert not (tmp_path / "rx.survey.toml").exists()


def test_consensus_broadcast_gross_aborts(tmp_path, monkeypatch):
    # precise pair agrees, but the broadcast sanity solver is 5 m off → a real
    # bug, not a product signature → abort rather than write.
    canned = {"pride": _cm("pride", [0, 0, 0], 0.02, "rapid"),
              "csrs": _cm("csrs", [0.005, 0, 0.005], 0.01, "rapid"),
              "rtklib": _cm("rtklib", [0, 0, 5.0], 0.1, "broadcast")}
    _patch(monkeypatch, canned)
    rc = run_consensus(backends=["pride", "csrs", "rtklib"], obs_files=[],
                       work_dir=str(tmp_path), receiver_uid="rx",
                       positions_dir=str(tmp_path))
    assert rc == 1
    assert not (tmp_path / "rx.survey.toml").exists()


def test_consensus_broadcast_dm_offset_is_kept_as_sanity(tmp_path, monkeypatch):
    # broadcast 0.48 m off (product signature) → warn, exclude, still write.
    canned = {"pride": _cm("pride", [0, 0, 0], 0.02, "rapid"),
              "csrs": _cm("csrs", [0.005, 0, 0.005], 0.01, "rapid"),
              "rtklib": _cm("rtklib", [0, 0, 0.48], 0.1, "broadcast")}
    _patch(monkeypatch, canned)
    rc = run_consensus(backends=["pride", "csrs", "rtklib"], obs_files=[],
                       work_dir=str(tmp_path), receiver_uid="rx",
                       positions_dir=str(tmp_path))
    assert rc == 0
    assert (tmp_path / "rx.survey.toml").exists()


def test_ingest_csrs(tmp_path):
    p = tmp_path / "csrs.toml"
    p.write_text('ecef_m = [3979160.48, -4257.86, 4968043.16]\n'
                 'sigma_m = 0.009\niar_pct = 89.5\ngrade = "rapid"\n')
    m = _ingest_csrs(str(p))
    assert m is not None and m.ok and m.grade == "rapid"
    assert m.meta["iar_pct"] == 89.5
    assert abs(m.ecef_m[0] - 3979160.48) < 1e-6


def test_ingest_csrs_missing_or_bad(tmp_path):
    assert _ingest_csrs(None) is None
    assert _ingest_csrs(str(tmp_path / "nope.toml")) is None
    bad = tmp_path / "bad.toml"
    bad.write_text('sigma_m = 0.009\n')          # no ecef_m
    assert _ingest_csrs(str(bad)) is None

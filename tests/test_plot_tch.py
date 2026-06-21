"""Synthetic three-cornered-hat (TCH) recovery + guard tests.

No hardware.  Generate three INDEPENDENT phase-noise processes with
KNOWN, DISTINCT Allan levels (white-FM at different h0, plus one with
added random-walk-FM = the "free-runner"), form the three pairwise
difference series the way a shared-reference TICC capture would, run the
TCH solver, and assert it RECOVERS each known individual ADEV within
tolerance at mid τ.  A second case forces a negative-variance condition
(one node ≫ the others) and asserts the guard NaNs it without crashing.
"""
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np
import pytest

_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_ROOT / "scripts"))
sys.path.insert(0, str(_ROOT / "tools"))

import plot_tch  # noqa: E402


def _white_fm_phase(n, h0, rng):
    """Phase (s) of a white-FM process: white frequency noise integrated.

    White FM has ADEV(τ) = sqrt(h0 / (2 τ)).  We synthesize fractional-
    frequency white noise with the matching one-sided PSD and cumulatively
    integrate (× τ0 = 1 s) to phase.  Returns a length-n phase array.
    """
    # For white FM sampled at τ0=1s, ADEV(τ)=sqrt(h0/(2τ)) corresponds to
    # per-sample fractional-freq std σ_y(1) = sqrt(h0/2).
    sigma_y = np.sqrt(h0 / 2.0)
    y = rng.normal(0.0, sigma_y, n)          # fractional frequency
    phase = np.cumsum(y)                       # × τ0 (=1) → phase in s
    return phase


def _rw_fm_phase(n, q, rng):
    """Phase (s) of a random-walk-FM process (the free-runner term).

    RWFM frequency is a random walk; phase is its double integral.  ADEV
    of RWFM has a POSITIVE slope (∝ sqrt(τ)) — the signature the tool must
    surface as 'this node ramps at long τ'.
    """
    dy = rng.normal(0.0, np.sqrt(q), n)        # freq increments
    y = np.cumsum(dy)                           # random-walk frequency
    phase = np.cumsum(y)                        # integrate freq → phase
    return phase


def _write_tch_csv(path: Path, phase_a, phase_b, phase_z):
    """Write a shared-reference TICC CSV whose chA/chB rows encode
    d_XZ = A − Z and d_YZ = B − Z (the two legs a real TICC measures).

    ref time stored as (ref_sec, ref_ps).  We bake the leg phase into
    ref_ps as the deviation from a perfect 1 Hz edge: chA ref_ps carries
    (A − Z); chB ref_ps carries (B − Z).  The loader reconstructs the
    legs (Rb cancels in chA−chB → A−B).  An offset keeps ref_ps positive.
    """
    n = len(phase_a)
    t0 = datetime(2026, 6, 20, 0, 0, 0, tzinfo=timezone.utc)
    d_xz = phase_a - phase_z                    # seconds
    d_yz = phase_b - phase_z
    base = 500_000_000                          # 0.5 s offset, keeps ps > 0
    lines = [
        "# capture_tool=synthetic capture_version=1 git=test utc=2026\n",
        "ts_iso,channel,ref_sec,ref_ps,recv_mono\n",
    ]
    for i in range(n):
        ts = (t0 + timedelta(seconds=i)).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
        a_ps = int(round(base + d_xz[i] * 1e12))
        b_ps = int(round(base + d_yz[i] * 1e12))
        lines.append(f"{ts},chA,{1000 + i},{a_ps},{i + 0.95:.6f}\n")
        lines.append(f"{ts},chB,{1000 + i},{b_ps},{i + 0.96:.6f}\n")
    path.write_text("".join(lines))


def test_tch_recovers_known_individual_adev(tmp_path):
    """Three independent clocks, distinct white-FM levels + one with RWFM.
    TCH must recover each node's individual ADEV within tolerance at mid τ.
    """
    rng = np.random.default_rng(7)
    n = 20000
    # Distinct white-FM levels so the three nodes are clearly separable.
    h0_a = 1e-20      # quietest
    h0_b = 4e-20      # 2× ADEV of A
    h0_z = 9e-20      # 3× ADEV of A  (the "reference" is the noisiest here)
    pa = _white_fm_phase(n, h0_a, rng)
    pb = _white_fm_phase(n, h0_b, rng)
    pz = _white_fm_phase(n, h0_z, rng)
    # Make Z the free-runner: add a small RWFM term so its ADEV ramps up.
    pz = pz + _rw_fm_phase(n, 1e-26, rng)

    csv_path = tmp_path / "synth.csv"
    _write_tch_csv(csv_path, pa, pb, pz)

    res = plot_tch.compute_tch(csv_path)
    assert res["n"] >= n - 5
    taus = res["taus"]
    assert len(taus) > 0

    # Expected white-FM ADEV(τ) = sqrt(h0 / (2τ)); pick a mid τ.
    def exp_adev(h0, tau):
        return np.sqrt(h0 / (2.0 * tau))

    tau_idx = {float(t): i for i, t in enumerate(taus)}
    for tau in (10.0, 30.0, 100.0):
        if tau not in tau_idx:
            continue
        i = tau_idx[tau]
        got_a = res["adev"]["chA"][i]
        got_b = res["adev"]["chB"][i]
        # White-FM components recovered within 35% at mid τ (3CH on finite
        # N has real estimator variance; this is a recovery check, not a
        # precision claim).
        assert np.isfinite(got_a)
        assert np.isfinite(got_b)
        assert got_a == pytest.approx(exp_adev(h0_a, tau), rel=0.35)
        assert got_b == pytest.approx(exp_adev(h0_b, tau), rel=0.35)

    # The free-runner (Rb here) must be the most divergent at long τ:
    # its ADEV/ADEV(short) ratio should exceed the white-FM nodes', which
    # FALL with τ.  Compare long-τ vs short-τ Rb deviation.
    rb = res["adev"]["Rb"]
    finite = np.isfinite(rb)
    assert np.any(finite)
    # Rb's long-τ deviation should not collapse like pure white FM.
    short_i = tau_idx.get(2.0)
    long_i = tau_idx.get(300.0)
    if short_i is not None and long_i is not None:
        cha = res["adev"]["chA"]
        # chA (pure white FM) falls steeply ~1/√τ; Rb (white+RWFM) falls
        # much less.  So Rb/chA ratio GROWS from short to long τ.
        if (np.isfinite(rb[short_i]) and np.isfinite(rb[long_i])
                and np.isfinite(cha[short_i]) and np.isfinite(cha[long_i])):
            ratio_short = rb[short_i] / cha[short_i]
            ratio_long = rb[long_i] / cha[long_i]
            assert ratio_long > ratio_short


def test_tch_negative_variance_guard_nans_without_crash(tmp_path):
    """Force one node to dominate so a solved component variance goes
    negative, and assert the guard NaNs it (no crash, no sqrt-of-negative)
    and counts it."""
    rng = np.random.default_rng(11)
    n = 4000
    # chA and Rb extremely quiet; chB enormous.  Then
    # σ²_chA = ½(σ²_XY + σ²_XZ − σ²_YZ): with B huge, σ²_XY ≈ σ²_YZ ≈ σ²_B
    # and σ²_XZ tiny → σ²_chA ≈ ½(σ²_B + tiny − σ²_B) which, with finite-N
    # estimator scatter, dips negative at some τ.  Likewise σ²_Rb.
    pa = _white_fm_phase(n, 1e-24, rng)        # tiny
    pz = _white_fm_phase(n, 1e-24, rng)        # tiny
    pb = _white_fm_phase(n, 1e-14, rng)        # ~10^5× the ADEV
    csv_path = tmp_path / "dominated.csv"
    _write_tch_csv(csv_path, pa, pb, pz)

    res = plot_tch.compute_tch(csv_path)       # must not raise
    assert res["neg_count"] > 0
    # At least one node has a NaN somewhere (the guarded point).
    any_nan = any(np.any(np.isnan(res["adev"][k])) for k in ("chA", "chB", "Rb"))
    assert any_nan
    # No deviation is ever negative (guard worked; sqrt never hit a neg).
    for k in ("chA", "chB", "Rb"):
        arr = res["adev"][k]
        assert np.all(arr[np.isfinite(arr)] >= 0.0)


def test_tch_groslambert_runs_and_recovers(tmp_path):
    """The Groslambert estimator path runs and recovers the same ordering
    (chA quietest, chB louder) at mid τ."""
    rng = np.random.default_rng(3)
    n = 16000
    pa = _white_fm_phase(n, 1e-20, rng)
    pb = _white_fm_phase(n, 4e-20, rng)
    pz = _white_fm_phase(n, 2e-20, rng)
    csv_path = tmp_path / "gros.csv"
    _write_tch_csv(csv_path, pa, pb, pz)

    res = plot_tch.compute_tch(csv_path, groslambert=True)
    assert res["estimator"] == "groslambert"
    taus = res["taus"]
    tau_idx = {float(t): i for i, t in enumerate(taus)}
    i = tau_idx.get(30.0)
    assert i is not None
    a = res["adev"]["chA"][i]
    b = res["adev"]["chB"][i]
    assert np.isfinite(a) and np.isfinite(b)
    # chB has 4× the h0 → 2× the ADEV; ordering must hold.
    assert b > a


def test_tch_short_capture_returns_empty(tmp_path):
    """Too-short captures return an empty result, not a crash."""
    rng = np.random.default_rng(1)
    n = 30
    pa = _white_fm_phase(n, 1e-20, rng)
    pb = _white_fm_phase(n, 1e-20, rng)
    pz = _white_fm_phase(n, 1e-20, rng)
    csv_path = tmp_path / "short.csv"
    _write_tch_csv(csv_path, pa, pb, pz)
    res = plot_tch.compute_tch(csv_path)
    assert len(res["taus"]) == 0


def test_tch_render_writes_stamped_png(tmp_path):
    """render_tch writes a non-empty stamped PNG and the summary table."""
    rng = np.random.default_rng(5)
    n = 8000
    pa = _white_fm_phase(n, 1e-20, rng)
    pb = _white_fm_phase(n, 4e-20, rng)
    pz = _white_fm_phase(n, 9e-20, rng)
    csv_path = tmp_path / "render.csv"
    _write_tch_csv(csv_path, pa, pb, pz)
    out_png = tmp_path / "tch.png"
    res = plot_tch.render_tch(csv_path, out_png, "DO-A", "DO-B")
    assert out_png.exists() and out_png.stat().st_size > 0
    lines = plot_tch.summary_table(res, "DO-A", "DO-B")
    joined = "\n".join(lines)
    assert "Rb" in joined
    assert "negative-variance" in joined


def test_compare_clocks_tch_flag(tmp_path, monkeypatch):
    """compare_clocks.py --tch writes tch.png and appends the TCH block to
    the verdict, leaving the CDF/stability outputs intact."""
    import compare_clocks

    rng = np.random.default_rng(9)
    n = 6000
    pa = _white_fm_phase(n, 1e-20, rng)
    pb = _white_fm_phase(n, 4e-20, rng)
    pz = _white_fm_phase(n, 9e-20, rng)
    csv_path = tmp_path / "cap.csv"
    _write_tch_csv(csv_path, pa, pb, pz)
    out_dir = tmp_path / "out"

    argv = ["compare_clocks.py", "--from-csv", str(csv_path),
            "--label-a", "DO-A", "--label-b", "DO-B",
            "--out-dir", str(out_dir), "--budget-ns", "2.0", "--tch"]
    monkeypatch.setattr(sys, "argv", argv)
    rc = compare_clocks.main()
    assert rc == 0

    assert (out_dir / "tch.png").exists()
    assert (out_dir / "agreement_cdf.png").exists()
    assert (out_dir / "stability_stack.png").exists()
    text = (out_dir / "verdict.txt").read_text()
    assert "three-cornered hat" in text
    assert "VERDICT:" in text

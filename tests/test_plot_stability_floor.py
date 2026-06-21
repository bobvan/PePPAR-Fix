"""Synthetic tests for the stability-floor overlay helpers.

No hardware.  Exercises the χ² confidence band + the floor/Rb overlay on
synthetic data:

  * the band brackets the point estimate at every τ (lo ≤ tdev ≤ hi),
  * the band WIDENS at long τ (fewer independent windows),
  * NaN Rb points (ill-conditioned TCH) are handled — left as gaps, no
    crash, and the render still writes a stamped PNG,
  * render_stability_floor writes a non-empty stamped PNG and returns a
    summary with the TICC floor + budget references in ns.
"""
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np
import pytest

_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_ROOT / "scripts"))
sys.path.insert(0, str(_ROOT / "tools"))

import plot_stability_floor as psf  # noqa: E402


# --- synthetic phase/CSV helpers (mirror test_plot_tch conventions) ------

def _white_fm_phase(n, h0, rng):
    """White-FM phase (s): ADEV(τ)=sqrt(h0/(2τ)); σ_y(1)=sqrt(h0/2)."""
    sigma_y = np.sqrt(h0 / 2.0)
    y = rng.normal(0.0, sigma_y, n)
    return np.cumsum(y)


def _rw_fm_phase(n, q, rng):
    """Random-walk-FM phase (s) — positive-slope ADEV (the free-runner)."""
    dy = rng.normal(0.0, np.sqrt(q), n)
    y = np.cumsum(dy)
    return np.cumsum(y)


def _write_tch_csv(path: Path, phase_a, phase_b, phase_z):
    """Write a shared-ref TICC CSV: chA carries (A−Z), chB carries (B−Z)."""
    n = len(phase_a)
    t0 = datetime(2026, 6, 21, 0, 0, 0, tzinfo=timezone.utc)
    d_xz = phase_a - phase_z
    d_yz = phase_b - phase_z
    base = 500_000_000
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


# --- χ² band helper -------------------------------------------------------

def test_edf_collapses_at_long_tau():
    """Equivalent DoF must fall as τ grows (fewer independent windows)."""
    taus = np.array([1.0, 10.0, 100.0, 1000.0])
    edf = psf.edf_from_taus(taus, n=10000)
    assert np.all(np.diff(edf) < 0)            # strictly decreasing
    assert np.all(edf >= 1.0)                   # floored at 1


def test_band_brackets_point_estimate():
    """lo ≤ tdev ≤ hi at every τ for a positive edf range."""
    taus = np.array([1.0, 10.0, 100.0, 1000.0])
    tdev = np.array([0.05, 0.08, 0.20, 0.40])
    edf = psf.edf_from_taus(taus, n=10000)
    lo, hi = psf.chi2_tdev_ci(tdev, edf)
    assert np.all(lo <= tdev + 1e-15)
    assert np.all(hi >= tdev - 1e-15)
    assert np.all(lo > 0)


def test_band_widens_at_long_tau():
    """The fractional band width hi/lo must GROW from short to long τ."""
    taus = np.array([1.0, 10.0, 100.0, 1000.0])
    tdev = np.full_like(taus, 0.1)              # flat estimate isolates edf
    edf = psf.edf_from_taus(taus, n=10000)
    lo, hi = psf.chi2_tdev_ci(tdev, edf)
    width_ratio = hi / lo
    assert np.all(np.diff(width_ratio) > 0)     # monotonically widening


def test_tdev_with_band_on_synthetic_phase():
    """tdev_with_band returns aligned arrays; band brackets the estimate
    and widens at the longest τ vs the shortest."""
    rng = np.random.default_rng(13)
    phase_ns = _white_fm_phase(20000, 1e-20, rng) * 1e9
    taus, tdev, lo, hi = psf.tdev_with_band(phase_ns)
    assert len(taus) > 0
    assert len(taus) == len(tdev) == len(lo) == len(hi)
    good = np.isfinite(tdev)
    assert np.all(lo[good] <= tdev[good] + 1e-12)
    assert np.all(hi[good] >= tdev[good] - 1e-12)
    # Band fractional width at the longest τ exceeds the shortest.
    assert (hi[-1] / lo[-1]) > (hi[0] / lo[0])


def test_tdev_with_band_empty_on_short():
    """Too-short record → empty arrays, no crash."""
    taus, tdev, lo, hi = psf.tdev_with_band(np.zeros(10))
    assert len(taus) == 0 and len(tdev) == 0 and len(lo) == 0 and len(hi) == 0


# --- render + Rb overlay --------------------------------------------------

def test_render_writes_stamped_png_and_summary(tmp_path):
    """render_stability_floor writes a non-empty PNG and returns a summary
    carrying the TICC floor + budget in ns, plus per-clock TDEV arrays."""
    rng = np.random.default_rng(21)
    n = 12000
    pa = _white_fm_phase(n, 1e-20, rng)
    pb = _white_fm_phase(n, 4e-20, rng)
    pz = _white_fm_phase(n, 9e-20, rng)
    csv_path = tmp_path / "synth.csv"
    _write_tch_csv(csv_path, pa, pb, pz)
    out_png = tmp_path / "floor.png"

    res = psf.render_stability_floor(csv_path, out_png, "DO-A", "DO-B",
                                     dataset="synth 2026-06-21")
    assert out_png.exists() and out_png.stat().st_size > 0
    assert res["ticc_floor_ns"] == pytest.approx(0.060)
    assert res["budget_ns"] == pytest.approx(0.350)
    assert len(res["DO-A"]["tdev"]) > 0
    assert len(res["DO-B"]["tdev"]) > 0
    # Per-clock band arrays align with the TDEV arrays.
    assert len(res["DO-A"]["lo"]) == len(res["DO-A"]["tdev"])
    # Rb curve present (may carry NaN gaps); arrays are aligned.
    assert len(res["rb"]["taus"]) == len(res["rb"]["tdev"])


def test_render_handles_nan_rb_ill_conditioned(tmp_path):
    """Very disparate clocks → TCH Rb estimate hits negative-variance NaN
    gaps; the render must NOT crash and must still write a PNG.  We assert
    that at least one Rb τ point is NaN (the ill-conditioned gap)."""
    rng = np.random.default_rng(31)
    n = 4000
    # chA + Rb tiny, chB enormous → forces negative-variance guard (same
    # construction as test_plot_tch's guard test).
    pa = _white_fm_phase(n, 1e-24, rng)
    pz = _white_fm_phase(n, 1e-24, rng)
    pb = _white_fm_phase(n, 1e-14, rng)
    csv_path = tmp_path / "disparate.csv"
    _write_tch_csv(csv_path, pa, pb, pz)
    out_png = tmp_path / "floor_nan.png"

    res = psf.render_stability_floor(csv_path, out_png, "quiet", "loud")
    assert out_png.exists() and out_png.stat().st_size > 0
    assert res["rb"]["neg_count"] > 0
    rb = res["rb"]["tdev"]
    # At least one Rb point is a NaN gap (left out of the line).
    assert np.any(np.isnan(rb))

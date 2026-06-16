"""Tests for the Q[3,3]-source A/B harness.

The real PiFace/clkPoC3 traces live on gt (~10 MB each, not in-repo), so
these tests synthesize traces and exercise the replay mechanics, the arm
threshold logic, and the synthetic over-tight arm.  The authoritative
real-data PASS numbers are recorded in docs/q-matrix-source-change-sim.md
and reproduced by `python -m peppar_fix.qmatrix_source_ab --piface-csv ...
--clkpoc3-csv ...`.
"""
import csv
import math

import numpy as np
import pytest

from peppar_fix import qmatrix_source_ab as qa


# ── synthetic trace generator ───────────────────────────────────────── #

def _synth_rows(n=600, ticc_noise_ns=2.0, dtrx_rate_ns_s=0.0, seed=0,
                with_qerr=False):
    """Build servo-CSV-shaped rows with a noisy TICC measurement stream.

    The recorded adjfine is produced by replaying through DOFreqEst at a
    reference Q, so faithfulness at that Q reproduces it exactly.
    """
    rng = np.random.default_rng(seed)
    ticc = rng.normal(0.0, ticc_noise_ns, n)
    dtrx = np.cumsum(np.full(n, dtrx_rate_ns_s)) + rng.normal(0.0, 0.1, n)
    rows = []
    for i in range(n):
        r = {
            "discipline_interval": "1.0",
            "pps_err_ticc_ns": f"{ticc[i]:.4f}",
            "ticc_confidence": "0.6",
            "dt_rx_ns": f"{dtrx[i]:.4f}",
            "dt_rx_sigma_ns": "0.1",
            "adjfine_ppb": "0.0",        # filled below
            "qerr_ns": ("0.01" if with_qerr else ""),
        }
        rows.append(r)
    return rows


def _stamp_recorded_adjfine(rows, ref_q):
    """Set adjfine_ppb to what a replay at ref_q produces (so the anchor
    reproduces it to ~0 error — tests the plumbing, not the physics)."""
    res = qa.replay_servo_csv(rows, ref_q)
    for i, r in enumerate(rows):
        v = res.adjfine_ppb[i]
        r["adjfine_ppb"] = f"{(0.0 if not math.isfinite(v) else v):.6f}"
    return rows


# ── helpers ─────────────────────────────────────────────────────────── #

def test_f_parsing():
    assert qa._f({"a": "1.5"}, "a") == 1.5
    assert qa._f({"a": ""}, "a") is None
    assert qa._f({"a": "nan"}, "a") != qa._f({"a": "nan"}, "a")  # NaN
    assert qa._f({}, "missing") is None


def test_load_servo_csv_roundtrip(tmp_path):
    p = tmp_path / "servo.csv"
    with open(p, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["adjfine_ppb", "pps_err_ticc_ns"])
        w.writeheader()
        w.writerow({"adjfine_ppb": "1.0", "pps_err_ticc_ns": "2.0"})
    rows = qa.load_servo_csv(str(p))
    assert len(rows) == 1 and rows[0]["adjfine_ppb"] == "1.0"


def test_recorded_adjfine_std():
    rows = [{"adjfine_ppb": str(v)} for v in (1.0, 2.0, 3.0)]
    assert qa.recorded_adjfine_std(rows) == pytest.approx(np.std([1, 2, 3]))


# ── replay mechanics ────────────────────────────────────────────────── #

def test_replay_shape_and_finiteness():
    rows = _synth_rows(n=300, seed=1)
    res = qa.replay_servo_csv(rows, qa.NEW_SIGMA_DO_FREQ_PPB)
    assert res.adjfine_ppb.shape == (300,)
    assert np.isfinite(res.adjfine_ppb).all()
    assert np.isfinite(res.sqrt_p22_ns).all()


def test_replay_empty_raises():
    with pytest.raises(ValueError):
        qa.replay_servo_csv([], qa.NEW_SIGMA_DO_FREQ_PPB)


def test_replay_gap_is_predict_only():
    rows = _synth_rows(n=400, seed=2)
    gs, gl = 150, 100
    res = qa.replay_servo_csv(rows, qa.NEW_SIGMA_DO_FREQ_PPB,
                              gap_start_idx=gs, gap_len=gl)
    # No adjfine emitted inside the gap (predict-only).
    assert np.isnan(res.adjfine_ppb[gs:gs + gl]).all()
    assert np.isfinite(res.adjfine_ppb[:gs]).all()
    # P[2,2] uncertainty grows across the gap (no measurement to shrink it).
    assert res.sqrt_p22_ns[gs + gl - 1] > res.sqrt_p22_ns[gs]


def test_replay_qerr_arm_not_fed():
    # qErr column present but the harness must never feed the qErr arm
    # (F9P/X20P HPG report 0).  Result is identical with/without the column.
    a = qa.replay_servo_csv(_synth_rows(seed=3, with_qerr=False),
                            qa.NEW_SIGMA_DO_FREQ_PPB).adjfine_ppb
    b = qa.replay_servo_csv(_synth_rows(seed=3, with_qerr=True),
                            qa.NEW_SIGMA_DO_FREQ_PPB).adjfine_ppb
    np.testing.assert_allclose(a, b)


def test_looser_q_chases_more():
    """Core mechanism: a looser Q[3,3] amplifies measurement noise into the
    frequency/actuator estimate, so adjfine std rises with sigma_do_freq."""
    rows = _synth_rows(n=800, ticc_noise_ns=3.0, seed=4)
    tight = qa.replay_servo_csv(rows, qa.NEW_SIGMA_DO_FREQ_PPB).adjfine_std()
    loose = qa.replay_servo_csv(rows, qa.OLD_SIGMA_DO_FREQ_PPB).adjfine_std()
    assert loose > tight


# ── arms ────────────────────────────────────────────────────────────── #

def test_faithfulness_anchor_passes_on_self_consistent_trace():
    q = qa.NEW_SIGMA_DO_FREQ_PPB
    rows = _stamp_recorded_adjfine(_synth_rows(n=500, seed=5), q)
    r = qa.faithfulness_anchor(rows, q)
    assert r.passed
    assert r.detail["rel_err"] < 0.05


def test_faithfulness_anchor_fails_on_wrong_q():
    # Recorded at a tight Q, anchor replays at a wildly loose Q → mismatch.
    rows = _stamp_recorded_adjfine(_synth_rows(n=500, ticc_noise_ns=3.0,
                                               seed=6),
                                   qa.NEW_SIGMA_DO_FREQ_PPB)
    r = qa.faithfulness_anchor(rows, qa.OLD_SIGMA_DO_FREQ_PPB)
    assert not r.passed


def test_over_loose_arm_threshold_logic():
    # PiFace-like: noisy ticc so OLD/NEW std ratio is large.
    pf = _synth_rows(n=900, ticc_noise_ns=8.0, seed=7)
    # clkPoC3-like: clean, recorded == replay at NEW so the stable check holds.
    clk = _stamp_recorded_adjfine(
        _synth_rows(n=900, ticc_noise_ns=0.5, seed=8),
        qa.NEW_SIGMA_DO_FREQ_PPB)
    r = qa.over_loose_arm(pf, clk, min_reduction=2.0)
    assert r.detail["reduction_x"] >= 2.0
    assert r.detail["clkpoc3_new_vs_recorded_rel"] < 0.30
    assert r.passed
    # A reduction threshold above the achieved ratio must FAIL the arm.
    r2 = qa.over_loose_arm(pf, clk, min_reduction=1e9)
    assert not r2.passed


def test_over_tight_arm_synthetic_matched_beats_overconfident():
    r = qa.over_tight_arm_synthetic(seed=7)
    assert not r.detail["matched_diverged"]
    assert not r.detail["overtight_diverged"]
    # Matched Q recovers with a no-larger post-gap excursion.
    assert (r.detail["matched_post_max_ns"]
            <= r.detail["overtight_post_max_ns"])
    assert r.passed


def test_arm_result_str():
    r = qa.ArmResult("x", True, {"a": 1})
    assert "PASS" in str(r) and "a=1" in str(r)
    assert "FAIL" in str(qa.ArmResult("y", False))

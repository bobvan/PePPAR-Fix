"""Unit tests for tools/sync_agreement_model.py (forward two-clock agreement model).

Guards the invariants the model is used for, not the exact numbers: the
three-band structure (DO grows with tau_c, meas shrinks, ref is flat), the
crossover producing an interior optimum, and round-tripping backfit.
"""
import math
import sys
from pathlib import Path

import pytest

_TOOLS = Path(__file__).resolve().parent.parent / "tools"
sys.path.insert(0, str(_TOOLS))

import sync_agreement_model as sam  # noqa: E402

OCXO = dict(a1=1e-11, floor=3e-12)
EXTINT = 8.0


def test_quantization_rms_is_q_over_sqrt12():
    """One epoch, 1 Hz: the meas term is exactly the uniform-quantization RMS."""
    assert sam.sigma_meas_ns(EXTINT, tau_c=1.0, fs=1.0) == \
        pytest.approx(8.0 / math.sqrt(12.0))


def test_bands_have_opposite_slopes():
    """DO term grows with tau_c, measurement term shrinks -- that is the crossover."""
    taus = [10.0, 30.0, 100.0, 300.0]
    do = [sam.sigma_do_ns(t, **OCXO) for t in taus]
    meas = [sam.sigma_meas_ns(EXTINT, t) for t in taus]
    assert do == sorted(do)
    assert meas == sorted(meas, reverse=True)


def test_optimum_is_interior_and_balanced():
    """The crossover gives an interior optimum where the two bands are comparable."""
    p95, tau, terms = sam.optimize_tau(q_ns=EXTINT, **OCXO)
    assert 10.0 < tau < 1000.0
    assert 0.3 < terms['DO'] / terms['meas'] < 3.0
    # Three independent routes (this model, the 1-D EKF in
    # doqerr-extint-tick-model.md, and the budget's 354 ps/clock) agree near 1 ns.
    assert 0.7 < p95 < 1.3


def test_ref_term_is_flat_in_tau_and_survives_optimization():
    """sigma_ref does not average down -- the loop tracks it."""
    for tau in (10.0, 100.0, 1000.0):
        _, terms = sam.p95_ns(tau, q_ns=EXTINT, sigma_ref_ns=0.4, **OCXO)
        assert terms['ref'] == 0.4
    lo, _, _ = sam.optimize_tau(q_ns=EXTINT, sigma_ref_ns=0.0, **OCXO)
    hi, _, _ = sam.optimize_tau(q_ns=EXTINT, sigma_ref_ns=2.0, **OCXO)
    assert hi > lo


def test_timestamper_choice_stops_mattering_under_large_ref():
    """Above ~0.5 ns of reference error the observer columns converge.

    This is why the TICC-vs-EXTINT A/B kept coming out ambiguous.
    """
    a, _, _ = sam.optimize_tau(q_ns=8.0, sigma_ref_ns=2.0, **OCXO)
    b, _, _ = sam.optimize_tau(q_ns=0.060, sigma_ref_ns=2.0, **OCXO)
    assert abs(a - b) / a < 0.05
    # ...and it very much does matter when the reference is clean.
    a0, _, _ = sam.optimize_tau(q_ns=8.0, sigma_ref_ns=0.0, **OCXO)
    b0, _, _ = sam.optimize_tau(q_ns=0.060, sigma_ref_ns=0.0, **OCXO)
    assert a0 / b0 > 5.0


def test_bias_enters_undiminished():
    """A constant per-unit bias is not averaged by anything."""
    p95, _, _ = sam.optimize_tau(q_ns=0.060, bias_ns=1.0, **OCXO)
    assert p95 > sam.P95_K * math.sqrt(2.0) * 1.0


def test_backfit_round_trips():
    kw = dict(q_ns=0.060, **OCXO)
    _, tau, _ = sam.optimize_tau(**kw)
    truth = 2.9
    p95, _ = sam.p95_ns(tau, sigma_ref_ns=truth, **kw)
    assert sam.backfit_sigma_ref(p95, tau, **kw) == \
        pytest.approx(truth, rel=1e-6)


def test_backfit_clamps_below_floor():
    """A measurement at/below the modelled floor implies zero, not a NaN."""
    kw = dict(q_ns=EXTINT, **OCXO)
    _, tau, _ = sam.optimize_tau(**kw)
    assert sam.backfit_sigma_ref(0.001, tau, **kw) == 0.0


def test_faster_measurement_rate_helps_a_coarse_observer():
    slow, _, _ = sam.optimize_tau(q_ns=EXTINT, fs=1.0, **OCXO)
    fast, _, _ = sam.optimize_tau(q_ns=EXTINT, fs=10.0, **OCXO)
    assert fast < slow


def test_component_libraries_are_consistent():
    for d in sam.DO_CLASSES.values():
        assert d['a1'] > d['floor'] > 0
        assert d['label']
    assert sam.TIMESTAMPERS['extint-f9t']['q_ns'] == 8.0   # 125 MHz grid
    assert sam.DACS['16-bit'] > sam.DACS['18-bit'] > sam.DACS['20-bit']

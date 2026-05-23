"""Unit tests for TdcpInnovGate (L2 of TDCP slip protection)."""

from __future__ import annotations

import math

import pytest

from peppar_fix.tdcp_innov_gate import TdcpInnovGate, GateResult


def test_warmup_accepts_all_with_nan_deviation():
    """First `window` observations always accepted; deviation NaN."""
    g = TdcpInnovGate(window=5, n_sigma=5.0)
    for v in (0.10, 0.11, 0.09, 0.12, 0.08):
        r = g.evaluate(v)
        assert r.accepted is True
        assert math.isnan(r.deviation)


def test_post_warmup_accepts_in_range():
    """After warmup, observations near the median pass."""
    g = TdcpInnovGate(window=5, n_sigma=5.0)
    # Steady-state: noise σ = 0.01 ppb around 0.10
    for v in (0.10, 0.11, 0.09, 0.12, 0.08):
        g.evaluate(v)
    r = g.evaluate(0.105)
    assert r.accepted is True
    assert r.deviation < 5.0


def test_post_warmup_rejects_co_slip_punch():
    """A 0.6 ppb punch (multi-SV co-slip) is rejected."""
    g = TdcpInnovGate(window=5, n_sigma=5.0)
    for v in (0.10, 0.11, 0.09, 0.12, 0.08):
        g.evaluate(v)
    r = g.evaluate(0.7)  # 0.6 ppb above the ~0.10 baseline
    assert r.accepted is False
    assert r.deviation > 5.0


def test_buffer_always_buffers_for_adaptive_drift():
    """A sustained shift eventually re-anchors the gate."""
    g = TdcpInnovGate(window=5, n_sigma=5.0)
    # Warmup at 0.10
    for v in (0.10, 0.11, 0.09, 0.12, 0.08):
        g.evaluate(v)

    # A sustained step to 0.50 (would be 'co-slip'-shaped initially)
    # — first epoch is rejected.  We still buffer it (adaptive),
    # so after `window` more observations the gate has re-anchored.
    for _ in range(5):
        g.evaluate(0.50)

    # Now the gate centers near 0.50.  An observation at 0.51 should pass.
    r = g.evaluate(0.51)
    assert r.accepted is True, f"deviation={r.deviation}"


def test_mad_zero_safe_fallback():
    """Identical buffer → MAD=0; the σ floor prevents div-by-zero
    AND the gate should remain permissive (don't crash, don't reject
    everything)."""
    g = TdcpInnovGate(window=5, n_sigma=5.0)
    for _ in range(5):
        g.evaluate(0.10)
    # Buffer is identical → median=0.10, mad=0, σ→floor.
    # Any non-trivial deviation will be rejected (huge devation count),
    # but the small floor is enough to NOT crash.
    r = g.evaluate(0.10)
    assert r.accepted is True
    assert r.sigma > 0
    # A real outlier should still be rejected
    r = g.evaluate(0.50)
    assert r.accepted is False


def test_constructor_rejects_bad_params():
    with pytest.raises(ValueError):
        TdcpInnovGate(window=2)
    with pytest.raises(ValueError):
        TdcpInnovGate(n_sigma=0)
    with pytest.raises(ValueError):
        TdcpInnovGate(n_sigma=-1.0)
    with pytest.raises(ValueError):
        TdcpInnovGate(sigma_floor_ppb=0)
    with pytest.raises(ValueError):
        TdcpInnovGate(sigma_floor_ppb=-0.01)


def test_reset_clears_buffer():
    g = TdcpInnovGate(window=5, n_sigma=5.0)
    for v in (0.10, 0.11, 0.09, 0.12, 0.08):
        g.evaluate(v)
    # After warmup, a 0.50 obs would be rejected.
    g.reset()
    # Post-reset: back to warmup, accept everything for `window`.
    r = g.evaluate(0.50)
    assert r.accepted is True
    assert math.isnan(r.deviation)


def test_sunrise_coslip_fixture():
    """Simulates day0419h sunrise: ~5 minutes of clean ~0.026 ppb σ,
    then a single ~0.6 ppb frequency punch from an N=3 co-slip on
    rising SVs (3 × 0.19 m / 16 SVs / 1 s ≈ 36 mm/s ≈ 0.12 ppb in
    ensemble terms — but with worse case ~0.6 ppb when only a few
    SVs are tracked at sunrise)."""
    import random
    random.seed(42)
    g = TdcpInnovGate(window=30, n_sigma=5.0)

    # 30 epochs of clean noise around 0 ppb
    for _ in range(30):
        g.evaluate(random.gauss(0.0, 0.026))

    # Post-warmup steady: 30 more epochs, all should pass
    pre_punch_passes = 0
    for _ in range(30):
        r = g.evaluate(random.gauss(0.0, 0.026))
        if r.accepted:
            pre_punch_passes += 1
    assert pre_punch_passes >= 28, f"clean noise should pass; got {pre_punch_passes}/30"

    # Single co-slip frequency punch
    r = g.evaluate(0.6)
    assert not r.accepted, "co-slip punch must be rejected"

    # Next epoch back to clean — should pass again (single bad epoch
    # didn't permanently shift the gate)
    r = g.evaluate(0.01)
    assert r.accepted is True

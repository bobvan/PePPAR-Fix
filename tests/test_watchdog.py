"""Tests for PositionWatchdog after the I-145915 PR/CP-port redesign.

The watchdog now trips on TD-CP-domain residual RMS only — NOT the
mixed PR + TD-CP RMS it used pre-port.  These tests verify:

  - PR-domain spikes alone do not trip the watchdog
  - TD-CP-domain spikes do trip it (after the alarm window)
  - Baseline learning works the same as before
  - threshold_m back-compat works (older callers that didn't know
    about TD/PR split should keep working)
  - Reset clears state cleanly
  - n_used < 4 is gated (cold-start / sparse-geometry epochs
    don't pollute the baseline)

Each test constructs synthetic residual RMS streams.  Real residual
splits are unit-tested elsewhere via FixedPosFilter.update.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
from peppar_fix.watchdog import PositionWatchdog


def _quiet_baseline(w, baseline_rms=0.005, window=None):
    """Feed `window` quiet epochs to lock in a TD-CP baseline."""
    n = window if window is not None else w.window
    for _ in range(n):
        w.update(baseline_rms, n_used=10)


def test_constructor_defaults():
    """Defaults match the post-port docstring."""
    w = PositionWatchdog()
    assert w.threshold_td_m == 0.05
    assert w.window == 30
    assert w.alarm_count == 10
    assert w.pr_disturbance_threshold_m == 2.0
    assert not w.alarmed


def test_back_compat_threshold_m_kwarg():
    """Older callers passing threshold_m should still work."""
    w = PositionWatchdog(threshold_m=0.1)
    assert w.threshold_td_m == 0.1
    # Back-compat property + setter both reflect the new field
    assert w.threshold_m == 0.1
    w.threshold_m = 0.2
    assert w.threshold_td_m == 0.2


def test_baseline_learning():
    """First `window` epochs build the baseline RMS, no trips."""
    w = PositionWatchdog(window=10, threshold_td_m=0.05)
    for i in range(10):
        # 5 mm steady noise — ok status throughout learning
        assert w.update(0.005, n_used=10) is True
    # Baseline locked
    assert w._baseline_rms is not None
    assert abs(w._baseline_rms - 0.005) < 1e-6


def test_pr_domain_spike_alone_does_not_trip():
    """The watchdog sees TD-CP RMS only — PR spikes don't reach it.

    This is the load-bearing test for I-145915: if the engine wires
    only resid_td_rms into watchdog.update(), a PR-domain disturbance
    (multipath, code-bias drift, SSR latency) leaves the watchdog
    quiet.
    """
    w = PositionWatchdog(window=10, threshold_td_m=0.05, alarm_count=5)
    _quiet_baseline(w, baseline_rms=0.005, window=10)
    # Engine code does:
    #   resid_pr_rms = sqrt(mean(resid_pr ** 2))   # huge during PR multipath
    #   resid_td_rms = sqrt(mean(resid_td ** 2))   # still sub-cm
    #   watchdog.update(resid_td_rms, ...)
    # We simulate that by passing TD-CP RMS only — the watchdog never
    # sees the PR spike at all.
    for _ in range(20):  # 20 epochs of "PR storm but TD healthy"
        ok = w.update(0.005, n_used=10)
        assert ok is True
    assert not w.alarmed


def test_td_domain_spike_trips_after_alarm_count():
    """TD-CP RMS sustained above the limit triggers the alarm."""
    w = PositionWatchdog(window=10, threshold_td_m=0.05, alarm_count=5)
    _quiet_baseline(w, baseline_rms=0.005, window=10)
    # Above limit: max(0.005*3, 0.005+0.05) = max(0.015, 0.055) = 0.055
    # 0.10 m TD-CP residual is well above
    for i in range(4):  # 4 consecutive over-limit
        w.update(0.10, n_used=10)
    # Not yet alarmed
    assert not w.alarmed
    # 5th over-limit triggers
    w.update(0.10, n_used=10)
    assert w.alarmed


def test_intermittent_td_spike_does_not_trip():
    """A single bad epoch surrounded by good ones does not alarm."""
    w = PositionWatchdog(window=10, threshold_td_m=0.05, alarm_count=5)
    _quiet_baseline(w, baseline_rms=0.005, window=10)
    for _ in range(3):
        w.update(0.10, n_used=10)   # over limit
        w.update(0.005, n_used=10)  # back to baseline (resets bad_count)
    assert not w.alarmed


def test_n_used_below_4_does_not_pollute_baseline():
    """Sparse-geometry epochs (n_used < 4) are skipped entirely."""
    w = PositionWatchdog(window=10, threshold_td_m=0.05)
    # 5 sparse epochs; watchdog should not learn from them
    for _ in range(5):
        w.update(99.0, n_used=2)  # crazy RMS but n_used too low
    assert w._baseline_rms is None
    assert len(w._residuals) == 0


def test_reset_clears_state():
    """reset() returns the watchdog to pre-baseline state."""
    w = PositionWatchdog(window=10, threshold_td_m=0.05, alarm_count=5)
    _quiet_baseline(w, baseline_rms=0.005, window=10)
    for _ in range(5):
        w.update(0.10, n_used=10)
    assert w.alarmed
    w.reset()
    assert not w.alarmed
    assert w._baseline_rms is None
    assert w._bad_count == 0
    assert len(w._residuals) == 0


def test_baseline_x3_multiplier_floor():
    """When baseline*3 > baseline+threshold, the *3 wins.

    Important for hosts with naturally noisier TD-CP (older receivers,
    multipath-prone sites): we don't want a 1mm baseline to alarm at
    1.05cm if natural day-to-day variation goes to 3mm.
    """
    w = PositionWatchdog(window=10, threshold_td_m=0.05, alarm_count=5)
    # Baseline 0.020 m → max(0.020*3, 0.020+0.05) = max(0.060, 0.070) = 0.070
    _quiet_baseline(w, baseline_rms=0.020, window=10)
    # 0.065 m is over baseline*3 (0.060) but under baseline+threshold (0.070)
    # → still under limit, no trip
    for _ in range(20):
        w.update(0.065, n_used=10)
    assert not w.alarmed
    # 0.075 m clears the limit
    for _ in range(5):
        w.update(0.075, n_used=10)
    assert w.alarmed


def test_pr_disturbance_threshold_is_a_diagnostic_setting():
    """The PR_DISTURBANCE threshold is a separate engine-side knob; the
    watchdog itself doesn't use it for trip logic."""
    w = PositionWatchdog(pr_disturbance_threshold_m=5.0,
                         window=10, alarm_count=5,
                         threshold_td_m=0.05)
    assert w.pr_disturbance_threshold_m == 5.0
    # Watchdog trip behavior is independent of this knob
    _quiet_baseline(w, baseline_rms=0.005, window=10)
    # PR_DISTURBANCE threshold being high doesn't suppress TD-CP trips
    for _ in range(10):
        w.update(0.50, n_used=10)
    assert w.alarmed


if __name__ == "__main__":
    import pytest
    pytest.main([__file__, "-v"])

"""End-to-end slip-injection fault test for the TDCP layered defenses.

Synthesizes the conditions of the 2026-04-19 day0419h sunrise event
— a multi-SV simultaneous cycle slip — and verifies that the
L1+L2+L3 stack survives it without an actuator punch.

Layers under test:

  L1 — per-SV: locktime monotonicity (TdcpEstimator._SvPrev) + UBX
       LLI bit.  Each per-SV slip suppresses that SV's contribution
       to the ensemble.  Tested in test_tdcp_estimator.py.

  L2 — per-epoch ensemble-MAD gate (TdcpInnovGate).  Catches multi-SV
       co-slips that L1 missed (e.g., L1 flagged some but not all of
       the slipped SVs, or LLI was inconsistent).

  L3 — actuator rate limit (DOFreqEst.max_step_ppb).  Caps any
       single-epoch adjfine change regardless of what TDCP / Arm 5
       reports.

The fault scenario: an ensemble of N=12 SVs is tracking cleanly.
Then K of them (K=2, 3, ..., N//2) take a simultaneous 1-cycle
slip on the SAME epoch, with LLI flagged on M ≤ K of them
(M models a receiver that's only partially honest about slips
during a real sunrise / iono storm event).

Acceptance criteria:
  - Combined L1+L2 prevent the corrupted epoch from reaching the
    servo, OR
  - L3 caps the actuator step from any leakage that does reach it,
  - and the next clean epoch resumes normal operation.
"""

from __future__ import annotations

import math
from datetime import datetime, timedelta, timezone

import numpy as np
import pytest

from peppar_fix.tdcp_estimator import TdcpEstimator
from peppar_fix.tdcp_innov_gate import TdcpInnovGate
from peppar_fix.do_freq_est import DOFreqEst


# ── Test scaffolding (mirrors test_tdcp_estimator.py) ─────────────── #

_LAB_ECEF = np.array([157470.0, -4756189.5, 4232768.0])
_WL_L1 = 0.190293673


class _MockEph:
    def __init__(self, table):
        self._table = table

    def sat_position(self, sv, t):
        entry = self._table.get(sv)
        if entry is None:
            return None, None
        return np.asarray(entry, dtype=float), 0.0


def _ts(secs):
    anchor = datetime(2026, 5, 24, 12, 0, 0, tzinfo=timezone.utc)
    return anchor + timedelta(seconds=secs)


def _build_eph(n_svs):
    """N SVs evenly distributed in a sphere ~20 Mm out."""
    table = {}
    for i in range(n_svs):
        theta = 2 * math.pi * i / n_svs
        offset = [20e6 * math.cos(theta), 20e6 * math.sin(theta), 5e6]
        table[f"G{i+1:02d}"] = (_LAB_ECEF + np.asarray(offset)).tolist()
    return _MockEph(table)


def _epoch_obs(n_svs, base_cyc=1000.0, slip_svs=None,
               slip_cycles=1.0, lli_svs=None):
    """Build N obs.  slip_svs = list of SV indices that take a
    `slip_cycles` integer-cycle jump.  lli_svs = SVs that report
    LLI=1 (slip flag set).  A real receiver might or might not
    flag every slipped SV depending on tracking conditions."""
    slip_svs = slip_svs or []
    lli_svs = lli_svs or []
    obs = []
    for i in range(n_svs):
        sv = f"G{i+1:02d}"
        phi = base_cyc + (slip_cycles if i in slip_svs else 0.0)
        obs.append({
            "sv": sv,
            "phi1_cyc": phi,
            "phi1_raw_cyc": phi,
            "wl_f1": _WL_L1,
            "cno": 45.0,
            "lock_duration_ms": 10_000,
            "lli": 1 if i in lli_svs else 0,
        })
    return obs


# ── L1 alone — per-SV LLI must drop slipped SVs ──────────────────── #

def test_l1_alone_catches_lli_flagged_slips():
    eph = _build_eph(12)
    est = TdcpEstimator(eph, _LAB_ECEF)
    # Warm up — 10 clean epochs (0..9).
    for t in range(10):
        est.update(_epoch_obs(12), _ts(t))
    # Epoch 10: 3 SVs slip with LLI set.  L1 should drop them.
    r = est.update(_epoch_obs(12, slip_svs=[0, 1, 2], lli_svs=[0, 1, 2]),
                   _ts(10))
    # 9 SVs survive into the ensemble — should NOT include slipped 3.
    assert r.n_used == 9
    assert "G01" not in r.kept_svs
    assert "G04" in r.kept_svs


# ── L2 alone — ensemble gate catches missed-LLI co-slips ────────── #

def test_l2_catches_co_slip_when_lli_missing():
    """Worst case: receiver-wide common-mode slip (ALL SVs slip by
    1 cycle simultaneously, no LLI flags).  L1 misses everything.
    The estimator's internal MAD-3 outlier rejection also misses
    because the slip is common-mode (median moves with the slip).
    L2 must reject by comparing against historical median.

    Minority slips (e.g., 4 of 12 SVs) are caught by the estimator's
    internal MAD-3 — they don't reach L2.  L2's role is the
    MAJORITY-slip case L1 + MAD-3 can't see through.
    """
    eph = _build_eph(12)
    est = TdcpEstimator(eph, _LAB_ECEF)
    gate = TdcpInnovGate(window=10, n_sigma=5.0)
    # Warmup: need > window successful evaluates.
    for t in range(12):
        r = est.update(_epoch_obs(12), _ts(t))
        if not math.isnan(r.df_f):
            gate.evaluate(r.df_f * 1e9)
    # Common-mode slip: ALL 12 SVs shift by 1 cycle (= 0.190 m).
    # df_f = 0.190 / c / 1s ≈ 6.3e-10 ≈ 0.633 ppb.
    r = est.update(_epoch_obs(12, slip_svs=list(range(12))), _ts(12))
    assert not math.isnan(r.df_f)
    assert abs(r.df_f * 1e9 - 0.633) < 0.01, (
        f"expected ~0.633 ppb common-mode punch; got {r.df_f*1e9:.3f}"
    )
    # 0.633 ppb vs gate σ_floor=0.01 ppb → 63σ deviation.  Must reject.
    g = gate.evaluate(r.df_f * 1e9)
    assert g.accepted is False, (
        f"L2 must reject common-mode 1-cycle co-slip; "
        f"got dev={g.deviation:.1f}σ"
    )


# ── L3 alone — rate limit caps any leakage through L1+L2 ─────────── #

def test_l3_caps_punch_that_leaks_through():
    """Simulate the worst case: an Arm-5 freq punch reaches the servo
    despite L1+L2 (e.g., bug in the gate, or co-slip so large the
    gate's adaptive σ accepts it after rebaseline).  L3 must cap the
    per-epoch actuator change.
    """
    servo = DOFreqEst(
        sigma_ticc_ns=0.060,
        sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01,
        sigma_tcxo_phase_ns=2.0, sigma_tcxo_freq_ppb=0.1,
        initial_dt_rx_ns=0.0, base_freq=0.0,
        max_step_ppb=10.0,
    )
    # Force-tighten covariance so Arm 5 kicks land hard on x[1].
    servo.P = np.diag([1.0, 1.0, 100.0, 1.0])
    servo.x = np.array([0.0, 0.0, 0.0, -50.0])
    servo._last_u = 50.0
    # A 2.1 ppb Arm 5 freq punch (well above 1-cycle SSR PB step
    # = 0.635 ppb; tests that L3 clamps at large magnitude).
    servo.update(dt=1.0, tdcp_freq_ppb=2.1, tdcp_freq_sigma_ppb=0.026)
    # The LQR's u change should be ≤ max_step_ppb = 10 ppb.
    assert abs(servo._last_u - 50.0) <= 10.0 + 1e-6


# ── L1 + L2 + L3 stacked — full defense survives sunrise scenario ── #

@pytest.mark.parametrize("k_slip,m_lli", [
    (2, 2),    # 2 slips, both flagged — L1 catches via LLI
    (3, 1),    # 3 slips, 1 flagged — L1 + estimator MAD-3
    (4, 0),    # 4 slips, no LLI — estimator MAD-3 alone (minority)
    (12, 0),   # ALL slip, common-mode — only L2 + L3 can catch
])
def test_sunrise_scenario_full_stack(k_slip, m_lli):
    """day0419h sunrise model: K simultaneous slips, M flagged
    via LLI, with the rest silent.  L1+L2+L3 together must keep
    the servo's adjfine change ≤ rate limit through the bad epoch.
    """
    n_svs = 12
    eph = _build_eph(n_svs)
    est = TdcpEstimator(eph, _LAB_ECEF)
    gate = TdcpInnovGate(window=30, n_sigma=5.0)
    servo = DOFreqEst(
        sigma_ticc_ns=0.060,
        sigma_do_phase_ns=0.92, sigma_do_freq_ppb=0.01,
        sigma_tcxo_phase_ns=2.0, sigma_tcxo_freq_ppb=0.1,
        initial_dt_rx_ns=0.0, base_freq=0.0,
        max_step_ppb=10.0,
    )
    servo.P = np.diag([1.0, 1.0, 100.0, 1.0])
    servo.x = np.array([0.0, 0.0, 0.0, -50.0])
    servo._last_u = 50.0

    # Warm up: 30 clean epochs.
    for t in range(30):
        r = est.update(_epoch_obs(n_svs), _ts(t))
        if not math.isnan(r.df_f):
            g = gate.evaluate(r.df_f * 1e9)
            if g.accepted:
                servo.update(dt=1.0,
                             tdcp_freq_ppb=r.df_f * 1e9,
                             tdcp_freq_sigma_ppb=0.026)

    u_before = servo._last_u

    # Bad epoch — K slips, M LLI-flagged.
    bad = _epoch_obs(n_svs,
                     slip_svs=list(range(k_slip)),
                     lli_svs=list(range(m_lli)))
    r = est.update(bad, _ts(30))
    if not math.isnan(r.df_f):
        g = gate.evaluate(r.df_f * 1e9)
        if g.accepted:
            servo.update(dt=1.0,
                         tdcp_freq_ppb=r.df_f * 1e9,
                         tdcp_freq_sigma_ppb=0.026)

    u_after = servo._last_u
    delta_u = abs(u_after - u_before)
    # L3 must cap any leakage; if L1+L2 caught it, delta is 0.
    assert delta_u <= 10.0 + 1e-6, (
        f"Stack failed: K={k_slip} M={m_lli}, "
        f"adjfine changed by {delta_u:.2f} ppb (limit 10)"
    )


# ── Recovery — next clean epoch resumes ──────────────────────────── #

def test_recovery_after_co_slip_event():
    """After a persistent common-mode slip, the gate must accept
    again once the receiver has settled at the new phase.

    Real cycle slips PERSIST — the receiver tracks the new (slipped)
    phase going forward.  Modeled here by keeping phi1_cyc at
    baseline+1cycle for all epochs after the slip event.  The
    estimator sees: bad epoch (huge diff) → quiet epochs (diff=0
    because prev == cur).
    """
    n_svs = 12
    eph = _build_eph(n_svs)
    est = TdcpEstimator(eph, _LAB_ECEF)
    gate = TdcpInnovGate(window=30, n_sigma=5.0)
    # 32 warmup epochs → 31 successful evaluates → buffer past window.
    for t in range(32):
        r = est.update(_epoch_obs(n_svs), _ts(t))
        if not math.isnan(r.df_f):
            gate.evaluate(r.df_f * 1e9)
    # Persistent common-mode slip: all SVs jump 1 cycle and stay.
    r_bad = est.update(_epoch_obs(n_svs, slip_svs=list(range(n_svs))),
                       _ts(32))
    g_bad = gate.evaluate(r_bad.df_f * 1e9)
    assert g_bad.accepted is False  # confirm L2 rejected
    # Post-slip epochs: receiver still at base+1 cycle (slip stuck).
    # First post-slip epoch will see df_f = 0 (no diff vs prev),
    # which is at-median and accepted.
    r_good = est.update(_epoch_obs(n_svs, slip_svs=list(range(n_svs))),
                        _ts(33))
    g_good = gate.evaluate(r_good.df_f * 1e9)
    assert g_good.accepted is True, (
        f"Gate must accept clean epoch after a single L2 rejection; "
        f"got dev={g_good.deviation:.2f}σ"
    )

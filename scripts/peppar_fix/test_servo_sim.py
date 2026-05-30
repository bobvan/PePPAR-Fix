"""Tests for the closed-loop servo simulator (peppar_fix.servo_sim).

These pin the INVARIANTS the sim must hold to be a trustworthy test bed,
not the exact empirical lock-quality numbers (those depend on the coast
scheduler + fat-tailed real innovations the v1 sim models only
approximately — see docs/closed-loop-servo-sim.md "Faithfulness").
"""
import numpy as np
import pytest

from peppar_fix.servo_sim import (
    ClosedLoopSim, SimConfig, preset, run_two_clock)


def test_freerun_floor_calibrated():
    """DO noise knobs land the free-run TDEV(1s) near the per-host floor.

    Real floors: clkPoC3 45.4 ps, PiFace 54.4 ps (day0527 freerun char).
    """
    clk = ClosedLoopSim(preset("clkpoc3")).freerun_tdev_1s()
    pif = ClosedLoopSim(preset("piface-ungated")).freerun_tdev_1s()
    assert 0.030 < clk < 0.065, f"clkpoc3 freerun {clk*1e3:.1f} ps off"
    assert 0.038 < pif < 0.075, f"piface freerun {pif*1e3:.1f} ps off"
    assert pif > clk  # PiFace floor is higher than clkPoC3


def test_clean_input_locks_and_does_not_diverge():
    """The clean-input preset locks and stays bounded (sign-convention
    regression: the actuator is the NEGATED update() return; getting it
    wrong makes the loop diverge)."""
    res = ClosedLoopSim(preset("clkpoc3", duration_s=3000.0)).run()
    assert not res.diverged()
    assert res.locked()
    assert res.max_excursion_ns() < 10.0


def test_output_tdev_above_floor_but_close():
    """Disciplined output TDEV(1s) is at/above the DO floor (the servo
    can't beat the oscillator) but within a small factor of it."""
    cfg = preset("clkpoc3", duration_s=3000.0)
    sim = ClosedLoopSim(cfg)
    floor = sim.freerun_tdev_1s()
    out = sim.run().tdev_1s()
    assert out >= 0.5 * floor          # not magically below the floor
    assert out < 8.0 * floor           # not wildly degraded at coast=1


def test_gate_starves_without_x2_carrier():
    """OCXO gate (TICC-primary, no EXTINT) rejects the ns-scale TICC
    innovations and starves the loop → does NOT lock.  This is the
    clkpoc3GateOverRejectsBeforeLock finding."""
    res = ClosedLoopSim(preset("clkpoc3-gated", duration_s=3000.0)).run()
    assert res.gate_stats.get("n_rejected", 0) > 100
    assert not res.locked()


def test_gate_with_extint_carrier_still_locks():
    """Same OCXO gate, but EXTINT carries x[2]: the loop locks despite
    the gate rejecting TICC.  Isolates the real lesson — gate TICC, keep
    a non-TICC x[2] carrier (disciplineModeFsm)."""
    res = ClosedLoopSim(preset("piface-v1", duration_s=3000.0)).run()
    assert res.gate_stats.get("n_rejected", 0) > 100   # gate is active
    assert res.locked()
    assert not res.diverged()


def test_ticc_innovations_are_ns_scale():
    """With PPP fed, the median TICC innovation is ns-scale (the GNSS
    PPS-edge jitter the PPP arm can't see), matching the day0527
    captures (clkPoC3 median ~3.4 ns, PiFace ~2.2 ns) — NOT 60 ps."""
    res = ClosedLoopSim(preset("clkpoc3", duration_s=3000.0)).run()
    inn = res.ticc_innov_ns[~np.isnan(res.ticc_innov_ns)]
    med = float(np.median(np.abs(inn)))
    assert 1.0 < med < 6.0, f"median |innov| {med:.2f} ns out of band"


def test_coast_before_freq_convergence_diverges():
    """Captured failure mode: a long coast on the overconfident EKF
    diverges (√P[3,3] shrinks but x[3] stays biased).  This is the
    longTauGnssCoupling overconfidence problem, reproduced
    deterministically — the sim is the test bed for its fix."""
    res = ClosedLoopSim(preset("piface-ungated", duration_s=4000.0,
                               coast_interval_s=100.0)).run()
    assert res.diverged()


def test_actuator_quantization_is_optional_and_bounded():
    """adjfine_lsb=0 is ideal; a modest LSB still locks (regression that
    quantization doesn't destabilize a converged loop)."""
    cfg = preset("clkpoc3", duration_s=3000.0)
    cfg.adjfine_lsb_ppb = 0.073   # i226 PHC LSB
    res = ClosedLoopSim(cfg).run()
    assert res.locked()
    assert not res.diverged()


def test_two_clock_shared_gnss_runs():
    """Two-clock differential returns aligned arrays and a finite diff."""
    a = preset("clkpoc3", duration_s=2000.0)
    b = preset("clkpoc3", duration_s=2000.0)
    res_a, res_b, diff = run_two_clock(a, b, share_gnss=True)
    assert len(diff) == len(res_a.t_s) == len(res_b.t_s)
    assert np.all(np.isfinite(diff))


def test_determinism_same_seed():
    """Same seed → identical trajectory (deterministic test bed)."""
    cfg = preset("clkpoc3", duration_s=1000.0, seed=7)
    r1 = ClosedLoopSim(cfg).run()
    r2 = ClosedLoopSim(preset("clkpoc3", duration_s=1000.0, seed=7)).run()
    assert np.allclose(r1.phi_do_true_ns, r2.phi_do_true_ns)


def test_unknown_preset_raises():
    with pytest.raises(KeyError):
        preset("nonesuch")


# ── #107 binary-layer A/B harness — gross-fault detector + holdover ── #

_DROP = (300.0, 600.0)


def _binary_layer_cfg(*, in_holdover=False, enabled=True):
    """PiFace-preset short A/B: 300s of dropped measurements forces
    sustained distance_to_lock=1.0; toggle in_holdover to test
    suppression."""
    return preset(
        "piface-ungated", duration_s=800.0,
        drop_measurements_window_s=_DROP,
        in_holdover_window_s=(_DROP if in_holdover else None),
        binary_layer_enabled=enabled,
        binary_layer_consec_max_epochs=60)


def test_binary_layer_disabled_records_no_faults():
    """Feature off ⇒ no events recorded even when distance_to_lock
    saturates during the drop window."""
    sim = ClosedLoopSim(_binary_layer_cfg(enabled=False))
    sim.run()
    assert sim.gross_fault_events == []


def test_binary_layer_fires_outside_holdover():
    """Sustained distance_to_lock=1.0 outside holdover ⇒ at least one
    gross_fault, first firing after consec_max_epochs from window
    start (drop@300 + ~14s P22 saturation + 60s consec = ~374s)."""
    sim = ClosedLoopSim(_binary_layer_cfg(in_holdover=False))
    sim.run()
    assert len(sim.gross_fault_events) >= 1, "expected at least one fault"
    t_first, _p22 = sim.gross_fault_events[0]
    assert 360.0 <= t_first <= 420.0, (
        f"first fault at {t_first}s, expected ~374s post-window-start")


def test_binary_layer_suppressed_in_holdover():
    """Same drop window but with in_holdover=True for its duration ⇒
    the layer must not fire even though distance_to_lock=1.0 sustains."""
    sim = ClosedLoopSim(_binary_layer_cfg(in_holdover=True))
    sim.run()
    assert sim.gross_fault_events == [], (
        f"expected suppression, got {len(sim.gross_fault_events)} fault(s)")


def test_binary_layer_reset_preserves_actuator_command():
    """servo.reset() default-preserves the actuator command (DO keeps
    coasting at last good freq while the filter rebuilds).  Adjfine
    should be byte-identical across the reset epoch."""
    sim = ClosedLoopSim(_binary_layer_cfg(in_holdover=False))
    res = sim.run()
    assert sim.gross_fault_events, "test prerequisite: needs ≥1 fault"
    t_fault = sim.gross_fault_events[0][0]
    idx = int(t_fault / sim.cfg.dt_s)
    # Compare adjfine one step before and one step after the reset
    # epoch.  Reset preserves _last_u / freq, so the next applied
    # adjfine is the same value.
    pre = float(res.adjfine_ppb[idx - 1])
    post = float(res.adjfine_ppb[idx + 1])
    assert abs(post - pre) < 1e-9, (
        f"adjfine changed across reset: pre={pre}, post={post}")


# ── #110 router v2 A/B harness — qVIR-routed two-candidate ── #

def _v2_sole_ticc_cfg(*, mode, ext_noise, override=None):
    """Sole-TICC MadHat-like warm-start; mode in {'off','v1','v2'}."""
    cfg = SimConfig(
        label=f"{mode}-v2-ext{ext_noise}", duration_s=2000.0, seed=0,
        use_ppp=False, use_qerr=False, use_tdcp=False,
        use_extint=False, use_ticc=True,
        rx_f0_ppb=16.0,
        do_wfm_ppb=0.050, do_f0_ppb=-44.0,
        initial_freq_ppb=44.0,
        initial_phi_do_ns=0.5, initial_dt_rx_ns=0.0,
        gate_sigma_ns=None,
        ext_qerr_noise_ns=ext_noise,
        ticc_ext_correlated_override=override)
    if mode == 'v1':
        cfg.routed_qerr_enabled = True
    elif mode == 'v2':
        cfg.router_v2_enabled = True
    return cfg


def _route_counts(sim):
    from collections import Counter
    skip = 300
    routes = sim.last_routes[skip:]
    return Counter(routes.tolist())


def test_router_v2_clean_ext_routes_to_ext():
    """F9T-clean ext (0.1 ns < 1 ns) → ticc_ext_correlated=True →
    router picks 'ext' every epoch."""
    sim = ClosedLoopSim(_v2_sole_ticc_cfg(mode='v2', ext_noise=0.1))
    sim.run()
    rc = _route_counts(sim)
    assert rc.get('ext', 0) > 1000
    assert rc.get('int', 0) == 0   # v2 has no internal candidate
    assert rc.get('raw', 0) == 0


def test_router_v2_noisy_ext_routes_to_raw():
    """F10T-noisy ext (3 ns > 1 ns) → ticc_ext_correlated=False →
    router falls back to 'raw' (NEVER to v1's 'internal')."""
    sim = ClosedLoopSim(_v2_sole_ticc_cfg(mode='v2', ext_noise=3.0))
    sim.run()
    rc = _route_counts(sim)
    assert rc.get('raw', 0) > 1000
    assert rc.get('int', 0) == 0   # the v2 design guarantee
    assert rc.get('ext', 0) == 0


def test_router_v2_no_regression_vs_v1_at_clean_ext():
    """Headline regression check bravo asked for: v2 must not be worse
    than v1 in the F9T-clean regime where v1 was already optimal."""
    sim_v1 = ClosedLoopSim(_v2_sole_ticc_cfg(mode='v1', ext_noise=0.1))
    sim_v2 = ClosedLoopSim(_v2_sole_ticc_cfg(mode='v2', ext_noise=0.1))
    res_v1 = sim_v1.run(); res_v2 = sim_v2.run()
    inn_v1 = res_v1.ticc_innov_ns[~np.isnan(res_v1.ticc_innov_ns)]
    inn_v2 = res_v2.ticc_innov_ns[~np.isnan(res_v2.ticc_innov_ns)]
    rms_v1 = float(np.sqrt((inn_v1**2).mean()))
    rms_v2 = float(np.sqrt((inn_v2**2).mean()))
    # Allow tiny float drift; v2 should be within 10% of v1's RMS.
    assert rms_v2 < rms_v1 * 1.1, f"v2 regressed: {rms_v2} vs v1 {rms_v1}"


def test_router_v2_beats_v1_in_noisy_regime():
    """The structural win: dropping 'internal' means v2's worst-case
    fallback ('raw') is better than v1's worst-case fallback ('int'
    via rx-TCXO leakage)."""
    sim_v1 = ClosedLoopSim(_v2_sole_ticc_cfg(mode='v1', ext_noise=3.0))
    sim_v2 = ClosedLoopSim(_v2_sole_ticc_cfg(mode='v2', ext_noise=3.0))
    res_v1 = sim_v1.run(); res_v2 = sim_v2.run()
    inn_v1 = res_v1.ticc_innov_ns[~np.isnan(res_v1.ticc_innov_ns)]
    inn_v2 = res_v2.ticc_innov_ns[~np.isnan(res_v2.ticc_innov_ns)]
    rms_v1 = float(np.sqrt((inn_v1**2).mean()))
    rms_v2 = float(np.sqrt((inn_v2**2).mean()))
    assert rms_v2 < rms_v1, (
        f"expected v2 ({rms_v2}) better than v1 ({rms_v1}) in noisy regime")


def test_router_v2_override_forces_decision():
    """ticc_ext_correlated_override bypasses the noise-magnitude heuristic
    so tests can pin the router to a specific branch."""
    sim = ClosedLoopSim(_v2_sole_ticc_cfg(
        mode='v2', ext_noise=0.1, override=False))   # clean but force not-correlated
    sim.run()
    rc = _route_counts(sim)
    assert rc.get('raw', 0) > 1000   # forced to raw despite clean ext
    assert rc.get('ext', 0) == 0


def test_router_v1_v2_mutual_exclusion_rejected():
    """SimConfig must reject v1+v2 simultaneously (engine-level invariant
    reflected at config-construction time so the operator gets a clear
    error instead of an ambiguous routing path)."""
    cfg = _v2_sole_ticc_cfg(mode='v2', ext_noise=0.1)
    cfg.routed_qerr_enabled = True   # v2 already enabled
    with pytest.raises(ValueError, match="mutually exclusive"):
        SimConfig(**{k: getattr(cfg, k) for k in cfg.__dataclass_fields__})

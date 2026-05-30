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

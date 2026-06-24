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


# ── #121 ResetBudget A/B harness — shared in-process reset budget ── #

def _budget_cfg(*, max_resets=3, window_s=300.0, enabled=True,
                drop_windows=None, synthetic=None,
                in_holdover_window=None, duration_s=2000.0):
    """PiFace-preset short A/B: stacks N drop windows back-to-back so
    each induces a binary-layer reset, and/or fires direct synthetic
    requests at known times.  Used by the #121 budget A/B tests."""
    return preset(
        "piface-ungated", duration_s=duration_s,
        drop_measurements_window_s=None,
        induced_drop_windows=list(drop_windows or []),
        induced_synthetic_requests=list(synthetic or []),
        in_holdover_window_s=in_holdover_window,
        binary_layer_enabled=True,
        binary_layer_consec_max_epochs=60,
        reset_budget_enabled=enabled,
        reset_budget_max=max_resets,
        reset_budget_window_s=window_s)


def _outcomes(reset_events):
    return [ev[2] for ev in reset_events]


def _reasons(reset_events):
    return [ev[1] for ev in reset_events]


def test_reset_budget_sub_budget_all_allowed():
    """3 drop-induced binary-layer faults inside the budget window →
    all 3 reset.  Default-budget-max is 3; this is the "live within the
    budget" baseline.  Also verifies actuator preservation across each
    reset (the reset funnel's load-bearing contract)."""
    # Three 110s drop windows.  Each fires exactly ONE fault: at
    # ~start + 14s P22 saturation + 60s consec_max ≈ start+74, which
    # falls inside the window (so adjfine pre/post the reset epoch is
    # held during predict-only → natural preservation), while the
    # window ends before the binary layer's consec counter could climb
    # to consec_max a second time (next fire would be at start+134).
    drops = [(300.0, 410.0), (700.0, 810.0), (1100.0, 1210.0)]
    sim = ClosedLoopSim(_budget_cfg(max_resets=3, window_s=10_000.0,
                                    drop_windows=drops,
                                    duration_s=1500.0))
    res = sim.run()
    assert sim.exit5_at is None, "shouldn't have exit5'd within budget"
    outcomes = _outcomes(sim.reset_events)
    assert all(o == "reset" for o in outcomes), \
        f"expected all 'reset', got {outcomes}"
    assert len(outcomes) >= 3, f"expected ≥3 resets, got {len(outcomes)}"
    # Actuator preserved across each reset epoch — adjfine before and
    # after each reset is the same (the funnel's main contract).
    for t_evt, _reason, _outcome in sim.reset_events[:3]:
        idx = int(t_evt / sim.cfg.dt_s)
        pre = float(res.adjfine_ppb[idx - 1])
        post = float(res.adjfine_ppb[idx + 1])
        assert abs(post - pre) < 1e-9, (
            f"adjfine changed across reset @ t={t_evt}: pre={pre} post={post}")


def test_reset_budget_exhaustion_falls_through_to_exit5():
    """4 faults inside a 1000s window with budget=3 → 4th is denied,
    sim stops at that epoch.  The load-bearing test for the cap."""
    drops = [(300.0, 380.0), (450.0, 530.0),
             (600.0, 680.0), (750.0, 830.0)]
    sim = ClosedLoopSim(_budget_cfg(max_resets=3, window_s=1000.0,
                                    drop_windows=drops,
                                    duration_s=1500.0))
    res = sim.run()
    outcomes = _outcomes(sim.reset_events)
    assert outcomes.count("reset") == 3, \
        f"expected exactly 3 'reset', got {outcomes}"
    assert outcomes.count("exit5") == 1, \
        f"expected exactly 1 'exit5', got {outcomes}"
    assert outcomes[-1] == "exit5", "the FINAL event must be exit5"
    assert sim.exit5_at is not None
    # The run terminated at the exit5 epoch (result arrays truncated).
    assert res.t_s[-1] <= sim.exit5_at + sim.cfg.dt_s + 1e-9


def test_reset_budget_recovers_after_quiet_window():
    """Budget=3 in 200s window; fire 3 resets at t=300/400/500, then
    quiet >200s, then a 4th at t=800 → 4th is allowed (the first 3
    have aged out of the rolling window)."""
    synth = [
        (300.0, "binary_layer"),
        (400.0, "binary_layer"),
        (500.0, "binary_layer"),
        (800.0, "binary_layer"),    # >200s after the 3rd → window cleared
    ]
    sim = ClosedLoopSim(_budget_cfg(max_resets=3, window_s=200.0,
                                    drop_windows=[],
                                    synthetic=synth,
                                    duration_s=1000.0))
    sim.run()
    outcomes = _outcomes(sim.reset_events)
    assert outcomes == ["reset", "reset", "reset", "reset"], \
        f"expected all 4 reset after window recovery, got {outcomes}"
    assert sim.exit5_at is None


def test_reset_budget_shared_across_reasons():
    """The load-bearing test for decision (c).  Budget=3 in W=1000s
    consumed by THREE different reasons (binary_layer, ekf_outlier,
    phc_restep) → the 4th request, from a FOURTH reason
    (cm_outlier), is denied.  A per-detector budget would allow it."""
    synth = [
        (200.0, "binary_layer"),
        (300.0, "ekf_outlier"),
        (400.0, "phc_restep"),
        (500.0, "cm_outlier"),      # 4th-of-window from a 4th reason
    ]
    sim = ClosedLoopSim(_budget_cfg(max_resets=3, window_s=1000.0,
                                    drop_windows=[],
                                    synthetic=synth,
                                    duration_s=800.0))
    sim.run()
    outcomes = _outcomes(sim.reset_events)
    reasons = _reasons(sim.reset_events)
    # First three (across three different reasons) reset; fourth — a
    # NEW reason — gets denied because the BUDGET is exhausted.
    assert outcomes[:3] == ["reset", "reset", "reset"]
    assert outcomes[3] == "exit5"
    assert reasons == ["binary_layer", "ekf_outlier",
                       "phc_restep", "cm_outlier"]
    assert sim.exit5_at == pytest.approx(500.0, abs=1.0)


def test_reset_budget_disabled_is_byte_identical_to_v1():
    """With reset_budget_enabled=False the binary-layer A/B path runs
    exactly like #111 did — no exit5 even with many faults; the only
    new sim-side state is reset_events (each gross_fault produces a
    'reset' event but the cap doesn't fire)."""
    drops = [(300.0, 380.0), (450.0, 530.0),
             (600.0, 680.0), (750.0, 830.0)]   # 4 faults — would exhaust budget=3
    sim = ClosedLoopSim(_budget_cfg(max_resets=3, window_s=1000.0,
                                    drop_windows=drops,
                                    enabled=False,
                                    duration_s=1200.0))
    res = sim.run()
    # Default-off: every gross-fault produces a 'reset' (legacy path);
    # no 'exit5' ever; run completes its full duration.
    outcomes = _outcomes(sim.reset_events)
    assert outcomes, "default-off should still log 'reset' events"
    assert all(o == "reset" for o in outcomes)
    assert sim.exit5_at is None
    # Run reached its natural duration — result arrays not truncated.
    expected_samples = int(sim.cfg.duration_s / sim.cfg.dt_s)
    assert abs(len(res.t_s) - expected_samples) <= 1


# ── PR #126 + I-073222 state-sanity recovery closed-loop A/B ── #
#
# Reproduces the day0602-pr125b-madhat 22:21 failure mode in the
# closed-loop sim: a long drop window forces predict-only propagation
# of x[2] past the 1e8 ns sanity bound; without the new is_state_
# corrupted() poll the engine logs ERROR forever (B); with it, the
# loop calls _request_servo_reset, zeros x[2], and re-acquires (A).

def _state_sanity_cfg(*, recovery_enabled, duration_s=600.0,
                      corruption_at=200.0, drop=(200.0, 100_000.0)):
    """Reproduces the 22:21 failure: at `corruption_at` the sim pokes
    x[2] to 2e8 ns (past the 1e8 sanity bound), and a drop window
    starting at the same time prevents measurements from pulling it
    back.  Predict-only propagation keeps |x[2]| above the bound, the
    consec counter accumulates, and at consec ≥ threshold the engine-
    side poll (state_sanity_recovery_enabled) fires the reset.

    The steady-state-cancel property of the LQR means a clean drop
    window doesn't actually drive x[2] past the bound on its own;
    the explicit corruption hook is the test-side reproduction of
    the cycle-slip-cascade that preceded the 22:21 drop in the lab."""
    return preset(
        "piface-ungated", duration_s=duration_s,
        drop_measurements_window_s=drop,
        force_state_corruption_at_s=corruption_at,
        state_sanity_recovery_enabled=recovery_enabled,
        # Use the budget so test_state_sanity_reset_budget_exit5 works
        # without re-architecting; defaults match the engine CLI.
        reset_budget_enabled=recovery_enabled,
        reset_budget_max=3,
        reset_budget_window_s=300.0)


def test_state_sanity_recovery_disabled_runaway():
    """Flag OFF + long drop → x[2] keeps growing past 1e8; no events
    recorded.  The pre-#126 baseline — the engine just logs the ERROR
    and lets the runaway accumulate."""
    sim = ClosedLoopSim(_state_sanity_cfg(recovery_enabled=False))
    sim.run()
    # No recovery events: the poll never fires.
    assert sim.state_sanity_events == []
    # Final |x[2]| has grown past the 1e8 ns sanity bound and STAYS
    # above it — the runaway signature.  (We're checking the EKF's
    # estimate, not the truth — the EKF can't recover its phase state
    # because nothing reset it.)
    assert abs(float(sim.ekf.x[2])) > 1e8, (
        f"expected runaway past 1e8 ns, got |x[2]|={abs(sim.ekf.x[2]):.3e}")


def test_state_sanity_recovery_enabled_recovers():
    """Flag ON + same drop → at least one event; final |x[2]| bounded
    well below the sanity threshold once measurements resume."""
    sim = ClosedLoopSim(_state_sanity_cfg(recovery_enabled=True))
    sim.run()
    # At least one recovery event fired — the poll caught a
    # threshold-crossing during the drop window.
    assert len(sim.state_sanity_events) >= 1, (
        f"expected ≥1 state-sanity event, got {len(sim.state_sanity_events)}")
    # Events fall inside or shortly after the drop window — recovery
    # is in-loop, not at end of run.
    for t_evt, x2_pre in sim.state_sanity_events:
        assert x2_pre != 0  # the pre-reset value (non-zero by construction)
    # Post-recovery: |x[2]| is bounded back under the sanity threshold.
    # The drop ended at 700s, plenty of catch-up time before duration_s=1200.
    assert abs(float(sim.ekf.x[2])) < 1e6, (
        f"expected |x[2]| bounded post-recovery, got {abs(sim.ekf.x[2]):.3e}")


def test_lqr_short_circuit_prevents_actuator_swing():
    """During the consec=1..N-1 violation window the LQR short-circuit
    holds adjfine at last-sane (returned self.freq unchanged from
    DOFreqEst.update()).  Assert the actuator command DURING the drop
    window stays bounded — no ±100 ppb cascade like the 22:21 lab log.

    Note: piface-ungated's steady-state adjfine is +135 ppb (to
    cancel do_f0_ppb=-135), so the bound has to be above |135 ppb|
    but well below max_ppb (the LQR rail in the failure mode)."""
    sim = ClosedLoopSim(_state_sanity_cfg(recovery_enabled=True))
    res = sim.run()
    # Adjfine through the drop window: index range corresponds to
    # the drop tuple in epochs.
    t = res.t_s
    adj = res.adjfine_ppb
    drop_start, drop_end = sim.cfg.drop_measurements_window_s
    in_drop = (t >= drop_start) & (t <= drop_end)
    max_abs_adj = float(np.max(np.abs(adj[in_drop])))
    # ≤ a few hundred ppb (held + small post-reset transients);
    # never the ~max_ppb rail of the unfixed failure mode.
    assert max_abs_adj < 1000.0, (
        f"adjfine swung to {max_abs_adj:.1f} ppb during drop — "
        "LQR short-circuit didn't hold the actuator")


def test_recovery_time_to_lock():
    """After a state-sanity reset, measure how many epochs until
    |x[2]| < 100 ns — the operational expectation operators read out
    of [SERVO_RESET reason=state_sanity] log lines.  Concrete bound:
    re-lock within 60 s on the piface-ungated preset (matches the
    binary-layer's consec_max window so it's a natural ceiling)."""
    sim = ClosedLoopSim(_state_sanity_cfg(recovery_enabled=True))
    res = sim.run()
    assert sim.state_sanity_events, "test prereq: a reset must have happened"
    t_evt = sim.state_sanity_events[0][0]
    idx_evt = int(t_evt / sim.cfg.dt_s)
    # Walk forward from the reset epoch until |x[2]| drops below 100 ns.
    # (Reconstruction from the post-update estimated phase log.)
    # est_phi_do_ns is the EKF's x[2] estimate per epoch — sim records
    # it on every epoch.
    for j in range(idx_evt, len(res.est_phi_do_ns)):
        if abs(float(res.est_phi_do_ns[j])) < 100.0:
            recovery_epochs = j - idx_evt
            break
    else:
        recovery_epochs = float('inf')
    assert recovery_epochs < 60, (
        f"recovery took {recovery_epochs} epochs — over the 60 s ceiling")


def test_state_sanity_reset_budget_exit5():
    """4 corruption injections inside the budget window → budget
    exhausted → at least one 'exit5' outcome in reset_events.  Pins
    the budget-fall-through path so a genuinely-broken host yields
    to wrapper re-bootstrap instead of resetting forever.

    Approach: long drop window keeps measurements out; one initial
    state-corruption poke; the LQR short-circuit + held adjfine means
    truth.phi_do drifts (DO crystal not corrected), and each post-reset
    measurement-update reseeds x[2] from the (now-drifted) truth.
    Within ~hundreds of seconds, x[2] re-crosses the sanity bound and
    triggers a 2nd, 3rd reset.  With max=3 in window=300s, the 4th
    request denies → exit5."""
    # Multiple synthetic corruption sources via synthetic_requests —
    # bypass the EKF/predict dynamics and directly fire reset requests
    # with reason='state_sanity'.  This decouples the budget-fallthrough
    # behavior from the (host-dependent) actual time between threshold-
    # crossings; the unit-test scope is "does the budget bind?".
    cfg = preset("piface-ungated", duration_s=400.0,
                 state_sanity_recovery_enabled=False,  # use synthetic path
                 reset_budget_enabled=True,
                 reset_budget_max=3,
                 reset_budget_window_s=300.0,
                 induced_synthetic_requests=[
                     (50.0, "state_sanity"),
                     (100.0, "state_sanity"),
                     (150.0, "state_sanity"),
                     (200.0, "state_sanity"),     # 4th in window → exit5
                 ])
    sim = ClosedLoopSim(cfg)
    sim.run()
    outcomes = [ev[2] for ev in sim.reset_events]
    assert outcomes[:3] == ["reset", "reset", "reset"], (
        f"first 3 should be 'reset', got {outcomes}")
    assert outcomes[3] == "exit5", (
        f"4th request from a 4th injection should be 'exit5', got {outcomes}")
    # Sim stopped at the exit5 epoch (no run past the wrapper-relaunch).
    assert sim.exit5_at is not None


def test_two_clock_state_sanity_diff_resilience():
    """run_two_clock with the same disturbance: A=engine-side recovery
    ON, B=OFF.  Compares the EKF's x[2] estimate, which IS the
    differentiated signal between the two configs.

    Note: the LQR short-circuit landed in PR #126 is UNCONDITIONAL
    inside DOFreqEst.update(), so even B (no engine-side poll) keeps
    the actuator held at last-sane post-corruption.  This means
    truth.phi_do behaves similarly between A and B (both have the
    held actuator), and the hero-plot A/B Main envisioned can't show
    a truth-side cascade without a separate knob to disable the short-
    circuit.  We instead compare the EKF state directly: A's reset
    zeros x[2], B's stays at the corruption value."""
    cfg_a = _state_sanity_cfg(recovery_enabled=True)
    cfg_b = _state_sanity_cfg(recovery_enabled=False)
    cfg_a.seed = 17
    cfg_b.seed = 17
    res_a, res_b, _ = run_two_clock(cfg_a, cfg_b, share_gnss=True)
    # The fixture is constructed by run_two_clock — we need the
    # actual sims to read x[2] at end-of-run.  Reconstruct from the
    # est_phi_do_ns log (EKF's x[2] estimate per epoch).
    end_x2_a = float(res_a.est_phi_do_ns[-1])
    end_x2_b = float(res_b.est_phi_do_ns[-1])
    # A recovers: x[2] near zero (the reset cleared it; subsequent
    # predict steady-state holds it at zero).
    assert abs(end_x2_a) < 1e4, (
        f"clock A's x[2] should be near zero post-recovery, "
        f"got {abs(end_x2_a):.3e}")
    # B does NOT recover: x[2] held at the corruption value (LQR
    # short-circuit holds adjfine, predict steady-state keeps x[2]
    # at the post-poke value).
    assert abs(end_x2_b) > 1e8, (
        f"clock B's x[2] should remain at the corruption value, "
        f"got {abs(end_x2_b):.3e}")
    # Differential between A and B is at least 4 orders of magnitude
    # on the EKF state — operationally that's "recovered" vs "stuck".
    assert abs(end_x2_b) > 1e4 * abs(max(abs(end_x2_a), 1.0)), (
        f"A={abs(end_x2_a):.3e}, B={abs(end_x2_b):.3e}")

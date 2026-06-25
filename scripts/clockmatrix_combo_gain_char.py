#!/usr/bin/env python3
"""Combo-gain characterization — measure a ClockMatrix combo-bus actuator's
realized/naive frequency ratio and write [steering].combo_gain into the DO's
per-section schema (state/dos/<uid>.toml).

The combo sibling of do_steering_char.py.  Where the DAC tool measures an EFC
slope (ppb/code), this measures the **combo gain**: the ratio of the realized
DCO frequency shift to the naive datasheet value written to COMBO_SW_VALUE.
That ratio is per-chip, so — per Bob's no-default-actuator-gain rule — the
engine REFUSES to discipline a ClockMatrix_combo DO until it is measured
(do_char_resolve.should_refuse_for_steering + resolve_combo_actuator_params).

NOTE (otcBob1 DPLL3, 2026-06-25): the steady-state gain measures ≈ 1.0 — the
naive datasheet scaling is essentially correct.  The PoC's "~0.7×" was an
artifact of short measurement windows + PHASE_STATUS glitch reads (see
fit_phase_slope), not a real frequency discount.  Still measured per host
because the ratio is in principle per-chip and the rule forbids a default.

Method (on-chip, no external gear):
  - Leave DPLL3 in PLL/auto so PHASE_STATUS (DCO-vs-F9T-PPS) is live.
  - For each *naive* combo step (the actuator runs at combo_gain=1.0, so a
    commanded ppb is written verbatim), hold it and sample PHASE_STATUS over
    a window; the phase RAMP rate = the DCO frequency offset it induced.
    realized_ppb = −d(phase_ns)/dt  (sign: +combo ⇒ DO faster ⇒ phase ramps
    negative — the validated convention).
  - Fit realized = gain·naive + offset.  The SLOPE is combo_gain; the
    INTERCEPT absorbs the OCXO's free-running offset (so it doesn't bias the
    gain — better than a through-origin fit).

This measures the SAME chip it steers (the on-chip phase readback), which is
legitimate for a *gain* measurement — the F9T-PPS reference noise averages out
over a multi-second ramp.  It does NOT validate servo QUALITY; that needs the
independent TICC#4 cross-check (platform plan P4).

Crash-safe: teardown() restores the saved registers — handing the DO back to
the hardware DPLL — on clean exit, Ctrl-C, SIGTERM, SIGHUP (a dropped SSH
connection — observed on ptBoat 2026-06-25), or any exception.  (SIGKILL /
power-loss can't be caught here; a hardware-side revert is the P3/P4 gate.)

Run on the host (needs I2C; stops timebeat for exclusive ClockMatrix access):

    sudo ~/peppar-fix/venv/bin/python \\
        ~/peppar-fix/scripts/clockmatrix_combo_gain_char.py \\
        --uid otc-otcbob1 --bus 15 --addr 0x58 --dpll 3

--dry-run measures + fits + validates but does not write the .toml.
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from peppar_fix import do_schema  # noqa: E402
from peppar_fix.clockmatrix import ClockMatrixI2C  # noqa: E402
from peppar_fix.clockmatrix_combo_actuator import ClockMatrixComboActuator  # noqa: E402
from peppar_fix.clockmatrix_phase import ClockMatrixPhaseSource  # noqa: E402


def _utcnow() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# ── Pure fit helpers (hardware-free, unit-tested) ──────────────────── #


def ramp_to_realized_ppb(phase_slope_ns_per_s: float) -> float:
    """Realized DCO frequency offset (ppb) from a PHASE_STATUS ramp rate.

    1 ppb = 1e-9 s/s = 1 ns/s.  Sign: +combo ⇒ DO faster ⇒ phase ramps
    negative, so realized_ppb = −slope.
    """
    return -phase_slope_ns_per_s


def fit_phase_slope(times_s, phases_ns) -> float:
    """Robust (Theil–Sen) slope (ns/s) of phase vs time.

    PHASE_STATUS occasionally returns a GLITCH read — a value near 0 while the
    true ramped phase is far from 0 (observed ~every 5 s on otcBob1 DPLL3, a
    register-update race on the I2C read).  Least-squares is destroyed by these
    (a few ~0 points among a ramp to thousands of ns drag the slope toward 0 —
    the cause of the spurious "gain≈0" over long windows).  Theil–Sen — the
    median of all pairwise slopes — ignores up to ~29% such outliers, so the
    median-of-3-per-sample plus Theil–Sen-across-samples recovers the true
    steady ramp rate.
    """
    t = np.asarray(times_s, dtype=float)
    p = np.asarray(phases_ns, dtype=float)
    n = len(t)
    if n < 2:
        raise ValueError("need >= 2 phase samples to fit a ramp")
    # All-pairs slopes (n≈21-31 → a few hundred pairs, trivial).
    ti, tj = np.triu_indices(n, k=1)
    dt = t[tj] - t[ti]
    good = dt > 0
    if not np.any(good):
        raise ValueError("phase samples have no time separation")
    slopes = (p[tj] - p[ti])[good] / dt[good]
    return float(np.median(slopes))


def fit_combo_gain(points) -> dict:
    """Fit realized = gain·naive + offset over [(naive_ppb, realized_ppb), ...].

    Returns {combo_gain, freerun_offset_ppb, rmse_ppb, r2, n}.  The slope is
    the combo gain (realized/naive); the intercept is the OCXO free-running
    offset (combo=0 contribution removed), reported as a diagnostic.
    """
    naive = np.asarray([p[0] for p in points], dtype=float)
    realized = np.asarray([p[1] for p in points], dtype=float)
    if len(naive) < 2:
        raise ValueError("need >= 2 sweep points to fit combo gain")
    slope, intercept = np.polyfit(naive, realized, 1)
    pred = slope * naive + intercept
    resid = realized - pred
    rmse = float(np.sqrt(np.mean(resid ** 2)))
    ss_res = float(np.sum(resid ** 2))
    ss_tot = float(np.sum((realized - realized.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0
    return {"combo_gain": float(slope),
            "freerun_offset_ppb": float(intercept),
            "rmse_ppb": rmse, "r2": r2, "n": len(naive)}


def combo_steering_fields(fit: dict, naive_steps, measure_s: int,
                          settle_s: int, temps: dict | None = None) -> dict:
    """Map a fit_combo_gain result → schema [steering] fields.

    combo_gain is THE field the engine reads (resolve_combo_actuator_params).
    The rest are provenance: how the gain was measured, so a future reader can
    judge it.  source is added by the schema writer, not here.
    """
    fields = {
        "combo_gain": float(fit["combo_gain"]),
        "combo_freerun_offset_ppb": float(fit["freerun_offset_ppb"]),
        "rmse_ppb": float(fit["rmse_ppb"]),
        "r2": float(fit["r2"]),
        "n_points": int(fit["n"]),
        "naive_steps_ppb": ",".join(str(s) for s in naive_steps),
        "measure_s": int(measure_s),
        "settle_s": int(settle_s),
        "measured_at": _utcnow(),
    }
    if temps:
        fields.update(temps)
    return fields


# ── Hardware sweep ─────────────────────────────────────────────────── #


def measure_ramp(phase_reader, measure_s: int, *, log=print) -> tuple[float, int]:
    """Sample PHASE_STATUS for measure_s seconds; return (slope_ns_per_s, n)."""
    t0 = time.monotonic()
    times, phases = [], []
    while True:
        t = time.monotonic() - t0
        # median-of-3 to reject single-read I2C glitches (matches the PoC phm()).
        reads = sorted(phase_reader.read_phase_ns() for _ in range(3))
        times.append(t)
        phases.append(reads[1])
        if t >= measure_s:
            break
        time.sleep(1.0)
    return fit_phase_slope(times, phases), len(times)


def sweep(actuator, phase_reader, naive_steps, settle_s, measure_s, *,
          baseline=True, log=print):
    """Step the combo through naive_steps; measure the realized ramp at each.

    With ``baseline=True`` (default) each step is BRACKETED by a combo=0
    measurement and the realized offset is the DIFFERENCE
    (realized = realized(naive) − realized(0)).  This removes the OCXO's
    free-running drift, which — because BW≈0 leaves the DCO undisciplined
    during the sweep — otherwise contaminates each step's ramp by a
    time-varying amount a single fit intercept cannot absorb.  (Observed
    2026-06-25 on otcBob1: un-bracketed per-step gain scattered 0.59–1.0;
    the drift, not the combo response, was the variance.)

    Returns [{naive_ppb, slope_ns_per_s, base_slope_ns_per_s, realized_ppb, n}].
    The caller owns actuator setup/teardown + the hand-back-on-exit finally.
    """
    results = []
    for i, naive in enumerate(naive_steps):
        base_slope = 0.0
        if baseline:
            log(f"[{i + 1:2d}/{len(naive_steps)}] baseline (combo=0): "
                f"settle {settle_s}s, measure {measure_s}s")
            actuator.adjust_frequency_ppb(0.0)
            time.sleep(settle_s)
            base_slope, _ = measure_ramp(phase_reader, measure_s, log=log)
        log(f"[{i + 1:2d}/{len(naive_steps)}] naive={naive:+.0f} ppb: "
            f"settle {settle_s}s, measure {measure_s}s")
        actuator.adjust_frequency_ppb(naive)  # gain=1.0 → writes naive verbatim
        time.sleep(settle_s)
        slope, n = measure_ramp(phase_reader, measure_s, log=log)
        realized = ramp_to_realized_ppb(slope) - ramp_to_realized_ppb(base_slope)
        gain_note = f", gain {realized / naive:+.3f}" if naive else ""
        log(f"    -> ramp {slope:+.1f} ns/s (baseline {base_slope:+.1f}) "
            f"realized {realized:+.2f} ppb{gain_note} (n={n})")
        results.append({"naive_ppb": naive, "slope_ns_per_s": slope,
                        "base_slope_ns_per_s": base_slope,
                        "realized_ppb": realized, "n": n})
    return results


def _read_temps(bus_num: int) -> dict:
    """Best-effort char temps (reuses do_steering_char.read_char_temps)."""
    try:
        from do_steering_char import read_char_temps
        return read_char_temps(bus_num)
    except Exception:  # noqa: BLE001
        return {}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uid", required=True,
                    help="DO uid (matches <uid>.toml; MAC/path uids sanitized)")
    ap.add_argument("--dos-dir", default=None, help="state/dos dir override")
    ap.add_argument("--bus", type=int, default=15,
                    help="I2C bus (default 15 = OTC; Mini = 16)")
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=0x58,
                    help="ClockMatrix I2C address (default 0x58)")
    ap.add_argument("--dpll", type=int, default=3, choices=[0, 1, 2, 3],
                    help="DO DPLL (default 3 = otcBob1/ptBoat OCXO)")
    ap.add_argument("--naive-steps", default="-300,-200,-100,100,200,300",
                    help="Comma-separated naive combo ppb steps to sweep")
    ap.add_argument("--settle-s", type=int, default=5,
                    help="Seconds after each combo write before measuring")
    ap.add_argument("--measure-s", type=int, default=30,
                    help="Seconds of PHASE_STATUS ramp measurement per step")
    ap.add_argument("--no-baseline", action="store_true",
                    help="Do NOT bracket each step with a combo=0 baseline "
                         "(faster, but free-run drift contaminates the gain)")
    ap.add_argument("--no-manage-timebeat", action="store_true",
                    help="Do NOT stop/restart timebeat (caller already has the bus)")
    ap.add_argument("--dry-run", action="store_true",
                    help="measure + fit + validate, but do NOT write the .toml")
    args = ap.parse_args(argv)

    # Fail fast before any hardware: the DO must be registered, and ideally a
    # ClockMatrix_combo actuator (warn, don't hard-fail, if the type differs).
    char_path = do_schema._char_path(args.uid, args.dos_dir)
    if not os.path.exists(char_path):
        ap.error(f"no characterization file at {char_path} — register the DO "
                 f"(scripts/do_register.py / migrate_do_state.py) first.")
    try:
        _c = do_schema.load_do_characterization(args.uid, dos_dir=args.dos_dir)
        atype = _c.identity.get("actuator_type")
        if atype != "ClockMatrix_combo":
            print(f"WARNING: {args.uid} actuator_type={atype!r}, expected "
                  f"'ClockMatrix_combo' — the engine reads combo_gain only for "
                  f"ClockMatrix_combo DOs.")
    except do_schema.SchemaError as e:
        ap.error(str(e))

    naive_steps = [float(s) for s in args.naive_steps.split(",")]
    if len(naive_steps) < 3:
        ap.error("need >= 3 naive steps to fit + validate a gain (a 2-point fit "
                 "is trivially R²=1.0 and can't catch a bad DPLL state)")
    manage_tb = not args.no_manage_timebeat

    # Turn async kills into an exception so the finally hands the DO back to
    # the hardware DPLL + restarts timebeat.  SIGHUP matters specifically:
    # running over a bare `ssh host '...'` (no setsid/nohup), a dropped SSH
    # connection delivers SIGHUP, whose DEFAULT action terminates the process
    # WITHOUT running finally — leaving DPLL3 mid-combo with timebeat stopped.
    # (Observed 2026-06-25: ptBoat ssh dropped mid-sweep.)  SIGKILL/power-loss
    # still can't be caught — that's the hardware-revert gate for P3/P4.
    def _on_kill(signum, frame):
        raise KeyboardInterrupt(signal.Signals(signum).name)
    for _sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(_sig, _on_kill)

    if manage_tb:
        print("stopping timebeat for exclusive ClockMatrix access ...")
        subprocess.run(["sudo", "systemctl", "stop", "timebeat"], check=False)
        time.sleep(1.5)

    i2c = ClockMatrixI2C(args.bus, args.addr)
    actuator = ClockMatrixComboActuator(i2c, dpll_id=args.dpll, combo_gain=1.0)
    # Read PHASE_STATUS on the same DPLL WITHOUT reconfiguring it — setup() is
    # not called, so the DPLL keeps timebeat's PLL/auto state (phase live).
    phase_reader = ClockMatrixPhaseSource(i2c, dpll_id=args.dpll)

    print(f"Combo-gain char for {args.uid}: {len(naive_steps)} steps, "
          f"DPLL{args.dpll}, ~{len(naive_steps) * (args.settle_s + args.measure_s) / 60:.1f} min")
    try:
        try:
            actuator.setup()
            results = sweep(actuator, phase_reader, naive_steps,
                            args.settle_s, args.measure_s,
                            baseline=not args.no_baseline)
        finally:
            try:
                actuator.teardown()  # hands the DO back to the hardware DPLL
            except Exception as e:  # noqa: BLE001
                print(f"  WARNING: teardown failed: {e}")
            i2c.close()
            if manage_tb:
                subprocess.run(["sudo", "systemctl", "start", "timebeat"],
                               check=False)
                print("timebeat restarted; DO back on the hardware DPLL")
    except KeyboardInterrupt as e:
        # SIGTERM/SIGHUP/Ctrl-C: the finally above already restored the
        # hardware DPLL + timebeat — exit cleanly, write nothing partial.
        print(f"\n*** interrupted ({e}) — DO handed back to the hardware DPLL, "
              f"no [steering] written.")
        return 130

    points = [(r["naive_ppb"], r["realized_ppb"]) for r in results]
    fit = fit_combo_gain(points)
    print(f"\ncombo_gain = {fit['combo_gain']:+.4f}  "
          f"(freerun offset {fit['freerun_offset_ppb']:+.2f} ppb, "
          f"rmse {fit['rmse_ppb']:.2f} ppb, R²={fit['r2']:.4f}, n={fit['n']})")
    if abs(fit["combo_gain"]) < 0.1:
        print("*** combo_gain ~0 — the combo path is not steering the DCO "
              "(check SLAVE0=0x28 / BW≈0 / DPLL).  No [steering] written.")
        return 1
    if fit["r2"] < 0.9:
        print(f"*** poor linearity (R²={fit['r2']:.3f} < 0.9) — noisy ramp or "
              f"nonlinear combo response.  No [steering] written.")
        return 1

    fields = combo_steering_fields(fit, naive_steps, args.measure_s,
                                   args.settle_s, temps=_read_temps(args.bus))

    if args.dry_run:
        import tomllib
        with open(char_path, "rb") as f:
            data = tomllib.load(f)
        data["steering"] = {"source": "measured", **fields}
        errs = do_schema.validate_characterization(data)
        print("DRY-RUN: would validate clean (not written)." if not errs
              else f"DRY-RUN: would FAIL validation: {'; '.join(errs)}")
        return 0 if not errs else 1

    path = do_schema.update_characterization_section(
        args.uid, "steering", fields, dos_dir=args.dos_dir)
    print(f"wrote [steering] (source=measured, combo_gain={fit['combo_gain']:+.4f}) "
          f"-> {path}")
    print("Combo gain measured — the engine will now discipline this DO via "
          "the combo bus.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

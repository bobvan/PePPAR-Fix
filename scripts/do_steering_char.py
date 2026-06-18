#!/usr/bin/env python3
"""Steering characterization — measure a VCOCXO+DAC actuator's EFC gain curve
and write the ``[steering]`` section of the DO's per-section schema
(``state/dos/<uid>.toml``).

The third leg of the DO characterization suite:

    do_freerun_char.py   -> [freerun_noise]   (DO open-loop noise; TICC chA)
    (sidecar)            -> <uid>.noise.toml  (dense ADEV/TDEV/PSD curves)
    do_steering_char.py  -> [steering]         (THIS — the DAC->freq gain curve)
    (future)             -> [actuation_noise]  (sigma_q)  +  [thermal] (temp slope)

This is the tool the engine's steering refuse-to-actuate gate
(``do_char_resolve.should_refuse_for_steering``) has been waiting for: run it
ONCE per DAC DO so its [steering] is measured and the engine will discipline
it.  (PHC_adjfine / ClockMatrix_FCW gains are intrinsic constants the gate
already exempts — they don't need this.)

Measurement reuses ``dac_slope_cal``'s proven primitives — ``measure_freq_offset``
(TICC chA-chB linear regression) and ``detect_linear_region`` (excludes the
saturated EFC tails) — so this tool only adds the new-schema write path.  Same
safety contract: a ``finally`` returns the DAC to ``--center-code`` and closes
the I2C bus on exit, even on Ctrl-C.

Run on the host (needs I2C + the OCXO PPS on the TICC):

    sudo ~/peppar-fix/venv/bin/python ~/peppar-fix/scripts/do_steering_char.py \\
        --uid 54:49:4d:45:00:6b --bus 1 --addr 0x4C --gain 1 \\
        --ticc-port /dev/ticc2

``--dry-run`` measures + fits + validates but does not write the file.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
import tomllib
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dac_slope_cal import measure_freq_offset, detect_linear_region  # noqa: E402
from peppar_fix.dac_actuator import DacActuator  # noqa: E402
from peppar_fix import do_schema  # noqa: E402


def _utcnow() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def steering_fields_from_fit(fit: dict, center_code: int,
                             dac_gain: int = 0) -> dict:
    """Map a ``detect_linear_region`` result -> schema ``[steering]`` fields.

    The fit is ppb = intercept + slope*(code - center_code), so the intercept
    IS the freq offset at center_code -> parked_code = center_code,
    intercept_ppb_at_parked = intercept.  ``asymmetry_factor`` defaults to 1.0
    (ramp-rate asymmetry isn't measured by this sweep; a later revision can
    fill it).  ``source`` is added by the schema writer, not here.

    ``dac_gain`` is the DAC output-stage gain bit the sweep ran at (0=1×,
    1=2×).  It is recorded so the engine can set the DAC to the SAME gain at
    runtime — a slope measured at 2× is wrong if applied at 1× (the volts/code,
    and hence ppb/code, differ).  This closes the silent-gain-mismatch hole.
    """
    return {
        "slope_ppb_per_code": float(fit["slope"]),
        "intercept_ppb_at_parked": float(fit["intercept"]),
        "parked_code": int(center_code),
        "code_min": int(fit["code_min"]),
        "code_max": int(fit["code_max"]),
        "asymmetry_factor": 1.0,
        "dac_gain": int(dac_gain),
        "rmse_ppb": float(fit["rmse"]),
        "measured_at": _utcnow(),
    }


def sweep(dac, codes, settle_s, measure_s, ticc_port, *, log=print):
    """Step the DAC through ``codes``; measure DO freq offset at each.

    Returns ``[{code, ppb, n_pairs, mean_phase_ns}]``.  The caller owns DAC
    setup/teardown + the recenter-on-exit ``finally`` (so an interrupt mid-sweep
    still parks the DAC)."""
    results = []
    for i, code in enumerate(codes):
        log(f"[{i + 1:2d}/{len(codes)}] code={code:5d}: "
            f"settle {settle_s}s, measure {measure_s}s")
        dac._write_code(code)
        time.sleep(settle_s)
        mean_phase, ppb, n = measure_freq_offset(ticc_port, measure_s)
        log(f"    -> {ppb:+10.3f} ppb  (n={n} pairs, mean phase {mean_phase:+.1f} ns)")
        results.append({"code": code, "ppb": ppb, "n_pairs": n,
                        "mean_phase_ns": mean_phase})
    return results


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uid", required=True,
                    help="DO uid (matches <uid>.toml; MAC/path uids are sanitized)")
    ap.add_argument("--dos-dir", default=None,
                    help="state/dos dir (default: schema default)")
    ap.add_argument("--bus", type=int, default=1, help="I2C bus (default 1)")
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=0x4C,
                    help="I2C address (default 0x4C)")
    ap.add_argument("--gain", type=int, choices=[0, 1], default=1,
                    help="AD5693R GAIN bit: 0=1x, 1=2x (default 1; 0..2xVref)")
    ap.add_argument("--ticc-port", default="/dev/ticc2",
                    help="TICC serial port (default /dev/ticc2)")
    ap.add_argument("--codes",
                    default="5000,10000,15000,20000,25000,30000,32768,"
                            "35000,40000,45000,50000,55000,60000",
                    help="Comma-separated DAC codes to sweep")
    ap.add_argument("--center-code", type=int, default=32768,
                    help="Fit center + DAC park-on-exit code (default 32768)")
    ap.add_argument("--settle-s", type=int, default=30,
                    help="Seconds after each DAC write before measuring")
    ap.add_argument("--measure-s", type=int, default=30,
                    help="Seconds of TICC measurement per code")
    ap.add_argument("--dry-run", action="store_true",
                    help="measure + fit + validate, but do NOT write the .toml")
    args = ap.parse_args(argv)

    # Fail fast if the DO isn't registered, BEFORE the long hardware sweep.
    char_path = do_schema._char_path(args.uid, args.dos_dir)
    if not os.path.exists(char_path):
        ap.error(f"no characterization file at {char_path} — run "
                 f"scripts/migrate_do_state.py or register the DO first.")

    codes = [int(c) for c in args.codes.split(",")]
    dac = DacActuator(args.bus, args.addr, bits=16, dac_type="ad5693r",
                      dac_gain=args.gain)
    dac.setup()
    print(f"Steering char for {args.uid}: {len(codes)} codes, gain={args.gain}, "
          f"~{len(codes) * (args.settle_s + args.measure_s) / 60:.1f} min")
    try:
        results = sweep(dac, codes, args.settle_s, args.measure_s, args.ticc_port)
    finally:
        try:
            dac._write_code(args.center_code)
            print(f"DAC returned to center code {args.center_code}.")
        except Exception as e:  # noqa: BLE001
            print(f"  warning: recenter failed: {e}")
        dac.teardown()

    valid = [r for r in results if r["n_pairs"] >= 5 and r["ppb"] == r["ppb"]]
    if len(valid) < 3:
        print(f"*** too few valid points ({len(valid)}/3) — TICC silent on "
              f"chA/chB, or the DAC isn't steering the DO.  No [steering] written.")
        return 1
    fit = detect_linear_region(valid, center_code=args.center_code)
    if fit is None:
        print("*** could not isolate a linear region (DAC not steering, or "
              "fully saturated).  No [steering] written.")
        return 1

    fields = steering_fields_from_fit(fit, args.center_code, dac_gain=args.gain)
    print(f"\n[steering]  slope={fields['slope_ppb_per_code']:+.5f} ppb/code  "
          f"intercept={fields['intercept_ppb_at_parked']:+.2f} ppb @ code "
          f"{fields['parked_code']}  linear=[{fields['code_min']},"
          f"{fields['code_max']}]  gain={fields['dac_gain']} "
          f"({'2×' if fields['dac_gain'] else '1×'})  "
          f"rmse={fields['rmse_ppb']:.2f} ppb")
    print(f"  NOTE: the engine will set the DAC to gain={fields['dac_gain']} "
          f"({'2×' if fields['dac_gain'] else '1×'}) to match this measurement "
          f"— ensure that matches the hardware build.")

    if args.dry_run:
        with open(char_path, "rb") as f:
            data = tomllib.load(f)
        data["steering"] = {"source": "measured", **fields}
        errs = do_schema.validate_characterization(data)
        print("DRY-RUN: would validate clean (not written)." if not errs
              else f"DRY-RUN: would FAIL validation: {'; '.join(errs)}")
        return 0 if not errs else 1

    path = do_schema.update_characterization_section(
        args.uid, "steering", fields, dos_dir=args.dos_dir)
    print(f"wrote [steering] (source=measured) -> {path}")
    print("DAC steering measured — the engine will now discipline this DO.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

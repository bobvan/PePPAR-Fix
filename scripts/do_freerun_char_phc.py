#!/usr/bin/env python3
"""Free-run + phase-noise characterization of a PHC-parked DO (vs TICC Rb).

PHC-actuator counterpart to ``do_freerun_char.py``.  Parks the PHC
adjfine at a fixed value (default ``0 ppb`` = true freerun, the
crystal at its uncorrected rate), captures TICC chA timestamps for
the configured duration, and merges the analysis into the host DO
state JSON.

Target: TimeHat's i226 internal TCXO (Linux PHC, no DAC).  Other
PHC-actuator platforms (E810, future i225/i350 ports) use this same
script.  For DAC-parked DOs (PiFace OCXO, clkPoC3 OCXO, MadHat
OCXO-33) use the original ``do_freerun_char.py``.

Both scripts share ``peppar_fix.freerun_analysis`` — same capture,
same deviation/PSD math, same JSON merge.  Only the parking step
differs.

# Parking semantics

DAC-side parks at a code derived from a slope-cal zero-crossing;
PHC-side parks adjfine at ``0 ppb`` by default — the DO runs at its
natural crystal rate, no software correction.  The slope of the
captured phase residual then reveals the freerun frequency offset
of the crystal vs the TICC's Rb reference.

If a different parking point is desired (e.g. to capture the
characterization at the host's calibrated freerun offset rather
than at 0 ppb), pass ``--park-adjfine-ppb VALUE``.

The prior adjfine is **saved before** the park and **restored on
exit** (including on ^C / unhandled exception), so the script is
safe to interrupt without leaving the host's clock in a wrong
state.

Usage:
    sudo ~/peppar-fix/venv/bin/python ~/peppar-fix/scripts/do_freerun_char_phc.py \\
        --ptp-device /dev/ptp_i226 \\
        --ticc-port /dev/ticc1 \\
        --duration-s 3600 \\
        --output state/dos/<your-i226-uid>.json \\
        --do-label i226-timehat-tcxo --host TimeHat
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

from peppar_fix.freerun_analysis import (  # noqa: E402
    _TAUS_S,
    analyze_samples,
    collect_phase_series,
    merge_characterization_into_json,
    print_summary,
    write_raw_csv,
)
from peppar_fix.phc_actuator import PhcAdjfineActuator  # noqa: E402
from peppar_fix.ptp_device import PtpDevice  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ptp-device', required=True,
                    help='PTP device path (e.g. /dev/ptp_i226).  Resolved at '
                         'open time via /sys/class/ptp/.../clock_name; the '
                         'engine\'s udev rules typically expose a stable '
                         'symlink like /dev/ptp_i226 or /dev/ptp_e810.')
    ap.add_argument('--park-adjfine-ppb', type=float, default=0.0,
                    help='Adjfine value to park at, in ppb (default 0.0 = '
                         'true freerun, crystal uncorrected).  Pass a non-'
                         'zero value to characterize at a calibrated offset.')
    ap.add_argument('--ticc-port', default='/dev/ticc1')
    ap.add_argument('--output', required=True,
                    help='JSON to merge characterization into')
    ap.add_argument('--do-label', default=None)
    ap.add_argument('--host', default=None,
                    help='Host name written into the characterization section')
    ap.add_argument('--duration-s', type=int, default=3600,
                    help='Capture duration in seconds (default: 3600)')
    ap.add_argument('--warmup-s', type=int, default=60,
                    help='Settling time after the adjfine step before '
                         'starting capture (default: 60).')
    ap.add_argument('--raw-csv', default=None,
                    help='Optional path for raw chA samples (ref_sec, ref_ps). '
                         'Lets us re-analyze offline without another capture.')
    args = ap.parse_args()

    out_path = Path(args.output)
    do_label = args.do_label or out_path.stem

    print(f"DO freerun + phase-noise characterization for {do_label}")
    print(f"  PHC:        {args.ptp_device}")
    print(f"  park:       adjfine = {args.park_adjfine_ppb:+.3f} ppb")
    print(f"  TICC:       {args.ticc_port}")
    print(f"  Duration:   {args.duration_s} s ({args.duration_s/60:.1f} min)")
    print()

    ptp = PtpDevice(args.ptp_device)
    actuator = PhcAdjfineActuator(ptp)
    actuator.setup()

    # Save prior adjfine so we restore on exit (no clock disturbance
    # to whatever else is running on the host once the char ends).
    prior_adjfine_ppb = actuator.read_frequency_ppb()
    print(f"PHC prior adjfine: {prior_adjfine_ppb:+.3f} ppb (will restore)")

    samples = []
    try:
        actuator.adjust_frequency_ppb(args.park_adjfine_ppb)
        print(f"PHC parked at adjfine = {args.park_adjfine_ppb:+.3f} ppb")

        print(f"\nWarmup {args.warmup_s}s after adjfine step (thermal settle)…")
        time.sleep(args.warmup_s)
        print(f"\nCapturing {args.duration_s} s of TICC chA (vs Rb)…")
        t_start = time.monotonic()
        samples = collect_phase_series(args.ticc_port, args.duration_s)
        t_cap = time.monotonic() - t_start
        print(f"  → {len(samples)} chA samples in {t_cap:.0f} s")
    finally:
        # Restore prior adjfine — see PhcAdjfineActuator.teardown()
        # docstring: that method deliberately does NOT zero the
        # adjfine (to preserve the engine's drift-file value across
        # restarts).  We do the explicit restore here so the host's
        # clock state matches what we found.
        try:
            actuator.adjust_frequency_ppb(prior_adjfine_ppb)
            print(f"PHC adjfine restored to {prior_adjfine_ppb:+.3f} ppb")
        except Exception as e:
            print(f"WARN: could not restore adjfine: {e}")
        try:
            actuator.teardown()
        except Exception:
            pass
        try:
            ptp.close()
        except Exception:
            pass

    if len(samples) < 60:
        print(f"\nToo few samples ({len(samples)}) for meaningful statistics")
        return 1

    if args.raw_csv:
        write_raw_csv(Path(args.raw_csv), samples)
        print(f"Raw chA samples saved → {args.raw_csv}")

    result = analyze_samples(samples, taus_s=_TAUS_S)
    if result is None:
        print(f"\nAnalysis failed (too few samples)")
        return 1

    print_summary(result)

    parking = {
        'parked_adjfine_ppb': args.park_adjfine_ppb,
        'prior_adjfine_ppb_restored': prior_adjfine_ppb,
        'ptp_device': args.ptp_device,
        'freq_offset_ppb_at_parked_state': result['freq_offset_ppb'],
    }
    merge_characterization_into_json(
        out_path, result, parking, do_label=do_label, host=args.host or '')
    print(f"\nSaved characterization → {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

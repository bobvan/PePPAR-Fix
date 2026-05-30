#!/usr/bin/env python3
"""Free-run + phase-noise characterization of a DAC-parked DO (vs TICC Rb).

Parks the DAC at a fixed code (default: code yielding ~0 ppb offset
from the existing dac_slope_cal.py output, if present in the JSON;
else mid-scale 32768), then captures TICC chA timestamps for a
configured duration.  Computes:

  - RMS of phase residual (ns)
  - PSD curve at standard offset frequencies (0.005–0.5 Hz from 1 Hz
    TICC; this is the *low-frequency* phase noise band — the spectral
    region that overlaps with the moonshot's 1-sec to ~200-sec
    timescale.  Higher-band L(f) requires a phase-noise analyzer.)
  - ASD at 0.1 Hz, 0.01 Hz
  - Spectral slope (log-log linear fit) + noise-type classification
  - ADEV / TDEV / MDEV at standard τ

Output: a `characterization` section merged into the existing DO JSON
file (e.g. state/dos/<label>.json).  If the file has a slope-cal
section already (from dac_slope_cal.py), this script preserves it.

# Reference: TICC's internal Rb-class timebase, not GNSS

The DO PPS is read on TICC chA and measured against the TICC's own
Rb timebase (the TICC reports each edge as <ref_sec, ref_ps> in its
internal frame).  chB (GNSS PPS) is **deliberately ignored** — a
freerun characterization has nothing to do with GNSS.  Mixing in
chB introduces ~2 ns of GNSS-PPS sawtooth at τ=1 s that's unrelated
to the DO, inflates the reported noise floor by ~40×, and makes the
result incomparable to servoed-mode chA-vs-Rb numbers.  See dayplan
freerunCharChAOnly for the 2026-05-26 → 27 MadHat incident that
caught this.

Linear drift (constant frequency offset from the parked code's
residual) is removed before spectral analysis so the PSD reflects
the *noise* floor, not the drift.

# Companion scripts (per-actuator)

This script handles DAC-parked DOs.  For PHC-adjfine-parked DOs
(e.g. TimeHat's i226 TCXO) use ``do_freerun_char_phc.py``.  Both
scripts share ``peppar_fix.freerun_analysis`` for capture +
analysis + JSON merge.

Usage:
    sudo ~/peppar-fix/venv/bin/python ~/peppar-fix/scripts/do_freerun_char.py \\
        --bus 1 --addr 0x4C --gain 1 \\
        --code 35000  \\
        --ticc-port /dev/ticc2 \\
        --duration-s 3600 \\
        --output state/dos/isotemp-ocxo-33-madhat.json

Safe to interrupt — restores DAC to mid-scale in `finally`.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

from peppar_fix.dac_actuator import DacActuator  # noqa: E402
from peppar_fix.freerun_analysis import (  # noqa: E402
    _TAUS_S,
    analyze_samples,
    collect_phase_series,
    merge_characterization_into_json,
    print_summary,
    write_raw_csv,
)


def pick_code_for_zero_ppb(json_path: Path, fallback: int = 32768) -> int:
    """If a slope-cal section is present in the JSON, return the DAC
    code nearest to ppb=0.  Otherwise return fallback (mid-scale).

    DAC-specific helper — kept in this script rather than the shared
    library because the slope/intercept fields it reads
    (``dac_ppb_per_code`` + ``dac_center_freq_offset_ppb``) are
    written by ``scripts/dac_slope_cal.py``, the DAC's own
    calibration tool.  The PHC variant uses a different parking
    convention (default 0 adjfine = true freerun) and doesn't need
    this lookup.
    """
    if not json_path.exists():
        return fallback
    try:
        data = json.loads(json_path.read_text())
    except Exception:
        return fallback
    slope = data.get('dac_ppb_per_code')
    icpt = data.get('dac_center_freq_offset_ppb')
    if slope is None or icpt is None:
        return fallback
    if slope == 0:
        return fallback
    code = round(32768 - icpt / slope)
    code = max(0, min(65535, code))
    return code


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bus', type=int, default=1)
    ap.add_argument('--addr', type=lambda s: int(s, 0), default=0x4C)
    ap.add_argument('--gain', type=int, choices=[0, 1], default=0)
    ap.add_argument('--code', type=int, default=None,
                    help='DAC code to park at (default: derived from slope cal '
                         'in --output JSON; else 32768)')
    ap.add_argument('--ticc-port', default='/dev/ticc2')
    ap.add_argument('--output', required=True,
                    help='JSON to merge characterization into')
    ap.add_argument('--do-label', default=None)
    ap.add_argument('--host', default=None,
                    help='Host name written into the characterization section')
    ap.add_argument('--duration-s', type=int, default=3600,
                    help='Capture duration in seconds (default: 3600)')
    ap.add_argument('--raw-csv', default=None,
                    help='Optional path for raw chA samples (ref_sec, ref_ps). '
                         'Lets us re-analyze offline without another capture.')
    args = ap.parse_args()

    out_path = Path(args.output)
    code = args.code if args.code is not None else pick_code_for_zero_ppb(out_path)
    do_label = args.do_label or out_path.stem

    print(f"DO freerun + phase-noise characterization for {do_label}")
    print(f"  DAC:        bus={args.bus} addr=0x{args.addr:02X} "
          f"gain={args.gain} code={code}")
    print(f"  TICC:       {args.ticc_port}")
    print(f"  Duration:   {args.duration_s} s ({args.duration_s/60:.1f} min)")
    print()

    dac = DacActuator(args.bus, args.addr, bits=16, ppb_per_code=1.0,
                      dac_type='ad5693r', dac_gain=args.gain)
    dac.setup()
    dac._write_code(code)
    print(f"DAC parked at code {code}")

    samples = []
    try:
        print(f"\nWarmup 60s after DAC step (thermal settle)…")
        time.sleep(60)
        print(f"\nCapturing {args.duration_s} s of TICC chA (vs Rb)…")
        t_start = time.monotonic()
        samples = collect_phase_series(args.ticc_port, args.duration_s)
        t_cap = time.monotonic() - t_start
        print(f"  → {len(samples)} chA samples in {t_cap:.0f} s")
    finally:
        try:
            dac._write_code(32768)
        except Exception:
            pass
        dac.teardown()

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
        'parked_dac_code': code,
        'parked_dac_gain': args.gain,
        'freq_offset_ppb_at_parked_code': result['freq_offset_ppb'],
    }
    merge_characterization_into_json(
        out_path, result, parking, do_label=do_label, host=args.host or '')
    print(f"\nSaved characterization → {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

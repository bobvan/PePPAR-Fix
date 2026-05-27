#!/usr/bin/env python3
"""DAC slope calibration for a VCOCXO+DAC actuator.

Steps the DAC through a sequence of codes; at each code, holds for
thermal settling, then measures DO frequency offset via TICC chA−chB.
Linear-fits ppb vs code → `dac_ppb_per_code`.

Workflow:
    sudo ~/peppar-fix/venv/bin/python ~/peppar-fix/scripts/dac_slope_cal.py \\
        --bus 1 --addr 0x4C --gain 0 \\
        --ticc-port /dev/ticc2 \\
        --output state/dos/isotemp-ocxo-33-madhat.json

The TICC chA must be the DO PPS edge; chB the GNSS PPS edge.

The output JSON has `dac_ppb_per_code`, `dac_center_freq_offset_ppb`,
`dac_max_ppb`, plus the per-code measurements for transparency.
Suggested `config/<host>.toml` additions are printed at the end.

Safe to interrupt: a `finally` block returns the DAC to mid-scale
(code 32768) and closes the I2C bus before exit.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

from peppar_fix.dac_actuator import DacActuator  # noqa: E402
from ticc import Ticc  # noqa: E402


def measure_freq_offset(ticc_port: str, duration_s: int):
    """Listen for TICC chA/chB pairs and compute the DO frequency offset
    in ppb via linear regression of (chA − chB) against time.

    Returns (mean_phase_ns, freq_offset_ppb, n_pairs).
    """
    chA: dict[int, float] = {}
    chB: dict[int, float] = {}
    pairs: list[tuple[int, float]] = []  # (ref_sec, phase_ns = chA−chB)
    seen_secs: set[int] = set()
    t_start: float | None = None

    with Ticc(ticc_port, wait_for_boot=False) as ticc:
        for ch, ref_sec, ref_ps in ticc:
            if t_start is None:
                t_start = time.monotonic()
            elif time.monotonic() - t_start > duration_s:
                break

            phase_s = ref_sec + ref_ps * 1e-12
            if ch == 'chA':
                chA[ref_sec] = phase_s
            elif ch == 'chB':
                chB[ref_sec] = phase_s

            # Pair any newly-completed seconds
            for sec in set(chA) & set(chB):
                if sec not in seen_secs:
                    pairs.append((sec, (chA[sec] - chB[sec]) * 1e9))
                    seen_secs.add(sec)
            # Cleanup orphans older than 5 s to keep dicts small
            cutoff = ref_sec - 5
            for sec in list(chA):
                if sec < cutoff and sec not in chB:
                    del chA[sec]
            for sec in list(chB):
                if sec < cutoff and sec not in chA:
                    del chB[sec]

    if len(pairs) < 5:
        return float('nan'), float('nan'), len(pairs)

    secs = [p[0] for p in pairs]
    phases_ns = [p[1] for p in pairs]
    n = len(pairs)
    mean_s = sum(secs) / n
    mean_p = sum(phases_ns) / n
    num = sum((s - mean_s) * (p - mean_p) for s, p in zip(secs, phases_ns))
    den = sum((s - mean_s) ** 2 for s in secs)
    if den == 0:
        return mean_p, float('nan'), n
    # slope in ns/s == ppb (1 ns/s drift = 1 part per billion)
    slope_ppb = num / den
    return mean_p, slope_ppb, n


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bus', type=int, default=1,
                    help='I2C bus number (default: 1)')
    ap.add_argument('--addr', type=lambda s: int(s, 0), default=0x4C,
                    help='I2C device address (default: 0x4C)')
    ap.add_argument('--gain', type=int, choices=[0, 1], default=0,
                    help='AD5693R GAIN bit: 0=1× (default), 1=2×')
    ap.add_argument('--ticc-port', default='/dev/ticc2',
                    help='TICC serial port (default: /dev/ticc2)')
    ap.add_argument('--output', required=True,
                    help='Output JSON path (state/dos/<label>.json)')
    ap.add_argument('--do-label', default=None,
                    help='DO label written to JSON (default: derived from --output filename)')
    ap.add_argument('--codes', default='5000,10000,15000,20000,25000,30000,32768,35000,40000,45000,50000,55000,60000',
                    help='Comma-separated DAC codes to test')
    ap.add_argument('--settle-s', type=int, default=30,
                    help='Seconds to wait after writing DAC before measuring (default: 30)')
    ap.add_argument('--measure-s', type=int, default=30,
                    help='Seconds of TICC measurement per code (default: 30)')
    args = ap.parse_args()

    codes = [int(c) for c in args.codes.split(',')]
    per_code_s = args.settle_s + args.measure_s
    do_label = args.do_label or Path(args.output).stem

    print(f"DAC slope calibration for {do_label}")
    print(f"  DAC:        bus={args.bus} addr=0x{args.addr:02X} gain={args.gain}")
    print(f"  TICC:       {args.ticc_port}")
    print(f"  Codes:      {len(codes)} from {min(codes)} to {max(codes)}")
    print(f"  Per-code:   {args.settle_s}s settle + {args.measure_s}s measure = {per_code_s}s")
    print(f"  Total:      ~{len(codes) * per_code_s / 60:.1f} min")
    print()

    dac = DacActuator(args.bus, args.addr, bits=16, ppb_per_code=1.0,
                      dac_type='ad5693r', dac_gain=args.gain)
    dac.setup()
    print(f"DAC opened: bus={args.bus} addr=0x{args.addr:02X} gain={args.gain}")

    results: list[dict] = []
    try:
        for i, code in enumerate(codes):
            print(f"\n[{i+1:2d}/{len(codes)}] code={code:5d}: writing DAC, settling {args.settle_s}s, measuring {args.measure_s}s")
            dac._write_code(code)
            time.sleep(args.settle_s)
            mean_phase, ppb, n = measure_freq_offset(args.ticc_port, args.measure_s)
            print(f"             → freq offset: {ppb:+10.3f} ppb  (n={n:3d} pairs, mean phase={mean_phase:+10.1f} ns)")
            results.append({'code': code, 'ppb': ppb, 'n_pairs': n,
                            'mean_phase_ns': mean_phase})
    finally:
        print(f"\nReturning DAC to mid-scale (32768)...")
        try:
            dac._write_code(32768)
        except Exception as e:
            print(f"  warning: failed to recenter: {e}")
        dac.teardown()

    # Linear fit on valid points
    valid = [r for r in results if r['n_pairs'] >= 5 and r['ppb'] == r['ppb']]
    if len(valid) < 3:
        print(f"\n*** TOO FEW VALID POINTS ({len(valid)}/3) — fit not produced")
        print(f"    Likely cause: TICC chA or chB silent, or DAC not steering the DO")
        return 1

    cs = [r['code'] - 32768 for r in valid]
    ps = [r['ppb'] for r in valid]
    n = len(valid)
    mean_c = sum(cs) / n
    mean_p = sum(ps) / n
    num = sum((c - mean_c) * (p - mean_p) for c, p in zip(cs, ps))
    den = sum((c - mean_c) ** 2 for c in cs)
    if den == 0:
        print(f"*** ALL CODES IDENTICAL — fit not produced")
        return 1
    slope = num / den
    intercept = mean_p - slope * mean_c
    resids = [p - (intercept + slope * c) for c, p in zip(cs, ps)]
    rmse = (sum(r ** 2 for r in resids) / n) ** 0.5
    full_range_codes = min(32768, 65535 - 32768)
    max_ppb = full_range_codes * abs(slope)

    print(f"\n=== LINEAR FIT (ppb = intercept + slope × (code − 32768)) ===")
    print(f"  Points used:                {n} of {len(results)}")
    print(f"  Slope (ppb_per_code):       {slope:+.5f}")
    print(f"  Intercept (offset @ 32768): {intercept:+.2f} ppb")
    print(f"  RMS residual:               {rmse:.2f} ppb")
    print(f"  Estimated max_ppb (center → rail): {max_ppb:.1f}")

    out = {
        'do_label': do_label,
        'characterization_date': datetime.now(tz=timezone.utc).isoformat(),
        'dac_type': 'ad5693r',
        'dac_bus': args.bus,
        'dac_addr': args.addr,
        'dac_bits': 16,
        'dac_gain': args.gain,
        'dac_ppb_per_code': slope,
        'dac_center_freq_offset_ppb': intercept,
        'dac_max_ppb': max_ppb,
        'fit_rmse_ppb': rmse,
        'fit_n_points': n,
        'measurements': results,
    }
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, 'w') as f:
        json.dump(out, f, indent=2)
    print(f"\nSaved characterization to {out_path}")

    print(f"\nSuggested config/<host>.toml additions:")
    print(f"  dac_bus = {args.bus}")
    print(f'  dac_addr = "0x{args.addr:02X}"')
    print(f"  dac_bits = 16")
    print(f'  dac_type = "ad5693r"')
    print(f"  dac_gain = {args.gain}")
    print(f'  dac_ppb_per_code = "{slope:.5f}"')
    print(f"  dac_max_ppb = {max_ppb:.1f}")
    return 0


if __name__ == '__main__':
    sys.exit(main())

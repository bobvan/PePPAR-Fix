#!/usr/bin/env python3
"""DAC slope calibration for a VCOCXO+DAC actuator.

Steps the DAC through a sequence of codes; at each code, holds for
thermal settling, then measures DO frequency offset via TICC chA−chB.
Linear-fits ppb vs code → `dac_ppb_per_code`.

All ppb values are reported in **engine convention**: positive ppb
means the DO is **fast** (firing earlier than the reference each
second).  This matches scripts/peppar_fix/timestamper.py's
measure_differential_frequency and the DAC actuator's sign expectation
in peppar_fix.dac_actuator.  Output `dac_ppb_per_code` can be either
sign — negative means an OCXO with inverted EFC polarity (increasing
DAC code → decreasing frequency).  See dayplan bootstrapSignFlip for
the bug-history context.

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
    # slope in ns/s = ppb in *engine* convention: positive ppb means
    # DO is fast (= chA-chB DECREASING over time).  Raw regression
    # gives d(chA-chB)/dt, which is positive when chA-chB INCREASES
    # i.e. DO is slow.  Negate to match the engine's convention used
    # by timestamper.py::measure_differential_frequency and the DAC
    # actuator's ppb/code sign in peppar_fix.dac_actuator.  See
    # dayplan bootstrapSignFlip for the bug this fix addresses.
    slope_ppb = -(num / den)
    return mean_p, slope_ppb, n


def _fit(points, center_code):
    """OLS fit of ppb vs (code - center). Returns (slope, intercept, rmse)."""
    cs = [c - center_code for c, _ in points]
    ps = [p for _, p in points]
    n = len(points)
    mc = sum(cs) / n
    mp = sum(ps) / n
    num = sum((c - mc) * (p - mp) for c, p in zip(cs, ps))
    den = sum((c - mc) ** 2 for c in cs)
    if den == 0:
        return None
    slope = num / den
    intercept = mp - slope * mc
    rmse = (sum((p - (intercept + slope * c)) ** 2
                for c, p in zip(cs, ps)) / n) ** 0.5
    return slope, intercept, rmse


def detect_linear_region(valid, center_code=32768, min_slope_frac=0.6,
                         resid_drop_ppb=8.0):
    """Isolate the linear EFC region, excluding saturated tails.

    OCXO EFC curves clip at one or both ends (the frequency stops
    responding to voltage).  Past the clip, the ppb↔code slope goes
    to ~0 and the linear model is fiction.  This finds the contiguous
    run of codes whose consecutive-pair slope stays within
    min_slope_frac of the steepest segment, then trims any boundary
    point whose fit residual exceeds resid_drop_ppb.

    Returns dict(slope, intercept, rmse, code_min, code_max, n_linear)
    or None if no region can be isolated.
    """
    pts = sorted((r['code'], r['ppb']) for r in valid)
    if len(pts) < 3:
        return None
    seg = [(pts[i + 1][1] - pts[i][1]) / (pts[i + 1][0] - pts[i][0])
           for i in range(len(pts) - 1)]
    max_abs = max(abs(s) for s in seg)
    if max_abs == 0:
        return None  # totally flat — DAC not steering the DO
    thresh = min_slope_frac * max_abs
    steepest = max(range(len(seg)), key=lambda i: abs(seg[i]))
    lo, hi = steepest, steepest
    while lo > 0 and abs(seg[lo - 1]) >= thresh:
        lo -= 1
    while hi < len(seg) - 1 and abs(seg[hi + 1]) >= thresh:
        hi += 1
    linear = pts[lo:hi + 2]  # hi is a segment index → +2 for point count

    # Residual trim: drop boundary points that don't fit the line.
    while len(linear) >= 3:
        res = _fit(linear, center_code)
        if res is None:
            return None
        slope, intercept, rmse = res
        worst_i, worst_r = max(
            enumerate(abs(p - (intercept + slope * (c - center_code)))
                      for c, p in linear),
            key=lambda x: x[1])
        if worst_r <= resid_drop_ppb or worst_i not in (0, len(linear) - 1):
            break
        linear.pop(worst_i)

    res = _fit(linear, center_code)
    if res is None:
        return None
    slope, intercept, rmse = res
    return {
        'slope': slope, 'intercept': intercept, 'rmse': rmse,
        'code_min': linear[0][0], 'code_max': linear[-1][0],
        'n_linear': len(linear),
    }


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

    # Linear-region fit: exclude saturated tails where the EFC clips.
    valid = [r for r in results if r['n_pairs'] >= 5 and r['ppb'] == r['ppb']]
    if len(valid) < 3:
        print(f"\n*** TOO FEW VALID POINTS ({len(valid)}/3) — fit not produced")
        print(f"    Likely cause: TICC chA or chB silent, or DAC not steering the DO")
        return 1

    fit = detect_linear_region(valid, center_code=32768)
    if fit is None:
        print(f"*** Could not isolate a linear region — fit not produced")
        return 1
    slope = fit['slope']
    intercept = fit['intercept']
    rmse = fit['rmse']
    code_min = fit['code_min']
    code_max = fit['code_max']
    n = fit['n_linear']
    n_saturated = len(valid) - n

    # Reachable frequency range from the REAL linear region (asymmetric).
    # NOT slope × codes-to-rail — that's fiction past saturation.
    ppb_at_min = (code_min - 32768) * slope
    ppb_at_max = (code_max - 32768) * slope
    ppb_fast = max(ppb_at_min, ppb_at_max)
    ppb_slow = min(ppb_at_min, ppb_at_max)
    # max_ppb = the symmetric envelope (smaller one-sided range), for
    # legacy callers.  The asymmetric truth lives in code_min/code_max.
    max_ppb = min(abs(ppb_fast), abs(ppb_slow))

    print(f"\n=== LINEAR-REGION FIT (ppb = intercept + slope × (code − 32768)) ===")
    print(f"  Linear points:              {n} of {len(valid)} "
          f"({n_saturated} saturated, excluded)")
    print(f"  Linear code range:          [{code_min}, {code_max}]")
    print(f"  Slope (ppb_per_code):       {slope:+.5f}")
    print(f"  Intercept (offset @ 32768): {intercept:+.2f} ppb")
    print(f"  RMS residual:               {rmse:.2f} ppb")
    print(f"  Reachable range:            {ppb_slow:+.1f} .. {ppb_fast:+.1f} ppb "
          f"(ASYMMETRIC)")
    print(f"  Symmetric envelope (max_ppb): {max_ppb:.1f}")
    if n_saturated > 0:
        print(f"  *** SATURATION DETECTED — {n_saturated} points clipped "
              f"outside [{code_min}, {code_max}]")

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # Merge into the existing DO-state JSON, not overwrite.  The same
    # file carries the freerun characterization (written by
    # do_freerun_char.py under the 'characterization' key) — a blind
    # overwrite here clobbered it (lost MadHat's 47 ps freerun floor
    # 2026-05-28).  Read-modify-write only the DAC-slope keys.
    existing = {}
    if out_path.exists():
        try:
            existing = json.loads(out_path.read_text())
        except Exception:
            existing = {}

    existing.update({
        'unique_id': do_label,
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
        'dac_code_min': code_min,
        'dac_code_max': code_max,
        'dac_ppb_fast': ppb_fast,
        'dac_ppb_slow': ppb_slow,
        'fit_rmse_ppb': rmse,
        'fit_n_points': n,
        'fit_n_saturated': n_saturated,
        'measurements': results,
    })
    with open(out_path, 'w') as f:
        json.dump(existing, f, indent=2)
    _kept = 'characterization' in existing
    print(f"\nSaved DAC slope cal to {out_path}"
          f"{' (freerun characterization preserved)' if _kept else ''}")

    print(f"\nSuggested config/<host>.toml additions:")
    print(f"  dac_bus = {args.bus}")
    print(f'  dac_addr = "0x{args.addr:02X}"')
    print(f"  dac_bits = 16")
    print(f'  dac_type = "ad5693r"')
    print(f"  dac_gain = {args.gain}")
    print(f'  dac_ppb_per_code = "{slope:.5f}"')
    print(f"  dac_max_ppb = {max_ppb:.1f}")
    print(f"  dac_code_min = {code_min}")
    print(f"  dac_code_max = {code_max}")
    return 0


if __name__ == '__main__':
    sys.exit(main())

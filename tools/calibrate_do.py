#!/usr/bin/env python3
"""calibrate_do.py — Characterize a DO's tuning range and sensitivity.

Measures frequency vs actuator setting using a TICC.  Works with any
FrequencyActuator (DAC, ClockMatrix FCW, PHC adjfine) and any TICC.

The DO under test drives chA of the TICC.  chB should be driven by a
stable reference (GPSDO).  The tool sweeps the actuator through a
series of setpoints, dwells at each for a configurable interval, and
measures the resulting frequency offset from the TICC intervals.

Output: a table of (actuator_setting_ppb, measured_freq_offset_ppb)
plus derived metrics:
  - Tuning range (ppb)
  - Gain (measured ppb / commanded ppb)
  - Resolution (smallest detectable step)
  - Linearity (max deviation from best-fit line)

Usage:
    # DAC-driven OCXO on clkPoC3, TICC #3
    python3 tools/calibrate_do.py \\
        --ticc-port /dev/ticc3 \\
        --dac-bus 1 --dac-addr 0x4C --dac-bits 16 \\
        --dac-ppb-per-code 0.1 \\
        --sweep-ppb -500,500 --steps 11 --dwell 30

    # PHC adjfine sweep (built-in oscillator)
    python3 tools/calibrate_do.py \\
        --ticc-port /dev/ticc1 \\
        --phc /dev/ptp0 \\
        --sweep-ppb -1000,1000 --steps 5 --dwell 60

    # Just measure current frequency (no sweep)
    python3 tools/calibrate_do.py \\
        --ticc-port /dev/ticc3 \\
        --channel chA --dwell 10
"""

import argparse
import json
import logging
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

log = logging.getLogger("calibrate_do")


def measure_frequency_offset(ticc, channel, dwell_s):
    """Measure mean interval and frequency offset over a dwell period.

    The DO under test should produce edges at a nominal rate (e.g. 1 Hz
    from a divide-by-10M counter).  We measure successive intervals and
    compute the fractional frequency offset.

    Returns:
        dict with:
            mean_interval_ps: mean interval in picoseconds
            freq_offset_ppb: fractional frequency offset in ppb
            interval_std_ps: standard deviation of intervals
            n_intervals: number of intervals measured
    """
    edges = []
    deadline = time.monotonic() + dwell_s

    for ch, ref_sec, ref_ps in ticc:
        if time.monotonic() > deadline:
            break
        if ch != channel:
            continue
        edges.append((ref_sec, ref_ps))

    if len(edges) < 3:
        return None

    # Compute intervals in picoseconds
    intervals_ps = []
    for i in range(1, len(edges)):
        sec_diff = edges[i][0] - edges[i-1][0]
        ps_diff = edges[i][1] - edges[i-1][1]
        total_ps = sec_diff * 1_000_000_000_000 + ps_diff
        if total_ps > 0:
            intervals_ps.append(total_ps)

    if len(intervals_ps) < 2:
        return None

    mean_ps = sum(intervals_ps) / len(intervals_ps)
    # Nominal interval: round to nearest second (PPS from divider)
    nominal_ps = round(mean_ps / 1_000_000_000_000) * 1_000_000_000_000
    if nominal_ps == 0:
        nominal_ps = 1_000_000_000_000  # assume 1 Hz

    freq_offset_ppb = (mean_ps - nominal_ps) / nominal_ps * 1e9

    variance = sum((x - mean_ps) ** 2 for x in intervals_ps) / len(intervals_ps)
    std_ps = variance ** 0.5

    return {
        "mean_interval_ps": mean_ps,
        "freq_offset_ppb": freq_offset_ppb,
        "interval_std_ps": std_ps,
        "n_intervals": len(intervals_ps),
    }


def build_actuator(args):
    """Build a FrequencyActuator from CLI args.

    Returns (actuator, label) or (None, None) for measure-only mode.
    """
    if getattr(args, 'dac_bus', None) is not None:
        from peppar_fix.dac_actuator import DacActuator
        dac_addr = int(args.dac_addr, 0)
        actuator = DacActuator(
            bus_num=args.dac_bus,
            addr=dac_addr,
            bits=args.dac_bits,
            center_code=args.dac_center_code,
            ppb_per_code=args.dac_ppb_per_code,
            max_ppb=args.dac_max_ppb,
            dac_type=args.dac_type,
            dac_gain=getattr(args, 'dac_gain', 0),
        )
        gain_label = "2×" if getattr(args, 'dac_gain', 0) else "1×"
        label = (f"DAC bus={args.dac_bus} addr=0x{dac_addr:02x} "
                 f"({gain_label} mode)")
        return actuator, label

    if getattr(args, 'phc', None) is not None:
        from peppar_fix.ptp_device import PtpDevice
        from peppar_fix.phc_actuator import PhcAdjfineActuator
        ptp = PtpDevice(args.phc)
        actuator = PhcAdjfineActuator(ptp)
        label = f"PHC {args.phc}"
        return actuator, label

    return None, None


def main():
    ap = argparse.ArgumentParser(
        description="Characterize DO tuning range and sensitivity via TICC",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # TICC
    ap.add_argument("--ticc-port", required=True,
                    help="TICC serial port (e.g. /dev/ticc3)")
    ap.add_argument("--ticc-baud", type=int, default=115200)
    ap.add_argument("--channel", default="chA",
                    help="TICC channel carrying the DO under test (default: chA)")

    # Measurement
    ap.add_argument("--dwell", type=int, default=30,
                    help="Seconds to dwell at each setpoint (default: 30)")
    ap.add_argument("--settle", type=int, default=5,
                    help="Seconds to wait after changing setpoint before measuring (default: 5)")

    # Sweep
    ap.add_argument("--sweep-ppb", default=None,
                    help="Sweep range as min,max in ppb (e.g. -500,500)")
    ap.add_argument("--steps", type=int, default=5,
                    help="Number of sweep points (default: 5)")
    ap.add_argument("--sweep-codes", default=None,
                    help="Sweep range as min,max in DAC codes (e.g. 0,65535)")

    # DAC actuator
    ap.add_argument("--dac-bus", type=int, default=None)
    ap.add_argument("--dac-addr", default="0x4C")
    ap.add_argument("--dac-bits", type=int, default=16)
    ap.add_argument("--dac-center-code", type=int, default=None)
    ap.add_argument("--dac-ppb-per-code", type=float, default=1.0,
                    help="Initial estimate of ppb/code (refined by calibration)")
    ap.add_argument("--dac-max-ppb", type=float, default=None)
    ap.add_argument("--dac-type", default="ad5693r")
    ap.add_argument("--dac-gain", type=int, default=0, choices=[0, 1],
                    help="AD5693R GAIN bit: 0=1× output (0..Vref, default), "
                         "1=2× output (0..2×Vref).  Recharacterize after "
                         "changing — ppb_per_code roughly doubles in 2× mode.")

    # PHC actuator (alternative)
    ap.add_argument("--phc", default=None,
                    help="PTP device for PHC adjfine sweep (e.g. /dev/ptp0)")

    # Output
    ap.add_argument("--output", default=None,
                    help="Save results to JSON file")
    ap.add_argument("-v", "--verbose", action="store_true")

    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    from ticc import Ticc

    actuator, actuator_label = build_actuator(args)

    # Measure-only mode (no actuator, no sweep)
    if actuator is None and args.sweep_ppb is None and args.sweep_codes is None:
        log.info("Measure-only mode — reading %s for %ds", args.channel, args.dwell)
        with Ticc(args.ticc_port, args.ticc_baud) as ticc:
            result = measure_frequency_offset(ticc, args.channel, args.dwell)
        if result is None:
            log.error("No measurements on %s", args.channel)
            return 1
        print(f"\nFrequency offset: {result['freq_offset_ppb']:+.3f} ppb")
        print(f"Interval std:     {result['interval_std_ps']:.1f} ps "
              f"({result['interval_std_ps']/1000:.3f} ns)")
        print(f"Measurements:     {result['n_intervals']}")
        return 0

    if actuator is None:
        log.error("Sweep requires an actuator (--dac-bus or --phc)")
        return 1

    # Build sweep setpoints
    if args.sweep_codes is not None:
        # Sweep in raw DAC codes
        lo, hi = [int(x) for x in args.sweep_codes.split(",")]
        if args.steps == 1:
            codes = [lo]
        else:
            codes = [lo + i * (hi - lo) // (args.steps - 1) for i in range(args.steps)]
        # Convert to ppb for the actuator
        setpoints_ppb = [(c - actuator._center_code) * actuator._ppb_per_code
                         for c in codes]
        log.info("Sweep: %d codes from %d to %d", len(codes), lo, hi)
    elif args.sweep_ppb is not None:
        lo, hi = [float(x) for x in args.sweep_ppb.split(",")]
        if args.steps == 1:
            setpoints_ppb = [lo]
        else:
            step = (hi - lo) / (args.steps - 1)
            setpoints_ppb = [lo + i * step for i in range(args.steps)]
        log.info("Sweep: %d points from %.1f to %.1f ppb", len(setpoints_ppb), lo, hi)
    else:
        log.error("Specify --sweep-ppb or --sweep-codes")
        return 1

    # Run the sweep
    log.info("Actuator: %s", actuator_label)
    actuator.setup()

    results = []
    try:
        with Ticc(args.ticc_port, args.ticc_baud) as ticc:
            for i, target_ppb in enumerate(setpoints_ppb):
                actual_ppb = actuator.adjust_frequency_ppb(target_ppb)
                log.info("[%d/%d] Set %.3f ppb (actual %.3f), settling %ds...",
                         i + 1, len(setpoints_ppb), target_ppb, actual_ppb,
                         args.settle)
                # Settle: read and discard
                settle_deadline = time.monotonic() + args.settle
                for ch, ref_sec, ref_ps in ticc:
                    if time.monotonic() > settle_deadline:
                        break

                log.info("  Measuring for %ds...", args.dwell)
                meas = measure_frequency_offset(ticc, args.channel, args.dwell)
                if meas is None:
                    log.warning("  No measurements — skipping")
                    continue

                results.append({
                    "commanded_ppb": target_ppb,
                    "actual_ppb": actual_ppb,
                    "measured_ppb": meas["freq_offset_ppb"],
                    "interval_std_ps": meas["interval_std_ps"],
                    "n_intervals": meas["n_intervals"],
                })
                log.info("  Measured: %+.3f ppb (std=%.1f ps, n=%d)",
                         meas["freq_offset_ppb"], meas["interval_std_ps"],
                         meas["n_intervals"])

    finally:
        actuator.teardown()

    if len(results) < 2:
        log.error("Need at least 2 successful measurements for calibration")
        return 1

    # Compute calibration metrics
    commanded = [r["actual_ppb"] for r in results]
    measured = [r["measured_ppb"] for r in results]

    # Linear fit: measured = gain * commanded + offset
    n = len(commanded)
    sum_x = sum(commanded)
    sum_y = sum(measured)
    sum_xy = sum(x * y for x, y in zip(commanded, measured))
    sum_xx = sum(x * x for x in commanded)
    denom = n * sum_xx - sum_x * sum_x
    if abs(denom) > 1e-12:
        gain = (n * sum_xy - sum_x * sum_y) / denom
        offset = (sum_y - gain * sum_x) / n
    else:
        gain = 1.0
        offset = 0.0

    # Linearity: max residual from fit
    residuals = [abs(m - (gain * c + offset)) for c, m in zip(commanded, measured)]
    max_residual = max(residuals)

    # Tuning range
    range_ppb = max(measured) - min(measured)

    print(f"\n{'='*60}")
    print(f"DO Calibration Results")
    print(f"{'='*60}")
    print(f"Actuator:       {actuator_label}")
    print(f"Channel:        {args.channel}")
    print(f"Dwell:          {args.dwell}s per point")
    print(f"Points:         {len(results)}")
    print()
    print(f"  {'Commanded':>12s}  {'Measured':>12s}  {'Residual':>10s}  {'Std':>8s}")
    print(f"  {'(ppb)':>12s}  {'(ppb)':>12s}  {'(ppb)':>10s}  {'(ps)':>8s}")
    print(f"  {'-'*12}  {'-'*12}  {'-'*10}  {'-'*8}")
    for r in results:
        resid = r["measured_ppb"] - (gain * r["actual_ppb"] + offset)
        print(f"  {r['actual_ppb']:+12.3f}  {r['measured_ppb']:+12.3f}  "
              f"{resid:+10.3f}  {r['interval_std_ps']:8.1f}")
    print()
    print(f"Gain:           {gain:.6f} (measured/commanded)")
    print(f"Offset:         {offset:+.3f} ppb (at commanded=0)")
    print(f"Tuning range:   {range_ppb:.1f} ppb")
    print(f"Max residual:   {max_residual:.3f} ppb (linearity)")
    print(f"Resolution:     {actuator.resolution_ppb:.4f} ppb/step")

    if hasattr(actuator, '_ppb_per_code'):
        corrected = actuator._ppb_per_code * gain
        print(f"\nCorrected ppb_per_code: {corrected:.6f} "
              f"(was {actuator._ppb_per_code:.6f})")

    # Save results
    if args.output:
        output = {
            "actuator": actuator_label,
            "channel": args.channel,
            "dwell_s": args.dwell,
            "gain": gain,
            "offset_ppb": offset,
            "range_ppb": range_ppb,
            "max_residual_ppb": max_residual,
            "resolution_ppb": actuator.resolution_ppb,
            "points": results,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }
        if hasattr(actuator, '_ppb_per_code'):
            output["ppb_per_code_initial"] = actuator._ppb_per_code
            output["ppb_per_code_corrected"] = actuator._ppb_per_code * gain
        os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
        with open(args.output, "w") as f:
            json.dump(output, f, indent=2)
            f.write("\n")
        log.info("Results saved to %s", args.output)

    return 0


if __name__ == "__main__":
    sys.exit(main())

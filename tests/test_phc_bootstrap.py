#!/usr/bin/env python3
"""test_phc_bootstrap.py — Warm-start bootstrap integration tests.

Runs phc_bootstrap.py under controlled fault conditions and verifies
it makes the right decisions about phase and frequency intervention.

Requires:
  - A working GNSS receiver with dual-frequency observations
  - A PHC device (/dev/ptp_i226 or similar)
  - A populated receiver state file with last_known_position
  - NTRIP connectivity

Tests:
  1. Well-disciplined PHC → bless without intervention
  2. Bad phase, good frequency → step phase only
  3. Bad frequency, good phase → set frequency only
  4. Bad phase + bad frequency → fix both

Usage:
    python3 test_phc_bootstrap.py \
        --serial /dev/gnss-top --baud 115200 --port-type USB \
        --ntrip-conf ntrip.conf --eph-mount BCEP00BKG0 \
        --ptp-dev /dev/ptp_i226 --extts-channel 0 --pps-pin 1 --program-pin
"""

import argparse
import json
import os
import subprocess
import sys
import time

import pytest

# This is a CLI hardware-integration script, not a pytest unit test.
# Its test_* helpers take argparse `args` (which pytest tries to treat
# as a fixture), and it spawns a `phc_bootstrap.py` subprocess that
# drives a real PHC + GNSS + TICC chain.  Module-level skip stops
# `./bin/test` from picking up the four test_* functions as fixture-
# missing errors during routine runs.
#
# To run manually with hardware attached:
#     venv/bin/python tests/test_phc_bootstrap.py --serial /dev/gnss-top \
#         --baud 115200 --port-type USB --ntrip-conf ntrip.conf \
#         --eph-mount BCEP00BKG0 --ptp-dev /dev/ptp_i226 \
#         --extts-channel 0 --pps-pin 1 --program-pin
#
# Also: this test was built around data/drift.json.  Drift storage
# has moved to state/dos/<uid>.json; the fault-injection helpers
# below need to be ported before this integration test is useful
# again.  Kept here as a reference for the test intent.
pytest.skip(
    "Hardware-integration script — run manually under venv with --serial etc. "
    "Pending port from data/drift.json to state/dos/<uid>.json.",
    allow_module_level=True,
)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
from peppar_fix.ptp_device import PtpDevice


def read_drift(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def write_drift(path, adjfine_ppb, phc_dev):
    """[DEPRECATED] Drift storage moved to state/dos/<uid>.json."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        json.dump({"adjfine_ppb": adjfine_ppb, "phc": phc_dev,
                    "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}, f)


def run_bootstrap(args, extra_args=None):
    """Run phc_bootstrap.py, return (exit_code, stdout)."""
    cmd = [
        sys.executable, os.path.join(os.path.dirname(__file__), "phc_bootstrap.py"),
        "--serial", args.serial,
        "--baud", str(args.baud),
        "--port-type", args.port_type,
        "--ntrip-conf", args.ntrip_conf,
        "--eph-mount", args.eph_mount,
        "--systems", args.systems,
        "--ptp-dev", args.ptp_dev,
        "--extts-channel", str(args.extts_channel),
        "--pps-pin", str(args.pps_pin),
        "--phc-timescale", args.phc_timescale,
        "--drift-file", args.drift_file,
        "--epochs", str(args.epochs),
        "--step-error-ns", str(args.step_error_ns),
        "--settime-lag-ns", str(args.settime_lag_ns),
        "--max-pps-iterations", str(args.max_pps_iterations),
    ]
    if args.program_pin:
        cmd.append("--program-pin")
    if extra_args:
        cmd.extend(extra_args)
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    return result.returncode, result.stdout + result.stderr


def parse_output(output):
    """Extract key decisions from bootstrap output."""
    info = {
        "blessed": "blessing without intervention" in output,
        "stepped_phase": "Stepping PHC phase" in output,
        "set_frequency": "Setting PHC frequency" in output,
        "phase_ok": "Phase OK" in output and "leaving PHC time alone" in output,
        "freq_ok": "Frequency OK" in output and "leaving adjfine" in output,
        "drift_updated": "Drift file updated" in output,
    }
    # Extract step residual if present
    for line in output.split("\n"):
        if "Step result:" in line:
            parts = line.split("residual=")[1].split(" ")[0]
            info["step_residual_ns"] = float(parts.replace("+", "").replace("ns", ""))
    return info


def setup_good_state(args):
    """Run bootstrap once to get PHC into a known-good state."""
    print("  Setting up known-good PHC state (running bootstrap)...")
    rc, output = run_bootstrap(args)
    if rc != 0:
        print(f"  WARNING: setup bootstrap returned {rc}")
    # Run again — second run should either bless or get closer
    rc, output = run_bootstrap(args)
    return rc, output


def test_bless_no_intervention(args):
    """Test 1: Well-disciplined PHC should be blessed without intervention."""
    print("\n" + "=" * 60)
    print("TEST 1: Well-disciplined PHC → bless without intervention")
    print("=" * 60)

    # First, get PHC into good state
    setup_good_state(args)

    # Now run bootstrap — it should bless
    print("  Running bootstrap (expecting bless)...")
    rc, output = run_bootstrap(args)
    info = parse_output(output)

    passed = info["blessed"] and not info["stepped_phase"] and not info["set_frequency"]
    print(f"  Exit code: {rc}")
    print(f"  Blessed: {info['blessed']}")
    print(f"  Stepped phase: {info['stepped_phase']}")
    print(f"  Set frequency: {info['set_frequency']}")
    print(f"  RESULT: {'PASS' if passed else 'FAIL'}")
    if not passed:
        print("  --- Output ---")
        for line in output.split("\n"):
            if "INFO" in line or "WARNING" in line or "ERROR" in line:
                print(f"    {line.strip()}")
    return passed


def test_bad_phase_good_freq(args):
    """Test 2: Bad phase should step phase only, leave frequency alone."""
    print("\n" + "=" * 60)
    print("TEST 2: Bad phase, good frequency → step phase only")
    print("=" * 60)

    # Get to good state first
    setup_good_state(args)

    # Save current drift file
    drift_before = read_drift(args.drift_file)

    # Inject phase error: nudge PHC by 5 seconds
    # Open PTP device briefly for fault injection, then release
    print("  Injecting phase fault: +5 seconds...")
    ptp = PtpDevice(args.ptp_dev)
    phc_now, _ = ptp.read_phc_ns()
    ptp.set_phc_ns(phc_now + 5_000_000_000)
    ptp.close()

    # Run bootstrap
    print("  Running bootstrap (expecting phase step only)...")
    rc, output = run_bootstrap(args)
    info = parse_output(output)

    drift_after = read_drift(args.drift_file)

    stepped = info["stepped_phase"]
    freq_untouched = info["freq_ok"] and not info["set_frequency"]
    drift_unchanged = (
        (drift_before is None and drift_after is None) or
        (drift_before is not None and drift_after is not None and
         drift_before.get("adjfine_ppb") == drift_after.get("adjfine_ppb"))
    )

    passed = stepped and freq_untouched and drift_unchanged
    print(f"  Exit code: {rc}")
    print(f"  Stepped phase: {stepped}")
    print(f"  Frequency untouched: {freq_untouched}")
    print(f"  Drift file unchanged: {drift_unchanged}")
    if "step_residual_ns" in info:
        print(f"  Step residual: {info['step_residual_ns']:+.0f} ns")
    print(f"  RESULT: {'PASS' if passed else 'FAIL'}")
    if not passed:
        print("  --- Output ---")
        for line in output.split("\n"):
            if "INFO" in line or "WARNING" in line or "ERROR" in line:
                print(f"    {line.strip()}")
    return passed


def test_good_phase_bad_freq(args):
    """Test 3: Bad frequency should set frequency only, leave phase alone."""
    print("\n" + "=" * 60)
    print("TEST 3: Good phase, bad frequency → set frequency only")
    print("=" * 60)

    # Get to good state first
    setup_good_state(args)

    # Inject frequency error: must be large enough to trigger correction
    # (> freq_tolerance_ppb, default 10) but small enough that drift over
    # the ~25s bootstrap window stays within step_error_ns.
    # 100 ppb * 25s = 2500 ns drift — well within 10µs threshold.
    freq_fault_ppb = 100.0
    print(f"  Injecting frequency fault: +{freq_fault_ppb} ppb...")
    ptp = PtpDevice(args.ptp_dev)
    ptp.adjfine(freq_fault_ppb)
    ptp.close()

    # Write a drift file that reflects the bad frequency
    write_drift(args.drift_file, freq_fault_ppb, args.ptp_dev)

    # Run bootstrap
    print("  Running bootstrap (expecting frequency set only)...")
    rc, output = run_bootstrap(args)
    info = parse_output(output)

    freq_set = info["set_frequency"]
    phase_untouched = info["phase_ok"] and not info["stepped_phase"]

    passed = freq_set and phase_untouched
    print(f"  Exit code: {rc}")
    print(f"  Set frequency: {freq_set}")
    print(f"  Phase untouched: {phase_untouched}")
    print(f"  RESULT: {'PASS' if passed else 'FAIL'}")
    if not passed:
        print("  --- Output ---")
        for line in output.split("\n"):
            if "INFO" in line or "WARNING" in line or "ERROR" in line:
                print(f"    {line.strip()}")
    return passed


def test_bad_phase_bad_freq(args):
    """Test 4: Bad phase + bad frequency → fix both."""
    print("\n" + "=" * 60)
    print("TEST 4: Bad phase + bad frequency → fix both")
    print("=" * 60)

    # Get to good state first
    setup_good_state(args)

    # Inject both faults: open PTP briefly
    import random as _random
    print("  Injecting phase fault: random time...")
    ptp = PtpDevice(args.ptp_dev)
    target_sec = _random.randint(0, 2**31 - 1)
    ptp.set_phc_ns(target_sec * 1_000_000_000)
    freq_fault_ppb = 5000.0
    print(f"  Injecting frequency fault: +{freq_fault_ppb} ppb...")
    ptp.adjfine(freq_fault_ppb)
    ptp.close()

    write_drift(args.drift_file, freq_fault_ppb, args.ptp_dev)

    # Run bootstrap
    print("  Running bootstrap (expecting both interventions)...")
    rc, output = run_bootstrap(args)
    info = parse_output(output)

    both_fixed = info["stepped_phase"] and info["set_frequency"]

    print(f"  Exit code: {rc}")
    print(f"  Stepped phase: {info['stepped_phase']}")
    print(f"  Set frequency: {info['set_frequency']}")
    print(f"  RESULT: {'PASS' if both_fixed else 'FAIL'}")
    if not both_fixed:
        print("  --- Output ---")
        for line in output.split("\n"):
            if "INFO" in line or "WARNING" in line or "ERROR" in line:
                print(f"    {line.strip()}")
    return both_fixed


def main():
    ap = argparse.ArgumentParser(description="PHC bootstrap integration tests")
    ap.add_argument("--serial", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--port-type", default="USB")
    ap.add_argument("--ntrip-conf", required=True)
    ap.add_argument("--eph-mount", required=True)
    ap.add_argument("--ssr-mount", default=None)
    ap.add_argument("--systems", default="gps,gal")
    ap.add_argument("--ptp-dev", required=True)
    ap.add_argument("--extts-channel", type=int, default=0)
    ap.add_argument("--pps-pin", type=int, default=1)
    ap.add_argument("--program-pin", action="store_true")
    ap.add_argument("--phc-timescale", default="tai")
    ap.add_argument("--drift-file", default="data/drift-test.json")
    ap.add_argument("--epochs", type=int, default=10)
    ap.add_argument("--step-error-ns", type=int, default=10000,
                    help="Phase sanity threshold in ns (default: 10000)")
    ap.add_argument("--settime-lag-ns", type=int, default=0,
                    help="Mean clock_settime-to-PHC landing lag in ns (from characterization)")
    ap.add_argument("--max-pps-iterations", type=int, default=8,
                    help="Max PPS feedback iterations (default: 8)")
    ap.add_argument("--tests", default="1,2,3,4",
                    help="Comma-separated test numbers to run (default: 1,2,3,4)")
    args = ap.parse_args()

    # Ensure drift file directory exists
    os.makedirs(os.path.dirname(args.drift_file) or ".", exist_ok=True)

    tests_to_run = set(int(t) for t in args.tests.split(","))

    results = {}
    test_funcs = {
        1: ("bless_no_intervention", test_bless_no_intervention),
        2: ("bad_phase_good_freq", test_bad_phase_good_freq),
        3: ("good_phase_bad_freq", test_good_phase_bad_freq),
        4: ("bad_phase_bad_freq", test_bad_phase_bad_freq),
    }

    for num, (name, func) in test_funcs.items():
        if num not in tests_to_run:
            continue
        try:
            results[name] = func(args)
        except Exception as e:
            print(f"\n  EXCEPTION in test {num}: {e}")
            results[name] = False

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    total = len(results)
    passed = sum(1 for v in results.values() if v)
    for name, result in results.items():
        print(f"  {'PASS' if result else 'FAIL'}  {name}")
    print(f"\n  {passed}/{total} tests passed")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())

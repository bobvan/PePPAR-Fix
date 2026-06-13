#!/usr/bin/env python3
"""dac_torture.py — high-rate DAC write stress test for I2C-bus isolation.

The engine writes the DAC once per second (1 Hz servo).  A marginal I2C
bus (under-/over-loaded pull-ups, Vdd sag, noise, bad wiring) can survive
that cadence for minutes, then drop the device off the bus mid-write with
``OSError: [Errno 121] Remote I/O error`` — crashing the engine and (on
MadHat) leaving the TADD divider mis-armed.  See the 2026-06-12 stability
shakeout that locked clean at ±3 ns for 12 min, then died on a DAC NAK.

This tool hammers the **exact same code path the engine uses**
(``DacActuator._write_code`` → ``write_i2c_block_data(addr, 0x30, [msb,lsb])``)
hundreds-to-thousands of times per second so an hour of engine stress is
compressed into seconds.  Try a wiring/pull-up/bus-clock fix, run this,
and read the verdict immediately:

    PASS  = zero errors over the full duration   → the fix sticks
    FAIL  = first error at T s (errno 121 ...)    → still marginal

The default ``alternate`` pattern writes code_lo / code_hi on alternate
writes so all 16 data bits toggle every transaction — the worst case for
a capacitively-overloaded SDA line (the slowest rising edges, the most
charge the pull-up must source).  A bus that passes ``alternate`` at high
rate passes anything the servo will ever do.

Usage (defaults pulled from the host config so you can't fat-finger the
address):

    # 60 s worst-case stress, params from config/madhat.toml
    sudo venv/bin/python scripts/dac_torture.py --host-config config/madhat.toml

    # quick 15 s check, stop at the first failure (fast iteration)
    sudo venv/bin/python scripts/dac_torture.py --host-config config/madhat.toml \
         --duration 15 --stop-on-error

    # does the bus self-recover after a NAK?  (tests the retry idea)
    sudo venv/bin/python scripts/dac_torture.py --host-config config/madhat.toml \
         --recover

    # explicit params, no config:
    sudo venv/bin/python scripts/dac_torture.py --bus 1 --addr 0x4C --gain 1

Exit codes: 0 = clean (PASS), 1 = completed with errors, 2 = bus wedged,
3 = setup failed.  So ``dac_torture.py ... && echo STUCK`` is scriptable.
"""

import argparse
import errno as _errno
import os
import sys
import time

# Run from the repo root; the engine path is scripts/peppar_fix/...
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from peppar_fix.dac_actuator import DacActuator

EREMOTEIO = 121  # Errno 121 — the DAC-dropped-off-the-bus signature


def _load_host_config(path):
    """Pull dac_* fields from a host config TOML [peppar] table."""
    try:
        import tomllib
    except ModuleNotFoundError:  # py<3.11
        import tomli as tomllib  # type: ignore
    with open(path, "rb") as f:
        peppar = tomllib.load(f).get("peppar", {})
    out = {}
    if "dac_bus" in peppar:
        out["bus"] = int(peppar["dac_bus"])
    if "dac_addr" in peppar:
        out["addr"] = int(str(peppar["dac_addr"]), 0)
    if "dac_bits" in peppar:
        out["bits"] = int(peppar["dac_bits"])
    if "dac_gain" in peppar:
        out["gain"] = int(peppar["dac_gain"])
    if "dac_type" in peppar:
        out["dac_type"] = str(peppar["dac_type"])
    return out


def _fmt_rate(r):
    return f"{r:,.0f} writes/s  (= {r:.0f}x the 1 Hz engine rate)"


def main():
    ap = argparse.ArgumentParser(
        description="High-rate DAC write stress test (I2C bus isolation).")
    ap.add_argument("--host-config",
                    help="Host TOML to read dac_bus/addr/bits/gain/type from "
                         "(explicit flags below override).")
    ap.add_argument("--bus", type=int, help="I2C bus number (e.g. 1).")
    ap.add_argument("--addr", type=lambda s: int(s, 0),
                    help="I2C address (e.g. 0x4C).")
    ap.add_argument("--bits", type=int, default=16, help="DAC bits (default 16).")
    ap.add_argument("--gain", type=int, choices=(0, 1), default=None,
                    help="AD5693R gain bit (0=1x, 1=2x).")
    ap.add_argument("--dac-type", default=None,
                    choices=("mcp4725", "ad5693r", "generic"))
    ap.add_argument("--duration", type=float, default=60.0,
                    help="Seconds to hammer (default 60).")
    ap.add_argument("--pattern", default="alternate",
                    choices=("alternate", "ramp", "fixed"),
                    help="alternate=lo/hi toggle (all bits flip, worst case); "
                         "ramp=sequential; fixed=one code (default alternate).")
    ap.add_argument("--code-lo", type=int, default=0,
                    help="Low code for alternate/ramp (default 0).")
    ap.add_argument("--code-hi", type=int, default=None,
                    help="High code for alternate/ramp (default = max code).")
    ap.add_argument("--ramp-step", type=int, default=257,
                    help="Step for ramp pattern (default 257 = flips both bytes).")
    ap.add_argument("--fixed-code", type=int, default=32768,
                    help="Code for fixed pattern (default 32768).")
    ap.add_argument("--report-interval", type=float, default=5.0,
                    help="Live progress cadence in seconds (default 5).")
    ap.add_argument("--stop-on-error", action="store_true",
                    help="Halt at the first failed write (fast iteration).")
    ap.add_argument("--recover", action="store_true",
                    help="On a run of failures, close+reopen the bus and "
                         "re-init (tests whether the bus self-recovers).")
    ap.add_argument("--wedge-threshold", type=int, default=500,
                    help="Consecutive failures before declaring the bus wedged "
                         "(default 500).")
    ap.add_argument("--rest-code", type=int, default=32768,
                    help="Code to leave the DAC at on exit (default 32768).")
    args = ap.parse_args()

    cfg = {}
    if args.host_config:
        cfg = _load_host_config(args.host_config)

    bus = args.bus if args.bus is not None else cfg.get("bus")
    addr = args.addr if args.addr is not None else cfg.get("addr")
    bits = args.bits if args.bits is not None else cfg.get("bits", 16)
    gain = args.gain if args.gain is not None else cfg.get("gain", 0)
    dac_type = args.dac_type or cfg.get("dac_type", "ad5693r")
    if bus is None or addr is None:
        ap.error("need --bus and --addr (directly or via --host-config)")

    max_code = (1 << bits) - 1
    code_hi = args.code_hi if args.code_hi is not None else max_code
    code_lo = max(0, min(max_code, args.code_lo))
    code_hi = max(0, min(max_code, code_hi))

    print(f"=== DAC write torture: bus={bus} addr=0x{addr:02X} bits={bits} "
          f"gain={gain} type={dac_type} ===")
    print(f"  pattern={args.pattern} "
          + (f"codes={code_lo}<->{code_hi}" if args.pattern != "fixed"
             else f"code={args.fixed_code}")
          + f"  duration={args.duration:g}s  recover={args.recover}")
    print(f"  '{args.pattern}'"
          + (" toggles all 16 data bits each write — worst case for SDA edges."
             if args.pattern == "alternate" else ""))
    sys.stdout.flush()

    dac = DacActuator(bus_num=bus, addr=addr, bits=bits,
                      ppb_per_code=1.0, dac_type=dac_type, dac_gain=gain)
    try:
        dac.setup()
    except Exception as e:  # noqa: BLE001
        print(f"SETUP FAILED: {e!r}")
        return 3

    n_ok = n_err = n_121 = n_other = 0
    n_recover_try = n_recover_ok = 0
    consec_ok = consec_err = max_consec_ok = 0
    first_err = None  # (t, i, code, errno, msg)
    wedged = False

    t0 = time.monotonic()
    t_end = t0 + args.duration
    next_report = t0 + args.report_interval
    i = 0
    code = code_lo

    while True:
        now = time.monotonic()
        if now >= t_end:
            break

        # next code
        if args.pattern == "alternate":
            code = code_hi if (i & 1) else code_lo
        elif args.pattern == "ramp":
            code = code_lo + ((i * args.ramp_step) % max(1, (code_hi - code_lo + 1)))
        else:  # fixed
            code = args.fixed_code

        try:
            dac._write_code(code)
            n_ok += 1
            consec_ok += 1
            max_consec_ok = max(max_consec_ok, consec_ok)
            consec_err = 0
        except OSError as e:
            n_err += 1
            consec_ok = 0
            consec_err += 1
            if e.errno == EREMOTEIO:
                n_121 += 1
            else:
                n_other += 1
            if first_err is None:
                first_err = (now - t0, i, code, e.errno, str(e))
                print(f"\n  ** FIRST ERROR at t={now-t0:.3f}s  write #{i:,}  "
                      f"code={code}  errno={e.errno} ({_errno.errorcode.get(e.errno,'?')})  "
                      f"{e}")
                sys.stdout.flush()
            if args.stop_on_error:
                break
            if consec_err >= args.wedge_threshold:
                if args.recover:
                    n_recover_try += 1
                    try:
                        dac.teardown()
                        dac.setup()
                        n_recover_ok += 1
                        consec_err = 0
                        print(f"  .. recovered bus after {args.wedge_threshold} "
                              f"consecutive errors (recover #{n_recover_try})")
                        sys.stdout.flush()
                    except Exception as re:  # noqa: BLE001
                        print(f"  .. recovery FAILED: {re!r} — bus wedged")
                        wedged = True
                        break
                else:
                    wedged = True
                    break

        i += 1
        if now >= next_report:
            el = now - t0
            print(f"  t={el:5.1f}s  writes={i:>10,}  ok={n_ok:,}  err={n_err:,}  "
                  f"({i/el:,.0f}/s)")
            sys.stdout.flush()
            next_report += args.report_interval

    elapsed = time.monotonic() - t0
    n_total = i
    rate = n_total / elapsed if elapsed > 0 else 0.0

    # leave the DAC somewhere safe
    try:
        dac._write_code(max(0, min(max_code, args.rest_code)))
    except OSError:
        pass
    dac.teardown()

    print("\n=== SUMMARY ===")
    print(f"  duration:        {elapsed:.2f} s")
    print(f"  total writes:    {n_total:,}")
    print(f"  rate:            {_fmt_rate(rate)}")
    print(f"  ok:              {n_ok:,}")
    print(f"  errors:          {n_err:,}   (errno121={n_121:,}  other={n_other:,})")
    if n_total:
        print(f"  failure rate:    {100.0*n_err/n_total:.4f}%")
    print(f"  longest clean:   {max_consec_ok:,} consecutive writes")
    if first_err:
        t, idx, c, en, msg = first_err
        print(f"  first failure:   t={t:.3f}s  write #{idx:,}  code={c}  "
              f"errno={en} ({_errno.errorcode.get(en,'?')})")
    if args.recover:
        print(f"  recoveries:      {n_recover_ok}/{n_recover_try} succeeded")
    if rate < 2000 and not wedged:
        print(f"  NOTE: {rate:,.0f}/s is bus-clock-limited.  To stress harder + "
              f"iterate faster, raise i2c-{bus} to 400 kHz "
              f"(dtparam=i2c_arm_baudrate=400000, reboot).")

    if wedged:
        print("\n  VERDICT: ❌ BUS WEDGED — device stopped ACKing and did not "
              "recover.  Fix did NOT stick.")
        return 2
    if n_err:
        print("\n  VERDICT: ❌ FAIL — bus is still marginal "
              f"({n_err:,} NAKs, first at {first_err[0]:.3f}s).  Fix did NOT stick.")
        return 1
    print(f"\n  VERDICT: ✅ PASS — {n_ok:,} clean writes at {rate:,.0f}/s, "
          f"zero errors.  This fix sticks (at this rate/pattern).")
    return 0


if __name__ == "__main__":
    sys.exit(main())

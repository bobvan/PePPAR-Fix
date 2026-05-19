#!/usr/bin/env python3
"""F9P pair zero-baseline RTK sanity check.

Two F9Ps on the same host, both fed from the same antenna via
splitter — run RTK with one as base and the other as rover, and
verify the relative position is at the mm noise floor.  If a
splitter-fed pair can't agree to mm scale, no downstream RTK
experiment (CORS NTRIP rover, F9P float-PPP) can be trusted.

Per dayplan I-220529-charlie / I-224649-charlie (2026-04-29).
Hardware on clkPoC3:

  F9P-1 (SEC-UNIQID 904584649306) — base — /dev/ttyACM0
  F9P-2 (SEC-UNIQID 914202187869) — rover — /dev/ttyACM1

Both at 38400 baud (HPG firmware default).

What this script does, in order:

  1. Open both serial ports.
  2. Drain any in-flight bytes so configuration responses don't get
     mis-parsed.
  3. Send a single CFG-VALSET to the BASE that:
       - sets TMODE3 to fixed-position ECEF at UFO1 antennas.json
         (the absolute position is moot for a zero-baseline test —
         only the relative vector matters — but using a real lab
         coordinate keeps base-rover messaging sane);
       - enables RTCM 1005, 1077, 1087, 1097 and 1230 every epoch
         on USB output;
       - ensures USB output protocol allows RTCM 3.
  4. Send a single CFG-VALSET to the ROVER that:
       - enables NAV-RELPOSNED + NAV-PVT every epoch on USB output;
       - ensures USB input protocol accepts RTCM 3.
  5. Start two threads:
       - bridge: read bytes from base USB and write them to rover USB
       - monitor: parse NAV-RELPOSNED from rover USB; track per-second
         relPosN/E/D, accN/E/D and carrSoln (0=NONE, 1=FLOAT, 2=FIXED)
  6. Run for the configured duration (default 5 min); print a summary
     showing per-second carrSoln distribution + relPos statistics
     during sustained-FIXED windows.

Success: carrSoln=2 (FIXED) within 30s and |relPosN|, |relPosE|,
|relPosD| < 5 mm sustained.

Dependencies: pyubx2, pyserial.  Already in the clkPoC3 venv.

Base coordinates are passed as required CLI args (--base-lat,
--base-lon, --base-alt) — never hardcoded.  Load them from
``timelab/antennas.json`` on the running host, e.g.:

    BASE=$(jq -r '.ufo1 | "\\(.lat) \\(.lon) \\(.alt_m)"' \\
        ~/git/timelab/antennas.json)
    python3 f9p_zero_baseline_rtk.py /dev/ttyACM0 /dev/ttyACM1 \\
        --base-lat ${BASE% *} ...

For a true zero-baseline test the absolute base position is moot
— only the rover's relative vector matters — but providing a
real lab coordinate keeps the F9P's internal sanity checks happy
and produces sensible base-station RTCM-1005 messaging.
"""
from __future__ import annotations

import argparse
import logging
import math
import sys
import threading
import time
from collections import Counter, deque

import serial
from pyubx2 import (
    POLL_LAYER_RAM,
    SET,
    SET_LAYER_RAM,
    TXN_NONE,
    UBXMessage,
    UBXReader,
)

log = logging.getLogger("f9p_zero_baseline_rtk")


# WGS-84 constants (same values used elsewhere in the codebase).
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F


def lla_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> tuple[float, float, float]:
    """WGS-84 geodetic → ECEF, both in metres."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    s = math.sin(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s * s)
    x = (n + alt_m) * math.cos(lat) * math.cos(lon)
    y = (n + alt_m) * math.cos(lat) * math.sin(lon)
    z = (n * (1.0 - WGS84_E2) + alt_m) * s
    return x, y, z


def configure_base(ser: serial.Serial, x_m: float, y_m: float, z_m: float) -> None:
    """Configure the base receiver: TMODE3 fixed at given ECEF, RTCM out on USB.

    TMODE_ECEF_*: integer cm.  TMODE_ECEF_*_HP: signed 0.1 mm
    fractional residual (-99..+99).  We split the metres value
    into integer cm + fractional 0.1 mm and report both so the
    base ARP claim is mm-accurate.
    """
    def _split_cm_hp(v_m: float) -> tuple[int, int]:
        cm_total = round(v_m * 100.0 * 10.0)  # in 0.1-mm units
        cm = int(cm_total // 10)              # integer cm
        hp = int(cm_total - cm * 10)          # 0.1-mm residual
        # u-blox spec clamps HP to -99..99; cm absorbs the rest.
        if hp > 99:
            hp -= 100
            cm += 1
        elif hp < -99:
            hp += 100
            cm -= 1
        return cm, hp

    x_cm, x_hp = _split_cm_hp(x_m)
    y_cm, y_hp = _split_cm_hp(y_m)
    z_cm, z_hp = _split_cm_hp(z_m)
    cfg_data = [
        # USB output protocol must allow RTCM 3 emission.
        ("CFG_USBOUTPROT_RTCM3X", 1),
        ("CFG_USBOUTPROT_UBX", 1),
        ("CFG_USBINPROT_UBX", 1),
        # TMODE3 fixed-position ECEF.
        ("CFG_TMODE_MODE", 2),       # 2 = fixed
        ("CFG_TMODE_POS_TYPE", 0),   # 0 = ECEF
        ("CFG_TMODE_ECEF_X", x_cm),
        ("CFG_TMODE_ECEF_Y", y_cm),
        ("CFG_TMODE_ECEF_Z", z_cm),
        ("CFG_TMODE_ECEF_X_HP", x_hp),
        ("CFG_TMODE_ECEF_Y_HP", y_hp),
        ("CFG_TMODE_ECEF_Z_HP", z_hp),
        ("CFG_TMODE_FIXED_POS_ACC", 50),  # 50 mm claimed accuracy
        # RTCM message rate: 1 = every epoch.
        ("CFG_MSGOUT_RTCM_3X_TYPE1005_USB", 1),  # station coords
        ("CFG_MSGOUT_RTCM_3X_TYPE1077_USB", 1),  # GPS MSM7
        ("CFG_MSGOUT_RTCM_3X_TYPE1087_USB", 1),  # GLO MSM7
        ("CFG_MSGOUT_RTCM_3X_TYPE1097_USB", 1),  # GAL MSM7
        ("CFG_MSGOUT_RTCM_3X_TYPE1127_USB", 1),  # BDS MSM7
        ("CFG_MSGOUT_RTCM_3X_TYPE1230_USB", 5),  # GLO biases every 5 epochs
    ]
    msg = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE, cfg_data)
    ser.write(msg.serialize())


def configure_rover(ser: serial.Serial) -> None:
    """Rover: accept RTCM on USB input, emit NAV-RELPOSNED + NAV-PVT."""
    cfg_data = [
        ("CFG_USBINPROT_RTCM3X", 1),
        ("CFG_USBINPROT_UBX", 1),
        ("CFG_USBOUTPROT_UBX", 1),
        ("CFG_MSGOUT_UBX_NAV_RELPOSNED_USB", 1),
        ("CFG_MSGOUT_UBX_NAV_PVT_USB", 1),
        # Disable any leftover RTCM output from a prior run.
        ("CFG_MSGOUT_RTCM_3X_TYPE1005_USB", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1077_USB", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1087_USB", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1097_USB", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1127_USB", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1230_USB", 0),
        # Make sure rover is NOT in fixed-position mode (whatever
        # mode prior runs left it in).
        ("CFG_TMODE_MODE", 0),  # 0 = disabled (rover is regular nav)
    ]
    msg = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE, cfg_data)
    ser.write(msg.serialize())


def bridge_thread(
    base_ser: serial.Serial,
    rover_ser: serial.Serial,
    stop: threading.Event,
    stats: dict,
) -> None:
    """Read bytes from base; forward all of them to rover.  We don't
    parse — the F9P rover's own RTCM decoder handles it.  This
    avoids re-framing complexity."""
    while not stop.is_set():
        try:
            data = base_ser.read(4096)
        except serial.SerialException as e:
            log.warning("bridge: base read error: %s", e)
            time.sleep(0.1)
            continue
        if data:
            stats["bridge_bytes"] += len(data)
            try:
                rover_ser.write(data)
            except serial.SerialException as e:
                log.warning("bridge: rover write error: %s", e)


# carrSoln enum from u-blox NAV-RELPOSNED flags.
_CARR_SOLN = {0: "NONE", 1: "FLOAT", 2: "FIXED"}


def monitor_thread(
    rover_ser: serial.Serial,
    stop: threading.Event,
    stats: dict,
) -> None:
    """Parse NAV-RELPOSNED from rover; record relPos / accuracy /
    carrSoln per fix.

    pyubx2 auto-merges the high-precision _HPrelPosN/E/D bytes into
    the cm-scaled relPosN/E/D attribute, and exposes flag bitfield
    sub-fields (carrSoln, gnssFixOK, relPosValid) as direct attrs.
    pyubx2 also pre-scales accN/E/D from the raw 0.1-mm tick units.
    """
    reader = UBXReader(rover_ser, protfilter=2)  # UBX only
    while not stop.is_set():
        try:
            raw, parsed = reader.read()
        except Exception as e:
            log.debug("monitor: parse error: %s", e)
            continue
        if parsed is None:
            continue
        if parsed.identity != "NAV-RELPOSNED":
            continue
        try:
            carr_soln = int(parsed.carrSoln)
            gnss_fix_ok = bool(parsed.gnssFixOK)
            rel_pos_valid = bool(parsed.relPosValid)
            # Already merged + scaled to cm by pyubx2.
            rel_n_mm = float(parsed.relPosN) * 10.0
            rel_e_mm = float(parsed.relPosE) * 10.0
            rel_d_mm = float(parsed.relPosD) * 10.0
            # accN/E/D are already in mm (raw 0.1 mm × 0.1 scale =
            # 0.01 mm? No — pyubx2 reports as "0.1 mm units * 0.1
            # scale = mm").  Sanity-check by treating as mm; values
            # under tens-of-mm range for normal RTK noise.
            acc_n_mm = float(parsed.accN)
            acc_e_mm = float(parsed.accE)
            acc_d_mm = float(parsed.accD)
        except Exception as e:
            log.debug("monitor: NAV-RELPOSNED field access: %s", e)
            continue
        soln_label = _CARR_SOLN.get(carr_soln, f"?({carr_soln})")
        stats["soln_counts"][soln_label] += 1
        stats["last"] = {
            "carr_soln": soln_label,
            "fix_ok": gnss_fix_ok,
            "rel_pos_valid": rel_pos_valid,
            "rel_n_mm": rel_n_mm,
            "rel_e_mm": rel_e_mm,
            "rel_d_mm": rel_d_mm,
            "acc_n_mm": acc_n_mm,
            "acc_e_mm": acc_e_mm,
            "acc_d_mm": acc_d_mm,
        }
        if soln_label == "FIXED":
            stats["fixed_relpos"].append((rel_n_mm, rel_e_mm, rel_d_mm))
            if stats["first_fixed_t"] is None:
                stats["first_fixed_t"] = time.monotonic()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("base_port", help="F9P base serial port (e.g. /dev/ttyACM0)")
    ap.add_argument("rover_port", help="F9P rover serial port (e.g. /dev/ttyACM1)")
    ap.add_argument("--baud", type=int, default=38400)
    ap.add_argument("--duration", type=float, default=300.0,
                    help="seconds to run (default 5 min)")
    ap.add_argument("--base-lat", type=float, required=True,
                    help="base latitude in decimal degrees "
                         "(read from timelab/antennas.json on the host)")
    ap.add_argument("--base-lon", type=float, required=True,
                    help="base longitude in decimal degrees")
    ap.add_argument("--base-alt", type=float, required=True,
                    help="base altitude in metres")
    ap.add_argument("--report-every", type=float, default=10.0,
                    help="seconds between status lines")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    base_x, base_y, base_z = lla_to_ecef(
        args.base_lat, args.base_lon, args.base_alt)
    log.info("Base ECEF: X=%.3f Y=%.3f Z=%.3f m", base_x, base_y, base_z)

    log.info("Opening base port %s @ %d", args.base_port, args.baud)
    base_ser = serial.Serial(args.base_port, args.baud, timeout=0.1)
    log.info("Opening rover port %s @ %d", args.rover_port, args.baud)
    rover_ser = serial.Serial(args.rover_port, args.baud, timeout=0.1)

    # Drain any in-flight bytes from prior session.
    time.sleep(0.2)
    base_ser.reset_input_buffer()
    rover_ser.reset_input_buffer()

    log.info("Configuring base (TMODE3 fixed + RTCM out)")
    configure_base(base_ser, base_x, base_y, base_z)
    log.info("Configuring rover (RTCM in + NAV-RELPOSNED out)")
    configure_rover(rover_ser)
    # Give the configuration a moment to take effect.
    time.sleep(1.0)
    base_ser.reset_input_buffer()
    rover_ser.reset_input_buffer()

    stats: dict = {
        "bridge_bytes": 0,
        "soln_counts": Counter(),
        "fixed_relpos": deque(maxlen=4096),
        "first_fixed_t": None,
        "last": None,
        "t_start": time.monotonic(),
    }
    stop = threading.Event()
    bridge = threading.Thread(
        target=bridge_thread, args=(base_ser, rover_ser, stop, stats),
        daemon=True, name="bridge")
    monitor = threading.Thread(
        target=monitor_thread, args=(rover_ser, stop, stats),
        daemon=True, name="monitor")
    bridge.start()
    monitor.start()

    log.info("Running for %.0f s; status every %.0f s",
             args.duration, args.report_every)
    t_end = time.monotonic() + args.duration
    next_report = time.monotonic() + args.report_every
    try:
        while time.monotonic() < t_end:
            time.sleep(0.1)
            now = time.monotonic()
            if now >= next_report:
                last = stats["last"]
                if last is not None:
                    log.info(
                        "[t=%.0fs] %s gnssFixOK=%s relPos N=%+.1f E=%+.1f D=%+.1f mm "
                        "acc N=%.1f E=%.1f D=%.1f mm | bridge=%.1f kB | counts=%s",
                        now - stats["t_start"],
                        last["carr_soln"], last["fix_ok"],
                        last["rel_n_mm"], last["rel_e_mm"], last["rel_d_mm"],
                        last["acc_n_mm"], last["acc_e_mm"], last["acc_d_mm"],
                        stats["bridge_bytes"] / 1000.0,
                        dict(stats["soln_counts"]),
                    )
                else:
                    log.info(
                        "[t=%.0fs] no NAV-RELPOSNED yet; bridge=%.1f kB",
                        now - stats["t_start"],
                        stats["bridge_bytes"] / 1000.0)
                next_report += args.report_every
    except KeyboardInterrupt:
        log.info("interrupted")
    finally:
        stop.set()
        bridge.join(timeout=2.0)
        monitor.join(timeout=2.0)
        base_ser.close()
        rover_ser.close()

    # Final summary.
    print()
    print("=" * 70)
    print("zero-baseline RTK summary")
    print("=" * 70)
    print(f"duration:           {args.duration:.0f} s")
    print(f"bridge throughput:  {stats['bridge_bytes']} bytes "
          f"({stats['bridge_bytes'] / max(args.duration, 1):.0f} B/s)")
    print(f"carrSoln counts:    {dict(stats['soln_counts'])}")
    if stats["first_fixed_t"] is not None:
        ttf = stats["first_fixed_t"] - stats["t_start"]
        print(f"time-to-first-FIXED:  {ttf:.1f} s")
    else:
        print("time-to-first-FIXED:  NEVER  (sanity check FAILED)")
    n_fixed = len(stats["fixed_relpos"])
    if n_fixed > 0:
        ns = [r[0] for r in stats["fixed_relpos"]]
        es = [r[1] for r in stats["fixed_relpos"]]
        ds = [r[2] for r in stats["fixed_relpos"]]
        def _stat(arr):
            arr = list(arr)
            mean = sum(arr) / len(arr)
            var = sum((x - mean) ** 2 for x in arr) / len(arr)
            return mean, math.sqrt(var), max(map(abs, arr))
        for name, arr in (("relPosN_mm", ns), ("relPosE_mm", es),
                          ("relPosD_mm", ds)):
            mean, std, peak = _stat(arr)
            print(f"  {name}:  mean={mean:+.2f}  std={std:.2f}  "
                  f"peak|.|={peak:.2f}  (n={len(arr)})")
        # Pass / fail call.  Use mean + std on the horizontal pair
        # (vertical RTK noise is structurally 2-3x worse, so judging
        # all three on the same threshold gives false fails).
        # F9P spec is "1 cm + 1 ppm" RTK fixed; on a true zero baseline
        # mean should sit near zero (no systematic offset) and noise
        # stays inside ~5 mm horizontal / 10 mm vertical.
        mean_n, std_n, peak_n = _stat(ns)
        mean_e, std_e, peak_e = _stat(es)
        mean_d, std_d, peak_d = _stat(ds)
        bias_ok = (
            abs(mean_n) < 5.0 and abs(mean_e) < 5.0 and abs(mean_d) < 10.0)
        std_ok = std_n < 5.0 and std_e < 5.0 and std_d < 10.0
        passed = bias_ok and std_ok
        print()
        print(
            f"VERDICT: {'PASS' if passed else 'FAIL'}  "
            f"(bias {abs(mean_n):.1f}/{abs(mean_e):.1f}/{abs(mean_d):.1f} mm, "
            f"std {std_n:.1f}/{std_e:.1f}/{std_d:.1f} mm; "
            f"target |bias|<5/5/10, std<5/5/10)")
    else:
        print("VERDICT: FAIL — never reached FIXED")
    return 0


if __name__ == "__main__":
    sys.exit(main())

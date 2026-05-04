#!/usr/bin/env python3
"""F9P pair as two independent CORS NTRIP rovers — UFO1 ARP fix.

Both F9Ps share UFO1 via the lab splitter; each is wired to a
*different* CORS mountpoint (NTRIP, single-base RTK).  Each F9P
solves an independent fixed-RTK position from its caster.  The
mean ECEF of each, after the run, is an estimate of UFO1's ARP.
The diff between the two estimates exposes per-caster bias plus
per-receiver / per-splitter-port bias.

Usage (one pass):

    python3 f9p_dual_cors_rover.py \\
        --r1-port /dev/ttyACM0 --r1-mount NAPERVILLE-RTCM3.1-MSM5 \\
                                --r1-caster 50.149.86.86 --r1-port-tcp 12054 \\
        --r2-port /dev/ttyACM1 --r2-mount WHEATON-RTCM3 \\
                                --r2-caster 50.149.86.86 --r2-port-tcp 12055 \\
        --user VANVALZAH --password 8888 --duration 1800

To separate caster bias from receiver+port bias, run a second
pass with the --r1 and --r2 mountpoints swapped (same physical
F9Ps, same USB ports, swapped corrections).  Bob's protocol per
2026-04-29.

Per dayplan I-220529-charlie #1 follow-on (ARP fix).  Hardware on
clkPoC3:

  F9P-1 (SEC-UNIQID 904584649306) — /dev/ttyACM0
  F9P-2 (SEC-UNIQID 914202187869) — /dev/ttyACM1

Both 38400 baud, HPG 1.51.  CORS credentials live in
``timelab/cors-access/README.md`` (private repo) — pass via CLI
or env (NTRIP_USER / NTRIP_PASS).
"""
from __future__ import annotations

import argparse
import logging
import math
import os
import sys
import threading
import time
from collections import Counter

import serial
from pyubx2 import (
    SET_LAYER_RAM,
    TXN_NONE,
    UBXMessage,
    UBXReader,
)

# Local NtripStream from scripts/ntrip_client.py.  Both this script
# and ntrip_client.py live in scripts/, so a sibling import works
# when run from the peppar-fix repo root or scripts/ directly.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ntrip_client import NtripStream  # noqa: E402

log = logging.getLogger("f9p_dual_cors_rover")


# ---------- WGS-84 helpers ------------------------------------------------

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F


def ecef_to_lla(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Bowring 1976 closed-form ECEF→LLA (good to ~mm at lab altitude)."""
    a = WGS84_A
    e2 = WGS84_E2
    b = a * math.sqrt(1.0 - e2)
    ep2 = (a * a - b * b) / (b * b)
    p = math.hypot(x, y)
    th = math.atan2(z * a, p * b)
    lon = math.atan2(y, x)
    lat = math.atan2(
        z + ep2 * b * math.sin(th) ** 3,
        p - e2 * a * math.cos(th) ** 3,
    )
    n = a / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
    alt = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), alt


def ecef_diff_to_enu(
    dx: float, dy: float, dz: float,
    ref_lat_deg: float, ref_lon_deg: float,
) -> tuple[float, float, float]:
    """Rotate a small ECEF displacement into local ENU at ref_lat/lon."""
    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    e = -so * dx + co * dy
    n = -sl * co * dx - sl * so * dy + cl * dz
    u = cl * co * dx + cl * so * dy + sl * dz
    return e, n, u


# ---------- F9P configuration --------------------------------------------


def configure_rover(ser: serial.Serial) -> None:
    """Configure an F9P as plain RTK rover.

    - TMODE_MODE = 0 (rover, not survey/fixed).
    - USB input accepts RTCM 3 + UBX (we won't send UBX via USB once
      configured, but keep it for future tweaks).
    - USB output emits UBX (NAV-HPPOSECEF + NAV-PVT) and disables
      any leftover RTCM output from a prior run.
    """
    cfg = [
        ("CFG_USBINPROT_RTCM3X", 1),
        ("CFG_USBINPROT_UBX", 1),
        ("CFG_USBOUTPROT_UBX", 1),
        ("CFG_USBOUTPROT_RTCM3X", 0),
        ("CFG_TMODE_MODE", 0),
        # Outputs: HPPOSECEF every epoch is the primary measurement;
        # PVT every epoch gives carrSoln + numSV for sanity.
        ("CFG_MSGOUT_UBX_NAV_HPPOSECEF_USB", 1),
        ("CFG_MSGOUT_UBX_NAV_PVT_USB", 1),
        # Disable noisy outputs that could compete for USB bandwidth.
        ("CFG_MSGOUT_UBX_NAV_RELPOSNED_USB", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1005_USB", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1077_USB", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1087_USB", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1097_USB", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1127_USB", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1230_USB", 0),
    ]
    msg = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE, cfg)
    ser.write(msg.serialize())


# ---------- Threads ------------------------------------------------------


def ntrip_to_usb_thread(
    label: str,
    stream: NtripStream,
    targets: list[tuple[str, serial.Serial, dict]],
    stop: threading.Event,
) -> None:
    """Pull RTCM frames from one NTRIP mountpoint; broadcast each
    frame verbatim to one or more F9P USB ports.  We use raw_frames()
    (which yields complete RTCM messages) rather than streaming raw
    socket bytes — this guarantees we never send a half-message and
    lets us tally per-message-type counts for diagnostics.

    Each target is a (sub_label, serial.Serial, stats_dict) triple.
    Per-target byte / msg counts are tallied independently in case
    one USB write fails.  The Leica casters at this lab enforce a
    one-data-stream-per-source-IP limit, so the broadcast pattern
    (one NTRIP, two F9Ps) is how we feed both rovers from a single
    caster simultaneously.
    """
    log.info("%s: NTRIP→USB thread starting (%d targets)",
             label, len(targets))
    try:
        for msg_type, frame in stream.raw_frames():
            if stop.is_set():
                break
            for sub_label, ser, st in targets:
                try:
                    ser.write(frame)
                except serial.SerialException as e:
                    log.warning("%s→%s: USB write error: %s",
                                label, sub_label, e)
                    continue
                st["bytes"] += len(frame)
                st["msg_counts"][msg_type] += 1
    except Exception as e:
        log.error("%s: NTRIP thread crashed: %s", label, e)
    finally:
        try:
            stream.disconnect()
        except Exception:
            pass
        log.info("%s: NTRIP→USB thread exiting", label)


# carrSoln enum from u-blox NAV-PVT/NAV-RELPOSNED.
_CARR_SOLN = {0: "NONE", 1: "FLOAT", 2: "FIXED"}


def monitor_thread(
    label: str,
    ser: serial.Serial,
    stop: threading.Event,
    stats: dict,
) -> None:
    """Parse NAV-HPPOSECEF + NAV-PVT from the rover.

    NAV-HPPOSECEF gives the high-precision ECEF position (cm-scaled,
    pyubx2 auto-merges the _HP residual, so parsed.ecefX is already
    cm with 0.01 cm = 0.1 mm precision).  NAV-PVT gives carrSoln
    so we can partition the HPPOSECEF samples by fix quality.

    The NAV-HPPOSECEF and NAV-PVT messages share the same iTOW.  We
    keep the most recent NAV-PVT carrSoln as the label for incoming
    HPPOSECEF samples — at 1 Hz on a single USB stream the off-by-
    one risk is negligible.
    """
    reader = UBXReader(ser, protfilter=2)  # UBX only
    last_carr_soln = 0
    last_num_sv = 0
    while not stop.is_set():
        try:
            raw, parsed = reader.read()
        except Exception as e:
            log.debug("%s: parse error: %s", label, e)
            continue
        if parsed is None:
            continue
        ident = parsed.identity
        if ident == "NAV-PVT":
            try:
                last_carr_soln = int(parsed.carrSoln)
                last_num_sv = int(parsed.numSV)
            except Exception:
                pass
            continue
        if ident != "NAV-HPPOSECEF":
            continue
        try:
            x_cm = float(parsed.ecefX)
            y_cm = float(parsed.ecefY)
            z_cm = float(parsed.ecefZ)
            # pAcc reported in mm (0.1 mm raw × 0.1 scale).
            p_acc_mm = float(parsed.pAcc)
            invalid = bool(parsed.invalidEcef)
        except Exception as e:
            log.debug("%s: HPPOSECEF field access: %s", label, e)
            continue
        if invalid:
            stats["invalid_count"] += 1
            continue
        soln_label = _CARR_SOLN.get(last_carr_soln, f"?({last_carr_soln})")
        stats["soln_counts"][soln_label] += 1
        # Convert cm → m for accumulation.  The HP merge has already
        # happened in pyubx2, so ecefX = "cm with sub-mm precision".
        x_m = x_cm * 0.01
        y_m = y_cm * 0.01
        z_m = z_cm * 0.01
        sample = {
            "x_m": x_m, "y_m": y_m, "z_m": z_m,
            "p_acc_mm": p_acc_mm,
            "carr_soln": soln_label,
            "num_sv": last_num_sv,
            "t_mono": time.monotonic(),
        }
        stats["last"] = sample
        if soln_label == "FIXED":
            stats["fixed_samples"].append((x_m, y_m, z_m, p_acc_mm))
            if stats["first_fixed_t"] is None:
                stats["first_fixed_t"] = time.monotonic()


# ---------- Stats helpers ------------------------------------------------


def _mean_std(arr: list[float]) -> tuple[float, float]:
    if not arr:
        return float("nan"), float("nan")
    m = sum(arr) / len(arr)
    var = sum((x - m) ** 2 for x in arr) / len(arr)
    return m, math.sqrt(var)


def summarize_one(label: str, stats: dict, t_run: float) -> dict | None:
    """Print + return a per-rover summary."""
    print()
    print("=" * 70)
    print(f"{label}: rover summary")
    print("=" * 70)
    n_fixed = len(stats["fixed_samples"])
    n_total = sum(stats["soln_counts"].values())
    print(f"  duration:        {t_run:.1f} s")
    print(f"  RTCM bytes:      {stats['bytes']}")
    msg_breakdown = ", ".join(
        f"{k}:{v}" for k, v in sorted(stats["msg_counts"].items()))
    if msg_breakdown:
        print(f"  RTCM msgs:       {msg_breakdown}")
    print(f"  carrSoln counts: {dict(stats['soln_counts'])}")
    print(f"  invalid:         {stats['invalid_count']}")
    if stats["first_fixed_t"] is None or n_fixed == 0:
        print("  → never reached FIXED — no ARP estimate")
        return None
    ttf = stats["first_fixed_t"] - stats["t_start"]
    print(f"  TTF (NONE→FIXED):  {ttf:.1f} s")
    xs = [s[0] for s in stats["fixed_samples"]]
    ys = [s[1] for s in stats["fixed_samples"]]
    zs = [s[2] for s in stats["fixed_samples"]]
    paccs = [s[3] for s in stats["fixed_samples"]]
    mx, sx = _mean_std(xs)
    my, sy = _mean_std(ys)
    mz, sz = _mean_std(zs)
    mp, _ = _mean_std(paccs)
    lat, lon, alt = ecef_to_lla(mx, my, mz)
    print(f"  fixed epochs:    {n_fixed} / {n_total}")
    print(f"  mean ECEF (m):   X={mx:.4f}  Y={my:.4f}  Z={mz:.4f}")
    print(f"  std  ECEF (mm):  X={sx*1000:.1f}  Y={sy*1000:.1f}  Z={sz*1000:.1f}")
    print(f"  mean LLA:        lat={lat:.8f}°  lon={lon:.8f}°  alt={alt:.3f} m")
    print(f"  mean F9P pAcc:   {mp:.1f} mm")
    return {
        "label": label,
        "mean_ecef": (mx, my, mz),
        "std_ecef_mm": (sx * 1000, sy * 1000, sz * 1000),
        "mean_lla": (lat, lon, alt),
        "n_fixed": n_fixed,
        "n_total": n_total,
        "ttf_s": ttf,
        "mean_pacc_mm": mp,
    }


def summarize_diff(s1: dict | None, s2: dict | None) -> None:
    print()
    print("=" * 70)
    print("Comparison (rover-1 minus rover-2)")
    print("=" * 70)
    if s1 is None or s2 is None:
        print("  one or both rovers never reached FIXED — no comparison")
        return
    x1, y1, z1 = s1["mean_ecef"]
    x2, y2, z2 = s2["mean_ecef"]
    dx, dy, dz = x1 - x2, y1 - y2, z1 - z2
    # ENU at the midpoint LLA is fine for sub-metre diffs.
    mid_lat = 0.5 * (s1["mean_lla"][0] + s2["mean_lla"][0])
    mid_lon = 0.5 * (s1["mean_lla"][1] + s2["mean_lla"][1])
    de, dn, du = ecef_diff_to_enu(dx, dy, dz, mid_lat, mid_lon)
    horiz_mm = math.hypot(de, dn) * 1000.0
    threed_mm = math.sqrt(dx * dx + dy * dy + dz * dz) * 1000.0
    print(f"  ΔECEF (mm):      X={dx*1000:+.1f}  Y={dy*1000:+.1f}  "
          f"Z={dz*1000:+.1f}  |3D|={threed_mm:.1f}")
    print(f"  ΔENU  (mm):      E={de*1000:+.1f}  N={dn*1000:+.1f}  "
          f"U={du*1000:+.1f}  |horiz|={horiz_mm:.1f}")
    print()
    print("  This Δ folds together: per-caster bias (different RTK")
    print("  base sites + different RTCM versions), per-receiver bias,")
    print("  and per-splitter-port bias.  Run a second pass with the")
    print("  caster mountpoints swapped between the two F9Ps to")
    print("  separate caster-bias from receiver+port-bias.")


# ---------- main ---------------------------------------------------------


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--r1-port", required=True, help="rover 1 serial port")
    ap.add_argument("--r1-mount", required=True, help="rover 1 NTRIP mountpoint")
    ap.add_argument("--r1-caster", required=True, help="rover 1 caster host")
    ap.add_argument("--r1-port-tcp", type=int, required=True,
                    help="rover 1 caster TCP port")
    ap.add_argument("--r2-port", help="rover 2 serial port "
                    "(omit for --single mode)")
    ap.add_argument("--r2-mount", help="rover 2 NTRIP mountpoint "
                    "(omit for --shared or --single mode)")
    ap.add_argument("--r2-caster", help="rover 2 caster host "
                    "(omit for --shared or --single mode)")
    ap.add_argument("--r2-port-tcp", type=int,
                    help="rover 2 caster TCP port "
                    "(omit for --shared or --single mode)")
    ap.add_argument("--shared", action="store_true",
                    help="open ONE NTRIP stream (using r1 caster/mount) "
                    "and broadcast to both F9P USB ports.  Use this when "
                    "the caster enforces a per-source-IP session limit "
                    "that prevents two simultaneous mountpoint streams "
                    "(observed on the lab's Leica Spider 7.10.1.168).")
    ap.add_argument("--single", action="store_true",
                    help="single-rover mode: only R1 runs.  Use this when "
                    "only one F9P is wired (e.g. clkPoC3 post-2026-05-03 "
                    "with one F9P removed) or for sequential per-mountpoint "
                    "reproducibility runs.  No diff summary is produced.")
    ap.add_argument("--user", default=os.environ.get("NTRIP_USER"),
                    help="NTRIP user (or env NTRIP_USER)")
    ap.add_argument("--password", default=os.environ.get("NTRIP_PASS"),
                    help="NTRIP password (or env NTRIP_PASS)")
    ap.add_argument("--baud", type=int, default=38400)
    ap.add_argument("--duration", type=float, default=1800.0,
                    help="seconds to run (default 30 min)")
    ap.add_argument("--report-every", type=float, default=30.0,
                    help="seconds between status lines")
    ap.add_argument("--tls", action="store_true",
                    help="force TLS on both casters (default: TLS only on port 443)")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    if not args.user or not args.password:
        log.error("NTRIP credentials missing (pass --user/--password "
                  "or set NTRIP_USER/NTRIP_PASS)")
        return 2

    if args.single and args.shared:
        log.error("--single and --shared are mutually exclusive")
        return 2
    if args.single and args.r2_port:
        log.warning("--single ignores --r2-port")
    if not args.shared and not args.single and (
            args.r2_mount is None or args.r2_caster is None
            or args.r2_port_tcp is None or args.r2_port is None):
        log.error("dual-mount mode requires --r2-port, --r2-mount, "
                  "--r2-caster, and --r2-port-tcp (or pass --shared / --single)")
        return 2

    # Open serial ports (R2 only when not in single mode).
    log.info("Opening rover 1 %s @ %d", args.r1_port, args.baud)
    r1_ser = serial.Serial(args.r1_port, args.baud, timeout=0.1)
    r2_ser = None
    if not args.single:
        log.info("Opening rover 2 %s @ %d", args.r2_port, args.baud)
        r2_ser = serial.Serial(args.r2_port, args.baud, timeout=0.1)

    time.sleep(0.2)
    r1_ser.reset_input_buffer()
    if r2_ser is not None:
        r2_ser.reset_input_buffer()

    log.info("Configuring rover 1 (NTRIP→%s)", args.r1_mount)
    configure_rover(r1_ser)
    if r2_ser is not None:
        log.info("Configuring rover 2 (NTRIP→%s)", args.r2_mount)
        configure_rover(r2_ser)
    time.sleep(1.0)
    r1_ser.reset_input_buffer()
    if r2_ser is not None:
        r2_ser.reset_input_buffer()

    # Build NTRIP streams (no auto-connect; the thread will connect).
    tls1 = True if args.tls else (args.r1_port_tcp == 443)
    s1 = NtripStream(args.r1_caster, args.r1_port_tcp, args.r1_mount,
                     user=args.user, password=args.password, tls=tls1)

    def _new_stats() -> dict:
        return {
            "t_start": time.monotonic(),
            "bytes": 0,
            "msg_counts": Counter(),
            "soln_counts": Counter(),
            "invalid_count": 0,
            "fixed_samples": [],
            "first_fixed_t": None,
            "last": None,
        }

    r1_stats = _new_stats()
    r2_stats = _new_stats()

    stop = threading.Event()

    if args.single:
        r1_label = f"R1 [{args.r1_mount}]"
        threads = [
            threading.Thread(target=ntrip_to_usb_thread,
                             args=(args.r1_mount, s1,
                                   [(r1_label, r1_ser, r1_stats)], stop),
                             daemon=True, name="ntrip-r1"),
            threading.Thread(target=monitor_thread,
                             args=(r1_label, r1_ser, stop, r1_stats),
                             daemon=True, name="mon-r1"),
        ]
        s2 = None
    elif args.shared:
        # One NTRIP, broadcast to both F9Ps.
        r1_label = f"R1 [{args.r1_mount}]"
        r2_label = f"R2 [{args.r1_mount}]"  # same mount, different rover
        threads = [
            threading.Thread(
                target=ntrip_to_usb_thread,
                args=(args.r1_mount, s1,
                      [(r1_label, r1_ser, r1_stats),
                       (r2_label, r2_ser, r2_stats)],
                      stop),
                daemon=True, name="ntrip-shared"),
            threading.Thread(target=monitor_thread,
                             args=(r1_label, r1_ser, stop, r1_stats),
                             daemon=True, name="mon-r1"),
            threading.Thread(target=monitor_thread,
                             args=(r2_label, r2_ser, stop, r2_stats),
                             daemon=True, name="mon-r2"),
        ]
        s2 = None
    else:
        tls2 = True if args.tls else (args.r2_port_tcp == 443)
        s2 = NtripStream(args.r2_caster, args.r2_port_tcp, args.r2_mount,
                         user=args.user, password=args.password, tls=tls2)
        r1_label = f"R1 [{args.r1_mount}]"
        r2_label = f"R2 [{args.r2_mount}]"
        threads = [
            threading.Thread(target=ntrip_to_usb_thread,
                             args=(args.r1_mount, s1,
                                   [(r1_label, r1_ser, r1_stats)], stop),
                             daemon=True, name="ntrip-r1"),
            threading.Thread(target=ntrip_to_usb_thread,
                             args=(args.r2_mount, s2,
                                   [(r2_label, r2_ser, r2_stats)], stop),
                             daemon=True, name="ntrip-r2"),
            threading.Thread(target=monitor_thread,
                             args=(r1_label, r1_ser, stop, r1_stats),
                             daemon=True, name="mon-r1"),
            threading.Thread(target=monitor_thread,
                             args=(r2_label, r2_ser, stop, r2_stats),
                             daemon=True, name="mon-r2"),
        ]
    for t in threads:
        t.start()

    log.info("Running for %.0f s; status every %.0f s",
             args.duration, args.report_every)
    t_end = time.monotonic() + args.duration
    next_report = time.monotonic() + args.report_every
    try:
        while time.monotonic() < t_end:
            time.sleep(0.1)
            now = time.monotonic()
            if now >= next_report:
                rovers = (("R1", r1_stats),) if args.single \
                    else (("R1", r1_stats), ("R2", r2_stats))
                for label, st in rovers:
                    last = st["last"]
                    if last is not None:
                        log.info(
                            "[t=%.0fs] %s %s nSV=%d pAcc=%.1fmm "
                            "ECEF=(%.3f, %.3f, %.3f) | RTCM=%dB | counts=%s",
                            now - st["t_start"], label, last["carr_soln"],
                            last["num_sv"], last["p_acc_mm"],
                            last["x_m"], last["y_m"], last["z_m"],
                            st["bytes"], dict(st["soln_counts"]))
                    else:
                        log.info("[t=%.0fs] %s no NAV-HPPOSECEF yet "
                                 "(RTCM=%dB)",
                                 now - st["t_start"], label, st["bytes"])
                next_report += args.report_every
    except KeyboardInterrupt:
        log.info("interrupted")
    finally:
        stop.set()
        # Closing the sockets is what unblocks the NTRIP threads (they're
        # blocked in _recv()).  raw_frames() will then raise + drop out.
        try:
            s1.disconnect()
        except Exception:
            pass
        if s2 is not None:
            try:
                s2.disconnect()
            except Exception:
                pass
        for t in threads:
            t.join(timeout=2.0)
        r1_ser.close()
        if r2_ser is not None:
            r2_ser.close()

    t_run = time.monotonic() - r1_stats["t_start"]
    s1_summary = summarize_one(f"R1 [{args.r1_mount}]", r1_stats, t_run)
    if not args.single:
        r2_mount_label = args.r1_mount if args.shared else args.r2_mount
        s2_summary = summarize_one(f"R2 [{r2_mount_label}]", r2_stats, t_run)
        summarize_diff(s1_summary, s2_summary)
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
log_observations.py — Log raw GNSS observations from a configured F9T.

Captures UBX-RXM-RAWX, RXM-SFRBX, NAV-PVT, NAV-SAT, and TIM-TP messages
to binary (UBX) and CSV files for offline PPP-AR processing.

Usage:
    python log_observations.py /dev/ttyF9T --baud 460800 --duration 3600
    python log_observations.py /dev/ttyUSB0 --duration 86400 --out data/run1

Output files (in --out directory):
    <prefix>_rawx.csv     — per-epoch per-SV: pseudorange, carrier phase, Doppler, C/N0
    <prefix>_pvt.csv      — position/velocity/time solution
    <prefix>_timtp.csv    — PPS timing: qErr, time-of-week
    <prefix>_raw.ubx      — raw binary UBX stream (for replay/reprocessing)
"""

import argparse
import csv
import os
import signal
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

try:
    from pyubx2 import UBXReader, UBXMessage
    from serial import Serial
except ImportError:
    print("ERROR: requires pyubx2 and pyserial", file=sys.stderr)
    print("  pip install pyubx2 pyserial", file=sys.stderr)
    sys.exit(1)


SHUTDOWN = False


def handle_signal(signum, frame):
    global SHUTDOWN
    SHUTDOWN = True
    print("\nShutting down...", file=sys.stderr)


def gnss_id_name(gnss_id):
    return {0: "GPS", 1: "SBAS", 2: "GAL", 3: "BDS", 5: "QZSS", 6: "GLO"}.get(
        gnss_id, f"GNSS{gnss_id}")


def sv_label(gnss_id, sv_id):
    prefix = {"GPS": "G", "GAL": "E", "BDS": "C", "GLO": "R", "QZSS": "J",
              "SBAS": "S"}
    name = gnss_id_name(gnss_id)
    p = prefix.get(name, "X")
    return f"{p}{sv_id:02d}"


def signal_name(gnss_id, sig_id):
    """Map gnssId + sigId to human-readable signal name."""
    table = {
        (0, 0): "GPS-L1CA", (0, 3): "GPS-L2CL", (0, 4): "GPS-L2CM",
        (0, 6): "GPS-L5I", (0, 7): "GPS-L5Q",
        (2, 0): "GAL-E1C", (2, 1): "GAL-E1B",
        (2, 5): "GAL-E5aI", (2, 6): "GAL-E5aQ",
        (2, 3): "GAL-E5bI", (2, 4): "GAL-E5bQ",
        (3, 0): "BDS-B1I", (3, 5): "BDS-B2aI",
        (3, 1): "BDS-B1C", (3, 7): "BDS-B2I",
    }
    return table.get((gnss_id, sig_id), f"{gnss_id_name(gnss_id)}-sig{sig_id}")


def open_csv(path, fieldnames):
    f = open(path, "w", newline="")
    w = csv.DictWriter(f, fieldnames=fieldnames)
    w.writeheader()
    return f, w


def main():
    ap = argparse.ArgumentParser(
        description="Log raw GNSS observations from a configured F9T")
    ap.add_argument("port", help="Serial port (e.g. /dev/ttyF9T)")
    ap.add_argument("--baud", type=int, default=460800,
                    help="Baud rate (default 460800)")
    ap.add_argument("--duration", type=int, default=0,
                    help="Duration in seconds (0 = run until Ctrl-C)")
    ap.add_argument("--out", default=None,
                    help="Output directory (default: data/)")
    ap.add_argument("--prefix", default=None,
                    help="File prefix (default: peppar_<timestamp>)")
    args = ap.parse_args()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    outdir = Path(args.out) if args.out else Path("data")
    outdir.mkdir(parents=True, exist_ok=True)

    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    prefix = args.prefix or f"peppar_{ts}"

    # Open output files
    ubx_path = outdir / f"{prefix}_raw.ubx"
    ubx_file = open(ubx_path, "wb")

    rawx_file, rawx_writer = open_csv(outdir / f"{prefix}_rawx.csv", [
        "timestamp", "rcv_tow_s", "week", "leap_s",
        "sv_id", "gnss_id", "signal_id",
        "pseudorange_m", "carrier_phase_cy", "doppler_hz",
        "cno_dBHz", "lock_duration_ms",
        "pr_valid", "cp_valid", "half_cycle_resolved",
    ])

    pvt_file, pvt_writer = open_csv(outdir / f"{prefix}_pvt.csv", [
        "timestamp", "fix_type", "num_sv", "lon_deg", "lat_deg", "height_m",
        "h_acc_m", "v_acc_m", "pdop",
        "year", "month", "day", "hour", "min", "sec", "nano_s",
        "tow_ms",
    ])

    timtp_file, timtp_writer = open_csv(outdir / f"{prefix}_timtp.csv", [
        "timestamp", "tow_ms", "tow_sub_ms", "q_err_ps", "week",
        "flags", "ref_info",
    ])

    print(f"PePPAR Fix — Observation Logger")
    print(f"  Port: {args.port} @ {args.baud}")
    print(f"  Output: {outdir}/{prefix}_*")
    if args.duration:
        print(f"  Duration: {args.duration}s")
    else:
        print(f"  Duration: until Ctrl-C")
    print()

    ser = Serial(args.port, baudrate=args.baud, timeout=1)
    ubr = UBXReader(ser, protfilter=2)

    start = time.monotonic()
    counts = {"RXM-RAWX": 0, "RXM-SFRBX": 0, "NAV-PVT": 0,
              "NAV-SAT": 0, "TIM-TP": 0, "other": 0}
    last_status = start

    try:
        while not SHUTDOWN:
            if args.duration and (time.monotonic() - start) >= args.duration:
                break

            try:
                raw, parsed = ubr.read()
            except Exception:
                continue

            if raw is None or parsed is None:
                continue

            now = datetime.now(timezone.utc).isoformat(timespec="milliseconds")

            # Save raw UBX
            ubx_file.write(raw)

            ident = parsed.identity

            if ident == "RXM-RAWX":
                counts["RXM-RAWX"] += 1
                rcv_tow = getattr(parsed, "rcvTow", 0)
                week = getattr(parsed, "week", 0)
                leap = getattr(parsed, "leapS", 0)
                num_meas = getattr(parsed, "numMeas", 0)

                for i in range(1, num_meas + 1):
                    suffix = f"_{i:02d}"
                    pr = getattr(parsed, f"prMes{suffix}", None)
                    cp = getattr(parsed, f"cpMes{suffix}", None)
                    do = getattr(parsed, f"doMes{suffix}", None)
                    gnss = getattr(parsed, f"gnssId{suffix}", None)
                    sv = getattr(parsed, f"svId{suffix}", None)
                    sig = getattr(parsed, f"sigId{suffix}", None)
                    cno = getattr(parsed, f"cno{suffix}", None)
                    lock = getattr(parsed, f"locktime{suffix}", None)
                    pr_valid = getattr(parsed, f"prValid{suffix}", 0)
                    cp_valid = getattr(parsed, f"cpValid{suffix}", 0)
                    half_resolved = getattr(parsed, f"halfCyc{suffix}", 0)

                    if pr is None or gnss is None:
                        continue

                    rawx_writer.writerow({
                        "timestamp": now,
                        "rcv_tow_s": f"{rcv_tow:.9f}" if rcv_tow else "",
                        "week": week,
                        "leap_s": leap,
                        "sv_id": sv_label(gnss, sv),
                        "gnss_id": gnss_id_name(gnss),
                        "signal_id": signal_name(gnss, sig),
                        "pseudorange_m": f"{pr:.4f}" if pr else "",
                        "carrier_phase_cy": f"{cp:.4f}" if cp else "",
                        "doppler_hz": f"{do:.3f}" if do else "",
                        "cno_dBHz": f"{cno:.0f}" if cno else "",
                        "lock_duration_ms": f"{lock:.0f}" if lock else "",
                        "pr_valid": pr_valid,
                        "cp_valid": cp_valid,
                        "half_cycle_resolved": half_resolved,
                    })

            elif ident == "NAV-PVT":
                counts["NAV-PVT"] += 1
                pvt_writer.writerow({
                    "timestamp": now,
                    "fix_type": getattr(parsed, "fixType", ""),
                    "num_sv": getattr(parsed, "numSV", ""),
                    "lon_deg": f"{getattr(parsed, 'lon', 0):.7f}",
                    "lat_deg": f"{getattr(parsed, 'lat', 0):.7f}",
                    "height_m": f"{getattr(parsed, 'height', 0) * 1e-3:.3f}",
                    "h_acc_m": f"{getattr(parsed, 'hAcc', 0) * 1e-3:.3f}",
                    "v_acc_m": f"{getattr(parsed, 'vAcc', 0) * 1e-3:.3f}",
                    "pdop": f"{getattr(parsed, 'pDOP', 0):.2f}",
                    "year": getattr(parsed, "year", ""),
                    "month": getattr(parsed, "month", ""),
                    "day": getattr(parsed, "day", ""),
                    "hour": getattr(parsed, "hour", ""),
                    "min": getattr(parsed, "min", ""),
                    "sec": getattr(parsed, "sec", ""),
                    "nano_s": getattr(parsed, "nano", ""),
                    "tow_ms": getattr(parsed, "iTOW", ""),
                })

            elif ident == "TIM-TP":
                counts["TIM-TP"] += 1
                tow = getattr(parsed, "towMS", 0)
                sub_ms = getattr(parsed, "towSubMS", 0)
                q_err_ps = getattr(parsed, "qErr", 0)  # already in ps
                week = getattr(parsed, "week", 0)
                flags = getattr(parsed, "flags", 0)
                ref_info = getattr(parsed, "refInfo", 0)

                timtp_writer.writerow({
                    "timestamp": now,
                    "tow_ms": tow,
                    "tow_sub_ms": sub_ms,
                    "q_err_ps": q_err_ps,
                    "week": week,
                    "flags": flags,
                    "ref_info": ref_info,
                })

            elif ident == "RXM-SFRBX":
                counts["RXM-SFRBX"] += 1
                # Raw binary saved to .ubx file; no CSV extraction needed

            elif ident == "NAV-SAT":
                counts["NAV-SAT"] += 1
                # Satellite info — useful for monitoring, not logged to CSV

            else:
                counts["other"] += 1

            # Status line every 30s
            elapsed = time.monotonic() - start
            if time.monotonic() - last_status >= 30:
                last_status = time.monotonic()
                rate_str = f"{counts['RXM-RAWX'] / elapsed:.1f}" if elapsed > 0 else "?"
                print(f"  [{int(elapsed)}s] RAWX={counts['RXM-RAWX']} "
                      f"PVT={counts['NAV-PVT']} TIM-TP={counts['TIM-TP']} "
                      f"SFRBX={counts['RXM-SFRBX']} "
                      f"({rate_str} RAWX/s)",
                      file=sys.stderr)

    finally:
        ubx_file.close()
        rawx_file.close()
        pvt_file.close()
        timtp_file.close()

        elapsed = time.monotonic() - start
        print(f"\nDone. {int(elapsed)}s elapsed.", file=sys.stderr)
        print(f"  RAWX epochs: {counts['RXM-RAWX']}", file=sys.stderr)
        print(f"  PVT epochs:  {counts['NAV-PVT']}", file=sys.stderr)
        print(f"  TIM-TP:      {counts['TIM-TP']}", file=sys.stderr)
        print(f"  Files: {outdir}/{prefix}_*", file=sys.stderr)

    ser.close()


if __name__ == "__main__":
    main()

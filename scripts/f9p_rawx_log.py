#!/usr/bin/env python3
"""F9P raw-observation logger for offline PRIDE-PPP / RTKLIB analysis.

Configures one or both F9Ps on clkPoC3 to emit ``RXM-RAWX`` (raw
pseudorange + carrier-phase per SV per epoch) and ``RXM-SFRBX``
(broadcast nav subframes per SV) at 1 Hz, plus ``NAV-PVT`` for
context, and dumps the raw UBX byte stream from each F9P to a
file.  Convert offline with::

    convbin -r ubx -v 3.04 -o sitename.obs -n sitename.nav f9p1.ubx

then feed ``sitename.obs`` to PRIDE-PPP for an independent ARP fix.
That makes a third independent leg alongside the live F9P RTK
results in ``timelab/surveys/2026-04-29-ufo1-cors-rtk.md``.

Per Bob's 2026-04-29 ask: GRX1200 / OPUS path is blocked by Leica
Windows-only conversion software, so build the PRIDE leg from F9P
raw observations instead.

Hardware on clkPoC3 (post DO-decommission):

  F9P-1 (SEC-UNIQID 904584649306) — /dev/ttyACM0, 38400 baud
  F9P-2 (SEC-UNIQID 914202187869) — /dev/ttyACM1, 38400 baud

Both feed UFO1 via the lab splitter — see
``timelab/usb-identification.md`` for the by-path mapping.

Output files (default): ``data/raw/<host>-<f9p-tag>-<UTC>.ubx`` —
plain UBX bytes, no framing or compression.  Rotated hourly; the
first file's epoch is in the filename.

Stop the logger by sending SIGTERM (or Ctrl-C); it flushes + closes
files cleanly.  Detached usage::

    nohup python3 scripts/f9p_rawx_log.py \\
        --r1-port /dev/ttyACM0 --r1-tag F9P1 \\
        --r2-port /dev/ttyACM1 --r2-tag F9P2 \\
        --out-dir data/raw --duration 28800 \\
        > /tmp/rawx_log.out 2>&1 &
"""
from __future__ import annotations

import argparse
import datetime as dt
import logging
import os
import signal
import sys
import threading
import time

import serial
from pyubx2 import (
    SET_LAYER_RAM,
    TXN_NONE,
    UBXMessage,
    UBXReader,
)

log = logging.getLogger("f9p_rawx_log")


# ---------- F9P configuration --------------------------------------------


def configure_for_rawx(ser: serial.Serial, rate_hz: float = 1.0,
                       iface: str = "USB") -> None:
    """Configure the receiver to emit RXM-RAWX + RXM-SFRBX + NAV-PVT at the
    requested measurement rate, on the named iface, UBX protocol only.

    iface is "USB", "UART1", or "UART2" — selects which CFG keys get
    written.  USB is the F9P/X20P default; UART1 is used when the
    receiver is wired to a host serial port (e.g. F9T on Pi GPIO UART).

    1 Hz is the standard PRIDE-PPP rate.  Higher rates (5 Hz, 10 Hz)
    are supported by HPG firmware but multiply file size and offer no
    benefit for a daily ARP fix.

    Disables NMEA + RTCM output to keep the UBX byte stream clean
    and minimise bandwidth contention with RAWX (which can be
    ~1.5 KB/epoch with all four constellations active).
    """
    iface = iface.upper()
    if iface not in ("USB", "UART1", "UART2"):
        raise ValueError(f"unsupported iface {iface!r}; use USB/UART1/UART2")
    rate_ms = max(50, int(round(1000.0 / rate_hz)))

    # Essential set — must all ACK as one atomic VALSET.  These are the
    # keys every supported receiver (F9P, F9T-TIM, X20P) accepts.
    essential = [
        ("CFG_RATE_MEAS", rate_ms),
        ("CFG_RATE_NAV", 1),
        (f"CFG_{iface}OUTPROT_UBX", 1),
        (f"CFG_{iface}OUTPROT_NMEA", 0),
        (f"CFG_{iface}OUTPROT_RTCM3X", 0),
        (f"CFG_{iface}INPROT_UBX", 1),
        (f"CFG_{iface}INPROT_RTCM3X", 0),
        ("CFG_TMODE_MODE", 0),
        (f"CFG_MSGOUT_UBX_RXM_RAWX_{iface}", 1),
        (f"CFG_MSGOUT_UBX_RXM_SFRBX_{iface}", 1),
        (f"CFG_MSGOUT_UBX_NAV_PVT_{iface}", 1),
    ]
    msg = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE, essential)
    ser.write(msg.serialize())

    # Best-effort cleanup — disable possibly-noisy outputs left from a
    # prior run.  Some keys (e.g. NAV-RELPOSNED, HPPOSECEF/HPPOSLLH) are
    # RTK-only and NAK on F9T TIM firmware, so send each individually
    # and ignore NAKs.
    cleanup = [
        f"CFG_MSGOUT_UBX_NAV_RELPOSNED_{iface}",
        f"CFG_MSGOUT_UBX_NAV_HPPOSECEF_{iface}",
        f"CFG_MSGOUT_UBX_NAV_HPPOSLLH_{iface}",
        f"CFG_MSGOUT_RTCM_3X_TYPE1005_{iface}",
        f"CFG_MSGOUT_RTCM_3X_TYPE1077_{iface}",
        f"CFG_MSGOUT_RTCM_3X_TYPE1087_{iface}",
        f"CFG_MSGOUT_RTCM_3X_TYPE1097_{iface}",
        f"CFG_MSGOUT_RTCM_3X_TYPE1127_{iface}",
        f"CFG_MSGOUT_RTCM_3X_TYPE1230_{iface}",
    ]
    for key in cleanup:
        try:
            m = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE, [(key, 0)])
            ser.write(m.serialize())
        except Exception:
            pass  # key not in pyubx2 db — receiver wouldn't accept it either


def _iface_for_port(port: str) -> str:
    """Auto-detect the receiver iface that maps to a host serial port."""
    base = os.path.basename(port)
    if base.startswith("ttyACM"):
        return "USB"
    if base.startswith("ttyAMA") or base.startswith("ttyS"):
        return "UART1"
    return "USB"  # safest default


# ---------- Logging thread -----------------------------------------------


def _new_filename(out_dir: str, host: str, tag: str, t_utc: dt.datetime) -> str:
    name = f"{host}-{tag}-{t_utc.strftime('%Y%m%dT%H%M%SZ')}.ubx"
    return os.path.join(out_dir, name)


def logger_thread(
    label: str,
    ser: serial.Serial,
    out_dir: str,
    host: str,
    tag: str,
    stop: threading.Event,
    stats: dict,
    rotate_seconds: float,
) -> None:
    """Read raw bytes from the F9P serial port and write them
    verbatim to a UBX dump file.  Rotates every ``rotate_seconds``
    seconds (one new file per hour by default)."""
    file_t0 = dt.datetime.now(dt.timezone.utc)
    path = _new_filename(out_dir, host, tag, file_t0)
    f = open(path, "wb")
    log.info("%s: logging to %s", label, path)
    stats["paths"].append(path)
    last_rotate = time.monotonic()

    # Tee a parser into the same stream so we can sniff NAV-PVT for
    # operator-friendly sanity output.  The parser's read drains
    # nothing extra — we feed it the same bytes we wrote.
    parser_buf = bytearray()

    while not stop.is_set():
        try:
            data = ser.read(4096)
        except serial.SerialException as e:
            log.warning("%s: serial read error: %s", label, e)
            time.sleep(0.5)
            continue
        if data:
            try:
                f.write(data)
            except OSError as e:
                log.error("%s: write error: %s", label, e)
                continue
            stats["bytes"] += len(data)
            parser_buf.extend(data)

            # Cheap NAV-PVT sniff: scan for UBX sync 0xB5 0x62.
            i = 0
            while i < len(parser_buf) - 8:
                if parser_buf[i] == 0xB5 and parser_buf[i + 1] == 0x62:
                    cls, mid = parser_buf[i + 2], parser_buf[i + 3]
                    ln = parser_buf[i + 4] | (parser_buf[i + 5] << 8)
                    total = 8 + ln  # sync(2)+cls(1)+mid(1)+len(2)+payload+ck(2)
                    if i + total <= len(parser_buf):
                        if cls == 0x01 and mid == 0x07 and ln == 92:
                            # NAV-PVT — extract numSV + lon/lat + fixType
                            payload = parser_buf[i + 6:i + 6 + ln]
                            stats["last_pvt_t"] = time.monotonic()
                            try:
                                fix_type = payload[20]
                                num_sv = payload[23]
                                stats["last_fix_type"] = fix_type
                                stats["last_num_sv"] = num_sv
                            except IndexError:
                                pass
                        i += total
                        continue
                i += 1
            # Trim consumed prefix to bound memory.
            if i > 0:
                del parser_buf[:i]
            if len(parser_buf) > 16384:
                # Last-ditch: never let the sniff buffer grow unbounded.
                del parser_buf[:len(parser_buf) - 8192]

        # Hourly rotation.
        if time.monotonic() - last_rotate >= rotate_seconds:
            try:
                f.flush()
                f.close()
            except OSError:
                pass
            file_t0 = dt.datetime.now(dt.timezone.utc)
            path = _new_filename(out_dir, host, tag, file_t0)
            f = open(path, "wb")
            log.info("%s: rotated to %s", label, path)
            stats["paths"].append(path)
            last_rotate = time.monotonic()

    try:
        f.flush()
        f.close()
    except OSError:
        pass
    log.info("%s: logger thread exiting (last file %s)", label, path)


# ---------- main ---------------------------------------------------------


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--r1-port", required=True)
    ap.add_argument("--r1-tag", default="F9P1",
                    help="short label for rover 1 (used in filenames)")
    ap.add_argument("--r2-port",
                    help="rover 2 serial port (omit to log only one F9P)")
    ap.add_argument("--r2-tag", default="F9P2")
    ap.add_argument("--baud", type=int, default=38400)
    ap.add_argument("--iface", default="auto",
                    choices=("auto", "USB", "UART1", "UART2"),
                    help="receiver iface for CFG keys (default: auto from "
                    "port path — ttyACM*=USB, ttyAMA*/ttyS*=UART1)")
    ap.add_argument("--rate-hz", type=float, default=1.0,
                    help="measurement rate (default 1 Hz, the PRIDE-PPP "
                    "standard)")
    ap.add_argument("--duration", type=float, default=43200.0,
                    help="seconds to log (default 12 h).  0 = forever; "
                    "stop with SIGTERM.")
    ap.add_argument("--out-dir", default="data/raw",
                    help="directory for .ubx output files; created if "
                    "missing")
    ap.add_argument("--rotate-hours", type=float, default=1.0,
                    help="open a new file every N hours (default 1)")
    ap.add_argument("--report-every", type=float, default=300.0,
                    help="status line every N seconds (default 5 min)")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    os.makedirs(args.out_dir, exist_ok=True)
    host = os.uname().nodename.split(".")[0]
    rotate_s = max(60.0, args.rotate_hours * 3600.0)

    rovers = [(args.r1_port, args.r1_tag)]
    if args.r2_port:
        rovers.append((args.r2_port, args.r2_tag))

    serials: list[tuple[str, str, serial.Serial, dict]] = []
    for port, tag in rovers:
        log.info("Opening %s @ %d (tag=%s)", port, args.baud, tag)
        ser = serial.Serial(port, args.baud, timeout=0.1)
        time.sleep(0.2)
        ser.reset_input_buffer()
        iface = _iface_for_port(port) if args.iface == "auto" else args.iface
        log.info("Configuring %s for RAWX/SFRBX @ %.1f Hz on %s",
                 tag, args.rate_hz, iface)
        configure_for_rawx(ser, rate_hz=args.rate_hz, iface=iface)
        time.sleep(0.5)
        ser.reset_input_buffer()
        st = {
            "bytes": 0,
            "paths": [],
            "last_pvt_t": None,
            "last_fix_type": None,
            "last_num_sv": None,
        }
        serials.append((port, tag, ser, st))

    stop = threading.Event()

    # SIGTERM / SIGINT: flush + close cleanly.
    def _sig(signum, frame):
        log.info("signal %s — stopping", signum)
        stop.set()
    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    threads = []
    for port, tag, ser, st in serials:
        t = threading.Thread(
            target=logger_thread,
            args=(f"{tag} [{port}]", ser, args.out_dir, host, tag,
                  stop, st, rotate_s),
            daemon=True,
            name=f"log-{tag}",
        )
        t.start()
        threads.append(t)

    t_start = time.monotonic()
    if args.duration > 0:
        log.info("Logging for %.0f s (%.1f h); status every %.0f s",
                 args.duration, args.duration / 3600.0, args.report_every)
    else:
        log.info("Logging until SIGTERM; status every %.0f s",
                 args.report_every)

    next_report = t_start + args.report_every
    try:
        while not stop.is_set():
            if args.duration > 0 and time.monotonic() - t_start >= args.duration:
                log.info("duration reached — stopping")
                stop.set()
                break
            time.sleep(0.5)
            if time.monotonic() >= next_report:
                age = time.monotonic() - t_start
                msgs = []
                for port, tag, ser, st in serials:
                    pvt_age = ("?"
                               if st["last_pvt_t"] is None
                               else f"{time.monotonic() - st['last_pvt_t']:.0f}s")
                    msgs.append(
                        f"{tag}: {st['bytes']/1024:.0f}kB "
                        f"fix={st['last_fix_type']} "
                        f"nSV={st['last_num_sv']} pvt_age={pvt_age}")
                log.info("[t=%.0fs] %s", age, " | ".join(msgs))
                next_report += args.report_every
    finally:
        stop.set()
        for t in threads:
            t.join(timeout=2.0)
        for _, _, ser, _ in serials:
            try:
                ser.close()
            except Exception:
                pass

    print()
    print("=" * 70)
    print("RAWX log summary")
    print("=" * 70)
    for port, tag, _, st in serials:
        print(f"  {tag} ({port}): {st['bytes']} bytes "
              f"in {len(st['paths'])} file(s)")
        for p in st["paths"]:
            try:
                size = os.path.getsize(p)
            except OSError:
                size = -1
            print(f"    {p} ({size} B)")
    print()
    print("Convert UBX → RINEX 3.04 (multi-GNSS, mixed file) with RTKLIB:")
    print("  convbin -r ubx -v 3.04 -o <site>.obs -n <site>.nav <file>.ubx")
    print()
    print("Then feed <site>.obs to PRIDE-PPP for an independent ARP.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Galileo HAS (High Accuracy Service) E6-B page monitor / header decoder.

Reads Galileo E6-B C/NAV pages from a u-blox ZED-X20P via UBX-RXM-SFRBX
(gnssId=2 GAL, sigId=8 E6), parses the HAS page header, classifies
operational vs dummy pages, and tracks per-message page collection so you
can see when a complete (Reed-Solomon-decodable) message set has arrived.

This is the *page-level* decoder — the front end that de-risks using
Galileo HAS on the X20 (see docs/galileo-has-research.md).  It does NOT
do the High-Parity Vertical Reed-Solomon body decode into SSR
corrections; hand the collected page bodies (--dump-json) to CSSRlib /
HAS-decoding for that step.

Empirically determined on PiPuss 2026-06-08 (ZED-X20P-00B, HPG 2.02):
  - RXM-SFRBX delivers the 492-bit E6-B page in 16 big-endian dwrds.
  - The HAS page header starts at a 14-bit lead-in offset, then the
    standard 24-bit layout: HASS(2) rsv(2) MT(2) MID(5) MS(5) PID(8).
  - Real HAS pages: HASS=1 (Operational), MT=1.  Dummy pages: HASS=2,
    MT=3, byte-identical across satellites.
  - The 424-bit message body follows the header (bits 38..462 of page).

Usage:
    has_page_monitor.py [--port /dev/ttyACM0] [--baud 115200]
                        [--seconds 90] [--dump-json out.json]
"""
import argparse
import collections
import json
import time

HEADER_OFFSET = 14          # lead-in before the HAS page header (bits)
BODY_OFFSET = HEADER_OFFSET + 24   # 424-bit message body starts here
BODY_BITS = 424


def _words_to_int(words):
    v = 0
    for w in words:
        v = (v << 32) | w
    return v, len(words) * 32


def _gb(val, nbits, start, length):
    """Extract `length` bits at MSB-first position `start`."""
    return (val >> (nbits - (start + length))) & ((1 << length) - 1)


def parse_header(val, nbits, off=HEADER_OFFSET):
    return {
        "hass": _gb(val, nbits, off, 2),       # 1=Operational, 2=test/reserved
        "mt":   _gb(val, nbits, off + 4, 2),   # 1=MT1 (only defined type)
        "mid":  _gb(val, nbits, off + 6, 5),   # message id
        "ms":   _gb(val, nbits, off + 11, 5),  # message size (pages needed)
        "pid":  _gb(val, nbits, off + 16, 8),  # page id (1..255)
    }


def body_bits(val, nbits):
    return _gb(val, nbits, BODY_OFFSET, BODY_BITS)


def is_operational(h):
    return h["hass"] == 1 and h["mt"] == 1


def _iter_e6_pages(port, baud, seconds):
    from pyubx2 import UBXMessage, UBXReader
    import serial
    ser = serial.Serial(port, baud, timeout=0.5)
    ser.write(UBXMessage.config_set(
        1, 0, [("CFG_MSGOUT_UBX_RXM_SFRBX_USB", 1)]).serialize())
    time.sleep(0.3)
    ubr = UBXReader(ser, protfilter=2)
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            try:
                _raw, p = ubr.read()
            except Exception:
                continue
            if p is None or p.identity != "RXM-SFRBX":
                continue
            if getattr(p, "gnssId", None) != 2 or getattr(p, "sigId", None) != 8:
                continue
            nw = getattr(p, "numWords", 0)
            words = [getattr(p, "dwrd_%02d" % i) for i in range(1, nw + 1)]
            val, nbits = _words_to_int(words)
            yield getattr(p, "svId"), val, nbits
    finally:
        ser.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=90)
    ap.add_argument("--dump-json", default=None,
                    help="write collected operational message bodies "
                         "(keyed by MID -> {PID: body_hex}) for downstream "
                         "Reed-Solomon decode")
    args = ap.parse_args()

    # mid -> {ms, pages{pid: body_int}, total_seen}
    msgs = collections.defaultdict(
        lambda: {"ms": None, "pages": {}, "seen": 0})
    n_oper = n_dummy = n_total = 0

    for sv, val, nbits in _iter_e6_pages(args.port, args.baud, args.seconds):
        n_total += 1
        h = parse_header(val, nbits)
        if not is_operational(h):
            n_dummy += 1
            continue
        n_oper += 1
        m = msgs[h["mid"]]
        m["ms"] = h["ms"]
        m["seen"] += 1
        m["pages"].setdefault(h["pid"], body_bits(val, nbits))

    print("E6 pages: %d total | %d operational (HASS=1,MT=1) | %d dummy/other"
          % (n_total, n_oper, n_dummy))
    print("\n%-5s %-5s %-6s %-10s %s"
          % ("MID", "MS", "seen", "distPIDs", "complete (distPIDs>=MS)?"))
    complete_mids = []
    for mid in sorted(msgs):
        m = msgs[mid]
        npid = len(m["pages"])
        ms = m["ms"] or 0
        done = ms > 0 and npid >= ms
        if done:
            complete_mids.append(mid)
        print("%-5d %-5d %-6d %-10d %s"
              % (mid, ms, m["seen"], npid, "YES" if done else "no"))
    print("\n%d message(s) reached a complete page set (RS-decodable): %s"
          % (len(complete_mids), complete_mids or "none"))

    if args.dump_json:
        out = {str(mid): {"ms": msgs[mid]["ms"],
                          "pages": {str(pid): "%0106x" % body
                                    for pid, body in msgs[mid]["pages"].items()}}
               for mid in complete_mids}
        with open(args.dump_json, "w") as f:
            json.dump(out, f, indent=1)
        print("wrote %d complete message(s) to %s for RS decode"
              % (len(out), args.dump_json))


if __name__ == "__main__":
    main()

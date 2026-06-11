#!/usr/bin/env python3
"""ubx_filter.py — keep only the UBX messages needed for PPP, drop the rest.

A time-appliance F9T (e.g. a Timebeat OTC Mini) emits timing messages
(NAV-SIG, NAV2-SIG, NAV2-POSECEF, TIM-TP, TIM-SVIN, MON-SPAN) alongside the
RAWX/SFRBX/NAV-PVT that f9p_rawx_log.py adds. For an ARP survey only
RXM-RAWX + RXM-SFRBX (+ NAV-PVT for an a-priori) are needed; the timing
messages are typically ~half the bytes. Stripping them BEFORE a slow /
multi-hop file transfer roughly halves the payload, and is non-invasive —
it touches a file, never the receiver config or a running timebeat.

Pure stdlib (runs on any host with python3). Byte-level passthrough with
checksum validation + resync, so unknown messages and transfer glitches are
handled safely.

Usage:
    python3 ubx_filter.py in.ubx out.ubx
    python3 ubx_filter.py in.ubx out.ubx --keep rawx,sfrbx,pvt,navsig
"""
import argparse
import sys

# name -> (class, id)
_MSGS = {
    "rawx":   (0x02, 0x15),   # RXM-RAWX   (carrier phase + pseudorange)
    "sfrbx":  (0x02, 0x13),   # RXM-SFRBX  (broadcast ephemeris subframes)
    "pvt":    (0x01, 0x07),   # NAV-PVT    (a-priori position)
    "navsig": (0x01, 0x43),   # NAV-SIG
    "timtp":  (0x0D, 0x01),   # TIM-TP
    "monspan":(0x0A, 0x31),   # MON-SPAN
}
DEFAULT_KEEP = "rawx,sfrbx,pvt"


def _ubx_cksum(payload_and_hdr: memoryview):
    a = b = 0
    for x in payload_and_hdr:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return a, b


def filter_stream(fin, fout, keep):
    buf = bytearray()
    eof = False
    kept = {}      # (cls,id) -> count kept
    dropped = {}   # (cls,id) -> count dropped
    bad = 0
    in_bytes = out_bytes = 0
    CHUNK = 1 << 20
    while not eof or buf:
        if not eof:
            chunk = fin.read(CHUNK)
            if chunk:
                buf += chunk
                in_bytes += len(chunk)
            else:
                eof = True
        progress = False
        while True:
            j = buf.find(b"\xb5\x62")
            if j < 0:
                # no sync in buffer; keep only a possible trailing 0xB5
                if buf and buf[-1] == 0xB5:
                    del buf[:-1]
                else:
                    buf.clear()
                break
            if j > 0:
                del buf[:j]            # discard pre-sync bytes (e.g. stray NMEA)
            if len(buf) < 8:
                break                  # need sync+class+id+len(2)+ck(2) minimum
            ln = buf[4] | (buf[5] << 8)
            end = 6 + ln + 2
            if len(buf) < end:
                if eof:
                    buf.clear()        # truncated trailing frame at EOF
                break                  # wait for more data
            ca, cb = _ubx_cksum(memoryview(buf)[2:end - 2])
            if ca != buf[end - 2] or cb != buf[end - 1]:
                bad += 1
                del buf[:2]            # false sync — step past it and resync
                progress = True
                continue
            key = (buf[2], buf[3])
            if key in keep:
                fout.write(buf[:end])
                out_bytes += end
                kept[key] = kept.get(key, 0) + 1
            else:
                dropped[key] = dropped.get(key, 0) + 1
            del buf[:end]
            progress = True
        if eof and not progress:
            break
    return kept, dropped, bad, in_bytes, out_bytes


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("infile")
    ap.add_argument("outfile")
    ap.add_argument("--keep", default=DEFAULT_KEEP,
                    help=f"comma-separated message names (default: {DEFAULT_KEEP}; "
                         f"known: {','.join(_MSGS)})")
    args = ap.parse_args()

    keep = set()
    for name in (s.strip().lower() for s in args.keep.split(",") if s.strip()):
        if name not in _MSGS:
            print(f"ERROR: unknown message '{name}'; known: {','.join(_MSGS)}",
                  file=sys.stderr)
            return 2
        keep.add(_MSGS[name])

    with open(args.infile, "rb") as fin, open(args.outfile, "wb") as fout:
        kept, dropped, bad, nin, nout = filter_stream(fin, fout, keep)

    def fmt(d):
        return ", ".join(f"{c:02X}-{i:02X}:{n}" for (c, i), n in sorted(d.items())) or "none"
    print(f"in {nin/1e6:.1f} MB -> out {nout/1e6:.1f} MB "
          f"({nout/nin*100:.0f}% kept)" if nin else "empty input")
    print(f"kept:    {fmt(kept)}")
    print(f"dropped: {fmt(dropped)}")
    if bad:
        print(f"resynced past {bad} bad-checksum/false-sync points")
    return 0


if __name__ == "__main__":
    sys.exit(main())

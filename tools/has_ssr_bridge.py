#!/usr/bin/env python3
"""Galileo HAS -> SSR-records bridge for the live engine.

Closes the loop for the HAS-vs-broadcast time-transfer experiment
(docs/has-time-transfer-experiment.md): decodes HAS off the X20's E6
signal out-of-process (CSSRlib) and writes the latest orbit/clock/
code-bias corrections to a JSON file that the engine ingests via
`--ssr-records-file` (realtime_ppp.ssr_records_reader).

The engine owns the serial port and writes all UBX to a file
(`--ubx-out`); this bridge tails that file — no second reader on the
receiver.  Only this process imports CSSRlib (galois/numba), keeping the
engine dependency-light.

Modes:
  --ubx-file PATH    tail the engine's --ubx-out UBX byte stream (live)
  --pages-file PATH  read a captured page file (has_page_monitor
                     --raw-pages format) — offline / validation

Writes `--out` atomically (temp + rename) each time a HAS message decodes,
so the engine never reads a half-written file.
"""
import argparse
import json
import os
import sys
import tempfile
import time

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "scripts"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

DEFAULT_GMAT = os.path.join(_REPO, "support", "has", "has_gmat.csv")


def _write_records(out_path, epoch_s, records):
    tmp = out_path + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"epoch_s": epoch_s, "generated_mono": time.monotonic(),
                   "records": records}, f)
    os.replace(tmp, out_path)   # atomic


def _iter_pages_file(path):
    """Yield (epoch_int, [page_hex,...]) batches from a raw-pages file."""
    import has_decode_cssr as H
    for epoch, batch in H.load_pages(path):
        yield int(float(epoch)), batch


def _iter_ubx_file(path, idle_sleep=0.5, stop_after_s=None):
    """Tail a UBX byte file; yield (wall_sec, [page_hex,...]) per second.

    Groups E6 (gnssId=2, sigId=8) RXM-SFRBX pages by integer wall-second
    so the decoder accumulates a full page set before its timeout.
    """
    from pyubx2 import UBXReader
    f = open(path, "rb")
    ubr = UBXReader(f, protfilter=2)
    t0 = time.monotonic()
    cur_sec, batch = None, []
    while True:
        try:
            _raw, p = ubr.read()
        except Exception:
            p = None
        if p is None:
            if batch:
                yield cur_sec, batch
                cur_sec, batch = None, []
            if stop_after_s and time.monotonic() - t0 > stop_after_s:
                return
            time.sleep(idle_sleep)
            continue
        if getattr(p, "identity", "") != "RXM-SFRBX":
            continue
        if getattr(p, "gnssId", None) != 2 or getattr(p, "sigId", None) != 8:
            continue
        nw = getattr(p, "numWords", 0)
        val = 0
        for i in range(1, nw + 1):
            val = (val << 32) | getattr(p, "dwrd_%02d" % i)
        sec = int(time.time())
        if cur_sec is None:
            cur_sec = sec
        if sec != cur_sec and batch:
            yield cur_sec, batch
            batch = []
            cur_sec = sec
        batch.append(format(val, "0%dx" % (nw * 32 // 4)))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--ubx-file", help="tail the engine's --ubx-out (live)")
    g.add_argument("--pages-file", help="captured raw-pages file (offline)")
    ap.add_argument("--out", required=True, help="records JSON for the engine")
    ap.add_argument("--gmat", default=DEFAULT_GMAT)
    ap.add_argument("--seconds", type=float, default=None,
                    help="stop tailing after this long (ubx-file mode)")
    args = ap.parse_args()

    from cssrlib.cssr_has import cssr_has, cnav_msg
    from has_ssr_adapter import cssr_to_records

    cnav = cnav_msg()
    cnav.load_gmat(args.gmat)
    dec = cssr_has(tempfile.NamedTemporaryFile(suffix=".log", delete=False).name)
    dec.monlevel = 0

    if args.pages_file:
        batches = _iter_pages_file(args.pages_file)
    else:
        batches = _iter_ubx_file(args.ubx_file, stop_after_s=args.seconds)

    n_msg = n_write = 0
    for epoch, batch in batches:
        try:
            m = cnav.decode_cnav(epoch, [{"nav": h} for h in batch])
        except Exception:
            continue
        if not m:
            continue
        n_msg += 1
        dec.decode_cssr(bytes(m))
        recs = cssr_to_records(dec)
        if recs:
            _write_records(args.out, float(getattr(dec, "toh", 0.0)), recs)
            n_write += 1
    print("HAS bridge: decoded %d messages, wrote %d records updates to %s"
          % (n_msg, n_write, args.out))


if __name__ == "__main__":
    main()

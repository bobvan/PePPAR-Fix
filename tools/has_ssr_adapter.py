#!/usr/bin/env python3
"""Adapt CSSRlib-decoded Galileo HAS corrections into our SSRState.

Bridges the HAS decoder (tools/has_decode_cssr.py, CSSRlib) to the
engine's SSRState (scripts/ssr_corrections.py), so the same float-PPP
time/position filter that runs on BKG SSR can run on HAS instead — the
HAS-vs-broadcast time-transfer comparison on the DO-less X20 host.

`cssr_to_records()` extracts decoder-neutral records from a decoded
`cssr_has` object; `SSRState.update_from_records()` ingests them.  The
split keeps the engine and its tests free of the heavy CSSRlib/galois
dependency — only this tool imports CSSRlib.

Sign/units: CSSRlib normalizes HAS to the RTCM/IGS-SSR convention
internally (it negates the HAS orbit and applies the per-constellation
clock multiplier), so its `lc[0].dorb`/`dclk` (metres) map directly to
our OrbitCorrection/ClockCorrection.  This is the convention to confirm
against a BKG-SSR cross-check before trusting HAS in production (HAS and
BKG clock corrections for the same SVs should track with slope +1, off
by only a per-constellation datum constant).

Run as a script to decode a captured page file end-to-end into an
SSRState and print a summary:

    tools/has_ssr_adapter.py data/has_pages.txt
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "scripts"))


def _scalar(x):
    a = np.ravel(np.asarray(x, dtype=float))
    return float(a[0]) if a.size else 0.0


def cssr_to_records(cssr):
    """Extract per-satellite HAS records from a decoded cssr_has object.

    Returns a list of dicts compatible with SSRState.update_from_records:
    {prn, iode, orbit:(r,a,c)|None, clock:c0|None, code_bias:{code:bias}}.
    """
    from cssrlib.gnss import sat2id
    lc = cssr.lc[0]
    records = []
    for sat in cssr.sat_n[:cssr.nsat_n]:
        rec = {"prn": sat2id(sat)}
        do = lc.dorb.get(sat) if isinstance(lc.dorb, dict) else None
        if do is not None:
            rac = np.ravel(np.asarray(do, dtype=float))
            if rac.size >= 3 and np.any(rac[:3] != 0):
                rec["orbit"] = (rac[0], rac[1], rac[2])
                iode = lc.iode.get(sat) if isinstance(lc.iode, dict) else None
                rec["iode"] = int(iode) if iode is not None else 0
        dc = lc.dclk.get(sat) if isinstance(lc.dclk, dict) else None
        if dc is not None and _scalar(dc) != 0.0:
            rec["clock"] = _scalar(dc)
        cb = lc.cbias.get(sat) if isinstance(lc.cbias, dict) else None
        if cb:
            rec["code_bias"] = {str(sig)[1:]: float(val)
                                for sig, val in cb.items()}
        if len(rec) > 1:   # more than just prn
            records.append(rec)
    return records


def populate_ssrstate(cssr, ssr, epoch_s, rx_mono=None):
    """Decode-to-SSRState convenience: extract + ingest in one call."""
    return ssr.update_from_records(cssr_to_records(cssr), epoch_s,
                                   rx_mono=rx_mono, src_mount="HAS")


def _main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("pages", help="raw page file (has_page_monitor --raw-pages)")
    ap.add_argument("--gmat", default=os.path.join(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))), "support", "has",
        "has_gmat.csv"))
    args = ap.parse_args()

    import tempfile
    from cssrlib.cssr_has import cssr_has, cnav_msg
    from ssr_corrections import SSRState
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import has_decode_cssr as H

    cnav = cnav_msg()
    cnav.load_gmat(args.gmat)
    dec = cssr_has(tempfile.NamedTemporaryFile(suffix=".log", delete=False).name)
    dec.monlevel = 0
    ssr = SSRState()

    totals = {"orbit": 0, "clock": 0, "code_bias": 0}
    n_msgs = 0
    for epoch, batch in H.load_pages(args.pages):
        m = cnav.decode_cnav(int(float(epoch)), [{"nav": h} for h in batch])
        if not m:
            continue
        dec.decode_cssr(bytes(m))
        c = populate_ssrstate(dec, ssr, epoch_s=float(getattr(dec, "toh", 0)))
        for k in totals:
            totals[k] += c[k]
        n_msgs += 1

    print("decoded %d HAS messages -> SSRState: %d orbit, %d clock, %d code-bias writes"
          % (n_msgs, totals["orbit"], totals["clock"], totals["code_bias"]))
    print("SSRState now holds: %d sats with orbit, %d with clock"
          % (ssr.n_orbit, ssr.n_clock))
    # show a few corrections read back through the engine getters
    shown = 0
    for prn in sorted(ssr._orbit):
        oc = ssr.get_orbit(prn)
        cc = ssr.get_clock(prn)
        if oc is None or cc is None:
            continue
        cbs = {s: round(ssr.get_code_bias(prn, s), 3)
               for s in ssr._code_bias.get(prn, {})}
        print("  %s: clock c0=%+.3f m  orbit(r/a/c)=(%.3f,%.3f,%.3f) m  iode=%d  cbias=%s"
              % (prn, cc.c0, oc.radial, oc.along, oc.cross, oc.iod, cbs))
        shown += 1
        if shown >= 6:
            break


if __name__ == "__main__":
    _main()

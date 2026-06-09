#!/usr/bin/env python3
"""Reed-Solomon + SSR decode of Galileo HAS pages, via CSSRlib.

Second half of the HAS pipeline (the first half — capturing E6-B pages
from the X20 — is tools/has_page_monitor.py --raw-pages).  Reads a raw
page file ('<epoch> <svId> <page_hex>' per line, page_hex = the SFRBX
dwrds big-endian with the HAS header at bit 14) and uses CSSRlib to:

  1. HPVRS Reed-Solomon decode each complete page set into a HAS message
     (cnav_msg.decode_cnav, gMat from the HAS SIS ICD Annex B), and
  2. parse the HAS message into SSR corrections — mask / orbit / clock /
     code-bias / phase-bias (cssr_has.decode_cssr).

Then it prints a per-message summary so you can see real corrections for
the constellation in view.

CSSRlib has heavy deps (galois/numba), so run this on the dev box, not on
a Pi — feed it pages captured on PiPuss.

  pip install cssrlib   # dev-box venv only
  tools/has_decode_cssr.py data/has_pages.txt

Validated against the ICD's own Annex D example and against live X20-00B
pages (PiPuss 2026-06-08).  See docs/galileo-has-research.md.
"""
import argparse
import os
import sys
import tempfile

import numpy as np

DEFAULT_GMAT = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "support", "has", "has_gmat.csv")


def load_pages(path):
    """Yield (epoch, [page_hex, ...]) groups in file order."""
    cur_epoch = None
    batch = []
    for line in open(path):
        parts = line.split()
        if len(parts) < 3:
            continue
        epoch, _sv, page_hex = parts[0], parts[1], parts[2]
        if cur_epoch is None:
            cur_epoch = epoch
        if epoch != cur_epoch and batch:
            yield cur_epoch, batch
            batch = []
            cur_epoch = epoch
        batch.append(page_hex)
    if batch:
        yield cur_epoch, batch


def _const_breakdown(dec):
    """Count decoded sats per constellation using CSSRlib sat ids."""
    from cssrlib.gnss import sat2prn, uGNSS
    names = {uGNSS.GPS: "GPS", uGNSS.GAL: "GAL", uGNSS.GLO: "GLO",
             uGNSS.BDS: "BDS", uGNSS.QZS: "QZS"}
    counts = {}
    for s in dec.sat_n[:dec.nsat_n]:
        sys, _prn = sat2prn(s)
        counts[names.get(sys, str(sys))] = counts.get(names.get(sys, str(sys)), 0) + 1
    return counts


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("pages", help="raw page file from has_page_monitor --raw-pages")
    ap.add_argument("--gmat", default=DEFAULT_GMAT)
    ap.add_argument("--max-messages", type=int, default=8,
                    help="stop after this many decoded HAS messages")
    args = ap.parse_args()

    from cssrlib.cssr_has import cssr_has, cnav_msg

    cnav = cnav_msg()
    cnav.load_gmat(args.gmat)
    logf = tempfile.NamedTemporaryFile(suffix=".log", delete=False).name
    dec = cssr_has(logf)
    dec.monlevel = 0

    n_pages = n_msgs = 0
    for epoch, batch in load_pages(args.pages):
        n_pages += len(batch)
        vi = [{"nav": h} for h in batch]
        try:
            has_msg = cnav.decode_cnav(int(float(epoch)), vi)
        except Exception as e:
            print("decode_cnav error at epoch %s: %s" % (epoch, e))
            continue
        if not has_msg:
            continue
        try:
            dec.decode_cssr(bytes(has_msg))
        except Exception as e:
            print("decode_cssr error: %s" % e)
            continue
        n_msgs += 1
        blocks = []
        for name, arr in (("orbit", "dorb"), ("clock", "dclk"),
                          ("cbias", "cbias"), ("pbias", "pbias")):
            try:
                a = np.array(getattr(dec.lc[0], arr))
                if np.count_nonzero(a) > 0:
                    blocks.append(name)
            except Exception:
                pass
        try:
            const = _const_breakdown(dec)
        except Exception:
            const = {}
        print("HAS msg #%d: TOH=%s iod_s=%s nsat=%d %s blocks=%s"
              % (n_msgs, getattr(dec, "toh", "?"), getattr(dec, "iodssr", "?"),
                 getattr(dec, "nsat_n", 0), const, blocks or "(mask only)"))
        # sample real correction values
        try:
            dclk = np.array(dec.lc[0].dclk)
            nz = np.nonzero(dclk)[0]
            if len(nz):
                from cssrlib.gnss import sat2id
                samp = ", ".join("%s=%.3fm" % (sat2id(dec.sat_n[k]), dclk[k])
                                 for k in nz[:4])
                print("    sample clock corr: %s" % samp)
        except Exception:
            pass
        if n_msgs >= args.max_messages:
            break

    print("\nfed %d pages -> decoded %d HAS message(s)" % (n_pages, n_msgs))
    if n_msgs == 0:
        print("No complete messages decoded — capture longer, or check that "
              "operational (HASS=1) pages were flowing.")
        sys.exit(1)


if __name__ == "__main__":
    main()

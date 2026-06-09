#!/usr/bin/env python3
"""Multi-arm receiver-clock (dt_rx) comparison on IDENTICAL observations.

The clean way to evaluate Galileo HAS (docs/has-time-transfer-experiment.md):
one process owns the X20 and reads its observations ONCE, then runs N
parallel FixedPosFilter clock estimators that share those identical
observations but each get a DIFFERENT satellite-correction source.  Because
every arm sees the same obs, rx-TCXO, and sky, the pairwise dt_rx
*difference* between arms cancels the rx-TCXO drift (which dominated the
sequential first look) and isolates the correction quality.

Arms (the broadcast→precise bracket, with HAS judged against it):
  broadcast  no SSR (broadcast ephemeris only) ............ floor
  has        Galileo HAS (E6 SIS) via the records file ..... free, no-internet
  bkg        BKG  SSRA00BKG0 (combined-IGS)  NTRIP
  cas        CAS  SSRA01CAS1 (single-AC)     NTRIP (same caster/creds as BKG)
  cnes       CNES SSRA00CNE0 (single-AC)     NTRIP (products.igs-ip.net)

Eph + BKG + CAS share the Australian caster (--ntrip-conf); CNES uses
--cnes-conf.  The HAS arm needs the bridge running separately on this
host's --ubx-out (tools/has_ssr_bridge.py --ubx-file <ubx> --out <records>);
this tool only reads the records file (no CSSRlib dependency here).

Output CSV: one row per epoch with dt_rx_ns and n_used for each enabled arm.
Analyze with tools/tt_tdev_compare.py (and pairwise differences).
"""
import argparse
import configparser
import csv
import math
import os
import queue
import sys
import threading
import time
from datetime import datetime, timezone

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "scripts"))

C = 299792458.0


def _load_arp(label):
    import json
    from peppar_fix.position_state import find_antennas_json
    d = json.load(open(find_antennas_json()))[label]["ecef_m"]
    return (d["x"], d["y"], d["z"])


def _ntrip(conf_path, mount, stream_cls):
    c = configparser.ConfigParser()
    c.read(conf_path)
    s = c["ntrip"]
    return stream_cls(caster=s["caster"], port=int(s.get("port", 2101)),
                      mountpoint=mount, user=s.get("user"),
                      password=s.get("password"),
                      tls=s.getboolean("tls", False))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--serial", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--arp-label", default="ufo1")
    ap.add_argument("--ntrip-conf", default="ntrip.conf",
                    help="Australian caster creds (eph + BKG + CAS)")
    ap.add_argument("--cnes-conf", default="ntrip-cnes.conf")
    ap.add_argument("--has-records", default="/tmp/has_records.json",
                    help="records file written by the external HAS bridge")
    ap.add_argument("--ubx-out", default="/tmp/x20_multiarm.ubx",
                    help="raw UBX dump for the HAS bridge to tail")
    ap.add_argument("--out", required=True, help="per-epoch dt_rx CSV")
    ap.add_argument("--duration", type=float, default=1800)
    ap.add_argument("--eph-mount", default="BCEP00BKG0")
    ap.add_argument("--arms", default="broadcast,has,bkg,cas,cnes")
    ap.add_argument("--systems", default="gps,gal",
                    help="constellations admitted to ALL arms.  Default "
                         "gps,gal = the set Galileo HAS Phase 1 covers, so "
                         "every arm corrects the same SVs (fair quality "
                         "comparison).  Adding bds lets the precise streams "
                         "correct BDS while HAS falls back to broadcast for "
                         "it (shows HAS's coverage gap, but confounds the "
                         "quality comparison).")
    args = ap.parse_args()

    from broadcast_eph import BroadcastEphemeris
    from ntrip_client import NtripStream
    from realtime_ppp import serial_reader, ntrip_reader, ssr_records_reader
    from ssr_corrections import SSRState, RealtimeCorrections
    from solve_ppp import FixedPosFilter
    from peppar_fix.receiver import get_driver

    arms = [a.strip() for a in args.arms.split(",") if a.strip()]
    arp = _load_arp(args.arp_label)
    print("ARP %s = %s; arms = %s" % (args.arp_label, arp, arms))

    beph = BroadcastEphemeris()
    ssr = {a: SSRState() for a in arms}   # broadcast's stays empty
    stop = threading.Event()
    obs_q = queue.Queue()
    threads = []

    # serial: obs -> queue, SFRBX -> beph, raw UBX -> file (for the bridge)
    ubx_f = open(args.ubx_out, "ab")
    systems = set(s.strip() for s in args.systems.split(",") if s.strip())
    threads.append(threading.Thread(target=serial_reader, args=(
        args.serial, args.baud, obs_q, stop, beph), kwargs={
        "systems": systems, "driver": get_driver("x20p"),
        "ubx_log_file": ubx_f}, daemon=True))

    # broadcast eph stream (shared beph) — use a throwaway SSRState sink
    eph_stream = _ntrip(args.ntrip_conf, args.eph_mount, NtripStream)
    threads.append(threading.Thread(target=ntrip_reader, args=(
        eph_stream, beph, SSRState(), stop, "EPH"), daemon=True))

    # per-arm SSR sources
    if "bkg" in arms:
        threads.append(threading.Thread(target=ntrip_reader, args=(
            _ntrip(args.ntrip_conf, "SSRA00BKG0", NtripStream),
            beph, ssr["bkg"], stop, "BKG"), daemon=True))
    if "cas" in arms:
        threads.append(threading.Thread(target=ntrip_reader, args=(
            _ntrip(args.ntrip_conf, "SSRA01CAS1", NtripStream),
            beph, ssr["cas"], stop, "CAS"), daemon=True))
    if "cnes" in arms:
        threads.append(threading.Thread(target=ntrip_reader, args=(
            _ntrip(args.cnes_conf, "SSRA00CNE0", NtripStream),
            beph, ssr["cnes"], stop, "CNES"), daemon=True))
    if "has" in arms:
        threads.append(threading.Thread(target=ssr_records_reader, args=(
            args.has_records, ssr["has"], stop, "HAS"), daemon=True))

    for t in threads:
        t.start()

    filt = {a: FixedPosFilter(arp) for a in arms}
    fout = open(args.out, "w", newline="")
    w = csv.writer(fout)
    w.writerow(["host_timestamp"] +
               sum([["dt_rx_%s_ns" % a, "n_%s" % a] for a in arms], []))

    t0 = time.time()
    prev_t = None
    n = 0
    try:
        while time.time() - t0 < args.duration and not stop.is_set():
            try:
                gps_time, obs = obs_q.get(timeout=5)
            except Exception:
                continue
            if len(obs) < 4:
                continue
            dt = (gps_time - prev_t).total_seconds() if prev_t else 1.0
            prev_t = gps_time
            row = [datetime.now(timezone.utc).isoformat()]
            for a in arms:
                corr = RealtimeCorrections(beph, ssr[a])
                if 0 < dt <= 30:
                    filt[a].predict(dt)
                try:
                    n_used, _resid, _ntd = filt[a].update(
                        obs, corr, gps_time, clk_file=corr)
                except Exception:
                    n_used = 0
                dtrx = filt[a].x[filt[a].IDX_CLK] / C * 1e9
                row += ["%.4f" % dtrx, n_used]
            w.writerow(row)
            fout.flush()
            n += 1
            if n % 10 == 0:
                summ = "  ".join("%s=%.0fns(n%d)" % (
                    a, float(row[1 + 2 * i]), int(row[2 + 2 * i]))
                    for i, a in enumerate(arms))
                print("[%d] %s" % (n, summ))
    finally:
        stop.set()
        fout.close()
        ubx_f.close()
        print("wrote %d epochs to %s" % (n, args.out))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""log_ssr_corrections.py — log all four SSR correction components (orbit,
clock, code bias, phase bias) per satellite to a long-format CSV in both
native (metres) and time (ns) units, for the corrections-visualization
story (docs/visual-stories.md).

Reuses the engine's NtripStream + SSRState decoder.  Every ``--interval``
seconds it snapshots the live SSR state and appends one row per
(sv, component[, signal]).  Output is "long" so it's trivial to pivot at
plot time:

    utc_iso,mono_s,prn,sys,component,signal,value_m,value_ns,iod,epoch_s,age_s,disc

components: orbit_radial / orbit_along / orbit_cross / clock_c0 / clock_c1 /
code_bias / phase_bias.  Bias rows carry the RINEX signal code; orbit/clock
rows leave ``signal`` blank.  value_ns = value_m / c * 1e9 (1 m = 3.3356 ns);
for the LOS-relevant timing view, orbit_radial is the closest single proxy.
``age_s`` = seconds since this correction was last received (snapshot time −
rx_time) — a true staleness measure, so a coverage dropout is detectable even
on the quasi-static biases that don't change value (use age > a few × cadence
to gap-break biases at plot time, instead of the value-change heuristic).
``disc`` = SSR phase/code-bias discontinuity counter (bias rows only); it
increments when the AC re-references that (sv, signal) bias — the flagged
phase-bias jumps the BeiDou plots show.  Blank for orbit/clock.

Usage:
    python3 scripts/log_ssr_corrections.py --conf ntrip-cnes.conf \
        --duration 86400 --interval 10 --out ssr_corrections.csv
"""
import argparse
import configparser
import sys
import time
from datetime import datetime, timezone

sys.path.insert(0, "scripts")
from ntrip_client import NtripStream
from ssr_corrections import SSRState

C = 299792458.0


def _m2ns(x):
    return x / C * 1e9


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conf", default="ntrip-cnes.conf",
                    help="NTRIP conf (ini with [ntrip] caster/port/mount/user/password/tls)")
    ap.add_argument("--duration", type=float, default=86400.0, help="seconds (default 24h)")
    ap.add_argument("--interval", type=float, default=10.0, help="snapshot cadence s")
    ap.add_argument("--out", default="ssr_corrections.csv")
    ap.add_argument("--report-every", type=float, default=1800.0)
    args = ap.parse_args()

    conf = configparser.ConfigParser()
    if not conf.read(args.conf):
        print(f"ERROR: cannot read conf {args.conf!r}", file=sys.stderr)
        return 2
    n = conf["ntrip"]
    stream = NtripStream(n["caster"], int(n["port"]), n["mount"],
                         user=n.get("user"), password=n.get("password"),
                         tls=n.getboolean("tls", fallback=True))
    ssr = SSRState()
    out = open(args.out, "w")
    out.write("utc_iso,mono_s,prn,sys,component,signal,value_m,value_ns,iod,epoch_s,age_s,disc\n")

    n_rows = 0

    def snapshot(now_mono):
        nonlocal n_rows
        now_dt = datetime.now(timezone.utc)
        utc = now_dt.isoformat(timespec="milliseconds")
        ts = f"{now_mono:.3f}"

        def age(c):                        # seconds since this correction was received
            rx = getattr(c, "rx_time", None)
            return f"{(now_dt - rx).total_seconds():.1f}" if rx else ""

        buf = []
        for prn, o in list(ssr._orbit.items()):
            a = age(o)
            for comp, val in (("orbit_radial", o.radial),
                              ("orbit_along", o.along),
                              ("orbit_cross", o.cross)):
                buf.append(f"{utc},{ts},{prn},{prn[0]},{comp},,"
                           f"{val:.4f},{_m2ns(val):.4f},"
                           f"{getattr(o, 'iod', '')},{getattr(o, 'epoch_s', 0.0):.1f},{a},\n")
        for prn, c in list(ssr._clock.items()):
            ep = f"{getattr(c, 'epoch_s', 0.0):.1f}"
            a = age(c)
            buf.append(f"{utc},{ts},{prn},{prn[0]},clock_c0,,"
                       f"{c.c0:.4f},{_m2ns(c.c0):.4f},,{ep},{a},\n")
            buf.append(f"{utc},{ts},{prn},{prn[0]},clock_c1,,"
                       f"{c.c1:.6f},{_m2ns(c.c1):.6f},,{ep},{a},\n")
        for kind, store in (("code_bias", ssr._code_bias),
                            ("phase_bias", ssr._phase_bias)):
            for prn, sigs in list(store.items()):
                for sig, b in list(sigs.items()):
                    buf.append(f"{utc},{ts},{prn},{prn[0]},{kind},{sig},"
                               f"{b.bias_m:.4f},{_m2ns(b.bias_m):.4f},,,"
                               f"{age(b)},{getattr(b, 'disc_counter', '')}\n")
        out.write("".join(buf))
        out.flush()
        n_rows += len(buf)

    print(f"connecting {n['caster']}:{n['port']}/{n['mount']} for "
          f"{args.duration:.0f}s, snapshot every {args.interval:.0f}s -> {args.out}",
          flush=True)
    t0 = time.monotonic()
    last_snap = -1e9
    last_report = t0
    for msg, meta in stream.messages_with_metadata():
        now = time.monotonic() - t0
        try:
            ssr.update_from_rtcm(msg)
        except Exception:
            pass
        if now - last_snap >= args.interval:
            snapshot(now)
            last_snap = now
        nowm = time.monotonic()
        if nowm - last_report >= args.report_every:
            ncb = sum(len(v) for v in ssr._code_bias.values())
            npb = sum(len(v) for v in ssr._phase_bias.values())
            print(f"  t={now:6.0f}s rows={n_rows} n_orbit={len(ssr._orbit)} "
                  f"n_clock={len(ssr._clock)} n_cbias={ncb} n_pbias={npb}", flush=True)
            last_report = nowm
        if now > args.duration:
            break
    out.close()
    print(f"done: {n_rows} rows over {args.duration:.0f}s -> {args.out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())

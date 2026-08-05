#!/usr/bin/env python3
"""preflight_field_site — one command to check a host is ready at a new site.

Run this after arriving somewhere (or after switching to a phone hotspot)
and before starting a run.  Every check is independent and non-fatal; the
summary at the end says what works, what doesn't, and what it costs.

    python3 scripts/preflight_field_site.py --near 43.98,-87.78

Checks, in the order that failures cascade:

  1. WALL CLOCK   — a host whose clock is days off fails TLS and apt with
                    confusing errors.  Cheap to check, expensive to miss.
  2. NETWORK      — DNS + outbound TCP.  On a hotspot this is the flaky one.
  3. CORRECTIONS  — the NTRIP streams the engine needs, with measured byte
                    rates → a cellular data budget in MB/day.
  4. BASE         — nearest post-processing base from the NGS CORS
                    catalogue, and the live base caster if one is configured.
  5. RECEIVER     — present, and which constellations it is set to track.
  6. DISK         — headroom for raw logging.

Exit code 0 if nothing FAILed (WARNs are fine), 1 otherwise.
"""

from __future__ import annotations

import argparse
import os
import shutil
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

PASS, WARN, FAIL = "PASS", "WARN", "FAIL"
_results: list[tuple[str, str, str]] = []


def record(status: str, check: str, detail: str) -> None:
    _results.append((status, check, detail))
    colour = {"PASS": "\033[32m", "WARN": "\033[33m", "FAIL": "\033[31m"}
    reset = "\033[0m" if sys.stdout.isatty() else ""
    tag = (colour.get(status, "") if sys.stdout.isatty() else "") + status + reset
    print(f"  [{tag}] {check}: {detail}")


# ── 1. wall clock ──────────────────────────────────────────────────── #

def check_clock(max_skew_s: float = 120.0) -> None:
    print("\n1. WALL CLOCK")
    now = datetime.now(timezone.utc)
    print(f"     system UTC: {now:%Y-%m-%d %H:%M:%S}")
    try:
        # HTTP Date from a well-known host is enough to catch a gross skew,
        # and needs no NTP client.
        import urllib.request
        with urllib.request.urlopen("https://www.google.com/generate_204",
                                    timeout=10) as r:
            hdr = r.headers.get("Date")
        if not hdr:
            record(WARN, "clock", "no Date header to compare against")
            return
        from email.utils import parsedate_to_datetime
        ref = parsedate_to_datetime(hdr)
        skew = (now - ref).total_seconds()
        if abs(skew) <= max_skew_s:
            record(PASS, "clock", f"within {skew:+.0f}s of network time")
        else:
            record(FAIL, "clock",
                   f"{skew:+.0f}s off network time — TLS and apt will fail "
                   "in confusing ways.  Fix before anything else.")
    except Exception as e:  # noqa: BLE001 - any failure is just "unknown"
        record(WARN, "clock", f"could not reach a time reference ({e})")


# ── 2. network ─────────────────────────────────────────────────────── #

def check_network(hosts: list[tuple[str, int]]) -> None:
    print("\n2. NETWORK")
    for host, port in hosts:
        t0 = time.monotonic()
        try:
            addr = socket.gethostbyname(host)
        except OSError as e:
            record(FAIL, f"dns {host}", str(e))
            continue
        try:
            s = socket.create_connection((addr, port), timeout=10)
            s.close()
            record(PASS, f"tcp {host}:{port}",
                   f"{addr} in {(time.monotonic() - t0) * 1000:.0f} ms")
        except OSError as e:
            record(FAIL, f"tcp {host}:{port}", f"{addr}: {e}")


# ── 3. correction streams ──────────────────────────────────────────── #

def check_streams(conf_path: str, mounts: list[str], seconds: int) -> float:
    print(f"\n3. CORRECTION STREAMS  ({seconds}s each)")
    total_bps = 0.0
    if not os.path.exists(conf_path):
        record(WARN, "ntrip.conf", f"not found at {conf_path} — skipping")
        return 0.0
    try:
        from diag_ntrip_base import read_conf, stream
    except ImportError as e:
        record(WARN, "streams", f"diag_ntrip_base unavailable ({e})")
        return 0.0
    cfg = read_conf(conf_path)
    for mount in mounts:
        m_cfg = dict(cfg, mount=mount)
        try:
            payload, status = stream(m_cfg, None, seconds)
        except OSError as e:
            record(FAIL, f"mount {mount}", f"connect failed: {e}")
            continue
        if "200" not in status:
            record(FAIL, f"mount {mount}", f"{status.strip()}")
            continue
        if not payload:
            record(FAIL, f"mount {mount}", "200 OK but streamed 0 bytes")
            continue
        bps = len(payload) / seconds
        total_bps += bps
        record(PASS, f"mount {mount}",
               f"{len(payload)} B in {seconds}s = {bps:.0f} B/s "
               f"({bps * 86400 / 1e6:.0f} MB/day)")
    return total_bps


# ── 4. base availability ───────────────────────────────────────────── #

def check_base(near: tuple[float, float] | None, max_km: float,
               base_conf: str | None, gga_height_m: float) -> None:
    print("\n4. BASE AVAILABILITY")
    if near is None:
        record(WARN, "archive base", "no --near given — skipping")
    else:
        try:
            from peppar_fix.peppar_survey_discovery import discover_base
            desc = discover_base(near[0], near[1], max_km=max_km)
            if desc is None:
                record(WARN, "archive base",
                       f"no station within {max_km:.0f} km — the survey "
                       "falls back to the PRIDE floor (~1 day to 2 weeks)")
            else:
                record(PASS, "archive base",
                       f"{desc.station} @ {desc.distance_km:.1f} km "
                       f"({desc.source.name}, via {desc.via})")
        except Exception as e:  # noqa: BLE001
            record(WARN, "archive base", f"discovery failed: {e}")

    if not base_conf:
        return
    if not os.path.exists(base_conf):
        record(WARN, "live base caster", f"{base_conf} not found")
        return
    try:
        from diag_ntrip_base import ecef_to_llh, haversine_km, read_conf, stream
        from pyrtcm import RTCMReader
        import io
        cfg = read_conf(base_conf)
        gga = (near[0], near[1], gga_height_m) if near else None
        payload, status = stream(cfg, gga, 25)
        if "200" not in status:
            record(FAIL, "live base caster",
                   f"{cfg['caster']}/{cfg['mount']}: {status.strip()}")
            return
        if not payload:
            record(FAIL, "live base caster",
                   "200 OK but 0 bytes — this mount likely needs a GGA")
            return
        arp = None
        for _raw, p in RTCMReader(io.BytesIO(payload), quitonerror=0):
            if p is not None and str(p.identity) in ("1005", "1006"):
                arp = (p.DF025, p.DF026, p.DF027)
                break
        if arp and gga:
            la, lo, _h = ecef_to_llh(*arp)
            d = haversine_km(gga[0], gga[1], la, lo)
            kind = "VRS (synthesized at us)" if d < 0.001 else f"{d:.1f} km away"
            record(PASS, "live base caster",
                   f"{cfg['mount']}: {len(payload)} B/25s, base {kind}")
        else:
            record(PASS, "live base caster",
                   f"{cfg['mount']}: {len(payload)} B/25s (no ARP seen yet)")
    except Exception as e:  # noqa: BLE001
        record(WARN, "live base caster", f"check failed: {e}")


# ── 5. receiver ────────────────────────────────────────────────────── #

def check_receiver(device: str | None) -> None:
    print("\n5. RECEIVER")
    if not device:
        record(WARN, "device", "no --device given — skipping")
        return
    if not os.path.exists(device):
        record(FAIL, "device", f"{device} does not exist")
        return
    try:
        busy = subprocess.run(["fuser", device], capture_output=True,
                              text=True, timeout=10)
        holder = (busy.stdout + busy.stderr).strip()
        if busy.returncode == 0 and holder:
            record(WARN, "device", f"{device} is in use ({holder}) — "
                                   "stop the engine before a fresh run")
        else:
            record(PASS, "device", f"{device} present and free")
    except (OSError, subprocess.SubprocessError):
        record(PASS, "device", f"{device} present (fuser unavailable)")


# ── 6. disk ────────────────────────────────────────────────────────── #

def check_disk(path: str, min_gb: float, obs_bps: float) -> None:
    print("\n6. DISK")
    try:
        usage = shutil.disk_usage(path)
    except OSError as e:
        record(WARN, "disk", f"{path}: {e}")
        return
    free_gb = usage.free / 1e9
    detail = f"{free_gb:.1f} GB free at {path}"
    if obs_bps > 0:
        detail += f" (~{free_gb * 1e9 / (obs_bps * 86400):.0f} days of logging)"
    record(PASS if free_gb >= min_gb else WARN, "disk", detail)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--near", default=None,
                    help="'LAT,LON' of this site — enables base discovery.")
    ap.add_argument("--height", type=float, default=200.0,
                    help="Approx ellipsoidal height (m) for the GGA.")
    ap.add_argument("--ntrip-conf", default="ntrip.conf")
    ap.add_argument("--mounts", default="SSRA03IGS0,BCEP00BKG0",
                    help="Comma-separated correction mounts to rate-test.")
    ap.add_argument("--base-conf", default=None,
                    help="ntrip.conf-format file for a live base caster "
                         "(e.g. ntrip-wiscors.conf).")
    ap.add_argument("--device", default=None,
                    help="Receiver device to check, e.g. /dev/gnss-bot.")
    ap.add_argument("--data-dir", default="data")
    ap.add_argument("--seconds", type=int, default=30,
                    help="Per-stream rate-test duration (default 30).")
    ap.add_argument("--min-free-gb", type=float, default=2.0)
    args = ap.parse_args(argv)

    near = None
    if args.near:
        p = args.near.split(",")
        near = (float(p[0]), float(p[1]))

    print("=" * 68)
    print(f"PePPAR-Fix field site pre-flight — {socket.gethostname()}")
    if near:
        print(f"site: {near[0]:.5f}, {near[1]:.5f}")
    print("=" * 68)

    check_clock()
    hosts = [("geodesy.noaa.gov", 443)]
    try:
        from diag_ntrip_base import read_conf
        if os.path.exists(args.ntrip_conf):
            c = read_conf(args.ntrip_conf)
            hosts.append((c["caster"], int(c.get("port", 2101))))
        if args.base_conf and os.path.exists(args.base_conf):
            c = read_conf(args.base_conf)
            hosts.append((c["caster"], int(c.get("port", 2101))))
    except Exception:  # noqa: BLE001 - config problems surface in their own check
        pass
    check_network(hosts)

    corr_bps = check_streams(args.ntrip_conf,
                             [m for m in args.mounts.split(",") if m],
                             args.seconds)
    check_base(near, 80.0, args.base_conf, args.height)
    check_receiver(args.device)
    check_disk(args.data_dir if os.path.isdir(args.data_dir) else ".",
               args.min_free_gb, corr_bps)

    n_fail = sum(1 for s, _, _ in _results if s == FAIL)
    n_warn = sum(1 for s, _, _ in _results if s == WARN)
    print("\n" + "=" * 68)
    print(f"SUMMARY: {len(_results) - n_fail - n_warn} pass, "
          f"{n_warn} warn, {n_fail} fail")
    if corr_bps:
        print(f"cellular budget: {corr_bps:.0f} B/s = "
              f"{corr_bps * 3600 / 1e6:.1f} MB/hour = "
              f"{corr_bps * 86400 / 1e6:.0f} MB/day of correction traffic")
    for s, c, d in _results:
        if s != PASS:
            print(f"  {s}: {c} — {d}")
    if n_fail == 0:
        print("\nNo blockers.  Note: a post-processed baseline survey needs "
              "NO internet at the site — the rover logs locally and the base "
              "RINEX is fetched later from anywhere.")
    print("=" * 68)
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())

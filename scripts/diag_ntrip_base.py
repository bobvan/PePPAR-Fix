#!/usr/bin/env python3
"""diag_ntrip_base — prove a base NTRIP caster config actually works.

Answers the four questions you have before trusting a caster as a survey
base, without needing RTKLIB installed:

  1. Do the credentials authenticate?
  2. Does the mount stream anything?  (A caster that wants a GGA and
     doesn't get one returns a clean 200 and then silence — the failure
     looks exactly like success until you check the byte count.)
  3. WHICH station did it give us?  Network/nearest-base mounts pick or
     synthesize the base from our GGA, so the answer depends on where we
     say we are.  Decoded from RTCM 1005/1006, with the baseline length
     from our GGA point.
  4. Which constellations does it carry?  (MSM message numbers.)

Reads the same one-section-per-file ``[ntrip]`` config the engine and
``peppar-survey --base-ntrip-conf`` use.  Never prints the password.

    diag_ntrip_base.py ~/peppar-fix/ntrip-wiscors.conf \\
        --gga 43.98,-87.78,200 --seconds 20

Exit codes: 0 = streamed and decoded, 1 = connected but no usable data,
2 = config/connection error.
"""

from __future__ import annotations

import argparse
import base64
import configparser
import math
import os
import socket
import sys
import time
from datetime import datetime, timezone

# RTCM message groups we report on, by what they tell us.
_ARP_MSGS = {1005, 1006}
_ANT_MSGS = {1007, 1008, 1033}
# MSM7/6/5/4 blocks are 1070+ per constellation, 10 apart.
_MSM_BASE = {1070: "GPS", 1080: "GLONASS", 1090: "Galileo", 1100: "SBAS",
             1110: "QZSS", 1120: "BeiDou", 1130: "NavIC"}
# Legacy (pre-MSM) observables.  A stream can be perfectly usable while
# carrying none of the 107x+ blocks — RTCM 3.1 casters commonly serve these,
# and reporting "no constellations" for them is just wrong.
_LEGACY_OBS = {1001: "GPS", 1002: "GPS", 1003: "GPS", 1004: "GPS",
               1009: "GLONASS", 1010: "GLONASS",
               1011: "GLONASS", 1012: "GLONASS"}


def constellation_for(msgid: int) -> str | None:
    if msgid in _LEGACY_OBS:
        return _LEGACY_OBS[msgid]
    return _MSM_BASE.get((msgid // 10) * 10)


def is_msm(msgid: int) -> bool:
    return (msgid // 10) * 10 in _MSM_BASE


def _joined_chars(parsed, prefix: str) -> str:
    """Reassemble pyrtcm's per-character DFxxx_NN fields into a string.

    1007/1008/1033 carry descriptors one character per field (DF030_01,
    DF030_02, ...), so there is no single attribute to read.
    """
    out, i = [], 1
    while True:
        ch = getattr(parsed, f"{prefix}_{i:02d}", None)
        if ch is None:
            break
        out.append(str(ch))
        i += 1
    return "".join(out).strip()


def nmea_gga(lat: float, lon: float, height_m: float) -> bytes:
    """A minimal but valid GGA — enough for a caster to place us."""
    now = datetime.now(timezone.utc)
    hhmmss = now.strftime("%H%M%S.00")
    lat_h, lon_h = ("N" if lat >= 0 else "S"), ("E" if lon >= 0 else "W")
    lat_a, lon_a = abs(lat), abs(lon)
    lat_dm = f"{int(lat_a):02d}{(lat_a - int(lat_a)) * 60:09.6f}"
    lon_dm = f"{int(lon_a):03d}{(lon_a - int(lon_a)) * 60:09.6f}"
    body = (f"GPGGA,{hhmmss},{lat_dm},{lat_h},{lon_dm},{lon_h},"
            f"1,10,1.0,{height_m:.2f},M,0.0,M,,")
    csum = 0
    for ch in body:
        csum ^= ord(ch)
    return f"${body}*{csum:02X}\r\n".encode("ascii")


def ecef_to_llh(x: float, y: float, z: float) -> tuple[float, float, float]:
    """WGS84 ECEF → (lat_deg, lon_deg, height_m).  Bowring's method."""
    a, f = 6378137.0, 1 / 298.257223563
    b = a * (1 - f)
    e2, ep2 = f * (2 - f), (a * a - b * b) / (b * b)
    p = math.hypot(x, y)
    th = math.atan2(a * z, b * p)
    lat = math.atan2(z + ep2 * b * math.sin(th) ** 3,
                     p - e2 * a * math.cos(th) ** 3)
    n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    return (math.degrees(lat), math.degrees(math.atan2(y, x)),
            p / math.cos(lat) - n)


def haversine_km(lat1, lon1, lat2, lon2) -> float:
    r = 6371.0088
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    h = (math.sin(dp / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
    return 2 * r * math.asin(math.sqrt(h))


def read_conf(path: str) -> dict:
    if not os.path.exists(path):
        raise SystemExit(f"error: no such file: {path}")
    conf = configparser.ConfigParser()
    conf.read(path)
    if "ntrip" not in conf:
        raise SystemExit(f"error: {path}: no [ntrip] section")
    return dict(conf["ntrip"])


def stream(cfg: dict, gga: tuple[float, float, float] | None,
           seconds: int, gga_cycle_s: int = 10) -> tuple[bytes, str]:
    """Connect, optionally send GGA, read for `seconds`.  → (payload, status)"""
    host, port = cfg["caster"], int(cfg.get("port", 2101))
    mount = cfg["mount"]
    use_tls = str(cfg.get("tls", "false")).lower() in ("1", "true", "yes")

    req = (f"GET /{mount} HTTP/1.1\r\n"
           f"Host: {host}:{port}\r\n"
           "Ntrip-Version: Ntrip/2.0\r\n"
           "User-Agent: NTRIP peppar-fix/1.0\r\n")
    if cfg.get("user"):
        tok = base64.b64encode(
            f"{cfg['user']}:{cfg.get('password', '')}".encode()).decode()
        req += f"Authorization: Basic {tok}\r\n"
    req += "Connection: close\r\n\r\n"

    sock = socket.create_connection((host, port), timeout=15)
    if use_tls:
        import ssl
        sock = ssl.create_default_context().wrap_socket(
            sock, server_hostname=host)
    try:
        sock.sendall(req.encode())
        sock.settimeout(5)
        buf = b""
        # Read just the response head first so we can report auth failures
        # as auth failures rather than as "no data".
        while b"\r\n\r\n" not in buf and len(buf) < 8192:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                break
            if not chunk:
                break
            buf += chunk
        head, _, rest = buf.partition(b"\r\n\r\n")
        status = head.split(b"\r\n")[0].decode("latin-1", "replace") \
            if head else "(no response)"
        if b"200" not in head.split(b"\r\n")[0]:
            return rest, status

        payload = rest
        deadline = time.monotonic() + seconds
        last_gga = 0.0
        while time.monotonic() < deadline:
            if gga is not None and time.monotonic() - last_gga >= gga_cycle_s:
                try:
                    sock.sendall(nmea_gga(*gga))
                except OSError:
                    pass
                last_gga = time.monotonic()
            try:
                chunk = sock.recv(8192)
            except socket.timeout:
                continue
            if not chunk:
                break
            payload += chunk
        return payload, status
    finally:
        try:
            sock.close()
        except OSError:
            pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("conf", help="ntrip.conf-format file ([ntrip] section)")
    ap.add_argument("--mount", default=None, help="Override conf mount.")
    ap.add_argument("--gga", default=None,
                    help="'LAT,LON[,HEIGHT_M]' station position to send. "
                         "Required by nmea=1 mounts (all WISCORS mounts).")
    ap.add_argument("--seconds", type=int, default=20,
                    help="Capture duration (default 20).")
    ap.add_argument("--save", default=None, help="Write raw RTCM here.")
    args = ap.parse_args(argv)

    cfg = read_conf(args.conf)
    if args.mount:
        cfg["mount"] = args.mount
    gga = None
    if args.gga:
        parts = [float(p) for p in args.gga.split(",")]
        gga = (parts[0], parts[1], parts[2] if len(parts) > 2 else 0.0)

    print(f"caster : {cfg['caster']}:{cfg.get('port', 2101)}  "
          f"mount={cfg['mount']}  tls={cfg.get('tls', 'false')}")
    print(f"user   : {cfg.get('user', '(anonymous)')}  "
          f"password={'set' if cfg.get('password') else 'NOT SET'}")
    print(f"gga    : {gga if gga else 'not sent'}")

    try:
        payload, status = stream(cfg, gga, args.seconds)
    except OSError as e:
        print(f"\nFAIL connect: {e}")
        return 2

    print(f"status : {status}")
    print(f"bytes  : {len(payload)} in {args.seconds}s")
    if "200" not in status:
        print("\nFAIL: caster refused the request "
              "(401 = bad credentials, 404 = no such mount).")
        return 2
    if not payload:
        print("\nFAIL: authenticated but streamed nothing. "
              "If this mount is nmea=1 it needs --gga.")
        return 1
    if args.save:
        with open(args.save, "wb") as f:
            f.write(payload)
        print(f"saved  : {args.save}")

    try:
        from pyrtcm import RTCMReader
    except ImportError:
        print("\n(pyrtcm not installed — byte count only)")
        return 0

    import io
    counts: dict[int, int] = {}
    arp = None
    phys = None
    antenna: list[str] = []
    receiver: list[str] = []
    reader = RTCMReader(io.BytesIO(payload), quitonerror=0)
    for _raw, parsed in reader:
        if parsed is None:
            continue
        try:
            mid = int(parsed.identity)
        except (AttributeError, ValueError):
            continue
        counts[mid] = counts.get(mid, 0) + 1
        if mid in _ARP_MSGS and arp is None:
            try:
                arp = (parsed.DF025, parsed.DF026, parsed.DF027)
            except AttributeError:
                pass
        if mid == 1032 and phys is None:
            # "Physical Reference Station Position" — a VRS names the real
            # station its network solution is anchored to.  Worth having:
            # it says whether the synthesized base is backed by the monument
            # you think it is.
            try:
                phys = ((parsed.DF025, parsed.DF026, parsed.DF027),
                        getattr(parsed, "DF023", None))
            except AttributeError:
                pass
        if mid in _ANT_MSGS:
            desc = _joined_chars(parsed, "DF030")     # antenna descriptor
            if desc:
                antenna.append(desc)
            rcv = _joined_chars(parsed, "DF228")      # receiver type (1033)
            if rcv:
                receiver.append(rcv)

    print("\nmessages:")
    for mid in sorted(counts):
        sysname = constellation_for(mid)
        tag = ""
        if sysname:
            tag = f"  ({sysname} {'MSM' if is_msm(mid) else 'legacy obs'})"
        print(f"  {mid:5d} x{counts[mid]:<4d}{tag}")

    if not counts:
        print("\nFAIL: bytes received but no RTCM decoded — wrong format?")
        return 1

    consts = sorted({constellation_for(m) for m in counts
                     if constellation_for(m)})
    obs_kind = ("MSM" if any(is_msm(m) for m in counts)
                else "legacy (1001-1012)" if consts else "none")
    print(f"\nconstellations: {', '.join(consts) if consts else 'NO OBSERVABLES'}"
          f"   [obs format: {obs_kind}]")

    if arp:
        lat, lon, h = ecef_to_llh(*arp)
        print(f"base ARP      : {lat:.8f}, {lon:.8f}, {h:.3f} m  "
              f"(ECEF {arp[0]:.4f}, {arp[1]:.4f}, {arp[2]:.4f})")
        if gga:
            d = haversine_km(gga[0], gga[1], lat, lon)
            print(f"baseline      : {d:.2f} km from the GGA point")
            if d < 0.001:
                print("  NOTE: ~0 km means this is a VRS — the 'base' is "
                      "synthesized AT your position, not a real monument.")
    else:
        print("base ARP      : not seen (no 1005/1006 in this window — "
              "they are usually sent every 10-30 s, try --seconds 40)")
    if phys:
        (px, py, pz), pid = phys
        plat, plon, ph = ecef_to_llh(px, py, pz)
        extra = f"  id={pid}" if pid is not None else ""
        print(f"phys ref stn  : {plat:.8f}, {plon:.8f}, {ph:.3f} m{extra}"
              "   (RTCM 1032 — the real station behind the network solution)")
        if gga:
            print(f"                {haversine_km(gga[0], gga[1], plat, plon):.2f}"
                  " km from the GGA point")
    if antenna:
        ants = sorted(set(antenna))
        print(f"antenna       : {', '.join(ants)}")
        if any("NULLANTENNA" in a.upper() for a in ants):
            print("  NOTE: a NULL antenna descriptor means the caster has "
                  "already removed the base antenna's phase-centre "
                  "variations.  Post-processing must NOT apply a PCV model "
                  "to this base, or the correction is double-counted.")
    if receiver:
        print(f"receiver      : {', '.join(sorted(set(receiver)))}")

    return 0


if __name__ == "__main__":
    sys.exit(main())

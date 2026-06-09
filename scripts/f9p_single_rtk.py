#!/usr/bin/env python3
"""Drive one F9P as an RTK rover from ONE NTRIP caster; report the
RTK-FIXED ECEF position.  For PATCH3 multi-caster cross-check
(onocoy vs Wheaton vs Naperville).  Reuses configure_rover() from
f9p_dual_cors_rover.py.  Raw-socket NTRIP (auth + optional GGA) so
the roaming onocoy NRBY_ADV mount works."""
import sys, time, socket, ssl, base64, threading, argparse, statistics
import serial
from pyubx2 import UBXReader
sys.path.insert(0, "/home/bob/peppar-fix/scripts")
from f9p_dual_cors_rover import configure_rover

def make_gga(lat, lon, alt=199.0):
    la, lo = abs(lat), abs(lon)
    latm = f"{int(la):02d}{(la-int(la))*60:08.5f}"
    lonm = f"{int(lo):03d}{(lo-int(lo))*60:08.5f}"
    body = (f"GPGGA,000000.00,{latm},{'N' if lat>=0 else 'S'},"
            f"{lonm},{'E' if lon>=0 else 'W'},1,12,0.8,{alt:.1f},M,-34.0,M,,")
    cs = 0
    for c in body: cs ^= ord(c)
    return f"${body}*{cs:02X}\r\n".encode()

def feed(host, port, mount, user, pw, gga, ser, stop, st, tls=False):
    auth = base64.b64encode(f"{user}:{pw}".encode()).decode() if user else None
    try:
        s = socket.create_connection((host, port), timeout=15)
        if tls:  # TLS casters (e.g. clients.onocoy.com:2121) reset plain sockets
            s = ssl.create_default_context().wrap_socket(s, server_hostname=host)
        req = (f"GET /{mount} HTTP/1.1\r\nHost: {host}:{port}\r\n"
               f"User-Agent: NTRIP f9p\r\nNtrip-Version: Ntrip/2.0\r\n")
        if auth: req += f"Authorization: Basic {auth}\r\n"
        req += "\r\n"
        s.sendall(req.encode())
        s.settimeout(10)
        hdr = b""
        while b"\r\n\r\n" not in hdr and len(hdr) < 4096:
            d = s.recv(1)
            if not d: break
            hdr += d
        st['ntrip'] = hdr.split(b"\r\n")[0].decode(errors='replace')
        if b"200" not in hdr[:40] and b"ICY" not in hdr[:40]:
            stop.set(); return
        if gga: s.sendall(gga)
        last = time.time(); s.settimeout(2)
        while not stop.is_set():
            try:
                d = s.recv(4096)
                if not d: break
                ser.write(d); st['rtcm'] += len(d)
            except socket.timeout:
                pass
            if gga and time.time()-last > 10:
                try: s.sendall(gga)
                except Exception: pass
                last = time.time()
        s.close()
    except Exception as e:
        st['err'] = str(e); stop.set()

def monitor(ser, stop, st):
    ubr = UBXReader(ser, protfilter=2)
    soln = 0; sv = 0
    while not stop.is_set():
        try: raw, p = ubr.read()
        except Exception: continue
        if p is None: continue
        i = getattr(p, "identity", "")
        if i == "NAV-PVT":
            try: soln = int(p.carrSoln); sv = int(p.numSV)
            except Exception: pass
            continue
        if i == "NAV-HPPOSECEF":
            try:
                if int(getattr(p, "invalidEcef", 0)): continue
                x = float(p.ecefX)*0.01; y = float(p.ecefY)*0.01; z = float(p.ecefZ)*0.01
                pacc = float(p.pAcc)
            except Exception: continue
            lbl = {0:"NONE",1:"FLOAT",2:"FIXED"}.get(soln, "?")
            st['soln'][lbl] = st['soln'].get(lbl, 0) + 1
            st['last'] = (x, y, z, pacc, lbl, sv)
            if lbl == "FIXED":
                if st['ttff'] is None: st['ttff'] = time.monotonic()-st['t0']
                st['fixed'].append((x, y, z, pacc))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/ttyACM2")
    ap.add_argument("--baud", type=int, default=38400)
    ap.add_argument("--host", required=True); ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--mount", required=True)
    ap.add_argument("--user", default=None); ap.add_argument("--pw", default=None)
    ap.add_argument("--tls", action="store_true", help="wrap NTRIP socket in TLS (e.g. clients.onocoy.com:2121)")
    ap.add_argument("--gga-lat", type=float, default=None); ap.add_argument("--gga-lon", type=float, default=None)
    ap.add_argument("--collect", type=int, default=45, help="seconds of FIXED to collect")
    ap.add_argument("--maxwait", type=int, default=240)
    a = ap.parse_args()
    gga = make_gga(a.gga_lat, a.gga_lon) if a.gga_lat is not None else None
    ser = serial.Serial(a.dev, a.baud, timeout=1)
    print(f"configuring F9P {a.dev} for RTK rover ...", flush=True)
    configure_rover(ser); time.sleep(1)
    st = {'rtcm':0,'soln':{},'fixed':[],'last':None,'ttff':None,'ntrip':'','err':'','t0':time.monotonic()}
    stop = threading.Event()
    tf = threading.Thread(target=feed, args=(a.host,a.port,a.mount,a.user,a.pw,gga,ser,stop,st,a.tls), daemon=True)
    tm = threading.Thread(target=monitor, args=(ser,stop,st), daemon=True)
    tf.start(); tm.start()
    t0 = time.monotonic(); fixed_start = None
    while time.monotonic()-t0 < a.maxwait:
        time.sleep(3)
        L = st['last']
        msg = f"  t={time.monotonic()-t0:4.0f}s rtcm={st['rtcm']//1024}KB soln={st['soln']} ntrip='{st['ntrip']}'"
        if L: msg += f" sv={L[5]} pAcc={L[3]:.1f}mm last={L[4]}"
        print(msg, flush=True)
        if st['err']: print("  ERR:", st['err']); break
        if st['fixed']:
            if fixed_start is None: fixed_start = time.monotonic()
            if time.monotonic()-fixed_start >= a.collect: break
    stop.set(); time.sleep(0.5)
    fx = st['fixed']
    print(f"\n=== RESULT {a.mount} @ {a.host} ===")
    print(f"  ntrip: {st['ntrip']}  rtcm: {st['rtcm']//1024} KB  soln dist: {st['soln']}")
    if not fx:
        print("  NO RTK-FIXED samples obtained."); 
        if st['last']: print(f"  best soln seen: {st['last'][4]} pAcc={st['last'][3]:.0f}mm sv={st['last'][5]}")
        return
    xs=[f[0] for f in fx]; ys=[f[1] for f in fx]; zs=[f[2] for f in fx]
    mx,my,mz=statistics.mean(xs),statistics.mean(ys),statistics.mean(zs)
    sd = (statistics.pstdev(xs)**2+statistics.pstdev(ys)**2+statistics.pstdev(zs)**2)**0.5
    print(f"  TTFF: {st['ttff']:.0f}s  | FIXED samples: {len(fx)}  | mean pAcc: {statistics.mean(f[3] for f in fx):.0f}mm")
    print(f"  ECEF mean (m): X={mx:.4f} Y={my:.4f} Z={mz:.4f}  (3D scatter σ={sd*1000:.0f}mm)")
    print(f"  ECEF_FIX={mx:.4f},{my:.4f},{mz:.4f}")

if __name__ == "__main__":
    main()

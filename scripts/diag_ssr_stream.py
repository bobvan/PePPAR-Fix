#!/usr/bin/env python3
"""Sample the CNES SSR correction stream directly and characterize its
stability — pickyEaterSSR (I-210733): is the time-varying position
excursion driven by the SSR stream or by atmosphere/geometry?
Reuses the engine's NtripStream + SSRState decoder."""
import sys, time, configparser, math
sys.path.insert(0, "scripts")
from ntrip_client import NtripStream
from ssr_corrections import SSRState

conf = configparser.ConfigParser(); conf.read(sys.argv[2] if len(sys.argv)>2 else "ntrip-cnes.conf")
n = conf["ntrip"]
dur = int(sys.argv[1]) if len(sys.argv) > 1 else 900
stream = NtripStream(n["caster"], int(n["port"]), n["mount"],
                     user=n.get("user"), password=n.get("password"),
                     tls=n.getboolean("tls", fallback=True))
ssr = SSRState()
t0 = time.monotonic(); last = -1e9; last_msg = None
gaps = []; samples = []  # (t, {prn:c0})
out = open("/tmp/ssr_clock_samples.csv", "w"); out.write("t_s,prn,c0_m,c1_mps\n")
print(f"connecting {n['caster']}:{n['port']}/{n['mount']} for {dur}s", flush=True)
for msg, meta in stream.messages_with_metadata():
    now = time.monotonic() - t0
    try:
        ssr.update_from_rtcm(msg)
    except Exception:
        pass
    if last_msg is not None:
        dt = now - last_msg
        if dt > 15.0: gaps.append  # real dropout (>>~5s cadence)((round(last_msg,1), round(dt,1)))
    last_msg = now
    if now - last >= 10.0:
        snap = {}
        for prn, c in list(ssr._clock.items()):
            snap[prn] = c.c0
            out.write(f"{now:.1f},{prn},{c.c0:.4f},{getattr(c,'c1',0.0):.6f}\n")
        samples.append((now, snap)); out.flush()
        print(f"  t={now:5.0f}s n_clock={len(snap)} n_orbit={len(ssr._orbit)}", flush=True)
        last = now
    if now > dur: break
out.close()
print(f"\n=== SSR STREAM STABILITY ({len(samples)} snapshots over {dur}s) ===")
print(f"update gaps >5s: {len(gaps)}" + (f" -> {gaps[:8]}" if gaps else " (none)"))
# per-SV clock c0: drift + jumps; common-mode
allp = set().union(*[set(s.keys()) for _,s in samples]) if samples else set()
import statistics as st
cm = []  # common-mode (mean c0 across SVs) per snapshot
for t,s in samples:
    if s: cm.append((t, st.mean(s.values())))
if len(cm) > 2:
    cmv = [v for _,v in cm]; tt=[t for t,_ in cm]
    # linear drift of common-mode
    n2=len(cmv); mt=st.mean(tt); mv=st.mean(cmv)
    sxx=sum((t-mt)**2 for t in tt); sxy=sum((tt[i]-mt)*(cmv[i]-mv) for i in range(n2))
    slope=sxy/sxx if sxx else 0
    print(f"common-mode clock c0: mean={mv:.3f}m range[{min(cmv):.3f},{max(cmv):.3f}] drift={slope*3600:.3f} m/hr std={st.pstdev(cmv):.3f}m")
# per-SV jumps (max |Δc0| between consecutive snapshots)
maxjump=0; jumpsv=None
for prn in allp:
    series=[(t,s[prn]) for t,s in samples if prn in s]
    for i in range(1,len(series)):
        dt=series[i][0]-series[i-1][0]; dc=abs(series[i][1]-series[i-1][1])
        if dt<30 and dc>maxjump: maxjump=dc; jumpsv=(prn, round(series[i][0]), round(dc,3))
print(f"largest per-SV clock-c0 jump between adjacent ~10s snaps: {maxjump:.3f}m at {jumpsv}")
print(f"(context: a smooth low-drift stream with no gaps => SSR is NOT the time-varying driver; jumps/gaps/large common-mode drift => SSR implicated)")

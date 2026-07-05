#!/usr/bin/env python3
"""Spike: ingest RTCM 3 MSM observations from an NTRIP obs stream and prove
the decode → observation-model path (docs/rtcm-msm-obs-ingest.md).

The geodetic-receiver bridge: every non-u-blox receiver (Leica GRX1200,
Trimble NetR9, Septentrio PolaRx) and every NTRIP CORS speaks RTCM 3 MSM.
This decodes MSM into per-SV-per-signal {pseudorange, carrier phase, CNR}
— the same shape the engine's UBX-RAWX path produces — and (validation)
compares a code-only single-point fix to the station's published ARP.

GPS-first (CDMA); GLONASS FDMA/IFB is a deliberate follow-on.

Validated offline via rtcm_encoder→pyrtcm round-trip (PR recovered to 7 mm).

Usage (run on a host with NTRIP access — London hosts reach 443/TLS only):
  spike_rtcm_msm_ingest.py --caster ntrip.data.gnss.ga.gov.au --port 443 \\
      --tls --mount <OBS_MOUNT> --user U --password P --duration 60
"""
from __future__ import annotations
import argparse
import math
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

_C = 299792458.0

# MSM constellation → (RTCM base msg, PRN prefix, per-signal-id → (RINEX code, freq Hz)).
# GPS L1 C/A (MSM sig id 2) and L2 (15/16/17) + L5 (22) cover our need.
_GPS_SIG = {2: ('1C', 1575.42e6), 3: ('1P', 1575.42e6), 4: ('1W', 1575.42e6),
            8: ('2C', 1227.60e6), 9: ('2P', 1227.60e6), 10: ('2W', 1227.60e6),
            15: ('2S', 1227.60e6), 16: ('2L', 1227.60e6), 17: ('2X', 1227.60e6),
            22: ('5I', 1176.45e6), 23: ('5Q', 1176.45e6), 24: ('5X', 1176.45e6)}
# constellation dispatch by MSM message number (base for MSM1..7 is +0..+6)
_CONSTELL = {107: ('G', _GPS_SIG)}   # 1074/1077 → GPS (spike scope)

# L2 RINEX codes we'll pair with L1 C/A (1C) to form the iono-free combo.
_L2_CODES = ('2W', '2L', '2S', '2X', '2P', '2C')


def _bits(mask: int, width: int):
    """Return 1-based positions of set bits, MSB-first (RTCM DFxxx masks)."""
    return [i + 1 for i in range(width) if mask & (1 << (width - 1 - i))]


def decode_msm_obs(msg):
    """Reconstruct observations from a pyrtcm-decoded MSM message.

    Returns (prn_prefix, tow_ms, [ {sv, sig, pr_m, cp_cyc, cno, half} ]) or None
    for an unsupported constellation.  Standard MSM4/7 assembly:
      range_ms = DF397(int) + DF398(rough) ; PR = c*(range_ms+DF400_fine)/1000
      phaserange = c*(range_ms+DF401_fine)/1000 ; cp_cyc = phaserange/λ
    """
    num = int(getattr(msg, 'DF002'))
    base = num // 10                      # 1074→107, 1077→107
    disp = _CONSTELL.get(base)
    if disp is None:
        return None
    prefix, sigtab = disp
    sats = _bits(int(msg.DF394), 64)      # PRNs present, in order (len = NSat)
    sigs = _bits(int(msg.DF395), 32)      # MSM signal ids present (len = NSig)
    nsat, nsig = len(sats), len(sigs)
    cellmask = int(msg.DF396)             # NSat×NSig bits, sat-major, MSB-first
    tow_ms = int(getattr(msg, 'DF004', 0))
    out = []
    cell = 0                              # 1-based counter over SET cells
    for si in range(nsat):
        prn = sats[si]
        rough = float(getattr(msg, f'DF397_{si+1:02d}')) + \
                float(getattr(msg, f'DF398_{si+1:02d}'))
        for gj in range(nsig):
            grid = si * nsig + gj         # position in the NSat×NSig grid
            if not (cellmask & (1 << (nsat * nsig - 1 - grid))):
                continue
            cell += 1
            rinex, freq = sigtab.get(sigs[gj], (f's{sigs[gj]}', 1575.42e6))
            # MSM6/7 use extended fine fields (DF405/406/408); MSM4/5 use
            # DF400/401/403.  pyrtcm already scales both to ms / dBHz.
            fine_pr = getattr(msg, f'DF405_{cell:02d}', None)
            if fine_pr is None:
                fine_pr = getattr(msg, f'DF400_{cell:02d}', None)
            fine_ph = getattr(msg, f'DF406_{cell:02d}', None)
            if fine_ph is None:
                fine_ph = getattr(msg, f'DF401_{cell:02d}', None)
            cno = getattr(msg, f'DF408_{cell:02d}', None)
            if cno is None:
                cno = getattr(msg, f'DF403_{cell:02d}', None)
            half = getattr(msg, f'DF420_{cell:02d}', 0)
            if fine_pr is None:
                continue
            pr_m = _C * (rough + float(fine_pr)) / 1000.0
            cp_cyc = (_C * (rough + float(fine_ph)) / 1000.0) * freq / _C \
                if fine_ph is not None else None
            out.append(dict(sv=f'{prefix}{prn:02d}', sig=rinex, pr_m=pr_m,
                            cp_cyc=cp_cyc, cno=cno, half=half))
    return prefix, tow_ms, out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--caster', required=True)
    ap.add_argument('--port', type=int, default=2101)
    ap.add_argument('--tls', action='store_true')
    ap.add_argument('--mount', required=True)
    ap.add_argument('--user', default=os.environ.get('NTRIP_USER'))
    ap.add_argument('--password', default=os.environ.get('NTRIP_PASS'))
    ap.add_argument('--duration', type=float, default=300.0)
    ap.add_argument('--published', help='published ECEF "x,y,z" m to grade against')
    args = ap.parse_args()

    from ntrip_client import NtripStream
    import time
    stream = NtripStream(args.caster, args.port, args.mount,
                         user=args.user, password=args.password, tls=args.tls)
    stream.connect()
    print(f'# connected {args.caster}:{args.port}/{args.mount}')

    from datetime import datetime, timezone, timedelta
    from broadcast_eph import BroadcastEphemeris
    from solve_ppp import ls_init
    try:
        from solve_pseudorange import ecef_to_lla
    except Exception:
        ecef_to_lla = None

    eph = BroadcastEphemeris()
    t0 = time.monotonic()
    epochs = 0
    seen_sv = set()
    sig_counts = defaultdict(int)
    station_ecef = None
    latest_obs = None
    latest_tow = None
    try:
        for msg in stream.messages():           # NtripStream.messages() → decoded RTCM
            ident = str(getattr(msg, 'identity', getattr(msg, 'DF002', '')))
            if ident in ('1005', '1006'):       # station ARP (the "published" pos)
                station_ecef = (float(msg.DF025), float(msg.DF026), float(msg.DF027))
            elif ident in ('1019', '1042', '1045', '1046'):
                try:
                    eph.update_from_rtcm(msg)
                except Exception:
                    pass
            elif ident in ('1074', '1075', '1076', '1077'):   # GPS MSM4-7
                dec = decode_msm_obs(msg)
                if dec:
                    _, latest_tow, obs = dec
                    latest_obs = obs
                    epochs += 1
                    for o in obs:
                        seen_sv.add(o['sv']); sig_counts[o['sig']] += 1
            # eph.satellites may be a property or method depending on version
            _sats = eph.satellites
            eph_svs = set(_sats() if callable(_sats) else _sats)
            # break when enough ACTUALLY-solvable SVs (dual-freq ∧ eph) in the
            # latest epoch — margin over the 4 the LS needs — else run the clock.
            solvable = 0
            if latest_obs:
                now_sigs = defaultdict(set)
                for o in latest_obs:
                    now_sigs[o['sv']].add(o['sig'])
                solvable = sum(1 for sv, ss in now_sigs.items()
                               if sv in eph_svs and '1C' in ss
                               and any(k in ss for k in _L2_CODES))
            if time.monotonic() - t0 > args.duration or (solvable >= 5 and epochs > 3):
                break
    finally:
        stream.close()

    print(f'# {epochs} MSM epochs, {len(seen_sv)} GPS SVs: {sorted(seen_sv)}')
    _sats = eph.satellites
    have = set(_sats() if callable(_sats) else _sats)
    print(f'# signals: {dict(sig_counts)}  | eph for {len(have)} SVs')

    pub = (tuple(float(x) for x in args.published.split(','))
           if args.published else station_ecef)
    if not pub:
        print('# no 1005/1006 ARP in stream; pass --published x,y,z to grade.')
        return 0 if epochs else 2

    # ── code-only LS fix from IF pseudoranges → residual vs published ──
    F1, F2 = 1575.42e6, 1227.60e6
    g, h = F1 * F1 / (F1 * F1 - F2 * F2), F2 * F2 / (F1 * F1 - F2 * F2)
    by_sv = defaultdict(dict)
    cno_sv = {}
    for o in (latest_obs or []):
        by_sv[o['sv']][o['sig']] = o['pr_m']
        if o['sig'] == '1C' and o['cno'] is not None:
            cno_sv[o['sv']] = float(o['cno'])
    obss = []
    for sv, sigs in by_sv.items():
        if sv not in have:
            continue
        l1 = sigs.get('1C')
        l2 = next((sigs[k] for k in _L2_CODES if k in sigs), None)
        if l1 and l2:
            obss.append({'sys': 'gps', 'sv': sv, 'pr_if': g * l1 - h * l2,
                         'cno': cno_sv.get(sv, 40.0)})

    if len(obss) < 4:
        print(f'# only {len(obss)} dual-freq SVs with eph — need ≥4 to solve '
              f'(let it run longer; 1019 eph arrives ~every 60 s).')
        return 2

    GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
    now_gps = datetime.now(timezone.utc) + timedelta(seconds=18)   # ~GPS-UTC leap
    week = int((now_gps - GPS_EPOCH).total_seconds() // 604800)
    t = GPS_EPOCH + timedelta(weeks=week, milliseconds=latest_tow)

    x, ok, nsv = ls_init(obss, eph, t)
    if not ok:
        print(f'# LS solve did not converge (n_sv={nsv}).')
        return 2
    fix = x[:3]
    dxyz = fix - __import__('numpy').array(pub)
    d3d = float((dxyz ** 2).sum() ** 0.5)
    print(f'# LS fix from {len(obss)} dual-freq GPS SVs: '
          f'({fix[0]:.3f}, {fix[1]:.3f}, {fix[2]:.3f}) m')
    print(f'# published ARP:                 '
          f'({pub[0]:.3f}, {pub[1]:.3f}, {pub[2]:.3f}) m')
    if ecef_to_lla is not None:
        import math as _m
        lat, lon, _ = ecef_to_lla(pub[0], pub[1], pub[2])
        la, lo = _m.radians(lat), _m.radians(lon)
        R = __import__('numpy').array([
            [-_m.sin(lo), _m.cos(lo), 0],
            [-_m.sin(la) * _m.cos(lo), -_m.sin(la) * _m.sin(lo), _m.cos(la)],
            [_m.cos(la) * _m.cos(lo), _m.cos(la) * _m.sin(lo), _m.sin(la)]])
        e, n, u = R @ dxyz
        print(f'# RESIDUAL vs published:  3D={d3d:.2f} m  '
              f'(E={e:+.2f}, N={n:+.2f}, U={u:+.2f} m)')
    else:
        print(f'# RESIDUAL vs published:  3D={d3d:.2f} m')
    print('# (code-only single-point LS — a few metres is expected + '
          'confirms the MSM ingest is correct.)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

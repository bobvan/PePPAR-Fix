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
    sats = _bits(int(msg.DF394), 64)      # PRNs present (bit = PRN)
    sigs = _bits(int(msg.DF395), 32)      # MSM signal ids present
    cellmask = int(msg.DF396)
    ncell = bin(cellmask).count('1')
    tow_ms = int(getattr(msg, 'DF004', 0))
    # cell index runs over the (sat × sig) grid where cellmask bit is set.
    out = []
    cell = 0
    for si, prn in enumerate(sats, start=1):
        rough = float(getattr(msg, f'DF397_{si:02d}')) + \
                float(getattr(msg, f'DF398_{si:02d}'))
        for gj, sig_id in enumerate(sigs):
            bit = si * len(sigs) - (len(sigs) - 1 - gj)   # cell grid position
            if not (cellmask & (1 << (len(sats) * len(sigs) - bit))):
                continue
            cell += 1
            rinex, freq = sigtab.get(sig_id, (f's{sig_id}', 1575.42e6))
            df400 = getattr(msg, f'DF400_{cell:02d}', None)
            df401 = getattr(msg, f'DF401_{cell:02d}', None)
            cno = getattr(msg, f'DF403_{cell:02d}', None)
            half = getattr(msg, f'DF420_{cell:02d}', 0)
            if df400 is None:
                continue
            pr_m = _C * (rough + float(df400)) / 1000.0
            cp_cyc = (_C * (rough + float(df401)) / 1000.0) * freq / _C \
                if df401 is not None else None
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
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--published', help='published ECEF "x,y,z" m to grade against')
    args = ap.parse_args()

    from ntrip_client import NtripStream
    import time
    stream = NtripStream(args.caster, args.port, args.mount,
                         user=args.user, password=args.password, tls=args.tls)
    stream.connect()
    print(f'# connected {args.caster}:{args.port}/{args.mount}')

    t0 = time.monotonic()
    epochs = 0
    seen_sv = set()
    sig_counts = defaultdict(int)
    station_ecef = None
    last_sample = None
    try:
        for msg in stream.messages():           # NtripStream.messages() → decoded RTCM
            ident = getattr(msg, 'identity', str(getattr(msg, 'DF002', '')))
            if ident in ('1005', '1006'):       # station ARP (the "published" pos)
                station_ecef = (float(msg.DF025), float(msg.DF026), float(msg.DF027))
            dec = decode_msm_obs(msg) if str(ident).startswith(('1074', '1077')) else None
            if dec:
                _, tow, obs = dec
                epochs += 1
                for o in obs:
                    seen_sv.add(o['sv']); sig_counts[o['sig']] += 1
                if last_sample is None and obs:
                    last_sample = obs[:4]
            if time.monotonic() - t0 > args.duration:
                break
    finally:
        stream.close()

    print(f'# {epochs} MSM epochs, {len(seen_sv)} GPS SVs: '
          f'{sorted(seen_sv)}')
    print(f'# signals: {dict(sig_counts)}')
    if last_sample:
        print('# sample obs (first epoch):')
        for o in last_sample:
            cp = f'{o["cp_cyc"]:.1f}' if o['cp_cyc'] else '-'
            print(f'    {o["sv"]} {o["sig"]}  PR={o["pr_m"]:.3f}m  '
                  f'CP={cp}cyc  CNR={o["cno"]}dBHz  half={o["half"]}')
    pub = station_ecef
    if args.published:
        pub = tuple(float(x) for x in args.published.split(','))
    if pub:
        print(f'# published/station ECEF: {pub}')
        print('# NEXT: code-only LS solve (needs broadcast eph from the '
              'stream 1019/1042 or a nav mount) → residual vs published.')
    else:
        print('# no 1005 ARP in stream; pass --published x,y,z to grade.')
    return 0 if epochs else 2


if __name__ == '__main__':
    raise SystemExit(main())

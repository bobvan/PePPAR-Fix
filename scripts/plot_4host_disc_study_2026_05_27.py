"""4-host discipline-study plots: disciplined chA TDEV/ADEV vs freerun baseline,
plus DAC actuator activity over time ("light touch" view).

Data: data/disc-study/day0527-disc2-<host>-ticc.csv (chA + chB samples) and
data/disc-study/day0527-disc2-<host>-arm-state.csv (DAC adj in x3_f_do_ppb).
Freerun baseline: data/freerun-day0527-2hb-comparison/<host>.json
(chA-vs-Rb characterization from morning runs) + the TimeHat ticc_read.py
output.  madhat is intentionally absent — its engine couldn't bootstrap
(see dayplan madhatBootstrapStuckPostArm).

Outputs:
  docs/4host-disc-vs-freerun-tdev-2026-05-27.png — TDEV overlay
  docs/4host-disc-dac-activity-2026-05-27.png    — DAC adj vs time
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / 'scripts'))

import allantools  # noqa: E402

D = _REPO / 'data' / 'disc-study'
FREERUN_DIR = _REPO / 'data' / 'freerun-day0527-2hb-comparison'
DOCS = _REPO / 'docs'

TAUS = (1, 2, 3, 5, 7, 10, 15, 20, 30, 50, 70, 100, 150, 200, 300, 500, 700, 1000, 1500, 2000, 3000)

HOSTS = [
    ('PiFace',  '#d62728'),
    ('clkPoC3', '#2ca02c'),
    ('TimeHat', '#1f77b4'),
]
SKIP_BOOTSTRAP_S = 180  # discard first 3 min while EKF converges


def _load_ticc(host_lc: str):
    """Return (chA_secs, chA_ps, chB_secs, chB_ps) as numpy int arrays.

    Each chA and chB stream is returned separately (no pairing).  chA = DO
    PPS measured against the TICC Rb; chB = GNSS PPS measured against the
    same Rb.  We analyze each as a single-channel TDEV vs the Rb reference,
    matching the freerun chA-vs-Rb methodology.
    """
    chA_s, chA_p, chB_s, chB_p = [], [], [], []
    path = D / f'day0527-disc2-{host_lc}-ticc.csv'
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                ch = row['channel']
                s = int(row['ref_sec'])
                p = int(row['ref_ps'])
            except (KeyError, ValueError):
                continue
            if ch == 'chA':
                chA_s.append(s); chA_p.append(p)
            elif ch == 'chB':
                chB_s.append(s); chB_p.append(p)
    return chA_s, chA_p, chB_s, chB_p


def _phase_ns_series(secs: list[int], pss: list[int],
                      skip_s: float = 0.0):
    """Return (t_rel_s, phase_ns) ndarrays.  Skips the first `skip_s`
    seconds of capture relative to the first sample.  Phase is the
    cumulative integer-ps from sample[0], converted to ns at the end
    (precision-preserving)."""
    if not secs:
        return np.array([]), np.array([])
    secs_arr = np.asarray(secs, dtype=float)
    t_rel = secs_arr - secs_arr[0]
    if skip_s > 0:
        keep = t_rel >= skip_s
        secs = [s for s, k in zip(secs, keep) if k]
        pss = [p for p, k in zip(pss, keep) if k]
        if not secs:
            return np.array([]), np.array([])
        secs_arr = np.asarray(secs, dtype=float)
        t_rel = secs_arr - secs_arr[0]
    ps0 = pss[0]
    phase_ns = np.asarray([p - ps0 for p in pss], dtype=np.int64).astype(float) * 1e-3
    return t_rel, phase_ns


def _allan_metrics(t_rel, phase_ns, taus):
    if len(phase_ns) < 60:
        return np.array([]), np.array([]), np.array([])
    slope, intercept = np.polyfit(t_rel, phase_ns, 1)
    residual = phase_ns - (slope * t_rel + intercept)
    valid = [t for t in taus if t < len(residual) / 1.0 / 4]
    tau_arr = np.array(valid, dtype=float)
    phase_s = residual * 1e-9
    t_a, adev, _, _ = allantools.adev(phase_s, rate=1.0, data_type='phase', taus=tau_arr)
    t_t, tdev, _, _ = allantools.tdev(phase_s, rate=1.0, data_type='phase', taus=tau_arr)
    return np.asarray(t_a), np.asarray(adev), np.asarray(tdev) * 1e9


def _freerun_tdev(host: str):
    """Return (taus, tdev_ns) ndarray from the morning freerun JSONs."""
    fname = host.lower() + '.json'
    path = FREERUN_DIR / fname
    if not path.exists():
        return np.array([]), np.array([])
    j = json.loads(path.read_text())
    c = j.get('characterization', {}) or {}
    if host == 'TimeHat':
        # TimeHat uses my earlier analyzer's schema
        m = c.get('tdev_ns', {}) or {}
        items = sorted((int(float(k)), float(v)) for k, v in m.items())
    else:
        src = c.get('sources', {}) or {}
        s = src.get('DO PPS (chA-chB)') or src.get('DO PPS (chA vs TICC Rb)')
        if not s:
            return np.array([]), np.array([])
        m = s.get('tdev_ns_by_tau_s', {}) or {}
        items = sorted((int(float(k)), float(v)) for k, v in m.items())
    if not items:
        return np.array([]), np.array([])
    taus = np.asarray([t for t, _ in items], dtype=float)
    tdev = np.asarray([v for _, v in items], dtype=float)
    return taus, tdev


def _load_dac_adj(host_lc: str):
    """Return (t_rel_s, dac_adj_ppb) from the arm-state log.

    Column x3_f_do_ppb is the engine's current DO frequency-state estimate
    in ppb.  The actuator command (DAC code -> applied ppb) is the
    negative of this in tracking, but for the "light touch" view the
    state's time-evolution is what matters — large oscillations = the
    engine is fighting the DO; quiet = settled.
    """
    path = D / f'day0527-disc2-{host_lc}-arm-state.csv'
    if not path.exists():
        return np.array([]), np.array([])
    secs, adj = [], []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                secs.append(float(row['host_monotonic']))
                adj.append(float(row['x3_f_do_ppb']))
            except (KeyError, ValueError):
                continue
    if not secs:
        return np.array([]), np.array([])
    secs = np.asarray(secs)
    adj = np.asarray(adj)
    return secs - secs[0], adj


def main():
    series = {}
    for host, color in HOSTS:
        lc = host.lower()
        chA_s, chA_p, chB_s, chB_p = _load_ticc(lc)
        t_A, phA = _phase_ns_series(chA_s, chA_p, skip_s=SKIP_BOOTSTRAP_S)
        t_B, phB = _phase_ns_series(chB_s, chB_p, skip_s=SKIP_BOOTSTRAP_S)
        tausA, adevA, tdevA = _allan_metrics(t_A, phA, TAUS)
        tausB, adevB, tdevB = _allan_metrics(t_B, phB, TAUS)
        fr_taus, fr_tdev = _freerun_tdev(host)
        dac_t, dac_adj = _load_dac_adj(lc)
        series[host] = dict(color=color,
                            tausA=tausA, tdevA=tdevA, adevA=adevA, nA=len(phA),
                            tausB=tausB, tdevB=tdevB,
                            fr_taus=fr_taus, fr_tdev=fr_tdev,
                            dac_t=dac_t, dac_adj=dac_adj)
        print(f"{host:<10} n_chA={len(phA):>5}  TDEV_chA(1s)={tdevA[0] if len(tdevA) else float('nan'):.3f} ns  "
              f"TDEV_chA(1000s)={tdevA[np.argmin(np.abs(tausA-1000))] if len(tausA) else float('nan'):.3f} ns")

    tau_ref = np.array([1, 1e4], dtype=float)

    # === TDEV plot ===
    fig, ax = plt.subplots(figsize=(11, 7))
    for host, s in series.items():
        if len(s['tausA']):
            ax.loglog(s['tausA'], s['tdevA'], '-o', color=s['color'],
                      markersize=5, linewidth=2, label=f"{host} chA disciplined")
        if len(s['fr_taus']):
            ax.loglog(s['fr_taus'], s['fr_tdev'], '--', color=s['color'],
                      alpha=0.55, linewidth=1.6,
                      label=f"{host} chA freerun (morning)")
        if len(s['tausB']):
            ax.loglog(s['tausB'], s['tdevB'], ':', color=s['color'],
                      alpha=0.45, linewidth=1.2,
                      label=f"{host} chB (GNSS PPS)")

    # TDCP floor (24 ps @ 1 s, τ^½)
    ax.loglog(tau_ref, 0.024 * np.sqrt(tau_ref), ':', color='#9467bd',
              linewidth=2, label='TDCP measurement floor (24 ps @ 1 s, τ^½)')
    # GPS PPS reference (F9T 2.3 ns @ 1 s, white-phase τ^(-½))
    ax.loglog(tau_ref, 2.3 / np.sqrt(tau_ref), '--', color='#7f7f7f',
              linewidth=1.5, label='GPS PPS (F9T, 2.3 ns @ 1 s, white-phase τ^(-½))')
    # Moonshot per-clock budget + shared-antenna excursion bound
    ax.axhline(0.354, color='#bcbd22', linewidth=1.2, alpha=0.6,
               label='moonshot per-clock budget (354 ps)')
    ax.axhline(1.0, color='#e377c2', linewidth=1.2, alpha=0.6,
               label='shared-antenna excursion bound (1 ns)')

    ax.set_xlabel('τ (s)')
    ax.set_ylabel('TDEV (ns)')
    ax.set_title(f'3-host disciplined vs freerun chA TDEV — 2026-05-27 '
                  f'(skip first {SKIP_BOOTSTRAP_S}s of disc capture)')
    ax.set_xlim(0.9, 5000)
    ax.set_ylim(1e-2, 5e2)
    ax.grid(True, which='both', alpha=0.3)
    ax.legend(fontsize=8, loc='upper left', framealpha=0.9, ncol=2)
    fig.tight_layout()
    out_t = DOCS / '3host-disc-vs-freerun-tdev-2026-05-27.png'
    fig.savefig(out_t, dpi=140)
    print(f"\nWrote → {out_t}")

    # === DAC activity (x3_f_do_ppb) over time ===
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for ax, (host, _color) in zip(axes, HOSTS):
        s = series[host]
        if not len(s['dac_t']):
            continue
        ax.plot(s['dac_t'], s['dac_adj'], linewidth=1.0, color=s['color'])
        ax.axhline(0, color='black', linewidth=0.4, alpha=0.3)
        ax.set_ylabel(f'{host}\nx3_f_do_ppb')
        ax.grid(True, alpha=0.3)
        # mark the SKIP_BOOTSTRAP_S boundary
        ax.axvline(SKIP_BOOTSTRAP_S, color='#888', linestyle='--', linewidth=0.8,
                   alpha=0.5)
    axes[0].set_title('DO frequency state x3_f_do_ppb over capture — '
                       '"light touch" view (dashed line = bootstrap-skip boundary)')
    axes[-1].set_xlabel('seconds since capture start')
    fig.tight_layout()
    out_d = DOCS / '3host-disc-dac-activity-2026-05-27.png'
    fig.savefig(out_d, dpi=140)
    print(f"Wrote → {out_d}")


if __name__ == '__main__':
    main()

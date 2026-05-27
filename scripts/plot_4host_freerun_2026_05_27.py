"""4-host freerun TDEV + ADEV plots with TDCP + GPS overlays.

Reads raw chA timestamps from each host's 2 h capture (DAC hosts: do_freerun_char
raw_csv with 'ref_sec,ref_ps' schema; TimeHat: ticc_read.py with 'host_time,
channel,ref_sec,ref_ps' schema, filter to chA).  Linearly detrends each, then
recomputes ADEV+TDEV at full precision via allantools (the JSON ADEV values
are corrupted by a round-to-6-decimals bug — see dayplan doFreerunCharJsonCleanup).

Overlays:
- TDCP measurement floor: 24 ps @ τ=1s, integrated white-FM → TDEV(τ) ∝ τ¹
  (from prior moonshot plot data/tdcp_moonshot_with_cha_20260524.png).
- GPS PPS reference: F9T PPS TDEV(1s) ≈ 2.3 ns per CLAUDE.md ticc-baseline-
  2026-04-01 — white-phase noise from receiver, TDEV ∝ τ^(-1/2),
  ADEV ∝ τ^(-1) ("GPS -1 slope" reference line).
- Moonshot per-clock budget: 354 ps TDEV horizontal line (from
  two-site-sync-budget.md, derived from the 1 ns shared-antenna excursion bound).
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / 'scripts'))

from do_freerun_char import _TAUS_S, detrend_linear  # noqa: E402

import allantools  # noqa: E402


D = _REPO / 'data'
TAUS = (1, 2, 3, 5, 7, 10, 15, 20, 30, 50, 70, 100, 150, 200, 300, 500, 700, 1000, 1500, 2000)


def load_dac_csv(path: Path):
    """do_freerun_char raw_csv: ref_sec, ref_ps  → (t_rel, phase_ns)."""
    secs, pss = [], []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            secs.append(int(row['ref_sec']))
            pss.append(int(row['ref_ps']))
    return _to_phase(secs, pss)


def load_timehat_csv(path: Path):
    """ticc_read.py CSV: host_time, channel, ref_sec, ref_ps  → filter chA."""
    secs, pss = [], []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            if row['channel'] != 'chA':
                continue
            secs.append(int(row['ref_sec']))
            pss.append(int(row['ref_ps']))
    return _to_phase(secs, pss)


def _to_phase(secs, pss):
    secs = np.asarray(secs, dtype=float)
    ps0 = pss[0]
    phase_ns = np.asarray(
        [p - ps0 for p in pss], dtype=np.int64
    ).astype(float) * 1e-3
    t_rel = secs - secs[0]
    return t_rel, phase_ns


def analyze(t_rel, phase_ns):
    residual, _, _ = detrend_linear(t_rel, phase_ns)
    n = len(residual)
    rate = 1.0
    valid = [t for t in TAUS if t < n / rate / 4]
    tau_arr = np.array(valid, dtype=float)
    # allantools wants phase in seconds; residual is ns.
    phase_s = residual * 1e-9
    t_a, adev, _, _ = allantools.adev(phase_s, rate=rate, data_type='phase', taus=tau_arr)
    t_t, tdev, _, _ = allantools.tdev(phase_s, rate=rate, data_type='phase', taus=tau_arr)
    return np.asarray(t_a), np.asarray(adev), np.asarray(tdev)


HOSTS = [
    ('PiFace (CTI OCXO 5V)',         'freerun-day0527-2hb-piface-raw.csv',   '#d62728', 'dac'),
    ('clkPoC3 (ocxo)',               'freerun-day0527-2hb-clkpoc3-raw.csv',  '#2ca02c', 'dac'),
    ('madhat (IsoTemp OCXO-33)',     'freerun-day0527-2hb-madhat-raw.csv',   '#ff7f0e', 'dac'),
    ('TimeHat (i226 TCXO)',          'freerun-day0527-2h-timehat-raw.csv',   '#1f77b4', 'timehat'),
]


def main():
    series = {}
    for label, fname, color, kind in HOSTS:
        path = D / fname
        if kind == 'dac':
            t_rel, phase_ns = load_dac_csv(path)
        else:
            t_rel, phase_ns = load_timehat_csv(path)
        taus, adev, tdev = analyze(t_rel, phase_ns)
        series[label] = dict(taus=taus, adev=adev, tdev_ns=tdev * 1e9,
                              color=color, n=len(t_rel))
        print(f"{label:<32}  n={len(t_rel):>5}  "
              f"TDEV(1s)={tdev[0]*1e9:6.3f} ns  "
              f"ADEV(1s)={adev[0]:8.2e}")

    tau_ref = np.array([1, 1e4], dtype=float)

    # === TDEV plot ===
    fig, ax = plt.subplots(figsize=(11, 7))
    for label, s in series.items():
        ax.loglog(s['taus'], s['tdev_ns'], marker='o', linewidth=2,
                  markersize=5, color=s['color'], label=label)

    # TDCP measurement floor: 24 ps @ 1s, integrated noise (white-FM) → TDEV ∝ τ½
    tdcp_anchor_ps = 24.0
    tdcp_line_ps = tdcp_anchor_ps * np.sqrt(tau_ref / 1.0)
    ax.loglog(tau_ref, tdcp_line_ps / 1e3, ':', color='#9467bd', linewidth=2.0,
              label='TDCP measurement floor (24 ps @ 1 s, τ^½)')

    # GPS PPS reference: F9T PPS TDEV(1s) = 2.3 ns, white phase → TDEV ∝ τ^(-1/2)
    gps_anchor_ns = 2.3
    gps_wphase_ns = gps_anchor_ns / np.sqrt(tau_ref / 1.0)
    ax.loglog(tau_ref, gps_wphase_ns, '--', color='#7f7f7f', linewidth=1.5,
              label='GPS PPS (F9T, 2.3 ns @ 1 s, white-phase τ^(-½))')

    # Pure -1 slope reference line ("GPS -1 slope" per user request)
    slope_m1_ns = gps_anchor_ns / (tau_ref / 1.0)
    ax.loglog(tau_ref, slope_m1_ns, '-.', color='#000000', linewidth=1.0, alpha=0.5,
              label='reference: slope −1 (anchored at 2.3 ns)')

    # Moonshot per-clock budget — 354 ps horizontal
    ax.axhline(0.354, color='#bcbd22', linewidth=1.2, alpha=0.7,
               label='moonshot per-clock budget (354 ps)')

    # Shared-antenna excursion bound — 1 ns horizontal
    ax.axhline(1.0, color='#e377c2', linewidth=1.2, alpha=0.7,
               label='shared-antenna excursion bound (1 ns)')

    ax.set_xlabel('τ (s)')
    ax.set_ylabel('TDEV (ns)')
    ax.set_title('4-host freerun TDEV — chA vs TICC Rb, 2 h captures, 2026-05-27')
    ax.set_xlim(0.9, 2500)
    ax.set_ylim(1e-2, 5e2)
    ax.grid(True, which='both', alpha=0.3)
    ax.legend(fontsize=8, loc='upper left', framealpha=0.9)
    fig.tight_layout()
    out_t = D / 'freerun-day0527-2h-4host-tdev.png'
    fig.savefig(out_t, dpi=140)
    print(f"\nWrote → {out_t}")

    # === ADEV plot ===
    fig, ax = plt.subplots(figsize=(11, 7))
    for label, s in series.items():
        ax.loglog(s['taus'], s['adev'], marker='o', linewidth=2,
                  markersize=5, color=s['color'], label=label)

    # TDCP floor in ADEV: from TDEV ≈ τ·MDEV/√3 ≈ τ·ADEV/√3 (loose for white-FM)
    # 24 ps @1s TDEV with τ^½ slope → ADEV ≈ 24ps/(1s·1/√3) = 41.6e-12 σ_y at τ=1
    # and ADEV ∝ τ^(-1/2) for white-FM after integration.
    tdcp_adev_1s = (tdcp_anchor_ps * 1e-12) * np.sqrt(3.0) / 1.0
    tdcp_adev_line = tdcp_adev_1s / np.sqrt(tau_ref / 1.0)
    ax.loglog(tau_ref, tdcp_adev_line, ':', color='#9467bd', linewidth=2.0,
              label=f'TDCP measurement floor ({tdcp_adev_1s:.1e} σ_y @ 1 s, white-FM τ^(-½))')

    # GPS PPS in ADEV: white phase → ADEV ∝ τ^(-1)
    gps_adev_1s = (gps_anchor_ns * 1e-9) * np.sqrt(3.0) / 1.0
    gps_adev_line = gps_adev_1s / (tau_ref / 1.0)
    ax.loglog(tau_ref, gps_adev_line, '--', color='#7f7f7f', linewidth=1.5,
              label=f'GPS PPS (F9T, {gps_adev_1s:.1e} σ_y @ 1 s, white-phase τ^(-1))')

    # Pure -1 slope on ADEV is the GPS line itself; mark it explicitly
    ax.loglog(tau_ref, 1e-8 / (tau_ref / 1.0), '-.', color='#000000', linewidth=1.0, alpha=0.5,
              label='reference: slope −1 (anchored at 1e-8 σ_y @ 1 s)')

    ax.set_xlabel('τ (s)')
    ax.set_ylabel('ADEV  σ_y(τ)')
    ax.set_title('4-host freerun ADEV — chA vs TICC Rb, 2 h captures, 2026-05-27')
    ax.set_xlim(0.9, 2500)
    ax.set_ylim(1e-13, 1e-7)
    ax.grid(True, which='both', alpha=0.3)
    ax.legend(fontsize=8, loc='upper right', framealpha=0.9)
    fig.tight_layout()
    out_a = D / 'freerun-day0527-2h-4host-adev.png'
    fig.savefig(out_a, dpi=140)
    print(f"Wrote → {out_a}")


if __name__ == '__main__':
    main()

"""3-host discipline-study plots: rebased-timestamp TDEV (full + per-window),
residual time series, and DAC actuator activity.

Rebasing convention (fixes the silent ref_ps-wrap bug from the first cut):

    elapsed_ps_int = (s - s0) * 1_000_000_000_000 + (p - p0)
    elapsed_ns     = elapsed_ps_int.astype(float) * 1e-3

The integer subtraction stays exact (int64 holds 9e18; we cap at ~10^16
ps over the 2.5 h capture).  The *1e-3 conversion lands inside float64's
2^53-precision window (~9e15) so ps-resolution is preserved.

The same formula is applied to the morning freerun raw CSVs so the
comparison is apples-to-apples.  madhat is intentionally absent — see
dayplan madhatBootstrapStuckPostArm.

Outputs (under docs/):
  3host-disc-vs-freerun-tdev-2026-05-27.png  (TDEV overlay)
  3host-disc-per-window-tdev-2026-05-27.png  (per-window TDEV → event vs continuous)
  3host-disc-residual-timeseries-2026-05-27.png  (where the events are)
  3host-disc-dac-activity-2026-05-27.png   (unchanged from first cut)
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / 'scripts'))

import allantools  # noqa: E402

D = _REPO / 'data' / 'disc-study'
FREERUN_DATA = _REPO / 'data'
DOCS = _REPO / 'docs'

TAUS = (1, 2, 3, 5, 7, 10, 15, 20, 30, 50, 70, 100, 150, 200, 300, 500, 700, 1000, 1500, 2000, 3000)

HOSTS = [
    ('PiFace',  '#d62728'),
    ('clkPoC3', '#2ca02c'),
    ('TimeHat', '#1f77b4'),
]
SKIP_BOOTSTRAP_S = 180


def _rebased_elapsed_ns(secs: list[int], pss: list[int]):
    """Convert raw TICC integer (sec, ps) pairs to elapsed-ns float64,
    rebased to first sample.  Same formula used everywhere so freerun
    and disc are apples-to-apples.

    Returns (t_rel_s, elapsed_ns) ndarrays.
    """
    s0, p0 = secs[0], pss[0]
    elapsed_ps = np.asarray(
        [(s - s0) * 1_000_000_000_000 + (p - p0) for s, p in zip(secs, pss)],
        dtype=np.int64)
    elapsed_ns = elapsed_ps.astype(float) * 1e-3
    t_rel = np.asarray([s - s0 for s in secs], dtype=float)
    return t_rel, elapsed_ns


def _load_disc_ticc(host_lc: str, channel: str = 'chA'):
    """Read disc-study ticc CSV; return (secs, pss) for one channel."""
    secs, pss = [], []
    with open(D / f'day0527-disc2-{host_lc}-ticc.csv') as f:
        r = csv.DictReader(f)
        for row in r:
            if row.get('channel') != channel:
                continue
            try:
                secs.append(int(row['ref_sec']))
                pss.append(int(row['ref_ps']))
            except (KeyError, ValueError):
                continue
    return secs, pss


def _load_freerun_ticc(host_lc: str):
    """Read morning freerun raw CSV; return chA (secs, pss).  DAC-host
    schema is `ref_sec,ref_ps`; TimeHat schema is
    `host_time,channel,ref_sec,ref_ps` (filter chA)."""
    secs, pss = [], []
    if host_lc == 'timehat':
        path = FREERUN_DATA / 'freerun-day0527-2h-timehat-raw.csv'
        with open(path) as f:
            r = csv.DictReader(f)
            for row in r:
                if row.get('channel') != 'chA':
                    continue
                try:
                    secs.append(int(row['ref_sec']))
                    pss.append(int(row['ref_ps']))
                except (KeyError, ValueError):
                    continue
    else:
        path = FREERUN_DATA / f'freerun-day0527-2hb-{host_lc}-raw.csv'
        with open(path) as f:
            r = csv.DictReader(f)
            for row in r:
                try:
                    secs.append(int(row['ref_sec']))
                    pss.append(int(row['ref_ps']))
                except (KeyError, ValueError):
                    continue
    return secs, pss


def _detrend_and_tdev(t_rel, elapsed_ns, taus):
    if len(elapsed_ns) < 60:
        return np.array([]), np.array([]), np.array([])
    slope, intercept = np.polyfit(t_rel, elapsed_ns, 1)
    residual = elapsed_ns - (slope * t_rel + intercept)
    valid = [t for t in taus if t < len(residual) / 4]
    tau_arr = np.array(valid, dtype=float)
    phase_s = residual * 1e-9
    t_t, tdev, _, _ = allantools.tdev(phase_s, rate=1.0, data_type='phase', taus=tau_arr)
    return np.asarray(t_t), np.asarray(tdev) * 1e9, residual


def _skip(t_rel, ns, skip_s):
    keep = t_rel >= skip_s
    if not keep.any():
        return t_rel[:0], ns[:0]
    return t_rel[keep] - skip_s, ns[keep]


def _load_dac_adj(host_lc: str):
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
        # Disc chA (skip bootstrap)
        d_secs, d_pss = _load_disc_ticc(lc, 'chA')
        d_t, d_ns = _rebased_elapsed_ns(d_secs, d_pss)
        d_t, d_ns = _skip(d_t, d_ns, SKIP_BOOTSTRAP_S)
        d_taus, d_tdev, d_resid = _detrend_and_tdev(d_t, d_ns, TAUS)

        # Freerun chA (rebased, same formula)
        f_secs, f_pss = _load_freerun_ticc(lc)
        f_t, f_ns = _rebased_elapsed_ns(f_secs, f_pss)
        f_taus, f_tdev, f_resid = _detrend_and_tdev(f_t, f_ns, TAUS)

        # DAC adj
        dac_t, dac_adj = _load_dac_adj(lc)

        series[host] = dict(
            color=color,
            d_t=d_t, d_resid=d_resid, d_taus=d_taus, d_tdev=d_tdev,
            f_taus=f_taus, f_tdev=f_tdev,
            dac_t=dac_t, dac_adj=dac_adj,
        )
        td1_disc = d_tdev[0] if len(d_tdev) else float('nan')
        td1_free = f_tdev[0] if len(f_tdev) else float('nan')
        print(f"{host:<10}  disc n={len(d_ns):>5}  freerun n={len(f_ns):>5}  "
              f"TDEV(1s) disc={td1_disc:>7.3f} ns  freerun={td1_free:>7.3f} ns  "
              f"ratio={td1_disc/td1_free if td1_free else float('nan'):>5.1f}×")

    tau_ref = np.array([1, 1e4], dtype=float)

    # === Plot 1: TDEV comparison (rebased) ===
    fig, ax = plt.subplots(figsize=(11, 7))
    for host, s in series.items():
        if len(s['d_taus']):
            ax.loglog(s['d_taus'], s['d_tdev'], '-o', color=s['color'],
                      markersize=5, linewidth=2, label=f"{host} disciplined")
        if len(s['f_taus']):
            ax.loglog(s['f_taus'], s['f_tdev'], '--', color=s['color'],
                      alpha=0.55, linewidth=1.6, label=f"{host} freerun")
    ax.loglog(tau_ref, 0.024 * np.sqrt(tau_ref), ':', color='#9467bd',
              linewidth=2, label='TDCP measurement floor (24 ps @ 1 s, τ^½)')
    ax.loglog(tau_ref, 2.3 / np.sqrt(tau_ref), '--', color='#7f7f7f',
              linewidth=1.5, label='GPS PPS reference (F9T, 2.3 ns @ 1 s, τ^(-½))')
    ax.axhline(0.354, color='#bcbd22', linewidth=1.2, alpha=0.6,
               label='moonshot per-clock budget (354 ps)')
    ax.axhline(1.0, color='#e377c2', linewidth=1.2, alpha=0.6,
               label='shared-antenna excursion bound (1 ns)')
    ax.set_xlabel('τ (s)')
    ax.set_ylabel('TDEV (ns)')
    ax.set_title(f'3-host disciplined vs freerun chA TDEV — '
                  f'REBASED math, skip first {SKIP_BOOTSTRAP_S}s of disc (2026-05-27)')
    ax.set_xlim(0.9, 5000)
    ax.set_ylim(1e-2, 5e2)
    ax.grid(True, which='both', alpha=0.3)
    ax.legend(fontsize=8, loc='upper left', framealpha=0.9, ncol=2)
    fig.tight_layout()
    out = DOCS / '3host-disc-vs-freerun-tdev-2026-05-27.png'
    fig.savefig(out, dpi=140)
    print(f"\nWrote → {out}")

    # === Plot 2: TDEV per ~2500-s window (event-driven vs continuous) ===
    windows = [('early', 180, 180 + 2500),
               ('mid',   3000, 3000 + 2500),
               ('late',  6000, 6000 + 2500)]
    fig, axes = plt.subplots(1, 3, figsize=(15, 6), sharey=True)
    for ax, (host, s) in zip(axes, series.items()):
        # rebuild full t,ns for the host (we have d_t in skip-rebased frame)
        d_secs, d_pss = _load_disc_ticc(host.lower(), 'chA')
        t_full, ns_full = _rebased_elapsed_ns(d_secs, d_pss)
        for name, t_lo, t_hi, style in (
                ('early', windows[0][1], windows[0][2], '-o'),
                ('mid',   windows[1][1], windows[1][2], '-s'),
                ('late',  windows[2][1], windows[2][2], '-^')):
            mask = (t_full >= t_lo) & (t_full < t_hi)
            t_sub = t_full[mask] - t_lo
            ns_sub = ns_full[mask]
            if len(ns_sub) < 200:
                continue
            taus, tdev, _ = _detrend_and_tdev(t_sub, ns_sub, TAUS)
            ax.loglog(taus, tdev, style, markersize=4, linewidth=1.4,
                      label=f'{name} ({t_lo}–{t_hi} s)')
        if len(s['f_taus']):
            ax.loglog(s['f_taus'], s['f_tdev'], '--', color='black',
                      alpha=0.4, linewidth=1.2, label='freerun (morning)')
        ax.axhline(0.354, color='#bcbd22', linewidth=0.8, alpha=0.5)
        ax.set_title(f'{host}')
        ax.set_xlabel('τ (s)')
        if ax is axes[0]:
            ax.set_ylabel('TDEV (ns)')
        ax.grid(True, which='both', alpha=0.3)
        ax.set_xlim(0.9, 1500)
        ax.set_ylim(1e-2, 1e3)
        ax.legend(fontsize=8, loc='upper left')
    fig.suptitle('Per-window TDEV — event-driven noise reveals itself when one '
                  'window deviates from the others')
    fig.tight_layout()
    out = DOCS / '3host-disc-per-window-tdev-2026-05-27.png'
    fig.savefig(out, dpi=140)
    print(f"Wrote → {out}")

    # === Plot 3: chA residual time series (where the events are) ===
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for ax, (host, _color) in zip(axes, HOSTS):
        s = series[host]
        if not len(s['d_resid']):
            continue
        ax.plot(s['d_t'], s['d_resid'], linewidth=0.7, color=s['color'])
        ax.axhline(0, color='black', linewidth=0.4, alpha=0.3)
        ax.set_ylabel(f'{host}\nchA residual (ns)')
        ax.grid(True, alpha=0.3)
        # Annotate per-window TDEV(1s) at top
        rms = np.std(s['d_resid'])
        ax.text(0.01, 0.96, f'RMS={rms:.1f} ns  peak={np.max(np.abs(s["d_resid"])):.1f} ns',
                transform=ax.transAxes, ha='left', va='top', fontsize=9,
                bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))
    axes[0].set_title('chA residual (linear-detrended) time series — '
                       'where the events live (post-bootstrap-skip frame)')
    axes[-1].set_xlabel('seconds since bootstrap-skip')
    fig.tight_layout()
    out = DOCS / '3host-disc-residual-timeseries-2026-05-27.png'
    fig.savefig(out, dpi=140)
    print(f"Wrote → {out}")

    # === Plot 4: DAC activity (unchanged) ===
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for ax, (host, _color) in zip(axes, HOSTS):
        s = series[host]
        if not len(s['dac_t']):
            continue
        ax.plot(s['dac_t'], s['dac_adj'], linewidth=1.0, color=s['color'])
        ax.axhline(0, color='black', linewidth=0.4, alpha=0.3)
        ax.set_ylabel(f'{host}\nx3_f_do_ppb')
        ax.grid(True, alpha=0.3)
        ax.axvline(SKIP_BOOTSTRAP_S, color='#888', linestyle='--', linewidth=0.8,
                   alpha=0.5)
    axes[0].set_title('DO frequency state x3_f_do_ppb over capture — '
                       '"light touch" view (dashed = bootstrap-skip boundary)')
    axes[-1].set_xlabel('seconds since capture start')
    fig.tight_layout()
    out = DOCS / '3host-disc-dac-activity-2026-05-27.png'
    fig.savefig(out, dpi=140)
    print(f"Wrote → {out}")


if __name__ == '__main__':
    main()

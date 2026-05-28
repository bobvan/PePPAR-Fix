"""Compare disciplined chA TDEV: gated (day0528) vs ungated (day0527-disc2).

The OCXO-trusted gate ran overnight on PiFace + clkPoC3.  This compares
the disciplined output stability (TICC chA, detrended) against the
ungated baseline from the night before, and reports the LIVE gate
rejection rate from the arm-state-log's ocxo_gate_reject column.
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / 'scripts'))
import allantools  # noqa: E402

D = _REPO / 'data' / 'disc-study'
DOCS = _REPO / 'docs'

TAUS = (1, 2, 3, 5, 7, 10, 15, 20, 30, 50, 70, 100, 150, 200, 300,
        500, 700, 1000, 1500, 2000, 3000, 5000)
SKIP_BOOTSTRAP_S = 300

HOSTS = [('piface', 'PiFace', '#d62728'),
         ('clkpoc3', 'clkPoC3', '#2ca02c')]


def load_cha(path):
    secs, pss = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            if row['channel'] == 'chA':
                secs.append(int(row['ref_sec']))
                pss.append(int(row['ref_ps']))
    if not secs:
        return None, None
    s0, p0 = secs[0], pss[0]
    elapsed_ps = np.asarray(
        [(s - s0) * 1_000_000_000_000 + (p - p0) for s, p in zip(secs, pss)],
        dtype=np.int64)
    elapsed_ns = elapsed_ps.astype(np.float64) * 1e-3
    t_rel = np.asarray([s - s0 for s in secs], dtype=np.float64)
    return t_rel, elapsed_ns


def detrend_tdev(t_rel, elapsed_ns):
    if t_rel is None or len(elapsed_ns) < 60:
        return np.array([]), np.array([])
    mask = t_rel > SKIP_BOOTSTRAP_S
    t, e = t_rel[mask], elapsed_ns[mask]
    if len(e) < 60:
        return np.array([]), np.array([])
    t = t - t[0]
    slope, intercept = np.polyfit(t, e, 1)
    resid = e - (slope * t + intercept)
    valid = [x for x in TAUS if x < len(resid) / 4]
    if not valid:
        return np.array([]), np.array([])
    to, td, _, _ = allantools.tdev(resid * 1e-9, rate=1.0,
                                   data_type='phase', taus=valid)
    return np.asarray(to), np.asarray(td) * 1e9


def gate_stats(arm_csv):
    """Read live ocxo_gate_reject column.  Returns (n_ticc, n_rej)."""
    n_ticc = n_rej = 0
    with open(arm_csv) as f:
        for row in csv.DictReader(f):
            if row.get('arm_ticc_used') == '1':
                n_ticc += 1
                if row.get('ocxo_gate_reject') == '1':
                    n_rej += 1
    return n_ticc, n_rej


def tdev_at(taus, tdev, target):
    if len(taus) == 0:
        return None
    i = int(np.argmin(np.abs(taus - target)))
    return tdev[i] if abs(taus[i] - target) < target * 0.5 else None


def main():
    fig, ax = plt.subplots(figsize=(9, 7))
    print(f"{'Host':<9} {'TDEV(1s) ungated':<18} {'TDEV(1s) gated':<16} "
          f"{'gate rej rate':<14}")
    print('-' * 60)
    for key, label, color in HOSTS:
        ung = D / f'day0527-disc2-{key}-ticc.csv'
        gat = D / f'day0528-gated-{key}-ticc.csv'
        tu, eu = load_cha(ung)
        tg, eg = load_cha(gat)
        taus_u, tdev_u = detrend_tdev(tu, eu)
        taus_g, tdev_g = detrend_tdev(tg, eg)
        if len(taus_u):
            ax.loglog(taus_u, tdev_u, color=color, ls=':', lw=1.6,
                      label=f'{label} ungated (day0527)')
        if len(taus_g):
            ax.loglog(taus_g, tdev_g, color=color, ls='-', lw=2.2,
                      label=f'{label} gated (day0528)')
        n_ticc, n_rej = gate_stats(D / f'day0528-gated-{key}-arm-state.csv')
        rate = n_rej / n_ticc if n_ticc else 0
        u1 = tdev_at(taus_u, tdev_u, 1)
        g1 = tdev_at(taus_g, tdev_g, 1)
        print(f"{label:<9} "
              f"{f'{u1:.3f} ns' if u1 else 'N/A':<18} "
              f"{f'{g1:.3f} ns' if g1 else 'N/A':<16} "
              f"{f'{n_rej}/{n_ticc} = {rate:.1%}':<14}")
    ax.set_xlabel('τ (s)')
    ax.set_ylabel('TDEV (ns)')
    ax.set_title('Disciplined chA TDEV: OCXO gate ON (day0528) vs OFF (day0527)')
    ax.legend(fontsize=9)
    ax.grid(True, which='both', alpha=0.3)
    out = DOCS / 'ocxo-gate-overnight-tdev-2026-05-28.png'
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f'\nSaved: {out}')
    plt.close(fig)


if __name__ == '__main__':
    main()

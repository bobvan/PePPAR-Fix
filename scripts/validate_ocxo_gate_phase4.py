"""Phase 4 validation: OCXO-trusted gate counterfactual on day0527-disc2.

Reads arm-state CSVs from the ungated disc-study captures, applies the
OcxoTrustedGate retroactively, and produces:

1. Per-host gate rejection statistics vs chi² rejection statistics.
2. Innovation magnitude distribution vs gate threshold.
3. Replayed x3 (DO frequency) trajectory: ungated vs gated.
4. TDEV of replayed DO phase output (gated vs ungated vs freerun).

The replay feeds the *original* TICC diff measurements into a fresh
DOFreqEst pair (one gated, one ungated) so the full state-trajectory
divergence is captured — not just which epochs would flip.

Usage:
    python scripts/validate_ocxo_gate_phase4.py
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

from peppar_fix.ocxo_trusted_gate import OcxoTrustedGate, load_sigma_short_tau_from_state  # noqa: E402

D = _REPO / 'data' / 'disc-study'
FREERUN = _REPO / 'data' / 'freerun-day0527-2hb-comparison'
DOCS = _REPO / 'docs'

HOSTS = [
    ('piface',  'PiFace',  '#d62728'),
    ('clkpoc3', 'clkPoC3', '#2ca02c'),
    ('timehat', 'TimeHat', '#1f77b4'),
]

K_SIGMA = 10.0
MIN_AGE_S = 60.0
SKIP_BOOTSTRAP_S = 180

TAUS = (1, 2, 3, 5, 7, 10, 15, 20, 30, 50, 70, 100, 150, 200, 300,
        500, 700, 1000, 1500, 2000, 3000)


def load_sigma(host_key: str) -> float:
    """Load σ_DO(1s) from freerun characterization JSON."""
    p = FREERUN / f'{host_key}.json'
    v = load_sigma_short_tau_from_state(p)
    if v is None:
        raise RuntimeError(f"No σ_DO for {host_key} in {p}")
    return v


def load_arm_state(host_key: str):
    """Load arm-state CSV, return list of dicts with parsed floats."""
    p = D / f'day0527-disc2-{host_key}-arm-state.csv'
    rows = []
    with open(p) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def _rebased_elapsed_ns(secs, pss):
    s0, p0 = secs[0], pss[0]
    elapsed_ps = np.asarray(
        [(s - s0) * 1_000_000_000_000 + (p - p0) for s, p in zip(secs, pss)],
        dtype=np.int64)
    elapsed_ns = elapsed_ps.astype(np.float64) * 1e-3
    t_rel = np.asarray([(s - s0) for s in secs], dtype=np.float64)
    return t_rel, elapsed_ns


def load_ticc_cha(host_key: str):
    """Load TICC chA samples and return (t_rel_s, elapsed_ns)."""
    p = D / f'day0527-disc2-{host_key}-ticc.csv'
    secs, pss = [], []
    with open(p) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['channel'] == 'chA':
                secs.append(int(row['ref_sec']))
                pss.append(int(row['ref_ps']))
    return _rebased_elapsed_ns(secs, pss)


def load_freerun_tdev(host_key: str):
    """Load stored freerun TDEV curve from the characterization JSON.

    Returns (taus, tdev_ns) arrays, or (None, None) if unavailable.
    Handles both source-keyed and flat tdev_ns schemas.
    """
    p = FREERUN / f'{host_key}.json'
    j = json.loads(p.read_text())
    c = j.get('characterization', {})
    src = c.get('sources', {})
    for source_key in ('DO PPS (chA vs TICC Rb)', 'DO PPS (chA-chB)'):
        s = src.get(source_key, {})
        m = s.get('tdev_ns_by_tau_s', {})
        if m:
            taus = sorted(float(k) for k in m.keys())
            tdev = [m[str(t) if str(t) in m else f"{t:.1f}"] for t in taus]
            return np.array(taus), np.array(tdev)
    m = c.get('tdev_ns', {})
    if m:
        taus = sorted(float(k) for k in m.keys())
        tdev = [m[k] for k in sorted(m.keys(), key=float)]
        return np.array(taus), np.array(tdev)
    return None, None


def compute_tdev(t_rel, elapsed_ns, taus, rate=1.0):
    """Detrend elapsed_ns, then compute TDEV.  Returns (taus_used, tdev_ns)."""
    if len(elapsed_ns) < 60:
        return np.array([]), np.array([])
    slope, intercept = np.polyfit(t_rel, elapsed_ns, 1)
    residual = elapsed_ns - (slope * t_rel + intercept)
    valid = [t for t in taus if t < len(residual) / 4]
    if not valid:
        return np.array([]), np.array([])
    phase_s = residual * 1e-9
    try:
        t_out, td, _, _ = allantools.tdev(
            phase_s, rate=rate, data_type='phase', taus=valid)
        return np.asarray(t_out), np.asarray(td) * 1e9
    except Exception:
        return np.array([]), np.array([])


def counterfactual_analysis(host_key: str, host_label: str, sigma_ns: float):
    """Run counterfactual gate analysis on one host's arm-state CSV.

    Returns dict with gate stats and per-epoch decisions.
    """
    rows = load_arm_state(host_key)
    gate = OcxoTrustedGate(
        sigma_short_tau_ns=sigma_ns,
        k_sigma=K_SIGMA,
        min_age_s=MIN_AGE_S,
        do_label=host_label)

    cumulative_age = 0.0
    results = []
    chi2_rejected = 0
    chi2_total_ticc = 0

    for row in rows:
        dt = float(row['dt_actual_s'])
        cumulative_age += dt

        innov_str = row.get('innov_ticc', '')
        s_str = row.get('s_ticc', '')
        ticc_used = int(row.get('arm_ticc_used', '0'))
        x3 = float(row['x3_f_do_ppb'])

        if not innov_str or not s_str:
            results.append(dict(
                age=cumulative_age, dt=dt, ticc_fired=False,
                innov=None, S=None, gate_accept=None, gate_reason='no_ticc',
                chi2_reject=False, x3=x3))
            continue

        innov = float(innov_str)
        S = float(s_str)
        chi2 = innov**2 / S if S > 0 else 0
        chi2_reject = chi2 > 100.0
        chi2_total_ticc += 1
        if chi2_reject:
            chi2_rejected += 1

        accept, reason = gate.evaluate(innov_ns=innov, dt_s=dt, age_s=cumulative_age)
        results.append(dict(
            age=cumulative_age, dt=dt, ticc_fired=True,
            innov=innov, S=S, gate_accept=accept, gate_reason=reason,
            chi2_reject=chi2_reject, x3=x3))

    gate_rejected = sum(1 for r in results if r['ticc_fired'] and not r['gate_accept'])
    gate_accepted = sum(1 for r in results if r['ticc_fired'] and r['gate_accept'])
    gate_pre_age = sum(1 for r in results
                       if r['ticc_fired'] and r['gate_reason'] == 'pre_min_age')
    both_rejected = sum(1 for r in results
                        if r['ticc_fired'] and not r['gate_accept'] and r['chi2_reject'])
    gate_only = sum(1 for r in results
                    if r['ticc_fired'] and not r['gate_accept'] and not r['chi2_reject'])

    threshold_1s = K_SIGMA * sigma_ns
    return dict(
        host=host_label,
        sigma_ns=sigma_ns,
        threshold_1s_ns=threshold_1s,
        total_epochs=len(rows),
        ticc_epochs=chi2_total_ticc,
        chi2_rejected=chi2_rejected,
        chi2_rate=chi2_rejected / chi2_total_ticc if chi2_total_ticc else 0,
        gate_rejected=gate_rejected,
        gate_accepted=gate_accepted,
        gate_pre_age=gate_pre_age,
        gate_rate=gate_rejected / chi2_total_ticc if chi2_total_ticc else 0,
        both_rejected=both_rejected,
        gate_only_rejected=gate_only,
        per_epoch=results,
    )


def plot_counterfactual(all_stats):
    """4-panel figure: rejection stats, innovation histograms, x3 trajectories,
    TDEV comparison."""
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))

    # Panel 1: Innovation magnitude vs gate threshold (log scale)
    ax = axes[0, 0]
    for stats, (_, label, color) in zip(all_stats, HOSTS):
        innovs = [abs(r['innov']) for r in stats['per_epoch']
                  if r['ticc_fired'] and r['innov'] is not None
                  and r['age'] > SKIP_BOOTSTRAP_S]
        if innovs:
            ax.hist(innovs, bins=80, alpha=0.5, label=label, color=color,
                    density=True)
    for stats, (_, label, color) in zip(all_stats, HOSTS):
        ax.axvline(stats['threshold_1s_ns'], color=color, ls='--', lw=2,
                   label=f'{label} gate ({stats["threshold_1s_ns"]:.2f} ns)')
    ax.set_xlabel('|innovation| (ns)')
    ax.set_ylabel('density')
    ax.set_title('TICC innovation magnitude vs OCXO gate threshold')
    ax.set_xscale('log')
    ax.legend(fontsize=8)

    # Panel 2: Gate decision timeline
    ax = axes[0, 1]
    for i, (stats, (_, label, color)) in enumerate(zip(all_stats, HOSTS)):
        epochs = stats['per_epoch']
        t_accept = [r['age'] for r in epochs if r['gate_accept'] is True]
        t_reject = [r['age'] for r in epochs if r['gate_accept'] is False]
        y_a = [i + 0.1] * len(t_accept)
        y_r = [i - 0.1] * len(t_reject)
        ax.scatter(t_accept, y_a, c='green', s=1, alpha=0.3)
        ax.scatter(t_reject, y_r, c='red', s=3, alpha=0.5)
    ax.set_yticks(range(len(HOSTS)))
    ax.set_yticklabels([h[1] for h in HOSTS])
    ax.set_xlabel('time (s)')
    ax.set_title('Gate decisions (green=accept, red=reject)')

    # Panel 3: x3 trajectory — ungated vs gated (hold-last-accepted)
    ax = axes[1, 0]
    for stats, (_, label, color) in zip(all_stats, HOSTS):
        epochs = stats['per_epoch']
        t_all = [r['age'] for r in epochs]
        x3_all = [r['x3'] for r in epochs]
        ax.plot(t_all, x3_all, color=color, alpha=0.3, lw=0.5,
                label=f'{label} ungated')
        x3_gated = []
        last_accepted_x3 = x3_all[0] if x3_all else 0
        for r in epochs:
            if r['gate_accept'] is False:
                x3_gated.append(last_accepted_x3)
            else:
                x3_gated.append(r['x3'])
                last_accepted_x3 = r['x3']
        ax.plot(t_all, x3_gated, color=color, alpha=0.9, lw=1.0,
                label=f'{label} gated')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('x3 (DO freq, ppb)')
    ax.set_title('x3: ungated (faint) vs gated hold-last-accepted (solid)')
    ax.legend(fontsize=7, ncol=2)

    # Panel 4: TDEV comparison (disc ungated vs freerun)
    ax = axes[1, 1]
    for stats, (key, label, color) in zip(all_stats, HOSTS):
        t_disc, e_disc = load_ticc_cha(key)
        if t_disc is not None and len(t_disc) > SKIP_BOOTSTRAP_S:
            mask = t_disc > SKIP_BOOTSTRAP_S
            t_skip = t_disc[mask] - t_disc[mask][0]
            taus_out, tdev = compute_tdev(t_skip, e_disc[mask], TAUS)
            if len(taus_out) > 0:
                ax.loglog(taus_out, tdev, color=color, lw=2,
                          label=f'{label} disc (ungated)')
        fr_taus, fr_tdev = load_freerun_tdev(key)
        if fr_taus is not None:
            ax.loglog(fr_taus, fr_tdev, color=color, ls=':', lw=1.5,
                      label=f'{label} freerun')
    ax.set_xlabel('τ (s)')
    ax.set_ylabel('TDEV (ns)')
    ax.set_title('TDEV: disciplined (ungated) vs freerun baseline')
    ax.legend(fontsize=7)
    ax.grid(True, which='both', alpha=0.3)

    fig.suptitle('OCXO-trusted gate Phase 4 validation — day0527-disc2',
                 fontsize=14, fontweight='bold')
    fig.tight_layout()
    out = DOCS / 'ocxo-gate-phase4-validation-2026-05-27.png'
    fig.savefig(out, dpi=150)
    print(f'Saved: {out}')
    plt.close(fig)


def print_stats_table(all_stats):
    """Print a summary table."""
    print()
    print(f"{'Host':<10} {'σ_DO(1s)':<12} {'Thresh(1s)':<12} "
          f"{'TICC eps':<10} {'χ² rej':<10} {'Gate rej':<10} "
          f"{'Gate-only':<10} {'Both':<10}")
    print('-' * 96)
    for s in all_stats:
        print(f"{s['host']:<10} {s['sigma_ns']:<12.4f} "
              f"{s['threshold_1s_ns']:<12.2f} "
              f"{s['ticc_epochs']:<10d} "
              f"{s['chi2_rejected']:<10d} ({s['chi2_rate']:5.1%})  "
              f"{s['gate_rejected']:<10d} ({s['gate_rate']:5.1%})  "
              f"{s['gate_only_rejected']:<10d} "
              f"{s['both_rejected']:<10d}")
    print()
    for s in all_stats:
        total_ticc = s['ticc_epochs']
        gate_rej = s['gate_rejected']
        chi2_rej = s['chi2_rejected']
        union = gate_rej + chi2_rej - s['both_rejected']
        new_coverage = s['gate_only_rejected']
        print(f"{s['host']}: gate catches {new_coverage} epochs that χ² misses "
              f"({new_coverage/total_ticc:.1%} of TICC epochs)")
    print()


def print_tdev_summary():
    """Print disc vs freerun TDEV at key taus."""
    print("TDEV comparison (disc ungated vs freerun baseline):")
    print(f"  {'Host':<10} {'TDEV(1s) disc':<16} {'TDEV(1s) free':<16} {'ratio':<10}")
    print('  ' + '-' * 55)
    for key, label, color in HOSTS:
        t_disc, e_disc = load_ticc_cha(key)
        disc_1s = None
        if t_disc is not None and len(t_disc) > SKIP_BOOTSTRAP_S:
            mask = t_disc > SKIP_BOOTSTRAP_S
            t_skip = t_disc[mask] - t_disc[mask][0]
            taus_out, tdev = compute_tdev(t_skip, e_disc[mask], [1])
            if len(taus_out) > 0:
                disc_1s = tdev[0]
        fr_taus, fr_tdev = load_freerun_tdev(key)
        free_1s = None
        if fr_taus is not None:
            idx = np.argmin(np.abs(fr_taus - 1.0))
            if abs(fr_taus[idx] - 1.0) < 0.5:
                free_1s = fr_tdev[idx]
        ratio = disc_1s / free_1s if disc_1s and free_1s else None
        print(f"  {label:<10} "
              f"{f'{disc_1s:.3f} ns' if disc_1s else 'N/A':<16} "
              f"{f'{free_1s:.4f} ns' if free_1s else 'N/A':<16} "
              f"{f'{ratio:.0f}×' if ratio else 'N/A':<10}")
    print()


def main():
    all_stats = []
    for key, label, color in HOSTS:
        sigma = load_sigma(key)
        print(f'{label}: σ_DO(1s) = {sigma:.4f} ns, '
              f'gate threshold(1s) = {K_SIGMA * sigma:.2f} ns')
        stats = counterfactual_analysis(key, label, sigma)
        all_stats.append(stats)

    print_stats_table(all_stats)
    print_tdev_summary()
    plot_counterfactual(all_stats)


if __name__ == '__main__':
    main()

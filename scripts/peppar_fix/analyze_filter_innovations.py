#!/usr/bin/env python3
"""Analyze FixedPosFilter innovations for the 40-ps-obs → ns-class-dt_rx gap.

Four diagnostics, all from existing logs:

  1. Per-SV innovation autocorrelation at lags 1, 5, 30, 60, 300 s.
     White → R-as-independent model is OK.  Colored (positive autocorr
     at lag ≥ 1) → time-correlated obs noise (multipath, SSR latency,
     atmospheric trends) leaking through R.

  2. Per-SV innov.std / resid.std.  Close to 1 ⇒ filter accepts the
     observation almost as-is (low Kalman gain).  Much larger ⇒ filter
     aggressively pulls state toward observation (high gain).

  3. P[CLK, ZTD] / P[CLK,CLK]_post over time.  Big and persistent →
     ZTD walk leaks into clock estimate.

  4. Mean innov² vs filter's claimed (HPH^T + R) — Mehra consistency.
     Requires per-row R (which we only have as the rcal config), so
     this is approximate.  Reports a single ratio per row class.

Outputs HTML with per-SV autocorrelation curves + summary tables.

Hypothesis assignment:
- H1 (multipath colored noise): high autocorr at lag 1-60 s
- H2 (ZTD-CLK leak): large P[CLK,ZTD] / P[CLK,CLK]
- H3 (SSR latency colored): autocorr peaks at min-scale (60-300 s)
- H4 (IEKF exit early): innov.std/resid.std ratio close to filter's
  own claim (would need IEKF-internal diagnostics for a clean call)
- H5 (Q_clk model mismatch): wouldn't show in this analysis; needs
  separate freerun characterization
"""
from __future__ import annotations
import argparse
import csv
import math
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots


def load_per_sv(path):
    by_sv = defaultdict(lambda: {'t': [], 'pr_resid': [], 'pr_innov': [],
                                  'td_resid': [], 'td_innov': [],
                                  'elev': [], 'sys': None})
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                t = float(r['epoch_unix'])
                sv = r['sv']
                d = by_sv[sv]
                d['t'].append(t)
                d['pr_resid'].append(float(r['resid_pr_m']))
                d['pr_innov'].append(float(r.get('innov_pr_m', 'nan')))
                td_r = r.get('resid_td_m', 'nan').strip()
                td_i = r.get('innov_td_m', 'nan').strip()
                d['td_resid'].append(float(td_r) if td_r != 'nan' else float('nan'))
                d['td_innov'].append(float(td_i) if td_i != 'nan' else float('nan'))
                d['elev'].append(float(r['elev_deg']))
                d['sys'] = r['sys']
            except (ValueError, KeyError):
                continue
    return by_sv


def load_filter_state(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                rows.append({
                    't': float(r['epoch_unix']),
                    'p_clk_clk_pre': float(r['P_clk_clk_pre']),
                    'p_clk_clk_post': float(r['P_clk_clk_post']),
                    'p_clk_ztd': float(r['P_clk_ztd']),
                    'p_clk_isb_gal': float(r['P_clk_isb_gal']),
                    'q_clk_step': float(r['Q_clk_step']),
                })
            except (ValueError, KeyError):
                continue
    return rows


def autocorr_at_lag(x, lag):
    """Sample autocorrelation at integer lag, NaN-safe."""
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if len(x) < lag + 10:
        return None
    x0 = x - x.mean()
    n = len(x0)
    num = np.sum(x0[:n - lag] * x0[lag:])
    den = np.sum(x0 * x0)
    return float(num / den) if den > 0 else None


def std_ignore_nan(arr):
    a = np.asarray(arr, dtype=float)
    a = a[np.isfinite(a)]
    return float(np.std(a)) if len(a) > 1 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--per-sv-log", required=True, type=Path)
    ap.add_argument("--filter-state-log", required=True, type=Path)
    ap.add_argument("--start", default=None,
                    help="ISO UTC start (skip rows before)")
    ap.add_argument("--min-samples", type=int, default=300)
    ap.add_argument("-o", "--output", required=True, type=Path)
    args = ap.parse_args()

    start_unix = (datetime.fromisoformat(args.start).timestamp()
                  if args.start else 0.0)

    print(f"Loading per-SV log: {args.per_sv_log}")
    by_sv = load_per_sv(args.per_sv_log)
    print(f"  {len(by_sv)} SVs")
    print(f"Loading filter-state log: {args.filter_state_log}")
    fs = load_filter_state(args.filter_state_log)
    fs = [r for r in fs if r['t'] >= start_unix]
    print(f"  {len(fs)} epochs")

    # ============ Diagnostic 1: per-SV innov autocorr ============
    LAGS = [1, 5, 30, 60, 300]
    print(f"\n=== Diagnostic 1: per-SV TD-CP innovation autocorrelation ===")
    print(f"{'SV':>4} {'sys':>4} {'n':>5} {'elev':>6} "
          f"{'std_innov':>10} {'std_resid':>10} {'ratio':>6} "
          f"{'rho(1s)':>8} {'rho(5s)':>8} {'rho(30s)':>9} "
          f"{'rho(60s)':>9} {'rho(300s)':>10}")
    print("-" * 100)
    rho_summary = {lag: [] for lag in LAGS}
    fig = make_subplots(rows=2, cols=1, shared_xaxes=True,
                        subplot_titles=("TD-CP innovation autocorrelation per SV",
                                        "PR-IF innovation autocorrelation per SV"))
    for sv in sorted(by_sv.keys()):
        d = by_sv[sv]
        # Skip pre-start rows
        t = np.array(d['t'])
        mask = t >= start_unix
        td_innov = np.array(d['td_innov'])[mask]
        pr_innov = np.array(d['pr_innov'])[mask]
        td_resid = np.array(d['td_resid'])[mask]
        elev = np.array(d['elev'])[mask]
        n = mask.sum()
        if n < args.min_samples:
            continue
        td_innov_clean = td_innov[np.isfinite(td_innov)]
        if len(td_innov_clean) < args.min_samples:
            continue
        # Center on median (remove slow trend / bias)
        td_centered = td_innov - np.median(td_innov_clean)
        std_innov = std_ignore_nan(td_centered)
        std_resid = std_ignore_nan(td_resid - np.median(td_resid[np.isfinite(td_resid)]))
        ratio = std_innov / std_resid if std_resid > 0 else float('nan')
        rhos = {lag: autocorr_at_lag(td_centered, lag) for lag in LAGS}
        for lag in LAGS:
            if rhos[lag] is not None:
                rho_summary[lag].append((sv, rhos[lag]))
        def f(v):
            if v is None: return "—"
            return f"{v:+.2f}"
        print(f"{sv:>4} {d['sys']:>4} {n:>5} {np.median(elev):>5.1f}° "
              f"{std_innov*1000:>8.2f}mm {std_resid*1000:>8.2f}mm {ratio:>5.1f}x "
              f"{f(rhos[1]):>8} {f(rhos[5]):>8} {f(rhos[30]):>9} "
              f"{f(rhos[60]):>9} {f(rhos[300]):>10}")

        # Plot per-SV autocorr (lag 0 to 300 s, 30 points log-spaced)
        plot_lags = sorted(set([int(round(x)) for x in
                                 np.logspace(0, np.log10(min(300, n//4)), 20)]))
        td_autocorr = [autocorr_at_lag(td_centered, lag) for lag in plot_lags]
        pr_centered = pr_innov - np.median(pr_innov[np.isfinite(pr_innov)])
        pr_autocorr = [autocorr_at_lag(pr_centered, lag) for lag in plot_lags]
        fig.add_trace(go.Scatter(x=plot_lags, y=td_autocorr,
                                  mode='lines+markers', name=f"{sv} TD",
                                  legendgroup=sv),
                       row=1, col=1)
        fig.add_trace(go.Scatter(x=plot_lags, y=pr_autocorr,
                                  mode='lines+markers', name=f"{sv} PR",
                                  legendgroup=sv, showlegend=False),
                       row=2, col=1)

    # ============ Diagnostic 3: ZTD-CLK coupling timeseries ============
    print(f"\n=== Diagnostic 3: P[CLK, ZTD] / P[CLK,CLK]_post over time ===")
    ts = np.array([r['t'] for r in fs])
    if len(ts) > 0:
        p_clk = np.array([r['p_clk_clk_post'] for r in fs])
        p_clk_ztd = np.array([r['p_clk_ztd'] for r in fs])
        p_clk_isb = np.array([r['p_clk_isb_gal'] for r in fs])
        ratio_ztd = np.abs(p_clk_ztd) / np.maximum(p_clk, 1e-15)
        ratio_isb = np.abs(p_clk_isb) / np.maximum(p_clk, 1e-15)
        print(f"  |P[CLK,ZTD]| / P[CLK,CLK]_post:  median={np.median(ratio_ztd):.2f}, "
              f"p90={np.percentile(ratio_ztd, 90):.2f}")
        print(f"  |P[CLK,ISB_GAL]| / P[CLK,CLK]_post:  median={np.median(ratio_isb):.2f}, "
              f"p90={np.percentile(ratio_isb, 90):.2f}")

    # ============ Diagnostic 4: Mehra consistency ============
    # E[ν²] vs claimed.  Filter's claimed: HPH^T + R.  P[CLK,CLK]_pre is
    # the dominant H_clk × P × H_clk term for PR/TD rows.  R is from
    # rcal config (we don't have it per-row here without re-fitting).
    print(f"\n=== Diagnostic 4: innovation variance vs P[CLK,CLK]_pre ===")
    if len(ts) > 0:
        all_pr_innov = []
        for sv in by_sv:
            d = by_sv[sv]
            t = np.array(d['t'])
            m = t >= start_unix
            for v in np.array(d['pr_innov'])[m]:
                if np.isfinite(v):
                    all_pr_innov.append(v)
        all_pr_innov = np.array(all_pr_innov)
        if len(all_pr_innov) > 100:
            innov_var = np.var(all_pr_innov - np.median(all_pr_innov))
            mean_p_clk_pre = np.mean(p_clk)
            print(f"  PR-IF: E[innov²] = {innov_var:.3f} m²")
            print(f"  P[CLK,CLK]_pre mean = {mean_p_clk_pre:.3e} m²")
            print(f"  ratio (innov² / P_pre) = {innov_var / mean_p_clk_pre:.1f}")
            print(f"  (≫1 ⇒ R undersized OR HPH^T contributes from other states)")

    # ============ Diagnostic 2 summary across SVs ============
    print(f"\n=== Diagnostic 2: rho(lag) summary across SVs ===")
    print(f"  Per-lag median over SVs (TD-CP):")
    for lag in LAGS:
        vals = [v for sv, v in rho_summary[lag]]
        if vals:
            print(f"    lag={lag:>4}s:  median ρ = {np.median(vals):+.3f}  "
                  f"(p90 |ρ| = {np.percentile(np.abs(vals), 90):.3f}, n={len(vals)})")

    # ============ HTML output ============
    fig.update_xaxes(type='log', title_text='lag (s)', row=2, col=1)
    fig.update_yaxes(title_text='ρ (TD)', row=1, col=1, range=[-0.5, 1.05])
    fig.update_yaxes(title_text='ρ (PR)', row=2, col=1, range=[-0.5, 1.05])
    fig.add_hline(y=0, line=dict(color='gray', dash='dot'), row=1, col=1)
    fig.add_hline(y=0, line=dict(color='gray', dash='dot'), row=2, col=1)
    fig.update_layout(
        title="FixedPosFilter per-SV innovation autocorrelation — "
              "white-noise model holds if ρ→0 at lag≥1",
        height=800,
    )
    fig.write_html(args.output, include_plotlyjs="cdn")
    print(f"\nWrote {args.output}")


if __name__ == "__main__":
    main()

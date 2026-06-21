#!/usr/bin/env python3
"""Three-cornered-hat (TCH) decomposition of a two-channel TICC capture.

A single shared-reference TICC capture (chA + chB rows, each carrying
``ref_sec``/``ref_ps`` = that channel's PPS edge timestamped against the
TICC's shared Rb timebase) already contains a COMPLETE three-cornered
hat over the three nodes {chA, chB, Rb}:

    X = chA   (the disciplined-clock-A PPS)
    Y = chB   (the disciplined-clock-B / GNSS PPS)
    Z = Rb    (the TICC's reference oscillator, e.g. FE-5680A)

The TICC gives two of the three pairwise differences DIRECTLY:

    d_XZ = chA − Rb   (the chA rows' ref time, vs the Rb timebase)
    d_YZ = chB − Rb   (the chB rows' ref time, vs the Rb timebase)

and the third leg is their per-second difference, in which the common Rb
timebase cancels exactly:

    d_XY = chA − chB = d_XZ − d_YZ   (paired by ref_sec)

With all three pairwise differences we solve for each NODE's INDIVIDUAL
stability at every τ — including the Rb's own — WITHOUT needing a
reference better than the devices under test.  This is load-bearing:
``docs/two-site-sync-budget.md`` §3.1.1 shows the single-clock chA-vs-Rb
curve cannot attribute a long-τ TDEV bulge to the clock vs the Rb's own
random walk (the Rb's RWFM climbs to and past the per-clock budget beyond
~1000 s).  TCH is how we recover each clock's OWN stability past that Rb
floor and tell WHICH node is free-running (TDEV ramps at long τ) vs locked
(TDEV stays bounded).

Standard 3CH math (nodes X, Y, Z).  Per τ, with σ²(τ) = Allan variance of
each leg:

    σ²_chA = ½ (σ²_XY + σ²_XZ − σ²_YZ)
    σ²_chB = ½ (σ²_XY + σ²_YZ − σ²_XZ)
    σ²_Rb  = ½ (σ²_XZ + σ²_YZ − σ²_XY)

Take the square root for ADEV/TDEV.

NEGATIVE-VARIANCE GUARD.  When one node dominates, or the
mutual-independence assumption is violated, or there are too few samples
at a τ, a solved component variance can come out negative.  ``sqrt`` of a
negative number is not a deviation — so this tool sets that component
NaN at that τ and counts it, emitting a warning, rather than crashing or
clamping silently to zero (allantools' built-in
``three_cornered_hat_phase`` clamps to zero, which hides the violation).

INDEPENDENCE ASSUMPTION.  The three nodes {chA, chB, Rb} must be MUTUALLY
UNCORRELATED for the closed form to be unbiased.  If chA and chB are two
GNSS-locked clocks sharing one antenna (common SSR corrections, common
atmospheric/multipath path), their noise is correlated and the TCH split
is BIASED — the shared component leaks into the third (Rb) estimate.  The
clean configuration is two INDEPENDENT disciplined oscillators (or one
disciplined clock vs an independent GNSS PPS) against the Rb.  Stated in
the stderr note as well so the operator can't miss it.

The ``--groslambert`` option swaps the simple closed form for the
Groslambert-covariance estimator (a covariance-based 3CH that is more
robust to small residual correlations / finite-N variance); the simple
closed form above is the default.

PRECISION INVARIANT — TICC timestamps are kept as integer (sec, ps)
throughout the per-epoch arithmetic, same subtract-separately pattern as
``plot_clock_stability_stack.load_chA_phase`` and
``plot_xhost_agreement_cdf.load_pairs``.  See ``load_aligned_legs``.
"""
from __future__ import annotations

import argparse
import csv
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import allantools

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analysis_provenance import (stamp, provenance_line,  # noqa: E402
                                 skip_comment_lines)

_TOOLNAME = 'plot_tch.py'

_PS_PER_S = 10 ** 12
_TAUS = np.array([1, 2, 5, 10, 30, 100, 300, 1000, 3000, 10000],
                 dtype=np.float64)
# τ rows for the stderr/txt summary table (the task's reporting grid).
_REPORT_TAUS = (1.0, 10.0, 100.0, 1000.0, 10000.0)

_COLORS = {'chA': '#1f77b4', 'chB': '#ff7f0e', 'Rb': '#2ca02c'}

_INDEPENDENCE_NOTE = (
    'TCH ASSUMES the three nodes {chA, chB, Rb} are MUTUALLY UNCORRELATED. '
    'Two GNSS-locked clocks sharing one antenna are CORRELATED — the shared '
    'noise biases the split (leaks into the Rb estimate). Use two '
    'INDEPENDENT oscillators (or clock-vs-independent-GNSS-PPS) for an '
    'unbiased decomposition.')


def _read_ts(row: dict, ts_col: str) -> datetime:
    return datetime.fromisoformat(row[ts_col].replace('Z', '+00:00'))


def load_aligned_legs(path: Path, skip_before: datetime | None = None
                      ) -> tuple[np.ndarray, np.ndarray]:
    """Load chA-vs-Rb and chB-vs-Rb phase series (ns), aligned by ref_sec.

    Returns ``(d_XZ_ns, d_YZ_ns)`` over the longest run of consecutive
    seconds for which BOTH channels have a sample — so the three legs
    (d_XZ, d_YZ, and their difference d_XY) are all defined on the same
    epochs and 1 Hz-index math stays exact.

    Subtract-separately invariant (Bob, 2026-05-30): each event's total
    phase ``sec * _PS_PER_S + ps`` is built in Python int (arbitrary
    precision); the deviation from a perfect 1 Hz rate is computed in
    int64; only then cast to float64.  Same as ``load_chA_phase``.

    NOT linear-detrended here — the TCH solver works on the Allan
    variance of each leg, and the closed form differences three legs that
    must share a common detrend basis.  We detrend the per-leg phase
    consistently (see ``_legs_from_aligned``).
    """
    by_sec_a: dict[int, int] = {}
    by_sec_b: dict[int, int] = {}
    with open(path) as f:
        reader = csv.DictReader(skip_comment_lines(f))
        ts_col = ('ts_iso' if 'ts_iso' in reader.fieldnames
                  else 'host_timestamp' if 'host_timestamp' in reader.fieldnames
                  else None)
        for row in reader:
            ch = row['channel']
            if ch not in ('chA', 'chB'):
                continue
            if skip_before is not None and ts_col is not None:
                if _read_ts(row, ts_col) < skip_before:
                    continue
            s = int(row['ref_sec'])
            total = s * _PS_PER_S + int(row['ref_ps'])  # Python int — exact
            (by_sec_a if ch == 'chA' else by_sec_b)[s] = total

    common = sorted(set(by_sec_a) & set(by_sec_b))
    if len(common) < 60:
        return np.array([]), np.array([])

    secs = np.array(common)
    diffs = np.diff(secs)
    bounds = ([0] + list(np.where(diffs != 1)[0] + 1) + [len(common)])
    segs = [(bounds[i], bounds[i + 1]) for i in range(len(bounds) - 1)]
    a, b = max(segs, key=lambda ab: ab[1] - ab[0])
    seg = common[a:b]
    if len(seg) < 60:
        return np.array([]), np.array([])

    s0 = seg[0]
    a0 = by_sec_a[s0]
    b0 = by_sec_b[s0]
    # Deviation from perfect 1 Hz, per leg, kept in int64 then → ns.
    d_xz_int = np.array(
        [(by_sec_a[s] - a0) - (s - s0) * _PS_PER_S for s in seg],
        dtype=np.int64)
    d_yz_int = np.array(
        [(by_sec_b[s] - b0) - (s - s0) * _PS_PER_S for s in seg],
        dtype=np.int64)
    return d_xz_int.astype(np.float64) * 1e-3, d_yz_int.astype(np.float64) * 1e-3


def _detrend(phase_ns: np.ndarray) -> np.ndarray:
    """Linear-detrend a phase series (drop constant offset + freq offset)."""
    t = np.arange(len(phase_ns), dtype=np.float64)
    poly = np.polyfit(t, phase_ns, 1)
    return phase_ns - np.polyval(poly, t)


def _legs_from_aligned(d_xz_ns: np.ndarray, d_yz_ns: np.ndarray
                       ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Form the three detrended pairwise phase legs (ns).

    d_XZ = chA − Rb, d_YZ = chB − Rb, d_XY = d_XZ − d_YZ = chA − chB.
    Each leg is independently linear-detrended (constant offset = cable
    length; linear term = parked frequency offset vs the Rb — neither is a
    stability metric).  d_XY is formed from the RAW (pre-detrend) legs so
    the Rb cancels exactly, then detrended itself.
    """
    d_xy_raw = d_xz_ns - d_yz_ns          # Rb cancels exactly here
    return _detrend(d_xz_ns), _detrend(d_yz_ns), _detrend(d_xy_raw)


def _avar(phase_ns: np.ndarray, taus: np.ndarray, modified: bool
          ) -> tuple[np.ndarray, np.ndarray]:
    """Allan (or modified-Allan) variance of a phase series at ``taus``.

    Returns ``(taus_out, variance_s2)`` in s².  ``modified=True`` uses
    mdev (so the per-node TDEV = τ·mdev/√3 below); ``modified=False`` uses
    adev (so the per-node ADEV is the plain Allan deviation).
    """
    phase_s = phase_ns * 1e-9
    fn = allantools.mdev if modified else allantools.adev
    taus_out, dev, _, _ = fn(phase_s, rate=1.0, taus=taus, data_type='phase')
    return taus_out, dev * dev


def _solve_closed_form(var_xy: np.ndarray, var_xz: np.ndarray,
                       var_yz: np.ndarray
                       ) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    """Closed-form 3CH split with a negative-variance NaN guard.

        σ²_chA = ½(σ²_XY + σ²_XZ − σ²_YZ)
        σ²_chB = ½(σ²_XY + σ²_YZ − σ²_XZ)
        σ²_Rb  = ½(σ²_XZ + σ²_YZ − σ²_XY)

    Any component variance < 0 at a τ is set to NaN (NOT clamped to 0 —
    that would hide the assumption violation).  Returns the three variance
    arrays (s², with NaN where guarded) plus the count of negative entries.
    """
    var_a = 0.5 * (var_xy + var_xz - var_yz)
    var_b = 0.5 * (var_xy + var_yz - var_xz)
    var_r = 0.5 * (var_xz + var_yz - var_xy)
    neg = 0
    for v in (var_a, var_b, var_r):
        mask = v < 0
        neg += int(np.count_nonzero(mask))
        v[mask] = np.nan
    return var_a, var_b, var_r, neg


def _solve_groslambert(d_xz: np.ndarray, d_yz: np.ndarray, d_xy: np.ndarray,
                       taus: np.ndarray, modified: bool
                       ) -> tuple[np.ndarray, np.ndarray, np.ndarray,
                                  np.ndarray, int]:
    """Groslambert-covariance 3CH estimator.

    Instead of differencing three auto-variances, estimate each node's
    variance from the COVARIANCE of the two legs that share it.  For node
    X (= chA), the two legs sharing X are d_XZ and d_XY (X − Z and X − Y);
    their Allan covariance isolates X's own variance because the Z and Y
    terms are uncorrelated with each other and with X (under the same
    independence assumption).  This is more robust to finite-N variance
    and small residual correlations than differencing auto-variances.

    Allan covariance at τ is computed from the second differences of the
    two phase series:  cov = ⟨Δ²x_i · Δ²y_i⟩ / (2 τ²) for the m-sample
    averaging factor (modified variant uses the overlapping/averaged
    second difference, matching mdev's prefactor so the resulting TDEV is
    directly comparable to the closed-form path).

    Returns ``(taus_out, var_a, var_b, var_r, neg_count)`` in s².
    """
    ax = d_xz * 1e-9
    ay = d_yz * 1e-9
    axy = d_xy * 1e-9
    # Legs sharing each node (signs chosen so the shared node enters with
    # the same sign in both legs of a pair):
    #   X (chA): +d_XZ (X−Z),  +d_XY (X−Y)        → cov isolates X
    #   Y (chB): +d_YZ (Y−Z),  −d_XY (Y−X)        → cov isolates Y
    #   Z (Rb) : −d_XZ (Z−X),  −d_YZ (Z−Y)        → cov isolates Z
    pairs = {
        'A': (ax, axy),
        'B': (ay, -axy),
        'R': (-ax, -ay),
    }
    out: dict[str, np.ndarray] = {}
    taus_out_ref = None
    for key, (p, q) in pairs.items():
        cov = _allan_cov(p, q, taus, modified)
        taus_out_ref = cov[0]
        out[key] = cov[1]
    var_a, var_b, var_r = out['A'], out['B'], out['R']
    neg = 0
    for v in (var_a, var_b, var_r):
        mask = v < 0
        neg += int(np.count_nonzero(mask))
        v[mask] = np.nan
    return taus_out_ref, var_a, var_b, var_r, neg


def _allan_cov(p_s: np.ndarray, q_s: np.ndarray, taus: np.ndarray,
               modified: bool) -> tuple[np.ndarray, np.ndarray]:
    """Allan covariance of two phase series (s) at ``taus`` (s).

    For the standard (non-modified) variant at integer averaging factor
    m, using second differences of the phase:
        Δ²_i = x[i+2m] − 2 x[i+m] + x[i]
        cov(p,q) = ⟨Δ²p_i · Δ²q_i⟩ / (2 (m τ0)²)
    Setting p == q recovers the ordinary Allan variance, so this is a
    faithful covariance generalization.  The modified variant additionally
    averages the phase over m samples before second-differencing (the
    same smoothing mdev applies), so the covariance prefactor matches and
    the diagonal recovers the modified Allan variance.

    Returns ``(taus_kept, cov_s2)`` for taus with ≥1 usable term.
    """
    n = len(p_s)
    taus_keep: list[float] = []
    cov_keep: list[float] = []
    for tau in taus:
        m = int(round(tau))
        if m < 1:
            continue
        if modified:
            # Average over m, then second-difference at lag m.
            if n < 3 * m + 1:
                continue
            cs_p = np.cumsum(np.insert(p_s, 0, 0.0))
            cs_q = np.cumsum(np.insert(q_s, 0, 0.0))
            avg_p = (cs_p[m:] - cs_p[:-m]) / m
            avg_q = (cs_q[m:] - cs_q[:-m]) / m
            # second difference of the averaged phase at lag m
            d2p = avg_p[2 * m:] - 2 * avg_p[m:-m] + avg_p[:-2 * m]
            d2q = avg_q[2 * m:] - 2 * avg_q[m:-m] + avg_q[:-2 * m]
        else:
            if n < 2 * m + 1:
                continue
            d2p = p_s[2 * m:] - 2 * p_s[m:-m] + p_s[:-2 * m]
            d2q = q_s[2 * m:] - 2 * q_s[m:-m] + q_s[:-2 * m]
        if len(d2p) < 1:
            continue
        cov = float(np.mean(d2p * d2q)) / (2.0 * (m ** 2))
        taus_keep.append(float(m))
        cov_keep.append(cov)
    return np.array(taus_keep), np.array(cov_keep)


def _tdev_ns_from_var(taus: np.ndarray, var_s2: np.ndarray, modified: bool
                      ) -> np.ndarray:
    """TDEV (ns) from a per-node variance.

    TDEV(τ) = τ · Mdev(τ) / √3 by definition, so a modified-Allan variance
    converts directly.  When the closed form used plain Allan variance
    (``modified=False``) we still report a τ·σ/√3 figure as a TDEV-like
    proxy; pass ``modified=True`` (the default in main) for a true TDEV.
    """
    dev = np.sqrt(var_s2)            # NaN propagates through sqrt
    return (taus * dev / np.sqrt(3.0)) * 1e9


def compute_tch(path: Path, skip_before: datetime | None = None,
                groslambert: bool = False, taus: np.ndarray = _TAUS
                ) -> dict:
    """Run the TCH decomposition on a two-channel TICC CSV.

    Returns a dict with keys:
        'n'        : aligned-epoch count
        'taus'     : np.ndarray of τ actually solved
        'adev'     : {'chA':arr,'chB':arr,'Rb':arr}  (dimensionless)
        'tdev_ns'  : {'chA':arr,'chB':arr,'Rb':arr}  (ns)
        'neg_count': total negative-variance points guarded to NaN
        'estimator': 'closed-form' | 'groslambert'
    Empty 'taus' (and zero 'n') if the capture is too short.
    """
    d_xz, d_yz = load_aligned_legs(path, skip_before)
    if len(d_xz) < 60:
        return {'n': len(d_xz), 'taus': np.array([]), 'adev': {},
                'tdev_ns': {}, 'neg_count': 0,
                'estimator': 'groslambert' if groslambert else 'closed-form'}
    leg_xz, leg_yz, leg_xy = _legs_from_aligned(d_xz, d_yz)
    n = len(leg_xz)
    feasible = taus[taus * 3 < n]
    if not len(feasible):
        return {'n': n, 'taus': np.array([]), 'adev': {}, 'tdev_ns': {},
                'neg_count': 0,
                'estimator': 'groslambert' if groslambert else 'closed-form'}

    result: dict = {'n': n,
                    'estimator': 'groslambert' if groslambert else 'closed-form'}
    # ADEV (plain Allan) and TDEV (modified Allan) are computed on the
    # SAME legs, each via its own solve so both panels are self-consistent.
    if groslambert:
        ta_a, va_a, vb_a, vr_a, neg_a = _solve_groslambert(
            leg_xz, leg_yz, leg_xy, feasible, modified=False)
        ta_t, va_t, vb_t, vr_t, neg_t = _solve_groslambert(
            leg_xz, leg_yz, leg_xy, feasible, modified=True)
    else:
        ta_a, vxy_a = _avar(leg_xy, feasible, modified=False)
        _, vxz_a = _avar(leg_xz, feasible, modified=False)
        _, vyz_a = _avar(leg_yz, feasible, modified=False)
        va_a, vb_a, vr_a, neg_a = _solve_closed_form(vxy_a, vxz_a, vyz_a)
        ta_t, vxy_t = _avar(leg_xy, feasible, modified=True)
        _, vxz_t = _avar(leg_xz, feasible, modified=True)
        _, vyz_t = _avar(leg_yz, feasible, modified=True)
        va_t, vb_t, vr_t, neg_t = _solve_closed_form(vxy_t, vxz_t, vyz_t)

    result['taus'] = ta_t
    result['adev'] = {'chA': np.sqrt(va_a), 'chB': np.sqrt(vb_a),
                      'Rb': np.sqrt(vr_a)}
    result['tdev_ns'] = {
        'chA': _tdev_ns_from_var(ta_t, va_t, modified=True),
        'chB': _tdev_ns_from_var(ta_t, vb_t, modified=True),
        'Rb':  _tdev_ns_from_var(ta_t, vr_t, modified=True),
    }
    # Total negative-variance guard count across BOTH panels.
    result['neg_count'] = neg_a + neg_t
    return result


def summary_table(res: dict, label_a: str = 'chA', label_b: str = 'chB'
                  ) -> list[str]:
    """Build the per-node TDEV summary table lines (the stderr/txt grid).

    Rows = the three nodes (chA, chB, Rb); columns = τ in _REPORT_TAUS.
    NaN-guarded points render as 'NaN'.
    """
    taus = res.get('taus', np.array([]))
    tdev = res.get('tdev_ns', {})
    names = {'chA': f'{label_a} (chA)', 'chB': f'{label_b} (chB)', 'Rb': 'Rb'}
    tau_to_idx = {float(t): i for i, t in enumerate(taus)}
    header = (f'{"node":<18s}'
              + ''.join(f'{("τ=" + ("%g" % t) + "s"):>14s}'
                        for t in _REPORT_TAUS))
    lines = [header, '-' * len(header)]
    for key in ('chA', 'chB', 'Rb'):
        arr = tdev.get(key)
        cells = []
        for t in _REPORT_TAUS:
            if arr is None or t not in tau_to_idx:
                cells.append(f'{"—":>14s}')
            else:
                v = arr[tau_to_idx[t]]
                cells.append(f'{"NaN":>14s}' if (v != v)
                             else f'{v:>14.4f}')
        lines.append(f'{names[key]:<18s}' + ''.join(cells))
    lines.append(f'negative-variance τ points (guarded → NaN): '
                 f'{res.get("neg_count", 0)}')
    lines.append(f'estimator: {res.get("estimator", "closed-form")}; '
                 f'TDEV in ns')
    return lines


def _plot_node_curve(ax, taus: np.ndarray, ys: np.ndarray, color: str,
                     label: str, marker: str) -> None:
    """Plot one node's curve, skipping NaN-guarded points (mask them out
    so a NaN doesn't break the log-log line)."""
    if taus is None or not len(taus):
        return
    good = np.isfinite(ys) & (ys > 0)
    if not np.any(good):
        return
    ax.plot(taus[good], ys[good], color=color, ls='-', lw=2.0, marker=marker,
            ms=5, label=label)


def render_tch(path: Path, output: Path, label_a: str = 'chA',
               label_b: str = 'chB', skip_before: datetime | None = None,
               groslambert: bool = False, title: str | None = None,
               also_adev: bool = True) -> dict:
    """Compute the TCH split and write a stamped overlay PNG.

    TDEV panel (always) + ADEV panel (when ``also_adev``), log-log, with
    the moonshot per-clock budget (350 ps) and TICC resolution (60 ps)
    reference lines on the TDEV panel.  Returns the ``compute_tch`` dict.
    """
    res = compute_tch(path, skip_before, groslambert)
    print(_INDEPENDENCE_NOTE, file=sys.stderr)
    if not len(res.get('taus', [])):
        print(f'TCH: insufficient aligned data (n={res.get("n", 0)}) — '
              f'no figure written.', file=sys.stderr)
        return res

    for line in summary_table(res, label_a, label_b):
        print(line, file=sys.stderr)

    taus = res['taus']
    tdev = res['tdev_ns']
    adev = res['adev']
    node_label = {'chA': f'{label_a} (chA)', 'chB': f'{label_b} (chB)',
                  'Rb': 'Rb (reference)'}
    markers = {'chA': 'o', 'chB': 's', 'Rb': '^'}

    if also_adev:
        fig, (ax_tdev, ax_adev) = plt.subplots(1, 2, figsize=(13, 6))
        axes = [(ax_tdev, tdev, 'TDEV (ns)', 'TDEV(τ) — individual nodes'),
                (ax_adev, adev, 'ADEV (dimensionless)',
                 'ADEV(τ) — individual nodes')]
    else:
        fig, ax_tdev = plt.subplots(figsize=(8, 6))
        axes = [(ax_tdev, tdev, 'TDEV (ns)', 'TDEV(τ) — individual nodes')]

    for ax, ymap, ylabel, subtitle in axes:
        for key in ('chA', 'chB', 'Rb'):
            _plot_node_curve(ax, taus, ymap.get(key, np.array([])),
                             _COLORS[key], node_label[key], markers[key])
        ax.set_xscale('log')
        ax.set_yscale('log')
        ax.set_xlabel('τ (s)')
        ax.set_ylabel(ylabel)
        ax.set_title(subtitle)
        ax.grid(True, which='both', ls=':', alpha=0.4)
        ax.legend(loc='best', fontsize=8)

    # Budget reference lines on the TDEV panel.
    if ax_tdev.get_ylim()[0] < 0.35 < ax_tdev.get_ylim()[1]:
        ax_tdev.axhline(0.35, color='darkgreen', ls=':', alpha=0.55, lw=1.0)
        ax_tdev.text(taus[0], 0.36, ' moonshot per-clock budget (350 ps)',
                     fontsize=8, color='darkgreen', va='bottom')
    ax_tdev.axhline(0.060, color='red', ls=':', alpha=0.55, lw=1.0)
    ax_tdev.text(taus[0], 0.062, ' TICC single-shot resolution (60 ps)',
                 fontsize=8, color='red', va='bottom')

    est = res['estimator']
    default_title = (f'Three-cornered hat: {label_a} / {label_b} / Rb '
                     f'(individual stability, {est})')
    fig.suptitle(title or default_title, fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    stamp(fig, _TOOLNAME)
    fig.savefig(output, dpi=130)
    print(f'\nWrote {output}', file=sys.stderr)
    return res


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--from-csv', required=True, type=Path,
                    help='Two-channel (chA+chB) shared-reference TICC CSV.')
    ap.add_argument('--output', required=True, type=Path,
                    help='Output PNG path.')
    ap.add_argument('--label-a', default='chA',
                    help="Label for chA / node X (e.g. 'PiFace DO').")
    ap.add_argument('--label-b', default='chB',
                    help="Label for chB / node Y (e.g. 'GNSS PPS').")
    ap.add_argument('--groslambert', action='store_true',
                    help='Use the Groslambert-covariance estimator instead '
                         'of the simple closed form (more robust to small '
                         'residual correlations / finite N).')
    ap.add_argument('--no-adev', action='store_true',
                    help='Plot only the TDEV panel (skip the ADEV panel).')
    ap.add_argument('--skip-before', default=None,
                    help='ISO 8601 UTC timestamp; drop earlier rows.')
    ap.add_argument('--title', default=None)
    args = ap.parse_args()

    print(provenance_line(_TOOLNAME), file=sys.stderr)
    if not args.from_csv.exists():
        ap.error(f'--from-csv path does not exist: {args.from_csv}')

    skip_dt = None
    if args.skip_before:
        skip_dt = datetime.fromisoformat(args.skip_before.replace('Z', '+00:00'))

    res = render_tch(args.from_csv, args.output, args.label_a, args.label_b,
                     skip_before=skip_dt, groslambert=args.groslambert,
                     title=args.title, also_adev=not args.no_adev)
    return 0 if len(res.get('taus', [])) else 1


if __name__ == '__main__':
    sys.exit(main())

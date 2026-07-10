#!/usr/bin/env python3
"""clock_report.py — one-command N-clock comparison report (PDF).

The general-N successor to ``two_clock_report.py``.  Given N disciplined
clocks — each identified by ``(label, csv, channel, group)`` and all
measured against the lab's ONE shared Rb through (possibly several)
shared-reference TICCs — produce a 4-page 16:9 landscape PDF plus a printed
morning summary:

  Page 1  Pairwise excursion MATRIX     N×N p95 |Δ|, PASS/FAIL vs budget
                                        (the at-a-glance morning deliverable)
  Page 2  All-pairs agreement CDF       every pair's P(|Δ| ≤ T) + 1/2 ns lines
  Page 3  Per-clock TDEV & ADEV vs Rb   all N overlaid (Rb+TDC-limited)
  Page 4  Per-clock individual stability N-cornered hat (Rb removed)

Budget grading is GROUP-AWARE (docs/two-site-sync-budget.md): a pair whose
two clocks share an antenna group is graded against the shared-antenna
bound (default 1 ns @ p95); a cross-group pair against the separate-antenna
bound (default 2 ns).  Both configurable (``--shared-ns`` / ``--separate-ns``).

Two independent time axes:
  * ``--skip-before`` / ``--skip-after`` bound the EXCURSION window — the span
    over which the clocks are jointly comparable (e.g. from when the LAST live
    clock came up + settled).  Applies to the pairwise excursion + the N-hat.
  * per-clock TDEV/ADEV uses each clock's FULL trace (stability is a property
    of its own trace), optionally trimmed by ``--stability-skip-before``.

Non-contemporaneous clocks: a clock from a different time window (e.g. a
historical GNSSDO+ run) can be included for its stability curve alone — it
still appears in the per-clock TDEV/ADEV overlay, but every excursion pair
involving it is N/A (no shared window to difference; never 0, never a crash).
Mark it with ``stability_only = true`` in the manifest, a ``:stability`` 4th
field on ``--host``, or just let the tool auto-detect the zero overlap.  So
"6 live + 1 historical" reads as a 7×7 TDEV overlay with an effectively 6×6
excursion matrix.

This is a REPORT COMPOSER, not new analysis: every number comes from the
provenance-stamped cores in ``clock_report_core`` (which in turn reuse the
campaign's single-purpose tools), so a figure here is apples-to-apples with
``compare_clocks.py`` / ``plot_xhost_*`` / ``plot_tch``.

Two ways to name the clocks:

  # Repeatable --host (channel default chA; group default "default"):
  python3 scripts/clock_report.py \
      --host 'PiFace=data/piface-overnight.ticc.csv:chA:ufoLondon1' \
      --host 'PiPuss=data/pipuss-overnight.ticc.csv:chA:ufoLondon3' \
      --host 'otcBob1=data/madhat-ticc1-overnight.ticc.csv:chA:ufoLondon1' \
      --host 'ptBoat=data/madhat-ticc1-overnight.ticc.csv:chB:ufoLondon1' \
      --host 'GNSSDO+=data/madhat-ticc4-overnight.ticc.csv:chA:ufoLondon1' \
      --host 'Dot166=data/madhat-ticc4-overnight.ticc.csv:chB:ufoLondon1' \
      --out data/overnight-20260709-clock-report.pdf \
      --skip-before 2026-07-09T04:00:00Z
  # → ufoLondon1 group = {PiFace, otcBob1, ptBoat, GNSSDO+, Dot166} (1 ns),
  #   PiPuss = ufoLondon3 (2 ns vs the group).  15 pairs auto-enumerated.

  # Add a 7th, historical (non-contemporaneous) clock for stability compare:
  #   --host 'OldGNSSDO=data/gnssdo-atomichron-20260702.ticc.csv:chA:ufoLondon1:stability'
  #   --skip-before 2026-07-09T20:25:00Z   # all 6 live clocks up + settled

  # ...or a manifest (TOML/JSON) instead of many flags:
  python3 scripts/clock_report.py --manifest data/overnight.toml --out report.pdf

  # N=2 convenience — one csv's chA/chB (the old two_clock_report shape):
  python3 scripts/clock_report.py --from-csv cap.csv \
      --label-a 'GNSSDO+' --label-b 'OTA166' --out report.pdf --shared-ns 1.0
"""
from __future__ import annotations

import argparse
import itertools
import sys
import tempfile
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

_SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPT_DIR))
sys.path.insert(0, str(_SCRIPT_DIR.parent / 'tools'))

from analysis_provenance import stamp, provenance_line  # noqa: E402
from plot_xhost_agreement_cdf import cdf_xy  # noqa: E402
import clock_report_core as core  # noqa: E402

# Reuse the TICC ⊕ FE-5680A-Rb measurement floor without inheriting
# plot_stability_slide's import-time rcParams mutation.
import matplotlib as _mpl  # noqa: E402
_saved_rc = _mpl.rcParams.copy()
from plot_stability_slide import measurement_floor  # noqa: E402
_mpl.rcParams.update(_saved_rc)

_TOOLNAME = 'clock_report.py'
_FIGSIZE = (12.8, 7.2)   # 16:9 landscape

# A stable, high-contrast color per clock (up to 10; cycles beyond).
_PALETTE = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
            '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']


def _colors_for(labels: list) -> dict:
    return {lab: _PALETTE[i % len(_PALETTE)] for i, lab in enumerate(labels)}


def _finish(pdf: PdfPages, fig) -> None:
    stamp(fig, _TOOLNAME)
    pdf.savefig(fig)
    plt.close(fig)


def _add_floor(ax, kind: str, tau_max: float) -> None:
    """Shade the TICC single-shot ⊕ Rb measurement floor behind the data."""
    ymin, ymax = ax.get_ylim()
    grid = np.logspace(0, np.log10(max(tau_max, 10.0)), 200)
    floor = measurement_floor(grid, kind)
    ax.fill_between(grid, ymin, np.clip(floor, ymin, ymax),
                    color='0.82', alpha=0.6, zorder=0, lw=0)
    ax.plot(grid, floor, color='0.5', lw=1.0, zorder=1,
            label='TICC ⊕ FE-5680A Rb floor')
    ax.set_ylim(ymin, ymax)


# --------------------------------------------------------------------------
# Page 1 — pairwise excursion matrix (the morning at-a-glance)
# --------------------------------------------------------------------------
def page_matrix(pdf: PdfPages, result: core.AnalysisResult) -> None:
    p95, bound, labels = core.p95_matrix(result)
    n = len(labels)
    groups = [result.hosts[l].spec.group for l in labels]
    ratio = p95 / bound       # 1.0 = exactly at budget; >1 = FAIL

    fig, ax = plt.subplots(figsize=_FIGSIZE)
    # Green (pass) → red (fail) around the budget at ratio=1.
    cmap = plt.get_cmap('RdYlGn_r').copy()
    cmap.set_bad('0.85')
    im = ax.imshow(np.ma.masked_invalid(ratio), cmap=cmap, vmin=0.0, vmax=2.0)
    cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label('p95 |Δ|  /  budget    (1.0 = at bound; <1 PASS, >1 FAIL)')

    stab = [result.hosts[l].spec.stability_only for l in labels]
    ticks = [f'{l}{" *" if s else ""}\n[{g}]'
             for l, g, s in zip(labels, groups, stab)]
    ax.set_xticks(range(n)); ax.set_xticklabels(ticks, fontsize=8, rotation=45,
                                                ha='right')
    ax.set_yticks(range(n)); ax.set_yticklabels(ticks, fontsize=8)
    for i in range(n):
        for j in range(n):
            if i == j:
                ax.text(j, i, '—', ha='center', va='center', fontsize=9,
                        color='0.5')
                continue
            if np.isnan(p95[i, j]):
                ax.text(j, i, 'N/A', ha='center', va='center',
                        fontsize=7, color='0.4')
                continue
            grade = 'PASS' if p95[i, j] <= bound[i, j] else 'FAIL'
            ax.text(j, i, f'{p95[i, j]:.2f}\n{grade}', ha='center',
                    va='center', fontsize=7,
                    color='black' if ratio[i, j] < 1.4 else 'white',
                    fontweight='bold' if grade == 'FAIL' else 'normal')

    npass = sum(1 for p in result.pairs if p.grade == 'PASS')
    nfail = sum(1 for p in result.pairs if p.grade == 'FAIL')
    nna = sum(1 for p in result.pairs if p.grade in core.NA_GRADES)
    star = '   (* = stability-only / non-contemporaneous → N/A across the row)' \
        if any(stab) else ''
    ax.set_title(
        f'Pairwise excursion matrix — p95 |Δ| (ns), group-aware budget\n'
        f'{len(result.pairs)} pairs: {npass} PASS / {nfail} FAIL'
        f'{f" / {nna} N/A" if nna else ""}   '
        f'(same group ≤ {result.shared_ns:g} ns, cross group '
        f'≤ {result.separate_ns:g} ns){star}', fontsize=10)
    fig.tight_layout()
    _finish(pdf, fig)


# --------------------------------------------------------------------------
# Page 2 — all-pairs phase-agreement CDF overlay
# --------------------------------------------------------------------------
def page_cdf(pdf: PdfPages, result: core.AnalysisResult,
             samples_by_label: dict) -> None:
    fig, ax = plt.subplots(figsize=_FIGSIZE)
    max_x = 2200.0
    plotted = 0
    # Order curves worst-first so the offenders are on top of the legend.
    for pr in sorted(result.pairs, key=lambda p: -(p.p95 if p.p95 == p.p95
                                                    else -1)):
        if pr.grade in core.NA_GRADES:      # no contemporaneous window
            continue
        abs_ns = core.pair_abs_delta_ns(samples_by_label[pr.a],
                                        samples_by_label[pr.b])
        if len(abs_ns) < 60:
            continue
        abs_ps = abs_ns * 1e3
        sx, ypct = cdf_xy(abs_ps)
        max_x = max(max_x, float(sx[-1]))
        ls = '-' if pr.shared else '--'
        col = '#d62728' if pr.grade == 'FAIL' else '#2ca02c'
        ax.plot(sx, ypct, lw=1.3, ls=ls, color=col, alpha=0.75,
                label=f'{pr.a}↔{pr.b} p95={pr.p95:.2f} [{pr.grade}]')
        plotted += 1

    ax.set_xscale('log')
    ax.set_xlim(60.0, max_x * 1.2)
    ax.set_ylim(0, 100)
    for x, c, txt in ((60, 'red', 'TICC res\n(60 ps)'),
                      (1000, 'darkgreen', 'shared-ant\n(1 ns)'),
                      (2000, 'blue', 'separate-ant\n(2 ns)')):
        if ax.get_xlim()[0] <= x <= ax.get_xlim()[1]:
            ax.axvline(x, color=c, ls=':', alpha=0.65, lw=1.0)
            ax.text(x, 1.5, f' {txt}', fontsize=8, color=c, va='bottom')
    for q in (50, 95, 99):
        ax.axhline(q, color='gray', ls=':', alpha=0.55, lw=0.8)
        ax.text(62, q + 0.7, f'{q}%', fontsize=8, color='gray')
    ax.set_xlabel('|Δ| (ps) — log scale')
    ax.set_ylabel('Cumulative % of paired epochs   P(|Δ| ≤ T)')
    ax.set_title('All-pairs phase-agreement CDF  '
                 '(solid = same antenna group, dashed = cross group; '
                 'green PASS / red FAIL)', fontsize=11)
    ax.grid(True, which='both', ls=':', alpha=0.4)
    if plotted:
        ax.legend(loc='lower right', fontsize=6.5, ncol=1 if plotted <= 8 else 2)
    else:
        ax.text(0.5, 0.5, 'no pairs with enough overlap',
                ha='center', va='center', transform=ax.transAxes)
    fig.tight_layout()
    _finish(pdf, fig)


# --------------------------------------------------------------------------
# Page 3 — per-clock TDEV & ADEV vs the shared Rb (all N overlaid)
# --------------------------------------------------------------------------
def page_stability_vs_rb(pdf: PdfPages, result: core.AnalysisResult) -> None:
    colors = _colors_for(result.order)
    fig, (ax_t, ax_a) = plt.subplots(1, 2, figsize=_FIGSIZE)
    tmax_t = tmax_a = 10.0
    for lab in result.order:
        hr = result.hosts[lab]
        c = colors[lab]
        if hr.tdev:
            tt = sorted(hr.tdev)
            ax_t.plot(tt, [hr.tdev[t] for t in tt], color=c, lw=1.8,
                      marker='o', ms=3, label=f'{lab} (n={hr.n})')
            tmax_t = max(tmax_t, tt[-1])
        if hr.adev:
            ta = sorted(hr.adev)
            ax_a.plot(ta, [hr.adev[t] for t in ta], color=c, lw=1.8,
                      marker='o', ms=3, label=f'{lab} (n={hr.n})')
            tmax_a = max(tmax_a, ta[-1])

    for ax, ylabel, sub, kind, tmax in (
            (ax_t, 'TDEV (ns)', 'TDEV(τ) — chX vs Rb', 'tdev', tmax_t),
            (ax_a, 'ADEV (dimensionless)', 'ADEV(τ) — chX vs Rb', 'adev', tmax_a)):
        ax.set_xscale('log'); ax.set_yscale('log')
        ax.set_xlabel('τ (s)'); ax.set_ylabel(ylabel); ax.set_title(sub)
        ax.grid(True, which='both', ls=':', alpha=0.4)
        _add_floor(ax, kind, tmax)
        ax.legend(loc='best', fontsize=7)
    if ax_t.get_ylim()[0] < 0.35 < ax_t.get_ylim()[1]:
        ax_t.axhline(0.35, color='darkgreen', ls=':', alpha=0.55, lw=1.0)
        ax_t.text(1.0, 0.36, ' moonshot per-clock budget (350 ps)',
                  fontsize=8, color='darkgreen', va='bottom')
    fig.suptitle('Per-clock stability vs the shared Rb reference   '
                 '(each curve carries the Rb + TDC floor — see next page)',
                 fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    _finish(pdf, fig)


# --------------------------------------------------------------------------
# Page 4 — per-clock individual stability (N-cornered hat)
# --------------------------------------------------------------------------
def page_hat(pdf: PdfPages, result: core.AnalysisResult) -> None:
    colors = _colors_for(result.order)
    fig, (ax_t, ax_a) = plt.subplots(1, 2, figsize=_FIGSIZE)
    tmax_t = tmax_a = 10.0
    any_data = False
    for lab in result.order:
        h = result.hat.get(lab)
        if not h:
            continue
        c = colors[lab]
        tt, td = h['taus_tdev'], h['tdev_ns']
        ta, da = h['taus_adev'], h['adev']
        good_t = np.isfinite(td) & (td > 0)
        good_a = np.isfinite(da) & (da > 0)
        if np.any(good_t):
            ax_t.plot(tt[good_t], td[good_t], color=c, lw=1.8, marker='o',
                      ms=3, label=lab)
            tmax_t = max(tmax_t, float(tt[good_t][-1])); any_data = True
        if np.any(good_a):
            ax_a.plot(ta[good_a], da[good_a], color=c, lw=1.8, marker='o',
                      ms=3, label=lab)
            tmax_a = max(tmax_a, float(ta[good_a][-1])); any_data = True

    for ax, ylabel, sub, kind, tmax in (
            (ax_t, 'TDEV (ns)', 'TDEV(τ) — individual clock', 'tdev', tmax_t),
            (ax_a, 'ADEV (dimensionless)', 'ADEV(τ) — individual clock',
             'adev', tmax_a)):
        ax.set_xscale('log'); ax.set_yscale('log')
        ax.set_xlabel('τ (s)'); ax.set_ylabel(ylabel); ax.set_title(sub)
        ax.grid(True, which='both', ls=':', alpha=0.4)
        if any_data:
            _add_floor(ax, kind, tmax)
        ax.legend(loc='best', fontsize=7)
    if ax_t.get_ylim()[0] < 0.35 < ax_t.get_ylim()[1]:
        ax_t.axhline(0.35, color='darkgreen', ls=':', alpha=0.55, lw=1.0)
        ax_t.text(1.0, 0.36, ' moonshot per-clock budget (350 ps)',
                  fontsize=8, color='darkgreen', va='bottom')
    if not any_data:
        for ax in (ax_t, ax_a):
            ax.text(0.5, 0.5, 'insufficient aligned data for the N-hat',
                    ha='center', va='center', transform=ax.transAxes)
    fig.suptitle(
        f'Per-clock individual stability — N-cornered hat (Rb removed; '
        f'{result.hat_neg} neg-var τ→NaN)\n'
        f'ASSUMES clocks mutually uncorrelated; clocks sharing one antenna '
        f'are correlated → split biased', fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    _finish(pdf, fig)


# --------------------------------------------------------------------------
# Printed morning summary
# --------------------------------------------------------------------------
def print_summary(result: core.AnalysisResult, out=sys.stdout) -> None:
    p = lambda *a: print(*a, file=out)
    p('\n== Per-clock TDEV vs Rb (ns) ==')
    hdr = f'{"clock":<16s}{"group":<14s}{"n":>7s}' + ''.join(
        f'{("τ=" + ("%g" % t) + "s"):>12s}' for t in core.REPORT_TAUS)
    p(hdr); p('-' * len(hdr))
    for lab in result.order:
        hr = result.hosts[lab]
        cells = ''.join(
            (f'{hr.tdev[t]:>12.3f}' if t in hr.tdev else f'{"—":>12s}')
            for t in core.REPORT_TAUS)
        tag = ' *' if hr.spec.stability_only else ''
        p(f'{(lab + tag):<16s}{hr.spec.group:<14s}{hr.n:>7d}{cells}')
    if any(result.hosts[l].spec.stability_only for l in result.order):
        p('  (* = stability-only / non-contemporaneous: TDEV from its own '
          'full trace; N/A in every excursion pair)')

    p('\n== Pairwise p95 |Δ| matrix (ns) — [group] budget, PASS/FAIL ==')
    p95, bound, labels = core.p95_matrix(result)
    colw = 13
    head = f'{"":<16s}' + ''.join(f'{l[:colw-1]:>{colw}s}' for l in labels)
    p(head)
    for i, li in enumerate(labels):
        row = f'{li[:15]:<16s}'
        for j in range(len(labels)):
            if i == j:
                row += f'{"—":>{colw}s}'
            elif np.isnan(p95[i, j]):
                row += f'{"N/A":>{colw}s}'
            else:
                g = 'P' if p95[i, j] <= bound[i, j] else 'F'
                row += f'{f"{p95[i,j]:.2f}/{g}":>{colw}s}'
        p(row)

    npass = sum(1 for pr in result.pairs if pr.grade == 'PASS')
    nfail = sum(1 for pr in result.pairs if pr.grade == 'FAIL')
    nna = sum(1 for pr in result.pairs if pr.grade in core.NA_GRADES)
    p(f'\nVERDICT: {len(result.pairs)} pairs — {npass} PASS / {nfail} FAIL'
      f'{f" / {nna} N/A" if nna else ""}   '
      f'(same group ≤ {result.shared_ns:g} ns, cross group '
      f'≤ {result.separate_ns:g} ns)')
    if nfail:
        p('  FAILing pairs:')
        for pr in sorted(result.pairs, key=lambda x: -x.p95):
            if pr.grade == 'FAIL':
                kind = 'shared' if pr.shared else 'separate'
                p(f'    {pr.a}↔{pr.b}  p95={pr.p95:.2f} ns > '
                  f'{pr.bound_ns:g} ns ({kind})')


# --------------------------------------------------------------------------
# Report builder
# --------------------------------------------------------------------------
def build_report(host_specs: list, out_pdf: Path,
                 shared_ns: float = core.SHARED_NS_DEFAULT,
                 separate_ns: float = core.SEPARATE_NS_DEFAULT,
                 skip_before: datetime | None = None,
                 skip_after: datetime | None = None,
                 stability_skip_before: datetime | None = None,
                 work_dir: Path | None = None,
                 title: str | None = None) -> core.AnalysisResult:
    """Analyze N clocks and write the 4-page PDF.  Returns the AnalysisResult."""
    out_pdf = Path(out_pdf)
    out_pdf.parent.mkdir(parents=True, exist_ok=True)
    wd = Path(work_dir) if work_dir else out_pdf.parent

    result = core.analyze(host_specs, shared_ns=shared_ns,
                          separate_ns=separate_ns, skip_before=skip_before,
                          skip_after=skip_after,
                          stability_skip_before=stability_skip_before,
                          work_dir=wd, do_hat=True)

    # Reload windowed samples once for the CDF page (analyze() doesn't retain
    # them to keep AnalysisResult light).  Stability-only clocks never enter
    # the excursion pairing, so they get no samples here.
    resolved = core.resolve_windows(
        [h.csv for h in host_specs if not h.stability_only],
        skip_before, skip_after, wd)
    samples_by_label = {
        h.label: ([] if h.stability_only else
                  core.load_ticc_cha(resolved.get(h.csv, h.csv), None,
                                     channel=h.channel))
        for h in host_specs}

    with PdfPages(out_pdf) as pdf:
        page_matrix(pdf, result)
        page_cdf(pdf, result, samples_by_label)
        page_stability_vs_rb(pdf, result)
        page_hat(pdf, result)
        d = pdf.infodict()
        d['Title'] = title or f'{len(host_specs)}-clock comparison report'
        d['Creator'] = _TOOLNAME
    return result


def _specs_from_args(args) -> list:
    specs: list = []
    if args.manifest:
        specs, meta = core.load_manifest(Path(args.manifest))
        if 'shared_ns' in meta and args.shared_ns is None:
            args.shared_ns = meta['shared_ns']
        if 'separate_ns' in meta and args.separate_ns is None:
            args.separate_ns = meta['separate_ns']
    for h in args.host or []:
        specs.append(core.parse_host_spec(h))
    if args.from_csv:
        # N=2 convenience: one csv's chA/chB, same antenna group.
        grp = args.group or 'default'
        specs.append(core.HostSpec(args.label_a, Path(args.from_csv),
                                   args.channel_a, grp))
        specs.append(core.HostSpec(args.label_b, Path(args.from_csv),
                                   args.channel_b, grp))
    return specs


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__[__doc__.index('Two ways'):])
    ap.add_argument('--host', action='append', default=[],
                    help='Repeatable clock spec LABEL=CSV[:CHANNEL][:GROUP] '
                         '(channel default chA, group default "default").')
    ap.add_argument('--manifest', default=None,
                    help='TOML/JSON manifest of hosts (alternative to --host).')
    ap.add_argument('--from-csv', default=None,
                    help='N=2 convenience: one shared-ref TICC csv; its chA/chB '
                         'become two clocks in the same antenna group.')
    ap.add_argument('--label-a', default='clock A', help='--from-csv chA label.')
    ap.add_argument('--label-b', default='clock B', help='--from-csv chB label.')
    ap.add_argument('--channel-a', default='chA', help='--from-csv clock-A channel.')
    ap.add_argument('--channel-b', default='chB', help='--from-csv clock-B channel.')
    ap.add_argument('--group', default=None,
                    help='--from-csv antenna group for both clocks.')
    ap.add_argument('--out', required=True, type=Path, help='Output PDF path.')
    ap.add_argument('--shared-ns', type=float, default=None,
                    help=f'Same-group p95 budget (default '
                         f'{core.SHARED_NS_DEFAULT:g} ns).')
    ap.add_argument('--separate-ns', type=float, default=None,
                    help=f'Cross-group p95 budget (default '
                         f'{core.SEPARATE_NS_DEFAULT:g} ns).')
    ap.add_argument('--skip-before', default=None,
                    help='ISO-8601 UTC; start of the EXCURSION window (when '
                         'all live clocks are up + settled).  Applies to the '
                         'pairwise excursion + N-hat only — per-clock TDEV '
                         'still uses each full trace.')
    ap.add_argument('--skip-after', default=None,
                    help='ISO-8601 UTC; end of the excursion window (cut a '
                         'late event).  Excursion-only, like --skip-before.')
    ap.add_argument('--stability-skip-before', default=None,
                    help='ISO-8601 UTC; trim each per-clock TDEV/ADEV trace '
                         "(drop a clock's own bootstrap transient).  Default: "
                         'use each clock full trace.')
    ap.add_argument('--title', default=None)
    args = ap.parse_args()

    print(provenance_line(_TOOLNAME), file=sys.stderr)
    specs = _specs_from_args(args)
    if len(specs) < 2:
        ap.error('need at least 2 clocks (via --host / --manifest / --from-csv)')

    missing = [str(s.csv) for s in specs if not Path(s.csv).exists()]
    if missing:
        ap.error(f'CSV path(s) do not exist: {", ".join(sorted(set(missing)))}')

    shared_ns = args.shared_ns if args.shared_ns is not None else core.SHARED_NS_DEFAULT
    separate_ns = (args.separate_ns if args.separate_ns is not None
                   else core.SEPARATE_NS_DEFAULT)
    sb = core.parse_iso(args.skip_before) if args.skip_before else None
    sa = core.parse_iso(args.skip_after) if args.skip_after else None
    ssb = (core.parse_iso(args.stability_skip_before)
           if args.stability_skip_before else None)

    with tempfile.TemporaryDirectory(prefix='clock_report_') as td:
        wd = args.out.parent if (sb or sa) is None else Path(td)
        result = build_report(specs, args.out, shared_ns=shared_ns,
                              separate_ns=separate_ns, skip_before=sb,
                              skip_after=sa, stability_skip_before=ssb,
                              work_dir=wd, title=args.title)
    print_summary(result)
    print(f'\n{_TOOLNAME}: wrote {args.out}  (4 pages, {len(specs)} clocks)',
          file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())

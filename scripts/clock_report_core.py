#!/usr/bin/env python3
"""Shared cores for the N-clock comparison report (``clock_report.py``).

This module holds the *general* (N-clock) analysis cores that the report
composer builds on, factored out so the composer stays thin and the math
stays testable without matplotlib.  It deliberately REUSES the proven,
provenance-stamped single-purpose analysis functions from the campaign
tools rather than re-deriving them:

  * pairing + linear-detrend of a virtual cross-host Δ ...... plot_xhost_virtual
  * per-channel-vs-Rb detrended phase stream ................ plot_clock_stability_stack
  * TDEV/ADEV (allantools) with finite-N filtering .......... plot_stability_slide
  * empirical CDF x/y ....................................... plot_xhost_agreement_cdf

What is NEW here (and could not simply be reused, because every existing
tool is pair-bound to 2 clocks / 1 TICC):

  * the (label, csv, channel, group) HOST MODEL and its ``--host`` /
    manifest parsers,
  * ALL-PAIRS enumeration + budget grading (group-aware: same antenna group
    → shared-antenna bound, different groups → separate-antenna bound;
    docs/two-site-sync-budget.md),
  * the N-cornered-hat (averaged pairwise three-cornered hat over the fleet)
    that recovers each clock's OWN stability past the shared-Rb floor.

TICC CSV schema (ticc_capture.py):
    ts_iso|host_timestamp, channel, ref_sec, ref_ps, recv_mono|host_monotonic
Each row = one PPS edge on ``channel`` timestamped against that TICC's Rb
timebase.  All lab TICCs share ONE physical Rb, so any two channels' vs-Rb
streams are directly differenceable across different TICC units (paired by
host_timestamp UTC).  Cross-TICC pairs carry two TDCs' noise in quadrature
(~30% more) — inherent and acceptable.

PRECISION INVARIANT (inherited from the reused cores): per-event
``total_ps = ref_sec * 1e12 + ref_ps`` is built in Python int (arbitrary
precision); the pairing subtraction stays in Python int and is cast to
int64/float64 only AFTER the subtract, so sub-ns precision survives the
10^5-10^6 s host-uptime ref_sec magnitudes.
"""
from __future__ import annotations

import itertools
import json
import sys
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

_SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPT_DIR))
sys.path.insert(0, str(_SCRIPT_DIR.parent / 'tools'))

# Reused analysis cores — the whole point is ONE implementation of each.
from plot_xhost_virtual import (  # noqa: E402
    Sample, load_ticc_cha, virtual_diff, detrend_signed)
from plot_clock_stability_stack import load_chA_phase  # noqa: E402
from plot_stability_slide import dev_with_err  # noqa: E402

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:  # pragma: no cover
    tomllib = None

_PS_PER_S = 10 ** 12

# Default moonshot excursion bounds (docs/two-site-sync-budget.md).
SHARED_NS_DEFAULT = 1.0     # same antenna group (shared RF via splitter)
SEPARATE_NS_DEFAULT = 2.0   # different antenna groups (≤ 5 km baseline)

# Reporting τ grid (s) for the per-clock TDEV summary table.
REPORT_TAUS = (1.0, 10.0, 100.0)

# Data-driven τ-reach caps (same convention as two_clock_report): every
# plotted point rests on ≥ this many independent intervals.
ADEV_MIN_INTERVALS = 4     # ADEV τ_max = N/4  (classic T/4 cap)
TDEV_MIN_INTERVALS = 10    # TDEV runs off-scale sooner


# --------------------------------------------------------------------------
# Host model
# --------------------------------------------------------------------------
@dataclass
class HostSpec:
    """One disciplined clock in the comparison.

    ``csv`` + ``channel`` locate the clock's PPS-vs-Rb stream (multiple
    hosts may share one csv on different channels — e.g. MadHat ticc1 with
    otcBob1=chA and ptBoat=chB).  ``group`` is the antenna group; pairs
    within a group are graded against the shared-antenna bound, pairs across
    groups against the separate-antenna bound.

    ``stability_only`` marks a clock that is included for its per-clock
    TDEV/ADEV (stability is a property of its OWN trace, time-independent)
    but is NOT contemporaneous with the live fleet — e.g. a historical run
    from a different time window.  Such a clock still appears in the
    stability overlay, but every excursion pair involving it is N/A (there
    is no shared time window to difference).  Zero time overlap is ALSO
    auto-detected and N/A'd even without this flag; the flag lets the
    operator declare the intent explicitly (and skips the pointless pairing
    attempt).
    """
    label: str
    csv: Path
    channel: str = 'chA'
    group: str = 'default'
    stability_only: bool = False


_STABILITY_TOKENS = frozenset(
    {'stability_only', 'stability', 'stab', 'so', 'true', '1'})


def parse_host_spec(s: str) -> HostSpec:
    """Parse ``LABEL=CSV[:CHANNEL][:GROUP][:FLAG]``.

    CHANNEL defaults to ``chA``; GROUP defaults to ``default``.  A field may
    be left empty to keep its default while setting a later one, e.g.
    ``PiPuss=pipuss.csv::ufoLondon3`` (empty channel → chA).  The optional
    4th FLAG field marks a stability-only (non-contemporaneous) clock; accepted
    values: ``stability`` / ``stability_only`` / ``stab`` / ``so`` (e.g.
    ``OldGNSSDO=hist.csv:chA:ufoLondon1:stability``).
    """
    if '=' not in s:
        raise ValueError(
            f'--host spec {s!r} missing "=": expected '
            f'LABEL=CSV[:CHANNEL][:GROUP][:FLAG]')
    label, rest = s.split('=', 1)
    label = label.strip()
    if not label:
        raise ValueError(f'--host spec {s!r} has an empty label')
    parts = rest.split(':')
    csv = Path(parts[0]).expanduser()
    channel = parts[1].strip() if len(parts) >= 2 and parts[1].strip() else 'chA'
    group = parts[2].strip() if len(parts) >= 3 and parts[2].strip() else 'default'
    stability_only = False
    if len(parts) >= 4 and parts[3].strip():
        tok = parts[3].strip().lower()
        if tok not in _STABILITY_TOKENS:
            raise ValueError(
                f'--host spec {s!r}: unknown 4th field {parts[3]!r} '
                f'(expected a stability flag like "stability")')
        stability_only = True
    return HostSpec(label, csv, channel, group, stability_only)


def load_manifest(path: Path) -> tuple[list[HostSpec], dict]:
    """Load a host list + optional bounds from a TOML or JSON manifest.

    Accepted shapes (TOML shown; JSON mirrors it):

        shared_ns = 1.0            # optional
        separate_ns = 2.0          # optional
        [[host]]                   # or a top-level "hosts" array in JSON
        label = "PiFace"
        csv = "data/piface-...csv"
        channel = "chA"            # optional, default chA
        group = "ufoLondon1"       # optional, default "default"

    Returns ``(hosts, meta)`` where ``meta`` may carry ``shared_ns`` /
    ``separate_ns`` overrides.
    """
    path = Path(path)
    text = path.read_text()
    if path.suffix.lower() == '.json':
        data = json.loads(text)
    elif path.suffix.lower() in ('.toml', ''):
        if tomllib is None:  # pragma: no cover
            raise RuntimeError('tomllib unavailable; use a .json manifest')
        data = tomllib.loads(text)
    else:
        raise ValueError(f'unknown manifest type {path.suffix!r} (use .toml/.json)')

    raw_hosts = data.get('host') or data.get('hosts') or []
    if not raw_hosts:
        raise ValueError(f'{path}: no [[host]] / "hosts" entries found')
    base = path.parent
    hosts: list[HostSpec] = []
    for h in raw_hosts:
        csv = Path(h['csv']).expanduser()
        if not csv.is_absolute():
            csv = (base / csv)
        hosts.append(HostSpec(
            label=str(h['label']),
            csv=csv,
            channel=str(h.get('channel', 'chA')),
            group=str(h.get('group', 'default')),
            stability_only=bool(h.get('stability_only', False)),
        ))
    meta = {k: data[k] for k in ('shared_ns', 'separate_ns') if k in data}
    return hosts, meta


# --------------------------------------------------------------------------
# CSV windowing (skip-before / skip-after) — shared with two_clock_report
# --------------------------------------------------------------------------
def parse_iso(s: str) -> datetime:
    """Parse an ISO-8601 UTC timestamp to an offset-AWARE datetime (UTC when
    no offset is given)."""
    dt = datetime.fromisoformat(s.replace('Z', '+00:00'))
    return dt if dt.tzinfo is not None else dt.replace(tzinfo=timezone.utc)


def window_csv(src: Path, dst: Path, skip_before: datetime | None,
               skip_after: datetime | None) -> int:
    """Copy ``src`` → ``dst`` keeping only rows whose timestamp is in
    [skip_before, skip_after), preserving header/comment lines.  Returns the
    number of data rows kept.  (Behaviour matches two_clock_report._window_csv.)
    """
    from analysis_provenance import skip_comment_lines  # local: cheap
    kept = 0
    with open(src) as fin, open(dst, 'w') as fout:
        header_seen = False
        ts_idx = None
        for line in fin:
            s = line.rstrip('\n')
            if s.startswith('#'):
                fout.write(line)
                continue
            if not header_seen:
                cols = s.split(',')
                for cand in ('ts_iso', 'host_timestamp'):
                    if cand in cols:
                        ts_idx = cols.index(cand)
                        break
                fout.write(line)
                header_seen = True
                continue
            cols = s.split(',')
            if ts_idx is not None and ts_idx < len(cols):
                try:
                    ts = parse_iso(cols[ts_idx])
                except ValueError:
                    continue
                if skip_before is not None and ts < skip_before:
                    continue
                if skip_after is not None and ts >= skip_after:
                    continue
            fout.write(line)
            kept += 1
    return kept


def resolve_windows(csvs, skip_before, skip_after, work_dir) -> dict:
    """Materialize a windowed copy of each unique CSV (once) when a
    skip-before/after is set.  Returns ``{orig_path: path_to_use}``.

    Windowing each unique file ONCE and reusing it for every channel on it
    guarantees all clocks (and all pages) see identical epochs.
    """
    uniq = {Path(c).resolve(): Path(c) for c in csvs}
    if skip_before is None and skip_after is None:
        return {orig: orig for orig in uniq.values()}
    work_dir = Path(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)
    out: dict = {}
    for i, (res, orig) in enumerate(uniq.items()):
        dst = work_dir / f'{orig.stem}.win{i}.csv'
        window_csv(orig, dst, skip_before, skip_after)
        out[orig] = dst
    return out


# --------------------------------------------------------------------------
# Per-host stability (chX vs the shared Rb)
# --------------------------------------------------------------------------
def octave_taus(n: int, min_intervals: int) -> np.ndarray:
    """1-2-5-per-decade τ grid (s) up to a data-driven τ_max = n/min_intervals
    so every point has ≥ min_intervals independent intervals."""
    tau_max = n / float(max(1, min_intervals))
    out, d = [], 1
    while d <= tau_max:
        for m in (1, 2, 5):
            v = float(m * d)
            if 1.0 <= v <= tau_max:
                out.append(v)
        d *= 10
    return np.array(out if out else [1.0])


@dataclass
class HostResult:
    spec: HostSpec
    n: int
    tdev: dict = field(default_factory=dict)   # {tau_s: ns}
    adev: dict = field(default_factory=dict)   # {tau_s: dimensionless}
    phase_ns: np.ndarray = field(default_factory=lambda: np.array([]))
    n_outliers: int = 0


def reject_gross_outliers(phase_ns, k_mad: float = 25.0,
                          floor_ns: float = 200.0):
    """Drop gross per-sample outliers from a detrended phase series before TDEV.

    A glitching / boundary-straddling PPS (e.g. ptBoat's ~500 ms edge
    excursions, 2026-07-10 xhost overnight) injects a handful of samples
    orders of magnitude beyond the clock's real noise.  TDEV is variance-based,
    so those destroy it — while the p95 excursion is unaffected (robust to a
    <0.1 % tail).  Flag samples where ``|x - median| > max(k_mad*1.4826*MAD,
    floor_ns)`` and linear-interpolate over them so the evenly-sampled series
    stays gap-free.  The floor keeps a genuinely quiet clock (tiny MAD) from
    rejecting its own real ns-scale noise; only truly gross (>~200 ns single-
    sample) excursions go.  Returns ``(clean_phase, n_dropped)``.
    """
    x = np.asarray(phase_ns, dtype=float)
    if x.size < 8:
        return x, 0
    med = float(np.median(x))
    mad = float(np.median(np.abs(x - med)))
    thresh = max(k_mad * 1.4826 * mad, floor_ns)
    bad = np.abs(x - med) > thresh
    n = int(bad.sum())
    # Never blank out (nearly) the whole series — that would signal a bad
    # threshold, not outliers; leave the data untouched in that pathological case.
    if n == 0 or n >= x.size - 2:
        return x, 0
    idx = np.arange(x.size)
    clean = x.copy()
    clean[bad] = np.interp(idx[bad], idx[~bad], x[~bad])
    return clean, n


def host_stability(phase_ns: np.ndarray) -> tuple[dict, dict]:
    """TDEV(ns) and ADEV maps for a detrended phase series, on data-driven
    grids (reuses ``dev_with_err`` — the campaign TDEV/ADEV core)."""
    n = len(phase_ns)
    if n < 60:
        return {}, {}
    grid_t = octave_taus(n, TDEV_MIN_INTERVALS)
    grid_a = octave_taus(n, ADEV_MIN_INTERVALS)
    tt, td, _ = dev_with_err(phase_ns, 'tdev', grid_t)
    ta, da, _ = dev_with_err(phase_ns, 'adev', grid_a)
    return ({float(t): float(v) for t, v in zip(tt, td)},
            {float(t): float(v) for t, v in zip(ta, da)})


# --------------------------------------------------------------------------
# All-pairs virtual cross-clock Δ + budget grading
# --------------------------------------------------------------------------
def pair_abs_delta_ns(a_samples: list, b_samples: list) -> np.ndarray:
    """|Δ| (ns) of the virtual cross-clock difference, constant-offset +
    linear-drift removed.  Reuses ``virtual_diff`` (UTC-grid pairing, exact
    int subtract) + ``detrend_signed`` (removes cable/boot offset + rate)."""
    times, delta_ps = virtual_diff(a_samples, b_samples)
    if not times:
        return np.array([])
    _, dev_dt_ps, _ = detrend_signed(times, delta_ps)
    return np.abs(dev_dt_ps) * 1e-3


@dataclass
class PairResult:
    a: str
    b: str
    group_a: str
    group_b: str
    n: int
    p50: float          # ns
    p95: float          # ns
    p99: float          # ns
    max: float          # ns
    rms: float          # ns
    bound_ns: float
    shared: bool        # same antenna group?
    grade: str          # 'PASS' | 'FAIL' | 'NODATA' | 'NA'


# Grades that mean "no gradeable excursion here" — rendered as N/A.
NA_GRADES = frozenset({'NODATA', 'NA'})


def na_pair(a: HostSpec, b: HostSpec, shared_ns: float, separate_ns: float,
            grade: str = 'NA') -> PairResult:
    """A pair with no contemporaneous window (a stability-only clock, or zero
    time overlap) — N/A, never a crash and never a spurious 0."""
    shared = (a.group == b.group)
    bound = shared_ns if shared else separate_ns
    nan = float('nan')
    return PairResult(a.label, b.label, a.group, b.group, 0,
                      nan, nan, nan, nan, nan, bound, shared, grade)


def grade_pair(a: HostSpec, b: HostSpec, abs_ns: np.ndarray,
               shared_ns: float, separate_ns: float,
               min_samples: int = 60) -> PairResult:
    shared = (a.group == b.group)
    bound = shared_ns if shared else separate_ns
    n = len(abs_ns)
    if n < min_samples:
        # Too few (or zero) overlapping epochs → auto-N/A, not a failure.
        return na_pair(a, b, shared_ns, separate_ns, grade='NODATA')
    p50 = float(np.percentile(abs_ns, 50))
    p95 = float(np.percentile(abs_ns, 95))
    p99 = float(np.percentile(abs_ns, 99))
    mx = float(np.max(abs_ns))
    rms = float(np.sqrt(np.mean(abs_ns ** 2)))
    grade = 'PASS' if p95 <= bound else 'FAIL'
    return PairResult(a.label, b.label, a.group, b.group, n,
                      p50, p95, p99, mx, rms, bound, shared, grade)


# --------------------------------------------------------------------------
# N-cornered hat — each clock's OWN stability past the shared-Rb floor
# --------------------------------------------------------------------------
def _aligned_run(a_samples: list, b_samples: list
                 ) -> tuple[np.ndarray, np.ndarray]:
    """Two clocks' phase (ns) over their longest common run of consecutive
    UTC seconds, each as deviation-from-1 Hz (subtract-separately int math).

    Pairing key = rounded UTC second (host_timestamp); works both within one
    TICC (chA/chB) and across TICCs (shared Rb, different boot epoch).
    """
    def _by_sec(samples):
        m = {}
        for s in samples:
            m[int(round(s.ts.timestamp()))] = s.total_ps
        return m

    ma, mb = _by_sec(a_samples), _by_sec(b_samples)
    common = sorted(set(ma) & set(mb))
    if len(common) < 60:
        return np.array([]), np.array([])
    secs = np.array(common)
    diffs = np.diff(secs)
    bounds = [0] + list(np.where(diffs != 1)[0] + 1) + [len(common)]
    segs = [(bounds[i], bounds[i + 1]) for i in range(len(bounds) - 1)]
    lo, hi = max(segs, key=lambda ab: ab[1] - ab[0])
    seg = common[lo:hi]
    if len(seg) < 60:
        return np.array([]), np.array([])
    s0 = seg[0]
    a0, b0 = ma[s0], mb[s0]
    pa = np.array([(ma[s] - a0) - (s - s0) * _PS_PER_S for s in seg],
                  dtype=np.int64).astype(np.float64) * 1e-3
    pb = np.array([(mb[s] - b0) - (s - s0) * _PS_PER_S for s in seg],
                  dtype=np.int64).astype(np.float64) * 1e-3
    return pa, pb


def _avar_map(phase_ns: np.ndarray, taus: np.ndarray, modified: bool) -> dict:
    """{tau_s: Allan (or modified-Allan) variance in s²} for the feasible τ."""
    import allantools
    n = len(phase_ns)
    feasible = taus[taus * 3 < n]
    if not len(feasible):
        return {}
    fn = allantools.mdev if modified else allantools.adev
    t_out, dev, _, _ = fn(phase_ns * 1e-9, rate=1.0, taus=feasible,
                          data_type='phase')
    return {float(t): float(d * d) for t, d in zip(t_out, dev)}


def n_corner_hat(samples_by_label: dict, taus: np.ndarray
                 ) -> tuple[dict, int]:
    """Averaged pairwise three-cornered hat over the whole fleet.

    All clocks share ONE Rb, and we measure every clock-vs-Rb leg directly
    (chX-vs-Rb) plus every clock-vs-clock leg (virtual diff, Rb cancels).
    For clock i, using ANY other clock j as the third corner:

        σ²_i(τ) = ½ ( AVAR(d_i − d_j) + AVAR(d_i) − AVAR(d_j) )

    where d_k = clock_k − Rb.  The shared Rb cancels out of this estimator
    (it appears in AVAR(d_i) and AVAR(d_j) with equal weight and is removed
    by the difference), so the recovered σ²_i is the clock's OWN variance
    plus only its channel's TDC noise — NOT the Rb random walk that the raw
    chX-vs-Rb curve carries.  Averaging the estimate over all j≠i is the
    N-cornered ("multi-reference") generalization: more references → lower
    estimator variance.

    Independence caveat: assumes the clocks are mutually uncorrelated.  Two
    GNSS-locked clocks on ONE shared antenna share correction/atmospheric
    noise, which biases the split — flagged by the caller.

    Returns ``({label: {'taus', 'adev', 'tdev_ns'}}, neg_count)``.  A τ whose
    solved variance is negative (independence violated / too few samples) is
    set NaN and counted.  A clock with no contemporaneous partner (empty or
    non-overlapping samples) simply gets no estimates and is omitted from the
    output — it cannot be placed in the fleet hat, only in the per-clock
    vs-Rb overlay.
    """
    labels = [lab for lab in samples_by_label
              if len(samples_by_label[lab]) >= 60]
    # Accumulators: est[label][modified][tau] -> list of variance estimates.
    est = {lab: {False: {}, True: {}} for lab in labels}
    neg = 0
    for i, j in itertools.combinations(labels, 2):
        pa, pb = _aligned_run(samples_by_label[i], samples_by_label[j])
        if not len(pa):
            continue
        d_ij = pa - pb
        for modified in (False, True):
            va = _avar_map(pa, taus, modified)
            vb = _avar_map(pb, taus, modified)
            vij = _avar_map(d_ij, taus, modified)
            common_t = set(va) & set(vb) & set(vij)
            for t in common_t:
                s2_i = 0.5 * (vij[t] + va[t] - vb[t])
                s2_j = 0.5 * (vij[t] + vb[t] - va[t])
                for lab, s2 in ((i, s2_i), (j, s2_j)):
                    if s2 < 0:
                        neg += 1
                        continue
                    est[lab][modified].setdefault(t, []).append(s2)

    out: dict = {}
    for lab in labels:
        if not est[lab][False] and not est[lab][True]:
            continue   # no contemporaneous partner → can't place in the hat
        taus_a = sorted(est[lab][False])
        taus_t = sorted(est[lab][True])
        adev = np.array([np.sqrt(np.mean(est[lab][False][t])) for t in taus_a])
        var_t = np.array([np.mean(est[lab][True][t]) for t in taus_t])
        tdev_ns = (np.array(taus_t) * np.sqrt(var_t) / np.sqrt(3.0)) * 1e9
        out[lab] = {
            'taus_adev': np.array(taus_a, dtype=float),
            'adev': adev,
            'taus_tdev': np.array(taus_t, dtype=float),
            'tdev_ns': tdev_ns,
        }
    return out, neg


# --------------------------------------------------------------------------
# Top-level analyze() — the orchestrator the composer + tests call
# --------------------------------------------------------------------------
@dataclass
class AnalysisResult:
    hosts: dict            # {label: HostResult}
    pairs: list            # [PairResult]
    hat: dict              # {label: {taus_*, adev, tdev_ns}}
    hat_neg: int
    shared_ns: float
    separate_ns: float
    order: list            # labels in input order


def analyze(host_specs: list, shared_ns: float = SHARED_NS_DEFAULT,
            separate_ns: float = SEPARATE_NS_DEFAULT,
            skip_before: datetime | None = None,
            skip_after: datetime | None = None,
            stability_skip_before: datetime | None = None,
            work_dir: Path | None = None,
            do_hat: bool = True) -> AnalysisResult:
    """Run the full N-clock analysis: per-host TDEV/ADEV vs Rb, all-pairs
    graded excursion, and (optionally) the N-cornered hat.

    Two INDEPENDENT time axes, deliberately decoupled:

    * ``skip_before`` / ``skip_after`` define the EXCURSION window — the span
      over which the clocks are jointly comparable (e.g. from when the last
      live clock came up + settled).  Applied to the all-pairs excursion and
      the N-cornered hat (both are cross-clock, contemporaneity-dependent).
    * per-clock TDEV/ADEV uses each clock's FULL trace (stability is a
      property of the clock's own trace, independent of when the others were
      up), optionally trimmed by ``stability_skip_before`` to drop a clock's
      own bootstrap transient.

    So a historical / non-contemporaneous clock still contributes its
    stability curve while every excursion pair involving it is N/A.

    Pure compute — no plotting — so it is unit-testable directly.
    """
    if len(host_specs) < 2:
        raise ValueError('need at least 2 hosts to compare')
    labels = [h.label for h in host_specs]
    if len(set(labels)) != len(labels):
        raise ValueError(f'duplicate host labels: {labels}')

    wd = Path(work_dir) if work_dir else Path('.')
    # Window only the csvs of contemporaneous (non-stability-only) clocks;
    # stability-only clocks never enter the excursion pairing.
    pairing_csvs = [h.csv for h in host_specs if not h.stability_only]
    resolved = resolve_windows(pairing_csvs, skip_before, skip_after, wd)

    hosts: dict = {}
    samples_by_label: dict = {}
    for h in host_specs:
        # Per-clock stability: FULL trace (excursion window does NOT apply).
        # A stability-only clock is non-contemporaneous by definition, so the
        # live-window stability skip must NOT touch it — else it trims the
        # historical trace to nothing (n=0).
        sb = None if h.stability_only else stability_skip_before
        phase = load_chA_phase(h.csv, sb, channel=h.channel)
        phase, n_out = reject_gross_outliers(phase)
        tdev, adev = host_stability(phase)
        hosts[h.label] = HostResult(spec=h, n=len(phase), tdev=tdev,
                                    adev=adev, phase_ns=phase, n_outliers=n_out)
        # Excursion samples: windowed; stability-only clocks are excluded.
        if h.stability_only:
            samples_by_label[h.label] = []
        else:
            path = resolved.get(h.csv, h.csv)
            samples_by_label[h.label] = load_ticc_cha(path, None,
                                                      channel=h.channel)

    pairs: list = []
    for a, b in itertools.combinations(host_specs, 2):
        if a.stability_only or b.stability_only:
            pairs.append(na_pair(a, b, shared_ns, separate_ns))
            continue
        abs_ns = pair_abs_delta_ns(samples_by_label[a.label],
                                   samples_by_label[b.label])
        pairs.append(grade_pair(a, b, abs_ns, shared_ns, separate_ns))

    hat, hat_neg = ({}, 0)
    if do_hat:
        # Widest candidate grid; feasibility filtered per pair inside.
        cand = octave_taus(10 ** 7, 1)
        hat, hat_neg = n_corner_hat(samples_by_label, cand)

    return AnalysisResult(hosts=hosts, pairs=pairs, hat=hat, hat_neg=hat_neg,
                          shared_ns=shared_ns, separate_ns=separate_ns,
                          order=labels)


def p95_matrix(result: AnalysisResult) -> tuple[np.ndarray, np.ndarray, list]:
    """Build symmetric N×N matrices of (p95 |Δ| ns, bound ns) from the graded
    pairs.  Diagonal is NaN.  Returns ``(p95, bound, labels)``."""
    labels = result.order
    idx = {lab: i for i, lab in enumerate(labels)}
    n = len(labels)
    p95 = np.full((n, n), np.nan)
    bound = np.full((n, n), np.nan)
    for pr in result.pairs:
        i, j = idx[pr.a], idx[pr.b]
        p95[i, j] = p95[j, i] = pr.p95
        bound[i, j] = bound[j, i] = pr.bound_ns
    return p95, bound, labels

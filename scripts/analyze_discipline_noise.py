#!/usr/bin/env python3
"""Retrospectively attribute disciplined-clock phase noise to the servo.

Free-run characterization (do_freerun_char.py) measures the DO's noise
over QUIET intervals (DAC parked).  This tool answers the complementary
question: of the excess phase noise a *disciplined* run shows above that
floor, how much is injected AROUND the epochs where the discipline loop
acts, versus DO noise that is merely exposed between corrections?

Inputs are two retrospective logs from a live run, correlated by their
shared CLOCK_MONOTONIC stamp (the only reliable cross-stream timescale —
see docs/stream-timescale-correlation.md):

  --ticc       TICC chA log (DO PPS vs the TICC's Rb), 60 ps resolution.
  --arm-state  per-servo.update() log carrying x3_f_do_ppb, dt_actual_s,
               and ocxo_gate_reject — the discipline-action markers.

Method
------
1.  Reconstruct chA phase (ns) in integer-ps math, robust to gaps and
    TICC resets, then linear-detrend (remove constant freq offset).
2.  Resample onto a uniform 1 s grid (so τ scaling is well defined).
3.  Mark "discipline events" — by default the gate-ACCEPT epochs
    (ocxo_gate_reject==0): the moments a measurement actually moved the
    loop.  (--event dx3 instead marks |Δx3| steps.)
4.  Split grid seconds into NEAR (within --window of any event) vs QUIET.
5.  Compare class-conditional TDEV(τ=1 s) (the local second-difference
    estimator, identical to InBandNoiseEstimator) for NEAR vs QUIET, and
    the full-run TDEV curve against the free-run floor.
6.  Per-event: phase KINK (discontinuity across the event) and frequency
    step — does the loop inject a glitch, and did the actuator deliver
    the step the filter intended?

If NEAR >> QUIET (or kinks are large), discipline is injecting the noise.
If NEAR ~= QUIET while both >> floor, the loop is merely exposing a DO
short-τ noise source the free-run characterization underestimated.
"""

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

PS_PER_S = 1_000_000_000_000


# ── TICC chA phase reconstruction ──────────────────────────────────── #

def load_ticc_cha(path):
    """Return (mono[N], phase_ns[N]) for chA, gap/reset robust.

    Phase is the DO PPS time error vs the TICC Rb: integer-ps math with
    the nominal 1 Hz removed via round((ref - ref0)/1s) = pulse index.
    """
    mono, sec, ps = [], [], []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            if row.get('channel') != 'chA':
                continue
            try:
                mono.append(float(row['host_monotonic']))
                sec.append(int(row['ref_sec']))
                ps.append(int(row['ref_ps']))
            except (ValueError, KeyError):
                continue
    if len(sec) < 8:
        raise SystemExit(f"too few chA samples in {path} ({len(sec)})")
    mono = np.asarray(mono)
    sec = np.asarray(sec, dtype=np.int64)
    ps = np.asarray(ps, dtype=np.int64)

    # Drop everything before the last TICC reset (ref_sec jumps back).
    back = np.where(np.diff(sec) < -10)[0]
    if len(back):
        cut = back[-1] + 1
        print(f"  TICC reset: dropping {cut} stale chA edges")
        mono, sec, ps = mono[cut:], sec[cut:], ps[cut:]

    # total_ps relative to first sample (Python ints → no precision loss).
    total_ps = (sec - sec[0]).astype(object) * PS_PER_S + (ps - ps[0]).astype(object)
    total_ps = np.array([int(v) for v in total_ps], dtype=np.int64)
    n_pulse = np.round(total_ps / PS_PER_S).astype(np.int64)
    resid_ps = total_ps - n_pulse * PS_PER_S
    phase_ns = resid_ps.astype(float) * 1e-3

    # Linear-detrend vs monotonic time (remove residual frequency offset).
    slope, intercept = np.polyfit(mono - mono[0], phase_ns, 1)
    phase_ns = phase_ns - (slope * (mono - mono[0]) + intercept)
    print(f"  chA: {len(phase_ns)} samples, residual freq offset "
          f"{slope:+.4f} ppb, RMS {np.sqrt(np.mean(phase_ns**2)):.3f} ns")
    return mono, phase_ns


# ── arm-state discipline markers ───────────────────────────────────── #

def load_arm_events(path, event_kind, dx3_thr):
    """Return (event_mono[], x3_mono[], x3[], gate_reject_frac).

    event_mono: monotonic stamps of discipline events.
    x3_mono/x3: the full commanded-frequency trace (for Δx3 stats).
    """
    mono, x3, gate = [], [], []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            try:
                m = float(row['host_monotonic'])
            except (ValueError, KeyError):
                continue
            gx = row.get('ocxo_gate_reject', '')
            xv = row.get('x3_f_do_ppb', '')
            mono.append(m)
            x3.append(float(xv) if xv not in ('', None) else math.nan)
            gate.append(gx)
    mono = np.asarray(mono)
    x3 = np.asarray(x3)
    gi = np.array([1 if g == '1' else (0 if g == '0' else -1) for g in gate])
    seen = gi >= 0
    reject_frac = float((gi[seen] == 1).mean()) if seen.any() else float('nan')

    if event_kind == 'gate-accept':
        ev = mono[gi == 0]
    elif event_kind == 'gate-reject':
        ev = mono[gi == 1]
    else:  # dx3: commanded-frequency steps
        dx3 = np.abs(np.diff(x3))
        idx = np.where(dx3 > dx3_thr)[0] + 1
        ev = mono[idx]
    return ev, mono, x3, reject_frac


# ── uniform 1 s grid + class masks ─────────────────────────────────── #

def to_grid(mono, phase_ns):
    """Average chA phase into a uniform 1 s grid. NaN where no sample."""
    sec = np.round(mono - mono[0]).astype(int)
    T = int(sec.max()) + 1
    s = np.zeros(T)
    c = np.zeros(T)
    np.add.at(s, sec, phase_ns)
    np.add.at(c, sec, 1.0)
    grid = np.where(c > 0, s / np.maximum(c, 1), np.nan)
    return grid, mono[0]


def near_mask(event_mono, mono0, T, window):
    """Boolean grid mask: True for seconds within `window` of any event."""
    m = np.zeros(T, dtype=bool)
    if len(event_mono) == 0:
        return m
    esec = np.round(event_mono - mono0).astype(int)
    esec = esec[(esec >= 0) & (esec < T)]
    for e in esec:
        lo, hi = max(0, e - window), min(T, e + window + 1)
        m[lo:hi] = True
    return m


# ── second-difference TDEV(τ) on a (possibly masked) grid ──────────── #

def tdev_2nd_diff(grid, n, center_mask=None):
    """TDEV at τ = n·1s via the local 2nd-difference estimator.

    d2[i] = x[i+2n] - 2 x[i+n] + x[i]; TDEV = sqrt(<d2²>/6).  For n=1 this
    is identical to InBandNoiseEstimator's τ0 estimator.  Triples with any
    NaN member are skipped; if center_mask is given, only triples whose
    center (i+n) is True are counted — yielding a class-conditional TDEV.
    """
    T = len(grid)
    if T < 3 * n + 1:
        return math.nan, 0
    i = np.arange(T - 2 * n)
    a, b, c = grid[i], grid[i + n], grid[i + 2 * n]
    d2 = c - 2 * b + a
    ok = ~(np.isnan(a) | np.isnan(b) | np.isnan(c))
    if center_mask is not None:
        ok &= center_mask[i + n]
    d2 = d2[ok]
    if len(d2) < 8:
        return math.nan, len(d2)
    return math.sqrt(np.mean(d2 ** 2) / 6.0), len(d2)


def full_tdev_curve(grid, taus):
    """Proper allantools TDEV over the gap-filled grid (for floor compare)."""
    try:
        import allantools
    except ImportError:
        return {}
    x = grid.copy()
    if np.isnan(x).any():  # linear-interpolate gaps for the curve only
        idx = np.arange(len(x))
        good = ~np.isnan(x)
        x = np.interp(idx, idx[good], x[good])
    valid = [t for t in taus if t < len(x) / 4]
    if not valid:
        return {}
    ta, td, _, _ = allantools.tdev(x * 1e-9, rate=1.0, data_type='phase',
                                   taus=np.array(valid, dtype=float))
    return {float(t): float(v) * 1e9 for t, v in zip(ta, td)}


# ── per-event kink + frequency step ────────────────────────────────── #

def event_kinks(grid, event_mono, mono0, window):
    """For each event: phase discontinuity (ns) and freq step (ppb)."""
    T = len(grid)
    esec = np.round(event_mono - mono0).astype(int)
    esec = esec[(esec >= window) & (esec < T - window)]
    kinks, fsteps = [], []
    for e in esec:
        pre_i = np.arange(e - window, e)
        post_i = np.arange(e + 1, e + window + 1)
        pre = grid[pre_i]
        post = grid[post_i]
        pm, qm = ~np.isnan(pre), ~np.isnan(post)
        if pm.sum() < 3 or qm.sum() < 3:
            continue
        sp, ip = np.polyfit(pre_i[pm] - e, pre[pm], 1)
        sq, iq = np.polyfit(post_i[qm] - e, post[qm], 1)
        kinks.append(iq - ip)        # phase gap extrapolated to event (ns)
        fsteps.append(sq - sp)        # slope change (ns/s == ppb)
    return np.asarray(kinks), np.asarray(fsteps)


def load_floor(path):
    if not path or not Path(path).exists():
        return {}
    try:
        data = json.loads(Path(path).read_text())
        src = data.get('characterization', {}).get('sources', {})
        for key in ('DO PPS (chA vs TICC Rb)', 'DO PPS (chA-chB)'):
            if key in src:
                return {float(k): v for k, v
                        in src[key].get('tdev_ns_by_tau_s', {}).items()}
        if src:  # any source
            first = next(iter(src.values()))
            return {float(k): v for k, v
                    in first.get('tdev_ns_by_tau_s', {}).items()}
    except (json.JSONDecodeError, OSError, StopIteration):
        pass
    return {}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ticc', required=True, help='TICC chA CSV')
    ap.add_argument('--arm-state', required=True, help='arm-state CSV')
    ap.add_argument('--freerun', default=None, help='DO-state JSON for floor')
    ap.add_argument('--event', choices=['gate-accept', 'gate-reject', 'dx3'],
                    default='gate-accept', help='discipline-event marker')
    ap.add_argument('--dx3-thr', type=float, default=0.5,
                    help='ppb |Δx3| threshold for --event dx3')
    ap.add_argument('--window', type=int, default=8,
                    help='± seconds around each event = NEAR window')
    ap.add_argument('--skip-s', type=float, default=0.0,
                    help='drop the first N s (e.g. acquisition transient) '
                         'so TDEV reflects the locked steady-state')
    ap.add_argument('--out', default=None, help='PNG output path')
    ap.add_argument('--label', default='clkPoC3', help='plot title label')
    args = ap.parse_args()

    print(f"Loading TICC chA from {args.ticc}")
    mono, phase = load_ticc_cha(args.ticc)
    print(f"Loading arm-state from {args.arm_state}")
    ev, am, x3, reject_frac = load_arm_events(args.arm_state, args.event,
                                              args.dx3_thr)

    if args.skip_s > 0:
        t0 = mono[0]
        keep = (mono - t0) >= args.skip_s
        mono, phase = mono[keep], phase[keep]
        ev = ev[(ev - t0) >= args.skip_s]
        print(f"  skipped first {args.skip_s:.0f}s -> {len(mono)} chA samples remain")

    grid, mono0 = to_grid(mono, phase)
    T = len(grid)
    have = np.isfinite(grid)
    near = near_mask(ev, mono0, T, args.window)
    quiet = ~near
    near_have = near & have
    quiet_have = quiet & have

    taus = [1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024]
    floor = load_floor(args.freerun)
    full = full_tdev_curve(grid, taus)

    # class-conditional TDEV at small τ
    rows = []
    for n in (1, 2, 4):
        tn, cn = tdev_2nd_diff(grid, n, center_mask=near)
        tq, cq = tdev_2nd_diff(grid, n, center_mask=quiet)
        ta, ca = tdev_2nd_diff(grid, n, center_mask=None)
        rows.append((n, tn, cn, tq, cq, ta, ca))

    kinks, fsteps = event_kinks(grid, ev, mono0, args.window)

    # ── report ──────────────────────────────────────────────────────
    print("\n" + "=" * 68)
    print(f"DISCIPLINE-NOISE ATTRIBUTION — {args.label}")
    print("=" * 68)
    dur_min = T / 60.0
    print(f"  run length (grid):     {T} s  ({dur_min:.1f} min)")
    print(f"  chA grid fill:         {have.mean()*100:.1f}%  ({have.sum()} s)")
    print(f"  gate reject fraction:  {reject_frac*100:.1f}%")
    print(f"  event marker:          {args.event}  (n_events={len(ev)})")
    print(f"  NEAR window:           +/-{args.window} s")
    print(f"  NEAR / QUIET seconds:  {near_have.sum()} / {quiet_have.sum()}")
    if floor:
        f1 = floor.get(1.0)
        print(f"  free-run floor TDEV(1s): "
              f"{f1*1000:.1f} ps" if f1 else "  free-run floor: n/a")

    print("\n  TDEV (2nd-diff estimator), ns:")
    print(f"  {'tau':>5}  {'NEAR':>10} {'(n)':>7}  {'QUIET':>10} {'(n)':>7}  "
          f"{'ALL':>10}  {'floor':>9}")
    for n, tn, cn, tq, cq, ta, ca in rows:
        fl = floor.get(float(n))
        print(f"  {n:>4}s  {tn:>10.4f} {cn:>7d}  {tq:>10.4f} {cq:>7d}  "
              f"{ta:>10.4f}  {('%.4f'%fl) if fl else 'n/a':>9}")

    if len(kinks):
        print(f"\n  per-event phase KINK (ns):  n={len(kinks)}  "
              f"mean={kinks.mean():+.3f}  RMS={np.sqrt(np.mean(kinks**2)):.3f}  "
              f"|max|={np.abs(kinks).max():.3f}")
        print(f"  per-event FREQ step (ppb):  "
              f"mean={fsteps.mean():+.4f}  RMS={np.sqrt(np.mean(fsteps**2)):.4f}")

    # verdict heuristic
    t1n, t1q = rows[0][1], rows[0][3]
    print("\n  VERDICT:")
    if math.isfinite(t1n) and math.isfinite(t1q):
        ratio = t1n / t1q if t1q > 0 else float('inf')
        print(f"    NEAR/QUIET TDEV(1s) ratio = {ratio:.2f}")
        if ratio > 1.5:
            print("    -> discipline INJECTS short-tau noise around events.")
        else:
            print("    -> NEAR ~= QUIET: noise is uniform, not event-driven")
            print("       (DO short-tau floor likely underestimated, or every-")
            print("        epoch actuation jitter — not discrete corrections).")
    if floor and math.isfinite(t1q):
        f1 = floor.get(1.0)
        if f1:
            print(f"    QUIET/floor TDEV(1s) ratio = {t1q/f1:.1f}x "
                  f"(quiet {t1q*1000:.0f} ps vs floor {f1*1000:.0f} ps)")

    if args.out:
        _plot(args, grid, mono0, ev, near, full, floor, taus, rows,
              kinks, fsteps, reject_frac)
    return 0


def _plot(args, grid, mono0, ev, near, full, floor, taus, rows,
          kinks, fsteps, reject_frac):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(1, 3, figsize=(18, 5))
    T = len(grid)
    t = np.arange(T)

    # Panel 1: phase residual + events + near shading
    ax = axs[0]
    ax.plot(t, grid, lw=0.5, color='steelblue')
    seg = np.where(near)[0]
    if len(seg):
        ax.scatter(seg, grid[seg], s=2, color='orange',
                   label='NEAR event', zorder=3)
    ax.set_title(f"{args.label}: chA phase residual (detrended)")
    ax.set_xlabel('time (s)'); ax.set_ylabel('phase (ns)')
    ax.legend(loc='upper right', fontsize=8)

    # Panel 2: TDEV curves
    ax = axs[1]
    if full:
        ax.loglog(list(full.keys()), list(full.values()), 'o-',
                  color='black', label='disciplined (full)')
    if floor:
        ax.loglog(list(floor.keys()), list(floor.values()), 's--',
                  color='green', label='free-run floor')
    nt = [r[0] for r in rows]
    ax.loglog(nt, [r[1] for r in rows], '^-', color='red', label='NEAR')
    ax.loglog(nt, [r[3] for r in rows], 'v-', color='blue', label='QUIET')
    ax.set_title('TDEV vs tau')
    ax.set_xlabel('tau (s)'); ax.set_ylabel('TDEV (ns)')
    ax.grid(True, which='both', alpha=0.3)
    ax.legend(fontsize=8)

    # Panel 3: kink histogram
    ax = axs[2]
    if len(kinks):
        ax.hist(kinks, bins=40, color='purple', alpha=0.7)
        ax.set_title(f'per-event phase kink (n={len(kinks)})\n'
                     f'RMS={np.sqrt(np.mean(kinks**2)):.3f} ns')
        ax.set_xlabel('phase kink (ns)'); ax.set_ylabel('count')
    else:
        ax.text(0.5, 0.5, 'no kinks computed', ha='center')

    fig.suptitle(f"Discipline-noise attribution — gate reject "
                 f"{reject_frac*100:.0f}%", fontsize=11)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"\nSaved plot -> {args.out}")


if __name__ == '__main__':
    sys.exit(main())

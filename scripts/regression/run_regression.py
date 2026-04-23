"""End-to-end regression harness for PePPAR Fix's PPP pipeline.

Threads RINEX OBS + RINEX NAV (+ optional Bias-SINEX OSB) through
`PPPFilter` epoch-by-epoch and reports the final position error
against an independent truth coordinate.

## Usage

Float-PPP only (no AR), broadcast orbits, no SSR biases.  Loose
tolerance — confirms the position pipeline computes a reasonable
solution from RINEX inputs:

    python scripts/regression/run_regression.py \
        --obs /path/to/abmf0010.20o \
        --nav /path/to/brdc0010.20p \
        --truth "2919785.79086,-5383744.95943,1774604.85992" \
        --tolerance-m 10 \
        --max-epochs 200 \
        --profile l5

Add SSR biases (when a .BIA file is available):

    ... --bia /path/to/file.BIA --tolerance-m 1

Returns 0 on pass, non-zero on fail.  Reports per-axis errors and
RMS to stdout.

## Scope

This first cut is **float-PPP only**.  No MW tracker, no LAMBDA,
no per-SV state machine.  The goal is to validate that the basic
position-computation pipeline (filter + sat-position propagation
from broadcast NAV + observation ingest) produces an answer
consistent with the IGS-published truth coordinate.

Known TODO before the runner can actually converge against truth
within tight tolerance:

- **Receiver-clock initialization** — at startup, the real receiver
  carries a clock bias of microseconds-to-milliseconds, which shows
  up as a uniform per-SV pseudorange offset.  Float-PPP without the
  filter's clock state pre-seeded sees this as huge residuals on
  every observation and rejects most of them.  Use `solve_ppp.ls_init`
  on the first epoch to get a position+clock seed before launching
  the filter, the way `peppar_fix_engine.run_bootstrap` does.
- **SSR phase- and code-bias application** — `bias_sinex_reader`
  parses these but the runner doesn't yet apply them to obs.  Once
  wired, ~10 m → ~10 cm.
- **MW + LAMBDA + state machine** — once float-PPP converges, run
  the AR path against the same data and tighten to mm-level.

Until those land, the runner is useful as plumbing validation only.
"""

from __future__ import annotations

import argparse
import logging
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import numpy as np

from regression.rinex_reader import (
    iter_epochs, parse_header as parse_obs_header,
    extract_dual_freq, L5_PROFILE, L2_PROFILE,
)
from regression.rinex_nav_reader import load_into_ephemeris

log = logging.getLogger("regression")


C_LIGHT = 299_792_458.0


def _parse_truth(s: str) -> np.ndarray:
    parts = [float(x) for x in s.split(',')]
    if len(parts) != 3:
        raise ValueError(f"truth must be 'X,Y,Z' in meters: {s!r}")
    return np.array(parts)


_SYS_TO_LOWER = {'GPS': 'gps', 'GAL': 'gal', 'BDS': 'bds',
                 'GLO': 'glo', 'QZS': 'qzs'}


# L2C-family tracking modes (L, S, X) and L5 I-or-combined (Q, X) all
# target the same physical signal, and analysis centers typically
# publish one bias value that covers all tracking variants.  CODE's
# IAR products for 2020/001 specifically publish L2X as the canonical
# L2C attribute and verifiably use **identical** numeric values for
# L2C/L2W/L2X (e.g. G08 all three = 0.70203 ns).  RINEX OBS files,
# however, typically record whichever variant the receiver happened to
# track — L2L on a Septentrio.  Without this fallback, every L2L/L2S
# lookup misses and the harness processes uncorrected phase.
_OSB_ATTR_FALLBACK = {
    'L5Q': ('L5X',), 'L5X': ('L5Q',),
    'L2L': ('L2X', 'L2C'), 'L2S': ('L2X', 'L2C'),
    'L2C': ('L2X',), 'L2X': ('L2C',),
    'C5Q': ('C5X',), 'C5X': ('C5Q',),
    'C2L': ('C2X', 'C2C'), 'C2S': ('C2X', 'C2C'),
    'C2C': ('C2X',), 'C2X': ('C2C',),
    'C1C': ('C1X',), 'C1X': ('C1C',),
    'L1C': ('L1X',), 'L1X': ('L1C',),
}


def _osb_get(osb, sv: str, code: str):
    """OSB lookup with tracking-attribute fallback for CODE-style BIA files."""
    v = osb.get_osb(sv, code)
    if v is not None:
        return v
    for alt in _OSB_ATTR_FALLBACK.get(code, ()):
        v = osb.get_osb(sv, alt)
        if v is not None:
            return v
    return None


def _build_obs_for_filter(rx_obs, gps_time, osb=None):
    """Convert SvObservation list to the dict format PPPFilter.update
    expects (matches realtime_ppp.serial_reader output, including the
    lowercase 'sys' name convention).

    If an OSBParser is supplied, satellite-side code + phase biases are
    subtracted from the raw observations before the IF combination is
    formed — matching what `solve_ppp.load_ppp_epochs` does for the
    RAWX path.  Without this correction, per-SV L1-L5 ISC biases of
    several meters leak into pseudorange residuals."""
    try:
        from solve_ppp import SIG_TO_RINEX
    except ImportError:
        SIG_TO_RINEX = {}
    out = []
    for o in rx_obs:
        # Compute IF combination coefficients
        f1 = C_LIGHT / o.wl_f1
        f2 = C_LIGHT / o.wl_f2
        a1 = f1 * f1 / (f1 * f1 - f2 * f2)
        a2 = -f2 * f2 / (f1 * f1 - f2 * f2)
        pr1 = o.pr1_m
        pr2 = o.pr2_m
        phi1_m = o.phi1_cyc * o.wl_f1
        phi2_m = o.phi2_cyc * o.wl_f2
        if osb is not None:
            rinex_f1 = SIG_TO_RINEX.get(o.f1_sig_name)
            rinex_f2 = SIG_TO_RINEX.get(o.f2_sig_name)
            if rinex_f1 and rinex_f2:
                c1 = _osb_get(osb, o.sv, rinex_f1[0])
                c2 = _osb_get(osb, o.sv, rinex_f2[0])
                if c1 is not None and c2 is not None:
                    pr1 -= c1
                    pr2 -= c2
                p1 = _osb_get(osb, o.sv, rinex_f1[1])
                p2 = _osb_get(osb, o.sv, rinex_f2[1])
                if p1 is not None and p2 is not None:
                    phi1_m -= p1
                    phi2_m -= p2
        pr_if = a1 * pr1 + a2 * pr2
        phi_if_m = a1 * phi1_m + a2 * phi2_m
        out.append({
            'sv': o.sv,
            'sys': _SYS_TO_LOWER.get(o.sys, o.sys.lower()),
            'pr_if': pr_if,
            'phi_if_m': phi_if_m,
            'cno': o.cno,
            'lock_duration_ms': o.lock_duration_ms,
            'half_cyc_ok': o.half_cyc_ok,
            'phi1_cyc': o.phi1_cyc,
            'phi2_cyc': o.phi2_cyc,
            'phi1_raw_cyc': o.phi1_raw_cyc,
            'phi2_raw_cyc': o.phi2_raw_cyc,
            'pr1_m': o.pr1_m,
            'pr2_m': o.pr2_m,
            'wl_f1': o.wl_f1,
            'wl_f2': o.wl_f2,
            'f1_lock_ms': o.f1_lock_ms,
            'f2_lock_ms': o.f2_lock_ms,
            'f1_sig_name': o.f1_sig_name,
            'f2_sig_name': o.f2_sig_name,
        })
    return out


def run(args) -> int:
    """Run one regression scenario.  Returns process exit code."""
    # Late imports so the module is importable without engine deps
    from broadcast_eph import BroadcastEphemeris
    from solve_ppp import PPPFilter, ls_init, ecef_to_enu
    from ppp_ar import MelbourneWubbenaTracker

    truth_ecef = _parse_truth(args.truth)
    profile = L5_PROFILE if args.profile == "l5" else L2_PROFILE
    wl_only = bool(getattr(args, "wl_only", False))
    position_csv_path = getattr(args, "position_csv", None)
    position_csv_writer = None
    position_csv_fh = None
    if position_csv_path:
        import csv
        position_csv_fh = open(position_csv_path, "w", newline="")
        position_csv_writer = csv.writer(position_csv_fh)
        # Headers: epoch index, UTC timestamp, per-axis ECEF
        # error (m), per-axis ENU error (m), 3D / H / V norms,
        # SV counts so we can correlate residual shape with
        # geometry.  This is the direct per-epoch bias trace
        # for the Q1 systematic-bias question.
        position_csv_writer.writerow([
            "ep_idx", "utc",
            "err_ecef_x", "err_ecef_y", "err_ecef_z",
            "err_e", "err_n", "err_u",
            "err_3d", "err_h", "err_v",
            "n_used", "n_filter_svs", "n_wl_fixed",
        ])

    residuals_csv_path = getattr(args, "residuals_csv", None)
    residuals_csv_writer = None
    residuals_csv_fh = None
    if residuals_csv_path:
        import csv
        residuals_csv_fh = open(residuals_csv_path, "w", newline="")
        residuals_csv_writer = csv.writer(residuals_csv_fh)
        # Per-measurement row at every processed epoch.  Each
        # filt.update() call yields one row per PR measurement
        # and one per phase measurement per SV used.  Aligned
        # with `filt.last_residual_labels` (sv, kind, elev) so we
        # can do per-SV + per-signal analysis post-run:
        # clusters of same-signed residuals → common-mode
        # (reference frame / clock); per-SV scatter → per-SV
        # bias table (DCB / TGD / ISC).
        residuals_csv_writer.writerow([
            "ep_idx", "utc", "sv", "sys", "kind", "elev_deg",
            "post_resid_m",
        ])

    # Header — gives us the receiver's APPROX POSITION as seed if no
    # explicit seed; gives us the observation interval too.
    obs_path = Path(args.obs)
    obs_hdr = parse_obs_header(obs_path)
    interval_s = obs_hdr.interval_s or 30.0

    # Ephemeris source: SP3 precise orbits when available (sub-cm
    # accuracy), broadcast NAV otherwise (~1–2 m).  Both provide the
    # same `sat_position(sv, t) → (pos, clk)` interface, so the filter
    # doesn't care which it gets.
    if args.sp3:
        from solve_pseudorange import SP3
        sp3 = SP3(args.sp3)
        log.info("Loaded SP3: %d epochs, %d SVs",
                 len(sp3.epochs), len(sp3.positions))
        eph_source = sp3
    else:
        nav_path = Path(args.nav) if args.nav else None
        if nav_path is None:
            log.error("must provide --nav or --sp3")
            return 2
        beph = BroadcastEphemeris()
        n_eph = load_into_ephemeris(nav_path, beph)
        log.info("Loaded %d broadcast ephemeris records (%d SVs)",
                 n_eph, beph.n_satellites)
        eph_source = beph

    # Optional high-rate satellite clock file.  30 s RINEX CLK files
    # from analysis centers override the 300 s SP3 clocks with ~30–50 ps
    # accuracy — essential for sub-dm PPP since the 300 s SP3 clock
    # interpolation error can be several ns of pseudorange.
    clk_file = None
    if args.clk:
        from ppp_corrections import CLKFile
        clk_file = CLKFile(args.clk)
        log.info("Loaded CLK: %d SVs", len(clk_file._t0))

    # Optional satellite-side code + phase bias file (Bias-SINEX OSB).
    # CODE, WUM, and CNES all publish these; applying them removes the
    # per-SV L1-L5 ISC biases and (for phase) enables PPP-AR downstream.
    osb = None
    if args.bia:
        from ppp_corrections import OSBParser
        osb = OSBParser(args.bia)
        log.info("Loaded OSB: %d (PRN, signal) bias entries across %d SVs",
                 len(osb.biases), len(osb.prns()))

    # Filter is initialised lazily on the first usable epoch — we use
    # ls_init() to seed both position AND receiver clock from that
    # epoch's pseudoranges.  Seeding clock=0 (the previous behavior)
    # leaves the filter facing a microsecond-to-millisecond receiver
    # clock bias on every observation, which it rejects as outliers
    # before its EKF can converge.
    filt: Optional[PPPFilter] = None
    systems_lower = {_SYS_TO_LOWER.get(s, s.lower()) for s in profile.keys()}
    # Optional per-constellation gate — lets us isolate which
    # constellation drives any systematic bias.  --systems gps,gal
    # keeps only GPS and Galileo observations; SVs from other
    # constellations are dropped at _build_obs_for_filter time.
    # Name convention matches the engine's --systems flag.
    systems_filter: Optional[set[str]] = None
    if getattr(args, "systems", None):
        systems_filter = {
            _SYS_TO_LOWER.get(s.strip().upper(), s.strip().lower())
            for s in args.systems.split(",")
        }
        # Also narrow the filter-init set so the filter doesn't
        # allocate ISB states for systems we're skipping.
        systems_lower = systems_lower & systems_filter
        log.info("Systems filter: %s (profile had %s)",
                 sorted(systems_filter),
                 sorted({_SYS_TO_LOWER.get(s, s.lower())
                         for s in profile.keys()}))
    seed_offset: Optional[float] = None

    # Melbourne-Wubbena wide-lane tracker.  Per-SV WL integer fixing
    # plus jump (cycle-slip) detection.  In WL-only / float mode we
    # don't apply any pseudo-measurement to the float IF state from
    # MW — the win is purely from cycle-slip detection: when MW
    # detects a jump on an SV, we drop and re-add that SV's
    # ambiguity in the float filter so the slip doesn't poison the
    # float estimate for the rest of the run.
    #
    # Engine reference: `peppar_fix_engine.py:1911-1928` for the
    # MW.update call, `:1929-1980` for slip handling on already-
    # WL-fixed SVs (post-fix drift monitor).  We adapt the same
    # pattern but skip the drift monitor — for the harness the
    # simpler `MelbourneWubbenaTracker.detect_jump` is enough.
    mw = MelbourneWubbenaTracker()
    n_wl_fixed_max = 0       # high-water mark of concurrent WL fixes
    n_slip_resets = 0        # SV ambiguity resets due to MW jump
    if wl_only:
        log.info("WL-only mode: MW slip detection on, no NL constraint")

    # Iterate epochs
    prev_t = None
    n_processed = 0
    n_skipped_empty = 0
    n_skipped_too_few = 0
    last_pos = truth_ecef
    lock_accum: dict = {}
    # Convergence-checkpoint reporting: drop a one-line per-epoch
    # summary at each of these processed-epoch counts so the gate
    # ladder is self-documenting.  FINAL is reported at end.  The
    # epochs are spaced log-style — early epochs converge fast,
    # later epochs reveal long-tail biases.
    checkpoint_epochs = sorted({100, 500, 1000, 2000, 4000, 8000, 16000})
    checkpoint_results: list[tuple[int, float, float, float]] = []

    for ep_idx, ep in enumerate(iter_epochs(obs_path)):
        if args.max_epochs and ep_idx >= args.max_epochs:
            break

        t = ep.ts.replace(tzinfo=timezone.utc)
        sv_obs_list = extract_dual_freq(
            ep, profile=profile, interval_s=interval_s,
            lock_accum=lock_accum,
        )
        if not sv_obs_list:
            n_skipped_empty += 1
            continue

        observations = _build_obs_for_filter(sv_obs_list, t, osb=osb)
        if systems_filter is not None:
            observations = [o for o in observations
                            if o['sys'] in systems_filter]
            if not observations:
                n_skipped_empty += 1
                continue

        # First-usable-epoch bootstrap via ls_init: solves for
        # position + receiver-clock offset from the IF pseudoranges
        # alone.  Without this seed, the filter starts with clk=0
        # but the real receiver carries a μs–ms clock bias that
        # shows up as huge per-SV pseudorange residuals.
        if filt is None:
            try:
                ls_result, ls_ok, ls_n = ls_init(
                    observations, eph_source, t, clk_file=clk_file,
                )
            except Exception as e:
                log.warning("ls_init failed at epoch %d: %s", ep_idx, e)
                continue
            if not ls_ok or ls_n < 4:
                log.debug("ls_init not converged at epoch %d (ok=%s n=%d)",
                          ep_idx, ls_ok, ls_n)
                continue
            init_ecef = np.array(ls_result[:3])
            init_clk = float(ls_result[3])
            seed_offset = float(np.linalg.norm(init_ecef - truth_ecef))
            log.info("ls_init bootstrap: pos=%s, clk=%.3e s "
                     "(%.2f m from truth, n_used=%d)",
                     init_ecef.tolist(), init_clk / C_LIGHT,
                     seed_offset, ls_n)
            filt = PPPFilter()
            filt.initialize(init_ecef, init_clk, systems=systems_lower)

        # Filter prediction step
        if prev_t is not None:
            dt = (t - prev_t).total_seconds()
            if dt > 0:
                filt.predict(dt)
        prev_t = t

        # Ambiguity management.  Must happen BEFORE filt.update
        # so phase observations contribute (the update loop
        # gates `if sv in self.sv_to_idx` for phase).  Mirrors
        # what `solve_ppp.__main__`'s steady-state loop does at
        # lines 1172-1178; the existing harness lacked this so
        # it was silently running PR-only PPP.
        current_svs = {o['sv'] for o in observations}
        tracked = set(filt.sv_to_idx.keys())
        # SVs that vanished this epoch — drop their ambiguity
        # state so stale float values don't linger as SV rises
        # again (cycle slip + arc gap semantics are handled via
        # MW's detect_jump + explicit remove above).
        for sv in tracked - current_svs:
            filt.remove_ambiguity(sv)
        # New SVs this epoch — seed the ambiguity from the first
        # phase observation.  N_init = phi_if - pr_if is the
        # standard cold-start for a float IF ambiguity
        # (meters), leverages the PR estimate of range so the
        # initial phase ambiguity is close to truth.
        for o in observations:
            if (o['sv'] not in filt.sv_to_idx
                    and o.get('phi_if_m') is not None):
                filt.add_ambiguity(
                    o['sv'], o['phi_if_m'] - o['pr_if'],
                )

        # MW wide-lane update (per SV).  The pre-update step here
        # mirrors the engine's order: MW first so the slip detector
        # sees current observations against the pre-update average,
        # then filter update absorbs the (possibly slip-flushed)
        # observations.  Slip detection on an SV that's already in
        # the float filter triggers a remove + re-add: dropping the
        # ambiguity flushes its float estimate so the post-slip
        # observations don't anchor against a now-wrong integer.
        # Filter re-adds the SV's ambiguity on the next observation
        # at line 502-503 of solve_ppp.py.
        mw._current_epoch = ep_idx
        slip_resets_this_ep = 0
        for o in observations:
            sv = o['sv']
            phi1 = o.get('phi1_cyc')
            phi2 = o.get('phi2_cyc')
            pr1 = o.get('pr1_m')
            pr2 = o.get('pr2_m')
            wl1 = o.get('wl_f1')
            wl2 = o.get('wl_f2')
            if not all(v is not None for v in (phi1, phi2, pr1, pr2, wl1, wl2)):
                continue
            f1_hz = C_LIGHT / wl1
            f2_hz = C_LIGHT / wl2
            # Slip detection BEFORE update: detect_jump compares the
            # incoming MW sample against the existing tracker state;
            # a real cycle slip lands many σ outside the rolling
            # residual window.
            try:
                jump = mw.detect_jump(o)
            except Exception:
                jump = None
            jumped = bool(jump and jump.get('is_slip'))
            if jumped and sv in filt.sv_to_idx:
                # Reset MW state and remove the ambiguity from the
                # filter.  Filter re-adds with a fresh float estimate
                # on the next phase observation for this SV.
                mw.reset(sv)
                filt.remove_ambiguity(sv)
                slip_resets_this_ep += 1
                n_slip_resets += 1
            mw.update(sv, phi1, phi2, pr1, pr2, f1_hz, f2_hz)

        # Track WL-fix high-water mark for diagnostics.
        n_wl_now = mw.n_fixed
        if n_wl_now > n_wl_fixed_max:
            n_wl_fixed_max = n_wl_now

        # Filter update — eph_source supplies sat_position which returns
        # (pos, clk).  clk_file overrides the clock when given (high-rate
        # CLK product); otherwise the filter uses the clock value from
        # sat_position.
        try:
            n_used, resid, sys_counts = filt.update(
                observations, eph_source, t, clk_file=clk_file,
            )
        except Exception as e:
            log.error("filt.update failed at epoch %d (%s): %s",
                      ep_idx, t, e)
            continue

        if n_used < 4:
            n_skipped_too_few += 1
            continue

        n_processed += 1
        last_pos = filt.x[:3].copy()

        # Per-SV residual dump.  Runs after filt.update so the
        # residuals are post-fit.  Labels are (sv, kind, elev)
        # and align with the `resid` vector returned above.
        # Matching labels against resid is O(n_used); n_used is
        # typically ~10-25, so per-epoch overhead is negligible.
        if residuals_csv_writer is not None:
            labels_out = getattr(filt, "last_residual_labels", [])
            for (sv, kind, elev), r in zip(labels_out, resid):
                residuals_csv_writer.writerow([
                    ep_idx, t.strftime("%Y-%m-%dT%H:%M:%S"),
                    sv, sv[0], kind, f"{elev:.1f}",
                    f"{float(r):.4f}",
                ])

        # Per-epoch error tracking.  ENU decomposition anchored at
        # the truth point (not the filter estimate) so the ENU
        # values are the true east / north / up components of our
        # bias, not of our uncertainty ellipse.
        err_ecef = last_pos - truth_ecef
        err_enu = ecef_to_enu(err_ecef, truth_ecef)

        if position_csv_writer is not None:
            position_csv_writer.writerow([
                ep_idx, t.strftime("%Y-%m-%dT%H:%M:%S"),
                f"{err_ecef[0]:.4f}", f"{err_ecef[1]:.4f}",
                f"{err_ecef[2]:.4f}",
                f"{err_enu[0]:.4f}", f"{err_enu[1]:.4f}",
                f"{err_enu[2]:.4f}",
                f"{float(np.linalg.norm(err_ecef)):.4f}",
                f"{float(np.linalg.norm(err_ecef[:2])):.4f}",
                f"{float(abs(err_ecef[2])):.4f}",
                n_used, len(filt.sv_to_idx), n_wl_now,
            ])

        # Convergence-checkpoint capture: snapshot the position error
        # at each milestone so the gate ladder reports show how the
        # error decays over time.
        while checkpoint_epochs and n_processed == checkpoint_epochs[0]:
            checkpoint_results.append((
                n_processed,
                float(np.linalg.norm(err_ecef)),
                float(np.linalg.norm(err_ecef[:2])),
                float(abs(err_ecef[2])),
            ))
            checkpoint_epochs.pop(0)

        if n_processed == 1 or n_processed % 20 == 0:
            err_h = float(np.linalg.norm(err_ecef[:2]))
            err_v = float(abs(err_ecef[2]))
            slip_frag = (f" slips={slip_resets_this_ep}"
                         if slip_resets_this_ep else "")
            log.info("epoch %4d  t=%s  n_used=%2d  err_h=%6.2fm "
                     "err_v=%6.2fm  wl_fixed=%d/%d%s",
                     ep_idx, t.strftime("%H:%M:%S"), n_used, err_h, err_v,
                     n_wl_now, len(filt.sv_to_idx), slip_frag)

    if position_csv_fh is not None:
        position_csv_fh.close()
        log.info("Wrote per-epoch position errors to %s", position_csv_path)

    if residuals_csv_fh is not None:
        residuals_csv_fh.close()
        log.info("Wrote per-SV residuals to %s", residuals_csv_path)

    # Final assessment
    err = last_pos - truth_ecef
    err_3d = float(np.linalg.norm(err))
    err_h = float(np.linalg.norm(err[:2]))
    err_v = float(abs(err[2]))

    print(f"\n{'=' * 60}")
    print(f"Regression result")
    print(f"{'=' * 60}")
    print(f"Profile:           {args.profile}")
    print(f"AR mode:           {'wl-only' if wl_only else 'float'}")
    print(f"Epochs processed:  {n_processed}")
    print(f"Epochs skipped:    {n_skipped_empty} (empty), "
          f"{n_skipped_too_few} (too-few-SVs)")
    print(f"Initial seed err:  "
          f"{seed_offset:.3f} m" if seed_offset is not None else "n/a")
    print(f"Max concurrent WL: {n_wl_fixed_max}")
    print(f"MW slip resets:    {n_slip_resets}")
    if checkpoint_results:
        print(f"\nConvergence ladder (3D / H / V error in m):")
        for n_ep, e3, eh, ev in checkpoint_results:
            print(f"  @ {n_ep:>5d} ep:  {e3:7.3f}  /  {eh:7.3f}  /  {ev:7.3f}")
    print(f"\nFinal position:    {last_pos.tolist()}")
    print(f"Truth position:    {truth_ecef.tolist()}")
    print(f"Final error 3D:    {err_3d:.3f} m")
    print(f"Final error H:     {err_h:.3f} m")
    print(f"Final error V:     {err_v:.3f} m")
    print(f"Tolerance:         {args.tolerance_m:.3f} m (3D)")
    if n_processed == 0:
        print("FAIL — no epochs processed (check NAV file, observation "
              "format, or systems-filter settings)")
        return 2
    if err_3d <= args.tolerance_m:
        print("PASS")
        return 0
    print("FAIL")
    return 1


def main():
    ap = argparse.ArgumentParser(
        description="Run a regression scenario through PePPAR Fix's PPP pipeline"
    )
    ap.add_argument("--obs", required=True,
                    help="RINEX 3.x OBS file (PRIDE-PPPAR or IGS MGEX)")
    ap.add_argument("--nav", default=None,
                    help="RINEX 3.x NAV file (broadcast ephemeris).  "
                         "Either --nav or --sp3 must be provided.")
    ap.add_argument("--sp3", default=None,
                    help="SP3 precise orbit file (e.g. CODE com20863.eph).  "
                         "If provided, overrides --nav as the orbit source "
                         "and gives sub-cm satellite position accuracy.")
    ap.add_argument("--clk", default=None,
                    help="RINEX CLK file with high-rate precise clocks "
                         "(e.g. CODE com20863.clk at 30 s).  Overrides the "
                         "clock values from --sp3 / --nav.  Required for "
                         "sub-dm results.")
    ap.add_argument("--bia", default=None,
                    help="Optional Bias-SINEX OSB file")
    ap.add_argument("--truth", required=True,
                    help="Truth ECEF position 'X,Y,Z' in meters")
    ap.add_argument("--tolerance-m", type=float, default=5.0,
                    help="3D position-error tolerance in meters (default 5)")
    ap.add_argument("--profile", choices=["l5", "l2"], default="l5",
                    help="Receiver profile: l5 (F9T-L5) or l2 (F9T-L2)")
    ap.add_argument("--max-epochs", type=int, default=None,
                    help="Limit epoch count for quick runs (default: full file)")
    ap.add_argument("--systems", default=None,
                    help="Comma-separated constellation filter "
                         "(e.g. 'gps', 'gal', 'gps,gal').  "
                         "Drops observations from constellations "
                         "not in the list before they reach the "
                         "filter.  Default: all constellations "
                         "in the active --profile.  Used to "
                         "isolate per-constellation systematic "
                         "bias signatures.")
    ap.add_argument("--residuals-csv", default=None,
                    help="If set, write one row per per-SV-per-"
                         "epoch post-fit residual to this CSV.  "
                         "Columns: ep_idx, utc, sv, sys, kind "
                         "(pr/phi), elev_deg, post_resid_m.  "
                         "Use to diff per-SV signatures between "
                         "constellations and pin down systematic "
                         "biases (common-mode = reference-frame "
                         "issue; per-SV scatter = DCB/TGD/ISC).")
    ap.add_argument("--position-csv", default=None,
                    help="If set, write one row per processed "
                         "epoch to this CSV file.  Columns: "
                         "ep_idx, utc, ECEF error (x, y, z), ENU "
                         "error (e, n, u), 3D / H / V norms, SV "
                         "counts.  ENU is anchored at the truth "
                         "point — the values are the systematic "
                         "bias, east / north / up, of our "
                         "solution vs ITRF14 at every epoch.  "
                         "Intended for post-run bias-signature "
                         "analysis: trend, periodicity, per-axis "
                         "breakdown.  Not enabled by default; the "
                         "CSV is ~30 KB per 1000 epochs.")
    ap.add_argument("--wl-only", action="store_true",
                    help="WL-only AR mode: enable Melbourne-Wubbena slip "
                         "detection (resets ambiguity in the float filter "
                         "on a detected jump) but apply no NL fixing.  "
                         "Mirrors the engine's --wl-only contract: MW "
                         "tracks per-SV WL fix status for diagnostic "
                         "reporting (high-water mark + slip count) but "
                         "the float IF filter receives no pseudo-"
                         "measurement on its ambiguity state.  The win "
                         "vs pure float-PPP is purely from cycle-slip "
                         "detection preventing slips from poisoning the "
                         "float ambiguity for the rest of the run.  "
                         "Without this, undetected slips on a 24h run "
                         "leave the float ambiguity stuck at a wrong "
                         "value, biasing position by 10 cm-1 m for "
                         "the affected SV.  Target: ABMF 2020 DOY 001 "
                         "≤ 20 cm 3D vs ITRF14.")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    return run(args)


if __name__ == "__main__":
    sys.exit(main())

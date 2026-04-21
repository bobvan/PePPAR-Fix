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


def _build_obs_for_filter(rx_obs, gps_time):
    """Convert SvObservation list to the dict format PPPFilter.update
    expects (matches realtime_ppp.serial_reader output)."""
    out = []
    for o in rx_obs:
        # Compute IF combination
        f1 = C_LIGHT / o.wl_f1
        f2 = C_LIGHT / o.wl_f2
        a1 = f1 * f1 / (f1 * f1 - f2 * f2)
        a2 = -f2 * f2 / (f1 * f1 - f2 * f2)
        pr_if = a1 * o.pr1_m + a2 * o.pr2_m
        # Phase combination: convert cycles → meters first
        phi1_m = o.phi1_cyc * o.wl_f1
        phi2_m = o.phi2_cyc * o.wl_f2
        phi_if_m = a1 * phi1_m + a2 * phi2_m
        out.append({
            'sv': o.sv,
            'sys': o.sys,
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
    from solve_ppp import PPPFilter

    truth_ecef = _parse_truth(args.truth)
    profile = L5_PROFILE if args.profile == "l5" else L2_PROFILE

    # Header — gives us the receiver's APPROX POSITION as seed if no
    # explicit seed; gives us the observation interval too.
    obs_path = Path(args.obs)
    obs_hdr = parse_obs_header(obs_path)
    interval_s = obs_hdr.interval_s or 30.0

    # Initial position — start from APPROX POSITION (a few-meter
    # estimate the receiver carries in the file) so the filter's
    # convergence path is realistic.  If truth is sub-cm this is
    # already pretty close, but we want to test the filter's
    # ability to refine, not just declare truth.
    init_ecef = (
        np.array(obs_hdr.approx_xyz) if obs_hdr.approx_xyz else truth_ecef
    )
    seed_offset = float(np.linalg.norm(init_ecef - truth_ecef))
    log.info("Initial seed ECEF: %s (%.2f m from truth)",
             init_ecef.tolist(), seed_offset)

    # Load broadcast ephemeris from RINEX NAV
    nav_path = Path(args.nav)
    beph = BroadcastEphemeris()
    n_eph = load_into_ephemeris(nav_path, beph)
    log.info("Loaded %d broadcast ephemeris records (%d SVs)",
             n_eph, beph.n_satellites)

    # Initialise PPP filter
    filt = PPPFilter()
    systems = set()
    for sys_name in profile.keys():
        systems.add(sys_name)
    filt.initialize(init_ecef, 0.0, systems=systems)

    # Iterate epochs
    prev_t = None
    n_processed = 0
    n_skipped_empty = 0
    n_skipped_too_few = 0
    last_pos = init_ecef
    lock_accum: dict = {}

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

        observations = _build_obs_for_filter(sv_obs_list, t)

        # Filter prediction step
        if prev_t is not None:
            dt = (t - prev_t).total_seconds()
            if dt > 0:
                filt.predict(dt)
        prev_t = t

        # Filter update — beph supplies sat_position which returns
        # (pos, clk).  clk_file=None tells the filter to use the clock
        # value from sat_position (BroadcastEphemeris doesn't expose a
        # separate sat_clock method, unlike CLK precise products).
        try:
            n_used, resid, sys_counts = filt.update(
                observations, beph, t, clk_file=None,
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

        if n_processed == 1 or n_processed % 20 == 0:
            err = last_pos - truth_ecef
            err_h = float(np.linalg.norm(err[:2]))
            err_v = float(abs(err[2]))
            log.info("epoch %4d  t=%s  n_used=%2d  err_h=%6.2fm err_v=%6.2fm",
                     ep_idx, t.strftime("%H:%M:%S"), n_used, err_h, err_v)

    # Final assessment
    err = last_pos - truth_ecef
    err_3d = float(np.linalg.norm(err))
    err_h = float(np.linalg.norm(err[:2]))
    err_v = float(abs(err[2]))

    print(f"\n{'=' * 60}")
    print(f"Regression result")
    print(f"{'=' * 60}")
    print(f"Profile:           {args.profile}")
    print(f"Epochs processed:  {n_processed}")
    print(f"Epochs skipped:    {n_skipped_empty} (empty), "
          f"{n_skipped_too_few} (too-few-SVs)")
    print(f"Initial seed err:  {seed_offset:.3f} m")
    print(f"Final position:    {last_pos.tolist()}")
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
    ap.add_argument("--nav", required=True,
                    help="RINEX 3.x NAV file (broadcast ephemeris)")
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
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    return run(args)


if __name__ == "__main__":
    sys.exit(main())

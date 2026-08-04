#!/usr/bin/env python3
"""peppar-survey — write state/positions/<uid>.survey.toml from
external authoritative observations.

Optional companion to the engine, per
``docs/position-state-and-monitoring.md``.  Triggered by a systemd
timer, ``@daily`` cron, or any external orchestrator.

The contract is:

  Input:  external authoritative observation source — captured
          RINEX submitted to OPUS or processed by PRIDE, or a
          quick NTRIP CORS-RTK check against a nearby reference.
  Output: atomic temp+rename write of
          state/positions/<uid>.survey.toml with current mount_sn
          tag, fresh ECEF, sigma, and provenance metadata.

Naming convention: "survey" is reserved for external authoritative
observations.  The engine's own AntPosEst output is a "PPP
solution" and goes to .ppp.toml (written by the engine, not by
peppar-survey).  This CLI cannot write a .survey.toml from a PPP
snapshot — that confusion was the root cause of the TimeHat
mount_sn=1 incident 2026-05-18 (an early-epoch PPP snapshot
promoted to "survey" via the now-removed --from-ppp backend
seeded the engine with a 0.95 m sigma estimate that AntPosEst
later detected was 1.5 m off, triggering an auto-step that
ultimately did the right thing but cost an engine restart).

Backends (none implemented in this skeleton):

  --pride <work_dir>  Wrap PRIDE PPP-AR over the captured RINEX,
                      extract the daily solution, append to a
                      history.jsonl, and aggregate via running_mean
                      from scripts/peppar_fix/arp_history.py.
  --opus              Submit to NGS OPUS-Static and parse the result.
  --cors              Quick NTRIP CORS-RTK check against a nearby
                      reference station (minutes-class accuracy).
  --rtklib            Local RTKLIB PPP run.

When a real backend lands, the contract above stays the same — the
caller (this CLI) is unchanged; only the backend module differs.
Until then, peppar-survey errors out at argparse stage: there's no
backend to run.  This is by design — installations without survey
infrastructure simply rely on AntPosEst's own warm-start via
.ppp.toml.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys


log = logging.getLogger("peppar-survey")

# Make peppar_fix importable when run from the repo root.
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))


def discover_single_receiver_uid(receivers_dir: str) -> str | None:
    """Find the single receiver UID from state/receivers/.  Returns
    None when zero or multiple receiver files exist — caller must
    pass --receiver-uid explicitly in those cases."""
    if not os.path.isdir(receivers_dir):
        return None
    uids = []
    for name in os.listdir(receivers_dir):
        if name.endswith(".json") and not name.endswith(".bak"):
            # Skip the *.json.<tag>.bak files even though they don't
            # end in .bak — common pattern is e.g. ".day0424i.bak".
            if ".bak." in name or name.endswith(".bak"):
                continue
            uid = name[:-5]  # strip ".json"
            uids.append(uid)
    if len(uids) == 1:
        return uids[0]
    if len(uids) == 0:
        log.error("No receivers found in %s", receivers_dir)
    else:
        log.error("Multiple receivers found in %s (%s) — pass "
                  "--receiver-uid", receivers_dir, ", ".join(uids))
    return None


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=("Detailed usage in the module docstring at the top "
                "of this file."),
    )
    ap.add_argument("--receiver-uid",
                    help="Receiver SEC-UNIQID (decimal string) or "
                         "synthetic UID (e.g. synth_D30GD1PE).  "
                         "Auto-discovered from state/receivers/ "
                         "when there's exactly one receiver.")
    ap.add_argument("--positions-dir", default=None,
                    help="Override state/positions/ directory.")
    ap.add_argument("--receivers-dir", default=None,
                    help="Override state/receivers/ directory.")
    ap.add_argument("--dry-run", action="store_true",
                    help="Compute the survey snapshot, log what would "
                         "be written, but don't write the file.")
    ap.add_argument("--mount-sn", type=int, default=None,
                    help="Antenna mount serial to stamp on the result "
                         "(also the history partition).  Default: read the "
                         "receiver's CURRENT mount_sn from "
                         "state/receivers/<uid>.json — pass this only to "
                         "override deliberately.  A stamp that disagrees "
                         "with the engine's current mount_sn is silently "
                         "discarded as a seed.")
    ap.add_argument("-v", "--verbose", action="store_true")

    # --- backends --- #
    backends = ap.add_argument_group(
        "backends",
        "Select exactly one backend.  Each one consumes its own "
        "additional flags from the relevant group below.",
    )
    backends.add_argument(
        "--pride", action="store_true",
        help="Use PRIDE-PPP-AR over captured RINEX.  Requires "
             "--rinex-glob and pdp3 installed (PEPPAR_PDP3_BIN env var "
             "or default ~/.PRIDE_PPPAR_BIN/pdp3).",
    )
    backends.add_argument(
        "--rtklib", action="store_true",
        help="Use RTKLIB rnx2rtkp PPP-static over captured RINEX "
             "(absolute, no base).  Practical alternative to --pride "
             "for F9T L1+L5 hardware where WUM phase biases don't cover "
             "GPS L5.  For a relative baseline against a CORS/peer base, "
             "use --baseline.  Requires rnx2rtkp on PATH or via "
             "PEPPAR_RNX2RTKP_BIN.",
    )
    backends.add_argument(
        "--baseline", action="store_true",
        help="RTKLIB RTK-static RELATIVE baseline against a base station "
             "(NOAA CORS via --base, or a peer caster via "
             "--base-ntrip-host).  cm-class where --rtklib/--pride are "
             "dm/latency-bound.  The base is pre-converted to "
             "ITRF2020@epoch so the rover result is native ITRF2020.  "
             "Requires --rinex-glob (rover) + a base, and rnx2rtkp.",
    )
    backends.add_argument(
        "--auto", action="store_true",
        help="Pick the fastest backend that will work for this receiver + "
             "site (dual-freq + a discoverable nearby base → --baseline), "
             "falling back to the PRIDE PPP-AR floor otherwise.  Heuristics "
             "only buy speed — the floor guarantees correctness.  Use "
             "--plan-only to see the choice without running.",
    )
    pride = ap.add_argument_group("--pride options")
    pride.add_argument(
        "--rinex-glob", default=None,
        help="Glob matching the daily RINEX obs files for this "
             "receiver, e.g. 'data/rinex/MadHat-*.obs'.",
    )
    pride.add_argument(
        "--pride-work-dir", default=None,
        help="Scratch directory for pdp3 (per-day subdirs created "
             "inside).  Default: $TMPDIR/peppar-survey-pride/.",
    )
    pride.add_argument(
        "--history-dir", default=None,
        help="Override state/arp/ directory (history.jsonl lives "
             "under <history-dir>/<uid>/history.jsonl).",
    )
    pride.add_argument(
        "--n-days", type=int, default=7,
        help="Window size for the running ARP mean (default 7).",
    )
    pride.add_argument(
        "--pride-sys", default="GREC,GR",
        help="Comma-separated -sys strings to try in order.  Default "
             "'GREC,GR' (multi-GNSS first, GPS+GLO fallback).",
    )
    pride.add_argument(
        "--max-sig0-m", type=float, default=None,
        help="Quality gate: reject solutions with sig0_m above this "
             "(formal-precision check).  Default 10.0 m (catches "
             "catastrophically bad days while accepting weather-noisy "
             "ones).  Lower for stricter labs; higher to accept short "
             "fragment captures from a non-24h-continuous engine.",
    )
    pride.add_argument(
        "--min-n-obs", type=int, default=None,
        help="Quality gate: reject solutions with fewer than N "
             "observations USED by PRIDE.  Default 10000 (defends "
             "against truncated runs; a healthy 24h F9T capture "
             "clears this 2-10x).  Lower (e.g. 100) when validating "
             "against short engine-fragment captures.",
    )
    pride.add_argument(
        "--brdm-source", default=None,
        help="Path to a pre-fetched multi-GNSS broadcast nav file "
             "(or a directory containing brdmDDD0.YYp files).  "
             "Staged into pdp3's work_dir before each run so pdp3 "
             "skips its own IGS-MGEX download.  Workaround for the "
             "GPS-only-brdm fall-through that produces DEL_BADRANGE "
             "on every GAL/BDS SV (see docs / prideMultiGnssBrdm).  "
             "File: used as-is for every obs file.  Directory: looks "
             "up brdmDDD0.YYp matching each obs file's year/doy.",
    )
    pride.add_argument(
        "--wum-source", default=None,
        help="Directory of pre-fetched WUM products (orbit / clock / "
             "phase-bias / ERP) for pdp3.  Files matching each obs "
             "file's year/doy are gunzipped into pdp3's product/common "
             "dir before each run, so pdp3's USECACHE path picks them "
             "up and the flaky bdspride→IGN→gnsswhu→bdspride→gnsswhu "
             "download ladder never has to fire.  Codified gnsswhu "
             "fallback paths live in scripts/fetch_wum_products.sh; "
             "see wumProductGnsswhuFallback-charlie for the discovery.",
    )

    rtklib = ap.add_argument_group("--rtklib options")
    rtklib.add_argument(
        "--rtklib-work-dir", default=None,
        help="Scratch directory for rnx2rtkp (per-day subdirs created "
             "inside).  Default: $TMPDIR/peppar-survey-rtklib/.",
    )
    rtklib.add_argument(
        "--rtklib-nav", default=None,
        help="Path to a broadcast-nav RINEX file (.<yy>p) to pass to "
             "rnx2rtkp.  If unset, rnx2rtkp tries to find one in "
             "work_dir.  Shared with --baseline.",
    )

    baseline = ap.add_argument_group("--baseline options")
    baseline.add_argument(
        "--base", default=None,
        help="Base station for the relative baseline: either a 4-char "
             "NOAA CORS station code (e.g. 'dsp1', fetched from "
             "geodesy.noaa.gov/corsdata) OR a path to a base RINEX file "
             "(auto-detected: a value with '/' or '.' or an existing "
             "file is treated as a path, else a station code).  Its "
             "APPROX POSITION is pre-converted to ITRF2020@epoch.",
    )
    baseline.add_argument(
        "--base-realization", default="NAD83(2011)",
        help="The base's regional datum realization for the ITRF2020 "
             "pre-conversion.  Default 'NAD83(2011)' (NOAA CORS); use "
             "'ETRS89' for EUREF bases.",
    )
    baseline.add_argument(
        "--base-work-dir", default=None,
        help="Scratch directory for the baseline run.  Default: "
             "$TMPDIR/peppar-survey-baseline/.",
    )
    baseline.add_argument(
        "--base-nav", default=None,
        help="Broadcast-nav RINEX for the baseline rnx2rtkp run "
             "(falls back to --rtklib-nav, then work_dir discovery).",
    )
    baseline.add_argument(
        "--base-ntrip-host", default=None,
        help="Hostname/IP of a peer peppar-fix's NTRIP caster "
             "(scripts/ntrip_caster.py).  Streams RTCM 3.3 MSM4 + 1005 "
             "for --base-ntrip-duration seconds and uses the converted "
             "RINEX as the base.  Mutually exclusive with --base.  "
             "Requires str2str + convbin alongside rnx2rtkp.",
    )
    baseline.add_argument(
        "--base-ntrip-port", type=int, default=2102,
        help="Peer caster port.  Default 2102 (scripts/ntrip_caster.py).",
    )
    baseline.add_argument(
        "--base-ntrip-mount", default="PEPPAR",
        help="Peer caster mountpoint.  Default 'PEPPAR'.",
    )
    baseline.add_argument(
        "--base-ntrip-duration", type=int, default=None,
        help="Seconds of RTCM to capture from the peer.  Default 300 "
             "(5 min).  Must overlap the rover RINEX time range.",
    )

    auto = ap.add_argument_group("--auto options")
    auto.add_argument(
        "--plan-only", action="store_true",
        help="With --auto: print the selected backend + reasoning and exit, "
             "without fetching a base or running a solve.",
    )
    auto.add_argument(
        "--near", default=None,
        help="With --auto: 'LAT,LON' (deg) override for region + base "
             "selection.  Default: the receiver's last-known fix.",
    )
    auto.add_argument(
        "--caster-host", default=None,
        help="With --auto: NTRIP caster host whose sourcetable is searched "
             "for a nearby base.  Only needed where the region's archive has "
             "no station catalogue of its own (EUREF); in North America "
             "--auto ranks the NGS CORS catalogue without any caster.",
    )
    auto.add_argument(
        "--caster-port", type=int, default=2101,
        help="With --auto: caster port for sourcetable discovery "
             "(default 2101).",
    )
    auto.add_argument(
        "--max-km", type=float, default=80.0,
        help="With --auto: reject bases farther than this from the site "
             "(default 80 km — a longer baseline erodes the cm advantage).",
    )

    args = ap.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    # Auto-discover receiver UID (still useful — future backends will
    # want it, and this exercises that path).
    if args.receiver_uid is None:
        from peppar_fix.receiver_state import DEFAULT_STATE_DIR
        rdir = args.receivers_dir or DEFAULT_STATE_DIR
        uid = discover_single_receiver_uid(rdir)
        if uid is None:
            return 1
        args.receiver_uid = uid
        log.info("Auto-discovered receiver_uid=%s", uid)

    # Resolve the mount_sn to stamp BEFORE any backend runs.  The engine
    # filters seed candidates on mount_sn (filter_current_mount), so a survey
    # stamped with the wrong one is computed, written, and then silently
    # ignored forever — the failure is invisible from the survey side.
    from peppar_fix.position_state import resolve_mount_sn
    args.mount_sn, _mount_src = resolve_mount_sn(
        args.receiver_uid, args.mount_sn, receivers_dir=args.receivers_dir)
    if _mount_src == "default":
        log.warning(
            "mount_sn=0 by default — no mount_sn in state/receivers/%s.json.  "
            "If the engine's current mount_sn is not 0, this result will be "
            "discarded as a seed.  Run the engine once, or pass --mount-sn.",
            args.receiver_uid)
    else:
        log.info("mount_sn=%d (from %s)", args.mount_sn, _mount_src)

    if args.auto:
        return _run_auto(args)
    if args.pride:
        return _run_pride(args)
    if args.rtklib:
        return _run_rtklib(args)
    if args.baseline:
        return _run_baseline(args)

    log.error(
        "No backend selected.  Pass one of: --auto, --pride, --rtklib, "
        "--baseline.  All four are implemented today."
    )
    return 2


def _parse_near(near: str | None) -> tuple[float, float] | None:
    if not near:
        return None
    try:
        lat_s, lon_s = near.split(",")
        return float(lat_s), float(lon_s)
    except ValueError:
        raise SystemExit(f"--near must be 'LAT,LON'; got {near!r}")


def _run_auto(args) -> int:
    """Select the fastest viable backend (baseline > pride floor > rtklib)
    and either print the plan (--plan-only) or dispatch to it."""
    from peppar_fix.peppar_survey_auto import plan_auto

    plan = plan_auto(
        uid=args.receiver_uid,
        receivers_dir=args.receivers_dir,
        near=_parse_near(args.near),
        caster_host=args.caster_host,
        caster_port=args.caster_port,
        max_km=args.max_km,
    )

    log.info("auto: selected %s backend — %s", plan.backend, plan.reason)
    if args.plan_only:
        base = plan.base
        print(f"backend: {plan.backend}")
        print(f"reason:  {plan.reason}")
        if base is not None:
            print(f"base:    {base.station} @ {base.distance_km:.1f} km "
                  f"({base.base_realization})")
        return 0

    if plan.backend == "baseline":
        # Hand the discovered base to the --baseline backend.  A NOAA CORS
        # station is fetched by _run_baseline itself (by rover date); a
        # EUREF base is fetched here via S2 so --baseline sees a RINEX path.
        return _dispatch_baseline_from_plan(args, plan)
    if plan.backend == "rtklib":
        return _run_rtklib(args)
    return _run_pride(args)


def _dispatch_baseline_from_plan(args, plan) -> int:
    """Fetch the planned base RINEX (S2) and run the --baseline backend with
    the base pre-converted from its regional datum (S1)."""
    import tempfile
    from glob import glob
    from pathlib import Path

    from peppar_fix.peppar_survey_discovery import fetch_base_rinex
    from peppar_fix.peppar_survey_rtklib import doy_from_obs_name

    if not args.rinex_glob:
        log.error("--auto baseline requires --rinex-glob (rover)")
        return 2
    obs_files = sorted(glob(args.rinex_glob))
    if not obs_files:
        log.error("--rinex-glob %r matched no files", args.rinex_glob)
        return 1
    yd = doy_from_obs_name(Path(obs_files[0]))
    if yd is None:
        log.error("can't derive (year, doy) from rover filename %s "
                  "to fetch the base", obs_files[0])
        return 1
    year, doy = yd
    work_dir = Path(args.base_work_dir or os.path.join(
        tempfile.gettempdir(), "peppar-survey-baseline"))
    work_dir.mkdir(parents=True, exist_ok=True)
    base_rinex = fetch_base_rinex(plan.base, year, doy, work_dir)
    if base_rinex is None:
        log.error("failed to fetch base %s for %d/%03d; "
                  "falling back to the PRIDE floor", plan.base.station, year, doy)
        return _run_pride(args)
    args.base = str(base_rinex)
    args.base_realization = plan.base_realization
    log.info("auto: fetched base %s -> %s (%s)", plan.base.station,
             base_rinex, plan.base_realization)
    return _run_baseline(args)


def _run_pride(args) -> int:
    """Dispatch to the PRIDE backend with CLI args."""
    import tempfile
    from glob import glob
    from pathlib import Path

    from peppar_fix.arp_history import DEFAULT_MAX_SIG0_M, DEFAULT_MIN_N_OBS
    from peppar_fix.peppar_survey_pride import run_pride_backend

    if not args.rinex_glob:
        log.error("--pride requires --rinex-glob "
                  "(e.g. 'data/rinex/MadHat-*.obs')")
        return 2
    obs_files = [Path(p) for p in sorted(glob(args.rinex_glob))]
    if not obs_files:
        log.error("--rinex-glob %r matched no files", args.rinex_glob)
        return 1
    work_dir = Path(args.pride_work_dir or os.path.join(
        tempfile.gettempdir(), "peppar-survey-pride"))

    sys_attempts = tuple(s.strip() for s in args.pride_sys.split(",") if s.strip())

    return run_pride_backend(
        obs_files=obs_files,
        work_dir=work_dir,
        receiver_uid=args.receiver_uid,
        positions_dir=args.positions_dir,
        history_dir=args.history_dir,
        mount_sn=args.mount_sn,
        sys_attempts=sys_attempts,
        n_days=args.n_days,
        max_sig0_m=(args.max_sig0_m if args.max_sig0_m is not None
                    else DEFAULT_MAX_SIG0_M),
        min_n_obs=(args.min_n_obs if args.min_n_obs is not None
                   else DEFAULT_MIN_N_OBS),
        brdm_source=args.brdm_source,
        wum_source=args.wum_source,
        dry_run=args.dry_run,
    )


def _run_rtklib(args) -> int:
    """Dispatch to the RTKLIB rnx2rtkp backend, PPP-static (absolute, no base).

    The relative-baseline (RTK vs a base) path lives in --baseline / _run_baseline.
    """
    import tempfile
    from glob import glob
    from pathlib import Path

    from peppar_fix.arp_history import DEFAULT_MAX_SIG0_M, DEFAULT_MIN_N_OBS
    from peppar_fix.peppar_survey_rtklib import run_rtklib_backend

    if not args.rinex_glob:
        log.error("--rtklib requires --rinex-glob "
                  "(e.g. 'data/rinex/MadHat-*.obs')")
        return 2
    obs_files = [Path(p) for p in sorted(glob(args.rinex_glob))]
    if not obs_files:
        log.error("--rinex-glob %r matched no files", args.rinex_glob)
        return 1
    work_dir = Path(args.rtklib_work_dir or os.path.join(
        tempfile.gettempdir(), "peppar-survey-rtklib"))

    return run_rtklib_backend(
        obs_files=obs_files,
        work_dir=work_dir,
        receiver_uid=args.receiver_uid,
        mode="ppp",
        nav_file=Path(args.rtklib_nav) if args.rtklib_nav else None,
        positions_dir=args.positions_dir,
        history_dir=args.history_dir,
        mount_sn=args.mount_sn,
        n_days=args.n_days,
        max_sig0_m=(args.max_sig0_m if args.max_sig0_m is not None
                    else DEFAULT_MAX_SIG0_M),
        min_n_obs=(args.min_n_obs if args.min_n_obs is not None
                   else DEFAULT_MIN_N_OBS),
        dry_run=args.dry_run,
    )


def _run_baseline(args) -> int:
    """Dispatch to the RTKLIB rnx2rtkp backend, RTK-static relative baseline.

    Base source is exactly one of: --base (CORS station code or RINEX path) or
    --base-ntrip-host (peer caster).  The base is pre-converted to
    ITRF2020@epoch so the rover result is native ITRF2020 (I-071401).
    """
    import tempfile
    from glob import glob
    from pathlib import Path

    from peppar_fix.arp_history import DEFAULT_MAX_SIG0_M, DEFAULT_MIN_N_OBS
    from peppar_fix.peppar_survey_rtklib import run_rtklib_backend

    if not args.rinex_glob:
        log.error("--baseline requires --rinex-glob (the ROVER obs, "
                  "e.g. 'data/rinex/MadHat-*.obs')")
        return 2
    obs_files = [Path(p) for p in sorted(glob(args.rinex_glob))]
    if not obs_files:
        log.error("--rinex-glob %r matched no files", args.rinex_glob)
        return 1

    if bool(args.base) == bool(args.base_ntrip_host):
        log.error("--baseline requires exactly one base source: --base "
                  "(CORS station or RINEX path) OR --base-ntrip-host")
        return 2

    # --base is a CORS station code or a RINEX path (auto-detect).
    base_station = None
    base_rinex_path = None
    if args.base:
        looks_like_path = ("/" in args.base or "." in args.base
                           or Path(args.base).exists())
        if looks_like_path:
            base_rinex_path = Path(args.base)
        else:
            base_station = args.base

    work_dir = Path(args.base_work_dir or os.path.join(
        tempfile.gettempdir(), "peppar-survey-baseline"))
    nav_file = (Path(args.base_nav) if args.base_nav
                else Path(args.rtklib_nav) if args.rtklib_nav else None)

    base_ntrip = None
    if args.base_ntrip_host:
        from peppar_fix.peppar_survey_cors import (
            CorsNtripConfig, DEFAULT_CORS_NTRIP_DURATION_S,
        )
        base_ntrip = CorsNtripConfig(
            host=args.base_ntrip_host,
            port=args.base_ntrip_port,
            mount=args.base_ntrip_mount,
            duration_s=(args.base_ntrip_duration
                        if args.base_ntrip_duration is not None
                        else DEFAULT_CORS_NTRIP_DURATION_S),
        )

    return run_rtklib_backend(
        obs_files=obs_files,
        work_dir=work_dir,
        receiver_uid=args.receiver_uid,
        mode="rtk",
        base_station=base_station,
        base_rinex_path=base_rinex_path,
        base_realization=args.base_realization,
        base_ntrip=base_ntrip,
        nav_file=nav_file,
        positions_dir=args.positions_dir,
        history_dir=args.history_dir,
        mount_sn=args.mount_sn,
        n_days=args.n_days,
        max_sig0_m=(args.max_sig0_m if args.max_sig0_m is not None
                    else DEFAULT_MAX_SIG0_M),
        min_n_obs=(args.min_n_obs if args.min_n_obs is not None
                   else DEFAULT_MIN_N_OBS),
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    sys.exit(main())

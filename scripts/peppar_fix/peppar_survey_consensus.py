#!/usr/bin/env python3
"""peppar_survey_consensus.py — SKETCH: multi-backend consensus survey.

Design captured from the 2026-06-12 London Mini PT run (see
docs/arp-survey-strategy.md "Techniques proven on the London run").

peppar-survey runs exactly ONE backend today (--pride | --rtklib | --opus |
--cors), each of which solves *and* writes .survey.toml.  This layer instead
runs SEVERAL backends over the SAME capture, forms a **product-grade-aware,
agreement-gated mean**, and writes ONE consensus .survey.toml.  London:
PRIDE PPP-AR (WUM rapid-RTS) + CSRS-PPP (EMR ultra-rapid) agreed to 2.3 cm 3D
→ that mean became the APC; RTKLIB-with-broadcast-eph (~48 cm vertical = the
broadcast-products *signature*, not a bug) was kept only as a gross-error
sanity check, never averaged in.

Four London techniques implemented here:
  1. multi-backend consensus (mean of agreeing precise-product solvers);
  2. product-grade awareness (broadcast = sanity check, never in the mean);
  3. APC-not-ARP framing for uncalibrated antennas (apc=True);
  4. provisional-now → finals-later lifecycle (provisional=True + rerun stamp).

Wired: `peppar-survey --consensus pride,rtklib[,csrs]`.  Each backend exposes a
compute-only `solve_*_capture()` that returns its single-capture RunningArp +
grade + meta (the write half stays in `write_survey_from_running()`); this layer
collects them, gates, and writes one consensus `.survey.toml`.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field

import numpy as np

from peppar_fix.geo_frames import CANONICAL_REALIZATION, Frame
from peppar_fix.position_state import (
    DEFAULT_POSITIONS_DIR, PositionState, save_survey_state, utc_now_iso,
)

log = logging.getLogger("peppar-survey.consensus")

# Product grade decides a member's role.  Precise/rapid/final solvers form the
# MEAN; broadcast-eph solvers are a gross-error cross-check only — averaging a
# broadcast solution (dm-class vertical product signature) into a precise mean
# is exactly the mistake the London run avoided (would turn 2 cm into 50 cm).
GRADE_MEAN = {"final", "rapid", "precise"}
GRADE_SANITY = {"broadcast"}

# A sanity (broadcast) solver sitting farther than this from the consensus is a
# real pipeline bug, not a product signature → abort rather than write.
SANITY_GROSS_M = 2.0
# Default finals re-run horizon: IGS/WUM finals land ~13-20 d out.
FINALS_RERUN_DAYS = 20.0


@dataclass
class ConsensusMember:
    """One backend's single-capture solution (NOT yet written to disk)."""
    name: str                                   # "pride" | "rtklib" | "csrs"
    ecef_m: tuple[float, float, float]
    sigma_m: float                              # formal/aggregated 1σ 3D (m)
    grade: str                                  # final|rapid|precise|broadcast
    frame: Frame
    ok: bool                                    # backend's own quality gate
    meta: dict = field(default_factory=dict)    # iar%, nl/wl, sys, n_obs, products…


# --------------------------------------------------------------------------- #
#  Per-backend solve seam (the one piece needing a small backend refactor)
# --------------------------------------------------------------------------- #
def _solve_member(name: str, *, obs_files, work_dir, receiver_uid,
                  history_dir=None, mount_sn=0, dry_run=False,
                  csrs_pos=None, solve_kw=None) -> ConsensusMember | None:
    """Run ONE backend over the capture and return its single-capture solution
    WITHOUT writing .survey.toml.

      pride  → peppar_survey_pride.solve_pride_capture  (grade 'rapid'/'final')
      rtklib → peppar_survey_rtklib.solve_rtklib_capture (RTK 'precise', else
               'broadcast' = sanity-check only)
      csrs   → ingest an operator-downloaded CSRS-PPP result (--csrs-result);
               CSRS REST is unreliable (opaque SEVERE errors) so transcribe,
               don't auto-submit.

    `solve_kw` is {backend: {extra kwargs}} forwarded to the solve function.
    """
    from pathlib import Path

    from peppar_fix.position_state import decimal_year_from_mjd
    kw = (solve_kw or {}).get(name, {})

    if name == "csrs":
        return _ingest_csrs(csrs_pos, **kw)

    wd = Path(work_dir) / name          # isolate per-backend scratch
    # Isolate each backend's history too — otherwise pride and rtklib would
    # append to the SAME history.jsonl (keyed by receiver_uid) and their
    # running means would cross-contaminate.
    hist = str(Path(history_dir) / name) if history_dir else str(wd)
    if name == "pride":
        from peppar_fix.peppar_survey_pride import solve_pride_capture
        running, grade, meta = solve_pride_capture(
            obs_files, wd, receiver_uid, history_dir=hist,
            mount_sn=mount_sn, dry_run=dry_run, **kw)
    elif name == "rtklib":
        from peppar_fix.peppar_survey_rtklib import solve_rtklib_capture
        running, grade, meta = solve_rtklib_capture(
            obs_files, wd, receiver_uid, history_dir=hist,
            mount_sn=mount_sn, dry_run=dry_run, **kw)
    else:
        log.error("unknown consensus backend %r (use pride|rtklib|csrs)", name)
        return None

    if running is None:
        return ConsensusMember(name, (0.0, 0.0, 0.0), float("inf"), grade,
                               Frame(CANONICAL_REALIZATION, None),
                               ok=False, meta=meta)
    mid_mjd = 0.5 * (running.oldest_mjd + running.newest_mjd)
    meta["window_count"] = running.count
    meta["epoch_decimal_year"] = decimal_year_from_mjd(mid_mjd)
    return ConsensusMember(
        name, tuple(running.ecef_m), float(running.sigma_3d_m), grade,
        Frame(CANONICAL_REALIZATION, decimal_year_from_mjd(mid_mjd)),
        ok=True, meta=meta)


def _ingest_csrs(path, *, grade: str = "rapid",
                 epoch_decimal_year=None) -> ConsensusMember | None:
    """Ingest an operator-downloaded CSRS-PPP result as a consensus member.

    CSRS-PPP's REST endpoint returns opaque SEVERE errors; the reliable path is
    the operator's browser upload, then transcribe the report into a small TOML
    stub::

        ecef_m  = [3979160.4832, -4257.8608, 4968043.1627]   # m, ITRF2020
        sigma_m = 0.0093                                       # 3D 1σ (σ95/1.96)
        iar_pct = 89.5                                         # optional
        grade   = "rapid"                                      # or "final"
    """
    if not path:
        log.warning("csrs member needs --csrs-result <toml>; skipping")
        return None
    try:
        import tomllib
    except ModuleNotFoundError:                       # py < 3.11
        import tomli as tomllib                        # type: ignore
    try:
        with open(path, "rb") as f:
            d = tomllib.load(f)
        ecef = tuple(float(v) for v in d["ecef_m"])
        sigma = float(d["sigma_m"])
    except (OSError, KeyError, ValueError, TypeError) as e:
        log.error("csrs result %s unreadable: %s", path, e)
        return None
    meta = {k: d[k] for k in ("iar_pct", "n_obs", "product") if k in d}
    return ConsensusMember(
        "csrs", ecef, sigma, str(d.get("grade", grade)),
        Frame(CANONICAL_REALIZATION,
              d.get("epoch_decimal_year", epoch_decimal_year)),
        ok=True, meta=meta)


# --------------------------------------------------------------------------- #
#  Consensus
# --------------------------------------------------------------------------- #
def run_consensus(*, backends, obs_files, work_dir, receiver_uid,
                  positions_dir=None, history_dir=None, mount_sn=0,
                  tol_3d_cm=3.0, apc=False, provisional=False,
                  csrs_pos=None, solve_kw=None, dry_run=False) -> int:
    """Run `backends` over one capture, write the agreement-gated mean.

    backends: ordered list, e.g. ['pride', 'rtklib', 'csrs'].
    solve_kw: {backend: {extra solve kwargs}} forwarded to each solve_*_capture.
    Returns a process exit code (0 ok, 1 no-consensus/failure).
    """
    members: list[ConsensusMember] = []
    for name in backends:
        try:
            m = _solve_member(name, obs_files=obs_files, work_dir=work_dir,
                              receiver_uid=receiver_uid, history_dir=history_dir,
                              mount_sn=mount_sn, dry_run=dry_run,
                              csrs_pos=csrs_pos, solve_kw=solve_kw)
        except Exception as e:                       # one bad backend ≠ fatal
            log.error("backend %s failed: %s", name, e)
            m = None
        if m and m.ok:
            members.append(m)
            log.info("%-7s %s grade  ECEF=%.4f,%.4f,%.4f  σ=%.3fm",
                     m.name, m.grade, *m.ecef_m, m.sigma_m)
        else:
            log.warning("%-7s no usable solution — excluded", name)

    mean_set = [m for m in members if m.grade in GRADE_MEAN]
    sanity_set = [m for m in members if m.grade in GRADE_SANITY]

    if len(mean_set) < 2:
        log.error("consensus needs ≥2 precise-product solvers; got %d "
                  "(broadcast-only is a sanity check, never the mean). "
                  "Falling back to a single backend is the caller's choice.",
                  len(mean_set))
        return 1

    P = np.array([m.ecef_m for m in mean_set])                      # (N, 3)
    # --- agreement gate: max pairwise 3D separation ≤ tol --- #
    pairwise = [float(np.linalg.norm(P[i] - P[j]))
                for i in range(len(P)) for j in range(i + 1, len(P))]
    dmax = max(pairwise)
    tol = tol_3d_cm / 100.0
    if dmax > tol:
        log.error("CONSENSUS FAILED: max inter-solver 3D %.1f cm > %.1f cm tol "
                  "(%s). Not writing a survey.", dmax * 100, tol_3d_cm,
                  ", ".join(m.name for m in mean_set))
        for m in mean_set:
            log.error("  %-7s %.4f %.4f %.4f", m.name, *m.ecef_m)
        return 1

    mean = P.mean(axis=0)
    spread_3d = math.sqrt(np.mean(np.sum((P - mean) ** 2, axis=1)))
    formal = max(m.sigma_m for m in mean_set)
    sigma_3d = math.sqrt(formal ** 2 + spread_3d ** 2)   # formal ⊕ inter-solver spread

    # --- product-grade sanity: broadcast solver vs the precise mean --- #
    excluded = {}
    for s in sanity_set:
        off = float(np.linalg.norm(np.array(s.ecef_m) - mean))
        excluded[s.name] = round(off, 4)
        if off > SANITY_GROSS_M:
            log.error("sanity solver %s is %.2f m off the consensus (> %.1f m) "
                      "— gross error, not a product signature. Aborting.",
                      s.name, off, SANITY_GROSS_M)
            return 1
        log.info("sanity %-7s %.1f cm off consensus (broadcast-products "
                 "signature; excluded from mean — correct).", s.name, off * 100)

    # --- provenance (the auditable .survey.toml schema from London) --- #
    extra = {
        "consensus_3d_cm": round(spread_3d * 100, 3),
        "consensus_2d_cm": round(float(np.linalg.norm(
            _enu(P, mean)[:, :2].std(axis=0))) * 100, 3),
        "consensus_n_solvers": len(mean_set),
        "consensus_tol_cm": tol_3d_cm,
    }
    for m in mean_set:                        # per-solver coords + fixing metadata
        extra[f"{m.name}_x_m"], extra[f"{m.name}_y_m"], extra[f"{m.name}_z_m"] = (
            round(m.ecef_m[0], 5), round(m.ecef_m[1], 5), round(m.ecef_m[2], 5))
        extra.update({f"{m.name}_{k}": v for k, v in m.meta.items()})
    for name, off in excluded.items():
        extra[f"{name}_excluded_reason"] = (
            f"broadcast-eph PPP at {off*100:.0f} cm off the precise consensus is "
            f"the broadcast-products signature, not a bug; sanity-check only.")
    if apc:
        extra["kind_note"] = (
            "APC, not ARP: antenna uncalibrated (zero-PCV applied). Adopt as APC "
            "reference; do not chase the absence of antex as a bias.")
    if provisional:
        extra["provisional"] = True
        extra["finals_rerun_after_iso"] = _plus_days_iso(FINALS_RERUN_DAYS)

    state = PositionState(
        mount_sn=mount_sn,
        ecef_m=tuple(mean),                                    # type: ignore[arg-type]
        sigma_m=sigma_3d,
        updated=utc_now_iso(),
        source="consensus: " + " + ".join(
            f"{m.name}({m.grade})" for m in mean_set),
        frame=_consensus_frame(mean_set),
        kind="survey",
        extra=extra,
    )

    path = f"{(positions_dir or DEFAULT_POSITIONS_DIR)}/{receiver_uid}.survey.toml"
    if dry_run:
        log.info("DRY RUN — consensus %.1f cm 3D spread, σ_3d=%.1f mm from %d "
                 "solvers; would write %s", spread_3d * 100, sigma_3d * 1000,
                 len(mean_set), path)
        return 0
    save_survey_state(state, receiver_uid, positions_dir=positions_dir)
    log.info("wrote %s — σ_3d=%.1f mm, consensus spread %.1f cm, solvers: %s%s",
             path, sigma_3d * 1000, spread_3d * 100,
             ", ".join(m.name for m in mean_set),
             "  [PROVISIONAL]" if provisional else "")
    return 0


# --------------------------------------------------------------------------- #
#  helpers
# --------------------------------------------------------------------------- #
def _enu(P, mean):
    """ECEF deviations rotated to local ENU about `mean` (for the 2-D number)."""
    lat = math.atan2(mean[2], math.hypot(mean[0], mean[1]))
    lon = math.atan2(mean[1], mean[0])
    sl, cl, so, co = (math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon))
    R = np.array([[-so, co, 0.0],
                  [-sl * co, -sl * so, cl],
                  [cl * co, cl * so, sl]])
    return (P - mean) @ R.T


def _consensus_frame(members) -> Frame:
    """All precise PPP products realize ITRF2020/IGS20; stamp the capture epoch
    (mean of the members' epochs; current epoch if none carry one)."""
    eps = [m.frame.epoch for m in members
           if getattr(m, "frame", None) and m.frame.epoch is not None]
    if eps:
        return Frame(CANONICAL_REALIZATION, sum(eps) / len(eps))
    from datetime import datetime, timezone
    now = datetime.now(timezone.utc)
    y0 = datetime(now.year, 1, 1, tzinfo=timezone.utc)
    y1 = datetime(now.year + 1, 1, 1, tzinfo=timezone.utc)
    return Frame(CANONICAL_REALIZATION, now.year + (now - y0) / (y1 - y0))


def _plus_days_iso(days: float) -> str:
    """Reminder horizon for the finals re-run (now + days, UTC)."""
    from datetime import datetime, timedelta, timezone
    return (datetime.now(timezone.utc) + timedelta(days=days)).strftime(
        "%Y-%m-%dT%H:%M:%SZ")

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

STATUS: the consensus layer (gate / grade split / mean / provenance / write)
is complete.  The one integration seam is `_solve_member()` — each existing
backend needs a compute-only entry point that RETURNS its single-capture
solution instead of writing.  That's a small refactor of run_pride_backend /
run_rtklib_backend (factor the solve+running_mean part out of the write part,
which already lives in their separate write_survey_from_running()).  NOT wired
into the peppar_survey.py CLI yet.
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
def _solve_member(name: str, *, obs_files, work_dir, apc: bool,
                  csrs_pos=None, **kw) -> ConsensusMember | None:
    """Run ONE backend over the capture and return its single-capture solution
    WITHOUT writing .survey.toml.

    Integration points (factor the solve part out of each run_*_backend; the
    write half already lives in write_survey_from_running()):

      pride  → peppar_survey_pride.solve_capture(obs_files, …, antex=None if apc)
               returns RunningArp(+meta); WUM-RTS ⇒ grade 'rapid', finals ⇒ 'final'
      rtklib → peppar_survey_rtklib.solve_capture(obs_files, …)
               sateph=brdc ⇒ grade 'broadcast'; precise SP3/CLK ⇒ 'precise'
      csrs   → parse an operator-downloaded CSRS-PPP .pos (--csrs-pos); grade per
               its product line (ultra-rapid⇒'rapid', final⇒'final').  CSRS REST
               is unreliable (opaque SEVERE errors) so ingest, don't auto-submit.

    `apc=True` ⇒ ANT=NONE / zero-PCV (uncalibrated antenna ⇒ APC not ARP).
    """
    if name == "csrs":
        if not csrs_pos:
            log.warning("csrs member needs --csrs-pos <downloaded .pos>; skipping")
            return None
        # parse_pos / a CSRS reader → ecef + σ95 + IAR%; grade from the header
        raise NotImplementedError("ingest CSRS .pos via pride_pos_reader/CSRS parser")
    # pride / rtklib: call the (to-be-added) compute-only solve and wrap it.
    raise NotImplementedError(
        f"wire {name}: add solve_capture(...) -> (RunningArp, grade, meta) to "
        f"peppar_survey_{name}, then build ConsensusMember from it")


# --------------------------------------------------------------------------- #
#  Consensus
# --------------------------------------------------------------------------- #
def run_consensus(*, backends, obs_files, work_dir, receiver_uid,
                  positions_dir=None, mount_sn=0, tol_3d_cm=3.0,
                  apc=False, provisional=False, csrs_pos=None,
                  dry_run=False, **kw) -> int:
    """Run `backends` over one capture, write the agreement-gated mean.

    backends: ordered list, e.g. ['pride', 'rtklib', 'csrs'].
    Returns a process exit code (0 ok, 1 no-consensus/failure).
    """
    members: list[ConsensusMember] = []
    for name in backends:
        try:
            m = _solve_member(name, obs_files=obs_files, work_dir=work_dir,
                              apc=apc, csrs_pos=csrs_pos, mount_sn=mount_sn, **kw)
        except NotImplementedError as e:
            log.error("backend %s not wired: %s", name, e)
            m = None
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
    """All precise PPP products realize ITRF2020/IGS20; stamp the capture epoch."""
    ep = [m.meta.get("epoch_decimal_year") for m in members
          if m.meta.get("epoch_decimal_year")]
    return Frame(CANONICAL_REALIZATION, sum(ep) / len(ep) if ep else None)


def _plus_days_iso(days: float) -> str:
    # NOTE: real impl stamps capture_end + days; kept seam-simple in the sketch.
    raise NotImplementedError("stamp capture_end_iso + days for the finals rerun")

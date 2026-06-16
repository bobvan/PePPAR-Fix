"""Q[3,3]-source A/B harness — gates the doCharacterizationArchitecture
σ_do_freq change before it touches engine code.

Context (docs/q-matrix-source-change-sim.md, docs/do-characterization-
architecture.md §"Q[3,3] design change"):

    OLD  σ_do_freq derived from the disciplined adjfine/freq_command ASD
         (a control-loop signal).  On PiFace this channel-mixing produced
         σ_do_freq = 0.1975 ppb/√s — 517× looser than correct.
    NEW  σ_do_freq derived ONLY from the freerun-ADEV RWFM tail
         (0.000382 ppb/√s on clkPoC3).  The disciplined adjfine ASD is
         reframed as σ_q under [actuation_noise] and never feeds Q[3,3].

This module validates that change empirically.  It is **not** one tool but
two, because no single tool faithfully reproduces both the over-loose and
over-tight regimes:

  * Open-loop **trace replay** (`replay_servo_csv`) — feeds the *recorded*
    per-epoch measurements back through a fresh ``DOFreqEst`` under a chosen
    Q[3,3].  Faithful exactly when the original loop was stable (clkPoC3
    recorded adjfine std 3.95 ppb → replay 4.87 ppb, 23%).  It deliberately
    *diverges* on a near-unstable trace (PiFace OLD Q → 61 863 ppb) because
    it strips the engine's reset-cascade containment (ramp_reacquire +
    ResetBudget + exit-5 + wrapper relaunch) that held the field wobble to
    60 ppb.  That is why Arm 1 grades a **relative reduction ratio** under
    identical recorded inputs, not an absolute adjfine std — the absolute
    60 ppb was an artifact of the reset cascade, not a property of the Q
    change being validated.

  * Closed-loop **synthetic sim** (``servo_sim.ClosedLoopSim``) — has plant
    feedback, so it can run a controllable wandering-DO over-tight contrast
    without the open-loop divergence.  Used for the discriminating half of
    the over-tight arm.

Three checks, each on the tool that is faithful for it:

  1. FAITHFULNESS ANCHOR (`faithfulness_anchor`) — clkPoC3 replay reproduces
     its recorded adjfine std within 30%.  Proves the column mapping +
     DOFreqEst wiring are correct on a stable host before any A/B is trusted.

  2. OVER-LOOSE ARM (`over_loose_arm`) — under PiFace's recorded measurement
     environment, NEW Q reduces the replayed frequency-state chasing by
     ≥10× vs OLD Q (≈43× measured), AND on clkPoC3 (clean env) NEW Q leaves
     the healthy host within 30% of recorded (no penalty).

  3. OVER-TIGHT ARM — two parts:
     a. `over_tight_arm_replay` — clkPoC3 real-trace 15-min coast under NEW
        Q: √P[2,2] grows bounded, the first post-gap measurements are
        accepted (χ² < 100), and the frequency estimate recovers within a
        few epochs.  Faithful real-data PASS, but clkPoC3's DO is so stable
        it doesn't wander during a coast, so this half can't *discriminate*
        an overconfident Q.
     b. `over_tight_arm_synthetic` — a controllable wandering-DO sim where a
        matched NEW Q recovers from a measurement gap with a smaller
        post-gap excursion than an overconfident (100× tighter) Q, and
        neither diverges (the engine's adaptive-Q boost + χ² gate +
        sole-observer admit provide margin against Q misspecification —
        which is itself the reassuring over-tight finding).

PASS overall = all three arms pass.  This is the gate the design doc
requires before the engine-integration PR (step 3, PR 2) lands.

See the dayplan item ``doSchemaEngineIntegration`` (I-192603-charlie).
"""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from peppar_fix.do_freq_est import DOFreqEst


# Canonical Q[3,3]^0.5 values argued in the design doc / postmortem.
OLD_SIGMA_DO_FREQ_PPB = 0.1975      # PiFace channel-mixed (control-loop ASD)
NEW_SIGMA_DO_FREQ_PPB = 0.000382    # clkPoC3 measured freerun-ADEV RWFM tail
# σ_do_phase paired with the NEW source (clkPoC3 measured); the over-loose
# arm holds this fixed across OLD/NEW so only Q[3,3] varies.
DEFAULT_SIGMA_DO_PHASE_NS = 0.0425


# ── CSV ingest ──────────────────────────────────────────────────────── #

# Columns consumed from the engine's servo-overnight.csv (one row per
# servo.update()).  Mapping mirrors the engine call site
# (peppar_fix_engine.py:8886 adjfine_ppb = -servo.update(...) and :8704
# ticc_sigma_ns = max(ticc_confidence, 0.001)).
def _f(row: dict, key: str) -> Optional[float]:
    v = row.get(key, "")
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load_servo_csv(path: str) -> list[dict]:
    """Load a servo-overnight.csv into a list of row dicts."""
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


@dataclass
class ReplayResult:
    """Per-epoch outputs of an open-loop replay."""
    adjfine_ppb: np.ndarray         # = -servo.update() each measured epoch
    chi2_ticc: np.ndarray           # Arm-4 χ² each epoch (NaN if not fired)
    sqrt_p22_ns: np.ndarray         # √P[2,2] each epoch (DO-phase uncertainty)
    est_freq_ppb: np.ndarray        # x[3] each epoch (DO freq estimate)
    gap_start_idx: Optional[int] = None
    gap_len: int = 0

    def adjfine_std(self, skip_frac: float = 0.0) -> float:
        a = self.adjfine_ppb[int(len(self.adjfine_ppb) * skip_frac):]
        a = a[np.isfinite(a)]
        return float(np.std(a)) if len(a) else float("nan")


def replay_servo_csv(rows: list[dict], sigma_do_freq_ppb: float,
                     sigma_do_phase_ns: float = DEFAULT_SIGMA_DO_PHASE_NS,
                     gap_start_idx: Optional[int] = None,
                     gap_len: int = 0) -> ReplayResult:
    """Replay recorded measurements through a fresh DOFreqEst under one Q[3,3].

    OPEN-LOOP: the plant does not respond to the recomputed adjfine, so this
    is faithful only when the original loop was stable.  See module docstring.

    A measurement gap can be injected with ``gap_start_idx`` / ``gap_len``:
    for epochs inside the gap the filter runs predict-only (no arm fires),
    modelling a GNSS outage so the over-tight arm can probe coast / recovery.
    """
    if not rows:
        raise ValueError("replay_servo_csv: empty rows")
    r0 = rows[0]
    adj0 = _f(r0, "adjfine_ppb") or 0.0
    dtrx0 = _f(r0, "dt_rx_ns")
    ekf = DOFreqEst(
        sigma_do_freq_ppb=sigma_do_freq_ppb,
        sigma_do_phase_ns=sigma_do_phase_ns,
        initial_freq=adj0, base_freq=adj0,
        initial_dt_rx_ns=dtrx0 if dtrx0 is not None else 0.0)

    n = len(rows)
    adj = np.full(n, np.nan)
    chi2 = np.full(n, np.nan)
    sqrt_p22 = np.full(n, np.nan)
    est_freq = np.full(n, np.nan)

    for i, r in enumerate(rows):
        dt = _f(r, "discipline_interval") or 1.0
        in_gap = (gap_start_idx is not None
                  and gap_start_idx <= i < gap_start_idx + gap_len)
        if in_gap:
            ekf.update(dt=dt)  # predict-only — P grows, adjfine held
        else:
            kw: dict = {"dt": dt}
            ticc = _f(r, "pps_err_ticc_ns")
            tconf = _f(r, "ticc_confidence")
            if ticc is not None and tconf is not None:
                kw["ticc_diff_ns"] = ticc
                kw["ticc_sigma_ns"] = max(tconf, 0.001)
            dtrx = _f(r, "dt_rx_ns")
            dtrxs = _f(r, "dt_rx_sigma_ns")
            if dtrx is not None and dtrxs is not None:
                kw["dt_rx_ns"] = dtrx
                kw["dt_rx_sigma_ns"] = dtrxs
            # qErr arm intentionally not fed: F9P/X20P HPG firmware report
            # qErr = 0 (qerr_ns column empty on both PiFace and clkPoC3).
            adj[i] = -ekf.update(**kw)

        iv = ekf.last_arm_innov.get("ticc")
        S = ekf.last_arm_S.get("ticc")
        if iv is not None and S:
            chi2[i] = iv * iv / S
        sqrt_p22[i] = math.sqrt(max(0.0, float(ekf.P[2, 2])))
        est_freq[i] = ekf.estimated_freq_ppb

    return ReplayResult(adjfine_ppb=adj, chi2_ticc=chi2, sqrt_p22_ns=sqrt_p22,
                        est_freq_ppb=est_freq, gap_start_idx=gap_start_idx,
                        gap_len=gap_len)


def recorded_adjfine_std(rows: list[dict]) -> float:
    """Std of the adjfine_ppb the engine actually commanded (ground truth)."""
    a = np.array([v for v in (_f(r, "adjfine_ppb") for r in rows)
                  if v is not None])
    return float(np.std(a)) if len(a) else float("nan")


# ── Arm results ─────────────────────────────────────────────────────── #

@dataclass
class ArmResult:
    name: str
    passed: bool
    detail: dict = field(default_factory=dict)

    def __str__(self) -> str:
        flag = "PASS" if self.passed else "FAIL"
        kv = "  ".join(f"{k}={v}" for k, v in self.detail.items())
        return f"[{flag}] {self.name}: {kv}"


def faithfulness_anchor(clkpoc3_rows: list[dict], own_q: float,
                        tol_frac: float = 0.30) -> ArmResult:
    """clkPoC3 replay must reproduce its recorded adjfine std within tol_frac.

    `own_q` is clkPoC3's production σ_do_freq (its measured freerun value),
    NOT the OLD channel-mixed value — the anchor checks the harness is
    faithful on a stable host running its real Q.
    """
    rec = recorded_adjfine_std(clkpoc3_rows)
    rep = replay_servo_csv(clkpoc3_rows, own_q).adjfine_std()
    rel = abs(rep - rec) / rec if rec else float("inf")
    return ArmResult(
        "faithfulness_anchor(clkPoC3)", rel <= tol_frac,
        {"recorded_std_ppb": round(rec, 2), "replay_std_ppb": round(rep, 2),
         "rel_err": round(rel, 3), "tol": tol_frac})


def over_loose_arm(piface_rows: list[dict], clkpoc3_rows: list[dict],
                   old_q: float = OLD_SIGMA_DO_FREQ_PPB,
                   new_q: float = NEW_SIGMA_DO_FREQ_PPB,
                   min_reduction: float = 10.0,
                   stable_tol_frac: float = 0.30) -> ArmResult:
    """NEW Q tames PiFace's loose-Q chasing AND leaves clkPoC3 unpenalized.

    Graded as a RELATIVE ratio under identical recorded inputs — robust to
    the open-loop replay caveat that makes absolute PiFace magnitudes
    meaningless (the recorded 60 ppb was reset-cascade-contained).
    """
    pf_old = replay_servo_csv(piface_rows, old_q).adjfine_std()
    pf_new = replay_servo_csv(piface_rows, new_q).adjfine_std()
    reduction = pf_old / pf_new if pf_new else float("inf")

    clk_rec = recorded_adjfine_std(clkpoc3_rows)
    clk_new = replay_servo_csv(clkpoc3_rows, new_q).adjfine_std()
    clk_rel = abs(clk_new - clk_rec) / clk_rec if clk_rec else float("inf")

    passed = reduction >= min_reduction and clk_rel <= stable_tol_frac
    return ArmResult(
        "over_loose_arm", passed,
        {"piface_old_std_ppb": round(pf_old, 1),
         "piface_new_std_ppb": round(pf_new, 1),
         "reduction_x": round(reduction, 1), "min_reduction": min_reduction,
         "clkpoc3_new_vs_recorded_rel": round(clk_rel, 3),
         "stable_tol": stable_tol_frac})


def over_tight_arm_replay(clkpoc3_rows: list[dict],
                          new_q: float = NEW_SIGMA_DO_FREQ_PPB,
                          gap_frac: float = 0.6, gap_len: int = 900,
                          chi2_max: float = 100.0,
                          recovery_max_epochs: int = 30,
                          recovery_tol_ppb: float = 0.05) -> ArmResult:
    """clkPoC3 real-trace coast under NEW Q: bounded √P22, post-gap accepted,
    fast freq recovery.  Faithful real-data PASS (not a sharp discriminator —
    clkPoC3's DO barely wanders over a 15-min coast; see _synthetic below)."""
    n = len(clkpoc3_rows)
    gs = int(n * gap_frac)
    res = replay_servo_csv(clkpoc3_rows, new_q,
                           gap_start_idx=gs, gap_len=gap_len)
    p22_pre = res.sqrt_p22_ns[gs - 1]
    p22_gap_max = float(np.nanmax(res.sqrt_p22_ns[gs:gs + gap_len]))
    post = res.chi2_ticc[gs + gap_len:gs + gap_len + 10]
    post = post[np.isfinite(post)][:5]
    post_accepted = bool(len(post)) and bool((post < chi2_max).all())

    f_base = np.nanmedian(res.est_freq_ppb[gs - 60:gs])
    ferr = np.abs(res.est_freq_ppb[gs + gap_len:] - f_base)
    rec_idx = int(np.argmax(ferr < recovery_tol_ppb)) if (
        ferr < recovery_tol_ppb).any() else -1

    p22_bounded = math.isfinite(p22_gap_max) and p22_gap_max < 1e3
    recovered = 0 <= rec_idx <= recovery_max_epochs
    passed = p22_bounded and post_accepted and recovered
    return ArmResult(
        "over_tight_arm_replay(clkPoC3)", passed,
        {"p22_pre_ns": round(float(p22_pre), 4),
         "p22_gap_max_ns": round(p22_gap_max, 3),
         "post_gap_chi2": [round(float(c), 1) for c in post],
         "post_accepted": post_accepted,
         "recovery_epochs": rec_idx})


def over_tight_arm_synthetic(new_q: float = 0.02,
                             overtight_q: Optional[float] = None,
                             rwfm_ppb_per_sqrt_s: float = 0.02,
                             gap: tuple = (1500.0, 2400.0),
                             seed: int = 7) -> ArmResult:
    """Controllable wandering-DO over-tight contrast (closed-loop sim).

    A matched NEW Q must recover from a measurement gap with a SMALLER
    post-gap excursion than an overconfident (100× tighter) Q, and NEITHER
    may diverge.  Uses servo_sim so the DO genuinely wanders during the gap
    (the discriminating stress clkPoC3's real trace can't supply).
    """
    # Imported lazily so the replay path has no servo_sim dependency.
    from peppar_fix.servo_sim import ClosedLoopSim, SimConfig
    if overtight_q is None:
        overtight_q = new_q / 100.0

    def _run(sdf: float):
        cfg = SimConfig(
            label="over_tight", do_f0_ppb=21.0, do_wfm_ppb=0.079,
            do_wpm_ns=0.01, do_rwfm_ppb_per_sqrt_s=rwfm_ppb_per_sqrt_s,
            rx_f0_ppb=3.0, use_ppp=True, use_qerr=True, use_ticc=True,
            sigma_do_freq_ppb=sdf, sigma_do_phase_ns=DEFAULT_SIGMA_DO_PHASE_NS,
            duration_s=3500.0, seed=seed, coast_interval_s=1.0,
            drop_measurements_window_s=gap)
        res = ClosedLoopSim(cfg).run()
        post = (res.t_s > gap[1]) & (res.t_s < gap[1] + 30.0)
        iv = res.ticc_innov_ns[post]
        iv = iv[np.isfinite(iv)]
        return dict(
            diverged=res.diverged(),
            post_max=float(np.max(np.abs(iv))) if len(iv) else float("nan"))

    matched = _run(new_q)
    over = _run(overtight_q)
    passed = (not matched["diverged"] and not over["diverged"]
              and matched["post_max"] <= over["post_max"])
    return ArmResult(
        "over_tight_arm_synthetic", passed,
        {"matched_diverged": matched["diverged"],
         "overtight_diverged": over["diverged"],
         "matched_post_max_ns": round(matched["post_max"], 1),
         "overtight_post_max_ns": round(over["post_max"], 1)})


# ── CLI ─────────────────────────────────────────────────────────────── #

def run_full_ab(piface_csv: str, clkpoc3_csv: str,
                clkpoc3_own_q: float = NEW_SIGMA_DO_FREQ_PPB) -> list[ArmResult]:
    """Run all three arms against the real traces.  Returns arm results."""
    pf = load_servo_csv(piface_csv)
    clk = load_servo_csv(clkpoc3_csv)
    return [
        faithfulness_anchor(clk, clkpoc3_own_q),
        over_loose_arm(pf, clk),
        over_tight_arm_replay(clk),
        over_tight_arm_synthetic(),
    ]


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(
        description="Q[3,3]-source A/B gate (doCharacterizationArchitecture)")
    ap.add_argument("--piface-csv", required=True,
                    help="PiFace servo-overnight.csv (over-loose arm)")
    ap.add_argument("--clkpoc3-csv", required=True,
                    help="clkPoC3 servo-overnight.csv (faithfulness + arms)")
    ap.add_argument("--clkpoc3-own-q", type=float,
                    default=NEW_SIGMA_DO_FREQ_PPB,
                    help="clkPoC3 production sigma_do_freq for the anchor")
    args = ap.parse_args(argv)

    results = run_full_ab(args.piface_csv, args.clkpoc3_csv,
                          args.clkpoc3_own_q)
    print("Q[3,3]-source A/B — doCharacterizationArchitecture gate\n")
    for r in results:
        print(r)
    all_pass = all(r.passed for r in results)
    print(f"\nOVERALL: {'PASS — Q[3,3] change cleared' if all_pass else 'FAIL'}")
    return 0 if all_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())

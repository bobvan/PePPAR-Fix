# pride-harness-ar — branch status and cleanup plan

Last updated: 2026-04-23 by Agent Bravo.

This branch was the working tree for a one-day arc of PRIDE-yardstick
investigation on the PePPAR Fix observation model.  Agent Main
selectively ported the production-worthy pieces to `main`; this branch
remains the diagnostic test bed.

## Where each commit went

15 commits ahead of merge-base `7790dbd`:

| Commit    | Summary                                           | Ported to main? |
|---        |---                                                |---              |
| `09e32b8` | MW slip + `--wl-only` + convergence ladder        | No (harness only) |
| `f0e4d83` | **SP3.sat_position safe-window bounds check**     | **NO — see ASK below** |
| `f506744` | `test_sp3_sat_position.py` (8 tests)              | **NO — pairs with f0e4d83** |
| `8d16dfd` | `--systems` + `--position-csv` CSV dumps          | No (harness only) |
| `6f95186` | Phase integration + `--residuals-csv`             | No (harness only) |
| `a7b5ee2` | `--state-csv` filter state dump                   | No (harness only) |
| `6c2f4f8` | Hard gate for single-constellation + SP3          | Soft warning on engine (cb44f4e) |
| `d1edb9f` | Per-constellation `--profile gps:l2,gal:l5`       | No (harness only) |
| `079c392` | `--ztd-tie` / `--mean-amb-tie` diagnostics        | No (falsified) |
| `f11a880` | `sigma_3d_m` / `sigma_clk_m` / `sigma_ztd_m` in state-csv | No (harness only) |
| `44a7e49` | IERS 2010 solid Earth tide + `--solid-tide`       | Yes (engine `7dc3bb2`), module path moved |
| `d95d880` | `--q-pos-converged`, `--sigma-pr`, `--sigma-phi` overrides | No (harness only) |
| `b28e66a` | `--ztd-ou-tau` / `--ztd-ou-sigma` diagnostics     | No (falsified) |
| `73664d4` | ANTEX parser + `--antex` flag + PCV corrections   | Yes (engine `f7da44e`), module path moved |
| `20f3826` | Nominal yaw-steering sat body frame               | Yes (folded into `f7da44e`) |

See `git log 7790dbd..pride-harness-ar` for full commit messages.

## Open ask to Agent Main

**Cherry-pick `f0e4d83` and `f506744` into main.**  Shared code
(`scripts/solve_pseudorange.py`) is missing the SP3 safe-window
bounds check.  Extrapolation past the SP3 file's coverage
diverges (1231 m blow-up on ABMF 2020/001 motivated the fix).
Engine doesn't call SP3 today but it's latent for any future
consumer.  Five LOC + pre-existing 190-line test file.

Memo: `project_to_main_sp3_bounds_cherry_pick_ask_20260423.md`.

## Known duplications with main (deal with at next rebase)

Main added two root-level modules that duplicate what's in this
branch's `scripts/regression/`:

| This branch                      | Main                 |
|---                               |---                   |
| `scripts/regression/solid_tide.py` | `scripts/solid_tide.py` |
| `scripts/regression/antex.py`    | `scripts/antex.py`    |

Main's versions are the authoritative ones (they have tests at
`scripts/peppar_fix/test_solid_tide.py` and `test_antex.py`, plus
engine integration in `peppar_fix_engine.py`).

## Cleanup plan for the next rebase

When this branch is next picked up:

1. **Rebase on main.**  Expect clean rebase — most commits touch
   `scripts/regression/run_regression.py` + `solve_ppp.py` where
   main's edits don't overlap.  σ_phi=0.30 landed-then-reverted
   on main so there's no conflict there.

2. **Delete the duplicated modules** after rebase brings in
   main's `scripts/solid_tide.py` and `scripts/antex.py`:
   ```
   git rm scripts/regression/solid_tide.py
   git rm scripts/regression/antex.py
   ```

3. **Update harness imports** from the regression package to the
   root.  Grep for `from regression.solid_tide` and
   `from regression.antex` in `scripts/regression/run_regression.py`
   — should be a handful of lines — and rewrite as
   `from solid_tide` / `from antex`.

4. **Verify tests still pass**:
   ```
   PYTHONPATH=scripts python3 -m unittest scripts.regression.test_run_regression -v
   PYTHONPATH=scripts python3 -m unittest scripts.regression.test_sp3_sat_position -v
   PYTHONPATH=scripts python3 -m unittest scripts.peppar_fix.test_solid_tide -v
   PYTHONPATH=scripts python3 -m unittest scripts.peppar_fix.test_antex -v
   ```

5. **One commit** covering the delete + import rename.  Keeps the
   history linear and easy to follow.

## Diagnostic flags intentionally kept as dead tools

These flags are on this branch's `run_regression.py` but their
hypotheses were falsified; they remain available as diagnostic
tooling for future investigations that want to re-test the ideas
with different parameters or stations:

- `--ztd-tie SIGMA` / `--mean-amb-tie SIGMA` — per-epoch ZTD /
  mean-ambiguity pseudo-measurement.  Falsified because
  σ_posterior = σ_obs/√N shrinks with epochs, strangling the
  real 18 cm atmospheric ZTD signal at any σ that's tight
  enough to constrain the null mode.
- `--ztd-ou-tau SECONDS` / `--ztd-ou-sigma SIGMA_M` — OU process
  model for ZTD.  Falsified because the mean reversion to 0
  fights real atmospheric tracking regardless of τ (tested over
  1 h–24 h × σ 0.05–0.50 m, all regressed the clean baseline).
- `--q-pos-converged VAR` / `--sigma-pr SIGMA_M` /
  `--sigma-phi SIGMA_M` — filter-tuning knob overrides.  These
  informed the σ_phi=0.30 proposal (which also didn't transfer
  to lab, per main's `bb01b78` revert) but remain useful for
  running Q2-style σ-trust experiments on other datasets.

## Harness-only features you might want to revisit

Kept on this branch but not ported to engine because they're
regression-scope:

- `--position-csv` / `--residuals-csv` / `--state-csv` CSV dumps
  with Q2 sigma columns.  Essential for the σ-trust analysis —
  if someone repeats Q2 on a different station, re-run via these.
- Per-constellation `--profile` syntax (`gps:l2,gal:l5`).
  Hypothesis that GPS-L1/L2 vs GPS-L1/L5 matters was falsified
  on ABMF, but could matter on other datasets with different
  SSR/clock conventions.
- `--systems` filter for isolating per-constellation bias.
- Phase-observation integration (the pre-`6f95186` harness was
  silently PR-only).

## Ceiling + state of the art

Best measured harness result after Phase 1 + 2 obs-model:

- ABMF 2020/001 GAL+BDS + solid-tide + ANTEX + σ_phi=0.30:
  **0.170 m 3D** (vs 0.287 m without obs-model).  PRIDE ceiling
  on same file: 3.27 mm.  ~50× gap remaining is null-mode filter
  structure, not observation model.
- ABMF GPS+GAL aggressive (σ_phi=1.0, Q_pos=1e-10, tide, antex):
  0.927 m — still above the 500 mm break-even target Main set
  for Phase 2.  Null-mode limits.

For context on why the gap is filter-structural see
`project_to_main_pride_ablation_20260423` and
`project_to_main_solid_tide_result_20260423`.  The ~100× ABMF
GPS+GAL amplification of cm obs-model residuals into meter-scale
filter drift is the open door.  Structural fix needs adaptive
OU or eigenvector-constrained null-mode damping — session-scale
to scope, probably multi-session to land.

## Files worth knowing about on this branch

- `scripts/regression/run_regression.py` — the runner with all
  flags and CSV outputs.
- `scripts/regression/antex.py` — ANTEX 1.4 parser + nominal
  yaw-steering body frame.  **Will be deleted in cleanup;**
  canonical copy is main's `scripts/antex.py`.
- `scripts/regression/solid_tide.py` — IERS 2010 Step 1 SET.
  **Will be deleted in cleanup;** canonical copy is main's
  `scripts/solid_tide.py`.
- `scripts/regression/test_sp3_sat_position.py` — 8 tests for
  the SP3 bounds check.  Depends on `f0e4d83` — if the
  cherry-pick ask is accepted, this file may move to
  `scripts/peppar_fix/test_sp3_bounds.py` to match main's test
  layout conventions.

## When this branch is ready to retire

After:
1. The SP3 cherry-pick lands on main
2. This branch gets rebased + module duplicates deleted
3. Any harness-only features still worth having get a PR to
   main (most likely: none — the harness and its state-csv /
   ANTEX / solid-tide support stays regression-scope)

The branch may then be merged cleanly into main (the diff will
show mostly `scripts/regression/` additions — runner, tests,
state-csv columns, et al), or kept as a long-lived regression
branch.  Main's call.

Meanwhile: nothing on this branch blocks anything.  The
production-worthy work has already flowed through.

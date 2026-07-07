# Bootstrap-time antenna-move detection — reject stale position history

**Status**: design (2026-07-06). Motivated by the ptBoat Chicago→London
incident below. Not yet implemented; the immediate operational workaround
(clear the stale position → the engine falls back to NAV2) is already applied
to ptBoat.

## The problem

A clock host carries a saved position history — `state/positions/<uid>.ppp.toml`
(AntPosEst output), `state/positions/<uid>.survey.toml` (peppar-survey ARP), and
`state/receivers/<uid>.json → last_known_position`. It is shut down, **physically
moved to a different antenna/site**, and restarted. On restart the engine loads
the *stale* position and pins `FixedPosFilter` there. Every pseudorange residual
is then dominated by the multi-km position error → `n_used < 4` every epoch → the
DO bootstrap logs *"Filter did not converge in 10 epochs"* and never disciplines.
(If the move were smaller the servo would instead lock onto a subtly wrong clock
solution — quieter but still wrong.)

### Motivating incident (2026-07-06)

ptBoat came back online in **London** on the shared `ufoLondon1` antenna (NAV2:
`51.4946, -0.0613`, sub-metre identical to otcBob1 on the same splitter), but its
saved `.ppp.toml` was a stale **Chicago/Wheaton** ARP (`41.843, -88.104`). The
FixedPos was pinned **6142 km** from the antenna. Result: repeated bootstrap
failures for hours, misdiagnosed first as "marginal sky" and then chased through
receiver-band and SSR-coverage work — none of which was the cause. The receivers,
sky, and antenna were identical to otcBob1 (which disciplined cleanly); the *only*
difference was the stale pin.

Two existing mechanisms should have caught it and did not:

1. **The runtime NAV2 watchdog** (`position_state.WatchdogActor`) *did* see it —
   `[WATCHDOG_STEP_PENDING] source=nav2 displ=6142215.013m` — but it only
   "auto-moves" after `DEFAULT_AUTO_MOVE_THRESHOLD_S = 3600 s` of sustained bark.
   The bootstrap fails in ~30 s, long before the 1-hour graceful window. **Wrong
   timescale for a cold-start move.**
2. **The seed LS-validation gate** (`peppar_fix_engine.py:11357-11416`) already
   implements exactly the right check — get a live LS fix, and if
   `separation_m > 100` → *"File may be stale or corrupted. Falling back to
   bootstrap."* — **but it is SKIPPED for "trusted" seeds** (`skip_validation`,
   line 11346): `pos_sigma_m < _TRUSTED_POSITION_SIGMA_M` where the threshold is
   `10.0 m`. Every real position (survey σ≈0.1 m, ppp σ≈0.4 m) is < 10 m, so the
   move check effectively **never runs**. ptBoat's σ0.42 m Chicago `.ppp.toml`
   sailed straight through.

**Root cause: the code conflates two separate questions and gates both on σ-trust:
"is this seed *precise*?" (legitimately skippable for a trusted source) and "has
the antenna *moved*?" (must ALWAYS be answered).**

## Design

Split the two concerns. Keep the σ-trust gate for the *precision* re-validation,
but add a **gross-move check that runs unconditionally** — before pinning the
FixedPos — whenever there is a saved position to reject.

```
load seed (survey / ppp / receiver-state)         # unchanged provenance resolver
if seed exists:
    live = acquire_independent_fix()              # NAV2 opinion (primary), LS (fallback)
    if live is confident:
        displ = |seed - live|
        if displ > MOVE_THRESHOLD_M:              # default 100 m, CLI+profile
            -> MOVE DETECTED: reject + invalidate history, re-acquire (below)
        elif not trusted(seed):
            -> existing precision LS-validation path (separation<=100 m branch)
        else:
            -> trusted + not-moved: pin as today
    else:
        -> no confident live fix: fail SAFE — keep the seed (today's behavior),
           log that the move check could not run
```

### The signal — NAV2 primary, LS fallback

- **NAV2** (F9T secondary nav engine) is the natural signal: a single-epoch,
  history-independent fix, already wired (`nav2_store.get_opinion`,
  `wait_for_nav2_seed`, `_nav2_bootstrap_seed`). It is 1–5 m accurate with a
  documented ~1.5–4 m receiver-specific bias — negligible against a 100 m move
  threshold. Require a confident opinion (3D fix, `nSV ≥ N`, `hAcc ≤ bound`)
  before acting; on ptBoat this is exactly what let the engine fall back to the
  correct London position once the stale file was cleared.
- **LS fallback** for receivers without NAV2 (E810, non-u-blox): reuse the
  existing broadcast-only `ls_init` LS fix already used at 11378 (≈5–10 m, `n_sv
  ≥ 6`). Same threshold, same semantics.
- A NAV2-*seeded* position is exempt — it *is* the live fix, so it cannot be
  "stale relative to itself" (already handled by `seeded_from_nav2` at 11344).

### Threshold

Reuse the existing **100 m** LS threshold, promoted to a named constant + CLI arg
`--bootstrap-move-threshold-m` (default 100.0) + per-host `profile.get` override —
no new magic number. Rationale: well above NAV2 bias (1.5–4 m) and any *same-site*
antenna swap (< a few m, which the FixedPos tolerates anyway and which is NOT a
move we need to reject), and far below the smallest cross-site move
(hundreds of m to thousands of km). Same scale the runtime watchdog and the
untrusted LS path already trust.

### On move detected — "reject the position history"

1. **Reject the seed**: do not pin the FixedPos there (`known_ecef = None`).
2. **Invalidate the stale history** so it is not reused next boot — the helpers
   already exist: `position_state.invalidate_ppp_state(uid)` (archives/marks the
   `.ppp.toml`), `bump_mount_sn(uid)` (invalidates the antenna-instance serial),
   and clear `receiver-state.last_known_position`. A `.survey.toml` should be
   marked stale too (it is an ARP for the *old* antenna) — but only the engine's
   own ppp/receiver state is engine-writable; a stale *survey* triggers a LOUD
   operator alert to re-run `peppar-survey`, per the survey/engine write-ownership
   rule in `docs/position-state-and-monitoring.md`.
3. **Re-acquire a fresh position**:
   - Default / AntPosEst modes: fall to the existing `run_bootstrap` Phase-1 PPP
     (already the untrusted-path action at 11393), *or* seed directly from the
     confident NAV2 fix and let AntPosEst refine — NAV2 alone (σ≈1 m) is enough
     for the FixedPos to converge (proven on ptBoat 2026-07-06).
   - **`--no-antposest` (time-only)**: the pinned ARP came from a *survey* and is
     now invalid. The engine cannot self-survey. Behavior: seed from the coarse
     NAV2 fix so timekeeping can proceed *degraded*, and **log LOUDLY** that a
     re-survey is required for precision (`[MOVE_DETECTED] pinned survey ARP is
     N km from the live fix — re-run peppar-survey`). A `--strict-arp` flag can
     make this *refuse to start* instead, for deployments where a wrong ARP is
     worse than no clock.
4. **Log a first-class `[MOVE_DETECTED]` event**: old→new lat/lon, `displ_m`,
   `mount_sn` bump, and which history files were invalidated. This is an
   operator-visible state transition, not a warning buried in the stream.

### False-positive safety

- Act only on a **confident** live fix (3D, `nSV ≥ N`, `hAcc ≤ bound`); if NAV2 is
  stale/poor and LS can't get ≥6 SVs, **do not reject** — keep today's behavior
  and log that the check could not run. A missed move merely reproduces today's
  (already-broken) failure; a *false* rejection of a good seed forces a slower,
  coarser re-acquire — acceptable but not free, so require confidence.
- The 100 m threshold plus NAV2's few-metre bias gives a wide safety margin; no
  need for a sustain window at bootstrap (unlike the runtime watchdog) because a
  single confident fix 100 m+ off a pinned seed is already unambiguous, and the
  cost of waiting is a failed bootstrap.

### Relationship to the runtime watchdog

This is the **cold-start, immediate, hard** counterpart to
`WatchdogActor`'s **runtime, slow, graceful** move handling:

| | Bootstrap gate (new) | Runtime `WatchdogActor` (exists) |
|---|---|---|
| When | Once, before pinning FixedPos | Every cycle, mid-run |
| Speed | Immediate | Sustain 60 s → auto-move after 3600 s |
| Action | Hard: reject seed + re-acquire | Graceful: slew, or step after sustain |
| Threshold | 100 m (move) | 10 m (`DEFAULT_NAV2_THRESHOLD_M`, gross events) |

They **share** the NAV2 signal and the invalidation helpers
(`invalidate_ppp_state` / `bump_mount_sn`). The bootstrap gate handles
"woke up somewhere else"; the runtime watchdog handles "antenna moved while
running." Neither replaces the other.

## Implementation sketch

- **`position_state.py`**: add a pure, unit-testable
  `detect_move(seed_ecef, live_ecef, threshold_m) -> (moved: bool, displ_m)`
  (thin wrapper over the ECEF norm already at 11384) and a
  `MOVE_THRESHOLD_M` constant (default 100.0).
- **`peppar_fix_engine.py:11330-11416`**: restructure the `skip_validation`
  branch. Before the σ-trust skip, always attempt a confident live fix (NAV2
  opinion first, LS fallback) and run `detect_move`. On move → the existing
  `known_ecef=None; run_bootstrap(...)` path PLUS `invalidate_ppp_state(uid)` +
  `bump_mount_sn(uid)` + clear receiver-state position + `[MOVE_DETECTED]` log.
  On no-move → keep the σ-trust skip (precision) exactly as today.
- **CLI/profile**: `--bootstrap-move-threshold-m` (default 100.0, `profile.get`
  override), `--strict-arp` (default off → degrade; on → refuse on move).
- **Reuse**: `nav2_store.get_opinion`, `wait_for_nav2_seed`, `ls_init`,
  `run_bootstrap`, `invalidate_ppp_state`, `bump_mount_sn`,
  `save_position_to_receiver`.

## Tests

- Unit (`detect_move`): same-site swap (2 m) → no move; ptBoat case (6142 km) →
  move; NAV2 bias (4 m) → no move; exactly-at-threshold boundary.
- Integration (engine seed path, mockable): a σ0.4 m trusted `.ppp.toml` 6000 km
  from a confident NAV2 opinion → move detected, `.ppp.toml` invalidated,
  `mount_sn` bumped, re-acquire fired, `[MOVE_DETECTED]` logged. Same seed with a
  co-located NAV2 → no move, trusted-skip preserved (no needless re-bootstrap).
- No-live-fix: poor/absent NAV2 and <6-SV LS → check skipped, seed kept, warned.
- Regression: the existing trusted-skip and untrusted-LS-validation tests still
  pass (this splits, not removes, the σ-trust gate).

## Edge cases

- **First run, no history** → nothing to reject; unchanged cold-start.
- **No NAV2 + no LS-capable fix** (offline start, few SVs) → fail safe, keep seed,
  warn; the move is caught on the next epoch the live fix becomes confident (or by
  the runtime watchdog once obs flow).
- **Same-site antenna swap** (< threshold) → correctly *not* a move; FixedPos
  absorbs the few-metre offset and the ARP stays close enough.
- **Survey ARP moved** (`--no-antposest`) → detected the same way; engine can't
  self-survey, so degrade-to-NAV2 + loud re-survey alert (or `--strict-arp`
  refuse).

## Immediate workaround (already applied to ptBoat 2026-07-06)

`rm state/positions/<uid>.ppp.toml` + clear `receiver-state.last_known_position`
→ with no stale history and no local `antennas.json`, the engine fell back to
NAV2 (`51.4946, -0.0613`, London) and the bootstrap converged. This feature makes
that automatic and self-documenting.

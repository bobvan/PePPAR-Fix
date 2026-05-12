# Phase B Engine-Side Gate — NAV-SIG admission integration spec

**Companion to**: `docs/cat-reject-resilience-design.md` (rev 2)
**Status**: spec, 2026-05-12.  Pre-implementation; awaiting Bravo's
`Nav2SignalStore` parser to land on `f9tClockTelemetry-bravo`.
**Owner**: main.  Lands once PR #25 merges and Bravo's parser is
available on a shared branch.

## Goal

Integrate the receiver's per-signal usage verdict (UBX-NAV-SIG
`prUsed` bit) into the engine's obs admission decisions.  Compose
with our existing domain-specific exclusions.  Log every disagreement
between the receiver and us (both directions) so we can validate the
composition empirically.

## Architecture

```
                                   ┌───────────────────────┐
  UBX serial reader ──NAV-SIG──▶   │   Nav2SignalStore     │  (bravo)
                                   │   (sv, sigId) → state │
                                   └───────────┬───────────┘
                                               │ get(sv, sigId, max_age_s)
                                               ▼
  obs from RAWX  ──▶   admit_obs(obs, sv, sigId)  ──▶   filter.update(z)
                              │
                              ├── existing checks (clock_bad, below_mask, no_eph,
                              │   off_const, PB_GAP_DROP, AR readiness, ZTD-suspect)
                              ├── NEW: NAV-SIG prUsed check  ────────┐
                              │                                       │
                              └── on disagreement: snapshot quality state
                                  → [NAV-SIG_DISAGREE ...] log line  (charlie's Phase A.5)
```

Two filter paths use the gate:
- **PPPFilter** (`scripts/solve_ppp.py`, the position filter) — admits PR + CP per SV per frequency.
- **FixedPosFilter** (`scripts/solve_ppp.py`, the time filter) — admits PR + TD-CP per SV per frequency after position pin.

Single helper function used by both, with per-filter policy flag (see below).

## Interface contract with `Nav2SignalStore`

The store is Bravo's deliverable.  Engine consumes via a stable API:

```python
class SigStatus:
    """One UBX-NAV-SIG row, decoded.

    Fields populated from the message; bits decoded into bools.
    """
    sv:          str        # e.g. "G07", "E19"
    sigid:       int        # UBX sigId
    band_name:   str        # "L1CA", "L5Q", "E1C", "E5aQ", "B1I", "B2aI", ...
    pr_used:     bool       # flags bit 0
    cr_used:     bool       # flags bit 1
    do_used:     bool       # flags bit 2
    pr_smoothed: bool       # flags bit 3
    health:      int        # 0=unk, 1=healthy, 2=unhealthy (bits 8-9)
    quality_ind: int        # tracking quality 0-7
    cno:         float      # dBHz
    pr_res_m:    float      # NAV-SIG.prRes in meters (signed)
    iono_model:  int        # iono correction model used
    recv_mono:   float      # CLOCK_MONOTONIC when message was read

class Nav2SignalStore:
    def get(self, sv: str, sigid: int, max_age_s: float = 2.0) -> SigStatus | None:
        """Return the latest SigStatus for this (sv, sigid) IF its
        recv_mono is within max_age_s of now.  None if stale or absent.
        """

    def all_signals_at(self, max_age_s: float = 2.0) -> list[SigStatus]:
        """Snapshot of every signal status fresh within max_age_s.
        Used by the disagreement logger to detect transitions.
        """
```

**Contract from main side**:
- Store is thread-safe (engine main thread reads it; serial-reader thread writes it).
- Reads are non-blocking and cheap (dict lookup + age check).
- Returns `None` rather than stale data when age > threshold.

**Contract from bravo side**:
- Parser populates EVERY signal seen in the NAV-SIG message, even ones we don't currently use.  (We need to see the receiver's full picture, not just what RAWX gave us.)
- band_name is decoded via the same `_GPS_GAL_SIG_NAMES` / `F9T_BDS_SIG_NAMES` / `F10_BDS_SIG_NAMES` tables already in `peppar_fix/receiver.py`.

## Composition rules

For each obs admission decision, compute:

```python
nav_sig = nav_sig_store.get(obs.sv, obs.sigid, max_age_s=2.0)
receiver_excludes = (nav_sig is not None and not nav_sig.pr_used)
receiver_unhealthy = (nav_sig is not None and nav_sig.health == 2)
our_quality_admits = (all existing checks pass)

# Per-filter policy:
if filter == PPPFilter (position):
    admit = our_quality_admits and not receiver_excludes and not receiver_unhealthy
elif filter == FixedPosFilter (time):
    # Bob's directive: we may USE SVs receiver doesn't (for time-only
    # solving where our needs differ).  Default same as PPP but
    # configurable via --time-filter-follow-receiver.
    if args.time_filter_follow_receiver:
        admit = our_quality_admits and not receiver_excludes and not receiver_unhealthy
    else:
        admit = our_quality_admits

# Always log disagreements both directions (independent of filter)
if our_quality_admits != (not receiver_excludes):
    log_nav_sig_disagree(obs.sv, obs.sigid, nav_sig, our_quality_state)
```

**Composition stated plainly**: `use_sv = receiver_allows AND we_allow`
for position; for time we may run `use_sv = we_allow` (less restrictive)
when Bob's flag is off.

## Disagreement logging (Phase A.5)

Charlie's Phase A.5 logger snapshots are emitted whenever:
- We admit an obs the receiver excluded (`our_quality_admits=True` ∧ `receiver_excludes=True`)
- We reject an obs the receiver admitted (`our_quality_admits=False` ∧ `receiver_excludes=False`)

Log format (single line, structured for downstream tools):

```
[NAV-SIG_DISAGREE ep=12345 sv=E19 sig=E5aQ
                  receiver=excluded our=admit
                  receiver_prRes=+412.7m receiver_cno=39.0 receiver_health=1
                  our_filter=PPPFilter
                  our_pr_resid=+3.2m our_gf=0.3cm our_mw=-2.71c
                  our_lock_ms=64500 our_elev=29 our_az=185
                  our_last_slip_reason=mw_jump conf=LOW
                  decision=we_use]
```

Reverse-direction:
```
[NAV-SIG_DISAGREE ep=12346 sv=G07 sig=L1CA
                  receiver=included our=reject
                  receiver_prRes=+0.3m receiver_cno=44.0 receiver_health=1
                  our_filter=PPPFilter
                  our_reject_reason=PB_GAP_DROP
                  decision=we_drop]
```

These lines become the empirical dataset for tuning composition policy
later.

## NAV-SIG staleness handling

If `nav_sig_store.get()` returns `None` (no fresh status for this signal):

- **Position filter**: treat as `receiver_excludes = False`.  Fall back to
  our own admission logic alone.  Rationale: don't introduce a NAV-SIG
  dependency that breaks operation when the message lags or is missing.
- **Always log**: when ≥80% of admission decisions in an epoch fall
  through with `nav_sig is None`, emit `[NAV-SIG_STALE epoch=N pct=…]`
  warning so operators see we've degraded to NAV-SIG-less mode.

If the warning sustains for >60 s, the engine emits an info-level
"NAV-SIG missing or stale — operating without receiver verdict.  Check
that UBX-NAV-SIG is enabled on the receiver."  This is degraded but
not fatal; nothing about the system requires NAV-SIG.

## CLI flags

```
--nav-sig-gate           default: on
    Enable NAV-SIG-based exclusion in PPPFilter.

--time-filter-follow-receiver   default: off
    Apply NAV-SIG-based exclusion to FixedPosFilter as well.
    When off (default), time filter uses only our own quality checks.

--nav-sig-max-age-s      default: 2.0
    SigStatus older than this is treated as None (stale).

--nav-sig-disagree-log   default: on
    Emit [NAV-SIG_DISAGREE] lines on every disagreement.
    Off for noise reduction in production runs that have stable
    composition policy.
```

## Implementation footprint

| Component | Location | LOC |
|-----------|----------|-----|
| Admission gate helper | new `scripts/peppar_fix/nav_sig_gate.py` | 60 |
| PPPFilter integration | `scripts/solve_ppp.py` per-obs loop | 30 |
| FixedPosFilter integration | `scripts/solve_ppp.py` per-obs loop | 20 |
| Disagreement log emitter | `scripts/peppar_fix/nav_sig_disagree_logger.py` | 80 |
| CLI flags | `scripts/peppar_fix_engine.py` arg parser | 15 |
| Unit tests (helper + emitter) | `scripts/peppar_fix/test_nav_sig_gate.py` | 120 |
| Integration tests (replay harness extension) | `tools/analysis/replay_cat_reject.py` | 30 |

Total: ~205 LOC + 150 LOC tests = ~3-4h focused work.

## Tests

**Unit tests** for `nav_sig_gate.compose(...)`:
- `receiver_excludes=True`, our_admits=False → drop, no disagreement log
- `receiver_excludes=True`, our_admits=True → drop in position filter, log disagree
- `receiver_excludes=False`, our_admits=True → admit, no disagreement
- `receiver_excludes=False`, our_admits=False → drop, log disagree (we know something receiver doesn't)
- `nav_sig is None` (stale) → fall through to our_admits alone, NO disagreement log (we didn't disagree, just didn't ask)
- `health=unhealthy` overrides → drop regardless of prUsed
- `--time-filter-follow-receiver` policy switching

**Integration tests** (extend `tools/analysis/replay_cat_reject.py`):
- Mock Nav2SignalStore with synthetic NAV-SIG transitions
- Replay through engine on captured day0511 data
- Assert that 4/11 DRIFTING_OUTLIER cascades would have been
  prevented by NAV-SIG gate firing
- Assert that 2/11 UNKNOWN cascades would NOT have been prevented
  (no clean NAV-SIG signal for those)

## Migration plan

Three phases, each independently shippable:

**Phase B1 — gate code with `--nav-sig-gate=off` default**.  Lands the
gate code, integration points, helper, tests.  Disabled by default.
No behavior change in production.  Engine has the machinery; we don't
flip the switch yet.

**Phase B2 — enable disagreement logger with gate still off**.  Set
`--nav-sig-disagree-log=on`, capture overnight data on F9T + F10T
hosts.  Empirically validate: which disagreements are NAV-SIG=excluded
but our checks would admit?  Vice versa?  How often does each happen?

**Phase B3 — flip `--nav-sig-gate=on` for PPPFilter only**.  Time
filter stays unrestricted (Bob's directive).  Validate overnight on
MadHat (F10T host, 11-cascade reference).  Target: cascade #6 (the
21 ms chip slip) would never reach cat-reject because NAV-SIG would
have flagged the offending SV(s) before the bad obs entered the EKF.

**Phase B4 — `--time-filter-follow-receiver` opt-in**.  Add the flag,
let Bob/lab evaluate whether following the receiver on time-side
helps or hurts.

## Risks and how to mitigate

1. **NAV-SIG lag**: receiver state may take 1-2 epochs to flip
   `prUsed=0` after the signal becomes bad.  In that window, we
   admit bad obs.  Mitigation: existing cat-reject mechanism is the
   downstream safety net for the lag window.

2. **Receiver false-excludes a signal we still want**: NAV-SIG may
   exclude a signal that's borderline (low CN0, edge-of-mask) which
   we want for ZTD or AR.  Mitigation: log it as disagreement; we
   can run with `--nav-sig-gate=off` if the loss is measurable.

3. **Receiver false-admits**: NAV-SIG flags `prUsed=1` for an obs our
   own quality check would reject.  Mitigation: composition is AND,
   so we still reject.  Disagreement log captures the case for
   review.

4. **Receiver totally hosed (no NAV-SIG)**: graceful degradation to
   NAV-SIG-less mode (Phase A current behavior).  Engine still
   functional.

5. **Receiver and we both wrong**: caught by cat-reject + recoveryRetry
   downstream.  No worse than current behavior.

## Resolved questions

1. **Per-band exclusion semantics**: per-band, not per-SV.  *Resolved
   by Bob 2026-05-12: "One bad signal doesn't make for a bad SV."*
   If NAV-SIG says `L1CA prUsed=0` but `L5Q prUsed=1` for the same
   GPS SV, exclude L1CA only, keep L5Q.  Both observables stay in
   their respective per-frequency admission paths.

2. **NL ambiguity handling on receiver-de-admit**: **decision-by-data.
   Defer to NAV-SIG_DISAGREE empirics.**  *Resolved by Bob: the
   disagreement logger should give us evidence — when the receiver
   de-admits and re-admits, do we see (a) the ambiguity holds and
   resumes cleanly (keep-warm wins), or (b) we eat bad data on
   re-admit because the cycle count was corrupted (flush wins)?*
   Phase B2 captures the data; resolve before Phase B3 deploy.

3. **Log volume rate-limit**: per-(sv, sigid), first 10 disagreements
   per hour, summary thereafter.  *Resolved by Bob.*

## Open question (deferred)

4. **Interaction with PB_GAP_DROP**: PB_GAP_DROP is a SSR-coverage
   gap, not a receiver-quality issue.  Should disagreements with
   reason `PB_GAP_DROP` be suppressed from the log (since they're
   not really "we know better")?  Bob: "Not sure."  Default for
   implementation: keep them logged but in a SEPARATE counter
   (`disagree_pb_gap_drop` vs `disagree_quality`) so the dominant
   coverage-gap pattern doesn't drown out the receiver-quality
   signal.  Revisit after Phase B2 data.

## Validation criteria (for merge)

- All unit tests pass (./bin/test).
- Replay harness shows ≥80% of day0511 cascades have NAV-SIG-explained
  flag transitions on the offending SV(s) at the right time.
- Phase B2 overnight on F10T host produces >100 disagreement events
  with a coherent narrative (we can tell WHY we differed from the
  receiver in every case).
- Phase B3 overnight on MadHat (F10T) reduces exit-5 events from
  ~10/night to ≤2/night.
- No TDEV regression on PiFace (the F9T baseline) — the gate must
  not exclude good obs that we currently use.

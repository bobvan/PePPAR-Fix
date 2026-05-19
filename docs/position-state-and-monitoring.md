# Position state and monitoring

How a PePPAR-Fix engine acquires, persists, monitors, and refines its
antenna position estimate.  Covers cold start, steady state, watchdog
behavior, confidence disclosure, the optional peppar-survey companion
process, and NTRIP integration in both client and caster directions.

This supersedes `docs/arp-lifecycle-state-schema.md` for the *state
file layout* and *watchdog behavior* — that doc's design predates
several design discussions captured here and should be considered
historical.  The schema doc's history-file design (`history.jsonl`
+ `running_mean`) remains current and lives separately.

Companion: `docs/multi-receiver-design.md` for the
two-or-more-receivers-per-host design space.  Everything below is
receiver-keyed and respects multi-receiver as a future possibility.

## Goals

A peppar-fix engine launched with no CLI position arguments should
make reasonable decisions about its antenna position based on what
state it finds on disk.  The two-receiver case is not precluded:
each receiver is independent, with its own state files keyed by
`<receiver_uid>`.

Specifically:

- **Default invocation `peppar-fix`** reads state files and picks
  the most-confident known position, falling back to NAV2 when no
  state exists.  CLI overrides are unusual rather than the norm.
- **No position file format is mandatory.**  Engine works with
  none, one, two, or three of (`.ppp.toml`, `.survey.toml`, NAV2).
- **The engine never blocks waiting for survey perfection.**  PPS
  comes up immediately from NAV2-class estimates; confidence is
  *disclosed*, not hidden.  Downstream consumers (ptp4l BMC,
  operator dashboards) decide what to do with low confidence.
- **Engine restarts and crashes lose no position state.**  All
  position state lives in files; the engine reads them at startup.
- **The peppar-survey companion is optional.**  Installations with
  hand-surveyed ARPs (or no interest in survey refinement) skip it
  entirely.

## State files (receiver-keyed)

Four files per receiver UID, plus one shared lab antenna database.
All receiver-keyed files use the receiver's `SEC-UNIQID` (the F9T-
side hardware identifier, decimal-encoded as the filename).

```
state/receivers/<uid>.json           # engine-written
state/positions/<uid>.ppp.toml       # engine-written
state/positions/<uid>.survey.toml    # peppar-survey-written (optional)
data/rinex/<uid>-{date}.obs          # engine-written
timelab/antennas.json                # lab antenna DB (operator-curated)
```

**Disjoint writers.**  Each file has exactly one writer.  No locking,
no schema discipline, no shared-key conflict mode.

**The engine never writes `.survey.toml`.**  The word "survey" is
reserved for results of an external survey workflow — OPUS-Static,
PRIDE PPP-AR, CORS NTRIP RTK, etc. — and only peppar-survey-class
backends may produce that file.  The engine reads survey-class data
(both `.survey.toml` and `antennas.json`) but never writes it; its
own position-filter snapshots go to `.ppp.toml`.

### `state/receivers/<uid>.json` — receiver identity + mount_sn

Engine-written.  Holds hardware identity, runtime telemetry, and
**the current `mount_sn`**.  mount_sn is incremented by the engine on
step-class watchdog events (= antenna probably moved) or by operator
action via CLI.

Position files (`*.ppp.toml`, `*.survey.toml`) embed the mount_sn
they were written under.  At startup the engine compares stored
`mount_sn` against the receiver's current `mount_sn`; mismatch → file
is stale (previous antenna mount), ignore.

### `state/positions/<uid>.ppp.toml` — PPP filter's converged estimate

Engine-written.  The engine's own PPP filter converges over hours of
runtime; the engine periodically snapshots the converged position
plus its sigma to this file.  Survives engine restarts.

Useful for warm-start when no survey has run.

```toml
mount_sn = 3
ecef_m = [157469.3814, -4756189.0729, 4232768.5274]
sigma_m = 0.087
updated = "2026-05-18T03:42:11Z"
n_epochs = 14437
source = "peppar_fix_engine PPP-AR"
```

### `state/positions/<uid>.survey.toml` — peppar-survey's PRIDE solution

Written by the optional `peppar-survey` process (see below).  Holds
the most recent PRIDE / OPUS / external-tool solution that survey
chose to publish.  Typically tighter than `.ppp.toml` because it uses
post-processing-grade analysis and multi-day fusion.

Absence is fine — engine handles "no survey ever ran" gracefully.

```toml
mount_sn = 3
ecef_m = [157469.3812, -4756189.0731, 4232768.5276]
sigma_m = 0.008
updated = "2026-05-18T00:00:00Z"
source = "PRIDE PPP-AR 6-day mean"
n_solutions = 6
first_solution = "2026-05-13"
last_solution = "2026-05-18"
```

### `data/rinex/<uid>-{date}.obs` — observations for survey

Engine-written.  Standard RINEX 3.x.  peppar-survey (if run) reads
these and produces `.survey.toml`.  Configurable rotation cadence
(daily UTC-midnight by default).

## Cold start: how the engine seeds position

Decision tree on engine startup, in order:

1. **CLI `--known-pos LAT,LON,ALT` set?**  Use it directly.  Pin
   immediately with operator-supplied sigma (default 0.10 m if not
   given).  Overrides all files.

2. **Read receiver UID + current `mount_sn`** from
   `state/receivers/<uid>.json`.  If file absent (first run on this
   receiver), `mount_sn = 0`.

3. **Load up to three candidates:**
   - `state/positions/<uid>.ppp.toml` (engine-written PPP snapshot)
   - `state/positions/<uid>.survey.toml` (peppar-survey-written)
   - `timelab/antennas.json[arp_label]` (lab antenna DB; in-memory
     read only, never produces a `.survey.toml` on disk)

   Position files must have their embedded `mount_sn` match the
   receiver's current `mount_sn`; mismatch → ignore (stale).
   - `--ignore-ppp` skips `.ppp.toml` (e.g., when PPP filter
     diverged badly and you don't trust the snapshot).
   - `--ignore-survey` skips **both** `.survey.toml` and the
     `antennas.json` read — both are survey-class data and a single
     flag toggles them together.
   - `--ignore-arp-state` is shorthand for both ignore flags.

4. **Pick most-confident.**  Smallest σ wins.  Tie-break (within ~2×)
   to `.survey.toml` because PPP-filter σ can be optimistic — random
   walk on position state under-estimates slow-drift uncertainty,
   while a PRIDE solution is a snapshot with rigorous error
   propagation.

5. **If no state file has a position** (first cold start on this
   antenna; ignore flags consumed everything), fall back to NAV2
   first-fix.  σ_arp seeded at NAV2's reported hAcc (typically
   1–2 m).

6. **Compare picked σ to `pin_threshold_m`** (default 0.10 m,
   tunable in code):
   - σ < threshold → engine enters **pinned mode** (FixedPosFilter).
     AntPosEst still runs in parallel as watchdog.
   - σ ≥ threshold → engine enters **survey mode** (AntPosEst is the
     primary position estimator).

7. **NTRIP-caster bootstrap** (see below) can shortcut the cold-start
   ambiguity-resolution path by consuming a peer engine's broadcast
   ephemeris stream — independent of position seeding, just speeds up
   convergence.

The cold-start window (before AntPosEst converges to sub-cm) is a
known vulnerability — the engine produces PPS the whole time from
NAV2-class estimates, and the time-side confidence is *disclosed* via
ptp4l clockClass (see below).  Typical convergence to <3 ns is well
under an hour with PPP-AR; downstream consumers honoring clockClass
will treat the engine as a degraded source until it tightens.

## Steady-state operation: pinned vs survey mode

**Pinned mode** (σ_arp < pin_threshold): FixedPosFilter holds the
ARP at the picked position.  AntPosEst still runs in parallel as a
watchdog — its job in this mode is *change detection*, not position
estimation.

**Survey mode** (σ_arp ≥ pin_threshold): AntPosEst is the primary
position estimator.  FixedPosFilter is off.  RINEX writing continues
(important — peppar-survey needs it).  Convergence brings σ down;
when σ crosses pin_threshold for the first time, engine *can* flip
to pinned mode mid-run (transition via `.ppp.toml` write +
in-process state change, no restart needed).

## Watchdogs

Two watchdogs run independently in pinned mode.  Either can bark.

### NAV2 watchdog

- **Always on** when pinned.  No activation gate.
- **Source:** NAV2 PVT fix (already in every epoch).
- **Threshold:** `nav2_watchdog_threshold_m` (default ~0.5 m,
  tunable).  Coarse — designed to catch gross movements (antenna
  fell off the mast, mount knocked over, cable kicked).
- **Sustain:** `nav2_sustain_epochs` (default 60).
- **Why useful:** cheap, immediate.  Catches the "operator dropped
  the antenna" failure mode in seconds.

### AntPosEst watchdog

- **Conditionally active.**  Gated on AntPosEst's own σ dropping
  below `antpos_activation_sigma_m` (default 0.05 m, tunable).
  Before AntPosEst converges, the per-epoch σ is meters — any
  threshold would be insensitive, so the watchdog is disarmed.
  NAV2 watchdog covers the cold-start gap.
- **Source:** AntPosEst output position.  Running mean over last
  N epochs (`antpos_window_epochs`, default 60) for additional
  sensitivity — σ on the mean shrinks as √N, so the watchdog
  becomes useful well before per-epoch σ would suggest.
- **Threshold:** `antpos_watchdog_threshold_m` (sub-cm to cm range,
  tunable).  This is the watchdog that catches the sub-ns-relevant
  movements (antenna re-settling, slow creep, subsidence).
- **Sustain:** `antpos_sustain_epochs` (default 60).

### What happens when a watchdog barks

The engine evaluates the *displacement magnitude* (3D ECEF distance
between current best estimate and the pinned ARP):

- **Slew (< slew_step_threshold_m, default 1 m)**: update the
  FixedPosFilter's pinned position in place.  Next epoch uses the
  new value.  Residuals briefly attribute to clock; absorb.  No
  mount_sn bump.  No restart.  Log emits
  `[WATCHDOG_SLEW from=... to=... displ=...]`.

- **Step (≥ slew_step_threshold_m)**: treat as a probable antenna
  move.  Engine atomically:
  1. Increments mount_sn in `state/receivers/<uid>.json`.
  2. Truncates / invalidates `.ppp.toml` (a fresh one will be
     written when the new-mount filter reconverges).
  3. Logs `[WATCHDOG_STEP_AUTO_MOVE old_mount=N new_mount=N+1
     displ=...]`.
  4. Initiates clean shutdown.  Wrapper/systemd respawns; the new
     engine instance starts in survey mode (since `.ppp.toml` was
     invalidated and `.survey.toml` has stale mount_sn).

The 1 m / 3 ns slew-step boundary is the starting recommendation
based on the c ≈ 30 cm/ns relationship — 1 m position step is
roughly 3.3 ns clock-equivalent.  Both values are tunable in code.

**`--pin-position` bypasses both slew and step.**  When the operator
explicitly pins (e.g., a surveyed `--known-pos` they want NOT to be
auto-moved), the WatchdogActor is skipped entirely.  The CONFIDENCE
log lines still surface NAV2/AntPosEst displacement for diagnostic
visibility, but no automated action is taken — operator intent wins
over watchdog action.  This is the right behaviour for surveyed
truth pins where any disagreement reflects measurement bias, not a
real mount move.

**Auto-move is on by default** (when `--pin-position` is not set)
once the AntPosEst watchdog sustains barking past the step
threshold.  An installation that prefers operator-confirmed mount
moves can set `auto_move_threshold_s = 0` to disable.  Slew is
always automatic
(it's just a refinement, not a move declaration).

## Confidence disclosure

The engine never hides its confidence.  GPS time has two
independently-recovered components and the confidence reporting
respects that split:

- **Phase confidence** — how well we know *absolute* GPS time.
  Dominated by antenna-position uncertainty: a position error
  couples into clock-phase error via geometry (c ≈ 30 cm/ns, so 1
  m of position σ contributes ≈ 3.3 ns of phase σ in the worst
  case).  Recovered by the position filter chain (NAV2 seed →
  AntPosEst convergence → survey refinement).
- **Frequency confidence** — how well we know the *rate* of GPS
  time.  Dominated by servo tracking residual against the PPS
  measurement.  Recovered by the time filter (FixedPosFilter
  clock states).

The two are aggregated by RSS into a single total σ that drives
ptp4l clockClass.  Three disclosure surfaces follow.

### Periodic log lines

Every M epochs (default 60 = once per minute at 1 Hz), three
sibling log lines:

```
[CONFIDENCE_PHASE] sigma=0.034ns pos_sigma=0.0010m
  source=antpos_mean mount_sn=3 mode=pinned
[CONFIDENCE_FREQ]  sigma=0.840ns dt_rx_sigma=0.840ns
  scheduler=settled
[CONFIDENCE_TOTAL] sigma=0.841ns class_target=6 class_current=6
```

Searchable, joinable to other log streams, no separate "replay
watchdog state" tooling needed.

The `source=` field on `[CONFIDENCE_PHASE]` reports which σ_position
source dominated.  Priority order (best to worst):

1. `antpos_mean` — AntPosEst running-mean σ from the watchdog
   (sub-cm once armed)
2. `antpos_epoch` — AntPosEst per-epoch σ (when watchdog is
   unarmed but the thread is publishing)
3. `seed` — σ from the chosen state file at startup
   (`.survey.toml` or `.ppp.toml`)
4. `nav2_hAcc` — NAV2's reported horizontal accuracy
5. `unknown` — none of the above available (cold-start gap)

### ptp4l clockClass

The aggregate `total_sigma` from `[CONFIDENCE_TOTAL]` drives the
GM's clockClass via PMC management messages (existing layered
supervision in `docs/ptp4l-supervision.md`).  Low confidence → high
clockClass → BMC elects an alternate GM if one is available.

**Promotion thresholds with hysteresis** (separate rising/falling
to avoid clockClass flapping at band boundaries):

| Transition | σ_total threshold | Class result |
|---|---|---|
| Promote 248 → 52 | rises through < **800 ns** | 52 (initialized) |
| Promote 52 → 6   | rises through < **20 ns** | 6 (locked) |
| Demote 6 → 52    | falls below > **30 ns** | 52 (initialized) |
| Demote 52 → 248  | falls below > **1200 ns** | 248 (freerun) |

The 5 ns dead band around the 6↔52 boundary (20–30 ns) and the
400 ns dead band around the 52↔248 boundary (800–1200 ns) are
intentionally wide enough to absorb minute-scale σ_total wobble
without flapping the BMC.

The accuracy fields (`ACCURACY_25NS` for class 6, `ACCURACY_1US`
for class 52) align with these bands: class 6 promises ≤ 25 ns,
which we honour by promoting only when σ_total < 20 ns.

**Event-driven transitions** (unchanged by this design):

- Any → 7 (holdover): observation idle timeout.
- Any → 248: PHC divergence (exit code 5), watchdog alarm step
  action, engine crash, wrapper exit.

This is the proper place for downstream PTP-network consumers to
see degraded confidence — they don't have to parse logs or watch
state files.  BMC handles it.

### Optional PPS squelch

**Off by default.**  The engine keeps producing PPS even at high
clockClass — consumers with their own holdover (or that honor
clockClass) prefer "keep going degraded" over "silent."

Installations that prefer "silent over wrong" can opt in via
`pps_squelch_on_bark = true` in per-host config (or
`--pps-squelch-on-bark` CLI).  When enabled, the PPS-OUT path is
muted when either watchdog enters `barking` state.

Squelch capability is per-DO: ClockMatrix supports output-enable
directly, AD5693R DAC-driven OCXOs need a hardware mute path that
isn't always present.  When the DO can't squelch, the option is a
no-op and the engine logs a warning at startup.

## peppar-survey companion (optional)

Independent process that runs an **external authoritative
observation source** (OPUS, PRIDE, a quick NTRIP CORS-RTK check,
RTKLIB, etc.) and writes `state/positions/<uid>.survey.toml`.
Triggered by systemd timer, `@daily` cron, or any external
orchestrator.

**Naming convention** (load-bearing): "survey" is reserved for
external authoritative observations.  The engine's own AntPosEst
output is a **PPP solution** — that's what `.ppp.toml` holds, and
the engine writes it directly.  peppar-survey cannot promote a
PPP solution to a survey; that conflation was the root cause of
the TimeHat mount_sn=1 incident 2026-05-18 and the corresponding
`--from-ppp` backend has been removed.

Implementation is intentionally underspecified — could be a
Makefile, a shell script, or a Python wrapper around PRIDE / OPUS /
RTKLIB.  The contract is just:

- **Input:** an external authoritative source — captured RINEX
  submitted to OPUS or processed by PRIDE, a quick CORS-RTK fix
  against a nearby reference, etc.
- **Output:** atomic temp+rename write of
  `state/positions/<uid>.survey.toml` with current mount_sn tag,
  fresh ECEF, sigma, and provenance metadata.

Installations that skip peppar-survey entirely simply have no
`.survey.toml` file.  Engine handles that case — it falls back to
`.ppp.toml` or NAV2.

The engine **watches `.survey.toml` mtime** during runtime.  When the
file appears or updates with a matching mount_sn, the engine
evaluates the new position vs. its current pin:

- Within slew threshold → update pin in place (gentle slew).
- Beyond step threshold → treat as a probable mount move (handled
  by the watchdog step path, including mount_sn bump + restart).

This means peppar-survey can be added to an existing installation
without coordinated downtime — engine notices the first
`.survey.toml` and gently slews to the new position on the next mtime
event.

### Shared-antenna cross-check

When two receivers share an antenna (PiPuss-class), peppar-survey
runs over each receiver's RINEX independently and writes two
separate `.survey.toml` files (per the receiver-keyed convention).
A separate cross-check step (`peppar-survey --crosscheck` or a
separate small daemon) compares them: convergence indicates both
receivers are healthy; divergence beyond joint-σ flags a
per-receiver fault.

## NTRIP integration

Two directions, both relevant to the position lifecycle.

### NTRIP client (existing — engine consumes)

The engine already pulls from external NTRIP casters (CNES SSR,
broadcast ephemeris mounts, etc.) for orbit/clock/bias corrections
and broadcast ephemeris.  No change here — documented in
`docs/correction-sources.md` and elsewhere.

Relevant to position seeding: NTRIP delivery of broadcast ephemeris
+ SSR corrections is what makes AntPosEst's cold-start convergence
fast.  Without NTRIP, AntPosEst is single-frequency-coarse for many
minutes; with NTRIP, sub-cm convergence in under an hour.

### NTRIP caster (planned — engine serves peers)

Per `docs/caster-ephemeris.md` and `docs/peer-bootstrap-sketch.md`:
the engine can act as a local NTRIP caster, serving its own captured
RXM-SFRBX (encoded to RTCM 1019/1042/1046) and optionally its own
observations to peer engines on the local network.

This is the **peer-bootstrap** path for the position lifecycle: a
newly-launched engine on the same local network can discover an
existing engine via mDNS (`docs/ntrip-mdns-discovery.md`) and
shortcut its cold-start ambiguity-resolution by consuming a peer's
broadcast ephemeris stream.  This does *not* replace position
seeding from `.ppp.toml` / `.survey.toml` / NAV2 — those still
choose the engine's starting ARP.  Caster bootstrap just gets the
PPP filter converging faster.

For position lifecycle purposes the relevant fact is: the engine's
RINEX-writing path and the NTRIP-caster path are independent.
Disabling one does not disable the other.  An installation that
wants peer-bootstrap but not RINEX archival can configure
accordingly.

## CLI surface

Minimal — most operators run `peppar-fix` with no args.

| Flag | Purpose |
|---|---|
| `--known-pos LAT,LON,ALT[,SIGMA]` | Override all state files; pin directly. |
| `--ignore-ppp` | Ignore `.ppp.toml` when picking starting position. |
| `--ignore-survey` | Ignore `.survey.toml` when picking starting position. |
| `--ignore-arp-state` | Both `--ignore-ppp` and `--ignore-survey`. |
| `--pps-squelch-on-bark` | Mute PPS-OUT when any watchdog enters `barking` state.  Off by default. |
| `--auto-move-threshold-s SECS` | Override per-host `auto_move_threshold_s`.  Set to 0 to disable auto-move. |

Other CLI flags (engine debugging, log paths, etc.) are
documented separately.

## Thresholds (defaults in code)

All thresholds are defined in code with sensible defaults.  Per-host
TOML override is supported but not required — operators only set
overrides when a host's behavior diverges from default.

| Constant | Default | Purpose |
|---|---|---|
| `pin_threshold_m` | 0.10 | σ below which we operate pinned |
| `nav2_watchdog_threshold_m` | 0.5 | NAV2-vs-pin displacement that fires NAV2 dog |
| `nav2_sustain_epochs` | 60 | Consecutive over-threshold epochs to bark |
| `antpos_activation_sigma_m` | 0.05 | AntPosEst σ must drop below this to arm its watchdog |
| `antpos_window_epochs` | 60 | Running-mean window for AntPosEst watchdog |
| `antpos_watchdog_threshold_m` | 0.03 | AntPosEst-vs-pin displacement that fires AntPos dog |
| `antpos_sustain_epochs` | 60 | Consecutive over-threshold epochs to bark |
| `slew_step_threshold_m` | 1.0 | Bark displacement below = slew, above = step |
| `auto_move_threshold_s` | 3600 | Sustained step-class bark before auto mount_sn bump |
| `confidence_log_period_epochs` | 60 | How often to emit `[CONFIDENCE ...]` log line |
| `ppp_state_write_period_s` | 600 | How often engine snapshots PPP filter to `.ppp.toml` |

## What this design preserves for future work

- **Multi-receiver:** all state is receiver-keyed; no host-level
  position state.  See `docs/multi-receiver-design.md`.
- **Operator-only surveyed installations:** `.survey.toml` may be
  hand-written once and never refreshed; engine treats it as a
  first-class case.
- **Peer-bootstrap via NTRIP caster:** independent of position
  state files; bootstrap speeds convergence, doesn't replace
  position seeding.
- **PPS-out squelch on DOs that support it:** opt-in, no-op on
  DOs that don't, no surprises.

## Open questions

1. **Where does the engine snapshot its PPP filter to `.ppp.toml`
   from?**  Probably from AntPosEst's converged state.  Need to
   pick a single source of truth.
2. **How does the engine detect that a new `.survey.toml` write
   completed atomically?**  Probably inotify-or-poll-on-mtime; the
   peppar-survey writer always temp+renames.
3. **Per-host configuration of threshold overrides** — TOML key
   layout TBD.  Probably `[position]` section in per-host toml.
4. **`peppar-arp` CLI for operator mount moves** — out of scope
   here; covered by future work.

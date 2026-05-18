# Multi-receiver design — the space, not a decision

PePPAR-Fix has run with one receiver per host for its entire production
life.  PiPuss has two F9Ts physically wired up but isn't currently
running a dual-engine setup.  No host today *requires* multi-receiver
support.

This document captures the design space for two-or-more receivers per
host so today's architectural choices don't accidentally close doors.
**Nothing here is a decision** — the goal is to enumerate the
tradeoffs and pin a "do not preclude" list that today's implementation
work respects.

## Use cases worth keeping open

1. **Failure resilience** (production-timing).  One receiver's
   firmware glitches, SV admission collapses, USB enumeration shifts
   under it — the other receiver keeps producing observations.  The
   DO keeps being disciplined.

2. **Cross-check** (data integrity).  Two co-antenna receivers should
   produce identical clock estimates modulo per-receiver biases.
   Divergence beyond the joint-σ is a per-receiver fault diagnostic
   that no single-receiver self-test can match.

3. **Coverage diversity.**  Different hardware tracks different
   signal sets (F9T-20B sees L5 ≠ F9T-10).  Running both broadens SV
   admission and improves redundancy on partial-constellation outages.

4. **Survey hot-spare.**  One receiver in steady-state pinned
   operation; the other in continuous re-survey, antenna swaps, signal
   experiments.  Independent operational profiles per physical
   receiver.

5. **Antenna-shared cross-check.**  When two receivers share an
   antenna (PiPuss-class), their independent PRIDE-survey solutions
   should converge to the same ECEF.  Divergence indicates a
   per-receiver problem (signal tracking, firmware) — extremely
   powerful because the data shares almost every other systematic.

## Architecture options

### A. Process per receiver

Each receiver gets its own peppar-fix process.  Each writes its own
`state/*/<receiver_uid>.*` files (already receiver-keyed everywhere
today).

| Pros | Cons |
|---|---|
| Failure isolation — one engine crash doesn't touch the other | DO arbitration: only one DO per host, N engines want it — needs a servo-owner mechanism |
| State-file keying already matches | Cross-check needs IPC (or shared state file) |
| Per-receiver config natural (one toml per receiver) | More processes, more systemd units, more log streams |
| Each engine independent: own SSR mount, systems list, RINEX file | NTRIP connection per engine (wasteful for shared SSR streams) |

### B. Filter parallelism in one process

A single peppar-fix process runs N position/time filter pairs and one
shared servo.

| Pros | Cons |
|---|---|
| Single DO arbitration trivially solved (engine owns the actuator) | Shared blast radius — one filter's crash brings down both |
| Cross-check is in-process — direct memory access | Multiplexing N receivers into one engine main loop is a significant refactor (loop assumes one observation stream) |
| One NTRIP connection per host (efficient) | Per-receiver config overlay becomes complex |
| One log stream, one set of resource locks | Hardware-fault recovery harder (engine restart kills both filter chains) |

### C. Hybrid — receiver-engines + servo process

Each receiver runs an observation-and-position-estimate-only engine,
writing its clock-offset estimate to a shared IPC channel (file or
socket).  A separate `peppar-servo` process reads from N engines and
drives the DO.

| Pros | Cons |
|---|---|
| Failure isolation for receiver-engines | Most processes of all three options |
| Single servo owner — clean DO arbitration | IPC channel needs design (freshness, schema, atomicity) |
| Servo can do best-input selection or sensor fusion | Servo failure is now its own thing to monitor |
| Receiver engines can come and go in isolation | Three+ processes per host (receivers + servo + survey) |

## "Do not preclude" list

Today's code may already have assumptions that, if not corrected now,
make A/B/C harder to land later.  These are the things to keep mobile
even while we ship single-receiver features:

1. **State-file keying must stay receiver-keyed.**  Don't introduce
   host-keyed state files that bundle data from multiple receivers.
   Today's `state/receivers/<uid>.json`, `state/dos/<uid>.json`, etc.
   conventions are correct — don't regress.

2. **Engine bootstrap must not assume one F9T per host.**  F9T
   enumeration via `/dev/serial/by-path/...` in per-host toml works
   for multi-receiver hosts by giving each its own toml.  Don't
   hardcode "the receiver" as a singleton.

3. **TICC port binding must not be one-per-engine-only.**  Multi-
   receiver hosts may want one TICC per receiver (for independent
   chB GPS-PPS measurement) OR one shared TICC with multiple
   channels.  The engine code shouldn't hardwire one-TICC-per-engine.

4. **Actuator binding needs a "servo owner" flag.**  Engine today
   constructs its FrequencyActuator at startup and owns it for the
   run.  For A or C, only one engine on a host can own the actuator.
   Add a `peppar.servo_owner = true` (or equivalent) per-engine
   config flag even though today's hosts always have exactly one
   engine which is always the owner.  Defaults to `true` if absent —
   single-receiver hosts unaffected.

5. **NTRIP connection pooling is a future optimization.**  Today
   each engine opens its own NTRIP.  N engines on a host = N
   connections (waste of credentials and bandwidth on shared SSR
   streams).  Don't optimize now, but don't preclude an
   `ntrip_proxy` host service later.

6. **Log lines should be UID-discriminable.**  Today's logs don't
   tag receiver UID because there's only one.  In multi-receiver,
   either log-per-engine (filename includes UID) or shared log with
   `[recv=<uid>]` tag prefix.  Probably log-per-engine is cleaner
   given everything else is keyed by UID.

7. **Cross-check must work via state files, not in-process state.**
   The shared-antenna cross-check (use case 5) is a cron job
   comparing two receivers' `state/positions/<uid>.survey.toml`
   files.  If we preserve receiver-keyed file state, this comes
   for free.  Don't introduce in-process-only state that makes
   cross-check require IPC.

## Servo arbitration — the hard sub-problem

A and C both need to answer: when N engines want to discipline the
same DO, who wins?  Three sub-options:

- **Designated owner via config.**  One engine's toml says
  `servo_owner = true`; the others are observers only.  Operator
  decides at config time.  Simple, brittle (owner crashes →
  no servo).
- **Election among receiver-engines.**  Engines coordinate via a
  lock file or systemd-managed semaphore; whichever holds the lock
  is the servo owner.  Failover on owner crash.  More complex.
- **Separate `peppar-servo` process.**  Receiver engines are
  pure data-producers (clock estimate via IPC).  Servo is a
  separate small process that consumes from all and arbitrates.
  Cleanest separation, most processes.  Architecture C.

Use case 1 (failure resilience) really wants the election or
separate-process answer.  Use cases 2–5 are agnostic.  If we ever
prioritize use case 1 hard, separate `peppar-servo` is the answer.

## Recommendation (provisional)

Architecture **A (process-per-receiver) is the cheapest path forward**
from today's single-receiver code, *if* we accept the DO-arbitration
problem as a config-time decision (designated owner).  Failover is
manual.

If we ever take use case 1 seriously, architecture **C** is the
target — and the migration path from A → C is "extract the actuator
ownership into a separate process," which is exactly what the "do not
preclude" list #4 prepares for.

Architecture **B** locks in shared-blast-radius failure and requires
the most refactor for the least gain.  Probably skip.

## Open questions for whenever this becomes a real decision

1. Does the cross-check (use case 5) want to live inside
   peppar-survey as a post-step, or as its own small daemon?
2. For shared-antenna receivers, do PRIDE-survey solutions get
   averaged into a host-level "best ARP" or stay per-receiver
   (operator must compare manually)?  The schema says per-receiver
   with operator-copyable files.  Probably right.
3. NTRIP credential sharing: if two receivers want the same SSR
   stream, do we pool the connection (one read, two consumers) or
   accept two parallel reads?
4. When does the multi-receiver use case become real enough to
   justify the work?  PiPuss is the obvious test bed.  Production
   timing customers would be the forcing function.

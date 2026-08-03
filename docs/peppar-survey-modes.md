# peppar-survey: on-demand vs ongoing

*Requirements refinement from Bob, 2026-08-03 (surveyTwoModes, I-153044).
Design; not built.  Pairs with [arp-survey-strategy.md](arp-survey-strategy.md)
(which tier runs) and [time-only-architecture.md](time-only-architecture.md)
(who owns what).*

`peppar-survey` has two genuinely different users, and trying to serve both
from one invocation shape is what makes its CLI feel like a pile of flags.
Split it:

| | **On-demand** (no args) | **Ongoing** (`--daemon`) |
|---|---|---|
| Question asked | "Where am I, as well as you can tell me?" | "Keep the ARP true, forever" |
| Receiver | **Owns it** | Never touches it |
| Runs alongside | nothing | `peppar-fix` (or any raw-obs logger) |
| Output | stdout, progressively | filesystem contracts |
| Lifetime | until refinement is exhausted, then exits | until stopped |
| Typical caller | a person, a field script | systemd |

This is the shape the "gpsd of survey-grade positioning" framing implies. gpsd
has exactly this split too — `gpspipe` for a human, the daemon for everything
else — and the reason is the same: **owning the device and cooperating with a
device owner are incompatible jobs.**

## Mode 1 — on-demand (no CLI args)

```
$ peppar-survey
```

Opens the receiver, configures raw observations, and starts answering
immediately. It never waits for a good answer before giving an answer.

### Output: JSONL on stdout, human summary on stderr

One JSON object per refinement on stdout; a rendered line per refinement on
stderr. `peppar-survey | jq` works, `peppar-survey >/dev/null` gives a human a
progress display, and `2>/dev/null` gives a script clean data. Same reasoning
as gpsd's JSON: the moment the output is for both a person and a program,
choose the program and render for the person on the other stream.

```jsonc
{"refinement": 0, "t": "2026-08-06T14:02:11Z", "tier": "nav2",
 "ecef_m": [...], "llh": [...], "sigma_m": 3.9, "sigma_kind": "nominal",
 "epochs": 1, "note": "receiver single-point fix"}
{"refinement": 1, ..., "tier": "baseline", "base": "WMTW",
 "baseline_km": 20.6, "sigma_m": 0.11, "sigma_kind": "formal", "epochs": 1800}
{"refinement": 2, ..., "tier": "baseline", "sigma_m": 0.042,
 "sigma_kind": "empirical", "epochs": 5400, "delta_from_prev_m": 0.031}
{"final": true, "tier": "baseline", "sigma_m": 0.038, "epochs": 8100,
 "blocked_on": ["pride-rapid: products ~1 day out",
                "pride-final: products ~2 weeks out"]}
```

Every record carries `sigma_kind` because an honest σ is the whole
differentiator (see [peppar-survey-vs-oss]). `formal` is the engine's own
optimistic covariance; `empirical` is the scatter of independent sub-solutions.
Never print one and label it the other.

### Termination: "no further refinement is possible"

The literal reading — wait for PRIDE-final — means the command runs for two
weeks, which is not a command-line tool. The honest predicate is *no further
refinement is possible **from data that exists now***:

1. the current tier's estimate has stopped moving by more than its own σ over
   the last refinement interval, **and**
2. every faster tier has already been tried, **and**
3. the remaining tiers are blocked on time (more session, or later products) —
   not on something the operator could fix.

Then print the `final` record, name what's blocked and what each would buy,
and exit 0. If (3) is false — a tier is blocked on something *fixable* (no
`pdp3` installed, no `ntrip.conf`, `--max-km` too tight) — say so explicitly
and exit non-zero. "Converged" and "gave up" must never look alike.

`--until <tier>` / `--max-time` override the predicate for scripted use.

### Guard: two owners is the bear trap

On-demand mode owns the receiver; so does `peppar-fix`. Both at once produces
the class of failure the lab already knows well (stale processes, `fuser` on
`/dev/gnss-top`, half-configured receivers). On-demand mode must refuse to
start when the receiver is already claimed, and say the useful thing:

```
peppar-survey: /dev/gnss-top is held by peppar-fix (pid 3312).
  This host is already logging raw observations — use --daemon,
  which consumes them instead of taking the receiver.
```

## Mode 2 — ongoing (`--daemon`)

Owns nothing but its own solves. Two filesystem contracts in, one out.

### Input contract — the raw-obs spool

`peppar-fix` (or any logger) writes hourly raw-observation files to a spool
the survey daemon consumes:

```
data/rawobs/<uid>/<YYYY>/<DDD>/<uid>_<YYYY><DDD><HH>.ubx      # being written
data/rawobs/<uid>/<YYYY>/<DDD>/<uid>_<YYYY><DDD><HH>.ubx.done # complete
```

- **UBX RAWX + SFRBX bytes, each stamped `recv_mono`** — the Group A format
  already specified in [pos-replay-capture-manifest.md](pos-replay-capture-manifest.md).
  One capture then serves both `pos_replay` and the survey daemon; do not
  invent a second raw format.
- **The `.done` marker is the contract.** The consumer reads only marked
  files, so it can never parse a half-flushed hour, and either side can crash
  and restart without coordination.
- **Files, not a pipe or shared memory.** A pipe couples process lifetimes and
  loses data across a restart; the spool is replayable, inspectable, and
  survives both processes. The cost is disk, which is exactly why the engine —
  not the survey — decides when logging is on (below).

### Output contract — unchanged

`state/positions/<uid>.survey.toml`, atomic temp+rename, plus the accumulating
`history.jsonl` that `arp_history.py` already means over. `survey_state_watcher`
already picks refreshes up on the engine side and routes them to SLEW or STEP.
Nothing here changes.

### Coordination: publish facts, never commands

**`peppar-fix` should never start, stop, or restart `peppar-survey`.** It
should publish the facts that make the right action obvious, and let the
survey side (or systemd) act on them. Three reasons this is not merely
stylistic:

1. **peppar-survey is optional.** The engine must run identically on a host
   where it was never installed. A command channel has to degrade; a published
   fact is just unread.
2. **Either side must be restartable alone.** A command is a message with a
   delivery guarantee nobody wants to implement; a fact on disk is idempotent
   and re-readable after any crash.
3. **The decoupling is the architecture.** The engine owns time, the survey
   owns position. An engine that supervises the survey has quietly taken back
   ownership of position, and PRIDE-sized solves end up in the process tree of
   a 1 Hz servo.

#### The fact the engine already publishes: the lifecycle state

`survey_lifecycle.py` already derives ACQUIRING / REFINING / SURVEYED from
(σ_r, converged), and already carries the rule *"SURVEYED → hold the pin, stop
logging raw obs."* That is the answer to "when does peppar-fix want
peppar-survey running":

| Lifecycle | Raw-obs logging | Survey daemon |
|---|---|---|
| ACQUIRING | on | wanted, highest priority — the pin is a coarse seed |
| REFINING | on | wanted — better products may still land |
| SURVEYED | **off** | idle; nothing new to consume |

So the daemon doesn't need to be told to stop. When the engine reaches
SURVEYED it stops feeding the spool, and the daemon runs out of work on its
own. That also protects the small-disk hosts (otcBob1 at 7 GB): the expensive
resource is the spool, and the engine gates it.

Publish the state where the daemon can see it, e.g. alongside the position
state as `state/positions/<uid>.engine.toml`:

```toml
lifecycle       = "REFINING"     # ACQUIRING | REFINING | SURVEYED
obs_spool_dir   = "data/rawobs/PiPuss-F9T-20"
obs_logging     = true
arp_epoch_id    = 7
arp_epoch_since = 2026-08-06T13:58:04Z
```

#### The fact that's missing: `arp_epoch_id`

Bob's case — *"peppar-fix might have to restart peppar-survey if NAV2 reported
the antenna had moved"* — is a real gap today. `survey_state_watcher` handles a
**mount_sn** change (receiver swapped) by shutting the engine down for a clean
respawn. But `mount_sn` is *receiver* identity. **Moving the antenna on the
same receiver doesn't change it** — which is precisely the PiPuss-goes-to-
Newton case, and the general one the NAV2 watchdog exists to catch.

Add a monotonically-increasing **`arp_epoch_id`**: the engine bumps it whenever
it believes the antenna is no longer where the survey thinks it is —

- the NAV2 watchdog trips (sustained-N checks at the 10 m threshold; loose
  enough to clear the known 1.5–4 m NAV2 bias, per `docs/wrong-int-basin-2026-05-11.md`),
- an operator declares a move (`peppar-fix --arp-moved`, for the case you
  *know* you moved it and shouldn't have to wait for a watchdog),
- the receiver's `mount_sn` changes (subsumes today's signal).

Then, with no command channel at all:

- **The daemon** sees the bump, archives `history.jsonl` under the old epoch,
  discards the old running mean, and restarts its cascade at Tier A. Its next
  `.survey.toml` carries the new `arp_epoch_id`.
- **The engine's seed resolver gates on epoch *before* σ.** A `.survey.toml`
  or `.ppp.toml` stamped with an older epoch is not "a less certain estimate,"
  it is an estimate *of a different place* — it must be rejected outright, not
  weighed. This is the general form of two bugs we've already paid for: the
  overconfident `.ppp.toml` shadowing a survey (`project_seed_resolver_ppp_shadows_survey_20260704`)
  and ptBoat's stale Chicago `.ppp.toml` in London
  (`project_ptboat_back_online_smoke_20260706`). Both are stale-position
  problems that σ ranking cannot see, because the stale value's σ is *small*.
- **A hard restart is systemd's job**, not the engine's — `Restart=on-failure`
  plus an operator `systemctl restart` when the daemon is genuinely wedged.
  The epoch bump is a state change, and state changes don't need process
  lifecycles attached to them.

#### Why not the alternatives

- **Engine forks/supervises the survey** — couples an optional component into
  the timing process, puts PRIDE-sized solves in the servo's process tree, and
  makes "peppar-survey not installed" a code path in the engine.
- **Survey watches NAV2 itself and does its own move detection** — duplicates
  `_check_nav2` and its hard-won 10 m threshold in a second place, where it
  will drift out of agreement. One detector, published as a fact.
- **A socket / D-Bus command channel** — needs both sides up simultaneously,
  which is the one thing the filesystem contract buys us out of.

## Build order

1. **`arp_epoch_id`** — publish it, bump it on NAV2-trip / operator declare /
   mount_sn change, and make the seed resolver reject stale epochs. Smallest
   change, biggest correctness win, and it stands alone: valuable even if the
   daemon is never built.
2. **`state/positions/<uid>.engine.toml`** — publish lifecycle + spool path +
   `obs_logging`. Mostly plumbing over `survey_lifecycle.py`.
3. **The raw-obs spool + `.done` marker** — shared with `pos_replay`; build it
   once, for both.
4. **`--daemon`** — consume spool, honour `arp_epoch_id`, write `.survey.toml`.
5. **On-demand mode** — the no-args path, the JSONL/stderr split, the
   termination predicate, and the receiver-ownership guard. Last because it
   needs the tier cascade to be robust before it's worth pointing a human at.

## Related

- [arp-survey-strategy.md](arp-survey-strategy.md) — the tier cascade and
  `--auto`; this doc is about *when* and *how* it's invoked, not which tier wins.
- [time-only-architecture.md](time-only-architecture.md) — the engine half.
- [pos-replay-capture-manifest.md](pos-replay-capture-manifest.md) — the raw
  capture format the spool must reuse.
- [position-state-and-monitoring.md](position-state-and-monitoring.md) — the
  `.survey.toml` / `.ppp.toml` schema + atomic-write contract.

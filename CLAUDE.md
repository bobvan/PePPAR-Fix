# PePPAR Fix — Polecat Operating Manual

You are working on PePPAR Fix, a GNSS-disciplined precision clock system.
This file contains hard-won operational knowledge. Read it before writing
code or touching lab hardware.

## Cooperating with other agents — read the dayplan, write via the tool

PePPAR-Fix is co-developed by multiple agents (`main`, `bravo`,
`charlie`, …) plus Bob.  Coordination happens via a shared day plan
backed by an append-only ops log.  **Before doing anything else in a
fresh session, render the dayplan to get context:**

```sh
/home/bob/.claude/projects/-home-bob-git-PePPAR-Fix/dayplan/dayplan.py render | less
```

That path is a stable, worktree-neutral **symlink alias**; the tool
itself is versioned in the repo at `tools/dayplan.py` (source of
truth — edit + commit there).  The live-log dir is a fixed shared
path decoupled from the script's location, so every worktree's copy
appends to the one shared log.  See
[`docs/dayplan-cooperation.md`](docs/dayplan-cooperation.md) for why.

To file a new item, comment on an existing one, or ack a review,
**use the same CLI** (`propose` / `discuss` / `ack` / `amend` /
`status`).  **Do not edit `/tmp/dp-*.txt`** — that's render output,
not storage; edits there are invisible to other agents.

**Timezone: dayplan uses America/Chicago (CDT/CST), not UTC.**  As
of 2026-05-11 the dayplan tool's `today()` and `now_ts()` both
return local Chicago time — file boundaries cut at local midnight,
and timestamps in render output (`[charlie @ 14:24:08]`) are CDT
hours, not UTC.  Historical logs from 2026-05-10 and earlier keep
their original UTC-day boundaries; everything from 2026-05-11
onward is local.  If you're rendering the dayplan and don't see a
post you expect from another agent, **first refresh the render**
(`rm /tmp/dp-*.txt && dayplan.py render > /tmp/dp-now.txt`) rather
than concluding the post is missing — stale renders are the
single most common cause of "I haven't seen agent X respond"
confusion.

Full workflow + conventions: [`docs/dayplan-cooperation.md`](docs/dayplan-cooperation.md).

## Project goal

PePPAR Fix aims to faithfully transfer the **long-term stability of GPS
time** to the **Disciplined Oscillator** (DO) — the crystal at the
servo's actuator (e.g., the i226 TCXO, the Timebeat OTC's OCXO via
ClockMatrix) — while preserving the DO's **superior short-term
stability**.

Two oscillators bound the achievable result:

1. **The Disciplined Oscillator (DO)** — the servo can't make it more
   stable than its own free-running noise floor.  It can only steer
   the DO's frequency, not eliminate its phase noise.

2. **The GNSS receiver's oscillator (RX TCXO)** — every carrier-phase
   observation is tainted by the receiver's clock noise.  Servo inputs
   derived from the receiver (PPP dt_rx, PPS edges with qErr) inherit
   this floor.  We can't pull the DO below the RX TCXO's stability
   using GNSS-based inputs.

The **moonshot target**: at every tau, the DO output is as stable as
the *best* of (DO free-running noise floor, RX TCXO noise floor).  At
short tau the better oscillator's noise floor should shine through
unmolested by the discipline loop.  At long tau the DO should track
GPS time as faithfully as the GNSS receiver allows.  The discipline
loop should guide the transition ever so gently — no servo-induced
noise, no overshoot, no loop-bandwidth artifacts.

Beating PPS or PPS+qErr alone is not the goal — those are limited by
the F9T's measurement resolution, not by what either oscillator can
actually deliver.

### Cross-host PPS OUT agreement — the two-clock excursion bound

An equal-weight goal: **any pair of PePPAR Fix clocks must produce
PPS OUT edges that agree in phase, with a probability-bounded
maximum excursion.**  Measured by connecting the two PPS OUT
signals to two channels of a shared-reference TICC (chA and chB on
the same unit), so the differential measurement is unaffected by
that TICC's own reference noise.

**Numerical targets** (derived in `docs/two-site-sync-budget.md` —
read that doc before changing servo, DO, or actuator architecture):

| Configuration | Bound | Probability | Window |
|---|---|---|---|
| Shared antenna (same RF via splitter) | \|Δ\| ≤ 1 ns | 95% | any τ from 0.1 s to ~1000 s |
| Separate antennas, baseline ≤ 5 km | \|Δ\| ≤ 2 ns | 95% | any 1-hour window |

These are excursion bounds, not TDEV values.  TDEV is a variance
metric and does **not** bound the phase-difference trajectory between
two clocks — two clocks with TDEV(1s) = 250 ps can still drift ≥ 1 ns
in seconds when white-frequency noise dominates.  The budget that
backs out from these excursion targets is roughly:

- Per-clock σ_phase vs GPS ≤ ~350 ps RMS at all τ
- Equivalent TDEV(τ) ≤ ~150 ps at τ = 1 s, no positive-slope regions
  below τ ≈ 1000 s
- Carrier-phase TD-CP in the time filter (~5-10 ps per-epoch
  precision) is well within budget; the limiters are **DO
  free-running noise above the servo bandwidth** and **actuator
  resolution**, not measurement noise

**DO classification — what's part of the sync target vs best-effort**:

- **OCXO-class hosts** (Isotemp/CTI/Microchip OCXOs, or the
  Renesas 8A34002 ClockMatrix architecture used on ptBoat/otcBob1)
  are the sync targets.  ADEV(1s) ≤ 1e-11 is necessary to meet the
  per-clock budget at 1 Hz servo cadence.
- **TCXO-class hosts** (TimeHat i226 internal TCXO, similar) are
  **best-effort**.  At ADEV(1s) ≈ 1e-10, the integrated free-running
  noise above 1 Hz loop BW already eats the entire per-clock budget;
  no amount of servo tuning recovers it.  TCXO hosts continue to
  serve as F9T-PPS / measurement-chain validators but are not
  graded against the 1 ns / 2 ns excursion bounds.

**Two stages of validation**:

1. **Shared antenna first** — two clocks driven by the same RF via a
   splitter.  Eliminates atmospheric, multipath, and orbit/clock
   correction variability as sources of disagreement.  What remains
   is the discipline loop's own contribution plus any per-receiver
   biases or per-filter integer-resolution errors.  This is the
   cleanest test bed for servo design and for catching
   ambiguity-resolution bugs.  **Target: 1 ns excursion bound,
   95% probability.**

2. **Separate antennas next** — the real-world goal.  Two clocks at
   independent sites driving independent antennas must agree in
   phase/frequency after both converge, limited by per-site
   atmospheric and multipath differentials.  **Target: 2 ns
   excursion bound, 95% probability, baseline ≤ 5 km, clean sky at
   both sites.**

Cross-host PPS agreement is downstream of cross-host *position*
agreement: until two PePPAR Fix receivers converge to the same ARP,
their clock solutions will absorb the position disagreement and
their PPS edges can't agree either.  So before chasing sub-ns PPS
alignment, the position solutions must agree to sub-cm on a shared
antenna (and to within multipath/atmospheric limits on separate
antennas).

We are searching for:

- The best **servo input** — PPS, PPS+qErr, PPP carrier phase, PPP-AR
- The best **servo tuning** — loop bandwidth, gain scheduling, anti-windup
- The right **bootstrap initialization** — drift file, frequency seed
- The right **measurement chain** — TICC, EXTTS, on-chip TDC — to
  characterize each component without contaminating the result

Along the way we document and illustrate the obstacles: measurement
noise floors, quantization errors at every stage, oscillator drift
sources, two-oscillator differentials, loop dynamics.  Each gets a
story in `docs/visual-stories.md`.

## Before running on a lab host — read this first

**Read `docs/lab-operations.md`** for the deployment procedure,
pre-flight checklist, and known stumbling points.

### The repo is the source of truth

Every lab host has its `peppar-fix` checkout at `~/peppar-fix`, and
that directory **must** be a git working tree.  The first thing to do
when starting work on any lab host is:

```sh
ssh <host>
cd ~/peppar-fix
git status        # ← see what's locally modified before doing anything
git pull          # ← when you actually want fresh code from upstream
```

`git pull` is on-demand, not automatic.  Lab hosts can lag the upstream
indefinitely — that's a feature when one host is in a known-good state
you want to keep as a comparison baseline.  Pull only when you have a
reason to.  Always `git status` before pulling so you know whether
local edits are about to land in a merge.

**Local edits on a lab host are encouraged**, not avoided.  When
you're debugging something that only reproduces on a particular host,
edit files directly in `~/peppar-fix` on that host, test there, and
let `git status`/`git diff` track what you tried.  Don't pre-commit
every speculative fix — `git checkout -- <file>` discards what didn't
pan out, and `git add && git commit` keeps what did.  When a fix is
worth keeping, push *from the lab host*:

```sh
git push origin main           # ← lands on gt's bare local upstream
                               #    which auto-mirrors to GitHub
```

You can then pull the same fix to other lab hosts to confirm it didn't
break them, all before publishing anything beyond the bare upstream.

### **Hard rule: never `scp` or `rsync` code that's tracked in the repo.**

If you find yourself wanting to `scp scripts/foo.py to:somewhere` or
`rsync -a scripts/peppar_fix/ to:somewhere`, **stop**.  That's a sign
the workflow has broken — fix it via commit + push + pull on the
affected hosts.  scp around version control just creates drift between
hosts that git can't see.  Catastrophes from violating this rule
include the 2026-04-08 ocxo incident where multiple non-git copies
piled up at `~/peppar-fix`, `~/git/PePPAR-Fix`, and `~/PePPAR-Fix` and
nobody knew which was authoritative.

The only legitimate cross-host file copies are for things that **are
not in the repo**: `ntrip.conf` (credentials), `data/*.csv` (capture
artifacts pulled back to gt for archival), `data/position.json` and
`data/drift.json` (host-local runtime state).

### gt is the local upstream

The primary git upstream for lab hosts is the bare repo on the gt
home server at `bob@gt:git/PePPAR-Fix.git`, **not** GitHub directly.
Pushes to gt's bare are auto-mirrored to GitHub by a `post-receive`
hook (additive only — never deletes refs on GitHub even if they're
gone from the bare; see `hooks/post-receive` inside the bare for the
incident that produced this rule).

The reason for using gt as the local upstream rather than GitHub
directly:
- **Faster** — local network instead of github.com round trip.
- **Safer iteration** — push a fix from one lab host to gt, then pull
  on a *second* lab host and confirm it didn't break things there,
  *before* the change ever reaches GitHub.  Catches "fixed it on
  host A, broke host B" early.
- **Works without internet** — the lab is on a local network; gt is
  always reachable even when GitHub is not.

GitHub is still the public origin and remains the long-term home of
record.  It's just that lab hosts and gt's dev tree both push *through*
the gt bare upstream, not directly to it.

### After `gh pr merge` on GitHub — sync gt explicitly

The gt mirror hook is **forward-only**: it pushes gt → GitHub when gt
receives a push, never the other direction.  When a PR is merged via
`gh pr merge --squash` (or any GitHub-side merge), the resulting
commit lands on GitHub but **does not propagate to gt**.  Subsequent
dev-box commits then end up rooted on a parent that no longer exists
in GitHub's main history, and the hook will refuse to forward them
("post-receive: failed to forward refs/heads/main") — leaving gt with
local commits that never reach GitHub.

**After any GitHub-side merge, sync gt before pushing more code:**

```sh
git fetch github main
git checkout main
git merge --ff-only github/main   # picks up the squash merge
git push origin main              # gt now matches GitHub
```

If gt has already accumulated divergent local commits (because the
sync was skipped), the only clean fix is reset + force-push:

```sh
git fetch github main
git checkout main
# Confirm content is preserved (the squash usually subsumes pre-merge work)
git diff main github/main
git tag pre-force-push-recovery-$(git rev-parse --short main) main
git reset --hard github/main
git push --force origin main      # gt aligns with GitHub
```

Then on each lab host:

```sh
ssh <host> 'cd ~/peppar-fix && git fetch && git reset --hard origin/main'
```

(plain `git pull --ff-only` fails when the lab host's local main has
the now-orphaned commits; reset is required.)

Catastrophe from skipping this: 2026-05-20 morning — dev-box had a
standalone docs commit on gt, PR #42 + #46 merged on GitHub, lab
hosts pulled gt's old code, canary deploy hit "unrecognized argument:
--no-antposest", and the fix required force-pushing gt to align.

### Squash-merging a stacked PR's base auto-closes the dependent PR

When PR B's base is PR A's branch (typical stacked-PR setup), and PR
A is merged with `gh pr merge --squash --delete-branch`, GitHub
deletes PR A's branch and PR B **auto-closes** because its base ref
no longer exists.  `gh pr reopen` fails with "Could not open the
pull request" — there's no way to revive the original PR number.

Recovery — rebase the dependent branch onto main and open a new PR:

```sh
# Locally — drop the commits that PR A already squashed in
git checkout -b rebase-prB github/charlie/featureB
git rebase --onto main <old-tip-of-PR-A> rebase-prB
./bin/test                                    # confirm clean
git push --force-with-lease github HEAD:charlie/featureB
git checkout main
git branch -D rebase-prB

# Open a fresh PR with base=main
gh pr create --base main --head charlie/featureB \
    --title "..." --body "Reopen of #N (auto-closed when its base \
    branch was deleted on PR A merge).  Rebased onto main."
```

The original PR number is dead; the replacement gets a new number.
Cross-link them in the new PR body so the review history is
discoverable.  Caught 2026-05-20 when PR #50 (charlie/surveyCorsConsumer,
stacked on PR #48) auto-closed and reopened as PR #51.

### Common lab-host failures (in order of frequency)

1. **Missing Python deps**: set up the venv first:
   `cd ~/peppar-fix && python3 -m venv venv && venv/bin/pip install pyubx2 pyserial "pyproj>=3.6"`
   (add `smbus2` on I2C hosts). Never use `--break-system-packages`.
   `pyproj>=3.6` is required for `geo_frames` (NAD83↔ITRF datum conversion);
   it bundles PROJ ≥ 9.0 which has ITRF2020.
2. **Missing directories**: `mkdir -p ~/peppar-fix/data`
3. **Missing ntrip.conf**: `scp TimeHat:~/peppar-fix/ntrip.conf ~/peppar-fix/`
   (this is the *one* legitimate scp — credentials are not in the repo).
4. **Stale processes**: `sudo pkill -f peppar` before starting.
5. **TICC args need splitting**: `--engine-arg --ticc-port --engine-arg /dev/ticc1`
   (NOT `--engine-arg "--ticc-port /dev/ticc1"`).
6. **Timebeat must be stopped on OTC hosts**: `sudo systemctl stop timebeat`.

Always use the `peppar-fix` orchestration wrapper, not individual scripts.

## Lab Test Protocol

PePPAR Fix is implemented as component scripts that can be run directly from the CLI, but
users would be unlikely to run them individually. They are normally invoked by the orchestration
wrapper in scripts/peppar-fix. That's over 500 lines of code that should be tested whenever
possible. Always prefer testing using the wrapper as a user would, but it's ok to
run components individually for diagnosis or troubleshooting.

## Recommended operational mode: `--no-antposest` (time-only)

For time-mission deployments, **the recommended way to run the engine
is `--no-antposest`**.  See [`docs/time-only-architecture.md`](docs/time-only-architecture.md).
TL;DR: peppar-survey delivers the ARP offline; the engine skips
its own position filter entirely; FixedPosFilter runs at the pinned
ARP and drives PHC discipline; NAV2 watchdogs gross ARP moves at
10 m.  Empirically validated 2026-05-20 on PiFace canary — TICC chA
TDEV at τ = 1000 s matched the default-mode baseline within 1 ps
(0.73 ns vs 0.72 ns).  All four lab hosts switched to `--no-antposest`
2026-05-20.

The default engine behavior is preserved (Phase 1 PPPFilter
bootstrap + AntPosEst runtime refinement) for backward compatibility
and for moving-platform / real-time positioning use cases where the
position filter is the point.

## Optional component: peppar-survey

`peppar-survey` is the optional companion to `peppar-fix` that writes
authoritative ARP estimates to `state/positions/<uid>.survey.toml`
from external observation backends (currently `--pride` only).  Native
deps are tracked separately from `peppar-fix`'s `pyproject.toml`
because PRIDE-PPP-AR is Fortran, not pip-installable.

In the recommended `--no-antposest` mode, peppar-survey is
**load-bearing** for ARP acquisition — the engine refuses to start
without a seed (`--known-pos` OR `arp_label → antennas.json` OR a
`.survey.toml` / `.ppp.toml` state file).

Full install procedure: [`docs/peppar-survey-install.md`](docs/peppar-survey-install.md).
TL;DR: `bash scripts/install_peppar_survey.sh` on each lab host.

**Required step that's easy to miss: edit PRIDE's antex database.**
PRIDE-PPP-AR ships the IGS antex catalog, which does NOT include
receiver-specific NGS antenna calibrations like `SFESPK6618H NONE`
(the CHOKE1 + UFO1 antenna we use).  Without the matching block,
pdp3 silently falls back to a different (or zero) calibration and
the ARP biases by ~1 m vs OPUS-Static.  `scripts/install_peppar_survey.sh`
calls `scripts/inject_lab_antennas.sh` automatically at install time
to append the lab-relevant blocks from `support/antex/`.  When adding
a new antenna to the lab, append its NGS antex (e.g. from
`https://geodesy.noaa.gov/ANTCAL/`) to `support/antex/` AND update the
`LAB_ANTENNAS` list in `scripts/inject_lab_antennas.sh`, then re-run
the injector on every host that has PRIDE installed.  Idempotent —
already-injected blocks are skipped.

## Lab Hosts and Access

All lab hosts are Raspberry Pis or similar SBCs. SSH access is
passwordless for user `bob`.

| Host | Access | Role | GNSS | Notes |
|---|---|---|---|---|
| TimeHat | `ssh TimeHat` | Primary peppar-fix dev + PHC discipline | F9T-TOP on `/dev/gnss-top` (EVK-F9T-10-00, ZED-F9T, TIM 2.20 — older firmware, no native L5/B2a-I tracking) | Has i226 PHC, TICC #1, heatsink on TCXO |
| PiPuss | `ssh PiPuss.local` | Dual-F9T, caster/client testing | F9T-TOP `/dev/gnss-top`, F9T-BOT `/dev/gnss-bot` | Zero-baseline (both on Patch3 via GUS #2) |
| ~~Onocoy~~ | mothballed 2026-04-08 | F10T + PX1125T disconnected; TICC #2 moved to ocxo | – | Powered down. Never had a peppar-fix checkout. |
| otcBob1 | `ssh otcBob1` | Timebeat OTC SBC, OCXO, Renesas ClockMatrix | F9T on `/dev/ttyAMA0` at 460800 | Stop `timebeat` before accessing I2C or GNSS |
| ptBoat | `ssh ptBoat` | Timebeat OTC Mini PT, weatherproof, PoE | F9T on `/dev/ttyAMA0` at 115200 | Same Renesas ClockMatrix as otcBob1 |
| ~~ocxo~~ → ptpmon | `ssh ptpmon` | **Decommissioned 2026-06-14.** The E810-XXVDA4T x86 machine that was host `ocxo` was recommissioned as **ptpmon** (x86_64 Ubuntu utility host — NTRIP/offline captures, x86-only tools). `ocxo` and `ptpmon` are the **same physical machine** and never coexist; **do not expect to host `ocxo` again.** | – | See [[ptpmon-arch]] |
| bbb | `ssh bbb` (→ 10.168.13.14, PTP LAN) | BeagleBone, GPS L1 only | `/dev/gps0` at 9600 | Legacy NTP/PTP GM. **Reachable ONLY via the PTP LAN** — no trusted-LAN/Tailscale/mDNS interface, so PTP-LAN SSH is the sole option here (see resolution note). |

**Hostname resolution**: Try `<host>` first (DNS search domain VanValzah.Com).
If that fails, try `<host>.local` (mDNS). PiPuss only resolves via
`.local`.

**SSH interface preference**: always prefer a host's **trusted-LAN or
Tailscale** interface, and keep the PTP LAN (10.168.13.x) clean for
timing traffic.  **Exception**: when the PTP LAN is a host's *only*
reachable interface — as it is for `bbb` (10.168.13.14, no other NIC) —
use it for SSH.  Connectivity over the PTP LAN beats no connectivity;
the "keep PTP clean" rule yields to "reach the host at all" when there's
no alternative.

## Serial Port Gotchas

### TICC resets on serial open

TAPR TICCs use Arduino Mega 2560. **Opening the serial port toggles DTR,
which reboots the Arduino.** The TICC goes silent for ~10 seconds during
boot, then outputs its config header before starting measurements.

The Arduino resets on the **rising edge** of DTR (via a capacitor to
RESET). When a process closes the serial port, the `cdc_acm` driver
drops DTR. When the next process opens it, DTR rises — triggering a
reboot. `dsrdtr=False` in pyserial is **not sufficient** to prevent
this; it only controls pyserial's flow control, not the kernel driver.

The fix is to clear the `HUPCL` termios flag, which tells the kernel
to leave DTR asserted when the fd closes:

```python
import termios

# RIGHT — prevents DTR drop on close, so next open won't reboot:
ser = serial.Serial("/dev/ticc1", 115200, dsrdtr=False, rtscts=False)
attrs = termios.tcgetattr(ser.fd)
attrs[2] &= ~termios.HUPCL  # cflag
termios.tcsetattr(ser.fd, termios.TCSANOW, attrs)
```

All TICC access should go through `scripts/ticc.py` which handles
this automatically via the `_SharedTiccPort` helper.

If you WANT to reset a TICC intentionally:
```python
ser = serial.Serial("/dev/ticc1", 115200, dsrdtr=True)
ser.close()
# Wait 15 seconds for boot
```

### F9T EVKs have no USB serial number

All F9T EVKs report the same VID:PID (`1546:01a9`) with no serial number.
You cannot distinguish them by USB descriptor. On PiPuss (two F9Ts),
udev uses USB path matching which breaks if cables move. On single-F9T
hosts (TimeHat), VID:PID matching is fine.

Each F9T does have a unique `SEC-UNIQID` queryable via UBX protocol,
but this is not visible to udev.

### Stable device names

Devices with unique serial numbers get stable names everywhere via
`99-timelab.rules`:

| Device | Name | Basis |
|---|---|---|
| `/dev/ticc1` | TICC #1 | Arduino serial `95037323535351803130` |
| `/dev/ticc2` | TICC #2 | Arduino serial `44236313835351B02001` |
| `/dev/ticc3` | TICC #3 | Arduino serial `44236313835351B0A091` |
| `/dev/f10t` | NEO-F10T (ArduSimple) | FTDI serial `D30GD1PE` |

Devices without unique serials (PX1125T, F9T EVKs) do NOT get udev
symlinks. Your code must identify them at runtime.

## NTRIP Credentials and Configuration

NTRIP config file: `/home/bob/peppar-fix/ntrip.conf` on TimeHat.

Credentials are in `ntrip.conf` on each lab host (not committed to the repo).
See `ntrip.conf.example` for the format.  To deploy credentials to a new host:

```bash
scp TimeHat:~/peppar-fix/ntrip.conf .
```

Caster: `ntrip.data.gnss.ga.gov.au:443` (TLS).
SSR mount: `SSRA03IGS0` (IGS-RTS combined).  **`SSRA00BKG0` was retired
2026-07-22** — BKG's real-time SSR combination vanished from both the GA
caster and BKG's own `products.igs-ip.net` (a Wheaton host still pointed
at it gets a silent 404 → broadcast-only PPP with no SSR).  The IGS-RTS
combined stream `SSRA03IGS0` is the verified successor: same GA caster,
same `bobvan`/TLS creds, HTTP 200, decodes 96 orbit / 96 clock / 480
code-bias.  Also on `products.igs-ip.net` (`SSRA02IGS0` = IGS02 is the
alternate).
Broadcast ephemeris mount: `BCEP00BKG0` (same caster, pass via `--eph-mount`
— **still alive**, unaffected by the SSR retirement).

**SSR status**: Orbit + clock + code bias available. **Phase bias = 0**
(not provided by this stream). This means PPP-AR is not possible with
this SSR source alone.  (Unchanged by the BKG→IGS-combined switch — both
are float-only; AR phase biases still need a single-AC mount like CAS/WHU
per `ac-datum-mixing.md`.)

## Known Broken Things

### ~~BDS ISB 1500 ns~~ — closed 2026-04-19 (PR #3 + PR #5)

Default is `--systems gps,gal`.  BDS previously produced ISBs of
1500+ ns (should be <200 ns).  Two root causes identified and
fixed 2026-04-19:

1. **BDS signal-code map for RTCM 1260 was wrong** (sig_ids 2-11
   mis-labeled or missing).  F9T L5-fleet tracks BDS-3 MEO on
   B1I + B2a-I; sig_id=9 (B2a-I) was stored under the wrong RINEX
   key, so every BDS L5-band code-bias lookup MISSed.  **Fixed in
   PR #3, commit `150c495`.**
2. **BDS-3 modernized cpMes is L1-reference cycles.**  F9T firmware
   TIM 2.25 reports B2a-I carrier phase in L1-band cycles, not
   native.  GF/MW/IF downstream math assumed native cycles,
   producing ±60-190 m per-epoch GF blow-ups and 1800+ false-positive
   slips per minute.  **Fixed in PR #5, commit `12a76a3`**
   (multiply cpMes by λ_L1 / λ_native at ingest).

Validation: day0419c on MadHat + clkPoC3 reached BDS AR via LAMBDA+
rounding on C24/C35/C44 with sub-cm NL residuals matching GAL
quality.  Cross-host horizontal agreement < 1 cm.  See
`project_bds_ar_first_success_20260419.md`.

Still open (lower priority — residuals don't demand it):

- Dual-TGD handling for IF(B1I, B2a-I) on B3I-referenced broadcast
  clock.  Commit `63a4af1` decoded TGD2 (DF514) into `_BDS_MAP` but
  `_sat_clock` still applies TGD1 only.  SSR phase biases appear to
  absorb the per-SV TGD variance in practice.

Prior attribution to the 14 s BDT/GPST offset was wrong — that's
already handled correctly in `broadcast_eph.py:_bds_seconds_of_week`.
Full research + references: `docs/bds-ppp-integration.md`.

### ~~F10T on Onocoy doesn't respond to UBX~~ — Onocoy mothballed 2026-04-08

The NEO-F10T issue is parked along with the host.  When we revive F10T
work it'll be on a different host.

## peppar-fix Python Environment

| Host | Venv | Activation |
|---|---|---|
| dev box (gt) | `/home/bob/git/PePPAR-Fix/venv` | `source /home/bob/git/PePPAR-Fix/venv/bin/activate` |
| TimeHat | `/home/bob/peppar-fix/venv` | `source ../venv/bin/activate` |
| PiPuss | `/home/bob/pygpsclient` | `source ~/pygpsclient/bin/activate` |
| ~~Onocoy~~ | mothballed 2026-04-08 | – |
| otcBob1 | `/home/bob/peppar-fix/venv` | `source ../venv/bin/activate` |
| ptBoat | `/home/bob/peppar-fix/venv` | `source ../venv/bin/activate` |

**Provisioning a host — `scripts/provision_lab_host.sh`** (2026-06-25): the
idempotent setup.  `cd ~/peppar-fix && bash scripts/provision_lab_host.sh`
creates the venv and installs the **engine-only** stack
(`pip install -e '.[timebeat]'` → numpy/scipy/pyubx2/pyserial/pyrtcm/pyproj +
smbus2; **no** analysis libs — matplotlib/allantools/pandas live in the
`[analysis]` extra), creates data/ + state/ dirs, and runs a preflight
(imports, serial/PTP, ntrip.conf, **stray-copy guard**, **disk-pressure
warning**).  The Timebeat hosts have small disks (otcBob1 = 7 GB, 4.6 GB of it
`/usr`) — **do NO on-host analysis; pull captures to gt and crunch there**, and
keep no stray repo copies (found + removed 97 M + 28 M + 771 M of strays
provisioning OTC/Mini 2026-06-25).

Scripts live in `/home/bob/peppar-fix/scripts/` on every provisioned host (the
repo IS `~/peppar-fix`).

### Running unit tests — always from the dev-box venv

`pyserial` is a core dep declared in `pyproject.toml` and required by
the engine.  Running `pytest` against system python3 (which lacks
pyserial) produces ~12 spurious failures that are NOT real test
failures — just `ModuleNotFoundError` from `import serial` in
`scripts/ticc.py` propagating up through anything that imports
`peppar_fix_engine`.

**Always run tests via the dev-box venv** at
`/home/bob/git/PePPAR-Fix/venv`, OR through the `bin/test` wrapper
at the repo root which exec's the venv's pytest:

```bash
# From any worktree on the dev box:
./bin/test                                    # full suite
./bin/test scripts/peppar_fix/test_foo.py     # specific file
./bin/test -k some_pattern                    # pattern match
```

If `bin/test` errors on `venv not found`, the venv is missing on
this dev box — bootstrap it once with:

```bash
python3 -m venv /home/bob/git/PePPAR-Fix/venv
/home/bob/git/PePPAR-Fix/venv/bin/pip install -e '.[dev]'
```

`[dev]` resolves through `pyproject.toml`'s `optional-dependencies`
to pull in pytest + numpy + pyubx2 + pyserial + pyrtcm + matplotlib +
allantools + smbus2 + textual.  One install covers all test paths.

When reporting test results in PR reviews and dayplan posts, "all
tests pass" should be **literally** all tests pass under the venv —
no "12 pre-existing failures we ignore."  Real failures uncovered
by the venv get filed as their own dayplan items, not papered over.

## Lab Storage Warning

**Lab hosts use eMMC or SD cards.** These can fail without warning.
After any significant capture or test, pull results back to the GT
server (`/home/bob/gt/`) which has RAIDZ-3 + offsite backup. Large
datasets can stay on lab hosts; everything else should be pulled.

## Resource Allocation

Lab hardware is shared. Before using a host or device, check that no
other work is running on it:

```bash
# Check for running processes using a serial port
ssh <host> "fuser /dev/gnss-top 2>/dev/null"

# Check for running peppar-fix or analysis processes
ssh <host> "ps aux | grep -E 'peppar|servo|ticc|analyze' | grep -v grep"
```

Beads that need hardware carry `hw:` labels (e.g. `hw:TimeHat`,
`hw:F9T-TOP`). The Mayor checks these before assigning work. If your
bead has a hardware label, you are the exclusive user of that hardware
for the duration of your work.

## Key Technical Context

### Terminology — DO vs PHC, rx TCXO vs bare TCXO

**Read `docs/glossary.md`** for definitions of all domain terms.

Critical naming rules:

- **DO** (Disciplined Oscillator): the crystal being steered.  Use
  "DO" in any context that isn't PHC-specific.
- **PHC**: use only when referring to the Linux PTP Hardware Clock
  API (`adjfine`, `clock_settime`, `EXTTS`).  Not all DOs are PHCs.
- **rx TCXO**: the TCXO inside the GNSS receiver (F9T).  Never use
  bare "TCXO" — it's ambiguous with the DO's crystal on i226 hosts.
- **gnss_pps** / **do_pps**: the two PPS streams.  PPS error =
  gnss_pps − do_pps (positive = DO is late).

### Stream correlation via CLOCK_MONOTONIC — read this first

**Read `docs/stream-timescale-correlation.md` before modifying any
code that matches data from different streams** (qErr, TICC, EXTTS,
PPP, NTRIP).

Every data stream operates on its own timescale.  The TICC makes
this obvious (seconds since boot), but even PHC timestamps have no
guaranteed relationship to GPS or UTC unless we establish it by
measurement.  A PPS edge is just a voltage transition — it's only
correlated with a GNSS epoch because **we** correlate it.

The **only** shared timescale is `CLOCK_MONOTONIC`.  Whether it's
an EXTTS read through the PTP driver, a TICC timestamp read through
serial port X, or a qErr message read through serial port Y, we
have a timestamp on the read against `CLOCK_MONOTONIC`.  From that,
we must correlate everything.  There is no other reliable way given
queueing, CPU scheduling, and network delays.

**Critical rules for qErr correction of TICC measurements**:

- **Sign**: `corrected = ticc_diff_ns + qerr_ns` (plus, not minus)
- **Matching**: qerr must correspond to the **same PPS edge** the
  TICC measured.  "Same PPS edge" = their `CLOCK_MONOTONIC` read
  timestamps match the expected timing relationship.  Match using
  `ticc_measurement.recv_mono` (expected_offset ≈ 0.95s), NOT the
  EXTTS `pps_event.recv_mono`.  Off-by-one edge makes TDEV **worse**
  than raw PPS (3.3 ns vs 2.1 ns — confirmed 2026-04-11).
- **qVIR (qErr Variance Improvement Ratio) `Δvar(raw)/Δvar(raw+qerr)`
  must be > 1.5.  If ≤ 1.0, the correlation is broken — stop using
  qerr immediately.  Do not discover this after an overnight run.

This applies to **all** time-based correlation between streams with
independent timescales, not just qErr.

### Position finding

The PPP position finder (`peppar_find_position.py` or Phase 1 of
`peppar_fix_cmd.py`) converges in ~90 seconds at sigma 0.5m with
GPS+GAL and NTRIP broadcast ephemeris. The convergence requires:
- Per-system ephemeris warmup (all configured systems must have ≥8 SVs)
- Satellite health filtering (excludes unhealthy Galileo E14/E18)
- Satellite clock sanity check (|sat_clk| < 2ms)
- LS outlier rejection (>50m residuals excluded)

### Survey vs PPP terminology — load-bearing distinction

Three distinct sources of position knowledge.  Don't conflate them
in code, log lines, or operator-facing docs:

- **Survey** = external authoritative measurement from a
  surveying-grade pipeline.  Examples: OPUS-Static multi-day mean,
  PRIDE PPP-AR, a quick NTRIP CORS-RTK check against a nearby
  reference station, NGS-calibrated antenna setup.  Sub-cm typical,
  authoritative.  Lives in `timelab/antennas.json` (operational
  ARP database) and survey writeups under `timelab/surveys/`.  At
  runtime goes to `state/positions/<uid>.survey.toml` — **only
  ever written by `peppar-survey`-class backends, never by the
  engine** (design in `docs/position-state-and-monitoring.md`).
  The engine *reads* survey-class data (both .survey.toml and
  `antennas.json`) but never writes it.
- **PPP solution** = the engine's own AntPosEst output, written
  to `state/positions/<uid>.ppp.toml` by the engine itself.
  Convergence depends on AR mode: WL-only ~m-class; PPP-AR with
  resolved narrow-lane integers can reach cm-class.
- **NAV2** = the F9T/F10T's onboard secondary navigation engine.
  Single-epoch code-only fix, intrinsically 1–5 m accurate, often
  exhibits a multi-meter receiver-specific bias (see below).

`peppar-survey --from-ppp` (removed 2026-05-18) was the offender
that conflated PPP solution → survey.  Don't reintroduce that
pattern in any new tool.

### NAV2 bias — the floor on NAV2-based thresholds

NAV2 SPP solutions exhibit a persistent receiver-specific bias of
**~1.5–4 m** relative to the surveyed ARP.  Documented in
`docs/wrong-int-basin-2026-05-11.md` (4 m on MadHat post-CHOKE1)
and surfaced again 2026-05-18 by the new `[CONFIDENCE_PHASE]`
log lines.  CHOKE1 surveys (`timelab/surveys/2026-05-05-choke1-
cors-rtk.md`, `timelab/surveys/2026-05-07-choke1-opus-static.md`)
agree sub-cm across CORS-RTK / OPUS / tape methods, so the bias
is in NAV2, not in the survey.  NAV2 can also drift further than
the bias under multipath or sky-view changes.

**Operational implication**: any NAV2-based watchdog threshold
must be loose enough to absorb both the bias and the noise on top
of it.  Tight thresholds (e.g., 0.5 m) generate continuous false
positives and trigger spurious reset / step / re-bootstrap
cascades.  The pre-existing `_check_nav2` uses 10 m horizontal
sustained-N-checks for "antenna moved" detection — that's the
right order of magnitude.  Slice 7's `WatchdogActor.nav2_threshold_m`
was retuned to match 2026-05-18.

NAV2 is useful for catching **gross** physical events (antenna
fell off the mast, cable kicked) where the displacement dwarfs
the bias.  For sub-cm-scale antenna stability monitoring, use
AntPosEst's running-mean watchdog instead.

### Authoritative ARP — always `timelab/antennas.json`

Operational ARPs live in `timelab/antennas.json` (gitignored
antenna database, one entry per antenna).  Current entries: `ufo1`
(decommissioned 2026-05-05) and `choke1` (CHOKE1 antenna on the
lab roof, shared via GUS splitter across all F9T/F10T hosts).
Each ARP comes from an OPUS-Static multi-day mean, σ ≈ 12 mm.

If you need the surveyed ARP for `--known-pos`, `state/positions/
<uid>.survey.toml`, or any other purpose, **read it from
`timelab/antennas.json`** rather than copying coordinates into
scripts/wrappers/docs — the coordinates evolve as new survey runs
land and the JSON file is the truth.

**File access pattern**: the engine reads `antennas.json` directly
as an in-memory survey-class seed candidate (gated by per-host
`arp_label = "..."` in the host config TOML).  No `.survey.toml`
is written from this read — the "survey" word stays reserved for
files actually produced by a peppar-survey-class backend.

### Unified CLI

`peppar_fix_cmd.py` is the unified entry point:
- Phase 1: PPPFilter bootstrap (estimates position from scratch)
- Phase 2: FixedPosFilter (clock estimation with optional servo)
- `--servo /dev/ptp0 --pps-pin 1` enables PHC discipline
- `--position-file` skips Phase 1 if the file exists and is valid
- Position file is validated against live LS fix before Phase 2

### Free-running TCXO baseline

TimeHAT v5 TCXO with heatsink: TDEV(1s) = 1.17 ns as measured by
TICC (60 ps resolution, 2h capture, 0.2% reproducibility).
F9T PPS TDEV(1s) = 2.3 ns (2h baseline, varies 1.0-1.4 ns on 30 min
windows depending on sawtooth phase).

### EXTTS TDEV measurements are unreliable — use TICC

**Both i226 and E810 EXTTS have ~8 ns effective resolution.**  This
matches the F9T's 125 MHz clock period.  EXTTS TDEV measurements
underreport true timing noise because the quantization masks real
PPS jitter:

- **E810 EXTTS**: 77% identical adjacent timestamps.  Reports falsely
  low TDEV (0.34 ns for a signal that's actually 2.3 ns).  The sub-ns
  timestamp format does not reflect sub-ns measurement resolution for
  GPIO/SMA events.
- **i226 EXTTS**: adds ~2.9 ns RSS noise but at least tracks PPS
  movement (0% identical adjacent).  Still underreports at short tau.

**Never report TDEV from EXTTS alone.**  EXTTS-only TDEV makes results
look better than they are.  Always use TICC (60 ps resolution) for
TDEV characterization.  EXTTS data may be shown alongside TICC for
contrast — shade the area between TICC and EXTTS to reveal the
measurement error:
- Area between TICC and E810 EXTTS = "actual TDEV unreported by E810"
- Area between TICC and i226 EXTTS = "i226 measurement noise"

See `docs/ticc-baseline-2026-04-01.md` for the full analysis and
`docs/visual-stories.md` for plot specifications.

### TICC stability metric: use chA alone, not chA-chB

When characterizing servo output stability (TDEV/ADEV), use **TICC chA
alone** (the DO's PPS, detrended).  This measures the absolute phase
stability of the disciplined oscillator — what a downstream consumer
actually sees.

**Do not use chA-chB** (DO PPS minus GNSS PPS) for this purpose.
chA-chB measures how well the DO *tracks GPS* — i.e., the work the
servo did — not the quality of the output.  A perfect servo tracking a
noisy GPS reference would show low chA-chB but high chA: the output
inherited the reference noise.  Conversely, a servo that drifts slowly
from GPS might show large chA-chB while chA stays smooth.

Use chA-chB only when the question is "how faithfully does the DO
follow GPS?" — e.g., diagnosing servo gain or loop bandwidth.

## Design Documentation

The `docs/` directory contains design documents and research notes. Start
here before changing anything in the areas they cover.

| File | Summary |
|---|---|
| [dayplan-cooperation.md](docs/dayplan-cooperation.md) | **Read this first if you're new to a multi-agent session.** How to use the shared dayplan tool (`propose` / `discuss` / `ack` / `amend` / `status` / `render`).  Storage model, I-number convention, threading conventions, and the load-bearing rule "don't edit /tmp/dp-*.txt — that's render output, not storage." |
| [stream-timescale-correlation.md](docs/stream-timescale-correlation.md) | **Read this first.** How to correctly correlate events from independent timescales (GNSS, PPS, TICC, NTRIP). Covers why queue-order matching fails, the strict correlation gate design, confidence scoring, and fault injection testing. |
| [two-site-sync-budget.md](docs/two-site-sync-budget.md) | **Read before changing servo/DO/actuator architecture.** Math behind the moonshot's 1 ns shared-antenna / 2 ns separate-antenna excursion bounds.  Decomposes σ_Δ(τ) into measurement floor + servo residual + DO above-BW noise + actuator quantization.  Per-DO-class achievability analysis.  Survey of DC-OCXOs vs 18-bit-DAC-upgrade path; recommends OCXO + AD5781 at ~$80/host as best price-performance. |
| [gpsdo-noise-and-external-clock.md](docs/gpsdo-noise-and-external-clock.md) | **The mental model behind why peppar-fix exists.**  GPSDO two-oscillator noise model + crossover (DO flywheel vs GNSS noisy-absolute-reference, output = lower envelope); oscillator upgrade helps holdover always but normal-running only at τ < crossover; why even ideal GNSS is noisy short-τ (measurement noise + rx TCXO + corrections); carrier-phase-vs-PPS as peppar-fix's actual lever.  **Then the external-clock receiver architecture:** clocking the receiver *with the DO* (NetRS/NetR9/PolaRx external 10 MHz in) turns it into a direct GPS-minus-DO phase comparator, **deleting the rx TCXO floor** the moonshot is bounded by.  NetRS = L1/L2 GPS-only (cheap PoC); NetR9 / Septentrio PolaRx5TR keep our L5+Galileo+BeiDou bands.  Faster-than-1 Hz raw obs → higher loop-BW headroom. |
| [clock-state-modeling.md](docs/clock-state-modeling.md) | Where time-domain knowledge enters the position filter.  Maps the three oscillators (rx TCXO, DO, RO) to filter states, lays out four levers (stochastic rx TCXO model, TICC+qErr pseudo-measurement, full co-estimation, RO characterization), and the recommended ordering.  Attacks the null-mode clock axis identified in the 2026-04-23 PRIDE arc. |
| [full-data-flow.md](docs/full-data-flow.md) | Complete inventory of live data sources, their timescales, sink policies (freshest-only vs loss-free vs correlated-window), freshness requirements, and decimation effects. |
| [platform-support.md](docs/platform-support.md) | Per-platform status for TimeHat (i226) and ocxo (E810). Documents device paths, PHC behavior, GNSS transport differences, and bring-up checklists. |
| [gnssdo-plus-integration.md](docs/gnssdo-plus-integration.md) | **Read before touching SparkPNT SXT-D (GNSSDO+) hardware.** How to make its STP3593LF OCXO a PePPAR-Fix DO: two-chip (mosaic-T + ESP32) architecture, the ESP32-console-over-mosaic-T-IPS1 TCP backdoor (read-only in stock firmware — no command disables discipline or writes the control word), and the minimal opt-in firmware change that adds a host-driven `$W`-writes-control-word mode with a fail-safe watchdog + non-persistence.  Patch: `support/gnssdo-plus/0001-external-oscillator-control.patch` (git-am-able onto a fresh SparkFun clone).  CI-built + hardware-validated on MadHat 2026-07-06 (`$R`/`$T`/`$E`/`$W` + watchdog auto-revert all pass); validated `.bin` + stock backup at `~/gt/firmware/gnssdo-plus/`.  Build = Dockerized/CI arduino-cli, ESP32 core 3.0.7; front-panel USB flash. |
| [gnssdo-servo-loop-bandwidth.md](docs/gnssdo-servo-loop-bandwidth.md) | **The servo self-noise problem, on the single-oscillator GNSSDO+ bench.**  The free-running Rakon OCXO beats the disciplined output out to ~1000 s (TDEV 54 ps vs 187 ps @ 100 s) → the loop is *adding* mid-τ noise below the OCXO-vs-GPS crossover.  Root: LQR phase gain `L[2]=−0.05` → corner ~20 s, ~50× too high; push to ~−0.001 (corner ~1000 s) so the DO free-runs below and tracks GPS above.  Same knob fixes the tight-Q oscillation (under-damping = estimation-lag).  New `--lqr-phase-gain` knob + existing `--max-interval`/`coast_tdev`; A/B sweep plan (measure disciplined chA TDEV vs free-run floor).  Single oscillator = no rx-TCXO confound, so the TDEV gap IS the servo self-noise. |
| [gnssdo-plus-pps-alignment.md](docs/gnssdo-plus-pps-alignment.md) | GNSSDO+ hardware PPS-to-DO-edge alignment (D-flip-flop retimes the mosaic-T PPS to the next 10 MHz DO edge; on by default, overridable).  Two concerns: (1) it adds a constant PPS-OUT **bias** (0–100 ns; differs per box → matters for cross-host PPS agreement); (2) **100 ns boundary steps while disciplined** if the operating point sits within the flip-flop setup time of a DO edge (metastability).  Distinct from the deterministic free-run cycle-slips (those are de-glitched; these are a real defect).  Bench test + accept/reject criteria (scope raw mosaic PPS vs box PPS OUT: constant bias, off the boundary → leave on).  Plan agreed; scope pending lab access. |
| [time-and-platform-todo.md](docs/time-and-platform-todo.md) | Concrete work breakdown: remaining tasks for E810 GNSS, TimeHat PPS, correlation model, legacy cleanup, diagnostics. |
| [timebeat-otc-research.md](docs/timebeat-otc-research.md) | Early ClockMatrix research (some addresses wrong). See register-map doc for confirmed addresses. |
| [timebeat-otc-signal-routing.md](docs/timebeat-otc-signal-routing.md) | Signal flow sketch (DPLL mapping outdated — see register-map doc for confirmed configs). |
| [timebeat-otc-register-map.md](docs/timebeat-otc-register-map.md) | **Authoritative** 8A34002 register map: correct I2C addressing, DPLL/status/TDC registers, confirmed on both hosts. |
| [timebeat-integration-paths.md](docs/timebeat-integration-paths.md) | Integration plan: ptBoat (easy, PHC-only or write_freq) vs otcBob1 (complex, live TDC). Runtime MODE writes confirmed working. |
| [data-flow.md](docs/data-flow.md) | Original data flow sketch (superseded by full-data-flow.md for sink policy details). |
| [position-convergence.md](docs/position-convergence.md) | PPP position bootstrap convergence analysis and tuning. |
| [nic-survey.md](docs/nic-survey.md) | Survey of NICs with PTP hardware timestamping support. |
| [e810-cm5-research.md](docs/e810-cm5-research.md) | E810 on Raspberry Pi CM5: showstopper (ice driver x86-only). |
| [phc-bootstrap.md](docs/phc-bootstrap.md) | PHC bootstrap design: cold/warm start, optimal stopping, glide slope, characterization method, drift file, how the servo starts with bounded error. |
| [pps-ppp-error-source.md](docs/pps-ppp-error-source.md) | PPS+PPP servo error source: using carrier-phase dt_rx to replace TIM-TP qErr via 125 MHz tick model. Experiment results, calibration procedure, formula. |
| [correction-sources.md](docs/correction-sources.md) | How to get SSR corrections: registration, caster options, which streams for float PPP vs PPP-AR, why AR requires a single analysis center. |
| [ssr-mount-survey.md](docs/ssr-mount-survey.md) | F9T-focused survey of SSR mounts for PPP-AR.  Ranks WHU OSBC00WHU1, MADOCA-PPP, Galileo HAS, CAS phase-2, CNES by F9T-L5Q suitability.  Corrects our earlier "L5I bias can't be used on L5Q" premise. |
| [known-good-obs-validation.md](docs/known-good-obs-validation.md) | **Diagnostic: subtract the clock out of the error budget.**  Run our PPP on observations from a national-timescale-clocked reference station (PTBB=UTC(PTB), BRUX=UTC(ORB); both BIPM, on igs-ip.net/euref-ip.net as RTCM 3.3 MSM7) — any clock-bias wander then can't be the clock, so it fingerprints our SSR + filter (also deletes DO/rx-TCXO/antenna/multipath).  Credited to Ole Petter Rønningen.  Creds (`bob` on igs-ip/euref-ip) + pyrtcm MSM decode + engine `--obs-ntrip-mount` seam already in hand; live-proven on PiFace 2026-07-22 (PTBB dt_rx flat, σ→0.11 ns).  Diagnostic only — not a production input, can't be a differential clock, doesn't touch the DO/actuator limiter.  Includes the NTRIP stream-name decoder ring (country field ⇒ observations; agency field ⇒ corrections). |
| [ac-datum-mixing.md](docs/ac-datum-mixing.md) | **Read before touching SSR mount routing.** Why phase biases from different ACs are calibrations in different datums (not noise around a common truth), why mixing is OK for float-PPP and forbidden for AR, and the static gap-fill design that lets the secondary mount fill primary-AC coverage gaps without poisoning shared-signal AR. |
| [galileo-has-research.md](docs/galileo-has-research.md) | Galileo HAS: free PPP-AR corrections via E6-B signal. |
| [peer-bootstrap-sketch.md](docs/peer-bootstrap-sketch.md) | NTRIP caster mode for peer-to-peer bootstrap. |
| [caster-ephemeris.md](docs/caster-ephemeris.md) | Spec + implementation plan: encode F9T RXM-SFRBX as RTCM 1019/1042/1046 so the local caster serves broadcast ephemeris alongside observations. Removes external NTRIP dependency for peer bootstrap. |
| [ntrip-mdns-discovery.md](docs/ntrip-mdns-discovery.md) | Spec: mDNS service advertisement for NTRIP peer discovery. Caster announces `_ntrip._tcp`, client discovers and selects by accuracy/proximity. |
| [ntrip-recaster-options.md](docs/ntrip-recaster-options.md) | Survey of open-source NTRIP re-casters (relay/relabel/aggregate) for local lab correction distribution: `str2str` (RTKLIB) to pull GA's SSR once and re-serve relabeled over the LAN, BNC `Combi` for multi-AC combination (float only — single AC for AR, per ac-datum-mixing), BKG Icecast caster if we outgrow `str2str`.  Complements `ntrip_caster.py` (upstream/base-station role). |
| [local-caster-onocoy-plan.md](docs/local-caster-onocoy-plan.md) | **Architecture plan (2026-07-22) for local casters + Onocoy contribution.**  Organizing matrix: two directions (correction re-caster, HW-free → gt; observation caster → receiver host) × two tiers (production=calibrated UFO1/Mosaic-T, externally visible; research=CHOKE1/experimental, never external).  Boxes: SSR relay + obs fan-out (str2str/systemd on gt), production obs caster tee'd to Onocoy + LAN + log, research casters.  Onocoy = NTRIP-server **push** to servers.onocoy.com:2101, RTCM3 MSM7+1005/1006/1033/1230, <1 s, ITRF ARP — used as a free third-party consistency check, not for rewards.  Engine's `ntrip_caster.py` can't push (pull-only) → use str2str / receiver built-in server.  Stream logging = provenance + pos_replay fodder.  Phasing + open decisions.  Pairs with [ntrip-recaster-options.md](docs/ntrip-recaster-options.md). |
| [ticc-calibration-2026-03-19.md](docs/ticc-calibration-2026-03-19.md) | TICC calibration procedure and results. |
| [hw-labels.md](docs/hw-labels.md) | Hardware labeling conventions for beads. |
| [draft-dupage-inquiry.md](docs/draft-dupage-inquiry.md) | Draft inquiry to DuPage County about GNSS antenna siting. |
| [packaging-plan.md](docs/packaging-plan.md) | Plan for making peppar-fix pip-installable from GitHub Releases. Phased: pyproject.toml stub (done), flatten imports, versioned releases. |
| [ptp4l-supervision.md](docs/ptp4l-supervision.md) | Layered ptp4l clockClass supervision via systemd. Three layers: engine (Python UDS), wrapper (pmc command), systemd ExecStopPost. Covers clock-class mapping, ptp4l config, privilege model, and example unit file in `deploy/`. |
| [extts-lifecycle.md](docs/extts-lifecycle.md) | EXTTS (PPS IN/OUT) initialization lifecycle. Bootstrap owns pin programming; engine inherits and verifies. Covers PTP profile extension for IN+OUT pins, PEROUT for TICC, fd persistence, platform matrix (i226/E810), and phased migration path. |
| [i226-perout-500ms-bug.md](docs/i226-perout-500ms-bug.md) | **Read before debugging PEROUT.** i226 PEROUT can fire at 500ms phase — hardware issue, confirmed with SatPulse. Some boards always land wrong regardless of software. Detection, workaround, affected hosts. |
| [qerr-correlation.md](docs/qerr-correlation.md) | **Read before modifying qErr matching.** How qErr is matched to TICC timestamps, why TICC qVIR is the definitive check (no DO noise, no servo), why chA-chB diff qVIR is wrong, the TIM-TP-initiated window matching design, queue_remains principle. |
| [glossary.md](docs/glossary.md) | **Terminology reference.** DO vs PHC, rx TCXO vs bare TCXO, gnss_pps vs do_pps, all acronyms (EKF, LQR, TDEV, ADEV, TICC, PPP, SSR, etc.). |
| [receiver-clock-hierarchy.md](docs/receiver-clock-hierarchy.md) | The three tiers of GNSS receiver clock architecture (PPS-only / good-TCXO+raw-obs / geodetic+external-clock), the "second hop" cost of disciplining off an internal-TCXO PPS, and the single-oscillator design (geodetic receiver on an external DO — e.g. GNSSDO+ = Mosaic-T + Rakon OCXO).  Linked from the README. |
| [wr-gm-research.md](docs/wr-gm-research.md) | White Rabbit GM architecture review: softpll internals (helper/main/external PLLs), how GM uses PPS vs 10 MHz, qErr injection points, PEROUT at 10 MHz, two integration paths (PHC PEROUT vs OCXO+ClockMatrix). |
| [ticc-baseline-2026-04-01.md](docs/ticc-baseline-2026-04-01.md) | F9T PPS baseline TDEV(1s)=2.3 ns (2h runs); i226 TCXO PEROUT TDEV(1s)=1.170 ns (0.2% spread); servo bandwidth implications; EXTTS quantization analysis. |
| [ticc-vs-extint-do-observer-experiment.md](docs/ticc-vs-extint-do-observer-experiment.md) | **Design for the "is a per-clock TICC worth it?" overnight experiment.**  Compares DO-PPS observers as servo arms — EXTINT/Arm3 (~5–10ns) vs TICC/Arm4 (~60ps) vs both vs TDCP/Arm5 (~15–40ps, needs no TICC) — metric = TDEV of TICC chA detrended (logged in every arm; `--no-ticc` keeps logging while gating the servo arm, so the metric stays independent).  Arms via `--no-ticc`/`--no-extint`; time-only + `--known-pos`.  Primary host MadHat OCXO (needs a DO-PPS→F9T-EXTINT jumper added), TimeHat TCXO as cross-DO check.  Interleaved 2h arms ×2 cycles overnight for reproducibility; reset-free is required.  Decision framework incl. the likely punchline: if TDCP≲TICC, one *shared* TICC for validation beats one-per-clock.  Plan/design, not yet run. |
| [receiver-comparison-2026-06-01.md](docs/receiver-comparison-2026-06-01.md) | F9P vs F10T vs F9T head-to-head from the MadHat F9P swap overnight.  chB(1s): F9P 2.11ns matches F9T 2.32ns, F10T 4.56ns is 2× noisier at short τ.  TDCP σ_trim: PiFace F9T 37ps, F9P 71ps, clkPoC3 F9T 109ps (F9T per-unit variance > F9P-vs-F9T gap).  Value verdict: F9P (~$200) is the winner at short τ where the servo loop lives; F10T (~$250) didn't let MadHat lock overnight; F9T (~$340) remains the cleanest-spec but the per-unit variance is real. |
| [moonshot-overnight-2026-05-31.md](docs/moonshot-overnight-2026-05-31.md) | First full overnight of the merged moonshot stack on all 4 hosts.  PR #117 vectorize validated in production (28% → 0.94% CPU, py-spy on clkPoC3 Pi 4 at peak burst window).  Per-host TDEV r1/r2/overnight reproducibility; cross-host PiFace↔clkPoC3 CDF post-bootstrap at p95 = 14.3 ns (4.4× better than the pre-merge baseline, ~10× from the 1 ns moonshot target).  Open follow-ups: exitFiveToServoReset for the MadHat cold-start cascade pattern; freerunPhcPeroutArm to unblock TimeHat DO char. |
| [weak-antenna-doom-loop-2026-04-29.md](docs/weak-antenna-doom-loop-2026-04-29.md) | First systematic record of what the engine looks like under sustained PR-multipath bias (Patch4, ANA-MB2 indoor on a window ledge).  6m18s ZTD-trip limit cycle, 9.6 cycle-slips/min, P_IB ≈ 0.10 (NL never attempts), σ_position fine but ZTD poisoned.  Comparison hypothesis filed for the same-model outdoor Patch3 capture. |
| [ppp-ar-design.md](docs/ppp-ar-design.md) | Design for PPP-AR: phase bias sources, filter changes, ambiguity resolution algorithm, 4-phase implementation plan, 5 validation tests. |
| [ppp-ar-filter-redesign.md](docs/ppp-ar-filter-redesign.md) | **Read after ppp-ar-design.md.** Why IF ambiguities are not integer, WL/NL decomposition, Melbourne-Wubbena tracker + narrow-lane resolver design, ~210 lines total. Supersedes Phase B/C integrality approach. |
| [ztd-state-for-ppp-ar.md](docs/ztd-state-for-ppp-ar.md) | **Next step for AR.** PPPFilter needs a ZTD state to separate atmospheric drift from position. Without it, NL fixes lock in tropospheric bias — cross-host agreement stuck at ~5m horizontal despite correct integers. |
| [filter-stiffness-redesign.md](docs/filter-stiffness-redesign.md) | **Active rewrite — read before touching solve_ppp.py P/Q.**  NAV2-seeded filter init + physics-tight ZTD prior + collapse Phase 1/Phase 2 dichotomy + replace scrub_for_retry's 100m blowup.  Diagnoses systematic SSR PB bias landing in position/ZTD as cascading symptom of wrong-stiffness-routing.  Includes 4-stage convergence walkthrough (why Q≈0 doesn't trap at NAV2 floor), code-site map, validation plan vs cc23840 baseline, deferred sub-change 5 (innovation-correlation discontinuity detector). |
| [arp-survey-strategy.md](docs/arp-survey-strategy.md) | **Position acquisition — `peppar-survey --auto`** (the position half of the decoupled architecture; engine owns time, survey owns position).  Mental model: a contingency-free FLOOR (24 h dual-freq raw → PRIDE → cm, anywhere, no base/AR/phase-biases) + graceful degradation → the whole problem is one axis = time-to-fix.  `--auto` cascade (A: RTKLIB relative-baseline, minutes/cm, open archive or stream-log; B: real-time PPP-AR; C: PRIDE-rapid ~1 day; D: PRIDE-final floor; D′: self-survey), each capability-gated, falling through to the floor.  Inputs: rx caps + NAV2 region + sourcetable search.  Testable because heuristics only buy speed (predicates + ranking fn + region→source data table; floor guarantees correctness).  ETRS89/NAD83→ITRF2020@epoch datum rule in one place.  Tier A proven 2026-07-03 (London); `--auto` + `--baseline` + discovery all BUILT (#265/#271/#273).  **Base discovery has two sources**: NTRIP sourcetable ranking (EUREF) and archive station-catalogue ranking (NGS CORS — ~1650 operational sites that appear in no sourcetable, published as one daily file `coord_20/itrf2020_geo.comp.txt`, cached 30 d so field runs work offline) — so in North America `--auto` needs **no `--caster-host`** (added 2026-08-03 after Newton WI showed WMTW at 20.6 km being ignored for a 207 km caster mount).  Pairs with [time-only-architecture.md](docs/time-only-architecture.md) and [peppar-survey-modes.md](docs/peppar-survey-modes.md). |
| [peppar-survey-modes.md](docs/peppar-survey-modes.md) | **On-demand vs ongoing peppar-survey** (design, 2026-08-03).  No args = on-demand: owns the receiver, JSONL refinements to stdout (human render on stderr), exits when no further refinement is possible *from data that exists now* — naming what's blocked and distinguishing "converged" from "gave up".  `--daemon` = ongoing: owns nothing, consumes a raw-obs spool (`.done`-marker contract, reusing the pos_replay Group-A format), writes the same `.survey.toml`.  **Coordination rule: peppar-fix publishes facts, never commands** — it must run identically where peppar-survey was never installed.  `SurveyLifecycle` (ACQUIRING/REFINING/SURVEYED) already gates raw-obs logging = "when is a survey wanted".  **ARP epoch = `mount_sn`** — already the antenna-mount tag, already bumped by the NAV2 watchdog `step`, already gating seeds via `filter_current_mount`.  Two gaps around it fixed 2026-08-03: peppar-survey never *read* it (`--mount-sn` defaulted to 0, so every post-move survey was written fine and then silently discarded by the engine), and nothing bumps it when a **human** moves the antenna with the engine stopped (→ `scripts/peppar_arp_moved.py`).  Key insight: **a stale pin's σ is small**, so σ ranking structurally cannot catch a moved antenna — the epoch gate must be categorical, never a weighting. |
| [position-drift-investigation-2026-06.md](docs/position-drift-investigation-2026-06.md) | **Read before re-chasing the free-position drift.**  Week-long investigation (seedNullDriftExperiment I-222315).  Rules out seed / solid-tide / PCV / AR / Q_pos / receiver as drivers; the drift is a time-varying (pos,ZTD,clk)-null excitation, and state constraints RELOCATE the misfit (don't eliminate it — "misfit-conservation").  The RTKLIB recipe (`--clock-model wno` + `--q-ztd-antpos 1e-4` + `--q-pos-converged 1e-9` + float, default tie) bounds free position around truth; committed PR #145.  PRIDE yardstick: residual is PRODUCTS-limited not filter-limited; cm needs final products → pin ARP via PRIDE-final survey + `--no-antposest`.  Acceptance bar: stable ±10cm + honest σ (10cm≈333ps).  Spawned pickyEaterSSR (I-210733). |
| [simulators-and-replay.md](docs/simulators-and-replay.md) | **Vocabulary + design for the filter test rigs.**  Two paradigms: a **simulator** synthesizes the world from a truth you set (synthetic, truth-down — bounded by the forward model); a **replay harness** re-runs the world you recorded against a truth you measured (real inputs, external post-proc truth).  Three rigs: `servo_sim` (exists, synthetic, time/servo `DOFreqEst`), **`pos_sim`** (new, synthetic, position filter — reuse `solve_ppp`'s forward model to emit obs from known truth, dial the (pos,ZTD,clk) null), **`pos_replay`** (new, captured-replay harness for position — reference captures {logged inputs + OPUS/PRIDE/NRCan cross-checked truth incl. external ZTD(t)+CI}, product-matched ablation, case library).  Complementary; `pos_sim` first (cheap/exact), `pos_replay` second (real/localizing).  Naming-honesty: a replay harness does NOT simulate. |
| [pos-replay-capture-manifest.md](docs/pos-replay-capture-manifest.md) | **Concrete capture spec for `pos_replay`.**  A *reference capture* = {logged inputs + offline truth}.  Group A inputs = RAW byte streams (UBX RAWX/SFRBX/NAV-*/TIM-TP, RTCM SSR, broadcast eph, TICC) each stamped `recv_mono` (CLOCK_MONOTONIC) for deterministic full-pipeline replay; Group B = engine outputs incl. **PPPFilter pos+ZTD + σ** (needed for the error/own-σ false-confidence score — `--filter-state-log` is FixedPosFilter-oriented today, must extend); Group C = METAR pressure (ZHD anchor) + `antennas.json` ARP/height.  Truth: captured RAWX→RINEX (`rinex_writer.py`)→ PRIDE (epoch-wise) + OPUS-S (GPS-only control) + **NRCan (multi-GNSS + ZTD(t)+CI, the crown jewel)** + surveyed ARP.  Pins the ZTD convention (ZHD-from-pressure, mapping fn, antenna height, real-time-lag), product-matched replay (filter-vs-corrections), case library, bundle layout.  NOTE: capture runs the position filter (NOT `--no-antposest`).  Build order: pressure-log + state-log σ (small) → unified raw-capture wrapper → truth-ingest+ZTD-compare → deterministic replay+product-swap. |
| [clockmatrix-bootstrap-plan.md](docs/clockmatrix-bootstrap-plan.md) | ClockMatrix supplements PHC: bootstrap sequence, FCW handoff, hybrid architecture for Timebeat OTC. |
| [igc-kernel-patches.md](docs/igc-kernel-patches.md) | **Read before driver work on i226 hosts.** Inventory of igc patches (ppsfix + adjfine), per-host deployment status, verification checklist, incident history. v3 adjfine patch reduces but does not eliminate TX timeout cascade — EXTTS wedges after ~44 min MTBF. |
| [lambda-ar-plan.md](docs/lambda-ar-plan.md) | LAMBDA integer least-squares AR: decorrelation, search, ratio test, partial AR. Replaces per-satellite rounding — handles ZTD-ambiguity correlation, faster TTFF, stronger validation. ~150 lines. |
| [sv-lifecycle-and-pfr-split.md](docs/sv-lifecycle-and-pfr-split.md) | **Per-SV state machine** (TRACKING → FLOAT → WL_FIXED → NL_SHORT_FIXED → NL_LONG_FIXED, plus SQUELCHED) orthogonal to `AntPosEstState`.  Defines the **fix set** (short-term + long-term members) and splits the old PFR monitor into three stateless per-eval monitors: **FalseFixMonitor** (wrong-integer detection on short-term members), **SettingSvDropMonitor** (elev-weighted graceful drop), and **FixSetIntegrityAlarm** (systemic failure catcher).  Replaces the PFR L1/L2/L3 cascade; motivated by day0419d ~2 PFR L3/hr observations caused by a level-persistence bug. |
| [lab-operations.md](docs/lab-operations.md) | **Read before running on any lab host.** Deployment procedure, pre-flight checklist, stumble analysis, standard host layout, future automation work. |
| [position-bootstrap-reliability-plan.md](docs/position-bootstrap-reliability-plan.md) | Implementation plan to make cold-start Phase-1 trustworthy so lab runs no longer need `--known-pos`. Residual-consistency gate, NAV2 cross-check, harder abort, σ tightening, state persistence. 6 work items, W1–W5 core, W6 fallback. |
| [l5i-l5q-phase-bias-empirical.md](docs/l5i-l5q-phase-bias-empirical.md) | Empirical proof (19 GPS SVs, lab day0418h) that CNES L5I and WHU L5Q phase biases differ by mean −0.73 m with 1.46 m SD — far beyond the λ/4 carrier-phase offset.  RTCM "phase bias" bundles satellite-side delays + AC-datum offsets + reference-receiver I/Q group delays; can't be substituted with a single-cycle correction.  Supports the dual-mount fusion path. |
| [bds-ppp-integration.md](docs/bds-ppp-integration.md) | Research notes on BDS PPP integration: the 1500 ns ISB isn't the 14 s BDT/GPST offset (already correct).  Real causes are (1) wrong RTCM 1260 signal-code map — fixed in PR #3, and (2) BDS-3 broadcast clock is B3I-referenced so IF(B1I,B2a-I) needs TGD1+TGD2 not TGD1 alone — still open.  Includes the complete RTCM 3.3 BDS sig-id table and PRIDE/RTKLIB references. |
| [bds-b2a-phase-bias-survey-2026-05-09.md](docs/bds-b2a-phase-bias-survey-2026-05-09.md) | **Why BDS is dropped from `systems=` on dual-mount + L5-tracking hosts.**  Probe of every IGS-IP SSR mount (CNES / WHU / CAS / CHC / SHAO / GFZ / BKG): no AC publishes BDS B2a-I phase biases as of 2026-05-09.  CNES has B1I+B3I+B2I-legacy; WHU has GPS phase only; CAS has B1I+B1C; CHC has B1I+B2I-legacy; rest publish no BDS phase biases at all.  F9T-20B / F10T track B2a-I and report under RINEX `C5I`/`L5I` — the matching phase bias doesn't exist anywhere we can consume.  Operational mitigation (config drop) + engine-side gate widening (queued as I-165118 Fix #2). |

## Lab Documentation Pointers

**The `../timelab` repo is the reference for the physical timelab** used
to develop PePPAR-Fix — wiring, gear, receivers, TICCs, antennas, surveys,
udev rules, and lab utility scripts.  It is a **separate private GitHub
repo** (`git@github.com:bobvan/timelab.git`, sibling checkout at
`../timelab`, branch `main`) — **not** part of the PePPAR-Fix repo.  Treat
it as living documentation: **when you change the physical lab (move a
receiver, re-cable a splitter, add/retire a host, re-survey an ARP, swap an
antenna), update the matching file in `../timelab` and `git commit` +
`git push` it.**  A change that isn't pushed is invisible to the other
agents and to future sessions — don't leave lab state stranded in an
uncommitted working tree.

The `timelab/` directory (same content, at the town root) has authoritative lab state:

| File | Contents |
|---|---|
| `timelab/topology.md` | Current wiring: antenna→splitter→receiver→host→TICC, PTP domains, NTP |
| `timelab/gear.md` | Hardware inventory: every host, receiver, TICC, antenna, with specs |
| `timelab/usb-identification.md` | USB serial numbers, udev policy, device identification |
| `timelab/status.md` | Active experiments and recent results (may be stale) |
| `timelab/calibration.md` | TICC calibration procedures |
| `timelab/99-timelab.rules` | Universal udev rules (deployed to all Pis) |
| `timelab/scripts/` | Lab utility scripts (ticc_read.py, calibration_capture.py, etc.) |
| `timelab/antennas.json` | Lab antenna database — one entry per antenna with surveyed ARP, σ, history, cross-checks.  Gitignored at both ends — coords stay out of the public PePPAR-Fix repo.  Read this when you need the lab ARP (e.g., to seed `--known-pos` for a dev/test run, or to validate a cold-start convergence against a reference).  Engine reads it in-memory at startup via `arp_label = "..."` in the host config; never written by the engine. |

If you need to know what's physically connected where, start with
`topology.md`. If you need device specs, start with `gear.md`.
Operational positions are **not** hardcoded in PePPAR-Fix — they're
loaded from `state/receivers/<uid>.json` at runtime (written by
Phase 1 bootstrap) and verified against `timelab/antennas.json`.

## Code style — naming honesty

> One of my pet peeves in code is misnomers.  Maybe the name of a
> class made sense when it was first created, but as it ages, the
> code changes, and the name doesn't, to the point where the name
> is just plain wrong.  In the days of emacs and vi, this was more
> forgivable, but I've had little patience for this ever since IDEs
> were invented.  What, two decades ago now? — Bob

When you find an identifier whose name implies one thing but whose
implementation does another, log it in
[`docs/misnomers.md`](docs/misnomers.md).  Don't batch-rename — pollutes
git blame for no test signal — but fix opportunistically when the
surrounding code is being touched.  Severity scale and entry format
are documented in that file.

Names that are wrong about *what kind of signal* a thing measures are
the most dangerous.  See the 2026-04-28 `wl_drift_monitor` rework for a
case where the docstring framed it as a "wrong WL integer detector"
while empirically it was reacting to pseudorange-domain noise — that
mis-framing burned hours of investigation.

## Acceptance Criteria for Beads

Your bead is NOT done until:
- Code changes are committed to your feature branch
- A test plan is documented in a bead comment
- If the bead has `hw:` labels: code stays on branch (`--no-merge`),
  lab validation happens separately
- If pure software: include unit tests or a verification script
- Research tasks MUST produce a markdown document in `docs/`

Do not close a bead with "Completed with no code changes" unless you
can explain in a comment exactly what you verified and why no changes
were needed.

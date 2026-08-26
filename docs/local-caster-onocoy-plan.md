# Local casters / re-casters + Onocoy contribution — architecture plan

*2026-07-22 — planning doc, not a build order. Lays out the full option space
for local NTRIP casters/re-casters, contributing our observations to Onocoy,
and the production-vs-research split that keeps them from contaminating each
other. Goal: don't foreclose any path. Builds on
[ntrip-recaster-options.md](ntrip-recaster-options.md) (Delta's tool survey),
[caster-ephemeris.md](caster-ephemeris.md), [peer-bootstrap-sketch.md](peer-bootstrap-sketch.md),
[ac-datum-mixing.md](ac-datum-mixing.md), and
[coordinate-reference-frames.md](coordinate-reference-frames.md).*

## The organizing idea: two directions × two tiers

There are **two caster directions** (Delta's key distinction) and **two
service tiers**. Every box below is one cell of this matrix.

**Directions:**
- **Correction re-caster** — consume *corrections* upstream (GA SSR, PTBB obs),
  re-serve *downstream* to lab consumers. No local hardware. → runs on **gt**.
- **Observation caster** — take raw obs from *our* receivers, serve them
  out (LAN and/or to Onocoy). Tied to the receiver's host (hardware stream).

**Tiers:**
- **Production** — calibrated ARP (UFO1), stable, durable, externally visible
  (Onocoy). Must be trustworthy: Onocoy grades us.
- **Research** — experimental receivers/antennas/streams, transient, may use
  CHOKE1 (un-calibrated) or oddball configs. Never streamed externally.

|  | Correction re-caster (gt, HW-free) | Observation caster (receiver host) |
|---|---|---|
| **Production** | SSR relay + obs fan-out (systemd on gt) | UFO1/Mosaic-T → LAN + **Onocoy** |
| **Research** | ad-hoc capture fan-out | CHOKE1 / NetRS / F9P experiments |

The production/research split is **mandatory once we stream to Onocoy** — they
cross-check our station against their network, so production must be a
calibrated, stable, honest station and must never carry a research experiment.

## Component inventory

**Receivers (Onocoy/caster fodder):**

| Receiver | Bands / GNSS | RTCM3 MSM7 + NTRIP-server push? | Best role |
|---|---|---|---|
| **SparkPNT SXT-D** mosaic-T (on UFO1, `sxt-d.VanValzah.Com`) | L1/L2/L5, GPS+GLO+GAL+BDS | Yes — Septentrio built-in NTRIP server, full MSM7 + 1005/1033/1230 | **Production Onocoy station** |
| **ZED-F9P** (idle) | L1/L2, GPS+GLO+GAL+BDS | Yes — MSM7 out; push via str2str or ArduSimple firmware | Onocoy #2 / research |
| **Leica GRX1200 GG Pro** (on UFO1) | L1/L2, GPS+GLO | RTCM2/3 — **verify live MSM streaming** (we use it for OPUS file logging) | Keep as OPUS ARP-truth; caster only if MSM confirmed |
| **Trimble NetRS** (just arrived) | L1/L2, **GPS-only** | RTCM3 — GPS-only limits Onocoy value | Research / external-clock PoC (per [gpsdo-noise-and-external-clock.md](gpsdo-noise-and-external-clock.md)) |

**Antennas:** **UFO1** (SFESPK6618H, **NGS-calibrated**, OPUS-Static ARP σ≈6 mm,
ITRF) — the *only* calibrated ARP → **production**. CHOKE1 (un-calibrated) →
research only. Onocoy needs a calibrated ARP + a recognized antenna descriptor
(1033/1008); UFO1 qualifies, CHOKE1 does not.

**Upstream correction sources:** GA `SSRA03IGS0` (float SSR, our production),
`BCEP00BKG0` (eph), PTBB/BRUX obs (known-good validation). See CLAUDE.md.

**Caster software (from Delta's survey):** `str2str` (relay/relabel + `ntripc`
caster + **NTRIP-server push** — one tool covers pull-fan-out AND Onocoy push),
BNC `Combi` (multi-AC float combine, never AR), BKG Icecast caster (managed,
Docker), our `ntrip_caster.py` (serves MSM4 from u-blox RAWX — pull-only, no
push).

## The boxes

### Box 1 — Internal SSR relay (gt, systemd, HW-free) — *do first*
`str2str` pulls GA `SSRA03IGS0` **once**, re-serves on the LAN under our own
mountpoint (e.g. `gt:2101/SSR-IGS03`). Lab hosts + PTBB capture engines pull
from gt instead of each opening a GA connection. Cuts upstream connection count,
decouples the lab from GA outages (gt holds last-good), single point to swap
when the upstream mount changes again (as `SSRA00BKG0`→`SSRA03IGS0` just did).
Add `BCEP00BKG0` the same way. **Internal only.**

### Box 2 — Internal obs fan-out (gt, systemd, HW-free)
`str2str` pulls PTBB + BRUX **once** each → LAN mounts. The multi-stream PTBB
capture then opens 1 external PTBB + 1 external BRUX instead of N — polite to
IGS-IP and the scaling fix for >~4 correction streams. Same box class as Box 1.

### Box 3 — Production observation caster + Onocoy (UFO1/Mosaic-T host)
The Mosaic-T on UFO1 emits RTCM3 **MSM7** (1077/1087/1097/1127) + **1006** (ARP,
ITRF) + **1033** (antenna) + **1230** (GLO biases). Fan that one stream three ways:
1. **Onocoy push** — NTRIP-server to `servers.onocoy.com:2101` with a
   per-station credential (Onocoy dashboard → Reference Stations → NTRIP
   Credentials). <1 s latency for rewards. Our external consistency check.
2. **Local LAN caster** — our own base station for local RTK / relative-baseline
   surveys (pairs with [arp-survey-strategy.md](arp-survey-strategy.md) Tier A).
3. **Raw log** — see Logging below.

Cleanest topology: **receiver's built-in NTRIP server → Onocoy directly** (no
extra moving parts, lowest latency), and **str2str tees** a second copy to the
LAN caster + file log. Or the Septentrio pushes to Onocoy while `str2str`
(client on the Mosaic's second RTCM output) handles LAN + log.

### Box 4 — Research observation casters (various, transient)
F9P/NetRS on CHOKE1 or bench, experimental RTCM, correction-stream comparisons.
Never external. May reuse `ntrip_caster.py` or str2str ad hoc.

## Onocoy specifics (contribution side)

- **Endpoint:** NTRIP-server **push** to `servers.onocoy.com:2101`, per-station
  credential; mountpoint name is cosmetic (dashboard only).
- **Messages:** RTCM3 MSM (MSM7 preferred) + 1005/1006 + 1033 + 1230; **≥ MSM4**
  or they drop it. **< 1 s** end-to-end latency.
- **ARP frame:** ITRF — we already picked ITRS as the canonical frame *because*
  of the Onocoy signup ([coordinate-reference-frames.md](coordinate-reference-frames.md));
  UFO1's ARP is stored ITRF2020@epoch, so it's ready.
- **Antenna descriptor:** must be a type Onocoy recognizes for PCO/PCV —
  confirm `SFESPK6618H` maps cleanly (we already NGS-inject its antex for PRIDE).
- **Value:** not the crypto — Onocoy validates every contributing station
  against its network, so a green station there is an **independent third-party
  attestation** that our UFO1 ARP + antenna + receiver chain are self-consistent
  to their tolerance. A cheap, continuous external check on the production chain.

## Engine caster vs off-the-shelf

Use **str2str** for relay/fan-out (Boxes 1–2) and as the tee/push hub (Box 3):
it already does NTRIP-server push, which `ntrip_caster.py` **cannot** — that's
the deciding capability for Onocoy. Use the **receiver's built-in NTRIP server**
for the actual Onocoy push where possible (lowest latency, fewest parts).
Keep **`ntrip_caster.py`** for its niche: serving the engine's *own processed*
observations / the peer-bootstrap + caster-ephemeris direction
([caster-ephemeris.md](caster-ephemeris.md), [peer-bootstrap-sketch.md](peer-bootstrap-sketch.md))
— not for corrections or Onocoy. Don't grow it into a general caster; that's
re-inventing str2str/BKG.

## Logging — yes, log the streams

Worth doing for every box, cheap (RTCM is compact):
- **Provenance / audit** — what we served/pushed and when (essential if Onocoy
  ever flags our station: we have the raw record to defend it).
- **Replay fodder** — raw RTCM byte streams stamped `recv_mono` are *exactly*
  the [pos-replay-capture-manifest.md](pos-replay-capture-manifest.md) Group-A
  inputs. A logging caster doubles as a reference-capture recorder.
- **Debug** — reproduce a bad epoch offline.

How: `str2str` can `-out file://` in parallel with its caster/push output (tee),
or a dedicated logger consumes the LAN mount. Rotate (logrotate), timestamp
filenames by UTC day, **pull to gt** for archival (RAIDZ-3; per the Lab Storage
Warning). Keep production and research logs in separate trees.

## Production vs research separation (concrete)

| Aspect | Production | Research |
|---|---|---|
| Antenna / ARP | UFO1, calibrated, ITRF | CHOKE1 / bench, any |
| Receiver | Mosaic-T (stable) | F9P / NetRS / whatever |
| External (Onocoy) | Yes | **Never** |
| Uptime | systemd, durable | ad-hoc |
| Config tree | `config/production/…` | `config/research/…` |
| Log tree | `data/caster/prod/…` | `data/caster/research/…` |

## Suggested phasing (not overnight)

1. **Box 1** (SSR relay on gt, systemd) — immediate win, decouples the lab from
   GA, and we just felt the pain of a dead upstream mount.
2. **Box 2** (obs fan-out on gt) — folds the current PTBB capture behind one
   upstream pull; enables scaling the correction-stream comparison.
3. **Box 3 read-only first** — bring the Mosaic-T's RTCM3 up on the LAN caster +
   logging, validate MSM7 completeness + latency + ARP, *before* pushing to
   Onocoy.
4. **Onocoy push** — register the station, push, watch its dashboard grade.
5. **Box 4 / more receivers** — F9P second station, NetRS external-clock PoC.

## Open decisions for Bob

- Which host runs the gt casters — literally gt, or a container on gt?
- Mosaic-T → Onocoy via its **own** NTRIP server, or via str2str tee? (latency
  vs one-hub simplicity)
- Do we want a **managed** caster (BKG Icecast/Docker) from the start, or grow
  into it from str2str?
- Confirm `SFESPK6618H` is an Onocoy-recognized antenna descriptor.
- Second Onocoy station (F9P) on CHOKE1 for network diversity, or keep CHOKE1
  research-only?

## Deployment log & learnings — Box 1/2 built 2026-07-22

**Deployed on gt** (`str2str`, rtklibexplorer/demo5 fork built in `~/opt/RTKLIB`,
symlinked `~/bin/str2str`), three `systemctl --user` services (linger enabled →
boot-start; creds in a 600 `~/opt/ntrip-relay/relay.env`; each tees a rotating
RTCM log to `~/opt/ntrip-relay/logs/`):

| Service | Serves | Source |
|---|---|---|
| `ntrip-ssr-relay` | `ssr.ntrip.VanValzah.Com:2101/SSR` | products.igs-ip.net `SSRA03IGS0` |
| `ntrip-obs-relay` | `obs.ntrip.VanValzah.Com:2102/PTBB` | igs-ip.net `PTBB00DEU0` |
| `ntrip-eph-relay` | `:2103/BCEP` (broadcast eph) | igs-ip.net `BRUX00BEL0` in-band eph |

DNS: `ssr.ntrip` / `obs.ntrip` are CNAMEs → `gt.VanValzah.Com` → 10.168.60.22
(trusted LAN). Service-class names (`SSR`/`PTBB`/`BCEP`), not backend names, so
an upstream-AC swap is invisible to clients.

**Hard-won facts about `str2str` as a re-caster:**
- Its `ntripc` caster **fans out to many clients** ✓ (verified 2 concurrent).
- **One mount per port** — each stream is its own str2str instance + port. This
  is why obs/SSR/eph are on 2102/2101/2103, and why the engine needed
  `--eph-caster`/`--eph-port` (obs and eph can't share a port here).
- **NTRIP v1 / HTTP/1.0 only** — `ntripc` ignores HTTP/1.1 and replies
  `ICY 200 OK\r\n` then streams immediately (no `\r\n\r\n`). Engine fixes:
  `--ntrip-http10` (minimal v1 request) + v1 header parse (stop at first `\r\n`).
- **No TLS input** — pull from plain-HTTP casters (products/igs-ip.net:2101),
  not GA:443.

**BCEP is inaccessible to us** — `BCEP00BKG0`/`00CAS0`/`03BKG0` all **403** on
products.igs-ip.net for our `bob` account, and GA's is TLS-only (str2str can't).
So the "BCEP relay" is sourced from **BRUX's in-band broadcast eph** instead —
functionally a broadcast-eph stream, just not the BKG product. (If we ever need
the real BCEP, write a tiny TLS-capable Python bridge using our own
`NtripStream`, or get products BCEP entitlement.)

**Engine changes committed** (74e6484, 6c3d8c7): `--ntrip-http10`, NTRIP-v1
header parse, in-band eph routing (`run_msm_ntrip_source` forwards `beph`),
`--eph-caster`/`--eph-port`/`--eph-tls`.

**OPEN — blocks moving the capture behind the fan-out:** with obs pulled via the
str2str relay, the correction gate reports `broadcast_ready=False`
(`waiting_broadcast`, `broadcast_age=N/A`) **per epoch**, so 0 epochs solve —
even with a dedicated fresh BRUX eph relay (eph loads fine: `G8 E12 C8`). The
same engine on a **direct** feed solves normally, so it's the relay/http10 obs
path interacting with the **obs↔correction recv_mono correlation gate** (see
[stream-timescale-correlation.md](stream-timescale-correlation.md)), not the eph
source or freshness. **Until this is debugged, the PTBB capture stays on its
direct igs-ip.net feed.** Next step: instrument the correction snapshot's
`broadcast_ready`/`broadcast_age` computation for a relayed-obs epoch vs a
direct one, and check whether relayed-obs `recv_mono`/`gps_time` de-correlates
from the eph store.

## Box 3 deployment log & learnings — 2026-07-24 (I-213407-main)

**Blocker found: the GNSSDO+ mosaic-T cannot emit RTCM3 natively.** It's a
timing/PPP SKU (S/N 3810231) whose permission file has **`dgnssbase=0`,
`rtkbase=0`, `rtkrover=0`, `movingbase=0`** (`lif, Permissions`). Every
`setRTCMv3Output` returns "Invalid command!" and `RTCMv3` is not a valid
`setDataInOut` output. So the "receiver's built-in NTRIP server → Onocoy
directly" option in Box 3 above is **infeasible** for this box without a
Septentrio base-license upgrade (which buys only architectural simplicity —
NOT a higher reward tier, since the tier depends on the obs, not the push path).

**Adopted path: str2str SBF→RTCM3 transcode on gt (validated live).** The mosaic
*does* stream SBF MeasEpoch on IPS2 (tcp 28800). `str2str` transcodes it:

```
str2str -in tcpcli://sxt-d.VanValzah.Com:28800#sbf \
  -msg "1077(1),1087(1),1097(1),1127(1),1006(10),1033(10),1230(10)" \
  -out ntripc://:2104/UFO1_MSM7#rtcm3 \
  -out file://…/logs/prod/ufo1_msm7_%Y%m%d_%h.rtcm3::T \
  -px 157469.2017 -4756188.1927 4232767.8802 -a "SFESPK6618H,NONE" -i "mosaic-T,GNSSDO+"
```

Decoded output = 1006/1033/1077/1087/1097/1127/1230, GPS+GLO+GAL+BDS, MSM7 @1Hz.
`-msg` MUST precede `-out`. str2str synthesizes 1006 (ARP) + 1033 (antenna
descriptor) from `-px`/`-a`, so we stamp OUR surveyed UFO1 ARP + `SFESPK6618H`
independent of the receiver's PVT/antenna state. This str2str hop is therefore
the **permanent steady state** for this receiver, not a throwaway phase-1 stage —
there is no "direct push cutover" to retire it to.

**Staged, not activated** (per Bob — Onocoy feed stays off until the mosaic's own
timing config is finished): `systemctl --user` unit `onocoy-obs-transcode.service`
(disabled) + runbook `~/opt/ntrip-relay/onocoy-activation-README.md`. Activation =
add the `-out ntrips://…@servers.onocoy.com:2101/…#rtcm3` push once the Onocoy
credential exists, then `enable --now`.

**Onocoy specifics reconfirmed:** self-surveys the station position (a sub-cm ARP
in 1006 is not required of us), hardware-agnostic, MSM7 + 1 Hz + <1 s latency,
credential-name = NTRIP username. Wheaton spacing OK (~85% of potential rewards).

**Mosaic-T antenna for its own PPS-to-UTC (separate from Onocoy):** the receiver's
antenna table (v23.3.0, a snapshot of the **NGS ANTCAL catalog** — not IGS-only)
returns `UNKNOWN` for `SFESPK6618H NONE` because that snapshot predates the
antenna's Aug-2023 NGS publication. Fix = install **AntInfo v25.1.0** (NGS snapshot
01-Apr-2025, 1010 antennas), SparkFun-hosted at
`github.com/sparkfun/SparkFun_GNSS_mosaic-T/…/antinfo/antinfo-25.1.0-mosaic-T.suf`;
then `setAntennaOffset … "SFESPK6618H NONE"` + pin the surveyed ARP. No antenna
swap needed (the Leica AX1202GG is in-DB but is GPS+GLO L1/L2 only — would lose
L5+Galileo+BeiDou).

### Log-rotation bug in the staged unit — found + fixed 2026-08-24

The `-out file://…` in the transcode command above ends in `::T`, which is
**not** a swap interval. Per RTKLIB `src/stream.c:691` it is the *time-tag* flag
(it produces the `.rtcm3.tag` sidecars); rotation is `::S=<hours>`. With
`swapintv=0` the `%Y%m%d_%h` template expands **once at process start** and a
single file grows until the service restarts.

This is the same defect that filled gt's `/home` on 2026-08-24 via the three
sibling relays (`ntrip-{ssr,obs,eph}-relay`, ~480 MB/day unbounded, two files at
4.4 GB / 4.3 GB — fixed by main under `fullSuiteTestPollution`). The Box 3 unit
was **not** in that sweep. It had not bitten only because it is staged —
`disabled`, never started, `logs/prod/` still empty — so activating it as
written would have reproduced the failure on a box that had just been cleaned up.

Fixed on gt (`~/.config/systemd/user`, backups in
`~/opt/ntrip-relay/unit-backup-20260824/`):

1. `::S=24` appended to the unit's `file://` output, matching its three siblings.
2. `ntrip-relay-retention.service`'s `find` widened `-maxdepth 1` → **`-maxdepth 2`**.
   This stream logs to `logs/prod/`, a **subdirectory**; at depth 1 the pruner
   could not see it, so daily rotation alone would still have grown without
   bound. Rotation and retention had to BOTH be fixed — either alone is a leak.

Verified: `systemd-analyze --user verify` clean on both units; retention
dry-run at depth 2 matches 0 files for deletion and newly sees the 12 current
stream files; the service runs to `Result=success`.

**Budget caveat:** the ~3.4 GB steady state in `RETENTION.md` covers the three
research relays only. This stream is 4-constellation MSM7 @ 1 Hz and has never
run, so its MB/day is **unmeasured** — measure it at activation and re-check
`RETAIN_DAYS` before trusting that number. If production provenance later needs
a longer window than the research relays, give `logs/prod/` its own retention
unit rather than raising the global `RETAIN_DAYS`.

**Standing exposure:** these units, `relay.env`, `RETENTION.md`, and
`onocoy-activation-README.md` are durable tooling living **outside version
control** (`~/.config/systemd/user`, `~/opt/ntrip-relay`). That is exactly why
the fourth unit was missed by a fix that touched the other three — nothing
enumerates them. Tracked with the post-London "hunt durable-tools-outside-repo"
item; this doc is currently the only versioned record of the Box 3 unit.

### Four units collapsed into one template — 2026-08-24

The four str2str relays (SSR / PTBB obs / BCEP eph / Onocoy transcode) were
separate near-identical unit files. That duplication is the structural reason a
rotation fix earlier the same day reached three of them and missed the fourth.
They are now one `str2str-relay@.service` with per-instance files:

```
deploy/ntrip-relay/units/str2str-relay@.service   one unit, one ExecStart
deploy/ntrip-relay/bin/str2str-relay-exec         composes the command
deploy/ntrip-relay/instances/{ssr,obs,eph,onocoy}.env
~/opt/ntrip-relay/relay.env                       credentials, still not in git
```

`systemctl --user enable --now str2str-relay@ssr`, and so on. Rotation, restart
policy, and logging are defined once. Onocoy is `@onocoy`, installed but not
enabled.

**Verified argument-identical before deployment.** Each instance's composed
argv was diffed against the corresponding legacy unit's expanded `ExecStart`,
credentials and `%Y%m%d_%h` templates included: 8/8/8/18 arguments, all
identical. The refactor changes structure, not behaviour.

**Why a wrapper rather than a pure `ExecStart`.** systemd does not recursively
expand `${VAR}` inside a value loaded from an EnvironmentFile — measured: argv
arrives holding the literal `ntrip://${IGS_USER}:x@h/M`. str2str takes
credentials only inside the URL, so the URL has to be assembled after the
credentials are in scope. The wrapper does that with bash indirect expansion,
no `eval`. Related measured behaviours: unquoted `$ARGS` does word-split; the
instance files are shell-sourced by the wrapper (not parsed by systemd), so
`ARGS` is quoted as a whole, `$HOME` expands, and str2str's `%Y%m%d_%h` takes
**single** `%` rather than the `%%` a unit file needs.

**A third probe trap, found while writing the health check.** Beyond "NTRIP v1
is not valid HTTP so curl reports 000" and "ntripc requires a User-Agent":
str2str's ntripc services accepted connections on an internal cycle, so a
single probe is unreliable. Ten back-to-back probes all answered; probes spaced
2 s apart alternated answer / no-answer. The check retries three times.

`deploy/ntrip-relay/check.sh` enforces the invariants — no orphan or legacy
str2str unit, `::S=` on every `file://` out, every log dir inside the retention
sweep's `-maxdepth`, no quote characters in `ARGS`, no `@`/`:`/`/` in a
credential, and no drift from the repo. Each check was validated by injecting
the corresponding fault.

### Addressed by name, not by IP — 2026-08-25

The transcode was hardcoded to `10.168.13.196`, which was a **dynamic DHCP
lease** (`getIPSettings` on the receiver returned `DHCP, "0.0.0.0", ...` — no
static config at all, and reverse DNS gave the generated name `Dyn196`).  The
box was given a static address and the name `sxt-d.VanValzah.Com` the same day;
`.196` went dead within minutes.

Had the Onocoy feed been live, `str2str` would have retried a dead address
forever (`-r 10000`) and the station would have gone **silently** off the air —
compounding badly with the fact that onocoy has no mechanism to accept a
downtime notice, so we would simply have looked unreliable until someone
noticed.  It fails safe (no bad data is emitted, and a non-SBF listener on a
reused address transcodes to nothing) but it fails quietly.

Rule: reference lab instruments by DNS name in anything long-running.  The IP
was baked into eight places across two repos and a stream-map SVG; a name makes
a future re-IP one edit instead of an archaeology exercise.

**Vendor rename:** SparkFun split its business and the product is now the
**SXT-D** ("SXT-D GNSS Disciplined Oscillator Plus") from **SparkPNT**
(<https://www.sparkpnt.com/products/sxt-d-gnssdo>).  Prose here uses SXT-D; the
`gnssdo_*` host-config keys and `GnssdoActuator` are deliberately unchanged
because they are load-bearing in deployed TOMLs.  Logged in `docs/misnomers.md`.

### Box 3 LIVE — 2026-08-26

`str2str-relay@onocoy` enabled and pushing to `servers.onocoy.com:2101` at
~13.3 kbps, while still serving `gt:2104/UFO1_MSM7` on the LAN and rotating
`logs/prod/`.  Preceded by the AntInfo 26.1.0 install and ARP pin of 2026-08-25,
so the station's 1006 carries our surveyed UFO1 ARP and the receiver applies the
real SFESPK6618H phase-centre model rather than zero.

**Credential slot mapping — counter-intuitive, cost three attempts.**  onocoy's
dashboard shows a "Credential" and a "Mountpoint"; neither is used where the
name suggests:

  - "Credential" (station page) = the **auto-generated NTRIP username**, and it
    goes in the **mountpoint** slot.
  - "Mountpoint" = cosmetic, *"only informative, not public"* — **unused**.
  - The password is the one chosen under **Reference Stations -> NTRIP
    Credentials**, and is not shown on the station page.

The reason is `str2str`: it pushes with NTRIP v1, whose handshake is
`SOURCE <password> <mountpoint>` with **no username field**
(`ntrips://[:passwd@]addr[:port]/mntpnt` — empty user).  onocoy documents this
exact workaround for v1 devices: *"enter the NTRIP username as MOUNTPOINT and
password into the password field.  Leave the device's username field, if any,
empty."*

**Failure signatures**, worth knowing because the first is silent and reads like
a network fault:

| what is wrong | caster reply |
|---|---|
| wrong value in the mountpoint slot | **empty**, connection dropped |
| right slot, wrong password | **HTTP/1.1 401** |
| both right | **HTTP/1.1 200 OK** |

`GET /` returns **403 Forbidden** (no public sourcetable), which is a useful
liveness check that the caster is reachable at all.

Implementation: the wrapper composes the push from `PUSH_CRED` + `PUSH_HOST`,
taking `<PREFIX>_CRED` for the mountpoint slot and `<PREFIX>_PASS` for the
password.  Both live in `~/opt/ntrip-relay/relay.env` (0600) — never the repo,
since onocoy states the mountpoint is not public and the credential obviously
is not.

### Which streams may be contributed to Onocoy — and which must never be

Asked 2026-08-26, worth writing down because gt now carries four RTCM streams
and only one of them is ours.  `str2str` will happily transcode any of them, so
the guard has to be a rule, not a technical limit.

| Instance | Content | Contributable? |
|---|---|---|
| `@ssr` | SSR corrections (SSRA03IGS0) | **No** — corrections, not observations.  Not a reference station in any sense. |
| `@obs` | PTBB00DEU0 observations | **NEVER** — this is **PTB's** station, pulled under our BKG account for the known-good-obs diagnostic. |
| `@eph` | BRUX00BEL0 observations | **NEVER** — **ORB/EUREF's** station, same reasoning. |
| `@onocoy` | our SXT-D on UFO1 | Yes — already contributing. |

Re-contributing `@obs` or `@eph` would be passing third-party reference stations
off as our own to a network that rewards contributions.  It breaches BKG's terms
and Onocoy's, and it would corrupt a network other people use for RTK.  The fact
that the transcode is one flag away (`#ubx` / `#sbf` in, RTCM3 out) is exactly
why the rule needs to be explicit.

**Our other lab receivers are also not candidates**, for a quieter reason: they
are on the *same antenna*.  `piface` and `clkpoc3` both carry
`arp_label = "ufo1"`, fed from the UFO1 GUS splitter — the identical RF the
SXT-D already contributes.  And the only other antenna we own in Wheaton,
CHOKE1, is **0.98 m** from UFO1 (computed from `antennas.json`).  Onocoy's
validation and reward model is about spatial coverage; stations a metre apart
carry no independent information and would read as gaming the network.

Two further practical blockers, for completeness: the engine owns the receiver's
serial port (a second reader needs a TCP bridge, as `peppar-bnc-glue`'s
`f9t_broadcaster.py` did on ptpmon), and lab hosts get restarted and re-cabled
constantly, which is the opposite of the rigid, stable mount a reference station
is supposed to be.

**When this WOULD make sense:** a receiver on its own antenna at a genuinely
different site.  Then it is a copy of `instances/onocoy.env` with `#ubx` instead
of `#sbf` and its own credential — the template makes that a new env file and
nothing else.

### Can a relayed stream be passed off as your own station?  (threat model, 2026-08-26)

Bob's question: if a station's observations are readable without authentication,
could someone transcode them and claim the rewards — the way open SMTP relays
were abused early on?  **Structurally, yes**, and the analogy is apt in the way
that matters: both designs infer trust from plausibility rather than proof.

**Why onocoy's published checks do not cover it.**  Their validator network
grades submissions on four signal properties — RMS code, RMS phase, cycle-slip
ratio, sky visibility — plus latency, epoch rate and uptime (confirmed against
`models.ServerStatisticsDaily` in their Public API spec).  Every one of those
except latency is a property of *the signal*, and a relayed stream **is** real
GNSS data, so it reproduces them perfectly.  Those metrics answer "is this
plausible GNSS?", which detects **synthesis**.  Replay is not synthesis; it is
**misattribution**, and no amount of signal-quality scoring separates "I
received these signals" from "someone else received them and handed them to me."
The bytes are identical.

**What could catch it, in rough order of strength:**

1. **Duplicate correlation** — two stations submitting near-identical
   observations is trivially detectable *if both are inside onocoy*.  It fails
   when the source sits outside their network (a public CORS caster, an IGS
   mount, a community caster), because there is nothing to correlate against
   unless onocoy actively ingests those networks to compare.
2. **Position collision** — a replayed stream self-surveys to the *original*
   station's coordinates.  Obvious if that station is also on onocoy; invisible
   if it is not.
3. **Latency** — the one relay-sensitive metric they publish, and it is in the
   API schema (`latency_avg/min/max`).  But it is weak: a relay hop adds tens of
   milliseconds against a <1 s bar.

**The general problem is unsolved.**  Proving *who* observed a signal, rather
than that the signal is genuine, is the "proof of location" problem.  Galileo
OSNMA does not help: it authenticates the *navigation message*, so it proves the
data genuinely came from Galileo — which is exactly as true for the relayer as
for the original observer.  A fix needs cryptography binding an observation to a
specific receiver at a specific place and time, which no civil GNSS service
currently provides.

**Epistemic caveat, and it matters.**  All of the above describes their
*published* defenses.  Not documenting anti-fraud measures is itself sound
practice, so absence from the docs is not evidence of absence — onocoy's
validators may well run duplicate-correlation they simply do not advertise.
This section is "their published model does not address replay", not "onocoy
cannot detect replay."

**What it means for us: nothing bad.**  We use onocoy's grading as an
independent quality check on *our own* stream, and we know our stream is
genuinely ours.  The threat model affects how much one should trust onocoy's
*network-wide* station list, not their assessment of a station you control.

#### The reward formula makes the replay point sharper — and hands us a yardstick

The quality-scale page gives the actual formula:

    signal quality = (RMSCode^2 + RMSPhase^2 + CycleSlipRatio^2 + SkyVisibility^2) / 4
    Quality Scale  = constellation reward * band reward * signal quality reward

**Every factor is a property of the signal.**  Constellations tracked, bands
tracked, and the four signal metrics — a relayed stream reproduces all of them
*exactly*, because it is the same bytes.  There is no term in the reward formula
that a relay could degrade.  Note especially that **latency is not in the
quality score at all**, even though `latency_avg/min/max` are collected and
exposed in the API.  The one relay-sensitive quantity they measure does not
appear to be priced.

That also makes the incentive perverse in a specific way: rewards scale with
quality and with spatial sparsity, so the most profitable stream to relay is a
*high-quality station in a poorly-covered region* — exactly where genuine
contribution is most valuable.

**The economics are still bounded**, which is the honest counterweight: the
0.99-score thresholds are geodetic-grade (see below), and stations that clean
generally sit on authenticated casters (IGS / EUREF / BKG all require
registration).  The casters that are genuinely open tend to carry hobbyist
stations whose scores are lower, so the attack pays least where it is easiest.

#### Their thresholds in our units

Worth recording because it connects onocoy's grading to our own error budget:

| onocoy threshold (0.99 score) | in time |
|---|---|
| RMS phase <= 0.0014 m | **4.67 ps** |
| RMS code <= 0.14 m | **467 ps** |
| cycle slips < 1 in 2300 | — |
| sky visibility > 99% | — |

For comparison, `two-site-sync-budget.md` puts TD-CP per-epoch precision at
~5-10 ps and the per-clock budget at <=350 ps RMS.  So onocoy's top-tier
carrier-phase bar sits right at our own carrier-phase measurement precision.

Once the station is validated, `phase_rms_avg` from `onocoy-status.py` is
therefore not just a grade — it is an **independently computed carrier-phase
noise figure for the UFO1/SXT-D chain, in units we can convert straight to
picoseconds and compare against the timing budget.**  That is a better deal than
"third-party consistency check" implied.

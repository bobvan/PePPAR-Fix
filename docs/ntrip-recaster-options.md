# NTRIP Re-caster Options — Relay, Relabel, Aggregate

*2026-07-11 — survey of open-source NTRIP "re-caster" software: tools that
pull corrections from upstream caster(s) and re-serve them downstream, either
passing them through, relabeling mountpoints, or combining multiple sources.
Written so we don't re-research it when we wire up local correction
distribution for the lab.*

## TL;DR

A "re-caster" is just an NTRIP **client** (pulls upstream) bolted to an NTRIP
**caster** (serves downstream). Pick the tool by *what* you need:

| Need | Tool | Notes |
|---|---|---|
| Pass-through / **relabel** one stream, fan out to many local clients | **`str2str`** (RTKLIB, rtklibexplorer fork) | Lightweight; its own `ntripc` caster output. Decouples lab hosts from the upstream caster. |
| **Aggregate / combine** multiple SSR streams into one | **BNC** (BKG Ntrip Client), `Combi.bnc` mode | GPL. Combines orbit/clock; **not** AR-safe across ACs (see caveat). |
| Full managed **caster** with many mountpoints | **BKG Professional NtripCaster** (Icecast-based, GPL) | Dockerized packaging available (goblimey/ntripcaster). |
| Commercial comparison point | **SNIP** (SubCarrier Systems) | Convenient relay/aggregate/relabel, free tier — **proprietary, not OSS.** |

**For our lab:** use `str2str` as a local caster to pull GA's SSR once and
re-serve it (relabeled) to all lab hosts — one upstream pull, LAN fan-out.
Combine multiple ACs with BNC only for float-PPP robustness/gap-fill, never
for the AR path. See [correction-sources.md](correction-sources.md) and
[ac-datum-mixing.md](ac-datum-mixing.md).

## The options in detail

### 1. `str2str` (RTKLIB) — pass-through / relabel

The canonical relay tool. Input = NTRIP client; output = either an
NTRIP-server push to another caster, or its **own local caster** via the
`ntripc` output type, where you choose the mountpoint name. It multiplexes one
upstream pull to many downstream clients.

- **Gotcha:** not every RTKLIB build includes the caster (`ntripc`)
  functionality. Use the **rtklibexplorer (demo5) fork** or a recent upstream
  build — *not* a random distro package.
- `strsvr` is the same thing with a (Windows-only) GUI.
- Best fit for "pull GA's `SSRA00BKG0` once, re-serve on the LAN under our own
  mountpoint name, and stop every host hammering the upstream caster."

### 2. BNC (BKG Ntrip Client) — aggregate / combine SSR

GPL, multi-stream. The `Combi.bnc` configuration pulls several
broadcast-correction (SSR) streams plus a broadcast-ephemeris stream, produces
a **combined** correction product, re-encodes it as RTCM3 SSR, and uploads it
to a caster. Genuine multi-AC combination, not just relaying. (We already have
[bnc-log-reference.md](bnc-log-reference.md) for BNC's log format.)

> **AR-datum caveat — the load-bearing point.** BNC's combination aligns
> **orbit/clock** across analysis centers, which is fine for float PPP,
> redundancy, and gap-fill. But **phase biases from different ACs live in
> different datums** — a combined stream is **not** AR-consistent. You cannot
> PPP-AR off a naïvely mixed multi-AC stream. Aggregate for float/robustness;
> keep a **single AC** for the ambiguity-resolution path. Full reasoning in
> [ac-datum-mixing.md](ac-datum-mixing.md).

### 3. BKG Professional NtripCaster — full caster

Open source (GPL), built on Icecast, C, Linux. Explicitly usable to distribute
SSR streams with managed mountpoints. Heavier than `str2str`'s caster but a
"real" caster if we want mountpoint management, auth, and a source table. A
**Dockerized packaging** (goblimey/ntripcaster) avoids the build. Lives in
BKG's open-source RTCM-NTRIP hub (software.rtcm-ntrip.org), alongside the
reference NtripServer/NtripClient sources.

## Relationship to our own caster

We already run `ntrip_caster.py` — but that serves **raw observations** (RTCM
MSM4) + station reference position (RTCM 1005), i.e. the *base-station /
upstream* role, with broadcast ephemeris planned
([caster-ephemeris.md](caster-ephemeris.md)). A **re-caster is the opposite
direction**: it consumes *corrections* from upstream and re-serves them
*downstream*. The two are complementary, not the same box.

A local re-caster also dovetails with the peer-bootstrap direction: the same
LAN-local caster can be the thing advertised for discovery
([ntrip-mdns-discovery.md](ntrip-mdns-discovery.md)) and used to bootstrap
peers ([peer-bootstrap-sketch.md](peer-bootstrap-sketch.md)).

## Recommendation for PePPAR-Fix

1. **Local SSR relay/relabel:** `str2str` (rtklibexplorer) as a local `ntripc`
   caster — pull GA's SSR once, re-serve relabeled to the lab. Low footprint,
   decouples hosts from the upstream, reduces upstream connection count.
2. **Multi-AC combination (float only):** BNC `Combi` → upload into that caster
   — for robustness/gap-fill, honoring the AR-datum caveat.
3. **If we outgrow `str2str`'s caster:** drop in the BKG Icecast caster
   (Docker) behind it for managed mountpoints.

## Sources

- [rtklibexplorer RTKLIB fork (str2str + NTRIP caster)](https://github.com/rtklibexplorer/RTKLIB)
- [BKG Ntrip Client (BNC) manual — Combi combination](https://software.rtcm-ntrip.org/export/HEAD/ntrip/trunk/BNC/src/bnchelp.html)
- [BKG Professional NtripCaster (Icecast-based, GPL)](https://igs.bkg.bund.de/ntrip/bkgcaster)
- [goblimey/ntripcaster — BKG caster in Docker](https://github.com/goblimey/ntripcaster)
- [software.rtcm-ntrip.org — BKG open-source NTRIP hub](https://software.rtcm-ntrip.org/)

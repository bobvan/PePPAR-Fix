# RTCM 3 MSM observation ingest — the geodetic-receiver bridge

*Scope + spike plan for ingesting RTCM 3 MSM observations into the engine's
observation model — the universal bridge to external-clock geodetic
receivers (Leica GRX1200 GG Pro, Trimble NetR9, Septentrio PolaRx, …).
See `docs/gpsdo-noise-and-external-clock.md` §3e for why this is the path.*

## Why

Our engine ingests **UBX-RAWX** (u-blox binary). Every geodetic/timing
receiver instead speaks a proprietary raw format, but they all can emit —
and NTRIP casters all carry — **RTCM 3 MSM**, the standard multi-GNSS raw
observation format (code + carrier phase + Doppler + CNR per SV per
signal). An MSM→obs-model adapter is therefore the *one* piece that
unblocks every non-u-blox receiver, including the external-clock path where
the DO clocks the receiver and the receiver reports `dt_DO` via its raw
obs.

## What we have vs what's new

| Piece | Status |
|---|---|
| NTRIP client (`scripts/ntrip_client.py`, `NtripStream`) | **Exists** — connects, auths (Basic), yields *decoded* RTCM3 messages via pyrtcm |
| pyrtcm MSM decode (1074–1127) | **Exists** (pyrtcm 1.1.12) |
| RTCM SSR decode + signal-code maps | **Exists** (`ssr_corrections.py`; BDS/GPS/GAL sig-id maps from the AR work) |
| MSM4 *encode* (our caster) | **Exists** (`rtcm_encoder.py`, `ntrip_caster.py`) |
| **MSM obs → engine observation model** | **NEW — this work** |
| GLONASS FDMA / inter-frequency bias handling | **NEW — deferred** (see below) |

## The observation-model target

The engine's obs model (fed today by UBX-RAWX in `realtime_ppp.py`) is
per-SV-per-signal: `{pseudorange_m, carrier_phase_cyc, doppler_hz, cno_dbhz,
lli/lock}`, keyed by a RINEX-style signal code. MSM carries exactly this.
The adapter's job:

1. **Decode** MSM4 (1074/1084/…) or MSM7 (1077/1087/…) via pyrtcm. MSM7 is
   preferred — finer resolution + phase-range-rate; MSM4 is fine for the
   spike.
2. **Map MSM signal IDs → our signal-code keys.** RTCM's per-constellation
   MSM signal-ID tables (DF395 mask) → RINEX 3 codes (e.g. GPS sig 2 →
   `1C` L1 C/A, sig 15/16 → `2X` L2C, sig 22 → `5X` L5). **Reuse the
   existing sig-id map infrastructure** built for SSR/BDS.
3. **Reconstruct observables.** MSM gives rough range (ms) + fine
   pseudorange + fine phase-range + phase-range-rate; assemble
   `pseudorange_m` and `carrier_phase_cyc = phaserange / λ`. Handle the
   MSM half-cycle ambiguity flag and lock-time → LLI.
4. **Emit** into the same obs structure `solve_ppp` consumes — downstream
   (GF/MW/IF, cycle-slip, PPP filter) is format-agnostic once obs are in
   the model.

## Scope discipline

- **GPS first.** GPS L1 C/A + L2 (and L5 where present) is CDMA and drops
  straight into the existing model. Validates the whole adapter with zero
  FDMA detour.
- **GLONASS deferred.** GLONASS is **FDMA** — per-satellite frequency
  channels ⇒ per-SV wavelengths and **inter-frequency biases (IFB)** in
  code *and* phase. The engine (GPS/GAL/BDS today) has no GLONASS path;
  adding it is genuine new work (our first FDMA support) and should be a
  deliberate follow-on, not smuggled into the ingest spike. The Leica GG
  Pro giving GPS+GLONASS is a bonus, not a requirement — **L1/L2 GPS alone
  is enough to build and validate the ingest.**
- **Obs-only, not the external-clock architecture yet.** Until the DO is
  wired to the receiver's reference input, ingesting a receiver's obs
  estimates its *own-clock* `dt`, not `dt_DO`. Correct sequencing: build
  the ingest now (remotely), validate the external-clock benefit later
  when the DO is wired.

## Validation — compare our PPP to a published position

The clean test (no lab hardware): point the adapter at a **CORS station's
live RTCM 3 MSM stream**, run our PPP/LS solve on the decoded obs, and
compare the solved position to the station's **published coordinates**.
End-to-end proof of decode → obs-model → PPP.

- **Source.** London hosts reach casters only on **443/TLS**
  (`ntrip.data.gnss.ga.gov.au`, port 443 — 2101 is firewalled). That GA
  caster carries AUSCORS/IGS **obs** mountpoints (MSM) with published
  positions, reachable with the existing account. An **IGS station** gives
  an authoritative ITRF published position to grade against.
- **The spike** (`scripts/spike_rtcm_msm_ingest.py`): `NtripStream` →
  filter MSM (1074–1127) → decode+map obs → LS position solve (broadcast
  eph) → report residual vs the mountpoint's published ECEF. GPS-only to
  start.
- **Grade.** Code-only single-point LS should land within a few metres of
  the published ARP; that confirms the obs are decoded and assembled
  correctly. (cm-class PPP is a later refinement — the spike's job is to
  prove the *ingest*, not to out-survey IGS.)

**Live-proven 2026-07-05** (PiPuss → `ntrip.data.gnss.ga.gov.au:443`,
mount `ALIC00AUS0` = Alice Springs IGS, the account already has obs access):
25 s yielded **51 GPS-MSM messages, 11 PRNs decoded**, and message **1006
gave the station ARP (−4052052.74, 4212835.99, −2545104.59) m** = ALIC's
published IGS position. Each AUSCORS mount carries **1074/1077 (GPS MSM) +
1006 (ARP) + 1019 (GPS eph)** on one stream — obs, truth, and ephemeris
together, so the LS-solve-and-compare needs no extra source.

**Full loop closed 2026-07-05** — the spike decodes MSM (MSM4/5 *and* MSM7
extended fields), ingests 1019 eph, forms iono-free pseudoranges, and runs
`ls_init`. Two stations, two configs:

- **ALIC** (remote IGS, obs+eph on one stream): LS fix **3.04 m** from the
  published IGS ARP (E +0.19, N +1.06, U +2.84) — textbook code-only SPP.
- **NAPERVILLE** (local Wheaton CORS on the ISTHA Leica caster, run from gt):
  the base caster carries **no 1019**, so the spike's **`--eph-mount`
  two-stream mode** fed eph from **BCEP00BKG0** (full constellation, 97 SVs)
  while obs came from NAPERVILLE. LS fix **12.14 m 3D** — but only **~2.3 m
  horizontal** (E −0.39, N +2.25); the ~12 m is vertical (code-only SPP with
  no troposphere model + a likely NAD83↔ITRF datum offset on the caster's
  published ARP). Good horizontal ⇒ the decode is correct.

The two-stream `--eph-mount` mode is **the pattern the real Leica GRX1200
ingest needs** — receiver gives obs, eph comes from a BCEP nav stream.

**Why the spike's vertical is loose (and why that's not a requirement).** The
spike grades with `ls_init` — the engine's *coarse, troposphere-free
single-point bootstrap* — purely to prove the decode is correct with the
lightest possible harness. The ~12 m NAPERVILLE vertical is an `ls_init`
property, **not** an RTCM-ingest one. The actual integration (build step 3)
feeds MSM obs into the **same observation model** `PPPFilter` already
consumes from UBX-RAWX, so the **existing ZTD/troposphere model, AR, and
clock estimation apply unchanged — no new tropo code**. That format-parity
is the entire point: the adapter's only job is to emit the engine's obs
structure; everything downstream is source-agnostic.

## Build order

1. **Spike** — `NtripStream`→MSM-decode→obs→LS-solve→compare-to-published
   (GPS L1/L2, one CORS). Proves the decode + obs assembly. ← *start here*
2. **Adapter module** — promote the spike's decode+map into a clean
   `rtcm_msm_obs.py` that emits the engine's obs structure; unit-test the
   sig-id mapping + observable reconstruction against captured MSM.
3. **Engine seam** — an obs *source* option so the engine can take MSM
   (from NTRIP or a receiver's TCP stream) in place of UBX-RAWX.
4. **GLONASS FDMA/IFB** — deliberate follow-on.
5. **External-clock validation** — once the DO is wired to a receiver's
   reference input; `dt` becomes `dt_DO`.

## Related
- `docs/gpsdo-noise-and-external-clock.md` — why external-clock receivers,
  and the interface/format survey (§3e) this implements.
- `scripts/ntrip_client.py` (`NtripStream`), `scripts/rtcm_encoder.py`
  (MSM4 encode — the inverse), `scripts/realtime_ppp.py` (the UBX-RAWX obs
  path this parallels).

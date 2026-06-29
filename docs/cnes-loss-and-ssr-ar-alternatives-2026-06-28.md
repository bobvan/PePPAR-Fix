# CNES SSR loss → SSR/HAS AR alternatives (2026-06-28)

> **UPDATE 2026-06-29 — it's a temporary BKG outage, not a move.** CNES replied
> directly (Thierry, CNES PPP Team): *"BKG IGS-IP caster (products.igs-ip.net) is
> the right place for CNES real-time SSR streams download. Unfortunately there is
> for the moment an outage on this flow. CNES PPP Team is doing its best to restore
> them. Thank you for your patience."* So **`products.igs-ip.net` stays the
> authoritative source and the CNES streams will return there — prescription is
> patience**, not migration. The "CNES moved to its own REGINA caster, re-register"
> reading in §1 below was an over-conclusion (REGINA exists in parallel and does
> serve SSRA00CNE0 behind a separate login, but it is an *optional interim*, not the
> fix). Sections below are kept as the contemporaneous investigation; read them
> through this correction. The SSR analysis-center survey and HAS findings (§2–§5)
> are unaffected and remain useful contingency.

**Trigger:** the rebuilt MadHat surfaced that CNES `SSRA00CNE0` had vanished from
the BKG IGS-IP caster (`products.igs-ip.net`), leaving every CNES-configured host
with 0 orbit/0 clock when wrapper-launched without a `--ssr-mount` override. CNES
was the only analysis center whose Galileo phase biases empirically fixed
ambiguities on our u-blox L5 fleet (see `l5i-l5q-phase-bias-empirical.md`,
`bds-b2a-phase-bias-survey-2026-05-09.md`).

Two deep-research passes (run IDs wv26495le = CNES/SSR, wx262v7vt = Galileo HAS)
+ direct empirical probing of the casters. Findings below are tagged with
confidence; **bold = verified by me directly on the wire**, not just web-sourced.

---

## 1. What happened to CNES — it moved casters, not decommissioned

CNES was **removed only from the third-party BKG caster** (`products.igs-ip.net`).
The product still lives on **CNES's own REGINA caster**. CNES's own page
(`regina.cnes.fr/en/download-corrections`) confirms the stream names:
`SSRA00CNE0` (= old CLK93, antenna-phase-center) and `SSRC00CNE0` (= old CLK90,
center-of-mass).

**Verified on the wire (2026-06-29):**
- `http://regina-ip.cnes.fr:2101/SSRA00CNE0` → **HTTP 401 Unauthorized**
  (`WWW-Authenticate: Basic realm="/SSRA00CNE0"`), **not 404** → the mount is
  **LIVE but UNLISTED** (the public REGINA sourcetable shows 207 station-obs
  mounts, zero SSR).
- Our existing **IGS-IP credentials (bobvan) are REJECTED** at REGINA (still 401)
  → REGINA requires a **separate CNES registration**.

**Recovery path (answers "do I re-apply?": YES):** register for CNES REGINA /
PPP-Wizard access, then point `ntrip-cnes.conf` at `regina-ip.cnes.fr:2101`
mount `SSRA00CNE0` with the new credentials. CNES is a **self-consistent single-AC
set** (orbit + clock + code + phase bias in one mountpoint, zero-difference IAR /
Laurichesse method) — the cleanest AR source if access is restored. *Caveat:* the
research could NOT independently confirm CNES's Galileo phase bias covers the exact
E5a component our F9T tracks (it refuted the unverified version of that claim) —
but it worked for us empirically before, so high likelihood it works again.

---

## 2. AR without CNES — are we limited to WHU? No (at message-type level)

Live `products.igs-ip.net` sourcetable query (2026-06-28): **many single ACs
publish Galileo phase bias + orbit/clock in one mount** — not WHU-only:

| AC | Self-contained AR mount(s) | Phase-bias constellations | O/C in same mount |
|---|---|---|---|
| BKG | SSRA00BKG0/BKG1, SSRC00BKG0/1 | GPS, GAL, GLO | yes |
| GFZ | SSRA00GFZ0/1, SSRC00GFZ0/1 | GPS, GAL, GLO | yes |
| CAS | SSRA01CAS1, SSRC01CAS1 | GPS, GAL, BDS | yes |
| SHAO | SSRA01SHA1, SSRC01SHA1 | GPS, GAL, BDS, GLO | yes |
| CHCNAV | SSRA00CHC1, SSRC00CHC1 | GPS, GAL, BDS | yes |
| WHU | **OSBC00WHU1 (bias-only)** + SSRC00WHU0 (orbit/clock) | GPS, GAL, BDS | no — needs the pair |

(A newer `SSRC*` / `SSRA01*` family has appeared. **Zero CNES mounts** remain on
this caster.)

**WHU = strongest documented free alternative** (research, high conf): observable-
specific code+phase OSBs on `OSBC00WHU1`, orbit/clock on `SSRC00WHU0`; reported
narrow-lane fix rates ~82% GPS / 85% GAL / 76% BDS-3; also served on a **backup
caster** (Wuhan `58.49.94.212`). Single-AC pair → datum-consistent for AR.

### The real gate: signal-code (E5a I-vs-Q) match — UNVERIFIED for everyone but CNES
The table is **message-type** presence (RTCM 1267 / IGS-SSR 4076_066), not
**signal-code** coverage. Two hard facts:
- Our hosts run BKG `SSRA00BKG0` and get **0 phase bias in practice** despite it
  advertising 1267 → advertisement ≠ usable bias for a u-blox client (likely
  signal-code / observable mismatch — same class of issue as our CNES L5 wall).
- The harness **refuted** (2-3 / 0-3 votes) the unverified claims that CNES/WHU
  Galileo phase biases cover the specific E5a component u-blox tracks.

⇒ **Which AC actually fixes on our F9T/X20 E5a is an empirical lab question**, not
answerable from sourcetables/papers. Test candidates head-to-head (WHU first).

---

## 3. Galileo HAS — NOT an AR solution today; a float fallback

| Property | Status (verified vs live GSC page 2026-06-28) |
|---|---|
| Operational? | Yes, free, since 2023-01-24 (Initial Service / SL1), 24/7, ~decimetre |
| **Phase biases?** | **NO** — SL1 provides orbit/clock/**code** bias only → **float PPP, not native AR** |
| GPS L5? | **NOT corrected** (deferred) — explains our X20P broadcast-HAS L1+L5 divergence |
| Signals (operational) | GAL E1/E5a/E5b/E6 + GPS L1/L2C |
| HAS-based AR | only via user-side WL/NL **FCB self-estimation** from HAS orbit/clock (research pipeline, not drop-in); ~13–29% fixed-vs-float improvement |
| vs free SSR | WHU **outperforms** HAS in matched float studies (HAS is the noisier/slower one — consistent with our prior 20–30× finding) |
| Full Service (phase biases + GPS L5) | roadmap: FOC ~Q4 2026, declaration Q1–Q2 2027 |

**Delivery modes vs our fleet:**
- **IDD (internet, all hosts):** GSC NTRIP caster, open CSSR-like format,
  BNC-consumable. **Free but gated** — registration + EUSPA authorization,
  **capacity-limited, per-organization account caps** (constrains fanning out to
  every host). Float only.
- **E6-B broadcast (X20P only):** F9T/F9P **cannot** (no E6 tracking) → IDD only.
  Whether **ZED-X20P natively decodes/outputs E6-B HAS** is an **open question** —
  not confirmed in u-blox docs by the research; verify against X20P integration
  docs. Float only, noisier, no GPS L5.

⇒ **HAS today = CNES-independent FLOAT fallback** (esp. broadcast HAS for an
offline X20P), **not** an AR replacement. Re-evaluate for AR when Full Service
lands (~2027), or build an FCB-estimation pipeline if HAS-AR becomes a priority.

---

## 4. Datum consistency (confirmed)
PPP-AR requires orbit/clock **and** phase bias from a **single self-consistent AC**
— mixing ACs breaks integer recovery (peer-reviewed). Our old MadHat config
(CNES orbit/clock + WHU phase bias) was a **two-AC mix** = wrong for AR. Valid
single-AC sets: CNES (one mount), WHU (orbit/clock + bias pair, same AC), or any
BKG/GFZ/CAS/SHAO/CHC self-contained mount **if** its signal-codes match.

---

## 5. Recommendation for the fleet

1. **Restore AR the proven way → re-register with CNES REGINA.** Apply for REGINA/
   PPP-Wizard access; repoint `ntrip-cnes.conf` to `regina-ip.cnes.fr:2101`
   (`SSRA00CNE0`) with new creds; re-enable the per-host `ssr_ntrip_conf`. CNES is
   the only source empirically proven to fix on our signals.
2. **In parallel, lab-test the all-WHU single-AC pair** (`SSRC00WHU0` + `OSBC00WHU1`)
   as the CNES-independent AR backup — grade by whether it actually fixes E5a on the
   F9T/X20 (the unverified signal-code risk). Backup caster: Wuhan `58.49.94.212`.
3. **Treat HAS as a float fallback, not AR.** Optionally register HAS IDD as a
   CNES-independent float source (mind the per-org account cap); keep broadcast-HAS
   as the offline X20P option. Revisit HAS-AR at Full Service (~2027).
4. **Float baseline already in place:** all hosts now default to BKG `SSRA00BKG0`
   (config cleanup committed 50e835e) → float PPP works fleet-wide without overrides
   while AR access is sorted.

## Sources (primary, verified)
- CNES: regina.cnes.fr/en/{ppp-wizard-project,download-corrections,calculation-transmission}; ppp-wizard.net; **on-wire 401 probe of regina-ip.cnes.fr:2101/SSRA00CNE0**
- HAS: gsc-europa.eu Galileo HAS service pages + SDD v1.0 + Info Note; euspa.europa.eu/galileo-has; GPS Solutions 10.1007/s10291-025-01852-6; Naciri et al. PMC9931823; Adv. Space Res. 2025 (HAS vs WHU RTS)
- WHU: IGSMAIL-8424; GPS Solutions 10.1007/s10291-023-01610-6
- Live `products.igs-ip.net` + `regina-ip.cnes.fr` NTRIP sourcetables (queried 2026-06-28/29)

Full machine-readable research outputs: workflow runs wv26495le, wx262v7vt.

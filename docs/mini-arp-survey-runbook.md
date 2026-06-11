# Mini ARP Survey Runbook — capture on the Mini, analyze on gt

**Purpose.** Survey the Antenna Reference Point (ARP) of a Timebeat OTC
Mini PT (F9T-based) to within a few cm 3D, by capturing 24 h of raw GNSS
on the Mini and doing the PPP analysis back on **gt**. Written to transfer
the method from the **ptBoat dry run (2026-06-11)** to the **London Mini**,
which a *different agent* will operate (one agent can't easily hold access
to both Minis at once).

**Key architectural choice — analyze on gt, not on the Mini.** The Mini's
only job is to *log raw observations*. All PPP processing runs on gt (or any
capable box). This avoids installing PRIDE-PPP-AR + gfortran on the London
Mini and keeps the Mini's role to one low-risk task: capture UBX
RAWX/SFRBX and ship the file back.

---

## Validation status (be honest about what's proven)

- **Capture method: VALIDATED.** ptBoat ran the exact logger + detached
  wrapper below for 24 h on 2026-06-11, including auto-restarting after a
  power outage. The F9T-TIM-firmware NAK handling is in the committed logger.
- **Analysis pipeline: DESIGNED, first real run pending** ptBoat completion
  (~2026-06-12). Update Phase 3 with any lessons once ptBoat's data is
  actually processed.

---

## Prerequisite — get the patched logger

`scripts/f9p_rawx_log.py` must be the version with `--iface` auto-detect +
NAK-tolerant CFG cleanup (**commit `e96bc99`, branch `delta/main`**). F9T TIM
firmware NAKs the RTK-only CFG-MSGOUT keys (`NAV-RELPOSNED` / `HPPOSECEF` /
`HPPOSLLH`); the old USB-only batch VALSET fails the whole config on a
UART-wired F9T. **Make sure that commit is on the London Mini's checkout**
(merge `delta/main` → `main`, or `git fetch && git checkout delta/main`).

---

## Phase 1 — Capture (on the London Mini)

**Verify first — do not assume; these differ across Minis:**
- **Device + baud.** ptBoat's F9T is `/dev/ttyAMA0 @ 115200`. The *other* OTC
  (otcBob1) is `460800`. Confirm the London Mini's from its timebeat config or
  a quick probe **before** launching — wrong baud = garbage/no data.
- **Antenna model + its NGS antex.** Record it now; you need the exact antex
  block in Phase 3 or PRIDE biases the ARP by ~1 m (see gotchas).
- **Power.** Assume unprotected (ptBoat lost power twice). Build in outage
  auto-restart + frequent gt backup.

**Stop timebeat (it owns the F9T):** `sudo systemctl stop timebeat`

**Detached wrapper** (mirrors ptBoat's `run_rawx_capture.sh`). The trap
restores timebeat on clean exit; on a hard power cut, boot brings timebeat
back since it's `enabled`:

```bash
#!/bin/bash
trap "sudo systemctl start timebeat.service" EXIT
OUT=~/peppar-survey-data/raw/london-24h-<DATE>
mkdir -p "$OUT"; cd ~/peppar-fix            # the Mini's git checkout
python3 scripts/f9p_rawx_log.py \
  --r1-port /dev/ttyAMA0 --r1-tag F9T \
  --baud <VERIFIED_BAUD> --duration 86400 \
  --out-dir "$OUT" --report-every 1800 >> "$OUT/capture.log" 2>&1
echo "[$(date -u +%FT%TZ)] logger exited rc=$?" >> "$OUT/capture.log"
```

**Launch detached** so it survives SSH/agent disconnect (this is what kept
ptBoat logging through reconnects):
```bash
setsid /home/bob/run_rawx_capture.sh </dev/null >/dev/null 2>&1 &
```

The logger auto-detects **UART1** from a `ttyAMA*` port (the iface patch),
emits RAWX+SFRBX+NAV-PVT at 1 Hz, and rotates hourly `.ubx` files
(~250 MB for 24 h). Sanity-check the first status line shows `fix=3` and a
healthy `nSV`.

**Resilience:** rsync the dir to gt ~hourly. If the logger dies before 24 h
and the host rebooted, relaunch the wrapper into a fresh dir (boot already
restarts timebeat).

---

## Phase 2 — Bring the data to gt

Raw capture files are *data, not repo code*, so file copy is the right tool
(the no-scp-repo-code rule does not apply to capture artifacts).

**First, strip timing-only messages** — important when the path to gt is slow
or multi-hop. A time-appliance F9T emits NAV-SIG / NAV2-SIG / TIM-TP /
MON-SPAN / TIM-SVIN / NAV2-POSECEF alongside the RAWX/SFRBX/PVT we need (≈ half
the bytes). `scripts/ubx_filter.py` drops them without touching the receiver or
timebeat (validated 2026-06-11: ~56% smaller, RAWX/SFRBX/PVT byte-identical,
convbin RINEX unchanged):

```bash
python3 scripts/ubx_filter.py raw.ubx raw.min.ubx     # keeps rawx,sfrbx,pvt
```

Then copy the filtered file(s) to gt under `~/gt/...` (RAIDZ-3 + offsite) by
whatever path works — direct rsync if the host can reach gt, otherwise hop it:

```bash
rsync -a london-mini:peppar-survey-data/raw/london-24h-<DATE>/ \
         ~/gt/peppar-survey-data/raw/london-24h-<DATE>/
```

Nothing else from the Mini is needed for analysis.

---

## Phase 3 — Analysis on gt (3-way PPP consensus)

1. **Concatenate** the hourly `.ubx` files, then **convbin (RTKLIB) → RINEX 3**:
   `convbin -r ubx -o out.obs -n out.nav <concatenated>.ubx`
2. **Three independent static-PPP solutions** (independent so agreement is
   meaningful):
   - **RTKLIB:** `rnx2rtkp -p 7 -m S ...` (PPP static).
   - **PRIDE-PPP-AR:** `pdp3 ...` (cm-class, ambiguity-resolved).
     **Antex gotcha:** inject the **London antenna's** NGS calibration into
     PRIDE's antex DB (`scripts/inject_lab_antennas.sh` + `support/antex/`).
     Without the exact `<ANT> NONE` block, PRIDE silently falls back to a
     different/zero calibration and biases the ARP by ~1 m vs OPUS.
   - **CSRS-PPP:** upload the RINEX to NRCan's online service — **no local
     install**, a clean third opinion.
3. **Consensus.** If RTKLIB / PRIDE / CSRS agree **≤ 3 cm 3D**, take the mean as
   the ARP and write `state/positions/<uid>.survey.toml` (survey-class — only
   ever written by a survey backend, never by the engine). If they disagree,
   investigate (antex mismatch, multipath, short/!static arc) before trusting
   any single solution.

---

## Gotchas distilled from ptBoat

- **F9T TIM firmware NAKs RTK-only CFG keys** → use the patched logger (above).
- **Verify baud per Mini** — 115200 (ptBoat) vs 460800 (otcBob1).
- **Unprotected power** → outage auto-restart + hourly gt backup.
- **timebeat must be stopped** for capture and is auto-restored on exit/boot.
- **Don't scp repo code to the Mini** — `git pull`. Only data and
  `ntrip.conf`-class credentials move by scp/rsync.
- **Antenna antex must match the London antenna exactly** for PRIDE — this is
  the single most likely silent ~1 m bias.
- **The Mini never needs PRIDE/gfortran** — that's the whole point of
  analyzing on gt.

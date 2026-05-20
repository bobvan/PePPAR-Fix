# peppar-survey install (optional component)

`peppar-survey` is the optional companion to `peppar-fix` that writes
`state/positions/<uid>.survey.toml` from external authoritative
observations (PRIDE-PPP-AR, OPUS, CORS-RTK, or RTKLIB).  Installations
that don't need survey-grade ARP seeding can skip it entirely — the
engine's own AntPosEst warm-start via `.ppp.toml` still works.

Requirements live in this file rather than `pyproject.toml`'s
`[project.optional-dependencies]` because the backends rely on native
binaries (PRIDE-PPP-AR is Fortran, RTKLIB is C, OPUS is a web POST)
that aren't pip-installable.  Python deps for peppar-survey itself
are exactly the same as peppar-fix — no additional Python wheels.

## Per-backend requirements

| Backend | Native deps | Notes |
|---|---|---|
| `--pride` | gfortran, gcc, GNU make, then PRIDE-PPP-AR built from source.  Internet (one-off, to fetch satellite products for each daily run). | Strictest precision (~5 mm horizontal for a 24h F9T capture); recommended baseline. |
| `--opus` | (Python `urllib` only) but requires the NGS submission email account in operator config. | Heavyweight (10-30 min server-side turnaround per submission); 24h minimum capture; rate-limited per email. |
| `--cors` | (Python only) requires NTRIP credentials for a nearby CORS site. | Minutes-class accuracy; useful for sanity checks rather than archival surveys. |
| `--rtklib` | RTKLIB built from source. | No external dependency once installed; precision varies by config. |

Only `--pride` is implemented today.  This doc covers that backend.

## PRIDE-PPP-AR install (per lab host)

The pdp3 binary needs to live at `~/.PRIDE_PPPAR_BIN/pdp3` (the
PRIDE-PPP-AR upstream installer's default; respected by
`peppar_survey_pride.py` via `PEPPAR_PDP3_BIN` env override if you
need to relocate).

```bash
# 1. Build + runtime deps (Debian / Raspberry Pi OS / Ubuntu).
# bc is needed at pdp3 runtime — pdp3.sh shells out for arithmetic.
sudo apt-get update
sudo apt-get install -y gfortran gcc make bc

# 2. Fetch + build PRIDE-PPP-AR.  ~/PRIDE-PPPAR.install is a build
#    tree only — the installed binaries land in ~/.PRIDE_PPPAR_BIN.
git clone --depth 1 https://github.com/PrideLab/PRIDE-PPPAR.git \
    ~/PRIDE-PPPAR.install
cd ~/PRIDE-PPPAR.install
# install.sh ships without the executable bit on git checkouts; invoke
# via `bash install.sh` rather than `./install.sh`.
# install.sh asks "Run tests? [Y/N]" — decline (N) for the first
# install; tests need internet for product downloads and a full hour.
echo N | bash install.sh

# 3. Verify.
~/.PRIDE_PPPAR_BIN/pdp3 -V
```

Build time on a Raspberry Pi 5 (Cortex-A76 @ 2.4 GHz, ARM64):
~3-6 min for the Fortran sources.  Much faster on x86 lab hosts.

Disk: ~80 MB for the source tree, ~30 MB for installed binaries.

## ANTEX file — required injection step

PRIDE ships the IGS antex catalog at
`~/PRIDE-PPPAR.install/table/igs20_<NNNN>.atx` where `<NNNN>` is the
GPS week the calibration was issued.  At time of writing the bundled
file is `igs20_2415.atx`.

**The IGS catalog does NOT include receiver-specific NGS antennas
like `SFESPK6618H NONE`** (the CHOKE1 + UFO1 antenna the lab uses).
Without the matching antex block, pdp3 silently falls back to a
different (or zero) calibration and the resulting ARP biases by ~1 m
vs OPUS-Static (validated empirically on MadHat 2026-05-18: PRIDE
without SFESPK6618H showed 1.46 m offset from antPos.json[choke1]).

The `install_peppar_survey.sh` script handles this automatically by
calling `scripts/inject_lab_antennas.sh` after the PRIDE build.  The
injector is idempotent — already-injected blocks are skipped, so
re-running it on subsequent PRIDE updates is safe.

When adding a new antenna to the lab:

1. Fetch its NGS antex from `https://geodesy.noaa.gov/ANTCAL/` (or
   extract from an `ngsXX.atx` composite).  Save under
   `support/antex/<TYPE>_<RADOME>.atx` in the peppar-fix repo, one
   antenna block per file.
2. Append the basename (without `.atx`) to the `LAB_ANTENNAS` array
   in `scripts/inject_lab_antennas.sh`.
3. Commit + push + pull on every lab host that has PRIDE installed.
4. Re-run `scripts/inject_lab_antennas.sh` on each PRIDE host (idempotent;
   only the new antenna is appended on hosts already injected with
   prior antennas).

## Running peppar-survey --pride

```bash
# Defaults: history under state/arp/<uid>/history.jsonl,
# survey under state/positions/<uid>.survey.toml,
# pdp3 work dir under $TMPDIR/peppar-survey-pride.
./scripts/peppar_survey.py --pride --rinex-glob 'data/rinex/<host>-*.obs'

# Overrides for one-off lab runs:
./scripts/peppar_survey.py --pride \
    --receiver-uid 333115535856 \
    --rinex-glob 'data/rinex/MadHat-*.obs' \
    --pride-work-dir /tmp/madhat-pride \
    --history-dir /tmp/madhat-pride/arp \
    --positions-dir /tmp/madhat-pride/positions \
    --mount-sn 0
```

Exit codes:
- `0` — success, .survey.toml written
- `1` — no input RINEX
- `2` — every pdp3 attempt failed, or no quality-passing solution
- `3` — pdp3 binary not found at `PEPPAR_PDP3_BIN` /
        `~/.PRIDE_PPPAR_BIN/pdp3`

Quality gates (defaults from `scripts/peppar_fix/arp_history.py`):

- `sig0_m ≤ 10 m` — formal sigma of the daily fit
- `n_obs ≥ 10 000` — defends against truncated captures

Solutions failing either gate are still archived to `history.jsonl`
with `quality_ok=false` (so post-hoc audits can see why σ_arp didn't
move on a given day) but excluded from the running mean.

## Bypassing pdp3's broadcast-nav download with `--brdm-source`

`pdp3.sh`'s IGS-MGEX multi-GNSS broadcast-nav download chain
(gnsswhu / IGN / DLR) intermittently falls through to GPS-only nav
when the daily mixed-nav URLs are unavailable.  Without GAL/BDS
ephemeris, pdp3's `elevation()` returns dist=-1 for every E**/C**
SV → `DEL_BADRANGE` → zero kin solution rows.  Diagnosed in
`prideBadRangeDiagnostic-main` (2026-05-19).

Workaround: stage a pre-fetched multi-GNSS broadcast nav file under
pdp3's canonical name before each run.  `--brdm-source` accepts
either a single file (used for every obs day) or a directory:

```bash
# Single-day file: operator points at a known-good multi-GNSS brdm.
./scripts/peppar_survey.py --pride \
    --rinex-glob 'data/rinex/MadHat-2026139.obs' \
    --brdm-source /home/bob/gt/brdm-archive/brdm1390.26p

# Directory: per-day lookup by brdmDDD0.YYp filename.
./scripts/peppar_survey.py --pride \
    --rinex-glob 'data/rinex/MadHat-*.obs' \
    --brdm-source /home/bob/gt/brdm-archive/
```

When the brdm is staged successfully pdp3 logs
`===> PrepareRinexNav done` without the
`warning: no navigation message for GNSS (E)` line that's the
signature of the GPS-only fall-through.

This is the operator-managed quick path.  The longer-term fix
(persist engine's NTRIP BCEP00BKG0 broadcast eph automatically)
lives under the same bead family — for now, the engine's broadcast
nav has to be sourced out-of-band.

## Scheduling

The default operational cadence is daily, at a time that gives PRIDE
time to fetch the previous day's final satellite products (~6 hours
after UTC midnight).  A systemd timer:

```ini
# /etc/systemd/system/peppar-survey.timer
[Unit]
Description=peppar-survey daily ARP update

[Timer]
OnCalendar=*-*-* 12:00:00 UTC
Persistent=true

[Install]
WantedBy=timers.target
```

```ini
# /etc/systemd/system/peppar-survey.service
[Unit]
Description=peppar-survey daily ARP update
After=network-online.target

[Service]
Type=oneshot
User=bob
WorkingDirectory=/home/bob/peppar-fix
ExecStart=/home/bob/peppar-fix/scripts/peppar_survey.py --pride \
    --rinex-glob 'data/rinex/<host>-*.obs'
```

(The actual `<host>` token + `--receiver-uid` should be hard-coded per
deployment — `peppar-survey` auto-discovers the receiver UID only
when state/receivers/ holds exactly one file.)

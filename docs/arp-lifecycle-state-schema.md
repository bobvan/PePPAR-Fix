# ARP lifecycle state schema (I-013239-main)

*Spec for `state/arp/<receiver_uid>.json`, the file that drives whether
the engine runs in pinned vs re-survey mode.  The single number σ_arp
is the lifecycle parameter; everything else is metadata for
provenance, drift detection, and operator visibility.*

## Design principle

There are no discrete labels (no UNKNOWN / CONVERGED / ESTABLISHED).
A receiver always has *some* knowledge of its antenna position — at
minimum NAV2's hAcc (~1-2 m).  The continuous parameter is **σ_arp**:
the 1-σ uncertainty on the antenna reference point.  Operations gate
on σ_arp:

- σ_arp < `pin_threshold_m` (~0.10 m) → engine runs `FixedPosFilter`
  pinned at `ecef_m`; AntPosEst thread off; NAV2 acts as continuous
  watchdog.
- σ_arp ≥ `pin_threshold_m` → engine runs AntPosEst + RINEX capture;
  daily post-processing accumulates solutions; σ_arp tightens over
  days/weeks.

This makes "is the receiver in survey mode?" a single function of
σ_arp, not a state-machine label that gets out of sync with reality.

## File location

```
state/arp/<receiver_uid>.json
```

Receiver-keyed (matches `state/receivers/<receiver_uid>.json`
convention).  Each receiver has its own ARP file even when two
receivers share an antenna — the truth is the same, but each
receiver's confidence/history is its own.  An operator can copy the
file across receivers on the same antenna to share survey work.

Files are gitignored (per repo policy on antenna coordinates).

## Schema

```json
{
  "schema_version": 1,
  "receiver_uid": 136395244089,
  "antenna_id": "UFO1",
  "ecef_m": [<X>, <Y>, <Z>],
  "lla": {
    "lat": <LAT>,
    "lon": <LON>,
    "alt_m": <ALT>
  },
  "sigma_m": 0.012,
  "feedline_ns": null,
  "antenna_calibration": {
    "model": "SFESPK6618H",
    "radome": "NONE",
    "calibration_source": "ngs",
    "atx_path": "/home/bob/peppar-fix/ngs20.atx"
  },
  "source": {
    "method": "opus_static_6day_mean",
    "n_solutions": 6,
    "first_solution": "2026-04-26",
    "last_solution": "2026-05-02",
    "products": "ngs_static",
    "sigma_per_solution_m": 0.0086,
    "across_day_sigma_m": 0.012
  },
  "watchdog": {
    "k_factor": 5,
    "sustain_epochs": 60,
    "alarm_state": "ok",
    "alarm_first_epoch": null,
    "last_check_ts": "2026-05-04T14:00:00Z",
    "last_displacement_m": 1.43
  },
  "history": [
    {"ts": "2026-04-26", "method": "opus_static", "ecef_m": [...], "sigma_m": 0.014},
    ...
  ],
  "mount_id": 1,
  "updated_ts": "2026-05-03T16:35:00Z"
}
```

### Field definitions

- **`schema_version`** (int): file format version.  Bump when
  field layout changes incompatibly.

- **`receiver_uid`** (int): the F9T `SEC-UNIQID` (or user label
  when no UID is available).  Matches `state/receivers/<uid>.json`.

- **`antenna_id`** (str): operator-friendly antenna name (`"UFO1"`,
  `"Patch3"`, etc).  Provenance only — not used by code.

- **`ecef_m`** (list[float]): the operational antenna reference
  point in ECEF metres.  This is the value `--surveyed-position-from`
  reads when the engine launches in pinned mode.

- **`lla`** (object): LLA derived from `ecef_m`.  Convenience for
  human-readable comparison.  Code uses `ecef_m`.

- **`sigma_m`** (float): the σ_arp in metres.  3D-equivalent
  combined uncertainty across all axes.  This is the lifecycle
  gate.

- **`feedline_ns`** (float | null): optional GNSS PPS propagation
  delay through the antenna cable, in nanoseconds.  Per
  I-013342-main amendment.  Engine applies as a constant offset on
  the F9T PPS time reference so PPS OUT aligns to the antenna,
  not to the receiver jack.  null → no correction (default for
  cold-start hosts without site characterization).

- **`antenna_calibration`** (object): records which ANTEX entry the
  engine should use for PCV.  When the engine launches with this
  state file, it auto-supplies `--antex-path` and
  `--receiver-antenna` from these fields.

- **`source`** (object): provenance of the current `ecef_m` /
  `sigma_m`:
  - `method`: one of `opus_static_Nday_mean`, `pride_pp_running_mean`,
    `f9p_rtk_cors`, `nrcan_csrs_pp`, `manual_seed`, ...
  - `n_solutions`: how many independent solutions agreed
  - `first_solution`, `last_solution`: date range
  - `products`: short tag for which IGS / NGS products were used
  - `sigma_per_solution_m`: typical 3D σ for one solution from the
    method
  - `across_day_sigma_m`: empirical spread of solutions around
    their mean (the dominant contribution to `sigma_m` once
    `n_solutions ≥ 3`)

- **`watchdog`** (object): NAV2 watchdog tuning + state:
  - `k_factor`: bark threshold = K × max(NAV2.hAcc, σ_arp).  Default 5.
  - `sustain_epochs`: bark fires after this many consecutive epochs
    above threshold.  Default 60 (~1 minute at 1 Hz).
  - `alarm_state`: `ok` | `suspect` | `barking` | `re_surveying`.
    Set by the engine watchdog evaluator.
  - `alarm_first_epoch`: the epoch at which displacement first
    exceeded threshold (resets to null on `ok`).
  - `last_check_ts`: most recent watchdog evaluation timestamp.
  - `last_displacement_m`: most recent NAV2-vs-pin 3D displacement.

- **`history`** (list): per-solution append-only record.  One entry
  per daily PRIDE / OPUS / cross-check run.  Used by the running-mean
  computation in I-013307-main and as audit trail.  Trimmed to the
  last N (e.g. 90) solutions to keep the file small.

- **`mount_id`** (int): incremented when the watchdog fires
  `barking` and orchestrator inflates σ_arp.  All `history`
  entries with the prior `mount_id` are retained for audit but
  don't contribute to the running mean for the new mount_id.

- **`updated_ts`** (ISO 8601 UTC): last write time.

## Operations on the file

### Initial creation (cold mount)

The orchestrator creates the file with `sigma_m: Infinity` (or a
large sentinel like `1e9`) and empty `history`.  Engine on next
launch sees σ_arp > pin_threshold_m → re-survey mode.  Each daily
PRIDE run appends to `history` and recomputes `sigma_m`.  When
σ_arp crosses below `pin_threshold_m`, next launch transitions to
pinned mode.

### Engine read (every launch)

Engine receives `--surveyed-position-from <receiver_uid>`, reads the
file, branches:

```python
if state["sigma_m"] < args.pin_threshold_m:
    # Pinned mode
    init_pinned_mode(
        ecef=state["ecef_m"],
        feedline_ns=state.get("feedline_ns"),
        antex=state["antenna_calibration"]["atx_path"],
        antenna=f"{state['antenna_calibration']['model']:<16s}"
                f"{state['antenna_calibration']['radome']:<4s}",
    )
else:
    # Re-survey mode
    init_resurvey_mode()  # AntPosEst on, RINEX capture
```

### Watchdog updates (per epoch under pinned mode)

```python
displacement = norm(nav2_ecef - state["ecef_m"])
threshold = state["watchdog"]["k_factor"] * max(nav2_hAcc, state["sigma_m"])
if displacement > threshold:
    if state["watchdog"]["alarm_first_epoch"] is None:
        state["watchdog"]["alarm_first_epoch"] = epoch
        state["watchdog"]["alarm_state"] = "suspect"
    elif epoch - state["watchdog"]["alarm_first_epoch"] >= state["watchdog"]["sustain_epochs"]:
        # Bark
        state["watchdog"]["alarm_state"] = "barking"
        state["sigma_m"] = max(state["sigma_m"], nav2_hAcc * 5)
        state["mount_id"] += 1
        log_alarm("ARP_WATCHDOG_BARK", displacement, threshold)
else:
    state["watchdog"]["alarm_first_epoch"] = None
    state["watchdog"]["alarm_state"] = "ok"
```

### Daily post-processing append (I-013307-main)

```python
new_solution = {
    "ts": today_utc_date,
    "method": "pride_pp_24h_static",
    "ecef_m": pride_result.ecef_m,
    "sigma_m": pride_result.sigma_m,
    "products": "igs_final",
}
state["history"].append(new_solution)
state["history"] = state["history"][-90:]  # cap

# Recompute running mean over solutions sharing current mount_id
current_mount_solutions = [
    h for h in state["history"]
    if h.get("mount_id", 1) == state["mount_id"]
]
if len(current_mount_solutions) >= 3:
    mean_ecef, across_day_sigma = compute_running_mean(current_mount_solutions)
    state["ecef_m"] = mean_ecef
    state["sigma_m"] = across_day_sigma
    state["source"]["n_solutions"] = len(current_mount_solutions)
```

## Defaults & thresholds (for discussion)

- **`pin_threshold_m`**: 0.10 m.  When σ_arp drops below 10 cm,
  pinned mode is operationally safe.  This is roughly 3× the
  per-day OPUS uncertainty, which sets a conservative bar.
- **`watchdog.k_factor`**: 5.  Distance is in 3D, NAV2.hAcc is
  horizontal-only — slightly inflated K accounts for vertical
  multipath that NAV2 doesn't separately reflect.
- **`watchdog.sustain_epochs`**: 60.  ~1 minute at 1 Hz.  Long
  enough to ride through transient NAV2 multipath spikes; short
  enough to detect a real mount move within a minute.
- **`history` cap**: 90 solutions.  Three months of daily PPP at
  one solution per day.  Sufficient to detect long-term drift
  (antenna creep, pylon settling, mount loosening) while keeping
  file size manageable.

These are starting defaults — finalized after I-013307 has produced
real per-day uncertainty distributions for our hosts.

## Migration from existing `last_known_position`

`state/receivers/<uid>.json` already has a `last_known_position`
field with `ecef_m`, `sigma_m`, `source`, `updated`.  That field
is "what the realtime PPP filter computed at the end of its last
run" — not "what the surveyed ARP is".  They serve different
purposes:

- `last_known_position`: warm-start seed for next engine launch.
  Updated continuously by the realtime filter.  σ_m is sub-meter
  but reflects realtime PPP uncertainty (which is decimetric at
  best, biased per yesterday's NAV2-anchor analysis).
- `surveyed ARP` (this schema): authoritative truth from
  multi-day post-processing.  Updated daily.  σ_m sub-cm.

When σ_arp < pin_threshold_m, engine prefers surveyed ARP.  When
σ_arp ≥ pin_threshold_m or the ARP file doesn't exist, engine
falls back to `last_known_position` (existing seeded warm-start
behavior).  This makes the new schema additive — old hosts run
unchanged until their ARP file accumulates enough solutions.

## Open questions (for morning consensus)

1. **Receiver-keyed vs antenna-keyed file location**: receiver-keyed
   matches existing convention but means duplicate files on a
   shared antenna.  Antenna-keyed is conceptually purer but
   requires antenna-id discipline.  Spec leans receiver-keyed for
   consistency.
2. **Default `pin_threshold_m`**: 0.10 m or tighter (0.05 m)?
   Tighter requires more daily solutions to converge; looser is
   more permissive of noisier survey methods.
3. **Watchdog displacement metric**: 3D ECEF distance, or
   horizontal-only (NAV2.hAcc baseline)?  Different antennas have
   different vertical multipath profiles.  3D recommended; revisit
   if false alarms appear.
4. **`history` retention**: 90 solutions is a guess.  The right
   answer depends on how often we want long-term drift trends to
   be visible — e.g., do we want to see seasonal mount creep?

## References

- I-013239-main (orchestrator: this schema's purpose)
- I-013307-main (Axis 1: produces `history` entries)
- I-013342-main (Axis 2: consumes the file at engine startup)
- I-024942-main (METAR-seeded ZTD: separate, complementary path)
- `state/receivers/README.md` (existing receiver-keyed convention)
- `timelab/surveys/2026-05-03-ufo1-opus-static.md` (first surveyed
  ARP that motivates the schema)

#!/usr/bin/env bash
#
# holdover_preflight.sh — mark a host's pre-departure state for a
# holdover-drift experiment (e.g. "take PiFace to lunch on UPS, measure how
# far the DO drifted while it was away from the antenna").
#
# The drift itself is NOT observed during the trip — there is no GPS-traceable
# reference travelling with the host.  It is read off at REACQUISITION, when a
# good antenna comes back and the engine's first clean carrier-phase epoch
# shows how far the DO slipped versus GPS.  This script marks a clean "zero"
# so holdover_postflight.sh can diff against it, and — critically — verifies
# the running engine is configured to SURVIVE a long no-observation holdover
# without re-bootstrapping (which would destroy the filter continuity the
# whole measurement depends on).
#
# Usage:
#   tools/holdover_preflight.sh [LABEL]
#     LABEL  short tag for the run, e.g. no-antenna | indoor-noisy (default: run)
#
# Env overrides:
#   PEPPAR_REPO   repo checkout (default: $HOME/peppar-fix)
#
# Run this BEFORE you unplug the antenna, while the host is cleanly locked.
# If it prints HAZARD lines, fix them with a RELAUNCH NOW (you haven't left
# yet — a relaunch here is free).  Never relaunch after departure: a restart
# mid-holdover breaks phase continuity and voids the measurement.

set -euo pipefail

REPO="${PEPPAR_REPO:-$HOME/peppar-fix}"
cd "$REPO"

LABEL="${1:-run}"
TS="$(date -u +%Y%m%dT%H%M%SZ)"
OUT="data/holdover/${TS}-${LABEL}-departure"
mkdir -p "$OUT"

# --- helper: pull "--flag value" or "--flag=value" out of a cmdline string ---
flag_val() { sed -nE "s/.*--$1[ =]([^ ]+).*/\1/p" <<<"$2" | head -1; }
has_flag() { grep -q -- "--$1" <<<"$2"; }

# --- 1. locate the running engine and capture its exact invocation ----------
PID="$(pgrep -f 'peppar_fix_engine\.py' | head -1 || true)"
if [[ -z "$PID" ]]; then
    echo "FATAL: no peppar_fix_engine.py process found — is the engine running?" >&2
    exit 1
fi
CMDLINE="$(tr '\0' ' ' < "/proc/$PID/cmdline")"
echo "$PID"     > "$OUT/engine.pid"
echo "$CMDLINE" > "$OUT/engine.cmdline"

# --- 2. holdover-safety audit of the running config -------------------------
# These must be right on the *currently running* process, because the only way
# to change them is a relaunch — which is fine now, fatal after departure.
HAZARD=0
CHECKS="$OUT/CHECKS.txt"
: > "$CHECKS"
note() { echo "$1" | tee -a "$CHECKS"; }

if flag_val obs-stall-recovery-s "$CMDLINE" | grep -qx '0'; then
    note "OK    obs-stall-recovery disabled (engine will coast, not exit-5)."
else
    note "HAZARD obs-stall-recovery NOT disabled — with the default 60s/3-try"
    note "       ladder the engine exit-5's ~4 min after obs stop and the"
    note "       wrapper re-bootstraps, destroying phase continuity."
    note "       FIX: relaunch with  --obs-stall-recovery-s 0"
    HAZARD=1
fi

if has_flag gross-fault-reset "$CMDLINE"; then
    note "HAZARD --gross-fault-reset enabled — may reset the filter during the"
    note "       long holdover.  FIX: relaunch without it."
    HAZARD=1
else
    note "OK    --gross-fault-reset not enabled."
fi

if has_flag no-antposest "$CMDLINE"; then
    note "OK    --no-antposest set (position pinned; no re-survey on return)."
else
    note "NOTE  --no-antposest not set — position filter is live; watch for a"
    note "       position re-bootstrap when the antenna returns."
fi

# --- 3. diagnostic logging must be on, or reacquisition isn't captured -------
declare -a LOGS=()
for f in filter-state-log servo-log ticc-log tdcp-log extint-log qerr-log; do
    p="$(flag_val "$f" "$CMDLINE")"
    [[ -n "$p" ]] && LOGS+=("$p")
done
if [[ ${#LOGS[@]} -eq 0 ]]; then
    note "HAZARD no diagnostic log flags on the cmdline — the reacquisition"
    note "       error won't be recorded.  FIX: relaunch with --filter-state-log"
    note "       (and --servo-log / --ticc-log) pointing at persistent files."
    HAZARD=1
elif ! grep -qE -- '--filter-state-log|--servo-log' "$OUT/engine.cmdline"; then
    note "NOTE  neither --filter-state-log nor --servo-log present; the"
    note "       drift-at-reacquisition capture will be weak (TICC only)."
fi

# --- 4. record log offsets so post-flight slices the exact trip window -------
: > "$OUT/log_offsets.tsv"
for p in "${LOGS[@]}"; do
    if [[ -f "$p" ]]; then
        lines="$(wc -l < "$p")"; bytes="$(stat -c%s "$p")"
        printf '%s\t%s\t%s\n' "$p" "$lines" "$bytes" >> "$OUT/log_offsets.tsv"
        tail -n 50 "$p" > "$OUT/$(basename "$p").tail-at-departure" || true
    else
        note "NOTE  log path from cmdline does not exist yet: $p"
    fi
done

# --- 5. snapshot state (position seed, freq/drift seed, characterization) ----
# Tolerate root-owned .bak/.stale strays in state/ we neither need nor can read;
# a partial cp of the readable files (positions, DO runtime toml) is what matters.
if [[ -d state ]]; then
    cp -a state "$OUT/state" 2>/dev/null || note "NOTE  some state/ files unreadable (root-owned strays) — skipped"
fi
mkdir -p "$OUT/data"
shopt -s nullglob
for j in data/*.json; do cp -a "$j" "$OUT/data/" 2>/dev/null || true; done
shopt -u nullglob

# --- 6. provenance + environment --------------------------------------------
{
    echo "label=$LABEL"
    echo "departure_utc=$TS"
    echo "host=$(hostname)"
    echo "engine_pid=$PID"
    echo "mono_uptime_s=$(cut -d' ' -f1 /proc/uptime)"
    echo "git_sha=$(git rev-parse HEAD 2>/dev/null || echo unknown)"
    echo "cmdline=$CMDLINE"
} > "$OUT/manifest.env"

# --- 7. temperatures (best-effort; the drift is mostly a thermal story) ------
{
    command -v vcgencmd >/dev/null 2>&1 && echo "cpu: $(vcgencmd measure_temp)"
    for z in /sys/class/thermal/thermal_zone*/temp; do
        [[ -f "$z" ]] && echo "$z: $(cat "$z")"
    done
} > "$OUT/temps-at-departure.txt" 2>/dev/null || true

# --- 8. summary -------------------------------------------------------------
echo
echo "=================================================================="
echo " Departure state marked:  $OUT"
echo "=================================================================="
echo " engine PID   : $PID  (post-flight will verify this is unchanged)"
echo " logs tracked : ${LOGS[*]:-<none>}"
if [[ ${#LOGS[@]} -gt 0 && -f "${LOGS[0]}" ]]; then
    echo " last-good line (${LOGS[0]}):"
    tail -n 1 "${LOGS[0]}" | sed 's/^/   /'
fi
echo "------------------------------------------------------------------"
if [[ $HAZARD -eq 1 ]]; then
    echo " *** HAZARDS FOUND — see above / $CHECKS ***"
    echo " *** Fix with a RELAUNCH NOW, then re-run this script.        ***"
    echo " *** Do NOT relaunch once the host has left the lab.          ***"
else
    echo " Config looks holdover-safe.  Safe to unplug once re-locked."
fi
echo "=================================================================="

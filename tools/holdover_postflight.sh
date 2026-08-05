#!/usr/bin/env bash
#
# holdover_postflight.sh — read the drift after a host returns from a holdover
# excursion.  Pairs with holdover_preflight.sh.
#
# The headline number is the DO's phase error versus GPS at the FIRST clean
# reacquisition epoch, before the servo pulls it back in — that is the whole
# accumulated drift.  This script:
#   * verifies the engine did NOT restart during the trip (PID continuity);
#     a restart voids the measurement, so you want to know immediately;
#   * (--watch) streams the primary log so you can watch reacquisition live as
#     you reconnect the antenna;
#   * (default) snapshots the return state, slices each log to the exact
#     departure->return window, and prints the departure-vs-reacquisition
#     lines plus the holdover duration and temperature excursion.
#
# Usage:
#   tools/holdover_postflight.sh [DEPARTURE_DIR] --watch    # before reconnecting antenna
#   tools/holdover_postflight.sh [DEPARTURE_DIR]            # after it has re-locked
#
#   DEPARTURE_DIR  defaults to the newest data/holdover/*-departure
#
# Env overrides:
#   PEPPAR_REPO   repo checkout (default: $HOME/peppar-fix)
#
# Recommended return order (why network+corrections+TICC BEFORE the antenna):
#   1. network up   -> monitoring attaches AND the NTRIP/SSR stream reconnects
#                      so the first fixes are corrected, not broadcast-only;
#   2. Rb -> TICC    -> the TICC re-locks and can measure the DO settling as an
#                      independent (Rb-referenced) cross-check;
#   3. start --watch here;
#   4. reconnect the GPS antenna -> reacquisition; the first clean epoch is the
#      drift.  Let it glide back; don't hard-step.

set -euo pipefail

REPO="${PEPPAR_REPO:-$HOME/peppar-fix}"
cd "$REPO"

WATCH=0
DEP=""
for a in "$@"; do
    case "$a" in
        --watch) WATCH=1 ;;
        *)       DEP="$a" ;;
    esac
done

# --- resolve the departure snapshot -----------------------------------------
if [[ -z "$DEP" ]]; then
    DEP="$(ls -1d data/holdover/*-departure 2>/dev/null | sort | tail -1 || true)"
fi
if [[ -z "$DEP" || ! -d "$DEP" ]]; then
    echo "FATAL: no departure snapshot found (looked in data/holdover/*-departure)." >&2
    echo "       Pass one explicitly: tools/holdover_postflight.sh <dir> [--watch]" >&2
    exit 1
fi
# shellcheck disable=SC1090
source "$DEP/manifest.env"

# primary log = filter-state-log if present, else first tracked log
PRIMARY="$(awk -F'\t' '$1 ~ /filter-state|servo/ {print $1; exit}' "$DEP/log_offsets.tsv" 2>/dev/null || true)"
[[ -z "$PRIMARY" ]] && PRIMARY="$(awk -F'\t' 'NR==1{print $1}' "$DEP/log_offsets.tsv" 2>/dev/null || true)"

echo "Departure snapshot : $DEP"
echo "Departure (UTC)    : ${departure_utc:-?}   label=${label:-?}"
echo "Engine PID then    : ${engine_pid:-?}"
echo "Primary log        : ${PRIMARY:-<none>}"
echo

# --- PID continuity: did the engine survive the trip? -----------------------
NOW_PID="$(pgrep -f 'peppar_fix_engine\.py' | head -1 || true)"
if [[ -z "$NOW_PID" ]]; then
    echo "!! engine is not running now — cannot confirm continuity."
elif [[ "$NOW_PID" == "${engine_pid:-}" ]]; then
    echo "OK  engine PID unchanged ($NOW_PID) — filter continuity intact."
else
    echo "!! ENGINE RESTARTED during the trip (was ${engine_pid:-?}, now $NOW_PID)."
    echo "!! Phase continuity is broken — the reacquisition error is NOT a clean"
    echo "!! whole-trip drift.  Inspect the log segment for the exit/re-bootstrap."
fi
echo

# --- watch mode: stream the primary log live --------------------------------
if [[ "$WATCH" -eq 1 ]]; then
    if [[ -z "$PRIMARY" || ! -f "$PRIMARY" ]]; then
        echo "FATAL: no primary log to watch." >&2; exit 1
    fi
    echo "Watching $PRIMARY  (Ctrl-C to stop) — reconnect the antenna now."
    echo "Watch for observations to resume; the first clean epoch after the gap"
    echo "is the drift.  New lines below:"
    echo "------------------------------------------------------------------"
    exec tail -n 0 -F "$PRIMARY"
fi

# --- analysis mode: snapshot return state + slice the trip window -----------
TS="$(date -u +%Y%m%dT%H%M%SZ)"
RET="${DEP%-departure}-return-${TS}"
mkdir -p "$RET"

[[ -d state ]] && cp -a state "$RET/state" 2>/dev/null || true
mkdir -p "$RET/data"
shopt -s nullglob
for j in data/*.json; do cp -a "$j" "$RET/data/" 2>/dev/null || true; done
shopt -u nullglob

# slice each tracked log from its departure line-count to now = the trip window
if [[ -f "$DEP/log_offsets.tsv" ]]; then
    while IFS=$'\t' read -r path lines _bytes; do
        [[ -f "$path" ]] || continue
        seg="$RET/$(basename "$path").trip-segment"
        tail -n "+$((lines + 1))" "$path" > "$seg" || true
    done < "$DEP/log_offsets.tsv"
fi

# temps
{
    command -v vcgencmd >/dev/null 2>&1 && echo "cpu: $(vcgencmd measure_temp)"
    for z in /sys/class/thermal/thermal_zone*/temp; do
        [[ -f "$z" ]] && echo "$z: $(cat "$z")"
    done
} > "$RET/temps-at-return.txt" 2>/dev/null || true

# holdover duration (best-effort, from the UTC stamps in the dir names)
dur=""
if command -v date >/dev/null 2>&1 && [[ -n "${departure_utc:-}" ]]; then
    d0="$(date -d "$(sed -E 's/T/ /; s/Z//' <<<"$departure_utc")" +%s 2>/dev/null || true)"
    d1="$(date -u +%s)"
    [[ -n "$d0" ]] && dur="$(( (d1 - d0) / 60 )) min"
fi

echo "=================================================================="
echo " Return snapshot    : $RET"
echo " Holdover duration  : ${dur:-<unknown>}"
echo "------------------------------------------------------------------"
depline="$DEP/$(basename "${PRIMARY:-none}").tail-at-departure"
if [[ -f "$depline" ]]; then
    echo " Departure last-good (locked, ~zero error):"
    tail -n 1 "$depline" | sed 's/^/   /'
fi
seg="$RET/$(basename "${PRIMARY:-none}").trip-segment"
if [[ -f "$seg" ]]; then
    echo
    echo " Holdover onset (first epochs after antenna removed):"
    head -n 3 "$seg" | sed 's/^/   /'
    echo
    echo " Reacquisition + settle (last epochs — the DRIFT is the first clean"
    echo " fix after obs resume; read the clk/innovation column):"
    tail -n 20 "$seg" | sed 's/^/   /'
fi
echo "------------------------------------------------------------------"
echo " Departure temps : $(paste -sd' ' "$DEP/temps-at-departure.txt" 2>/dev/null || echo '?')"
echo " Return temps    : $(paste -sd' ' "$RET/temps-at-return.txt" 2>/dev/null || echo '?')"
echo "------------------------------------------------------------------"
echo " Full trip-segment logs saved under: $RET/*.trip-segment"
echo " Also compare freq seed: $DEP/data/*.json  vs  $RET/data/*.json"
echo " (the re-locked ppb/control-word minus departure = the FREQUENCY drift)"
echo "=================================================================="

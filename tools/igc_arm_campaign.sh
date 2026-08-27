#!/bin/bash
# Unattended arm campaign for the igc TX-timestamp wedge.
#   docs/igc-wedge-repro-plan.md  (igcWedgeAttribution, I-153714)
#
# Runs the arms back-to-back with no operator present.  Two design rules:
#
#  1. EVERY ARM STARTS WITH A FRESH insmod OF ITS NAMED BINARY, by absolute
#     path, never modprobe.  That implements the plan's §2a rule -- the driver
#     under test is a recorded fact, not an inherited assumption -- and it also
#     guarantees each arm begins from an identical, un-wedged state, so no arm
#     can contaminate the next.  A reload costs ~5 s; cross-arm contamination
#     would cost the whole campaign.
#
#  2. NOTHING IS INFERRED THAT CAN BE MEASURED.  srcversion is asserted after
#     every load, EXTTS is pre-flighted before every arm, and a failure is
#     recorded and skipped rather than silently producing a null result.
#
# SAFETY: rmmod igc drops ONLY the DUT.  Management is eth0 on macb; the DUT is
# eth1 on igc.  Verified before this script was written -- re-verified at start.
# On any exit the patched driver is restored and the link brought back up.
set -u

IFACE=eth1
EXTTS_PIN=1
EXTTS_CH=0
REPO=/home/bob/peppar-fix
TOOL="$REPO/tools/igc_tx_timeout_repro.py"
DATA=/home/bob/igc-wedge-data
STAGE=/root/igc-arms
KREL="$(uname -r)"
ARCH="$(uname -m)"
PATCHED_XZ="/var/lib/dkms/igc/6.12.0-ppsfix.1/$KREL/$ARCH/module/igc.ko.xz"
STOCK_XZ="/var/lib/dkms/igc/original_module/$KREL/$ARCH/igc.ko.xz"

mkdir -p "$DATA" "$STAGE"
LOG="$DATA/campaign.log"

say() { echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] $*" | tee -a "$LOG"; }

# ── preconditions ──────────────────────────────────────────────────────────
mgmt=$(ip -o route get 1.1.1.1 2>/dev/null | grep -o 'dev [^ ]*' | awk '{print $2}')
mgmt_drv=$(basename "$(readlink -f /sys/class/net/$mgmt/device/driver 2>/dev/null)")
if [ "$mgmt_drv" = "igc" ]; then
    say "ABORT: management iface $mgmt is on igc — rmmod would strand this host."
    exit 1
fi
say "management=$mgmt ($mgmt_drv), DUT=$IFACE (igc) — safe to reload"

# ── stage the two known binaries ───────────────────────────────────────────
stage() {   # stage <src.xz> <name>  -> echoes the staged .ko path
    local src="$1" name="$2" dst="$STAGE/$name.ko"
    [ -f "$dst" ] || { xz -dc "$src" > "$dst" 2>/dev/null || return 1; }
    echo "$dst"
}
PATCHED_KO=$(stage "$PATCHED_XZ" patched) || { say "ABORT: cannot stage patched"; exit 1; }
STOCK_KO=$(stage "$STOCK_XZ" stock)       || { say "ABORT: cannot stage stock"; exit 1; }

sv_of_file() { modinfo -F srcversion "$1" 2>/dev/null; }
sv_running() { cat /sys/module/igc/srcversion 2>/dev/null; }

PATCHED_SV=$(sv_of_file "$PATCHED_KO")
STOCK_SV=$(sv_of_file "$STOCK_KO")
say "patched: $PATCHED_KO sv=$PATCHED_SV sha=$(sha256sum "$PATCHED_KO" | cut -c1-16)"
say "stock  : $STOCK_KO sv=$STOCK_SV sha=$(sha256sum "$STOCK_KO" | cut -c1-16)"
if [ "$PATCHED_SV" = "$STOCK_SV" ]; then
    say "ABORT: the two binaries have the SAME srcversion — the assertion could"
    say "       not tell the arms apart, which is the whole point of §2a."
    exit 1
fi

# ── driver control ─────────────────────────────────────────────────────────
kill_tool() {   # never `pkill -f` a pattern that our own cmdline contains
    for p in $(ps -eo pid,cmd | awk '/[i]gc_tx_timeout_repro\.py/ {print $1}'); do
        kill "$p" 2>/dev/null
    done
    sleep 2
}

resolve_phc() {  # PHC index can change across a reload — never hardcode ptp0
    local d
    d=$(ls /sys/class/net/$IFACE/device/ptp 2>/dev/null | head -1)
    [ -n "$d" ] && echo "/dev/$d"
}

load_driver() {  # load_driver <ko> <expected_sv> <name>
    local ko="$1" want="$2" name="$3"
    kill_tool
    ip link set "$IFACE" down 2>/dev/null
    rmmod igc 2>/dev/null
    sleep 1
    if ! insmod "$ko" 2>>"$LOG"; then
        say "  FAIL: insmod $ko"
        return 1
    fi
    # wait for the NIC to re-appear and come up
    for _ in $(seq 1 30); do [ -e "/sys/class/net/$IFACE" ] && break; sleep 1; done
    ip link set "$IFACE" up 2>/dev/null
    for _ in $(seq 1 30); do
        [ "$(cat /sys/class/net/$IFACE/carrier 2>/dev/null)" = "1" ] && break
        sleep 1
    done
    local got; got=$(sv_running)
    if [ "$got" != "$want" ]; then
        say "  FAIL: loaded $name but srcversion is $got, expected $want"
        return 1
    fi
    PHC=$(resolve_phc)
    [ -n "$PHC" ] || { say "  FAIL: no PHC for $IFACE after loading $name"; return 1; }
    # pin routing does not survive a reload
    echo "1 $EXTTS_CH" > "/sys/class/ptp/$(basename "$PHC")/pins/SDP$EXTTS_PIN" 2>/dev/null
    say "  loaded $name sv=$got phc=$PHC carrier=$(cat /sys/class/net/$IFACE/carrier)"
    return 0
}

restore() {
    say "restoring patched driver and link"
    load_driver "$PATCHED_KO" "$PATCHED_SV" patched || say "  RESTORE FAILED — check $IFACE"
}
trap restore EXIT

# ── the arms ───────────────────────────────────────────────────────────────
# label | driver | adjfine_hz | tx_hz | duration_s
#
# E* is the EXTTS-vs-TX-rate sweep: adjfine OFF throughout so the ONLY variable
# is TX-timestamp load.  It answers the question arm A raised but could not
# settle -- EXTTS died in ~2 s at unthrottled TX, but does 1 Hz ptp4l do it?
# That is the case the lab actually runs.
ARMS=(
  "E1-tx1-patched|patched|0|1|1200"
  "E2-tx5-patched|patched|0|5|900"
  "E3-tx20-patched|patched|0|20|900"
  "E4-tx100-patched|patched|0|100|600"
  "E5-tx1000-patched|patched|0|1000|600"
  "E6-txmax-patched|patched|0|unthrottled|600"
  "B-prod1hz-patched|patched|1|1|5400"
  "C-prod1hz-stock|stock|1|1|5400"
  "D-control-stock|stock|0|1|3600"
  "A2-1hz-patched|patched|0|1|3600"
)

say "=== campaign start: ${#ARMS[@]} arms ==="
for spec in "${ARMS[@]}"; do
    IFS='|' read -r label drv adj tx dur <<< "$spec"
    case "$drv" in
        patched) ko="$PATCHED_KO"; sv="$PATCHED_SV" ;;
        stock)   ko="$STOCK_KO";   sv="$STOCK_SV" ;;
        *) say "SKIP $label: unknown driver '$drv'"; continue ;;
    esac
    say "--- ARM $label  driver=$drv adjfine=${adj}Hz tx=${tx} dur=${dur}s ---"
    if ! load_driver "$ko" "$sv" "$drv"; then
        say "SKIP $label: driver load failed"
        continue
    fi
    sleep 3   # let the link settle before pre-flight
    python3 "$TOOL" --iface "$IFACE" --ptp "$PHC" \
        --adjfine-hz "$adj" --tx-hz "$tx" --duration "$dur" \
        --extts-index "$EXTTS_CH" --extts-pin "$EXTTS_PIN" \
        --expect-srcversion "$sv" --label "$label" \
        --csv "$DATA/$label.csv" > "$DATA/$label.log" 2>&1
    rc=$?
    say "  $label finished rc=$rc  (rc=1 means timeouts were observed)"
    grep -E "^(kernel events|inter-burst|VERDICT|EXTTS)" "$DATA/$label.log" \
        | sed 's/^/    /' | tee -a "$LOG"
done
say "=== campaign complete ==="

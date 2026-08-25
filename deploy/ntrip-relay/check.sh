#!/usr/bin/env bash
# Verify the gt NTRIP re-caster deployment.  Read-only; exits non-zero on any
# FAIL so it can be a cron/CI gate.
#
# Each check below exists because that exact failure has already happened:
#
#   ORPHAN   2026-08-24: a fix touched 3 of 4 str2str units.  The 4th was
#            invisible because it was STAGED (disabled, no disk usage, absent
#            from `systemctl --user list-units`).  Nothing enumerated them.
#   ROTATION 2026-08-24: `::T` is RTKLIB's time-tag flag, not a swap interval.
#            With no `::S=`, one file grew to 4.4 GB and filled /home.
#   REACH    a log written below the retention sweep's -maxdepth is never
#            pruned, so rotation alone still leaks.  logs/prod/ hit this.
#   DRIFT    units are copies, so a hand-edit on the box silently diverges
#            from the repo.
set -uo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/units"
DST="${HOME}/.config/systemd/user"
ENVDIR="${HOME}/opt/ntrip-relay"
RET="$DST/ntrip-relay-retention.service"

fail=0
ok()   { printf '  ok    %s\n' "$*"; }
bad()  { printf '  FAIL  %s\n' "$*"; fail=1; }
warn() { printf '  warn  %s\n' "$*"; }

echo "== units in repo are installed and match =="
for f in "$SRC"/*.service "$SRC"/*.timer; do
    n="$(basename "$f")"
    if   [[ ! -f "$DST/$n" ]]; then bad "$n not installed"
    elif ! cmp -s "$f" "$DST/$n"; then bad "$n DRIFT — installed differs from repo"
    else ok "$n"; fi
done

echo "== no orphan str2str units on the box =="
# Any unit whose ExecStart runs str2str must be tracked in the repo.
for f in "$DST"/*.service; do
    [[ -f "$f" ]] || continue
    grep -q 'str2str' "$f" 2>/dev/null || continue
    n="$(basename "$f")"
    [[ -f "$SRC/$n" ]] && ok "$n tracked" || bad "$n is an ORPHAN — runs str2str, not in the repo"
done

echo "== every file:// output rotates (::S=) =="
for f in "$SRC"/*.service; do
    n="$(basename "$f")"
    while read -r out; do
        [[ -z "$out" ]] && continue
        if [[ "$out" == *"::S="* ]]; then ok "$n rotates ($(echo "$out" | grep -o '::S=[0-9]*'))"
        else bad "$n has a file:// out with NO ::S= swap interval — it will grow without bound: $out"; fi
    done < <(grep -o 'file://[^ ]*' "$f" 2>/dev/null)
done

echo "== retention sweep reaches every log path =="
if [[ ! -f "$RET" ]]; then
    bad "retention unit not installed"
else
    logdir=$(grep -oP 'Environment=LOGDIR=\K\S+' "$RET" | head -1)
    depth=$(grep -oP '\-maxdepth \K[0-9]+' "$RET" | head -1)
    : "${logdir:=$ENVDIR/logs}"; : "${depth:=1}"
    ok "sweep root=$logdir maxdepth=$depth"
    for f in "$SRC"/*.service; do
        n="$(basename "$f")"
        while read -r out; do
            [[ -z "$out" ]] && continue
            path="${out#file://}"; path="${path%%::*}"
            dir="$(dirname "$path")"
            case "$dir" in
                "$logdir") lvl=1 ;;
                "$logdir"/*) rel="${dir#"$logdir"/}"; lvl=$(( $(tr -cd '/' <<<"$rel" | wc -c) + 2 )) ;;
                *) bad "$n logs OUTSIDE the sweep root: $dir"; continue ;;
            esac
            [[ $lvl -le $depth ]] && ok "$n log dir within maxdepth ($dir)" \
                || bad "$n logs to $dir (depth $lvl) but sweep is -maxdepth $depth — never pruned"
        done < <(grep -o 'file://[^ ]*' "$f" 2>/dev/null)
    done
fi

echo "== credentials =="
if [[ ! -f "$ENVDIR/relay.env" ]]; then bad "relay.env missing"
else
    perm=$(stat -c %a "$ENVDIR/relay.env")
    [[ "$perm" == "600" ]] && ok "relay.env mode 0600" || warn "relay.env mode $perm (want 600)"
    for k in IGS_USER IGS_PASS PRODUCTS_USER PRODUCTS_PASS; do
        grep -q "^$k=" "$ENVDIR/relay.env" && ok "$k set" || bad "$k missing from relay.env"
    done
fi

echo "== enabled relays are serving =="
for spec in ntrip-ssr-relay:2101 ntrip-obs-relay:2102 ntrip-eph-relay:2103; do
    u="${spec%:*}"; port="${spec#*:}"
    [[ "$(systemctl --user is-enabled "$u" 2>/dev/null)" == "enabled" ]] || { warn "$u not enabled — skipping probe"; continue; }
    # Two traps in this one line:
    #  1. NTRIP v1 answers "SOURCETABLE 200 OK" / "ICY 200 OK", which is not
    #     valid HTTP — curl reports 000 for a perfectly healthy mount.  Probe
    #     with a raw socket.
    #  2. str2str's ntripc requires a User-Agent header.  Without one it
    #     accepts the connection and then answers NOTHING, which is
    #     indistinguishable from a dead service.  Measured 2026-08-24.
    resp=$(printf 'GET / HTTP/1.0\r\nUser-Agent: NTRIP relay-check\r\n\r\n' \
             | timeout 8 nc -w5 127.0.0.1 "$port" 2>/dev/null | head -1 | tr -d '\r')
    [[ "$resp" == *"200 OK"* ]] && ok "$u serving on :$port ($resp)" || bad "$u not answering on :$port (got '${resp:-nothing}')"
done

echo
[[ $fail -eq 0 ]] && echo "ALL CHECKS PASSED" || echo "CHECKS FAILED"
exit $fail

#!/usr/bin/env bash
# Verify the gt NTRIP re-caster deployment.  Read-only; exits non-zero on any
# FAIL so it can be a cron/CI gate.
#
# Every check below exists because that exact failure has already happened:
#
#   ORPHAN   2026-08-24: a rotation fix touched 3 of 4 str2str units.  The 4th
#            was invisible — STAGED, so no disk usage and absent from
#            `systemctl --user list-units`.  Nothing enumerated them.
#   ROTATION `::T` is RTKLIB's time-tag flag, not a swap interval.  With no
#            `::S=`, one file grew to 4.4 GB and filled /home.
#   REACH    a log written below the retention sweep's -maxdepth is never
#            pruned, so rotation alone still leaks.  logs/prod/ hit this.
#   SPLIT    ARGS is word-split unquoted, so a value containing whitespace or
#            a quote reaches str2str mangled.
#   CREDCHAR str2str takes credentials raw inside the URL; an '@' or ':' in a
#            password makes the authority ambiguous.
#   DRIFT    units and instance files are copies, so a hand-edit on the box
#            silently diverges from the repo.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DST="${HOME}/.config/systemd/user"
ENVDIR="${NTRIP_RELAY_DIR:-${HOME}/opt/ntrip-relay}"
RET="$DST/ntrip-relay-retention.service"

fail=0
ok()   { printf '  ok    %s\n' "$*"; }
bad()  { printf '  FAIL  %s\n' "$*"; fail=1; }
warn() { printf '  warn  %s\n' "$*"; }

echo "== units and instance files match the repo =="
for f in "$HERE"/units/*.service "$HERE"/units/*.timer; do
    n="$(basename "$f")"
    if   [[ ! -f "$DST/$n" ]];      then bad "$n not installed"
    elif ! cmp -s "$f" "$DST/$n";   then bad "$n DRIFT — installed differs from repo"
    else ok "$n"; fi
done
if ! cmp -s "$HERE/bin/str2str-relay-exec" "$ENVDIR/bin/str2str-relay-exec"; then
    bad "str2str-relay-exec DRIFT or missing at $ENVDIR/bin/"
else ok "str2str-relay-exec"; fi
for f in "$HERE"/instances/*.env; do
    n="$(basename "$f")"
    if   [[ ! -f "$ENVDIR/instances/$n" ]];      then bad "instances/$n not installed"
    elif ! cmp -s "$f" "$ENVDIR/instances/$n";   then bad "instances/$n DRIFT"
    else ok "instances/$n"; fi
done

echo "== no legacy or orphan str2str units on the box =="
# Anything that runs str2str, or our launcher, must be the template.  This is
# the fourth-unit bug caught structurally rather than by diligence.
found_orphan=0
for f in "$DST"/*.service; do
    [[ -f "$f" ]] || continue
    n="$(basename "$f")"
    [[ "$n" == "str2str-relay@.service" ]] && continue
    # Match the ExecStart line only — the retention unit mentions str2str in a
    # comment, and a comment is not a process.
    if grep -qE '^ExecStart=.*str2str' "$f" 2>/dev/null; then
        bad "$n runs str2str outside the template — legacy or orphan"; found_orphan=1
    fi
done
[[ $found_orphan -eq 0 ]] && ok "only str2str-relay@.service runs str2str"

echo "== every instance is well-formed =="
for f in "$HERE"/instances/*.env; do
    i="$(basename "$f" .env)"
    bash -n "$f" 2>/dev/null || { bad "instances/$i.env is not valid shell"; continue; }
    ( set -a; . "$f" ) >/dev/null 2>&1 || { bad "instances/$i.env failed to source"; continue; }
    IN=""; ARGS=""; CRED=""
    # shellcheck disable=SC1090
    eval "$(grep -E '^(IN|ARGS|CRED)=' "$f")" 2>/dev/null
    [[ -n "$IN"   ]] && ok "$i IN set"   || bad "$i missing IN="
    [[ -n "$ARGS" ]] && ok "$i ARGS set" || bad "$i missing ARGS="

    # SPLIT — ARGS is word-split unquoted by the wrapper, so no embedded quotes.
    # Inspect the RAW line, not the sourced value: sourcing performs quote
    # removal, so an inner quote would be gone before we could see it.  The
    # outer quotes that wrap the whole assignment are stripped first.
    raw_args="$(grep -E '^ARGS=' "$f" | head -1)"; raw_args="${raw_args#ARGS=}"
    raw_args="${raw_args#\"}"; raw_args="${raw_args%\"}"
    case "$raw_args" in
        *'"'*|*"'"*) bad "$i ARGS contains a quote character — it is word-split unquoted, so the quote would reach str2str verbatim" ;;
        *) ok "$i ARGS has no embedded quotes" ;;
    esac

    # ROTATION + REACH, per file:// output.
    for out in $(printf '%s\n' $ARGS | grep '^file://' || true); do
        [[ "$out" == *"::S="* ]] && ok "$i rotates ($(echo "$out" | grep -o '::S=[0-9]*'))" \
            || bad "$i has a file:// out with NO ::S= swap interval — it will grow without bound"
        path="${out#file://}"; path="${path%%::*}"; dir="$(dirname "$path")"
        logdir=$(grep -oP 'Environment=LOGDIR=\K\S+' "$RET" 2>/dev/null | head -1); : "${logdir:=$ENVDIR/logs}"
        depth=$(grep -oP '\-maxdepth \K[0-9]+' "$RET" 2>/dev/null | head -1);       : "${depth:=1}"
        case "$dir" in
            "$logdir") lvl=1 ;;
            "$logdir"/*) rel="${dir#"$logdir"/}"; lvl=$(( $(tr -cd '/' <<<"$rel" | wc -c) + 2 )) ;;
            *) bad "$i logs OUTSIDE the sweep root: $dir"; continue ;;
        esac
        [[ $lvl -le $depth ]] && ok "$i log dir within maxdepth ($dir)" \
            || bad "$i logs to $dir (depth $lvl) but sweep is -maxdepth $depth — never pruned"
    done

    # A named credential pair must exist.
    if [[ -n "$CRED" ]]; then
        u="${CRED}_USER"; p="${CRED}_PASS"
        if grep -q "^$u=" "$ENVDIR/relay.env" 2>/dev/null && grep -q "^$p=" "$ENVDIR/relay.env" 2>/dev/null
        then ok "$i CRED=$CRED resolves"; else bad "$i CRED=$CRED but $u/$p missing from relay.env"; fi
    fi
done

echo "== credentials =="
if [[ ! -f "$ENVDIR/relay.env" ]]; then bad "relay.env missing"
else
    perm=$(stat -c %a "$ENVDIR/relay.env")
    [[ "$perm" == "600" ]] && ok "relay.env mode 0600" || warn "relay.env mode $perm (want 600)"
    while IFS='=' read -r k v; do
        [[ "$k" =~ ^[A-Z]+_(USER|PASS)$ ]] || continue
        # CREDCHAR — these go raw into the URL authority.
        case "$v" in
            *@*|*:*|*/*) bad "$k contains @, : or / — ambiguous inside the str2str URL" ;;
            *) ok "$k url-safe" ;;
        esac
    done < <(grep -E '^[A-Z]+_(USER|PASS)=' "$ENVDIR/relay.env" 2>/dev/null)
fi

echo "== enabled instances are serving =="
# rtkdirect serves no local mount -- it only pushes -- so it has no port here
declare -A PORT=( [ssr]=2101 [obs]=2102 [eph]=2103 [ufo1]=2104 )
for f in "$HERE"/instances/*.env; do
    i="$(basename "$f" .env)"; port="${PORT[$i]:-}"
    [[ -n "$port" ]] || { warn "$i has no known port — skipping probe"; continue; }
    if [[ "$(systemctl --user is-enabled "str2str-relay@$i.service" 2>/dev/null)" != "enabled" ]]; then
        warn "str2str-relay@$i not enabled (staged) — skipping probe"; continue
    fi
    # Two traps here:
    #  1. NTRIP v1 answers "SOURCETABLE 200 OK" / "ICY 200 OK", which is NOT
    #     valid HTTP — curl reports 000 for a perfectly healthy mount.
    #  2. str2str's ntripc REQUIRES a User-Agent.  Without one it accepts the
    #     connection and answers nothing, indistinguishable from a dead service.
    #  3. str2str's ntripc services accepted connections on an internal cycle,
    #     so a SINGLE probe is not reliable: measured 2026-08-24, ten
    #     back-to-back probes all answered, but probes spaced 2 s apart
    #     alternated answer / no-answer.  Retry before concluding it is down —
    #     a health check that cries wolf half the time is worse than none.
    #     Do NOT "simplify" this back to one attempt.
    resp=""
    for _try in 1 2 3; do
        resp=$(printf 'GET / HTTP/1.0\r\nUser-Agent: NTRIP relay-check\r\n\r\n' \
                 | timeout 8 nc -w5 127.0.0.1 "$port" 2>/dev/null | head -1 | tr -d '\r')
        [[ "$resp" == *"200 OK"* ]] && break
        sleep 1
    done
    [[ "$resp" == *"200 OK"* ]] && ok "str2str-relay@$i serving on :$port ($resp)" \
        || bad "str2str-relay@$i not answering on :$port after 3 tries (got '${resp:-nothing}')"
done

echo
[[ $fail -eq 0 ]] && echo "ALL CHECKS PASSED" || echo "CHECKS FAILED"
exit $fail

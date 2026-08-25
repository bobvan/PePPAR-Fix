#!/usr/bin/env bash
# Install the gt NTRIP re-casters.  Idempotent — safe to re-run.
#
# COPIES rather than symlinks, deliberately: `systemctl --user disable` DELETES
# a unit file that is a symlink into a repo.  Observed 2026-08-24 on
# rtcm-archive-gt.service, which vanished from ~/.config/systemd/user on a
# plain `disable --now`.  Copies survive; check.sh reports drift.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DST="${HOME}/.config/systemd/user"
ENVDIR="${NTRIP_RELAY_DIR:-${HOME}/opt/ntrip-relay}"

RELAYS=(ssr obs eph)          # continuously running
STAGED=(onocoy)               # present but never auto-enabled

mkdir -p "$DST" "$ENVDIR"/{bin,instances,logs/prod}

if [[ ! -f "$ENVDIR/relay.env" ]]; then
    echo "!! $ENVDIR/relay.env is missing."
    echo "   cp $HERE/relay.env.example $ENVDIR/relay.env"
    echo "   chmod 0600 $ENVDIR/relay.env   # then fill in the credentials"
    exit 1
fi
chmod 0600 "$ENVDIR/relay.env"

# ── migrate off the pre-template units ───────────────────────────────────
# Four near-identical units were collapsed into str2str-relay@.service.  They
# bind the same ports, so they must be gone before the instances start.
LEGACY=(ntrip-ssr-relay ntrip-obs-relay ntrip-eph-relay onocoy-obs-transcode)
for u in "${LEGACY[@]}"; do
    if [[ -f "$DST/$u.service" ]]; then
        echo "  - $u.service (legacy — stopping, disabling, removing)"
        systemctl --user disable --now "$u.service" >/dev/null 2>&1 || true
        rm -f "$DST/$u.service"
    fi
done

install_file() {   # src dst mode
    if [[ -f "$2" ]] && cmp -s "$1" "$2"; then printf '  = %s\n' "$(basename "$2")"
    else install -m "$3" "$1" "$2"; printf '  + %s\n' "$(basename "$2")"; CHANGED=1; fi
}

CHANGED=0
for f in "$HERE"/units/*.service "$HERE"/units/*.timer; do
    install_file "$f" "$DST/$(basename "$f")" 0644
done
install_file "$HERE/bin/str2str-relay-exec" "$ENVDIR/bin/str2str-relay-exec" 0755
for f in "$HERE"/instances/*.env; do
    install_file "$f" "$ENVDIR/instances/$(basename "$f")" 0644
done

[[ $CHANGED -eq 1 ]] && { systemctl --user daemon-reload; echo "  daemon-reload done"; }

for i in "${RELAYS[@]}"; do
    systemctl --user enable --now "str2str-relay@$i.service" >/dev/null 2>&1 || true
done
systemctl --user enable --now ntrip-relay-retention.timer >/dev/null 2>&1 || true

# Staged instances are installed but never started here: activating one has an
# external side effect (@onocoy begins feeding Onocoy).  See
# onocoy-activation-README.md.
for i in "${STAGED[@]}"; do
    echo "  · str2str-relay@$i installed but NOT enabled (staged)"
done

echo
echo "Installed.  Verify with: $HERE/check.sh"

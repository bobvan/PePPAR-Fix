#!/usr/bin/env bash
# Install the gt NTRIP re-caster units.  Idempotent — safe to re-run.
#
# COPIES rather than symlinks, deliberately: `systemctl --user disable` DELETES
# a unit file that is a symlink into a repo.  Observed 2026-08-24 on
# rtcm-archive-gt.service, which vanished from ~/.config/systemd/user on a
# plain `disable --now`.  A copy survives disable; run check.sh to detect drift.
set -euo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/units"
DST="${HOME}/.config/systemd/user"
ENVDIR="${HOME}/opt/ntrip-relay"

mkdir -p "$DST" "$ENVDIR/logs/prod"

if [[ ! -f "$ENVDIR/relay.env" ]]; then
    echo "!! $ENVDIR/relay.env is missing."
    echo "   cp $(dirname "$SRC")/relay.env.example $ENVDIR/relay.env"
    echo "   chmod 0600 $ENVDIR/relay.env   # then fill in the credentials"
    exit 1
fi
chmod 0600 "$ENVDIR/relay.env"

changed=0
for f in "$SRC"/*.service "$SRC"/*.timer; do
    n="$(basename "$f")"
    if [[ -f "$DST/$n" ]] && cmp -s "$f" "$DST/$n"; then
        printf '  = %s\n' "$n"
    else
        install -m 0644 "$f" "$DST/$n"
        printf '  + %s (installed)\n' "$n"
        changed=1
    fi
done

[[ $changed -eq 1 ]] && systemctl --user daemon-reload && echo "  daemon-reload done"

# The three relays run continuously.  onocoy-obs-transcode is deliberately NOT
# enabled here — it is staged, and activating it is a decision with an external
# side effect (it starts feeding Onocoy).  See onocoy-activation-README.md.
systemctl --user enable --now \
    ntrip-ssr-relay.service ntrip-obs-relay.service ntrip-eph-relay.service \
    ntrip-relay-retention.timer >/dev/null 2>&1 || true

echo
echo "Installed.  Verify with: $(dirname "$SRC")/check.sh"

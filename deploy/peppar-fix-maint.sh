#!/bin/bash
# Bound the local log (logrotate) + ship compressed rotated segments to the
# sshfs archive (gt-nfs/<host>/logs).  Runs OFF the servo hot path (a timer).
#
# Stderr is intentionally NOT suppressed: a real logrotate/rsync/sshfs failure
# must surface in the journal and fail the unit.  The previous version hid
# stderr and rsync'd an unguarded glob, so every cycle with no rotated segment
# waiting passed the literal pattern to rsync and exited 23 ("partial transfer")
# — masking genuine offload failures behind a constant benign-looking one.
set -u
H=$(hostname)
/usr/sbin/logrotate -s "$HOME/peppar-fix/data/.logrotate.state" /etc/logrotate.d/peppar-fix
DEST="$HOME/gt-nfs/$H/logs"
if mountpoint -q "$HOME/gt-nfs"; then
  shopt -s nullglob
  segments=("$HOME"/peppar-fix/data/peppar-fix.log.*.gz)
  if (( ${#segments[@]} )); then
    mkdir -p "$DEST"
    # move rotated+compressed segments off the tiny local disk to gt.
    # logrotate reuses .1.gz every cycle (--remove-source-files means segments
    # never accumulate locally to .2/.3/.4), so a flat rsync would overwrite the
    # previous archived segment each time.  Stamp each with its mtime so the
    # archive actually accumulates instead of keeping only the latest.
    for seg in "${segments[@]}"; do
      stamp=$(date -r "$seg" +%Y%m%dT%H%M%S)
      rsync -a --remove-source-files "$seg" "$DEST/peppar-fix.log-$stamp.gz"
    done
  fi
fi

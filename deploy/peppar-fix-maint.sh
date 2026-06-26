#!/bin/bash
# Bound the local log (logrotate) + ship compressed rotated segments to the
# sshfs archive (gt-nfs/<host>/logs).  Runs OFF the servo hot path (a timer).
set -u
H=$(hostname)
/usr/sbin/logrotate -s "$HOME/peppar-fix/data/.logrotate.state" /etc/logrotate.d/peppar-fix 2>/dev/null
DEST="$HOME/gt-nfs/$H/logs"
if mountpoint -q "$HOME/gt-nfs"; then
  mkdir -p "$DEST"
  # move rotated+compressed segments off the tiny local disk to gt
  rsync -a --remove-source-files "$HOME"/peppar-fix/data/peppar-fix.log.*.gz "$DEST/" 2>/dev/null
fi

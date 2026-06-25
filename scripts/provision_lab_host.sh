#!/usr/bin/env bash
# provision_lab_host.sh — make a lab host a first-class PePPAR-Fix home.
#
# Idempotent: run it any time to (re)establish the standard setup + run a
# preflight check.  Closes the "need a deploy checklist / setup script" gap
# from docs/lab-operations.md.  Run FROM the repo working tree:
#
#     cd ~/peppar-fix && bash scripts/provision_lab_host.sh
#
# What it does (provision):
#   - verifies ~/peppar-fix is the SINGLE authoritative git checkout
#   - creates the venv at ./venv and installs the ENGINE-ONLY stack via
#     pyproject (`pip install -e '.[timebeat]'` — numpy/scipy/pyubx2/pyserial/
#     pyrtcm/pyproj + smbus2; never --break-system-packages).  Deliberately
#     LEAN: NO analysis libs (matplotlib/allantools/pandas live in the
#     `[analysis]` extra).  These disk-constrained Timebeat hosts do NO on-host
#     crunching — pull captures to gt and analyze there.
#   - creates data/ + state/ runtime dirs
# What it checks (preflight, non-fatal warnings):
#   - engine imports, serial/PTP devices, ntrip.conf, host config
set -u
cd "$(dirname "$0")/.." || exit 2
REPO="$(pwd)"
HOST="$(hostname)"
WARN=0
ok(){   echo "  ✓ $*"; }
warn(){ echo "  ⚠ $*"; WARN=$((WARN+1)); }
die(){  echo "  ✗ $*"; exit 2; }

echo "=== PePPAR-Fix host provisioning: $HOST ($REPO) ==="

# 1. single authoritative git checkout (guard the 2026-04-08 multi-copy mess)
[ -d .git ] || die "not a git working tree — \$HOME/peppar-fix must be the checkout"
ok "git working tree ($(git branch --show-current 2>/dev/null) @ $(git rev-parse --short HEAD 2>/dev/null))"
# Stray-copy guard — catch ANY ~/peppar* / ~/PePPAR* / ~/git/PePPAR-Fix that
# isn't the authoritative checkout.  The 2026-04-08 incident was non-git copies
# at ~/peppar-fix / ~/git/PePPAR-Fix / ~/PePPAR-Fix; on disk-constrained
# Timebeat hosts they also waste scarce space (otcBob1: peppar-fix-old 97M +
# peppar_venv 28M found 2026-06-25).
for stray in "$HOME"/peppar* "$HOME"/PePPAR* "$HOME/git/PePPAR-Fix"; do
  [ -d "$stray" ] && [ "$stray" != "$REPO" ] || continue
  # Only flag actual repo/venv COPIES — a .git, a scripts/peppar_fix tree, or a
  # venv (pyvenv.cfg).  Skips legit data dirs (e.g. peppar-survey-data/raw).
  if [ -d "$stray/.git" ] || [ -d "$stray/scripts/peppar_fix" ] || [ -f "$stray/pyvenv.cfg" ]; then
    warn "stray copy at $stray ($(du -sh "$stray" 2>/dev/null | cut -f1)) — remove it; the repo lives ONLY at ~/peppar-fix (CLAUDE.md 2026-04-08)"
  fi
done

# 2. venv + engine deps via pyproject (canonical; smbus2 for ClockMatrix/I2C)
if [ ! -x venv/bin/python ]; then
  echo "  creating venv ..."; python3 -m venv venv || die "venv creation failed"
fi
echo "  installing engine-only stack (pip install -e '.[timebeat]'; no analysis libs) ..."
venv/bin/pip install --quiet --upgrade pip >/dev/null 2>&1
if venv/bin/pip install --quiet -e '.[timebeat]' >/tmp/provision_pip.log 2>&1; then
  ok "engine installed in ./venv"
else
  tail -5 /tmp/provision_pip.log; die "pip install -e '.[timebeat]' failed (see /tmp/provision_pip.log)"
fi

# 3. runtime dirs
mkdir -p data state/dos state/receivers state/positions && ok "data/ + state/ dirs"

# 3.5 udev rules — non-root device access (dialout group + stable PTP symlinks).
# Without these /dev/ptp* is root-only (0600) and the engine can't open it as a
# normal user — the i226 igc PHC needs MODE=0664 GROUP=dialout (bob is in
# dialout).  Caught provisioning ptBoat 2026-06-25; the rules are also what
# give /dev/ptp_i226 / /dev/ticc* their stable names.
RULES=timelab/99-timelab.rules
DEST=/etc/udev/rules.d/99-timelab.rules
if [ -f "$RULES" ]; then
  if sudo cmp -s "$RULES" "$DEST" 2>/dev/null; then
    ok "udev rules current"
  elif sudo cp "$RULES" "$DEST" 2>/dev/null && sudo udevadm control --reload-rules 2>/dev/null \
       && sudo udevadm trigger 2>/dev/null; then
    ok "deployed 99-timelab.rules + reloaded udev"
  else
    warn "udev rule deploy failed (passwordless sudo?) — /dev/ptp* may stay root-only"
  fi
else
  warn "timelab/99-timelab.rules missing from repo — can't grant non-root device access"
fi

# 4. preflight checks (warnings only)
echo "--- preflight ---"
venv/bin/python -c "import numpy,scipy,pyubx2,pyrtcm,serial,pyproj,smbus2" 2>/dev/null \
  && ok "engine imports OK" || warn "engine import failed (rerun provision)"

CFG="config/$(echo "$HOST" | tr '[:upper:]' '[:lower:]').toml"
if [ -f "$CFG" ]; then
  ok "host config $CFG"
  SER=$(grep -E '^\s*serial' "$CFG" | head -1 | sed -E 's/.*=\s*"?([^"]+)"?.*/\1/')
  [ -n "${SER:-}" ] && { [ -e "$SER" ] && ok "serial $SER present" || warn "serial $SER MISSING"; }
  PTP=$(grep -E '^\s*ptp_dev' "$CFG" | head -1 | sed -E 's/.*=\s*"?([^"]+)"?.*/\1/')
  if [ -n "${PTP:-}" ]; then
    if [ ! -e "$PTP" ]; then warn "PTP $PTP MISSING"
    elif [ -r "$PTP" ] && [ -w "$PTP" ]; then ok "PTP $PTP present + user-accessible (rw)"
    else warn "PTP $PTP present but NOT user-accessible (root-only) — 99-timelab.rules + dialout needed"; fi
  fi
else
  warn "no host config $CFG — create one (see config/otcbob1.toml) so the wrapper auto-discovers it"
fi

[ -f ntrip.conf ] && ok "ntrip.conf present" || \
  warn "ntrip.conf MISSING — scp TimeHat:~/peppar-fix/ntrip.conf . (the one legitimate scp; credentials not in repo)"

# disk pressure — Timebeat hosts have small disks (otcBob1 = 7 G).  Keep logs +
# captures lean; pull captures to gt and don't leave stray copies.
AVAIL=$(df -P / | awk 'NR==2{print $4}')
USEPCT=$(df -P / | awk 'NR==2{print $5}')
if [ "${AVAIL:-0}" -lt 307200 ]; then   # < 300 MB
  warn "low disk: / is $USEPCT used, $((AVAIL/1024)) MB free — clean captures (pull to gt) / strays"
else
  ok "disk / $USEPCT used, $((AVAIL/1024)) MB free"
fi

echo "=== $HOST: provisioned ($WARN warning(s)) ==="
exit 0

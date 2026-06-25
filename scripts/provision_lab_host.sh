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
#   - creates the venv at ./venv and installs the engine via pyproject
#     (`pip install -e '.[timebeat]'` — canonical, pulls numpy/scipy/pyubx2/
#     pyserial/pyrtcm/pyproj + smbus2; never --break-system-packages)
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
for stray in "$HOME/git/PePPAR-Fix" "$HOME/PePPAR-Fix"; do
  [ -e "$stray" ] && [ "$stray" != "$REPO" ] && \
    warn "stray copy at $stray — the repo must live ONLY at ~/peppar-fix (see CLAUDE.md, 2026-04-08 incident)"
done

# 2. venv + engine deps via pyproject (canonical; smbus2 for ClockMatrix/I2C)
if [ ! -x venv/bin/python ]; then
  echo "  creating venv ..."; python3 -m venv venv || die "venv creation failed"
fi
echo "  installing engine (pip install -e '.[timebeat]') ..."
venv/bin/pip install --quiet --upgrade pip >/dev/null 2>&1
if venv/bin/pip install --quiet -e '.[timebeat]' >/tmp/provision_pip.log 2>&1; then
  ok "engine installed in ./venv"
else
  tail -5 /tmp/provision_pip.log; die "pip install -e '.[timebeat]' failed (see /tmp/provision_pip.log)"
fi

# 3. runtime dirs
mkdir -p data state/dos state/receivers state/positions && ok "data/ + state/ dirs"

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
  [ -n "${PTP:-}" ] && { [ -e "$PTP" ] && ok "PTP $PTP present" || warn "PTP $PTP MISSING"; }
else
  warn "no host config $CFG — create one (see config/otcbob1.toml) so the wrapper auto-discovers it"
fi

[ -f ntrip.conf ] && ok "ntrip.conf present" || \
  warn "ntrip.conf MISSING — scp TimeHat:~/peppar-fix/ntrip.conf . (the one legitimate scp; credentials not in repo)"

echo "=== $HOST: provisioned ($WARN warning(s)) ==="
exit 0

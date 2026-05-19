#!/usr/bin/env bash
# Install peppar-survey native dependencies (--pride backend).
#
# Idempotent: re-running on a host that's already set up is a no-op
# for the apt step and a rebuild for PRIDE-PPP-AR.
#
# Verifies pdp3 at the end.  Exit 0 on success, non-zero on failure.
#
# See docs/peppar-survey-install.md for the full procedure + scheduling.

set -euo pipefail

INSTALL_DIR="${HOME}/PRIDE-PPPAR.install"
BIN_DIR="${HOME}/.PRIDE_PPPAR_BIN"

echo "=== 1/4 apt deps (gfortran gcc make bc) ==="
# bc is needed at pdp3 runtime (pdp3.sh line ~852 does arithmetic via bc).
# Not strictly a build dep but easier to bundle here than to ask the
# operator to install it separately on first --pride invocation.
if ! command -v gfortran >/dev/null 2>&1 \
   || ! command -v gcc >/dev/null 2>&1 \
   || ! command -v make >/dev/null 2>&1 \
   || ! command -v bc >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y gfortran gcc make bc
else
    echo "  all build + runtime deps present, skipping apt"
fi

echo "=== 2/4 fetch PRIDE-PPP-AR ==="
if [ -d "${INSTALL_DIR}/.git" ]; then
    echo "  existing checkout at ${INSTALL_DIR}, pulling latest"
    git -C "${INSTALL_DIR}" pull --ff-only
else
    echo "  cloning into ${INSTALL_DIR}"
    git clone --depth 1 \
        https://github.com/PrideLab/PRIDE-PPPAR.git "${INSTALL_DIR}"
fi

echo "=== 3/4 build + install ==="
cd "${INSTALL_DIR}"
# install.sh ships without exec bit on git checkouts; invoke via bash.
# It asks "Run tests? [Y/N]" — decline for first install (tests need
# internet + ~1h).  Subsequent runs can be Y if you want to validate.
echo N | bash install.sh

echo "=== 4/5 verify pdp3 ==="
if [ ! -x "${BIN_DIR}/pdp3" ]; then
    echo "ERROR: ${BIN_DIR}/pdp3 not found after install.sh"
    exit 1
fi
"${BIN_DIR}/pdp3" -V || {
    echo "ERROR: pdp3 -V failed"
    exit 1
}

echo "=== 5/5 inject lab-specific antenna calibrations ==="
# PRIDE's bundled IGS antex doesn't include receiver-specific NGS
# calibrations like SFESPK6618H NONE (CHOKE1/UFO1 antennas).  Without
# the matching antex block, pdp3 silently falls back to a different
# or zero calibration and ARP results bias by ~1 m vs OPUS-Static.
# Injecting now is part of the install, not an afterthought.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -x "${script_dir}/inject_lab_antennas.sh" ]; then
    "${script_dir}/inject_lab_antennas.sh" "${INSTALL_DIR}"
else
    echo "WARNING: ${script_dir}/inject_lab_antennas.sh not found — "
    echo "         PRIDE will fall back to default antex for lab antennas."
    echo "         Run scripts/inject_lab_antennas.sh manually after pulling peppar-fix."
fi

echo
echo "peppar-survey --pride backend ready on this host."
echo "  pdp3:  ${BIN_DIR}/pdp3"
echo "  build: ${INSTALL_DIR}"
echo "  antex injected: SFESPK6618H NONE"
echo
echo "Next: ./scripts/peppar_survey.py --pride --rinex-glob '...'"

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

echo "=== 1/4 apt deps (gfortran gcc make) ==="
if ! command -v gfortran >/dev/null 2>&1 \
   || ! command -v gcc >/dev/null 2>&1 \
   || ! command -v make >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y gfortran gcc make
else
    echo "  all compilers present, skipping apt"
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

echo "=== 4/4 verify pdp3 ==="
if [ ! -x "${BIN_DIR}/pdp3" ]; then
    echo "ERROR: ${BIN_DIR}/pdp3 not found after install.sh"
    exit 1
fi
"${BIN_DIR}/pdp3" -V || {
    echo "ERROR: pdp3 -V failed"
    exit 1
}

echo
echo "peppar-survey --pride backend ready on this host."
echo "  pdp3:  ${BIN_DIR}/pdp3"
echo "  build: ${INSTALL_DIR}"
echo
echo "Next: ./scripts/peppar_survey.py --pride --rinex-glob '...'"

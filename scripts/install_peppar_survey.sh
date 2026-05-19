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

echo "=== 1/5 apt deps (gfortran gcc make bc) ==="
# bc is needed at pdp3 runtime (pdp3.sh line ~852 does arithmetic via bc).
# Not strictly a build dep but easier to bundle here than to ask the
# operator to install it separately on first --pride invocation.
if ! command -v gfortran >/dev/null 2>&1 \
   || ! command -v gcc >/dev/null 2>&1 \
   || ! command -v make >/dev/null 2>&1 \
   || ! command -v bc >/dev/null 2>&1; then
    # apt-get may return non-zero from unrelated dpkg triggers (e.g.
    # rpi-chromium-mods on Raspberry Pi OS, NVIDIA on Ubuntu) even when
    # our deps install cleanly.  Don't let that fail the install — re-
    # check command availability afterward, only error if any of OUR
    # deps still missing.
    sudo apt-get update || true
    sudo apt-get install -y gfortran gcc make bc || true
    missing=""
    for c in gfortran gcc make bc; do
        command -v "$c" >/dev/null 2>&1 || missing="$missing $c"
    done
    if [ -n "$missing" ]; then
        echo "ERROR: failed to install required deps:$missing"
        exit 1
    fi
else
    echo "  all build + runtime deps present, skipping apt"
fi

echo "=== 2/5 fetch PRIDE-PPP-AR ==="
if [ -d "${INSTALL_DIR}/.git" ]; then
    echo "  existing checkout at ${INSTALL_DIR}, pulling latest"
    git -C "${INSTALL_DIR}" pull --ff-only
else
    echo "  cloning into ${INSTALL_DIR}"
    # GnuTLS/curl errors on some lab hosts when cloning from github.com
    # over IPv6 or a flaky link.  Retry up to 3 times before giving up.
    n=0
    until git clone --depth 1 \
            https://github.com/PrideLab/PRIDE-PPPAR.git "${INSTALL_DIR}" \
            > /tmp/pride-clone.log 2>&1; do
        n=$((n + 1))
        if [ $n -ge 3 ]; then
            cat /tmp/pride-clone.log
            echo "ERROR: PRIDE-PPP-AR clone failed after 3 attempts."
            echo "       If GnuTLS/curl errors persist, scp the source from"
            echo "       another lab host: 'rsync -a gt:PRIDE-PPPAR/ ${INSTALL_DIR}/'"
            exit 1
        fi
        echo "  clone attempt $n failed, retrying..."
        rm -rf "${INSTALL_DIR}"
        sleep 5
    done
fi

echo "=== 3/5 build + install ==="
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
# pdp3 -V prints the banner but exits non-zero (PRIDE script convention
# for help/version output).  Don't treat that as failure — check for
# the banner text instead.  If pdp3 itself is broken we'll see no output.
pdp3_out="$("${BIN_DIR}/pdp3" -V 2>&1 || true)"
if echo "$pdp3_out" | grep -q "PRIDE PPP-AR version"; then
    echo "$pdp3_out" | grep "PRIDE PPP-AR version"
else
    echo "ERROR: pdp3 -V did not print expected banner:"
    echo "$pdp3_out"
    exit 1
fi

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

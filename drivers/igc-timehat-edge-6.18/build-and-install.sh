#!/bin/bash
# Build and install the TimeHAT-patched igc kernel driver via DKMS,
# port for Linux kernel 6.8.x on x86-64 Debian/Ubuntu hosts (e.g. ocxo).
#
# See drivers/igc-timehat-edge/ for the original 6.12 version (Raspberry
# Pi 5 / Pi OS Trixie).  This 6.8 port exists because `ocxo` runs
# Debian 12-style kernel 6.8.0-<N>-generic and the 6.12 source tree
# won't compile there (leds classdev infrastructure differs).
#
# Prerequisites:
#   - x86-64 Debian/Ubuntu host with kernel 6.8.x
#   - Intel i226 NIC
#   - linux-headers-$(uname -r)
#   - dkms
#   - sudo access
#
# Usage:
#   cd drivers/igc-timehat-edge-6.8
#   sudo ./build-and-install.sh           # build, install, defer reload to next boot
#   sudo ./build-and-install.sh --load    # build, install, AND reload module now

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SOURCE_DIR="$SCRIPT_DIR/intel-igc-ppsfix"
PKG_NAME="igc"
PKG_VERSION="6.18.34-ppsfix.1"
DKMS_SRC="/usr/src/${PKG_NAME}-${PKG_VERSION}"
LOAD_MODULE=false

if [[ "${1:-}" == "--load" ]]; then
    LOAD_MODULE=true
fi

if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root (use sudo)." >&2
    exit 1
fi

KVER="$(uname -r)"
echo "=== TimeHAT igc PPS fix (6.8 port) — DKMS install ==="
echo "Kernel: $KVER"
echo "DKMS package: $PKG_NAME-$PKG_VERSION"
echo

if [[ ! -f "$SOURCE_DIR/dkms.conf" ]]; then
    echo "ERROR: vendored source missing at $SOURCE_DIR" >&2
    exit 1
fi

case "$KVER" in
    6.8.*) ;;
    *)
        echo "WARNING: this driver targets kernel 6.8.x; you are on $KVER." >&2
        echo "         Build may fail or produce a module that won't load." >&2
        echo "         Continuing anyway." >&2
        ;;
esac

# --- Prerequisites: dkms + kernel headers -----------------------------------

NEED_INSTALL=()
command -v dkms >/dev/null || NEED_INSTALL+=(dkms)

if [[ ! -f "/lib/modules/$KVER/build/Makefile" ]]; then
    # Debian/Ubuntu x86-64: generic per-kernel headers package.
    NEED_INSTALL+=("linux-headers-$KVER")
fi

if (( ${#NEED_INSTALL[@]} > 0 )); then
    echo "=== Installing prerequisites: ${NEED_INSTALL[*]} ==="
    apt-get update -qq
    apt-get install -y "${NEED_INSTALL[@]}"
fi

if [[ ! -f "/lib/modules/$KVER/build/Makefile" ]]; then
    echo "ERROR: kernel headers still missing at /lib/modules/$KVER/build" >&2
    echo "       Try: sudo apt install linux-headers-\$(uname -r)" >&2
    exit 1
fi

# --- Stage source under /usr/src for DKMS -----------------------------------

if [[ -d "$DKMS_SRC" ]]; then
    echo "=== Removing previous DKMS staging at $DKMS_SRC ==="
    dkms remove "${PKG_NAME}/${PKG_VERSION}" --all 2>/dev/null || true
    rm -rf "$DKMS_SRC"
fi

echo "=== Staging vendored source to $DKMS_SRC ==="
mkdir -p "$DKMS_SRC"
cp -a "$SOURCE_DIR/." "$DKMS_SRC/"

# --- DKMS build + install ---------------------------------------------------

echo "=== dkms add ==="
dkms add -m "$PKG_NAME" -v "$PKG_VERSION"

echo "=== dkms build ==="
dkms build --force -m "$PKG_NAME" -v "$PKG_VERSION"

echo "=== dkms install ==="
dkms install --force -m "$PKG_NAME" -v "$PKG_VERSION"

# --- Replace the stock module so the patched one wins on next boot ---------

STOCK_DIR="/lib/modules/$KVER/kernel/drivers/net/ethernet/intel/igc"
DKMS_KO_ZST="/lib/modules/$KVER/updates/dkms/igc.ko.zst"
DKMS_KO_XZ="/lib/modules/$KVER/updates/dkms/igc.ko.xz"
DKMS_KO="/lib/modules/$KVER/updates/dkms/igc.ko"

replace_if_present() {
    local stock="$1"
    local dkms="$2"
    if [[ -f "$stock" && -f "$dkms" ]]; then
        if [[ ! -f "${stock}.bak" ]]; then
            echo "=== Backing up ${stock} to ${stock}.bak ==="
            cp "$stock" "${stock}.bak"
        fi
        echo "=== Replacing $stock with patched copy ==="
        cp "$dkms" "$stock"
        return 0
    fi
    return 1
}

if ! replace_if_present "$STOCK_DIR/igc.ko.zst" "$DKMS_KO_ZST"; then
  if ! replace_if_present "$STOCK_DIR/igc.ko.xz" "$DKMS_KO_XZ"; then
    if ! replace_if_present "$STOCK_DIR/igc.ko" "$DKMS_KO"; then
        echo "WARNING: could not locate stock igc module path; relying on" >&2
        echo "         DKMS /updates priority alone." >&2
    fi
  fi
fi

echo "=== depmod -a ==="
depmod -a "$KVER"

echo "=== update-initramfs -u -k $KVER ==="
update-initramfs -u -k "$KVER" || {
    echo "WARNING: update-initramfs failed; the patched module may not" >&2
    echo "         load on next boot if the initramfs holds a stale copy." >&2
}

if $LOAD_MODULE; then
    echo "=== Reloading igc module ==="
    if ip link show i226 &>/dev/null; then
        echo "  interface 'i226' will go down briefly during reload"
    fi
    rmmod igc 2>/dev/null || true
    modprobe igc
    sleep 2
    SRCVER=$(cat /sys/module/igc/srcversion 2>/dev/null || echo unknown)
    echo "  Loaded igc srcversion: $SRCVER"
fi

echo
echo "=== Done ==="
echo "Patched igc-${PKG_VERSION} installed via DKMS."
if ! $LOAD_MODULE; then
    echo "Reboot (or pass --load next time) to activate the patched module."
fi
echo
echo "Verify with:"
echo "  sudo testptp -d /dev/ptp0 -L1,1     # pin 1 -> EXTTS chan 1"
echo "  sudo testptp -d /dev/ptp0 -e 5      # 5 events should be exactly 1 s apart"

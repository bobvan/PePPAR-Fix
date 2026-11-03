#!/usr/bin/env bash
# Rebase ppsfix onto the stock igc of the kernel you are about to run.
#
# Replaces the old approach of carrying a whole patched driver snapshot per
# kernel line.  A snapshot silently reverts every upstream igc fix made since
# it was taken; this applies ppsfix to the exact source the running kernel
# package was built from, so upstream fixes are kept.  See README.md.
#
# Usage:  sudo ./rebase.sh [kernel-release]        # default: uname -r
set -euo pipefail

KREL="${1:-$(uname -r)}"
SERIES="$(echo "$KREL" | sed -E 's/^([0-9]+\.[0-9]+).*/\1/')"
PKG_NAME="igc"
PKG_VER="${SERIES}-ppsfix.rebased"
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="/usr/src/${PKG_NAME}-${PKG_VER}"

die() { echo "ERROR: $*" >&2; exit 1; }
[[ $EUID -eq 0 ]] || die "run as root"
[[ -f "$HERE/ppsfix.patch" ]] || die "ppsfix.patch not found beside this script"
[[ -d "/lib/modules/$KREL/build" ]] || die "no headers for $KREL — install linux-headers-$KREL"

echo "=== 1. exact kernel source for $KREL ==="
# linux-source-<series> ships the tarball the kernel package was built from.
# A public git tag is NOT a substitute: verified 2026-11-03 that
# raspberrypi/linux tag stable_20260911 claimed 6.18.50 but its igc called
# xsk_buff_get_metadata() with 3 args while the installed 6.18.50 headers
# declare 2 — i.e. the tag was ahead of the shipped kernel and would not build.
command -v dkms >/dev/null || die "dkms not installed"
dpkg -s "linux-source-$SERIES" >/dev/null 2>&1 || {
    echo "  installing linux-source-$SERIES ..."
    DEBIAN_FRONTEND=noninteractive apt-get install -y -q "linux-source-$SERIES"
}
TARBALL="$(ls /usr/src/linux-source-${SERIES}.tar* 2>/dev/null | head -1)"
[[ -n "$TARBALL" ]] || die "no linux-source-$SERIES tarball under /usr/src"
echo "  source: $TARBALL ($(dpkg-query -W -f='${Version}' "linux-source-$SERIES"))"

echo "=== 2. extract stock igc ==="
WORK="$(mktemp -d)"; trap 'rm -rf "$WORK"' EXIT
tar -xf "$TARBALL" -C "$WORK" --wildcards '*/drivers/net/ethernet/intel/igc/*'
STOCK="$(dirname "$(find "$WORK" -name igc_main.c | head -1)")"
[[ -n "$STOCK" ]] || die "igc not found in the source tarball"
echo "  $(ls "$STOCK"/*.c "$STOCK"/*.h | wc -l) stock files"

echo "=== 3. stage + apply ppsfix ==="
rm -rf "$SRC"; mkdir -p "$SRC/src"
cp "$STOCK"/*.c "$STOCK"/*.h "$SRC/src/"
cp "$HERE/Makefile.oot" "$SRC/src/Makefile"     # out-of-tree build; stock's is kernel-tree style
sed -e "s/^PACKAGE_NAME=.*/PACKAGE_NAME=\"$PKG_NAME\"/" \
    -e "s/^PACKAGE_VERSION=.*/PACKAGE_VERSION=$PKG_VER/" \
    "$HERE/dkms.conf.in" > "$SRC/dkms.conf"
( cd "$SRC/src" && patch -p1 --no-backup-if-mismatch < "$HERE/ppsfix.patch" )
[[ -z "$(ls "$SRC/src"/*.rej 2>/dev/null)" ]] || die "patch rejects — ppsfix needs updating for $SERIES"

echo "=== 4. verify ppsfix went in, and upstream fixes stayed ==="
fail=0
need() { c=$(grep -rc "$1" "$SRC/src" 2>/dev/null | awk -F: '{s+=$2} END{print s+0}')
         [[ "$c" -ge "${2:-1}" ]] && printf '  ok   %s\n' "$1" || { printf '  FAIL %s (found %s)\n' "$1" "$c"; fail=1; }; }
# ppsfix must be present
need 'edge_check_delay_us'; need 'edge_check_invert'; need 'igc_sdp_val'
need 'PPS fix: edge filtering'; need 'PPS fix: force rising edge'; need 'ts0_pin'
need 'IGC_CTRL_SDP0_VAL'
# PEROUT 1 Hz frequency-mode special case must be GONE
if grep -q 'ns == 500000000LL' "$SRC/src/igc_ptp.c"; then
    echo "  FAIL PEROUT freq-mode special case still present"; fail=1
else echo "  ok   PEROUT freq-mode special case removed"; fi
[[ $fail -eq 0 ]] || die "verification failed — do not install this module"

echo "=== 5. dkms build + install for $KREL ==="
dkms remove -m "$PKG_NAME" -v "$PKG_VER" --all 2>/dev/null || true
dkms add -m "$PKG_NAME" -v "$PKG_VER"
dkms build -m "$PKG_NAME" -v "$PKG_VER" -k "$KREL"
dkms install --force -m "$PKG_NAME" -v "$PKG_VER" -k "$KREL"

echo "=== 6. done — verify the BUILT module, not the exit code ==="
KO="$(find "/var/lib/dkms/$PKG_NAME/$PKG_VER/$KREL" -name 'igc.ko*' | head -1)"
modinfo "$KO" | grep -E '^(parm|vermagic)'
echo
echo "After rebooting into $KREL, confirm on the live module:"
echo "  cat /sys/module/igc/parameters/edge_check_delay_us   # -> 20"
echo "  cat /sys/module/igc/parameters/edge_check_invert     # -> 0"

#!/usr/bin/env bash
# Append our lab-specific antenna calibration blocks to PRIDE's bundled
# antex files.  Idempotent: skips files that already contain each block.
#
# Why this exists: PRIDE-PPP-AR ships with the IGS antex catalog,
# which does NOT include receiver-specific NGS antennas like
# SFESPK6618H NONE.  Without these blocks, pdp3 silently falls back
# to a different (or zero) calibration and ARP results are biased by
# ~1 m vs OPUS-Static (which uses the NGS antex).  We extract each
# lab-relevant block once into timelab/antex/<TYPE>_<RADOME>.atx
# and append it to every igs20*.atx (and igs14*.atx, igsR3*.atx if
# present) in PRIDE's table/.
#
# Usage:
#   ./scripts/inject_lab_antennas.sh
#   ./scripts/inject_lab_antennas.sh /custom/path/to/PRIDE-PPPAR.install
#
# Exit 0 on success; non-zero on missing source antex or unwritable PRIDE.

set -euo pipefail

PRIDE_TREE="${1:-${HOME}/PRIDE-PPPAR.install}"
# Default antex source is the in-repo support/antex/.  Override with
# the ANTEX_SOURCE_DIR env var when running outside a checkout, e.g.
# when building a fresh lab host before peppar-fix is cloned.
_THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ANTEX_SOURCE_DIR="${ANTEX_SOURCE_DIR:-${_THIS_DIR}/../support/antex}"

# Lab-relevant antennas to inject.  Filenames under
# timelab/antex/<TYPE>_<RADOME>.atx must each contain exactly one
# antenna block (the block_start..block_end range).
LAB_ANTENNAS=(
    "SFESPK6618H_NONE"   # CHOKE1 + UFO1 receiver-side antenna
)

PRIDE_TABLE="${PRIDE_TREE}/table"
if [ ! -d "${PRIDE_TABLE}" ]; then
    echo "ERROR: ${PRIDE_TABLE} not found.  Pass PRIDE-PPPAR build tree as arg."
    exit 1
fi

inject_one() {
    local source_atx="$1"
    local target_atx="$2"
    local antenna_token="$3"   # what to grep for to detect prior injection

    if [ ! -r "${source_atx}" ]; then
        echo "  ERROR: source ${source_atx} not readable"
        return 1
    fi
    if grep -q "${antenna_token}" "${target_atx}"; then
        echo "  ${target_atx##*/}: already has ${antenna_token}, skip"
        return 0
    fi

    # Extract just the antenna block (START OF ANTENNA → END OF ANTENNA).
    local block
    block="$(awk '/START OF ANTENNA/,/END OF ANTENNA/' "${source_atx}")"
    if [ -z "${block}" ]; then
        echo "  ERROR: no START/END OF ANTENNA block in ${source_atx}"
        return 1
    fi
    # Atomic append: write the new content to a temp + rename.
    local tmp="${target_atx}.tmp.$$"
    cp "${target_atx}" "${tmp}"
    printf '%s\n' "${block}" >> "${tmp}"
    mv "${tmp}" "${target_atx}"
    echo "  ${target_atx##*/}: appended ${antenna_token} (+$(wc -c <<<"${block}") bytes)"
}

shopt -s nullglob
target_files=("${PRIDE_TABLE}"/igs20*.atx "${PRIDE_TABLE}"/igs14*.atx "${PRIDE_TABLE}"/igsR3*.atx)
if [ ${#target_files[@]} -eq 0 ]; then
    echo "ERROR: no igs*.atx files in ${PRIDE_TABLE}"
    exit 1
fi

for ant in "${LAB_ANTENNAS[@]}"; do
    source_atx="${ANTEX_SOURCE_DIR}/${ant}.atx"
    token="${ant%%_*}"  # SFESPK6618H_NONE → SFESPK6618H
    echo "=== injecting ${ant} (token=${token}) ==="
    for target in "${target_files[@]}"; do
        inject_one "${source_atx}" "${target}" "${token}"
    done
done

echo
echo "Done.  PRIDE antex now includes:"
for ant in "${LAB_ANTENNAS[@]}"; do
    token="${ant%%_*}"
    echo "  ${token} in $(grep -l "${token}" "${target_files[@]}" 2>/dev/null | wc -l) of ${#target_files[@]} igs*.atx files"
done

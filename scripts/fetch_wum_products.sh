#!/usr/bin/env bash
# Fetch WUM products (orbit / clock / phase-bias / ERP) from gnsswhu
# into a local cache dir that peppar-survey --wum-source consumes.
#
# Background: pdp3.sh's PrepareProducts walks an URL ladder
# (bdspride → IGN → gnsswhu → bdspride WCC → gnsswhu IGS2R03FIN) for
# each product type.  In practice that chain trips on flaky FTPS,
# unwritten RAP products, or missing-day cases and never reaches the
# RTS fallback at gnsswhu.  This script downloads directly from the
# gnsswhu mirrors that DO carry current-day products and stages them
# under a single dir for --wum-source.  Empirically discovered
# 2026-05-20 (wumProductGnsswhuFallback-charlie).
#
# Usage:
#   bash scripts/fetch_wum_products.sh YEAR DOY [TARGET_DIR]
#
# Args:
#   YEAR        4-digit year (e.g. 2026)
#   DOY         day-of-year, 1-366 (e.g. 140)
#   TARGET_DIR  destination directory (default: ./wum-cache).  Files
#               are placed at TARGET_DIR/<filename>.gz; peppar-survey
#               --wum-source gunzips them into pdp3's product/common
#               at run time.
#
# Output:
#   Up to 4 .gz files per RTS day:
#     WUM0MGXRTS_YYYYDOY0000_01D_05M_ORB.SP3.gz   (orbit)
#     WUM0MGXRTS_YYYYDOY0000_01D_05S_CLK.CLK.gz   (clock)
#     WUM0MGXRTS_YYYYDOY0000_01D_05M_OSB.BIA.gz   (phase bias)
#     WUM0MGXRTS_YYYYDOY0000_01D_01D_ERP.ERP.gz   (Earth rotation)
#   If RTS isn't available (older day), falls back to FIN under
#   /pub/gps/products/mgex/<gpsweek>/.
#
# Caveat (per wumOsbBiaDownload-charlie close): WUM's OSB.BIA does
# NOT include GPS L5 phase biases.  This matters when processing F9T-
# L1+L5 captures: AR won't fully converge on the L5 signals.  This
# script downloads what's available; the L5 coverage gap is a
# product-side limitation, not a download issue.
#
# Exit codes:
#   0  at least one product downloaded.
#   1  bad arguments.
#   2  every product download failed.

set -euo pipefail

if [ $# -lt 2 ] || [ $# -gt 3 ]; then
    echo "usage: $0 YEAR DOY [TARGET_DIR]" >&2
    exit 1
fi

YEAR=$1
DOY=$(printf "%03d" "$2")
TARGET_DIR="${3:-./wum-cache}"

# Compute GPS week from (year, doy) for FIN-fallback URL.  GPS week 0
# starts 1980-01-06.  Bash arithmetic via `date -d`.
EPOCH_DATE=$(date -u -d "${YEAR}-01-01 +$((10#$DOY - 1)) days" +%Y-%m-%d 2>/dev/null) \
    || { echo "ERROR: invalid YEAR/DOY: $YEAR $DOY" >&2; exit 1; }
SECS_TARGET=$(date -u -d "$EPOCH_DATE" +%s)
SECS_GPS_EPOCH=$(date -u -d "1980-01-06" +%s)
DAYS_SINCE_GPS_EPOCH=$(( (SECS_TARGET - SECS_GPS_EPOCH) / 86400 ))
GPS_WEEK=$(( DAYS_SINCE_GPS_EPOCH / 7 ))

mkdir -p "$TARGET_DIR"

# RTS first (covers current + recent days), then FIN (older days).
# Each product type has its own gnsswhu subdir for RTS.
declare -A RTS_URLS=(
    [orbit]="ftp://igs.gnsswhu.cn/pub/whu/phasebias/${YEAR}/orbit/WUM0MGXRTS_${YEAR}${DOY}0000_01D_05M_ORB.SP3.gz"
    [clock]="ftp://igs.gnsswhu.cn/pub/whu/phasebias/${YEAR}/clock/WUM0MGXRTS_${YEAR}${DOY}0000_01D_05S_CLK.CLK.gz"
    [bias]="ftp://igs.gnsswhu.cn/pub/whu/phasebias/${YEAR}/bias/WUM0MGXRTS_${YEAR}${DOY}0000_01D_05M_OSB.BIA.gz"
    [erp]="ftp://igs.gnsswhu.cn/pub/whu/phasebias/${YEAR}/orbit/WUM0MGXRTS_${YEAR}${DOY}0000_01D_01D_ERP.ERP.gz"
)
declare -A FIN_URLS=(
    [orbit]="ftp://igs.gnsswhu.cn/pub/gps/products/mgex/${GPS_WEEK}/WUM0MGXFIN_${YEAR}${DOY}0000_01D_05M_ORB.SP3.gz"
    [clock]="ftp://igs.gnsswhu.cn/pub/gps/products/mgex/${GPS_WEEK}/WUM0MGXFIN_${YEAR}${DOY}0000_01D_30S_CLK.CLK.gz"
    [bias]="ftp://igs.gnsswhu.cn/pub/gps/products/mgex/${GPS_WEEK}/WUM0MGXFIN_${YEAR}${DOY}0000_01D_01D_OSB.BIA.gz"
    [erp]="ftp://igs.gnsswhu.cn/pub/gps/products/mgex/${GPS_WEEK}/WUM0MGXFIN_${YEAR}${DOY}0000_01D_01D_ERP.ERP.gz"
)

n_ok=0
n_fail=0
for kind in orbit clock bias erp; do
    rts_url="${RTS_URLS[$kind]}"
    fin_url="${FIN_URLS[$kind]}"
    rts_name=$(basename "$rts_url")
    fin_name=$(basename "$fin_url")

    # Already cached?
    if [ -f "${TARGET_DIR}/${rts_name}" ] || [ -f "${TARGET_DIR}/${rts_name%.gz}" ] \
       || [ -f "${TARGET_DIR}/${fin_name}" ] || [ -f "${TARGET_DIR}/${fin_name%.gz}" ]; then
        echo "  ${kind}: cached, skipping"
        n_ok=$((n_ok + 1))
        continue
    fi

    # RTS first (recent days).
    echo "  ${kind}: ${rts_url}"
    if curl -fsS --connect-timeout 10 --max-time 60 -o "${TARGET_DIR}/${rts_name}" "$rts_url"; then
        echo "    → ${rts_name}"
        n_ok=$((n_ok + 1))
        continue
    fi
    rm -f "${TARGET_DIR}/${rts_name}"

    # FIN fallback (older days).
    echo "  ${kind}: RTS not found, trying FIN: ${fin_url}"
    if curl -fsS --connect-timeout 10 --max-time 60 -o "${TARGET_DIR}/${fin_name}" "$fin_url"; then
        echo "    → ${fin_name}"
        n_ok=$((n_ok + 1))
        continue
    fi
    rm -f "${TARGET_DIR}/${fin_name}"

    echo "    FAILED — neither RTS nor FIN available for ${kind}" >&2
    n_fail=$((n_fail + 1))
done

echo
echo "Fetched ${n_ok}/4 products into ${TARGET_DIR}"
if [ $n_ok -eq 0 ]; then
    echo "ERROR: no WUM products available for ${YEAR}/${DOY}" >&2
    exit 2
fi
echo "Next: peppar-survey --pride --wum-source ${TARGET_DIR} --rinex-glob ..."

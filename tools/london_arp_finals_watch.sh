#!/bin/bash
# londonArpFinals finals WATCHER — daily probe on gt until a multi-GNSS FINAL
# product for the London Mini PT arc (2026-06-12/13 UTC, GPS week 2422,
# DOY 163/164) publishes, then nudge the refinement.  Dayplan item:
# londonArpFinals (I-103808-charlie).
#
# On a confirmed FINAL it posts a "READY, run the refinement" dayplan nudge and
# self-removes from crontab; past a backstop it posts "still missing"; otherwise
# one quiet log line.  Does NOT run the heavy PPP (the agent's job once nudged).
#
# has_fin distinguishes THREE outcomes so an unreachable archive is NEVER a
# silent "no final" (the exact false-negative this watcher exists to avoid):
#   0 = FINAL present   1 = no-final (dir reachable)   2 = COULD-NOT-REACH
# Only a confirmed FINAL (rc 0) arms the nudge.
#
#   --probe-only : run the probes + print results; NO dayplan post, NO crontab
#                  change.  For testing / verifying the arms by hand.
#
# AC coverage (verified from gt 2026-07-04):
#   CODE (AIUB)  COD0MGXFIN  — reachable; cuts MGEX finals ~5 days after obs. PRIMARY.
#   GFZ  (ISDC)  GBM/GFZ0MGXFIN — best-effort; ISDC path can be unreachable from gt.
#   IGS-combined IGS0OPSFIN  — long latency; the IGN archive can be unreachable from gt.
#   WHU phasebias tree is RAPID/RTS only (no WUM0MGXFIN) — dropped as a final source.

set -uo pipefail
PROBE_ONLY=0
[ "${1:-}" = "--probe-only" ] && PROBE_ONLY=1
LOG=/home/bob/london_arp_finals_watch.log
DP=/home/bob/.claude/projects/-home-bob-git-PePPAR-Fix/dayplan/dayplan.py
SELF="$(readlink -f "$0")"   # repo copy (tools/); was ~/scripts before the PePPAR-Fix/tools move
BACKSTOP=20260715            # YYYYMMDD: stop watching after this (investigate)
[ "$PROBE_ONLY" = 1 ] || exec >>"$LOG" 2>&1
echo "===== finals watch: $(date '+%Y-%m-%d %H:%M:%S %Z')$([ "$PROBE_ONLY" = 1 ] && echo ' (probe-only)') ====="

# Probe a SPECIFIC final product (not just dir reachability).
#   $1=label  $2=url  $3=grep-pattern
#   returns 0 found / 1 no-final (reachable) / 2 could-not-reach
has_fin() {
  local listing rc out
  listing=$(curl -s --connect-timeout 20 --max-time 70 "$2" 2>/dev/null); rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "  [$1] COULD NOT REACH (curl rc=$rc) — probe BLIND, NOT a 'no final'"
    return 2
  fi
  out=$(printf '%s\n' "$listing" | grep -iE "$3" || true)
  if [ -n "$out" ]; then
    echo "  [$1] FINAL present: $(printf '%s' "$out" | head -1)"
    return 0
  fi
  if [ -z "$listing" ]; then
    echo "  [$1] no final (dir empty — verify the path if this AC should have data)"
  else
    echo "  [$1] no final yet"
  fi
  return 1
}

COD="ftp://ftp.aiub.unibe.ch/CODE_MGEX/CODE/2026/"
GFZ="ftp://isdcftp.gfz-potsdam.de/gnss/products/mgex/w2422/"
IGN="ftp://igs.ign.fr/pub/igs/products/2422/"
READY=0
has_fin "COD-FIN 163/164"          "$COD" "COD0MGXFIN_2026(163|164).*(ORB\.SP3|CLK\.CLK)" && READY=1
has_fin "GFZ-FIN 163/164"          "$GFZ" "(GBM0MGXFIN|GFZ0MGXFIN)_2026(163|164).*(ORB|CLK)" && READY=1
has_fin "IGS-combined-FIN 163/164" "$IGN" "IGS0OPSFIN_2026(163|164).*(SP3|CLK)" && READY=1

disarm() { (crontab -l 2>/dev/null | grep -v "london_arp_finals_watch.sh") | crontab - ; echo "  self-removed crontab line."; }

if [ "$PROBE_ONLY" = 1 ]; then
  echo "  probe-only: READY=$READY (no dayplan post, no crontab change)"
  exit 0
fi

if [ "$READY" = "1" ]; then
  $DP discuss --by charlie --id I-103808-charlie --msg "FINALS READY 2026-$(date +%m-%d) (auto-probe): a multi-GNSS FINAL (CODE/GFZ/IGS) for the London arc (GPS wk2422 DOY 163/164) has published. Run the sigma-refinement ON GT now: (1) cd /home/bob/gt/peppar-survey-data/raw/london-24h-l1l2-20260612 && pdp3 -m S -sys GE -frq G12 E17 -n LOND london-1Hz.obs (NO antex injection; confirm the pdp3 log shows it downloaded a FIN product, not RAP/RTS); (2) RTKLIB rnx2rtkp PPP-static with a final SP3/CLK; (3) CSRS-PPP finals upload of london-30s.obs.gz; (4) consensus <=3cm 3D -> adopt mean as APC, update london.survey.toml, refine sigma, keep APC-not-ARP framing." 2>&1 | tail -1
  echo "  posted READY nudge."
  disarm
elif [ "$(date +%Y%m%d)" -ge "$BACKSTOP" ]; then
  $DP discuss --by charlie --id I-103808-charlie --msg "FINALS BACKSTOP 2026-$(date +%m-%d) (auto-probe): no multi-GNSS final for the London arc (GPS wk2422 DOY 163/164) confirmed >1 month after the obs -- unusual (or all probed archives are unreachable from gt -- check the log for COULD-NOT-REACH). Provisional 2.3cm APC still stands. Stopping the auto-watch; needs a human/agent look." 2>&1 | tail -1
  echo "  past backstop; posted + disarming."
  disarm
else
  echo "  still waiting; will re-check tomorrow."
fi

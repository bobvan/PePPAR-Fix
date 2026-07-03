#!/bin/bash
# londonArpFinals finals WATCHER — daily probe on gt until IGS/WUM finals for
# the London Mini PT arc (2026-06-12/13 UTC, GPS week 2422, DOY 163/164)
# publish, then nudge the refinement.  Dayplan item: londonArpFinals
# (I-103808-charlie).
#
# Why a daily watcher (not a one-shot): the 06-26 one-shot found finals NOT yet
# published (only WUM0MGXRAP/RTS rapid, no WUM0MGXFIN; no IGS combined final on
# IGN wk2422).  This probes for the SPECIFIC final products each day and:
#   - finals found  -> dayplan post "READY, run the refinement" + self-remove.
#   - not found      -> quiet (one log line); keeps watching.
#   - past backstop  -> dayplan post "still missing, investigate" + self-remove.
# Does NOT run the heavy PPP (that's the agent's job once nudged).

set -u
LOG=/home/bob/london_arp_finals_watch.log
DP=/home/bob/.claude/projects/-home-bob-git-PePPAR-Fix/dayplan/dayplan.py
SELF="$(readlink -f "$0")"   # repo copy (tools/); was ~/scripts before PePPAR-Fix/tools move
BACKSTOP=20260715          # YYYYMMDD: stop watching after this (investigate)
exec >>"$LOG" 2>&1
echo "===== finals watch: $(date '+%Y-%m-%d %H:%M:%S %Z') ====="

# Probe for a SPECIFIC final product file (not just dir reachability).
# Returns 0 if the listing names a *FIN* product for the day.
has_fin() {  # $1=label $2=url $3=grep-pattern
  local out
  out=$(curl -s --connect-timeout 20 --max-time 70 "$2" 2>/dev/null | grep -iE "$3")
  if [ -n "$out" ]; then echo "  [$1] FINAL present: $(echo "$out" | head -1)"; return 0; fi
  echo "  [$1] no final yet"; return 1
}

# Probe every AC that publishes a multi-GNSS FINAL for this arc — NOT just WUM.
# 2026-07-03: CODE's COD0MGXFIN was available from ~Jun 18 but this watcher only
# checked WUM+IGS-combined and missed it for 2 weeks (charlie, I-103808).  CODE
# and GFZ cut MGEX finals days after the obs; WUM/IGS-combined lag much longer.
WHU="ftp://igs.gnsswhu.cn/pub/whu/phasebias/2026/orbit/"
IGN="ftp://igs.ign.fr/pub/igs/products/2422/"
COD="ftp://ftp.aiub.unibe.ch/CODE_MGEX/CODE/2026/"
GFZ="ftp://isdcftp.gfz-potsdam.de/gnss/products/mgex/w2422/"
READY=0
has_fin "COD-FIN 163/164"       "$COD" "COD0MGXFIN_2026(163|164).*(ORB\.SP3|CLK\.CLK)" && READY=1
has_fin "GFZ-FIN 163/164"       "$GFZ" "(GBM0MGXFIN|GFZ0MGXFIN)_2026(163|164).*(ORB|CLK)" && READY=1
has_fin "WUM-FIN 163/164"       "$WHU" "WUM0MGXFIN_2026(163|164)" && READY=1
has_fin "IGS-combined-FIN wk2422" "$IGN" "IGS0OPSFIN.*(SP3|CLK)" && READY=1

disarm() { (crontab -l 2>/dev/null | grep -v "london_arp_finals_watch.sh") | crontab - ; echo "  self-removed crontab line."; }

if [ "$READY" = "1" ]; then
  $DP discuss --by charlie --id I-103808-charlie --msg "FINALS READY 2026-$(date +%m-%d) (auto-probe): IGS/WUM FINAL products for the London arc (GPS wk2422 DOY 163/164) have published. Run the sigma-refinement ON GT now: (1) cd /home/bob/gt/peppar-survey-data/raw/london-24h-l1l2-20260612 && pdp3 -m S -sys GE -frq G12 E17 -n LOND london-1Hz.obs (NO antex injection; confirm the pdp3 log shows it downloaded a FIN product, not RAP/RTS); (2) RTKLIB rnx2rtkp PPP-static with a final SP3/CLK; (3) CSRS-PPP finals upload of london-30s.obs.gz; (4) consensus <=3cm 3D -> adopt mean as APC, update london.survey.toml, refine sigma, keep APC-not-ARP framing." 2>&1 | tail -1
  echo "  posted READY nudge."
  disarm
elif [ "$(date +%Y%m%d)" -ge "$BACKSTOP" ]; then
  $DP discuss --by charlie --id I-103808-charlie --msg "FINALS BACKSTOP 2026-$(date +%m-%d) (auto-probe): IGS/WUM finals for the London arc (GPS wk2422 DOY 163/164) STILL not published >1 month after the obs -- unusual. Investigate the product source (WUM0MGXFIN naming/path may have changed, or use a different final AC: GFZ/COD/IGS-combined). Provisional 2.3cm APC still stands. Stopping the auto-watch; needs a human/agent look." 2>&1 | tail -1
  echo "  past backstop; posted + disarming."
  disarm
else
  echo "  still waiting; will re-check tomorrow."
fi

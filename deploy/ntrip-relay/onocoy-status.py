#!/usr/bin/env python3
"""Query onocoy's Public API for our own reference station's daily statistics.

Why this is worth having: onocoy grades every contributing station, so these
numbers are an INDEPENDENT assessment of our observation quality — phase/code
RMS, cycle-slip rate, latency, sky visibility — computed by someone who has no
stake in our results.  That is the "free third-party consistency check" the
Box 3 plan was after, and unlike the console it is machine-readable.

Credentials come from ~/opt/ntrip-relay/relay.env (ONOCOY_CRED / ONOCOY_PASS),
never from the command line or the repo.

    ./onocoy-status.py            # last 7 days
    ./onocoy-status.py --days 30

API notes (spec: https://api-hub.onocoy.com/docs):
  - Auth is HTTP Basic; the STATION credential works for the my_stats/* paths.
  - A station credential is only accepted once the station "has a successful
    validation within the last 7 days".  Before that the API answers 403, which
    is not a misconfiguration — validation takes onocoy 24-36 h after first
    data.  401 vs 403 tells the two apart, and this tool says which.
  - Statistics go back at most 3 months, and nothing exists before 2025-12-22.
"""
import argparse, base64, datetime as dt, json, os, re, sys, urllib.request, urllib.error

ENV = os.path.expanduser(os.environ.get("NTRIP_RELAY_DIR", "~/opt/ntrip-relay")) + "/relay.env"
BASE = "https://api-hub.onocoy.com/api/v1"


def creds():
    if not os.path.exists(ENV):
        sys.exit(f"missing {ENV}")
    v = {}
    for line in open(ENV):
        line = line.strip()
        if "=" in line and not line.startswith("#"):
            k, val = line.split("=", 1)
            v[k] = val
    try:
        return v["ONOCOY_CRED"], v["ONOCOY_PASS"]
    except KeyError:
        sys.exit(f"ONOCOY_CRED / ONOCOY_PASS not set in {ENV}")


def get(path, params, user, pw):
    q = "&".join(f"{k}={v}" for k, v in params.items() if v)
    req = urllib.request.Request(f"{BASE}{path}?{q}")
    tok = base64.b64encode(f"{user}:{pw}".encode()).decode()
    req.add_header("Authorization", f"Basic {tok}")
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        body = e.read().decode(errors="replace")[:200]
        if e.code == 401:
            sys.exit("401 invalid credentials — ONOCOY_CRED/ONOCOY_PASS are wrong.\n"
                     "  ONOCOY_CRED must be the auto-generated username onocoy shows as\n"
                     '  "Credential" on the reference-station page.')
        if e.code == 403:
            sys.exit("403 access denied — the credential is VALID but the station has no\n"
                     "  successful validation in the last 7 days.  If the feed was only just\n"
                     "  switched on this is expected: onocoy takes 24-36 h to validate.\n"
                     "  Check the push is actually up:  systemctl --user status str2str-relay@onocoy")
        sys.exit(f"HTTP {e.code}: {body}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--days", type=int, default=7)
    a = ap.parse_args()
    user, pw = creds()
    # UTC, not local: the API bins by UTC day, and west of Greenwich the local
    # date lags it, so date.today() silently asks for the wrong window.
    end = dt.datetime.now(dt.timezone.utc).date()
    start = end - dt.timedelta(days=a.days)
    # end_date is EXCLUSIVE -- measured: start=08-26&end=08-26 returns [] while
    # start=08-26&end=08-27 returns the 08-26 row.  Ask for tomorrow to include
    # today.  This is not documented.
    rows = get("/my_stats/stations",
               {"start_date": start.isoformat(),
                "end_date": (end + dt.timedelta(days=1)).isoformat()}, user, pw)
    if not rows:
        print(f"no statistics for {start} .. {end} UTC (station may be too new — "
              f"onocoy publishes one row per COMPLETE UTC day)")
        return
    hdr = (f"{'date':11s} {'mount':14s} {'epochs/s':>9s} {'latency s':>10s} "
           f"{'code RMS':>9s} {'phase RMS':>10s} {'cs rate':>8s} {'sky':>6s} {'rewards':>9s}")
    print(hdr); print("-" * len(hdr))
    for r in sorted(rows, key=lambda x: x.get("date", "")):
        f = lambda k: r.get(k)
        num = lambda k, p=3: f"{f(k):.{p}f}" if isinstance(f(k), (int, float)) else "-"
        print(f"{str(f('date'))[:10]:11s} {str(f('mountpoint'))[:14]:14s} "
              f"{num('epoch_rate_avg',2):>9s} {num('latency_avg',2):>10s} "
              f"{num('code_rms_avg'):>9s} {num('phase_rms_avg',4):>10s} "
              f"{num('cs_rate_avg',4):>8s} {num('sky_vis_avg',1):>6s} {num('rewards',2):>9s}")


if __name__ == "__main__":
    main()

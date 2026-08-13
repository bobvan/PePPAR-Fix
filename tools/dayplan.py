#!/usr/bin/env python3
"""dayplan — shared multi-agent day plan via append-only ops log.

Storage:
  Live (fixed shared dir, $DAYPLAN_LIVE_DIR): YYYY-MM-DD.log (JSONL ops,
    append-only) — deliberately OUTSIDE the repo so every agent's
    worktree copy appends to one shared log (see LIVE_DIR below).
  Archive (PePPAR-Fix repo): hist/dayplan-YYYY-MM-DD/{plan.md,log.jsonl,lessons.md}

Concurrency: append-only ops; sub-4KB JSON lines are kernel-atomic via
POSIX O_APPEND.  Multiple agents can write simultaneously without
collision.  "Current state" is computed by replaying the log.

Usage:
  dayplan propose --by main --name LandA1NlPib \
    --title "land A1 NL P_IB" --reviewers charlie bravo --owner main
  dayplan ack --by bravo --id LandA1NlPib
  dayplan status --by main --id LandA1NlPib --value in-progress
  dayplan status --by main --id LandA1NlPib --value done --evidence abc123
  dayplan amend --by bravo --id LandA1NlPib --field depends_on --value ZtdStatePrep
  dayplan discuss --by charlie --msg "should we land A1 before B1?"
  dayplan discuss --by charlie --id LandA1NlPib --msg "thread on A1"
  dayplan render
  dayplan close --by main --lessons "..."

The item's --name IS its key: `propose --name AddTwoUps` then
`ack/status/amend/discuss --id AddTwoUps`.  Names must be unique within
a day.  Older items keyed I-HHMM-<agent> still replay and are still
addressable by that id; only new items are name-keyed.

Status values: in-progress | done | deferred | disputed
AGREED is derived (all reviewers ack'd); explicit status overrides.
"""
import argparse
import json
import os
import re
import shutil
from datetime import datetime, timezone
from pathlib import Path
from zoneinfo import ZoneInfo

# Live daily log dir — a FIXED shared absolute path, deliberately
# decoupled from this script's own location.  This file now lives in
# the repo (tools/dayplan.py) and is checked out into every agent's
# worktree (PePPAR-Fix/, PePPAR-Fix-charlie/, …); those copies must all
# append to ONE shared log.  Deriving the log dir from __file__ (the old
# behaviour) would splinter it into a per-worktree log and the agents
# would silently stop hearing each other — the exact thing dayplan
# exists to prevent.  Override with $DAYPLAN_LIVE_DIR (e.g. for tests).
LIVE_DIR = Path(
    os.environ.get(
        "DAYPLAN_LIVE_DIR",
        "/home/bob/.claude/projects/-home-bob-git-PePPAR-Fix/dayplan",
    )
)
HIST_DIR = Path("/home/bob/git/PePPAR-Fix/hist")
VALID_STATUS = ("in-progress", "done", "blocked", "deferred",
                "abandoned", "disputed")

# Day boundaries + ts values track Bob's body clock (CDT/CST), not
# UTC.  Switched 2026-05-11 per Bob's request — UTC midnight cut
# days at noon-ish his local time, which was confusing.  America/
# Chicago auto-handles CDT↔CST transitions.  The 2026-05-11.log file
# becomes ~30 hours long (covers UTC midnight through next local
# midnight); subsequent days are 24-hour local-CDT days.
LOCAL_TZ = ZoneInfo("America/Chicago")


def today():
    return datetime.now(LOCAL_TZ).strftime("%Y-%m-%d")


def now_ts():
    # ISO 8601 with timezone offset (e.g. 2026-05-11T05:30:00-05:00)
    # so consumers can still cross-reference UTC trivially.
    return datetime.now(LOCAL_TZ).strftime("%Y-%m-%dT%H:%M:%S%z")


def log_path(date):
    return LIVE_DIR / f"{date}.log"


def append_op(op_dict, date=None):
    """Append a single op as a JSON line.  POSIX O_APPEND atomic for
    sub-PIPE_BUF (4KB) writes — concurrent appends are safe."""
    p = log_path(date or today())
    line = json.dumps(op_dict, separators=(",", ":")) + "\n"
    if len(line) > 4000:
        # Stay below PIPE_BUF to keep the atomicity guarantee.
        raise SystemExit(
            f"Op too large ({len(line)} bytes > 4000); split body into "
            f"a separate `discuss` op or shorten."
        )
    with open(p, "a") as f:
        f.write(line)


def read_ops(date=None):
    p = log_path(date or today())
    if not p.exists():
        return []
    ops = []
    for raw in p.read_text().splitlines():
        raw = raw.strip()
        if not raw:
            continue
        try:
            ops.append(json.loads(raw))
        except json.JSONDecodeError as e:
            print(f"WARN: skipping malformed op: {raw[:80]}... ({e})")
    return ops


SLUG_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_-]{1,40}$")


def validate_name(name):
    """The item key is a required meatware-friendly slug (no integer IDs).
    A letter, then letters/digits/-/_ — no spaces, so it stays a clean key
    you can carry in your head and retype."""
    if not SLUG_RE.match(name):
        raise SystemExit(
            f"Bad --name '{name}': use a short meatware-friendly key — a "
            f"letter, then letters/digits/-/_ (2–41 chars), no spaces. "
            f"E.g. AddTwoUps, fix-dns-ptr, rinexEpochPhase."
        )
    return name


# ── subcommand handlers ─────────────────────────────────────────── #

def cmd_propose(args):
    name = validate_name(args.name)
    # The name IS the key, so it must be unique within the day's log.
    items, _, _ = _build_state(read_ops(args.date or today()))
    if name in items:
        raise SystemExit(
            f"--name '{name}' already exists today (proposed by "
            f"{items[name]['proposed_by']}). Pick a distinct name, or work "
            f"the existing item with `ack/status/amend/discuss --id {name}`."
        )
    op = {
        "op": "propose",
        "id": name,
        "by": args.by,
        "ts": now_ts(),
        "owner": args.owner or args.by,
        "reviewers": args.reviewers or [],
    }
    if args.title:
        op["title"] = args.title
    if args.body:
        op["body"] = args.body
    append_op(op, args.date)
    print(f"Proposed {name}" + (f": {args.title}" if args.title else ""))


def cmd_ack(args):
    op = {"op": "ack", "ref": args.id, "by": args.by, "ts": now_ts()}
    if args.note:
        op["note"] = args.note
    append_op(op, args.date)
    print(f"Acked {args.id} by {args.by}")


def cmd_amend(args):
    op = {
        "op": "amend",
        "ref": args.id,
        "by": args.by,
        "ts": now_ts(),
        "field": args.field,
        "value": args.value,
    }
    if args.rationale:
        op["rationale"] = args.rationale
    append_op(op, args.date)
    print(f"Amended {args.id}: {args.field}={args.value}")


def cmd_status(args):
    op = {
        "op": "status",
        "ref": args.id,
        "by": args.by,
        "ts": now_ts(),
        "value": args.value,
    }
    if args.evidence:
        op["evidence"] = args.evidence
    append_op(op, args.date)
    print(f"Status {args.id} → {args.value}")


def cmd_discuss(args):
    op = {"op": "discuss", "by": args.by, "ts": now_ts(), "msg": args.msg}
    if args.id:
        op["ref"] = args.id
    append_op(op, args.date)
    print(f"Discussion posted by {args.by}")


def _build_state(ops):
    items = {}
    untargeted_discussions = []
    closed = None
    for op in ops:
        kind = op.get("op")
        if kind == "propose":
            items[op["id"]] = {
                "id": op["id"],
                "title": op.get("title", ""),
                "body": op.get("body"),
                "owner": op.get("owner"),
                "reviewers": op.get("reviewers", []),
                "proposed_by": op["by"],
                "proposed_ts": op["ts"],
                "acks": {},
                "amends": [],
                "statuses": [],
                "discussions": [],
            }
        elif kind == "ack":
            ref = op.get("ref")
            if ref in items:
                items[ref]["acks"][op["by"]] = {
                    "ts": op["ts"], "note": op.get("note"),
                }
        elif kind == "amend":
            ref = op.get("ref")
            if ref in items:
                items[ref]["amends"].append(op)
                # A `title` amend renames the item so meatware-friendly
                # camelCase slugs can be added retroactively (Bob's request,
                # 2026-07-06).  Other fields stay propose-time (owner is
                # deliberately not remutable here).
                if op.get("field") == "title" and op.get("value"):
                    items[ref]["title"] = op["value"]
        elif kind == "status":
            ref = op.get("ref")
            if ref in items:
                items[ref]["statuses"].append(op)
        elif kind == "discuss":
            ref = op.get("ref")
            if ref and ref in items:
                items[ref]["discussions"].append(op)
            else:
                untargeted_discussions.append(op)
        elif kind == "close":
            closed = op
    return items, untargeted_discussions, closed


def _status_label(item):
    """PROPOSED → AGREED (all reviewers acked) → explicit status override."""
    if item["statuses"]:
        return item["statuses"][-1]["value"].upper()
    if item["reviewers"] and all(r in item["acks"] for r in item["reviewers"]):
        return "AGREED"
    return "PROPOSED"


def _render(date):
    ops = read_ops(date)
    items, untargeted, closed = _build_state(ops)
    out = []
    out.append(f"# Dayplan {date}")
    out.append("")
    if closed:
        out.append(f"**CLOSED** at {closed['ts']} by {closed['by']}")
        if closed.get("lessons"):
            out.append(f"Lessons: {closed['lessons']}")
        out.append("")
    if not items and not untargeted:
        out.append("(no items yet — use `dayplan propose --by ... --name ...`)")
        return "\n".join(out)

    # Sort items by proposed_ts
    for item_id in sorted(items, key=lambda i: items[i]["proposed_ts"]):
        item = items[item_id]
        label = _status_label(item)
        _t = item.get("title") or ""
        out.append(f"### [{label}] {item_id}" + (f": {_t}" if _t else ""))
        if item.get("owner"):
            out.append(f"  - owner: **{item['owner']}**")
        if item.get("body"):
            out.append(f"  - body: {item['body']}")
        out.append(f"  - proposed: {item['proposed_by']} @ {item['proposed_ts']}")
        if item["reviewers"]:
            ack_parts = []
            for r in item["reviewers"]:
                if r in item["acks"]:
                    ack = item["acks"][r]
                    note = (f' "{ack["note"]}"' if ack.get("note")
                            else "")
                    ack_parts.append(f"{r}✓@{ack['ts'][11:19]}{note}")
                else:
                    ack_parts.append(f"{r}⏳")
            out.append(f"  - reviewers: {' '.join(ack_parts)}")
        for amend in item["amends"]:
            note = (f" — {amend['rationale']}"
                    if amend.get("rationale") else "")
            out.append(
                f"  - amend [{amend['by']} @ {amend['ts'][11:19]}]: "
                f"{amend['field']}={amend['value']}{note}"
            )
        for s in item["statuses"]:
            ev = f" (evidence: {s['evidence']})" if s.get("evidence") else ""
            out.append(
                f"  - status [{s['by']} @ {s['ts'][11:19]}]: "
                f"{s['value']}{ev}"
            )
        for d in item["discussions"]:
            out.append(f"  - discuss [{d['by']} @ {d['ts'][11:19]}]: {d['msg']}")
        out.append("")
    if untargeted:
        out.append("## General discussion")
        for d in untargeted:
            out.append(f"- [{d['by']} @ {d['ts'][11:19]}] {d['msg']}")
        out.append("")
    # Sign-off summary
    out.append("## Sign-off summary")
    parties = set()
    for item in items.values():
        parties.update(item["reviewers"])
        if item.get("owner"):
            parties.add(item["owner"])
        parties.add(item["proposed_by"])
        parties.update(item["acks"].keys())
    for p in sorted(parties):
        proposed = sum(1 for i in items.values() if i["proposed_by"] == p)
        owner = sum(1 for i in items.values() if i.get("owner") == p)
        ackd = sum(1 for i in items.values() if p in i["acks"])
        pending = sum(1 for i in items.values()
                      if p in i["reviewers"] and p not in i["acks"])
        out.append(
            f"- **{p}**: proposed {proposed}, owner {owner}, acked {ackd}, "
            f"pending {pending}"
        )
    return "\n".join(out)


def cmd_render(args):
    date = args.date or today()
    print(_render(date))


def cmd_close(args):
    date = args.date or today()
    plan_md = _render(date)
    close_op = {"op": "close", "by": args.by, "ts": now_ts()}
    if args.lessons:
        close_op["lessons"] = args.lessons
    append_op(close_op, date)
    HIST_DIR.mkdir(exist_ok=True)
    archive_dir = HIST_DIR / f"dayplan-{date}"
    archive_dir.mkdir(exist_ok=True)
    src = log_path(date)
    shutil.copy2(src, archive_dir / "log.jsonl")
    (archive_dir / "plan.md").write_text(plan_md)
    lessons_text = (args.lessons or
                    "# Lessons learned\n\n(to be filled in)\n")
    (archive_dir / "lessons.md").write_text(lessons_text)
    print(f"Closed dayplan {date} → {archive_dir}")


def main():
    ap = argparse.ArgumentParser(
        description="Shared multi-agent day plan via append-only ops log",
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    def add_by(p):
        p.add_argument("--by", required=True,
                       help="Agent name (main/bravo/charlie/...)")

    def add_date(p):
        p.add_argument("--date", default=None,
                       help="YYYY-MM-DD (default: today America/Chicago)")

    p = sub.add_parser("propose", help="Propose a plan item")
    add_by(p); add_date(p)
    p.add_argument("--name", required=True,
                   help="REQUIRED short meatware-friendly key = the item's "
                        "id (e.g. AddTwoUps, rinexEpochPhase). Unique per "
                        "day; reused in every later --id.")
    p.add_argument("--owner", help="Owner (default: --by)")
    p.add_argument("--reviewers", nargs="+", default=[],
                   help="Agents whose ack is required for AGREED")
    p.add_argument("--title",
                   help="Optional longer description (the name carries the gist)")
    p.add_argument("--body", help="Optional details (kept short)")
    p.set_defaults(func=cmd_propose)

    p = sub.add_parser("ack", help="Acknowledge an item")
    add_by(p); add_date(p)
    p.add_argument("--id", required=True,
                   help="Item name — the key set by `propose --name`")
    p.add_argument("--note",
                   help="Optional reservation/qualifier rendered "
                        "next to the ack")
    p.set_defaults(func=cmd_ack)

    p = sub.add_parser("amend", help="Amend a field on an item")
    add_by(p); add_date(p)
    p.add_argument("--id", required=True,
                   help="Item name — the key set by `propose --name`")
    p.add_argument("--field", required=True)
    p.add_argument("--value", required=True)
    p.add_argument("--rationale", help="Why")
    p.set_defaults(func=cmd_amend)

    p = sub.add_parser("status", help="Update item status")
    add_by(p); add_date(p)
    p.add_argument("--id", required=True,
                   help="Item name — the key set by `propose --name`")
    p.add_argument("--value", required=True, choices=VALID_STATUS)
    p.add_argument("--evidence", help="Commit / path / URL")
    p.set_defaults(func=cmd_status)

    p = sub.add_parser("discuss", help="Post a discussion note")
    add_by(p); add_date(p)
    p.add_argument("--id", help="Item name to thread under (from propose --name)")
    p.add_argument("--msg", required=True)
    p.set_defaults(func=cmd_discuss)

    p = sub.add_parser("render", help="Render current state as markdown")
    add_date(p)
    p.set_defaults(func=cmd_render)

    p = sub.add_parser("close", help="Close + archive to PePPAR-Fix/hist/")
    add_by(p); add_date(p)
    p.add_argument("--lessons", help="Lessons learned text")
    p.set_defaults(func=cmd_close)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()

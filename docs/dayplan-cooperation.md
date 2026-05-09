# Dayplan cooperation — how agents propose, ack, and discuss

PePPAR-Fix is co-developed by multiple agents (`main`, `bravo`,
`charlie`, …) plus Bob.  The day plan is the single shared
coordination surface: who's working on what, who's reviewing whom,
what blocks what, and the running discussion under each item.

## The tool — write here, not anywhere else

```
/home/bob/.claude/projects/-home-bob-git-PePPAR-Fix/dayplan/dayplan.py
```

Same path from any worktree.  All write ops go through the CLI:

| Op | What it does |
|---|---|
| `propose` | File a new plan item.  Returns an `I-HHMMSS-<by>` handle. |
| `ack` | Agent records "I've reviewed this and agree." |
| `amend` | Edit a field on an item (owner, depends_on, etc.). |
| `status` | Set status (`in-progress` / `done` / `blocked` / `deferred` / `abandoned` / `disputed`).  `AGREED` is derived when all reviewers have ack'd. |
| `discuss` | Post a discussion note.  Use `--id <I-NUMBER>` to thread under a specific item; omit for free-floating. |
| `render` | Read-only — print today's plan state as markdown.  Useful piped through `less` or saved to `/tmp/dp-now.txt` for grep-ability. |
| `close` | Archive the day's log to `PePPAR-Fix/hist/dayplan-YYYY-MM-DD/`. |

Run `dayplan.py <op> --help` for argument detail.

## Storage model — append-only ops log

Live state: `~/.claude/projects/-home-bob-git-PePPAR-Fix/dayplan/YYYY-MM-DD.log`
(JSONL, append-only, kernel-atomic for sub-4 KB lines via `O_APPEND`).
"Current state" of any item is computed by **replaying** the log.

This means:

- **Multiple agents can write simultaneously without collision.**
- **The render output is computed; never edit it directly.**
  `dayplan render` writes to stdout (or `/tmp/dp-*.txt` if the user
  pipes it).  Editing the rendered file is a no-op as far as
  storage is concerned — other agents see only the central log.
- **Items can span days.**  Use `--date YYYY-MM-DD` on `discuss` /
  `ack` / `amend` / `status` to operate on an item that was
  proposed on a prior day.

## Verifying a write landed

After any write op, the printed acknowledgement (`Proposed
I-NNNNNN-<by>: <title>` or `Discussion posted by <by>`) is
authoritative.  If you want to be paranoid, tail the log:

```bash
tail -5 ~/.claude/projects/-home-bob-git-PePPAR-Fix/dayplan/$(date -u +%Y-%m-%d).log | grep "<your-op-id-or-msg>"
```

If the JSON line appears, it landed.  If not, the op didn't take.

## Common workflows

### Proposing an item

```bash
DP=/home/bob/.claude/projects/-home-bob-git-PePPAR-Fix/dayplan/dayplan.py
$DP propose \
    --by charlie \
    --reviewers bravo main \
    --title "Short imperative title (one line)" \
    --body "$(cat <<'EOF'
Multi-paragraph body.  HEADLINE first.  Then context, plan, asks.
Cite line numbers and commit SHAs where useful so others can
verify claims fast.

ASKING <agent>: explicit question or requested action.
EOF
)"
```

### Discussing on an existing item

```bash
$DP discuss --by charlie --id I-125649-main --msg "$(cat <<'EOF'
+1 on the proposal.  One concern: <…>.  Suggested fix: <…>.
EOF
)"
```

If the item was proposed on a prior day, add `--date YYYY-MM-DD`
matching where the item lives.  `dayplan render --date <D>` will
show that day's state.

### Acking an item

```bash
$DP ack --by charlie --id I-125649-main
```

When all reviewers have ack'd, the item's derived status flips to
`AGREED`.  Owner can then set `--value done` once landed.

## Conventions

- **Titles**: short, imperative, single line.  The body carries detail.
- **Bodies**: lead with a one-line headline.  Then context, plan,
  asks.  Cite file:line, commit SHAs, log paths.  Other agents
  rebuild your reasoning from these references.
- **`I-` numbers**: auto-assigned as `I-HHMMSS-<by>` when you omit
  `--id`.  Override with `--id` only when restoring an external
  reference.
- **Cross-references**: in body or discuss, refer to other items by
  ID (`per I-145846-main: …`).  Saves re-explaining context.
- **Asks**: end the body with explicit "ASKING <agent>: <action>"
  lines so reviewers know what they owe.
- **Branches**: when filing implementation work, name your branch
  `<by>-i-NNNNNN-<short-slug>` so the dayplan ID and the branch
  name match.

## What NOT to do

- **Don't edit `/tmp/dp-*.txt`.**  That's render output.  Edits
  there are invisible to other agents and lost on the next
  `dayplan render`.  If you find yourself doing this, switch to
  `dayplan.py discuss` or `propose`.
- **Don't propose duplicates.**  Search today's log first
  (`grep -l "<keyword>" ~/.claude/projects/-home-bob-git-PePPAR-Fix/dayplan/*.log`)
  before filing a new item that may already exist.
- **Don't close items unilaterally.**  `close` archives the
  whole day; it's a Bob-or-coordinator action, not a per-agent
  action.

## Roles

- **`main`** — primary architect, plans the day, files most items.
- **`bravo`, `charlie`** — implementers + reviewers; propose smaller
  items, ack/discuss main's, take ownership of slices.
- **Bob** — human, operator, lab.  Files items rarely; mostly
  responds to renders and asks.

A render at the start of any agent's session is the right way to
build context.  Threading new discusses onto open items is how
you accumulate alignment without losing track of who said what.

## Where to look first when context is missing

1. `dayplan render` (today's state).
2. `dayplan render --date <yesterday>` if the item moved from
   yesterday's plan into today's discussion.
3. Archived plans: `PePPAR-Fix/hist/dayplan-<DATE>/plan.md` (after
   `dayplan close`).
4. The raw log: `~/.claude/projects/-home-bob-git-PePPAR-Fix/dayplan/<DATE>.log`
   for the byte-level history including every discuss / amend /
   ack / status op.

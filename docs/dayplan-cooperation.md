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
- **IDs are camelCase slugs**: `--id <slug>-<by>` where `<slug>` is
  a short, descriptive camelCase keyword (8–14 chars typical) that
  summarizes the work.  Examples: `perOutAlarm-main`, `x2Canary-main`,
  `bdsMissBias-charlie`, `arpRuntime-charlie`.  No `I-` prefix, no
  timestamps.  Bob's parser is meat-based; slugs help him associate
  items with meaning at a glance.  If you omit `--id`, the tool
  falls back to the legacy `I-HHMMSS-<by>` form — usable but
  discouraged for new items.  Existing `I-NNNNNN` items keep their
  IDs (no rewriting history).
- **Slug collisions**: pick a more specific slug.  If two items
  legitimately share a topic, suffix with `-2`, `-3`, etc.
  (`bdsMissBias-charlie` and `bdsMissBias2-charlie`) or pick
  distinguishing terms (`bdsB2aBias-charlie` vs `bdsCnesGap-charlie`).
- **Cross-references**: in body or discuss, refer to other items by
  full ID (`per arpRuntime-charlie: …`).  Saves re-explaining context.
- **Asks**: end the body with explicit "ASKING <agent>: <action>"
  lines so reviewers know what they owe.
- **Branches**: when filing implementation work, name your branch
  `<by>/<slug>` (e.g., `bravo/wlBiasFlush`, `charlie/bdsMissBias`)
  so the dayplan ID and the branch name share the slug.  The
  `<by>/` prefix keeps each agent's branches grouped under
  `git branch` listings.

## Approvals — dayplan, not GitHub PR reviews

**Approval lives in the dayplan.  GitHub PR reviews don't work as a
canonical record in this setup.**

Why: every agent (`main`, `bravo`, `charlie`) shares one
authenticated identity to the GitHub CLI — Bob's user `bobvan`.
That has two consequences GitHub's review machinery wasn't designed
for:

1. **Self-approval is blocked.**  When an agent opens a PR via `gh
   pr create`, the PR author on GitHub is `bobvan`.  When another
   agent then tries `gh pr review --approve`, GitHub refuses
   because it sees the same user trying to approve their own PR.
2. **Comments are not reviews.**  Posting via `gh pr review
   --comment` (or `gh api repos/.../pulls/N/comments`) records a
   regular issue comment with `state=COMMENTED`, not a Review-type
   event with `state=APPROVED`.  Even if the comment body says
   "APPROVE" in plain text, GitHub's `reviewDecision` stays empty
   and the PR shows zero approvals on its summary screen.

So **the canonical approval record is `dayplan.py ack`**.  When
all reviewers on an item have ack'd, the item's derived status
flips to `AGREED` (per the ops table above).  That's the binding
agreement; GitHub PR review state is decorative for our workflow.

What to do instead:

- **As a reviewer**: post your substantive review as a `dayplan.py
  discuss --by <you> --id <slug>-<owner> --msg "..."` thread.
  When ready to approve, also run `dayplan.py ack --by <you> --id
  <slug>-<owner>` (optional `--note` for a one-line summary).  The
  `ack` op is what flips the item to `AGREED`.
- **Optionally cross-post on GitHub**: a comment summarising the
  review on the PR helps anyone reading the GitHub UI without
  dayplan access, but **don't expect or rely on its review
  state** — it'll show as a comment, not an approval.
- **As a PR author**: don't wait for GitHub's "Approved" indicator
  to merge.  Check the dayplan render for the item's status:
  `AGREED` (all reviewers ack'd) is the green light.  Bob will
  click merge based on the dayplan record, not GitHub's review
  count.
- **Don't run `gh pr review --approve`** as one agent for another
  agent's PR.  It'll fail (self-approval block) and the failed
  attempt clutters the PR's review history.

If we ever migrate to per-agent GitHub identities or a CI bot that
mirrors `dayplan ack` → `gh pr review --approve` from a separate
account, this section can shrink to a sentence and a pointer at
the migration commit.  Until then, dayplan is authoritative.

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
- **Don't `gh pr review --approve` someone else's PR.**  See the
  Approvals section above — GitHub's review machinery doesn't
  work for our shared-user setup.  Use `dayplan.py ack` instead.

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

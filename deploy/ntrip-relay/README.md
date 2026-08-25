# gt NTRIP re-casters — deployment

The str2str re-casters that serve `ssr.` / `obs.` / `eph.ntrip.VanValzah.Com`
on gt, plus the staged Onocoy transcode and the log-retention timer.

```sh
cp relay.env.example ~/opt/ntrip-relay/relay.env   # then fill in + chmod 0600
./install.sh
./check.sh
```

## Where things live, and why

| Thing | Home | Why there |
|---|---|---|
| Unit files | this directory (public repo) | No secrets in them — credentials are `${VAR}` refs resolved from an EnvironmentFile at runtime. |
| `relay.env` | `~/opt/ntrip-relay/relay.env`, **not in git** | Same rule as `ntrip.conf` and `timelab/antennas.json`: secrets stay out of *both* repos. A private repo is still a repo. |
| Stream logs | `~/opt/ntrip-relay/logs/` | Rotated daily by `::S=24`, pruned at `RETAIN_DAYS` by the timer. |
| Runbooks | `~/opt/ntrip-relay/*.md` | Operator-facing; `RETENTION.md` and `onocoy-activation-README.md`. |

**Answering the two questions this started from.** *Can these run as scripts
started by user-mode systemd?* They already do — `systemctl --user`, str2str
invoked straight from `ExecStart`, no wrapper script. The mechanism was never
the problem. *Should this live in timelab because credentials are involved?*
No: the credentials should not be in **any** repo, and once they are out, the
units carry nothing private, so they belong next to the code that consumes
them. timelab documents the *physical* lab — cables, antennas, surveys — and
these are software services.

## What actually went wrong, and what now catches it

On 2026-08-24 a rotation bug filled gt's `/home`. The fix touched three of the
four str2str units. The fourth was missed — not through carelessness, but
because **nothing enumerated them**: it was staged (disabled, never started),
so it had no disk usage to find and did not appear in `systemctl --user
list-units`. Only `list-unit-files` shows a disabled unit.

`check.sh` exists for that class of failure. Every check corresponds to a
defect that has already happened here:

- **ORPHAN** — any unit on the box whose `ExecStart` runs str2str but which is
  not in this directory. This is the fourth-unit bug, caught structurally.
- **ROTATION** — every `file://` output carries `::S=`. `::T` alone is RTKLIB's
  *time-tag* flag, not a swap interval (`src/stream.c:691`); without `::S=` the
  `%Y%m%d_%h` template expands once at process start and one file grows for
  ever. Two reached 4.4 GB.
- **REACH** — every log directory sits within the retention sweep's
  `-maxdepth`. Rotation alone is not enough: `logs/prod/` is a subdirectory the
  sweep could not see, and a live file's mtime is always ~now, so `-mtime +7`
  can never match it. That combination looks covered and is not.
- **DRIFT** — installed units match the repo.
- **CREDENTIALS** — `relay.env` present, mode 0600, required keys set.
- **SERVING** — each enabled mount actually answers.

The checks are verified by injecting each fault and confirming it is reported;
re-do that if you change the script.

### Two probe traps worth knowing

NTRIP v1 answers `SOURCETABLE 200 OK` / `ICY 200 OK`, which is **not valid
HTTP** — `curl` reports `000` for a perfectly healthy mount. And str2str's
`ntripc` **requires a `User-Agent` header**: without one it accepts the
connection and then answers nothing, which is indistinguishable from a dead
service. Probe with a raw socket *and* a User-Agent.

### Why `install.sh` copies instead of symlinking

`systemctl --user disable` **deletes** a unit file that is a symlink into a
repo — observed 2026-08-24, when `rtcm-archive-gt.service` vanished from
`~/.config/systemd/user` on a plain `disable --now`. Copies survive; `check.sh`
reports drift.

## Proposed next step — collapse four units into one template

Four near-identical units is the structural reason a fix can touch three of
them. A systemd template would make that impossible:

```
str2str-relay@.service          # one unit, one ExecStart, one place to fix
~/opt/ntrip-relay/ssr.env       # IN=, OUT=, MOUNT=, MSG=
~/opt/ntrip-relay/obs.env
~/opt/ntrip-relay/eph.env
~/opt/ntrip-relay/onocoy.env
```

`systemctl --user enable --now str2str-relay@ssr` and so on. Rotation, restart
policy, and logging are then defined once. `check.sh`'s ORPHAN check keeps
working unchanged, since it keys on "ExecStart runs str2str".

Not done here: the current units are live and working, and a refactor of
running relays wants a maintenance window and a re-validation of all four
mounts. It is a clean follow-up rather than something to fold into a bug fix.

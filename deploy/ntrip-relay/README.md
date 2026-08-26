# gt NTRIP re-casters — deployment

The str2str relays that serve `ssr.` / `obs.` / `eph.ntrip.VanValzah.Com` on gt,
plus the staged Onocoy transcode and the log-retention timer.

```sh
cp relay.env.example ~/opt/ntrip-relay/relay.env   # then fill in + chmod 0600
./install.sh
./check.sh
```

## One unit, four instances

There is a single `str2str-relay@.service`. Rotation, restart policy, and
logging are defined in it exactly once, so **a fix cannot reach three streams
and miss the fourth** — which is precisely what happened on 2026-08-24, when a
log-rotation fix touched three of four separate units and the fourth kept the
bug. It was invisible because it was staged: disabled, no disk usage, and
absent from `systemctl --user list-units`, which only shows *enabled* units.

| Instance | Upstream | Serves | Log |
|---|---|---|---|
| `@ssr` | `products.igs-ip.net/SSRA03IGS0` | `:2101/SSR` — `ssr.ntrip` | `logs/ssr_*.rtcm3` |
| `@obs` | `igs-ip.net/PTBB00DEU0` | `:2102/PTBB` — `obs.ntrip` | `logs/ptbb_*.rtcm3` |
| `@eph` | `igs-ip.net/BRUX00BEL0` | `:2103/BCEP` — `eph.ntrip` | `logs/bcep_*.rtcm3` |
| `@onocoy` | SXT-D mosaic-T SBF, `sxt-d.VanValzah.Com:28800` | `:2104/UFO1_MSM7` **staged** | `logs/prod/ufo1_msm7_*` |

```sh
systemctl --user status str2str-relay@ssr
journalctl --user -u str2str-relay@eph -f
```

## Where things live, and why

| Thing | Home | Why there |
|---|---|---|
| `str2str-relay@.service`, retention units | this directory (public repo) | No secrets — the unit names no credential at all. |
| `bin/str2str-relay-exec` | installed to `~/opt/ntrip-relay/bin/` | A stable path independent of any git worktree; worktrees move and switch branches. |
| `instances/<i>.env` | installed to `~/opt/ntrip-relay/instances/` | One stream's `IN` / `CRED` / `ARGS`. No secrets: `CRED` *names* a pair, it does not contain one. |
| `relay.env` | `~/opt/ntrip-relay/`, **not in git** | Same rule as `ntrip.conf` and `timelab/antennas.json`: secrets stay out of *both* repos. A private repo is still a repo. |
| Stream logs | `~/opt/ntrip-relay/logs/` | Rotated daily by `::S=24`, pruned at `RETAIN_DAYS`. |

These already ran under user-mode systemd before this refactor — the mechanism
was never the problem, the *inventory* was. And they do not belong in timelab:
once credentials are out of git the units carry nothing private, and timelab
documents the **physical** lab — cables, antennas, surveys — while these are
software services.

## Why there is a wrapper script

`ExecStart` runs `bin/str2str-relay-exec <instance>` rather than str2str
directly, because **systemd does not recursively expand `${VAR}` references
that appear inside a value loaded from an EnvironmentFile.** Measured
2026-08-24: with `ARGS='-in ntrip://${IGS_USER}:x@h/M …'`, argv arrives holding
the literal string `ntrip://${IGS_USER}:x@h/M`. str2str accepts credentials
only inside the URL, so something must assemble that URL once the credentials
are in scope. The wrapper does it with bash indirect expansion (`${!name}`) —
no `eval`, so a config file never becomes a code-execution surface.

Three more behaviours this design depends on, all measured the same day:

- An unquoted `$ARGS` **is** split at whitespace into separate argv entries.
- Quote *removal* happens too — but only for quotes the shell/systemd parses,
  which is why an inner quote inside a value would survive and reach str2str
  verbatim. `check.sh` rejects any quote character in `ARGS`.
- The instance files are **shell-sourced by the wrapper**, not parsed by
  systemd. So `ARGS` must be quoted as a whole, `$HOME` expands, and str2str's
  `%Y%m%d_%h` filename template is written with **single** `%` — the `%%`
  escaping that unit files require would be wrong here.

## check.sh

Every check corresponds to a defect that has already happened, and each was
verified by injecting the fault and confirming it is reported:

- **ORPHAN / legacy** — anything whose `ExecStart` runs str2str outside the
  template. This is the fourth-unit bug, caught structurally rather than by
  diligence. It matches the `ExecStart` line only; the retention unit mentions
  str2str in a comment, and a comment is not a process.
- **ROTATION** — every `file://` output carries `::S=`. `::T` alone is RTKLIB's
  *time-tag* flag (`src/stream.c:691`), not a swap interval; without `::S=` the
  filename template expands once at process start and one file grows for ever.
  Two reached 4.4 GB.
- **REACH** — every log directory sits inside the retention sweep's
  `-maxdepth`. Rotation alone is not enough: a live file's mtime is always
  ~now, so `-mtime +N` can never match it. That combination *looks* covered.
- **SPLIT** — no quote characters in `ARGS`, read from the raw file text
  (sourcing would strip the very quote we are looking for).
- **CREDCHAR** — no `@`, `:` or `/` in a credential; str2str takes them raw in
  the URL and the authority would parse ambiguously. The wrapper refuses too.
- **DRIFT** — installed units, wrapper, and instance files match the repo.
- **SERVING** — each enabled mount answers.

### Three probe traps

NTRIP v1 answers `SOURCETABLE 200 OK` / `ICY 200 OK`, which is **not valid
HTTP** — `curl` reports `000` for a healthy mount. str2str's `ntripc`
**requires a `User-Agent`**; without one it accepts the connection and answers
nothing, indistinguishable from a dead service. And it services accepted
connections on an internal cycle, so a **single probe is unreliable**: ten
back-to-back probes all answered, while probes spaced 2 s apart alternated
answer / no-answer. `check.sh` retries three times — do not simplify that away.

### Why install.sh copies instead of symlinking

`systemctl --user disable` **deletes** a unit file that is a symlink into a
repo — observed 2026-08-24, when `rtcm-archive-gt.service` vanished from
`~/.config/systemd/user` on a plain `disable --now`. Copies survive, and
`check.sh` reports drift.

## Adding a stream

Write `instances/<name>.env` with `IN=`, optional `CRED=`, and a quoted `ARGS=`;
run `./install.sh`; `systemctl --user enable --now str2str-relay@<name>`. Add
the port to the `PORT` map in `check.sh` so it gets probed. Nothing else — the
unit is already correct, which is the entire point.

## Checking the Onocoy station — `onocoy-status.py`

onocoy has a Public API (spec: <https://api-hub.onocoy.com/docs>, Swagger 2.0,
host `api-hub.onocoy.com`).  It is not linked from the miner docs, and their
support docs say no API exists — it does.

```sh
./onocoy-status.py            # last 7 days
./onocoy-status.py --days 30
```

**Why bother**, given the web console exists: onocoy grades every contributing
station, so these are an *independent* assessment of our observation quality,
computed by a party with no stake in our results — `phase_rms_avg`,
`code_rms_avg`, `cs_rate_avg` (cycle slips), `latency_avg`, `sky_vis_avg`,
`epoch_rate_avg`.  That is exactly the third-party consistency check Box 3 was
for, and unlike the console it is machine-readable.  It is also the only
programmatic view of whether our station is up — onocoy provides no way to
*announce* downtime, so this is the closest thing to monitoring.

Auth is HTTP Basic with the **station** credential (`ONOCOY_CRED` /
`ONOCOY_PASS` from `relay.env`).  Endpoints:

| Endpoint | Credential needed |
|---|---|
| `/api/v1/my_stats/stations` | station — our daily quality stats |
| `/api/v1/my_stats/data_usage` | station |
| `/api/v1/stations/list` | **client** credential + active onocoyStream plan |
| `/api/v1/stations/status` | **client** credential + active onocoyStream plan |

**A station credential only works once the station "has a successful validation
within the last 7 days."**  Before that the API answers `403 access denied`,
which is *not* a misconfiguration — validation takes 24-36 h after first data.
The two failures are distinguishable and the tool says which:

    401 invalid credentials  -> ONOCOY_CRED/ONOCOY_PASS wrong
    403 access denied        -> credential fine, station not yet validated

Statistics reach back at most 3 months, and none exist before 2025-12-22.

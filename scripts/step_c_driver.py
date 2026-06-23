#!/usr/bin/env python3
"""STEP C interleaved A/B driver (midTauTrackingResidual I-102153).

Cycles a host through the realization arms in interleaved ~60-min segments
(time-of-day common-mode), logging each segment to its own CSV.  Reset-free.

DRY-RUN BY DEFAULT — prints the exact ssh/launch commands without executing,
because the lab is handed to Main until takedown+return.  Pass --go to run
(only when the hosts are back and assigned to you).

Encodes the lab gotchas (see reference_lab_engine_restart_gotchas):
  - STOP = `sudo pkill -f "[p]eppar"` as the ONLY content of its ssh command
    (any other literal 'peppar' in the same command self-kills the shell → 255).
  - root-owned /tmp logs need sudo to archive.
  - launch via the production wrapper; NO ssh -n, NO timeout.

Arms (flags appended to the host's normal invocation):
  baseline : (none)
  coast    : --coast-cap --coast-cap-k-sigma 0.5
  stiffQ   : --sigma-do-freq-override 0.0003
  both     : --coast-cap --coast-cap-k-sigma 0.5 --sigma-do-freq-override 0.0003

Usage:
  step_c_driver.py --host clkPoC3 --config config/clkpoc3.toml \
      --arms baseline,coast,stiffQ,both --cycles 2 --seg-min 60 [--go]
  step_c_driver.py --host MadHat --config config/madhat.toml \
      --arms baseline,coast --cycles 2 --seg-min 60 [--go]    # reference-floor control
"""
import argparse
import shlex
import sys

ARM_FLAGS = {
    "baseline": [],
    "coast":    ["--coast-cap", "--coast-cap-k-sigma", "0.5"],
    "stiffQ":   ["--sigma-do-freq-override", "0.0003"],
    "both":     ["--coast-cap", "--coast-cap-k-sigma", "0.5",
                 "--sigma-do-freq-override", "0.0003"],
}
BASE = ["./scripts/peppar-fix", "--host-config", "{config}", "--no-antposest",
        "--qerr-latest-chi"]


def launch_cmd(host, config, arm, seg_tag):
    logs = (f"--ticc-log /tmp/ticc-{seg_tag}.csv "
            f"--servo-log /tmp/servo-{seg_tag}.csv "
            f"--extint-log /tmp/extint-{seg_tag}.csv")
    inner = " ".join(BASE).format(config=config) + " " \
        + " ".join(ARM_FLAGS[arm]) + " " + logs \
        + f" > /tmp/run-{seg_tag}.log 2>&1 </dev/null & disown"
    return (f"ssh {host} \"sudo bash -c 'cd /home/bob/peppar-fix && "
            f"nohup {inner}'\"")


def stop_cmd(host):
    # ONLY content — bracket-trick + no other 'peppar' literal (avoids self-kill).
    return f"ssh {host} 'sudo pkill -f \"[p]eppar\"; sleep 6; echo stopped'"


def archive_cmd(host, seg_tag):
    return (f"ssh {host} 'D=/home/bob/stepc-{seg_tag}; mkdir -p $D; "
            f"sudo mv /tmp/ticc-{seg_tag}.csv /tmp/servo-{seg_tag}.csv "
            f"/tmp/extint-{seg_tag}.csv /tmp/run-{seg_tag}.log $D/ 2>/dev/null; "
            f"ls $D'")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--config", required=True)
    ap.add_argument("--arms", required=True, help="comma list of arm names")
    ap.add_argument("--cycles", type=int, default=2)
    ap.add_argument("--seg-min", type=int, default=60)
    ap.add_argument("--go", action="store_true",
                    help="actually run (default: dry-run print only)")
    a = ap.parse_args()
    arms = a.arms.split(",")
    for arm in arms:
        if arm not in ARM_FLAGS:
            print(f"unknown arm {arm!r}; valid: {list(ARM_FLAGS)}"); return 2

    plan = []
    for cyc in range(1, a.cycles + 1):
        for arm in arms:
            tag = f"{arm}-c{cyc}"
            plan.append((arm, tag))

    print(f"# STEP C driver — host={a.host} arms={arms} cycles={a.cycles} "
          f"seg={a.seg_min}min  ({'LIVE' if a.go else 'DRY-RUN'})")
    print(f"# total {len(plan)} segments ≈ {len(plan)*a.seg_min/60:.1f} h\n")
    for arm, tag in plan:
        print(f"## segment {tag}")
        print("  " + stop_cmd(a.host))
        print("  " + launch_cmd(a.host, a.config, arm, tag))
        print(f"  # … wait {a.seg_min} min (engine bootstraps + runs the segment) …")
        print("  " + archive_cmd(a.host, tag))
        print()
    print("# final:")
    print("  " + stop_cmd(a.host))

    if not a.go:
        print("\n# DRY-RUN — nothing executed. Re-run with --go when the host "
              "is back and assigned to you. Pull per-arm logs to "
              "gt:~/gt/stepc-<host>-<date>/ then: tdev_step_c.py <that dir>")
        return 0
    print("\n!!! --go set but live execution is intentionally not implemented "
          "in this prep artifact; wire the wait/ssh loop only when running "
          "interactively on a returned lab host. !!!")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Deacon topology patrol: SSH to lab hosts, check connected devices,
compare against timelab/resources.json, and flag discrepancies.

Usage:
    python3 topology_patrol.py                  # Check all hosts
    python3 topology_patrol.py --host TimeHat   # Check one host
    python3 topology_patrol.py --json           # Output JSON report
    python3 topology_patrol.py --update-topology /path/to/topology.md

Run frequency: on-demand + every few hours via Deacon cron.
Per decision pf-zx9.
"""

import argparse
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

RESOURCES_PATH = Path(__file__).resolve().parent.parent / "timelab" / "resources.json"

# Device glob patterns to probe on each host.
DEVICE_GLOBS = ["/dev/gnss*", "/dev/ticc*", "/dev/ptp*", "/dev/gps*"]


def load_resources(path=None):
    """Load resources.json inventory."""
    p = Path(path) if path else RESOURCES_PATH
    with open(p) as f:
        return json.load(f)


def ssh_ls_devices(host_access, timeout=10):
    """SSH to a host and list matching device nodes.

    Returns (dict of category -> list[str], error_string or None).
    Categories: gnss, ticc, ptp, gps.
    """
    # Build a single ls command for all globs. ls returns non-zero if some
    # globs don't match, so we use 'ls -1d ... 2>/dev/null; true' to suppress
    # missing-glob errors while still capturing what exists.
    globs_str = " ".join(DEVICE_GLOBS)
    cmd = f"ls -1d {globs_str} 2>/dev/null; true"

    # Extract hostname from access string (e.g. "ssh TimeHat" -> "TimeHat")
    parts = host_access.split()
    ssh_target = parts[-1]  # last token is the target

    try:
        result = subprocess.run(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
             ssh_target, cmd],
            capture_output=True, text=True, timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return {}, f"SSH timeout after {timeout}s"
    except OSError as e:
        return {}, f"SSH failed: {e}"

    if result.returncode not in (0, 1):
        # returncode 255 = SSH connection failure
        stderr = result.stderr.strip()
        if result.returncode == 255:
            return {}, f"SSH connection failed: {stderr}"
        return {}, f"SSH returned {result.returncode}: {stderr}"

    devices = {"gnss": [], "ticc": [], "ptp": [], "gps": []}
    for line in result.stdout.strip().splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("/dev/gnss"):
            devices["gnss"].append(line)
        elif line.startswith("/dev/ticc"):
            devices["ticc"].append(line)
        elif line.startswith("/dev/ptp"):
            devices["ptp"].append(line)
        elif line.startswith("/dev/gps"):
            devices["gps"].append(line)
    return devices, None


def check_host(hostname, host_cfg):
    """Check a single host against its expected_devices.

    Returns dict with keys: hostname, reachable, error, expected, actual,
    missing, unexpected.
    """
    result = {
        "hostname": hostname,
        "reachable": False,
        "error": None,
        "expected": host_cfg["expected_devices"],
        "actual": {},
        "missing": [],
        "unexpected": [],
    }

    actual, err = ssh_ls_devices(host_cfg["access"])
    if err:
        result["error"] = err
        return result

    result["reachable"] = True
    result["actual"] = actual

    # Flatten expected devices into a set.
    expected_set = set()
    for devs in host_cfg["expected_devices"].values():
        expected_set.update(devs)

    # Flatten actual devices into a set.
    actual_set = set()
    for devs in actual.values():
        actual_set.update(devs)

    result["missing"] = sorted(expected_set - actual_set)
    result["unexpected"] = sorted(actual_set - expected_set)

    return result


def run_patrol(resources, hosts=None):
    """Run patrol on all (or selected) hosts. Returns list of host results."""
    results = []
    target_hosts = hosts if hosts else list(resources["hosts"].keys())

    for hostname in target_hosts:
        if hostname not in resources["hosts"]:
            results.append({
                "hostname": hostname,
                "reachable": False,
                "error": f"Host '{hostname}' not in resources.json",
                "expected": {},
                "actual": {},
                "missing": [],
                "unexpected": [],
            })
            continue

        host_cfg = resources["hosts"][hostname]
        result = check_host(hostname, host_cfg)
        results.append(result)

    return results


def format_report(results):
    """Format patrol results as human-readable text."""
    lines = []
    now = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    lines.append(f"Topology Patrol Report — {now}")
    lines.append("=" * 60)

    any_discrepancy = False

    for r in results:
        status = "OK" if r["reachable"] and not r["missing"] and not r["unexpected"] else "ISSUE"
        if r["error"]:
            status = "UNREACHABLE"
        lines.append(f"\n{r['hostname']}: {status}")

        if r["error"]:
            lines.append(f"  Error: {r['error']}")
            any_discrepancy = True
            continue

        if r["missing"]:
            any_discrepancy = True
            for dev in r["missing"]:
                lines.append(f"  MISSING: {dev}")

        if r["unexpected"]:
            any_discrepancy = True
            for dev in r["unexpected"]:
                lines.append(f"  UNEXPECTED: {dev}")

        if not r["missing"] and not r["unexpected"]:
            # Show what we found
            all_devs = []
            for devs in r["actual"].values():
                all_devs.extend(devs)
            if all_devs:
                lines.append(f"  Devices: {', '.join(sorted(all_devs))}")
            else:
                lines.append("  Devices: (none in glob pattern)")

    lines.append("\n" + "=" * 60)
    if any_discrepancy:
        lines.append("DISCREPANCIES FOUND — review and update resources.json")
    else:
        lines.append("All hosts match expected topology.")

    return "\n".join(lines)


def append_topology_change(topology_path, results):
    """Append a change log entry to topology.md if discrepancies found."""
    discrepancies = [r for r in results
                     if r["missing"] or r["unexpected"] or r["error"]]
    if not discrepancies:
        return False

    now = datetime.now(timezone.utc)
    date_str = now.strftime("%Y-%m-%d")
    time_str = now.strftime("%H:%M UTC")

    lines = [f"\n### {date_str} — Topology patrol discrepancy ({time_str})\n"]
    lines.append("Automated patrol detected the following discrepancies:\n")

    for r in discrepancies:
        if r["error"]:
            lines.append(f"- **{r['hostname']}**: unreachable ({r['error']})")
        if r["missing"]:
            lines.append(f"- **{r['hostname']}**: missing devices: {', '.join(r['missing'])}")
        if r["unexpected"]:
            lines.append(f"- **{r['hostname']}**: unexpected devices: {', '.join(r['unexpected'])}")

    lines.append("")

    p = Path(topology_path)
    if not p.exists():
        print(f"Warning: {topology_path} not found, skipping topology update",
              file=sys.stderr)
        return False

    with open(p, "a") as f:
        f.write("\n".join(lines))

    print(f"Appended change log entry to {topology_path}", file=sys.stderr)
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Deacon topology patrol: check lab hardware state")
    parser.add_argument("--resources", type=str, default=None,
                        help="Path to resources.json (default: timelab/resources.json)")
    parser.add_argument("--host", type=str, action="append", dest="hosts",
                        help="Check specific host(s) only (repeatable)")
    parser.add_argument("--json", action="store_true",
                        help="Output results as JSON")
    parser.add_argument("--update-topology", type=str, metavar="PATH",
                        help="Append discrepancies to topology.md at PATH")
    parser.add_argument("--timeout", type=int, default=10,
                        help="SSH timeout per host in seconds (default: 10)")
    args = parser.parse_args()

    resources = load_resources(args.resources)
    results = run_patrol(resources, args.hosts)

    if args.json:
        print(json.dumps(results, indent=2))
    else:
        print(format_report(results))

    if args.update_topology:
        append_topology_change(args.update_topology, results)

    # Exit 1 if any discrepancies found (useful for CI/cron alerting).
    has_issues = any(r["missing"] or r["unexpected"] or r["error"]
                     for r in results)
    sys.exit(1 if has_issues else 0)


if __name__ == "__main__":
    main()

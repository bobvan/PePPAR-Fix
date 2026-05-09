#!/usr/bin/env python3
"""Resolve per-host PePPAR Fix config and emit shell defaults."""

import argparse
import os
import shlex
import socket
import sys
import tomllib
from pathlib import Path


KEYS = (
    "serial",
    "baud",
    "ubx_port",
    "port_type",
    "receiver",
    "ptp_profile",
    "ptp_dev",
    "ntrip_conf",
    "eph_mount",
    "ssr_mount",
    "ssr_ntrip_conf",
    "ssr_bias_mount",
    "ssr_bias_ntrip_conf",
    "known_pos",
    "systems",
    "duration",
    "log",
    "phase_step_bias_ns",
    "ticc_landing_horizon_s",
    "ticc_settled_threshold_ns",
    "ticc_settled_deadband_ns",
    "ticc_settled_count",
    "do_label",
    "do_type",
    "dac_bus",
    "dac_addr",
    "dac_bits",
    "dac_center_code",
    "dac_ppb_per_code",
    "dac_max_ppb",
    "dac_type",
    "dac_gain",
    "tadd_gpio",
    "tadd_hold_s",
    "ticc_port",
)


def candidate_paths(repo_root: Path, explicit: str | None, hostname: str) -> list[Path]:
    if explicit:
        return [Path(explicit)]
    short = hostname.split(".", 1)[0].lower()
    names = [short]
    if short == "oxco":
        names.append("ocxo")  # legacy typo fallback
    paths = [repo_root / "config" / f"{name}.toml" for name in names]
    paths.append(Path("/etc/peppar-fix/config.toml"))
    return paths


def load_config(path: Path) -> dict:
    with path.open("rb") as f:
        data = tomllib.load(f)
    return data.get("peppar", {})


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root", required=True)
    ap.add_argument("--hostname", default=socket.gethostname())
    ap.add_argument("--explicit", default=os.environ.get("PEPPAR_HOST_CONFIG"))
    args = ap.parse_args()

    repo_root = Path(args.repo_root)
    for path in candidate_paths(repo_root, args.explicit, args.hostname):
        if not path.exists():
            continue
        cfg = load_config(path)
        print(f"HOSTCFG_PATH={shlex.quote(str(path))}")
        if cfg.get("ubx_port") is None and cfg.get("port_type") is not None:
            cfg["ubx_port"] = cfg["port_type"]
        for key in KEYS:
            value = cfg.get(key)
            if value is None:
                continue
            name = f"HOSTCFG_{key.upper()}"
            print(f"{name}={shlex.quote(str(value))}")
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

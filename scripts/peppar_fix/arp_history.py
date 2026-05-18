"""ARP history accumulator: per-receiver daily PRIDE solutions →
running ECEF mean + cross-day spread.

Per I-013307-main: each receiver capturing RINEX has its own
`state/arp/<uid>/history.jsonl` (append-only, one daily-solution
record per line).  Running mean across the most recent N days
plus the cross-day spread gives σ_arp — the single number the
I-013239 orchestrator gates on.

mount_sn partitions the history: when an antenna is moved, the
orchestrator increments mount_sn, and post-move records live in
their own append window.  running_mean() considers only one
mount_sn at a time so old solutions don't pollute new mount geometry.

This module is intentionally schema-shy about state/arp/<uid>.json
itself — that's I-013239's territory.  We expose a `RunningArp`
dataclass that the state-file writer can consume once the schema
is frozen.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

from peppar_fix.pride_pos_reader import PrideSolution


# --- defaults --------------------------------------------------------- #

# Quality filter knobs (Bob 2026-05-04: rejected runs DON'T pollute the
# running mean).  Rationale:
#   - sig0 ~ 4.4 m for a healthy 24h static ARP at IGS-class receivers
#     (sample at timelab/surveys/data/pride/pos_2026116_ufo1).  A
#     rough 2× cap of 10 m catches catastrophically bad days
#     (gross-error contamination) while not rejecting weather-noisy
#     but solvable days.
#   - n_obs ≥ 10 000 — at 30 s sampling for 24h that's ~2880 obs per
#     SV minimum, but PRIDE pools across all observed SVs so 10k is
#     a comfortable lower bound that any real F9T capture clears by
#     2-10×.
DEFAULT_MAX_SIG0_M = 10.0
DEFAULT_MIN_N_OBS = 10_000

# Running-mean window default.  Per I-013307-main "recompute running
# mean ECEF + cross-day std across last N days (N=7 default)."
DEFAULT_N_DAYS = 7


# --- dataclasses ------------------------------------------------------ #


@dataclass
class RunningArp:
    """Aggregate ARP estimate from a window of daily solutions.

    ecef_m         — mean (X, Y, Z) ECEF, meters
    sigma_xyz_m    — per-axis cross-day standard deviation (m)
    sigma_3d_m     — RSS of sigma_xyz_m
    count          — number of solutions in the window
    mount_sn       — partition this aggregate covers
    oldest_mjd     — earliest solution included
    newest_mjd     — latest solution included
    n_total        — total records in history (may exceed count)
    """
    ecef_m: tuple[float, float, float]
    sigma_xyz_m: tuple[float, float, float]
    sigma_3d_m: float
    count: int
    mount_sn: int
    oldest_mjd: float
    newest_mjd: float
    n_total: int


# --- public API ------------------------------------------------------- #


def apply_quality_filter(
    sol: PrideSolution, *,
    max_sig0_m: float = DEFAULT_MAX_SIG0_M,
    min_n_obs: int = DEFAULT_MIN_N_OBS,
) -> bool:
    """Return True iff the solution is healthy enough for the running mean.

    Two cheap, discriminating gates:
      - sig0_m below max_sig0_m (formal precision of the daily fit)
      - n_obs above min_n_obs (defends against truncated runs)

    Callers should log rejected solutions but still archive them in
    history.jsonl with `quality_ok=False` so post-hoc audits can see
    why σ_arp didn't move on a given day.
    """
    if not math.isfinite(sol.sig0_m) or sol.sig0_m <= 0:
        return False
    if sol.sig0_m > max_sig0_m:
        return False
    if sol.n_obs < min_n_obs:
        return False
    return True


def append_solution(
    history_path: str | Path,
    sol: PrideSolution, *,
    mount_sn: int = 0,
    quality_ok: bool | None = None,
    ingested_at: datetime | None = None,
) -> dict:
    """Append one PrideSolution → history.jsonl as a JSON record.

    Creates the parent directory + file on first call.

    quality_ok: if None, compute via apply_quality_filter() with
    defaults.  Pass explicit True/False when the caller wants to
    use custom thresholds and record the verdict.

    Returns the record (a dict) that was written, for caller logging.
    """
    p = Path(history_path)
    p.parent.mkdir(parents=True, exist_ok=True)
    if quality_ok is None:
        quality_ok = apply_quality_filter(sol)
    rec = {
        "mjd": sol.mjd,
        "date": sol.date_iso,
        "ecef_m": list(sol.ecef_m),
        "sigma_xyz_m": list(sol.sigma_xyz_m),
        "sig0_m": sol.sig0_m,
        "n_obs": sol.n_obs,
        "name": sol.name,
        "mode": sol.mode,
        "interval_s": sol.interval_s,
        "mask_deg": sol.mask_deg,
        "receiver_type": sol.receiver_type,
        "antenna_type": sol.antenna_type,
        "products": sol.products,
        "ambiguity_enabled": sol.ambiguity_enabled,
        "ambiguity_fixing": sol.ambiguity_fixing,
        "mount_sn": int(mount_sn),
        "quality_ok": bool(quality_ok),
        "ingested_at": (ingested_at
                        or datetime.now(timezone.utc)).isoformat(),
        "src_pos_path": sol.src_path,
    }
    with open(p, "a") as f:
        f.write(json.dumps(rec, separators=(",", ":")) + "\n")
    return rec


def read_history(history_path: str | Path) -> list[dict]:
    """Return all records in history.jsonl as a list of dicts.

    Skips blank lines and silently ignores malformed JSON lines
    (logs to stderr would be heavy here; rotation tooling can audit
    separately).  Returns [] if the file doesn't exist.
    """
    p = Path(history_path)
    if not p.is_file():
        return []
    out: list[dict] = []
    with open(p) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return out


def running_mean(
    history_path: str | Path, *,
    n_days: int = DEFAULT_N_DAYS,
    mount_sn: int = 0,
    require_quality_ok: bool = True,
) -> RunningArp | None:
    """Compute the running ECEF mean + cross-day std over the last
    n_days of solutions on the given mount_sn partition.

    Returns None when history is empty or no solutions match the
    mount_sn + quality filters.

    Std uses sample formula (ddof=1) when count ≥ 2; for a single
    solution sigma_xyz_m is the formal sigma from that day's PRIDE
    fit (PRIDE's Sig0·sqrt(Sx) per axis).
    """
    records = read_history(history_path)
    pool = [r for r in records
            if r.get("mount_sn", 0) == mount_sn
            and (not require_quality_ok or r.get("quality_ok", False))]
    pool.sort(key=lambda r: r.get("mjd", 0.0))
    if not pool:
        return None
    window = pool[-n_days:]
    n = len(window)

    sx = sy = sz = 0.0
    for r in window:
        x, y, z = r["ecef_m"]
        sx += x; sy += y; sz += z
    mean_ecef = (sx / n, sy / n, sz / n)

    if n >= 2:
        # Sample-std (ddof=1) per axis across the window.
        var_x = var_y = var_z = 0.0
        for r in window:
            x, y, z = r["ecef_m"]
            var_x += (x - mean_ecef[0]) ** 2
            var_y += (y - mean_ecef[1]) ** 2
            var_z += (z - mean_ecef[2]) ** 2
        denom = n - 1
        sigma_xyz = (math.sqrt(var_x / denom),
                     math.sqrt(var_y / denom),
                     math.sqrt(var_z / denom))
    else:
        # Single solution: fall back to its formal per-axis σ.
        s = window[0].get("sigma_xyz_m", [0.0, 0.0, 0.0])
        sigma_xyz = (float(s[0]), float(s[1]), float(s[2]))

    sigma_3d = math.sqrt(sum(s * s for s in sigma_xyz))
    return RunningArp(
        ecef_m=mean_ecef,
        sigma_xyz_m=sigma_xyz,
        sigma_3d_m=sigma_3d,
        count=n,
        mount_sn=mount_sn,
        oldest_mjd=float(window[0]["mjd"]),
        newest_mjd=float(window[-1]["mjd"]),
        n_total=len(records),
    )

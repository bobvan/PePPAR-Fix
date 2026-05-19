"""peppar-survey --pride backend: PRIDE-PPP-AR over captured RINEX.

Per the peppar-survey contract in scripts/peppar_survey.py docstring:
input is RINEX, output is an atomic write of state/positions/<uid>.survey.toml.

This module orchestrates pdp3 (PRIDE-PPP-AR) over one or more daily
RINEX observation files, archives each solution to a per-receiver
history.jsonl, recomputes the running mean across the most recent
N days, and writes the resulting ECEF + sigma_3d to .survey.toml.

The pdp3 invocation, pos-file parsing, history append, and running-
mean math each have a single owner module already; this module is
the thin glue that wires them in the contract order.

Operator usage lives in scripts/peppar_survey.py.  This module is
imported by that CLI and shouldn't normally be invoked directly.
"""
from __future__ import annotations

import logging
import os
import re
import shutil
import subprocess
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence

from peppar_fix.arp_history import (
    DEFAULT_N_DAYS, RunningArp,
    append_solution, apply_quality_filter, running_mean,
)
from peppar_fix.position_state import (
    DEFAULT_POSITIONS_DIR, PositionState,
    save_survey_state, utc_now_iso,
)
from peppar_fix.pride_pos_reader import (
    PrideParseError, PrideSolution, parse_pos,
)


log = logging.getLogger("peppar-survey.pride")


# pdp3 binary path.  Installed under $HOME/.PRIDE_PPPAR_BIN by the
# upstream installer.  Override via PEPPAR_PDP3_BIN env var for tests
# or non-standard installs.
DEFAULT_PDP3 = os.environ.get(
    "PEPPAR_PDP3_BIN",
    os.path.expanduser("~/.PRIDE_PPPAR_BIN/pdp3"),
)

# pdp3 takes a long time to run on a full daily RINEX (1-5 min typical
# for an F9T 24h capture, longer on slow hosts).  Cap at 30 min so a
# stuck job doesn't hang the whole sweep indefinitely.
DEFAULT_PDP3_TIMEOUT_S = 30 * 60

# Default GNSS systems to try, in order.  GREC = GPS+GAL+BDS+GLO,
# preferred when products + observations support it.  GR is the
# robust fallback when BDS or GAL cause pdp3 to fail.
DEFAULT_SYS_ATTEMPTS = ("GREC", "GR")

# Standard receiver-state arp history location.  Per arp_history.py
# convention: state/arp/<uid>/history.jsonl.
def default_history_path(uid: str, history_dir: str | None = None) -> Path:
    base = Path(history_dir) if history_dir else Path("state/arp")
    return base / uid / "history.jsonl"


@dataclass
class PrideRunResult:
    """One pdp3 attempt result.  pos_path is None on failure."""
    obs_file: Path
    sys_attempted: str
    returncode: int
    pos_path: Path | None
    log_path: Path | None
    error: str = ""


_DOY_FROM_NAME_RE = re.compile(r"-(\d{4})(\d{3})\b")  # MadHat-2026133.obs


def doy_from_obs_name(path: Path) -> tuple[int, int] | None:
    """Extract (year, doy) from a filename like MadHat-2026133.obs.

    Returns None when the filename doesn't match the expected pattern;
    callers fall back to reading the RINEX header in that case.
    """
    m = _DOY_FROM_NAME_RE.search(path.name)
    if m is None:
        return None
    return int(m.group(1)), int(m.group(2))


def site_from_obs_header(path: Path) -> str | None:
    """Read MARKER NAME (4-char site code) from the RINEX header.

    Returns lower-case 4-char marker, or None when not present.
    pdp3 lowercases site names, so callers should pass through lowercase.
    """
    try:
        with open(path) as f:
            for line in f:
                if "MARKER NAME" in line:
                    name = line[:60].strip().split()[0] if line[:60].strip() else None
                    if name:
                        return name.lower()[:4]
                if "END OF HEADER" in line:
                    break
    except OSError as e:
        log.warning("can't read RINEX header for site name: %s", e)
    return None


def expected_pos_name(year: int, doy: int, site: str) -> str:
    """The basename pdp3 writes for a static-mode solution."""
    return f"pos_{year:04d}{doy:03d}_{site}"


def expected_log_name(year: int, doy: int, site: str) -> str:
    """The basename pdp3 writes for the per-day log."""
    return f"log_{year:04d}{doy:03d}_{site}"


def invoke_pdp3(
    obs_file: Path,
    work_dir: Path,
    sys_str: str,
    *,
    pdp3_bin: str = DEFAULT_PDP3,
    timeout_s: int = DEFAULT_PDP3_TIMEOUT_S,
    extra_args: Sequence[str] = (),
) -> PrideRunResult:
    """Run pdp3 once on obs_file in work_dir with the given -sys string.

    Static mode (-m S).  Returns a PrideRunResult capturing the
    expected pos_*/log_* paths under work_dir and pdp3's exit code.

    pos_path is set only if pdp3 returned 0 AND the pos file exists.
    """
    work_dir.mkdir(parents=True, exist_ok=True)
    # Copy the obs file into work_dir if not already there so pdp3's
    # auxiliary output files (orb/clk caches, .res, .kin) don't pile up
    # next to the original.
    obs_local = work_dir / obs_file.name
    if obs_local.resolve() != obs_file.resolve():
        shutil.copy2(obs_file, obs_local)

    cmd = [pdp3_bin, "-m", "S", "-sys", sys_str] + list(extra_args) + [obs_local.name]
    log.info("Running pdp3 in %s: %s", work_dir, " ".join(cmd))
    try:
        proc = subprocess.run(
            cmd, cwd=str(work_dir),
            capture_output=True, text=True,
            timeout=timeout_s,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return PrideRunResult(
            obs_file=obs_file, sys_attempted=sys_str,
            returncode=-1, pos_path=None, log_path=None,
            error=f"pdp3 timed out after {timeout_s}s",
        )
    except FileNotFoundError as e:
        return PrideRunResult(
            obs_file=obs_file, sys_attempted=sys_str,
            returncode=-1, pos_path=None, log_path=None,
            error=f"pdp3 binary not found: {e}",
        )

    # Find the pos file by glob in work_dir — pdp3 names it
    # pos_YYYYDDD_<site>.  We don't pre-compute the expected name
    # because the site comes from the RINEX header inside pdp3 and
    # may not match our doy_from_obs_name() prediction perfectly.
    pos_candidates = sorted(work_dir.glob("pos_*"))
    log_candidates = sorted(work_dir.glob("log_*"))
    # Most recent pos file wins (in case work_dir holds prior attempts).
    pos_path = max(pos_candidates, key=lambda p: p.stat().st_mtime) \
        if pos_candidates else None
    log_path = max(log_candidates, key=lambda p: p.stat().st_mtime) \
        if log_candidates else None

    if proc.returncode != 0:
        return PrideRunResult(
            obs_file=obs_file, sys_attempted=sys_str,
            returncode=proc.returncode,
            pos_path=None, log_path=log_path,
            error=f"pdp3 exit={proc.returncode}: {proc.stderr.strip()[:500]}",
        )
    if pos_path is None or not pos_path.is_file():
        return PrideRunResult(
            obs_file=obs_file, sys_attempted=sys_str,
            returncode=proc.returncode,
            pos_path=None, log_path=log_path,
            error="pdp3 returned 0 but no pos_* file present in work_dir",
        )
    return PrideRunResult(
        obs_file=obs_file, sys_attempted=sys_str,
        returncode=proc.returncode,
        pos_path=pos_path, log_path=log_path,
    )


def process_one_obs(
    obs_file: Path,
    work_dir: Path,
    *,
    sys_attempts: Sequence[str] = DEFAULT_SYS_ATTEMPTS,
    pdp3_bin: str = DEFAULT_PDP3,
    timeout_s: int = DEFAULT_PDP3_TIMEOUT_S,
    pdp3_runner=invoke_pdp3,  # injectable for tests
) -> tuple[PrideSolution | None, PrideRunResult | None]:
    """Run pdp3 on one obs file with fallback through sys_attempts.

    Returns (solution, last_result).  solution is None when every
    sys attempt failed.  last_result is the most-recent attempt for
    diagnostic logging (may have a pos_path even on failure if pdp3
    partially completed).
    """
    last_result: PrideRunResult | None = None
    for sys_str in sys_attempts:
        # Each attempt gets its own subdirectory so a failed run's
        # partial files don't pollute the next attempt.
        attempt_dir = work_dir / f"sys_{sys_str}"
        result = pdp3_runner(
            obs_file, attempt_dir, sys_str,
            pdp3_bin=pdp3_bin, timeout_s=timeout_s,
        )
        last_result = result
        if result.pos_path is None:
            log.warning("pdp3 -sys %s on %s failed: %s",
                        sys_str, obs_file.name, result.error)
            continue
        try:
            sol = parse_pos(result.pos_path)
            return sol, result
        except PrideParseError as e:
            log.warning("pdp3 -sys %s on %s: pos file unparseable: %s",
                        sys_str, obs_file.name, e)
            continue
    return None, last_result


def write_survey_from_running(
    running: RunningArp,
    uid: str,
    *,
    positions_dir: str | None = None,
    source_label: str = "peppar-survey --pride",
    dry_run: bool = False,
) -> tuple[PositionState, str]:
    """Convert RunningArp → PositionState(kind='survey') and write.

    Returns (state, path).  When dry_run=True, returns the path that
    WOULD have been written but does not write.
    """
    state = PositionState(
        mount_sn=running.mount_sn,
        ecef_m=tuple(running.ecef_m),  # type: ignore[arg-type]
        sigma_m=running.sigma_3d_m,
        updated=utc_now_iso(),
        source=source_label,
        kind="survey",
        extra={
            "pride_window_count": running.count,
            "pride_window_n_total": running.n_total,
            "pride_sigma_x_m": float(running.sigma_xyz_m[0]),
            "pride_sigma_y_m": float(running.sigma_xyz_m[1]),
            "pride_sigma_z_m": float(running.sigma_xyz_m[2]),
            "pride_oldest_mjd": float(running.oldest_mjd),
            "pride_newest_mjd": float(running.newest_mjd),
        },
    )
    d = positions_dir or DEFAULT_POSITIONS_DIR
    path = os.path.join(d, f"{uid}.survey.toml")
    if dry_run:
        log.info("DRY RUN — would write %s (σ_3d=%.4fm from %d-day mean)",
                 path, running.sigma_3d_m, running.count)
        return state, path
    save_survey_state(state, uid, positions_dir=positions_dir)
    return state, path


def run_pride_backend(
    obs_files: Sequence[Path],
    work_dir: Path,
    receiver_uid: str,
    *,
    positions_dir: str | None = None,
    history_dir: str | None = None,
    mount_sn: int = 0,
    sys_attempts: Sequence[str] = DEFAULT_SYS_ATTEMPTS,
    n_days: int = DEFAULT_N_DAYS,
    pdp3_bin: str = DEFAULT_PDP3,
    timeout_s: int = DEFAULT_PDP3_TIMEOUT_S,
    dry_run: bool = False,
    pdp3_runner=invoke_pdp3,
    source_label: str = "peppar-survey --pride",
) -> int:
    """Run PRIDE over each RINEX, archive solutions, write survey.toml.

    Exit code (shell convention):
      0  one or more solutions ingested and a running mean computed.
      1  no input obs files.
      2  every obs file failed pdp3 (or no quality-passing solution).
      3  pdp3 binary missing or not executable.
    """
    if not obs_files:
        log.error("no RINEX obs files supplied")
        return 1

    if not dry_run and not os.path.isfile(pdp3_bin):
        log.error("pdp3 binary not found at %s — set PEPPAR_PDP3_BIN "
                  "or install PRIDE-PPP-AR", pdp3_bin)
        return 3

    work_dir = Path(work_dir)
    history_path = default_history_path(receiver_uid, history_dir)

    n_solved = 0
    n_quality_ok = 0
    n_failed = 0
    for obs in sorted(Path(p) for p in obs_files):
        log.info("--- pdp3: %s ---", obs.name)
        sol, last = process_one_obs(
            obs, work_dir / obs.stem,
            sys_attempts=sys_attempts,
            pdp3_bin=pdp3_bin,
            timeout_s=timeout_s,
            pdp3_runner=pdp3_runner,
        )
        if sol is None:
            n_failed += 1
            log.warning("FAILED %s: %s",
                        obs.name, last.error if last else "no attempt completed")
            continue
        quality_ok = apply_quality_filter(sol)
        n_solved += 1
        if quality_ok:
            n_quality_ok += 1
        if dry_run:
            log.info("  DRY RUN — would append to %s "
                     "(sig0=%.3fm σ_3d=%.4fm n_obs=%d quality_ok=%s)",
                     history_path, sol.sig0_m, sol.sigma_3d_m,
                     sol.n_obs, quality_ok)
            continue
        rec = append_solution(
            history_path, sol,
            mount_sn=mount_sn,
            quality_ok=quality_ok,
        )
        log.info("  appended %s → %s (σ_3d=%.4fm quality_ok=%s)",
                 sol.date_iso, history_path, sol.sigma_3d_m, quality_ok)

    log.info("pdp3 sweep complete: %d solved (%d quality_ok), %d failed",
             n_solved, n_quality_ok, n_failed)

    if n_solved == 0:
        log.error("no PRIDE solutions produced — survey.toml not written")
        return 2

    running = running_mean(
        history_path,
        n_days=n_days,
        mount_sn=mount_sn,
        require_quality_ok=True,
    )
    if running is None:
        log.error("running_mean returned None (no quality_ok solutions "
                  "in mount_sn=%d partition) — survey.toml not written",
                  mount_sn)
        return 2

    state, path = write_survey_from_running(
        running, receiver_uid,
        positions_dir=positions_dir,
        source_label=source_label,
        dry_run=dry_run,
    )
    log.info("survey result: %s mount_sn=%d ECEF=(%.4f, %.4f, %.4f) "
             "σ_3d=%.4fm (window %d days, %d records total)",
             path, state.mount_sn,
             state.ecef_m[0], state.ecef_m[1], state.ecef_m[2],
             state.sigma_m, running.count, running.n_total)
    return 0

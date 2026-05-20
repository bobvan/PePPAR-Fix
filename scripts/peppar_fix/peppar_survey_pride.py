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

import gzip
import logging
import os
import re
import shutil
import subprocess
from dataclasses import dataclass
from datetime import date, datetime, timedelta, timezone
from pathlib import Path
from typing import Sequence

from peppar_fix.arp_history import (
    DEFAULT_MAX_SIG0_M, DEFAULT_MIN_N_OBS, DEFAULT_N_DAYS, RunningArp,
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


def brdm_filename(year: int, doy: int) -> str:
    """pdp3.sh's canonical broadcast-nav name for a given day.

    pdp3.sh:1578 builds this as ``brdm${doy_s}0.${ymd_s:2:2}p``.  If
    a file by this name is already present in pdp3's rinex_dir, the
    download chain (pdp3.sh:2110-2167) skips and uses the local copy
    directly — which is the integration point ``--brdm-source`` uses
    to feed a multi-GNSS broadcast nav from outside PRIDE's normal
    IGS download path.
    """
    return f"brdm{doy:03d}0.{year % 100:02d}p"


# ─── WUM products (orbit / clock / bias / ERP) ───────────────────── #
#
# pdp3.sh's PrepareProducts builds an URL ladder (bdspride → IGN → gnsswhu
# → bdspride WCC → gnsswhu IGS2R03FIN) for each of orbit/clock/bias/ERP.
# In practice the gnsswhu /pub/whu/phasebias/{year}/ tree is the most
# reliable mirror for current-day RTS variants, but pdp3's chain often
# trips on flaky FTPS or unwritten RAP products and never reaches the
# RTS fall-through.  Staging the products into pdp3's $product_cmn_dir
# (= work_dir.parent / "product" / "common") under their canonical
# names lets CopyOrDownloadProduct's USECACHE branch find them and skip
# the download chain entirely.
#
# Filename conventions (per pdp3.sh URL lists at lines 2283-2285,
# 2575-2577, 2861-2863, 2924; basename of those URLs is the canonical
# form pdp3 looks up):
#
#   orbit:  WUM0MGX{VARIANT}_{YYYY}{DOY}0000_01D_05M_ORB.SP3
#   clock:  WUM0MGX{VARIANT}_{YYYY}{DOY}0000_01D_{INT}_CLK.CLK
#   bias:   WUM0MGX{VARIANT}_{YYYY}{DOY}0000_01D_{INT}_OSB.BIA
#   erp:    WUM0MGX{VARIANT}_{YYYY}{DOY}0000_01D_01D_ERP.ERP
#
# where VARIANT ∈ {RAP, RTS, NRT, FIN}.  Sample intervals differ
# between RAP and RTS (clock: 30S RAP vs 05S RTS; bias: 01D RAP vs
# 05M RTS).  See _WUM_VARIANT_INTERVALS below for the per-variant
# value pdp3 uses, mirrored from those URL lists.
#
# NRT (near-real-time, 2-day file) lives at
# /pub/gps/products/mgex/<gpsweek>/WUM0MGXNRT_*_02D_*.gz — distinct
# enough that we don't try to merge it into the staging path; operators
# pre-fetching it should rename to the same canonical scheme.

WUM_KINDS = ("orbit", "clock", "bias", "erp")
WUM_VARIANTS = ("RAP", "RTS", "FIN")  # tried in this order when resolving

# pdp3.sh's CopyOrDownloadProduct strips the URL .gz extension and uses
# the leftover as the cached-name target.  These are the .gz suffixes
# pdp3 expects in $product_cmn_dir (without leading directory).
_WUM_FILENAME_TEMPLATES = {
    "orbit": "WUM0MGX{variant}_{year}{doy:03d}0000_01D_05M_ORB.SP3",
    # Clock has variant-dependent sample interval.  Mirrors pdp3.sh
    # line 2487 (RAP 30S) vs line 2540 (RTS 05S).
    "clock_RAP": "WUM0MGX{variant}_{year}{doy:03d}0000_01D_30S_CLK.CLK",
    "clock_RTS": "WUM0MGX{variant}_{year}{doy:03d}0000_01D_05S_CLK.CLK",
    "clock_FIN": "WUM0MGX{variant}_{year}{doy:03d}0000_01D_30S_CLK.CLK",
    # Bias varies similarly: pdp3.sh line 2863 (RAP 01D) vs line 2924
    # (RTS 05M).
    "bias_RAP": "WUM0MGX{variant}_{year}{doy:03d}0000_01D_01D_OSB.BIA",
    "bias_RTS": "WUM0MGX{variant}_{year}{doy:03d}0000_01D_05M_OSB.BIA",
    "bias_FIN": "WUM0MGX{variant}_{year}{doy:03d}0000_01D_01D_OSB.BIA",
    "erp": "WUM0MGX{variant}_{year}{doy:03d}0000_01D_01D_ERP.ERP",
}

# gnsswhu paths discovered 2026-05-20 (wumProductGnsswhuFallback bead):
# the RTS variant lives under /pub/whu/phasebias/{year}/{bias|clock|orbit}/,
# FIN lives under /pub/gps/products/mgex/{gpsweek}/.  These mirror
# pdp3.sh's own URL templates (lines 2285, 2344, 2577, 2638, 2863, 2924)
# but with the value set we actually observed working when pdp3's chain
# fails through to its tail.
_GNSSWHU_BASE = "ftp://igs.gnsswhu.cn"
_GNSSWHU_RTS_SUBDIR_BY_KIND = {
    "orbit": "orbit",
    "clock": "clock",
    "bias": "bias",
    "erp": "orbit",  # ERP files live under orbit/ in the RTS tree
}


def wum_filename(kind: str, variant: str, year: int, doy: int,
                 *, gzipped: bool = False) -> str:
    """Canonical WUM filename for (kind, variant, year, doy).

    ``kind`` ∈ ``WUM_KINDS``, ``variant`` ∈ ``WUM_VARIANTS``.
    Returns the decompressed name by default; set ``gzipped=True``
    for the ``.gz`` form pdp3 downloads.
    """
    if kind not in WUM_KINDS:
        raise ValueError(f"unknown WUM kind: {kind}")
    if variant not in WUM_VARIANTS:
        raise ValueError(f"unknown WUM variant: {variant}")
    tmpl_key = f"{kind}_{variant}" if f"{kind}_{variant}" in _WUM_FILENAME_TEMPLATES else kind
    tmpl = _WUM_FILENAME_TEMPLATES[tmpl_key]
    name = tmpl.format(variant=variant, year=year, doy=doy)
    return f"{name}.gz" if gzipped else name


def ydoy_to_gpsweek(year: int, doy: int) -> int:
    """Convert (year, day-of-year) → GPS week number.

    GPS week 0 starts 1980-01-06 (Sunday).  Used to compute the
    /pub/gps/products/mgex/{gpsweek}/ path for FIN products.
    """
    d = date(year, 1, 1) + timedelta(days=doy - 1)
    days_since_gps_epoch = (d - date(1980, 1, 6)).days
    return days_since_gps_epoch // 7


def gnsswhu_url(kind: str, variant: str, year: int, doy: int) -> str:
    """gnsswhu FTP URL for a WUM product (kind, variant, year, doy).

    RTS lives under /pub/whu/phasebias/{year}/{subdir}/.
    RAP/FIN live under /pub/gps/products/mgex/{gpsweek}/.

    Returns the .gz URL.  See bead wumProductGnsswhuFallback-charlie
    for the empirically-verified paths.
    """
    fname = wum_filename(kind, variant, year, doy, gzipped=True)
    if variant == "RTS":
        sub = _GNSSWHU_RTS_SUBDIR_BY_KIND[kind]
        return f"{_GNSSWHU_BASE}/pub/whu/phasebias/{year}/{sub}/{fname}"
    gpsweek = ydoy_to_gpsweek(year, doy)
    return f"{_GNSSWHU_BASE}/pub/gps/products/mgex/{gpsweek}/{fname}"


def resolve_wum_source(wum_source: Path | str | None,
                       year: int, doy: int) -> dict[str, Path]:
    """Resolve a wum_source directory to {kind: filepath} for (year, doy).

    Walks ``wum_source`` looking for WUM0MGX*_<YYYY><DOY>0000_*_*.gz (or
    decompressed) files for each kind, preferring earlier variants in
    ``WUM_VARIANTS`` (RAP > RTS > FIN).  Caller stages whatever was
    found; missing kinds fall through to pdp3's normal download.

    Returns ``{}`` when ``wum_source`` is ``None``, not a directory,
    or contains no matching files.
    """
    if wum_source is None:
        return {}
    p = Path(wum_source).expanduser()
    if not p.is_dir():
        log.warning("--wum-source %s is not a directory — ignoring "
                    "(no WUM products will be staged)", p)
        return {}

    found: dict[str, Path] = {}
    for kind in WUM_KINDS:
        for variant in WUM_VARIANTS:
            cand_plain = p / wum_filename(kind, variant, year, doy)
            cand_gz = p / wum_filename(kind, variant, year, doy, gzipped=True)
            if cand_plain.is_file():
                found[kind] = cand_plain
                break
            if cand_gz.is_file():
                found[kind] = cand_gz
                break
    return found


def stage_wum_products(files: dict[str, Path], product_cmn_dir: Path) -> int:
    """Copy + gunzip resolved WUM files into pdp3's product/common dir.

    pdp3.sh's CopyOrDownloadProduct (line 3477) looks for the
    decompressed name first under USECACHE=YES (the default), so we
    drop the gunzipped form there.  ``.gz`` inputs are streamed through
    gzip; already-decompressed inputs are copied verbatim.

    Returns count of files staged.
    """
    product_cmn_dir.mkdir(parents=True, exist_ok=True)
    n = 0
    for kind, src in files.items():
        # Always store decompressed; pdp3's USECACHE path prefers it.
        if src.name.endswith(".gz"):
            target = product_cmn_dir / src.name[:-3]
            with gzip.open(src, "rb") as fin, open(target, "wb") as fout:
                shutil.copyfileobj(fin, fout)
        else:
            target = product_cmn_dir / src.name
            if target.resolve() != src.resolve():
                shutil.copy2(src, target)
        log.info("staged WUM %s %s → %s", kind, src.name, target.name)
        n += 1
    return n


def resolve_brdm_source(brdm_source: Path | str | None,
                        year: int, doy: int) -> Path | None:
    """Resolve ``brdm_source`` to a concrete file for (year, doy).

    Accepts file-or-directory paths:
      - ``None``: returns ``None`` (caller falls back to pdp3's
        normal download behavior).
      - File path: returned as-is — operator's responsibility to pass
        a file matching the obs day.
      - Directory path: looks up ``brdm{doy:03d}0.{yy:02d}p`` inside.
        Returns ``None`` (with a warning) when the per-day file isn't
        present.

    Made a top-level helper rather than inlined into ``invoke_pdp3``
    so tests can exercise the path resolution independently of the
    pdp3 subprocess.
    """
    if brdm_source is None:
        return None
    p = Path(brdm_source).expanduser()
    if p.is_file():
        return p
    if p.is_dir():
        candidate = p / brdm_filename(year, doy)
        if candidate.is_file():
            return candidate
        log.warning("--brdm-source dir %s has no %s — falling through to "
                    "pdp3's normal download", p, brdm_filename(year, doy))
        return None
    log.warning("--brdm-source %s is neither file nor directory — "
                "falling through to pdp3's normal download", p)
    return None


def invoke_pdp3(
    obs_file: Path,
    work_dir: Path,
    sys_str: str,
    *,
    pdp3_bin: str = DEFAULT_PDP3,
    timeout_s: int = DEFAULT_PDP3_TIMEOUT_S,
    extra_args: Sequence[str] = (),
    brdm_source: Path | str | None = None,
    wum_source: Path | str | None = None,
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

    # Repair any partial-epoch corruption in the work-dir copy before
    # pdp3 runs.  Files captured by the engine BEFORE the writer's
    # repair-on-reopen fix can contain time tags whose declared NN
    # exceeds the actual rows that followed (engine killed mid-epoch).
    # PRIDE's rdrnxoi3 trips on these.  Repair rewrites NN in place to
    # match actual row count; data preserved, original file untouched
    # (we only modify the work_dir copy).  Idempotent.
    from peppar_fix.rinex_writer import repair_partial_epochs
    n_repaired = repair_partial_epochs(obs_local)
    if n_repaired:
        log.info("Repaired %d partial epoch(s) in %s before pdp3",
                 n_repaired, obs_local.name)

    # Stage a pre-fetched multi-GNSS broadcast nav file when given.
    # pdp3.sh's IGS-MGEX download chain (gnsswhu/IGN/DLR) sometimes
    # falls through to GPS-only nav; without GAL/BDS ephemeris pdp3's
    # elevation() returns dist=-1 for every E**/C** SV → DEL_BADRANGE.
    # Dropping the operator-supplied brdm into work_dir under pdp3's
    # canonical name lets pdp3.sh:2110-2117 find it and skip the
    # download.  See prideMultiGnssBrdm-charlie / prideBadRangeDiagnostic-main.
    if brdm_source is not None or wum_source is not None:
        ydoy = doy_from_obs_name(obs_file)
        if ydoy is None:
            log.warning("can't derive year/doy from %s — --brdm-source / "
                        "--wum-source ignored for this obs file",
                        obs_file.name)
        else:
            year, doy = ydoy
            if brdm_source is not None:
                src = resolve_brdm_source(brdm_source, year, doy)
                if src is not None:
                    target = work_dir / brdm_filename(year, doy)
                    if target.resolve() != src.resolve():
                        shutil.copy2(src, target)
                    log.info("staged broadcast nav %s → %s",
                             src, target.name)
            if wum_source is not None:
                wum_files = resolve_wum_source(wum_source, year, doy)
                if wum_files:
                    # pdp3.sh resolves product_dir as $(rdlk ..)/product/
                    # from its cwd (= work_dir).  So product/common is
                    # the sibling of work_dir, shared across sys_attempts.
                    product_cmn_dir = work_dir.parent / "product" / "common"
                    n = stage_wum_products(wum_files, product_cmn_dir)
                    log.info("staged %d WUM product(s) for %d/%d → %s",
                             n, year, doy, product_cmn_dir)
                else:
                    log.info("no WUM products matched %d/%d in %s",
                             year, doy, wum_source)

    cmd = [pdp3_bin, "-m", "S", "-sys", sys_str] + list(extra_args) + [obs_local.name]
    log.info("Running pdp3 in %s: %s", work_dir, " ".join(cmd))
    # pdp3 (the bash wrapper) does `$(dirname $(which pdp3))/config_template`
    # to find its config template.  That `which` returns empty when pdp3
    # is invoked by absolute path without being on PATH, producing a
    # silent dirname-of-empty → "/config_template" error.  Ensure the
    # bin dir is on PATH so `which` succeeds.
    env = os.environ.copy()
    pdp3_dir = os.path.dirname(os.path.abspath(pdp3_bin))
    if pdp3_dir and pdp3_dir not in env.get("PATH", "").split(os.pathsep):
        env["PATH"] = pdp3_dir + os.pathsep + env.get("PATH", "")
    try:
        proc = subprocess.run(
            cmd, cwd=str(work_dir),
            capture_output=True, text=True,
            timeout=timeout_s,
            check=False,
            env=env,
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

    # Find the pos file by recursive glob — pdp3 writes outputs to
    # a nested $work_dir/<year>/<doy>/pos_YYYYDDD_<site> tree, not
    # the top-level work dir.  Recursive search means we don't have
    # to predict the site name (which comes from the RINEX header
    # MARKER NAME) or the year/doy directory layout.
    pos_candidates = sorted(work_dir.rglob("pos_*"))
    log_candidates = sorted(work_dir.rglob("log_*"))
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
    brdm_source: Path | str | None = None,
    wum_source: Path | str | None = None,
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
            brdm_source=brdm_source,
            wum_source=wum_source,
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
    max_sig0_m: float = DEFAULT_MAX_SIG0_M,
    min_n_obs: int = DEFAULT_MIN_N_OBS,
    pdp3_bin: str = DEFAULT_PDP3,
    timeout_s: int = DEFAULT_PDP3_TIMEOUT_S,
    dry_run: bool = False,
    pdp3_runner=invoke_pdp3,
    source_label: str = "peppar-survey --pride",
    brdm_source: Path | str | None = None,
    wum_source: Path | str | None = None,
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
            brdm_source=brdm_source,
            wum_source=wum_source,
        )
        if sol is None:
            n_failed += 1
            log.warning("FAILED %s: %s",
                        obs.name, last.error if last else "no attempt completed")
            continue
        quality_ok = apply_quality_filter(
            sol, max_sig0_m=max_sig0_m, min_n_obs=min_n_obs)
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

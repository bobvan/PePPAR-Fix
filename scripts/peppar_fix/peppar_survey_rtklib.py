"""peppar-survey --rtklib backend: RTKLIB rnx2rtkp over captured RINEX.

Sibling to peppar_survey_pride.py.  Both produce the same survey
artifact (state/positions/<uid>.survey.toml) — they just call out to
different external processors.

Why this exists (2026-05-20, surveyRtklibBackend-main + closure of
wumOsbBiaDownload-charlie): PRIDE-batch can produce sub-cm positions
when the WUM phase-bias product covers the receiver's tracked
signals.  Our F9T-20B lab fleet runs an L1+L5 GPS profile and WUM
doesn't publish GPS L5 phase biases — so PRIDE-batch can't reach
sub-cm on our captures.  RTKLIB's rnx2rtkp can run PPP-static (no
base, uses broadcast or precise products) or RTK-static (against a
nearby CORS base RINEX) without the same L5 coverage dependency.
That makes RTKLIB the practical survey path for our F9T-L5 fleet
and the keystone consumer for peer-bootstrap-from-RTCM (see Main's
bead body).

CORS source default: NOAA CORS publishes daily RINEX 3 obs files at
geodesy.noaa.gov/corsdata.  No auth, ~1-day latency.  Operator
picks the station via --cors-station.

Two modes:
  - PPP-static (no base): rnx2rtkp -p 7 -m S -k <ppp.conf> rover.obs nav
  - RTK-static (CORS base): rnx2rtkp -p 1 -m S -k <rtk.conf>
    rover.obs base.obs nav

Both write a .pos file with one row per processed epoch.  Static
mode converges over the arc; the converged position is the last
"valid" row (Q flag in {1, 2, 6} = fix / float / ppp).
"""
from __future__ import annotations

import logging
import os
import re
import shutil
import subprocess
import urllib.request
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence

from peppar_fix.arp_history import (
    DEFAULT_MAX_SIG0_M, DEFAULT_MIN_N_OBS, DEFAULT_N_DAYS, RunningArp,
    append_solution, apply_quality_filter, running_mean,
)
from peppar_fix.geo_frames import CANONICAL_REALIZATION, Frame
from peppar_fix.position_state import (
    DEFAULT_POSITIONS_DIR, PositionState,
    decimal_year_from_mjd, save_survey_state, utc_now_iso,
)
from peppar_fix.pride_pos_reader import PrideSolution


log = logging.getLogger("peppar-survey.rtklib")


# rnx2rtkp binary path.  rtklibexplorer fork preferred for F9-era
# signal handling; upstream tomojitakasu RTKLIB also works.  Default
# location: built from source, installed to /usr/local/bin or
# $HOME/.local/bin.  Override via PEPPAR_RNX2RTKP_BIN.
DEFAULT_RNX2RTKP = os.environ.get(
    "PEPPAR_RNX2RTKP_BIN",
    shutil.which("rnx2rtkp") or "/usr/local/bin/rnx2rtkp",
)

# rnx2rtkp is fast — ~30s for a 24h capture on a Pi — but cap at 10
# min to be safe against pathological data.
DEFAULT_RNX2RTKP_TIMEOUT_S = 10 * 60

# NOAA CORS daily RINEX URL pattern.  Path: /<year>/<doy>/<lower-station>/<lower-station><doy>0.<yy>o.gz
DEFAULT_CORS_URL_TMPL = (
    "https://geodesy.noaa.gov/corsdata/rinex/{year}/{doy:03d}/"
    "{station_lc}/{station_lc}{doy:03d}0.{yy:02d}o.gz"
)


# Reusing the regex from peppar_survey_pride for consistency.
_DOY_FROM_NAME_RE = re.compile(r"-(\d{4})(\d{3})\b")


def doy_from_obs_name(path: Path) -> tuple[int, int] | None:
    """Extract (year, doy) from a filename like MadHat-2026133.obs.

    Same convention as peppar_survey_pride.doy_from_obs_name.
    """
    m = _DOY_FROM_NAME_RE.search(path.name)
    if m is None:
        return None
    return int(m.group(1)), int(m.group(2))


@dataclass
class RtklibRunResult:
    """One rnx2rtkp invocation result.  pos_path is None on failure."""
    obs_file: Path
    mode: str  # "ppp" or "rtk"
    returncode: int
    pos_path: Path | None
    error: str = ""


@dataclass
class RtklibPosEpoch:
    """One row from a rnx2rtkp .pos output."""
    utc: datetime
    lat: float           # degrees
    lon: float           # degrees
    height: float        # meters
    q: int               # quality: 1=fix, 2=float, 5=single, 6=ppp
    n_sats: int
    sdn: float           # 1-sigma north (m)
    sde: float           # 1-sigma east (m)
    sdu: float           # 1-sigma up (m)


@dataclass
class RtklibSolution:
    """Aggregated survey result from a single rnx2rtkp run.

    Built by collapsing the per-epoch .pos rows: position = mean over
    the FINAL converged segment (last N epochs where Q is in the
    accepted set), sigma = RMS of the position residuals from that
    mean.
    """
    first_epoch: datetime   # UTC, first kept epoch
    lat: float
    lon: float
    height: float
    sigma_3d_m: float
    sig0_m: float          # alias for sigma_3d_m
    n_obs: int             # number of kept epochs
    mode: str              # "rtklib_ppp" or "rtklib_cors"
    cors_base: str = ""    # populated for rtklib_cors mode
    extras: dict = field(default_factory=dict)


# Modified Julian Day reference: 1858-11-17 00:00:00 UTC
_MJD_EPOCH = datetime(1858, 11, 17, tzinfo=timezone.utc)


def _datetime_to_mjd(dt: datetime) -> float:
    """UTC datetime → Modified Julian Day (with fractional day)."""
    delta = dt - _MJD_EPOCH
    return delta.days + (delta.seconds + delta.microseconds * 1e-6) / 86400.0


def rtklib_to_pride_solution(
    rtk_sol: RtklibSolution,
    *,
    receiver_type: str = "",
    antenna_type: str = "",
    src_path: str = "",
) -> PrideSolution:
    """Convert RtklibSolution → PrideSolution so the same
    arp_history + running_mean pipeline that consumes PRIDE outputs
    can consume RTKLIB outputs.

    Field mapping notes:
      - ecef_m: lla_to_ecef(lat, lon, height)
      - sigma_xyz_m: 3D σ split equally across axes (RTKLIB-aggregate
        gives 3D residual RMS; the per-axis breakdown is not preserved
        through the static-mode aggregation).  Downstream consumers
        either use sigma_3d_m directly or treat sigma_xyz_m as an
        upper-bound-per-axis estimate.
      - mode: 'Static'  (matches PrideSolution convention)
      - source_processors / extras: carry the RTKLIB-specific
        provenance + the per-epoch metrics via raw_header.
    """
    from solve_pseudorange import lla_to_ecef
    ecef = lla_to_ecef(rtk_sol.lat, rtk_sol.lon, rtk_sol.height)
    sigma_per_axis = rtk_sol.sigma_3d_m / (3 ** 0.5)
    raw_header = {
        "rtklib_mode": rtk_sol.mode,
        "rtklib_cors_base": rtk_sol.cors_base,
    }
    # Surface the per-epoch quality distribution + counts for audit
    for k, v in rtk_sol.extras.items():
        raw_header[k] = str(v)
    return PrideSolution(
        name=Path(src_path).stem[:4].lower() if src_path else "ufo1",
        mjd=_datetime_to_mjd(rtk_sol.first_epoch),
        ecef_m=tuple(ecef),  # type: ignore[arg-type]
        cofactor_xyz=(sigma_per_axis ** 2,) * 3,
        cofactor_off=(0.0, 0.0, 0.0),
        sig0_m=rtk_sol.sig0_m,
        sigma_xyz_m=(sigma_per_axis,) * 3,
        n_obs=rtk_sol.n_obs,
        mode="Static",
        first_epoch=rtk_sol.first_epoch,
        last_epoch=rtk_sol.first_epoch,
        receiver_type=receiver_type,
        antenna_type=antenna_type,
        raw_header=raw_header,
        src_path=src_path,
    )


# ── NOAA CORS RINEX fetch ──────────────────────────────────────── #


def fetch_cors_rinex(
    station: str,
    year: int,
    doy: int,
    work_dir: Path,
    *,
    url_template: str = DEFAULT_CORS_URL_TMPL,
    fetcher=urllib.request.urlretrieve,
) -> Path | None:
    """Download a CORS station's daily RINEX 3 obs file to work_dir.

    Returns the path to the gunzipped .obs file, or None on failure.
    Caches: if the target file already exists in work_dir, skips the
    download and returns the cached path.

    The ``fetcher`` parameter is dependency-injected so tests can
    mock the HTTP layer without going to NOAA.

    NOAA CORS RINEX availability lags real-time by ~1 day.  Pass
    a recent-but-not-current day for best results.
    """
    work_dir.mkdir(parents=True, exist_ok=True)
    yy = year % 100
    station_lc = station.lower()[:4]
    local_name = f"{station_lc}{doy:03d}0.{yy:02d}o"
    local_path = work_dir / local_name
    if local_path.exists() and local_path.stat().st_size > 0:
        log.info("CORS RINEX cached: %s", local_path)
        return local_path

    url = url_template.format(
        year=year, doy=doy, yy=yy, station_lc=station_lc)
    gz_path = work_dir / (local_name + ".gz")
    log.info("Fetching CORS RINEX: %s", url)
    try:
        fetcher(url, str(gz_path))
    except Exception as e:
        log.warning("CORS fetch failed (%s): %s", url, e)
        return None

    # Gunzip in-place.  Use a streaming decompressor to handle any
    # size cleanly.
    import gzip
    try:
        with gzip.open(gz_path, "rb") as gz, open(local_path, "wb") as out:
            shutil.copyfileobj(gz, out)
    except OSError as e:
        log.warning("CORS RINEX gunzip failed: %s", e)
        return None
    gz_path.unlink()
    return local_path


# ── .pos file parsing ──────────────────────────────────────────── #


# Sample .pos line (lat/lon/height format):
# 2024/01/15 00:00:00.000  LAT  LON  HEIGHT  Q  ns  sdn  sde  sdu ...
# (RTKLIB writes degrees + meters; Q=1 fix, 2 float, 5 single, 6 ppp;
# placeholders here to dodge the repo's secrets-pattern guard)
_POS_LINE_RE = re.compile(
    r"^\s*(\d{4})/(\d{2})/(\d{2})\s+(\d{2}):(\d{2}):([\d.]+)\s+"
    r"([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+"
    r"(\d+)\s+(\d+)\s+"
    r"([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)"
)


def parse_pos(path: Path | str) -> list[RtklibPosEpoch]:
    """Parse a RTKLIB .pos file into per-epoch RtklibPosEpoch rows.

    Skips header lines (starting with '%') and any rows that don't
    match the lat/lon/height format.  Returns rows in file order.
    """
    path = Path(path)
    rows: list[RtklibPosEpoch] = []
    if not path.exists():
        return rows
    with open(path) as f:
        for line in f:
            if line.startswith("%") or not line.strip():
                continue
            m = _POS_LINE_RE.match(line)
            if m is None:
                continue
            y, mo, d, h, mi, sec = (m.group(i) for i in range(1, 7))
            try:
                ts = datetime(
                    int(y), int(mo), int(d), int(h), int(mi),
                    int(float(sec)),
                    int((float(sec) - int(float(sec))) * 1e6),
                    tzinfo=timezone.utc,
                )
                rows.append(RtklibPosEpoch(
                    utc=ts,
                    lat=float(m.group(7)),
                    lon=float(m.group(8)),
                    height=float(m.group(9)),
                    q=int(m.group(10)),
                    n_sats=int(m.group(11)),
                    sdn=float(m.group(12)),
                    sde=float(m.group(13)),
                    sdu=float(m.group(14)),
                ))
            except (ValueError, OverflowError) as e:
                log.debug("pos parse skip at %r: %s", line[:60].rstrip(), e)
    return rows


# ── rnx2rtkp config templates ──────────────────────────────────── #


# navsys is a bit-flag sum.  Per rnx2rtkp source (rtkpos.c):
#   1=GPS, 2=SBAS, 4=GLO, 8=GAL, 16=QZS, 32=BDS, 64=IRN.
# GPS+GAL+BDS = 1+8+32 = 41.  The prior value 53 = 1+4+16+32 silently
# included GLO+QZS and dropped GAL (PiFace lost ~half its strong SVs).
# Caught empirically 2026-05-20 by branch-main during PR #48 lab test.
_PPP_STATIC_CONFIG = """\
# Generated by peppar_survey_rtklib — PPP-static
pos1-posmode       =ppp-static
pos1-frequency     =l1+l2+l5
pos1-soltype       =forward
pos1-elmask        =7
pos1-snrmask_r     =off
pos1-snrmask_b     =off
pos1-dynamics      =off
pos1-tidecorr      =on
pos1-ionoopt       =dual-freq
pos1-tropopt       =est-ztd
pos1-sateph        =brdc
pos1-posopt1       =on
pos1-posopt2       =on
pos1-posopt3       =on
pos1-posopt4       =on
pos1-posopt5       =off
pos1-posopt6       =off
pos1-exclsats      =
pos1-navsys        =41        # GPS(1) + GAL(8) + BDS(32) = 41
pos2-armode        =continuous
pos2-gloarmode     =off
pos2-arthres       =3
pos2-arlockcnt     =0
pos2-arelmask      =0
pos2-arminfix      =10
pos2-armaxiter     =1
pos2-aroutcnt      =5
out-solformat      =llh
out-outhead        =on
out-outopt         =on
out-timesys        =gpst
out-timeform       =hms
out-timendec       =3
out-degform        =deg
out-fieldsep       =
"""

_RTK_STATIC_CONFIG = """\
# Generated by peppar_survey_rtklib — RTK-static
pos1-posmode       =static
pos1-frequency     =l1+l2+l5
pos1-soltype       =forward
pos1-elmask        =7
pos1-snrmask_r     =off
pos1-snrmask_b     =off
pos1-dynamics      =off
pos1-tidecorr      =on
pos1-ionoopt       =dual-freq
pos1-tropopt       =est-ztd
pos1-sateph        =brdc
pos1-navsys        =41        # GPS(1) + GAL(8) + BDS(32) = 41
pos2-armode        =fix-and-hold
pos2-gloarmode     =off
pos2-arthres       =3
pos2-arlockcnt     =0
pos2-arelmask      =15
pos2-arminfix      =10
out-solformat      =llh
out-outhead        =on
out-outopt         =on
out-timesys        =gpst
out-timeform       =hms
out-timendec       =3
out-degform        =deg
"""


def write_config(work_dir: Path, mode: str) -> Path:
    """Materialize the rnx2rtkp config file for the given mode.

    Returns the path to the written config file.
    """
    config_text = {"ppp": _PPP_STATIC_CONFIG,
                   "rtk": _RTK_STATIC_CONFIG}[mode]
    path = work_dir / f"rnx2rtkp-{mode}.conf"
    with open(path, "w") as f:
        f.write(config_text)
    return path


# ── rnx2rtkp subprocess invocation ─────────────────────────────── #


def invoke_rnx2rtkp(
    obs_file: Path,
    work_dir: Path,
    mode: str,
    *,
    base_obs: Path | None = None,
    nav_file: Path | None = None,
    rnx2rtkp_bin: str = DEFAULT_RNX2RTKP,
    timeout_s: int = DEFAULT_RNX2RTKP_TIMEOUT_S,
    extra_args: Sequence[str] = (),
) -> RtklibRunResult:
    """Run rnx2rtkp once on obs_file in work_dir.

    mode: "ppp" (no base, uses broadcast eph + WUM products if
    available) or "rtk" (requires base_obs).

    Returns an RtklibRunResult.  pos_path is set only if rnx2rtkp
    returned 0 AND the .pos file exists with non-zero size.
    """
    if mode not in ("ppp", "rtk"):
        return RtklibRunResult(
            obs_file=obs_file, mode=mode, returncode=-1, pos_path=None,
            error=f"unknown mode {mode!r}; expected 'ppp' or 'rtk'",
        )
    if mode == "rtk" and base_obs is None:
        return RtklibRunResult(
            obs_file=obs_file, mode=mode, returncode=-1, pos_path=None,
            error="rtk mode requires base_obs",
        )
    work_dir.mkdir(parents=True, exist_ok=True)

    # Copy rover obs into work_dir to keep auxiliary files contained.
    obs_local = work_dir / obs_file.name
    if obs_local.resolve() != obs_file.resolve():
        shutil.copy2(obs_file, obs_local)

    config_path = write_config(work_dir, mode)
    pos_path = work_dir / (obs_file.stem + ".pos")
    cmd = [rnx2rtkp_bin, "-k", config_path.name, "-o", pos_path.name,
           obs_local.name]
    if base_obs is not None:
        base_local = work_dir / base_obs.name
        if base_local.resolve() != base_obs.resolve():
            shutil.copy2(base_obs, base_local)
        cmd.append(base_local.name)
    if nav_file is not None:
        nav_local = work_dir / nav_file.name
        if nav_local.resolve() != nav_file.resolve():
            shutil.copy2(nav_file, nav_local)
        cmd.append(nav_local.name)
    cmd.extend(extra_args)

    log.info("Running rnx2rtkp in %s: %s", work_dir, " ".join(cmd))
    try:
        proc = subprocess.run(
            cmd, cwd=str(work_dir),
            capture_output=True, text=True,
            timeout=timeout_s, check=False,
        )
    except subprocess.TimeoutExpired:
        return RtklibRunResult(
            obs_file=obs_file, mode=mode, returncode=-1, pos_path=None,
            error=f"rnx2rtkp timed out after {timeout_s}s",
        )
    except FileNotFoundError as e:
        return RtklibRunResult(
            obs_file=obs_file, mode=mode, returncode=-1, pos_path=None,
            error=f"rnx2rtkp binary not found: {e}",
        )

    if proc.returncode != 0:
        return RtklibRunResult(
            obs_file=obs_file, mode=mode, returncode=proc.returncode,
            pos_path=None,
            error=f"rnx2rtkp exit={proc.returncode}: "
                  f"{proc.stderr.strip()[:500]}",
        )
    if not pos_path.is_file() or pos_path.stat().st_size == 0:
        return RtklibRunResult(
            obs_file=obs_file, mode=mode, returncode=proc.returncode,
            pos_path=None, error="rnx2rtkp returned 0 but pos file empty",
        )
    return RtklibRunResult(
        obs_file=obs_file, mode=mode, returncode=proc.returncode,
        pos_path=pos_path,
    )


# ── Solution aggregation ───────────────────────────────────────── #


def _wgs84_lla_to_enu_local(
    lat_deg: float, lon_deg: float, h: float,
    ref_lat: float, ref_lon: float, ref_h: float,
) -> tuple[float, float, float]:
    """LLA → local ENU at (ref_lat, ref_lon, ref_h).  For small
    displacements, suitable for computing residuals within an
    converging static-mode trajectory.  See peppar_fix.pride_engine_diff
    for the canonical ECEF version when full-precision is needed."""
    import math
    _A = 6378137.0
    _F = 1.0 / 298.257223563
    _E2 = _F * (2 - _F)
    # Local-tangent linearization: lat/lon meters-per-degree at ref_lat
    rad = math.radians(ref_lat)
    n = _A / math.sqrt(1.0 - _E2 * math.sin(rad) ** 2)
    m = _A * (1.0 - _E2) / (1.0 - _E2 * math.sin(rad) ** 2) ** 1.5
    de = (lon_deg - ref_lon) * math.cos(rad) * n * math.pi / 180.0
    dn = (lat_deg - ref_lat) * m * math.pi / 180.0
    du = h - ref_h
    return de, dn, du


def aggregate_solution(
    epochs: list[RtklibPosEpoch],
    *,
    mode_label: str,
    converged_window_s: float = 600.0,
    accepted_q: frozenset[int] = frozenset({1, 2, 6}),
    cors_base: str = "",
) -> RtklibSolution | None:
    """Collapse per-epoch RTKLIB rows to a single RtklibSolution.

    Strategy: take the final ``converged_window_s`` seconds of epochs
    whose Q flag is in ``accepted_q`` (fix=1 / float=2 / ppp=6),
    compute the mean position + RMS of the residuals.  Static-mode
    rnx2rtkp converges, so the trailing window holds the most
    refined estimate.

    Returns None when no epoch passes the Q filter.
    """
    if not epochs:
        return None
    keep = [e for e in epochs if e.q in accepted_q]
    if not keep:
        return None
    # Trailing window
    tail_t = keep[-1].utc.timestamp() - converged_window_s
    tail = [e for e in keep if e.utc.timestamp() >= tail_t]
    if len(tail) < 2:
        # Too few epochs in trailing window; use everything that
        # passed the Q filter
        tail = keep
    lat = sum(e.lat for e in tail) / len(tail)
    lon = sum(e.lon for e in tail) / len(tail)
    height = sum(e.height for e in tail) / len(tail)
    # Residual RMS in local ENU
    import math
    sumsq = 0.0
    for e in tail:
        de, dn, du = _wgs84_lla_to_enu_local(
            e.lat, e.lon, e.height, lat, lon, height)
        sumsq += de * de + dn * dn + du * du
    sigma_3d = math.sqrt(sumsq / len(tail))
    return RtklibSolution(
        first_epoch=keep[0].utc,
        lat=lat, lon=lon, height=height,
        sigma_3d_m=sigma_3d, sig0_m=sigma_3d,
        n_obs=len(keep),
        mode=mode_label,
        cors_base=cors_base,
        extras={
            "rtklib_n_epochs_total": len(epochs),
            "rtklib_n_epochs_kept": len(keep),
            "rtklib_n_epochs_tail": len(tail),
            "rtklib_q_distribution":
                {q: sum(1 for e in epochs if e.q == q)
                 for q in {e.q for e in epochs}},
        },
    )


# ── Orchestration ──────────────────────────────────────────────── #


def process_one_obs(
    obs_file: Path,
    work_dir: Path,
    *,
    mode: str = "ppp",
    cors_station: str | None = None,
    cors_rinex_path: Path | None = None,
    cors_ntrip=None,  # peppar_survey_cors.CorsNtripConfig | None
    nav_file: Path | None = None,
    rnx2rtkp_bin: str = DEFAULT_RNX2RTKP,
    timeout_s: int = DEFAULT_RNX2RTKP_TIMEOUT_S,
    rnx2rtkp_runner=invoke_rnx2rtkp,
    cors_fetcher=fetch_cors_rinex,
    cors_ntrip_capturer=None,  # injectable for tests
) -> tuple[RtklibSolution | None, RtklibRunResult | None]:
    """Run rnx2rtkp on one obs file and aggregate into a solution.

    For mode="rtk", a CORS base RINEX is required, supplied via
    exactly one of:
      - ``cors_rinex_path``: explicit path to a pre-staged base
      - ``cors_station``: NOAA CORS station code; fetched by
        ``cors_fetcher``
      - ``cors_ntrip``: ``CorsNtripConfig`` for a live-NTRIP capture
        from a peer peppar-fix caster; streamed by
        ``cors_ntrip_capturer``
    """
    last_result: RtklibRunResult | None = None
    base_obs: Path | None = None
    if mode == "rtk":
        if cors_rinex_path is not None:
            base_obs = cors_rinex_path
        elif cors_station is not None:
            ydoy = doy_from_obs_name(obs_file)
            if ydoy is None:
                return None, RtklibRunResult(
                    obs_file=obs_file, mode=mode, returncode=-1,
                    pos_path=None,
                    error="can't derive (year, doy) from rover filename; "
                          "pass --cors-rinex-path explicitly",
                )
            year, doy = ydoy
            base_obs = cors_fetcher(cors_station, year, doy, work_dir)
            if base_obs is None:
                return None, RtklibRunResult(
                    obs_file=obs_file, mode=mode, returncode=-1,
                    pos_path=None,
                    error=f"CORS fetch failed for station={cors_station} "
                          f"year={year} doy={doy}",
                )
        elif cors_ntrip is not None:
            # Lazy import: avoid pulling peppar_survey_cors (and the
            # subprocess machinery it imports) into the static-CORS
            # paths that don't need it.
            if cors_ntrip_capturer is None:
                from peppar_fix.peppar_survey_cors import (
                    capture_cors_base_via_ntrip as cors_ntrip_capturer
                )
            base_obs = cors_ntrip_capturer(cors_ntrip, work_dir)
            if base_obs is None:
                return None, RtklibRunResult(
                    obs_file=obs_file, mode=mode, returncode=-1,
                    pos_path=None,
                    error=f"live NTRIP capture failed for "
                          f"{cors_ntrip.host}:{cors_ntrip.port}/"
                          f"{cors_ntrip.mount}",
                )
        else:
            return None, RtklibRunResult(
                obs_file=obs_file, mode=mode, returncode=-1, pos_path=None,
                error="rtk mode requires --cors-station, "
                      "--cors-rinex-path, or --cors-ntrip-*",
            )

    result = rnx2rtkp_runner(
        obs_file, work_dir, mode,
        base_obs=base_obs, nav_file=nav_file,
        rnx2rtkp_bin=rnx2rtkp_bin, timeout_s=timeout_s,
    )
    last_result = result
    if result.pos_path is None:
        log.warning("rnx2rtkp on %s failed: %s",
                    obs_file.name, result.error)
        return None, last_result

    epochs = parse_pos(result.pos_path)
    mode_label = "rtklib_cors" if mode == "rtk" else "rtklib_ppp"
    cors_base_label = base_obs.name if base_obs is not None else ""
    sol = aggregate_solution(
        epochs, mode_label=mode_label, cors_base=cors_base_label)
    if sol is None:
        log.warning("rnx2rtkp on %s: no epochs passed quality filter",
                    obs_file.name)
        return None, last_result
    return sol, last_result


def default_history_path(uid: str, history_dir: str | None = None) -> Path:
    """state/arp/<uid>/history.jsonl — shared with the PRIDE backend."""
    base = Path(history_dir) if history_dir else Path("state/arp")
    return base / uid / "history.jsonl"


def write_survey_from_running(
    running: RunningArp,
    uid: str,
    *,
    positions_dir: str | None = None,
    source_label: str = "peppar-survey --rtklib",
    dry_run: bool = False,
) -> tuple[PositionState, str]:
    """Convert RunningArp → PositionState(kind='survey') and write.

    Shape matches peppar_survey_pride.write_survey_from_running.
    """
    state = PositionState(
        mount_sn=running.mount_sn,
        ecef_m=tuple(running.ecef_m),  # type: ignore[arg-type]
        sigma_m=running.sigma_3d_m,
        updated=utc_now_iso(),
        source=source_label,
        # RTKLIB PPP-static solves against IGS products → ITRF2020.
        # Stamp the mid-window epoch the running mean is valid at.
        frame=Frame(CANONICAL_REALIZATION, decimal_year_from_mjd(
            0.5 * (running.oldest_mjd + running.newest_mjd))),
        kind="survey",
        extra={
            "rtklib_window_count": running.count,
            "rtklib_window_n_total": running.n_total,
            "rtklib_sigma_x_m": float(running.sigma_xyz_m[0]),
            "rtklib_sigma_y_m": float(running.sigma_xyz_m[1]),
            "rtklib_sigma_z_m": float(running.sigma_xyz_m[2]),
            "rtklib_oldest_mjd": float(running.oldest_mjd),
            "rtklib_newest_mjd": float(running.newest_mjd),
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


def run_rtklib_backend(
    obs_files: Sequence[Path],
    work_dir: Path,
    receiver_uid: str,
    *,
    mode: str = "ppp",
    cors_station: str | None = None,
    cors_rinex_path: Path | None = None,
    nav_file: Path | None = None,
    positions_dir: str | None = None,
    history_dir: str | None = None,
    mount_sn: int = 0,
    n_days: int = DEFAULT_N_DAYS,
    max_sig0_m: float = DEFAULT_MAX_SIG0_M,
    min_n_obs: int = DEFAULT_MIN_N_OBS,
    rnx2rtkp_bin: str = DEFAULT_RNX2RTKP,
    timeout_s: int = DEFAULT_RNX2RTKP_TIMEOUT_S,
    cors_ntrip=None,
    dry_run: bool = False,
    rnx2rtkp_runner=invoke_rnx2rtkp,
    cors_fetcher=fetch_cors_rinex,
    cors_ntrip_capturer=None,
    source_label: str | None = None,
) -> int:
    """Run rnx2rtkp over each obs file, archive each solution to
    history.jsonl, compute the running mean across the most recent
    N days, write the resulting ECEF + sigma to .survey.toml.

    Exit code (shell convention):
      0  one or more solutions ingested and a running mean computed
      1  no input obs files
      2  every obs file failed (or no quality-passing solution)
      3  rnx2rtkp binary missing or not executable
    """
    if not obs_files:
        log.error("no RINEX obs files supplied")
        return 1

    if not dry_run and not os.path.isfile(rnx2rtkp_bin):
        log.error("rnx2rtkp binary not found at %s — set "
                  "PEPPAR_RNX2RTKP_BIN or install rtklibexplorer",
                  rnx2rtkp_bin)
        return 3

    work_dir = Path(work_dir)
    history_path = default_history_path(receiver_uid, history_dir)
    if source_label is None:
        if mode == "rtk" and cors_ntrip is not None:
            source_label = (
                f"peppar-survey --rtklib --cors-ntrip "
                f"{cors_ntrip.host}:{cors_ntrip.port}/{cors_ntrip.mount}")
        elif mode == "rtk" and cors_station:
            source_label = f"peppar-survey --rtklib --cors-station {cors_station}"
        else:
            source_label = "peppar-survey --rtklib"

    n_solved = 0
    n_quality_ok = 0
    n_failed = 0
    for obs in sorted(Path(p) for p in obs_files):
        log.info("--- rnx2rtkp: %s ---", obs.name)
        sol, last = process_one_obs(
            obs, work_dir / obs.stem,
            mode=mode,
            cors_station=cors_station,
            cors_rinex_path=cors_rinex_path,
            cors_ntrip=cors_ntrip,
            nav_file=nav_file,
            rnx2rtkp_bin=rnx2rtkp_bin,
            timeout_s=timeout_s,
            rnx2rtkp_runner=rnx2rtkp_runner,
            cors_fetcher=cors_fetcher,
            cors_ntrip_capturer=cors_ntrip_capturer,
        )
        if sol is None:
            n_failed += 1
            log.warning("FAILED %s: %s",
                        obs.name, last.error if last else "no attempt")
            continue
        # Adapt to PrideSolution so arp_history's append + running_mean
        # can consume RTKLIB output through the same pipeline.
        adapter = rtklib_to_pride_solution(
            sol, src_path=str(obs))
        quality_ok = apply_quality_filter(
            adapter, max_sig0_m=max_sig0_m, min_n_obs=min_n_obs)
        n_solved += 1
        if quality_ok:
            n_quality_ok += 1
        if dry_run:
            log.info("  DRY RUN — would append (σ_3d=%.4fm n_obs=%d "
                     "quality_ok=%s)",
                     sol.sigma_3d_m, sol.n_obs, quality_ok)
            continue
        append_solution(
            history_path, adapter,
            mount_sn=mount_sn, quality_ok=quality_ok,
        )
        log.info("  appended %s → %s (σ_3d=%.4fm quality_ok=%s)",
                 sol.date_iso, history_path, sol.sigma_3d_m, quality_ok)

    log.info("rnx2rtkp sweep complete: %d solved (%d quality_ok), %d failed",
             n_solved, n_quality_ok, n_failed)

    if n_solved == 0:
        log.error("no rtklib solutions produced — survey.toml not written")
        return 2

    running = running_mean(
        history_path,
        n_days=n_days, mount_sn=mount_sn,
        require_quality_ok=True,
    )
    if running is None:
        log.error("running_mean returned None — survey.toml not written")
        return 2

    write_survey_from_running(
        running, receiver_uid,
        positions_dir=positions_dir,
        source_label=source_label,
        dry_run=dry_run,
    )
    return 0

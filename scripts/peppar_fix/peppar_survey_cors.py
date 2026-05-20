"""peppar-survey --rtklib live-NTRIP base: stream RTCM from a peer
peppar-fix's NTRIP caster, convert to a base RINEX, hand off to the
existing PR #48 rnx2rtkp pipeline.

Architecture:
    peer caster (scripts/ntrip_caster.py, port 2102, mount /PEPPAR)
        ↓ RTCM 3.3 MSM4 1074/1094/1124 + 1005 ARP
    str2str (NTRIP client + writer)
        ↓ raw RTCM3 stream → file
    convbin (RTKLIB binary converter)
        ↓ RTCM3 → RINEX 3 obs
    process_one_obs (peppar_survey_rtklib, PR #48)
        ↓ rover RINEX + base RINEX + nav RINEX → .pos
    aggregate_solution → .survey.toml

Why subprocess RTKLIB tools rather than a Python RTCM-to-RINEX path:
RTKLIB's str2str and convbin are battle-tested for MSM4 → RINEX,
handle every signal-code corner case, and ship alongside rnx2rtkp.
Reusing them keeps the new code surface small and avoids a custom
decoder that would need its own multi-GNSS validation matrix.

Operator usage lives in scripts/peppar_survey.py; this module is
imported by that CLI's --cors-ntrip-* path.
"""
from __future__ import annotations

import logging
import os
import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path


log = logging.getLogger("peppar-survey.cors")


# str2str + convbin binary paths.  Both ship with RTKLIB alongside
# rnx2rtkp; same install layout assumed (rtklibexplorer fork preferred
# but upstream tomojitakasu also works).  Override via env.
DEFAULT_STR2STR = os.environ.get(
    "PEPPAR_STR2STR_BIN",
    shutil.which("str2str") or "/usr/local/bin/str2str",
)
DEFAULT_CONVBIN = os.environ.get(
    "PEPPAR_CONVBIN_BIN",
    shutil.which("convbin") or "/usr/local/bin/convbin",
)


# Default streaming duration when --cors-ntrip-duration isn't given.
# 5 min produces ~300 epochs at 1 Hz, enough for RTK static convergence
# on short baselines and a usable convbin → RINEX product.
DEFAULT_CORS_NTRIP_DURATION_S = 5 * 60

# When the str2str run hits its requested duration we terminate it
# politely; if it doesn't exit on SIGTERM within this many seconds we
# escalate to SIGKILL.
_TERMINATE_GRACE_S = 5

# str2str sometimes needs a moment after termination before its output
# buffer is fully flushed.  Wait this long before checking file size.
_FLUSH_PAUSE_S = 0.5


@dataclass
class CorsNtripConfig:
    """Parameters for a single live-NTRIP base capture."""
    host: str
    port: int
    mount: str
    duration_s: int = DEFAULT_CORS_NTRIP_DURATION_S
    user: str = ""        # peer caster doesn't require auth
    password: str = ""

    def __post_init__(self) -> None:
        if not self.host:
            raise ValueError("CorsNtripConfig.host is required")
        if not (0 < self.port < 65536):
            raise ValueError(f"port out of range: {self.port}")
        if not self.mount:
            raise ValueError("CorsNtripConfig.mount is required")
        if self.duration_s <= 0:
            raise ValueError(
                f"duration_s must be > 0: {self.duration_s}")


def ntripcli_url(cfg: CorsNtripConfig) -> str:
    """Build str2str's ntripcli:// URL from a CorsNtripConfig.

    Format: ``ntripcli://[user[:password]]@host:port/mount``.  When
    user is empty we use the bare ``host:port/mount`` form, which the
    peer caster (no auth) accepts.
    """
    auth = ""
    if cfg.user:
        auth = cfg.user
        if cfg.password:
            auth = f"{cfg.user}:{cfg.password}"
        auth += "@"
    return f"ntripcli://{auth}{cfg.host}:{cfg.port}/{cfg.mount}"


def stream_rtcm_from_ntrip(
    cfg: CorsNtripConfig,
    output_path: Path,
    *,
    str2str_bin: str = DEFAULT_STR2STR,
    terminate_grace_s: int = _TERMINATE_GRACE_S,
) -> Path | None:
    """Run str2str to capture RTCM from an NTRIP caster into a file.

    str2str runs as a child process for ``cfg.duration_s`` seconds,
    then is terminated.  ``output_path`` ends up containing the raw
    RTCM3 byte stream (one or more MSM4 + 1005 frames).

    Returns the output path on success, or None if str2str failed or
    produced an empty file.
    """
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    # Wipe any prior stream so partial-file size checks below are honest.
    if output_path.exists():
        output_path.unlink()

    in_url = ntripcli_url(cfg)
    out_url = f"file://{output_path}"
    cmd = [str2str_bin, "-in", in_url, "-out", out_url]
    log.info("str2str: %s (duration=%ds)", " ".join(cmd), cfg.duration_s)
    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except FileNotFoundError as e:
        log.error("str2str binary not found at %s: %s", str2str_bin, e)
        return None

    try:
        # Let str2str run for the requested duration.  Wait with a
        # timeout so we wake up promptly when done.
        try:
            proc.wait(timeout=cfg.duration_s)
        except subprocess.TimeoutExpired:
            pass  # expected: duration elapsed
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=terminate_grace_s)
            except subprocess.TimeoutExpired:
                log.warning("str2str didn't exit on SIGTERM; killing")
                proc.kill()
                proc.wait(timeout=terminate_grace_s)

    # Give the kernel a moment to flush str2str's output buffer to disk.
    time.sleep(_FLUSH_PAUSE_S)

    if not output_path.exists() or output_path.stat().st_size == 0:
        stderr_tail = (proc.stderr.read() if proc.stderr else "")[-500:]
        log.warning("str2str produced no RTCM in %s (stderr tail: %s)",
                    output_path, stderr_tail.strip())
        return None

    log.info("str2str captured %d bytes into %s",
             output_path.stat().st_size, output_path)
    return output_path


def rtcm3_to_rinex(
    rtcm_path: Path,
    work_dir: Path,
    *,
    output_obs_name: str = "cors-ntrip.obs",
    convbin_bin: str = DEFAULT_CONVBIN,
    timeout_s: int = 5 * 60,
) -> Path | None:
    """Convert a captured RTCM3 stream to a RINEX 3 obs file.

    Uses RTKLIB ``convbin`` with ``-r rtcm3`` to decode the byte stream.
    The output obs file is named ``output_obs_name`` inside ``work_dir``
    (callers can pick a name that doesn't collide with the rover RINEX).

    Returns the path to the .obs file, or None on failure.  convbin
    also emits .nav / .gnav / .qnav etc. as siblings; we ignore those
    here because the rover-side flow supplies its own broadcast nav.
    """
    rtcm_path = Path(rtcm_path)
    work_dir = Path(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)
    if not rtcm_path.is_file():
        log.error("rtcm input %s missing — skipping convbin", rtcm_path)
        return None
    if rtcm_path.stat().st_size == 0:
        log.error("rtcm input %s is empty — skipping convbin", rtcm_path)
        return None

    out_obs = work_dir / output_obs_name
    cmd = [
        convbin_bin, str(rtcm_path),
        "-r", "rtcm3",
        "-od", str(work_dir),
        "-o", out_obs.name,
        # Suppress the nav-output files — the rover side already has
        # its own broadcast nav via --nav-file or pdp3's normal chain.
        "-n", "/dev/null",
        "-g", "/dev/null",
        "-h", "/dev/null",
        "-q", "/dev/null",
        "-l", "/dev/null",
        "-b", "/dev/null",
        "-i", "/dev/null",
        "-s", "/dev/null",
    ]
    log.info("convbin: %s", " ".join(cmd))
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True,
            timeout=timeout_s, check=False,
            cwd=str(work_dir),
        )
    except FileNotFoundError as e:
        log.error("convbin binary not found at %s: %s", convbin_bin, e)
        return None
    except subprocess.TimeoutExpired:
        log.error("convbin timed out after %ds", timeout_s)
        return None

    if proc.returncode != 0:
        log.warning("convbin exit=%d on %s: %s",
                    proc.returncode, rtcm_path, proc.stderr.strip()[:500])
        return None
    if not out_obs.is_file() or out_obs.stat().st_size == 0:
        log.warning("convbin emitted no .obs at %s", out_obs)
        return None
    log.info("convbin wrote %d bytes of RINEX → %s",
             out_obs.stat().st_size, out_obs)
    return out_obs


def capture_cors_base_via_ntrip(
    cfg: CorsNtripConfig,
    work_dir: Path,
    *,
    output_obs_name: str = "cors-ntrip.obs",
    str2str_bin: str = DEFAULT_STR2STR,
    convbin_bin: str = DEFAULT_CONVBIN,
    rtcm_filename: str = "cors-ntrip.rtcm3",
    ntrip_streamer=stream_rtcm_from_ntrip,
    rtcm_converter=rtcm3_to_rinex,
) -> Path | None:
    """End-to-end: NTRIP stream → RTCM file → base RINEX.

    Two-step composition so each subprocess can be mocked independently
    in tests.  Returns Path to base RINEX, or None if either step
    failed.
    """
    work_dir = Path(work_dir)
    rtcm_path = work_dir / rtcm_filename
    captured = ntrip_streamer(
        cfg, rtcm_path, str2str_bin=str2str_bin)
    if captured is None:
        return None
    return rtcm_converter(
        captured, work_dir,
        output_obs_name=output_obs_name,
        convbin_bin=convbin_bin,
    )

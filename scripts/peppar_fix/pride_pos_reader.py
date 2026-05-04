"""Parse PRIDE-PPP static-mode `pos_YYYYDDD_<site>` output files.

PRIDE-PPP (https://github.com/PrideLab/PRIDE-PPPAR) is the precise
post-processor we use to ground the ARP for static deployments.
Each daily run produces one .pos file with a ~50-line header and
exactly one data line in static mode.

This module parses both into a `PrideSolution` dataclass: the
geodetic answer (ECEF + sigmas) plus enough provenance metadata
(satellite products used, processing config, ambiguity-fixing
counts) for downstream quality filtering and reproducibility.

Sample at timelab/surveys/data/pride/pos_2026116_ufo1.

Output sigma convention:
  - PRIDE pos file lists *cofactors* Sx, Sy, Sz (diagonal), Rxy/Rxz/Ryz
    (off-diagonal) and a single variance factor Sig0 (m).
  - Actual coordinate σ is `Sig0 * sqrt(Sx)` for each axis.  We expose
    both the raw cofactor and the derived σ_xyz_m so callers don't
    have to remember the convention.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from pathlib import Path


class PrideParseError(Exception):
    """Raised when a PRIDE pos file cannot be parsed."""


# PRIDE pos files use a fixed two-column layout: value in cols 0-59,
# label in cols 60+.  The label column is left-justified and starts
# with an uppercase letter; the value column is left-justified text
# or whitespace-padded numerics.
_LABEL_COL = 60

# Static mode emits exactly one data line per file: leading space,
# 4-char station name (alphanumeric, lowercase by RINEX convention),
# then 12 whitespace-separated numeric fields.  Distinguished from
# left-padded numeric header lines by the alphanumeric leading token.
_DATA_LINE_RE = re.compile(
    r"^\s+([a-zA-Z0-9_]{1,8})\s+([\d.\-eE+]+)"                   # name MJD
    r"\s+([\d.\-eE+]+)\s+([\d.\-eE+]+)\s+([\d.\-eE+]+)"          # X Y Z
    r"\s+([\d.\-eE+]+)\s+([\d.\-eE+]+)\s+([\d.\-eE+]+)"          # Sx Sy Sz
    r"\s+([\d.\-eE+]+)\s+([\d.\-eE+]+)\s+([\d.\-eE+]+)"          # Rxy Rxz Ryz
    r"\s+([\d.\-eE+]+)\s+(\d+)\s*$"                               # Sig0 Nobs
)


@dataclass
class PrideSolution:
    """One day of PRIDE static-mode ARP solution.

    Geometric:
      name           — station 4-char ID from RINEX header
      mjd            — Modified Julian Day (epoch midpoint per PRIDE)
      ecef_m         — (X, Y, Z) ECEF tuple, meters
      cofactor_xyz   — diagonal cofactors Sx, Sy, Sz (PRIDE units)
      cofactor_off   — off-diagonal Rxy, Rxz, Ryz
      sig0_m         — variance factor (m), Sig0
      sigma_xyz_m    — Sig0 × sqrt(Sx,Sy,Sz) — the σ callers usually want
      n_obs          — number of observations used

    Provenance:
      mode           — 'Static' / 'Kinematic' (we expect Static)
      first_epoch    — observation start (UTC)
      last_epoch     — observation end (UTC)
      interval_s     — observation interval (seconds)
      mask_deg       — elevation mask
      receiver_type  — RINEX MARKER + receiver type
      antenna_type   — RINEX antenna type + radome
      products       — dict of satellite-product file basenames
                       keys: orbit, clock, attitude, bias, erp, quaternions
      ambiguity_fixing — dict {sys: n_fixed} (GPS, GAL, BDS2, BDS3, QZSS)
      ambiguity_enabled — True if AR was attempted
      raw_header     — raw key→value strings from the header (for round-trip)
    """
    name: str
    mjd: float
    ecef_m: tuple[float, float, float]
    cofactor_xyz: tuple[float, float, float]
    cofactor_off: tuple[float, float, float]
    sig0_m: float
    sigma_xyz_m: tuple[float, float, float]
    n_obs: int

    mode: str = ""
    first_epoch: datetime | None = None
    last_epoch: datetime | None = None
    interval_s: float | None = None
    mask_deg: float | None = None
    receiver_type: str = ""
    antenna_type: str = ""
    products: dict = field(default_factory=dict)
    ambiguity_fixing: dict = field(default_factory=dict)
    ambiguity_enabled: bool = False
    raw_header: dict = field(default_factory=dict)
    src_path: str = ""

    @property
    def sigma_3d_m(self) -> float:
        """3D RSS formal sigma in meters."""
        return math.sqrt(sum(s * s for s in self.sigma_xyz_m))

    @property
    def date_iso(self) -> str:
        """Calendar date of first_epoch (UTC, ISO YYYY-MM-DD).

        Falls back to MJD-derived date when first_epoch absent.
        """
        if self.first_epoch is not None:
            return self.first_epoch.date().isoformat()
        return _mjd_to_date(self.mjd).isoformat()


def parse_pos(path: str | Path) -> PrideSolution:
    """Parse a PRIDE static-mode .pos file → PrideSolution.

    Raises PrideParseError on missing data line or malformed header.
    """
    p = Path(path)
    if not p.is_file():
        raise PrideParseError(f"file not found: {p}")

    header: dict[str, str] = {}
    data_match = None
    pco_lines: list[str] = []  # PCO lines all share the SITE ANTENNA PCO label
    try:
        with open(p) as f:
            for line in f:
                # Strip trailing newline only; preserve internal columns.
                line = line.rstrip("\n").rstrip("\r")
                if not line.strip():
                    continue
                # Data line first — exactly one in static mode.
                m = _DATA_LINE_RE.match(line)
                if m is not None:
                    data_match = m
                    continue
                # Header lines: split at the fixed label column.
                if len(line) >= _LABEL_COL:
                    value = line[:_LABEL_COL].strip()
                    label = line[_LABEL_COL:].strip()
                else:
                    # Lines shorter than the label column are
                    # comments or section headings; ignore.
                    continue
                if not label or not label[0].isupper():
                    continue
                if label.startswith("SITE ANTENNA PCO"):
                    pco_lines.append(value)
                else:
                    header[label] = value
    except OSError as exc:
        raise PrideParseError(f"reading {p}: {exc}") from exc

    if data_match is None:
        raise PrideParseError(f"no static-mode data line found in {p}")

    name = data_match.group(1)
    mjd = float(data_match.group(2))
    ecef = (float(data_match.group(3)),
            float(data_match.group(4)),
            float(data_match.group(5)))
    cof_xyz = (float(data_match.group(6)),
               float(data_match.group(7)),
               float(data_match.group(8)))
    cof_off = (float(data_match.group(9)),
               float(data_match.group(10)),
               float(data_match.group(11)))
    sig0 = float(data_match.group(12))
    n_obs = int(data_match.group(13))

    sigma_xyz = tuple(sig0 * math.sqrt(c) if c >= 0 else math.nan
                       for c in cof_xyz)

    sol = PrideSolution(
        name=name, mjd=mjd, ecef_m=ecef,
        cofactor_xyz=cof_xyz, cofactor_off=cof_off,
        sig0_m=sig0, sigma_xyz_m=sigma_xyz, n_obs=n_obs,
        raw_header=header, src_path=str(p),
    )
    _populate_provenance(sol, header)
    return sol


def _populate_provenance(sol: PrideSolution, h: dict[str, str]) -> None:
    """Fill provenance fields on sol from the parsed header dict."""
    pos_mode = h.get("POS MODE/PRIORI (meter)", "")
    if pos_mode:
        sol.mode = pos_mode.split()[0]

    sol.first_epoch = _parse_epoch(h.get("OBS FIRST EPOCH"))
    sol.last_epoch = _parse_epoch(h.get("OBS LAST EPOCH"))
    interval_s = h.get("OBS INTERVAL (sec)")
    if interval_s:
        try:
            sol.interval_s = float(interval_s)
        except ValueError:
            pass
    mask = h.get("OBS MASK ANGLE (deg)")
    if mask:
        try:
            sol.mask_deg = float(mask)
        except ValueError:
            pass

    sol.receiver_type = h.get("SITE RECEIVER TYPE", "")
    sol.antenna_type = h.get("SITE ANTENNA TYPE", "")

    products = {}
    for key, dest in (
        ("SAT ORBIT", "orbit"),
        ("SAT CLOCK", "clock"),
        ("SAT ATTITUDE", "attitude"),
        ("SAT BIAS", "bias"),
        ("EARTH ROTATION PARAMETERS", "erp"),
        ("SAT QUATERNIONS", "quaternions"),
    ):
        v = h.get(key)
        if v:
            products[dest] = v
    sol.products = products

    ambig_line = h.get("AMB FIXING")
    if ambig_line:
        # 'YES  GPS    58  GAL     0  BDS2    0  BDS3    0  QZSS    0'
        # 'NO ...' indicates AR not attempted.
        toks = ambig_line.split()
        sol.ambiguity_enabled = (toks[0].upper() == "YES") if toks else False
        # Pair (key, count) tokens after the YES/NO prefix.
        pairs = toks[1:]
        for i in range(0, len(pairs) - 1, 2):
            sys_key = pairs[i]
            try:
                sol.ambiguity_fixing[sys_key] = int(pairs[i + 1])
            except ValueError:
                pass


def _parse_epoch(s: str | None) -> datetime | None:
    """Parse PRIDE 'YYYY MM DD HH MI SS.SS' epoch line → UTC datetime."""
    if not s:
        return None
    parts = s.split()
    if len(parts) < 6:
        return None
    try:
        y, mo, d, h, mi = (int(parts[i]) for i in range(5))
        sec = float(parts[5])
    except (ValueError, IndexError):
        return None
    s_int = int(sec)
    us = int(round((sec - s_int) * 1_000_000))
    try:
        return datetime(y, mo, d, h, mi, s_int, us, tzinfo=timezone.utc)
    except ValueError:
        return None


_MJD_EPOCH = datetime(1858, 11, 17, tzinfo=timezone.utc)


def _mjd_to_date(mjd: float):
    """Return UTC calendar date (date object) for a Modified Julian Day.

    PRIDE's static MJD is the midpoint of the 24h window, so a value
    ending in .4998 maps to that day's date — floor() recovers it.
    """
    return (_MJD_EPOCH + timedelta(days=math.floor(mjd))).date()

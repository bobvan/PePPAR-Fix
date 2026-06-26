"""Parse NRCan CSRS-PPP SINEX_TRO (.tro) troposphere files → ZTD(t).

NRCan's CSRS-PPP service (https://webapp.csrs-scrs.nrcan-rncan.gc.ca/)
returns, alongside the .pos/.sum, a `.tro` SINEX_TRO file carrying the
batch-estimated **total zenith tropospheric delay** at a fixed cadence
(typically 300 s).  This is the external ZTD *truth* the pos_replay ZTD
comparison scores our real-time random-walk ZTD against (capture manifest
§5): batch-smoothed, so it legitimately *leads* our forward-filter ZTD —
which is exactly why ``compare_ztd`` removes the constant lag+apriori offset
and scores only the time-varying departure.

## Format (SINEX_TRO 2.00, the CSRS-PPP dialect)

Blocks are ``+NAME`` / ``-NAME`` delimited.  Two matter here:

  +TROP/DESCRIPTION
   SOLUTION_FIELDS_1   TROTOT STDDEV TGNTOT STDDEV TGETOT STDDEV
  -TROP/DESCRIPTION
  +TROP/SOLUTION
  *SITE ____EPOCH___ TROTOT STDDEV TGNTOT STDDEV TGETOT STDDEV
   ABCD 26:001:00000 2451.6    1.2   -0.4    0.3    0.1    0.3
  -TROP/SOLUTION

``SOLUTION_FIELDS`` names the columns that follow ``SITE`` + ``EPOCH`` in
each solution row, so we locate ``TROTOT`` (total ZTD, **mm**) and the
``STDDEV`` immediately after it by name rather than fixed offset — robust to
the field set varying between products.  Epochs are ``YY:DOY:SSSSS``.

## Timescale note

We parse the ``YY:DOY:SSSSS`` epoch verbatim into a UTC calendar datetime, so
``.timestamp()`` lands on the same "GPS-seconds-as-calendar-fields" basis the
engine's ``[PPP_STATE] gps=`` key uses (engine ``gps_time`` =
``datetime(1980,1,6, utc) + gps_seconds``, no leap correction).  If a given
product's epochs are actually UTC rather than GPS, the residual is a constant
~18 s lag — absorbed by ``compare_ztd``'s offset removal and negligible
against ZTD's slow variation.  ``time_offset_s`` is exposed for an explicit
correction if a product ever needs it.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Iterable, Optional

# 'YY:DOY:SSSSS' (TRO uses a 2-digit year; tolerate a 4-digit one too).
_TRO_EPOCH = re.compile(r"(\d{2,4}):(\d{1,3}):(\d{1,5})")
_MM_PER_M = 1000.0


@dataclass
class TroZtd:
    """One total-ZTD sample from a .tro solution row."""
    t_s: float            # unix seconds (GPS-as-UTC calendar basis; see module doc)
    ztd_m: float          # TROTOT, metres (converted from mm)
    sigma_ztd_m: float    # STDDEV, metres
    site: str


def _parse_tro_epoch(s: str, *, time_offset_s: float = 0.0) -> Optional[float]:
    """'YY:DOY:SSSSS' → unix seconds (UTC calendar basis), or None."""
    m = _TRO_EPOCH.fullmatch(s.strip())
    if not m:
        return None
    year = int(m.group(1))
    if year < 100:                       # 2-digit: SINEX pivots at 50
        year += 2000 if year <= 50 else 1900
    doy = int(m.group(2))
    secs = int(m.group(3))
    base = datetime(year, 1, 1, tzinfo=timezone.utc)
    return (base + timedelta(days=doy - 1, seconds=secs)).timestamp() + time_offset_s


def _solution_field_indices(fields: list) -> tuple:
    """From a SOLUTION_FIELDS list, return (trotot_idx, stddev_idx) — the
    0-based positions of TROTOT and the STDDEV that immediately follows it,
    within the value columns AFTER site+epoch.  stddev_idx is None if absent.
    """
    upper = [f.upper() for f in fields]
    try:
        ti = upper.index("TROTOT")
    except ValueError:
        return None, None
    si = None
    if ti + 1 < len(upper) and "STD" in upper[ti + 1]:
        si = ti + 1
    return ti, si


def iter_tro_ztd(lines: Iterable[str], *, site: Optional[str] = None,
                 time_offset_s: float = 0.0):
    """Yield :class:`TroZtd` for each TROP/SOLUTION row (optionally one site).

    ``site`` (case-insensitive) filters to a single station; None yields all
    (use when the file has exactly one site, as CSRS-PPP single-station runs
    do).  Falls back to a default field layout (TROTOT, STDDEV as the first
    two value columns) if no SOLUTION_FIELDS block was seen — the CSRS-PPP
    default.
    """
    want = site.upper() if site else None
    in_solution = False
    ti, si = None, None           # column indices from SOLUTION_FIELDS
    for raw in lines:
        ln = raw.rstrip("\n")
        stripped = ln.strip()
        if stripped.startswith("+TROP/SOLUTION"):
            in_solution = True
            continue
        if stripped.startswith("-TROP/SOLUTION"):
            in_solution = False
            continue
        if stripped.upper().startswith("SOLUTION_FIELDS"):
            # 'SOLUTION_FIELDS_1   TROTOT STDDEV ...' → field names after the key
            ti, si = _solution_field_indices(stripped.split()[1:])
            continue
        if not in_solution or not stripped or stripped.startswith("*"):
            continue
        toks = stripped.split()
        if len(toks) < 3:
            continue
        site_id, epoch_str, values = toks[0], toks[1], toks[2:]
        if want is not None and site_id.upper() != want:
            continue
        t_s = _parse_tro_epoch(epoch_str, time_offset_s=time_offset_s)
        if t_s is None:
            continue
        # Resolve columns: SOLUTION_FIELDS if seen, else CSRS-PPP default
        # (TROTOT then STDDEV as the first two value columns).
        _ti = ti if ti is not None else 0
        _si = si if (ti is not None) else 1
        if _ti >= len(values):
            continue
        try:
            ztd_mm = float(values[_ti])
            sig_mm = (float(values[_si])
                      if _si is not None and _si < len(values) else 0.0)
        except ValueError:
            continue
        yield TroZtd(t_s=t_s, ztd_m=ztd_mm / _MM_PER_M,
                     sigma_ztd_m=sig_mm / _MM_PER_M, site=site_id)


def parse_tro_lines(lines: Iterable[str], *, site: Optional[str] = None,
                    time_offset_s: float = 0.0) -> list:
    """Parse .tro content (an iterable of lines) → time-ordered TroZtd list."""
    out = list(iter_tro_ztd(lines, site=site, time_offset_s=time_offset_s))
    out.sort(key=lambda p: p.t_s)
    return out


def parse_tro(path, *, site: Optional[str] = None,
              time_offset_s: float = 0.0) -> list:
    """Read a .tro file into a time-ordered list of :class:`TroZtd`."""
    with open(path) as f:
        return parse_tro_lines(f, site=site, time_offset_s=time_offset_s)

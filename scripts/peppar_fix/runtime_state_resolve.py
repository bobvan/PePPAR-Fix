"""DO operational (runtime) state — engine-owned, .runtime.toml only.

The engine's per-checkpoint operational state — last commanded frequency
offset and last DAC code — is the engine-WRITTEN counterpart to the
tool-written characterization (do_char_resolve).  It lives in the
engine-only state/dos/<uid>.runtime.toml (do_schema RuntimeState).

Post burn-down (PR 4) there is no legacy <uid>.json fallback: a missing
runtime file is a COLD START (all-None, NOT fatal — unlike a missing
characterization, runtime state is re-derived on the next checkpoint).
Writes go to <uid>.runtime.toml only.

Why a read-modify-write per field: the engine updates freq and DAC code
at different sites/times, but save_runtime_state writes the whole
RuntimeState atomically.  The update_* helpers load the current runtime,
set one field, and save.
"""
from __future__ import annotations

import logging
import math
import os
import time as _time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional

from peppar_fix import do_schema

log = logging.getLogger(__name__)


SOURCE_RUNTIME_TOML = "runtime.toml"
SOURCE_NONE = "none"


@dataclass
class ResolvedRuntime:
    freq_offset_ppb: Optional[float]
    dac_code: Optional[int]
    source: str


def _now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def resolve_runtime_state(do_uid: str, *, dos_dir: Optional[str] = None,
                          json_state_dir: Optional[str] = None
                          ) -> ResolvedRuntime:
    """Read the DO's runtime state from <uid>.runtime.toml.

    Returns an all-None ResolvedRuntime (SOURCE_NONE = cold start) when the
    file is absent.  (json_state_dir kept for signature stability; the
    legacy JSON fallback was removed in the burn-down.)
    """
    rs = do_schema.load_runtime_state(do_uid, dos_dir=dos_dir)
    if rs is not None:
        return ResolvedRuntime(
            freq_offset_ppb=rs.last_known_freq_offset_ppb,
            dac_code=rs.last_known_dac_code, source=SOURCE_RUNTIME_TOML)
    return ResolvedRuntime(freq_offset_ppb=None, dac_code=None,
                           source=SOURCE_NONE)


def _save(do_uid: str, freq_ppb: Optional[float], dac_code: Optional[int],
          dos_dir: Optional[str]) -> None:
    # Preserve "freq unknown" as None — never persist a misleading 0.0 (a
    # DAC-code-only write before any freq is known must NOT warm-start the
    # DO at 0.0 ppb; Main's PR #176 nit).
    rs = do_schema.RuntimeState(
        last_known_freq_offset_ppb=(None if freq_ppb is None
                                    else float(freq_ppb)),
        last_known_dac_code=(None if dac_code is None else int(dac_code)),
        last_updated=_now_iso())
    do_schema.save_runtime_state(do_uid, rs, dos_dir=dos_dir)


def update_runtime_freq(do_uid: str, freq_ppb: float, *,
                        dos_dir: Optional[str] = None,
                        json_state_dir: Optional[str] = None) -> None:
    """Set last_known_freq_offset_ppb in <uid>.runtime.toml, preserving the
    current dac_code (seeded from legacy on first write)."""
    cur = resolve_runtime_state(do_uid, dos_dir=dos_dir,
                                json_state_dir=json_state_dir)
    _save(do_uid, freq_ppb, cur.dac_code, dos_dir)


def update_runtime_dac_code(do_uid: str, code: int, *,
                            dos_dir: Optional[str] = None,
                            json_state_dir: Optional[str] = None) -> None:
    """Set last_known_dac_code in <uid>.runtime.toml, preserving the current
    freq offset (seeded from legacy on first write)."""
    cur = resolve_runtime_state(do_uid, dos_dir=dos_dir,
                                json_state_dir=json_state_dir)
    _save(do_uid, cur.freq_offset_ppb, code, dos_dir)


def is_warm_startable(do_uid: str, *, max_age_s: float = 86400.0,
                      max_ppb: float = 500.0, dos_dir: Optional[str] = None,
                      json_state_dir: Optional[str] = None) -> tuple:
    """Warm-start check against <uid>.runtime.toml.

    Pass criteria: the runtime file exists, its freq is finite and within
    ±max_ppb, and its mtime is within max_age_s.  Returns (bool, info-dict).
    (json_state_dir kept for signature stability; the legacy JSON fallback
    was removed in the burn-down — an absent runtime file is "no_state_file"
    = cold start.)
    """
    toml_path = do_schema._runtime_path(do_uid, dos_dir)
    if not os.path.exists(toml_path):
        return (False, {"reason": "no_state_file"})
    path, source = toml_path, SOURCE_RUNTIME_TOML

    try:
        age_s = _time.time() - os.path.getmtime(path)
    except OSError as e:
        return (False, {"reason": f"stat_failed: {e}"})
    if age_s > max_age_s:
        return (False, {"reason": f"too_old: age={age_s:.0f}s "
                                  f"max={max_age_s:.0f}s"})
    rr = resolve_runtime_state(do_uid, dos_dir=dos_dir,
                               json_state_dir=json_state_dir)
    freq = rr.freq_offset_ppb
    if freq is None:
        return (False, {"reason": "no_last_known_freq"})
    if not math.isfinite(freq):
        return (False, {"reason": f"freq_not_finite: {freq}"})
    if abs(freq) > max_ppb:
        return (False, {"reason": f"freq_out_of_envelope: "
                                  f"|{freq:.1f}| > {max_ppb:.1f} ppb"})
    return (True, {"freq_ppb": freq, "age_s": age_s, "reason": "fresh",
                   "source": source})

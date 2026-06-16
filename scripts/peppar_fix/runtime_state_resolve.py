"""Transition bridge — DO operational (runtime) state.

The engine's per-checkpoint operational state — last commanded frequency
offset and last DAC code — is the engine-WRITTEN counterpart to the
tool-written characterization (do_char_resolve).  The
doCharacterizationArchitecture moves it from the legacy
state/dos/<uid>.json (mixed with characterization) to a dedicated
engine-only state/dos/<uid>.runtime.toml (do_schema RuntimeState).

This module is the single place the engine reads and writes runtime
state during the migration.  Reads PREFER <uid>.runtime.toml and FALL
BACK to the legacy JSON's last_known_* fields; writes go to
<uid>.runtime.toml only (the new schema's "engine writes only the
runtime file" rule).  After PR 3 migrates each host the legacy read
fallback is dead weight; it and this whole bridge are deleted in the
PR 4 burn-down.

Why a read-modify-write per field: the engine updates freq and DAC code
at different sites/times, but save_runtime_state writes the whole
RuntimeState atomically.  The update_* helpers load the current runtime
(seeding from legacy on first write), set one field, and save.
"""
from __future__ import annotations

import logging
import math
import os
import time as _time
import warnings
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional

from peppar_fix import do_schema

log = logging.getLogger(__name__)


SOURCE_RUNTIME_TOML = "runtime.toml"
SOURCE_LEGACY = "legacy-json"
SOURCE_NONE = "none"


_warned_legacy: set[str] = set()


@dataclass
class ResolvedRuntime:
    freq_offset_ppb: Optional[float]
    dac_code: Optional[int]
    source: str


def _now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _warn_legacy_once(do_uid: str) -> None:
    if do_uid in _warned_legacy:
        return
    _warned_legacy.add(do_uid)
    warnings.warn(
        f"DO {do_uid!r} runtime state read from legacy "
        f"state/dos/{do_uid}.json — the engine now writes "
        f"{do_uid}.runtime.toml; run scripts/migrate_do_state.py.  "
        f"Legacy runtime fallback is removed in the burn-down (PR 4).",
        DeprecationWarning, stacklevel=2)
    log.warning("DO %s runtime state from LEGACY JSON — will write "
                ".runtime.toml going forward (removed in PR 4)", do_uid)


def _legacy_runtime(do_uid: str, json_state_dir: Optional[str]
                    ) -> Optional[ResolvedRuntime]:
    from peppar_fix import do_state
    ds = do_state.load_do_state(do_uid, state_dir=json_state_dir)
    if not ds:
        return None
    freq = ds.get("last_known_freq_offset_ppb")
    table = ds.get("dac_code_by_temperature")
    code = table.get("last") if isinstance(table, dict) else None
    return ResolvedRuntime(
        freq_offset_ppb=(None if freq is None else float(freq)),
        dac_code=(None if code is None else int(code)),
        source=SOURCE_LEGACY)


def resolve_runtime_state(do_uid: str, *, dos_dir: Optional[str] = None,
                          json_state_dir: Optional[str] = None
                          ) -> ResolvedRuntime:
    """Read the DO's runtime state, preferring <uid>.runtime.toml.

    Falls back to the legacy JSON's last_known_freq_offset_ppb +
    dac_code_by_temperature["last"] (one-shot deprecation), then to an
    all-None ResolvedRuntime when neither exists.
    """
    rs = do_schema.load_runtime_state(do_uid, dos_dir=dos_dir)
    if rs is not None:
        return ResolvedRuntime(
            freq_offset_ppb=rs.last_known_freq_offset_ppb,
            dac_code=rs.last_known_dac_code, source=SOURCE_RUNTIME_TOML)
    legacy = _legacy_runtime(do_uid, json_state_dir)
    if legacy is not None:
        _warn_legacy_once(do_uid)
        return legacy
    return ResolvedRuntime(freq_offset_ppb=None, dac_code=None,
                           source=SOURCE_NONE)


def _save(do_uid: str, freq_ppb: Optional[float], dac_code: Optional[int],
          dos_dir: Optional[str]) -> None:
    rs = do_schema.RuntimeState(
        last_known_freq_offset_ppb=(0.0 if freq_ppb is None
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
    """Runtime-schema-aware warm-start check (mirrors
    do_state.is_do_warm_startable, but reads <uid>.runtime.toml first).

    Pass criteria: a runtime source exists, its freq is finite and within
    ±max_ppb, and the backing file's mtime is within max_age_s.  Returns
    (bool, info-dict) just like the legacy helper.
    """
    toml_path = do_schema._runtime_path(do_uid, dos_dir)
    if os.path.exists(toml_path):
        path, source = toml_path, SOURCE_RUNTIME_TOML
    else:
        from peppar_fix import do_state
        json_path = do_state._do_path(do_uid, json_state_dir)
        if not os.path.exists(json_path):
            return (False, {"reason": "no_state_file"})
        path, source = json_path, SOURCE_LEGACY

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

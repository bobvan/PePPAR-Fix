"""Transition bridge — resolve a DO's engine-consumed characterization.

The engine consumes a small fixed set of scalars from a DO's
characterization: σ_do_phase, σ_do_freq (Q[2,2]/Q[3,3]), the coast-cap
TDEV power law, and σ_q (actuator).  Two storage formats coexist during
the doCharacterizationArchitecture migration:

  NEW  state/dos/<uid>.toml      — do_schema.load_do_characterization,
                                    one quantity per section, class
                                    defaults fill absent sections.
  OLD  state/dos/<uid>.json      — do_state.load_do_state + the
                                    derive_* heuristics, σ_q from
                                    --dac-ppb-per-code.

This module is the single place the engine asks "what are this DO's Q
inputs, and where did each come from".  It PREFERS the new schema and
FALLS BACK to the legacy JSON so the engine keeps running on un-migrated
hosts (PR 2 of doSchemaEngineIntegration ships before the per-host
migration in PR 3).  The hard "refuse unregistered DO" gate is PR 4
(burn-down), at which point this bridge and the legacy path are deleted.

Provenance is carried through verbatim so the engine can log it and so
the steering-MISSING refuse-to-actuate policy has something to gate on.
"""
from __future__ import annotations

import logging
import os
import warnings
from dataclasses import dataclass, field
from typing import Optional

from peppar_fix import do_schema

log = logging.getLogger(__name__)


# One-shot deprecation per do_uid (don't spam the log every checkpoint).
_warned_legacy: set[str] = set()


# Provenance schema label.  "toml" = new schema; "legacy-json" =
# migrated-from-old; "none" = neither file present (engine falls back to
# its own _PHASE_FALLBACK until PR 4 makes this fatal).
SCHEMA_TOML = "toml"
SCHEMA_LEGACY = "legacy-json"
SCHEMA_NONE = "none"


@dataclass
class EngineCharacterization:
    """The scalars the engine consumes, plus per-section provenance.

    Any of the Q scalars may be None when neither storage format
    supplies them; the engine keeps its existing fallback for those.
    """
    schema: str                                   # SCHEMA_*
    sigma_do_phase_ns: Optional[float] = None     # Q[2,2]^0.5
    sigma_do_freq_ppb: Optional[float] = None     # Q[3,3]^0.5
    # (tdev_ref_ns, slope, tau_ref_s) for the longTauGnssCoupling cap,
    # tau_ref fixed at 1.0 s so tdev_ref_ns IS TDEV(1 s).  None ⇒ no cap.
    coast_tdev: Optional[tuple] = None
    sigma_q_ns: Optional[float] = None            # actuator per-write noise
    steering: dict = field(default_factory=dict)  # measured steering, else {}
    provenance: dict = field(default_factory=dict)  # section -> source str

    @property
    def steering_provenance(self) -> str:
        return self.provenance.get("steering", "absent")


def _resolve_from_toml(do_uid: str, dos_dir: Optional[str]
                       ) -> EngineCharacterization:
    c = do_schema.load_do_characterization(do_uid, dos_dir=dos_dir)
    fr = c.freerun_noise
    coast = None
    ref = fr.get("coast_tdev_ref_ns")
    slope = fr.get("coast_tdev_slope")
    if ref is not None and slope is not None:
        # tau_ref fixed at 1 s by schema; matches derive_coast_tdev_from_char.
        coast = (float(ref), float(slope), 1.0)
    sq = c.actuation_noise.get("sigma_q_ns")
    return EngineCharacterization(
        schema=SCHEMA_TOML,
        sigma_do_phase_ns=(None if fr.get("sigma_do_phase_ns") is None
                           else float(fr["sigma_do_phase_ns"])),
        sigma_do_freq_ppb=(None if fr.get("sigma_do_freq_ppb") is None
                           else float(fr["sigma_do_freq_ppb"])),
        coast_tdev=coast,
        sigma_q_ns=(None if sq is None else float(sq)),
        steering=dict(c.steering),
        provenance=dict(c.provenance))


def _resolve_from_legacy(do_uid: str, json_state_dir: Optional[str],
                         dac_ppb_per_code: Optional[float]
                         ) -> Optional[EngineCharacterization]:
    from peppar_fix import do_state
    ds = do_state.load_do_state(do_uid, state_dir=json_state_dir)
    if not ds:
        return None
    if do_uid not in _warned_legacy:
        _warned_legacy.add(do_uid)
        warnings.warn(
            f"DO {do_uid!r} loaded from legacy state/dos/{do_uid}.json — "
            f"run scripts/migrate_do_state.py to convert to the per-section "
            f"schema (state/dos/{do_uid}.toml).  Legacy path is removed in "
            f"the doCharacterizationArchitecture burn-down (PR 4).",
            DeprecationWarning, stacklevel=2)
        log.warning("DO %s using LEGACY JSON characterization — migrate to "
                    ".toml (do_char_resolve fallback; removed in PR 4)",
                    do_uid)
    prov: dict = {}
    sigma_phase = sigma_freq = None
    coast = None
    char = ds.get("characterization")
    if char:
        derived = do_state.derive_do_process_noise(char) or {}
        if derived.get("sigma_do_phase_ns") is not None:
            sigma_phase = float(derived["sigma_do_phase_ns"])
            prov["freerun_noise.sigma_do_phase_ns"] = (
                f"legacy-json:{derived.get('sigma_do_phase_source')}")
        if derived.get("sigma_do_freq_ppb") is not None:
            sigma_freq = float(derived["sigma_do_freq_ppb"])
            prov["freerun_noise.sigma_do_freq_ppb"] = (
                f"legacy-json:{derived.get('sigma_do_freq_source')}")
        coast = do_state.derive_coast_tdev_from_char(char)
        if coast is not None:
            prov["freerun_noise.coast_tdev"] = "legacy-json:derive_coast_tdev"
    # σ_q: legacy engine derived it from --dac-ppb-per-code (ppb ≈ ns @ 1 s).
    sigma_q = None
    if dac_ppb_per_code is not None and dac_ppb_per_code != 0.0:
        sigma_q = abs(float(dac_ppb_per_code))
        prov["actuation_noise.sigma_q_ns"] = "legacy-json:dac_ppb_per_code"
    # Legacy steering provenance is unknown to this layer (the DAC slope
    # lives in args / a separate cal file), so don't assert MISSING — the
    # refuse-to-actuate gate only applies on the new .toml path.
    prov["steering"] = "legacy-json"
    return EngineCharacterization(
        schema=SCHEMA_LEGACY, sigma_do_phase_ns=sigma_phase,
        sigma_do_freq_ppb=sigma_freq, coast_tdev=coast, sigma_q_ns=sigma_q,
        steering={}, provenance=prov)


def resolve_engine_characterization(
        do_uid: str, *, dos_dir: Optional[str] = None,
        json_state_dir: Optional[str] = None,
        dac_ppb_per_code: Optional[float] = None) -> EngineCharacterization:
    """Resolve a DO's engine Q inputs, preferring the new .toml schema.

    Order:
      1. state/dos/<uid>.toml present → new schema (class defaults fill
         absent sections; full provenance).
      2. else state/dos/<uid>.json present → legacy heuristics + a
         one-shot DeprecationWarning.
      3. else → SCHEMA_NONE with all-None scalars (engine keeps its own
         fallback; PR 4 makes this a hard refusal).

    `dos_dir` / `json_state_dir` override the default state dirs (tests).
    `dac_ppb_per_code` supplies the legacy σ_q only.
    """
    toml_path = do_schema._char_path(do_uid, dos_dir)
    if os.path.exists(toml_path):
        return _resolve_from_toml(do_uid, dos_dir)
    legacy = _resolve_from_legacy(do_uid, json_state_dir, dac_ppb_per_code)
    if legacy is not None:
        return legacy
    return EngineCharacterization(schema=SCHEMA_NONE,
                                  provenance={"steering": "absent"})


def format_provenance_log(do_uid: str, ec: EngineCharacterization) -> list[str]:
    """Operator-facing provenance lines for engine startup.

    Class-default and MISSING sections get a loud trailing call to action
    (per docs §"Class defaults" — a default is not a measurement).
    """
    lines = [f"DO {do_uid} characterization source = {ec.schema}"]
    for section, src in sorted(ec.provenance.items()):
        nudge = ""
        if "class-default" in src:
            tool = {"freerun_noise": "do_freerun_char.py",
                    "actuation_noise": "do_actuator_char.py",
                    "steering": "do_steering_char.py"}.get(section, "do_*_char.py")
            nudge = f"   ← RUN {tool} FOR A REAL MEASUREMENT"
        elif src == "MISSING":
            nudge = "   ← REQUIRED (run do_steering_char.py) — engine will refuse to actuate"
        lines.append(f"  [{section}] {src}{nudge}")
    return lines

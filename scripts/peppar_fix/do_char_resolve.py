"""Engine adapter — resolve a DO's engine-consumed characterization.

The engine consumes a small fixed set of scalars from a DO's
characterization: σ_do_phase, σ_do_freq (Q[2,2]/Q[3,3]), the coast-cap
TDEV power law, and σ_q (actuator).  These all live in the per-section
schema at state/dos/<uid>.toml (do_schema.load_do_characterization;
class defaults fill absent sections).

This module is the single place the engine asks "what are this DO's Q
inputs, and where did each come from".  Post burn-down (PR 4) there is
exactly ONE storage format: an unregistered DO (no .toml) RAISES — the
engine refuses to discipline it — and there is no silent Q fallback.
(The legacy <uid>.json path was removed in the burn-down; the migration
tool scripts/migrate_do_state.py is the only remaining reader of it.)

Provenance is carried through verbatim so the engine can log it and so
the steering-MISSING refuse-to-actuate policy has something to gate on.
"""
from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field
from typing import Optional

from peppar_fix import do_schema

log = logging.getLogger(__name__)


# Provenance schema label.  Only the new per-section schema remains.
SCHEMA_TOML = "toml"


@dataclass
class EngineCharacterization:
    """The scalars the engine consumes, plus per-section provenance.

    For a registered DO the .toml schema always supplies the Q scalars
    (measured or class-default), so they are non-None; an unregistered DO
    raises in resolve_engine_characterization rather than yielding Nones.
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


def resolve_engine_characterization(
        do_uid: str, *, dos_dir: Optional[str] = None,
        json_state_dir: Optional[str] = None,
        dac_ppb_per_code: Optional[float] = None) -> EngineCharacterization:
    """Resolve a DO's engine Q inputs from the per-section .toml schema.

    Burn-down (PR 4): the legacy <uid>.json fallback is GONE — the engine
    reads ONLY state/dos/<uid>.toml.  An unregistered DO (no .toml) RAISES
    SchemaError ("refuse unregistered DOs"); a present-but-invalid .toml
    also raises (propagated, not swallowed) — both are fatal at the caller.
    There is no longer any silent Q fallback.  (json_state_dir /
    dac_ppb_per_code are accepted for signature stability but unused now
    that the legacy path is removed.)
    """
    toml_path = do_schema._char_path(do_uid, dos_dir)
    if not os.path.exists(toml_path):
        raise do_schema.SchemaError(
            f"DO {do_uid!r} is not registered: no {toml_path}.  Run "
            f"scripts/do_register.py (then the do_*_char.py tools, or "
            f"scripts/migrate_do_state.py for a legacy host).  The engine "
            f"refuses to discipline an unregistered DO — there is no longer "
            f"a silent Q fallback.")
    return _resolve_from_toml(do_uid, dos_dir)


def should_refuse_for_steering(ec: EngineCharacterization, *,
                               has_servo: bool,
                               allow_default_steering: bool) -> bool:
    """Engine policy: refuse to actuate an uncalibrated actuator.

    True only when ALL hold: [steering] provenance is not "measured"
    (class-default or MISSING), a servo/actuator is configured, and the
    operator hasn't opted in with --allow-default-steering.  The escape
    hatch stays until do_steering_char has populated measured [steering] on
    the hosts (they currently run with --allow-default-steering); only then
    is the flag removed and uncalibrated actuation refused unconditionally.
    """
    return (ec.schema == SCHEMA_TOML
            and ec.steering_provenance != "measured"
            and has_servo
            and not allow_default_steering)


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

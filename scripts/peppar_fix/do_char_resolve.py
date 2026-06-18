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
    actuator_type: Optional[str] = None           # [identity].actuator_type
    provenance: dict = field(default_factory=dict)  # section -> source str

    @property
    def steering_provenance(self) -> str:
        return self.provenance.get("steering", "absent")


# Actuators whose steering gain is a PER-UNIT quantity that must be MEASURED
# (the DAC's EFC slope, ppb/code, varies per oscillator → do_steering_char).
# PHC_adjfine and ClockMatrix_FCW are deliberately absent: their gain is an
# exact, fixed hardware constant (the PTP-clock / ClockMatrix register→freq
# mapping), so there is nothing to characterize and no [steering] measurement
# to demand — the refuse gate must not fire on them.
STEERING_MEASURED_REQUIRED = ("DAC",)


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
        actuator_type=c.identity.get("actuator_type"),
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
                               has_servo: bool) -> bool:
    """Engine policy: refuse to actuate an actuator whose MEASURABLE steering
    gain was never measured.

    True only when ALL hold: a servo/actuator is configured, the DO is on the
    .toml schema, the actuator's gain is a per-unit MEASURABLE quantity
    (STEERING_MEASURED_REQUIRED — DAC), and its [steering] provenance is not
    "measured".  Actuator-aware: a PHC_adjfine / ClockMatrix_FCW gain is an
    intrinsic hardware constant (nothing to measure), so the gate never fires
    on it — which is why TimeHat no longer needs an escape hatch.  There is no
    --allow-default-steering: a DAC DO must be characterized (do_steering_char)
    before it can discipline; an uncharacterized DAC is fatal.
    """
    return (has_servo
            and ec.schema == SCHEMA_TOML
            and ec.actuator_type in STEERING_MEASURED_REQUIRED
            and ec.steering_provenance != "measured")


def resolve_dac_actuator_params(
        do_uid: Optional[str], *, dos_dir: Optional[str] = None,
        config_ppb_per_code: Optional[float] = None) -> Optional[dict]:
    """Source DAC actuator parameters from the DO's MEASURED [steering].

    The EFC slope (ppb/code) of a VCOCXO+DAC is a PER-UNIT quantity — an
    OCXO's pull range is configurable over a wide range, so a guessed/default
    slope is an accident waiting to happen.  This function makes the measured
    [steering] the single source of truth for the actuator's gain, code range,
    and gain mode.  (Previously the engine built the actuator from
    --dac-ppb-per-code and silently ignored the measured slope.)

    Returns a dict of DacActuator kwargs (ppb_per_code, center_code, code_min,
    code_max, and dac_gain when recorded) when the DO has measured steering.

    Raises SchemaError when the DO is a DAC actuator whose steering was never
    measured — the engine must STOP, not fall back to a default.  (Mirrors
    should_refuse_for_steering, but at actuator-construction time so no default
    slope can reach the loop.)

    Returns None when the actuator is not a per-unit-measured type (PHC_adjfine
    / ClockMatrix_FCW have intrinsic constant gains) or when do_uid is None
    (no registered DO — the legacy config path still applies for that case).
    """
    if do_uid is None:
        return None
    c = do_schema.load_do_characterization(do_uid, dos_dir=dos_dir)
    if c.identity.get("actuator_type") not in STEERING_MEASURED_REQUIRED:
        return None  # intrinsic-gain actuator — nothing to source from steering
    if c.provenance.get("steering") != "measured":
        raise do_schema.SchemaError(
            f"DO {do_uid!r}: [steering] provenance is "
            f"{c.provenance.get('steering', 'absent')!r}, not 'measured' — "
            f"refusing to discipline a DAC on a non-measured EFC slope.  Run "
            f"scripts/do_steering_char.py first.  There is no default ppb/code: "
            f"an OCXO's pull range is per-unit, so a guessed slope is unsafe.")
    st = c.steering
    params: dict = {"ppb_per_code": float(st["slope_ppb_per_code"])}
    for key in ("parked_code", "code_min", "code_max"):
        if st.get(key) is not None:
            params["center_code" if key == "parked_code" else key] = int(st[key])
    g = st.get("dac_gain")
    if g is not None:
        params["dac_gain"] = int(g)
    else:
        log.warning(
            "DO %s: [steering] has no dac_gain — cannot verify the slope was "
            "measured at the runtime DAC gain mode.  Re-run do_steering_char "
            "to record it; using the config/default gain meanwhile.", do_uid)
    slope = params["ppb_per_code"]
    if (config_ppb_per_code is not None and config_ppb_per_code != 0.0
            and abs(config_ppb_per_code - slope) > 0.1 * abs(slope)):
        log.warning(
            "DO %s: config --dac-ppb-per-code=%.5f disagrees with measured "
            "[steering] slope=%.5f (>10%%).  USING THE MEASURED value; remove "
            "dac_ppb_per_code from the host config to avoid confusion.",
            do_uid, config_ppb_per_code, slope)
    return params


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

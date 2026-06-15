"""DO characterization schema, validator, class defaults, single loader.

Implements docs/do-characterization-architecture.md.  Two files per DO:

  state/dos/<uid>.toml          — characterization (tool-write-only)
  state/dos/<uid>.runtime.toml  — operational state (engine-write-only)

The split is structural, not stylistic.  The engine reads only the
characterization file at startup and writes only the runtime file at
checkpoint.  Tools write only the characterization file, and only into
their owned section.  Both behaviors are enforced by separate functions
that operate on separate paths — there is no shared writer that could
be misused.

Fail-loud everywhere:

- Unrecognized schema_version  → refuse to load.
- Receiver-side or control-loop entries in [freerun_noise]
  measurement_channel  → refuse to write (the PiFace-style mistake
  is unrepresentable at this layer).
- coast_tdev_slope ≤ 0  → refuse to write (a non-positive slope
  silently disables Goldilocks at runtime, which is the kind of
  bug we wrote this doc to prevent).
- Negative or zero sigma values  → refuse to write.
- Missing [identity]  → refuse to load.

Class defaults: per docs §"Class defaults", each conservative
starting point is looser than the worst-real-unit measurement we have
in the lab, so a default-using DO is never implicitly asserted to be
tighter than a measured DO of the same class.
"""
from __future__ import annotations

import logging
import os
import tempfile
import tomllib
from dataclasses import dataclass, field
from typing import Any

log = logging.getLogger(__name__)


SCHEMA_VERSION = "1"


# Default directory.  Overridable per call (for tests, alt-state-dir).
DEFAULT_DOS_DIR = os.path.expanduser("~/peppar-fix/state/dos")


# Allowed measurement channels for [freerun_noise].
#
# DO-pointing only.  Receiver-side ("Carrier", "dt_rx", "qErr") and
# control-loop ("adjfine", "freq_command") channels are rejected by
# this enum — the PiFace cascade was caused by control-loop and
# receiver-side measurements landing in the same `sources` dict and
# being treated as DO observations.  Schema-level rejection makes
# that mistake unrepresentable.
#
# Preference order matches docs §"Topology":
#   1. chA vs Rb  — Rb is the cleanest reference
#   2. chA-chB    — carries rx-TCXO motion but usually DO-dominated
ALLOWED_MEASUREMENT_CHANNELS = (
    "DO PPS (chA vs TICC Rb)",
    "DO PPS (chA-chB)",
)


# Allowed DO classes.  Maps to CLASS_DEFAULTS keys.
ALLOWED_CLASSES = ("OCXO", "TCXO", "Rb", "PHC")


# Allowed actuator types.  Maps to FrequencyActuator implementations.
ALLOWED_ACTUATOR_TYPES = ("DAC", "PHC_adjfine", "ClockMatrix_FCW")


# Class defaults per docs §"Class defaults".  Numbers are argued in PR
# review; the structure (per-class table, looser than worst-real-unit)
# is what matters here.
#
# Lineage column in the doc cites which real unit each default must
# beat — if a measurement appears that's tighter than the default, the
# default isn't loose enough.
CLASS_DEFAULTS: dict[str, dict[str, float]] = {
    "OCXO": {
        # ~2.6× looser than clkPoC3 measured (sigma_do_phase=0.0425,
        # sigma_do_freq=0.000382).  Leaves headroom for noisier OCXOs.
        "sigma_do_phase_ns": 0.1,
        "sigma_do_freq_ppb": 0.001,
        "coast_tdev_ref_ns": 0.1,
        "coast_tdev_slope": 0.5,    # white-FM conservative
        "sigma_q_ns": 0.05,
    },
    "TCXO": {
        # TimeHat measured TDEV(1s) = 2.6 ns/√s — class default must be
        # looser, hence 3.0.
        "sigma_do_phase_ns": 3.0,
        "sigma_do_freq_ppb": 0.1,
        "coast_tdev_ref_ns": 3.0,
        "coast_tdev_slope": 1.0,    # flicker-FM conservative
        "sigma_q_ns": 0.1,
    },
    "Rb": {
        # Beats most OCXOs by design.  A Rb that needs class-default is
        # broken — but the schema still has to provide values.
        "sigma_do_phase_ns": 0.01,
        "sigma_do_freq_ppb": 0.0001,
        "coast_tdev_ref_ns": 0.01,
        "coast_tdev_slope": 0.5,
        "sigma_q_ns": 0.01,
    },
    "PHC": {
        # Empirically from [[adaptive-scheduler-motivation-timehat]] —
        # 1 Hz actuation worse than coasting implies actuator σ_q is
        # ns-class, not the 15-fs LSB floor.
        "sigma_do_phase_ns": 0.5,
        "sigma_do_freq_ppb": 0.1,
        "coast_tdev_ref_ns": 0.5,
        "coast_tdev_slope": 1.0,
        "sigma_q_ns": 0.5,
    },
}


@dataclass(frozen=True)
class Characterization:
    """Loaded characterization, with section-by-section provenance.

    Each section's `source` field tells the engine whether the values
    are measured or class-default.  The engine logs this provenance at
    startup so an operator can see at a glance which Q values came
    from actual measurements vs class fallbacks.
    """
    do_uid: str
    identity: dict[str, Any]
    steering: dict[str, Any]
    freerun_noise: dict[str, Any]
    actuation_noise: dict[str, Any]
    aging: dict[str, Any]
    # Per-section provenance: "measured" or "class-default[<class>]"
    provenance: dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class RuntimeState:
    """Engine-owned runtime state.  Written every save_do_state checkpoint."""
    last_known_freq_offset_ppb: float
    last_known_dac_code: int | None
    last_updated: str


class SchemaError(ValueError):
    """Raised when a TOML payload violates the schema."""


# ── Validation ───────────────────────────────────────────────────── #


def validate_characterization(data: dict) -> list[str]:
    """Return a list of validation errors; empty list = clean."""
    errors: list[str] = []

    v = data.get("schema_version")
    if v != SCHEMA_VERSION:
        errors.append(
            f"schema_version must be {SCHEMA_VERSION!r}, got {v!r} "
            f"— refusing to load (no silent forward-compat)"
        )

    # [identity] required, all subfields required.
    ident = data.get("identity")
    if not isinstance(ident, dict):
        errors.append("missing [identity] section")
    else:
        for k in ("do_uid", "model", "class", "actuator_type",
                  "nominal_freq_hz"):
            if k not in ident:
                errors.append(f"[identity].{k} required")
        if ident.get("class") not in ALLOWED_CLASSES:
            errors.append(
                f"[identity].class must be one of {list(ALLOWED_CLASSES)}, "
                f"got {ident.get('class')!r}"
            )
        if (ident.get("actuator_type") is not None and
                ident.get("actuator_type") not in ALLOWED_ACTUATOR_TYPES):
            errors.append(
                f"[identity].actuator_type must be one of "
                f"{list(ALLOWED_ACTUATOR_TYPES)}, "
                f"got {ident.get('actuator_type')!r}"
            )

    # [freerun_noise] validation.  Optional section — class defaults
    # fill in when absent.  When present and source=measured, the full
    # contract applies.
    fr = data.get("freerun_noise")
    if isinstance(fr, dict) and fr.get("source") == "measured":
        ch = fr.get("measurement_channel")
        if ch not in ALLOWED_MEASUREMENT_CHANNELS:
            errors.append(
                f"[freerun_noise].measurement_channel must be one of "
                f"{list(ALLOWED_MEASUREMENT_CHANNELS)}, got {ch!r}.  "
                f"Receiver-side ('Carrier', 'dt_rx', 'qErr', ...) and "
                f"control-loop ('adjfine', 'freq_command', ...) channels "
                f"are rejected by design — they don't see the DO."
            )
        slope = fr.get("coast_tdev_slope")
        if slope is not None and slope <= 0:
            errors.append(
                f"[freerun_noise].coast_tdev_slope must be > 0 "
                f"(positive RWFM/flicker rising-tail slope), got {slope!r}.  "
                f"A slope ≤ 0 would silently disable Goldilocks at runtime "
                f"(compute_goldilocks_tau treats slope ≤ 0 as degenerate "
                f"and returns max_interval, scheduler quietly off)."
            )
        for k in ("coast_tdev_ref_ns", "sigma_do_phase_ns",
                  "sigma_do_freq_ppb"):
            v = fr.get(k)
            if v is not None and v <= 0:
                errors.append(f"[freerun_noise].{k} must be > 0, got {v!r}")

    # [actuation_noise] validation
    an = data.get("actuation_noise")
    if isinstance(an, dict):
        sq = an.get("sigma_q_ns")
        if sq is not None and sq <= 0:
            errors.append(
                f"[actuation_noise].sigma_q_ns must be > 0, got {sq!r}"
            )

    # [steering] validation — slope must be non-zero (sign-meaningful)
    st = data.get("steering")
    if isinstance(st, dict) and st.get("source") == "measured":
        slope = st.get("slope_ppb_per_code")
        if slope is not None and slope == 0:
            errors.append(
                "[steering].slope_ppb_per_code must be non-zero "
                "(measured slope of zero is a calibration failure, "
                "not a valid DO)"
            )

    return errors


def class_defaults(do_class: str) -> dict[str, float]:
    """Return a fresh copy of class-default values for a given class."""
    if do_class not in CLASS_DEFAULTS:
        raise SchemaError(
            f"Unknown DO class {do_class!r}; known: {list(CLASS_DEFAULTS)}"
        )
    return dict(CLASS_DEFAULTS[do_class])


# ── Loader ───────────────────────────────────────────────────────── #


def _char_path(do_uid: str, dos_dir: str | None = None) -> str:
    return os.path.join(dos_dir or DEFAULT_DOS_DIR, f"{do_uid}.toml")


def _runtime_path(do_uid: str, dos_dir: str | None = None) -> str:
    return os.path.join(dos_dir or DEFAULT_DOS_DIR,
                        f"{do_uid}.runtime.toml")


def load_do_characterization(do_uid: str,
                              dos_dir: str | None = None) -> Characterization:
    """Load + validate <uid>.toml.

    Fills in class-default values for any section marked
    source = "class-default" or absent entirely.  Raises SchemaError
    on any validation failure.
    """
    path = _char_path(do_uid, dos_dir)
    try:
        with open(path, "rb") as f:
            data = tomllib.load(f)
    except (tomllib.TOMLDecodeError, OSError) as e:
        raise SchemaError(
            f"Failed to read {path}: {e}.  "
            f"Engine refuses to start without a registered DO file.  "
            f"Run scripts/do_register.py to bootstrap."
        ) from e

    errors = validate_characterization(data)
    if errors:
        msg = "; ".join(errors)
        raise SchemaError(f"{path}: {msg}")

    ident = data["identity"]  # validated present above
    do_class = ident["class"]
    defaults = class_defaults(do_class)

    # Resolve per-section provenance.  A section with
    # source = "class-default" (or absent) inherits the class
    # defaults; the engine logs which case applies.
    provenance: dict[str, str] = {}

    def _resolve(section_name: str, source_specific_keys: tuple) -> dict:
        """Pull from the file if measured, else fill from class defaults."""
        section = data.get(section_name)
        if isinstance(section, dict) and section.get("source") == "measured":
            provenance[section_name] = "measured"
            return dict(section)
        # Class-default path: build a minimal dict with the keys the
        # engine consumes, sourced from the class defaults table.
        provenance[section_name] = f"class-default[{do_class}]"
        out = {"source": "class-default"}
        for k in source_specific_keys:
            if k in defaults:
                out[k] = defaults[k]
        return out

    freerun = _resolve("freerun_noise",
                       ("sigma_do_phase_ns", "sigma_do_freq_ppb",
                        "coast_tdev_ref_ns", "coast_tdev_slope"))
    actuation = _resolve("actuation_noise", ("sigma_q_ns",))

    # [steering] doesn't get class-default fallback — a DO without
    # measured steering can't actuate.  The engine refuses to run
    # against a class-default steering, but the loader still returns
    # the section so the operator sees the provenance.
    steering_section = data.get("steering")
    if isinstance(steering_section, dict) and steering_section.get("source") == "measured":
        provenance["steering"] = "measured"
        steering = dict(steering_section)
    else:
        provenance["steering"] = "MISSING"
        steering = {}

    # [aging] is optional and informational.
    aging_section = data.get("aging") or {}
    provenance["aging"] = ("measured" if aging_section else "absent")
    aging = dict(aging_section)

    provenance["identity"] = "measured"

    return Characterization(
        do_uid=ident["do_uid"],
        identity=dict(ident),
        steering=steering,
        freerun_noise=freerun,
        actuation_noise=actuation,
        aging=aging,
        provenance=provenance,
    )


def load_runtime_state(do_uid: str,
                       dos_dir: str | None = None) -> RuntimeState | None:
    """Load <uid>.runtime.toml.  Returns None if absent (cold start)."""
    path = _runtime_path(do_uid, dos_dir)
    if not os.path.exists(path):
        return None
    try:
        with open(path, "rb") as f:
            data = tomllib.load(f)
    except (tomllib.TOMLDecodeError, OSError) as e:
        log.warning("Failed to read runtime state at %s: %s", path, e)
        return None
    op = data.get("operational_state") or {}
    return RuntimeState(
        last_known_freq_offset_ppb=float(
            op.get("last_known_freq_offset_ppb", 0.0)),
        last_known_dac_code=op.get("last_known_dac_code"),
        last_updated=op.get("last_updated", ""),
    )


# ── Saver (runtime only — engine never writes characterization) ──── #


def _format_runtime_toml(rs: RuntimeState) -> str:
    """Hand-format runtime TOML.  Flat, no comments, atomic-writable.

    We hand-format rather than depend on tomli_w because:
      - schema is fixed and trivial (4 fields)
      - no new dependency (project convention; see
        position_state._format_toml)
      - operator-readable output we control exactly
    """
    lines = [
        f"schema_version = \"{SCHEMA_VERSION}\"",
        "",
        "[operational_state]",
        f"last_known_freq_offset_ppb = {rs.last_known_freq_offset_ppb:.6f}",
    ]
    if rs.last_known_dac_code is not None:
        lines.append(f"last_known_dac_code = {int(rs.last_known_dac_code)}")
    lines.append(f"last_updated = \"{rs.last_updated}\"")
    return "\n".join(lines) + "\n"


def save_runtime_state(do_uid: str, rs: RuntimeState,
                       dos_dir: str | None = None) -> None:
    """Atomically write <uid>.runtime.toml.

    Never opens <uid>.toml — that's the structural firewall between
    engine writes and tool writes.  Per docs §"Schema rules" rule 5.
    """
    path = _runtime_path(do_uid, dos_dir)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    text = _format_runtime_toml(rs)
    # Atomic write via temp + rename.
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path),
                                prefix=f".{do_uid}.runtime.",
                                suffix=".toml.tmp")
    try:
        with os.fdopen(fd, "w") as f:
            f.write(text)
        os.replace(tmp, path)
    except Exception:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise

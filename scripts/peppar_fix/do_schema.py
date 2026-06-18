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
#
# Repo-relative (matches do_state.DO_STATE_DIR) so the new schema and the
# legacy JSON resolve to the SAME state/dos on every host.  On lab hosts the
# checkout IS ~/peppar-fix so this equals ~/peppar-fix/state/dos; the earlier
# hard-coded expanduser diverged from do_state on the dev box (repo =
# ~/git/PePPAR-Fix), a migration/tooling trap flagged in PR #176 review.
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_DOS_DIR = os.path.join(_REPO_ROOT, "state", "dos")


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
    """Engine-owned runtime state.  Written every save_do_state checkpoint.

    last_known_freq_offset_ppb is Optional: a freq that was never measured
    must read back as None, NOT a misleading 0.0 — otherwise a runtime file
    written by a DAC-code-only checkpoint before any freq is known would
    warm-start the DO at 0.0 ppb instead of "unknown" (Main's PR #176 nit).
    """
    last_known_freq_offset_ppb: float | None
    last_known_dac_code: int | None
    last_updated: str


class SchemaError(ValueError):
    """Raised when a TOML payload violates the schema."""


# ── Validation ───────────────────────────────────────────────────── #


# Required keys for each measured section.  When a section has
# source = "measured", these keys must be present AND positive (or
# non-zero for steering slope which is sign-meaningful).  The
# class-default path builds these keys explicitly from CLASS_DEFAULTS,
# so the measured path needs the same completeness guarantee — else a
# measured section missing a key would silently fall through to the
# engine, which is the exact silent-incompleteness this module is here
# to prevent.
_REQUIRED_MEASURED_KEYS: dict[str, tuple[str, ...]] = {
    "freerun_noise": (
        "sigma_do_phase_ns",
        "sigma_do_freq_ppb",
        "coast_tdev_ref_ns",
        "coast_tdev_slope",
    ),
    "actuation_noise": (
        "sigma_q_ns",
    ),
    "steering": (
        "slope_ppb_per_code",
    ),
}


def _check_measured_required(section_name: str, section: dict,
                              required: tuple[str, ...]) -> list[str]:
    """Enforce that a source=measured section has every consumed key.

    Returns errors for missing keys.  Positivity is checked by the
    individual validation blocks below (which also catch the
    class-default path's invariants).
    """
    errors: list[str] = []
    for key in required:
        if key not in section:
            errors.append(
                f"[{section_name}].{key} required when source='measured' "
                f"(present in CLASS_DEFAULTS but missing in the file — "
                f"this is the silent-incompleteness this schema prevents)"
            )
    return errors


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
        if ident.get("actuator_type") not in ALLOWED_ACTUATOR_TYPES:
            # Required-keys loop above catches the missing case; this
            # only fires when actuator_type is present but unrecognized.
            if "actuator_type" in ident:
                errors.append(
                    f"[identity].actuator_type must be one of "
                    f"{list(ALLOWED_ACTUATOR_TYPES)}, "
                    f"got {ident.get('actuator_type')!r}"
                )

    # [freerun_noise] validation.  Optional section — class defaults
    # fill in when absent.  When present and source=measured, the full
    # contract applies: every consumed key must be present AND positive.
    fr = data.get("freerun_noise")
    if isinstance(fr, dict) and fr.get("source") == "measured":
        errors.extend(_check_measured_required(
            "freerun_noise", fr, _REQUIRED_MEASURED_KEYS["freerun_noise"]))
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
    if isinstance(an, dict) and an.get("source") == "measured":
        errors.extend(_check_measured_required(
            "actuation_noise", an,
            _REQUIRED_MEASURED_KEYS["actuation_noise"]))
        sq = an.get("sigma_q_ns")
        if sq is not None and sq <= 0:
            errors.append(
                f"[actuation_noise].sigma_q_ns must be > 0, got {sq!r}"
            )

    # [steering] validation — slope must be non-zero (sign-meaningful)
    st = data.get("steering")
    if isinstance(st, dict) and st.get("source") == "measured":
        errors.extend(_check_measured_required(
            "steering", st, _REQUIRED_MEASURED_KEYS["steering"]))
        slope = st.get("slope_ppb_per_code")
        if slope is not None and slope == 0:
            errors.append(
                "[steering].slope_ppb_per_code must be non-zero "
                "(measured slope of zero is a calibration failure, "
                "not a valid DO)"
            )
        # dac_gain (optional, back-compat): the DAC output-stage gain mode
        # the slope was MEASURED at.  A slope is only valid at the same gain
        # mode it was characterized under (1× vs 2× changes the volts/code,
        # hence ppb/code).  Recording it lets the engine set the DAC to the
        # matching gain instead of guessing — see measuredSteeringIsAuthoritative.
        g = st.get("dac_gain")
        if g is not None and g not in (0, 1):
            errors.append(
                f"[steering].dac_gain must be 0 (1× mode) or 1 (2× mode), "
                f"got {g!r}"
            )
        # ppb_at_code_min (optional in this schema rev; canonical going
        # forward) — the OCXO pull at the lower code edge.  This is the
        # edge-anchored replacement for parked_code/intercept_ppb_at_parked
        # (noMagicCenterCode): a line is {code_min, code_max, slope,
        # ppb_at_code_min}, no privileged center.  Validated as numeric when
        # present; the loader derives it for legacy files (parked_code +
        # intercept) so consumers can rely on it regardless of file vintage.
        pmin = st.get("ppb_at_code_min")
        if pmin is not None and not isinstance(pmin, (int, float)):
            errors.append(
                f"[steering].ppb_at_code_min must be numeric, got {pmin!r}")
        # Characterization-temperature provenance (optional): the OCXO oven
        # and on-board CPU temperatures the steering curve was MEASURED at.
        # ppb_at_code_min is implicitly ppb_at_code_min_AT_CHAR_TEMP — the
        # GNSS-matching code drifts with temperature, so the reference temp
        # is needed to project headroom/temp-margin (Bob 2026-06-18).  Both
        # optional (not every host has an OCXO oven sensor); plausible range
        # matches temp_sensor.read_onboard_temps.
        for tk in ("char_temp_ocxo_c", "char_temp_cpu_c"):
            tv = st.get(tk)
            if tv is not None and (not isinstance(tv, (int, float))
                                   or not (-40.0 <= tv <= 150.0)):
                errors.append(
                    f"[steering].{tk} must be numeric in −40..150 °C, "
                    f"got {tv!r}")

    return errors


def class_defaults(do_class: str) -> dict[str, float]:
    """Return a fresh copy of class-default values for a given class."""
    if do_class not in CLASS_DEFAULTS:
        raise SchemaError(
            f"Unknown DO class {do_class!r}; known: {list(CLASS_DEFAULTS)}"
        )
    return dict(CLASS_DEFAULTS[do_class])


# ── Loader ───────────────────────────────────────────────────────── #


def safe_uid(do_uid: str) -> str:
    """Filesystem-safe DO uid — MUST match do_state._do_path's mapping.

    A do_uid can be a clean label ("ocxo-clkpoc3"), a MAC ("54:49:4d:45:00:6b"),
    or a device path ("/dev/ptp_i226").  Without this, f"{do_uid}.toml" both
    (a) leaves colons/slashes in the filename and (b) — for a path uid —
    os.path.join ABSOLUTE-PATH-ESCAPES ("/dev/ptp_i226.toml" lands in /dev,
    not under dos_dir), so MAC/path-uid hosts never find their .toml and fall
    back silently to legacy (the doUidTomlPathSanitization bug).  Mirrors
    do_state._do_path exactly so the new schema and legacy JSON co-resolve.
    """
    return str(do_uid).replace(":", "-").replace("/", "_")


def _char_path(do_uid: str, dos_dir: str | None = None) -> str:
    return os.path.join(dos_dir or DEFAULT_DOS_DIR, f"{safe_uid(do_uid)}.toml")


def _runtime_path(do_uid: str, dos_dir: str | None = None) -> str:
    return os.path.join(dos_dir or DEFAULT_DOS_DIR,
                        f"{safe_uid(do_uid)}.runtime.toml")


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
    # do_uid <-> filename mismatch guard.  A renamed or copy-pasted
    # file is exactly the multi-file confusion (clkPoC3 v1/v2) that
    # motivated this whole architecture — a silent identity mismatch
    # would let the engine discipline DO X using DO Y's
    # characterization.  Fail loud.  Compare on the SANITIZED form so the
    # equivalent uid spellings that map to one file (colon vs dash MAC,
    # device-path slashes) are accepted — only a genuine cross-DO mismatch
    # trips this, not "54:49:..6b" (engine) vs "54-49-..6b" (stored).
    if safe_uid(ident.get("do_uid", "")) != safe_uid(do_uid):
        raise SchemaError(
            f"{path}: [identity].do_uid={ident.get('do_uid')!r} does not "
            f"match requested do_uid={do_uid!r}.  Filename and identity "
            f"must agree — a mismatch means a renamed or copy-pasted file "
            f"that would discipline the wrong DO."
        )
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
        # noMagicCenterCode back-compat: expose ppb_at_code_min so consumers
        # can rely on the edge-anchored model regardless of file vintage.
        # Derive it for legacy files (parked_code + intercept) that predate
        # the field — evaluating the same fitted line at code_min:
        #   ppb_at_code_min = intercept_ppb_at_parked
        #                     + slope·(code_min − parked_code)
        if steering.get("ppb_at_code_min") is None:
            slope = steering.get("slope_ppb_per_code")
            cmin = steering.get("code_min")
            parked = steering.get("parked_code")
            intercept = steering.get("intercept_ppb_at_parked")
            if None not in (slope, cmin, parked, intercept):
                steering["ppb_at_code_min"] = (
                    float(intercept) + float(slope) * (float(cmin) - float(parked)))
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
    """Load <uid>.runtime.toml.  Returns None if absent (cold start)
    or on a soft I/O failure.

    Schema-version mismatch returns None and logs a warning rather
    than raising — runtime state is engine-rewritten on the next
    checkpoint, so an unrecognized version means "cold start" not
    "refuse to operate."
    """
    path = _runtime_path(do_uid, dos_dir)
    if not os.path.exists(path):
        return None
    try:
        with open(path, "rb") as f:
            data = tomllib.load(f)
    except (tomllib.TOMLDecodeError, OSError) as e:
        log.warning("Failed to read runtime state at %s: %s", path, e)
        return None
    v = data.get("schema_version")
    if v != SCHEMA_VERSION:
        log.warning(
            "Runtime state at %s has schema_version=%r, expected %r — "
            "treating as cold start; engine will rewrite on next checkpoint",
            path, v, SCHEMA_VERSION)
        return None
    op = data.get("operational_state") or {}
    _freq = op.get("last_known_freq_offset_ppb")
    return RuntimeState(
        last_known_freq_offset_ppb=(None if _freq is None else float(_freq)),
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
    ]
    # Omit the freq line when unknown so it reads back as None, not 0.0.
    if rs.last_known_freq_offset_ppb is not None:
        lines.append("last_known_freq_offset_ppb = "
                     f"{rs.last_known_freq_offset_ppb:.6f}")
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
    # Atomic write via temp + rename.  Sanitize the prefix too — a raw
    # path/MAC uid would put slashes/colons in the temp name.
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path),
                                prefix=f".{safe_uid(do_uid)}.runtime.",
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


# ── Characterization section writer (tool-side; shared by do_*_char) ── #

# Sections that live in the CHARACTERIZATION file (<uid>.toml).
# operational_state is deliberately absent — it lives in the engine-owned
# <uid>.runtime.toml and must never be written through this path (the
# "engine writes only the runtime file" firewall, rule #5).
_CHAR_SECTIONS = ("identity", "steering", "thermal", "freerun_noise",
                  "actuation_noise", "aging")


def _fmt_toml_value(v) -> str:
    """Format a scalar for a hand-written TOML line.  Fail loud on types the
    schema doesn't use (lists/dicts belong in the .noise.toml sidecar)."""
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        return repr(v)
    if isinstance(v, str):
        return '"%s"' % v.replace("\\", "\\\\").replace('"', '\\"')
    raise SchemaError(
        f"unsupported TOML value type {type(v).__name__}: {v!r}")


def _format_characterization_toml(data: dict) -> str:
    """Serialize a characterization dict to TOML, sections in canonical order.

    Hand-formatted (no tomli_w dependency, matching _format_runtime_toml).
    Comments are NOT preserved — the file is tool-managed; values + section
    structure are what round-trip.  Unknown dict sections are preserved
    (appended, sorted) rather than dropped, so a forward-compat field is
    never silently lost.
    """
    lines = [f'schema_version = "{data.get("schema_version", SCHEMA_VERSION)}"']
    extra = sorted(k for k, v in data.items()
                   if isinstance(v, dict) and k not in _CHAR_SECTIONS)
    for sec in list(_CHAR_SECTIONS) + extra:
        body = data.get(sec)
        if not isinstance(body, dict):
            continue
        lines.append("")
        lines.append(f"[{sec}]")
        for k, v in body.items():
            lines.append(f"{k} = {_fmt_toml_value(v)}")
    return "\n".join(lines) + "\n"


def update_characterization_section(do_uid: str, section: str, fields: dict,
                                    *, dos_dir: str | None = None) -> str:
    """Read-modify-write ONE characterization section of <uid>.toml.

    The shared writer for the do_*_char tool family (steering now; freerun /
    actuation as they migrate to the schema).  Loads the existing (registered)
    <uid>.toml, replaces ``[section]`` with ``{source: "measured", **fields}``,
    re-validates the WHOLE file through ``validate_characterization`` (fail
    loud), then atomically writes it.  Refuses to:

      * write operational_state / any non-characterization section (firewall),
      * write a section the schema doesn't define,
      * touch a DO that isn't registered yet (no <uid>.toml),
      * land a file that fails validation (the new section is rejected before
        it can be written — the same guarantee the loader gives on read).

    Returns the path written.
    """
    if section not in _CHAR_SECTIONS:
        raise SchemaError(
            f"refusing to write section {section!r}: not a characterization "
            f"section {list(_CHAR_SECTIONS)} (operational_state is engine-only, "
            f"in <uid>.runtime.toml)")
    path = _char_path(do_uid, dos_dir)
    if not os.path.exists(path):
        raise SchemaError(
            f"{path}: no characterization file — run scripts/migrate_do_state.py "
            f"or register the DO before measuring its {section}.")
    with open(path, "rb") as f:
        data = tomllib.load(f)
    data[section] = {"source": "measured", **fields}
    errors = validate_characterization(data)
    if errors:
        raise SchemaError(f"{path}: {'; '.join(errors)}")
    text = _format_characterization_toml(data)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path),
                               prefix=f".{safe_uid(do_uid)}.",
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
    return path

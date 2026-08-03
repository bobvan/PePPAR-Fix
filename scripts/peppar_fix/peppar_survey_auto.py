"""peppar-survey --auto backend selector (S3 of I-071401).

Picks the fastest survey backend that will *work* for a given receiver +
site, and falls back to the PRIDE PPP-AR floor when no faster path applies.

The design invariant (from the I-071401 spec): **heuristics only buy
speed** — the PRIDE floor guarantees correctness, so a wrong-but-viable
heuristic can only ever cost time, never accuracy.  That makes the whole
selector safely testable as pure predicates + a ranking function over a
region→source data table.

Speed order of the backends (fastest first):

  1. ``baseline`` — RTKLIB relative baseline vs a nearby fixed base.
     Viable iff the rover is dual-frequency AND the site falls in a
     region with an open base archive (S2 region table) AND a fixed
     base is discoverable within range (S2 discover_base).  cm-class
     and fast because a short baseline cancels orbit/clock/atmosphere.
  2. ``pride``   — PRIDE PPP-AR.  The accuracy floor; needs precise
     products (internet) + pdp3 installed, but no base.  This is what
     we fall back to.
  3. ``rtklib``  — RTKLIB PPP-static (absolute, no base).  A deeper
     fallback for when even PRIDE isn't available on the host.

Nothing here fetches or solves; it produces a :class:`BackendPlan` that
``peppar_survey._run_auto`` either prints (``--plan-only``) or executes
by dispatching to the existing ``--baseline`` / ``--pride`` / ``--rtklib``
backends.
"""
from __future__ import annotations

import logging
import os
import shutil
from dataclasses import dataclass

from peppar_fix.peppar_survey_discovery import (
    BaseDescriptor, RegionSource, discover_base, source_for_position,
)

log = logging.getLogger("peppar_survey.auto")


# ---------------------------------------------------------------------------
# Receiver capability predicates
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class RxCapabilities:
    """What a receiver can do, as far as backend selection cares.

    dual_freq gates every carrier-phase backend (baseline/rtklib/pride-AR);
    rtk_fw (positioning firmware, F9P/X20P) is informational for the plan
    reason — offline relative-baseline post-processing needs only
    dual-frequency carrier phase, not live RTK firmware.
    """
    module: str
    dual_freq: bool
    rtk_fw: bool
    bands: tuple[str, ...]


# Keyed by a substring of the receiver's `module` identity string
# (receiver_state.py: "ZED-F9T", "ZED-F9P", "ZED-X20P", "NEO-F10T",
# "LEA-F9T-11B", ...).  All current lab receivers are dual-frequency; an
# unknown module is treated conservatively (single-freq, no RTK) so --auto
# falls back to the PRIDE floor rather than assuming a capability.
_CAP_TABLE: tuple[RxCapabilities, ...] = (
    RxCapabilities("ZED-F9P", True, True, ("L1", "L2")),
    RxCapabilities("ZED-X20P", True, True, ("L1", "L2", "L5")),
    RxCapabilities("ZED-F9T", True, False, ("L1", "L2")),   # or L1/L5, switchable
    RxCapabilities("LEA-F9T", True, False, ("L1", "L2")),
    RxCapabilities("NEO-F10T", True, False, ("L1", "L5")),
)


def capabilities_for_module(module: str | None) -> RxCapabilities:
    """Map a receiver module identity to its capabilities (conservative
    default for unknown modules)."""
    m = (module or "").upper()
    for caps in _CAP_TABLE:
        if caps.module in m:
            return caps
    return RxCapabilities(module or "unknown", dual_freq=False,
                          rtk_fw=False, bands=())


def ecef_to_latlon(ecef) -> tuple[float, float]:
    """WGS84 lat/lon (deg) from ECEF metres.  Only used to pick a region and
    rank bases, so WGS84 (vs a specific ITRF realization) is plenty."""
    from pyproj import Transformer
    tf = Transformer.from_crs(4978, 4979, always_xy=True)
    lon, lat, _h = tf.transform(float(ecef[0]), float(ecef[1]), float(ecef[2]))
    return lat, lon


def capabilities_for_uid(
    uid: str,
    receivers_dir: str | None,
) -> tuple[RxCapabilities, tuple[float, float] | None]:
    """Read a receiver's state → (capabilities, approx lat/lon | None).

    The approx position is the receiver's last-known NAV2/PPP fix — good to
    a few metres, which is all region selection + base ranking need.
    """
    from peppar_fix.receiver_state import (
        load_position_from_receiver, load_receiver_state,
    )
    state = load_receiver_state(uid, receivers_dir)
    module = (state or {}).get("module") if state else None
    caps = capabilities_for_module(module)
    latlon = None
    ecef = load_position_from_receiver(uid, receivers_dir)
    if ecef is not None:
        try:
            latlon = ecef_to_latlon(ecef)
        except Exception as e:  # noqa: BLE001 - bad coord shouldn't crash planning
            log.warning("ecef_to_latlon failed for %s: %s", uid, e)
    return caps, latlon


# ---------------------------------------------------------------------------
# Tool availability (the deeper-fallback predicates)
# ---------------------------------------------------------------------------
def pride_available() -> bool:
    """True if pdp3 is runnable (PEPPAR_PDP3_BIN or the default install)."""
    cand = os.environ.get("PEPPAR_PDP3_BIN") or os.path.expanduser(
        "~/.PRIDE_PPPAR_BIN/pdp3")
    return bool(shutil.which(cand) or (os.path.isfile(cand)
                                       and os.access(cand, os.X_OK)))


def rtklib_available() -> bool:
    """True if rnx2rtkp is runnable (PEPPAR_RNX2RTKP_BIN or on PATH)."""
    cand = os.environ.get("PEPPAR_RNX2RTKP_BIN") or "rnx2rtkp"
    return bool(shutil.which(cand) or (os.path.isfile(cand)
                                       and os.access(cand, os.X_OK)))


# ---------------------------------------------------------------------------
# The plan + the ranking function
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class BackendPlan:
    backend: str                 # "baseline" | "pride" | "rtklib"
    reason: str
    base: BaseDescriptor | None = None
    base_realization: str | None = None


def select_backend(
    *,
    caps: RxCapabilities,
    region_source: RegionSource | None,
    base_desc: BaseDescriptor | None,
    have_pride: bool,
    have_rtklib: bool,
) -> BackendPlan:
    """Pure ranking function — pick the fastest viable backend, else the
    PRIDE floor.  All inputs are already resolved (network/tool checks done
    by the caller) so this is deterministic and trivially testable."""
    # 1. Fastest: relative baseline, if the rover can carry phase AND we
    #    found a fixed base in an archive-covered region.
    if caps.dual_freq and region_source is not None and base_desc is not None:
        rtk_note = "" if caps.rtk_fw else " (offline post-proc; RTK fw not required)"
        return BackendPlan(
            backend="baseline",
            reason=(f"dual-freq {caps.module}: relative baseline vs "
                    f"{base_desc.station} @ {base_desc.distance_km:.0f} km "
                    f"({region_source.name}, {base_desc.base_realization})"
                    f"{rtk_note}"),
            base=base_desc,
            base_realization=base_desc.base_realization,
        )
    # 2. Floor: PRIDE PPP-AR.  Correct + cm-class; the safe fallback.
    if have_pride:
        why = _no_baseline_reason(caps, region_source, base_desc)
        return BackendPlan("pride", f"PRIDE PPP-AR floor ({why})")
    # 3. Deeper fallback: PPP-static when even PRIDE isn't installed.
    if caps.dual_freq and have_rtklib:
        return BackendPlan("rtklib", "RTKLIB PPP-static fallback "
                           "(pdp3 not installed; no nearby base)")
    # 4. Nothing runnable found — still name the floor so --plan-only tells
    #    the operator what to install.  A real run will error honestly.
    return BackendPlan("pride", "PRIDE PPP-AR floor — but pdp3 not found; "
                       "install PRIDE-PPP-AR (scripts/install_peppar_survey.sh)")


def _no_baseline_reason(
    caps: RxCapabilities,
    region_source: RegionSource | None,
    base_desc: BaseDescriptor | None,
) -> str:
    if not caps.dual_freq:
        return f"{caps.module} is not dual-frequency"
    if region_source is None:
        return "no open base archive covers this region"
    if base_desc is None:
        if region_source.catalog_url:
            return (f"no {region_source.name} station within --max-km "
                    "(and no nearer caster mount)")
        return ("no fixed base found within range "
                "(needs --caster-host and a base <= --max-km)")
    return "no faster backend viable"


def plan_auto(
    *,
    uid: str,
    receivers_dir: str | None,
    near: tuple[float, float] | None = None,
    caster_host: str | None = None,
    caster_port: int = 2101,
    max_km: float = 80.0,
    have_pride: bool | None = None,
    have_rtklib: bool | None = None,
    source_for_position_fn=source_for_position,
    discover_base_fn=discover_base,
) -> BackendPlan:
    """Resolve everything the selector needs (caps, region, base, tools) and
    return the chosen :class:`BackendPlan`.  I/O boundaries are injected so
    the glue is deterministic in tests."""
    caps, latlon = capabilities_for_uid(uid, receivers_dir)
    if near is not None:
        latlon = near
    if have_pride is None:
        have_pride = pride_available()
    if have_rtklib is None:
        have_rtklib = rtklib_available()

    region_source = None
    base_desc = None
    if latlon is not None:
        lat, lon = latlon
        region_source = source_for_position_fn(lat, lon)
        # Only bother discovering a base if the region has an archive to
        # pre-convert from AND some way to enumerate its stations — its own
        # catalogue (NGS CORS) or a caster's sourcetable (EUREF).  Without
        # either, baseline can't be pinned in ITRF2020 anyway.
        enumerable = bool(caster_host) or bool(
            region_source is not None and region_source.catalog_url)
        if region_source is not None and enumerable and caps.dual_freq:
            base_desc = discover_base_fn(
                lat, lon, caster_host=caster_host, caster_port=caster_port,
                max_km=max_km)
    else:
        log.warning("no approx position for %s (no --near, no last-known "
                    "fix) — cannot rank a baseline; using the PRIDE floor",
                    uid)

    return select_backend(
        caps=caps, region_source=region_source, base_desc=base_desc,
        have_pride=have_pride, have_rtklib=have_rtklib)

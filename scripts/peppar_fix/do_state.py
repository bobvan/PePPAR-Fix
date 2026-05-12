"""DO and PHC state persistence — load/save per-device state files.

DO state lives in state/dos/<unique_id>.json.
PHC state lives in state/phcs/<unique_id>.json.

For bundled PHC+DO (i226, E810), the PHC MAC serves as both IDs.
For external DOs (VCOCXO, ClockMatrix), the DO needs its own label.

See docs/state-persistence-design.md for the full entity model.
"""

import glob as _glob
import json
import logging
import os
from datetime import datetime, timezone

log = logging.getLogger(__name__)

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DO_STATE_DIR = os.path.join(_REPO_ROOT, "state", "dos")
PHC_STATE_DIR = os.path.join(_REPO_ROOT, "state", "phcs")


# ── PHC unique ID discovery ─────────────────────────────────────────────── #

def discover_phc_mac(ptp_path):
    """Discover the MAC address of the NIC backing a PTP device.

    Walks sysfs: /sys/class/ptp/ptpN/device/net/*/address

    Returns:
        MAC address string (e.g. "54:49:4d:45:00:6b") or None
    """
    basename = os.path.basename(ptp_path)
    if not basename.startswith("ptp"):
        return None
    net_dir = f"/sys/class/ptp/{basename}/device/net"
    if not os.path.isdir(net_dir):
        return None
    for iface in os.listdir(net_dir):
        addr_path = os.path.join(net_dir, iface, "address")
        try:
            with open(addr_path) as f:
                mac = f.read().strip()
            if mac and mac != "00:00:00:00:00:00":
                return mac
        except OSError:
            continue
    return None


def discover_phc_driver(ptp_path):
    """Discover the driver name for a PTP device.

    Returns driver name (e.g. "igc", "ice") or None.
    """
    basename = os.path.basename(ptp_path)
    if not basename.startswith("ptp"):
        return None
    try:
        sys_path = f"/sys/class/ptp/{basename}/device/driver"
        return os.path.basename(os.readlink(sys_path))
    except (OSError, ValueError):
        return None


def phc_unique_id(ptp_path):
    """Get a stable unique ID for a PHC device.

    Uses MAC address (preferred) or falls back to the device path.
    """
    mac = discover_phc_mac(ptp_path)
    if mac is not None:
        return mac
    return ptp_path


# ── DO state ─────────────────────────────────────────────────────────────── #

def _do_path(unique_id, state_dir=None):
    d = state_dir or DO_STATE_DIR
    # MAC addresses contain colons — replace with dashes for filenames
    safe_id = str(unique_id).replace(":", "-").replace("/", "_")
    return os.path.join(d, f"{safe_id}.json")


def load_do_state(unique_id, state_dir=None):
    """Load DO state file.

    Returns dict or None if not found.
    """
    path = _do_path(unique_id, state_dir)
    if not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            data = json.load(f)
        log.info("Loaded DO state from %s", path)
        return data
    except (json.JSONDecodeError, OSError) as e:
        log.warning("Failed to load DO state from %s: %s", path, e)
        return None


def save_do_state(state, state_dir=None):
    """Save DO state file. Atomic write via tmp+replace."""
    uid = state.get("unique_id")
    if uid is None:
        log.warning("Cannot save DO state without unique_id")
        return
    d = state_dir or DO_STATE_DIR
    os.makedirs(d, exist_ok=True)
    path = _do_path(uid, state_dir)
    tmp = path + ".tmp"
    with open(tmp, 'w') as f:
        json.dump(state, f, indent=2)
        f.write('\n')
    os.replace(tmp, path)
    log.info("Saved DO state to %s", path)


def new_do_state(unique_id, label=None):
    """Create a fresh DO state dict."""
    now = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    return {
        "unique_id": unique_id,
        "label": label or unique_id,
        "characterization": None,
        "adjustment": None,
        "last_known_freq_offset_ppb": None,
        "last_known_temp_c": None,
        "updated": now,
    }


def save_do_freq_offset(unique_id, adjfine_ppb, state_dir=None):
    """Update the DO's last-known frequency offset.

    This replaces the adjfine_ppb field in the old data/drift.json.
    """
    state = load_do_state(unique_id, state_dir)
    if state is None:
        state = new_do_state(unique_id)
    now = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    state["last_known_freq_offset_ppb"] = adjfine_ppb
    state["updated"] = now
    save_do_state(state, state_dir)


def save_do_characterization(unique_id, characterization, state_dir=None):
    """Store DO characterization (from build_do_characterization.py).

    Args:
        unique_id: DO unique ID
        characterization: dict with asd, psd, tdev_1s, noise_floor_ns, etc.
    """
    state = load_do_state(unique_id, state_dir)
    if state is None:
        state = new_do_state(unique_id)
    state["characterization"] = characterization
    state["updated"] = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    save_do_state(state, state_dir)


def is_do_warm_startable(unique_id, max_age_s=86400, max_ppb=500.0,
                         state_dir=None):
    """Decide whether the DO can skip cold-start bootstrap.

    Cold-start bootstrap costs ~15-30 s: DAC center reset + post-ARM
    settle + freerun PPS measurement.  When the prior run's
    last_known_freq_offset_ppb is recent and physically plausible,
    the DAC can be seeded from it directly and the freerun
    measurement skipped.  This matters across watchdog re-bootstraps
    where the saved freq is ~seconds old.  See dacWarmStart-main.

    Pass criteria (all must hold):
      - State file exists and parses
      - 'last_known_freq_offset_ppb' is present and finite
      - |freq| <= max_ppb (sanity envelope; a 5000 ppb value is
        almost certainly a parsing error or stale corruption)
      - File mtime within max_age_s of now

    Returns:
        (True, dict) where dict has keys 'freq_ppb', 'age_s',
            'reason' when warm-startable.
        (False, dict) where dict has 'reason' describing why not.
    """
    import math
    import time as _time
    path = _do_path(unique_id, state_dir)
    if not os.path.exists(path):
        return (False, {"reason": "no_state_file"})
    try:
        mtime = os.path.getmtime(path)
    except OSError as e:
        return (False, {"reason": f"stat_failed: {e}"})
    age_s = _time.time() - mtime
    if age_s > max_age_s:
        return (False, {"reason": f"too_old: age={age_s:.0f}s "
                                 f"max={max_age_s:.0f}s"})
    state = load_do_state(unique_id, state_dir)
    if state is None:
        return (False, {"reason": "load_failed"})
    freq = state.get("last_known_freq_offset_ppb")
    if freq is None:
        return (False, {"reason": "no_last_known_freq"})
    try:
        freq_f = float(freq)
    except (TypeError, ValueError):
        return (False, {"reason": f"freq_not_numeric: {freq!r}"})
    if not math.isfinite(freq_f):
        return (False, {"reason": f"freq_not_finite: {freq_f}"})
    if abs(freq_f) > max_ppb:
        return (False, {"reason": f"freq_out_of_envelope: "
                                 f"|{freq_f:.1f}| > {max_ppb:.1f} ppb"})
    return (True, {"freq_ppb": freq_f, "age_s": age_s,
                   "reason": "fresh"})


def derive_do_process_noise(characterization):
    """Extract DOFreqEst process-noise parameters from a DO characterization.

    The DOFreqEst EKF uses (sigma_do_phase_ns, sigma_do_freq_ppb) as
    random-walk amplitudes per √s for the DO state.  Defaults
    (0.92 ns, ≈0.05 ppb) are 10–100× looser than well-disciplined
    OCXO + DAC chains actually wander, which drives the EKF to
    over-react to GNSS-side measurement noise and inject it into the
    actuator.  Driving these from measured characterization closes
    that gap.  See doProcessNoiseFromChar-main.

    Heuristic (v1):
      sigma_do_phase_ns  = ASD@0.1Hz of the DO's PPS or Carrier
                           output (units ns/√Hz, used directly as
                           ns/√s for the random-walk).  Prefer
                           Carrier > PPS > PPS+qErr.
      sigma_do_freq_ppb  = ASD@0.1Hz of the `adjfine` source
                           (units ppb/√Hz).  This is the freq-noise
                           floor the DO sees on its actuator input.

    Both numbers are conservative — they're the noise floor at the
    PSD measurement band-center, not the absolute white-FM floor.
    Refinement: factor in slope and pick a better point on the PSD
    curve.  Doing the simple thing first.

    Returns:
        dict with keys 'sigma_do_phase_ns' and/or 'sigma_do_freq_ppb'
        for whichever were extractable; an empty dict if nothing
        usable was found; None if characterization is malformed.
    """
    if not isinstance(characterization, dict):
        return None
    sources = characterization.get("sources")
    if not isinstance(sources, dict):
        return {}

    out = {}

    # Phase noise — prefer Carrier (cleanest DO output) > PPS > PPS+qErr.
    for key in ("Carrier", "PPS", "PPS+qErr"):
        src = sources.get(key)
        if not isinstance(src, dict):
            continue
        if src.get("units") != "ns":
            continue
        v = src.get("asd_at_0.1Hz")
        if isinstance(v, (int, float)) and v > 0:
            out["sigma_do_phase_ns"] = float(v)
            out["sigma_do_phase_source"] = key
            break

    # Frequency noise — adjfine ASD measures the DO's freq-domain
    # noise floor at the actuator input.
    src = sources.get("adjfine")
    if isinstance(src, dict) and src.get("units") == "ppb":
        v = src.get("asd_at_0.1Hz")
        if isinstance(v, (int, float)) and v > 0:
            out["sigma_do_freq_ppb"] = float(v)
            out["sigma_do_freq_source"] = "adjfine"

    return out


# ── PHC state ────────────────────────────────────────────────────────────── #

def _phc_path(unique_id, state_dir=None):
    d = state_dir or PHC_STATE_DIR
    safe_id = str(unique_id).replace(":", "-").replace("/", "_")
    return os.path.join(d, f"{safe_id}.json")


def load_phc_state(unique_id, state_dir=None):
    """Load PHC state file.

    Returns dict or None if not found.
    """
    path = _phc_path(unique_id, state_dir)
    if not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            data = json.load(f)
        log.info("Loaded PHC state from %s", path)
        return data
    except (json.JSONDecodeError, OSError) as e:
        log.warning("Failed to load PHC state from %s: %s", path, e)
        return None


def save_phc_state(state, state_dir=None):
    """Save PHC state file. Atomic write via tmp+replace."""
    uid = state.get("unique_id")
    if uid is None:
        log.warning("Cannot save PHC state without unique_id")
        return
    d = state_dir or PHC_STATE_DIR
    os.makedirs(d, exist_ok=True)
    path = _phc_path(uid, state_dir)
    tmp = path + ".tmp"
    with open(tmp, 'w') as f:
        json.dump(state, f, indent=2)
        f.write('\n')
    os.replace(tmp, path)
    log.info("Saved PHC state to %s", path)


def new_phc_state(ptp_path):
    """Create a fresh PHC state dict from a device path.

    Discovers MAC, driver, and links to a bundled DO.
    """
    mac = discover_phc_mac(ptp_path)
    driver = discover_phc_driver(ptp_path)
    uid = mac or ptp_path
    now = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')

    return {
        "unique_id": uid,
        "device": ptp_path,
        "driver": driver,
        "mac": mac,
        "extts": None,
        "perout": None,
        "do_unique_id": uid,  # Bundled: PHC and DO share ID
        "last_known_device": ptp_path,
        "last_seen": now,
    }


# ── Drift file migration ────────────────────────────────────────────────── #

def migrate_drift_file(drift_path, ptp_path, state_dir=None):
    """Migrate legacy data/drift.json into DO and receiver state.

    Reads drift.json, splits fields:
    - adjfine_ppb → DO state (last_known_freq_offset_ppb)
    - tcxo_freq_corr_ppb → returned for receiver state update
    - dt_rx_ns → returned for receiver state update

    Returns:
        (tcxo_freq_corr_ppb, dt_rx_ns) or (None, None) if no drift file.
    """
    if not os.path.exists(drift_path):
        return None, None
    try:
        with open(drift_path) as f:
            drift = json.load(f)
    except (json.JSONDecodeError, OSError):
        return None, None

    do_uid = phc_unique_id(ptp_path)
    adjfine = drift.get("adjfine_ppb")
    if adjfine is not None:
        save_do_freq_offset(do_uid, adjfine, state_dir)
        log.info("Migrated adjfine_ppb=%.1f from %s to DO state %s",
                 adjfine, drift_path, do_uid)

    return drift.get("tcxo_freq_corr_ppb"), drift.get("dt_rx_ns")


def load_drift_from_state(ptp_path, do_state_dir=None):
    """Load drift info from DO state, returning a dict compatible with
    the legacy drift.json format for backward compatibility.

    Returns dict with adjfine_ppb, phc, timestamp or None.
    """
    do_uid = phc_unique_id(ptp_path)
    state = load_do_state(do_uid, do_state_dir)
    if state is None:
        return None
    adjfine = state.get("last_known_freq_offset_ppb")
    if adjfine is None:
        return None
    return {
        "adjfine_ppb": adjfine,
        "phc": ptp_path,
        "timestamp": state.get("updated", ""),
    }

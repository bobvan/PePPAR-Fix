"""Receiver state persistence — load/save per-receiver state files.

State files live in state/receivers/<unique_id>.json, keyed by the
receiver's SEC-UNIQID (as a decimal integer string).  Each file holds
identity, capabilities, last-known position, and TCXO offset.

See docs/state-persistence-design.md for the full entity model.
"""

import json
import logging
import os
from datetime import datetime, timezone

log = logging.getLogger(__name__)

# Repo root: scripts/peppar_fix/../../ = repo root.
# State directories are always relative to repo root, not CWD.
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_STATE_DIR = os.path.join(_REPO_ROOT, "state", "receivers")


def _state_path(unique_id, state_dir=None):
    """Return the file path for a receiver's state file."""
    d = state_dir or DEFAULT_STATE_DIR
    return os.path.join(d, f"{unique_id}.json")


def load_receiver_state(unique_id, state_dir=None):
    """Load stored state for a receiver.

    Args:
        unique_id: receiver unique ID (int or str — converted to str for filename)
        state_dir: directory containing receiver state files

    Returns:
        dict of stored state, or None if no state file exists.
    """
    path = _state_path(unique_id, state_dir)
    if not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            data = json.load(f)
        log.info("Loaded receiver state from %s", path)
        return data
    except (json.JSONDecodeError, OSError) as e:
        log.warning("Failed to load receiver state from %s: %s", path, e)
        return None


def save_receiver_state(state, state_dir=None):
    """Save receiver state to its state file.

    The state dict must contain 'unique_id'.  The file is written
    atomically via tmp+replace.

    Args:
        state: dict with at least 'unique_id'
        state_dir: directory for state files (created if needed)
    """
    uid = state.get("unique_id")
    if uid is None:
        log.warning("Cannot save receiver state without unique_id")
        return
    d = state_dir or DEFAULT_STATE_DIR
    os.makedirs(d, exist_ok=True)
    path = _state_path(uid, state_dir)
    tmp = path + ".tmp"
    with open(tmp, 'w') as f:
        json.dump(state, f, indent=2)
        f.write('\n')
    os.replace(tmp, path)
    log.info("Saved receiver state to %s", path)


def find_receiver_by_port(port, state_dir=None):
    """Find the most recently seen receiver on a given port.

    Scans all state files for last_known_port matching the given port.
    Returns the state dict of the most recently seen match, or None.
    """
    d = state_dir or DEFAULT_STATE_DIR
    if not os.path.isdir(d):
        return None
    best = None
    for name in os.listdir(d):
        if not name.endswith(".json"):
            continue
        path = os.path.join(d, name)
        try:
            with open(path) as f:
                data = json.load(f)
        except (json.JSONDecodeError, OSError):
            continue
        if data.get("last_known_port") != port:
            continue
        if best is None or data.get("last_seen", "") > best.get("last_seen", ""):
            best = data
    return best


def new_receiver_state(identity, port):
    """Create a fresh state dict from a query_receiver_identity() result.

    Args:
        identity: dict from query_receiver_identity()
        port: serial port path

    Returns:
        state dict ready for save_receiver_state()
    """
    now = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')  # wallclock-ok: state-file record timestamp
    return {
        "unique_id": identity["unique_id"],
        "unique_id_hex": identity.get("unique_id_hex"),
        "module": identity.get("module", "unknown"),
        "firmware": identity.get("firmware", "unknown"),
        "protver": identity.get("protver"),
        "capabilities": {},
        "tcxo": {},
        "last_known_position": None,
        "last_known_port": port,
        "last_seen": now,
    }


def update_receiver_state(state, identity, port):
    """Update an existing state dict with fresh identity info.

    Preserves position, capabilities, and TCXO data.  Updates firmware,
    port, and last_seen.  If firmware changed, clears cached capabilities
    so they get re-probed.

    Returns:
        (state, firmware_changed) tuple
    """
    now = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')  # wallclock-ok: state-file record timestamp
    old_firmware = state.get("firmware")
    new_firmware = identity.get("firmware", "unknown")
    firmware_changed = (old_firmware != new_firmware
                        and old_firmware is not None
                        and new_firmware != "unknown")

    state["module"] = identity.get("module", state.get("module", "unknown"))
    state["firmware"] = new_firmware
    state["protver"] = identity.get("protver", state.get("protver"))
    state["last_known_port"] = port
    state["last_seen"] = now

    if firmware_changed:
        log.warning("Firmware changed: %s -> %s — clearing cached capabilities",
                    old_firmware, new_firmware)
        state["capabilities"] = {}

    return state, firmware_changed


def check_receiver_change(current_id, port, state_dir=None):
    """Check whether the receiver on a port has changed.

    Compares current_id (from query_receiver_identity) against the
    last-known receiver on this port.

    Returns:
        (stored_state, change_type) where change_type is:
            "same" — same receiver, same firmware
            "firmware_changed" — same receiver, different firmware
            "receiver_changed" — different receiver on this port
            "new" — no prior state for this receiver
    """
    if current_id is None or current_id.get("unique_id") is None:
        return None, "new"

    uid = current_id["unique_id"]

    # Check if we have state for this specific receiver
    stored = load_receiver_state(uid, state_dir)
    if stored is not None:
        old_fw = stored.get("firmware")
        new_fw = current_id.get("firmware", "unknown")
        if old_fw != new_fw and old_fw is not None and new_fw != "unknown":
            return stored, "firmware_changed"
        return stored, "same"

    # No state for this receiver — check if a different receiver was
    # last seen on this port
    old_on_port = find_receiver_by_port(port, state_dir)
    if old_on_port is not None:
        return old_on_port, "receiver_changed"

    return None, "new"


def save_position_to_receiver(unique_id, ecef, sigma_m, source, state_dir=None):
    """Save a position fix into the receiver's state file.

    The receiver state carries its own position because different
    receivers may be connected to different antennas — position is a
    property of the receiver/antenna pair, not of the host.

    Args:
        unique_id: receiver unique ID
        ecef: [x, y, z] in meters (list or numpy array)
        sigma_m: position sigma in meters
        source: string describing origin (e.g. 'ppp_bootstrap')
    """
    state = load_receiver_state(unique_id, state_dir)
    if state is None:
        log.warning("Cannot save position — no state file for receiver %s", unique_id)
        return
    now = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')  # wallclock-ok: state-file record timestamp
    state["last_known_position"] = {
        "ecef_m": [round(float(ecef[0]), 3),
                    round(float(ecef[1]), 3),
                    round(float(ecef[2]), 3)],
        "sigma_m": round(float(sigma_m), 4),
        "source": source,
        "updated": now,
    }
    save_receiver_state(state, state_dir)


def load_position_from_receiver(unique_id, state_dir=None):
    """Load last-known position from a receiver's state file.

    Returns:
        numpy array [x, y, z] in ECEF meters, or None if no position stored.
    """
    state = load_receiver_state(unique_id, state_dir)
    if state is None:
        return None
    pos = state.get("last_known_position")
    if pos is None:
        return None
    ecef = pos.get("ecef_m")
    if ecef is None or len(ecef) != 3:
        return None
    try:
        import numpy as np
        return np.array(ecef, dtype=float)
    except (TypeError, ValueError):
        return None


def load_position_detail_from_receiver(unique_id, state_dir=None):
    """Load position with quality metadata from a receiver's state file.

    Returns:
        (ecef, sigma_m, source) tuple, or (None, None, None) if no position.
        ecef is a numpy array [x, y, z] in ECEF meters.
    """
    state = load_receiver_state(unique_id, state_dir)
    if state is None:
        return None, None, None
    pos = state.get("last_known_position")
    if pos is None:
        return None, None, None
    ecef = pos.get("ecef_m")
    if ecef is None or len(ecef) != 3:
        return None, None, None
    try:
        import numpy as np
        return (np.array(ecef, dtype=float),
                pos.get("sigma_m"),
                pos.get("source"))
    except (TypeError, ValueError):
        return None, None, None


# A receiver clock bias (dt_rx) is NOT persistent across a receiver reset or
# a long engine gap — it only stays meaningful across a brief relaunch.  The
# saved value is timestamped so consumers can reject a stale one.  Seeding the
# DOFreqEst φ_rx state (which has a tight seed covariance) from a stale dt_rx
# makes the first Arm-1 innovation a massive outlier that couples into the
# rx-TCXO frequency state and drives a startup EKF blow-up (ptBoat 2026-07-06:
# -15.9 ms seed 8 days stale vs live ~+2.6 ms → x[1] diverged to 2e5 ppb,
# 6 state_sanity resets, near exit-5).
DT_RX_SEED_MAX_AGE_S = 300.0


def save_dt_rx_to_receiver(unique_id, dt_rx_ns, freq_offset_ppb=None,
                           state_dir=None):
    """Persist the rx-TCXO last-known dt_rx (and optional freq offset) into
    the receiver's ``tcxo`` state block, stamped with the current UTC time.

    The timestamp is what lets :func:`dt_rx_seed_if_fresh` reject a value that
    is too old to seed the DOFreqEst rx-TCXO phase.
    """
    state = load_receiver_state(unique_id, state_dir)
    if state is None:
        log.warning("Cannot save dt_rx — no state file for receiver %s",
                    unique_id)
        return
    tcxo = state.setdefault("tcxo", {})
    if freq_offset_ppb is not None:
        tcxo["last_known_freq_offset_ppb"] = float(freq_offset_ppb)
    if dt_rx_ns is not None:
        # Only bump the freshness timestamp when a real dt_rx is written —
        # otherwise a freq-only save would mark a stale dt_rx as fresh and
        # defeat dt_rx_seed_if_fresh().
        tcxo["last_known_dt_rx_ns"] = float(dt_rx_ns)
        tcxo["updated"] = datetime.now(  # wallclock-ok: dt_rx freshness stamp (state record)
            timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    save_receiver_state(state, state_dir)


def dt_rx_seed_if_fresh(state, max_age_s=DT_RX_SEED_MAX_AGE_S, now=None):
    """Return ``(dt_rx_ns, age_s)`` from ``tcxo.last_known_dt_rx_ns`` iff it
    was saved within ``max_age_s``; otherwise ``(None, age_s_or_None)``.

    ``age_s`` is returned even on rejection so the caller can log why.  A
    missing value, a missing/unparseable timestamp, a negative age (clock
    stepped backward), or an age beyond ``max_age_s`` all reject the seed.
    See :data:`DT_RX_SEED_MAX_AGE_S` for the rationale.
    """
    tcxo = (state or {}).get("tcxo", {}) or {}
    dt_rx = tcxo.get("last_known_dt_rx_ns")
    if dt_rx is None:
        return None, None
    updated = tcxo.get("updated")
    if not updated:
        return None, None
    try:
        saved = datetime.strptime(
            updated, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc)
    except (ValueError, TypeError):
        return None, None
    if now is None:
        now = datetime.now(timezone.utc)  # wallclock-ok: coarse seed-staleness age vs state record (injectable via now=)
    age_s = (now - saved).total_seconds()
    if age_s < 0 or age_s > max_age_s:
        return None, age_s
    return float(dt_rx), age_s


def receiver_has_position(unique_id, state_dir=None):
    """Check whether a receiver has a stored position.

    Returns True if the receiver has a last_known_position, False otherwise.
    Useful for deciding whether to run position bootstrap.
    """
    state = load_receiver_state(unique_id, state_dir)
    if state is None:
        return False
    pos = state.get("last_known_position")
    return pos is not None and pos.get("ecef_m") is not None

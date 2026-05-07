"""Timestamper state persistence and noise parameter lookup.

A timestamper is anything that measures PPS edges: TICC channels,
EXTTS channels on PHCs.  Each has characterized noise properties
that the servo needs — currently hardcoded magic numbers.

State files live in state/timestampers/<unique_id>.json.
TICC: keyed by Arduino serial number.
EXTTS: keyed by PHC MAC + channel index.

See docs/state-persistence-design.md for the full entity model.
"""

import json
import logging
import os
from datetime import datetime, timezone

log = logging.getLogger(__name__)

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
TIMESTAMPER_STATE_DIR = os.path.join(_REPO_ROOT, "state", "timestampers")

# Pessimistic defaults when no characterization is available.
# These are safe starting points — real characterization will be better.
DEFAULTS = {
    "ticc": {
        # TAPR spec: <60 ps single-shot resolution.  Confirmed by the
        # 2026-05-07 TEE2 calibration of TICC #4/#5 (0.14-0.24 ns
        # per-TICC floor when paired through a tee, on the order of the
        # TAPR spec; see timelab/calibration.md "Run TEE2").  Earlier
        # 0.178 ns default was conservative and pre-Rb-reference.
        "measurement_noise_ns": 0.060,   # TAPR spec + TEE2 cross-validation
        "resolution_ps": 60,
    },
    "extts_igc": {
        "measurement_noise_ns": 2.9,     # i226 EXTTS baseline analysis
        "resolution_ns": 8.0,
        "identical_adjacent_pct": 0.0,
    },
    "extts_ice": {
        "measurement_noise_ns": 0.34,    # E810 EXTTS (misleadingly low — quantization)
        "resolution_ns": 8.0,
        "identical_adjacent_pct": 77.0,
    },
}

# Derived servo parameters from timestamper noise.
# PPS confidence = timestamper noise RSS with F9T sawtooth (~2.3 ns TDEV(1s)).
# PPS+qErr confidence = timestamper noise alone (qErr removes sawtooth).
F9T_SAWTOOTH_NS = 2.3


def _ts_path(unique_id, state_dir=None):
    d = state_dir or TIMESTAMPER_STATE_DIR
    safe_id = str(unique_id).replace(":", "-").replace("/", "_")
    return os.path.join(d, f"{safe_id}.json")


def load_timestamper_state(unique_id, state_dir=None):
    """Load timestamper state file."""
    path = _ts_path(unique_id, state_dir)
    if not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            return json.load(f)
    except (json.JSONDecodeError, OSError) as e:
        log.warning("Failed to load timestamper state from %s: %s", path, e)
        return None


def save_timestamper_state(state, state_dir=None):
    """Save timestamper state file. Atomic write via tmp+replace."""
    uid = state.get("unique_id")
    if uid is None:
        return
    d = state_dir or TIMESTAMPER_STATE_DIR
    os.makedirs(d, exist_ok=True)
    path = _ts_path(uid, state_dir)
    tmp = path + ".tmp"
    with open(tmp, 'w') as f:
        json.dump(state, f, indent=2)
        f.write('\n')
    os.replace(tmp, path)
    log.info("Saved timestamper state to %s", path)


def ticc_unique_id(arduino_serial):
    """Build unique ID for a TICC from its Arduino serial number."""
    return f"ticc-{arduino_serial}"


def extts_unique_id(phc_mac, channel):
    """Build unique ID for an EXTTS channel."""
    return f"extts-{phc_mac}-ch{channel}"


class TimestamperParams:
    """Resolved noise parameters for the active timestamper.

    Replaces hardcoded constants throughout the engine.  Constructed
    from state files with pessimistic defaults when uncharacterized.
    """

    def __init__(self, measurement_noise_ns, pps_confidence_ns,
                 qerr_confidence_ns, source="default"):
        self.measurement_noise_ns = measurement_noise_ns
        self.pps_confidence_ns = pps_confidence_ns
        self.qerr_confidence_ns = qerr_confidence_ns
        self.source = source

    @classmethod
    def for_ticc(cls, ticc_serial=None, state_dir=None):
        """Build params for TICC-driven servo.

        Args:
            ticc_serial: Arduino serial number (for state lookup).
                If None, uses pessimistic defaults.
        """
        noise_ns = DEFAULTS["ticc"]["measurement_noise_ns"]
        source = "default"

        if ticc_serial:
            uid = ticc_unique_id(ticc_serial)
            state = load_timestamper_state(uid, state_dir)
            if state is not None:
                noise_ns = state.get("measurement_noise_ns", noise_ns)
                source = f"state ({uid})"

        # TICC noise is small relative to F9T sawtooth
        import math
        pps_conf = math.sqrt(noise_ns**2 + F9T_SAWTOOTH_NS**2)
        qerr_conf = noise_ns  # qErr removes sawtooth

        return cls(noise_ns, pps_conf, qerr_conf, source)

    @classmethod
    def for_extts(cls, phc_mac=None, channel=0, driver=None, state_dir=None):
        """Build params for EXTTS-driven servo.

        Args:
            phc_mac: PHC MAC address (for state lookup).
            channel: EXTTS channel index.
            driver: NIC driver name ("igc", "ice") for default selection.
        """
        # Pick defaults by driver
        if driver == "ice":
            defaults = DEFAULTS["extts_ice"]
        else:
            defaults = DEFAULTS["extts_igc"]

        noise_ns = defaults["measurement_noise_ns"]
        source = f"default ({driver or 'unknown'})"

        if phc_mac is not None:
            uid = extts_unique_id(phc_mac, channel)
            state = load_timestamper_state(uid, state_dir)
            if state is not None:
                noise_ns = state.get("measurement_noise_ns", noise_ns)
                source = f"state ({uid})"

        import math
        pps_conf = math.sqrt(noise_ns**2 + F9T_SAWTOOTH_NS**2)
        qerr_conf = noise_ns

        return cls(noise_ns, pps_conf, qerr_conf, source)

    @classmethod
    def resolve(cls, args, state_dir=None):
        """Auto-resolve the right timestamper params from CLI args.

        This is the single entry point that replaces all the
        `0.178 if args.ticc_drive else 1.9` conditionals.
        """
        if getattr(args, 'ticc_port', None) is not None:
            # TICC serial could come from udev (not yet wired up)
            ticc_serial = getattr(args, 'ticc_serial', None)
            return cls.for_ticc(ticc_serial, state_dir)
        else:
            # EXTTS — discover PHC info
            phc_dev = getattr(args, 'servo', None)
            phc_mac = None
            driver = None
            if phc_dev:
                try:
                    from peppar_fix.do_state import discover_phc_mac, discover_phc_driver
                    phc_mac = discover_phc_mac(phc_dev)
                    driver = discover_phc_driver(phc_dev)
                except Exception:
                    pass
            channel = getattr(args, 'extts_channel', 0)
            return cls.for_extts(phc_mac, channel, driver, state_dir)

    def __repr__(self):
        return (f"TimestamperParams(noise={self.measurement_noise_ns:.3f}ns, "
                f"pps_conf={self.pps_confidence_ns:.1f}ns, "
                f"qerr_conf={self.qerr_confidence_ns:.3f}ns, "
                f"source={self.source!r})")

"""Optional ambient/oven temperature sensor support.

Probes for an ADT7410 (Analog Devices ±0.5 °C precision sensor) on
I2C bus 1 at address 0x48 at engine startup.  If found, the engine
logs temperature periodically alongside other status; if absent, the
engine continues silently with no error.

Why ADT7410 and address 0x48: that's the part Bob standardized on
2026-06-02 for OCXO oven feedforward.  TMP117/TMP119 (the more
accurate alternatives) live at the same address range and share the
same probe behavior — extend `_PROBE_TARGETS` to add support when
those land.
"""
from __future__ import annotations

import glob
import logging
import os
from typing import Optional

log = logging.getLogger(__name__)


# ── On-board thermal zones (RPi CPU etc.) ──────────────────────────── #
#
# Every Raspberry Pi exposes at least one Linux thermal zone (the SoC/CPU
# sensor) at /sys/class/thermal/thermal_zone*/temp (millidegrees C), with a
# human label in the sibling `type` file (e.g. "cpu-thermal", "rp1_adc").
# These don't measure lab air or the OCXO oven directly, but they track the
# DO's operating ENVIRONMENT closely enough to be a useful proxy until a
# dedicated I2C sensor (ADT7410 above) is fitted per host.  No deps, no
# probing — just sysfs reads; returns {} on a non-Linux/sensorless host.
_THERMAL_ROOT = "/sys/class/thermal"


def read_onboard_temps(thermal_root: str = _THERMAL_ROOT) -> dict[str, float]:
    """Return {label: celsius} for every readable on-board thermal zone.

    Labels come from each zone's `type` file (deduped with a numeric
    suffix on collision).  Implausible readings (outside −40..150 °C, e.g.
    a disabled zone reporting 0 or a huge sentinel) are dropped.  Never
    raises — a host with no thermal zones just yields an empty dict.
    """
    out: dict[str, float] = {}
    try:
        zones = sorted(glob.glob(os.path.join(thermal_root, "thermal_zone*")))
    except OSError:
        return out
    for i, zdir in enumerate(zones):
        try:
            with open(os.path.join(zdir, "temp")) as f:
                milli = int(f.read().strip())
            c = milli / 1000.0
            if not (-40.0 <= c <= 150.0):
                continue
            try:
                with open(os.path.join(zdir, "type")) as f:
                    label = f.read().strip() or f"zone{i}"
            except OSError:
                label = f"zone{i}"
            if label in out:                      # dedupe same-type zones
                label = f"{label}{i}"
            out[label] = round(c, 3)
        except (OSError, ValueError):
            continue
    return out


# (label, addr, id_reg, expected_id_byte)
# ADT7410: device-ID register at 0x0B returns 0xCB.
_PROBE_TARGETS = (
    ("adt7410", 0x48, 0x0B, 0xCB),
)


class TempSensor:
    """Minimal I2C temperature sensor reader.

    Construction probes the bus; if no supported chip responds, the
    instance is created with ``available=False`` and read methods
    return ``None``.  Callers don't need to special-case the missing
    case — just check ``ts.available`` or accept ``None`` from
    ``read_celsius()``.
    """

    def __init__(self, bus_num: int = 1):
        self.bus_num = bus_num
        self.available: bool = False
        self.label: Optional[str] = None
        self.addr: Optional[int] = None
        self._bus = None
        self._probe()

    def _probe(self) -> None:
        try:
            import smbus2
        except ImportError:
            log.debug("temp_sensor: smbus2 not installed — temperature logging disabled")
            return
        try:
            bus = smbus2.SMBus(self.bus_num)
        except (FileNotFoundError, PermissionError, OSError) as e:
            log.debug("temp_sensor: cannot open /dev/i2c-%d: %s — temperature logging disabled",
                      self.bus_num, e)
            return
        for label, addr, id_reg, expected in _PROBE_TARGETS:
            try:
                got = bus.read_byte_data(addr, id_reg)
            except OSError:
                continue
            if got == expected:
                self.available = True
                self.label = label
                self.addr = addr
                self._bus = bus
                log.info("temp sensor: %s @ 0x%02x on bus %d — periodic logging enabled",
                         label, addr, self.bus_num)
                return
        try:
            bus.close()
        except Exception:
            pass

    def read_celsius(self) -> Optional[float]:
        """Read temperature in degrees Celsius.  Returns ``None`` on
        any error or if no sensor is available."""
        if not self.available or self._bus is None:
            return None
        try:
            # ADT7410 default 13-bit mode: temp register 0x00 returns
            # 16-bit big-endian word; upper 13 bits are temp data, low
            # 3 are flags.  Decode works for both 13-bit and 16-bit
            # modes because the top bits are positioned identically.
            data = self._bus.read_i2c_block_data(self.addr, 0x00, 2)
        except OSError as e:
            log.debug("temp_sensor read failed: %s", e)
            return None
        raw = (data[0] << 8) | data[1]
        # 16-bit signed interpretation; for 13-bit-mode readings the
        # low 3 bits are zero, so the conversion is identical.
        if raw & 0x8000:
            raw -= 0x10000
        return raw * 0.0078125

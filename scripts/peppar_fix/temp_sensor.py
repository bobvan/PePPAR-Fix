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

import logging
from typing import Optional

log = logging.getLogger(__name__)


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

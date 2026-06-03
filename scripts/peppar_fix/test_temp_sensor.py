"""Tests for the optional temp sensor probe.

Construction must never raise — missing smbus2, missing /dev/i2c-N,
no device responding at the probed addresses all return an
``available=False`` instance with no log.warning/log.error noise.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest import mock

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))


class _NoBus:
    """smbus2.SMBus stand-in that raises on construction."""
    def __init__(self, *a, **k):
        raise FileNotFoundError(2, "no such device")


class _OpenBus:
    """smbus2.SMBus stand-in with controllable register reads."""
    def __init__(self, *a, **k):
        self._regs: dict[tuple[int, int], int] = {}
        self.closed = False

    def set_byte(self, addr: int, reg: int, val: int) -> None:
        self._regs[(addr, reg)] = val

    def set_block(self, addr: int, reg: int, vals: list[int]) -> None:
        for i, v in enumerate(vals):
            self._regs[(addr, reg + i)] = v

    def read_byte_data(self, addr: int, reg: int) -> int:
        if (addr, reg) not in self._regs:
            raise OSError(121, "Remote I/O error")
        return self._regs[(addr, reg)]

    def read_i2c_block_data(self, addr: int, reg: int, length: int):
        out = []
        for i in range(length):
            if (addr, reg + i) not in self._regs:
                raise OSError(121, "Remote I/O error")
            out.append(self._regs[(addr, reg + i)])
        return out

    def close(self):
        self.closed = True


class TempSensorAbsenceTests(unittest.TestCase):
    """When no sensor responds, construction returns a quiet inert obj."""

    def test_no_smbus_installed(self):
        from peppar_fix import temp_sensor
        # Re-import path simulates ImportError by patching the import.
        with mock.patch.dict("sys.modules", {"smbus2": None}):
            ts = temp_sensor.TempSensor(bus_num=99)
        self.assertFalse(ts.available)
        self.assertIsNone(ts.read_celsius())

    def test_no_device_file(self):
        from peppar_fix import temp_sensor
        with mock.patch.object(temp_sensor, "__name__", temp_sensor.__name__):
            with mock.patch.dict("sys.modules", {"smbus2": mock.Mock(SMBus=_NoBus)}):
                ts = temp_sensor.TempSensor(bus_num=99)
        self.assertFalse(ts.available)
        self.assertIsNone(ts.read_celsius())

    def test_no_device_responds(self):
        from peppar_fix import temp_sensor
        bus = _OpenBus()
        with mock.patch.dict("sys.modules",
                             {"smbus2": mock.Mock(SMBus=lambda *_: bus)}):
            ts = temp_sensor.TempSensor(bus_num=1)
        self.assertFalse(ts.available)
        self.assertIsNone(ts.read_celsius())


class TempSensorAdt7410Tests(unittest.TestCase):
    """ADT7410 detection + temp decode."""

    def test_detects_adt7410(self):
        from peppar_fix import temp_sensor
        bus = _OpenBus()
        bus.set_byte(0x48, 0x0B, 0xCB)            # ADT7410 ID
        bus.set_block(0x48, 0x00, [0x15, 0x18])   # 30 °C-ish
        with mock.patch.dict("sys.modules",
                             {"smbus2": mock.Mock(SMBus=lambda *_: bus)}):
            ts = temp_sensor.TempSensor(bus_num=1)
        self.assertTrue(ts.available)
        self.assertEqual(ts.label, "adt7410")
        self.assertEqual(ts.addr, 0x48)
        # 0x1518 = 5400 → 5400 × 0.0078125 = 42.1875
        self.assertAlmostEqual(ts.read_celsius(), 42.1875, places=4)

    def test_decode_negative(self):
        from peppar_fix import temp_sensor
        bus = _OpenBus()
        bus.set_byte(0x48, 0x0B, 0xCB)
        bus.set_block(0x48, 0x00, [0xFF, 0x80])   # -1.0 °C in 16-bit two's complement
        with mock.patch.dict("sys.modules",
                             {"smbus2": mock.Mock(SMBus=lambda *_: bus)}):
            ts = temp_sensor.TempSensor(bus_num=1)
        self.assertTrue(ts.available)
        # 0xFF80 = -128 → -128 × 0.0078125 = -1.0
        self.assertAlmostEqual(ts.read_celsius(), -1.0, places=4)

    def test_wrong_id_byte_not_detected(self):
        from peppar_fix import temp_sensor
        bus = _OpenBus()
        bus.set_byte(0x48, 0x0B, 0x00)   # not 0xCB
        with mock.patch.dict("sys.modules",
                             {"smbus2": mock.Mock(SMBus=lambda *_: bus)}):
            ts = temp_sensor.TempSensor(bus_num=1)
        self.assertFalse(ts.available)

    def test_read_returns_none_on_ioerror(self):
        from peppar_fix import temp_sensor
        bus = _OpenBus()
        bus.set_byte(0x48, 0x0B, 0xCB)
        # No temp register set → block read raises OSError.
        with mock.patch.dict("sys.modules",
                             {"smbus2": mock.Mock(SMBus=lambda *_: bus)}):
            ts = temp_sensor.TempSensor(bus_num=1)
        self.assertTrue(ts.available)
        self.assertIsNone(ts.read_celsius())


if __name__ == "__main__":
    unittest.main()

"""Tests for AD5693R 2× mode (GAIN bit) support in DacActuator (I-000711).

Verifies that DacActuator correctly writes the AD5693R control
register with the requested GAIN bit on setup, and that the default
preserves the chip's POR state (1× mode) for backward compatibility
with existing DOs (PiFace's CTI OSC5A2B02).
"""
from __future__ import annotations

import os
import sys
import types
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)


class _MockSMBus:
    """Mock smbus2.SMBus that records writes."""
    def __init__(self, bus_num):
        self.bus_num = bus_num
        self.writes = []  # list of (addr, cmd, data)

    def write_i2c_block_data(self, addr, cmd, data):
        self.writes.append((addr, cmd, list(data)))

    def close(self):
        pass


# DacActuator does a deferred `import smbus2` inside setup() so the
# library is optional in non-DAC environments.  Test runner doesn't
# have smbus2 installed; inject a stub module before the test class
# imports DacActuator.
_BUSES = []  # captured by stub for inspection in each test

def _stub_smbus_factory(bus_num):
    bus = _MockSMBus(bus_num)
    _BUSES.append(bus)
    return bus

if "smbus2" not in sys.modules:
    sys.modules["smbus2"] = types.SimpleNamespace(SMBus=_stub_smbus_factory)

from peppar_fix.dac_actuator import DacActuator  # noqa: E402


class DacActuatorGainBitTest(unittest.TestCase):

    def setUp(self):
        _BUSES.clear()

    def _setup_with_gain(self, dac_gain, dac_type="ad5693r"):
        """Create a DacActuator with given dac_gain, return (actuator, mock_bus)."""
        actuator = DacActuator(
            bus_num=1, addr=0x4c, bits=16,
            ppb_per_code=0.01, dac_type=dac_type,
            dac_gain=dac_gain,
        )
        actuator.setup()
        # The stub _stub_smbus_factory captures every SMBus(...) call.
        # Last entry is this actuator's bus.
        return actuator, _BUSES[-1]

    def test_default_gain_zero(self):
        """No dac_gain kwarg defaults to 0 (1× mode) for backward compat."""
        actuator = DacActuator(
            bus_num=1, addr=0x4c, bits=16,
            ppb_per_code=0.01, dac_type="ad5693r",
        )
        self.assertEqual(actuator._dac_gain, 0)

    def test_explicit_gain_zero_writes_ctrl_reg_with_gain_bit_clear(self):
        """dac_gain=0 still writes the control register (idempotent w.r.t. POR)
        but with GAIN bit (D11) clear → MSB byte 0x00."""
        _, mock_bus = self._setup_with_gain(0)
        # First write should be control register (cmd 0x40), then DAC code (cmd 0x30).
        ctrl_writes = [w for w in mock_bus.writes if w[1] == 0x40]
        self.assertEqual(len(ctrl_writes), 1,
                         f"expected exactly one control-register write, got "
                         f"{len(ctrl_writes)} in {mock_bus.writes}")
        addr, cmd, data = ctrl_writes[0]
        self.assertEqual(addr, 0x4c)
        self.assertEqual(cmd, 0x40)
        # D11 (GAIN) clear → MSB byte 0x00
        self.assertEqual(data[0], 0x00,
                         f"expected GAIN bit clear (MSB=0x00), got 0x{data[0]:02x}")
        self.assertEqual(data[1], 0x00,
                         "all other ctrl-reg bits should be 0 in default mode")

    def test_explicit_gain_one_writes_ctrl_reg_with_gain_bit_set(self):
        """dac_gain=1 writes the control register with GAIN bit (D11) set.

        D11 = bit 3 of the MSB byte = mask 0x08.  Earlier 0x10 was
        wrong — that's D12 (which is 'REF select' on AD5693, leaving
        the DAC output uncontrolled when no external reference is
        connected).  Verified against AD5693R datasheet 2026-05-08
        after a 2×-mode calibration on clkPoC3 showed flat (zero
        gain) DAC response across the full sweep — the smoking gun
        was the offset at code 32767 (= 2.5V) jumping from +798 ppb
        (1× mode, internal-ref operation) to +1109 ppb (Vctrl
        floating because external-ref was selected with no ref
        connected)."""
        _, mock_bus = self._setup_with_gain(1)
        ctrl_writes = [w for w in mock_bus.writes if w[1] == 0x40]
        self.assertEqual(len(ctrl_writes), 1)
        addr, cmd, data = ctrl_writes[0]
        # D11 in MSB byte → bit 3 → 0x08
        self.assertEqual(data[0], 0x08,
                         f"expected GAIN bit set at D11 (MSB=0x08), got 0x{data[0]:02x}")
        self.assertEqual(data[1], 0x00,
                         "no RESET, no PD: LSB byte 0")

    def test_ctrl_register_written_before_dac_code(self):
        """Control register must be written before any DAC code write so the
        chip is in the correct mode when the center-code write lands."""
        _, mock_bus = self._setup_with_gain(1)
        # Find indices of ctrl write (cmd 0x40) and code write (cmd 0x30).
        ctrl_idx = next((i for i, w in enumerate(mock_bus.writes) if w[1] == 0x40), None)
        code_idx = next((i for i, w in enumerate(mock_bus.writes) if w[1] == 0x30), None)
        self.assertIsNotNone(ctrl_idx, "ctrl-reg write missing")
        self.assertIsNotNone(code_idx, "DAC-code write missing")
        self.assertLess(ctrl_idx, code_idx,
                        "ctrl-register write must precede DAC-code write")

    def test_non_ad5693r_dac_does_not_write_ctrl_reg(self):
        """MCP4725 + generic DACs ignore dac_gain — they don't have a
        compatible control register and shouldn't see a 0x40 write."""
        actuator, mock_bus = self._setup_with_gain(1, dac_type="mcp4725")
        # MCP4725 fast-mode write uses cmd byte = upper 4 bits of code (= 0x00..0x0F).
        # No 0x40 control-register write should be issued.
        ctrl_writes = [w for w in mock_bus.writes if w[1] == 0x40]
        self.assertEqual(len(ctrl_writes), 0,
                         "non-AD5693R DAC types must not write 0x40 ctrl reg")

    def test_none_dac_gain_treated_as_zero(self):
        """Caller may pass dac_gain=None (e.g. CLI default) — treat as 0."""
        actuator = DacActuator(
            bus_num=1, addr=0x4c, bits=16,
            ppb_per_code=0.01, dac_type="ad5693r",
            dac_gain=None,
        )
        self.assertEqual(actuator._dac_gain, 0)


if __name__ == "__main__":
    unittest.main()

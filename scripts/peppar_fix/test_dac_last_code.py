"""Tests for DacActuator last_code startup + DO-state last-code persistence.

The DAC midpoint code is just the y-intercept of the linear ppb↔code
model — it carries no claim about the OCXO's nominal frequency.  On
startup the actuator should begin at the *last* code it was steered to
(the OCXO's actual operating point), not at center.  These tests cover:

- last_code starts the DAC there and computes _current_ppb from it
- absence of last_code falls back to center (0 ppb)
- teardown holds the last code (no center reset)
- save_last_dac_code / load_last_dac_code round-trip
- save functions inject unique_id so externally-written state files
  (dac_slope_cal.py output, which omits unique_id) don't silently
  fail to persist
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
import types
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)


class _MockSMBus:
    def __init__(self, bus_num):
        self.bus_num = bus_num
        self.writes = []

    def write_i2c_block_data(self, addr, cmd, data):
        self.writes.append((addr, cmd, list(data)))

    def close(self):
        pass


_BUSES = []


def _stub_smbus_factory(bus_num):
    bus = _MockSMBus(bus_num)
    _BUSES.append(bus)
    return bus


if "smbus2" not in sys.modules:
    sys.modules["smbus2"] = types.SimpleNamespace(SMBus=_stub_smbus_factory)

from peppar_fix.dac_actuator import DacActuator  # noqa: E402
from peppar_fix import do_state  # noqa: E402


class DacLastCodeStartupTest(unittest.TestCase):

    def setUp(self):
        _BUSES.clear()

    def test_last_code_starts_there(self):
        """When last_code is given, the DAC starts at that code (frame-
        independent warm start) and _current_ppb is the edge-anchored pull."""
        act = DacActuator(
            bus_num=1, addr=0x4c, bits=16, ppb_per_code=-0.008311,
            dac_type="ad5693r", dac_gain=1, last_code=36843,
            code_min=5000, code_max=60000, ppb_at_code_min=200.0)
        act.setup()
        self.assertEqual(act.current_code, 36843)
        # ppb(code) = ppb_at_code_min + (code - code_min)·slope
        self.assertAlmostEqual(act.read_frequency_ppb(),
                               200.0 + (36843 - 5000) * -0.008311, places=3)

    def test_no_last_code_parks_at_neutral(self):
        """Absent last_code, the DAC parks at the NEUTRAL command (ppb=0,
        ↔ adjfine=0) — not a magic center."""
        act = DacActuator(
            bus_num=1, addr=0x4c, bits=16, ppb_per_code=-0.008311,
            dac_type="ad5693r", dac_gain=1,
            code_min=5000, code_max=60000, ppb_at_code_min=200.0)
        act.setup()
        # neutral code = code_min + round((0 - ppb_at_code_min)/slope), clamped
        expected = max(5000, min(60000,
                                 5000 + round((0 - 200.0) / -0.008311)))
        self.assertEqual(act.current_code, expected)
        self.assertAlmostEqual(act.read_frequency_ppb(), 0.0, delta=0.01)

    def test_last_code_clamped_to_range(self):
        """A last_code beyond the DAC's code range is clamped."""
        act = DacActuator(
            bus_num=1, addr=0x4c, bits=16, ppb_per_code=0.01,
            dac_type="ad5693r", last_code=999999)
        act.setup()
        self.assertEqual(act.current_code, act._max_code)

    def test_teardown_holds_last_code(self):
        """teardown() must NOT reset to center — the OCXO keeps running
        at the last commanded frequency through restarts."""
        act = DacActuator(
            bus_num=1, addr=0x4c, bits=16, ppb_per_code=0.01,
            dac_type="ad5693r", last_code=40000)
        act.setup()
        # Access the bus through the actuator (the shared _BUSES global
        # is unreliable across test files — whichever installs the
        # smbus2 stub first wins).
        bus = act._bus
        writes_before = len(bus.writes)
        act.teardown()
        # No new DAC-code write (cmd 0x30) on teardown.
        new_code_writes = [w for w in bus.writes[writes_before:] if w[1] == 0x30]
        self.assertEqual(len(new_code_writes), 0,
                         "teardown must not write a new DAC code")


class LastDacCodePersistenceTest(unittest.TestCase):

    def setUp(self):
        self._tmp = tempfile.mkdtemp()

    def test_round_trip(self):
        do_state.save_last_dac_code("ocxo-x", 12345, state_dir=self._tmp)
        self.assertEqual(do_state.load_last_dac_code("ocxo-x", state_dir=self._tmp),
                         12345)

    def test_load_missing_returns_none(self):
        self.assertIsNone(
            do_state.load_last_dac_code("nope", state_dir=self._tmp))

    def test_stored_under_last_sentinel(self):
        """Code lives under dac_code_by_temperature['last']."""
        do_state.save_last_dac_code("ocxo-x", 22222, state_dir=self._tmp)
        path = do_state._do_path("ocxo-x", state_dir=self._tmp)
        with open(path) as f:
            j = json.load(f)
        self.assertEqual(j["dac_code_by_temperature"]["last"], 22222)

    def test_persists_into_external_cal_file(self):
        """A state file written by dac_slope_cal.py keys on do_label and
        omits unique_id.  save_last_dac_code / save_do_freq_offset must
        inject unique_id so the save actually lands (regression: silent
        no-op observed on MadHat 2026-05-27)."""
        path = do_state._do_path("isotemp-x", state_dir=self._tmp)
        os.makedirs(self._tmp, exist_ok=True)
        with open(path, "w") as f:
            json.dump({"do_label": "isotemp-x",
                       "dac_ppb_per_code": -0.0083,
                       "measurements": []}, f)
        do_state.save_last_dac_code("isotemp-x", 36843, state_dir=self._tmp)
        do_state.save_do_freq_offset("isotemp-x", -85.0, state_dir=self._tmp)
        with open(path) as f:
            j = json.load(f)
        self.assertEqual(j["dac_code_by_temperature"]["last"], 36843)
        self.assertEqual(j["last_known_freq_offset_ppb"], -85.0)
        # Pre-existing cal fields preserved.
        self.assertEqual(j["dac_ppb_per_code"], -0.0083)


if __name__ == "__main__":
    unittest.main()

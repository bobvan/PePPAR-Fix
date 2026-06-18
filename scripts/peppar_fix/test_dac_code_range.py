"""Tests for DacActuator linear-region code clamping (asymmetric EFC).

MadHat's IsoTemp OCXO-33 EFC saturates above code ~45000 (-26.5 ppb
floor).  Commands past the linear region must clamp to [code_min,
code_max], producing an asymmetric reachable frequency range.
See feedback_ocxo_asymmetric_pull_range.
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
    def __init__(self, bus_num):
        self.bus_num = bus_num
        self.writes = []

    def write_i2c_block_data(self, addr, cmd, data):
        self.writes.append((addr, cmd, list(data)))

    def close(self):
        pass


if "smbus2" not in sys.modules:
    sys.modules["smbus2"] = types.SimpleNamespace(SMBus=_MockSMBus)

from peppar_fix.dac_actuator import DacActuator  # noqa: E402


# MadHat geometry: ppb_per_code=-0.008311, linear region 5000..42000.
# Edge-anchored (noMagicCenterCode): ppb_at_code_min is the pull at
# code_min.  Chosen so ppb=0 lands at the old center 32768 — i.e. this
# frame is identical to the retired center=32768 frame, so the clamp/
# range assertions below are unchanged in meaning.
_MADHAT = dict(
    bus_num=1, addr=0x4c, bits=16, ppb_per_code=-0.008311,
    dac_type="ad5693r", dac_gain=1,
    ppb_at_code_min=(5000 - 32768) * -0.008311,
    code_min=5000, code_max=42000,
)


class CodeRangeClampTest(unittest.TestCase):

    def test_command_within_range_passes(self):
        act = DacActuator(**_MADHAT)
        act.setup()
        # -50 ppb → code = 32768 + (-50/-0.008311) = 32768 + 6016 = 38784
        act.adjust_frequency_ppb(-50.0)
        self.assertEqual(act.current_code, 32768 + round(-50.0 / -0.008311))
        self.assertFalse(act.at_range_limit)

    def test_command_past_slow_limit_clamps(self):
        """A large slow command (high code) clamps at code_max=42000."""
        act = DacActuator(**_MADHAT)
        act.setup()
        # -272 ppb → code ≈ 65500, way past code_max → clamp to 42000
        actual = act.adjust_frequency_ppb(-272.0)
        self.assertEqual(act.current_code, 42000)
        self.assertTrue(act.at_range_limit)
        # actual ppb reflects code_max, not the requested -272
        expected = (42000 - 32768) * -0.008311
        self.assertAlmostEqual(actual, expected, places=2)

    def test_command_past_fast_limit_clamps(self):
        """A large fast command (low code) clamps at code_min=5000."""
        act = DacActuator(**_MADHAT)
        act.setup()
        actual = act.adjust_frequency_ppb(+500.0)
        self.assertEqual(act.current_code, 5000)
        self.assertTrue(act.at_range_limit)

    def test_asymmetric_reachable_range(self):
        """Fast side (+397 ppb) reaches much further than slow side."""
        act = DacActuator(**_MADHAT)
        # code_min=5000: ppb = (5000-32768)*-0.008311 = +230.8 (fast)
        # code_max=42000: ppb = (42000-32768)*-0.008311 = -76.7 (slow)
        self.assertGreater(act._ppb_fast, 200)
        self.assertLess(act._ppb_slow, -50)
        # Confirm the asymmetry: fast range >> slow range
        self.assertGreater(abs(act._ppb_fast), abs(act._ppb_slow) * 2)

    def test_default_range_is_full(self):
        """Without code_min/max, the full 0..max_code range is used."""
        act = DacActuator(bus_num=1, addr=0x4c, bits=16,
                          ppb_per_code=0.01, dac_type="ad5693r")
        self.assertEqual(act._code_min, 0)
        self.assertEqual(act._code_max, 65535)

    def test_warning_fires_once(self):
        act = DacActuator(**_MADHAT)
        act.setup()
        act.adjust_frequency_ppb(-272.0)
        self.assertTrue(act._range_warned)
        # Second saturating command doesn't reset the flag
        act.adjust_frequency_ppb(-300.0)
        self.assertTrue(act._range_warned)


if __name__ == "__main__":
    unittest.main()

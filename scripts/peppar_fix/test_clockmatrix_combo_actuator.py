"""Tests for ClockMatrixComboActuator (combo-bus loop-filter-bypass steering).

Covers the validated otcBob1 DPLL3 recipe (2026-06-25):
  - combo encoding (signed 48-bit FFO × 2⁻⁵³, 6 bytes LE) round-trips
  - setup() saves originals, sets BW~0 + SLAVE0=0x28 + combo=0
  - teardown() restores exactly the saved bytes (hand-back to hardware DPLL)
  - adjust_frequency_ppb applies the per-host combo gain and clamps
  - sign/round-trip of adjust → read_frequency_ppb

Uses a dict-backed fake ClockMatrixI2C — no smbus2, no hardware.
"""
from __future__ import annotations

import os
import sys
import unittest

_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from peppar_fix.clockmatrix_combo_actuator import (  # noqa: E402
    ClockMatrixComboActuator, ppb_to_combo, combo_to_ppb,
    _COMBO_MAX, _COMBO_MIN,
)

# DPLL3 register addresses (from the validated PoC / register map).
_MODE = 0xC4B7
_SLAVE0 = 0xC4B2
_REF_MODE = 0xC4B5
_REF_P0 = 0xC48F
_BW = 0xC6C0
_COMBO = 0xC6E4


class _FakeI2C:
    """Byte-addressable register store mimicking ClockMatrixI2C.

    Backs a flat address→byte dict; multi-byte reads/writes are
    little-endian-contiguous, matching the real 16-bit-addressed device.
    Records the ordered write log for sequence assertions.
    """

    def __init__(self, initial=None):
        self.mem: dict[int, int] = {}
        self.writes: list[tuple[int, list[int]]] = []
        if initial:
            for addr, byte in initial.items():
                self.mem[addr] = byte

    def read(self, reg_addr: int, length: int) -> bytes:
        return bytes(self.mem.get(reg_addr + i, 0) for i in range(length))

    def write(self, reg_addr: int, data) -> None:
        data = list(data)
        self.writes.append((reg_addr, data))
        for i, byte in enumerate(data):
            self.mem[reg_addr + i] = byte & 0xFF

    def read_modify_write(self, reg_addr: int, mask: int, value: int) -> int:
        current = self.mem.get(reg_addr, 0)
        new_val = (current & ~mask) | (value & mask)
        self.write(reg_addr, [new_val])
        return new_val


def _timebeat_initial(ref_p0=2, ref_mode=0x01):
    """A plausible pre-existing DPLL3 state: PLL/auto, some BW, SLAVE0 already
    SRC_ID=8 (Timebeat's running value), a non-zero combo value.

    Defaults model the OTC (ref already CLK2/manual, so the ref pin is a no-op);
    pass ``ref_p0=5`` to model the OTC Mini (default REF_P0=CLK5, dead)."""
    init = {
        _MODE: 0x00,        # PLL/auto, state auto
        _SLAVE0: 0x28,      # EN + SRC_ID=8 (Timebeat's running value)
        _REF_MODE: ref_mode,
        _REF_P0: ref_p0,
        _BW: 0x34, _BW + 1: 0x12,  # arbitrary non-zero BW
    }
    # An arbitrary saved combo value (6 bytes).
    for i, b in enumerate((0x11, 0x22, 0x33, 0x44, 0x55, 0x06)):
        init[_COMBO + i] = b
    return init


class ComboEncodingTest(unittest.TestCase):

    def test_zero(self):
        self.assertEqual(ppb_to_combo(0.0), 0)
        self.assertEqual(combo_to_ppb(0), 0.0)

    def test_roundtrip_small(self):
        for ppb in (1.0, -1.0, 50.0, 200.0, -300.0, 0.111):
            val = ppb_to_combo(ppb)
            back = combo_to_ppb(val)
            # 1 LSB ≈ 1.11e-7 ppb, so round-trip is essentially exact.
            self.assertAlmostEqual(back, ppb, places=5)

    def test_sign(self):
        self.assertGreater(ppb_to_combo(100.0), 0)
        self.assertLess(ppb_to_combo(-100.0), 0)

    def test_matches_poc_formula(self):
        # The validated PoC used val = round((ppb/1e9) * 2**53).
        for ppb in (200.0, -300.0, 7.5):
            self.assertEqual(ppb_to_combo(ppb),
                             int(round((ppb / 1e9) * 2**53)))

    def test_clamps_to_48bit(self):
        self.assertEqual(ppb_to_combo(1e12), _COMBO_MAX)
        self.assertEqual(ppb_to_combo(-1e12), _COMBO_MIN)


class ComboActuatorSetupTeardownTest(unittest.TestCase):

    def test_setup_arms_combo_path(self):
        i2c = _FakeI2C(_timebeat_initial())
        act = ClockMatrixComboActuator(i2c, dpll_id=3)
        act.setup()
        # BW driven to ~0 (1 µHz = [0x01, 0x00]).
        self.assertEqual(list(i2c.read(_BW, 2)), [0x01, 0x00])
        # SLAVE0 subscribed to the SW-combo source.
        self.assertEqual(i2c.read(_SLAVE0, 1)[0], 0x28)
        # Combo value zeroed at arm time.
        self.assertEqual(int.from_bytes(i2c.read(_COMBO, 6), 'little'), 0)
        # MODE not rewritten when already PLL/auto (initial 0x00).
        self.assertNotIn(_MODE, [addr for addr, _ in i2c.writes])

    def test_setup_forces_pll_auto_when_write_freq(self):
        # The bootstrap may leave DPLL3 in write_freq (pll_mode=2); setup() must
        # FORCE PLL/auto so PHASE_STATUS is live (not the −409 µs artifact).
        init = _timebeat_initial()
        init[_MODE] = (2 << 3)  # pll_mode=2 (write_freq) = 0x10
        i2c = _FakeI2C(init)
        act = ClockMatrixComboActuator(i2c, dpll_id=3)
        act.setup()
        mode = i2c.read(_MODE, 1)[0]
        self.assertEqual((mode >> 3) & 0x07, 0,
                         "setup must force pll_mode→PLL (0) when starting write_freq")

    def test_teardown_restores_exact_saved_bytes_and_pll_auto(self):
        initial = _timebeat_initial()
        initial[_MODE] = (2 << 3)  # start in write_freq (bootstrap's doing)
        i2c = _FakeI2C(initial)
        act = ClockMatrixComboActuator(i2c, dpll_id=3)
        act.setup()
        act.adjust_frequency_ppb(123.0)  # perturb the combo register
        act.teardown()
        # BW, SLAVE0, and the full 6-byte combo value are back to originals.
        self.assertEqual(i2c.read(_BW, 2),
                         bytes(initial[_BW + i] for i in range(2)))
        self.assertEqual(i2c.read(_SLAVE0, 1)[0], initial[_SLAVE0])
        self.assertEqual(i2c.read(_COMBO, 6),
                         bytes(initial[_COMBO + i] for i in range(6)))
        # MODE handed back in PLL/auto (NOT the saved write_freq) — timebeat
        # does not auto-recover a write_freq DPLL.
        self.assertEqual((i2c.read(_MODE, 1)[0] >> 3) & 0x07, 0)


class ComboActuatorRefPinTest(unittest.TestCase):
    """setup() must pin the phase reference to CLK<pps_clk> so PHASE_STATUS is
    the live DO-vs-F9T error — the Mini fix (default REF_P0=CLK5 is dead)."""

    def test_pins_ref_clk2_on_mini_default_clk5(self):
        # OTC Mini: DPLL3 default REF_P0=CLK5, auto ref-mode → setup must
        # rewrite to CLK2 + manual.
        i2c = _FakeI2C(_timebeat_initial(ref_p0=5, ref_mode=0x00))
        act = ClockMatrixComboActuator(i2c, dpll_id=3)  # pps_clk default 2
        act.setup()
        self.assertEqual(i2c.read(_REF_P0, 1)[0], 2, "REF_P0 must be CLK2")
        self.assertEqual(i2c.read(_REF_MODE, 1)[0], 0x01, "REF_MODE manual")

    def test_ref_pin_is_noop_when_already_clk2_manual(self):
        # OTC: ref already CLK2/manual → setup must NOT touch the ref registers.
        i2c = _FakeI2C(_timebeat_initial(ref_p0=2, ref_mode=0x01))
        act = ClockMatrixComboActuator(i2c, dpll_id=3)
        act.setup()
        written = [addr for addr, _ in i2c.writes]
        self.assertNotIn(_REF_P0, written)
        self.assertNotIn(_REF_MODE, written)

    def test_teardown_restores_original_ref(self):
        # Mini: setup pins CLK2, teardown must hand REF_P0/REF_MODE back exactly.
        i2c = _FakeI2C(_timebeat_initial(ref_p0=5, ref_mode=0x00))
        act = ClockMatrixComboActuator(i2c, dpll_id=3)
        act.setup()
        self.assertEqual(i2c.read(_REF_P0, 1)[0], 2)  # pinned
        act.teardown()
        self.assertEqual(i2c.read(_REF_P0, 1)[0], 5, "REF_P0 restored to CLK5")
        self.assertEqual(i2c.read(_REF_MODE, 1)[0], 0x00, "REF_MODE restored")

    def test_custom_pps_clk(self):
        i2c = _FakeI2C(_timebeat_initial(ref_p0=5, ref_mode=0x00))
        act = ClockMatrixComboActuator(i2c, dpll_id=3, pps_clk=3)
        act.setup()
        self.assertEqual(i2c.read(_REF_P0, 1)[0], 3, "REF_P0 must be CLK3")


class ComboActuatorSteeringTest(unittest.TestCase):

    def test_adjust_naive_gain_roundtrip(self):
        i2c = _FakeI2C(_timebeat_initial())
        act = ClockMatrixComboActuator(i2c, dpll_id=3, combo_gain=1.0)
        act.setup()
        applied = act.adjust_frequency_ppb(200.0)
        self.assertAlmostEqual(applied, 200.0, places=4)
        self.assertAlmostEqual(act.read_frequency_ppb(), 200.0, places=4)

    def test_combo_gain_applied(self):
        # With gain 0.7, a request for 70 realized ppb must write 100 naive ppb.
        i2c = _FakeI2C(_timebeat_initial())
        act = ClockMatrixComboActuator(i2c, dpll_id=3, combo_gain=0.7)
        act.setup()
        applied = act.adjust_frequency_ppb(70.0)
        self.assertAlmostEqual(applied, 70.0, places=3)
        # Raw register holds the naive value (100 ppb).
        raw = int.from_bytes(i2c.read(_COMBO, 6), 'little')
        if raw & (1 << 47):
            raw -= (1 << 48)
        self.assertAlmostEqual(combo_to_ppb(raw), 100.0, places=3)

    def test_clamps_to_max_ppb(self):
        i2c = _FakeI2C(_timebeat_initial())
        act = ClockMatrixComboActuator(i2c, dpll_id=3, max_ppb=500.0)
        act.setup()
        self.assertAlmostEqual(act.adjust_frequency_ppb(9999.0), 500.0, places=2)
        self.assertAlmostEqual(act.adjust_frequency_ppb(-9999.0), -500.0, places=2)

    def test_negative_steer_roundtrip(self):
        i2c = _FakeI2C(_timebeat_initial())
        act = ClockMatrixComboActuator(i2c, dpll_id=3)
        act.setup()
        applied = act.adjust_frequency_ppb(-300.0)
        self.assertAlmostEqual(applied, -300.0, places=4)
        self.assertAlmostEqual(act.read_frequency_ppb(), -300.0, places=4)

    def test_zero_gain_rejected(self):
        i2c = _FakeI2C(_timebeat_initial())
        with self.assertRaises(ValueError):
            ClockMatrixComboActuator(i2c, dpll_id=3, combo_gain=0)

    def test_resolution_positive(self):
        i2c = _FakeI2C(_timebeat_initial())
        act = ClockMatrixComboActuator(i2c, dpll_id=3, combo_gain=0.7)
        self.assertGreater(act.resolution_ppb, 0)
        self.assertEqual(act.max_adj_ppb, 1000.0)


if __name__ == "__main__":
    unittest.main()

"""ClockMatrix *combo-bus* frequency actuator — steers Renesas 8A34002 by
summing a software frequency into the DCO **bypassing the loop filter**.

This is the no-wire, no-TICC discipline path validated on otcBob1 DPLL3
(2026-06-25, Timebeat/Phuong's find — see
``docs/clockmatrix-input-tdc-plan.md`` and
``docs/clockmatrix-platform-plan.md`` P1).  It is a sibling to
``clockmatrix_actuator.py`` (``ClockMatrixActuator``), which uses the
``write_freq`` (FCW) path.  The crucial difference:

  - **write_freq (FCW)** opens the hardware loop to steer — which *kills*
    the live ``PHASE_STATUS`` reading (the DPLL is no longer measuring).
    Steer XOR measure on a single channel.
  - **combo bus** sums a software FFO into the DCO *while the DPLL stays in
    PLL/auto*, so ``PHASE_STATUS`` (DO-vs-F9T-PPS, the on-chip TICC
    replacement) stays live.  Steer AND measure on the same channel, no
    external wire.  This is what lets one DPLL be a full software servo.

The recipe (DPLL3 / OTC addresses, validated):
  1. Leave the DPLL in **PLL/auto** (``MODE`` pll_mode bits = 0) — keeps
     ``PHASE_STATUS`` live.
  2. **Loop BW → ~0** (``BW`` = 1 µHz, ``[0x01, 0x00]``) so the hardware
     loop filter injects nothing (no GNSS-derived steering leaks in).
  3. **Subscribe to the SW-combo source**: ``COMBO_SLAVE_CFG_0`` = 0x28
     (``PRI_COMBO_SRC_EN=1, PRI_COMBO_SRC_ID=8``; SRC_ID 8 is the software
     combo value — 0..7 are real DPLLs).
  4. **Steer** by writing ``COMBO_SW_VALUE_CNFG`` (signed **48-bit** FFO ×
     2⁻⁵³, i.e. ``val = round(ppb/1e9 × 2⁵³)``) — added to the DCO via the
     combo bus.

``teardown()`` restores the saved registers, **handing the DO back to the
hardware DPLL** — the safe crash-fallback so the OTC never loses discipline.

Sign (validated open-loop): ``+combo ppb → DO speeds up → PHASE_STATUS ramps
negative`` (a late clock, phase > 0, catches up).  So combo ppb carries the
conventional ``FrequencyActuator`` sign: positive = speed up.  No inversion
at this boundary.

**Combo gain:** ``combo_gain`` = realized_ppb / naive_ppb.  Measured ≈ 1.0 on
otcBob1 DPLL3 (clockmatrix_combo_gain_char.py, 2026-06-25 — the naive datasheet
scaling is essentially correct; the PoC's "~0.7×" was a short-window +
PHASE_STATUS-glitch artifact, not a real discount).  Still MUST be measured per
host because the ratio is in principle per-chip.  Per Bob's
no-default-actuator-gain rule, engine integration (P3) refuses to build this
actuator without a measured value; the ``combo_gain=1.0`` default here is the
*naive* datasheet scaling, used ONLY by the calibration routine itself (which
runs at naive gain to measure the real one) and by unit tests.

**This actuator does NOT manage timebeat.**  It assumes it already has
exclusive ClockMatrix I2C access.  The timebeat stop/handoff/restore dance is
P3 (engine platform integration), one layer up.
"""

import logging

from peppar_fix.interfaces import FrequencyActuator
from peppar_fix.clockmatrix import ClockMatrixI2C

log = logging.getLogger(__name__)

# DPLL module bases (MODE = base+0x37, COMBO_SLAVE_CFG_0 = base+0x32)
_DPLL_BASES = {0: 0xC3B0, 1: 0xC400, 2: 0xC438, 3: 0xC480}
_DPLL_MODE_OFFSET = 0x37
_DPLL_COMBO_SLAVE0_OFFSET = 0x32

# DPLL_CTRL module bases (BW = base+0x04, COMBO_SW_VALUE_CNFG = base+0x28)
_DPLL_CTRL_BASES = {0: 0xC600, 1: 0xC63C, 2: 0xC680, 3: 0xC6BC}
_DPLL_CTRL_BW_OFFSET = 0x04
_DPLL_CTRL_COMBO_SW_OFFSET = 0x28

# MODE register pll_mode bits[5:3]: 0 = PLL (loop-controlled, phase live).
_PLL_MODE_SHIFT = 3
_PLL_MODE_MASK = 0x07 << _PLL_MODE_SHIFT
_PLL_MODE_PLL = 0

# COMBO_SLAVE_CFG_0: PRI_COMBO_SRC_EN=1 (bit5) + PRI_COMBO_SRC_ID=8 (bits[3:0]).
# 0x28 = 0b0010_1000.  SRC_ID 8 = the software combo value (0..7 = real DPLLs).
_COMBO_SLAVE_SW_SOURCE = 0x28

# BW = 1 µHz ≈ 0: the hardware loop filter contributes nothing.
_BW_ZERO = [0x01, 0x00]

# COMBO_SW_VALUE_CNFG is a signed 48-bit FFO field (6 bytes LE):
#   FFO = val × 2⁻⁵³, so val = round(ffo × 2⁵³) = round(ppb/1e9 × 2⁵³).
# NOTE: docs/timebeat-otc-register-map.md lists this register's size as 4 —
# that is WRONG (see docs/misnomers.md).  The validated PoC and the firmware
# both use a 6-byte / 48-bit signed field.
_COMBO_BYTES = 6
_COMBO_SCALE = 2**53
_COMBO_MAX = 2**47 - 1
_COMBO_MIN = -(2**47)

# Default steering envelope (realized ppb).  The combo register itself spans
# ±15 625 ppm, far beyond any OCXO's pull — so the meaningful bound is a
# runaway guard, not a hardware limit.  Overridable; the servo's own output
# clamp is the operational limiter.
_DEFAULT_MAX_ADJ_PPB = 1000.0


def ppb_to_combo(ppb: float) -> int:
    """Convert a *naive* frequency offset in ppb to the signed 48-bit combo value.

    ``val = round(ppb/1e9 × 2⁵³)``, clamped to the signed 48-bit range.  This
    is the raw datasheet scaling; the per-host combo gain is applied by the
    actuator, not here.
    """
    val = int(round((ppb / 1e9) * _COMBO_SCALE))
    return max(_COMBO_MIN, min(_COMBO_MAX, val))


def combo_to_ppb(val: int) -> float:
    """Convert a signed 48-bit combo value back to *naive* ppb (inverse of
    :func:`ppb_to_combo`)."""
    return val * 1e9 / _COMBO_SCALE


class ClockMatrixComboActuator(FrequencyActuator):
    """Frequency steering via the 8A34002 combo bus (loop-filter bypass).

    On ``setup()``, configures the DPLL's combo path (BW→0, subscribe to the
    SW-combo source) while leaving it in PLL/auto so ``PHASE_STATUS`` stays
    live.  On ``teardown()``, restores the saved registers — handing the DO
    back to the hardware DPLL (the safe fallback).

    Args:
        i2c: ClockMatrixI2C instance (bus + address configured by the caller;
            i2c-15 on OTC / i2c-16 on Mini — P3 platform detection).
        dpll_id: DPLL number (0-3).  Default 3 = the DO on otcBob1/ptBoat.
        combo_gain: realized_ppb / naive_ppb for this host's combo path
            (≈0.7 measured on otcBob1).  Default 1.0 = naive datasheet scaling,
            for the calibration routine + tests only; engine integration (P3)
            supplies a measured value and refuses the default.
        max_ppb: steering envelope (realized ppb).  Default 1000.0 (runaway
            guard); the servo's own output clamp is the real limiter.
    """

    def __init__(self, i2c: ClockMatrixI2C, dpll_id: int = 3,
                 combo_gain: float = 1.0, max_ppb: float | None = None):
        if combo_gain == 0:
            raise ValueError("combo_gain must be non-zero")
        self._i2c = i2c
        self._dpll_id = dpll_id
        self._combo_gain = float(combo_gain)
        self._max_ppb = (_DEFAULT_MAX_ADJ_PPB if max_ppb is None
                         else float(max_ppb))

        dpll_base = _DPLL_BASES[dpll_id]
        ctrl_base = _DPLL_CTRL_BASES[dpll_id]
        self._mode_reg = dpll_base + _DPLL_MODE_OFFSET
        self._slave0_reg = dpll_base + _DPLL_COMBO_SLAVE0_OFFSET
        self._bw_reg = ctrl_base + _DPLL_CTRL_BW_OFFSET
        self._combo_reg = ctrl_base + _DPLL_CTRL_COMBO_SW_OFFSET

        # Saved originals for teardown (hand-back to hardware DPLL).
        self._orig_mode: int | None = None
        self._orig_bw: list | None = None
        self._orig_slave0: int | None = None
        self._orig_combo: bytes | None = None
        self._current_ppb = 0.0

    def setup(self) -> None:
        """Configure the combo path: force PLL/auto, BW→0, subscribe to SW-combo.

        The combo bus only sums into the DCO while the DPLL is in PLL/auto (so
        ``PHASE_STATUS`` is the live DO-vs-F9T error).  We FORCE pll_mode→PLL
        because the engine bootstrap may have left DPLL3 in write_freq mode
        (the FCW path) — in which case ``PHASE_STATUS`` reads the fixed
        write_freq artifact (~−409 µs) and the servo trips the outlier gate
        (observed otcBob1 2026-06-25).  Saves MODE / BW / SLAVE0 / COMBO_SW for
        teardown.  Writes combo = 0 so we start from no software contribution.
        """
        self._orig_mode = self._i2c.read(self._mode_reg, 1)[0]
        pll_mode = (self._orig_mode >> _PLL_MODE_SHIFT) & 0x07
        if pll_mode != _PLL_MODE_PLL:
            pll_auto = self._orig_mode & ~_PLL_MODE_MASK  # pll_mode bits → 0
            self._i2c.write(self._mode_reg, [pll_auto])
            log.info("ClockMatrix combo DPLL_%d: forced MODE 0x%02X→0x%02X "
                     "(pll_mode %d→0, write_freq→PLL/auto) so PHASE_STATUS is live",
                     self._dpll_id, self._orig_mode, pll_auto, pll_mode)

        self._orig_bw = self._i2c.read(self._bw_reg, 2)
        self._orig_slave0 = self._i2c.read(self._slave0_reg, 1)[0]
        self._orig_combo = self._i2c.read(self._combo_reg, _COMBO_BYTES)
        log.info("ClockMatrix combo DPLL_%d: orig BW=%s SLAVE0=0x%02X "
                 "(SRC_ID=%d) — saved for hand-back",
                 self._dpll_id, list(self._orig_bw), self._orig_slave0,
                 self._orig_slave0 & 0x0F)

        self._i2c.write(self._bw_reg, _BW_ZERO)
        self._i2c.write(self._slave0_reg, [_COMBO_SLAVE_SW_SOURCE])
        self._write_combo_raw(0)
        self._current_ppb = 0.0

        slave0 = self._i2c.read(self._slave0_reg, 1)[0]
        log.info("ClockMatrix combo DPLL_%d: armed (BW~0, SLAVE0=0x%02X, "
                 "combo=0, gain=%.4f)", self._dpll_id, slave0, self._combo_gain)

    def teardown(self) -> None:
        """Restore saved registers — hand the DO back to the hardware DPLL.

        Hands DPLL3 back in **PLL/auto** (pll_mode→PLL), NOT whatever pre-combo
        mode was saved: the engine bootstrap may have left it in write_freq, and
        timebeat does NOT auto-recover a write_freq DPLL on restart (observed
        otcBob1 2026-06-25 — the host stayed mis-disciplined).  PLL/auto + the
        restored BW lets the hardware DPLL re-lock to F9T PPS.
        """
        if self._orig_combo is not None:
            self._i2c.write(self._combo_reg, list(self._orig_combo))
        if self._orig_slave0 is not None:
            self._i2c.write(self._slave0_reg, [self._orig_slave0])
        if self._orig_bw is not None:
            self._i2c.write(self._bw_reg, list(self._orig_bw))
        if self._orig_mode is not None:
            self._i2c.write(self._mode_reg, [self._orig_mode & ~_PLL_MODE_MASK])
        log.info("ClockMatrix combo DPLL_%d: restored (PLL/auto) — DO back on "
                 "hardware DPLL", self._dpll_id)
        self._current_ppb = 0.0

    def adjust_frequency_ppb(self, ppb: float) -> float:
        """Steer the DO to a realized offset of ``ppb`` via the combo bus.

        Writes ``naive = ppb / combo_gain`` so the realized shift is ``ppb``.
        Returns the realized ppb actually applied (after clamping).
        """
        clamped = max(-self._max_ppb, min(self._max_ppb, ppb))
        naive = clamped / self._combo_gain
        val = ppb_to_combo(naive)
        self._write_combo_raw(val)
        self._current_ppb = combo_to_ppb(val) * self._combo_gain
        return self._current_ppb

    def read_frequency_ppb(self) -> float:
        """Read back the combo value and return the realized ppb."""
        data = self._i2c.read(self._combo_reg, _COMBO_BYTES)
        raw = int.from_bytes(data, 'little')
        if raw & (1 << 47):
            raw -= (1 << 48)
        self._current_ppb = combo_to_ppb(raw) * self._combo_gain
        return self._current_ppb

    @property
    def max_adj_ppb(self) -> float:
        return self._max_ppb

    @property
    def resolution_ppb(self) -> float:
        # 1 combo LSB = 1e9/2⁵³ naive ppb (≈ 1.11e-7), × gain = realized.
        return abs(combo_to_ppb(1) * self._combo_gain)

    def _write_combo_raw(self, val: int) -> None:
        """Write the signed 48-bit combo value (6 bytes LE)."""
        val = max(_COMBO_MIN, min(_COMBO_MAX, val))
        raw = val & ((1 << 48) - 1)
        self._i2c.write(self._combo_reg, list(raw.to_bytes(_COMBO_BYTES, 'little')))

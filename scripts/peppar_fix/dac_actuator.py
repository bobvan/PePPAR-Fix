"""DAC-based frequency actuator for voltage-controlled oscillators.

Steers a VCOCXO by writing a voltage to a DAC over I2C.  The DAC
output voltage controls the varactor, which tunes the crystal frequency.

The mapping from ppb to DAC code depends on the oscillator's tuning
sensitivity (Hz/V) and the DAC's voltage range.  These must be
characterized per-oscillator and provided at construction.

Supported DACs:
- MCP4725 (12-bit, I2C, 0-Vcc)
- AD5693R (16-bit, I2C, 0-Vcc or 0-2×Vref)
- Generic: any DAC addressable via smbus2 write_word_data

See docs/state-persistence-design.md Phase 4.
"""

import logging
import struct

from peppar_fix.interfaces import FrequencyActuator

log = logging.getLogger(__name__)


class DacActuator(FrequencyActuator):
    """Frequency actuator via DAC → VCOCXO varactor.

    Args:
        bus_num: I2C bus number (e.g. 1 for /dev/i2c-1)
        addr: I2C device address (e.g. 0x60 for MCP4725)
        bits: DAC resolution in bits (12 for MCP4725, 16 for AD5693R)
        ppb_per_code: tuning sensitivity in ppb per DAC LSB.
            Positive = higher code → higher frequency.
            Must be characterized per-oscillator.
        code_min, code_max: usable linear code range (commands clamp here).
        ppb_at_code_min: OCXO pull (ppb from nominal) at code_min — the
            edge anchor of the steering line.  Together with ppb_per_code +
            code_min it fully defines code↔ppb; there is NO center_code.
            Default 0.0 suits sweep utilities that only call _write_code.
        max_ppb: maximum frequency adjustment range (default: computed
            from the linear range).
        dac_type: "mcp4725" or "ad5693r" or "generic"
    """

    def __init__(self, bus_num, addr, bits=12,
                 ppb_per_code=1.0, max_ppb=None, dac_type="mcp4725",
                 dac_gain=0, last_code=None, last_ppb=None,
                 code_min=None, code_max=None, ppb_at_code_min=0.0):
        self._bus_num = bus_num
        self._addr = addr
        self._bits = bits
        self._max_code = (1 << bits) - 1
        self._ppb_per_code = ppb_per_code
        self._dac_type = dac_type
        # Usable LINEAR code range — outside this the OCXO EFC saturates
        # (frequency clips and the ppb_per_code model is fiction).  Commands
        # clamp to [code_min, code_max].  Default None → full 0..max_code.
        self._code_min = 0 if code_min is None else max(0, int(code_min))
        self._code_max = (self._max_code if code_max is None
                          else min(self._max_code, int(code_max)))
        # noMagicCenterCode: the steering line is anchored at the LOWER EDGE,
        # never a privileged center.  ppb_at_code_min = the OCXO pull (ppb
        # from nominal) at code_min, so:
        #     ppb(code)  = ppb_at_code_min + (code − code_min)·ppb_per_code
        #     code(ppb)  = code_min + round((ppb − ppb_at_code_min)/ppb_per_code)
        # `ppb` here is the TRUE pull from nominal — identical in meaning to a
        # PHC's adjfine command.  The DAC is just the affine adapter from that
        # universal ppb correction to a code.  There is NO center_code: the old
        # midscale default was the magic-center error (broke on PiFace's
        # asymmetric 3.3 V DAC + CTI OCXO).  Default ppb_at_code_min=0.0 is the
        # benign sweep-tool case (utilities that only call _write_code, never
        # the servo mapping).
        self._ppb_at_code_min = float(ppb_at_code_min)
        # AD5693R control-register GAIN bit: 0 = 1× output (0..Vref),
        # 1 = 2× output (0..2×Vref).  POR default is 0; ignored on other DAC
        # types.  Required = 1 where the OCXO needs >0..Vref Vctrl to reach
        # GPS rate (e.g. clkPoC3 IsoTemp OCXO131-100).  See I-000711-main.
        self._dac_gain = int(dac_gain) if dac_gain is not None else 0
        self._bus = None
        self._last_code = last_code
        # Warm-start seed, "up a layer" (Bob 2026-06-18): the last-saved
        # frequency CORRECTION in ppb (true pull from nominal, ↔ adjfine).
        # Preferred over last_code because it is the actuator-agnostic
        # operating point — robust to a stale persisted dac_code (which on
        # PiFace lagged the operating code and made seed-by-code jump the DO
        # → glide blow-up).  Mapped to a code via the edge anchor at setup().
        self._last_ppb = last_ppb
        self._current_code = self._code_min
        self._current_ppb = self._ppb_for_code(self._code_min)
        self._range_warned = False

        # Reachable frequency range (ppb from nominal) at each code edge.
        ppb_lo = self._ppb_for_code(self._code_min)
        ppb_hi = self._ppb_for_code(self._code_max)
        self._ppb_fast = max(ppb_lo, ppb_hi)
        self._ppb_slow = min(ppb_lo, ppb_hi)
        # max_ppb kept for legacy callers (symmetric envelope = the smaller of
        # the two one-sided ranges about the GNSS-matching code).  But that
        # symmetric framing assumes a centered operating point; the honest
        # directional budget is _ppb_fast / _ppb_slow.  Explicit override wins.
        max_from_range = min(abs(self._ppb_fast), abs(self._ppb_slow))
        self._max_ppb = max_ppb if max_ppb is not None else max_from_range
        self._resolution_ppb = abs(ppb_per_code)

    def _ppb_for_code(self, code):
        """OCXO pull (ppb from nominal) the steering line predicts at ``code``."""
        return self._ppb_at_code_min + (code - self._code_min) * self._ppb_per_code

    def _code_for_ppb(self, ppb):
        """DAC code that yields pull ``ppb`` (unclamped), edge-anchored."""
        return self._code_min + round((ppb - self._ppb_at_code_min) / self._ppb_per_code)

    def setup(self):
        """Open I2C bus, configure control register (GAIN), set DAC.

        Warm-start seed priority (no magic midscale park):
          1. ``last_ppb`` — the last-saved frequency correction (true pull
             from nominal, ↔ adjfine).  PREFERRED: it is the actuator-agnostic
             operating point, mapped to a code via the edge anchor, and is
             robust to a stale persisted dac_code.
          2. ``last_code`` — the raw last code (frame-independent fallback when
             no ppb is recorded, e.g. first boot after a code-only migration).
          3. NEUTRAL — the code for ppb = 0 (DO at nominal, ↔ adjfine = 0),
             clamped to the linear range.
        """
        try:
            import smbus2
            self._bus = smbus2.SMBus(self._bus_num)
        except ImportError:
            raise ImportError("smbus2 required for DAC actuator: pip install smbus2")
        # Configure AD5693R control register before any data writes so the
        # chip is in the correct mode when we set the first code.  Other DAC
        # types ignore this — POR-default behavior preserved.
        if self._dac_type == "ad5693r":
            self._write_ad5693r_control_register()
        if self._last_ppb is not None:
            start_code = max(self._code_min,
                             min(self._code_max, self._code_for_ppb(self._last_ppb)))
            seed = "last_ppb (warm start)"
        elif self._last_code is not None:
            start_code = max(self._code_min,
                             min(self._code_max, int(self._last_code)))
            seed = "last_code (warm start)"
        else:
            # Neutral command: ppb = 0 (nominal), clamped to the linear range.
            start_code = max(self._code_min,
                             min(self._code_max, self._code_for_ppb(0.0)))
            seed = "neutral (ppb=0)"
        self._write_code(start_code)
        self._current_code = start_code
        self._current_ppb = self._ppb_for_code(start_code)
        log.info("DAC actuator: bus=%d addr=0x%02x bits=%d start=%d (%.1f ppb, "
                 "%s) ppb/code=%.4f ppb@code_min=%.1f range=[%d,%d] "
                 "gain=%d (%s mode)",
                 self._bus_num, self._addr, self._bits, start_code,
                 self._current_ppb, seed, self._ppb_per_code,
                 self._ppb_at_code_min, self._code_min, self._code_max,
                 self._dac_gain, "2×" if self._dac_gain else "1×")

    def teardown(self):
        """Close bus.  DAC holds its last code — the OCXO keeps running
        at the last commanded frequency through restarts."""
        if self._bus is not None:
            log.info("DAC teardown: holding last code %d (%.1f ppb)",
                     self._current_code, self._current_ppb)
            self._bus.close()
            self._bus = None

    def adjust_frequency_ppb(self, ppb):
        """Set the DO's pull to ``ppb`` (true pull from nominal, ↔ adjfine).
        Returns the actual ppb applied.

        Edge-anchored: code = code_min + round((ppb − ppb_at_code_min)/slope),
        clamped to the usable linear range [code_min, code_max].  A command
        outside the range saturates at the boundary and logs once — the OCXO
        physically cannot reach the requested frequency (EFC out of linear
        range, normal for an asymmetric/temperature-shifted OCXO).
        """
        code = self._code_for_ppb(ppb)
        clamped = max(self._code_min, min(self._code_max, code))
        if clamped != code and not self._range_warned:
            log.warning(
                "DAC command %d (%.1f ppb) outside linear range "
                "[%d, %d] — saturating at %d.  OCXO cannot reach "
                "requested frequency (asymmetric EFC range).",
                code, ppb, self._code_min, self._code_max, clamped)
            self._range_warned = True
        code = clamped
        self._write_code(code)
        self._current_code = code
        self._current_ppb = self._ppb_for_code(code)
        return self._current_ppb

    @property
    def at_range_limit(self):
        """True if the last command saturated against a code-range limit."""
        return self._current_code in (self._code_min, self._code_max)

    def read_frequency_ppb(self):
        """Return last-written frequency offset."""
        return self._current_ppb

    @property
    def max_adj_ppb(self):
        return self._max_ppb

    @property
    def resolution_ppb(self):
        return self._resolution_ppb

    @property
    def current_code(self):
        return self._current_code

    def _write_ad5693r_control_register(self):
        """Write AD5693R control register with the configured GAIN bit.

        Control-register layout (AD5693R/AD5692R/AD5691R/AD5693
        datasheet page 22, "Write Control Register" command 0x40):
          D15     = RESET  (1 = soft reset to power-on defaults)
          D14     = PD1    (power-down mode bit 1)
          D13     = PD0    (power-down mode bit 0)
          D12     = REF    (external reference select; AD5693 only —
                            must be 0 on -R variants, which have a
                            fixed internal reference; selecting
                            external on an -R part leaves DAC output
                            uncontrolled because there is no external
                            reference pin to drive)
          D11     = GAIN   (0 = 1× output 0..Vref, 1 = 2× output 0..2×Vref)
          D10..D0 = reserved (write 0)

        D11 → bit 3 of the MSB byte → mask 0x08.
        D15..D8 = MSB byte (sent first), D7..D0 = LSB byte.

        Sent as 3 bytes after the I2C address: command 0x40, then
        16 bits of control data MSB-first.
        """
        if self._bus is None:
            raise RuntimeError("DAC bus not open — call setup() first")
        gain_bit = 0x08 if self._dac_gain else 0x00  # D11 in MSB byte
        self._bus.write_i2c_block_data(self._addr, 0x40,
                                       [gain_bit, 0x00])

    def _write_code(self, code):
        """Write a DAC code to the device."""
        if self._bus is None:
            raise RuntimeError("DAC bus not open — call setup() first")

        if self._dac_type == "mcp4725":
            # MCP4725: fast mode write (2 bytes, upper 4 bits = 0b0000)
            # Byte 0: [0 0 PD1 PD0 D11 D10 D9 D8]
            # Byte 1: [D7 D6 D5 D4 D3 D2 D1 D0]
            high = (code >> 8) & 0x0F
            low = code & 0xFF
            self._bus.write_i2c_block_data(self._addr, high, [low])

        elif self._dac_type == "ad5693r":
            # AD5693R: write DAC register (command 0x30)
            # 3 bytes: cmd, MSB, LSB (16-bit left-aligned)
            msb = (code >> 8) & 0xFF
            lsb = code & 0xFF
            self._bus.write_i2c_block_data(self._addr, 0x30, [msb, lsb])

        else:
            # Generic: write 16-bit value to register 0
            msb = (code >> 8) & 0xFF
            lsb = code & 0xFF
            self._bus.write_i2c_block_data(self._addr, 0x00, [msb, lsb])

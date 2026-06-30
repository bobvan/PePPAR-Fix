"""TADD-2 Mini divider ARM control via GPIO.

The TAPR TADD-2 Mini divides a 10 MHz input down to 1 PPS.  The ARM
pin resets the divider counter: holding it LOW for >1 second causes
the divider to restart from zero on the next SYNC PPS edge (from the
GNSS receiver).  The first PPS output appears within 4 cycles of the
input clock (≤400 ns lag for 10 MHz).

This module drives the ARM pin via libgpiod, falling back to a
no-op mock on hosts without GPIO hardware.

Adapted from github.com/bobvan/clkPoC TADD.py for peppar-fix.
"""

import contextlib
import glob
import logging
import time

log = logging.getLogger(__name__)

try:
    import gpiod as _GPIOD
except Exception:
    _GPIOD = None


def _open_gpio_output(offset, consumer="peppar-tadd"):
    """Open a GPIO line as output, HIGH initial state.

    Tries libgpiod v2 then v1, returns a set_value callable and a
    close callable.

    Fails LOUD on a real divider host: if GPIO hardware is present
    (/dev/gpiochip*) but gpiod is missing or no chip accepts the line,
    raises RuntimeError instead of returning a no-op mock.  A silent
    no-op ARM is the worst outcome — it looks like the divider runs but
    never GNSS-locks (cost a red-herring SYNC-cable swap on PiPuss
    2026-06-30).  The no-op mock survives ONLY on hosts with no GPIO
    hardware at all (dev box / CI / tests).
    """
    chips = sorted(glob.glob("/dev/gpiochip*"))

    if _GPIOD is None:
        if chips:
            raise RuntimeError(
                "TADD ARM requires the 'gpiod' Python module, but it is not "
                f"installed while GPIO hardware is present ({', '.join(chips)}). "
                "Without it the divider ARM is a silent no-op and the DO PPS "
                "never GNSS-locks.  Install it: pip install gpiod  "
                "(declared in the .[timebeat] extra and requirements.txt)."
            )
        log.warning("TADD: gpiod not installed and no GPIO hardware present "
                    "— ARM is a no-op (non-GPIO host / test).")
        return lambda v: None, lambda: None

    # libgpiod v2
    if hasattr(_GPIOD, "request_lines"):
        settings = _GPIOD.LineSettings(
            direction=_GPIOD.line.Direction.OUTPUT,
            output_value=_GPIOD.line.Value.ACTIVE,
        )
        for chip_path in sorted(glob.glob("/dev/gpiochip*")):
            try:
                req = _GPIOD.request_lines(
                    chip_path, consumer=consumer,
                    config={offset: settings},
                )
                log.info("TADD: GPIO%d on %s (gpiod v2)", offset, chip_path)
                def _set(v, _r=req, _o=offset):
                    _r.set_value(_o,
                        _GPIOD.line.Value.ACTIVE if v else _GPIOD.line.Value.INACTIVE)
                def _close(_r=req):
                    with contextlib.suppress(Exception):
                        _r.release()
                return _set, _close
            except Exception:
                continue

    # libgpiod v1
    for chip_dev in sorted(glob.glob("/dev/gpiochip*")):
        try:
            chip = _GPIOD.Chip(chip_dev)
            line = chip.get_line(offset)
            line.request(consumer=consumer, type=getattr(_GPIOD, "LINE_REQ_DIR_OUT", 1),
                         default_val=1)
            log.info("TADD: GPIO%d on %s (gpiod v1)", offset, chip_dev)
            def _set(v, _l=line):
                _l.set_value(v)
            def _close(_l=line, _c=chip):
                with contextlib.suppress(Exception):
                    _l.release()
                with contextlib.suppress(Exception):
                    _c.close()
            return _set, _close
        except Exception:
            with contextlib.suppress(Exception):
                chip.close()
            continue

    if chips:
        raise RuntimeError(
            f"TADD ARM: gpiod is installed but no gpiochip accepted GPIO{offset} "
            f"(tried {', '.join(chips)}).  Check the arm_gpio number and that the "
            "line isn't already held by another process."
        )
    log.warning("TADD: no GPIO hardware present — ARM is a no-op "
                "(non-GPIO host / test).")
    return lambda v: None, lambda: None


class TADDDivider:
    """TADD-2 Mini divider with ARM synchronization.

    Args:
        arm_gpio: BCM GPIO number for the ARM pin (default: 16)
        arm_hold_s: how long to hold ARM low (default: 1.1s, spec requires >1s)
        max_phase_offset_ns: maximum expected phase offset after ARM
            (spec: ≤400 ns for 10 MHz input)
    """

    def __init__(self, arm_gpio=16, arm_hold_s=1.1, max_phase_offset_ns=400):
        self.arm_gpio = arm_gpio
        self.arm_hold_s = arm_hold_s
        self.max_phase_offset_ns = max_phase_offset_ns
        self._set_value = None
        self._close = None

    def setup(self):
        """Initialize GPIO for ARM pin control."""
        self._set_value, self._close = _open_gpio_output(self.arm_gpio)

    def teardown(self):
        """Release GPIO resources."""
        if self._close is not None:
            self._close()
            self._set_value = None
            self._close = None

    def arm(self):
        """Pulse ARM pin to synchronize divider output to next GNSS PPS.

        Holds ARM LOW for >1 second per TADD-2 Mini spec.  After
        release, the divider restarts on the next rising edge of the
        SYNC input (F9T PPS).  The first output PPS appears within
        max_phase_offset_ns of the SYNC edge.

        Returns:
            float: CLOCK_MONOTONIC timestamp when ARM was released
                (the divider will sync on the next PPS after this time)
        """
        if self._set_value is None:
            log.warning("TADD: not initialized — call setup() first")
            return time.monotonic()

        log.info("TADD: arming divider (GPIO%d LOW for %.1fs)...",
                 self.arm_gpio, self.arm_hold_s)
        self._set_value(0)
        time.sleep(self.arm_hold_s)
        self._set_value(1)
        released = time.monotonic()
        log.info("TADD: ARM released — divider will sync on next PPS "
                 "(phase offset ≤%d ns)", self.max_phase_offset_ns)
        return released

    def __enter__(self):
        self.setup()
        return self

    def __exit__(self, *_):
        self.teardown()

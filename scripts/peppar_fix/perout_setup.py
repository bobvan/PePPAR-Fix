"""Full PEROUT setup chain for SDP-pin-driven PPS OUT.

Combines the three steps the engine performs when it brings a PHC
PEROUT online:

  1. Pin function assignment (PTP_PIN_SETFUNC ioctl, with E810 sysfs
     fallback when the ioctl isn't supported).
  2. PEROUT enable (PTP_PEROUT_REQUEST2 with correct start_nsec for
     the driver, handled by PtpDevice.enable_perout).
  3. Optional phase verification via TICC, retrying on the igc
     hardware half-period latch bug.

This was previously inlined as ``_enable_pps_out`` in
``scripts/phc_bootstrap.py``.  Extracted 2026-05-16 so that
``tools/calibrate_do.py`` (PHC path) and ``scripts/perout_kick.py``
can drive the same setup chain — previously only the engine's
bootstrap path got the full setup, leaving the characterization tool
unable to produce chA edges on hosts where PEROUT had been disabled.

See dayplan item ``calToolPhcPerout-main`` for the motivation.
"""

from __future__ import annotations

import logging
import time

log = logging.getLogger(__name__)


def ticc_check_perout_phase(ticc_port: str, timeout_s: float = 8.0):
    """Read TICC briefly; check if chA and chB are aligned or 500ms apart.

    Returns (aligned, chA_frac, chB_frac) where aligned is True if
    the fractional seconds are within 100ms; None if no data captured
    or the port can't be opened.

    Opens the TICC with ``HUPCL`` disabled so closing the port doesn't
    DTR-reset the Arduino.  See CLAUDE.md "TICC resets on serial open".
    """
    import serial
    import termios
    try:
        ser = serial.Serial(ticc_port, 115200, dsrdtr=False,
                            rtscts=False, timeout=2.0)
        attrs = termios.tcgetattr(ser.fd)
        attrs[2] &= ~termios.HUPCL
        termios.tcsetattr(ser.fd, termios.TCSANOW, attrs)
    except (OSError, serial.SerialException) as e:
        log.warning("Cannot open TICC %s for phase check: %s", ticc_port, e)
        return None

    chA_frac = None
    chB_frac = None
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            line = ser.readline().decode(errors='replace').strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) >= 2:
                try:
                    ts = float(parts[0])
                    frac = ts % 1.0
                    if 'chA' in parts[1]:
                        chA_frac = frac
                    elif 'chB' in parts[1]:
                        chB_frac = frac
                except ValueError:
                    continue
            if chA_frac is not None and chB_frac is not None:
                break
    finally:
        ser.close()

    if chA_frac is None or chB_frac is None:
        log.warning("TICC phase check: missing channel data "
                    "(chA=%s chB=%s)", chA_frac, chB_frac)
        return None

    delta = abs(chA_frac - chB_frac)
    if delta > 0.5:
        delta = 1.0 - delta  # handle wrap
    aligned = delta < 0.1  # within 100ms
    log.info("TICC phase check: chA=%.3fs chB=%.3fs delta=%.3fs %s",
             chA_frac, chB_frac, delta,
             "ALIGNED" if aligned else "500ms OFF")
    return (aligned, chA_frac, chB_frac)


def setup_perout(ptp,
                 pin_index: int,
                 channel: int,
                 *,
                 period_ns: int = 1_000_000_000,
                 program_pin: bool = True,
                 ptp_dev_path: str | None = None,
                 verify_via_ticc_port: str | None = None,
                 max_attempts: int = 4) -> bool:
    """Configure SDP pin function + enable PEROUT, optionally TICC-verify.

    The full chain the engine runs when it brings up disciplined PPS OUT:

      1. PIN_SETFUNC ioctl on ``pin_index`` → PEROUT, channel ``channel``.
         If the ioctl raises ``OSError`` (E810 ice driver rejects it),
         fall back to sysfs at /sys/class/ptp/<ptpN>/pins/<name>.
      2. ``enable_perout(channel)`` — handles start_nsec for igc/non-igc.
      3. If ``verify_via_ticc_port`` is set: wait ~4 s for PEROUT to
         start, then read TICC chA + chB.  igc Target Time hardware has
         a half-period latch that randomly picks the wrong start_nsec
         polarity ~50% of the time; retry with the opposite offset up
         to ``max_attempts`` times.

    Args:
        ptp: PtpDevice instance, already opened.
        pin_index: SDP pin number (e.g., 0 for SDP0 on i226).
        channel: PEROUT channel (typically 0).
        period_ns: pulse period, default 1 Hz.
        program_pin: try the ioctl pin assignment.  Set False if a
            previous bootstrap already configured the pin and we just
            need PEROUT re-enabled.
        ptp_dev_path: path string for sysfs fallback (e.g.,
            ``/dev/ptp_i226``).  Required if program_pin=True and the
            ioctl might fail (E810).  Pass ``ptp.path`` typically.
        verify_via_ticc_port: serial port for phase verification.  If
            None, no verification — accept whichever PEROUT phase the
            hardware lands on.
        max_attempts: retry limit when phase verification fails.

    Returns:
        bool: True if PEROUT enabled (and verified aligned, if requested);
              False if pin programming failed entirely.  Misalignment
              after max_attempts logs an error but still returns True.

    Raises:
        Nothing.  All failures logged and returned via the bool.
    """
    from peppar_fix.ptp_device import PTP_PF_PEROUT
    try:
        pin_set = False
        if program_pin:
            try:
                ptp.set_pin_function(pin_index, PTP_PF_PEROUT, channel)
                pin_set = True
            except OSError as e:
                log.debug("PTP_PIN_SETFUNC ioctl failed (%s) — trying sysfs",
                          e)
        if not pin_set:
            # E810 ice driver rejects the ioctl; sysfs path works.
            # Caller may pass an explicit ptp_dev_path; otherwise try
            # to derive from ptp.
            dev = ptp_dev_path or getattr(ptp, "path", None)
            if dev is None:
                log.error("Cannot resolve PTP device path for sysfs "
                          "fallback; aborting PEROUT setup")
                return False
            try:
                from phc_bootstrap import (_set_pin_function_sysfs,
                                            _E810_PIN_NAMES)
                pin_name = _E810_PIN_NAMES.get(pin_index, str(pin_index))
                _set_pin_function_sysfs(dev, pin_name, PTP_PF_PEROUT,
                                        channel)
                pin_set = True
            except (ImportError, OSError) as e:
                log.error("sysfs PEROUT pin setup failed: %s", e)
                return False
    except OSError as e:
        log.warning("Failed PEROUT pin setup: %s", e)
        return False

    # Try PEROUT enable + (optional) TICC phase verification with retry.
    # Offsets alternate to test both polarities of the igc half-period
    # latch.  ``None`` = auto-detect via enable_perout's igc handling.
    offsets = [None, 0, 500_000_000, 0]
    for attempt in range(1, max_attempts + 1):
        override = offsets[(attempt - 1) % len(offsets)]
        try:
            if override is not None:
                ptp.enable_perout(channel, period_ns=period_ns,
                                  start_nsec_override=override)
            else:
                ptp.enable_perout(channel, period_ns=period_ns)
        except OSError as e:
            log.warning("enable_perout failed on attempt %d: %s", attempt, e)
            return False
        log.info("PEROUT programmed (attempt %d/%d, start_nsec=%s)",
                 attempt, max_attempts,
                 "auto" if override is None else override)

        if not verify_via_ticc_port:
            # No verification requested — accept whichever phase the
            # hardware picked.
            break

        # Wait for PEROUT to start (start_sec = phc_now + 2 in
        # enable_perout) before TICC sees edges.
        time.sleep(4)
        result = ticc_check_perout_phase(verify_via_ticc_port)
        if result is None:
            log.warning("TICC phase check inconclusive — accepting")
            break
        aligned, _chA, _chB = result
        if aligned:
            log.info("PEROUT phase verified via TICC on attempt %d", attempt)
            break
        if attempt < max_attempts:
            log.warning("PEROUT 500ms off — trying opposite offset "
                        "(attempt %d/%d)", attempt, max_attempts)
            try:
                ptp.disable_perout(channel)
            except OSError:
                pass
            time.sleep(1)
        else:
            log.error("PEROUT still 500ms off after %d attempts — "
                      "hardware half-period latch.  May need driver "
                      "reload (rmmod igc && modprobe igc).",
                      max_attempts)

    log.info("PPS OUT enabled: pin %d, PEROUT channel %d",
             pin_index, channel)
    return True

"""
ticc.py — Serial reader for the TAPR TICC time interval counter.

The TICC outputs one line per PPS edge:
    <seconds_since_boot>  chA|chB
e.g.
    402.342588195696 chA
    402.342588174417 chB

Timestamps have 11–12 decimal places.  Older firmware used 12 (1 ps LSB);
newer firmware uses 11 (10 ps LSB).  The counter's single-shot noise is ~60 ps,
so the last displayed digit is noise in either case.
Lines starting with '#' are comments (boot-time header); they are skipped.

Output ordering: the TICC outputs whichever channel's timestamp is ready
first.  When both channels fire close together (e.g. same PPS source split
to both inputs), chB for second N may appear before chA for second N-1.
This is documented TICC behavior, not a parsing error.

Boot behavior (Arduino Mega auto-reset):
  Opening the serial port on an Arduino Mega toggles DTR, which resets
  the microcontroller via a hardware capacitor.  The TICC then:
    1. Prints a config header (# lines)
    2. Waits ~5 seconds for config menu input ("# ....")
    3. Prints "# timestamp (seconds)"  ← sentinel
    4. Begins outputting timestamp lines at ref_sec=1

  The OS serial buffer may contain stale timestamps from before the
  reboot.  These have large ref_sec values and different ref_ps ranges
  from the fresh post-boot data.

  Callers should either:
    (a) Use wait_for_boot=True (default) to automatically wait for the
        boot sentinel and discard stale data.  Takes ~10 seconds.
    (b) Use wait_for_boot=False when the port is already open and the
        TICC is known to be running (e.g. between calibration runs).

Precision notes:
  - Integer and fractional parts are parsed separately to avoid float64
    precision loss.  float64 has ~15-16 significant digits total; a
    6-digit integer part leaves only ~9 decimal digits, losing ps
    resolution after ~28 hours of TICC uptime.
  - ref_sec and ref_ps are returned as Python ints (arbitrary precision).
    Convert to float only at the final analysis stage.
"""

from __future__ import annotations

import logging
import re
import termios
import time as _time
import atexit

import serial

log = logging.getLogger(__name__)

from peppar_fix.event_time import (
    TiccEvent,
    estimate_correlation_confidence,
    estimator_sample_weight,
)

from peppar_fix.timebase_estimator import TimebaseRelationEstimator

# Integer part DOT 11-or-12 fractional digits whitespace ch followed by A or B.
_LINE_RE = re.compile(r"^(\d+)\.(\d{11,12})\s+(ch[AB])$")

# Boot sentinel: the TICC prints this line just before starting timestamp output.
_BOOT_SENTINEL = "# timestamp"

_shared_ticc_ports: dict[tuple[str, int], "_SharedTiccPort"] = {}


class _SharedTiccPort:
    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.serial: serial.Serial | None = None
        self.refcount = 0
        self.booted = False

    def _open_serial(self) -> serial.Serial:
        # dsrdtr=False, rtscts=False: tell pyserial not to manage
        # DTR/RTS for hardware flow control.
        #
        # HUPCL clear: prevent the kernel from dropping DTR when the
        # last fd closes.  Without this, closing the port drops DTR,
        # and the next open raises it — the rising edge triggers the
        # Arduino Mega 2560's auto-reset via the DTR-RESET capacitor,
        # rebooting the TICC (~10s of lost data).  With HUPCL cleared,
        # DTR stays asserted across close/reopen, so subsequent opens
        # (same or different process) don't trigger a reboot.
        # See CLAUDE.md "TICC resets on serial open".
        #
        # exclusive=True: kernel-enforced exclusive open (TIOCEXCL).
        # Prevents other processes from opening the same tty.
        # Automatically released when the fd is closed, even on
        # crash or SIGKILL — no stale lock files to clean up.
        try:
            ser = serial.Serial(
                self.port,
                self.baud,
                timeout=2.0,
                dsrdtr=False,
                rtscts=False,
                exclusive=True,
            )
        except serial.SerialException:
            ser = serial.Serial(
                self.port,
                self.baud,
                timeout=2.0,
                dsrdtr=False,
                rtscts=False,
                exclusive=False,
            )
        # Clear HUPCL so DTR stays asserted after close.
        attrs = termios.tcgetattr(ser.fd)
        attrs[2] &= ~termios.HUPCL  # cflag
        termios.tcsetattr(ser.fd, termios.TCSANOW, attrs)
        return ser

    def acquire(self, wait_for_boot: bool) -> None:
        if self.serial is None:
            self.serial = self._open_serial()
            self.serial.reset_input_buffer()
            self.booted = False
        self.refcount += 1
        if wait_for_boot and not self.booted:
            self._wait_for_boot()
            self.booted = True

    def release(self) -> None:
        if self.refcount > 0:
            self.refcount -= 1

    def _force_dtr_reset(self) -> None:
        """Force the Arduino to reboot by pulsing DTR low-then-high.

        Needed when the TICC is in the cold state where DTR is already
        asserted (e.g. after a ModemManager probe at boot), so the
        normal open doesn't produce the rising edge that triggers the
        Arduino's auto-reset capacitor.
        """
        assert self.serial is not None
        try:
            self.serial.dtr = False
            _time.sleep(0.5)
            self.serial.dtr = True
        except (serial.SerialException, OSError):
            pass

    # Boot-recovery tunables.  Class attributes so tests can override them
    # to keep test runs fast (the timing constants assume a real TICC at
    # the other end of the wire).
    _BOOT_MAX_RESET_ATTEMPTS = 3
    _BOOT_PER_ATTEMPT_BUDGET_S = 18.0   # banner + 5 s menu wait + settle
    _BOOT_SILENCE_THRESHOLD_S = 3.5     # no bytes at all → cold
    _BOOT_POST_SENTINEL_THRESHOLD_S = 4.0  # sentinel printed but no timestamps
    _BOOT_POST_HEADER_THRESHOLD_S = 9.0    # headers but no sentinel/timestamps

    def _wait_for_boot(self) -> None:
        """Wait until the TICC is producing timestamp lines.

        DTR is toggled lazily — we touch it only when we have evidence
        the device is wedged.  In the warm-path case (TICC has been
        running for hours, we just opened the port from another
        process), the first ``readline()`` returns a timestamp and we
        return immediately without ever pulsing DTR.

        Recovery path: up to ``_BOOT_MAX_RESET_ATTEMPTS`` DTR resets.
        Each attempt detects three distinct failure modes:

          - **cold**: no bytes at all on the wire for
            ``_BOOT_SILENCE_THRESHOLD_S``.  DTR was never pulsed
            (because HUPCL is cleared) and the Arduino is in
            pre-boot state.

          - **stuck after sentinel**: the boot sentinel line
            (``# timestamp``) printed but timestamps never started
            within ``_BOOT_POST_SENTINEL_THRESHOLD_S``.  This means
            something disturbed the chip after the boot wait
            completed — extremely rare but possible if a process is
            actively writing to the port.

          - **stuck in headers**: more than
            ``_BOOT_POST_HEADER_THRESHOLD_S`` of header lines without
            ever seeing the boot sentinel.  This is the classic
            "menu-stuck" symptom: the firmware received a stray byte
            during the 5 s menu wait and dropped into the interactive
            setup menu, which prints the configuration on a loop and
            never gets to the sentinel.

        On any of those three, force a DTR low-then-high pulse to
        reboot the Arduino and try again.

        We **never** write to the serial port — the TICC firmware
        interprets any byte during the boot menu wait as a key press
        and enters the interactive setup menu, which is exactly the
        state we're trying to escape from.  The only acceptable
        recovery is the DTR pulse, which the Arduino's auto-reset
        capacitor turns into a hardware reset.

        Permanent prevention is in 99-timelab.rules: ModemManager and
        brltty are told to ignore Arduino devices via udev environment
        variables, so they don't probe the TICC and put it in the
        menu in the first place.  This in-process recovery is defense
        in depth — for a host where the rule isn't deployed yet, or
        for a device that wedged before the rule could take effect.

        Raises TimeoutError after all reset attempts are exhausted.
        """
        assert self.serial is not None

        last_failure = "(none)"
        for attempt in range(self._BOOT_MAX_RESET_ATTEMPTS + 1):
            result = self._wait_for_boot_one_attempt()
            if result == "ok":
                return
            last_failure = result
            if attempt < self._BOOT_MAX_RESET_ATTEMPTS:
                log.warning(
                    "TICC on %s: %s — forcing DTR reset (attempt %d/%d)",
                    self.port, result, attempt + 1, self._BOOT_MAX_RESET_ATTEMPTS,
                )
                self._force_dtr_reset()

        raise TimeoutError(
            f"TICC on {self.port} did not produce timestamps after "
            f"{self._BOOT_MAX_RESET_ATTEMPTS} reset attempts; "
            f"last failure: {last_failure}"
        )

    def _wait_for_boot_one_attempt(self) -> str:
        """One pass through the boot-detect state machine.

        Returns ``"ok"`` on success or a human-readable failure reason
        string suitable for logging and the eventual TimeoutError.

        Each loop iteration checks failure conditions whether or not
        ``readline()`` returned bytes — empty (timed-out) reads still
        advance the silence detector.  ``last_byte_at`` is updated on
        any non-empty ``raw``, including partial reads at timeout, so
        a TICC that's dribbling out a stuck-menu config dump won't be
        misclassified as cold.
        """
        assert self.serial is not None
        try:
            self.serial.reset_input_buffer()
        except (serial.SerialException, OSError):
            pass

        attempt_start = _time.monotonic()
        last_byte_at = attempt_start
        first_header_at: float | None = None
        sentinel_at: float | None = None

        while _time.monotonic() - attempt_start < self._BOOT_PER_ATTEMPT_BUDGET_S:
            try:
                raw = self.serial.readline()
            except (serial.SerialException, OSError) as e:
                # Try a single reopen — if it works, restart this attempt's
                # state machine; if it doesn't, surface the error.
                try:
                    self.serial.close()
                except (serial.SerialException, OSError):
                    pass
                _time.sleep(0.5)
                try:
                    self.serial = self._open_serial()
                except (serial.SerialException, OSError):
                    return f"serial reopen failed after error: {e}"
                attempt_start = _time.monotonic()
                last_byte_at = attempt_start
                first_header_at = None
                sentinel_at = None
                continue

            now = _time.monotonic()

            if raw:
                last_byte_at = now
                line = raw.decode(errors="replace").strip()
                if line:
                    if _LINE_RE.match(line):
                        return "ok"
                    if line.startswith("#"):
                        if first_header_at is None:
                            first_header_at = now
                        if _BOOT_SENTINEL in line and sentinel_at is None:
                            sentinel_at = now

            # Failure-condition checks run every iteration, even after
            # an empty (timed-out) read — that's how we detect cold
            # state and post-deadline staleness during silence.

            if (first_header_at is None
                    and now - last_byte_at > self._BOOT_SILENCE_THRESHOLD_S):
                return f"cold (no bytes for {now - last_byte_at:.1f}s)"

            if (sentinel_at is not None
                    and now - sentinel_at > self._BOOT_POST_SENTINEL_THRESHOLD_S):
                return (f"stuck after sentinel "
                        f"({now - sentinel_at:.1f}s without timestamps)")

            if (first_header_at is not None
                    and sentinel_at is None
                    and now - first_header_at > self._BOOT_POST_HEADER_THRESHOLD_S):
                return (f"stuck in headers "
                        f"({now - first_header_at:.1f}s of headers, no sentinel)")

        return f"budget elapsed ({self._BOOT_PER_ATTEMPT_BUDGET_S:.0f}s)"

    def close(self) -> None:
        if self.serial is not None:
            try:
                self.serial.close()
            finally:
                self.serial = None
                self.booted = False


def _get_shared_port(port: str, baud: int) -> _SharedTiccPort:
    key = (port, baud)
    if key not in _shared_ticc_ports:
        _shared_ticc_ports[key] = _SharedTiccPort(port, baud)
    return _shared_ticc_ports[key]


def _close_all_shared_ports() -> None:
    for port in list(_shared_ticc_ports.values()):
        port.close()


atexit.register(_close_all_shared_ports)


def warm_ticc_port(port: str, baud: int = 115200) -> None:
    """Open the TICC serial port early to absorb any Arduino reboot.

    Must be called BEFORE the GNSS serial reader starts.  Opening the
    TICC port may trigger an Arduino reboot (DTR rising edge) which
    causes USB re-enumeration — if the F9T is on the same USB bus,
    its /dev/ttyACMx path can shift or disconnect.

    After this call, the shared port is open with HUPCL cleared.
    Subsequent opens (bootstrap timestamper, TICC reader thread) go
    through _SharedTiccPort and get the warm path — no reboot, no
    USB disruption.
    """
    shared = _get_shared_port(port, baud)
    shared.acquire(wait_for_boot=True)
    log.info("TICC port warmed: %s (boot=%s)", port,
             "warm" if shared.booted else "cold")


class Ticc:
    """
    Context manager that opens the TICC serial port and yields
    (channel, ref_sec, ref_ps) tuples as edges arrive.

    channel : 'chA' or 'chB'
    ref_sec : int, integer seconds since TICC boot (arbitrary epoch)
    ref_ps  : int, picoseconds 0..999_999_999_999
              11-digit firmware → 10 ps resolution (last digit = 0)
              12-digit firmware →  1 ps resolution
    """

    def __init__(self, port: str, baud: int = 115200,
                 wait_for_boot: bool = True,
                 chb_stall_threshold_s: float = 1.5):
        self.port = port
        self.baud = baud
        self.wait_for_boot = wait_for_boot
        self._ser: serial.Serial | None = None
        self._recv_estimator = TimebaseRelationEstimator(
            min_sigma_s=0.05,
            sigma_scale=4.0,
        )
        self._shared_port = _get_shared_port(self.port, self.baud)
        # chB host_monotonic stall watchdog (CPU-starvation indicator).
        # chB fires at 1 Hz; a delta >> 1 s between consecutive recv_mono
        # stamps means the serial-reader thread didn't run for that
        # long — i.e. CPU stall on the engine host.  Counters are
        # exposed as instance attributes so consumers (e.g. the engine
        # status log) can surface cumulative state.  Warnings are
        # emitted inline so each stall lands in the run log as it
        # happens.  Companion: peppar_fix_engine.run_steady_state
        # tracks main-loop stalls in `skip_stats[n_stalls_gt_*]`.
        self.chb_stall_threshold_s = float(chb_stall_threshold_s)
        self._last_chb_recv_mono: float | None = None
        self.n_chb_stalls: int = 0
        self.max_chb_stall_s: float = 0.0

    def __enter__(self) -> "Ticc":
        # Exclusive access is enforced by the serial port's TIOCEXCL
        # (exclusive=True in _open_serial).  No flock needed — the
        # kernel releases the exclusion automatically when the fd
        # closes, even on crash or SIGKILL.
        self._shared_port.acquire(self.wait_for_boot)
        self._ser = self._shared_port.serial
        return self

    def __exit__(self, *_) -> None:
        self._shared_port.release()

    def __iter__(self):
        """Yield (channel, ref_sec, ref_ps) for each valid edge line."""
        for raw in self._ser:
            line = raw.decode(errors="replace").strip()
            m = _LINE_RE.match(line)
            if not m:
                continue
            ref_sec = int(m.group(1))
            ref_ps  = int(m.group(2).ljust(12, '0'))   # normalise 11→12 digits
            yield m.group(3), ref_sec, ref_ps

    def iter_events(self):
        """Yield TiccEvent records with host receive timestamps."""
        for raw in self._ser:
            recv_mono = _time.monotonic()
            queue_remains = bool(getattr(self._ser, "in_waiting", 0))
            line = raw.decode(errors="replace").strip()
            m = _LINE_RE.match(line)
            if not m:
                continue
            ref_sec = int(m.group(1))
            ref_ps = int(m.group(2).ljust(12, '0'))
            # chB stall watchdog — see __init__ comment.  Fires only on
            # chB (1 Hz cadence) to avoid the chA/chB interleave producing
            # spurious "stalls" from same-second pair ordering.
            if m.group(3) == 'chB':
                self._observe_chb_event(recv_mono)
            base_confidence = estimate_correlation_confidence(
                queue_remains=queue_remains,
                parse_age_s=0.0,
            )
            source_time_s = ref_sec + (ref_ps * 1e-12)
            estimator_sample = self._recv_estimator.update(
                source_time_s,
                recv_mono,
                sample_weight=estimator_sample_weight(
                    queue_remains=queue_remains,
                    base_confidence=base_confidence,
                ),
            )
            yield TiccEvent(
                channel=m.group(3),
                ref_sec=ref_sec,
                ref_ps=ref_ps,
                recv_mono=recv_mono,
                queue_remains=queue_remains,
                parse_age_s=0.0,
                correlation_confidence=max(
                    0.05,
                    min(1.0, base_confidence * estimator_sample["confidence"]),
                ),
                estimator_residual_s=estimator_sample["residual_s"],
            )

    def _observe_chb_event(self, recv_mono: float) -> None:
        """Update chB stall watchdog state and emit a warning if the
        interval since the previous chB exceeds the configured
        threshold.  Pulled out of ``iter_events`` so unit tests can
        exercise it without mocking the serial port.

        Initial call (``_last_chb_recv_mono is None``) bootstraps the
        timer; no warning can fire from the very first chB sample
        because there's no previous to compare against.
        """
        if self._last_chb_recv_mono is not None:
            delta = recv_mono - self._last_chb_recv_mono
            if delta > self.chb_stall_threshold_s:
                self.n_chb_stalls += 1
                if delta > self.max_chb_stall_s:
                    self.max_chb_stall_s = delta
                log.warning(
                    "[TICC] chB stall: %.2fs since last chB "
                    "(threshold %.2fs); cum n=%d max=%.2fs",
                    delta, self.chb_stall_threshold_s,
                    self.n_chb_stalls, self.max_chb_stall_s,
                )
        self._last_chb_recv_mono = recv_mono

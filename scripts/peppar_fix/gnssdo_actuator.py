"""Frequency actuator for the SparkFun GNSSDO+ (SXT-D) oscillator.

Steers the SXT-D's STP3593LF double-oven OCXO by writing its frequency
control word over the ESP32 console, using the ``$W`` external-control
command added by the ``external-oscillator-control`` firmware branch
(see docs/gnssdo-plus-integration.md).  The console is reachable either
over the ESP32's front-panel USB serial port, or over TCP via the
mosaic-T IPS1 backdoor — both carry the same line protocol:

  ``$E,1`` / ``$E,0``   enable / disable external control (we own the OCXO)
  ``$W,<word>``          write the control word (absolute); re-arms the
                         firmware watchdog; reply ``$W,OK,<word>``
  ``$T,<seconds>``       set the fail-safe watchdog timeout
  ``$R``                 status: ``$R,<ext>,<word>,<bias_s>,<state>,<wd>,<since_ms>``

The console also emits 1 Hz CSV telemetry, so replies are found by
scanning incoming lines for the expected ``$`` prefix.

Control-word → frequency mapping (STP3593LF): 1 LSB = 8e-13 fractional
frequency = 8e-4 ppb.  ``ppb_per_code`` is a per-oscillator calibration
(magnitude is a digital datasheet constant; **sign must be confirmed
empirically** — see tools/calibrate_do.py / scripts for the GNSSDO+).
Like DacActuator, ``center_word`` defines the 0-ppb anchor; when not
supplied it defaults to the control word read at setup (start where the
internal loop left it — no frequency jump on hand-over, no magic center).

Fail-safe: the firmware requires a ``$W`` at least every
``watchdog_s`` seconds or it resumes its own discipline.  The servo
calls adjust_frequency_ppb() ~1 Hz, which re-arms it; setup() also sets
the timeout generously so a briefly-stalled servo does not trip it.
"""

import logging
import re
import socket
import time

from peppar_fix.interfaces import FrequencyActuator

log = logging.getLogger(__name__)

# STP3593LF frequency control word: 20-bit unsigned, LSB = 8e-13 fractional
# frequency.  These are the defaults for the SXT-D; other GNSSDO variants
# (SiT5811 39-bit signed, SiT5358 26-bit signed) override via constructor.
_STP3593LF_PPB_PER_CODE = 8e-13 * 1e9   # 8e-4 ppb per LSB
_STP3593LF_WORD_MIN = 0
_STP3593LF_WORD_MAX = (1 << 20) - 1

_RE_W_OK = re.compile(r"^\$W,OK,(-?\d+)")
_RE_R = re.compile(
    r"^\$R,(\d),(-?\d+),([-+0-9.eE]+),(\w+),(\d+),(\d+)")


class GnssdoConsoleError(RuntimeError):
    """The GNSSDO+ console did not answer as expected."""


class _ConsoleLink:
    """Line-oriented link to the GNSSDO+ ESP32 console (serial or TCP).

    Exactly one of ``serial_port`` or ``tcp`` must be given.  ``tcp`` is
    ``(host, port)`` for the mosaic-T IPS1 backdoor.
    """

    def __init__(self, serial_port=None, tcp=None, baud=115200,
                 timeout=2.0):
        if (serial_port is None) == (tcp is None):
            raise ValueError("specify exactly one of serial_port or tcp")
        self._serial_port = serial_port
        self._tcp = tcp
        self._baud = baud
        self._timeout = timeout
        self._ser = None
        self._sock = None
        self._buf = b""

    def open(self):
        if self._serial_port is not None:
            import serial  # pyserial (core engine dep)
            # Opening does not reset the SXT-D's ESP32 (its USB-serial
            # auto-reset lines are not asserted by a plain open).
            self._ser = serial.Serial(self._serial_port, self._baud,
                                      timeout=0.2)
        else:
            host, port = self._tcp
            self._sock = socket.create_connection((host, port),
                                                  timeout=self._timeout)
            self._sock.settimeout(0.2)

    def close(self):
        if self._ser is not None:
            self._ser.close()
            self._ser = None
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def _read_some(self):
        try:
            if self._ser is not None:
                return self._ser.read(4096)
            data = self._sock.recv(4096)
            if data == b"":
                raise GnssdoConsoleError("console connection closed")
            return data
        except socket.timeout:
            return b""

    def _next_line(self, deadline):
        """Return the next complete line (without EOL), or None on timeout."""
        while True:
            nl = self._buf.find(b"\n")
            if nl >= 0:
                line = self._buf[:nl]
                self._buf = self._buf[nl + 1:]
                return line.rstrip(b"\r").decode("ascii", "replace")
            if time.monotonic() >= deadline:
                return None
            self._buf += self._read_some()

    def _write(self, text):
        data = text.encode("ascii")
        if self._ser is not None:
            self._ser.write(data)
            self._ser.flush()
        else:
            self._sock.sendall(data)

    def send(self, line):
        """Fire-and-forget a command line (LF-terminated)."""
        self._write(line + "\n")

    def request(self, line, want, timeout=None):
        """Send ``line`` and return the first reply line matching ``want``.

        ``want`` is a compiled regex.  Raises GnssdoConsoleError on
        timeout.  Interleaved CSV telemetry lines are skipped.
        """
        timeout = self._timeout if timeout is None else timeout
        # Drop any buffered telemetry so we match this command's reply.
        self._buf = b""
        self._write(line + "\n")
        deadline = time.monotonic() + timeout
        while True:
            got = self._next_line(deadline)
            if got is None:
                raise GnssdoConsoleError(
                    "no reply to %r within %.1fs" % (line, timeout))
            m = want.match(got)
            if m:
                return m


class GnssdoActuator(FrequencyActuator):
    """Frequency actuator for the SparkFun GNSSDO+ via the ``$W`` console.

    Args:
        serial_port: ESP32 console serial device (e.g. "/dev/ttyUSB0").
        tcp: ``(host, port)`` for the mosaic-T IPS1 backdoor instead of
            serial.  Exactly one of serial_port / tcp is required.
        ppb_per_code: ppb per control-word LSB.  Default is the STP3593LF
            datasheet value (8e-4); **sign must be confirmed by
            calibration** — positive means higher word → higher frequency.
        center_word: control word defining 0 ppb.  Default None → the
            word read at setup() (anchor to the live starting frequency,
            no jump on hand-over).
        word_min, word_max: usable control-word range (default STP3593LF
            0..2^20-1).
        watchdog_s: firmware fail-safe timeout to request via ``$T`` at
            setup.  Must comfortably exceed the servo period.
        baud: console baud (serial only; 115200).
    """

    def __init__(self, serial_port=None, tcp=None,
                 ppb_per_code=_STP3593LF_PPB_PER_CODE, center_word=None,
                 word_min=_STP3593LF_WORD_MIN, word_max=_STP3593LF_WORD_MAX,
                 watchdog_s=30, baud=115200, _link=None):
        if ppb_per_code == 0:
            raise ValueError("ppb_per_code must be non-zero")
        self._link = _link if _link is not None else _ConsoleLink(
            serial_port=serial_port, tcp=tcp, baud=baud)
        self._ppb_per_code = float(ppb_per_code)
        self._center_word = center_word
        self._word_min = int(word_min)
        self._word_max = int(word_max)
        self._watchdog_s = int(watchdog_s)
        self._current_word = None
        self._current_ppb = 0.0
        self._range_warned = False

    # -- FrequencyActuator interface ------------------------------------

    def setup(self):
        """Open the console, take external control, anchor at the live word.

        Reads the current control word via ``$R``; if ``center_word`` was
        not supplied it becomes the 0-ppb anchor.  Sets the watchdog and
        enables external control (``$E,1``) so the internal loop stops
        touching the OCXO.
        """
        self._link.open()
        ext, word, _bias, state = self._status()
        if self._center_word is None:
            self._center_word = word
        self._current_word = word
        self._current_ppb = (word - self._center_word) * self._ppb_per_code

        # Set the fail-safe watchdog, then take control.
        try:
            self._link.request("$T,%d" % self._watchdog_s,
                               re.compile(r"^\$T,OK"))
        except GnssdoConsoleError:
            log.warning("GNSSDO+ did not ack $T,%d (continuing)",
                        self._watchdog_s)
        self._link.request("$E,1", re.compile(r"^\$E,OK,1"))
        log.info("GNSSDO+ actuator: external control ON via %s; "
                 "anchor word=%d, ppb/code=%.3e, watchdog=%ds, "
                 "internal state was %s",
                 "TCP" if self._link._tcp else self._link._serial_port,
                 self._center_word, self._ppb_per_code,
                 self._watchdog_s, state)

    def teardown(self):
        """Hand the OCXO back to the internal discipline loop (``$E,0``)."""
        try:
            self._link.request("$E,0", re.compile(r"^\$E,OK,0"), timeout=2.0)
            log.info("GNSSDO+ actuator: external control OFF "
                     "(internal discipline resumes)")
        except GnssdoConsoleError as e:
            log.warning("GNSSDO+ teardown: %s (firmware watchdog will "
                        "resume internal discipline)", e)
        finally:
            self._link.close()

    def adjust_frequency_ppb(self, ppb):
        """Write the control word for ``ppb``.  Returns actual ppb applied.

        Clamps to [word_min, word_max]; a saturating command logs once.
        Every call re-arms the firmware watchdog.
        """
        word = self._center_word + round(ppb / self._ppb_per_code)
        clamped = max(self._word_min, min(self._word_max, word))
        if clamped != word and not self._range_warned:
            log.warning("GNSSDO+ command word %d (%.1f ppb) outside "
                        "[%d, %d] — saturating at %d",
                        word, ppb, self._word_min, self._word_max, clamped)
            self._range_warned = True
        word = clamped
        m = self._link.request("$W,%d" % word, _RE_W_OK)
        applied_word = int(m.group(1))
        self._current_word = applied_word
        self._current_ppb = (applied_word - self._center_word) * self._ppb_per_code
        return self._current_ppb

    def read_frequency_ppb(self):
        """Return the current frequency offset (ppb), read from the OCXO."""
        _ext, word, _bias, _state = self._status()
        self._current_word = word
        self._current_ppb = (word - self._center_word) * self._ppb_per_code
        return self._current_ppb

    @property
    def max_adj_ppb(self):
        # Asymmetric about the anchor; report the smaller reachable side.
        anchor = self._center_word if self._center_word is not None \
            else (self._word_min + self._word_max) // 2
        down = (anchor - self._word_min) * abs(self._ppb_per_code)
        up = (self._word_max - anchor) * abs(self._ppb_per_code)
        return min(down, up)

    @property
    def resolution_ppb(self):
        return abs(self._ppb_per_code)

    # -- extras ---------------------------------------------------------

    @property
    def current_word(self):
        return self._current_word

    def _status(self):
        """Return (ext:int, word:int, bias_s:float, state:str) from ``$R``."""
        m = self._link.request("$R", _RE_R)
        return (int(m.group(1)), int(m.group(2)),
                float(m.group(3)), m.group(4))

# SHARED CODE — verbatim copy from testAnt (src/testant/ticc.py).
# Tracked in shared_files.toml. Do not edit here; sync from testAnt.
# See also: Deacon check-shared-code plugin for automated drift detection.
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

Robustness notes:
  - After a power cycle the TICC enters a silent setup mode for ~5-10 s
    before starting timestamp output (like a BIOS waiting for config input).
    The port is open but produces nothing during this window — this is
    normal.  Do not treat it as an error or retry; just wait.  Subsequent
    opens (TICC already in timestamp mode) produce output immediately.
  - reset_input_buffer() on open discards stale OS-buffered lines that
    would otherwise appear as a discontinuity of several seconds.
  - Every line is matched against _LINE_RE before parsing; partial lines,
    corrupt fragments, and buffer artefacts are silently dropped.
"""

from __future__ import annotations

import re

import serial

# Integer part DOT 11-or-12 fractional digits whitespace ch followed by A or B.
# Capture integer and fractional parts separately to avoid float64 precision loss:
# float64 has ~15-16 significant digits total; a 6-digit integer part leaves only
# ~9 decimal digits, losing ps resolution after ~28 hours of TICC uptime.
_LINE_RE = re.compile(r"^(\d+)\.(\d{11,12})\s+(ch[AB])$")


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

    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self._ser: serial.Serial | None = None

    def __enter__(self) -> "Ticc":
        self._ser = serial.Serial(self.port, self.baud, timeout=2.0)
        self._ser.reset_input_buffer()   # discard stale buffered lines
        return self

    def __exit__(self, *_) -> None:
        if self._ser:
            self._ser.close()

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

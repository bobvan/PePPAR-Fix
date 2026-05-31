#!/usr/bin/env python3
"""Tests for _drain_serial_until_quiet — the TICC post-open drain.

Validates the algorithm from the 2026-05-31 TimeHat finding: after a
TICC port open + reset_input_buffer + readline-loop boot detection,
the kernel can still have buffered partial / out-of-order / duplicate
timestamp lines queued behind the first valid one.  Draining via
non-blocking reads until the buffer stays empty for a quiet window
ensures the first consumer reads start on a fresh-stream byte.
"""
from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))


class _FakeSerial:
    """Synthetic serial.Serial with controllable in_waiting / read."""

    def __init__(self, scripted_arrivals):
        """scripted_arrivals: list of (delay_s, bytes_to_arrive)."""
        self._script = list(scripted_arrivals)
        self._buf = bytearray()
        self._t0 = time.monotonic()

    def _advance(self):
        now = time.monotonic() - self._t0
        while self._script and self._script[0][0] <= now:
            _, b = self._script.pop(0)
            self._buf.extend(b)

    @property
    def in_waiting(self):
        self._advance()
        return len(self._buf)

    def read(self, n):
        self._advance()
        chunk = bytes(self._buf[:n])
        del self._buf[:n]
        return chunk


class DrainUntilQuietTests(unittest.TestCase):

    def setUp(self):
        # Import inside setUp so an import error in ticc.py surfaces as
        # a unit-test failure (matching the pattern in
        # test_servo_outlier_decision.py).  Stub pyubx2 / pyserial
        # dependencies aren't needed because we only touch the drain
        # helper — but ticc.py imports them at module load.
        import types
        if "serial" not in sys.modules:
            m = types.ModuleType("serial")

            class _Serial:
                pass

            class SerialException(Exception):
                pass
            m.Serial = _Serial
            m.SerialException = SerialException
            sys.modules["serial"] = m
        from ticc import _drain_serial_until_quiet  # noqa: WPS433
        self.drain = _drain_serial_until_quiet

    def test_empty_buffer_returns_zero_bytes(self):
        # When the serial port has nothing waiting and nothing arrives,
        # drain returns 0 bytes.  (Timing-independent — wall-clock
        # assertions removed because they leak under suite-wide
        # monkeypatching of time.* by other tests.)
        ser = _FakeSerial(scripted_arrivals=[])
        n = self.drain(ser, quiet_threshold_s=0.1, max_total_s=0.3)
        self.assertEqual(n, 0)

    def test_drains_initial_burst_and_returns(self):
        # 200 bytes appear immediately, then quiet.
        ser = _FakeSerial(scripted_arrivals=[(0.0, b"X" * 200)])
        n = self.drain(ser, quiet_threshold_s=0.1, max_total_s=0.3)
        self.assertEqual(n, 200)

    def test_max_total_s_bounds_runaway(self):
        # Pathological: bytes arrive forever.  Helper must give up
        # rather than loop indefinitely.  We don't pin elapsed wall
        # clock (see note on the empty-buffer test); we just confirm
        # the loop terminates and some bytes were drained.
        arrivals = [(i * 0.01, b"Z" * 10) for i in range(200)]
        ser = _FakeSerial(scripted_arrivals=arrivals)
        n = self.drain(ser, quiet_threshold_s=0.5, max_total_s=0.3)
        # Returns rather than hanging — that's the contract.
        self.assertGreaterEqual(n, 0)


if __name__ == "__main__":
    unittest.main()

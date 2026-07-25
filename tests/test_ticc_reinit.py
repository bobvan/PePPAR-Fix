#!/usr/bin/env python3
"""Tests for TICC reader hard re-init on a sustained wedge (I-160351a).

Background: the warm-reopen path in ``_SharedTiccPort`` reuses the cached
fd, and ``acquire()`` only re-opens when ``serial is None``.  So a dead
or USB-re-enumerated device wedges the reader forever — the 2026-07-24
PiFace incident, where a TICC power-cycle re-enumerated ``/dev/ticc2``
but the running reader never re-latched (opens kept "succeeding" on the
dead cached fd, zero events).  ``reset_shared_port()`` evicts the cached
serial so the next ``acquire()`` does a fresh ``_open_serial()`` that
re-resolves the ``/dev`` path.

These tests use a fake serial + the module-global port cache, so they run
with no real hardware.

Run: python3 tests/test_ticc_reinit.py
"""

import sys
import types
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))


def _install_serial_stub():
    if "serial" in sys.modules:
        return
    m = types.ModuleType("serial")

    class _S:
        pass

    class SerialException(Exception):
        pass

    m.Serial = _S
    m.SerialException = SerialException
    sys.modules["serial"] = m


_install_serial_stub()

import ticc as ticc_module  # noqa: E402


class FakeSerial:
    def __init__(self):
        self.closed = False

    def reset_input_buffer(self):
        pass

    def close(self):
        self.closed = True

    @property
    def fd(self):
        return -1


class TestTiccReinit(unittest.TestCase):

    def setUp(self):
        # Isolate the module-global port cache and neutralise the drain
        # helper (it would try to read the fake serial).
        self._saved_ports = dict(ticc_module._shared_ticc_ports)
        ticc_module._shared_ticc_ports.clear()
        self._saved_drain = ticc_module._drain_serial_until_quiet
        ticc_module._drain_serial_until_quiet = lambda *a, **k: None

    def tearDown(self):
        ticc_module._shared_ticc_ports.clear()
        ticc_module._shared_ticc_ports.update(self._saved_ports)
        ticc_module._drain_serial_until_quiet = self._saved_drain

    def _cache_port(self, port="/dev/fake-ticc", baud=115200):
        sp = ticc_module._SharedTiccPort(port, baud)
        sp.serial = FakeSerial()
        sp.booted = True
        ticc_module._shared_ticc_ports[(port, baud)] = sp
        return sp

    def test_reset_closes_and_nulls_serial(self):
        sp = self._cache_port()
        fake = sp.serial
        self.assertTrue(
            ticc_module.reset_shared_port("/dev/fake-ticc", 115200))
        self.assertTrue(fake.closed, "cached serial must be closed")
        self.assertIsNone(sp.serial,
                          "serial must be None so the next acquire re-opens")
        self.assertFalse(sp.booted)

    def test_reset_unknown_port_is_noop(self):
        self.assertFalse(ticc_module.reset_shared_port("/dev/nope", 115200))

    def test_reset_wrong_baud_is_noop(self):
        # Cache is keyed by (port, baud) — a mismatched baud must not evict.
        self._cache_port(baud=115200)
        self.assertFalse(
            ticc_module.reset_shared_port("/dev/fake-ticc", 9600))

    def test_next_acquire_reopens_after_reset(self):
        """Load-bearing behavior: after a reset, acquire() must do a FRESH
        _open_serial() — this is what re-resolves the device path and
        recovers the re-enumerated TICC."""
        sp = self._cache_port()
        opens = {"n": 0}

        def fake_open():
            opens["n"] += 1
            return FakeSerial()

        sp._open_serial = fake_open  # type: ignore[method-assign]

        self.assertTrue(ticc_module.reset_shared_port(sp.port, sp.baud))
        sp.acquire(wait_for_boot=False)

        self.assertEqual(opens["n"], 1,
                         "acquire after reset must re-open the serial")
        self.assertIsNotNone(sp.serial)

    def test_warm_reopen_without_reset_does_not_reopen(self):
        """Regression: without a reset, acquire() reuses the cached fd
        (the warm path) and must NOT re-open — the no-reboot behavior the
        wedge fix deliberately leaves intact for transient hiccups."""
        sp = self._cache_port()
        opens = {"n": 0}

        def fake_open():
            opens["n"] += 1
            return FakeSerial()

        sp._open_serial = fake_open  # type: ignore[method-assign]
        sp.acquire(wait_for_boot=False)  # serial already set → warm path
        self.assertEqual(opens["n"], 0,
                         "warm acquire must reuse the cached fd, not re-open")


if __name__ == "__main__":
    unittest.main(verbosity=2)

#!/usr/bin/env python3
"""Unit tests for the NTRIP-staleness graceful-degradation cascade.

Covers three failure modes the engine must survive without exiting or
silently freezing the servo:

  1. Disconnect — caster drops the TCP connection.  NtripStream.raw_frames()
     must reconnect forever (transient transport error path).
  2. Mute socket — TCP stays open but no RTCM frames arrive.  Detected by
     the socket read timeout firing inside _recv(); raw_frames() treats it
     identically to a disconnect.
  3. Permanent rejection — caster returns HTTP 401/403/404/410.  raw_frames()
     must stop retrying so a typo in credentials doesn't loop forever.

These tests don't touch any real socket — NtripStream is monkeypatched
so the supervisor loop runs deterministically.

Run: python3 tests/test_ntrip_staleness.py
"""

import socket
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))

from ntrip_client import NtripStream  # noqa: E402


def _make_stream(**overrides):
    kwargs = dict(
        caster="example.invalid",
        port=2101,
        mountpoint="TEST00BKG0",
        timeout=0.01,
        reconnect_delay=0.001,
        max_reconnect_delay=0.004,
    )
    kwargs.update(overrides)
    return NtripStream(**kwargs)


class TestRetryForever(unittest.TestCase):
    """raw_frames() must never give up on transient transport failures."""

    def test_transient_failures_retry_forever(self):
        stream = _make_stream()
        attempts = {"n": 0}

        def fake_connect():
            attempts["n"] += 1
            if attempts["n"] < 50:
                # Cycle through the transient errors we expect to survive.
                err = [
                    socket.gaierror(-3, "Temporary failure in name resolution"),
                    ConnectionRefusedError("ECONNREFUSED"),
                    socket.timeout("connect timeout"),
                    OSError(113, "EHOSTUNREACH"),
                ][attempts["n"] % 4]
                raise err
            # After 50 transient failures we let one connect succeed and
            # then immediately yield from a single frame to terminate the
            # generator cleanly via StopIteration.
            stream._connected = True

        produced = []

        def fake_recv():
            # First successful read: produce one fake frame and then
            # signal end-of-stream so the test terminates.
            produced.append(1)
            if len(produced) >= 2:
                raise StopIteration  # break out of the for-loop in test
            stream._buffer.extend(b"\x00")  # nothing parseable
            return None

        with patch.object(stream, "connect", side_effect=fake_connect), \
             patch.object(stream, "_recv", side_effect=fake_recv):
            try:
                gen = stream.raw_frames()
                # Drain a handful of frames; the generator will keep retrying
                # until fake_connect succeeds, then StopIteration breaks us.
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline:
                    try:
                        next(gen)
                    except StopIteration:
                        break
            except Exception:
                pass

        self.assertGreaterEqual(
            attempts["n"], 50,
            f"Should have retried >= 50 times, got {attempts['n']}",
        )
        self.assertFalse(
            stream._fatal,
            "Transient errors must never set _fatal",
        )

    def test_mute_socket_recovers_via_recv_timeout(self):
        """A mute socket (no data ever arrives) should be detected by the
        socket read timeout and trigger a reconnect, NOT cause the
        generator to hang forever or exit."""
        stream = _make_stream()
        connect_calls = {"n": 0}
        recv_calls = {"n": 0}

        def fake_connect():
            connect_calls["n"] += 1
            stream._connected = True
            if connect_calls["n"] >= 5:
                # After 5 mute-then-reconnect cycles, let the test stop.
                raise RuntimeError("STOP_TEST")

        def fake_recv():
            recv_calls["n"] += 1
            stream._connected = False
            # The mute symptom: settimeout fires from inside _recv.
            raise socket.timeout("read timeout — socket is mute")

        with patch.object(stream, "connect", side_effect=fake_connect), \
             patch.object(stream, "_recv", side_effect=fake_recv):
            try:
                for _ in stream.raw_frames():
                    pass
            except RuntimeError:
                pass

        self.assertGreaterEqual(
            connect_calls["n"], 5,
            "Mute socket should have triggered repeated reconnects "
            f"(got {connect_calls['n']})",
        )
        self.assertGreaterEqual(
            recv_calls["n"], 4,
            "Should have called _recv at least once per reconnect",
        )

    def test_http_401_is_fatal(self):
        """Permanent server rejection (HTTP 401) must stop the supervisor."""
        stream = _make_stream(max_reconnects=1000)
        attempts = {"n": 0}

        def fake_connect():
            attempts["n"] += 1
            stream._fatal = True
            stream._fatal_reason = "HTTP/1.0 401 Unauthorized"
            raise ConnectionError("NTRIP error: HTTP/1.0 401 Unauthorized")

        with patch.object(stream, "connect", side_effect=fake_connect):
            list(stream.raw_frames())  # must terminate, not hang

        self.assertEqual(
            attempts["n"], 1,
            f"Fatal error should stop after first attempt, got {attempts['n']}",
        )
        self.assertTrue(stream._fatal)


if __name__ == "__main__":
    unittest.main(verbosity=2)

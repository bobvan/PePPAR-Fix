#!/usr/bin/env python3
"""
ntrip_client.py — NTRIP v2 client for receiving RTCM3 correction streams.

Connects to an NTRIP caster, authenticates, and yields decoded RTCM3 messages
using pyrtcm. Handles reconnection on connection loss.

Supports multiple concurrent mountpoints (e.g., broadcast ephemeris on one
stream and SSR corrections on another).

Usage:
    from ntrip_client import NtripStream

    stream = NtripStream(
        caster='products.igs-ip.net', port=2101,
        mountpoint='CLK93', user='myuser', password='mypass',
    )
    for msg in stream.messages():
        print(msg.identity, msg)

Standalone test:
    python ntrip_client.py --caster products.igs-ip.net --port 2101 \\
        --mountpoint CLK93 --user USER --password PASS --duration 60
"""

import base64
import logging
import socket
import struct
import sys
import time
from datetime import datetime, timezone

log = logging.getLogger(__name__)

# RTCM3 frame constants
RTCM3_PREAMBLE = 0xD3
RTCM3_MAX_LEN = 1023

# CRC-24Q lookup table (RTCM standard)
_CRC24Q_TABLE = None


def _init_crc24q():
    global _CRC24Q_TABLE
    if _CRC24Q_TABLE is not None:
        return
    _CRC24Q_TABLE = [0] * 256
    for i in range(256):
        crc = i << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
        _CRC24Q_TABLE[i] = crc & 0xFFFFFF


def crc24q(data):
    """Compute CRC-24Q checksum for RTCM3 frame."""
    _init_crc24q()
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ _CRC24Q_TABLE[((crc >> 16) ^ b) & 0xFF]) & 0xFFFFFF
    return crc


class NtripStream:
    """A single NTRIP stream connection to one mountpoint."""

    def __init__(self, caster, port, mountpoint, user=None, password=None,
                 timeout=10, reconnect_delay=5, max_reconnects=10):
        self.caster = caster
        self.port = port
        self.mountpoint = mountpoint
        self.user = user
        self.password = password
        self.timeout = timeout
        self.reconnect_delay = reconnect_delay
        self.max_reconnects = max_reconnects
        self._sock = None
        self._buffer = bytearray()
        self._connected = False
        self._reconnect_count = 0
        self._bytes_received = 0
        self._msgs_decoded = 0
        self._connect_time = None

    def connect(self):
        """Establish connection to the NTRIP caster."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(self.timeout)

        log.info(f"Connecting to {self.caster}:{self.port}/{self.mountpoint}")
        self._sock.connect((self.caster, self.port))

        # Build NTRIP v2 request
        request = (
            f"GET /{self.mountpoint} HTTP/1.1\r\n"
            f"Host: {self.caster}:{self.port}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"User-Agent: PePPAR-Fix/0.4\r\n"
        )

        if self.user and self.password:
            credentials = base64.b64encode(
                f"{self.user}:{self.password}".encode()
            ).decode()
            request += f"Authorization: Basic {credentials}\r\n"

        request += "\r\n"
        self._sock.sendall(request.encode())

        # Read response header
        header = b""
        while b"\r\n\r\n" not in header:
            chunk = self._sock.recv(1024)
            if not chunk:
                raise ConnectionError("Connection closed during header")
            header += chunk

        header_str = header.split(b"\r\n\r\n")[0].decode(errors='replace')
        status_line = header_str.split("\r\n")[0]

        if "200" not in status_line and "ICY 200" not in status_line:
            raise ConnectionError(f"NTRIP error: {status_line}")

        # Any data after the header boundary goes into our buffer
        remainder = header.split(b"\r\n\r\n", 1)[1]
        if remainder:
            self._buffer.extend(remainder)

        self._connected = True
        self._connect_time = datetime.now(timezone.utc)
        self._reconnect_count = 0
        log.info(f"Connected to {self.mountpoint}: {status_line}")

    def disconnect(self):
        """Close the connection."""
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
        self._connected = False

    def _recv(self, n=4096):
        """Receive data and append to buffer."""
        data = self._sock.recv(n)
        if not data:
            raise ConnectionError("Connection closed by server")
        self._buffer.extend(data)
        self._bytes_received += len(data)

    def _read_frame(self):
        """Extract one RTCM3 frame from the buffer.

        Returns (message_type, payload_bytes) or None if no complete frame.
        """
        while True:
            # Find preamble
            try:
                idx = self._buffer.index(RTCM3_PREAMBLE)
            except ValueError:
                self._buffer.clear()
                return None

            # Discard bytes before preamble
            if idx > 0:
                del self._buffer[:idx]

            # Need at least 6 bytes: preamble(1) + length(2) + min_payload(0) + CRC(3)
            if len(self._buffer) < 6:
                return None

            # Extract length (10 bits from bytes 1-2)
            length = ((self._buffer[1] & 0x03) << 8) | self._buffer[2]
            if length > RTCM3_MAX_LEN:
                # Invalid length — skip this preamble byte
                del self._buffer[0]
                continue

            frame_len = 3 + length + 3  # header + payload + CRC
            if len(self._buffer) < frame_len:
                return None

            frame = bytes(self._buffer[:frame_len])

            # Verify CRC
            expected_crc = crc24q(frame[:-3])
            actual_crc = (frame[-3] << 16) | (frame[-2] << 8) | frame[-1]
            if expected_crc != actual_crc:
                # Bad CRC — skip this preamble
                del self._buffer[0]
                continue

            # Valid frame — consume it
            del self._buffer[:frame_len]

            # Extract message type (12 bits from payload start)
            if length >= 2:
                msg_type = (frame[3] << 4) | (frame[4] >> 4)
            else:
                msg_type = 0

            return msg_type, frame

    def raw_frames(self):
        """Generator yielding (msg_type, frame_bytes) tuples.

        Handles reconnection automatically. Use this if you want to do
        your own RTCM3 decoding.
        """
        while True:
            if not self._connected:
                if self._reconnect_count >= self.max_reconnects:
                    log.error("Max reconnection attempts reached")
                    return
                try:
                    self.connect()
                except (ConnectionError, OSError, socket.timeout) as e:
                    self._reconnect_count += 1
                    log.warning(f"Connection failed ({self._reconnect_count}/"
                                f"{self.max_reconnects}): {e}")
                    time.sleep(self.reconnect_delay)
                    continue

            try:
                self._recv()
            except (ConnectionError, OSError, socket.timeout) as e:
                log.warning(f"Connection lost: {e}")
                self.disconnect()
                continue

            while True:
                result = self._read_frame()
                if result is None:
                    break
                yield result

    def messages(self):
        """Generator yielding decoded pyrtcm RTCMMessage objects.

        This is the primary API. Each yielded message has attributes
        for all decoded fields (e.g., msg.DF009 for GPS satellite ID).
        """
        try:
            from pyrtcm import RTCMReader
        except ImportError:
            log.error("pyrtcm not installed — pip install pyrtcm")
            return

        for msg_type, frame in self.raw_frames():
            try:
                _, parsed = RTCMReader.parse(frame)
                self._msgs_decoded += 1
                yield parsed
            except Exception as e:
                log.debug(f"Failed to parse message type {msg_type}: {e}")

    def status(self):
        """Return a dict with connection statistics."""
        uptime = None
        if self._connect_time:
            uptime = (datetime.now(timezone.utc) - self._connect_time).total_seconds()
        return {
            'connected': self._connected,
            'mountpoint': self.mountpoint,
            'bytes_received': self._bytes_received,
            'msgs_decoded': self._msgs_decoded,
            'reconnects': self._reconnect_count,
            'uptime_s': uptime,
        }


# ── CLI for testing ─────────────────────────────────────────────────────────── #
def main():
    import argparse

    ap = argparse.ArgumentParser(description="NTRIP client test")
    ap.add_argument("--caster", required=True, help="NTRIP caster hostname")
    ap.add_argument("--port", type=int, default=2101)
    ap.add_argument("--mountpoint", required=True, help="Mountpoint name")
    ap.add_argument("--user", default=None)
    ap.add_argument("--password", default=None)
    ap.add_argument("--duration", type=int, default=60, help="Run time in seconds")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    stream = NtripStream(
        caster=args.caster, port=args.port,
        mountpoint=args.mountpoint,
        user=args.user, password=args.password,
    )

    start = time.time()
    msg_counts = {}
    try:
        for msg in stream.messages():
            identity = str(getattr(msg, 'identity', '?'))
            msg_counts[identity] = msg_counts.get(identity, 0) + 1

            elapsed = time.time() - start
            if elapsed > args.duration:
                break

            # Status every 10 seconds
            if int(elapsed) % 10 == 0 and int(elapsed) > 0:
                status = stream.status()
                print(f"  [{elapsed:.0f}s] {status['bytes_received']} bytes, "
                      f"{status['msgs_decoded']} msgs", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        stream.disconnect()

    elapsed = time.time() - start
    print(f"\n{'='*50}", file=sys.stderr)
    print(f"  Duration: {elapsed:.1f}s", file=sys.stderr)
    print(f"  Bytes: {stream.status()['bytes_received']}", file=sys.stderr)
    print(f"  Messages decoded: {stream.status()['msgs_decoded']}", file=sys.stderr)
    print(f"  Message types:", file=sys.stderr)
    for mt, count in sorted(msg_counts.items()):
        print(f"    {mt}: {count}", file=sys.stderr)


if __name__ == "__main__":
    main()

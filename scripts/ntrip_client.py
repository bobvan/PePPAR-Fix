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
import ssl
import struct
import sys
import time
from datetime import datetime, timezone

from peppar_fix.event_time import (
    estimate_correlation_confidence,
    estimator_sample_weight,
)
from peppar_fix.timebase_estimator import TimebaseRelationEstimator

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
                 timeout=10, reconnect_delay=5, max_reconnect_delay=60,
                 max_reconnects=None, tls=None, http10=False):
        self.caster = caster
        # http10: send a minimal NTRIP-v1 / HTTP/1.0 request instead of the
        # HTTP/1.1 default.  Needed for casters whose parser only accepts v1 —
        # notably RTKLIB str2str's `ntripc` local re-caster (Box 1/2), which
        # ignores HTTP/1.1 requests.  See docs/local-caster-onocoy-plan.md.
        self.http10 = http10
        self.port = port
        self.mountpoint = mountpoint
        self.user = user
        self.password = password
        self.timeout = timeout
        self.reconnect_delay = reconnect_delay
        self.max_reconnect_delay = max_reconnect_delay
        # max_reconnects retained for tests/CLI compat; production use is None
        # (retry forever with exponential backoff).  See raw_frames().
        self.max_reconnects = max_reconnects
        # Mark fatal once a permanent server-side rejection has been seen
        # (HTTP 401/403/404/410).  Transient transport failures
        # (DNS, ECONNREFUSED, timeouts, TLS handshake errors) never set this.
        self._fatal = False
        self._fatal_reason = None
        self.tls = tls if tls is not None else (port == 443)
        self._sock = None
        self._buffer = bytearray()
        self._chunked = False
        self._chunk_buf = bytearray()
        self._connected = False
        self._reconnect_count = 0
        self._bytes_received = 0
        self._msgs_decoded = 0
        self._connect_time = None
        self._epoch_estimator = TimebaseRelationEstimator(
            min_sigma_s=1.0,
            sigma_scale=4.0,
        )

    def connect(self):
        """Establish connection to the NTRIP caster."""
        raw_sock = socket.create_connection(
            (self.caster, self.port), timeout=self.timeout)
        if self.tls:
            ctx = ssl.create_default_context()
            self._sock = ctx.wrap_socket(raw_sock, server_hostname=self.caster)
        else:
            self._sock = raw_sock
        self._sock.settimeout(self.timeout)

        log.info(f"Connecting to {self.caster}:{self.port}/{self.mountpoint}"
                 f"{' (TLS)' if self.tls else ''}")

        # Build NTRIP request.  Default is HTTP/1.1 — some casters (notably
        # products.igs-ip.net) don't stream data with HTTP/1.0.  We handle
        # chunked transfer encoding in _recv() if the server uses it.  When
        # http10 is set, send a minimal NTRIP-v1 request instead (no Host, no
        # Ntrip-Version) for v1-only casters like str2str's `ntripc`; the
        # response is "ICY 200 OK" (accepted by the status check below) and is
        # never chunked.
        if self.http10:
            request = (
                f"GET /{self.mountpoint} HTTP/1.0\r\n"
                f"User-Agent: NTRIP PePPAR-Fix/0.5\r\n"
            )
        else:
            request = (
                f"GET /{self.mountpoint} HTTP/1.1\r\n"
                f"Host: {self.caster}:{self.port}\r\n"
                f"Ntrip-Version: Ntrip/2.0\r\n"
                f"User-Agent: NTRIP PePPAR-Fix/0.5\r\n"
            )

        if self.user and self.password:
            credentials = base64.b64encode(
                f"{self.user}:{self.password}".encode()
            ).decode()
            request += f"Authorization: Basic {credentials}\r\n"

        request += "\r\n"
        self._sock.sendall(request.encode())

        # Read response header.  NTRIP v2 (HTTP/1.1) ends the header block
        # with a blank line (\r\n\r\n).  NTRIP v1 (http10, e.g. str2str's
        # ntripc) replies "ICY 200 OK\r\n" and then streams RTCM immediately —
        # there is NO blank line — so we must stop after the first \r\n, or we
        # would read RTCM bytes into the "header" and mis-frame the stream.
        _sep = b"\r\n" if self.http10 else b"\r\n\r\n"
        header = b""
        while _sep not in header:
            chunk = self._sock.recv(1024)
            if not chunk:
                raise ConnectionError("Connection closed during header")
            header += chunk

        header_str = header.split(_sep)[0].decode(errors='replace')
        status_line = header_str.split("\r\n")[0]

        if "200" not in status_line and "ICY 200" not in status_line:
            # Permanent rejections: bad credentials (401/403).
            # Distinguished from transport failures so the supervisor loop
            # in raw_frames() knows to stop retrying.
            #
            # NOTE: 404 and 410 are NOT treated as permanent.  NTRIP
            # casters go through maintenance restarts where mounts
            # disappear briefly and return 404.  On 2026-04-08 the
            # Australian caster returned 404 for SSRA00BKG0 during a
            # maintenance window; treating it as fatal killed Carrier
            # for 93% of TimeHat's overnight run.  404/410 are retried
            # with the normal exponential backoff.
            for code in ("401", "403"):
                if code in status_line:
                    self._fatal = True
                    self._fatal_reason = status_line
                    break
            raise ConnectionError(f"NTRIP error: {status_line}")

        # Detect chunked transfer encoding
        self._chunked = False
        self._chunk_buf = bytearray()
        for line in header_str.split("\r\n")[1:]:
            if line.lower().startswith("transfer-encoding:"):
                if "chunked" in line.lower():
                    self._chunked = True
                    log.info(f"  Transfer-Encoding: chunked — decoding chunks")
                break

        # Any data after the header boundary goes into our buffer
        remainder = header.split(_sep, 1)[1]
        if remainder:
            if self._chunked:
                self._dechunk(remainder)
            else:
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
        self._chunked = False
        self._chunk_buf.clear()

    close = disconnect

    def _dechunk(self, data):
        """Decode HTTP chunked transfer encoding into self._buffer.

        Accumulates raw socket bytes in self._chunk_buf, extracts complete
        chunks, and appends the decoded payload to self._buffer.  A zero-
        length chunk signals end-of-stream (raises ConnectionError so the
        reconnect loop handles it like any other close).
        """
        self._chunk_buf.extend(data)
        while True:
            # Look for chunk-size line terminated by \r\n
            sep = self._chunk_buf.find(b"\r\n")
            if sep < 0:
                break
            size_str = self._chunk_buf[:sep].decode(errors="replace").strip()
            # Chunk extensions (;key=val) are allowed but unused
            size_str = size_str.split(";", 1)[0].strip()
            if not size_str:
                # Empty line between chunks — skip
                del self._chunk_buf[:sep + 2]
                continue
            try:
                chunk_len = int(size_str, 16)
            except ValueError:
                # Not a valid chunk header — discard and resync
                del self._chunk_buf[:sep + 2]
                continue
            if chunk_len == 0:
                # Terminal chunk — server closed the stream
                raise ConnectionError("Chunked stream ended (zero-length chunk)")
            # Need chunk_len bytes + trailing \r\n after the chunk data
            total = sep + 2 + chunk_len + 2
            if len(self._chunk_buf) < total:
                break  # wait for more data
            payload = self._chunk_buf[sep + 2:sep + 2 + chunk_len]
            self._buffer.extend(payload)
            del self._chunk_buf[:total]

    def _recv(self, n=4096):
        """Receive data and append to buffer."""
        data = self._sock.recv(n)
        if not data:
            raise ConnectionError("Connection closed by server")
        if self._chunked:
            self._dechunk(data)
        else:
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

        Reconnects forever on transport failure with exponential backoff
        (reconnect_delay → max_reconnect_delay).  The only conditions that
        cause this generator to return are:

          - The caster returned a permanent HTTP rejection (401/403/404/410).
            self._fatal is set inside connect() and we stop retrying.
          - max_reconnects is set to a finite integer (legacy / test mode)
            and that budget is exhausted.

        Transient failures (DNS resolution failures, ECONNREFUSED, timeouts,
        TLS handshake errors, EHOSTUNREACH, ECONNRESET) never give up — the
        supervisor loop just keeps retrying so a transient NTRIP outage shows
        up to the consumer as growing correction age, not as a dead thread.

        Note: this is the supervisor loop in the user's two-thread mental
        model.  The connection-maintenance happens here; the consumer
        (ntrip_reader in realtime_ppp.py) just iterates and lets correction
        freshness propagate downstream.
        """
        backoff = self.reconnect_delay
        while True:
            if not self._connected:
                if self._fatal:
                    log.error(
                        "NTRIP %s: permanent rejection (%s) — not retrying",
                        self.mountpoint, self._fatal_reason)
                    return
                if (self.max_reconnects is not None
                        and self._reconnect_count >= self.max_reconnects):
                    log.error("Max reconnection attempts reached")
                    return
                try:
                    self.connect()
                    backoff = self.reconnect_delay  # reset on success
                except (ConnectionError, OSError, socket.timeout, ssl.SSLError) as e:
                    self._reconnect_count += 1
                    if self._fatal:
                        # connect() classified this as permanent (HTTP 4xx)
                        continue  # next iteration will hit the _fatal branch
                    log.warning(
                        "NTRIP %s reconnect %d failed: %s — retry in %ds",
                        self.mountpoint, self._reconnect_count, e, backoff)
                    time.sleep(backoff)
                    backoff = min(backoff * 2, self.max_reconnect_delay)
                    continue

            try:
                self._recv()
            except (ConnectionError, OSError, socket.timeout, ssl.SSLError) as e:
                log.warning("NTRIP %s connection lost: %s", self.mountpoint, e)
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
        for parsed, _meta in self.messages_with_metadata():
            yield parsed

    def messages_with_metadata(self):
        """Generator yielding decoded messages with host timing metadata."""
        try:
            from pyrtcm import RTCMReader
        except ImportError:
            log.error("pyrtcm not installed — pip install pyrtcm")
            return

        for msg_type, frame in self.raw_frames():
            parse_mono = time.monotonic()
            queue_remains = bool(self._buffer)
            try:
                result = RTCMReader.parse(frame)
                # pyrtcm >= 1.1: returns RTCMMessage directly
                # pyrtcm < 1.1: returns (raw, parsed) tuple
                if isinstance(result, tuple):
                    parsed = result[1]
                else:
                    parsed = result
                base_confidence = estimate_correlation_confidence(
                    queue_remains=queue_remains,
                    parse_age_s=0.0,
                )
                confidence = base_confidence
                estimator_residual_s = None
                source_time_s = self._extract_source_time_s(parsed)
                if source_time_s is not None:
                    estimator_sample = self._epoch_estimator.update(
                        source_time_s,
                        parse_mono,
                        sample_weight=estimator_sample_weight(
                            queue_remains=queue_remains,
                            base_confidence=base_confidence,
                        ),
                    )
                    confidence = max(
                        0.05,
                        min(1.0, base_confidence * estimator_sample["confidence"]),
                    )
                    estimator_residual_s = estimator_sample["residual_s"]
                self._msgs_decoded += 1
                yield parsed, {
                    "recv_mono": parse_mono,
                    "queue_remains": queue_remains,
                    "parse_age_s": 0.0,
                    "correlation_confidence": confidence,
                    "estimator_residual_s": estimator_residual_s,
                    # raw RTCM3 frame bytes + the canonical recv_mono above —
                    # for the pos_replay raw-capture tap (manifest §2).  The
                    # exact frame the engine correlated, not a re-serialization.
                    "raw": frame,
                }
            except Exception as e:
                log.debug(f"Failed to parse message type {msg_type}: {e}")

    def _extract_source_time_s(self, parsed):
        """Return absolute GPS-like seconds for RTCM messages with usable epochs.

        Only some RTCM families carry a message epoch that is meaningfully
        related to receive latency. Broadcast ephemeris toe/toc are model
        reference epochs, not transport timestamps, so they are intentionally
        excluded here.
        """
        identity = str(getattr(parsed, "identity", ""))
        if not (identity.startswith("4076_") or identity.isdigit()):
            return None

        epoch_s = getattr(parsed, "IDF003", None)
        if epoch_s is None:
            epoch_s = getattr(parsed, "DF385", None)
        if epoch_s is None:
            return None

        now_utc = datetime.now(timezone.utc)
        gps_now = now_utc.timestamp() + 18.0
        week_s = 604800.0
        current_week = int(gps_now // week_s)
        candidate = current_week * week_s + float(epoch_s)
        while candidate - gps_now > (week_s / 2.0):
            candidate -= week_s
        while gps_now - candidate > (week_s / 2.0):
            candidate += week_s
        return candidate

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

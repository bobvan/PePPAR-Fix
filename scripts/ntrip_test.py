#!/usr/bin/env python3
"""
NTRIP client for receiving RTCM correction streams from IGS RTS casters.

Connects to an NTRIP caster, authenticates, and streams RTCM data from a
specified mountpoint. Supports both HTTP and HTTPS casters.

Usage:
    python3 ntrip_client.py --caster igs-ip.net --port 2101 --mount ALGO00CAN0 \
        --user USERNAME --password PASSWORD

    # With config file:
    python3 ntrip_client.py --config ntrip.conf

    # Dump raw RTCM to file:
    python3 ntrip_client.py --config ntrip.conf --output raw.rtcm

    # List available mountpoints:
    python3 ntrip_client.py --caster igs-ip.net --port 2101 --sourcetable
"""

import argparse
import base64
import configparser
import os
import signal
import socket
import ssl
import sys
import time
from pathlib import Path


DEFAULT_TIMEOUT = 10
RECONNECT_DELAY = 5
MAX_RECONNECT_DELAY = 60
BUFFER_SIZE = 4096


class NtripClient:
    """NTRIP v1/v2 client for receiving RTCM correction streams."""

    def __init__(self, caster: str, port: int, mount: str,
                 user: str = "", password: str = "",
                 tls: bool = False, timeout: int = DEFAULT_TIMEOUT):
        self.caster = caster
        self.port = port
        self.mount = mount
        self.user = user
        self.password = password
        self.tls = tls or port == 443
        self.timeout = timeout
        self._sock = None
        self._running = False

        # Stats
        self.bytes_received = 0
        self.connect_time = None
        self.last_data_time = None

    def _connect(self) -> socket.socket:
        """Open TCP connection to caster, optionally wrapping in TLS."""
        sock = socket.create_connection((self.caster, self.port),
                                        timeout=self.timeout)
        if self.tls:
            ctx = ssl.create_default_context()
            sock = ctx.wrap_socket(sock, server_hostname=self.caster)
        return sock

    def _build_request(self, path: str) -> bytes:
        """Build an NTRIP GET request with Basic auth."""
        lines = [
            f"GET /{path} HTTP/1.1",
            f"Host: {self.caster}:{self.port}",
            "Ntrip-Version: Ntrip/2.0",
            "User-Agent: PePPAR-NTRIP/0.1",
        ]
        if self.user:
            cred = base64.b64encode(
                f"{self.user}:{self.password}".encode()
            ).decode()
            lines.append(f"Authorization: Basic {cred}")
        lines.append("Connection: close")
        lines.append("")
        lines.append("")
        return "\r\n".join(lines).encode()

    def _parse_response_header(self, sock: socket.socket) -> tuple[int, dict]:
        """Read HTTP response header. Returns (status_code, headers)."""
        buf = b""
        while b"\r\n\r\n" not in buf:
            chunk = sock.recv(1)
            if not chunk:
                raise ConnectionError("Connection closed during header read")
            buf += chunk
            if len(buf) > 8192:
                raise ConnectionError("Response header too large")

        header_data, _ = buf.split(b"\r\n\r\n", 1)
        lines = header_data.decode("ascii", errors="replace").split("\r\n")

        # Parse status line: "HTTP/1.1 200 OK" or "ICY 200 OK"
        status_line = lines[0]
        parts = status_line.split(None, 2)
        if len(parts) < 2:
            raise ConnectionError(f"Malformed status line: {status_line}")
        status_code = int(parts[1])

        headers = {}
        for line in lines[1:]:
            if ":" in line:
                key, val = line.split(":", 1)
                headers[key.strip().lower()] = val.strip()

        return status_code, headers

    def get_sourcetable(self) -> list[dict]:
        """Fetch and parse the caster's sourcetable."""
        sock = self._connect()
        try:
            sock.sendall(self._build_request(""))
            status, _headers = self._parse_response_header(sock)

            if status != 200:
                raise ConnectionError(f"Sourcetable request failed: HTTP {status}")

            body = b""
            while True:
                chunk = sock.recv(BUFFER_SIZE)
                if not chunk:
                    break
                body += chunk
                if b"ENDSOURCETABLE" in body:
                    break

            entries = []
            for line in body.decode("ascii", errors="replace").splitlines():
                if line.startswith("STR;"):
                    fields = line.split(";")
                    if len(fields) >= 10:
                        entries.append({
                            "mount": fields[1],
                            "name": fields[2],
                            "format": fields[3],
                            "messages": fields[4],
                            "carrier": fields[5],
                            "system": fields[6],
                            "network": fields[7],
                            "country": fields[8],
                            "lat": fields[9],
                            "lon": fields[10] if len(fields) > 10 else "",
                        })
            return entries
        finally:
            sock.close()

    def connect(self) -> None:
        """Connect to caster and request the mountpoint stream."""
        self._sock = self._connect()
        self._sock.sendall(self._build_request(self.mount))
        status, headers = self._parse_response_header(self._sock)

        if status == 200:
            self.connect_time = time.monotonic()
            self.bytes_received = 0
            return

        self._sock.close()
        self._sock = None

        if status == 401:
            raise PermissionError(
                f"Authentication failed (401). Check credentials.")
        elif status == 403:
            raise PermissionError(
                f"Forbidden (403). Account not authorized for {self.mount}.")
        elif status == 404:
            raise ConnectionError(
                f"Mountpoint '{self.mount}' not found (404).")
        else:
            raise ConnectionError(
                f"Unexpected response: HTTP {status}")

    def read(self) -> bytes:
        """Read a chunk of RTCM data from the stream. Blocks until data."""
        if not self._sock:
            raise ConnectionError("Not connected")
        data = self._sock.recv(BUFFER_SIZE)
        if not data:
            raise ConnectionError("Stream ended (connection closed by caster)")
        self.bytes_received += len(data)
        self.last_data_time = time.monotonic()
        return data

    def close(self) -> None:
        """Close the connection."""
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def stream(self, output=None, callback=None, stats_interval: int = 10):
        """
        Stream RTCM data continuously with auto-reconnect.

        Args:
            output: File-like object to write raw RTCM data to (or None).
            callback: Called with each chunk of bytes received.
            stats_interval: Seconds between status line prints.
        """
        self._running = True
        reconnect_delay = RECONNECT_DELAY
        last_stats = time.monotonic()

        while self._running:
            try:
                print(f"Connecting to {self.caster}:{self.port}/{self.mount}...",
                      flush=True)
                self.connect()
                print(f"Connected. Streaming RTCM data.", flush=True)
                reconnect_delay = RECONNECT_DELAY

                while self._running:
                    data = self.read()

                    if output:
                        output.write(data)
                        output.flush()

                    if callback:
                        callback(data)

                    now = time.monotonic()
                    if now - last_stats >= stats_interval:
                        elapsed = now - self.connect_time
                        rate = self.bytes_received / elapsed if elapsed > 0 else 0
                        print(f"  [{elapsed:.0f}s] {self.bytes_received:,} bytes "
                              f"({rate:.0f} B/s)", flush=True)
                        last_stats = now

            except PermissionError as e:
                print(f"Auth error: {e}", file=sys.stderr)
                self._running = False
                break
            except (ConnectionError, OSError, socket.timeout) as e:
                self.close()
                if not self._running:
                    break
                print(f"Connection lost: {e}", file=sys.stderr)
                print(f"Reconnecting in {reconnect_delay}s...", flush=True)
                time.sleep(reconnect_delay)
                reconnect_delay = min(reconnect_delay * 2, MAX_RECONNECT_DELAY)

        self.close()
        print("Stream stopped.", flush=True)

    def stop(self):
        """Signal the stream loop to stop."""
        self._running = False
        self.close()


def load_config(path: str) -> dict:
    """Load NTRIP config from INI file."""
    config = configparser.ConfigParser()
    config.read(path)
    s = config["ntrip"]
    return {
        "caster": s.get("caster"),
        "port": s.getint("port", 2101),
        "mount": s.get("mount"),
        "user": s.get("user", ""),
        "password": s.get("password", ""),
        "tls": s.getboolean("tls", False),
    }


def print_sourcetable(entries: list[dict]):
    """Pretty-print sourcetable entries."""
    if not entries:
        print("No streams found.")
        return
    print(f"{'Mountpoint':<25} {'System':<25} {'Format':<12} {'Network':<8} "
          f"{'Country':<5}")
    print("-" * 80)
    for e in entries:
        print(f"{e['mount']:<25} {e['system']:<25} {e['format']:<12} "
              f"{e['network']:<8} {e['country']:<5}")


def rtcm_frame_counter(data: bytes, state: dict):
    """Callback that counts RTCM3 frames in the stream."""
    buf = state.get("buf", b"") + data
    count = state.get("count", 0)
    msg_types = state.get("msg_types", {})

    while len(buf) >= 6:
        # RTCM3 frame: 0xD3 + 6-bit reserved + 10-bit length
        if buf[0] != 0xD3:
            buf = buf[1:]
            continue

        msg_len = ((buf[1] & 0x03) << 8) | buf[2]
        frame_len = msg_len + 6  # header(3) + payload + CRC(3)

        if len(buf) < frame_len:
            break  # need more data

        # Extract message type (first 12 bits of payload)
        if msg_len >= 2:
            msg_type = (buf[3] << 4) | (buf[4] >> 4)
            msg_types[msg_type] = msg_types.get(msg_type, 0) + 1

        count += 1
        buf = buf[frame_len:]

    state["buf"] = buf
    state["count"] = count
    state["msg_types"] = msg_types

    # Print summary every 50 frames
    if count > 0 and count % 50 == 0:
        types_str = ", ".join(f"{t}:{c}" for t, c in sorted(msg_types.items()))
        print(f"  RTCM frames: {count} | Types: {types_str}", flush=True)


def main():
    parser = argparse.ArgumentParser(
        description="NTRIP client for RTCM correction streams"
    )
    parser.add_argument("--config", help="Config file path")
    parser.add_argument("--caster", help="Caster hostname")
    parser.add_argument("--port", type=int, default=2101, help="Caster port")
    parser.add_argument("--mount", help="Mountpoint name")
    parser.add_argument("--user", default="", help="Username")
    parser.add_argument("--password", default="", help="Password")
    parser.add_argument("--tls", action="store_true", help="Use TLS/HTTPS")
    parser.add_argument("--output", "-o", help="Write raw RTCM to file")
    parser.add_argument("--sourcetable", action="store_true",
                        help="List available mountpoints and exit")
    parser.add_argument("--duration", type=int, default=0,
                        help="Stop after N seconds (0=indefinite)")
    parser.add_argument("--stats-interval", type=int, default=10,
                        help="Seconds between stats output")
    args = parser.parse_args()

    # Load config file, then override with CLI args
    conf = {}
    if args.config:
        conf = load_config(args.config)
    for key in ("caster", "port", "mount", "user", "password", "tls"):
        cli_val = getattr(args, key)
        if cli_val and (key not in conf or cli_val != parser.get_default(key)):
            conf[key] = cli_val

    if not conf.get("caster"):
        parser.error("--caster or config file with caster is required")

    client = NtripClient(
        caster=conf["caster"],
        port=conf.get("port", 2101),
        mount=conf.get("mount", ""),
        user=conf.get("user", ""),
        password=conf.get("password", ""),
        tls=conf.get("tls", False),
    )

    # Handle Ctrl-C
    def handle_signal(sig, frame):
        client.stop()
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    if args.sourcetable:
        entries = client.get_sourcetable()
        print_sourcetable(entries)
        return

    if not conf.get("mount"):
        parser.error("--mount or config file with mount is required")

    # Set up output file
    outfile = None
    if args.output:
        outfile = open(args.output, "wb")

    # Set up RTCM frame counter
    frame_state = {}

    def on_data(data: bytes):
        rtcm_frame_counter(data, frame_state)

    # Duration timer
    if args.duration > 0:
        def stop_timer(sig, frame):
            client.stop()
        signal.signal(signal.SIGALRM, stop_timer)
        signal.alarm(args.duration)

    try:
        client.stream(output=outfile, callback=on_data,
                      stats_interval=args.stats_interval)
    finally:
        if outfile:
            outfile.close()
        # Print final stats
        if frame_state.get("count"):
            types_str = ", ".join(
                f"{t}:{c}" for t, c in sorted(frame_state["msg_types"].items())
            )
            print(f"\nFinal: {frame_state['count']} RTCM frames")
            print(f"Message types: {types_str}")
        if client.bytes_received:
            print(f"Total bytes: {client.bytes_received:,}")


if __name__ == "__main__":
    main()

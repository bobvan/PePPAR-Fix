#!/usr/bin/env python3
"""
ntrip_caster.py — Minimal NTRIP v1 caster for PePPAR Fix peer bootstrap.

Serves raw GNSS observations as RTCM MSM4 corrections over TCP, enabling
nearby PePPAR Fix nodes to bootstrap their position from a peer that has
already converged.

Architecture:
    F9T serial ──→ UBX parser ──→ RXM-RAWX observations
                                        ↓
                                  rtcm_encoder.py (MSM4 + 1005)
                                        ↓
                                  NTRIP TCP server
                                        ↓
                                  Client 1, Client 2, ...

Usage:
    # Standalone:
    python ntrip_caster.py --serial /dev/gnss-bot --bind :2102 \\
        --position data/position.json

    # The unified CLI will integrate this as --caster :2102
"""

import logging
import queue
import select
import socket
import struct
import sys
import threading
import time
from datetime import datetime, timezone, timedelta

from rtcm_encoder import encode_epoch, encode_1005
from peppar_fix import load_position

log = logging.getLogger(__name__)


class NtripCasterServer:
    """NTRIP v1 TCP server that broadcasts RTCM data to connected clients.

    Accepts HTTP-style NTRIP GET requests and streams RTCM frames.
    Supports multiple concurrent clients.
    """

    MOUNTPOINT = "PEPPAR"

    def __init__(self, bind_addr="", bind_port=2102, station_id=0):
        self.bind_addr = bind_addr
        self.bind_port = bind_port
        self.station_id = station_id
        self._server_sock = None
        self._clients = []  # list of (socket, addr) tuples
        self._clients_lock = threading.Lock()
        self._running = False
        self._stats = {
            'clients_total': 0,
            'clients_active': 0,
            'bytes_sent': 0,
            'frames_sent': 0,
            'epochs_encoded': 0,
        }

    def start(self):
        """Start the TCP listener in a background thread."""
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((self.bind_addr, self.bind_port))
        self._server_sock.listen(5)
        self._server_sock.settimeout(1.0)
        self._running = True

        self._accept_thread = threading.Thread(
            target=self._accept_loop, daemon=True, name="ntrip-accept")
        self._accept_thread.start()

        log.info(f"NTRIP caster listening on {self.bind_addr or '*'}:{self.bind_port}"
                 f" (mountpoint: /{self.MOUNTPOINT})")

    def stop(self):
        """Stop the server and disconnect all clients."""
        self._running = False
        with self._clients_lock:
            for sock, addr in self._clients:
                try:
                    sock.close()
                except OSError:
                    pass
            self._clients.clear()
        if self._server_sock:
            try:
                self._server_sock.close()
            except OSError:
                pass

    def _accept_loop(self):
        """Accept new NTRIP client connections."""
        while self._running:
            try:
                client_sock, addr = self._server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    log.error("Accept error", exc_info=True)
                break

            # Handle the NTRIP handshake in a short-lived thread
            threading.Thread(
                target=self._handle_client,
                args=(client_sock, addr),
                daemon=True,
                name=f"ntrip-client-{addr[0]}:{addr[1]}"
            ).start()

    def _handle_client(self, sock, addr):
        """Handle NTRIP client handshake (HTTP GET)."""
        try:
            sock.settimeout(10.0)
            # Read the HTTP request
            request = b""
            while b"\r\n\r\n" not in request and len(request) < 4096:
                chunk = sock.recv(1024)
                if not chunk:
                    sock.close()
                    return
                request += chunk

            request_str = request.decode(errors='replace')
            lines = request_str.split('\r\n')
            request_line = lines[0] if lines else ""

            log.info(f"NTRIP client {addr}: {request_line}")

            # Check for valid mountpoint request
            # Accept: GET /PEPPAR HTTP/1.0 or GET /PEPPAR HTTP/1.1
            if f"GET /{self.MOUNTPOINT}" in request_line:
                # Send NTRIP v1 response (ICY 200 OK)
                response = (
                    "ICY 200 OK\r\n"
                    "Content-Type: gnss/data\r\n"
                    "Cache-Control: no-cache\r\n"
                    "\r\n"
                )
                sock.sendall(response.encode())
                sock.settimeout(None)  # No timeout for streaming

                with self._clients_lock:
                    self._clients.append((sock, addr))
                    self._stats['clients_total'] += 1
                    self._stats['clients_active'] = len(self._clients)

                log.info(f"NTRIP client {addr} connected to /{self.MOUNTPOINT} "
                         f"({self._stats['clients_active']} active)")

            elif "GET /" in request_line and f"/{self.MOUNTPOINT}" not in request_line:
                # Sourcetable request (GET / or GET /anything_else)
                sourcetable = self._build_sourcetable()
                response = (
                    "SOURCETABLE 200 OK\r\n"
                    "Content-Type: text/plain\r\n"
                    f"Content-Length: {len(sourcetable)}\r\n"
                    "\r\n"
                ) + sourcetable
                sock.sendall(response.encode())
                sock.close()
            else:
                # Unknown request
                sock.sendall(b"HTTP/1.0 400 Bad Request\r\n\r\n")
                sock.close()

        except (OSError, ConnectionError) as e:
            log.debug(f"Client handshake error {addr}: {e}")
            try:
                sock.close()
            except OSError:
                pass

    def _build_sourcetable(self):
        """Build NTRIP sourcetable response."""
        # STR record format (simplified)
        # STR;mountpoint;identifier;format;format-details;carrier;nav-system;
        # network;country;lat;lon;nmea;solution;generator;compression;auth;fee;bitrate
        return (
            f"STR;{self.MOUNTPOINT};PePPAR Fix;RTCM 3.3;1005(1),1074(1),1094(1),1124(1);"
            f"2;GPS+GAL+BDS;PePPAR;USA;0.00;0.00;0;0;PePPAR-Fix;none;N;N;0;\r\n"
            f"ENDSOURCETABLE\r\n"
        )

    def broadcast(self, data):
        """Send data to all connected clients. Remove dead ones."""
        if not data:
            return

        dead = []
        with self._clients_lock:
            for i, (sock, addr) in enumerate(self._clients):
                try:
                    sock.sendall(data)
                    self._stats['bytes_sent'] += len(data)
                except (OSError, ConnectionError, BrokenPipeError):
                    log.info(f"NTRIP client {addr} disconnected")
                    dead.append(i)
                    try:
                        sock.close()
                    except OSError:
                        pass

            # Remove dead clients (reverse order to preserve indices)
            for i in reversed(dead):
                del self._clients[i]

            self._stats['clients_active'] = len(self._clients)

    def broadcast_epoch(self, raw_observations, gps_time, ref_ecef=None):
        """Encode and broadcast one epoch of observations.

        Args:
            raw_observations: list of raw obs dicts (from UBX RAWX parser)
            gps_time: datetime of the epoch
            ref_ecef: [x,y,z] ECEF reference position (for 1005), or None
        """
        with self._clients_lock:
            if not self._clients:
                return  # No clients, skip encoding

        frames = encode_epoch(raw_observations, gps_time,
                              station_id=self.station_id)

        if not frames:
            return

        self._stats['epochs_encoded'] += 1

        # Prepend 1005 (reference position) every epoch if available
        data = bytearray()
        if ref_ecef is not None:
            data.extend(encode_1005(ref_ecef, station_id=self.station_id))
            self._stats['frames_sent'] += 1

        for frame in frames:
            data.extend(frame)
            self._stats['frames_sent'] += 1

        self.broadcast(bytes(data))

    def status(self):
        """Return caster statistics."""
        return dict(self._stats)


# ── UBX RAWX reader for caster mode ──────────────────────────────────────── #

def rawx_to_caster_obs(parsed):
    """Convert a pyubx2 RXM-RAWX message to raw observation dicts for the encoder.

    Returns (gps_time, observations) or (None, []) if invalid.
    """
    rcvTow = getattr(parsed, 'rcvTow', None)
    week = getattr(parsed, 'week', None)
    numMeas = getattr(parsed, 'numMeas', 0)

    if rcvTow is None or week is None:
        return None, []

    gps_epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
    gps_time = gps_epoch + timedelta(weeks=week, seconds=rcvTow)

    # Signal name mapping (same as realtime_ppp.py serial_reader)
    SIG_NAMES = {
        (0, 0): 'GPS-L1CA', (0, 3): 'GPS-L2CL', (0, 4): 'GPS-L2CM',
        (0, 6): 'GPS-L5I', (0, 7): 'GPS-L5Q',
        (2, 0): 'GAL-E1C', (2, 1): 'GAL-E1B',
        (2, 3): 'GAL-E5aI', (2, 4): 'GAL-E5aQ',
        (2, 5): 'GAL-E5bI', (2, 6): 'GAL-E5bQ',
        (3, 0): 'BDS-B1I', (3, 1): 'BDS-B1C',
        (3, 5): 'BDS-B2aI', (3, 7): 'BDS-B2I',
    }
    SYS_PREFIX = {0: 'G', 2: 'E', 3: 'C'}

    observations = []
    for i in range(1, numMeas + 1):
        i2 = f"{i:02d}"
        gnss_id = getattr(parsed, f'gnssId_{i2}', None)
        sig_id = getattr(parsed, f'sigId_{i2}', None)
        sv_id = getattr(parsed, f'svId_{i2}', None)
        if gnss_id is None or sig_id is None:
            continue

        prefix = SYS_PREFIX.get(gnss_id)
        if prefix is None:
            continue

        sig_name = SIG_NAMES.get((gnss_id, sig_id))
        if sig_name is None:
            continue

        pr = getattr(parsed, f'prMes_{i2}', None)
        cp = getattr(parsed, f'cpMes_{i2}', None)
        cno = getattr(parsed, f'cno_{i2}', None)
        lock_ms = getattr(parsed, f'locktime_{i2}', 0)
        pr_valid = getattr(parsed, f'prValid_{i2}', 0)
        cp_valid = getattr(parsed, f'cpValid_{i2}', 0)
        half_cyc = getattr(parsed, f'halfCyc_{i2}', 0)

        if not pr_valid or pr is None:
            continue
        if pr < 1e6 or pr > 4e7:
            continue

        sv = f"{prefix}{int(sv_id):02d}"
        observations.append({
            'sv': sv,
            'pr': pr,
            'cp': cp if cp_valid and cp is not None else None,
            'cno': cno or 0,
            'lock_ms': lock_ms or 0,
            'sig_name': sig_name,
            'half_cyc': bool(half_cyc),
        })

    return gps_time, observations


def caster_serial_loop(port, baud, caster, position_file, stop_event,
                       phase2_event=None):
    """Read UBX from serial and feed observations to the NTRIP caster.

    Args:
        port: serial port path
        baud: baud rate
        caster: NtripCasterServer instance
        position_file: path to position.json for reference ARP
        stop_event: threading.Event to signal shutdown
        phase2_event: optional threading.Event, set when position is converged
                      (Phase 2 lock). Reference position is only broadcast
                      after this event is set.
    """
    try:
        from pyubx2 import UBXReader
        import serial as pyserial
    except ImportError:
        log.error("pyubx2/pyserial not installed")
        stop_event.set()
        return

    log.info(f"Caster serial: opening {port} at {baud} baud")
    ser = pyserial.Serial(port, baud, timeout=2)
    ubr = UBXReader(ser, protfilter=2)

    n_epochs = 0
    ref_ecef = None
    ref_loaded = False

    while not stop_event.is_set():
        try:
            raw, parsed = ubr.read()
            if parsed is None:
                continue

            if parsed.identity != 'RXM-RAWX':
                continue

            gps_time, observations = rawx_to_caster_obs(parsed)
            if gps_time is None or len(observations) < 4:
                continue

            # Load/refresh reference position
            # Only advertise after Phase 2 lock (if phase2_event is provided)
            if phase2_event is None or phase2_event.is_set():
                if not ref_loaded or n_epochs % 60 == 0:
                    loaded = load_position(position_file)
                    if loaded is not None:
                        ref_ecef = loaded
                        ref_loaded = True

            caster.broadcast_epoch(observations, gps_time,
                                   ref_ecef=ref_ecef)
            n_epochs += 1

            if n_epochs % 60 == 0:
                status = caster.status()
                log.info(f"Caster: {n_epochs} epochs, "
                         f"{status['clients_active']} clients, "
                         f"{status['bytes_sent']} bytes sent, "
                         f"ref={'yes' if ref_ecef is not None else 'no'}")

        except Exception as e:
            if not stop_event.is_set():
                log.error(f"Caster serial error: {e}")
            break

    ser.close()
    log.info(f"Caster serial stopped after {n_epochs} epochs")


# ── CLI ───────────────────────────────────────────────────────────────────── #

def main():
    import argparse
    import signal

    ap = argparse.ArgumentParser(
        description="PePPAR Fix NTRIP caster — serve RTCM MSM4 corrections")
    ap.add_argument("--serial", required=True,
                    help="Serial port for F9T (e.g. /dev/gnss-bot)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--bind", default=":2102",
                    help="Bind address:port (default: :2102)")
    ap.add_argument("--position", default="data/position.json",
                    help="Position file for reference ARP (default: data/position.json)")
    ap.add_argument("--station-id", type=int, default=0,
                    help="RTCM station ID (0-4095)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        stream=sys.stderr,
    )

    # Parse bind address
    bind_parts = args.bind.rsplit(':', 1)
    if len(bind_parts) == 2:
        bind_addr = bind_parts[0]
        bind_port = int(bind_parts[1])
    else:
        bind_addr = ""
        bind_port = int(bind_parts[0])

    stop_event = threading.Event()

    def on_signal(signum, frame):
        log.info("Shutdown signal received")
        stop_event.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # Start caster
    caster = NtripCasterServer(
        bind_addr=bind_addr, bind_port=bind_port,
        station_id=args.station_id)
    caster.start()

    # Run serial reader → caster loop
    try:
        caster_serial_loop(
            args.serial, args.baud, caster, args.position, stop_event)
    finally:
        caster.stop()

    log.info("NTRIP caster stopped")


if __name__ == "__main__":
    main()

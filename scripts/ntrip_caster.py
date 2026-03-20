#!/usr/bin/env python3
"""ntrip_caster.py — Serve raw GNSS observations as RTCM MSM4 corrections.

Reads UBX RXM-RAWX from the receiver serial port, encodes observations as
RTCM MSM4 (GPS 1074, GAL 1094, BDS 1124), and serves them via NTRIP to
multiple TCP clients. Includes RTCM 1005 (reference station ARP) when a
converged position is available.

Usage:
    # With known position:
    python ntrip_caster.py --serial /dev/gnss-top --baud 9600 \\
        --known-pos '41.8430626,-88.1037190,201.671' \\
        --caster :2102

    # With position from file (from peppar_find_position.py):
    python ntrip_caster.py --serial /dev/gnss-top --baud 9600 \\
        --position-file data/position.json \\
        --caster :2102

    # Without position (MSM only, no RTCM 1005):
    python ntrip_caster.py --serial /dev/gnss-top --baud 9600 \\
        --caster :2102

Clients connect with standard NTRIP v1:
    GET /PEPPAR HTTP/1.1
"""

import argparse
import json
import logging
import math
import os
import queue
import signal
import socket
import sys
import threading
import time

import numpy as np

from solve_pseudorange import lla_to_ecef
from peppar_fix.rtcm_encoder import (
    encode_epoch, UBX_SIG_NAMES, GNSS_ID_TO_SYS, MSM_SIGNAL_ID,
    BDS_MIN_PRN,
)

log = logging.getLogger("ntrip_caster")


# ── Raw observation reader ───────────────────────────────────────────────────── #

def raw_obs_reader(port, baud, obs_queue, stop_event, systems=None):
    """Read UBX RXM-RAWX and produce raw observation epochs.

    Puts dicts on obs_queue: {
        'gps_tow_ms': int,
        'week': int,
        'obs': {system: {prn: {sig_name: {pr, cp, cno, lock_ms, half_cyc}}}}
    }
    """
    try:
        from pyubx2 import UBXReader
        import serial as pyserial
    except ImportError:
        log.error("pyubx2/pyserial not installed — pip install pyubx2 pyserial")
        stop_event.set()
        return

    system_filter = set(systems) if systems else None

    log.info(f"Opening serial {port} at {baud} baud")
    ser = pyserial.Serial(port, baud, timeout=2)
    ubr = UBXReader(ser, protfilter=2)  # UBX only

    n_epochs = 0
    while not stop_event.is_set():
        try:
            raw, parsed = ubr.read()
            if parsed is None:
                continue

            if parsed.identity != 'RXM-RAWX':
                continue

            rcv_tow = parsed.rcvTow
            week = parsed.week
            num_meas = parsed.numMeas
            gps_tow_ms = round(rcv_tow * 1000) % 604800000

            # Build raw observations per system/prn/signal
            obs = {}  # system → prn → sig_name → measurement
            for i in range(1, num_meas + 1):
                i2 = f"{i:02d}"
                gnss_id = getattr(parsed, f'gnssId_{i2}', None)
                sig_id = getattr(parsed, f'sigId_{i2}', None)
                sv_id = getattr(parsed, f'svId_{i2}', None)
                if gnss_id is None or sig_id is None:
                    continue

                sig_name = UBX_SIG_NAMES.get((gnss_id, sig_id))
                if sig_name is None or sig_name not in MSM_SIGNAL_ID:
                    continue

                sys_name = GNSS_ID_TO_SYS.get(gnss_id)
                if sys_name is None:
                    continue
                if system_filter and sys_name.lower() not in system_filter:
                    continue

                prn = int(sv_id)
                # Skip BDS-2 GEO/IGSO
                if sys_name == 'BDS' and prn < BDS_MIN_PRN:
                    continue

                pr = getattr(parsed, f'prMes_{i2}', None)
                cp = getattr(parsed, f'cpMes_{i2}', None)
                cno = getattr(parsed, f'cno_{i2}', None)
                lock_ms = getattr(parsed, f'locktime_{i2}', 0) or 0.0
                pr_valid = getattr(parsed, f'prValid_{i2}', 0)
                cp_valid = getattr(parsed, f'cpValid_{i2}', 0)
                half_cyc = getattr(parsed, f'halfCyc_{i2}', 0)

                if not pr_valid or pr is None:
                    continue
                if pr < 1e6 or pr > 4e7:
                    continue

                if sys_name not in obs:
                    obs[sys_name] = {}
                if prn not in obs[sys_name]:
                    obs[sys_name][prn] = {}

                obs[sys_name][prn][sig_name] = {
                    'pr': pr,
                    'cp': cp if cp_valid else None,
                    'cno': cno or 0.0,
                    'lock_ms': lock_ms,
                    'half_cyc': half_cyc,
                }

            n_epochs += 1
            total_sigs = sum(
                len(sigs)
                for sys_obs in obs.values()
                for sigs in sys_obs.values()
            )
            if n_epochs <= 3 or n_epochs % 60 == 0:
                total_sats = sum(len(so) for so in obs.values())
                log.info(f"RAWX epoch {n_epochs}: {total_sats} SVs, "
                         f"{total_sigs} signals, systems={list(obs.keys())}")

            if total_sigs > 0:
                obs_queue.put({
                    'gps_tow_ms': gps_tow_ms,
                    'week': week,
                    'obs': obs,
                })

        except Exception as e:
            if not stop_event.is_set():
                log.warning(f"Serial read error: {e}")
                time.sleep(0.5)

    ser.close()
    log.info("Serial reader stopped")


# ── NTRIP TCP server ─────────────────────────────────────────────────────────── #

class NtripCaster:
    """Minimal NTRIP v1 caster serving RTCM corrections to TCP clients."""

    def __init__(self, bind_addr, port, mountpoint='PEPPAR', station_id=0):
        self.bind_addr = bind_addr
        self.port = port
        self.mountpoint = mountpoint
        self.station_id = station_id
        self._clients = []
        self._lock = threading.Lock()
        self._server = None
        self._running = False

    def start(self):
        """Start listening for NTRIP client connections."""
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((self.bind_addr, self.port))
        self._server.listen(5)
        self._server.settimeout(1.0)
        self._running = True
        threading.Thread(target=self._accept_loop, daemon=True).start()
        log.info(f"NTRIP caster listening on {self.bind_addr}:{self.port} "
                 f"mount=/{self.mountpoint}")

    def _accept_loop(self):
        while self._running:
            try:
                client, addr = self._server.accept()
                threading.Thread(
                    target=self._handle_client, args=(client, addr),
                    daemon=True
                ).start()
            except socket.timeout:
                continue
            except OSError:
                break

    def _handle_client(self, client, addr):
        """Handle incoming NTRIP client connection."""
        try:
            client.settimeout(5.0)
            data = client.recv(4096).decode('ascii', errors='replace')
            lines = data.split('\r\n')
            request = lines[0] if lines else ''
            parts = request.split()
            method = parts[0] if parts else ''
            path = parts[1] if len(parts) > 1 else '/'

            if method != 'GET':
                client.sendall(b'HTTP/1.1 405 Method Not Allowed\r\n\r\n')
                client.close()
                return

            if path == f'/{self.mountpoint}':
                # Stream RTCM data
                response = b'ICY 200 OK\r\n\r\n'
                client.sendall(response)
                client.settimeout(None)
                with self._lock:
                    self._clients.append(client)
                log.info(f"NTRIP client connected: {addr[0]}:{addr[1]}")
                return  # keep socket open

            elif path == '/':
                # Sourcetable
                entry = (f"STR;{self.mountpoint};PePPAR Fix;"
                         f"RTCM 3.3;1074(1),1094(1),1124(1),1005(10);"
                         f"2;GPS+GAL+BDS;PePPAR;AUS;;;0;0;No;B;N;0;\r\n"
                         f"ENDSOURCETABLE\r\n")
                header = (f"SOURCETABLE 200 OK\r\n"
                          f"Content-Type: text/plain\r\n"
                          f"Content-Length: {len(entry)}\r\n\r\n")
                client.sendall((header + entry).encode())
                client.close()
            else:
                client.sendall(b'HTTP/1.1 404 Not Found\r\n\r\n')
                client.close()

        except Exception as e:
            log.debug(f"Client handler error: {e}")
            try:
                client.close()
            except OSError:
                pass

    def broadcast(self, data):
        """Send RTCM data to all connected clients. Removes dead clients."""
        if not data:
            return
        with self._lock:
            dead = []
            for client in self._clients:
                try:
                    client.sendall(data)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    dead.append(client)
            for client in dead:
                self._clients.remove(client)
                try:
                    addr = client.getpeername()
                    log.info(f"NTRIP client disconnected: {addr[0]}:{addr[1]}")
                except OSError:
                    log.info("NTRIP client disconnected")
                try:
                    client.close()
                except OSError:
                    pass

    @property
    def client_count(self):
        with self._lock:
            return len(self._clients)

    def stop(self):
        self._running = False
        if self._server:
            try:
                self._server.close()
            except OSError:
                pass
        with self._lock:
            for client in self._clients:
                try:
                    client.close()
                except OSError:
                    pass
            self._clients.clear()
        log.info("NTRIP caster stopped")


# ── Main caster loop ─────────────────────────────────────────────────────────── #

def run_caster(args):
    """Main caster loop: read observations, encode RTCM, broadcast."""
    stop_event = threading.Event()

    def signal_handler(sig, frame):
        log.info(f"Received signal {sig}, shutting down...")
        stop_event.set()
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Parse position
    position_ecef = None
    if args.known_pos:
        parts = args.known_pos.split(',')
        lat, lon, alt = float(parts[0]), float(parts[1]), float(parts[2])
        position_ecef = tuple(lla_to_ecef(lat, lon, alt))
        log.info(f"Reference position: {lat:.7f}, {lon:.7f}, {alt:.3f}")
    elif args.position_file and os.path.exists(args.position_file):
        with open(args.position_file) as f:
            pos = json.load(f)
        lat, lon, alt = pos['lat'], pos['lon'], pos['alt']
        position_ecef = tuple(lla_to_ecef(lat, lon, alt))
        log.info(f"Position from file: {lat:.7f}, {lon:.7f}, {alt:.3f}")
    else:
        log.warning("No reference position — RTCM 1005 will NOT be sent. "
                     "Use --known-pos or --position-file.")

    # Parse bind address
    bind_addr = ''
    port = 2101
    caster_spec = args.caster
    if caster_spec.startswith(':'):
        port = int(caster_spec[1:])
    elif ':' in caster_spec:
        bind_addr, port_str = caster_spec.rsplit(':', 1)
        port = int(port_str)
    else:
        port = int(caster_spec)

    station_id = args.station_id
    mountpoint = args.mount

    # Start NTRIP caster
    caster = NtripCaster(bind_addr, port, mountpoint, station_id)
    caster.start()

    # Start serial reader
    systems = set(args.systems.split(',')) if args.systems else None
    obs_q = queue.Queue(maxsize=10)
    reader_thread = threading.Thread(
        target=raw_obs_reader,
        args=(args.serial, args.baud, obs_q, stop_event, systems),
        daemon=True,
    )
    reader_thread.start()

    # RTCM 1005 interval
    msg_1005_interval = args.pos_interval
    last_1005_time = 0

    n_epochs = 0
    try:
        while not stop_event.is_set():
            try:
                epoch = obs_q.get(timeout=1.0)
            except queue.Empty:
                continue

            gps_tow_ms = epoch['gps_tow_ms']
            raw_obs = epoch['obs']

            # Include position in RTCM 1005 periodically
            now = time.monotonic()
            include_pos = None
            if position_ecef and (now - last_1005_time) >= msg_1005_interval:
                include_pos = position_ecef
                last_1005_time = now

            # Encode and broadcast
            rtcm_data = encode_epoch(
                station_id, gps_tow_ms, raw_obs, include_pos
            )
            if rtcm_data:
                caster.broadcast(rtcm_data)

            n_epochs += 1
            if n_epochs <= 3 or n_epochs % 60 == 0:
                log.info(f"Epoch {n_epochs}: {len(rtcm_data)} bytes RTCM, "
                         f"{caster.client_count} clients")

            # Duration limit
            if args.duration and n_epochs >= args.duration:
                log.info(f"Duration limit reached ({args.duration} epochs)")
                break

    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        caster.stop()
        reader_thread.join(timeout=3)

    log.info(f"Caster finished: {n_epochs} epochs served")


# ── CLI ──────────────────────────────────────────────────────────────────────── #

def main():
    ap = argparse.ArgumentParser(
        description="NTRIP caster — serve GNSS observations as RTCM MSM4",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Serial
    ap.add_argument("--serial", required=True,
                    help="F9T serial port (e.g. /dev/gnss-top)")
    ap.add_argument("--baud", type=int, default=9600,
                    help="Serial baud rate (default: 9600)")

    # Position
    ap.add_argument("--known-pos",
                    help="Reference position as lat,lon,alt (enables RTCM 1005)")
    ap.add_argument("--position-file",
                    help="JSON position file (from peppar_find_position.py)")

    # Caster
    ap.add_argument("--caster", default=":2101",
                    help="Bind address:port (default: :2101). "
                         "Examples: :2102, 0.0.0.0:2101, 2101")
    ap.add_argument("--mount", default="PEPPAR",
                    help="NTRIP mountpoint name (default: PEPPAR)")
    ap.add_argument("--station-id", type=int, default=0,
                    help="RTCM station ID (default: 0)")

    # Filtering
    ap.add_argument("--systems", default="gps,gal,bds",
                    help="GNSS systems to include (default: gps,gal,bds)")
    ap.add_argument("--pos-interval", type=float, default=10.0,
                    help="RTCM 1005 broadcast interval in seconds (default: 10)")

    # Runtime
    ap.add_argument("--duration", type=int, default=None,
                    help="Run for N epochs then stop")
    ap.add_argument("-v", "--verbose", action="store_true")

    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        stream=sys.stderr,
    )

    run_caster(args)


if __name__ == "__main__":
    main()

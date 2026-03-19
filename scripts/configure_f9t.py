#!/usr/bin/env python3
"""
configure_f9t.py — Configure a u-blox ZED-F9T for PPP-AR observations.

Cross-rig note: This script is an intentional FORK of testAnt's
configure_receivers.py. Both share infrastructure (factory reset, ACK
waiting, CFG-VALSET, baud probing) but differ in signal configuration:
  - testAnt: GPS L1+L5, GAL E1 only, BDS B1 only (antenna evaluation)
  - peppar_fix: GPS L1+L5, GAL E1+E5a, BDS B1+B2a (dual-freq IF for PPP-AR)
Do NOT add this to shared_files.toml — it should diverge.
Long-term plan: extract shared F9T utilities (reset, ACK, VALSET, baud
probing) into a separate repo (e.g. bobvan/f9tLibs) and import from both.

Sequence:
  1. Factory reset (CFG-RST)
  2. Configure dual-frequency signals (GPS L1C/A+L5, GAL E1+E5a, BDS B1+B2a)
  3. Set measurement rate (default 1 Hz, parameterizable to 10 Hz)
  4. Enable required UBX messages on the active port:
       - RXM-RAWX   (raw observations: pseudorange, carrier phase, Doppler)
       - RXM-SFRBX  (broadcast navigation data / ephemeris)
       - NAV-PVT    (position/velocity/time for bootstrap & monitoring)
       - NAV-SAT    (satellite info: elevation, azimuth, C/N0)
       - TIM-TP     (PPS quantization error)
  5. Configure survey-in mode (default 300s, 5m accuracy)
  6. Set UART baud rate if needed (default 460800 for 10 Hz headroom)
  7. Save configuration to flash (CFG-RST with BBR mask)

Usage:
    python configure_f9t.py /dev/ttyF9T
    python configure_f9t.py /dev/ttyF9T --rate 10 --survey-dur 600 --survey-acc 2.5
    python configure_f9t.py /dev/ttyF9T --port USB  # skip baud rate change
"""

import argparse
import sys
import time

try:
    from pyubx2 import UBXMessage, UBXReader, SET, POLL
    from serial import Serial
except ImportError:
    print("ERROR: requires pyubx2 and pyserial", file=sys.stderr)
    print("  pip install pyubx2 pyserial", file=sys.stderr)
    sys.exit(1)


# ── Signal configuration ──────────────────────────────────────────────────── #
# Receiver-specific signal config from the driver abstraction.
# The driver knows which signals to enable for each receiver model.
from peppar_fix.receiver import get_driver, F9TDriver


def probe_baud(port):
    """Try common baud rates and return the one that produces valid UBX/NMEA."""
    for baud in [9600, 38400, 115200, 230400, 460800]:
        try:
            ser = Serial(port, baudrate=baud, timeout=2)
            ser.reset_input_buffer()
            time.sleep(1.5)
            data = ser.read(500)
            ser.close()
            if b'\xb5\x62' in data or b'$G' in data:
                return baud
        except Exception:
            pass
    return None


def send_cfg(ser, ubr, key_values, description):
    """Send a VALSET configuration and wait for ACK."""
    layers = 7  # RAM + BBR + Flash
    cfg_data = list(key_values.items())
    msg = UBXMessage.config_set(layers, 0, cfg_data)
    print(f"  {description}...", end=" ", flush=True)
    ser.write(msg.serialize())
    ack = wait_ack(ubr, "CFG", "VALSET", timeout=3.0)
    if ack:
        print("OK")
    else:
        print("TIMEOUT (no ACK)")
    return ack


def wait_ack(ubr, cls_name, msg_name, timeout=3.0):
    """Wait for UBX-ACK-ACK or UBX-ACK-NAK."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            raw, parsed = ubr.read()
        except Exception:
            continue
        if parsed is None:
            continue
        if parsed.identity == "ACK-ACK":
            return True
        if parsed.identity == "ACK-NAK":
            print(f"[NAK received for {cls_name}-{msg_name}]", file=sys.stderr)
            return False
    return False


def factory_reset(ser, ubr):
    """Issue a controlled software reset with factory defaults."""
    print("  Factory reset...", end=" ", flush=True)
    # CFG-RST: navBbrMask=0xFFFF (clear all), resetMode=1 (controlled SW reset)
    msg = UBXMessage(
        "CFG", "CFG-RST",
        SET,
        navBbrMask=0xFFFF,
        resetMode=1,  # controlled software reset
        reserved0=0,
    )
    ser.write(msg.serialize())
    # Receiver reboots — brief pause before caller closes port
    time.sleep(1)
    print("OK (receiver rebooting)")


def configure_rate(ser, ubr, rate_hz):
    """Set measurement and navigation rate."""
    meas_ms = int(1000 / rate_hz)
    return send_cfg(ser, ubr, {
        "CFG_RATE_MEAS": meas_ms,
        "CFG_RATE_NAV": 1,           # one nav solution per measurement
        "CFG_RATE_TIMEREF": 0,       # UTC
    }, f"Measurement rate = {rate_hz} Hz ({meas_ms} ms)")


def configure_signals(ser, ubr, driver=None):
    """Enable dual-frequency signals for PPP-AR."""
    if driver is None:
        driver = F9TDriver()
    return send_cfg(ser, ubr, driver.signal_config,
                    f"Signals: GPS L1+L5, GAL E1+E5a, BDS B1+B2a ({driver.name})")


def configure_messages(ser, ubr, port_id):
    """Enable required UBX messages on the specified port."""
    # Port IDs: 1=UART1, 2=UART2, 3=USB, 4=SPI
    port_suffix = {1: "UART1", 2: "UART2", 3: "USB", 4: "SPI"}
    pname = port_suffix.get(port_id, f"port{port_id}")

    messages = {
        f"CFG_MSGOUT_UBX_RXM_RAWX_{pname}": 1,
        f"CFG_MSGOUT_UBX_RXM_SFRBX_{pname}": 1,
        f"CFG_MSGOUT_UBX_NAV_PVT_{pname}": 1,
        f"CFG_MSGOUT_UBX_NAV_SAT_{pname}": 5,   # every 5th epoch (save bandwidth)
        f"CFG_MSGOUT_UBX_TIM_TP_{pname}": 1,
    }

    return send_cfg(ser, ubr, messages,
                    f"UBX messages on {pname}: RAWX, SFRBX, PVT, SAT, TIM-TP")


def configure_nmea_off(ser, ubr, port_id):
    """Disable NMEA output on the port to save bandwidth (best-effort)."""
    port_suffix = {1: "UART1", 2: "UART2", 3: "USB", 4: "SPI"}
    pname = port_suffix.get(port_id, f"port{port_id}")

    # Only use message names known to work on F9T
    nmea_off = {}
    for nmea_msg in ["GGA", "GLL", "GSA", "GSV", "RMC", "VTG"]:
        key = f"CFG_MSGOUT_NMEA_ID_{nmea_msg}_{pname}"
        nmea_off[key] = 0

    result = send_cfg(ser, ubr, nmea_off, f"Disable NMEA output on {pname}")
    if not result:
        print("    (NMEA disable failed — non-critical, continuing)")
    return True  # non-critical


def configure_tmode(ser, ubr, survey_dur_s, survey_acc_m):
    """Configure survey-in for Time Mode (required for TIM-TP and timing operation)."""
    acc_mm = int(survey_acc_m * 1000)  # convert to mm
    acc_tenths_mm = acc_mm * 10        # CFG-TMODE uses 0.1 mm units

    return send_cfg(ser, ubr, {
        "CFG_TMODE_MODE": 1,                 # 1 = Survey-In
        "CFG_TMODE_SVIN_MIN_DUR": survey_dur_s,
        "CFG_TMODE_SVIN_ACC_LIMIT": acc_tenths_mm,
    }, f"Survey-in: {survey_dur_s}s, {survey_acc_m}m accuracy")


def configure_uart_baud(ser, ubr, baud):
    """Set UART1 baud rate for high-rate output."""
    cfg_data = [("CFG_UART1_BAUDRATE", baud)]
    msg = UBXMessage.config_set(7, 0, cfg_data)
    print(f"  UART1 baud rate = {baud}...", end=" ", flush=True)
    ser.write(msg.serialize())
    # Receiver changes baud immediately — switch host before reading ACK
    time.sleep(0.2)
    ser.baudrate = baud
    time.sleep(0.5)
    ser.reset_input_buffer()
    print("OK (baud changed, ACK skipped)")
    return True


def configure_gps_l5_health(ser, ubr):
    """Override GPS L5 health status so receiver tracks L5 signals.

    GPS Block IIF/III satellites flag L5 as "unhealthy" in the navigation
    message while the constellation is pre-operational.  This raw CFG-VALSET
    sets key 0x10320001 to 1, causing the receiver to substitute GPS L1 C/A
    health status for L5.

    Source: u-blox App Note UBX-21038688 "GPS L5 configuration"
    A NAK means the key is unsupported — GPS L5 simply won't be tracked.
    """
    raw_msg = bytes([
        0xB5, 0x62,              # UBX sync
        0x06, 0x8A,              # class=CFG, id=VALSET
        0x09, 0x00,              # length = 9
        0x01, 0x07, 0x00, 0x00,  # version=1, layers=RAM+BBR+Flash, reserved
        0x01, 0x00, 0x32, 0x10,  # key 0x10320001 (little-endian)
        0x01,                    # value = 1 (enable override)
        0xE5, 0x26,              # Fletcher checksum
    ])
    print("  GPS L5 health override (UBX-21038688)...", end=" ", flush=True)
    ser.write(raw_msg)
    ack = wait_ack(ubr, "CFG", "VALSET", timeout=3.0)
    if ack:
        print("OK")
    else:
        print("NAK (key not supported — GPS L5 will not be tracked)")
    return ack


def save_config(ser, ubr):
    """Save current config to flash (BBR + Flash layers)."""
    # Saving is implicit when using layers=7 in VALSET, but belt-and-suspenders:
    print("  Configuration saved to RAM + BBR + Flash.")


def verify_messages(ubr, timeout=10):
    """Wait for expected messages to confirm configuration is active."""
    print(f"\n  Verifying output (waiting {timeout}s for messages)...")
    seen = set()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            raw, parsed = ubr.read()
        except Exception:
            continue
        if parsed is None:
            continue
        ident = parsed.identity
        if ident not in seen:
            seen.add(ident)
            print(f"    ✓ {ident}")
    expected = {"RXM-RAWX", "RXM-SFRBX", "NAV-PVT", "TIM-TP"}
    missing = expected - seen
    if missing:
        print(f"  WARNING: did not see: {missing}")
    else:
        print("  All expected messages confirmed.")
    return missing


def main():
    ap = argparse.ArgumentParser(
        description="Configure a u-blox receiver for PPP-AR observations")
    ap.add_argument("port", help="Serial port (e.g. /dev/ttyF9T, /dev/ttyUSB0)")
    ap.add_argument("--receiver", default="f9t",
                    help="Receiver model: f9t, f10t (default: f9t)")
    ap.add_argument("--baud", type=int, default=9600,
                    help="Current baud rate (default 9600 after factory reset)")
    ap.add_argument("--target-baud", type=int, default=None,
                    help="Target UART baud rate (default: from driver). Ignored for USB.")
    ap.add_argument("--rate", type=int, default=1, choices=range(1, 11),
                    help="Measurement rate in Hz (default 1, max 10)")
    ap.add_argument("--survey-dur", type=int, default=300,
                    help="Survey-in minimum duration in seconds (default 300)")
    ap.add_argument("--survey-acc", type=float, default=5.0,
                    help="Survey-in accuracy threshold in meters (default 5.0)")
    ap.add_argument("--port-type", default="UART", choices=["UART", "USB"],
                    help="Connection type (default UART)")
    ap.add_argument("--skip-reset", action="store_true",
                    help="Skip factory reset (use if already configured)")
    ap.add_argument("--verify", action="store_true", default=True,
                    help="Verify messages after configuration (default True)")
    ap.add_argument("--no-verify", action="store_false", dest="verify",
                    help="Skip message verification")
    args = ap.parse_args()

    driver = get_driver(args.receiver)
    if args.target_baud is None:
        args.target_baud = driver.default_baud

    port_id = 1 if args.port_type == "UART" else 3

    print(f"PePPAR Fix — {driver.name} Configuration")
    print(f"  Port: {args.port} ({args.port_type})")
    print(f"  Receiver: {driver.name} (PROTVER {driver.protver})")
    print(f"  Rate: {args.rate} Hz")
    if driver.supports_timing_mode:
        print(f"  Survey-in: {args.survey_dur}s, {args.survey_acc}m")
    else:
        print(f"  Survey-in: N/A (no timing mode on {driver.name})")
    print()

    ser = Serial(args.port, baudrate=args.baud, timeout=1)
    ubr = UBXReader(ser, protfilter=2)  # UBX protocol only

    # Step 1: Factory reset
    if not args.skip_reset:
        factory_reset(ser, ubr)
        # Receiver reboots — close and reopen to flush stale data.
        # Baud may or may not change (OTC hardware keeps 115200).
        ser.close()
        time.sleep(5)  # F9T needs several seconds to boot
        # Probe for correct baud after reset (retry once)
        post_baud = probe_baud(args.port)
        if post_baud is None:
            print("  First probe failed, waiting 5s more...", file=sys.stderr)
            time.sleep(5)
            post_baud = probe_baud(args.port)
        if post_baud is None:
            print("ERROR: Cannot find receiver after reset", file=sys.stderr)
            sys.exit(1)
        print(f"  Receiver found at {post_baud} baud after reset")
        ser = Serial(args.port, baudrate=post_baud, timeout=1)
        ser.reset_input_buffer()
        ubr = UBXReader(ser, protfilter=2)

    # Step 2: Configure signals + GPS L5 health override
    configure_signals(ser, ubr, driver=driver)
    l5_ok = False
    if driver.supports_l5_health_override:
        l5_ok = configure_gps_l5_health(ser, ubr)
    else:
        print(f"  GPS L5 health override: N/A for {driver.name}")

    # The L5 health override and signal config are saved to flash but
    # require a warm restart to take effect.  Without this, GPS L5
    # will not be tracked even though the config is accepted.
    if l5_ok:
        print("  Warm restart for L5 health override...", end=" ", flush=True)
        msg = UBXMessage(
            "CFG", "CFG-RST", SET,
            navBbrMask=0x0001,  # hot start (keep ephemeris)
            resetMode=1,        # controlled software reset
            reserved0=0,
        )
        ser.write(msg.serialize())
        ser.close()
        time.sleep(10)
        post_baud = probe_baud(args.port)
        if post_baud is None:
            time.sleep(5)
            post_baud = probe_baud(args.port)
        if post_baud is None:
            print("ERROR: receiver not found after L5 restart", file=sys.stderr)
            sys.exit(1)
        ser = Serial(args.port, baudrate=post_baud, timeout=1)
        ser.reset_input_buffer()
        ubr = UBXReader(ser, protfilter=2)
        print(f"OK (found at {post_baud} baud)")

    # Step 3: Set measurement rate
    configure_rate(ser, ubr, args.rate)

    # Step 4: Enable UBX messages, disable NMEA
    configure_messages(ser, ubr, port_id)
    configure_nmea_off(ser, ubr, port_id)

    # Step 5: Survey-in for Time Mode (timing receivers only)
    if driver.supports_timing_mode:
        configure_tmode(ser, ubr, args.survey_dur, args.survey_acc)
    else:
        print(f"  Survey-in: skipped ({driver.name} has no timing mode)")

    # Step 6: UART baud rate (skip for USB)
    if args.port_type == "UART" and args.target_baud != 9600:
        configure_uart_baud(ser, ubr, args.target_baud)

    # Step 7: Save confirmation
    save_config(ser, ubr)

    # Verify
    if args.verify:
        missing = verify_messages(ubr, timeout=15)
        if missing:
            print(f"\nWARNING: Missing messages: {missing}", file=sys.stderr)
            sys.exit(1)

    print(f"\n{driver.name} configured for PPP-AR observations.")
    print(f"  Next: python scripts/log_observations.py {args.port}"
          f" --baud {args.target_baud if args.port_type == 'UART' else args.baud}")
    ser.close()


if __name__ == "__main__":
    main()

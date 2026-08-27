#!/usr/bin/env python3
"""Reproducer + attribution rig for the igc TX-timestamp wedge.

The igc driver (Intel i225/i226) wedges its hardware TX-timestamp path:
"Tx timestamp timeout" in dmesg, `tx_hwtstamp_timeouts` climbing, and
(reportedly) EXTTS capture dying with it.  Two mechanisms are on the
table and neither is verified -- see docs/igc-wedge-repro-plan.md.

This runs two independent loops against one NIC:

  1. `clock_adjtime(ADJ_FREQUENCY)` on the PHC, at `--adjfine-hz`
  2. UDP sends with SO_TIMESTAMPING hardware TX timestamps, at `--tx-hz`

Both accept 0 = OFF and `unthrottled` = as fast as the CPU allows.
**`--adjfine-hz 0` is what makes arm A possible** (does it wedge with no
adjfine at all?), and arm A is the decisive one: if it wedges, every
adjfine-centred hypothesis dies at once.

The rig needs LINK UP AND NOTHING ELSE on the DUT -- SO_BINDTODEVICE plus
224.0.0.1 link-local multicast means no ARP, no peer, no route, no IP
config.  No antenna, no PPS, no TICC, no ptp4l peer.  Nothing has to
receive the packets.

Driver identity is asserted, not assumed.  `--expect-srcversion` is checked
against /sys/module/igc/srcversion before anything runs, and the value is
stamped into the CSV.  This exists because the 2026-04 record cannot say
which binary produced its observations: the DKMS package is named for one
patch but carries two, and nobody logged what was loaded.  Don't reintroduce
that -- load the module by absolute path and assert what came up.

Usage:
    # screening: find the fast regime
    sudo ./igc_tx_timeout_repro.py --iface eth1 --ptp /dev/ptp0 \
        --adjfine-hz unthrottled --tx-hz unthrottled --duration 120

    # arm A: adjfine OFF, 1 Hz TX, patched driver
    sudo ./igc_tx_timeout_repro.py --iface eth1 --ptp /dev/ptp0 \
        --adjfine-hz 0 --tx-hz 1 --duration 7200 \
        --expect-srcversion 34A0F0BF20444879727CD8F --csv armA.csv

Requires root for SO_TIMESTAMPING and clock_adjtime on the PHC.
"""

import argparse
import ctypes
import ctypes.util
import errno
import hashlib
import os
import re
import select
import socket
import struct
import subprocess
import sys
import threading
import time

ETHTOOL = "/usr/sbin/ethtool"
DMESG_RE = re.compile(r"^\[\s*(\d+\.\d+)\] .*Tx timestamp timeout", re.M)
# Counters we sample once per second.  tx_hwtstamp_timeouts is the one the
# whole method rests on; the others are cheap context.
COUNTERS = ("tx_hwtstamp_timeouts", "tx_hwtstamp_skipped", "rx_hwtstamp_cleared")


# ── driver identity ────────────────────────────────────────────────────────

def running_srcversion():
    """Runtime identity of the loaded igc, independent of package names."""
    try:
        with open("/sys/module/igc/srcversion") as f:
            return f.read().strip()
    except OSError:
        return None


def assert_driver(expected):
    """Refuse to run an arm against an unidentified or wrong driver."""
    actual = running_srcversion()
    if actual is None:
        sys.exit("FATAL: igc not loaded (no /sys/module/igc/srcversion)")
    if expected and actual != expected:
        sys.exit(f"FATAL: driver mismatch — expected srcversion {expected}, "
                 f"loaded is {actual}.  Load the intended binary by absolute "
                 f"path (insmod, not modprobe) and re-run.")
    return actual


# ── counters ───────────────────────────────────────────────────────────────

def read_counters(iface):
    """Parse `ethtool -S` for the hwtstamp counters.  {} if unavailable."""
    try:
        out = subprocess.run([ETHTOOL, "-S", iface], capture_output=True,
                             text=True, timeout=5).stdout
    except Exception:
        return {}
    vals = {}
    for name in COUNTERS:
        m = re.search(rf"^\s*{name}:\s*(\d+)\s*$", out, re.M)
        if m:
            vals[name] = int(m.group(1))
    return vals


def dmesg_timeout_times():
    """Kernel timestamps (s, µs resolution) of every 'Tx timestamp timeout'.

    This -- not the 1 Hz counter -- is the H1 discriminator.  The i226 has
    four TX-timestamp slots and igc_ptp_tx_hang()'s rd32(IGC_TXSTMPH_0)
    invalidates all of them at once, so orphaned slots time out together in
    a burst microseconds wide.  Per-second counter deltas can only say "+4
    this second"; the kernel timestamps show the burst directly.
    """
    try:
        out = subprocess.run(["dmesg"], capture_output=True, text=True,
                             timeout=10).stdout
    except Exception:
        return []
    return [float(m) for m in DMESG_RE.findall(out)]


def burst_report(times, gap=1.0):
    """Group kernel-timestamped events into bursts split on gaps > `gap`."""
    if not times:
        return []
    bursts = [[times[0]]]
    for t in times[1:]:
        if t - bursts[-1][-1] <= gap:
            bursts[-1].append(t)
        else:
            bursts.append([t])
    return bursts


# ── EXTTS liveness ─────────────────────────────────────────────────────────

class ExttsCounter:
    """Count EXTTS events on one channel, to time EXTTS death vs first timeout.

    On TimeHAT-class boards the PPS INPUT is **SDP1**, not SDP0 -- SDP0 is the
    PEROUT *output*.  Tools that default to pin 0 watch the output pin and see
    no edges (cost an afternoon on MadHat 2026-05-22).  Hence --extts-pin,
    defaulting to 1, and the pre-flight below.

    THE PRE-FLIGHT IS THE POINT.  An unrouted pin and a dead EXTTS look
    identical from here -- both are zero events.  Reporting "EXTTS died at
    t=0" when the pin was simply never routed would be worse than not
    measuring at all, so a run that asks for EXTTS must SEE events before the
    arm starts, or abort.
    """
    PTP_EXTTS_REQUEST = 0x40103d02          # v1: flags unvalidated
    PTP_EXTTS_REQUEST2 = 0x40103d0b         # v2: validates flags strictly
    PTP_ENABLE_FEATURE = 1 << 0
    PTP_RISING_EDGE = 1 << 1
    # struct ptp_extts_event: ptp_clock_time{s64 sec; u32 nsec; u32 rsv}
    # + u32 index + u32 flags + u32 rsv[2]  =  16 + 4 + 4 + 8 = 32 bytes.
    EVENT_SIZE = 32

    def __init__(self, phc_fd, ptp_dev, index, pin):
        self.fd, self.index, self.pin = phc_fd, index, pin
        self.count, self.ok = 0, False
        self.ptp_name = os.path.basename(os.path.realpath(ptp_dev))

    def route_pin(self):
        """Point the SDP pin at this EXTTS channel (func 1 = PTP_PF_EXTTS)."""
        if self.pin is None:
            return True
        path = f"/sys/class/ptp/{self.ptp_name}/pins/SDP{self.pin}"
        try:
            with open(path, "w") as f:
                f.write(f"1 {self.index}")
            return True
        except OSError as e:
            print(f"  could not route SDP{self.pin} -> EXTTS ch{self.index}: {e}")
            return False

    def enable(self):
        """Enable the channel, preferring the validating v2 ioctl.

        v2 rejects PTP_ENABLE_FEATURE unless an edge bit is also set (EINVAL),
        so ask for rising edges explicitly.  v1 does not validate flags and is
        the fallback for kernels/drivers without v2.
        """
        import fcntl
        self.route_pin()
        attempts = (
            ("PTP_EXTTS_REQUEST2", self.PTP_EXTTS_REQUEST2,
             self.PTP_ENABLE_FEATURE | self.PTP_RISING_EDGE),
            ("PTP_EXTTS_REQUEST", self.PTP_EXTTS_REQUEST,
             self.PTP_ENABLE_FEATURE),
        )
        for name, nr, flags in attempts:
            req = struct.pack("IIII", self.index, flags, 0, 0)
            try:
                fcntl.ioctl(self.fd, nr, req)
                self.ok = True
                self.via = name
                return True
            except OSError as e:
                last = f"{name}: {e.strerror}"
        print(f"  EXTTS channel {self.index} not enabled ({last})")
        return False

    def preflight(self, need=2, timeout_s=5.0):
        """Refuse to run unless real edges are arriving on the pin."""
        if not self.ok:
            sys.exit(f"FATAL: --extts-index {self.index} was requested but the "
                     f"channel could not be enabled.  Refusing to run: a "
                     f"disabled channel reports zero events, which is "
                     f"indistinguishable from 'EXTTS died'.")
        t0, seen = time.monotonic(), 0
        while time.monotonic() - t0 < timeout_s and seen < need:
            time.sleep(0.25)
            seen += self.drain() or 0
        self.count = 0                       # don't bill pre-flight to the run
        if seen < need:
            sys.exit(f"FATAL: asked for EXTTS on SDP{self.pin}/ch{self.index} "
                     f"but saw {seen} edges in {timeout_s:g}s.  An UNROUTED pin "
                     f"and a DEAD EXTTS look identical from here — refusing to "
                     f"run rather than report a misleading 'EXTTS died at t=0'. "
                     f"Check the PPS source and that SDP{self.pin} is the INPUT "
                     f"pin on this board (SDP0 is PPS OUT on TimeHAT).")
        print(f"  EXTTS pre-flight OK: {seen} edges on SDP{self.pin}/ch{self.index}")
        return True

    def drain(self):
        """Read pending events without blocking; returns count seen.

        Gate every read on select() rather than trusting O_NONBLOCK.  The PTP
        chardev does NOT honour it here: with the channel enabled and the
        queue empty, os.read() slept in the kernel's ptp_read() (observed on
        TimeHat, kernel 6.12.75, wchan=ptp_read syscall=63) instead of raising
        EWOULDBLOCK.  A blocking read here would hang an entire arm.
        """
        if not self.ok:
            return None
        n = 0
        while True:
            r, _, _ = select.select([self.fd], [], [], 0)
            if not r:
                break
            try:
                data = os.read(self.fd, self.EVENT_SIZE)
            except (BlockingIOError, OSError):
                break
            if not data:
                break
            n += len(data) // self.EVENT_SIZE
        self.count += n
        return n


# ── load generators ────────────────────────────────────────────────────────

class Timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]


class Timex(ctypes.Structure):
    _fields_ = [
        ("modes", ctypes.c_uint), ("offset", ctypes.c_long),
        ("freq", ctypes.c_long), ("maxerror", ctypes.c_long),
        ("esterror", ctypes.c_long), ("status", ctypes.c_int),
        ("constant", ctypes.c_long), ("precision", ctypes.c_long),
        ("tolerance", ctypes.c_long), ("time", Timeval),
        ("tick", ctypes.c_long), ("ppsfreq", ctypes.c_long),
        ("jitter", ctypes.c_long), ("shift", ctypes.c_int),
        ("stabil", ctypes.c_long), ("jitcnt", ctypes.c_long),
        ("calcnt", ctypes.c_long), ("errcnt", ctypes.c_long),
        ("stbcnt", ctypes.c_long), ("tai", ctypes.c_int),
    ]


def _pace(target_hz, n, t0):
    """Sleep to hold `target_hz`.  None/0 target means caller shouldn't call."""
    due = t0 + n / target_hz
    slack = due - time.monotonic()
    if slack > 0:
        time.sleep(slack)


def adjfine_loop(phc_fd, rate, stop, stats):
    """clock_adjtime(ADJ_FREQUENCY) at `rate` Hz.  rate=0 → thread never runs."""
    librt = ctypes.CDLL(ctypes.util.find_library("rt"), use_errno=True)
    ADJ_FREQUENCY = 0x0002
    clockid = (~phc_fd << 3) | 3
    n, toggle, t0 = 0, False, time.monotonic()
    while not stop.is_set():
        tx = Timex()
        tx.modes = ADJ_FREQUENCY
        tx.freq = 100 if toggle else -100      # ±~0.0015 ppb, deliberately tiny
        toggle = not toggle
        if librt.clock_adjtime(ctypes.c_int32(clockid), ctypes.byref(tx)) < 0:
            stats["adjfine_errno"] = ctypes.get_errno()
            break
        n += 1
        if rate:
            _pace(rate, n, t0)
    stats["adjfine_count"] = n


def tx_loop(iface, rate, stop, stats):
    """UDP sends requesting HW TX timestamps at `rate` Hz."""
    SO_TIMESTAMPING = 37
    flags = ((1 << 0)    # SOF_TIMESTAMPING_TX_HARDWARE
             | (1 << 6)  # SOF_TIMESTAMPING_RAW_HARDWARE
             | (1 << 11))  # SOF_TIMESTAMPING_OPT_TSONLY
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPING, flags)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE,
                    iface.encode() + b"\0")
    sock.settimeout(0.01)
    dest = ("224.0.0.1", 9999)          # link-local multicast: goes nowhere
    payload = b"igc_repro" + b"\x00" * 32
    n, errs, t0 = 0, 0, time.monotonic()
    while not stop.is_set():
        try:
            sock.sendto(payload, dest)
            n += 1
        except OSError:
            errs += 1
            n += 1
        if rate:
            _pace(rate, n, t0)
        elif n % 100 == 0:
            time.sleep(0.0001)          # yield, don't starve the adjfine thread
    sock.close()
    stats["tx_count"], stats["tx_errors"] = n, errs


# ── main ───────────────────────────────────────────────────────────────────

def hz(v):
    """Rate argument: a number, 0 for OFF, or 'unthrottled'."""
    if str(v).lower() in ("unthrottled", "max", "-1"):
        return None
    f = float(v)
    if f < 0:
        raise argparse.ArgumentTypeError("rate must be >= 0, or 'unthrottled'")
    return f


def main():
    ap = argparse.ArgumentParser(
        description="igc TX-timestamp wedge reproducer / attribution rig",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", required=True, help="DUT interface (e.g. eth1)")
    ap.add_argument("--ptp", required=True, help="PHC device (e.g. /dev/ptp0)")
    ap.add_argument("--adjfine-hz", type=hz, default=None, metavar="N",
                    help="adjfine rate; 0 = OFF (arm A), 'unthrottled' = max "
                         "(default: unthrottled)")
    ap.add_argument("--tx-hz", type=hz, default=None, metavar="N",
                    help="TX rate; 0 = OFF, 'unthrottled' = max "
                         "(default: unthrottled)")
    ap.add_argument("--duration", type=float, default=30.0, metavar="S")
    ap.add_argument("--csv", metavar="PATH",
                    help="per-second counter samples (the burst discriminator)")
    ap.add_argument("--expect-srcversion", metavar="HEX",
                    help="abort unless the loaded igc matches this srcversion")
    ap.add_argument("--extts-index", type=int, metavar="N",
                    help="enable EXTTS channel N and count events per second; "
                         "aborts unless real edges arrive (see --extts-pin)")
    ap.add_argument("--extts-pin", type=int, default=1, metavar="N",
                    help="SDP pin carrying PPS IN (default 1 — SDP0 is PPS OUT "
                         "on TimeHAT boards, watching it sees nothing)")
    ap.add_argument("--label", default="", help="arm label, recorded in the CSV")
    args = ap.parse_args()

    if os.geteuid() != 0:
        sys.exit("ERROR: must run as root (sudo)")

    # A killed run must not lose its output.  The first screening run died
    # early and its block-buffered stdout was a 0-byte file, while the CSV
    # (line-buffered) survived.  Don't repeat that.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except AttributeError:
        pass

    srcver = assert_driver(args.expect_srcversion)

    phc_fd = os.open(args.ptp, os.O_RDWR | os.O_NONBLOCK)
    extts = None
    if args.extts_index is not None:
        extts = ExttsCounter(phc_fd, args.ptp, args.extts_index, args.extts_pin)
        extts.enable()
        extts.preflight()

    base = read_counters(args.iface)
    if "tx_hwtstamp_timeouts" not in base:
        sys.exit(f"FATAL: no tx_hwtstamp_timeouts counter on {args.iface} "
                 f"via {ETHTOOL} — the whole method rests on it.")

    def rate_str(r):
        return "unthrottled" if r is None else ("OFF" if r == 0 else f"{r:g} Hz")

    print(f"iface={args.iface} phc={args.ptp} label={args.label or '-'}")
    print(f"driver srcversion={srcver}"
          f"{' (asserted)' if args.expect_srcversion else ' (NOT asserted)'}")
    print(f"adjfine={rate_str(args.adjfine_hz)}  tx={rate_str(args.tx_hz)}  "
          f"duration={args.duration:g}s")
    print(f"baseline {dict(base)}")
    print()

    stop, stats = threading.Event(), {}
    threads = []
    if args.adjfine_hz != 0:
        threads.append(threading.Thread(target=adjfine_loop,
                                        args=(phc_fd, args.adjfine_hz, stop, stats)))
    else:
        print("  adjfine loop NOT started (--adjfine-hz 0) — this is arm A")
    if args.tx_hz != 0:
        threads.append(threading.Thread(target=tx_loop,
                                        args=(args.iface, args.tx_hz, stop, stats)))
    for t in threads:
        t.daemon = True
        t.start()

    csv = None
    if args.csv:
        csv = open(args.csv, "w", buffering=1)
        csv.write(f"# igc_tx_timeout_repro label={args.label}\n")
        csv.write(f"# iface={args.iface} phc={args.ptp}\n")
        csv.write(f"# driver_srcversion={srcver}\n")
        csv.write(f"# adjfine_hz={rate_str(args.adjfine_hz)} "
                  f"tx_hz={rate_str(args.tx_hz)}\n")
        csv.write(f"# started_unix={time.time():.3f}\n")
        csv.write("elapsed_s,unix_time,tx_hwtstamp_timeouts,tx_hwtstamp_skipped,"
                  "rx_hwtstamp_cleared,d_timeouts,extts_events,read_ok,"
                  "driver_srcversion\n")

    dmesg_base = (dmesg_timeout_times() or [0.0])[-1] + 1e-6
    t_start = time.monotonic()
    prev = base.get("tx_hwtstamp_timeouts", 0)
    first_timeout_at = None
    first_extts_gap_at = None
    event_times = []      # monotonic seconds at which timeouts were observed
    last_extts_seen = t_start

    try:
        while time.monotonic() - t_start < args.duration:
            time.sleep(1.0)
            now = time.monotonic()
            el = now - t_start
            c = read_counters(args.iface)
            read_ok = "tx_hwtstamp_timeouts" in c
            cur = c.get("tx_hwtstamp_timeouts", prev)
            d = cur - prev
            ev = extts.drain() if extts else None

            if ev:
                last_extts_seen = now
            elif extts and extts.ok and first_extts_gap_at is None \
                    and now - last_extts_seen > 3.0:
                first_extts_gap_at = el

            if d > 0:
                if first_timeout_at is None:
                    first_timeout_at = el
                    print(f"\n*** first tx_hwtstamp_timeout at {el:.1f}s "
                          f"(+{d}) ***")
                event_times.extend([el] * d)

            if csv:
                csv.write(f"{el:.3f},{time.time():.3f},{cur},"
                          f"{c.get('tx_hwtstamp_skipped','')},"
                          f"{c.get('rx_hwtstamp_cleared','')},{d},"
                          f"{'' if ev is None else ev},{int(read_ok)},{srcver}\n")
            prev = cur
            print(f"  {el:6.0f}s  timeouts={cur:<6d} d={d:<3d}"
                  f"{'' if ev is None else f'  extts={ev}'}", end="\r")
    except KeyboardInterrupt:
        print("\ninterrupted")

    stop.set()
    for t in threads:
        t.join(timeout=2)
    if csv:
        csv.close()
    os.close(phc_fd)

    el = time.monotonic() - t_start
    final = read_counters(args.iface)
    n_to = final.get("tx_hwtstamp_timeouts", 0) - base.get("tx_hwtstamp_timeouts", 0)
    adj_n, tx_n = stats.get("adjfine_count", 0), stats.get("tx_count", 0)

    print(f"\n\n=== {args.label or 'run'} — {el:.1f}s ===")
    print(f"driver srcversion : {srcver}")
    print(f"adjfine calls     : {adj_n} ({adj_n/el:.1f}/s)")
    print(f"TX packets        : {tx_n} ({tx_n/el:.1f}/s), "
          f"{stats.get('tx_errors', 0)} send errors")
    print(f"tx_hwtstamp_timeouts : {n_to} over {el:.0f}s")
    if extts:
        print(f"EXTTS events      : {extts.count}"
              f"{'' if extts.ok else ' (channel not enabled)'}")
        if first_extts_gap_at is not None:
            print(f"EXTTS first >3s gap at {first_extts_gap_at:.1f}s "
                  f"(first timeout at "
                  f"{'n/a' if first_timeout_at is None else f'{first_timeout_at:.1f}s'})")

    if n_to == 0:
        print("\nNo TX-timestamp timeouts observed.")
        print("NOTE: absence over one window is not an MTBF.  Report the "
              "window, not a derived rate.")
        sys.exit(0)

    # Inter-event structure is the H1 discriminator: orphaned-slot bursts
    # should arrive in clusters of up to 4 inside one 15 s window, not as
    # isolated singles.
    print(f"\nfirst timeout at  : {first_timeout_at:.1f}s (run-relative)")

    kt = [t for t in dmesg_timeout_times() if t >= dmesg_base]
    if not kt:
        print("no kernel-timestamped events found in dmesg — cannot judge H1 "
              "from counters alone (1 Hz sampling cannot resolve a burst).")
        sys.exit(1)

    bursts = burst_report(kt)
    sizes = [len(b) for b in bursts]
    print(f"kernel events     : {len(kt)} in {len(bursts)} burst(s), "
          f"sizes {sizes}")
    for b in bursts:
        span_us = (b[-1] - b[0]) * 1e6
        print(f"  t={b[0]:.6f}  n={len(b)}  span={span_us:.0f} us")
    if len(bursts) > 1:
        inter = [b[0] - a[-1] for a, b in zip(bursts, bursts[1:])]
        print(f"inter-burst gaps  : "
              + ", ".join(f"{g:.2f}s" for g in inter))

    # H1: four slots invalidated together -> bursts of up to 4, microseconds
    # wide.  Isolated singles spread out would refute it.
    clustered = sum(1 for n in sizes if n > 1)
    if clustered and max(sizes) <= 4:
        print(f"\nVERDICT: {clustered}/{len(bursts)} bursts have n>1, "
              f"max n={max(sizes)} (<= 4 slots) -> SUPPORTS H1 "
              f"(orphaned-slot invalidation)")
    elif max(sizes) == 1:
        print("\nVERDICT: all events isolated singles -> REFUTES H1")
    else:
        print(f"\nVERDICT: ambiguous — burst sizes {sizes}")

    if len(kt) < 5:
        print("\nfewer than 5 events — report counts and window, NOT an MTBF.")
    sys.exit(1)


if __name__ == "__main__":
    main()

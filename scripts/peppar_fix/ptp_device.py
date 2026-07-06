"""PTP hardware clock interface via Linux ioctls."""

import array
import ctypes
import ctypes.util
import fcntl
import logging
import os
import select
import struct
import time

log = logging.getLogger(__name__)


def _is_igc_driver(ptp_path: str) -> bool:
    """Return True if this PTP device is backed by the igc driver."""
    try:
        phc_basename = os.path.basename(ptp_path)
        if not phc_basename.startswith("ptp"):
            return False
        sys_path = f"/sys/class/ptp/{phc_basename}/device/driver"
        driver_name = os.path.basename(os.readlink(sys_path))
    except (OSError, ValueError):
        return False
    return driver_name == "igc"


def _stock_igc_freq_mode_workaround_needed(ptp_path: str) -> bool:
    """Detect when this PTP device is on a stock (unpatched) igc driver.

    Stock igc has a special case in its PEROUT request handler:
    when the requested period is exactly 1_000_000_000 ns (1 Hz), it
    drops the request into hardware frequency mode and ignores the
    requested start time entirely.  Frequency mode picks its own phase
    reference from the i226's internal counter, so the resulting 1 PPS
    output ends up at an arbitrary offset (often 500 ms) from the
    actual GPS second — disastrous for any tool consuming the PEROUT
    edge as a 1 PPS reference.

    The TimeHAT igc patch series (drivers/igc-timehat-edge/) removes
    this special case so 1 Hz uses Target Time mode like every other
    rate, AND adds the ``edge_check_delay_us`` module parameter as a
    side effect of its rising-edge filter.  We use the presence of
    that module parameter as a runtime fingerprint of the patched
    module — much more reliable than version sniffing.

    Returns True only when:
      1. The PTP device is backed by the igc driver, AND
      2. /sys/module/igc/parameters/edge_check_delay_us does not exist
         (indicating the stock module is loaded).

    On any other driver (ice/E810, e1000e, macb, etc.) this returns
    False because the bug doesn't apply — those drivers handle 1 Hz
    PEROUT correctly.  On the patched igc module this returns False
    because there's nothing to work around.

    The detection is intentionally conservative: if anything in the
    sysfs walk fails (unexpected layout, permissions, missing files)
    we return False and use the unmodified period.  Better to leave
    the period correct on an unknown system than to apply a workaround
    that might silently introduce drift.
    """
    try:
        phc_basename = os.path.basename(ptp_path)
        if not phc_basename.startswith("ptp"):
            return False
        # /sys/class/ptp/ptpN/device → PCI device → driver symlink
        sys_path = f"/sys/class/ptp/{phc_basename}/device/driver"
        driver_name = os.path.basename(os.readlink(sys_path))
    except (OSError, ValueError):
        return False
    if driver_name != "igc":
        return False
    return not os.path.exists(
        "/sys/module/igc/parameters/edge_check_delay_us"
    )


class DualEdgeFilter:
    """Suppress duplicate EXTTS events from NICs that timestamp both edges.

    The Intel i226 (and i210/i225 — they share the timing core) reports
    a hardware EXTTS event for *both* the rising and falling edges of any
    PPS signal on the EXTTS pin.  See the i226 dual-edge quirk note in
    CLAUDE.md and the kernel-patch discussion at
    https://github.com/Time-Appliances-Project/TimeHAT.

    With a typical GPS receiver emitting a 100 ms wide PPS pulse at 1 Hz,
    the engine sees 2 events per second 100 ms apart.  Without filtering,
    the engine treats whichever edge it picks as "the PPS edge" for that
    epoch — when it picks the falling edge it computes a phase error 100 ms
    larger than the truth and the servo demands an absurd correction.

    This filter is *purely temporal*: any event closer than ``min_spacing_s``
    to the previous accepted event is dropped.  No assumption about PHC
    alignment, second boundaries, pulse width, or oscillator state — works
    during cold-start bootstrap, during steady-state servo, and across
    arbitrary GPS receiver pulse-width configurations up to ~400 ms.

    Default ``min_spacing_s`` of 0.4 s is chosen to:
    - safely reject the falling edge of pulses up to ~400 ms wide,
    - safely accept the rising edge of the next 1 Hz period (1.0 - 0.4 = 0.6 s
      margin),
    - leave headroom for jitter and clock skew during bootstrap.

    For PPS rates other than 1 Hz, instantiate with a smaller spacing
    (e.g. ``min_spacing_s=0.04`` for 10 Hz).  The filter has no concept of
    "expected rate" beyond this minimum.
    """

    def __init__(self, min_spacing_s: float = 0.4):
        self.min_spacing_s = min_spacing_s
        self._last_phc_s: float | None = None
        self.dropped: int = 0
        self.accepted: int = 0

    def accept(self, phc_sec: int, phc_nsec: int) -> bool:
        """Return True if the (phc_sec, phc_nsec) event should be processed.

        Drops events whose PHC time is within ``min_spacing_s`` of the
        last accepted event's PHC time.  Updates internal state on accept.
        """
        ts = phc_sec + phc_nsec * 1e-9
        if self._last_phc_s is not None and (ts - self._last_phc_s) < self.min_spacing_s:
            self.dropped += 1
            return False
        self._last_phc_s = ts
        self.accepted += 1
        return True

    def reset(self) -> None:
        """Forget the previous accepted event (e.g. after a PHC step)."""
        self._last_phc_s = None


# ── PTP ioctl constants (from linux/ptp_clock.h) ─────────────────────── #

PTP_CLK_MAGIC = ord('=')

_IOC_WRITE = 1
_IOC_READ = 2


def _IOC(direction, typ, nr, size):
    return (direction << 30) | (size << 16) | (typ << 8) | nr


def _IOR(typ, nr, size):
    return _IOC(_IOC_READ, typ, nr, size)


def _IOW(typ, nr, size):
    return _IOC(_IOC_WRITE, typ, nr, size)


PTP_EXTTS_REQUEST = _IOW(PTP_CLK_MAGIC, 2, 16)
PTP_EXTTS_REQUEST2 = _IOW(PTP_CLK_MAGIC, 11, 16)
PTP_PEROUT_REQUEST = _IOW(PTP_CLK_MAGIC, 3, 56)
PTP_PEROUT_REQUEST2 = _IOW(PTP_CLK_MAGIC, 12, 56)  # same struct size as REQUEST1
PTP_EXTTS_EVENT_SIZE = 32
PTP_CLOCK_GETCAPS = _IOR(PTP_CLK_MAGIC, 1, 80)
PTP_PIN_SETFUNC = _IOW(PTP_CLK_MAGIC, 7, 96)

PTP_ENABLE_FEATURE = (1 << 0)
PTP_RISING_EDGE = (1 << 1)

PTP_PF_NONE = 0
PTP_PF_EXTTS = 1
PTP_PF_PEROUT = 2

ADJ_FREQUENCY = 0x0002
ADJ_SETOFFSET = 0x0100
ADJ_NANO = 0x2000


def _clock_id_from_fd(fd):
    """Encode PTP device fd as clockid_t for clock_adjtime."""
    return (~fd << 3) | 3


class PtpDevice:
    """Low-level interface to a Linux PTP hardware clock."""

    def __init__(self, dev_path="/dev/ptp0"):
        self.path = dev_path
        self.fd = os.open(dev_path, os.O_RDWR)
        # Kernel-enforced exclusive open — prevents other processes from
        # opening the same PTP device.  Automatically released when the
        # fd is closed, even on crash or SIGKILL.  No stale lock files.
        try:
            TIOCEXCL = 0x540C
            fcntl.ioctl(self.fd, TIOCEXCL)
        except OSError:
            pass  # Not all PTP devices support TIOCEXCL; proceed anyway
        self.clock_id = _clock_id_from_fd(self.fd)
        self._libc = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)

    def close(self):
        os.close(self.fd)

    def get_caps(self):
        """Query PTP clock capabilities."""
        buf = array.array('b', b'\x00' * 80)
        fcntl.ioctl(self.fd, PTP_CLOCK_GETCAPS, buf, True)
        raw = buf.tobytes()
        max_adj = struct.unpack_from('<i', raw, 0)[0]
        n_alarm = struct.unpack_from('<i', raw, 4)[0]
        n_ext_ts = struct.unpack_from('<i', raw, 8)[0]
        n_per_out = struct.unpack_from('<i', raw, 12)[0]
        pps = struct.unpack_from('<i', raw, 16)[0]
        n_pins = struct.unpack_from('<i', raw, 20)[0]
        return {
            'max_adj': max_adj,
            'n_alarm': n_alarm,
            'n_ext_ts': n_ext_ts,
            'n_per_out': n_per_out,
            'pps': pps,
            'n_pins': n_pins,
        }

    def set_pin_function(self, pin_index, func, channel):
        """Configure an SDP pin (EXTTS, PEROUT, or NONE)."""
        buf = bytearray(96)
        struct.pack_into('<64sIII', buf, 0, b'', pin_index, func, channel)
        fcntl.ioctl(self.fd, PTP_PIN_SETFUNC, bytes(buf))

    def enable_extts(self, channel, rising_edge=True):
        """Enable external timestamp capture on a channel."""
        flags = PTP_ENABLE_FEATURE
        if rising_edge:
            flags |= PTP_RISING_EDGE
        buf = struct.pack('<IIII', channel, flags, 0, 0)
        try:
            fcntl.ioctl(self.fd, PTP_EXTTS_REQUEST2, buf)
        except OSError:
            fcntl.ioctl(self.fd, PTP_EXTTS_REQUEST, buf)

    def disable_extts(self, channel):
        """Disable external timestamp capture."""
        buf = struct.pack('<IIII', channel, 0, 0, 0)
        try:
            fcntl.ioctl(self.fd, PTP_EXTTS_REQUEST2, buf)
        except OSError:
            fcntl.ioctl(self.fd, PTP_EXTTS_REQUEST, buf)

    def enable_perout(self, channel, period_ns=1_000_000_000,
                      start_nsec_override=None):
        """Enable periodic output (1PPS) on a channel.

        Kernel struct ptp_perout_request (56 bytes):
          ptp_clock_time start;   // {s64 sec, u32 nsec, u32 reserved}
          ptp_clock_time period;  // {s64 sec, u32 nsec, u32 reserved}
          u32 index;
          u32 flags;
          ptp_clock_time on;      // {s64 sec, u32 nsec, u32 reserved}

        Start at an upcoming PHC second boundary (nsec=0).  The igc
        driver fires the first pulse AT start (it does not add any
        offset).  Verified empirically 2026-04-06 by setting different
        start_nsec values and observing the TICC chA timestamp shift
        by exactly the same amount.

        Use PTP_PEROUT_DUTY_CYCLE flag with 1 ms ON time so the pulse
        is unambiguously a short rising-edge event.

        Stock-igc 1 Hz frequency-mode workaround: when running against
        a stock (unpatched) Intel igc driver, the kernel intercepts a
        period of exactly 1_000_000_000 ns and silently switches the
        i226 hardware into frequency mode, which ignores our start
        time and produces an output phase-shifted by an arbitrary
        amount (typically ~500 ms) from the actual GPS second.  When
        we detect that situation we nudge the requested period by 1 ns
        to dodge the special-case `==` and force Target Time mode,
        which respects start time exactly.  The resulting 1 PPB period
        offset between PEROUT and the PHC is undetectable for our
        servo and worth the trade for a correctly-phased pulse.

        On the patched TimeHAT igc, on E810 (ice driver), and on any
        other PHC the period is left at exactly 1_000_000_000 ns.
        """
        if (period_ns == 1_000_000_000
                and _stock_igc_freq_mode_workaround_needed(self.path)):
            log.warning(
                "Stock igc detected on %s: nudging PEROUT period "
                "1_000_000_000 → 999_999_999 ns to dodge the 1 Hz "
                "frequency-mode bug. Install the TimeHAT igc patch "
                "(drivers/igc-timehat-edge/) for the proper fix.",
                self.path,
            )
            period_ns = 999_999_999
        period_s = period_ns // 1_000_000_000
        period_sub = period_ns % 1_000_000_000
        phc_ns, _sys_ns = self.read_phc_ns()
        start_sec = phc_ns // 1_000_000_000 + 2
        if start_nsec_override is not None:
            start_nsec = start_nsec_override
        elif _is_igc_driver(self.path):
            # igc uses 50% duty cycle and the start time specifies the
            # falling edge (start of LOW).  To align the rising edge
            # with the top of the PHC second, offset by half the period.
            # Discovered via SatPulse (jclark/satpulse) which applies
            # the same correction.  Without this, the rising edge fires
            # at 500 ms into the second on all igc hardware.
            start_nsec = period_ns // 2
            log.info("igc detected on %s: setting PEROUT start_nsec=%d "
                     "(half-period offset for rising-edge alignment)",
                     self.path, start_nsec)
        else:
            start_nsec = 0
        # Disable first and wait — on igc, re-enabling over an active
        # PEROUT can latch to the old phase, ignoring start_nsec.
        # A full disable-wait-enable cycle forces the hardware to
        # re-read the start time.
        self.disable_perout(channel)
        import time as _time
        _time.sleep(1)
        # Use PTP_PEROUT_REQUEST2 with flags=0 (no PTP_PEROUT_DUTY_CYCLE).
        # igc uses 50% duty cycle regardless of the duty cycle flag, and
        # some kernels reject REQUEST2 with duty cycle on CM4/CM5.
        # SatPulse also uses flags=0 for igc/igb.
        buf = struct.pack('<qII qII II qII',
                          start_sec, start_nsec, 0,
                          period_s, period_sub, 0,
                          channel, 0,
                          0, 0, 0)
        try:
            fcntl.ioctl(self.fd, PTP_PEROUT_REQUEST2, buf)
        except OSError:
            # Fall back to REQUEST1 if REQUEST2 not supported
            fcntl.ioctl(self.fd, PTP_PEROUT_REQUEST, buf)

    def disable_perout(self, channel):
        """Disable periodic output on a channel."""
        buf = struct.pack('<qII qII II qII',
                          0, 0, 0,
                          0, 0, 0,
                          channel, 0,
                          0, 0, 0)
        try:
            fcntl.ioctl(self.fd, PTP_PEROUT_REQUEST2, buf)
        except OSError:
            fcntl.ioctl(self.fd, PTP_PEROUT_REQUEST, buf)

    def read_extts(self, timeout_ms=1500):
        """Read one external timestamp event.

        Returns (sec, nsec, index, recv_mono, queue_remains, parse_age_s) or None.
        """
        r, _, _ = select.select([self.fd], [], [], timeout_ms / 1000.0)
        if not r:
            return None
        data = os.read(self.fd, PTP_EXTTS_EVENT_SIZE)
        recv_mono = time.monotonic()
        if len(data) < 20:
            return None
        sec, nsec, _reserved, index = struct.unpack_from('<qIII', data, 0)
        r_more, _, _ = select.select([self.fd], [], [], 0.0)
        return (sec, nsec, index, recv_mono, bool(r_more), 0.0)

    def read_extts_dedup(self, dedup, timeout_ms=1500):
        """read_extts() with a DualEdgeFilter applied to suppress falling edges.

        Calls read_extts in a loop until an event passes the filter or the
        deadline is reached.  Returns the same tuple shape as read_extts(),
        or None on timeout.

        Note for one-shot use: if ``dedup`` has no prior state, the very
        first event is always accepted regardless of which edge it is.
        For one-shot reads where you need a confirmed rising edge (e.g.
        bootstrap phase verification), use ``read_one_rising_edge`` instead.
        """
        deadline = time.monotonic() + timeout_ms / 1000.0
        while True:
            remaining_ms = int((deadline - time.monotonic()) * 1000)
            if remaining_ms <= 0:
                return None
            ev = self.read_extts(timeout_ms=remaining_ms)
            if ev is None:
                return None
            if dedup.accept(ev[0], ev[1]):
                return ev
            # rejected — loop and try the next event

    def read_one_rising_edge(self, channel_unused=None,
                             min_spacing_s: float = 0.4,
                             collection_window_s: float = 1.3,
                             timeout_s: float = 3.0):
        """One-shot read that guarantees the returned event is a rising edge.

        Reads EXTTS events for at least ``collection_window_s`` (default 1.3 s,
        slightly longer than one PPS period at 1 Hz) so that *at least one*
        full PPS period passes through the filter, then returns the *last*
        accepted event.  After one full period the last accepted event is
        always the leading edge of a pair (a rising edge), regardless of
        whether the first event happened to land on a falling edge.

        Returns ``(sec, nsec, index, recv_mono, queue_remains, parse_age_s)``
        on success, or ``None`` if no events arrive within ``timeout_s``.

        EXTTS must already be enabled on the relevant channel before calling.
        """
        dedup = DualEdgeFilter(min_spacing_s=min_spacing_s)
        deadline_total = time.monotonic() + timeout_s
        first_accept_mono = None
        last_accepted = None
        while time.monotonic() < deadline_total:
            remaining_total_ms = int((deadline_total - time.monotonic()) * 1000)
            if remaining_total_ms <= 0:
                break
            ev = self.read_extts(timeout_ms=min(2000, remaining_total_ms))
            if ev is None:
                if last_accepted is None:
                    return None  # nothing came; bubble up the timeout
                break
            if dedup.accept(ev[0], ev[1]):
                if first_accept_mono is None:
                    first_accept_mono = time.monotonic()
                last_accepted = ev
                # Once we've held a stable accepted event for at least one
                # full period, the last one is provably a rising edge.
                if time.monotonic() - first_accept_mono >= collection_window_s:
                    break
        return last_accepted

    def read_adjfine(self):
        """Read current PHC frequency adjustment in ppb."""
        timex_size = 208
        buf = ctypes.create_string_buffer(timex_size)
        # modes=0: read-only query
        ret = self._libc.clock_adjtime(
            ctypes.c_int32(self.clock_id),
            buf,
        )
        if ret < 0:
            errno = ctypes.get_errno()
            raise OSError(errno, f"clock_adjtime read failed: {os.strerror(errno)}")
        freq = struct.unpack_from('<q', buf.raw, 16)[0]
        return freq / 65.536

    def adjfine(self, ppb):
        """Adjust PHC frequency by ppb (parts per billion)."""
        import errno as errno_mod
        freq = int(ppb * 65.536)
        timex_size = 208
        buf = bytearray(timex_size)
        struct.pack_into('<I', buf, 0, ADJ_FREQUENCY)
        struct.pack_into('<q', buf, 16, freq)
        # Retry on EBUSY — the patched igc driver returns EBUSY when
        # adjfine races with a pending TX timestamp.
        for _ in range(50):
            ret = self._libc.clock_adjtime(
                ctypes.c_int32(self.clock_id),
                ctypes.c_char_p(bytes(buf)),
            )
            if ret >= 0:
                return ppb
            err = ctypes.get_errno()
            if err != errno_mod.EBUSY:
                raise OSError(err, f"clock_adjtime failed: {os.strerror(err)}")
            time.sleep(0.0001)  # 100µs — TX timestamp completes in ~1ms
        raise OSError(errno_mod.EBUSY, "clock_adjtime: EBUSY after 50 retries")

    def step_time(self, offset_ns):
        """Step the PHC by offset_ns nanoseconds using ADJ_SETOFFSET."""
        sec = int(offset_ns // 1_000_000_000)
        nsec = int(offset_ns % 1_000_000_000)
        if nsec < 0:
            sec -= 1
            nsec += 1_000_000_000
        timex_size = 208
        buf = bytearray(timex_size)
        struct.pack_into('<I', buf, 0, ADJ_SETOFFSET | ADJ_NANO)
        struct.pack_into('<q', buf, 72, sec)
        struct.pack_into('<q', buf, 80, nsec)
        ret = self._libc.clock_adjtime(
            ctypes.c_int32(self.clock_id),
            ctypes.c_char_p(bytes(buf)),
        )
        if ret < 0:
            errno = ctypes.get_errno()
            raise OSError(errno, f"clock_adjtime ADJ_SETOFFSET failed: {os.strerror(errno)}")

    # ── PTP_SYS_OFFSET: cross-timestamp PHC vs system clock ───────── #

    _PTP_SYS_OFFSET_SIZE = 832  # 16-byte header + 51 * 16-byte timestamps
    _PTP_SYS_OFFSET = None  # resolved on first call

    def _resolve_sys_offset_ioctl(self):
        """Try PTP_SYS_OFFSET2 then PTP_SYS_OFFSET."""
        PTP_CLK = ord('=')
        _IOC_W = 1
        def iow(nr, sz):
            return (_IOC_W << 30) | (sz << 16) | (PTP_CLK << 8) | nr
        for nr in (14, 5):  # SYS_OFFSET2 then SYS_OFFSET
            ioctl_nr = iow(nr, self._PTP_SYS_OFFSET_SIZE)
            buf = bytearray(self._PTP_SYS_OFFSET_SIZE)
            struct.pack_into('<I', buf, 0, 3)  # n_samples=3
            try:
                fcntl.ioctl(self.fd, ioctl_nr, buf, True)
                self._PTP_SYS_OFFSET = ioctl_nr
                return
            except OSError:
                continue
        raise OSError("PTP_SYS_OFFSET not supported on this device")

    def read_phc_ns(self, n_samples=5):
        """Read PHC time cross-referenced to system clock.

        Uses PTP_SYS_OFFSET to get interleaved sys/PHC/sys timestamps,
        picks the tightest triplet. Returns (phc_ns, sys_ns).
        """
        if self._PTP_SYS_OFFSET is None:
            self._resolve_sys_offset_ioctl()
        buf = bytearray(self._PTP_SYS_OFFSET_SIZE)
        struct.pack_into('<I', buf, 0, n_samples)
        fcntl.ioctl(self.fd, self._PTP_SYS_OFFSET, buf, True)

        def _ts(offset):
            sec = struct.unpack_from('<q', buf, offset)[0]
            nsec = struct.unpack_from('<I', buf, offset + 8)[0]
            return sec * 1_000_000_000 + nsec

        best_span = float('inf')
        best_phc = 0
        best_sys = 0
        for i in range(n_samples):
            base = 16 + i * 32  # each pair is sys(16) + phc(16)
            sys_before = _ts(base)
            phc = _ts(base + 16)
            sys_after = _ts(base + 32) if i < n_samples - 1 else _ts(16 + (2 * n_samples) * 16)
            # Actually interleaved: ts[0]=sys, ts[1]=phc, ts[2]=sys, ts[3]=phc, ..., ts[2n]=sys
            pass

        # Re-parse correctly: timestamps are at 16-byte intervals after 16-byte header
        # Layout: sys0, phc0, sys1, phc1, ..., phcN-1, sysN  (2*n_samples + 1 entries)
        timestamps = []
        for i in range(2 * n_samples + 1):
            timestamps.append(_ts(16 + i * 16))

        for i in range(n_samples):
            sys_before = timestamps[2 * i]
            phc = timestamps[2 * i + 1]
            sys_after = timestamps[2 * i + 2]
            span = sys_after - sys_before
            if span < best_span:
                best_span = span
                best_phc = phc
                best_sys = (sys_before + sys_after) // 2

        return best_phc, best_sys

    def set_phc_ns(self, time_ns):
        """Set the PHC to an absolute time in nanoseconds."""
        sec = int(time_ns // 1_000_000_000)
        nsec = int(time_ns % 1_000_000_000)

        class Timespec(ctypes.Structure):
            _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]

        ts = Timespec(sec, nsec)
        ret = self._libc.clock_settime(
            ctypes.c_int32(self.clock_id), ctypes.byref(ts))
        if ret != 0:
            errno = ctypes.get_errno()
            raise OSError(errno, f"clock_settime failed: {os.strerror(errno)}")

    def adj_setoffset(self, offset_ns):
        """Apply a relative phase step via clock_adjtime(ADJ_SETOFFSET).

        Much more precise than set_phc_ns (clock_settime) because the
        offset is relative — systematic read latency cancels.  On E810,
        this gives ±2 ns residual vs ±20 ms for clock_settime.
        """
        ADJ_SETOFFSET = 0x0100
        ADJ_NANO = 0x2000

        # Handle negative offsets: kernel expects tv_sec and tv_nsec
        # with the same sign, or tv_sec=-1 for sub-second negative.
        if offset_ns >= 0:
            sec = int(offset_ns // 1_000_000_000)
            nsec = int(offset_ns % 1_000_000_000)
        else:
            # For negative: sec is floor-toward-negative-infinity,
            # nsec is the remainder (always 0..999999999).
            abs_ns = -offset_ns
            sec = -(int(abs_ns // 1_000_000_000))
            nsec = -(int(abs_ns % 1_000_000_000))
            if nsec < 0:
                sec -= 1
                nsec += 1_000_000_000

        class Timeval(ctypes.Structure):
            _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]

        class Timex(ctypes.Structure):
            _fields_ = [
                ("modes", ctypes.c_uint),
                ("offset", ctypes.c_long),
                ("freq", ctypes.c_long),
                ("maxerror", ctypes.c_long),
                ("esterror", ctypes.c_long),
                ("status", ctypes.c_int),
                ("constant", ctypes.c_long),
                ("precision", ctypes.c_long),
                ("tolerance", ctypes.c_long),
                ("time", Timeval),
                ("tick", ctypes.c_long),
                ("ppsfreq", ctypes.c_long),
                ("jitter", ctypes.c_long),
                ("shift", ctypes.c_int),
                ("stabil", ctypes.c_long),
                ("jitcnt", ctypes.c_long),
                ("calcnt", ctypes.c_long),
                ("errcnt", ctypes.c_long),
                ("stbcnt", ctypes.c_long),
                ("tai", ctypes.c_int),
            ]

        tx = Timex()
        tx.modes = ADJ_SETOFFSET | ADJ_NANO
        tx.time.tv_sec = sec
        tx.time.tv_usec = nsec  # nsec field when ADJ_NANO set

        ret = self._libc.clock_adjtime(
            ctypes.c_int32(self.clock_id), ctypes.byref(tx))
        if ret < 0:
            errno = ctypes.get_errno()
            raise OSError(errno, f"clock_adjtime(ADJ_SETOFFSET) failed: {os.strerror(errno)}")

    def step_relative(self, target_ns, pps_anchor_ns=None, pps_realtime_ns=None):
        """Step the PHC using ADJ_SETOFFSET (relative, single-shot).

        Reads current PHC time, computes the error relative to target,
        and applies the correction as a relative offset.  No lag
        compensation needed — systematic read latency cancels.

        Returns (residual_ns, attempts=1, accepted=True).
        """
        if pps_anchor_ns is not None:
            rt_now = time.clock_gettime_ns(time.CLOCK_REALTIME)  # wallclock-todo: PHC-bootstrap CLOCK_REALTIME transfer standard; staged review
            target_ns = pps_anchor_ns + (rt_now - pps_realtime_ns)

        phc_now, sys_now = self.read_phc_ns()
        if pps_anchor_ns is not None:
            expected = pps_anchor_ns + (sys_now - pps_realtime_ns)
        else:
            expected = target_ns
        error = phc_now - expected

        self.adj_setoffset(-error)

        # Verify
        phc_after, sys_after = self.read_phc_ns()
        if pps_anchor_ns is not None:
            expected_after = pps_anchor_ns + (sys_after - pps_realtime_ns)
        else:
            expected_after = target_ns
        residual = phc_after - expected_after
        return residual, 1, True

    def step_to(self, target_ns=0, phc_optimal_stop_limit_s=1.0,
                phc_settime_lag_ns=0,
                pps_anchor_ns=None, pps_realtime_ns=None):
        """Step the PHC to a target time using optimal stopping.

        Parametric optimal stopping (secretary problem variant):
        Observe first 1/e (~37%) of the search budget, tracking the
        best |readback residual|.  Then accept the first attempt that
        equals or beats the 5th percentile of observations.

        Self-adapts to any PHC — no prior characterization of step
        error distribution needed.  Works with tight i226 latency
        and bimodal E810 latency alike.

        PPS-anchored target (pps_anchor_ns + pps_realtime_ns): the PHC
        should read pps_anchor_ns at the moment CLOCK_REALTIME was
        pps_realtime_ns.  Each iteration recomputes the target using
        CLOCK_REALTIME as a transfer standard.

        phc_settime_lag_ns: mean clock_settime-to-PHC landing delay.
        The aim includes this lag so the PHC reads the correct time
        at the moment the write completes.

        Returns (residual_ns, attempts, accepted).
        """
        import math
        deadline = time.monotonic() + phc_optimal_stop_limit_s
        observe_until = time.monotonic() + phc_optimal_stop_limit_s / math.e
        attempts = 0
        observe_samples = []
        observing = True
        residual_ns = 0

        while time.monotonic() < deadline:
            attempts += 1
            if pps_anchor_ns is not None:
                rt_now = time.clock_gettime_ns(time.CLOCK_REALTIME)  # wallclock-todo: PHC-bootstrap CLOCK_REALTIME transfer standard; staged review
                target_ns = pps_anchor_ns + (rt_now - pps_realtime_ns)
            aim_ns = target_ns + phc_settime_lag_ns
            self.set_phc_ns(aim_ns)
            phc_after, sys_at_read = self.read_phc_ns()

            if pps_anchor_ns is not None:
                expected_ns = pps_anchor_ns + (sys_at_read - pps_realtime_ns)
                residual_ns = phc_after - expected_ns
            else:
                residual_ns = phc_after - target_ns

            if observing:
                observe_samples.append(abs(residual_ns))
                if time.monotonic() >= observe_until:
                    observing = False
                    observe_samples.sort()
                    idx = max(0, len(observe_samples) * 5 // 100 - 1)
                    threshold = observe_samples[idx]
            else:
                if abs(residual_ns) <= threshold:
                    return residual_ns, attempts, True

        return residual_ns, attempts, False

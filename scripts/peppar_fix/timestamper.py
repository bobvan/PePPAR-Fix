"""Timestamper abstraction — unified interface for PPS edge measurement.

A timestamper is anything that measures PPS edges: EXTTS channels on
PHCs, or standalone instruments like a TICC.  This module provides an
ABC and two concrete implementations so that bootstrap frequency
measurement, servo setup, and noise characterization can be written
once against the interface.

See docs/state-persistence-design.md § Timestamper for the data model.
"""

import abc
import logging
import math
import statistics
import time

log = logging.getLogger(__name__)


class Timestamper(abc.ABC):
    """Anything that measures PPS edges and can estimate DO frequency."""

    @abc.abstractmethod
    def measure_pps_frequency(self, n_samples=10, timeout_s=20):
        """Capture PPS edges and compute the DO's frequency offset.

        Returns (freq_ppb, sigma_ppb, n_intervals) on success, or
        (None, None, 0) if not enough edges arrive.

        freq_ppb: measured frequency offset of DO relative to GNSS PPS.
                  Positive = DO is fast.
        sigma_ppb: per-interval residual jitter (1-sigma).
        n_intervals: number of PPS intervals used.
        """


class ExttsTimestamper(Timestamper):
    """Measures PPS edges via a PHC's EXTTS hardware timestamping.

    Wraps PtpDevice.  The existing phc_bootstrap.measure_pps_frequency
    logic is moved here.
    """

    def __init__(self, ptp, channel, pps_pin=None):
        self.ptp = ptp
        self.channel = channel
        self.pps_pin = pps_pin

    def measure_pps_frequency(self, n_samples=10, timeout_s=20):
        from peppar_fix.ptp_device import DualEdgeFilter, PTP_PF_EXTTS

        ptp = self.ptp
        channel = self.channel

        # i226 requires pin→function assignment before EXTTS enable.
        # Without it, enable_extts returns EBUSY.
        if self.pps_pin is not None:
            try:
                ptp.set_pin_function(self.pps_pin, PTP_PF_EXTTS, channel)
            except OSError:
                pass  # sysfs fallback or implicit mapping

        ptp.enable_extts(channel, rising_edge=True)
        samples = []   # (elapsed_sec, nsec)
        first_sec = None
        deadline = time.monotonic() + timeout_s
        dedup = DualEdgeFilter()

        for _ in range(n_samples + 1):
            if time.monotonic() > deadline:
                break
            remaining_ms = max(50, int((deadline - time.monotonic()) * 1000))
            event = ptp.read_extts_dedup(dedup, timeout_ms=min(2000, remaining_ms))
            if event is None:
                continue
            sec, nsec, _idx, _mono, _qr, _pa = event
            if first_sec is None:
                first_sec = sec
            samples.append((sec - first_sec, nsec))

        ptp.disable_extts(channel)
        if dedup.dropped:
            log.info("EXTTS freq measurement: dual-edge filter dropped %d events",
                     dedup.dropped)

        if len(samples) < 2:
            return None, None, 0

        n_intervals = samples[-1][0]
        if n_intervals <= 0:
            return None, None, 0

        nsec_drift = samples[-1][1] - samples[0][1]
        if nsec_drift > 500_000_000:
            nsec_drift -= 1_000_000_000
            n_intervals += 1
        elif nsec_drift < -500_000_000:
            nsec_drift += 1_000_000_000
            n_intervals -= 1

        freq_ppb = nsec_drift / n_intervals

        if len(samples) >= 3:
            residuals = []
            for elapsed_s, nsec in samples[1:]:
                if elapsed_s <= 0:
                    continue
                predicted = samples[0][1] + freq_ppb * elapsed_s
                diff = nsec - predicted
                if diff > 500_000_000:
                    diff -= 1_000_000_000
                elif diff < -500_000_000:
                    diff += 1_000_000_000
                residuals.append(diff)
            sigma_ppb = statistics.stdev(residuals) if len(residuals) >= 2 else 0.0
        else:
            sigma_ppb = 0.0

        return freq_ppb, sigma_ppb, len(samples) - 1


class TiccTimestamper(Timestamper):
    """Measures PPS edges on a single TICC channel.

    A TICC is a two-channel time interval counter, but each channel is
    an independent timestamper — just like an EXTTS channel on a PHC.
    The TICC timestamps edges against its reference oscillator (a GPSDO
    OCXO on our lab TICCs).

    Successive timestamps on one channel give the PPS interval; the
    deviation from nominal 1e12 ps is the source's frequency offset
    relative to the TICC reference.  With a GPSDO reference, this is
    close to the absolute offset.

    If you need the frequency offset between two PPS sources (e.g.,
    DO vs GNSS), create two TiccTimestampers and subtract — or see
    ``measure_differential_frequency`` which handles the pairing.

    Resolution: ~60 ps single-shot noise.
    """

    def __init__(self, ticc_port, ticc_baud=115200, channel='chA'):
        self.ticc_port = ticc_port
        self.ticc_baud = ticc_baud
        self.channel = channel

    def measure_pps_frequency(self, n_samples=10, timeout_s=30):
        """Measure PPS frequency offset from successive single-channel intervals.

        Each pair of successive edges gives one interval.  The deviation
        from nominal (1 s = 1e12 ps) is the frequency offset in ppb.
        Linear regression over the full baseline gives better precision
        than averaging individual intervals.

        timeout_s default is 30 to allow for TICC boot (~10s) plus
        measurement (~n_samples seconds).
        """
        from ticc import Ticc

        edges = []  # (ref_sec, ref_ps)
        boot_discard = 2

        with Ticc(self.ticc_port, self.ticc_baud, wait_for_boot=True) as ticc:
            deadline = time.monotonic() + timeout_s
            for ch, ref_sec, ref_ps in ticc:
                if time.monotonic() > deadline:
                    break
                if ch != self.channel:
                    continue
                if ref_sec <= boot_discard:
                    continue
                edges.append((ref_sec, ref_ps))
                if len(edges) >= n_samples + 1:
                    break

        if len(edges) < 3:
            log.warning("TICC freq measurement (%s): only %d edges (need ≥3)",
                        self.channel, len(edges))
            return None, None, 0

        # Compute intervals in picoseconds
        intervals_ps = []
        for i in range(1, len(edges)):
            dt_sec = edges[i][0] - edges[i - 1][0]
            dt_ps = edges[i][1] - edges[i - 1][1]
            total_ps = dt_sec * 1_000_000_000_000 + dt_ps
            if total_ps > 0:
                intervals_ps.append(total_ps)

        if len(intervals_ps) < 2:
            return None, None, 0

        # Full-baseline frequency: total drift over N intervals.
        # Same approach as ExttsTimestamper — first-to-last, not averages.
        first = edges[0]
        last = edges[-1]
        total_sec = last[0] - first[0]
        total_ps_drift = (last[0] - first[0]) * 1_000_000_000_000 + (last[1] - first[1])
        nominal_ps = total_sec * 1_000_000_000_000
        if nominal_ps == 0:
            return None, None, 0

        freq_ppb = (total_ps_drift - nominal_ps) / nominal_ps * 1e9

        # Per-interval residual jitter
        nominal_1s_ps = 1_000_000_000_000
        residuals = [iv - nominal_1s_ps - (freq_ppb * 1000) for iv in intervals_ps]
        # freq_ppb * 1000 converts ppb to ps/interval for a 1-second nominal
        sigma_ps = statistics.stdev(residuals) if len(residuals) >= 2 else 0.0
        sigma_ppb = sigma_ps / nominal_1s_ps * 1e9

        log.info("TICC freq measurement (%s): %.1f ±%.1f ppb "
                 "(%d intervals, %ds baseline)",
                 self.channel, freq_ppb,
                 sigma_ppb / math.sqrt(max(1, len(intervals_ps))),
                 len(intervals_ps), total_sec)

        return freq_ppb, sigma_ppb, len(intervals_ps)


class TiccDifferentialTimestamper(Timestamper):
    """Measures DO-vs-reference frequency via paired TICC channels.

    Wraps ``measure_differential_frequency`` in the Timestamper interface
    so bootstrap code can call ``timestamper.measure_pps_frequency()``
    uniformly for both EXTTS and TICC paths.

    This is NOT a single-channel timestamper — it uses two channels and
    subtracts, cancelling the TICC reference oscillator's own drift.
    Use ``TiccTimestamper`` for single-channel measurements.
    """

    def __init__(self, ticc_port, ticc_baud=115200,
                 do_channel='chA', ref_channel='chB'):
        self.ticc_port = ticc_port
        self.ticc_baud = ticc_baud
        self.do_channel = do_channel
        self.ref_channel = ref_channel

    def measure_pps_frequency(self, n_samples=10, timeout_s=30):
        return measure_differential_frequency(
            self.ticc_port, self.ticc_baud,
            self.do_channel, self.ref_channel,
            n_samples, timeout_s)


def measure_differential_frequency(ticc_port, ticc_baud=115200,
                                   do_channel='chA', ref_channel='chB',
                                   n_samples=10, timeout_s=30):
    """Measure DO frequency offset relative to a reference PPS via TICC.

    Pairs timestamps on two TICC channels by ref_sec and computes the
    slope of the differential — cancels the TICC reference oscillator's
    own drift since both channels are timestamped against it within the
    same epoch.

    Returns (freq_ppb, sigma_ppb, n_pairs) or (None, None, 0).
    """
    from ticc import Ticc

    pairs = []  # (elapsed_sec, diff_ns)
    pending = {}  # ref_sec → {channel: (ref_sec, ref_ps)}
    first_ref_sec = None
    boot_discard = 5  # skip first 5s: TICC boot artifacts + post-ARM settling

    with Ticc(ticc_port, ticc_baud, wait_for_boot=True) as ticc:
        deadline = time.monotonic() + timeout_s
        for channel, ref_sec, ref_ps in ticc:
            if time.monotonic() > deadline:
                break
            if channel not in (do_channel, ref_channel):
                continue
            if ref_sec <= boot_discard:
                continue

            pending.setdefault(ref_sec, {})[channel] = (ref_sec, ref_ps)

            if (do_channel in pending.get(ref_sec, {}) and
                    ref_channel in pending.get(ref_sec, {})):
                do = pending[ref_sec][do_channel]
                ref = pending[ref_sec][ref_channel]
                del pending[ref_sec]

                diff_ps = ((do[0] - ref[0]) * 1_000_000_000_000
                           + do[1] - ref[1])
                diff_ns = diff_ps * 1e-3

                if first_ref_sec is None:
                    first_ref_sec = ref_sec
                elapsed = ref_sec - first_ref_sec
                pairs.append((elapsed, diff_ns))

                if len(pairs) >= n_samples:
                    break

            # Evict stale pending entries
            cutoff = ref_sec - 4
            for k in [k for k in pending if k < cutoff]:
                del pending[k]

    if len(pairs) < 3:
        log.warning("TICC differential: only %d pairs (need ≥3)", len(pairs))
        return None, None, 0

    # Linear regression: diff_ns = intercept + slope * elapsed
    # slope = frequency offset in ns/s = ppb
    n = len(pairs)
    sx = sum(t for t, _ in pairs)
    sy = sum(d for _, d in pairs)
    sxy = sum(t * d for t, d in pairs)
    sxx = sum(t * t for t, _ in pairs)
    denom = n * sxx - sx * sx
    if denom == 0:
        return None, None, 0

    slope = (n * sxy - sx * sy) / denom
    intercept = (sy - slope * sx) / n

    residuals = [d - (intercept + slope * t) for t, d in pairs]
    sigma = statistics.stdev(residuals) if len(residuals) >= 2 else 0.0

    n_intervals = pairs[-1][0] - pairs[0][0]
    if n_intervals <= 0:
        n_intervals = len(pairs) - 1

    # Negate: diff_ns = DO - GNSS, positive slope = DO PPS getting later
    # = DO is slow.  Negate so positive freq_ppb = DO is fast, matching
    # the EXTTS convention (positive = too fast = reduce adjfine).
    freq_ppb = -slope

    log.info("TICC differential freq (%s-%s): %+.1f ppb (±%.1f ppb, "
             "%d pairs, %ds baseline)",
             do_channel, ref_channel, freq_ppb,
             sigma / math.sqrt(max(1, n_intervals)),
             len(pairs), n_intervals)

    return freq_ppb, sigma, n_intervals


def measure_differential_phase(ticc_port, ticc_baud=115200,
                               do_channel='chA', ref_channel='chB',
                               n_samples=3, timeout_s=20):
    """Measure the median phase offset between DO PPS and ref PPS via TICC.

    Like measure_differential_frequency but returns just the median
    instantaneous |phase| — used by the warm-start gate to decide
    whether to step (via TADD ARM) or slew the divider.

    Returns (median_diff_ns, n_pairs) or (None, 0) on insufficient
    samples.  positive median_diff_ns = DO PPS is later than ref PPS.
    """
    from ticc import Ticc

    pairs = []
    pending = {}
    boot_discard = 5

    with Ticc(ticc_port, ticc_baud, wait_for_boot=True) as ticc:
        deadline = time.monotonic() + timeout_s
        for channel, ref_sec, ref_ps in ticc:
            if time.monotonic() > deadline:
                break
            if channel not in (do_channel, ref_channel):
                continue
            if ref_sec <= boot_discard:
                continue

            pending.setdefault(ref_sec, {})[channel] = (ref_sec, ref_ps)
            if (do_channel in pending.get(ref_sec, {}) and
                    ref_channel in pending.get(ref_sec, {})):
                do = pending[ref_sec][do_channel]
                ref = pending[ref_sec][ref_channel]
                del pending[ref_sec]
                diff_ps = ((do[0] - ref[0]) * 1_000_000_000_000
                           + do[1] - ref[1])
                pairs.append(diff_ps * 1e-3)
                if len(pairs) >= n_samples:
                    break

            cutoff = ref_sec - 4
            for k in [k for k in pending if k < cutoff]:
                del pending[k]

    if len(pairs) < 1:
        return None, 0
    median = statistics.median(pairs)
    return median, len(pairs)

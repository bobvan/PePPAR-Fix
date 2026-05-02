"""Two-phase engine-log consumer: replay then follow.

Why "reader" not "tailer": the monitor needs the engine's start time
(for uptime) and any state the engine has accumulated since startup.
That requires reading the log from the beginning before we can do
anything useful.  Once caught up, we continue following the file for
live updates — the same way ``tail -n +1 -f`` would.

The reader runs in its own daemon thread and exposes a thread-safe
``LogState`` snapshot.  Consumers (the Textual app) poll the state on
a timer — no callbacks, no event queue plumbing for the first pass.
When we add real state (SV-state histogram, NL fix counts, etc.) the
same pattern scales: the reader writes into the state dataclass,
readers read it.

Scope today: extract the engine's start time from the first
timestamped line.  Everything else is a hook for the next commit.
"""

from __future__ import annotations

import logging
import re
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

from peppar_mon._util import parse_log_timestamp

log = logging.getLogger(__name__)


# Poll interval for the follow phase.  Fast enough to feel live on a
# 1 Hz engine, slow enough that a quiet log doesn't spin a core.
_FOLLOW_POLL_S = 0.2

# Retry delay when the log file doesn't exist yet.  The engine might
# not have started or might be writing somewhere else; don't busy-loop.
_WAIT_FOR_FILE_S = 1.0


@dataclass
class LogState:
    """Thread-safe-ish snapshot of what the reader has inferred.

    The writing thread (LogReader) sets fields via simple attribute
    writes.  The reading thread (Textual app) reads them.  Python's
    GIL makes single-attribute reads and writes atomic, so no lock is
    needed for the scalars currently exposed here.  When fields like
    dicts or lists land, a lock goes in.
    """

    #: Parsed timestamp from the first timestamped line we saw.  None
    #: until we've observed one.  Naive-local, same convention as
    #: ``datetime.now()`` — see ``parse_log_timestamp``.
    engine_start_time: Optional[datetime] = None

    #: Line count processed (for debugging — will become a heartbeat
    #: once we have real parsing).
    lines_read: int = 0

    #: Last-line timestamp (useful to detect a stalled engine — if the
    #: log hasn't advanced in a while, the engine likely crashed).
    last_line_time: Optional[datetime] = None

    #: Current state of the AntPosEst state machine (lowercase string
    #: matching the enum values in scripts/peppar_fix/states.py:
    #: "surveying", "verifying", "converging", "anchoring", "anchored",
    #: "moved").  None until the first [STATE] line is observed.
    ant_pos_est_state: Optional[str] = None

    #: AntPosEst latches — have the filter's milestone states been
    #: reached at least once this run?  Clear only on integrity-
    #: monitor trip.  Populated from
    #: ``[STATE] AntPosEst: reached_anchoring + reached_anchored
    #: cleared (fix_set_integrity_trip)`` log lines (clear on trip)
    #: and implicitly True on first entry to ANCHORING / ANCHORED
    #: observed in a ``[STATE] AntPosEst: ... → anchoring / anchored``
    #: transition.  Drives the RECONVERGING / REANCHORING derived
    #: labels in AntennaPositionLine.
    reached_anchoring: bool = False
    reached_anchored: bool = False

    #: Current state of the DOFreqEst state machine (lowercase string:
    #: "uninitialized", "phase_setting", "freq_verifying", "tracking",
    #: "holdover").  None until the first [STATE] line is observed.
    do_freq_est_state: Optional[str] = None

    #: States each machine has visited so far this run (ordered set,
    #: preserving first-visit order).  Used by StateBar widgets to
    #: render visited-vs-unvisited distinction.  Reassigned (rather
    #: than mutated in place) so readers see a consistent snapshot.
    ant_pos_est_visited: tuple[str, ...] = field(default_factory=tuple)
    do_freq_est_visited: tuple[str, ...] = field(default_factory=tuple)

    #: Per-SV current state (SvAmbState as string), keyed by SV
    #: identifier like ``G05``, ``E21``, ``C32``.  Populated from
    #: ``[SV_STATE] <sv>: <from> → <to>`` transition lines.  SVs
    #: that haven't produced a transition yet aren't present —
    #: engine logs one at admission so every observed SV lands here
    #: within a few epochs.  Immutable from readers' perspective:
    #: the reader thread replaces the dict on each update rather
    #: than mutating in place, so an app tick sees a consistent
    #: snapshot.
    sv_states: dict[str, str] = field(default_factory=dict)

    #: Constellation prefixes that have NL integer-fix capability
    #: given the receiver + correction streams currently connected.
    #: Populated from ``Phase bias lookup`` log lines: if any SV of
    #: constellation X has been seen with both f1 and f2 bias HITs,
    #: X is in this set.  Used by SvStateTable to render ``-`` in
    #: NL cells for constellations that *architecturally* can't
    #: reach NL (ptpmon F9T-L2 tracking L2W + CNES publishing L2L
    #: → GPS never NL-capable).  Latched on first HIT-HIT; never
    #: downgraded, because a single confirmed SV proves the bias
    #: pair exists in the stream.
    nl_capable_constellations: frozenset[str] = field(
        default_factory=frozenset)

    #: Latest AntPosEst filter position, extracted from the most
    #: recent ``[AntPosEst N] σ=X pos=(lat, lon, alt) ...`` log
    #: line.  None before the first such line lands.  Tuple is
    #: (lat_deg, lon_deg, alt_m).  Precision matches whatever the
    #: engine logged — today 6 decimals on lat/lon (~11 cm) and
    #: 1 decimal on altitude (~10 cm).  See
    #: ``project_to_main_position_log_precision_20260421.md`` for
    #: the engine-side precision ask.
    antenna_position: Optional[tuple[float, float, float]] = None

    #: Latest AntPosEst 3D σ (m) from the same log line.  Drives
    #: per-digit uncertainty shading in the AntennaPositionLine
    #: widget — digits below the σ quantum render dim.  None
    #: before first observation.
    antenna_sigma_m: Optional[float] = None

    #: Latest nav2Δ (m) — scalar 3D distance from AntPosEst pos to
    #: the F9T's secondary-engine position.  Engine only emits the
    #: delta, not the absolute NAV2 position, so that's what we
    #: display in the "2nd Opinion" row.  None until first
    #: observation or when the engine runs without a nav2_store.
    nav2_delta_m: Optional[float] = None

    #: Latest n= count from the AntPosEst summary line — the number
    #: of SVs actually contributing to the current epoch's filter
    #: solution (post-mask, post-bias-availability).  Used as the
    #: ground truth for "how many SVs are currently active."  The
    #: per-constellation SvStateTable counts (derived from
    #: ``sv_states``) should be in the same ballpark; a wide gap
    #: triggers ``sv_state_count_warning`` to flag drift between
    #: the state-machine's view and the filter's view.
    antenna_n_active: Optional[int] = None

    #: Set when len(non-WAITING sv_states) drifts substantially from
    #: ``antenna_n_active`` — the state machine has zombie SVs that
    #: stopped being observed but never received a → SET transition.
    #: A non-None value here is a signal to investigate whether the
    #: engine's stale-obs sweep is firing as expected.
    sv_state_count_warning: Optional[str] = None

    #: Latest worstσ (m) — the largest per-SV position-sensitivity
    #: sigma produced by the engine's null-mode eigenvalue monitor
    #: (engine commit e5637d9).  Effectively the filter's
    #: smallest-eigenvalue reciprocal expressed as a per-SV σ; huge
    #: values (~10³ m) flag a rank-deficient mode, small values
    #: (<1 m) flag well-observed geometry.  Displayed alongside
    #: positionσ because they answer complementary "how
    #: trustworthy is the fix?" questions.
    worst_sigma_m: Optional[float] = None

    #: Latest ZTD residual (metres, signed).  Log emits as
    #: ``ZTD=<mm>±<sigma_mm>mm`` in millimetres; we store the
    #: central value in m to match the other sigma conventions in
    #: this struct.  None before the first AntPosEst line that
    #: carries ZTD (i.e., engine versions past the
    #: solid-tide/ZTD-log port).
    ztd_m: Optional[float] = None

    #: Latest ZTD uncertainty (mm, positive).  Optional — older
    #: engine versions emitted bare ``ZTD=<mm>mm`` without the
    #: ± field.
    ztd_sigma_mm: Optional[int] = None

    #: Total solid Earth tide magnitude (mm, positive) at the
    #: current epoch.  Parsed from the engine's ``tide=<mm>mm(U<±N>)``
    #: field on the [AntPosEst] line.  None before the first line
    #: that carries the field (older engine builds or --no-solid-tide
    #: runs).  See `docs/obs-model-completion-plan.md`.
    earth_tide_mm: Optional[int] = None

    #: Vertical (Up) component of the solid Earth tide (mm, signed).
    #: Dominates the total magnitude for most stations at most
    #: epochs (peak ±30 cm), so displayed alongside the total as an
    #: orientation cue.
    earth_tide_u_mm: Optional[int] = None

    #: NTRIP mount identifier for the broadcast-ephemeris stream
    #: the engine is connected to, e.g. ``"BCEP00BKG0"``.  Parsed
    #: once from the startup ``Ephemeris stream: HOST:PORT/MOUNT``
    #: line.  None if the engine is running without NTRIP (offline
    #: mode with `--nav` file).
    eph_mount: Optional[str] = None

    #: NTRIP mount identifier for the SSR-corrections stream, e.g.
    #: ``"SSRA00CNE0"``.  None if SSR isn't connected (engine
    #: running in broadcast-only NAV mode).
    ssr_mount: Optional[str] = None

    #: True after we've observed the engine announcing its own
    #: peer-bus publishing.  Latches once set (never flips back to
    #: False within a run) — if the engine was publishing at any
    #: point, peppar-mon retires its LogToBusBridge and lets the
    #: engine be the sole publisher.  Detection via the engine's
    #: ``peer-bus active:`` startup log line (see
    #: ``scripts/peer_publisher.py::initialize``).  Absent in logs
    #: produced by engines without ``--peer-bus``.
    engine_peer_bus_active: bool = False

    #: Last observed ``[COHORT]`` line's position-cohort size —
    #: number of peers (including self) that contributed to the
    #: shared-ARP median.  Part of the consensus-monitor
    #: observability layer; see ``docs/fleet-consensus-monitors.md``.
    #: None when the engine has never logged a [COHORT] line with
    #: a position cohort (no peer-bus, cohort < 2, or no
    #: ``--peer-antenna-ref`` at launch).
    cohort_pos_n: Optional[int] = None

    #: Last observed horizontal distance from this host to the
    #: position cohort median, in millimetres.  Engine reports this
    #: at ~0 mm on a clean shared-antenna fleet; large values mean
    #: this host disagrees with the cohort (wrong integers, null-
    #: mode drift, etc.).
    cohort_delta_h_mm: Optional[int] = None

    #: Last observed 3-D distance from this host to the position
    #: cohort median, in millimetres.
    cohort_delta_3d_mm: Optional[int] = None

    #: Last observed ``[COHORT]`` line's ZTD-cohort size — number
    #: of peers (including self) that contributed to the shared-
    #: atmosphere median.
    cohort_ztd_n: Optional[int] = None

    #: Last observed signed ZTD delta (this host minus the ZTD
    #: cohort median) in millimetres.  Float because engine emits
    #: one decimal.  Positive = this host's residual is wetter
    #: than the cohort.
    cohort_delta_ztd_mm: Optional[float] = None

    #: Most recent ``[FIX_SET_INTEGRITY] TRIPPED`` event observed.
    #: Carries (timestamp, reason, params-string) so the widget can
    #: render "last trip: pos_consensus 4:32 ago" and keep the full
    #: param string for drill-down.  None until the first trip
    #: observed this session.
    fix_set_integrity_last_trip: Optional["FixSetIntegrityTrip"] = None

    #: Running count of integrity-monitor trips observed since the
    #: reader started (includes replayed history).  Useful for
    #: sanity-checking "one trip per day" expectation at a glance.
    fix_set_integrity_trip_count: int = 0

    #: Latest WL Integer Bootstrap success rate (Teunissen 1998/1999),
    #: parsed from ``[WL_AR_READINESS] p_wl_ib=... n=...`` log lines
    #: (engine commit 6e9cca6, every ~10 epochs).  Range [0, 1].  Per
    #: Geng et al. 2010 + Charlie's AR-readiness diagnostic memo:
    #: > 0.999 = full WL AR; > 0.99 = partial AR (PAR); < 0.99 =
    #: diagnose.  None until the engine has emitted at least one
    #: line (older builds didn't).
    wl_p_ib: Optional[float] = None

    #: Number of warmed-up SVs (≥ 20-epoch MW history) contributing
    #: to ``wl_p_ib``.  Tells the operator how thin the float is —
    #: P_IB = 0.99 with n=2 is much weaker evidence than the same
    #: P_IB at n=8.
    wl_p_ib_n: Optional[int] = None

    #: Latest NL Integer Bootstrap success rate (Teunissen 1998/1999),
    #: parsed from ``[AR_READINESS] p_nl_ib=... n=...`` log lines
    #: (engine commit 3cc429e, Charlie A1).  Range [0, 1].  Same
    #: thresholds as ``wl_p_ib`` (>0.999=full, >0.99=PAR, <0.99
    #: diagnose).  ``None`` means one of two things, distinguished
    #: by ``nl_screened_too_few``:
    #:   * Engine has not emitted [AR_READINESS] yet (older build,
    #:     or first ~10 epochs).  ``nl_screened_too_few`` is False.
    #:   * Engine emitted ``p_nl_ib=- n=N (too few screened)`` —
    #:     fewer than 2 SVs passed the NL pre-screen, so P_IB is
    #:     undefined.  ``nl_screened_too_few`` is True; ``nl_p_ib_n``
    #:     carries the screened count.
    nl_p_ib: Optional[float] = None

    #: Number of SVs in the NL pre-screened candidate set.  Always
    #: populated when [AR_READINESS] has been observed even if
    #: ``nl_p_ib`` is None (the "too few screened" case).
    nl_p_ib_n: Optional[int] = None

    #: True when the most recent [AR_READINESS] line carried the
    #: ``p_nl_ib=- n=N (too few screened)`` form — distinguishes the
    #: "no signal yet" state (False, also nl_p_ib None) from "signal
    #: says NL not viable this epoch" (True).  Resets to False the
    #: moment a numeric P_NL_IB lands.
    nl_screened_too_few: bool = False


@dataclass(frozen=True)
class FixSetIntegrityTrip:
    """One ``[FIX_SET_INTEGRITY] TRIPPED`` event parsed out of the log.

    Reason is the trip type (``window_rms``, ``anchor_collapse``,
    ``ztd_impossible``, ``ztd_cycling``, ``pos_consensus``,
    ``ztd_consensus``).  Params is the raw param substring as
    emitted by the engine (e.g. ``delta_m=0.234 threshold_m=0.200
    sustained_epochs=30``) — kept unparsed so the widget can render
    it verbatim without losing engine-side precision or format
    changes.  Timestamp comes from the log line's own prefix.
    """

    timestamp: datetime
    reason: str
    params: str


class LogReader:
    """Threaded engine-log consumer.

    Usage::

        reader = LogReader(Path("/var/log/peppar-fix.log"))
        reader.start()
        ...
        reader.state.engine_start_time   # readable from any thread
        reader.stop()

    The thread is a daemon so process exit doesn't wait on it.  Errors
    inside the thread are logged at WARNING and don't propagate — a
    dead reader just stops updating state; the UI keeps working.
    """

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self.state = LogState()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ── Lifecycle ──────────────────────────────────────────────── #

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run, name="peppar-mon-log-reader", daemon=True,
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=timeout)

    # ── Reader thread body ─────────────────────────────────────── #

    def _run(self) -> None:
        try:
            self._wait_for_file()
            if self._stop.is_set():
                return
            with self.path.open("r", encoding="utf-8", errors="replace") as f:
                # Replay: consume everything currently in the file.
                self._consume(f, follow=False)
                if self._stop.is_set():
                    return
                # Follow: block-poll for new content.
                self._consume(f, follow=True)
        except Exception:
            # Any exception kills the reader thread but not the app.
            # Log it and return — the state snapshot freezes at whatever
            # we'd inferred before the failure.
            log.warning("LogReader crashed", exc_info=True)

    def _wait_for_file(self) -> None:
        """Block until the log file exists or stop is signalled.

        The engine might not be running yet.  We don't want to error
        out — just wait politely.
        """
        while not self._stop.is_set():
            if self.path.exists():
                return
            if self._stop.wait(timeout=_WAIT_FOR_FILE_S):
                return  # stop signalled

    def _consume(self, f, *, follow: bool) -> None:
        """Read lines from ``f``.

        ``follow=False`` reads to EOF and returns.  ``follow=True``
        keeps polling for new content, returning only when stop is set.
        """
        while not self._stop.is_set():
            line = f.readline()
            if not line:
                if not follow:
                    return
                # EOF during follow — sleep briefly and try again.  The
                # Event.wait call makes stop() responsive without
                # blocking the full poll interval.
                if self._stop.wait(timeout=_FOLLOW_POLL_S):
                    return
                continue
            self._ingest(line)

    # ── Per-line processing ────────────────────────────────────── #

    def _ingest(self, line: str) -> None:
        """Update state from one raw log line.

        Extracts:
          * timestamps (first = engine_start_time, latest = last_line_time)
          * [STATE] transitions for AntPosEst and DOFreqEst
        """
        self.state.lines_read += 1
        ts = parse_log_timestamp(line)
        if ts is not None:
            if self.state.engine_start_time is None:
                self.state.engine_start_time = ts
            self.state.last_line_time = ts
        self._parse_state_line(line)
        self._parse_sv_state_line(line)
        self._parse_phase_bias_lookup(line)
        self._parse_antposest_line(line)
        self._parse_stream_lines(line)
        self._parse_peer_bus_active(line)
        self._parse_cohort_line(line)
        self._parse_fix_set_integrity_trip(line)
        self._parse_wl_ar_readiness_line(line)
        self._parse_ar_readiness_line(line)

    def _parse_antposest_line(self, line: str) -> None:
        """Extract position + σ + nav2Δ from ``[AntPosEst N] ...``.

        The engine emits one of these every ~10 epochs with format:

            [AntPosEst 4200] σ=0.023m pos=(LAT, LON, ALT) ...
                             ... nav2Δ=2.8m ZTD=+274±3mm

        We extract three pieces:
          * σ (3D, metres) — drives uncertainty shading on the
            antenna-position display
          * (lat, lon, alt) — the filter's current position
          * nav2Δ — scalar delta to the F9T's secondary-engine
            position; rendered as the "2nd Opinion" row

        nav2Δ is optional (may not appear on early-bootstrap lines
        before a NAV2 fix arrives, or when no nav2_store is
        configured).  Position and σ are always present on
        ``[AntPosEst ...]`` lines, so missing them silently would
        mask a log-format change — treat as a noisy pattern
        mismatch and just skip the line.
        """
        m = _ANTPOSEST_LINE_RE.search(line)
        if m is None:
            return
        self.state.antenna_sigma_m = float(m.group("sigma"))
        self.state.antenna_position = (
            float(m.group("lat")),
            float(m.group("lon")),
            float(m.group("alt")),
        )
        n_active_str = m.group("n_active")
        if n_active_str is not None:
            n_active = int(n_active_str)
            self.state.antenna_n_active = n_active
            # Cross-check against the SV state machine's count: how
            # many SVs are in any non-WAITING state?  WAITING is the
            # cooldown bucket after a slip — those SVs aren't active
            # in the solution by definition.  TRACKING records can
            # be created by monitor lookups before the SV is ever
            # observed; exclude those too.
            sm_active = sum(1 for s in self.state.sv_states.values()
                            if s not in ("WAITING", "TRACKING"))
            # Only warn on a substantial gap.  Some drift is
            # expected: the filter rejects SVs per epoch on
            # residuals; the state machine reflects the most recent
            # transition.  A gap of >5 suggests zombie SVs the
            # engine's stale-obs sweep should have dropped.
            if sm_active > n_active + 5:
                self.state.sv_state_count_warning = (
                    f"sv_state_machine sees {sm_active} non-WAITING SVs "
                    f"but AntPosEst only n={n_active}; "
                    f"diff={sm_active - n_active}. Possible stale SV "
                    "records — engine SET sweep may not be firing.")
            else:
                self.state.sv_state_count_warning = None
        nav2 = m.group("nav2d")
        if nav2 is not None:
            self.state.nav2_delta_m = float(nav2)
        worst = m.group("worst")
        if worst is not None:
            self.state.worst_sigma_m = float(worst)
        ztd_mm = m.group("ztd_mm")
        if ztd_mm is not None:
            self.state.ztd_m = int(ztd_mm) * 1e-3
            ztd_sigma = m.group("ztd_sigma_mm")
            if ztd_sigma is not None:
                self.state.ztd_sigma_mm = int(ztd_sigma)
        tide_mm = m.group("tide_mm")
        if tide_mm is not None:
            self.state.earth_tide_mm = int(tide_mm)
            self.state.earth_tide_u_mm = int(m.group("tide_u_mm"))

    def _parse_cohort_line(self, line: str) -> None:
        """Parse ``  [COHORT] pos_cohort_n=3 Δh=2mm Δ3d=4mm  ztd_cohort_n=4 Δztd=+12.3mm``.

        The engine emits one of these at the AntPosEst 10-epoch
        cadence whenever peer_subscriber is active and at least
        one cohort (pos or ztd) has ≥ 2 members.  The pos and ztd
        segments are independently optional — a single-antenna
        cohort may produce only the pos segment or only the ztd
        segment depending on whether ``--peer-antenna-ref`` and/or
        ``--peer-site-ref`` are set.

        Each segment updates its corresponding state fields; a line
        that carries only the pos segment leaves the ztd fields
        untouched (and vice-versa), so stale values don't linger
        when coverage temporarily drops.  When a segment IS
        present, we overwrite the last snapshot — the widget only
        cares about "latest".
        """
        if "[COHORT]" not in line:
            return
        pos = _COHORT_POS_RE.search(line)
        if pos is not None:
            self.state.cohort_pos_n = int(pos.group("n"))
            self.state.cohort_delta_h_mm = int(pos.group("dh"))
            self.state.cohort_delta_3d_mm = int(pos.group("d3"))
        else:
            # Segment absent on this line — clear so the widget
            # doesn't render a stale "pos cohort" next to a new
            # ztd-only line.
            self.state.cohort_pos_n = None
            self.state.cohort_delta_h_mm = None
            self.state.cohort_delta_3d_mm = None
        ztd = _COHORT_ZTD_RE.search(line)
        if ztd is not None:
            self.state.cohort_ztd_n = int(ztd.group("n"))
            self.state.cohort_delta_ztd_mm = float(ztd.group("d"))
        else:
            self.state.cohort_ztd_n = None
            self.state.cohort_delta_ztd_mm = None

    def _parse_fix_set_integrity_trip(self, line: str) -> None:
        """Parse ``[FIX_SET_INTEGRITY] TRIPPED reason=X params=... at pos=[...]``.

        Engine emits exactly one of these per trip (edge-triggered
        per the monitor's interface contract).  We capture the
        reason token and the raw param substring; the widget
        renders them alongside "X ago" elapsed from the log line's
        own timestamp.

        Six possible reasons today (see
        ``scripts/peppar_fix/fix_set_integrity_monitor.py``):
        ``window_rms``, ``anchor_collapse``, ``ztd_impossible``,
        ``ztd_cycling``, ``pos_consensus``, ``ztd_consensus``.  We
        don't enumerate them here — a future reason auto-surfaces
        in the widget the moment engine code emits it.
        """
        m = _FIX_SET_INTEGRITY_TRIP_RE.search(line)
        if m is None:
            return
        ts = parse_log_timestamp(line)
        if ts is None:
            # Trip line with no parseable timestamp — drop the event
            # rather than fabricate one, since elapsed-since-trip is
            # the whole point of storing the timestamp.
            return
        self.state.fix_set_integrity_last_trip = FixSetIntegrityTrip(
            timestamp=ts,
            reason=m.group("reason"),
            params=m.group("params").strip(),
        )
        self.state.fix_set_integrity_trip_count += 1

    def _parse_wl_ar_readiness_line(self, line: str) -> None:
        """Parse ``[WL_AR_READINESS] p_wl_ib=0.9876 n=5 (...)``.

        Engine emits one of these every 10 epochs alongside
        [AntPosEst] (per commit 6e9cca6).  ``p_wl_ib`` is the
        Teunissen 1998/1999 Integer Bootstrap success rate on the
        wide-lane sub-block of Q_â — the right "is the float ready
        for WL AR?" gate per Geng et al. 2010.  ``n`` is the count
        of warmed-up SVs (≥ 20 epochs of MW history).

        Decision thresholds (from the commit message):
          * > 0.999 = full WL AR green-light
          * > 0.99  = partial AR (PAR) green-light
          * < 0.99  = diagnose Q_â per-SV before AR push

        Both fields update on every observation; older engine
        builds without commit 6e9cca6 simply leave the state
        fields at None.
        """
        m = _WL_AR_READINESS_RE.search(line)
        if m is None:
            return
        self.state.wl_p_ib = float(m.group("p"))
        self.state.wl_p_ib_n = int(m.group("n"))

    def _parse_ar_readiness_line(self, line: str) -> None:
        """Parse ``[AR_READINESS] p_nl_ib=... n=... (...)``.

        Engine emits one of these every 10 epochs alongside the
        WL line (commit 3cc429e, Charlie A1).  Two forms:

          * ``p_nl_ib=0.9876 n=5 (>0.99=PAR-ready, >0.999=full)`` —
            normal case, ``nl_p_ib`` set, ``nl_screened_too_few``
            False.
          * ``p_nl_ib=- n=2 (too few screened)`` — fewer than 2
            SVs passed the NL pre-screen so P_IB is undefined;
            ``nl_p_ib`` stays None, ``nl_screened_too_few`` True.

        Both forms update ``nl_p_ib_n`` so the widget can display
        the screened count regardless of whether P_IB is computable.

        The tag is ``[AR_READINESS]`` not ``[NL_AR_READINESS]`` —
        distinct from the WL line by the field name (``p_nl_ib``
        vs ``p_wl_ib``) and by the absence of the ``WL_`` prefix.
        Be careful the regex doesn't match the WL line: anchor on
        the field name not the bracket, since ``[WL_AR_READINESS]``
        contains ``AR_READINESS`` as a substring.
        """
        m = _AR_READINESS_NUMERIC_RE.search(line)
        if m is not None:
            self.state.nl_p_ib = float(m.group("p"))
            self.state.nl_p_ib_n = int(m.group("n"))
            self.state.nl_screened_too_few = False
            return
        m = _AR_READINESS_TOO_FEW_RE.search(line)
        if m is not None:
            self.state.nl_p_ib = None
            self.state.nl_p_ib_n = int(m.group("n"))
            self.state.nl_screened_too_few = True

    def _parse_stream_lines(self, line: str) -> None:
        """Capture the NTRIP correction-stream identifiers.

        Engine logs one ``Ephemeris stream:`` and one ``SSR stream:``
        line at startup listing ``HOST:PORT/MOUNT``.  We store the
        mount names for display — the host+port is noise for the
        operator glancing at the monitor.
        """
        m = _EPH_STREAM_RE.search(line)
        if m is not None:
            self.state.eph_mount = m.group("mount")
            return
        m = _SSR_STREAM_RE.search(line)
        if m is not None:
            self.state.ssr_mount = m.group("mount")

    def _parse_peer_bus_active(self, line: str) -> None:
        """Latch ``engine_peer_bus_active`` when the engine's
        ``peer-bus active:`` startup line arrives.

        One-way latch: set once on first match, never cleared
        within a run.  If the engine crashes and restarts without
        ``--peer-bus``, we keep the bridge retired — acceptable
        because that's an unusual case and the staleness heuristic
        in the bus (5 s heartbeat timeout) already handles the
        'peer quietly left' story elsewhere.
        """
        if self.state.engine_peer_bus_active:
            return
        if _PEER_BUS_ACTIVE_RE.search(line) is not None:
            self.state.engine_peer_bus_active = True

    def _parse_phase_bias_lookup(self, line: str) -> None:
        """Detect NL-capable constellations from phase-bias lookup logs.

        Engine emits one log line per (SV, signal-pair) the first time
        that pair's biases are looked up — and re-emits whenever the
        bias values change.  Two log line formats are recognized:

        Old format (pre-2026-04-25):
            ``Phase bias lookup: G24 f1=...(HIT) f2=...(HIT) avail=...``
        New format (commit 24a30ab and later):
            ``[PB_APPLIED] G24 f1=GPS-L1CA→C1C val=+0.123m
                                f2=GPS-L5Q→C5Q val=+0.456m avail=...``

        In the new format, HIT is signaled by a numeric ``val=±N.NNNm``
        and MISS by the literal ``val=MISSm``.  Both formats encode the
        same information; the parser accepts either.

        Both HITs → constellation of this SV can reach NL.  We latch
        the constellation as NL-capable on the first HIT-HIT and never
        downgrade.  A single confirmed SV proves the bias pair exists
        in the stream for that system — other SVs may fall in and out
        of individual HIT status, but capability is a property of the
        correction stream, not of any one SV.

        The complement is the useful signal: if no SV of constellation
        X ever shows HIT-HIT, X stays out of the set and the widget
        renders ``-`` for NL cells.
        """
        m = _PB_APPLIED_RE.search(line)
        if m is not None:
            sv = m.group("sv")
            f1_ok = m.group("f1_val") != "MISS"
            f2_ok = m.group("f2_val") != "MISS"
        else:
            m = _PHASE_BIAS_LOOKUP_RE.search(line)
            if m is None:
                return
            sv = m.group("sv")
            f1_ok = m.group("f1_status") == "HIT"
            f2_ok = m.group("f2_status") == "HIT"
        if not (f1_ok and f2_ok):
            return
        prefix = sv[:1]
        if prefix in self.state.nl_capable_constellations:
            return  # already latched
        self.state.nl_capable_constellations = (
            self.state.nl_capable_constellations | frozenset({prefix})
        )

    def _parse_sv_state_line(self, line: str) -> None:
        """Extract per-SV state from ``[SV_STATE] <sv>: <from> → <to>``.

        Engine emits one per transition (the peppar_fix.sv_state
        tracker logs every legal edge).  We capture only the
        post-transition state; the history isn't needed for the
        table view.

        Special handling: the engine emits ``→ SET`` (synthetic
        transition, not a real enum value) when an SV's record is
        forgotten via the stale-obs sweep — i.e. the SV has set
        below horizon.  We treat SET as removal: pop the SV from
        ``sv_states`` so it no longer appears as an active SV in
        the per-constellation count widgets.  Without this, SVs
        that physically set stay visible forever in their last-
        observed state — confusing for long-term monitoring.

        Updates ``self.state.sv_states`` by copy-on-write so readers
        always see a consistent snapshot.  Python dict copies are
        cheap for the 20–40 SVs we typically track.
        """
        m = _SV_STATE_LINE_RE.search(line)
        if m is None:
            return
        sv = m.group("sv")
        new_state = m.group("to")
        new_dict = dict(self.state.sv_states)
        if new_state == "SET":
            new_dict.pop(sv, None)
        else:
            new_dict[sv] = new_state
        self.state.sv_states = new_dict

    def _parse_state_line(self, line: str) -> None:
        """Look for a [STATE] transition and update the relevant
        field, plus AntPosEst latch lines.

        Engine emits three relevant variants (see
        scripts/peppar_fix/states.py):
          * initial:    ``[STATE] AntPosEst: → surveying (initial)``
          * transition: ``[STATE] AntPosEst: surveying → verifying after 12s``
          * latch-clr:  ``[STATE] AntPosEst: reached_anchoring + reached_anchored cleared (fix_set_integrity_trip)``

        Transitions end with ``→ <new_state>`` — the regex catches
        both initial and transition variants.  Latch-clear lines
        are matched separately via substring check since their
        structure doesn't fit the arrow regex.
        """
        # Latch-clear: substring match — the engine emits this once
        # per FixSetIntegrityMonitor trip and covers whichever of
        # the two latches were set at the time.  Both fields drop
        # to False whenever we see this line; subsequent
        # transitions re-latch as appropriate.
        if "[STATE] AntPosEst:" in line and "cleared" in line:
            if "reached_anchored" in line:
                self.state.reached_anchored = False
            if "reached_anchoring" in line:
                self.state.reached_anchoring = False
            return

        m = _STATE_LINE_RE.search(line)
        if m is None:
            return
        machine = m.group("machine")
        new_state = m.group("to")
        if machine == "AntPosEst":
            self.state.ant_pos_est_state = new_state
            if new_state not in self.state.ant_pos_est_visited:
                self.state.ant_pos_est_visited = (
                    self.state.ant_pos_est_visited + (new_state,)
                )
            # Latches follow the engine side (states.py transition):
            # first entry to ANCHORING sets reached_anchoring; first
            # entry to ANCHORED sets both.  We never un-latch on a
            # transition — that only happens on the explicit clear
            # line handled above.
            if new_state == "anchoring":
                self.state.reached_anchoring = True
            elif new_state == "anchored":
                self.state.reached_anchoring = True
                self.state.reached_anchored = True
        elif machine == "DOFreqEst":
            self.state.do_freq_est_state = new_state
            if new_state not in self.state.do_freq_est_visited:
                self.state.do_freq_est_visited = (
                    self.state.do_freq_est_visited + (new_state,)
                )


# ════════════════════════════════════════════════════════════════════
# REGEX → ENGINE EMISSION SITE MAP
# ════════════════════════════════════════════════════════════════════
# Each regex below pairs with one or more log.info(...) emission sites
# in the engine.  Both sides carry a ``peppar-mon contract:`` comment
# pointing at the other.
#
# When ADDING a new regex here, add a matching ``peppar-mon contract:``
# comment near the engine log.info(...) that produces it, naming this
# regex by its module-attribute name.  Otherwise a future engine-side
# change to the format will silently break peppar-mon.
#
# When CHANGING an existing regex (or the engine-side format), update
# both sides + the corresponding test in peppar_mon/tests/test_log_reader.py.
#
# Current map (regex → engine site):
#
#   _STATE_LINE_RE        scripts/peppar_fix/states.py
#                         (StateMachine.__init__ + .transition + latch
#                         clear in AntPosEst.clear_latches)
#   _SV_STATE_LINE_RE     scripts/peppar_fix/sv_state.py
#                         (SvRecordSet.transition); also synthetic
#                         ``→ SET`` from scripts/peppar_fix_engine.py
#                         in the stale-obs sweep
#   _PHASE_BIAS_LOOKUP_RE legacy — no current engine emitter (replaced
#                         by [PB_APPLIED] in 2026-04-25 commit 24a30ab).
#                         Retained for back-compat with archived logs.
#   _PB_APPLIED_RE        scripts/realtime_ppp.py:[PB_APPLIED] log
#                         in the per-epoch phase-bias-application path
#   _ANTPOSEST_LINE_RE    scripts/peppar_fix_engine.py [AntPosEst N]
#                         summary log emitted every ~10 epochs
#   _EPH_STREAM_RE        scripts/peppar_fix_engine.py and
#                         scripts/realtime_ppp.py "Ephemeris stream:"
#   _SSR_STREAM_RE        same files, "SSR stream:" startup line
#   _PEER_BUS_ACTIVE_RE   scripts/peer_publisher.py "peer-bus active:"
# ════════════════════════════════════════════════════════════════════

# Matches both ``[STATE] AntPosEst: → surveying (initial)`` and
# ``[STATE] AntPosEst: converging → anchoring after 393s (details)``.
# Anchoring on ``[STATE]`` avoids false positives from other log lines
# that happen to contain an arrow.  The ``from`` group is optional to
# handle the initial-state log line which has no from-state.
# Engine source: scripts/peppar_fix/states.py
_STATE_LINE_RE = re.compile(
    r"\[STATE\] (?P<machine>\w+): "
    r"(?:(?P<from>[\w_]+) )?→ (?P<to>[\w_]+)\b"
)

# Matches ``[SV_STATE] G05: TRACKING → FLOATING (epoch=…, elev=…, reason=…)``.
# SV is the PRN identifier: one alpha (G/E/C/R/J/I), two or three
# digits.  States are the SvAmbState enum values, all uppercase with
# underscores.  The parenthesised details are not captured — the
# table only needs the current state.
# Engine source: scripts/peppar_fix/sv_state.py (SvRecordSet.transition)
# plus synthetic ``→ SET`` from scripts/peppar_fix_engine.py
# (stale-obs sweep, see ``forget_stale_with_states`` callsite).
_SV_STATE_LINE_RE = re.compile(
    r"\[SV_STATE\] (?P<sv>[A-Z]\d{2,3}): "
    r"(?P<from>[A-Z_]+) → (?P<to>[A-Z_]+)\b"
)

# Matches ``Phase bias lookup: G24 f1=GPS-L1CA→('C1C', 'L1C')(HIT) ``
# ``f2=GPS-L2CL→('C2L', 'L2L')(MISS) avail=[...]``.  We only need
# the SV identifier and the two HIT/MISS statuses — the details
# after ``avail=`` aren't used.  The signal-mapping itself contains
# a tuple in parens (``('C1C', 'L1C')``), so the regex between
# ``f1=`` and ``(HIT|MISS)`` uses non-greedy ``.*?`` to skip past
# the tuple and lock onto the status parens.
# Engine source: NONE — legacy format.  Replaced by [PB_APPLIED]
# format in scripts/realtime_ppp.py at commit 24a30ab (2026-04-25).
# Retained here for back-compat with archived logs.
_PHASE_BIAS_LOOKUP_RE = re.compile(
    r"Phase bias lookup: (?P<sv>[A-Z]\d{2,3})\s+"
    r"f1=.*?\((?P<f1_status>HIT|MISS)\)\s+"
    r"f2=.*?\((?P<f2_status>HIT|MISS)\)"
)

# Matches the engine's newer ``[PB_APPLIED]`` form (commit 24a30ab).
# Capture the SV id and the two ``val=...m`` fields.  HIT shows the
# numeric value (e.g. ``val=+0.123m`` or ``val=-2.876m``), MISS shows
# the literal ``val=MISSm``.  We capture the value-or-MISS token; the
# parser distinguishes by string compare to "MISS".
# Engine source: scripts/realtime_ppp.py per-epoch phase-bias log
# (search the file for "[PB_APPLIED]").
_PB_APPLIED_RE = re.compile(
    # Tolerates optional ``src=<mount>`` after each ``val=...m`` —
    # added 2026-05-02 (P1 of I-122350) for cross-mount bias
    # source attribution.  Captured if present.
    r"\[PB_APPLIED\]\s+(?P<sv>[A-Z]\d{2,3})\s+"
    r"f1=\S+→\S+\s+val=(?P<f1_val>MISS|[-+]?[\d.]+)m"
    r"(?:\s+src=(?P<f1_src>\S+))?\s+"
    r"f2=\S+→\S+\s+val=(?P<f2_val>MISS|[-+]?[\d.]+)m"
    r"(?:\s+src=(?P<f2_src>\S+))?"
)

# Matches ``[AntPosEst 4200] positionσ=0.023m pos=(LAT, LON, ALT) ...``.
# positionσ, lat, lon, alt are always present; nav2Δ, ZTD, and
# worstσ are optional — nav2Δ is absent on pre-NAV2 bootstrap
# lines or runs without nav2_store; ZTD + worstσ came in with
# engine ports f7da44e / e5637d9 and appear alongside obs-model
# corrections.  Altitude is signed (bootstrap-glitch negatives
# seen) and can have any number of decimals.  ZTD is signed in
# mm with optional ±sigma (e.g. ``ZTD=-2850±293mm``); worstσ is
# the filter's null-mode smallest-eigenvalue proxy in metres
# (huge = uncostrained; small = well-observed).
#
# Field name change history: the σ field was renamed to
# ``positionσ`` by engine commit f17fc05 to disambiguate from
# the new ``worstσ`` null-mode metric on the same line.  We
# only match the new name; older pre-rename logs won't parse
# (acceptable — peppar-mon's support window is the current
# engine).
# Engine source: scripts/peppar_fix_engine.py per-epoch summary
# log (search for "[AntPosEst %d] positionσ").  The captured
# ``n=N`` field feeds the zombie-SV consistency check; keep it
# present and accurate.
_ANTPOSEST_LINE_RE = re.compile(
    r"\[AntPosEst \d+\]\s+"
    r"positionσ=(?P<sigma>[\d.]+)m\s+"
    r"pos=\((?P<lat>-?[\d.]+),\s*(?P<lon>-?[\d.]+),\s*(?P<alt>-?[\d.]+)\)"
    r"(?:\s+n=(?P<n_active>\d+))?"
    r"(?:.*?nav2Δ=(?P<nav2d>[\d.]+)m)?"
    r"(?:.*?ZTD=(?P<ztd_mm>[-+]?\d+)(?:±(?P<ztd_sigma_mm>\d+))?mm)?"
    r"(?:.*?tide=(?P<tide_mm>\d+)mm\(U(?P<tide_u_mm>[-+]?\d+)\))?"
    r"(?:.*?worstσ=(?P<worst>[\d.]+)m)?"
)

# Startup lines identifying the correction streams the engine
# connected to.  Emitted once at engine boot, replayed by the
# log reader.
# Engine source: scripts/peppar_fix_engine.py and
# scripts/realtime_ppp.py — both modules emit on connection.
_EPH_STREAM_RE = re.compile(
    r"Ephemeris stream:\s*(?P<host>[\w.-]+):(?P<port>\d+)/(?P<mount>[\w_]+)"
)
# Engine source: same as _EPH_STREAM_RE — the SSR stream startup
# log line lives in scripts/peppar_fix_engine.py (and a duplicate
# in scripts/realtime_ppp.py for the standalone-mode entry point).
_SSR_STREAM_RE = re.compile(
    r"SSR stream:\s*(?P<host>[\w.-]+):(?P<port>\d+)/(?P<mount>[\w_]+)"
)

# Engine's ``peer-bus active: ...`` startup line, emitted once by
# ``scripts/peer_publisher.py::initialize`` when the engine has
# actually opened a peer bus (via ``--peer-bus``).  Peppar-mon's
# fleet mode uses this to decide whether to keep its own
# LogToBusBridge running or retire it in favour of the engine's
# native publishing.
# Engine source: scripts/peer_publisher.py
_PEER_BUS_ACTIVE_RE = re.compile(r"peer-bus active:")

# Matches the ``pos_cohort_n=3 Δh=2mm Δ3d=4mm`` segment of a
# ``[COHORT]`` line.  Δh / Δ3d come from ``ecef_distance_m`` which
# is non-negative, but we tolerate a leading sign for safety.
# Sit loose on whitespace between the three fields — engine joins
# them with single spaces, but the whole [COHORT] line uses double
# spaces between the two segments so ``\s+`` handles both.
# Engine source: scripts/peppar_fix_engine.py per-epoch [COHORT]
# log emitted by AntPosEstThread when peer_subscriber is active.
_COHORT_POS_RE = re.compile(
    r"pos_cohort_n=(?P<n>\d+)\s+"
    r"Δh=(?P<dh>-?\d+)mm\s+"
    r"Δ3d=(?P<d3>-?\d+)mm"
)

# Matches the ``ztd_cohort_n=4 Δztd=+12.3mm`` segment.  Δztd is
# signed with one decimal per the engine's ``:+.1f`` format
# specifier.
# Engine source: scripts/peppar_fix_engine.py — same [COHORT]
# emission as _COHORT_POS_RE; ZTD segment is independently
# present per peer-subscriber's site_ref configuration.
_COHORT_ZTD_RE = re.compile(
    r"ztd_cohort_n=(?P<n>\d+)\s+"
    r"Δztd=(?P<d>[-+]?\d+\.\d+)mm"
)

# Matches the NL form ``  [AR_READINESS] p_nl_ib=0.9876 n=5 (...)`` —
# the engine emission from commit 3cc429e (Charlie A1).  Two
# tag-disambiguating constraints:
#
#   1. The bracket prefix must be ``[AR_READINESS]`` exactly, NOT
#      ``[WL_AR_READINESS]``.  We anchor on a non-letter / non-
#      underscore boundary before the ``[`` so the ``WL_`` prefix
#      doesn't slide into the match.
#   2. The ``p_nl_ib=`` field name distinguishes it from
#      ``p_wl_ib=`` regardless.  Belt + suspenders.
#
# Two forms emitted by the engine:
#   * Numeric: ``p_nl_ib=0.NNNN n=N (>0.99=PAR-ready, >0.999=full)``
#   * "Too few screened": ``p_nl_ib=- n=N (too few screened)``
# Separate regexes keep parsing simple and the state-update logic
# obvious — order of attempts in _parse_ar_readiness_line.
# Engine source: scripts/peppar_fix_engine.py AntPosEstThread —
# emitted every 10 epochs alongside [WL_AR_READINESS]; engine
# commit 3cc429e (Charlie A1).
_AR_READINESS_NUMERIC_RE = re.compile(
    r"(?<![A-Z_])\[AR_READINESS\]\s+"
    r"p_nl_ib=(?P<p>[\d.]+)\s+"
    r"n=(?P<n>\d+)"
)
# Engine source: scripts/peppar_fix_engine.py AntPosEstThread —
# emitted by the same [AR_READINESS] code path as the numeric
# form, taken when fewer than 2 SVs pass NL pre-screen.
_AR_READINESS_TOO_FEW_RE = re.compile(
    r"(?<![A-Z_])\[AR_READINESS\]\s+"
    r"p_nl_ib=-\s+"
    r"n=(?P<n>\d+)\s*"
    r"\(too few screened\)"
)

# Matches ``  [WL_AR_READINESS] p_wl_ib=0.9876 n=5 (>0.99=PAR-ready, >0.999=full)``.
# Engine emission from commit 6e9cca6 (Charlie A2): the Teunissen
# 1998/1999 Integer Bootstrap success rate on the WL sub-block of
# Q_â, plus the count of SVs contributing.  We only capture the
# two numbers — the parenthetical hint to the operator is fixed
# text and not parsed.  ``p_wl_ib`` is a probability in [0,1] so
# the regex accepts ``0.NNNN`` or ``1.0000``; ``n`` is a small
# integer (typical 3-12 SVs).
# Engine source: scripts/peppar_fix_engine.py AntPosEstThread —
# emitted every 10 epochs; engine commit 6e9cca6 (Charlie A2).
_WL_AR_READINESS_RE = re.compile(
    r"\[WL_AR_READINESS\]\s+"
    r"p_wl_ib=(?P<p>[\d.]+)\s+"
    r"n=(?P<n>\d+)"
)

# Matches ``[FIX_SET_INTEGRITY] TRIPPED reason=<r> <params> at pos=[...]``.
# The params substring is whatever the engine formatted for this
# reason — per-reason layout is in
# ``scripts/peppar_fix_engine.py::_apply_integrity_trip`` but we
# don't parse it here; we just capture the whole thing for
# display.  Non-greedy capture so ``at pos=`` anchors the end.
# Engine source: scripts/peppar_fix_engine.py::_apply_integrity_trip
# (search the file for ``[FIX_SET_INTEGRITY] TRIPPED``).
_FIX_SET_INTEGRITY_TRIP_RE = re.compile(
    r"\[FIX_SET_INTEGRITY\] TRIPPED reason=(?P<reason>\w+)\s+"
    r"(?P<params>.+?)\s+at pos="
)

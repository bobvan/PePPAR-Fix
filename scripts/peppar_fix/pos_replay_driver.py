"""Deterministic replay driver — pos_replay **stage 1**: the bit-identical re-feed.

Reads a captured reference bundle (``RawCaptureBundle``, manifest §6) and
re-feeds every raw record back through the **same stores the live engine uses**,
in global ``recv_mono`` order, driven by a **virtual clock** whose ``now_mono``
is the captured ``recv_mono`` of the record being processed — never a wall-clock
read.  This realizes capture-manifest **milestone 0**: with both the ingest
stamp (``recv_mono``) and every freshness/age decision (``now_mono``) sourced
from the captured stream, each store's state is a *pure function of the bundle*,
so two replays of one bundle are bit-identical and independent of when they run
(the precondition for using replay as a regression).

Faithful by construction: it parses each stream with the engine's own decoders
(``pyubx2.UBXReader``, ``ticc._LINE_RE``) and updates the engine's own store
classes (``QErrStore``, ``NavClockStore``, ``NavTimeGpsStore``,
``Nav2PositionStore``, ``TimTm2Store``) via their ``recv_mono=`` ingest path —
no reimplementation of the parse/correlation logic.  The UBX ``identity → store``
dispatch mirrors ``realtime_ppp.serial_reader``; keep the two in step.

**Scope boundary (stage 1 vs stage 2).**  This drives the *virtual-clock-pure*
stores — the qErr / NAV-CLOCK / NAV-TIMEGPS / TIM-TM2 / NAV-PVT chain whose
freshness logic was made ``now_mono``-pure in the milestone-0 refactor (#224–
#230).  SSR / eph / RAWX records are surfaced in the trace (counted, in order)
but **not yet applied**: regenerating ``[PPP_STATE]`` / ``[METAR]`` needs the
filter epoch step, which lives inside the engine's threaded ``run_steady_state``
and must be factored into a reusable, thread-free callable first.  That is
**stage 2** (and carries Charlie's #230 obs↔PPS RAWX canonical-stamp once-over).
Stage 1 proves the foundation stage 2 stands on.
"""
from __future__ import annotations

import os
import sys
from collections import defaultdict
from typing import Callable, Optional

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix.raw_capture import merged_records  # noqa: E402

# Streams surfaced in the trace but applied only in stage 2 (the filter step).
_DEFERRED_STREAMS = ("ssr", "ssr_bias", "eph")


class VirtualClock:
    """``now_mono`` advanced by the replay stream — the single time source.

    Replaces ``time.monotonic()`` for every freshness/age decision during
    replay, so no wall-clock read can leak nondeterminism in.  ``now_mono`` is
    None until the first record is processed.
    """

    __slots__ = ("now_mono",)

    def __init__(self):
        self.now_mono: Optional[float] = None

    def set(self, recv_mono: float) -> None:
        self.now_mono = recv_mono


def default_replay_stores() -> dict:
    """Build the engine's virtual-clock-pure stores for a stage-1 replay."""
    from realtime_ppp import (QErrStore, NavClockStore, NavTimeGpsStore,
                              Nav2PositionStore)
    from peppar_fix.extint_reader import TimTm2Store
    return {
        "qerr": QErrStore(),
        "nav_clock": NavClockStore(),
        "nav_time_gps": NavTimeGpsStore(),
        "nav2": Nav2PositionStore(),
        "nav_pvt": Nav2PositionStore(),
        # disable the late-edge filter so re-feed is a pure passthrough — the
        # filter's trend state would make the store path order-dependent in a
        # way that's about the filter, not the replay determinism under test.
        "extint": TimTm2Store(late_edge_filter=None),
        "ticc_events": [],     # accumulated TiccEvents (filter consumes in stage 2)
    }


class ReplayDriver:
    """Re-feed a captured bundle through the real stores under a virtual clock.

    After :meth:`run`, ``trace`` is the deterministic ``(recv_mono, stream,
    identity)`` sequence, ``counts`` the per-stream record totals, and ``stores``
    the populated engine stores (queryable with ``clock.now_mono`` for a
    deterministic, wall-clock-independent freshness read).
    """

    def __init__(self, bundle_dir: str, *,
                 build_stores: Optional[Callable[[], dict]] = None):
        self.bundle_dir = bundle_dir
        self.clock = VirtualClock()
        self.stores = (build_stores or default_replay_stores)()
        self.trace: list = []
        self.counts: dict = defaultdict(int)

    def run(self) -> list:
        for recv_mono, stream, payload in merged_records(self.bundle_dir):
            self.clock.set(recv_mono)          # advance the virtual clock first
            identity = self._dispatch(stream, payload, recv_mono)
            self.trace.append((recv_mono, stream, identity))
            self.counts[stream] += 1
        return self.trace

    def _dispatch(self, stream: str, payload: bytes, recv_mono: float) -> str:
        if stream == "ubx":
            return self._dispatch_ubx(payload, recv_mono)
        if stream == "ticc":
            return self._dispatch_ticc(payload, recv_mono)
        if stream in _DEFERRED_STREAMS:
            return stream                       # stage 2 applies these
        return stream

    def _dispatch_ubx(self, payload: bytes, recv_mono: float) -> str:
        """Mirror serial_reader's identity→store map for the pure stores."""
        from pyubx2 import UBXReader
        try:
            parsed = UBXReader.parse(payload)
        except Exception:
            return "ubx?"
        if parsed is None:
            return "ubx?"
        ident = parsed.identity
        s = self.stores
        if ident == "TIM-TP" and s.get("qerr") is not None:
            qerr_ps = getattr(parsed, "qErr", None)
            tow_ms = getattr(parsed, "towMS", None)
            if qerr_ps is not None:
                s["qerr"].update(qerr_ps, tow_ms, recv_mono=recv_mono)
        elif ident == "NAV2-PVT" and s.get("nav2") is not None:
            s["nav2"].update(parsed, recv_mono=recv_mono)
        elif ident == "NAV-PVT" and s.get("nav_pvt") is not None:
            s["nav_pvt"].update(parsed, recv_mono=recv_mono)
        elif ident == "NAV-CLOCK" and s.get("nav_clock") is not None:
            s["nav_clock"].update(parsed, recv_mono=recv_mono)
        elif ident == "NAV-TIMEGPS" and s.get("nav_time_gps") is not None:
            s["nav_time_gps"].update(parsed, recv_mono=recv_mono)
        elif ident == "TIM-TM2" and s.get("extint") is not None:
            s["extint"].update(parsed, recv_mono=recv_mono)
        return ident

    def _dispatch_ticc(self, payload: bytes, recv_mono: float) -> str:
        from ticc import _LINE_RE
        from peppar_fix.event_time import TiccEvent
        line = payload.decode("utf-8", "replace").strip()
        m = _LINE_RE.match(line)
        if not m:
            return "ticc?"
        ev = TiccEvent(channel=m.group(3), ref_sec=int(m.group(1)),
                       ref_ps=int(m.group(2).ljust(12, "0")),
                       recv_mono=recv_mono, raw_line=line)
        self.stores.setdefault("ticc_events", []).append(ev)
        return "ticc:" + m.group(3)

    def freshness_snapshot(self) -> dict:
        """Non-destructive store reads at the virtual clock's current time.

        A deterministic, wall-clock-independent fingerprint of the stores after
        replay — equal across two runs of the same bundle iff replay is
        bit-identical.  Reads only with ``now_mono=clock.now_mono`` so the result
        can't depend on when the snapshot is taken.
        """
        now = self.clock.now_mono
        s = self.stores
        out: dict = {"now_mono": now}
        if s.get("qerr") is not None:
            out["qerr"] = s["qerr"].get(now_mono=now)
        if s.get("nav_clock") is not None:
            out["nav_clock"] = s["nav_clock"].get(now_mono=now)
        if s.get("nav_time_gps") is not None:
            out["nav_time_gps"] = s["nav_time_gps"].get(now_mono=now)
        out["n_ticc_events"] = len(s.get("ticc_events", []))
        return out


def main():
    import argparse
    ap = argparse.ArgumentParser(description="pos_replay stage-1 deterministic "
                                 "re-feed of a captured bundle")
    ap.add_argument("bundle_dir", help="a RawCaptureBundle directory")
    ap.add_argument("--verify-deterministic", action="store_true",
                    help="replay twice and confirm bit-identical trace+state")
    args = ap.parse_args()

    r = ReplayDriver(args.bundle_dir)
    r.run()
    print(f"replayed {len(r.trace)} records  "
          f"span recv_mono [{r.trace[0][0]:.3f} … {r.trace[-1][0]:.3f}]"
          if r.trace else "replayed 0 records (empty bundle)")
    for stream in sorted(r.counts):
        print(f"  {stream:9s} {r.counts[stream]}")
    if args.verify_deterministic:
        r2 = ReplayDriver(args.bundle_dir)
        r2.run()
        ok = (r.trace == r2.trace
              and r.freshness_snapshot() == r2.freshness_snapshot())
        print("determinism: " + ("BIT-IDENTICAL across two replays" if ok
                                  else "DIVERGED — replay is NOT deterministic"))
        if not ok:
            raise SystemExit(1)


if __name__ == "__main__":
    main()

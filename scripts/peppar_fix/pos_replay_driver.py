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

Two distinct properties, don't conflate them (Charlie #236):

- **Deterministic** — replay is a pure function of the bundle: same bundle →
  bit-identical trace + store state, independent of wall-clock.  This is what
  stage 1 *proves* (``--verify-deterministic`` / the determinism tests).
- **Faithful to the live engine** — the replayed store state equals what the
  live engine actually held.  This is *bounded*, not proven: it holds only
  insofar as the dispatch mirrors ``serial_reader`` EXACTLY.  Note the trap —
  ``--verify-deterministic`` compares replay-vs-replay, so a dispatch that
  diverges from live *identically on both runs* stays green; fidelity gaps
  must be closed by matching ``serial_reader``, not by the determinism check.
  The qErr-invalid handling is mirrored for exactly this reason; the
  ``late_edge_filter`` divergence is deliberate and flagged for stage 2.

It parses each stream with the engine's own decoders (``pyubx2.UBXReader``,
``ticc._LINE_RE``) and updates the engine's own store classes via their
``recv_mono=`` ingest path — no reimplementation of parse/correlation logic.
The UBX ``identity → store`` dispatch mirrors ``realtime_ppp.serial_reader``;
keep the two in step (fidelity depends on it).

**Scope (stages so far).**

- **Stage 1** — the *virtual-clock-pure* UBX/TICC stores (qErr / NAV-CLOCK /
  NAV-TIMEGPS / TIM-TM2 / NAV-PVT) whose freshness logic was made
  ``now_mono``-pure in the milestone-0 refactor (#224–#230).
- **Stage 2a** — the **corrections**: SSR / eph RTCM frames are now *applied*
  (not just surfaced) into ``SSRState`` / ``BroadcastEphemeris`` via
  ``realtime_ppp.route_rtcm_message`` — the SAME router the live
  ``ntrip_reader`` calls, so replay routing can't drift from live.  ``ssr_bias``
  routes bias-only (secondary mount).  This is the correction state the filter
  step needs, and the seam where **product-swap** plugs in (point the SSR
  stream at final products instead of the captured real-time SSR).

**Stage 2b — obs reconstruction (here, opt-in ``decode_obs=True``).**  Decodes
the captured RAWX inline (in ``recv_mono`` order, so the SSRState used for bias
correction matches the live interleave) into a ``(gps_time, observations,
obs_counts)`` timeline (``self.epochs``) via the engine's own ``rawx_decode`` +
``rawx_to_observations`` (#239) with the manifest-derived sig config (#242).
This is the input the filter step consumes.

**Still ahead.**  Driving ``AntPosEstThread._process_epoch`` (#240) over
``self.epochs`` to regenerate ``[PPP_STATE]`` via ``format_ppp_state_line``,
scored by ``pos_replay_compare`` / the ``pos_sim`` DivergenceMonitor; plus the
#236-F2 ``late_edge_filter`` config and the #236-F4 TICC recv-estimator
reconstruction.
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

# Per-run engine config the replay must mirror to stay faithful to live (the
# generalization of the #236-F1 / #237 "a per-run flag must reach replay"
# lesson — read the run config wholesale instead of one-off-ing each flag).
# Each entry: manifest [conventions] key → default when absent.
_BIAS_SKIP_KEYS = ("skip_biases", "skip_code_biases", "skip_phase_biases")


def read_manifest_conventions(bundle_dir: str) -> dict:
    """The bundle's ``manifest.toml`` ``[conventions]`` table (or {} if absent).

    The single reader for every per-run config the replay mirrors (bias-skip
    flags, receiver, systems, …) — read wholesale so the next field is a lookup,
    not a new I/O path (Charlie #237)."""
    path = os.path.join(bundle_dir, "manifest.toml")
    try:
        import tomllib
        with open(path, "rb") as f:
            return tomllib.load(f).get("conventions", {})
    except (OSError, ValueError):
        return {}           # no/garbled manifest


def read_run_config(bundle_dir: str) -> dict:
    """The RTCM bias-skip flags (``--no-primary-biases`` / ``--no-ssr-code-bias``
    / ``--no-ssr-phase-bias``) the engine recorded in ``[conventions]``.  Missing
    keys → all False (engine default).  Passed straight to ``route_rtcm_message``
    so replay routing matches the captured run."""
    conv = read_manifest_conventions(bundle_dir)
    return {k: bool(conv.get(k, False)) for k in _BIAS_SKIP_KEYS}


def replay_sig_config(bundle_dir: str):
    """Per-receiver signal config for RAWX→obs reconstruction, from the bundle's
    manifest ``receiver``: ``(signal_names, sig_lookup, bds_l1_ref_cycles)``
    ready for ``realtime_ppp.rawx_to_observations``.

    The replay can't decode the captured RAWX without knowing which receiver
    produced it (signal map + IF pairs are receiver-specific), so a manifest
    that names no receiver is an error here — unlike the bias-skip flags, there
    is no safe default.  Reuses the engine's own ``get_driver`` + the shared
    ``build_sig_lookup`` so the replay's sig config is identical to live's."""
    receiver = read_manifest_conventions(bundle_dir).get("receiver", "")
    if not receiver:
        raise ValueError(
            "bundle manifest names no receiver; RAWX→obs reconstruction needs "
            "it (the engine records it in [conventions] at capture)")
    from peppar_fix.receiver import get_driver
    from realtime_ppp import build_sig_lookup
    driver = get_driver(receiver)
    return driver.signal_names, build_sig_lookup(driver), driver.bds_l1_ref_cycles


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
    """Build the engine's stores for replay: the virtual-clock-pure UBX/TICC
    stores (stage 1) plus the correction state (stage 2a: SSRState +
    BroadcastEphemeris, fed through the shared RTCM router)."""
    from realtime_ppp import (QErrStore, NavClockStore, NavTimeGpsStore,
                              Nav2PositionStore)
    from peppar_fix.extint_reader import TimTm2Store
    from ssr_corrections import SSRState
    from broadcast_eph import BroadcastEphemeris
    return {
        "qerr": QErrStore(),
        "nav_clock": NavClockStore(),
        "nav_time_gps": NavTimeGpsStore(),
        "nav2": Nav2PositionStore(),
        "nav_pvt": Nav2PositionStore(),
        # Correction state — fed via realtime_ppp.route_rtcm_message (the SAME
        # router the live ntrip_reader uses, so replay can't drift).  ssr =
        # primary mount; ssr_bias routes bias-only into the same SSRState.
        "ssr": SSRState(),
        "beph": BroadcastEphemeris(),
        # disable the late-edge filter so re-feed is a pure passthrough — the
        # filter's trend state would make the store path order-dependent in a
        # way that's about the filter, not the replay determinism under test.
        # STAGE-2 TODO (Charlie #236 F2): the live engine runs extint WITH a
        # late-edge filter, so this is a deliberate replay-vs-live divergence.
        # Once stage 2 feeds the TIM-TM2 arm into the servo to regenerate
        # [PPP_STATE], it must use the SAME late_edge_filter config the captured
        # run used (record it in the bundle manifest) or the extint arm won't
        # match the live filter's state.
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
                 build_stores: Optional[Callable[[], dict]] = None,
                 run_config: Optional[dict] = None,
                 decode_obs: bool = False):
        self.bundle_dir = bundle_dir
        self.clock = VirtualClock()
        self.stores = (build_stores or default_replay_stores)()
        # Mirror the captured run's RTCM bias-skip config (read from the
        # manifest unless overridden) so replay SSRState matches live (#237).
        self.run_config = (run_config if run_config is not None
                           else read_run_config(bundle_dir))
        self.trace: list = []
        self.counts: dict = defaultdict(int)
        # Obs reconstruction (runner): decode the captured RAWX into a
        # (gps_time, observations, obs_counts) timeline.  Opt-in — stage-1
        # determinism replays don't need it, and replay_sig_config raises when
        # the manifest names no receiver (correct for a capture meant to be
        # decoded; not something a pure re-feed should hit).
        self.decode_obs = decode_obs
        self.epochs: list = []
        if decode_obs:
            (self._sig_names, self._sig_lookup,
             self._bds_l1_ref_cycles) = replay_sig_config(bundle_dir)
            self._systems = read_manifest_conventions(bundle_dir).get(
                "systems", [])

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
        if stream in ("ssr", "ssr_bias", "eph"):
            return self._dispatch_rtcm(stream, payload, recv_mono)
        return stream

    def _dispatch_rtcm(self, stream: str, payload: bytes,
                       recv_mono: float) -> str:
        """Apply a captured RTCM frame (SSR / eph) via the SHARED router, so
        replay routing is identical to the live ntrip_reader by construction.
        ``ssr_bias`` is the secondary mount → bias-only into the same SSRState.
        """
        from pyrtcm import RTCMReader
        from realtime_ppp import route_rtcm_message, RtcmMessageView
        from peppar_fix.event_time import RtcmEvent
        try:
            msg = RTCMReader.parse(payload)
        except Exception:
            return "rtcm?"
        if msg is None:
            return "rtcm?"
        identity = str(getattr(msg, "identity", ""))
        # recv_utc=None is the correct determinism choice (no wall-clock); the
        # apply path (update_from_rtcm + monotonic freshness) never reads it.
        event = RtcmEvent(identity=identity, message=msg, recv_mono=recv_mono,
                          recv_utc=None)
        msg_view = RtcmMessageView(msg, event)
        tag, _detail = route_rtcm_message(
            identity, msg_view, self.stores["beph"], self.stores["ssr"],
            label=stream, bias_only=(stream == "ssr_bias"),
            # mirror the captured run's bias-skip flags (#237)
            **self.run_config)
        return f"{stream}:{identity}"

    def _dispatch_ubx(self, payload: bytes, recv_mono: float) -> str:
        """Mirror serial_reader's identity→store map for the pure stores."""
        # RAWX → observations (runner): decode inline, so the SSRState used for
        # bias correction is exactly what was applied up to THIS recv_mono — the
        # same interleave the live engine saw.  Only when reconstructing obs;
        # otherwise RAWX falls through to UBXReader.parse (no store touches it).
        if self.decode_obs:
            from peppar_fix import rawx_decode
            if rawx_decode.is_rawx(payload):
                return self._decode_rawx_epoch(payload)
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
                # Mirror serial_reader EXACTLY (incl. the invalid-sample path):
                # an invalid qErr is filtered out of the deque get() reads, so
                # omitting qerr_invalid would let a captured-invalid sample
                # appear in replay though it was dropped live — a replay-vs-live
                # divergence INVISIBLE to --verify-deterministic (both replays
                # omit it identically).  F9T TIM fw does set qErrInvalid on
                # reacquisition, so real captures hit this (Charlie #236 F1).
                flags = getattr(parsed, "flags", 0)
                decoded_invalid = getattr(parsed, "qErrInvalid", None)
                if decoded_invalid is None:
                    qerr_invalid = bool(flags & 0x10) if isinstance(flags, int) \
                        else False
                else:
                    qerr_invalid = bool(decoded_invalid)
                s["qerr"].update(qerr_ps, tow_ms, qerr_invalid=qerr_invalid,
                                 recv_mono=recv_mono)
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

    def _decode_rawx_epoch(self, payload: bytes) -> str:
        """Decode one captured RAWX frame → (gps_time, observations, obs_counts)
        appended to ``self.epochs`` (the input the filter step consumes).

        Reuses the engine's own ``rawx_decode`` + ``rawx_to_observations`` with
        the manifest-derived sig config and the live SSRState, so the rebuilt
        observations match what the engine queued at capture.  gps_time is the
        canonical RAWX-header time (GPS epoch + week + rcvTow), same as
        serial_reader."""
        from datetime import datetime, timedelta, timezone
        from peppar_fix import rawx_decode
        from realtime_ppp import rawx_to_observations
        try:
            rawx = rawx_decode.decode_rawx(payload)
        except ValueError:
            return "RXM-RAWX?"          # mirror serial_reader's decode-skip
        gps_time = (datetime(1980, 1, 6, tzinfo=timezone.utc)
                    + timedelta(weeks=int(rawx.week), seconds=float(rawx.rcvTow)))
        observations, raw_obs, n_off, n_single = rawx_to_observations(
            rawx, self._systems, self.stores.get("ssr"),
            self._sig_names, self._sig_lookup, self._bds_l1_ref_cycles)
        obs_counts = {"n_raw": len(raw_obs), "n_off_const": n_off,
                      "n_single": n_single}
        self.epochs.append((gps_time, observations, obs_counts))
        return "RXM-RAWX"

    def _dispatch_ticc(self, payload: bytes, recv_mono: float) -> str:
        from ticc import _LINE_RE
        from peppar_fix.event_time import TiccEvent
        line = payload.decode("utf-8", "replace").strip()
        m = _LINE_RE.match(line)
        if not m:
            return "ticc?"
        # STAGE-2 TODO (Charlie #236 F4): this omits the estimator-derived fields
        # iter_events populates (correlation_confidence, estimator_residual_s,
        # queue_remains).  Fine for stage-1 counting; before stage 2 lets the
        # filter consume these events it must run the recv-estimator over the
        # replayed (source_time_s, recv_mono) pairs to reconstruct them.
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
        # nav2/nav_pvt via the non-destructive get_opinion; extint via its
        # deterministic public counters (consume_latest is destructive) — so the
        # fingerprint certifies the whole store set, not a subset (Charlie #236 M3).
        for key in ("nav2", "nav_pvt"):
            if s.get(key) is not None:
                out[key] = s[key].get_opinion(now_mono=now)
        if s.get("extint") is not None:
            ex = s["extint"]
            out["extint"] = (ex.n_received, ex.n_dropped_acc_est,
                             ex.n_dropped_late_edge)
        # correction state — deterministic counters (stage 2a)
        if s.get("beph") is not None:
            out["beph_n_sats"] = getattr(s["beph"], "n_satellites", None)
        if s.get("ssr") is not None:
            out["ssr_counts"] = (getattr(s["ssr"], "n_orbit", None),
                                 getattr(s["ssr"], "n_clock", None))
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

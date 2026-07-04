"""pos_replay runner — drive the engine's position filter from a replayed
bundle to regenerate ``[PPP_STATE]`` and score it.

The loop closer (pos-replay-stage2b-plan §4): a ``ReplayDriver(decode_obs=True)``
rebuilds the ``(gps_time, observations, obs_counts)`` epoch timeline from the
captured RAWX (#243); this drives the **real** ``AntPosEstThread._process_epoch``
(#240) over it — inline via the driver's ``epoch_sink``, so the filter's
corrections (``RealtimeCorrections`` over the replay's ``SSRState`` /
``BroadcastEphemeris``) reflect the SSR/eph applied up to each epoch's
``recv_mono`` (the same per-epoch interleave as the obs bias correction, not the
post-run final state).  ``_process_epoch`` emits ``[PPP_STATE]`` via the shared
``format_ppp_state_line``; :class:`PppStateCapture` collects those lines, which
``pos_replay_compare`` then scores with the ``pos_sim`` DivergenceMonitor.

Faithful by reuse: the **engine's own** filter (``AntPosEstThread``), epoch step,
RAWX→obs, and RTCM routing — no reimplementation.  End-to-end ``[PPP_STATE]``
output needs a real bundle (broadcast eph for sat positions); the wiring here is
unit-tested with a synthetic RAWX + the real filter construction.
"""
from __future__ import annotations

import logging
import os
import sys
import threading
from typing import Optional

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from peppar_fix.pos_replay_driver import (ReplayDriver,  # noqa: E402
                                          read_manifest_conventions,
                                          read_filter_config,
                                          read_ape_init_state,
                                          read_anchor_decisions)

_ENGINE_LOGGER = "peppar-fix"


class PppStateCapture(logging.Handler):
    """Capture ``[PPP_STATE]`` log lines emitted by ``_process_epoch``.

    Attaches to the engine logger; collects each record whose message is a
    ``[PPP_STATE]`` line (the same string ``pos_replay_compare`` parses).  Use as
    a context manager — it detaches on exit."""

    def __init__(self):
        super().__init__()
        self.lines: list = []
        self._saved_level = None

    def emit(self, record):
        try:
            msg = record.getMessage()
        except Exception:
            return
        if msg.startswith("[PPP_STATE]"):
            self.lines.append(msg)

    def __enter__(self):
        logger = logging.getLogger(_ENGINE_LOGGER)
        # [PPP_STATE] is logged at INFO; ensure the logger passes INFO to us even
        # if the ambient config left it at WARNING (root default) — save/restore
        # so we don't permanently change the engine logger's verbosity.
        if logger.getEffectiveLevel() > logging.INFO:
            self._saved_level = logger.level
            logger.setLevel(logging.INFO)
        logger.addHandler(self)
        return self

    def __exit__(self, *exc):
        logger = logging.getLogger(_ENGINE_LOGGER)
        logger.removeHandler(self)
        if self._saved_level is not None:
            logger.setLevel(self._saved_level)
            self._saved_level = None


def dump_ppp_filter_state(filt) -> dict:
    """Serialize a PPPFilter's converged state (I-204115): the live AntPos
    inherits the Phase-1 bootstrap filter, so a faithful replay must START its
    steady-state filter from that SAME state, not a cold seed.  Captures the
    state vector + covariance + the per-SV ambiguity index map + the ISB/ZTD
    bookkeeping — the float ambiguities live in ``x``/``P`` (indexed by
    ``sv_to_idx``), so restoring these makes the carrier phase immediately usable
    exactly as it was at bootstrap exit.  ``prev_obs`` (slip-detector history) is
    intentionally NOT captured — at most one SV re-floats on the first replayed
    epoch (negligible vs the multi-metre cold-start it replaces).

    KNOWN AR-REPLAY BOUNDARY (Charlie #254): the live AntPos ALSO inherits
    ``mw_tracker`` from bootstrap (engine:2203), already past ``min_epochs=60`` so
    WL/NL can fix immediately; the replay's MW tracker is fresh and must
    re-accumulate ~60 epochs before AR engages.  For FLOAT PPP (no phase-bias
    source → AR never runs) this is moot — validated identical on two float
    receivers (F9T, X20P).  For an AR-capable (e.g. CNES) bundle it would make
    the first ~minute of [PPP_STATE] diverge in the AR transient; closing that
    means also dumping ``mw_tracker._state`` / ``_sv_state`` (same dump/restore
    pattern).  So today's snapshot is steady-state-faithful and float-AR-faithful,
    not AR-transient-faithful — a documented limit, not a silent one."""
    import numpy as np
    return {
        "x": np.asarray(filt.x, dtype=float).tolist(),
        "P": np.asarray(filt.P, dtype=float).tolist(),
        "sv_to_idx": dict(filt.sv_to_idx),
        "pinned_isbs": sorted(getattr(filt, "_pinned_isbs", set())),
        "ztd_sigma_m": float(getattr(filt, "_ztd_sigma_m", 0.2)),
        "pos_sigma_m_init": float(getattr(filt, "_pos_sigma_m_init", 10.0)),
        "ztd_window_elapsed": float(getattr(filt, "_ztd_window_elapsed", 0.0)),
        # ZTD process-noise coef (--q-ztd-antpos / RTKLIB recipe).  Reaches the
        # live AntPos filter via bootstrap inheritance, NOT the fresh _shim, so
        # capturing the filter's actual attribute is the faithful way to carry it
        # into replay (else the replay reverts to the 1.29e-3 default and a
        # Q-tuned bundle replays with the WRONG ZTD stiffness — I-215452).
        "ztd_q_coef": float(getattr(filt, "_ztd_q_coef", 1.29e-3)),
    }


def restore_ppp_filter_state(filt, state: dict) -> None:
    """Restore a :func:`dump_ppp_filter_state` snapshot onto a freshly built
    PPPFilter — overwriting the cold ``initialize()`` seed with the captured
    bootstrap-exit state so replay starts converged (initial-P included, Charlie
    #253 finding 2).  The filter keeps its own ``clock_model``/Q config (already
    matched from ``[filter_config]``); only the estimated state is replaced."""
    import numpy as np
    filt.x = np.asarray(state["x"], dtype=filt.x.dtype if filt.x is not None
                        else float)
    filt.P = np.asarray(state["P"], dtype=float)
    filt.sv_to_idx = dict(state.get("sv_to_idx", {}))
    filt._pinned_isbs = set(state.get("pinned_isbs", []))
    filt._ztd_sigma_m = float(state.get("ztd_sigma_m", 0.2))
    filt._pos_sigma_m_init = float(state.get("pos_sigma_m_init", 10.0))
    filt._ztd_window_elapsed = float(state.get("ztd_window_elapsed", 0.0))
    # ZTD process-noise coef — default to the engine's 1.29e-3 for old snapshots
    # that predate this field, so they restore exactly as before.
    filt._ztd_q_coef = float(state.get("ztd_q_coef", 1.29e-3))
    filt.prev_obs = {}                 # slip history not captured — see dump()
    filt.initialized = True            # clock is in the restored x, no self-seed


def _build_replay_antex_parser(args):
    """The antenna PCV parser for replay, or None (logged) when unavailable.

    Mirrors the engine's PCV resolution (resolve_pcv_defaults → ANTEXParser,
    engine:10065+) so replay applies the same phase-center correction.  Resolves
    by the captured receiver antenna against the replay host's antex catalog —
    portable across hosts, unlike a captured absolute path.  Returns None (with a
    loud log) when PCV is off, no antenna was captured, or no antex is found, so
    PCV is never SILENTLY inert (the failure mode Charlie #253 flagged)."""
    log = logging.getLogger(_ENGINE_LOGGER)
    if not bool(getattr(args, "pcv", True)):
        return None                                  # --no-pcv on the live run
    antenna = getattr(args, "receiver_antenna", None)
    if not antenna:
        log.info("pos_replay PCV: no receiver_antenna captured — PCV inactive")
        return None
    # A captured antex_path is only usable if it exists on THIS host; otherwise
    # re-resolve by antenna (the common case: default catalog, no --antex-path).
    antex_cli = getattr(args, "antex_path", None)
    if antex_cli and not os.path.exists(antex_cli):
        antex_cli = None
    try:
        from peppar_fix.antex_resolve import resolve_pcv_defaults
        from antex import ANTEXParser
        pcv = resolve_pcv_defaults(arp_label=None, antex_path_cli=antex_cli,
                                   receiver_antenna_cli=antenna, pcv_flag=True)
        if pcv.enabled and pcv.antex_path:
            return ANTEXParser(pcv.antex_path)
        log.info("pos_replay PCV: no antex for %r — PCV inactive in replay (%s)",
                 antenna, getattr(pcv, "detail", ""))
    except Exception as exc:               # noqa: BLE001 — never break the build
        log.warning("pos_replay PCV: antex parser build failed (%s) — "
                    "PCV inactive in replay", exc)
    return None


def build_filter_thread(driver: ReplayDriver, known_ecef, *, systems=None,
                        args=None, corrections=None):
    """Construct the engine's ``AntPosEstThread``.  By default the filter's
    corrections are ``RealtimeCorrections`` wrapping the SAME ``beph``/``ssr`` the
    ``driver`` populates inline (filter + obs decode share one evolving state).

    Pass ``corrections`` to swap the filter's orbit/clock source — the
    **precise-orbit** product-swap (Charlie #245-1): a corrections OBJECT (e.g.
    :class:`PreciseCorrections` over SP3+CLK), not a `beph` fill.  The thread is
    built but NOT started; the runner calls ``_process_epoch`` per replayed
    epoch."""
    from types import SimpleNamespace
    import peppar_fix_engine as eng
    from ssr_corrections import RealtimeCorrections
    # The captured per-run filter config (I-191033): the live engine recorded
    # its AntPosEst-relevant args in the bundle's [filter_config]; build the
    # replay's args from it so the thread is configured IDENTICALLY (NAV2 anchor,
    # ZTD tie, solid tide, clock model, …).  An explicit args= still wins (tests
    # / product-swap callers); otherwise a missing manifest yields spec defaults.
    fc = read_filter_config(driver.bundle_dir)
    if args is None:
        args = SimpleNamespace(**fc, ppp_state_log=True)
    # Q_POS_CONVERGED is a PPPFilter CLASS attribute the engine sets globally at
    # startup (engine:12624); mirror that here from the captured config so a
    # Q-tuned bundle replays with the live converged-position stiffness.  Always
    # set it (default 1e-4 when unset) so a prior replay's override can't leak
    # into this one in a shared process (I-215452).
    #
    # KEEP CASES SEQUENTIAL WHILE Q LIVES ON THE CLASS (Charlie review #258):
    # this is global class state, so it's only safe because the case library
    # replays bundles one at a time in a shared process (the always-set reset
    # handles the leak BETWEEN sequential cases).  If replay ever fans out
    # bundles concurrently in one process, this write would race across cases —
    # move Q_POS_CONVERGED to a per-filter instance attribute first.
    from solve_ppp import PPPFilter as _PF
    _qpc = getattr(args, "q_pos_converged", None)
    _PF.Q_POS_CONVERGED = float(_qpc) if _qpc is not None else 1e-4
    if corrections is None:
        corrections = RealtimeCorrections(driver.stores["beph"],
                                          driver.stores["ssr"])
    ape_sm = eng.AntPosEst(wl_only=bool(getattr(args, "wl_only", False)))
    # Build the antenna PCV parser so replay applies the SAME phase-center
    # correction the live engine did (Charlie #253 finding 1).  The thread's
    # _pcv_enabled requires BOTH receiver_antenna_type AND antex_parser — passing
    # the antenna without the parser leaves PCV silently inert (the live filter
    # ran it on).  Re-resolve by antenna against the replay host's antex catalog
    # (portable: a captured absolute path may not exist here); on any miss, log a
    # loud skip rather than silently dropping the correction.
    antex_parser = _build_replay_antex_parser(args)
    # Replicate the live engine's AntPosEstThread construction (engine:~10718):
    # pass the same null-constraining config + the NAV2 store the replay driver
    # already populates from captured NAV2-PVT/NAV-PVT.  Without these the filter
    # wanders the (pos,ZTD,clk) null and diverges ~8 m from captured-live.
    thread = eng.AntPosEstThread(
        tuple(known_ecef), corrections, threading.Event(), ape_sm,
        systems=list(systems) if systems else [],
        nav2_store=driver.stores.get("nav2"),
        # engine coerces a None ar-elev-mask to 25.0 at parse (engine:12586);
        # mirror that so replay matches the live AR mask (and never passes None,
        # which NarrowLaneResolver's `mask > 0` test can't compare).
        ar_elev_mask_deg=(25.0 if getattr(args, "ar_elev_mask", None) is None
                          else float(args.ar_elev_mask)),
        join_test_enabled=bool(getattr(args, "join_test", True)),
        wl_only=bool(getattr(args, "wl_only", False)),
        solid_tide=bool(getattr(args, "solid_tide", True)),
        receiver_antenna_type=getattr(args, "receiver_antenna", None),
        pcv_enabled=bool(getattr(args, "pcv", True)),
        clock_model=getattr(args, "clock_model", "random_walk"),
        rx_tcxo_adev_1s=getattr(args, "rx_tcxo_adev_1s", None),
        phase_windup_enabled=bool(getattr(args, "phase_windup", False)),
        gmf_enabled=bool(getattr(args, "gmf", False)),
        slip_rate_limit_s=float(getattr(args, "slip_rate_limit_s", 0.0)),
        ztd_tie_sigma=getattr(args, "ztd_tie_sigma", None),
        nav2_anchor_enabled=bool(getattr(args, "nav2_soft_anchor", True)),
        nav2_anchor_max_hacc_m=float(getattr(args, "nav2_anchor_max_hacc_m", 3.0)),
        nav2_floor_enabled=bool(getattr(args, "nav2_floor", False)),
        nav2_floor_trigger_m=float(getattr(args, "nav2_floor_trigger_m", 5.0)),
        pin_position=bool(getattr(args, "pin_position", False)),
        antex_parser=antex_parser,
        args=args)
    # Age captured NAV2 opinions against the replayed epoch's recv_mono (the
    # driver's virtual clock), not the replay host's wall clock — else the
    # anchor sees every captured fix as stale and never fires.
    thread._now_mono = lambda: driver.clock.now_mono
    # Deterministic NAV2 anchor (I-215452): if the capture recorded the live
    # anchor's per-epoch firing decisions, apply them verbatim in replay instead
    # of re-deriving via get_opinion — removing the freshness-timing +
    # amplification mismatch that drove the dynamic-window divergence (the anchor
    # ablation showed it injects ~600mm in dynamic windows).  Old bundles (no
    # log) → None → the get_opinion path (current behavior) is preserved.
    thread._anchor_decisions = read_anchor_decisions(driver.bundle_dir)
    # Seed state (I-204115): if the capture recorded the AntPos initial filter
    # state (the Phase-1 bootstrap result the live AntPos inherited), restore it
    # so replay starts ALREADY CONVERGED — same position/ZTD/clock/float-
    # ambiguities and initial P the live filter began with — instead of cold.
    # This closes the ~1.6 m cold-start null-coordinate residual that remained
    # after the config fix.  Old bundles (no snapshot) fall back to the clock
    # self-seed below.
    init_state = read_ape_init_state(driver.bundle_dir)
    if init_state is not None:
        restore_ppp_filter_state(thread._filt, init_state)
    else:
        # No captured seed → FRESH filter (runner bootstrap gap, I-175208): the
        # runner has no bootstrap_result to inherit a converged clock, and
        # initialize() leaves it initialized=True with clock=0 → every IF
        # residual is a ~ms (~100s-km) outlier → all obs rejected → n_used=0,
        # never converges.  initialized=False lets PPPFilter.update self-seed the
        # clock (the live engine's "Clock seeded from N PRs").
        thread._filt.initialized = False
    return thread


class PreciseCorrections:
    """Precise (final-products) orbit/clock corrections as a corrections OBJECT,
    for the precise-orbit product-swap (Charlie #245-1): SP3 orbits + CLKFile
    clocks, bridged into the engine's ``sat_position(sv,t) → (pos, clk)`` /
    ``sat_clock(prn,t)`` interface.

    ``SP3.sat_position`` already returns the engine's ``(pos_array, clk)`` 2-tuple
    (same shape as ``RealtimeCorrections.sat_position``; its docstring's "(x,y,z)"
    describes only the position part — verified at solve_pseudorange.py:133), so
    this just unpacks it and substitutes the precise CLK clock when a ``clk_file``
    is given (Charlie #248).  Like ``RealtimeCorrections`` it returns
    ``(None, None)`` whenever it can't form a COMPLETE correction (outside the SP3
    window, or no precise clock for the SV) so the engine's ``sat_pos is not
    None`` guard skips the SV cleanly — never ``(pos, None)``.  Biases are NOT
    here (SP3/CLK carry none); the obs leg reads ``stores['ssr']``, so pair this
    with a Bias-SINEX ssr-fill loader (the second half of a precise swap)."""

    def __init__(self, sp3, clk_file=None):
        self.sp3 = sp3
        self.clk_file = clk_file

    def sat_position(self, sv, t):
        pos, sp3_clk = self.sp3.sat_position(sv, t)   # SP3 returns (pos, clk)
        if pos is None:
            return None, None              # outside SP3 window → skip SV cleanly
        if self.clk_file is not None:
            clk = self.clk_file.sat_clock(sv, t)
            if clk is None:
                return None, None          # no precise clock for SV → skip
        else:
            clk = sp3_clk                  # no CLK file → SP3's own clock (NOT 0)
        return pos, clk

    def sat_clock(self, prn, t):
        if self.clk_file is not None:
            return self.clk_file.sat_clock(prn, t)
        _pos, sp3_clk = self.sp3.sat_position(prn, t)  # SP3's own clock, not 0
        return sp3_clk


def make_precise_corrections(sp3_path, clk_path=None):
    """Build :class:`PreciseCorrections` from final-product files — ``SP3``
    (solve_pseudorange) orbits + optional ``CLKFile`` (ppp_corrections) clocks.
    Deterministic (fixed local files).  The filter's orbits/clocks become
    precise; pair with ``make_ssr_records_loader``/a Bias-SINEX loader for the
    obs-leg biases and ``swap_streams`` to drop the captured eph/ssr."""
    from solve_pseudorange import SP3
    sp3 = SP3(sp3_path)
    clk_file = None
    if clk_path:
        from ppp_corrections import CLKFile
        clk_file = CLKFile(clk_path)
    return PreciseCorrections(sp3, clk_file)


def run_pos_replay(bundle_dir: str, known_ecef, *, systems=None, args=None,
                   truth=None, corrections_loader=None, swap_streams=None,
                   corrections_override=None) -> dict:
    """Replay a bundle through the real position filter; return the regenerated
    ``[PPP_STATE]`` lines (and an optional position score vs ``truth``).

    ``systems`` defaults to the manifest's; ``truth`` (a ``StaticTruth``) enables
    the divergence-monitor position score.

    **Product-swap** — pass ``corrections_loader``, a ``callable(stores)`` that
    populates ``stores['ssr']`` / ``stores['beph']`` from an ALTERNATE source.
    ``swap_streams`` controls which captured correction streams are swapped OUT
    (default: all three — for a precise-orbit swap).  For an **ssr-source** swap
    (different SSR/biases over the SAME broadcast orbits, e.g.
    :func:`make_ssr_records_loader`), pass ``swap_streams={'ssr','ssr_bias'}`` so
    the captured broadcast ``eph`` — the orbit base those corrections ride on —
    is KEPT (Charlie #245-1/#246).  This is the knob that separates *our filter*
    from *our corrections*: if a ZTD drift disappears under the swapped
    corrections, the gap was products; if it persists, it's the
    filter/observability (manifest §6).

    ``swap_streams`` *with* a loader = **swap-and-replace** (the loader supplies
    the substitute corrections).  ``swap_streams`` *without* a loader = a
    deliberate **ablation** — those streams are dropped with nothing replacing
    them (e.g. "how does the filter do with no SSR, on broadcast orbits?").
    Legitimate as an experiment, just not an accident (Charlie #247).

    **Precise-orbit swap** — pass ``corrections_override`` (a corrections OBJECT,
    e.g. :class:`PreciseCorrections` over SP3+CLK) to replace the filter's
    orbit/clock source entirely.  Captured eph/ssr are swapped out (the override
    supplies orbits+clocks); pair with an ssr-bias ``corrections_loader`` +
    ``swap_streams`` for the obs-leg biases (SP3/CLK carry none)."""
    if systems is None:
        systems = read_manifest_conventions(bundle_dir).get("systems") or None
    swap = corrections_loader is not None or corrections_override is not None
    # precise-orbit override swaps all captured corrections by default (it
    # supplies orbits+clocks); an ssr-source loader narrows that via swap_streams
    if corrections_override is not None and swap_streams is None:
        swap_streams = {"ssr", "ssr_bias", "eph"}
    driver = ReplayDriver(bundle_dir, decode_obs=True,
                          apply_captured_corrections=not swap,
                          swap_streams=swap_streams)
    if corrections_loader is not None:
        # load the alternate corrections BEFORE replay so they're in place for
        # the first epoch (final products are slowly-varying; a snapshot over a
        # replay window is the right first model)
        corrections_loader(driver.stores)
    thread = build_filter_thread(driver, known_ecef, systems=systems, args=args,
                                 corrections=corrections_override)
    driver.epoch_sink = thread._process_epoch
    with PppStateCapture() as cap:
        driver.run()
    result = {
        "ppp_state_lines": cap.lines,
        "n_epochs_decoded": len(driver.epochs),
        "product_swapped": swap,
        "thread": thread,
        "driver": driver,
    }
    if truth is not None:
        from peppar_fix.pos_replay_compare import parse_ppp_state, compare_position
        result["position"] = compare_position(parse_ppp_state(cap.lines), truth)
    return result


def make_ssr_records_loader(path):
    """An **ssr-source** product-swap loader (the first concrete one): populate
    the replay's ``SSRState`` from an external SSR-records JSON — the HAS-bridge
    format (``load_ssr_records`` / ``SSRState.update_from_records``: per-SV
    orbit/clock/code-bias corrections to broadcast).  Pair with
    ``swap_streams={'ssr','ssr_bias'}`` so the captured broadcast ``eph`` (the
    orbit base these corrections ride on) is kept.

    Deterministic by construction — loads from a fixed local file, not a timed
    network fetch, so ``--verify-deterministic`` holds under swap (Charlie
    #245-2).  Raises if the file has no records (a loud coverage check beats
    silently feeding empty corrections — Charlie #245-2)."""
    def _load(stores):
        from realtime_ppp import load_ssr_records
        epoch_s, records = load_ssr_records(path)
        if not records:
            # load_ssr_records returns (None, None) for missing/garbled/empty —
            # all of which would silently feed empty corrections; fail loud.
            raise ValueError(
                f"ssr-records product file missing/garbled/empty (no records): "
                f"{path} (coverage check — would silently feed no corrections)")
        stores["ssr"].update_from_records(records, epoch_s)
    return _load

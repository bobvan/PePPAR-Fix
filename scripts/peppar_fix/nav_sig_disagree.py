"""NavSigDisagreeMonitor — Phase A.5 of slipDetectUnified-main.

Bob's directive (dayplan 2026-05-12 12:35 CDT): "we may use SVs the
receiver doesn't, and may exclude SVs it uses, but we MUST LOG when
we differ".  Pure logger — no exclusion behavior change.

The receiver's UBX-NAV-SIG message reports per-(SV, signal) usage
flags (``prUsed``, ``crUsed``, ``health``, ``prRes``, ``cno``,
``ionoModel``, ``corrUsed``).  When the receiver's verdict differs
from our engine's admit decision, the disagreement is data we use
in Phase B to tune engine-side exclusion.

## Expected ``Nav2SignalStore`` interface

Bravo is implementing this in ``f9tClockTelemetry-bravo``.  The
contract this module relies on::

    class Nav2SignalStore:
        def get_signal(self, sv: str, sig_id: str,
                       max_age_s: float = 5.0) -> dict | None:
            '''Return signal status dict or None if stale/missing.

            Dict keys (minimum): prUsed (bool), prRes (float, m),
            cno (float, dB-Hz), health (int).
            Optional: crUsed, doUsed, prSmoothed, ionoModel, corrUsed.
            '''

        def iter_signals(self, max_age_s: float = 5.0):
            '''Yield (sv, sig_id) pairs for all currently-fresh signals.'''

A ``StubSignalStore`` is provided below for unit testing and for
back-compat when the engine runs without NAV-SIG plumbing.

## Hook point in engine

Caller (engine) collects its per-epoch admission set with
diagnostic fields, then calls ``monitor.check_epoch(epoch,
our_admit_set, signal_store)`` exactly once per epoch.  Monitor
walks the receiver's signal set, classifies each (SV, sig_id) as
agree/disagree against our admissions, and emits one structured
``[NAV-SIG_DISAGREE]`` log line per *transition* (entering a new
disagreement state).  No log spam — only state changes.

Both directions are tracked:
- ``receiver=excluded our=admit``: receiver caught something we
  haven't (likely chip-slip, lock loss).  Phase B candidates.
- ``receiver=admitted our=exclude``: our gates (PB_GAP_DROP,
  elev mask, ZTD-trip, slip-flush cooldown) are stricter.  Phase
  B-side candidates for relaxing or auditing.

## Mapping engine signal names → NAV-SIG sig_id

The engine uses internal names like ``GPS-L1CA``, ``GAL-E5aQ``.
UBX-NAV-SIG uses the gnssId/sigId numeric codes (per UBX integration
spec).  ``_engine_sig_to_ubx_sigid()`` translates; the same mapping
also lives in bravo's ``rinex_writer.py`` ``_INTERNAL_TO_BAND_ATTR``.
"""

from __future__ import annotations

import logging
from typing import Any

log = logging.getLogger(__name__)


# Engine internal signal name → (gnssId numeric, sigId numeric).
# Matches UBX-NAV-SIG payload per u-blox F9/F10 integration spec.
# When bravo's parser lands the canonical mapping, this can move to
# a shared module.
_ENGINE_SIG_TO_UBX: dict[str, tuple[int, int]] = {
    # GPS (gnssId=0)
    "GPS-L1CA": (0, 0),
    "GPS-L2CL": (0, 3),
    "GPS-L2CM": (0, 4),
    "GPS-L5Q":  (0, 7),
    "GPS-L5I":  (0, 6),
    # Galileo (gnssId=2)
    "GAL-E1C":  (2, 0),
    "GAL-E1B":  (2, 1),
    "GAL-E5aI": (2, 3),
    "GAL-E5aQ": (2, 4),
    "GAL-E5bI": (2, 5),
    "GAL-E5bQ": (2, 6),
    # BeiDou (gnssId=3)
    "BDS-B1I":  (3, 0),
    "BDS-B2I":  (3, 2),
    "BDS-B2aI": (3, 5),
    "BDS-B2aQ": (3, 6),
    "BDS-B3I":  (3, 7),
}


def engine_sig_to_ubx_sigid(engine_sig: str) -> tuple[int, int] | None:
    """Engine internal signal name → (gnssId, sigId).  None if unknown."""
    return _ENGINE_SIG_TO_UBX.get(engine_sig)


class StubSignalStore:
    """Drop-in for Nav2SignalStore until bravo's lands.

    Pre-populate with ``store.set(sv, sig_id, prUsed=..., prRes=...)``
    in tests.  Production engine should plug in the real store.
    """

    def __init__(self) -> None:
        self._sigs: dict[tuple[str, str], dict[str, Any]] = {}

    def set(self, sv: str, sig_id: str, **fields: Any) -> None:
        self._sigs[(sv, sig_id)] = dict(fields)

    def clear(self, sv: str, sig_id: str) -> None:
        self._sigs.pop((sv, sig_id), None)

    def get_signal(self, sv: str, sig_id: str,
                   max_age_s: float = 5.0) -> dict | None:
        return self._sigs.get((sv, sig_id))

    def iter_signals(self, max_age_s: float = 5.0):
        for k in self._sigs:
            yield k


class NavSigDisagreeMonitor:
    """Detect transitions in receiver vs engine admit-decision pairs.

    Phase A.5 of ``slipDetectUnified-main``.  Pure logger; no engine
    behavior change.  See module docstring for the data design.

    Usage::

        monitor = NavSigDisagreeMonitor()
        # ...in the engine's epoch loop, after admissions are decided:
        monitor.check_epoch(epoch, our_admit_set, signal_store)

    where ``our_admit_set`` is a dict mapping ``(sv, sig_id_engine)``
    to a diagnostic dict for the signals the engine admitted this
    epoch.  Missing keys = engine excluded.
    """

    def __init__(self) -> None:
        # (sv, sig_id_engine) → (receiver_admits: bool, our_admits: bool)
        self._last_state: dict[tuple[str, str], tuple[bool, bool]] = {}

    def check_epoch(
        self,
        epoch: int,
        our_admit_set: dict[tuple[str, str], dict[str, Any]],
        signal_store: Any,
    ) -> int:
        """One epoch's worth of agreement/disagreement classification.

        Returns the count of disagreement log lines emitted this call
        (useful for tests + diagnostic counters).
        """
        if signal_store is None:
            return 0

        n_disagree = 0
        seen: set[tuple[str, str]] = set()

        # Walk receiver's known signals first.  For each, look up our
        # admit decision and the diagnostic snapshot.
        for sv, sig_id_engine in self._iter_receiver_signals(
                our_admit_set, signal_store):
            seen.add((sv, sig_id_engine))
            receiver_sig = self._receiver_sig_for(
                sv, sig_id_engine, signal_store)
            if receiver_sig is None:
                # Receiver-side ID translation failed or stale.
                # Don't classify — leave last_state untouched.
                continue
            receiver_admits = bool(receiver_sig.get('prUsed', False))
            our_admits = (sv, sig_id_engine) in our_admit_set
            new_state = (receiver_admits, our_admits)
            last = self._last_state.get((sv, sig_id_engine))
            if last == new_state:
                continue
            self._last_state[(sv, sig_id_engine)] = new_state
            if receiver_admits == our_admits:
                # Transitioned, but into agreement — no log spam.
                continue
            # Disagreement transition.
            n_disagree += 1
            our_diag = our_admit_set.get((sv, sig_id_engine), {})
            self._emit(
                epoch=epoch,
                sv=sv,
                sig_id=sig_id_engine,
                receiver_admits=receiver_admits,
                our_admits=our_admits,
                receiver_sig=receiver_sig,
                our_diag=our_diag,
            )
        return n_disagree

    def _iter_receiver_signals(self, our_admit_set, signal_store):
        """Yield (sv, sig_id_engine) for all signals we should classify.

        Union of receiver-known and engine-admitted signals.  Avoids
        missing the rare ``we_admit + receiver_doesn't_track`` case
        (e.g., a signal we got via RAWX but the nav engine never saw).
        """
        # Receiver-known.  iter_signals() may yield (gnssId, sigId)
        # numeric pairs or (sv_str, sig_id_str); accept both.
        for entry in signal_store.iter_signals():
            sv, sig_id_engine = self._normalize_signal_key(entry)
            if sv is None:
                continue
            yield sv, sig_id_engine
        # Engine-admitted that receiver didn't yield.
        for sv, sig_id_engine in our_admit_set:
            yield sv, sig_id_engine

    @staticmethod
    def _normalize_signal_key(entry):
        """Accept either ('G07', 'GPS-L1CA') or ((0, 0)) iter_signals shape."""
        if isinstance(entry, tuple) and len(entry) == 2:
            a, b = entry
            if isinstance(a, str) and isinstance(b, str):
                return a, b
            if isinstance(a, int) and isinstance(b, int):
                # numeric (gnssId, sigId) — too ambiguous without sv.
                # Bravo's store should yield (sv, sig_id_engine) pairs
                # for cleanest interop.  Defer.
                return None, None
        return None, None

    def _receiver_sig_for(self, sv: str, sig_id_engine: str, signal_store):
        """Pull receiver's signal record by either engine-sig or UBX-sigid."""
        # Try engine-name first (preferred interop).
        rec = signal_store.get_signal(sv, sig_id_engine)
        if rec is not None:
            return rec
        # Fall back to UBX (gnssId, sigId).
        ubx = engine_sig_to_ubx_sigid(sig_id_engine)
        if ubx is not None:
            return signal_store.get_signal(sv, f"{ubx[0]}:{ubx[1]}")
        return None

    @staticmethod
    def _emit(epoch, sv, sig_id, receiver_admits, our_admits,
              receiver_sig, our_diag):
        if our_admits and not receiver_admits:
            direction = "receiver=excluded our=admit"
        else:
            direction = "receiver=admitted our=exclude"
        # Receiver-side fields
        pr_res = receiver_sig.get('prRes')
        r_cno = receiver_sig.get('cno')
        health = receiver_sig.get('health')
        # Our-side fields (best-effort, all optional)
        lock_ms = our_diag.get('lock_time_ms')
        e_cno = our_diag.get('cno')
        gf = our_diag.get('gf_phase_m')
        mw = our_diag.get('mw_cycles_smoothed')
        pr_resid = our_diag.get('our_pr_resid_m')
        elev = our_diag.get('elev_deg')
        az = our_diag.get('az_deg')
        slip = our_diag.get('last_slip_reason')
        reason = our_diag.get('our_admission_reason')
        # Pack as key=val for grep-ability; omit None fields.
        fields = []
        for k, v in [
            ('lock_ms', lock_ms),
            ('cno_eng', e_cno),
            ('cno_rcv', r_cno),
            ('gf_m', gf),
            ('mw_c', mw),
            ('pr_resid_eng_m', pr_resid),
            ('pr_res_rcv_m', pr_res),
            ('health', health),
            ('elev', elev),
            ('az', az),
            ('slip', slip),
            ('our_reason', reason),
        ]:
            if v is None:
                continue
            if isinstance(v, float):
                fields.append(f"{k}={v:.3f}")
            else:
                fields.append(f"{k}={v}")
        log.info(
            "[NAV-SIG_DISAGREE ep=%d sv=%s sig=%s] %s %s",
            epoch, sv, sig_id, direction, " ".join(fields),
        )

    # ─── Diagnostics ─────────────────────────────────────────────── #

    def state_size(self) -> int:
        return len(self._last_state)

    def reset(self) -> None:
        self._last_state.clear()

"""NavSigDisagreeMonitor — Phase A.5 of slipDetectUnified-main.

Bob's directive (dayplan 2026-05-12 12:35 CDT): "we may use SVs the
receiver doesn't, and may exclude SVs it uses, but we MUST LOG when
we differ".  Pure logger — no exclusion behavior change.

The receiver's UBX-NAV-SIG message reports per-(SV, signal) usage
flags (``prUsed``, ``crUsed``, ``health``, ``prRes``, ``cno``,
``ionoModel``, ``corrUsed``).  When the receiver's verdict differs
from our engine's admit decision, the disagreement is data we use
in Phase B to tune engine-side exclusion.

## ``Nav2SignalStore`` interface (bravo, PR #26)

Bravo's ``realtime_ppp.Nav2SignalStore`` API consumed here::

    store.snapshot() -> dict[(sv_label, sig_name), SigStatus]
    store.get(sv_label, sig_name) -> SigStatus | None

where ``SigStatus`` has attributes (not dict keys):
    pr_used, cr_used, do_used, pr_smoothed, health,
    cno, quality_ind, pr_res_m, sig_name, host_mono, ...

``sv_label`` is the engine's standard "G07" / "E19" / "C42" form;
``sig_name`` matches the engine internal signal names ("GPS-L1CA",
"GAL-E5aQ", "BDS-B2aI", ...) when the driver supplied the lookup
table, else "sys{gnssId}/sig{sigId}" fallback.

A ``StubSignalStore`` is provided below for unit testing and for
back-compat when the engine runs without NAV-SIG plumbing.  It
mimics bravo's API surface — ``snapshot()``, ``get()``, and a
``set()`` helper for tests.

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


class _StubSigStatus:
    """Test-only SigStatus mimic — attribute access on a kwargs blob."""

    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)


class StubSignalStore:
    """Drop-in for Nav2SignalStore in unit tests.

    Mimics bravo's ``Nav2SignalStore`` API surface — ``snapshot()`` and
    ``get()``.  Pre-populate via ``store.set(sv, sig_name, pr_used=...,
    pr_res_m=..., cno=..., health=...)``.  Production engine plugs in
    bravo's real store.
    """

    def __init__(self) -> None:
        self._sigs: dict[tuple[str, str], _StubSigStatus] = {}

    def set(self, sv: str, sig_name: str, **fields: Any) -> None:
        self._sigs[(sv, sig_name)] = _StubSigStatus(**fields)

    def clear(self, sv: str, sig_name: str) -> None:
        self._sigs.pop((sv, sig_name), None)

    def get(self, sv: str, sig_name: str):
        return self._sigs.get((sv, sig_name))

    def snapshot(self):
        return dict(self._sigs)


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

        # Single snapshot per epoch — atomic w.r.t. signal-store updates,
        # and one lock acquisition for the whole walk.
        recv_snap = signal_store.snapshot()
        n_disagree = 0

        # Union of receiver-known and engine-admitted signals.  Catches
        # the rare 'we admit but receiver doesn't know' case.
        keys: set[tuple[str, str]] = set(recv_snap.keys())
        keys.update(our_admit_set.keys())

        for sv, sig_name in keys:
            receiver_sig = recv_snap.get((sv, sig_name))
            if receiver_sig is None:
                # We admitted but receiver doesn't even have a record.
                # Translate via UBX fallback name if applicable.
                receiver_sig = self._fallback_receiver_lookup(
                    sv, sig_name, recv_snap)
                if receiver_sig is None:
                    # Truly unknown to receiver — defer classification.
                    continue
            receiver_admits = bool(getattr(receiver_sig, 'pr_used', False))
            our_admits = (sv, sig_name) in our_admit_set
            new_state = (receiver_admits, our_admits)
            last = self._last_state.get((sv, sig_name))
            if last == new_state:
                continue
            self._last_state[(sv, sig_name)] = new_state
            if receiver_admits == our_admits:
                # Transitioned, but into agreement — no log spam.
                continue
            # Disagreement transition.
            n_disagree += 1
            our_diag = our_admit_set.get((sv, sig_name), {})
            self._emit(
                epoch=epoch,
                sv=sv,
                sig_id=sig_name,
                receiver_admits=receiver_admits,
                our_admits=our_admits,
                receiver_sig=receiver_sig,
                our_diag=our_diag,
            )
        return n_disagree

    @staticmethod
    def _fallback_receiver_lookup(sv, sig_name, recv_snap):
        """If engine name didn't match a snapshot key, try UBX numeric name.

        Bravo's store names signals via the driver-supplied (gnssId,
        sigId) → name map when present, else falls back to
        ``"sys{gnssId}/sig{sigId}"``.  When the driver hasn't populated
        the map yet, the snapshot may key on numeric names while the
        engine still uses internal names — try the translation.
        """
        ubx = engine_sig_to_ubx_sigid(sig_name)
        if ubx is None:
            return None
        return recv_snap.get((sv, f"sys{ubx[0]}/sig{ubx[1]}"))

    @staticmethod
    def _emit(epoch, sv, sig_id, receiver_admits, our_admits,
              receiver_sig, our_diag):
        if our_admits and not receiver_admits:
            direction = "receiver=excluded our=admit"
        else:
            direction = "receiver=admitted our=exclude"
        # Receiver-side fields (SigStatus attribute access)
        pr_res = getattr(receiver_sig, 'pr_res_m', None)
        r_cno = getattr(receiver_sig, 'cno', None)
        health = getattr(receiver_sig, 'health', None)
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

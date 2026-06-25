"""Unit tests for ntrip_reader bias-skip diagnostic flags.

Verifies that ``skip_biases`` / ``skip_code_biases`` / ``skip_phase_biases``
correctly drop messages before they reach SSRState, while non-bias messages
continue to be routed.

Why this matters: the cross-AC SSR diagnostic (2026-04-25) needs to isolate
which class of correction (orbit/clock vs code-bias vs phase-bias) drives
the systematic obs-model bias we see when CNES SSR is enabled.  These flags
are the engine-side substrate for that diagnostic.
"""

import sys
import threading
from pathlib import Path
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

from realtime_ppp import (
    BIAS_MSG_TYPES,
    CODE_BIAS_MSG_TYPES,
    EPH_MSG_TYPES,
    PHASE_BIAS_MSG_TYPES,
    SSR_MSG_TYPES,
    ntrip_reader,
)


# ── Fixtures ────────────────────────────────────────────────────────── #


def _fake_meta():
    return {
        "recv_mono": 0.0,
        "queue_remains": 0,
        "parse_age_s": 0.0,
        "correlation_confidence": 1.0,
        "estimator_residual_s": None,
    }


def _make_msg(identity):
    msg = MagicMock()
    msg.identity = identity
    return msg


def _fake_stream(identities):
    """Return a stub NtripStream emitting these identities once each."""
    stream = MagicMock()
    stream.messages_with_metadata.return_value = [
        (_make_msg(i), _fake_meta()) for i in identities
    ]
    return stream


def _run_reader(identities, **kwargs):
    """Run ntrip_reader against a fake stream, return (beph, ssr) call records."""
    stream = _fake_stream(identities)
    beph = MagicMock()
    ssr = MagicMock()
    stop_event = threading.Event()
    ntrip_reader(stream, beph, ssr, stop_event, label="TEST", **kwargs)
    return beph, ssr


# ── Smoke tests ────────────────────────────────────────────────────── #


def test_message_type_split_is_disjoint():
    assert not (CODE_BIAS_MSG_TYPES & PHASE_BIAS_MSG_TYPES), (
        "Code and phase bias message types must not overlap")
    assert CODE_BIAS_MSG_TYPES | PHASE_BIAS_MSG_TYPES == BIAS_MSG_TYPES


def test_default_routes_everything():
    """No skip flags: ephemeris, SSR, and biases all routed."""
    eph = next(iter(EPH_MSG_TYPES))
    ssr_msg = next(iter(SSR_MSG_TYPES - BIAS_MSG_TYPES))
    code = next(iter(CODE_BIAS_MSG_TYPES))
    phase = next(iter(PHASE_BIAS_MSG_TYPES))

    beph, ssr = _run_reader([eph, ssr_msg, code, phase])
    assert beph.update_from_rtcm.call_count == 1
    # ssr_msg + code + phase = 3 SSR routes (biases also go via SSR_MSG_TYPES)
    assert ssr.update_from_rtcm.call_count == 3


# ── --no-primary-biases (skip_biases=True) ─────────────────────────── #


def test_skip_biases_drops_both_classes():
    eph = next(iter(EPH_MSG_TYPES))
    ssr_msg = next(iter(SSR_MSG_TYPES - BIAS_MSG_TYPES))
    code = next(iter(CODE_BIAS_MSG_TYPES))
    phase = next(iter(PHASE_BIAS_MSG_TYPES))

    beph, ssr = _run_reader([eph, ssr_msg, code, phase], skip_biases=True)
    assert beph.update_from_rtcm.call_count == 1, "ephemeris must still route"
    # Only the non-bias SSR message reaches the SSR handler.
    assert ssr.update_from_rtcm.call_count == 1


# ── --no-ssr-code-bias / --no-ssr-phase-bias ───────────────────────── #


def test_skip_code_biases_drops_only_code():
    code = next(iter(CODE_BIAS_MSG_TYPES))
    phase = next(iter(PHASE_BIAS_MSG_TYPES))
    _, ssr = _run_reader([code, phase], skip_code_biases=True)
    assert ssr.update_from_rtcm.call_count == 1
    routed_msg = ssr.update_from_rtcm.call_args[0][0]
    assert routed_msg.identity == phase


def test_skip_phase_biases_drops_only_phase():
    code = next(iter(CODE_BIAS_MSG_TYPES))
    phase = next(iter(PHASE_BIAS_MSG_TYPES))
    _, ssr = _run_reader([code, phase], skip_phase_biases=True)
    assert ssr.update_from_rtcm.call_count == 1
    routed_msg = ssr.update_from_rtcm.call_args[0][0]
    assert routed_msg.identity == code


def test_skip_both_classes_drops_all_biases():
    code = next(iter(CODE_BIAS_MSG_TYPES))
    phase = next(iter(PHASE_BIAS_MSG_TYPES))
    ssr_msg = next(iter(SSR_MSG_TYPES - BIAS_MSG_TYPES))
    _, ssr = _run_reader([code, phase, ssr_msg],
                         skip_code_biases=True, skip_phase_biases=True)
    assert ssr.update_from_rtcm.call_count == 1
    assert ssr.update_from_rtcm.call_args[0][0].identity == ssr_msg


# ── bias_only mode interactions ────────────────────────────────────── #


def test_bias_only_with_skip_phase_routes_only_code():
    """Secondary mount in bias_only=True; --no-ssr-phase-bias drops phase."""
    code = next(iter(CODE_BIAS_MSG_TYPES))
    phase = next(iter(PHASE_BIAS_MSG_TYPES))
    eph = next(iter(EPH_MSG_TYPES))  # already excluded by bias_only
    _, ssr = _run_reader([code, phase, eph],
                         bias_only=True, skip_phase_biases=True)
    assert ssr.update_from_rtcm.call_count == 1
    assert ssr.update_from_rtcm.call_args[0][0].identity == code


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))


# ── pos_replay raw-capture tap ──────────────────────────────────────── #


def test_ntrip_reader_taps_raw_frames_with_canonical_recv_mono(tmp_path):
    """The SSR/eph tap records each raw RTCM frame with the canonical
    meta["recv_mono"] (no clock re-read, no re-serialization)."""
    import os
    from peppar_fix.raw_capture import RawCaptureBundle, read_stream

    bundle = RawCaptureBundle(str(tmp_path))
    stream = MagicMock()
    stream.messages_with_metadata.return_value = [
        (_make_msg("1057"), {**_fake_meta(), "recv_mono": 10.0, "raw": b"\xd3aaa"}),
        (_make_msg("1058"), {**_fake_meta(), "recv_mono": 11.5, "raw": b"\xd3bbb"}),
    ]
    ntrip_reader(stream, MagicMock(), MagicMock(), threading.Event(),
                 label="SSR", raw_bundle=bundle, raw_stream="ssr")
    bundle.close()
    recs = list(read_stream(os.path.join(str(tmp_path), "raw", "ssr.cap")))
    assert recs == [(10.0, b"\xd3aaa"), (11.5, b"\xd3bbb")]


def test_ntrip_reader_no_capture_when_meta_lacks_raw(tmp_path):
    """A stream whose meta has no 'raw' key (pre-tap) is a safe no-op."""
    import os
    from peppar_fix.raw_capture import RawCaptureBundle

    bundle = RawCaptureBundle(str(tmp_path))
    _run_reader(["1057"], raw_bundle=bundle, raw_stream="ssr")  # _fake_meta has no raw
    bundle.close()
    assert not os.path.exists(os.path.join(str(tmp_path), "raw", "ssr.cap"))

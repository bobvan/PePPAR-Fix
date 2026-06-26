"""Unit tests for route_rtcm_message — the shared RTCM router used by both the
live ntrip_reader and pos_replay (so replay routing can't drift from live)."""
import sys
from pathlib import Path
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

from realtime_ppp import (  # noqa: E402
    route_rtcm_message,
    EPH_MSG_TYPES,
    CODE_BIAS_MSG_TYPES,
    PHASE_BIAS_MSG_TYPES,
)


def _stores():
    beph = MagicMock()
    beph.update_from_rtcm.return_value = 7          # a prn
    beph.n_satellites = 10
    ssr = MagicMock()
    ssr.update_from_rtcm.return_value = "ok"
    return beph, ssr


def _eph_id():
    return sorted(EPH_MSG_TYPES)[0]                 # e.g. '1019'


def _code_bias_id():
    return sorted(CODE_BIAS_MSG_TYPES)[0]


def _phase_bias_id():
    return sorted(PHASE_BIAS_MSG_TYPES)[0]


def test_eph_routes_to_beph():
    beph, ssr = _stores()
    tag, detail = route_rtcm_message(_eph_id(), object(), beph, ssr, "L")
    assert tag == "eph" and detail == 7
    beph.update_from_rtcm.assert_called_once()
    ssr.update_from_rtcm.assert_not_called()


def test_ssr_4076_routes_to_ssr_with_src_mount():
    beph, ssr = _stores()
    mv = object()
    tag, detail = route_rtcm_message("4076_021", mv, beph, ssr, "CNES")
    assert tag == "ssr" and detail == "ok"
    ssr.update_from_rtcm.assert_called_once_with(mv, src_mount="CNES")


def test_unknown_identity_ignored():
    beph, ssr = _stores()
    tag, _ = route_rtcm_message("1005", object(), beph, ssr, "L")
    assert tag == "ignored"
    beph.update_from_rtcm.assert_not_called()
    ssr.update_from_rtcm.assert_not_called()


def test_skip_code_bias():
    beph, ssr = _stores()
    tag, _ = route_rtcm_message(_code_bias_id(), object(), beph, ssr, "L",
                                skip_code_biases=True)
    assert tag == "skip_code_bias"
    ssr.update_from_rtcm.assert_not_called()


def test_skip_phase_bias():
    beph, ssr = _stores()
    tag, _ = route_rtcm_message(_phase_bias_id(), object(), beph, ssr, "L",
                                skip_phase_biases=True)
    assert tag == "skip_phase_bias"
    ssr.update_from_rtcm.assert_not_called()


def test_skip_all_biases():
    beph, ssr = _stores()
    tag, _ = route_rtcm_message(_code_bias_id(), object(), beph, ssr, "L",
                                skip_biases=True)
    assert tag == "skip_bias"
    ssr.update_from_rtcm.assert_not_called()


def test_bias_only_routes_bias_gapfill():
    beph, ssr = _stores()
    mv = object()
    tag, detail = route_rtcm_message(_phase_bias_id(), mv, beph, ssr, "WHU",
                                     bias_only=True)
    assert tag == "bias_gapfill" and detail == "ok"
    ssr.update_from_rtcm.assert_called_once_with(mv, src_mount="WHU",
                                                 gap_fill_only=True)


def test_bias_only_skips_non_bias():
    beph, ssr = _stores()
    # in bias_only mode, a non-bias (eph) message is dropped
    tag, _ = route_rtcm_message(_eph_id(), object(), beph, ssr, "WHU",
                                bias_only=True)
    assert tag == "skip_non_bias"
    beph.update_from_rtcm.assert_not_called()
    ssr.update_from_rtcm.assert_not_called()

"""Tests for rinex_writer.RinexWriter — round-trip through rinex_reader.

Why this file exists: we need to produce RINEX OBS files the regression
harness (and PRIDE-PPPAR) can consume, for the cross-AC bug-vs-datum
investigation.  The cheapest correctness test is to write a synthetic
session with known observation values, parse it back through our
existing rinex_reader, and verify the values come back unchanged.
"""

import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
from peppar_fix.rinex_writer import RinexWriter
from regression.rinex_reader import parse_header, iter_epochs as parse_epochs


@pytest.fixture
def tmp_rnx(tmp_path):
    return tmp_path / "session.rnx"


def _make_writer(tmp_rnx):
    return RinexWriter(
        tmp_rnx,
        marker_name="UFO1",
        approx_xyz=(157544.0, -4756190.0, 4232770.0),
        antenna_type="SFESPK6618H     NONE",
        receiver_model="ZED-F9T",
        receiver_fw="TIM 2.25",
        receiver_serial="3b41fabd5b",
        antenna_serial="UFO1-A",
        observer="test",
        agency="test-lab",
        interval_s=1.0,
    )


def test_version_line_col21_is_obs_type(tmp_rnx):
    """RINEX 3 spec § 5.1: the OBS file type marker ('O') must land
    at column 21 of the VERSION / TYPE line.  PRIDE-PPP-AR's pdp3.sh
    enforces this via `cut -c 21-21 == "O"`; getting it wrong makes
    every .obs file unconsumable by pdp3 (validated on MadHat 2026-05-18).
    """
    w = _make_writer(tmp_rnx)
    epoch = datetime(2026, 4, 25, 22, 0, 0, tzinfo=timezone.utc)
    w.write_epoch(epoch, {
        "G16": {
            "GPS-L1CA": {"pr": 23e6, "cp": 1.2e8, "cno": 45.0,
                         "half_cyc": True, "lock_ms": 5000},
        },
    })
    w.close()
    with open(tmp_rnx) as f:
        first_line = f.readline()
    # Spec: cols 1-9 version, 10-20 blanks, col 21 = 'O'
    assert first_line[20] == "O", (
        f"col 21 (0-indexed 20) is {first_line[20]!r}, expected 'O'.  "
        f"First line: {first_line!r}")
    # Sanity: label ends at col 80
    assert first_line[60:80] == "RINEX VERSION / TYPE"


def test_header_round_trip(tmp_rnx):
    """Write minimal session, parse header back, check fields."""
    w = _make_writer(tmp_rnx)
    epoch = datetime(2026, 4, 25, 22, 0, 0, tzinfo=timezone.utc)
    raw_obs = {
        "G16": {
            "GPS-L1CA": {"pr": 23000000.0, "cp": 120500000.0, "cno": 45.0,
                         "half_cyc": True, "lock_ms": 5000},
            "GPS-L5Q":  {"pr": 23000000.5, "cp": 90100000.0, "cno": 42.0,
                         "half_cyc": True, "lock_ms": 5000},
        },
    }
    w.write_epoch(epoch, raw_obs)
    w.close()

    hdr = parse_header(tmp_rnx)
    assert hdr.version == "3.04"
    assert hdr.marker == "UFO1"
    assert hdr.approx_xyz == pytest.approx((157544.0, -4756190.0, 4232770.0))
    assert "G" in hdr.sys_obs_types
    # GPS column declarations include both L1C and L5Q phase types
    assert "L1C" in hdr.sys_obs_types["G"]
    assert "L5Q" in hdr.sys_obs_types["G"]


def test_observation_values_round_trip(tmp_rnx):
    """Values written should come back via the reader within float precision."""
    w = _make_writer(tmp_rnx)
    epoch = datetime(2026, 4, 25, 22, 0, 0, tzinfo=timezone.utc)
    raw_obs = {
        "G16": {
            "GPS-L1CA": {"pr": 23456789.123, "cp": 123456789.456,
                         "cno": 45.0, "half_cyc": True, "lock_ms": 5000},
        },
        "E04": {
            "GAL-E1C":  {"pr": 24112233.987, "cp": 126987654.321,
                         "cno": 41.0, "half_cyc": True, "lock_ms": 5000},
            "GAL-E5aQ": {"pr": 24112234.654, "cp": 94888777.222,
                         "cno": 39.0, "half_cyc": True, "lock_ms": 5000},
        },
    }
    w.write_epoch(epoch, raw_obs)
    w.close()

    epochs = list(parse_epochs(tmp_rnx))
    assert len(epochs) == 1
    obs = epochs[0].obs

    # G16 L1CA: PR + CP within 1 mm (RINEX has 3-decimal precision)
    g16_pr = obs["G16"]["C1C"][0]
    g16_cp = obs["G16"]["L1C"][0]
    assert g16_pr == pytest.approx(23456789.123, abs=1e-3)
    assert g16_cp == pytest.approx(123456789.456, abs=1e-3)

    # E04 dual-band
    e04_pr1 = obs["E04"]["C1C"][0]
    e04_pr2 = obs["E04"]["C5Q"][0]
    e04_cp1 = obs["E04"]["L1C"][0]
    e04_cp2 = obs["E04"]["L5Q"][0]
    assert e04_pr1 == pytest.approx(24112233.987, abs=1e-3)
    assert e04_pr2 == pytest.approx(24112234.654, abs=1e-3)
    assert e04_cp1 == pytest.approx(126987654.321, abs=1e-3)
    assert e04_cp2 == pytest.approx(94888777.222, abs=1e-3)


def test_lli_set_on_lock_drop(tmp_rnx):
    """A drop in lock_ms between epochs should set LLI bit 0."""
    w = _make_writer(tmp_rnx)
    e1 = datetime(2026, 4, 25, 22, 0, 0, tzinfo=timezone.utc)
    e2 = datetime(2026, 4, 25, 22, 0, 1, tzinfo=timezone.utc)
    obs = {
        "G16": {"GPS-L1CA": {"pr": 23e6, "cp": 1.2e8, "cno": 45.0,
                              "half_cyc": True, "lock_ms": 5000}},
    }
    w.write_epoch(e1, obs)
    obs2 = {
        "G16": {"GPS-L1CA": {"pr": 23e6, "cp": 1.2e8, "cno": 45.0,
                              "half_cyc": True, "lock_ms": 100}},  # dropped
    }
    w.write_epoch(e2, obs2)
    w.close()

    epochs = list(parse_epochs(tmp_rnx))
    assert len(epochs) == 2
    # Epoch 1 LLI should be 0 (no prior); epoch 2 LLI should be 1
    lli_e1 = epochs[0].obs["G16"]["L1C"][1]
    lli_e2 = epochs[1].obs["G16"]["L1C"][1]
    assert lli_e1 == 0
    assert lli_e2 == 1


def test_half_cyc_sets_lli_bit_1(tmp_rnx):
    """half_cyc=False sets LLI bit 1 (half-cycle ambiguity)."""
    w = _make_writer(tmp_rnx)
    epoch = datetime(2026, 4, 25, 22, 0, 0, tzinfo=timezone.utc)
    obs = {
        "G16": {"GPS-L1CA": {"pr": 23e6, "cp": 1.2e8, "cno": 45.0,
                              "half_cyc": False, "lock_ms": 5000}},
    }
    w.write_epoch(epoch, obs)
    w.close()
    epochs = list(parse_epochs(tmp_rnx))
    lli = epochs[0].obs["G16"]["L1C"][1]
    assert lli & 2, f"LLI bit 1 should be set; got LLI={lli}"


def test_unknown_signal_is_skipped(tmp_rnx):
    """An unknown internal sig name gets dropped, doesn't crash."""
    w = _make_writer(tmp_rnx)
    epoch = datetime(2026, 4, 25, 22, 0, 0, tzinfo=timezone.utc)
    raw_obs = {
        "G16": {
            "GPS-L1CA": {"pr": 23e6, "cp": 1.2e8, "cno": 45.0,
                         "half_cyc": True, "lock_ms": 5000},
            "GPS-NONESUCH": {"pr": 1.0, "cp": 2.0, "cno": 0.0,
                             "half_cyc": True, "lock_ms": 0},
        },
    }
    w.write_epoch(epoch, raw_obs)
    w.close()
    epochs = list(parse_epochs(tmp_rnx))
    # Known signal still came through
    assert epochs[0].obs["G16"]["L1C"][0] == pytest.approx(1.2e8, abs=1e-3)


# ── make_writer_from_args smoke tests ─────────────────────────────── #
#
# These cover the bug shapes Bravo found and fixed in commits 1471216
# + 8fa49fb on engine commit 9597c51:
#   * NameError on undefined receiver_state at the rinex_writer init
#   * UnboundLocalError on known_ecef referenced before assignment
#
# Both were engine-side scope errors, not RinexWriter API bugs.  The
# fix was to move all init logic into make_writer_from_args() which
# uses getattr/.get() defaults — so missing or unset args don't raise.
# These tests exercise that helper with progressively-sparser args
# Namespaces to verify nothing in the init path requires more state
# than the helper signature promises.


from argparse import Namespace
from peppar_fix.rinex_writer import make_writer_from_args


def test_make_writer_returns_none_when_rinex_out_unset(tmp_path):
    """No --rinex-out → no writer.  No exception either."""
    args = Namespace(rinex_out=None)
    assert make_writer_from_args(args) is None


def test_make_writer_returns_none_when_args_missing_attr(tmp_path):
    """Args namespace without rinex_out at all → None.  Was a NameError
    risk before the helper centralized the lookup."""
    args = Namespace()
    assert make_writer_from_args(args) is None


def test_make_writer_minimal_args_no_optional_state(tmp_path):
    """Bare minimum: only rinex_out is set; no known_pos, no
    receiver metadata, no antenna identifier.  The helper must not
    raise NameError or UnboundLocalError on any unset optional input.
    This is the regression test for Bravo's 1471216 + 8fa49fb fixes."""
    out = tmp_path / "minimal.rnx"
    args = Namespace(rinex_out=str(out))
    w = make_writer_from_args(args)
    assert w is not None
    # Header fields fall back to module defaults.
    assert w._marker == "UFO1"  # default
    assert w._antenna_type == "SFESPK6618H     NONE"  # default
    assert w._approx_xyz == (0.0, 0.0, 0.0)  # no known_pos
    w.close()


def test_make_writer_with_known_pos_derives_approx_xyz(tmp_path):
    """When --known-pos is supplied, approx_xyz comes from LLA→ECEF.
    Catches the UnboundLocalError-on-known_ecef bug shape: known_pos
    parsing must happen inline in the helper, not depend on a variable
    populated elsewhere in the engine's main()."""
    out = tmp_path / "known.rnx"
    args = Namespace(
        rinex_out=str(out),
        known_pos="40.0,-90.0,200.0",  # placeholder — coords gitignored
    )
    w = make_writer_from_args(args)
    assert w is not None
    # ECEF for (40°, -90°, 200m) ≈ (0, -4_892_861, 4_078_114)
    x, y, z = w._approx_xyz
    assert abs(x) < 100, f"X={x} expected near 0 for lon=-90"
    assert -4_900_000 < y < -4_880_000
    assert 4_070_000 < z < 4_090_000
    w.close()


def test_make_writer_with_full_engine_args(tmp_path):
    """Engine-realistic args: --rinex-out + --known-pos + --receiver-
    antenna + --peer-antenna-ref.  All fields propagate to header
    defaults correctly."""
    out = tmp_path / "full.rnx"
    args = Namespace(
        rinex_out=str(out),
        known_pos="40.0,-90.0,200.0",  # placeholder
        receiver_antenna="SFESPK6618H     NONE",
        peer_antenna_ref="UFO1",
    )
    w = make_writer_from_args(args)
    assert w is not None
    assert w._marker == "UFO1"
    assert w._antenna_type == "SFESPK6618H     NONE"
    w.close()


def test_make_writer_known_pos_parse_failure_doesnt_raise(tmp_path):
    """Garbage --known-pos string: helper should warn but still build
    a writer with default approx_xyz.  The engine's startup must not
    fail on a bad config string."""
    out = tmp_path / "bad-pos.rnx"
    args = Namespace(
        rinex_out=str(out),
        known_pos="this,is,not,coordinates",
    )
    # Should NOT raise — helper logs a warning and falls back.
    w = make_writer_from_args(args)
    assert w is not None
    assert w._approx_xyz == (0.0, 0.0, 0.0)
    w.close()


def _writer_with_template(tmp_path, decimate_s=None):
    return RinexWriter(
        str(tmp_path / "{host}-{date}.obs"),
        marker_name="UFO1",
        approx_xyz=(157470.222, -4756189.544, 4232767.952),
        antenna_type="SFESPK6618H     NONE",
        host="madhat",
        decimate_s=decimate_s,
    )


def _stub_obs():
    """Smallest non-empty raw_obs that produces a written epoch."""
    return {"G16": {"GPS-L1CA": {"pr": 21000000.0, "cp": 110000000.0,
                                  "cno": 45.0, "half_cyc": True,
                                  "lock_ms": 60000}}}


def test_path_template_expansion(tmp_path):
    w = _writer_with_template(tmp_path)
    epoch = datetime(2026, 5, 4, 14, 0, 0, tzinfo=timezone.utc)
    w.write_epoch(epoch, _stub_obs())
    w.close()
    expected = tmp_path / "madhat-2026124.obs"  # DOY 124 = May 4
    assert expected.exists(), f"path template not expanded; files: {list(tmp_path.iterdir())}"


def test_utc_midnight_rotation(tmp_path):
    """Two epochs straddling UTC midnight produce two files with the
    same writer instance.  Rotation is automatic — caller doesn't have
    to do anything special."""
    w = _writer_with_template(tmp_path)
    # 23:59:30 UTC May 4 → DOY 124
    w.write_epoch(datetime(2026, 5, 4, 23, 59, 30, tzinfo=timezone.utc),
                  _stub_obs())
    # 00:00:00 UTC May 5 → DOY 125 — should rotate
    w.write_epoch(datetime(2026, 5, 5, 0, 0, 0, tzinfo=timezone.utc),
                  _stub_obs())
    w.close()
    files = sorted(p.name for p in tmp_path.iterdir() if p.suffix == ".obs")
    assert files == ["madhat-2026124.obs", "madhat-2026125.obs"], files
    # Both files have full headers (END OF HEADER + at least one epoch).
    for fn in files:
        text = (tmp_path / fn).read_text()
        assert "END OF HEADER" in text
        assert "> 2026" in text  # at least one epoch line


def test_decimation_drops_close_epochs(tmp_path):
    """decimate_s = 30 means epochs less than 30s apart are dropped
    after the first.  The first epoch is ALWAYS written."""
    w = _writer_with_template(tmp_path, decimate_s=30.0)
    base = datetime(2026, 5, 4, 14, 0, 0, tzinfo=timezone.utc)
    # 4 epochs at 1s intervals — only first should be written
    for i in range(4):
        from datetime import timedelta
        w.write_epoch(base + timedelta(seconds=i), _stub_obs())
    # then jump 30s → second epoch written
    from datetime import timedelta
    w.write_epoch(base + timedelta(seconds=30), _stub_obs())
    # then 35s → third (because 35-30 = 5s < 30s no, wait: last_written is at +30, +35-30=5 < 30 → dropped)
    w.write_epoch(base + timedelta(seconds=35), _stub_obs())
    # +60 → fourth written (60-30 = 30s ≥ 30)
    w.write_epoch(base + timedelta(seconds=60), _stub_obs())
    w.close()
    text = (tmp_path / "madhat-2026124.obs").read_text()
    epoch_lines = [ln for ln in text.splitlines() if ln.startswith("> ")]
    assert len(epoch_lines) == 3, f"expected 3 epochs (0, 30, 60), got {len(epoch_lines)}: {epoch_lines}"


def test_decimation_first_epoch_always_written(tmp_path):
    """Even with aggressive decimation, the first epoch must be
    written so the file has a header + first obs."""
    w = _writer_with_template(tmp_path, decimate_s=300.0)
    epoch = datetime(2026, 5, 4, 14, 0, 0, tzinfo=timezone.utc)
    w.write_epoch(epoch, _stub_obs())
    w.close()
    text = (tmp_path / "madhat-2026124.obs").read_text()
    assert "END OF HEADER" in text
    assert "> 2026  5  4 14  0" in text


def test_decimation_header_interval(tmp_path):
    """Header INTERVAL should reflect decimated rate, not 1Hz, since
    that's what consumers see in the file."""
    w = _writer_with_template(tmp_path, decimate_s=30.0)
    w.write_epoch(datetime(2026, 5, 4, 14, 0, 0, tzinfo=timezone.utc),
                  _stub_obs())
    w.close()
    text = (tmp_path / "madhat-2026124.obs").read_text()
    interval_line = [ln for ln in text.splitlines() if "INTERVAL" in ln]
    assert interval_line, f"no INTERVAL line: {text[:500]}"
    # "    30.000" with INTERVAL marker
    assert "30.000" in interval_line[0]


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))

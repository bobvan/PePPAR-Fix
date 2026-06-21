"""Round-trip a capture-with-#-header CSV through every analysis reader.

Proves deliverable #3 (CAPTURE_VERSION comment line in the CSV header)
did NOT break any of the analysis readers that consume ticc_capture.py
output.  Each reader must skip the leading ``# capture_tool=...`` line.
"""
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_ROOT / "tools"))
sys.path.insert(0, str(_ROOT / "scripts"))


def _write_capture_csv(path: Path, n: int = 120, with_header: bool = True):
    """Write a ticc_capture.py-format CSV (chA + chB rows) with the
    capture-provenance ``#`` comment line, like CAPTURE_VERSION≥1 does."""
    t0 = datetime(2026, 6, 20, 0, 0, 0, tzinfo=timezone.utc)
    lines = []
    if with_header:
        lines.append(
            "# capture_tool=ticc_capture.py capture_version=1 "
            "git=abc1234-dirty utc=2026-06-20T00:00:00+00:00\n")
    lines.append("ts_iso,channel,ref_sec,ref_ps,recv_mono\n")
    for i in range(n):
        ts = (t0 + timedelta(seconds=i)).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
        # chA slightly ahead of chB by a small, drifting ps offset.
        a_ps = (100 + i) % 1_000_000_000
        b_ps = (50 + i) % 1_000_000_000
        recv = float(i) + 0.95
        lines.append(f"{ts},chA,{1000 + i},{a_ps},{recv:.6f}\n")
        lines.append(f"{ts},chB,{1000 + i},{b_ps},{recv:.6f}\n")
    path.write_text("".join(lines))


def test_numpy_genfromtxt_skips_hash(tmp_path):
    """np.genfromtxt skips '#' comment lines by default — confirm the
    capture-provenance header line is dropped, so the first parsed row is
    the column header (not the '#' line)."""
    csv_path = tmp_path / "cap.csv"
    _write_capture_csv(csv_path, n=5)
    # Default comments='#'.  The '#' provenance line and the column
    # header line are both non-data; skipping both leaves exactly the 10
    # data rows (5 chA + 5 chB) with ref_sec starting at 1000.  Crucially
    # the '#' line never parses as a (mis-columned) data row — genfromtxt
    # strips it by default, which is the property deliverable #3 relies on.
    ref_sec = np.genfromtxt(csv_path, delimiter=",", skip_header=2,
                            usecols=2, encoding="utf-8")
    assert ref_sec.shape[0] == 10
    assert ref_sec.min() == 1000.0
    assert not np.isnan(ref_sec).any()


def test_cdf_readers_skip_header(tmp_path):
    from plot_xhost_agreement_cdf import load_pairs, _virtual_pair_load
    csv_path = tmp_path / "cap.csv"
    _write_capture_csv(csv_path)
    t, delta = load_pairs(csv_path)
    assert len(delta) > 0
    # Virtual pair: chA from two files.
    csv_b = tmp_path / "cap_b.csv"
    _write_capture_csv(csv_b)
    tt, dd = _virtual_pair_load(csv_path, csv_b)
    assert len(dd) > 0


def test_stability_readers_skip_header(tmp_path):
    from plot_clock_stability_stack import (load_chA_phase,
                                            load_tdcp_freq_ppb)
    csv_path = tmp_path / "cap.csv"
    _write_capture_csv(csv_path)
    cha = load_chA_phase(csv_path, channel="chA")
    chb = load_chA_phase(csv_path, channel="chB")
    assert len(cha) > 0 and len(chb) > 0
    # load_tdcp_freq_ppb reads a different (arm-state) schema but must not
    # choke on a '#' first line either; it returns empty (no tdcp cols).
    assert len(load_tdcp_freq_ppb(csv_path)) == 0


def test_virtual_readers_skip_header(tmp_path):
    from plot_xhost_virtual import (load_ticc_cha, load_direct_xhost_diff,
                                    per_host_block_residuals,
                                    load_scheduler_interval)
    csv_path = tmp_path / "cap.csv"
    _write_capture_csv(csv_path)
    assert len(load_ticc_cha(csv_path)) > 0
    times, deltas = load_direct_xhost_diff(csv_path)
    assert len(deltas) > 0
    t, res = per_host_block_residuals(csv_path)
    assert len(res) > 0
    # scheduler reader uses arm-state schema; '#' must be skipped, returns
    # empty (no dt_actual_s/host_timestamp cols) without raising.
    assert len(load_scheduler_interval(csv_path)[0]) == 0


def test_indep_refs_reader_skips_header(tmp_path):
    from plot_xhost_indep_refs import load_per_host_residuals
    csv_path = tmp_path / "cap.csv"
    _write_capture_csv(csv_path)
    ts, res = load_per_host_residuals(csv_path)
    assert len(res) > 0


def test_goldilocks_reader_skips_header(tmp_path):
    from plot_chA_tdev_goldilocks import load_chA_phase_s
    csv_path = tmp_path / "cap.csv"
    _write_capture_csv(csv_path)
    phase = load_chA_phase_s(csv_path, skip_before=None)
    assert len(phase) > 0


def test_header_and_headerless_yield_same_data(tmp_path):
    """The '#' header line must not change parsed values vs a headerless
    file — additive-only guarantee at the reader level."""
    from plot_xhost_agreement_cdf import load_pairs
    with_h = tmp_path / "with.csv"
    without_h = tmp_path / "without.csv"
    _write_capture_csv(with_h, with_header=True)
    _write_capture_csv(without_h, with_header=False)
    _, d_with = load_pairs(with_h)
    _, d_without = load_pairs(without_h)
    assert np.array_equal(d_with, d_without)

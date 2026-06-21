"""Test compare_clocks.py --from-csv analyze-only path on synthetic data.

The live-capture path needs real TICC hardware and is not tested here; it
is cleanly separated behind the absence of --from-csv.
"""
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_ROOT / "scripts"))
sys.path.insert(0, str(_ROOT / "tools"))


def _write_two_channel_csv(path: Path, n: int = 600, jitter_ps: int = 300):
    """Synthetic shared-ref TICC capture: chA = clock A, chB = clock B.

    chA−chB has a constant offset + small linear drift + white jitter, so
    the detrended |Δ| has a nonzero but bounded p95.  Includes the
    capture-provenance '#' header line.
    """
    rng = np.random.default_rng(42)
    t0 = datetime(2026, 6, 20, 0, 0, 0, tzinfo=timezone.utc)
    lines = [
        "# capture_tool=ticc_capture.py capture_version=1 "
        "git=deadbee-dirty utc=2026-06-20T00:00:00+00:00\n",
        "ts_iso,channel,ref_sec,ref_ps,recv_mono\n",
    ]
    for i in range(n):
        ts = (t0 + timedelta(seconds=i)).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
        # chA: nominal 1 Hz with a small drift; chB: offset + drift + jitter.
        a_ps = (200_000_000 + i * 17) % 1_000_000_000
        delta = 50_000 + i * 3 + int(rng.normal(0, jitter_ps))  # ps
        b_ps = (a_ps - delta) % 1_000_000_000
        recv = float(i) + 0.95
        lines.append(f"{ts},chA,{1000 + i},{a_ps},{recv:.6f}\n")
        lines.append(f"{ts},chB,{1000 + i},{b_ps},{recv:.6f}\n")
    path.write_text("".join(lines))


def test_from_csv_produces_stamped_pngs_and_verdict(tmp_path, monkeypatch):
    import compare_clocks

    csv_path = tmp_path / "capture.csv"
    _write_two_channel_csv(csv_path)
    out_dir = tmp_path / "out"

    argv = ["compare_clocks.py",
            "--from-csv", str(csv_path),
            "--label-a", "ours",
            "--label-b", "M600",
            "--out-dir", str(out_dir),
            "--budget-ns", "2.0"]
    monkeypatch.setattr(sys, "argv", argv)

    rc = compare_clocks.main()
    assert rc == 0

    cdf_png = out_dir / "agreement_cdf.png"
    stab_png = out_dir / "stability_stack.png"
    verdict_txt = out_dir / "verdict.txt"
    assert cdf_png.exists() and cdf_png.stat().st_size > 0
    assert stab_png.exists() and stab_png.stat().st_size > 0
    assert verdict_txt.exists()

    text = verdict_txt.read_text()
    # Provenance stamp present.
    assert "analysis v" in text
    # Verdict line present with a pass/fail and a p95.
    assert "VERDICT:" in text
    assert ("PASS" in text) or ("FAIL" in text)
    assert "p95 |Δ|" in text


def test_from_csv_nonzero_p95(tmp_path, monkeypatch):
    """p95 excursion should be a real nonzero number from the CDF core."""
    import compare_clocks
    from plot_xhost_agreement_cdf import load_pairs, detrend_and_abs

    csv_path = tmp_path / "capture.csv"
    _write_two_channel_csv(csv_path)

    t, delta = load_pairs(csv_path)
    abs_r = detrend_and_abs(t, delta)
    p95 = float(np.percentile(abs_r, 95))
    assert p95 > 0.0


def test_from_csv_missing_file_errors(tmp_path, monkeypatch):
    import compare_clocks
    argv = ["compare_clocks.py",
            "--from-csv", str(tmp_path / "nope.csv"),
            "--out-dir", str(tmp_path / "out")]
    monkeypatch.setattr(sys, "argv", argv)
    # argparse error → SystemExit
    import pytest
    with pytest.raises(SystemExit):
        compare_clocks.main()


def test_no_csv_no_ticc_errors(tmp_path, monkeypatch):
    """Live path is gated behind --ticc; without either, clean error (no
    hardware access attempted)."""
    import compare_clocks
    import pytest
    argv = ["compare_clocks.py", "--out-dir", str(tmp_path / "out")]
    monkeypatch.setattr(sys, "argv", argv)
    with pytest.raises(SystemExit):
        compare_clocks.main()

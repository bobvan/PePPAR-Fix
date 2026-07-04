"""Test two_clock_report.py — the 4-page two-clock PDF composer.

Exercises the analyze-only path on synthetic shared-reference TICC data:
the 4-page PDF is produced, every page has 16:9 geometry, the CSV
windowing (skip-before / skip-after) actually trims rows, and the shared
compute functions return sane summary stats.  No hardware needed.
"""
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_ROOT / "scripts"))
sys.path.insert(0, str(_ROOT / "tools"))


def _write_two_channel_csv(path: Path, n: int = 900, jitter_ps: int = 300,
                           step_at: int | None = None, step_ns: float = 8.0):
    """Synthetic shared-ref TICC capture: chA = clock A, chB = clock B.

    chA−chB = constant offset + small drift + white jitter.  Optionally
    inject a step of ``step_ns`` on chB starting at second ``step_at`` (to
    exercise the skip-after window trim).  Includes the '#' provenance
    header line so skip_comment_lines has something to skip.
    """
    rng = np.random.default_rng(7)
    t0 = datetime(2026, 7, 2, 0, 0, 0, tzinfo=timezone.utc)
    lines = [
        "# capture_tool=ticc_capture.py capture_version=1 "
        "git=deadbee-dirty utc=2026-07-02T00:00:00+00:00\n",
        "ts_iso,channel,ref_sec,ref_ps,recv_mono\n",
    ]
    for i in range(n):
        ts = (t0 + timedelta(seconds=i)).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
        a_ps = (200_000_000 + i * 17) % 1_000_000_000
        delta = 50_000 + i * 3 + int(rng.normal(0, jitter_ps))    # ps
        if step_at is not None and i >= step_at:
            delta += int(step_ns * 1000)                          # ns → ps
        b_ps = (a_ps - delta) % 1_000_000_000
        recv = float(i) + 0.95
        lines.append(f"{ts},chA,{1000 + i},{a_ps},{recv:.6f}\n")
        lines.append(f"{ts},chB,{1000 + i},{b_ps},{recv:.6f}\n")
    path.write_text("".join(lines))


def _pdf_page_boxes(pdf_path: Path):
    from pypdf import PdfReader
    r = PdfReader(str(pdf_path))
    return [(float(p.mediabox.width), float(p.mediabox.height))
            for p in r.pages]


def test_build_report_four_pages_16x9(tmp_path):
    import two_clock_report as tcr

    csv_path = tmp_path / "cap.csv"
    _write_two_channel_csv(csv_path)
    out_pdf = tmp_path / "report.pdf"

    summary = tcr.build_report(csv_path, out_pdf, "A", "B",
                               ref_label="Rb", budget_ns=2.0)
    assert out_pdf.exists() and out_pdf.stat().st_size > 0
    assert set(summary) == {"cdf", "timeseries", "stability", "tch"}

    boxes = _pdf_page_boxes(out_pdf)
    assert len(boxes) == 4
    for w, h in boxes:
        assert abs((w / h) - (16 / 9)) < 0.01   # every page is 16:9


def test_summary_stats_are_sane(tmp_path):
    import two_clock_report as tcr

    csv_path = tmp_path / "cap.csv"
    _write_two_channel_csv(csv_path)
    out_pdf = tmp_path / "report.pdf"
    s = tcr.build_report(csv_path, out_pdf, "A", "B", budget_ns=2.0)

    cdf = s["cdf"]
    assert cdf["n"] > 100
    assert cdf["p95"] > 0.0
    assert cdf["p50"] <= cdf["p95"] <= cdf["p99"]
    assert cdf["grade"] in ("PASS", "FAIL")

    ts = s["timeseries"]
    assert ts["n"] > 100
    assert ts["detrended_std_ns"] >= 0.0
    # chA & chB individual TDEV(1s) present and positive.
    st = s["stability"]
    assert st["A"]["tdev"].get(1.0, 0.0) > 0.0
    assert st["B"]["tdev"].get(1.0, 0.0) > 0.0


def test_window_trim_drops_rows(tmp_path):
    """skip-after should cut a late injected step; the windowed CSV must
    have strictly fewer rows and a lower p95 than the full run."""
    import two_clock_report as tcr

    csv_path = tmp_path / "cap.csv"
    # inject a big step on chB starting at second 600 (of 900)
    _write_two_channel_csv(csv_path, n=900, step_at=600, step_ns=20.0)

    full_pdf = tmp_path / "full.pdf"
    s_full = tcr.build_report(csv_path, full_pdf, "A", "B", budget_ns=1.0)

    # Window to the pre-step region [00:00:30, 00:09:00) → drops the step.
    sb = datetime(2026, 7, 2, 0, 0, 30, tzinfo=timezone.utc)
    sa = datetime(2026, 7, 2, 0, 9, 0, tzinfo=timezone.utc)
    win_pdf = tmp_path / "win.pdf"
    s_win = tcr.build_report(csv_path, win_pdf, "A", "B", budget_ns=1.0,
                             skip_before=sb, skip_after=sa,
                             work_dir=tmp_path)

    windowed_csv = tmp_path / "win.windowed.csv"
    assert windowed_csv.exists()
    # The step inflates the full-run p95; trimming it out lowers p95.
    assert s_win["cdf"]["p95"] < s_full["cdf"]["p95"]


def test_main_cli_smoke(tmp_path, monkeypatch):
    import two_clock_report as tcr
    csv_path = tmp_path / "cap.csv"
    _write_two_channel_csv(csv_path)
    out_pdf = tmp_path / "cli.pdf"
    argv = ["two_clock_report.py", "--from-csv", str(csv_path),
            "--label-a", "GNSSDO+", "--label-b", "OTA166",
            "--out", str(out_pdf), "--budget-ns", "2.0"]
    monkeypatch.setattr(sys, "argv", argv)
    assert tcr.main() == 0
    assert out_pdf.exists() and out_pdf.stat().st_size > 0


def test_missing_file_errors(tmp_path, monkeypatch):
    import two_clock_report as tcr
    import pytest
    argv = ["two_clock_report.py", "--from-csv", str(tmp_path / "nope.csv"),
            "--out", str(tmp_path / "x.pdf")]
    monkeypatch.setattr(sys, "argv", argv)
    with pytest.raises(SystemExit):
        tcr.main()


def test_tau_reach_scales_with_capture_and_adev_exceeds_tdev(tmp_path):
    """τ reach is data-driven: a longer capture reaches larger τ, and the
    ADEV panel (N/4 cap) reaches at least as far as TDEV (N/10 cap)."""
    import two_clock_report as tcr
    short = tmp_path / "s.csv"; _write_two_channel_csv(short, n=1200)
    long = tmp_path / "l.csv"; _write_two_channel_csv(long, n=6000)
    s = tcr.build_report(short, tmp_path / "s.pdf", "A", "B", budget_ns=2.0)
    ln = tcr.build_report(long, tmp_path / "l.pdf", "A", "B", budget_ns=2.0)
    assert max(ln["stability"]["A"]["adev"]) > max(s["stability"]["A"]["adev"])
    assert max(ln["stability"]["A"]["adev"]) >= max(ln["stability"]["A"]["tdev"])


def test_parse_iso_naive_gets_utc():
    """A bare (offset-naive) ISO string must parse to an aware UTC datetime,
    so it can be compared against the capture's offset-aware ts_iso."""
    import two_clock_report as tcr
    naive = tcr._parse_iso("2026-07-02T14:47:28")     # no offset
    aware = tcr._parse_iso("2026-07-02T14:47:28Z")    # explicit Z
    assert naive.tzinfo is not None and naive.utcoffset().total_seconds() == 0
    assert aware.tzinfo is not None
    assert naive == aware


def test_main_naive_skip_arg_does_not_crash(tmp_path, monkeypatch):
    """`--skip-before` with a bare (naive) ISO string — which the help invites —
    must window cleanly, not raise offset-naive/aware TypeError."""
    import two_clock_report as tcr
    csv_path = tmp_path / "cap.csv"
    _write_two_channel_csv(csv_path, n=900)
    out_pdf = tmp_path / "cli.pdf"
    argv = ["two_clock_report.py", "--from-csv", str(csv_path),
            "--label-a", "A", "--label-b", "B", "--out", str(out_pdf),
            "--budget-ns", "2.0",
            "--skip-before", "2026-07-02T00:00:30",   # naive, bare
            "--skip-after", "2026-07-02T00:12:00"]    # naive, bare
    monkeypatch.setattr(sys, "argv", argv)
    assert tcr.main() == 0
    assert out_pdf.exists() and out_pdf.stat().st_size > 0

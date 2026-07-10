"""Test the N-clock comparison report (clock_report.py / clock_report_core).

Synthetic shared-Rb TICC captures with KNOWN injected white-phase noise let
us assert the analysis math cold:

  * per-clock TDEV(1s) ≈ injected σ_x   (for white PM, TDEV(1s) = σ_x),
  * pairwise p95 |Δ| ≈ 1.645·√(σ_i²+σ_j²) after detrend,
  * budget grading is group-aware: a clean same-group pair PASSes the 1 ns
    bound, a deliberately-noisy same-group pair FAILs it, and a cross-group
    pair is graded against the looser 2 ns bound.

Both TICC CSV schemas are exercised so ingest is locked:
  1. engine --ticc-log  : header host_timestamp,host_monotonic,ref_sec,ref_ps,
                          channel  (NO comment line)
  2. ticc_capture.py    : FIRST line is a '# capture_tool=...' comment, then
                          header ts_iso,channel,ref_sec,ref_ps,recv_mono
No hardware needed.
"""
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_ROOT / "scripts"))
sys.path.insert(0, str(_ROOT / "tools"))

_PS_PER_S = 10 ** 12
_T0 = datetime(2026, 7, 9, 3, 0, 0, tzinfo=timezone.utc)


def _phase_ps(n, sigma_ns, drift_ps_s, offset_ps, seed):
    """One channel's ref phase (ps since a per-channel boot), = perfect 1 Hz
    + constant offset + linear drift + white phase jitter of σ = sigma_ns."""
    rng = np.random.default_rng(seed)
    i = np.arange(n)
    jitter = rng.normal(0, sigma_ns * 1000.0, n)          # ns σ → ps
    return (offset_ps + i * _PS_PER_S + drift_ps_s * i + jitter).astype(np.int64)


def _write_csv(path, channels, boot_sec=100_000, fmt="capture", t0=_T0):
    """Write a shared-Rb TICC capture.

    ``channels`` = {channel_name: phase_ps_array}.  All channels share the
    same UTC timeline (one row per channel per second) starting at ``t0``.
    ``fmt`` selects the on-disk schema: 'capture' (ticc_capture.py,
    '#'-comment + ts_iso) or 'engine' (--ticc-log, host_timestamp, no comment).
    """
    n = len(next(iter(channels.values())))
    lines = []
    if fmt == "capture":
        lines.append("# capture_tool=ticc_capture.py capture_version=1 "
                      "git=deadbee-dirty utc=2026-07-09T03:00:00+00:00\n")
        lines.append("ts_iso,channel,ref_sec,ref_ps,recv_mono\n")
    else:
        lines.append("host_timestamp,host_monotonic,ref_sec,ref_ps,channel\n")
    for i in range(n):
        ts = (t0 + timedelta(seconds=i)).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
        mono = float(i) + 0.95
        for ch, ph in channels.items():
            total = int(ph[i]) + boot_sec * _PS_PER_S
            sec, ps = divmod(total, _PS_PER_S)
            if fmt == "capture":
                lines.append(f"{ts},{ch},{sec},{ps},{mono:.6f}\n")
            else:
                lines.append(f"{ts},{mono:.6f},{sec},{ps},{ch}\n")
    Path(path).write_text("".join(lines))


def _fleet(tmp_path, n=4000):
    """Four clocks across two CSV files + two schemas.

    G1 (shared antenna, 1 ns bound):
      - clean1  σ=0.10 ns   (engine fmt csv, chA)
      - clean2  σ=0.10 ns   (engine fmt csv, chB)   → clean1↔clean2 PASSes 1 ns
      - noisy   σ=1.00 ns   (capture fmt csv, chA)  → noisy↔clean1 FAILs 1 ns
    G2 (separate antenna, 2 ns bound):
      - far     σ=0.10 ns   (capture fmt csv, chB)  → cross-group, 2 ns bound
    """
    csv_engine = tmp_path / "internal-ticc.csv"      # PiFace/PiPuss-style
    csv_capture = tmp_path / "madhat-ticc.csv"       # ticc_capture.py-style
    _write_csv(csv_engine, {
        "chA": _phase_ps(n, 0.10, drift_ps_s=3.0, offset_ps=200_000_000, seed=1),
        "chB": _phase_ps(n, 0.10, drift_ps_s=-2.0, offset_ps=-150_000_000, seed=2),
    }, fmt="engine")
    _write_csv(csv_capture, {
        "chA": _phase_ps(n, 1.00, drift_ps_s=5.0, offset_ps=400_000_000, seed=3),
        "chB": _phase_ps(n, 0.10, drift_ps_s=1.0, offset_ps=-50_000_000, seed=4),
    }, fmt="capture")

    import clock_report_core as core
    return [
        core.HostSpec("clean1", csv_engine, "chA", "G1"),
        core.HostSpec("clean2", csv_engine, "chB", "G1"),
        core.HostSpec("noisy", csv_capture, "chA", "G1"),
        core.HostSpec("far", csv_capture, "chB", "G2"),
    ]


def _pair(result, a, b):
    for pr in result.pairs:
        if {pr.a, pr.b} == {a, b}:
            return pr
    raise KeyError((a, b))


def test_both_csv_schemas_ingest(tmp_path):
    """The '#'-comment capture format AND the comment-less engine format must
    both yield samples — a naive DictReader would eat the comment as a header
    and silently return 0 rows."""
    import clock_report_core as core
    specs = _fleet(tmp_path, n=1000)
    result = core.analyze(specs, work_dir=tmp_path, do_hat=False)
    for lab in ("clean1", "clean2", "noisy", "far"):
        assert result.hosts[lab].n > 500, f"{lab} ingested too few rows"


def test_per_clock_tdev_matches_injected_noise(tmp_path):
    """For white PM, TDEV(1s) = σ_x.  clean clocks (σ=0.1 ns) and the noisy
    clock (σ=1.0 ns) should land near their injected σ."""
    import clock_report_core as core
    specs = _fleet(tmp_path)
    result = core.analyze(specs, work_dir=tmp_path, do_hat=False)
    td_clean = result.hosts["clean1"].tdev[1.0]
    td_noisy = result.hosts["noisy"].tdev[1.0]
    assert 0.06 < td_clean < 0.16, td_clean          # ≈ 0.10 ns
    assert 0.7 < td_noisy < 1.4, td_noisy            # ≈ 1.00 ns
    assert td_noisy > 3 * td_clean                   # clearly separated


def test_pair_p95_and_group_aware_grading(tmp_path):
    import clock_report_core as core
    specs = _fleet(tmp_path)
    result = core.analyze(specs, work_dir=tmp_path, do_hat=False)

    assert len(result.pairs) == 6                    # C(4,2)

    # clean1↔clean2: same group, both σ=0.1 → Δσ≈0.14, p95≈0.23 ns < 1 ns.
    cc = _pair(result, "clean1", "clean2")
    assert cc.shared and cc.bound_ns == 1.0
    assert cc.p95 < 1.0 and cc.grade == "PASS"
    # sanity on the p95 magnitude (1.645·√(0.1²+0.1²) ≈ 0.23 ns).
    assert 0.1 < cc.p95 < 0.5

    # noisy↔clean1: same group, noisy σ=1.0 → p95≈1.65 ns > 1 ns → FAIL.
    nc = _pair(result, "noisy", "clean1")
    assert nc.shared and nc.bound_ns == 1.0
    assert nc.p95 > 1.0 and nc.grade == "FAIL"

    # noisy↔far: CROSS group → 2 ns bound; p95≈1.65 ns < 2 ns → PASS on the
    # looser separate-antenna budget even though the pair is noisy.
    nf = _pair(result, "noisy", "far")
    assert (not nf.shared) and nf.bound_ns == 2.0
    assert nf.grade == "PASS"

    # far↔clean1: cross group, both quiet → PASS 2 ns.
    fc = _pair(result, "far", "clean1")
    assert (not fc.shared) and fc.grade == "PASS"


def test_p95_matrix_symmetric(tmp_path):
    import clock_report_core as core
    specs = _fleet(tmp_path)
    result = core.analyze(specs, work_dir=tmp_path, do_hat=False)
    p95, bound, labels = core.p95_matrix(result)
    n = len(labels)
    for i in range(n):
        assert np.isnan(p95[i, i])
        for j in range(n):
            if i != j:
                assert np.isclose(p95[i, j], p95[j, i])
                assert np.isclose(bound[i, j], bound[j, i])


def test_n_corner_hat_recovers_individual_noise(tmp_path):
    """The N-hat removes the shared Rb; each clock's individual TDEV(1s)
    should still track its injected σ_x (± TDC), and the noisy clock stays
    well above the clean ones."""
    import clock_report_core as core
    specs = _fleet(tmp_path)
    result = core.analyze(specs, work_dir=tmp_path, do_hat=True)
    hat = result.hat
    assert set(hat) == {"clean1", "clean2", "noisy", "far"}

    def tdev1(lab):
        h = hat[lab]
        i = int(np.argmin(np.abs(h["taus_tdev"] - 1.0)))
        assert abs(h["taus_tdev"][i] - 1.0) < 1e-9
        return float(h["tdev_ns"][i])

    assert tdev1("noisy") > 3 * tdev1("clean1")
    assert 0.6 < tdev1("noisy") < 1.5


def test_build_report_pdf_four_pages(tmp_path):
    import clock_report
    specs = _fleet(tmp_path, n=1500)
    out_pdf = tmp_path / "report.pdf"
    result = clock_report.build_report(specs, out_pdf, work_dir=tmp_path)
    assert out_pdf.exists() and out_pdf.stat().st_size > 0
    from pypdf import PdfReader
    r = PdfReader(str(out_pdf))
    assert len(r.pages) == 4
    for pg in r.pages:
        w, h = float(pg.mediabox.width), float(pg.mediabox.height)
        assert abs((w / h) - (16 / 9)) < 0.01
    npass = sum(1 for p in result.pairs if p.grade == "PASS")
    nfail = sum(1 for p in result.pairs if p.grade == "FAIL")
    assert npass + nfail == len(result.pairs)


def _hist_spec(tmp_path, n=4000, stability_only=False):
    """A 7th 'historical' clock whose capture is from a DIFFERENT day → ZERO
    time overlap with the live fleet."""
    import clock_report_core as core
    csv = tmp_path / "historical.csv"
    t0_old = _T0 - timedelta(days=2)
    _write_csv(csv, {
        "chA": _phase_ps(n, 0.30, drift_ps_s=4.0, offset_ps=1e8, seed=42),
    }, fmt="capture", t0=t0_old)
    return core.HostSpec("OldGNSSDO", csv, "chA", "ufoLondon1",
                         stability_only=stability_only)


def test_disjoint_clock_appears_in_tdev_but_na_in_matrix(tmp_path):
    """A non-contemporaneous clock (auto-detected zero overlap, no flag) must:
    appear in per-clock TDEV, be N/A in every excursion pair (not 0, not a
    crash), and be omitted from the N-hat (no contemporaneous partner)."""
    import clock_report_core as core
    specs = _fleet(tmp_path) + [_hist_spec(tmp_path, stability_only=False)]
    result = core.analyze(specs, work_dir=tmp_path, do_hat=True)

    # 1) shows up in per-clock TDEV from its OWN trace (σ=0.30 → TDEV(1s)≈0.3).
    assert result.hosts["OldGNSSDO"].n > 3000
    assert 0.2 < result.hosts["OldGNSSDO"].tdev[1.0] < 0.45

    # 2) every pair touching it is N/A (auto-detected zero overlap).
    hist_pairs = [p for p in result.pairs
                  if "OldGNSSDO" in (p.a, p.b)]
    assert len(hist_pairs) == 4                      # vs each live clock
    for p in hist_pairs:
        assert p.grade in core.NA_GRADES
        assert p.p95 != p.p95                        # NaN, not 0

    # 3) the live pairs are unaffected — still graded.
    live = [p for p in result.pairs if "OldGNSSDO" not in (p.a, p.b)]
    assert len(live) == 6
    assert all(p.grade in ("PASS", "FAIL") for p in live)

    # 4) N-hat omits it (can't be placed without a contemporaneous partner).
    assert "OldGNSSDO" not in result.hat
    assert {"clean1", "clean2", "noisy", "far"} <= set(result.hat)

    # 5) matrix cells for it are NaN (N/A), and the matrix stays symmetric.
    p95, bound, labels = core.p95_matrix(result)
    hi = labels.index("OldGNSSDO")
    for j in range(len(labels)):
        if j != hi:
            assert np.isnan(p95[hi, j]) and np.isnan(p95[j, hi])


def test_stability_only_flag_forces_na_even_with_overlap(tmp_path):
    """The explicit stability_only flag N/As all excursion pairs even if the
    clock DOES overlap in time (operator declares it comparison-only)."""
    import clock_report_core as core
    specs = _fleet(tmp_path)
    # 'far' overlaps everyone; declaring it stability_only must N/A its pairs.
    for s in specs:
        if s.label == "far":
            s.stability_only = True
    result = core.analyze(specs, work_dir=tmp_path, do_hat=True)
    far_pairs = [p for p in result.pairs if "far" in (p.a, p.b)]
    assert far_pairs and all(p.grade == "NA" for p in far_pairs)
    assert "far" not in result.hat                  # excluded from the hat
    assert result.hosts["far"].tdev.get(1.0, 0.0) > 0.0   # still in TDEV


def test_skip_before_windows_excursion_not_per_clock_tdev(tmp_path):
    """--skip-before trims the pairwise excursion window but per-clock TDEV
    keeps using the full trace (a longer trace → equal-or-larger τ reach)."""
    import clock_report_core as core
    from datetime import timezone
    specs = _fleet(tmp_path, n=4000)
    full = core.analyze(specs, work_dir=tmp_path / "a", do_hat=False)
    # Window the excursion to the last ~1000 s of the 4000 s run.
    sb = _T0 + timedelta(seconds=3000)
    win = core.analyze(specs, work_dir=tmp_path / "b", skip_before=sb,
                       do_hat=False)
    # Per-clock TDEV n (full trace) is unchanged by the excursion window.
    assert win.hosts["clean1"].n == full.hosts["clean1"].n
    # ...but the pairwise sample count drops with the window.
    cc_full = _pair(full, "clean1", "clean2").n
    cc_win = _pair(win, "clean1", "clean2").n
    assert cc_win < cc_full and cc_win > 500


def test_from_csv_two_clock_convenience(tmp_path):
    """--from-csv path: one csv's chA/chB become two same-group clocks."""
    import clock_report_core as core
    import clock_report
    csv = tmp_path / "cap.csv"
    _write_csv(csv, {
        "chA": _phase_ps(1500, 0.10, 3.0, 200_000_000, seed=5),
        "chB": _phase_ps(1500, 0.10, -2.0, -100_000_000, seed=6),
    }, fmt="capture")
    specs = [core.HostSpec("A", csv, "chA", "same"),
             core.HostSpec("B", csv, "chB", "same")]
    out_pdf = tmp_path / "two.pdf"
    result = clock_report.build_report(specs, out_pdf)
    assert len(result.pairs) == 1
    assert result.pairs[0].shared and result.pairs[0].grade == "PASS"


def test_gross_outliers_dropped_and_flagged(tmp_path):
    """ptBoat-style: a few ~500 ms PPS glitches must be dropped from the
    per-clock TDEV path (else variance-based TDEV blows up) and reported,
    while a clean co-channel on the same TICC is untouched."""
    import clock_report_core as core
    a = _phase_ps(4000, 0.10, 0.0, 200_000_000, seed=11)
    for i in (500, 1500, 2500, 3200, 3800):
        a[i] += 500_000_000_000                       # +500 ms glitch (ps)
    b = _phase_ps(4000, 0.10, 0.0, -100_000_000, seed=12)
    csv = tmp_path / "glitch.csv"
    _write_csv(csv, {"chA": a, "chB": b}, fmt="capture")
    specs = [core.HostSpec("glitchy", csv, "chA", "G"),
             core.HostSpec("clean", csv, "chB", "G")]
    result = core.analyze(specs, work_dir=tmp_path, do_hat=False)
    hg = result.hosts["glitchy"]
    assert hg.n_outliers >= 5                          # all glitches caught
    assert hg.tdev and max(hg.tdev.values()) < 50.0    # TDEV sane, not ms-scale
    assert result.hosts["clean"].n_outliers == 0       # co-channel untouched

    # quiet clock keeps its own real ns-scale noise (nothing dropped)
    y = np.random.default_rng(3).normal(0.0, 0.05, 5000)
    _, ny = core.reject_gross_outliers(y)
    assert ny == 0


def test_stability_skip_before_spares_stability_only(tmp_path):
    """--stability-skip-before windows LIVE clocks to the undisturbed run,
    but must NOT touch a non-contemporaneous stability-only clock — else it
    trims the historical trace to n=0 (2026-07-10 windowing bug)."""
    import clock_report_core as core
    a = _phase_ps(2000, 0.10, 0.0, 200_000_000, seed=21)   # live clock
    b = _phase_ps(2000, 0.10, 0.0, -100_000_000, seed=22)  # historical trace
    csv = tmp_path / "hist.csv"
    _write_csv(csv, {"chA": a, "chB": b}, fmt="capture")
    specs = [core.HostSpec("live", csv, "chA", "g"),
             core.HostSpec("hist", csv, "chB", "g", stability_only=True)]
    ssb = core.parse_iso("2030-01-01T00:00:00Z")           # after all data
    result = core.analyze(specs, work_dir=tmp_path, do_hat=False,
                          stability_skip_before=ssb)
    assert result.hosts["hist"].n > 0     # stability-only NOT trimmed (the fix)
    assert result.hosts["hist"].tdev
    assert result.hosts["live"].n == 0    # a live clock IS trimmed by the skip

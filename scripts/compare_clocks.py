#!/usr/bin/env python3
"""compare_clocks.py — turnkey two-channel clock head-to-head.

One command for a head-to-head on a shared-reference single TICC:

    chA = clock A,  chB = clock B   (the ref-immune differential method —
    the TICC's own reference noise cancels in chA−chB).

This is the campaign driver for the multi-clock PPS-comparison work (our
clocks vs each other + external clocks otcBob1 / M600 / GNSSDO+), graded
on the moonshot excursion bounds (docs/two-site-sync-budget.md):

    shared antenna:   |Δ| ≤ 1 ns @ 95 %
    separate antennas: |Δ| ≤ 2 ns @ 95 %

It does NOT reimplement any analysis math.  It IMPORTS and REUSES:

  * tools/plot_xhost_agreement_cdf.render_cdf  → the excursion CDF
    (p95/p99/max |Δ| vs the budget) — the pass/fail metric.
  * tools/plot_clock_stability_stack.render_pair_stability → the
    TDEV/ADEV overlay — the "why / at which τ" diagnostic.

Both PNGs and the result .txt carry the analysis-provenance stamp
(tools/analysis_provenance), so every figure is self-identifying and
never mistaken across analysis versions.

Two modes:

  Live capture (needs hardware):
      compare_clocks.py --ticc /dev/ticc4 \\
          --label-a ours --label-b M600 \\
          --duration 3600 --out-dir data/m600-headtohead

  Analyze an existing two-channel CSV (no hardware):
      compare_clocks.py --from-csv data/m600-headtohead/capture.csv \\
          --label-a ours --label-b M600 \\
          --out-dir data/m600-headtohead

The grade is on the CDF p95 excursion (the budget metric); TDEV/ADEV
say at which τ one clock beats the other.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPT_DIR))
sys.path.insert(0, str(_SCRIPT_DIR.parent / "tools"))

from analysis_provenance import provenance_line  # noqa: E402
from plot_xhost_agreement_cdf import load_pairs, render_cdf  # noqa: E402
from plot_clock_stability_stack import render_pair_stability  # noqa: E402


_TOOLNAME = "compare_clocks.py"


def _capture(ticc: str, duration_s: float, out_csv: Path) -> None:
    """Run ticc_capture.py for ``duration_s`` seconds, writing chA+chB to
    a single CSV at ``out_csv``.

    Live-capture path only — requires real TICC hardware.  Cleanly
    separated from the analysis path: nothing here runs unless the user
    omitted ``--from-csv``.  ticc_capture.py writes to
    ``{out_dir}/{prefix}-{YYYY-MM-DD}.csv``; we point it at our out_dir
    with a deterministic prefix and then resolve the file it produced.
    """
    out_dir = out_csv.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    prefix = out_csv.stem
    capture_py = _SCRIPT_DIR / "ticc_capture.py"
    proc = subprocess.Popen(
        [sys.executable, str(capture_py),
         "--device", ticc, "--out-dir", str(out_dir), "--prefix", prefix],
        stdout=sys.stderr, stderr=sys.stderr,
    )
    try:
        time.sleep(duration_s)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.kill()
    # ticc_capture rotates by UTC day; for a single bounded run grab the
    # produced file(s) under our prefix.  Most runs land one file; if a
    # run straddles UTC midnight, the first is the bulk — we analyze it.
    produced = sorted(out_dir.glob(f"{prefix}-*.csv"))
    if not produced:
        raise RuntimeError(
            f"ticc_capture produced no CSV under {out_dir}/{prefix}-*.csv")
    # Symlink/copy-free: just point the caller at the produced file.
    if produced[0] != out_csv:
        out_csv.write_text(produced[0].read_text())


def grade(p95_ps: float, budget_ns: float) -> str:
    """PASS/FAIL on the p95 excursion vs the budget (the moonshot metric)."""
    return "PASS" if p95_ps <= budget_ns * 1000.0 else "FAIL"


def _stability_verdict(stab: dict, label_a: str, label_b: str) -> str:
    """At which τ does A beat B (lower TDEV = more stable)?  Returns a
    short human sentence for the verdict line."""
    tdev_a = stab.get(label_a, {}).get("tdev", {})
    tdev_b = stab.get(label_b, {}).get("tdev", {})
    common = sorted(set(tdev_a) & set(tdev_b))
    if not common:
        return "stability comparison unavailable (insufficient overlap)"
    a_wins = [t for t in common if tdev_a[t] < tdev_b[t]]
    b_wins = [t for t in common if tdev_b[t] < tdev_a[t]]
    if a_wins and not b_wins:
        return f"{label_a} more stable than {label_b} at all measured τ"
    if b_wins and not a_wins:
        return f"{label_b} more stable than {label_a} at all measured τ"
    if a_wins and b_wins:
        a_max = max(a_wins)
        return (f"{label_a} more stable for τ≤{a_max:g}s; "
                f"{label_b} more stable at longer τ")
    return "clocks comparable in stability across measured τ"


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ticc", default=None,
                    help="TICC serial device for live capture (e.g. "
                         "/dev/ticc4).  Ignored when --from-csv is given.")
    ap.add_argument("--from-csv", default=None, type=Path,
                    help="Analyze an existing two-channel CSV (chA+chB) "
                         "instead of capturing.  No hardware needed.")
    ap.add_argument("--label-a", default="clock A",
                    help="Label for chA (e.g. 'ours').")
    ap.add_argument("--label-b", default="clock B",
                    help="Label for chB (e.g. 'M600').")
    ap.add_argument("--duration", type=float, default=3600.0,
                    help="Live-capture duration in seconds (default 3600). "
                         "Ignored when --from-csv is given.")
    ap.add_argument("--out-dir", required=True, type=Path,
                    help="Directory for the capture CSV, PNGs, and result "
                         "txt.")
    ap.add_argument("--budget-ns", type=float, default=1.0,
                    help="p95 excursion budget in ns for PASS/FAIL "
                         "(default 1.0 = shared-antenna; use 2.0 for "
                         "separate antennas).")
    ap.add_argument("--skip-before", default=None,
                    help="ISO-8601 UTC; drop earlier rows (bootstrap trim) "
                         "from the stability overlay.")
    args = ap.parse_args()

    print(provenance_line(_TOOLNAME), file=sys.stderr)

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    # Resolve the CSV to analyze.
    if args.from_csv is not None:
        csv_path = args.from_csv
        if not csv_path.exists():
            ap.error(f"--from-csv path does not exist: {csv_path}")
    else:
        if not args.ticc:
            ap.error("either --from-csv or --ticc is required")
        csv_path = out_dir / "capture.csv"
        print(f"compare_clocks: capturing {args.duration:.0f}s on "
              f"{args.ticc} → {csv_path}", file=sys.stderr)
        _capture(args.ticc, args.duration, csv_path)

    skip_dt = None
    if args.skip_before:
        skip_dt = datetime.fromisoformat(args.skip_before.replace("Z", "+00:00"))

    # --- Excursion CDF (the pass/fail metric) ---------------------------
    cdf_png = out_dir / "agreement_cdf.png"
    pair_label = f"{args.label_a} (chA) vs {args.label_b} (chB)"
    cdf_stats = render_cdf(
        [(pair_label, lambda: load_pairs(csv_path, skip_before=skip_dt))],
        cdf_png,
        title=f"{args.label_a} vs {args.label_b} — phase-agreement CDF",
    )
    if not cdf_stats:
        print("compare_clocks: no paired samples — cannot grade.",
              file=sys.stderr)
        return 1
    stats = cdf_stats[pair_label]
    p95_ps = stats["p95"]
    p99_ps = stats["p99"]
    max_ps = stats["max"]
    n = stats["n"]
    verdict = grade(p95_ps, args.budget_ns)

    # --- Stability overlay (the "why / at which τ") ---------------------
    stab_png = out_dir / "stability_stack.png"
    stab = render_pair_stability(
        csv_path, args.label_a, args.label_b, stab_png,
        skip_before=skip_dt,
        title=f"{args.label_a} vs {args.label_b} — stability head-to-head")
    stab_sentence = _stability_verdict(stab, args.label_a, args.label_b)

    # --- Verdict -------------------------------------------------------
    prov = provenance_line(_TOOLNAME)
    lines = [
        prov,
        f"head-to-head: {args.label_a} (chA) vs {args.label_b} (chB)",
        f"source: {csv_path}",
        f"paired epochs: {n}",
        f"p95 |Δ| = {p95_ps / 1000.0:.3f} ns "
        f"(p99 = {p99_ps / 1000.0:.3f} ns, max = {max_ps / 1000.0:.3f} ns)",
        f"VERDICT: p95 |Δ| = {p95_ps / 1000.0:.3f} ns vs "
        f"{args.budget_ns:g} ns target → {verdict}; {stab_sentence}",
        f"figures: {cdf_png}  {stab_png}",
    ]
    result_txt = out_dir / "verdict.txt"
    result_txt.write_text("\n".join(lines) + "\n")

    # One-line VERDICT to stdout (the headline).
    print(lines[5])
    print(f"compare_clocks: wrote {result_txt}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())

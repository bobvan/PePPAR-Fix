"""CI guard: no wall-clock (CLOCK_REALTIME) reads in timing-critical code.

The 2026-07-06 otcBob1 stall: the obs<->PPS correlation compared a PPS edge
second stamped in the SYSTEM clock against a GPS-epoch target.  When otcBob1's
(undisciplined) system clock drifted ~800 ppm from GPS, the two frames crossed
the 11 s correlation window and the match broke -> the servo starved of
observations -> the DO drifted a 146 us excursion.  NTP masks it; it does not
fix it, because the code is still *asking the system what time it is*.

RULE (docs/stream-timescale-correlation.md): the ONLY shared timescale for
correlation / intervals / timeouts is CLOCK_MONOTONIC (``time.monotonic()``).
Asking the system for the wall clock (``time.time`` / ``datetime.now`` /
``utcnow`` / ``CLOCK_REALTIME`` / no-arg ``gmtime``/``localtime``/``ctime``) is
forbidden in the timing-critical runtime modules -- UNLESS the value is a
genuine human/record *"when"* (a log line, a state-file ``updated`` stamp, a
date-stamped filename), in which case annotate the line ``# wallclock-ok: why``.

Like a coordinate is meaningless without its CRS, a time value is meaningless
without its clock frame.  Name or comment the frame (wall vs mono) at the
declaration so the next reader can't confuse them.

Standalone audit:  ``python -m peppar_fix.test_wallclock_discipline``
"""
import io
import os
import re
import tokenize

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.dirname(_HERE)

# Runtime modules where a wall-clock read in a timing context is a bug.
# Display-only tools (plot_*, *report*, analyze_*, char scripts, overlay/) are
# excluded -- they legitimately format wall time for humans.
CRITICAL_FILES = [
    "peppar_fix_engine.py",
    "realtime_ppp.py",
    os.path.join("peppar_fix", "correlation_gate.py"),
    os.path.join("peppar_fix", "ptp_device.py"),
    os.path.join("peppar_fix", "extint_reader.py"),
    os.path.join("peppar_fix", "ticc.py"),
    os.path.join("peppar_fix", "msm_obs_source.py"),
    os.path.join("peppar_fix", "sbf_obs_source.py"),
    os.path.join("peppar_fix", "noise_estimator.py"),
    os.path.join("peppar_fix", "do_state.py"),
    os.path.join("peppar_fix", "runtime_state_resolve.py"),
    os.path.join("peppar_fix", "receiver_state.py"),
    os.path.join("peppar_fix", "position_state.py"),
    os.path.join("peppar_fix", "survey_state_watcher.py"),
    os.path.join("peppar_fix", "arp_history.py"),
]

_WALLCLOCK = re.compile(
    r"\btime\.time\s*\("
    r"|\btime\.time_ns\s*\("
    r"|\bdatetime\.now\s*\("
    r"|\.utcnow\s*\("
    r"|\btime\.gmtime\s*\(\s*\)"
    r"|\btime\.localtime\s*\(\s*\)"
    r"|\btime\.ctime\s*\(\s*\)"
    r"|CLOCK_REALTIME"
)
# ``# wallclock-ok: <reason>``   -> a genuine human/record timestamp (permanent).
# ``# wallclock-todo: <reason>`` -> a known bug staged for a monotonic fix (the
#     correlation-core change lands behind servo_sim; the test reports these
#     loudly but does NOT fail, so the mechanical fixes can land first).
_ALLOW = re.compile(r"#\s*wallclock-ok\b")
_TODO = re.compile(r"#\s*wallclock-todo\b")


def _code_lines(path):
    """Return (original_lines, code_lines) where code_lines has every string
    and comment token blanked out -- so a wall-clock name mentioned in a
    docstring or comment (e.g. ptp_device's "CLOCK_REALTIME as a transfer
    standard") is not mistaken for a real read.  We tokenize rather than
    string-split so the match is precise, not heuristic."""
    with open(path, encoding="utf-8") as f:
        src = f.read()
    lines = src.splitlines()
    blanked = list(lines)
    try:
        for tok in tokenize.generate_tokens(io.StringIO(src).readline):
            if tok.type not in (tokenize.STRING, tokenize.COMMENT):
                continue  # f-string embedded code stays visible (it is real code)
            (sr, sc), (er, ec) = tok.start, tok.end
            for r in range(sr, er + 1):
                if r - 1 >= len(blanked):
                    break
                ln = blanked[r - 1]
                a = sc if r == sr else 0
                b = ec if r == er else len(ln)
                blanked[r - 1] = ln[:a] + " " * (b - a) + ln[b:]
    except (tokenize.TokenError, IndentationError):
        pass  # fall back to raw lines on a partial/odd file
    return lines, blanked


def _scan():
    """Return (unmarked, todos): each a list of (relpath, lineno, line)."""
    unmarked, todos = [], []
    for rel in CRITICAL_FILES:
        path = os.path.join(_SCRIPTS, rel)
        if not os.path.exists(path):
            continue
        orig, code = _code_lines(path)
        for i, (raw, stripped_code) in enumerate(zip(orig, code), 1):
            if not _WALLCLOCK.search(stripped_code):
                continue
            # The read is in code; its annotation (if any) is in the comment,
            # which lives on the *original* line.
            if _ALLOW.search(raw):
                continue
            (todos if _TODO.search(raw) else unmarked).append(
                (rel, i, raw.strip()))
    return unmarked, todos


def find_violations():
    """Un-annotated wall-clock reads (neither wallclock-ok nor -todo)."""
    return _scan()[0]


def test_no_unannotated_wallclock_in_timing_paths():
    unmarked, todos = _scan()
    if todos:  # loud, non-fatal: staged correlation-core fixes
        print("\n# wallclock-todo (staged, must land behind servo_sim):")
        for f, n, c in todos:
            print(f"  {f}:{n}:  {c}")
    if unmarked:
        detail = "\n".join(f"  {f}:{n}:  {c}" for f, n, c in unmarked)
        raise AssertionError(
            f"{len(unmarked)} wall-clock (CLOCK_REALTIME) read(s) in "
            f"timing-critical code without a '# wallclock-ok:' (legit record "
            f"timestamp) or '# wallclock-todo:' (staged fix) annotation.\n"
            f"Use time.monotonic() for intervals/correlation.\n" + detail
        )


if __name__ == "__main__":
    unmarked, todos = _scan()
    for f, n, c in unmarked:
        print(f"UNMARKED  {f}:{n}:  {c}")
    for f, n, c in todos:
        print(f"TODO      {f}:{n}:  {c}")
    print(f"\n{len(unmarked)} unmarked, {len(todos)} staged-todo")

"""pos_replay reference-capture bundle — the raw-stream writer/reader core.

Concrete realization of the bundle layout in
docs/pos-replay-capture-manifest.md §6.  A :class:`RawCaptureBundle` records
each Group-A stream's raw payloads, each tagged with its ``CLOCK_MONOTONIC``
``recv_mono``, into a self-describing per-stream ``.cap`` file, so a
deterministic replay can re-feed every message in ``recv_mono`` order (§2).
Capturing raw + ``recv_mono`` — not the already-correlated form — is what lets
the replay re-run the obs↔PPS gate and ``match_pps_mono`` deterministically
(milestone 0, the now_mono/recv_mono refactor).

This module is the format + I/O core: self-contained, unit-testable, no
hardware.  The engine-side ``--raw-capture-dir`` reader tap (which feeds this
from the live readers that already stamp ``recv_mono``) is the follow-up.

``.cap`` record framing (little-endian, streamable, self-describing):

    [recv_mono: float64][length: uint32][payload: length bytes] ...

Binary-safe (UBX/RTCM may contain any byte, including NUL); TICC text lines
are stored as their UTF-8 bytes with a trailing newline.
"""
from __future__ import annotations

import os
import struct
import subprocess
import threading
from typing import Iterator, Tuple

_REC_HDR = struct.Struct("<dI")          # recv_mono (float64), length (uint32)
SCHEMA_VERSION = "1"

# Group-A streams (capture manifest §2): logical name → .cap filename.
STREAMS = {
    "ubx":      "ubx.cap",       # GNSS UBX: RAWX/SFRBX/NAV-*/TIM-TP (qErr here)
    "ssr":      "ssr.cap",       # RTCM SSR (orbit/clock/code+phase bias)
    "ssr_bias": "ssr_bias.cap",  # secondary SSR mount, phase-bias-only (dual-mount)
    "eph":      "eph.cap",       # broadcast eph (RTCM 1019/1042/1046 or SFRBX)
    "ticc":     "ticc.cap",      # TICC chA/chB lines
}


class RawCaptureBundle:
    """Write a reference-capture bundle directory (manifest §6 layout).

    Streams are appended to lazily (a ``.cap`` opens on first record).  Use as
    a context manager, or call :meth:`close` explicitly.

    Thread-safe: the live ``--raw-capture-dir`` tap feeds one bundle from the
    independent reader threads (UBX, TICC, NTRIP/SSR, eph), so ``record()``
    holds a short lock around the file-handle dict + the two writes + the
    count (a tiny critical section — never across ``flush()``/disk I/O, per
    the no-long-GIL-holds rule).
    """

    def __init__(self, bundle_dir: str):
        self.dir = bundle_dir
        self.raw_dir = os.path.join(bundle_dir, "raw")
        os.makedirs(self.raw_dir, exist_ok=True)
        for sub in ("env", "engine", "truth"):
            os.makedirs(os.path.join(bundle_dir, sub), exist_ok=True)
        self._files: dict[str, "object"] = {}
        self._counts: dict[str, int] = {}
        # Group-B engine outputs (manifest §3): text sinks under engine/, keyed
        # by filename.  The live [PPP_STATE]/[ZTD_APRIORI] trajectory lands here
        # so pos_replay can score regenerated-vs-captured, not just convergence.
        self._engine_files: dict[str, "object"] = {}
        self._lock = threading.Lock()
        self._flush_stop: "threading.Event | None" = None
        self._flusher: "threading.Thread | None" = None

    def _fh(self, stream: str):
        fh = self._files.get(stream)
        if fh is None:
            fname = STREAMS.get(stream, f"{stream}.cap")
            fh = open(os.path.join(self.raw_dir, fname), "ab")
            self._files[stream] = fh
            self._counts.setdefault(stream, 0)
        return fh

    def record(self, stream: str, payload: bytes, recv_mono: float) -> None:
        """Append one raw message (``payload``) stamped with ``recv_mono``."""
        hdr = _REC_HDR.pack(float(recv_mono), len(payload))
        with self._lock:
            fh = self._fh(stream)
            fh.write(hdr)
            fh.write(payload)
            self._counts[stream] = self._counts.get(stream, 0) + 1

    def record_line(self, stream: str, line, recv_mono: float) -> None:
        """Append a text line (TICC) — stored as bytes with a trailing \\n."""
        b = line.encode("utf-8", "replace") if isinstance(line, str) else bytes(line)
        if not b.endswith(b"\n"):
            b += b"\n"
        self.record(stream, b, recv_mono)

    def engine_log(self, filename: str, line: str) -> None:
        """Append one engine-output text ``line`` to ``engine/<filename>``.

        Group-B sink (manifest §3): the captured live ``[PPP_STATE]`` /
        ``[ZTD_APRIORI]`` trajectory the position filter emits, so a replay can
        be scored against what the live engine actually produced (the
        regenerated≈captured gate), not just whether it converged.  Same lazy-
        open + short-lock discipline as :meth:`record`; the periodic flusher and
        :meth:`close` cover these handles too."""
        b = line.encode("utf-8", "replace") if isinstance(line, str) else bytes(line)
        if not b.endswith(b"\n"):
            b += b"\n"
        with self._lock:
            fh = self._engine_files.get(filename)
            if fh is None:
                fh = open(os.path.join(self.dir, "engine", filename), "ab")
                self._engine_files[filename] = fh
            fh.write(b)

    @property
    def counts(self) -> dict:
        return dict(self._counts)

    def write_manifest(self, *, host: str, started_iso: str,
                       conventions: dict | None = None,
                       software: dict | None = None, notes: str = "") -> str:
        """Write ``manifest.toml`` (provenance + conventions + versions).

        Hand-formatted (``tomli_w`` isn't a dependency — same as do_schema);
        round-trips through ``tomllib``.  Returns the manifest path.
        """
        soft = dict(software or {})
        # Resolve the git rev from the CODE checkout (this module's location),
        # NOT the bundle dir — bundles are archived to gt/RAIDZ, which isn't a
        # git checkout, so cwd=bundle would silently record "unknown" and
        # defeat the provenance field.  An explicit software={"git_rev": ...}
        # from the engine tap still wins via setdefault.
        soft.setdefault("git_rev", _git_rev())
        sections = {
            "capture": {
                "host": host,
                "started": started_iso,
                "streams": list(self._counts.keys()),
                "record_counts": dict(self._counts),
                "notes": notes,
            },
            "conventions": dict(conventions or {}),
            "software": soft,
        }
        path = os.path.join(self.dir, "manifest.toml")
        with open(path, "w") as f:
            f.write(f'schema_version = "{SCHEMA_VERSION}"\n')
            for name, body in sections.items():
                f.write(f"\n[{name}]\n")
                for k, v in body.items():
                    f.write(f"{k} = {_toml(v)}\n")
        return path

    def flush(self) -> None:
        """Flush all stream buffers to the OS.  Lock-safe: snapshots the file
        handles under the lock, then flushes OUTSIDE it — never holds the lock
        across disk I/O (the no-long-GIL-holds rule)."""
        with self._lock:
            handles = list(self._files.values()) + list(self._engine_files.values())
        for fh in handles:
            try:
                fh.flush()
            except (OSError, ValueError):
                pass

    def start_flusher(self, interval_s: float = 5.0) -> None:
        """Flush periodically from a daemon thread, so a SIGKILL / power loss
        on a long capture loses at most ~``interval_s`` of the unflushed tail
        (``read_stream`` is truncation-safe).  ``interval_s <= 0`` disables;
        idempotent (a second call is a no-op)."""
        if not interval_s or interval_s <= 0 or self._flush_stop is not None:
            return
        self._flush_stop = threading.Event()

        def _loop():
            while not self._flush_stop.wait(interval_s):
                self.flush()

        self._flusher = threading.Thread(target=_loop, daemon=True,
                                         name="raw-capture-flush")
        self._flusher.start()

    def close(self) -> None:
        if self._flush_stop is not None:
            self._flush_stop.set()         # stop the periodic flusher ...
            if self._flusher is not None:
                self._flusher.join(timeout=2.0)   # ... and wait it out, so it
                # can't touch a handle we're about to close (deterministic).
        self.flush()                       # flush() takes the lock itself
        with self._lock:
            for fh in self._files.values():
                fh.close()
            for fh in self._engine_files.values():
                fh.close()
            self._files.clear()
            self._engine_files.clear()

    def __enter__(self) -> "RawCaptureBundle":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def read_stream(cap_path: str) -> Iterator[Tuple[float, bytes]]:
    """Yield ``(recv_mono, payload)`` per record from a ``.cap`` file, in
    order.  A truncated trailing record (e.g. a killed capture) ends the
    iteration cleanly rather than raising.

    For **quiescent/closed** bundles (the replay use case).  A short trailing
    record is treated as a clean end — so a *live tailer* reading a still-being-
    written bundle would stop early the instant it hit a mid-flush boundary
    (header flushed, payload pending) and miss everything after; a tailer must
    retry on a short payload instead of stopping.  Replay reads after close,
    so this is correct there."""
    with open(cap_path, "rb") as f:
        while True:
            hdr = f.read(_REC_HDR.size)
            if len(hdr) < _REC_HDR.size:
                return
            recv_mono, length = _REC_HDR.unpack(hdr)
            payload = f.read(length)
            if len(payload) < length:
                return
            yield recv_mono, payload


def merged_records(bundle_dir: str) -> Iterator[Tuple[float, str, bytes]]:
    """Yield ``(recv_mono, stream, payload)`` across all streams in global
    ``recv_mono`` order — the order a deterministic replay re-feeds them.

    Stable on ties: ties break by the ``STREAMS`` declaration order then by
    arrival, so replay is reproducible.
    """
    import heapq
    raw_dir = os.path.join(bundle_dir, "raw")
    order = {name: i for i, name in enumerate(STREAMS)}
    gens = []
    for name in sorted(os.listdir(raw_dir)) if os.path.isdir(raw_dir) else []:
        if not name.endswith(".cap"):
            continue
        stream = next((s for s, fn in STREAMS.items() if fn == name),
                      name[:-4])
        gens.append((order.get(stream, 999), stream,
                     read_stream(os.path.join(raw_dir, name))))
    # heap of (recv_mono, stream_order, seq, stream, payload)
    heap = []
    seq = 0
    for rank, stream, gen in gens:
        try:
            rm, pl = next(gen)
            heapq.heappush(heap, (rm, rank, seq, stream, pl, gen))
            seq += 1
        except StopIteration:
            pass
    while heap:
        rm, rank, _s, stream, pl, gen = heapq.heappop(heap)
        yield rm, stream, pl
        try:
            nrm, npl = next(gen)
            heapq.heappush(heap, (nrm, rank, seq, stream, npl, gen))
            seq += 1
        except StopIteration:
            pass


# ── helpers ─────────────────────────────────────────────────────────── #

def _toml_str(s: str) -> str:
    """A valid TOML basic string — escape backslash/quote + control chars."""
    out = (s.replace("\\", "\\\\").replace('"', '\\"')
           .replace("\n", "\\n").replace("\r", "\\r").replace("\t", "\\t"))
    return '"' + out + '"'


def _toml(v) -> str:
    """Minimal TOML value formatter for the shapes the manifest uses."""
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        # TOML has no inf/nan basic value — fall back to a quoted string.
        return repr(v) if (v == v and abs(v) != float("inf")) else _toml_str(str(v))
    if isinstance(v, int):
        return repr(v)
    if isinstance(v, str):
        return _toml_str(v)
    if isinstance(v, dict):
        return "{ " + ", ".join(f"{k} = {_toml(x)}" for k, x in v.items()) + " }"
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(_toml(x) for x in v) + "]"
    return _toml_str(str(v))


def _git_rev(cwd: str | None = None) -> str:
    # Default to the module's checkout (where the code lives), not the caller's
    # cwd / the bundle dir — see write_manifest's note.
    cwd = cwd or os.path.dirname(os.path.abspath(__file__))
    try:
        out = subprocess.run(["git", "rev-parse", "HEAD"], cwd=cwd,
                             capture_output=True, text=True, timeout=5)
        return out.stdout.strip() if out.returncode == 0 else "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"

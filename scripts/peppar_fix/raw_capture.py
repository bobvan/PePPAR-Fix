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
from typing import Iterator, Tuple

_REC_HDR = struct.Struct("<dI")          # recv_mono (float64), length (uint32)
SCHEMA_VERSION = "1"

# Group-A streams (capture manifest §2): logical name → .cap filename.
STREAMS = {
    "ubx":  "ubx.cap",      # GNSS UBX: RAWX/SFRBX/NAV-*/TIM-TP (qErr rides here)
    "ssr":  "ssr.cap",      # RTCM SSR (orbit/clock/code+phase bias)
    "eph":  "eph.cap",      # broadcast ephemeris (RTCM 1019/1042/1046 or SFRBX)
    "ticc": "ticc.cap",     # TICC chA/chB lines
}


class RawCaptureBundle:
    """Write a reference-capture bundle directory (manifest §6 layout).

    Streams are appended to lazily (a ``.cap`` opens on first record).  Use as
    a context manager, or call :meth:`close` explicitly.
    """

    def __init__(self, bundle_dir: str):
        self.dir = bundle_dir
        self.raw_dir = os.path.join(bundle_dir, "raw")
        os.makedirs(self.raw_dir, exist_ok=True)
        for sub in ("env", "engine", "truth"):
            os.makedirs(os.path.join(bundle_dir, sub), exist_ok=True)
        self._files: dict[str, "object"] = {}
        self._counts: dict[str, int] = {}

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
        fh = self._fh(stream)
        fh.write(_REC_HDR.pack(float(recv_mono), len(payload)))
        fh.write(payload)
        self._counts[stream] = self._counts.get(stream, 0) + 1

    def record_line(self, stream: str, line, recv_mono: float) -> None:
        """Append a text line (TICC) — stored as bytes with a trailing \\n."""
        b = line.encode("utf-8", "replace") if isinstance(line, str) else bytes(line)
        if not b.endswith(b"\n"):
            b += b"\n"
        self.record(stream, b, recv_mono)

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
        soft.setdefault("git_rev", _git_rev(self.dir))
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
        for fh in self._files.values():
            fh.flush()

    def close(self) -> None:
        for fh in self._files.values():
            fh.close()
        self._files.clear()

    def __enter__(self) -> "RawCaptureBundle":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def read_stream(cap_path: str) -> Iterator[Tuple[float, bytes]]:
    """Yield ``(recv_mono, payload)`` per record from a ``.cap`` file, in
    order.  A truncated trailing record (e.g. a killed capture) ends the
    iteration cleanly rather than raising."""
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

def _toml(v) -> str:
    """Minimal TOML value formatter for the shapes the manifest uses."""
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return repr(v)
    if isinstance(v, str):
        return '"' + v.replace("\\", "\\\\").replace('"', '\\"') + '"'
    if isinstance(v, dict):
        return "{ " + ", ".join(f"{k} = {_toml(x)}" for k, x in v.items()) + " }"
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(_toml(x) for x in v) + "]"
    return '"' + str(v).replace('"', '\\"') + '"'


def _git_rev(cwd: str) -> str:
    try:
        out = subprocess.run(["git", "rev-parse", "HEAD"], cwd=cwd or ".",
                             capture_output=True, text=True, timeout=5)
        return out.stdout.strip() if out.returncode == 0 else "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"

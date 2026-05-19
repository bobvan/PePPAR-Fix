"""RINEX 3.04 OBS writer for PePPAR Fix engine observations.

Sister module to ``scripts/regression/rinex_reader.py``.  The reader
parses observed RINEX OBS files for the regression harness; this writer
emits a RINEX OBS file from the live engine's observation stream so we
can hand the recorded session to a reference engine (PRIDE PPP-AR,
RTKLIB) for cross-verification.

## Why this exists (2026-04-25)

The 2x2 SSR isolation showed CNES biases land ~1.2 m west of WHU biases
on the same orbit/clock at our site.  Could be a real AC datum
difference or a subtle bias-magnitude-sensitive bug in our application.
To distinguish, we need to feed the SAME observations our engine
processed into a reference PPP engine (e.g. PRIDE PPP-AR) with each
AC's products.  PRIDE consumes RINEX OBS; this writer produces it.

## Usage

```python
from peppar_fix.rinex_writer import RinexWriter

w = RinexWriter(
    "/tmp/run.rnx",
    marker_name="UFO1",
    approx_xyz=(157544.0, -4756190.0, 4232770.0),  # rough is fine
    antenna_type="SFESPK6618H     NONE",
    receiver_model="ZED-F9T",
    receiver_fw="TIM 2.25",
)

# Per epoch in serial_reader's loop, after raw_obs is built:
w.write_epoch(epoch_dt, raw_obs)
# raw_obs format: { 'G16': { 'GPS-L1CA': {'pr': ..., 'cp': ..., 'cno': ..., 'half_cyc': ..., 'lock_ms': ...}, ... }, ... }

w.close()
```

## Format

RINEX 3.04 OBS, ASCII.  Per RINEX spec:
- Header in fixed-column format with line markers in cols 61-80
- Epoch line: ``> YYYY MM DD HH MM SS.sssssss EF NN``
- Per-SV line: ``Gnn`` then 16-char wide observation fields (3-decimal
  pseudorange/phase + 1 char LLI + 1 char SSI), in the order declared
  by SYS / # / OBS TYPES.

We declare the maximal F9T signal set per constellation at header write
time; absent observations write as blanks (16 spaces).

## Engine integration

``make_writer_from_args(args)`` constructs a configured RinexWriter
from an argparse-style Namespace.  Returns None if ``--rinex-out``
isn't set.  Engine's main() calls this once at startup so the
init-side logic (receiver-meta lookup, approx XYZ derivation) lives
in this module — both for testability and to prevent the kind of
NameError/UnboundLocalError bugs that shipped in commit 9597c51 and
were caught by Bravo (commits 1471216 + 8fa49fb).
"""

from __future__ import annotations

import math
from datetime import datetime, timezone
from pathlib import Path

# Per-system observation type list (RINEX 3.x 3-char codes), in the
# order they will appear per epoch.  Covers the maximal F9T fleet
# (F9T-10/L2 and L5 profiles, F9T-20B).  Each (sys, sig_internal)
# maps to one (band, attr) where the RINEX type letter is C, L, S
# (pseudorange / carrier phase / SNR).  Doppler omitted — engine
# doesn't write it today.
#
# Internal sig name → (band+attr, RINEX-type-prefix)
_INTERNAL_TO_BAND_ATTR: dict[str, str] = {
    # GPS
    "GPS-L1CA": "1C",
    "GPS-L2CL": "2L",
    "GPS-L2W":  "2W",
    "GPS-L5Q":  "5Q",
    "GPS-L5I":  "5I",
    # Galileo
    "GAL-E1C":  "1C",
    "GAL-E1B":  "1B",
    "GAL-E5aQ": "5Q",
    "GAL-E5aI": "5I",
    "GAL-E5bQ": "7Q",
    "GAL-E5bI": "7I",
    # BeiDou
    "BDS-B1I":  "2I",
    "BDS-B2I":  "7I",
    "BDS-B2aI": "5I",
    "BDS-B2aQ": "5Q",
    "BDS-B3I":  "6I",
}

# Per-system declared observation types (header order).  Choose a
# superset that covers any F9T tracking profile we might run.  Order
# matters: per-epoch fields are emitted in this order.
_SYS_OBS_TYPES: dict[str, list[str]] = {
    "G": ["C1C", "L1C", "S1C",
          "C2L", "L2L", "S2L",
          "C2W", "L2W", "S2W",
          "C5Q", "L5Q", "S5Q"],
    "E": ["C1C", "L1C", "S1C",
          "C5Q", "L5Q", "S5Q",
          "C7Q", "L7Q", "S7Q"],
    "C": ["C2I", "L2I", "S2I",
          "C7I", "L7I", "S7I",
          "C5I", "L5I", "S5I",
          "C5Q", "L5Q", "S5Q",
          "C6I", "L6I", "S6I"],
}


def _sys_prefix(sv: str) -> str:
    """SV id 'G16' → 'G'; supports G, E, C, R, J, S, I."""
    return sv[0]


def _internal_to_obs_codes(sig_internal: str) -> tuple[str, str, str]:
    """Internal sig name like 'GPS-L1CA' → ('C1C', 'L1C', 'S1C')."""
    band_attr = _INTERNAL_TO_BAND_ATTR.get(sig_internal)
    if band_attr is None:
        raise ValueError(f"unknown internal signal name: {sig_internal!r}")
    return f"C{band_attr}", f"L{band_attr}", f"S{band_attr}"


def _fmt_obs(value: float | None, lli: int = 0, ssi: int = 0) -> str:
    """Format one observation field — 14.3 + LLI(1) + SSI(1) = 16 chars.

    Blank if value is None or NaN.  Per RINEX 3.x §5.5: missing values
    are 16 spaces.  LLI: 0=no slip, 1=lost-lock-since-last; bit 1 (=2)
    half-cycle ambiguity present.  SSI: 1-9 mapped from C/N0; 0=undef.
    """
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return " " * 16
    return f"{value:14.3f}{lli:1d}{ssi:1d}"


def _format_approx_pos_line(x: float, y: float, z: float) -> str:
    """Format the APPROX POSITION XYZ line.  Fixed length so the line
    can be rewritten in place after the header has been emitted.
    """
    return f"{x:14.4f}{y:14.4f}{z:14.4f}{'':<18}APPROX POSITION XYZ\n"


def _find_approx_pos_offset(p: Path) -> int | None:
    """Scan an existing RINEX file's header for the APPROX POSITION
    XYZ line and return its byte offset (start of line).  Returns
    None if not found before END OF HEADER or on I/O error.

    Used when reopening a partially-written daily RINEX after a
    wrapper-respawn so set_approx_xyz() can still rewrite the seed
    in place.
    """
    try:
        with open(p, "rb") as rf:
            offset = 0
            for line in rf:
                if b"APPROX POSITION XYZ" in line:
                    return offset
                if b"END OF HEADER" in line:
                    return None
                offset += len(line)
    except OSError:
        return None
    return None


def _cno_to_ssi(cno: float | None) -> int:
    """Map C/N0 (dB-Hz) to RINEX SSI 1-9.  See RINEX 3.x §5.5.

    1 ≈ < 12 dB-Hz (≈ minimum possible)
    9 ≈ ≥ 54 dB-Hz (≈ maximum)
    Linear in between, clamped.
    """
    if cno is None or (isinstance(cno, float) and math.isnan(cno)):
        return 0
    ssi = int((cno - 12.0) / 6.0) + 1
    return max(1, min(9, ssi))


class RinexWriter:
    """Stream-write a RINEX 3.04 OBS file as the engine processes epochs.

    Lazy header: emitted on the first ``write_epoch`` call so we can
    populate TIME OF FIRST OBS from the actual data.
    """

    def __init__(self, path: str | Path, marker_name: str,
                 approx_xyz: tuple[float, float, float],
                 antenna_type: str,
                 receiver_model: str = "ZED-F9T",
                 receiver_fw: str = "",
                 receiver_serial: str = "",
                 antenna_serial: str = "",
                 observer: str = "PePPAR Fix",
                 agency: str = "",
                 interval_s: float = 1.0,
                 decimate_s: float | None = None,
                 host: str = ""):
        # path is a template — supports {date} (UTC YYYYDDD), {host}.
        # Plain paths (no braces) keep current behavior: one file per run,
        # no rotation.  Daily rotation triggers when {date} appears.
        self._path_tmpl = str(path)
        self._host = host
        self._marker = marker_name
        self._approx_xyz = approx_xyz
        self._antenna_type = antenna_type
        self._rx_model = receiver_model
        self._rx_fw = receiver_fw
        self._rx_serial = receiver_serial
        self._ant_serial = antenna_serial
        self._observer = observer
        self._agency = agency
        # If decimate_s is set, header advertises the decimated interval
        # since that's what consumers actually see in the file.
        self._decimate_s = float(decimate_s) if decimate_s else None
        self._interval = (self._decimate_s if self._decimate_s
                          else interval_s)
        self._fp = None
        self._path: Path | None = None
        self._current_date: str | None = None  # YYYYDDD of current file
        self._header_written = False
        self._approx_pos_offset: int | None = None  # byte offset of
        #   APPROX POSITION XYZ line in current file; enables in-place
        #   rewrite via set_approx_xyz() after the header is emitted.
        self._last_lock_ms: dict[tuple[str, str], int] = {}
        self._last_written_dt: datetime | None = None
        self._epoch_count = 0

    def _expand_path(self, epoch_dt: datetime) -> tuple[Path, str]:
        """Expand path template against the epoch's UTC date.

        Returns (resolved Path, YYYYDDD date string used).
        """
        utc = epoch_dt.astimezone(timezone.utc) if epoch_dt.tzinfo \
            else epoch_dt.replace(tzinfo=timezone.utc)
        date_str = utc.strftime("%Y%j")
        expanded = self._path_tmpl.format(date=date_str, host=self._host)
        p = Path(expanded)
        p.parent.mkdir(parents=True, exist_ok=True)
        return p, date_str

    def _open_for_date(self, epoch_dt: datetime) -> None:
        """Open (or rotate to) the file for this epoch's UTC date.

        Append-safe: if the target already exists with non-zero size
        (wrapper-respawn after exit-5 recovery), open r+ and seek to
        EOF rather than truncating.  Header was emitted by the prior
        process — don't re-write it, but locate the existing APPROX
        POSITION XYZ line so set_approx_xyz() can update it in place.
        """
        p, date_str = self._expand_path(epoch_dt)
        self._path = p
        self._current_date = date_str
        self._approx_pos_offset = None
        if p.exists() and p.stat().st_size > 0:
            self._fp = open(p, "r+")
            self._fp.seek(0, 2)  # SEEK_END
            self._header_written = True
            self._approx_pos_offset = _find_approx_pos_offset(p)
        else:
            self._fp = open(p, "w")
            self._header_written = False
        # Header (when written) carries TIME OF FIRST OBS for this file.
        # Lock-ms continuity across rollover doesn't propagate (next file's
        # first epoch will see no LLI=1 even on a real slip — acceptable
        # for daily-archive use).

    def _write_header(self, first_epoch: datetime) -> None:
        """Emit the RINEX 3.04 OBS header with TIME OF FIRST OBS set."""
        f = self._fp
        # Version line — fixed columns per RINEX 3.04 spec § 5.1:
        #   A9: version (right-justified)
        #   11X: 11 blanks (cols 10-20)
        #   A20: file type — first char ("O") must land at col 21
        #   A20: satellite system
        #   A20: label "RINEX VERSION / TYPE"
        # Type field is "OBSERVATION DATA" with sat-system "M (MIXED)";
        # PRIDE-PPPAR (pdp3.sh:213) rejects the bare "M" form via a
        # strict `cut -c 21-21 == "O"` check so getting both the
        # spelling AND the column alignment right is load-bearing.
        f.write(f"{'3.04':>9}{'':<11}"
                f"{'OBSERVATION DATA':<20}{'M (MIXED)':<20}"
                f"RINEX VERSION / TYPE\n")
        # Pgm / Run by / Date
        now = datetime.now(timezone.utc).strftime("%Y%m%d %H%M%S UTC")
        f.write(f"{'PePPAR Fix engine':<20}{self._observer:<20}"
                f"{now:<20}PGM / RUN BY / DATE\n")
        # Marker + observer + agency
        f.write(f"{self._marker:<60}MARKER NAME\n")
        f.write(f"{'GEODETIC':<20}{'':<40}MARKER TYPE\n")
        f.write(f"{self._observer:<20}{self._agency:<40}OBSERVER / AGENCY\n")
        # Receiver: type + version + serial
        f.write(f"{self._rx_serial:<20}{self._rx_model:<20}"
                f"{self._rx_fw:<20}REC # / TYPE / VERS\n")
        f.write(f"{self._ant_serial:<20}{self._antenna_type:<40}"
                f"ANT # / TYPE\n")
        # Record where APPROX POSITION lives so set_approx_xyz() can
        # rewrite it in place when the engine resolves known_ecef
        # later (NAV2/PPP bootstrap paths).  Fixed-length line.
        f.flush()
        self._approx_pos_offset = f.tell()
        x, y, z = self._approx_xyz
        f.write(_format_approx_pos_line(x, y, z))
        # Antenna delta H/E/N (we don't track these — use 0 0 0)
        f.write(f"{0.0:14.4f}{0.0:14.4f}{0.0:14.4f}{'':<18}"
                f"ANTENNA: DELTA H/E/N\n")
        # Per-system obs types
        for sys_id in ("G", "E", "C"):
            obs = _SYS_OBS_TYPES[sys_id]
            n = len(obs)
            # First continuation line: "X NN OBS1 OBS2 ... OBS13" (max
            # 13 codes per line per RINEX 3.x).
            head = f"{sys_id:<1}  {n:>3}"
            cells = "".join(f" {c}" for c in obs[:13])
            tail = " " * (60 - len(head) - len(cells))
            f.write(f"{head}{cells}{tail}SYS / # / OBS TYPES\n")
            for chunk_start in range(13, n, 13):
                cells = "".join(f" {c}" for c in obs[chunk_start:chunk_start+13])
                f.write(f"{'':<6}{cells:<54}SYS / # / OBS TYPES\n")
        # Interval
        f.write(f"{self._interval:10.3f}{'':<50}INTERVAL\n")
        # Time of first obs (GPS time scale)
        f.write(
            f"{first_epoch.year:6d}"
            f"{first_epoch.month:6d}"
            f"{first_epoch.day:6d}"
            f"{first_epoch.hour:6d}"
            f"{first_epoch.minute:6d}"
            f"{first_epoch.second + first_epoch.microsecond * 1e-6:13.7f}"
            f"     GPS         "
            f"TIME OF FIRST OBS\n")
        f.write(f"{'':<60}END OF HEADER\n")

    def write_epoch(self, epoch_dt: datetime,
                    raw_obs: dict[str, dict[str, dict]]) -> None:
        """Write one epoch of observations.

        epoch_dt: epoch timestamp (GPS-time-equivalent UTC).
        raw_obs: { sv: { sig_internal: { 'pr': float, 'cp': float,
                  'cno': float, 'half_cyc': bool, 'lock_ms': int }, ... }, ... }

        SVs / signals not present are silently omitted (RINEX writer
        emits blank fields for any declared obs-type the SV didn't
        track this epoch).

        If decimate_s was set on the writer, epochs less than that many
        seconds after the previous written epoch are silently dropped
        (the first epoch is always written).

        If the path template contained ``{date}``, the writer rotates
        files at UTC-midnight: the first epoch of a new UTC day closes
        the current file and opens the next.
        """
        # Decimation: skip if too soon since last write.
        if (self._decimate_s is not None
                and self._last_written_dt is not None):
            interval = (epoch_dt - self._last_written_dt).total_seconds()
            if interval < self._decimate_s:
                return

        # Rotation: open initial file or roll over at UTC-midnight when
        # the template has {date}.
        if self._fp is None:
            self._open_for_date(epoch_dt)
        else:
            _, new_date = self._expand_path(epoch_dt)
            if new_date != self._current_date:
                self._fp.flush()
                self._fp.close()
                self._open_for_date(epoch_dt)

        f = self._fp
        if not self._header_written:
            self._write_header(epoch_dt)
            self._header_written = True

        usable_svs = [sv for sv, sigs in raw_obs.items()
                      if _sys_prefix(sv) in _SYS_OBS_TYPES and sigs]
        n = len(usable_svs)
        if n == 0:
            return

        sec = epoch_dt.second + epoch_dt.microsecond * 1e-6
        f.write(f"> {epoch_dt.year:4d} "
                f"{epoch_dt.month:2d} {epoch_dt.day:2d} "
                f"{epoch_dt.hour:2d} {epoch_dt.minute:2d}"
                f"{sec:11.7f}  0{n:3d}\n")

        for sv in sorted(usable_svs):
            sys_id = _sys_prefix(sv)
            obs_codes = _SYS_OBS_TYPES[sys_id]
            cells: dict[str, str] = {}
            for sig_internal, fields in raw_obs[sv].items():
                try:
                    c_code, l_code, s_code = _internal_to_obs_codes(sig_internal)
                except ValueError:
                    continue
                # LLI bit 0 = lost-lock since last; we infer from a drop
                # in lock_ms.
                lock_ms = fields.get("lock_ms")
                key = (sv, sig_internal)
                last = self._last_lock_ms.get(key)
                lli = 0
                if (lock_ms is not None and last is not None
                        and lock_ms < last):
                    lli = 1
                if lock_ms is not None:
                    self._last_lock_ms[key] = lock_ms
                if not fields.get("half_cyc", True):
                    lli |= 2  # bit 1 = half-cycle ambiguity
                ssi = _cno_to_ssi(fields.get("cno"))
                cp_cyc = fields.get("cp")  # cycles
                pr_m = fields.get("pr")    # metres
                if c_code in obs_codes:
                    cells[c_code] = _fmt_obs(pr_m, lli, ssi)
                if l_code in obs_codes:
                    cells[l_code] = _fmt_obs(cp_cyc, lli, ssi)
                if s_code in obs_codes:
                    cells[s_code] = _fmt_obs(fields.get("cno"), 0, 0)

            line = sv + "".join(cells.get(c, " " * 16) for c in obs_codes)
            f.write(line + "\n")
        self._epoch_count += 1
        self._last_written_dt = epoch_dt
        if self._epoch_count % 60 == 0:
            self._fp.flush()

    def set_approx_xyz(self, xyz: tuple[float, float, float]) -> None:
        """Update APPROX POSITION XYZ.  Idempotent.

        Use case: the engine constructs the writer at startup, before
        known_ecef is resolved (NAV2 / PPP bootstrap path takes 30-90s
        to converge).  The writer ships header with the best seed
        available — arp_label → antennas.json or --known-pos — but if
        both were absent the seed is (0,0,0) which makes PRIDE-PPP-AR
        reject every SV as DEL_BADRANGE downstream.

        Calling this after known_ecef is finalized:
          - Before any epoch has been written: just updates the field;
            the eventual first-epoch header write picks up the new
            value.
          - After the header is on disk: seek-writes the same
            fixed-length APPROX POSITION line in place, preserving
            all surrounding bytes.
          - If we can't locate the offset (e.g., respawn on a header
            we didn't author): no-op with a logged warning later.
        """
        xyz_t = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
        self._approx_xyz = xyz_t
        if (self._fp is not None
                and self._header_written
                and self._approx_pos_offset is not None):
            x, y, z = xyz_t
            cur = self._fp.tell()
            self._fp.seek(self._approx_pos_offset)
            self._fp.write(_format_approx_pos_line(x, y, z))
            self._fp.seek(cur)
            self._fp.flush()

    def close(self) -> None:
        if self._fp:
            self._fp.flush()
            self._fp.close()
            self._fp = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


# ── Engine integration helper ─────────────────────────────────────── #

def make_writer_from_args(args, log=None):
    """Build a RinexWriter from a parsed-args Namespace.

    Returns ``None`` if ``args.rinex_out`` isn't set (the no-op case).

    Lives in this module rather than inline in ``peppar_fix_engine.py``
    so the init logic is unit-testable and can't ship NameError /
    UnboundLocalError bugs against undefined call-site locals.

    All optional inputs are looked up via ``getattr(args, name, None)``
    and ``.get(key, default)``, so missing values resolve cleanly to
    sensible header defaults.

    Engine should call this at startup once, after argparse has parsed
    args and before threads start:

    ```python
    from peppar_fix.rinex_writer import make_writer_from_args
    rinex_writer_obj = make_writer_from_args(args, log=log)
    if rinex_writer_obj is not None:
        serial_kwargs['rinex_writer'] = rinex_writer_obj
    ```
    """
    rinex_out = getattr(args, 'rinex_out', None)
    if not rinex_out:
        return None

    # Receiver metadata lookup — feed RinexWriter's header from the
    # saved receiver-state file when one exists.  Missing fields fall
    # through to the kwargs' defaults.
    receiver_meta: dict = {}
    rx_uid = getattr(args, 'receiver_unique_id', None)
    if rx_uid:
        try:
            from peppar_fix.receiver_state import load_receiver_state
            loaded = load_receiver_state(rx_uid)
            if loaded:
                receiver_meta = loaded
        except Exception as e:
            if log is not None:
                log.warning("RINEX writer: receiver-state lookup failed "
                            "(%s); using header defaults", e)

    # Derive APPROX POSITION XYZ.  PRIDE-PPP-AR uses this as the
    # first-epoch seed and rejects every SV as DEL_BADRANGE if it's
    # (0,0,0) (seed lands ~6378 km off, PR residuals diverge).
    # Sources tried, in priority order:
    #   1. --known-pos          (operator-supplied LLA)
    #   2. arp_label            (lab antenna database in antennas.json)
    # If both miss, ship (0,0,0) and rely on the engine to call
    # set_approx_xyz() once NAV2/PPP bootstrap resolves known_ecef.
    approx_xyz: tuple[float, float, float] | None = None
    known_pos = getattr(args, 'known_pos', None)
    if known_pos:
        try:
            lat, lon, alt = (float(s) for s in known_pos.split(','))
            # Lazy import to keep this module's deps light.
            from solve_ppp import lla_to_ecef
            ecef = lla_to_ecef(lat, lon, alt)
            approx_xyz = (float(ecef[0]), float(ecef[1]), float(ecef[2]))
        except Exception as e:
            if log is not None:
                log.warning("RINEX writer: --known-pos parse failed "
                            "(%s); trying arp_label fallback", e)

    if approx_xyz is None:
        arp_label = getattr(args, 'arp_label', None)
        if arp_label:
            try:
                from peppar_fix.position_state import load_arp_from_antennas
                state = load_arp_from_antennas(
                    arp_label,
                    mount_sn=0,  # unused for header derivation
                    antennas_path=getattr(args, 'antennas_json', None),
                )
                if state is not None:
                    e_m = state.ecef_m
                    approx_xyz = (float(e_m[0]), float(e_m[1]), float(e_m[2]))
            except Exception as e:
                if log is not None:
                    log.warning("RINEX writer: arp_label '%s' lookup "
                                "failed (%s); APPROX XYZ stays (0,0,0)",
                                arp_label, e)

    if approx_xyz is None:
        if log is not None:
            log.warning(
                "RINEX writer: APPROX POSITION XYZ unresolved at startup "
                "(no --known-pos, no arp_label in antennas.json).  "
                "PRIDE-PPP-AR downstream will reject SVs as DEL_BADRANGE "
                "until the engine calls set_approx_xyz() with known_ecef.")
        approx_xyz = (0.0, 0.0, 0.0)

    import os
    decimate_s = getattr(args, 'rinex_decimate_s', None)
    return RinexWriter(
        rinex_out,
        marker_name=getattr(args, 'peer_antenna_ref', '') or 'UFO1',
        approx_xyz=tuple(approx_xyz),
        antenna_type=(getattr(args, 'receiver_antenna', None) or
                      'SFESPK6618H     NONE'),
        receiver_model=str(receiver_meta.get('module', 'ZED-F9T')),
        receiver_fw=str(receiver_meta.get('firmware', '')),
        receiver_serial=str(receiver_meta.get('unique_id_hex', '')),
        antenna_serial=getattr(args, 'peer_antenna_ref', '') or '',
        observer='PePPAR Fix engine',
        agency='lab',
        interval_s=1.0,
        decimate_s=decimate_s,
        host=os.uname().nodename.split('.')[0],
    )

#!/usr/bin/env python3
"""TDCP (time-differenced carrier phase) frequency-error prototype.

Read-only analysis of carrier-phase observations from a RINEX OBS file,
producing a per-epoch clock frequency-error estimate via time-differenced
carrier phase.

Goal: characterize the achievable short-tau (1-100 s) frequency-stability
floor of a TDCP-only estimator on real F9T data, comparing against:
  - Raw PPS:               TDEV(1s) ~ 3 ns
  - PPS + qErr correction: TDEV(1s) ~ 250 ps
  - Current FixedPosFilter dt_rx: TDEV(1s) ~ 450 ps
  - Bare carrier-phase floor (single SV, no ensemble): TDEV(1s) ~ 50 ps

Math:
  For SV i at epochs t and t-Δt, with continuous lock (no slip):
    Δφ_meas(i) ≈ Δρ_geom(i) - c·Δdt_sat(i) + c·Δdt_rx + Δtropo(i) + ε
  where Δφ_meas, Δρ_geom are in metres (cycles × λ).

  Per-epoch ensemble estimator:
    c·Δdt_rx = median over SVs of (Δφ_meas - Δρ_geom + c·Δdt_sat)
  Outlier rejection: drop SVs > 3·MAD from median.

  Frequency: f_rx = (c·Δdt_rx) / (c · Δt) = Δdt_rx / Δt
  (dimensionless, fractional frequency offset df/f)

Ephemeris: parses a RINEX 3 NAV file directly, reuses
broadcast_eph.py's _kepler_ecef and _sat_clock for the math.

Scope of the prototype:
  - Single frequency (L1 only) — clean math, ~50 ps noise floor
  - GPS-only — most stable broadcast nav, simplest case
  - Read from RINEX OBS file (regression/rinex_reader.py)
  - Output: CSV of per-epoch (sow, n_sv, c_dt_rx_m, df_f_unitless)
            + TDEV(τ) summary

No servo wiring, no engine integration. Just measurement.
"""

from __future__ import annotations

import argparse
import logging
import math
import re
import sys
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from statistics import median

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "scripts"))
sys.path.insert(0, str(_REPO_ROOT / "scripts" / "regression"))

from broadcast_eph import _kepler_ecef, _sat_clock, GM_GPS, GM_GAL, GM_BDS  # noqa: E402
from broadcast_eph import GPS_EPOCH, SECONDS_PER_WEEK, BDT_GPST_OFFSET, OMEGA_E, F_REL  # noqa: E402
import rinex_reader as rr  # noqa: E402

log = logging.getLogger("tdcp_proto")

C_LIGHT = 299_792_458.0


# ── RINEX 3 NAV parsing ───────────────────────────────────────────────── #

# Map RINEX NAV field positions (record line + offset) to eph dict keys
# for the GPS broadcast nav record (IS-GPS-200 / RINEX 3 spec).
# Each NAV record is 8 lines for GPS, 8 for GAL, 8 for BDS.
# Line 0: PRN + epoch + (af0 af1 af2)
# Lines 1-7: 4 fields each, 19 chars wide.

# Field layout for GPS NAV record after the PRN+epoch line:
# Index = (line_number - 1) * 4 + position_in_line
_GPS_FIELDS = {
    # line 1: IODE  Crs  delta_n  M0
    0: "IODE", 1: "Crs", 2: "delta_n", 3: "M0",
    # line 2: Cuc  e  Cus  sqrt_a
    4: "Cuc", 5: "e", 6: "Cus", 7: "sqrt_a",
    # line 3: toe  Cic  omega0  Cis
    8: "toe", 9: "Cic", 10: "omega0", 11: "Cis",
    # line 4: i0  Crc  omega  omega_dot
    12: "i0", 13: "Crc", 14: "omega", 15: "omega_dot",
    # line 5: i_dot  L2codes  GPS_week  L2P_flag
    16: "i_dot", 17: "_L2codes", 18: "_GPS_week", 19: "_L2P",
    # line 6: SV_accuracy  health  tgd  IODC
    20: "_SV_accuracy", 21: "health", 22: "tgd", 23: "_IODC",
    # line 7: Tx_time  fit_interval
    24: "_Tx_time", 25: "_fit_interval",
}
# GAL has same Keplerian layout — different secondary fields but those
# don't affect orbit/clock for our purposes.
_GAL_FIELDS = dict(_GPS_FIELDS)
# BDS follows the same Keplerian layout per RTCM 1042 / RINEX 3 spec.
_BDS_FIELDS = dict(_GPS_FIELDS)


def _f(s: str) -> float:
    """Parse a RINEX float field (uses 'D' for exponent in some sources)."""
    return float(s.replace("D", "E").replace("d", "e"))


def parse_rinex_nav(path: Path, sysprefix: str = "G") -> dict[str, list[dict]]:
    """Parse a RINEX 3 mixed NAV file, return {prn: [eph_dict, ...]}.

    Reuses broadcast_eph.py's expected eph-dict field names so downstream
    Kepler computation Just Works.

    Args:
        sysprefix: 'G' for GPS, 'E' for GAL, 'C' for BDS, or '*' for all.
    """
    out: dict[str, list[dict]] = {}
    syskey = None
    field_map = None
    record_lines = 8  # GPS, GAL, BDS all 8 lines in RINEX 3
    with path.open() as f:
        # Skip header
        for line in f:
            if "END OF HEADER" in line:
                break

        # Body
        while True:
            line = f.readline()
            if not line:
                break
            if len(line) < 4 or line[0] == " ":
                continue
            sys_char = line[0]
            if sys_char not in ("G", "E", "C", "R", "J", "I", "S"):
                continue
            if sysprefix != "*" and sys_char != sysprefix:
                # consume the rest of this record
                for _ in range(record_lines - 1):
                    f.readline()
                continue
            if sys_char == "G":
                field_map = _GPS_FIELDS
                gm = GM_GPS
            elif sys_char == "E":
                field_map = _GAL_FIELDS
                gm = GM_GAL
            elif sys_char == "C":
                field_map = _BDS_FIELDS
                gm = GM_BDS
            else:
                # Skip GLO, etc. (not supported by broadcast_eph here)
                for _ in range(record_lines - 1):
                    f.readline()
                continue

            try:
                prn = f"{sys_char}{int(line[1:3]):02d}"
            except ValueError:
                continue
            # Epoch (toc): yyyy mm dd hh mm ss
            try:
                toc = datetime(
                    int(line[4:8]), int(line[9:11]), int(line[12:14]),
                    int(line[15:17]), int(line[18:20]), int(line[21:23]),
                    tzinfo=timezone.utc,
                )
            except ValueError:
                continue
            # af0 af1 af2 on line 0 starting at col 23 (4 chars then 3×19)
            af0 = _f(line[23:42])
            af1 = _f(line[42:61])
            af2 = _f(line[61:80])

            eph: dict = {
                "system": sys_char, "gm": gm, "toc": toc,
                "af0": af0, "af1": af1, "af2": af2,
            }
            # Lines 1-7
            for lineno in range(1, record_lines):
                row = f.readline()
                if len(row) < 4:
                    break
                # 4 fields per line, each 19 chars starting at col 4
                for col in range(4):
                    s = row[4 + col * 19: 4 + (col + 1) * 19]
                    if not s.strip():
                        continue
                    idx = (lineno - 1) * 4 + col
                    key = field_map.get(idx)
                    if key is None or key.startswith("_"):
                        continue
                    try:
                        eph[key] = _f(s)
                    except ValueError:
                        pass

            # Default tgd to 0 if missing (some BDS/GAL records may omit)
            eph.setdefault("tgd", 0.0)
            eph.setdefault("health", 0.0)
            out.setdefault(prn, []).append(eph)

    return out


# ── Satellite position interface (mimics BroadcastEphemeris) ─────────────#

@dataclass
class _SimpleEph:
    """Wraps a dict-of-lists ephemeris store with sat_position(prn, t)."""

    ephs: dict[str, list[dict]]

    def sat_position(self, prn: str, t: datetime) -> tuple[np.ndarray | None, float | None]:
        records = self.ephs.get(prn)
        if not records:
            return None, None
        # Pick record with closest toc (smallest |t - toc|)
        best = min(records, key=lambda e: abs((t - e["toc"]).total_seconds()))
        sys = best["system"]
        gm = best["gm"]
        # Seconds-of-week
        if sys == "C":
            # BDT = GPST - 14
            gps_delta = (t - GPS_EPOCH).total_seconds()
            bdt_delta = gps_delta - BDT_GPST_OFFSET
            week = int(bdt_delta // SECONDS_PER_WEEK)
            sow = bdt_delta - week * SECONDS_PER_WEEK
        else:
            gps_delta = (t - GPS_EPOCH).total_seconds()
            week = int(gps_delta // SECONDS_PER_WEEK)
            sow = gps_delta - week * SECONDS_PER_WEEK
        toe = best.get("toe")
        if toe is None:
            return None, None
        tk = sow - toe
        # Week crossover
        if tk > SECONDS_PER_WEEK / 2:
            tk -= SECONDS_PER_WEEK
        elif tk < -SECONDS_PER_WEEK / 2:
            tk += SECONDS_PER_WEEK
        try:
            pos, Ek = _kepler_ecef(best, tk, gm)
        except (KeyError, ValueError, ZeroDivisionError):
            return None, None
        # Clock: dt_clk = sow_now - sow_of_toc_record
        if sys == "C":
            toc_gps_delta = (best["toc"] - GPS_EPOCH).total_seconds() - BDT_GPST_OFFSET
        else:
            toc_gps_delta = (best["toc"] - GPS_EPOCH).total_seconds()
        toc_sow = toc_gps_delta - (toc_gps_delta // SECONDS_PER_WEEK) * SECONDS_PER_WEEK
        dt_clk = sow - toc_sow
        if dt_clk > SECONDS_PER_WEEK / 2:
            dt_clk -= SECONDS_PER_WEEK
        elif dt_clk < -SECONDS_PER_WEEK / 2:
            dt_clk += SECONDS_PER_WEEK
        try:
            clk = _sat_clock(best, dt_clk, Ek)
        except (KeyError, ValueError):
            clk = 0.0
        return pos, clk


# ── TDCP estimator ───────────────────────────────────────────────────────#

@dataclass
class _SvPrev:
    """Previous-epoch state for an SV."""
    gps_time_s: float       # GPS seconds (continuous time)
    phi_m: float            # carrier phase in metres (cycles × λ)
    rho_geom_m: float       # geometric range to sat at prev epoch
    sat_clk_m: float        # c · sat clock offset at prev epoch


class TdcpEstimator:
    """Time-differenced carrier-phase clock-frequency estimator.

    Per-epoch flow:
      1. For each SV with continuous lock (no LLI), compute its TDCP
         residual:  r(sv) = Δφ_meas - (Δρ_geom - Δ(c·dt_sat))
         where the right-hand side is the predicted change in
         pseudorange-equivalent units, and r(sv) ≈ c·Δdt_rx + noise.
      2. Ensemble: median across SVs, with 3-MAD outlier rejection.
      3. Output the per-epoch (c·Δdt_rx, df/f, n_sv_used).
    """

    def __init__(self, eph: _SimpleEph, rx_ecef: np.ndarray,
                 max_dt_s: float = 31.0, mad_k: float = 3.0):
        self.eph = eph
        self.rx_ecef = np.asarray(rx_ecef, dtype=float)
        self.max_dt_s = max_dt_s   # drop SV if epoch gap exceeds this
        self.mad_k = mad_k         # outlier threshold
        self._prev: dict[str, _SvPrev] = {}

    def update(self, obs_epoch: list[dict], t: datetime) -> dict:
        """Process one epoch's observation dicts. Return ensemble result.

        Each obs dict must contain: 'sv', 'phi1_cyc', 'wl_f1', 'gps_time'
        (seconds, continuous), and 'lock_duration_ms' or LLI proxy.
        """
        # Compute per-SV current state + residual against prev
        residuals_m: list[tuple[str, float, float]] = []  # (sv, r, cn0)
        new_prev: dict[str, _SvPrev] = {}
        # Track epoch interval for df/f conversion (use the most common
        # dt across SVs — they should all be identical at a given epoch).
        dts_seen: list[float] = []

        for o in obs_epoch:
            sv = o["sv"]
            phi_cyc = o.get("phi1_cyc")
            wl = o.get("wl_f1")
            cn0 = o.get("cno", 0.0) or 0.0
            t_now = o.get("gps_time")
            if phi_cyc is None or wl is None or t_now is None:
                continue
            phi_m = phi_cyc * wl

            pos, clk = self.eph.sat_position(sv, t)
            if pos is None:
                continue
            sat_clk_m = clk * C_LIGHT

            # Apply Sagnac (light-travel-time) Earth rotation correction.
            # For TDCP, the Earth-rotation effect on Δρ is dominated by
            # the change in line-of-sight, but applying Sagnac at each
            # epoch keeps the math consistent.
            # Iterative range solve (1 pass is enough for ~m-level rx_pos):
            d = pos - self.rx_ecef
            rho = float(np.linalg.norm(d))
            tau = rho / C_LIGHT
            theta = OMEGA_E * tau
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            pos_rot = np.array([
                cos_t * pos[0] + sin_t * pos[1],
                -sin_t * pos[0] + cos_t * pos[1],
                pos[2],
            ])
            d = pos_rot - self.rx_ecef
            rho_geom_m = float(np.linalg.norm(d))

            prev = self._prev.get(sv)
            cur = _SvPrev(gps_time_s=t_now, phi_m=phi_m,
                          rho_geom_m=rho_geom_m, sat_clk_m=sat_clk_m)
            new_prev[sv] = cur

            if prev is None:
                continue
            dt = t_now - prev.gps_time_s
            if dt <= 0 or dt > self.max_dt_s:
                continue
            dts_seen.append(dt)

            # LLI bit 0 = loss-of-lock since previous epoch ⇒ slip ⇒
            # update prev (so next epoch can diff against this one) but
            # skip this epoch's residual.
            lli = o.get("lli", 0) or 0
            if lli & 1:
                continue

            d_phi = phi_m - prev.phi_m
            d_rho = rho_geom_m - prev.rho_geom_m
            d_sat = sat_clk_m - prev.sat_clk_m

            # TDCP residual.  Positive ≈ rx clock advanced faster than
            # nominal (rx ahead of GPS time).
            r = d_phi - (d_rho - d_sat)
            residuals_m.append((sv, r, cn0))

        self._prev = new_prev
        self._last_dt_s = float(median(dts_seen)) if dts_seen else 1.0

        if not residuals_m:
            return {"t": t, "n_total": len(obs_epoch), "n_used": 0,
                    "median_r_m": float("nan"), "mad_r_m": float("nan"),
                    "c_dt_rx_m": float("nan"), "df_f": float("nan"),
                    "kept_svs": []}

        # 3-MAD outlier rejection
        rs = np.array([r[1] for r in residuals_m])
        med = float(np.median(rs))
        mad = float(np.median(np.abs(rs - med))) * 1.4826  # → sigma equiv
        if mad < 0.001:
            mad = 0.001  # floor: 1 mm
        keep_mask = np.abs(rs - med) <= self.mad_k * mad
        kept = [residuals_m[i] for i in range(len(residuals_m))
                if keep_mask[i]]

        if not kept:
            return {"t": t, "n_total": len(obs_epoch),
                    "n_used": 0, "median_r_m": med, "mad_r_m": mad,
                    "c_dt_rx_m": float("nan"), "df_f": float("nan"),
                    "kept_svs": []}

        # Final ensemble: median of kept residuals (robust)
        c_dt_rx_m = float(np.median([k[1] for k in kept]))
        # df/f over the 1-second interval
        # c·Δdt_rx [m] / c [m/s] / Δt [s] = Δdt_rx / Δt = df/f
        # df/f = Δdt_rx / Δt.  We tracked Δt during the per-SV pass.
        dt = max(0.1, getattr(self, "_last_dt_s", 1.0))
        df_f = (c_dt_rx_m / C_LIGHT) / dt

        return {
            "t": t,
            "n_total": len(obs_epoch),
            "n_used": len(kept),
            "median_r_m": med,
            "mad_r_m": mad,
            "c_dt_rx_m": c_dt_rx_m,
            "df_f": df_f,
            "kept_svs": [k[0] for k in kept],
        }


# ── Driver ───────────────────────────────────────────────────────────────#

def _gps_time_from_dt(t: datetime) -> float:
    """Datetime → continuous GPS seconds since GPS epoch (no week rollover)."""
    return (t - GPS_EPOCH).total_seconds()


# Per-system L1 codes (RINEX 3 conventions).
#   GPS L1 C/A → L1C/S1C @ 1575.42 MHz
#   GAL E1     → L1C/S1C @ 1575.42 MHz
#   BDS B1I    → L2I/S2I @ 1561.098 MHz  (RINEX band 2 is BDS B1)
_RINEX_SYS_L1 = {
    "G": ("L1C", "S1C", 1575.42e6),
    "E": ("L1C", "S1C", 1575.42e6),
    "C": ("L2I", "S2I", 1561.098e6),
}


def _iter_rinex_epochs(path: Path):
    """Yield (datetime_utc, [obs_dict, ...]) from a RINEX 3 OBS file."""
    for epoch in rr.iter_epochs(Path(path)):
        t_dt = epoch.ts.replace(tzinfo=timezone.utc)
        t_s = _gps_time_from_dt(t_dt)
        flat = []
        for sv, sv_obs in epoch.obs.items():
            sys_char = sv[0]
            codes = _RINEX_SYS_L1.get(sys_char)
            if not codes:
                continue
            ph_code, snr_code, f1_hz = codes
            ph_entry = sv_obs.get(ph_code)
            if ph_entry is None:
                continue
            phi_cyc, lli, _ssi = ph_entry
            if phi_cyc == 0:
                continue
            snr_entry = sv_obs.get(snr_code)
            cno = snr_entry[0] if snr_entry else 0.0
            flat.append({
                "sv": sv, "phi1_cyc": phi_cyc, "wl_f1": C_LIGHT / f1_hz,
                "cno": cno, "gps_time": t_s, "lli": lli,
            })
        yield t_dt, flat


# UBX RAWX (gnssId, sigId) → (sys_char, freq_hz).  L1-band only for the
# single-frequency TDCP prototype.
_UBX_L1_SIG = {
    (0, 0): ("G", 1575.42e6),     # GPS L1CA
    (2, 0): ("E", 1575.42e6),     # GAL E1C
    (2, 1): ("E", 1575.42e6),     # GAL E1B
    (3, 0): ("C", 1561.098e6),    # BDS B1I
}


def _iter_ubx_epochs(path: Path):
    """Yield (datetime_utc, [obs_dict, ...]) from a UBX byte-stream log.

    Reads UBX-RXM-RAWX messages.  L1-band signals only.  Slip detection
    uses the UBX trkStat field (bit 1 = phase valid) plus a per-SV
    lock-time monotonicity check (locktime resetting to 0 ⇒ slip).
    """
    from pyubx2 import UBXReader, UBX_PROTOCOL

    # Per-SV lock-time tracking — drop epoch's obs if locktime < prior.
    prior_lock: dict[str, int] = {}

    with open(path, "rb") as f:
        ubr = UBXReader(f, protfilter=UBX_PROTOCOL)
        for raw, parsed in ubr:
            if parsed is None or parsed.identity != "RXM-RAWX":
                continue
            week = parsed.week
            tow = parsed.rcvTow
            try:
                t_dt = GPS_EPOCH + timedelta(
                    seconds=week * SECONDS_PER_WEEK + tow)
            except (OverflowError, ValueError):
                continue
            t_s = (t_dt - GPS_EPOCH).total_seconds()
            num = parsed.numMeas

            flat = []
            new_lock: dict[str, int] = {}
            for i in range(1, num + 1):
                idx = f"_{i:02d}"
                gnss_id = getattr(parsed, f"gnssId{idx}", None)
                sv_id = getattr(parsed, f"svId{idx}", None)
                sig_id = getattr(parsed, f"sigId{idx}", None)
                cp_mes = getattr(parsed, f"cpMes{idx}", None)
                cno = getattr(parsed, f"cno{idx}", None)
                cp_valid = getattr(parsed, f"cpValid{idx}", 0) or 0
                lock = getattr(parsed, f"locktime{idx}", 0) or 0
                if gnss_id is None or sv_id is None or sig_id is None:
                    continue
                key = (gnss_id, sig_id)
                meta = _UBX_L1_SIG.get(key)
                if not meta:
                    continue
                sys_char, f_hz = meta
                if cp_mes is None or cp_mes == 0:
                    continue
                if not cp_valid:
                    continue  # phase invalid per receiver
                sv = f"{sys_char}{sv_id:02d}"
                # Slip flag: locktime reset
                prev = prior_lock.get(sv, 0)
                lli = 1 if lock < prev else 0
                new_lock[sv] = lock
                flat.append({
                    "sv": sv, "phi1_cyc": cp_mes, "wl_f1": C_LIGHT / f_hz,
                    "cno": cno or 0.0, "gps_time": t_s, "lli": lli,
                })
            prior_lock = new_lock
            yield t_dt, flat


def run(obs_path: Path, nav_path: Path, rx_ecef: tuple[float, float, float],
        out_csv: Path | None, max_epochs: int | None,
        systems: str) -> tuple[list[dict], dict]:
    """Run the TDCP estimator. Return list of per-epoch results + summary."""
    log.info("Parsing NAV: %s", nav_path)
    nav_ephs: dict = {}
    for prefix in systems:
        sub = parse_rinex_nav(nav_path, sysprefix=prefix)
        log.info("  %s: %d SVs, %d records",
                 prefix, len(sub), sum(len(v) for v in sub.values()))
        nav_ephs.update(sub)

    eph = _SimpleEph(nav_ephs)
    est = TdcpEstimator(eph, np.array(rx_ecef))

    log.info("Reading OBS: %s", obs_path)
    if str(obs_path).endswith(".ubx"):
        epoch_iter = _iter_ubx_epochs(obs_path)
    else:
        epoch_iter = _iter_rinex_epochs(obs_path)

    results: list[dict] = []
    n_epochs = 0
    for t_dt, flat in epoch_iter:
        n_epochs += 1
        if max_epochs and n_epochs > max_epochs:
            break
        flat = [o for o in flat if o["sv"][0] in systems]
        out = est.update(flat, t_dt)
        results.append(out)

    # Summary stats
    df_f_vals = np.array([r["df_f"] for r in results if not math.isnan(r["df_f"])])
    n_used = np.array([r["n_used"] for r in results])
    summary = {
        "epochs_total": len(results),
        "epochs_with_solution": int(np.sum(~np.isnan([r["df_f"] for r in results]))),
        "median_n_sv_used": float(np.median(n_used)) if len(n_used) else 0.0,
        "df_f_mean": float(np.mean(df_f_vals)) if len(df_f_vals) else float("nan"),
        "df_f_std": float(np.std(df_f_vals)) if len(df_f_vals) else float("nan"),
        "df_f_rms_post_detrend": (
            float(np.std(df_f_vals - np.poly1d(np.polyfit(
                np.arange(len(df_f_vals)), df_f_vals, 1))(np.arange(len(df_f_vals)))))
            if len(df_f_vals) > 100 else float("nan")
        ),
    }

    if out_csv:
        with out_csv.open("w") as f:
            f.write("# tdcp_proto.py output\n")
            f.write("gps_seconds,n_total,n_used,c_dt_rx_m,df_f,median_r_m,mad_r_m\n")
            for r in results:
                t_s = (r["t"] - GPS_EPOCH).total_seconds()
                f.write(f"{t_s:.3f},{r['n_total']},{r['n_used']},"
                        f"{r['c_dt_rx_m']:.6e},{r['df_f']:.6e},"
                        f"{r['median_r_m']:.6e},{r['mad_r_m']:.6e}\n")
        log.info("Wrote CSV: %s", out_csv)

    return results, summary


def _try_tdev(results: list[dict]) -> None:
    """Compute TDEV(τ) on the TDCP-derived frequency series.

    Implementation note: allantools.tdev expects either:
      - phase samples (s) with `data_type="phase"` and `rate` in Hz
      - fractional frequency samples (dimensionless) with
        `data_type="freq"` and `rate` in Hz

    We have df/f samples at the obs file's epoch interval (often 30 s
    in archived files, 1 s in live captures).  Pass them as `freq` and
    let allantools handle the dt scaling.
    """
    try:
        import allantools
    except ImportError:
        print("allantools not available; skipping TDEV")
        return

    df_f = np.array([r["df_f"] for r in results])
    valid = ~np.isnan(df_f)
    if np.sum(valid) < 30:
        print(f"TDEV: only {int(np.sum(valid))} valid epochs — too few")
        return

    # Detect cadence from epoch timestamps (gps seconds in results)
    ts = np.array([(r["t"] - GPS_EPOCH).total_seconds() for r in results])
    dts = np.diff(ts[valid])
    dt_med = float(np.median(dts)) if len(dts) else 1.0
    rate = 1.0 / dt_med

    df_clean = np.where(valid, df_f, np.nan)
    # Drop NaNs by interpolation (allantools doesn't like NaN)
    nan_mask = np.isnan(df_clean)
    if np.any(nan_mask):
        idx = np.arange(len(df_clean))
        df_clean = np.interp(idx, idx[~nan_mask], df_clean[~nan_mask])

    # Detrend the mean: TDEV is on residuals around mean df/f
    df_clean = df_clean - np.mean(df_clean)

    # Reasonable taus given the data length + cadence
    max_tau = int(len(df_clean) * dt_med / 3)
    taus = [t for t in [1, 2, 5, 10, 30, 60, 120, 300, 600, 1200, 3600]
            if t >= dt_med and t <= max_tau]
    if not taus:
        print(f"TDEV: time series too short (N={len(df_clean)}, dt={dt_med}s)")
        return

    tau_out, tdev, _, _ = allantools.tdev(df_clean, rate=rate,
                                          data_type="freq", taus=taus)
    print()
    print(f"TDEV(τ) on TDCP df/f series  (sample dt = {dt_med:.1f} s, "
          f"N = {len(df_clean)})")
    print(f"  {'τ (s)':>8}  {'TDEV (s)':>14}  {'TDEV':>14}")
    for τ, td in zip(tau_out, tdev):
        if td < 1e-9:
            disp = f"{td * 1e12:.1f} ps"
        elif td < 1e-6:
            disp = f"{td * 1e9:.2f} ns"
        else:
            disp = f"{td * 1e6:.3f} µs"
        print(f"  {τ:>8.1f}  {td:>14.3e}  {disp:>14}")
    print()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--obs", required=True, type=Path, help="RINEX OBS file")
    ap.add_argument("--nav", required=True, type=Path, help="RINEX NAV file")
    ap.add_argument("--rx-ecef", default="157470.0,-4756189.5,4232768.0",
                    help="Approx receiver ECEF X,Y,Z in metres (10 m OK)")
    ap.add_argument("--out", type=Path, default=None,
                    help="Output CSV path (optional)")
    ap.add_argument("--max-epochs", type=int, default=None,
                    help="Limit epochs (smoke test)")
    ap.add_argument("--systems", default="G",
                    help="Constellation prefixes (G/E/C). Default: GPS only.")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(name)s: %(message)s",
    )

    rx = tuple(float(x) for x in args.rx_ecef.split(","))
    if len(rx) != 3:
        raise SystemExit("--rx-ecef must be X,Y,Z")

    results, summary = run(args.obs, args.nav, rx, args.out,
                           args.max_epochs, args.systems)

    print()
    print("== Summary ==")
    for k, v in summary.items():
        print(f"  {k:32s} {v}")
    print()

    _try_tdev(results)
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Migrate legacy DO state JSON → the per-section schema (PR 3 of
doCharacterizationArchitecture / I-192603-charlie).

For each DO it converts the legacy state/dos/<uid>.json (+ any
<uid>_noise.json / <uid>-v2.json / <uid>.bak* variants) into three files:

  state/dos/<uid>.toml          characterization (tool-write-only)
  state/dos/<uid>.runtime.toml  operational state (engine-write-only)
  state/dos/<uid>.noise.toml    sidecar — the dense ADEV/TDEV/PSD curves
                                (the one genuinely irreplaceable artifact)

then validates the new files through the schema loader, prints a
per-section PROVENANCE summary (so it's obvious which sections fell to
class-default and need re-characterization), and deletes the old JSON.

Design: docs/do-characterization-architecture.md §"Migration".  One-shot,
supervised, no .preMigration backups (the sidecar IS the new home of the
dense data; everything else recomputes or class-defaults).  Default is
DRY-RUN — pass --apply to write + delete.

Authoritative selection: when several JSON variants exist for one uid, the
one carrying a DO-pointing freerun channel wins (chA-vs-Rb preferred over
chA-chB); the rest are reported as dropped.  Receiver-side / control-loop
sources are never migrated — they are exactly the PiFace channel-mix the
new schema makes unrepresentable.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys

# Allow running as a plain script (scripts/ is not a package).
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))

from peppar_fix import do_schema
from peppar_fix.do_state import _sigma_do_freq_ppb_from_adev


# DO-pointing freerun channels, preference order (matches the schema enum).
_DO_CHANNELS = (
    "DO PPS (chA vs TICC Rb)",
    "DO PPS (chA-chB)",
)


# ── TOML serialization (hand-formatted; project avoids tomli_w) ──────── #

def _q(s: str) -> str:
    return '"' + str(s).replace("\\", "\\\\").replace('"', '\\"') + '"'


def _num(x: float) -> str:
    # Faithful round-trip without scientific-notation surprises in TOML.
    return repr(float(x))


# ── rising-tail TDEV fit (coast-cap power law) ──────────────────────── #

def derive_coast_params(tdev_by_tau: dict):
    """Coast-cap power-law params: (coast_tdev_ref_ns, coast_tdev_slope).

    Per docs/do-characterization-architecture.md the schema models the DO's
    freerun TDEV as TDEV(τ) = ref·(τ/1s)^slope where:
      ref   = the ACTUAL measured TDEV at τ=1 s (pins the magnitude), and
      slope = the RWFM/flicker RISING-tail slope.

    A real OCXO TDEV curve is U-shaped (flicker-phase falling, then RWFM
    rising), so a single power law can't fit both arms.  Anchoring ref at
    the true 1 s value and using the rising slope OVER-estimates TDEV in the
    dip — the conservative/safe direction for a coast cap (caps the coast
    shorter, never longer).  This is also why the legacy global log-log fit
    (derive_coast_tdev_from_char) is wrong: it can return slope ≤ 0 on the
    U-curve and silently disable Goldilocks (bravo's #172 forward note).

    Returns (ref_1s, slope) with slope > 0, or None if no usable rising tail.
    """
    pairs = []
    for k, v in (tdev_by_tau or {}).items():
        try:
            tau, td = float(k), float(v)
        except (TypeError, ValueError):
            continue
        if tau > 0 and math.isfinite(td) and td > 0:
            pairs.append((tau, td))
    if len(pairs) < 3:
        return None
    pairs.sort()
    # ref = actual TDEV at τ=1 s (log-log interpolate / nearest if absent).
    ref_1s = _tdev_at_1s(pairs)
    if ref_1s is None or ref_1s <= 0:
        return None
    # slope = rising tail (from the τ of minimum TDEV outward).
    i_min = min(range(len(pairs)), key=lambda i: pairs[i][1])
    tail = pairs[i_min:]
    if len(tail) < 2:
        return None
    n = len(tail)
    sx = sum(math.log(t) for t, _ in tail)
    sy = sum(math.log(d) for _, d in tail)
    sxx = sum(math.log(t) ** 2 for t, _ in tail)
    sxy = sum(math.log(t) * math.log(d) for t, d in tail)
    denom = n * sxx - sx * sx
    if denom == 0:
        return None
    slope = (n * sxy - sx * sy) / denom
    if slope <= 0:
        return None
    return (ref_1s, slope)


def _tdev_at_1s(pairs):
    """Actual TDEV at τ=1 s: exact entry, else log-log interpolation between
    the bracketing τ, else the nearest-τ value (sorted (τ, tdev) pairs)."""
    for tau, td in pairs:
        if tau == 1.0:
            return td
    below = [(t, d) for t, d in pairs if t < 1.0]
    above = [(t, d) for t, d in pairs if t > 1.0]
    if below and above:
        t0, d0 = below[-1]
        t1, d1 = above[0]
        # log-log linear interpolation at log(τ)=0
        f = (0.0 - math.log(t0)) / (math.log(t1) - math.log(t0))
        return math.exp(math.log(d0) + f * (math.log(d1) - math.log(d0)))
    return pairs[0][1] if not below else below[-1][1]


# ── legacy discovery + authoritative selection ──────────────────────── #

def _safe_uid(uid: str) -> str:
    return str(uid).replace(":", "-").replace("/", "_")


def discover_variants(uid: str, dos_dir: str) -> list:
    """All legacy files for a uid: <uid>.json, _noise.json, -v2.json, .bak*."""
    s = _safe_uid(uid)
    pats = [f"{s}.json", f"{s}_noise.json", f"{s}-v*.json",
            f"{s}.json.bak*", f"{s}.bak*"]
    out: list = []
    for p in pats:
        out.extend(glob.glob(os.path.join(dos_dir, p)))
    # stable, de-duped
    return sorted(set(out))


def _load_json(path: str):
    try:
        with open(path) as f:
            return json.load(f)
    except (json.JSONDecodeError, OSError):
        return None


def select_source(data: dict):
    """Return (channel_name, source_dict) for the best DO-pointing freerun
    source in a JSON payload, or (None, None)."""
    if not isinstance(data, dict):
        return (None, None)
    char = data.get("characterization")
    sources = char.get("sources") if isinstance(char, dict) else None
    if not isinstance(sources, dict):
        return (None, None)
    for ch in _DO_CHANNELS:
        src = sources.get(ch)
        if isinstance(src, dict) and src.get("units") == "ns":
            return (ch, src)
    return (None, None)


def select_authoritative(uid: str, dos_dir: str):
    """Among a uid's JSON variants, pick the one with a DO-pointing source
    (chA-vs-Rb first).  Returns (chosen_path, data, channel, src, dropped).
    Falls back to a runtime-only JSON (no DO source) when none has one."""
    variants = [p for p in discover_variants(uid, dos_dir)
                if p.endswith(".json")]
    candidates = []
    for p in variants:
        d = _load_json(p)
        if d is None:
            continue
        ch, src = select_source(d)
        candidates.append((p, d, ch, src))
    if not candidates:
        return (None, None, None, None, variants)
    # Rank: has chA-vs-Rb (0) > has chA-chB (1) > runtime-only (2);
    # tiebreak on 'updated' (most recent first).
    def rank(c):
        ch = c[2]
        chrank = (_DO_CHANNELS.index(ch) if ch in _DO_CHANNELS else 99)
        return (chrank, c[1].get("updated", ""))
    candidates.sort(key=lambda c: (rank(c)[0], _neg_str(rank(c)[1])))
    chosen = candidates[0]
    dropped = [c[0] for c in candidates[1:]] + [
        p for p in discover_variants(uid, dos_dir) if not p.endswith(".json")]
    return (chosen[0], chosen[1], chosen[2], chosen[3], sorted(set(dropped)))


def _neg_str(s: str):
    # Sort timestamps descending (most recent first) within a rank tier.
    return tuple(-ord(c) for c in (s or ""))


# ── identity inference ──────────────────────────────────────────────── #

def infer_class(uid: str, data: dict, override):
    if override:
        return override
    hay = f"{uid} {data.get('label', '')}".lower()
    for cls in ("ocxo", "tcxo", "rb"):
        if cls in hay:
            return cls.upper() if cls != "rb" else "Rb"
    return None  # caller fails loud → operator supplies --class


def infer_actuator(data: dict, override):
    if override:
        return override
    if isinstance(data.get("dac_code_by_temperature"), dict):
        return "DAC"
    return None  # ambiguous → operator supplies --actuator


# ── builders ────────────────────────────────────────────────────────── #

def build_characterization(uid, data, channel, src, *, do_class, actuator,
                           model, nominal_freq_hz):
    """Return (toml_text, provenance dict).  Freerun is 'measured' only if a
    full (phase, freq, coast ref+slope) set can be assembled from src."""
    prov = {"identity": "measured"}
    lines = ['schema_version = "1"', "",
             "# Migrated from legacy state/dos JSON by migrate_do_state.py.",
             "", "[identity]",
             f"do_uid = {_q(uid)}",
             f"model = {_q(model or data.get('label') or 'unknown')}",
             f"class = {_q(do_class)}",
             f"actuator_type = {_q(actuator)}",
             f"nominal_freq_hz = {int(nominal_freq_hz)}"]
    if data.get("updated"):
        lines.append(f"registered = {_q(data['updated'])}")

    # [freerun_noise] — measured only if complete; else omit (loader → default)
    fr_ok = False
    if isinstance(src, dict):
        phase = src.get("asd_at_0.1Hz")
        sig_f, _ = _sigma_do_freq_ppb_from_adev(src.get("adev_by_tau_s"))
        tail = derive_coast_params(src.get("tdev_ns_by_tau_s"))
        missing = []
        if not (isinstance(phase, (int, float)) and phase > 0):
            missing.append("asd_at_0.1Hz (sigma_do_phase_ns)")
        if not (sig_f and sig_f > 0):
            missing.append("ADEV RWFM tail (sigma_do_freq_ppb)")
        if tail is None:
            missing.append("rising-tail TDEV (coast_tdev_ref/slope)")
        if not missing:
            ref, slope = tail
            lines += ["", "[freerun_noise]", 'source = "measured"',
                      f"measurement_channel = {_q(channel)}",
                      f"sigma_do_phase_ns = {_num(phase)}",
                      f"sigma_do_freq_ppb = {_num(sig_f)}",
                      f"coast_tdev_ref_ns = {_num(ref)}",
                      f"coast_tdev_slope = {_num(slope)}"]
            if data.get("updated"):
                lines.append(f"captured = {_q(data['updated'])}")
            prov["freerun_noise"] = f"measured ({channel})"
            fr_ok = True
        else:
            prov["freerun_noise"] = (
                "class-default (incomplete legacy data: "
                + ", ".join(missing) + ")")
    if not fr_ok and "freerun_noise" not in prov:
        prov["freerun_noise"] = "class-default (no DO-pointing source)"

    # [steering] — map legacy 'adjustment' if it carries a slope; else MISSING.
    adj = data.get("adjustment")
    slope_pc = None
    if isinstance(adj, dict):
        slope_pc = adj.get("slope_ppb_per_code") or adj.get("ppb_per_code")
    if slope_pc is not None:
        lines += ["", "[steering]", 'source = "measured"',
                  f"slope_ppb_per_code = {_num(slope_pc)}"]
        for k_src, k_dst in (("intercept_ppb_at_parked", "intercept_ppb_at_parked"),
                             ("parked_code", "parked_code"),
                             ("code_min", "code_min"), ("code_max", "code_max")):
            v = adj.get(k_src)
            if v is not None:
                lines.append(f"{k_dst} = {_num(v) if isinstance(v, float) else int(v)}")
        prov["steering"] = "measured"
    else:
        prov["steering"] = "MISSING (run do_steering_char.py)"

    prov["actuation_noise"] = "class-default (run do_actuator_char.py)"
    prov["aging"] = "absent"
    return ("\n".join(lines) + "\n", prov)


def build_sidecar(uid, channel, src) -> str:
    """Preserve the dense ADEV/TDEV/MDEV/PSD curves (engine never reads this)."""
    lines = ['schema_version = "1"', "",
             "# Dense freerun curves preserved from the legacy JSON.",
             "# Engine never reads this file; for replot / re-fit only.",
             f"do_uid = {_q(uid)}",
             f"measurement_channel = {_q(channel)}"]
    for sk in ("units", "noise_type"):
        if src.get(sk) is not None:
            lines.append(f"{sk} = {_q(src[sk])}")
    for sk in ("rms", "peak", "asd_at_0.1Hz", "asd_at_0.01Hz", "slope"):
        if isinstance(src.get(sk), (int, float)):
            lines.append(f"{sk} = {_num(src[sk])}")
    pc = src.get("psd_curve")
    if isinstance(pc, list) and pc:
        rows = ["[" + ", ".join(_num(x) for x in row) + "]"
                for row in pc
                if isinstance(row, (list, tuple))
                and all(isinstance(x, (int, float)) for x in row)]
        if rows:
            lines.append(f"psd_curve = [{', '.join(rows)}]")
    for tbl in ("adev_by_tau_s", "tdev_ns_by_tau_s", "mdev_by_tau_s"):
        t = src.get(tbl)
        if isinstance(t, dict) and t:
            lines += ["", f"[{tbl}]"]
            for k, v in t.items():
                try:
                    lines.append(f"{_q(k)} = {_num(v)}")
                except (TypeError, ValueError):
                    continue
    return "\n".join(lines) + "\n"


# ── per-DO migration ────────────────────────────────────────────────── #

def migrate_one(uid, *, dos_dir, do_class=None, actuator=None, model=None,
                nominal_freq_hz=10_000_000, apply=False):
    """Migrate one DO.  Returns a summary dict (provenance, paths, errors)."""
    chosen, data, channel, src, dropped = select_authoritative(uid, dos_dir)
    summary = {"uid": uid, "chosen": chosen, "channel": channel,
               "dropped": dropped, "errors": [], "provenance": {},
               "written": [], "deleted": []}
    if data is None:
        summary["errors"].append(f"no readable legacy JSON for {uid}")
        return summary

    cls = infer_class(uid, data, do_class)
    act = infer_actuator(data, actuator)
    if cls is None:
        summary["errors"].append(
            "could not infer [identity].class — pass --class OCXO|TCXO|Rb|PHC")
    if act is None:
        summary["errors"].append(
            "could not infer [identity].actuator_type — pass --actuator "
            "DAC|PHC_adjfine|ClockMatrix_FCW")
    if summary["errors"]:
        return summary

    char_text, prov = build_characterization(
        uid, data, channel, src, do_class=cls, actuator=act, model=model,
        nominal_freq_hz=nominal_freq_hz)
    summary["provenance"] = prov

    # Runtime state from legacy last_known_* + dac table.
    freq = data.get("last_known_freq_offset_ppb")
    table = data.get("dac_code_by_temperature")
    code = table.get("last") if isinstance(table, dict) else None
    rs = do_schema.RuntimeState(
        last_known_freq_offset_ppb=(None if freq is None else float(freq)),
        last_known_dac_code=(None if code is None else int(code)),
        last_updated=data.get("updated", ""))

    sidecar_text = build_sidecar(uid, channel, src) if src else None

    char_path = do_schema._char_path(uid, dos_dir)
    runtime_path = do_schema._runtime_path(uid, dos_dir)
    sidecar_path = os.path.join(dos_dir, f"{_safe_uid(uid)}.noise.toml")

    if not apply:
        summary["written"] = ["(dry-run) " + p for p in
                              [char_path, runtime_path]
                              + ([sidecar_path] if sidecar_text else [])]
        # Validate the would-be characterization in memory.
        _validate_text(char_text, uid, summary)
        return summary

    os.makedirs(dos_dir, exist_ok=True)
    _atomic_write(char_path, char_text)
    summary["written"].append(char_path)
    do_schema.save_runtime_state(uid, rs, dos_dir=dos_dir)
    summary["written"].append(runtime_path)
    if sidecar_text:
        _atomic_write(sidecar_path, sidecar_text)
        summary["written"].append(sidecar_path)

    # Validate via the real loader before deleting anything.
    try:
        do_schema.load_do_characterization(uid, dos_dir=dos_dir)
        do_schema.load_runtime_state(uid, dos_dir=dos_dir)
    except do_schema.SchemaError as e:
        summary["errors"].append(f"post-write validation FAILED: {e}")
        return summary

    for p in dropped + ([chosen] if chosen else []):
        try:
            os.unlink(p)
            summary["deleted"].append(p)
        except OSError as e:
            summary["errors"].append(f"could not delete {p}: {e}")
    return summary


def _validate_text(char_text, uid, summary):
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        with open(os.path.join(td, f"{_safe_uid(uid)}.toml"), "w") as f:
            f.write(char_text)
        try:
            do_schema.load_do_characterization(uid, dos_dir=td)
        except do_schema.SchemaError as e:
            summary["errors"].append(f"would FAIL validation: {e}")


def _atomic_write(path, text):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, path)


# ── CLI ─────────────────────────────────────────────────────────────── #

def _print_summary(s):
    print(f"\n=== {s['uid']} ===")
    if s["chosen"]:
        print(f"  authoritative: {os.path.basename(s['chosen'])}"
              + (f"  (channel: {s['channel']})" if s["channel"] else
                 "  (no DO-pointing source)"))
    for d in s["dropped"]:
        print(f"  dropped:       {os.path.basename(d)}")
    for sec, src in s.get("provenance", {}).items():
        print(f"    [{sec}]: {src}")
    for w in s["written"]:
        print(f"  wrote:   {w}")
    for d in s["deleted"]:
        print(f"  deleted: {d}")
    for e in s["errors"]:
        print(f"  ERROR:   {e}")


def discover_uids(dos_dir):
    uids = set()
    for p in glob.glob(os.path.join(dos_dir, "*.json")):
        b = os.path.basename(p)[:-5]
        for suf in ("_noise",):
            if b.endswith(suf):
                b = b[: -len(suf)]
        b = b.split("-v")[0] if "-v" in b else b
        uids.add(b)
    return sorted(uids)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("uid", nargs="*",
                    help="DO uid(s) to migrate; default = all in --dos-dir")
    ap.add_argument("--dos-dir", default=do_schema.DEFAULT_DOS_DIR)
    ap.add_argument("--class", dest="do_class",
                    choices=list(do_schema.ALLOWED_CLASSES))
    ap.add_argument("--actuator", choices=list(do_schema.ALLOWED_ACTUATOR_TYPES))
    ap.add_argument("--model")
    ap.add_argument("--nominal-freq", type=int, default=10_000_000)
    ap.add_argument("--apply", action="store_true",
                    help="write the new files + delete the old JSON "
                         "(default: dry-run preview)")
    args = ap.parse_args(argv)

    uids = args.uid or discover_uids(args.dos_dir)
    if not uids:
        print(f"No DO state JSON found in {args.dos_dir}")
        return 0
    if not args.apply:
        print("DRY RUN — no files written or deleted (pass --apply to commit)")

    any_err = False
    for uid in uids:
        s = migrate_one(uid, dos_dir=args.dos_dir, do_class=args.do_class,
                        actuator=args.actuator, model=args.model,
                        nominal_freq_hz=args.nominal_freq, apply=args.apply)
        _print_summary(s)
        any_err = any_err or bool(s["errors"])
    return 1 if any_err else 0


if __name__ == "__main__":
    raise SystemExit(main())

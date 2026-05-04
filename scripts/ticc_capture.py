#!/usr/bin/env python3
"""ticc_capture.py — standalone TICC logger for cross-host PPS comparison.

Reads a TAPR TICC time-interval counter and writes one CSV row per edge.
Independent of peppar-fix — opens its own serial device, writes its own
CSV — so the host running this can also run peppar-fix on a *different*
TICC.  Used for cross-host PPS-OUT agreement measurement (chA = host A
PPS OUT, chB = host B PPS OUT, shared TICC reference).

CSV schema matches peppar-fix engine TICC logs so existing analysis
tools (pos_adev.py, tdev_overlay.py, etc.) work unchanged:

    ts_iso, channel, ref_sec, ref_ps, recv_mono

Daily UTC-midnight rotation: out_dir/{prefix}-{YYYY-MM-DD}.csv.

Reuses scripts/ticc.Ticc, which handles the HUPCL fix that prevents
Arduino reboot on serial open/close (see CLAUDE.md "TICC resets on
serial open").
"""
from __future__ import annotations

import argparse
import csv
import signal
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPT_DIR))
sys.path.insert(0, str(_SCRIPT_DIR / "peppar_fix"))

from ticc import Ticc  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--device", default="/dev/ticc4",
                    help="TICC serial device (default: /dev/ticc4)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out-dir", default="data",
                    help="CSV output directory (default: ./data)")
    ap.add_argument("--prefix", default="ticc4-capture",
                    help="CSV filename prefix (default: ticc4-capture)")
    args = ap.parse_args()

    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    sig = {"caught": False}

    def _handler(signum, frame):
        sig["caught"] = True
    signal.signal(signal.SIGTERM, _handler)
    signal.signal(signal.SIGINT, _handler)

    print(f"ticc_capture: opening {args.device} @ {args.baud}",
          file=sys.stderr, flush=True)
    print(f"ticc_capture: writing {out_dir}/{args.prefix}-YYYY-MM-DD.csv",
          file=sys.stderr, flush=True)

    current_date: str | None = None
    f = None
    w = None
    n_total = 0
    n_chA = 0
    n_chB = 0
    last_status = time.monotonic()

    try:
        with Ticc(args.device, baud=args.baud, wait_for_boot=True) as ticc:
            print("ticc_capture: TICC booted, streaming",
                  file=sys.stderr, flush=True)
            for channel, ref_sec, ref_ps in ticc:
                if sig["caught"]:
                    break
                recv_mono = time.monotonic()
                now_utc = datetime.now(timezone.utc)
                day = now_utc.strftime("%Y-%m-%d")
                if day != current_date:
                    if f is not None:
                        f.close()
                    path = out_dir / f"{args.prefix}-{day}.csv"
                    new_file = not path.exists()
                    f = open(path, "a", newline="", buffering=1)
                    w = csv.writer(f)
                    if new_file:
                        w.writerow(["ts_iso", "channel",
                                    "ref_sec", "ref_ps", "recv_mono"])
                    current_date = day
                    print(f"ticc_capture: writing to {path}",
                          file=sys.stderr, flush=True)
                ts_iso = now_utc.strftime("%Y-%m-%dT%H:%M:%S.%fZ")
                w.writerow([ts_iso, channel,
                            ref_sec, ref_ps, f"{recv_mono:.6f}"])
                n_total += 1
                if channel == "chA":
                    n_chA += 1
                elif channel == "chB":
                    n_chB += 1
                # Periodic status to stderr so screen/journalctl shows life.
                if recv_mono - last_status > 60.0:
                    print(f"ticc_capture: n={n_total} "
                          f"chA={n_chA} chB={n_chB}",
                          file=sys.stderr, flush=True)
                    last_status = recv_mono
    except KeyboardInterrupt:
        pass
    finally:
        if f is not None:
            f.close()

    print(f"ticc_capture: done.  total={n_total} chA={n_chA} chB={n_chB}",
          file=sys.stderr, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())

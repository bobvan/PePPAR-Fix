#!/usr/bin/env python3
"""
Test SSR decoder against recorded CLK93 RTCM3 data from rtkexplorer.

Reads a raw RTCM3 binary file, decodes each frame with pyrtcm, and feeds
the decoded messages into SSRState to verify the correction pipeline.

Usage:
    python3 scripts/test_ssr_decode.py data/f9t_ssr/clk93_0000.rtcm3
"""

import sys
import time
from collections import defaultdict
from pathlib import Path

from pyrtcm import RTCMReader

# Add scripts/ to path for local imports
sys.path.insert(0, str(Path(__file__).parent))
from ssr_corrections import SSRState


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <rtcm3_file>")
        sys.exit(1)

    rtcm_file = sys.argv[1]
    ssr = SSRState()

    # Disable staleness checks for offline replay
    # (rx_time is set to now() but data is from the past)
    import ssr_corrections
    ssr_corrections.MAX_ORBIT_AGE = 1e9
    ssr_corrections.MAX_CLOCK_AGE = 1e9
    ssr_corrections.MAX_BIAS_AGE = 1e9

    msg_counts = defaultdict(int)
    ssr_updates = defaultdict(int)
    decode_errors = 0
    total_frames = 0
    first_epoch = None
    last_epoch = None

    print(f"Reading {rtcm_file}...")
    t0 = time.monotonic()

    with open(rtcm_file, 'rb') as f:
        reader = RTCMReader(f)
        for raw, parsed in reader:
            total_frames += 1

            if parsed is None:
                decode_errors += 1
                continue

            identity = str(parsed.identity)
            msg_counts[identity] += 1

            # Try feeding to SSR state
            result = ssr.update_from_rtcm(parsed)
            if result:
                ssr_updates[result] += 1

                # Track epochs
                epoch_s = getattr(parsed, 'DF385', None)
                if epoch_s is not None:
                    if first_epoch is None:
                        first_epoch = epoch_s
                    last_epoch = epoch_s

            # Progress every 10000 frames
            if total_frames % 10000 == 0:
                print(f"  {total_frames:,} frames... {ssr.summary()}")

    elapsed = time.monotonic() - t0

    print(f"\n{'='*70}")
    print(f"Decoding complete: {total_frames:,} frames in {elapsed:.1f}s "
          f"({total_frames/elapsed:,.0f} frames/s)")
    print(f"Decode errors: {decode_errors}")

    if first_epoch is not None and last_epoch is not None:
        duration_s = last_epoch - first_epoch
        if duration_s < 0:
            duration_s += 604800  # wrap around GPS week
        print(f"Data span: {duration_s:.0f}s ({duration_s/3600:.1f}h)")

    print(f"\nRTCM message types decoded:")
    for mt in sorted(msg_counts, key=lambda x: int(x) if x.isdigit() else 0):
        print(f"  {mt:>6}: {msg_counts[mt]:>6,}")

    print(f"\nSSR updates by type:")
    for st in sorted(ssr_updates):
        print(f"  {st:>15}: {ssr_updates[st]:>6,}")

    print(f"\nFinal SSR state:")
    print(f"  {ssr.summary()}")

    # Show sample corrections
    print(f"\nSample orbit corrections:")
    shown = 0
    for prn in sorted(ssr._orbit.keys()):
        oc = ssr._orbit[prn]
        print(f"  {prn}: radial={oc.radial:+.4f}m  along={oc.along:+.4f}m  "
              f"cross={oc.cross:+.4f}m  IOD={oc.iod}")
        shown += 1
        if shown >= 5:
            print(f"  ... ({len(ssr._orbit) - shown} more)")
            break

    print(f"\nSample clock corrections:")
    shown = 0
    for prn in sorted(ssr._clock.keys()):
        cc = ssr._clock[prn]
        print(f"  {prn}: c0={cc.c0:+.4f}m  ({cc.c0/299792458*1e9:+.2f} ns)")
        shown += 1
        if shown >= 5:
            print(f"  ... ({len(ssr._clock) - shown} more)")
            break

    print(f"\nSample code biases:")
    shown = 0
    for prn in sorted(ssr._code_bias.keys()):
        biases = ssr._code_bias[prn]
        bias_str = ", ".join(f"{k}={v.bias_m:+.3f}m" for k, v in sorted(biases.items()))
        print(f"  {prn}: {bias_str}")
        shown += 1
        if shown >= 5:
            print(f"  ... ({len(ssr._code_bias) - shown} more)")
            break

    print(f"\nSample phase biases:")
    shown = 0
    for prn in sorted(ssr._phase_bias.keys()):
        biases = ssr._phase_bias[prn]
        bias_str = ", ".join(f"{k}={v.bias_m:+.3f}m" for k, v in sorted(biases.items()))
        print(f"  {prn}: {bias_str}")
        shown += 1
        if shown >= 5:
            print(f"  ... ({len(ssr._phase_bias) - shown} more)")
            break


if __name__ == "__main__":
    main()

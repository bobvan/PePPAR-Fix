# rx_tcxo frequency injection via UBX-MGA-INI-FREQ

## Problem

When an F9T or F9P receiver cold-starts (post power cycle, not a
mere engine restart), its reference-oscillator frequency offset is
unknown to the receiver.  TTFF is dominated by the Doppler search
this uncertainty forces:

- GPS L1 carrier: 1575.42 MHz
- TCXO spec on ZED-F9T: ±0.5 ppm initial, up to ±2.5 ppm over
  temp/aging; ZED-F9P similar
- At ±2.5 ppm of L1, the receiver must search ±3.9 kHz of extra
  Doppler per SV beyond the orbit-predicted Doppler
- Typical Doppler bin is ~500 Hz, so ~16 extra bins per SV per
  stride of the acquisition search

PePPAR-Fix's `RxTcxoTracker` already measures exactly this quantity
(TCXO frequency offset in ppb) continuously during normal
operation.  On every shutdown we throw it away; on every restart
the receiver re-estimates from scratch.  This wastes ~tens of
seconds of TTFF on cold start, or a second or two on warm start.

## The message

**UBX-MGA-INI-FREQ** — class `0x13`, ID `0x40`, version `0x04`:

```
u1   type          0x04   (frequency assistance subtype)
u1   version       0x00
x2   reserved
x4   flags         bit 0: autonomous (our source, not aiding)
i4   freq          scaled ppb, signed.  Check scale vs protocol
                   version (historically 2^-8 ppb per LSB).
u4   freqAcc       accuracy, 2^-8 ppb per LSB, one-sigma.  Scales
                   the receiver's confidence in our prior — too
                   tight → rejected if slightly wrong; too loose →
                   doesn't meaningfully narrow search.
```

Siblings of interest:
- **UBX-MGA-INI-CLKD** (subtype `0x05`) — clock drift rate.
  Different semantic: the rate of change of the receiver clock
  offset vs its nominal 1PPS, in ns/s.  Related but orthogonal —
  CLKD models the clock's *future* drift during the gap between
  power-on and first fix.
- **UBX-UPD-SOS** — save-on-shutdown, broader mechanism that
  preserves the whole receiver state (ephemeris, tracking, TCXO
  estimate) to flash.  Different tradeoff: wider scope but more
  brittle shutdown path.

MGA-INI-FREQ is the minimal fit for what PePPAR-Fix already knows.

## Fit with PePPAR-Fix

`RxTcxoTracker` produces the estimate at the correct quantity and
unit (frequency offset in ppb).  State-persistence infrastructure
already exists at `state/receivers/<uid>.json` (maintained by
`peppar_fix/receiver_state.py`).  The extension is additive:

```json
{
  "unique_id": 675836739647,
  "last_known_position": { ... },
  "rx_tcxo": {
    "ppb": +12345.6,
    "ppb_sigma": 5.0,
    "updated": "2026-04-22T15:34:00Z",
    "source": "RxTcxoTracker"
  }
}
```

On shutdown: write the latest tracker estimate to the receiver
state file, alongside the position block.  Simple, atomic, already
protected by the existing persistence path.

On startup, after config-apply but before observation loop starts:

```
if rx_tcxo is recent (last 24h?) and sigma reasonable:
    send UBX-MGA-INI-FREQ with freq=ppb, freqAcc=sigma
```

The freshness gate matters because TCXO frequency does drift with
temperature over hours.  A 24-hour-old estimate off a machine at
an extreme temperature delta is worse than no prior at all.

## Where this pays off

| Start type       | Receiver state             | Benefit     |
|------------------|----------------------------|-------------|
| Cold start       | No TCXO estimate           | **Large**   |
| Warm start       | Ephemeris valid, no TCXO   | Moderate    |
| Hot start        | Still tracking             | None        |

The typical PePPAR-Fix "engine restart" is a hot-start scenario —
the F9 stays powered, keeps its TCXO estimate across the UBX
disconnect.  Zero benefit there.

The real targets are:

- Full lab reboot / power cycle
- Swapping receivers between hosts (F9 picked up with no powered
  history at the new host)
- USB-reset events that take the F9 through a reset cycle
- Deploying to a fresh install where the on-receiver memory has
  never seen this lab environment

In those cases, cold-start TTFF has a real wall-clock cost
(operator waits, servo stays idle, position converges from
scratch).  Shaving seconds-to-minutes there is the payoff.

## Implementation sketch

Not starting soon; below is what the design would look like.

### State persistence

Extend `peppar_fix/receiver_state.py` with:

```python
@dataclass
class RxTcxoFreqSnapshot:
    ppb: float
    ppb_sigma: float
    updated: datetime
    source: str = "RxTcxoTracker"

def save_rx_tcxo_freq(uid: int, snapshot: RxTcxoFreqSnapshot) -> None:
    ...

def load_rx_tcxo_freq(uid: int, max_age_hours: float = 24.0
                     ) -> Optional[RxTcxoFreqSnapshot]:
    ...
```

The existing receiver-state JSON gets a new top-level `rx_tcxo`
block.  Load with freshness gate; don't inject a prior older than
the max age.

### Engine integration points

Two places in `peppar_fix_engine.py`:

1. **Shutdown path**: after the final RxTcxoTracker update, call
   `save_rx_tcxo_freq` with the current estimate.  Piggyback on
   the existing signal handler or `try/finally` around the main
   loop.

2. **Startup path** (post config-apply, pre observation loop):

   ```python
   snap = load_rx_tcxo_freq(uid, max_age_hours=24.0)
   if snap is not None and snap.ppb_sigma < MAX_INJECT_SIGMA_PPB:
       _send_mga_ini_freq(ser, snap.ppb, snap.ppb_sigma)
       log.info("MGA-INI-FREQ sent: ppb=%+.1f ± %.1f",
                snap.ppb, snap.ppb_sigma)
   ```

`_send_mga_ini_freq` is a new helper in
`peppar_fix/receiver.py` (or wherever UBX-send lives).  Packs the
payload per the UBX spec.

### Where the message actually goes

The F9 accepts UBX-MGA-INI-* on any UART or USB interface.  In
our setup that's the same serial port the engine already uses
for configuration.  Timing: must happen **before** the receiver
starts locking; ideally in the same serial conversation as the
initial config-apply, so the receiver absorbs the prior before
its tracking loops spin up.

## Gotchas

### Sign convention

u-blox historically has had sign-convention subtleties across
the "Doppler" vs "oscillator drift" domains.  The spec says
`freq` is the **oscillator offset**: `freq = (f_actual - f_nominal)
/ f_nominal` in ppb.  Positive = fast (higher frequency than
nominal).

`RxTcxoTracker`'s convention must be verified to match.  The
PePPAR-Fix sign convention tends to follow "receiver clock error"
(positive = receiver clock ahead of GPS), which may or may not
flip when converted to oscillator frequency offset.  Validate
before trusting in production.

### Scaling factor

The `freq` field is scaled ppb.  The scaling factor has been
`2^-8 ppb per LSB` historically, but different protocol versions
have different units.  Check against the F9T TIM 2.25 / F9P
HPG 1.50 manual for the current F9 generation before trusting
the encoding.

### Does F9 honor MGA-INI on warm start?

Some u-blox receivers only honor MGA-INI-* messages when the
receiver is in a cold-enough state (no internal estimate to
preserve).  If the F9 ignores the prior because it has a recent
internal TCXO estimate from the last few seconds of tracking,
the injection is a no-op.

Testing this experimentally is the only way to know — vendor
documentation is typically vague on which start states honor
MGA-INI.  Use the TIM-TP sawtooth behavior or ACQM monitor
messages to verify the receiver's acquisition search actually
narrowed.

### Freshness

A TCXO estimate is valid for hours at stable temperature, less
at large thermal deltas.  The F9T/F9P crystal moves ~0.01 ppm
per °C typical; a 30°C lab vs garage delta means ~0.3 ppm drift
— still within the ±5 kHz Doppler search window, still useful,
but the `freqAcc` should be widened to match.

A simple heuristic:

```
max_age_hours = 24
baseline_acc_ppb = max(snap.ppb_sigma, 50)  # ~0.05 ppm floor
age_hours = (now - snap.updated).total_seconds() / 3600
injected_acc_ppb = baseline_acc_ppb + 20 * age_hours
                   # widen ~0.02 ppm per hour stale
```

Tune from data, not first principles.

## Experimental validation

Before enabling in production, one afternoon's data-collection:

1. Deep power-cycle a lab F9 (unplug USB, wait 30 s, re-plug).
   Measure TTFF to first 3D fix without any assistance.  Baseline.
2. Repeat with position + time + ephemeris assistance only (our
   existing receiver_state + broadcast_eph cache).
3. Repeat with position + time + ephemeris + MGA-INI-FREQ from
   the persisted TCXO value.

Delta between 2 and 3 is the answer.  Reasonable expected values:

- Step 1 TTFF: 30-60 s (classic cold start with aiding from
  almanac on F9)
- Step 2 TTFF: 5-15 s (warm start, no frequency prior)
- Step 3 TTFF: 3-10 s (warm start + frequency prior)

The exact delta depends on the TCXO's actual offset from nominal
— a host with a TCXO running at +8 ppm benefits more than one
at +0.5 ppm.  TimeHat's older F9T-10 hardware would be the best
subject for the large-delta test.

Repeat across: ptpmon (F9T + E810), TimeHat (older F9T-10),
MadHat (F9T-20B), clkPoC3 (F9T-20B).  Per-host TTFF distribution
gives us both the realistic improvement and the variance.

## Non-goals

- Not UBX-UPD-SOS (broader scope; different tradeoff; separate
  design).
- Not UBX-MGA-INI-TIME or UBX-MGA-INI-POS — those mechanisms are
  already covered by the receiver_state position cache and the
  system-time-at-startup that the F9 absorbs by default.
- Not per-SV ephemeris assistance (UBX-MGA-GPS-EPH etc.) —
  separate mechanism, not connected to TCXO modeling.
- Not a cross-host TCXO prior sharing mechanism.  Each receiver
  has its own TCXO; estimates don't transfer.

## Open questions for when this is picked up

1. **Integration point**: does MGA-INI-FREQ need to be sent
   before or after UBX-CFG-VALSET on startup?  Probably before
   (receiver needs the prior during acquisition, not after
   steady-state).
2. **Does the F9 accept it on USB-CDC or only on the raw UART
   pins?**  Check the protocol-version-specific notes.
3. **What does UBX-ACQM look like with vs without the prior?**
   We could instrument this for the experimental validation
   step — gives a direct signal that the Doppler search actually
   narrowed, independent of TTFF.
4. **Is CLKD worth sending alongside FREQ?**  Probably only for
   very stale snapshots where the frequency has likely drifted
   further — then CLKD helps the receiver extrapolate.  For
   fresh (< 1 h) snapshots, FREQ alone is enough.

## Estimated size

~100 lines of new code:
- ~30 lines in `receiver_state.py` (save/load, dataclass)
- ~40 lines in a new `mga_ini.py` helper (UBX packing, send)
- ~20 lines in `peppar_fix_engine.py` (call sites + config gate)
- ~10 lines of tests

Plus ~a few hours of experimental validation across the fleet.
Straightforward when we pick it up.

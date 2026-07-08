# The GNSS receiver clock hierarchy

Not all GNSS receivers relate to *time* the same way, and for PePPAR-Fix — whose whole
job is to discipline an oscillator against GNSS time — a receiver's **clock
architecture** matters more than almost any other spec. There's an unwritten hierarchy.

## Tier 1 — PPS only

The bottom tier has some internal oscillator and a single **1 PPS output**. It knows
what time it is and emits a pulse to prove it — and that's all. Typical of the
unbranded, inexpensive GNSS modules. To discipline an external oscillator you have only
the PPS edge: quantized to nanoseconds, one sample per second.

## Tier 2 — good internal oscillator + raw observations

The next tier keeps the PPS and adds three things:

- a good-quality internal **TCXO** for better short-term stability;
- the ability to **timestamp an external event** against the receiver's own timescale
  (an EXTINT / time-mark input); and
- output of the **raw GNSS code and carrier-phase observations**.

The u-blox ZED-F9T is the stalwart example. The raw carrier phase is what makes
picosecond-class clock estimation possible — but it is measured against the receiver's
**internal TCXO**, so every observation carries that TCXO's noise. (This is the "RX
TCXO floor" in the main README's two-oscillator budget.)

## Tier 3 — geodetic, with an external clock input

The top tier — "geodetic" receivers — has everything above **plus an optional external
clock input**. Fed an external reference, the receiver takes its code and carrier-phase
observations directly against *that* oscillator. The receiver's clock estimate then
**is** the external oscillator's error against GNSS time: a direct error signal for
disciplining it.

That is the crucial difference. A Tier-1 or Tier-2 receiver outputs a PPS aligned to
its **internal** oscillator, so disciplining an external oscillator from that PPS is a
**second hop** —

```
GNSS time ──▶ receiver's internal-TCXO-timed PPS ──▶ external disciplined oscillator
```

— and every hop adds noise and quantization. A geodetic receiver on an external clock
removes the middle hop entirely: the measurement and the disciplined oscillator are the
**same** clock.

### What PePPAR-Fix does with a geodetic receiver

A geodetic receiver still ships with its own internal oscillator. PePPAR-Fix explicitly
**disables** it and substitutes an external oscillator it can steer, so the same clock
that makes the carrier-phase measurements is the one being disciplined.

Worked example — the **SparkFun GNSSDO+** contains both a Septentrio **Mosaic-T** and a
**Rakon OCXO**. The Mosaic-T is configured to use the OCXO as its external clock through
its external-clock input; PePPAR-Fix disciplines that OCXO. The carrier phase is thus
measured against the very oscillator we steer — the **single-oscillator** case in the
main README, where the receiver-clock and disciplined-oscillator floors collapse into
one.

This is the architecture that lets carrier phase observe the disciplined oscillator
*directly*, at the full observation rate, with no PPS/TICC hop in between — the cleanest
error signal available to the servo.

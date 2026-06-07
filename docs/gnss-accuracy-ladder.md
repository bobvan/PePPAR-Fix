# How Long You Watch the Sky Determines How Well You Know Where and When You Are

## The basic idea

GPS satellites broadcast their positions and the time.  Your receiver
watches these signals and works out where you are and what time it is.
The longer and more carefully you watch, and the more help you get from
others who are also watching, the better your answer gets.

This document walks through the accuracy you can expect at each level
of effort, from a few seconds on your phone to a two-week wait for the
best answer science can deliver.

We assume a **warm start**: the receiver already knows the satellites'
orbital details (ephemerides) from a recent session or from the
internet.  It doesn't know where it is — you've moved since last time.

## The picture at a glance

![GNSS positioning regimes: observation time vs position+time accuracy](gnss-accuracy-blobs.png)

Each blob is one method's working regime in the
(observation-time × position-accuracy) plane.  Blobs overlap on
purpose — the methods are continuous tradeoffs, not crisp tiers.  The
secondary y-axis on the right shows the c-equivalent timing accuracy
(1 m ≈ 3.33 ns); method-specific timing accuracy is annotated inside
each blob, because some methods (the F9T-class timing receiver in
particular) do better on time than the c-equivalent of their position
would suggest.  The shaded band under 10 cm marks the regime that
requires real-time corrections from the internet — receiver-only
operation with broadcast ephemeris tops out around there.

The levels below walk through each regime in detail.


## Level 0: Your phone (seconds, meters)

You switch on your phone in a new city.  Within 2-5 seconds, a blue
dot appears on the map.  You're somewhere in the right block.

**Position**: 2-5 meters.  Good enough to navigate on foot or by car.

**Time**: ~100 nanoseconds.  Your phone doesn't tell you this, but its
internal clock is now set to GPS time within about a tenth of a
microsecond.  Good enough for every consumer application.

**How it works**: The receiver measures the travel time of each
satellite's signal — how long the radio wave took to get from the
satellite to you.  Multiply by the speed of light and you have a
distance.  Four or more distances from satellites in different parts
of the sky pin down your position and clock offset.

**What limits it**: The travel-time measurement is coarse — it's based
on matching a pattern in the signal that repeats roughly every
microsecond (about 300 meters of radio travel).  The receiver can
interpolate within that pattern, but the fundamental granularity is
about a meter.  Signals bouncing off buildings (multipath) and
bending through the atmosphere add more error.


## Level 1: A surveyor's receiver, no outside help (minutes to hours, decimeters)

A better receiver can extract finer detail from the same signals.
Instead of just matching the coarse pattern, it tracks the underlying
wave — counting the individual oscillations of the radio signal, which
repeat billions of times per second.  Each oscillation is about 19
centimeters long (for the primary GPS frequency), so the granularity
drops from meters to millimeters.

The catch: the receiver doesn't know *which* oscillation it locked onto.
It knows it's some whole number of 19 cm wavelengths away from the
satellite, but not which whole number.  This is the **integer
ambiguity** — the receiver must figure out whether it's 100,000,000
wavelengths away or 100,000,001.

With a single receiver and no outside help, this ambiguity takes time
to resolve.  The receiver watches the satellites move across the sky
and uses the changing geometry to narrow down the answer.

**After a few minutes** of watching: position ~30 cm, time ~10 ns.
The ambiguities are still floating (not pinned to exact integers),
but the geometry has improved enough to constrain the answer to
sub-meter.

**After an hour**: position ~5-10 cm, time ~2-5 ns.  The ambiguities
are converging.  Watching on two frequencies (the satellites broadcast
on multiple radio channels) helps the receiver separate atmospheric
bending from true distance, which is the main remaining error.

**After 24 hours**: position ~2-3 cm, time ~1-2 ns.  The satellites
have completed a full lap of the sky, giving maximum geometric
diversity.  But you're still limited by the accuracy of the satellite
positions and clocks as broadcast — the satellites know where they are
to about a meter, and the clocks they broadcast are good to about a
nanosecond but not perfect.


## Level 2: Help from a nearby watcher (seconds to minutes, centimeters)

Suppose someone a few kilometers away is also watching the same
satellites, and they already know exactly where they are (a
permanently installed reference station).  They can tell you: "From
where I'm standing, satellite #12 appears to be 3 centimeters farther
away than its broadcast position claims."

This is what NTRIP correction services provide.  A network of fixed
reference stations continuously watches the sky and streams their
measurements over the internet.  Your receiver downloads these
corrections and uses them to cancel most of the errors that are
common to both of you — atmospheric bending, satellite position
errors, satellite clock errors.

Because you and the reference station are close together (under
~30 km), you're looking through nearly the same patch of atmosphere,
so the atmospheric errors cancel almost perfectly.

**Position**: 1-2 cm within minutes.  The integer ambiguities resolve
quickly because the corrections remove most of the error, making the
remaining ambiguities obvious.

**Time**: ~1 ns.

**What limits it**: You need a reference station nearby, and you need
an internet connection to receive its corrections.  The corrections
are only as good as the reference station's known position.  In
remote areas, there may be no nearby station.


## Level 3: Help from many watchers worldwide — combined corrections (minutes to hours, centimeters)

Instead of one nearby reference station, what if hundreds of stations
around the world pool their measurements?  Analysis centers (academic
and government institutions) combine this global data to compute very
precise satellite orbits and clocks — far better than what the
satellites broadcast about themselves.

These corrections are streamed over the internet in real time (with
a few seconds of delay).  Your receiver applies them to improve its
own solution.  This is called Precise Point Positioning (PPP).

The key difference from Level 2: you don't need a nearby reference
station.  The corrections describe the satellites themselves, not
your local atmosphere.  You can be anywhere on Earth with an internet
connection.

The tradeoff: without local atmospheric cancellation, your receiver
must estimate the atmosphere on its own.  This takes longer — the
ambiguities converge over 20-40 minutes instead of seconds.

When multiple analysis centers contribute, their corrections are
averaged.  This averaging is good for reliability but introduces
small inconsistencies between centers — each center makes slightly
different modeling choices, and when you average their results, the
integer nature of the ambiguities gets smeared.  The position
converges, but you can't cleanly resolve the ambiguities to exact
integers.

**Position**: 5-10 cm after 20 minutes, improving to 2-3 cm after
an hour.  The ambiguities remain "floating" (fractional estimates,
never pinned to integers).

**Time**: 2-5 ns after convergence.


## Level 4: Help from one careful watcher — single analysis center (minutes, centimeters)

If instead of averaging corrections from many analysis centers, you
use corrections from a single center that has been internally
consistent in all its modeling choices, something powerful happens:
the small inconsistencies from averaging go away.  The corrections
preserve the integer nature of the ambiguities.

Your receiver can now resolve the ambiguities to exact integers —
the difference between "I'm about 100,000,000.3 wavelengths away"
and "I'm exactly 100,000,000 wavelengths away."  Pinning that 0.3
to exactly 0 removes several centimeters of uncertainty in one step.

This is PPP with ambiguity resolution (PPP-AR).  It requires:
- Corrections from a single analysis center (not a blend)
- Both frequency and phase corrections (not just orbit and clock)
- A receiver tracking on two frequencies

**Position**: 1-2 cm after 5-15 minutes.  The wide-lane ambiguities
(formed from the difference of two frequencies, creating a long
86 cm virtual wavelength) resolve quickly — within a minute or two.
The narrow-lane ambiguities (formed from the sum, creating a short
11 cm virtual wavelength) take longer but still converge faster than
float PPP because the wide-lane constraints help.

**Time**: sub-nanosecond after convergence.  Once the ambiguities
are integers, the receiver clock estimate snaps to a much tighter
value — the same way that knowing you're *exactly* 100,000,000
wavelengths away is more informative than knowing you're *about*
100,000,000.3.

**What limits it**: You need corrections from a single analysis
center that provides phase corrections — not all do.  You need a
dual-frequency receiver.  And you still need to wait for the
ambiguities to converge, though it's faster than float PPP.


## Level 5: Watch for two days, wait two weeks (centimeters, sub-nanosecond)

The corrections available in real time are predictions — analysis
centers estimate satellite orbits and clocks based on recent data and
extrapolate forward.  These "rapid" products are good, but not as
good as what you can compute after the fact.

If you record 48 hours of raw measurements from a fixed point and
then wait two weeks, the International GNSS Service (IGS) publishes
"final" products: satellite orbits known to 1-2 cm and clocks known
to ~0.1 ns, computed by combining data from hundreds of stations
worldwide with full hindsight.

Post-processing your 48-hour recording against these final products
gives the best answer available:

**Position**: 3-5 mm horizontal, ~1 cm vertical.

**Time**: ~0.1 ns (100 picoseconds).

Nobody needs this for navigation.  It's for geodesy (measuring how
continents move), timing (synchronizing clocks across continents),
and calibration (establishing truth for everything else to be
compared against).


## Summary

| Level | Outside help | Watch time | Position | Time | When you know |
|---|---|---|---|---|---|
| 0. Phone | None | 2-5 s | 2-5 m | ~100 ns | Immediately |
| 1. Standalone dual-freq | None | 1-24 h | 3-30 cm | 1-10 ns | Immediately |
| 2. Local reference (NTRIP RTK) | Nearby station | 1-5 min | 1-2 cm | ~1 ns | Immediately |
| 3. Global PPP (multi-AC) | Worldwide network | 20-60 min | 2-10 cm | 2-5 ns | Immediately |
| 4. PPP-AR (single AC) | Single analysis center | 5-15 min | 1-2 cm | <1 ns | Immediately |
| 5. Post-processed (IGS final) | 48h recording + 2 week wait | 48 h | 3-5 mm | ~0.1 ns | 2 weeks later |


## What PePPAR Fix does

PePPAR Fix operates at **Level 3-4** in real time: dual-frequency
receiver, real-time SSR corrections from NTRIP, carrier-phase PPP
with optional ambiguity resolution.  The goal is to extract the best
possible time estimate — sub-nanosecond — and use it to discipline a
local oscillator so that the oscillator's output (PPS or PTP) tracks
GPS time faithfully.

The position is a means to an end: you need to know where you are
to compute how far away each satellite is, so you can isolate the
clock offset.  A 3 cm position error contributes ~0.1 ns of time
error (3 cm / speed of light).  This is why we care about position
accuracy even though we're building a clock, not a map.

## TODO

- [ ] Plot: position accuracy (y, log scale) vs observation time
  (x, log scale), one curve per level.  Overlay time accuracy on
  a secondary y-axis.
- [ ] Add confidence intervals / shading to show the range of
  outcomes (good sky vs obstructed, good multipath vs urban canyon).

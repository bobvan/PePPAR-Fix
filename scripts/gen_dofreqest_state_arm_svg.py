#!/usr/bin/env python3
"""Generate docs/dofreqest-state-arm-map.svg.

Renders DOFreqEst (scripts/peppar_fix/do_freq_est.py) as a state×arm
incidence diagram:
  - 4 state nodes (rows), color-coded measured vs hidden
  - 6 measurement arms entering from the left at the state(s) they observe
  - the predict-step dynamics coupling (f→φ integrators) as internal arrows
  - the actuator feedback (adjfine → φ_do, derived from the hidden f_do)

The point of the picture: make "x3 (f_do) is hidden but recoverable via the
f_do→φ_do integrator" visually obvious, and show the one non-diagonal arm
(TICC couples x0+x2).  The same incidence table lives as a block comment at
the top of do_freq_est.py — keep the two in sync if the arms change.

Run from anywhere:
    scripts/gen_dofreqest_state_arm_svg.py
"""
from pathlib import Path

# Output lands next to the other committed docs assets, regardless of CWD.
OUT = Path(__file__).resolve().parent.parent / "docs" / "dofreqest-state-arm-map.svg"

# ── layout constants ──────────────────────────────────────────────── #
W, H = 920, 560
STATE_X, STATE_W, STATE_H = 470, 230, 64
ARM_X = 40
ARM_W = 150
# state vertical centers
SY = {"x0": 110, "x1": 210, "x2": 350, "x3": 450}

MEASURED = "#2e7d32"   # green
MEASURED_BG = "#e8f5e9"
HIDDEN = "#c62828"     # red
HIDDEN_BG = "#fdecea"
ARM_C = "#1565c0"      # blue
DYN_C = "#6a1b9a"      # purple (dynamics coupling)
ACT_C = "#ef6c00"      # orange (actuator)

states = [
    ("x0", "φ_rx", "rx-TCXO phase (ns)", "measured"),
    ("x1", "f_rx", "rx-TCXO freq (ppb)", "measured"),
    ("x2", "φ_do", "DO phase (ns)", "measured"),
    ("x3", "f_do", "DO freq (ppb)", "hidden"),
]

# (arm label, sigma, target state key(s), note)
arms = [
    ("A1 PPP",    "σ≈0.1 ns",  ["x0"], ""),
    ("A2 qErr",   "σ≈0.3 ppb", ["x1"], ""),
    ("A5 TDCP",   "σ≈0.026 ppb", ["x1"], ""),
    ("A3 EXTINT", "σ≈5–10 ns", ["x2"], ""),
    ("A6 pseudo", "σ grows",   ["x2"], "holdover"),
    ("A4 TICC",   "σ≈60 ps",   ["x0", "x2"], "nonlinear / coupled"),
]
# stack arms vertically on the left
ARM_TOP, ARM_GAP = 80, 70
arm_y = {a[0]: ARM_TOP + i * ARM_GAP for i, a in enumerate(arms)}


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


parts = []
parts.append(
    f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
    f'viewBox="0 0 {W} {H}" font-family="Helvetica,Arial,sans-serif">')
parts.append(f'<rect width="{W}" height="{H}" fill="white"/>')
# arrowhead markers
parts.append('''<defs>
  <marker id="armhead" markerWidth="8" markerHeight="8" refX="7" refY="3"
          orient="auto"><path d="M0,0 L7,3 L0,6 Z" fill="#1565c0"/></marker>
  <marker id="dynhead" markerWidth="8" markerHeight="8" refX="7" refY="3"
          orient="auto"><path d="M0,0 L7,3 L0,6 Z" fill="#6a1b9a"/></marker>
  <marker id="acthead" markerWidth="8" markerHeight="8" refX="7" refY="3"
          orient="auto"><path d="M0,0 L7,3 L0,6 Z" fill="#ef6c00"/></marker>
</defs>''')

parts.append(f'<text x="{W/2}" y="34" text-anchor="middle" font-size="20" '
             f'font-weight="bold">DOFreqEst — 4-state EKF: states × measurement arms</text>')
parts.append(f'<text x="{W/2}" y="54" text-anchor="middle" font-size="12" '
             f'fill="#555">green = directly measured · red = hidden '
             f'(no arm; recovered via dynamics) · purple = predict-step coupling · '
             f'orange = actuator</text>')

# ── arm boxes (left) ──────────────────────────────────────────────── #
for label, sigma, targets, note in arms:
    y = arm_y[label]
    parts.append(
        f'<rect x="{ARM_X}" y="{y-18}" width="{ARM_W}" height="36" rx="5" '
        f'fill="#e3f2fd" stroke="{ARM_C}" stroke-width="1.5"/>')
    parts.append(f'<text x="{ARM_X+ARM_W/2}" y="{y-2}" text-anchor="middle" '
                 f'font-size="12" font-weight="bold">{esc(label)}</text>')
    parts.append(f'<text x="{ARM_X+ARM_W/2}" y="{y+12}" text-anchor="middle" '
                 f'font-size="9" fill="#555">{esc(sigma)}{(" · "+esc(note)) if note else ""}</text>')

# ── state boxes (right) ───────────────────────────────────────────── #
for key, sym, desc, cls in states:
    y = SY[key]
    bg = HIDDEN_BG if cls == "hidden" else MEASURED_BG
    stroke = HIDDEN if cls == "hidden" else MEASURED
    dash = ' stroke-dasharray="6 3"' if cls == "hidden" else ""
    parts.append(
        f'<rect x="{STATE_X}" y="{y-STATE_H/2}" width="{STATE_W}" '
        f'height="{STATE_H}" rx="7" fill="{bg}" stroke="{stroke}" '
        f'stroke-width="2.5"{dash}/>')
    parts.append(f'<text x="{STATE_X+14}" y="{y-8}" font-size="16" '
                 f'font-weight="bold" fill="{stroke}">{esc(key)}  {esc(sym)}</text>')
    parts.append(f'<text x="{STATE_X+14}" y="{y+12}" font-size="11" '
                 f'fill="#444">{esc(desc)}</text>')
    tag = "HIDDEN — no arm" if cls == "hidden" else "measured"
    parts.append(f'<text x="{STATE_X+STATE_W-10}" y="{y-STATE_H/2+16}" '
                 f'text-anchor="end" font-size="10" font-style="italic" '
                 f'fill="{stroke}">{esc(tag)}</text>')

# ── measurement arrows (arm → state) ──────────────────────────────── #
for label, sigma, targets, note in arms:
    ay = arm_y[label]
    ax = ARM_X + ARM_W
    for t in targets:
        sy = SY[t]
        # route into the left edge of the state box, offset a touch if coupled
        tx = STATE_X
        ty = sy + (10 if (len(targets) > 1 and t == "x0") else
                   (-10 if len(targets) > 1 else 0))
        # cubic for a gentle curve
        mx = (ax + tx) / 2
        parts.append(
            f'<path d="M{ax},{ay} C{mx},{ay} {mx},{ty} {tx-2},{ty}" '
            f'fill="none" stroke="{ARM_C}" stroke-width="1.6" '
            f'marker-end="url(#armhead)" opacity="0.85"/>')

# ── dynamics coupling (predict step) ──────────────────────────────── #
# f_rx → φ_rx   (φ_rx += f_rx·dt)
def dyn_arrow(y_from, y_to, label):
    x = STATE_X + STATE_W + 28
    parts.append(
        f'<path d="M{STATE_X+STATE_W},{y_from} C{x},{y_from} {x},{y_to} '
        f'{STATE_X+STATE_W},{y_to}" fill="none" stroke="{DYN_C}" '
        f'stroke-width="2" marker-end="url(#dynhead)"/>')
    parts.append(f'<text x="{x+8}" y="{(y_from+y_to)/2+4}" font-size="10" '
                 f'fill="{DYN_C}">{esc(label)}</text>')

dyn_arrow(SY["x1"], SY["x0"], "∫ f_rx·dt")
dyn_arrow(SY["x3"], SY["x2"], "∫ (f_do+adjfine)·dt")

# ── actuator loop (adjfine) ───────────────────────────────────────── #
act_x, act_y = STATE_X + 40, 520
parts.append(f'<rect x="{act_x}" y="{act_y-16}" width="150" height="32" rx="5" '
             f'fill="#fff3e0" stroke="{ACT_C}" stroke-width="1.8"/>')
parts.append(f'<text x="{act_x+75}" y="{act_y+4}" text-anchor="middle" '
             f'font-size="12" font-weight="bold" fill="{ACT_C}">'
             f'adjfine = −f_do (PHC steer)</text>')
# f_do → actuator
parts.append(f'<path d="M{STATE_X+40},{SY["x3"]+STATE_H/2} '
             f'L{act_x+30},{act_y-16}" fill="none" stroke="{ACT_C}" '
             f'stroke-width="1.8" marker-end="url(#acthead)" stroke-dasharray="4 2"/>')
# actuator → φ_do (closes the loop into x2)
parts.append(f'<path d="M{act_x+150},{act_y} C{act_x+260},{act_y} '
             f'{STATE_X+STATE_W+90},{SY["x2"]+24} '
             f'{STATE_X+STATE_W},{SY["x2"]+24}" fill="none" stroke="{ACT_C}" '
             f'stroke-width="1.8" marker-end="url(#acthead)" stroke-dasharray="4 2"/>')

parts.append('</svg>')

OUT.write_text("\n".join(parts) + "\n")
print(f"wrote {OUT}")

#!/usr/bin/env python3
"""Builds docs/system_overview.svg - the README diagram of the rover as built.

Run:  python3 docs/diagrams/build_overview.py
Every number on the drawing comes from the firmware (rover_pins.h, nav_cores.h,
motion_cores.h, waterdroid_esp32.ino) - update it here if those change.
"""
import os
from xml.sax.saxutils import escape

W, H = 1200, 1090
INK, MUTED, LINE, PAPER = '#1f2933', '#52606d', '#cbd2d9', '#ffffff'
WATER, CTRL, POWER, NET, MISSION = '#1f6fb2', '#0f7c6c', '#c0392b', '#6b4fbb', '#b7791f'

out = []


def add(s):
    out.append(s)


def text(x, y, s, size=13, weight='normal', color=INK, anchor='start', italic=False):
    st = ' font-style="italic"' if italic else ''
    add(f'<text x="{x}" y="{y}" font-size="{size}" font-weight="{weight}" fill="{color}" '
        f'text-anchor="{anchor}"{st}>{escape(s)}</text>')


def box(x, y, w, h, title, lines=(), color=CTRL, fill=None, title_size=14, line_size=12):
    fill = fill or color + '14'
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="10" fill="{fill}" stroke="{color}" stroke-width="1.6"/>')
    text(x + w / 2, y + 22, title, title_size, 'bold', color, 'middle')
    for i, ln in enumerate(lines):
        text(x + w / 2, y + 42 + i * (line_size + 4), ln, line_size, 'normal', INK, 'middle')


def panel(x, y, w, h, title, color):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="14" fill="#f8fafc" stroke="{LINE}" stroke-width="1.2"/>')
    add(f'<rect x="{x}" y="{y}" width="6" height="{h}" rx="3" fill="{color}"/>')
    text(x + 20, y + 28, title, 17, 'bold', color)


def arrow(x1, y1, x2, y2, color=INK, label=None, dashed=False, both=False, lx=None, ly=None, lsize=11):
    dash = ' stroke-dasharray="6 4"' if dashed else ''
    m = color.strip('#')
    start = f' marker-start="url(#s{m})"' if both else ''
    add(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="2"{dash}{start} marker-end="url(#e{m})"/>')
    if label:
        text(lx if lx is not None else (x1 + x2) / 2, ly if ly is not None else (y1 + y2) / 2 - 6,
             label, lsize, 'normal', color, 'middle')


def path_arrow(d, color=INK, dashed=False):
    dash = ' stroke-dasharray="6 4"' if dashed else ''
    add(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="2"{dash} marker-end="url(#e{color.strip("#")})"/>')


# ---------------------------------------------------------------- header
add(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" '
    f'font-family="Segoe UI, Helvetica, Arial, sans-serif">')
add('<defs>')
for c in (INK, WATER, CTRL, POWER, NET, MISSION, MUTED):
    m = c.strip('#')
    add(f'<marker id="e{m}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
        f'<path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>')
    add(f'<marker id="s{m}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
        f'<path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>')
add('</defs>')
add(f'<rect width="{W}" height="{H}" fill="{PAPER}"/>')
text(30, 40, 'Water Rover v2 (Waterdroid) — system overview', 24, 'bold')
text(30, 62, 'One ESP32 runs the whole rover · 400 mL tanks, 380 mL per trip · fixed route by dead reckoning · '
             'controlled from Node-RED over Wi-Fi', 13, color=MUTED)

# ---------------------------------------------------------------- A. water path
panel(20, 80, 1160, 300, 'A · Water path (one trip)', WATER)
bw, bh, gap, x0, y0 = 120, 112, 20, 45, 122
steps = [
    ('River', ['coarse mesh', 'pre-filter on', 'the hose end']),
    ('Intake pump', ['relay CH1', 'on the servo boom', 'runs only at 45°']),
    ('YF-S401 flow', ['counts the batch', '(380 mL)', 'stops on dry run']),
    ('Stage 1 tank', ['raw water', 'max 400 mL']),
    ('Stage 2 column', ['gravel · sand', '· charcoal', 'gravity, ~100 mL']),
    ('Transfer pump', ['relay CH2', 'column → Stage 3', 'runs at ≥ 20 mL']),
    ('Stage 3 tank', ['clean water', 'max 400 mL']),
    ('Delivery pump', ['relay CH4', '→ fixed nozzle', '→ container']),
]
xs = []
for i, (t, ls) in enumerate(steps):
    x = x0 + i * (bw + gap)
    xs.append(x)
    box(x, y0, bw, bh, t, ls, WATER, title_size=13, line_size=11)
    if i:
        arrow(x - gap + 1, y0 + bh / 2, x - 2, y0 + bh / 2, WATER)
# servo boom on the intake
sx = xs[1]
box(sx - 6, 262, bw + 12, 74, 'SG90 servo boom', ['0° raised (start)', '45° in the water'], CTRL, title_size=12, line_size=11)
arrow(sx + bw / 2, 262, sx + bw / 2, y0 + bh + 2, CTRL, both=True)
# chlorine into stage 3
cx = xs[6]
box(cx - 6, 262, bw + 12, 74, 'Chlorine pump', ['relay CH3 · 1:50 bleach', '3 mL/L, 30 min contact'], WATER, title_size=12, line_size=11)
arrow(cx + bw / 2, 262, cx + bw / 2, y0 + bh + 2, WATER)
text(xs[3] + bw + gap / 2, 300, 'No level sensors: tank volumes are estimated from the flow sensor, pump run times and the',
     12, color=MUTED, anchor='middle')
text(xs[3] + bw + gap / 2, 318, 'gravity filter rate (VolumeModel). Bench test mode can run any pump for N seconds regardless.',
     12, color=MUTED, anchor='middle')

# ---------------------------------------------------------------- B. control & power
panel(20, 400, 760, 380, 'B · Control and power (all on the 5 V rail, one common GND)', CTRL)
ex, ey, ew, eh = 270, 450, 230, 220
box(ex, ey, ew, eh, 'ESP32 DevKit V1', [
    'Mission state machine', 'Fixed route (dead reckoning)', 'Power sequencer +', 'brownout guard',
    'Safety interlocks', 'Bench test mode', 'Wi-Fi + MQTT client'], CTRL, fill='#e6f4f1', title_size=15, line_size=12)
# inputs (left)
box(40, 460, 190, 70, 'YF-S401 flow sensor', ['signal → GPIO34', 'via 10k / 20k divider'], CTRL, title_size=12, line_size=11)
arrow(230, 495, ex - 2, 495, CTRL)
box(40, 548, 190, 70, 'Rail voltage sense', ['5 V rail → GPIO39 (VN)', 'via 10k / 10k divider'], CTRL, title_size=12, line_size=11)
arrow(230, 583, ex - 2, 583, CTRL)
# outputs (right)
ox = 540
box(ox, 440, 220, 70, 'Relay board A (2-ch)', ['CH1 intake  ← GPIO23', 'CH2 transfer ← GPIO19'], CTRL, title_size=12, line_size=11)
box(ox, 518, 220, 70, 'Relay board B (2-ch)', ['CH3 chlorine ← GPIO18', 'CH4 delivery ← GPIO4'], CTRL, title_size=12, line_size=11)
box(ox, 596, 220, 52, 'SG90 servo ← GPIO13', ['intake boom 0–45°'], CTRL, title_size=12, line_size=11)
box(ox, 656, 220, 70, 'L298N → 4 × TT motors', ['ENA 14 · IN1-4 27/26/25/33', 'ENB 32 · skid steer'], CTRL, title_size=12, line_size=11)
for yy in (475, 553, 622, 691):
    arrow(ex + ew + 2, min(max(yy, ey + 12), ey + eh - 12), ox - 2, yy, CTRL)
# power strip
add(f'<rect x="40" y="700" width="460" height="64" rx="10" fill="{POWER}12" stroke="{POWER}" stroke-width="1.6"/>')
text(270, 722, '10,000 mAh power bank (two USB outputs)', 13, 'bold', POWER, 'middle')
text(270, 740, 'USB-1 → ESP32 (laptop USB while flashing; VIN left empty)', 11, color=INK, anchor='middle')
text(270, 756, 'USB-2 → 5 V rail → relays · pumps · servo · L298N · sensor', 11, color=INK, anchor='middle')
arrow(385, 700, 385, ey + eh + 2, POWER)
text(540, 742, 'Interlocks: one pump starts at a time;', 11, color=MUTED)
text(540, 756, 'wheels locked while a pump runs or', 11, color=MUTED)
text(540, 770, 'the intake is lowered; E-STOP stops all.', 11, color=MUTED)

# ---------------------------------------------------------------- C. network
panel(800, 400, 380, 380, 'C · Network and software', NET)
box(830, 445, 320, 62, 'Phone hotspot (2.4 GHz Wi-Fi)', ['ESP32 and laptop on the same network'], NET, title_size=13, line_size=11)
box(830, 535, 320, 96, 'Laptop', ['Mosquitto MQTT broker :1883', 'Node-RED dashboard :1880/ui', 'Mission · Water train · Intake & Delivery',
                                  'Drive & Map · Power'], NET, title_size=13, line_size=11)
box(830, 660, 320, 82, 'ROS 2 Jazzy + Gazebo Harmonic', ['simulated rover, same mission', 'MQTT bridge → same dashboard',
                                                             '(mqtt:=true)'], NET, title_size=13, line_size=11)
arrow(990, 507, 990, 533, NET, both=True)
arrow(990, 658, 990, 633, NET)
arrow(782, 476, 828, 476, NET, both=True)
text(990, 764, 'MQTT topics: rover/telemetry · estop · mission/cmd · test/cmd …', 10, color=MUTED, anchor='middle')

# ---------------------------------------------------------------- D. mission
panel(20, 800, 1160, 270, 'D · Mission cycle (fixed route, metres from the depot)', MISSION)
# map
mx0, my0, sc = 95, 965, 22
def P(x, y):
    return mx0 + (x + 3) * sc, my0 - y * sc
add(f'<rect x="{mx0 - 20}" y="{my0 - 3.6 * sc - 18}" width="{12.6 * sc + 60}" height="{7.1 * sc + 34}" rx="8" fill="#fffaf0" stroke="{LINE}"/>')
rx_, ry_ = P(9.6, 0)
add(f'<rect x="{rx_ + 4}" y="{my0 - 3.4 * sc}" width="22" height="{6.8 * sc}" fill="{WATER}33"/>')
text(rx_ + 15, my0 - 3.4 * sc - 4, 'river', 11, color=WATER, anchor='middle')
route = [(0, 0), (9.6, 0), (9.0, 0), (6, 3), (6, -3), (-3, -3.5), (0, 0)]
pts = ' '.join(f'{P(x, y)[0]:.1f},{P(x, y)[1]:.1f}' for x, y in route)
add(f'<polyline points="{pts}" fill="none" stroke="{MISSION}" stroke-width="2.2" stroke-linejoin="round"/>')
marks = [((0, 0), 'depot', 'start / end'), ((9.6, 0), '', ''), ((6, 3), 'H1', ''), ((6, -3), 'H2', ''), ((-3, -3.5), 'H3', '')]
for (x, y), lab, _ in marks:
    px, py = P(x, y)
    add(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="6" fill="{MISSION if lab != "depot" else INK}"/>')
    if lab:
        text(px + (0 if lab == 'depot' else 0), py - 10 if y >= 0 else py + 20, lab, 11, 'bold', INK, 'middle')
# step chips
steps_m = [
    'Start at the depot, facing river', 'Drive 9.6 m to the river', 'Servo lowers intake to 45°',
    'Pump 380 mL (flow sensor)', 'Raise intake, back off 0.6 m', 'Filter, transfer to Stage 3',
    'Chlorinate, 30 min contact', 'Deliver at H1 → H2 → H3', 'Return to the depot']
cx0, cy0, cw, ch = 466, 838, 228, 40
for i, s in enumerate(steps_m):
    col, row = i % 3, i // 3
    x, y = cx0 + col * (cw + 12), cy0 + row * (ch + 12)
    add(f'<rect x="{x}" y="{y}" width="{cw}" height="{ch}" rx="8" fill="{MISSION}14" stroke="{MISSION}" stroke-width="1.2"/>')
    add(f'<circle cx="{x + 18}" cy="{y + ch / 2}" r="11" fill="{MISSION}"/>')
    text(x + 18, y + ch / 2 + 4, str(i + 1), 12, 'bold', '#ffffff', 'middle')
    text(x + 36, y + ch / 2 + 4, s, 12)
text(470, 1010, 'Position is estimated from drive and turn times (calibrate NAV_SPEED_MPS / NAV_TURN_DPS). Press “Arrived”',
     12, color=MUTED)
text(470, 1027, 'on the dashboard to snap the estimate back if the rover is off; a compass (QMC5883L) removes most drift.',
     12, color=MUTED)
text(470, 1051, 'Safety: E-STOP on every dashboard page · brownout guard sheds pumps · pumps and wheels never run together.',
     12, 'bold', MISSION)
add('</svg>')

dst = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'system_overview.svg')
with open(dst, 'w', encoding='utf-8') as f:
    f.write('\n'.join(out))
print('wrote', os.path.normpath(dst))

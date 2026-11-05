#!/usr/bin/env python3
"""Generate the Water Rover v2 wiring diagrams from the pin map (firmware/waterdroid_esp32/src/rover_pins.h).

    python3 docs/wiring/build_wiring.py            -> docs/wiring/*.svg (+ wiring_page.html fragment)

Wire colour code (same on every sheet):
  red 5 V rail · black GND · orange 3.3 V logic · purple PWM · blue direction · teal relay inputs
  amber servo signal · green flow-sensor pulse · brown motor / pump leads · grey analog sense
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# Literal colours for the standalone .svg files (the web page swaps these for theme tokens)
STYLE = """
.paper{fill:#fbfaf6}.box{fill:#ffffff;stroke:#2b3440;stroke-width:1.6}.box2{fill:#eef3f7;stroke:#2b3440;stroke-width:1.6}
.t{font:600 15px 'Barlow Condensed','Arial Narrow',Arial,sans-serif;fill:#1d2530}.s{font:12px 'Source Sans 3',Arial,sans-serif;fill:#5b6573}
.p{font:500 11px 'JetBrains Mono',Consolas,monospace;fill:#1d2530}.n{font:600 11px 'Source Sans 3',Arial,sans-serif;fill:#9a3412}
.w{fill:none;stroke-width:2.6;stroke-linejoin:round;stroke-linecap:round}.bus{stroke-width:6}
.w5{stroke:#d62828}.wg{stroke:#1f2328}.w33{stroke:#f08c00}.wp{stroke:#7b3fbf}.wd{stroke:#1c64d1}.wr{stroke:#0f9488}
.ws{stroke:#d9a400}.wf{stroke:#2f9e44}.wm{stroke:#8b5a2b}.wa{stroke:#7a8494}
.dot{fill:#1d2530}.pin{fill:#ffffff;stroke:#2b3440;stroke-width:1.3}.sym{fill:none;stroke:#1d2530;stroke-width:1.8}
.mot{fill:#f6e7a8;stroke:#2b3440;stroke-width:1.6}.pump{fill:#dbe9f6;stroke:#2b3440;stroke-width:1.6}.x{stroke:#c0392b;stroke-width:2}
"""


class Sheet:
    def __init__(self, w, h, title):
        self.w, self.h, self.title, self.parts = w, h, title, []

    def add(self, s):
        self.parts.append(s)

    def text(self, x, y, s, cls='s', anchor='start'):
        s = s.replace('&', '&amp;').replace('<', '&lt;')
        self.add(f'<text x="{x}" y="{y}" class="{cls}" text-anchor="{anchor}">{s}</text>')

    def box(self, x, y, w, h, title, sub='', cls='box', left=(), right=(), top=(), bottom=(), pitch=30, start=None):
        """Component with labelled pins. Returns {pin_name: (x, y)} of the pin tips."""
        self.add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="6" class="{cls}"/>')
        self.text(x + w / 2, y + 20, title, 't', 'middle')
        if sub:
            self.text(x + w / 2, y + 36, sub, 's', 'middle')
        pins = {}
        y0 = start if start is not None else y + 56
        for i, name in enumerate(left):
            py = y0 + i * pitch
            self.add(f'<circle cx="{x}" cy="{py}" r="3.5" class="pin"/>')
            self.text(x + 8, py + 4, name, 'p')
            pins[name] = (x, py)
        for i, name in enumerate(right):
            py = y0 + i * pitch
            self.add(f'<circle cx="{x + w}" cy="{py}" r="3.5" class="pin"/>')
            self.text(x + w - 8, py + 4, name, 'p', 'end')
            pins[name] = (x + w, py)
        for i, name in enumerate(top):
            px = x + 30 + i * 70
            self.add(f'<circle cx="{px}" cy="{y}" r="3.5" class="pin"/>')
            self.text(px, y + 52 if sub else y + 38, name, 'p', 'middle')
            pins[name] = (px, y)
        for i, name in enumerate(bottom):
            px = x + 30 + i * 70
            self.add(f'<circle cx="{px}" cy="{y + h}" r="3.5" class="pin"/>')
            self.text(px, y + h - 10, name, 'p', 'middle')
            pins[name] = (px, y + h)
        return pins

    def wire(self, pts, c, bus=False, label=None, lx=None, ly=None, anchor='middle'):
        d = ' '.join(f'{x},{y}' for x, y in pts)
        self.add(f'<polyline points="{d}" class="w {c}{" bus" if bus else ""}"/>')
        if label:
            if lx is None:
                (x1, y1), (x2, y2) = pts[0], pts[1]
                lx, ly = (x1 + x2) / 2, (y1 + y2) / 2 - 6
            self.text(lx, ly, label, 's', anchor)

    def dot(self, x, y):
        self.add(f'<circle cx="{x}" cy="{y}" r="4" class="dot"/>')

    def note(self, x, y, s):
        self.text(x, y, s, 'n')

    def resistor(self, x, y, vertical=True, label=''):
        """30 px zig-zag resistor starting at (x, y)."""
        if vertical:
            pts = [(x, y), (x, y + 6)] + [(x + (6 if i % 2 == 0 else -6), y + 9 + i * 3.5) for i in range(5)] + [(x, y + 26), (x, y + 32)]
        else:
            pts = [(x, y), (x + 6, y)] + [(x + 9 + i * 3.5, y + (6 if i % 2 == 0 else -6)) for i in range(5)] + [(x + 26, y), (x + 32, y)]
        self.add(f'<polyline points="{" ".join(f"{a},{b}" for a, b in pts)}" class="sym"/>')
        if label:
            self.text(x + 12 if vertical else x + 16, y + 20 if vertical else y - 10, label, 'p', 'start' if vertical else 'middle')

    def cap(self, x, y, label='', polar=False):
        """Vertical capacitor between (x, y) and (x, y + 24)."""
        self.add(f'<line x1="{x}" y1="{y}" x2="{x}" y2="{y + 9}" class="sym"/><line x1="{x - 10}" y1="{y + 9}" x2="{x + 10}" y2="{y + 9}" class="sym"/>')
        self.add(f'<line x1="{x - 10}" y1="{y + 15}" x2="{x + 10}" y2="{y + 15}" class="sym"/><line x1="{x}" y1="{y + 15}" x2="{x}" y2="{y + 24}" class="sym"/>')
        if polar:
            self.text(x + 13, y + 8, '+', 'p')
        if label:
            self.text(x + 14, y + 20, label, 'p')

    def diode(self, x, y1, y2):
        """Vertical flyback diode between y1 (cathode, +) and y2 (anode, -)."""
        m = (y1 + y2) / 2
        self.add(f'<line x1="{x}" y1="{y1}" x2="{x}" y2="{m - 7}" class="sym"/><line x1="{x}" y1="{m + 7}" x2="{x}" y2="{y2}" class="sym"/>')
        self.add(f'<polygon points="{x - 7},{m + 7} {x + 7},{m + 7} {x},{m - 6}" class="sym"/><line x1="{x - 7}" y1="{m - 7}" x2="{x + 7}" y2="{m - 7}" class="sym"/>')

    def gnd(self, x, y):
        self.add(f'<line x1="{x}" y1="{y}" x2="{x}" y2="{y + 8}" class="w wg"/>'
                 f'<line x1="{x - 10}" y1="{y + 8}" x2="{x + 10}" y2="{y + 8}" class="sym"/>'
                 f'<line x1="{x - 6}" y1="{y + 12}" x2="{x + 6}" y2="{y + 12}" class="sym"/>'
                 f'<line x1="{x - 2}" y1="{y + 16}" x2="{x + 2}" y2="{y + 16}" class="sym"/>')

    def cross(self, x, y, label):
        self.add(f'<line x1="{x - 7}" y1="{y - 7}" x2="{x + 7}" y2="{y + 7}" class="x"/><line x1="{x - 7}" y1="{y + 7}" x2="{x + 7}" y2="{y - 7}" class="x"/>')
        self.note(x + 11, y + 4, label)

    def svg(self, standalone=True):
        style = f'<style>{STYLE}</style>' if standalone else ''
        bg = f'<rect width="{self.w}" height="{self.h}" class="paper"/>' if standalone else ''
        return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.w} {self.h}" role="img" aria-label="{self.title}">'
                f'<title>{self.title}</title>{style}{bg}' + ''.join(self.parts) + '</svg>')


# =========================================================== 1. power distribution
def power():
    s = Sheet(1000, 560, 'Power distribution')
    pb = s.box(20, 150, 190, 170, 'Power bank 10,000 mAh', '5 V USB', right=['USB #1', 'USB #2', 'GND'], start=210, pitch=40)
    esp = s.box(330, 30, 190, 120, 'ESP32 DevKit V1', 'logic only', left=['USB'], right=['3V3', 'GND', 'GPIO39'], start=80, pitch=24)
    s.wire([pb['USB #1'], (260, 210), (260, 80), esp['USB']], 'w5', label='USB cable', lx=268, ly=150, anchor='start')
    # 5 V rail and GND bus
    s.wire([pb['USB #2'], (960, 250)], 'w5', bus=True)
    s.text(340, 240, '5 V RAIL', 't', 'middle')
    s.wire([pb['GND'], (960, 290)], 'wg', bus=True)
    s.text(340, 312, 'COMMON GND', 't', 'middle')
    s.wire([esp['GND'], (560, 104), (560, 290)], 'wg')
    s.dot(560, 290)
    s.note(568, 200, 'ESP32 GND joins the GND bus')
    # loads hanging from the rail
    loads = [(250, 'L298N', '+12V (Vs) and +5V'), (390, 'Relay module', 'JD-VCC + 4 x COM'),
             (530, 'SG90 servo', 'red / brown'), (670, 'Flow sensor', 'red / black')]
    for x, name, sub in loads:
        s.wire([(x, 250), (x, 380)], 'w5')
        s.wire([(x + 40, 290), (x + 40, 380)], 'wg')
        s.dot(x, 250); s.dot(x + 40, 290)
        s.box(x - 30, 380, 110, 70, name, sub)
    # bulk capacitor
    s.wire([(820, 250), (820, 360)], 'w5'); s.dot(820, 250)
    s.cap(820, 360, '1000 uF', True)
    s.wire([(820, 384), (820, 470)], 'wg')
    s.wire([(860, 290), (860, 470), (820, 470)], 'wg'); s.dot(860, 290)
    s.text(840, 500, 'bulk cap across the rail', 's', 'middle')
    # rail sense divider
    s.wire([(900, 250), (900, 330)], 'w5'); s.dot(900, 250)
    s.resistor(900, 330, label='10k')
    s.wire([(900, 362), (900, 380)], 'wa'); s.dot(900, 380)
    s.resistor(900, 380, label='10k')
    s.wire([(900, 412), (900, 440)], 'wg'); s.gnd(900, 440)
    s.wire([(900, 380), (940, 380), (940, 160), (600, 160), (600, 128), esp['GPIO39']], 'wa')
    s.note(30, 520, 'Two USB outputs isolate the ESP32 from motor inrush. Single-output bank: feed the ESP32 from the rail and keep the 1000 uF cap.')
    return s


# =========================================================== 2. drive
def drive():
    s = Sheet(1000, 520, 'Drive: ESP32 to L298N to four TT motors')
    left = ['GPIO14', 'GPIO27', 'GPIO26', 'GPIO25', 'GPIO33', 'GPIO32', 'GND']
    lpins = ['ENA', 'IN1', 'IN2', 'IN3', 'IN4', 'ENB', 'GND']
    esp = s.box(30, 90, 170, 300, 'ESP32', 'rover_pins.h', right=left, start=150, pitch=32)
    l = s.box(400, 90, 190, 300, 'L298N', 'dual H-bridge', left=lpins, right=['OUT1', 'OUT2', 'OUT3', 'OUT4'],
              start=150, pitch=32)
    cls = ['wp', 'wd', 'wd', 'wd', 'wd', 'wp', 'wg']
    for a, b, c in zip(left, lpins, cls):
        s.wire([esp[a], l[b]], c)
    s.text(300, 140, 'PWM left', 's', 'middle'); s.text(300, 302, 'PWM right', 's', 'middle')
    # supply terminals on top
    s.add('<circle cx="440" cy="90" r="3.5" class="pin"/><circle cx="500" cy="90" r="3.5" class="pin"/><circle cx="560" cy="90" r="3.5" class="pin"/>')
    s.text(446, 80, '+12V', 'p'); s.text(506, 80, '+5V', 'p'); s.text(566, 80, 'GND', 'p')
    s.wire([(440, 90), (440, 40), (300, 40)], 'w5'); s.wire([(500, 90), (500, 40)], 'w5'); s.dot(440, 40)
    s.text(296, 44, '5 V rail', 's', 'end')
    s.wire([(560, 90), (560, 22), (600, 22)], 'wg'); s.text(604, 26, 'GND bus', 's')
    s.cross(40, 440, 'remove the ENA and ENB jumpers (PWM comes from the ESP32)')
    s.cross(40, 466, 'remove the 5V-EN jumper; feed +12V and +5V both from the rail')
    # motors
    mots = [('LEFT FRONT', 700, 40), ('LEFT REAR', 700, 120), ('RIGHT FRONT', 700, 240), ('RIGHT REAR', 700, 320)]
    for name, x, y in mots:
        s.add(f'<rect x="{x}" y="{y}" width="190" height="56" rx="10" class="mot"/>')
        s.text(x + 95, y + 24, name, 't', 'middle'); s.text(x + 95, y + 42, 'TT geared motor', 's', 'middle')
        s.add(f'<circle cx="{x}" cy="{y + 18}" r="3.5" class="pin"/><circle cx="{x}" cy="{y + 40}" r="3.5" class="pin"/>')
        s.text(x - 6, y + 22, 'a', 'p', 'end'); s.text(x - 6, y + 44, 'b', 'p', 'end')
    # OUT1/2 -> both left motors in parallel, OUT3/4 -> both right motors
    for out, xm, ys in [('OUT1', 625, [58, 138]), ('OUT2', 655, [80, 160]), ('OUT3', 625, [258, 338]), ('OUT4', 655, [280, 360])]:
        ox, oy = l[out]
        s.wire([(ox, oy), (xm, oy), (xm, ys[0]), (700, ys[0])], 'wm')
        s.wire([(xm, oy), (xm, ys[1]), (700, ys[1])], 'wm')
        s.dot(xm, oy)
    s.note(700, 410, 'Both motors on a side are in parallel.')
    s.note(700, 430, 'A side spins backwards? Swap its two leads.')
    s.text(30, 500, 'Dot = joined. Wires crossing without a dot are not connected.', 's')
    return s


# =========================================================== 3. relays and pumps
def relays():
    s = Sheet(1000, 620, 'Relays: ESP32 to 4-channel relay module to four pumps')
    left = ['3V3', 'GND', 'GPIO23', 'GPIO19', 'GPIO18', 'GPIO4']
    rl = ['VCC', 'GND', 'IN1', 'IN2', 'IN3', 'IN4']
    esp = s.box(30, 120, 170, 260, 'ESP32', '', right=left, start=170, pitch=34)
    rm = s.box(380, 120, 200, 460, '4-ch relay module', 'active LOW', left=rl, start=170, pitch=34)
    for a, b, c in zip(left, rl, ['w33', 'wg', 'wr', 'wr', 'wr', 'wr']):
        s.wire([esp[a], rm[b]], c)
    # JD-VCC on top
    s.add('<circle cx="540" cy="120" r="3.5" class="pin"/>'); s.text(534, 112, 'JD-VCC', 'p', 'end')
    s.wire([(540, 120), (540, 70), (720, 70)], 'w5')
    s.cross(40, 420, 'remove the VCC to JD-VCC jumper on the relay board')
    # contacts on the right
    names = [('CH1', 'INTAKE PUMP', '#1'), ('CH2', 'ALUM PUMP', '#3'), ('CH3', 'CHLORINE PUMP', '#3'), ('CH4', 'DELIVERY PUMP', '#8')]
    s.wire([(720, 70), (720, 570)], 'w5', bus=True); s.text(728, 64, '5 V rail', 's')
    s.wire([(960, 100), (960, 570)], 'wg', bus=True); s.text(964, 94, 'GND', 's', 'end')
    for i, (ch, pump, issue) in enumerate(names):
        y = 150 + i * 110
        com, no = (580, y), (580, y + 26)
        s.add(f'<circle cx="580" cy="{y}" r="3.5" class="pin"/><circle cx="580" cy="{y + 26}" r="3.5" class="pin"/>')
        s.text(572, y + 4, f'{ch} COM', 'p', 'end'); s.text(572, y + 30, f'{ch} NO', 'p', 'end')
        s.wire([com, (720, y)], 'w5'); s.dot(720, y)
        # pump box
        px, py = 780, y - 14
        s.add(f'<rect x="{px}" y="{py}" width="140" height="64" rx="8" class="pump"/>')
        s.text(px + 70, py + 26, pump, 't', 'middle'); s.text(px + 70, py + 44, f'5 V pump . Issue {issue}', 's', 'middle')
        s.wire([no, (755, y + 26), (755, py + 18), (px, py + 18)], 'wm')
        s.text(px - 6, py + 12, '+', 'p', 'end')
        s.wire([(px + 140, py + 46), (960, py + 46)], 'wg'); s.dot(960, py + 46)
        s.text(px + 146, py + 40, '-', 'p')
        # flyback diode across the pump: cathode on the + lead, anode to GND
        s.dot(755, y + 26)
        s.wire([(755, y + 26), (755, y + 40)], 'wm')
        s.diode(755, y + 40, y + 76)
        s.wire([(755, y + 76), (755, y + 84), (940, y + 84), (940, py + 46)], 'wg'); s.dot(940, py + 46)
    s.note(30, 460, 'Pin order CH1-CH4 = intake, alum, chlorine, delivery (power_core.h).')
    s.note(30, 482, 'Flyback diode across each pump: stripe (cathode) to +.')
    s.note(30, 504, 'JD-VCC on the 5 V rail, VCC on 3.3 V:')
    s.note(30, 524, 'then the 3.3 V GPIOs drive the opto-couplers.')
    return s


# =========================================================== 4. servo, flow sensor, rail sense
def sensors():
    s = Sheet(1000, 470, 'Servo, flow sensor and battery sense')
    # SG90
    sv = s.box(110, 70, 170, 150, 'SG90 servo', 'intake arm (#6)', left=['orange', 'red', 'brown'], start=126, pitch=34)
    s.wire([sv['orange'], (60, 126)], 'ws'); s.text(54, 130, 'GPIO13', 'p', 'end')
    s.wire([sv['red'], (60, 160)], 'w5'); s.text(54, 164, '5 V', 'p', 'end')
    s.wire([sv['brown'], (60, 194)], 'wg'); s.text(54, 198, 'GND', 'p', 'end')
    s.dot(85, 160); s.dot(95, 194)
    s.wire([(85, 160), (85, 250)], 'w5'); s.cap(85, 250, '100 uF', True)
    s.wire([(85, 274), (85, 290), (95, 290), (95, 194)], 'wg')
    s.text(110, 320, '100 uF across the servo leads, close to it', 's')
    # flow sensor
    fs = s.box(340, 70, 200, 150, 'Water-flow sensor', 'intake hose (#1, #5)', right=['red', 'black', 'yellow'], start=126, pitch=34)
    s.wire([fs['red'], (600, 126)], 'w5'); s.text(606, 130, '5 V', 'p')
    s.wire([fs['black'], (600, 160)], 'wg'); s.text(606, 164, 'GND', 'p')
    s.wire([fs['yellow'], (580, 194), (580, 220)], 'wf')
    s.resistor(580, 220, label='10k')
    s.wire([(580, 252), (580, 280)], 'wf'); s.dot(580, 280)
    s.resistor(580, 280, label='20k')
    s.wire([(580, 312), (580, 330)], 'wg'); s.gnd(580, 330)
    s.wire([(580, 280), (650, 280)], 'wf'); s.text(656, 284, 'GPIO34', 'p')
    s.note(340, 380, '5 V pulses -> 3.3 V: 10k / 20k divider.')
    s.note(340, 400, 'Fit it after the pre-filter, arrow in the flow direction.')
    # rail sense
    s.text(830, 90, 'Battery / 5 V rail sense (#10)', 't', 'middle')
    s.wire([(830, 110), (830, 140)], 'w5'); s.text(838, 124, '5 V rail', 'p')
    s.resistor(830, 140, label='10k')
    s.wire([(830, 172), (830, 200)], 'wa'); s.dot(830, 200)
    s.resistor(830, 200, label='10k')
    s.wire([(830, 232), (830, 260)], 'wg'); s.gnd(830, 260)
    s.wire([(830, 200), (900, 200)], 'wa'); s.text(906, 204, 'GPIO39', 'p')
    s.wire([(870, 200), (870, 222)], 'wa'); s.dot(870, 200)
    s.cap(870, 222, '100 nF')
    s.wire([(870, 246), (870, 260), (830, 260)], 'wg')
    s.note(740, 320, 'GPIO39 reads half the rail voltage.')
    s.note(740, 340, 'Set VCAL from a multimeter reading.')
    return s


SHEETS = [('power', power, 'Power distribution',
           'Two USB outputs from the power bank: one powers the ESP32 by its USB cable, the other is the 5 V rail '
           'for the motors, pumps, servo and flow sensor. Every ground is joined.'),
          ('drive', drive, 'Drive (#9)',
           'The four TT motors run in pairs: both left motors in parallel on OUT1/OUT2, both right motors on '
           'OUT3/OUT4. ENA and ENB carry PWM from the ESP32, IN1-IN4 set direction.'),
          ('relays', relays, 'Relays and pumps (#1 #3 #8 #10)',
           'The four relays switch the four pumps, never the motors. Each relay passes the 5 V rail from COM to NO '
           'and on to its pump; the pump returns to ground.'),
          ('sensors', sensors, 'Servo, flow sensor, battery sense (#5 #6 #10)',
           'The servo signal and the flow-sensor pulse are the only other ESP32 inputs and outputs. Both sensors '
           'that feed analog or 5 V signals into the ESP32 go through dividers.')]


def main():
    out = []
    for key, fn, title, blurb in SHEETS:
        sheet = fn()
        with open(os.path.join(HERE, f'{key}.svg'), 'w') as f:
            f.write(sheet.svg(standalone=True) + '\n')
        out.append((key, title, blurb, sheet.svg(standalone=False)))
    print(f'{len(out)} sheets written to {HERE}')
    return out


if __name__ == '__main__':
    main()


# ----------------------------------------------------------------- web page (artifact)
PAGE_CSS = """
:root{--paper:#f7f6f1;--panel:#ffffff;--ink:#1d2530;--muted:#5b6573;--line:#2b3440;--soft:#e7e9e4;--chip:#eef3f7;
--note:#9a3412;--v5:#d62828;--gnd:#1f2328;--v33:#e07b00;--pwm:#7b3fbf;--dir:#1c64d1;--rly:#0f9488;--srv:#c99700;
--flw:#2f9e44;--mot:#8b5a2b;--ana:#7a8494;--motfill:#f6e7a8;--pumpfill:#dbe9f6;
--fd:'Barlow Condensed','Arial Narrow',Arial,sans-serif;--fb:'Source Sans 3',system-ui,sans-serif;--fm:'JetBrains Mono',ui-monospace,Consolas,monospace}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--paper:#14181d;--panel:#1b2128;--ink:#e6e9ee;--muted:#9aa4b2;--line:#8a96a6;--soft:#262d36;--chip:#232c36;
--note:#f4a261;--v5:#ff5a5a;--gnd:#c9ced6;--v33:#ffa53a;--pwm:#b48cff;--dir:#5b9dff;--rly:#2dd4bf;--srv:#ffd43b;--flw:#51cf66;--mot:#d29b62;--ana:#a3acb9;
--motfill:#4a4122;--pumpfill:#1f3550;color-scheme:dark}}
:root[data-theme="dark"]{--paper:#14181d;--panel:#1b2128;--ink:#e6e9ee;--muted:#9aa4b2;--line:#8a96a6;--soft:#262d36;--chip:#232c36;
--note:#f4a261;--v5:#ff5a5a;--gnd:#c9ced6;--v33:#ffa53a;--pwm:#b48cff;--dir:#5b9dff;--rly:#2dd4bf;--srv:#ffd43b;--flw:#51cf66;--mot:#d29b62;--ana:#a3acb9;
--motfill:#4a4122;--pumpfill:#1f3550;color-scheme:dark}
body{background:var(--paper);color:var(--ink);font:15px/1.55 var(--fb)}
.wrap{max-width:1120px;margin:0 auto;padding-block:28px 56px;padding-left:max(16px,env(safe-area-inset-left,0px));padding-right:max(16px,env(safe-area-inset-right,0px))}
header{display:flex;flex-wrap:wrap;gap:8px 24px;align-items:end;justify-content:space-between;border-bottom:2px solid var(--line);padding-bottom:14px}
h1{font:600 34px/1.05 var(--fd);letter-spacing:.01em;margin:0;text-wrap:balance}
.meta{font:12px var(--fm);color:var(--muted);text-align:right}
.lead{max-width:68ch;color:var(--muted);margin:14px 0 0}
nav{display:flex;flex-wrap:wrap;gap:6px;margin:18px 0 6px}
nav a{font:600 13px var(--fd);letter-spacing:.06em;text-transform:uppercase;color:var(--ink);text-decoration:none;border:1px solid var(--line);border-radius:3px;padding:5px 10px}
nav a:hover,nav a:focus-visible{background:var(--chip);outline:none}
.legend{display:flex;flex-wrap:wrap;gap:6px 16px;margin:16px 0 4px;font-size:13px;color:var(--muted)}
.legend span{display:inline-flex;align-items:center;gap:6px}.legend i{width:22px;height:4px;border-radius:2px;display:inline-block}
section{margin-top:34px}
h2{font:600 22px var(--fd);letter-spacing:.02em;margin:0 0 4px;display:flex;gap:10px;align-items:baseline}
h2 small{font:500 12px var(--fm);color:var(--muted)}
section>p{margin:0 0 12px;max-width:72ch;color:var(--muted)}
.sheet{background:var(--panel);border:1px solid var(--soft);border-radius:4px;overflow-x:auto}
.sheet svg{display:block;min-width:760px;width:100%;height:auto}
.box{fill:var(--panel);stroke:var(--line);stroke-width:1.6}.box2{fill:var(--chip);stroke:var(--line);stroke-width:1.6}
.t{font:600 15px var(--fd);fill:var(--ink)}.s{font:12px var(--fb);fill:var(--muted)}.p{font:500 11px var(--fm);fill:var(--ink)}.n{font:600 11px var(--fb);fill:var(--note)}
.w{fill:none;stroke-width:2.6;stroke-linejoin:round;stroke-linecap:round}.bus{stroke-width:6}
.w5{stroke:var(--v5)}.wg{stroke:var(--gnd)}.w33{stroke:var(--v33)}.wp{stroke:var(--pwm)}.wd{stroke:var(--dir)}.wr{stroke:var(--rly)}
.ws{stroke:var(--srv)}.wf{stroke:var(--flw)}.wm{stroke:var(--mot)}.wa{stroke:var(--ana)}
.dot{fill:var(--ink)}.pin{fill:var(--panel);stroke:var(--line);stroke-width:1.3}.sym{fill:none;stroke:var(--ink);stroke-width:1.8}
.mot{fill:var(--motfill);stroke:var(--line);stroke-width:1.6}.pump{fill:var(--pumpfill);stroke:var(--line);stroke-width:1.6}.x{stroke:var(--note);stroke-width:2}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:20px;margin-top:10px}
.tbl{overflow-x:auto;background:var(--panel);border:1px solid var(--soft);border-radius:4px}
table{border-collapse:collapse;width:100%;font-size:13.5px}
th,td{text-align:left;padding:7px 12px;border-bottom:1px solid var(--soft);vertical-align:top}
th{font:600 12px var(--fd);letter-spacing:.08em;text-transform:uppercase;color:var(--muted)}
td:first-child{font:600 12.5px var(--fm);white-space:nowrap;font-variant-numeric:tabular-nums}
tr:last-child td{border-bottom:0}
.sw{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:6px;vertical-align:-1px}
ol.check{margin:0;padding-left:20px;display:grid;gap:6px;max-width:72ch}
ol.check li::marker{font:600 13px var(--fm);color:var(--muted)}
footer{margin-top:40px;font-size:12.5px;color:var(--muted);border-top:1px solid var(--soft);padding-top:12px}
code{font:12.5px var(--fm);background:var(--chip);padding:1px 5px;border-radius:3px}
"""

PINS = [('14', 'L298N ENA (left PWM)', '--pwm'), ('27 / 26', 'L298N IN1 / IN2 (left direction)', '--dir'),
        ('25 / 33', 'L298N IN3 / IN4 (right direction)', '--dir'), ('32', 'L298N ENB (right PWM)', '--pwm'),
        ('23', 'Relay IN1 - intake pump', '--rly'), ('19', 'Relay IN2 - alum pump', '--rly'),
        ('18', 'Relay IN3 - chlorine pump', '--rly'), ('4', 'Relay IN4 - delivery pump', '--rly'),
        ('13', 'SG90 signal (orange)', '--srv'), ('34', 'Flow sensor pulse via 10k / 20k', '--flw'),
        ('39 (VN)', '5 V rail sense, 10k / 10k divider', '--ana'), ('3V3', 'Relay module VCC', '--v33'),
        ('35 / 36', 'free: optional level sensors', '--muted'), ('16 / 17 · 21 / 22', 'reserved: GPS · compass (#7)', '--muted')]

LEGEND = [('5 V rail', '--v5'), ('GND', '--gnd'), ('3.3 V', '--v33'), ('PWM', '--pwm'), ('Direction', '--dir'),
          ('Relay input', '--rly'), ('Servo signal', '--srv'), ('Flow pulse', '--flw'), ('Motor / pump leads', '--mot'),
          ('Analog sense', '--ana')]

CHECK = ['Power bank off. Wire every GND to one common bus first.',
         'L298N: remove the ENA, ENB and 5V-EN jumpers; +12V and +5V both on the 5 V rail.',
         'Relay board: remove the VCC to JD-VCC jumper; VCC to ESP32 3V3, JD-VCC to the 5 V rail.',
         'Fit a flyback diode across each pump (stripe to +) and the 1000 uF cap across the rail.',
         'Flow sensor yellow wire through the 10k / 20k divider to GPIO34, never straight in.',
         'ESP32 on its own USB output. Flash, open the Serial Monitor at 115200, check the boot line.',
         'Switch on the motor rail. Tap Forward on the dashboard: both sides must turn forward.',
         'Run each pump from the dashboard (intake, dose alum 1 mL, dose chlorine 0.5 mL, deliver 50 mL).']


def page():
    sheets = main()
    nav = ''.join(f'<a href="#{k}">{t.split(" (")[0]}</a>' for k, t, _, _ in sheets) + '<a href="#pins">Pin map</a><a href="#check">Checklist</a>'
    legend = ''.join(f'<span><i style="background:var({v})"></i>{n}</span>' for n, v in LEGEND)
    secs = ''
    for i, (k, t, blurb, svg) in enumerate(sheets, 1):
        secs += (f'<section id="{k}"><h2>{t}<small>sheet {i} / {len(sheets)}</small></h2><p>{blurb}</p>'
                 f'<div class="sheet">{svg}</div></section>')
    rows = ''.join(f'<tr><td>{p}</td><td><span class="sw" style="background:var({c})"></span>{d}</td></tr>' for p, d, c in PINS)
    checks = ''.join(f'<li>{c}</li>' for c in CHECK)
    html = f"""<title>Water Rover v2 Wiring</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="preconnect" href="https://fonts.googleapis.com"><link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Barlow+Condensed:wght@500;600&family=JetBrains+Mono:wght@500;600&family=Source+Sans+3:wght@400;600&display=swap">
<style>{PAGE_CSS}</style>
<div class="wrap">
<header><h1>Water Rover v2 Wiring</h1><div class="meta">ESP32 DevKit V1 · pin map rover_pins.h<br>build Nov 2026 · flow sensor, no level sensors</div></header>
<p class="lead">Four sheets for the one-ESP32 build: power, drive, relays and pumps, and the servo and sensors. Every GPIO matches <code>firmware/waterdroid_esp32/src/rover_pins.h</code>; the wire colours are the same on every sheet.</p>
<nav>{nav}</nav>
<div class="legend">{legend}</div>
{secs}
<section id="pins"><h2>Pin map<small>ESP32 GPIO</small></h2><p>Pins 34-39 are input-only and stay readable with Wi-Fi on. Never use GPIO 6-11 (SPI flash).</p>
<div class="grid"><div class="tbl"><table><thead><tr><th>GPIO</th><th>Connects to</th></tr></thead><tbody>{rows}</tbody></table></div>
<div><h2 id="check" style="font-size:19px">Bring-up checklist</h2><ol class="check">{checks}</ol></div></div></section>
<footer>Generated from <code>docs/wiring/build_wiring.py</code> in n02531614p/water-rover-v2-simulation. Edit the script, not the drawings.</footer>
</div>"""
    return html


if __name__ == '__main__' and os.environ.get('WIRING_PAGE'):
    with open(os.environ['WIRING_PAGE'], 'w') as f:
        f.write(page())
    print('page written to', os.environ['WIRING_PAGE'])

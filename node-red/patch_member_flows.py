#!/usr/bin/env python3
"""One-shot (idempotent) fixes that connect the team members' Node-RED flows to the rover.

    python3 node-red/patch_member_flows.py

  robotic_arm_flow.json      (#6)  'serial_command' was only a debug preview -> MQTT rover/arm/cmd + angle gauge
  rover_teleop_flow.json     (#9)  buttons went to a debug node and their ui_group was missing
                                   -> group added, buttons -> MQTT rover/drive/cmd
  chemical_dosing_flow.json  (#3)  Raspberry-Pi GPIO nodes replaced by ESP32-metered doses on rover/dosing/cmd,
                                   volumes rescaled from 200 L tanks to the real 0.5-1 L tanks, dosing panel added
  river_pump_flow.json       (#1)  'Manual Water Intake' button + fill gauge (rover/intake/*) added
"""
import json
import os

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
BROKER = {'id': 'wd_broker', 'type': 'mqtt-broker', 'name': 'Rover broker', 'broker': 'localhost', 'port': '1883',
          'clientid': '', 'autoConnect': True, 'usetls': False, 'protocolVersion': '4', 'keepalive': '60',
          'cleansession': True, 'autoUnsubscribe': True, 'birthTopic': '', 'birthQos': '0', 'birthPayload': '',
          'birthMsg': {}, 'closeTopic': '', 'closeQos': '0', 'closePayload': '', 'closeMsg': {}, 'willTopic': '',
          'willQos': '0', 'willPayload': '', 'willMsg': {}, 'userProps': '', 'sessionExpiry': ''}


def load(name):
    return json.load(open(os.path.join(ROOT, name)))


INDENT = {'robotic_arm_flow.json': 2}      # keep each file's original indentation (small diffs)


def save(name, nodes):
    with open(os.path.join(ROOT, name), 'w') as f:
        json.dump(nodes, f, indent=INDENT.get(name, 4), ensure_ascii=False)
        f.write('\n')
    print('patched', name)


def put(nodes, node):
    """insert or replace by id"""
    for i, n in enumerate(nodes):
        if n['id'] == node['id']:
            nodes[i] = node
            return node
    nodes.append(node)
    return node


def get(nodes, id_):
    return next(n for n in nodes if n['id'] == id_)


def mqtt_out(id_, z, topic, x, y, broker='wd_broker', retain='false'):
    return {'id': id_, 'type': 'mqtt out', 'z': z, 'name': topic, 'topic': topic, 'qos': '0', 'retain': retain,
            'respTopic': '', 'contentType': '', 'userProps': '', 'correl': '', 'expiry': '', 'broker': broker,
            'x': x, 'y': y, 'wires': []}


def mqtt_in(id_, z, topic, x, y, wires, broker='wd_broker'):
    return {'id': id_, 'type': 'mqtt in', 'z': z, 'name': topic, 'topic': topic, 'qos': '0', 'datatype': 'json',
            'broker': broker, 'nl': False, 'rap': True, 'rh': 0, 'inputs': 0, 'x': x, 'y': y, 'wires': [wires]}


def fn(id_, z, name, code, outputs, wires, x, y):
    return {'id': id_, 'type': 'function', 'z': z, 'name': name, 'func': code, 'outputs': outputs, 'timeout': 0,
            'noerr': 0, 'initialize': '', 'finalize': '', 'libs': [], 'x': x, 'y': y, 'wires': wires}


def gauge(id_, z, group, title, unit, mn, mx, order, x, y):
    return {'id': id_, 'type': 'ui_gauge', 'z': z, 'name': title, 'group': group, 'order': order, 'width': 3,
            'height': 3, 'gtype': 'gage', 'title': title, 'label': unit, 'format': '{{value}}', 'min': mn, 'max': mx,
            'colors': ['#e6a23c', '#3498db', '#2ecc71'], 'seg1': '', 'seg2': '', 'diff': False, 'className': '',
            'x': x, 'y': y, 'wires': []}


def text(id_, z, group, label, order, x, y):
    return {'id': id_, 'type': 'ui_text', 'z': z, 'group': group, 'order': order, 'width': 6, 'height': 1,
            'name': label, 'label': label, 'format': '{{msg.payload}}', 'layout': 'row-spread', 'className': '',
            'x': x, 'y': y, 'wires': []}


def button(id_, z, group, label, payload, order, wires, x, y, bg=''):
    return {'id': id_, 'type': 'ui_button', 'z': z, 'name': label, 'group': group, 'order': order, 'width': 3,
            'height': 1, 'passthru': False, 'label': label, 'tooltip': '', 'color': '#ffffff' if bg else '',
            'bgcolor': bg, 'className': '', 'icon': '', 'payload': payload, 'payloadType': 'str', 'topic': '',
            'topicType': 'str', 'x': x, 'y': y, 'wires': [wires]}


# ------------------------------------------------------------------ #6 robotic arm
f = load('robotic_arm_flow.json')
put(f, BROKER)
put(f, mqtt_out('serial_command', 'arm_tab', 'rover/arm/cmd', 640, 160))      # keeps every existing wire
put(f, {'id': 'arm_cmd_preview', 'type': 'debug', 'z': 'arm_tab', 'name': 'Arm Command Preview', 'active': True,
        'tosidebar': True, 'console': False, 'tostatus': True, 'complete': 'payload', 'targetType': 'msg',
        'statusVal': 'payload', 'statusType': 'auto', 'x': 650, 'y': 220, 'wires': []})
for bid in ('extend_button', 'retract_button'):
    w = get(f, bid)['wires'][0]
    if 'arm_cmd_preview' not in w:
        w.append('arm_cmd_preview')
get(f, 'angle_command')['wires'][0] = ['serial_command', 'angle_display', 'arm_cmd_preview']
put(f, mqtt_in('arm_status_in', 'arm_tab', 'rover/arm/status', 160, 300, ['arm_status_angle']))
put(f, fn('arm_status_angle', 'arm_tab', 'Live arm angle', 'msg.payload = Number((msg.payload || {}).angle || 0);\nreturn msg;',
          1, [['arm_angle_gauge']], 400, 300))
put(f, gauge('arm_angle_gauge', 'arm_tab', 'arm_group', 'Live arm angle', 'deg', 0, 90, 5, 640, 300))
c = get(f, 'arm_comment')
c['name'] = 'Commands go to the ESP32 over MQTT rover/arm/cmd (EXTEND | RETRACT | ANGLE:n)'
c['info'] = 'For a USB bench test with robotic_arm.ino instead, swap the mqtt out for a serial out (9600 baud).'
save('robotic_arm_flow.json', f)

# ------------------------------------------------------------------ #9 teleop
f = load('rover_teleop_flow.json')
put(f, BROKER)
put(f, {'id': 'teleop-ui-tab', 'type': 'ui_tab', 'name': 'Rover Teleop', 'icon': 'gamepad', 'order': 6,
        'disabled': False, 'hidden': False})
put(f, {'id': 'teleop-group', 'type': 'ui_group', 'name': 'D-pad (each press ~0.6 s)', 'tab': 'teleop-ui-tab',
        'order': 1, 'disp': True, 'width': '6', 'collapse': False, 'className': ''})
put(f, mqtt_out('teleop-mqtt', 'teleop-flow', 'rover/drive/cmd', 520, 200))
for n in f:
    if n['type'] == 'ui_button' and 'teleop-mqtt' not in n['wires'][0]:
        n['wires'][0].append('teleop-mqtt')
save('rover_teleop_flow.json', f)

# ------------------------------------------------------------------ #3 chemical dosing
f = load('chemical_dosing_flow.json')
ctl = get(f, 'fn_dosing_controller')
code = ctl['func']
for old, new in [
        ('pump_rate_ml_sec: 1.5,       // Peristaltic pump rate (e.g. 90 mL/min = 1.5 mL/s)',
         'pump_rate_ml_sec: 1.0,       // 5 V peristaltic pump: 2 s = 2 mL (calibrate)'),
        ('ratio_ml_per_L: 0.06,        // 0.06 mL Alum per Liter water (~30-60 mg/L typical)',
         'ratio_ml_per_L: 3.0,         // 1 % alum stock: 3 mL per L water (~30 mg/L)'),
        ('min_dose_threshold_L: 20,    // Trigger dose when at least 20L processed',
         'min_dose_threshold_L: 0.5,   // rover tanks are 0.5-1 L: dose per half litre'),
        ('fixed_timer_dose_ml: 5.0,    // Dose for pure timer trigger', 'fixed_timer_dose_ml: 1.5,    // Dose for pure timer trigger'),
        ('max_single_dose_ml: 15.0,    // Safety ceiling per injection', 'max_single_dose_ml: 10.0,    // Safety ceiling per injection'),
        ('current_stock_ml: 1000.0     // Reservoir level', 'current_stock_ml: 250.0      // Reservoir level'),
        ('pump_rate_ml_sec: 0.8,       // Peristaltic pump rate (e.g. 48 mL/min = 0.8 mL/s)',
         'pump_rate_ml_sec: 1.0,       // 5 V dosing pump (calibrate)'),
        ('ratio_ml_per_L: 0.005,       // 0.005 mL of 5% NaOCl per Liter (~2.5 ppm Cl2)',
         'ratio_ml_per_L: 0.6,         // 0.35 % NaOCl stock (bleach 1:10): 0.6 mL per L (~2 mg/L)'),
        ('fixed_timer_dose_ml: 2.0,    // Dose for pure timer trigger', 'fixed_timer_dose_ml: 0.3,    // Dose for pure timer trigger'),
        ('max_single_dose_ml: 8.0,     // Safety ceiling per injection', 'max_single_dose_ml: 5.0,     // Safety ceiling per injection'),
        ('current_stock_ml: 500.0      // Reservoir level', 'current_stock_ml: 250.0      // Reservoir level'),
        ("let config = flow.get('chem_config') || {", "let config = flow.get('chem_config_v2') || {"),
        ("flow.set('chem_config', config);", "flow.set('chem_config_v2', config);")]:
    code = code.replace(old, new)
ctl['func'] = code
for iid, label, vol, tot in [('inject_sim_stage1_vol', 'Simulate Stage 1 Inflow (+0.5 L)', 0.5, 1.0),
                             ('inject_sim_stage3_vol', 'Simulate Stage 3 Inflow (+0.5 L)', 0.5, 1.0)]:
    n = get(f, iid)
    p = json.loads(n['payload'])
    p['volume_added_liters'] = vol
    p['total_tank_volume'] = tot
    n['payload'] = json.dumps(p)
    n['name'] = label
# The 10-minute repeat would inject chemicals into empty tanks or while driving once it
# reaches the real pumps; the mission (MissionCore) doses per batch, so make it manual.
t = get(f, 'inject_timer_trigger')
t['repeat'] = ''
t['name'] = 'Timer dose (manual - the mission doses each batch automatically)'
get(f, 'mqtt_in_telemetry')['topic'] = 'rover/water/+/inflow'
get(f, 'mqtt_out_telemetry')['topic'] = 'rover/dosing/telemetry'
f = [n for n in f if n['id'] not in ('gpio_out_alum', 'gpio_out_chlorine', 'mqtt_out_alum', 'mqtt_out_chlorine')]
put(f, BROKER)
put(f, fn('fn_dose_to_esp32', 'tab_chem_dosing', 'ESP32 metered dose',
          "// The ESP32 times the pump itself (DosingPump core), so only the dose volume is sent -\n"
          "// network delay can no longer change the injected volume.\n"
          "if (msg.state !== 'ON' || !msg.dose_ml) return null;\n"
          "const pump = String(msg.chemical || '').toLowerCase().startsWith('chlor') ? 'chlorine' : 'alum';\n"
          "msg.topic = 'rover/dosing/cmd';\n"
          "msg.payload = JSON.stringify({pump: pump, ml: Number(msg.dose_ml)});\nreturn msg;",
          1, [['mqtt_out_dose']], 1040, 300))
put(f, mqtt_out('mqtt_out_dose', 'tab_chem_dosing', 'rover/dosing/cmd', 1280, 300))
put(f, fn('fn_estop_to_esp32', 'tab_chem_dosing', 'E-stop -> stop pumps',
          "if (msg.payload !== true) return null;\nmsg.payload = 'stop';\nreturn msg;", 1, [['mqtt_out_dose']], 1040, 360))
get(f, 'fn_alum_pulse_timer')['wires'] = [['dbg_alum_out', 'fn_dose_to_esp32']]
get(f, 'fn_chlorine_pulse_timer')['wires'] = [['dbg_chlorine_out', 'fn_dose_to_esp32']]
es = get(f, 'inject_estop')['wires'][0]
if 'fn_estop_to_esp32' not in es:
    es.append('fn_estop_to_esp32')
# dosing control panel (Issue #3 Node-RED role)
put(f, {'id': 'chem_ui_tab', 'type': 'ui_tab', 'name': 'Chemical Dosing', 'icon': 'science', 'order': 7,
        'disabled': False, 'hidden': False})
put(f, {'id': 'chem_ui_group', 'type': 'ui_group', 'name': 'Dosing control panel', 'tab': 'chem_ui_tab', 'order': 1,
        'disp': True, 'width': '6', 'collapse': False, 'className': ''})
put(f, button('chem_btn_alum', 'tab_chem_dosing', 'chem_ui_group', 'Dose alum 3 mL', '{"pump":"alum","ml":3}', 1,
              ['mqtt_out_dose'], 1040, 440, '#8e44ad'))
put(f, button('chem_btn_cl', 'tab_chem_dosing', 'chem_ui_group', 'Dose chlorine 0.6 mL', '{"pump":"chlorine","ml":0.6}',
              2, ['mqtt_out_dose'], 1040, 480, '#16a085'))
put(f, mqtt_in('chem_status_in', 'tab_chem_dosing', 'rover/dosing/status', 820, 540, ['chem_status_fmt']))
put(f, fn('chem_status_fmt', 'tab_chem_dosing', 'Totals (mL)',
          "const s = msg.payload || {};\n"
          "return [{payload: Number(s.alum_total_ml || 0).toFixed(2) + ' mL'}, {payload: Number(s.chlorine_total_ml || 0).toFixed(2) + ' mL'},\n"
          "        {payload: Number(s.alum_stock_ml || 0)}, {payload: Number(s.chlorine_stock_ml || 0)}];",
          4, [['chem_txt_alum'], ['chem_txt_cl'], ['chem_g_alum'], ['chem_g_cl']], 1040, 540))
put(f, text('chem_txt_alum', 'tab_chem_dosing', 'chem_ui_group', 'Total alum injected', 3, 1280, 520))
put(f, text('chem_txt_cl', 'tab_chem_dosing', 'chem_ui_group', 'Total chlorine injected', 4, 1280, 560))
put(f, gauge('chem_g_alum', 'tab_chem_dosing', 'chem_ui_group', 'Alum stock', 'mL', 0, 250, 5, 1280, 600))
put(f, gauge('chem_g_cl', 'tab_chem_dosing', 'chem_ui_group', 'Chlorine stock', 'mL', 0, 250, 6, 1280, 640))
save('chemical_dosing_flow.json', f)

# ------------------------------------------------------------------ #1 river intake
f = load('river_pump_flow.json')
put(f, BROKER)
put(f, {'id': 'intake_ui_tab', 'type': 'ui_tab', 'name': 'Water Intake', 'icon': 'water_drop', 'order': 8,
        'disabled': False, 'hidden': False})
put(f, {'id': 'intake_ui_group', 'type': 'ui_group', 'name': 'Raw water intake', 'tab': 'intake_ui_tab', 'order': 1,
        'disp': True, 'width': '6', 'collapse': False, 'className': ''})
put(f, mqtt_out('intake_cmd_out', 'river_tank_tab', 'rover/intake/cmd', 700, 420))
put(f, button('intake_btn_start', 'river_tank_tab', 'intake_ui_group', 'Manual Water Intake', 'start', 1,
              ['intake_cmd_out'], 160, 400, '#2980b9'))
put(f, button('intake_btn_stop', 'river_tank_tab', 'intake_ui_group', 'Stop intake', 'stop', 2, ['intake_cmd_out'],
              160, 440))
put(f, mqtt_in('intake_status_in', 'river_tank_tab', 'rover/intake/status', 160, 500, ['intake_status_fmt']))
put(f, fn('intake_status_fmt', 'river_tank_tab', 'Fill status',
          "const s = msg.payload || {};\nreturn [{payload: Number(s.level_pct || 0)},\n"
          "        {payload: (s.pump ? 'PUMP ON' : 'pump off') + ' . ' + (s.state || '') + (s.fault ? ' . FAULT ' + s.fault : '')}];",
          2, [['intake_fill_gauge'], ['intake_status_txt']], 420, 500))
put(f, gauge('intake_fill_gauge', 'river_tank_tab', 'intake_ui_group', 'Stage 1 fill', '%', 0, 100, 3, 700, 480))
put(f, text('intake_status_txt', 'river_tank_tab', 'intake_ui_group', 'Intake', 4, 700, 520))
save('river_pump_flow.json', f)

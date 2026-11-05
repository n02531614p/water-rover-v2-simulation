#!/usr/bin/env python3
"""Build the central Water Rover v2 SCADA dashboard flow (Issue #12).

    python3 node-red/build_dashboard.py      ->  node-red/waterdroid_dashboard.json

Import into Node-RED (needs node-red-dashboard). Talks to the ESP32 firmware
(firmware/waterdroid_esp32) or to the Gazebo simulation (status_monitor with
mqtt:=true) over the topic map in docs/TOPICS.md. One tab per team member's
subsystem plus a Mission tab with the system-wide E-STOP and a CSV mission log.
"""
import json
import os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'waterdroid_dashboard.json')
TAB, BROKER = 'wd_tab', 'wd_broker'
nodes = []
_y = {}


def add(n):
    nodes.append(n)
    return n['id']


def pos(col, row_key):
    _y[row_key] = _y.get(row_key, 40) + 40
    return {'x': 160 + col * 230, 'y': _y[row_key]}


def ui_tab(id_, name, icon, order):
    add({'id': id_, 'type': 'ui_tab', 'name': name, 'icon': icon, 'order': order, 'disabled': False, 'hidden': False})


def group(id_, tab, name, order, width=6):
    add({'id': id_, 'type': 'ui_group', 'name': name, 'tab': tab, 'order': order, 'disp': True,
         'width': str(width), 'collapse': False, 'className': ''})


def mqtt_in(id_, topic, wires, row, datatype='json'):
    add({'id': id_, 'type': 'mqtt in', 'z': TAB, 'name': topic, 'topic': topic, 'qos': '0', 'datatype': datatype,
         'broker': BROKER, 'nl': False, 'rap': True, 'rh': 0, 'inputs': 0, **pos(0, row), 'wires': [wires]})


def mqtt_out(id_, topic, row, retain=False):
    add({'id': id_, 'type': 'mqtt out', 'z': TAB, 'name': topic, 'topic': topic, 'qos': '0',
         'retain': 'true' if retain else 'false', 'respTopic': '', 'contentType': '', 'userProps': '',
         'correl': '', 'expiry': '', 'broker': BROKER, **pos(3, row), 'wires': []})


def func(id_, name, code, outputs, wires, row):
    add({'id': id_, 'type': 'function', 'z': TAB, 'name': name, 'func': code, 'outputs': outputs, 'timeout': 0,
         'noerr': 0, 'initialize': '', 'finalize': '', 'libs': [], **pos(1, row), 'wires': wires})


def button(id_, grp, label, payload, ptype, wires, row, order, bg='', width=2, icon='', topic=''):
    add({'id': id_, 'type': 'ui_button', 'z': TAB, 'name': label, 'group': grp, 'order': order, 'width': width,
         'height': 1, 'passthru': False, 'label': label, 'tooltip': '', 'color': '#ffffff' if bg else '',
         'bgcolor': bg, 'className': '', 'icon': icon, 'payload': payload, 'payloadType': ptype,
         'topic': topic, 'topicType': 'str', **pos(0, row), 'wires': [wires]})


def gauge(id_, grp, title, unit, mn, mx, order, row, width=3, seg=('', ''), colors=None):
    add({'id': id_, 'type': 'ui_gauge', 'z': TAB, 'name': title, 'group': grp, 'order': order, 'width': width,
         'height': 3, 'gtype': 'gage', 'title': title, 'label': unit, 'format': '{{value}}', 'min': mn, 'max': mx,
         'colors': colors or ['#ca3838', '#e6a23c', '#52c41a'], 'seg1': seg[0], 'seg2': seg[1], 'diff': False,
         'className': '', **pos(2, row), 'wires': []})


def text(id_, grp, label, order, row, width=6):
    add({'id': id_, 'type': 'ui_text', 'z': TAB, 'group': grp, 'order': order, 'width': width, 'height': 1,
         'name': label, 'label': label, 'format': '{{msg.payload}}', 'layout': 'row-spread', 'className': '',
         **pos(2, row), 'wires': []})


def template(id_, grp, name, html, order, row, width=6, height=4):
    add({'id': id_, 'type': 'ui_template', 'z': TAB, 'group': grp, 'name': name, 'order': order, 'width': width,
         'height': height, 'format': html, 'storeOutMessages': True, 'fwdInMessages': False, 'resendOnRefresh': True,
         'templateScope': 'local', 'className': '', **pos(2, row), 'wires': [[]]})


def numeric(id_, grp, label, mn, mx, step, value_order, row, wires, topic):
    add({'id': id_, 'type': 'ui_numeric', 'z': TAB, 'name': label, 'label': label, 'tooltip': '', 'group': grp,
         'order': value_order, 'width': 3, 'height': 1, 'wrap': False, 'passthru': True, 'topic': topic,
         'topicType': 'str', 'format': '{{value}}', 'min': mn, 'max': mx, 'step': step, 'className': '',
         **pos(0, row), 'wires': [wires]})


def slider(id_, grp, label, mn, mx, order, row, wires):
    add({'id': id_, 'type': 'ui_slider', 'z': TAB, 'name': label, 'label': label, 'tooltip': '', 'group': grp,
         'order': order, 'width': 6, 'height': 1, 'passthru': False, 'outs': 'end', 'topic': 'angle',
         'topicType': 'str', 'min': mn, 'max': mx, 'step': 1, 'className': '', **pos(0, row), 'wires': [wires]})


# ======================================================================= base
add({'id': TAB, 'type': 'tab', 'label': 'Water Rover v2 - Central Dashboard (Issue #12)', 'disabled': False,
     'info': 'Central SCADA for the Water Rover v2. Every subsystem team has its own dashboard tab.\n'
             'Topic map: docs/TOPICS.md. Works with the ESP32 firmware or with the Gazebo sim '
             '(ros2 launch waterdroid_bringup simulation.launch.py mqtt:=true).'})
add({'id': BROKER, 'type': 'mqtt-broker', 'name': 'Rover broker', 'broker': 'localhost', 'port': '1883',
     'clientid': '', 'autoConnect': True, 'usetls': False, 'protocolVersion': '4', 'keepalive': '60',
     'cleansession': True, 'autoUnsubscribe': True, 'birthTopic': '', 'birthQos': '0', 'birthPayload': '',
     'birthMsg': {}, 'closeTopic': '', 'closeQos': '0', 'closePayload': '', 'closeMsg': {}, 'willTopic': '',
     'willQos': '0', 'willPayload': '', 'willMsg': {}, 'userProps': '', 'sessionExpiry': ''})

for i, (tid, name, icon) in enumerate([
        ('wd_t_mission', 'Mission', 'dashboard'), ('wd_t_water', 'Water train', 'opacity'),
        ('wd_t_arm', 'Arm & Delivery', 'pan_tool'), ('wd_t_drive', 'Drive & Map', 'directions_car'),
        ('wd_t_power', 'Power', 'battery_charging_full')]):
    ui_tab(tid, name, icon, i + 1)

group('wd_g_estop', 'wd_t_mission', 'System E-STOP', 1)
group('wd_g_state', 'wd_t_mission', 'Master System State (#11)', 2)
group('wd_g_log', 'wd_t_mission', 'Mission log', 3)
group('wd_g_intake', 'wd_t_water', 'Intake (#1)', 1, 4)
group('wd_g_filter', 'wd_t_water', 'Filtration (#5)', 2, 6)
group('wd_g_dose', 'wd_t_water', 'Chemical dosing (#3)', 3, 6)
group('wd_g_arm', 'wd_t_arm', 'Delivery arm (#6)', 1)
group('wd_g_deliv', 'wd_t_arm', 'Clean water delivery (#8)', 2)
group('wd_g_drive', 'wd_t_drive', 'Teleoperation D-pad (#9)', 1, 4)
group('wd_g_map', 'wd_t_drive', 'Position & waypoints (#7)', 2, 8)
group('wd_g_power', 'wd_t_power', 'Power bank & mode (#10)', 1)

# ======================================================================= E-STOP (#12)
mqtt_out('wd_estop_out', 'rover/estop', 'estop', retain=True)
button('wd_btn_estop', 'wd_g_estop', 'EMERGENCY STOP', '1', 'str', ['wd_estop_out'], 'estop', 1,
       bg='#c0392b', width=6, icon='report')
button('wd_btn_estop_rst', 'wd_g_estop', 'Release E-STOP', '0', 'str', ['wd_estop_out'], 'estop', 2, width=6,
       icon='lock_open')
mqtt_in('wd_estop_in', 'rover/estop', ['wd_estop_fmt'], 'estop', datatype='utf8')
func('wd_estop_fmt', 'E-stop badge',
     "const on = String(msg.payload) === '1';\n"
     "msg.payload = on ? '<div style=\"background:#c0392b;color:#fff;padding:10px;border-radius:8px;text-align:center;font-weight:bold\">E-STOP ACTIVE - all pumps and motors off</div>'\n"
     "                 : '<div style=\"background:#27ae60;color:#fff;padding:10px;border-radius:8px;text-align:center\">System armed</div>';\n"
     "return msg;", 1, [['wd_estop_badge']], 'estop')
template('wd_estop_badge', 'wd_g_estop', 'E-stop state', '<div ng-bind-html="msg.payload"></div>', 3, 'estop', height=1)
mqtt_in('wd_online_in', 'rover/online', ['wd_online_fmt'], 'estop', datatype='utf8')
func('wd_online_fmt', 'Rover online', "msg.payload = String(msg.payload) === '1' ? 'ONLINE' : 'OFFLINE';\nreturn msg;",
     1, [['wd_online_txt']], 'estop')
text('wd_online_txt', 'wd_g_estop', 'Rover link', 4, 'estop')

# ======================================================================= Mission (#11)
mqtt_out('wd_mission_out', 'rover/mission/cmd', 'mission')
for i, (lbl, p, icon) in enumerate([('Start mission', 'start', 'play_arrow'), ('Pause', 'pause', 'pause'),
                                     ('Resume', 'resume', 'play_circle_outline'), ('Abort / home', 'abort', 'home')]):
    button(f'wd_btn_m{i}', 'wd_g_state', lbl, p, 'str', ['wd_mission_out'], 'mission', 3 + i, width=3, icon=icon)
for i, (lbl, p) in enumerate([('Arrived: river', 'arrived_river'), ('Arrived: homestead', 'arrived_home'),
                              ('Arrived: depot', 'arrived_depot')]):
    button(f'wd_btn_a{i}', 'wd_g_state', lbl, p, 'str', ['wd_mission_out'], 'mission', 7 + i, width=2)
mqtt_in('wd_state_in', 'rover/state', ['wd_state_fmt'], 'mission')
func('wd_state_fmt', 'State readout',
     "const s = msg.payload || {};\n"
     "const pretty = String(s.state || 'UNKNOWN');\n"
     "return [{payload: pretty}, {payload: s.state_id}, {payload: s.battery_v}];", 3,
     [['wd_state_txt'], ['wd_stateid_txt'], ['wd_batt_gauge']], 'mission')
text('wd_state_txt', 'wd_g_state', 'State', 1, 'mission')
text('wd_stateid_txt', 'wd_g_state', 'State id (0 idle . 1 river . 2 intake/dose . 3 homestead . 4 deliver . 5 e-stop)', 2, 'mission')
gauge('wd_batt_gauge', 'wd_g_power', '5 V rail', 'V', 3.5, 5.5, 1, 'mission', seg=('4.5', '4.75'))

# mission telemetry log -> CSV
mqtt_in('wd_tel_in', 'rover/telemetry', ['wd_tel_csv', 'wd_tel_last'], 'log')
func('wd_tel_csv', 'Telemetry -> CSV line',
     "const t = msg.payload || {};\n"
     "const cols = ['state','state_id','battery_v','s1','s2','s3','arm','alum_ml','chlorine_ml','delivered_ml','health','estop'];\n"
     "let line = new Date().toISOString() + ',' + cols.map(c => t[c] !== undefined ? (typeof t[c] === 'object' ? JSON.stringify(t[c]).replace(/,/g,';') : t[c]) : '').join(',');\n"
     "if (!context.get('header')) { context.set('header', true); node.send({payload: 'time,' + cols.join(',')}); }\n"
     "msg.payload = line;\nreturn msg;", 1, [['wd_tel_file']], 'log')
add({'id': 'wd_tel_file', 'type': 'file', 'z': TAB, 'name': 'Mission log CSV', 'filename': 'rover_mission_log.csv',
     'filenameType': 'str', 'appendNewline': True, 'createDir': True, 'overwriteFile': 'false', 'encoding': 'none',
     **pos(2, 'log'), 'wires': [[]]})
func('wd_tel_last', 'Last frame',
     "const t = msg.payload || {};\n"
     "msg.payload = `${new Date().toLocaleTimeString()} - ${t.state || ''} - S1 ${t.s1 ?? '-'}% S3 ${t.s3 ?? '-'}% - delivered ${t.delivered_ml ?? 0} mL`;\n"
     "return msg;", 1, [['wd_tel_txt']], 'log')
text('wd_tel_txt', 'wd_g_log', 'Last telemetry', 1, 'log')
add({'id': 'wd_log_note', 'type': 'comment', 'z': TAB, 'name': 'Log file: ~/.node-red/rover_mission_log.csv',
     'info': 'One CSV row per telemetry frame (1 Hz).', **pos(3, 'log'), 'wires': []})

# ======================================================================= Intake (#1)
mqtt_out('wd_intake_out', 'rover/intake/cmd', 'intake')
button('wd_btn_in_start', 'wd_g_intake', 'Manual Water Intake', 'start', 'str', ['wd_intake_out'], 'intake', 1,
       bg='#2980b9', width=4, icon='water_drop')
button('wd_btn_in_stop', 'wd_g_intake', 'Stop', 'stop', 'str', ['wd_intake_out'], 'intake', 2, width=2)
button('wd_btn_in_rst', 'wd_g_intake', 'Reset fault', 'reset', 'str', ['wd_intake_out'], 'intake', 3, width=2)
mqtt_in('wd_intake_in', 'rover/intake/status', ['wd_intake_fmt'], 'intake')
func('wd_intake_fmt', 'Intake status',
     "const s = msg.payload || {};\n"
     "return [{payload: Number(s.level_pct || 0)}, {payload: (s.pump ? 'PUMP ON' : 'pump off') + ' . ' + (s.state || '') + (s.fault ? ' . FAULT ' + s.fault : '')}];",
     2, [['wd_intake_gauge'], ['wd_intake_txt']], 'intake')
gauge('wd_intake_gauge', 'wd_g_intake', 'Stage 1 fill', '%', 0, 100, 4, 'intake', width=4, seg=('20', '90'),
      colors=['#e6a23c', '#3498db', '#2ecc71'])
text('wd_intake_txt', 'wd_g_intake', 'Intake', 5, 'intake', width=4)

# ======================================================================= Filtration (#5)
mqtt_out('wd_filter_out', 'rover/filtration/cmd', 'filter')
mqtt_in('wd_filter_in', 'rover/filtration/status', ['wd_filter_fmt'], 'filter')
func('wd_filter_fmt', 'Filtration status', "const d = msg.payload || {};\n"
     "return [{payload: d.s1}, {payload: d.s2}, {payload: d.s3}, {payload: d}];", 4,
     [['wd_g_s1'], ['wd_g_s2'], ['wd_g_s3'], ['wd_filter_badge']], 'filter')
gauge('wd_g_s1', 'wd_g_filter', 'Stage 1 raw', '%', 0, 100, 1, 'filter', width=2, seg=('10', '90'))
gauge('wd_g_s2', 'wd_g_filter', 'Stage 2 column', '%', 0, 100, 2, 'filter', width=2, seg=('70', '90'),
      colors=['#52c41a', '#e6a23c', '#ca3838'])
gauge('wd_g_s3', 'wd_g_filter', 'Stage 3 clean', '%', 0, 100, 3, 'filter', width=2, seg=('10', '95'))
template('wd_filter_badge', 'wd_g_filter', 'Health badge',
         '<div style="padding:10px;border-radius:10px;text-align:center;font-weight:bold;color:#fff;background:'
         '{{ {HEALTHY:\'#2ecc71\',CLOGGING:\'#e67e22\',OVERFLOW:\'#e74c3c\',SOURCE_LOW:\'#f1c40f\','
         'TANK_FULL:\'#3498db\',MANUAL_PAUSE:\'#7f8c8d\'}[msg.payload.health] || \'#999\' }}">'
         'Filtration: {{msg.payload.health}} . intake {{msg.payload.intake ? \'ON\' : \'off\'}}</div>',
         4, 'filter', height=1)
button('wd_btn_f_pause', 'wd_g_filter', 'Pause intake', 'pause', 'str', ['wd_filter_out'], 'filter', 5, width=3)
button('wd_btn_f_auto', 'wd_g_filter', 'Auto', 'auto', 'str', ['wd_filter_out'], 'filter', 6, width=3)

# ======================================================================= Dosing (#3)
mqtt_out('wd_dose_out', 'rover/dosing/cmd', 'dose')
numeric('wd_num_alum', 'wd_g_dose', 'Alum mL', 0, 10, 0.5, 1, 'dose', ['wd_dose_alum_fmt'], 'alum')
func('wd_dose_alum_fmt', 'store alum mL', "flow.set('alum_ml', Number(msg.payload));\nreturn null;", 1, [[]], 'dose')
button('wd_btn_dose_alum', 'wd_g_dose', 'Dose alum', 'alum', 'str', ['wd_dose_build'], 'dose', 2, width=3,
       bg='#8e44ad')
numeric('wd_num_cl', 'wd_g_dose', 'Chlorine mL', 0, 5, 0.1, 3, 'dose', ['wd_dose_cl_fmt'], 'chlorine')
func('wd_dose_cl_fmt', 'store chlorine mL', "flow.set('chlorine_ml', Number(msg.payload));\nreturn null;", 1,
     [[]], 'dose')
button('wd_btn_dose_cl', 'wd_g_dose', 'Dose chlorine', 'chlorine', 'str', ['wd_dose_build'], 'dose', 4, width=3,
       bg='#16a085')
func('wd_dose_build', 'Build dose command',
     "const pump = msg.payload;\n"
     "const ml = pump === 'alum' ? (flow.get('alum_ml') ?? 3) : (flow.get('chlorine_ml') ?? 0.6);\n"
     "msg.payload = JSON.stringify({pump: pump, ml: ml});\nreturn msg;", 1, [['wd_dose_out']], 'dose')
button('wd_btn_dose_stop', 'wd_g_dose', 'Stop dosing', 'stop', 'str', ['wd_dose_out'], 'dose', 5, width=6)
mqtt_in('wd_dose_in', 'rover/dosing/status', ['wd_dose_fmt'], 'dose')
func('wd_dose_fmt', 'Dosing totals',
     "const s = msg.payload || {};\n"
     "return [{payload: `${Number(s.alum_total_ml||0).toFixed(2)} mL ${s.alum_on ? '(dosing)' : ''}`},\n"
     "        {payload: `${Number(s.chlorine_total_ml||0).toFixed(2)} mL ${s.chlorine_on ? '(dosing)' : ''}`},\n"
     "        {payload: Number(s.alum_stock_ml||0)}, {payload: Number(s.chlorine_stock_ml||0)}];", 4,
     [['wd_alum_tot'], ['wd_cl_tot'], ['wd_alum_stock'], ['wd_cl_stock']], 'dose')
text('wd_alum_tot', 'wd_g_dose', 'Total alum injected', 6, 'dose')
text('wd_cl_tot', 'wd_g_dose', 'Total chlorine injected', 7, 'dose')
gauge('wd_alum_stock', 'wd_g_dose', 'Alum stock', 'mL', 0, 250, 8, 'dose', seg=('25', '75'))
gauge('wd_cl_stock', 'wd_g_dose', 'Chlorine stock', 'mL', 0, 250, 9, 'dose', seg=('25', '75'))

# ======================================================================= Arm (#6)
mqtt_out('wd_arm_out', 'rover/arm/cmd', 'arm')
button('wd_btn_ext', 'wd_g_arm', 'Extend Arm', 'EXTEND', 'str', ['wd_arm_out'], 'arm', 1, width=3, icon='call_made')
button('wd_btn_ret', 'wd_g_arm', 'Retract Arm', 'RETRACT', 'str', ['wd_arm_out'], 'arm', 2, width=3,
       icon='call_received')
slider('wd_arm_slider', 'wd_g_arm', 'Arm angle (0 stow - 90 deploy)', 0, 90, 3, 'arm', ['wd_arm_angle'])
func('wd_arm_angle', 'ANGLE:n', "let a = Math.max(0, Math.min(90, Math.round(Number(msg.payload))));\n"
     "if (!Number.isFinite(a)) return null;\nmsg.payload = 'ANGLE:' + a;\nreturn msg;", 1, [['wd_arm_out']], 'arm')
mqtt_in('wd_arm_in', 'rover/arm/status', ['wd_arm_fmt'], 'arm')
func('wd_arm_fmt', 'Arm status', "const s = msg.payload || {};\n"
     "return [{payload: Number(s.angle||0)}, {payload: s.deployed ? 'DEPLOYED' : (s.stowed ? 'STOWED' : (s.moving ? 'MOVING' : 'HOLD'))}];",
     2, [['wd_arm_gauge'], ['wd_arm_txt']], 'arm')
gauge('wd_arm_gauge', 'wd_g_arm', 'Arm angle', 'deg', 0, 90, 4, 'arm', seg=('30', '80'),
      colors=['#7f8c8d', '#e6a23c', '#2ecc71'])
text('wd_arm_txt', 'wd_g_arm', 'Arm', 5, 'arm', width=3)

# ======================================================================= Delivery (#8)
mqtt_out('wd_deliv_out', 'rover/delivery/cmd', 'deliv')
numeric('wd_num_deliv', 'wd_g_deliv', 'Volume mL', 50, 1000, 50, 1, 'deliv', ['wd_deliv_store'], 'ml')
func('wd_deliv_store', 'store volume', "flow.set('deliv_ml', Number(msg.payload));\nreturn null;", 1, [[]], 'deliv')
button('wd_btn_deliv', 'wd_g_deliv', 'Deliver', 'go', 'str', ['wd_deliv_build'], 'deliv', 2, width=3, bg='#2980b9')
func('wd_deliv_build', 'Build delivery command',
     "msg.payload = JSON.stringify({ml: flow.get('deliv_ml') ?? 300});\nreturn msg;", 1, [['wd_deliv_out']], 'deliv')
button('wd_btn_deliv_stop', 'wd_g_deliv', 'Stop delivery', 'stop', 'str', ['wd_deliv_out'], 'deliv', 3, width=6)
mqtt_in('wd_deliv_in', 'rover/delivery/status', ['wd_deliv_fmt'], 'deliv')
func('wd_deliv_fmt', 'Delivery status', "const s = msg.payload || {};\n"
     "return [{payload: Number(s.clean_ml||0)}, {payload: `${s.deliveries||0} deliveries . ${Number(s.delivered_total_ml||0).toFixed(0)} mL total . ${s.state || (s.pump ? 'PUMPING' : 'idle')}${s.reason ? ' (' + s.reason + ')' : ''}`}];",
     2, [['wd_clean_gauge'], ['wd_deliv_txt']], 'deliv')
gauge('wd_clean_gauge', 'wd_g_deliv', 'Clean water on board', 'mL', 0, 1000, 4, 'deliv', seg=('100', '300'))
text('wd_deliv_txt', 'wd_g_deliv', 'Delivery', 5, 'deliv')
add({'id': 'wd_deliv_note', 'type': 'comment', 'z': TAB, 'name': 'Delivery pump only runs with the arm deployed',
     'info': 'Interlock in firmware (DeliveryController) and in the sim (water_processor).', **pos(3, 'deliv'),
     'wires': []})

# ======================================================================= Drive (#9)
mqtt_out('wd_drive_out', 'rover/drive/cmd', 'drive')
for i, (lbl, p, icon, w) in enumerate([('', 'none', '', 1), ('Forward', 'forward', 'arrow_upward', 2), ('', 'none', '', 1),
                                        ('Left', 'left', 'rotate_left', 1), ('Stop', 'stop', 'stop', 2),
                                        ('Right', 'right', 'rotate_right', 1), ('', 'none', '', 1),
                                        ('Reverse', 'reverse', 'arrow_downward', 2), ('', 'none', '', 1)]):
    if p == 'none':
        add({'id': f'wd_sp{i}', 'type': 'ui_spacer', 'z': TAB, 'name': 'spacer', 'group': 'wd_g_drive',
             'order': i + 1, 'width': w, 'height': 1, 'className': ''})
    else:
        button(f'wd_btn_d{i}', 'wd_g_drive', lbl, p, 'str', ['wd_drive_out'], 'drive', i + 1, width=w, icon=icon,
               bg='#c0392b' if p == 'stop' else '')
add({'id': 'wd_drive_note', 'type': 'comment', 'z': TAB, 'name': 'Each press moves ~0.6 s (dead-man)',
     'info': 'Drive is refused while pumps run or during a mission leg - pause the mission first.',
     **pos(3, 'drive'), 'wires': []})
mqtt_in('wd_drive_in', 'rover/drive/status', ['wd_drive_fmt'], 'drive')
func('wd_drive_fmt', 'Drive status', "const s = msg.payload || {};\n"
     "msg.payload = s.inhibit ? 'INHIBITED (pumps / mission / e-stop)' : (s.left !== undefined ? `L ${s.left}  R ${s.right}` : `speed ${s.speed}`);\nreturn msg;",
     1, [['wd_drive_txt']], 'drive')
text('wd_drive_txt', 'wd_g_drive', 'Motors', 10, 'drive', width=4)

# ======================================================================= Map (#7)
mqtt_in('wd_pose_in', 'rover/nav/pose', ['wd_map'], 'map')
template('wd_map', 'wd_g_map', 'Village map',
         '<svg viewBox="-6 -8 20 16" style="width:100%;height:320px;background:#e8f1e4;border-radius:8px">\n'
         ' <g transform="scale(1,-1)">\n'
         '  <rect x="10" y="-8" width="3" height="16" fill="#3a86ff" opacity="0.6"/>\n'
         '  <rect x="-5" y="-1.5" width="18" height="3" fill="#c9a26b" opacity="0.5"/>\n'
         '  <rect x="-0.4" y="-0.4" width="0.8" height="0.8" fill="#555"/>\n'
         '  <rect x="5.5" y="2.5" width="1" height="1" fill="#e67e22"/><rect x="5.5" y="-3.5" width="1" height="1" fill="#e67e22"/>\n'
         '  <rect x="-3.5" y="-3.5" width="1" height="1" fill="#e67e22"/>\n'
         '  <circle ng-if="msg.payload.target" ng-attr-cx="{{msg.payload.target[0]}}" ng-attr-cy="{{msg.payload.target[1]}}" r="0.35" fill="none" stroke="#c0392b" stroke-width="0.1"/>\n'
         '  <g ng-attr-transform="translate({{msg.payload.x||0}},{{msg.payload.y||0}}) rotate({{(msg.payload.yaw||0)*57.2958}})">\n'
         '   <rect x="-0.3" y="-0.2" width="0.6" height="0.4" fill="#2c3e50"/><polygon points="0.3,0.15 0.5,0 0.3,-0.15" fill="#f1c40f"/>\n'
         '  </g>\n </g>\n</svg>\n'
         '<div style="font-size:12px">x {{msg.payload.x}} m . y {{msg.payload.y}} m . target {{msg.payload.target || "-"}}'
         ' &nbsp; <span style="color:#3a86ff">river</span> . <span style="color:#e67e22">offload bays</span> . depot (0,0)</div>\n'
         '<div style="font-size:11px;color:#888">Live pose comes from the Gazebo sim (rover/nav/pose). On the real rover this needs the Issue #7 GPS/compass.</div>',
         1, 'map', width=8, height=8)

# ======================================================================= Power (#10)
mqtt_out('wd_power_out', 'rover/power/cmd', 'power')
for i, (lbl, p, bg) in enumerate([('Active', 'active', '#27ae60'), ('Sleep', 'sleep', '#7f8c8d'),
                                   ('Emergency shutdown', 'emergency', '#c0392b'), ('Reset', 'reset', '')]):
    button(f'wd_btn_p{i}', 'wd_g_power', lbl, p, 'str', ['wd_power_out'], 'power', 2 + i, width=3, bg=bg)
mqtt_in('wd_power_in', 'rover/power/status', ['wd_power_fmt'], 'power')
func('wd_power_fmt', 'Power status',
     "const s = msg.payload || {};\nconst names = ['Intake','Alum','Chlorine','Delivery'];\n"
     "const ch = (s.ch || []).map((on, i) => names[i] + (on ? ' ON' : ' off')).join(' . ');\n"
     "return [{payload: Number(s.rail_v || 0)}, {payload: `${s.mode || '?'} . rail ${s.rail || '?'}${s.fault ? ' . ' + s.fault : ''}`}, {payload: ch}];",
     3, [['wd_batt_gauge'], ['wd_power_txt'], ['wd_relay_txt']], 'power')
text('wd_power_txt', 'wd_g_power', 'Power mode', 6, 'power')
text('wd_relay_txt', 'wd_g_power', 'Relays', 7, 'power')

with open(OUT, 'w') as f:
    json.dump(nodes, f, indent=2)
print(f'{len(nodes)} nodes -> {OUT}')

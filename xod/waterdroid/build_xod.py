#!/usr/bin/env python3
"""Generate the XOD C++ nodes for Water Rover v2 from the firmware cores.

Each node's patch.cpp = the shared core header (firmware/waterdroid_esp32/src/*.h,
include guards stripped so several nodes can live in one XOD program) + a small
XOD `node {}` wrapper. patch.xodp holds the terminals. Re-run after editing a core:

    python3 xod/waterdroid/build_xod.py

so the XOD nodes, the ESP32 sketch and the host tests always share identical logic.
"""
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', '..', 'firmware', 'waterdroid_esp32', 'src')


def core(name):
    text = open(os.path.join(SRC, name)).read()
    text = re.sub(r'^#ifndef \w+_H\n#define \w+_H\n', '', text, flags=re.M)
    text = re.sub(r'\n#endif\s*(//.*)?\s*$', '\n', text)
    return text


T = {'b': 'boolean', 'n': 'number', 'p': 'pulse', 's': 'string'}
DEFAULT = {'b': 'False', 'n': '0', 'p': 'Never', 's': '""'}


def xodp(desc, inputs, outputs):
    nodes = []
    for i, (label, kind, default, *_d) in enumerate(inputs):
        n = {'id': f'in{i:02d}', 'type': f'xod/patch-nodes/input-{T[kind]}',
             'position': {'x': i * 102, 'y': 0}, 'label': label,
             'boundLiterals': {'__out__': default if default is not None else DEFAULT[kind]}}
        if _d:
            n['description'] = _d[0]
        nodes.append(n)
    nodes.append({'id': 'impl', 'type': 'xod/patch-nodes/not-implemented-in-xod',
                  'position': {'x': 0, 'y': 153}})
    for i, (label, kind, *_d) in enumerate(outputs):
        n = {'id': f'out{i:02d}', 'type': f'xod/patch-nodes/output-{T[kind]}',
             'position': {'x': i * 102, 'y': 306}, 'label': label}
        if _d:
            n['description'] = _d[0]
        nodes.append(n)
    return {'description': desc, 'nodes': nodes}


NODES = {}

# ----------------------------------------------------------------- Issue #1
NODES['intake-controller'] = dict(
    desc='Issue #1 intake pump: timed fill of Stage 1 with motor-burnout cutoff (MAXR s) and dry-run '
         'protection (relay ON but LVL not rising). WANT goes to relay-sequencer REQ1; feed the real relay '
         'state back into RON. ST: 0 idle 1 pumping 2 full 3 blocked 4 DRY_RUN 5 TIMEOUT.',
    inputs=[('STRT', 'p', None, 'Manual Water Intake / mission start'), ('STOP', 'p', None),
            ('RST', 'p', None, 'clear a DRY_RUN / TIMEOUT fault'), ('LVL', 'n', '0', 'Stage 1 level 0-100 %'),
            ('BLK', 'b', 'False', 'block: column overflow / e-stop'), ('RON', 'b', 'False', 'relay CH1 actually ON'),
            ('MAXR', 'n', '90', 'max run seconds')],
    outputs=[('WANT', 'b'), ('ST', 'n'), ('RUN', 'n', 'run time s'), ('FLT', 'b', 'dry-run or timeout fault')],
    cpp=core('pump_cores.h') + '''
node {
    IntakeController c;
    void evaluate(Context ctx) {
        uint32_t now = millis();
        c.cfg.maxRunMs = (uint32_t)(getValue<input_MAXR>(ctx) * 1000);
        if (isInputDirty<input_RST>(ctx)) c.clearFault();
        if (isInputDirty<input_STOP>(ctx)) c.stop();
        if (isInputDirty<input_STRT>(ctx)) c.start();
        c.tick(now, getValue<input_RON>(ctx), getValue<input_LVL>(ctx), getValue<input_BLK>(ctx));
        emitValue<output_WANT>(ctx, c.want);
        emitValue<output_ST>(ctx, (Number)c.state);
        emitValue<output_RUN>(ctx, c.runMs / 1000.0);
        emitValue<output_FLT>(ctx, c.state == IN_FAULT_DRY || c.state == IN_FAULT_TIMEOUT);
        setTimeout(ctx, 20);
    }
}
''')

# ----------------------------------------------------------------- Issue #3
NODES['dosing-pump'] = dict(
    desc='Issue #3 volumetric pump: the chlorine dosing pump, and also the TRANSFER pump (column -> delivery tank, RATE ~8). DOSE moves ML millilitres by running the '
         'pump for ML / RATE seconds (2 s = 2 mL at 1 mL/s). Only real relay-ON time (RON) counts. '
         'TOT = total injected mL for the dashboard.',
    inputs=[('DOSE', 'p', None), ('ML', 'n', '2'), ('ABRT', 'p', None), ('RATE', 'n', '1', 'pump mL/s (calibrate)'),
            ('MAX', 'n', '10', 'max mL per dose'), ('RON', 'b', 'False', 'relay actually ON')],
    outputs=[('WANT', 'b'), ('BUSY', 'b'), ('TOT', 'n', 'total mL injected'), ('DONE', 'p')],
    cpp=core('pump_cores.h') + '''
node {
    DosingPump p;
    bool wasBusy = false;
    void evaluate(Context ctx) {
        uint32_t now = millis();
        p.rateMlPerS = getValue<input_RATE>(ctx);
        p.maxDoseMl = getValue<input_MAX>(ctx);
        if (isInputDirty<input_ABRT>(ctx)) p.abort();
        if (isInputDirty<input_DOSE>(ctx)) p.start(getValue<input_ML>(ctx));
        p.tick(now, getValue<input_RON>(ctx));
        emitValue<output_WANT>(ctx, p.want);
        emitValue<output_BUSY>(ctx, p.busy());
        emitValue<output_TOT>(ctx, p.totalMl);
        if (wasBusy && !p.busy()) emitValue<output_DONE>(ctx, 1);
        wasBusy = p.busy();
        setTimeout(ctx, 20);
    }
}
''')

# ----------------------------------------------------------------- Issue #5
NODES['level-monitor'] = dict(
    desc='Issue #5 Stage 2 filtration column monitor. S1/S2/S3 = tank levels in % (multiply the 0-1 output '
         'of xod/common-hardware/analog-sensor by 100 after calibration). ALLW=false pauses the intake '
         '(column overflow with 90/50 % hysteresis, Stage 3 full, manual pause). HLTH: 0 HEALTHY 1 CLOGGING '
         '2 OVERFLOW 3 SOURCE_LOW 4 TANK_FULL 5 MANUAL_PAUSE.',
    inputs=[('S1', 'n', '0'), ('S2', 'n', '0'), ('S3', 'n', '0'), ('INON', 'b', 'False', 'intake pump running'),
            ('PAUS', 'b', 'False', 'manual pause from dashboard')],
    outputs=[('ALLW', 'b'), ('HLTH', 'n'), ('OVF', 'b'), ('CLOG', 'b'), ('FLOW', 'n', 'Stage 3 rise %/s')],
    cpp=core('motion_cores.h') + '''
node {
    FiltrationMonitor f;
    void evaluate(Context ctx) {
        f.manualPause = getValue<input_PAUS>(ctx);
        f.tick(millis(), getValue<input_S1>(ctx), getValue<input_S2>(ctx), getValue<input_S3>(ctx), getValue<input_INON>(ctx));
        emitValue<output_ALLW>(ctx, f.allowIntake);
        emitValue<output_HLTH>(ctx, (Number)f.health);
        emitValue<output_OVF>(ctx, f.overflow);
        emitValue<output_CLOG>(ctx, f.clogged);
        emitValue<output_FLOW>(ctx, f.flowPctPerS);
        setTimeout(ctx, 100);
    }
}
''')

# ----------------------------------------------------------------- Issue #6
NODES['servo-arm'] = dict(
    desc='Issue #6 SG90 servo INTAKE boom with smooth interpolation (1 deg / 20 ms). Raised = 0 deg, in the water = 90 deg. '
         'Connect VAL (0-1 = 0-180 deg) to xod-dev/servo/rotate-servo (or ANG in degrees). '
         'FRZ = e-stop: hold position. DEP -> mission DEP and NOT DEP -> intake BLK; NOT STW -> drive INH.',
    inputs=[('EXT', 'p', None, 'Lower intake'), ('RET', 'p', None, 'Raise intake'), ('ANG', 'n', '45', 'slider 0-90'),
            ('SET', 'p', None, 'go to ANG'), ('FRZ', 'b', 'False')],
    outputs=[('ANGO', 'n', 'current angle deg'), ('VAL', 'n', 'servo value 0-1'), ('DEP', 'b'), ('STW', 'b')],
    cpp=core('motion_cores.h') + '''
node {
    ServoArm a;
    void evaluate(Context ctx) {
        a.freeze(getValue<input_FRZ>(ctx));
        if (isInputDirty<input_EXT>(ctx)) a.command("EXTEND");
        if (isInputDirty<input_RET>(ctx)) a.command("RETRACT");
        if (isInputDirty<input_SET>(ctx)) a.setTarget((int)getValue<input_ANG>(ctx));
        a.tick(millis());
        emitValue<output_ANGO>(ctx, a.angle);
        emitValue<output_VAL>(ctx, a.angle / 180.0);
        emitValue<output_DEP>(ctx, a.deployed());
        emitValue<output_STW>(ctx, a.stowed());
        setTimeout(ctx, 20);
    }
}
''')

# ----------------------------------------------------------------- Issue #8
NODES['delivery-controller'] = dict(
    desc='Issue #8 delivery pump (Stage 3 -> fixed nozzle -> homestead container). REQ starts an ML delivery that '
         'only pumps while HOME (arrived at homestead) is true; leave ARM = true (the nozzle is fixed). '
         'Stops at the volume or when Stage 3 (S3 %) is nearly empty. '
         'ST: 0 idle 1 waiting for arm 2 pumping 3 done 4 aborted.',
    inputs=[('REQ', 'p', None), ('ML', 'n', '120'), ('ABRT', 'p', None), ('HOME', 'b', 'False'),
            ('ARM', 'b', 'True', 'fixed nozzle: leave true'), ('S3', 'n', '0'), ('FLOW', 'n', '25', 'pump mL/s (calibrate)'),
            ('RON', 'b', 'False', 'relay CH4 actually ON')],
    outputs=[('WANT', 'b'), ('ST', 'n'), ('DLV', 'n', 'mL this delivery'), ('TOT', 'n'), ('DONE', 'p')],
    cpp=core('pump_cores.h') + '''
node {
    DeliveryController d;
    uint8_t last = 0;
    void evaluate(Context ctx) {
        d.flowMlPerS = getValue<input_FLOW>(ctx);
        if (isInputDirty<input_ABRT>(ctx)) d.abort("operator");
        if (isInputDirty<input_REQ>(ctx)) d.request(getValue<input_ML>(ctx));
        d.tick(millis(), getValue<input_RON>(ctx), getValue<input_HOME>(ctx), getValue<input_ARM>(ctx), getValue<input_S3>(ctx));
        emitValue<output_WANT>(ctx, d.want);
        emitValue<output_ST>(ctx, (Number)d.state);
        emitValue<output_DLV>(ctx, d.deliveredMl);
        emitValue<output_TOT>(ctx, d.totalMl);
        if (d.state == DL_DONE && last != DL_DONE) emitValue<output_DONE>(ctx, 1);
        last = d.state;
        setTimeout(ctx, 20);
    }
}
''')

# ----------------------------------------------------------------- Issue #9
NODES['l298n-drive'] = dict(
    desc='Issue #9 4WD skid-steer drive on the L298N (left motor pair = channel A, right pair = B). '
         'L/R = -1..1 per side, latched on UPD (dashboard D-pad: forward 1/1, reverse -1/-1, left -1/1, '
         'right 1/-1, stop 0/0). Slew-limited, minimum duty, 0.6 s dead-man. INH = inhibit (pumps / e-stop). '
         'IN1-IN4 -> digital-write, ENA/ENB (0-1) -> pwm-write.',
    inputs=[('L', 'n', '0'), ('R', 'n', '0'), ('UPD', 'p', None), ('INH', 'b', 'False'),
            ('TOUT', 'n', '0.6', 'dead-man seconds (0 = off)')],
    outputs=[('IN1', 'b'), ('IN2', 'b'), ('IN3', 'b'), ('IN4', 'b'), ('ENA', 'n'), ('ENB', 'n')],
    cpp=core('motion_cores.h') + '''
node {
    DriveController d;
    void evaluate(Context ctx) {
        uint32_t now = millis();
        Number t = getValue<input_TOUT>(ctx);
        d.timeoutMs = t > 0 ? (uint32_t)(t * 1000) : 0xFFFFFFFF;
        d.inhibit = getValue<input_INH>(ctx);
        if (isInputDirty<input_UPD>(ctx))
            d.set((int)(getValue<input_L>(ctx) * 255), (int)(getValue<input_R>(ctx) * 255), now);
        d.tick(now);
        emitValue<output_IN1>(ctx, d.out.in1);
        emitValue<output_IN2>(ctx, d.out.in2);
        emitValue<output_IN3>(ctx, d.out.in3);
        emitValue<output_IN4>(ctx, d.out.in4);
        emitValue<output_ENA>(ctx, d.out.ena / 255.0);
        emitValue<output_ENB>(ctx, d.out.enb / 255.0);
        setTimeout(ctx, 20);
    }
}
''')

# ----------------------------------------------------------------- Issue #11
NODES['mission-fsm'] = dict(
    desc='Issue #11 central state machine: Navigating -> Pumping -> Dosing -> Delivering. Arrival pulses come '
         'from navigation (Issue #7), the ROS 2 sim or dashboard buttons. Outputs one-shot pulses that drive '
         'the subsystem nodes (TRF/TRML -> a dosing-pump node used as the transfer pump), and DRV=false inhibits the motors while pumps run. SID matches the Node-RED '
         'serial bridge: 0 IDLE 1 NAV_RIVER 2 INTAKE_DOSE 3 NAV_HOMESTEAD 4 DELIVER_WATER 5 EMERGENCY_STOP. '
         'ST = detailed state 0-10 (STATE_IDLE ... STATE_EMERGENCY_STOP).',
    inputs=[('STRT', 'p', None), ('PAUS', 'p', None), ('RSUM', 'p', None), ('ABRT', 'p', None),
            ('ESTP', 'b', 'False'), ('ARIV', 'p', None, 'arrived at river'), ('AHOM', 'p', None, 'arrived at homestead'),
            ('ADEP', 'p', None, 'arrived at depot'), ('INB', 'b', 'False', 'intake busy'), ('TRB', 'b', 'False', 'transfer pump busy'),
            ('CLB', 'b', 'False', 'chlorine busy'), ('S1', 'n', '0'), ('S2', 'n', '0'), ('COL', 'n', '0', 'column mL (volume-model)'), ('DEP', 'b', 'False', 'intake lowered'),
            ('STW', 'b', 'True', 'intake raised'), ('DLB', 'b', 'False', 'delivery busy'), ('CML', 'n', '0', 'Stage 3 mL'),
            ('HOMS', 'n', '3', 'homesteads per trip'), ('CNT', 'n', '1800', 'chlorine contact s')],
    outputs=[('ST', 'n'), ('SID', 'n'), ('DRV', 'b', 'motors allowed'), ('INTK', 'p'), ('TRF', 'p', 'run transfer pump'), ('TRML', 'n'),
             ('CHLR', 'p'), ('CLML', 'n'), ('EXT', 'p'), ('RET', 'p'), ('DLV', 'p'), ('DLML', 'n'), ('STOP', 'p')],
    cpp=core('mission_core.h') + '''
node {
    MissionCore m;
    void evaluate(Context ctx) {
        uint32_t now = millis();
        m.cfg.homesteads = (uint8_t)getValue<input_HOMS>(ctx);
        m.cfg.contactMs = (uint32_t)(getValue<input_CNT>(ctx) * 1000);
        m.setEstop(getValue<input_ESTP>(ctx), now);
        if (isInputDirty<input_STRT>(ctx)) m.start(now);
        if (isInputDirty<input_PAUS>(ctx)) m.pause(now);
        if (isInputDirty<input_RSUM>(ctx)) m.resume(now);
        if (isInputDirty<input_ABRT>(ctx)) m.abort(now);
        if (isInputDirty<input_ARIV>(ctx)) m.arrivedRiver(now);
        if (isInputDirty<input_AHOM>(ctx)) m.arrivedHomestead(now);
        if (isInputDirty<input_ADEP>(ctx)) m.arrivedDepot(now);
        MissionInputs in;
        in.intakeBusy = getValue<input_INB>(ctx);
        in.transferBusy = getValue<input_TRB>(ctx);
        in.columnMl = getValue<input_COL>(ctx);
        in.chlorineBusy = getValue<input_CLB>(ctx);
        in.s1Pct = getValue<input_S1>(ctx);
        in.s2Pct = getValue<input_S2>(ctx);
        in.armDeployed = getValue<input_DEP>(ctx);
        in.armStowed = getValue<input_STW>(ctx);
        in.deliveryBusy = getValue<input_DLB>(ctx);
        in.cleanMl = getValue<input_CML>(ctx);
        m.tick(now, in);
        MissionActions& a = m.act;
        if (a.stopAll) emitValue<output_STOP>(ctx, 1);
        if (a.startIntake) emitValue<output_INTK>(ctx, 1);
        if (a.transferMl > 0) { emitValue<output_TRML>(ctx, a.transferMl); emitValue<output_TRF>(ctx, 1); }
        if (a.doseChlorineMl > 0) { emitValue<output_CLML>(ctx, a.doseChlorineMl); emitValue<output_CHLR>(ctx, 1); }
        if (a.armExtend) emitValue<output_EXT>(ctx, 1);
        if (a.armRetract) emitValue<output_RET>(ctx, 1);
        if (a.deliverMl > 0) { emitValue<output_DLML>(ctx, a.deliverMl); emitValue<output_DLV>(ctx, 1); }
        a.clear();
        emitValue<output_ST>(ctx, (Number)m.state);
        emitValue<output_SID>(ctx, m.stateId());
        emitValue<output_DRV>(ctx, m.driveAllowed);
        setTimeout(ctx, 20);
    }
}
''')

# ----------------------------------------------------------------- Issue #12
NODES['telemetry-json'] = dict(
    desc='Issue #12 formats the rover telemetry into one JSON frame (same keys as the ESP32 sketch publishes '
         'on rover/telemetry). Send OUT over xod-dev/esp32 MQTT/HTTP or a serial line to Node-RED on every UPD.',
    inputs=[('UPD', 'p', 'Continuously'), ('SID', 'n', '0'), ('BATT', 'n', '5'), ('S1', 'n', '0'), ('S2', 'n', '0'),
            ('S3', 'n', '0'), ('ARM', 'n', '0'), ('TRF', 'n', '0', 'transferred mL'), ('CHLR', 'n', '0'), ('DLV', 'n', '0'),
            ('HLTH', 'n', '0'), ('ESTP', 'b', 'False')],
    outputs=[('OUT', 's'), ('SENT', 'p')],
    cpp='''// Issue #12 - telemetry JSON frame for Node-RED (keys match rover/telemetry from waterdroid_esp32.ino)
#include <stdio.h>

static const char* const TJ_HEALTH[] = {"HEALTHY", "CLOGGING", "OVERFLOW", "SOURCE_LOW", "TANK_FULL", "MANUAL_PAUSE"};

node {
    char buf[256];
    CStringView view = CStringView(buf);
    void evaluate(Context ctx) {
        if (!isInputDirty<input_UPD>(ctx)) return;
        int h = (int)getValue<input_HLTH>(ctx);
        if (h < 0 || h > 5) h = 0;
        snprintf(buf, sizeof(buf),
                 "{\\"state_id\\":%d,\\"battery_v\\":%.2f,\\"s1\\":%.1f,\\"s2\\":%.1f,\\"s3\\":%.1f,\\"arm\\":%d,"
                 "\\"transfer_ml\\":%.0f,\\"chlorine_ml\\":%.2f,\\"delivered_ml\\":%.0f,\\"health\\":\\"%s\\",\\"estop\\":%s}",
                 (int)getValue<input_SID>(ctx), getValue<input_BATT>(ctx), getValue<input_S1>(ctx),
                 getValue<input_S2>(ctx), getValue<input_S3>(ctx), (int)getValue<input_ARM>(ctx),
                 getValue<input_TRF>(ctx), getValue<input_CHLR>(ctx), getValue<input_DLV>(ctx),
                 TJ_HEALTH[h], getValue<input_ESTP>(ctx) ? "true" : "false");
        emitValue<output_OUT>(ctx, XString(&view));
        emitValue<output_SENT>(ctx, 1);
    }
}
''')

# ----------------------------------------------------------------- flow sensor (#1 / #5)
NODES['flow-meter'] = dict(
    desc='Water-flow sensor YF-S401 (hall turbine, ~5880 pulses/L, 0.3-6 L/min) on the intake line. '
         'Counts pulses on PORT by interrupt. TOT = mL since boot, BAT = mL since RST (use as the intake '
         'fill), RATE = mL/s, DRY = no pulse for DRYT seconds (pump on + DRY = dry run / blocked hose).',
    inputs=[('PORT', 'n', '34'), ('PPL', 'n', '5880', 'pulses per litre: YF-S401 ~5880 (calibrate with 1 L)'),
            ('RST', 'p', None, 'reset the batch counter'), ('DRYT', 'n', '4')],
    outputs=[('TOT', 'n'), ('BAT', 'n'), ('RATE', 'n'), ('DRY', 'b')],
    cpp=core('nav_cores.h') + '''
static volatile uint32_t wd_flowPulses = 0;
static void wd_onFlowPulse() { wd_flowPulses++; }

node {
    FlowMeter f;
    bool attached = false;
    void evaluate(Context ctx) {
        if (!attached) {
            int port = (int)getValue<input_PORT>(ctx);
            pinMode(port, INPUT);
            attachInterrupt(digitalPinToInterrupt(port), wd_onFlowPulse, FALLING);
            attached = true;
        }
        f.pulsesPerLitre = getValue<input_PPL>(ctx);
        if (isInputDirty<input_RST>(ctx)) f.resetBatch();
        uint32_t now = millis();
        f.tick(now, wd_flowPulses);
        emitValue<output_TOT>(ctx, f.totalMl);
        emitValue<output_BAT>(ctx, f.batchMl);
        emitValue<output_RATE>(ctx, f.rateMlPerS);
        emitValue<output_DRY>(ctx, f.noFlowFor(now, (uint32_t)(getValue<input_DRYT>(ctx) * 1000)));
        setTimeout(ctx, 50);
    }
}
''')

# ----------------------------------------------------------------- volume model (#5 / #8)
NODES['volume-model'] = dict(
    desc='Estimated tank volumes without level sensors: IN = intake mL (flow-meter TOT), TR = transfer-pump mL (column -> delivery tank), '
         'CL = chlorine mL, OUT = delivered mL (all running totals). Gravity flow Stage 1 -> column -> Stage 3 '
         'at FILT mL/s (calibrate: time 500 mL through the column). S1/S2/S3 in % feed the level-monitor '
         'and mission-fsm; COL = column mL -> mission COL; FLTR = water still filtering.',
    inputs=[('IN', 'n', '0'), ('TR', 'n', '0', 'transfer pump total mL'), ('CL', 'n', '0'), ('OUT', 'n', '0'), ('FILT', 'n', '3'),
            ('RST', 'p', None, 'tanks emptied by hand')],
    outputs=[('S1', 'n'), ('S2', 'n'), ('S3', 'n'), ('CML', 'n', 'Stage 3 mL'), ('COL', 'n', 'column mL'), ('FLTR', 'b')],
    cpp=core('nav_cores.h') + '''
node {
    VolumeModel v;
    Number li = 0, lt = 0, lc = 0, lo = 0;
    bool first = true;
    void evaluate(Context ctx) {
        Number i = getValue<input_IN>(ctx), tr = getValue<input_TR>(ctx), c = getValue<input_CL>(ctx), o = getValue<input_OUT>(ctx);
        if (first || isInputDirty<input_RST>(ctx)) { v.reset(); li = i; lt = tr; lc = c; lo = o; first = false; }
        v.filterMlPerS = getValue<input_FILT>(ctx);
        v.drainMlPerS = 0;
        v.addRaw(i - li); v.moveColumnToClean(tr - lt); v.addClean(c - lc); v.removeClean(o - lo);
        li = i; lt = tr; lc = c; lo = o;
        v.tick(millis());
        emitValue<output_S1>(ctx, v.rawPct());
        emitValue<output_S2>(ctx, v.columnPct());
        emitValue<output_S3>(ctx, v.cleanPct());
        emitValue<output_CML>(ctx, v.cleanMl);
        emitValue<output_COL>(ctx, v.columnMl);
        emitValue<output_FLTR>(ctx, v.filtering());
        setTimeout(ctx, 100);
    }
}
''')

# ----------------------------------------------------------------- fixed route (#7)
NODES['route-navigator'] = dict(
    desc='Issue #7 without GPS/compass: follows the fixed route in nav_cores.h (river, reverse, 3 homesteads, '
         'depot) by dead reckoning. GO = mission travelling (mission-fsm DRV and state is en-route). Pivot to '
         'face the next point, then drive straight. L/R (-1..1) go to l298n-drive with UPD every evaluation. '
         'Pulses RIV / HOM / DEP go to mission-fsm ARIV / AHOM / ADEP. SPD = m/s and TRN = deg/s at the drive '
         'duties (calibrate on your ground). EXEC = 1 when the drive is really moving (0 while inhibited).',
    inputs=[('GO', 'b', 'False'), ('EXEC', 'n', '1'), ('SPD', 'n', '0.3'), ('TRN', 'n', '90'),
            ('RST', 'p', None, 'at the depot facing the river'), ('SRIV', 'p', None, 'operator: at river'),
            ('SHOM', 'p', None, 'operator: at homestead'), ('SDEP', 'p', None, 'operator: at depot')],
    outputs=[('L', 'n'), ('R', 'n'), ('UPD', 'p'), ('RIV', 'p'), ('HOM', 'p'), ('DEP', 'p'),
             ('X', 'n'), ('Y', 'n'), ('YAW', 'n')],
    cpp=core('nav_cores.h') + '''
node {
    RouteNavigator n;
    void evaluate(Context ctx) {
        n.speedMps = getValue<input_SPD>(ctx);
        n.turnDegPerS = getValue<input_TRN>(ctx);
        if (isInputDirty<input_RST>(ctx)) n.reset();
        if (isInputDirty<input_SRIV>(ctx)) n.syncTo(RA_RIVER);
        if (isInputDirty<input_SHOM>(ctx)) n.syncTo(RA_HOME);
        if (isInputDirty<input_SDEP>(ctx)) n.syncTo(RA_DEPOT);
        n.tick(millis(), getValue<input_EXEC>(ctx));
        if (getValue<input_GO>(ctx)) n.go(); else n.halt();
        if (n.event == RA_RIVER) emitValue<output_RIV>(ctx, 1);
        if (n.event == RA_HOME) emitValue<output_HOM>(ctx, 1);
        if (n.event == RA_DEPOT) emitValue<output_DEP>(ctx, 1);
        n.event = RA_NONE;
        Number l = 0, r = 0;
        if (n.cmd == 1) { l = r = 0.784; } else if (n.cmd == -1) { l = r = -0.784; }   // = drive cruise 200/255
        else if (n.cmd == 2) { l = -0.67; r = 0.67; } else if (n.cmd == -2) { l = 0.67; r = -0.67; }
        emitValue<output_L>(ctx, l);
        emitValue<output_R>(ctx, r);
        emitValue<output_UPD>(ctx, 1);
        emitValue<output_X>(ctx, n.x);
        emitValue<output_Y>(ctx, n.y);
        emitValue<output_YAW>(ctx, n.yaw);
        setTimeout(ctx, 20);
    }
}
''')


def main():
    for name, spec in NODES.items():
        d = os.path.join(HERE, name)
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, 'patch.xodp'), 'w') as f:
            json.dump(xodp(spec['desc'], spec['inputs'], spec['outputs']), f, indent=2)
            f.write('\n')
        header = (f'// {name} - generated by xod/waterdroid/build_xod.py from firmware/waterdroid_esp32/src.\n'
                  '// Do not edit by hand: edit the core header and re-run the generator.\n\n')
        with open(os.path.join(d, 'patch.cpp'), 'w') as f:
            f.write(header + spec['cpp'])
    with open(os.path.join(HERE, 'project.xod'), 'w') as f:
        json.dump({'authors': ['Water Rover v2 team (NUST)'],
                   'description': 'XOD nodes for every Water Rover v2 subsystem (Issues #1 #3 #5 #6 #8 #9 #11 #12). '
                                  'Power sequencing (#10) lives in power-management/xod. Logic is shared with '
                                  'firmware/waterdroid_esp32.',
                   'license': 'MIT', 'name': 'waterdroid', 'version': '0.1.0'}, f, indent=2)
        f.write('\n')
    print(f'{len(NODES)} XOD nodes written to {HERE}')


if __name__ == '__main__':
    main()

// Route status: ESP32 (or simulator) status JSON -> dashboard widgets
let s = msg.payload;
if (typeof s === 'string') {
    try { s = JSON.parse(s); } catch (e) { node.warn('rover/power/status: bad JSON'); return null; }
}
if (!s || typeof s !== 'object' || s.rail_v === undefined) return null;

const NAMES = ['CH1 Intake pump', 'CH2 Left motor', 'CH3 Right motor', 'CH4 Aux pump'];
const v = Number(s.rail_v);
const prev = context.get('prev') || {};
context.set('prev', { rail: s.rail, mode: s.mode, brownouts: s.brownouts });

const gauge  = { payload: Math.round(v * 100) / 100 };
const chart  = { payload: v, topic: '5 V rail' };
const rail   = { payload: s.rail + '  ·  ' + v.toFixed(2) + ' V' };
const mode   = { payload: s.mode + (s.fault ? '  —  ' + s.fault : '') };

let rows = '';
for (let i = 0; i < 4; i++) {
    const on = s.ch && s.ch[i], want = s.want && s.want[i];
    let label = 'OFF', colour = '#8a8f98';
    if (on) { label = 'ON'; colour = '#28a745'; }
    else if (want && s.mode === 'ACTIVE') { label = 'WAITING'; colour = '#e6a23c'; }
    else if (want) { label = 'BLOCKED (' + s.mode + ')'; colour = '#dc3545'; }
    rows += '<tr><td style="padding:3px 0">' + NAMES[i] + '</td><td style="text-align:right">' +
            '<span style="background:' + colour + ';color:#fff;border-radius:10px;padding:2px 10px;font-size:12px">' +
            label + '</span></td></tr>';
}
const table = { payload: '<table style="width:100%">' + rows + '</table>' };

let toast = null;
if ((s.rail === 'SAG' || s.rail === 'CRITICAL') && s.rail !== prev.rail) {
    toast = { topic: '5 V rail ' + s.rail, payload: 'Rail voltage ' + v.toFixed(2) + ' V' };
} else if (s.mode === 'EMERGENCY' && prev.mode !== 'EMERGENCY') {
    toast = { topic: 'EMERGENCY SHUTDOWN', payload: (s.fault || 'all loads off') };
} else if (s.brownouts > (prev.brownouts || 0)) {
    toast = { topic: 'Brownout guard', payload: 'Load shed (' + s.brownouts + ' total)' };
}

return [gauge, chart, rail, mode, table, toast, { payload: s }, { payload: 'ONLINE' }, { payload: 'ping' }];

// Simulated ESP32 power manager (no hardware needed). Same rules as power_core.h.
const NOW = Date.now();
let S = context.get('S');
if (!S) {
    S = { enabled: false, mode: 'SLEEP', latched: false, want: [0, 0, 0, 0], on: [0, 0, 0, 0],
          lastStart: null, offAt: [null, null, null, null], lockoutAt: null, lowSince: null,
          critSince: null, okSince: null, streak: 0, brown: 0, fault: '', sagUntil: 0,
          t0: NOW, lastPub: 0, lastSig: '', railName: 'OK', pending: 0, justShed: false };
}
const C = { stagger: 600, minOff: 1000, shedDebounce: 150, backoff: 5000, critHold: 1000, decay: 30000,
            settle: 1000, maxBrown: 3, startMin: 4.75, shed: 4.50, crit: 4.20, rank: [2, 0, 1, 3] };

function switchOff(i) { if (S.on[i]) { S.on[i] = 0; S.offAt[i] = NOW; } }
function emergency(why) { S.latched = true; S.mode = 'EMERGENCY'; S.fault = why; for (let i = 0; i < 4; i++) switchOff(i); S.pending = 0; }
function setMode(m) {
    if (m === 'EMERGENCY') { emergency('operator emergency shutdown'); return; }
    if (S.latched) return;
    if (m === 'ACTIVE' && S.mode !== 'ACTIVE') S.streak = 0;
    S.mode = m;
}
function reset() { if (!S.latched) return; S.justShed = false; S.latched = false; S.mode = 'SLEEP'; S.fault = ''; S.streak = 0; S.lockoutAt = null; S.lowSince = S.critSince = null; }
function shedOne() {
    let victim = -1;
    for (let i = 0; i < 4; i++) if (S.on[i] && (victim < 0 || C.rank[i] > C.rank[victim])) victim = i;
    if (victim < 0) return false;
    switchOff(victim); return true;
}

// ---------- commands ----------
const topic = msg.topic || '';
if (topic === 'sim/enable') {
    S.enabled = !!msg.payload;
    if (S.enabled) { S.mode = 'SLEEP'; S.latched = false; S.on = [0, 0, 0, 0]; S.want = [0, 0, 0, 0]; S.fault = ''; S.brown = 0; S.streak = 0; S.t0 = NOW; }
    context.set('S', S); return null;
}
if (!S.enabled) return null;
if (topic === 'rover/power/cmd') {
    const c = String(msg.payload).toLowerCase();
    if (c === 'active') setMode('ACTIVE'); else if (c === 'sleep') setMode('SLEEP');
    else if (c === 'emergency' || c === 'estop') setMode('EMERGENCY'); else if (c === 'reset') reset();
} else if (topic.indexOf('rover/power/req/') === 0) {
    const ch = parseInt(topic.split('/').pop(), 10);
    if (ch >= 1 && ch <= 4) S.want[ch - 1] = (String(msg.payload) === '1' || msg.payload === true) ? 1 : 0;
} else if (topic === 'sim/sag') {
    S.sagUntil = NOW + 1500;
}
if (topic !== 'tick') { context.set('S', S); return null; }

// ---------- rail model ----------
const nOn = S.on.reduce((a, b) => a + b, 0);
let v = 5.05 - 0.04 * nOn + (Math.random() - 0.5) * 0.02;
if (S.lastStart !== null && NOW - S.lastStart < 400) v -= 0.22;      // inrush dip
if (NOW < S.sagUntil) v = 4.35 + (Math.random() - 0.5) * 0.02;       // injected brownout
S.railName = v < C.crit ? 'CRITICAL' : (v < C.shed ? 'SAG' : 'OK');

// ---------- sequencer (port of PowerSequencer::tick) ----------
if (v < C.crit) {
    if (S.critSince === null) S.critSince = NOW;
    else if (NOW - S.critSince >= C.critHold && S.mode !== 'EMERGENCY') emergency('rail critical (< criticalV)');
} else S.critSince = null;

if (v < C.shed) {
    S.okSince = null;
    if (S.lowSince === null) S.lowSince = NOW;
    else if (NOW - S.lowSince >= (S.justShed ? C.settle : C.shedDebounce)) {
        S.lowSince = NOW;
        if (S.mode === 'ACTIVE' && shedOne()) {
            S.justShed = true; S.brown++; S.streak++; S.lockoutAt = NOW;
            if (S.streak >= C.maxBrown) emergency('repeated brownouts');
        }
    }
} else {
    S.lowSince = null; S.justShed = false;
    if (S.okSince === null) S.okSince = NOW; else if (NOW - S.okSince >= C.decay) S.streak = 0;
}

if (S.mode !== 'ACTIVE') {
    for (let i = 0; i < 4; i++) switchOff(i);
    S.pending = 0;
} else {
    if (S.lockoutAt !== null && NOW - S.lockoutAt >= C.backoff) S.lockoutAt = null;
    for (let i = 0; i < 4; i++) if (S.on[i] && !S.want[i]) switchOff(i);
    let pick = -1;
    for (let i = 0; i < 4; i++) if (S.want[i] && !S.on[i] && (pick < 0 || C.rank[i] < C.rank[pick])) pick = i;
    S.pending = pick >= 0 ? 1 : 0;
    if (pick >= 0) {
        const staggerOk = S.lastStart === null || NOW - S.lastStart >= C.stagger;
        const restOk = S.offAt[pick] === null || NOW - S.offAt[pick] >= C.minOff;
        if (staggerOk && restOk && v >= C.startMin && S.lockoutAt === null) {
            S.on[pick] = 1; S.lastStart = NOW; S.pending = 0;
            for (let i = 0; i < 4; i++) if (S.want[i] && !S.on[i]) S.pending = 1;
        }
    }
}

// ---------- publish (on change or every 1 s) ----------
const sig = [S.on.join(''), S.mode, S.railName, S.pending, S.brown].join('|');
context.set('S', S);
if (sig === S.lastSig && NOW - S.lastPub < 1000) return null;
S.lastSig = sig; S.lastPub = NOW; context.set('S', S);
const r2 = Math.round(v * 100) / 100;
return { topic: 'rover/power/status', payload: {
    mode: S.mode, mode_id: S.mode === 'ACTIVE' ? 0 : (S.mode === 'SLEEP' ? 1 : 2),
    rail_v: r2, battery_v: r2, rail: S.railName, ch: S.on.slice(), want: S.want.slice(),
    pending: S.pending, brownouts: S.brown, fault: S.fault, uptime_s: Math.round((NOW - S.t0) / 1000) } };

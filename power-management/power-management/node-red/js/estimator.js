// Battery estimate. A USB power bank outputs a regulated 5 V until it is almost empty, so the
// rail voltage cannot show remaining charge. Instead we integrate the load current (mAh used).
// ASSUMPTIONS - replace with values measured on the real rover using a USB power meter:
const CAPACITY_MAH = 10000;     // label capacity of the bank (rated at ~3.7 V cell voltage)
const EFFICIENCY   = 0.65;      // usable fraction at 5 V after boost-converter losses
const I_ACTIVE_BASE = 260;      // mA: ESP32 (Wi-Fi, 240 MHz) + relay board idle
const I_SLEEP_BASE  = 110;      // mA: ESP32 at 80 MHz, loads off
const I_RELAY_COIL  = 70;       // mA per energised relay coil
const I_CH = [900, 650, 650, 450];   // mA per load while running: pump, left, right, aux

let used = context.get('used') || 0;
let last = context.get('last') || null;
const now = Date.now();

if (msg.reset) {
    used = 0; last = now;
    context.set('used', used); context.set('last', last);
    return [{ payload: 100 }, { payload: 'estimate reset to 100 %' }];
}

const s = msg.payload;
if (!s || s.mode === undefined) return null;
let mA = (s.mode === 'ACTIVE') ? I_ACTIVE_BASE : I_SLEEP_BASE;
for (let i = 0; i < 4; i++) if (s.ch && s.ch[i]) mA += I_CH[i] + I_RELAY_COIL;

if (last !== null) used += mA * Math.min((now - last) / 1000, 30) / 3600;   // ignore gaps > 30 s
last = now;
context.set('used', used); context.set('last', last);

const usable = CAPACITY_MAH * EFFICIENCY;
const remaining = Math.max(0, usable - used);
const pct = Math.round(100 * remaining / usable);
const hrs = remaining / mA;
const h = Math.floor(hrs), m = Math.round((hrs - h) * 60);
return [{ payload: pct }, { payload: '~' + h + ' h ' + m + ' min at ' + mA + ' mA  (' + Math.round(used) + ' mAh used)' }];

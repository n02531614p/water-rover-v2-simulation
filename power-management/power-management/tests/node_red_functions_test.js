const fs = require('fs'); const assert = require('assert');
function mk(file) {
  const src = fs.readFileSync(__dirname + '/../node-red/js/' + file, 'utf8'); const store = {};
  const ctx = { get: k => store[k], set: (k, v) => { store[k] = v; } };
  const fn = new Function('msg', 'context', 'node', 'flow', 'global', src);
  return (msg) => fn(msg, ctx, { warn: console.warn }, ctx, ctx);
}
let t = 1_000_000; Date.now = () => t;
const sim = mk('simulator.js'), route = mk('route_status.js'), est = mk('estimator.js');
let last = null, starts = [], prevOn = [0,0,0,0];
function tick(ms) { for (let e = 0; e < ms; e += 200) { t += 200; const r = sim({ topic: 'tick', payload: '1' }); if (r) { last = r.payload;
  last.ch.forEach((v, i) => { if (v && !prevOn[i]) starts.push([i + 1, t]); }); prevOn = last.ch.slice(); } } }
sim({ topic: 'sim/enable', payload: true });
sim({ topic: 'rover/power/cmd', payload: 'active' });
for (let i = 1; i <= 4; i++) sim({ topic: 'rover/power/req/' + i, payload: '1' });
tick(5000);
console.log('start order (CH, t):', starts.map(s => s[0] + '@' + (s[1] - 1_000_000)).join(' '));
assert.deepStrictEqual(starts.map(s => s[0]), [2, 3, 1, 4]);
for (let i = 1; i < starts.length; i++) assert(starts[i][1] - starts[i-1][1] >= 600);
assert.deepStrictEqual(last.ch, [1,1,1,1]);
// brownout -> aux shed
sim({ topic: 'sim/sag', payload: '1' }); tick(600);
console.log('after 0.6 s sag: ch=', last.ch, 'brownouts=', last.brownouts, 'rail=', last.rail);
assert.deepStrictEqual(last.ch, [1,1,1,0]); assert.strictEqual(last.brownouts, 1);
tick(1200); console.log('after 1.8 s: ch=', last.ch, 'brownouts=', last.brownouts); assert.deepStrictEqual(last.ch, [0,1,1,0]);
tick(12000); console.log('after recovery: ch=', last.ch, 'brownouts=', last.brownouts);
// emergency latch
sim({ topic: 'rover/power/cmd', payload: 'emergency' }); tick(400);
assert.strictEqual(last.mode, 'EMERGENCY'); assert.deepStrictEqual(last.ch, [0,0,0,0]);
sim({ topic: 'rover/power/cmd', payload: 'active' }); tick(400); assert.strictEqual(last.mode, 'EMERGENCY');
sim({ topic: 'rover/power/cmd', payload: 'reset' }); tick(400); assert.strictEqual(last.mode, 'SLEEP');
// route + estimator
const out = route({ payload: last });
assert.strictEqual(out.length, 9); console.log('route outputs ok; mode text =', out[3].payload, '| rail text =', out[2].payload);
const out2 = route({ payload: { ...last, mode: 'EMERGENCY', fault: 'operator emergency shutdown', rail: 'SAG', brownouts: 9 } });
console.log('toast:', JSON.stringify(out2[5]));
let e; for (let k = 0; k < 180; k++) { t += 10000; e = est({ payload: { mode: 'ACTIVE', ch: [1,1,1,0] } }); }
console.log('estimator:', JSON.stringify(e)); assert(e[0].payload < 100);
console.log(JSON.stringify(est({ reset: true })));
console.log('JS HARNESS PASSED');

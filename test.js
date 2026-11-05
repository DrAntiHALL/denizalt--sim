'use strict';
// Çalıştır: node test.js
const assert = require('assert');
const P = require('./sub-physics.js');
const { CONFIG, createSim, neutralFill } = P;

let failed = 0;
function test(name, fn) {
  try { const info = fn(); console.log(`  GEÇTİ  ${name}${info ? '  → ' + info : ''}`); }
  catch (e) { failed++; console.log(`  KALDI  ${name}\n         ${e.message}`); }
}
const run = (sim, secs, cmd) => { const n = Math.round(secs / CONFIG.sim.dt); for (let i = 0; i < n; i++) sim.step(typeof cmd === 'function' ? cmd(sim) : cmd); };

console.log('\nFizik çekirdeği testleri (dt = ' + CONFIG.sim.dt + ' s, RK4)\n');

test('0) Nötr balast doluluğu 0..1 aralığında', () => {
  const f = neutralFill();
  assert(f > 0 && f < 1, 'f=' + f);
  return `nötr doluluk = ${f.toFixed(3)}`;
});

test('1) Nötr yüzdürmede denizaltı olduğu derinlikte kalır', () => {
  const sim = createSim(CONFIG, { z: 5 });
  run(sim, 120, { ballast: neutralFill() });
  const dz = Math.abs(sim.z - 5);
  assert(dz < 1e-6, `sapma ${dz} m`);
  assert(Math.abs(sim.theta) < 1e-9 && Math.abs(sim.x) < 1e-9);
  return `120 s sonra sapma = ${dz.toExponential(2)} m`;
});

test('2) Pozitif ağırlıkta sınır hıza ulaşır (analitik değerle uyumlu)', () => {
  const sim = createSim(CONFIG, { z: 1.5, b: 1 });       // balast tam dolu → ağır
  const d = P.derived(CONFIG);
  const m = CONFIG.hull.dryMass + CONFIG.ballast.tankVolume * CONFIG.env.rho;
  const dW = m * CONFIG.env.g - CONFIG.env.rho * CONFIG.env.g * CONFIG.hull.volume;
  assert(dW > 0);
  const vt = Math.sqrt(2 * dW / (CONFIG.env.rho * CONFIG.drag.Cz * d.Alateral));
  run(sim, 15, { ballast: 1 });
  const tel = sim.telemetry();
  assert(!sim.onBottom, 'dibe çarptı, test süresi kısaltılmalı');
  const err = Math.abs(tel.vz - vt) / vt;
  assert(err < 0.01, `vz=${tel.vz.toFixed(4)} beklenen=${vt.toFixed(4)}`);
  return `v_sınır = ${tel.vz.toFixed(4)} m/s (analitik ${vt.toFixed(4)}, hata %${(err * 100).toFixed(3)})`;
});

test('3) Küçük yunuslama bozulması sönümlenerek düzelir', () => {
  const th0 = 5 * Math.PI / 180;
  const sim = createSim(CONFIG, { z: 5, theta: th0 });
  const peaks = []; let prev = 0, prev2 = 0;
  for (let i = 0; i < 4000; i++) {
    sim.step({ ballast: neutralFill() });
    if (prev > prev2 && prev > sim.theta && prev > 0) peaks.push(prev);
    prev2 = prev; prev = sim.theta;
  }
  assert(peaks.length >= 2, 'salınım gözlenmedi');
  for (let i = 1; i < peaks.length; i++) assert(peaks[i] < peaks[i - 1], 'tepe artıyor');
  assert(Math.abs(sim.theta) < 0.05 * th0, `son θ=${sim.theta}`);
  return `θ0=5°, 40 s sonra |θ|=${(Math.abs(sim.theta) * 180 / Math.PI).toExponential(2)}°, ${peaks.length} tepe azalarak`;
});

test('4) Yüzeyde kaldırma kısmen değişir (z=+R tam, z=0 yarı, z=−R sıfır)', () => {
  const R = CONFIG.hull.diameter / 2;
  assert.strictEqual(P.submergedFraction(R, R), 1);
  assert(Math.abs(P.submergedFraction(0, R) - 0.5) < 1e-12);
  assert.strictEqual(P.submergedFraction(-R, R), 0);
  // pozitif yüzdürmeli (hafif) gövde yüzeye çıkıp su çizgisinde dengelenir
  const sim = createSim(CONFIG, { z: 1, b: 0 });
  run(sim, 120, { ballast: 0 });
  assert(sim.z > -R && sim.z < R, 'z=' + sim.z);
  return `hafif gövde su çizgisinde dengelendi, z = ${sim.z.toFixed(4)} m`;
});

test('5) Kıç dümeni işareti: pozitif δ ileri hızda burunu yukarı kaldırır', () => {
  const sim = createSim(CONFIG, { z: 5, u: 1.0 });
  run(sim, 3, { ballast: neutralFill(), plane: 10 * Math.PI / 180 });
  assert(sim.theta > 0, 'θ=' + sim.theta);
  return `3 s sonra θ = ${(sim.theta * 180 / Math.PI).toFixed(2)}°`;
});

test('6) Balast pompası hız sınırlı (25 s\'den önce 0→1 olamaz)', () => {
  const sim = createSim(CONFIG, { z: 5, b: 0 });
  run(sim, 10, { ballast: 1 });
  assert(Math.abs(sim.b - 10 * CONFIG.ballast.fillRate) < 1e-6, 'b=' + sim.b);
  return `10 s sonra doluluk = ${sim.b.toFixed(3)}`;
});

test('7) İtki ile ileri sınır hız (kuadratik sürtünme dengesi)', () => {
  const sim = createSim(CONFIG, { z: 5 });
  run(sim, 40, { ballast: neutralFill(), thrust: 1 });
  const d = P.derived(CONFIG);
  const vt = Math.sqrt(2 * CONFIG.thruster.maxForward / (CONFIG.env.rho * CONFIG.drag.Cx * d.Afront));
  assert(Math.abs(sim.u - vt) / vt < 0.02, `u=${sim.u} beklenen=${vt}`);
  return `u = ${sim.u.toFixed(3)} m/s (analitik ${vt.toFixed(3)})`;
});

test('8) Ek kütle ataleti: RK4 Kirchhoff enerji korunumu (sürtünmesiz, serbest)', () => {
  const cfg = JSON.parse(JSON.stringify(CONFIG));
  cfg.drag = { Cx: 0, Cz: 0, pitchLinear: 0, pitchQuadratic: 0 };
  cfg.hull.BG = 0;
  cfg.addedMass.munkFactor = 1;                 // enerji korunumu tam Munk momentiyle geçerli
  cfg.sternPlane.area = 0;                      // dümen kuvveti enerji ekler/alır, hariç tut
  const sim = createSim(cfg, { z: 5, u: 0.5, w: 0.1, q: 0.1, b: neutralFill(cfg) });
  const d = P.derived(cfg);
  const m = cfg.hull.dryMass + sim.b * cfg.ballast.tankVolume * cfg.env.rho;
  const Iy = m * cfg.hull.pitchGyradius ** 2 + d.IaPitch;
  const E = () => 0.5 * ((m + d.maSurge) * sim.u ** 2 + (m + d.maHeave) * sim.w ** 2 + Iy * sim.q ** 2);
  const E0 = E();
  run(sim, 5, { ballast: sim.b });   // yüzeye/dibe ulaşmadan kısa süre
  const rel = Math.abs(E() - E0) / E0;
  assert(rel < 1e-6, 'bağıl enerji hatası ' + rel);
  return `5 s sonra bağıl enerji kayması = ${rel.toExponential(2)}`;
});

console.log(failed ? `\n${failed} test KALDI\n` : '\nTüm testler geçti.\n');
process.exit(failed ? 1 : 0);

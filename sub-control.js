/* sub-control.js — derinlik tutma (PID) kontrolcüsü. Arayüzden bağımsız, Node'da test edilebilir.
 *
 * Yapı (kaskad):
 *   dış döngü : derinlik hatası --PID--> hedef yunuslama açısı θ_ref  (Kp, Ki, Kd ayarlanabilir)
 *   iç döngü  : δ = Kθ·(θ_ref − θ) − Kq·q   (kıç dümeni; ileri hız gerektirir)
 *   balast    : yavaş integral trim (pompa yavaş; sürekli ağırlık sapmasını giderir)
 * İşaretler: z aşağı pozitif; δ ve θ burun yukarı pozitif.
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.SubControl = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';
  const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

  const CONTROL = {
    pid: { Kp: 0.30, Ki: 0.01, Kd: 1.00 },   // rad/m, rad/(m·s), rad·s/m
    pitchLimit: 20 * Math.PI / 180,           // rad  hedef yunuslama sınırı
    inner: { Ktheta: 2.0, Kq: 1.5 },          // dümen rad / yunuslama rad ; dümen rad / (rad/s)
    ballastTrim: { Kb: 0.005, limit: 0.3 },   // doluluk/(m·s) ; trim sınırı (±)
    integralZone: 1.0,                        // m  |hata| bundan büyükken integratörler durur (windup önlemi)
    minSpeed: 0.15,                           // m/s altında dümen yok sayılır, integratör durur
  };

  function createDepthHold(neutralFill, control = CONTROL) {
    const c = {
      enabled: false,
      setpoint: 3,                            // m
      Kp: control.pid.Kp, Ki: control.pid.Ki, Kd: control.pid.Kd,
      trimEnabled: true,
      I: 0, trim: 0, thetaRef: 0,
      reset() { c.I = 0; c.trim = 0; c.thetaRef = 0; },
      /** sim: createSim nesnesi. Dönüş: { plane (rad), ballast (0..1) } */
      update(sim, dt) {
        const e = c.setpoint - sim.z;                                   // + → daha derine
        const vz = -sim.u * Math.sin(sim.theta) + sim.w * Math.cos(sim.theta);
        const speed = Math.abs(sim.u);
        const active = speed > control.minSpeed;

        // Dış PID (D terimi ölçümden: ani hedef değişimine tekme yok)
        const unsat = -(c.Kp * e + c.Ki * c.I + c.Kd * (-vz));
        const sat = clamp(unsat, -control.pitchLimit, control.pitchLimit);
        // anti-windup: doyumdayken hata doyum yönündeyse integrali durdurma
        const winding = unsat !== sat && Math.sign(e) === -Math.sign(sat);
        const inZone = Math.abs(e) < control.integralZone;
        if (active && !winding && inZone) c.I += e * dt;
        c.thetaRef = clamp(-(c.Kp * e + c.Ki * c.I + c.Kd * (-vz)), -control.pitchLimit, control.pitchLimit);

        // İç döngü
        // açı farkı −π..π'ye sarılır: takla atmış bir gövdede kontrolcü turları geri sarmaya çalışmasın
        const eTh = Math.atan2(Math.sin(c.thetaRef - sim.theta), Math.cos(c.thetaRef - sim.theta));
        let plane = control.inner.Ktheta * eTh - control.inner.Kq * sim.q;
        if (!active) plane = 0;

        // Balast trimi
        if (c.trimEnabled && inZone) {
          c.trim = clamp(c.trim + control.ballastTrim.Kb * e * dt, -control.ballastTrim.limit, control.ballastTrim.limit);
        }
        const ballast = clamp(neutralFill + (c.trimEnabled ? c.trim : 0), 0, 1);
        return { plane, ballast };
      },
    };
    return c;
  }
  return { CONTROL, createDepthHold };
});

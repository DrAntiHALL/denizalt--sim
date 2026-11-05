/* ==========================================================================
 * sub-physics.js — 2B dikey düzlem denizaltı fizik çekirdeği (arayüzden bağımsız)
 *
 * Koordinatlar : x ileri (m), z DERİNLİK (m, aşağı pozitif, z=0 su yüzeyi)
 * Gövde ekseni : u ileri hız, w aşağı hız (m/s), q yunuslama hızı (rad/s)
 * Yunuslama θ  : burun yukarı pozitif (rad)
 * Dümen δ      : pozitif = burun yukarı komutu (kıçı aşağı iter)
 *
 * Tarayıcıda  : <script src="sub-physics.js"></script>  → window.SubPhysics
 * Node'da     : const SubPhysics = require('./sub-physics.js')
 * ========================================================================== */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.SubPhysics = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  /* ======================= CONFIG (hepsi SI birimleri) ======================= */
  const CONFIG = {
    sim: {
      dt: 0.01,                 // s   sabit RK4 adımı
    },
    env: {
      rho: 1000,                // kg/m³  su yoğunluğu (tatlı su; tuzlu için 1025)
      g: 9.81,                  // m/s²
      bottomDepth: 10,          // m   dip derinliği
    },
    hull: {
      length: 1.0,              // m   boy
      diameter: 0.12,           // m   çap
      dryMass: 9.5,             // kg  balast suyu hariç kütle
      volume: 0.0100,           // m³  yer değiştirilen hacim (tam batık)
      BG: 0.02,                 // m   kaldırma merkezi ile ağırlık merkezi arası (B, G'nin üstünde)
      pitchGyradius: 0.289,     // m   yunuslama atalet yarıçapı ≈ L/√12 (Iy = m·k²)
    },
    ballast: {
      tankVolume: 0.0010,       // m³  balast tankı hacmi (1 L)
      fillRate: 0.04,           // 1/s pompa hızı (dolulukta saniyede değişim; 25 s'de boştan doluya)
    },
    addedMass: {                // yer değiştiren su kütlesine (ρ·V) oranla basit katsayılar
      surge: 0.10,              // ileri eksen
      heave: 1.00,              // düşey (dik) eksen
      pitch: 1.00,              // yunuslama: Ia = katsayı · ρV · L²/12
      munkFactor: 0.5,          // -  Munk (destabilize) momentinin ölçeği; 1 = tam teorik, 0 = yok
    },
    drag: {
      Cx: 0.30,                 // -   ileri sürtünme katsayısı (ön kesit alanına göre)
      Cz: 1.00,                 // -   düşey (cross-flow) sürtünme katsayısı (L·D alanına göre)
      pitchLinear: 0.5,         // N·m·s/rad    dönme sönümlemesi (doğrusal)
      pitchQuadratic: 1.9,      // N·m·s²/rad²  dönme sönümlemesi (kuadratik)
    },
    sternPlane: {
      area: 0.0035,             // m²   toplam kıç dümeni alanı (küçültüldü: etkinlik azaltıldı)
      arm: 0.42,                // m    ağırlık merkezinden kıç dümenine mesafe (arkada)
      CLalpha: 3.0,             // 1/rad  kaldırma eğimi
      CLmax: 1.0,               // -    stall sınırı (tanh ile yumuşak doyum)
      maxAngle: 30 * Math.PI / 180,   // rad  mekanik sınır
      maxRate: 60 * Math.PI / 180,    // rad/s servo hız sınırı
    },
    seabed: {                 // dip teması: gövde uçlarında yay-sönümleyici + sürtünme
      stiffness: 5000,          // N/m    uç başına normal yay
      damping: 500,             // N·s/m  uç başına normal sönüm (yalnız iterken)
      friction: 0.5,            // -      Coulomb sürtünme katsayısı μ
      slipVel: 0.01,            // m/s    sürtünmenin yumuşatma hızı (tanh)
    },
    thruster: {
      maxForward: 5.0,          // N
      maxReverse: 2.5,          // N
      tau: 0.3,                 // s  motor/pervane birinci derece gecikmesi
    },
  };

  /* ============================ Yardımcı fonksiyonlar ============================ */
  const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
  const wrapAngle = a => Math.atan2(Math.sin(a), Math.cos(a));

  function derived(cfg) {
    const { rho } = cfg.env, h = cfg.hull;
    const R = h.diameter / 2;
    const dispMass = rho * h.volume;
    return {
      R,
      dispMass,
      Afront: Math.PI * R * R,
      Alateral: h.length * h.diameter,
      maSurge: cfg.addedMass.surge * dispMass,
      maHeave: cfg.addedMass.heave * dispMass,
      IaPitch: cfg.addedMass.pitch * dispMass * h.length * h.length / 12,
    };
  }

  /** Nötr yüzdürme için balast doluluğu (0..1). */
  function neutralFill(cfg = CONFIG) {
    const d = derived(cfg);
    return (d.dispMass - cfg.hull.dryMass) / (cfg.env.rho * cfg.ballast.tankVolume);
  }

  /** Batık hacim oranı: gövde enine kesiti (daire), merkez z derinliğinde. 0..1. */
  function submergedFraction(z, R) {
    if (z >= R) return 1;
    if (z <= -R) return 0;
    const above = (R * R * Math.acos(z / R) - z * Math.sqrt(R * R - z * z)) / (Math.PI * R * R);
    return 1 - above;
  }

  /** Kıç dümeni kuvveti ve momenti (gövde ekseninde). */
  function planeForce(u, w, q, delta, cfg, f) {
    const sp = cfg.sternPlane, rho = cfg.env.rho;
    const ws = w + q * sp.arm;                         // dümen noktasındaki yerel düşey hız
    const V2 = u * u + ws * ws;
    const gate = Math.max(0, Math.tanh(u / 0.05));     // geri giderken etkisiz
    const alphaFlow = Math.atan2(ws, Math.abs(u) + 1e-6);
    const alpha = delta - alphaFlow;
    const CL = sp.CLmax * Math.tanh(sp.CLalpha * alpha / sp.CLmax);
    const Fz = gate * f * 0.5 * rho * sp.area * CL * V2; // +z (aşağı) kuvvet
    return { Fz, M: sp.arm * Fz };                       // kıçta aşağı kuvvet → burun yukarı moment
  }

  /**
   * Dip teması (dünya çerçevesi). Gövde: uçları ±L/2 olan, yarıçapı R kapsül.
   * Her uç dibe gömülürse normal kuvvet N = k·pen + c·vz (≥0) ve
   * sürtünme Ft = −μ·N·tanh(vx/v0). Uçta uygulandığı için yunuslama momenti de üretir
   * (burun dibe dayanınca gövdeyi yatırır). Dönüş: { Fx, Fz (dünya, z aşağı), M, pen }
   */
  function seabedForce(z, th, xdot, zdot, q, cfg) {
    const sb = cfg.seabed, R = cfg.hull.diameter / 2, half = cfg.hull.length / 2;
    const c = Math.cos(th), s = Math.sin(th);
    let Fx = 0, Fz = 0, M = 0, pen = -Infinity;
    for (const bx of [half, -half]) {
      const rx = bx * c, rz = -bx * s;                 // uç merkezi (CG'ye göre, dünya)
      const p = z + rz + R - cfg.env.bottomDepth;      // gömülme
      if (p > pen) pen = p;
      if (p <= 0) continue;
      const vx = xdot + q * rz, vz = zdot - q * rx;    // uç noktasının hızı
      const N = Math.max(0, sb.stiffness * p + sb.damping * vz);
      const Ft = -sb.friction * N * Math.tanh(vx / sb.slipVel);
      Fx += Ft; Fz -= N;
      M += Ft * (rz + R) + N * rx;                     // temas noktası: uç + R aşağı
    }
    return { Fx, Fz, M, pen };
  }

  /** Durum türevi. s=[x,z,θ,u,w,q]; ctl={b,delta,thrust} (RK4 adımı boyunca sabit). */
  function deriv(s, ctl, cfg) {
    const [, z, th, u, w, q] = s;
    const { rho, g } = cfg.env, h = cfg.hull;
    const d = derived(cfg);
    const sinT = Math.sin(th), cosT = Math.cos(th);

    const f = submergedFraction(z, d.R);
    const m = h.dryMass + ctl.b * cfg.ballast.tankVolume * rho;       // toplam kütle
    const W = m * g;
    const B = rho * g * h.volume * f;                                  // kaldırma (su çizgisine göre)

    // Gövde sürtünmesi (batık orana göre ölçeklenir)
    const Xd = -0.5 * rho * cfg.drag.Cx * d.Afront * u * Math.abs(u) * f;
    const Zd = -0.5 * rho * cfg.drag.Cz * d.Alateral * w * Math.abs(w) * f;
    const Md = -(cfg.drag.pitchLinear * q + cfg.drag.pitchQuadratic * q * Math.abs(q)) * f;

    const pl = planeForce(u, w, q, ctl.delta, cfg, f);

    // Dip teması: dünya kuvvetini gövde eksenine çevir
    const xdot = u * cosT + w * sinT, zdot = -u * sinT + w * cosT;
    const sb = seabedForce(z, th, xdot, zdot, q, cfg);
    const Xs = sb.Fx * cosT - sb.Fz * sinT;
    const Zs = sb.Fx * sinT + sb.Fz * cosT;

    // Gövde eksenindeki kuvvetler: net ağırlık (W-B) dünya aşağı = (-sinθ, cosθ)
    const Fx = -(W - B) * sinT + ctl.thrust + Xd + Xs;
    const Fz = (W - B) * cosT + pl.Fz + Zd + Zs;
    // Moment: kaldırma G'nin BG üstündeki B noktasında → doğrultucu −B·BG·sinθ
    const My = -B * h.BG * sinT + pl.M + Md + sb.M;

    // Ek kütle dahil etkin kütleler (Kirchhoff): M1=m+ma_x, M3=m+ma_z
    const M1 = m + d.maSurge, M3 = m + d.maHeave;
    const Iy = m * h.pitchGyradius * h.pitchGyradius + d.IaPitch;

    const du = (Fx - M3 * w * q) / M1;
    const dw = (Fz + M1 * u * q) / M3;
    const dq = (My + cfg.addedMass.munkFactor * (M3 - M1) * u * w) / Iy;                          // Munk momenti

    return [
      xdot,                       // dx
      zdot,                       // dz (aşağı pozitif)
      q,                          // dθ
      du, dw, dq,
    ];
  }

  function rk4(s, ctl, dt, cfg) {
    const add = (a, k, c) => a.map((v, i) => v + c * k[i]);
    const k1 = deriv(s, ctl, cfg);
    const k2 = deriv(add(s, k1, dt / 2), ctl, cfg);
    const k3 = deriv(add(s, k2, dt / 2), ctl, cfg);
    const k4 = deriv(add(s, k3, dt), ctl, cfg);
    return s.map((v, i) => v + dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
  }

  /* ================================== Simülasyon ================================== */
  function createSim(cfg = CONFIG, init = {}) {
    const sim = {
      cfg,
      t: 0,
      // fiziksel durum
      x: 0, z: 2, theta: 0, u: 0, w: 0, q: 0,
      // aktüatör durumları
      b: neutralFill(cfg),      // balast doluluğu 0..1
      delta: 0,                 // dümen açısı (rad)
      thrust: 0,                // gerçek itki (N)
      onBottom: false,
    };
    Object.assign(sim, init);

    /**
     * Bir dt ilerlet. cmd = { ballast:0..1, plane:rad, thrust:-1..1 }
     * Aktüatörler (pompa, servo, motor) RK4 dışında, hız sınırlı güncellenir.
     */
    sim.step = function (cmd = {}) {
      const dt = cfg.sim.dt;
      const bCmd = clamp(cmd.ballast ?? sim.b, 0, 1);
      const dCmd = clamp(cmd.plane ?? sim.delta, -cfg.sternPlane.maxAngle, cfg.sternPlane.maxAngle);
      const tn = clamp(cmd.thrust ?? 0, -1, 1);
      const tCmd = tn >= 0 ? tn * cfg.thruster.maxForward : tn * cfg.thruster.maxReverse;

      const db = cfg.ballast.fillRate * dt, dd = cfg.sternPlane.maxRate * dt;
      sim.b += clamp(bCmd - sim.b, -db, db);
      sim.delta += clamp(dCmd - sim.delta, -dd, dd);
      sim.thrust += (tCmd - sim.thrust) * (1 - Math.exp(-dt / cfg.thruster.tau));

      const s = rk4([sim.x, sim.z, sim.theta, sim.u, sim.w, sim.q],
                    { b: sim.b, delta: sim.delta, thrust: sim.thrust }, dt, cfg);
      [sim.x, sim.z, sim.theta, sim.u, sim.w, sim.q] = s;

      // Dip teması artık deriv() içinde kuvvet olarak hesaplanıyor; burada yalnız durum bayrağı
      sim.onBottom = seabedForce(sim.z, sim.theta, 0, 0, 0, cfg).pen > -0.005;
      sim.t += dt;
      return sim;
    };

    /** Telemetri (dünya çerçevesinde hızlar dahil). */
    sim.telemetry = function () {
      const sT = Math.sin(sim.theta), cT = Math.cos(sim.theta);
      return {
        t: sim.t, depth: sim.z, x: sim.x,
        speed: Math.hypot(sim.u, sim.w),
        vz: -sim.u * sT + sim.w * cT,         // dikey dünya hızı (aşağı +)
        pitchDeg: wrapAngle(sim.theta) * 180 / Math.PI,       // −180..180 (taklalar sarılır)
        ballast: sim.b, planeDeg: sim.delta * 180 / Math.PI, thrust: sim.thrust,
        onBottom: sim.onBottom,
      };
    };
    return sim;
  }

  return { CONFIG, createSim, deriv, rk4, neutralFill, submergedFraction, planeForce, seabedForce, derived, clamp, wrapAngle };
});

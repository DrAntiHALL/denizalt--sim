# Denizaltı Simülasyonu (2B dikey düzlem)

~1 m boyunda model denizaltının fizik tabanlı yan görünüş simülasyonu. Harici kütüphane yok; telefonda tarayıcıdan açılır.

## Nasıl çalıştırılır
- **Telefonda / bilgisayarda:** `index.html` dosyasını (yanındaki `sub-physics.js` ve `sub-control.js` ile birlikte) tarayıcıda açın.
- **Yerel sunucu (isteğe bağlı):** `python3 -m http.server 8000` → `http://<bilgisayar-ip>:8000`
- **Testler:** `node test.js` (veya `npm test`)

### GitHub Pages ile yayınlama
Repo → *Settings → Pages → Build and deployment → Source: Deploy from a branch*, dal olarak `main` (veya bu dalı birleştirdikten sonra) ve klasör `/ (root)` seçin. Adres: `https://<kullanıcı>.github.io/denizalt--sim/`. Bu ayar hesabınızdan yapılmalıdır.

## Dosyalar
| Dosya | İçerik |
|---|---|
| `sub-physics.js` | Fizik çekirdeği: `CONFIG`, RK4 (dt=0.01 s), kuvvetler, aktüatörler |
| `sub-control.js` | Derinlik tutma kaskad PID kontrolcüsü |
| `index.html` | Canvas görünüm, telemetri, grafik, kontroller |
| `test.js` | 12 test (fizik + PID) |

## Kullanım
- **Manuel:** Balast, kıç dümeni ve itiş kaydırıcıları; basılı tutulan butonlar (Doldur/Boşalt, Burun ↑/↓), Nötr, İleri/Geri/Dur.
- **Derinlik tutma:** Hedef kaydırıcıyla ya da görünüme/grafiğe dokunarak seçilir. Kp/Ki/Kd alanları anında etki eder. Dümen yalnızca ileri hızda etkilidir, itişi ≥ %30 yapın. Bu modda balast ve dümeni PID yönetir.

## Fizik modeli
Durum: `x, z (derinlik, aşağı +), θ (burun yukarı +), u, w, q` + aktüatörler (balast, dümen, itki).
- Ağırlık `m·g` (m = kuru kütle + balast suyu), kaldırma `ρ·g·V·f(z)`; `f` yüzeyde dairesel kesite göre batık orandır (z=+R tam, 0 yarı, −R sıfır).
- Kuadratik sürtünme (ileri ve düşey ayrı), yunuslama sönümlemesi (doğrusal + kuadratik), ikisi de batık orana göre ölçeklenir.
- Kıç dümeni: `½ρ·A·CL(α)·V²`, `CL = CLmax·tanh(CLα·α/CLmax)` (yumuşak stall), α = δ − akış açısı; geri giderken etkisiz.
- Ek kütle: `ρV` oranında katsayılar; Kirchhoff bağlaşım terimleri ve Munk momenti dahil.
- Doğrultucu moment `−B·BG·sinθ`.
- Aktüatörler RK4 dışında, hız sınırlı: balast pompası, dümen servosu, itici (1. derece gecikme).
- Basitleştirmeler: balast suyu ağırlık merkezinde (trim etkisi yok), dalga/akıntı yok, dip yatay ve çarpışma kayıpsız-sıfırlama.

## Parametreler neyi etkiler (`CONFIG`, SI)
| Parametre | Etki |
|---|---|
| `hull.dryMass`, `hull.volume`, `ballast.tankVolume` | Nötr balast doluluğunu belirler (`neutralFill`); hacim büyürse gövde hafifler |
| `hull.BG` | Doğrultucu moment; büyükse yunuslama daha sert düzelir, salınım frekansı artar |
| `ballast.fillRate` | Pompa hızı: dalış/yükselişin ne kadar çabuk başladığı |
| `drag.Cx`, `Cz` | İleri maksimum hız (`√(2T/ρCxA)`) ve düşey sınır hız |
| `drag.pitchLinear/Quadratic` | Yunuslama salınımının sönümü |
| `addedMass.*` | Ataletin artışı (ivmelenme ağırlaşır) |
| `addedMass.munkFactor` | Munk momenti (ileri hızda yunuslama kararsızlığı); 0 = yok, 1 = teorik tam |
| `sternPlane.area`, `CLalpha`, `arm` | Dümen etkinliği (yunuslama momenti ∝ alan·kol·V²) |
| `sternPlane.maxAngle`, `maxRate` | Dümen sınırı ve servo hızı |
| `thruster.maxForward/Reverse`, `tau` | İtki ve motor tepkisi |
| `env.rho`, `env.bottomDepth` | Su yoğunluğu (tuzlu: 1025; nötr balast yeniden hesaplanır), dip derinliği |

PID (`sub-control.js` → `CONTROL`): dış döngü `PID(derinlik hatası) → hedef yunuslama` (sınır ±20°), iç döngü `δ = Kθ(θref−θ) − Kq·q`. Integratör, `|hata| < 1 m` bölgesinde çalışır ve doyumda durur (anti-windup). Balast trimi yavaş bir integraldir.

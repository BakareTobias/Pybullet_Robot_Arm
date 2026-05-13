"""
DERS 9: Jacobian Matrisi ve İleri Seviye IK
Kendi IK algoritmamızı sıfırdan yazacağız!
"""

import numpy as np
import matplotlib.pyplot as plt
import math

print("="*60)
print("DERS 9: JACOBIAN MATRİSİ VE İLERİ SEVİYE IK")
print("="*60)

# =====================================================
# BÖLÜM 1: Jacobian Nedir?
# =====================================================
print("\nBÖLÜM 1: Jacobian Matrisi - Temel Kavram")
print("-"*60)
print("""
TANIM:
Jacobian, bir fonksiyonun türevlerinin matrisidir.

Robotlarda:
  Girdi  : Eklem açıları (θ₁, θ₂, ...)
  Çıktı  : El pozisyonu (x, y, z)
  Jacobian: "Açılar biraz değişirse, pozisyon ne kadar değişir?"

MATEMATİK:
  Forward Kinematics: f(θ₁, θ₂) = (x, y)

  Jacobian J:
    J = [∂x/∂θ₁  ∂x/∂θ₂]
        [∂y/∂θ₁  ∂y/∂θ₂]

  Anlamı:
    ∂x/∂θ₁ = "θ₁, 1 birim değişirse x ne kadar değişir?"

KULLANIM:
  Δp = J × Δθ

  "Pozisyon değişimi = Jacobian × Açı değişimi"
""")

input("Devam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 2: 2-Link Robot için Jacobian
# =====================================================
print("="*60)
print("BÖLÜM 2: 2-Link Robot Örneği")
print("="*60)

L1 = 1.0  # Link 1 uzunluğu
L2 = 0.8  # Link 2 uzunluğu

print(f"\nRobot Parametreleri:")
print(f"  L1 = {L1}m")
print(f"  L2 = {L2}m")

def forward_kinematics(theta1, theta2):
    """
    Forward Kinematics
    Girdi: θ₁, θ₂ (radyan)
    Çıktı: (x, y)
    """
    x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
    return x, y

def jacobian(theta1, theta2):
    """
    Jacobian matrisini hesapla

    Matematik:
      x = L1*cos(θ₁) + L2*cos(θ₁+θ₂)
      y = L1*sin(θ₁) + L2*sin(θ₁+θ₂)

    Türevler:
      ∂x/∂θ₁ = -L1*sin(θ₁) - L2*sin(θ₁+θ₂)
      ∂x/∂θ₂ = -L2*sin(θ₁+θ₂)
      ∂y/∂θ₁ = L1*cos(θ₁) + L2*cos(θ₁+θ₂)
      ∂y/∂θ₂ = L2*cos(θ₁+θ₂)

    Döndürür:
      J: 2x2 numpy array
    """
    # Türevleri hesapla
    dx_dtheta1 = -L1 * math.sin(theta1) - L2 * math.sin(theta1 + theta2)
    dx_dtheta2 = -L2 * math.sin(theta1 + theta2)
    dy_dtheta1 = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    dy_dtheta2 = L2 * math.cos(theta1 + theta2)

    # Jacobian matrisi
    J = np.array([
        [dx_dtheta1, dx_dtheta2],
        [dy_dtheta1, dy_dtheta2]
    ])

    return J

print("\n" + "="*60)
print("Jacobian Formülü (Elle Türev Aldık):")
print("="*60)
print("""
  J = [∂x/∂θ₁  ∂x/∂θ₂]
      [∂y/∂θ₁  ∂y/∂θ₂]

  J = [-L1*sin(θ₁) - L2*sin(θ₁+θ₂)    -L2*sin(θ₁+θ₂)    ]
      [ L1*cos(θ₁) + L2*cos(θ₁+θ₂)     L2*cos(θ₁+θ₂)    ]
""")

# Örnek hesaplama
theta1 = math.radians(30)  # 30 derece
theta2 = math.radians(45)  # 45 derece

J = jacobian(theta1, theta2)

print(f"\nÖrnek: θ₁={math.degrees(theta1):.0f}°, θ₂={math.degrees(theta2):.0f}°")
print(f"\nJacobian Matrisi:")
print(f"  J = [{J[0,0]:7.3f}  {J[0,1]:7.3f}]")
print(f"      [{J[1,0]:7.3f}  {J[1,1]:7.3f}]")

print(f"\nAnlamı:")
print(f"  θ₁, 1 rad değişirse:")
print(f"    x → {J[0,0]:.3f}m değişir")
print(f"    y → {J[1,0]:.3f}m değişir")
print(f"  θ₂, 1 rad değişirse:")
print(f"    x → {J[0,1]:.3f}m değişir")
print(f"    y → {J[1,1]:.3f}m değişir")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 3: Jacobian ile Hız İlişkisi
# =====================================================
print("="*60)
print("BÖLÜM 3: Jacobian ile Hız İlişkisi")
print("="*60)

print("""
Jacobian'ın bir diğer kullanımı: HIZ İLİŞKİSİ

Formül:
  v = J × θ̇

Burada:
  v = El'in hızı (vx, vy)
  θ̇ = Eklem açısal hızları (θ̇₁, θ̇₂)
  J = Jacobian matrisi
""")

# Örnek
theta1_dot = 0.5  # rad/s (θ₁'in açısal hızı)
theta2_dot = 0.3  # rad/s (θ₂'nin açısal hızı)

theta_dot = np.array([theta1_dot, theta2_dot])
v = J @ theta_dot  # Matris çarpımı

print(f"\nÖrnek:")
print(f"  Eklem hızları: θ̇₁={theta1_dot} rad/s, θ̇₂={theta2_dot} rad/s")
print(f"  El hızı: vx={v[0]:.3f} m/s, vy={v[1]:.3f} m/s")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 4: INVERSE KINEMATICS - Jacobian Yöntemi
# =====================================================
print("="*60)
print("BÖLÜM 4: Jacobian ile Inverse Kinematics")
print("="*60)

print("""
ŞİMDİ ASIL KONU: IK nasıl çözülür?

Problem:
  Hedef pozisyon: (x*, y*)
  Mevcut pozisyon: (x, y)
  Gerekli: θ₁, θ₂ değerlerini bul

ÇÖZÜM: İteratif Yöntem (Adım adım yaklaş)

Algoritma:
  1. Mevcut açılardan başla: θ₁, θ₂
  2. Mevcut pozisyonu hesapla: (x, y) = FK(θ₁, θ₂)
  3. Hata hesapla: e = (x*, y*) - (x, y)
  4. Jacobian hesapla: J
  5. Açı değişimini bul: Δθ = J⁻¹ × e
  6. Açıları güncelle: θ₁ += Δθ₁, θ₂ += Δθ₂
  7. Hata küçük mü? Evet → Bitti! Hayır → 2'ye git

Bu yönteme "Jacobian Inverse" veya "Newton-Raphson" denir.
""")

input("Devam için ENTER'a basın...\n")

def inverse_kinematics_jacobian(target_x, target_y, theta1_init=0, theta2_init=0,
                                max_iter=100, tolerance=0.001):
    """
    Jacobian yöntemi ile Inverse Kinematics

    Parametreler:
        target_x, target_y: Hedef pozisyon
        theta1_init, theta2_init: Başlangıç açıları
        max_iter: Maksimum iterasyon sayısı
        tolerance: Hata toleransı (metre)

    Döndürür:
        theta1, theta2: Hesaplanan eklem açıları
        iterations: Kaç iterasyonda bulundu
        errors: Her iterasyondaki hata
    """
    # Başlangıç
    theta1 = theta1_init
    theta2 = theta2_init

    errors = []

    for iteration in range(max_iter):
        # Mevcut pozisyon
        x, y = forward_kinematics(theta1, theta2)

        # Hata vektörü
        error_x = target_x - x
        error_y = target_y - y
        error = np.array([error_x, error_y])

        # Hata büyüklüğü (Euclidean distance)
        error_norm = np.linalg.norm(error)
        errors.append(error_norm)

        # Hedefe ulaştık mı?
        if error_norm < tolerance:
            return theta1, theta2, iteration, errors

        # Jacobian hesapla
        J = jacobian(theta1, theta2)

        # Jacobian'ın tersini al (pseudo-inverse kullan)
        try:
            J_inv = np.linalg.pinv(J)  # Pseudo-inverse (daha güvenilir)
        except:
            print("  ⚠ Jacobian tersi alınamadı (singular matrix)")
            return theta1, theta2, iteration, errors

        # Açı değişimini hesapla
        delta_theta = J_inv @ error

        # Açıları güncelle (öğrenme katsayısı ekle - daha stabil)
        alpha = 0.5  # Öğrenme katsayısı (0-1 arası)
        theta1 += alpha * delta_theta[0]
        theta2 += alpha * delta_theta[1]

    # Maksimum iterasyona ulaşıldı
    return theta1, theta2, max_iter, errors

# =====================================================
# TEST 1: Basit IK Testi
# =====================================================
print("="*60)
print("TEST 1: Jacobian IK - İlk Deneme")
print("="*60)

target = (1.2, 0.8)
print(f"\nHedef pozisyon: ({target[0]}, {target[1]})")
print(f"Başlangıç açıları: θ₁=0°, θ₂=0°")

print("\nÇözüm aranıyor...")
theta1_sol, theta2_sol, iters, errors = inverse_kinematics_jacobian(
    target[0], target[1],
    theta1_init=0,
    theta2_init=0
)

print(f"\n✓ Çözüm bulundu!")
print(f"  İterasyon sayısı: {iters}")
print(f"  Bulunan açılar:")
print(f"    θ₁ = {math.degrees(theta1_sol):6.2f}°")
print(f"    θ₂ = {math.degrees(theta2_sol):6.2f}°")

# Doğrulama
x_final, y_final = forward_kinematics(theta1_sol, theta2_sol)
print(f"  Ulaşılan pozisyon: ({x_final:.4f}, {y_final:.4f})")
print(f"  Hedef pozisyon   : ({target[0]:.4f}, {target[1]:.4f})")
print(f"  Hata: {np.linalg.norm([target[0]-x_final, target[1]-y_final])*1000:.2f} mm")

# Hata grafiği
plt.figure(figsize=(10, 6))
plt.plot(errors, 'b-', linewidth=2)
plt.xlabel('İterasyon', fontsize=12)
plt.ylabel('Hata (metre)', fontsize=12)
plt.title('IK Yakınsama - Hata vs İterasyon', fontsize=14, fontweight='bold')
plt.grid(True, alpha=0.3)
plt.yscale('log')  # Logaritmik ölçek (hata hızla azalır)
plt.tight_layout()
plt.show()

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# TEST 2: Farklı Başlangıç Noktaları
# =====================================================
print("="*60)
print("TEST 2: Farklı Başlangıç Açıları")
print("="*60)

print("""
IK çözümü bazen BAŞLANGIÇ AÇILARINA bağlıdır!
Aynı hedef, farklı başlangıçlarla farklı çözümlere ulaşabilir.
""")

target = (1.0, 1.0)
baslangiclar = [
    (0, 0),
    (math.radians(45), math.radians(45)),
    (math.radians(-30), math.radians(60))
]

print(f"\nHedef: ({target[0]}, {target[1]})")
print("\nFarklı başlangıçlar:")

cozumler = []

for idx, (t1_init, t2_init) in enumerate(baslangiclar, 1):
    print(f"\n{idx}. Başlangıç: θ₁={math.degrees(t1_init):.0f}°, θ₂={math.degrees(t2_init):.0f}°")

    t1, t2, iters, errs = inverse_kinematics_jacobian(
        target[0], target[1],
        theta1_init=t1_init,
        theta2_init=t2_init
    )

    print(f"   Çözüm: θ₁={math.degrees(t1):.1f}°, θ₂={math.degrees(t2):.1f}° ({iters} iterasyon)")

    # Doğrulama
    x, y = forward_kinematics(t1, t2)
    hata = np.linalg.norm([target[0]-x, target[1]-y])
    print(f"   Hata: {hata*1000:.2f} mm")

    cozumler.append((t1, t2, errs))

# Görselleştirme
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

# Sol: Robot konfigürasyonları
ax1.set_xlim(-2, 2)
ax1.set_ylim(-2, 2)
ax1.set_aspect('equal')
ax1.grid(True, alpha=0.3)
ax1.set_xlabel('X (m)', fontsize=12)
ax1.set_ylabel('Y (m)', fontsize=12)
ax1.set_title('Farklı Çözümler (Aynı Hedef)', fontsize=14, fontweight='bold')

renkler = ['red', 'blue', 'green']

for idx, (t1, t2, _) in enumerate(cozumler):
    # Robot çiz
    x0, y0 = 0, 0
    x1 = L1 * math.cos(t1)
    y1 = L1 * math.sin(t1)
    x2, y2 = forward_kinematics(t1, t2)

    ax1.plot([x0, x1], [y0, y1], color=renkler[idx], linewidth=3, label=f'Çözüm {idx+1}')
    ax1.plot([x1, x2], [y1, y2], color=renkler[idx], linewidth=3)
    ax1.plot(x2, y2, 'o', color=renkler[idx], markersize=10)

# Hedef
ax1.plot(target[0], target[1], 'k*', markersize=20, label='Hedef')
ax1.legend(fontsize=10)

# Sağ: Yakınsama grafikleri
for idx, (_, _, errs) in enumerate(cozumler):
    ax2.plot(errs, color=renkler[idx], linewidth=2, label=f'Çözüm {idx+1}')

ax2.set_xlabel('İterasyon', fontsize=12)
ax2.set_ylabel('Hata (m)', fontsize=12)
ax2.set_title('Yakınsama Hızları', fontsize=14, fontweight='bold')
ax2.set_yscale('log')
ax2.grid(True, alpha=0.3)
ax2.legend(fontsize=10)

plt.tight_layout()
plt.show()

print("\nGözlem:")
print("  ✓ Aynı hedefe farklı yollarla ulaşıldı")
print("  ✓ Her çözüm geçerli (hepsi hedefe ulaşıyor)")
print("  ✓ Bazı başlangıçlar daha hızlı yakınsıyor")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# TEST 3: Ulaşılamayan Hedef
# =====================================================
print("="*60)
print("TEST 3: Ulaşılamayan Hedef")
print("="*60)

uzak_hedef = (2.5, 2.5)  # Çok uzak!
print(f"\nÇok uzak hedef: ({uzak_hedef[0]}, {uzak_hedef[1]})")
print(f"  (Robot maksimum {L1+L2}m = {L1+L2}m ulaşabilir)")

t1, t2, iters, errs = inverse_kinematics_jacobian(
    uzak_hedef[0], uzak_hedef[1],
    max_iter=50
)

x_final, y_final = forward_kinematics(t1, t2)
mesafe = np.sqrt(x_final**2 + y_final**2)

print(f"\n✓ Algoritma durdu (maksimum iterasyon)")
print(f"  En yakın pozisyon: ({x_final:.3f}, {y_final:.3f})")
print(f"  Hedef: ({uzak_hedef[0]}, {uzak_hedef[1]})")
print(f"  Ulaşılan mesafe: {mesafe:.3f}m")
print(f"  Hata: {errs[-1]:.3f}m = {errs[-1]*100:.1f}cm")
print("\n  Robot mümkün olan en yakın noktaya gitti!")

# =====================================================
# ÖZET
# =====================================================
print("\n" + "="*60)
print("DERS 9 ÖZET")
print("="*60)
print("""
ÖĞRENDİKLERİMİZ:
✓ Jacobian matrisi nedir
✓ Jacobian nasıl hesaplanır (türev)
✓ Jacobian ile hız ilişkisi
✓ Jacobian Inverse IK algoritması
✓ İteratif çözüm yöntemi
✓ Farklı başlangıç noktaları farklı çözümler verir

JACOBIAN FORMÜLÜ (2-Link):
  J = [∂x/∂θ₁  ∂x/∂θ₂]
      [∂y/∂θ₁  ∂y/∂θ₂]

IK ALGORİTMASI:
  1. Başlangıç açıları
  2. FK ile pozisyon hesapla
  3. Hata = Hedef - Mevcut
  4. Δθ = J⁻¹ × Hata
  5. θ'yi güncelle
  6. Tekrarla (hata < tolerans olana kadar)

AVANTAJLAR:
  ✓ Genel (her robot için)
  ✓ Gerçek zamanlı
  ✓ Kısmi hareketler (velocity control)

DEZAVANTAJLAR:
  ✗ Singularity'lerde problem (J tersi alınamaz)
  ✗ Local minimum'a takılabilir
  ✗ Başlangıç açılarına hassas

İLERİ OKUMA:
  - Damped Least Squares (DLS) - Singularity çözümü
  - Weighted Jacobian - Eklem limitleri
  - Task Priority - Çoklu hedef
""")

print("\n" + "="*60)
print("ROBOT YAZILIM EĞİTİMİ TAMAMLANDI!")
print("="*60)
print("""
🎉 TEBRİKLER! Şunları öğrendiniz:

1. ✅ PyBullet ile simülasyon
2. ✅ Forward Kinematics (açı → pozisyon)
3. ✅ Inverse Kinematics (pozisyon → açı)
4. ✅ Trajectory Planning (yumuşak hareket)
5. ✅ Collision Avoidance (engel kaçınma)
6. ✅ Jacobian matrisi (matematik)
7. ✅ İleri seviye IK algoritması

SONRAKI ADIMLAR:
  → ROS 2 öğrenin (gerçek robotlar)
  → Gerçek robot kol edinin (UR5, Kuka, veya DIY)
  → Computer Vision ekleyin (kamera + robot)
  → Machine Learning (pekiştirmeli öğrenme)

Başarılar! 🚀🤖
""")

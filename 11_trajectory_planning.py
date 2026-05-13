"""
DERS 7: Trajectory Planning (Yörünge Planlama)
Robotun A noktasından B noktasına DÜZGÜN hareket etmesi
"""

import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import matplotlib.pyplot as plt

print("="*60)
print("DERS 7: TRAJECTORY PLANNING")
print("="*60)

# =====================================================
# BÖLÜM 1: Problem Tanımı
# =====================================================
print("\nBÖLÜM 1: Neden Trajectory Planning?")
print("-"*60)
print("""
Senaryo: Robot eli (0, 0.3, 0.5) noktasından (0.4, 0.2, 0.7) noktasına gitsin.

YANLIŞ YÖNTEM: Ani sıçrama
  t=0.0s: (0.0, 0.3, 0.5)
  t=0.1s: (0.4, 0.2, 0.7)  ← ZIPLAAA!

  Sorunlar:
  ✗ Titreşim
  ✗ Motor aşırı yüklenir
  ✗ Objelere çarpma riski
  ✗ Profesyonel değil

DOĞRU YÖNTEM: Yumuşak geçiş
  t=0.0s: (0.00, 0.30, 0.50)
  t=0.5s: (0.10, 0.28, 0.55)
  t=1.0s: (0.20, 0.26, 0.60)
  t=1.5s: (0.30, 0.24, 0.65)
  t=2.0s: (0.40, 0.20, 0.70) ✓

  Avantajlar:
  ✓ Düzgün hareket
  ✓ Güvenli
  ✓ Hesaplanabilir
""")

input("Devam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 2: Linear Interpolation (Doğrusal Enterpolasyon)
# =====================================================
print("="*60)
print("BÖLÜM 2: Linear Interpolation (En Basit Yöntem)")
print("="*60)

def linear_interpolation(baslangic, bitis, t):
    """
    İki nokta arasında doğrusal enterpolasyon

    Parametreler:
        baslangic: Başlangıç değeri
        bitis: Bitiş değeri
        t: Zaman (0.0 = başlangıç, 1.0 = bitiş)

    Döndürür:
        Ara değer
    """
    return baslangic + (bitis - baslangic) * t

print("\nFormül: p(t) = p_start + (p_end - p_start) * t")
print("  t = 0.0 → Başlangıç")
print("  t = 0.5 → Orta nokta")
print("  t = 1.0 → Bitiş")

# Örnek
print("\nÖrnek: 10'dan 50'ye gitme")
for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
    deger = linear_interpolation(10, 50, t)
    print(f"  t={t:.2f} → {deger:.1f}")

input("\nDevam için ENTER'a basın...\n")

# Grafik çiz
t_dizi = np.linspace(0, 1, 100)
pozisyon = [linear_interpolation(10, 50, t) for t in t_dizi]
hiz = [50 - 10 for _ in t_dizi]  # Sabit hız (türev)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

ax1.plot(t_dizi, pozisyon, 'b-', linewidth=2)
ax1.set_ylabel('Pozisyon', fontsize=12)
ax1.set_title('Linear Interpolation', fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3)

ax2.plot(t_dizi, hiz, 'r-', linewidth=2)
ax2.set_xlabel('Zaman (t)', fontsize=12)
ax2.set_ylabel('Hız', fontsize=12)
ax2.set_title('Hız Profili (Sabit)', fontsize=12)
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

print("\nGözlem:")
print("  ✓ Pozisyon: Düz çizgi (doğrusal)")
print("  ✗ Hız: Sabit (ani başlangıç ve duruş!)")

input("Devam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 3: Cubic Interpolation (Kübik Enterpolasyon)
# =====================================================
print("="*60)
print("BÖLÜM 3: Cubic Interpolation (Daha İyi!)")
print("="*60)

def cubic_interpolation(baslangic, bitis, t):
    """
    Kübik enterpolasyon (yumuşak başlangıç ve bitiş)

    Formül: p(t) = p_start + (p_end - p_start) * (3t² - 2t³)
    """
    # Yumuşak geçiş fonksiyonu
    s = 3 * t**2 - 2 * t**3
    return baslangic + (bitis - baslangic) * s

print("\nFormül: p(t) = p_start + (p_end - p_start) * (3t² - 2t³)")
print("  Bu formül hızı yumuşatır:")
print("  - Başlangıçta yavaş başlar")
print("  - Ortada hızlanır")
print("  - Sonda yavaşlar\n")

# Örnek
print("Örnek: 10'dan 50'ye gitme (Cubic)")
for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
    deger = cubic_interpolation(10, 50, t)
    print(f"  t={t:.2f} → {deger:.1f}")

input("\nDevam için ENTER'a basın...\n")

# Grafik karşılaştırma
t_dizi = np.linspace(0, 1, 100)

linear_pos = [linear_interpolation(10, 50, t) for t in t_dizi]
cubic_pos = [cubic_interpolation(10, 50, t) for t in t_dizi]

# Hız (türev) hesapla
linear_vel = np.gradient(linear_pos, t_dizi)
cubic_vel = np.gradient(cubic_pos, t_dizi)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

ax1.plot(t_dizi, linear_pos, 'b-', linewidth=2, label='Linear')
ax1.plot(t_dizi, cubic_pos, 'r-', linewidth=2, label='Cubic')
ax1.set_ylabel('Pozisyon', fontsize=12)
ax1.set_title('Linear vs Cubic Interpolation', fontsize=14, fontweight='bold')
ax1.legend()
ax1.grid(True, alpha=0.3)

ax2.plot(t_dizi, linear_vel, 'b-', linewidth=2, label='Linear Hız')
ax2.plot(t_dizi, cubic_vel, 'r-', linewidth=2, label='Cubic Hız')
ax2.set_xlabel('Zaman (t)', fontsize=12)
ax2.set_ylabel('Hız', fontsize=12)
ax2.set_title('Hız Profilleri', fontsize=12)
ax2.legend()
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

print("\nKarşılaştırma:")
print("  Linear : Ani başlar/durur (hız sabitse çizgi düz)")
print("  Cubic  : Yumuşak başlar/durur (hız kademeli değişir)")
print("  ✓ Cubic daha profesyonel!")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 4: PyBullet'te Uygulama
# =====================================================
print("="*60)
print("BÖLÜM 4: Gerçek Robot ile Test")
print("="*60)

# Simülasyon başlat
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
print("✓ Simülasyon başladı")

robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print("✓ Robot yüklendi")

p.resetDebugVisualizerCamera(
    cameraDistance=2.0,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.5]
)

def eklem_hareket_ettir(eklem_no, aci):
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=eklem_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=aci,
        force=200,
        maxVelocity=3.0
    )

def simule_et(adim):
    for _ in range(adim):
        p.stepSimulation()
        time.sleep(1./240.)

def ik_hesapla(x, y, z):
    """IK ile eklem açılarını hesapla"""
    return p.calculateInverseKinematics(robot, 6, [x, y, z])

# Marker
marker = None
def marker_goster(x, y, z, renk=[0, 1, 0, 0.8]):
    global marker
    if marker is not None:
        try:
            p.removeBody(marker)
        except:
            pass
    gorsel = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=renk)
    marker = p.createMultiBody(0, baseVisualShapeIndex=gorsel, basePosition=[x, y, z])

# =====================================================
# TEST 1: Linear Trajectory
# =====================================================
print("\n" + "="*60)
print("TEST 1: Linear Trajectory")
print("="*60)

baslangic_pos = np.array([0.3, 0.3, 0.5])
bitis_pos = np.array([0.5, -0.2, 0.7])

print(f"\nBaşlangıç: {baslangic_pos}")
print(f"Bitiş    : {bitis_pos}")
print("\n→ Robot LINEAR hareket edecek (3 saniye)")

# Başlangıç pozisyonuna git
eklem_acilari = ik_hesapla(*baslangic_pos)
for i in range(7):
    eklem_hareket_ettir(i, eklem_acilari[i])
simule_et(300)

print("  Hareket başlıyor...")
sure = 3.0  # saniye
adim_sayisi = int(sure * 240)  # 240 Hz

for adim in range(adim_sayisi):
    t = adim / adim_sayisi  # 0.0 → 1.0

    # Linear interpolation
    hedef = baslangic_pos + (bitis_pos - baslangic_pos) * t

    # IK ve hareket
    marker_goster(*hedef, renk=[1, 0, 0, 0.8])
    eklem_acilari = ik_hesapla(*hedef)
    for i in range(7):
        eklem_hareket_ettir(i, eklem_acilari[i])

    p.stepSimulation()
    time.sleep(1./240.)

print("  ✓ Linear hareket tamamlandı")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# TEST 2: Cubic Trajectory
# =====================================================
print("="*60)
print("TEST 2: Cubic Trajectory (Daha Yumuşak)")
print("="*60)

# Aynı başlangıç ve bitiş
print(f"\nBaşlangıç: {baslangic_pos}")
print(f"Bitiş    : {bitis_pos}")
print("\n→ Robot CUBIC hareket edecek (3 saniye)")

# Başlangıca dön
eklem_acilari = ik_hesapla(*baslangic_pos)
for i in range(7):
    eklem_hareket_ettir(i, eklem_acilari[i])
simule_et(300)

print("  Hareket başlıyor...")

for adim in range(adim_sayisi):
    t = adim / adim_sayisi  # 0.0 → 1.0

    # Cubic interpolation
    s = 3 * t**2 - 2 * t**3  # Yumuşak geçiş
    hedef = baslangic_pos + (bitis_pos - baslangic_pos) * s

    # IK ve hareket
    marker_goster(*hedef, renk=[0, 1, 0, 0.8])
    eklem_acilari = ik_hesapla(*hedef)
    for i in range(7):
        eklem_hareket_ettir(i, eklem_acilari[i])

    p.stepSimulation()
    time.sleep(1./240.)

print("  ✓ Cubic hareket tamamlandı")
print("\nFark gördünüz mü?")
print("  Linear: Keskin başlangıç/bitiş")
print("  Cubic : Yumuşak başlangıç/bitiş")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# TEST 3: Çoklu Nokta (Waypoints)
# =====================================================
print("="*60)
print("TEST 3: Waypoints (Ara Noktalardan Geçme)")
print("="*60)

waypoints = [
    np.array([0.4, 0.2, 0.5]),   # Nokta 1
    np.array([0.5, -0.1, 0.7]),  # Nokta 2
    np.array([0.3, -0.3, 0.6]),  # Nokta 3
    np.array([0.2, 0.0, 0.4])    # Nokta 4
]

print(f"\n{len(waypoints)} ara noktadan geçecek:")
for idx, wp in enumerate(waypoints, 1):
    print(f"  Nokta {idx}: {wp}")

# Noktaları göster (kalıcı küreçikler)
for wp in waypoints:
    gorsel = p.createVisualShape(p.GEOM_SPHERE, radius=0.04, rgbaColor=[1, 1, 0, 0.7])
    p.createMultiBody(0, baseVisualShapeIndex=gorsel, basePosition=wp.tolist())

print("\n→ Robot tüm noktalardan geçecek...")

for idx in range(len(waypoints) - 1):
    baslangic = waypoints[idx]
    bitis = waypoints[idx + 1]

    print(f"  Segment {idx+1}: {baslangic} → {bitis}")

    segment_sure = 2.0
    segment_adim = int(segment_sure * 240)

    for adim in range(segment_adim):
        t = adim / segment_adim
        s = 3 * t**2 - 2 * t**3  # Cubic
        hedef = baslangic + (bitis - baslangic) * s

        marker_goster(*hedef, renk=[0, 0, 1, 0.8])
        eklem_acilari = ik_hesapla(*hedef)
        for i in range(7):
            eklem_hareket_ettir(i, eklem_acilari[i])

        p.stepSimulation()
        time.sleep(1./240.)

print("\n✓ Tüm waypoint'lerden geçildi!")

# =====================================================
# ÖZET
# =====================================================
print("\n" + "="*60)
print("DERS 7 ÖZET")
print("="*60)
print("""
ÖĞRENDİKLERİMİZ:
✓ Trajectory Planning nedir
✓ Linear Interpolation (basit ama keskin)
✓ Cubic Interpolation (yumuşak, profesyonel)
✓ Waypoints (ara noktalardan geçme)

FORMÜLLER:
  Linear: p(t) = p₀ + (p₁ - p₀) * t
  Cubic : p(t) = p₀ + (p₁ - p₀) * (3t² - 2t³)

GERÇEK DÜNYA:
  - Endüstriyel robotlar hep trajectory planning kullanır
  - Pick-and-place uygulamaları
  - Kaynak, boyama, montaj

SONRAKI DERS:
→ Collision Avoidance (Çarpışma Önleme)
  Engellerden kaçarak hedefe gitme!
""")

p.disconnect()
print("\n✓ PyBullet kapatıldı")

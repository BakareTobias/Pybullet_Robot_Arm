"""
DERS 5: Forward Kinematics Matematiği
2-Link robot için kendi FK fonksiyonumuzu yazacağız
"""

import pybullet as p
import pybullet_data
import time
import math
import matplotlib.pyplot as plt

# =====================================================
# BÖLÜM 1: MATEMATİK - Forward Kinematics
# =====================================================
print("="*60)
print("BÖLÜM 1: Forward Kinematics Matematiği")
print("="*60)

print("""
2-Link Planar Robot (Düzlemde):

        ● El (x2, y2)
        |
   L2   | Link 2
        |
        ● Eklem 2 (x1, y1)
        |
   L1   | Link 1
        |
        ● Eklem 1 (0, 0)

Eklem 1: θ1 derece döndü (Z ekseni etrafında)
Eklem 2: θ2 derece döndü (Z ekseni etrafında)
""")

# Robot parametreleri
L1 = 1.0  # Link 1 uzunluğu (metre)
L2 = 0.8  # Link 2 uzunluğu (metre)

print(f"Robot Parametreleri:")
print(f"  L1 (Link 1 uzunluğu) = {L1} metre")
print(f"  L2 (Link 2 uzunluğu) = {L2} metre")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# Forward Kinematics Fonksiyonu
# =====================================================
def forward_kinematics(theta1, theta2, L1, L2):
    """
    2-Link robot için Forward Kinematics

    Parametreler:
        theta1: Eklem 1 açısı (radyan)
        theta2: Eklem 2 açısı (radyan)
        L1: Link 1 uzunluğu
        L2: Link 2 uzunluğu

    Döndürür:
        (x, y): El pozisyonu
    """
    # Eklem 2'nin pozisyonu (Link 1'in ucu)
    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)

    # El pozisyonu (Link 2'nin ucu)
    # Link 2, theta1 + theta2 açısında
    x2 = x1 + L2 * math.cos(theta1 + theta2)
    y2 = y1 + L2 * math.sin(theta1 + theta2)

    return (x2, y2)

print("="*60)
print("Forward Kinematics Fonksiyonu Tanımlandı!")
print("="*60)
print("""
Matematik:
  Eklem 2 pozisyonu:
    x1 = L1 * cos(θ1)
    y1 = L1 * sin(θ1)

  El pozisyonu:
    x2 = x1 + L2 * cos(θ1 + θ2)
    y2 = y1 + L2 * sin(θ1 + θ2)
""")

input("Devam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 2: TEST - Farklı Açılar
# =====================================================
print("="*60)
print("BÖLÜM 2: Farklı Açılarla Test")
print("="*60)

test_acilar = [
    (0, 0, "Her iki eklem 0°"),
    (90, 0, "Eklem 1: 90°, Eklem 2: 0°"),
    (45, 45, "Her iki eklem 45°"),
    (90, -90, "Eklem 1: 90°, Eklem 2: -90°"),
    (0, 90, "Eklem 1: 0°, Eklem 2: 90°")
]

for theta1_derece, theta2_derece, aciklama in test_acilar:
    # Dereceyi radyana çevir
    theta1 = math.radians(theta1_derece)
    theta2 = math.radians(theta2_derece)

    # Forward Kinematics hesapla
    x, y = forward_kinematics(theta1, theta2, L1, L2)

    print(f"\n{aciklama}")
    print(f"  Girdi : θ1={theta1_derece}°, θ2={theta2_derece}°")
    print(f"  Çıktı : x={x:.3f}m, y={y:.3f}m")

    # Tabana uzaklık
    uzaklik = math.sqrt(x**2 + y**2)
    print(f"  Tabana uzaklık: {uzaklik:.3f}m")

input("\n\nDevam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 3: GÖRSELLEŞTİRME
# =====================================================
print("="*60)
print("BÖLÜM 3: Matplotlib ile Görselleştirme")
print("="*60)

def robot_ciz(theta1, theta2, L1, L2, baslik=""):
    """
    Robotu 2D düzlemde çizer
    """
    # Eklem pozisyonları
    x0, y0 = 0, 0  # Taban
    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)
    x2, y2 = forward_kinematics(theta1, theta2, L1, L2)

    # Çizim
    plt.figure(figsize=(8, 8))

    # Link 1
    plt.plot([x0, x1], [y0, y1], 'r-', linewidth=8, label='Link 1')

    # Link 2
    plt.plot([x1, x2], [y1, y2], 'b-', linewidth=8, label='Link 2')

    # Eklemler
    plt.plot(x0, y0, 'ko', markersize=15, label='Taban')
    plt.plot(x1, y1, 'go', markersize=12, label='Eklem 2')
    plt.plot(x2, y2, 'mo', markersize=15, label='El')

    # Grid ve ayarlar
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.xlim(-2, 2)
    plt.ylim(-2, 2)
    plt.xlabel('X (metre)', fontsize=12)
    plt.ylabel('Y (metre)', fontsize=12)
    plt.title(baslik, fontsize=14, fontweight='bold')
    plt.legend(fontsize=10)

    # Eksen çizgileri
    plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    plt.axvline(x=0, color='k', linestyle='--', alpha=0.3)

    plt.tight_layout()
    plt.show()

print("Örnek 1: Her iki eklem 0° (Düz yukarı)")
robot_ciz(0, 0, L1, L2, "θ1=0°, θ2=0°")

print("\nÖrnek 2: Eklem 1: 45°, Eklem 2: 0°")
robot_ciz(math.radians(45), 0, L1, L2, "θ1=45°, θ2=0°")

print("\nÖrnek 3: Eklem 1: 90°, Eklem 2: -45°")
robot_ciz(math.radians(90), math.radians(-45), L1, L2, "θ1=90°, θ2=-45°")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 4: ÇALIŞMA ALANI (Workspace)
# =====================================================
print("="*60)
print("BÖLÜM 4: Workspace (Çalışma Alanı)")
print("="*60)
print("Robot elinin ulaşabileceği TÜM noktaları çizeceğiz")
print("Bu birkaç saniye sürebilir...\n")

# Çok sayıda açı kombinasyonu dene
x_noktalar = []
y_noktalar = []

for theta1_derece in range(0, 360, 5):  # 0° ile 360° arası, 5° adımlarla
    for theta2_derece in range(-180, 180, 5):  # -180° ile 180° arası
        theta1 = math.radians(theta1_derece)
        theta2 = math.radians(theta2_derece)

        x, y = forward_kinematics(theta1, theta2, L1, L2)
        x_noktalar.append(x)
        y_noktalar.append(y)

# Çizim
plt.figure(figsize=(10, 10))
plt.scatter(x_noktalar, y_noktalar, c='blue', s=1, alpha=0.5)
plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.xlabel('X (metre)', fontsize=12)
plt.ylabel('Y (metre)', fontsize=12)
plt.title('Robot Workspace (Çalışma Alanı)', fontsize=14, fontweight='bold')
plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
plt.axvline(x=0, color='k', linestyle='--', alpha=0.3)

# Min/Max ulaşım
max_uzaklik = L1 + L2
min_uzaklik = abs(L1 - L2)
print(f"✓ Workspace hesaplandı!")
print(f"  Maksimum ulaşım: {max_uzaklik:.3f}m (kollar düz)")
print(f"  Minimum ulaşım: {min_uzaklik:.3f}m (kollar üst üste)")

plt.tight_layout()
plt.show()

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 5: PYBULLET ile KARŞILAŞTIRMA
# =====================================================
print("="*60)
print("BÖLÜM 5: PyBullet ile Doğrulama")
print("="*60)
print("Kendi FK fonksiyonumuz PyBullet'inkiyle aynı mı?\n")

# Simülasyon başlat
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
print("✓ PyBullet başlatıldı")

# Kuka robot yükle (ilk 2 eklemini kullanacağız)
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print("✓ Kuka robot yüklendi")

def eklem_hareket_ettir(eklem_no, aci):
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=eklem_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=aci
    )

def simule_et(adim):
    for _ in range(adim):
        p.stepSimulation()
        time.sleep(1./240.)

# Test açıları
test_kombinasyonlar = [
    (0, 0),
    (math.radians(30), math.radians(45)),
    (math.radians(90), math.radians(-30))
]

print("\nKarşılaştırma:")
for theta1, theta2 in test_kombinasyonlar:
    # Bizim hesaplamamız
    x_bizim, y_bizim = forward_kinematics(theta1, theta2, L1, L2)

    # PyBullet'e aynı açıları ver
    eklem_hareket_ettir(0, theta1)
    eklem_hareket_ettir(1, theta2)
    simule_et(300)

    # PyBullet'in hesabı
    durum = p.getLinkState(robot, 6)
    x_pybullet, y_pybullet, z_pybullet = durum[0]

    print(f"\nθ1={math.degrees(theta1):.0f}°, θ2={math.degrees(theta2):.0f}°")
    print(f"  Bizim hesap    : x={x_bizim:.3f}, y={y_bizim:.3f}")
    print(f"  PyBullet       : x={x_pybullet:.3f}, y={y_pybullet:.3f}, z={z_pybullet:.3f}")
    print(f"  Not: PyBullet 3D, bizimki 2D. X ve Y'yi karşılaştırıyoruz.")

print("\n" + "="*60)
print("DERS 5 TAMAMLANDI!")
print("="*60)
print("""
ÖĞRENDİKLERİMİZ:
✓ Forward Kinematics formülü
✓ Kendi FK fonksiyonumuzu yazdık
✓ Matplotlib ile görselleştirdik
✓ Workspace kavramını anladık
✓ PyBullet ile doğruladık

SONRAKI DERS:
→ Inverse Kinematics (Ters Kinematik)
  "El şu noktada olsun" → Eklem açılarını hesapla
""")

p.disconnect()

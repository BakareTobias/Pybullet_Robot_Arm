"""
DERS 6: Inverse Kinematics (Ters Kinematik)
"Eli şu noktaya götür" → Eklem açılarını hesapla
"""

import pybullet as p
import pybullet_data
import time
import math

print("="*60)
print("DERS 6: INVERSE KINEMATICS")
print("="*60)

# =====================================================
# BÖLÜM 1: Inverse Kinematics Nedir?
# =====================================================
print("\nBÖLÜM 1: Kavram")
print("-"*60)
print("""
Forward Kinematics (FK):
  Girdi  : θ1, θ2 (eklem açıları)
  Çıktı  : (x, y) el pozisyonu
  Örnek  : θ1=45°, θ2=30° → x=1.2m, y=0.8m

Inverse Kinematics (IK):
  Girdi  : (x, y) hedef pozisyon
  Çıktı  : θ1, θ2 (gerekli eklem açıları)
  Örnek  : x=1.2m, y=0.8m → θ1=?, θ2=?

Problem: Matematik daha zor!
  - Bazen birden fazla çözüm var
  - Bazen hiç çözüm yok (ulaşılamaz nokta)
""")

input("Devam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 2: PyBullet'in IK'sini Kullanma
# =====================================================
print("="*60)
print("BÖLÜM 2: PyBullet'in Otomatik IK'si")
print("="*60)

# Simülasyon başlat
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
print("✓ Simülasyon başladı")

# Robot yükle
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print("✓ Kuka robot yüklendi")

# Zemin ekle
p.loadURDF("plane.urdf")

# Kamerayı ayarla
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
        maxVelocity=2.0
    )

def simule_et(adim):
    for _ in range(adim):
        p.stepSimulation()
        time.sleep(1./240.)

# Hedef göstermek için marker
marker = None
def hedef_goster(x, y, z, renk=[1, 0, 0, 0.8]):
    """Hedef pozisyonu kırmızı küre ile göster"""
    global marker
    if marker is not None:
        try:
            p.removeBody(marker)
        except:
            pass

    gorsel = p.createVisualShape(p.GEOM_SPHERE, radius=0.08, rgbaColor=renk)
    marker = p.createMultiBody(0, baseVisualShapeIndex=gorsel, basePosition=[x, y, z])

# =====================================================
# PyBullet IK Fonksiyonu
# =====================================================
print("\nPyBullet'in calculateInverseKinematics() fonksiyonu:")
print("  Hedef pozisyon verirsiniz → Tüm eklem açılarını hesaplar\n")

def pybullet_ik(hedef_x, hedef_y, hedef_z):
    """
    PyBullet'in IK çözücüsünü kullan

    Parametreler:
        hedef_x, hedef_y, hedef_z: Hedef pozisyon

    Döndürür:
        eklem_acilari: 7 eklem için açılar (tuple)
    """
    # End-effector = Link 6
    end_effector_index = 6

    # IK hesapla
    eklem_acilari = p.calculateInverseKinematics(
        bodyUniqueId=robot,
        endEffectorLinkIndex=end_effector_index,
        targetPosition=[hedef_x, hedef_y, hedef_z]
    )

    return eklem_acilari

# =====================================================
# TEST 1: İlk IK Testi
# =====================================================
print("="*60)
print("TEST 1: İlk IK Hesaplama")
print("="*60)

hedef = [0.4, 0.3, 0.5]  # (x, y, z)
print(f"\nHedef pozisyon: x={hedef[0]}, y={hedef[1]}, z={hedef[2]}")

# Hedefi göster (kırmızı küre)
hedef_goster(hedef[0], hedef[1], hedef[2])

print("  → IK hesaplanıyor...")
eklem_acilari = pybullet_ik(hedef[0], hedef[1], hedef[2])

print(f"  ✓ Hesaplanan eklem açıları (derece):")
for i in range(7):
    print(f"    Eklem {i}: {math.degrees(eklem_acilari[i]):6.1f}°")

print("\n  → Robotu hesaplanan açılara götürüyorum...")
for i in range(7):
    eklem_hareket_ettir(i, eklem_acilari[i])

simule_et(500)

# Gerçek pozisyonu kontrol et
durum = p.getLinkState(robot, 6)
gercek_pos = durum[0]
print(f"\n  ✓ Robot eli ulaştı!")
print(f"    Hedef    : ({hedef[0]:.3f}, {hedef[1]:.3f}, {hedef[2]:.3f})")
print(f"    Gerçek   : ({gercek_pos[0]:.3f}, {gercek_pos[1]:.3f}, {gercek_pos[2]:.3f})")

# Hata hesapla
hata = math.sqrt(
    (hedef[0]-gercek_pos[0])**2 +
    (hedef[1]-gercek_pos[1])**2 +
    (hedef[2]-gercek_pos[2])**2
)
print(f"    Hata     : {hata*1000:.1f} mm")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# TEST 2: Birden Fazla Hedef
# =====================================================
print("="*60)
print("TEST 2: Robot 5 Farklı Noktaya Gidecek")
print("="*60)

hedefler = [
    ([0.5, 0.0, 0.5], [1, 0, 0, 0.8]),   # Kırmızı
    ([0.3, 0.3, 0.8], [0, 1, 0, 0.8]),   # Yeşil
    ([0.0, 0.5, 0.4], [0, 0, 1, 0.8]),   # Mavi
    ([0.4, -0.2, 0.6], [1, 1, 0, 0.8]),  # Sarı
    ([0.2, 0.2, 0.3], [1, 0, 1, 0.8])    # Mor
]

for idx, (hedef, renk) in enumerate(hedefler, 1):
    x, y, z = hedef

    print(f"\nHedef {idx}: ({x:.2f}, {y:.2f}, {z:.2f})")

    # Hedefi göster
    hedef_goster(x, y, z, renk)

    # IK hesapla
    eklem_acilari = pybullet_ik(x, y, z)

    # Robotu hareket ettir
    for i in range(7):
        eklem_hareket_ettir(i, eklem_acilari[i])

    simule_et(400)

    # Doğrulama
    durum = p.getLinkState(robot, 6)
    gercek = durum[0]
    hata = math.sqrt((x-gercek[0])**2 + (y-gercek[1])**2 + (z-gercek[2])**2)
    print(f"  ✓ Ulaştı! Hata: {hata*1000:.1f} mm")

    time.sleep(0.5)

print("\n✓ Tüm hedeflere başarıyla ulaşıldı!")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# TEST 3: Ulaşılamayan Nokta
# =====================================================
print("="*60)
print("TEST 3: Ulaşılamayan Nokta")
print("="*60)

# Çok uzak bir nokta
hedef_uzak = [2.0, 2.0, 2.0]
print(f"\nÇok uzak hedef: ({hedef_uzak[0]}, {hedef_uzak[1]}, {hedef_uzak[2]})")
print("  (Robot'un maksimum erişimi ~1.5m civarı)")

hedef_goster(hedef_uzak[0], hedef_uzak[1], hedef_uzak[2], [1, 0.5, 0, 0.8])

print("\n  → IK hesaplanıyor...")
eklem_acilari = pybullet_ik(hedef_uzak[0], hedef_uzak[1], hedef_uzak[2])

print("  → Robot en yakın pozisyona gidiyor...")
for i in range(7):
    eklem_hareket_ettir(i, eklem_acilari[i])

simule_et(500)

# Gerçek pozisyon
durum = p.getLinkState(robot, 6)
gercek = durum[0]
hata = math.sqrt(
    (hedef_uzak[0]-gercek[0])**2 +
    (hedef_uzak[1]-gercek[1])**2 +
    (hedef_uzak[2]-gercek[2])**2
)

print(f"\n  Hedef    : ({hedef_uzak[0]:.3f}, {hedef_uzak[1]:.3f}, {hedef_uzak[2]:.3f})")
print(f"  Ulaşılan : ({gercek[0]:.3f}, {gercek[1]:.3f}, {gercek[2]:.3f})")
print(f"  Hata     : {hata:.3f} m = {hata*100:.1f} cm")
print("\n  ⚠ Robot hedefe tam ulaşamadı (çok uzak!)")
print("     Ama mümkün olan en yakın noktaya gitti.")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# TEST 4: İnteraktif Mod
# =====================================================
print("="*60)
print("TEST 4: İnteraktif - Fareyle Hedef Belirleme")
print("="*60)
print("""
Şimdi fareyle tıklayarak hedef belirleyebilirsiniz!

Nasıl kullanılır:
  1. PyBullet penceresinde bir yere tıklayın
  2. Robot o noktaya gitmeye çalışacak
  3. CTRL+C ile durdurun

Not: Bu basit bir örnek, mouse_click eventi yok ama
     rastgele hedefler üretip robot göndereceğiz (demo için)
""")

input("İnteraktif moda başlamak için ENTER'a basın...\n")

print("10 saniye boyunca robot rastgele hedeflere gidecek...")
print("(Gerçek uygulamada mouse tıklamaları kullanılır)\n")

import random

baslangic = time.time()
while time.time() - baslangic < 10:
    # Rastgele hedef (workspace içinde)
    x = random.uniform(0.2, 0.6)
    y = random.uniform(-0.3, 0.3)
    z = random.uniform(0.3, 0.8)

    print(f"\rHedef: ({x:.2f}, {y:.2f}, {z:.2f})", end="")

    # IK ve hareket
    hedef_goster(x, y, z)
    eklem_acilari = pybullet_ik(x, y, z)

    for i in range(7):
        eklem_hareket_ettir(i, eklem_acilari[i])

    simule_et(100)

print("\n\n✓ İnteraktif mod tamamlandı!")

# =====================================================
# ÖZET
# =====================================================
print("\n" + "="*60)
print("DERS 6 ÖZET")
print("="*60)
print("""
ÖĞRENDİKLERİMİZ:
✓ Inverse Kinematics kavramı
✓ PyBullet'in IK çözücüsü (calculateInverseKinematics)
✓ Hedef pozisyon → Eklem açıları
✓ Ulaşılabilir/ulaşılamayan noktalar
✓ Hata hesaplama

KULLANIM:
  eklem_acilari = p.calculateInverseKinematics(
      robot,
      end_effector_index,
      [hedef_x, hedef_y, hedef_z]
  )

SONRAKI DERS:
→ Kendi IK algoritmanı yazma (Jacobian yöntemi)
  (İleri seviye, ama merak ediyorsanız devam edelim!)
""")

p.disconnect()
print("\n✓ PyBullet kapatıldı")
